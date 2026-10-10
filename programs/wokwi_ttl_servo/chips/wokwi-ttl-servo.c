#include "wokwi-api.h"
#include <stdlib.h>
#include <string.h>
#include "wokwi-ttl-servo.bg.h"

#define MODEL_NUMBER 1200 /* XL330-M288 */
#define FIRMWARE_VERSION 1
#define MAX_PARAMS 64
#define MAX_READ_LEN 64

// Velocity unit 0.229 rpm per unit: ROBOTIS XL330-M077 e-Manual https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/
// Same constant as lib/dxl_units/dxl_units.h; this chip's MODEL_NUMBER 1200 is XL330-M288 (same unit), hand-copied here.
#define TICKS_PER_REV 4096.0
#define VEL_TICKS_PER_S_PER_UNIT (0.229 * TICKS_PER_REV / 60.0)

#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE  64
#define ADDR_GOAL_VELOCITY  104
#define ADDR_GOAL_POSITION  116
#define ADDR_PRESENT_POSITION 132

#define INST_PING          0x01
#define INST_READ          0x02
#define INST_WRITE         0x03
#define INST_STATUS        0x55

typedef enum {
  STATE_HEADER1,
  STATE_HEADER2,
  STATE_HEADER3,
  STATE_RESERVED,
  STATE_ID,
  STATE_LEN_L,
  STATE_LEN_H,
  STATE_INSTRUCTION,
  STATE_PARAMETERS,
  STATE_CRC_L,
  STATE_CRC_H
} parse_state_t;

typedef struct {
  uart_dev_t uart;
  pin_t data_pin;
  uint8_t servo_id;
  uint32_t current_position;
  double pos_frac; // Sub-tick remainder of integrated position
  int32_t speed;
  uint64_t last_update_us;

  // Half-duplex: TX pin is shared with RX on DATA, so our own reply echoes back
  pin_t tx_pin;
  bool transmitting;
  uint64_t tx_deadline_us;

  // Custom attributes / Operating states
  uint8_t operating_mode;
  uint8_t torque_enable;

  // Display: framebuffer handle and dims reported by the host (never hardcoded)
  buffer_t fb;
  uint32_t fb_w;
  uint32_t fb_h;
  uint32_t last_pos_drawn; // 0xFFFFFFFF = never drawn
  uint8_t last_torque_drawn;
  uint8_t draw_div;        // Toggles every timer tick; redraw on every 2nd tick

  // Parser variables
  parse_state_t state;
  uint16_t packet_length;
  uint8_t rx_id;
  uint8_t instruction;
  uint8_t params[MAX_PARAMS];
  uint16_t param_count;
  uint16_t param_idx;
  uint8_t tx_buf[96];
} chip_state_t;

// Horn display. Pixel format assumed RGBA32 as uint32 0xAABBGGRR (alpha 0xFF, red in low byte).
// CONFIRMED in the Wokwi simulator by the repo owner.
#define HORN_RADIUS 28 // Reference ring radius, pixels
#define HORN_BG        0xFF202020u
#define HORN_RING      0xFF404040u
#define HORN_TICK      0xFF808080u
#define HORN_BODY_ON   0xFFE0E0E0u
#define HORN_BODY_OFF  0xFF707070u
#define HORN_TIP_ON    0xFF0000FFu // Red (0xAABBGGRR, byte order confirmed in the Wokwi simulator)
#define HORN_TIP_OFF   0xFF000080u
#define HORN_MAX_DIM 64
// Background is a baked 64x64 image generated from wokwi-ttl-servo.svg by tools/svg_to_pixels.sh (`make wokwi_ttl_servo-bg`).
// Same 0xAABBGGRR pixel format as above, CONFIRMED in the Wokwi simulator by the repo owner.
#define BG_DIM 64
// Horn geometry in half-pixel units (doubled coordinates)
#define HORN_L 40
#define HORN_W 6
#define HORN_HUB 14
#define HORN_BORE 4
#define HORN_HOLE_R 2
#define HORN_HOLE_U1 24
#define HORN_HOLE_U2 32

static uint32_t pixels[HORN_MAX_DIM * HORN_MAX_DIM]; // Static storage, not stack

// sin(i * 2pi / 64) * 1024, rounded. cos(i) = SIN_1024[(i + 16) & 63].
static const int32_t SIN_1024[64] = {
     0,  100,  200,  297,  392,  483,  569,  650,  724,  792,  851,  903,  946,  980, 1004, 1019,
  1024, 1019, 1004,  980,  946,  903,  851,  792,  724,  650,  569,  483,  392,  297,  200,  100,
     0, -100, -200, -297, -392, -483, -569, -650, -724, -792, -851, -903, -946, -980,-1004,-1019,
 -1024,-1019,-1004, -980, -946, -903, -851, -792, -724, -650, -569, -483, -392, -297, -200, -100
};

// Arm-local coordinates (u along arm, v across). Returns 0 if not horn.
static uint32_t horn_pixel(int32_t u, int32_t v, uint32_t body, uint32_t tip) {
  if (u * u + v * v <= HORN_BORE * HORN_BORE) {
    return HORN_BG;
  }
  int32_t du1 = u - HORN_HOLE_U1;
  int32_t du2 = u - HORN_HOLE_U2;
  if (du1 * du1 + v * v <= HORN_HOLE_R * HORN_HOLE_R || du2 * du2 + v * v <= HORN_HOLE_R * HORN_HOLE_R) {
    return HORN_BG;
  }
  if (u * u + v * v <= HORN_HUB * HORN_HUB) {
    return body;
  }
  int32_t du = (u < 0) ? -u : (u > HORN_L ? u - HORN_L : 0);
  if (du * du + v * v <= HORN_W * HORN_W) {
    return (u > HORN_L - 8) ? tip : body;
  }
  return 0;
}

// Background, reference ring, ticks, servo horn. Position 0 = arm up, clockwise.
static void draw_horn(chip_state_t *chip) {
  uint32_t w = chip->fb_w;
  uint32_t h = chip->fb_h;
  // Handle 0 is valid in Wokwi; validity is judged from the dimensions framebuffer_init reports.
  if (w == 0 || h == 0 || w > HORN_MAX_DIM || h > HORN_MAX_DIM) {
    return;
  }

  uint32_t body = chip->torque_enable ? HORN_BODY_ON : HORN_BODY_OFF;
  uint32_t tip = chip->torque_enable ? HORN_TIP_ON : HORN_TIP_OFF;

  // 4096 ticks per turn -> 64 table steps
  uint32_t idx = (chip->current_position / 64) & 63;
  int32_t sn = SIN_1024[idx];
  int32_t cs = SIN_1024[(idx + 16) & 63];

  // Ring: 2R in half-pixel units, squared, within (R-1)..(R+1)
  int32_t r_in2 = (2 * (HORN_RADIUS - 1)) * (2 * (HORN_RADIUS - 1));
  int32_t r_out2 = (2 * (HORN_RADIUS + 1)) * (2 * (HORN_RADIUS + 1));

  for (int32_t y = 0; y < (int32_t)h; y++) {
    for (int32_t x = 0; x < (int32_t)w; x++) {
      int32_t X = 2 * x + 1 - (int32_t)w;
      int32_t Y = 2 * y + 1 - (int32_t)h;
      int32_t ax = (X < 0) ? -X : X;
      int32_t ay = (Y < 0) ? -Y : Y;
      int32_t d2 = X * X + Y * Y;

      uint32_t c = HORN_BG;
      if (w == BG_DIM && h == BG_DIM) {
        uint32_t b = servo_bg_pixels[y * BG_DIM + x];
        if (b >> 24) {
          c = b;
        }
        if (!chip->torque_enable) {
          c = ((c >> 1) & 0x007F7F7Fu) | 0xFF000000u;
        }
      }
      if (d2 >= r_in2 && d2 <= r_out2) {
        c = HORN_RING;
      }
      if ((ax <= 1 && ay >= 48 && ay <= 56) || (ay <= 1 && ax >= 48 && ax <= 56)) {
        c = HORN_TICK;
      }

      // >> on negative int32 is arithmetic in clang/wasm
      int32_t u = (X * sn - Y * cs) >> 10;
      int32_t v = (X * cs + Y * sn) >> 10;
      uint32_t horn = horn_pixel(u, v, body, tip);
      if (horn != 0) {
        c = horn;
      }
      pixels[y * (int32_t)w + x] = c;
    }
  }

  buffer_write(chip->fb, 0, pixels, w * h * 4);
}

// Dynamixel Protocol 2.0 CRC-16 (poly 0x8005, MSB first, init 0)
static uint16_t calculate_crc(uint16_t crc, const uint8_t *data, uint16_t size) {
  for (uint16_t j = 0; j < size; j++) {
    crc ^= (uint16_t)data[j] << 8;
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x8005) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// Transmits a complete Dynamixel Status Packet back to the host
static void send_status_packet(chip_state_t *chip, uint8_t error, const uint8_t *data, uint16_t data_len) {
  uint16_t packet_len = 4 + data_len; // Inst (1) + Error (1) + Data (N) + CRC (2)
  uint16_t total_tx_size = 7 + packet_len;
  uint8_t *tx_buf = chip->tx_buf;

  if (total_tx_size > sizeof(chip->tx_buf)) {
    return;
  }

  tx_buf[0] = 0xFF;
  tx_buf[1] = 0xFF;
  tx_buf[2] = 0xFD;
  tx_buf[3] = 0x00;
  tx_buf[4] = chip->servo_id;
  tx_buf[5] = packet_len & 0xFF;
  tx_buf[6] = (packet_len >> 8) & 0xFF;
  tx_buf[7] = INST_STATUS;
  tx_buf[8] = error;

  if (data_len > 0 && data != NULL) {
    memcpy(&tx_buf[9], data, data_len);
  }

  uint16_t crc = calculate_crc(0, tx_buf, total_tx_size - 2);
  tx_buf[total_tx_size - 2] = crc & 0xFF;
  tx_buf[total_tx_size - 1] = (crc >> 8) & 0xFF;

  // Previous reply still on the wire: drop this one.
  if (chip->transmitting) {
    return;
  }

  // Drive TX only while the reply is on the wire; release it (INPUT_PULLUP) otherwise.
  uint64_t now = get_sim_nanos() / 1000;
  chip->transmitting = true;
  chip->tx_deadline_us = now + (uint64_t)total_tx_size * 10 * 1000000 / 57600 + 2 * 20000; // Fallback if write_done never fires (+2 timer ticks of margin)
  pin_mode(chip->tx_pin, OUTPUT_HIGH);
  uart_write(chip->uart, tx_buf, total_tx_size);
}

// Reply fully shifted out: release TX and stop dropping RX bytes.
static void chip_uart_write_done(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (!chip->transmitting) {
    return;
  }
  chip->transmitting = false;
  pin_mode(chip->tx_pin, INPUT_PULLUP);
}

static void process_packet(chip_state_t *chip) {
  if (chip->rx_id != chip->servo_id && chip->rx_id != 0xFE) {
    return;
  }

  // Never answer a status packet (our own reply echoed back, or another servo's)
  if (chip->instruction == INST_STATUS) {
    return;
  }

  if (chip->instruction == INST_PING) {
    uint8_t info[3] = { MODEL_NUMBER & 0xFF, MODEL_NUMBER >> 8, FIRMWARE_VERSION };
    send_status_packet(chip, 0, info, 3);
    return;
  }

  if (chip->param_count < 2) {
    return;
  }
  uint16_t target_address = chip->params[0] | (chip->params[1] << 8);

  if (chip->instruction == INST_WRITE) {
    if (chip->param_count < 3) {
      return;
    }
    uint16_t data_len = chip->param_count - 2;

    if (target_address == ADDR_OPERATING_MODE) {
      chip->operating_mode = chip->params[2];
    } else if (target_address == ADDR_TORQUE_ENABLE) {
      chip->torque_enable = chip->params[2];
      if (chip->torque_enable == 0) {
        chip->pos_frac = 0;
      }
    } else if (target_address == ADDR_GOAL_VELOCITY && data_len >= 4) {
      memcpy(&chip->speed, &chip->params[2], 4);
    }
    // Return empty status packet acknowledging write success
    if (chip->rx_id != 0xFE) {
      send_status_packet(chip, 0, NULL, 0);
    }
  }
  else if (chip->instruction == INST_READ) {
    if (chip->param_count < 4) {
      return;
    }
    uint16_t read_length = chip->params[2] | (chip->params[3] << 8);
    if (read_length == 0 || read_length > MAX_READ_LEN) {
      return;
    }

    if (target_address == ADDR_PRESENT_POSITION && read_length == 4) {
      uint8_t buffer[4];
      memcpy(buffer, &chip->current_position, 4);
      send_status_packet(chip, 0, buffer, 4);
    } else if (target_address == ADDR_TORQUE_ENABLE && read_length == 1) {
      send_status_packet(chip, 0, &chip->torque_enable, 1);
    } else {
      // Fallback zeroed array response for unhandled register reads
      uint8_t empty[MAX_READ_LEN] = {0};
      send_status_packet(chip, 0, empty, read_length);
    }
  }
}

void chip_uart_byte_received(void *user_data, uint8_t byte) {
  chip_state_t *chip = (chip_state_t *)user_data;

  // Drop our own reply echoed back on DATA
  if (chip->transmitting) {
    return;
  }

  switch (chip->state) {
    case STATE_HEADER1: chip->state = (byte == 0xFF) ? STATE_HEADER2 : STATE_HEADER1; break;
    case STATE_HEADER2: chip->state = (byte == 0xFF) ? STATE_HEADER3 : STATE_HEADER1; break;
    case STATE_HEADER3: chip->state = (byte == 0xFD) ? STATE_RESERVED : STATE_HEADER1; break;
    case STATE_RESERVED: chip->state = (byte == 0x00) ? STATE_ID : STATE_HEADER1; break;
    case STATE_ID: chip->rx_id = byte; chip->state = STATE_LEN_L; break;
    case STATE_LEN_L: chip->packet_length = byte; chip->state = STATE_LEN_H; break;
    case STATE_LEN_H:
      chip->packet_length |= (byte << 8);
      if (chip->packet_length >= 3 && chip->packet_length < 128) {
        chip->state = STATE_INSTRUCTION;
      } else {
        chip->state = STATE_HEADER1;
      }
      break;
    case STATE_INSTRUCTION:
      chip->instruction = byte;
      chip->param_idx = 0;
      chip->param_count = chip->packet_length - 3;
      if (chip->param_count > MAX_PARAMS) {
        chip->state = STATE_HEADER1;
      } else {
        chip->state = (chip->param_count > 0) ? STATE_PARAMETERS : STATE_CRC_L;
      }
      break;
    case STATE_PARAMETERS:
      chip->params[chip->param_idx++] = byte;
      if (chip->param_idx >= (chip->packet_length - 3)) {
        chip->state = STATE_CRC_L;
      }
      break;
    case STATE_CRC_L: chip->state = STATE_CRC_H; break;
    case STATE_CRC_H:
      process_packet(chip);
      chip->state = STATE_HEADER1;
      break;
    default: chip->state = STATE_HEADER1; break;
  }
}

static void chip_timer_callback(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  uint64_t now = get_sim_nanos() / 1000;
  double delta_t = (now - chip->last_update_us) / 1000000.0;
  chip->last_update_us = now;

  // Fallback release if write_done never fired (timer ticks every 20 ms, so this is late by up to one tick)
  if (chip->transmitting && now >= chip->tx_deadline_us) {
    chip->transmitting = false;
    pin_mode(chip->tx_pin, INPUT_PULLUP);
  }

  // delta_t is in seconds (us / 1e6), so ticks = units * ticks-per-s-per-unit * seconds
  if (chip->torque_enable && chip->speed != 0) {
    chip->pos_frac += chip->speed * VEL_TICKS_PER_S_PER_UNIT * delta_t;
    int32_t whole = (int32_t)chip->pos_frac;
    chip->pos_frac -= whole;
    chip->current_position = (uint32_t)((((int64_t)chip->current_position + whole) % 4096 + 4096) % 4096);
  }

  // Redraw only on position change, and only every 2nd tick (~25 Hz)
  chip->draw_div ^= 1;
  if (chip->draw_div == 0 &&
      (chip->current_position != chip->last_pos_drawn || chip->torque_enable != chip->last_torque_drawn)) {
    draw_horn(chip);
    chip->last_pos_drawn = chip->current_position;
    chip->last_torque_drawn = chip->torque_enable;
  }
}

void chip_init() {
  chip_state_t *chip = calloc(1, sizeof(chip_state_t));
  chip->servo_id = 1;
  chip->current_position = 2048;
  chip->speed = 0;
  chip->operating_mode = 3; // Position mode default
  chip->torque_enable = 0;
  chip->state = STATE_HEADER1;
  chip->param_count = 0;
  chip->last_update_us = get_sim_nanos() / 1000;
  chip->last_pos_drawn = 0xFFFFFFFFu;
  chip->draw_div = 0;

  // Full-duplex: separate RX and TX pins for clean bidirectional communication.
  pin_t rx_pin = pin_init("RX", INPUT_PULLUP);
  pin_t tx_pin = pin_init("TX", INPUT_PULLUP);
  chip->tx_pin = tx_pin;
  chip->transmitting = false;
  chip->tx_deadline_us = 0;
  const uart_config_t uart_config = {
    .tx = tx_pin,
    .rx = rx_pin,
    .baud_rate = 57600,
    .rx_data = chip_uart_byte_received,
    .write_done = chip_uart_write_done,
    .user_data = chip,
  };
  chip->uart = uart_init(&uart_config);
  pin_mode(tx_pin, INPUT_PULLUP); // uart_init may have driven TX high; release it until a reply

  // Framebuffer: dims come from the host; draw_horn skips if they are 0 or > 64
  chip->fb = framebuffer_init(&chip->fb_w, &chip->fb_h);
  draw_horn(chip);
  if (chip->fb_w != 0 && chip->fb_h != 0) {
    chip->last_pos_drawn = chip->current_position;
    chip->last_torque_drawn = chip->torque_enable;
  }

  const timer_config_t timer_config = {
    .callback = chip_timer_callback,
    .user_data = chip,
  };
  timer_t timer = timer_init(&timer_config);
  timer_start(timer, 20000, true);
}
