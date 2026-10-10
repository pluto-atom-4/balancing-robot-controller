#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  pin_t txd;      // From ESP TX
  pin_t rxd;      // To ESP RX
  pin_t data;     // Half-duplex shared bus
  bool txd_drive; // True when we are driving TXD low (waiting for echo)
} adapter_state_t;

static void on_txd_change(void *user_data, pin_t pin, uint32_t value) {
  adapter_state_t *state = (adapter_state_t *)user_data;
  
  // When TXD goes low, drive DATA low (ESP sending). When TXD goes high, release DATA (idle).
  if (value == 0) {
    pin_mode(state->data, OUTPUT_LOW);
    state->txd_drive = true;
  } else {
    pin_mode(state->data, INPUT_PULLUP);
    state->txd_drive = false;
  }
}

static void on_data_change(void *user_data, pin_t pin, uint32_t value) {
  adapter_state_t *state = (adapter_state_t *)user_data;
  
  // Drive RXD to follow DATA, but not if we (the adapter) are currently driving DATA.
  // This suppresses echo: the host's own TX transmissions don't loop back to RX.
  if (!state->txd_drive) {
    if (value == 0) {
      pin_write(state->rxd, 0);
    } else {
      pin_write(state->rxd, 1);
    }
  }
}

void chip_init(void) {
  adapter_state_t *state = calloc(1, sizeof(adapter_state_t));
  
  state->txd = pin_init("TXD", INPUT_PULLUP);
  state->rxd = pin_init("RXD", OUTPUT_HIGH);  // Idle high
  state->data = pin_init("DATA", INPUT_PULLUP);
  state->txd_drive = false;
  
  // Watch TXD: when ESP sends, drive DATA. When ESP idles, release DATA.
  pin_watch_config_t txd_watch = {
    .edge = BOTH,
    .pin_change = on_txd_change,
    .user_data = state,
  };
  pin_watch(state->txd, &txd_watch);
  
  // Watch DATA: when the servo responds, copy to RXD (unless we are sending).
  pin_watch_config_t data_watch = {
    .edge = BOTH,
    .pin_change = on_data_change,
    .user_data = state,
  };
  pin_watch(state->data, &data_watch);
}
