#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  pin_t txd;      // From ESP TX
  pin_t rxd;      // To ESP RX
  pin_t data;     // Half-duplex shared bus
  bool txd_drive; // True when we are driving DATA low (waiting for echo)
  uint32_t txd_edge_count;
  uint32_t data_edge_count;
  uint32_t rxd_write_count;
} adapter_state_t;

static void on_txd_change(void *user_data, pin_t pin, uint32_t value) {
  adapter_state_t *state = (adapter_state_t *)user_data;
  state->txd_edge_count++;
  
  // When TXD goes low, drive DATA low (ESP sending). When TXD goes high, release DATA (idle).
  if (value == 0) {
    // Set txd_drive first: pin_mode fires on_data_change synchronously and must see it.
    state->txd_drive = true;
    pin_mode(state->data, OUTPUT_LOW);
#ifdef ADAPTER_TRACE
    printf("[adapter] TXD edge #%lu low -> DATA OUTPUT_LOW\n", state->txd_edge_count);
#endif
  } else {
    // Release DATA before clearing txd_drive so the release edge is suppressed (RXD stays high).
    pin_mode(state->data, INPUT_PULLUP);
    state->txd_drive = false;
#ifdef ADAPTER_TRACE
    printf("[adapter] TXD edge #%lu high -> DATA INPUT_PULLUP\n", state->txd_edge_count);
#endif
  }
}

static void on_data_change(void *user_data, pin_t pin, uint32_t value) {
  adapter_state_t *state = (adapter_state_t *)user_data;
  state->data_edge_count++;
  
  // Drive RXD to follow DATA, but not if we (the adapter) are currently driving DATA.
  // This suppresses echo: the host's own TX transmissions don't loop back to RX.
  if (!state->txd_drive) {
    state->rxd_write_count++;
    if (value == 0) {
      pin_write(state->rxd, 0);
#ifdef ADAPTER_TRACE
      printf("[adapter] DATA edge #%lu low -> RXD write 0 (#%lu)\n", state->data_edge_count, state->rxd_write_count);
#endif
    } else {
      pin_write(state->rxd, 1);
#ifdef ADAPTER_TRACE
      printf("[adapter] DATA edge #%lu high -> RXD write 1 (#%lu)\n", state->data_edge_count, state->rxd_write_count);
#endif
    }
  } else {
#ifdef ADAPTER_TRACE
    printf("[adapter] DATA edge #%lu (suppressed, txd_drive=true)\n", state->data_edge_count);
#endif
  }
}

void chip_init(void) {
  adapter_state_t *state = calloc(1, sizeof(adapter_state_t));
  
  state->txd = pin_init("TXD", INPUT_PULLUP);
  state->rxd = pin_init("RXD", OUTPUT_HIGH);  // Idle high
  state->data = pin_init("DATA", INPUT_PULLUP);
  state->txd_drive = false;
  state->txd_edge_count = 0;
  state->data_edge_count = 0;
  state->rxd_write_count = 0;
  
  printf("[adapter] Initialized: TXD, RXD, DATA pins\n");
  
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

  // Sync RXD to current DATA level so it does not start at a stale value.
  pin_write(state->rxd, pin_read(state->data));
}
