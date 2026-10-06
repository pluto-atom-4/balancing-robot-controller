#pragma once

// Loop period min/mean/max tracker (jitter evidence). Header-only, no heap,
// no Arduino dependency. Integer microseconds.
//
// Caller duties (this lib does NOT do these):
//  - Convert seconds to us, or better use micros() deltas (unsigned
//    subtraction is wrap-safe). Never pass a negative dt: contract v1 says
//    dt < 0 is a fault/stop, so reject it BEFORE calling add().
//  - The first hal wait_next_tick() returns the NOMINAL period, not a
//    measurement. Skip it (or do not feed it) to avoid biasing the stats.
//  - Check count > 0 before reading min_us (it is 0xFFFFFFFF when empty).
// The jitter budget numbers come from freecad-workspace#345 (F.6); this lib
// only provides the tracker. over_budget counts LATE ticks only
// (period_us > budget_max_us); early ticks show up in min_us.
// Not part of the HAL contract: does not touch kContractVersion.

#include <cstdint>

namespace loop_stats {

struct LoopStats {
  uint32_t count = 0;
  uint32_t min_us = 0xFFFFFFFFu;
  uint32_t max_us = 0;
  uint32_t over_budget = 0;
  uint64_t sum_us = 0;

  // Records one loop period. Once count reaches 0xFFFFFFFF further samples
  // are ignored (saturate), so mean_us() never goes wrong from wraparound.
  void add(uint32_t period_us, uint32_t budget_max_us) {
    if (count == 0xFFFFFFFFu) return;
    ++count;
    sum_us += period_us;
    if (period_us < min_us) min_us = period_us;
    if (period_us > max_us) max_us = period_us;
    if (period_us > budget_max_us) ++over_budget;
  }

  // Mean period, truncated toward zero. 0 if count == 0.
  uint32_t mean_us() const {
    if (count == 0) return 0;
    return static_cast<uint32_t>(sum_us / count);
  }

  void reset() { *this = LoopStats(); }
};

}  // namespace loop_stats
