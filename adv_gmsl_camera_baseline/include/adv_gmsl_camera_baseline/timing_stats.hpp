#ifndef ADV_GMSL_CAMERA_BASELINE__TIMING_STATS_HPP_
#define ADV_GMSL_CAMERA_BASELINE__TIMING_STATS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace adv_gmsl_camera_baseline
{

// Single-pass windowed accumulator (sum / sum-of-squares) for a stream of
// nanosecond-precision durations. Not the most numerically robust method
// for very long runs (Welford's algorithm would be preferable at millions
// of samples), but adequate for baseline captures on the order of
// thousands of frames per window.
struct TimingStats
{
  uint64_t count = 0;
  int64_t sum_ns = 0;
  double sum_sq_ns2 = 0.0;
  int64_t min_ns = std::numeric_limits<int64_t>::max();
  int64_t max_ns = std::numeric_limits<int64_t>::min();

  void add(int64_t ns)
  {
    ++count;
    sum_ns += ns;
    sum_sq_ns2 += static_cast<double>(ns) * static_cast<double>(ns);
    min_ns = std::min(min_ns, ns);
    max_ns = std::max(max_ns, ns);
  }

  double avg_ms() const
  {
    if (count == 0) {
      return 0.0;
    }
    return static_cast<double>(sum_ns) / static_cast<double>(count) / 1e6;
  }

  double stddev_ms() const
  {
    if (count <= 1) {
      return 0.0;
    }
    const double mean_ns = static_cast<double>(sum_ns) / static_cast<double>(count);
    const double variance_ns2 =
      (sum_sq_ns2 / static_cast<double>(count)) - (mean_ns * mean_ns);
    return std::sqrt(std::max(0.0, variance_ns2)) / 1e6;
  }

  double min_ms() const
  {
    if (count == 0) {
      return 0.0;
    }
    return static_cast<double>(min_ns) / 1e6;
  }

  double max_ms() const
  {
    if (count == 0) {
      return 0.0;
    }
    return static_cast<double>(max_ns) / 1e6;
  }

  void reset()
  {
    count = 0;
    sum_ns = 0;
    sum_sq_ns2 = 0.0;
    min_ns = std::numeric_limits<int64_t>::max();
    max_ns = std::numeric_limits<int64_t>::min();
  }
};

}  // namespace adv_gmsl_camera_baseline

#endif  // ADV_GMSL_CAMERA_BASELINE__TIMING_STATS_HPP_
