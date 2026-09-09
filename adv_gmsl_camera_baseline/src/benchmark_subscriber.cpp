// adv_gmsl_camera_baseline / benchmark_subscriber.cpp
//
// Windowed measurement node for the camera baseline. Every `report_every_n`
// frames it closes out a measurement window and emits one row of stats
// (to the console log, and optionally to a CSV file) covering:
//
//   - fps (from window monotonic-clock duration, not just period average)
//   - end-to-end latency (steady_now - header.stamp): min/avg/max
//   - bandwidth: bytes/sec over the window (mirrors `ros2 topic bw`)
//   - suspected drops within the window
//
// This node is meant to run ALONGSIDE, not replace, the following
// independent verifications (see README "Recording a full baseline
// snapshot"):
//   - `ros2 topic hz <topic> --window 200`  (cross-check against this
//     node's own fps/stddev numbers)
//   - `ros2 topic bw <topic>`               (cross-check bandwidth)
//   - `tegrastats`                          (system-level CPU/GPU/power -
//     this is out of scope for a ROS2 node and should stay a separate tool)
//   - `nvpmodel -q` / `jetson_clocks --show` (confirms DVFS is locked
//     BEFORE the run; this node cannot verify or control that)
//
// Drop detection remains a heuristic (see camera_node.cpp / README):
// sensor_msgs/Image has no sequence number in ROS2, so "suspected_drops"
// is inferred from inter-arrival gaps exceeding 1.5x the expected period,
// not a ground-truth count. Cross-check against camera_node's own
// captured/published counters for the real picture.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cstdint>
#include <cmath>
#include <limits>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace
{
constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

std::string now_iso8601()
{
  auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::ostringstream oss;
  oss << std::put_time(std::localtime(&t), "%Y-%m-%dT%H:%M:%S");
  return oss.str();
}
}  // namespace

class BenchmarkSubscriber : public rclcpp::Node
{
public:
  BenchmarkSubscriber()
  : Node("benchmark_subscriber")
  {
    topic_name_ = this->declare_parameter<std::string>("topic", "camera/image_raw");
    target_fps_ = this->declare_parameter<double>("target_fps", 30.0);
    report_every_n_ = this->declare_parameter<int>("report_every_n", 100);
    csv_path_ = this->declare_parameter<std::string>("csv_output", "");

    expected_period_ns_ = static_cast<int64_t>(kNanosecondsPerSecond / target_fps_);
    drop_threshold_ns_ = static_cast<int64_t>(expected_period_ns_ * 1.5);

    if (!csv_path_.empty()) {
      csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
      if (!csv_file_.is_open()) {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to open CSV output file: %s", csv_path_.c_str());
      } else {
        csv_file_
          << "wall_time,window_frame_count,avg_fps,"
          << "latency_min_ms,latency_avg_ms,latency_max_ms,"
          << "period_min_ms,period_avg_ms,period_max_ms,period_stddev_ms,"
          << "bandwidth_MBps,suspected_drops_in_window,"
          << "cumulative_received,cumulative_suspected_drops\n";
        RCLCPP_INFO(this->get_logger(), "Writing CSV rows to: %s", csv_path_.c_str());
      }
    }

    reset_window();
    cumulative_received_ = 0;
    cumulative_suspected_drops_ = 0;
    has_previous_arrival_ = false;

    auto qos = rclcpp::SensorDataQoS();
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_name_, qos,
      std::bind(&BenchmarkSubscriber::on_image, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Subscribed to '%s' (SensorDataQoS), target_fps=%.1f, "
      "drop threshold=%.1f ms, reporting every %d frames%s",
      topic_name_.c_str(), target_fps_,
      static_cast<double>(drop_threshold_ns_) / 1e6,
      report_every_n_,
      csv_path_.empty() ? " (console only, no CSV configured)" : "");
  }

  ~BenchmarkSubscriber() override
  {
    if (csv_file_.is_open()) {
      csv_file_.close();
    }
  }

  void print_summary()
  {
    RCLCPP_INFO(this->get_logger(), "%s", "");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), " Benchmark Subscriber - Final Summary");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(
      this->get_logger(), "cumulative_received=%lu  cumulative_suspected_drops=%lu",
      static_cast<unsigned long>(cumulative_received_),
      static_cast<unsigned long>(cumulative_suspected_drops_));
    if (!csv_path_.empty()) {
      RCLCPP_INFO(this->get_logger(), "Per-window data written to: %s", csv_path_.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

private:
  void reset_window()
  {
    window_frame_count_ = 0;
    window_start_time_ = steady_clock_.now();
    window_bytes_ = 0;

    window_min_period_ns_ = std::numeric_limits<int64_t>::max();
    window_max_period_ns_ = std::numeric_limits<int64_t>::min();
    window_period_sum_ms_ = 0.0;
    window_period_sq_sum_ = 0.0;  // for stddev, in (ms)^2
    window_period_samples_ = 0;

    window_min_latency_ns_ = std::numeric_limits<int64_t>::max();
    window_max_latency_ns_ = std::numeric_limits<int64_t>::min();
    window_latency_sum_ns_ = 0;
    window_latency_samples_ = 0;

    window_suspected_drops_ = 0;
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    const rclcpp::Time now = steady_clock_.now();

    ++cumulative_received_;
    ++window_frame_count_;
    window_bytes_ += msg->data.size();

    // ---- Latency: T_now - T0 ----
    const rclcpp::Time stamp(msg->header.stamp, RCL_STEADY_TIME);
    int64_t latency_ns = (now - stamp).nanoseconds();
    if (latency_ns < 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), steady_clock_, 5000,
        "Negative latency (%ld ns) - check clock source consistency between "
        "camera_node and this subscriber.",
        static_cast<long>(latency_ns));
    } else {
      window_latency_sum_ns_ += latency_ns;
      window_min_latency_ns_ = std::min(window_min_latency_ns_, latency_ns);
      window_max_latency_ns_ = std::max(window_max_latency_ns_, latency_ns);
      ++window_latency_samples_;
    }

    // ---- Inter-arrival period / suspected drop detection ----
    if (has_previous_arrival_) {
      int64_t period_ns = (now - previous_arrival_).nanoseconds();
      double period_ms = static_cast<double>(period_ns) / 1e6;

      window_period_sum_ms_ += period_ms;
      window_period_sq_sum_ += period_ms * period_ms;
      window_min_period_ns_ = std::min(window_min_period_ns_, period_ns);
      window_max_period_ns_ = std::max(window_max_period_ns_, period_ns);
      ++window_period_samples_;

      if (period_ns > drop_threshold_ns_) {
        ++window_suspected_drops_;
        ++cumulative_suspected_drops_;
        RCLCPP_WARN(
          this->get_logger(),
          "Suspected drop: gap=%.2f ms (> 1.5x expected %.2f ms)",
          period_ms, static_cast<double>(expected_period_ns_) / 1e6);
      }
    }
    previous_arrival_ = now;
    has_previous_arrival_ = true;

    if (window_frame_count_ >= report_every_n_) {
      emit_window(now);
      reset_window();
    }
  }

  void emit_window(const rclcpp::Time & window_end_time)
  {
    double window_duration_s =
      (window_end_time - window_start_time_).nanoseconds() / 1e9;
    double avg_fps = (window_duration_s > 0.0)
      ? window_frame_count_ / window_duration_s : 0.0;
    double bandwidth_MBps = (window_duration_s > 0.0)
      ? (window_bytes_ / window_duration_s) / (1024.0 * 1024.0) : 0.0;

    double period_avg_ms = (window_period_samples_ > 0)
      ? window_period_sum_ms_ / window_period_samples_ : 0.0;
    double period_variance = (window_period_samples_ > 0)
      ? (window_period_sq_sum_ / window_period_samples_) - (period_avg_ms * period_avg_ms)
      : 0.0;
    double period_stddev_ms = (period_variance > 0.0) ? std::sqrt(period_variance) : 0.0;

    double latency_avg_ms = (window_latency_samples_ > 0)
      ? (static_cast<double>(window_latency_sum_ns_) / window_latency_samples_) / 1e6 : 0.0;
    double latency_min_ms = (window_min_latency_ns_ == std::numeric_limits<int64_t>::max())
      ? 0.0 : static_cast<double>(window_min_latency_ns_) / 1e6;
    double latency_max_ms = (window_max_latency_ns_ == std::numeric_limits<int64_t>::min())
      ? 0.0 : static_cast<double>(window_max_latency_ns_) / 1e6;
    double period_min_ms = (window_min_period_ns_ == std::numeric_limits<int64_t>::max())
      ? 0.0 : static_cast<double>(window_min_period_ns_) / 1e6;
    double period_max_ms = (window_max_period_ns_ == std::numeric_limits<int64_t>::min())
      ? 0.0 : static_cast<double>(window_max_period_ns_) / 1e6;

    RCLCPP_INFO(
      this->get_logger(),
      "n=%d fps=%.2f | latency(min/avg/max)=%.2f/%.2f/%.2fms | "
      "period(min/avg/max/std)=%.2f/%.2f/%.2f/%.2fms | "
      "bw=%.2fMB/s | drops_in_window=%lu | cumulative_recv=%lu drops=%lu",
      window_frame_count_, avg_fps,
      latency_min_ms, latency_avg_ms, latency_max_ms,
      period_min_ms, period_avg_ms, period_max_ms, period_stddev_ms,
      bandwidth_MBps,
      static_cast<unsigned long>(window_suspected_drops_),
      static_cast<unsigned long>(cumulative_received_),
      static_cast<unsigned long>(cumulative_suspected_drops_));

    if (csv_file_.is_open()) {
      csv_file_
        << now_iso8601() << ","
        << window_frame_count_ << ","
        << avg_fps << ","
        << latency_min_ms << "," << latency_avg_ms << "," << latency_max_ms << ","
        << period_min_ms << "," << period_avg_ms << "," << period_max_ms << ","
        << period_stddev_ms << ","
        << bandwidth_MBps << ","
        << window_suspected_drops_ << ","
        << cumulative_received_ << ","
        << cumulative_suspected_drops_ << "\n";
      csv_file_.flush();  // flush per-window so data survives Ctrl+C / crashes
    }
  }

  // Config
  std::string topic_name_;
  double target_fps_;
  int report_every_n_;
  int64_t expected_period_ns_;
  int64_t drop_threshold_ns_;
  std::string csv_path_;
  std::ofstream csv_file_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  // Cumulative (whole run)
  uint64_t cumulative_received_;
  uint64_t cumulative_suspected_drops_;

  // Cross-window state (arrival tracking must persist across window resets)
  bool has_previous_arrival_;
  rclcpp::Time previous_arrival_;

  // Current window accumulators
  int window_frame_count_;
  rclcpp::Time window_start_time_;
  uint64_t window_bytes_;

  int64_t window_min_period_ns_;
  int64_t window_max_period_ns_;
  double window_period_sum_ms_;
  double window_period_sq_sum_;   // sum of squares, ms^2, for stddev
  int window_period_samples_;

  int64_t window_min_latency_ns_;
  int64_t window_max_latency_ns_;
  int64_t window_latency_sum_ns_;
  int window_latency_samples_;

  uint64_t window_suspected_drops_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BenchmarkSubscriber>();
  rclcpp::spin(node);
  node->print_summary();
  rclcpp::shutdown();
  return 0;
}
