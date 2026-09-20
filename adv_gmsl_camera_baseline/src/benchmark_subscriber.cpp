// adv_gmsl_camera_baseline / benchmark_subscriber.cpp
//
// Windowed measurement node for the camera baseline. Every `report_every_n`
// frames it closes out a measurement window and emits one row of stats
// (console log, and optionally CSV) covering:
//
//   - fps (from window steady-clock duration)
//   - pipeline_latency (steady_now - header.stamp): min/avg/max/stddev
//       This is the TOTAL latency from camera_node's post-DQBUF software
//       timestamp to this subscriber's callback execution. See
//       camera_node.cpp / README for why header.stamp is a software
//       timestamp, not the camera's true hardware SOF time.
//   - wire_transport (rmw received_timestamp - rmw source_timestamp):
//       the portion of pipeline_latency spent in DDS/RTPS transport itself
//       (serialization already happened before source_timestamp is
//       recorded on the publish side, so this is transport-only, not
//       serialize+transport).
//   - executor_dispatch (steady_now - rmw received_timestamp):
//       the portion spent queued, waiting for this node's executor to
//       actually invoke the callback after the message physically arrived.
//       This is the term expected to grow with multiple concurrent
//       subscriptions/nodes sharing one executor (multi-camera scaling).
//   - period (inter-arrival gap): min/avg/max/stddev, and suspected-drop
//       heuristic (gap > 1.5x expected period)
//   - bandwidth: bytes/sec over the window (mirrors `ros2 topic bw`)
//
// IMPORTANT: source_timestamp/received_timestamp come from the RMW layer
// (rmw_message_info_t), not from this node's own code. They have been
// unreliable/zero on some RMW implementations and ROS2 versions in the
// past. This node validates them every window (non-zero, received >=
// source) and reports how many samples were rejected -- do not trust
// wire_transport/executor_dispatch numbers if invalid_timestamp_count is
// a large fraction of window_frame_count.
//
// This node is meant to run ALONGSIDE, not replace, independent
// verification tools (see README "Recording a full baseline snapshot"):
// `ros2 topic hz`, `ros2 topic bw`, `tegrastats`, `nvpmodel -q` /
// `jetson_clocks --show`.
//
// Drop detection remains a heuristic (see camera_node.cpp / README):
// sensor_msgs/Image has no sequence number in ROS2, so "suspected_drops"
// is inferred from inter-arrival gaps, not a ground-truth count.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <adv_gmsl_camera_baseline/timing_stats.hpp>

#include <cstdint>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>

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
          << "pipeline_latency_min_ms,pipeline_latency_avg_ms,"
          << "pipeline_latency_max_ms,pipeline_latency_stddev_ms,"
          << "wire_transport_min_ms,wire_transport_avg_ms,"
          << "wire_transport_max_ms,wire_transport_stddev_ms,"
          << "executor_dispatch_min_ms,executor_dispatch_avg_ms,"
          << "executor_dispatch_max_ms,executor_dispatch_stddev_ms,"
          << "invalid_timestamp_count,"
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
      std::bind(
        &BenchmarkSubscriber::on_image, this,
        std::placeholders::_1, std::placeholders::_2));

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
    window_invalid_timestamp_count_ = 0;
    window_suspected_drops_ = 0;

    pipeline_latency_stats_.reset();
    wire_transport_stats_.reset();
    executor_dispatch_stats_.reset();
    period_stats_.reset();
  }

  void on_image(
    const sensor_msgs::msg::Image::SharedPtr msg,
    const rclcpp::MessageInfo & info)
  {
    const rclcpp::Time now = steady_clock_.now();

    ++cumulative_received_;
    ++window_frame_count_;
    window_bytes_ += msg->data.size();

    // ---- Total pipeline latency: T_now - T0 (camera_node's post-DQBUF stamp) ----
    const rclcpp::Time stamp(msg->header.stamp, RCL_STEADY_TIME);
    int64_t pipeline_latency_ns = (now - stamp).nanoseconds();
    if (pipeline_latency_ns < 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), steady_clock_, 5000,
        "Negative pipeline latency (%ld ns) - check clock source consistency "
        "between camera_node and this subscriber.",
        static_cast<long>(pipeline_latency_ns));
    } else {
      pipeline_latency_stats_.add(pipeline_latency_ns);
    }

    // ---- Breakdown: wire transport + executor dispatch, from RMW timestamps ----
    // These come from the middleware, not our own code -- validate before use.
    const rmw_message_info_t & rmw_info = info.get_rmw_message_info();
    const int64_t source_ts_ns = rmw_info.source_timestamp;
    const int64_t received_ts_ns = rmw_info.received_timestamp;

    const bool timestamps_valid =
      source_ts_ns != 0 && received_ts_ns != 0 && received_ts_ns >= source_ts_ns;

    if (!timestamps_valid) {
      ++window_invalid_timestamp_count_;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), steady_clock_, 5000,
        "Invalid RMW timestamps this sample (source=%ld received=%ld) - "
        "wire_transport/executor_dispatch breakdown unavailable for this "
        "frame. If this happens frequently, do not trust the breakdown "
        "columns for this run; fall back to pipeline_latency only.",
        static_cast<long>(source_ts_ns), static_cast<long>(received_ts_ns));
    } else {
      const int64_t wire_transport_ns = received_ts_ns - source_ts_ns;
      const int64_t executor_dispatch_ns = system_clock_.now().nanoseconds() - received_ts_ns;

      wire_transport_stats_.add(wire_transport_ns);
      // executor_dispatch should not be negative either (received happens
      // before the callback runs, by definition) -- guard the same way.
      if (executor_dispatch_ns < 0) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), steady_clock_, 5000,
          "Negative executor_dispatch (%ld ns) computed from RMW "
          "received_timestamp - treating this sample's breakdown as invalid.",
          static_cast<long>(executor_dispatch_ns));
        ++window_invalid_timestamp_count_;
      } else {
	executor_dispatch_stats_.add(executor_dispatch_ns);
      }
    }

    // ---- Inter-arrival period / suspected drop detection ----
    if (has_previous_arrival_) {
      int64_t period_ns = (now - previous_arrival_).nanoseconds();
      period_stats_.add(period_ns);

      if (period_ns > drop_threshold_ns_) {
        ++window_suspected_drops_;
        ++cumulative_suspected_drops_;
        RCLCPP_WARN(
          this->get_logger(),
          "Suspected drop: gap=%.2f ms (> 1.5x expected %.2f ms)",
          static_cast<double>(period_ns) / 1e6,
          static_cast<double>(expected_period_ns_) / 1e6);
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

    RCLCPP_INFO(
      this->get_logger(),
      "n=%d fps=%.2f | "
      "pipeline_latency(min/avg/max/std)=%.2f/%.2f/%.2f/%.2fms | "
      "wire_transport(avg/std)=%.3f/%.3fms | "
      "executor_dispatch(avg/std)=%.3f/%.3fms | "
      "period(min/avg/max/std)=%.2f/%.2f/%.2f/%.2fms | "
      "bw=%.2fMB/s | drops_in_window=%lu | invalid_ts=%lu | "
      "cumulative_recv=%lu drops=%lu",
      window_frame_count_, avg_fps,
      pipeline_latency_stats_.min_ms(), pipeline_latency_stats_.avg_ms(),
      pipeline_latency_stats_.max_ms(), pipeline_latency_stats_.stddev_ms(),
      wire_transport_stats_.avg_ms(), wire_transport_stats_.stddev_ms(),
      executor_dispatch_stats_.avg_ms(), executor_dispatch_stats_.stddev_ms(),
      period_stats_.min_ms(), period_stats_.avg_ms(),
      period_stats_.max_ms(), period_stats_.stddev_ms(),
      bandwidth_MBps,
      static_cast<unsigned long>(window_suspected_drops_),
      static_cast<unsigned long>(window_invalid_timestamp_count_),
      static_cast<unsigned long>(cumulative_received_),
      static_cast<unsigned long>(cumulative_suspected_drops_));

    if (window_invalid_timestamp_count_ > 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "%lu/%d samples in this window had invalid RMW timestamps -- "
        "wire_transport/executor_dispatch numbers above are computed from "
        "the remaining valid samples only, not all %d frames.",
        static_cast<unsigned long>(window_invalid_timestamp_count_),
        window_frame_count_, window_frame_count_);
    }

    if (csv_file_.is_open()) {
      csv_file_
        << now_iso8601() << ","
        << window_frame_count_ << ","
        << avg_fps << ","
        << pipeline_latency_stats_.min_ms() << ","
        << pipeline_latency_stats_.avg_ms() << ","
        << pipeline_latency_stats_.max_ms() << ","
        << pipeline_latency_stats_.stddev_ms() << ","
        << wire_transport_stats_.min_ms() << ","
        << wire_transport_stats_.avg_ms() << ","
        << wire_transport_stats_.max_ms() << ","
        << wire_transport_stats_.stddev_ms() << ","
        << executor_dispatch_stats_.min_ms() << ","
        << executor_dispatch_stats_.avg_ms() << ","
        << executor_dispatch_stats_.max_ms() << ","
        << executor_dispatch_stats_.stddev_ms() << ","
        << window_invalid_timestamp_count_ << ","
        << period_stats_.min_ms() << ","
        << period_stats_.avg_ms() << ","
        << period_stats_.max_ms() << ","
        << period_stats_.stddev_ms() << ","
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
  rclcpp::Clock system_clock_{RCL_SYSTEM_TIME};

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
  uint64_t window_invalid_timestamp_count_;
  uint64_t window_suspected_drops_;

  adv_gmsl_camera_baseline::TimingStats pipeline_latency_stats_;
  adv_gmsl_camera_baseline::TimingStats wire_transport_stats_;
  adv_gmsl_camera_baseline::TimingStats executor_dispatch_stats_;
  adv_gmsl_camera_baseline::TimingStats period_stats_;
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
