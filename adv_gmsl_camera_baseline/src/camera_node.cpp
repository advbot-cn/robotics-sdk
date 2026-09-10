// camera_node.cpp
//
// Minimal, intentionally "unoptimized" V4L2 -> ROS2 Image publisher.
//
// Design goals (per baseline methodology):
//   ...
//   - Timestamp is a SOFTWARE timestamp (RCL_STEADY_TIME), taken
//     immediately after VIDIOC_DQBUF returns -- NOT the V4L2 buffer's
//     own buf.timestamp field. On JetPack 6.2, buf.timestamp comes from
//     the RTCPU coprocessor's own clock domain, and converting it into
//     the CPU's clock domain requires an offset that has been reported
//     unreliable by multiple developers (see capture_loop() comments for
//     detail). This means T0 excludes the fixed hardware capture delay
//     (exposure+readout+RTCPU+driver notification), but that fixed delay
//     is present identically across baseline and later NITROS comparison
//     runs, so it cancels out in the A/B comparison that matters here.
//
// This node deliberately does NOT do: DMA-BUF, NvBufSurface, CUDA,
// NITROS types, or any GPU-side preprocessing. That is the point of a
// baseline.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <poll.h>

#include <cstring>
#include <stdexcept>
#include <vector>
#include <thread>
#include <atomic>

namespace
{

struct MmapBuffer
{
  void * start = nullptr;
  size_t length = 0;
};

}  // namespace

class CameraNode : public rclcpp::Node
{
public:
  CameraNode()
  : Node("camera_node"),
    fd_(-1),
    streaming_(false),
    running_(true)
  {
    // ---- Parameters (explicit, no hidden defaults) ----
    device_ = this->declare_parameter<std::string>("device", "/dev/video2");
    width_ = this->declare_parameter<int>("width", 1920);
    height_ = this->declare_parameter<int>("height", 1080);
    // "UYVY" is the 4-char V4L2 pixel format code; mapped to
    // sensor_msgs::image_encodings::YUV422 below.
    requested_fps_ = this->declare_parameter<int>("fps", 30);
    buffer_count_ = this->declare_parameter<int>("buffer_count", 4);
    topic_name_ = this->declare_parameter<std::string>("topic", "camera/image_raw");

    RCLCPP_INFO(
      this->get_logger(),
      "Opening %s (%dx%d @ %d fps requested, %d mmap buffers)",
      device_.c_str(), width_, height_, requested_fps_, buffer_count_);

    open_device();
    set_format();
    request_buffers();
    stream_on();

    // Explicit QoS: sensor-data profile (best-effort, keep-last-1).
    // This is stated here on purpose -- see baseline discussion: QoS is
    // a variable that must be fixed and known before comparing against
    // an accelerated pipeline later.
    auto qos = rclcpp::SensorDataQoS();
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, qos);
    RCLCPP_INFO(
      this->get_logger(),
      "Publishing on '%s' with SensorDataQoS (BEST_EFFORT, depth=%ld)",
      topic_name_.c_str(), qos.get_rmw_qos_profile().depth);

    // VIDIOC_DQBUF blocks, so capture runs on its own thread; ROS2
    // publisher::publish() is safe to call from a non-executor thread.
    capture_thread_ = std::thread(&CameraNode::capture_loop, this);
  }

  ~CameraNode() override
  {
    running_ = false;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    stream_off();
    unmap_buffers();
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  void open_device()
  {
    fd_ = open(device_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("Failed to open " + device_ + ": " + std::strerror(errno));
    }

    struct v4l2_capability cap {};
    if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
      throw std::runtime_error("VIDIOC_QUERYCAP failed: " + std::string(std::strerror(errno)));
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
      throw std::runtime_error(device_ + " does not support V4L2_CAP_VIDEO_CAPTURE");
    }
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
      throw std::runtime_error(device_ + " does not support V4L2_CAP_STREAMING (mmap)");
    }
  }

  void set_format()
  {
    struct v4l2_format fmt {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width_;
    fmt.fmt.pix.height = height_;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
      throw std::runtime_error("VIDIOC_S_FMT failed: " + std::string(std::strerror(errno)));
    }

    // The driver may adjust width/height/format to something it actually
    // supports -- always read back the negotiated format, never assume
    // the request was honored exactly.
    width_ = fmt.fmt.pix.width;
    height_ = fmt.fmt.pix.height;
    bytes_per_line_ = fmt.fmt.pix.bytesperline;
    image_size_ = fmt.fmt.pix.sizeimage;

    RCLCPP_INFO(
      this->get_logger(),
      "Negotiated format: %dx%d, bytesperline=%u, sizeimage=%u",
      width_, height_, bytes_per_line_, image_size_);

    struct v4l2_streamparm parm {};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = requested_fps_;
    if (ioctl(fd_, VIDIOC_S_PARM, &parm) < 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "VIDIOC_S_PARM failed (device may not support frame rate control): %s",
        std::strerror(errno));
    }
  }

  void request_buffers()
  {
    struct v4l2_requestbuffers req {};
    req.count = buffer_count_;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
      throw std::runtime_error("VIDIOC_REQBUFS failed: " + std::string(std::strerror(errno)));
    }
    if (req.count < 2) {
      throw std::runtime_error("Insufficient buffer memory (got " + std::to_string(req.count) + ")");
    }
    buffer_count_ = req.count;
    buffers_.resize(buffer_count_);

    for (uint32_t i = 0; i < static_cast<uint32_t>(buffer_count_); ++i) {
      struct v4l2_buffer buf {};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;

      if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
        throw std::runtime_error("VIDIOC_QUERYBUF failed: " + std::string(std::strerror(errno)));
      }

      buffers_[i].length = buf.length;
      buffers_[i].start = mmap(
        nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);

      if (buffers_[i].start == MAP_FAILED) {
        throw std::runtime_error("mmap failed: " + std::string(std::strerror(errno)));
      }

      if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
        throw std::runtime_error("Initial VIDIOC_QBUF failed: " + std::string(std::strerror(errno)));
      }
    }
  }

  void stream_on()
  {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
      throw std::runtime_error("VIDIOC_STREAMON failed: " + std::string(std::strerror(errno)));
    }
    streaming_ = true;
  }

  void stream_off()
  {
    if (!streaming_ || fd_ < 0) {
      return;
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd_, VIDIOC_STREAMOFF, &type);
    streaming_ = false;
  }

  void unmap_buffers()
  {
    for (auto & b : buffers_) {
      if (b.start && b.start != MAP_FAILED) {
        munmap(b.start, b.length);
        b.start = nullptr;
      }
    }
  }

  void capture_loop()
  {
    uint64_t frame_count = 0;
    uint64_t publish_count = 0;

    while (running_ && rclcpp::ok()) {
      struct pollfd pfd {};
      pfd.fd = fd_;
      pfd.events = POLLIN;

      int poll_ret = poll(&pfd, 1, 2000 /* ms timeout */);
      if (poll_ret < 0) {
        if (errno == EINTR) {continue;}
        RCLCPP_ERROR(this->get_logger(), "poll() failed: %s", std::strerror(errno));
        break;
      }
      if (poll_ret == 0) {
        RCLCPP_WARN(this->get_logger(), "V4L2 poll timeout - no frame in 2s");
        continue;
      }

      struct v4l2_buffer buf {};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        if (errno == EAGAIN) {continue;}
        RCLCPP_ERROR(this->get_logger(), "VIDIOC_DQBUF failed: %s", std::strerror(errno));
        break;
      }

      ++frame_count;

      // ---- Software timestamp — stamp immediately after DQBUF returns ----
      // No longer use buf.timestamp (RTCPU domain) + offset correction. On JP6.2, this
      // offset has been reported by multiple developers to be unreliable (second-level
      // error, continuous drift), and cannot be used as a basis for precise correction.
      // Use ROS2's own clock instead (this->get_clock()->now(), CLOCK_MONOTONIC domain),
      // so that the publisher and subscriber sides remain in the same clock domain
      // throughout, with no need for any cross-domain offset.
      //
      // Cost: This timestamp lags the actual hardware SOF instant by the fixed delay of
      // "exposure + readout + RTCPU processing + driver notification" (the exact value
      // depends on the sensor/driver, but is essentially constant for the same camera).
      // Benefit: This fixed delay is present equally in the baseline and in future NITROS
      // comparison experiments, so it cancels out automatically in difference comparisons
      // — and that is exactly the number we really want to see.
      const rclcpp::Time capture_time = capture_clock_.now();

      // ---- Intentional CPU copy (this is the baseline's defining trait) ----
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp = capture_time;
      msg->header.frame_id = "camera";
      msg->height = height_;
      msg->width = width_;
      msg->encoding = sensor_msgs::image_encodings::YUV422;  // UYVY packed
      msg->is_bigendian = false;
      msg->step = bytes_per_line_;

      const size_t bytes_to_copy = buf.bytesused > 0 ? buf.bytesused : image_size_;
      msg->data.resize(bytes_to_copy);
      std::memcpy(
        msg->data.data(),
        buffers_[buf.index].start,
        bytes_to_copy);
      // ---- End of the copy we intend to measure / later remove ----

      publisher_->publish(std::move(msg));
      ++publish_count;

      if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
        RCLCPP_ERROR(this->get_logger(), "VIDIOC_QBUF (requeue) failed: %s", std::strerror(errno));
        break;
      }

      if (frame_count % 100 == 0) {
        RCLCPP_INFO(
          this->get_logger(), "frames captured=%lu published=%lu",
          static_cast<unsigned long>(frame_count),
          static_cast<unsigned long>(publish_count));
      }
    }
  }

  // V4L2 state
  std::string device_;
  int fd_;
  int width_;
  int height_;
  int requested_fps_;
  int buffer_count_;
  uint32_t bytes_per_line_ = 0;
  uint32_t image_size_ = 0;
  bool streaming_;
  std::vector<MmapBuffer> buffers_;
  rclcpp::Clock capture_clock_{RCL_STEADY_TIME};

  // ROS2 state
  std::string topic_name_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  std::thread capture_thread_;
  std::atomic<bool> running_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("camera_node"), "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
