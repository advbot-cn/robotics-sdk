# adv_gmsl_camera_baseline

最小、故意不做优化的 V4L2 → ROS2 Image 发布节点，作为后续 NITROS/零拷贝对比实验的基准（baseline）。

## 设计要点（对应之前讨论的 baseline 方法论）

- 直接用 V4L2 mmap 采集，不套用现成的 camera driver 包，链路每一步都在自己掌控之中。
- **故意保留 CPU memcpy**（V4L2 buffer → `sensor_msgs::Image::data`）——这就是以后要用零拷贝优化掉的开销，现在不能提前优化。
- **显式声明 QoS**（`SensorDataQoS`，best-effort + depth=1），不依赖隐式默认值，避免以后对比时混入未声明的变量。
- 时间戳取自 V4L2 buffer 自带的硬件/驱动时间戳（`buf.timestamp`），不是应用层在 `DQBUF` 返回后临时打的时间戳。
- 读取协商后的实际格式（`VIDIOC_S_FMT` 之后回读 `width/height/bytesperline`），不假设驱动一定按请求值配置成功。

**没有做**（也不应该做，这是baseline的关键）：DMA-BUF、NvBufSurface、CUDA预处理、NITROS消息类型、GPU zero-copy。

## 编译

把这个包放进你的 Isaac ROS workspace（`${ISAAC_ROS_WS}/src/`），在容器内：

```bash
cd ${ISAAC_ROS_WS}
colcon build --packages-select adv_gmsl_camera_baseline --symlink-install
source install/setup.bash
```

## 运行

```bash
ros2 run adv_gmsl_camera_baseline camera_node \
  --ros-args \
  -p device:=/dev/video2 \
  -p width:=1920 \
  -p height:=1080 \
  -p fps:=30 \
  -p buffer_count:=4 \
  -p topic:=camera/image_raw
```

## 验证（对应 baseline 文档第六步）

```bash
# 另开一个终端
ros2 topic hz /camera/image_raw
# 期望: average rate 接近 30 Hz (取决于实测V4L2的稳定值)

ros2 topic info /camera/image_raw
# 确认 publisher 数量为1

# RViz2 里订阅 /camera/image_raw，确认画面正常显示
```

## 已知限制 / 下一步要补的（不是bug，是有意留到后续阶段处理的）

1. **没有 publish/received 计数比对**——当前只在节点内部打印 `captured`/`published`（两者理论上应始终相等，因为发布失败会走异常路径）。要验证 ROS2 层本身有没有丢消息，需要在订阅端（比如一个简单的测试订阅节点）单独计数，跟这里的 `published` 计数比对。SensorDataQoS 是 BEST_EFFORT，订阅端处理跟不上时中间件可能丢消息，这一步没有做。
2. **UYVY 编码假设**——如果实际摄像头协商后的像素格式不是 UYVY（比如被驱动调整成了 YUYV），`msg->encoding` 目前写死成 `YUV422 (UYVY)`，需要根据 `VIDIOC_S_FMT` 回读的 `fmt.fmt.pix.pixelformat` 实际值动态映射，当前版本里这行是手动写死的，建议实测确认一致后再继续，不一致需要改成 `YUV422_YUY2`（YUYV对应的encoding名）或做相应转换。
3. **DVFS/功耗模式未在节点内控制**——测试前需要按之前讨论的顺序手动执行 `nvpmodel -m 0` + `jetson_clocks`，节点本身不做这件事。
4. **时钟基准**——`v4l2_timestamp_to_ros()` 假设驱动时间戳是系统时钟（`RCL_SYSTEM_TIME`）。部分 V4L2/UVC 驱动实际使用 `CLOCK_MONOTONIC`，如果后续要跟其他基于 `TIME_MONOTONIC` 的测量点（比如 EtherCAT 那边用的 `CLOCK_MONOTONIC`）对齐延迟数据，需要先确认这台设备驱动实际用的时钟源（`v4l2-ctl -d /dev/video2 --all` 里的 `Streaming Parameters` / 内核驱动源码可查），否则跨系统时间戳换算会有偏差。
