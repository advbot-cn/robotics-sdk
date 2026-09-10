# adv_gmsl_camera_baseline

GMSL 摄像头的最小 V4L2 → ROS2 Image baseline，配套一个窗口化统计的 benchmark subscriber，用于建立后续 NITROS/零拷贝对比实验的基准数据。

## 组成

```
adv_gmsl_camera_baseline/
├── CMakeLists.txt
├── package.xml
├── LICENSE (MIT)
├── README.md
├── launch/
│   └── baseline_launch.py
└── src/
    ├── camera_node.cpp          # V4L2 mmap 采集 -> sensor_msgs/Image 发布
    └── benchmark_subscriber.cpp # 订阅并按窗口统计 FPS/延迟/带宽/疑似丢帧
```

## 设计要点

### camera_node

- V4L2 mmap 采集，不套用现成的 camera driver 包，链路每一步（open/QUERYCAP/S_FMT/REQBUFS/mmap/STREAMON/DQBUF/QBUF）都在自己掌控之中。
- **故意保留 CPU memcpy**（V4L2 buffer → `sensor_msgs::Image::data`）——这是以后接 NITROS/零拷贝时要优化掉、并用来做 A/B 对比的关键开销，第一版不能提前优化掉。
- 显式声明 **`SensorDataQoS`**（best-effort + depth=1），不依赖隐式默认值，避免以后对比实验里混入未声明的变量。
- 读取协商后的实际格式（`VIDIOC_S_FMT` 之后回读 `width/height/bytesperline/sizeimage`），不假设驱动一定按请求值配置成功。

### 时间戳 = 软件时间戳，不是硬件 SOF 时间戳（重要，务必先读这段）

`msg->header.stamp` 使用的是 **`VIDIOC_DQBUF` 返回后立刻打的软件时间戳**（`rclcpp::Clock{RCL_STEADY_TIME}`），**不是** V4L2 buffer 自带的 `buf.timestamp`。

这是一个刻意的设计决定，原因：

- Jetson AGX Orin（JetPack 6.2）上，CSI/GMSL 摄像头的 `buf.timestamp` 来自 **RTCPU 协处理器自己的时钟域**，与 CPU 的 `CLOCK_MONOTONIC` 不是同一个时钟。要把它转换到 CPU 时钟域，需要一个精确的 offset 修正。
- 经实测排查（并交叉核对 NVIDIA 开发者论坛上多个 JetPack 6.x 用户的类似报告），目前公开的 offset 计算方法（基于 `cntvct_el0`/`cntfrq_el0` 与 `CLOCK_MONOTONIC_RAW` 做差）**不可靠**：offset 数值本身会持续漂移，换算结果与真实 `CLOCK_MONOTONIC_RAW` 相差可达数秒。这不是本项目代码的 bug，是 JetPack 6.x 上一个尚未被 NVIDIA 官方彻底解决的已知问题。
- 因此改为：发布端（camera_node）和订阅端（benchmark_subscriber）全程使用**同一个显式声明的时钟**（`RCL_STEADY_TIME`），不跨时钟域，不需要任何 offset 修正。

**代价**：这样测出的延迟，是从"DQBUF 返回、软件感知到这一帧存在"算起，**不包含**摄像头曝光、读出、RTCPU 处理、驱动通知这段固定的硬件管线延迟（具体数值取决于传感器/驱动，但对同一颗摄像头基本恒定）。

**为什么这个代价可以接受**：这段固定硬件延迟，在 baseline 和以后的 NITROS/零拷贝对比实验里都同等存在。做 A/B 差值对比时，这段延迟会自动抵消——而"零拷贝到底省了多少延迟"这个差值，才是本项目真正要回答的问题，不是绝对时间戳精度。

> 如果以后需要真正意义上"从摄像头曝光那一刻"算起的绝对延迟（例如做 IMU/激光雷达传感器融合），这是一个更深的问题，目前没有可靠解法，需要单独跟踪 NVIDIA 官方进展，不在本 baseline 的范围内。

**为什么选 `RCL_STEADY_TIME` 而不是 `RCL_SYSTEM_TIME`/默认的 `RCL_ROS_TIME`**：单调时钟不受 NTP/chrony 时间同步跳变影响，也不会被 ROS2 的仿真时间（`use_sim_time`）意外劫持，测试前后不需要额外操作（比如手动关闭 NTP）。

### benchmark_subscriber

- 按 `report_every_n`（默认100）帧为一个窗口，窗口结束时输出一行统计，而不是逐帧打印或者只依赖人工盯 `ros2 topic hz`。
- 每个窗口统计：
  - **FPS**（按窗口的实际墙钟/稳时钟时长计算，不是简单的周期平均倒数）
  - **延迟** min/avg/max（`now - header.stamp`，语义见上一节——是"post-capture pipeline latency"，不是绝对端到端延迟）
  - **帧间隔** min/avg/max/**stddev**
  - **带宽**（MB/s，窗口内总字节数 / 窗口时长，对标 `ros2 topic bw`，可交叉核对）
  - **窗口内疑似丢帧数** + **累计疑似丢帧数**
- 可选 CSV 输出（`csv_output` 参数），每个窗口一行，逐窗口 flush，Ctrl+C/崩溃也不会丢失已写入的数据。
- 同样显式使用 `RCL_STEADY_TIME`，与 camera_node 保持时钟域一致（这一点是正确性的前提，两边必须一致，否则会在时间戳相减时直接抛 `std::runtime_error`——rclcpp 对 clock_type 不一致的比较/减法操作会主动检查并报错，不会静默给出错误数字）。

**丢帧检测是启发式的，不是精确计数**：ROS2 的 `sensor_msgs/Image` 没有序列号字段（不同于 ROS1 的 `header.seq`），本节点只能靠"帧间隔超过1.5倍目标周期"推测丢帧，不能给出精确的"丢了第几帧"。如需精确计数，需要在 camera_node 里额外维护一个自增序号（比如塞进 `header.frame_id`），本节点解析比对——当前未实现。

## 编译

把整个包放进 workspace：
```bash
cp -r adv_gmsl_camera_baseline cd ${ISAAC_ROS_WS}
cd ${ISAAC_ROS_WS}
colcon build --packages-select adv_gmsl_camera_baseline --symlink-install
source install/setup.bash
```

## 运行方式一：Launch 文件（推荐，一次起两个节点）

```bash
ros2 launch adv_gmsl_camera_baseline baseline.launch.py 
# 或者
ros2 launch adv_gmsl_camera_baseline baseline.launch.py \
  device:=/dev/video2 \
  width:=1920 \
  height:=1080 \
  fps:=30 \
  buffer_count:=4 \
  topic:=/camera/image_raw \
  output:=/workspaces/isaac_ros-dev/baseline_results
```

**`output` 参数务必指向容器内挂载目录**（比如 `/workspaces/isaac_ros-dev/...`），**不要用默认的 `/tmp`**——`run_dev.sh` 启动的容器带 `--rm`，容器退出后 `/tmp` 下的文件会连同容器一起消失，只有挂载目录（对应宿主机 `~/adv_robot`）里的文件才会保留。CSV 会写到 `<output>/camera_benchmark.csv`。

launch 文件里 `camera_node` 的 `output` 目前是注释掉的（`#output='log'`），意味着它的日志只写文件、不会实时打印在终端；`benchmark_subscriber` 显式设了 `output='screen'`，会实时打印。如果需要实时看 camera_node 的 `frames captured=X published=Y` 用于三方计数交叉核对（见下一节），取消这行注释，或者测试结束后去日志文件里翻。

## 运行方式二：手动两个终端（调试时更直观）

**终端1：**
```bash
ros2 run adv_gmsl_camera_baseline camera_node \
  --ros-args -p device:=/dev/video2 -p width:=1920 -p height:=1080 -p fps:=30 -p buffer_count:=4 -p topic:=camera/image_raw
```

**终端2**（`docker exec -it <容器名> bash` 进同一个容器后）：
```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 run adv_gmsl_camera_baseline benchmark_subscriber \
  --ros-args -p topic:=camera/image_raw -p target_fps:=30.0 -p report_every_n:=100
```

## 记录一份完整 baseline 快照

`benchmark_subscriber` 只覆盖 ROS2 消息层面的指标，**不能替代**以下几类独立工具，做一次完整的 baseline 记录时需要都跑一遍：

| 工具 | 覆盖的数据 | 备注 |
|---|---|---|
| `ros2 topic hz <topic> --window 200` | 带 std dev 的精确 hz | 与本节点自己算出的 fps/stddev 交叉核对 |
| `ros2 topic bw <topic>` | 消息带宽 | 与本节点的 bandwidth_MBps 交叉核对 |
| `tegrastats` | CPU/GPU/内存/功耗/温度 | 系统级工具，跟ROS2消息无关，需要单独、持续采样 |
| `sudo nvpmodel -q` / `sudo jetson_clocks --show` | DVFS锁频状态 | 测试**前**确认，避免频率漂移污染数据；本节点无法验证或控制这一项 |

## 三方计数交叉验证

跑完一段时间后，把以下三个数字放在一起看，才能真正确认没有隐藏丢帧：

| 来源 | 指标 | 位置 |
|---|---|---|
| V4L2 层 | captured / dropped | `v4l2-ctl --stream-mmap --stream-count=N --verbose`（单独测，不跟ROS2一起跑） |
| camera_node | captured / published | 节点日志（每100帧打印一次，需要`output='screen'`或翻日志文件） |
| benchmark_subscriber | cumulative_received / cumulative_suspected_drops | 节点日志/CSV最终汇总 |

理想情况下 `captured ≈ published ≈ received`，且都接近"目标FPS × 测试时长"。如果对不上，按上表顺序排查：V4L2层本身丢帧（硬件/驱动问题）→ camera_node发布失败（不应发生）→ DDS层丢消息（`BEST_EFFORT` QoS下的正常代价，需要记录但不是bug）。

## CSV 字段说明

```
wall_time,window_frame_count,avg_fps,
latency_min_ms,latency_avg_ms,latency_max_ms,
period_min_ms,period_avg_ms,period_max_ms,period_stddev_ms,
bandwidth_MBps,suspected_drops_in_window,
cumulative_received,cumulative_suspected_drops
```

`latency_*` 三列是"post-capture pipeline latency"，含义见前面"时间戳"一节，**不是**绝对端到端延迟，报告/展示这份数据时请保留这个限定语，避免被误读。

## 已知限制

1. **丢帧检测是启发式的**，见上文说明，不是精确计数。
2. **延迟是相对管线延迟，不含摄像头硬件捕获延迟**，见"时间戳"一节，但这不影响 baseline vs 加速版的 A/B 对比有效性。
3. **DVFS/功耗模式需要手动锁定**：测试前执行 `sudo nvpmodel -m 0 && sudo jetson_clocks`，两个节点都不会替你做这件事。
4. **容器内 `--rm` 导致的数据丢失风险**：CSV/日志务必输出到挂载目录（`~/adv_robot` 对应的容器内路径），不要用容器内非挂载路径（如默认的 `/tmp`），否则容器一关测试数据就没了。
