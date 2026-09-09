**安装Iassc ROS 步骤**
# 1. 安装docker & docker-buildx-plugin
Isaac ROS 3.2 需要 Docker Engine 27.2.0 或更新版本。

```
## 1. 卸载旧版本（如果有）
sudo apt-get remove -y docker docker-engine docker.io containerd runc docker-ce docker-ce-cli 2>/dev/null || true

## 2. 安装依赖
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg lsb-release

## 3. 添加阿里云 Docker 源（国内速度快）
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://mirrors.aliyun.com/docker-ce/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://mirrors.aliyun.com/docker-ce/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update

## 4. 安装指定版本 Docker 27.5.1（满足 Isaac ROS 要求）
VERSION="5:27.5.1-1~ubuntu.22.04~jammy"

sudo apt-get install -y \
  docker-ce=$VERSION \
  docker-ce-cli=$VERSION \
  containerd.io \
  docker-buildx-plugin \
  docker-compose-plugin

## 5. 启动 Docker 并设置开机自启
sudo systemctl enable --now docker

## 6. 把当前用户加入 docker 组（免 sudo）
sudo usermod -aG docker $USER
newgrp docker
```

# 2. L4T BSP 安装JetPack 组件
nvidia-jetpack 包含CUDA、cuDNN、TensorRT、VPI、Multimedia API、nvidia-container 等这些 GPU 相关库；
```
sudo apt update
sudo apt install nvidia-jetpack
```

# 3. 其它设置
nvidia-ctk runtime configure
```
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```
运行以下命令，将电源模式设置为最大功耗模式（电源模式控制）。
```
sudo /usr/sbin/nvpmodel -m 0 
```
运行以下命令，将 GPU 和 CPU 时钟频率设置为最高（最大化 Jetson 性能）。
```
sudo /usr/bin/jetson_clocks
```


# 4. NVIDIA Isaac ROS 环境搭建

参考连接https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/index.html#ros-support完成docker 配置；
我们强烈建议您使用 Isaac ROS Dev Docker 镜像来设置开发环境。这将简化您在 Jetson 和 x86_64 平台上的开发环境配置，确保依赖版本正确。在 Isaac ROS Dev Docker 容器中工作会自动设置 ROS，并自动配置 Isaac Apt 仓库。

Isaac ROS Dev Docker的使用参考这两个连接：
https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/dev_env_setup.html 和
https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_common/index.html

## 4.1. 克隆对应 JP6.2 (r36.4.3) 的 3.2 分支
```
export ISAAC_ROS_WS=~/adv_robot
echo 'export ISAAC_ROS_WS=~/adv_robot' >> ~/.bashrc

# 设置git
sudo apt-get install -y git-lfs
git lfs install

mkdir -p ${ISAAC_ROS_WS}/src
cd ${ISAAC_ROS_WS}/src
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
```

## 4.2. 启动Isaac ROS Dev Docker 容器
```
### 生成 CDI 规格文件
sudo nvidia-ctk cdi generate --mode=csv --output=/etc/cdi/nvidia.yaml

cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh
```
***确保环境是完整可用的***
```
# 确认ROS2环境变量已经加载
echo $ROS_DISTRO
# 应该输出: humble

# 确认容器内能看到GPU(前面CDI问题解决的关键验证)
nvidia-smi
# 或者Jetson专属的方式:
tegrastats
```
**注意事项**:**run_dev.sh 启动的开发容器带有 --rm 参数,退出(包括 Ctrl+C)后容器会被自动删除,docker ps -a 中不会保留记录,这是预期行为而非异常。容器内 ~/adv_robot 挂载目录下的代码/数据不受影响(挂载在宿主机上,不随容器销毁而丢失),但任何在容器内通过 apt install 等方式安装的额外软件包、或对挂载目录之外文件的修改,会在容器退出后丢失**。如需长期保留的依赖,请将对应安装步骤写入自定义 Dockerfile 并重新构建镜像,不要依赖手动安装后"容器不关闭"这种方式。


# 5. 创建ROS2 Baseline 工程
```
ros2 pkg create adv_gmsl_camera_baseline \
    --build-type ament_cmake \
    --license MIT \
    --dependencies rclcpp sensor_msgs
```
     
Setup Source 章节：
```
sudo curl -sSL https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```


## 1.4 配置跨内存零拷贝与环境
在 ~/.bashrc 中追加以下环境配置，确保 FastDDS / CycloneDDS 能够使用 NVMM（NVIDIA Memory Management）内存共享：
```
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds/shm_profile.xml # 使能 Shared Memory
```
