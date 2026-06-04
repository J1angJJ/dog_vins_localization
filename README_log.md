# dog_vins_localization 检查日志

日期：2026-06-01

目标：检查新放入 `vins_ws` 的 VINS-Fusion 关键包，参考 `R:\Temp\comp2026_ws` 中机器狗感知主机工程背景，确认实机上可供 VINS 使用的硬件话题、坐标系和关键约束。

更新：2026-06-01 12:25 重新拉取后复查，VINS-Fusion 核心源码已补齐。

## 1. 本仓库检查

仓库根目录当前包含：

```text
LICENSE
README.md
.editorconfig
.gitattributes
.gitignore
vins_ws/
```

`vins_ws` 当前结构：

```text
vins_ws/src/
  LICENCE
  README.md
  camera_models/
  config/
  global_fusion/
  loop_fusion/
  vins_estimator/
```

识别到的 ROS 包：

```text
camera_models
global_fusion
loop_fusion
vins
```

重要发现：

- 重新拉取后，`vins_estimator/src`、`loop_fusion/src`、`global_fusion/src`、`camera_models/src`、`camera_models/include` 均已存在。
- `vins_estimator/CMakeLists.txt` 引用的 `src/estimator/parameters.cpp`、`src/rosNodeTest.cpp`、`src/KITTIOdomTest.cpp` 等源码文件已经存在。
- `loop_fusion` 的 DBoW/DVision/DUtils 第三方源码、`global_fusion` 的 GeographicLib 第三方源码也已存在。
- 从文件完整性看，当前 `vins_ws` 已具备源码层面的编译条件。
- 真正能否在机器狗感知主机上编译，还取决于 ROS Noetic、OpenCV、Ceres、Boost、Eigen、cv_bridge、image_transport、roslib 等依赖是否安装并兼容。

本次没有修改 `vins_ws` 内部文件。

## 2. comp2026_ws 背景摘要

参考工程路径：

```text
R:\Temp\comp2026_ws
```

感知主机背景：

```text
系统：Ubuntu 20.04
ROS：Noetic
实机工程路径：~/comp2026_ws
主要职责：D435i、导航、识别、rosbag、上位任务逻辑
```

运动主机背景：

```text
系统：Ubuntu 20.04.5 LTS
硬件：RK3588
底层：QNX 运动控制系统
感知主机访问地址：192.168.1.120
```

RealSense D435i 记录：

```text
用途：导航深度图、depthimage_to_laserscan、RGB 识别、rosbag 复盘
安装：原装支架
相机主光轴相对水平地面 pitch：约 -18.82 deg
相机离地高度：约 41.5 cm
```

当前 D435i ROS profile 记录：

```text
RGB CameraInfo:
  topic: /camera/color/camera_info
  resolution: 1280x720
  hfov_deg: 70.399133
  vfov_deg: 43.290963

Depth CameraInfo:
  topic: /camera/depth/camera_info
  resolution: 640x480
  hfov_deg: 78.971242
  vfov_deg: 63.426843
```

导航链路当前按 D435i 深度 3.0m 作为较可靠使用上限。

## 3. 与 VINS 相关的现有硬件话题

### 3.1 相机话题

`allmovebase/launch/camera.launch` 使用 `realsense2_camera/rs_camera.launch` 启动 D435i。默认参数：

```text
align_depth=false
enable_pointcloud=false
enable_color=false
enable_depth=true
color_width=640
color_height=480
color_fps=5
depth_width=640
depth_height=480
depth_fps=15
publish_tf=false
publish_odom_tf=false
```

已记录/使用的话题：

```text
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_rect_raw
/camera/depth/camera_info
/camera/depth/color/points
```

注意：

- VINS-Fusion 通常需要灰度/彩色图像输入，单目+IMU 可优先考虑 `/camera/color/image_raw`。
- 当前导航默认不开彩色流；试跑 VINS 时需要显式启动彩色流，例如沿用 `camera_meter_only.launch` 或给 `camera.launch` 传 `enable_color:=true`。
- D435i 是滚动快门 RGB，相比全局快门相机更容易受运动模糊和时序误差影响；初次试跑建议低速、短距离、录包复盘。

VINS-Fusion 自带 `config/realsense_d435i` 示例配置：

```text
imu_topic: /camera/imu
image0_topic: /camera/infra1/image_rect_raw
image1_topic: /camera/infra2/image_rect_raw
num_of_cam: 2
image_width: 640
image_height: 480
```

该示例更偏向 D435i 双红外 + RealSense 自带 IMU；与当前比赛工程已确认链路不同。当前 `comp2026_ws` 里更确定的是 D435i RGB/Depth 和运动主机回传 `/imu/data`。因此 `config/realsense_d435i/realsense_stereo_imu_config.yaml` 不能直接当作机器狗正式配置使用。

### 3.2 IMU 话题

`message_transformer/src/qnx2ros.cpp` 中存在 IMU 发布：

```text
/imu/data
```

相关细节：

```text
消息类型：sensor_msgs/Imu
frame_id：base_link
来源：运动主机/QNX 回传数据，经 qnx2ros 转为 ROS
```

`allmovebase/launch/throttle_imu.launch` 会把 IMU 节流：

```text
/imu/data -> /imu/data_throttled
频率：30 Hz
```

现有 EKF 使用：

```text
imu0: /imu/data_throttled
imu0_remove_gravitational_acceleration: true
```

对 VINS 的意义：

- VINS-Fusion 单目+IMU 通常可先尝试订阅 `/imu/data`，避免节流后的 30 Hz 过低。
- 需要实机确认 `/imu/data` 原始频率、时间戳稳定性、角速度/加速度单位，以及 `frame_id=base_link` 与相机坐标的外参关系。

### 3.3 里程计与融合定位话题

运动主机桥接发布：

```text
/leg_odom
/leg_odom2
```

`allmovebase/config/odom_add_frame.yaml`：

```text
sub_topic: /leg_odom2
pub_topic: /odom_add_frame
frame_id: odom
child_frame_id: base_link
```

EKF 链路：

```text
/leg_odom2 -> /odom_add_frame -> /odom_reset -> /odometry/filtered
```

现有导航使用：

```text
AMCL odom remap: /odometry/filtered
TF: odom -> base_link 由 robot_localization 发布
```

对 VINS 的意义：

- VINS 本体不一定需要腿部里程计，但 `/leg_odom2`、`/odometry/filtered` 可作为对照评估 VIO 漂移。
- 不建议初次把 VINS 结果直接接入导航闭环；先只旁路运行和录包对比。

### 3.4 TF 与坐标系

现有相机静态 TF：

```text
base_link -> camera_link
```

配置文件：

```text
src/allmovebase/config/camera2base_tf.yaml
src/allmovebase/config/camera2base_tf_tilted.yaml
src/allmovebase/config/camera2base_tf_bracket.yaml
```

发布方式：

```text
allmovebase/launch/camera2base_tf.launch
allmovebase/scripts/camera_static_tf_from_yaml.py
```

注意：

- `camera.launch` 中 `publish_tf=false`，相机 TF 由工程自定义节点发布。
- VINS 需要的 `body_T_cam` 不能直接等同于当前导航 TF，必须确认 VINS 的 IMU/body 坐标定义、相机 optical frame 定义和 RealSense 实际图像 frame。
- 当前文档记录 D435i 俯仰约 -18.82 deg，这是 VINS 外参初始化的重要线索，但仍需要标定或至少实机验证。

## 4. 推荐的首次试跑输入组合

优先级 1：旁路单目+IMU，不接导航闭环。

```text
image_topic: /camera/color/image_raw
imu_topic: /imu/data
```

需要同步检查：

```bash
rostopic hz /camera/color/image_raw
rostopic hz /camera/color/camera_info
rostopic hz /imu/data
rostopic echo -n 1 /imu/data
rosrun tf tf_echo base_link camera_link
```

优先级 2：录包离线调配置。

建议录制：

```text
/camera/color/image_raw
/camera/color/camera_info
/imu/data
/tf
/tf_static
/leg_odom2
/odometry/filtered
```

可额外记录导航对照：

```text
/scan
/amcl_pose
/cmd_vel
/move_base/status
```

## 5. 上机前风险与待办

0. 已新增独立 bringup 包。

为避免影响 `comp2026_ws` 和 VINS-Fusion 上游源码，新增：

```text
vins_ws/src/dog_vins_bringup/
  README.md
  TEST_GUIDE.md
  config/dog_mono_d435i_internal_imu_config.yaml
  config/dog_mono_imu_config.yaml
  config/dog_color_pinhole_1280x720.yaml
  launch/dog_standalone_d435i_vins.launch
  launch/dog_realsense_d435i_color_imu.launch
  launch/dog_mono_imu_passive.launch
```

默认入口 `dog_standalone_d435i_vins.launch` 会在本仓库内独立启动 D435i 彩色流、D435i 内置 IMU 和 VINS，不启动 `comp2026_ws`。VINS 输出放在 `/dog_vins/...` 命名空间，结果文件写到 `/tmp/dog_vins_output`。配置由 VINS YAML 文件直接读取，不加载到全局 ROS 参数树。

1. 编译验证。

重新拉取后源码已补齐。下一步需要在感知主机 Ubuntu/ROS Noetic 环境中执行：

```bash
cd ~/dog_vins_localization/vins_ws
catkin_make
source devel/setup.bash
rospack find vins
rospack find dog_vins_bringup
```

2. 准备机器狗专用 VINS 配置。

至少需要：

```text
image_topic
imu_topic
camera model / intrinsics
distortion parameters
body_T_cam / extrinsicRotation / extrinsicTranslation
time offset 设置
IMU 噪声参数
```

当前可参考但不能照搬：

```text
vins_ws/src/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

当前新增的试跑配置：

```text
vins_ws/src/dog_vins_bringup/config/dog_mono_imu_config.yaml
```

独立启动建议：

```bash
source ~/dog_vins_localization/vins_ws/devel/setup.bash
roslaunch dog_vins_bringup dog_standalone_d435i_vins.launch
```

如果确实要使用运动主机 `/imu/data`，可以只启动外部传感器/桥接后使用被动模式：

```bash
source ~/dog_vins_localization/vins_ws/devel/setup.bash
roslaunch dog_vins_bringup dog_mono_imu_passive.launch
```

若继续使用 `comp2026_ws` 默认 `camera.launch` 的 `640x480` 彩色 profile，必须重新测量 `/camera/color/camera_info` 并新增对应相机内参文件，不能复用当前 `1280x720` 内参。

3. 确认 D435i 彩色流是否适合 VINS。

当前工程主要把 D435i 用于深度避障和 RGB 识别。D435i RGB 为滚动快门，运动中视觉惯性估计可能不稳；建议先慢速试跑，并用 rosbag 离线调参。

4. 确认 IMU 来源和坐标。

`/imu/data` 来自运动主机，不是 D435i 内置 IMU。其 `frame_id` 当前为 `base_link`，需要确认坐标轴方向、单位、频率和时间戳是否满足 VINS-Fusion。

5. 不要一开始改现有导航主链路。

首次建议只启动相机、运动主机桥接和 VINS，观察 VINS odometry 与 `/leg_odom2`、`/odometry/filtered` 的相对趋势。确认稳定后再考虑是否把 VINS 输出纳入 `robot_localization` 或导航。

## 6. 本次操作记录

- 读取了本仓库根目录和 `vins_ws/src` 结构。
- 读取了 `R:\Temp\comp2026_ws` 的物理配置、测试手册和相关 launch/config/source 文件。
- 重点确认了 D435i、IMU、腿部里程计、EKF、TF、rosbag 相关话题。
- 新建本文档记录检查过程与结论。
- 2026-06-01 12:25 复查确认 VINS-Fusion 源码已补齐，并记录自带 D435i 示例配置与机器狗现有话题链路的差异。
- 新增 `dog_vins_bringup` 独立配置包，用于旁路订阅 `/camera/color/image_raw` 和 `/imu/data`，避免修改 `comp2026_ws`。
- 新增 `dog_vins_bringup/TEST_GUIDE.md`，记录独立启动、话题检查、录包复盘和标定项。
- 2026-06-01 22:07 日志显示 D435i 驱动已启动 IMU publisher，但随后出现 `Motion Module failure`，VINS 持续 `wait for imu ...`，判断为 `/camera/imu` 无有效消息。
- 新增 D435i 双目红外 stereo-only 备用链路：`dog_standalone_d435i_stereo.launch`、`dog_realsense_d435i_stereo.launch`、`dog_d435i_stereo_config.yaml` 和左右红外 640x480 内参文件。该链路禁用 gyro/accel，用于绕开 D435i Motion Module 问题并继续验证 VINS 主体。
- 2026-06-01 双目备用链路已在机器狗端确认三个核心 topic 有信息；新增 `initial_reset` 启动参数和静态检查说明，便于不移动机器狗时验证图像、特征跟踪和 VINS 输出状态。
- 2026-06-02 增加松耦合外部融合首版：`vins_odom_adapter.py` 规范 VINS odometry frame 并清零首帧，`dog_external_fusion.launch` 使用 `robot_localization` 融合 `/dog_vins/odom_base_link`、`/leg_odom2`、`/imu/data`，默认输出 `/dog_vins_fusion/odometry/filtered` 且不发布 TF。
- 2026-06-02 同步机器狗环境更新：感知主机为 Jetson Xavier NX / JetPack 5.1.2 / L4T 35.4.1，ROS Noetic，librealsense2 2.50.0，系统 OpenCV 4.2.0，Python cv2 4.9.0，Ceres 1.14.0；`robot_localization` 已存在。根据 `realsense-mod.txt` 将双目红外 640x480 内参更新为 fx/fy=388.389587、cx=325.711243、cy=239.344116，并将 stereo-only 初始基线设为 50.002 mm。
- 2026-06-02 实机测试发现 stereo-only 的 `/dog_vins/vins_estimator/image_track` 无输出，确认原因是 `show_track: 0`；已将 `dog_d435i_stereo_config.yaml` 改为 `show_track: 1`，用于首轮调试观察特征跟踪。
- 2026-06-02 实机录包 30 秒约 5.8GB 且触发 rosbag buffer 压力；测试手册新增轻量尺度测试包和 10 秒图像调试包命令，默认建议尺度测试不录 raw 图像和 `image_track`。
- 2026-06-03 增加 VINS 后端紧耦合工程原型：新增 `LegOdomFactor`，订阅 `/leg_odom2`，在 VINS 滑窗相邻帧之间加入腿部里程计相对约束；默认因子模式为 translation distance + yaw，避免未标定 base/camera 轴系时把 dx/dy 硬绑错。新增入口 `dog_standalone_d435i_stereo_leg_odom.launch` 和配置 `dog_d435i_stereo_leg_odom_config.yaml`。
- 2026-06-03 实机编译通过后确认：`dog_standalone_d435i_stereo_leg_odom.launch` 本身不发布 `/leg_odom2`，该话题来自机器狗 UDP 状态桥接。已在紧耦合 launch 中发布已知 tilted mount `base_link -> camera_link` 静态 TF，并新增本仓库内只读桥接包 `dog_robot_bridge`，`dog_standalone_d435i_stereo_leg_odom_with_bridge.launch` 不再依赖 `comp2026_ws/message_transformer`。
- 2026-06-03 新增定位调试 RViz：`config/dog_vins_localization.rviz`，显示 VINS path/odometry/point cloud、`/leg_odom2`、TF、`image_track`。紧耦合 launch 增加 `rviz_config`、`publish_world_odom_tf` 参数，默认发布 identity `world -> odom` 仅用于 RViz 对比显示。
- 2026-06-03 根据实机 log `20260603-2248.txt` 确认“开 RViz 导致主链路断开”的原因是重复启动同一主 launch，ROS 因同名节点注册关闭旧节点。新增 `dog_vins_rviz.launch` 作为 RViz-only 入口，主链路已运行时只打开 RViz，不重复启动桥接、相机或 VINS。
- 2026-06-04 将 RViz 从机器狗定位仓库迁移到开发机仓库 `dog_dev_station`：机器狗端 launch 不再包含 RViz 节点，`dog_dev_station/rviz_ws/src/dog_dev_rviz` 提供远程 RViz 配置和 `dog_vins_remote_rviz.launch`。机器狗端保持 headless，只运行桥接、相机和 VINS。
- 未修改 VINS-Fusion 上游核心包；新增和调整均位于 `dog_vins_bringup` 适配包及本文档。
