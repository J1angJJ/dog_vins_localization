# dog_vins_bringup 测试手册

本文记录 `dog_vins_localization` 中 VINS-Fusion 独立试跑链路的命令、用途和需要标定的内容。

## 1. 链路目标

当前默认测试链路是独立模式：

```text
D435i color image  -> /camera/color/image_raw -> VINS
D435i internal IMU -> /camera/imu             -> VINS
VINS output        -> /dog_vins/vins_estimator/odometry
                   -> /dog_vins/vins_estimator/path
                   -> /dog_vins/vins_estimator/imu_propagate
```

该链路不启动 `comp2026_ws`，不启动导航，不发布底盘速度，也不修改全局参数。VINS 参数由本包 YAML 文件直接读取：

```text
vins_ws/src/dog_vins_bringup/config/dog_mono_d435i_internal_imu_config.yaml
```

如果 D435i 内置 IMU 报 `Motion Module failure` 或 `/camera/imu` 长时间无消息，可切换到双目红外备用链路：

```text
D435i infra1 image -> /camera/infra1/image_rect_raw -> VINS
D435i infra2 image -> /camera/infra2/image_rect_raw -> VINS
IMU disabled
```

对应配置：

```text
vins_ws/src/dog_vins_bringup/config/dog_d435i_stereo_config.yaml
```

## 2. 编译与环境

进入 VINS 工作区：

```bash
cd ~/dog_vins_localization/vins_ws
```

用途：进入当前仓库的 ROS 工作区，不进入 `~/comp2026_ws`，避免两个工程混在一起。

编译：

```bash
catkin_make
```

用途：编译 VINS-Fusion、`camera_models`、`dog_vins_bringup` 等包。

加载环境：

```bash
source devel/setup.bash
```

用途：让当前终端能找到 `vins`、`dog_vins_bringup` 等包。

确认包存在：

```bash
rospack find vins
rospack find dog_vins_bringup
```

用途：确认当前终端加载的是本工作区环境。

## 3. 独立启动

启动 D435i 彩色流、D435i 内置 IMU 和 VINS：

```bash
roslaunch dog_vins_bringup dog_standalone_d435i_vins.launch
```

用途：一条命令启动完整独立试跑链路。该 launch 会调用：

```text
dog_realsense_d435i_color_imu.launch
vins_node
```

默认相机设置：

```text
color: 1280x720 @ 15 Hz
imu: gyro 200 Hz, accel 100 Hz, united as /camera/imu
depth: disabled
infra1/infra2: disabled
pointcloud: disabled
RealSense TF: disabled
```

如果有多台 RealSense，可指定序列号：

```bash
roslaunch dog_vins_bringup dog_standalone_d435i_vins.launch serial_no:=<D435I_SERIAL>
```

用途：避免 RealSense 驱动打开错设备。

如果要同时打开 RViz：

```bash
roslaunch dog_vins_bringup dog_standalone_d435i_vins.launch start_rviz:=true
```

用途：启动 VINS 自带 RViz 配置，观察轨迹、特征点和路径。

如果 `/camera/imu` 不发布，启动双目红外备用链路：

```bash
roslaunch dog_vins_bringup dog_standalone_d435i_stereo.launch
```

用途：只打开 D435i 左右红外图像并以 VINS stereo-only 模式运行，绕开 D435i Motion Module。

如果 RealSense 刚被异常中断过，可让驱动启动时重置设备：

```bash
roslaunch dog_vins_bringup dog_standalone_d435i_stereo.launch initial_reset:=true
```

用途：只影响本次 launch 内的 RealSense 设备初始化，不修改系统版本或全局参数。

## 4. 基础话题检查

检查彩色图像频率：

```bash
rostopic hz /camera/color/image_raw
```

用途：确认 D435i 彩色图像正在发布。默认期望接近 15 Hz。

检查彩色相机内参：

```bash
rostopic echo -n 1 /camera/color/camera_info
```

用途：确认当前 RealSense profile 的 `width`、`height`、`K`、`D` 是否和配置文件一致。

检查 D435i IMU：

```bash
rostopic hz /camera/imu
rostopic echo -n 1 /camera/imu
```

用途：确认 VINS 的 IMU 输入存在、时间戳在变化、加速度和角速度不是全零。

检查双目红外备用链路：

```bash
rostopic hz /camera/infra1/image_rect_raw
rostopic hz /camera/infra2/image_rect_raw
rostopic echo -n 1 /camera/infra1/camera_info
rostopic echo -n 1 /camera/infra2/camera_info
```

用途：确认 stereo-only 模式的两路图像都接近 30 Hz，且分辨率为 `640x480`。

`/dog_vins/vins_estimator/image_track` 是 VINS 的特征跟踪调试图，只有配置文件中 `show_track: 1` 时才会发布。当前 stereo-only 配置已默认打开：

```text
config/dog_d435i_stereo_config.yaml
show_track: 1
```

检查 VINS 输出：

```bash
rostopic hz /dog_vins/vins_estimator/odometry
rostopic echo -n 1 /dog_vins/vins_estimator/odometry
```

用途：确认 VINS 已经初始化并开始输出里程计。刚启动时可能需要移动一段才有稳定输出。

注意：如果只静止放着，VINS 可能一直没有 `/dog_vins/vins_estimator/odometry` 消息。这不是 topic 不存在，而是尚未完成初始化。先确认输入图像和 `/dog_vins/vins_estimator/image_track`。

检查 VINS 路径：

```bash
rostopic echo -n 1 /dog_vins/vins_estimator/path
```

用途：确认路径消息存在，可用于 RViz 显示。

不移动机器狗时的静态检查：

```bash
rostopic echo -n 1 /dog_vins/vins_estimator/odometry
rostopic echo -n 1 /dog_vins/vins_estimator/path
rostopic hz /dog_vins/vins_estimator/image_track
```

用途：确认 VINS 已经有输出、特征跟踪 topic 在刷新。静止状态下 odometry 可能变化很小，这是正常现象。

查看节点：

```bash
rosnode list
rosnode info /dog_vins/vins_estimator
```

用途：确认 VINS 节点订阅的 image/IMU topic 是否符合配置。

## 5. 推荐首次试跑动作

1. 静止启动 10 秒。

用途：观察 IMU 是否正常、图像是否稳定、VINS 是否报时间戳或图像异常。

2. 手持或推着机器狗低速平移 1 到 2 米。

用途：给 VINS 足够视差初始化，避免原地旋转导致初始化困难。

3. 小角度转向。

用途：检查轨迹方向是否合理，IMU/相机外参是否明显错误。

4. 停下观察漂移。

用途：判断 IMU 噪声和时间同步是否导致静止漂移严重。

## 6. 录包与复盘

创建录包目录：

```bash
mkdir -p ~/bags/dog_vins
```

用途：存放本工程 VINS 测试包，避免混入 `comp2026_ws` 的任务包。

录制最小 VINS 调试包：

```bash
rosbag record -O ~/bags/dog_vins/vins_min.bag \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /camera/imu \
  /dog_vins/vins_estimator/odometry \
  /dog_vins/vins_estimator/path \
  /dog_vins/vins_estimator/image_track
```

用途：记录 VINS 输入和核心输出，便于离线检查初始化、特征跟踪和轨迹。

录制带 TF 的扩展包：

```bash
rosbag record -O ~/bags/dog_vins/vins_debug.bag \
  /tf /tf_static \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /camera/imu \
  /dog_vins/vins_estimator/odometry \
  /dog_vins/vins_estimator/path \
  /dog_vins/vins_estimator/imu_propagate \
  /dog_vins/vins_estimator/image_track \
  /dog_vins/vins_estimator/extrinsic
```

用途：记录更多 VINS 输出，适合排查外参、IMU 传播和轨迹跳变。

轻量尺度测试包：

```bash
rosbag record --lz4 --buffsize=2048 -O ~/bags/dog_vins/straight_1m_scale_light.bag \
  /dog_vins/vins_estimator/odometry \
  /dog_vins/vins_estimator/path \
  /leg_odom2 \
  /imu/data \
  /tf \
  /tf_static
```

用途：只判断 VINS 轨迹尺度、腿部里程计对照和融合输入是否正常，不记录大体积 raw 图像。优先用于 1 米直线、往返和旋转测试。

带图像的短调试包：

```bash
rosbag record --lz4 --buffsize=4096 --duration=10 -O ~/bags/dog_vins/stereo_debug_10s.bag \
  /camera/infra1/image_rect_raw \
  /camera/infra2/image_rect_raw \
  /camera/infra1/camera_info \
  /camera/infra2/camera_info \
  /dog_vins/vins_estimator/image_track \
  /dog_vins/vins_estimator/odometry \
  /dog_vins/vins_estimator/path
```

用途：只在需要排查图像、同步或特征跟踪时短录。不要长时间同时录左右 raw 图像和 `image_track`，否则包体积会快速变大并可能触发 rosbag buffer 溢出。

回放录包：

```bash
rosparam set use_sim_time true
rosbag play --clock ~/bags/dog_vins/vins_debug.bag
```

用途：离线复现测试数据。回放前建议只启动 VINS，不再启动 RealSense。

## 7. 外部融合首版

紧耦合做法是在 VINS 后端滑窗里加入腿部里程计因子，工程量较大。当前工程先提供松耦合首版：

```bash
roslaunch dog_vins_bringup dog_external_fusion.launch
```

输入：

```text
/dog_vins/vins_estimator/odometry -> /dog_vins/odom_base_link
/leg_odom2
/imu/data
```

输出：

```text
/dog_vins_fusion/odometry/filtered
```

用途：在不改 VINS 后端、不接管原导航 `/odometry/filtered` 的情况下，先验证 VINS、腿部里程计和机器狗 IMU 的松耦合效果。

检查：

```bash
rostopic hz /dog_vins/odom_base_link
rostopic hz /leg_odom2
rostopic hz /imu/data
rostopic hz /dog_vins_fusion/odometry/filtered
```

如果融合方向不对，优先调整：

```bash
roslaunch dog_vins_bringup dog_external_fusion.launch yaw_offset:=<rad>
```

当前 `publish_tf:=false`，不会和 `comp2026_ws` 里已有的 `odom -> base_link` TF 抢发布权。确认稳定后再讨论是否接入导航主链路。

## 8. 紧耦合原型

启动双目 VINS，并在 VINS 滑窗优化中加入 `/leg_odom2` 相对约束：

```bash
roslaunch dog_vins_bringup dog_standalone_d435i_stereo_leg_odom.launch
```

注意：这个 launch 会启动 D435i、VINS 和已知的 `base_link -> camera_link`
静态 TF，但不会生成 `/leg_odom2`。`/leg_odom2` 来自机器狗桥接
`message_transformer/qnx2ros`。实机测试有两种方式：

```bash
# 方式 A：终端 1 先启动机器狗桥接
source ~/comp2026_ws/devel/setup.bash
roslaunch message_transformer message_transformer.launch

# 终端 2 再启动 VINS 紧耦合
source ~/dog_vins_localization/vins_ws/devel/setup.bash
roslaunch dog_vins_bringup dog_standalone_d435i_stereo_leg_odom.launch
```

```bash
# 方式 B：如果 comp2026_ws 和 dog_vins_localization/vins_ws 都已 source，
# 用 wrapper 一次拉起桥接、相机和 VINS
source ~/comp2026_ws/devel/setup.bash
source ~/dog_vins_localization/vins_ws/devel/setup.bash
roslaunch dog_vins_bringup dog_standalone_d435i_stereo_leg_odom_with_bridge.launch
```

输入：

```text
/camera/infra1/image_rect_raw
/camera/infra2/image_rect_raw
/leg_odom2
```

当前原型默认使用：

```text
leg_odom_factor_mode: 0
```

含义：只约束相邻帧平移距离和 yaw，不直接约束局部 dx/dy。这样在 `body_T_cam` 尚未严格等同 `base_link -> camera` 外参前，不会把机器狗底盘坐标轴和相机光学坐标轴硬绑错。

检查：

```bash
rostopic hz /leg_odom2
rostopic hz /dog_vins/vins_estimator/odometry
rostopic hz /dog_vins/vins_estimator/path
rosrun tf tf_echo base_link camera_link
```

轻量录包：

```bash
rosbag record --lz4 --buffsize=2048 -O ~/bags/dog_vins/tight_leg_odom_test.bag \
  /dog_vins/vins_estimator/odometry \
  /dog_vins/vins_estimator/path \
  /leg_odom2 \
  /imu/data \
  /tf \
  /tf_static
```

后续若已标定 `base_link` 与 VINS body/camera 的轴向关系，可尝试：

```text
leg_odom_factor_mode: 1
```

含义：约束相邻帧局部 dx、dy 和 yaw，约束更强，但错误外参会明显拉坏轨迹。

## 9. 需要标定或确认的内容

### 9.1 相机内参

当前 RGB 单目文件：

```text
config/dog_color_pinhole_1280x720.yaml
```

当前来源：`comp2026_ws` 文档中记录的 `/camera/color/camera_info`，分辨率为 `1280x720`。

需要确认：

```bash
rostopic echo -n 1 /camera/color/camera_info
```

检查项：

```text
width/height 是否为 1280x720
K[0] 是否等于 fx
K[4] 是否等于 fy
K[2] 是否等于 cx
K[5] 是否等于 cy
D 是否需要写入畸变参数
```

如果分辨率改为 `640x480` 或其他 profile，必须新建对应内参文件，不要复用 `1280x720` 内参。

当前双目红外备用链路文件：

```text
config/dog_d435i_infra_left_640x480.yaml
config/dog_d435i_infra_right_640x480.yaml
```

当前来源：2026-06-02 `rs-enumerate-devices -c` 记录的 Infrared 1/2 `640x480` 内参：

```text
fx/fy: 388.389587
cx: 325.711243
cy: 239.344116
FOV: H 78.96 deg, V 63.43 deg
```

注意：`640x480` 红外横向 FOV 约 `78.96 deg`；如果后续想保留约 `89.34 deg` 横向 FOV，可改用 `848x480` 或 `640x360`，但必须同步新增对应 VINS 内参配置。

### 9.2 相机-IMU 外参

当前文件：

```text
config/dog_mono_d435i_internal_imu_config.yaml
```

当前字段：

```text
body_T_cam0
estimate_extrinsic: 1
```

现状：`body_T_cam0` 是首跑初始猜测，不是正式标定值。`estimate_extrinsic: 1` 表示让 VINS 在线优化外参。

需要标定：

```text
D435i color camera -> D435i internal IMU
```

建议方式：

```text
Kalibr 标定，或用 VINS 多次在线估计结果作参考后再固定。
```

当外参可信后，可改为：

```text
estimate_extrinsic: 0
```

并把标定后的 `body_T_cam0` 写入配置。

### 9.3 时间偏移

当前字段：

```text
estimate_td: 1
td: 0.0
```

用途：让 VINS 在线估计图像时间和 IMU 时间差。

需要确认：

```text
td 是否逐渐稳定
VINS 是否频繁提示时间同步或初始化异常
```

如果多次测试 `td` 稳定，可把 `td` 写成稳定值，并考虑改为：

```text
estimate_td: 0
```

### 9.4 IMU 噪声

当前字段：

```text
acc_n
gyr_n
acc_w
gyr_w
```

现状：这些是示例级占位值。

需要标定：

```text
D435i internal IMU accelerometer / gyroscope noise
accelerometer / gyroscope bias random walk
```

建议：静止录制一段 `/camera/imu`，后续用 Allan variance 工具估计。

### 9.5 里程计对照

当前独立链路不使用机器狗腿部里程计。

如果之后想和原工程对照，需要另行记录：

```text
/leg_odom2
/odometry/filtered
```

用途：只用于对比 VINS 轨迹，不建议首轮直接融合进导航。

## 10. 常见问题

### 没有 `/camera/imu`

检查 RealSense launch 是否启动：

```bash
rosnode list | grep camera
rostopic list | grep camera
```

用途：确认驱动是否发布 IMU 相关话题。

如果 `/camera/gyro/sample`、`/camera/accel/sample` 存在但 `/camera/imu` 不存在，检查 RealSense 驱动是否支持：

```text
unite_imu_method:=linear_interpolation
```

如果 launch 输出 `Motion Module failure`，先重插 D435i USB3 线并重新启动 launch。当前默认使用 `gyro_fps:=200`、`accel_fps:=100`，这是本机日志中 D435i 实际接受的 IMU profile；不要再强制 `accel_fps:=250`。

若多次重启后仍然出现：

```text
Hardware Notification:Motion Module failure
wait for imu ...
```

先切换到双目红外备用链路：

```bash
roslaunch dog_vins_bringup dog_standalone_d435i_stereo.launch
```

该链路不使用 `/camera/imu`，能判断相机图像和 VINS 主体是否正常。后续再单独处理 D435i 内置 IMU 或改接运动主机 `/imu/data`。

### VINS 没有 odometry 输出

检查输入：

```bash
rostopic hz /camera/color/image_raw
rostopic hz /camera/imu
```

用途：确认图像和 IMU 都存在。

然后做低速平移，不要只原地旋转。VINS 初始化需要足够视差。

如果暂时不能移动机器狗，先检查 `/dog_vins/vins_estimator/image_track`。若 image_track 有频率，说明图像输入和特征跟踪已经通；是否能稳定初始化要等下次移动测试。

### 轨迹方向明显不对

优先怀疑：

```text
body_T_cam0 外参方向不对
相机实际 optical frame 与配置假设不一致
IMU 坐标方向不符合 VINS 预期
```

处理：录制 `vins_debug.bag`，检查 `/dog_vins/vins_estimator/extrinsic`，再做正式外参标定。

### 静止漂移严重

优先检查：

```text
IMU 噪声参数
IMU 单位
时间偏移 td
相机图像是否模糊或曝光过低
```

处理：先静止录 `/camera/imu`，再低速短距离测试，不要直接上高速运动。

## 11. 文件速查

```text
launch/dog_standalone_d435i_vins.launch
launch/dog_standalone_d435i_stereo.launch
launch/dog_realsense_d435i_color_imu.launch
launch/dog_realsense_d435i_stereo.launch
launch/dog_mono_imu_passive.launch
launch/dog_external_fusion.launch
launch/dog_standalone_d435i_stereo_leg_odom.launch
launch/dog_standalone_d435i_stereo_leg_odom_with_bridge.launch
config/dog_mono_d435i_internal_imu_config.yaml
config/dog_d435i_stereo_config.yaml
config/dog_d435i_stereo_leg_odom_config.yaml
config/dog_vins_external_fusion.yaml
config/dog_mono_imu_config.yaml
config/dog_color_pinhole_1280x720.yaml
config/dog_d435i_infra_left_640x480.yaml
config/dog_d435i_infra_right_640x480.yaml
README.md
TEST_GUIDE.md
```
