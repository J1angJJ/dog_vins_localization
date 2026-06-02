# dog_vins_bringup

Standalone bringup files for trying VINS-Fusion on the quadruped perception host.

## Modes

Standalone D435i internal IMU mode:

```bash
roslaunch dog_vins_bringup dog_standalone_d435i_vins.launch
```

This starts RealSense color plus D435i internal IMU and then runs VINS in the
`/dog_vins` namespace. It does not start `comp2026_ws`.
Default IMU profiles are `gyro_fps:=200` and `accel_fps:=100`.

Standalone D435i stereo fallback mode:

```bash
roslaunch dog_vins_bringup dog_standalone_d435i_stereo.launch
```

This starts D435i `infra1` and `infra2` at `640x480 @ 30 Hz`, disables gyro and
accel, and runs VINS in stereo-only mode. Use it when `/camera/imu` is missing
or the RealSense log reports `Motion Module failure`.
Add `initial_reset:=true` when the camera was just interrupted or left in a bad
runtime state.

Passive external IMU mode:

```bash
roslaunch dog_vins_bringup dog_mono_imu_passive.launch
```

This only runs VINS and expects another launch to provide `/camera/color/image_raw`
and `/imu/data`.

Loose external fusion mode:

```bash
roslaunch dog_vins_bringup dog_external_fusion.launch
```

This runs a separate `robot_localization` EKF using normalized VINS odometry,
`/leg_odom2`, and `/imu/data`. It publishes under `/dog_vins_fusion` and does
not publish TF by default, so it can be tested alongside the existing navigation
stack.

## Notes

- Parameters are file-based VINS YAML configs in this package, not global ROS params.
- Runtime output is written to `/tmp/dog_vins_output`.
- The current extrinsics and IMU noise values are first-run guesses. Replace them after calibration.
- Test procedure and calibration notes are in `TEST_GUIDE.md`.
