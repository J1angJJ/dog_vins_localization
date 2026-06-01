# dog_vins_bringup

Standalone bringup files for trying VINS-Fusion on the quadruped perception host.

## Modes

Standalone D435i internal IMU mode:

```bash
roslaunch dog_vins_bringup dog_standalone_d435i_vins.launch
```

This starts RealSense color plus D435i internal IMU and then runs VINS in the
`/dog_vins` namespace. It does not start `comp2026_ws`.

Passive external IMU mode:

```bash
roslaunch dog_vins_bringup dog_mono_imu_passive.launch
```

This only runs VINS and expects another launch to provide `/camera/color/image_raw`
and `/imu/data`.

## Notes

- Parameters are file-based VINS YAML configs in this package, not global ROS params.
- Runtime output is written to `/tmp/dog_vins_output`.
- The current extrinsics and IMU noise values are first-run guesses. Replace them after calibration.
- Test procedure and calibration notes are in `TEST_GUIDE.md`.
