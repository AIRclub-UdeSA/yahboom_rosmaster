# Motion profiles and ground truth

The simulator exposes two independent pose sources:

- `/odom` is encoder-style mecanum odometry integrated from simulated wheel
  joint positions. It owns `odom -> base_footprint` on TF.
- `/ground_truth/odom` is the Gazebo model's actual world pose, published as
  `nav_msgs/msg/Odometry` at 50 Hz with simulation timestamps. It is for tests,
  analysis, and bags only; it does not publish TF and must not be used by the
  robot-facing EKF or navigation stack.

## Selecting a profile

The simulator defaults to the deterministic `stress` profile:

```bash
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py
```

Use the former zero-slip behavior explicitly when a test needs an ideal
baseline:

```bash
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py \
  motion_profile:=ideal
```

The combined navigation launch accepts the same argument. Profile values are
stored in `config/motion_profiles.yaml` and applied when the robot xacro is
expanded. Unsupported profile names fail launch instead of silently falling
back.

## Current profiles

| Parameter | Ideal | Stress |
| --- | ---: | ---: |
| Grip-axis friction `mu` | 1.0 | 0.8 |
| Roller-axis friction `mu2` | 0.0 | 0.03 |
| Roller-axis compliance `slip2` | 0.0 | 0.002 |
| Front-left grip compliance `slip1` | 0.0 | 0.014 |
| Front-right grip compliance `slip1` | 0.0 | 0.008 |
| Back-left grip compliance `slip1` | 0.0 | 0.012 |
| Back-right grip compliance `slip1` | 0.0 | 0.010 |

The stress values reduce diagonal grip, add roller resistance, and deliberately
make the four wheel contacts unequal. They are a controlled mechanism for
exercising error-sensitive software, not measured ROSMASTER X3 parameters.

## Runtime baseline

On 2026-07-22, three sequential repetitions of 11 three-second motion cases in
the empty world produced the following settled endpoint differences between
wheel odometry and ground truth:

| Group | Samples | Mean translation error | Maximum translation error | Maximum yaw error |
| --- | ---: | ---: | ---: | ---: |
| Differential-compatible commands (`linear.y=0`) | 18 | 1.42 mm | 2.11 mm | 0.00224 rad |
| Holonomic commands | 15 | 6.68 mm | 7.54 mm | 0.00125 rad |
| All cases | 33 | 3.81 mm | 7.54 mm | 0.00224 rad |

The ideal profile previously produced a maximum translation difference of
about 0.0904 mm over the same matrix. The stress profile therefore creates a
clear, repeatable divergence while preserving forward, reverse, rotation,
arc, strafe, diagonal, and mixed motion.

These are sequential simulator trials, not independent physical calibration
runs. Contact values remain deterministic, and the native MecanumDrive still
uses ideal wheel-velocity targets.

## Calibration path

Do not rename `stress` to `calibrated` or add a calibrated default merely from
visual judgment. First capture synchronized real-robot `/cmd_vel`, wheel or
encoder feedback, odometry, IMU, and independent external pose for repeated:

- forward and reverse motion at several speeds;
- left and right strafe;
- clockwise and counterclockwise rotation;
- arcs, diagonals, and mixed x/y/yaw commands;
- acceleration, stop, reversal, payload, and representative floor cases.

Fit deterministic contact values against mean longitudinal, cross-track, yaw,
and stop errors, then validate them on held-out trials. Encoder quantization,
wheel-radius error, motor saturation, latency, random variation, and changing
floor surfaces should remain separately configurable layers. Add a
`calibrated` profile only when its source data, robot configuration, surface,
date, fitting method, and acceptance bounds are recorded.
