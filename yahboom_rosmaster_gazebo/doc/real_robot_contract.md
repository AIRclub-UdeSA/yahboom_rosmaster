# Real-robot interface contract

`config/real_robot_contract.yaml` records the ROSMASTER X3's real ROS 2
interface as observed in `bags_x3.zip`: 5 Humble sqlite3 bags, about 62
minutes and 425,513 messages total. The archive itself is untracked (see
`.gitignore`) and is never extracted into the repository; the contract file
is the durable, reviewable artifact derived from it.

## Regenerating the measurements

`scripts/real_robot_bag_analyzer.py` reads bags directly with `rosbag2_py`
and never requires full playback:

```bash
mkdir -p ~/rosbags_x3
unzip bags_x3.zip 'bags_x3/bag_x3_6/*' -d ~/rosbags_x3
# ...repeat for the bags you want to include

ros2 run yahboom_rosmaster_gazebo real_robot_bag_analyzer.py \
  ~/rosbags_x3/bags_x3/bag_x3_6 \
  ~/rosbags_x3/bags_x3/bag_x3_7 \
  ~/rosbags_x3/bags_x3/bag_x3_8_cones \
  ~/rosbags_x3/bags_x3/bag_x3_9_obstacles \
  --output /tmp/bag_report.yaml
```

The analyzer emits per-topic rate, frame, and noise statistics plus the
recorded URDF joint geometry for each bag it is given. It intentionally
skips `/image_raw` payload bytes (about 93% of on-disk size) and instead
reads `/camera_info` for resolution/rate and samples only the first few
`/image_raw` messages for encoding.

`bag_x3_5` is excluded from every motion statistic: its odometry path length
is exactly zero and its IMU is frozen for the whole recording while
`/cmd_vel` and `/scan` keep changing, so it cannot represent real motion.

## What the contract is not

The bags contain no independent ground truth (no map, world, mocap,
tracking-camera, GPS, or AprilTag-pose topic), and `/joint_states` positions
are always zero with velocity never reported. The contract therefore
describes the real robot's *recorded ROS interface* — topic names, frames,
rates, sensor geometry, and the estimator's reported command-response
behavior — not a physical wheel-slip or encoder calibration. See
`calibration_gaps` in the contract file for what a future controlled capture
would need to add.

## Confidence levels

Each contract section is tagged `measured` (computed by the analyzer and
cross-checked against the original manual audit), `manual_audit` (from the
2026-07-26 manual audit, not yet reproduced by the analyzer script — for
example the stable-command-window gain table), `inferred`, or `absent`.
Prefer promoting `manual_audit` sections into analyzer output over silently
copying new numbers into the YAML by hand.
