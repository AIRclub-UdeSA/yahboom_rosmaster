# Real-robot parity ledger

`config/real_robot_contract.yaml` is the single place where the simulator is
compared with the physical ROSMASTER X3. It records what the robot delivers,
what the simulator delivers today, and which step of the parity plan
(#43) closes each difference. The contract probes and the description contract
read their expected values from it, so a number changes in one place.

## The three sections

| Section | Holds | Comes from |
|---|---|---|
| `physical` | The robot's measured and configured interface: topics, rates, latency, frames and mounts, camera intrinsics, depth quality, point-cloud layout, LiDAR, IMU, odometry, command limits | [physical_rosmaster](https://github.com/AIRclub-UdeSA/physical_rosmaster) at the commit in `physical.provenance` |
| `simulator` | One parity entry per physical value the simulator can be compared on | The xacro and configuration on `main` (`nominal`), and the step-2 baseline measurement (`measured`) |
| `legacy_bag_audit` | The bag-derived contract this file replaced | `bags_x3.zip`, audited 2026-07-26 |

Every physical group carries a `confidence`: `measured` (observed on the
robot by a reproducible tool), `manual_audit` (a hand measurement, such as the
tape-measured camera height), `inferred` (read from physical_rosmaster source
or configuration, not observed on the wire), or `absent`.

### Parity entries

The `simulator` section mirrors the `physical` keys. `simulator.camera.width`
is compared with `physical.camera.width`, and so on. Each entry is a mapping:

```yaml
rate_hz:
  nominal: 5.0            # configured on main
  contract: [4.5, 5.5]    # what sensor_contract_probe grades
  measured: 4.98          # step 2 baseline
  matches_physical: false
  closes_in_step: 5       # the #43 step that closes the gap
```

- `nominal` is the configured value. `measured` is what the step-2 baseline
  observed, or the step that last re-measured the entry (see "Closing a
  gap"). An entry needs at least one of them.
- `contract` is the `[min, max]` range the probes grade. Only rates have one.
- `matches_physical` is `true` or `false`. When it is `false`,
  `closes_in_step` names the #43 step (4-9) that closes the gap, or
  `out_of_scope` with a `note` saying why #43 leaves it open.
- `tolerance` is an absolute tolerance in the entry's unit. Without it,
  numbers must be equal. The camera mounts carry none, so the simulator holds
  the same digits as the physical entry: the color frame's are the live TF
  rounded to six decimals, and `cam_1_link` and `cam_1_depth_frame` keep the
  xacro's own values (their y has nine decimals). Re-measuring or re-rounding
  a physical value means changing the simulator's with it.

`test/real_robot_contract_test.py` checks every flag. Where the simulator and
physical values are comparable it recomputes the match and fails when a flag
disagrees. A `true` flag must be verifiable: if the two values can't be
compared, for example a simulator rate of 5 Hz against the physical cloud's
measured range of 2.83-10.93 Hz, the test fails it. Either make the values
comparable, or set the flag to `false` with a `closes_in_step`. A `false` flag
with values that can't be compared is accepted as a recorded gap.

## Who reads it

The probes resolve the installed copy through the ament index
(`share/yahboom_rosmaster_gazebo/config/`). The unit tests read the source
tree, so they also run standalone. `scripts/real_robot_contract.py` is the
shared loader.

| Reader | Keys |
|---|---|
| `scripts/sensor_contract_probe.py` | `topics.<topic>.rate_hz.contract` (only with `performance_checks:=true`), `topics.<topic>.frame_id`, `topics./odom.child_frame_id`, and the `camera` size, intrinsics, distortion and `horizontal_fov_rad` that `camera_info` must carry |
| `scripts/depth_geometry_probe.py` | `camera.width`, `camera.height`, the depth image and cloud `frame_id`, `depth.min_range_m`, `depth.max_range_m` (the clip range), and the depth and color `frames.mounts`: the color frames' calibrated offset in TF, where the color aperture sees the target, and where each cloud point must land in `cam_1_depth_frame` |
| `test/depth_geometry.launch.py` | `frames.mounts.cam_1_color_frame`, `frames.mounts.cam_1_depth_frame` and `frames.base_footprint_to_base_link_z_m`, to put the target at a known depth along the color optical axis, centred on the depth aperture |
| `yahboom_rosmaster_description/test/robot_description_contract_test.py` | `frames.*` mounts and camera frames, `wheels.*`, the camera, LiDAR and IMU settings the xacro must produce, and the physical `frames.tape_check` camera setback that `camera.obj` must reproduce |
| `test/sensor_contract_probe_test.py` | Every rate the probe grades has a contract; the probe's Best Effort topics and wheel joint names match the ledger |
| `test/real_robot_contract_test.py` | Every parity flag (a `true` one must be verifiable), the provenance commits and `step_measurements` records, the legacy `superseded_by` references, and the `/joint_states` rate in `config/ros2_control.yaml` |

`sensor_contract_ci` still runs with `performance_checks:=false`, so it reads
the frames and field of view but never grades the rate contracts.

## Closing a gap

Each step of #43 closes gaps the same way:

1. Change the model: xacro, bridge or node.
2. In each affected `simulator` entry, set `nominal` to the new value, set
   `matches_physical: true` and remove `closes_in_step`. Move `contract` ranges
   along with rates.
3. Re-measure with the physical probe (see the caveats below), or with TF
   lookups for frames. Update `measured`, and add a record to
   `simulator.provenance.step_measurements` with the step, the commit you
   measured, the date, the method and the keys you re-measured.
   `real_robot_contract` checks that the commit is a full SHA and that every
   listed key has a `measured` value.
4. Run `real_robot_contract`, `robot_description_contract` and the launch
   contracts. A flag that no longer agrees with the values fails the first.

## Refreshing the physical section

The physical section is copied from physical_rosmaster by hand. Read its
`main` without checking it out:

```bash
git -C ../physical_rosmaster fetch origin
git -C ../physical_rosmaster rev-parse origin/main
git -C ../physical_rosmaster show origin/main:docs/sensor_capabilities.md
```

| Ledger keys | physical_rosmaster source |
|---|---|
| `frames` | `yahboomcar_description/urdf/yahboomcar_X3.urdf.xacro`; live TF in `robot_artifacts/<capture>/post_deploy_tf.json`; the tape checks in `docs/sensor_capabilities.md` |
| `wheels` | The xacro; `yahboomcar_bringup/param/x3_odometry.yaml` and `x3_driver.yaml` |
| `topics` | `docs/sensor_capabilities.md` "Rates and latency"; `robot_artifacts/<capture>/camera_public.json` and `sensors_stationary.json`. QoS: `yahboomcar_astra/yahboomcar_astra/sensor_adapter.py` and `sllidar_ros2/src/sllidar_node.cpp` |
| `camera` | `docs/sensor_capabilities.md` "Camera"; `camera_public.json`; `yahboomcar_astra/launch/astra_platform.launch.py` |
| `depth` | `docs/depth_camera_calibration.md` |
| `point_cloud` | `sensor_adapter.py` `transform_cloud()` with the launch defaults `cloud_strip_nan` and `cloud_decimation` |
| `lidar` | `sensors_stationary.json`; `docs/sensor_capabilities.md` "LiDAR" |
| `imu` | `sensors_stationary.json`; `yahboomcar_bringup/param/imu_filter_param.yaml` |
| `odometry`, `command` | `x3_odometry.yaml`, `x3_driver.yaml` |
| `hardware_extensions` | `config/robot_contract.yaml` |

After copying:

1. Set `physical.provenance.commit` to the full SHA and `read_on` to the date.
   Update `sources` if a file moved.
2. Keep each group's `confidence` honest. A value read from a parameter file is
   `inferred` until a capture observes it.
3. When a newer capture replaces a pipeline the ledger still describes, keep
   the old numbers in a `superseded_<date>` subgroup, as `point_cloud` does.
4. Run `real_robot_contract_test.py`. Every flag the new physical values
   invalidate fails there. Fix the flags, not the physical numbers.

**The point-cloud timing is provisional** (#43 decision D4). The physical
figures describe the 2026-09-17 pipeline, which used 32-byte organized points.
The shipped pipeline strips NaN returns and packs 16-byte points, and nobody
has measured its timing yet.
[physical_rosmaster#43](https://github.com/AIRclub-UdeSA/physical_rosmaster/issues/43)
re-measures it. Refresh `physical.topics./cam_1/depth/color/points` and
`physical.point_cloud` from that result before step 6 fits its timing model.

## The simulator baseline and its caveats

The `measured` values come from the step-2 baseline on `main` @ `f7c931b`
([#43 comment](https://github.com/AIRclub-UdeSA/yahboom_rosmaster/issues/43#issuecomment-5814742158)),
except for the keys a later step re-measured and listed in
`simulator.provenance.step_measurements`.
It ran physical_rosmaster's `tools/sensor_capability_probe.py`, unmodified,
against the simulator, plus a sim-clock helper for latency. Three caveats
carry forward to every later measurement:

- **The physical probe cannot run on sim time.** It accepts no `--ros-args`,
  so its latency fields read wall epoch minus sim time and are not comparable.
  Its rates are wall clock, which equals sim time only while the real-time
  factor is about 1. Measure simulator latency on the ROS clock instead
  (sim time at receipt minus `header.stamp`).
- **Simulator topics have intrinsic sim-time latency.** On the host GPU:
  images 16 ms, cloud 26 ms, `/scan` 38 ms. On llvmpipe with 4 CPUs: 64-66, 80
  and 120 ms. `/imu/data`, `/odom` and `/joint_states` stay under 1 ms. The
  timing models of steps 6 and 8 must add their delay on top of this, not
  assume it is zero.
- **The 320x240 camera at 30 Hz cuts the real-time factor on CI's llvmpipe to
  about 0.38 with 4 CPUs** (0.95 on the host GPU, step 5). It still renders at
  30 Hz in sim time, so stamp-based rate checks pass, but a wall-clock
  deadline gets about 0.4 s of sim time per second, and wall-clock rates read
  low. Measure rates on stamps, or on the host GPU.

## The legacy bag audit

`legacy_bag_audit` keeps what the replaced contract measured from
`bags_x3.zip`: 5 Humble sqlite3 bags, about 62 minutes and 425,513 messages,
recorded with the vendor software stack that physical_rosmaster has since
replaced. The archive is untracked (see `.gitignore`). Two parts stay because
nothing newer measures them: the `/cmd_vel` rates and magnitudes, and the
`command_response` gain table. The rest is under `superseded`, each value
naming its replacement. That includes the old `base_footprint -> base_link`
height of 0.0815 m, superseded by the physical 0.0714 m, and the "always zero"
`/joint_states`.

`scripts/real_robot_bag_analyzer.py` covers **only this bag format**. It opens
sqlite3 bags and reads the vendor stack's topic names (`/image_raw`,
`/camera_info`) alongside `/odom`, `/joint_states`, `/scan`, `/imu/data`,
`/tf_static` and the recorded `/robot_description`. It knows nothing of the
`/cam_1/*` topics or physical_rosmaster's capability-probe JSON, and nothing in
the `physical` section comes from it. To re-run it on the old bags:

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

`bag_x3_5` is excluded from every motion statistic. Its odometry path length
is exactly zero and its IMU is frozen for the whole recording, while
`/cmd_vel` and `/scan` keep changing.
