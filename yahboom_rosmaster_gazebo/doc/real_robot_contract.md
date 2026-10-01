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
rate_hz:                  # simulator.topics./scan.rate_hz
  nominal: 5.0            # configured on main
  contract: [4.5, 5.5]    # what sensor_contract_probe grades
  measured: 4.99          # step 2 baseline
  matches_physical: false
  closes_in_step: 8       # the #43 step that closes the gap
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

A physical value that is a measured range is written `{min: low, max: high}`,
as `physical.depth.scale_error` is. A simulator number agrees with it when it
lies inside the range, widened by the entry's `tolerance`, so "the simulator's
scale error is in the physical range" is checked, not assumed. A bare
`[low, high]` list is not a range: lists compare element by element, and a
simulator number against one is not comparable.

### Simulator-only settings

`simulator.render_settings` is the one group of the simulator section that is
not made of parity entries. It holds what the simulator is configured with that
has no physical counterpart, each as `{nominal, note}`: today the depth camera's
clip planes, `depth_near_clip_m` (0.05) and `depth_far_clip_m` (8.0). They are
read with `RealRobotContract.setting()` by `scripts/depth_geometry_probe.py`
(the range its finite depths must fall in) and by the description contract (the
xacro's `<clip>`). They are not `depth.min_range_m` and `depth.max_range_m`,
which are parity entries: the minimum range the robot delivers, 0.6 m, and the
same far limit as an out-of-scope gap. `test/real_robot_contract_test.py`
validates the group: each setting must be a positive number, must not carry
`matches_physical`, and must not share a name with a physical key, which would
make it a parity entry that has to be compared.

`test/real_robot_contract_test.py` checks every flag. Where the simulator and
physical values are comparable it recomputes the match and fails when a flag
disagrees. A `true` flag must be verifiable: if the two values can't be
compared, for example a simulator rate of 30 Hz against the physical cloud's
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
| `scripts/sensor_contract_probe.py` | `topics.<topic>.rate_hz.contract` (only with `performance_checks:=true`; the cloud's is graded on its mean rate over the window, since its gaps make the median period meaningless), `topics.<topic>.frame_id`, `topics./odom.child_frame_id`, and the `camera` size, intrinsics, distortion and `horizontal_fov_rad` that `camera_info` must carry. The cloud's layout is graded in every run against the physical adapter's |
| `scripts/depth_geometry_probe.py` | `camera.width`, `camera.height`, the depth image and cloud `frame_id`, `render_settings.depth_near_clip_m` and `depth_far_clip_m` (the clip range), and the depth and color `frames.mounts`: the color frames' calibrated offset in TF, where the color aperture sees the target, and where each cloud point must land in `cam_1_depth_frame` |
| `test/depth_geometry.launch.py` | `frames.mounts.cam_1_color_frame`, `frames.mounts.cam_1_depth_frame` and `frames.base_footprint_to_base_link_z_m`, to put the target at a known depth along the color optical axis, centred on the depth aperture |
| `yahboom_rosmaster_description/test/robot_description_contract_test.py` | `frames.*` mounts and camera frames, `wheels.*`, the camera, LiDAR and IMU settings the xacro must produce (the clip planes from `render_settings`), and the physical `frames.tape_check` camera setback that `camera.obj` must reproduce |
| `scripts/sensor_contract_probe.py` (with `scripts/depth_quality.py`) | `physical.depth`: the scale error range, the minimum range and the noise model, which it parses from the ledger's string. It grades the published depth image against the rendered one of the same stamp (see "The depth quality check") |
| `test/command_limits_test.py`, `test/wheel_state_odometry_test.py` | `command.*_limit_*` against `config/command_limits.yaml`, and `odometry.*_covariance_x_y_yaw` against `config/wheel_odometry.yaml`, both as physical values and as simulator nominals. `scripts/base_feedback_probe.py` reads the limits from the ledger and checks them on `/cmd_vel_gz` |
| `test/sensor_contract_probe_test.py` | Every rate the probe grades has a contract; the probe's Best Effort topics and wheel joint names match the ledger |
| `test/real_robot_contract_test.py` | Every parity flag (a `true` one must be verifiable), the provenance commits and `step_measurements` records, the legacy `superseded_by` references, the `/joint_states` rate in `config/ros2_control.yaml` (the broadcaster's `update_rate`, a whole division of the loop's), and that the cloud's timing entries (`rate_hz`, `period_p95_ms`, `latency_ms`, `gap_frames`, `worst_gap_s`) and the depth entries (`scale_error`, `noise_model`, `min_range_m`) are what `config/sensor_profiles.yaml`'s physical profile implies, `render_settings`, the range comparison, and that no measured key is in two `step_measurements` records |

`sensor_contract_ci` still runs with `performance_checks:=false`, so it reads
the frames and field of view and the cloud's layout but never grades the rate
contracts. It also grades 15 sim seconds of the cloud's timing against
`config/sensor_profiles.yaml` (`scripts/cloud_timing_probe.py`), which is where
the timing entries above come from, not from the ledger: the ledger records
that the profile matches the robot, and the profile is what the simulator runs.

## The depth quality check

`sensor_contract_probe` grades the depth the adapter publishes (#43 step 7)
in every run of `sensor_contract_ci`, `_empty`, `_cafe` and `_empty_ideal`, so
the CI gate carries it without an extra simulator. For the length of its
grading window (`depth_quality_frames`, default the `samples` count) it
subscribes to the adapter's raw input, `/internal/cam_1/depth/image_raw`, and to
`/cam_1/depth/image_raw`, both Best Effort and five frames deep, and pairs the
two by exact stamp. The render is the truth: no geometry model, and no code
shared with the adapter. `scripts/depth_quality.py` says whether the pairs look
like the physical camera (or, under `sensor_profile:=ideal`, like the render):

- every profile: no published infinity, and NaN wherever the render has no
  return, exactly;
- `ideal`: every finite rendered pixel published bit for bit;
- `physical`: nothing below `depth.min_range_m` is ever published, exactly;
  pixels more than six standard deviations under it are NaN and over it finite;
  the mean of published over rendered, where the render is 1.5 m or more away,
  lies in the physical scale range within its standard error; and in each band of
  distances (0.65 to 8.1 m, about 0.6 m to 8 m of the empty world's floor) the
  residual has the standard deviation of the ledger's noise model, to five
  standard errors and a percent.

The bands are graded in the frames' own distances: an empty-world frame spans
the whole range from its floor. A correct simulator failed this check in none of
1,500 seeded trials of an independent model of the camera; each departure a test
introduces (no scale, a wrong exponent, no noise floor, no cutoff, a leaked
infinity) fails it and is named.

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
   `real_robot_contract` checks that the commit is a full SHA, that every
   listed key has a `measured` value, and that no key is in two records: a step
   that re-measures a key takes it over from the older record, so remove it from
   that record's `keys` and say in its `note` where it went. The harness in
   `tools/` (see `tools/README.md`) produces the camera figures.
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
| `point_cloud` | `sensor_adapter.py` `transform_cloud()` with the launch defaults `cloud_strip_nan` and `cloud_decimation`; the field offsets and datatypes from `robot_artifacts/<capture>/camera_settled_*.json`. The cloud's timing entries under `topics` come from `docs/sensor_capabilities.md` "Point cloud", and its gap table is copied into `config/sensor_profiles.yaml` |
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

**The point-cloud timing is the 2026-09-24 re-measurement** (#43 decision D4).
The physical figures come from
[physical_rosmaster#45](https://github.com/AIRclub-UdeSA/physical_rosmaster/pull/45)
(`docs/sensor_capabilities.md` "Point cloud" and `robot_artifacts/
x3c_sensor_capability_2026-09-24/`), which measured the shipped pipeline:
16-byte points, `cloud_strip_nan` true. #43 step 7 re-pinned
`physical.provenance.commit` to #45's merge commit, 468662c, and re-read every
value taken from `docs/sensor_capabilities.md` or the 2026-09-24 artifacts
there. None changed. #45's last commit, b76b2b5, scoped the boot claim to one
warm reboot's 25-65 s window (the ledger's comment says so) and reworded the
paired-run counts (the driver produced a cloud for "at least 36%" of frames); no
value here depends on either. The compared timing values are scalars, so the
mean rate (8.26 Hz), the p95 period (367 ms) and the latency (50 ms) need no
range support. The 7.3-9.2 Hz settled range and the 43-53 ms run medians are
kept as descriptive fields that nothing is compared with. The one physical value
that is a range, the depth scale error, uses the `{min, max}` form above.

The simulator's frame period is not the robot's. With the 1 ms physics step the
camera's stamps come 33 ms apart (30.3 Hz), against the robot's 33.3 ms, so the
gap table's frames convert to 33 ms in the simulator: a p95 of 363 ms against
367. The `tolerance` on each entry absorbs the 1% difference.

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
