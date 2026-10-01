# Deferred simulator changes

The original Donatello CAD migration was intentionally visual-only. The ideas
below were encountered while integrating and diagnosing the model; each is
evaluated independently, and accepted follow-ups are marked as implemented.

## Correct physical wheel centres (implemented)

The physical robot description places its wheel axes at `x = +/-0.08 m` and
approximately `y = +/-0.0845 m`: a `0.160 m` wheelbase and `0.169 m` wheel
separation. The simulator previously subtracted a `0.01 m` lateral offset,
leaving its joints and collision spheres only `0.149 m` apart even though the
drive plugin and wheel-state odometry both used `0.169 m`.

The obsolete offset has been removed. Joints, collision spheres, and wheel
inertias now use `y = +/-0.0845 m`, making contact geometry and rotational
kinematics consistent with the real robot, drive plugin, and odometry model.

The equal-and-opposite visual offsets were removed at the same time, so the CAD
wheel meshes remain in their established rendered positions. The description
contract protects the corrected joint origins and link-local visual origins. A
headless ideal-profile yaw regression compares wheel odometry with raw ground
truth after a meaningful turn.

## Controller-free joint-state path

Gazebo's native `JointStatePublisher` can replace the read-only
`gz_ros2_control` and `joint_state_broadcaster` path. A prototype avoided one
observed controller-manager startup timeout and worked on Linux and macOS.

- Potential benefit: fewer runtime components and no controller-manager startup
  dependency for wheel TF and odometry.
- Risk: changes the supported architecture, publication headers and timing, and
  shutdown behavior of the high-rate Gazebo bridge.
- Evaluate with: repeated GUI/headless startup and shutdown runs, topic/frame
  contracts, TF timestamp tests, and all motion and odometry tests.

Any future implementation should be a separate change. It may need a small
relay to restore `JointState.header.frame_id` and cap the native stream to
10 Hz, the rate `joint_state_broadcaster` now publishes (#43 step 8a).

## Command limits and odometry (implemented)

The base follows the physical driver's command limits and publishes odometry
at its rate and with its covariance (#43 step 8a). Every number is in a config
file with its source and tied to the parity ledger by a test.

- **Limits.** `config/command_limits.yaml`: ±1.0 m/s on x and y, ±5.0 rad/s on
  yaw, from physical_rosmaster 468662c `x3_driver.yaml` (`xlinear_limit`,
  `ylinear_limit`, `angular_limit`). The driver's `clamp_motion_command`
  saturates each component on its own, without scaling the vector, and answers
  a non-finite command with zero. `cmd_vel_watchdog.py` does the same, and does
  it before the motion bias: the driver clamps before it hands the command to
  the motor controller, and the bias stands for the drivetrain that follows. A
  biased command may therefore exceed a limit slightly, as the real wheels can.
  The watchdog has no default for the limits, so a launch that forgets them
  fails. The limits are inferred (no run drove the robot past them).
- **Rate.** `joint_state_broadcaster` has no `publish_rate` parameter in
  Humble's ros2_controllers 2.54, so the `publish_rate: 30.0` that
  `ros2_control.yaml` carried was always ignored and the 30 Hz came from the
  controller manager's `update_rate`. The controller's own `update_rate: 10`
  now makes it publish every third cycle of the unchanged 30 Hz loop. The
  steps are 34 ms, so the stamps are 102 ms apart (9.80 Hz against the robot's
  10.00 Hz); the ledger's tolerance of 0.3 Hz covers that, and the measured
  period is what its nominal states. With `use_ros2_control:=false` the
  `topic_tools` throttle is set to 10 Hz instead.
- **Covariance.** `config/wheel_odometry.yaml`, passed to
  `wheel_state_odometry.py` as its parameters: pose `0.001` and twist `0.0001`
  on x, y and yaw, from `x3_odometry.yaml`, which marks them provisional. Only
  the entries at 0, 7 and 35 are set, as in the robot's `odometry.hpp`.
- **Integration step.** Wheel odometry now integrates a third as often. Its
  midpoint rule is exact for motion along a constant heading and a constant
  turn rate, and its error on a changing command grows with the step, so a
  bound that depends on it follows the 100 ms step, not the 33 ms one.

## Render visibility masks

Gazebo supports per-visual visibility flags and camera/LiDAR visibility masks.
They can hide robot visuals from its own rendering sensors.

- Potential benefit: prevents self-occlusion if a future visual crosses a
  sensor's real near plane.
- Risk: adds renderer-specific behavior and does not affect RViz overlays.
- Evaluate with: raw RGB, depth, point-cloud, and LiDAR messages—not the RViz
  Camera display—while stationary and moving.

The current CAD migration does not enable a blanket robot-wide mask. It gives
only the LiDAR housing a private render bit because the detailed shell encloses
the GPU ray origin; the LiDAR geometry test requires that self-filter. The RViz
camera panel is configured as image-only instead. The camera needs no mask at
its physical mount; see "Self-occlusion" below.

## Startup timing changes

Starting wheel state and odometry before sensor bridges, or delaying RViz, can
reduce transient message-filter warnings on slow machines.

- Potential benefit: cleaner first seconds after launch.
- Risk: hides an underlying startup fault and increases launch complexity.
- Evaluate with: repeated cold starts on supported platforms and explicit
  readiness checks instead of fixed delays.

## Camera mount and extrinsics (implemented)

The camera frames follow the physical X3 (#43 step 4). The values are in
`config/real_robot_contract.yaml`, relative to `base_link`:

- `cam_1_link` and `cam_1_depth_frame` sit at
  `(0.057105, 0.000017948, 0.03755) m`, physical_rosmaster's
  `camera_mount_joint`. The robot's live TF confirms it, and so does the tape:
  the optical axis is 108.95 mm above the floor, against a measured 105-109 mm.
  The mount was `(0.105, 0, 0.05) m`.
- `cam_1_color_frame` sits 25.1 mm to the left, at the Orbbec factory
  calibration of the robot's Astra (serial ACRC64300ET), rotated 0.34 degrees.
  It is specific to that unit. It was co-located with the depth frame.
- The `cam_1_infra1_*` and `cam_1_infra2_*` frames are gone; the physical
  robot publishes neither.

Since step 5 the Gazebo camera renders from `cam_1_color_frame`; see "Camera
sensor model" below.

### Camera visual

The challenge website moves the camera housing 10 mm rearward, and
`tools/convert_rosmaster_cad.py` used to copy that shift. It no longer does,
because the physical robot disagrees:

| Housing front | x from `base_link` | Behind the chassis front (116.5 mm) |
|---|---|---|
| Tape measurement on the physical X3 | about 76.5 mm | 40 mm, +/-2 mm |
| physical_rosmaster `camera_link.STL` at the physical mount | 77.2 mm | 39.3 mm |
| `camera.obj` without the 10 mm shift (now) | 76.2 mm | 40.3 mm |
| `camera.obj` with the shift (before) | 66.2 mm | 50.3 mm |

The unshifted mesh matches the physical STL's bounding box within 1 mm on
every axis. The tool now bakes `camera.obj` relative to the physical mount, and
the description contract checks the 40 mm setback.

The legacy chassis STL is byte-identical to the body mesh in the physical
repository, and the Donatello LiDAR envelope and wheels match its meshes (the
wheels within about 0.2 mm). Only the camera placement has been checked against
the physical unit itself. A camera collision envelope remains a separate
decision; `enable_collision` is off.

### Self-occlusion

The render origin, `cam_1_color_frame` since step 5, sits inside the housing,
21 mm behind its front face. The 0.05 m near clip hides the whole housing;
recheck this if the near clip ever drops below 21 mm. Beyond the near clip,
the chassis front edge passes 9.7 mm below the bottom of the 47.6-degree
vertical field of view. The color frame's 0.27-degree downward pitch and its
position 2.2 mm further back take 1.4 mm off the 11.1 mm that step 4 found from
`cam_1_link`. At step 4's 56.5 degrees from `cam_1_link` the margin was
4.6 mm. Raw `/cam_1/depth/image_raw` frames in the empty world show no robot
pixels at either step: every finite pixel within 0.5 m is floor to within
1 um, and the bottom row sees the floor 0.245 m away. The camera needs no
visibility mask.

## Camera sensor model (implemented)

The camera follows the physical Astra's interface (#43 step 5). The values are
in `config/real_robot_contract.yaml`.

- Both streams are 320x240 at 30 Hz with the color intrinsics of
  physical_rosmaster's `camera_public.json`: fx = fy = 271.80938720703125,
  `plumb_bob` with zero coefficients. `rgbd_camera.urdf.xacro` derives the
  horizontal FOV Gazebo renders from, 1.0640610 rad (60.97 degrees). With the
  1 ms physics step, `update_rate` 30 fires every 33 ms of sim time, so the
  stamps run at 30.3 Hz. That is the closest the step allows: 34 ms would
  give 29.4 Hz.
- Gazebo renders from `cam_1_color_frame`, and all four image and
  `camera_info` topics carry `cam_1_color_optical_frame`. The Astra registers
  depth to color, so one aperture at the color frame is the physical model,
  not a simplification. URDF-to-SDF lumping carries the frame's 25.1 mm
  offset and 0.34-degree rotation into the sensor pose.
- **The principal point cannot follow the physical unit.** Fortress
  (gz-sensors 6.9, gz-rendering 6.6) writes `<intrinsics>` and `<projection>`
  into `camera_info`, but `Ogre2DepthCamera` does not override
  `SetProjectionMatrix`, so the RGB-D camera always renders a symmetric frustum
  from the FOV. Principal points of (158.719, 120.092), (160, 120) and
  (140, 100) produced bit-identical color, depth and cloud data; only
  `camera_info` changed. A straight-line fit of Gazebo's own cloud rays puts
  the rendered axis at (159.5, 119.5) in ROS pixel coordinates (pixel centres
  at integers), with 7e-5 px residual. A red box's edges land in the depth
  image on the columns and rows that axis predicts (97-242, 82-154), not on
  those of (160, 120) or (158.719, 120.092). The simulator publishes
  (159.5, 119.5): 0.78 px and 0.59 px from physical, recorded as ledger
  tolerances. Gazebo's default of width / 2 would be half a pixel off its own
  image.
- Gazebo's native cloud is expressed in the sensor's regular frame, now
  `cam_1_color_frame`. `pointcloud_frame_relay.py` used to relabel it
  `cam_1_depth_frame`, which would now be 25 mm and 0.34 degrees wrong. The
  relay instead transforms every finite point with the static
  `cam_1_depth_frame` <- `cam_1_color_frame` transform, as physical_rosmaster's
  `sensor_adapter.py` does, and keeps Gazebo's organized 24-byte layout. Step 6
  replaced it with the camera adapter (below). It costs 1.6 ms per cloud. Back to back on the host GPU,
  the cloud reached 25.2 Hz on the probe against 27.1 Hz with the
  relabel-only relay (26.7 Hz in the final measurement). Keep its arithmetic
  element-wise: a numpy matrix product
  ran on a multithreaded BLAS whose spinning threads took 8.5 cores and cut the
  real-time factor to 0.77.
- `depth_geometry` checks the rendering origin through parallax. Its target
  is centred on the depth aperture, so from the color aperture its centroid
  sits at 169.5 px against 159.5 px from the depth aperture. It also checks
  that each sampled cloud point lies within 5 mm of the depth image's point
  transformed into `cam_1_depth_frame`.

### Rendering cost

On the host GPU (Radeon Renoir) the simulator runs at a real-time factor of
0.95 with the 30 Hz camera, against 0.996 at 5 Hz. CI's software rendering
(llvmpipe, reproduced with 4 pinned CPUs) drops to 0.38, against 0.87 at
5 Hz. Every sensor keeps its sim-time rate there, so stamp-based checks pass,
but wall-clock rates read low: 11.7 Hz for the images, and the cloud misses
more than half its frames (13.0 Hz on stamps). The CI contract gate grades no
rates and ran in about 125 s locally under those conditions. Lowering the
camera for CI would stop CI testing the camera that ships (#43 decision D1).

## Point cloud pipeline (implemented)

The point cloud follows the physical Astra's adapter in layout and, by default,
in timing (#43 step 6). The numbers are in `config/sensor_profiles.yaml` and the
parity ledger.

- **The node.** `camera_adapter.py` replaces `pointcloud_frame_relay.py` and the
  depth image's QoS relay, and Gazebo's cloud is no longer bridged. It pairs
  the bridged color and depth images by exact stamp, back-projects the depth
  with the color `camera_info`, attaches the color, transforms the points into
  `cam_1_depth_frame` and packs 16-byte `x, y, z, rgb` points, with
  `cloud_strip_nan` and `cloud_decimation` meaning what they mean on the robot.
  Against Gazebo's native cloud, transformed as the relay did, on 150 frames
  with a red, green and blue box in view, the largest point error was 1.1e-6 m,
  the finite points were the same set and the `rgb` bytes were identical,
  including the byte order: a red pixel is 0, 0, 255, 0.
- **Timing.** Under `sensor_profile:=physical` each frame is one tick of the 30 Hz
  frame clock, and the gap to the next cloud is drawn from the robot's measured
  table (988 gaps, whole frames, no autocorrelation, so independent draws).
  Only the frames that become clouds are built. A cloud is published at its
  frame's stamp plus 50 ms of sim time, measured from the stamp and not from
  when the images arrived, which is 7-8 ms on the host GPU and 35 ms on
  llvmpipe with 4 CPUs. None of 1,294 clouds on the GPU and 502 on llvmpipe was
  ready late; one that is publishes at once and is logged. Every depth image is
  published either way, and it is published the moment it arrives, as on `main`
  and in physical_rosmaster's `sensor_adapter.py`: a lost or late color image,
  or a `camera_info` that has not come, does not drop or delay it. It passes
  through one conditioning step (`DepthConditioner`, from step 7, see "Depth
  quality" below), which needs neither the color nor `camera_info`. The conditioned array is kept
  by stamp, and the cloud is built from that same array when its color image
  arrives. A `camera_info` of another size, or a bad color image, drops only
  that cloud.
- **Frame clock.** The simulated camera's frames come 32, 33 or 34 ms apart
  (30.3 Hz), and wander up to 2 ms from a 33 ms grid over many frames, so the
  tests grade a gap as a whole number of frames to within 3 ms. Gazebo's camera
  also emits a burst of catch-up frames, a millisecond or so apart, when it
  starts, so a frame stamped less than half a period after the last one counted
  makes no cloud.
- **Verification.** Three 40 s runs of physical_rosmaster's
  `sensor_capability_probe.py` (from #44, with its receipt clock moved to sim
  time) on the host GPU gave 1,006 gaps: median 2 frames, p95 10 (the table's is
  11; 1,006 draws give 10 about one time in five), longest 32, mean 3.49
  against the table's 3.62, per-run rates 8.95, 8.45 and 8.49 Hz against the
  robot's 7.3-9.2 (a 40 s run holds only about 340 clouds), and a 50.0 ms median
  latency. No gap length differed from the table by more than 1.3 percentage
  points.
- **Tests.** Unit tests cover the arithmetic, layout, stripping, padding, malformed
  input, the seeded sampler, the scheduler, the profile file, and the probe's
  grading. `cloud_timing_physical` grades 60 sim seconds and `cloud_timing_ideal`
  every frame, locally. `sensor_contract_ci` grades 15 sim seconds inside the
  simulator it already launches, adding about 28 s to the CI gate under
  llvmpipe. `depth_geometry` runs under `ideal` with the organized cloud, and
  checks the cloud's red points against the target's known front face, which
  no longer depends on the depth image the cloud came from.

### Cost

The adapter costs more CPU than the two relays it replaces. Measured over 30 s
on the host GPU (Ryzen 5 3600, Radeon RX 5600/5700; percent of one core):

| | `main` | `physical` | `ideal` |
|---|---|---|---|
| Real-time factor | 0.994 | 0.997 | 0.997 |
| Gazebo server | 225 | 219 | 215 |
| Adapter / cloud relay | 19 | 53 | 55 |
| QoS relays | 13 | 8 | 7 |
| Bridges | 26 | 20 | 19 |

and on llvmpipe pinned to 4 CPUs (real-time factor about 0.5):

| | `main` | `physical` | `ideal` |
|---|---|---|---|
| Real-time factor | 0.494 | 0.499 | 0.502 |
| Gazebo server | 189 | 190 | 189 |
| Adapter / cloud relay | 7 | 19 | 22 |
| QoS relays | 5 | 3 | 3 |
| Bridges | 11 | 9 | 10 |

Neither changes the real-time factor. Measured again for `physical` after the
depth image was decoupled from the pairing, the adapter used 51% of a core and
the real-time factor was 0.998 on the GPU, and 19% and 0.496 on llvmpipe: no
change. The adapter's arithmetic is not what costs: a paired 320x240 frame
builds a cloud in 2-5 ms, and with its sim-time subscription switched off the
adapter used 11.5% of a core. The other 40 points are rclpy's executor waking
for every tick of the 1 kHz `/clock`, in Python. If that ever matters, subscribe
the adapter to a `topic_tools throttle` of `/clock` at a few hundred Hz and
schedule from the nearest tick, or move the delay into a small C++ node.
Giving the adapter the color image and both `camera_info` topics as well saved
1-2 points of about 300 and left the real-time factor alone, so those relays
stay.

Gazebo computes its own cloud whether or not anything subscribes to it: the
server's CPU was 201% with a subscriber on the internal cloud topic and 202%
without. Removing the bridge entry and the relay saves the relay, the bridge's
conversion of 1.8 MB messages (about 5 points) and the transport.

A Best Effort 600 kB cloud is sometimes lost between processes. About 1% were
between two quiet processes, and up to a third when a probe shares four
saturated CPUs with software rendering, so the local launch tests grade the
gaps with tolerances derived from the sample size, and the ideal grade allows
3% of clouds lost. The robot's own Best Effort cloud has the same exposure.

## Depth quality (implemented)

Under `sensor_profile:=physical`, the default, the depth image has the physical
Astra's scale error, noise and minimum range (#43 step 7). The numbers are in
`config/sensor_profiles.yaml`, each with its source, and in the parity ledger.

- **The model.** `DepthConditioner` in `camera_adapter.py` runs once on every
  depth image as it arrives. For a pixel with rendered depth `z`, in this order:
  `z' = z * (1 + s)` with `s = -0.011858`, the inverse of physical_rosmaster's
  recommended correction `true ~= reported * 1.012`
  (`docs/depth_camera_calibration.md` "Scale"; the recorded range is -1.1% to
  -1.3%, and the linear fit's slope was -1.33%); then
  `z' + sigma(z') * n` with `sigma(d) = max(0.002, 0.0019 * d^2.36)` m and an
  independent standard normal `n` per pixel ("Noise"); then NaN below 0.6 m
  ("Range limits and dropout"). The public image and the cloud come from that one
  conditioned array.
- **Which distance.** The doc tabulates scale, noise and range in the distance the
  camera reports, not the tape's: its noise table's distances (0.63, 0.83, 1.12,
  1.63 m) are the "Reported" column of its scale table, and its 0.6 m minimum and
  its dead-zone table are reported distances too. So sigma takes the scaled `z'`
  and the cutoff tests the published value, after noise. The published contract is
  then exactly "nothing below 0.6 m", and does not depend on the rendered depth.
  Sigma from the rendered depth would differ by 2.8%.
- **Extrapolation.** The fit is calibrated to 3.6 m (its farthest wall) and the
  simulator applies it beyond, unclamped: about 12 cm at 5.8 m, the deepest return
  in physical_rosmaster's `camera_public.json`, and 25 cm at the 8 m far clip.
  Nothing measured those. The profile's sources and the README say so.
- **No return is NaN.** `metric_depth()` in physical_rosmaster's
  `sensor_adapter.py` (468662c) sets zero to NaN, so a pixel with no return is
  NaN. Gazebo has +inf beyond the far clip. Both profiles now publish NaN for every
  non-finite pixel; `ideal` publishes every finite pixel bit for bit. This is a
  breaking change for both profiles, including the organized cloud under
  `cloud_strip_nan:=false`. The 1 mm quantization of the robot's `16UC1` depth is
  not modelled: 0.29 mm rms against a 2 mm noise floor.
- **Randomness.** A frame's noise comes from
  `Generator(PCG64(SeedSequence([sensor_seed, frame stamp in ns, tag])))`. It
  needs no state, so a dropped frame shifts no other, and it is a stream of its
  own: the cloud gap sampler is Python's `random.Random(seed)`, untouched, and its
  sequence for a seed is pinned in `test/camera_adapter_test.py` from `main`.
  numpy does not promise the same stream across versions (the CI image has 1.21.5,
  a development machine may have 2.x), so no test pins a noise value: they assert
  statistics, and reproducibility within a process.
- **Arithmetic.** float32, element-wise, one thread (`OMP_NUM_THREADS=1`), no
  matrix products.
- **What is checked, and where.** Unit tests (`camera_adapter_test.py`) check the
  scale, the mean and standard deviation at six distances to five standard errors,
  the 0.6 m cutoff, NaN for no return, the `ideal` path bit for bit on finite
  pixels, the per-(seed, stamp) stream, and that the cloud holds exactly the
  published finite pixels. Each was shown to fail when the code is broken on
  purpose: no scale, a flipped sign, exponent 2, no noise floor, no cutoff, a
  cutoff before the noise, sigma from the rendered depth, infinity left in place,
  noise independent of stamp or of seed, one advancing generator, noise drawn only
  for valid pixels, a second conditioning for the cloud, float64 output, and a
  reseeded gap sampler. `sensor_contract_probe` checks the running simulator: it
  pairs the adapter's raw input with the published depth of the same stamp, so the
  render is the truth (see `doc/real_robot_contract.md`, "The depth quality
  check"). On the host GPU in `empty.world` it read a scale of -0.01188 (standard
  error 0.00005) against the profile's -0.011858, and noise of 1.99, 2.65, 5.31,
  11.9, 26.3, 51.9 and 142 mm in the bands 0.65-0.8, 1.0-1.3, 1.3-1.8, 1.8-2.5,
  2.5-3.6, 3.6-5.0 and 5.0-8.1 m, against the model's 2.00, 2.66, 5.35, 12.0, 26.4,
  52.2 and 140 mm.
- **numpy.** The unit tests and the CI gate pass with `PYTHONNOUSERSITE=1`
  (numpy 1.21.5, the CI image's) as with the user's numpy 2.2.6.
- **Re-measured** with `tools/measure_cloud_timing.py` (physical_rosmaster's probe
  at 468662c, three 40 s runs, numpy 1.21.5): on the host GPU 993 cloud gaps with
  median 2, p95 11, longest 32 and mean 3.60 frames, 8.42 Hz, a 363 ms p95 period,
  a worst gap of 1.056 s and a 50 ms latency in every run; on llvmpipe 492 gaps,
  8.61 Hz on stamps and 50 ms. The depth image ran at 30.303 Hz on stamps. In
  `empty.world` 18.6% of its pixels are valid and the rest NaN, on both.

### Cost

Measured with `tools/measure_adapter_cost.py`, alternating `main` and this branch
five times each, `PYTHONNOUSERSITE=1` on both (numpy 1.21.5), on `empty.world`
under `physical`, over 30 s after 20 s of settling. Median, with the range in
brackets, in percent of one core on the Ryzen 5 3600:

| | GPU `main` | GPU step 7 | llvmpipe `main` | llvmpipe step 7 |
|---|---|---|---|---|
| Adapter CPU | 53.1 (46.8-54.1) | 54.0 (51.3-54.4) | 18.6 (17.9-19.0) | 20.3 (20.2-20.6) |
| Depth latency, sim ms | 6 (6-7) | 9 (8-9) | 34 (34-34) | 36 (35-36) |
| Real-time factor | 0.999 | 0.998 | 0.504 (0.501-0.508) | 0.496 (0.494-0.500) |
| Depth rate on stamps, Hz | 30.303 | 30.303 | 30.303 | 30.303 |

The conditioning costs about 2 ms of float32 arithmetic per frame (a benchmark on
one pinned core gave 2.1 ms, 6.5% of a core at 30.3 Hz): about 1 point on the GPU,
where the run-to-run drift is larger, and 1.7 points on llvmpipe. It delays the
depth image, which is published after it, by 2-3 ms of sim time. The GPU's
real-time factor is unchanged; on llvmpipe, with four cores shared, it falls
about 1.6% (0.504 to 0.496, the ranges do not overlap). About 40 points of the
adapter's CPU remain rclpy waking for every tick of the 1 kHz `/clock` (see
"Point cloud pipeline" above); that, and not the arithmetic, is the cost to cut.

### Deferred

- **The floor's dropout and the close-range wedge** of the physical sensor need an
  incidence-angle model. They are the follow-up #59, and the ledger's
  `depth.floor_returns` and `depth.close_range_wedge` rows are out of scope with a
  note naming it. The bottom third of the physical image never returns the floor;
  the simulator's floor returns from 0.6 m out.
- **The fraction of valid pixels** depends on the scene more than on the camera,
  so `depth.valid_fraction` and `depth.nan_fraction` are recorded, not compared.
  In `empty.world`, where only the floor returns, the simulator's is about 19%
  under `physical` (48.9% before, with the floor from 0.25 m); the robot's 38.5% is
  one room.
- **The far limit** of the physical camera has not been measured, so
  `depth.max_range_m` (the simulator's 8 m far clip) stays out of scope.

## Legacy mesh removal

The old STL visuals are no longer referenced after the CAD migration, but they
remain in the package while the new model is being validated.

- Potential benefit: removing them later saves about 33 MB.
- Risk: makes comparison and rollback less convenient during stabilization.
- Evaluate after: Gazebo, RViz, Classic expansion, and packaging checks have
  remained stable through review.

## Broader description contracts

A focused test protects unchanged links, joints, sensor frames, collisions,
inertia, plugins, and installed visual assets. It also protects the accepted
wheel-centre correction. Renderer masks and alternative state sources should
be added to the contract only when those changes are separately accepted.
