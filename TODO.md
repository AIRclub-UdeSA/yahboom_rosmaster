# TODO

This list tracks follow-up work intentionally left for later iterations. See
[Deferred simulator changes](yahboom_rosmaster_gazebo/doc/deferred_simulator_changes.md)
for the observations, tradeoffs, measurements, and validation criteria behind
these items.

## Camera and real-robot alignment

- [x] Identify the camera and mount installed on the physical robot. An
  Orbbec Astra (serial ACRC64300ET) on physical_rosmaster's
  `camera_mount_joint` (#43 step 4).
- [x] Measure the lens centre and pitch relative to `base_link`; do not infer
  functional sensor extrinsics from the camera housing mesh. The simulator uses
  the physical robot's frames; the tape confirms their height and setback to
  about 2 mm. The pitch is the physical description's nominal zero and has not
  been measured separately (#43 step 4).
- [x] Compare the physical camera's actual rate and latency with the simulator's
  current 5 Hz RGB-D output. The simulator now runs the physical 30 Hz. Its
  intrinsic image latency, 20-21 ms of sim time on a GPU, already exceeds the
  physical 2-6 ms, so none is added (#43 step 5). The cloud's timing is modelled
  separately (#43 step 6, below).
- [x] Decide whether to retain the website assembly's 10 mm rearward camera
  visual adjustment after measuring the physical robot. Dropped: without it the
  housing front sits 40.3 mm behind the chassis front, against the tape's 40 mm
  (#43 step 4).
- [x] Re-evaluate the near clip, first visible ground points, self-occlusion,
  depth geometry, and point-cloud TF after any accepted camera change. Done for
  #43 step 5: no robot pixels, the bottom row sees the floor 0.245 m away,
  `depth_geometry` checks the new render origin by parallax, and the cloud is
  transformed into `cam_1_depth_frame`. Repeat for later camera changes.

- [x] Match the physical point cloud: its 16-byte NaN-stripped layout, its
  gaps of whole 30 Hz frames and its 50 ms latency, under
  `sensor_profile:=physical` (#43 step 6). `depth_geometry` and the ledger
  record how; the depth noise, scale and dropouts are step 7.
- [ ] Cut the camera adapter's CPU. About 40 of its 53 points of a core on the
  host GPU are the executor waking for every tick of the 1 kHz `/clock`; see
  "Point cloud pipeline" in the deferred changes for the options. The
  real-time factor is unaffected today.
- [ ] Re-pin `physical.provenance.commit` in the parity ledger to the merge
  commit of physical_rosmaster#45. It pins that PR's head until then.

## CAD model validation and cleanup

- [ ] Validate the Donatello visuals in the empty and cafe Fortress worlds and
  in RViz, including wheel placement and rotation.
- [ ] Check RGB, depth, point-cloud, and LiDAR output while the robot is moving,
  with particular attention to lag and self-occlusion.
- [ ] Expand and statically validate the Gazebo Classic model, then smoke-test
  it on Apple Silicon when that platform is available.
- [ ] Remove the unreferenced legacy STL visuals once the new model has remained
  stable through review and rollback is no longer needed.

## Physics and runtime architecture

- [x] Correct the physical wheel joint and collision centres to match the
  measured real-robot geometry and the drive plugin's 0.169 m wheel separation.
- [ ] Evaluate Gazebo's native joint-state publisher as a separate architectural
  change, including publication rate, headers, startup, and shutdown behavior.
- [x] Evaluate renderer visibility masks only if raw sensor messages show robot
  self-occlusion that cannot be resolved with measured sensor placement. Not
  needed for the camera: raw images at its physical mount show no robot pixels
  (#43 step 4).
- [ ] Investigate startup ordering or readiness checks if repeated cold starts
  show persistent bridge or message-filter failures.

## Automated contracts

- [x] Add a focused description contract test covering links, joints, sensor
  frames, collisions, inertia, plugins, wheel origins, and installed visuals.
- [ ] Repeat motion-profile, wheel-odometry, TF, LiDAR, RGB-D, IMU, and headless
  launch checks after each accepted physics or sensor change.
