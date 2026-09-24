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
- [ ] Compare the physical camera's actual rate and latency with the simulator's
  current 5 Hz RGB-D output.
- [x] Decide whether to retain the website assembly's 10 mm rearward camera
  visual adjustment after measuring the physical robot. Dropped: without it the
  housing front sits 40.3 mm behind the chassis front, against the tape's 40 mm
  (#43 step 4).
- [ ] Re-evaluate the near clip, first visible ground points, self-occlusion,
  depth geometry, and point-cloud TF after any accepted camera change.

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
