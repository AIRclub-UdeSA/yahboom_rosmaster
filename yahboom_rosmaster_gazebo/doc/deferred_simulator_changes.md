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
30 Hz.

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
camera panel is configured as image-only instead.

## Startup timing changes

Starting wheel state and odometry before sensor bridges, or delaying RViz, can
reduce transient message-filter warnings on slow machines.

- Potential benefit: cleaner first seconds after launch.
- Risk: hides an underlying startup fault and increases launch complexity.
- Evaluate with: repeated cold starts on supported platforms and explicit
  readiness checks instead of fixed delays.

## Camera rate and optical extrinsics

The simulator now publishes its RGB-D outputs at `5 Hz`. The physical robot's
current setup guide starts `usb_cam` at `10 Hz`, so rate and latency should be
measured on the robot before treating either value as a Sim2Real requirement.

The physical description places `camera_link` at
`(0.057105, 0.000017948, 0.03755) m` relative to `base_link`. That value locates
the camera housing mesh; it is not a measured lens or optical-frame transform.
The current USB-camera launch publishes `default_cam` and does not connect that
frame to `base_link`, while the Astra launch in the repository is disabled.

The model comparison also explains the visible gap before the point cloud:

- The former D435 visual ended at base-link `x = 0.105 m`, coincident with the
  unchanged functional camera origin.
- The Donatello camera housing ends at `x = 0.0662 m`, placing its front 38.8 mm
  behind the functional origin.
- The sensor's 0.05 m near clip puts the first possible sample about 88.8 mm
  ahead of the visible housing.
- The website assembly deliberately moved the camera 10 mm rearward. Undoing
  that adjustment would improve agreement with the physical repository's
  nominal housing placement, but it would not calibrate the optical frame.

The legacy chassis STL is byte-identical to the body mesh in the physical
repository, while the Donatello camera and LiDAR envelopes match its sensor
meshes and the wheels agree within about 0.2 mm. Neither complete visual should
be treated as the measured robot envelope until the real unit is checked.

Do not move the simulator's functional camera frame to the physical
`camera_link` value without a real-unit measurement. First identify the camera
and mount actually installed, measure the lens centre and pitch relative to
`base_link`, then refine the extrinsics with a calibration target or a known
floor plane. Re-evaluate camera near clipping, ground coverage, robot
self-occlusion, depth geometry, and point-cloud TF after any accepted change.

Keep these as separate later decisions: the selected simulator rate, the
10 mm visual offset, the functional optical transform, and any future collision
envelope. A measurement may justify one without justifying the others.

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
