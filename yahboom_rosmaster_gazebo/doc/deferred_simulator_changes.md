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
  `sensor_adapter.py` does, and keeps Gazebo's organized 24-byte layout for
  step 6 to replace. It costs 1.6 ms per cloud. Back to back on the host GPU,
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
