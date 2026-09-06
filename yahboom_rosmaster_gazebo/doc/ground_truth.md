# Ground truth and localization frames

## Raw measurement contract

`/ground_truth/odom` is the Gazebo model pose expressed as
`world -> base_footprint`. It is timestamped in simulation time and remains a
measurement-only topic for evaluation, bags, and tests. Gazebo's pose output is
not bridged into the robot TF tree, and no robot-facing component should use
ground truth for navigation or state estimation.

The robot estimate remains independent:

- `/odom` and `odom -> base_footprint` come from wheel-state odometry;
- a localization or SLAM system may publish `map -> odom`;
- `/calc_odom` is only a command-integrated reference.

## RViz diagnostic frame

`ground_truth_tf.py` publishes the separate `ground_truth_base` frame for
visual comparison. It does not reuse the raw world coordinates as odom
coordinates. Instead, when synchronized wheel odometry and ground truth first
become available, it captures the fixed alignment

```text
odom_T_world = odom_T_base(initial) * inverse(world_T_base(initial))
```

This makes nonzero Gazebo spawn poses safe and preserves subsequent physical
divergence between wheel odometry and truth.

The default `ground_truth_frame:=auto` mode initially publishes
`odom -> ground_truth_base`. If a `map -> odom` transform later appears, the
helper captures it once:

```text
map_T_world = map_T_odom(first) * odom_T_base(first)
              * inverse(world_T_base(first))
```

It then publishes `map -> ground_truth_base` using that fixed alignment. Later
SLAM corrections to `map -> odom` move the estimated robot but do not move
ground truth. Using a synchronized robot pose here also handles SLAM starting
after the robot has already moved. This is the intended comparison.

Use `ground_truth_frame:=odom` to keep the diagnostic frame in odom even if a
map appears. Use `ground_truth_frame:=map` to wait for map rather than publish
the initial odom form. Another connected localization frame can also be named
explicitly.

Because the automatic mode switches the diagnostic child's parent once when a
map first appears, it is for visualization and evaluation only. If a SLAM
process is deliberately restarted with a new map coordinate system, restart
the helper as well; an ordinary ongoing `map -> odom` correction must not cause
realignment.

## Regression coverage

The ground-truth tests protect three independent properties:

- the raw topic remains `world -> base_footprint`, has one publisher, uses
  simulation timestamps, and never claims the robot's TF frames;
- a nonzero world spawn aligns with the initial odometry origin;
- after the first synthetic `map -> odom` alignment, a later large SLAM
  correction does not move the truth frame, while new Gazebo motion still does.

The synthetic map test does not replace an end-to-end run with the challenge's
SLAM package. That integration run remains a follow-up check.
