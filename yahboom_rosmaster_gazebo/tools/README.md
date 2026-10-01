# Measurement tools

Scripts that measure the simulator, kept in the repository so that a
measurement can be repeated. They are not installed as nodes and are not
launched by the tests; run them from a source tree with ROS sourced. Each
starts, watches and stops its own simulator (`sim_run.py`) and needs none
running.

| Tool | Measures | Used for |
|---|---|---|
| `measure_cloud_timing.py` | physical_rosmaster's own `sensor_capability_probe.py`, run against the simulator on the camera group (default) or, with `--group stationary`, on the non-camera groups (LiDAR, IMU, odometry and health: rate, period, latency and reliability per topic, the scan's `scan_time` and `time_increment`, and the IMU and odometry series' spread). The camera group measures: the cloud's rate, gaps, latency and worst gap, the depth image's rate and latency, and the depth image's valid and NaN fractions | The `step_measurements` records of `config/real_robot_contract.yaml` |
| `measure_adapter_cost.py` | The camera adapter's CPU, the Gazebo server's, that of any node named with `--node`, the depth image's latency and rate, and the real-time factor, alternating two or more workspaces | Comparing a branch with `main` (#43 step 7, review item R5) |
| `sim_run.py` | (library) cleanup, detached launch, stop | Both |

## What `sim_run.py` does, and why

It follows the rules in `AGENTS.md`. Before every launch it kills leftover
simulator processes with anchored patterns (each starts with a bracketed
character so it cannot match the tool running it, and none is a bare word that
would reach a browser or the desktop's Xwayland), clears Fast DDS shared memory
and stops the ROS daemon. It starts `ros2 launch` in its own session with
`Popen(start_new_session=True)`, never a shell `&`. It stops it with SIGINT, then
SIGTERM after a stall, and last kills what remains of that session by pid, which
also removes the `Xvfb` of a killed `xvfb-run`. It never kills by name.

Isolation is `ROS_LOCALHOST_ONLY=1` with a domain and an `IGN_PARTITION` of its
own. Run only one simulator on the machine: two runs corrupt each other's timing
and each one's cleanup kills the other.

## Measuring the cloud and depth against the physical probe

```bash
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
python3 tools/measure_cloud_timing.py \
  --workspace ~/Documents/rosmaster_ws_step7 \
  --physical-repo ~/Documents/air-club/physical_rosmaster \
  --output step7_camera_gpu.json
```

It runs `--group camera --duration 40 --camera-frame-rate 30.30303 --per-message`
three times in one launch, on `empty.world`, headless, under
`sensor_profile:=physical` with a random seed, and pools
`frame_cadence.frames_per_gap` across the runs. `--render llvmpipe` reproduces
CI's software rendering pinned to four CPUs; the rates then read low on wall time,
but the stamp-based and sim-time figures hold. `--no-user-site` hides numpy in
`~/.local`, to use the distribution's, as CI's image does.

### What it pins

The probe is not vendored. `fetch_probe()` reads two files from the pinned
physical_rosmaster commit (`PHYSICAL_PIN`, a full SHA, the merge commit of #45)
with `git show`, and never checks the repository out:

* `tools/sensor_capability_probe.py`
* `tools/physical_contract_probe.py`, which the first imports, so they sit side
  by side in the work directory.

It then applies `patches/sensor_capability_probe_sim_time.patch` to the copy.
The patch is seven lines: the node runs with `use_sim_time`, and receipt is
stamped with the ROS clock, so `latency_ms` is sim time at receipt minus
`header.stamp`. The unpatched probe stamps receipt with `time.time()` and reads
wall epoch minus sim time, which is meaningless against sim-time stamps (#43
step 2). The copy keeps physical_rosmaster's Apache-2.0 header. A patch that no
longer applies stops the run, so a change upstream cannot silently change what is
measured. To move the pin, change `PHYSICAL_PIN`, re-run
`test/measurement_tools_test.py`, and re-measure.

## Measuring the stationary sensors

```bash
python3 tools/measure_cloud_timing.py --group stationary --no-user-site \
  --workspace ~/Documents/rosmaster_ws_step8 --output step8_stationary_gpu.json
```

It runs `--group lidar --group imu --group odom --group health --duration 40
--per-message` three times in one launch with the same pinned, patched probe,
and reports each topic on stamps (sim time). The probe exits 1 when a topic it
asked for stayed silent, which the simulator's missing hardware topics
(`/imu/mag`, `/vel_raw`, `/voltage`, `/diagnostics`, `/scan_filtered`) make
normal: the harness takes that exit as a result and lists those topics as absent.

## Comparing a branch with main

```bash
python3 tools/measure_adapter_cost.py --render gpu --runs 5 --no-user-site \
  --variant main=~/Documents/rosmaster_ws \
  --variant step7=~/Documents/rosmaster_ws_step7
```

Add `--node MARKER` (repeatable) to report the CPU of other processes by a
substring of their command line, for example `--node wheel_state_odometry`.

Variants are built workspaces, run alternately (A, B, A, B, ...) so a change in
the machine's load touches both. The report gives n, the median and the range of
each figure per variant, and the numpy each run used.

## Reading the results into the ledger

`config/real_robot_contract.yaml` records, per step, the commit measured and the
keys it re-measured (`simulator.provenance.step_measurements`). A key is in one
record only: a step that re-measures it takes it over from the older record.
`doc/real_robot_contract.md` says how to add a record.
