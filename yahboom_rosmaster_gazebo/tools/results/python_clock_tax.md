# Python nodes and the 1 kHz /clock: measurements

Detail behind the issue "three Python nodes spend ~140% of a core following the 1 kHz
/clock". Harness: `tools/measure_python_node_cpu.py`, `tools/idle_node.py`,
`tools/report_python_node_cpu.py`. Raw reports: `tools/results/data/` (one JSON per
configuration; `old_harness/` keeps four early runs whose wall-second CPU figures are
valid but whose real-time factor, per-sim-second figures and maximum latency are not, see
"Harness bug"). Prototypes (C++ nodes, launch switches): branch
`scratch/camera-adapter-cpu-proto`. Nothing here is implemented on main.

## Method

Hardware: Ryzen 5 3600 (6 cores / 12 threads), RX 5600/5700, native Ubuntu 22.04, ROS 2
Humble, Gazebo Fortress 6.18. Default headless launch (`headless:=true rviz:=false`,
`motion_bias:=false`), `sensor_profile:=physical` unless stated, seeds 1000+run. Each run
is a fresh launch: 20 s settle, a `/clock` subscriber query, 3 s drain, then a 30 s
wall-clock window. CPU is read from /proc (process and per-thread ticks over wall time),
percent of one core. Variants ran alternately (A, B, C, A, B, C, ...), n = 5 each unless
stated; cells are median (min-max). `PYTHONNOUSERSITE=1` (numpy 1.21.5, as CI) unless
stated. Sim time versus wall time: GPU real-time factor 0.96-1.00 in every variant, so
per-wall and per-sim figures agree; under llvmpipe (CI recipe: xvfb,
`LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe LP_NUM_THREADS=4`, `taskset -c 0-3`) the
factor is 0.48 (empty) and 0.43 (cafe), so both are given. No profiler: `perf` is blocked
by `kernel.perf_event_paranoid=4` and py-spy was not installed; the split comes from
per-thread /proc ticks and from an idle node pair (one rclpy node on sim time, one on wall
time, no callbacks).

### Harness bug, fixed (c68e47f)

A blocking subprocess left the probe's own sim clock ~13 s stale, which inflated the
real-time factor to 1.45 on a GPU that runs at 1.0 and made the first five clouds of every
window (the sensor QoS depth) read 100-500 ms late: the "350-1180 ms maximum cloud latency"
seen in the first runs. Not the camera's start-up burst. The probe now drains for 3 s
before the window; the maximum is 51-63 ms. Every figure outside `old_harness/` is from the
fixed harness.

## 1. Where the adapter's CPU goes

GPU, empty.world, physical, n = 5 (`data/f_gpu_empty.json`):

| Process | % of one core | Main thread | Other threads (Fast DDS) |
|---|---|---|---|
| camera_adapter.py, stock (sim time) | 52.2 (51.3-52.4) | 47.9 | ~3 |
| camera_adapter.py, `use_sim_time` false, `ideal` profile (`old_harness/`) | 21.4 (20.7-23.0) | 20.5 | ~1 |
| camera_adapter.py, behind the delay node (no clock, no timers) | 15.5 | | |
| idle rclpy node, sim time, no callbacks | 30.3 (30.0-34.4) | 27.1 | ~2.2 |
| idle rclpy node, wall time | < 0.5 | | |
| calculated_odometry.py | 39.8 (38.9-42.1) | 36.0 | |
| ground_truth_tf.py | 47.1 (46.5-48.3) | 42.7 | |
| Gazebo server (`ign`) | 91.0 (90.0-108.3) | | |

- Same adapter, sim time on versus off, `ideal` profile (n = 5 each): 54.7 (49.7-55.1)
  versus 21.4 (20.7-23.0). **The clock costs the adapter ~33 points, 60% of its CPU.** Its
  own work is ~15-21 points.
- An idle sim-time node, doing nothing, costs 28-36 points across the configurations
  below, 90% of it in the Python main thread (message deserialization and the executor
  wakeup per tick), not in DDS.
- Stock adapter / idle sim-time node, other configurations: GPU cafe 52.6 / 35.6; GPU empty
  with the user-site numpy 2.2.6, `ideal` 53.9 / 35.5, `physical` 53.9 / 37.2; llvmpipe
  empty 19.8 / 13.7 (41.2 / 28.5 per sim second); llvmpipe cafe 18.3 / 12.4 (43.2 / 28.7
  per sim second). numpy 1.21.5 versus 2.2.6 makes no difference to the adapter (within 2
  points).
- The adapter needs sim time for one thing: the 50 ms cloud delay (`latency_s`,
  camera_adapter.py:732-750). Under `ideal` the latency is 0.

## 2. Python nodes on sim time in a default launch

`ros2 topic info /clock -v`, union of two queries per run:

| Node | Language | % of one core (GPU empty) | Needs sim time for |
|---|---|---|---|
| camera_adapter | rclpy | 52.2 | the 50 ms delay of the cloud, nothing else |
| ground_truth_tf | rclpy | 47.1 | nothing: stamps come from the odom message; its 20 Hz timer, `time.monotonic()` timeouts and TF lookups (explicit stamps, `Time()` = latest) do not use the node clock. Its launch tests already run it on wall time |
| calculated_odometry (node name `cmd_vel_odometry`) | rclpy | 39.8 | the cmd_vel timeout and the dt of its 50 Hz integration |
| robot_state_publisher, controller_manager, joint_state_broadcaster | C++ | cheap | |

Not on sim time and cheap: cmd_vel_watchdog 1.6, wheel_state_odometry 1.0, imu_raw_relay
0.7, sensor_qos_relay 6.5. The three Python nodes: 139 points, ~90 of them the clock.

## 3. Variants (switches of the prototype launch file)

| Label | Switches |
|---|---|
| gt | `gt_clock:=wall` |
| delay | `adapter_clock:=delay_node` (rclcpp node holds each cloud until `/clock` reaches stamp + 50 ms; the adapter runs on wall time and publishes undelayed on `/internal/cam_1/depth/color/points`) |
| thr100 / thr200 / thr500 | `adapter_clock:=throttle clock_throttle_hz:=100/200/500` (a C++ node republishes `/clock` on `/clock_throttled` at the first message at or after each grid point; the adapter is remapped to it) |
| all200 | `adapter_clock:=throttle calc_clock:=throttle` |
| mix_thr | all200 + `gt_clock:=wall` (latency trim 0 in the first runs) |
| mix_thr_trim | mix_thr with the latency target reduced by half the throttle period (2.5 ms at 200 Hz; `latency_trim_ms:=2.5`, now `auto`) |
| mix_delay | `adapter_clock:=delay_node calc_clock:=throttle gt_clock:=wall` |
| mix_*_raw | the same with `throttle_impl:=raw` (serialized, no deserialization) |
| mix_delay_rel | mix_delay with `hop_reliable:=true` |

### GPU, empty.world, physical (`data/f_gpu_empty.json`, real-time factor 0.98)

| Variant | adapter | calc_odom | gt_tf | C++ node(s) | session | latency median / p95 / max (ms) |
|---|---|---|---|---|---|---|
| main | 52.2 | 39.8 | 47.1 | | 293.1 | 50 / 53 / 60 |
| gt | 50.9 | 38.5 | 12.0 | | 254.8 | 50 / 52 / 63 |
| all200 | 23.8 | 13.3 | 44.9 | 6.8 | 240.3 | 52 / 55 / 58 |
| delay | 15.5 | 38.4 | 45.9 | 6.6 | 258.3 | 50 / 50 / 51 |
| thr200 | 24.0 | 38.4 | 45.5 | 6.7 | 265.8 | 52 / 55 / 57 |
| mix_delay | 14.9 | 12.8 | 11.5 | 6.4 + 6.5 | 202.0 | 50 / 50 / 58 |
| mix_thr | 24.2 | 13.0 | 11.3 | 6.6 | 204.5 | 52 / 55 / 57 |
| mix_thr_raw | 23.8 | 13.4 | 11.5 | 6.2 | 202.7 | 52 / 55 / 59 |
| mix_delay_raw | 13.6 | 13.3 | 11.7 | 6.4 + 6.1 | 200.5 | 50 / 50 / 51 |

Earlier adapter-only runs of thr100 / thr200 / thr500 (`data/old_harness/gpu_empty_phys_proto.json`; harness
version before the drain fix, wall-second CPU valid): adapter 22.8 / 27.2 / 40.3, latency
median 55 / 52 / 51 ms. 100 Hz is out (55 ms is at the tolerance edge); 500 Hz keeps most
of the tax.

### The other configurations (session CPU, median)

| Config (file) | main | mix_thr / trim | mix_delay | notes |
|---|---|---|---|---|
| GPU empty, after a reboot (`g_gpu_empty`) | 333.6 | 276.6 / 277.9 | | adapter 31.8 and node 8.9 here, `ign` 114 vs 91: the machine was in a different state; trim: latency 50 / 52-53 / 53 |
| GPU cafe (`f_gpu_cafe`) | 349.6 | 289.6 (raw) | 290.5 | `ign` 131 stock vs 141-146 in the others at the same real-time factor: unexplained. Python-node saving: adapter 52.7 -> 20-31, calc_odom 43.7 -> 15, gt_tf 50.0 -> 13 |
| llvmpipe empty (`f_ll_empty`) | 233.9 | 207.4 | 206.7 | 0.48 real-time factor; adapter 19.8 -> 9.6, calc_odom 15.1 -> 5.2, gt_tf 17.4 -> 4.7; per sim second the adapter is 41.2 -> 19.7 (thr) / 11.8 (delay) |
| llvmpipe cafe (`g_ll_cafe`) | 238.3 | 212.3 / 211.7 | 213.8 | 0.43 real-time factor; adapter 17.6 -> 9.2; trim: latency 50 / 53 / 54 |
| llvmpipe cafe, reliable hop (`h_ll_cafe`) | 237.5 | | 212.9 (mix_delay_rel) | latency 50 / 52 / 62 |

The Gazebo server's own CPU is not constant across variants in some configurations (cafe
on the GPU, and across the two days), so session totals understate the Python saving there.
The per-node columns are the reliable figure; the 56-89 point spread of the GPU session
saving comes from this drift.

## 4. The Best Effort hop of the delay node drops clouds under load

With the hop Best Effort (the adapter's default sensor QoS), the delay node receives
77-81% of the clouds the adapter published under llvmpipe + cafe: 186 of 231, 215 of 264,
189 of 255, 196 of 248, 174 of 233 (the node's own received = published, so nothing is
lost after it). At the probe, final versus internal loss is 13-26% (median 19-21% in two
independent configurations) and the cloud rate falls to 6.5 Hz (median) against 7.9 for
stock. GPU cafe loses 1-3%; llvmpipe empty and GPU empty lose none. With a Reliable
(depth 10) hop: 260 of 260, 255 of 255 received, rate 8.19 Hz against 8.29 for stock, no
extra CPU (delay node 2.4% under llvmpipe).

## 5. Estimates

- (c) C++ port of the whole adapter: 793 lines, ~250 of them numpy math (seeded
  conditioning, back-projection, rotation, packing with NaN stripping and decimation) and
  ~150 image pairing and gap gating. Would remove the Python and clock cost (~15-21 points
  of own work to roughly 5-8, plus a 1 kHz C++ clock subscription at 6.5% on the GPU unless
  it uses the throttle). 1.5-2 weeks. The risk is the seeded depth noise and gap draws:
  numpy's RNG streams cannot be reproduced bit for bit in C++, so `depth_quality_contract`,
  `cloud_timing_contract` and the seed-determinism claims would have to be re-derived.
- (d) C++ port of `calculated_odometry` (168 lines): would replace 13-15 points with ~2-5;
  about half a day. Merging the three Python nodes into one process saves two of three clock
  subscriptions but entangles unrelated nodes. Throttling `/clock` at the bridge is out:
  `controller_manager` and Gazebo-side consumers need the 1 ms clock. A raw-serialized
  throttle saves under 1 point (6.2 against 6.6).

## 6. Ledger check (GPU, empty.world, `measure_cloud_timing.py`, 3 runs, `data/ledger/`)

Control (main) and recommended variant (`adapter_clock:=throttle calc_clock:=throttle
gt_clock:=wall latency_trim_ms:=2.5`), camera and stationary groups:

| Value | Ledger | main | recommended |
|---|---|---|---|
| cloud latency_ms, median (run medians) | 50 +/- 5 | 50.0 (50, 50, 50) | 50.0 (50, 50, 50) |
| cloud latency p95 (not in the ledger) | | 50 | 52 |
| cloud period_p95_ms | 367 | 363 | 363 |
| cloud gap_frames median / p95 / longest / mean | 2 / 11 / 34 / 3.63 | 2 / 11 / 34 / 3.67 | 2 / 11 / 34 / 3.75 |
| frames_per_gap histogram | | 983 gaps | 963 gaps (random seeds) |
| depth image rate on stamps | 30.3 | 30.303 | 30.303 |
| depth image latency, valid fraction | | 9 ms, 0.1863 | 9 ms, 0.1862 |
| /odom, /joint_states on stamps | 9.80 | 9.804 | 9.804 |
| /imu/data_raw, /imu/data | 10 | 10.0 | 10.0 |
| /scan rate, scan_time, time_increment | 7.19 | 7.194, 0.1343, 0 | 7.194, 0.1343, 0 |

Output of the nodes whose clock changes (n = 5 per variant, 30 s, GPU empty,
`--streams`): `ground_truth_tf` on wall time against sim time: `ground_truth_base` TF
50.0 Hz on stamps in every variant, 100% of its stamps equal a `/ground_truth/odom` stamp,
frames `odom -> ground_truth_base`, one `Captured fixed odom <- world` log line each,
counts 1467 (wall) and 1468 (sim). `/calc_odom` on the 200 Hz clock: 50.0 Hz, max gap
20-21 ms, count 1465-1480 against 1470 stock; its stamps lie on the 5 ms grid. The
map <- world alignment path was not exercised (no SLAM in the default launch). On the
prototype workspace `launch_argument_consistency`, `ground_truth_alignment_math`,
`ground_truth_contract`, `ground_truth_display`, `ground_truth_odom_mode`,
`ground_truth_map_mode`, `camera_adapter_contract`, `cloud_timing_contract` and
`depth_quality_contract` pass (9 of 9); the ground-truth launch tests already run the node
on wall time, so they do not prove the sim-time launch setup, the stream comparison does.

## 7. Caveats

- n = 5 per variant per configuration (3 for the ledger). Two days of the machine (before
  and after a reboot) differ in the Gazebo server's and the throttled adapter's absolute
  numbers; the ordering and the per-node savings are the same on both. The rise of the
  Gazebo server's CPU in some mixed variants is reported, not explained.
- Runs of three early configurations (GPU cafe, numpy 2.x) overlapped a mistaken `pkill`
  of the measuring session's own shells (it killed only shells, not simulator processes)
  and a polling loop; they were discarded and re-run.
- Latency is the probe's sim time at receipt minus `header.stamp`; the clock the probe
  reads is itself sim time, so its resolution is 1 ms.

## Reproducing

```bash
python3 tools/measure_python_node_cpu.py --render gpu --world empty.world \
  --profile physical --runs 5 --idle-nodes --no-user-site --streams \
  --count-topic /internal/cam_1/depth/color/points \
  --variant main=~/Documents/rosmaster_ws_cpu \
  --variant mix_thr=~/Documents/rosmaster_ws_cpuproto \
  --variant-arg mix_thr:adapter_clock:=throttle --variant-arg mix_thr:calc_clock:=throttle \
  --variant-arg mix_thr:gt_clock:=wall --output out.json
python3 tools/report_python_node_cpu.py out.json
```
