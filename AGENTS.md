# Working rules for coding agents

Build, test and contribution basics are in README.md ("Development Checks") and
CONTRIBUTING.md. This file only covers the traps that cost agents time here.

## Ground truth
- The simulator mirrors the physical ROSMASTER X3 (AIRclub-UdeSA/physical_rosmaster).
  `yahboom_rosmaster_gazebo/config/real_robot_contract.yaml` (the parity ledger) is
  the single source of expected values. Probes and tests read rates, frames,
  intrinsics and mounts from it. Never hard-code them.
- Every value that claims parity cites a physical commit and artifact.
- A `matches_physical: true` entry must be comparable: scalar against scalar, with
  an explicit `tolerance`. `real_robot_contract_test` rejects a true flag it can't
  verify.
- `step_measurements` records the full SHA of the commit that was measured.
- physical_rosmaster is a read-only reference. Read it with
  `git show origin/<branch>:<path>`. Its code is Apache-2.0: port the behavior, and
  if you copy code, keep its header.
- Issue #43 sets the order of the parity work: one PR per step.

## Before launching the simulator
- Isolate the sim: `export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=<unique>`. For manual
  launches also set `IGN_PARTITION`: two sims with the same world name share
  `/world/<name>/control`.
- Kill leftover sim processes and clear Fast DDS shared memory before EVERY launch.
  A stale robot_state_publisher feeds gz_ros2_control an old robot_description.
  - Use anchored patterns only: `pkill -f ign` also matches Firefox and system
    services.
  - Bracket the first character so the pattern doesn't match the shell running it:
    `pkill -9 -f '[/]usr/bin/ign gazebo'`, `'[r]os2 launch'`,
    `'[r]osmaster_ws/install/'`.
  - Then `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*` and
    `ros2 daemon stop` (with ROS sourced).
- Self-kill trap: `pkill -f` also kills the agent's own shell when that shell's
  command line contains the pattern (a heredoc, a `source .../install/setup.bash`,
  or even a grep). Write cleanup and launch commands into script files and run
  them as `bash file`.
- Never start `ros2 launch` with bash `&`: the child inherits SIGINT-ignored, and
  the launch can't be stopped cleanly. Use Python
  `subprocess.Popen(..., start_new_session=True)`. A killed `xvfb-run` leaves its
  `Xvfb` behind; kill that by PID.
- Run only one Gazebo at a time on a machine. Parallel runs corrupt each other's
  timing numbers, and each one's cleanup kills the other's run.

## Testing
- Unit tests are standalone scripts: `python3 test/<name>_test.py`.
  `python3 -m unittest test.<name>` imports the stdlib `test` package instead.
- Read results from CTest's own output (`--event-handlers console_direct+`).
  `colcon test-result` counts stale XML and doesn't see plain `add_test` targets.
- Run `ament_flake8` and `ament_pep257` on the whole package, unpiped. A `| tail`
  once hid a real error.
- Reproduce the CI gate locally under software rendering:
  `xvfb-run --auto-servernum --server-args="-screen 0 1280x1024x24 -nolisten tcp"`,
  `LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe LP_NUM_THREADS=4`,
  `taskset -c 0-3`, then `bash scripts/test_simulator_contracts.sh`.
- For a behavior change, also run the full suite on a GPU, sequentially, including
  the strict `sensor_contract_empty` and `sensor_contract_cafe`.
- Show that a new test catches the bug: it fails before the fix and passes after.
  For a behavior change, break the code on purpose and check the test fails.
- A probe must not import or copy the math of the code it checks. A shared bug
  would pass both.
- Known intermittent shutdown and start-up failures are tracked in #55, with their
  signatures. Re-run the target once and record it. Never loosen an exit-code or
  rate assertion to make a flake pass. Root-cause it with instrumentation instead
  (#58 was found with a waitpid tracer).
- Anything expected to take over ~2 min (test gates, repeat loops, CI waits) runs
  with `run_in_background: true`, output to a log under the scratchpad. Wait for the
  completion notice. To watch progress, use `Monitor` with
  `until <check>; do sleep 2; done`. Never block a foreground call on
  `until … sleep`, `timeout N tail --pid` or `sleep N; cmd`: the harness rejects the
  last form and the others stall the turn for minutes.

## Measuring
- Measure timing, CPU and real-time-factor claims on the machine at hand. Run the
  two variants alternately. Report n, median and range, the hardware, and sim time
  versus wall time.
- For rare events, give counts and a Fisher exact p. "0 of N" means below 3/N at
  95% confidence, so confirming a ~2% flake is fixed takes about 150 runs.
- Commit any measurement harness you'd need again. Scratchpads under /tmp vanish on
  reboot; the step-6 cloud-timing harness was lost that way.

## Git and pull requests
- No AI attribution anywhere: no Co-Authored-By trailer, and no "Generated with…"
  footer in commits, PR titles or PR bodies. The required `check-commit-messages`
  job rejects them. It doesn't re-run when you edit a PR body, so grep the body
  yourself.
- If `gh pr view` or `gh pr edit` fails with a Projects (classic) GraphQL error, use
  `gh api` (REST). After a body PATCH, diff the remote text against your file.
- `bind_pr` failed 6 of 6 times here, within 30 s of opening the PR ("could not read
  it as an open pull request"). Try it once; if it fails, don't retry or debug it.
  Carry on with `gh api`.
- Merge with "Create a merge commit". Never rebase a branch after its ledger
  measurement: GitHub's rebase-merge always rewrites SHAs, and `step_measurements`
  then points at commits that aren't on main.
- Any push dismisses approvals. Make all review fixes before requesting review.
  Answer every review comment with what was done, or why not.
- Stacked PRs: `delete_branch_on_merge` is off. After merging the base, delete its
  branch, which retargets dependents to main, or retarget by hand. Never click
  merge on a PR that still targets a feature branch. Retargeting doesn't start CI;
  close and reopen the PR to get a run against main.
- Only `check-commit-messages` is required. Confirm the
  "ROS 2 Humble / Gazebo Fortress" job passed yourself. CI doesn't run on pushes to
  main, so after merging several PRs, run the workflow on main with
  workflow_dispatch.
- Keep unrelated fixes out of a feature PR. The shutdown fix was split out of the
  camera PR into #50.
- A behavior change updates the README's "Working ROS Interfaces" and
  "Current Project Status" in the same PR, and flags breaking changes at the top of
  the PR description.

## Design rules
- `sensor_profile:=physical|ideal` changes quality and timing only, never the
  interface (topics, frames, message layout). Geometry and determinism tests use
  `ideal`. Profile values live in `config/sensor_profiles.yaml` with their source.
- Nodes that use numpy must not fan out into multithreaded BLAS. Use element-wise
  arithmetic and set `OMP_NUM_THREADS=1`: a matrix product once took 8.5 cores.
- Don't change the launch file's shutdown sequence without the shutdown test: stop
  the bridges, pause the world, request the stop, and force-kill only on a stall.

## Gazebo Fortress and ROS 2 Humble facts learned the hard way
- The RGB-D camera ignores an off-centre principal point and renders a symmetric
  frustum. Publish ((w − 1)/2, (h − 1)/2).
- With the 1 ms physics step, a 30 Hz sensor stamps every 33 ms (30.3 Hz). Use the
  measured period, not 1/30 s.
- The RGB-D camera emits a burst of catch-up frames at start-up.
- Gazebo computes its native point cloud whether or not anything subscribes.
- Large (~600 kB) Best Effort messages get dropped under CPU load.
- In Humble, `IncludeLaunchDescription` doesn't scope launch configurations, so
  command-line arguments reach included files. Still forward shared arguments
  explicitly (`launch_argument_consistency_test` checks both entrypoints).
- A threaded process shows `/proc` state `Z` about 70 ms before it can be reaped.
  Don't treat a zombie as stopped (#58).
- Gazebo accepts `--` inside XML comments. Python and xmllint don't, and the
  shutdown code parses world names with Python. Keep `.world` files well-formed.
- An rclpy node using sim time wakes on every tick of the 1 kHz `/clock`, which
  costs about 40% of a core in Python.

## Working in parallel sessions
- A parallel session gets its own git worktree and colcon workspace, and extends
  the cleanup patterns to cover that workspace's path. Until the machine is free,
  it only edits code and runs pure unit tests: no builds, no launches.
- For large changes, write a short design note and wait for approval before coding.
- Don't touch branches or worktrees you weren't assigned. Posting comments, filing
  issues and merging need the maintainer's OK.
- Check premises before acting on them, including reports from other sessions and
  review claims. Two claims in this work turned out wrong: that the #54 spawn
  arguments weren't forwarded, and that every mount value has six decimals.
