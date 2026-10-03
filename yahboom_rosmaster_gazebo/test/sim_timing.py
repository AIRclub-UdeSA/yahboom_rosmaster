#!/usr/bin/env python3
"""
Shared probe start times for the simulator launch tests.

The simulator launch starts its bridges BRIDGE_START_DELAY seconds in, and its
controller, wheel odometry and ground-truth node as soon as the robot has been
spawned, about 3.5 s in with software rendering.
"""

# A margin over both of those, not a measured readiness time: the probes that use
# it wait for the topics they need themselves. It comes from llvmpipe timings, so
# check it against the launch again if either start time moves.
PROBE_START_DELAY = 8.0

# The performance checks time each topic's first message from probe start, so
# they need a probe that starts once every sensor is already publishing.
PERFORMANCE_PROBE_START_DELAY = 15.0

# The physical contract probe starts when sensor_contract_probe exits, so the
# sensors are publishing and TF is flowing. It grades the simulator with
# target:=simulator, on sim time. Ten samples, not the default five: at five the
# cloud's 3 Hz floor (median period of four gaps) fails by chance about 1.8% of
# the time, and at ten about 6e-5 (#43 step 9). The timeout is wall seconds: the
# probe took 3.7 to 7.4 s under the CI gate's llvmpipe recipe (real-time factor
# 0.47-0.49; 5 fresh launches, and 4.2 to 5.5 s inside 5 gate runs) and 2.5 to
# 5.6 s on the GPU (30 launches), so 30 s is a hang guard four times the slowest.
PHYSICAL_PROBE_PARAMETERS = {
    "target": "simulator",
    "use_sim_time": True,
    "samples": 10,
    "timeout": 30.0,
}
