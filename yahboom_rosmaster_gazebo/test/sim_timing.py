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
