#!/usr/bin/env python3
"""
Smoke-test every occupancy map under maps/ through the real nav2_map_server.

map_assets_test.py already validates map YAML/PGM structure and alignment
against each world without touching ROS. This probe instead drives the
actual nav2_map_server lifecycle nodes the launch file starts (configure,
then activate) against every map -- the same loader the real stack uses --
so a map that passes the offline parser above but that nav2_map_server
itself rejects still gets caught. No Gazebo involved: these are plain
lifecycle nodes.

The probe drives the transitions itself instead of delegating to
nav2_lifecycle_manager's autostart: with N map_server processes starting at
once, a change_state call can finish on the server a beat after the client
gave up waiting for the response (observed directly while writing this
test), and nav2_lifecycle_manager treats that as a hard failure with no
retry. Rechecking get_state before giving up on a slow response, and
retrying the call itself a couple of times, absorbs that without masking an
actual configure/activate rejection (which comes back as `success: False`
promptly, not a timeout).
"""

import sys
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


SERVICE_WAIT_TIMEOUT_S = 10.0
CALL_TIMEOUT_S = 3.0
MAX_ATTEMPTS = 3

TRANSITIONS = (
    (Transition.TRANSITION_CONFIGURE, State.PRIMARY_STATE_INACTIVE, "configure"),
    (Transition.TRANSITION_ACTIVATE, State.PRIMARY_STATE_ACTIVE, "activate"),
)


class NodeClients:
    """The two lifecycle service clients a probed node needs."""

    def __init__(self, node, name):
        self.name = name
        self.change_state = node.create_client(ChangeState, f"/{name}/change_state")
        self.get_state = node.create_client(GetState, f"/{name}/get_state")

    def ready(self, timeout_sec):
        return (
            self.change_state.wait_for_service(timeout_sec=timeout_sec)
            and self.get_state.wait_for_service(timeout_sec=timeout_sec)
        )


class MapServerSmokeProbe(Node):
    """Drive each map_server node through configure -> activate."""

    def __init__(self):
        super().__init__("map_server_smoke_probe")
        self.declare_parameter("node_names", [""])
        self.node_names = [
            name for name in self.get_parameter("node_names").value if name]
        self.node_clients = {name: NodeClients(self, name) for name in self.node_names}

    def get_state(self, clients):
        """Return the current primary state id, or None if unreachable."""
        future = clients.get_state.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=CALL_TIMEOUT_S)
        if not future.done() or future.exception() is not None:
            return None
        return future.result().current_state.id

    def ensure_transition(self, clients, transition_id, target_state_id, label):
        """
        Drive one lifecycle transition, tolerant of a late response.

        Rechecks state before each attempt so a transition that actually
        succeeded server-side -- just too slowly for the previous attempt's
        call to see the response -- is recognized instead of retried (a
        second configure/activate call on an already-transitioned node is
        itself an invalid transition). A prompt `success: False` response is
        a real rejection and returns immediately, no retry.
        """
        for _ in range(MAX_ATTEMPTS):
            if self.get_state(clients) == target_state_id:
                return None
            future = clients.change_state.call_async(
                ChangeState.Request(transition=Transition(id=transition_id)))
            rclpy.spin_until_future_complete(self, future, timeout_sec=CALL_TIMEOUT_S)
            if future.done() and future.exception() is None:
                if future.result().success:
                    return None
                return f"transition '{label}' was rejected"
            # No response in time: recheck state above before the next
            # attempt, rather than assuming the call itself failed.
        return (
            f"transition '{label}' never confirmed after {MAX_ATTEMPTS} "
            "attempts (last state id "
            f"{self.get_state(clients)})")

    def bring_up(self, name):
        """Configure then activate one node. Returns an error string or None."""
        clients = self.node_clients[name]
        if not clients.ready(SERVICE_WAIT_TIMEOUT_S):
            return "lifecycle services never became available"
        for transition_id, target_state_id, label in TRANSITIONS:
            error = self.ensure_transition(clients, transition_id, target_state_id, label)
            if error:
                return error
        return None


def main():
    rclpy.init()
    node = MapServerSmokeProbe()
    try:
        if not node.node_names:
            node.get_logger().error(
                "no map_server node_names were passed to the probe")
            return 1

        node.get_logger().info(
            f"Bringing up {len(node.node_names)} map_server node(s)")
        start = time.monotonic()
        failures = {}
        for name in node.node_names:
            error = node.bring_up(name)
            if error:
                failures[name] = error
            node.get_logger().info(f"{name}: {'FAILED' if error else 'ok'}")

        if failures:
            node.get_logger().error(
                f"map_server smoke contract FAILED for {len(failures)}/"
                f"{len(node.node_names)} map(s) in {time.monotonic() - start:.1f}s:")
            for name, error in failures.items():
                node.get_logger().error(f"  {name}: {error}")
            return 1

        node.get_logger().info(
            f"map_server smoke contract PASSED for all {len(node.node_names)} "
            f"map(s) in {time.monotonic() - start:.1f}s")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
