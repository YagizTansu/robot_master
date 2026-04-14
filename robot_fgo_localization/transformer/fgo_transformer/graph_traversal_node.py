#!/usr/bin/env python3
"""
Graph Traversal Node
====================
Drives the robot through every edge in robot_map_graph.json at least once,
collecting spatially diverse data for Transformer training.

Algorithm
---------
1. Load graph from JSON; build adjacency list.
2. Compute an edge-covering traversal with a greedy DFS:
   - From current node prefer unvisited edges.
   - When all adjacent edges are visited, Dijkstra to nearest node that
     still has unvisited edges ("deadhead" repositioning step).
3. Execute each step by sending NavigateToPose action goals to Nav2.
4. Retry failed goals up to ``max_retries`` times, then skip.

Published Topics
----------------
/data_collection/status  (std_msgs/String)
    JSON: {"phase": "coverage|deadhead|COMPLETE", "step": int,
           "total": int, "from": node_id, "to": node_id}

Parameters
----------
graph_file        Path to robot_map_graph.json  (REQUIRED)
start_node        Node ID to begin from          (default: first node)
nav_timeout_sec   Max seconds to wait per goal   (default: 120)
max_retries       Retry count before skipping    (default: 3)
use_sim_time      Use /clock                     (default: true)
"""

import heapq
import json
import math
import os
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


# ─────────────────────────────────────────────────────────────────────────────
# Pure helpers
# ─────────────────────────────────────────────────────────────────────────────

def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _bearing(x1: float, y1: float, x2: float, y2: float) -> float:
    """Heading angle from (x1,y1) → (x2,y2)."""
    return math.atan2(y2 - y1, x2 - x1)


def _edge_key(a: str, b: str) -> str:
    """Canonical (order-independent) key for an undirected edge."""
    return f"{min(a, b)}::{max(a, b)}"


# ─────────────────────────────────────────────────────────────────────────────
# Graph helpers
# ─────────────────────────────────────────────────────────────────────────────

def _build_adjacency(nodes: Dict, edges: List) -> Dict[str, List[Tuple]]:
    """
    Returns adj[node_id] = [(neighbor_id, edge_key, edge_index), ...]
    Bidirectional edges appear in both directions.
    """
    adj: Dict[str, List[Tuple]] = {nid: [] for nid in nodes}
    for idx, edge in enumerate(edges):
        key = _edge_key(edge["from"], edge["to"])
        adj[edge["from"]].append((edge["to"],   key, idx))
        if edge.get("bidirectional", True):
            adj[edge["to"]].append((edge["from"], key, idx))
    return adj


def _dijkstra(adj: Dict, edges: List, nodes: Dict,
              source: str, target: str) -> List[str]:
    """Shortest path by edge cost from source to target. Returns node-ID list."""
    dist: Dict[str, float] = {nid: math.inf for nid in nodes}
    prev: Dict[str, Optional[str]] = {}
    dist[source] = 0.0
    heap = [(0.0, source)]

    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:
            continue
        if u == target:
            break
        for v, _key, idx in adj[u]:
            nd = d + edges[idx]["cost"]
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd, v))

    # Reconstruct
    path: List[str] = []
    cur: Optional[str] = target
    while cur in prev:
        path.append(cur)
        cur = prev[cur]
    path.append(source)
    path.reverse()
    return path


def _plan_edge_cover(
    nodes: Dict,
    edges: List,
    adj:   Dict,
    start: str,
) -> List[Tuple[str, str, str]]:
    """
    Returns a list of (from_id, to_id, phase) tuples that cover every edge.

    phase = "coverage"  → traversing a previously unvisited edge
    phase = "deadhead"  → repositioning move to reach an unvisited edge

    Strategy
    --------
    Greedy DFS with "most-unvisited-neighbours" tie-breaking (avoids premature
    dead-ends) and Dijkstra-based repositioning when stuck.
    """
    all_edge_keys: Set[str] = {_edge_key(e["from"], e["to"]) for e in edges}
    visited:       Set[str] = set()
    plan:          List[Tuple[str, str, str]] = []
    current:       str = start

    while visited != all_edge_keys:
        # Adjacent edges that have not been traversed yet
        candidates = [
            (nbr, key, idx)
            for nbr, key, idx in adj[current]
            if key not in visited
        ]

        if candidates:
            # Tie-break: prefer the neighbour that still has the most
            # unvisited adjacent edges (Fleury-style, avoids cutting off
            # sub-graphs prematurely).
            best = max(
                candidates,
                key=lambda t: sum(
                    1 for _, k, _ in adj[t[0]] if k not in visited
                ),
            )
            nbr, key, _ = best
            plan.append((current, nbr, "coverage"))
            visited.add(key)
            current = nbr

        else:
            # Dead-end: reposition to the closest node that has unvisited edges.
            targets_with_work = [
                nid for nid in nodes
                if any(k not in visited for _, k, _ in adj[nid])
            ]
            if not targets_with_work:
                break  # all edges covered

            # Euclidean pre-filter to pick the nearest target, then Dijkstra.
            cur_node = nodes[current]
            nearest = min(
                targets_with_work,
                key=lambda nid: math.hypot(
                    nodes[nid]["x"] - cur_node["x"],
                    nodes[nid]["y"] - cur_node["y"],
                ),
            )
            path = _dijkstra(adj, edges, nodes, current, nearest)
            for i in range(len(path) - 1):
                u, v = path[i], path[i + 1]
                k = _edge_key(u, v)
                visited.add(k)          # mark deadhead edges covered too
                plan.append((u, v, "deadhead"))
            current = nearest

    return plan


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 Node
# ─────────────────────────────────────────────────────────────────────────────

class GraphTraversalNode(Node):

    def __init__(self) -> None:
        super().__init__("graph_traversal")

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter("graph_file",       "")
        self.declare_parameter("start_node",       "")
        self.declare_parameter("nav_timeout_sec",  120.0)
        self.declare_parameter("max_retries",      3)

        graph_file  = str(self.get_parameter("graph_file").value).strip()
        start_node  = str(self.get_parameter("start_node").value).strip()
        self._timeout  = float(self.get_parameter("nav_timeout_sec").value)
        self._max_retries = int(self.get_parameter("max_retries").value)

        if not graph_file or not os.path.isfile(graph_file):
            raise RuntimeError(
                f"graph_file '{graph_file}' does not exist. "
                "Set the graph_file parameter to a valid path."
            )

        # ── Load graph ──────────────────────────────────────────────────────
        with open(graph_file, "r") as f:
            data = json.load(f)

        g = data["graph"]
        self._nodes: Dict = {n["id"]: n for n in g["nodes"]}
        self._edges: List = g["edges"]
        self._adj   = _build_adjacency(self._nodes, self._edges)

        start = (
            start_node
            if (start_node and start_node in self._nodes)
            else next(iter(self._nodes))
        )

        # ── Plan ────────────────────────────────────────────────────────────
        self._plan   = _plan_edge_cover(self._nodes, self._edges, self._adj, start)
        self._step   = 0
        self._retries = 0

        n_coverage = sum(1 for _, _, p in self._plan if p == "coverage")
        n_deadhead = sum(1 for _, _, p in self._plan if p == "deadhead")

        self.get_logger().info(
            f"\n{'='*60}\n"
            f"  GraphTraversalNode ready\n"
            f"{'='*60}\n"
            f"  Graph     : {len(self._nodes)} nodes, {len(self._edges)} edges\n"
            f"  Plan      : {len(self._plan)} steps "
            f"({n_coverage} coverage + {n_deadhead} deadhead)\n"
            f"  Start     : {start}\n"
            f"{'='*60}"
        )

        # ── Publishers ──────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(
            String, "/data_collection/status", 10
        )

        # ── Nav2 action client ───────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── Internal state for timeout ───────────────────────────────────────
        self._current_handle = None    # active goal handle for cancellation
        self._timeout_timer  = None    # one-shot timer per goal

        # ── Wait for Nav2, then kick off traversal ───────────────────────────
        self._started       = False
        self._wait_timer    = self.create_timer(2.0, self._wait_and_start)

    # ── Startup ───────────────────────────────────────────────────────────────

    def _wait_and_start(self) -> None:
        """Called every 2 s until Nav2 action server is available."""
        if self._started:
            self._wait_timer.cancel()
            return
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for Nav2 navigate_to_pose action server…"
            )
            return
        self._started = True
        self._wait_timer.cancel()
        self.get_logger().info("Nav2 ready — starting graph traversal")
        self._send_next_goal()

    # ── Navigation ───────────────────────────────────────────────────────────

    def _send_next_goal(self) -> None:
        if self._step >= len(self._plan):
            self.get_logger().info(
                f"✓ Graph traversal COMPLETE — {len(self._plan)} steps, "
                f"{len(self._edges)} edges covered."
            )
            self._publish_status("COMPLETE", "", "")
            return

        from_id, to_id, phase = self._plan[self._step]
        to_n = self._nodes[to_id]

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = to_n["x"]
        goal.pose.pose.position.y = to_n["y"]
        goal.pose.pose.position.z = 0.0
        # Identity quaternion: do NOT enforce arrival yaw.
        # Nav2 would try to rotate in-place to match a forced bearing and
        # often ABORT when it can't achieve the exact angle in tight spaces.
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f"[{self._step + 1}/{len(self._plan)}] {phase.upper()}: "
            f"{from_id} → {to_id}  "
            f"({to_n['x']:.2f}, {to_n['y']:.2f})"
        )
        self._publish_status(phase, from_id, to_id)

        # Start a hard-timeout timer so we never hang indefinitely.
        self._cancel_timeout_timer()
        self._timeout_timer = self.create_timer(
            self._timeout, self._on_goal_timeout
        )

        future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        future.add_done_callback(self._on_goal_accepted)

    def _feedback_callback(self, feedback_msg) -> None:
        """Optional: log remaining distance from Nav2 feedback."""
        pass  # Can add progress logging here if needed

    def _on_goal_accepted(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(
                f"Step {self._step + 1}: goal rejected — skipping"
            )
            self._cancel_timeout_timer()
            self._advance()
            return
        self._current_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        self._cancel_timeout_timer()
        self._current_handle = None
        result = future.result()
        # action_msgs/msg/GoalStatus: 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if result.status == 4:
            self.get_logger().info(
                f"  ✓ Step {self._step + 1} reached — "
                f"{self._plan[self._step][1]}"
            )
            self._advance()
        else:
            self._retries += 1
            self.get_logger().warn(
                f"  ✗ Step {self._step + 1} failed "
                f"(status={result.status}), "
                f"retry {self._retries}/{self._max_retries}"
            )
            if self._retries >= self._max_retries:
                self.get_logger().error(
                    f"Max retries reached for step {self._step + 1} — skipping"
                )
                self._advance()
            else:
                self._send_next_goal()  # retry same step

    def _on_goal_timeout(self) -> None:
        """Hard timeout — cancel the active Nav2 goal and skip the step."""
        self._cancel_timeout_timer()
        self.get_logger().error(
            f"  ✗ Step {self._step + 1} TIMED OUT after {self._timeout:.0f} s — "
            f"cancelling goal and skipping"
        )
        if self._current_handle is not None:
            self._current_handle.cancel_goal_async()
            self._current_handle = None
        self._retries = self._max_retries  # force skip on next result callback
        self._advance()

    def _cancel_timeout_timer(self) -> None:
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self.destroy_timer(self._timeout_timer)
            self._timeout_timer = None

    def _advance(self) -> None:
        self._step    += 1
        self._retries  = 0
        self._send_next_goal()

    # ── Status publisher ──────────────────────────────────────────────────────

    def _publish_status(self, phase: str, from_id: str, to_id: str) -> None:
        msg = String()
        msg.data = json.dumps({
            "phase": phase,
            "step":  self._step,
            "total": len(self._plan),
            "from":  from_id,
            "to":    to_id,
        })
        self._status_pub.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraphTraversalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
