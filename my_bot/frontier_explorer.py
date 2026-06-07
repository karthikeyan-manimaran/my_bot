#!/usr/bin/env python3
"""
Custom wavefront frontier explorer for ROS2 Humble + Nav2 + slam_toolbox.

How it works (and logs every step so you can see what it decides):
  1. Subscribe to /map (OccupancyGrid from slam_toolbox).
  2. Find frontier cells: FREE cells (value 0..threshold) that border UNKNOWN
     cells (value -1).
  3. Cluster adjacent frontier cells; each cluster's centroid is a candidate.
  4. Score candidates and send the best one to Nav2's NavigateToPose action.
  5. When the robot arrives (or the goal aborts), re-evaluate the latest map
     and pick the next frontier. Stop when no frontiers remain.

This reuses your existing stack -- mine world, SLAM, Nav2 (with your
nav2_explore.yaml params), and the kickstart spin. It replaces explore_lite only.

Run it (after world + SLAM + Nav2 + kickstart are up):
    ros2 run my_bot frontier_explorer
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
import tf2_ros


# Tunables -------------------------------------------------------------------
MIN_FRONTIER_CELLS = 8       # ignore clusters smaller than this (noise)
ROBOT_FRAME = "base_footprint"
MAP_FRAME = "map"
FREE_THRESHOLD = 50          # occupancy < this (and >=0) counts as free
REPLAN_AFTER_GOAL = True     # pick a new frontier each time one completes
# Scoring: prefer big frontiers, lightly penalise distance.
SIZE_WEIGHT = 1.0
DISTANCE_WEIGHT = 0.5
# ----------------------------------------------------------------------------


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        # /map is latched (transient local) by slam_toolbox
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.map = None
        self.create_subscription(OccupancyGrid, "map", self.map_cb, map_qos)

        self.marker_pub = self.create_publisher(MarkerArray, "frontier_markers", 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.busy = False          # True while a goal is in progress
        self.blacklist = []        # frontiers we failed to reach (world coords)

        # Main loop: try to pick a frontier every 2 s when idle
        self.create_timer(2.0, self.tick)
        self.get_logger().info("Frontier explorer started. Waiting for map + Nav2...")

    # -- callbacks -----------------------------------------------------------
    def map_cb(self, msg):
        self.map = msg

    def robot_cell(self):
        """Return the robot's (col,row) in the map grid, or None."""
        try:
            t = self.tf_buffer.lookup_transform(
                MAP_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except Exception:
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        info = self.map.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        return (col, row, x, y)

    # -- main logic ----------------------------------------------------------
    def tick(self):
        if self.map is None:
            self.get_logger().info("No map yet...")
            return
        if self.busy:
            return
        if not self.nav.server_is_ready():
            self.nav.wait_for_server(timeout_sec=2.0)
            if not self.nav.server_is_ready():
                self.get_logger().warn("Nav2 navigate_to_pose not ready yet...")
                return

        rc = self.robot_cell()
        if rc is None:
            self.get_logger().warn("No TF for robot pose yet...")
            return
        robot_col, robot_row, robot_x, robot_y = rc

        frontiers = self.find_frontiers()
        if not frontiers:
            self.get_logger().info(">>> No frontiers left. Exploration COMPLETE.")
            self.publish_markers([])
            return

        info = self.map.info

        # For each cluster pick a REAL frontier cell: the one farthest from the
        # robot. The centroid of a cluster can land on the robot itself (a ring
        # of frontier around the explored bubble), so we never use it as a goal.
        candidates = []  # (wx, wy, size, dist_to_robot)
        all_marker_cells = []
        for cells in frontiers:
            size = len(cells)
            best_cell = None
            best_d = -1.0
            for (c, r) in cells:
                wx = info.origin.position.x + (c + 0.5) * info.resolution
                wy = info.origin.position.y + (r + 0.5) * info.resolution
                all_marker_cells.append((c, r))
                d = math.hypot(wx - robot_x, wy - robot_y)
                if d > best_d:
                    best_d = d
                    best_cell = (wx, wy)
            if best_cell is not None:
                candidates.append((best_cell[0], best_cell[1], size, best_d))

        self.publish_markers(all_marker_cells)

        # Choose the best candidate: big cluster, not blacklisted, and far
        # enough away that it actually moves the robot (>0.5 m).
        best = None
        best_score = -1e9
        for (wx, wy, size, dist) in candidates:
            if self.is_blacklisted(wx, wy):
                continue
            if dist < 0.5:
                continue  # too close to be a real exploration target
            score = SIZE_WEIGHT * size - DISTANCE_WEIGHT * dist
            if score > best_score:
                best_score = score
                best = (wx, wy, size, dist)

        if best is None:
            self.get_logger().info(
                ">>> No reachable frontier > 0.5 m away. Clearing blacklist.")
            self.blacklist.clear()
            return

        wx, wy, size, dist = best
        self.get_logger().info(
            f">>> {len(candidates)} frontier clusters. "
            f"Going to edge cell ({wx:.2f}, {wy:.2f}) | size={size}, dist={dist:.2f} m")
        self.send_goal(wx, wy)

    def find_frontiers(self):
        """Return list of (col,row,size) frontier-cluster centroids."""
        info = self.map.info
        w, h = info.width, info.height
        data = self.map.data

        def val(c, r):
            return data[r * w + c]

        # Identify frontier cells
        is_frontier = bytearray(w * h)
        for r in range(1, h - 1):
            base = r * w
            for c in range(1, w - 1):
                if val(c, r) < 0 or val(c, r) >= FREE_THRESHOLD:
                    continue  # must be free
                # free cell bordering at least one unknown cell?
                if (val(c + 1, r) == -1 or val(c - 1, r) == -1 or
                        val(c, r + 1) == -1 or val(c, r - 1) == -1):
                    is_frontier[base + c] = 1

        # Cluster frontier cells (BFS, 8-connected) into centroids
        visited = bytearray(w * h)
        clusters = []
        for r in range(h):
            for c in range(w):
                idx = r * w + c
                if not is_frontier[idx] or visited[idx]:
                    continue
                q = deque([(c, r)])
                visited[idx] = 1
                cells = []
                while q:
                    cc, cr = q.popleft()
                    cells.append((cc, cr))
                    for dc in (-1, 0, 1):
                        for dr in (-1, 0, 1):
                            nc, nr = cc + dc, cr + dr
                            if 0 <= nc < w and 0 <= nr < h:
                                ni = nr * w + nc
                                if is_frontier[ni] and not visited[ni]:
                                    visited[ni] = 1
                                    q.append((nc, nr))
                if len(cells) >= MIN_FRONTIER_CELLS:
                    # Keep the whole cluster; tick() will choose a real edge cell
                    # from it rather than the (often useless) centroid.
                    clusters.append(cells)
        return clusters

    # -- Nav2 goal handling --------------------------------------------------
    def send_goal(self, wx, wy):
        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = MAP_FRAME
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = wx
        p.pose.position.y = wy
        p.pose.orientation.w = 1.0
        goal.pose = p

        self.busy = True
        self._pending = (wx, wy)
        future = self.nav.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Goal REJECTED by Nav2.")
            self.blacklist.append(self._pending)
            self.busy = False
            return
        handle.get_result_async().add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        status = future.result().status
        wx, wy = self._pending
        # status 4 == SUCCEEDED
        if status == 4:
            self.get_logger().info(f"Reached ({wx:.2f}, {wy:.2f}). Re-evaluating map.")
        else:
            self.get_logger().warn(
                f"Goal to ({wx:.2f}, {wy:.2f}) ended with status {status}. Blacklisting.")
            self.blacklist.append((wx, wy))
        self.busy = False  # tick() will pick the next frontier

    def is_blacklisted(self, wx, wy):
        for (bx, by) in self.blacklist:
            if math.hypot(wx - bx, wy - by) < 0.5:  # within 0.5 m
                return True
        return False

    # -- visualization -------------------------------------------------------
    def publish_markers(self, frontiers):
        arr = MarkerArray()
        info = self.map.info if self.map else None
        m = Marker()
        m.header.frame_id = MAP_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "frontiers"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.r = 1.0
        m.color.g = 0.4
        m.color.a = 1.0
        if info:
            for (c, r) in frontiers:
                pt = Point()
                pt.x = info.origin.position.x + (c + 0.5) * info.resolution
                pt.y = info.origin.position.y + (r + 0.5) * info.resolution
                m.points.append(pt)
        arr.markers.append(m)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
