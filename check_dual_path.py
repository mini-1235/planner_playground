#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import numpy as np


class DualPathFeasibilityChecker(Node):

    def __init__(self):
        super().__init__('dual_path_feasibility_checker')

        # ---------------- Parameters ----------------
        self.declare_parameter('min_radius', 2.5)
        self.min_radius = self.get_parameter('min_radius').value

        self.smoothed_topic = '/plan'
        self.raw_topic = '/unsmoothed_plan'

        self.marker_topic = '/path_violations'

        self.eps = 1e-6
        self.tolerance = 0.01  # 1 cm
        # --------------------------------------------

        self.sub_smoothed = self.create_subscription(
            Path, self.smoothed_topic,
            lambda msg: self.process_path(msg, smoothed=True),
            10
        )

        self.sub_raw = self.create_subscription(
            Path, self.raw_topic,
            lambda msg: self.process_path(msg, smoothed=False),
            10
        )

        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)

        self.get_logger().info(
            f"Checking:\n"
            f"  smoothed:   {self.smoothed_topic} (RED)\n"
            f"  unsmoothed: {self.raw_topic} (BLUE)\n"
            f"  min_radius: {self.min_radius:.2f} m"
        )

    # ======================================================
    # Core processing
    # ======================================================

    def process_path(self, msg: Path, smoothed: bool):
        poses = msg.poses
        if len(poses) < 3:
            return
        if smoothed:
           self.get_logger().info(f"smoothed path size: {len(poses)}")
        else:
           self.get_logger().info(f"path size: {len(poses)}")

        violations = []
        min_radius_found = float('inf')

        for i in range(1, len(poses) - 1):
            p1 = self.xy(poses[i - 1])
            p2 = self.xy(poses[i])
            p3 = self.xy(poses[i + 1])

            # ---------- CUSP SKIP ----------
            v1 = p2 - p1
            v2 = p3 - p2
            if np.dot(v1, v2) < 0.0:
                continue
            # -------------------------------

            radius = self.circumradius(p1, p2, p3)
            if radius is None:
                continue

            min_radius_found = min(min_radius_found, radius)

            if radius < self.min_radius - self.tolerance:
                violations.append(poses[i].pose.position)
                print("index", i)

        # ---------- Publish markers ----------
        self.publish_markers(
            violations,
            msg.header.frame_id,
            smoothed
        )

        tag = "SMOOTHED" if smoothed else "RAW"
        if violations:
            self.get_logger().error(
                f"[{tag}] {len(violations)} violations "
                f"(min={min_radius_found:.3f} m)"
            )
        else:
            self.get_logger().info(
                f"[{tag}] OK (min={min_radius_found:.3f} m)"
            )

    # ======================================================
    # Geometry
    # ======================================================

    def circumradius(self, p1, p2, p3):
        a = np.linalg.norm(p2 - p1)
        b = np.linalg.norm(p3 - p2)
        c = np.linalg.norm(p3 - p1)

        if a < self.eps or b < self.eps or c < self.eps:
            return None

        cross = np.cross(p2 - p1, p3 - p1)
        area = 0.5 * abs(cross)

        if area < self.eps:
            return float('inf')

        return (a * b * c) / (4.0 * area)

    # ======================================================
    # Visualization
    # ======================================================

    def publish_markers(self, points, frame_id, smoothed):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "smoothed" if smoothed else "raw"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST

        if not points:
            marker.action = Marker.DELETE
            self.marker_pub.publish(marker)
            return

        marker.action = Marker.ADD

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Color
        if smoothed:
            marker.color.r = 1.0  # RED
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0  # BLUE

        marker.color.a = 1.0

        for p in points:
            marker.points.append(p)

        self.marker_pub.publish(marker)

    # ======================================================

    @staticmethod
    def xy(pose_stamped):
        return np.array([
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y
        ])


# ==========================================================

def main(args=None):
    rclpy.init(args=args)
    node = DualPathFeasibilityChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()