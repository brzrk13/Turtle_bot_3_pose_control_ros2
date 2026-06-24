"""CLI client for /move_to_pose:  ros2 run pose_controller move_client X Y YAW_DEG"""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node

from pose_controller_interfaces.srv import MoveToPose


class MoveClient(Node):
    def __init__(self) -> None:
        super().__init__("move_to_pose_client")
        self._client = self.create_client(MoveToPose, "move_to_pose")

    def send(self, x: float, y: float, yaw_deg: float):
        if not self._client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("move_to_pose service not available.")
            return None
        req = MoveToPose.Request()
        req.x = x
        req.y = y
        req.yaw = yaw_deg
        self.get_logger().info(
            f"Requesting pose: x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f} deg ..."
        )
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None) -> None:
    if len(sys.argv) != 4:
        print("Usage: ros2 run pose_controller move_client X Y YAW_DEG")
        sys.exit(1)
    x, y, yaw = (float(v) for v in sys.argv[1:4])

    rclpy.init(args=args)
    node = MoveClient()
    result = node.send(x, y, yaw)
    if result is not None:
        print("-" * 56)
        print(f"success           : {result.success}")
        print(f"message           : {result.message}")
        print(f"final pose        : x={result.final_x:.3f} m, "
              f"y={result.final_y:.3f} m, yaw={result.final_yaw:.2f} deg")
        print(f"position error    : {result.position_error * 100:.1f} cm")
        print(f"orientation error : {result.orientation_error:.2f} deg")
        print("-" * 56)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
