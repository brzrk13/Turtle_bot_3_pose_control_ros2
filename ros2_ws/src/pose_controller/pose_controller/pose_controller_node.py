"""Drive a differential-drive robot to a commanded pose.

Exposes a blocking ``/move_to_pose`` service and a closed-loop controller that
reads ``/odom`` and publishes velocity on ``/cmd_vel``. A single polar control
law drives and steers at the same time, so the robot curves onto the goal.
Frames follow REP-103 (x forward, y left, yaw positive CCW about +z).
"""

import math
import threading
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from pose_controller_interfaces.srv import MoveToPose


def normalize_angle(angle):
    """Wrap to (-pi, pi] so heading errors take the short way round."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x, y, z, w):
    """Yaw about +z from a quaternion (the robot is planar)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _clamp(value, low, high):
    return max(low, min(high, value))


def _slew(current, target, max_step):
    """Limit change per tick so velocity commands ramp instead of jumping."""
    return current + _clamp(target - current, -max_step, max_step)


class State(Enum):
    IDLE = auto()
    DRIVING = auto()
    ORIENTING = auto()


class PoseControllerNode(Node):
    def __init__(self):
        super().__init__("pose_controller")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("use_stamped_twist", False)
        self.declare_parameter("control_frequency", 20.0)
        self.declare_parameter("k_rho", 0.45)
        self.declare_parameter("k_alpha", 1.2)
        self.declare_parameter("k_beta", -0.45)
        self.declare_parameter("k_yaw", 1.5)
        self.declare_parameter("max_linear_velocity", 0.20)
        self.declare_parameter("max_angular_velocity", 1.6)
        self.declare_parameter("linear_accel_limit", 0.5)
        self.declare_parameter("angular_accel_limit", 3.0)
        self.declare_parameter("position_tolerance", 0.04)
        self.declare_parameter("orientation_tolerance_deg", 4.0)
        self.declare_parameter("settle_cycles", 5)
        self.declare_parameter("goal_timeout", 60.0)

        # Stability requires k_rho > 0, k_beta < 0, k_alpha - k_rho > 0.
        self._k_rho = self._p("k_rho")
        self._k_alpha = self._p("k_alpha")
        self._k_beta = self._p("k_beta")
        self._k_yaw = self._p("k_yaw")
        self._max_v = self._p("max_linear_velocity")
        self._max_w = self._p("max_angular_velocity")
        self._pos_tol = self._p("position_tolerance")
        self._ori_tol = math.radians(self._p("orientation_tolerance_deg"))
        self._settle_cycles = int(self.get_parameter("settle_cycles").value)
        self._goal_timeout = self._p("goal_timeout")
        self._dt = 1.0 / self._p("control_frequency")
        self._lin_step = self._p("linear_accel_limit") * self._dt
        self._ang_step = self._p("angular_accel_limit") * self._dt
        self._use_stamped = bool(self.get_parameter("use_stamped_twist").value)

        # Shared state, guarded by _lock.
        self._lock = threading.Lock()
        self._state = State.IDLE
        self._have_odom = False
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._goal = None
        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._in_tol_count = 0
        self._done_event = threading.Event()
        self._result = None
        self._elapsed = 0.0

        # Separate callback groups so the blocking service and the control timer
        # run concurrently under the MultiThreadedExecutor.
        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._srv_group = MutuallyExclusiveCallbackGroup()

        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._odom_sub = self.create_subscription(
            Odometry, self._p_str("odom_topic"), self._odom_cb, odom_qos,
            callback_group=self._timer_group,
        )
        twist_type = TwistStamped if self._use_stamped else Twist
        self._cmd_pub = self.create_publisher(
            twist_type, self._p_str("cmd_vel_topic"), 10,
        )
        self._srv = self.create_service(
            MoveToPose, "move_to_pose", self._on_move_to_pose,
            callback_group=self._srv_group,
        )
        self._timer = self.create_timer(
            self._dt, self._control_step, callback_group=self._timer_group,
        )

        self.get_logger().info(
            f"pose_controller ready | odom='{self._p_str('odom_topic')}' "
            f"cmd_vel='{self._p_str('cmd_vel_topic')}' "
            f"({'TwistStamped' if self._use_stamped else 'Twist'}) "
            f"@ {self._p('control_frequency'):.0f} Hz"
        )

    def _p(self, name):
        return float(self.get_parameter(name).value)

    def _p_str(self, name):
        return str(self.get_parameter(name).value)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self._lock:
            self._x = p.x
            self._y = p.y
            self._yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            self._have_odom = True

    def _on_move_to_pose(self, request, response):
        """Blocking: returns once the robot has settled at the goal."""
        if not self._have_odom:
            response.success = False
            response.message = "No odometry received yet; is the simulation running?"
            return response

        with self._lock:
            if self._state != State.IDLE:
                response.success = False
                response.message = "Controller is busy with another goal."
                return response
            self._goal = (request.x, request.y, math.radians(request.yaw))
            self._cmd_v = 0.0
            self._cmd_w = 0.0
            self._in_tol_count = 0
            self._elapsed = 0.0
            self._result = None
            self._done_event.clear()
            self._state = State.DRIVING

        self.get_logger().info(
            f"New goal: x={request.x:.3f} m, y={request.y:.3f} m, yaw={request.yaw:.1f} deg"
        )

        finished = self._done_event.wait(timeout=self._goal_timeout + 5.0)
        if not finished:
            with self._lock:
                self._state = State.IDLE
            self._publish_cmd(0.0, 0.0)
            response.success = False
            response.message = "Service wait timed out."
            return response

        res = self._result
        response.success = res["success"]
        response.message = res["message"]
        response.final_x = res["x"]
        response.final_y = res["y"]
        response.final_yaw = math.degrees(res["yaw"])
        response.position_error = res["pos_err"]
        response.orientation_error = math.degrees(res["ori_err"])
        self.get_logger().info(f"Goal finished: {res['message']}")
        return response

    def _control_step(self):
        with self._lock:
            if self._state == State.IDLE or self._goal is None:
                return
            x, y, yaw = self._x, self._y, self._yaw
            gx, gy, gyaw = self._goal
            state = self._state
            self._elapsed += self._dt
            elapsed = self._elapsed

        dx, dy = gx - x, gy - y
        rho = math.hypot(dx, dy)                       # distance to goal
        angle_to_goal = math.atan2(dy, dx)
        alpha = normalize_angle(angle_to_goal - yaw)   # turn to face the goal
        beta = normalize_angle(gyaw - angle_to_goal)   # extra turn to final yaw
        yaw_error = normalize_angle(gyaw - yaw)

        if rho <= self._pos_tol and abs(yaw_error) <= self._ori_tol:
            with self._lock:
                self._in_tol_count += 1
                settled = self._in_tol_count >= self._settle_cycles
            if settled:
                self._finish(True, "Reached target pose within tolerance.",
                             x, y, yaw, rho, abs(yaw_error))
                return
        else:
            with self._lock:
                self._in_tol_count = 0

        if elapsed > self._goal_timeout:
            self._finish(False, "Timed out before reaching the target pose.",
                         x, y, yaw, rho, abs(yaw_error))
            return

        if rho > self._pos_tol and state != State.ORIENTING:
            # Drive and steer together; cos(alpha) eases off speed when mis-aimed.
            v_target = self._k_rho * rho * math.cos(alpha)
            w_target = self._k_alpha * alpha + self._k_beta * beta
            with self._lock:
                self._state = State.DRIVING
        else:
            # On the goal point: hold position and settle the final heading.
            v_target = 0.0
            w_target = self._k_yaw * yaw_error
            with self._lock:
                self._state = State.ORIENTING

        v_target = _clamp(v_target, -self._max_v, self._max_v)
        w_target = _clamp(w_target, -self._max_w, self._max_w)
        v_cmd = _slew(self._cmd_v, v_target, self._lin_step)
        w_cmd = _slew(self._cmd_w, w_target, self._ang_step)
        self._cmd_v, self._cmd_w = v_cmd, w_cmd
        self._publish_cmd(v_cmd, w_cmd)

    def _finish(self, success, message, x, y, yaw, pos_err, ori_err):
        self._publish_cmd(0.0, 0.0)
        with self._lock:
            self._state = State.IDLE
            self._goal = None
            self._cmd_v = 0.0
            self._cmd_w = 0.0
            self._result = {
                "success": success, "message": message,
                "x": x, "y": y, "yaw": yaw,
                "pos_err": pos_err, "ori_err": ori_err,
            }
            self._done_event.set()

    def _publish_cmd(self, v, w):
        if self._use_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = v
            msg.twist.angular.z = w
        else:
            msg = Twist()
            msg.linear.x = v
            msg.angular.z = w
        self._cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cmd(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
