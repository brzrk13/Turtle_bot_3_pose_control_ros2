# TurtleBot3 Pose Controller

A ROS 2 pose controller for the TurtleBot3. Call a service with a target
`(x, y, yaw)` and the robot drives to it under closed-loop control, correcting
against odometry until it settles within 5 cm and 5° of the goal.

The motion is continuous: the robot translates and rotates at the same time and
traces a smooth curve to the target, instead of turning in place, driving
straight, then turning again. No Nav2 or other navigation stack is involved —
the control law is implemented directly.

## How it works

The node runs a control loop at 20 Hz. On each tick it:

1. reads the current pose from `/odom`,
2. expresses the gap to the goal in polar form — the distance `ρ`, the heading
   error `α` (how far the robot is from pointing at the goal), and the
   orientation error `β` (the turn still needed to finish at the commanded yaw),
3. converts those into a velocity command and publishes it on `/cmd_vel`.

The control law is the polar pose controller described in Siegwart's
*Introduction to Autonomous Mobile Robots*:

```
v = k_rho   * ρ * cos(α)
ω = k_alpha * α + k_beta * β
```

Linear and angular velocity are commanded together every tick, which is what
produces the curved approach. The `cos(α)` factor eases off forward speed while
the robot is still pointing the wrong way, so it turns onto the line of sight
rather than driving away from it. The output is clamped to the TurtleBot3's
limits and rate-limited so the motion stays smooth. Once the robot is inside the
position tolerance it stops translating and settles the final heading with a
small proportional term, which avoids the angular jitter that `α` develops when
`ρ` approaches zero. The gains satisfy the usual stability conditions
(`k_rho > 0`, `k_beta < 0`, `k_alpha − k_rho > 0`).

The control approach and design decisions are covered in the slides:
[docs/PoseController_Presentation.pptx](docs/PoseController_Presentation.pptx).

## Layout

```
turtlebot3_pose_control/
├── ros2_ws/src/
│   ├── pose_controller_interfaces/    # MoveToPose service definition
│   └── pose_controller/
│       ├── pose_controller/
│       │   ├── pose_controller_node.py    # control node, control law, service
│       │   └── move_client.py             # CLI client for the service
│       ├── launch/
│       └── config/pose_controller.yaml
└── docs/                              # slides and demo
```

The service definition lives in its own package because custom interfaces have
to be built with `ament_cmake`, while the node is a plain `ament_python` package.
The whole controller — error computation, control law, service, and ROS plumbing
— is in `pose_controller_node.py`.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim (Harmonic), provided by `ros_gz`
- TurtleBot3 simulation packages

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-msgs \
  ros-jazzy-turtlebot3-simulations ros-jazzy-turtlebot3-gazebo
```

## Build

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

If you build packages selectively, build `pose_controller_interfaces` before
`pose_controller`.

## Running the simulation

Set the robot model and source the workspace in each terminal you use. Run
these from the `ros2_ws` directory (the source path is relative to it):

```bash
cd ros2_ws
export TURTLEBOT3_MODEL=burger
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Launch Gazebo and the controller together:

```bash
ros2 launch pose_controller tb3_pose_control.launch.py
```

This brings up the TurtleBot3 empty world with the robot at the origin
(x = 0, y = 0, yaw = 0, following the REP-103 frame convention) and starts the
controller. On Jazzy the TurtleBot3 Gazebo bridge subscribes to `/cmd_vel` as
`geometry_msgs/TwistStamped`, so the launch file sets `use_stamped_twist:=true`;
the controller can publish either `Twist` or `TwistStamped` depending on that
parameter.

If the simulation is already running, or you are driving real hardware, start
the controller on its own:

```bash
ros2 launch pose_controller pose_controller.launch.py
```

## Sending a goal

The service is `move_to_pose` (`pose_controller_interfaces/srv/MoveToPose`).
`x` and `y` are in metres and `yaw` is in degrees. The call blocks until the
robot settles and returns the pose it reached together with the remaining error.

A small client is included:

```bash
ros2 run pose_controller move_client 1.0  0.5   45
ros2 run pose_controller move_client -1.0 0.8   90
ros2 run pose_controller move_client 0.5  -0.5 -90
```

You can also call the service directly:

```bash
ros2 service call /move_to_pose pose_controller_interfaces/srv/MoveToPose \
  "{x: 1.0, y: 0.5, yaw: 45.0}"
```

A typical response:

```
success           : True
message           : Reached target pose within tolerance.
final pose        : x=0.980 m, y=0.480 m, yaw=44.98 deg
position error    : 2.9 cm
orientation error : 0.02 deg
```

## Interfaces

| Direction  | Name            | Type                                          |
|------------|-----------------|-----------------------------------------------|
| Subscribe  | `/odom`         | `nav_msgs/Odometry`                           |
| Publish    | `/cmd_vel`      | `geometry_msgs/Twist` or `TwistStamped`       |
| Service    | `/move_to_pose` | `pose_controller_interfaces/srv/MoveToPose`   |

## Configuration

Tuning is in
[config/pose_controller.yaml](ros2_ws/src/pose_controller/config/pose_controller.yaml).
The values that matter most:

| Parameter                                   | Default       | Meaning                              |
|---------------------------------------------|---------------|--------------------------------------|
| `k_rho`, `k_alpha`, `k_beta`                | 0.45, 1.2, −0.45 | control gains                     |
| `max_linear_velocity`                       | 0.20          | linear speed cap (m/s)               |
| `max_angular_velocity`                      | 1.6           | angular speed cap (rad/s)            |
| `linear_accel_limit`, `angular_accel_limit` | 0.5, 3.0      | slew-rate caps for smoothing         |
| `position_tolerance`                        | 0.04          | position goal tolerance (m)          |
| `orientation_tolerance_deg`                 | 4.0           | yaw goal tolerance (degrees)         |
| `use_stamped_twist`                         | false         | publish `TwistStamped` instead of `Twist` |

Point any launch file at a different file with
`params_file:=/path/to/your.yaml`.

## Results

Driving to the three example goals in sequence in Gazebo:

| Target (x, y, yaw) | Final pose                | Position error | Orientation error |
|--------------------|---------------------------|----------------|-------------------|
| (1.0, 0.5, 45°)    | (0.980, 0.480, 44.98°)    | 2.9 cm         | 0.02°             |
| (−1.0, 0.8, 90°)   | (−1.010, 0.773, 90.08°)   | 2.9 cm         | 0.08°             |
| (0.5, −0.5, −90°)  | (0.500, −0.469, −88.83°)  | 3.1 cm         | 1.17°             |

Each goal is reached comfortably inside the 5 cm / 5° requirement. A short
screen recording of these runs is in [docs/demo.mp4](docs/demo.mp4).

## Assumptions and limitations

The target is interpreted in the same `odom` frame the robot starts in, and the
controller trusts odometry. That is fine for the short moves here, but over long
distances it would drift without a map and localisation. There is no obstacle
avoidance — this is pose control rather than navigation. The robot is
differential drive, so it cannot move sideways, and the controller handles one
goal at a time. The law is proportional with a terminal heading settle and no
integral term, so a small steady-state bias is possible; in practice the settle
window absorbs it.

## License

Apache-2.0.
