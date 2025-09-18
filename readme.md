# Holonomic Vehicle Odometry Package

## Overview

`mr_odom_holo_box` is a ROS 2 package developed to simulate and manage odometry for a mobile robot. It provides two main nodes and one auxiliar:

- **Odometry Computation Node (`mr_odom_node_p1`)**: Computes and publishes robot odometry based on internal state and transformations.
- **Teleoperation Node (`mr_odom_node_p1_teleop`)**: Extends the odometry node to include real-time updates via velocity command inputs (`/cmd_vel`), useful for joystick or keyboard teleoperation.
- **Joystick Bridge Node (`joy_bridge_node.py`)**: This node make the bridge with Windows for Joystick Applications. The GamePad is connected to Windows, the Server Stream the data and the Bridge reconstruct the message and create the `/Joy` topic. 

This package is useful for educational and experimental purposes in mobile robotics.

## Features

- Publishes odometry information (`nav_msgs/Odometry`)
- Broadcasts TF transformations for robot localization
- Subscribes to velocity commands (`geometry_msgs/Twist`) for teleop
- Includes ROS 2 launch files and configuration for joystick control

## Dependencies

- ROS 2 (tested with Foxy/Humble)
- `geometry_msgs`
- `nav_msgs`
- `tf2_ros`
- `numpy`

## Package Structure

```bash
mr_odom_holo_box/
├── launch/
│   └── joystick.launch.py
├── joy_pc_server/
│   └── joystick_server.py
├── config/
│   ├── joystick.yaml
│   └── twist_mux.yaml
├── mr_odom_pkg/
│   ├── __init__.py
│   ├── mr_odom_node_p1.py
│   └── mr_odom_node_p1_teleop.py
│   └── joy_bridge_node.py
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.py
└── setup.cfg

```

## Workspace and Package Creation

```bash
mkdir -p ~/<ros2_ws_YYYY>/src
echo ''source ~/<ros2_ws_YYYY>/install/setup.bash'' >> ~/.bashrc
source ~/.bashrc
```
**Note**. Where `<ros2_ws_YYYY>` has to be change by the Year and the Semester. **Ex**. If the Year is 2025 and it is the 2nd Semester. Therefore, the Worksapce is call: `ros2_ws_2502`

```bash
# Comando general
~/<ros2_ws_YYYY>/src $ ros2 pkg create <ros_package_name> --build-type ament_python  --dependencies <package_dependencies>
```

## Building

From the root of the workspace:

```bash
colcon build --packages-select mr_odom_holo_box
source install/setup.bash
```

## Usage

To run the odometry node:

```bash
ros2 run mr_odom_holo_box mr_odom_node_p1
```

To run the teleoperation odometry node:


```bash
ros2 run mr_odom_holo_box mr_odom_node_p1_teleop
```
- To run the teleoperation keyboard:
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard 
    ```
Or
- To launch the joystick teleoperation node (on Ubuntu machine):

    ```bash
    ros2 launch mr_odom_holo_box joystick.launch.py
    ```

    Ensure your joystick is connected and configured as per `config/joystick.yaml`.

- To launch the joystick teleoperation node (on WSL machine):

    ```bash
    ros2 launch mr_odom_holo_box joystick_wsl.launch.py
    ```
- On Windows
    - Connect the GamePad to the PC.
    - Copy the folder `joy_pc_server` to Windows.
    - Run the script `joy_pc_server.py`

    **Note**. This script requiere a full installation of Python3.X and PyGame.  


## Other programs recommended to run 

- RViz2 with TF pluging
- RQT with Plot topic data

## License

Apache 2.0

## Detail Description `mr_odom_node_p1` 

### Libraries

These are the required libraries:

- `rclpy`: ROS 2 Python client library.
- `numpy`: For numerical operations.
- `tf2_ros`: For broadcasting transforms between frames.
- `geometry_msgs` and `nav_msgs`: Standard ROS 2 message types for transformations and odometry.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
```
### Jacobian Matrix Function

This function returns the Jacobian matrix used to transform velocities from the robot’s body frame to the world frame, depending on its orientation $\varphi$.

```python
def jacobin_matrix(phi):
    J = np.array([[np.cos(phi), -np.sin(phi), 0],
                  [np.sin(phi),  np.cos(phi), 0],
                  [0,            0,           1]])
    return J
```

### Main Structure

```python
class ODOM_Node_P1(Node):
    def __init__(self):
        super().__init__("odom_node_p1")

    def timer_callback(self):
        pass

    def odometry_publisher(self):
        pass

    def odom_tf_broadcaster(self):
        pass

def main():
    rclpy.init()
    node = ODOM_Node_P1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
### Methods and Content

**Initial Conditions**

This defines:

- $\eta$: Robot pose $[x, y, \varphi]$
- $\xi$: Velocities in body frame

```python
self.eta = np.array([[0.0], [0.0], [0.0]])  # [x, y, phi]
self.u = 0.1  # forward velocity
self.v = 0.0  # lateral velocity (usually 0 for differential drive)
self.r = 0.1  # angular velocity (yaw rate)
self.xi = np.array([[self.u], [self.v], [self.r]])  # velocity vector
```

**Publishers and TF Broadcaster**

- Publishes odometry to the `odom` topic.
- Broadcasts the transform between `world` and `odom`.

```python
# publishers and subscribers
qos = QoSProfile(depth=10)
self.odom_pub = self.create_publisher(Odometry, "odom", qos_profile=qos)
self.tf_broadcaster = TransformBroadcaster(self)
self.t = TransformStamped()
self.t.header.frame_id = 'world'
self.t.child_frame_id = 'odom'
```

**Timer and Update Loop**

Sets a periodic timer with interval `dt = 0.1` seconds to update robot pose.

```python
self.dt = 0.1
self.timer = self.create_timer(self.dt, self.timer_callback)
```

**Main Update Logic**

Each timer tick:
- Transforms body-frame velocities to world frame.
- Integrates pose using Euler method.
- Publishes updated odometry and TF transform.

```python
def timer_callback(self):
    psi = self.eta[2,0]
    J = jacobin_matrix(psi) # 
    eta_dot = J @ self.xi  # kinematic model
    self.eta += eta_dot * self.dt  # Euler integration

    self.odometry_publisher() # publish odometry message
    self.odom_tf_broadcaster() # publish tf message
    self.get_logger().info(f"x: {self.eta[0,0]:.2f}, y: {self.eta[1,0]:.2f}, phi: {self.eta[2,0]:.2f}")
```

**Odometry Message Publisher**

Publishes a `nav_msgs/Odometry` message with current position, orientation (as quaternion), and velocities.

```python
def odometry_publisher(self):

    now = self.get_clock().now()
    odom_msg = Odometry()
    odom_msg.header.stamp = now.to_msg()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

    # Position and orientation
    odom_msg.pose.pose.position.x = self.eta[0,0]
    odom_msg.pose.pose.position.y = self.eta[1,0]
    odom_msg.pose.pose.orientation.z = np.sin(self.eta[2,0]/2)
    odom_msg.pose.pose.orientation.w = np.cos(self.eta[2,0]/2)

    # Velocities
    odom_msg.twist.twist.linear.x = self.u
    odom_msg.twist.twist.linear.y = self.v
    odom_msg.twist.twist.angular.z = self.r

    self.odom_pub.publish(odom_msg)
```

**TF Broadcaster**

Broadcasts the current robot transform from `world → odom`.

```python
def odom_tf_broadcaster(self):
    self.t.header.stamp = self.get_clock().now().to_msg()
    self.t.transform.translation.x = self.eta[0,0]
    self.t.transform.translation.y = self.eta[1,0]
    self.t.transform.rotation.z = np.sin(self.eta[2,0]/2)
    self.t.transform.rotation.w = np.cos(self.eta[2,0]/2)

    self.tf_broadcaster.sendTransform(self.t)
```


## Presentation

*Professor*: David Rozo-Osorio, I.M. M.Sc. email: david.rozo31@eia.edu.co

**EIA University**, Mechatronical Eng. - Industrial Robotics