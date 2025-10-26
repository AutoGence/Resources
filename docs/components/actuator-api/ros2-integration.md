# ROS2 Integration

## Topics

The actuator controller automatically publishes to ROS2 topics:

```bash
# Joint states (sensor_msgs/JointState)
ros2 topic echo /joint_states

# Joint commands (sensor_msgs/JointState)
ros2 topic pub /joint_commands sensor_msgs/JointState ...

# Actuator diagnostics
ros2 topic echo /actuator_diagnostics
```

## Services

```bash
# Emergency stop service
ros2 service call /emergency_stop std_srvs/Trigger

# Set PID parameters
ros2 service call /set_pid_params autogence_msgs/SetPIDParams ...
```

## Python ROS2 Interface

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ActuatorNode(Node):
    def __init__(self):
        super().__init__('actuator_control')

        # Subscribe to joint commands
        self.subscription = self.create_subscription(
            JointState,
            '/joint_commands',
            self.command_callback,
            10
        )

        # Publish joint states
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.01, self.publish_state)  # 100Hz

    def command_callback(self, msg):
        # Execute joint commands
        for i, name in enumerate(msg.name):
            position = msg.position[i]
            self.controller.set_position(actuator_id=i+1, position=position)

    def publish_state(self):
        # Publish current joint states
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for actuator in self.controller.get_actuators():
            state = self.controller.get_state(actuator.id)
            msg.name.append(actuator.name)
            msg.position.append(state.position)
            msg.velocity.append(state.velocity)
            msg.effort.append(state.torque)

        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = ActuatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Available Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | sensor_msgs/JointState | Current position, velocity, and effort |
| `/joint_commands` | sensor_msgs/JointState | Commanded positions |
| `/actuator_diagnostics` | diagnostic_msgs/DiagnosticArray | Health and status info |
| `/emergency_stop` | std_msgs/Bool | Emergency stop state |

## Available Services

| Service | Type | Description |
|---------|------|-------------|
| `/emergency_stop` | std_srvs/Trigger | Trigger emergency stop |
| `/reset_errors` | std_srvs/Trigger | Clear error states |
| `/set_pid_params` | autogence_msgs/SetPIDParams | Update PID parameters |
| `/calibrate` | std_srvs/Trigger | Run calibration routine |

## Launch File

Create a launch file to start the actuator ROS2 node:

```python
# launch/actuator_controller.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autogence_actuator',
            executable='actuator_node',
            name='actuator_controller',
            parameters=[{
                'config_file': '/opt/autogence/actuator_config.yaml',
                'publish_rate': 100.0,
            }],
            output='screen'
        )
    ])
```

Run with:
```bash
ros2 launch autogence_actuator actuator_controller.launch.py
```
