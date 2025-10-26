# Examples

## Basic Movement Example

```python
from autogence import ActuatorController
import time

# Initialize controller
controller = ActuatorController()
controller.connect()

# Move actuator to home position
controller.set_position(actuator_id=1, position=0.0, speed=30.0)
controller.wait_for_completion()

# Move to target position
controller.set_position(actuator_id=1, position=90.0, speed=45.0)
controller.wait_for_completion()

# Move back to home
controller.set_position(actuator_id=1, position=0.0, speed=30.0)
controller.wait_for_completion()

controller.disconnect()
```

## Multi-Actuator Coordination

```python
from autogence import ActuatorController

controller = ActuatorController()
controller.connect()

# Define joint positions for a pose
joint_positions = {
    1: 45.0,    # Shoulder pitch
    2: -30.0,   # Shoulder roll
    3: 90.0,    # Elbow
    4: 0.0,     # Wrist pitch
    5: 0.0,     # Wrist roll
}

# Move all joints simultaneously
controller.set_positions(joint_positions, speed=45.0)
controller.wait_for_completion(timeout=5.0)

print("Pose achieved!")
```

## Sensor Monitoring Example

```python
from autogence import ActuatorController
import time

controller = ActuatorController()
controller.connect()

# Monitor actuator state for 10 seconds
start_time = time.time()
while time.time() - start_time < 10.0:
    state = controller.get_state(actuator_id=1)

    print(f"Position: {state.position:6.2f}° | "
          f"Velocity: {state.velocity:6.2f}°/s | "
          f"Torque: {state.torque:5.2f} Nm | "
          f"Temp: {state.temperature:4.1f}°C")

    time.sleep(0.1)  # 10Hz update rate
```

## Smooth Motion Profile

```python
from autogence import ActuatorController
from autogence.motion import MotionProfile

controller = ActuatorController()
controller.connect()

# Create smooth motion profile
profile = MotionProfile(
    target_position=180.0,
    max_velocity=60.0,
    max_acceleration=120.0,
    jerk_limit=360.0  # Smooth acceleration changes
)

# Execute profile
controller.execute_profile(actuator_id=1, profile=profile)

# Monitor progress
while not controller.is_motion_complete(actuator_id=1):
    state = controller.get_state(actuator_id=1)
    print(f"Progress: {state.position:.1f}° / 180.0°")
    time.sleep(0.1)

print("Motion complete!")
```

## Compliance Mode for Safe Interaction

```python
from autogence import ActuatorController

controller = ActuatorController()
controller.connect()

# Enable compliance mode with low stiffness
controller.set_compliance(actuator_id=1, enabled=True, stiffness=0.3)

print("Actuator is now compliant - you can move it by hand")
print("Press Ctrl+C to exit")

try:
    while True:
        # Monitor position changes from external forces
        position = controller.get_position(actuator_id=1)
        print(f"Current position: {position:.2f}°")
        time.sleep(0.5)
except KeyboardInterrupt:
    pass

# Restore rigid mode
controller.set_compliance(actuator_id=1, enabled=False)
controller.disconnect()
```

## ROS2 Joint State Publisher

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from autogence import ActuatorController

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.01, self.publish_states)  # 100Hz

        self.controller = ActuatorController()
        self.controller.connect()

        self.joint_names = ['shoulder_pitch', 'shoulder_roll', 'elbow']

    def publish_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for i, name in enumerate(self.joint_names):
            state = self.controller.get_state(actuator_id=i+1)
            msg.name.append(name)
            msg.position.append(state.position * 3.14159 / 180.0)  # Convert to radians
            msg.velocity.append(state.velocity * 3.14159 / 180.0)
            msg.effort.append(state.torque)

        self.publisher.publish(msg)

def main():
    rclpy.init()
    publisher = JointStatePublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Error Handling and Recovery

```python
from autogence import ActuatorController
from autogence.exceptions import ActuatorError, CommunicationError
import time

controller = ActuatorController()
controller.connect()

def safe_move_with_retry(actuator_id, position, max_retries=3):
    """Attempt movement with automatic retry on errors"""
    for attempt in range(max_retries):
        try:
            controller.set_position(actuator_id, position, speed=45.0)
            controller.wait_for_completion(timeout=5.0)
            print(f"Successfully moved to {position}°")
            return True

        except ActuatorError as e:
            print(f"Attempt {attempt + 1} failed: {e}")
            if attempt < max_retries - 1:
                controller.reset()  # Clear error state
                time.sleep(1.0)  # Wait before retry
            else:
                print("All retries exhausted")
                return False

        except CommunicationError as e:
            print(f"Communication error: {e}")
            print("Attempting to reconnect...")
            controller.disconnect()
            time.sleep(2.0)
            controller.connect()
            if attempt >= max_retries - 1:
                return False

    return False

# Use the safe move function
if safe_move_with_retry(actuator_id=1, position=90.0):
    print("Movement succeeded")
else:
    print("Movement failed after all retries")
```

## Complete Robot Arm Example

```python
from autogence import ActuatorController
from autogence.motion import MotionProfile
import time

class RobotArm:
    def __init__(self):
        self.controller = ActuatorController()
        self.controller.connect()

        # Define joint mapping
        self.joints = {
            'shoulder_pitch': 1,
            'shoulder_roll': 2,
            'elbow': 3,
            'wrist_pitch': 4,
            'wrist_roll': 5,
        }

    def move_to_pose(self, pose_dict, speed=45.0):
        """Move arm to named joint positions"""
        joint_positions = {}
        for joint_name, position in pose_dict.items():
            joint_id = self.joints[joint_name]
            joint_positions[joint_id] = position

        self.controller.set_positions(joint_positions, speed=speed)
        self.controller.wait_for_completion(timeout=10.0)

    def get_current_pose(self):
        """Get current joint positions"""
        pose = {}
        for joint_name, joint_id in self.joints.items():
            position = self.controller.get_position(joint_id)
            pose[joint_name] = position
        return pose

    def home(self):
        """Move to home position"""
        home_pose = {
            'shoulder_pitch': 0.0,
            'shoulder_roll': 0.0,
            'elbow': 0.0,
            'wrist_pitch': 0.0,
            'wrist_roll': 0.0,
        }
        self.move_to_pose(home_pose, speed=30.0)

    def wave(self):
        """Perform a waving motion"""
        # Move to wave start position
        self.move_to_pose({
            'shoulder_pitch': 45.0,
            'shoulder_roll': -60.0,
            'elbow': 90.0,
        })

        # Wave motion
        for _ in range(3):
            self.controller.set_position(
                actuator_id=self.joints['wrist_roll'],
                position=45.0,
                speed=90.0
            )
            time.sleep(0.5)

            self.controller.set_position(
                actuator_id=self.joints['wrist_roll'],
                position=-45.0,
                speed=90.0
            )
            time.sleep(0.5)

        # Return to home
        self.home()

# Use the robot arm
arm = RobotArm()
arm.home()
arm.wave()
print("Current pose:", arm.get_current_pose())
```

## More Examples

See the [actuator-examples repository](https://github.com/autogence/actuator-examples) for additional code samples including:

- Pick and place operations
- Force-controlled manipulation
- Trajectory following
- Inverse kinematics integration
- Vision-guided control
- Multi-robot coordination
