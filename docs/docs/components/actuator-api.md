# Actuator Control API

Complete API reference for controlling AutoGence joint actuators.

## Overview

AutoGence actuators are smart servo motors with built-in controllers, position/velocity feedback, and EtherCAT communication. Each actuator can be controlled individually or as part of a coordinated system.

## Connection and Setup

### EtherCAT Network Configuration

Actuators communicate via EtherCAT in a daisy-chain configuration:

```
Domain Controller → Actuator 1 → Actuator 2 → ... → Actuator N (terminated)
```

### Python SDK

```python
from autogence import ActuatorController

# Initialize controller
controller = ActuatorController()
controller.connect()

# Get all connected actuators
actuators = controller.get_actuators()
print(f"Found {len(actuators)} actuators")
```

## Basic Control

### Position Control

```python
# Move single actuator to position (degrees)
controller.set_position(actuator_id=1, position=90.0, speed=30.0)

# Move multiple actuators simultaneously
positions = {
    1: 45.0,    # Shoulder pitch
    2: -30.0,   # Shoulder roll
    3: 90.0,    # Elbow
}
controller.set_positions(positions, speed=45.0)

# Wait for movement completion
controller.wait_for_completion(timeout=5.0)
```

### Velocity Control

```python
# Set velocity (degrees/second)
controller.set_velocity(actuator_id=1, velocity=60.0)

# Velocity control with acceleration limit
controller.set_velocity(actuator_id=1, velocity=60.0, acceleration=120.0)
```

### Torque Control

```python
# Set torque (Nm)
controller.set_torque(actuator_id=1, torque=2.5)

# Torque with position limits
controller.set_torque(actuator_id=1, torque=2.5,
                     min_position=-90.0, max_position=90.0)
```

## Advanced Features

### PID Tuning

```python
# Get current PID parameters
pid = controller.get_pid_params(actuator_id=1)
print(f"P={pid.kp}, I={pid.ki}, D={pid.kd}")

# Update PID parameters
controller.set_pid_params(actuator_id=1, kp=1.2, ki=0.1, kd=0.05)
```

### Compliance Control

```python
# Enable compliance mode (for safe human interaction)
controller.set_compliance(actuator_id=1, enabled=True, stiffness=0.3)

# Disable compliance (rigid positioning)
controller.set_compliance(actuator_id=1, enabled=False)
```

### Motion Profiling

```python
# Smooth motion with custom profile
from autogence.motion import MotionProfile

profile = MotionProfile(
    target_position=90.0,
    max_velocity=60.0,
    max_acceleration=120.0,
    jerk_limit=360.0
)

controller.execute_profile(actuator_id=1, profile=profile)
```

## Sensor Data

### Position and Velocity Feedback

```python
# Get current position
position = controller.get_position(actuator_id=1)
print(f"Current position: {position:.2f} degrees")

# Get velocity
velocity = controller.get_velocity(actuator_id=1)
print(f"Current velocity: {velocity:.2f} deg/s")

# Get multiple sensors at once
state = controller.get_state(actuator_id=1)
print(f"Position: {state.position:.2f}°")
print(f"Velocity: {state.velocity:.2f}°/s")
print(f"Torque: {state.torque:.2f} Nm")
print(f"Temperature: {state.temperature:.1f}°C")
```

### Continuous Monitoring

```python
# Stream sensor data
def on_sensor_data(actuator_id, state):
    print(f"Actuator {actuator_id}: {state.position:.1f}°")

controller.start_monitoring(callback=on_sensor_data, rate=100)  # 100Hz
controller.stop_monitoring()
```

## Safety Features

### Emergency Stop

```python
# Emergency stop all actuators
controller.emergency_stop()

# Resume after emergency stop
controller.reset()
```

### Limits and Protection

```python
# Set position limits
controller.set_position_limits(actuator_id=1, min_pos=-90.0, max_pos=90.0)

# Set velocity limits
controller.set_velocity_limit(actuator_id=1, max_velocity=180.0)

# Set torque limits
controller.set_torque_limit(actuator_id=1, max_torque=5.0)

# Enable thermal protection
controller.set_thermal_protection(actuator_id=1, max_temp=70.0)
```

## Error Handling

```python
from autogence.exceptions import ActuatorError, CommunicationError

try:
    controller.set_position(actuator_id=1, position=90.0)
except ActuatorError as e:
    print(f"Actuator error: {e}")
except CommunicationError as e:
    print(f"Communication error: {e}")
```

## ROS2 Integration

### Topics

The actuator controller automatically publishes to ROS2 topics:

```bash
# Joint states (sensor_msgs/JointState)
ros2 topic echo /joint_states

# Joint commands (sensor_msgs/JointState)
ros2 topic pub /joint_commands sensor_msgs/JointState ...

# Actuator diagnostics
ros2 topic echo /actuator_diagnostics
```

### Services

```bash
# Emergency stop service
ros2 service call /emergency_stop std_srvs/Trigger

# Set PID parameters
ros2 service call /set_pid_params autogence_msgs/SetPIDParams ...
```

## Configuration

### Actuator Settings

Edit `/opt/autogence/actuator_config.yaml`:

```yaml
actuators:
  1:
    name: "shoulder_pitch"
    type: "AG-X20"
    gear_ratio: 100
    position_offset: 0.0
    invert_direction: false

  2:
    name: "shoulder_roll"
    type: "AG-X20"
    gear_ratio: 100
    position_offset: 0.0
    invert_direction: true
```

### Network Settings

```yaml
ethercat:
  cycle_time: 1000  # microseconds
  sync_mode: true
  distributed_clocks: true
```

## Specifications

### AG-X20 Actuator

| Parameter | Value |
|-----------|-------|
| Max Torque | 20 Nm |
| Max Speed | 180°/s |
| Position Resolution | 0.01° |
| Communication | EtherCAT |
| Voltage | 24V DC |
| Weight | 500g |

### AG-X50 Actuator

| Parameter | Value |
|-----------|-------|
| Max Torque | 50 Nm |
| Max Speed | 120°/s |
| Position Resolution | 0.01° |
| Communication | EtherCAT |
| Voltage | 24V DC |
| Weight | 800g |

## Troubleshooting

### Common Issues

**Actuator not responding:**
- Check EtherCAT cable connections
- Verify power supply voltage (24V)
- Check actuator ID configuration

**Position drift:**
- Recalibrate zero position
- Check for mechanical backlash
- Adjust PID parameters

**Communication errors:**
- Check EtherCAT termination
- Verify cycle time settings
- Check for electromagnetic interference

### Diagnostic Commands

```python
# Run built-in diagnostics
diagnostics = controller.run_diagnostics(actuator_id=1)
print(diagnostics.report)

# Check EtherCAT status
ethercat_status = controller.get_ethercat_status()
print(f"EtherCAT state: {ethercat_status.state}")
```

## Examples

See the [examples repository](https://github.com/autogence/actuator-examples) for complete code samples and tutorials.