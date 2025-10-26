# Basic Control

## Position Control

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

## Velocity Control

```python
# Set velocity (degrees/second)
controller.set_velocity(actuator_id=1, velocity=60.0)

# Velocity control with acceleration limit
controller.set_velocity(actuator_id=1, velocity=60.0, acceleration=120.0)
```

## Torque Control

```python
# Set torque (Nm)
controller.set_torque(actuator_id=1, torque=2.5)

# Torque with position limits
controller.set_torque(actuator_id=1, torque=2.5,
                     min_position=-90.0, max_position=90.0)
```

## Control Modes

The actuator supports three primary control modes:

1. **Position Mode**: Precise positioning with speed control
2. **Velocity Mode**: Constant speed operation with acceleration limiting
3. **Torque Mode**: Force control with optional position bounds

Switch between modes automatically by calling the respective control method.
