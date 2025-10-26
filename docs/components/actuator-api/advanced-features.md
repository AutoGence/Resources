# Advanced Features

## PID Tuning

```python
# Get current PID parameters
pid = controller.get_pid_params(actuator_id=1)
print(f"P={pid.kp}, I={pid.ki}, D={pid.kd}")

# Update PID parameters
controller.set_pid_params(actuator_id=1, kp=1.2, ki=0.1, kd=0.05)
```

## Compliance Control

```python
# Enable compliance mode (for safe human interaction)
controller.set_compliance(actuator_id=1, enabled=True, stiffness=0.3)

# Disable compliance (rigid positioning)
controller.set_compliance(actuator_id=1, enabled=False)
```

Compliance mode allows the actuator to yield to external forces, making it safe for human-robot interaction. The stiffness parameter (0.0 to 1.0) controls how much the actuator resists external forces.

## Motion Profiling

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

Motion profiling ensures smooth, controlled movements by limiting velocity, acceleration, and jerk (rate of acceleration change). This is essential for:

- Preventing mechanical stress
- Reducing vibrations
- Achieving smooth, natural-looking movements
- Extending actuator lifespan
