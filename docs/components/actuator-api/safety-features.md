# Safety Features

## Emergency Stop

```python
# Emergency stop all actuators
controller.emergency_stop()

# Resume after emergency stop
controller.reset()
```

The emergency stop function immediately halts all actuator movement and disables motor power. Always call `reset()` before resuming normal operation.

## Limits and Protection

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

## Safety Mechanisms

The actuator controller includes multiple safety mechanisms:

### Hardware Protection
- **Overcurrent protection**: Automatically shuts down on excessive current draw
- **Overvoltage/undervoltage protection**: Protects against power supply issues
- **Thermal shutdown**: Prevents overheating damage
- **Short circuit protection**: Safeguards motor windings

### Software Protection
- **Position limits**: Software-enforced min/max positions
- **Velocity limits**: Maximum speed restrictions
- **Torque limits**: Force output restrictions
- **Collision detection**: Detects unexpected resistance

### Watchdog Timer
The controller includes a watchdog that triggers emergency stop if:
- No commands received within timeout period (default: 1 second)
- Communication with any actuator is lost
- Controller process crashes or hangs

```python
# Configure watchdog timeout
controller.set_watchdog_timeout(timeout=2.0)  # seconds

# Disable watchdog (not recommended)
controller.set_watchdog_timeout(timeout=0)  # 0 = disabled
```
