# Configuration

## Actuator Settings

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

  3:
    name: "elbow"
    type: "AG-X20"
    gear_ratio: 100
    position_offset: 0.0
    invert_direction: false
```

## Network Settings

```yaml
ethercat:
  cycle_time: 1000  # microseconds
  sync_mode: true
  distributed_clocks: true
  interface: "eth0"  # Network interface for EtherCAT
```

## Control Parameters

```yaml
control:
  default_velocity: 45.0  # degrees/second
  default_acceleration: 120.0  # degrees/second^2
  position_tolerance: 0.5  # degrees
  watchdog_timeout: 1.0  # seconds
```

## Safety Limits

```yaml
safety:
  global_velocity_limit: 180.0  # degrees/second
  global_torque_limit: 20.0  # Nm
  thermal_shutdown_temp: 75.0  # degrees Celsius
  enable_collision_detection: true
```

## Configuration File Location

The configuration file can be placed in multiple locations (checked in order):

1. `/opt/autogence/actuator_config.yaml` (system-wide)
2. `~/.config/autogence/actuator_config.yaml` (user-specific)
3. `./actuator_config.yaml` (current directory)
4. Custom path via environment variable: `AUTOGENCE_CONFIG`

## Loading Custom Configuration

```python
from autogence import ActuatorController

# Load specific config file
controller = ActuatorController(config_file="/path/to/custom_config.yaml")

# Or use environment variable
import os
os.environ['AUTOGENCE_CONFIG'] = '/path/to/custom_config.yaml'
controller = ActuatorController()
```

## Per-Actuator Overrides

You can override configuration for individual actuators:

```python
# Override position offset for calibration
controller.set_config(actuator_id=1, position_offset=5.0)

# Override velocity limit
controller.set_config(actuator_id=2, max_velocity=90.0)

# Save current configuration
controller.save_config("/path/to/save_config.yaml")
```
