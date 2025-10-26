# Actuator Control API

Complete API reference for controlling AutoGence joint actuators.

## Overview

AutoGence actuators are smart servo motors with built-in controllers, position/velocity feedback, and EtherCAT communication. Each actuator can be controlled individually or as part of a coordinated system.

This documentation covers:

- **Connection and Setup**: EtherCAT network configuration and Python SDK initialization
- **Basic Control**: Position, velocity, and torque control methods
- **Advanced Features**: PID tuning, compliance control, and motion profiling
- **Sensor Data**: Reading position, velocity, torque, and temperature data
- **Safety Features**: Emergency stop, limits, and protection mechanisms
- **Error Handling**: Exception handling and error recovery
- **ROS2 Integration**: Topics, services, and ROS2 integration
- **Configuration**: YAML configuration files for actuators and network
- **Specifications**: Technical specifications for AG-X20 and AG-X50 actuators
- **Troubleshooting**: Common issues and diagnostic tools
- **Examples**: Code samples and tutorials

## Quick Start

```python
from autogence import ActuatorController

# Initialize controller
controller = ActuatorController()
controller.connect()

# Get all connected actuators
actuators = controller.get_actuators()
print(f"Found {len(actuators)} actuators")

# Move actuator to position
controller.set_position(actuator_id=1, position=90.0, speed=30.0)
```

## Next Steps

Explore the sections in the sidebar to learn more about specific features and capabilities of the Actuator Control API.
