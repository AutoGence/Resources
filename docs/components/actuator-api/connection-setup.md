# Connection and Setup

## EtherCAT Network Configuration

Actuators communicate via EtherCAT in a daisy-chain configuration:

```
Domain Controller → Actuator 1 → Actuator 2 → ... → Actuator N (terminated)
```

## Python SDK

```python
from autogence import ActuatorController

# Initialize controller
controller = ActuatorController()
controller.connect()

# Get all connected actuators
actuators = controller.get_actuators()
print(f"Found {len(actuators)} actuators")
```

## Network Requirements

- EtherCAT-capable network interface
- Proper cable termination on the last actuator
- Unique actuator IDs for each device
- 24V DC power supply for all actuators

## Verification

After setup, verify the connection:

```python
# Check connection status
status = controller.get_connection_status()
print(f"Connected: {status.connected}")
print(f"Actuator count: {status.actuator_count}")

# List all actuators
for actuator in controller.get_actuators():
    print(f"ID: {actuator.id}, Type: {actuator.type}, Status: {actuator.status}")
```
