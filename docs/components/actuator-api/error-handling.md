# Error Handling

## Exception Types

```python
from autogence.exceptions import ActuatorError, CommunicationError

try:
    controller.set_position(actuator_id=1, position=90.0)
except ActuatorError as e:
    print(f"Actuator error: {e}")
except CommunicationError as e:
    print(f"Communication error: {e}")
```

## Common Exceptions

### ActuatorError
Raised when an actuator encounters a problem:
- Position out of bounds
- Torque limit exceeded
- Thermal shutdown
- Hardware fault

### CommunicationError
Raised when communication fails:
- EtherCAT timeout
- Actuator not responding
- Network disconnection
- Invalid actuator ID

### ConfigurationError
Raised for configuration issues:
- Invalid parameters
- Missing configuration file
- Incompatible actuator type

## Error Recovery

```python
from autogence.exceptions import ActuatorError

def safe_move(actuator_id, position):
    """Move actuator with automatic error recovery"""
    max_retries = 3
    retry_count = 0

    while retry_count < max_retries:
        try:
            controller.set_position(actuator_id, position)
            return True
        except ActuatorError as e:
            print(f"Attempt {retry_count + 1} failed: {e}")
            retry_count += 1
            controller.reset()  # Reset error state
            time.sleep(0.5)  # Wait before retry

    return False  # All retries exhausted
```

## Error Codes

| Code | Name | Description | Recovery |
|------|------|-------------|----------|
| E001 | POSITION_LIMIT | Position limit exceeded | Adjust target position |
| E002 | VELOCITY_LIMIT | Velocity limit exceeded | Reduce speed |
| E003 | TORQUE_LIMIT | Torque limit exceeded | Reduce load or limit |
| E004 | OVERHEAT | Temperature too high | Allow cooling, check ventilation |
| E005 | COMM_TIMEOUT | Communication timeout | Check cables, restart controller |
| E006 | ENCODER_ERROR | Position sensor fault | Check encoder, replace if needed |
| E007 | MOTOR_FAULT | Motor driver error | Check motor, reset controller |

## Logging

Enable detailed logging for troubleshooting:

```python
import logging

# Enable debug logging
logging.basicConfig(level=logging.DEBUG)
controller = ActuatorController(log_level=logging.DEBUG)

# Log to file
controller.set_log_file("/var/log/autogence/actuator.log")
```
