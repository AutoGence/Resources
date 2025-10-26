# Troubleshooting

## Common Issues

### Actuator not responding

**Symptoms:**
- Controller cannot detect actuator
- No response to commands
- Communication timeout errors

**Solutions:**
1. Check EtherCAT cable connections
2. Verify power supply voltage (24V)
3. Check actuator ID configuration
4. Ensure proper cable termination
5. Verify network interface is up

```bash
# Check EtherCAT interface
ip link show eth0

# Verify actuator power
# Green LED should be solid on each actuator
```

### Position drift

**Symptoms:**
- Actuator slowly moves from commanded position
- Position error accumulates over time
- Inconsistent positioning

**Solutions:**
1. Recalibrate zero position
2. Check for mechanical backlash
3. Adjust PID parameters
4. Verify encoder functionality
5. Check for external forces/friction

```python
# Recalibrate actuator
controller.calibrate(actuator_id=1)

# Verify position stability
state = controller.get_state(actuator_id=1)
print(f"Position stability: {state.position_variance}")
```

### Communication errors

**Symptoms:**
- Intermittent connection loss
- EtherCAT timeout errors
- Inconsistent sensor data

**Solutions:**
1. Check EtherCAT termination (required on last actuator)
2. Verify cycle time settings match all devices
3. Check for electromagnetic interference
4. Use shielded CAT5e/CAT6 cables
5. Reduce cable length or add repeater

```python
# Check EtherCAT status
status = controller.get_ethercat_status()
print(f"EtherCAT state: {status.state}")
print(f"Working counter errors: {status.wc_errors}")
print(f"Lost frames: {status.lost_frames}")
```

### Overheating

**Symptoms:**
- Thermal shutdown errors
- Reduced performance
- Actuator feels hot to touch

**Solutions:**
1. Allow cooling period (10-15 minutes)
2. Check ambient temperature
3. Improve ventilation/airflow
4. Reduce continuous torque load
5. Add heat sinks if operating in high-duty cycles

```python
# Monitor temperature
temp = controller.get_temperature(actuator_id=1)
print(f"Temperature: {temp}°C")

# Enable thermal protection
controller.set_thermal_protection(actuator_id=1, max_temp=65.0)
```

### Excessive vibration or noise

**Symptoms:**
- Audible noise during movement
- Vibration at certain positions
- Jerky motion

**Solutions:**
1. Reduce PID gain (especially D term)
2. Lower acceleration limits
3. Enable motion profiling with jerk limits
4. Check for mechanical resonance
5. Verify mounting is secure

```python
# Reduce PID gains
controller.set_pid_params(actuator_id=1, kp=0.8, ki=0.05, kd=0.02)

# Enable smooth motion profiling
from autogence.motion import MotionProfile
profile = MotionProfile(
    target_position=90.0,
    max_velocity=45.0,
    max_acceleration=90.0,
    jerk_limit=180.0  # Lower value = smoother
)
controller.execute_profile(actuator_id=1, profile=profile)
```

## Diagnostic Commands

```python
# Run built-in diagnostics
diagnostics = controller.run_diagnostics(actuator_id=1)
print(diagnostics.report)

# Check EtherCAT status
ethercat_status = controller.get_ethercat_status()
print(f"EtherCAT state: {ethercat_status.state}")

# Verify actuator health
health = controller.get_health(actuator_id=1)
print(f"Overall health: {health.score}/100")
print(f"Issues: {health.warnings}")
```

## Error Code Reference

| Code | Description | Solution |
|------|-------------|----------|
| E001 | Position limit exceeded | Reduce target position |
| E002 | Velocity limit exceeded | Reduce speed setting |
| E003 | Torque limit exceeded | Reduce load or increase limit |
| E004 | Overheating | Allow cooling, check ventilation |
| E005 | Communication timeout | Check cables, restart controller |
| E006 | Encoder error | Check encoder, replace if faulty |
| E007 | Motor driver fault | Reset controller, check motor |
| E008 | Power supply fault | Check 24V supply |
| E009 | Configuration error | Verify config file |
| E010 | Calibration required | Run calibration routine |

## Logging and Debug Mode

Enable detailed logging for troubleshooting:

```python
import logging

# Enable debug logging
logging.basicConfig(level=logging.DEBUG)
controller = ActuatorController(log_level=logging.DEBUG)

# Log to file
controller.set_log_file("/var/log/autogence/actuator_debug.log")

# Enable verbose EtherCAT logging
controller.set_ethercat_debug(enabled=True)
```

## Getting Help

If you can't resolve the issue:

1. **Check documentation**: Review the relevant API documentation section
2. **Search GitHub issues**: https://github.com/autogence/actuator-sdk/issues
3. **Community forum**: https://forum.autogence.ai
4. **Discord support**: https://discord.gg/autogence
5. **Contact support**: support@autogence.ai

When reporting issues, include:
- Software version (`controller.get_version()`)
- Actuator model and firmware version
- Full error message and stack trace
- Configuration file
- Debug log output
- Steps to reproduce
