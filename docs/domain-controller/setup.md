# Domain Controller Setup

The Robot Domain Controller is the brain of your AutoGence robot system, providing real-time control and communication.

## Overview

The Domain Controller handles:
- Real-time joint control via EtherCAT
- ROS2 message routing
- Safety monitoring
- OTA update management

## Hardware Installation

### Prerequisites

- Robot Domain Controller unit
- EtherCAT cable (included)
- Power supply (24V, 5A minimum)
- Ethernet cable for network connection

### Installation Steps

1. **Mount the Controller**
   - Secure the domain controller in your robot chassis
   - Ensure adequate ventilation around the unit

2. **Power Connection**
   - Connect 24V power supply to the controller
   - Verify LED indicators show normal operation

3. **EtherCAT Network**
   - Connect actuators in daisy-chain configuration
   - Terminate the last actuator in the chain

## Software Configuration

### Initial Setup

1. Connect via Ethernet and navigate to the web interface
2. Configure network settings
3. Set up EtherCAT topology
4. Calibrate connected actuators

### ROS2 Integration

The domain controller runs ROS2 Humble by default:

```bash
# Check ROS2 topics
ros2 topic list

# Monitor joint states
ros2 topic echo /joint_states

# Send position commands
ros2 topic pub /joint_commands sensor_msgs/JointState ...
```

### Configuration Files

Key configuration files:
- `/etc/ethercat.conf` - EtherCAT network settings
- `/opt/autogence/config.yaml` - Domain controller parameters
- `/etc/systemd/system/autogence-controller.service` - Service configuration

## Troubleshooting

### Common Issues

**EtherCAT Communication Errors**
- Check cable connections and termination
- Verify actuator power status
- Review EtherCAT logs: `journalctl -u ethercat`

**ROS2 Topics Not Available**
- Restart the domain controller service
- Check network connectivity
- Verify ROS2 environment variables

### Diagnostic Commands

```bash
# Check EtherCAT status
sudo ethercat master

# View system logs
journalctl -u autogence-controller -f

# Test actuator communication
autogence-cli test-actuators
```

## Advanced Configuration

### Custom Joint Mappings

Edit `/opt/autogence/joint_mappings.yaml` to customize joint configurations:

```yaml
joints:
  left_shoulder_pitch:
    ethercat_address: 0
    position_limits: [-90, 90]
    velocity_limit: 180
```

### Safety Parameters

Configure safety limits in `/opt/autogence/safety.yaml`:

```yaml
safety:
  emergency_stop_enabled: true
  max_joint_velocity: 180  # degrees/sec
  collision_detection: true
```

## Next Steps

- EtherCAT Configuration (coming soon)
- ROS2 Integration Guide (coming soon)
- Safety Configuration (coming soon)