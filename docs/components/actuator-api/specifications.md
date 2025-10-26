# Specifications

## AG-X20 Actuator

| Parameter | Value |
|-----------|-------|
| Max Torque | 20 Nm |
| Max Speed | 180°/s |
| Position Resolution | 0.01° |
| Communication | EtherCAT |
| Voltage | 24V DC |
| Weight | 500g |
| Dimensions | 80mm × 80mm × 60mm |
| Operating Temperature | -10°C to 60°C |
| Protection Rating | IP54 |

### Typical Applications
- Humanoid robot arms (shoulder, elbow joints)
- Gripper mechanisms
- Pan-tilt camera systems
- Small mobile manipulators

## AG-X50 Actuator

| Parameter | Value |
|-----------|-------|
| Max Torque | 50 Nm |
| Max Speed | 120°/s |
| Position Resolution | 0.01° |
| Communication | EtherCAT |
| Voltage | 24V DC |
| Weight | 800g |
| Dimensions | 100mm × 100mm × 75mm |
| Operating Temperature | -10°C to 60°C |
| Protection Rating | IP54 |

### Typical Applications
- Humanoid robot legs (hip, knee joints)
- Heavy-duty manipulators
- Industrial automation
- Legged robot joints

## Performance Characteristics

### Position Accuracy
- **Repeatability**: ±0.03°
- **Absolute accuracy**: ±0.1° (after calibration)
- **Hysteresis**: &lt;0.05°

### Velocity Control
- **Velocity range**: 0.1°/s to max speed
- **Velocity accuracy**: ±2%
- **Acceleration**: Up to 500°/s²

### Torque Control
- **Torque range**: 0.01 Nm to max torque
- **Torque accuracy**: ±3%
- **Response time**: &lt;1ms

## Communication Specifications

### EtherCAT
- **Protocol**: EtherCAT CoE (CAN over EtherCAT)
- **Update rate**: 1 kHz
- **Latency**: &lt;1ms
- **Topology**: Daisy-chain
- **Cable**: CAT5e or better

### Data Exchange
- **Input data**: Position, velocity, torque, temperature, status
- **Output data**: Position command, velocity command, torque command, control mode
- **PDO mapping**: Configurable

## Electrical Specifications

### Power Requirements
- **Voltage**: 24V DC ±10%
- **Current (AG-X20)**: 3A continuous, 6A peak
- **Current (AG-X50)**: 5A continuous, 10A peak
- **Efficiency**: &gt;85% at rated load

### Protection
- **Overcurrent**: Hardware-protected
- **Overvoltage**: 32V absolute max
- **Undervoltage**: 18V shutdown
- **Reverse polarity**: Protected

## Mechanical Specifications

### Gearbox
- **Type**: Harmonic drive
- **Gear ratio**: 50:1 or 100:1 (configurable)
- **Backlash**: &lt;0.05°
- **Efficiency**: &gt;80%

### Motor
- **Type**: Brushless DC
- **Phases**: 3-phase
- **Encoder**: Absolute magnetic encoder
- **Resolution**: 14-bit (16384 counts/revolution)

## Environmental

### Operating Conditions
- **Temperature**: -10°C to 60°C
- **Humidity**: 20% to 80% RH (non-condensing)
- **Altitude**: Up to 2000m
- **Vibration**: 5G, 10-200Hz

### Storage Conditions
- **Temperature**: -20°C to 70°C
- **Humidity**: 10% to 90% RH (non-condensing)

## Certifications

- CE marked
- RoHS compliant
- FCC Part 15 Class A
