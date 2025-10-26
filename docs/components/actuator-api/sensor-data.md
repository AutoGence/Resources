# Sensor Data

## Position and Velocity Feedback

```python
# Get current position
position = controller.get_position(actuator_id=1)
print(f"Current position: {position:.2f} degrees")

# Get velocity
velocity = controller.get_velocity(actuator_id=1)
print(f"Current velocity: {velocity:.2f} deg/s")

# Get multiple sensors at once
state = controller.get_state(actuator_id=1)
print(f"Position: {state.position:.2f}°")
print(f"Velocity: {state.velocity:.2f}°/s")
print(f"Torque: {state.torque:.2f} Nm")
print(f"Temperature: {state.temperature:.1f}°C")
```

## Continuous Monitoring

```python
# Stream sensor data
def on_sensor_data(actuator_id, state):
    print(f"Actuator {actuator_id}: {state.position:.1f}°")

controller.start_monitoring(callback=on_sensor_data, rate=100)  # 100Hz
controller.stop_monitoring()
```

## Available Sensor Data

Each actuator provides real-time feedback on:

- **Position**: Angular position in degrees (0.01° resolution)
- **Velocity**: Angular velocity in degrees/second
- **Torque**: Current torque output in Nm
- **Temperature**: Internal motor temperature in °C
- **Current**: Motor current draw in Amperes
- **Voltage**: Supply voltage in Volts

## Sensor Accuracy

| Sensor | Resolution | Update Rate | Accuracy |
|--------|-----------|-------------|----------|
| Position | 0.01° | 1 kHz | ±0.05° |
| Velocity | 0.1°/s | 1 kHz | ±1.0°/s |
| Torque | 0.01 Nm | 1 kHz | ±2% |
| Temperature | 0.1°C | 10 Hz | ±1°C |
