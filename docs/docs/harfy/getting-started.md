# Getting Started with Harfy

Welcome to Harfy! This guide will walk you through setting up your new humanoid robot.

## What's in the Box

- Harfy humanoid robot (20 DOF)
- Robot Domain Controller
- Power supply and cables
- Quick start guide
- Safety instructions

## Initial Setup

### Step 1: Unboxing and Inspection

1. Carefully remove Harfy from the packaging
2. Inspect all components for any shipping damage
3. Ensure all joints move freely by hand

### Step 2: Power Connection

1. Connect the power supply to the Robot Domain Controller
2. Connect the domain controller to Harfy's main power port
3. Press the power button - LED should turn green

### Step 3: Network Configuration

1. Connect to Harfy's WiFi hotspot (SSID: `Harfy-XXXX`)
2. Open your browser and navigate to `192.168.4.1`
3. Complete the initial setup wizard

## First Movement

Once setup is complete, you can send your first movement command:

```python
from harfy_sdk import HarfyClient

client = HarfyClient()
client.connect()

# Wave hello!
client.move_arm_left(wave_pattern=True)
```

## Next Steps

- Programming Harfy (coming soon)
- Safety Guidelines (coming soon)
- Troubleshooting (coming soon)

## Need Help?

Join our [Discord community](https://discord.gg/autogence) for support and to connect with other Harfy users.