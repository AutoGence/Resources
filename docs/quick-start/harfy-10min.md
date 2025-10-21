# Harfy in 10 Minutes

Get your Harfy robot up and running in just 10 minutes with this quick start guide.

## Prerequisites

- Harfy robot (assembled)
- Robot Domain Controller
- Power supply
- Computer with WiFi capability

**Estimated time: 10 minutes**

## Step 1: Power Up (2 minutes)

1. **Connect Power**
   - Plug the power supply into the domain controller
   - Connect the domain controller to Harfy's power port
   - Press the power button on the domain controller

2. **Status Check**
   - Domain controller LED should be solid green
   - Harfy's eyes (if equipped) should light up
   - You should hear a brief startup chime

## Step 2: Network Connection (3 minutes)

1. **Find Harfy's Network**
   - On your computer, look for WiFi network `Harfy-XXXX`
   - Connect using password: `AutoGence2025`

2. **Access Web Interface**
   - Open browser and go to `http://192.168.4.1`
   - You should see the Harfy setup page

## Step 3: Basic Configuration (3 minutes)

1. **Complete Setup Wizard**
   - Set your robot's name
   - Configure your home WiFi (optional)
   - Accept safety acknowledgments

2. **Verify Connection**
   - Status page should show "Connected"
   - All joints should report "OK" status

## Step 4: First Movement (2 minutes)

1. **Safety First**
   - Ensure Harfy has clear space around it
   - Keep emergency stop button accessible

2. **Test Movement**
   - Click "Demo Movements" in web interface
   - Select "Wave Hello"
   - Press "Execute"

Your Harfy should now wave!

## What's Next?

Now that your Harfy is working, explore these next steps:

### Programming Options
- **Web Interface**: Use the built-in web dashboard for basic control
- **Python SDK**: Install `pip install harfy-sdk` for programmatic control
- **ROS2**: Full ROS2 integration for advanced robotics development

### Tutorials to Try
- First Movement Command (coming soon)
- Programming with Python (coming soon)
- ROS2 Integration (coming soon)

### Join the Community
- [Discord Server](https://discord.gg/autogence) - Get help and share projects
- [GitHub](https://github.com/autogence) - Contribute to open source projects
- [Forum](https://forum.autogence.ai) - Technical discussions and tutorials

## Troubleshooting

**Harfy won't power on?**
- Check power connections
- Verify power supply LED is on
- Try a different power outlet

**Can't connect to WiFi?**
- Make sure you're within range
- Check password is correct: `AutoGence2025`
- Restart your computer's WiFi

**Web interface not loading?**
- Try `http://192.168.4.1:8080`
- Clear browser cache
- Try a different browser

**Movement seems jerky?**
- Ensure stable power supply
- Check for loose connections
- Calibrate joints in settings

## Safety Reminders

**Important Safety Notes:**
- Always maintain clear space around Harfy during operation
- Keep the emergency stop button accessible
- Never leave Harfy unattended while powered
- Read the full safety manual before extended use

---

**Congratulations!** Your Harfy is now ready for action. Welcome to the AutoGence community!