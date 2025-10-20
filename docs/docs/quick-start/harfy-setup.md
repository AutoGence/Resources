---
sidebar_position: 2
---

# Harfy in 10 Minutes

Get your Harfy robot assembled, powered on, and running its first demo program.

:::tip You'll love this!
By the end of this guide, your Harfy will be walking, waving, and responding to basic commands. The magic happens fast!
:::

## Prerequisites

Before you start, make sure you have:
- Harfy robot (unboxed)
- Power adapter (included in box)
- WiFi network with internet access
- Smartphone or computer with web browser

:::info Estimated Time
**10 minutes** from unboxing to first movement
:::

## Step 1: Physical Assembly (3 minutes)

1. **Remove Harfy from packaging**
   - Handle by the torso, avoid grabbing limbs
   - Remove all protective foam

2. **Check joint positions**
   - All joints should be in "neutral" position
   - If any joint looks bent, gently move it to center

3. **Connect the power cable**
   - Locate the power port on Harfy's back
   - Connect the power adapter (don't plug into wall yet)

:::warning Safety First
Always connect the cable to Harfy BEFORE plugging into the wall outlet.
:::

## Step 2: Initial Power On (2 minutes)

1. **Plug in the power adapter**
   - Harfy's chest LED will turn blue
   - You'll hear a brief startup chime

2. **Wait for WiFi setup mode**
   - Chest LED will flash blue/white (about 30 seconds)
   - This means Harfy is ready for WiFi setup

3. **If LED turns solid red**
   - This indicates a hardware issue
   - Unplug for 10 seconds and try again
   - Contact support if issue persists

## Step 3: WiFi Connection (3 minutes)

1. **Open WiFi settings on your device**
   - Look for network: `Harfy-XXXX` (where XXXX is a unique ID)
   - Connect to this network (no password needed)

2. **Setup portal will open automatically**
   - If it doesn't, go to: `http://192.168.4.1`
   - Select your home WiFi network
   - Enter your WiFi password

3. **Wait for connection**
   - Harfy's LED will turn solid blue when connected
   - This may take 1-2 minutes

:::tip Pro Tip
Write down the IP address shown on the setup page - you'll use this for advanced programming later!
:::

## Step 4: First Movement (2 minutes)

1. **Access the control interface**
   - On your device, go to: `http://harfy.local` or use the IP from step 3
   - You'll see the Harfy Control Dashboard

2. **Run your first demo**
   - Click **"Wave Hello"** - Harfy will wave at you!
   - Try **"Stand Up"** - Harfy will demonstrate its balance
   - Click **"Dance"** - Watch a short dance routine

3. **Basic movement controls**
   - Use the directional pad to make Harfy walk
   - Adjust speed with the slider
   - Try the **"Stop"** button to halt all movement

## Success! What You've Accomplished

Your Harfy is now:
- Assembled and powered
- Connected to your WiFi
- Responding to basic commands
- Ready for programming and customization

## Next Steps

Now that Harfy is working, here's what you can do next:

### **Immediate Next Steps**
- **API Programming** - Control Harfy programmatically (guides coming soon)
- **Voice Commands** - "Harfy, walk forward" (feature coming soon)
- **Mobile App** - Control from anywhere (app coming soon)

### **Dive Deeper**
- **Programming Tutorial** - Write your first Harfy program (coming soon)
- **Custom Behaviors** - Teach Harfy new tricks (coming soon)
- **ROS2 Integration** - Advanced robotics programming (coming soon)

### **Get Connected**
- **[Join Discord](https://discord.gg/autogence)** - Share your Harfy videos and get help
- **[GitHub Repositories](https://github.com/autogence)** - Contribute to open source projects

## Troubleshooting

### LED is solid red
- **Cause**: Hardware initialization failed
- **Fix**: Unplug power for 30 seconds, then reconnect
- **Still red?** Contact support@autogence.ai with your Harfy serial number

### Can't find Harfy-XXXX network
- **Cause**: Harfy may still be starting up
- **Fix**: Wait 2-3 minutes after powering on
- **Still missing?** Press and hold the chest button for 5 seconds to restart WiFi setup

### Harfy won't move during demos
- **Cause**: Surface may be too slippery or uneven
- **Fix**: Place Harfy on a flat, non-slip surface (carpet works great)
- **Still having issues?** Check that all joints moved freely during assembly

### Control interface won't load
- **Cause**: Device may not be on same WiFi network
- **Fix**: Ensure your phone/computer is connected to your home WiFi (not the Harfy-XXXX network)
- **Alternative**: Try using the IP address directly: `http://[IP_ADDRESS]`

## Need More Help?

- **Discord**: Get real-time help from the community
- **Email**: support@autogence.ai for technical issues
- **Full Manual**: Complete Harfy documentation (coming soon)
- **Video Guide**: Watch the setup process on our [YouTube channel](https://youtube.com/autogence)

---

**Congratulations!** You've successfully set up your Harfy robot. Welcome to the AutoGence community!