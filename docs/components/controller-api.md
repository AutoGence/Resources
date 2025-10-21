---
id: controller-api
title: Harfy Controller APIs
sidebar_label: Controller APIs
sidebar_position: 20
description: Three interoperable API layers for controlling Harfy’s actuators—from low-level C++ drivers over EtherCAT/CAN, to ROS 2 topics, to a low-code Python/JavaScript bridge.
---

# Overview

Harfy exposes **three interoperable levels** of actuator-control APIs targeting different use cases and skill levels:

- **Level 1 — C++ Control Board (.so library)**  
  High-performance driver implementing **EtherCAT/CAN** to command motors directly. Distributed as compiled `.so` libraries with a C-style interface.

- **Level 2 — ROS 2 Topics (C++/Python packages)**  
  User-facing packages that publish/subscribe ROS 2 messages for control and status. Recommended for most developers.

- **Level 3 — Low-Code Bridge (Python/JavaScript)**  
  High-level SDK built on a ROS bridge to call simple functions from Python or JS with minimal setup.



---

## Units & Scaling

To maximize precision while keeping integer message fields:

- **Position** `rad` → value scaled by **1,000,000** (1e6)  
- **Velocity** `rad/s` → value scaled by **1,000,000** (1e6)  
- **Current** `A` → value scaled by **1,000,000** (1e6)  
- **Torque** `N·m` (unscaled unless otherwise stated)  
- **Temperature** `°C` (integer)  
- **Timestamps** `µs` (microseconds)

> In message payloads, position/velocity/current are multiplied by **1,000,000**; temperature is in °C; timestamps are in microseconds.

---

# Level 1 API — C++ Library for Raw Performance

**Purpose:** Real-time motor control via EtherCAT/CAN.  
**Status:** **Closed-source**, shipped as `.so` libraries and headers.

### 1.1 Initialization

```cpp
EcatapiRet EA_init(const std::string& config_file,
                   EcatapiStatusCallbackFuncPtr cb);
```

**Parameters**

- `config_file` (`std::string`): Absolute path to the configuration file.  
- `cb` (`EcatapiStatusCallbackFuncPtr`): Callback function to receive motor status frames.

### 1.2 Status Callback

```cpp
typedef void (*EcatapiStatusCallbackFuncPtr)(MotorSts* sts, uint8_t num);

typedef struct motor_sts {
  uint16_t slaveId;     // Slave ID
  uint16_t motorId;     // Motor ID
  int32_t  actualPos;   // Current position (rad * 1e6)
  int32_t  actualVel;   // Current velocity (rad/s * 1e6)
  int32_t  actualCur;   // Current current (A * 1e6)
  int16_t  temperature; // Temperature (°C)
  uint8_t  errcode;     // Error code (0 = OK)
} MotorSts;
```

**Callback Arguments**

- `sts`: Pointer to an array (queue) of motor status records.  
- `num`: Number of valid items in `sts`.

### 1.3 Motor Control

```cpp
EcatapiRet EA_ctrl(MotorCtrlPos* list, uint8_t num);

typedef enum {
  EMOTOR_SHUTDOWN = 0,
  EMOTOR_RUN      = 1
} MotorCtrlCmd;

typedef struct _motor_ctrl_pos {
  uint16_t    motorId;     // Motor ID
  int32_t     targetPos;   // Target position (rad * 1e6)
  int32_t     targetVel;   // Target velocity (rad/s * 1e6)
  int16_t     targetTorq;  // Target torque (N·m)
  MotorCtrlCmd cmd;        // Start/stop command
  int8_t      mode;        // Control mode
  uint64_t    seqId;       // Command sequence ID
  uint64_t    timeStamp;   // Timestamp (µs)
} MotorCtrlPos;
```

**Parameters**

- `list`: Array of control commands.  
- `num`: Number of valid commands.

:::note Control Modes
`mode` selects the low-level control policy (position/velocity/torque or hybrid). Valid values depend on firmware and will be documented per release.
:::

### 1.4 Examples

#### Example A — Minimal initialization with status logging

```cpp
#include <cstdio>
#include "ecatapi.h"  // Provided with the binary SDK

void status_cb(MotorSts* sts, uint8_t num) {
  for (uint8_t i = 0; i < num; ++i) {
    const auto& s = sts[i];
    printf("[slave=%u motor=%u] pos=%.6f rad vel=%.6f rad/s cur=%.6f A T=%d°C err=%u\n",
           s.slaveId, s.motorId,
           s.actualPos / 1e6, s.actualVel / 1e6, s.actualCur / 1e6,
           (int)s.temperature, (unsigned)s.errcode);
  }
}

int main() {
  const std::string cfg = "/opt/harfy/conf/ethercat.yaml";
  if (EA_init(cfg, status_cb) != ECATAPI_RET_OK) {
    fprintf(stderr, "EA_init failed\n");
    return 1;
  }
  // Spin your real-time loop here…
  // (driver provides its own thread for status_cb)
  while (true) { /* sleep or run control loop */ }
  return 0;
}
```

#### Example B — Send two position commands in one call

```cpp
#include <chrono>
#include <cstdint>
#include "ecatapi.h"

int main() {
  // ... assume EA_init(...) already called

  MotorCtrlPos cmds[2];
  uint64_t now_us = 1726500000000ULL; // replace with your monotonic clock

  cmds[0] = MotorCtrlPos{
      .motorId = 1,
      .targetPos = static_cast<int32_t>(1.570796 * 1e6), // 90 deg
      .targetVel = static_cast<int32_t>(0.5 * 1e6),      // 0.5 rad/s
      .targetTorq = 0,
      .cmd = EMOTOR_RUN,
      .mode = 1, // e.g., position mode
      .seqId = 1001,
      .timeStamp = now_us
  };

  cmds[1] = MotorCtrlPos{
      .motorId = 2,
      .targetPos = static_cast<int32_t>(0.785398 * 1e6), // 45 deg
      .targetVel = static_cast<int32_t>(0.3 * 1e6),
      .targetTorq = 0,
      .cmd = EMOTOR_RUN,
      .mode = 1,
      .seqId = 1002,
      .timeStamp = now_us
  };

  auto ret = EA_ctrl(cmds, 2);
  if (ret != ECATAPI_RET_OK) {
    // handle error
  }
  return 0;
}
```

---

# Level 2 API — ROS 2 Topics for Performance and Flexibilities

**Purpose:** Idiomatic ROS 2 control via topics.  
**Packages:** `robot_interfaces` (C++/Python).  
**Transport:** `rclcpp` / `rclpy`.

## 2.1 Command Topic — `/topic_motorctrl`

**Type:** `robot_interfaces/msg/SMotorCtrl`

```txt
robot_interfaces/msg/SMotorCtrl
  SMotorCtrlItem[] ctrl_list   # A list of motor control commands
```

```txt
robot_interfaces/msg/SMotorCtrlItem
  uint64 seq_id                # Sequence ID
  uint64 time_stamp            # Timestamp (µs)
  uint16 motor_id              # Motor ID
  EMotorCtrlCmd cmd            # Start/stop
  int8   mode                  # Control mode
  int32  target_pos            # Target position (rad * 1e6)
  int32  target_torq           # Target torque (N·m)
  int32  target_vel            # Target velocity (rad/s * 1e6)
```

```txt
robot_interfaces/msg/EMotorCtrlCmd
  uint16 E_MOTOR_SHUTDOWN = 0
  uint16 E_MOTOR_RUN      = 1
  uint16 value
```

**Field Notes**

- `motor_id`: Motor ID.  
- `target_pos`: Target position (rad, scaled by 1e6).  
- `target_vel`: Target velocity (rad/s, scaled by 1e6).  
- `target_torq`: Target torque (N·m).  
- `cmd`: Motor start/stop command (shutdown/run).  
- `mode`: Motor control mode (firmware-dependent).  
- `seq_id`: Command sequence ID.  
- `time_stamp`: Timestamp (microseconds).

**Example (CLI publish)**

```bash
# Example: set motor 3 to run at a target position & velocity
ros2 topic pub /topic_motorctrl robot_interfaces/SMotorCtrl "{
  ctrl_list: [{
    seq_id: 1001,
    time_stamp: 1726500000000,
    motor_id: 3,
    cmd: { value: 1 },
    mode: 1,
    target_pos: 1570796,   # ~1.570796 rad
    target_torq: 0,
    target_vel: 314159     # ~0.314159 rad/s
  }]
}"
```

### 2.1.1 Python publisher example (`rclpy`)

```python
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import SMotorCtrl, SMotorCtrlItem, EMotorCtrlCmd

class MotorPublisher(Node):
    def __init__(self):
        super().__init__('motor_pub')
        self.pub = self.create_publisher(SMotorCtrl, '/topic_motorctrl', 10)
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz
        self.seq = 0

    def tick(self):
        msg = SMotorCtrl()
        item = SMotorCtrlItem()
        item.seq_id = self.seq
        item.time_stamp = self.get_clock().now().nanoseconds // 1000  # µs
        item.motor_id = 3
        item.cmd = EMotorCtrlCmd(value=EMotorCtrlCmd.E_MOTOR_RUN)
        item.mode = 1
        item.target_pos = int(1.570796 * 1e6)
        item.target_torq = 0
        item.target_vel = int(0.3 * 1e6)
        msg.ctrl_list = [item]
        self.pub.publish(msg)
        self.seq += 1

def main():
    rclpy.init()
    node = MotorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.1.2 C++ publisher example (`rclcpp`)

```cpp
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/s_motor_ctrl.hpp"
#include "robot_interfaces/msg/s_motor_ctrl_item.hpp"
#include "robot_interfaces/msg/e_motor_ctrl_cmd.hpp"

using robot_interfaces::msg::SMotorCtrl;
using robot_interfaces::msg::SMotorCtrlItem;
using robot_interfaces::msg::EMotorCtrlCmd;

class MotorPub : public rclcpp::Node {
public:
  MotorPub() : Node("motor_pub"), seq_(0) {
    pub_ = this->create_publisher<SMotorCtrl>("/topic_motorctrl", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&MotorPub::tick, this));
  }

private:
  void tick() {
    SMotorCtrl msg;
    SMotorCtrlItem item;
    item.seq_id = seq_++;
    item.time_stamp = this->now().nanoseconds() / 1000; // µs
    item.motor_id = 3;
    EMotorCtrlCmd cmd;
    cmd.value = EMotorCtrlCmd::E_MOTOR_RUN;
    item.cmd = cmd;
    item.mode = 1;
    item.target_pos = static_cast<int32_t>(1.570796 * 1e6);
    item.target_torq = 0;
    item.target_vel = static_cast<int32_t>(0.3 * 1e6);
    msg.ctrl_list.push_back(item);
    pub_->publish(msg);
  }

  rclcpp::Publisher<SMotorCtrl>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t seq_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorPub>());
  rclcpp::shutdown();
  return 0;
}
```

## 2.2 Status Topic — `/topic_motorsts`

**Type:** `robot_interfaces/msg/CanMotorStatus`

```txt
robot_interfaces/msg/CanMotorStatus
  CanMotorStatusItem[] sts_list   # A list of motor runtime statuses
```

```txt
robot_interfaces/msg/CanMotorStatusItem
  uint16 slave_id                 # Slave ID
  uint16 motor_id                 # Motor ID
  int32  actual_pos               # Current position (rad * 1e6)
  int32  actual_vel               # Current velocity (rad/s * 1e6)
  int32  actual_cur               # Current current (A * 1e6)
  int16  temperature              # Temperature (°C)
  uint8  err_code                 # Error code (0 = OK)
```

**Example (subscribe)**

```bash
ros2 topic echo /topic_motorsts
```

### 2.2.1 Python subscriber example (`rclpy`)

```python
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import CanMotorStatus

class StatusSub(Node):
    def __init__(self):
        super().__init__('motor_status_sub')
        self.create_subscription(CanMotorStatus, '/topic_motorsts', self.cb, 10)

    def cb(self, msg: CanMotorStatus):
        for s in msg.sts_list:
            self.get_logger().info(
                f"[slave={s.slave_id} motor={s.motor_id}] pos={s.actual_pos/1e6:.6f} rad, "
                f"vel={s.actual_vel/1e6:.6f} rad/s, cur={s.actual_cur/1e6:.6f} A, "
                f"T={s.temperature}°C, err={s.err_code}"
            )

def main():
    rclpy.init()
    node = StatusSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

# Level 3 API — Low-Code API for Easy Access

**Purpose:** Minimal code to control Harfy from Python or JavaScript via `rosbridge`.

**Default Connection**

- **Host/IP:** Robot’s network address  
- **Port:** `9090` (default `rosbridge` port)

## 3.1 Python

### 3.1.1 Initialize

```python
from harfy import init

harfy = init(ip_address="192.168.1.50", port=9090)
# Returns a connected Harfy instance
```

**Parameters**

- `ip_address`: The robot’s network IP address.  
- `port`: The `rosbridge` port (default 9090).

### 3.1.2 `Harfy` Class (selected methods)

```python
# Position control: move a joint to a target angle (unit: rad)
harfy.move_motor_pos(joint_id: int, angle: float) -> None

# Torque control: apply a given joint torque (unit: N·m)
harfy.move_motor_force(joint_id: int, torque: float) -> None

# Read joint state (returns position/velocity/current/temperature/error, etc.)
state = harfy.motor_states(joint_id: int)
```

**Arguments**

- `joint_id`: Joint motor ID.  
- `angle`: Angle in radians.  
- `torque`: Torque in newton-meters.

**Return**

- `motor_states`: A motor state object containing position, velocity, current, temperature, error code, etc.

### 3.1.3 Python usage examples

#### A) Move joint to 30° and wait for settle

```python
import math, time
harfy = init("192.168.1.50", 9090)
harfy.move_motor_pos(joint_id=5, angle=math.radians(30))
time.sleep(2.0)  # simplistic wait; production code should check feedback
st = harfy.motor_states(5)
print(st)
```

#### B) Apply a light torque for 1 second

```python
import time
harfy = init("192.168.1.50", 9090)
harfy.move_motor_force(joint_id=5, torque=0.5)  # 0.5 N·m
time.sleep(1.0)
harfy.move_motor_force(joint_id=5, torque=0.0)
```

## 3.2 JavaScript (Node/Browser via rosbridge)

The **harfy-js** SDK exposes a minimal, ergonomic interface to control Harfy actuators over **rosbridge** (WebSocket) by publishing to `/topic_motorctrl` and subscribing to `/topic_motorsts`.

- Works in **Node.js** and **browsers**
- Position and torque control helpers
- Status subscription + convenient `motorStates()` getter
- Fixed-point helpers for converting `* 1e6` integer fields

> This SDK matches the Level 3 API described in the Actuator Control spec.

---

## Install

### npm
```bash
npm i harfy-js roslib
```

### pnpm
```bash
pnpm add harfy-js roslib
```

### yarn
```bash
yarn add harfy-js roslib
```

---

## Quick Start

```js
import { init } from 'harfy-js';

const harfy = await init({ ipAddress: '192.168.1.50', port: 9090 });

// Move joint 3 to 0.5 rad with a default velocity limit
await harfy.moveMotorPos({ jointId: 3, angleRad: 0.5 });

// Read the latest state (raw fixed-point ints per spec)
const st = await harfy.motorStates(3);
console.log(st);

// Apply 0.2 N·m torque for 500 ms, then release
await harfy.moveMotorForce({ jointId: 3, torqueNm: 0.2 });
await new Promise(r => setTimeout(r, 500));
await harfy.moveMotorForce({ jointId: 3, torqueNm: 0.0 });

// Shutdown the joint
harfy.shutdownMotor(3);

// Close when done
harfy.close();
```

---

## API Reference

### `init(opts) → Promise<Harfy>`

Connects to rosbridge and resolves to a ready `Harfy` instance.

**Options**

- `ipAddress` **string, required** — robot IP or hostname  
- `port` number = `9090` — rosbridge WebSocket port  
- `autoReconnect` boolean = `true` — auto reconnect on socket close  
- `reconnectDelayMs` number = `1000` — backoff between reconnects

### `class Harfy`

#### `harfy.moveMotorPos({ jointId, angleRad, velocityRadPerSec=0.3, torqueNm=0, mode=1, seqId?, timeStampUs? }) → void`

Publishes a position target for one joint.

- Sends `cmd={ value: 1 }` (E_MOTOR_RUN)  
- Scales `angleRad` and `velocityRadPerSec` by **1e6** per spec  
- `mode` is firmware-defined (commonly 1 for position)

#### `harfy.moveMotorForce({ jointId, torqueNm, mode=2, seqId?, timeStampUs? }) → void`

Publishes a torque command for one joint.

- `mode` commonly 2 for torque (check your firmware mapping)

#### `harfy.motorStates(jointId, { timeoutMs=1000 } = {}) → Promise<State>`

Returns the latest cached state for `jointId`, or waits for the next status frame up to `timeoutMs`.

**State (raw fields):**
```ts
{
  slave_id: number,
  motor_id: number,
  actual_pos: number, // rad * 1e6
  actual_vel: number, // rad/s * 1e6
  actual_cur: number, // A * 1e6
  temperature: number,// °C
  err_code: number    // 0 = OK
}
```

#### `harfy.publishMotorCtrl(items: SMotorCtrlItem[]) → void`

Publish a raw `SMotorCtrl` message containing `ctrl_list` of one or more items.

#### `harfy.shutdownMotor(jointId) → void`

Sends `E_MOTOR_SHUTDOWN` to a joint.

#### `harfy.close() → void`

Unsubscribes and closes the rosbridge connection.

### Helpers

```js
import { SCALE, toFixedInt, fromFixedInt, EMOTOR } from 'harfy-js';
```

- `SCALE` — `1e6`  
- `toFixedInt(x)` — multiply by `1e6` and round  
- `fromFixedInt(i)` — divide by `1e6`  
- `EMOTOR` — `{ SHUTDOWN: 0, RUN: 1 }`

---

## Examples

### Multi-joint command in a single publish

```js
import { init } from 'harfy-js';

const harfy = await init({ ipAddress: '192.168.1.50' });

const nowUs = Date.now() * 1000;
harfy.publishMotorCtrl([
  {
    seq_id: 1001,
    time_stamp: nowUs,
    motor_id: 1,
    cmd: { value: 1 },
    mode: 1,
    target_pos: Math.round(1.570796 * 1e6),
    target_torq: 0,
    target_vel: Math.round(0.3 * 1e6),
  },
  {
    seq_id: 1002,
    time_stamp: nowUs,
    motor_id: 2,
    cmd: { value: 1 },
    mode: 1,
    target_pos: Math.round(0.785398 * 1e6),
    target_torq: 0,
    target_vel: Math.round(0.3 * 1e6),
  },
]);
```

### Browser usage (Vite/Webpack)

```html
<script type="module">
  import { init } from '/node_modules/harfy-js/dist/harfy-js.js';
  const harfy = await init({ ipAddress: '192.168.1.50', port: 9090 });
  await harfy.moveMotorPos({ jointId: 1, angleRad: 0.3 });
</script>
```

### TypeScript tips

```ts
import { init, fromFixedInt } from 'harfy-js';
import type { HarfyState } from 'harfy-js';

const harfy = await init({ ipAddress: '192.168.1.50' });
const st: HarfyState = await harfy.motorStates(3);
const posRad = fromFixedInt(st.actual_pos);
```

---

## Troubleshooting

- **No connection?** Ensure `rosbridge_server` is running and that the WebSocket port (default **9090**) is reachable.  
- **No motion?** Confirm `cmd.value === 1` (RUN), control `mode` matches firmware, and safety interlocks are cleared.  
- **Weird units?** Remember position/velocity/current are scaled by **1e6**.  
- **Time correlation**: Use `seq_id` / `time_stamp` to match command and feedback in logs.

---

## Changelog

- `0.1.0` — Initial SDK exposing `init`, position/torque helpers, status cache, multi-joint publish, and TS declarations.


---

## Selecting the Right Level

- **Real-time, firmware/driver work, or EtherCAT/CAN integration?** Use **Level 1**.  
- **ROS-native development, simulation, orchestration with other ROS nodes?** Use **Level 2**.  
- **Fast prototyping, web/app integration, or scripting?** Use **Level 3**.

---

## Safety & Error Handling

- Always check `err_code` / `errcode` fields. `0` means **OK**.  
- Do not exceed joint limits or thermal limits. Monitor `temperature`.  
- Use `seq_id` and `time_stamp` to correlate commands and feedback in time-sensitive control loops.

---

## Troubleshooting Checklist

- No status updates? Verify `EA_init` callback registration (Level 1) or ROS graph connectivity (`ros2 node list`, `ros2 topic list`).  
- Commands not moving motors? Confirm `cmd` is `RUN`, the control `mode` matches the target fields, and safety interlocks are cleared.  
- Scaling confusion? Remember positions/velocities/currents are multiplied by **1e6** in messages.

---
