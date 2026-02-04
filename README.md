# Autonomous Mobile Robot Platform

This project implements a ROS 2-based autonomous mobile robot capable of
visual navigation using AprilTags, obstacle avoidance using ToF sensors,
and omnidirectional motion via an ESP32-based motor controller.


## System Architecture

```mermaid
graph LR
    Camera -->|USB| AprilTagNode
    AprilTagNode -->|cmd_vel| CmdVelBridge
    CmdVelBridge -->|Serial| ESP32
    ESP32 --> Motors
    ESP32 --> Encoders
    ESP32 --> IMU
    ESP32 --> ToF


---

### 3️⃣ Hardware Overview
Describe:
- ESP32 role
- Jetson / PC role
- Sensors (camera, ToF, IMU, encoders)

This is *gold* for reviewers and future-you.

---

### 4️ ROS 2 Nodes
Document **each node**:

```md
## ROS 2 Nodes

### `apriltag_follower`
- Subscribes to: camera stream
- Publishes: `/cmd_vel`
- Function: Detects and tracks AprilTags using a USB camera

### `cmdvel_serial_bridge`
- Subscribes to: `/cmd_vel`
- Publishes: serial commands to ESP32
- Function: Converts ROS velocity commands to motor control signals

## Navigation State Machine

```mermaid
stateDiagram-v2
    [*] --> SEARCH
    SEARCH --> TRACK : Tag detected
    TRACK --> APPROACH : Centered
    APPROACH --> STOP : Target distance reached
    TRACK --> SEARCH : Tag lost


---

###  How to Build & Run
This is mandatory and professional:

```md
## Build

```bash
colcon build --symlink-install
source install/setup.bash


ros2 run autonomous_robot cmdvel_serial_bridge
ros2 run apriltag_follower tag_follower


---

### 7️ Future Work / Roadmap
This shows planning and maturity:

```md
## Future Work
- Integrate front-facing ToF for obstacle avoidance
- Add multi-tag navigation
- Implement full state machine with recovery behaviors
- 


