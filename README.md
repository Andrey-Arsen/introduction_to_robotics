
---

## RoArm-M2-S ROS 2 Driver

This project contains ROS 2 nodes for controlling the **RoArm-M2-S** robotic arm over UART. The nodes allow you to:

- Enable/disable motor torque
- Read real-time joint angles from the arm
- Control the arm by sending target joint angles

---

### Project Structure

```
ros2_ws/
├── src/
│   ├── roarm_driver/           # Node 1: reads joint angles and publishes to ROS 2
│   │   └── roarm_driver.py
│   └── roarm_driver2/          # Node 2: subscribes and sends joint commands
│       └── roarm_driver2.py
├── install/
├── build/
├── ...
```

---

###  Dependencies

- ROS 2 Humble or later
- Python 3
- ROS 2 packages:
  - `rclpy`
  - `std_msgs`
  - `sensor_msgs`
  - `geometry_msgs`
- UART-connected microcontroller (e.g., via `/dev/ttyUSB0`)

---

###  Installation

1. Clone the repository:
```bash
git clone https://github.com/your-username/roarm_ws.git
cd roarm_ws
```

2. Build the workspace:
```bash
colcon build
source install/setup.bash
```

3. Make sure the robotic arm is connected (e.g., to `/dev/ttyUSB0`).

---

### ▶️ How to Run

#### 1. Start the feedback node

This node reads motor joint angles and publishes them to the `/joint_feedback` topic:

```bash
ros2 run roarm_driver roarm_driver
```

#### 2. Start the command node

This node subscribes to `/joint_feedback` and sends motion commands based on received joint angles:

```bash
ros2 run roarm_driver2 roarm_driver2
```

#### 3. Enable or disable motor torque

Example: turn **torque OFF** (to enable free movement):

```bash
ros2 topic pub --once /torque_ctrl std_msgs/Bool "{data: false}"
```

To turn it back ON:

```bash
ros2 topic pub --once /torque_ctrl std_msgs/Bool "{data: true}"
```

---

### 📡 ROS 2 Topics

- `/joint_feedback` (`sensor_msgs/JointState`) — publishes joint angles
- `/torque_ctrl` (`std_msgs/Bool`) — controls motor torque state

---

### ⚠️ Notes

- Check that `/dev/ttyUSB0` is the correct port and accessible
- Default baud rate is `115200`, must match microcontroller configuration
- Uses JSON-based UART communication with the robot

---

