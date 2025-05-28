# 🤚 ros2-Hand-Gesture-Controller

ROS2 TurtleSim controller using real-time hand gestures with **MediaPipe** and **OpenCV**. Control movement via webcam-detected gestures mapped to ROS2 velocity commands.

![Gesture Detection](./Screenshot%20from%202025-05-27%2022-20-31%20(copy).png)
![TurtleSim Movement](./Screenshot%20from%202025-05-27%2022-20-31.png)

---

## 🚀 Features

- Hand gesture recognition using MediaPipe
- Real-time webcam input via OpenCV
- Publishes to `/turtle1/cmd_vel` to move the turtle
- Supports gestures for:
  -  Move Forward
  -  Move Backward
  -  Turn Left
  -  Turn Right

---

## 🛠️ Prerequisites

Ensure you have:

- Ubuntu 20.04 or 22.04
- [ROS2](https://docs.ros.org/) (Foxy, Humble, etc.)
- Python 3.8+

Install the required Python packages:

```bash
pip install mediapipe opencv-python
```

---

## 📦 Setup Instructions

1. Navigate to your ROS2 workspace:

```bash
cd ~/ros2_ws/src
```

2. Clone this repository:

```bash
git clone https://github.com/YOUR_USERNAME/ros2-Hand-Gesture-Controller.git
```

3. Build the workspace:

```bash
cd ~/ros2_ws
colcon build
```

4. Source the workspace:

```bash
source install/setup.bash
```

5. Run TurtleSim:

```bash
ros2 run turtlesim turtlesim_node
```

6. Run the gesture controller node:

```bash
ros2 run ros2_hand_gesture_controller turtle_gesture_controller
```

---

## 🎮 Gesture Mapping

| Finger near Thumb | Action         |
|-------------------|----------------|
| Index Touch Thumb            | Move Forward   |
| Middle Touch Thumb           | Turn Left      |
| Ring Touch Thumb             | Turn Right     |
| Pinky Touch Thumb            | Move Backward  |

