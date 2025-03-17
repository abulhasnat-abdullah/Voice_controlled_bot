# Voice-Controlled TurtleBot3

## 🌟 Overview
Voice-Controlled TurtleBot3 is an exciting ROS2-based project that allows you to control a TurtleBot3 robot within a Gazebo simulation environment using simple voice commands. The robot responds dynamically to commands for movement and energy management, offering an engaging and hands-free robotics experience.

## 🚀 Key Features
- **🎙️ Voice-Controlled Navigation**: Direct the robot effortlessly with voice commands such as "forward", "left", and "right" to navigate the simulation.
- **🔋 Energy Management**: Manage the robot's energy by using voice triggers like "heal" to recharge or "kaboom" to initiate energy actions.
- **📡 Real-time ROS2 Communication**: Built using `rclpy`, the system processes voice commands and sends movement instructions to the `/cmd_vel` topic in real-time.
- **🐳 Dockerized Setup**: The entire system is containerized, making it simple to deploy, build, and run inside a Docker container for a smooth and consistent experience across different environments.

## 📁 Repository Structure
```
📂 voice_controlled_bot
 ├── 📂 src/                     # ROS2 package source code
 ├── 📄 Dockerfile               # Dockerfile for containerization
 ├── 📄 requirements.txt         # Python dependencies
 ├── 📄 README.md                # Project documentation
 ├── 📄 setup.py                 # ROS2 package setup
```

## 🔧 Installation
### **1️⃣ Clone the repository**
```bash
git clone https://github.com/abulhasnat-abdullah/voice_controlled_bot.git
cd voice_controlled_bot
```
### **2️⃣ Build the ROS2 Package**
```bash
colcon build
source install/setup.bash
```

## 🐳 Running with Docker

Dockerhub : 

### **1️⃣ Pull the Docker Image**
```bash
docker pull abulhasnatabdullah/voice_controlled_robot
```
### **2️⃣ Run the Container**
```bash
docker run --device /dev/snd:/dev/snd -it abulhasnatabdullah/voice_controlled_robot
```

## 🎤 Voice Commands
| Command      | Action |
|-------------|--------|
| "forward"    | Moves forward for 2 seconds |
| "forward 5s" | Moves forward for 5 seconds |
| "left"       | Rotates left |
| "right"      | Rotates right |
| "heal" / "kaboom" | Increases energy |

## 🤝 Contributing
Pull requests are welcome! Feel free to open an issue if you have any questions or suggestions.

## 📜 License
This project is licensed under the MIT License. See `LICENSE` for details.
