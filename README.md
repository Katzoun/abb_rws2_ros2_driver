# Tvarometr - Automated Face Analysis and Robot Drawing System

An automated system for capturing, analyzing, and robotically rendering human faces at public events. Uses computer vision and neural networks to detect faces, predict age, gender, and emotion, then generates trajectory paths for ABB GoFa robot execution via RWS 2.0 interface.

## Overview

This ROS2 system integrates webcam capture, deep learning inference, and industrial robot control to create an interactive installation for events such as open days at Brno University of Technology, Faculty of Mechanical Engineering (BUT FME).

**Workflow:**
1. Webcam captures image of person
2. Neural networks analyze face attributes (age, gender, emotion)
3. System generates drawing trajectory based on analysis
4. Trajectory sent to ABB GoFa robot via RWS 2.0 protocol
5. Robot executes drawing

## System Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS2 Humble
- **Python**: 3.10+
- **Robot**: ABB GoFa with RWS 2.0 support
- **Hardware**: Webcam (USB or integrated)

## Architecture

### ROS2 Packages

#### `camera`
Handles image acquisition and neural network inference.
- **Nodes:**
  - `camera_node_exec`: Captures frames from webcam, publishes on `/input_image` topic
  - `inference_node_exec`: Runs deep learning models for face attribute prediction

- **Topics:**
  - `/start_inference` (std_msgs/String): Trigger for image capture
  - `/input_image` (sensor_msgs/Image): Raw camera frames
  - `/inference_result` (custom): Face analysis results (age, gender, emotion)

#### `master_pkg`
Orchestrates system state machine and robot communication.
- **Nodes:**
  - `master_node`: State machine controller, RWS client, trajectory generator

- **Features:**
  - State machine for process workflow
  - ABB Robot Web Services (RWS 2.0) client
  - Path generation and RAPID code generation
  - Trajectory optimization for robot execution

### Neural Network Models

Located in `src/camera/camera/AgeGenderEmotionPrediction/models/`:
- `yolov8x_person_face.pt`: Face detection (YOLOv8)
- `model_imdb_cross_person_4.22_99.46.pth.tar`: Age estimation
- `affectnet7_model.pth`: Emotion classification

Models stored via Git LFS due to file size constraints.

## Dependencies

### ROS2 Packages
```bash
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs
```

### Python Libraries
Core dependencies (install via pip):
```bash
pip install opencv-python torch torchvision numpy scipy
pip install requests ultralytics
```

**Note:** Complete dependency list not available. Additional libraries may be required for neural network inference components.

## Installation

1. Clone repository:
```bash
cd ~
git clone https://github.com/Katzoun/tvarometr_ws_SOTA.git
cd tvarometr_ws_SOTA
```

2. Install Git LFS and pull models:
```bash
git lfs install
git lfs pull
```

3. Build ROS2 workspace:
```bash
colcon build
source install/setup.bash
```

## Usage

### Launch System

**Simulation Mode:**
```bash
ros2 launch master_pkg tvarometr_sim.launch.py
```

**Full System:**
```bash
source install/setup.bash
ros2 launch master_pkg tvarometr_system.launch.py
```

### Manual Node Execution

Terminal 1 - Camera and inference:
```bash
ros2 run camera camera_node_exec
```

Terminal 2 - Inference node:
```bash
ros2 run camera inference_node_exec
```

Terminal 3 - Master controller:
```bash
ros2 run master_pkg master_node_exec
```
Terminal 4 - Keyboard interface to control the pipeline:
```bash
ros2 run master_pkg keyboard_publisher_exec
```


### Robot Configuration

Edit robot IP address in `src/master_pkg/master_pkg/master.py`:
```python
# Simulation
robotIP = "192.168.0.30"
robot_port = 80

# Physical robot
robotIP = "192.168.0.37"
robot_port = 443
```
## Configuration

### RWS Connection
Credentials configured in master node:
- Username: `Admin`
- Password: `robotics`

## Project Structure

```
tvarometr_ws_SOTA/
├── src/
│   ├── camera/                    # Vision and inference package
│   │   └── camera/
│   │       ├── camera_controller.py
│   │       └── AgeGenderEmotionPrediction/
│   │           ├── face_attributes_node.py
│   │           ├── predict.py
│   │           └── models/         # Neural network weights (Git LFS)
│   └── master_pkg/                # Control and coordination package
│       ├── launch/
│       │   └── tvarometr_system.launch.py
│       └── master_pkg/
│           ├── master.py
│           └── utils/
│               ├── state_machine.py
│               ├── rwsprovider.py
│               ├── rwswrappers.py
│               └── path_generator_multiline.py
├── build/                         # Colcon build artifacts
├── install/                       # Installation directory
└── log/                          # Build and runtime logs
```

## Technical Details

### State Machine
Master node implements finite state machine with states:
- IDLE: Awaiting trigger
- CAPTURE: Image acquisition
- INFERENCE: Neural network processing
- TRAJECTORY_GENERATION: Path planning
- ROBOT_EXECUTION: RAPID program upload and execution
- COMPLETE: Process finished

### Path Generation
Multi-line trajectory generator creates robot-executable paths based on face analysis results.

## Development

### Building
```bash
colcon build --packages-select camera master_pkg
source install/setup.bash
```
## Known Issues

- Complete Python dependency list not documented

## Institution

Developed at Brno University of Technology (BUT)  
Faculty of Mechanical Engineering (FME)  
Brno, Czech Republic
