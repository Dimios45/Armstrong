```markdown
# 🤖 MuJoCo Simulation – SO100 6-DOF Robotic Arm

A complete forward and inverse kinematics simulation suite for the SO100 6-degree-of-freedom robotic arm using MuJoCo physics engine.

## 🎯 Overview

This project provides comprehensive kinematic analysis tools for the SO100 robot arm, featuring real-time MuJoCo visualization, joint limit enforcement, and optimization-based inverse kinematics solving. The simulation accurately models the physical constraints and dynamics of the robotic system.

## 📁 Project Structure

```
Mujoco_Simulation_so100/
├── forward_kinematics.py     # Forward kinematics solver with pose computation
├── inverse_kinematics.py     # Optimization-based IK solver with visualization
├── scene.xml                 # MuJoCo world environment configuration
├── so_arm100.xml            # SO100 robot model definition
└── assets/                  # 3D mesh files for robot components
    ├── Base.stl
    ├── Joint1.stl
    └── [additional STL files...]
```

## 🔧 Core Features

### Forward Kinematics (`forward_kinematics.py`)
- **Interactive Input**: Prompts for joint angles in degrees with automatic radian conversion
- **Pose Computation**: Calculates complete 4×4 transformation matrix for end-effector
- **Real-time Validation**: Enforces joint limits during input
- **MuJoCo Integration**: Leverages physics simulation for accurate pose calculation

### Inverse Kinematics (`inverse_kinematics.py`)
- **Multi-target Solving**: Processes multiple 3D target positions sequentially
- **Optimization Engine**: Uses scipy optimization with constraint handling
- **Joint Limit Enforcement**: Applies physical constraints during solving process
- **Live Visualization**: Renders each solved configuration in MuJoCo environment
- **Convergence Feedback**: Reports optimization success and final pose accuracy

## ⚙️ Installation & Setup

### Prerequisites
Ensure you have Python 3.8+ installed on your system.

### Standard Installation
```bash
pip install mujoco numpy scipy
```

### Using UV (Recommended)
```bash
uv venv
uv pip install mujoco numpy scipy
```

## 🚀 Usage Guide

### Forward Kinematics Simulation
```bash
uv run forward_kinematics.py
```

**Interactive Joint Configuration:**
```
Rotation:    [-110° to 110°]   # Base rotation
Pitch:       [-190° to 10°]    # Shoulder pitch
Elbow:       [-10° to 180°]    # Elbow flexion
Wrist_Pitch: [-95° to 95°]     # Wrist pitch adjustment
Wrist_Roll:  [-160° to 160°]   # Wrist roll rotation
Jaw:         [-10° to 100°]    # End-effector jaw opening
```

**Output**: Complete 4×4 homogeneous transformation matrix representing end-effector pose.

### Inverse Kinematics Simulation
```bash
uv run inverse_kinematics.py
```

**Sample Input Session:**
```
Enter target position 1 (x y z in meters): 0.2 0.4 0.3
Enter target position 2 (x y z in meters): 0.3 0.345 0.245
Enter target position 3 (x y z in meters): 0.25 0.3 0.25
```

**Process Flow:**
1. Solves IK for each target position
2. Applies joint constraints and limits
3. Visualizes each configuration in MuJoCo
4. Reports convergence status and pose accuracy

## 🎛️ Joint Specifications

| Joint | Range | Function |
|-------|-------|----------|
| **Rotation** | -110° to 110° | Base rotation around vertical axis |
| **Pitch** | -190° to 10° | Shoulder elevation/depression |
| **Elbow** | -10° to 180° | Elbow flexion/extension |
| **Wrist Pitch** | -95° to 95° | Wrist up/down articulation |
| **Wrist Roll** | -160° to 160° | Wrist rotation around forearm axis |
| **Jaw** | -10° to 100° | End-effector gripper opening |

## 🔮 Development Roadmap

### Phase 1: Enhanced Kinematics
- **Orientation Control**: Enhance the inverse kinematics system with orientation calculations and provide users with options when multiple joint angle solutions exist for reaching the same target position
- **Solution Multiplicity**: Implement algorithm to find and present all valid IK solutions for redundant configurations

### Phase 2: Performance Optimization
- **Jacobian-based IK**: Implement analytical Jacobian methods for faster convergence
- **Real-time Constraints**: Add dynamic obstacle avoidance and self-collision detection

### Phase 3: Advanced Features
- **Configuration Management**: Add logging and export capabilities for joint trajectories
- **Safety Systems**: Implement collision prevention to avoid robot self-intersection and ground contact
- **Path Planning**: Integrate trajectory optimization for smooth motion between waypoints

## 🛠️ Technical Details

- **Physics Engine**: MuJoCo for accurate dynamics simulation
- **Optimization**: Scipy's constrained optimization algorithms
- **File Formats**: XML model definitions with STL mesh assets
- **Coordinate System**: Right-handed coordinate system with Z-up convention

## 📋 Requirements

- **Python**: 3.8 or higher
- **MuJoCo**: Latest stable version
- **NumPy**: For numerical computations
- **SciPy**: For optimization algorithms

---

*This simulation suite provides a robust foundation for robotic arm research, education, and development with the SO100 platform.*
```
 

