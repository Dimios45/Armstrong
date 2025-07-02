# Armstrong: SO100 Robotic Arm Simulation

A **MuJoCo-based simulation** of the SO100 6-DOF robotic arm with high-fidelity STL mesh rendering, gripper control, and realistic physics.

![MuJoCo](https://img.shields.io/badge/MuJoCo-3.3.3%2B-blue.svg)
![Python](https://img.shields.io/badge/Python-3.13%2B-green.svg)


---

## 🚀 Quick Start

### Prerequisites
- Python ≥ 3.13
- OpenGL-enabled graphics card
- MuJoCo ≥ 3.3.3

### Installation

**Option 1: Using venv**
```bash
python3 -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
pip install mujoco
````

**Option 2: Using uv (recommended)**

```bash
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
uv pip install mujoco
```

### Run Simulation

```bash
uv run main.py
```

---

## 🦾 Robot Specifications

| Feature       | Details                                                     |
| ------------- | ----------------------------------------------------------- |
| **DOF**       | 6 degrees of freedom                                        |
| **Joints**    | Base rotation, shoulder pitch, elbow, wrist pitch/roll, jaw |
| **Rendering** | High-quality STL mesh-based visuals                         |
| **Physics**   | Collision detection with mesh geometry                      |

## 🛠 Technical Details

* **Physics Engine**: MuJoCo with implicit integration
* **Rendering**: OpenGL with PBR materials
* **File Format**: MJCF (MuJoCo XML)
* **Mesh Format**: STL binary

## 📄 License

so100.xml, assets and scene.xml license as in LICENSE.