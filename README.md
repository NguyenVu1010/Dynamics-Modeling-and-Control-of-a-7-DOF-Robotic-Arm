# 🤖 Dynamics Modeling and Control of a 7-DOF Robotic Arm

This project provides the dynamics modeling and control implementation for a 7-degree-of-freedom (7-DOF) robotic manipulator using Python, symbolic computation, and simulation tools.

---

## 📌 Project Overview

The main goals of this project include:

- Modeling the forward and inverse kinematics of a 7-DOF arm
- Deriving the dynamic equations using the Lagrangian method
- Simulating the arm's behavior under different control laws (PD, computed torque, etc.)
- Building a modular structure for future integration with ROS/Gazebo or hardware

---

## 📁 Project Structure

```bash
.
├── dynamics/             # Symbolic modeling of dynamics (e.g., Lagrange equations)
├── kinematics/           # FK / IK scripts
├── control/              # Control algorithms: PD, computed torque, etc.
├── simulation/           # Simulations using numerical integrators
├── utils/                # Common helper functions
├── urdf/    # URDF, DH parameters, etc.
├── plots/                # Saved plots and figures
├── main.py               # Entry point to run simulation
└── README.md             # Project documentation
```
## ⚙️ Dependencies
### Make sure you have Python 3.8+ and the following packages:
``` Bash
pip install -r requirements.txt
```

## 📐 Robot Parameters
7 revolute joints
Modified Denavit-Hartenberg convention

## 📊 Output
Joint angles over time
Trajectory tracking plots
Torque input over time
Energy analysis (optional)


