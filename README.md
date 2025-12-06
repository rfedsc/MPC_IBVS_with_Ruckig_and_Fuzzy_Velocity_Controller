<<<<<<< HEAD
<<<<<<< HEAD
⭐ MPC_IBVS_with_Ruckig_and_Fuzzy_Velocity_Controller_Final

A Real-Time IBVS Control Framework Integrating MPC, Ruckig, and Fuzzy Velocity Regulation

📌 Overview

This repository provides a complete implementation of an advanced Image-Based Visual Servoing (IBVS) control system that integrates:

Model Predictive Control (MPC) for constrained and optimal motion control

Ruckig real-time trajectory generation for smooth and dynamically feasible motion

Fuzzy Velocity Control for adaptive and robust speed regulation

The proposed framework achieves smooth, real-time, robust, and constraint-aware camera/robot motion for embodied intelligent systems.

🚀 Features
🔹 Image-Based Visual Servoing (IBVS)

Supports multi-feature visual servoing (centroid, scale, orientation, and point features)

Analytical derivation of image Jacobians

Nonlinear visual servoing with stable convergence

🔹 Model Predictive Control (MPC)

Handles field-of-view (FOV) constraints

Handles velocity and actuation constraints

Converts nonlinear IBVS model into LPV/QP form using online linearization

🔹 Ruckig Trajectory Generator

Guarantees jerk-limited, acceleration-limited, and velocity-limited motion

Ensures smooth transitions without jumps

Strong real-time computation performance

🔹 Fuzzy Velocity Controller

Mamdani fuzzy inference

Enables adaptive velocity modulation (slow down near target)

Improves tracking accuracy and robustness
=======
# 🚀 MPC-IBVS with Ruckig and Fuzzy Velocity Controller

### **A Real-Time Image-Based Visual Servoing Framework with MPC, Ruckig Trajectory Generation, and Fuzzy Logic Speed Regulation**

------

## 📌 Overview

This project implements a comprehensive **Image-Based Visual Servoing (IBVS)** system enhanced by:

- **Model Predictive Control (MPC)**
- **Real-time trajectory smoothing using Ruckig**
- **Adaptive fuzzy velocity regulation**

The framework enables **smooth, constraint-aware, and robust** robot motion for embodied intelligence applications.

------

## 📂 Project Structure

```
core_ibvs/                 # IBVS core functions (error, feature extraction, Jacobians)
mpc/                       # MPC controllers, QP matrices, linearization
constraints/               # FOV, velocity, and adaptive gain constraints
fuzzy_controller/          # Mamdani fuzzy logic velocity controller (V1, V2, V3)
trajectory/                # Polynomial and S-curve trajectory generation
robot_kinematics/          # Forward/Inverse kinematics & Jacobians
tests/                     # Test scripts and validation tools
main/                      # Project entry scripts
media/                     # Simulation videos or demo media
```

Each folder is modular and can be used independently or integrated together in the main IBVS control loop.

------

## ✨ Key Features

### 🔹 1. Image-Based Visual Servoing (IBVS)

- Multi-feature visual servoing
- Centroid, orientation, scale, and point features
- Analytical Jacobian matrix computation
- Stable closed-loop visual control

### 🔹 2. Model Predictive Control (MPC)

- QP-based constrained optimization
- LPV model online linearization
- Visual field-of-view (FOV) constraints
- Velocity and control input limits
- Tunable MPC cost structure

### 🔹 3. Ruckig Trajectory Generation

- Jerk-limited trajectory profiles
- Smooth velocity and acceleration curves
- Real-time feasible motion planning
- Avoids actuator shocks

### 🔹 4. Fuzzy Velocity Controller

- Mamdani fuzzy inference engine
- Multiple versions: V1, V2, V3
- Adaptive speed reduction near the target
- Improved robustness under disturbances

### 🔹 5. Robot Kinematics Support

- PUMA robot kinematics (FK/IK)
- JR603 robot kinematics
- Cross-product Jacobians
- Camera 3D-to-2D projection tools

------

## 🧠 System Pipeline

```
Camera Image → Feature Extraction → IBVS Error
        → MPC Optimization (QP)
        → Ruckig Trajectory Smoothing
        → Fuzzy Velocity Controller
        → Robot Motion Commands
```

This modular architecture ensures stable, real-time, and constraint-aware robot motion.

------

## ▶ How to Use

### **Run the main IBVS + MPC + Fuzzy system**

```
main/Main.m
```

### **Test a specific module**

Fuzzy controller:

```
tests/test_fuzzy_velocity_controller.m
```

MPC QP constraints:

```
tests/test_fx.m
```

Trajectory generation:

```
trajectory/quintic_polynomial_traj.m
```

------

## 📦 Requirements

- MATLAB
- Optimization Toolbox
- Fuzzy Logic Toolbox
- Robotics Toolbox (optional)

------

## 🎥 Demo

A demonstration video is provided:

```
media/robot_motion.avi
```

------

## 🤝 Contributing

Contributions and improvements are welcome!
 Feel free to open an issue or submit a pull request.

------

## 📜 License

MIT License
>>>>>>> e90e3d0 (update README)
=======
# MPC_IBVS_with_Ruckig_and_Fuzzy_Velocity_Controller
An implementation of Image-Based Visual Servo (IBVS) integrated with Model Predictive Control (MPC), Ruckig trajectory generator, and fuzzy logic-based velocity controller for robot motion control.
>>>>>>> 5253eda9fba834231bacfac022203659a7b03dc1
