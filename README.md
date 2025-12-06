# MPC-based IBVS with Ruckig and Fuzzy Velocity Controller

A Model Predictive Control (MPC) based Image-Based Visual Servoing (IBVS) system integrated with Ruckig trajectory generation and fuzzy logic velocity controller for robotic motion control.

## Project Overview

This project implements a complete visual servoing control system with the following features:

- **MPC Controller**: For predictive control optimization
- **IBVS Core**: Image-based visual servoing algorithms
- **Ruckig Integration**: Smooth trajectory generation and motion planning
- **Fuzzy Logic Control**: Intelligent velocity regulation
- **JR603 Robot Model**: Six-degree-of-freedom industrial robot simulation

## Project Structure

```
MPC_IBVS_with_Ruckig_and_Fuzzy_Velocity_Controller/
│
├── Main.m                          # Main program entry point
├── LICENSE                         # License file
│
├── fuzzy_velocity_controller/      # Fuzzy Velocity Controller
│   └── fuzzy_velocity_controller_V3.m
│
├── ibvs_core/                      # IBVS Core Algorithms
│   ├── Calculate_e.m              # Error calculation
│   ├── Calculate_Ls.m             # Interaction matrix computation
│   ├── Calculate_s.m              # Feature calculation
│   ├── Camera_3to2.m              # 3D to 2D camera projection
│   ├── jacob_cross_sdh.m          # Jacobian matrix calculation
│   ├── jr603_fkine.m              # JR603 forward kinematics
│   ├── jr603_ivkine.m             # JR603 inverse kinematics
│   └── jr603_sdh.m                # JR603 standard D-H parameters
│
├── mpc_controller/                 # MPC Controller Module
│   ├── build_L_library.m          # Build L matrix library
│   ├── combine_constraints.m      # Constraint combination
│   ├── compute_weights.m          # Weight computation
│   ├── cost_function.m            # Cost function
│   ├── cost_function_LPV.m        # LPV cost function
│   ├── fov_constrains.m           # Field of view constraints
│   ├── get_fov_constrains.m       # Get FOV constraints
│   ├── linearize_model.m          # Model linearization
│   ├── mpc_controller.m           # Main MPC controller function
│   ├── mpc_controller_qp.m        # MPC controller QP 
│   └── velocity_constraints.m     # Velocity constraints
│
├── rucking_mex/                   # Ruckig Trajectory Generator
│   ├── libruckig.dll              # Ruckig dynamic link library
│   ├── libruckig_mex.dll.a        # MEX link library
│   └── ruckig_mex.mexw64          # Ruckig MEX file
│
└── test/                          # Test scripts
    ├── test_tu.m
    ├── test_tu2.m
    ├── test_tu3.m
    ├── test_tu4.m
    └── test_workspace.m
```

## System Requirements

- MATLAB R2020b or higher
- Control System Toolbox
- Optimization Toolbox (for QP solving)
- Fuzzy Logic Toolbox
- Windows operating system (compatible with Ruckig MEX files)

## Installation and Running

1. Clone or download the project to your local machine

2. Set the current folder in MATLAB to the project root directory

3.  Run the main program:

   ```
   Main
   ```

## Main Functional Modules

### 1. IBVS Core Module (`ibvs_core/`)

- Image feature extraction and error calculation
- Interaction matrix computation
- Camera projection model
- JR603 robot kinematics

### 2. MPC Controller (`mpc_controller/`)

- Model predictive controller design
- Constraint handling (FOV, velocity, etc.)
- Optimization problem solving
- Linear Parameter Varying (LPV) model support

### 3. Fuzzy Velocity Controller (`fuzzy_velocity_controller/`)

- Fuzzy logic-based velocity regulation
- Adaptive control strategy
- Smooth velocity planning

### 4. Ruckig Integration (`rucking_mex/`)

- Real-time trajectory generation
- Smooth motion planning
- Dynamic constraint handling

## Usage Examples

### Basic IBVS Control

```
% Call IBVS core functions in main program
error = Calculate_e(current_features, desired_features);
Ls = Calculate_Ls(camera_params, features);
```

### MPC Controller Configuration

```
mpc_controller_qp(s,s_target,z_star,Np,Q,R_weight,W_weight,Ts,K,F,image_width,image_height,tau_now,A,B,v_max,w_max);
mpc_controller(s,s_target,z_star,Np,Q,R_weight,W_weight,Ts,K,F,q_current,image_width,image_height,tau_now,fig_handle,vis_predict,A,B);
```

### Fuzzy Velocity Control

```
% Use fuzzy controller for velocity adjustment
[max_acc, max_jerk, fis] = fuzzy_velocity_controller_V3(e, de, a_rated)
```

## Testing and Validation

The project includes multiple test scripts:

- `test_tu.m`~ `test_tu4.m`: Unit tests for different scenarios
- `test_workspace.m`: Workspace validation test

Run test scripts to verify system functionality:

```bash
test_tu1    % Basic functionality test
test_workspace  % Workspace analysis
```

## Dependencies

- **Ruckig Library**: For trajectory generation (MEX files included)
- **MATLAB Toolboxes**: Control System Toolbox Optimization Toolbox Fuzzy Logic Toolbox

## License

This project is open source under the MIT License. See LICENSE file for details.

## Contributing

Issues and Pull Requests are welcome to improve this project.

## Contact

For questions or suggestions, please contact via the project Issues page.

------

## Development Notes

### File Naming Convention

- Function files use lowercase letters and underscores
- Main files use capitalized first letters
- Test files start with `test_`prefix

### Code Structure

- Modular design for easy maintenance and extension
- Clear function interfaces and documentation
- Comprehensive test cases included

### Extension Suggestions

- Add support for more robot models
- Extend visual feature types
- Optimize real-time performance
- Add visualization tools

```bash
Now let me help you commit to the remote repository:
```

bash

# Add all files to staging area

git add .

# Commit changes

git commit -m "Add comprehensive README.md documentation with project structure, usage guide, and system documentation"

# Push to remote repository

git push origin main

```bash
If you need to modify the remote repository name (e.g., from main to master), please adjust the last command accordingly.

This README.md provides:
1. Clear project overview
2. Complete directory structure explanation
3. Installation and running guide
4. Detailed functional module descriptions
5. Usage examples and testing instructions
6. Dependency and license information

This documentation will help other developers quickly understand and use your project.
```
