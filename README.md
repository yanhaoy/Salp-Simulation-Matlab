# Salp Simulation

This repository contains MATLAB code for modeling, simulating, and analyzing the motion of a modular salp-inspired robot. The code supports symbolic modeling, ROS bag data extraction, motion reconstruction, and forward simulation.

## Features

- **Symbolic Model Construction:** Uses CasADi for symbolic kinematics and dynamics.
- **ROS Bag Data Processing:** Extracts and processes experimental data from ROS bag files.
- **Motion Reconstruction:** Compares predicted and experimental velocities and shape changes.
- **Forward Simulation:** Simulates the robot's motion under specified control inputs.
- **Visualization:** Plots translational, angular, and shape velocities for analysis.

## Directory Structure

```
.
├── main.m                      # Main entry point for configuration and running examples
├── salp_model.m                # Symbolic model construction
├── salp_bag.m                  # ROS bag data extraction and processing
├── salp_motion_reconstruction.m# Motion reconstruction and comparison
├── salp_forward_simulation.m   # Forward simulation and plotting
├── utils/                      # Utility functions (add to MATLAB path)
├── data/
│   ├── experiment/             # Folder for ROS bag files
│   ├── data_collection/        # Folder for ROS bag files
│   └── res.mat                 # Precomputed drag coefficients and control handles
```

## Requirements

- MATLAB R2020a or newer (recommended)
- [CasADi](https://web.casadi.org/) (symbolic computation library)
- Signal Processing Toolbox (for filtering)
- Robotics System Toolbox (for ROS bag support)

## Getting Started

1. **Clone the repository** and open it in MATLAB.

2. **Install CasADi** and add its path:
   ```matlab
   addpath('D:/casadi')
   ```

3. **Add the utils directory to your MATLAB path:**
   ```matlab
   addpath('./utils')
   ```

4. **Prepare your data:**
   - Place your ROS bag files in `./data/experiment/` and `./data/data_collection/`.
   - Ensure `res.mat` (with drag coefficients and control handles) is in `./data/`.

5. **Run the main script:**
   ```matlab
   main
   ```

   This will:
   - Build the symbolic model.
   - Run motion reconstruction (requires processed data).
   - Run a forward simulation.

## Typical Workflow

1. **Extract Data from ROS Bag:**
   ```matlab
   salp_bag(sys)
   ```
   This processes the ROS bag and saves a `.mat` file with experimental data.

2. **Reconstruct and Compare Motion:**
   ```matlab
   salp_motion_reconstruction(sys)
   ```
   Loads processed data and compares predicted vs. experimental velocities.

3. **Simulate Forward Dynamics:**
   ```matlab
   sys.control_handle = load('./data/res.mat').control;
   salp_forward_simulation(sys)
   ```

## Notes

- Adjust the configuration in main.m as needed for your hardware or experiment.
- The code assumes a 3-link salp robot; modify dimensions in main.m for other configurations.
- The code assumes LandSalp configuration with jet velocity control and viscous drag on wheels and joints; modify the drag coefficients in main.m and use q_dot_thrust_func in salp_model.m to implement a system with thrust control and viscous drag on links and joints.
- The IMU data is in its own frames, which are calculated in salp_model.m.
