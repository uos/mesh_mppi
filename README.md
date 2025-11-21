# Introduction

<div align="center" min-width=519px>
  <img src="docs/mesh_mppi_floor_is_lava.gif" alt="MeshMPPI"/>
</div>

**MeshMPPI** is an adaptation of the *model predictive path integral* (MPPI) control algorithm to surface meshes.
The MPPI algorithm generates control signals by simulating the trajectories resulting from a set of random samples.
This adaptation constraints the trajectory prediction to the surface defined by a triangular surface mesh.
The implementation provided in this repository implements the MeshMPPI algorithm for the ROS2-based [MeshNav](https://github.com/naturerobots/mesh_navigation) 3D navigation stack.

# Installation

If you have an existing [MeshNav](https://github.com/naturerobots/mesh_navigation) installation simply clone this repository and rebuild your workspace.
Otherwise start by creating a new colcon (ROS2) workspace:
```bash
mkdir -p mesh_nav_ws/src
cd mesh_nav_ws
```
Next clone the MeshNav repository into the `src` directory of your workspace:
```bash
cd src
git clone https://github.com/naturerobots/mesh_navigation.git
```
Then clone all source dependencies required by MeshNav:
```bash
vcs import --input mesh_navigation/source_dependencies.yaml
```
And all non-source dependencies:
```bash
rosdep install --from-paths . --ignore-src -r -y
```
We additionally recommend to install the [`embree4`](https://github.com/RenderKit/embree) library since MeshNav makes use of their optimized ray tracing kernels if they are available.
For Ubuntu 24 LTS you may install embree with the following command:
```bash
sudo apt install libembree-dev
```
Now clone this repository into the workspace's source directory:
```bash
git clone https://github.com/uos/mesh_mppi.git
```
Finally, go back to the workspace directory and build it:
```bash
cd ..
colcon build --cmake-args " -DCMAKE_BUILD_TYPE=Release"
```

<details>
<summary>Compilation on Ubuntu 22 and ROS2 Humble</summary>

This package requires the C++23 standard!
The default gcc version shipped with Ubuntu 22 is gcc-11, which supports some C++23 features but not all, notably `<format>` and `<expected>` are not available.
To get around this limitation you have to install a newer gcc version.
For this you need to add the `ubuntu-toolchain-r/test` repository.
```bash
sudo apt install software-properties-common
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
```
And then install a gcc version greater or equal to 13 and configure your system to use the new compiler.
```bash
sudo apt install gcc-13 g++-13
# Set gcc 13 as the system default
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 13 --slave /usr/bin/g++ g++ /usr/bin/g++-13
```
</details>

# Usage

The `mesh_mppi` package provides implementations of the *Differential Drive* and *Bicycle* kinematic models.
Each kinematic model is compiled into a separate MeshNav controller plugin to enable compile time optimizations.
If you want or need to use a different kinematic model see [this section below](#development).

## Configuration
### Differential Drive

The *Differential Drive* model uses two control signals:

| Control Signal Index | Semantic         | Unit  |
|----------------------|------------------|-------|
| 0                    | Linear Velocity  | m/s   |
| 1                    | Angular Velocity | rad/s |

The following is an example controller configuration for MeshNav using the *Differential Drive* kinematic model:
```yaml
mesh_controller:
  # The type parameter tells MeshNav which plugin to load as the controller
  type: 'mesh_mppi/DiffDriveMPC'
  # If the MeshMap's cost at the position of the robot is at or above this value the controller considers this as a collision.
  map_cost_limit: 1.0

  # Parameters for the kinematic model
  kinematics:
    # Velocity limits
    linear_velocity_max: 1.0
    linear_velocity_min: -0.25
    angular_velocity_max: 1.0
    # Acceleration limits
    linear_acceleration_max: 3.0
    linear_acceleration_min: -3.0
    angular_acceleration_max: 9.0

  # Parameters of the MPPI optimizer
  optimizer:
    # The number of time steps to predict the robots trajectory into the future
    prediction_steps: 56
    # The number of trajectories to predict per controller iteration. The frequency is determined by move_base_flex's 'controller_frequency' parameter.
    prediction_samples: 1000
    # The standard deviations used to sample the control signals. The number of control signals depends on the kinematic model.
    stddev:
      # Linear Velocity standard deviation in m/s
      - 0.2
      # Angular Velocity standard deviation in rad/s
      - 0.4
    # The temperature of the controller
    temperature: 2.0
    # The control cost of the controller
    gamma: 0.015

  # Cost function related parameters
  cost:
    # Cross-Track-Error punishes deviation from the reference path
    xte_weight: 5.0
    # Map costs punishes higher costs in the MeshMap's layered costmap
    map_weight: 5.0
    # Goal costs incentivises the controller to make progress towards the goal
    goal_weight: 10.0
```

### Bicycle

The *Bicycle* model uses two control signals:

| Control Signal Index | Semantic        | Unit |
|----------------------|-----------------|------|
| 0                    | Linear Velocity | m/s  |
| 1                    | Steering Angle  | rad  |

This kinematics assume that the mobile base has a low level controller to control the steering joint(s).
We model the low level controller in the state prediction of the kinematics using a configurable proportional gain.
This is how gazebo's [`AckermannSteering`](https://gazebosim.org/api/sim/10/classgz_1_1sim_1_1systems_1_1AckermannSteering.html) system works.
We found that this approach also works well in our real world testing, but it might require some tuning of the gain and velocity parameters.

The following is an example controller configuration for MeshNav using the *Bicycle* kinematic model:
```yaml
bicycle_controller:
  # The type parameter tells MeshNav which plugin to load as the controller
  type: 'mesh_mppi/BicycleMPC'
  # If the MeshMap's cost at the position of the robot is at or above this value the controller considers this as a collision.
  map_cost_limit: 1.0

  # Parameters for the kinematic model
  kinematics:
    # Velocity limits
    linear_velocity_max: 1.0
    linear_velocity_min: -0.75
    # Acceleration limits
    linear_acceleration_max: 3.0
    linear_acceleration_min: -3.0
    # The maximum steering angle for left and right steering
    steering_angle_max: 0.6
    # Distance between the front and rear axle in meter
    wheelbase: 0.55
    # Distance from the base link frame to the rear axle
    rear_axle_distance: 0.20
    # The maximum velocity the steering joint can change with (rad/s)
    steering_joint_velocity_max: 1.6
    # This plugin models the steering behaviour of the base using
    # proportinal velocity control with this gain:
    steering_joint_velocity_gain: 1.0
    # Steering joints (used to read the current steering angle)
    left_steer_joint: "fr_steer_left_joint"
    right_steer_joint: "fr_steer_right_joint"

  # Parameters of the MPPI optimizer
  optimizer:
    # The number of time steps to predict the robots trajectory into the future
    prediction_steps: 56
    # The number of trajectories to predict per controller iteration. The frequency is determined by move_base_flex's 'controller_frequency' parameter.
    prediction_samples: 1000
    # The standard deviations used to sample the control signals. The number of control signals depends on the kinematic model.
    stddev:
      # Linear Velocity standard deviation in m/s
      - 0.2
      # Steering Angle standard deviation in rad
      - 0.2
    # The temperature of the controller
    temperature: 2.0
    # The control cost of the controller
    gamma: 0.015

  # Cost function related parameters
  cost:
    # Cross-Track-Error punishes deviation from the reference path
    xte_weight: 5.0
    # Map costs punishes higher costs in the MeshMap's layered costmap
    map_weight: 5.0
    # Goal costs incentivises the controller to make progress towards the goal
    goal_weight: 10.0
```

# Development

TODO: Explain how to create a controller with custom kinematics
