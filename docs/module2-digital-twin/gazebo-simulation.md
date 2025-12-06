# Gazebo Simulation for Humanoid Robots

## Overview

Gazebo is a powerful open-source 3D simulation environment that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. For humanoid robotics, Gazebo offers an ideal platform to test control algorithms, sensor integration, and robot behaviors before deploying to physical hardware.

## Installing and Setting up Gazebo

### Prerequisites
- ROS 2 (Humble Hawksbill or Iron Irwini)
- Ubuntu 22.04 LTS or compatible system
- Sufficient computational resources (multi-core CPU, dedicated GPU recommended)

### Installation
```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

### Verification
Launch Gazebo to ensure proper installation:
```bash
gz sim
```

## Integrating with ROS 2

Gazebo integrates seamlessly with ROS 2 through the `gazebo_ros_pkgs` package, which provides:

- Bridge between Gazebo simulation and ROS 2 topics
- ROS 2 interfaces for controlling simulated robots
- Sensor data publishing to ROS 2 topics
- Robot state publishing

## Creating a Humanoid Robot Model

### Using URDF with Gazebo
Gazebo can directly use URDF files with additional Gazebo-specific tags for simulation properties:

```xml
<gazebo reference="joint_name">
  <mu1>0.9</mu1>
  <mu2>0.9</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
</gazebo>
```

### Physics Properties
For humanoid robots, it's important to tune physics properties:
- Collision shapes for accurate contact detection
- Inertial properties matching real robot
- Joint friction and damping parameters
- Contact compliance for stable simulation

## Gazebo Plugins for Humanoid Robots

### Joint Control Plugins
Gazebo provides several plugins for controlling robot joints:
- `libgazebo_ros_joint_pose_trajectory.so` - For trajectory execution
- `libgazebo_ros_joint_state_publisher.so` - For publishing joint states
- `libgazebo_ros_diff_drive.so` - For differential drive robots

### Sensor Plugins
Common sensors for humanoid robots in Gazebo:
- Camera sensors (`libgazebo_ros_camera.so`)
- IMU sensors (`libgazebo_ros_imu.so`)
- Force/Torque sensors (`libgazebo_ros_ft_sensor.so`)
- LIDAR sensors (`libgazebo_ros_laser.so`)

## Launching Humanoid Robot Simulation

### Creating a Launch File
Example launch file for spawning a humanoid robot in Gazebo:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='humanoid.urdf',
        description='Robot description file'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(LaunchConfiguration('model')).read()}]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'humanoid_robot'],
        output='screen'
    )

    # Add actions to launch description
    ld.add_action(model_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)

    return ld
```

## Controlling Humanoid Robots in Simulation

### Joint Trajectory Control
For precise control of humanoid robot joints, use the JointTrajectoryController:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    use_sim_time: true

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
```

### Walking Pattern Generation
Implement walking patterns using ROS 2 control messages:
- Send joint position commands for walking gaits
- Use inverse kinematics for foot placement
- Implement balance control algorithms

## Best Practices for Humanoid Simulation

### Model Optimization
- Use simplified collision meshes for better performance
- Balance visual quality with simulation speed
- Properly configure inertial properties
- Test with realistic friction coefficients

### Simulation Tuning
- Adjust physics engine parameters for stability
- Use appropriate update rates for control loops
- Implement proper error handling for simulation resets
- Validate simulation behavior against real robot data

## Debugging and Visualization

### Gazebo GUI Tools
- Model inspector for examining robot properties
- Physics visualization for debugging contacts
- Plotting tools for analyzing sensor data
- Real-time performance monitoring

### ROS 2 Integration Debugging
- Use `rqt` for monitoring topics and services
- Visualize robot state with `rviz2`
- Monitor TF transforms for kinematic issues
- Log and analyze control performance

## Exercises

1. **Basic Setup**: Install Gazebo and launch a simple humanoid model
2. **URDF Integration**: Import your URDF model into Gazebo with proper physics properties
3. **Joint Control**: Implement basic joint position control in simulation
4. **Sensor Integration**: Add and visualize sensor data from simulated IMU and cameras
5. **Walking Simulation**: Create a simple walking pattern for your humanoid robot