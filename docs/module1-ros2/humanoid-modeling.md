# ROS 2 for Humanoid Control - Humanoid Modeling

## Summary

This chapter delves into the crucial aspect of modeling humanoid robots within the ROS 2 ecosystem, focusing on Universal Robot Description Format (URDF) and Simulation Description Format (SDF). You will learn how to define the physical structure of a humanoid, including its joints, links, sensors, and actuators, which is essential for accurate simulation and control.

## Learning Objectives

Upon completing this chapter, you will be able to:
- Understand the purpose and structure of URDF for robot modeling.
- Understand the purpose and structure of SDF for robot and environment modeling in simulation.
- Define links, joints, and transmissions in a URDF file.
- Integrate sensors and actuators into a robot model using URDF/SDF.
- Create a basic URDF model for a simple humanoid robot.

## 3.1 Introduction to URDF

**Universal Robot Description Format (URDF)** is an XML-based file format used in ROS to describe all aspects of a robot. It represents the robot as a kinematic and dynamic model, specifying the robot's visual appearance, collision properties, and inertial properties. For humanoids, URDF allows us to precisely define the body segments (links) and their connections (joints).

### 3.1.1 Links

A **link** represents a rigid body segment of the robot. Examples in a humanoid include the torso, upper arm, forearm, hand, thigh, shin, and foot. Each link has associated geometric (visual and collision) and inertial properties (mass, inertia matrix).

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.2 0.4 0.6" />
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1" />
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.2 0.4 0.6" />
    </geometry>
  </collision>
  <inertial>
    <mass value="10.0" />
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
  </inertial>
</link>
```

### 3.1.2 Joints

A **joint** connects two links, defining their kinematic relationship and allowed motion. In humanoids, joints represent articulations like shoulders, elbows, hips, and knees. URDF supports various joint types, including revolute (rotational), prismatic (translational), fixed, and continuous.

```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0 0.2 0.3" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
</joint>
```

### 3.1.3 Transmissions

**Transmissions** define the relationship between an actuator (e.g., motor) and a joint. They map actuator effort/velocity to joint effort/velocity, which is crucial for inverse kinematics and dynamic control. While not directly part of the robot's physical structure, they are an important part of the control chain.

```xml
<transmission name="shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="shoulder_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="shoulder_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## 3.2 Simulation Description Format (SDF)

While URDF is excellent for describing single robots, **Simulation Description Format (SDF)** is a more comprehensive XML format designed for describing robots, environments, and even entire worlds for simulation in Gazebo. SDF can describe multiple robots, static objects, lighting, and other simulation-specific elements. For complex humanoid simulations, SDF often wraps or extends URDF definitions.

### Key Differences from URDF:
- **Environment Modeling**: SDF can describe entire worlds, including terrain, buildings, and other static objects.
- **Nested Models**: SDF supports nested models, allowing for hierarchical structuring of complex scenes.
- **Sensors and Plugins**: SDF has more extensive support for defining simulation-specific sensors and Gazebo plugins.

## 3.3 Sample Humanoid URDF/SDF Structure

Here's a simplified example of how a humanoid structure might be defined, showing the hierarchical link-joint relationships:

```xml
<!-- Minimal Humanoid Example (head, torso, one arm) -->
<robot name="simple_humanoid">

  <!-- Base Link: Torso -->
  <link name="torso">
    <!-- ... visual, collision, inertial properties ... -->
  </link>

  <!-- Head Link & Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="1"/>
  </joint>
  <link name="head">
    <!-- ... visual, collision, inertial properties ... -->
  </link>

  <!-- Right Arm: Shoulder -->
  <joint name="right_shoulder_pitch_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 0.25 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="5"/>
  </joint>
  <link name="right_upper_arm">
    <!-- ... visual, collision, inertial properties ... -->
  </link>

  <!-- Example Sensor: Camera on Head -->
  <joint name="camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
  </joint>
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.05 0.03" />
      </geometry>
    </visual>
  </link>

</robot>
```

## Conclusion

Accurate robot modeling using URDF and SDF is foundational for any humanoid robotics project. This chapter has equipped you with the knowledge to define the physical and kinematic properties of your robot, essential for both real-world control and high-fidelity simulation. With a solid model, you can now bring your humanoid to life in simulation environments, which we will explore in the next module.
