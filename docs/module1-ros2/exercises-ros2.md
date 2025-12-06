# ROS 2 for Humanoid Control - Exercises

## Summary

This chapter provides hands-on exercises to reinforce your understanding of ROS 2 fundamentals, Python integration, and humanoid robot modeling. These exercises are designed to guide you through creating basic ROS 2 packages, implementing communication nodes, and defining a simple robot structure.

## Learning Objectives

Upon completing these exercises, you will be able to:
- Independently create and configure a ROS 2 Python package.
- Implement and test ROS 2 publish-subscribe communication.
- Develop a basic ROS 2 service.
- Construct a simple URDF model for a robotic component.

## Exercise 1: ROS 2 Workspace and Package Setup

**Goal**: Create a ROS 2 workspace and a basic Python package for future humanoid control.

1. **Create a new ROS 2 Workspace**: If you haven't already, create a `colcon` workspace in your home directory or desired development location.
   ```bash
   mkdir -p ~/ros2_humanoid_ws/src
   cd ~/ros2_humanoid_ws/src
   ```
2. **Create a Python Package**: Inside your workspace's `src` directory, create a new Python package named `my_humanoid_control`.
   ```bash
   ros2 pkg create --build-type ament_python my_humanoid_control
   ```
3. **Inspect Package Structure**: Navigate into the newly created `my_humanoid_control` directory. Identify `setup.py`, `package.xml`, and the `my_humanoid_control` subdirectory. Briefly understand their roles.

## Exercise 2: Publisher and Subscriber Nodes

**Goal**: Implement and verify ROS 2 publish-subscribe communication using Python.

1. **Implement a Publisher**: Inside your `my_humanoid_control/my_humanoid_control` directory, create a Python file `my_publisher.py`. Implement a node that publishes `std_msgs/String` messages to a topic named `/robot_status` every 0.5 seconds. The message content should include a counter (e.g., "Robot status update: [count]").
2. **Implement a Subscriber**: In the same directory, create `my_subscriber.py`. Implement a node that subscribes to `/robot_status` and prints the received messages to the console.
3. **Update `setup.py`**: Modify the `setup.py` file in your `my_humanoid_control` package to include `my_publisher` and `my_subscriber` as console scripts.
4. **Build and Source**: From your `~/ros2_humanoid_ws` directory, build your package and source the setup files.
   ```bash
   colcon build --packages-select my_humanoid_control
   source install/setup.bash
   ```
5. **Run and Verify**: Open two separate terminals. Run the publisher in one and the subscriber in the other. Observe if messages are correctly exchanged.

## Exercise 3: Basic ROS 2 Service

**Goal**: Implement and test a simple ROS 2 service for requesting a robot action.

1. **Define a Service Interface**: In your `my_humanoid_control` package root, create a `srv` directory. Inside `srv`, define a custom service interface, e.g., `SetLED.srv`:
   ```srv
   string led_name
   bool state
   ---
   bool success
   string message
   ```
2. **Update `package.xml` and `CMakeLists.txt`**: Add dependencies for your new service in `package.xml` (e.g., `rosidl_default_generators`) and ensure `CMakeLists.txt` is configured to build the service (refer to official ROS 2 custom interface tutorials).
3. **Implement Service Server**: Create `my_service_server.py` in `my_humanoid_control/my_humanoid_control`. This node should implement the `SetLED` service, printing the request details and always returning `success=True`.
4. **Implement Service Client**: Create `my_service_client.py`. This node should call the `SetLED` service, requesting to turn on an LED named "head_led".
5. **Update `setup.py`**: Add `my_service_server` and `my_service_client` as console scripts.
6. **Build and Source**: Rebuild your workspace and re-source the setup files.
7. **Run and Verify**: Run the service server in one terminal. In another, run the client. Verify the client receives a successful response.

## Exercise 4: Simple URDF for a Robot Arm Segment

**Goal**: Create a basic URDF description for a single robot arm segment, including a link and a joint.

1. **Create a `urdf` directory**: Inside your `my_humanoid_control` package, create a new directory `urdf`.
2. **Define a Link**: In `urdf/simple_arm_segment.urdf`, define a `link` named `base_link` with visual, collision, and inertial properties. Make it a simple box geometry.
3. **Define a Joint**: Add a `joint` named `revolute_joint` of type `revolute`. Make `base_link` its parent and define a new `child` link named `end_effector_link`. Set appropriate `origin`, `axis`, and `limit` tags.
4. **Visualize (Optional - Requires `urdf_viz` or Gazebo)**: If you have `urdf_viz` installed (`sudo apt install ros-humble-urdf-viz`), you can visualize your URDF:
   ```bash
   urdf_to_graphiz simple_arm_segment.urdf
   # Or for visualization
   ros2 launch urdf_viz display.launch model:=simple_arm_segment.urdf
   ```
   *(Note: The `display.launch` command requires setting up a launch file and potentially installing `joint_state_publisher_gui` and `robot_state_publisher` packages).*

## Conclusion

These exercises have provided a practical foundation in ROS 2 development and robot modeling. You've gained hands-on experience with creating packages, implementing communication patterns, and defining robot structures, which are critical skills for building and controlling humanoid robots. Continue to experiment with these concepts, and you'll be well-prepared for the more advanced topics in digital twins and AI integration.
