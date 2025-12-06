# ROS 2 for Humanoid Control - Python Integration

## Summary

This chapter focuses on integrating Python agents with ROS 2 controllers, leveraging `rclpy`—the Python client library for ROS 2. You will learn how to create, build, and run basic ROS 2 nodes in Python, enabling your humanoid robot to communicate and interact within the ROS 2 ecosystem.

## Learning Objectives

Upon completing this chapter, you will be able to:
- Set up a Python environment for ROS 2 development.
- Create a basic ROS 2 Python package.
- Implement a simple ROS 2 publisher node using `rclpy`.
- Implement a simple ROS 2 subscriber node using `rclpy`.
- Understand how to compile and run Python ROS 2 nodes.

## 2.1 Setting Up Your Python Environment

Before diving into `rclpy`, ensure your Python environment is correctly configured for ROS 2. It's highly recommended to use a Python virtual environment to manage dependencies.

```bash
# Navigate to your project root or a dedicated workspace
# cd ~/humanoid-robotics-book

# Create a Python virtual environment if you haven't already (from the Quickstart Guide)
python3 -m venv ~/robotics_venv
source ~/robotics_venv/bin/activate

# Ensure rclpy and other necessary ROS 2 Python packages are installed
pip install -U rosdep rosinstall_generator vcstools colcon-common-extensions
pip install rclpy # rclpy is usually installed with ros-humble-desktop-full or equivalent
```

## 2.2 Creating a ROS 2 Python Package

Every ROS 2 application starts with a package. For Python nodes, you typically create a `colcon` workspace and then a Python package within it. We will use a simple Python package setup.

First, create a new `ros2_ws` (ROS 2 Workspace) and a Python package `humanoid_controller` inside it:

```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
ros2 pkg create --build-type ament_python humanoid_controller
```

This command creates a directory `humanoid_controller` with a basic `setup.py` and `package.xml` for your Python package.

## 2.3 Implementing a Simple ROS 2 Publisher Node

A publisher node sends messages to a topic. Let's create a node that publishes a simple string message "Hello ROS 2!" to a topic named `/chat_messages`.

Create a new file `humanoid_controller/humanoid_controller/talker.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'chat_messages', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2! %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Ensure `setup.py` in `humanoid_controller` is updated to include this executable. Add the following inside `setup( ... )` under `entry_points`:

```python
    entry_points={
        'console_scripts': [
            'talker = humanoid_controller.talker:main',
        ],
    },
```

## 2.4 Implementing a Simple ROS 2 Subscriber Node

A subscriber node receives messages from a topic. Let's create a node that subscribes to `/chat_messages` and prints the received data.

Create a new file `humanoid_controller/humanoid_controller/listener.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chat_messages',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Update `setup.py` again to include this new executable under `entry_points`:

```python
    entry_points={
        'console_scripts': [
            'talker = humanoid_controller.talker:main',
            'listener = humanoid_controller.listener:main',
        ],
    },
```

## 2.5 Building and Running Python ROS 2 Nodes

To run your nodes, you need to build your workspace and then source the setup files.

First, navigate back to your `ros2_ws` directory and build:

```bash
cd ~/humanoid-robotics-book/ros2_ws
colcon build --packages-select humanoid_controller
```

After a successful build, source the setup files. This makes your new executables available:

```bash
source install/setup.bash
```

Now, open two separate terminal windows. In the first, run the talker:

```bash
ros2 run humanoid_controller talker
```

In the second terminal, run the listener:

```bash
ros2 run humanoid_controller listener
```

You should see the talker publishing messages and the listener receiving and printing them in real-time.

## Conclusion

This chapter provided a hands-on introduction to `rclpy` and the creation of basic ROS 2 Python nodes. You've learned how to set up your environment, create a package, and implement publish-subscribe communication, laying the groundwork for more complex humanoid robot control. In the next chapter, we will delve into modeling humanoid robots using URDF and SDF, essential for defining their physical structure within ROS 2.
