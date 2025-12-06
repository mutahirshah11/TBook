---
title: "Building ROS 2 Packages with Python"
sidebar_label: "7. Building ROS 2 Packages with Python"
---

# Chapter 7: Building ROS 2 Packages with Python

In Chapter 6, we looked at individual nodes and communication patterns. Now, we'll learn how to organize that code into **Packages**, which are the standard unit of software distribution in ROS 2.

## 1. The ROS 2 Workspace

A **Workspace** is a directory where you develop your ROS 2 code. It typically has a specific structure:

```text
~/ros2_ws/
└── src/  <-- Your source code goes here
```

When you build the workspace, `colcon` (the ROS 2 build tool) creates three other directories:
*   `build/`: Intermediate build files.
*   `install/`: Where your package is installed to be used.
*   `log/`: Logs from the build process.

### Best Practice
Always run build commands from the root of your workspace (`~/ros2_ws`), not from inside the package directories.

## 2. Creating a Python Package

The `ros2` command line tool makes creating a package easy.

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python <package_name> --dependencies rclpy std_msgs
```

*   `--build-type ament_python`: Tells ROS 2 this is a Python package.
*   `--dependencies`: Adds dependencies to your `package.xml` immediately.

### Key Files Created
*   `package.xml`: Meta-information about your package (version, maintainer, dependencies).
*   `setup.py`: Installation instructions for Python.
*   `setup.cfg`: Configuration for `setup.py`.
*   `resource/<package_name>`: Marker file for ROS 2 to find your package.

## 3. Managing Dependencies

The `package.xml` file defines what your package needs to run. For a Python package, you typically see:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

If you import a new message type or library, you MUST add it here. This ensures that when you share your code, others can install the necessary dependencies easily using `rosdep`.

## 4. Defining Entry Points

In `setup.py`, the `entry_points` dictionary maps a command-line executable name to a specific Python function in your code.

```python
entry_points={
    'console_scripts': [
        'my_node = my_package.my_node:main',
        'talker = my_package.publisher:main',
        'listener = my_package.subscriber:main',
    ],
},
```

*   **Left side** (`'talker'`): The name of the executable (what you type after `ros2 run my_package ...`).
*   **Right side** (`'my_package.publisher:main'`): The path to the function to run (`<package>.<module>:<function>`).

**Important**: If you forget this step, `colcon build` might succeed, but `ros2 run` won't find your node!

## 5. Building and Running

Once your code is written and configured, go back to the workspace root to build.

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

*   `--symlink-install`: Creates links to your Python files instead of copying them. This allows you to edit your Python code and run it immediately without re-building every time (useful for development).

### Sourcing

After building, you must source the **overlay** to add your new package to the environment.

```bash
source install/setup.bash
```

### Running

Now you can run your nodes like any other ROS 2 executable:

```bash
ros2 run <package_name> <executable_name>
```

For example, if you defined an entry point named `talker` in package `my_first_package`:

```bash
ros2 run my_first_package talker
```



