# Chapter 9: Gazebo Simulation Environment Setup

## Introduction

Welcome to Part 3 of our textbook, focusing on **Robot Simulation with Gazebo**. In the dynamic field of robotics, the ability to test, develop, and refine robot behaviors in a safe, repeatable, and cost-effective virtual environment is invaluable. This is where high-fidelity simulators like Gazebo play a crucial role.

**Gazebo** is a powerful 3D robotics simulator that allows you to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. It provides robust physics engines, high-quality graphics, and a convenient programmatic interface, making it an indispensable tool for researchers and developers working with robotic systems.

In this chapter, you will learn how to:

-   Successfully install and configure Gazebo on your development machine.
-   Integrate Gazebo with the Robot Operating System 2 (ROS 2) for seamless communication.
-   Understand and navigate Gazebo's user interface to control and inspect simulations.
-   Create a simple custom Gazebo world, laying the groundwork for more complex simulation scenarios.
-   Troubleshoot common issues encountered during Gazebo installation and ROS 2 integration.

By mastering the setup of your Gazebo simulation environment, you will be well-equipped to design, test, and validate a wide range of robotic applications, from autonomous navigation to complex manipulation tasks, all within a powerful virtual sandbox.

## Install and Configure Gazebo

The first step towards effective robot simulation is to set up your Gazebo environment. We will focus on installing **Gazebo Garden**, a modern and actively developed version, and then integrating it with ROS 2 Humble.

**Prerequisites**:

-   A Linux operating system (Ubuntu 22.04 LTS recommended for ROS 2 Humble).
-   A working installation of ROS 2 Humble. Refer to Chapter 5 and 6 if you need to set up your ROS 2 environment.

***

### Step 1: Install Gazebo Garden

Gazebo Garden is part of the Ignition Gazebo series. The installation process involves adding the Gazebo a.k.a. Ignition repository and then installing the necessary packages.

1.  **Add the Gazebo Garden repository**:
    ```bash
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    ```

2.  **Update your package list**:
    ```bash
    sudo apt update
    ```

3.  **Install Gazebo Garden**:
    ```bash
    sudo apt install gazebo-garden
    sudo apt install libgazebo-garden-dev
    ```
    This command installs the core Gazebo simulator and development files.

4.  **Verify installation**:
    To ensure Gazebo Garden is installed correctly, you can try launching it:
    ```bash
    gazebo
    ```
    This should open the Gazebo GUI with an empty world.

***

### Step 2: Integrate Gazebo Garden with ROS 2 Humble

To enable communication between Gazebo Garden and ROS 2 Humble, you need to install the `ros_gz_sim` bridge packages. These packages provide the necessary interfaces for ROS 2 nodes to interact with Gazebo simulations (e.g., publishing sensor data, sending motor commands).

1.  **Install `ros_gz_sim` packages**:
    ```bash
    sudo apt install ros-humble-ros-gz
    sudo apt install ros-humble-ros-gz-sim
    ```
    The `ros-humble-ros-gz` metapackage provides core tools for ROS 2 and Gazebo integration, while `ros-humble-ros-gz-sim` offers specific interfaces for `gz-sim` (Gazebo Garden/Ignition Gazebo).

2.  **Verify integration**:
    To confirm that ROS 2 can communicate with Gazebo, you can try launching a basic Gazebo world from a ROS 2 launch file.

    First, ensure your ROS 2 environment is sourced:
    ```bash
    source /opt/ros/humble/setup.bash
    ```

    Then, launch the `turtlesim` simulation with Gazebo (this example assumes you have `ros-humble-turtlesim` and `ros-humble-ros-gz-sim-demos` installed):
    ```bash
    ros2 launch ros_gz_sim_demos turtlesim_bridge.launch.py
    ```
    This command should launch Gazebo with a turtlesim robot, and you should be able to control it via ROS 2 topics. If this works, your Gazebo and ROS 2 Humble environments are successfully integrated! The previous steps have demonstrated how to launch Gazebo with a default world and from ROS 2.

***

## Understanding the Gazebo Interface

Once Gazebo is installed and launched (either with `gazebo` or via a ROS 2 launch file), you'll be presented with its graphical user interface (GUI). Understanding this interface is key to effectively interacting with and manipulating your simulation environments.

The Gazebo GUI typically consists of several main components:

1.  **3D Viewport**: This is the central and largest area of the GUI, displaying the simulated world in 3D. You can navigate this view using your mouse (left-click to orbit, middle-click to pan, scroll wheel to zoom).
2.  **Scene Tree (Left Panel)**: This panel displays a hierarchical list of all entities in your simulation, including the world itself, models (robots, objects), lights, and sensors. You can select items here to view or modify their properties.
3.  **Inspector (Right Panel)**: When an entity is selected in the Scene Tree or 3D Viewport, the Inspector panel shows its properties (e.g., position, orientation, physics parameters). You can often modify these properties directly in the GUI.
4.  **Toolbar (Top)**: Contains various tools for interacting with the simulation:
    -   **Play/Pause**: Controls the simulation time. You can pause the simulation to inspect specific moments or step through it frame by frame.
    -   **Insert Models**: Allows you to add pre-defined models (e.g., simple shapes, robots from the online database) into your world.
    -   **Camera Control**: Tools to adjust the camera perspective (orbit, pan, zoom).
    -   **Selection/Manipulation Tools**: Tools for selecting, moving, rotating, and scaling models within the 3D viewport.
5.  **Status Bar (Bottom)**: Displays important information such as simulation time, real-time factor (ratio of simulation time to real time), and messages.

#### Key Functionalities:

-   **Adding Models**: Click the "Insert" tab in the left panel or use the "Insert Models" button in the toolbar to add objects from Gazebo's model database or local files.
-   **Controlling Simulation Flow**: Use the Play/Pause button to start, stop, or reset the simulation.
-   **Inspecting Properties**: Select any object in the scene to view and modify its properties in the Inspector panel.

Familiarizing yourself with these elements will enable you to efficiently set up, control, and monitor your robot simulations.

*(Note: Screenshots or diagrams illustrating the Gazebo GUI, Scene Tree, Inspector, and Toolbar should be inserted here for visual guidance.)*

## Creating a Simple Gazebo World

While Gazebo comes with many pre-built worlds, creating your own custom world files is essential for tailoring simulation environments to your specific robotics projects. Gazebo uses the **Simulation Description Format (SDF)** to define worlds, models, and other simulation elements. SDF is an XML-based format that provides a comprehensive description of objects and environments for simulation.

#### Basic `.world` File Structure

A `.world` file defines everything about your simulation environment. Here's a minimal structure for an empty world:

```xml
<?xml version="1.0" ?>
<sdf version="1.8"> <!-- Specify the SDF version, 1.8 is for Gazebo Garden -->
  <world name="empty_world">
    <!-- Optional: Add a light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- Optional: Add a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
  </world>
</sdf>
```

**Explanation:**

-   **`<sdf version="1.8">`**: The root element, specifying the SDF version. Ensure this matches the Gazebo version you are using.
-   **`<world name="empty_world">`**: Defines a world with a unique name. All elements inside this tag belong to this world.
-   **`<include>`**: A convenient tag to import pre-defined models or components. Here, we include a `sun` (for lighting) and a `ground_plane` from Gazebo's model database.

**Steps to Create and Launch Your Custom World:**

1.  **Create a new file**: Save the XML content above as `my_empty_world.world` in a location accessible by Gazebo (e.g., within a ROS 2 package or your Gazebo `~/.gazebo/worlds` directory).
2.  **Launch Gazebo with your world**:
    ```bash
    gazebo my_empty_world.world
    ```
    This command will launch Gazebo and load your custom empty world.

#### Adding Basic Geometric Shapes

You can add various geometric shapes to your world by defining them as `<model>` elements with `<link>` and `<visual>` tags.

**Example: World with a Box**

Let's add a simple box to our `my_empty_world.world` file.

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="shapes_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Define a simple box model -->
    <model name="my_box">
      <pose>0 0 0.5 0 0 0</pose> <!-- Position (x y z roll pitch yaw) -->
      <link name="box_link">
        <visual name="box_visual">
          <geometry>
            <box>
              <size>1 1 1</size> <!-- 1x1x1 meter box -->
            </box>
          </geometry>
          <material>
            <ambient>0.0 0.0 1.0 1.0</ambient> <!-- Blue color -->
            <diffuse>0.0 0.0 1.0 1.0</diffuse>
          </material>
        </visual>
        <collision name="box_collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

**Explanation:**

-   **`<model name="my_box">`**: Defines a new model named `my_box`.
-   **`<pose>`**: Sets the initial position and orientation of the model. `0 0 0.5` places the bottom of the 1m tall box on the ground plane.
-   **`<link name="box_link">`**: A link is a rigid body part of a model.
-   **`<visual name="box_visual">`**: Defines the visual properties of the link (how it looks).
    -   **`<geometry><box><size>1 1 1</size></box></geometry>`**: Specifies a box shape of 1x1x1 meters.
    -   **`<material>`**: Sets the color and other material properties.
-   **`<collision name="box_collision">`**: Defines the physical properties of the link for collision detection. It's good practice to match the geometry of the visual.

By following similar patterns, you can add other shapes like cylinders or spheres:

-   **Cylinder**: `<cylinder><radius>0.5</radius><length>1.0</length></cylinder>`
-   **Sphere**: `<sphere><radius>0.5</radius></sphere>`

You can combine multiple models and shapes within your `.world` file to create more complex and realistic simulation environments.

## Troubleshooting Common Issues

Setting up simulation environments can sometimes be challenging due to various factors like system configurations, dependencies, or network issues. Here are some common problems you might encounter during Gazebo installation and ROS 2 integration, along with their solutions.

1.  **Gazebo Installation Fails Due to Unmet Dependencies**:
    -   **Problem**: `sudo apt install gazebo-garden` reports unmet dependencies.
    -   **Solution**: Ensure you have a clean Ubuntu installation and all system packages are up to date (`sudo apt update && sudo apt upgrade`). Sometimes, adding the `universe` repository helps: `sudo add-apt-repository universe`.
    -   **Specific to ROS 2**: If you're mixing ROS 2 and Gazebo repositories, dependency conflicts can arise. Double-check the official installation guides for your specific ROS 2 distribution and Gazebo version.

2.  **Gazebo Launches, But the ROS 2 Bridge (ros_gz_sim) Doesn't Work**:
    -   **Problem**: You can launch Gazebo, but ROS 2 nodes cannot communicate with it (e.g., `ros2 topic list` doesn't show expected topics from the simulation, or control commands don't work).
    -   **Solution**:
        -   **Sourcing**: Ensure both your Gazebo and ROS 2 environments are sourced correctly. For `ros_gz_sim` packages, it's often best to source your ROS 2 environment *after* installing them.
        -   **Package Installation**: Verify that `ros-humble-ros-gz` and `ros-humble-ros-gz-sim` (or equivalent for your ROS 2 distro) are installed.
        -   **Bridge Nodes**: Ensure the necessary bridge nodes are running. The `ros_gz_sim` package provides nodes to bridge topics, services, and parameters. Check your launch file or run `ros2 node list` to confirm.
        -   **Version Compatibility**: Confirm that your `ros_gz_sim` package version is compatible with your Gazebo Garden installation. Mismatched versions are a common source of issues.

3.  **Visual Issues or Performance Problems in Gazebo**:
    -   **Problem**: Gazebo GUI is slow, choppy, or displays graphical artifacts.
    -   **Solution**:
        -   **Graphics Drivers**: Ensure your graphics drivers are up to date and correctly installed, especially if you have a dedicated GPU.
        -   **System Resources**: Gazebo can be resource-intensive. Close unnecessary applications.
        -   **GUI vs. Server**: For headless simulations (no GUI), launch `gz sim` directly or use `ros2 launch <package> <launch_file_name> headless:=true` if supported by the launch file.

4.  **Custom World File Loading Issues**:
    -   **Problem**: Your custom `.world` file doesn't load, or objects don't appear as expected.
    -   **Solution**:
        -   **SDF Version**: Double-check that the `<sdf version="X.Y">` tag matches the SDF version supported by your Gazebo version.
        -   **XML Syntax**: SDF is XML-based. Even a small syntax error can prevent the file from loading. Use an XML linter or validator.
        -   **Model Paths**: Ensure any `model://` URIs refer to models that Gazebo can find (either in its default paths or via `GAZEBO_MODEL_PATH`).
        -   **Pose**: Verify the `pose` values for your models. Objects might be loaded outside the view or intersecting other objects.

By systematically addressing these common issues, you can ensure a smoother setup and development experience with Gazebo and ROS 2.
