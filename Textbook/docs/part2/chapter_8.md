# Chapter 8: Launch Files and Parameter Management

## Introduction

Welcome to Chapter 8, where we delve into two crucial aspects of any robust ROS 2 application: **Launch Files** and **Parameter Management**. As your robotic systems grow in complexity, manually starting each node and configuring every setting becomes cumbersome and error-prone. ROS 2 provides powerful tools to streamline these processes, allowing for efficient development, deployment, and operation of your robots.

In this chapter, you will learn how to:

-   Understand the fundamental role of launch files in orchestrating ROS 2 nodes.
-   Create and modify launch files using both Python and XML for flexible system configuration.
-   Effectively manage parameters to dynamically adjust your robot's behavior without recompiling code.
-   Troubleshoot common issues and debug your launch configurations and parameter settings.

By the end of this chapter, you will have the knowledge and practical skills to confidently set up, run, and fine-tune your ROS 2 applications, laying a solid foundation for more advanced robotics development.

## Understanding ROS 2 Launch Files

In the world of robotics, a single application often consists of multiple interconnected software components, known as **nodes** in ROS 2. For instance, a simple mobile robot might have separate nodes for motor control, sensor data processing, navigation, and user interface. Starting and managing these nodes individually, especially with specific configurations and inter-dependencies, can quickly become tedious and error-prone. This is where **ROS 2 Launch Files** come into play.

**What is a Launch File?**

A launch file is a powerful tool in ROS 2 that allows you to define and manage the startup of a group of nodes simultaneously. Instead of executing each node command separately in different terminals, you can define all the nodes, their parameters, and other settings within a single launch file. When this file is executed, ROS 2 orchestrates the entire system startup for you.

**Key Benefits of Using Launch Files:**

1.  **Automation**: Automates the process of starting multiple nodes and applications with a single command.
2.  **Configuration**: Centralizes the configuration of nodes, making it easier to manage parameters and remappings.
3.  **Reproducibility**: Ensures that your ROS 2 system starts up consistently every time, which is crucial for testing and deployment.
4.  **Parameter Management**: Allows you to set initial parameters for your nodes, which can be dynamically changed later.
5.  **Remapping**: Provides a way to remap topic, service, and action names without modifying the node's source code.
6.  **Conditional Execution**: Supports conditional logic to launch nodes or include other launch files based on specific conditions (e.g., different robot models or simulation environments).

Launch files are typically written in Python, offering powerful programmatic control, though XML-based launch files (from ROS 1) are also supported for compatibility and simpler cases.

**Simple Launch File Examples**

Let's look at a basic example where we launch two `turtlesim` nodes: the `turtlesim_node` itself and a `teleop_turtle` node to control it.

***

#### Python Launch File Example (`my_turtlesim_launch.py`)

Python launch files are the recommended approach in ROS 2 due to their flexibility and power.

```python
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim_turtle'
        ),
        launch_ros.actions.Node(
            package='turtlesim',
            executable='teleop_turtle',
            name='turtle_controller'
        )
    ])
```

**To run this Python launch file:**

1.  Save the code above as `my_turtlesim_launch.py` in a ROS 2 package (e.g., in a `launch` subdirectory).
2.  Open your terminal and source your ROS 2 environment.
3.  Execute the launch file:
    ```bash
    ros2 launch <your_package_name> my_turtlesim_launch.py
    ```

***

#### XML Launch File Example (`my_turtlesim_launch.xml`)

XML launch files offer a declarative way to achieve similar results, often preferred for simpler, static configurations.

```xml
<launch>
    <node pkg="turtlesim" exec="turtlesim_node" name="sim_turtle"/>
    <node pkg="turtlesim" exec="teleop_turtle" name="turtle_controller"/>
</launch>
```

**To run this XML launch file:**

1.  Save the code above as `my_turtlesim_launch.xml` in a ROS 2 package (e.g., in a `launch` subdirectory).
2.  Open your terminal and source your ROS 2 environment.
3.  Execute the launch file:
    ```bash
    ros2 launch <your_package_name> my_turtlesim_launch.xml
    ```

***

These examples illustrate the basic structure. Next, we will delve into how to create and modify such files to suit your specific application needs.

**Basic Structure and Tags of Launch Files**

Both Python and XML launch files share core concepts for defining what gets launched.

#### Python Launch File Structure

-   **`import launch` and `import launch_ros.actions`**: These lines import the necessary modules from the ROS 2 launch system. `launch` provides the core functionalities, while `launch_ros.actions` provides actions specific to ROS nodes.
-   **`def generate_launch_description():`**: This function is the entry point for your Python launch file. It must return a `LaunchDescription` object.
-   **`launch.LaunchDescription([...])`**: This object holds a list of actions that the launch system will execute.
-   **`launch_ros.actions.Node(...)`**: This is the most common action, used to start a ROS 2 node. Key arguments include:
    -   `package`: The name of the ROS 2 package where the node executable resides.
    -   `executable`: The name of the node's executable file.
    -   `name`: An optional, user-defined name for the node instance. This is useful when you want to launch multiple instances of the same executable with different names.

#### XML Launch File Structure

-   **`<launch>` tag**: This is the root element of every XML launch file.
-   **`<node>` tag**: This tag is used to start a ROS 2 node. Key attributes include:
    -   `pkg`: (equivalent to `package` in Python) The name of the ROS 2 package.
    -   `exec`: (equivalent to `executable` in Python) The name of the node's executable.
    -   `name`: An optional, user-defined name for the node instance.

In both formats, the goal is to clearly specify which nodes to run and how they should be configured.

## Creating and Modifying Launch Files

Beyond simply starting nodes, launch files offer powerful mechanisms to customize their behavior, primarily through arguments (`args`) and parameters. This section focuses on `args`, which are dynamic inputs you can pass to your launch files or nodes.

**Understanding Launch Arguments (`LaunchConfiguration`)**

Launch arguments provide flexibility, allowing you to configure aspects of your system at launch time without modifying the underlying launch file or node code.

#### Defining and Using Arguments in Python Launch Files

In Python launch files, arguments are typically defined using `DeclareLaunchArgument` and accessed using `LaunchConfiguration`.

```python
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    # 1. Declare a launch argument
    turtlesim_name_arg = DeclareLaunchArgument(
        'turtle_name',
        default_value='my_custom_turtle',
        description='Name of the turtlesim node'
    )

    return launch.LaunchDescription([
        turtlesim_name_arg, # Include the declared argument

        launch_ros.actions.Node(
            package='turtlesim',
            executable='turtlesim_node',
            name=LaunchConfiguration('turtle_name') # 2. Use the argument
        ),
        launch_ros.actions.Node(
            package='turtlesim',
            executable='teleop_turtle',
            name='turtle_controller'
        )
    ])
```

**How to run with an overridden argument:**

```bash
ros2 launch <your_package_name> my_arg_launch.py turtle_name:=ros_turtle
```

Here, `turtle_name` is an argument for the launch file. `:=` is used to assign a value to a launch argument from the command line.

#### Defining and Using Arguments in XML Launch Files

XML launch files use the `<arg>` tag to define arguments and `$(arg <arg_name>)` to use them.

```xml
<launch>
    <!-- 1. Declare a launch argument -->
    <arg name="turtle_name" default="my_custom_turtle" description="Name of the turtlesim node"/>

    <node pkg="turtlesim" exec="turtlesim_node" name="$(arg turtle_name)"/> <!-- 2. Use the argument -->
    <node pkg="turtlesim" exec="teleop_turtle" name="turtle_controller"/>
</launch>
```

**How to run with an overridden argument:**

```bash
ros2 launch <your_package_name> my_arg_launch.xml turtle_name:=ros_turtle
```

#### Overriding Arguments

The `default_value` specified in `DeclareLaunchArgument` (Python) or the `default` attribute in `<arg>` (XML) provides a fallback. You can easily override this default from the command line as shown above. This flexibility is incredibly useful for testing different configurations or adapting your system to various environments without changing the launch file itself. The examples provided above illustrate practical ways to configure nodes and pass arguments within your launch files.

## Managing ROS 2 Parameters

Parameters are fundamental to ROS 2 nodes, offering a way to configure their behavior dynamically without altering the node's source code. Think of parameters as settings or configurable variables within a node that can be read and modified during runtime or defined at startup via launch files.

**Role and Usage of Parameters in ROS 2 Nodes**

Every ROS 2 node can declare and manage its own set of parameters. These parameters can represent anything from a sensor's refresh rate, a robot's speed limit, a PID controller's gains, or a configuration string for a specific algorithm.

**Key Characteristics:**

-   **Dynamic Configuration**: Parameters can be changed while a node is running, allowing for real-time adjustments to a robot's behavior.
-   **Strong Typing**: ROS 2 parameters are strongly typed (e.g., `bool`, `int`, `double`, `string`, `byte_array`, and arrays of these types), ensuring data integrity.
-   **Description**: Parameters can have descriptions, making them self-documenting and easier to understand.
-   **Constraints**: You can define ranges, step values, and other constraints for parameters, preventing invalid configurations.

**Why use Parameters?**

1.  **Flexibility**: Easily adapt node behavior to different scenarios or environments without recompiling.
2.  **Modularity**: Nodes can be developed generically and configured externally, promoting reusability.
3.  **Debugging**: Modify parameters on-the-fly to test different settings and diagnose issues.
4.  **Persistent Configuration**: Parameters can be saved to files and loaded automatically at startup, ensuring consistent behavior across sessions.

**Setting Parameters in Launch Files**

Parameters can be defined directly within your Python or XML launch files, or loaded from external YAML files.

#### Setting Parameters in Python Launch Files

You can set parameters for a node directly in a Python launch file using the `parameters` argument of the `Node` action.

```python
import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    background_r_arg = DeclareLaunchArgument(
        'background_r',
        default_value='255',
        description='Red component for turtlesim background'
    )

    return launch.LaunchDescription([
        background_r_arg,
        launch_ros.actions.Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim_turtle',
            parameters=[
                {'background_r': LaunchConfiguration('background_r')}, # Set parameter from launch arg
                {'background_g': 0}, # Directly set an integer parameter
                {'background_b': 0}
            ]
        )
    ])
```

#### Setting Parameters in XML Launch Files

In XML, parameters are set using the `<param>` tag within a `<node>` tag, or loaded using `$(find_pkg)/path/to/params.yaml`.

```xml
<launch>
    <arg name="background_r" default="255"/>
    <node pkg="turtlesim" exec="turtlesim_node" name="sim_turtle">
        <param name="background_r" value="$(arg background_r)"/>
        <param name="background_g" value="0"/>
        <param name="background_b" value="0"/>
    </node>
</launch>
```

**Loading Parameters from Multiple YAML Files and Overriding**

A common and highly flexible approach is to define parameters in external YAML files. This allows for easy management and sharing of configurations. ROS 2 also provides mechanisms to load multiple YAML files and explicitly define overriding rules.

Let's assume we have two YAML files:

***

#### `config/default_params.yaml`

```yaml
sim_turtle:
  ros__parameters:
    background_r: 255
    background_g: 255
    background_b: 255
    turtle_name: default_turtle
```

***

#### `config/custom_colors.yaml`

```yaml
sim_turtle:
  ros__parameters:
    background_g: 100
    turtle_name: green_turtle
```

***

#### Python Launch File with Multiple YAMLs

```python
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'your_package_name' # Replace with your package name
    config_dir = os.path.join(get_package_share_directory(pkg_name), 'config')
    
    default_params_file = os.path.join(config_dir, 'default_params.yaml')
    custom_colors_file = os.path.join(config_dir, 'custom_colors.yaml')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim_turtle',
            parameters=[
                default_params_file,
                custom_colors_file # Parameters in this file will override those in default_params_file
            ]
        )
    ])
```

**Explicit Rules for Overriding:**

When multiple parameter sources are provided (e.g., directly in the launch file, from `default_params.yaml`, and then `custom_colors.yaml`), the **last specified source takes precedence**. In the Python example above, `custom_colors.yaml` is loaded after `default_params.yaml`, so any parameters defined in both files will have the value from `custom_colors.yaml`. This allows for a layered approach to configuration, where general settings can be defined in one file and specific overrides in another.

You can also override parameters directly from the command line when launching:

```bash
ros2 launch <your_package_name> my_params_launch.py sim_turtle.ros__parameters.background_b:=100
```
This command-line override will take precedence over parameters defined in YAML files or directly in the launch file.

**Interacting with Parameters using ROS 2 Command-Line Tools**

ROS 2 provides a powerful set of command-line interface (CLI) tools under `ros2 param` to inspect, get, set, and dump parameters of running nodes. These tools are invaluable for debugging, monitoring, and dynamically reconfiguring your ROS 2 system.

#### Listing Parameters

To see all parameters currently available on all running nodes:

```bash
ros2 param list
```

This will output a list like:
```
/sim_turtle:
  background_b
  background_g
  background_r
  ...
```

To list parameters for a specific node (e.g., `/sim_turtle`):

```bash
ros2 param list /sim_turtle
```

#### Getting Parameter Values

To retrieve the current value of a parameter:

```bash
ros2 param get /sim_turtle background_r
```

Output:
```
Integer value is: 255
```

#### Setting Parameter Values

You can change a parameter's value on a running node. For instance, to change the background color of `turtlesim`:

```bash
ros2 param set /sim_turtle background_b 150
```

After executing this, the `turtlesim` window background should change its blue component.

#### Dumping Parameters to a File

You can save all parameters of a specific node to a YAML file, which is useful for replicating configurations or debugging:

```bash
ros2 param dump /sim_turtle > sim_turtle_params.yaml
```

The `sim_turtle_params.yaml` file will contain:
```yaml
/**:
  ros__parameters:
    background_b: 150
    background_g: 0
    background_r: 255
    ...
```
These CLI tools offer granular control and visibility into your nodes' configurations, making them indispensable for development and troubleshooting.

**Brief Introduction to ROS 2 Node Lifecycle Management**

While this chapter focuses on launching and parameter management, it's important to briefly touch upon **ROS 2 Node Lifecycle Management** as it can interact with how nodes are started and configured. In traditional ROS (ROS 1), nodes were simply launched and ran until terminated. ROS 2 introduces a managed lifecycle for nodes, allowing them to transition through states like `unconfigured`, `inactive`, `active`, and `finalized`. This provides greater control and predictability, especially in critical applications.

**Relevance to Launch Files:**

Launch files can be configured to manage the lifecycle of nodes. For instance, you can specify that a node should automatically transition to the `active` state after being launched. While a deep dive into lifecycle management is beyond the scope of this chapter (and warrants its own dedicated discussion), understanding that nodes can have these managed states helps in comprehending more advanced launch configurations where state transitions are orchestrated.

## Best Practices and Debugging

### Best Practices for Launch Files and Parameters

Effective organization and adherence to best practices are crucial for maintaining readable, scalable, and maintainable ROS 2 launch configurations and parameter sets.

1.  **Modularize Launch Files**:
    -   Break down complex launch files into smaller, more manageable ones. Use `IncludeLaunchDescription` (Python) or `<include>` (XML) to compose them. This improves readability and reusability.
    -   For example, have separate launch files for hardware drivers, navigation stack, perception modules, etc., and then a top-level launch file to bring them all together.

2.  **Use Launch Arguments Effectively**:
    -   Parameterize your launch files using `DeclareLaunchArgument` or `<arg>` to allow for runtime configuration. This avoids hardcoding values and makes your launch files more flexible.
    -   Provide clear `description` for each argument to enhance readability and help users understand its purpose.

3.  **Organize Parameters in YAML**:
    -   Store node parameters in external YAML files, especially for complex configurations. This separates configuration from logic.
    -   Use a structured directory for your YAML files (e.g., `config/` within your package).
    -   Leverage multiple YAML files for different scenarios (e.g., `default.yaml`, `simulation.yaml`, `robot_x_params.yaml`) and use the overriding rules to manage them efficiently.

4.  **Meaningful Naming Conventions**:
    -   Use descriptive names for nodes, topics, services, actions, and parameters. This improves code clarity and debugging.
    -   Follow established ROS 2 naming conventions where applicable.

5.  **Document Everything**:
    -   Add comments to your launch files, especially for complex logic or non-obvious configurations.
    -   Ensure that your parameters have meaningful descriptions.

6.  **Test Your Launch Files**:
    -   Before deploying to a robot, always test your launch configurations thoroughly in a simulation or a controlled environment.
    -   Verify that all nodes start correctly, parameters are set as expected, and remappings are applied.

### Debugging Launch Files and Parameters

Even with the best practices, issues can arise. Debugging launch files and parameter configurations is a critical skill for any ROS 2 developer.

#### Common Issues and How to Debug Them:

1.  **Node Fails to Launch**:
    -   **Check the console output**: ROS 2 provides verbose error messages. Look for `ERROR` or `FATAL` messages related to missing executables, incorrect package names, or permission issues.
    -   **Verify package and executable names**: Ensure the `package` and `executable` (or `pkg` and `exec` in XML) arguments are correct and the executable is built and discoverable.
    -   **Check environment variables**: Ensure your ROS 2 environment is sourced correctly.
    -   **Launch individually**: Try launching the problematic node directly without the launch file (`ros2 run <package_name> <executable_name>`) to isolate the issue.

2.  **Parameters Not Applied as Expected**:
    -   **Verify parameter names**: Double-check for typos in parameter names in your launch file or YAML.
    -   **Check parameter overriding rules**: Remember that the last specified parameter source takes precedence. Ensure you're not accidentally overriding a desired value.
    -   **Inspect running parameters**: Use `ros2 param list <node_name>` and `ros2 param get <node_name> <param_name>` to see the actual parameters of a running node.
    -   **Dump parameters**: Use `ros2 param dump <node_name>` to see the full set of parameters loaded for a node.

3.  **Missing Dependencies**:
    -   If your launch file includes other launch files or references external packages, ensure all dependencies are correctly installed and built.
    -   Look for messages indicating "package not found" or "launch file not found".

4.  **Syntax Errors in Launch Files**:
    -   **Python**: Python launch files are Python scripts. Syntax errors will result in typical Python traceback errors. Check line numbers indicated in the error message.
    -   **XML**: XML launch files are parsed as XML. Errors might lead to messages about invalid XML structure. Use a linter or XML validator if needed.

#### Useful Debugging Tools:

-   **`ros2 launch --debug`**: Running a launch file with the `--debug` flag provides much more verbose output, which can help pinpoint the exact cause of a failure.
-   **`ros2 node info <node_name>`**: Provides detailed information about a running node, including its topics, services, actions, and parameters.
-   **`ros2 param` CLI**: As discussed, this is your primary tool for inspecting and manipulating parameters at runtime.
-   **`rqt_console`**: A GUI tool that displays ROS 2 log messages, which can help visualize errors and warnings from all your nodes.
-   **`rqt_graph`**: A GUI tool that shows the connections between nodes, topics, services, and actions. Useful for verifying that your nodes are communicating as expected.

By systematically using these techniques and tools, you can efficiently diagnose and resolve issues within your ROS 2 launch and parameter configurations.
