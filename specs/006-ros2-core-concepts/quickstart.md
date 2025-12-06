# Quickstart: Running the Examples

To verify the content of these chapters, you can simulate the reader's experience.

## Prerequisites
- ROS 2 (Humble or Jazzy) installed.
- `colcon` installed (`sudo apt install python3-colcon-common-extensions`).
- Basic Python 3 environment.

## Verification Steps

1.  **Create a Workspace**:
    ```bash
    mkdir -p ~/verify_ws/src
    cd ~/verify_ws/src
    ```

2.  **Create the Package**:
    ```bash
    ros2 pkg create --build-type ament_python my_core_concepts --dependencies rclpy std_msgs example_interfaces
    ```

3.  **Add Code**:
    - Copy the Python node examples from `data-model.md` into `~/verify_ws/src/my_core_concepts/my_core_concepts/`.

4.  **Configure Entry Points**:
    - Edit `setup.py` to add:
      ```python
      entry_points={
          'console_scripts': [
              'publisher = my_core_concepts.simple_publisher:main',
              'subscriber = my_core_concepts.simple_subscriber:main',
          ],
      },
      ```

5.  **Build**:
    ```bash
    cd ~/verify_ws
    colcon build
    ```

6.  **Run**:
    ```bash
    source install/setup.bash
    ros2 run my_core_concepts publisher
    # In another terminal
    source install/setup.bash
    ros2 run my_core_concepts subscriber
    ```
