# Quickstart: Chapter 12 - Unity for Robot Visualization

**Prerequisites**:
- Unity Hub installed.
- Valid Unity Account.
- Windows 10/11 Machine with GPU.
- WSL2 with Ubuntu 22.04 and ROS2 Humble installed (from previous chapters).

## 1. Unity Project Setup

1.  Open **Unity Hub**.
2.  Click **New Project**.
3.  Select **3D (URP)** template.
4.  Name the project `RoboticsViz` and click **Create**.
5.  Wait for the Editor to open.

## 2. Install ROS Integration

1.  In Unity Editor, go to **Window > Package Manager**.
2.  Click the **+ (plus)** icon > **Add package from git URL**.
3.  Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
4.  Click **Add**. Wait for installation.
5.  A "Robotics" menu should appear in the top bar.

## 3. Configure Networking

1.  Open your **WSL2** terminal. Run `hostname -I` to get your IP (e.g., `172.29.x.x`).
2.  In Unity, go to **Robotics > ROS Settings**.
3.  **ROS IP Address**: Enter the WSL2 IP (`172.29.x.x`).
4.  **Host Port**: `10000`.
5.  **Protocol**: Select `ROS2`.

## 4. Launch ROS Endpoint

1.  In **WSL2**, navigate to your ROS2 workspace.
2.  Clone the endpoint: `git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint src/ros_tcp_endpoint`
3.  Build: `colcon build --packages-select ros_tcp_endpoint`
4.  Source: `source install/setup.bash`
5.  Run: `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0`
6.  *Windows Firewall Popup*: If prompted, allow access.

## 5. Verify Connection

1.  In Unity, click the **Play** button.
2.  Look at the "Game" view top-left corner. You should see a blue connection icon or "Connected" status in the HUD.
3.  In WSL2 terminal, the endpoint log should show `Connection from <HostIP>`.
