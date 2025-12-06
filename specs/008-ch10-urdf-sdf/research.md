# Research for Chapter 10: URDF and SDF Robot Description Formats

This document will contain findings from research on URDF/SDF versions, compatibility with ROS 2 and Gazebo, and URDF to SDF conversion tools.

## Research Tasks:

-   **Research Task 1**: Determine the recommended versions/standards for URDF and SDF to target, especially in the context of ROS 2 Humble and Gazebo Garden.
-   **Research Task 2**: Identify the most stable and user-friendly tool/method for URDF to SDF conversion, considering compatibility with ROS 2 Humble and Gazebo Garden.

## Findings:

-   **Recommended URDF/SDF Versions/Standards**: The chapter will target URDF and SDF specifications widely supported by ROS 2 Humble and Gazebo Garden.
-   **URDF to SDF Conversion Tool**: The primary recommended method will involve leveraging `ros_gz_bridge` (specifically components like `ros_gz_sim` for model spawning) for using URDF-defined robots within Gazebo Garden. Direct conversion using tools like `urdf_to_sdf` might be mentioned as an alternative or for debugging purposes.
