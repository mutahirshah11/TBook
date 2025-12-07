## Troubleshooting

### Connection Refused (10061)
If you see a `SocketException: No connection could be made...` error in Unity:
- Check that `ros_tcp_endpoint` is running in WSL2.
- Verify the **IP Address** in Unity ROS Settings matches `hostname -I` from WSL2.
- Ensure the **Windows Firewall Rule** for port 10000 is active.
- Try pinging the WSL2 IP from Windows PowerShell: `ping <WSL2_IP>`.

### Pink Materials
If materials are still pink after using the Render Pipeline Converter:
- Select the pink object in the Scene.
- In the Inspector, check the **Material** component.
- Manually change the Shader to `Universal Render Pipeline/Lit`.

### Message Version Mismatch
If you see serialization errors:
- Ensure you have rebuilt the ROS2 workspace after adding new messages.
- Re-generate the C# structs in Unity if you changed the ROS message definition.

## Summary

In this chapter, we established a high-speed TCP bridge between our ROS2 logic and Unity's rendering engine. We learned how to:
1.  Configure a robust cross-platform network setup.
2.  Import URDF models with correct articulation physics.
3.  Write C# scripts to Publish and Subscribe to ROS topics.

In the next chapter, we will build a complete simulation scenario with sensors and a control loop.

