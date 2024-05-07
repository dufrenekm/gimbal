colcon build --symlink-install --packages-select gimbal_urdf


Launch UR5 with gimbal:
ros2 launch gimbal_urdf view_ur.launch.py ur_type:=ur5e

g