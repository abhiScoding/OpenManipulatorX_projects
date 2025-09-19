connect with hardware

$ ros2 launch open_manipulator_x_bringup hardware.launch.py

move joints through waypoints

$ ros2 run open_manipulator_x_bringup move_joints.py "0 -0.997 0.7 0.299 0.015, 0 -0.204 -0.69 1.829 0.015, 0.491 0.137 -1.123 1.92 0.015, 0.428 -0.132 -0.143 1.208 0.015, 0.428 0.098 0.17 0.665 0.015, 0.428 0.098 0.17 0.665 0.009"

pick and place works
but need to repeat way points at picking place in order to wait till it picks: 0.428 0.098 0.17 0.665 -0.001, 0.428 0.098 0.17 0.665 -0.001

$ ros2 run open_manipulator_x_bringup move_joints.py "0 -0.997 0.7 0.299 0.015, 0 -0.204 -0.69 1.829 0.015, 0.491 0.137 -1.123 1.92 0.015, 0.428 -0.132 -0.143 1.208 0.015, 0.428 0.098 0.17 0.665 0.015, 0.428 0.098 0.17 0.665 0.009, 0.428 0.098 0.17 0.665 -0.001, 0.428 0.098 0.17 0.665 -0.001, 0.428 -0.365 -0.413 1.713 -0.001, -0.448 -0.077 -0.686 1.697 -0.001, -0.448 -0.077 -0.686 1.697 0.005"
