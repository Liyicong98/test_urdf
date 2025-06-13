# test_urdf
Visualization with RViz: Changes in the pitch, yaw, and roll angles of the underwater robot.
step1:
终端1：
cd ~/test_urdf
colcon build
source install/setup.bash
ros2 launch test_urdf display.launch.py
在rviz界面，点击左下角Add,选择Robot_Model,Description Topic：/robot_desciption,Fixed_Frame:base_link。

step2：
终端2：
cd ~/test_urdf
colcon build
source install/setup.bash
ros2 run test_urdf fixed_pose.py或者ros2 run test_urdf head_control.py
fixed_pose.py是控制机器人整体俯仰角、偏航角、滚转角的变化
head_control.py是控制机器人头部


https://github.com/user-attachments/assets/299e7251-9923-4f94-9e56-23b397530429



https://github.com/user-attachments/assets/33424494-baa0-4f1b-bac4-640b322ac662

