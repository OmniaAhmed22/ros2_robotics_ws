Project#1 Diff_robot_project_keyboard_move_and_publish_pose
In this project, a differential-drive robot is controlled via the keyboard.
The keyboard provides the robot’s center velocity, and the kinematic model computes and publishes the corresponding wheel joint speeds.
The movement is simulated in Gazebo, with launch files created for both the controller and the keyboard.

https://github.com/user-attachments/assets/5396b12e-957e-4b4f-8e71-5ea97fa94fa6


Project#2 Diff_robot_project_broadcast_odometry
In this project, a differential-drive robot is controlled via the keyboard. The keyboard supplies the robot’s center velocity, and the kinematic model computes and publishes the corresponding wheel joint speeds. Encoder readings are used to estimate the robot’s new position, orientation, and linear and angular velocities from changes in wheel rotation. These updated values are published as odometry and TF messages for display in RViz2.



