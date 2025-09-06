Project#1 Diff_robot_keyboard_move_and_publish_pose
In this project, a differential-drive robot is controlled via the keyboard.
The keyboard provides the robot’s center velocity, and the kinematic model computes and publishes the corresponding wheel joint speeds.
The movement is simulated in Gazebo, with launch files created for both the controller and the keyboard.

[![Watch the video](https://img.youtube.com/vi/q4IU0VDno98/0.jpg)](https://www.youtube.com/watch?v=q4IU0VDno98)




Project#2 Diff_robot_broadcast_odometry
In this project, a differential-drive robot is controlled via the keyboard. The keyboard supplies the robot’s center velocity, and the kinematic model computes and publishes the corresponding wheel joint speeds. Encoder readings are used to estimate the robot’s new position, orientation, and linear and angular velocities from changes in wheel rotation. These updated values are published as odometry and TF messages for display in RViz2.

[![Watch the video](https://img.youtube.com/vi/UcaGSv46jUI/0.jpg)](https://www.youtube.com/watch?v=UcaGSv46jUI)



Project#3 Diff_robot_draw_movement_trajectory
In this project, a differential-drive robot is controlled via the keyboard. The keyboard provides the robot’s center velocity, and the kinematic model computes and publishes the wheel joint speeds. Encoder readings estimate the robot’s new pose and velocities from wheel rotations. These odometry updates are published for another node to draw the corresponding trajectory in RViz2.

[![Watch the video](https://img.youtube.com/vi/qSpBzqmqf5w/0.jpg)](https://www.youtube.com/watch?v=qSpBzqmqf5w)


Project#4 Drift due to_sensor_noise
In this project, we compare ideal controllers with real controllers, accounting for encoder noise and dimensional errors, and display them as two separate TFs in RViz2, along with noise visualization in PlotJuggler.

[![Watch the video](https://img.youtube.com/vi/tZAHXG23HeA/0.jpg)](https://www.youtube.com/watch?v=tZAHXG23HeA)




