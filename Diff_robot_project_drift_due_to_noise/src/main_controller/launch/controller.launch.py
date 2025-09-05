import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from launch.actions import DeclareLaunchArgument

def adding_noise_to_readings(context, *args, **other):
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_python = LaunchConfiguration("use_python")
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))

    real_controller_py = Node(
        package="main_controller",
        executable="real_controller.py",
        parameters=[
            {"wheel_radius": wheel_radius + wheel_radius_error,
             "wheel_separation": wheel_separation + wheel_separation_error}],
       
    )

    return   [real_controller_py]
      
    


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )
  
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius", #the name of the argument should be the same name as the declared parameter in the .py file
        default_value="0.033",
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17",
    )
    
    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005",
    )
    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.02",
    )
    
    
    wheel_radius = LaunchConfiguration("wheel_radius") #  At launch time, get the value assigned to the launch argument named "wheel_radius" 
                                                       #and treat it as a variable called wheel_radius in this launch script.
   
   
    wheel_separation = LaunchConfiguration("wheel_separation")
    


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["simple_velocity_controller",  #The .yaml path is never in the launch file because it is already embedded in the URDF/Xacro and sent to Gazebo via the create node.
                                                    # The plugin inside Gazebo, not your launch file, reads and applies it.
                "--controller-manager",
                "/controller_manager"
        ]
    )

    ideal_controller_py = Node(             # ideal_controller_py launches a ideal Python ROS 2 node that you wrote so 
                                              # the executable is simple_controller.py, while the other nodes (like velocity_controller and 
                                             #  joint_state_broadcaster_spawner) launch built-in ROS 2 controller manager tools  (written in C++).
                package="main_controller",
                executable="ideal_controller.py",
                parameters=[{
                    "wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation }],
                
            )
   
    real_controller_launch = OpaqueFunction(function=adding_noise_to_readings)

    return LaunchDescription(
        [  
            wheel_radius_arg,
            wheel_separation_arg,
            wheel_radius_error_arg,
            wheel_separation_error_arg,

            joint_state_broadcaster_spawner,
            velocity_controller,

            ideal_controller_py,
            real_controller_launch,
            
            

        ]
    )