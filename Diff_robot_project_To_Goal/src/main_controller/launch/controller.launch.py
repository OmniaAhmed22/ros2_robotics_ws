import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from launch.actions import DeclareLaunchArgument

def generate_launch_description():


    use_python_arg = DeclareLaunchArgument(
        "use_python",
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
    
    
    
    
    wheel_radius = LaunchConfiguration("wheel_radius") #  At launch time, get the value assigned to the launch argument named "wheel_radius" 
                                                       #and treat it as a variable called wheel_radius in this launch script.
   
   
    wheel_separation = LaunchConfiguration("wheel_separation")

    


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
    to_goal_node = Node(
        package="main_controller",
        executable="to_goal.py",
        output="screen"
    )
   


    return LaunchDescription(
        [  
            wheel_radius_arg,
            wheel_separation_arg,
            

            joint_state_broadcaster_spawner,
            velocity_controller,

            ideal_controller_py,
         
            to_goal_node,
            

        ]
    )