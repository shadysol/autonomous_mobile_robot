import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    use_saved_map = LaunchConfiguration("use_saved_map", default="false")
    saved_map_path = LaunchConfiguration("saved_map_path", default="/home/shady/bumperbot_ws/small_house_GraphSLAM.yaml")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    use_saved_map_arg = DeclareLaunchArgument(
        "use_saved_map",
        default_value="false",
        description="Whether to use the saved SLAM map for localization"
    )

    saved_map_path_arg = DeclareLaunchArgument(
        "saved_map_path",
        default_value="/home/shady/bumperbot_ws/small_house_GraphSLAM.yaml",
        description="Path to the saved SLAM-generated map .yaml file"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_description"),
                "launch",
                "gazebo.launch.py"
            )
        ),
    )
    
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "true"
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_controller"),
                "launch",
                "joystick_teleop.launch.py"
            )
        ),
    )

    safety_stop = Node(
        package="bumperbot_utils",
        executable="safety_stop",
        output="screen",
    )



    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_mapping"),
                "launch",
                "slam.launch.py"
            )
        ),
        condition=IfCondition(use_slam)
    )

    rviz_localization = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("bumperbot_localization"),
                "rviz",
                "global_localization.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(use_slam)
    )

    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("bumperbot_mapping"),
                "rviz",
                "slam.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_slam)
    )

    # Include the local localization launch file
    local_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_localization"),
                "launch",
                "local_localization.launch.py"
            )
        ),
        launch_arguments={"use_python": "true"}.items(),  # Set use_python to true or false as needed
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_localization"),
                "launch",
                "global_localization.launch.py"
            )
        ),
        launch_arguments={
            "map": saved_map_path
        }.items(),
        condition=UnlessCondition(use_slam)
    )
    
    return LaunchDescription([
        use_slam_arg,
        use_saved_map_arg,
        saved_map_path_arg,
        gazebo,
        controller,
        joystick,
        safety_stop,
        localization,
        slam,
        rviz_localization,
        rviz_slam,
        local_localization,  # Add this line to include the local localization launch
    ])
