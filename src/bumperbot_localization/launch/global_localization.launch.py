import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")
    planner_config = LaunchConfiguration("planner_config")
    controller_config = LaunchConfiguration("controller_config")
    bt_navigator_config = LaunchConfiguration("bt_navigator_config")
    local_costmap_config = LaunchConfiguration("local_costmap_config")
    global_costmap_config = LaunchConfiguration("global_costmap_config")
    
    lifecycle_nodes = ["map_server", "amcl", "planner_server", "controller_server", "bt_navigator", "behavior_server"]

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    map_yaml_file_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "maps",
            "small_house",
            "map.yaml"
        ),
        description="Full path to the map yaml file to load"
    )

    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "config",
            "amcl.yaml"
        ),
        description="Full path to amcl yaml file to load"
    )

    planner_config_arg = DeclareLaunchArgument(
        "planner_config",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "config",
            "navfn_planner.yaml"
        ),
        description="Full path to the planner yaml file to load"
    )

    controller_config_arg = DeclareLaunchArgument(
        "controller_config",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "config",
            "dwb_controller.yaml"
        ),
        description="Full path to the controller yaml file to load"
    )
    
    bt_navigator_config_arg = DeclareLaunchArgument(
        "bt_navigator_config",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "config",
            "bt_navigator.yaml"
        ),
        description="Full path to the behavior tree navigator yaml file to load"
    )

    local_costmap_config_arg = DeclareLaunchArgument(
        "local_costmap_config",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "config",
            "local_costmap.yaml"
        ),
        description="Full path to the local costmap yaml file to load"
    )

    global_costmap_config_arg = DeclareLaunchArgument(
        "global_costmap_config",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "config",
            "global_costmap.yaml"
        ),
        description="Full path to the global costmap yaml file to load"
    )

    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_yaml_file},
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        parameters=[
            amcl_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            planner_config,
            global_costmap_config,  # Load the global costmap config directly
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[
            controller_config,
            local_costmap_config,  # Load the local costmap config directly
            {"use_sim_time": use_sim_time},
        ],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            bt_navigator_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    nav2_behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("bumperbot_localization"), "config", "behavior_server.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_yaml_file_arg,
        amcl_config_arg,
        planner_config_arg,
        controller_config_arg,
        bt_navigator_config_arg,
        local_costmap_config_arg,
        global_costmap_config_arg,
        nav2_map_server,
        nav2_amcl,
        nav2_planner,
        nav2_controller,
        bt_navigator,
        nav2_lifecycle_manager,
        nav2_behavior_server
    ])