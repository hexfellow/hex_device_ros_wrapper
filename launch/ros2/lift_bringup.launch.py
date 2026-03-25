import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
    
    enable_bridge = DeclareLaunchArgument(
        'enable_bridge',
        default_value='false',
        description='Whether to enable the hex_bridge node, you can set it to false if you want to launch the node separately.'
    )
    
    url = DeclareLaunchArgument(
        'url',
        default_value='0.0.0.0:8439',
        description='The URL of the robot.'
    )

    read_only = DeclareLaunchArgument(
        'read_only',
        default_value='false',
        description='Whether to read only the arm state.'
    )

    is_kcp = DeclareLaunchArgument(
        "is_kcp",
        default_value="true",
        description="Is KCP mode"
    )
    
    # =========== launch ==============
    
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hex_bridge'),
                'launch',
                'bridge.launch.py'
            )
        ),
        launch_arguments={
            'url': LaunchConfiguration('url'),
            'read_only': LaunchConfiguration('read_only'),
            'is_kcp': LaunchConfiguration('is_kcp'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_bridge'))
    )
    
    # ============= node =============== 
    lift_node = Node(
        package='hex_device',
        executable='lift_trans',
        name='lift_trans',
        remappings=[
            # publish
            ('/xtopic_lift/joint_cmd', '/joint_cmd')
        ]
    )
    
    return LaunchDescription([
        # parameter
        enable_bridge,
        url,
        read_only,
        is_kcp,
        # launch
        bridge_launch,
        # node
        lift_node
    ])