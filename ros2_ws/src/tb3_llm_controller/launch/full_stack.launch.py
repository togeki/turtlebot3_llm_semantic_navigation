from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    db_url = LaunchConfiguration('db_url')

    return LaunchDescription([
        DeclareLaunchArgument(
            'db_url',
            default_value='postgresql://tb3:tb3pass@localhost:5432/tb3db',
            description='PostgreSQL connection URL'
        ),

        # 1) 数据采集器（ROS -> DB）
        Node(
            package='tb3_data_pipeline',
            executable='tb3_collector',
            name='tb3_collector',
            output='screen',
            parameters=[{'db_url': db_url}]
        ),

        # 2) 语义地点 -> pose/json 的桥
        Node(
            package='tb3_llm_controller',
            executable='semantic_place_bridge',
            name='semantic_place_bridge',
            output='screen',
        ),

        # 3) json -> PoseStamped -> /go_to_pose 的桥
        Node(
            package='tb3_llm_controller',
            executable='llm_nav_bridge',
            name='llm_nav_bridge',
            output='screen',
        ),

        # 4) /go_to_pose -> Nav2 Action
        Node(
            package='tb3_llm_controller',
            executable='nav2_client',
            name='nav2_client',
            output='screen',
        ),
    ])

