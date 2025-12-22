from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 別パッケージのパラメータファイルを取得
    pkg_dir = get_package_share_directory('robomas_package_2')
    sender_param_file = os.path.join(
        get_package_share_directory('robomas_package_2'),
        'config',
        'sender_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='robomas_package_2',  # ← 別パッケージでもOK!!
            executable='sender',
            parameters=[sender_param_file],
            output='screen'
        )
        ,Node(
            package='nhk_r1_undercarriage',
            executable='undercarriage',
            name='undercarriage',
        )
        ,Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
        )
    ])

