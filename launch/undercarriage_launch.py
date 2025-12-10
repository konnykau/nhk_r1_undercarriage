from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 別パッケージのパラメータファイルを取得
    params = os.path.join(
        get_package_share_directory('robomas_package_2'),
        'config',
        'sender_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='robomas_package_2',  # ← 別パッケージでもOK!!
            executable='sender',
            name='sender',
            parameters=[params],
        )
        ,Node(
            package='nhk_r1_undercarriage',
            executable='undercarriage_node',
            name='undercarriage_node',
        )
    ])

