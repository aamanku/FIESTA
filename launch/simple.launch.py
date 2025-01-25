from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    fiesta_node = Node(
            package='fiesta_pkg',
            executable='fiesta',
            name='fiesta_node',
            output='screen',
            remappings=[
                ('transform', '/your_transform'),
                ('depth', '/your_depth'),
            ]
        )
    
    return LaunchDescription([fiesta_node])