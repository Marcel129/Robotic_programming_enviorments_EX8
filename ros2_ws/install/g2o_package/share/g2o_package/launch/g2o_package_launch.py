from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    src_dir = '/home/marcel/studies/2_sem_mgr/RPE_lab/EX8/ros2_ws/src/g2o_package/config'
    return LaunchDescription([
        Node(
            package='g2o_package',
            executable='g2o_node'
        )        
    ])

# ,
#         Node(
#             package='simulation_package',
#             executable='robot_simulator',
#             parameters=[os.path.join(
#                 src_dir,
#                 'simulator.yaml'
#             )]
#         )