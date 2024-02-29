import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_mapping_config_path = os.path.join(get_package_share_directory("my_mapping"),"rviz","mapper_lifelong_config.rviz")
    # rviz_nav_config_path = os.path.join(get_package_share_directory("mapping_basics"),"rviz","navigation_lifelong_config.rviz")
   
    return LaunchDescription(
        [
            Node(
                package="slam_toolbox",
                executable="lifelong_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    # get_package_share_directory(
                        os.path.join(
                            get_package_share_directory("my_mapping"),
                            "config",
                            "mapper_params_lifelong.yaml",
                        )
                    # )
                ],
                

            ),
             Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                parameters=[{"use_sim_time": True}],
                output="screen",
                arguments=["-d", rviz_mapping_config_path]
            ),
             
           
             
             
        ]
    )
