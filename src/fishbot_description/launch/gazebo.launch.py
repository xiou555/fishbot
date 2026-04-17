
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_in_model = 'fishbot'
    package_name = 'fishbot_description'
    urdf_name = "fishbot_gazebo.urdf"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    gazebo_world_path = os.path.join(pkg_share, 'world/fishbot_big.world')

    # Start Gazebo server
    # start_gazebo_cmd = ExecuteProcess(
    #     cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
    #     output='screen')
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        output='screen')
        
    # Launch the robot
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,
            '-file', urdf_model_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.02',
            '-Y', '0.0'
        ],
        output='screen'
    )
	
    # Start Robot State publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
    )

    # 深度图像转雷达数据
    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            # 输入：深度图像 & 相机内参（修正为 RGBD 相机的实际话题）
            ('depth', '/camera/rgbd_camera/depth/image_raw'),
            ('depth_camera_info', '/camera/rgbd_camera/depth/camera_info'),
            # 输出：LaserScan
            ('scan', '/camera/depth/scan'),
        ],
        parameters=[{
            # 输出的 LaserScan 使用的坐标系
            'output_frame': 'camera_link',
            # 和你深度相机的 near/far 大致保持一致即可
            'range_min': 0.2,
            'range_max': 10.0,
            # 扫描时间（帧率 30Hz ≈ 0.033）
            'scan_time': 0.033,
            # 从图像中取多少行来生成 scan（中间的行，避免上下边缘）
            'scan_height': 30,
        }]
    )

    # Launch RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', default_rviz_config_path]
        )

    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(depth_to_scan_node)
    ld.add_action(start_rviz_cmd)


    return ld