import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_in_model = 'fishbot_four_wheel'
    package_name = 'fishbot_description'
    xacro_name = 'fishbot_gazebo_four_wheeled_car.urdf.xacro'

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    xacro_model_path = os.path.join(pkg_share, f'urdf/{xacro_name}')
    controllers_yaml_path = os.path.join(pkg_share, 'config/four_wheel_controllers.yaml')
    gazebo_world_path = os.path.join(pkg_share, 'world/fishbot_big.world')

    robot_description_content = Command([
        'xacro ',
        xacro_model_path,
        ' ',
        'controllers_file:=',
        controllers_yaml_path,
    ])

    start_gazebo_cmd = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            gazebo_world_path,
        ],
        output='screen')

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.02',
            '-Y', '0.0',
        ],
        output='screen'
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }]
    )

    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            ('depth', '/camera/rgbd_camera/depth/image_raw'),
            ('depth_camera_info', '/camera/rgbd_camera/depth/camera_info'),
            ('scan', '/camera/depth/scan'),
        ],
        parameters=[{
            'output_frame': 'camera_link',
            'range_min': 0.2,
            'range_max': 10.0,
            'scan_time': 0.033,
            'scan_height': 30,
        }]
    )

    four_wheel_odom_node = Node(
        package='fishbot_four_wheel_controller',
        executable='four_wheel_closed_loop_odom',
        name='four_wheel_closed_loop_odom',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'wheel_radius': 0.06,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_footprint',
            'joint_states_topic': '/joint_states',
            'publish_tf': True,
            'joint_state_timeout_sec': 0.2,

            'front_left_x': 0.225,
            'front_left_y': 0.21,
            'front_right_x': 0.225,
            'front_right_y': -0.21,
            'rear_left_x': -0.225,
            'rear_left_y': 0.21,
            'rear_right_x': -0.225,
            'rear_right_y': -0.21,

            'front_left_wheel_dir_sign': 1.0,
            'front_right_wheel_dir_sign': 1.0,
            'rear_left_wheel_dir_sign': 1.0,
            'rear_right_wheel_dir_sign': 1.0,
        }]
    )
    
    four_wheel_controller_node = Node(
        package='fishbot_four_wheel_controller',
        executable='four_wheel_commander',
        output='screen',
        parameters=[{
            'wheel_separation': 0.42,
            'wheel_base': 0.45,
            'wheel_radius': 0.06,
            'wheel_steering_y_offset': 0.0,
            'cmd_vel_timeout_sec': 0.0,
            'joy_timeout_sec': 0.2,
            'joy_linear_x_gain': 1.0,
            'joy_linear_y_gain': 1.0,
            'joy_angular_z_gain': 1.0,
        }]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'forward_position_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'forward_velocity_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    load_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity_cmd,
            on_exit=[
                joint_state_broadcaster_spawner,
                position_controller_spawner,
                velocity_controller_spawner,
            ],
        )
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(depth_to_scan_node)
    ld.add_action(four_wheel_odom_node)
    ld.add_action(four_wheel_controller_node)
    ld.add_action(load_controllers_after_spawn)
    ld.add_action(start_rviz_cmd)

    return ld
