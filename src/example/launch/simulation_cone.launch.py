import os
import shutil
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_better_lidar = get_package_share_directory('better_lidar')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_better_lidar, 'worlds', 'cone_world.sdf')
    rviz_config = os.path.join(pkg_better_lidar, 'config', 'lidar.rviz')
    models_dir = os.path.join(pkg_better_lidar, 'models')

    plugin_lib_dir = os.path.normpath(
        os.path.join(pkg_better_lidar, '..', '..', 'lib', 'better_lidar'))

    def _process_xacro_model(name):
        model_dir = os.path.join(models_dir, name)
        xacro_path = os.path.join(pkg_better_lidar, 'urdf', f'{name}.xacro')
        os.makedirs(model_dir, exist_ok=True)
        subprocess.run(
            ['xacro', xacro_path, '-o', os.path.join(model_dir, 'model.sdf')],
            check=True,
        )
        with open(os.path.join(model_dir, 'model.config'), 'w') as f:
            f.write('<?xml version="1.0"?>\n')
            f.write('<model>\n')
            f.write(f'  <name>{name}</name>\n')
            f.write('  <version>1.0</version>\n')
            f.write('  <sdf version="1.6">model.sdf</sdf>\n')
            f.write('</model>\n')

    def _copy_preset_model(name):
        src = os.path.join(pkg_better_lidar, 'models', name)
        dst = os.path.join(models_dir, name)
        if os.path.isdir(src) and not os.path.isdir(dst):
            shutil.copytree(src, dst)

    _process_xacro_model('AT128P')
    _copy_preset_model('blue_cone')
    _copy_preset_model('yellow_cone')
    _copy_preset_model('cone_map')

    set_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=plugin_lib_dir)

    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_dir)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar/points@'
            'sensor_msgs/msg/PointCloud2'
            '[gz.msgs.PointCloudPacked',
        ],
        output='screen',
        name='lidar_points_bridge',
    )

    tf_static = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.5',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'AT128P/base_link',
        ],
        output='screen',
        name='tf_world_to_sensor',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        name='rviz2',
    )

    return LaunchDescription([
        set_plugin_path,
        set_model_path,
        gz_sim,
        bridge,
        tf_static,
        rviz,
    ])