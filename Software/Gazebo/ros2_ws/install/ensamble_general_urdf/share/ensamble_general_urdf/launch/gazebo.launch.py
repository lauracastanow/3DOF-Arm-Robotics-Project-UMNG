import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'ensamble_general_urdf'
    urdf_file = 'ensamble_general_urdf.urdf'

    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file)

    resource_paths = [os.path.join(pkg_share, '..'), pkg_share]
    os.environ["GZ_SIM_RESOURCE_PATH"] = ":".join(resource_paths)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -v 4'}.items()
    )

    # Nombre actualizado para coincidir con el nuevo paquete
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ensamble_general_urdf', 
            '-string', robot_desc, 
            '-z', '0.05'
        ],
        output='screen'
    )

    # Puente actualizado incluyendo los nuevos joints de la pinza (5 y 6)
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/ensamble_general_urdf/joint/joint_1/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/ensamble_general_urdf/joint/joint_2/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/ensamble_general_urdf/joint/joint_3/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/ensamble_general_urdf/joint/joint_5/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/ensamble_general_urdf/joint/joint_6/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
        ],
        output='screen'
    )

    return LaunchDescription([rsp_node, gz_sim, spawn_node, bridge_node])
