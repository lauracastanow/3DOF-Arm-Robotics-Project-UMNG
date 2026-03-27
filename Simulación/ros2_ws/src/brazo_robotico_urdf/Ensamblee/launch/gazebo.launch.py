import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'Ensamblee'
    urdf_file = 'Ensamblee.urdf'

    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file)

    # Configurar rutas de recursos para Gazebo
    resource_paths = [os.path.join(pkg_share, '..'), pkg_share]
    current_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    new_gz_path = ":".join(resource_paths)
    os.environ["GZ_SIM_RESOURCE_PATH"] = current_gz_path + ":" + new_gz_path if current_gz_path else new_gz_path

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 1. Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 2. Gazebo Sim (Mundo vacío)
    # Quitamos '-r' para que inicie pausado y el robot se estabilice al dar Play
    gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -v 4'}.items()
    )

    # 3. Spawn del Robot con Posiciones Iniciales (HOME)
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'brazo_laura', 
            '-string', robot_desc, 
            '-z', '0.05',
            '-initial_joint_positions', 'joint_1', '0.0', 'joint_2', '0.0', 'joint_3', '0.0'
        ],
        output='screen'
    )

    # 4. Puente (Bridge) Automático
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/brazo_laura/joint/joint_1/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/brazo_laura/joint/joint_2/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/brazo_laura/joint/joint_3/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/brazo_laura/joint/joint_4/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/brazo_laura/joint/joint_5/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/brazo_laura/joint/joint_6/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
        ],
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        gz_sim,
        spawn_node,
        bridge_node 
    ])
