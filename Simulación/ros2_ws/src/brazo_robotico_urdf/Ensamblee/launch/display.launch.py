import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Nombre de tu paquete
    package_name = 'Ensamblee'
    # IMPORTANTE: Revisa que este sea el nombre exacto de tu archivo dentro de la carpeta urdf/
    urdf_file = 'Ensamblee.urdf' 

    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Nodo que lee la cinemática de tu robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        # Nodo que crea la ventanita con los deslizadores para mover los motores
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        # Nodo que abre el simulador visual 3D
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
