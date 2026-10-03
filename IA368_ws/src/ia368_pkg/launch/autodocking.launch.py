"""Sobe a ponte ROS 2 <-> CoppeliaSim e o nó de docking autônomo.

    ros2 launch ia368_pkg autodocking.launch.py

Equivale a rodar o remoteAPI_ROS2_bridge.launch.py (battery_node,
bumper_and_velocity_node, charging_base_node e docking_node) e, junto, o
autodocking_node, que é o controlador da atividade.

Para subir só a ponte e rodar o controlador à mão (útil para depurar com
`ros2 topic echo /myRobot/cmd_vel`), use:

    ros2 launch ia368_pkg remoteAPI_ROS2_bridge.launch.py
    ros2 run ia368_pkg autodocking_node

Argumentos:

    auto_dock:=false    não liga o docking sozinho com a bateria baixa (só
                        pelo checkbox "docking" da cena)
    battery_low:=70.0   % de bateria em que o docking liga sozinho
    clean:=false        com o docking OFF, não limpa: deixa a teleoperação
    angle_sign:=-1.0    inverte o sentido do giro, se o robô virar para o
                        lado errado ao seguir o beacon
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    auto_dock = LaunchConfiguration('auto_dock')
    clean = LaunchConfiguration('clean')
    battery_low = LaunchConfiguration('battery_low')
    angle_sign = LaunchConfiguration('angle_sign')

    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_dock', default_value='true',
            description='liga o docking sozinho quando a bateria chega a battery_low'),
        DeclareLaunchArgument(
            'clean', default_value='true',
            description='com o docking OFF, limpa (bate e volta); false = teleoperação'),
        DeclareLaunchArgument(
            'battery_low', default_value='70.0',
            description='nível de bateria (%) que liga o modo de docking'),
        DeclareLaunchArgument(
            'angle_sign', default_value='1.0',
            description='sentido do relativeAngle (-1.0 inverte o giro)'),

        # A mesma ponte da atividade anterior, sem alteração.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ia368_pkg'),
                    'launch',
                    'remoteAPI_ROS2_bridge.launch.py'
                ])
            ])
        ),

        # O controlador: só fala ROS 2, não conhece o CoppeliaSim.
        Node(
            package='ia368_pkg',
            executable='autodocking_node',
            output='screen',
            parameters=[{
                'auto_dock_on_low_battery': auto_dock,
                'clean_when_idle': clean,
                'battery_low': battery_low,
                'angle_sign': angle_sign,
            }]
        ),
    ])
