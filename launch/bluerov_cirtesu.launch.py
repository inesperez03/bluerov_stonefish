from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Buscamos la ruta de stonefish_ros2 dinámicamente
    stonefish_simulator_launch = PathJoinSubstitution([
        FindPackageShare('stonefish_ros2'),
        'launch',
        'stonefish_simulator.launch.py'
    ])

    # Configuramos solo la acción de la simulación
    simulator_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stonefish_simulator_launch),
        launch_arguments={
            'simulation_data': PathJoinSubstitution([
                FindPackageShare('bluerov_stonefish'), 'data'
            ]),
            'scenario_desc': PathJoinSubstitution([
                FindPackageShare('bluerov_stonefish'), 'scenarios', 'bluerov_cirtesu_arucos.scn'
            ]),
            'simulation_rate': '50.0',
            'window_res_x': '1200',
            'window_res_y': '800',
            'rendering_quality': 'high'
        }.items()
    )

    return LaunchDescription([
        simulator_action
    ])
