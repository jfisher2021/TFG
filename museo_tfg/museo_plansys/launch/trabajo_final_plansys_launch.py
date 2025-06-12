import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

pkg_name = 'museo_plansys'

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory(pkg_name)
    kobuki_dir = get_package_share_directory('kobuki')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    # simulation_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(kobuki_dir, 'launch', 'simulation.launch.py')
    #     ),
    #     launch_arguments={
    #         'world': os.path.join(
    #                   get_package_share_directory('aws_robomaker_bookstore_world'),
    #                   'worlds',
    #                   'bookstore.world')
    #     }.items()
    # )

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': pkg_dir + '/pddl/domain.pddl',
          'params_file': pkg_dir + '/config/bringup_params.yaml',
          'namespace': namespace
          }.items())

    #! REAL MOVE With Nav2
    # # Specify the actions
    # move_cmd = Node(
    #     package='plansys2_bt_actions',
    #     executable='bt_action_node',
    #     name='move',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[
    #       pkg_dir + '/config/params.yaml',
    #       {
    #         'action_name': 'move',
    #         'publisher_port': 11,
    #         'server_port': 12,
    #         'bt_xml_file': pkg_dir + '/behavior_trees_xml/move.xml'
    #       }
    #     ])
    # Specify the actions

    move_fake = Node(
        package='museo_plansys',
        executable='move_fake_action_node',
        name='move_fake_action_node',
        output='screen',
        parameters=[])
    
    recharge_cmd = Node(
        package='museo_plansys',
        executable='recharge_action_node',
        name='recharge_action_node',
        output='screen',
        parameters=[])
        
    explain_cmd = Node(
        package='museo_plansys',
        executable='explain_action_node',
        name='explain_action_node',
        output='screen',
        parameters=[])
    welcome_cmd = Node(
        package='museo_plansys',
        executable='welcome_action_node',
        name='welcome_action_node',
        output='screen',
        parameters=[])

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    # ld.add_action(simulation_cmd)
    ld.add_action(plansys2_cmd)
    ld.add_action(recharge_cmd)
    ld.add_action(welcome_cmd)
    ld.add_action(explain_cmd)
    ld.add_action(move_fake)

    # ld.add_action(move_cmd)

    return ld