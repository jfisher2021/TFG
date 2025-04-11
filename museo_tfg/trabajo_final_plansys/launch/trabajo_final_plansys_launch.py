import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

pkg_name = 'trabajo_final_plansys'

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory(pkg_name)
    kobuki_dir = get_package_share_directory('kobuki')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kobuki_dir, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(
                      get_package_share_directory('aws_robomaker_bookstore_world'),
                      'worlds',
                      'bookstore.world')
        }.items()
    )

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

    # Specify the actions
    move_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move',
        namespace=namespace,
        output='screen',
        parameters=[
          pkg_dir + '/config/params.yaml',
          {
            'action_name': 'move',
            'publisher_port': 11,
            'server_port': 12,
            'bt_xml_file': pkg_dir + '/behavior_trees_xml/move.xml'
          }
        ])

    attend_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='attend_visitors',
        namespace=namespace,
        output='screen',
        parameters=[
          pkg_dir + '/config/params.yaml',
          {
            'action_name': 'attend_visitors',
            'publisher_port': 13,
            'server_port': 14,
            'bt_xml_file': pkg_dir + '/behavior_trees_xml/attend_visitors.xml'
          }
        ])
    
    solve_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='solve_rubik',
        namespace=namespace,
        output='screen',
        parameters=[
          pkg_dir + '/config/params.yaml',
          {
            'action_name': 'solve_rubik',
            'publisher_port': 15,
            'server_port': 16,
            'bt_xml_file': pkg_dir + '/behavior_trees_xml/solve_rubik.xml'
          }
        ])
    
    search_book = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='search_book',
        namespace=namespace,
        output='screen',
        parameters=[
          pkg_dir + '/config/params.yaml',
          {
            'action_name': 'search_book',
            'publisher_port': 17,
            'server_port': 18,
            'bt_xml_file': pkg_dir + '/behavior_trees_xml/search_book.xml'
          }
        ])
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(simulation_cmd)
    ld.add_action(plansys2_cmd)

    ld.add_action(move_cmd)
    ld.add_action(attend_cmd)
    ld.add_action(solve_cmd)
    ld.add_action(search_book)

    return ld