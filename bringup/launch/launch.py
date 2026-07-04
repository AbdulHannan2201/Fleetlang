from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Extract values from launch configurations
    num_robots_str = context.launch_configurations.get('num_robots', '4')
    num_robots = int(num_robots_str)
    
    allocator_type = context.launch_configurations.get('allocator_type', 'neighborhood_search')
    
    nodes = [
        # Semantic Map Node
        Node(
            package='semantic_map',
            executable='semantic_map_node',
            name='semantic_map_node',
            output='screen'
        ),
        # Instruction Parser Node
        Node(
            package='language',
            executable='instruction_parser_node',
            name='instruction_parser_node',
            output='screen'
        ),
        # Task Allocator Node
        Node(
            package='allocation',
            executable='task_allocator_node',
            name='task_allocator_node',
            output='screen',
            parameters=[{
                'num_robots': num_robots,
                'allocator_type': allocator_type
            }]
        ),
        # Fleet Monitor Node
        Node(
            package='monitor',
            executable='fleet_status_monitor_node',
            name='fleet_status_monitor_node',
            output='screen',
            parameters=[{
                'num_robots': num_robots
            }]
        ),
        # Warehouse Simulator Node
        Node(
            package='bringup',
            executable='warehouse_sim',
            name='warehouse_sim',
            output='screen',
            parameters=[{
                'num_robots': num_robots
            }]
        ),
        # Warehouse Visualizer Node
        Node(
            package='bringup',
            executable='warehouse_visualizer',
            name='warehouse_visualizer',
            output='screen',
            parameters=[{
                'num_robots': num_robots
            }]
        ),
    ]
    
    # Task Executor Node per robot
    for i in range(num_robots):
        rid = f"robot_{i}"
        node = Node(
            package='execution',
            executable='task_executor_node',
            name=f'task_executor_{rid}',
            output='screen',
            parameters=[{
                'robot_id': rid,
                'num_robots': num_robots
            }]
        )
        nodes.append(node)
        
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_robots',
            default_value='4',
            description='Number of robots to simulate'
        ),
        DeclareLaunchArgument(
            'allocator_type',
            default_value='neighborhood_search',
            description='Task allocator type: greedy or neighborhood_search'
        ),
        OpaqueFunction(function=launch_setup)
    ])
