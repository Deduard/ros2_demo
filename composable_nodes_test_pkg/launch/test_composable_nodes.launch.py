from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import FindExecutable

def generate_launch_description():
    container = ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='composable_nodes_test_pkg',
                    plugin='PublisherNode',
                    name='publisher_node1',
                    parameters=[
                      {'publish_frequency': 30.0,
                       'topic': 'topic1'}
                    ],
                ),
                ComposableNode(
                    package='composable_nodes_test_pkg',
                    plugin='PublisherNode',
                    name='publisher_node2',
                    parameters=[
                      {'publish_frequency': 20.0,
                       'topic': 'topic2'}
                    ],
                ),
                ComposableNode(
                    package='composable_nodes_test_pkg',
                    plugin='PublisherLifecycleNode',
                    name='publisher_node3',
                    parameters=[
                      {'publish_frequency': 10.0,
                       'topic': 'topic3'}
                    ],
                ),
                ComposableNode(
                    package='composable_nodes_test_pkg',
                    plugin='SubscriberNode',
                    name='subscriber_node1',
                ),
                ComposableNode(
                    package='composable_nodes_test_pkg',
                    plugin='SubscriberNode',
                    name='subscriber_node2',
                ),
            ],
            output='screen',
    )
    
    # container_2 = ComposableNodeContainer(
    #         name='container_2',
    #         namespace='',
    #         package='rclcpp_components',
    #         executable='component_container_mt',
    #         composable_node_descriptions=[
    #                 ComposableNode(
    #                 package='composable_nodes_test_pkg',
    #                 plugin='SubscriberNode',
    #                 name='subscriber_node1',
    #             ),
    #             ComposableNode(
    #                 package='composable_nodes_test_pkg',
    #                 plugin='SubscriberNode',
    #                 name='subscriber_node2',
    #             ),
    #         ],
    #         output='screen',
    # )
    
    def invoke_lifecycle_cmd(node_name, verb):
        ros2_exec = FindExecutable(name='ros2')
        return ExecuteProcess(
            cmd=[[ros2_exec, ' lifecycle set ',
                  '', '/', node_name, ' ', verb]],
            shell=True)

    configure_cmd = invoke_lifecycle_cmd('publisher_node3', 'configure')
    activate_cmd = invoke_lifecycle_cmd('publisher_node3', 'activate')

    return LaunchDescription([container,
                              TimerAction(period=2.0, actions=[configure_cmd]),
                              TimerAction(period=3.0, actions=[activate_cmd])
                              #container_2
                            ])