from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

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
                    plugin='PublisherNode',
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

    return LaunchDescription([container,
                              #container_2
                            ])