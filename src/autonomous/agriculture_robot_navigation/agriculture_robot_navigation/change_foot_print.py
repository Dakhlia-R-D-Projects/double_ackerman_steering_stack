import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
def main(args=None):
    rclpy.init(args=args)
    
    # Create a dummy node to initialize the parameter client
    node = Node('footprint_changer_node')
    
    # Define the target nodes for the costmaps
    # Note: Your node names might be different! 
    # Use `ros2 node list` to find the correct names for your costmap nodes.
    target_nodes = [
        'local_costmap/local_costmap', 
        'global_costmap/global_costmap'
    ]

    # Define the new footprint as a string in YAML format
    # This example defines a simple 0.5m x 0.3m rectangular footprint
    new_footprint_string = '[[1.25, 1.15], [1.25, -1.15], [-1.25, -1.15], [-1.25, 1.15]]'

    for target_node_name in target_nodes:
        # Create a synchronous parameter client
        param_client = AsyncParameterClient(node, target_node_name)
        
        # Wait for the parameter service to become available
        if not param_client.wait_for_services(timeout_sec=2.0):
            node.get_logger().error(f"Parameter service for node '{target_node_name}' not available.")
            continue

        # Define the new parameter
        new_param = Parameter('footprint', Parameter.Type.STRING, new_footprint_string)

        # Set the new parameter
        # set_parameters_atomically is used to set one or more parameters at once
        result = param_client.set_parameters_atomically([new_param])

        if result.successful:
            node.get_logger().info(f"Successfully set footprint for node: '{target_node_name}'")
        else:
            node.get_logger().error(f"Failed to set footprint for node '{target_node_name}': {result.reason}")

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()