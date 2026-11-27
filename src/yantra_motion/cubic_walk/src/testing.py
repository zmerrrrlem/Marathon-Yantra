import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter


class ParameterClient(Node):
    def __init__(self):
        super().__init__('parameter_client')
        self.target_node_name = 'turtlesim'  # Replace with the actual target node name
        self.param_name = 'background_b'
        self.param_value = 100  # Adjust value based on the parameter's expected type
        self.param_type = Parameter.Type.INTEGER  # Adjust type accordingly

        # Create a client for the 'set_parameters' service
        self.client = self.create_client(SetParameters, f'/{self.target_node_name}/set_parameters')

        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f"Waiting for {self.target_node_name}'s set_parameters service...")

        self.set_parameter()

    def set_parameter(self):
        # Create the request with the desired parameter
        param_msg = Parameter(
            name=self.param_name,
            value=self.param_value,
            type_=self.param_type
        ).to_parameter_msg()

        request = SetParameters.Request()
        request.parameters = [param_msg]

        # Call the service asynchronously
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            successful = all(result.successful for result in response.results)
            if successful:
                self.get_logger().info(f"Parameter '{self.param_name}' successfully set to '{self.param_value}' on '{self.target_node_name}'.")
            else:
                self.get_logger().error(f"Failed to set parameter '{self.param_name}' on '{self.target_node_name}'.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ParameterClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
