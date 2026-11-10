#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robotis_controller_msgs.srv import SetModule


def main():
    rclpy.init()
    node = Node('enable_walk_module')
    client = node.create_client(SetModule, '/robotis/set_present_ctrl_modules')
    
    node.get_logger().info('Waiting for /robotis/set_present_ctrl_modules service...')
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error('Service not available after 10 seconds')
        rclpy.shutdown()
        return 1
    
    req = SetModule.Request()
    req.module_name = 'altair_walk_module'
    
    node.get_logger().info('Calling service to enable altair_walk_module...')
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        node.get_logger().info(f'Module enabled: result={future.result().result}')
    else:
        node.get_logger().error('Service call failed')
        rclpy.shutdown()
        return 1
    
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())