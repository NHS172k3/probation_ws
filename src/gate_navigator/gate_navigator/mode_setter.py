#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode
import sys

def set_guided_mode():
    rclpy.init()
    node = Node('mode_setter_temp')
    
    client = node.create_client(SetMode, '/mavros/set_mode')
    
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Set mode service not available')
        return False
    
    request = SetMode.Request()
    request.custom_mode = 'GUIDED'
    
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    
    if future.result() is not None and future.result().mode_sent:
        node.get_logger().info('GUIDED mode set successfully')
        node.destroy_node()
        rclpy.shutdown()
        return True
    else:
        node.get_logger().error('Failed to set GUIDED mode')
        node.destroy_node()
        rclpy.shutdown()
        return False

def main():
    success = set_guided_mode()
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()