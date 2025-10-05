import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        
        self.vel_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )
        
        self.get_logger().info('Simple controller started')
        
    def move(self, x=0.0, y=0.0, z=0.0, yaw=0.0, duration=1.0):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.z = yaw
        
        self.get_logger().info(f'Moving: x={x}, y={y}, z={z}, yaw={yaw} for {duration}s')
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.vel_pub.publish(msg)
            time.sleep(0.1)
            
    def stop(self):
        self.move(0.0, 0.0, 0.0, 0.0, 0.1)

def main():
    rclpy.init()
    controller = SimpleController()
    
    # Test sequence
    controller.get_logger().info('Test 1: Move forward')
    controller.move(x=0.5, duration=2.0)
    controller.stop()
    
    time.sleep(1.0)
    
    controller.get_logger().info('Test 2: Move down')
    controller.move(z=-0.3, duration=2.0)
    controller.stop()
    
    time.sleep(1.0)
    
    controller.get_logger().info('Test 3: Rotate')
    controller.move(yaw=0.5, duration=2.0)
    controller.stop()
    
    controller.get_logger().info('Tests complete!')
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()