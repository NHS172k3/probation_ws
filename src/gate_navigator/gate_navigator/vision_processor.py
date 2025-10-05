import rclpy
from rclpy.node import Node
from vision_msgs.msg import BoundingBoxArray

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')
        
        self.subscription = self.create_subscription(
            BoundingBoxArray,
            '/main_camera/detection/bounding_boxes',
            self.bbox_callback,
            10
        )
        
        # Gate detection data
        self.gate_detected = False
        self.gate_center_x = 0.5
        self.gate_center_y = 0.5
        self.gate_width = 0.0
        self.gate_height = 0.0
        self.last_detection_time = self.get_clock().now()
        
        self.get_logger().info('Vision processor started!')
        
    def bbox_callback(self, msg):
        """Process bounding box detections"""
        print("tRYING TO CALLBACK!")
        self.gate_detected = False
        
        for bbox in msg.bounding_boxes:
            # Match by label_id (more reliable) or label name
            if bbox.label_id == 3 or bbox.label_name == 'gate':
                self.gate_detected = True
                self.last_detection_time = self.get_clock().now()
                
                self.gate_center_x = bbox.x
                self.gate_center_y = bbox.y
                self.gate_width = bbox.w
                self.gate_height = bbox.h
                
                self.get_logger().info(
                    f'✓ Gate detected! Center: ({self.gate_center_x:.2f}, {self.gate_center_y:.2f}), '
                    f'Size: {self.gate_width:.2f}x{self.gate_height:.2f}',
                    throttle_duration_sec=1.0
                )
                break
                
    def is_gate_detected(self):
        """Check if gate is currently detected"""
        return self.gate_detected
    
    def get_gate_info(self):
        """Get gate position and size information"""
        return {
            'detected': self.gate_detected,
            'center_x': self.gate_center_x,
            'center_y': self.gate_center_y,
            'width': self.gate_width,
            'height': self.gate_height
        }

def main():
    rclpy.init()
    vision_processor = VisionProcessor()
    rclpy.spin(vision_processor)
    vision_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()