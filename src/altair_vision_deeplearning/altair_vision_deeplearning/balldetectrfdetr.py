import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point # This to broadcast the ball's X, Y, and Distance

from cv_bridge import CvBridge
import cv2
import math
import supervision as sv

from rfdetr import RFDETRNano

class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector')
        
        self.bridge = CvBridge()
        
        # Change '/camera/image_raw' to whatever camera topic is named
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )
        
        # Create Publishers
        self.image_pub = self.create_publisher(Image, '/vision/annotated_frame', qos_profile_sensor_data) # Broadcasts the annotated video to be watched in RViz or rqt_image_view
        self.ball_pos_pub = self.create_publisher(Point, '/vision/ball_position', 10) # Broadcasts the actual coordinate data for the robot's brain to use
        
        # Initialize Model and Tools
        self.get_logger().info("Loading RF-DETR model...")
        try:
            self.model = RFDETRNano(pretrain_weights="balldetectweights.pth")
        except Exception as e:
            self.get_logger().warn(f"Could not load custom weights, falling back to base. {e}")
            self.model = RFDETRNano()
            
        self.model.optimize_for_inference()
        self.cal_const = 10000.0 # TS calibration constant for camera to guess distance to ball
        
        self.box_annotator = sv.BoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()
        
        self.get_logger().info("Ball Detector Node is actively watching...")

    def image_callback(self, msg):
        """This function runs automatically every time a new camera frame arrives."""
        # self.get_logger().info("Frame received") # To debug whether a frame is received or nah
        try:
            # Convert the ROS Image message into an OpenCV (BGR) image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        frame_h, frame_w, _ = frame.shape
        frame_center = (int(frame_w / 2), int(frame_h / 2))
        
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        detections = self.model.predict(rgb_frame, threshold=0.5)

        annotated_frame = frame.copy()
        custom_labels = []
        
        best_ball_data = None
        highest_conf = 0.0

        for i in range(len(detections)):
            x_min, y_min, x_max, y_max = detections.xyxy[i]
            confidence = detections.confidence[i]
            class_id = detections.class_id[i]

            w = x_max - x_min
            h = y_max - y_min
            
            center_x = int(x_min + w / 2)
            center_y = int(y_min + h / 2)
            
            diagonal = math.sqrt(w**2 + h**2)
            distance = self.cal_const / diagonal if diagonal > 0 else 0.0
            
            custom_labels.append(f"Class {class_id} {confidence:.2f} | Dist: {distance:.2f}")

            dx = center_x - frame_center[0]
            dy = center_y - frame_center[1]

            if confidence > highest_conf:
                highest_conf = confidence
                best_ball_data = (dx, dy, distance) # Save the best detection to publish to ROS
                
            cv2.circle(annotated_frame, (center_x, center_y), radius=5, color=(0, 0, 255), thickness=-1)
            cv2.line(annotated_frame, frame_center, (center_x, center_y), (0, 255, 255), 2)

        annotated_frame = self.box_annotator.annotate(scene=annotated_frame, detections=detections)
        annotated_frame = self.label_annotator.annotate(scene=annotated_frame, detections=detections, labels=custom_labels)
        
        # Publish the debug image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            annotated_msg.header = msg.header
            self.image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")

        # Publish the ball's coordinates if a ball was found
        if best_ball_data is not None:
            dx, dy, dist = best_ball_data
            
            point_msg = Point()
            point_msg.x = float(dx)       # X offset from center
            point_msg.y = float(dy)       # Y offset from center
            point_msg.z = float(dist)     # Estimated distance
            
            self.ball_pos_pub.publish(point_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    
    try:
        rclpy.spin(node) # This keeps the node running and listening for images
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
