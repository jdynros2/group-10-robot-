#!/usr/bin/env python3
import os
from datetime import datetime
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class FaceDetector(Node):
    def __init__(self):
        super().__init__('face_detector')

        self.get_logger().info('FaceDetector __init__ entered')

        # Parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('save_snapshots', True)
        self.declare_parameter('snapshot_dir', 'face_snapshots')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.save_snapshots = self.get_parameter('save_snapshots').get_parameter_value().bool_value
        self.snapshot_dir = self.get_parameter('snapshot_dir').get_parameter_value().string_value

        # IMPORTANT: Track if we've already taken a snapshot
        self.snapshot_taken = False

        # Directory for snapshots
        if self.save_snapshots and not os.path.exists(self.snapshot_dir):
            os.makedirs(self.snapshot_dir, exist_ok=True)
            self.get_logger().info(f'Created snapshot directory: {self.snapshot_dir}')

        # For locating the Haar cascade within the r2d10 package 
        pkg_dir = os.path.dirname(__file__)
        cascade_path = os.path.join(pkg_dir, 'data', 'haarcascade_frontalface_default.xml')
        
        # Try multiple locations
        if not os.path.isfile(cascade_path):
            cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
        
        if not os.path.isfile(cascade_path):
            cascade_path = '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml'

        self.get_logger().info(f'Using Haar cascade: {cascade_path}')

        if not os.path.isfile(cascade_path):
            self.get_logger().error(f'Haar cascade not found at: {cascade_path}')
            self.face_cascade = None
        else:
            self.face_cascade = cv2.CascadeClassifier(cascade_path)
            
            if self.face_cascade.empty():
                self.get_logger().error('Failed to load Haar cascade. Face detection will not work.')
                self.face_cascade = None
            else:
                self.get_logger().info('Haar cascade loaded successfully!')
                
        self.bridge = CvBridge()
        
        # Subscriber to raw camera image
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)

        # Publisher for annotated image with boxes
        self.face_image_pub = self.create_publisher(Image, '/camera/face_image', 10)

        self.get_logger().info(f'FaceDetector node started. Subscribing to {image_topic}')

    def image_callback(self, msg: Image):
        # Don't log every callback - too much spam
        
        if self.face_cascade is None:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Face detection
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        if len(faces) > 0 and not self.snapshot_taken:
            self.get_logger().info(f'✓ FACE DETECTED! Found {len(faces)} face(s)')

            # Draw rectangles
            for (x, y, w, h) in faces:
                cv_image = cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # Add text
                cv2.putText(cv_image, 'FACE DETECTED', (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Save snapshot ONCE
            if self.save_snapshots:
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                filename = os.path.join(self.snapshot_dir, f'face_capture_{timestamp}.png')
                try:
                    cv2.imwrite(filename, cv_image)
                    self.get_logger().info(f'✓ SNAPSHOT SAVED: {filename}')
                    self.snapshot_taken = True  # Never take another snapshot
                except Exception as e:
                    self.get_logger().warn(f'Failed to save snapshot: {e}')
                    
        elif self.snapshot_taken:
            # Add completion message to image
            cv2.putText(cv_image, 'SNAPSHOT COMPLETE', (50, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Publish the annotated image
        try:
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            out_msg.header = msg.header
            self.face_image_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish annotated image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
