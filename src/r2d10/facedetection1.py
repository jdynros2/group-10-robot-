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

        #directory for snapshots
        if self.save_snapshots and not os.path.exists(self.snapshot_dir):
            os.makedirs(self.snapshot_dir, exist_ok=True)
            self.get_logger().info(f'Created snapshot directory: {self.snapshot_dir}')

        # For locating the Haar cascade within the r2d10 package 
        pkg_dir = os.path.dirname(__file__)
        cascade_path = os.path.join(pkg_dir, '..', '..', 'share', 'r2d10', 'data', 'haarcascade_frontalface_default.xml')
        cascade_path = os.path.realpath(cascade_path)

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
                self.get_logger().info(f'Using Haar cascade: {cascade_path}')
                
        self.bridge = CvBridge() #Cv bridge 
        
        # Subscriber is the raw camera image
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)

        # Publisher is the annotated image with boxes
        self.face_image_pub = self.create_publisher(Image, '/camera/face_image', 10)

        self.get_logger().info(f'FaceDetector node started. Subscribing to {image_topic}')

    def image_callback(self, msg: Image):
        self.get_logger().info('image_callback called')
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # face detection
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5, minSize=(30, 30))

        self.get_logger().info(f'Faces array: {faces}')

        if len(faces) > 0:
            self.get_logger().info(f'Detected {len(faces)} face(s)')

            # Draw rectangles
            for (x, y, w, h) in faces:
                cv_image = cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # saving the snapshot
            if self.save_snapshots:
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                filename = os.path.join(self.snapshot_dir, f'face_{timestamp}.png')
                try:
                    cv2.imwrite(filename, cv_image)
                    self.get_logger().info(f'Snapshot saved: {filename}')
                except Exception as e:
                    self.get_logger().warn(f'Failed to save snapshot: {e}')
        else:
            self.get_logger().info('No faces detected in this frame')

        # to publish the annotated images
        try:
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            out_msg.header = msg.header  # keep timestamp/frame_id
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


