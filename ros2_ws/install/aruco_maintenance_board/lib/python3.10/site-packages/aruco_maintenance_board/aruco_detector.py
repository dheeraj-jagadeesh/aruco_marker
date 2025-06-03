#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
from ament_index_python.packages import get_package_share_directory

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Load configuration
        self.load_config()
        
        # Create publishers and subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.processed_image_pub = self.create_publisher(
            Image, '/aruco/processed_image', 10)
        self.detection_status_pub = self.create_publisher(
            String, '/aruco/detection_status', 10)
        
        self.get_logger().info('ArUco Detector Node initialized')
        self.get_logger().info(f'Using ArUco dictionary: {self.aruco_dict_name}')

    def load_config(self):
        """Load configuration from JSON file"""
        try:
            package_share_dir = get_package_share_directory('aruco_maintenance_board')
            config_path = os.path.join(package_share_dir, 'config', 'config.json')
            
            with open(config_path, 'r') as f:
                config = json.load(f)
                
            self.board_width = config['board_width']
            self.board_height = config['board_height']
            self.ref_board_height = config['reference_board_height']
            self.ref_square_fraction = config['reference_square_fraction']
            self.min_square_size = config['min_square_size']
            self.aruco_dict_name = config['aruco_dict']
            self.object_relative_positions = config['object_relative_positions']
            
            # ArUco dictionary mapping
            self.ARUCO_DICTS = {
                "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
                "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
                "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
                "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
                "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
                "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
                "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
                "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
                "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
                "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
                "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
                "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
                "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
                "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
                "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
                "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
                "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
                "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
                "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
                "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
                "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
            }
            
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {str(e)}')
            # Use default values
            self.board_width = 600
            self.board_height = 800
            self.ref_board_height = 400
            self.ref_square_fraction = 0.45
            self.min_square_size = 20
            self.aruco_dict_name = "DICT_4X4_50"
            self.object_relative_positions = [
                [0.17, 0.17], [0.5, 0.17], [0.83, 0.17],
                [0.17, 0.5], [0.83, 0.5],
                [0.17, 0.83], [0.5, 0.83], [0.83, 0.83]
            ]

    def detect_aruco_tags(self, frame):
        """Detect ArUco tags in the frame"""
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICTS[self.aruco_dict_name])
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        corners, ids, _ = detector.detectMarkers(frame)
        
        if ids is not None and len(ids) >= 3:
            return corners, ids
        return None, None

    def order_corners(self, corners):
        """Order corners for consistent board detection"""
        centers = [np.mean(corner[0], axis=0) for corner in corners]
        centers = np.array(centers)
        sum_coords = centers.sum(axis=1)
        diff_coords = centers[:, 0] - centers[:, 1]
        
        top_left_idx = np.argmin(sum_coords)
        top_right_idx = np.argmax(diff_coords)
        bottom_left_idx = np.argmax(centers[:,1])
        
        return [corners[top_left_idx], corners[top_right_idx], corners[bottom_left_idx]]

    def get_board_homography(self, corners):
        """Calculate homography matrix for board transformation"""
        src_pts = np.array([np.mean(corner[0], axis=0) for corner in corners], dtype=np.float32)
        tl, tr, bl = src_pts
        br = tr + (bl - tl)
        src = np.array([tl, tr, br, bl], dtype=np.float32)
        dst = np.array([[0, 0], [self.board_width, 0], 
                       [self.board_width, self.board_height], [0, self.board_height]], dtype=np.float32)
        H = cv2.getPerspectiveTransform(dst, src)
        return H

    def get_scaled_square_size(self, corners):
        """Calculate scaled square size based on board dimensions"""
        pts = [np.mean(corner[0], axis=0) for corner in corners]
        board_height = np.linalg.norm(pts[0] - pts[2])
        board_width = np.linalg.norm(pts[0] - pts[1])
        base_square_size = int(min(board_width, board_height) * self.ref_square_fraction)
        base_square_size = max(self.min_square_size, base_square_size)
        scale_factor = board_height / self.ref_board_height
        scaled_square_size = int(base_square_size * scale_factor)
        scaled_square_size = max(self.min_square_size, scaled_square_size)
        return scaled_square_size

    def draw_aligned_square_homography(self, img, center_board, square_size, H, color, thickness=2):
        """Draw aligned square using homography transformation"""
        half = square_size / 2
        board_pts = np.array([
            [center_board[0] - half, center_board[1] - half],
            [center_board[0] + half, center_board[1] - half],
            [center_board[0] + half, center_board[1] + half],
            [center_board[0] - half, center_board[1] + half]
        ], dtype=np.float32).reshape(-1,1,2)
        img_pts = cv2.perspectiveTransform(board_pts, H)
        img_pts = img_pts.astype(int)
        cv2.polylines(img, [img_pts], isClosed=True, color=color, thickness=thickness)

    def image_callback(self, msg):
        """Process incoming image messages"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Detect ArUco tags
            corners, ids = self.detect_aruco_tags(cv_image)
            display = cv_image.copy()
            
            status_msg = String()
            
            if corners is not None and len(corners) >= 3:
                # Process detected tags
                three_corners = self.order_corners(corners[:3])
                square_size = self.get_scaled_square_size(three_corners)
                H = self.get_board_homography(three_corners)
                
                # Draw tag positions
                tag_board_coords = [
                    (0, 0), (self.board_width, 0), (0, self.board_height)
                ]
                for pt in tag_board_coords:
                    self.draw_aligned_square_homography(display, pt, square_size, H, (0,255,0), thickness=2)
                
                # Draw board outline
                board_outline = np.array([
                    [0,0],[self.board_width,0],[self.board_width,self.board_height],[0,self.board_height]
                ], dtype=np.float32).reshape(-1,1,2)
                img_outline = cv2.perspectiveTransform(board_outline, H).astype(int)
                cv2.polylines(display, [img_outline], isClosed=True, color=(255,255,0), thickness=2)
                
                # Draw object positions
                for rel_x, rel_y in self.object_relative_positions:
                    board_x = rel_x * self.board_width
                    board_y = rel_y * self.board_height
                    self.draw_aligned_square_homography(display, (board_x, board_y), square_size, H, (0,0,255), thickness=2)
                
                cv2.putText(display, f"Dict: {self.aruco_dict_name}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display, f"Tags detected: {len(ids)}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                status_msg.data = f"DETECTED: {len(ids)} tags found"
                
            else:
                cv2.putText(display, "Waiting for 3 ArUco tags...", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                status_msg.data = "WAITING: Need 3 ArUco tags"
            
            # Publish processed image and status
            processed_msg = self.bridge.cv2_to_imgmsg(display, 'bgr8')
            processed_msg.header = msg.header
            self.processed_image_pub.publish(processed_msg)
            self.detection_status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()