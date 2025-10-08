#!/usr/bin/env python3

"""
Reads QR codes and stores the robots position (X,Y and Yaw) values along with the QRcode string

"""
# dependencies 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge   #This allows us to convert the ROS image message into OpenCV datatype
import cv2
import math
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException, Buffer, TransformListener
from tf_transformations import euler_from_quaternion # Helper for converting quaternion to yaw
import os
import json # Using JSON for file storage is often better than plain TXT


class QrCodeReader(Node):
    def __init__(self):
        super().__init__('qr_code_mazer_driver')
        self.get_logger().info('[qr_code_mazer_driver] Node initialized')

        # Customisable variables (depending on specific environment!)
        # ... (keep your existing variables) ...
        self.last_qr_code = "" 
        self.ideal_qr_measuring_distance = 0.45 #metres | amend as necessary

        # --- Coordinate Storage ---
        self.average_front_distance = 0.0 #Average of front 20 degrees of lidar readings to ensure robot is close enough to qr code to perform a read
        self.recorded_waypoints = {} # Dictionary to store: {qr_data: {'x': x, 'y': y, 'yaw': yaw}}
        self.qr_data_to_save = None  # Temporary storage for the QR data when a read occurs

        # --- TF2 setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_frame = 'map'
        self.robot_frame = 'base_footprint' 
        # 'base_footprint' should be the frame closest to the center of the robot.

        # Create publishers, subscribers, services, etc. here
        self.lidar_subscriber_ = self.create_subscription(LaserScan, "scan", self.callback_lidar_subscriber, 10)
        self.camera_subscriber_ = self.create_subscription(Image, "camera/image_raw", self.callback_camera_subscriber, 10)
        self.cv_bridge=CvBridge()
        
        # REMOVED: self.timer - The pose lookup should happen immediately after the QR code is read, not on a timer.
        # --- NEW: Shutdown hook for saving data ---
        # The node's on_shutdown hook will ensure data is saved when the node is shut down cleanly (e.g., Ctrl+C)
        # We handle this outside the __init__ in the main function.
        
        self.get_logger().info("Setup complete. Waiting for QR codes...")



    # ====================== callback functions ============================#

    def callback_lidar_subscriber(self, msg):
        #Subscribes to the Lidar scan data, calculates the average distance 
        #in the front 20-degree slice, and stores it in self.average_front_distance.

        # 1. Convert the tuple of ranges into a NumPy array
        lidar_data_array = np.array(msg.ranges)
        num_points = len(lidar_data_array) # 720
        # 2. Define the front 20-degree slice indices (10 degrees on each side of 0 degrees)
        # 0 to +10 degrees: Indices 0 to 19 (20 points)
        front_slice_1 = lidar_data_array[0:20] 
        # 350 to 360 degrees: Indices 700 to 719 (20 points)
        front_slice_2 = lidar_data_array[num_points - 20 : num_points] # [700:720]
        # 3. Combine the slices and filter the data
        # np.concatenate is the efficient way to join NumPy arrays
        front_readings = np.concatenate((front_slice_1, front_slice_2))
        # Filter out NaN/invalid values (0.0) and 'inf' (open space)
        # We only want the readings that are finite and within the sensor's valid range
        valid_readings = front_readings[
            np.isfinite(front_readings) & 
            (front_readings > msg.range_min) & 
            (front_readings < msg.range_max)
        ]
        # 4. Calculate the average distance
        if valid_readings.size > 0:
            self.average_front_distance = np.mean(valid_readings)
            self.get_logger().info(  #Note will only print to console if changed to self.get_logger().info()
                f"Average front distance (20 deg): {self.average_front_distance:.3f}m "
                f"from {valid_readings.size} valid points."
            )
        else:
            # If no valid readings (i.e., open space), set distance to infinity
            self.average_front_distance = float('inf') 
            self.get_logger().debug("Front of robot is clear (no valid Lidar reading).") #Note will only print to console if changed to self.get_logger().info()
            
        

    def callback_camera_subscriber(self, msg):
        # Only if front disatance is within 10% (positive or minus) of self.ideal_qr_measuring_distance (which has a value set at top of program)
        if abs(self.average_front_distance - self.ideal_qr_measuring_distance) <= (0.1 * self.ideal_qr_measuring_distance):
            self.frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY) # Convert to grayscale for better performance
            # Create a QR Code detector instance
            qr_detector = cv2.QRCodeDetector()
            # Detect and decode the QR code - The detectAndDecode function returns three values: the decoded string, the bounding box of the QR code, and a straight-on version of the code. We only need the data string.
            data, _, _ = qr_detector.detectAndDecode(gray_frame)
            if data:
                qr_data = data
                if qr_data and qr_data != self.last_qr_code:
                    self.get_logger().info(f"QR Code detected: {qr_data}")
                    
                    # --- NEW: Check if we already recorded this QR code ---
                    if qr_data in self.recorded_waypoints:
                        self.get_logger().warn(f"Waypoint {qr_data} already recorded. Skipping.")
                        self.last_qr_code = qr_data # Still update last_qr_code to suppress immediate re-read
                        return
                    
                    self.last_qr_code = qr_data # Store to suppress immediate re-read
                    
                    # --- NEW: Trigger Pose Lookup ---
                    self.get_logger().info(f"Attempting to read robot pose for QR: {qr_data}")
                    self.get_pose_and_save(msg.header.stamp, qr_data)



    def get_pose_and_save(self, timestamp, qr_data):
        try:
            # 1. Look up the transform (map -> base_link) at the exact time the image was captured
            t = self.tf_buffer.lookup_transform(
                self.map_frame, 
                self.robot_frame, 
                timestamp, 
                timeout=rclpy.duration.Duration(seconds=0.1) # Max wait time
            )
        except TransformException as ex:
            self.get_logger().error(f'Could not transform {self.robot_frame} to {self.map_frame}: {ex}')
            return

        # 2. Extract Position (x, y, z is usually 0 in 2D SLAM)
        x = t.transform.translation.x
        y = t.transform.translation.y
        
        # 3. Extract Orientation (Yaw)
        # The transform gives quaternion (x, y, z, w). We need to convert it to Euler angles (roll, pitch, yaw)
        quaternion = [
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
        ]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion) # yaw is the rotation around the Z-axis
        
        # 4. Store the data
        self.recorded_waypoints[qr_data] = {
            'x': round(x, 3), # Rounding for cleaner output
            'y': round(y, 3),
            'yaw': round(yaw, 3)
        }
        
        self.get_logger().info(
            f"SAVED: QR: {qr_data} | Pose: X={x:.3f}, Y={y:.3f}, Yaw={yaw:.3f}"
        )



    def save_waypoints_to_file(self):
        """Saves the recorded waypoints to a JSON file."""
        if not self.recorded_waypoints:
            self.get_logger().warn("No QR code waypoints were recorded. Skipping file save.")
            return

        # Define the file path (e.g., in the user's home directory or current working directory)
        # Using the current working directory for simplicity in a ROS 2 context
        file_name = "qr_codes_coordinates.json" 
        
        # Use JSON for structured, easy-to-read, and program-readable data
        try:
            with open(file_name, 'w') as f:
                json.dump(self.recorded_waypoints, f, indent=4)
            self.get_logger().info(f"Successfully saved {len(self.recorded_waypoints)} waypoints to {os.path.abspath(file_name)}")
        except Exception as e:
            self.get_logger().error(f"Failed to save waypoints to file: {e}")


    # ======================================================================#

def main(args=None):
    rclpy.init(args=args)
    node = QrCodeReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node shutdown initiated by user.")
    finally:
        # --- NEW: Call the save function on clean shutdown ---
        node.save_waypoints_to_file()
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()









        