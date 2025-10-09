#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import json 
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import time 
import sys # For clean exit

# ======================= GLOBAL DATA LOADING ========================= #

# This data needs to be loaded before the node starts.
try:
    package_name = 'slambot_controllers'
    share_directory = get_package_share_directory(package_name)
    json_file_path = os.path.join(share_directory, 'data', 'qr_codes_coordinates.json')
    
    # Check if the file exists before attempting to open
    if not os.path.exists(json_file_path):
        print(f"ERROR: Waypoint file not found at: {json_file_path}")
        sys.exit(1)

    with open(json_file_path, 'r') as f:
        QR_WAYPOINTS = json.load(f)
except Exception as e:
    print(f"FATAL ERROR: Failed to load QR waypoints from JSON: {e}")
    sys.exit(1)

# ===================================================================== #

class QRCodeWaypointFollower(Node):   
    
    def __init__(self):
        super().__init__('qr_code_waypoint_follower')
        self.get_logger().info('QRCodeWaypointFollower node initialized.')

        # 1. Initialize the BasicNavigator
        self.nav = BasicNavigator()

        # 2. Get the converted list of PoseStamped goals
        self.goal_poses = self.prepare_goals(QR_WAYPOINTS)
        
        # 3. Set the robot's initial pose (REQUIRED by Nav2)
        self.set_initial_pose()
        
        # 4. Start Navigation Sequencer (using a one-shot timer to allow the Node to finish setup)
        self.navigation_started = False
        self.status_timer = None 
        self.next_goal_delay_timer = None # NEW: Timer to manage the one-shot delay before the next goal
        self.start_nav_timer = self.create_timer(1.0, self.start_navigation_sequence)


    def set_initial_pose(self):
        """Sets the starting pose for Nav2 (e.g., origin [0,0,0])."""
        
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        
        self.nav.setInitialPose(initial_pose)
        self.get_logger().info("Waiting for Nav2 to become active...")
        
        # Wait for Nav2 to be ready
        self.nav.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active and ready to receive goals.")


    def create_goal_pose(self, x, y, yaw):
        """Helper function to create a PoseStamped message."""
        
        # Convert Euler yaw (radians) to Quaternion (x, y, z, w)
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, yaw) 
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
        
        return goal_pose


    def prepare_goals(self, waypoints_data):
        """Converts the JSON dictionary into a list of PoseStamped messages."""
        goal_list = []
        # Sort the waypoints by QR code name if they are named "wp_1", "wp_2", etc.
        sorted_keys = sorted(waypoints_data.keys()) 

        if not sorted_keys:
            self.get_logger().error("No waypoints found in JSON data!")
            return []

        for key in sorted_keys:
            wp = waypoints_data[key]
            
            x = wp.get('x', 0.0)
            y = wp.get('y', 0.0)
            yaw = wp.get('yaw', 0.0)
            
            goal_pose = self.create_goal_pose(x, y, yaw)
            goal_list.append((key, goal_pose)) 
            self.get_logger().info(f"Prepared goal for {key}: ({x:.2f}, {y:.2f}) at {yaw:.2f} rad")

        return goal_list


    def start_navigation_sequence(self):
        """This function starts the sequential loop to send goals."""
        
        self.start_nav_timer.cancel() 
        
        if self.navigation_started or not self.goal_poses:
            self.get_logger().warn("Navigation already started or no goals to process.")
            return

        self.navigation_started = True
        self.get_logger().info(f"Starting navigation through {len(self.goal_poses)} waypoints.")
        
        self.send_next_goal(0)


    def send_next_goal(self, index):
        """Sends a single goal and handles the completion/error logic."""
        
        # Clear the delay timer if it exists (in case of manual override)
        if self.next_goal_delay_timer:
            self.next_goal_delay_timer.cancel()
            self.next_goal_delay_timer = None

        if index >= len(self.goal_poses):
            self.get_logger().info("All waypoints reached. Navigation finished.")
            rclpy.shutdown() 
            return

        qr_name, goal_pose = self.goal_poses[index]
        self.get_logger().info(f"Sending goal {index + 1}/{len(self.goal_poses)}: {qr_name}")

        # Send the goal to Nav2
        self.nav.goToPose(goal_pose)

        # Start a loop to monitor the goal status
        self.monitor_goal_status(index, qr_name)


    def monitor_goal_status(self, index, qr_name):
        """Uses a timer to check the status of the current goal."""
        
        # Ensure old monitoring timer is canceled before creating a new one
        if self.status_timer:
            self.status_timer.cancel()

        # Timer to monitor status every 0.1 seconds
        self.status_timer = self.create_timer(0.1, lambda: self.check_goal(index, qr_name))

        # Reset flag for the new goal
        self.goal_monitored = False

    def _trigger_next_goal_delay_callback(self, next_index):
        """One-shot callback to cancel the delay timer and trigger the next goal."""
        if self.next_goal_delay_timer:
            self.next_goal_delay_timer.cancel()
            self.next_goal_delay_timer = None
        
        self.send_next_goal(next_index)


    def check_goal(self, index, qr_name):
        """Callback to check if the current goal is complete."""
        
        if self.nav.isTaskComplete() and not self.goal_monitored:
            self.goal_monitored = True
            
            # Stop the recurring status check timer immediately
            if self.status_timer:
                self.status_timer.cancel() 
                self.status_timer = None

            result = self.nav.getResult()
            
            # FIX 1: Check if the result string contains 'SUCCEEDED' 
            # (as TaskResult.SUCCEEDED was printed in the logs)
            # This is more stable than relying on integer code 3 or unreliable enumerations.
            result_str = str(result)
            
            if 'SUCCEEDED' in result_str:
                self.get_logger().info(f"Waypoint {qr_name} reached successfully.")
            else:
                # Log as an ERROR, printing the full status string/object for diagnosis
                self.get_logger().error(f"Waypoint {qr_name} failed with status: {result_str}. Continuing to next goal.")
            
            # FIX 2: Use the dedicated one-shot timer callback to manage the delay
            # This prevents the timer from recurring infinitely.
            self.next_goal_delay_timer = self.create_timer(
                1.0, 
                lambda: self._trigger_next_goal_delay_callback(index + 1)
            )
            
        else:
            # Provide feedback during navigation (optional, but good)
            feedback = self.nav.getFeedback()
            if feedback:
                time_remaining_sec = (
                    feedback.estimated_time_remaining.sec + 
                    feedback.estimated_time_remaining.nanosec / 1e9
                )
                
                self.get_logger().debug(
                    f"Time remaining to {qr_name}: {time_remaining_sec:.1f}s. "
                    f"Distance to goal: {feedback.distance_remaining:.2f}m"
                )


# ===================================================================== #

def main(args=None):
    rclpy.init(args=args)

    node = QRCodeWaypointFollower()  
    
    # We use rclpy.spin() to let the node run, but the navigation sequence is 
    # controlled internally by the timers and BasicNavigator methods.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
