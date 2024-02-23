import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, qos_profile_system_default
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from copy import deepcopy
import json

from geometry_msgs.msg import PoseStamped
from BasicNavigator import BasicNavigator, TaskResult

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from enum import Enum
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import BackUp, Spin
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from nav2_msgs.action import FollowPath, FollowWaypoints, NavigateThroughPoses, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap, ManageLifecycleNodes

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class GoToLocation(Node):
    def __init__(self):
        super().__init__('go_to_location_subscriber')
        self.curr_pos = None
        self.filename = "./locations.json"
        self.navigator = BasicNavigator()

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              'amcl_pose',
                                                              self._amclPoseCallback,
                                                              qos_profile_system_default)
        
        self.save_pos_sub = self.create_subscription(String, 'save_pos_sub', self.send_position_callback, 10)
        

          # callback to store the location
        
    def send_position_callback(self, location_name):
        # open the json file and append the location to it
        self.get_logger().info(location_name.data)
        self.get_logger().info(f"{self.curr_pos}")
        # self.get_logger().info("" + self.curr_pos)
        with open(self.filename, 'r+') as file:
            try:
                file_data = json.load(file)
            except:
                file.seek(0)
                file.truncate(0)
                file_data = {"locations": []}

            x, y = 0
            f = open(self.filename)
            data = json.load(f)
            for i in data['locations']:
                for j in i:
                    if i[location_name]:
                        x = j['x']
                        y = j['y']
            f.close()

            final_pose = PoseStamped()
            final_pose.header.frame_id = 'map'
            final_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            final_pose.pose.position.x = x
            final_pose.pose.position.y = y
            final_pose.pose.orientation.z = 0.0
            final_pose.pose.orientation.w = 1.0



    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
        


def main(args=None):
    rclpy.init(args=args)

    save_location_poses_subs = SaveLocationsPosesSubscriber()

    rclpy.spin(save_location_poses_subs)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    save_location_poses_subs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    