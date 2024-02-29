import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, qos_profile_system_default
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import json

from geometry_msgs.msg import PoseStamped
from BasicNavigator import BasicNavigator, TaskResult

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class GoToLocationSubscriber(Node):
    def __init__(self):
        super().__init__('go_to_location_subscriber')
        self.curr_pos = None
        self.filename = "./locations.json"
        self.navigator = BasicNavigator()

        self.send_pos_sub = self.create_subscription(String, 'send_pos_sub', self.send_position_callback, 10)

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

            # final_pose = PoseStamped()
            # final_pose.header.frame_id = 'map'
            # final_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            # final_pose.pose.position.x = x
            # final_pose.pose.position.y = y
            # final_pose.pose.orientation.z = 0.0
            # final_pose.pose.orientation.w = 1.0

            # self.navigator.goToPose(final_pose)

            # # i = 0
            # # while not self.navigator.isTaskComplete():

            # #     i = i + 1
            # #     feedback = self.navigator.getFeedback()
            # #     if feedback and i % 5 == 0:
            # #         print(
            # #             'Estimated time of arrival: '
            # #             + '{0:.0f}'.format(
            # #                 Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
            # #                 / 1e9
            # #             )
            # #             + ' seconds.'
            # #         )

            # #         # Some navigation timeout to demo cancellation
            # #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            # #             self.navigator.cancelTask()

            # #         # Some navigation request change to demo preemption
            # #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
            # #             final_pose.pose.position.x = -3.0
            # #             self.navigator.goToPose(final_pose)

            # # Do something depending on the return code
            # result = self.navigator.getResult()
            # if result == TaskResult.SUCCEEDED:
            #     print('Goal succeeded!')
            # elif result == TaskResult.CANCELED:
            #     print('Goal was canceled!')
            # elif result == TaskResult.FAILED:
            #     print('Goal failed!')
            # else:
            #     print('Goal has an invalid return status!')
        


def main(args=None):
    rclpy.init(args=args)

    send_location_poses_subs = GoToLocationSubscriber()

    rclpy.spin(send_location_poses_subs)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    send_location_poses_subs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    