#!/user/bin/env python3

import time
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import do_transform_point
import tf2_ros
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import rospy
import json
from tf2_ros.buffer import Buffer

class Record(Node):
    def __init__(self):
        super().__init__('stretch_tf_listener')

        self.declare_parameter('target_frame', 'target_object1')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        time_period = 1.0 # seconds
        self.timer = self.create_timer(time_period, self.on_timer)

    def on_timer(self):
        to_frame_rel = self.target_frame
        from_frame_rel = 'link_grasp_center'
        try:
            now = time()
            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        point = PointStamped()
        point.header.frame_id = 'link_grasp_center'
        point.point.x = 0.0
        point.point.y = 0.0
        point.point.z = 0.0
        transformed_point = do_transform_point(point, trans)
        saved_point = [transformed_point.x, transformed_point.y, transformed_point.z]
        file_path = 'saved_poses.json'
        with open(file_path, 'w') as json_file:
            json.dump(saved_point, json_file)
def main(args=None):
    print("Record python script")
    rclpy.init()
    node = Record()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
if __name__ == '__main__':
    main()