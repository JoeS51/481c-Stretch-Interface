#!/user/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransofmrBroadcaster
import tf_transformations
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import json
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import PointStamped

class Playback(Node):
    def __init__(self):
        super().__init__('stretch_tf_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        time_period = 1.0 # seconds
        self.timer = self.create_timer(time_period, self.on_timer)

    def on_timer(self):
        # use look up trans
        tpoint = PointStamped()
        tpoint.header.frame_id = 'target_object1'
        f = open('saved_poses.json')
        data = json.load(f)
        for i in data:
            arr = i
        tpoint.point.x = arr[0]
        tpoint.point.y = arr[1]
        tpoint.point.z = arr[2]
        try:
            transformed_message = self.tf_buffer.transform(tpoint, 'base_link')
            exit(0)
        except TransformException as ex:
            print("error")
            return


def main(args=None):
    print("playback python script")
    rclpy.init()
    node = Playback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
if __name__ == '__main__':
    main()