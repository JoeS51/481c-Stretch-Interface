import json
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FrameListener(Node):

    def __init__(self):
        super().__init__('pbd')

        self.declare_parameter('target_frame', 'base_link')
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.done = False
        self.obj = 'toy'

        time_period = 1.0 # seconds
        self.timer = self.create_timer(time_period, self.on_timer)

    def on_timer(self):
        from_frame_rel = self.obj
        to_frame_rel = self.target_frame
        print(self.obj)
        try:
            now = Time()
            trans = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, now)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        # aruco_point = {
        #     'x': trans.transform.translation.x,
        #     'y': trans.transform.translation.y,
        #     'z': trans.transform.translation.z,
        # }

        target_point = {
            'x': trans.transform.translation.x,
            'y': trans.transform.translation.y,
            'z': trans.transform.translation.z,
        }

        self.get_logger().info(f'{target_point}')

        #rotation = math.atan2(target_point['y'], target_point['x']) + (math.pi / 2)
        rotation = math.atan2(target_point['y'], target_point['x'])

        # if rotation > math.pi:
        #     rotation -= 2 * math.pi
        # if rotation < -math.pi:
        #     rotation += 2 * math.pi

        self.get_logger().info(f'{rotation}')
        self.get_logger().info(f'{math.degrees(rotation)}')

        distance = math.sqrt(target_point['x'] ** 2 + target_point['y'] ** 2)
        self.get_logger().info(f'{distance}')
        if self.obj == 'toy_box':
            with open('values.json', 'w') as f:
                json.dump({'rotation': rotation, 'distance': distance}, f)
        else:
            with open('value_toy.json', 'w') as f:
                json.dump({'rotation': rotation, 'distance': distance}, f)

        self.done = True

def main():
    rclpy.init()
    node = FrameListener()
    print(node.obj)
    node.obj = 'toy_box'
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()