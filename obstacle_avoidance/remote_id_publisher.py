import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class RemoteIDPublisher(Node):
    def __init__(self):
        super().__init__('remote_id_publisher')
        self.publishers_ = self.create_publisher(String,'remote_id',10)
        self.timer = self.create_timer(1.0,self.publish_remote_id)
        self.get_logger().info('Remote ID Publisher has been started')
    def publish_remote_id(self):

        remote_id_data = {
            "drone_id": "DRONE12345",
            "location": {
                "latitude":37.7749,
                "longitude": -122.4194,
                "altitude":100.5,
            },
            "velocity": {
                "x": 5.0,
                "y": 0.0,
                "z": 0.0
            },
            "status":"OK",
            "timestamp": self.get_clock().now().to_msg().sec
        }
        msg = String()
        #msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = json.dumps(remote_id_data)
        self.publishers_.publish(msg)
        self.get_logger().info(f'Publish Remote ID Data{msg.data}')




def main(args=None):
    rclpy.init(args=args)
    node = RemoteIDPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
