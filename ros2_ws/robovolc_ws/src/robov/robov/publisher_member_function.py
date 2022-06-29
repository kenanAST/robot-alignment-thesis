import rclpy
import keyboard
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/commanderFront/cmd_demo', 10)
        self.sidePublisher_ = self.create_publisher(Twist, '/commanderBack/cmd_demo', 10)
        while True:
            front = Twist()
            side = Twist()
            command = String()
            friction = -10
            try:
                if keyboard.is_pressed('w'):
                    command.data = "'{linear: {x: 1.0}}' -1"
                    front.linear.x = 10.0
                    side.linear.x = 10.0

                if keyboard.is_pressed('a'):
                    command.data = 'left'
                    front.linear.x = - 0.25
                    side.linear.x = 0.25

                if keyboard.is_pressed('s'):
                    command.data = "{linear: {x: - 1.0}}' -1"
                    front.linear.x = -5.0
                    side.linear.x = -5.0

                if keyboard.is_pressed('d'):
                    command.data = 'right'
                    front.linear.x = 0.25
                    side.linear.x = -0.25

                if keyboard.is_pressed('f'):
                    command.data = 'stop'
                    front.linear.x = 0
                    side.linear.x = 0

            except:
                pass
            if(command.data != ""):
                self.publisher_.publish(front)
                self.sidePublisher_.publish(side)
                self.get_logger().info('Publishing: "%s"' % side)
                self.get_logger().info('Publishing: "%s"' % front)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()