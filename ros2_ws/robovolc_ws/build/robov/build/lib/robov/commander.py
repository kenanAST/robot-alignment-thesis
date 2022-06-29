from turtle import right
import rclpy
import keyboard
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.left_1 = self.create_publisher(Twist, '/left_1/cmd_demo', 10)
        self.left_2 = self.create_publisher(Twist, '/left_2/cmd_demo', 10)
        self.left_3 = self.create_publisher(Twist, '/left_3/cmd_demo', 10)
        self.right_1 = self.create_publisher(Twist, '/right_1/cmd_demo', 10)
        self.right_2 = self.create_publisher(Twist, '/right_2/cmd_demo', 10)
        self.right_3 = self.create_publisher(Twist, '/right_3/cmd_demo', 10)
        while True:
            left1 = Twist()
            left2 = Twist()
            left3 = Twist()
            right1 = Twist()
            right2 = Twist()
            right3 = Twist()
            command = String()
            try:
                if keyboard.is_pressed('w'):
                    command.data = "'{linear: {x: 1.0}}' -1"
                    left1.linear.x = 5.0
                    left2.linear.x = 5.0
                    left3.linear.x = 5.0
                    right1.linear.x = -5.0
                    right2.linear.x = -5.0
                    right3.linear.x = -5.0
                    pass

                if keyboard.is_pressed('a'):
                    command.data = 'left'
                    left1.linear.x = -5.0
                    left2.linear.x = -5.0
                    left3.linear.x = -5.0
                    right1.linear.x = -5.0
                    right2.linear.x = -5.0
                    right3.linear.x = -5.0

                if keyboard.is_pressed('s'):
                    command.data = "{linear: {x: - 1.0}}' -1"
                    left1.linear.x = -5.0
                    left2.linear.x = -5.0
                    left3.linear.x = -5.0
                    right1.linear.x = 5.0
                    right2.linear.x = 5.0
                    right3.linear.x = 5.0

                if keyboard.is_pressed('d'):
                    command.data = 'right'
                    left1.linear.x = 5.0
                    left2.linear.x = 5.0
                    left3.linear.x = 5.0
                    right1.linear.x = 5.0
                    right2.linear.x = 5.0
                    right3.linear.x = 5.0

                if keyboard.is_pressed('f'):
                    command.data = 'stop'
                    left1.linear.x = 0
                    left2.linear.x = 0
                    left3.linear.x = 0
                    right1.linear.x = 0
                    right2.linear.x = 0
                    right3.linear.x = 0

            except:
                pass
            if(command.data != ""):
                self.left_1.publish(left1)
                self.left_2.publish(left2)
                self.left_3.publish(left3)
                self.right_1.publish(right1)
                self.right_2.publish(right2)
                self.right_3.publish(right3)
                print(left2)
            else:
                print("filler")


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