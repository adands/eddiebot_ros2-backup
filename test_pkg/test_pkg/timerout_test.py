import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from geometry_msgs.msg import Twist

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.sub = self.create_subscription(Twist,"turtle1/cmd_vel",self.my_callback,10)
        #self.timer = self.create_timer(1, self.my_callback)

        self.duration = Duration(seconds = 5)

    def my_callback(self,msg):
        print("hi")
        start_time = self.get_clock().now()
        time = self.get_clock().now()
        while time - start_time < self.duration:
            print(time - start_time)
            time = self.get_clock().now()
        print(self.duration)
        print("ok done")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    finally:
        # 清理工作
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
