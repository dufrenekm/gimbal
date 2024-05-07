# Simple class for calling action server and testing motor operations


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile
from time import sleep

class MotorTestClient(Node):

    def __init__(self):
        super().__init__('motor_test_client')
        qos_profile = QoSProfile(depth=10)
        self.targ_pub = self.create_publisher(JointState, 'gimbal_target', qos_profile)
        self.demo_pos1 = 0.0
        self.demo_pos2 = 0.0

    def send_target(self):
        js = JointState()
        now = self.get_clock().now()
        js.header.stamp = now.to_msg()
        js.name = ['dist', 'prox']
        js.velocity = [2.0, 2.0]
        js.position = [self.demo_pos1, self.demo_pos2]
        self.demo_pos1 = min(0.8, self.demo_pos1 + .05)
        self.demo_pos2 = min(0.8, self.demo_pos2 + .05)

        self.targ_pub.publish(js)

def main(args=None):
    rclpy.init(args=args)

    target_client = MotorTestClient()
    for i in range(2000):
        target_client.send_target()
        sleep(.05)
    rclpy.shutdown()



if __name__ == '__main__':
    main()