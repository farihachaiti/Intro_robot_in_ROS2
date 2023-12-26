import rclpy
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs  # This import is necessary for using transform_datatypes
import math
import numpy as np
from rclpy.node import Node
import tf

class FramePublisher():

    def __init__(self):
        self.node = rclpy.create_node('giraff_robot')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self.node)
        self.publish_rate = 1
        self.rate = self.node.create_rate(self.publish_rate)


    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk


        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q


    def publish_tf(self, x, y, z, roll, pitch, yaw, frame, parent_frame):
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        q = self.quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Publish the transforms
        try:
             self.tf_broadcaster.sendTransform(t)
             print("Successfully published transform.")
        except Exception as e:
             print(f"Failed to publish transform {e}.")
        self.rate.sleep()

def main():
    rclpy.init()
    robot = FramePublisher()

    timer_period = 1.0
    #while rclpy.ok():
    timer1 = robot.node.create_timer(timer_period, lambda:robot.publish_tf(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'leg_link', 'base_link'))
    timer2 = robot.node.create_timer(timer_period, lambda:robot.publish_tf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'arm_link', 'leg_link'))
    #timer3 = robot.node.create_timer(timer_period, lambda:robot.publish_tf(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'arm_link', 'leg_link'))
    timer3 = robot.node.create_timer(timer_period, lambda:robot.publish_tf(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'end_effector', 'arm_link'))
    while rclpy.ok():
        rclpy.spin_once(robot.node)

    timer1.cancel()
    timer2.cancel()
    timer3.cancel()
    #timer4.cancel()
    robot.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
