import rclpy
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs  # This import is necessary for using transform_datatypes
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from intro_robot.robot_controller import RobotController
from urdf_parser_py.urdf import URDF

class FramePublisher():

    def __init__(self):
        self.node = rclpy.create_node('giraff_robot')
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self.node)
        #self.joint_state_publisher = self.node.create_publisher(JointState, '/joint_states', 10)
        self.node.declare_parameter('robot_description', '')
        if self.node.has_parameter('robot_description'):
            self.robot_description = self.node.get_parameter('robot_description').get_parameter_value().string_value
        else:
            self.node.get_logger().error("Parameter 'robot_description' is missing!")
        self.node.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10  # QoS profile depth
        )
        #self.timer = self.node.create_timer(1.0, self.joint_state_callback)  # Trigger every 1 second


    def joint_state_callback(self, updated_msg):
        # Process joint state message
        #joint_state_msg = JointState()
        if updated_msg:
            print("Received Joint State:")
            print("Header: ", updated_msg.header)
            print("Joint Names: ", updated_msg.name[0])
            print("Joint Positions: ", updated_msg.position[0])            
            #joint_state_msg.header = updated_msg.header
            #joint_state_msg.name = updated_msg.name[0] # Add your joint names
            #position = float(updated_msg.position[0])
            #joint_state_msg.position = position # Add your joint positions
            #self.update_tf(updated_msg.position[0], updated_msg.name[0])
        else:
            print("Error receiving Joint State!")

        # Publish joint state information
        #self.joint_state_publisher.publish(joint_state_msg)
        
        

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
    


    def update_tf(self, q, j):
        T = []
        T.append(RobotController.compute_transform_matrix(q))

        try:
            with open(self.robot_description, 'r') as urdf_file:
                self.robot_desc = URDF.from_xml_string(urdf_file.read())
        except Exception as e:
            self.get_logger().error(f"Error parsing URDF: {e}")
            self.robot_desc = None
 

        for i, joint in enumerate(self.robot_desc.joints):
            if joint==j:
                if i==0:
                    roll = q[i]
                    pitch = 0.0
                    yaw = 0.0
                elif i==1:
                    roll = q[i]
                    pitch = 0.0
                    yaw = 0.0
                elif i==2:
                    roll = 0.0
                    pitch = 0.0
                    yaw = 0.0
                elif i==3:
                    roll = q[i]
                    pitch = 0.0
                    yaw = 0.0
                elif i==4:
                    roll = 0.0
                    pitch = q[i]
                    yaw = 0.0
                x = T[i][1, 3]
                y = T[i][2, 3]
                z = T[i][3, 3]
                frame = joint.child
                parent_frame = joint.parent               
                self.publish_tf(x, y, z, roll, pitch, yaw, frame, parent_frame)


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
             print(f"Transform lookup time: {t.header.stamp}")
             print(f"{t.child_frame_id}")
             print(f"{t.header.frame_id}")
        except Exception as e:
             print(f"Failed to publish transform {e}.")
             print(f"Transform lookup time: {t.header.stamp}")
        

def main():
    rclpy.init()
    robot = FramePublisher()
    try:
        while rclpy.ok():
            timer0 = robot.node.create_timer(1.0, lambda:robot.publish_tf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'base_link', 'world'))
            timer1 = robot.node.create_timer(1.0, lambda:robot.publish_tf(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'leg_link', 'base_link'))
            timer2 = robot.node.create_timer(1.1, lambda:robot.publish_tf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'arm_link', 'leg_link'))
            timer3 = robot.node.create_timer(1.1, lambda:robot.publish_tf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'end_effector', 'arm_link'))
            timer4 = robot.node.create_timer(1.1, lambda:robot.publish_tf(0.0, 0.0, 0.0, 0.0, np.radians(30.0), 0.0, 'tool_mic', 'end_effector'))
            rclpy.spin(robot.node)
            timer0.cancel()
            timer1.cancel()
            timer2.cancel()
            timer3.cancel()
            timer4.cancel()          
    except KeyboardInterrupt:
        pass
    finally:
        robot.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
