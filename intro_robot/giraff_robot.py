import rclpy
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs  # This import is necessary for using transform_datatypes
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF


robot = None


class FramePublisher():
    _node_created = False
    def __init__(self):
        if not FramePublisher._node_created:
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
            FramePublisher._node_created = True

        else:
            print("Node has already been created.")
        
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
    
    def compute_transform_matrix(self, q):
        # Extract joint angles and prismatic displacements
        theta1, theta2, d3, theta4, theta5 = q
        

        # Define robot parameters (replace these with the actual parameters of your robot)
        L1 = 0.1
        L2 = 0.6
        L3 = 0.6
        L4 = 0.6
        L5 = 0.6

        T01 =  np.array([
        [1, 0, 0, 0],
        [0, np.cos(np.radians(theta1)), -np.sin(np.radians(theta1)), 0],
        [0, np.sin(np.radians(theta1)), np.cos(np.radians(theta1)), L1],
        [0, 0, 0, 1]
        ])

        T12 =  np.array([
        [1, 0, 0, 0],
        [0, np.cos(np.radians(theta2)), -np.sin(np.radians(theta2)), 0],
        [0, np.sin(np.radians(theta2)), np.cos(np.radians(theta2)), L2],
        [0, 0, 0, 1]
        ])

        T23 =  np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d3+L3],
        [0, 0, 0, 1]
        ])


        T34 =  np.array([
        [1, 0, 0, -L4],
        [0, np.cos(np.radians(theta4)), -np.sin(np.radians(theta4)), 0],
        [0, np.sin(np.radians(theta4)), np.cos(np.radians(theta4)), 0],
        [0, 0, 0, 1]
        ])

        T4ee =  np.array([
        [np.cos(np.radians(theta5)), 0, np.sin(np.radians(theta5)), 0],
        [0, 1, 0, 0],
        [-np.sin(np.radians(theta5)), 0, np.cos(np.radians(theta5)), -L5],
        [0, 0, 0, 1]
        ])

        return T01, T12, T23, T34, T4ee

    


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
        
def update_tf(joint_name=None, q=None):
    global robot
    if robot is None:
        robot = FramePublisher()
    if q is not None:
        T = []
        T.append(robot.compute_transform_matrix(q))
        try:
            with open(robot.robot_description, 'r') as urdf_file:
                robot.robot_desc = URDF.from_xml_string(urdf_file.read())
        except Exception as e:
            robot.get_logger().error(f"Error parsing URDF: {e}")
            robot.robot_desc = None


        for i, joint in enumerate(robot.robot_desc.joints):
            if joint.name==joint_name:
                print('sdfsdfsd000000000000000')
                print(i)
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
                print('okay1')             
                robot.publish_tf(x, y, z, roll, pitch, yaw, frame, parent_frame)
    else:
        print('okay2')
        robot.publish_tf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'base_link', 'world')
        robot.publish_tf(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'leg_link', 'base_link')
        robot.publish_tf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'arm_link', 'leg_link')
        robot.publish_tf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'end_effector', 'arm_link')
        robot.publish_tf(0.0, 0.0, 0.0, 0.0, np.radians(30.0), 0.0, 'tool_mic', 'end_effector')
   
        


def main():
    rclpy.init()   
    try:      
        update_tf()
        while rclpy.ok():
            rclpy.spin(robot.node)       
    except KeyboardInterrupt:
        pass
    finally:
        robot.node.destroy_node()
        FramePublisher._node_created = False
        rclpy.shutdown()

if __name__ == '__main__':
    main()
