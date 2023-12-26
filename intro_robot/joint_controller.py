import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)

    def send_joint_effort(self, joint_name, effort):
        msg = JointTrajectory()
        msg.joint_names = [joint_name]
        point = JointTrajectoryPoint()
        point.positions = [0.0]  # Set the joint position if required
        point.velocities = [0.0]  # Set the joint velocity if required
        point.effort = [effort]
        msg.points = [point]
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = JointController()
    node.send_joint_effort('your_joint_name', 1.0)  # Replace 'your_joint_name' with the actual joint name
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
