import numpy as np
import sympy as sp
#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.srv import ApplyJointEffort
from sensor_msgs.msg import JointState
import tf2_ros
from urdf_parser_py.urdf import URDF


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        buffer_size = 10.0
        self.tf_buffer = tf2_ros.Buffer(cache_time=tf2_ros.Duration(seconds=10.0, nanoseconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.joint_trajectory_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_effort_srv_client= self.create_client(ApplyJointEffort, '/apply_joint_effort')
    

            # Wait for the service to be available
        while not self.joint_effort_srv_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service not available, waiting...')


    def listen_tf(self):
        try:
            transform1 = self.tf_buffer.lookup_transform('base_link', 'leg_link', rclpy.time.Time())
            transform2 = self.tf_buffer.lookup_transform('leg_link', 'arm_link', rclpy.time.Time())
            transform3 = self.tf_buffer.lookup_transform('arm_link', 'end_effector', rclpy.time.Time())
            self.get_logger().info(f'Transform received from base link to leg link: {transform1}')
            self.get_logger().info(f'Transform received from leg link to arm link: {transform2}')
            self.get_logger().info(f'Transform received from arm link to end effector: {transform3}')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Failed to lookup transform: {e}')

    def get_cost(self, config1, config2):
        return np.linalg.norm(np.array(config1) - np.array(config2)).astype(int)

    def nearest(self, graph, new_node):
        nearest = None
        min_distance = 100

        for node in graph:
            distance = self.get_cost(new_node, node)
            
            if (distance <= min_distance) and distance!=0:
                min_distance = distance
                nearest = node
        
        return nearest


    def extend(self, q_nearest, q_random, step_size=10):
        new_node = np.zeros(4)

        # Extend from the nearest node towards the sampled point
        delta_x1 = q_random[0] - q_nearest[0]
        delta_x2 = q_random[1] - q_nearest[1]
        delta_x3 = q_random[2] - q_nearest[2]
        delta_x4 = q_random[3] - q_nearest[3]
        #print(q_random[0] - q_nearest[0])
        norm = np.linalg.norm(np.array([delta_x1, delta_x2, delta_x3, delta_x4]))
    
        delta_x1 /= norm
        delta_x2 /= norm
        delta_x3 /= norm
        delta_x4 /= norm
        
        if norm>step_size:  
            
            new_node[0] = (q_nearest[0] + (step_size * delta_x1)).astype(int)
            new_node[1] = (q_nearest[1] + (step_size * delta_x2)).astype(int)
            new_node[2] = (q_nearest[2] + (step_size * delta_x3)).astype(int)
            new_node[3] = (q_nearest[3] + (step_size * delta_x4)).astype(int)    
        else:
            new_node = q_random
        
        
        return new_node


    def rewire(self, near_nodes, idx, q_new):
        near_nodes[idx] = q_new
        

    def is_clear(self, node, obstacles):
        # Check if the path from 'start' to 'end' is clear
        # The obstacles parameter is a list of Node objects representing obstacle positions

        # Simple collision detection for illustration purposes
        for obs in obstacles:
            if (self.get_cost(node, obs)<=0) :
                return False  # Collision detected

        return True  # Path is clear

    def get_trajectory(self, q_goal, q_init):
        graph = [q_init]
        while self.get_cost(q_goal, q_init)>0:
            q_random = [np.random.randint(0, 3), np.random.randint(0, 3), np.random.randint(0, 3), np.random.randint(0, 3)]
            if self.is_clear(q_random, graph):
                q_nearest = self.nearest(graph, q_random) 
                q_new = self.extend(q_nearest, q_random)
                min_cost = self.get_cost(q_goal,q_nearest)
        
                near_nodes = [node for node in graph if (self.get_cost(q_new, node) <= min_cost)] 
                for idx, near_node in enumerate(near_nodes):
                    if self.is_clear(near_node, graph) and np.all(np.array(graph))!=0:                   
                        if (self.get_cost(q_goal, near_node) + self.get_cost(q_new, near_node)) < min_cost:
                            min_cost = self.get_cost(q_goal, near_node) + self.get_cost(q_new, near_node)
                
                graph.append(q_new)
                for idx, near_node in enumerate(near_nodes):
                    if self.is_clear(near_node, graph) and np.all(np.array(graph))!=0: 
                        if (self.get_cost(q_goal, q_new) + self.get_cost(q_new, near_node))<self.get_cost(q_goal, near_node):
                            self.rewire(near_nodes, idx, q_new)  
            
                q_init = q_new      
        return graph



    def compute_inertia_matrix(self):
        M = np.zeros((5))
        self.robot_description = self.get_parameter('robot_description').get_parameter_value().string_value
        # Parse the URDF
        try:
            self.robot_desc = URDF.from_xml_string(self.robot_description)
        except Exception as e:
            self.get_logger().error(f"Error parsing URDF: {e}")
            self.robot_desc = None
            for idx, link in enumerate(self.robot_desc.link_map.items()):
                inertial = link.inertial
                if inertial is not None and inertial.inertia is not None:
                    M[idx]  = np.array([inertial.inertia.ixx[idx], -inertial.inertia.ixy[idx], -inertial.inertia.ixz[idx]],
                        [-inertial.inertia.ixy[idx], inertial.inertia.iyy[idx], -inertial.inertia.iyz[idx]],
                        [-inertial.inertia.ixz[idx], -inertial.inertia.iyz[idx], inertial.inertia.izz[idx]])


        return M
            

    def compute_coriolis_matrix(self, M, q, qd):
        n = len(q)
        C = np.zeros((5))
        for k in range(n):
            for i in range(len(M)):
                for j in range(len(M[i])):
                    C[k][i,j] = (0.5 * (sp.diff(M[k][i,j], q[k]) + sp.diff(M[k][i,j], q[k]) - sp.diff(M[k][j,i], q[k]))) * qd[k]

        return sp.simplify(C)



    def inverse_dynamics(self, qdd_desired, qd_desired, q_desired, qd_current, q_current, T_total):
        # Define symbolic variables
        M  = self.compute_inertia_matrix()
        C = self.compute_coriolis_matrix(M, q_current, qd_current)
        G = 9.81
        Kd = 0.5
        Kp = 2.0
        tau = np.dot(M, (qdd_desired + np.dot(Kd, (qd_desired - qd_current)) + np.dot(Kp, (q_desired - q_current)))) + np.dot(C, qd_current) + G


        # Joint positions and velocities
        #q = sp.Matrix([q1, q2])
        #qdot = sp.Matrix([sp.diff(qi) for qi in q])

        return tau

    def compute_forward_kinematics_from_configuration(self, q):

        T01, T12, T23, T34, T4ee = self.compute_transform_matrix(q)
        T_total = T01 @ T12 @ T23 @ T34 @ T4ee


        # Forward kinematics equations
        #x = L1 * np.cos(theta1) + L3 * np.cos(theta1 + theta3) + L4 * np.cos(theta1 + theta3 + theta4)
        #y = L1 * np.sin(theta1) + L3 * np.sin(theta1 + theta3) + L4 * np.sin(theta1 + theta3 + theta4)
        #z = d2 + L2 + L5
        
        return  np.array([T_total[0, 3], T_total[1, 3], T_total[2, 3]])
    

    
    def compute_transform_matrix(self, q):
                    # Extract joint angles and prismatic displacements
        theta1, theta2, d3, theta4 = q

        # Define robot parameters (replace these with the actual parameters of your robot)
        L1 = 0.1
        L2 = 0.6
        L3 = 0.6
        L4 = 0.6

        T01 =  np.array([
        [1, 0, 0, 0],
        [0, np.cos(theta1), -np.sin(theta1), 0],
        [0, np.sin(theta1), np.cos(theta1), L1],
        [0, 0, 0, 1]
        ])

        T12 =  np.array([
        [1, 0, 0, 0],
        [0, np.cos(theta2), -np.sin(theta2), 0],
        [0, np.sin(theta2), np.cos(theta2), L2],
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
        [0, np.cos(theta4), -np.sin(theta4), 0],
        [0, np.sin(theta4), np.cos(theta4), 0],
        [0, 0, 0, 1]
        ])

        T4ee =  np.array([
        [np.cos(np.radians(30)), 0, -np.sin(np.radians(30)), 0],
        [0, 1, 0, 0],
        [np.sin(np.radians(30)), 0, np.cos(np.radians(30)), 0],
        [0, 0, 0, 1]
        ])

        return T01, T12, T23, T34, T4ee


    def compute_direct_differential_kinematics_from_configuration(self, q):
        theta1, theta2, d3, theta4 = q

        T01, T12, T23, T34, T4ee = self.compute_transform_matrix(q)

        T02 = np.dot(T01, T12)
        T03 = np.dot(T02, T23)
        T04 = np.dot(T03, T34)
        T0ee = np.dot(T04, T4ee)

        P01 = T01[:3, 3]
        P02 = T02[:3, 3]
        P03 = T03[:3, 3]
        P04 = T04[:3, 3]
        P0ee = T0ee[:3, 3]
        P01 = P01.reshape(-1,1)
        P02 = P02.reshape(-1,1)
        P03 = P03.reshape(-1,1)
        P04 = P04.reshape(-1,1)
        P0ee = P0ee.reshape(-1,1)

        '''
        Z1 = base_joint (revolute)
        Z2 = base_spherical_joint (revolute)
        Z4 = arm_joint (revolute)
        Z5 = end_effector_joint (revolute)
        Z3 = leg_joint (prismatic)
        d3 = displacement across links of Z3
        Z3 = holding tool_mic link containing the microphone
        '''
        Z1 = Z2 = Z4 = np.array([[1, 0, 0],
                                [0, 0, 0],
                                [0, 0, 0]]) 
        Z3 = np.array([[0], [0], [d3]])
        Z5 = np.array([[0, 0, 0],
                      [0, 1, 0],
                      [0, 0, 0]])
        J1 = np.vstack((np.array(Z1 @ (P0ee - P01)), np.array([[1],[0],[0]])))
        J2 = np.vstack((np.array(Z2 @ (P0ee - P02)), np.array([[1],[0],[0]])))
        J3 = np.vstack((np.array(Z3), np.array([[0],[0],[0]])))
        J4 = np.vstack((np.array(Z4 @ (P0ee - P03)), np.array([[1],[0],[0]])))
        J5 = np.vstack((np.array(Z5 @ (P0ee - P04)), np.array([[0],[1],[0]])))
    
        
        J = np.concatenate((J1, J2, J3, J4, J5), axis=1)
       
        return J

    def compute_motion(self, p_desired, graph):
        # Given end effector position matrix 
        p_desired = np.array([1, 2, 1])
        tau = []

        # Initial guess for joint positions
        #q_current = np.array([0, 0, 0, 0])

        # Desired tolerance for convergence
        tolerance = 1e-6

        # Maximum number of iterations
        max_iterations = 100

        # Numerical inverse kinematics using the Jacobian inverse method
        for q_current in graph:
            p_current = self.compute_forward_kinematics_from_configuration(q_current)

            error = p_desired - p_current

            # Check if the error is below the tolerance
            if np.linalg.norm(error) < tolerance:
                print("Converged!")
                q_desired = q_current
                q_dot_desired = np.gradient(q_desired, dt=0.01, axis=0, edge_order=2)
                q_ddot_desired = np.gradient(q_dot_desired, dt=0.01, axis=0, edge_order=2)
                break

            # Compute Jacobian matrix
            #J = compute_jacobian(q_current)

            # Compute joint velocities using the Jacobian inverse
            q_dot = np.gradient(q_current, dt=0.01, axis=0, edge_order=2)

            # Update joint positions using the computed velocities
            q_current += q_dot * 0.01
        
        # Print the result
            tau.append(self.inverse_dynamics(q_ddot_desired, q_dot_desired, q_desired, q_dot, q_current, T_total))
        return tau

        #!/usr/bin/env python
 
    def apply_torque(self, joint_name, torque, position):
        msg = JointTrajectory()
        msg.joint_names = [joint_name]
        point = JointTrajectoryPoint()
        point.positions = [position]  # Set the joint position if required
        v =  np.gradient(position, dt=0.01, axis=0, edge_order=2)
        a =  np.gradient(v, dt=0.01, axis=0, edge_order=2)
        point.velocities = [v]  # Set the joint velocity if required
        point.accelerations = [a]
        point.effort = [torque]
        msg.points = [point]
        self.joint_trajectory_publisher.publish(msg)

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [joint_name]  # Add your joint names
        joint_state_msg.position = [position]  # Add your joint positions

        # Publish joint state information
        self.joint_state_publisher.publish(joint_state_msg)

        effort = ApplyJointEffort.Request()
        effort.joint_name = joint_name
        effort.effort = torque
        effort.start_time.sec = 0
        effort.start_time.nanosec = 0
        effort.duration.sec = 0
        effort.duration.nanosec = int(1e-8)  # Duration of 0.1 seconds
        future = self.joint_effort_srv_client.call_async(effort)

        # Wait for the result
        rclpy.spin_until_future_complete(self, future)
        # Check if the request was successful
        if future.result() is not None:
            self.get_logger().info("Effort applied successfully")
        else:
            self.get_logger().error("Failed to apply effort")
        

    def control_script(self, joints, tau, graph):
       # Specify joint name (replace 'joint1' with your joint name)
        #for i in range(4):
        for node in graph:
            for i, pos in enumerate(node):
                self.apply_torque(joints[i], tau[i], pos)

    def solve_numerical_inverse_kinematics(self, q_initial, target_position):
        q_result = q_initial.copy()

        for iteration in range(100):
            J = self.compute_direct_differential_kinematics_from_configuration(q_initial)
            error = (target_position - self.compute_forward_kinematics_from_configuration(q_initial)).reshape(-1, 1)
            error = np.vstack((error, np.zeros_like(error)))
            delta_theta = 0.1 * np.linalg.pinv(J) @ error
            q_result = q_result.astype(float)
            delta_theta = ((delta_theta[:4]).flatten()).T
            q_result += delta_theta
            q_initial = q_result
            
            if np.linalg.norm(delta_theta) < 1e-6:
                break
        
        return q_result



def main():
    rclpy.init()
    node = RobotController()
    try:
        while rclpy.ok():
            q_goal = node.solve_numerical_inverse_kinematics(np.array([0, 0, 0, 0]), np.array([1,2,1]))
            graph = node.get_trajectory(q_goal, np.array([0, 0, 0, 0]), np.array([0, 0, 0, 0]))
            tau = node.compute_motion(np.array([1,2,1]), graph)
            joints = np.array(['base_joint', 'base_spherical_joint', 'leg_joint', 'arm_joint'])
            node.control_script(joints, tau, graph)

            # Calculate end-effector position
            #end_effector_position = robot.forward_kinematics(joint_angles_values)[:3, 3]
        

            # Calculate end-effector velocity
            #jacobian_matrix = robot.direct_differential_kinematics(joint_angles_values)

            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
