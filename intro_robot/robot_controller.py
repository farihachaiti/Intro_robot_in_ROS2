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
        self.declare_parameter('robot_description', '')
        if self.has_parameter('robot_description'):
            self.robot_description = self.get_parameter('robot_description').get_parameter_value().string_value
        else:
            self.get_logger().error("Parameter 'robot_description' is missing!")
        # Parse the URDF
        try:
            with open(self.robot_description, 'r') as urdf_file:
                self.robot_desc = URDF.from_xml_string(urdf_file.read())
        except Exception as e:
            self.get_logger().error(f"Error parsing URDF: {e}")
            self.robot_desc = None
            # Wait for the service to be available
        while not self.joint_effort_srv_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service not available, waiting...')


    '''def listen_tf(self):
        try:
            transform1 = self.tf_buffer.lookup_transform('base_link', 'leg_link', rclpy.time.Time())
            transform2 = self.tf_buffer.lookup_transform('leg_link', 'arm_link', rclpy.time.Time())
            transform3 = self.tf_buffer.lookup_transform('arm_link', 'end_effector', rclpy.time.Time())
            transform4 = self.tf_buffer.lookup_transform('end_effector', 'tool_mic', rclpy.time.Time())
            self.get_logger().info(f'Transform received from base link to leg link: {transform1}')
            self.get_logger().info(f'Transform received from leg link to arm link: {transform2}')
            self.get_logger().info(f'Transform received from arm link to end effector: {transform3}')
            self.get_logger().info(f'Transform received from end effector to microphone tool: {transform4}')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Failed to lookup transform: {e}')'''

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
            q_random = [np.random.randint(q_goal[0]-1, q_goal[0]+1), np.random.randint(q_goal[1]-1, q_goal[1]+1), np.random.randint(q_goal[2]-1, q_goal[2]+1), np.random.randint(q_goal[3]-1, q_goal[3]+1)]
            if self.is_clear(q_random, graph):
                q_nearest = self.nearest(graph, q_random) 
                q_new = self.extend(q_nearest, q_random)
                min_cost = self.get_cost(q_goal,q_nearest)
        
                near_nodes = [node for node in graph if (self.get_cost(q_new, node) <= min_cost)] 
                for idx, near_node in enumerate(near_nodes):
                    if self.is_clear(near_node, graph) and np.all(np.array(graph))!=0:                   
                        if (self.get_cost(q_goal, near_node) + self.get_cost(q_new, near_node)) < min_cost:
                            min_cost = self.get_cost(q_goal, near_node) + self.get_cost(q_new, near_node)
                if (self.is_joint_okay(float(q_new))) and (not self.is_singular(self.compute_forward_kinematics_from_configuration(float(q_new)))):
                    graph.append(float(q_new))
                else:
                    q_new += np.array([0.1, 0.1, 0.1, 0.1])
                    graph.append(float(q_new))
                for idx, near_node in enumerate(near_nodes):
                    if self.is_clear(near_node, graph) and np.all(np.array(graph))!=0: 
                        if (self.get_cost(q_goal, q_new) + self.get_cost(q_new, near_node))<self.get_cost(q_goal, near_node):
                            self.rewire(near_nodes, idx, q_new)  

                
                    q_init = q_new    
        return graph



    def compute_inertia_matrix(self):
        M = np.zeros((5, 3, 3))
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
                    M[idx]  = np.array([inertial.inertia.ixx, -inertial.inertia.ixy, -inertial.inertia.ixz, 0, 0],
                        [-inertial.inertia.ixy[idx], inertial.inertia.iyy[idx], -inertial.inertia.iyz, 0, 0],
                        [-inertial.inertia.ixz, -inertial.inertia.iyz, inertial.inertia.izz, 0, 0],
                        [0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0])


        return M
            

    def compute_coriolis_matrix(self, M, q, qd):
        n = len(q)
        C = np.zeros((n, n))
        q_alt_symbols = {f'q{k}': sp.symbols(f'q{k}') for k in range(n)}
        qd_alt_symbols = {f'qd{k}': sp.symbols(f'qd{k}') for k in range(n)}
        for k in range(n):
            for i in range(n):
                for j in range(n):
                    if q[k] != 0.0 or qd[k] != 0.0:
                        q_alt = q_alt_symbols[f'q{k}']
                        qd_alt = qd_alt_symbols[f'qd{k}']
 
                        C[i, j] += 0.5 * (sp.diff(M[k][i, j], q_alt) - sp.diff(M[k][j, i], q_alt)) * qd_alt
                    else:
                        # Handle the case where the joint position is zero
                        # For example, set the derivative to zero or handle it based on your system's dynamics
                        C[i, j] += 0.0  # Set derivative to zero

        return C



    def inverse_dynamics(self, qdd_desired, qd_desired, q_desired, qd_current, q_current, Kp, Kd):
        # Define symbolic variables
        M  = self.compute_inertia_matrix()
        C = self.compute_coriolis_matrix(M, q_current, qd_current)
        G = 9.81
        for i in range(4):
            tau = np.dot(M[i], (qdd_desired[i] + np.dot(Kd, (qd_desired[i] - qd_current[i])) + np.dot(Kp, (q_desired[i] - q_current[i])))) + np.dot(C, qd_current[i]) + G


        # Joint positions and velocities
        #q = sp.Matrix([q1, q2])
        #qdot = sp.Matrix([sp.diff(qi) for qi in q])

        return tau

    def compute_forward_kinematics_from_configuration(self, q):

        T01, T12, T23, T34, T4ee = self.compute_transform_matrix(q)
        T_total = T01 @ T12 @ T23 @ T34 @ T4ee
        
        return  np.array([T_total[0, 3], T_total[1, 3], T_total[2, 3]])
    

    
    def compute_transform_matrix(self, q, theta5 = 30):
        # Extract joint angles and prismatic displacements
        theta1, theta2, d3, theta4 = q
        

        # Define robot parameters (replace these with the actual parameters of your robot)
        L1 = 0.1
        L2 = 0.6
        L3 = 0.6
        L4 = 0.6

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
        [-np.sin(np.radians(theta5)), 0, np.cos(np.radians(theta5)), 0],
        [0, 0, 0, 1]
        ])

        return T01, T12, T23, T34, T4ee


    def compute_direct_differential_kinematics_from_configuration(self, q, theta5):
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
        
        Z1 = np.dot(T01[:3,:3],np.array([[1], [0], [0]]))
        Z2 = np.dot(T02[:3,:3],np.array([[1], [0], [0]]))
        Z4 = np.dot(T03[:3,:3],np.array([[1], [0], [0]]))
        Z3 = np.dot(T04[:3,:3],np.array([[0], [0], [d3]]))
        Z5 = np.dot(T0ee[:3,:3],np.array([[0], [1], [0]]))

        J1 = np.vstack((np.cross(Z1, (P0ee - P01), axis=0), Z1))
        J2 = np.vstack((np.cross(Z2, (P0ee - P02), axis=0), Z2))
        J3 = np.vstack((np.array(Z3), np.array([[0],[0],[0]])))
        J4 = np.vstack((np.cross(Z4, (P0ee - P03), axis=0), Z4))
        J5 = np.vstack((np.cross(Z5, (P0ee - P04), axis=0), Z5))

        J = np.hstack((J1, J2, J3, J4, J5))
       
        return J

    def compute_motion(self, p_desired, graph, Kp, Kd):
        # Given end effector position matrix 
        p_desired = np.array([1, 2, 1])
        tau = []
        # Initial guess for joint positions
        #q_current = np.array([0, 0, 0, 0])

        # Desired tolerance for convergence
        tolerance = 3

        # Maximum number of iterations

        # Numerical inverse kinematics using the Jacobian inverse method
        for q_current in graph:
            p_current = self.compute_forward_kinematics_from_configuration(q_current)
            error = np.linalg.norm(np.array(p_desired) - np.array(p_current))

            # Check if the error is below the tolerance
            if np.linalg.norm(error) < tolerance:
                print("Converged!")
                q_desired = q_current               
                q_dot_desired = np.gradient(q_desired, 0.01)
                q_ddot_desired = np.gradient(q_dot_desired, 0.01)
                q_dot = np.gradient(q_current, 0.01)

                # Update joint positions using the computed velocities
                q_current += q_dot * 0.01
                # Print the result
                tau.append(self.inverse_dynamics(q_ddot_desired, q_dot_desired, q_desired, q_dot, q_current, Kp, Kd))
                break
            else:
                print("Invalid joint configuration!")
                continue
        # Compute Jacobian matrix
        #J = compute_jacobian(q_current)

        # Compute joint velocities using the Jacobian inverse

        return tau
 
    def apply_torque(self, joint_name, torque, position):
        msg = JointTrajectory()
        msg.joint_names = [joint_name]
        point = JointTrajectoryPoint()
        point.positions = [position]  # Set the joint position if required
        v =  np.gradient(position, 0.01)
        a =  np.gradient(v, 0.01)
        point.velocities = [v]  # Set the joint velocity if required
        point.accelerations = [a]
        point.effort = [torque]
        msg.points = [point]
        self.joint_trajectory_publisher.publish(msg)

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

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [joint_name]  # Add your joint names
        joint_state_msg.position = [position]  # Add your joint positions

        # Publish joint state information
        self.joint_state_publisher.publish(joint_state_msg)
        # Check if the request was successful
        if future.result() is not None:
            self.get_logger().info("Effort applied successfully")
        else:
            self.get_logger().error("Failed to apply effort")
        

    def control_script(self, joints, tau, graph, Kp, Kd):
       # Specify joint name (replace 'joint1' with your joint name)
        #for i in range(4):
        for node in graph:
            node = self.minimize_distance(node)
            for i, pos in enumerate(node):
                errors = graph[-1][i][pos] - pos
                v = np.gradient(pos, 0.01)

                # Compute the PD control signal
                pd_signal = Kp * errors + Kd * v

                # Add the PD control signal to the torques
                tau_modified = tau[i] + pd_signal
                if i<=3:
                    self.apply_torque(joints[i], tau_modified, pos)
                else:
                    joint_state_msg = JointState()
                    joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                    joint_state_msg.name = [joints[i]]  # Add your joint names
                    joint_state_msg.position = [np.radians(30)]  # Add your joint positions

                    # Publish joint state information
                    self.joint_state_publisher.publish(joint_state_msg)
                break


    def is_singular(self, J):
        okay = False
        num_dofs = 5  # Example: 3 degrees of freedom in task space
        if np.linalg.cond(J)>1e6:
            print('Singularity alert!')
            print('one')
        if np.linalg.matrix_rank(J)<num_dofs:
            print('Singularity alert!')
            print('two')
        elif np.linalg.eigvals(J) <=0.5:
            print('Singularity alert')
            print('three')
        else:
            print('no singularity! everything is fine :)!')
            okay = True
        print(np.linalg.cond(J))
        print(1e6)
        print(J.shape)
        print(J)
        print(J[:, 1])
        # Perform singular value decomposition (SVD) on the Jacobian matrix
        if not okay:
            U, s, V = np.linalg.svd(J)

            # Find linearly dependent columns (where singular values are close to zero)
            linearly_dependent_indices = np.where(s < 1e-10)[0]


            print('Problematics columns:')
            for col_index in linearly_dependent_indices:                
                print(np.linalg.matrix_rank(J[:, col_index]))
            return True
        else:
            return False

    def solve_numerical_inverse_kinematics(self, q_initial, target_position, theta5):
        print(q_initial)
        q_result = q_initial.copy()
        J = self.compute_direct_differential_kinematics_from_configuration(q_initial, theta5)

        for iteration in range(100):
        
            error = (target_position - self.compute_forward_kinematics_from_configuration(q_initial)).reshape(-1, 1)
            error = np.vstack((error, np.zeros_like(error)))
            delta_theta = 0.1 * np.linalg.pinv(J) @ error
            q_result = q_result.astype(float)
            delta_theta = ((delta_theta[:4]).flatten()).T
            q_result += delta_theta
            q_initial = q_result
            
            if np.linalg.norm(delta_theta) < 1e-6:
                if self.is_joint_okay(q_result):
                    break
                else:
                    continue
                        
                                
        return q_result.astype(int)
    

    def is_joint_okay(self, q_result):
        for i, joint in enumerate(self.robot_desc.joints):
            if q_result[i]>=joint.limit.lower and q_result[i]<=joint.limit.upper:
                print('Joint conf okay')
                if i==3:
                    break
            else:
                return False
        return True



    def minimize_distance(self, q_goal, theta5=30):
        null_step = 0.01
        q_goal_minimized = q_goal - (theta5 * null_step)
        if self.is_joint_okay(q_goal_minimized):
            return q_goal_minimized
        else:
            return q_goal
       


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    Kp = 0.5
    Kd = 0.1
    theta5 = 30
    try:
        while rclpy.ok():
            q_goal = node.solve_numerical_inverse_kinematics(np.array([0, 0, 0, 0]), np.array([1,2,1]), theta5)
            q_goal = node.minimize_distance(q_goal, theta5)
            graph = node.get_trajectory(q_goal, np.array([0, 0, 0, 0]))
            print(graph)
            '''tau = node.compute_motion(np.array([1,2,1]), graph, Kp, Kd)
            joints = np.array(['base_joint', 'base_spherical_joint', 'leg_joint', 'arm_joint', 'end_effector_joint'])
            node.control_script(joints, tau, graph, Kp, Kd)'''
            
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
