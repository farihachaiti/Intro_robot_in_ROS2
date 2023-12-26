import numpy as np
import sympy as sp
#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ApplyJointEffort
from std_msgs.msg import Float64

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

    def get_trajectory(self, q_goal, q_init, q_parent):
         graph = [q_init]
         while self.get_cost(q_goal,q_init) < self.get_cost(q_goal,q_parent):
            q_random = Node(np.random.uniform(0, 10), np.random.uniform(0, 10), np.random.uniform(0, 10), np.random.uniform(0, 10))
            if self.is_clear(q_random):
                q_nearest = self.nearest(graph, q_random)
                q_new = self.extend(q_nearest, q_random)
                near_nodes = [node for node in graph if self.get_cost(q_new, node) < max_distance]

                for near_node in near_nodes:
                    if self.get_cost(q_goal, near_node) + self.get_cost(q_new, near_node) < min_cost and self.is_clear(near_node):
                        min_cost_node = near_node
                        min_cost = self.get_cost(q_goal, q_nearest) + self.get_cost(q_new, q_nearest)
                q_init = q_new
                q_parent = min_cost_node
                graph.append(q_new)
                self.rewire(graph, q_new, self.get_cost(q_goal,q_parent))


    def compute_inertia_matrix(self, Ixx, Iyy, Izz, T, m):
            I0_local = sp.diag(Ixx[0], Iyy[0], Izz[0])
            I1_local = sp.diag(Ixx[1], Iyy[1], Izz[1])
            I2_local = sp.diag(Ixx[2], Iyy[2], Izz[2])
            I3_local = sp.diag(Ixx[3], Iyy[3], Izz[3])
            I4_local = sp.diag(Ixx[4], Iyy[4], Izz[4])

            I0_base = T[:3, :3].T * I0_local * T[:3, :3]
            I1_base = T[:3, :3].T * I1_local * T[:3, :3]
            I2_base = T[:3, :3].T * I2_local * T[:3, :3]
            I3_base = T[:3, :3].T * I3_local * T[:3, :3]
            I4_base = T[:3, :3].T * I4_local * T[:3, :3]

            # Compute the inertia matrix M symbolically
            M = m[0] * I0_base + m[1] * I1_base + m[2] * I2_base + m[3] * I3_base + m[4] * I4_base

            # Simplify the result
            return sp.simplify(M)
            

    def compute_coriolis_matrix(self, M, q, q_dot):
            n = len(q)
            C = np.zeros((n, n))

            for k in range(n):
                for j in range(n):
                    C[k, j] = 0.5 * (self.partial(M[k, j], q, q_dot) + self.partial(M[k, j], q, q_dot) - self.partial(M[j, k], q, q_dot))

            return C

    def partial(self, expr, q, q_dot):
            n = len(q)
            result = 0

            for i in range(n):
                result += diff(expr, q[i]) * q_dot[i]

            return result

    def diff(self, expr, var):
            # Placeholder for symbolic differentiation (you might want to use a symbolic library for exact derivatives)
            # This example assumes numerical differentiation for simplicity
            h = 1e-6
            expr_at_var_plus_h = expr.subs(var, var + h)
            expr_at_var_minus_h = expr.subs(var, var - h)
            return (expr_at_var_plus_h - expr_at_var_minus_h) / (2 * h)



    def inverse_dynamics(self, qdd_desired, qd_desired, q_desired, qd_current, q_current):
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
            # Extract joint angles and prismatic displacements
            theta1, d2, theta3, theta4 = q
            
            # Define robot parameters (replace these with the actual parameters of your robot)
            L1 = 0.1
            L2 = 0.6
            L3 = 0.6
            L4 = 0.6

            T01 =  np.array([
            [1, 0, 0, -L1],
            [0, np.cos(theta1), -np.sin(theta1), 0],
            [0, np.sin(theta1), np.cos(theta1), 0],
            [0, 0, 0, 1]
        ])
            
            T12 =  np.array([
            [1, 0, 0, -d2-L2],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
            
            T23 =  np.array([
            [np.cos(theta3), 0, -np.sin(theta3), 0],
            [0, 1, 0, 0],
            [np.sin(theta3), 0, np.cos(theta3), L3],
            [0, 0, 0, 1]
        ])
            
            T3ee =  np.array([
            [np.cos(theta4), 0, -np.sin(theta4), -L4],
            [0, 1, 0, 0],
            [np.sin(theta4), 0, np.cos(theta4), 0],
            [0, 0, 0, 1]
        ])
            T_total = T01 @ T12 @ T23 @ T3ee

            x = T_total[0, 3]
            y = T_total[1, 3]
            z = T_total[2, 3]
            # Forward kinematics equations
            #x = L1 * np.cos(theta1) + L3 * np.cos(theta1 + theta3) + L4 * np.cos(theta1 + theta3 + theta4)
            #y = L1 * np.sin(theta1) + L3 * np.sin(theta1 + theta3) + L4 * np.sin(theta1 + theta3 + theta4)
            #z = d2 + L2 + L5
            
            return np.array([x, y, z])

    def compute_motion(self, p_desired):

            # Given end effector position matrix 
            p_desired = np.array([1, 2, 1])

            # Initial guess for joint positions
            q_current = np.array([0, 0, 0, 0])

            # Desired tolerance for convergence
            tolerance = 1e-6

            # Maximum number of iterations
            max_iterations = 100

            # Numerical inverse kinematics using the Jacobian inverse method
            for iteration in range(max_iterations):
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
                q_current += q_dot
            
            # Print the result
            return self.inverse_dynamics(q_ddot_desired, q_dot_desired, q_desired, q_dot, q_current)
      

        #!/usr/bin/env python
 
    def apply_torque(self, joint_name, torque):
        pub = self.create_publisher(ApplyJointEffort, '/gazebo/apply_joint_effort', 10)
        effort = ApplyJointEffort()
        effort.joint_name = joint_name
        effort.effort = torque
        effort.start_time.sec = 0
        effort.start_time.nanosec = 0
        effort.duration.sec = 0
        effort.duration.nanosec = int(1e8)  # Duration of 0.1 seconds
        pub.publish(effort)

    def control_script(self, joints, tau):

       # Specify joint name (replace 'joint1' with your joint name)
        for i in range(5):
            self.apply_torque(joints[i], tau[i])

 

def main():
    rclpy.init()
    node = RobotController()
    tau = node.compute_motion(np.array([1,2,1]))
    joints = np.array(['base_joint', 'base_spherical_joint', 'neck_prismatic_joint', 'arm_joint', 'end_effector_joint'])
    node.control_script(tau, joints)

    # Calculate end-effector position
    #end_effector_position = robot.forward_kinematics(joint_angles_values)[:3, 3]
 

    # Calculate end-effector velocity
    #jacobian_matrix = robot.direct_differential_kinematics(joint_angles_values)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
