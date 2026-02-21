#!/usr/bin/env python
import os
import sys
import numpy as np
from solution.solveFK import FK

class IK:

    # JOINT LIMITS
    lower = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upper = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    fk = FK()

    def __init__(self):
        pass

    @staticmethod
    def calcJacobian(q):
        """
        Calculate the Jacobian of the end effector in a given configuration.
        INPUT:
        q - 1 x 7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]
        OUTPUT:
        J - the Jacobian matrix 
        """

        J = []       

        # YOUR CODE STARTS HERE

        J = np.zeros((6,7))
        
        T = np.eye(4)
        transforms = []
        
        # Building transforms
        for i in range(7):
            a, alpha, d = IK.fk.dh_params[i]
            theta = q[i]

            T_i = IK.fk.build_dh_transform(a,alpha, d, theta)
            
            z_offset = IK.fk.joint_offset[i][2]
            T_offset = np.eye(4)
            T_offset[2,3] = z_offset

            T = T @ T_i @ T_offset
            transform.append(T.copy())
        
        T0e = T
        P_e = T0e[0:3, 3]

        # Computing Jacobian columns
        for i in range(7)
            if i == 0:
                T_prev = np.eye(4)
            else:
                T_prev = transform[i-1]

            z_i = T_prev[0:3,2]
            p_i = T_prev[0:3,3]

            J_v = np.cross(z_i, p_e - p_i)
            J_w = z_i

            J[0:3, i] = J_v
            J[3:6, i] = J_w

        # YOUR CODE ENDS HERE
        return J

    @staticmethod
    def cal_target_transform_vec(target, current):
        """
        Calculate the displacement vector and axis of rotation from 
        the current frame to the target frame

        INPUTS:
        target - 4x4 numpy array representing the desired transformation from
                 end effector to world

        current - 4x4 numpy array representing the current transformation from
                  end effector to world

        OUTPUTS:
        translate_vec - a 3-element numpy array containing the target translation vector from
                        the current frame to the target frame, expressed in the world frame

        rotate_vec - a 3-element numpy array containing the target rotation vector from
                     the current frame to the end effector frame
        """

        translate_vec = []
        rotate_vec = []

        # YOUR CODE STARTS HERE

        p_t = target[0:3, 3]
        p_c = current[0:3, 3]
        translate_vec = p_t - p_c

        R_t = target[0:3, 0:3]
        R_c = current[0:3, 0:3]

        R_err = R_c.T @ R_t

        angle = np.arccos((np.trace(R_err) - 1) / 2)

        if angle < 1e-6:
            rotate_vec = np.zeros(3)
        else:
            axis = np.array([
                 R_err[2,1] - R_err[1,2],
                 R_err[0,2] - R_err[2,0],
                 R_err[1,0] - R_err[0,1]
            ]) / (2*np.sin(angle))

            rotate_vec = angle * axis

        ## YOUR CODE ENDS HERE

        return translate_vec, rotate_vec

    def check_joint_constraints(self,q,target):
        """
        Check if the given candidate solution respects the joint limits.

        INPUTS
        q - the given solution (joint angles)

        target - 4x4 numpy array representing the desired transformation from
                 end effector to world

        OUTPUTS:
        success - True if some predefined certain conditions are met. Otherwise False
        """

        success = False

        # YOUR CODE STARTS HERE

        within_limits = np.all(q >= IK.lower) and np.all(q <= IK.upper)

        _, current = IK.fk.forward(q)
        dp, dr = IK.cal_target_transform_vec(target, current)

        pos_err = np.linalg.norm(dp)
        rot_err = np.linalg.norm(dr)

        success = within_limits and pos_err < 1e-3 and rot_err < 1e-3

        # YOUR CODE ENDS HERE

        return success


    @staticmethod
    def solve_ik(q,target):
        """
        Uses the method you prefer to calculate the joint velocity 

        INPUTS:
        q - the current joint configuration, a "best guess" so far for the final solution

        target - a 4x4 numpy array containing the target end effector pose

        OUTPUTS:
        dq - a desired joint velocity
        Note: Make sure that it will smoothly decay to zero magnitude as the task is achieved.
        """

        dq = []

        # YOUR CODE STARTS HERE

        _, current = IK.fk.forward(q)
        dp, dr = IK.cal_target_transform_vec(target, current)

        e = np.hstack((dp, dr))

        J = IK.calcJacobian(q)

        alpha = 0.5
        dq = alpha * (np.linalg.pinv(J) @ e)

        return dq

        # YOUR CODE ENDS HERE

        return dq

    def inverse(self, target, initial_guess):
        """
        Solve the inverse kinematics of the robot arm

        INPUTS:
        target - 4x4 numpy array representing the desired transformation from
        end effector to world

        initial_guess - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6], which
        is the "initial guess" from which to proceed with the solution process (has set up for you)

        OUTPUTS:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6], giving the
        solution if success is True or the closest guess if success is False.

        success - True if IK is successfully solved. Otherwise False
        """

        q = initial_guess
        success = False

        # YOUR CODE STARTS HERE

        q = initial_guess.copy()
        max_iters = 200

        for i in range(max_iters):

        dq = IK.solve_ik(q, target)

        q = q + dq

        if self.check_joint_constraints(q, target):
            success = True
            break

        # YOUR CODE ENDS HERE

        return q, success

if __name__ == "__main__":
    pass
