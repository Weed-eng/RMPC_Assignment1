#!/usr/bin/env python3

# Note: Complete the following subfunctions to generate valid transformation matrices 
# from a translation vector and Euler angles, or a sequence of 
# successive rotations around z, y, and x.

import numpy as np

class transformation():

    @staticmethod
    def trans(d):
        """
        Calculate pure translation homogenous transformation by d
        """

        # YOUR CODE STARTS HERE

        M = np.eye(4)
        M[0:3, 3] = d[0:3]
	
        return M
    
        # YOUR CODE ENDS HERE
    
    @staticmethod
    def roll(a):
        """
        Calculate homogenous transformation for rotation around x axis by angle a
        """

        # YOUR CODE STARTS HERE

        c = np.cos(a)
        s = np.sin(a)

        T = np.eye(4)
        T[1,1] = c
        T[1,2] = -s
        T[2,1] = s
        T[2,2] = c

        return T
    
        # YOUR CODE ENDS HERE

    @staticmethod
    def pitch(a):
        """
        Calculate homogenous transformation for rotation around y axis by angle a
        """

        # YOUR CODE STARTS HERE

        c = np.cos(a)
        s = np.sin(a)

        T = np.eye(4)
        T[0,0] = c
        T[0,2] = s
        T[2,0] = -s
        T[2,2] = c

        return T
	
        # YOUR CODE ENDS HERE

    @staticmethod
    def yaw(a):
        """
        Calculate homogenous transformation for rotation around z axis by angle a
        """

        # YOUR CODE STARTS HERE

        c = np.cos(a)
        s = np.sin(a)

        T = np.eye(4)
        T[0,0] = c
        T[0,1] = -s
        T[1,0] = s
        T[1,1] = c

        return T

        # YOUR CODE ENDS HERE

    @staticmethod
    def transform(d,rpy):
        """
        Calculate a homogenous transformation for translation by d and
        rotation corresponding to roll-pitch-yaw euler angles
        """

        # YOUR CODE STARTS HERE
	
        r, p, y = rpy
        T = (transformation.trans(d) 
            @ transformation.yaw(y) 
            @ transformation.pitch(p) 
            @ transformation.roll(r))

        return T
    
        # YOUR CODE ENDS HERE
    
if __name__ == "__main__":
    pass
