import numpy as np
import matplotlib.pyplot as plt

class R5PPM_Simulator:
    """
    A class to simulate the forward and inverse kinematics of a 5R Planar Parallel Mechanism (5R-PPM).
    
    The dimensions and kinematic conventions are based on the paper:
    "Optimized design of 5R planar parallel mechanism aimed at gait-cycle of quadruped robots" 
    by Mangesh D. Ratolikar and Prasanth Kumar R.
    """

    def __init__(self, l1: float, l2: float, l3: float, l4: float, d: float):
        """
        Initializes the 5R-PPM with its geometric parameters.
        """
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
        self.d = d
        self.A1 = np.array([-d / 2, 0])
        self.A2 = np.array([d / 2, 0])
        # Store the last valid angles to hold pose when unreachable
        self.last_valid_angles = (np.pi/2, np.pi/2) 

    def get_mode_for_y_pos(self, y: float) -> str:

        """
        if y is negative, use '-+' mode;
        otherwise, use '+-' mode.
        """
        if y < 0:
            return '-+'
        else:
            return '+-'

    def inverse_kinematics(self, x: float, y: float) -> dict:
        """
        Calculates the actuator angles (theta1, theta2) for a given end-effector position (x, y).
        """
        P = np.array([x, y])
        a1 = np.linalg.norm(P - self.A1)
        a2 = np.linalg.norm(P - self.A2)

        alpha1 = np.arctan2(y, x + self.d / 2)
        alpha2 = np.arctan2(y, x - self.d / 2)
        
        if self.l1 * a1 == 0 or self.l2 * a2 == 0:
            return {}
            
        cos_beta1_arg = (self.l1**2 + a1**2 - self.l3**2) / (2 * self.l1 * a1)
        cos_beta2_arg = (self.l2**2 + a2**2 - self.l4**2) / (2 * self.l2 * a2)

        if not (abs(cos_beta1_arg) <= 1 and abs(cos_beta2_arg) <= 1):
            return {}

        beta1 = np.arccos(cos_beta1_arg)
        beta2 = np.arccos(cos_beta2_arg)

        solutions = {
            '++': (alpha1 + beta1, alpha2 + beta2),
            '+-': (alpha1 + beta1, alpha2 - beta2),
            '-+': (alpha1 - beta1, alpha2 + beta2),
            '--': (alpha1 - beta1, alpha2 - beta2)
        }
                
        return solutions