import numpy as np
import matplotlib.pyplot as plt

class R5PPM_Simulator:

    def __init__(self, l1: float, l2: float, l3: float, l4: float, d: float):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
        self.d = d
        # Positions of the fixed actuators
        self.A1 = np.array([-d / 2, 0])
        self.A2 = np.array([d / 2, 0])

    def inverse_kinematics(self, x: float, y: float) -> dict:
        """
        Calculates the actuator angles (theta1, theta2) for a given end-effector position (x, y).
        
        [cite_start]This method is based on the inverse position kinematics described in section 2.3 of the source paper [cite: 146-173].
        It returns a dictionary with the four possible working modes.

        Args:
            x (float): The x-coordinate of the target position.
            y (float): The y-coordinate of the target position.

        Returns:
            dict: A dictionary mapping the working mode name (e.g., '++', '+-') to a tuple 
                  of angles (theta1, theta2) in radians. Returns an empty dictionary if the 
                  point is unreachable.
        """
        P = np.array([x, y])
        
        # Calculate distances from actuators to the end-effector
        a1 = np.linalg.norm(P - self.A1)
        a2 = np.linalg.norm(P - self.A2)

        # Calculate angles alpha1 and alpha2
        alpha1 = np.arctan2(y, x + self.d / 2)
        alpha2 = np.arctan2(y, x - self.d / 2)
        
        # Calculate angles beta1 and beta2 using the law of cosines
        # Check if the target is reachable
        if self.l1 * a1 == 0 or self.l2 * a2 == 0: # Avoid division by zero
            return {}
            
        cos_beta1_arg = (self.l1**2 + a1**2 - self.l3**2) / (2 * self.l1 * a1)
        cos_beta2_arg = (self.l2**2 + a2**2 - self.l4**2) / (2 * self.l2 * a2)

        if not (abs(cos_beta1_arg) <= 1 and abs(cos_beta2_arg) <= 1):
            print(f"Warning: Target position ({x}, {y}) is unreachable.")
            return {}

        beta1 = np.arccos(cos_beta1_arg)
        beta2 = np.arccos(cos_beta2_arg)

        # [cite_start]The four working modes are determined by the combination of signs for beta1 and beta2 [cite: 171-173, 179]
        solutions = {
            '++': (alpha1 + beta1, alpha2 + beta2),
            '+-': (alpha1 + beta1, alpha2 - beta2),
            '-+': (alpha1 - beta1, alpha2 + beta2),
            '--': (alpha1 - beta1, alpha2 - beta2)
        }
        
        return solutions

    def forward_kinematics(self, theta1: float, theta2: float) -> list:
        # Calculate positions of joints B1 and B2
        B1 = self.A1 + np.array([self.l1 * np.cos(theta1), self.l1 * np.sin(theta1)])
        B2 = self.A2 + np.array([self.l2 * np.cos(theta2), self.l2 * np.sin(theta2)])

        dist_B1B2 = np.linalg.norm(B1 - B2)
        
        # Check if the circles intersect
        if dist_B1B2 == 0 or not (abs(self.l3 - self.l4) <= dist_B1B2 <= (self.l3 + self.l4)):
            return []

        # Calculate intersection points of two circles
        a = (self.l3**2 - self.l4**2 + dist_B1B2**2) / (2 * dist_B1B2)
        h_sq = self.l3**2 - a**2
        h = np.sqrt(h_sq) if h_sq >= 0 else 0

        # Midpoint on the line B1-B2
        Pm = B1 + a * (B2 - B1) / dist_B1B2
        
        # The two solutions are perpendicular to the B1-B2 line from Pm
        p1 = (Pm[0] + h * (B2[1] - B1[1]) / dist_B1B2, Pm[1] - h * (B2[0] - B1[0]) / dist_B1B2)
        p2 = (Pm[0] - h * (B2[1] - B1[1]) / dist_B1B2, Pm[1] + h * (B2[0] - B1[0]) / dist_B1B2)

        return [p1, p2]

    def compute_workspace(self, resolution: int = 200) -> tuple[np.ndarray, np.ndarray]:
        
        print(f"Computing workspace with {resolution}x{resolution} resolution... (This may take a moment)")
        theta_range = np.linspace(0, 2 * np.pi, resolution)
        points_mode1 = []
        points_mode2 = []

        for t1 in theta_range:
            for t2 in theta_range:
                solutions = self.forward_kinematics(t1, t2)
                if len(solutions) == 2:
                    points_mode1.append(solutions[0])
                    points_mode2.append(solutions[1])
        
        return np.array(points_mode1), np.array(points_mode2)

    def plot_workspace(self, resolution: int = 200):
        """
        Computes and plots the robot's workspace, distinguishing between assembly modes.

        Args:
            resolution (int): The resolution for the workspace computation.
        """
        ws_mode1, ws_mode2 = self.compute_workspace(resolution)

        if ws_mode1.size == 0 and ws_mode2.size == 0:
            print("Workspace is empty. Check link parameters.")
            return

        fig, ax = plt.subplots(figsize=(8, 8))
        ax.scatter(ws_mode1[:, 0], ws_mode1[:, 1], s=1, c='red', label='Assembly Mode 1')
        ax.scatter(ws_mode2[:, 0], ws_mode2[:, 1], s=1, c='black', label='Assembly Mode 2')

        ax.set_title('Reachable Workspace of the 5R-PPM')
        ax.set_xlabel('x (meters)')
        ax.set_ylabel('y (meters)')
        ax.grid(True)
        ax.set_aspect('equal', adjustable='box')
        ax.legend()
        plt.show()

    def _plot_arm(self, ax, theta1: float, theta2: float, target_pos: tuple, title: str):
        """Helper function to plot a single arm configuration."""
        B1 = self.A1 + np.array([self.l1 * np.cos(theta1), self.l1 * np.sin(theta1)])
        B2 = self.A2 + np.array([self.l2 * np.cos(theta2), self.l2 * np.sin(theta2)])
        P = np.array(target_pos)

        # Plot links
        ax.plot([self.A1[0], B1[0]], [self.A1[1], B1[1]], 'r-o', linewidth=2, label='Link 1 ($l_1$)')
        ax.plot([self.A2[0], B2[0]], [self.A2[1], B2[1]], 'b-o', linewidth=2, label='Link 2 ($l_2$)')
        ax.plot([B1[0], P[0]], [B1[1], P[1]], 'g-o', linewidth=2, label='Link 3 ($l_3$)')
        ax.plot([B2[0], P[0]], [B2[1], P[1]], 'm-o', linewidth=2, label='Link 4 ($l_4$)')
        
        # Plot base
        ax.plot([self.A1[0], self.A2[0]], [self.A1[1], self.A2[1]], 'k-o', linewidth=3, label='Base (d)')

        # Annotate points
        ax.text(self.A1[0], self.A1[1] - 0.05, 'A1', ha='center')
        ax.text(self.A2[0], self.A2[1] - 0.05, 'A2', ha='center')
        ax.text(B1[0], B1[1] + 0.05, 'B1', ha='center')
        ax.text(B2[0], B2[1] + 0.05, 'B2', ha='center')
        ax.text(P[0], P[1] + 0.05, 'P (Target)', ha='center', color='red', weight='bold')

        ax.set_title(f'Configuration for "{title}" Mode')
        ax.set_xlabel('x (meters)')
        ax.set_ylabel('y (meters)')
        ax.grid(True)
        ax.set_aspect('equal', adjustable='box')
        ax.legend()

    def plot_ik_solutions(self, x: float, y: float):
        """
        Calculates and plots all four possible IK solutions for a target position.
        
        Args:
            x (float): The x-coordinate of the target position.
            y (float): The y-coordinate of the target position.
        """
        solutions = self.inverse_kinematics(x, y)
        if not solutions:
            return

        fig, axs = plt.subplots(2, 2, figsize=(14, 12))
        fig.suptitle(f'Four Inverse Kinematic Solutions for Target P({x}, {y})', fontsize=16)
        
        # Flatten the axes array for easy iteration
        axs = axs.ravel()
        
        for i, (mode, (theta1, theta2)) in enumerate(solutions.items()):
            self._plot_arm(axs[i], theta1, theta2, (x, y), mode)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.show()


if __name__ == '__main__':
    # Examplelink parame ters
    link_params = {
        "l1": 0.4,  # meter
        "l2": 0.4,  # meter
        "l3": 0.4,  # meter
        "l4": 0.4,  # meter
        "d": 0.62   # meter
    }

    # Create a simulator instance with the new parameters
    robot = R5PPM_Simulator(**link_params)

    # Define a new target position that is reachable with the current link parameters.
    target_x, target_y = 0.2, 0.3

    print(f"Calculating and plotting Inverse Kinematics for target P({target_x}, {target_y})...")
    robot.plot_ik_solutions(target_x, target_y)