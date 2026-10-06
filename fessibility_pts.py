from Parallel_5r_arm import R5PPM_Simulator

link_params = {
    "l1": 0.4,  # meter
    "l2": 0.4,  # meter
    "l3": 0.4,  # meter
    "l4": 0.4,  # meter
    "d": 0.62   # meter
}

# Create a simulator instance with the new parameters
robot = R5PPM_Simulator(**link_params)

# A lower resolution (e.g., 100) will be faster for a quick check.
# A higher resolution (e.g., 300) will produce a more detailed plot.
robot.plot_workspace(resolution=300)