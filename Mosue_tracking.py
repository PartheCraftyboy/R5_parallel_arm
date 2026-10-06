import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from r5ppm_simulator import R5PPM_Simulator

# --- Configuration ---
# Use the symmetric link parameters
link_params = {
    "l1": 0.4, "l2": 0.4, "l3": 0.4, "l4": 0.4, "d": 0.62
}

# Create a robot instance
robot = R5PPM_Simulator(**link_params)

# --- Matplotlib Setup ---
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_aspect('equal')
# The title will be set dynamically in the update loop
title = ax.set_title("5R-PPM Mouse Tracker") 
ax.set_xlabel("x (meters)")
ax.set_ylabel("y (meters)")
ax.set_xlim(-1.0, 1.0)
ax.set_ylim(-0.5, 1.0)
ax.grid(True)
# Draw the y=0 line to make the switch obvious
ax.axhline(0, color='blue', linestyle='--', label='Mode Switch Line (y=0)')

# --- Global state for mouse position ---
mouse_pos = {'x': 0.0, 'y': 0.4}

# --- Plot Elements ---
line_l1, = ax.plot([], [], 'r-o', lw=2, label='Link 1 ($l_1$)')
line_l2, = ax.plot([], [], 'b-o', lw=2, label='Link 2 ($l_2$)')
line_l3, = ax.plot([], [], 'g-o', lw=2, label='Link 3 ($l_3$)')
line_l4, = ax.plot([], [], 'm-o', lw=2, label='Link 4 ($l_4$)')
line_base, = ax.plot([], [], 'k-o', lw=3, label='Base (d)')
target_marker, = ax.plot([], [], '+', color='red', markersize=10)
status_text = ax.text(0.05, 0.95, '', transform=ax.transAxes, verticalalignment='top')
ax.legend()


def on_mouse_move(event):
    """Callback function to update the global mouse position."""
    if event.inaxes:
        mouse_pos['x'] = event.xdata
        mouse_pos['y'] = event.ydata

# Connect the mouse movement event to the callback function
fig.canvas.mpl_connect('motion_notify_event', on_mouse_move)

def update(frame):
    """The main animation loop."""
    target_x, target_y = mouse_pos['x'], mouse_pos['y']
    
    # Dynamically select the working mode based on cursor's y-position
    active_mode = robot.get_mode_for_y_pos(target_y)
    
    # Update the plot title to show the current mode
    title.set_text(f"5R-PPM Mouse Tracker (Using '{active_mode}' mode)")

    solutions = robot.inverse_kinematics(target_x, target_y)
    
    if solutions:
        theta1, theta2 = solutions[active_mode]
        # Store the last valid angles for the current mode
        robot.last_valid_angles = (theta1, theta2)
        status = "Reachable"
        color = 'green'
    else:
        # If unreachable, hold the last valid position
        theta1, theta2 = robot.last_valid_angles
        status = "Unreachable"
        color = 'red'

    # Calculate joint positions
    B1 = robot.A1 + np.array([robot.l1 * np.cos(theta1), robot.l1 * np.sin(theta1)])
    B2 = robot.A2 + np.array([robot.l2 * np.cos(theta2), robot.l2 * np.sin(theta2)])
    P = np.array([target_x, target_y])

    # Update the plot data for each link
    line_l1.set_data([robot.A1[0], B1[0]], [robot.A1[1], B1[1]])
    line_l2.set_data([robot.A2[0], B2[0]], [robot.A2[1], B2[1]])
    line_l3.set_data([B1[0], P[0]], [B1[1], P[1]])
    line_l4.set_data([B2[0], P[0]], [B2[1], P[1]])
    line_base.set_data([robot.A1[0], robot.A2[0]], [robot.A1[1], robot.A2[1]])
    
    target_marker.set_data([target_x], [target_y])
    status_text.set_text(f'Target: ({target_x:.2f}, {target_y:.2f})\nStatus: {status}')
    status_text.set_color(color)

    # Return the updated artists
    return line_l1, line_l2, line_l3, line_l4, line_base, target_marker, status_text, title

# Create and run the animation
ani = FuncAnimation(fig, update, interval=30, blit=False)

plt.show()