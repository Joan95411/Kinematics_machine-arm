import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import tkinter as tk
from tkinter import *
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

red_block_picked_up = [False, 'none']
blue_block_picked_up = [False, 'none']
green_block_picked_up = [False, 'none']
def forward_kinematics(theta1, theta2, theta3):
    # Convert joint angles from degrees to radians
    theta1 = np.radians(theta1)
    theta2 = np.radians(theta2)
    theta3 = np.radians(theta3)

    # Compute end effector position
    x = link1 * np.cos(theta1) + link2 * np.cos(theta2) + link3 * np.cos(theta3)
    y = link1 * np.sin(theta1) + link2 * np.sin(theta2) + link3 * np.sin(theta3)

    return x, y, 0  # Return z-coordinate as 0 in a 2D planar robot


def update(val):
    global red_block_picked_up
    global red_box_x
    global red_box_y
    global blue_block_picked_up
    global blue_box_x
    global blue_box_y
    global green_block_picked_up
    global green_box_x
    global green_box_y
    # Retrieve slider values
    theta1 = -s1.val
    theta2 = -s2.val
    theta3 = -s3.val
    theta4 = -s4.val+180
    theta5 = -s5.val+180
    theta6 = -s6.val+180
    x, y, _ = forward_kinematics(theta1, theta2, theta3)
    x=x-4
    y=y+0.25
    p,q,_= forward_kinematics(theta4, theta5,theta6)
    p=p+4
    q=q+0.25
    # Update plot with new angles
    ax.clear()
    ax.axis([-12, 12, -2, 6])
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)

    # Plotting the ground

    ax.set_xlim(-12, 12)
    ax.set_xticks(np.arange(-12, 13, 1))
    ax.plot([-12, 12], [0, 0], 'k-')
    ax.plot([-4, -4], [0, 0.25], 'black', linewidth=3)
    x1 = link1 * np.cos(np.radians(theta1))-4
    y1 = link1 * np.sin(np.radians(theta1))+0.25
    ax.plot([-4, x1], [0.25, y1], 'teal', linewidth=3)

    x2 = x1 + link2 * np.cos(np.radians(theta2))
    y2 = y1 + link2 * np.sin(np.radians(theta2))
    ax.plot([x1, x2], [y1, y2], 'purple', linewidth=3)

    x3 = x2 + link3 * np.cos(np.radians(theta3))
    y3 = y2 + link3 * np.sin(np.radians(theta3))
    # Draw the end effector without picking up the box
    ax.plot([x3 - link3, x3 + link3], [y3, y3], 'black', linewidth=2)  # Horizontal line
    ax.plot([x3 - link3, x3 - link3], [y3 - link3, y3], 'black', linewidth=2)  # Left vertical line
    ax.plot([x3 + link3, x3 + link3], [y3 - link3, y3], 'black', linewidth=2)  # Right vertical line
    # Line segment from end of arm to end-effector
    ax.plot([x2, x3], [y2, y3], 'black', linewidth=2)

    ax.plot([4, 4], [0, 0.25], 'black', linewidth=3)
    x4 = link1 * np.cos(np.radians(theta4))+4
    y4 = link1 * np.sin(np.radians(theta4))+0.25
    ax.plot([4, x4 ], [0.25, y4], 'teal', linewidth=3)

    x5 = x4 + link2 * np.cos(np.radians(theta5))
    y5 = y4 + link2 * np.sin(np.radians(theta5))
    ax.plot([x4, x5], [y4, y5], 'purple', linewidth=3)

    x6 = x5 + link3 * np.cos(np.radians(theta6))
    y6 = y5 + link3 * np.sin(np.radians(theta6))
    # Draw the end effector without picking up the box
    ax.plot([x6 - link3, x6 + link3], [y6, y6], 'black', linewidth=2)  # Horizontal line
    ax.plot([x6 - link3, x6 - link3], [y6 - link3, y6], 'black', linewidth=2)  # Left vertical line
    ax.plot([x6 + link3, x6 + link3], [y6 - link3, y6], 'black', linewidth=2)  # Right vertical line
    # Line segment from end of arm to end-effector
    ax.plot([x5, x6], [y5, y6], 'black', linewidth=2)
    ax.plot(red_area, 0.5, 'rx', markersize=10)
    ax.plot(blue_area, 0.5, 'bx', markersize=10)
    ax.plot(green_area, 0.5, 'gx', markersize=10)


    if red_box_x-0.05<=x<=red_box_x+0.05 and red_box_y+1-0.05<=y<=red_box_y+1+0.05:
        print('red box picked up by left arm')
        red_block_picked_up[0] = True
        red_block_picked_up[1] = 'left'
        ax.add_patch(plt.Rectangle((x3 - 0.5, y3 - 1 ), 1, 1, color='yellow'))
        c_label.config(text='Red box is picked up!')
    if red_area-0.05<=x<=red_area+0.05 and 0.95<=y<=1.05:
        print('red box dropped down by left arm')
        red_block_picked_up[0] = False
        red_box_x=red_area
        red_box_y=0
        ax.add_patch(plt.Rectangle((x3 - 0.5, y3 - 1), 1, 1, color='yellow'))
        c_label.config(text='Red box is dropped down!')
    if red_box_x-0.05<=p<=red_box_x+0.05 and red_box_y+1-0.05<=q<=red_box_y+1+0.05:
        print('red box picked up by right arm')
        red_block_picked_up[0] = True
        red_block_picked_up[1] = 'right'
        ax.add_patch(plt.Rectangle((x6 - 0.5, y6 -1 ), 1, 1, color='yellow'))
        c_label.config(text='Red box is picked up!')
    if red_area-0.05<=p<=red_area+0.05 and 0.95<=q<=1.05:
        print('red box dropped down by right arm')
        red_block_picked_up[0] = False
        red_box_x=red_area
        red_box_y = 0
        ax.add_patch(plt.Rectangle((x6 - 0.5, y6 - 1), 1, 1, color='yellow'))
        c_label.config(text='Red box is dropped down!')
    else:
        if red_block_picked_up[0] == True and red_block_picked_up[1]=='left':
            ax.add_patch(plt.Rectangle((x3 - 0.5, y3 - 1), 1, 1, color='red'))
        elif red_block_picked_up[0] == True and red_block_picked_up[1]=='right':
            ax.add_patch(plt.Rectangle((x6 - 0.5, y6 - 1), 1, 1, color='red'))
        else:
            ax.add_patch(plt.Rectangle((red_box_x-0.5, red_box_y), 1, 1, color='red'))


    if blue_box_x-0.05<=x<=blue_box_x+0.05 and blue_box_y+1-0.05<=y<=blue_box_y+1+0.05:
        print('blue box picked up by left arm')
        blue_block_picked_up[0] = True
        blue_block_picked_up[1] = 'left'
        ax.add_patch(plt.Rectangle((x3 - 0.5, y3 - 1 ), 1, 1, color='yellow'))
        c_label.config(text='Blue box is picked up!')
    if blue_area-0.05<=x<=blue_area+0.05 and 0.95<=y<=1.05:
        print('blue box dropped down by left arm')
        blue_block_picked_up[0] = False
        blue_box_x=blue_area
        blue_box_y=0
        ax.add_patch(plt.Rectangle((x3 - 0.5, y3 - 1), 1, 1, color='yellow'))
        c_label.config(text='Blue box is dropped down!')
    if blue_box_x-0.05<=p<=blue_box_x+0.05 and blue_box_y+1-0.05<=q<=blue_box_y+1+0.05:
        print('blue box picked up by right arm')
        blue_block_picked_up[0] = True
        blue_block_picked_up[1] = 'right'
        ax.add_patch(plt.Rectangle((x6 - 0.5, y6 -1 ), 1, 1, color='yellow'))
        c_label.config(text='Blue box is picked up!')
    if blue_area-0.05<=p<=blue_area+0.05 and 0.95<=q<=1.05:
        print('blue box dropped down by right arm')
        blue_block_picked_up[0] = False
        blue_box_x=blue_area
        blue_box_y = 0
        ax.add_patch(plt.Rectangle((x6 - 0.5, y6 - 1), 1, 1, color='yellow'))
        c_label.config(text='Blue box is dropped down!')
    else:
        if blue_block_picked_up[0] == True and blue_block_picked_up[1]=='left':
            ax.add_patch(plt.Rectangle((x3 - 0.5, y3 - 1), 1, 1, color='blue'))
        elif blue_block_picked_up[0] == True and blue_block_picked_up[1]=='right':
            ax.add_patch(plt.Rectangle((x6 - 0.5, y6 - 1), 1, 1, color='blue'))
        else:
            ax.add_patch(plt.Rectangle((blue_box_x-0.5, blue_box_y), 1, 1, color='blue'))


    if green_box_x-0.05<=x<=green_box_x+0.05 and green_box_y+1-0.05<=y<=green_box_y+1+0.05:
        print('green box picked up by left arm')
        green_block_picked_up[0] = True
        green_block_picked_up[1] = 'left'
        ax.add_patch(plt.Rectangle((x3 - 0.5, y3 - 1 ), 1, 1, color='yellow'))
        c_label.config(text='Green box is picked up!')
    if green_area-0.05<=x<=green_area+0.05 and 0.95<=y<=1.05:
        print('green box dropped down by left arm')
        green_block_picked_up[0] = False
        green_box_x=green_area
        green_box_y=0
        ax.add_patch(plt.Rectangle((x3 - 0.5, y3 - 1), 1, 1, color='yellow'))
        c_label.config(text='Green box is dropped down!')
    if green_box_x-0.05<=p<=green_box_x+0.05 and green_box_y+1-0.05<=q<=green_box_y+1+0.05:
        print('green box picked up by right arm')
        green_block_picked_up[0] = True
        green_block_picked_up[1] = 'right'
        ax.add_patch(plt.Rectangle((x6 - 0.5, y6 -1 ), 1, 1, color='yellow'))
        c_label.config(text='Green box is picked up!')
    if green_area-0.05<=p<=green_area+0.05 and 0.95<=q<=1.05:
        print('green box dropped down by right arm')
        green_block_picked_up[0] = False
        green_box_x=green_area
        green_box_y=0
        ax.add_patch(plt.Rectangle((x6 - 0.5, y6 - 1), 1, 1, color='yellow'))
        c_label.config(text='Green box is dropped down!')
    else:
        if green_block_picked_up[0] == True and green_block_picked_up[1]=='left':
            ax.add_patch(plt.Rectangle((x3 - 0.5, y3 - 1), 1, 1, color='green'))
        elif green_block_picked_up[0] == True and green_block_picked_up[1]=='right':
            ax.add_patch(plt.Rectangle((x6 - 0.5, y6 - 1), 1, 1, color='green'))
        else:
            ax.add_patch(plt.Rectangle((green_box_x-0.5, green_box_y), 1, 1, color='green'))
    if (red_block_picked_up[0] == False and red_block_picked_up[1] != 'none' and
            blue_block_picked_up[0] == False and blue_block_picked_up[1] != 'none' and
            green_block_picked_up[0] == False and green_block_picked_up[1] != 'none'):
        c_label.config(text='Congratulations! You did it!')
    x_label.config(text=f"x: {x:.2f}")
    y_label.config(text=f"y: {y:.2f}")
    p_label.config(text=f"p: {p:.2f}")
    q_label.config(text=f"q: {q:.2f}")
    plt.draw()

red_box_x=0.5 #x轴中间点
red_box_y=0 #y轴最低点
red_area=-0.5
blue_box_x=0.5
blue_box_y=1
blue_area=-1.5
green_box_x=0.5
green_box_y=2
green_area=1.5

# Parameters
link1 = 3
link2 = 2
link3= 0.5
# Initial angles
theta1_init = -120
theta2_init = 0
theta3_init = 0
theta4_init = 120
theta5_init = 0
theta6_init = 0
# Create Tkinter window
root = tk.Tk()
root.title("3-DOF Robot")

# Create Matplotlib figure
fig, ax = plt.subplots(figsize=(10, 6))
plt.subplots_adjust(left=0.1, bottom=0.4)


# Create sliders
ax_theta1 = plt.axes([0.1, 0.15, 0.3, 0.03])
s1 = Slider(ax_theta1, 'Theta1', -120, 0, valinit=theta1_init)

ax_theta2 = plt.axes([0.1, 0.1, 0.3, 0.03])
s2 = Slider(ax_theta2, 'Theta2', -120, 90, valinit=theta2_init)

ax_theta3 = plt.axes([0.1, 0.05, 0.3, 0.03])
s3 = Slider(ax_theta3, 'Theta3', -120, 0, valinit=theta3_init)

ax_theta4 = plt.axes([0.6, 0.15, 0.3, 0.03])
s4 = Slider(ax_theta4, 'Theta4', 0, 120, valinit=theta4_init)

ax_theta5 = plt.axes([0.6, 0.1, 0.3, 0.03])
s5 = Slider(ax_theta5, 'Theta5', -90, 120, valinit=theta5_init)

ax_theta6 = plt.axes([0.6, 0.05, 0.3, 0.03])
s6 = Slider(ax_theta6, 'Theta6', 0, 120, valinit=theta6_init)
#End effector coordinate labels
x_label = Label(root, text="x:", borderwidth=1, relief="solid")
y_label = Label(root, text="y:", borderwidth=1, relief="solid")
p_label = Label(root, text="p:", borderwidth=1, relief="solid")
q_label = Label(root, text="q:", borderwidth=1, relief="solid")
c_label = Label(root, text="Object movement", borderwidth=1, relief="solid")
# Pack labels into a table
x_label.grid(row=0, column=0, padx=(10, 5), pady=10, sticky='e')
y_label.grid(row=0, column=1, padx=(5, 5), pady=10, sticky='w')
p_label.grid(row=0, column=2, padx=(5, 5), pady=10, sticky='e')
q_label.grid(row=0, column=3, padx=(5, 10), pady=10, sticky='w')
c_label.grid(row=1, column=0, columnspan=4, padx=10, pady=10)



#Update plot
s1.on_changed(update)
s2.on_changed(update)
s3.on_changed(update)
s4.on_changed(update)
s5.on_changed(update)
s6.on_changed(update)
# Embed Matplotlib figure in Tkinter window
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.draw()
#canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
canvas.get_tk_widget().grid(row=4, columnspan=5, sticky='nsew')
# Start Tkinter event loop
root.mainloop()
