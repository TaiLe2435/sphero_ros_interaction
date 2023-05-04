#!/usr/bin/env python3

import tkinter as tk
import subprocess

# Define the launch command for button 1
launch_cmd1 = 'roslaunch sphero_mini sphero_mini.launch'

# Define the launch command for button 2
launch_cmd2 = 'roslaunch sphero_mini sphero_mini.launch mac_address:=CB:42:41:16:C0:E5'

def launch_button1():
    # Launch the first roslaunch file
    subprocess.Popen(launch_cmd1, shell=True)

    root.destroy()

def launch_button2():
    # Launch the second roslaunch file
    subprocess.Popen(launch_cmd2, shell=True)

    root.destroy()

# Create the main window
root = tk.Tk()
root.title('ROS Launch App')

# Create the button 1
button1 = tk.Button(root, text='Blue Robot', command=launch_button1)
button1.pack(side=tk.LEFT, padx=10, pady=10)

# Create the button 2
button2 = tk.Button(root, text='Red Robot', command=launch_button2)
button2.pack(side=tk.LEFT, padx=10, pady=10)

# Run the main event loop
root.mainloop()
