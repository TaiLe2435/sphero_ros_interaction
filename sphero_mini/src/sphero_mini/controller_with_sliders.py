#!/usr/bin/env python3

import tkinter as tk
import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import numpy as np

class JoystickGUI(tk.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.pack()
        self.create_widgets()

    def create_widgets(self):
        # Create a joystick widget
        self.joystick = Joystick(self, size=100)
        self.joystick.pack(side="left", padx=50, pady=50)
        # self.joystick.place(relx=0.50, rely=0.5, anchor="center")

        # Create a directional pad widget
        self.dpad = DPad(self, size=100)
        self.dpad.pack(side="right", padx=50, pady=50)
        # self.dpad.place(relx=0.60, rely=0.5, anchor="center")

        # Bind the arrow key events to the dpad widget
        self.master.bind("<a>", lambda event: self.dpad.move_left())
        self.master.bind("<d>", lambda event: self.dpad.move_right())
        self.master.bind("<w>", lambda event: self.dpad.move_up())
        self.master.bind("<s>", lambda event: self.dpad.move_down())

        # Create sliders widget
        self.sliders = ColorSelector(self)
        self.sliders.pack()



class Joystick(tk.Canvas):
    def __init__(self, master=None, size=100, *args, **kwargs):
        super().__init__(master, width=size, height=size, *args, **kwargs)
        self.size = size
        self.center = size/2
        self.handle_size = size/4
        self.handle_x = self.center
        self.handle_y = self.center
        self.x = 0
        self.y = 0
        self.joystick_pub = rospy.Publisher('/GUI/joystick', Vector3, queue_size=1)
        self.joystick_message = Vector3()
        self.draw()

        # Bind the mouse events to the canvas
        self.bind("<Button-1>", self.move_handle)
        self.bind("<B1-Motion>", self.move_handle)
        self.bind("<ButtonRelease-1>", self.reset_handle)

    def move_handle(self, event):
        # Move the handle to the mouse position
        self.x = event.x - self.center
        self.y = event.y - self.center
        # print(self.x, " ", self.y)
        r = min(self.center, math.sqrt(self.x**2 + self.y**2))
        theta = math.atan2(self.y, self.x)
        self.handle_x = r * math.cos(theta) + self.center
        self.handle_y = r * math.sin(theta) + self.center
        
        self.joystick_message.x = int(r*3)
        self.joystick_message.y = int(theta * 180/np.pi + 90)
        self.joystick_message.z = 0
        
        self.joystick_pub.publish(self.joystick_message)
        # time.sleep(0.5)
        print("r, theta: ", r * 3, " ", theta * 180/np.pi + 90)
        self.draw()

    def reset_handle(self, event):
        # Snap the handle back to the center
        self.handle_x = self.center
        self.handle_y = self.center
        self.x = 0
        self.y = 0
        print("r, theta: ", self.x, " ", self.y)
        self.joystick_message.x = 0
        self.joystick_message.y = 0
        self.joystick_message.z = 0
        self.joystick_pub.publish(self.joystick_message)
        self.draw()

    def draw(self):
        # Draw the joystick background and handle
        self.delete(tk.ALL)
        self.create_oval(0, 0, self.size, self.size, fill="gray")
        self.create_oval(self.handle_x-self.handle_size, self.handle_y-self.handle_size,
                         self.handle_x+self.handle_size, self.handle_y+self.handle_size,
                         fill="red")


class DPad(tk.Canvas):
    def __init__(self, master=None, size=100, handle_size=10):
        super().__init__(master, width=size, height=size)
        self.size = size
        self.bind("<Button-1>", self.move)
        self.handle_size = handle_size
        self.center = self.size/2

        self.dpad_pub = rospy.Publisher("/GUI/dpad", String, queue_size=1)


        self.draw()


    def draw(self):
        # Draw the D-pad background and arrows
        self.delete(tk.ALL)
        self.create_rectangle(0, 0, self.size, self.size, fill="gray")
        self.create_polygon(self.center-self.handle_size, self.center-self.handle_size,
                            self.center, self.center-2*self.handle_size,
                            self.center+self.handle_size, self.center-self.handle_size,
                            fill="red", outline="black")
        self.create_polygon(self.center+self.handle_size, self.center-self.handle_size,
                            self.center+2*self.handle_size, self.center,
                            self.center+self.handle_size, self.center+self.handle_size,
                            fill="red", outline="black")
        self.create_polygon(self.center+self.handle_size, self.center+self.handle_size,
                            self.center, self.center+2*self.handle_size,
                            self.center-self.handle_size, self.center+self.handle_size,
                            fill="red", outline="black")
        self.create_polygon(self.center-self.handle_size, self.center+self.handle_size,
                            self.center-2*self.handle_size, self.center,
                            self.center-self.handle_size, self.center-self.handle_size,
                            fill="red", outline="black")

    def move_left(self):
        print("Left")
        dpad_message = "Left"
        self.dpad_pub.publish(dpad_message)
    
    def move_right(self):
        print("Right")
        dpad_message = "Right"
        self.dpad_pub.publish(dpad_message)

    def move_up(self):
        print("Up")
        dpad_message = "Up"
        self.dpad_pub.publish(dpad_message)
    
    def move_down(self):
        print("Down")
        dpad_message = "Down"
        self.dpad_pub.publish(dpad_message)

    def move(self, event):
        # Determine which arrow was clicked
        if event.x >= 2*self.size/3 and event.y <= 2*self.size/3 and event.y >= self.size/3:
            print("Right")
            dpad_message = "Right"
            self.dpad_pub.publish(dpad_message)
        elif event.x <= self.size/3 and event.y >= self.size/3:
            print("Left")
            dpad_message = "Left"
            self.dpad_pub.publish(dpad_message)
        elif event.x >= self.size/3 and event.x <= 2*self.size/3 and event.y < self.size/2:
            print("Up")
            dpad_message = "Up"
            self.dpad_pub.publish(dpad_message)
        elif event.x >= self.size/3 and event.x <= 2*self.size/3 and event.y > self.size/2:
            print("Down")
            dpad_message = "Down"
            self.dpad_pub.publish(dpad_message)
        else:
            print("No button pressed")
            dpad_message = "None"
            self.dpad_pub.publish(dpad_message)

class ColorSelector(tk.Canvas):
    def __init__(self, master=None, size=100, *args, **kwargs):
        super().__init__(master, width=size, height=size, *args, **kwargs)
        self.size = size
        self.center = size/2

        # Set default color to black
        self.color = "#000000"

        self.root = master

        # Create sliders for R, G, B values
        self.r_slider = tk.Scale(self.root, from_=0, to=255, orient=tk.HORIZONTAL, label="R", command=self.update_color)
        self.r_slider.pack()
        self.g_slider = tk.Scale(self.root, from_=0, to=255, orient=tk.HORIZONTAL, label="G", command=self.update_color)
        self.g_slider.pack()
        self.b_slider = tk.Scale(self.root, from_=0, to=255, orient=tk.HORIZONTAL, label="B", command=self.update_color)
        self.b_slider.pack()

        # Create canvas to display selected color
        self.canvas = tk.Canvas(self.root, width=200, height=200, bg=self.color)
        self.canvas.pack()
        
        rospy.init_node('GUI', anonymous=True, log_level=rospy.DEBUG)
        self.slider_pub = rospy.Publisher('/GUI/slider', Vector3, queue_size=1)
        self.slider_message = Vector3()
        # Run main loop
        self.root.mainloop()

    def update_color(self, val):
        # Update color based on slider values
        r = self.r_slider.get()
        g = self.g_slider.get()
        b = self.b_slider.get()
        self.color = "#{:02x}{:02x}{:02x}".format(r, g, b)
        print(r, ' ', g, ' ', b)
        self.slider_message.x = r
        self.slider_message.y = g
        self.slider_message.z = b
        self.slider_pub.publish(self.slider_message)
        self.canvas.config(bg=self.color)

def main():
    rospy.init_node('GUI', anonymous=True, log_level=rospy.DEBUG)   
    rate = rospy.Rate(1)  # 10 Hz
    root = tk.Tk()
    root.geometry("800x400")
    app = JoystickGUI(master=root)
    app.mainloop()

    rospy.spin()

if __name__ == "__main__":
    main()