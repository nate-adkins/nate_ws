import tkinter as tk
from tkinter import ttk
import rclpy
from std_msgs.msg import String
from robot_interfaces.msg import ScienceScoopControls

def start1(node):
    node.publish_gui_events("Start button 1 clicked")
    node.publish_scoop_controls(1,True,False,False)

def stop1(node):
    node.publish_gui_events("Stop button 1 clicked")
    node.publish_scoop_controls(1,False,True,False)

def reset1(node):
    node.publish_gui_events("Reset button 1 clicked")
    node.publish_scoop_controls(1,False,False,True)

def slider_changed1(node, value):
    node.publish_gui_events("Slider 1 value: {}".format(value))

def start2(node):
    node.publish_gui_events("Start button 2 clicked")
    node.publish_scoop_controls(2,True,False,False)

def stop2(node):
    node.publish_gui_events("Stop button 2 clicked")
    node.publish_scoop_controls(2,False,True,False)

def reset2(node):
    node.publish_gui_events("Reset button 2 clicked")
    node.publish_scoop_controls(2,False,False,True)

def slider_changed2(node, value):
    node.publish_gui_events("Slider 2 value: {}".format(value))

def start3(node):
    node.publish_gui_events("Start button 3 clicked")
    node.publish_scoop_controls(3,True,False,False)

def stop3(node):
    node.publish_gui_events("Stop button 3 clicked")
    node.publish_scoop_controls(3,True,False,False)

def reset3(node):
    node.publish_gui_events("Reset button 3 clicked")
    node.publish_scoop_controls(3,True,False,False)

def slider_changed3(node, value):
    node.publish_gui_events("Slider 3 value: {}".format(value))

class SliderWithButtons(tk.Frame):
    def __init__(self, master, start_func, stop_func, reset_func, slider_func, node):
        super().__init__(master)

        self.grid(row=0, column=master.num_instances, padx=10, pady=10)

        self.slider_label = ttk.Label(self, text="Slider")
        self.slider_label.grid(row=0, column=0, padx=10, pady=5)

        self.slider = ttk.Scale(self, from_=0, to=1, orient="vertical", command=lambda value: slider_func(node, value))
        self.slider.grid(row=1, column=0, padx=10, pady=5)

        self.buttons_label = ttk.Label(self, text="Buttons")
        self.buttons_label.grid(row=2, column=0, padx=10, pady=5)

        self.start_button = ttk.Button(self, text="Start", command=lambda: start_func(node))
        self.start_button.grid(row=3, column=0, padx=10, pady=5)

        self.stop_button = ttk.Button(self, text="Stop", command=lambda: stop_func(node))
        self.stop_button.grid(row=4, column=0, padx=10, pady=5)

        self.reset_button = ttk.Button(self, text="Reset", command=lambda: reset_func(node))
        self.reset_button.grid(row=5, column=0, padx=10, pady=5)

class MainWindow(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Science Controls")
        self.num_instances = 0

        # Initialize ROS2 node
        rclpy.init()
        self.node = rclpy.create_node("gui_node")

        for _ in range(3):
            self.add_slider()

    def add_slider(self):
        self.num_instances += 1
        SliderWithButtons(
            self,
            start_func=start1 if self.num_instances == 1 else (start2 if self.num_instances == 2 else start3),
            stop_func=stop1 if self.num_instances == 1 else (stop2 if self.num_instances == 2 else stop3),
            reset_func=reset1 if self.num_instances == 1 else (reset2 if self.num_instances == 2 else reset3),
            slider_func=slider_changed1 if self.num_instances == 1 else (slider_changed2 if self.num_instances == 2 else slider_changed3),
            node=self
        )

    def publish_gui_events(self, message):
        publisher = self.node.create_publisher(String, 'gui_output', 10)
        msg = String()
        msg.data = message
        publisher.publish(msg)

    def publish_scoop_controls(self, dynamixel_id: int, start: bool, stop: bool, reboot: bool):
        publisher = self.node.create_publisher(ScienceScoopControls, 'science_scoop_controls', 10)
        msg = ScienceScoopControls()
        msg.start_spin = start
        msg.stop_spin = stop 
        msg.reboot = reboot
        msg.dynamixel_id = dynamixel_id
        publisher.publish(msg)

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()

def main():
    app = MainWindow()
    app.mainloop()
    app.shutdown()

if __name__ == "__main__":
    main()