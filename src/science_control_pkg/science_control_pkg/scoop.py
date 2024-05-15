import rclpy, time, threading
from rclpy import Node
from robot_interfaces.msg import ScienceScoopControls
from dynamixel_lib import U2D2, Dynamixel, XL430

class ScienceScoopControlNode(Node):

    def __init__(self):
        super().__init__('science_scoop_control_node')

        self.science_scoop_sub = self.create_subscription(
            ScienceScoopControls,
            'science_scoop_controls',
            self.scoop_control_callback,
            10
        )

        self.thread_lock = threading.Lock()

        self.rotation_speed_rev_per_min = 60    

        self.u2d2 = U2D2('/dev/ttyUSB0', 1000000)
        self.dxl_scoop_1 = Dynamixel(XL430, 1, self.u2d2)
        self.dxl_scoop_2 = Dynamixel(XL430, 2, self.u2d2)
        self.dxl_scoop_3 = Dynamixel(XL430, 3, self.u2d2)
        self.dxls = [self.dxl_scoop_1, self.dxl_scoop_2, self.dxl_scoop_3]

        self.last_spin_start_time = time.time()
        self.spin_time_seconds = 30
        
        self.inactivity_monitor_thread = threading.Thread(target=self.monitor_inactivity)
        self.inactivity_monitor_thread.daemon = True
        self.inactivity_monitor_thread.start()

    def monitor_inactivity(self):
        while True:
            if (time.time() - self.last_spin_start_time) >= self.spin_time_seconds:
                with self.thread_lock:
                    for dxl in self.dxls:
                        self.stop_spinning(dxl)
            time.sleep(1)

    def reverse_direction(self):
        self.rotation_speed_rev_per_min *= -1

    def reboot(self, dxl: Dynamixel):
        with self.thread_lock:
            dxl.reboot(dxl)

    def stop_spinning(self, dxl: Dynamixel):
        with self.thread_lock:
            dxl.write(XL430.GoalVelocity, 0)
            dxl.write(XL430.TorqueEnable, 0)

    def start_spinning(self, dxl: Dynamixel):
        op_modes = {'velocity' : 1, 'position' : 3}
        with self.thread_lock:
            dxl.write(XL430.OperatingMode, op_modes.get('velocity'))
            dxl.write(XL430.TorqueEnable, 1)
            dxl.write(XL430.GoalVelocity, self.rotation_speed_rev_per_min)

    def scoop_control_callback(self, msg: ScienceScoopControls) -> None:
        self.last_message_time = time.time()

        curr_dxl: Dynamixel = None

        for dxl in self.dxls:
            if dxl._id == msg.dynamixel_id:
                curr_dxl = dxl
        
        if curr_dxl is None:
            self.get_logger().warn(f'scoop_control_callback: provided dynamixel id ({msg.dynamixel_id}) is not in use')
            return
        
        if msg.reboot:
            self.reboot(curr_dxl)

        if msg.start_spin and msg.stop_spin: 
            self.get_logger().warn(f'scoop_control_callback: scoop told to start and stop, defaulting to stop')
            self.stop_spinning(curr_dxl)

        elif msg.start_spin:
            self.start_spinning(curr_dxl)
            self.last_spin_start_time = time.time()

        elif msg.stop_spin:
            self.stop_spinning(curr_dxl)


def main(args=None):
    rclpy.init(args=args)
    node = ScienceScoopControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()