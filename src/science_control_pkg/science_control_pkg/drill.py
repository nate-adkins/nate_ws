import rclpy, can
from robot_interfaces.msg import CanCommand, ScienceDrillControls
from rclpy.node import Node
from myactuator_lib import Motor
from std_msgs.msg import UInt8MultiArray

class ScienceDrillControlNode(Node):

    def __init__(self):
        super().__init__('science_drill_control_node')

        self.drill_controls_subscriber = self.create_subscription(
            ScienceDrillControls,
            'science_drill_controls',
            self.drill_controls_callback,
            10,
        )

        self.can_publisher = self.create_publisher(
            CanCommand,
            'outgoing_can_commands',
            10,
        )

        self.pico_publisher = self.create_publisher(
            UInt8MultiArray,
            'pico_sub',
            10,
            )




        self.drill_arb_id = 0x14A
        self.drill_motor = Motor(self.drill_arb_id)

    def send_can_message(self, can_command: can.Message):
        rm = CanCommand()
        rm.arbitration_id = can_command.arbitration_id
        rm.is_extended_id = can_command.is_extended_id
        rm.byte_0 = can_command.data[0]
        rm.byte_1 = can_command.data[1]
        rm.byte_2 = can_command.data[2]
        rm.byte_3 = can_command.data[3]
        rm.byte_4 = can_command.data[4]
        rm.byte_5 = can_command.data[5]
        rm.byte_6 = can_command.data[6]
        rm.byte_7 = can_command.data[7]
        self.can_publisher.publish(rm)

    def package_linear_actuator(goal_relative_height):
        pass

    def drill_controls_callback(self, drill_cmd_msg: ScienceDrillControls):
        if drill_cmd_msg.reboot_drill_motor:
            self.can_publisher.publish(self.drill_motor.System_reset_command())
        
        self.can_publisher.publish(self.drill_motor.Speed_Closed_loop_Control_Command(drill_cmd_msg.drill_speed_degrees_per_second))

        # publish to pico_sub

def main(args=None):
    rclpy.init(args=args)

    node = ScienceDrillControlNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()