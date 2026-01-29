import os
import time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


# Constants
# ------------------------------------------------------------------------------


QOS_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE
)

MAX_OPERATING_TEMPERATURE = 55.0

TIMER_SECONDS = 0.01


# Class that handles all communication
# ------------------------------------------------------------------------------


class Driver(Node):
    # Class attributes
    # --------------------------------------------------------------------------

    # Used to communicate with the Teensy device
    device = None
    timer = None

    subscribed_topics = {}
    topic_data: dict[str, float] = {
        "motor_1": 1500.0,
        "servo_1": 1500.0,
        "servo_2": 1500.0,
        "servo_3": 1500.0,
        "servo_4": 1500.0,
        "servo_5": 1500.0,
        "servo_6": 1500.0,
        "temp_1": -273.15,
    }


    # Initialisation
    # --------------------------------------------------------------------------


    def __init__(self):
        """
        Initialise the robotic arm driver node.
        """
        super().__init__('driver')

        device = os.getenv('TEENSY_DEVICE')

        if device is None:
            self.get_logger().fatal('Cannot find Teensy. Did you forget to attach the device or run `$(python3 detect_devices.py)`?')
            return
        else:
            self.get_logger().info(f'Using Teensy device at {device}')

        self.device = serial.Serial(device, 115200, timeout=2)
        time.sleep(2)
        self.timer = self.create_timer(0.01, self.__process_input)

        self.__setup_ros_command_subscribers()


    def __process_input(self):
        if self.device.in_waiting <= 0:
            return

        message = self.device.readline().decode('utf-8').strip()

        if not message or 'hid_process_in_data' in message:
            return

        self.get_logger().info('Teensy: ' + str(message))

    def __update_data(self, name, message):
        if name == 'temp_1':
            return

        # Log received value and update local cache
        # self.get_logger().info(f'Received value for {name}: {message.data}')
        self.topic_data[name] = message.data

        # Prepare the message to send to the Teensy device
        serial_msg = f"{name}:{self.topic_data[name]}\n"

        # If device is not available, log and skip sending
        if not getattr(self, 'device', None):
            self.get_logger().warning(f"No device available; skipping write of: {serial_msg.strip()}")
            return

        try:
            # Write bytes to the serial device
            # Use utf-8 encoding and flush by writing a newline-terminated message
            self.device.write(serial_msg.encode('utf-8'))
            self.get_logger().debug(f"Wrote to device: {serial_msg.strip()}")
        except Exception as e:
            self.get_logger().error(f"Failed to write to device: {e}")


    def __setup_ros_command_subscribers(self):
        for topic in [
            'motor_1',
            'servo_1',
            'servo_2',
            'servo_3',
            'servo_4',
            'servo_5',
            'servo_6',
            'temp_1',
        ]:
            self.subscribed_topics[topic] = self.create_subscription(
                Float32,
                f'/robot/{topic}',
                lambda msg, name=topic: self.__update_data(name, msg),
                QOS_PROFILE,
            )


# Entrypoint
# ------------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
