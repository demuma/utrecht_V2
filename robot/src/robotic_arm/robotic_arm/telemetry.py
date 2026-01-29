import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8
from dataclasses import dataclass
from statistics import mean
import serial
import os
import time

@dataclass
class ParsedMessage:
    gamma: float
    beta_1: float
    beta_2: float
    beta_3: float
    alpha: float
    gripper: float
    buttons: list[int]
    temperature: float

ARDUINO_MESSAGE_LENGTH = 12

class Telemetry(Node):
    def __init__(self):
        super().__init__('telemetry')

        device = os.getenv('ARDUINO_DEVICE')

        if device is None:
            self.get_logger().fatal('Cannot find Arduino. Did you forget to attach the device or run `$(python3 detect_devices.py)`?')
            return
        else:
            self.get_logger().info(f'Using Arduino device at {device}')

        # Create publishers
        self.buttons_topic      = self.create_publisher(Int8   , '/robot/buttons', 10)
        self.gamma_topic        = self.create_publisher(Float32, '/robot/gamma',   10)
        self.beta_1_topic       = self.create_publisher(Float32, '/robot/beta_1',  10)
        self.beta_2_topic       = self.create_publisher(Float32, '/robot/beta_2',  10)
        self.beta_3_topic       = self.create_publisher(Float32, '/robot/beta_3',  10)
        self.alpha_topic        = self.create_publisher(Float32, '/robot/alpha',   10)
        self.gripper_topic      = self.create_publisher(Float32, '/robot/gripper', 10)
        self.temperature_topic  = self.create_publisher(Float32, '/robot/temp_1',  10)

        # Define reusable message objects
        self.buttons_msg     = Int8()
        self.gamma_msg       = Float32()
        self.beta_1_msg      = Float32()
        self.beta_2_msg      = Float32()
        self.beta_3_msg      = Float32()
        self.alpha_msg       = Float32()
        self.gripper_msg     = Float32()
        self.temperature_msg = Float32()
        self.last_temperatures = []

        # Connect to Arduino
        self.device = serial.Serial(device, 9600, timeout=2)
        time.sleep(2)
        self.timer = self.create_timer(0.01, self.__process_input)

        self.get_logger().info(f'Subscribed to robotic arm telemetry from {device}')

    def __process_input(self):
        if self.device.in_waiting <= 0:
            return

        message = self.__parse_input(self.device.readline().decode('utf-8').strip())

        if not message:
            return

        self.__publish_angles(message)
        self.__publish_buttons(message)
        self.__publish_temperature(message)

    def __parse_input(self, line: str) -> ParsedMessage | None:
        # Make spaces and tabs visible in logs:
        # - tabs are shown as the two characters "\t"
        # - spaces are shown as a middle dot "·"
        visible_line = line.replace('\t', '\\t').replace(' ', '·')

        # self.get_logger().info(f'Received message: {visible_line}')

        # Example message:
        # Angles·[degrees]:\t339.02\t310.45\t204.87\t48.30\t136.03\t310.61\tButtons·[pressed·is·1]:\t0\t1\t0\t0\t0\t0

        message_parts = line.split("\t")

        try:
            # Extract degrees and raw button values into a dict
            extracted = {
                "degrees": [
                    float(message_parts[1]),
                    float(message_parts[2]),
                    float(message_parts[3]),
                    float(message_parts[4]),
                    float(message_parts[5]),
                    float(message_parts[6]),
                ],
                # raw button values (0/1) as ints
                "buttons": [int(x) for x in message_parts[8:14]],
            }
            # self.get_logger().info(f'Extracted values: {extracted}')

            # Build pressed-buttons list (preserve original behavior)
            pressed_buttons = [
                index + 1
                for index, val
                in enumerate(extracted["buttons"])
                if val == 1
            ]

            return ParsedMessage(
                gamma=extracted["degrees"][2],
                beta_1=extracted["degrees"][3],
                beta_2=extracted["degrees"][4],
                beta_3=extracted["degrees"][5],
                alpha=extracted["degrees"][0],
                gripper=extracted["degrees"][1],
                buttons=pressed_buttons,
                temperature=float(1337),
            )
        except Exception as e:
            self.get_logger().error(f'Failed to parse message: {e}')
            return None

    def __publish_angles(self, message: ParsedMessage):
        """
        Angles should always between 0 and 360 degrees.
        """
        has_value_changed = False \
            or self.gamma_msg.data != message.gamma \
            or self.beta_1_msg.data != message.beta_1 \
            or self.beta_2_msg.data != message.beta_2 \
            or self.beta_3_msg.data != message.beta_3 \
            or self.alpha_msg.data != message.alpha \
            or self.gripper_msg.data != message.gripper

        self.gamma_msg.data = message.gamma
        self.gamma_topic.publish(self.gamma_msg)

        self.beta_1_msg.data = message.beta_1
        self.beta_1_topic.publish(self.beta_1_msg)

        self.beta_2_msg.data = message.beta_2
        self.beta_2_topic.publish(self.beta_2_msg)

        self.beta_3_msg.data = message.beta_3
        self.beta_3_topic.publish(self.beta_3_msg)

        self.alpha_msg.data = message.alpha
        self.alpha_topic.publish(self.alpha_msg)

        self.gripper_msg.data = message.gripper
        self.gripper_topic.publish(self.gripper_msg)

        # if has_value_changed:
        #     self.get_logger().info(f'γ: {message.gamma} β1: {message.beta_1} β2: {message.beta_2} β3: {message.beta_3} α: {message.alpha} Gripper: {message.gripper}')

    def __publish_buttons(self, message: ParsedMessage):
        """
        A message containing a button number will be sent for each button that
        is currently being pressed
        """
        if len(message.buttons) == 0:
            self.buttons_msg.data = 0
            self.buttons_topic.publish(self.buttons_msg)
        for button in message.buttons:
            self.buttons_msg.data = int(button)
            self.buttons_topic.publish(self.buttons_msg)
            self.get_logger().info(f'Button {button} is being pressed')

    def __publish_temperature(self, message: ParsedMessage):
        """
        Assumption: Temperature in degrees Celcius.
        """
        # Store last 10 temperature measurements
        self.last_temperatures.append(message.temperature)
        if len(self.last_temperatures) > 10:
            del self.last_temperatures[0]

        current_temperature = round(mean(self.last_temperatures))

        if abs(self.temperature_msg.data - current_temperature) >= 1.0:
            self.get_logger().info(f'Temperature is now {current_temperature} degrees')

        # Publish raw temperature measurements
        self.temperature_msg.data = message.temperature
        self.temperature_topic.publish(self.temperature_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Telemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
