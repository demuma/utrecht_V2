import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8
import time

class HydraulicArmControl(Node):
    def __init__(self):
        super().__init__('hydraulic_arm_control')

        # Joint angle limits (degrees)
        self.joint_limits = {
            'gamma': (-45, 45),
            'beta1': (84, 164),
            'beta2': (-81, 1),
            'beta3': (100, 0)  # Note: beta3 limits are reversed
        }

        # Servo PWM values (extend, neutral, retract)
        self.servo_pwm = {
            'servo_1': (1740, 1450, 1160),
            'servo_2': (1150, 1370, 1590),
            'servo_3': (1130, 1370, 1610),
            'servo_4': (1360, 1760, 1860)
        }
        self.servo_min_max = (1100, 2000)

        # Hydraulic pressure values
        self.pressure_init = 860
        self.pressure_nominal = 1100

        # Current joint angles
        self.current_gamma = 0.0
        self.current_beta1 = 0.0
        self.current_beta2 = 0.0
        self.current_beta3 = 0.0

        # Hydraulic oil temperature
        self.hydraulic_temperature = 0.0

        # Target button
        self.target_button = 1
        self.button_pressed = False

        # Dead zone compensation values (degrees)
        self.dead_zone = {
            'gamma': 1.0,
            'beta1': 1.0,
            'beta2': 1.0,
            'beta3': 1.0
        }

        # --- Subscribers ---
        self.gamma_sub = self.create_subscription(
            Float32, '/robot/gamma', self.gamma_callback, 10)
        self.beta1_sub = self.create_subscription(
            Float32, '/robot/beta_1', self.beta1_callback, 10)
        self.beta2_sub = self.create_subscription(
            Float32, '/robot/beta_2', self.beta2_callback, 10)
        self.beta3_sub = self.create_subscription(
            Float32, '/robot/beta_3', self.beta3_callback, 10)
        self.button_sub = self.create_subscription(
            Int8, '/robot/buttons', self.button_callback, 10)
        self.temperature_sub = self.create_subscription(
            Float32, '/robot/temp_1', self.temperature_callback, 10)

        # --- Publishers ---
        self.servo_1_pub = self.create_publisher(Float32, '/robot/servo_1', 10)
        self.servo_2_pub = self.create_publisher(Float32, '/robot/servo_2', 10)
        self.servo_3_pub = self.create_publisher(Float32, '/robot/servo_3', 10)
        self.servo_4_pub = self.create_publisher(Float32, '/robot/servo_4', 10)
        
        self.pump_pub = self.create_publisher(Float32, '/robot/motor_1', 10)

        # --- Timer for printing joint angles and temperature---
        self.print_timer = self.create_timer(0.1, self.print_status)  # 10 Hz

        # --- Initialize the arm ---
        self.initialize_arm()

        # --- Start control loop ---
        self.control_timer = self.create_timer(0.01, self.control_loop) # P controller at 100 Hz

    def gamma_callback(self, msg):
        self.current_gamma = msg.data

    def beta1_callback(self, msg):
        self.current_beta1 = msg.data

    def beta2_callback(self, msg):
        self.current_beta2 = msg.data

    def beta3_callback(self, msg):
        self.current_beta3 = msg.data

    def button_callback(self, msg):
        if msg.data == self.target_button:
            self.button_pressed = True
            self.get_logger().info(f"Button {self.target_button} pressed!")

    def temperature_callback(self, msg):
        self.hydraulic_temperature = msg.data

    def print_status(self):
        self.get_logger().info(
            f"Gamma: {self.current_gamma:.2f}, Beta1: {self.current_beta1:.2f}, "
            f"Beta2: {self.current_beta2:.2f}, Beta3: {self.current_beta3:.2f}, "
            f"Temperature: {self.hydraulic_temperature:.2f}"
        )

    def initialize_arm(self):
        self.get_logger().info("Initializing arm...")

        # Set initial hydraulic pressure
        pressure_msg = Float32()
        pressure_msg.data = float(self.pressure_init)
        self.pump_pub.publish(pressure_msg)
        time.sleep(2)  # Wait for pressure to stabilize

        # Set nominal hydraulic pressure
        pressure_msg.data = float(self.pressure_nominal)
        self.pump_pub.publish(pressure_msg)
        self.get_logger().info("Arm initialized.")

    def control_loop(self):
        if self.button_pressed:
            # Button is already pressed, do nothing
            return

        # --- Simple extension logic to reach button 1 ---
        
        # P gain for each joint
        Kp_gamma = 0.5
        Kp_beta1 = 0.5
        Kp_beta2 = 0.5
        Kp_beta3 = 0.5
        
        # Target angles for extending the arm (these need to be tuned)
        target_gamma = 0.0 # Example target
        target_beta1 = 164.0  # Fully extend beta1
        target_beta2 = -81.0 # Fully extend beta2
        target_beta3 = 0.0  # Fully extend beta3

        # --- P Control with Dead Zone Compensation ---
        # Calculate error for each joint
        error_gamma = target_gamma - self.current_gamma
        error_beta1 = target_beta1 - self.current_beta1
        error_beta2 = target_beta2 - self.current_beta2
        error_beta3 = target_beta3 - self.current_beta3

        # Apply dead zone compensation
        if abs(error_gamma) < self.dead_zone['gamma']:
            error_gamma = 0.0
        if abs(error_beta1) < self.dead_zone['beta1']:
            error_beta1 = 0.0
        if abs(error_beta2) < self.dead_zone['beta2']:
            error_beta2 = 0.0
        if abs(error_beta3) < self.dead_zone['beta3']:
            error_beta3 = 0.0

        # Calculate control signal (PWM) based on error and P gain
        control_gamma = Kp_gamma * error_gamma
        control_beta1 = Kp_beta1 * error_beta1
        control_beta2 = Kp_beta2 * error_beta2
        control_beta3 = Kp_beta3 * error_beta3

        # Map control signal to PWM values and publish
        self.publish_servo_pwm('servo_1', control_gamma)
        self.publish_servo_pwm('servo_2', control_beta1)
        self.publish_servo_pwm('servo_3', control_beta2)
        self.publish_servo_pwm('servo_4', control_beta3)

    def publish_servo_pwm(self, servo_name, control_signal):
        """Maps the control signal to PWM values and publishes it to the corresponding servo."""
        
        # Determine direction based on control signal
        if control_signal > 0: # Extend
            pwm_val = self.servo_pwm[servo_name][0]
        elif control_signal < 0: # Retract
            pwm_val = self.servo_pwm[servo_name][2]
        else: # Neutral
            pwm_val = self.servo_pwm[servo_name][1]

        # Interpolate PWM value based on magnitude of control signal
        if control_signal > 0:  # Extend
          pwm_val = self.servo_pwm[servo_name][1] + (self.servo_pwm[servo_name][0]-self.servo_pwm[servo_name][1]) * abs(control_signal)/100.0
        elif control_signal < 0:  # Retract
          pwm_val = self.servo_pwm[servo_name][1] - (self.servo_pwm[servo_name][1]-self.servo_pwm[servo_name][2]) * abs(control_signal)/100.0
        else:
          pwm_val = self.servo_pwm[servo_name][1] # Neutral

        # Limit PWM signal to min/max values
        pwm_val = max(min(pwm_val, self.servo_min_max[1]), self.servo_min_max[0])
        
        # Publish PWM value
        pwm_msg = Float32()
        pwm_msg.data = pwm_val
        if servo_name == 'servo_1':
            self.servo_1_pub.publish(pwm_msg)
        elif servo_name == 'servo_2':
            self.servo_2_pub.publish(pwm_msg)
        elif servo_name == 'servo_3':
            self.servo_3_pub.publish(pwm_msg)
        elif servo_name == 'servo_4':
            self.servo_4_pub.publish(pwm_msg)

def main(args=None):
    rclpy.init(args=args)
    hydraulic_arm_control = HydraulicArmControl()
    rclpy.spin(hydraulic_arm_control)
    hydraulic_arm_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
