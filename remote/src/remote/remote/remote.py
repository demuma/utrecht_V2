import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from flask import Flask, request, jsonify

# Constants
# ------------------------------------------------------------------------------

QOS_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE
)


# ROS node
# ------------------------------------------------------------------------------


class RemoteControl(Node):
    sensors = {
        "temp_1": 9999.9,
        "beta_1": -999.0,
        "beta_2": -999.0,
        "beta_3": -999.0,
        "gamma": -999.0,
        "alpha": -999.0,
        "gripper": -999.0,
        "buttons": 0,
    }

    def __init__(self):
        super().__init__('Remote_control')

        self.get_logger().info('Initializing')

        # Set up publishers
        self.motor_1_topic = self.create_publisher(Float32, '/robot/motor_1', 10)
        self.servo_1_topic = self.create_publisher(Float32, '/robot/servo_1', 10)
        self.servo_2_topic = self.create_publisher(Float32, '/robot/servo_2', 10)
        self.servo_3_topic = self.create_publisher(Float32, '/robot/servo_3', 10)
        self.servo_4_topic = self.create_publisher(Float32, '/robot/servo_4', 10)
        self.servo_5_topic = self.create_publisher(Float32, '/robot/servo_5', 10)
        self.servo_6_topic = self.create_publisher(Float32, '/robot/servo_6', 10)

        # Initialize in neutral-ish position
        self.motor_1_msg = Float32(data=1500.0)
        self.servo_1_msg = Float32(data=1500.0)
        self.servo_2_msg = Float32(data=1500.0)
        self.servo_3_msg = Float32(data=1500.0)
        self.servo_4_msg = Float32(data=1500.0)
        self.servo_5_msg = Float32(data=1500.0)
        self.servo_6_msg = Float32(data=1500.0)

        # Set up subscribers
        self.create_subscription(
            Float32,
            '/robot/temp_1',
            lambda msg: self.set_sensor("temp_1", float(msg.data)),
            QOS_PROFILE,
        )
        self.create_subscription(
            Float32,
            '/robot/beta_1',
            lambda msg: self.set_sensor("beta_1", float(msg.data)),
            QOS_PROFILE,
        )
        self.create_subscription(
            Float32,
            '/robot/beta_2',
            lambda msg: self.set_sensor("beta_2", float(msg.data)),
            QOS_PROFILE,
        )
        self.create_subscription(
            Float32,
            '/robot/beta_3',
            lambda msg: self.set_sensor("beta_3", float(msg.data)),
            QOS_PROFILE,
        )
        self.create_subscription(
            Float32,
            '/robot/gamma',
            lambda msg: self.set_sensor("gamma", float(msg.data)),
            QOS_PROFILE,
        )
        self.create_subscription(
            Float32,
            '/robot/alpha',
            lambda msg: self.set_sensor("alpha", float(msg.data)),
            QOS_PROFILE,
        )
        self.create_subscription(
            Float32,
            '/robot/gripper',
            lambda msg: self.set_sensor("gripper", float(msg.data)),
            QOS_PROFILE,
        )
        self.create_subscription(
            Int8,
            '/robot/buttons',
            lambda msg: self.set_sensor("buttons", int(msg.data)),
            QOS_PROFILE,
        )

    def set_sensor(self, key, value):
        try:
            self.sensors[key] = round(value, 1)
        except Exception as e:
            self.get_logger().error(f"Error setting sensor {key}: {e}")

    def publish_motor_1(self, value):
        msg = Float32(data=float(value))
        self.motor_1_topic.publish(msg)
        self.motor_1_msg = msg
        self.get_logger().info(f"Published motor_1: {value}")
        return True

    def publish_servo(self, servo_num, value):
        msg = Float32(data=float(value))
        if servo_num == 1:
            self.servo_1_topic.publish(msg)
            self.servo_1_msg = msg
        elif servo_num == 2:
            self.servo_2_topic.publish(msg)
            self.servo_2_msg = msg
        elif servo_num == 3:
            self.servo_3_topic.publish(msg)
            self.servo_3_msg = msg
        elif servo_num == 4:
            self.servo_4_topic.publish(msg)
            self.servo_4_msg = msg
        elif servo_num == 5:
            self.servo_5_topic.publish(msg)
            self.servo_5_msg = msg
        elif servo_num == 6:
            self.servo_6_topic.publish(msg)
            self.servo_6_msg = msg
        else:
            return False
        self.get_logger().info(f"Published servo_{servo_num}: {value}")
        return True


# Web server
# ------------------------------------------------------------------------------


def create_flask_app(remote):
    app = Flask(
        __name__,
        static_folder='/root/ros2_ws/public',
        static_url_path='',
    )

    @app.route('/')
    def index():
        return app.send_static_file('index.html')

    @app.route('/motor', methods=['POST'])
    def set_motor():
        data = request.get_json()
        value = data.get('pwm')
        remote.publish_motor_1(value)
        return '', 200

    @app.route('/servo/<int:servo_num>', methods=['POST'])
    def set_servo(servo_num):
        data = request.get_json()
        value = data.get('pwm')
        remote.publish_servo(servo_num, value)
        return '', 200

    @app.route('/sensors', methods=['GET'])
    def get_sensors():
        return jsonify(remote.sensors), 200

    return app


# Entrypoint
# ------------------------------------------------------------------------------


def main(args=None):
    print('Starting remote control node')

    rclpy.init(args=args)
    remote = RemoteControl()

    flask_app = create_flask_app(remote)
    flask_thread = threading.Thread(
        target=lambda: flask_app.run(
            host='0.0.0.0',
            port=80,
            debug=False,
            use_reloader=False,
        ),
        daemon=True
    )
    flask_thread.start()

    try:
        rclpy.spin(remote)
    except KeyboardInterrupt:
        pass
    finally:
        remote.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
