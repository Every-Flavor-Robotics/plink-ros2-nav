import numpy as np
import rclpy
from motorgo import ControlMode
from motorgo.plink import Plink
from rclpy.node import Node
from sensor_msgs.msg import Imu

from motorgo_msgs.msg import (  # Replace <package_name> with your package name
    MotorGoEncoderData,
    MotorGoMotorCommand,
)


class MotorGoNode(Node):
    def __init__(self, frequency=150):
        super().__init__("motorgo_node")

        self.frequency = frequency

        # Start up the motor controller
        self.plink = Plink(self.frequency)
        self.plink.connect()

        # Set control mode to POWER
        self.plink.channel1.control_mode = ControlMode.POWER
        self.plink.channel2.control_mode = ControlMode.POWER
        self.plink.channel3.control_mode = ControlMode.POWER
        self.plink.channel4.control_mode = ControlMode.POWER

        # Set power to 0 for all channels
        self.plink.channel1.power = 0
        self.plink.channel2.power = 0
        self.plink.channel3.power = 0
        self.plink.channel4.power = 0

        # Subscription to motor commands
        self.motor_command_sub = self.create_subscription(
            MotorGoMotorCommand,
            "motorgo/command",
            self.motor_command_callback,
            10,  # QoS depth
        )

        # Publisher for encoder data
        self.encoder_data_pub = self.create_publisher(
            MotorGoEncoderData,
            "motorgo/encoder",
            10,  # QoS depth
        )

        self.imu_pub = self.create_publisher(
            Imu,
            "motorgo/imu",
            10,
        )

        # Timer to publish encoder data at a fixed rate
        self.encoder_timer = self.create_timer(1 / self.frequency, self.publish_data)

        self.get_logger().info("motorgo_node has been initialized")

    def motor_command_callback(self, msg):
        """Callback to handle incoming motor commands."""

        # Set power to the respective channel
        self.plink.channel1.power_command = msg.motor_1_power
        self.plink.channel2.power_command = msg.motor_2_power
        self.plink.channel3.power_command = msg.motor_3_power
        self.plink.channel4.power_command = msg.motor_4_power

        # self.get_logger().info(f"Received motor command: {msg}")

    def publish_data(self):
        self.publish_encoder_data()
        self.publish_imu_data()

    def publish_encoder_data(self):
        """Publish encoder data at a fixed rate."""

        encoder_msg = MotorGoEncoderData()
        encoder_msg.header.stamp = self.get_clock().now().to_msg()

        encoder_msg.motor_1_position = self.plink.channel1.position
        encoder_msg.motor_2_position = self.plink.channel2.position
        encoder_msg.motor_3_position = self.plink.channel3.position
        encoder_msg.motor_4_position = self.plink.channel4.position

        encoder_msg.motor_1_velocity = self.plink.channel1.velocity
        encoder_msg.motor_2_velocity = self.plink.channel2.velocity
        encoder_msg.motor_3_velocity = self.plink.channel3.velocity
        encoder_msg.motor_4_velocity = self.plink.channel4.velocity

        self.encoder_data_pub.publish(encoder_msg)

        #
        # self.get_logger().info(f"Published encoder data: {self.}")

    def publish_imu_data(self):
        """Publish IMU data at a fixed rate."""
        quat = self.plink.imu.quaternion
        gyro = self.plink.imu.gyro
        accel = self.plink.imu.accel

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.orientation.x = quat.x
        imu_msg.orientation.y = quat.y
        imu_msg.orientation.z = quat.z
        imu_msg.orientation.w = quat.w

        imu_msg.angular_velocity.x = gyro[0].item()
        imu_msg.angular_velocity.y = gyro[1].item()
        imu_msg.angular_velocity.z = gyro[2].item()

        imu_msg.linear_acceleration.x = accel[0].item()
        imu_msg.linear_acceleration.y = accel[1].item()
        imu_msg.linear_acceleration.z = accel[2].item()

        self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorGoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
