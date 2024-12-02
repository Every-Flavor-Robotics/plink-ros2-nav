import time

import numpy as np
import rclpy
from motorgo import ControlMode
from motorgo.plink import Plink
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Imu

from motorgo_msgs.msg import (  # Replace <package_name> with your package name
    MotorGoEncoderData,
    MotorGoMotorCommand,
)


class MotorGoNode(Node):
    def __init__(self):
        super().__init__("motorgo_node")

        self.declare_parameter("frequency", 150)
        self.declare_parameter("motor_command_timeout", 0.5)

        # Load all parameters
        self.frequency = self.get_parameter("frequency").value
        self.motor_command_timeout = self.get_parameter("motor_command_timeout").value

        # Start up the motor controller
        self.plink = Plink(self.frequency)
        self.plink.connect()

        # Velocity PID parameters for each motor
        self.motors = {
            "motor_1": self.plink.channel1,
            "motor_2": self.plink.channel2,
            "motor_3": self.plink.channel3,
            "motor_4": self.plink.channel4,
        }

        for motor in self.motors.keys():
            self.declare_parameter(f"{motor}/velocity_pid/p", 0.0)
            self.declare_parameter(f"{motor}/velocity_pid/i", 0.0)
            self.declare_parameter(f"{motor}/velocity_pid/d", 0.0)
            self.declare_parameter(f"{motor}/velocity_pid/output_ramp", 10000.0)
            self.declare_parameter(f"{motor}/velocity_pid/lpf", 0.00001)

        # Set velocity PID parameters for each motor
        for name, channel in self.motors.items():
            p = self.get_parameter(f"{name}/velocity_pid/p").value
            i = self.get_parameter(f"{name}/velocity_pid/i").value
            d = self.get_parameter(f"{name}/velocity_pid/d").value
            output_ramp = self.get_parameter(f"{name}/velocity_pid/output_ramp").value
            lpf = self.get_parameter(f"{name}/velocity_pid/lpf").value

            channel.set_velocity_pid_gains(
                p=p, i=i, d=d, output_ramp=output_ramp, lpf=lpf
            )

        self.add_on_set_parameters_callback(self.on_parameter_change)

        # Set control mode to POWER
        self.plink.channel1.control_mode = ControlMode.NONE
        self.plink.channel2.control_mode = ControlMode.NONE
        self.plink.channel3.control_mode = ControlMode.NONE
        self.plink.channel4.control_mode = ControlMode.NONE

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

        self.last_command_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.1, self.check_motor_command_timeout)
        self.watchdog_timeout = False

        self.get_logger().info("motorgo_node has been initialized")

    def motor_command_callback(self, msg):
        """Callback to handle incoming motor commands."""

        self.last_command_time = self.get_clock().now()

        # Set control mode for each motor
        self.plink.channel1.control_mode = msg.motor_1_control_mode
        self.plink.channel2.control_mode = msg.motor_2_control_mode
        self.plink.channel3.control_mode = msg.motor_3_control_mode
        self.plink.channel4.control_mode = msg.motor_4_control_mode

        # Set power, velocity, or position for each motor
        if msg.motor_1_control_mode == ControlMode.VELOCITY:
            self.plink.channel1.velocity_command = msg.motor_1_command
        elif msg.motor_1_control_mode == ControlMode.POWER:
            self.plink.channel1.power_command = msg.motor_1_command

        if msg.motor_2_control_mode == ControlMode.VELOCITY:
            self.plink.channel2.velocity_command = msg.motor_2_command
        elif msg.motor_2_control_mode == ControlMode.POWER:
            self.plink.channel2.power_command = msg.motor_2_command

        if msg.motor_3_control_mode == ControlMode.VELOCITY:
            self.plink.channel3.velocity_command = msg.motor_3_command
        elif msg.motor_3_control_mode == ControlMode.POWER:
            self.plink.channel3.power_command = msg.motor_3_command

        if msg.motor_4_control_mode == ControlMode.VELOCITY:
            self.plink.channel4.velocity_command = msg.motor_4_command
        elif msg.motor_4_control_mode == ControlMode.POWER:
            self.plink.channel4.power_command = msg.motor_4_command

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

    def on_parameter_change(self, params):
        """Callback to handle parameter changes."""
        success = True

        # Temporary storage for batched updates
        motor_updates = {
            motor: {"p": None, "i": None, "d": None, "output_ramp": None, "lpf": None}
            for motor in self.motors.keys()
        }

        # Loop through all parameters and batch updates for each motor
        for param in params:
            try:
                if param.name.startswith("motor_") and "/velocity_pid/" in param.name:
                    # Parse the motor name and PID parameter from the parameter name
                    motor_name, pid_param = param.name.split("/velocity_pid/")
                    if motor_name in motor_updates:
                        motor_updates[motor_name][pid_param] = param.value
                    else:
                        self.get_logger().error(
                            f"Invalid motor name in parameter: {motor_name}"
                        )
                        success = False
                else:
                    # Log and ignore any unhandled parameters
                    self.get_logger().info(f"Ignoring parameter update: {param.name}")
            except Exception as e:
                self.get_logger().error(
                    f"Failed to update parameter {param.name}: {str(e)}"
                )
                success = False

        # Apply batched updates for each motor
        for motor_name, updates in motor_updates.items():
            channel = self.motors.get(motor_name)
            if updates:
                try:
                    # Apply all updates in one go
                    channel.set_velocity_pid_gains(**updates)
                    # Log all changes that are not None
                    for param, value in updates.items():
                        if value is not None:
                            self.get_logger().info(
                                f"Updated {motor_name} {param} to {value}"
                            )
                except Exception as e:
                    self.get_logger().error(
                        f"Failed to apply updates for {motor_name}: {str(e)}"
                    )
                    success = False

        return SetParametersResult(successful=success)

    def check_motor_command_timeout(self):
        """Check if motor commands have timed out."""
        time_since_last_command = (
            self.get_clock().now() - self.last_command_time
        ).nanoseconds / 1e9

        if time_since_last_command > self.motor_command_timeout:
            if not self.watchdog_timeout:
                self.get_logger().warn("Motor command timeout. Stopping all motors.")

                self.watchdog_timeout = True

                # Stop all motors
                if self.plink.channel1.control_mode != ControlMode.NONE:
                    self.plink.channel1.power_command = 0
                    self.plink.channel1.velocity_command = 0
                if self.plink.channel2.control_mode != ControlMode.NONE:
                    self.plink.channel2.power_command = 0
                    self.plink.channel2.velocity_command = 0
                if self.plink.channel3.control_mode != ControlMode.NONE:
                    self.plink.channel3.power_command = 0
                    self.plink.channel3.velocity_command = 0
                if self.plink.channel4.control_mode != ControlMode.NONE:
                    self.plink.channel4.power_command = 0
                    self.plink.channel4.velocity_command = 0

                # Set control mode to NONE to ensure motors are safe
                self.plink.channel1.control_mode = ControlMode.NONE
                self.plink.channel2.control_mode = ControlMode.NONE
                self.plink.channel3.control_mode = ControlMode.NONE
                self.plink.channel4.control_mode = ControlMode.NONE

        else:
            if self.watchdog_timeout:
                self.get_logger().info("Motor command timeout cleared.")
            self.watchdog_timeout = False


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
