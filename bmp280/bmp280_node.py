# bmp280/bmp280_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, FluidPressure
from .driver import BMP280

class BMP280Node(Node):
    """ROS 2 node that publishes temperature and pressure data from the BMP280 sensor."""

    def __init__(self):
        super().__init__('bmp280_node')
        self.get_logger().info('Initializing BMP280 Node...')

        # Declare and get parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x76)
        self.declare_parameter('publish_rate', 10.0)  # Hz

        i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        i2c_address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Initialize sensor
        try:
            self.sensor = BMP280(bus=i2c_bus, address=i2c_address)
            self.get_logger().info(f'BMP280 initialized on I2C bus {i2c_bus} at address {hex(i2c_address)}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize BMP280: {e}')
            raise e

        # Publishers
        self.temp_pub = self.create_publisher(Temperature, 'bmp280/temperature', 10)
        self.pressure_pub = self.create_publisher(FluidPressure, 'bmp280/pressure', 10)

        # Timer
        timer_period = 1.0 / rate
        self.timer = self.create_timer(timer_period, self.publish_sensor_data)

    def publish_sensor_data(self):
        try:
            temperature, pressure = self.sensor.read_temperature_pressure()
            now = self.get_clock().now().to_msg()

            # Publish temperature
            temp_msg = Temperature()
            temp_msg.header.stamp = now
            temp_msg.header.frame_id = 'bmp280_link'
            temp_msg.temperature = temperature
            temp_msg.variance = 0.0
            self.temp_pub.publish(temp_msg)

            # Publish pressure
            pressure_msg = FluidPressure()
            pressure_msg.header.stamp = now
            pressure_msg.header.frame_id = 'bmp280_link'
            pressure_msg.fluid_pressure = pressure  # in Pascals
            pressure_msg.variance = 0.0
            self.pressure_pub.publish(pressure_msg)

        except Exception as e:
            self.get_logger().warn(f'Failed to read from BMP280: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BMP280Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('BMP280 Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
