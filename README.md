# ros2-bmp280

ROS 2 driver for the Bosch BMP280 temperature and barometric pressure sensor. This package reads data over the I2C bus and publishes ROS 2 sensor messages for use in embedded or mobile robotics systems.

## Features

- Publishes temperature and pressure data via ROS 2 topics
- Configurable update rate
- Designed for Raspberry Pi running ROS 2 Jazzy Jalisco
- Easily extendable for other Bosch sensors (e.g., BME280)

---

## Installation

Clone this repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone git@github.com:JCorbin406/ros2-bmp280.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Hardware Setup

- Connect the BMP280 sensor to your Raspberry Pi via I2C
- Default I2C address: `0x76` (or `0x77` depending on breakout board)
- Enable I2C on your Raspberry Pi (via `raspi-config` or manually)

---

## Usage

### Launch the node

```bash
ros2 launch bmp280 bmp280.launch.py
```

### Parameters (in `config/bmp280.yaml`)

- `i2c_bus` (int): I2C bus number (e.g., `1` for `/dev/i2c-1`)
- `i2c_address` (int): I2C address of the sensor (`118` for `0x76`)
- `frame_id` (str): Frame name for the published messages
- `publish_rate` (float): Rate in Hz at which to publish data

---

## Topics

- `/bmp280/data` (`sensor_msgs/msg/FluidPressure` + `sensor_msgs/msg/Temperature`): Pressure and temperature readings

---

## Example Output

```bash
$ ros2 topic echo /bmp280/data
temperature: 24.87
pressure: 100325.67
```

---

## License

Licensed under the [Apache License 2.0](LICENSE).

---

## Author

Jack Corbin — [JCorbin406](https://github.com/JCorbin406)
