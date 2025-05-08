# bmp280 - ROS 2 Pressure and Temperature Sensor Node

This ROS 2 package provides a Python-based node that interfaces with the BMP280 barometric pressure and temperature sensor over I2C. It publishes sensor data to standard ROS 2 topics using `sensor_msgs/msg/Temperature` and `sensor_msgs/msg/FluidPressure`.

---

## Features

- Communicates with BMP280 via I2C
- Publishes temperature in Celsius and pressure in Pascals
- Uses onboard factory calibration coefficients
- Fully configurable via YAML and launch files
- Compatible with Raspberry Pi (tested on Pi 4 with ROS 2 Jazzy)

---

## Installation

### Clone into your ROS 2 workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/JCorbin406/bmp280.git
```

### Build the workspace

```bash
cd ~/ros2_ws
colcon build --packages-select bmp280
source install/setup.bash
```

---

## Usage

### Launch the sensor node with default parameters:

```bash
ros2 launch bmp280 bmp280.launch.py
```

This loads configuration from:
```
bmp280/config/bmp280_config.yaml
```

You can also manually run the node:

```bash
ros2 run bmp280 bmp280_node
```

---

## Parameters

You can customize behavior via the YAML config file:

```yaml
bmp280_node:
  ros__parameters:
    i2c_bus: 1               # I2C bus (typically 1 on Raspberry Pi)
    i2c_address: 118         # Decimal address (118 = 0x76 or 119 = 0x77)
    publish_rate: 10.0       # Hz
```

---

## Topics

The node publishes:

| Topic               | Type                         | Description                   |
|---------------------|------------------------------|-------------------------------|
| `/bmp280/temperature` | `sensor_msgs/msg/Temperature` | Ambient temperature in Celsius |
| `/bmp280/pressure`    | `sensor_msgs/msg/FluidPressure` | Barometric pressure in Pascals |

---

## File Structure

```
bmp280/
├── config/
│   └── bmp280_config.yaml
├── launch/
│   └── bmp280.launch.py
├── bmp280/
│   ├── __init__.py
│   ├── driver.py            # Sensor driver
│   └── bmp280_node.py       # ROS 2 node
├── package.xml
├── setup.py
└── LICENSE
```

---

## License

This project is licensed under the [Apache License 2.0](LICENSE).

---

## Maintainer

Jack Corbin
[https://github.com/JCorbin406](https://github.com/JCorbin406)
