# Search and Rescue Autonomous Vehicle

## Overview
This project is about an autonomous vehicle designed for search and rescue operations. The vehicle is powered by a **Raspberry Pi 4** and an **Arduino Mega**. It uses a variety of tools and technologies to navigate and perform its tasks.

## Tools and Technologies
- **RPLiDAR**: A low cost LIDAR sensor used for 360-degree environment scanning to detect and avoid obstacles.
- **ROS (Robot Operating System)**: A flexible framework for writing robot software, used for managing various robot peripherals and sensors.
- **TCP/IP with TLS secured connection**: For secure and reliable data communication between the vehicle and the control station.
- **SLAM (Simultaneous Localization and Mapping) algorithm**: Used for mapping the environment and tracking the vehicle's location.
- **Ultrasonic Sensors**: For close range obstacle detection and avoidance.
- **Colour Sensors**: For detecting coloured objects or signals during the mission.
- **Hall Sensors**: For detecting the presence of magnetic fields.

## Hardware Setup
The hardware of this vehicle consists of a Raspberry Pi 4 and an Arduino Mega. The Raspberry Pi 4 handles high level tasks such as communication, navigation and sensor data processing. The Arduino Mega controls the motors and reads data from ultrasonic, colour and hall sensors.

## Software Setup
The software is primarily written in Python and C++. Python is used for high level tasks on the Raspberry Pi 4, such as running the ROS nodes, handling TCP/IP communication and processing sensor data. C++ is used for programming the Arduino Mega due to its efficiency and direct control over the hardware.

## Getting Started
1. Clone this repository to your local machine.
2. Install the necessary libraries and dependencies.
3. Upload the Arduino code to the Arduino Mega.
4. Run the ROS nodes on the Raspberry Pi 4.
5. Start the mission!

## Contributing
We welcome contributions! Please read our contributing guidelines before making a pull request.

## License
This project is licensed under the MIT License. See the LICENSE file for details.
