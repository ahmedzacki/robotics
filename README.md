# Robotics Project

## Overview
The general purpose of this robotics project is to develop a versatile autonomous robot capable of navigating and interacting with its environment using a combination of sensors and actuators. The project is structured to achieve multiple functionalities, each enhancing the robot's capabilities to perform effectively in various settings.

## Primary Goals

### Locomotion and Navigation
Enabling the robot to move precisely and smoothly in predefined patterns, both forwards and backwards, and to stop at exact distances. This involves controlling motor speeds and integrating sensor data to manage dynamics and ensure graceful movement.

### Sensor Integration and Perception
Utilizing various sensors, such as ultrasonic sensors, to gather information about the robot's surroundings. This data helps the robot make decisions about movement, detect obstacles, and maintain a specific distance from walls, enhancing its ability to navigate complex environments.

### Dynamic Interaction
Implementing algorithms that allow the robot to react to its environment in real-time. This includes following walls using a PID controller to adjust its path based on continuous sensor feedback, and modifying its behavior based on the proximity and location of nearby objects.

### Task-Specific Functionality
Programming the robot to perform specific tasks, such as using a servo-controlled ultrasonic sensor to scan its environment at multiple angles. This functionality is crucial for tasks that require detailed spatial awareness and precise positioning.

### Goal-Oriented Movements
The robot is equipped to navigate to multiple predefined locations in a sequence using coordinate-based localization techniques. This capability is fundamental for applications where the robot needs to perform tasks at various points within an environment, such as material transport or surveying areas.

## Project Structure
- `Final Code`: Contains the complete, integrated code combining all aspects of the robotics functionalities.
- `Localization`: Focuses on robotic localization techniques using sensor data to determine the robot's position in a 2D space.
- `Locomotion & Manipulation`: Covers the fundamentals of robot movement and interaction with objects, including detailed motor and servo control.
- `PID Wall Follower`: Implements a PID controller for wall-following behavior, allowing the robot to navigate smoothly along the walls.
- `Robot Control (Servo Control with US Sensor)`: Utilizes a servo and ultrasonic sensor to perform tasks that require angular positioning.

<img width="1000" height="400" alt="Screen Shot 2024-08-03 at 2 29 48 PM" src="https://github.com/user-attachments/assets/01d45ae1-e042-4ab6-9ee2-50c1300a329f">
<img width="1000" height="400" alt="Screen Shot 2024-08-03 at 2 27 43 PM" src="https://github.com/user-attachments/assets/9392a39d-e2d9-4d1d-9c3c-e6b7d0cdf91b">

