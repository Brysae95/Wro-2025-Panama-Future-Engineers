Engineering documentation
====

This repository contains engineering documentation of a self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2025.

Okay, I will now provide the entire document in English, incorporating the **Large Angular Motor 45602** and **Small Angular Motor 45607** as specified, and drawing upon the visual information of a typical Lego robot setup without mentioning the images were provided by you.

-----

## Introduction to Lego Robot for Competitions

This document details the design, construction, and programming of a mobile robot based on Lego Spike Prime, conceived for participation in competitions that require precise navigation and task completion on a delimited playing field. The prototype is equipped with a color sensor and three ultrasonic sensors, along with a **Large Angular Motor 45602** and a **Small Angular Motor 45607**, and a Spike Brain. Its design aims to maximize efficiency in competition environments, allowing for the execution of complex strategies on a field with colored lines and potential obstacles. The robot demonstrates the application of robotic engineering principles, from material selection to the implementation of advanced motion and control systems.

## Challenges and Setbacks

During the development of a competition robot, it is common to encounter various challenges that require ingenious and often iterative solutions. Some typical setbacks that often arise in these types of projects include:

  * **Navigation Precision:** Maintaining a precise trajectory and executing exact turns on a field with lines and corners is a constant challenge. The robot might struggle to follow straight lines or turn with the required accuracy, which could lead to deviations or going off the playing area. This demands constant calibration of the color sensor for line detection and fine adjustments in the motion algorithms, leveraging the precise control of the **Large Angular Motor 45602** and **Small Angular Motor 45607**, and possibly using feedback from the ultrasonic sensors for alignment or wall detection.
  * **Power Management:** Ensuring that the Spike Brain's batteries have sufficient charge to complete all tasks and that the power supply is stable for all components (especially the **Large Angular Motor 45602** and **Small Angular Motor 45607**, as well as the three ultrasonic sensors and the color sensor) can be a challenge. Voltage fluctuations or insufficient battery life could affect the robot's performance midway through a competition round.
  * **Sensor Integration (particularly ultrasonic):** The positioning and calibration of the three ultrasonic sensors are crucial. If they are not mounted at the correct height or angle, they might not reliably detect obstacles or surfaces, or even interfere with each other, compromising accurate environmental perception. The color sensor must also be optimally positioned for consistent line detection.
  * **Mechanical Design Robustness:** The robot might experience vibrations or component misalignment during rapid movement or turns. This is especially critical for Lego gears and structural connections, which must withstand stress without disassembling, particularly with the power of the **Large Angular Motor 45602**. A deficient design can lead to mechanical failures at critical moments.
  * **Code Debugging:** Logical errors in the program can lead to unexpected robot behaviors. Identifying and correcting these errors, especially those related to the sequence of actions or decision-making based on data from the color sensor and ultrasonic sensors, is time-consuming. The coordination between the different sensors and the precise control of the angular motors for advanced navigation and obstacle avoidance can be particularly complex and error-prone.

## Motivation

The primary motivation behind the creation of this robot lies in the desire to apply knowledge of robotics, programming, and engineering design in a practical and challenging context. Participating in robotics competitions fosters critical thinking, problem-solving, and teamwork. The opportunity to see a conceptual design materialize and function autonomously, solving the challenges posed by a playing field, using resources such as the Spike Brain, the **angular motors (Large Angular Motor 45602 and Small Angular Motor 45607)**, and the suite of sensors (ultrasonic and color), is a significant source of learning and satisfaction. Furthermore, interaction with other teams and the possibility of innovating with new strategies are highly motivating elements for continuous development.

## Code Explanation (Example Code - Based on Materials)

While the specific code will depend on the exact competition requirements, the general logic for a robot with these materials would focus on navigation, line following, and obstacle avoidance. The software would run on the Spike Brain, utilizing the precise angular control capabilities of the motors.

```python
# Conceptual pseudocode example for a Lego Spike Prime robot

# Import necessary Spike Prime libraries
from spike import PrimeHub, LightMatrix, Button, ForceSensor, MotionSensor, Speaker, ColorSensor, DistSensor, MotorPair, Motor
from spike.control import wait_for_seconds, wait_until

# Initialize the hub
hub = PrimeHub()

# Port definition for motors and sensors
# We assume a differential drive configuration for mobility.
# The angular motors will connect to specific Spike Brain ports.
# You can assign the Large or Small to left/right based on your build.
motor_left_drive = Motor('A') # Example: Port A for the left motor (Large or Small Angular Motor)
motor_right_drive = Motor('B') # Example: Port B for the right motor (the other Large or Small Angular Motor)

# Sensors
color_sensor_front = ColorSensor('C') # Front-mounted color sensor
ultrasonic_front = DistSensor('D') # Front ultrasonic sensor
ultrasonic_left = DistSensor('E') # Left ultrasonic sensor
ultrasonic_right = DistSensor('F') # Right ultrasonic sensor

# Control constants
BASE_SPEED = 30 # Default robot speed
PROPORTIONAL_CORRECTION = 0.5 # Constant for PID/P control for line following
BLACK_LINE_THRESHOLD = 20 # Reflection value to detect a black line (adjust based on field line color)
OBSTACLE_DISTANCE = 150 # Distance in mm to consider an obstacle (adjust threshold)

def configure_motors():
    """Stops motors at startup and ensures they are ready."""
    motor_left_drive.stop()
    motor_right_drive.stop()
    print("Motors configured.")

def read_sensors():
    """Reads and returns the current values of all sensors."""
    reflection = color_sensor_front.get_reflected_light()
    dist_f = ultrasonic_front.get_distance_mm()
    dist_l = ultrasonic_left.get_distance_mm()
    dist_r = ultrasonic_right.get_distance_mm()
    return reflection, dist_f, dist_l, dist_r

def follow_line():
    """Implements a basic line-following logic using the color sensor."""
    reflection, _, _, _ = read_sensors()
    error = BLACK_LINE_THRESHOLD - reflection # Calculate error relative to the line threshold
    correction = error * PROPORTIONAL_CORRECTION

    left_speed = BASE_SPEED + correction
    right_speed = BASE_SPEED - correction

    motor_left_drive.set_speed(left_speed)
    motor_right_drive.set_speed(right_speed)
    motor_left_drive.run()
    motor_right_drive.run()

def turn_robot_degrees(robot_degrees, speed, direction="right"):
    """Executes a precise turn of the robot in degrees.
    robot_degrees: Angle in degrees the robot should turn.
    speed: Turn speed.
    direction: "right" or "left".

    Note: The conversion from robot degrees to motor degrees depends
    on the chassis design and wheel diameter.
    This value (e.g., 'motor_degrees_per_robot_degree') must be calibrated.
    """
    motor_degrees_per_robot_degree = 2 # Example value, calibrate this. If turning 90 robot degrees requires 180 motor degrees, then 2.
    motor_degrees = robot_degrees * motor_degrees_per_robot_degree

    motor_left_drive.stop()
    motor_right_drive.stop()
    wait_for_seconds(0.1) # Small pause

    if direction == "right":
        # Run motors in opposite directions to turn on the spot
        motor_left_drive.run_for_degrees(motor_degrees, speed)
        motor_right_drive.run_for_degrees(-motor_degrees, speed)
    elif direction == "left":
        motor_left_drive.run_for_degrees(-motor_degrees, speed)
        motor_right_drive.run_for_degrees(motor_degrees, speed)

    motor_left_drive.stop()
    motor_right_drive.stop()
    wait_for_seconds(0.5) # Ensure the turn completes

def avoid_obstacle():
    """Logic to detect and avoid obstacles using the three ultrasonic sensors."""
    _, dist_f, dist_l, dist_r = read_sensors()

    if dist_f is not None and dist_f < OBSTACLE_DISTANCE:
        print(f"Front obstacle detected at {dist_f} mm. Avoiding...")
        motor_left_drive.run_for_degrees(-200, BASE_SPEED) # Reverse a bit
        motor_right_drive.run_for_degrees(-200, BASE_SPEED)
        wait_for_seconds(1)

        # Decide avoidance direction based on side sensors
        if dist_l is not None and dist_r is not None:
            if dist_l > dist_r: # More space to the left
                turn_robot_degrees(90, BASE_SPEED, "left") # Turn 90 robot degrees
            else: # More space to the right
                turn_robot_degrees(90, BASE_SPEED, "right")
        else: # If side sensors are unreliable, turn in a predefined direction
            turn_robot_degrees(90, BASE_SPEED, "right")

        # Optional: Move forward a bit after turning to clear the obstacle
        motor_left_drive.run_for_degrees(100, BASE_SPEED)
        motor_right_drive.run_for_degrees(100, BASE_SPEED)
        wait_for_seconds(0.5)

def main():
    """Main function controlling the robot's behavior."""
    configure_motors()
    hub.light_matrix.write('GO') # Visual indicator on the Hub
    print("Starting robot program...")

    while True:
        line_reflection, dist_f, dist_l, dist_r = read_sensors()

        # Prioritize obstacle detection and avoidance with the front sensor
        if dist_f is not None and dist_f < OBSTACLE_DISTANCE:
            avoid_obstacle()
            continue # Recheck sensors after avoidance

        # Line following or junction management logic
        if line_reflection < BLACK_LINE_THRESHOLD: # If the color sensor detects the line
            follow_line()
        else: # If it deviates from the line or reaches a junction/end of line
            # More advanced logic would be needed here to navigate intersections or recover the line.
            motor_left_drive.stop()
            motor_right_drive.stop()
            print("Line not detected or possible junction. Strategic decisions here.")
            # Could search for the line, or follow a predefined turning routine for a junction
            wait_for_seconds(0.1)

        wait_for_seconds(0.01) # Small pause to avoid excessive CPU usage

if __name__ == "__main__":
    main()
```

**Code Explanation with Provided Materials:**

  * **Spike Brain (PrimeHub):** This is the microcontroller that executes the code and coordinates all robot operations. All interactions with motors and sensors are performed through it, using the Spike Prime API.
  * **Large Angular Motor 45602 and Small Angular Motor 45607:** These motors, with their high-precision encoders, allow for exact control of angular position and speed. They are ideal for the robot's propulsion, typically configured in a differential drive system. The Large Angular Motor 45602, being more powerful, is generally used where greater force is required, while the Small Angular Motor 45607, being more compact, is excellent for the other drive wheel or for auxiliary mechanisms that need defined angular movements. The code controls them using `set_speed()` for continuous motion and `run_for_degrees()` for movements with precise positioning.
  * **Color Sensor:** Fundamental for navigation, it is used to detect lines on the playing field. The `get_reflected_light()` function is key to implementing line-following algorithms, allowing the robot to stay on the correct path.
  * **3 Ultrasonic Sensors (DistSensor):** These sensors are crucial for the robot's spatial awareness. Located at the front, left, and right, they enable the robot to detect the presence and distance to obstacles, walls, or other environmental elements. The `get_distance_mm()` function is used to feed the obstacle avoidance logic and, potentially, precise alignment.
  * **Spike Cables:** These cables are the physical means of connection between the Spike Brain, the motors, and the sensors, ensuring the transmission of power and data for the system's proper functioning.

## Materials

The robot is predominantly constructed with Lego Technic and Lego Spike Prime components, offering excellent flexibility for prototyping and assembly. Key materials include:

  * **Lego Technic Parts:** These constitute the robot's frame, forming the chassis, sensor and motor mounts, and any additional structural elements. They provide the necessary robustness and versatility for assembly.
  * **Spike Brain (PrimeHub):** The electronic heart of the robot, integrating the microcontroller, rechargeable battery, and ports for connecting all peripherals.
  * **Large Angular Motor 45602:** A larger and more powerful Lego Spike Prime motor, which, thanks to its encoder, allows for precise angular position control. It is ideal for the robot's primary propulsion, providing the necessary force for movement and carrying loads.
  * **Small Angular Motor 45607:** A more compact Lego Spike Prime motor, also with an encoder for position control. It can be used as the second motor in the differential drive, complementing mobility, or for activating specific auxiliary mechanisms for the challenge, such as articulated arms, lifts, or grippers that require defined and repeatable angular movements.
  * **Color Sensor:** A Lego Spike Prime sensor capable of detecting colors and the intensity of reflected light. Its primary function is line following on the playing field and identifying specific markers.
  * **3 Ultrasonic Sensors:** Lego Spike Prime distance sensors, strategically arranged (one front and two side) to detect obstacles, measure distances to walls and environmental elements, and facilitate precise navigation and collision avoidance.
  * **4 Wheels:** The wheels are essential for the robot's mobility. Typically, two of them are driven (powered by the angular motors) and the other two are passive or support wheels that provide stability and facilitate turns.
  * **Spike Cables:** The specific Lego Spike Prime connection cables that establish electrical and data communication between the Spike Brain and all connected motors and sensors.

## Mobility Management

Mobility management is fundamental to the robot's success on the field. This robot's design relies on a differential drive system, a common and effective configuration in mobile robotics:

  * **Differential Drive with Large and Small Angular Motors:** The robot uses a differential drive system, where the **Large Angular Motor 45602** and **Small Angular Motor 45607** (or the two angular motors assigned to propulsion) independently control two drive wheels. The total of **4 wheels** ensures stability, with the non-driven wheels providing support. By precisely controlling the speed and rotation of these two angular motors, the robot can move forward, backward, and turn on its own axis with high precision, granting it excellent maneuverability.
  * **Precise Line Following:** The **Color Sensor** is positioned to scan the lines on the playing field. Mobility management relies on continuous feedback from this sensor to keep the robot on the desired trajectory. The control algorithm dynamically adjusts the power sent to each angular motor to correct any deviation, ensuring smooth and precise line following.
  * **Distance-Based Navigation and Orientation:** The **three ultrasonic sensors** (front, left, and right) allow the robot to perceive its environment in three dimensions. This is crucial for mobility management in tasks such as:
      * **Obstacle Avoidance:** Detecting objects in the path and maneuvering to go around them or avoid collisions.
      * **Wall/Object Alignment:** Using side distances to maintain a constant separation from walls or to precisely position itself in front of specific field elements.
      * **Boundary and Zone Detection:** Confirming that the robot is operating within the defined limits of the playing field.
  * **Turning Precision:** The ability to execute exact turns is vital for navigating complex field patterns. This is achieved through precise control of the **Large Angular Motor 45602** and **Small Angular Motor 45607** by degrees of rotation (using their internal encoders), assisted by detecting key points on the map with the **Color Sensor** and verifying alignment with the **ultrasonic sensors**.

## Motor Selection and Implementation

Motors are the actuators that provide motion and manipulation capabilities to the robot.

  * **Large Angular Motor 45602:** This larger and more powerful motor, with its precise angular position control capability thanks to its encoder, is ideal for the robot's primary propulsion. Its robustness and torque make it suitable for moving the structure and the **4 wheels**, especially in a differential drive system where force and positioning capability are crucial.
  * **Small Angular Motor 45607:** The more compact motor, also with an encoder for position control, can be used as the second motor in the differential drive, complementing mobility with the Large Angular Motor 45602. Alternatively, it is ideal for activating specific auxiliary mechanisms for the challenge, such as grippers, lifting arms, or delivery systems that require defined and repeatable angular movements in tighter spaces.
  * **Placement and Mounting:** The angular motors must be securely mounted on the chassis to efficiently drive the wheels. Their positioning is critical to ensure proper weight distribution and optimal power transmission to the **4 wheels**.
  * **Electrical and Data Connection:** Both angular motors connect to the **Spike Brain** via **Spike Cables**. These cables not only provide the necessary power for their operation but also transmit speed and direction control signals from the **Spike Brain**, and allow reading of the internal encoders for highly precise position and angle control.

## Implementation of Gear Systems

Gear systems are crucial for optimizing the power, speed, and direction of motion transferred from the angular motors to the **4 wheels** and any auxiliary mechanisms.

  * **Speed Reduction/Torque Increase:** It is common for the **Large Angular Motor 45602** and **Small Angular Motor 45607** to be connected to the wheels via a gear reduction system. This means a smaller gear on the motor's axle drives a larger gear on the wheel's axle. The goal is to increase the torque available at the **4 wheels**, allowing the robot to move with greater force, climb slopes, overcome small obstacles, and maintain stability in controlled movements, at the expense of maximum speed.
  * **4-Wheel Drive Transmission:** Depending on the design, if all **4 wheels** are driven, a more complex gear system would be required to effectively distribute power from the two angular motors to each wheel. If only two wheels are driven, the gears would focus on transmitting power to those wheels, while the others would act as passive caster wheels.
  * **Efficiency and Robustness:** The gear system design must minimize friction and backlash to maximize efficiency and prevent gears from disengaging under load. A compact and well-integrated mounting within the chassis is essential for the robot's durability and overall performance in a competition environment. The precise control offered by the angular motors (thanks to their encoders) directly benefits from a well-designed gear system, as any backlash can affect the accuracy of angular positioning.

## Chassis Design and Component Mounting

The chassis is the fundamental structure of the robot, and its design is key to stability, maneuverability, and the protection of all components.

  * **Compact and Low Structure:** A relatively compact and low chassis contributes to a low center of gravity, which improves the robot's stability, especially during rapid turns or abrupt movements. This is vital for maintaining control on a competition field.
  * **Optimized Weight Distribution:** The strategic placement of the **Spike Brain** (the heaviest component) and the **Large Angular Motor 45602** and **Small Angular Motor 45607**, along with the **sensors (Color and Ultrasonic)**, is fundamental for achieving a balanced weight distribution. Good distribution maximizes traction on the **4 wheels** and enhances the robot's overall stability.
  * **Precise Sensor Mounting:** The **three ultrasonic sensors** and the **color sensor** must be precisely mounted at the front of the robot. Their position and height are crucial to ensure a clear view of the field lines, obstacles, and the surrounding environment, without being obstructed by other parts of the structure. Side ultrasonic sensors require mounting that allows for effective readings without interference.
  * **Cable Organization:** The routing and organization of the **Spike Cables** are important aspects of the chassis design. Cables must be guided and secured to prevent them from snagging on moving parts, getting damaged, or accidentally disconnecting during robot operation.
  * **Robust Wheel Support:** The **4 wheels** must be securely attached to the structure using Lego Technic axles and supports. Robust mounting ensures that the wheels rotate freely, with minimal friction, and maintain alignment for precise and consistent movement, which is vital for leveraging the angular motors' precise control.

## Power and Sensor Management

The robot's reliability in competition largely depends on efficient power management and the correct functionality of its sensors.

  * **Power Source (Spike Brain):** The **Spike Brain (PrimeHub)** not only acts as the robot's brain but also houses the rechargeable battery, which is the sole power source for all components: the **Large Angular Motor 45602**, the **Small Angular Motor 45607**, the **Color Sensor**, and the **three Ultrasonic Sensors**. Battery life planning is crucial to ensure the robot can complete all tasks without interruption, especially considering that angular motors can consume significant power during positioning and continuous movement tasks.
  * **Power Distribution:** The **Spike Brain** is responsible for intelligent power distribution through its ports. Each motor and sensor connects to a specific port and receives the necessary power and control signals for its operation.
  * **Sensor Connection:** All **sensors (Color and the 3 Ultrasonic)** connect to the **Spike Brain** using **Spike Cables**. It is essential that these connections are secure and that the cables are well-managed to prevent accidental disconnections or interference.
  * **Sensor Data Reading and Processing:** The robot's software is designed to continuously read data from all sensors (reflection values from the **Color Sensor**, distances from the **Ultrasonic Sensors**). This data is processed in real-time to make informed decisions about navigation, obstacle avoidance, line detection, and interaction with the playing field environment.
  * **Sensor Calibration:** A critical aspect of sensor management is their periodic calibration. This involves adjusting detection thresholds (for example, to differentiate between line colors or to determine the precise distance to an obstacle) to adapt to varying lighting conditions or specific playing field properties. In the case of the **three ultrasonic sensors**, it is also important to calibrate them to minimize crosstalk or interference between them if they are mounted very close, and to ensure their readings are consistent with the angular motors' control needs.
