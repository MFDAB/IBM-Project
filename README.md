# IBM-Project

# Project Overview
The IBM Warehouse Project is a collaborative initiative where my teammates and I, as students, worked alongside industrial partners to develop and evaluate a drone-based QR code scanning system for smart warehousing applications. The primary objective was to automate inventory tracking and management using drones, minimizing human intervention while increasing efficiency and accuracy in warehouse operations.

Our project focused on deploying a Tello drone equipped with vision-based navigation techniques to scan QR codes placed on storage cabinets. The scanned QR codes would then be transmitted to a PC, simulating an automated warehouse inventory system.

# Project Scope
The project involved a well-defined demonstration scenario, which was executed in a controlled indoor environment. The key tasks included:

Drone Takeoff: The drone starts from a designated takeoff point (a table).
Autonomous Navigation: The drone flies toward a pre-defined position near a cabinet.
QR Code Scanning: The drone scans a QR code placed on the cabinet.
Data Transmission: The scanned QR code data is transmitted to a PC for further processing.
Return Flight: The drone safely returns to its takeoff point.

# Navigation Approaches Evaluated
To determine the most effective method for autonomous drone navigation, we tested and compared different approaches:

## Hardcoded Navigation (NYP Student Implementation)

The drone follows a predefined flight path with fixed waypoints.
Simple but lacks adaptability to dynamic environments.

## Tello Guidance Pad (NYP Student Implementation)

Utilizes Tello’s built-in guidance pad system for precise landing and positioning.
Improved accuracy compared to hardcoded navigation.

## AprilTags and ORB SLAM (Implemented by Ting An)

Uses AprilTags (fiducial markers) to assist in localization.
ORB SLAM (Oriented FAST and Rotated BRIEF Simultaneous Localization and Mapping) provides real-time visual odometry.
Most advanced and flexible approach for real-world warehouse settings.
# QR Code Evaluation
To determine the most suitable QR code size for drone-based scanning, we tested three different sizes:

5 cm x 5 cm
10 cm x 10 cm
15 cm x 15 cm
The evaluation considered factors such as scanning accuracy, recognition speed, and drone stability at various distances.

Experimental Setup
The testing environment was designed to replicate a small-scale warehouse setting:

Takeoff & Landing Position: A table near a workstation.
Flight Path: The drone covers a distance of approximately 3.5m – 4m to reach the scanning position.
Scanning Position: The drone stops 1.5m – 2m from the cabinet.
QR Code Placement: QR codes were placed at two different heights (0.6m above ground).
Return Path: After scanning, the drone returns to the starting position.
Challenges Encountered
Throughout the project, we encountered several challenges that required innovative problem-solving:

Stability & Positioning:

The drone’s small size made it susceptible to minor air turbulence.
Adjustments in flight speed and altitude were needed to maintain a stable scanning position.
QR Code Recognition:

The smallest QR code (5 cm x 5 cm) was difficult to scan from a distance.
Optimal performance was achieved with the 10 cm x 10 cm and 15 cm x 15 cm QR codes.
Navigation Accuracy:

Hardcoded flight paths lacked flexibility.
ORB SLAM with AprilTags provided better adaptability but required significant computational resources.
