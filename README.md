# Project NUEVO
![](/assets/NUEVO.png)

Lab project material for the MAE 162 robotics course (Winter/Spring 2026).

## Project Overview

A modular two-wheeled mobile robot platform designed for hands-on robotics education. Features customizable manipulators and a dual-layer control architecture for teaching embedded systems, ROS2, and mechatronics fundamentals.

## Problem

The engineering problem given for this project is to create an autonomous robot that is capable of assembling and delivering a 3D-printed “burger” to the correct customer after successfully navigating obstacles.

## Design and Approach

Mechanical: In order to complete these tasks, the robot must have a pick-up mechanism, a lidar sensor, cameras, motors, and a PCB. These components must be designed and integrated in a way that handles the following course specifications. Our robot contains three main unique mechanical components: static platform, dynamic platform, and a push/pull mechanism. Each of these are simple to manufacture and serve a different purpose for obtaining, containing, and delivering the burger. The static platform is where the burger rests while the robot is in motion while the dynamic platform moves up and down to align the push/pull mechanism. The push/pull mechanism slides the burger onto and off of the robot.

Electrical: The robot uses one camera to detect stoplights and perform facial recognition. It also has limit switches to home the mechanism.

Software: At the highest level, the software is organized around a finite-state machine (FSM) that governs the robot's behavior throughout the mission. The FSM provides a deterministic method of coordinating actions by dividing the overall task into a series of clearly defined operational states. The robot begins in an initialization state where sensors, navigation systems, and hardware interfaces are configured. It then transitions to an idle state where it waits for user input before beginning operation. Once activated, the robot executes a sequence of behaviors that include perception-based decision making, navigation to multiple target locations, manipulation of objects using a lift and arm mechanism, obstacle avoidance, and final delivery. By separating the mission into distinct states, the software remains organized, predictable, and easy to maintain while ensuring that only the appropriate actions are executed at any given time.

Testing: We validated our robot through subsystem and full-system testing to ensure that it met all the project requirements. Individual mechanisms such as the lift, push/pull mechanism, limit switches, camera, and LiDAR were all tested independently before being integrated into the final robot. We validated homing routines by repeatedly cycling the lift mechanism and verifying that zero positions were reproduced. Camera detection was tested for traffic lights, stop signs, and object detection and LiDAR was validated by introducing random obstacles and observing path correction. After subsystem testing, full-system validation was performed by repeated runs of the course. Successful operation required the robot to detect the green light, collect all three burger ingredients, navigate the ramp and obstacle field, identify the correct customer, and deliver the completed burger. For quantitative analysis, we used measurable metrics to help determine the performance of various robot tasks. The total mission completion time was measured from the moment the green light was detected until the robot delved the burger. Successful runs consistently remained under the seven minute customer satisfaction requirement. Obstacle avoidance capability was tested by recording the number of collisions with cones or walls during repeated runs.

## Results

The robot can successfully assemble a burger and navigate up and down ramps and through obstacles. Where we struggled was final integration of all robot capabilities including facial recogniation and delivery, which worked individually just not integrated into a complete main code.

## Team Contributions

Colin and Scott: Software Leads
Allison and Oscar: Firmware Leads
Sheryl, Elizabeth, and Oscar: Mechanical Leads


## System Architecture

**Low-Level Control (Arduino)**
- Real-time motor control (DC, stepper, servo)
- GPIO, LEDs, and button inputs
- UART communication to Raspberry Pi

**High-Level Control (Raspberry Pi 5 + ROS2)**
- Decision-making and path planning
- Camera and GPS sensor processing
- ROS2 node orchestration

**Custom PCB**
- Integrates Arduino, motor drivers, and power management
- Standardized interface for educational reproducibility

## Repository Structure

```
├── firmware/       Arduino firmware and firmware-specific docs
├── nuevo_ui/       Raspberry Pi bridge + web UI
├── ros2_ws/        ROS2 workspace and Pi-side tests
├── tlv_protocol/   TLV type definitions, payload schemas, generators
├── NUEVO board/    PCB design files (schematics, layouts, BOM)
├── mechanical/     CAD files for chassis and manipulators
├── docs/           Cross-project architecture, protocol, and design docs
└── assets/         Shared repo assets
```



## Key Documents

| Document | Purpose |
|----------|---------|
| [docs/README.md](docs/README.md) | Cross-project documentation map and source-of-truth index |
| [docs/COMMUNICATION_PROTOCOL.md](docs/COMMUNICATION_PROTOCOL.md) | Current human-readable source of truth for protocol behavior, framing, and logical TLV design |
| [docs/DESIGN_GUIDELINES.md](docs/DESIGN_GUIDELINES.md) | Cross-project conventions, numbering rules, and protocol update workflow |
| [tlv_protocol/TLV_Payloads.md](tlv_protocol/TLV_Payloads.md) | Exact payload layouts and sizes |
| [firmware/README.md](firmware/README.md) | Arduino firmware overview, current features, and build instructions |
| [firmware/docs/README.md](firmware/docs/README.md) | Firmware subsystem documentation index |
| [NUEVO board/SPECIFICATIONS.md](NUEVO%20board/SPECIFICATIONS.md) | PCB hardware specifications |

## Technologies

- **Embedded**: Arduino (C/C++)
- **High-Level**: ROS2 (Python/C++), Raspberry Pi 5
- **Communication**: UART serial protocol
- **Sensors**: Camera, GPS, encoders
- **Hardware**: Custom PCB, stepper/servo motors
