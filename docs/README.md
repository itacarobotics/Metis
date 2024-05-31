# Documentation

## Mechanics
All mechanical components have been designed from scratch using computer-aided design (CAD) software, specifically Autodesk Inventor 2024. Additionally, some components have been topological optimized with the software Autodesk Fusion. This process aims to optimize the material layout of mechanical components, reducing mass while increasing stiffness.

<img src="https://github.com/itacarobotics/Metis/blob/main/docs/assets/images/deltarobot.png" width="400" />

#### Actuation
Industrial delta robots commonly have a revolute actuation, which enables fast movements at the cost of larger motors, given the mechanical disadvantage of a longer arm. For this project, the actuation is performed with belt-driven prismatic joints. This design choice allows for a more compact and cost-effective drivetrain, maintaining rigidity and precision.

#### Prismatic Joints
Designing the linear rails of the prismatic joint presented one of the biggest challenges in the mechanical design process. Traditional options like linear ball bearings and linear bearing rails were deemed too heavy and expensive, while polymeric bushings introduced excessive friction. The solution implemented is a rail system with four V-shaped profile wheels rolling within the cavities of an aluminum profile, a mechanism commonly found in 3D printers. This system ensures smooth and stiff motion.

#### Spherical Joints
Each prismatic joint features two spherical pins bolted onto a threaded aluminum shaft. These shafts are held in place by a topologically optimized structure, enhancing both weight distribution and stiffness. A spherical joint, designed to fit with minimal clearance onto the spherical pin, is manufactured using resin additive manufacturing for a refined surface finish, as opposed to Fusion Deposition Modeling (FDM) technology.


#### Workspace
While designing an industrial robot, it is essential to define its workspace. The total working volume that the end-effector can reach is directly influenced by the lengths of the robot's links and the strokes of its joints. To establish the kinematic properties of the robot, a Python script has been created to visualize the workspace of the delta robot.

<img src="https://github.com/itacarobotics/Metis/blob/main/docs/assets/plots/workspace.png" width="300" />


## Electronics
The electronics bridges the gap between the high level control system and the mechanics. The motherboard of the robot communicates to the computer (master) via USB or WIFI and directly controls all sensors, actuators and other peripherals of the robot.
Given limited time for the development of this project, an open-source motherboard ([MKS-DLC32](https://github.com/makerbase-mks/MKS-DLC32)), developed for desktop engraving machines, has been implemented.

<img src="https://github.com/itacarobotics/Metis/blob/main/docs/assets/images/electronics_diagram.png"/>

#### Micro-controller
The ESP32 micro-controller is a low-power micro-controller. However, given its limited computational power, its main task is just to execute robot commands received by the main computer and directly control the low-level interfaces.

#### Motors and Motor Driver
The micro-controller has limited GPIO (General Pourpose Input Output). To solve this problem, the motherboard controls the stepper motor drivers with a shift register IO expander (74HC595). Stepper motors used for this project are NEMA17 with 60 Ncm of torque, which satisfy the project requirements.

#### Other Peripherals
The motherboard includes several peripheral interfaces essential for the robot's functionality. These peripherals include ports for limit switches and a MOSFET (Metal-Oxide-Semiconductor Field-Effect Transistor). Limit switches are used to set the zero position of the robot, when turned on. The MOSFET is integrated to enable and disable the electromagnetic gripper from the micro-controller.


## Software Architecture
The software of the delta robot is decentralized over two devices, the main computer, which is responsible for the heavier computational tasks and the micro-controller, which is responsible for controlling actuators, reading sensors and other peripherals. Overall the software architecture can be subdivided into three main sections    \item

- High-level interfaces
- Robot controller middle-ware
- Low-level firmware

#### High-Level Interfaces
The high-level interfaces is composed of all nodes the user can use to interact with the robot. For instance, the graphical user interface (GUI) helps the user control the position of the robot or, open and close the gripper. Another useful method for programming tasks of an industrial robot is the G-Code. It is a language commonly used for CNC machines or 3D printers and it consists of a set of instruction that the robot has to execute. Furthermore, computer vision algorithms can be implemented for an automated pick and place application.


#### Middle-Ware
The middle-ware robot controller is mainly responsible for handling incoming tasks, managing exceptions and computing trajectory planning and inverse geometry. It is made up of the "task scheduler" and the "robot controller" ROS2 nodes. The first one takes all incoming tasks from the high-level interfaces and add them to a queue. Once the robot is ready, the task scheduler sends a new task from the queue to the robot controller node. This last computes the robot movements needed to accomplish the desired task and sends them to the micro-controller via USB or WIFI.

#### Low-Level Firmware
The low-level firmware runs on an ESP32 micro-controller and is mainly responsible for executing commands sent by the robot controller node. In order to integrate the micro-controller with the ROS2 environment, the firmware has been developed using the micro-ROS framework.
The micro-controller directly controls the stepper motors, the gripper and reads the state of the limit switches. After having completed the task, the micro-controller sends an acknowledgment (ack) feedback to the robot controller node, which accordingly sets the new state of the robot.

![image](/docs/assets/images/nodes_diagram.png)


### Trajectory Planning


### Inverse Geometry