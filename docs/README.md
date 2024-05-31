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

#### High-Level Interface
The high-level interface is composed of all nodes the user can use to interact with the robot. For instance, the graphical user interface (GUI) helps the user control the position of the robot or, open and close the gripper. Another useful method for programming tasks of an industrial robot is the G-Code. It is a language commonly used for CNC machines or 3D printers and it consists of a set of instruction that the robot has to execute. Furthermore, computer vision algorithms can be implemented for an automated pick and place application.


#### Middle-Ware
The middle-ware robot controller is mainly responsible for handling incoming tasks, managing exceptions and computing trajectory planning and inverse geometry. It is made up of the "task scheduler" and the "robot controller" ROS2 nodes. The first one takes all incoming tasks from the high-level interfaces and add them to a queue. Once the robot is ready, the task scheduler sends a new task from the queue to the robot controller node. This last computes the robot movements needed to accomplish the desired task and sends them to the micro-controller via USB or WIFI.

#### Low-Level Firmware
The low-level firmware runs on an ESP32 micro-controller and is mainly responsible for executing commands sent by the robot controller node. In order to integrate the micro-controller with the ROS2 environment, the firmware has been developed using the micro-ROS framework.
The micro-controller directly controls the stepper motors, the gripper and reads the state of the limit switches. After having completed the task, the micro-controller sends an acknowledgment (ack) feedback to the robot controller node, which accordingly sets the new state of the robot.

![image](/docs/assets/images/nodes_diagram.png)


## Trajectory Planning
For trajectories planned in task space, the path of the end-effector is interpolated with multiple via-points, whose the robot follows along its movement.
For this project, the path has been linearly interpolated by the function $\theta(s)$, hence the path describes a straight line from $X_0$ to $X_f$

$\theta(s) = X_0 + s \cdot (X_f-X_0), \qquad s \in [0, 1]$

The parameter $s$ is then remapped to a time scaling function in order to better control velocity and its derivatives along the path. The function $s(t)$ is interpolated with a fifth-order polynomial

$q(t) = a_0 + a_1 \cdot t + a_2 \cdot t^2 + a_3 \cdot t^3 + a_4 \cdot t^4 + a_5 \cdot t^5 \qquad t \in [0, T_f]$

In this case, the system of differential equations is

$q(0) = 0, \quad \dot{q}(0) = 0, \quad \ddot{q}(0) = 0 $
$q(T_f) = 1, \quad \dot{q}(T_f) = 0, \quad \ddot{q}(T_f) = 0 $

The closed form solution is

$a_0 = q_0, \quad a_1 = 0, \quad a_2 = 0 $
$a_3 =  \frac{10}{{T_f^3}}, \quad a_4 =  -\frac{15}{{T_f^4}}, \quad a_5 =  \frac{6}{{T_f^5}}$

<img src="https://github.com/itacarobotics/Metis/blob/main/docs/assets/plots/time_scaling.png" width="300"/>

The total time of travel $T_f$ is calculated by setting joint's velocity and acceleration constraints.
Finally, inverse geometry is computed for each via point, resulting a joint trajectory.


## Inverse Geometry
The inverse geometry involves finding the joint angles that correspond to a desired end-effector position and orientation. 
The numerical inverse geometry of a delta robot, and many other robot arms, is a non linear optimization problem, which is solved over multiple iterations refining the initial guess for the joint angles, until a solution that minimizes the error between the desired and actual end-effector positions is found.
Overall, the problem admits a solution if the desired end-effector position is within the workspace of the robot. However, if a solution exists, there might be multiple or even infinite solutions.

In particular, if the problem admits a solution, it can be solved by minimizing the cost function $c(q)$, where, $f(q)$ is the forward geometry function and $p_d$ is the desired end-effector position. The goal is to find $q$ such that $f(q) \approx p_d$.


$c(q) = ||p_d - f(q)||$
For this project, the optimization problem has been solved with the Gauss-Newton algorithm, which starts with an initial guess $q_0$ and iterates as follows:


1. Compute the error vector  $e = p_d - f(q_i)$

2. Linearize the forward geometry function $f(q_i)$ around the current guess $q_i$ to obtain the Jacobian matrix $J(q_i)$.

3. Compute the step $\Delta q_i$ using the formula: 
$J^\dagger = J(q_i)^T J(q_i) + \lambda I, \quad \lambda \propto 10^{-2}$
$\Delta q_i = (J^\dagger)^{-1} J(q_i)^T e$

*Where* $J^\dagger$ *is the regularized pseudo-inverse of the Jacobian, which makes the matrix always invertible and positive.*

4. Update the guess:
$q_{i+1} = q_i + \Delta q_i$

5. Repeat steps 1-4 until convergence criteria are met, such as the error $||e||$ falling below a threshold or the change in $q_i$ falling below a threshold.

The cost function is a non-convex function, which means, it has at least one minimum. However, the algorithm may converge to a local minimum depending on the initial guess and the characteristics of the problem.