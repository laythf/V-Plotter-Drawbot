# V-Plotter-Drawbot

*Custom firmware for a Belt-Driven Drawing Robot (V-Plotter) featuring TMC2209 sensorless homing and real-time inverse kinematics.*

![V-Plotter Demo](https://github.com/user-attachments/assets/93bcfde7-d24c-4075-b181-9a55a270131d)

[**Demo on Vimeo (2x speed)**](https://vimeo.com/1169827064)

## Hardware Setup

I attached the Nema Steppers to 3D-printed mounts, and screwed the mounts to a wooden board.  Heavy screw/nut counterweights on the ends of the timing belt and pen assembly maintain cable tension.

![VPlotter Hardware](https://github.com/user-attachments/assets/6a8aa691-c9fe-4e1b-a41b-30d2661fc65c)

* **Microcontroller:** Arduino UNO R4 WiFi + V3.0 CNC Shield Extension Board
* **Motor Drivers:** BIGTREETECH TMC2209
* **Actuators:** * 2x Nema 17 Bipolar Steppers (8mm body)
    * 1x ES08MA II Mini Metal Gear Analog Servo (Pen lift mechanism)
* **Drivetrain:** KeeYees 5M GT2 Timing Belt (6mm width, 2mm pitch) + 20-Tooth Pulleys
* **Power:** 12V 3A Power Supply Adapter

## Control Logic & Homing

Instead of wiring up physical limit switches like a standard gantry, I configured the TMC2209 drivers to use their StallGuard feature for sensorless homing. 

On startup, the firmware drives the motors toward the physical boundaries of the frame. By monitoring the motor coil load via UART, the system detects the moment the end-effector stalls against the wall, establishing the geometric zero-points, and automatically centering itself.

## Inverse Kinematics

A V-plotter requires continuous calculation to translate target $(X, Y)$ coordinates into precise cable lengths for the left and right motors. 

Let the distance between the two motor shafts be $W$. 
Setting the origin $(0,0)$ at the top-center point between the two motors, the left motor sits at $(-\frac{W}{2}, 0)$ and the right motor at $(\frac{W}{2}, 0)$.

To move the pen to a Cartesian coordinate $(x, y)$, the microcontroller calculates the required cable lengths $L_{left}$ and $L_{right}$ using the Pythagorean theorem:

$$L_{left} = \sqrt{\left(\frac{W}{2} + x\right)^2 + y^2}$$

$$L_{right} = \sqrt{\left(\frac{W}{2} - x\right)^2 + y^2}$$

The code executes this math in real-time, converting the resulting millimeter lengths into raw stepper motor steps based on the pulley diameter and belt pitch.

## Acknowledgments

The 3D-printed mechanical components for this build are based on the [POLAR DRAWBOT](https://www.thingiverse.com/thing:798076) designs by Thingiverse user **daGHIZmo**.
