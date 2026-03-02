# V-Plotter-Drawbot
C++ control logic and inverse kinematics for a custom polar coordinate plotter.

## Inverse Kinematics

Unlike a Cartesian XY gantry, a V-plotter requires continuous calculation to translate a desired $(X, Y)$ coordinate into specific cable lengths for the left and right stepper motors. 

Let the distance between the two motor shafts be $W$. 
Setting the origin $(0,0)$ at the top-center point between the two motors, the left motor is positioned at $(-\frac{W}{2}, 0)$ and the right motor at $(\frac{W}{2}, 0)$.

To move the end-effector to a target Cartesian coordinate $(x, y)$, the microcontroller calculates the required cable lengths $L_{left}$ and $L_{right}$ using the Pythagorean theorem:

$$L_{left} = \sqrt{\left(\frac{W}{2} + x\right)^2 + y^2}$$

$$L_{right} = \sqrt{\left(\frac{W}{2} - x\right)^2 + y^2}$$

The firmware executes these calculations in real-time, converting the resulting millimeter lengths into raw step counts based on the pulley diameter and belt pitch.
