# robot-tweezers

ELEC/CPEN 491 Team JY-062 Project Repository

This project contains the control system simulations and source code for a 3 degree of freedom (DOF) manipulator, tweezer end-effector and human inferface controller requested by UBC Studios to handle small specimines used in the photogrammetry process. 

## Requirments

To run the robots firmware, you will need to install the following software in order to build the code and program the microcontroller. 

- [Visual Studio Code](https://code.visualstudio.com/download)
- [PlatformIO IDE](https://platformio.org/install/ide?install=vscode)
- [FreeRTOS-Teensy4](https://platformio.org/lib/show/6737/FreeRTOS-Teensy4/installation)
    - Should be able to install this through PlatformIO library manager

## Analysis

The manipulator control system was developed using transformations derived from the Denavit-Hartenberg (DH) convention. Using this system, and the Paden–Kahan Subproblems, we were able to perform the inverse and direct kinematic analysis, as well as the inverse and direct dynamics analysis.

![Spherical Wrist](assets/spherical_wrist_diagram.jpg)

### Direct Kinematics 

To related the end-effector coordinate frame and origin to the three joint variables, we used the DH parameters, $\theta$, $d$, $a$, $\alpha$ to model each joints frame and origin as a function of the previous joint, and finally for the end-effector as a function of the three joints.

$$^{i-1}C_{i} = e^{\theta k\times}e^{\alpha i\times}$$
$$x_{1,2} = \frac{-b \pm \sqrt{b^2-4ac}}{2b}.$$