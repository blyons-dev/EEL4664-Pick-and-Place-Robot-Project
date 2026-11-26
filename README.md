# EEL4664-Pick-and-Place-Robot-Project
## Project Overview
This project implements kinematic modeling, numerical inverse kinematics, and physical control of a miniArm robotic manipulator. Using Craig's Modified Denavit-Hartenberg convention, the project calculates the robot's end-effector position and determines the joint configurations required to reach specified XYZ coordinates.

The project uses MATLAB for kinematic simulation and a Python GUI with Arduino firmware for physical robot control. The completed system demonstrates Cartesian XYZ positioning and pick-and-place operations involving randomly positioned blocks.

## Repository Contents
The repository contains the following primary components:
- **MATLAB:** Forward kinematics, Jacobian calculations, inverse kinematics simulation, and collision verification.
- **Python GUI:** Graphical interface for XYZ positioning, inverse kinematics, servo control, and pick-and-place operations.
- **Arduino Firmware:** Controls the miniArm servos and executes movement commands received from the Python interface.
- **Calibration Utility:** Allows physical servo calibration and configuration of the claw's open and closed positions.

## Setup and Installation
### Requirements
- MATLAB (Base MATLAB; Robotics System Toolbox optional).
- Python 3.
- Arduino IDE.
- Arduino-compatible control board and miniArm robotic manipulator.
- Servo library.
- FastLED library version 3.10.3.
### 1. Clone the Repository
```bash
git clone https://github.com/YOUR_USERNAME/eel4664-robotic-arm-project-miniArm.git
```
Open the cloned repository in Visual Studio Code or your preferred development environment.
### 2. Run MATLAB Simulation
Open the following directory as the MATLAB current folder:
```text
lab03_inverse_kinematics/matlab
```
Run the following scripts:
```matlab
verify_miniarm
verify_collision
demo_ik
```
To test a specific XYZ target:
```matlab
demo_ik([-0.040; 0.100; 0.150])
```
This example commands the simulated robot to reach X = -40 mm, Y = 100 mm, and Z = 150 mm.
**Note:** MATLAB uses meters for Cartesian coordinates.
### 3. Run Python GUI Simulation
Navigate to the Python control directory:
```bash
cd lab03_inverse_kinematics/miniarm_controlC
```
Install the required dependencies:
```bash
python -m pip install -r requirements.txt
```
Launch the GUI in simulation mode:
```bash
python miniarm_controlC.py --simulate
```
Enter the desired XYZ coordinates in millimeters, select **Calculate / Preview**, and then select **Move Arm** after verifying the calculated configuration.
### 4. Connect to the Physical miniArm
Upload the Arduino firmware located at:
```text
miniarm_controlC/miniarm_controlC.ino
```
Install the required Arduino libraries and select the appropriate board and COM port.
Update the `SERIAL_PORT` variable near the top of `miniarm_controlC.py` to match the connected Arduino's COM port:
```python
SERIAL_PORT = "COMX"
```
Replace `COMX` with the actual assigned port.
Launch the GUI without simulation mode:
```bash
python miniarm_controlC.py
```
Verify that the GUI displays the **REAL ROBOT** banner and the expected GUI v2.2.1 / Firmware 2.1.0 versions.

**Important:** Physical servo initialization and measured calibration must be completed before attempting robot movements. Follow the lab calibration procedure using `miniarm_calibrate.py` and save the measured `calibration.json` file. This file is intentionally excluded from Git tracking and must be submitted separately.

## Modifications to Instructor-Provided Code
The following modifications were made to the original instructor-provided project files:
- 

## Demonstration Videos
The following videos demonstrate the MATLAB simulation and physical operation of the miniArm.
| Demonstration | Video Link |
|---|---|
| MATLAB Inverse Kinematics Simulation | [Insert Video Link] |
| Physical XYZ Positioning | [Insert Video Link] |
| Physical Pick-and-Place Demonstration | [Insert Video Link] |
