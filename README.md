# ToblerRobot-Arm-with-Custom-Firmware
<img width="1024" height="747" alt="image" src="[https://github.com/user-attachments/assets/5e556e20-8fe9-4986-92f3-9eea95d9da0d](https://github.com/KANCHANAKALANAPERERA/ToblerRobot-Arm-with-Custom-Firmware/blob/main/DSC_0513.JPG?raw=true)" />

# ToblerArm – 3D Printed Robot Arm (Zero to Advanced Guide) 🤖

**ToblerArm** is a 3D-printed robot arm based on the classic Tobler-style design, powered by:

- 🧠 **Arduino Mega 2560**  
- 🛠 **RAMPS 1.4** stepper shield  
- ⚙️ 3 × **NEMA17** stepper motors for the arm  
- ✋ 1 × **28BYJ-48** stepper motor for the gripper  

This repository is not just code.  
It is a **complete learning project** that takes you from **zero → working robot → advanced customization**.

> Author: **N. K. K. Perera (@kalanakanchana)**  
> Project name: **ToblerArm**  
> Target users: **Beginners to hobby-level advanced makers**

---

## 0. What You Can Do With ToblerArm

With **ToblerArm** you can:

- Move a real robot arm in **XYZ coordinates**, not just “spin motor 1”.
- Teach it **pick-and-place** motions using a simple serial protocol.
- Learn how **inverse kinematics**, **interpolation**, and **stepper control** work in practice.
- Extend the system with your own commands, tools, and software.

If you can:

- Upload code to an Arduino  
- Use a screwdriver  
- Follow a wiring diagram  

…then you can build this robot.

---

## 1. Features (Beginner → Advanced)

### Beginner-friendly

- Clear, commented firmware structure.
- Simple, text-based commands (G/M-code style).
- Example Python script to control the arm from a PC.

### Intermediate

- Homing using **limit switches** (`G28`).
- **Absolute** and **relative** coordinate modes (`G90`, `G91`).
- Smooth, cosine-based interpolation between positions (no jerky moves).

### Advanced

- **Inverse kinematics**: firmware converts XYZ → joint angles → motor steps.
- Modular architecture:
  - `RobotGeometry` – math for the arm.
  - `Interpolation` – motion planning.
  - `RampsStepper` – stepper control.
  - `Command + Queue` – serial protocol and command queue.
- Easy to extend with new commands (e.g., custom tools, macros).

---

## 2. Hardware Overview

### 2.1 Electronics

Required main components:

- 1 × **Arduino Mega 2560**
- 1 × **RAMPS 1.4** shield
- 3 × **Stepper drivers** (A4988 or DRV8825) for NEMA17 motors
- 3 × **NEMA17** stepper motors (base, shoulder, elbow)
- 1 × **28BYJ-48** stepper motor (for gripper)
- 1 × **12 V power supply** (3–5 A is fine for light loads)
- Jumper wires, Dupont connectors, screw terminals, cable ties

### 2.2 Switches / Sensors / IO

- 3 × limit switches for axes:
  - **X_MIN** – upper arm or shoulder
  - **Y_MIN** – lower arm
  - **Z_MIN** – base rotation
- 2 × limit switches for the gripper:
  - `Gripper_zero_min` – fully closed
  - `Gripper_zero_max` – fully open
- 1 × fan output (RAMPS **FAN_PIN**) – optional but supported
- On-board LED – used as a simple “alive/status” indicator

All pin assignments are centralized in:

> `firmware/pinout.h`

If you change wiring, you **only** need to update this file.

### 2.3 Mechanics (3D Printed Parts)

You will need a Tobler/Tobler-style 3D printed arm:

Typical parts include:

- Base and base ring
- Rotation gear(s)
- Lower arm, upper arm, lever
- Linkages (`Pleuel`, `Pleuel_bend`)
- Wrist / manipulator parts
- Gripper base and fingers

The firmware does **not** depend on the exact STL file names, only on how the motors and switches are wired.

---

## 3. Repo Structure

Suggested folder layout:

```text
.
├── firmware/
│   ├── robotArm_final_code.ino   # main Arduino sketch
│   ├── robotArm.h
│   ├── pinout.h                  # all pin definitions in one place
│   ├── robotGeometry.h / .cpp    # inverse kinematics (XYZ -> joint angles)
│   ├── interpolation.h / .cpp    # smooth motion planning
│   ├── RampsStepper.h / .cpp     # low-level stepper control
│   ├── command.h / .cpp          # parse G/M-style commands from serial
│   ├── queue.h                   # simple ring buffer for commands
│   ├── fanControl.h / .cpp       # helper for the fan output
│   └── (any extra helpers)
├── examples/
│   └── pc_control.py             # example Python script to control ToblerArm
├── docs/
│   ├── wiring.md                 # wiring diagrams and notes
│   ├── homing.md                 # details on homing behaviour
│   └── kinematics.md             # math notes (optional, for advanced users)
└── README.md

You can adjust file names if your project is already structured.

⸻

4. From Zero: Step-by-Step Build Guide

4.1 Tools You Need
	•	3D printer (or access to printed parts)
	•	Basic hand tools (screwdrivers, pliers, Allen keys, wire stripper)
	•	Soldering iron (optional but helpful)
	•	PC or laptop with:
	•	Arduino IDE
	•	Python (optional, for pc_control.py)

⸻

4.2 Step A – Print the Parts

General print suggestions:
	•	Material: PLA or PETG
	•	Layer height: 0.2 mm
	•	Walls: 3 perimeters
	•	Infill:
	•	30–40% for normal parts
	•	50–80% for gears, base and high-load parts

Make sure:
	•	Bearings fit snugly but not too tight.
	•	Gears mesh correctly (no obvious binding).
	•	Screw holes are not blocked (clear them with a drill bit if needed).

⸻

4.3 Step B – Mechanical Assembly (High Level)
	1.	Base & Rotation
	•	Install bearings and (if used) a thrust bearing in the base.
	•	Mount a NEMA17 motor in the base.
	•	Attach the large rotation gear and ensure it turns smoothly.
	2.	Lower & Upper Arm
	•	Install bearings into lower arm, upper arm, and joints.
	•	Mount NEMA17 motors for shoulder and elbow.
	•	Attach linkages (e.g. Pleuel, Pleuel_bend) to transmit motion.
	3.	Gripper
	•	Assemble gripper base and fingers.
	•	Mount the 28BYJ-48 motor and link the output to the gripper mechanism.
	4.	Stabilizers & Fine Tuning
	•	Add any stabilizer parts your design uses.
	•	Adjust washers and nut tension to remove play but keep joints freely moving.

At this stage, you should be able to move the arm by hand and feel all joints operating smoothly.

⸻

4.4 Step C – Wiring the Electronics
	1.	Plug RAMPS 1.4 onto Arduino Mega 2560.
	2.	Insert stepper drivers into X, Y, Z sockets.
	3.	Connect NEMA17 motors to:
	•	X motor → one joint
	•	Y motor → second joint
	•	Z motor → third joint
(Exactly which joint is up to you; just keep it consistent with the firmware config.)
	4.	Wire limit switches to X_MIN, Y_MIN, Z_MIN.
	5.	Wire the gripper stepper to the pins defined in pinout.h (either via ULN2003 or directly).
	6.	Wire gripper limit switches to Gripper_zero_min and Gripper_zero_max.
	7.	Connect a 12 V PSU to the RAMPS power input (check polarity carefully).
	8.	Set the stepper driver current using the small potentiometer on each driver.

⚠️ Double-check all wiring before powering up.
Reversed connections can damage your drivers or PSU.

⸻

4.5 Step D – Upload the Firmware
	1.	Open firmware/robotArm_final_code.ino in Arduino IDE.
	2.	Select:
	•	Board: Arduino Mega 2560
	•	Port: your Arduino’s COM/USB port
	3.	Make sure all .h / .cpp files are in the same project folder.
	4.	Click Upload.

If the upload is successful, the board will reset and start running the firmware.

⸻

5. How the Firmware Works (Concept → Implementation)

This section aims to explain the firmware from beginner to advanced concepts.

⸻

5.1 Concept: “I send a command, the arm moves”

From your PC, you send:

G1 X80 Y0 Z100 F80

This means:
	•	G1 → move in a straight line
	•	X80 Y0 Z100 → final XYZ position
	•	F80 → feedrate (speed setting)

Inside the firmware:
	1.	The Command parser reads the line via Serial.
	2.	A Cmd object is created with fields for X, Y, Z, F, etc.
	3.	The command is stored in a Queue (FIFO).
	4.	When the arm finishes the current move, the next command is executed.

⸻

5.2 Interpolation: Smooth Movement Between Two Points

Naive motion:

start → instantly full speed → instantly stop
(jerky, noisy, can skip steps)

ToblerArm motion:

start → accelerate → cruise → decelerate → stop

The Interpolation module:
	•	Stores the start point and the target point.
	•	Computes how long the move should take using the feedrate.
	•	Uses a cosine curve to compute “progress” from 0 to 1:

// inside interpolation:
float progress = -cos(t * tmul * PI) * 0.5 + 0.5;

At each cycle, it provides the current XYZ along that curve.

You can read the interpolated position using:

float x = interpolator.getXPosmm();
float y = interpolator.getYPosmm();
float z = interpolator.getZPosmm();


⸻

5.3 Inverse Kinematics: XYZ → Joint Angles

The arm is treated as a simple 3-link mechanism:
	•	Base rotation
	•	Lower arm
	•	Upper arm

RobotGeometry:
	1.	Takes XYZ from the interpolator.
	2.	Computes:
	•	Base rotation from atan2(y, x).
	•	Shoulder and elbow angles using trigonometry (sin, cos, acos, asin).
	3.	Returns joint angles in radians:

geometry.set(x_mm, y_mm, z_mm);
float rot  = geometry.getRotRad();
float low  = geometry.getLowRad();
float high = geometry.getHighRad();

You can adjust arm length and geometry constants in robotGeometry.cpp if your arm dimensions are different.

⸻

5.4 RampsStepper: Angles → Motor Steps

Each axis uses a RampsStepper object that knows:
	•	STEP, DIR, ENABLE pins
	•	Steps per revolution
	•	Gear ratio

You configure it once:

RampsStepper stepperRotate(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN);
// ...
stepperRotate.setReductionRatio(gearRatio, stepsPerRev);

To move to a given angle:

stepperRotate.stepToPositionRad(geometry.getRotRad());

Internally:

targetSteps = angleRad * radToStepFactor;

Then stepperRotate.update() generates step pulses until the motor reaches targetSteps.

This pattern repeats for all three axes.

⸻

5.5 Gripper Logic: Movement With Limits

The gripper uses:
	•	A 28BYJ-48 stepper motor.
	•	Two limit switches:
	•	Fully closed
	•	Fully open

Commands:
	•	M3 → close gripper
	•	M5 → open gripper

Firmware logic:
	•	Step the motor in the correct direction.
	•	Stop if:
	•	The relevant limit switch is pressed, OR
	•	A timeout (number of steps / time limit) is reached.

This protects the mechanism and makes it easier to calibrate.

⸻

6. Command Reference (Beginner-Friendly Cheat Sheet)

You send these commands over Serial (115200 baud).
Each line ends with \r\n (Arduino Serial Monitor: set “Both NL & CR”).

6.1 Coordinate Modes

G90   ; absolute mode (positions are global)
G91   ; relative mode (positions are offsets)

6.2 Motion

G0 X.. Y.. Z.. F..   ; rapid move (no strict path control)
G1 X.. Y.. Z.. F..   ; linear move with speed F
G4 Tn                ; dwell / wait n seconds
G28                  ; home all axes (uses limit switches)

Examples:

G90
G28                  ; home the robot
G1 X80 Y0 Z100 F80   ; move to (80, 0, 100)
G1 X120 Y20 Z90 F60  ; move to a new point

6.3 Steppers, Gripper, Fan

M17           ; enable all stepper drivers
M18           ; disable all stepper drivers

M3            ; close gripper (until "closed" switch)
M5            ; open gripper (until "open" switch)

M106          ; fan ON
M107          ; fan OFF

M114          ; report current XYZ position

Example mini program:

G90
G28
G1 X80 Y0 Z100 F80
M3
G1 Z140 F60
M5
M114


⸻

7. Example PC Script (Python)

Create examples/pc_control.py:

import time
import serial

# Change this to your actual port:
#   Windows: "COM3", "COM4", ...
#   Linux: "/dev/ttyACM0" or "/dev/ttyUSB0"
PORT = "COM3"
BAUD = 115200

def send(ser, line):
    line = line.strip()
    print(">>", line)
    ser.write((line + "\r\n").encode("ascii"))
    time.sleep(0.05)

def main():
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        # Give Arduino time to reset
        time.sleep(2)

        # Simple demo script for ToblerArm
        send(ser, "G28")                    # home
        send(ser, "G90")                    # absolute mode
        send(ser, "G1 X80 Y0 Z100 F80")     # move over object
        send(ser, "M3")                     # close gripper
        send(ser, "G1 Z140 F60")            # lift object
        send(ser, "G1 X0 Y0 Z120 F80")      # move to drop point
        send(ser, "M5")                     # open gripper
        send(ser, "M114")                   # ask for position

        # Print responses
        time.sleep(1)
        while ser.in_waiting:
            print("<<", ser.readline().decode(errors="ignore").strip())

if __name__ == "__main__":
    main()


⸻

8. Troubleshooting (Common Problems)

Motors just vibrate / buzz and don’t move
	•	Motor wires may be in the wrong order.
	•	Driver current might be too low or too high.
	•	Try a lower feedrate (F) in your G1 move.

Homing (G28) never finishes
	•	Check wiring of limit switches (normally open vs normally closed).
	•	Confirm the correct pins in pinout.h.
	•	Make sure the joint can physically reach the switch.

Gripper won’t stop moving
	•	Check gripper limit switch wiring.
	•	Confirm the correct pins for Gripper_zero_min / Gripper_zero_max.
	•	Inspect the mechanism so the switch is actually pressed at the end of travel.

Motion is reversed
	•	Swap the motor connector (reverse direction), or
	•	Invert direction logic / gear ratio sign in the axis setup code.

⸻

9. Customization & Advanced Notes
	•	Change arm length: edit constants in robotGeometry.cpp.
	•	Change gear ratios or microstepping: update the setReductionRatio() calls for each RampsStepper.
	•	Add new G/M commands: extend executeCommand(Cmd cmd) and the parser in command.cpp.
	•	Add tools: e.g., replace gripper with pen holder, suction cup, etc., then update motion and custom commands.

⸻

10. License & Credits

License

Choose a license that matches your goals (MIT is a popular, simple choice).
Example header for MIT:

MIT License

Copyright (c) 2025 N. K. K. Perera

Permission is hereby granted, free of charge, to any person obtaining a copy
...


Credits
	•	Original mechanical inspiration: Florin Tobler – RobotArm (2016)
	•	Firmware integration, structure, and documentation for ToblerArm:
N. K. K. Perera (@KANCHANAKALANAPERERA
)

If you build or modify ToblerArm, feel free to:
	•	open issues,
	•	submit pull requests,
	•	and share photos or videos of your robot in action.

Happy building! 🛠🤖

