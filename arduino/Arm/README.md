# Servo Arm Control

This PlatformIO project drives a simple four-servo manipulator arm via an Adafruit PCA9685 board.

## Hardware
- **Arduino Uno** or compatible
- **Adafruit PCA9685** PWM driver (I2C)
- Servos on channels:
  - **0** – Base rotation
  - **4** – Right arm joint
  - **11** – Left arm joint
  - **15** – Gripper

Wire the PCA9685's SDA and SCL lines to the Uno's A4 and A5 pins and provide a suitable power supply for the servos.

## Build and Upload
1. Install [PlatformIO](https://platformio.org/) on your system or within VS Code.
2. From the repository root run:
   ```bash
   cd arduino/Arm
   platformio run       # compile
   platformio upload    # flash to the board
   ```
   The board type and dependencies are defined in `platformio.ini`.

## Serial Command Interface
After uploading, open a serial monitor at **9600 baud**. Commands are sent as
text lines and immediately control the servos:

```
B:<pwm>  set base servo PWM (150–550)
R:<pwm>  set right arm servo PWM
L:<pwm>  set left arm servo PWM
G:<0|1>  open (0) or close (1) the gripper
```

Example:
```
B:500
G:1
```

The safe PWM ranges are defined in [`src/main.cpp`](src/main.cpp).

## Startup Behaviour
On reset the sketch executes a short sequence:
1. Move to a grab pose with the gripper open.
2. Close the gripper.
3. Lift slightly.

You can modify this sequence in `setup()`.

For quick manual testing on a PC, see `../servo_gui.py`, which sends these
commands via a simple Tkinter interface.
