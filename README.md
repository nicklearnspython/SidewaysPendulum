# SidewaysPendulum
Balance a Pendulum at 90 using a propeller and motor

## Components
1. Brushless Motor
2. Propeller
3. ESC
4. Arduino Nano
5. IMU

## Calibration Instructions
Calibration Usage, according to documentation(https://www.firediy.fr/files/drone/HW-01-V4.pdf) : 
1. Plug your Arduino to your computer with USB cable, open terminal, then type 1 to send max throttle to every ESC to enter programming mode
2. Power up your ESCs. You must hear "beep1 beep2 beep3" tones meaning the power supply is OK
3. After 2sec, "beep beep" tone emits, meaning the throttle highest point has been correctly confirmed
4. Type 0 to send min throttle
5. Several "beep" tones emits, which means the quantity of the lithium battery cells (3 beeps for a 3 cells LiPo)
6. A long beep tone emits meaning the throttle lowest point has been correctly confirmed
7. Type 2 to launch test function. This will send min to max throttle to ESCs to test them
@author lobodol <grobodol@gmail.com>