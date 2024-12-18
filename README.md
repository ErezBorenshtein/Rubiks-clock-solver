# Rubik's Clock Solver Robot

## Overview
The robot was meticulously designed and programmed to solve the Rubik's Clock puzzle in record time, showcasing precision engineering and advanced programming techniques.

## Features
- **Precision Mechanisms:** Powered by Nema 17 Bipolar 0.9° stepper motors for high accuracy.
- **Solving Algorithm:** Optimized algorithm for determining the quickest solution.
- **Hardware Components:**
  - Arduino Mega for robot control
  - Teensy 4.1 for LED display management
  - JF-0530B solenoids for precise movement
  - 4TB-8800TB motor driver
  - 32x64 LED display for status updates
- **Autonomous Operation:** Fully automated solving process from start to finish, not including clock's recognition.

## Hardware Setup
1. **Motors:** Connect the Nema 17 stepper motors to the motor driver.
2. **Solenoids:** Interface the JF-0530B solenoids with the Arduino Mega.
3. **Microcontrollers:**
   - Use the Arduino Mega as the main controller for the robot.
   - Use the Teensy 4.1 to control the 32x64 LED screen.
4. **LED Screen:** Mount the screen to display solving steps and timings.
5. **Power Supply:** Ensure stable power for all components.

### Dependencies
- [Arduino IDE](https://www.arduino.cc/en/software)
- [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html)

### Installation
1. Clone the repository:
   ```bash
   git clone https://https://github.com/ErezBorenshtein/Rubiks-clock-solver.git
   ```
2. Open the Arduino IDE and load the provided `.ino` files for the Arduino Mega and Teensy 4.1.
3. Install the required libraries via the Arduino Library Manager, and add the Teensy board from the Board Manager.
4. Upload the respective code to the Arduino Mega and Teensy 4.1.
5. Install the necessary libraries for the clock recognition system.

### Execution
1. Power on the robot.
2. Place the scrambled Rubik's Clock on the designated platform.
3. Press the start button to initiate the solving process.
4. Monitor the progress on the LED display.

## Challenges
- Designing precise mechanisms to manipulate the clock's dials and clocks.
- Synchronizing multiple components, including motors, solenoids, and the LED display.
- Creating full simulation of the clock using to test the algorithm. 
- Creating full 3D model of the robot to test the mechanism. 

## License
This project is open-sourced under the MIT License. Feel free to use, modify, and distribute as per the terms of the license.

## Acknowledgments
- The speedcubing community for inspiration.
- Open-source libraries and tools that made this project possible.
- Nyanyan-Maker for the inspiration and for suggesting motors and drivers. [youtube](https://www.youtube.com/watch?v=jc3e5xadDao)

## Contact
For questions or collaborations, reach out via [email](erez.borenshtein@gmail.com).