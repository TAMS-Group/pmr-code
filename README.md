# Printable Modular Robot (PMR) operation system

## Installation
Copy the PWMServo folder to your Arduino libraries folder in your sketchbook to make it available in the Arduino IDE.
For each module, set at least the Module Parameters befor flashing the board:
- master = true; Determines whether this module is master or not.
- adress = 0..255; This modules adress, 0 for the master, a unique number for each other module.
