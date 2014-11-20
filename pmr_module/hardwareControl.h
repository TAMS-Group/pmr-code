/*
    This file is part of PMR-Firmware.

    PMR-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PMR-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PMR-Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef HARDWARECONTROL_H
#define HARDWARECONTROL_H

    #include "configuration.h"
    #include <PWMServo.h>
    #include <EEPROM.h>

    class HardwareControl
    {
    public:
        HardwareControl(bool master);
        void tick();
        void engageServo(char message);
        void setServoPosition(int servoPosition);
        void setAngle(int angle);

        void setServoCalibration(char calibData);
        char restoreServoCalibration();
//        static void executeGCode(GCode *com);
//        static void waitUntilEndOfAllMoves();
 //       static void setFanSpeed(int speed,bool wait); /// Set fan speed 0..255
    private:
        bool master;
        int servoPosition;
        int calibrationOffset;

        PWMServo servo;

        void writeServoCalibDataToEeprom(char data);
 //       static int lowestRAMValue;
    };

#endif // HARDWARECONTROL_H