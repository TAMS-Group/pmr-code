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

#include "hardwareControl.h"

HardwareControl::HardwareControl()
{
    servoPosition = 0;
    int calibrationOffset = 0;

    //restore the servo calibration offset from EEPROM
    // attention if it has never been written, then we don't know what will be read!!
    restoreServoCalibration();
    if((calibrationOffset > 30) || (calibrationOffset < -30)) {
        if(MASTER) Serial.println("ERROR: calibration offset is to big/small...set calibration offset to '0' ");
        calibrationOffset = 0; 
    }
}

/* Must be called periodically with high frequence to hold the servos position.
   Otherwise the servo would drive to the desired position and then release the torque
*/
void HardwareControl::tick()
{
    servo.write(servoPosition+90);
}

void HardwareControl::engageServo(char message)
{  
  bool servoOn = (message>0);  
  if(servoOn) {if(!servo.attached()) servo.attach(SERVO_PIN, PWM_MINIMUM, PWM_MAXIMUM);
  }else{ 
    if(servo.attached()) servo.detach();
  }  
}

void HardwareControl::setServoPosition(int servoPosition)
{
    servoPosition = servoPosition;
}

// sets the servo to a value given by a message read from the bus
void HardwareControl::setAngle(int angle)
{  
  engageServo(1);
  setServoPosition(angle+calibrationOffset);
  
 /* if(master) {
    Serial.print("next angle: ");
    Serial.print(angle);
    Serial.print("-->");
    Serial.println(servoPos);
  }*/
}

//TODO: testing the following three functions
void HardwareControl::setServoCalibration(char calibData){
  calibrationOffset = static_cast<int>(calibData);
  writeServoCalibDataToEeprom(calibData);  
}

//loads the calibration data for the null position of the servo from eeprom
char HardwareControl::restoreServoCalibration(){
  calibrationOffset =  static_cast<int>(EEPROM.read(0));  
  return calibrationOffset;
}

//writes the calibration data for the null position of the servo to eeprom
void HardwareControl::writeServoCalibDataToEeprom(char data){
  EEPROM.write(0, data);
}