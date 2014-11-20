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

#ifndef CONFIGURATION_H
#define CONFIGURATION_H


//########## Pin configuration ####################
#define SERVO_PIN 9

//########### sine generator parameters ###########
#define PI 3.141
//########### sine generator parameters ###########

//########## Servo PWM parameters #################
#define PWM_MINIMUM 500 //400;
#define PWM_MAXIMUM 2100 //2600;
#define PWM_RANGE PWM_MAXIMUM - PWM_MINIMUM
//########## Servo PWM parameters #################

//########## Pin assignements #####################
#define SERVO_PIN 9
#define TOPO_PIN_P 7
#define TOPO_PIN_Y 6
#define TX_PIN 8
//########## Pin assignements #####################


enum CommandType
{
    TOPOLOGY = 0,//ok
    HEARTBEAT = 1,//ok
    SET_ANGLE = 2,//ok
    GET_ANGLE = 3,
    ENGAGE = 4,//ok
    MESSAGE = 5,//ok
    CALIB_SERVO = 6,
    ANSWER_ANGLE = 7,  //maybe message is enough...
    PING = 8,//ok
    CONNECT = 9//ok
};

enum ConnectionStatus{
    DISCONNECTED,
    HARDWARE,
    SOFTWARE
};

enum Locomotion
{
    WALK = 0,
    ROLL = 1
};

struct Topology
{
    char adress;
    bool orientation;
    Topology() {
        adress = 128;
        orientation = true;// true means the same orientation as the neighbour in upstream direction
    }
};

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "HardwareSerial.h"

#endif // CONFIGURATION_H