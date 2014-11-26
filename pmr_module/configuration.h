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

//###### IMPORTANT BASIC SETTINGS #################
#define MASTER true // Is this the master node?
#define ADRESS 0    // Adress of this node. 0 if master, 1..254 else
//If softwareSerial pins are improperly connected, the bus reads constant 0. To avoid this,
//only the master module is allowed to have adress=0 and the connection gets not accepted for a slave with this adress.
//###### IMPORTANT BASIC SETTINGS #################


//########## communication ########################
//baudrate for internal communication on the "bus"
#define BAUDRATE 19200 //57600//38400//28800//19200//14400//9600//2400
#define MASTER_BAUDRATE 57600 // faster baudrate for communication between master and hostcomputer
//########## communication ########################

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
#define LED_PIN 13
//########## Pin assignements #####################

//########## Heartbeat ############################
#define HEART_BEAT_INTERVAL 500.0
#define HEART_BEAT_TIMEOUT 3000.0
//########## Heartbeat ############################



#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "HardwareSerial.h"


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

struct Topology
{
    byte adress;
    bool orientation;
    Topology() {
        adress = 128;
        orientation = true;// true means the same orientation as the neighbour in upstream direction
    }
};

#endif // CONFIGURATION_H