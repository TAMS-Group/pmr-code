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


#include "configuration.h"
#include "hardwareControl.h"
#include "communication.h"
#include "pmrTopology.h"
#include "locomotion.h"
#include "busTest.h"
#include <PWMServo.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>


boolean verbose = false;

//tmp pointer for command parsing
uint8_t commP1;
uint8_t commP2;
String commandBuffer = "";

// variables used in loop()
byte _adress;
byte _type;
byte _message;
bool _orientation;

//objects
HardwareControl hc;
Communication com;
PMRTopology topo;
Locomotion locomotion(&com, &hc, &topo);
BusTest bustest(&com);

void setup()  
{
  pinMode(TOPO_PIN_P, INPUT);
  pinMode(TOPO_PIN_Y, INPUT);
  pinMode(TX_PIN, OUTPUT);

  com.init();
}


void loop() {
  // tick handshaking
  if(com.connect(&_adress, &_orientation)) {
    if(MASTER) {
      topo.enqueModule(_adress, _orientation);
    }else{
      com.sendUpstream(_adress, TOPOLOGY, (byte)_orientation);
    }
  }
  
  //tick heartbeat
  if(!com.heartBeat()) {
    //lost downstream connection, remove modules from list
    processCommand(ADRESS, TOPOLOGY, 2);
  }

  //tick readUpstream. This is not encapsulated because the return value is processed in main
  if(com.readUpstream(&_adress, &_type, &_message)) {
    processCommand(_adress, _type, _message);
  }

  //only if we are the master node
  if(MASTER) {
    // Communication between master and host is currently handled in the main object, since it invokes a lot of functions
    while(Serial.available() > 0) {
      char recv = Serial.read();
      if((recv == '\n') ||  (recv == '\r')){
        parseCommand();
        commandBuffer = "";
        break;
      }else{
        commandBuffer += recv;
      }
    }
  }else{
    //tick readDownstream. Unnecessary if we are master, in this case downstream would point to the host.
    if(com.readDownstream(&_adress, &_type, &_message)) {
      processCommand(_adress, _type, _message);
    }
  }

  //trigger locomotion module
  locomotion.tick();

  //trigger servo clock   
  hc.tick();

  //trigger bustest clock
  bustest.tick();
}



void parseCommand() {
  //extract the actual command
  commP1 = 0;
  commP2 = 0;
  String command = nextParameter();
  
  if(command.equalsIgnoreCase("setangle")) {
    byte commandAdress = (byte) nextParameter().toInt();
    //add 128 because we internally use one byte for angle, where 128 is 0, smaller values are negative angles
    byte angle = (byte) nextParameter().toInt()+128;    
    if(commandAdress == ADRESS) {
      processCommand(commandAdress, SET_ANGLE, angle);
    }else if(commandAdress == 15) {
      processCommand(commandAdress, SET_ANGLE, angle);
      com.sendDownstream(commandAdress, SET_ANGLE, angle);
    }else{
      com.sendDownstream(commandAdress, SET_ANGLE, angle);
    }
    
  }else if(command.equalsIgnoreCase("ping")) {
    byte commandAdress = (byte) nextParameter().toInt();
    byte value = (byte) nextParameter().toInt();
    Serial.print("Sent ping_request to ");
    Serial.print(int(commandAdress));
    Serial.print(" value: ");
    Serial.println(int(value));
    com.sendDownstream(commandAdress, PING, value);
    
  }else if(command.equalsIgnoreCase("topology")) {
    topo.printTopology();
    
  }else if(command.equalsIgnoreCase("start")) {
    locomotion.start();
    
  }else if(command.equalsIgnoreCase("stop")) {
    locomotion.stop();

  }else if(command.equalsIgnoreCase("locomotion")) {
    String type = nextParameter();
    if(type.equalsIgnoreCase("walk")){
      locomotion.walk();
    }
    if(type.equalsIgnoreCase("roll")){
      locomotion.roll();
    }
    
  }else if(command.equalsIgnoreCase("cd")) {
    locomotion.changeDirection();

  }else if(command.equalsIgnoreCase("oscillate")) {
    byte commandAdress = (byte) nextParameter().toInt();
    byte value = (byte) nextParameter().toInt();
    if(commandAdress==ADRESS || commandAdress==15) {
      if(value == 1) {
        locomotion.startOscillation();
      }else{
        locomotion.stopOscillation();
      }
      if(commandAdress==15) {
        com.sendDownstream(commandAdress, OSCILLATE, value);  
      }
    }else{
      com.sendDownstream(commandAdress, OSCILLATE, value);
    }
    
  }else if(command.equalsIgnoreCase("engage")) {
    byte commandAdress = (byte) nextParameter().toInt();
    byte state = (byte) nextParameter().toInt();
    if(commandAdress == ADRESS) {
      processCommand(commandAdress, ENGAGE, state);
    }else if(commandAdress == 15) {
      processCommand(commandAdress, ENGAGE, state);
      com.sendDownstream(commandAdress, ENGAGE, state); 
    }else{
      com.sendDownstream(commandAdress, ENGAGE, state); 
    }
    
  }else if(command.equalsIgnoreCase("setcalibration")) {
    byte commandAdress = (byte) nextParameter().toInt();
    byte calibData = (byte) nextParameter().toInt();   
    if(commandAdress == ADRESS) {
      processCommand(commandAdress, CALIB_SERVO, calibData);
    }else if(commandAdress == 15) {
      processCommand(commandAdress, CALIB_SERVO, calibData);
      com.sendDownstream(commandAdress, CALIB_SERVO, calibData);
    }else{
      com.sendDownstream(commandAdress, CALIB_SERVO, calibData);
    }    

  }else if(command.equalsIgnoreCase("setfrequency")) {  
    byte commandAdress = (byte) nextParameter().toInt();  
    String parameter = nextParameter();
    char tmp[parameter.length()+1];
    parameter.toCharArray(tmp, parameter.length()+1);
    if(commandAdress==ADRESS || commandAdress==15) {
      locomotion.setFrequency(atof(tmp));
      if(commandAdress==15) {
        com.sendDownstream(commandAdress, SET_FREQUENCY, (byte)(10/atof(tmp)));
      }
    }else{
      // We have a problem with the message concept at this point. Float values are not immediately possible.
      // Therefore, we send 10/freq and interpret this value at the recieve-side as freq=10/value.
      com.sendDownstream(commandAdress, SET_FREQUENCY, (byte)(10/atof(tmp)));
    }
             
  }else if(command.equalsIgnoreCase("setamplitude")) {    
    locomotion.setAmplitude(nextParameter().toInt());
       
  }else if(command.equalsIgnoreCase("setphase")) {    
    byte commandAdress = (byte) nextParameter().toInt();  
    int parameter = nextParameter().toInt();
    if(commandAdress==ADRESS || commandAdress==15) {
      if(commandAdress==15) {
        com.sendDownstream(commandAdress, SET_PHASE, (byte)(parameter/2));
      }
      locomotion.setPhase(parameter);
    }else{
      // We have a problem with the message concept at this point. 2^8=256, but we need 360.
      // Therefore, we send phase/2 and interpret this value at the recieve-side as phase = value*2.
      com.sendDownstream(commandAdress, SET_PHASE, (byte)(parameter/2));
    }

  }else if(command.equalsIgnoreCase("clockreset")) {    
    byte commandAdress = (byte) nextParameter().toInt();  
    Serial.print("Command adress: ");
    Serial.println(int(commandAdress));
    if(commandAdress==ADRESS || commandAdress==15) {
      locomotion.resetClockCounter();
      if(commandAdress==15) {
        Serial.println("sent clockreset broadcast");
        com.sendDownstream(commandAdress, OSCILLATOR_CLOCK_RESET, 0);
      }
    }else{
      com.sendDownstream(commandAdress, OSCILLATOR_CLOCK_RESET, 0);
    }
 
  }else if(command.equalsIgnoreCase("setsampling")) {    
    locomotion.setSampling(nextParameter().toInt());
    
  }else if(command.equalsIgnoreCase("enablelocomotiongather")) {    
    locomotion.gatherAngles(true);
    
  }else if(command.equalsIgnoreCase("disablelocomotiongather")) {    
    locomotion.gatherAngles(false);
    
  }else if(command.equalsIgnoreCase("bustest")) {    
    if(bustest.isActive()) {
      bustest.stop();
    }else{
      int freq = nextParameter().toInt();
      byte adress = nextParameter().toInt();
      bustest.start(freq, adress);
    }

  }else if(command.equalsIgnoreCase("verbose")) {
    verbose = !verbose;
    if(verbose) {
      Serial.println("Activated verbose information output");
    }else{
      Serial.println("Stopped verbose information output");
    }
                                
  }else{
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}


String nextParameter() {
  commP1 = commP2;
  commP2 = commandBuffer.indexOf(' ', commP1+1);
  String result = commandBuffer.substring(commP1, commP2);
  result.trim();
  return result;
}


void processCommand(byte adress, byte type, byte message) 
{
  if(MASTER) {
    if(verbose) {
      Serial.print("adress: ");
      Serial.print(int(adress));
      Serial.print(" type: ");
      Serial.print(int(type));
      Serial.print(" message: ");
      Serial.println(int(message));
    }
    switch((int)type) {
      case TOPOLOGY: {
        if(message == 2) {
          //remove all modules beyond sender of disconnect message
          //topology should be automatically redetected as soon as new modules are connected to the open end
          topo.removeModules(adress);
          Serial.println("Modules disconnected, new topology: ");
          topo.printTopology();
        }else{
          boolean topoBool = (message==0) ? false : true;
          topo.enqueModule(adress, topoBool);
        }
      } break; 
      case HEARTBEAT: {
        if(message == 0) com.setDownBeat();
      } break;
      case MESSAGE: {
         Serial.print("Message from ");
         Serial.print(int(adress));
         Serial.print(": ");
         Serial.print(int(message));
         Serial.println();
      } break;
      case CALIB_SERVO: {
        hc.setServoCalibration(message);
      } break;
      case ENGAGE: {
         hc.engageServo(message);
      }
      break;
      case SET_ANGLE: {
         hc.setAngle((int(message))-128);        
      } break;
      case GET_ANGLE: {
      //   int currentAngle = servo.read();
         //TODO
         //sendViaBluetooth(currentAngle);
      } break;
      case PING: {
        if(bustest.isActive()) {
          bustest.ping();
        }else{
          Serial.print("ping_reply from ");
          Serial.print(int(adress));
          Serial.print(" value: ");
          Serial.println(int(message));
        }
      } break;
      case OSCILLATE: {
        if(message == 1) {
          locomotion.startOscillation();
        }else{
          locomotion.stopOscillation();
        }
      } break;
      case CONNECT: {
        Serial.print("module ");
        Serial.print(int(adress));
        Serial.print(" tries handshake");  
        Serial.println();      
      } break;
      default: {
        if(verbose) {
          Serial.print("Recieved unknown command: ");
          Serial.print("adress: ");
          Serial.println(int(adress));
          Serial.print("type: ");          
          Serial.println(int(type));
          Serial.print("message: ");
          Serial.println(int(message));
          Serial.println();
        }
      } break;
    } 
  }else{
    switch((int)type) {
      case TOPOLOGY: {
        if(message == 2) {//disconnect from upstream, reset connections
          com.sendDownstream(adress, type, message);
          com.disconnect();
        }else{
          com.sendUpstream(adress, TOPOLOGY, message);
        }
      }
      break;
      case HEARTBEAT: {
        if(message == 0) com.setDownBeat();
        if(message == 1) com.setUpBeat();
      } break;
      case MESSAGE: {
        com.sendUpstream(adress, MESSAGE, message);
      } break;
      case CALIB_SERVO: {
        hc.setServoCalibration(message);
      } break;
      case ENGAGE: {
         hc.engageServo(message);
      } break;
      case SET_ANGLE: {
         hc.setAngle(int(message)-128);
      } break;
      case GET_ANGLE: {
      //   int currentAngle = servo.read();
         //TODO
         //sendUpstream(adress, ANSWER_ANGLE, static_cast<int>(currentAngle+90));
      } break;
      case PING: {
        com.sendUpstream(adress, PING, message);
      } break;
      case OSCILLATE: {
        if(message == 1) {
          locomotion.startOscillation();
          }else{
            locomotion.stopOscillation();
          }
      } break;
      case SET_FREQUENCY: {
        locomotion.setFrequency(10.0/message);
      } break;
      case SET_AMPLITUDE: {
        locomotion.setAmplitude(message);
      } break;
      case SET_PHASE: {
        locomotion.setPhase(message*2.0);
      } break;
      case OSCILLATOR_CLOCK_RESET: {
        locomotion.resetClockCounter();
      } break;
      default:
      break;
    }
  }
}