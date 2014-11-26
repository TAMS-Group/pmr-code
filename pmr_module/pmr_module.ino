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
#include <PWMServo.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>


//If softwareSerial pins are improperly connected, the bus reads constant 0. To avoid this,
//only the master module is allowed to have adress=0 and the connection gets not accepted for a slave with this adress.

//########sine generator parameters#########
float sine_amplitude = 37;
float sine_frequency = 0.5;
float sine_phase = 130;
float sine_offset = 0;
//##########################################

float locomotionAngles[15];
int sampling = 50;  //set one angle every x milliseconds
unsigned long lastSample = 0;

//internal variables
boolean verbose = false;

//tmp pointer for command parsing
uint8_t commP1;
uint8_t commP2;

//bus measuring
boolean busTest = false;
unsigned long busTestTime = 0;
unsigned long busTestLastSent = 0;
unsigned int busTestMessages = 0;
unsigned int busTestSent = 0;
unsigned int busTestRecv = 0;
unsigned int busTestFreq = 0;
byte busTestAdress;

byte modulesCount = 1;  //only used in master-mode. Initialized with 1 for master as first module.
byte pitchingJointsCount = 1;

//locomotion control variables
boolean locomote = false;
boolean forward = true;
Locomotion locomotion = WALK;
boolean locomotionGather = false;

String commandBuffer = "";

Topology topology[32];  //relative topology in direction from master to last slave
byte pitchingJoints[32]; //adresses of the pitching group

int maxAngle = 90;

byte _adress;
byte _type;
byte _message;
bool _orientation;


HardwareControl hc;
Communication com;

void setup()  
{
  pinMode(3, OUTPUT); 
  pinMode(4, OUTPUT);  
  pinMode(TOPO_PIN_P, INPUT);
  pinMode(TOPO_PIN_Y, INPUT);
  pinMode(TX_PIN, OUTPUT);

  if(MASTER) { 
    //initialize topology array with master adress
    topology[0].adress = ADRESS;
  }
  com.init();
}


void loop() {
  if(com.connect(&_adress, &_orientation)) {
    enqueModule(_adress, _orientation);
  }
  
  if(!com.heartBeat()) {
    //lost downstream connection, remove modules from list
    processCommand(ADRESS, TOPOLOGY, 2);
  }

  if(com.readUpstream(&_adress, &_type, &_message)) {
    processCommand(_adress, _type, _message);
  }

  // Communication between master and host is currently handled in the main object, since it invokes a lot of functions
  if(MASTER) {
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
    if(com.readDownstream(&_adress, &_type, &_message)) {
      processCommand(_adress, _type, _message);
    }
  }

  if(locomote && MASTER) {
    if(millis() > lastSample + sampling) {
      lastSample = millis();
      switch(locomotion) {
        case WALK: {
          moveSinusoidal(forward);
        } break; 
        case ROLL: {
          moveRoll(forward);
        } break;
        default: {
          moveSinusoidal(forward);
        }
        break;
      }
    }
  }
  //trigger servo clock   
  hc.tick();
}



// master: enques new module into topology arry
// slave:  sends detected module upstream
byte enqueModule(byte adress, boolean orientation) {  
  if(MASTER){
    Serial.println("Try to register new module with orientation: "+(String)orientation);
    if((adress>15) || (adress<1)) {
       Serial.println("ERROR: Invalid adress");
       return modulesCount;
    }
    if(searchDuplicateInTopology(adress)) {
      Serial.print("ERROR: Module ");
      Serial.print(adress);
      Serial.println(" is already registered");
      return modulesCount;
    }
    topology[modulesCount].adress = adress;
    topology[modulesCount].orientation = orientation ? topology[modulesCount-1].orientation : !topology[modulesCount-1].orientation;
    modulesCount++;
    countPitchingJoints();
    printTopology();
  } else{
     com.sendUpstream(adress, TOPOLOGY, (char)orientation); 
  }
  return modulesCount;
}


// returns true if adress is already registered
boolean searchDuplicateInTopology(byte adress){
  boolean foundAdress = false;
  for(int i=0; i<modulesCount; i++){
    if(topology[i].adress == adress){
      foundAdress = true;
      break;
    }
  }
  return foundAdress;
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
    printTopology();
    
  }else if(command.equalsIgnoreCase("start")) {
    locomote = true;
    
  }else if(command.equalsIgnoreCase("stop")) {
    locomote = false;   
    
  }else if(command.equalsIgnoreCase("locomotion")) {
    String type = nextParameter();
    if(type.equalsIgnoreCase("walk")){
      locomotion = WALK;
      Serial.println("Locomotion type: walk (standard)");
    }
    if(type.equalsIgnoreCase("roll")){
      locomotion = ROLL;
      Serial.println("Locomotion type: roll");
    }
    
  }else if(command.equalsIgnoreCase("cd")) {
    forward = forward ? false : true; 
    
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
    String parameter = nextParameter();
    char tmp[parameter.length()+1];
    parameter.toCharArray(tmp, parameter.length()+1);
    sine_frequency = atof(tmp);
             
  }else if(command.equalsIgnoreCase("setamplitude")) {    
    sine_amplitude = nextParameter().toInt();
       
  }else if(command.equalsIgnoreCase("setphase")) {    
    sine_phase = nextParameter().toInt();
 
  }else if(command.equalsIgnoreCase("setsampling")) {    
    sampling = nextParameter().toInt();
    
  }else if(command.equalsIgnoreCase("enablelocomotiongather")) {    
    locomotionGather = true;
    
  }else if(command.equalsIgnoreCase("disablelocomotiongather")) {    
    locomotionGather = false;    
    
  }else if(command.equalsIgnoreCase("bustest")) {    
    busTest = !busTest;
    if(busTest) {
      Serial.println("BusTest enabled");
      busTestTime = millis();
      busTestMessages = 0;
      busTestSent = 0;
      busTestRecv = 0;
      busTestFreq = nextParameter().toInt();
      busTestAdress = (byte) nextParameter().toInt();
      if(busTestFreq > 0) {
        Serial.print("sending ping message to ");
        Serial.print(busTestAdress);
        Serial.print(" every ");
        Serial.print(busTestFreq);
        Serial.println("ms");
      }
    }else{
      Serial.println("BusTest disabled");
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
          for(int i=modulesCount-1; i>0; i--) { // 1 is master and must not be deleted
            if(topology[i].adress != adress) {
              topology[i].adress = 255;
              topology[i].orientation = true;
              modulesCount = i;
            }else{
              break;
            }           
          }
          Serial.println("Modules disconnected, new topology: ");
          countPitchingJoints();
          printTopology();        
        }else{
          boolean topoBool = (message==0) ? false : true;
          enqueModule(adress, topoBool);
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
      case ANSWER_ANGLE: {
        // TODO: irgendwas mit dem Winkel machen! z.B. sammeln und per BT rausschicken
        //sendViaBluetooth(currentAngle);
      } break;
      case PING: {
        if(busTest && (busTestFreq > 0)) {
          busTestRecv++;
        }else{
          Serial.print("ping_reply from ");
          Serial.print(int(adress));
          Serial.print(" value: ");
          Serial.println(int(message));
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
      case ANSWER_ANGLE: {
        com.sendUpstream(adress, ANSWER_ANGLE, message);
      } break;
      case PING: {
        com.sendUpstream(adress, PING, message);
      } break;
      default:
      break;
    }
  }
}


void printTopology() {
  for(int i=0; i < modulesCount; i++) {
    Serial.print("adress: ");
    Serial.print(int(topology[i].adress));
    Serial.print(" orientation: ");
    topology[i].orientation ? Serial.println("pitch") : Serial.println("yaw");
  }
  Serial.print("number of pitching joints: ");
  Serial.println(pitchingJointsCount);
  Serial.println();
}



// converts an angle from degrees to pwm-value in microseconds
int degToPwm(float desiredDeg)
{
  int result;
  if(desiredDeg > maxAngle) {result = PWM_MAXIMUM;} 
  else if(desiredDeg < -maxAngle) {result = PWM_MINIMUM;} 
  else {
    result = (PWM_MAXIMUM-PWM_RANGE/2) + desiredDeg*(PWM_RANGE/(2*maxAngle));
  }  
  
  return result;  
}


// output is angle in degree
float sineFunction(float amp, float phase, float freq, float offset){
  float result = sin((millis()/(float)1000*2*PI)*freq+(phase/360*2*PI))*amp;
  result+=offset;
    
  return result;
}


// generates an array for one locomotion-step using a sinusoidal generator function for a specific number of joints
float* calculateNextJointPositions(int numOfJoints, bool forward){
  int dir = forward ? 1 : -1;
  for(int i=0; i<numOfJoints; i++){
    locomotionAngles[i] = sineFunction(sine_amplitude, sine_phase*(i*dir), sine_frequency, sine_offset);
  }      
  return locomotionAngles;
}


// generates sinusoidal locomotion in forward or backward direction
void moveSinusoidal(bool forward){ 
  float* angles = calculateNextJointPositions(pitchingJointsCount, forward); 
  
  byte currentPitchingJoint = 0;
  if(locomotionGather) {Serial.print("locomotion pattern:");}
  for(int i=0; i<modulesCount; i++){
    if(topology[i].orientation){  
      if(locomotionGather){        
        if(i>0) {Serial.print(" ");}
        if(i==(modulesCount-1)) {Serial.println(angles[i]);
        } else {Serial.print(angles[i]);}
      }        
      if(ADRESS == topology[i].adress){
        hc.setAngle(static_cast<int>(angles[i]));
      } else{
        com.sendDownstream(topology[i].adress, SET_ANGLE, (static_cast<byte>(angles[currentPitchingJoint]+128)));
      }
      currentPitchingJoint++;
    }
  }    
}

void moveRoll(bool forward){
  int dir = forward ? 1 : -1;
  short int rotate = 1;
  for(int i=0; i<modulesCount; i++){
    if(topology[i].orientation){
      if(ADRESS == topology[i].adress){
        hc.setServoPosition(static_cast<int>(sineFunction(sine_amplitude, 0, sine_frequency, sine_offset)));
      }else{
        com.sendDownstream(topology[i].adress, SET_ANGLE, (static_cast<byte>(rotate * sineFunction(sine_amplitude, 0, sine_frequency, sine_offset)+128)));
      }
    }else{
      com.sendDownstream(topology[i].adress, SET_ANGLE, (static_cast<byte>(rotate * sineFunction(sine_amplitude, sine_phase, sine_frequency, sine_offset)+128)));
      rotate = rotate*-1;
    }
  }
}


void countPitchingJoints(){
  pitchingJointsCount = 0;
  for(int i=0; i<modulesCount; i++){
    if(topology[i].orientation) pitchingJointsCount++;  
  }
} 