#include "configuration.h"
#include "hardwareControl.h"
#include <PWMServo.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>


//If softwareSerial pins are improperly connected, the bus reads constant 0. To avoid this,
//only the master module is allowed to have adress=0 and the connection gets not accepted for a slave with this adress.
//############# Module parameters ##################
boolean master = true;
byte adress = 0;
//###############################################

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
int servoPin = 9;  
int topoPinP  = 7;
int topoPinY = 6;
int txPin = 8;

//tmp pointer for command parsing
uint8_t commP1;
uint8_t commP2;

//heartbeat
unsigned long lastUpBeat;
unsigned long lastDownBeat;
unsigned long lastUpBeatSent = 0;
unsigned long lastDownBeatSent = 0;
unsigned long heartBeatInterval = 500;//500;//1000;
unsigned long heartBeatTimeout = 3000;
unsigned long lastBlink = 0;
unsigned long lastSoftwareUartInit = 0;
int blinkStatus = LOW;

//bus measuring
boolean busTest = false;
unsigned long busTestTime = 0;
unsigned long busTestLastSent = 0;
unsigned int busTestMessages = 0;
unsigned int busTestSent = 0;
unsigned int busTestRecv = 0;
unsigned int busTestFreq = 0;
byte busTestAdress;

boolean uartConnected = false;
boolean softwareUartConnected = false;
ConnectionStatus connectionStatus;
int baudRate = 19200;//2400;//57600;//38400;//28800;//19200;//14400;//9600;//2400;
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

SoftwareSerial softSerial = SoftwareSerial(topoPinP, txPin);

PWMServo servo;
//int servoPos = 0;//degToPwm(0);
int maxAngle = 90;


HardwareControl hc(master=true);

void setup()  
{
  pinMode(3, OUTPUT); 
  pinMode(4, OUTPUT);  
  pinMode(topoPinP, INPUT);
  pinMode(topoPinY, INPUT);
  pinMode(txPin, OUTPUT);
  
  connectionStatus = DISCONNECTED;
  
  if(master) { 
    //initialize topology array with master adress
    topology[0].adress = adress;    
    //initialize UART-serial for external communication with high baudrate
    Serial.begin(57600);
    Serial.print("Master module, adress: ");
    Serial.print(int(adress));
    Serial.println(" - waiting for command");
    uartConnected = true;
    connectionStatus = HARDWARE;
  }else{
    digitalWrite(13, LOW);
    Serial.begin(baudRate);
  }
}


void loop() {
  visualiseConnectionStatus();
    
  //send my adress as init beacon until initial handshake is completed
  if(!uartConnected) {
    handShake();      
  }else{
    //heartbeat
    if(!master) {
      if((millis()-lastUpBeatSent) > heartBeatInterval) {
        sendUpstream(adress, HEARTBEAT, 0);
        lastUpBeatSent = millis();
        Serial.flush();
      }
        
      if((millis()-lastUpBeat) > heartBeatTimeout) {
        //seems disconnected
        uartConnected = false;
        connectionStatus = DISCONNECTED;
        sendDownstream(15, TOPOLOGY, 2);
      }          
    }
    //read and process data from master-side
    readDownstream();
  }
    
  if(!softwareUartConnected && uartConnected){
    initSoftwareUart();      
  }
    
  if(softwareUartConnected){
    if((millis()-lastDownBeatSent) > heartBeatInterval) {
      sendDownstream(adress, HEARTBEAT, 1);
      lastDownBeatSent = millis();
    }
    unsigned long mil = millis();
    unsigned long diff = millis()-lastDownBeat;
    if(diff > heartBeatTimeout) {
      if(master) {
        Serial.print("ERROR: downbeat timeout: ");
        Serial.println(""+(String)lastDownBeat+" - "+(String)mil+" = "+(String)diff);          
      }
        
      softSerial.flush();
      Serial.flush();
          
      //seems disconnected
      softwareUartConnected = false;
      if(uartConnected) {connectionStatus = HARDWARE;}
      else connectionStatus = DISCONNECTED;
      if(master) {
        processCommand(adress, TOPOLOGY, 2); 
      }else{
        sendUpstream(adress, TOPOLOGY, 2);
      }
    }
    // read and process data from slave-side
    readUpstream();
      
    //do bustest evaluation
    if(busTest) {
      if(busTestFreq > 0) {
        if((millis() - busTestLastSent) > busTestFreq) {
          sendDownstream(busTestAdress, PING, 1);
          busTestLastSent = millis();
          busTestSent++;
        }
      }
      if((millis() - busTestTime) > 1000) {
        Serial.print("busTest: ");
        Serial.print(busTestMessages);
        Serial.println("msg/s");
        if(busTestFreq > 0) {
          Serial.print("sent: ");
          Serial.println(busTestSent);
          Serial.print("recv: ");
          Serial.println(busTestRecv);
          Serial.print("lost: ");
          Serial.println(busTestSent - busTestRecv);
          Serial.println();
        }
        busTestTime = millis();
        busTestMessages = 0;
        busTestSent = 0;
        busTestRecv = 0;
      }
    }
  }
  if(locomote && master) {
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
  hc.tick();
}


// todo: when alternating the orientation of the module sometimes there is data on the old connected topoPin which leads to temporary wrong orientation determination
void handShake(){
  sendUpstream(adress, CONNECT, 0);
  delay(10);
  while(Serial.available() > 3) {
    if(Serial.read() != byte(254)) continue;
    if(Serial.peek() < byte(254)) { 
      byte recv = Serial.read();
      byte type = recv << 4;
      type = type >> 4;
      byte messageAdress = recv >> 4;
      if(Serial.peek() >= byte(254)) continue;
      byte message = Serial.read();
      if(Serial.read() != byte(255)) continue; //look for ending signature
     // else{Serial.read();}
      if((adress == messageAdress) && (message == 1)) { //my adress or broadcast
        uartConnected = true;
        lastUpBeat = millis();
        connectionStatus = HARDWARE;   
      }
    }
  } 
}


byte readAdress(){
  byte adress = 255;
  while(softSerial.available() > 3) {
    if(softSerial.read() != byte(254)) {continue;}
    if(softSerial.peek() < byte(254)) { 
      byte recv = softSerial.read();
      byte type = recv << 4;
      type = type >> 4;
      adress = recv >> 4;
      if(softSerial.peek() >= byte(254)) {continue;}
      byte message = softSerial.read();
      if(softSerial.read() != byte(255)) {continue;} //look for ending signature
    }
    softSerial.read(); 
  }  
  return adress; 
}


void initSoftwareUart()
{      
  int nextAdress = 256;
  /*
  if(master) {
    Serial.print("oldBufferSize: ");
    Serial.print(softSerial.available());
    Serial.println()
  }
  */
  //try Pitch  
  softSerial = SoftwareSerial(topoPinP, txPin);  
  softSerial.begin(baudRate);
  delay(1); //safety margin
  softSerial.flush();
  delay(2);
  //try to read byte:   
  nextAdress = readAdress();
//  if(master) Serial.println("try pitch");
  if(nextAdress < 254){    
    enqueModule(byte(nextAdress), true);
    sendDownstream(nextAdress, CONNECT, 1);
    softwareUartConnected = true;
    connectionStatus = SOFTWARE;
    lastDownBeat = millis();        
  } else{
    softSerial = SoftwareSerial(topoPinY, txPin);   
    softSerial.begin(baudRate);
    delay(1);
    softSerial.flush();
    delay(2);
    nextAdress = readAdress();
  //  if(master) Serial.println("try yaw");
    if(nextAdress < 254){      
      enqueModule(byte(nextAdress), false);
      sendDownstream(nextAdress, CONNECT, 1);
      softwareUartConnected = true;
      connectionStatus = SOFTWARE;
      lastDownBeat = millis();
    }
  }     
}


// master: enques new module into topology arry
// slave:  sends detected module upstream
byte enqueModule(byte adress, boolean orientation) {  
  if(master){
    Serial.println("Try to register new module with orientation: "+(String)orientation);
    if((adress>15) || (adress<1)) {
       Serial.println("ERROR: Invalid adress");
       softSerial.flush();
       return modulesCount;
    }
    if(searchDuplicateInTopology(adress)) {
      Serial.print("ERROR: Module ");
      Serial.print(adress);
      Serial.println(" is already registered");
      softSerial.flush();
      return modulesCount;
    }
    softSerial.flush();
    topology[modulesCount].adress = adress;
    topology[modulesCount].orientation = orientation ? topology[modulesCount-1].orientation : !topology[modulesCount-1].orientation;
    modulesCount++;
    countPitchingJoints();
    printTopology();
    /*
    if(verbose) {
      Serial.print("Registered new module: ");
      Serial.print(int(adress));
      String pitchyaw = orientation ? "pitch" : "yaw";
      Serial.print(" orientation: ");
      Serial.println(pitchyaw);
    }
    */
  } else{
     sendUpstream(adress, TOPOLOGY, (byte)orientation); 
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


void sendUpstream(byte adress, byte type, byte message) 
{
  byte command = (adress << 4) | type;
  Serial.write((uint8_t)254);
  Serial.write(command);
  Serial.write(message);
  Serial.write((uint8_t)255);  
}


void sendDownstream(byte adress, byte type, byte message) 
{
  if(busTest) busTestMessages++;
  byte command = (adress << 4) | type;
  softSerial.write((uint8_t)254);  
  softSerial.write(command);
  softSerial.write(message);
  softSerial.write((uint8_t)255);  
}

boolean readUpstream() {
  boolean result = false;
  if(softSerial.available()){
    while(softSerial.available() > 3) {
      if(softSerial.read() != byte(254)) {continue;}
      if(softSerial.peek() < byte(254)) { 
        //Serial.println("reading soft");
        byte recv = softSerial.read();
        byte type = recv << 4;
        type = type >> 4;
        byte adress = recv >> 4;
        if(softSerial.peek() >= byte(254)) {continue;}
        byte message = softSerial.read();
        if(softSerial.read() != byte(255)) {continue;} //look for ending signature
        //lastDownBeat = millis();
        if(master) {
          //Serial.println("adress: "+(String)adress+" type: "+(String)type+" message: "+(String)message);
          processCommand(adress, type, message);
          if(busTest) busTestMessages++;
        }else{
          if((type == HEARTBEAT)){// || (type == CONNECT)) {
            processCommand(adress, type, message);
          }else{
            Serial.write((uint8_t)254);
            Serial.write(recv);
            Serial.write(message);
            Serial.write((uint8_t)255);
          }
        }
        result = true;
      }
    }
  }
  
  return result;
}


boolean readDownstream() {
  boolean result = false;  
  if(master){
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
    while(Serial.available() > 3) {
      if(Serial.read() != byte(254)) {continue;}
      if(Serial.peek() < byte(254)) { 
        byte recv = Serial.read();
        byte type = recv << 4;
        type = type >> 4;
        byte messageAdress = recv >> 4;
        if(Serial.peek() >= byte(254)) {continue;}
        byte message = Serial.read();
        if(Serial.read() != byte(255)) {continue;} //look for ending signature
        //lastUpBeat = millis();
        if((adress == messageAdress) || (messageAdress == 15)) { //my adress or broadcast
          processCommand(messageAdress, type, message);
          sendDownstream(messageAdress, type, message);                    
        } else if(type == HEARTBEAT){
          processCommand(messageAdress, type, message);  
        } else{
          sendDownstream(messageAdress, type, message);          
        }
      result = true;
      }
    }
  }
  return result;
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
    if(adress == commandAdress) {
      processCommand(commandAdress, SET_ANGLE, angle);
    }else if(commandAdress == 15) {
      processCommand(commandAdress, SET_ANGLE, angle);
      sendDownstream(commandAdress, SET_ANGLE, angle);
    }else{
      sendDownstream(commandAdress, SET_ANGLE, angle);
    }
    
  }else if(command.equalsIgnoreCase("ping")) {
    byte commandAdress = (byte) nextParameter().toInt();
    byte value = (byte) nextParameter().toInt();
    Serial.print("Sent ping_request to ");
    Serial.print(int(commandAdress));
    Serial.print(" value: ");
    Serial.println(int(value));
    sendDownstream(commandAdress, PING, value);
    
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
    if(adress == commandAdress) {
      processCommand(commandAdress, ENGAGE, state);
    }else if(commandAdress == 15) {
      processCommand(commandAdress, ENGAGE, state);
      sendDownstream(commandAdress, ENGAGE, state); 
    }else{
      sendDownstream(commandAdress, ENGAGE, state); 
    }
    
  }else if(command.equalsIgnoreCase("setcalibration")) {
    byte commandAdress = (byte) nextParameter().toInt();
    byte calibData = (byte) nextParameter().toInt();   
    if(adress == commandAdress) {
      processCommand(commandAdress, CALIB_SERVO, calibData);
    }else if(commandAdress == 15) {
      processCommand(commandAdress, CALIB_SERVO, calibData);
      sendDownstream(commandAdress, CALIB_SERVO, calibData);
    }else{
      sendDownstream(commandAdress, CALIB_SERVO, calibData);
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
  if(master) {
    if(verbose && false) {
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
        //Serial.print("old heartbeat at: ");
        //Serial.println(lastDownBeat);
        if(message == 0) lastDownBeat = millis();
        //Serial.print("new heartbeat at: ");
        //Serial.println(lastDownBeat);        
        if(softSerial.overflow()) Serial.println("overflow");
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
          sendDownstream(adress, type, message);
          uartConnected = false;
          softwareUartConnected = false;
          connectionStatus = DISCONNECTED;
        }else{
          sendUpstream(adress, TOPOLOGY, message);
        }
      }
      break;
      case HEARTBEAT: {
        if(message == 0) lastDownBeat = millis();
        if(message == 1) lastUpBeat = millis();
      } break;
      case MESSAGE: {
        sendUpstream(adress, MESSAGE, message);
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
        sendUpstream(adress, ANSWER_ANGLE, message);
      } break;
      case PING: {
        sendUpstream(adress, PING, message);
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
      if(adress == topology[i].adress){
        hc.setServoPosition(static_cast<int>(angles[i]));
      } else{
        sendDownstream(topology[i].adress, SET_ANGLE, (static_cast<byte>(angles[currentPitchingJoint]+128)));
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
      if(adress == topology[i].adress){
        hc.setServoPosition(static_cast<int>(sineFunction(sine_amplitude, 0, sine_frequency, sine_offset)));
      }else{
        sendDownstream(topology[i].adress, SET_ANGLE, (static_cast<byte>(rotate * sineFunction(sine_amplitude, 0, sine_frequency, sine_offset)+128)));
      }
    }else{
      sendDownstream(topology[i].adress, SET_ANGLE, (static_cast<byte>(rotate * sineFunction(sine_amplitude, sine_phase, sine_frequency, sine_offset)+128)));
      rotate = rotate*-1;
    }
  }
}




void visualiseConnectionStatus(){
  switch(connectionStatus){
    case DISCONNECTED : digitalWrite(13, LOW); break;
    case HARDWARE : {
      if(millis()-lastBlink > 30) {
        blinkStatus = (blinkStatus == HIGH) ? LOW : HIGH;
        digitalWrite(13, blinkStatus);
        lastBlink = millis();
      }      
    } break;
    case SOFTWARE : digitalWrite(13, HIGH); break;
  } 
}


void countPitchingJoints(){
  pitchingJointsCount = 0;
  for(int i=0; i<modulesCount; i++){
    if(topology[i].orientation) pitchingJointsCount++;  
  }
} 