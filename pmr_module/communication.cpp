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

#include "communication.h"

Communication::Communication()
: softSerial(TOPO_PIN_P, TX_PIN)
{
    connectionStatus = DISCONNECTED;
    softwareUartConnected = false;
    uartConnected = false;
    lastUpBeat = lastDownBeat = lastUpBeatSent = lastDownBeatSent = 0;
    lastBlink = 0;
    blinkStatus = LOW;
}

void Communication::init()
{
    if(MASTER) {   
        //initialize UART-serial for external communication with high baudrate
        Serial.begin(MASTER_BAUDRATE);
        Serial.print("Master module, adress: ");
        Serial.print(int(ADRESS));
        Serial.println(" - waiting for command");
        uartConnected = true;
        connectionStatus = HARDWARE;
    }else{
        digitalWrite(13, LOW);
        //initialize hardware serial for upstream communication (messages to the master)
        Serial.begin(BAUDRATE);
    }
}


/* This method validates the connection by sending and recieving heartbeats
   and must be called periodically from the main loop
   returns false if downstream connection is lost (for topology update if this node is master), true else.
*/
bool Communication::heartBeat()
{
    bool result = true;

    visualizeConnectionStatus();

    //Upstream interface
    if(!uartConnected) {
        // send my adress over the hardware serial towards the potential master as init beacon until initial handshake is completed
        handShake();      
    }else{ // actual heartbeat
        if(!MASTER) {
            // normal heartbeat
            if((millis()-lastUpBeatSent) > HEART_BEAT_INTERVAL) {
                sendUpstream(ADRESS, HEARTBEAT, 0);
                lastUpBeatSent = millis();
                Serial.flush();
            }
        
            // too much time since last hearbeat -> disconnected
            if((millis()-lastUpBeat) > HEART_BEAT_TIMEOUT) {
                uartConnected = false;
                connectionStatus = DISCONNECTED;
                sendDownstream(15, TOPOLOGY, 2);  //report the change in topology to master
            }          
        }
    }
    
    //Downstream interface
    if(!softwareUartConnected && uartConnected){
        // listen at the downstream interface only, if upstream connection (to the master) is already established
        initSoftwareUart();
    }
    
    // actual heartbeat
    if(softwareUartConnected){
        if((millis()-lastDownBeatSent) > HEART_BEAT_INTERVAL) {
            sendDownstream(ADRESS, HEARTBEAT, 1);
            lastDownBeatSent = millis();
        }

        // too much time since last hearbeat -> disconnected
        if(millis()-lastDownBeat > HEART_BEAT_TIMEOUT) {        
            softSerial.flush();
            Serial.flush();
          
            softwareUartConnected = false;
            if(uartConnected) {connectionStatus = HARDWARE;}
            else connectionStatus = DISCONNECTED;
            if(MASTER) {
                result = false;
            }else{
                sendUpstream(ADRESS, TOPOLOGY, 2);
            }
        }
    }
    return result;
}


void Communication::sendUpstream(char adress, char type, char message)
{
    char command = (adress << 4) | type;
    Serial.write((uint8_t)254);
    Serial.write(command);
    Serial.write(message);
    Serial.write((uint8_t)255); 
}

void Communication::sendDownstream(char adress, char type, char message) 
{
  char command = (adress << 4) | type;
  softSerial.write((uint8_t)254);  
  softSerial.write(command);
  softSerial.write(message);
  softSerial.write((uint8_t)255);  
}


bool Communication::readUpstream(char* adress, char* type, char* message) {
    bool result = false;
    if(softwareUartConnected){
        if(softSerial.available()){
            while(softSerial.available() > 3) {
                if(softSerial.read() != char(254)) {continue;}
                if(softSerial.peek() < char(254)) { 
                    char recv = softSerial.read();
                    *type = recv << 4;
                    *type = *type >> 4;
                    *adress = recv >> 4;
                    if(softSerial.peek() >= char(254)) {continue;}
                    *message = softSerial.read();
                    if(softSerial.read() != byte(255)) {continue;} //look for ending signature

                    if(MASTER) {
                        //Serial.println("adress: "+(String)adress+" type: "+(String)type+" message: "+(String)message);
                        result = true;
                        //if(busTest) busTestMessages++;
                    }else{
                        if((*type == HEARTBEAT)){// || (type == CONNECT)) {
                            if(*message == 0) setDownBeat();
                            if(*message == 1) setUpBeat();
                        }else{
                            Serial.write((uint8_t)254);
                            Serial.write(recv);
                            Serial.write(*message);
                            Serial.write((uint8_t)255);
                        }
                    }
                result = true;
                }
            }
        }
    }
    return result;
}


bool Communication::readDownstream(char* adress, char* type, char* message) {
    bool result = false;
    if(!MASTER && uartConnected){
        while(Serial.available() > 3) {
            if(Serial.read() != char(254)) {continue;}
            if(Serial.peek() < char(254)) { 
                char recv = Serial.read();
                *type = recv << 4;
                *type = *type >> 4;
                *adress = recv >> 4;
                if(Serial.peek() >= char(254)) {continue;}
                *message = Serial.read();
                if(Serial.read() != char(255)) {continue;} //look for ending signature
                if((*adress == ADRESS) || (*adress == 15)) { //my adress or broadcast
                    result = true;
                    if(*adress == 15) {
                        sendDownstream(*adress, *type, *message);
                    }
                }else if(*type == HEARTBEAT){
                    if(*message == 0) setDownBeat();
                    if(*message == 1) setUpBeat();
                }else{
                    sendDownstream(*adress, *type, *message);          
                }
            }
        }
    }
    return result;
}



/* Tries to read the adress from a freshly connected module
*/
char Communication::readAdress()
{
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


/* Sets the lastUpBeat to !now
*/
void Communication::setUpBeat()
{
    lastUpBeat = millis();
}

void Communication::setDownBeat()
{
    lastDownBeat = millis();
}

/* initSoftwareUart tries to connect the softwareSerial interface
   in pitch configuration, and if not successfull in yaw configuration.
   It returns the adress of the new connected module or 255 otherwise.
*/
char Communication::initSoftwareUart()
{
    byte nextAdress = 255;
//   /*
//   if(master) {
//     Serial.print("oldBufferSize: ");
//     Serial.print(softSerial.available());
//     Serial.println()
//   }
//   */
   //try Pitch  
   softSerial = SoftwareSerial(TOPO_PIN_P, TX_PIN);  
   softSerial.begin(BAUDRATE);
   delay(1); //safety margin
   softSerial.flush();
   delay(2);
   //try to read byte:   
   nextAdress = readAdress();
 //  if(master) Serial.println("try pitch");
   if(nextAdress < 254){    
//     enqueModule(byte(nextAdress), true);
        sendDownstream(nextAdress, CONNECT, 1); //handshake
        softwareUartConnected = true;
        connectionStatus = SOFTWARE;
        lastDownBeat = millis();        
    }else{ //try Yaw
        softSerial = SoftwareSerial(TOPO_PIN_Y, TX_PIN);   
        softSerial.begin(BAUDRATE);
        delay(1);
        softSerial.flush();
        delay(2);
        nextAdress = readAdress();
    //  if(master) Serial.println("try yaw");
        if(nextAdress < 254){      
//          enqueModule(byte(nextAdress), false);
            sendDownstream(nextAdress, CONNECT, 1); //handshake
            softwareUartConnected = true;
            connectionStatus = SOFTWARE;
            lastDownBeat = millis();
        }
   }
   return nextAdress;     
}

// todo: when alternating the orientation of the module sometimes there is data on the old connected topoPin which leads to temporary wrong orientation determination
void Communication::handShake(){
  sendUpstream(ADRESS, CONNECT, 0);
  delay(10);
  while(Serial.available() > 3) {
    if(Serial.read() != char(254)) continue;
    if(Serial.peek() < char(254)) { 
      char recv = Serial.read();
      char type = recv << 4;
      type = type >> 4;
      char messageAdress = recv >> 4;
      if(Serial.peek() >= char(254)) continue;
      char message = Serial.read();
      if(Serial.read() != char(255)) continue; //look for ending signature
     // else{Serial.read();}
      if((messageAdress == ADRESS) && (message == 1)) { //my adress or broadcast
        uartConnected = true;
        lastUpBeat = millis();
        connectionStatus = HARDWARE; 
      }
    }
  } 
}

void Communication::disconnect()
{
    uartConnected = false;
    softwareUartConnected = false;
    connectionStatus = DISCONNECTED;
}


void Communication::visualizeConnectionStatus(){
    switch(connectionStatus){
        case DISCONNECTED : digitalWrite(LED_PIN, LOW); break;
        case HARDWARE : {
            if(millis()-lastBlink > 30) {
                blinkStatus = (blinkStatus == HIGH) ? LOW : HIGH;
                digitalWrite(LED_PIN, blinkStatus);
                lastBlink = millis();
            }      
        } break;
        case SOFTWARE : digitalWrite(LED_PIN, HIGH); break;
    }
}