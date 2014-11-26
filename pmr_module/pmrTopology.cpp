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

#include "pmrTopology.h"

PMRTopology::PMRTopology()
{
    //initialize topology array with master adress
    topology[0].adress = 0;
    modulesCount = 1;  //only used in master-mode. Initialized with 1 for master as first module.
    pitchingJointsCount = 1;
}


// master: enques new module into topology arry
// slave:  sends detected module upstream
byte PMRTopology::enqueModule(byte adress, bool orientation) {  
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

    return modulesCount;
}


/* Removes all modules downstream of the given adress. The given adress will not be removed!
*/
byte PMRTopology::removeModules(byte adress) {
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
    countPitchingJoints();         
}


/* Prints information about the current topology to the host in human readable form
*/
void PMRTopology::printTopology() {
    if(MASTER) {
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
}


byte PMRTopology::getAdress(byte index)
{
    return topology[index].adress;
}

bool PMRTopology::getOrientation(byte index)
{
    return topology[index].orientation;
}

/* Returns the total number of currently connected modules
*/
byte PMRTopology::getModulesCount()
{
    return modulesCount;
}

/* Returns the number of pitching joints in this snake
*/
byte PMRTopology::getPitchingJointsCount()
{
    return pitchingJointsCount;
}

// returns true if adress is already registered
bool PMRTopology::searchDuplicateInTopology(byte adress){
  bool foundAdress = false;
  for(int i=0; i<modulesCount; i++){
    if(topology[i].adress == adress){
      foundAdress = true;
      break;
    }
  }
  return foundAdress;
}


/* Updates the internal counter of pitching joints
*/
void PMRTopology::countPitchingJoints(){
  pitchingJointsCount = 0;
  for(int i=0; i<modulesCount; i++){
    if(topology[i].orientation) pitchingJointsCount++;  
  }
} 