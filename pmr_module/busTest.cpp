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

#include "busTest.h"

BusTest::BusTest(Communication* com)
{
    active = false;
    testTime = 0; //how long since last print of results to console
    lastSent = 0; //how long since last ping message has been sent
    sent = 0; //ping messages sent for bus testing
    recv = 0; //ping messages recieved
    freq = 0; //desired ping frequency
    BusTest::com = com;
}

/* clock, must be called periodically from loop()
*/
void BusTest::tick()
{
    if(active) {
        if((millis() - lastSent) > freq) {
            com->sendDownstream(adress, PING, 1);
            lastSent = millis();
            sent++;
        }
        if((millis() - testTime) > 1000) {
            Serial.print("busTest: ");
            Serial.print(com->getMessagesSent());
            Serial.println("msg/s");
            Serial.print("sent: ");
            Serial.print(sent);
            Serial.println(" ping messages, ");
            Serial.print("recv: ");
            Serial.println(recv);
            Serial.print("lost: ");
            Serial.println(sent - recv);
            Serial.println();

            testTime = millis();
            sent = 0;
            recv = 0;
        }
    }
}

/* enables the bustest module, resets status
*/
void BusTest::start(unsigned int freq, byte adress)
{
    if(freq > 0) {
        testTime = 0;
        lastSent = 0;
        sent = 0;
        recv = 0;
        BusTest::freq = freq;
        BusTest::adress = adress;

        active = true;

        Serial.println("BusTest enabled!");
        Serial.print("sending ping message to ");
        Serial.print(int(adress));
        Serial.print(" every ");
        Serial.print(freq);
        Serial.println("ms");
    }
}

/* disables the bustest module
*/
void BusTest::stop()
{
    active = false;
    Serial.println("BusTest disabled");
}

/* Indicates a replied ping message. Called from processCommand, increases the counter.
*/
void BusTest::ping()
{
    recv++;
}

bool BusTest::isActive()
{
    return active;
}