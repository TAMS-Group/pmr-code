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

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

    #include "configuration.h"
    #include <SoftwareSerial.h>

    class Communication
    {
    public:
        Communication();
        void init();
        bool connect(byte* adress, bool* orientation);
        bool heartBeat();
        void sendUpstream(byte adress, byte type, byte message);
        void sendDownstream(byte adress, byte type, byte message);
        bool readUpstream(byte* adress, byte* type, byte* message);
        bool readDownstream(byte* adress, byte* type, byte* message);
        void setUpBeat();
        void setDownBeat();
        void disconnect();
    private:
        bool initSoftwareUart(byte* adress, bool* orientation);
        byte readAdress();
        void handShake();
        void visualizeConnectionStatus();

        bool uartConnected;
        bool softwareUartConnected;
        unsigned long lastUpBeat;
        unsigned long lastDownBeat;
        unsigned long lastUpBeatSent;
        unsigned long lastDownBeatSent;

        SoftwareSerial softSerial;
        ConnectionStatus connectionStatus;
        unsigned long lastBlink;
        uint8_t blinkStatus;
    };

#endif // COMMUNICATION_H