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

#ifndef BUSTEST_H
#define BUSTEST_H

    #include "configuration.h"
    #include "communication.h"

    class BusTest
    {
    public:
        BusTest(Communication* com);
        void tick();
        void start(unsigned int freq, byte adress);
        void stop();
        void ping();
        bool isActive();
    private:
        boolean active;
        unsigned long testTime;
        unsigned long lastSent;
        unsigned int sent;
        unsigned int recv;
        unsigned int freq;
        byte adress;
        Communication* com;
    };

#endif // BUSTEST_H