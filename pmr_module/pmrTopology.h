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

#ifndef PMRTOPOLOGY_H
#define PMRTOPOLOGY_H

    #include "configuration.h"

    class PMRTopology
    {
    public:
        PMRTopology();
        byte enqueModule(byte adress, bool orientation);
        byte removeModules(byte adress);
        void printTopology();

        byte getAdress(byte index);
        bool getOrientation(byte index);
        byte getModulesCount();
        byte getPitchingJointsCount();
    private:
        bool searchDuplicateInTopology(byte adress);
        void countPitchingJoints();

        byte modulesCount;
        byte pitchingJointsCount;
        Topology topology[32];  //relative topology in direction from master to last slave
        byte pitchingJoints[32]; //adresses of the pitching group
    };

#endif // PMRTOPOLOGY_H