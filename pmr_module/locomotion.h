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

#ifndef LOCOMOTION_H
#define LOCOMOTION_H

    #include "configuration.h"
    #include "communication.h"
    #include "hardwareControl.h"
    #include "pmrTopology.h"

    enum LocomotionType
    {
        WALK = 0,
        ROLL = 1
    };

    class Locomotion
    {
    public:
        Locomotion(Communication* com, HardwareControl* hc, PMRTopology* topology);
        void tick();
        void start();
        void stop();
        void walk();
        void roll();
        bool changeDirection();
        void setFrequency(float frequency);
        void setAmplitude(float amplitude);
        void setPhase(float phase);
        void setSampling(unsigned int sampling);
        void gatherAngles(bool gather);
    private:
        float sineFunction(float amp, float phase, float freq, float offset);
        float* calculateNextJointPositions(int numOfJoints, bool forward);
        void moveSinusoidal();
        void moveRoll();

        bool active;
        bool forward;
        LocomotionType locomotion;
        bool locomotionGather;
        float locomotionAngles[15];
        unsigned int sampling;
        unsigned long lastSample;

        //########sine generator parameters#########
        float sine_amplitude;
        float sine_frequency;
        float sine_phase;
        float sine_offset;
        //##########################################

        Communication* com;
        HardwareControl* hc;
        PMRTopology* topology;

    };

#endif // LOCOMOTION_H