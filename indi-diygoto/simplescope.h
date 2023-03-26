/**
 * @file simplescope.h
 * @author Bence Peter (ecneb2000@gmail.com)
 * @brief Simple GOTO implementation with Arduino UNO and Stepper Motors
 * @version 0.1
 * @date 2023-03-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include "libindi/inditelescope.h"

#define SIDEREAL_DAY 86164.09053083288  // seconds 
#define SIDEREAL_SPEED 15.04106864      // arcsec/s
#define STELLAR_DAY 86164.098903691     // seconds
#define STELLAR_SPEED 15.041067179      // arcsec/s

#define PULLEY_RATIO 60/20              // pulley reduction ratio
#define EQ32_RA_WORM_GEAR_RATIO 130/1   // EQ3-2 RA axis worm gear reduction ratio
#define EQ32_DE_WORM_GEAR_RATIO 65/1    // EQ3-2 DE axis worm gear reduction ratio
#define STEPPER_STEPS_PER_REV 200       // steps per revolution
#define STEPPER_STEPSIZE 1.8            // degrees/step
#define STEPS_PER_RA_REV (STEPPER_STEPS_PER_REV * PULLEY_RATIO * EQ32_RA_WORM_GEAR_RATIO) // steps / one revolution on the RA axis
#define STEPS_PER_DE_REV (STEPPER_STEPS_PER_REV * PULLEY_RATIO * EQ32_DE_WORM_GEAR_RATIO) // steps / one revolution on the DE axis
#define STEPSIZE_RA (STEPPER_STEPSIZE / (PULLEY_RATIO * EQ32_RA_WORM_GEAR_RATIO)) // degrees/step
#define STEPSIZE_DE (STEPPER_STEPSIZE / (PULLEY_RATIO * EQ32_DE_WORM_GEAR_RATIO)) // degrees/step

#define GOTO_RATE 2 // slew rate, degrees/s
#define SLEW_RATE 0.5 // slew rate, degree/s
#define FINE_SLEW_RATE 0.1 // slew rate, degrees/s
#define SID_RATE 0.004178 // slew rate, degrees/s

class SimpleScope : public INDI::Telescope
{
    public:
        SimpleScope();

    protected:
        bool Handshake() override;

        const char *getDefaultName() override;
        bool initProperties() override;

        // Telescope specific functions
        bool ReadScopeStatus() override;
        bool Goto(double, double) override;
        bool Abort() override;

        int32_t GetRAEncoder();
        int32_t GetDEEnconder();

        void EncoderToRADEC(int32_t rastep, int32_t destep, double ra, double dec);
        int32_t EncoderToRA(int32_t rastep);
        int32_t EncoderToDE(int32_t destep);

        void RADEToSteps(double ra, double de, int32_t rastep, int32_t destep);
        int32_t RAToSteps(double ra);
        int32_t DEToSteps(double de);

    private:
        enum Command {
            GOTO = 'A',
            TRACK = 'B',
            PARK = 'C',
            SETPARKPOS = 'D',
            GETAXISSTATUS = 'E',
            HANDSHAKE = 'F'
        };

        double currentRA {0};
        double currentDEC {90};
        double targetRA {0};
        double targetDEC {0};

        int32_t RAStepRel; // Current RA encoder position in step relative to position 0
        int32_t DEStepRel; // Current DE encoder position in step relative to position 0
        int32_t RAStepAbs; // Current RA encoder position in step
        int32_t DEStepAbs; // Current DE encoder position in step
        int32_t RAStepInit; // Initial RA position in step
        int32_t DEStepInit; // Initial DE position in step
        int32_t RAStepHome; // Home RA position in step
        int32_t DEStepHome; // Home DE position in step

        int32_t lastRAStep;
        int32_t lastDEStep;

        int32_t targetRAStep; // Target RA position in step
        int32_t targetDEStep; // Target RA position in step


        // Debug channel to write mount logs to
        // Default INDI::Logger debugging/logging channel are Message, Warn, Error, and Debug
        // Since scope information can be _very_ verbose, we create another channel SCOPE specifically
        // for extra debug logs. This way the user can turn it on/off as desired.
        uint8_t DBG_SCOPE { INDI::Logger::DBG_IGNORE };

        // Serial communication helper functions 
        bool sendCommand(const char*, char*, int, int);
        void hexDump(char*, const char*, int);

        // Serial communication helper properties
        int DRIVER_LEN = 64;
        int DRIVER_TIMEOUT = 3;
        char DRIVER_STOP_CHAR = '\0';
        char DRIVER_OK = 1;
        char DRIVER_ERROR = 2;
};
