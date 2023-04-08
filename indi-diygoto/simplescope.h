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

#include "indibasetypes.h"
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

#define CW 1 // Clockwise rotation of stepper
#define CCW -1 // Counter Clockwise ratation of stepper

#define GOTO_RATE 2 // slew rate, degrees/s
#define SLEW_RATE 0.5 // slew rate, degree/s
#define FINE_SLEW_RATE 0.1 // slew rate, degrees/s
#define TRACK_RATE 0.004178074 // slew rate, degrees/s

#define DRIVER_LEN 64

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

        // Tracking
        bool SetRaRate(double raRate);
        bool SetDeRate(double deRate);
        double GetRaTrackRate();
        double GetDeTrackRate();
        bool StartRATracking();
        bool StopRATracking();
        bool StartDETracking();
        bool StopDETracking();
        bool SetTrackMode(uint8_t mode) override;
        bool SetTrackRate(double raRate, double deRate) override; 
        bool SetTrackEnabled(bool enabled) override;

        double getLongitude();
        double getLatitude();
        double getJulianDate();
        double getLst(double jd, double lng);


        // Axis positions in steps
        INDI_EQ_AXIS GetRAEncoder(int32_t*);
        INDI_EQ_AXIS GetDEEncoder(int32_t*);

        // Conversions between steps and RA/DE
        void StepsToRADE(int32_t rastep, int32_t destep, double lst, double* ra, double* dec, double* ha, TelescopePierSide* pierSide);
        double StepsToHours(int32_t steps, uint32_t totalstep);
        double StepsToRA(int32_t rastep);
        double StepsToDE(int32_t destep);
        void RADEToSteps(double ra, double de, int32_t& rastep, int32_t& destep);
        double StepsFromRA(double ratarget, TelescopePierSide p, double lst, uint32_t totalstep);
        double StepsFromHour(double hours, uint32_t totalstep);
        double StepsFromDec(double detarget, TelescopePierSide p, uint32_t totalstep);
        double StepsFromDegree(double degree, uint32_t totalstep);
        double StepsToDegree(int32_t steps, uint32_t totalstep);

    private:
        enum Command {
            GOTO = 'A',
            TRACK = 'B',
            PARK = 'C',
            SETPARKPOS = 'D',
            GETAXISSTATUS = 'E',
            HANDSHAKE = 'F',
            SETTRACKRATE = 'G',
            ERROR = 'X'
        };

        double currentHA {0};
        double currentRA {0};
        double currentDEC {0};
        double targetHA;
        double targetRA;
        double targetDEC;
        TelescopePierSide targetPierSide {PIER_UNKNOWN};
        int32_t currentRAEncoder;
        int32_t currentDEEncoder;
        int32_t targetRAEncoder;
        int32_t targetDEEncoder;

        int32_t lastRAStep;
        int32_t lastDEStep;

        int32_t RAParkEncoder {0};
        int32_t DEParkEncoder {0};

        // Debug channel to write mount logs to
        // Default INDI::Logger debugging/logging channel are Message, Warn, Error, and Debug
        // Since scope information can be _very_ verbose, we create another channel SCOPE specifically
        // for extra debug logs. This way the user can turn it on/off as desired.
        uint8_t DBG_SCOPE { INDI::Logger::DBG_IGNORE };

        // Serial communication helper functions 
        bool sendCommand(const char*, char*, int, int);
        void hexDump(char*, const char*, int);

        // Serial communication helper properties
        int DRIVER_TIMEOUT = 10;
        char DRIVER_STOP_CHAR = '\0';
};
