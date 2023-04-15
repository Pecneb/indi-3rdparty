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
        /**
         * @brief Connect to arduino, by sending connection command
         * 
         * @return true 
         * @return false 
         */
        bool Handshake() override;

        /**
         * @brief Get the Default Name of the mount
         * 
         * @return const char* 
         */
        const char *getDefaultName() override;
        bool initProperties() override;

        // Telescope specific functions
        /**
         * @brief Get feedback from the mount hardware, where the telescope points,
         * which state it is in etc... This method called in each polling epoch, e.g.
         * in every 500 ms
         * 
         * @return true 
         * @return false 
         */
        bool ReadScopeStatus() override;
        /**
         * @brief Perform GOTO to the given RA DEC coordinates, by sending the steppers
         * the converted numbers (RA DEC converted into steps)
         * 
         * @return true 
         * @return false 
         */
        bool Goto(double, double) override;
        /**
         * @brief Abort mount motion, stop steppers as soon as possible, but not instantly. 
         * 
         * @return true 
         * @return false 
         */
        bool Abort() override;
        /**
         * @brief Park telescope.
         * 
         * @return true 
         * @return false 
         */
        bool Park() override;
        /**
         * @brief Unpark telescope, power on steppers.
         * 
         * @return true 
         * @return false 
         */
        bool UnPark() override;

        // Tracking
        /**
         * @brief Set the Ra tracking Rate
         * 
         * @param raRate 
         * @return true 
         * @return false 
         */
        bool SetRaRate(double raRate);
        /**
         * @brief Set the De tracking Rate, that is usually 0 if not custom rate
         * 
         * @param deRate 
         * @return true 
         * @return false 
         */
        bool SetDeRate(double deRate);
        /**
         * @brief Get the Ra Track Rate
         * 
         * @return double 
         */
        double GetRaTrackRate();
        /**
         * @brief Get the De Track Rate
         * 
         * @return double 
         */
        double GetDeTrackRate();
        /**
         * @brief Start tracking
         * 
         * @return true 
         * @return false 
         */
        bool StartTracking();
        /**
         * @brief Stop tracking
         * 
         * @return true 
         * @return false 
         */
        bool StopTracking();
        /**
         * @brief Set the Track Mode, e.g. Sidereal, Solar, Lunar, Custom
         * 
         * @param mode 
         * @return true 
         * @return false 
         */
        bool SetTrackMode(uint8_t mode) override;
        /**
         * @brief Set the custom Track Rate
         * 
         * @param raRate 
         * @param deRate 
         * @return true 
         * @return false 
         */
        bool SetTrackRate(double raRate, double deRate) override; 
        /**
         * @brief Enable or disable tracking
         * 
         * @param enabled 
         * @return true 
         * @return false 
         */
        bool SetTrackEnabled(bool enabled) override;

        /**
         * @brief Get the Longitude
         * 
         * @return double 
         */
        double getLongitude();
        /**
         * @brief Get the Latitude
         * 
         * @return double 
         */
        double getLatitude();
        /**
         * @brief Get the Julian Date
         * 
         * @return double 
         */
        double getJulianDate();
        /**
         * @brief Calculate the Local sidereal time from the Julian Date and the Longitude
         * 
         * @param jd The actual Julian date
         * @param lng The longitude
         * @return double 
         */
        double getLst(double jd, double lng);


        /**
         * @brief Get the RA Axis position in steps
         * 
         * @return INDI_EQ_AXIS 
         */
        INDI_EQ_AXIS GetRAEncoder(int32_t*);
        /**
         * @brief Get the DE Axis position in steps
         * 
         * @return INDI_EQ_AXIS 
         */
        INDI_EQ_AXIS GetDEEncoder(int32_t*);

        /**
         * @brief Convert RA and DEC encoder values to RA and DEC coordinates
         * 
         * @param rastep 
         * @param destep 
         * @param lst 
         * @param ra 
         * @param dec 
         * @param ha 
         * @param pierSide 
         */
        void StepsToRADE(int32_t rastep, int32_t destep, double lst, double* ra, double* dec, double* ha, TelescopePierSide* pierSide);
        /**
         * @brief Convert encoder steps to hours
         * 
         * @param steps 
         * @param totalstep 
         * @return double 
         */
        double StepsToHours(int32_t steps, uint32_t totalstep);
        /**
         * @brief Convert encoder ra steps to RA hours 
         * 
         * @param rastep 
         * @return double 
         */
        double StepsToRA(int32_t rastep);
        /**
         * @brief Convert encoder dec steps to DEC degrees
         * 
         * @param destep 
         * @return double 
         */
        double StepsToDE(int32_t destep);
        /**
         * @brief Convert RA hours and DEC degrees to Encoder step values.
         * 
         * @param ra 
         * @param de 
         * @param rastep 
         * @param destep 
         */
        void RADEToSteps(double ra, double de, int32_t& rastep, int32_t& destep);
        /**
         * @brief Convert RA hours to steps
         * 
         * @param ratarget 
         * @param p 
         * @param lst 
         * @param totalstep 
         * @return double 
         */
        double StepsFromRA(double ratarget, TelescopePierSide p, double lst, uint32_t totalstep);
        /**
         * @brief Convert HA hour angles to steps
         * 
         * @param hours 
         * @param totalstep 
         * @return double 
         */
        double StepsFromHour(double hours, uint32_t totalstep);
        /**
         * @brief Convert DEC degrees to steps
         * 
         * @param detarget 
         * @param p 
         * @param totalstep 
         * @return double 
         */
        double StepsFromDec(double detarget, TelescopePierSide p, uint32_t totalstep);
        /**
         * @brief Convert 360 degrees to steps
         * 
         * @param degree 
         * @param totalstep 
         * @return double 
         */
        double StepsFromDegree(double degree, uint32_t totalstep);
        /**
         * @brief Convert steps to degrees
         * 
         * @param steps 
         * @param totalstep 
         * @return double 
         */
        double StepsToDegree(int32_t steps, uint32_t totalstep);

    private:
        enum Command {
            GOTO = 65,      // "A RA DE"
            TRACK,          // "B TRACKRATE"
            PARK,           // "C"
            SETPARKPOS,     // "D"
            GETAXISSTATUS,  // "E AXIS_NUM"
            HANDSHAKE,      // "F"
            SETTRACKRATE,   // "G TRACKRATE_RA TRACKRATE_DE"
            ABORT,          // "H"
            SETIDLE,        // "I"
            ERROR = -1
        };
        double currentHA;
        double currentRA;
        double currentDEC;
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

        /**
         * @brief Dispatch command to the arduino on serial port.
         * 
         * @return true 
         * @return false 
         */
        bool sendCommand(const char*, char*, int, int);
        /**
         * @brief Convert char* to HEX
         * 
         */
        void hexDump(char*, const char*, int);

        // Serial communication helper properties
        int DRIVER_TIMEOUT = 10;
        char DRIVER_STOP_CHAR = '\n';
};
