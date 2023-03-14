/** \file simplescope.h
  * \brief Construct a basic INDI telescope device that performs GOTO commands.
  * \author Bence Peter 
  * \example simplescope.h
  * A simple GOTO telescope that performs slewing operation.
  */

#pragma once

#include "libindi/indiguiderinterface.h"
#include "libindi/inditelescope.h"
#include "libindi/alignment/AlignmentSubsystemForDrivers.h" 

class Mount: public INDI::Telescope //, public INDI::GuiderInterface, public INDI::AlignmentSubsystem::AlignmentSubsystemForDrivers
{
    public:
        Mount();

    protected:
        bool Handshake() override;

        const char *getDefaultName() override;
        bool initProperties() override;

        // Telescope specific functions
        bool ReadScopeStatus() override;
        bool Goto(double, double) override;
        bool Abort() override;

    private:
        double currentRA {0};
        double currentDEC {90};
        double targetRA {0};
        double targetDEC {0};

        // Debug channel to write mount logs to
        // Default INDI::Logger debugging/logging channel are Message, Warn, Error, and Debug
        // Since scope information can be _very_ verbose, we create another channel SCOPE specifically
        // for extra debug logs. This way the user can turn it on/off as desired.
        uint8_t DBG_SCOPE { INDI::Logger::DBG_IGNORE };

        // slew rate, degrees/s
        static const uint8_t SLEW_RATE = 3;
};
