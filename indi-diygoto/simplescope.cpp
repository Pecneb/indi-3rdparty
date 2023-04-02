/*
   INDI Developers Manual
   Tutorial #2

   "Simple Telescope Driver"

   We develop a simple telescope simulator.

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/

/** \file simplescope.cpp
    \brief Construct a basic INDI telescope device that simulates GOTO commands.
    \author Jasem Mutlaq

    \example simplescope.cpp
    A simple GOTO telescope that simulator slewing operation.
*/

#include "simplescope.h"

#include "libindi/indicom.h"

#include <cstdint>
#include <cstdio>
#include <libindi/indilogger.h>
#include <termios.h>
#include <cmath>
#include <memory>
#include <string.h>

static std::unique_ptr<SimpleScope> simpleScope(new SimpleScope());

SimpleScope::SimpleScope()
{
    // We add an additional debug level so we can log verbose scope status
    DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");
}

/**************************************************************************************
** We init our properties here. The only thing we want to init are the Debug controls
***************************************************************************************/
bool SimpleScope::initProperties()
{
    // ALWAYS call initProperties() of parent first
    INDI::Telescope::initProperties();

    // Add Debug control so end user can turn debugging/loggin on and off
    addDebugControl();

    // Enable simulation mode so that serial connection in INDI::Telescope does not try
    // to attempt to perform a physical connection to the serial port.
    setSimulation(false);

    // Set telescope capabilities. 0 is for the the number of slew rates that we support. We have none for this simple driver.
    SetTelescopeCapability(TELESCOPE_CAN_GOTO | TELESCOPE_CAN_ABORT, 0);

    return true;
}

/**************************************************************************************
** INDI is asking us to check communication with the device via a handshake
***************************************************************************************/
bool SimpleScope::Handshake()
{
    if (isSimulation()) return true;

    char cmd[DRIVER_LEN] {HANDSHAKE};
    char res[DRIVER_LEN] { 0 };
    sendCommand(cmd, res, DRIVER_LEN, DRIVER_LEN);

    char expect[DRIVER_LEN] { 1 };
    expect[1] = '\0';
    if (strcmp(res, expect)) return true;
    return false;
}

/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char *SimpleScope::getDefaultName()
{
    return "ArduinoGOTO";
}

int32_t SimpleScope::RAToSteps(double ra) {
    int32_t result = 0;
    if (ra > 12.0) {
        result = CW * STEPS_PER_RA_REV * (ra/24);
    } else {
        result = CCW * (STEPS_PER_RA_REV - (STEPS_PER_RA_REV * (ra/24)));
    }
    return result;
}

int32_t SimpleScope::DEToSteps(double de) {
    int32_t result = 0;
    if (targetPierSide == PIER_WEST){
        result = CCW * (STEPS_PER_DE_REV * ((90.0 - de)/360.0));
    } else if (targetPierSide == PIER_EAST) {
        result = CW * (STEPS_PER_DE_REV * ((90.0 - de)/360.0));
    }
    return result;
}

void SimpleScope::RADEToSteps(double ra, double de, int32_t& rastep, int32_t& destep) {
    rastep = RAToSteps(ra);
    destep = DEToSteps(de);
}

double SimpleScope::StepsToRA(int32_t rastep) {
    double result = 0;
    if (rastep > 0) {
        result = (double)(rastep / STEPS_PER_RA_REV) * 24;
    } else {
        result = (double)((STEPS_PER_RA_REV + rastep) / STEPS_PER_RA_REV) * 24;
    }
    return result;
} 

double SimpleScope::StepsToDE(int32_t destep) {
    double result = 0;
    int32_t absDeStep = std::abs(destep);
    double tmpDeg = ((double)(destep / STEPS_PER_DE_REV) * 360);
    if (tmpDeg <= 90) {
        result = 90 - tmpDeg;
    } else {
        result = (tmpDeg - 90) * -1;
    }
    return result;
}

void SimpleScope::StepsToRADE(int32_t rastep, int32_t destep, double& ra, double& dec) {
    ra = StepsToRA(rastep);
    dec = StepsToDE(destep);
}

/**************************************************************************************
** Client is asking us to slew to a new position
***************************************************************************************/
bool SimpleScope::Goto(double ra, double dec)
{
    targetRA  = ra;
    targetDEC = dec;

    // Set target pierside
    if (ra > 12.0) {
        targetPierSide = PIER_WEST;
    } else {
        targetPierSide = PIER_EAST;
    }

    RADEToSteps(targetRA, targetDEC, targetRAStep, targetDEStep);

    char cmd[DRIVER_LEN] {0};
    char res[DRIVER_LEN] {0};

    sprintf(cmd, "%c:%d:%d", GOTO, targetRAStep, targetDEStep);
    sendCommand(cmd, res, DRIVER_LEN, DRIVER_LEN);

    if (res[0] != GOTO) return false;

    LOGF_DEBUG("Slewing to RA: %g - DEC: %g", targetRA, targetDEC);

    char RAStr[64] = {0}, DecStr[64] = {0};

    // Parse the RA/DEC into strings
    fs_sexa(RAStr, targetRA, 2, 3600);
    fs_sexa(DecStr, targetDEC, 2, 3600);

    // Mark state as slewing
    TrackState = SCOPE_SLEWING;

    // Inform client we are slewing to a new position
    LOGF_INFO("Slewing to RA: %s - DEC: %s", RAStr, DecStr);

    // Success!
    return true;
}

/**************************************************************************************
** Client is asking us to abort our motion
***************************************************************************************/
bool SimpleScope::Abort()
{
    return true;
}

int32_t SimpleScope::GetRAEncoder() {
    char cmd[DRIVER_LEN] {0};
    char res[DRIVER_LEN] {0};
    int32_t RaSteps = 0;
    char resCode;

    sprintf(cmd, "%c:%d", GETAXISSTATUS, RA_AXIS);
    sendCommand(cmd, res, DRIVER_LEN, DRIVER_LEN);
    sscanf(res, "%c:%d", &resCode, &RaSteps);

    return RaSteps; 
}

int32_t SimpleScope::GetDEEnconder() {
    char cmd[DRIVER_LEN] {0};
    char res[DRIVER_LEN] {0};
    int32_t DeSteps = 0;
    char resCode;

    sprintf(cmd, "%c:%d", GETAXISSTATUS, RA_AXIS);
    sendCommand(cmd, res, DRIVER_LEN, DRIVER_LEN);
    sscanf(res, "%c:%d", &resCode, &DeSteps);

    return DeSteps; 
}

double SimpleScope::StepsToDEG(int32_t steps, int8_t axis) {
    double result = 0;
    switch (axis) {
        case RA_AXIS:
            if (steps > 0) {
                result = (double)(steps / STEPS_PER_RA_REV) * 360;
            } else {
                result = (double)((STEPS_PER_RA_REV + steps) / STEPS_PER_RA_REV) * 360;
            }
            break;
        case DE_AXIS:
            if (steps > 0) {
                result = (double)(steps / STEPS_PER_DE_REV) * 360;
            } else {
                result = (double)((STEPS_PER_DE_REV + steps) / STEPS_PER_DE_REV) * 360;
            }
            break;
    }
    return result;
}

/**************************************************************************************
** Client is asking us to report telescope status
***************************************************************************************/
bool SimpleScope::ReadScopeStatus()
{
    static struct timeval ltv
    {
        0, 0
    };
    struct timeval tv
    {
        0, 0
    };
    double dt = 0, da_ra = 0, da_dec = 0, dx = 0, dy = 0;
    int nlocked;

    /* update elapsed time since last poll, don't presume exactly POLLMS */
    gettimeofday(&tv, nullptr);

    if (ltv.tv_sec == 0 && ltv.tv_usec == 0)
        ltv = tv;

    dt  = tv.tv_sec - ltv.tv_sec + (tv.tv_usec - ltv.tv_usec) / 1e6;
    ltv = tv;

    // Get encoder status
    int32_t currentEncoderRA = GetRAEncoder();
    int32_t currentEncoderDE = GetDEEnconder();

    // Convert encoder steps to RA/DEC double
    StepsToRADE(currentEncoderRA, currentEncoderDE, currentRA, currentDEC);

    // Calculate how much we moved since last time
    da_ra  = SLEW_RATE * dt;
    da_dec = SLEW_RATE * dt;

    /* Process per current state. We check the state of EQUATORIAL_EOD_COORDS_REQUEST and act acoordingly */
    switch (TrackState)
    {
        case SCOPE_SLEWING:
            // Wait until we are "locked" into positon for both RA & DEC axis
            nlocked = 0;

            // Calculate diff in RA
            dx = targetRA - currentRA;

            // If diff is very small, i.e. smaller than how much we changed since last time, then we reached target RA.
            if (fabs(dx) * 15. <= da_ra)
            {
                currentRA = targetRA;
                nlocked++;
            }
            // Otherwise, increase RA
            //else if (dx > 0)
            //    currentRA += da_ra / 15.;
            // Otherwise, decrease RA
            //else
            //    currentRA -= da_ra / 15.;

            // Calculate diff in DEC
            dy = targetDEC - currentDEC;

            // If diff is very small, i.e. smaller than how much we changed since last time, then we reached target DEC.
            if (fabs(dy) <= da_dec)
            {
                currentDEC = targetDEC;
                nlocked++;
            }
            // Otherwise, increase DEC
            // else if (dy > 0)
            //     currentDEC += da_dec;
            // // Otherwise, decrease DEC
            // else
            //     currentDEC -= da_dec;

            // Let's check if we recahed position for both RA/DEC
            if (nlocked == 2)
            {
                lastPierSide = currentPierSide;
                Telescope::setPierSide(targetPierSide);

                // Let's set state to TRACKING
                TrackState = SCOPE_TRACKING;

                LOG_INFO("Telescope slew is complete. Tracking...");
            }
            break;

        default:
            break;
    }

    char RAStr[64] = {0}, DecStr[64] = {0};

    // Parse the RA/DEC into strings
    fs_sexa(RAStr, currentRA, 2, 3600);
    fs_sexa(DecStr, currentDEC, 2, 3600);

    DEBUGF(DBG_SCOPE, "Current RA: %s Current DEC: %s\n", RAStr, DecStr);

    NewRaDec(currentRA, currentDEC);
    return true;
}

bool SimpleScope::sendCommand(const char * cmd, char * res, int cmd_len, int res_len)
{
    int nbytes_written = 0, nbytes_read = 0, rc = -1;

    tcflush(PortFD, TCIOFLUSH);
    LOGF_DEBUG("CMD <%s>", cmd);

    if (cmd_len > 0)
    {
        char hex_cmd[DRIVER_LEN * 3] = {0};
        hexDump(hex_cmd, cmd, cmd_len);
        LOGF_DEBUG("CMD <%s>", hex_cmd);
        rc = tty_write(PortFD, cmd, cmd_len, &nbytes_written);
    }
    else
    {
        LOGF_DEBUG("CMD <%s>", cmd);
        rc = tty_write_string(PortFD, cmd, &nbytes_written);
    }

    if (rc != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial write error: %s.", errstr);
        return false;
    }

    if (res == nullptr)
        return true;

    if (res_len > 0)
        rc = tty_read(PortFD, res, res_len, DRIVER_TIMEOUT, &nbytes_read);
    else
        rc = tty_nread_section(PortFD, res, DRIVER_LEN, DRIVER_STOP_CHAR, DRIVER_TIMEOUT, &nbytes_read);

    if (rc != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial read error: %s.", errstr);
        return false;
    }

    if (res_len > 0)
    {
        char hex_res[DRIVER_LEN * 3] = {0};
        hexDump(hex_res, res, res_len);
        LOGF_DEBUG("RES <%s>", hex_res);
    }
    else
    {
        LOGF_DEBUG("RES <%s>", res);
    }

    tcflush(PortFD, TCIOFLUSH);

    return true;
}

void SimpleScope::hexDump(char * buf, const char * data, int size)
{
    for (int i = 0; i < size; i++)
        sprintf(buf + 3 * i, "%02X ", static_cast<uint8_t>(data[i]));

    if (size > 0)
        buf[3 * size - 1] = '\0';
}