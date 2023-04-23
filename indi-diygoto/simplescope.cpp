/**
 * @file simplescope.cpp
 * @author Bence Peter (ecneb2000@gmail.com)
 * @brief Simple GOTO implementation with Arduino UNO and Stepper Motors
 * @version 0.1
 * @date 2023-03-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "simplescope.h"

#include <libindi/indiapi.h>
#include <libindi/indibasetypes.h>
#include <libindi/indidevapi.h>
#include <libindi/indicom.h>

#include <cstdint>
#include <cstdio>
#include <libindi/indiapi.h>
#include <libindi/indibasetypes.h>
#include <libindi/indidevapi.h>
#include <libindi/indilogger.h>
#include <libnova/julian_day.h>
#include <libnova/sidereal_time.h>
#include <termios.h>
#include <cmath>
#include <memory>
#include <string.h>
#include "mach_gettime.h"

/* Preset Slew Speeds */
#define SLEWMODES 11
double slewspeeds[SLEWMODES - 1] = { 1.0, 2.0, 4.0, 8.0, 32.0, 64.0, 128.0, 600.0, 700.0, 800.0 };

static std::unique_ptr<SimpleScope> simpleScope(new SimpleScope());

SimpleScope::SimpleScope()
{
    // We add an additional debug level so we can log verbose scope status
    DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");

    currentHA = 0.0;
    currentDEC = 0.0;

    // Set telescope capabilities. 0 is for the the number of slew rates that we support. We have none for this simple driver.
    SetTelescopeCapability( TELESCOPE_CAN_GOTO          | 
                            TELESCOPE_CAN_ABORT         | 
                            TELESCOPE_CAN_CONTROL_TRACK |
                            TELESCOPE_HAS_PIER_SIDE     | 
                            TELESCOPE_HAS_TRACK_MODE    | 
                            TELESCOPE_HAS_TRACK_RATE    |
                            TELESCOPE_CAN_PARK, 4);
}

/**************************************************************************************
** We init our properties here. The only thing we want to init are the Debug controls
***************************************************************************************/
bool SimpleScope::initProperties()
{
    // ALWAYS call initProperties() of parent first
    INDI::Telescope::initProperties();
   
    /*
    for (int i = 0; i < SlewRateSP.nsp - 1; i++) {
        SlewRateSP.sp[i].s = ISS_OFF;
        sprintf(SlewRateSP.sp[i].label, "%.fx", slewspeeds[i]);
        SlewRateSP.sp[i].aux = (void*)&slewspeeds[i];
    }

    // Since last item is NOT maximum (but custom), let's set item before custom to SLEWMAX
    SlewRateSP.sp[SlewRateSP.nsp - 2].s = ISS_ON;
    strncpy(SlewRateSP.sp[SlewRateSP.nsp - 2].name, "SLEW_MAX", MAXINDINAME);
    // Last is custom
    strncpy(SlewRateSP.sp[SlewRateSP.nsp - 1].name, "SLEWCUSTOM", MAXINDINAME);
    strncpy(SlewRateSP.sp[SlewRateSP.nsp - 1].label, "Custom", MAXINDILABEL);
    */

    IUFillSwitch(&SlewRateS[SLEW_GUIDE], "SLEW_GUIDE", "Guide", ISS_OFF);
    IUFillSwitch(&SlewRateS[SLEW_CENTERING], "SLEW_CENTERING", "Centering", ISS_OFF);
    IUFillSwitch(&SlewRateS[SLEW_FIND], "SLEW_FIND", "Find", ISS_OFF);
    IUFillSwitch(&SlewRateS[SLEW_MAX], "SLEW_MAX", "Max", ISS_ON);
    IUFillSwitchVector(&SlewRateSP, SlewRateS, 4, getDeviceName(), "TELESCOPE_SLEW_RATE", "Slew Rate", MOTION_TAB,
                       IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    AddTrackMode("TRACK_SIDEREAL", "Sidereal", true);
    AddTrackMode("TRACK_SOLAR", "Solar");
    AddTrackMode("TRACK_LUNAR", "Lunar");
    AddTrackMode("TRACK_CUSTOM", "Custom");

    AxisStatus = IDLE;
    TrackState = SCOPE_IDLE;
 

    SetParkDataType(PARK_RA_DEC_ENCODER);

    //INDI::GuiderInterface::initGuiderProperties(getDeviceName(), MOTION_TAB);

    // Add Debug control so end user can turn debugging/loggin on and off
    addDebugControl();

    // Enable simulation mode so that serial connection in INDI::Telescope does not try
    // to attempt to perform a physical connection to the serial port.
    setSimulation(false);

    addAuxControls();

    /*
    serialConnection = new Connection::Serial(this);
    serialConnection->registerHandshake([&]() { return Handshake(); })
    serialConnection->setDefaultBaudRate(Connection::Serial::B_9600);
    serialConnection->setDefaultPort("/dev/ttyACM0");
    registerConnection(serialConnection);
    */

    return true;
}

/**************************************************************************************
** INDI is asking us to check communication with the device via a handshake
***************************************************************************************/
bool SimpleScope::Handshake()
{
    if (isSimulation()) return true;

    char cmd[DRIVER_LEN] { 0 };
    char res[DRIVER_LEN] { 0 };
    cmd[0] = HANDSHAKE;
    cmd[1] = DRIVER_STOP_CHAR;
    cmd[2] = '\0';
    sendCommand(cmd, res, 0, 0);

    if (cmd[0] != ERROR) return true;
    return false;
}

bool SimpleScope::Park() {
    if (TrackState == SCOPE_SLEWING) {
        LOG_INFO("Mount is currently slewing cant initiate parking right now!");
    } else {
        char cmd[DRIVER_LEN] {0};
        char res[DRIVER_LEN] {0};
        sprintf(cmd, "%c%c", PARK, DRIVER_STOP_CHAR);
        sendCommand(cmd, res, 0, 0);
        if (res[0] != ERROR) {
            TrackState = SCOPE_PARKING;
            AxisStatus = PARKING;
            return true;
        }
    }
    return false;
}

bool SimpleScope::UnPark() {
    SetParked(false);
    return true;
}

/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char *SimpleScope::getDefaultName()
{
    return "ArduinoGOTO";
}

double SimpleScope::getLongitude() {
    return (IUFindNumber(&LocationNP, "LONG")->value);
}

double SimpleScope::getLatitude() {
    return (IUFindNumber(&LocationNP, "LAT")->value);
}

double SimpleScope::getJulianDate() {
    return ln_get_julian_from_sys();
}

double SimpleScope::getLst(double jd, double lng) {
    double lst;
    lst = ln_get_apparent_sidereal_time(jd);
    lst += (lng / 15.0);
    lst = range24(lst);
    return lst;
} 

double SimpleScope::StepsToHours(int32_t steps, uint32_t totalstep) {
    double result = 0.0;
    if (steps < 0) {
        result = static_cast<double>((double)steps / totalstep) * 24.0;
    } else {
        result = static_cast<double>((double)steps / totalstep) * 24.0;
        result = 24.0 + result;
    }
    result = range24(result - 6);
    //LOGF_DEBUG("Calculating steps: %i to hours: %f", steps, result);
    return result;
}

double SimpleScope::StepsToDegree(int32_t steps, uint32_t totalstep) {
    double result = 0;
    result = static_cast<double>(((double)steps / totalstep)) * 360;
    //LOGF_DEBUG("Calculating steps: %i to degrees: %f. Calculation goes like: %i / %i = %f", steps, result, steps, totalstep, static_cast<double>(((double)steps / totalstep)));
    return result;
}

void SimpleScope::StepsToRADE(int32_t rastep, int32_t destep, double lst, double* ra, double* dec, double *ha, 
    TelescopePierSide* pierSide) {
    double RACurrent = 0.0, DECurrent = 0.0, HACurrent = 0.0;
    TelescopePierSide p;
    HACurrent = StepsToHours(rastep, STEPS_PER_RA_REV); 
    RACurrent = lst - HACurrent;
    DECurrent = StepsToDegree(destep, STEPS_PER_DE_REV);
    if ((DECurrent > 90.0) && (DECurrent <= 270.0)) {
        p = PIER_EAST;
    } else {
        //RACurrent = RACurrent - 12.0;
        p = PIER_WEST;
    }
    HACurrent = rangeHA(HACurrent);
    RACurrent = range24(RACurrent);
    DECurrent = rangeDec(DECurrent);
    *ra       = RACurrent;
    *dec       = DECurrent;
    if (ha)
        *ha = HACurrent;
    if (pierSide)
        *pierSide = p;
}

double SimpleScope::StepsFromHour(double hour, uint32_t totalstep) {
    double steps = 0.0;
    double shifthour = range24(hour + 6);
    if (shifthour < 12) {
        steps = round(CW * (shifthour / 24.0) * totalstep);
    } else {
        steps = round(CCW * ((24.0 - shifthour) / 24.0) * totalstep);
    }
    LOGF_DEBUG("HA: %f = Steps: %f", hour, steps);
    return steps;
}

double SimpleScope::StepsFromRA(double ratarget, TelescopePierSide p, double lst, uint32_t totalstep) {
    double ha = 0.0;
    //double raDeg = ratarget * 15.0;
    //double lstDeg = lst * 15.0;
    //double haDeg = lstDeg - raDeg;
    ha = lst - ratarget;


    //if (p == PIER_WEST)
    //    ha = ha + 12.0;

    ha = range24(ha);
    return StepsFromHour(ha, STEPS_PER_RA_REV);
}

double SimpleScope::StepsFromDegree(double degree, uint32_t totalstep) {
    double target = 0;
    target = range360(degree); 
    if (target > 270.0)
        target -= 360.0;
    target = round((target / 360.0) * totalstep);
    return target;
}

double SimpleScope::StepsFromDec(double detarget, TelescopePierSide p, uint32_t totalstep) {
    if (p == PIER_EAST)
        detarget = 180.0 - detarget;
    return StepsFromDegree(detarget, totalstep);
}

/**************************************************************************************
** Client is asking us to slew to a new position
***************************************************************************************/
bool SimpleScope::Goto(double ra, double dec)
{
    double juliandate, lng, lst;

    if ((TrackState == SCOPE_SLEWING) || (TrackState == SCOPE_PARKING) || (TrackState == SCOPE_PARKED)) {
        LOG_WARN("Can not perform goto while goto/park in progress, or scope parked.");
        return false;
    }

    juliandate = getJulianDate();
    lng = getLongitude();
    lst = getLst(juliandate, lng);

    targetRA = ra;
    targetDEC = dec;
    // Calculate hour angle from local sidereal time and right ascension coodinate
    targetHA = lst - ra;
    // Convert hour angle to -12 --> 12 system
    targetHA = rangeHA(targetHA);

    // Calculate target pier side
    if (targetHA > -6) {
        targetPierSide = PIER_WEST;
    } else {
        targetPierSide = PIER_EAST;
    }

    targetRAEncoder = static_cast<int32_t>(StepsFromRA(targetRA, targetPierSide, lst, STEPS_PER_RA_REV));
    targetDEEncoder = static_cast<int32_t>(StepsFromDec(targetDEC, targetPierSide, STEPS_PER_DE_REV));

    char RAStr[64] = {0}, DecStr[64] = {0};

    char cmd[DRIVER_LEN] {0};
    char res[DRIVER_LEN] {0};

    sprintf(cmd, "%c %d %d%c", GOTO, targetRAEncoder, targetDEEncoder, DRIVER_STOP_CHAR);
    sendCommand(cmd, res, DRIVER_LEN, DRIVER_LEN);

    LOGF_DEBUG("Goto response: %s", res);

    if (res[0] == ERROR) return false;

    // Mark state as slewing
    TrackState = SCOPE_SLEWING;
    AxisStatus = SLEWING_TO;

    //if (res[0] != GOTO) return false;

    // Parse the RA/DEC into strings
    fs_sexa(RAStr, targetRA, 2, 3600);
    fs_sexa(DecStr, targetDEC, 2, 3600);
    LOGF_INFO("Starting Goto RA %s DE %s (current RA %g DE %g)", RAStr, DecStr, currentRA, currentDEC);

    // Inform client we are slewing to a new position
    //LOGF_INFO("Slewing to RA: %s - DEC: %s", RAStr, DecStr);

    // Success!
    return true;
}

/**************************************************************************************
** Client is asking us to abort our motion
***************************************************************************************/
bool SimpleScope::Abort()
{
    char cmd[DRIVER_LEN] {0};
    char res[DRIVER_LEN] {0};
    sprintf(cmd, "%c%c", ABORT, DRIVER_STOP_CHAR);
    sendCommand(cmd, res, 0, 0);
    if (res[0] != ERROR) return true;
    return false;
}

INDI_EQ_AXIS SimpleScope::GetRAEncoder(int32_t* steps) {
    char cmd[DRIVER_LEN] {0};
    char res[DRIVER_LEN] {0};
    int32_t RaSteps = 0;
    int axis_num;
    char resCode;

    sprintf(cmd, "%c %d%c", GETAXISSTATUS, AXIS_RA, DRIVER_STOP_CHAR);
    sendCommand(cmd, res, 0, 0);
    sscanf(res, "%c %d %d", &resCode, &RaSteps, &axis_num);
    if (axis_num == AXIS_RA) *steps = RaSteps;
    if (*steps != lastRAStep) lastRAStep = *steps;
    return static_cast<INDI_EQ_AXIS>(axis_num);
}

INDI_EQ_AXIS SimpleScope::GetDEEncoder(int32_t* steps) {
    char cmd[DRIVER_LEN] {0};
    char res[DRIVER_LEN] {0};
    int32_t DeSteps = 0;
    int axis_num;
    char resCode;

    sprintf(cmd, "%c %d%c", GETAXISSTATUS, AXIS_DE, DRIVER_STOP_CHAR);
    sendCommand(cmd, res, 0, 0);
    sscanf(res, "%c %d %d", &resCode, &DeSteps, &axis_num);
    if (axis_num == AXIS_DE) *steps = DeSteps; 
    if (*steps != lastDEStep) lastDEStep = *steps;
    return static_cast<INDI_EQ_AXIS>(axis_num);
}

double SimpleScope::GetRASlew()
{
    ISwitch *sw;
    double rate = 1.0;
    sw          = IUFindOnSwitch(&SlewRateSP);
    if (!strcmp(sw->name, "SLEW_GUIDE"))
        rate = FINE_SLEW_RATE;
    if (!strcmp(sw->name, "SLEW_CENTERING"))
        rate = SLEW_RATE;
    if (!strcmp(sw->name, "SLEW_FIND"))
        rate = SLEW_RATE * 2;
    if (!strcmp(sw->name, "SLEW_MAX"))
        rate = GOTO_RATE;
    else
        rate = *((double *)sw->aux);
    LOGF_DEBUG("RASlewRate %.6f", rate);
    return rate;
}

double SimpleScope::GetDESlew()
{
    ISwitch *sw;
    double rate = 1.0;
    sw          = IUFindOnSwitch(&SlewRateSP);
    if (!strcmp(sw->name, "SLEW_GUIDE"))
        rate = FINE_SLEW_RATE;
    if (!strcmp(sw->name, "SLEW_CENTERING"))
        rate = SLEW_RATE;
    if (!strcmp(sw->name, "SLEW_FIND"))
        rate = SLEW_RATE * 2;
    if (!strcmp(sw->name, "SLEW_MAX"))
        rate = GOTO_RATE;
    else
        rate = *((double *)sw->aux);
    LOGF_DEBUG("DESlewRate %.6f", rate);
    return rate;
}

bool SimpleScope::SetRASlew(double rate) {
    char cmd[DRIVER_LEN] {0};
    char res[DRIVER_LEN] {0};

    double stepperRate = rate / STEPSIZE_RA;

    sprintf(cmd, "%c %d %f%c", SETSLEWRATE, AXIS_RA, stepperRate, DRIVER_STOP_CHAR);
    sendCommand(cmd, res, 0, 0);

    if (res[0] != ERROR)
        return true;
    return false;
}

bool SimpleScope::SetDESlew(double rate) {
    char cmd[DRIVER_LEN] {0};
    char res[DRIVER_LEN] {0};

    double stepperRate = rate / STEPSIZE_DE;

    sprintf(cmd, "%c %d %f%c", SETSLEWRATE, AXIS_DE, stepperRate, DRIVER_STOP_CHAR);
    sendCommand(cmd, res, 0, 0);

    if (res[0] != ERROR)
        return true;
    return false;
}

bool SimpleScope::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) {
    const char *dirStr = (dir == DIRECTION_NORTH) ? "North" : "South";
    double rate        = (dir == DIRECTION_NORTH) ? GetDESlew() : GetDESlew() * -1;
    LOGF_DEBUG("%s %f", dirStr, rate);
    char cmd[DRIVER_LEN] {0};
    char res[DRIVER_LEN] {0};

    switch (command)
    {
        case MOTION_START:
            if ((TrackState == SCOPE_SLEWING) || (TrackState == SCOPE_PARKING) || (TrackState == SCOPE_PARKED))
            {
                LOG_WARN("Can not slew while goto/park in progress, or scope parked.");
                return false;
            }

            LOGF_INFO("Starting %s slew.", dirStr);
            if (DEInverted)
                rate = -rate;
            if (!SetDESlew(rate))
                return false;

            sprintf(cmd, "%c %d%c", MOVE, AXIS_DE, DRIVER_STOP_CHAR);

            sendCommand(cmd, res, 0, 0);

            LOGF_DEBUG("%s", cmd);

            if (res[0] == ERROR)
                return false;

            TrackState = SCOPE_SLEWING;
            break;

        case MOTION_STOP:
            LOGF_INFO("%s Slew stopped", dirStr);

            sprintf(cmd, "%c %d%c", STOP, AXIS_DE, DRIVER_STOP_CHAR);

            sendCommand(cmd, res, 0, 0);

            if (res[0] == ERROR)
                return false;

            if (RememberTrackState == SCOPE_TRACKING)
            {
                LOG_INFO("Restarting DE Tracking...");
                TrackState = SCOPE_TRACKING;
                StartTracking();
            }
            else
                TrackState = SCOPE_IDLE;

            RememberTrackState = TrackState;

            break;
    }
    return true;
}

bool SimpleScope::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) {
    const char *dirStr = (dir == DIRECTION_WEST) ? "West" : "East";
    double rate        = (dir == DIRECTION_EAST) ? GetRASlew() : GetRASlew() * -1;

    char cmd[DRIVER_LEN] {0};
    char res[DRIVER_LEN] {0};

    switch (command)
    {
        case MOTION_START:
            if (TrackState == SCOPE_SLEWING || (TrackState == SCOPE_PARKING) || (TrackState == SCOPE_PARKED))
            {
                LOG_WARN("Can not slew while goto/park in progress, or scope parked.");
                return false;
            }

            LOGF_INFO("Starting %s slew.", dirStr);
            if (DEInverted)
                rate = -rate;
            if (!SetRASlew(rate))
                return false;

            sprintf(cmd, "%c %d%c", MOVE, AXIS_RA, DRIVER_STOP_CHAR);

            LOGF_DEBUG("%s", cmd);

            sendCommand(cmd, res, 0, 0);

            if (res[0] == ERROR)
                return false;

            TrackState = SCOPE_SLEWING;
            break;

        case MOTION_STOP:
            LOGF_INFO("%s Slew stopped", dirStr);

            sprintf(cmd, "%c %d%c", STOP, AXIS_RA, DRIVER_STOP_CHAR);

            sendCommand(cmd, res, 0, 0);

            if (res[0] == ERROR)
                return false;

            if (RememberTrackState == SCOPE_TRACKING)
            {
                LOG_INFO("Restarting DE Tracking...");
                TrackState = SCOPE_TRACKING;
                StartTracking();
            }
            else
                TrackState = SCOPE_IDLE;

            RememberTrackState = TrackState;

            break;
    }
    return true;
}

double SimpleScope::GetRATrackRate()
{
    double rate = 0.0;
    ISwitch *sw;
    sw = IUFindOnSwitch(&TrackModeSP);
    if (!sw)
        return 0.0;
    if (!strcmp(sw->name, "TRACK_SIDEREAL"))
    {
        rate = TRACKRATE_SIDEREAL;
    }
    else if (!strcmp(sw->name, "TRACK_LUNAR"))
    {
        rate = TRACKRATE_LUNAR;
    }
    else if (!strcmp(sw->name, "TRACK_SOLAR"))
    {
        rate = TRACKRATE_SOLAR;
    }
    else if (!strcmp(sw->name, "TRACK_CUSTOM"))
    {
        rate = IUFindNumber(&TrackRateNP, "TRACK_RATE_RA")->value;
    }
    else
        return 0.0;
    if (RAInverted)
        rate = -rate;
    return rate;
}

double SimpleScope::GetDETrackRate()
{
    double rate = 0.0;
    ISwitch *sw;
    sw = IUFindOnSwitch(&TrackModeSP);
    if (!sw)
        return 0.0;
    if (!strcmp(sw->name, "TRACK_SIDEREAL"))
    {
        rate = 0.0;
    }
    else if (!strcmp(sw->name, "TRACK_LUNAR"))
    {
        rate = 0.0;
    }
    else if (!strcmp(sw->name, "TRACK_SOLAR"))
    {
        rate = 0.0;
    }
    else if (!strcmp(sw->name, "TRACK_CUSTOM"))
    {
        rate = IUFindNumber(&TrackRateNP, "TRACK_RATE_DE")->value;
    }
    else
        return 0.0;
    if (DEInverted)
        rate = -rate;
    return rate;
}

bool SimpleScope::SetRaRate(double raRate) {
    char cmd[DRIVER_LEN] {0};
    char res[DRIVER_LEN] {0};
    char resCode;

    double stepRate = StepsFromDegree(raRate, STEPS_PER_RA_REV);

    sprintf(cmd, "%c %f%c", SETTRACKRATE, stepRate, DRIVER_STOP_CHAR);
    sendCommand(cmd, res, 0, 0);
    sscanf(res, "%c", &resCode);

    LOGF_INFO("Setting Tracking Rate - RA=%.6f arcsec/s", raRate);
    if (res[0] != ERROR) return true;
    return false;
}

bool SimpleScope::SetDeRate(double deRate) {
    char cmd[DRIVER_LEN] {0};
    char res[DRIVER_LEN] {0};

    double stepRate = StepsFromDegree(deRate, STEPS_PER_DE_REV);

    sprintf(cmd, "%c %f%c", SETTRACKRATE, stepRate, DRIVER_STOP_CHAR);
    sendCommand(cmd, res, 0, 0);

    LOGF_INFO("Setting Tracking Rate - DE=%.6f arcsec/s", deRate);
    if (res[0] != ERROR) return true;
    return false;
}

bool SimpleScope::SetTrackRate(double raRate, double deRate) {
    double RAStepRate = (CW * raRate / 3600) / STEPSIZE_RA;
    double DEStepRate = (CW * deRate / 3600) / STEPSIZE_DE;
    char cmd[DRIVER_LEN] {0};
    char res[DRIVER_LEN] {0};
    sprintf(cmd, "%c %f %f%c", SETTRACKRATE, RAStepRate, DEStepRate, DRIVER_STOP_CHAR);
    if (res[0] == ERROR) return false;
    LOGF_INFO("Setting Custom Tracking Rates - RA=%.6f  DE=%.6f arcsec/s", raRate, deRate);
    return true;
}

bool SimpleScope::SetTrackMode(uint8_t mode) {
    LOGF_INFO("%d", mode);
    ISwitch *sw;
    sw = IUFindOnSwitch(&TrackModeSP);

    double RArate = GetRATrackRate();
    double DErate = GetDETrackRate();

    RArate = (CW * RArate / 3600.0) / STEPSIZE_RA;
    DErate = (CW * DErate / 3600.0) / STEPSIZE_DE;
    
    char cmd[DRIVER_LEN] {0};
    char res[DRIVER_LEN] {0};
    
    sprintf(cmd, "%c %f %f%c", SETTRACKRATE, RArate, DErate, DRIVER_STOP_CHAR);

    sendCommand(cmd, res, 0, 0);

    LOGF_INFO("Setting Track Mode to '%s', RA=%.6f DE=%.6f", sw->label, RArate, DErate);
    LOGF_DEBUG("Setting Track Mode to '%s', RA=%.6f DE=%.6f", sw->label, RArate, DErate);

    if (res[0] == ERROR)
        return false;
    return true;
}

bool SimpleScope::SetTrackEnabled(bool enabled) {
    if (enabled) {
        //LOGF_INFO("Start Tracking (%s).", IUFindOnSwitch(&TrackModeSP)->label);
        TrackState = SCOPE_TRACKING;
        RememberTrackState = TrackState;
        return StartTracking();
    } else {
        //LOGF_INFO("Stopping Tracking (%s).", IUFindOnSwitch(&TrackModeSP)->label);
        TrackState = SCOPE_IDLE;
        RememberTrackState = TrackState;
        return StopTracking();
    }
}

bool SimpleScope::StartTracking() {
    char cmd[DRIVER_LEN] {0};
    char res[DRIVER_LEN] {0};
    char expRes[DRIVER_LEN] {TRACK};
    sprintf(cmd, "%c%c", TRACK, DRIVER_STOP_CHAR);
    sendCommand(cmd, res, 0, 0);

    if (res[0] == ERROR)
        return false;
    LOGF_INFO("Start Tracking (%s).", IUFindOnSwitch(&TrackModeSP)->label);
    //LOGF_INFO("CMD %s RES %s", cmd, res);
    return true;
}

bool SimpleScope::StopTracking() {
    char cmd[DRIVER_LEN] {0};
    char res[DRIVER_LEN] {0};
    char expRes[DRIVER_LEN] {SETIDLE};
    sprintf(cmd, "%c%c", SETIDLE, DRIVER_STOP_CHAR);
    sendCommand(cmd, res, 0, 0);

    if (res[0] == ERROR)
        return false;
    LOGF_INFO("Stop Tracking (%s).", IUFindOnSwitch(&TrackModeSP)->label);
    //LOGF_INFO("CMD %s RES %s", cmd, res);
    return true;
}

/**************************************************************************************
** Client is asking us to report telescope status
***************************************************************************************/
bool SimpleScope::ReadScopeStatus()
{
    // Time
    double juliandate = 0;
    double lst = 0;
    char hrlst[12] = {0};

    /*const char *datenames[] = { "LST", "JULIANDATE", "UTC" };
    double periods[2];
    const char *periodsnames[] = { "RAPERIOD", "DEPERIOD" };
    double horizvalues[2];
    const char *horiznames[2] = { "AZ", "ALT" };
    double steppervalues[2];
    const char *steppernames[] = { "RAStepsCurrent", "DEStepsCurrent" };
    */

    juliandate = getJulianDate();
    lst        = getLst(juliandate, getLongitude());

    fs_sexa(hrlst, lst, 2, 360000);
    hrlst[11] = '\0';
    //DEBUGF(DBG_SCOPE, "Compute local time: lst=%2.8f (%s) - julian date=%8.8f", lst, hrlst, juliandate);

    /*
    TimeLSTNP.update(&lst, (char **)(datenames), 1);
    TimeLSTNP.setState(IPS_OK);
    TimeLSTNP.apply();

    JulianNP.update(&juliandate, (char **)(datenames + 1), 1);
    JulianNP.setState(IPS_OK);
    JulianNP.apply();
    */

    TelescopePierSide pierSide;
    GetRAEncoder(&currentRAEncoder);
    GetDEEncoder(&currentDEEncoder);
    DEBUGF(DBG_SCOPE, "Current encoders RA=%ld DE=%ld", 
        static_cast<long>(currentRAEncoder), static_cast<long>(currentDEEncoder));
    StepsToRADE(currentRAEncoder, currentDEEncoder, lst, &currentRA, &currentDEC, &currentHA, &pierSide);
    setPierSide(pierSide);

    char CurrentRAString[64] = {0}, CurrentDEString[64] = {0};
    fs_sexa(CurrentRAString, currentRA, 2, 3600);
    fs_sexa(CurrentDEString, currentDEC, 2, 3600);

    /*LOGF_DEBUG("Scope RA (%s) DE (%s) , PierSide (%s)",
                       CurrentRAString,
                       CurrentDEString,
                       pierSide == PIER_EAST ? "East" : (pierSide == PIER_WEST ? "West" : "Unknown"));
    */
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

    // Calculate how much we moved since last time
    da_ra  = GOTO_RATE * dt;
    da_dec = GOTO_RATE * dt;

    /* Process per current state. We check the state of EQUATORIAL_EOD_COORDS_REQUEST and act acoordingly */
    switch (TrackState)
    {
        case SCOPE_SLEWING:
            switch (AxisStatus) {
                case SLEWING_TO:
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
                        setPierSide(targetPierSide);

                        if (RememberTrackState == SCOPE_TRACKING) {
                            ISwitch *sw = IUFindOnSwitch(&TrackModeSP);
                            char *name = sw->name;
                            StartTracking();
                            // Let's set state to TRACKING
                            TrackState = SCOPE_TRACKING;
                            AxisStatus = TRACKING;
                            LOGF_INFO("Telescope slew is complete. Tracking %s...", name);
                        } else {
                            TrackState = SCOPE_IDLE;
                            AxisStatus = IDLE;
                            RememberTrackState = TrackState;
                            LOG_INFO("Telescope slew is complete. Stopping...");
                        }
                    }
                    else {
                        char RATargetStr[64] = {0}, DETargetStr[64] = {0}, HATargetStr[64] = {0};
                        fs_sexa(RATargetStr, targetRA, 2, 3600);
                        fs_sexa(DETargetStr, targetDEC, 2, 3600);
                        fs_sexa(HATargetStr, targetHA, 2, 3600);
                        LOGF_DEBUG("HATarget: %s RATarget: %s DETarget: %s dx_ra %.6f da_ra %.6f dy_de %.6f da_de %.6f", 
                            HATargetStr, RATargetStr, DETargetStr, dx, da_ra, dy, da_dec);
                    }
                    break;
            }
            break;
        case SCOPE_PARKING:
            if (currentRAEncoder == RAParkEncoder && currentDEEncoder == DEParkEncoder) {
                SetParked(true);
            }
            break;
        default:
            break;
    }

    char RAStr[64] = {0}, DecStr[64] = {0}, HAStr[64] = {0};

    // Parse the RA/DEC into strings
    fs_sexa(RAStr, currentRA, 2, 3600);
    fs_sexa(DecStr, currentDEC, 2, 3600);
    fs_sexa(HAStr, currentHA, 2, 3600);

    DEBUGF(DBG_SCOPE, "Current HA: %s Current RA: %s Current DEC: %s Current Pierside %s", HAStr, RAStr, DecStr, 
        pierSide == PIER_EAST ? "East" : (pierSide == PIER_WEST ? "West" : "Unknown"));

    NewRaDec(currentRA, currentDEC);
    return true;
}

void SimpleScope::constructCommand(char *cmd) {
    sprintf(cmd, "%s%c", cmd, DRIVER_STOP_CHAR);
}

bool SimpleScope::sendCommand(const char * cmd, char * res, int cmd_len, int res_len)
{
    int nbytes_written = 0, nbytes_read = 0, rc = -1;

    tcflush(PortFD, TCIOFLUSH);

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