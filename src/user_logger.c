/*
 *****************************************************************************
 *  CarMaker - Version 15.0
 *  Virtual Test Driving Tool
 *
 *  Copyright ©1998-2025 IPG Automotive GmbH. All rights reserved.
 *  www.ipg-automotive.com
 *****************************************************************************
 */

/* NOTE (portfolio):
 * This file shows an example of a minimal logging extension for simulation data.
 * It is provided for educational/portfolio purposes and may require adaptation
 * to your local CarMaker version and project configuration.
 */

#include <Global.h>

#if defined(WIN32)
# include <windows.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ===================== ADDED (minimal) ===================== */
#include <stdio.h>
#include <time.h>
#include <direct.h>          // _mkdir on Windows
#include <errno.h>           // optional, but useful
#include "Car/Car.h"
#include "Traffic.h"
#include "Vehicle/Sensor_Object.h"

/* >>> ADDED: for CarMaker RadarRSI detection access (Vandana-style) <<< */
#include "Vehicle/Sensor_RadarRSI.h"
/* =========================================================== */

#if defined(XENO)
# include <mio.h>
#endif
#include <ioconf.h>

#include <CarMaker.h>
# include <Car/Vehicle_Car.h>

#include <ADASRP.h>

#include "IOVec.h"
#include "User.h"

/* @@PLUGIN-BEGIN-INCLUDE@@ - Automatically generated code - don't edit! */
/* @@PLUGIN-END@@ */

int UserCalcCalledByAppTestRunCalc = 0;

tUser User;

/* ===================== Logger globals ===================== */
static FILE* g_fpObj = NULL;
static FILE* g_fpCar = NULL;

/* RSI detection file handle (disabled below via #if 0) */
static FILE* g_fpRSI = NULL;
/* Used to avoid writing same detections multiple times per cycle */
static int   g_lastRsiTs = -1;

static double a_x = 0.0, a_y = 0.0, a_z = 0.0;
static const double a_thresh = 32.767;

/* ===================== Output directory (portable) ===================== */
/* Use environment variable CM_OUTDIR if set, else default to local folder "SimOutput" */
static const char* g_outDir = NULL;
static const char* kRefSubDir = "reference_odometry_output";

static const char* GetOutDir(void)
{
    if (g_outDir) return g_outDir;

    const char* env = getenv("CM_OUTDIR");
    if (env && env[0] != '\0') {
        g_outDir = env;
    } else {
        g_outDir = "SimOutput";
    }
    return g_outDir;
}

static void GetRefOutDir(char* buf, int bufSize)
{
#if defined(WIN32)
    snprintf(buf, bufSize, "%s\\%s", GetOutDir(), kRefSubDir);
#else
    snprintf(buf, bufSize, "%s/%s", GetOutDir(), kRefSubDir);
#endif
}

static void EnsureOutDir(void)
{
#if defined(WIN32)
    /* Create base output folder (ok if exists) */
    _mkdir(GetOutDir());

    /* Create subfolder for odometry outputs */
    {
        char refDir[1024] = { 0 };
        GetRefOutDir(refDir, (int)sizeof(refDir));
        _mkdir(refDir);
    }
#else
    mkdir(GetOutDir(), 0777);
    {
        char refDir[1024] = { 0 };
        GetRefOutDir(refDir, (int)sizeof(refDir));
        mkdir(refDir, 0777);
    }
#endif
}

static void MakeTimestamp(char* buf, int bufSize)
{
    time_t t = time(NULL);
    struct tm* tm_info = localtime(&t);
    if (tm_info) strftime(buf, bufSize, "%Y%m%d_%H%M%S", tm_info);
    else snprintf(buf, bufSize, "no_time");
}
/* =========================================================== */


/*
 * User_Init_First ()
 */
int
User_Init_First(void)
{
    memset(&User, 0, sizeof(User));
    return 0;
}

/*
 * User_PrintUsage ()
 */
void
User_PrintUsage(char const* Pgm)
{
    LogUsage("\n");
    LogUsage("Usage: %s [options] [testrun]\n", Pgm);
    LogUsage("Options:\n");

#if defined(CM_HIL)
    {
        tIOConfig const* cf;
        char const* defio = IO_GetDefault();
        LogUsage(" -io %-12s Default I/O configuration (%s)\n", "default",
            (defio != NULL && strcmp(defio, "none") != 0) ? defio : "minimal I/O");
        for (cf = IO_GetConfigurations(); cf->Name != NULL; cf++) {
            LogUsage(" -io %-12s %s\n", cf->Name, cf->Description);
        }
    }
#endif
}

/*
 * User_ScanCmdLine ()
 */
char**
User_ScanCmdLine(int argc, char** argv)
{
    char const* Pgm = argv[0];

    IO_SelectDefault("default");

    while (*++argv) {
        if (strcmp(*argv, "-io") == 0 && argv[1] != NULL) {
            if (IO_Select(*++argv) != 0) {
                return NULL;
            }
        }
        else if (strcmp(*argv, "-h") == 0 || strcmp(*argv, "-help") == 0) {
            User_PrintUsage(Pgm);
            SimCore_PrintUsage(Pgm);
            return NULL;
        }
        else if ((*argv)[0] == '-') {
            LogErrF(EC_General, "Unknown option '%s'", *argv);
            return NULL;
        }
        else {
            break;
        }
    }

    return argv;
}

/*
 * User_Init ()
 */
int
User_Init(void)
{
    return 0;
}

int
User_Register(void)
{
    /* @@PLUGIN-BEGIN-REGISTER@@ - Automatically generated code - don't edit! */
    /* @@PLUGIN-END@@ */
    return 0;
}

/*
 * User_DeclQuants ()
 */
void
User_DeclQuants(void)
{
    int i;

    for (i = 0; i < N_USEROUTPUT; i++) {
        char sbuf[32];
        sprintf(sbuf, "UserOut_%02d", i);
        DDefDouble(NULL, sbuf, "", &User.Out[i], DVA_IO_Out);
    }
}

/*
 * User_Param_Add ()
 */
int
User_Param_Add(void)
{
#if defined(CM_HIL)
    if (SimCore.TestRig.ECUParam.Inf == NULL) {
        return -1;
    }
#endif
    return 0;
}

/*
 * User_Param_Get ()
 */
int
User_Param_Get(void)
{
    int rv = 0;

    if (RT_ACTIVE) {
        if (SimCore.TestRig.ECUParam.Inf == NULL) {
            return -1;
        }
        if (IO_Param_Get(SimCore.TestRig.ECUParam.Inf) != 0) {
            rv = -2;
        }
    }

    if (SimCore.TestRig.SimParam.Inf == NULL) {
        return -4;
    }

    return rv;
}

/*
 * User_TestRun_Start_atBegin ()
 */
int
User_TestRun_Start_atBegin(void)
{
    int rv = 0;
    int i;

    for (i = 0; i < N_USEROUTPUT; i++) {
        User.Out[i] = 0.0;
    }

    if (IO_None) {
        return rv;
    }
#if defined(CM_HIL)
    if (FST_New(SimCore.TestRig.ECUParam.Inf) != 0) {
        rv = -6;
    }
#endif

    return rv;
}


/*
 * User_TestRun_Start_atEnd ()
 *
 * opens logger files + headers
 */
int
User_TestRun_Start_atEnd(void)
{
    /* Close previous handles if any (safety) */
    if (g_fpCar) { fclose(g_fpCar); g_fpCar = NULL; }
    if (g_fpObj) { fclose(g_fpObj); g_fpObj = NULL; }
    if (g_fpRSI) { fclose(g_fpRSI); g_fpRSI = NULL; }

    EnsureOutDir();

    char ts[64] = { 0 };
    MakeTimestamp(ts, (int)sizeof(ts));

    char car_path[1024] = { 0 };
    char obj_path[1024] = { 0 };
    /* RSI path (disabled below) */
    /* char rsi_path[1024] = { 0 }; */

    char refDir[1024] = { 0 };
    GetRefOutDir(refDir, (int)sizeof(refDir));

#if defined(WIN32)
    snprintf(car_path, sizeof(car_path),
        "%s\\Car_output_%s.txt", refDir, ts);

    snprintf(obj_path, sizeof(obj_path),
        "%s\\Object_Sensor_output_%s.txt", refDir, ts);
#else
    snprintf(car_path, sizeof(car_path),
        "%s/Car_output_%s.txt", refDir, ts);

    snprintf(obj_path, sizeof(obj_path),
        "%s/Object_Sensor_output_%s.txt", refDir, ts);
#endif

    /* (RSI file disabled) */
    /*
    snprintf(rsi_path, sizeof(rsi_path),
        "%s\\RSI_Detections.txt", GetOutDir());
    remove(rsi_path);
    */

    g_fpCar = fopen(car_path, "w+");
    g_fpObj = fopen(obj_path, "w+");

    /* (RSI file disabled) */
    /*
    g_fpRSI = fopen(rsi_path, "w");
    if (g_fpRSI) {
        fprintf(g_fpRSI,
            "Time_s,SensorIdx,DetIdx,Range_m,Azimuth_rad,Elevation_rad,Vel_mps,Power\n");
        fflush(g_fpRSI);
    }
    */

    /* do not fail if RSI is disabled */
    if (!g_fpCar || !g_fpObj) {
        if (g_fpCar) { fclose(g_fpCar); g_fpCar = NULL; }
        if (g_fpObj) { fclose(g_fpObj); g_fpObj = NULL; }
        if (g_fpRSI) { fclose(g_fpRSI); g_fpRSI = NULL; }
        LogErrStr(EC_General, "Reference odometry logger: failed to open output files.");
        return -1;
    }

    fprintf(g_fpCar,
        "TimeStamp[s], pos_x[m], pos_y[m], pos_z[m], vel_x[m/s], vel_y[m/s], vel_z[m/s], "
        "a_x[m/s^2], a_y[m/s^2], a_z[m/s^2], YawRate[rad/s], GearNo[-]\n");

    fprintf(g_fpObj,
        "TimeStamp[s], Classification[-], TrackingID[-], x[m], y[m], z[m], v_x[m/s], v_y[m/s], v_z[m/s], "
        "yaw[rad], pitch[rad], roll[rad], height[m], length[m], width[m]\n");

    /* reset RSI timestamp gate for this run */
    g_lastRsiTs = -1;

    return 0;
}

/*
 * User_TestRun_Start_StaticCond_Calc ()
 */
int
User_TestRun_Start_StaticCond_Calc(void)
{
    return 0;
}

/*
 * User_TestRun_Start_Finalize ()
 */
int
User_TestRun_Start_Finalize(void)
{
    return 0;
}

/*
 * User_TestRun_RampUp ()
 */
int
User_TestRun_RampUp(double dt)
{
    int IsReady = 1;
    return IsReady;
}

/*
 * User_TestRun_End_First ()
 *
 * close files
 */
int
User_TestRun_End_First(void)
{
    if (g_fpCar) { fclose(g_fpCar); g_fpCar = NULL; }
    if (g_fpObj) { fclose(g_fpObj); g_fpObj = NULL; }
    if (g_fpRSI) { fclose(g_fpRSI); g_fpRSI = NULL; }
    return 0;
}

/*
 * User_TestRun_End ()
 */
int
User_TestRun_End(void)
{
    return 0;
}

/*
 * User_In ()
 */
void
User_In(unsigned const CycleNo)
{
    if (SimCore.State != SCState_Simulate) {
        return;
    }
}

/*
 * User_DrivMan_Calc ()
 */
int
User_DrivMan_Calc(double dt)
{
    if (Vehicle.OperationState != OperState_Driving) {
        return 0;
    }
    return 0;
}

/*
 * User_VehicleControl_Calc ()
 */
int
User_VehicleControl_Calc(double dt)
{
    if (Vehicle.OperationState != OperState_Driving) {
        return 0;
    }
    return 0;
}

/*
 * User_Brake_Calc ()
 */
int
User_Brake_Calc(double dt)
{
    return 0;
}

/*
 * User_Traffic_Calc ()
 */
int
User_Traffic_Calc(double dt)
{
    if (SimCore.State != SCState_Simulate) {
        return 0;
    }
    return 0;
}

/*
 * User_Calc ()
 *
 * write car + object logger + RSI detections (RSI disabled below)
 */
int
User_Calc(double dt)
{
    if (SimCore.Time <= 0.0) return 0;
    if (SimCore.State != SCState_Simulate) return 0;

    /* RSI disabled, so don't require g_fpRSI */
    if (!g_fpCar || !g_fpObj) return 0;

    int traffic_ID = -1;

    a_x = Car.Fr1.a_0[0];
    a_y = Car.Fr1.a_0[1];
    a_z = Car.Fr1.a_0[2];

    if (fabs(a_x) > a_thresh) a_x = copysign(a_thresh, a_x);
    if (fabs(a_y) > a_thresh) a_y = copysign(a_thresh, a_y);
    if (fabs(a_z) > a_thresh) a_z = copysign(a_thresh, a_z);

    fprintf(g_fpCar,
        "%f, %f, %f, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %d\n",
        SimCore.Time,
        Car.Fr1.t_0[0], Car.Fr1.t_0[1], Car.Fr1.t_0[2],
        Car.Fr1.v_0[0], Car.Fr1.v_0[1], Car.Fr1.v_0[2],
        a_x, a_y, a_z,
        Car.YawRate, VehicleControl.GearNo);

    for (int i = 0; i < ObjectSensorCount; i++) {

        if (ObjectSensor[i].relvTarget.ObjId >= 16000000 &&
            ObjectSensor[i].relvTarget.ObjId < 17000000) {

            tTrafficObj* trf = Traffic_GetByObjId(ObjectSensor[i].relvTarget.ObjId);
            traffic_ID = (trf != NULL) ? trf->Cfg.Id : -1;
        }
        else {
            traffic_ID = -1;
        }

        fprintf(g_fpObj,
            "%f, %d, %d, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
            SimCore.Time,
            ObjectSensor[i].relvTarget.ObjId,
            traffic_ID,
            ObjectSensor[i].relvTarget.RefPnt.ds[0],
            ObjectSensor[i].relvTarget.RefPnt.ds[1],
            ObjectSensor[i].relvTarget.RefPnt.ds[2],
            ObjectSensor[i].relvTarget.RefPnt.dv[0],
            ObjectSensor[i].relvTarget.RefPnt.dv[1],
            ObjectSensor[i].relvTarget.RefPnt.dv[2],
            ObjectSensor[i].relvTarget.RefPnt.r_zyx[0],
            ObjectSensor[i].relvTarget.RefPnt.r_zyx[1],
            ObjectSensor[i].relvTarget.RefPnt.r_zyx[2],
            ObjectSensor[i].relvTarget.h,
            ObjectSensor[i].relvTarget.l,
            ObjectSensor[i].relvTarget.w);
    }

    /* ===========================================================
     * CarMaker RadarRSI detections (Vandana-style)
     * DISABLED (as requested)
     * =========================================================== */
#if 0
    if (RadarRSICount > 0) {
        int newTs = (int)(RadarRSI[0].TimeFired * 1000.0); // ms
        if (newTs != g_lastRsiTs) {
            for (int s = 0; s < RadarRSICount; s++) {
                int nd = RadarRSI[s].nDetections;
                for (int k = 0; k < nd; k++) {
                    fprintf(g_fpRSI,
                        "%.6f,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                        SimCore.Time,
                        s,
                        k,
                        RadarRSI[s].DetPoints[k].Coordinates[0], // range
                        RadarRSI[s].DetPoints[k].Coordinates[1], // azimuth
                        RadarRSI[s].DetPoints[k].Coordinates[2], // elevation
                        RadarRSI[s].DetPoints[k].Velocity,
                        RadarRSI[s].DetPoints[k].Power);
                }
            }
            fflush(g_fpRSI);
            g_lastRsiTs = newTs;
        }
    }
#endif

    return 0;
}

/*
 * User_Check_IsIdle ()
 */
int
User_Check_IsIdle(int IsIdle)
{
    double val;

    val = 0.5 * kmh2ms;
    if (Vehicle.v > val || fabs(Vehicle.Wheel[0]->vBelt) > val || fabs(Vehicle.Wheel[1]->vBelt) > val
        || fabs(Vehicle.Wheel[2]->vBelt) > val || fabs(Vehicle.Wheel[3]->vBelt) > val) {
        IsIdle = 0;
    }

    val = 1.0 * deg2rad;
    if (Vehicle.Steering.Ang > val || Vehicle.Steering.Ang < -val) {
        IsIdle = 0;
    }

    return IsIdle;
}

/*
 * User_Out ()
 */
void
User_Out(unsigned const CycleNo)
{
    if (SimCore.State != SCState_Simulate) {
        return;
    }
}

/*
 * User_ApoMsg_Eval ()
 */
int
User_ApoMsg_Eval(int Ch, char* Msg, int len, int who)
{
    if (Ch == ApoCh_CarMaker) {
#if defined(CM_HIL)
        if (FST_ApoMsgEval(Ch, Msg, len) <= 0) {
            return 0;
        }
#endif
    }

    return -1;
}

/*
 * User_ApoMsg_Send ()
 */
void
User_ApoMsg_Send(double T, unsigned const CycleNo)
{
}

/*
 * User_ShutDown ()
 */
int
User_ShutDown(int ShutDownForced)
{
    int IsDown = 0;
    if (1) {
        IsDown = 1;
    }
    return IsDown;
}

/*
 * User_End ()
 */
int
User_End(void)
{
    return 0;
}

/*
 * User_Cleanup ()
 */
void
User_Cleanup(void)
{
    if (g_fpCar) { fclose(g_fpCar); g_fpCar = NULL; }
    if (g_fpObj) { fclose(g_fpObj); g_fpObj = NULL; }

    /* RSI disabled: keep cleanup safe */
    if (g_fpRSI) { fclose(g_fpRSI); g_fpRSI = NULL; }
}
