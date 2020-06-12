#ifndef _HEROMOTAR_TYPES_H_
#define _HEROMOTAR_TYPES_H_

#include <string>
#include "Logger.h"
#include "LTSMC.h"
#include<vector>
using namespace LOGGER;
#define g_Logger (*CLogger::instance())
#define TCP_PORT 12345
#define BUF_LEN 1024
#define PULSE_UNIT (0.0443294)
using namespace std;
typedef unsigned char			uchar;
typedef unsigned int			uint;
typedef unsigned short			ushort;
typedef unsigned long			ulong;
typedef void *					HANDLE;

typedef enum {
    BACK_UP_DOWN_MOTION_AXIS1 = 0,
    FRONT_UP_DOWN_MOTION_AXIS1,
    PULP_OUT_AXIS,
    LEFT_RIGHT_MOTION_AXIS,
    BACK_UP_DOWN_MOTION_AXIS2,
    FRONT_UP_DOWN_MOTION_AXIS2,
} E_AXIS_NO;

typedef struct S_Single_Motion
{
    int	    nLogic;
    long	nPulse;
    int		nActionst;
    int		nAxis;
    double	dAcc;
    double	dDec;
    //double	dSPara;
    double	dSpeed;
    double	dSpeedMin;
    double  dSpeedStop;
public:
    S_Single_Motion()
    {
        nLogic = 1;
        nPulse = 6400;
        nActionst = 0;
        nAxis = LEFT_RIGHT_MOTION_AXIS;
        dAcc = 0.01;
        dDec = 0.01;
        //dSPara = 0.0;
        dSpeed = 500.0;
        dSpeedMin = 100.0;
        dSpeedStop = 200.0;
    }

} S_Single_Motion;


typedef struct S_In_Out
{
    BOOL	bSvon;
    BOOL	bOut0;
    BOOL	bOut1;
    BOOL	bOut2;
    BOOL	bOut3;
    BOOL	bOut4;
    BOOL	bOut5;
    BOOL	bOut6;
    short   nIn0;
    short   nIn1;
    short   nIn2;
    short   nIn3;
    short   nIn4;
    short   nIn5;
    short   nIn6;
    int     nOrg;           //1 :X轴原点信号:ON 0:X轴原点信号：OFF
public:
    S_In_Out()
    {
        bSvon = FALSE;
        bOut0 = FALSE;
        bOut1 = FALSE;
        bOut2 = FALSE;
        bOut3 = FALSE;
        bOut4 = FALSE;
        bOut5 = FALSE;
        bOut6 = FALSE;
        nIn0 = 0;
        nIn1 = 0;
        nIn2 = 0;
        nIn3 = 0;
        nIn4 = 0;
        nIn5 = 0;
        nIn6 = 0;
        nOrg = 0;
    }

} S_In_Out;

typedef struct S_Move_L
{
    int	    nLogic;
    //double	dXdist;
    //double	dSpeedCurrent;
    double	dSpeedstart;
    double	dSpeedrun;
    double	dSpeedstop;
    //double	dUdist;
    //double	dUPos;
    //double	dXpos;
    //double	dYdist;
    //double	dYpos;
    //double	dZdist;
    //int		nAxis;
    //double	dZpos;
    double	dTAcc;
    double	dTDec;
    int	    nCrd;
    double  dDis;
    WORD    nAxis[2];
    double  dDist[2];
public:
    S_Move_L()
    {
        nLogic = 1;
        dSpeedstart = 500.0;
        dSpeedrun = 1000.0;
        dSpeedstop = 200.0;
        dTAcc = 0.1;
        dTDec = 0.1;
        nCrd = 0;
        dDis = 0.0;
    }

} S_Move_L;

typedef struct S_Connection_Socket {
    SOCKET hSocket;
    char cBuf[BUF_LEN];
    int nBytes;
    S_Connection_Socket(SOCKET socket) :hSocket(socket), nBytes(0)//结构体构造函数，hSocket的初始值为socket，nBytes的初始值为0；
    {
        memset(cBuf, 0, sizeof(cBuf));
    }
} S_Connection_Socket;


typedef enum {
    STATIC_STATE = 0,
    UP_STATE,
    DOWN_STATE,
    LEFT_STATE,
    RIGHT_STATE,
} E_ACTION_STATE;




#endif
