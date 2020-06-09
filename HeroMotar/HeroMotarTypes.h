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
using namespace std;
typedef unsigned char			uchar;
typedef unsigned int			uint;
typedef unsigned short			ushort;
typedef unsigned long			ulong;
typedef void *					HANDLE;

typedef struct S_Single_Motion
{
    BOOL	bLogic;
    long	nPulse;
    int		nActionst;
    int		nAxis;
    double	dAcc;
    double	dDec;
    double	dSPara;
    double	dSpeed;
    double	dSpeedMin;
    double  dSpeedStop;
public:
    S_Single_Motion()
    {
        bLogic = TRUE;
        nPulse = 6400;
        nActionst = 0;
        nAxis = 0;
        dAcc = 0.01;
        dDec = 0.01;
        dSPara = 0.0;
        dSpeed = 3000.0;
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
    short   nIn1;
    short   nIn2;
    short   nIn3;
    short   nIn4;
    int     nOrg;           //1 :X轴原点信号:ON 0:X轴原点信号：OFF
public:
    S_In_Out()
    {
        bSvon = FALSE;
        bOut0 = FALSE;
        bOut1 = FALSE;
        bOut2 = FALSE;
        bOut3 = FALSE;
        nIn1 = 0;
        nIn2 = 0;
        nIn3 = 0;
        nIn4 = 0;
        nOrg = 0;
    }

} S_In_Out;

typedef struct S_Move_L
{
    double	dXdist;
    double	dSpeedCurrent;
    double	dSpeedstart;
    double	dSpeedrun;
    double	dSpeedstop;
    double	dUdist;
    double	dUPos;
    double	dXpos;
    double	dYdist;
    double	dYpos;
    double	dZdist;
    int		nAxis;
    double	dZpos;
    double	dTAcc;
    double	dTDec;
    int	    nCrd;
public:
    S_Move_L()
    {
        dXdist = 0.0;
        dSpeedCurrent = 0.0;
        dSpeedstart = 0.0;
        dSpeedrun = 0.0;
        dSpeedstop = 0.0;
        dUdist = 0.0;
        dUPos = 0.0;
        dXpos = 0.0;
        dYdist = 0.0;
        dYpos = 0.0;
        dZdist = 0.0;
        nAxis = 0;
        dZpos = 0.0;
        dTAcc = 0.0;
        dTDec = 0.0;
        nCrd = 0;
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

#endif
