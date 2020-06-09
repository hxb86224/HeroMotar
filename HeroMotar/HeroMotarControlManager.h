#ifndef HEROMOTARCONTROLMANAGER_H
#define HEROMOTARCONTROLMANAGER_H
//#include "Logger.h"
//using namespace LOGGER;
//#define g_Logger (*CLogger::instance())
#include "HeroMotarTypes.h"
class CHeroMotarControlManager
{
public:
    static CHeroMotarControlManager* GetInstance();

protected:
    CHeroMotarControlManager() {};
    virtual ~CHeroMotarControlManager() {};
public:
    virtual int Init(bool enable) = 0;
    virtual WORD getConnectState() = 0;
    virtual int getState() = 0;
    virtual int doSingleMotion() = 0;
    virtual int doSingleMotion2() = 0;
    virtual int doSingleMotion3() = 0;
    virtual int emgStop() = 0;
    virtual int doMoveL2() = 0;
};

#define g_HeroMotarControlManager (*CHeroMotarControlManager::GetInstance())
#endif // HEROMOTARCONTROLMANAGER_H
