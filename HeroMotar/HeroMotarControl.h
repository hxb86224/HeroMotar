#ifndef CHEROMOTARCONTROL_H
#define CHEROMOTARCONTROL_H
#include "HeroMotarControlManager.h"
#include "HeroMotarTcp.h"
//#include <mutex>
#include <regex>
class CHeroMotarControl : public CHeroMotarControlManager
{
public:
    static CHeroMotarControl * instance();

protected:
    CHeroMotarControl();
    virtual ~CHeroMotarControl();

public:
    virtual int Init(bool enable);
    virtual int getState();
    virtual int doSingleMotion();
    virtual int doSingleMotion2();
    virtual int doSingleMotion3();
    virtual int doMoveL2();
    virtual int doLeftTransverse();
    virtual int doRightTransverse();
    virtual int doLeftVertical();
    virtual int doRightVertical();
    virtual int OnDealMsgInfo(char* pData, unsigned int nLen, unsigned int nSocket);
    virtual int emgStop();
    int doMoveL(S_Move_L sMove_L);
    int multicoorStop();    //停止坐标系内所有轴的运动
    int eStop();            //紧急停止所有轴
    int doSingleMotion(S_Single_Motion sSingleMotion);
    int doMoveLDown(double dDis = 4000.0, bool bFront = true);
    int doMoveLUp(double dDis = 4000.0, bool bFront = true);
    int threadProcLeftVertical(UINT nValue);
    int threadProcRightVertical(UINT nValue);
    int threadProcRightTransverse(UINT nValue);
    int threadProcLeftTransverse(UINT nValue);
    int threadProcPulpOut();
    void threadProcMotar(UINT nParam);
    int emgStop2();
    int doSingleMotion(UINT nLen, int nLogic);
    vector<string> Split(const char* in, const char* delim);
    int doMotar(int nCommand, int nCommandValue);
    int threadProcLeftMove(UINT nValue);
    int threadProcRightMove(UINT nValue);
    bool InitLeftRightData();
    bool InitEndLeftRightData();
    int doPulpOut(WORD nLogic);
    bool HorizontalTransverse(UINT nLen);


private:
    WORD m_nConnectNo;
    S_Single_Motion m_sSingleMotion;
    short m_nActionState;
    S_In_Out m_sIn_Out;
    S_Move_L m_sMove_L;
    CHeroMotarTcp* m_pHeroMotarTcp;
    //mutex g_Mutex;
    bool m_bInitSuccess;
    bool m_bPulpOut;
    bool m_bEstop;
    bool m_bLeftRightMove;
    CRITICAL_SECTION m_criticalSection;
};

#endif // CHEROMOTARCONTROL_H
