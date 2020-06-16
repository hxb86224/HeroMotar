#ifndef CHEROMOTARCONTROL_H
#define CHEROMOTARCONTROL_H
#include "HeroMotarControlManager.h"
#include "HeroMotarTcp.h"
#include <mutex>
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
    int decStop();
    virtual int emgStop();
    int writeOutbit(S_In_Out sIO);
    int doMoveL(S_Move_L sMove_L);
    int multicoorStop();    //停止坐标系内所有轴的运动
    int eStop();            //紧急停止所有轴
    int doSingleMotion(S_Single_Motion sSingleMotion);
    int doMoveLDown(double dDis = 4000.0, bool bFront = true);
    int doMoveLUp(double dDis = 4000.0, bool bFront = true);
    void threadProcLeftVertical(UINT nValue);
    void threadProcRightVertical(UINT nValue);
    void threadProcRightTransverse(UINT nValue);
    void threadProcLeftTransverse(UINT nValue);
    void threadProcPulpOut();
    void threadProcMotar(UINT nParam);
    int emgStop2();
    int doSingleMotion(UINT nLen, int nLogic);
    vector<string> Split(const char* in, const char* delim);
    int doMotar(int nCommand, int nCommandValue);
    void threadProcLeftMove(UINT nValue);
    void threadProcRightMove(UINT nValue);
    void InitLeftRightData();
    int doPulpOut(WORD nLogic);
    bool HorizontalTransverse();


private:
    WORD m_nConnectNo;
    S_Single_Motion m_sSingleMotion;
    short m_nActionState;
    S_In_Out m_sIn_Out;
    S_Move_L m_sMove_L;
    CHeroMotarTcp* m_pHeroMotarTcp;
    mutex g_Mutex;
    bool m_bInitSuccess;
    bool m_bPulpOut;
    bool m_bEstop;
};

#endif // CHEROMOTARCONTROL_H
