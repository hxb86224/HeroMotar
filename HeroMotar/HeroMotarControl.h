#ifndef CHEROMOTARCONTROL_H
#define CHEROMOTARCONTROL_H
#include "HeroMotarControlManager.h"
#include "HeroMotarTcp.h"
class CHeroMotarControl : public CHeroMotarControlManager
{
public:
    static CHeroMotarControl * instance();

protected:
    CHeroMotarControl();
    virtual ~CHeroMotarControl();

public:
    virtual int Init(bool enable);
    virtual WORD getConnectState();
    virtual int getState();
    virtual int doSingleMotion();
    virtual int doSingleMotion2();
    virtual int doSingleMotion3();
    virtual int doMoveL2();
    void positionClear();
    int decStop();
    virtual int emgStop();
    void checkLogic();
    void radioAxis(int nType);
    int writeOutbit(S_In_Out sIO);
    int doMoveL(S_Move_L sMove_L);
    int multicoorStop();    //ֹͣ����ϵ����������˶�
    int eStop();            //����ֹͣ������
    int resetPosition();    //���õ�ǰָ��λ�ü�����ֵ

private:
    WORD m_nConnectNo;
    S_Single_Motion m_sSingleMotion;
    S_In_Out m_sIn_Out;
    S_Move_L m_sMove_L;
    CHeroMotarTcp* m_pHeroMotarTcp;
};

#endif // CHEROMOTARCONTROL_H
