#include "HeroMotarControl.h"
#include <iostream>
CHeroMotarControl::CHeroMotarControl()
{
    m_nConnectNo = 0;
	m_pHeroMotarTcp = nullptr;
}

CHeroMotarControl * CHeroMotarControl::instance()
{
    static CHeroMotarControl * _instance = nullptr;
    if( nullptr == _instance)
    {
        _instance = new CHeroMotarControl;
    }
    return _instance;
}

CHeroMotarControl::~CHeroMotarControl()
{
    smc_board_close(m_nConnectNo);
	if (m_pHeroMotarTcp)
	{
		delete m_pHeroMotarTcp;
		m_pHeroMotarTcp = nullptr;
	}
}

int CHeroMotarControl::Init(bool enable)
{
    char cIP[20] = { "192.168.5.11" };
    int nRet = smc_board_init(m_nConnectNo, 2, cIP, 115200);
    if(nRet != 0){
        g_Logger.TraceError("CHeroMotarControl::Init ����ʧ�ܣ�");
        return -1;
    }
    g_Logger.TraceInfo("CHeroMotarControl::Init ���ӳɹ���");
	//doSingleMotion();
	//m_pHeroMotarTcp = new CHeroMotarTcp(12345);
	//m_pHeroMotarTcp->createListen();
    return 0;
}

WORD CHeroMotarControl::getConnectState()
{
    return m_nConnectNo;
}

int CHeroMotarControl::getState()
{
	short nRet = -1;

	double position = 0.0;
	nRet = smc_get_position_unit(m_nConnectNo, m_sSingleMotion.nAxis, &position);          //��ȡ��ǰ��λ��
	printf("postion:%f", position);
	cout << position;
	if (nRet != 0)
	{
		g_Logger.TraceError("CHeroMotarControl::getState �����Ѿ��Ͽ����ӣ�");
		return -1;
	}
	g_Logger.TraceInfo("CHeroMotarControl::getState ��ǰλ�ã�%0.3f��", position);

	double NowSpe = 0.0;
	nRet = smc_read_current_speed_unit(m_nConnectNo, m_sSingleMotion.nAxis, &NowSpe);          //��ȡ��ǰ���ٶ�
	g_Logger.TraceInfo("CHeroMotarControl::getState ��ǰ�ٶȣ�%0.3f��", NowSpe);

	nRet = smc_check_done(m_nConnectNo, m_sSingleMotion.nAxis);           //�жϵ�ǰ��״̬
	if (nRet == 1)
	{
		g_Logger.TraceInfo("CHeroMotarControl::getState ��ǰ״̬����ֹ��");
	}
	else
	{
		g_Logger.TraceInfo("CHeroMotarControl::getState ��ǰ״̬���˶���");
	}
	WORD nMode = 0;
	nRet = smc_get_axis_run_mode(m_nConnectNo, m_sSingleMotion.nAxis, &nMode);           //�жϵ�ǰģʽ״̬
	if (nMode == 0)
	{
		g_Logger.TraceInfo("CHeroMotarControl::getState ��ǰģʽ�����У�");
	}
	else if (nMode == 1)
	{
		g_Logger.TraceInfo("CHeroMotarControl::getState ��ǰģʽ��������");
	}
	else if (nMode == 2)
	{
		g_Logger.TraceInfo("CHeroMotarControl::getState ��ǰģʽ�����٣�");
	}

	m_sIn_Out.nIn1 = smc_read_inbit(m_nConnectNo, 0);
	m_sIn_Out.nIn2 = smc_read_inbit(m_nConnectNo, 1);
	m_sIn_Out.nIn3 = smc_read_inbit(m_nConnectNo, 2);
	m_sIn_Out.nIn4 = smc_read_inbit(m_nConnectNo, 3);
	m_sIn_Out.nOrg = smc_axis_io_status(m_nConnectNo, 0) & 0x10;


	nRet = smc_check_done_multicoor(m_nConnectNo, m_sMove_L.nCrd);
	nRet = smc_read_current_speed_unit(m_nConnectNo, 0, &m_sMove_L.dSpeedCurrent);

	smc_get_position_unit(m_nConnectNo, 0, &m_sMove_L.dXpos);
	smc_get_position_unit(m_nConnectNo, 1, &m_sMove_L.dYpos);
	smc_get_position_unit(m_nConnectNo, 2, &m_sMove_L.dZpos);
	smc_get_position_unit(m_nConnectNo, 3, &m_sMove_L.dUPos);       //��ȡ��ǰλ��

	return nRet;
}

//ִ�е����˶�
int CHeroMotarControl::doSingleMotion()
{
	//short nRet = smc_pmove_unit(m_nConnectNo, 0, 10000, 0);
	//return nRet;
	short nRet = -1;
	if (smc_check_done(m_nConnectNo, m_sSingleMotion.nAxis) == 0) //�Ѿ����˶���
		return nRet;
	//nRet = smc_set_equiv(m_nConnectNo, m_sSingleMotion.nAxis, 1);//�������嵱��
	//nRet = smc_set_alm_mode(m_nConnectNo, m_sSingleMotion.nAxis, 0, 0, 0);//���ñ���ʹ�ܣ��رձ���
	//nRet = smc_write_sevon_pin(m_nConnectNo, m_sSingleMotion.nAxis, 0);//���ŷ�ʹ��
	//nRet = smc_set_pulse_outmode(m_nConnectNo, m_sSingleMotion.nAxis, 0);//�趨����ģʽ���˴�����ģʽ�̶�ΪP+D��������+����	
	nRet = smc_set_profile_unit(m_nConnectNo, m_sSingleMotion.nAxis, m_sSingleMotion.dSpeedMin, m_sSingleMotion.dSpeed, m_sSingleMotion.dAcc, m_sSingleMotion.dDec, m_sSingleMotion.dSpeedStop);//�趨�����˶��ٶȲ���	
	//nRet = smc_set_s_profile(m_nConnectNo, m_sSingleMotion.nAxis, 0, m_sSingleMotion.dSPara);//�趨S��ʱ��
	//
	if (m_sSingleMotion.nActionst == 0)
	{
		long nTemp = m_sSingleMotion.nPulse* (m_sSingleMotion.bLogic ? 1 : -1);
		nRet = smc_pmove_unit(m_nConnectNo, m_sSingleMotion.nAxis, nTemp * 10, 0);//��Զ����˶�
	}
	else
	{
		nRet = smc_vmove(m_nConnectNo, m_sSingleMotion.nAxis, m_sSingleMotion.bLogic ? 1 : 0);	//�����˶�
	}
	return nRet;
}

int CHeroMotarControl::doSingleMotion2()
{
	short nRet = -1;
	if (smc_check_done(m_nConnectNo, m_sSingleMotion.nAxis) == 0) //�Ѿ����˶���
		return nRet;
	nRet = smc_set_profile_unit(m_nConnectNo, m_sSingleMotion.nAxis, m_sSingleMotion.dSpeedMin, m_sSingleMotion.dSpeed, m_sSingleMotion.dAcc, m_sSingleMotion.dDec, m_sSingleMotion.dSpeedStop);//�趨�����˶��ٶȲ���	
	if (m_sSingleMotion.nActionst == 0)
	{
		long nTemp = -m_sSingleMotion.nPulse * (m_sSingleMotion.bLogic ? 1 : -1);
		nRet = smc_pmove_unit(m_nConnectNo, m_sSingleMotion.nAxis, nTemp * 10, 0);//��Զ����˶�
	}
	else
	{
		nRet = smc_vmove(m_nConnectNo, m_sSingleMotion.nAxis, m_sSingleMotion.bLogic ? 1 : 0);	//�����˶�
	}
	return nRet;
}

int CHeroMotarControl::doSingleMotion3()
{
	short nRet = -1;
	if (!smc_check_done(m_nConnectNo, m_sSingleMotion.nAxis) == 0) //�Ѿ����˶���
		return nRet;
	double dSpeed = m_sSingleMotion.dSpeed * 2;
	smc_change_speed_unit(m_nConnectNo, m_sSingleMotion.nAxis, dSpeed, 0.1);
}

//λ������
void CHeroMotarControl::positionClear()
{
	// TODO: Add your control notification handler code here
	for (int i = 0; i < 4; i++)
	{
		smc_set_position_unit(m_nConnectNo, i, 0);        //ָ��λ������
	}
}

int CHeroMotarControl::decStop()
{
	smc_set_dec_stop_time(m_nConnectNo, m_sSingleMotion.nAxis, m_sSingleMotion.dDec);//����10ms����ֹͣʱ��
	return smc_stop(m_nConnectNo, m_sSingleMotion.nAxis, 0);   //����ֹͣ
}

int CHeroMotarControl::emgStop()
{
	return smc_stop(m_nConnectNo, m_sSingleMotion.nAxis, 1);	//����ֹͣ	
}

void CHeroMotarControl::checkLogic()
{
	// TODO: Add your control notification handler code here
	//�߼�����
	m_sSingleMotion.bLogic = ~m_sSingleMotion.bLogic;
}

void CHeroMotarControl::radioAxis(int nType)
{
	// TODO: Add your control notification handler code here
	m_sSingleMotion.nAxis = nType;
}

int CHeroMotarControl::writeOutbit(S_In_Out sIO)
{
	smc_write_outbit(m_nConnectNo, 0, sIO.bOut0);
	smc_write_outbit(m_nConnectNo, 1, sIO.bOut1);
	smc_write_outbit(m_nConnectNo, 2, sIO.bOut2);
	smc_write_outbit(m_nConnectNo, 3, sIO.bOut3);      //����ͨ��IO�˿ڵ�ƽ״̬
	return smc_write_sevon_pin(m_nConnectNo, 0, !sIO.bSvon);   //����0�����ŷ��˿ڵ�ƽ״̬	
}

int CHeroMotarControl::doMoveL2()
{
	S_Move_L sMove_L;

	sMove_L.dSpeedstart = 500;
	sMove_L.dSpeedrun = 1000;
	sMove_L.dTAcc = 0.1;
	sMove_L.dTDec = 0.1;
	sMove_L.dXdist = 5000;
	sMove_L.dYdist = 6000;
	short nRet = 0;
	nRet = smc_set_vector_profile_unit(m_nConnectNo, sMove_L.nCrd, sMove_L.dSpeedstart, sMove_L.dSpeedrun, sMove_L.dTAcc, sMove_L.dTDec, sMove_L.dSpeedstop);      //���ò岹�ٶ�
	nRet = smc_set_vector_s_profile(m_nConnectNo, sMove_L.nCrd, 0, 0.01);//����S��ʱ��
	WORD axis[2] = { 1,2 };
	double dist[2] = { 1000, 1000 };
	nRet = smc_line_unit(m_nConnectNo, sMove_L.nCrd, 2, axis, dist, 0);

	//axis[0] = 1;
	//axis[1] = 2;
	//dist[0] = 1000;
	//dist[1] = 1000;
	//smc_set_vector_profile_unit(0, 0, 100, 1000, 0.1, 0.1, 0);
	//smc_set_vector_s_profile(0, 0, 0, 0.01);
	//nRet = smc_line_unit(0, 0, 2, axis, dist, 0);
	return nRet;
}

//ִ��ֱ�߲岹�˶�
int CHeroMotarControl::doMoveL(S_Move_L sMove_L)
{
	short nRet = 0;
	for (int i = 0; i < 4; i++)
	{
		nRet = smc_set_equiv(m_nConnectNo, i, 1);//�������嵱��
		nRet = smc_set_alm_mode(m_nConnectNo, i, 0, 0, 0);//���ñ���ʹ�ܣ��رձ���
		nRet = smc_write_sevon_pin(m_nConnectNo, i, 0);//���ŷ�ʹ��
	}

	nRet = smc_set_vector_profile_unit(m_nConnectNo, m_sMove_L.nCrd, sMove_L.dSpeedstart, sMove_L.dSpeedrun, sMove_L.dTAcc, sMove_L.dTDec, sMove_L.dSpeedstop);      //���ò岹�ٶ�
	nRet = smc_set_vector_s_profile(m_nConnectNo, m_sMove_L.nCrd, 0, 0.01);//����S��ʱ��
	WORD axis[2] = { 1,2};
	double dist[2] = { sMove_L.dXdist, sMove_L.dYdist };
	nRet = smc_line_unit(m_nConnectNo, m_sMove_L.nCrd, 2, axis, dist, 0);
	return nRet;
}

int CHeroMotarControl::multicoorStop()
{
	return smc_stop_multicoor(m_nConnectNo, m_sMove_L.nCrd, 0);//����ֹͣ
}

int CHeroMotarControl::eStop()
{
	return smc_emg_stop(m_nConnectNo);	      //����ֹͣ
}

int CHeroMotarControl::resetPosition()
{
	// TODO: Add your control notification handler code here
	for (int i = 0; i < 4; i++)
	{
		smc_set_position_unit(m_nConnectNo, i, 0.0);    //�������
	}
	return 0;
}