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
        g_Logger.TraceError("CHeroMotarControl::Init 连接失败！");
        return -1;
    }
    g_Logger.TraceInfo("CHeroMotarControl::Init 连接成功！");
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
	nRet = smc_get_position_unit(m_nConnectNo, m_sSingleMotion.nAxis, &position);          //获取当前轴位置
	printf("postion:%f", position);
	cout << position;
	if (nRet != 0)
	{
		g_Logger.TraceError("CHeroMotarControl::getState 可能已经断开连接！");
		return -1;
	}
	g_Logger.TraceInfo("CHeroMotarControl::getState 当前位置：%0.3f！", position);

	double NowSpe = 0.0;
	nRet = smc_read_current_speed_unit(m_nConnectNo, m_sSingleMotion.nAxis, &NowSpe);          //获取当前轴速度
	g_Logger.TraceInfo("CHeroMotarControl::getState 当前速度：%0.3f！", NowSpe);

	nRet = smc_check_done(m_nConnectNo, m_sSingleMotion.nAxis);           //判断当前轴状态
	if (nRet == 1)
	{
		g_Logger.TraceInfo("CHeroMotarControl::getState 当前状态：静止！");
	}
	else
	{
		g_Logger.TraceInfo("CHeroMotarControl::getState 当前状态：运动！");
	}
	WORD nMode = 0;
	nRet = smc_get_axis_run_mode(m_nConnectNo, m_sSingleMotion.nAxis, &nMode);           //判断当前模式状态
	if (nMode == 0)
	{
		g_Logger.TraceInfo("CHeroMotarControl::getState 当前模式：空闲！");
	}
	else if (nMode == 1)
	{
		g_Logger.TraceInfo("CHeroMotarControl::getState 当前模式：定长！");
	}
	else if (nMode == 2)
	{
		g_Logger.TraceInfo("CHeroMotarControl::getState 当前模式：恒速！");
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
	smc_get_position_unit(m_nConnectNo, 3, &m_sMove_L.dUPos);       //获取当前位置

	return nRet;
}

//执行单轴运动
int CHeroMotarControl::doSingleMotion()
{
	//short nRet = smc_pmove_unit(m_nConnectNo, 0, 10000, 0);
	//return nRet;
	short nRet = -1;
	if (smc_check_done(m_nConnectNo, m_sSingleMotion.nAxis) == 0) //已经在运动中
		return nRet;
	//nRet = smc_set_equiv(m_nConnectNo, m_sSingleMotion.nAxis, 1);//设置脉冲当量
	//nRet = smc_set_alm_mode(m_nConnectNo, m_sSingleMotion.nAxis, 0, 0, 0);//设置报警使能，关闭报警
	//nRet = smc_write_sevon_pin(m_nConnectNo, m_sSingleMotion.nAxis, 0);//打开伺服使能
	//nRet = smc_set_pulse_outmode(m_nConnectNo, m_sSingleMotion.nAxis, 0);//设定脉冲模式（此处脉冲模式固定为P+D方向：脉冲+方向）	
	nRet = smc_set_profile_unit(m_nConnectNo, m_sSingleMotion.nAxis, m_sSingleMotion.dSpeedMin, m_sSingleMotion.dSpeed, m_sSingleMotion.dAcc, m_sSingleMotion.dDec, m_sSingleMotion.dSpeedStop);//设定单轴运动速度参数	
	//nRet = smc_set_s_profile(m_nConnectNo, m_sSingleMotion.nAxis, 0, m_sSingleMotion.dSPara);//设定S段时间
	//
	if (m_sSingleMotion.nActionst == 0)
	{
		long nTemp = m_sSingleMotion.nPulse* (m_sSingleMotion.bLogic ? 1 : -1);
		nRet = smc_pmove_unit(m_nConnectNo, m_sSingleMotion.nAxis, nTemp * 10, 0);//相对定长运动
	}
	else
	{
		nRet = smc_vmove(m_nConnectNo, m_sSingleMotion.nAxis, m_sSingleMotion.bLogic ? 1 : 0);	//恒速运动
	}
	return nRet;
}

int CHeroMotarControl::doSingleMotion2()
{
	short nRet = -1;
	if (smc_check_done(m_nConnectNo, m_sSingleMotion.nAxis) == 0) //已经在运动中
		return nRet;
	nRet = smc_set_profile_unit(m_nConnectNo, m_sSingleMotion.nAxis, m_sSingleMotion.dSpeedMin, m_sSingleMotion.dSpeed, m_sSingleMotion.dAcc, m_sSingleMotion.dDec, m_sSingleMotion.dSpeedStop);//设定单轴运动速度参数	
	if (m_sSingleMotion.nActionst == 0)
	{
		long nTemp = -m_sSingleMotion.nPulse * (m_sSingleMotion.bLogic ? 1 : -1);
		nRet = smc_pmove_unit(m_nConnectNo, m_sSingleMotion.nAxis, nTemp * 10, 0);//相对定长运动
	}
	else
	{
		nRet = smc_vmove(m_nConnectNo, m_sSingleMotion.nAxis, m_sSingleMotion.bLogic ? 1 : 0);	//恒速运动
	}
	return nRet;
}

int CHeroMotarControl::doSingleMotion3()
{
	short nRet = -1;
	if (!smc_check_done(m_nConnectNo, m_sSingleMotion.nAxis) == 0) //已经在运动中
		return nRet;
	double dSpeed = m_sSingleMotion.dSpeed * 2;
	smc_change_speed_unit(m_nConnectNo, m_sSingleMotion.nAxis, dSpeed, 0.1);
}

//位置清零
void CHeroMotarControl::positionClear()
{
	// TODO: Add your control notification handler code here
	for (int i = 0; i < 4; i++)
	{
		smc_set_position_unit(m_nConnectNo, i, 0);        //指令位置清零
	}
}

int CHeroMotarControl::decStop()
{
	smc_set_dec_stop_time(m_nConnectNo, m_sSingleMotion.nAxis, m_sSingleMotion.dDec);//设置10ms减速停止时间
	return smc_stop(m_nConnectNo, m_sSingleMotion.nAxis, 0);   //减速停止
}

int CHeroMotarControl::emgStop()
{
	return smc_stop(m_nConnectNo, m_sSingleMotion.nAxis, 1);	//立即停止	
}

void CHeroMotarControl::checkLogic()
{
	// TODO: Add your control notification handler code here
	//逻辑正反
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
	smc_write_outbit(m_nConnectNo, 3, sIO.bOut3);      //设置通用IO端口电平状态
	return smc_write_sevon_pin(m_nConnectNo, 0, !sIO.bSvon);   //设置0号轴伺服端口电平状态	
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
	nRet = smc_set_vector_profile_unit(m_nConnectNo, sMove_L.nCrd, sMove_L.dSpeedstart, sMove_L.dSpeedrun, sMove_L.dTAcc, sMove_L.dTDec, sMove_L.dSpeedstop);      //设置插补速度
	nRet = smc_set_vector_s_profile(m_nConnectNo, sMove_L.nCrd, 0, 0.01);//设置S段时间
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

//执行直线插补运动
int CHeroMotarControl::doMoveL(S_Move_L sMove_L)
{
	short nRet = 0;
	for (int i = 0; i < 4; i++)
	{
		nRet = smc_set_equiv(m_nConnectNo, i, 1);//设置脉冲当量
		nRet = smc_set_alm_mode(m_nConnectNo, i, 0, 0, 0);//设置报警使能，关闭报警
		nRet = smc_write_sevon_pin(m_nConnectNo, i, 0);//打开伺服使能
	}

	nRet = smc_set_vector_profile_unit(m_nConnectNo, m_sMove_L.nCrd, sMove_L.dSpeedstart, sMove_L.dSpeedrun, sMove_L.dTAcc, sMove_L.dTDec, sMove_L.dSpeedstop);      //设置插补速度
	nRet = smc_set_vector_s_profile(m_nConnectNo, m_sMove_L.nCrd, 0, 0.01);//设置S段时间
	WORD axis[2] = { 1,2};
	double dist[2] = { sMove_L.dXdist, sMove_L.dYdist };
	nRet = smc_line_unit(m_nConnectNo, m_sMove_L.nCrd, 2, axis, dist, 0);
	return nRet;
}

int CHeroMotarControl::multicoorStop()
{
	return smc_stop_multicoor(m_nConnectNo, m_sMove_L.nCrd, 0);//减速停止
}

int CHeroMotarControl::eStop()
{
	return smc_emg_stop(m_nConnectNo);	      //紧急停止
}

int CHeroMotarControl::resetPosition()
{
	// TODO: Add your control notification handler code here
	for (int i = 0; i < 4; i++)
	{
		smc_set_position_unit(m_nConnectNo, i, 0.0);    //设置零点
	}
	return 0;
}