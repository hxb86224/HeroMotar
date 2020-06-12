#include "HeroMotarControl.h"
#include <iostream>
CHeroMotarControl::CHeroMotarControl()
{
    m_nConnectNo = 0;
	m_pHeroMotarTcp = nullptr;
	m_nActionState = STATIC_STATE;
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
    char cIP[20] = { "192.168.2.64" };
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
	lock_guard<std::mutex> lock(g_Mutex);
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

	m_sIn_Out.nIn0 = !smc_read_inbit(m_nConnectNo, 5);
	m_sIn_Out.nIn1 = !smc_read_inbit(m_nConnectNo, 6);
	m_sIn_Out.nIn2 = !smc_read_inbit(m_nConnectNo, 7);
	m_sIn_Out.nIn3 = !smc_read_inbit(m_nConnectNo, 8);
	m_sIn_Out.nIn4 = !smc_read_inbit(m_nConnectNo, 9);
	m_sIn_Out.nIn5 = !smc_read_inbit(m_nConnectNo, 10);
	m_sIn_Out.nIn6 = !smc_read_inbit(m_nConnectNo, 11);
	if (m_sIn_Out.nIn6 && (m_nActionState == UP_STATE))
	{
		multicoorStop();
	}
	if (m_sIn_Out.nIn5 && (m_nActionState == DOWN_STATE))
	{
		multicoorStop();
	}

	if ((m_sIn_Out.nIn1 || m_sIn_Out.nIn2) && (m_nActionState == LEFT_STATE))
	{
		emgStop2();
	}

	printf("asdfsfafsd %d\n", m_sIn_Out.nIn6);
	//m_sIn_Out.nOrg = smc_axis_io_status(m_nConnectNo, 0) & 0x10;
	//nRet = smc_check_done_multicoor(m_nConnectNo, m_sMove_L.nCrd);
	//nRet = smc_read_current_speed_unit(m_nConnectNo, 0, &m_sMove_L.dSpeedCurrent);

	//smc_get_position_unit(m_nConnectNo, 0, &m_sMove_L.dXpos);
	//smc_get_position_unit(m_nConnectNo, 1, &m_sMove_L.dYpos);
	//smc_get_position_unit(m_nConnectNo, 2, &m_sMove_L.dZpos);
	//smc_get_position_unit(m_nConnectNo, 3, &m_sMove_L.dUPos);       //获取当前位置

	return nRet;
}

//执行单轴运动
int CHeroMotarControl::doSingleMotion()
{
	short nRet = -1;
	S_Single_Motion sSingleMotion;
	sSingleMotion.nLogic = 1;
	sSingleMotion.nPulse = (600 / PULSE_UNIT);
	m_nActionState = RIGHT_STATE;
	nRet = doSingleMotion(sSingleMotion);
	return nRet;
}

int CHeroMotarControl::doSingleMotion(UINT nLen, int nLogic)
{
	short nRet = -1;
	S_Single_Motion sSingleMotion;
	sSingleMotion.nLogic = nLogic;
	sSingleMotion.nPulse = (nLen / PULSE_UNIT);
	m_nActionState = (nLogic > 0) ? RIGHT_STATE  : LEFT_STATE;
	nRet = doSingleMotion(sSingleMotion);
	return nRet;
}

int CHeroMotarControl::doSingleMotion(S_Single_Motion sSingleMotion)
{
	short nRet = -1;
	if (smc_check_done(m_nConnectNo, sSingleMotion.nAxis) == 0) //已经在运动中
	{
		return nRet;
	}
	nRet = smc_set_profile_unit(m_nConnectNo, sSingleMotion.nAxis, sSingleMotion.dSpeedMin, sSingleMotion.dSpeed, sSingleMotion.dAcc, sSingleMotion.dDec, sSingleMotion.dSpeedStop);//设定单轴运动速度参数	
	if (sSingleMotion.nActionst == 0)
	{
		long nTemp = sSingleMotion.nPulse * sSingleMotion.nLogic;
		nRet = smc_pmove_unit(m_nConnectNo, sSingleMotion.nAxis, sSingleMotion.nPulse * sSingleMotion.nLogic, 0);//相对定长运动
	}
	else
	{
		nRet = smc_vmove(m_nConnectNo, sSingleMotion.nAxis, sSingleMotion.nLogic);	//恒速运动
	}
	return nRet;
}

int CHeroMotarControl::doSingleMotion2()
{
	short nRet = -1;
	S_Single_Motion sSingleMotion;
	sSingleMotion.nLogic = -1;
	sSingleMotion.nPulse = (600 / PULSE_UNIT);
	m_nActionState = LEFT_STATE;
	nRet = doSingleMotion(sSingleMotion);
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
	m_nActionState = STATIC_STATE;
	return eStop();
	//return smc_stop(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS, 1);	//立即停止	
}

int CHeroMotarControl::emgStop2()
{
	m_nActionState = STATIC_STATE;
	return smc_stop(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS, 1);	//立即停止	
}

void CHeroMotarControl::checkLogic()
{
	// TODO: Add your control notification handler code here
	//逻辑正反
	//m_sSingleMotion.bLogic = ~m_sSingleMotion.bLogic;
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
	return doMoveLUp();
}

int CHeroMotarControl::doMoveLUp(double dDis, bool bFront)
{
	S_Move_L sMove_L;
	sMove_L.dSpeedstart = 500;
	sMove_L.dSpeedrun = 1000;
	sMove_L.dTAcc = 0.1;
	sMove_L.dTDec = 0.1;
	m_nActionState = UP_STATE;
	short nRet = 0;
	if (bFront)
	{
		sMove_L.nAxis[0] = FRONT_UP_DOWN_MOTION_AXIS1;
		sMove_L.nAxis[1] = FRONT_UP_DOWN_MOTION_AXIS2;
	}
	else
	{
		sMove_L.nAxis[0] = BACK_UP_DOWN_MOTION_AXIS1;
		sMove_L.nAxis[1] = BACK_UP_DOWN_MOTION_AXIS2;
	}
	sMove_L.nLogic = 1;
	sMove_L.dDis = dDis;
	sMove_L.dDist[0] = (-1) * sMove_L.nLogic * sMove_L.dDis;
	sMove_L.dDist[1] = sMove_L.nLogic * sMove_L.dDis;
	return doMoveL(sMove_L);
}

int CHeroMotarControl::doMoveLDown(double dDis, bool bFront)
{
	S_Move_L sMove_L;
	sMove_L.dSpeedstart = 500;
	sMove_L.dSpeedrun = 1000;
	sMove_L.dTAcc = 0.1;
	sMove_L.dTDec = 0.1;
	m_nActionState = DOWN_STATE;
	short nRet = 0;
	if (bFront)
	{
		sMove_L.nAxis[0] = FRONT_UP_DOWN_MOTION_AXIS1;
		sMove_L.nAxis[1] = FRONT_UP_DOWN_MOTION_AXIS2;
	}
	else
	{
		sMove_L.nAxis[0] = BACK_UP_DOWN_MOTION_AXIS1;
		sMove_L.nAxis[1] = BACK_UP_DOWN_MOTION_AXIS2;
	}
	sMove_L.nLogic = -1;
	sMove_L.dDis = dDis;
	sMove_L.dDist[0] = (-1) * sMove_L.nLogic * sMove_L.dDis;
	sMove_L.dDist[1] = sMove_L.nLogic * sMove_L.dDis;
	doMoveL(sMove_L);
	return nRet;
}

//执行直线插补运动
int CHeroMotarControl::doMoveL(S_Move_L sMove_L)
{
	short nRet = 0;
	nRet = smc_set_vector_profile_unit(m_nConnectNo, sMove_L.nCrd, sMove_L.dSpeedstart, sMove_L.dSpeedrun, sMove_L.dTAcc, sMove_L.dTDec, sMove_L.dSpeedstop);      //设置插补速度
	nRet = smc_set_vector_s_profile(m_nConnectNo, sMove_L.nCrd, 0, 0.01);//设置S段时间
	//WORD axis[2] = { 1,2};
	//double dist[2] = { sMove_L.dXdist, sMove_L.dYdist };
	nRet = smc_line_unit(m_nConnectNo, m_sMove_L.nCrd, 2, sMove_L.nAxis, sMove_L.dDist, 0);
	return nRet;
}

int CHeroMotarControl::multicoorStop()
{
	m_nActionState = STATIC_STATE;
	return smc_stop_multicoor(m_nConnectNo, m_sMove_L.nCrd, 1);//减速停止
}

int CHeroMotarControl::eStop()
{
	m_nActionState = STATIC_STATE;
	return smc_emg_stop(m_nConnectNo);	      //紧急停止
}

int CHeroMotarControl::doLeftTransverse()
{
	//doSingleMotion();
	//smc_write_outbit(m_nConnectNo, 1, 1);

	return 0;
}

int CHeroMotarControl::doRightTransverse()
{
	//doSingleMotion2();
	thread t1(&CHeroMotarControl::threadProcRightTransverse, this, FRONT_UP_DOWN_MOTION_AXIS1);
	t1.detach();
	return 0;
}

int CHeroMotarControl::doLeftVertical()
{
	thread t1(&CHeroMotarControl::threadProcLeftVertical, this, FRONT_UP_DOWN_MOTION_AXIS1);
	t1.detach();
	return 0;
}

int CHeroMotarControl::doRightVertical()
{
	return 0;
}

void CHeroMotarControl::threadProcLeftTransverse(int nAxis)
{
	S_Single_Motion sSingleMotion;
	sSingleMotion.nLogic = 1;
	sSingleMotion.nAxis = PULP_OUT_AXIS;
	sSingleMotion.nActionst = 1;
	doSingleMotion(sSingleMotion);
	int nCount = 0;
	while (1)
	{
		if (nCount >= 200)
		{
			smc_stop(m_nConnectNo, PULP_OUT_AXIS, 1);
			break;
		}
		Sleep(50);
		nCount++;
	}

	doMoveLDown(4000, false);
	short nRet = -1;
	Sleep(100);
	while (1)
	{
		nRet = smc_check_done(m_nConnectNo, nAxis);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}

	doSingleMotion(600, -1);
	Sleep(100);
	short nIn0 = 1;
	while (1)
	{

		nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProcRightVertical 当前状态：静止！");
			break;
		}
		Sleep(50);
	}

	doMoveLUp(80000, false);
	Sleep(100);
	while (1)
	{
		nRet = smc_check_done(m_nConnectNo, nAxis);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}
}

void CHeroMotarControl::threadProcRightTransverse(int nAxis)
{
	doMoveLUp(20000);
	Sleep(100);
	short nRet = -1;
	while (1)
	{
		nRet = smc_check_done(m_nConnectNo, nAxis);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}

	S_Single_Motion sSingleMotion;
	sSingleMotion.nLogic = -1;
	sSingleMotion.nAxis = PULP_OUT_AXIS;
	sSingleMotion.nActionst = 1;
	doSingleMotion(sSingleMotion);
	int nCount = 0;
	while (1)
	{
		if (nCount >= 200)
		{
			smc_stop(m_nConnectNo, PULP_OUT_AXIS, 1);
			break;
		}
		Sleep(50);
		nCount++;
	}

	doMoveLDown(80000);
	Sleep(100);
	while (1)
	{
		nRet = smc_check_done(m_nConnectNo, nAxis);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}

	doSingleMotion(600, 1);
	Sleep(100);
	short nIn0 = 1;
	while (1)
	{
		nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		if (nRet == 1)
		{
			break;
		}
		else
		{
			nIn0 = !smc_read_inbit(m_nConnectNo, 5);
			if (!nIn0)
			{
				emgStop2();
				break;
			}
		}
		Sleep(50);
	}

	doSingleMotion(20, 1);
	Sleep(100);
	while (1)
	{
		nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}

	nIn0 = !smc_read_inbit(m_nConnectNo, 5);
	if (nIn0)
	{
		doSingleMotion(300, 1);
		Sleep(100);
		while (1)
		{
			nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
			if (nRet == 1)
			{
				g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
				break;
			}
			Sleep(50);
		}
	}
	else
	{
		doMoveLDown(80000, false);
		Sleep(100);
		while (1)
		{
			nRet = smc_check_done(m_nConnectNo, BACK_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
			if (nRet == 1)
			{
				g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
				break;
			}
			Sleep(50);
		}

		doSingleMotion(300, 1);
		Sleep(100);
		while (1)
		{
			nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
			if (nRet == 1)
			{
				g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
				break;
			}
			Sleep(50);
		}
	}
}

void CHeroMotarControl::threadProcLeftVertical(int nAxis)
{
	doMoveLUp();
	Sleep(100);
	short nRet = -1;
	while (1)
	{
		
		nRet = smc_check_done(m_nConnectNo, nAxis);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}
	doSingleMotion2();
	Sleep(100);
	while (1)
	{

		nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}
	doMoveLDown(80000);
	Sleep(100);
	while (1)
	{
		nRet = smc_check_done(m_nConnectNo, nAxis);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}

	S_Single_Motion sSingleMotion;
	sSingleMotion.nLogic = -1;
	sSingleMotion.nAxis = PULP_OUT_AXIS;
	sSingleMotion.nActionst = 1;
	doSingleMotion(sSingleMotion);
	int nCount = 0;
	while (1)
	{
		if (nCount >= 200)
		{
			smc_stop(m_nConnectNo, PULP_OUT_AXIS, 1);
			break;
		}
		Sleep(50);
		nCount ++;
	}

	doMoveLUp(80000);
	Sleep(100);
	while (1)
	{
		nRet = smc_check_done(m_nConnectNo, nAxis);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}
	Sleep(200);
	doSingleMotion(10, 1);
	Sleep(100);
	while (1)
	{
		nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}

	doMoveLDown(80000);
	Sleep(100);
	while (1)
	{
		nRet = smc_check_done(m_nConnectNo, nAxis);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}
}

void CHeroMotarControl::threadProcRightVertical(int nAxis)
{
	doMoveLUp();
	Sleep(100);
	short nRet = -1;
	while (1)
	{

		nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProcRightVertical 当前状态：静止！");
			break;
		}
		Sleep(50);
	}
	doSingleMotion(600, 1);
	Sleep(100);
	short nIn0 = 1;
	while (1)
	{

		nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProcRightVertical 当前状态：静止！");
			break;
		}
		else
		{
			nIn0 = !smc_read_inbit(m_nConnectNo, 5);
			if (!nIn0)
			{
				emgStop2();
				break;
			}
		}
		Sleep(50);
	}

	nIn0 = !smc_read_inbit(m_nConnectNo, 5);
	if (nIn0)
	{
		return;
	}

	doMoveLDown(80000, false);
	Sleep(100);
	while (1)
	{
		nRet = smc_check_done(m_nConnectNo, nAxis);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}

	S_Single_Motion sSingleMotion;
	sSingleMotion.nLogic = 1;
	sSingleMotion.nAxis = PULP_OUT_AXIS;
	sSingleMotion.nActionst = 1;
	doSingleMotion(sSingleMotion);
	int nCount = 0;
	while (1)
	{
		if (nCount >= 200)
		{
			smc_stop(m_nConnectNo, PULP_OUT_AXIS, 1);
			break;
		}
		Sleep(50);
		nCount++;
	}

	Sleep(200);
	doSingleMotion(10, 1);
	Sleep(100);
	while (1)
	{
		nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}

	doMoveLUp(80000, false);
	Sleep(100);
	while (1)
	{
		nRet = smc_check_done(m_nConnectNo, nAxis);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}
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