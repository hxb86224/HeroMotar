#include "HeroMotarControl.h"
#include <iostream>

int OnDealMsg(char* pData, unsigned int nLen, unsigned int nSocket)
{
	g_HeroMotarControlManager.OnDealMsgInfo(pData, nLen, nSocket);
	return 0;
}

CHeroMotarControl::CHeroMotarControl()
{
    m_nConnectNo = 0;
	m_pHeroMotarTcp = nullptr;
	m_nActionState = STATIC_STATE;
	m_bInitSuccess = false;
	m_bPulpOut = false;
	m_bEstop = false;
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
		m_bInitSuccess = false;
        return -1;
    }
	m_bInitSuccess = true;
    g_Logger.TraceInfo("CHeroMotarControl::Init 连接成功！");
	//m_pHeroMotarTcp = new CHeroMotarTcp(12345);
	//m_pHeroMotarTcp->createListen();
	//m_pHeroMotarTcp->SetDealMsgCallback(OnDealMsg);
    return 0;
}

vector<string> CHeroMotarControl::Split(const char* in, const char* delim)
{
	regex re{ delim };
	return vector<string> {
		cregex_token_iterator(in, in + strlen(in), re, -1),
			cregex_token_iterator()
	};
}

int CHeroMotarControl::OnDealMsgInfo(char* pData, unsigned int nLen, unsigned int nSocket)
{
	int nRet = 0;
	S_Msg_Info* pMsg_Info = new S_Msg_Info(nSocket);
	memcpy(pMsg_Info->cBuf, pData, nLen);
	pMsg_Info->nBytes = nLen;
	thread t1(&CHeroMotarControl::threadProcMotar, this, (UINT)pMsg_Info);
	t1.detach();
	return nRet;
}

int CHeroMotarControl::doMotar(int nCommand, int nCommandValue)
{
	int nRet = 0;
	switch (nCommand)
	{
	case LEFT_TRANSVERSE:
		threadProcLeftTransverse(nCommandValue);
		break;

	case RIGHT_TRANSVERSE:
		threadProcRightTransverse(nCommandValue);
		break;

	case LEFT_VERTICAL:
		threadProcLeftVertical(nCommandValue);
		break;

	case RIGHT_VERTICAL:
		threadProcRightVertical(nCommandValue);
		break;

	case LEFT_MOVE:
		threadProcLeftMove(nCommandValue);
		break;

	case RIGHT_MOVE:
		threadProcRightMove(nCommandValue);
		break;

	default:
		break;
	}
	return nRet;
}

int CHeroMotarControl::getState()
{
	lock_guard<std::mutex> lock(g_Mutex);
	short nRet = -1;
	if (!m_bInitSuccess) {
		return nRet;
	}
	//double position = 0.0;
	//nRet = smc_get_position_unit(m_nConnectNo, m_sSingleMotion.nAxis, &position);          //获取当前轴位置
	//printf("postion:%f", position);
	//cout << position;
	//if (nRet != 0)
	//{
	//	//g_Logger.TraceError("CHeroMotarControl::getState 可能已经断开连接！");
	//	return -1;
	//}
	//g_Logger.TraceInfo("CHeroMotarControl::getState 当前位置：%0.3f！", position);

	//double NowSpe = 0.0;
	//nRet = smc_read_current_speed_unit(m_nConnectNo, m_sSingleMotion.nAxis, &NowSpe);          //获取当前轴速度
	//g_Logger.TraceInfo("CHeroMotarControl::getState 当前速度：%0.3f！", NowSpe);

	//nRet = smc_check_done(m_nConnectNo, m_sSingleMotion.nAxis);           //判断当前轴状态
	//if (nRet == 1)
	//{
	//	g_Logger.TraceInfo("CHeroMotarControl::getState 当前状态：静止！");
	//}
	//else
	//{
	//	g_Logger.TraceInfo("CHeroMotarControl::getState 当前状态：运动！");
	//}
	//WORD nMode = 0;
	//nRet = smc_get_axis_run_mode(m_nConnectNo, m_sSingleMotion.nAxis, &nMode);           //判断当前模式状态
	//if (nMode == 0)
	//{
	//	g_Logger.TraceInfo("CHeroMotarControl::getState 当前模式：空闲！");
	//}
	//else if (nMode == 1)
	//{
	//	g_Logger.TraceInfo("CHeroMotarControl::getState 当前模式：定长！");
	//}
	//else if (nMode == 2)
	//{
	//	g_Logger.TraceInfo("CHeroMotarControl::getState 当前模式：恒速！");
	//}

	//m_sIn_Out.nIn0 = !smc_read_inbit(m_nConnectNo, 5);
	m_sIn_Out.nIn1 = !smc_read_inbit(m_nConnectNo, 6);
	m_sIn_Out.nIn2 = !smc_read_inbit(m_nConnectNo, 7);
	m_sIn_Out.nIn3 = !smc_read_inbit(m_nConnectNo, 8);
	m_sIn_Out.nIn4 = !smc_read_inbit(m_nConnectNo, 9);
	m_sIn_Out.nIn5 = !smc_read_inbit(m_nConnectNo, 10);
	m_sIn_Out.nIn6 = !smc_read_inbit(m_nConnectNo, 11);
	if (m_sIn_Out.nIn6 && (m_nActionState == LEFT_UP_STATE))
	{
		multicoorStop();
	}
	if (m_sIn_Out.nIn5 && (m_nActionState == LEFT_DOWN_STATE))
	{
		multicoorStop();
	}
	if (m_sIn_Out.nIn4 && (m_nActionState == RIGHT_UP_STATE))
	{
		multicoorStop();
	}
	if (m_sIn_Out.nIn3 && (m_nActionState == RIGHT_DOWN_STATE))
	{
		multicoorStop();
	}

	if (m_sIn_Out.nIn1 && (m_nActionState == LEFT_STATE))
	{
		g_Logger.TraceInfo("CHeroMotarControl::getState: %d！", m_nActionState);
		if (m_sIn_Out.nIn2) 
		{
			emgStop2();
			
		}
		else
		{
			doSingleMotion3();
		}
	}
	else if (m_sIn_Out.nIn2 && (m_nActionState == LEFT_STATE))
	{
		g_Logger.TraceInfo("CHeroMotarControl::getState: %d！", m_nActionState);
		if (m_sIn_Out.nIn1)
		{
			emgStop2();

		}
		else
		{
			doSingleMotion3();
		}
	}

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
	//S_Single_Motion sSingleMotion;
	//sSingleMotion.nLogic = 1;
	//sSingleMotion.nPulse = (600 / PULSE_UNIT);
	//m_nActionState = RIGHT_STATE;
	//nRet = doSingleMotion(sSingleMotion);
	//threadProcRightMove(600);
	thread t1(&CHeroMotarControl::threadProcRightMove, this, 600);
	t1.detach();
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

int CHeroMotarControl::doPulpOut(WORD nLogic)
{
	short nRet = -1;
	S_Single_Motion sSingleMotion;
	sSingleMotion.nAxis = PULP_OUT_AXIS;
	sSingleMotion.dSpeedMin = 1000.0;
	sSingleMotion.dSpeed = 4000.0;
	sSingleMotion.dSpeedStop = 1000.0;
	if (smc_check_done(m_nConnectNo, sSingleMotion.nAxis) == 0) //已经在运动中
	{
		return nRet;
	}
	nRet = smc_set_profile_unit(m_nConnectNo, sSingleMotion.nAxis, sSingleMotion.dSpeedMin, sSingleMotion.dSpeed, sSingleMotion.dAcc, sSingleMotion.dDec, sSingleMotion.dSpeedStop);//设定单轴运动速度参数	
	nRet = smc_vmove(m_nConnectNo, sSingleMotion.nAxis, nLogic);	//恒速运动
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
	//S_Single_Motion sSingleMotion;
	//sSingleMotion.nLogic = -1;
	//sSingleMotion.nPulse = (600 / PULSE_UNIT);
	//m_nActionState = LEFT_STATE;
	//nRet = doSingleMotion(sSingleMotion);
	//threadProcLeftMove(600);
	thread t1(&CHeroMotarControl::threadProcLeftMove, this, 600);
	t1.detach();
	return nRet;
}

int CHeroMotarControl::doSingleMotion3()
{
	short nRet = -1;
	if (smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS) == 1) //
		return nRet;
	double dSpeed = m_sSingleMotion.dSpeed / 50;
	smc_change_speed_unit(m_nConnectNo, m_sSingleMotion.nAxis, dSpeed, 0.1);
}

int CHeroMotarControl::decStop()
{
	smc_set_dec_stop_time(m_nConnectNo, m_sSingleMotion.nAxis, m_sSingleMotion.dDec);//设置10ms减速停止时间
	return smc_stop(m_nConnectNo, m_sSingleMotion.nAxis, 0);   //减速停止
}

int CHeroMotarControl::emgStop()
{
	m_nActionState = STATIC_STATE;
	m_bEstop = true;
	return eStop();
	//return smc_stop(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS, 1);	//立即停止	
}

int CHeroMotarControl::emgStop2()
{
	m_nActionState = STATIC_STATE;
	return smc_stop(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS, 1);	//立即停止	
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
	return doMoveLUp(5000, true);
	//return doMoveLDown(1000, true);
}

int CHeroMotarControl::doMoveLUp(double dDis, bool bFront)
{
	S_Move_L sMove_L;
	sMove_L.dTAcc = 0.1;
	sMove_L.dTDec = 0.1;
	sMove_L.nLogic = 1;
	sMove_L.dDis = dDis;
	short nRet = 0;
	if (bFront)
	{
		sMove_L.nAxis[0] = FRONT_UP_DOWN_MOTION_AXIS1;
		sMove_L.nAxis[1] = FRONT_UP_DOWN_MOTION_AXIS2;
		sMove_L.dDist[0] = (-1) * sMove_L.nLogic * sMove_L.dDis;
		sMove_L.dDist[1] = sMove_L.nLogic * sMove_L.dDis;
		m_nActionState = LEFT_UP_STATE;
	}
	else
	{
		sMove_L.nAxis[0] = BACK_UP_DOWN_MOTION_AXIS1;
		sMove_L.nAxis[1] = BACK_UP_DOWN_MOTION_AXIS2;
		sMove_L.dDist[0] = (-1) * sMove_L.nLogic * sMove_L.dDis;
		sMove_L.dDist[1] = (-1) * sMove_L.nLogic * sMove_L.dDis;
		m_nActionState = RIGHT_UP_STATE;
	}

	//sMove_L.dDist[0] = (-1) * sMove_L.nLogic * sMove_L.dDis;
	//sMove_L.dDist[1] = sMove_L.nLogic * sMove_L.dDis;
	return doMoveL(sMove_L);
}

int CHeroMotarControl::doMoveLDown(double dDis, bool bFront)
{
	S_Move_L sMove_L;
	sMove_L.dTAcc = 0.1;
	sMove_L.dTDec = 0.1;
	sMove_L.nLogic = -1;
	sMove_L.dDis = dDis;
	sMove_L.dSpeedrun = 6000.0;
	short nRet = 0;
	if (bFront)
	{
		sMove_L.nAxis[0] = FRONT_UP_DOWN_MOTION_AXIS1;
		sMove_L.nAxis[1] = FRONT_UP_DOWN_MOTION_AXIS2;
		sMove_L.dDist[0] = (-1) * sMove_L.nLogic * sMove_L.dDis;
		sMove_L.dDist[1] = sMove_L.nLogic * sMove_L.dDis;
		m_nActionState = LEFT_DOWN_STATE;
	}
	else
	{
		sMove_L.nAxis[0] = BACK_UP_DOWN_MOTION_AXIS1;
		sMove_L.nAxis[1] = BACK_UP_DOWN_MOTION_AXIS2;
		sMove_L.dDist[0] = (-1) * sMove_L.nLogic * sMove_L.dDis;
		sMove_L.dDist[1] = (-1) * sMove_L.nLogic * sMove_L.dDis;
		m_nActionState = RIGHT_DOWN_STATE;
	}
	nRet = doMoveL(sMove_L);
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
	thread t1(&CHeroMotarControl::threadProcLeftTransverse, this, FRONT_UP_DOWN_MOTION_AXIS1);
	t1.detach();
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
	thread t1(&CHeroMotarControl::threadProcRightVertical, this, BACK_UP_DOWN_MOTION_AXIS1);
	t1.detach();
	return 0;
}

void CHeroMotarControl::threadProcLeftTransverse(UINT nValue)
{
	InitLeftRightData();
	short nRet = -1;
	doMoveLUp(6000, true);
	Sleep(100);
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
		nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}

	doPulpOut(1);
	int nCount = 0;
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
		if (nCount >= HORIZONTAL_TRANSVERSE_600)
		{
			smc_stop(m_nConnectNo, PULP_OUT_AXIS, 1);
			break;
		}
		Sleep(50);
		nCount++;
	}

	doMoveLDown(3000, false);
	//short nRet = -1;
	Sleep(100);
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
		nRet = smc_check_done(m_nConnectNo, BACK_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
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
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
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
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
		nRet = smc_check_done(m_nConnectNo, BACK_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}
}

bool CHeroMotarControl::HorizontalTransverse()
{
	doMoveLUp(14000, true);
	Sleep(100);
	short nRet = -1;
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return false;
		}
		nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}

	doPulpOut(0);
	int nCount = 0;
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return false;
		}
		if (nCount >= HORIZONTAL_TRANSVERSE_300)
		{
			smc_stop(m_nConnectNo, PULP_OUT_AXIS, 1);
			break;
		}
		Sleep(50);
		nCount++;
	}

	Sleep(3000);

	doMoveLDown(80000, true);
	Sleep(100);
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return false;
		}
		nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}

	doSingleMotion(300, 1);
	Sleep(100);
	short nIn0 = 1;
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return false;
		}
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
	return true;
}

void CHeroMotarControl::threadProcRightTransverse(UINT nValue)
{
	InitLeftRightData();
	if (!HorizontalTransverse())
	{
		return;
	}
	
	if (!HorizontalTransverse())
	{
		return;
	}

	short nRet = -1;
	short nIn0 = !smc_read_inbit(m_nConnectNo, 5);
	if (nIn0)
	{
		return;
	}
	doSingleMotion(20, 1);
	Sleep(100);
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
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
			if (m_bEstop)
			{
				eStop();
				m_bEstop = false;
				return;
			}
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
			if (m_bEstop)
			{
				eStop();
				m_bEstop = false;
				return;
			}
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
			if (m_bEstop)
			{
				eStop();
				m_bEstop = false;
				return;
			}
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

void CHeroMotarControl::threadProcLeftVertical(UINT nValue)
{
	InitLeftRightData();
	short nRet = -1;
	doMoveLUp(6000, true);
	Sleep(100);
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
		nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}

	//doSingleMotion(600, -1);
	S_Single_Motion sLeftMotion;
	sLeftMotion.nLogic = 0;
	sLeftMotion.nAxis = LEFT_RIGHT_MOTION_AXIS;
	sLeftMotion.nActionst = 1;
	m_nActionState = LEFT_STATE;
	doSingleMotion(sLeftMotion);

	Sleep(100);
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
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
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
		nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}

	//S_Single_Motion sSingleMotion;
	//sSingleMotion.nLogic = 0;
	//sSingleMotion.nAxis = PULP_OUT_AXIS;
	//sSingleMotion.nActionst = 1;
	//doSingleMotion(sSingleMotion);
	doPulpOut(0);
	int nCount = 0;
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
		if (nCount >= VERTICAL_TRANSVERSE)
		{
			smc_stop(m_nConnectNo, PULP_OUT_AXIS, 1);
			break;
		}
		Sleep(50);
		nCount ++;
	}
	Sleep(2000);
	doMoveLUp(80000);
	Sleep(100);
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
		nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
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
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
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
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
		nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}
}

void CHeroMotarControl::threadProcMotar(UINT nParam)
{
	//lock_guard<std::mutex> lock(g_Mutex);
	S_Msg_Info* pInfo = (S_Msg_Info*)nParam;
	auto s_result = Split(pInfo->cBuf, "[\\s,#]+");
	int nSize = s_result.size();
	if (nSize == 3)
	{
		string s_CommandID = s_result[0];
		string s_MsgType = s_result[1];
		string s_Value = s_result[2];
		if (s_MsgType == "MOTAR")
		{
			string sCommand1 = s_Value.substr(0, 5);
			string sCommand2 = s_Value.substr(5, 5);
			string sCommand3 = s_Value.substr(10, 5);
			string sCommand4 = s_Value.substr(15, 5);
			string sCommand5 = s_Value.substr(20, 5);
			string sCommand6 = s_Value.substr(25, 5);
			if ((sCommand1 != "00000") && (sCommand1.length() == 5))
			{
				doMotar(atoi(sCommand1.substr(0, 1).c_str()), atoi(sCommand1.substr(1, 4).c_str()));
			}
			if ((sCommand2 != "00000") && (sCommand2.length() == 5))
			{
				doMotar(atoi(sCommand2.substr(0, 1).c_str()), atoi(sCommand2.substr(1, 4).c_str()));
			}
			if ((sCommand3 != "00000") && (sCommand3.length() == 5))
			{
				doMotar(atoi(sCommand3.substr(0, 1).c_str()), atoi(sCommand3.substr(1, 4).c_str()));
			}
			if ((sCommand4 != "00000") && (sCommand4.length() == 5))
			{
				doMotar(atoi(sCommand4.substr(0, 1).c_str()), atoi(sCommand4.substr(1, 4).c_str()));
			}
			if ((sCommand5 != "00000") && (sCommand5.length() == 5))
			{
				doMotar(atoi(sCommand5.substr(0, 1).c_str()), atoi(sCommand5.substr(1, 4).c_str()));
			}
			if ((sCommand6 != "00000") && (sCommand6.length() == 5))
			{
				doMotar(atoi(sCommand6.substr(0, 1).c_str()), atoi(sCommand6.substr(1, 4).c_str()));
			}

			if (m_pHeroMotarTcp)
			{
				string sMsg = "";
				sMsg += s_result[0];
				sMsg += ",";
				sMsg += s_result[1];
				sMsg += ",";
				sMsg += s_result[2];
				sMsg += ",0,SUCCESS#";
				m_pHeroMotarTcp->tryWrite((char*)sMsg.c_str(), sMsg.length(), pInfo->hSocket);
			}
		}
	}
	else
	{
		if (m_pHeroMotarTcp)
		{
			string sMsg = "";
			sMsg += s_result[0];
			sMsg += ",";
			sMsg += s_result[1];
			sMsg += ",";
			sMsg += s_result[2];
			sMsg += ",1,参数错误#";
			m_pHeroMotarTcp->tryWrite((char*)sMsg.c_str(), sMsg.length(), pInfo->hSocket);
		}
	}

	if (pInfo != NULL)
	{
		delete pInfo;
		pInfo = NULL;
	}
}

void CHeroMotarControl::InitLeftRightData()
{
	short nFrontLowerLimit = !smc_read_inbit(m_nConnectNo, 10);
	short nBackUpperLimit = !smc_read_inbit(m_nConnectNo, 9);
	short nRet = -1;
	if (!nFrontLowerLimit)
	{
		doMoveLDown(80000, true);
		Sleep(100);
		while (1)
		{
			if (m_bEstop)
			{
				eStop();
				m_bEstop = false;
				return;
			}
			nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
			if (nRet == 1)
			{
				g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
				break;
			}
			Sleep(50);
		}
	}
	if (!nBackUpperLimit)
	{
		doMoveLUp(80000, false);
		Sleep(100);
		while (1)
		{
			if (m_bEstop)
			{
				eStop();
				m_bEstop = false;
				return;
			}
			nRet = smc_check_done(m_nConnectNo, BACK_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
			if (nRet == 1)
			{
				g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
				break;
			}
			Sleep(50);
		}
	}
}

void CHeroMotarControl::threadProcLeftMove(UINT nValue)
{
	InitLeftRightData();
	short nRet = -1;
	doMoveLUp(6000, true);
	Sleep(100);
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
		nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}

	doSingleMotion(nValue, -1);
	Sleep(100);
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
		nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}
}

void CHeroMotarControl::threadProcRightMove(UINT nValue)
{
	InitLeftRightData();
	short nRet = -1;
	doMoveLUp(6000, true);
	Sleep(100);
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
		nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}

	doSingleMotion(nValue, 1);
	Sleep(100);
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
		nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}
}

void CHeroMotarControl::threadProcPulpOut()
{
	m_bPulpOut = false;
	//S_Single_Motion sSingleMotion;
	//sSingleMotion.nLogic = 1;
	//sSingleMotion.nAxis = PULP_OUT_AXIS;
	//sSingleMotion.nActionst = 1;
	//doSingleMotion(sSingleMotion);
	doPulpOut(1);
	int nCount = 0;
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
		if (nCount >= VERTICAL_TRANSVERSE)
		{
			smc_stop(m_nConnectNo, PULP_OUT_AXIS, 1);
			break;
		}
		Sleep(50);
		nCount++;
	}
	Sleep(200);
	m_bPulpOut = true;
}

void CHeroMotarControl::threadProcRightVertical(UINT nValue)
{
	InitLeftRightData();

	doMoveLUp(6000, true);
	Sleep(100);
	short nRet = -1;
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
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
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
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

	//doPulpOut();
	thread t1(&CHeroMotarControl::threadProcPulpOut, this);
	t1.detach();

	doMoveLDown(80000, false);
	Sleep(200);
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
		nRet = smc_check_done(m_nConnectNo, BACK_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1 && m_bPulpOut)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			m_bPulpOut = false;
			break;
		}
		Sleep(50);
	}

	
	//doSingleMotion(10, 1);
	//Sleep(100);
	//while (1)
	//{
	//	nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
	//	if (nRet == 1)
	//	{
	//		g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
	//		break;
	//	}
	//	Sleep(50);
	//}

	doMoveLUp(80000, false);
	Sleep(100);
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return;
		}
		nRet = smc_check_done(m_nConnectNo, BACK_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
	}
}