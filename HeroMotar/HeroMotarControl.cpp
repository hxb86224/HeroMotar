#include "HeroMotarControl.h"
#include <iostream>
//#include <tchar.h>

static volatile bool g_bInit = false;
static volatile uint g_nPos = 0;

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
	m_bLeftRightMove = false;
	m_bLeftMove = false;
	g_bInit = false;
	g_nPos = 0;
	InitializeCriticalSection(&m_criticalSection);
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
	DeleteCriticalSection(&m_criticalSection);//删除临界区
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

	smc_set_counter_inmode(m_nConnectNo, PULP_OUT_AXIS, 0);
    g_Logger.TraceInfo("CHeroMotarControl::Init 连接成功！");
	m_pHeroMotarTcp = new CHeroMotarTcp(12345);
	m_pHeroMotarTcp->createListen();
	m_pHeroMotarTcp->SetDealMsgCallback(OnDealMsg);
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
	g_Logger.TraceInfo("CHeroMotarControl::OnDealMsgInfo msg:%s", pData);
	auto s_result = Split(pMsg_Info->cBuf, "[\\s,#]+");
	int nSize = s_result.size();
	if (nSize == 2)
	{
		string s_MsgType = s_result[1];
		if (s_MsgType == "MOTARESTOP")
		{
			emgStop();
		}
	}
	else if (nSize == 3)
	{
		thread t1(&CHeroMotarControl::threadProcMotar, this, (UINT)pMsg_Info);
		t1.detach();
	}
	return nRet;
}

int CHeroMotarControl::doMotar(int nCommand, int nCommandValue)
{
	int nRet = -1;
	g_Logger.TraceInfo("CHeroMotarControl::doMotar  nCommand : %d, nCommandValue : %d！", nCommand, nCommandValue);
	switch (nCommand)
	{
	case LEFT_TRANSVERSE:
		nRet = threadProcLeftTransverse(nCommandValue);
		break;

	case RIGHT_TRANSVERSE:
		nRet = threadProcRightTransverse(nCommandValue);
		break;

	case LEFT_VERTICAL:
		nRet = threadProcLeftVertical(nCommandValue);
		break;

	case RIGHT_VERTICAL:
		nRet = threadProcRightVertical(nCommandValue);
		break;

	case LEFT_MOVE:
		nRet = threadProcLeftMove(nCommandValue);
		break;

	case RIGHT_MOVE:
		nRet = threadProcRightMove(nCommandValue);
		break;

	default:
		break;
	}
	return nRet;
}

int CHeroMotarControl::getState()
{
	//lock_guard<std::mutex> lock(g_Mutex);
	short nRet = -1;
	if (!m_bInitSuccess) {
		return nRet;
	}

	/*short nResult = smc_ping_ip_status("192.168.2.64");
	if (nResult == 0)
	{
		g_Logger.TraceInfo("CHeroMotarControl::getState reconnect AAAAAAA00002！");
		emgStop2();
		smc_board_close(m_nConnectNo);
		if (m_pHeroMotarTcp)
		{
			delete m_pHeroMotarTcp;
			m_pHeroMotarTcp = nullptr;
		}
		int nCount = 0;
		while (nCount <= 12)
		{
			Sleep(500);
			emgStop2();
			nCount++;
		}
		
		if (Init(false) != 0)
		{
			m_bInitSuccess = false;
			return -1;
		}
	}*/
	m_sIn_Out.nIn1 = !smc_read_inbit(m_nConnectNo, 6);
	m_sIn_Out.nIn2 = !smc_read_inbit(m_nConnectNo, 7);
	m_sIn_Out.nIn5 = !smc_read_inbit(m_nConnectNo, 10);
	m_sIn_Out.nIn6 = !smc_read_inbit(m_nConnectNo, 11);

	if (m_sIn_Out.nIn6 && (m_nActionState == LEFT_UP_STATE))
	{
		multicoorStop();
		//eStop();
	}
	if (m_sIn_Out.nIn5 && (m_nActionState == LEFT_DOWN_STATE))
	{
		multicoorStop();
		//eStop();
	}
#if(BLOCK_LEN == 600)
	m_sIn_Out.nIn3 = !smc_read_inbit(m_nConnectNo, 8);
	m_sIn_Out.nIn4 = !smc_read_inbit(m_nConnectNo, 9);
	if (m_sIn_Out.nIn4 && (m_nActionState == RIGHT_UP_STATE))
	{
		multicoorStop();
	}
	if (m_sIn_Out.nIn3 && (m_nActionState == RIGHT_DOWN_STATE))
	{
		multicoorStop();
	}
#endif

	if (m_sIn_Out.nIn1 || m_sIn_Out.nIn2)
	{
		if (m_bLeftMove)
		{
			g_Logger.TraceInfo("CHeroMotarControl::getState 紧急停止 00001！");
			emgStop2();
			m_bLeftMove = false;
		}
		else
		{
			double dSpeed = 0.00;
			smc_read_current_speed_unit(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS, &dSpeed);
			if (dSpeed < 0)
			{
				g_Logger.TraceInfo("CHeroMotarControl::getState 紧急停止 0002！");
				emgStop2();
			}
		}

	}

	return nRet;
}

//执行单轴运动
int CHeroMotarControl::doSingleMotion()
{
	short nRet = -1;
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
	m_bLeftMove = (nLogic > 0) ? false : true;
	nRet = doSingleMotion(sSingleMotion);
	return nRet;
}

int CHeroMotarControl::doEmptyMotion(UINT nLen, int nLogic)
{
	short nRet = -1;
	S_Single_Motion sSingleMotion;
	sSingleMotion.nLogic = nLogic;
	sSingleMotion.nPulse = (nLen / PULSE_UNIT);
	m_nActionState = (nLogic > 0) ? RIGHT_STATE : LEFT_STATE;
	m_bLeftMove = (nLogic > 0) ? false : true;
	sSingleMotion.dSpeedMin = 90.0;
	sSingleMotion.dSpeedStop = 100.0;
	sSingleMotion.dSpeed = 3000.0;
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
		m_bLeftRightMove = true;
		nRet = smc_pmove_unit(m_nConnectNo, sSingleMotion.nAxis, sSingleMotion.nPulse * sSingleMotion.nLogic, 0);//相对定长运动
	}
	else
	{
		m_bLeftRightMove = true;
		nRet = smc_vmove(m_nConnectNo, sSingleMotion.nAxis, sSingleMotion.nLogic);	//恒速运动
	}
	return nRet;
}

int CHeroMotarControl::doSingleMotion2()
{
	short nRet = -1;
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
	return smc_change_speed_unit(m_nConnectNo, m_sSingleMotion.nAxis, dSpeed, 0.1);
}

int CHeroMotarControl::emgStop()
{
	m_nActionState = STATIC_STATE;
	m_bEstop = true;
	eStop();
	g_bInit = false;
	return 0;
	//return smc_stop(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS, 1);	//立即停止	
}

int CHeroMotarControl::emgStop2()
{
	m_nActionState = STATIC_STATE;
	return smc_stop(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS, 1);	//立即停止	
}

int CHeroMotarControl::doMoveL2()
{
	return doMoveLUp(10000, true);
	//return doMoveLDown(80000, true); //下降到下限位
	//doMoveLUp(80000, true);  //上升到上限位
	//return doPulpOut(1);		//正转



	//static int nCount = 1;
	//short nIn6 = !smc_read_inbit(m_nConnectNo, 11);
	//while (1)
	//{
	//	nIn6 = !smc_read_inbit(m_nConnectNo, 11);
	//	if (nIn6)
	//	{
	//		break;
	//	}
	//	doMoveLUp(100, true);
	//	nIn6 = !smc_read_inbit(m_nConnectNo, 11);
	//	if (nIn6)
	//	{
	//		break;
	//	}
	//	nCount++;
	//	g_Logger.TraceInfo("CHeroMotarControl::doMoveL2 nCount = %d ！", nCount);
	//	Sleep(500);
	//}
	//return 0;



	//return doPulpOut(0);      //反转
	//static int nCount = 1;
	//if (nCount % 2)
	//	doMoveLUp(80000, true);
	//else
	//	doMoveLDown(80000, true);
	//nCount++;
	//return 0;
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
		//sMove_L.nAxis[1] = FRONT_UP_DOWN_MOTION_AXIS2;
		sMove_L.dDist[0] = (-1) * sMove_L.nLogic * sMove_L.dDis;
		//sMove_L.dDist[1] = sMove_L.nLogic * sMove_L.dDis;
		m_nActionState = LEFT_UP_STATE;
	}
	else
	{
		sMove_L.nAxis[0] = BACK_UP_DOWN_MOTION_AXIS1;
		//sMove_L.nAxis[1] = BACK_UP_DOWN_MOTION_AXIS2;
		sMove_L.dDist[0] = (-1) * sMove_L.nLogic * sMove_L.dDis;
		//sMove_L.dDist[1] = (-1) * sMove_L.nLogic * sMove_L.dDis;
		m_nActionState = RIGHT_UP_STATE;
	}
	return doMoveL(sMove_L);
}

int CHeroMotarControl::doMoveLDown(double dDis, bool bFront)
{
	S_Move_L sMove_L;
	sMove_L.dTAcc = 0.1;
	sMove_L.dTDec = 0.1;
	sMove_L.nLogic = -1;
	sMove_L.dDis = dDis;
	sMove_L.dSpeedrun = 1000.0;
	short nRet = 0;
	if (bFront)
	{
		sMove_L.nAxis[0] = FRONT_UP_DOWN_MOTION_AXIS1;
		//sMove_L.nAxis[1] = FRONT_UP_DOWN_MOTION_AXIS2;
		sMove_L.dDist[0] = (-1) * sMove_L.nLogic * sMove_L.dDis;
		//sMove_L.dDist[1] = sMove_L.nLogic * sMove_L.dDis;
		m_nActionState = LEFT_DOWN_STATE;
	}
	else
	{
		sMove_L.nAxis[0] = BACK_UP_DOWN_MOTION_AXIS1;
		//sMove_L.nAxis[1] = BACK_UP_DOWN_MOTION_AXIS2;
		sMove_L.dDist[0] = (-1) * sMove_L.nLogic * sMove_L.dDis;
		//sMove_L.dDist[1] = (-1) * sMove_L.nLogic * sMove_L.dDis;
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
	nRet = smc_line_unit(m_nConnectNo, m_sMove_L.nCrd, 1, sMove_L.nAxis, sMove_L.dDist, 0);
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
	m_bLeftRightMove = false;
	return smc_emg_stop(m_nConnectNo);	      //紧急停止
}

int CHeroMotarControl::doLeftTransverse()
{
	//doSingleMotion();
	//smc_write_outbit(m_nConnectNo, 1, 1);
	thread t1(&CHeroMotarControl::threadProcLeftTransverse, this, 600);
	t1.detach();
	return 0;
}

int CHeroMotarControl::doRightTransverse()
{
	//doSingleMotion2();
	thread t1(&CHeroMotarControl::threadProcRightTransverse, this, 600);
	t1.detach();
	return 0;
}

int CHeroMotarControl::doLeftVertical()
{
	thread t1(&CHeroMotarControl::threadProcLeftVertical, this, 300);
	t1.detach();
	return 0;
}

int CHeroMotarControl::doRightVertical()
{
	thread t1(&CHeroMotarControl::threadProcRightVertical, this, 300);
	t1.detach();
	return 0;
}

int CHeroMotarControl::threadProcLeftTransverse(UINT nValue)
{
	if (!InitLeftRightData())
	{
		return -1;
	}
	short nRet = -1;
	UINT nCount = 0;
	if (toTargetHeight(INITIAL_HEIGHT) != 0)
	{
		return -1;
	}
	//doMoveLUp(INITIAL_HEIGHT, true);
	//Sleep(100);
	//while (1)
	//{
	//	if (nCount >= EVENT_TIME_OUT)
	//	{
	//		eStop();
	//		m_bEstop = false;
	//		return -1;
	//	}
	//	if (m_bEstop)
	//	{
	//		eStop();
	//		m_bEstop = false;
	//		return -1;
	//	}
	//	nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
	//	if (nRet == 1)
	//	{
	//		g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
	//		nCount = 0;
	//		break;
	//	}
	//	Sleep(50);
	//	nCount++;
	//}

	doPulpOut(1);

	UINT nInterval = (nValue * HORIZONTAL_TRANSVERSE_300) / ONCE_MOVE_LEN;

	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return -1;
		}
		if (nCount >= nInterval)
		{
			smc_stop(m_nConnectNo, PULP_OUT_AXIS, 1);
			nCount = 0;
			break;
		}
		Sleep(50);
		nCount++;
	}

	smc_set_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, 0);
	doSingleMotion(nValue, -1);
	Sleep(100);
	short nIn0 = 1;
	while (1)
	{
		if (nCount >= EVENT_TIME_OUT)
		{
			eStop();
			m_bEstop = false;
			m_bLeftRightMove = false;
			m_bLeftMove = false;
			return -1;
		}
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			m_bLeftRightMove = false;
			m_bLeftMove = false;
			return -1;
		}
		nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProcRightVertical 当前状态：静止！");
			nCount = 0;
			m_bLeftRightMove = false;
			m_bLeftMove = false;
			break;
		}
		Sleep(50);
		nCount++;
	}
	double dPos = 0.00;
	smc_get_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, &dPos);
	dPos = abs(dPos);
	double dExpectedPos = nValue * ENCODER_UNIT;
	double dInterval = abs(dPos - dExpectedPos);
	g_Logger.TraceInfo("CHeroMotarControl::threadProcLeftTransverse Pos = %f, dExpectedPos = %f ！", dPos, dExpectedPos);
	if (dInterval > IGNORE_LEN)
	{
		if (dPos < dExpectedPos)
		{
			UINT nLen = (UINT)(dInterval / ENCODER_UNIT);
			smc_set_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, 0);
			doSingleMotion(nLen, -1);
			Sleep(100);
			while (1)
			{
				if (nCount >= EVENT_TIME_OUT)
				{
					eStop();
					m_bEstop = false;
					m_bLeftRightMove = false;
					m_bLeftMove = false;
					return -1;
				}
				if (m_bEstop)
				{
					eStop();
					m_bEstop = false;
					m_bLeftRightMove = false;
					m_bLeftMove = false;
					return -1;
				}
				nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
				if (nRet == 1)
				{
					g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
					m_bLeftRightMove = false;
					nCount = 0;
					m_bLeftMove = false;
					break;
				}
				Sleep(50);
				nCount++;
			}

			smc_get_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, &dPos);
			g_Logger.TraceInfo("CHeroMotarControl::threadProcLeftMove Pos = %f！", dPos);
			dPos = abs(dPos);
			dInterval = abs(dPos - dInterval);
			if (dInterval > IGNORE_LEN)
			{
				eStop();
				m_bEstop = false;
				m_bLeftRightMove = false;
				return -1;
			}
		}
	}

	//if (!InitEndLeftRightData())
	//{
	//	return -1;
	//}
	return 0;
}

bool CHeroMotarControl::HorizontalTransverse(UINT nLen)
{
	thread t1(&CHeroMotarControl::threadProcPulpOut, this, nLen);
	t1.detach();

	if (toTargetHeight(HORIZONTAL_INITIAL_HEIGHT) != 0)
	{
		return false;
	}

	short nRet = -1;
	UINT nCount = 0;
	//doMoveLUp(14000, true);
	//Sleep(100);
	//g_Logger.TraceInfo("CHeroMotarControl::HorizontalTransverse BBBB000！");
	//while (1)
	//{
	//	if (nCount >= VERTICAL_EVENT_TIME_OUT)
	//	{
	//		eStop();
	//		m_bEstop = false;
	//		return false;
	//	}
	//	if (m_bEstop)
	//	{
	//		eStop();
	//		m_bEstop = false;
	//		return false;
	//	}
	//	nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
	//	if (nRet == 1 && m_bPulpOut)
	//	{
	//		g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
	//		nCount = 0;
	//		break;
	//	}
	//	Sleep(50);
	//	nCount++;
	//}

	Sleep(1000);

	g_Logger.TraceInfo("CHeroMotarControl::HorizontalTransverse BBBB002！");
	doMoveLDown(MAX_WHOLE_LEN, true);
	Sleep(100);
	while (1)
	{
		if (nCount >= EVENT_TIME_OUT)
		{
			eStop();
			m_bEstop = false;
			return false;
		}
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
			nCount = 0;
			break;
		}
		Sleep(50);
		nCount++;
	}
	g_nPos = 0;
	g_Logger.TraceInfo("CHeroMotarControl::HorizontalTransverse BBBB003！");
	smc_set_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, 0);
	doSingleMotion(nLen, 1);
	Sleep(100);
	short nIn0 = 1;
	bool bMissStep = false;
	while (1)
	{
		if (nCount >= EVENT_TIME_OUT)
		{
			eStop();
			m_bEstop = false;
			m_bLeftRightMove = false;
			return false;
		}
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			m_bLeftRightMove = false;
			return false;
		}
		nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		if (nRet == 1)
		{
			bMissStep = false;
			nCount = 0;
			m_bLeftRightMove = false;
			break;
		}
		else
		{
			nIn0 = !smc_read_inbit(m_nConnectNo, 5);
			if (!nIn0)
			{
				bMissStep = true;
				nCount = 0;
				emgStop2();
				m_bLeftRightMove = false;
				break;
			}
		}
		Sleep(50);
		nCount++;
	}

	if (!bMissStep)
	{
		double dPos = 0.00;
		smc_get_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, &dPos);
		g_Logger.TraceInfo("CHeroMotarControl::threadProcLeftMove Pos = %f！", dPos);
		dPos = abs(dPos);
		double dExpectedPos = nLen * ENCODER_UNIT;
		double dInterval = abs(dPos - dExpectedPos);
		if (dInterval > IGNORE_LEN)
		{
			if (dPos < dExpectedPos)
			{
				UINT nLenNew = (UINT)(dInterval / ENCODER_UNIT);
				smc_set_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, 0);
				doSingleMotion(nLenNew, 1);
				Sleep(100);
				while (1)
				{
					if (nCount >= EVENT_TIME_OUT)
					{
						eStop();
						m_bEstop = false;
						m_bLeftRightMove = false;
						return -1;
					}
					if (m_bEstop)
					{
						eStop();
						m_bEstop = false;
						m_bLeftRightMove = false;
						return -1;
					}
					nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
					if (nRet == 1)
					{
						g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
						m_bLeftRightMove = false;
						nCount = 0;
						break;
					}
					else
					{
						nIn0 = !smc_read_inbit(m_nConnectNo, 5);
						if (!nIn0)
						{
							nCount = 0;
							emgStop2();
							m_bLeftRightMove = false;
							bMissStep = true;
							break;
						}
					}
					Sleep(50);
					nCount++;
				}

				smc_get_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, &dPos);
				g_Logger.TraceInfo("CHeroMotarControl::threadProcLeftMove Pos = %f！", dPos);
				dPos = abs(dPos);
				dInterval = abs(dPos - dInterval);
				if ((dInterval > IGNORE_LEN) && !bMissStep)
				{
					eStop();
					m_bEstop = false;
					m_bLeftRightMove = false;
					return false;
				}
			}
		}
	}
	return true;
}

int CHeroMotarControl::threadProcRightTransverse(UINT nValue)
{
	if (!InitLeftRightData())
	{
		g_Logger.TraceInfo("CHeroMotarControl::threadProcRightTransverse BBBB001！");
		return -1;
	}

	g_Logger.TraceInfo("CHeroMotarControl::threadProcRightTransverse BBBB000！");
	UINT nCountLen = (nValue % ONCE_MOVE_LEN == 0) ? (nValue / ONCE_MOVE_LEN) : (nValue / ONCE_MOVE_LEN) + 1;
	for (UINT i = 0; i < nCountLen; i++)
	{
		if (i == nCountLen - 1)
		{
			nValue = (nValue % ONCE_MOVE_LEN == 0) ? ONCE_MOVE_LEN : nValue % ONCE_MOVE_LEN;
			g_Logger.TraceInfo("CHeroMotarControl::threadProcRightTransverse BBBB002 : %d！", nValue);
			if (!HorizontalTransverse(nValue))
			{
				return -1;
			}
		}
		else
		{
			UINT nLen = (nValue >= ONCE_MOVE_LEN) ? ONCE_MOVE_LEN : nValue;
			if (!HorizontalTransverse(nLen))
			{
				g_Logger.TraceInfo("CHeroMotarControl::threadProcRightTransverse BBBB003！");
				return -1;
			}
			g_Logger.TraceInfo("CHeroMotarControl::threadProcRightTransverse BBBB004！");
		}
	}
	g_Logger.TraceInfo("CHeroMotarControl::threadProcRightTransverse BBBB005！");
	//if (!InitEndLeftRightData())
	//{
	//	return -1;
	//}
	return 0;
}

int CHeroMotarControl::threadProcLeftVertical(UINT nValue)
{
	if (!InitLeftRightData())
	{
		return -1;
	}
	short nRet = -1;
	UINT nCount = 0;
	if (toTargetHeight(INITIAL_HEIGHT) != 0)
	{
		return -1;
	}

	//doMoveLUp(INITIAL_HEIGHT, true);
	//Sleep(100);
	//while (1)
	//{
	//	if (nCount >= EVENT_TIME_OUT)
	//	{
	//		eStop();
	//		m_bEstop = false;
	//		return -1;
	//	}
	//	if (m_bEstop)
	//	{
	//		eStop();
	//		m_bEstop = false;
	//		return -1;
	//	}
	//	nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
	//	if (nRet == 1)
	//	{
	//		g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
	//		nCount = 0;
	//		break;
	//	}
	//	Sleep(50);
	//	nCount++;
	//}
	
	S_Single_Motion sLeftMotion;
	sLeftMotion.nLogic = 0;
	sLeftMotion.nAxis = LEFT_RIGHT_MOTION_AXIS;
	sLeftMotion.nActionst = 1;
	sLeftMotion.dSpeedMin = 90.0;
	sLeftMotion.dSpeedStop = 100.0;
	sLeftMotion.dSpeed = 3000.0;
	m_nActionState = LEFT_STATE;
	m_bLeftMove = true;
	doSingleMotion(sLeftMotion);

	Sleep(100);
	while (1)
	{
		if (nCount >= EVENT_TIME_OUT)
		{
			eStop();
			m_bEstop = false;
			m_bLeftRightMove = false;
			m_bLeftMove = false;
			return -1;
		}
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			m_bLeftRightMove = false;
			m_bLeftMove = false;
			return -1;
		}
		nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			m_bLeftRightMove = false;
			m_bLeftMove = false;
			nCount = 0;
			break;
		}
		Sleep(50);
		nCount++;
	}

	//doMoveLDown(80000, true);
	//Sleep(100);
	//while (1)
	//{
	//	if (nCount >= VERTICAL_EVENT_TIME_OUT)
	//	{
	//		eStop();
	//		m_bEstop = false;
	//		return -1;
	//	}
	//	if (m_bEstop)
	//	{
	//		eStop();
	//		m_bEstop = false;
	//		return -1;
	//	}
	//	nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
	//	if (nRet == 1)
	//	{
	//		g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
	//		nCount = 0;
	//		break;
	//	}
	//	Sleep(50);
	//	nCount++;
	//}

	doPulpOut(0);
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return -1;
		}
		if (nCount >= VERTICAL_TRANSVERSE)
		{
			smc_stop(m_nConnectNo, PULP_OUT_AXIS, 1);
			nCount = 0;
			break;
		}
		Sleep(50);
		nCount ++;
	}
	Sleep(1000);
	doMoveLUp(MOVE_WHOLE_LEN - INITIAL_HEIGHT, true);
	Sleep(100);
	while (1)
	{
		if (nCount >= EVENT_TIME_OUT)
		{
			eStop();
			m_bEstop = false;
			return -1;
		}
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return -1;
		}
		nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			nCount = 0;
			break;
		}
		Sleep(50);
		nCount++;
	}
	g_nPos = MOVE_WHOLE_LEN;
	Sleep(200);
	doSingleMotion(30, 1);
	Sleep(100);
	while (1)
	{
		if (nCount >= 100)
		{
			eStop();
			m_bEstop = false;
			m_bLeftRightMove = false;
			return -1;
		}
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			m_bLeftRightMove = false;
			return -1;
		}
		nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			nCount = 0;
			m_bLeftRightMove = false;
			break;
		}
		Sleep(50);
		nCount++;
	}
	

	doMoveLDown(MOVE_WHOLE_LEN - INITIAL_HEIGHT, true);
	Sleep(100);
	while (1)
	{
		if (nCount >= EVENT_TIME_OUT)
		{
			eStop();
			m_bEstop = false;
			return -1;
		}
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return -1;
		}
		nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			nCount = 0;
			break;
		}
		Sleep(50);
		nCount++;
	}
	//g_nPos = 0;

	//doMoveLUp(INITIAL_HEIGHT, true);
	//Sleep(100);
	//while (1)
	//{
	//	if (nCount >= EVENT_TIME_OUT)
	//	{
	//		eStop();
	//		m_bEstop = false;
	//		return -1;
	//	}
	//	if (m_bEstop)
	//	{
	//		eStop();
	//		m_bEstop = false;
	//		return -1;
	//	}
	//	nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
	//	if (nRet == 1)
	//	{
	//		g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
	//		nCount = 0;
	//		break;
	//	}
	//	Sleep(50);
	//	nCount++;
	//}
	g_nPos = INITIAL_HEIGHT;

	doSingleMotion(20, -1);
	Sleep(100);
	while (1)
	{
		if (nCount >= EVENT_TIME_OUT)
		{
			eStop();
			m_bEstop = false;
			m_bLeftRightMove = false;
			m_bLeftMove = false;
			return -1;
		}
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			m_bLeftRightMove = false;
			m_bLeftMove = false;
			return -1;
		}
		nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			nCount = 0;
			m_bLeftRightMove = false;
			m_bLeftMove = false;
			break;
		}
		Sleep(50);
		nCount++;
	}

	//if (!InitEndLeftRightData())
	//{
	//	return -1;
	//}
	return 0;
}

void CHeroMotarControl::threadProcMotar(UINT nParam)
{
	g_Logger.TraceInfo("CHeroMotarControl::threadProcMotar AAAAAAABA00！");
	EnterCriticalSection(&m_criticalSection);//进入临界区
	g_Logger.TraceInfo("CHeroMotarControl::threadProcMotar AAAAAAABB00！");
	S_Msg_Info* pInfo = (S_Msg_Info*)nParam;
	auto s_result = Split(pInfo->cBuf, "[\\s,#]+");
	int nSize = s_result.size();
	int nRet = 1;
	if (nSize == 3)
	{
		m_bEstop = false;
		g_Logger.TraceInfo("CHeroMotarControl::threadProcMotar AAAAAAA01！");
		string s_CommandID = s_result[0];
		string s_MsgType = s_result[1];
		string s_Value = s_result[2];
		if (s_MsgType == "MOTAR")
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProcMotar AAAAAAA02！");
			if (!m_bInitSuccess)
			{
				nRet = 2;
				goto cleanup;
			}
			g_Logger.TraceInfo("CHeroMotarControl::threadProcMotar AAAAAAA03！");
			//m_sIn_Out.nIn7 = !smc_read_inbit(m_nConnectNo, 0);
			//if (!m_sIn_Out.nIn7)
			//{

			//	g_Logger.TraceInfo("CHeroMotarControl::threadProcMotar  !AAAAAAA00003！");
			//	nRet = 3;
			//	goto cleanup;
			//}
			int nLen = s_Value.length();
			string sCommand1, sCommand2, sCommand3, sCommand4, sCommand5, sCommand6;
			if(nLen >= 5)
				sCommand1 = s_Value.substr(0, 5);
			if (nLen >= 10)
				sCommand2 = s_Value.substr(5, 5);
			if (nLen >= 15)
				sCommand3 = s_Value.substr(10, 5);
			if (nLen >= 20)
				sCommand4 = s_Value.substr(15, 5);
			if (nLen >= 25)
				sCommand5 = s_Value.substr(20, 5);
			if (nLen >= 30)
				sCommand6 = s_Value.substr(25, 5);
			if ((sCommand1 != "00000") && (sCommand1.length() == 5))
			{
				nRet = doMotar(atoi(sCommand1.substr(0, 1).c_str()), atoi(sCommand1.substr(1, 4).c_str()));
				g_Logger.TraceInfo("CHeroMotarControl::threadProcMotar AAAAAA010！ %d ", nRet);
				if (nRet == -1)
				{
					goto cleanup;
				}
			}
			if ((sCommand2 != "00000") && (sCommand2.length() == 5))
			{
				nRet = doMotar(atoi(sCommand2.substr(0, 1).c_str()), atoi(sCommand2.substr(1, 4).c_str()));
				g_Logger.TraceInfo("CHeroMotarControl::threadProcMotar AAAAAA011！ %d ", nRet);
				if (nRet == -1)
				{
					goto cleanup;
				}
			}
			if ((sCommand3 != "00000") && (sCommand3.length() == 5))
			{
				int nCommand = atoi(sCommand3.substr(0, 1).c_str());
				if (m_pHeroMotarTcp && (nCommand == RIGHT_MOVE))
				{
					string sMsg = "";
					sMsg += s_result[0];
					sMsg += ",";
					sMsg += s_result[1];
					sMsg += ",";
					sMsg += s_result[2];
					if (nRet == 1)
					{
						sMsg += ",1,param error#";
					}
					else if (nRet == -1)
					{
						sMsg += ",2,execute error#";
					}
					else if (nRet == 2)
					{
						sMsg += ",3,the network may be disconnected#";
					}
					else if (nRet == 3)
					{
						sMsg += ",4,please open the enable switch#";
					}
					else
					{
						sMsg += ",0,SUCCESS#";
					}
					g_Logger.TraceInfo("CHeroMotarControl::threadProcMotar msg:%s", sMsg.c_str());
					m_pHeroMotarTcp->tryWrite((char*)sMsg.c_str(), sMsg.length(), pInfo->hSocket);
					nRet = doMotar(atoi(sCommand3.substr(0, 1).c_str()), atoi(sCommand3.substr(1, 4).c_str()));
					if (pInfo != NULL)
					{
						delete pInfo;
						pInfo = NULL;
					}
					g_Logger.TraceInfo("CHeroMotarControl::threadProcMotar AAAAAAABB01！");
					LeaveCriticalSection(&m_criticalSection);
					return;
				}

				nRet = doMotar(atoi(sCommand3.substr(0, 1).c_str()), atoi(sCommand3.substr(1, 4).c_str()));
				if (nRet == -1)
				{
					goto cleanup;
				}
			}
			if ((sCommand4 != "00000") && (sCommand4.length() == 5))
			{
				nRet = doMotar(atoi(sCommand4.substr(0, 1).c_str()), atoi(sCommand4.substr(1, 4).c_str()));
				if (nRet == -1)
				{
					goto cleanup;
				}
			}
			if ((sCommand5 != "00000") && (sCommand5.length() == 5))
			{
				nRet = doMotar(atoi(sCommand5.substr(0, 1).c_str()), atoi(sCommand5.substr(1, 4).c_str()));
				if (nRet == -1)
				{
					goto cleanup;
				}
			}
			if ((sCommand6 != "00000") && (sCommand6.length() == 5))
			{
				nRet = doMotar(atoi(sCommand6.substr(0, 1).c_str()), atoi(sCommand6.substr(1, 4).c_str()));
				if (nRet == -1)
				{
					goto cleanup;
				}
			}
			nRet = 0;
		}
		goto cleanup;
	}
	else
	{
		nRet = 1;
		goto cleanup;
	}


cleanup:
	{
		if (m_pHeroMotarTcp)
		{
			string sMsg = "";
			sMsg += s_result[0];
			sMsg += ",";
			sMsg += s_result[1];
			sMsg += ",";
			sMsg += s_result[2];
			if (nRet == 1)
			{
				sMsg += ",1,param error#";
			}
			else if (nRet == -1)
			{
				sMsg += ",2,execute error#";
			}
			else if (nRet == 2)
			{
				sMsg += ",3,the network may be disconnected#";
			}
			else if (nRet == 3)
			{
				sMsg += ",4,please open the enable switch#";
			}
			else
			{
				sMsg += ",0,SUCCESS#";
			}
			g_Logger.TraceInfo("CHeroMotarControl::threadProcMotar msg:%s", sMsg.c_str());
			//LeaveCriticalSection(&m_criticalSection);
			m_pHeroMotarTcp->tryWrite((char*)sMsg.c_str(), sMsg.length(), pInfo->hSocket);
			g_Logger.TraceInfo("CHeroMotarControl::threadProcMotar AAAAAAABB03！");

		}
		if (pInfo != NULL)
		{
			delete pInfo;
			pInfo = NULL;
		}
		g_Logger.TraceInfo("CHeroMotarControl::threadProcMotar AAAAAAABB02！");
		LeaveCriticalSection(&m_criticalSection);
		g_Logger.TraceInfo("CHeroMotarControl::threadProcMotar AAAAAAABB03！");
	}
}

bool CHeroMotarControl::InitEndLeftRightData()
{
	short nFrontLowerLimit = !smc_read_inbit(m_nConnectNo, 10);
	short nRet = -1;
	UINT nCount = 0;
	if (!nFrontLowerLimit)
	{
		doMoveLDown(80000, true);
		Sleep(100);
		while (1)
		{
			if (nCount >= VERTICAL_EVENT_TIME_OUT)
			{
				eStop();
				g_Logger.TraceInfo("CHeroMotarControl::InitLeftRightData AAAAAA001！");
				m_bEstop = false;
				return false;
			}
			if (m_bEstop)
			{
				eStop();
				m_bEstop = false;
				g_Logger.TraceInfo("CHeroMotarControl::InitLeftRightData AAAAAA002！");
				return false;
			}
			nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
			if (nRet == 1)
			{
				g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
				nCount = 0;
				break;
			}
			Sleep(50);
			nCount++;
		}

		//Sleep(1000);

		g_Logger.TraceInfo("CHeroMotarControl::InitLeftRightData AAAAAA010！");
		doMoveLUp(INITIAL_HEIGHT, true);
		g_Logger.TraceInfo("CHeroMotarControl::InitLeftRightData AAAAAA011！");
		Sleep(100);
		while (1)
		{
			if (nCount >= EVENT_TIME_OUT)
			{
				eStop();
				m_bEstop = false;
				g_Logger.TraceInfo("CHeroMotarControl::InitLeftRightData AAAAAA003！");
				return false;
			}
			if (m_bEstop)
			{
				eStop();
				g_Logger.TraceInfo("CHeroMotarControl::InitLeftRightData AAAAAA004！");
				m_bEstop = false;
				return false;
			}
			nRet = smc_check_done(m_nConnectNo, BACK_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
			if (nRet == 1)
			{
				g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
				break;
			}
			Sleep(50);
			nCount++;
		}
	}

	else
	{
		g_Logger.TraceInfo("CHeroMotarControl::InitLeftRightData AAAAAA012！");
		doMoveLUp(INITIAL_HEIGHT, true);
		g_Logger.TraceInfo("CHeroMotarControl::InitLeftRightData AAAAAA013！");
		Sleep(100);
		while (1)
		{
			if (nCount >= EVENT_TIME_OUT)
			{
				eStop();
				m_bEstop = false;
				g_Logger.TraceInfo("CHeroMotarControl::InitLeftRightData AAAAAA003！");
				return false;
			}
			if (m_bEstop)
			{
				eStop();
				g_Logger.TraceInfo("CHeroMotarControl::InitLeftRightData AAAAAA004！");
				m_bEstop = false;
				return false;
			}
			nRet = smc_check_done(m_nConnectNo, BACK_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
			if (nRet == 1)
			{
				g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
				break;
			}
			Sleep(50);
			nCount++;
		}
	}

#if(BLOCK_LEN == 600)
	short nBackUpperLimit = !smc_read_inbit(m_nConnectNo, 9);
	if (!nBackUpperLimit)
	{
		doMoveLUp(80000, false);
		Sleep(100);
		//SetEvent(m_gEvent);
		while (1)
		{
			if (nCount >= VERTICAL_EVENT_TIME_OUT)
			{
				eStop();
				m_bEstop = false;
				g_Logger.TraceInfo("CHeroMotarControl::InitLeftRightData AAAAAA003！");
				return false;
			}
			if (m_bEstop)
			{
				eStop();
				g_Logger.TraceInfo("CHeroMotarControl::InitLeftRightData AAAAAA004！");
				m_bEstop = false;
				return false;
			}
			nRet = smc_check_done(m_nConnectNo, BACK_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
			if (nRet == 1)
			{
				g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
				break;
			}
			Sleep(50);
			nCount++;
		}
	}
#endif
	return true;
}

bool CHeroMotarControl::InitLeftRightData()
{
	if (g_bInit)
	{
		return true;
	}
	short nRet = -1;
	UINT nCount = 0;

	doMoveLDown(MAX_WHOLE_LEN, true);
	Sleep(100);
	while (1)
	{
		if (nCount >= EVENT_TIME_OUT)
		{
			eStop();
			m_bEstop = false;
			g_Logger.TraceInfo("CHeroMotarControl::InitLeftRightData AAAAAA001！");
			return false;
		}
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			g_Logger.TraceInfo("CHeroMotarControl::InitLeftRightData AAAAAA002！");
			return false;
		}
		nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::InitLeftRightData 当前状态：静止！");
			nCount = 0;
			break;
		}
		Sleep(50);
		nCount ++;
	}

	g_nPos = 0;

	doMoveLUp(INITIAL_HEIGHT, true);
	Sleep(100);
	while (1)
	{
		if (nCount >= EVENT_TIME_OUT)
		{
			eStop();
			m_bEstop = false;
			return false;
		}
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return false;
		}
		nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::InitLeftRightData 当前状态：静止！");
			nCount = 0;
			break;
		}
		Sleep(50);
		nCount++;
	}

	g_nPos = INITIAL_HEIGHT;
	g_bInit = true;
	return true;
}

int CHeroMotarControl::threadProcLeftMove(UINT nValue)
{
	if (!InitLeftRightData())
	{
		return -1;
	}
	if (toTargetHeight(INITIAL_HEIGHT) != 0)
	{
		return -1;
	}
	//short nRet = -1;
	//doMoveLUp(INITIAL_HEIGHT, true);
	//Sleep(100);
	//UINT nCount = 0;
	//double dPos = 0.00;
	//while (1)
	//{
	//	if (nCount >= EVENT_TIME_OUT)
	//	{
	//		eStop();
	//		m_bEstop = false;
	//		return -1;
	//	}
	//	if (m_bEstop)
	//	{
	//		eStop();
	//		m_bEstop = false;
	//		return -1;
	//	}
	//	nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
	//	if (nRet == 1)
	//	{
	//		g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
	//		nCount = 0;
	//		break;
	//	}
	//	Sleep(50);
	//	nCount++;
	//}

	short nRet = -1;
	UINT nCount = 0;
	double dPos = 0.00;
	smc_set_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, 0);
	doEmptyMotion(nValue, -1);
	Sleep(100);
	while (1)
	{
		if (nCount >= EVENT_TIME_OUT)
		{
			eStop();
			m_bEstop = false;
			m_bLeftRightMove = false;
			m_bLeftMove = false;
			return -1;
		}
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			m_bLeftRightMove = false;
			m_bLeftMove = false;
			return -1;
		}
		nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			m_bLeftRightMove = false;
			m_bLeftMove = false;
			break;
		}
		Sleep(50);
		nCount++;
	}

	smc_get_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, &dPos);
	g_Logger.TraceInfo("CHeroMotarControl::threadProcLeftMove Pos = %f！", dPos);
	/*dPos = abs(dPos);
	double dExpectedPos = nValue * ENCODER_UNIT;
	double dInterval = abs(dPos - dExpectedPos);
	if (dInterval > IGNORE_LEN)
	{
		if (dPos < dExpectedPos)
		{
			UINT nLen = (UINT)(dInterval / ENCODER_UNIT);
			smc_set_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, 0);
			doEmptyMotion(nLen, -1);
			Sleep(100);
			while (1)
			{
				if (nCount >= EVENT_TIME_OUT)
				{
					eStop();
					m_bEstop = false;
					m_bLeftRightMove = false;
					return -1;
				}
				if (m_bEstop)
				{
					eStop();
					m_bEstop = false;
					m_bLeftRightMove = false;
					return -1;
				}
				nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
				if (nRet == 1)
				{
					g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
					m_bLeftRightMove = false;
					nCount = 0;
					break;
				}
				Sleep(50);
				nCount++;
			}

			smc_get_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, &dPos);
			g_Logger.TraceInfo("CHeroMotarControl::threadProcLeftMove Pos = %f！", dPos);
			dInterval = abs(dPos - dInterval);
			if (dInterval > IGNORE_LEN)
			{
				eStop();
				m_bEstop = false;
				m_bLeftRightMove = false;
				return -1;
			}
		}
	}*/

	//if (!InitEndLeftRightData())
	//{
	//	return -1;
	//}
	return 0;
}

int CHeroMotarControl::toTargetHeight(UINT nValue)
{
	short nRet = -1;
	UINT nCount = 0;
	if (g_nPos > nValue)
	{
		doMoveLDown(g_nPos - nValue, true);
	}
	else if (g_nPos < nValue)
	{
		doMoveLUp(nValue - g_nPos, true);
	}
	else
	{
		return 0;
	}
	Sleep(100);
	//UINT nCount = 0;
	while (1)
	{
		if (nCount >= EVENT_TIME_OUT)
		{
			eStop();
			m_bEstop = false;
			return -1;
		}
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return -1;
		}
		nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::toTargetHeight 当前状态：静止！");
			nCount = 0;
			break;
		}
		Sleep(50);
		nCount++;
	}
	g_nPos = nValue;
	return 0;
}

int CHeroMotarControl::threadProcRightMove(UINT nValue)
{
	if (!InitLeftRightData())
	{
		return -1;
	}
	
	short nRet = -1;
	UINT nCount = 0;
	if (toTargetHeight(INITIAL_RIGHT_HEIGHT) != 0)
	{
		return -1;
	}
	//if (g_nPos > INITIAL_RIGHT_HEIGHT)
	//{
	//	doMoveLDown(g_nPos - INITIAL_RIGHT_HEIGHT, true);
	//}
	//else
	//{
	//	doMoveLUp(INITIAL_RIGHT_HEIGHT - g_nPos, true);
	//}
	//Sleep(100);
	////UINT nCount = 0;
	//while (1)
	//{
	//	if (nCount >= EVENT_TIME_OUT)
	//	{
	//		eStop();
	//		m_bEstop = false;
	//		return -1;
	//	}
	//	if (m_bEstop)
	//	{
	//		eStop();
	//		m_bEstop = false;
	//		return -1;
	//	}
	//	nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
	//	if (nRet == 1)
	//	{
	//		g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
	//		nCount = 0;
	//		break;
	//	}
	//	Sleep(50);
	//	nCount ++;
	//}
	//g_nPos = INITIAL_RIGHT_HEIGHT;

	smc_set_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, 0);
	double dPos = 0.00;
	bool bMissStep = false;
	short nIn0 = 1;
	doEmptyMotion(nValue, 1);
	Sleep(100);
	while (1)
	{
		if (nCount >= EVENT_TIME_OUT)
		{
			eStop();
			m_bEstop = false;
			m_bLeftRightMove = false;
			return -1;
		}
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			m_bLeftRightMove = false;
			return -1;
		}
		nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProcRightMove 当前状态：静止！");
			m_bLeftRightMove = false;
			break;
		}
		else
		{
			nIn0 = !smc_read_inbit(m_nConnectNo, 5);
			if (!nIn0)
			{
				bMissStep = true;
				nCount = 0;
				emgStop2();
				m_bLeftRightMove = false;
				break;
			}
		}
		Sleep(50);
		nCount++;
	}

	if (bMissStep)
	{
		//doSingleMotion(20, 1);
		//Sleep(100);
		//while (1)
		//{
		//	if (nCount >= EVENT_TIME_OUT)
		//	{
		//		eStop();
		//		m_bEstop = false;
		//		m_bLeftRightMove = false;
		//		return false;
		//	}
		//	if (m_bEstop)
		//	{
		//		eStop();
		//		m_bEstop = false;
		//		m_bLeftRightMove = false;
		//		return false;
		//	}
		//	nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		//	if (nRet == 1)
		//	{
		//		g_Logger.TraceInfo("CHeroMotarControl::threadProcRightMove 当前状态：静止！");
		//		nCount = 0;
		//		m_bLeftRightMove = false;
		//		break;
		//	}
		//	Sleep(50);
		//	nCount++;
		//}

		//nIn0 = !smc_read_inbit(m_nConnectNo, 5);
		//if (nIn0)
		//{
		//	smc_get_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, &dPos);
		//	g_Logger.TraceInfo("CHeroMotarControl::threadProcRightMove Pos = %f！", dPos);
		//	dPos = abs(dPos);
		//	double dExpectedPos = nValue * ENCODER_UNIT;
		//	double dInterval = abs(dPos - dExpectedPos);
		//	if (dInterval > IGNORE_LEN)
		//	{
		//		if (dPos < dExpectedPos)
		//		{
		//			UINT nLenNew = (UINT)(dInterval / ENCODER_UNIT);
		//			smc_set_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, 0);
		//			doEmptyMotion(nLenNew, 1);
		//			Sleep(100);
		//			while (1)
		//			{
		//				if (nCount >= EVENT_TIME_OUT)
		//				{
		//					eStop();
		//					m_bEstop = false;
		//					m_bLeftRightMove = false;
		//					return -1;
		//				}
		//				if (m_bEstop)
		//				{
		//					eStop();
		//					m_bEstop = false;
		//					m_bLeftRightMove = false;
		//					return -1;
		//				}
		//				nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		//				if (nRet == 1)
		//				{
		//					g_Logger.TraceInfo("CHeroMotarControl::threadProcRightMove 当前状态：静止！");
		//					m_bLeftRightMove = false;
		//					nCount = 0;
		//					break;
		//				}
		//				Sleep(50);
		//				nCount++;
		//			}

		//			smc_get_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, &dPos);
		//			g_Logger.TraceInfo("CHeroMotarControl::threadProcRightMove Pos = %f！", dPos);
		//			dInterval = abs(dPos - dInterval);
		//			if (dInterval > IGNORE_LEN)
		//			{
		//				eStop();
		//				m_bEstop = false;
		//				m_bLeftRightMove = false;
		//				return -1;
		//			}
		//		}
		//	}
		//}
	}
	else
	{
		smc_get_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, &dPos);
		g_Logger.TraceInfo("CHeroMotarControl::threadProcRightMove Pos = %f！", abs(dPos));
		dPos = abs(dPos);
		double dExpectedPos = nValue * ENCODER_UNIT;
		double dInterval = abs(dPos - dExpectedPos);
		g_Logger.TraceInfo("CHeroMotarControl::threadProcRightMove Pos = %f, 移动到的距离%f", dPos, dInterval);
		if (dInterval > IGNORE_LEN)
		{
			if (dPos < dExpectedPos)
			{
				UINT nLen = (UINT)(dInterval / ENCODER_UNIT);
				smc_set_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, 0);
				doEmptyMotion(nLen, 1);
				Sleep(100);
				while (1)
				{
					if (nCount >= EVENT_TIME_OUT)
					{
						eStop();
						m_bEstop = false;
						m_bLeftRightMove = false;
						return -1;
					}
					if (m_bEstop)
					{
						eStop();
						m_bEstop = false;
						m_bLeftRightMove = false;
						return -1;
					}
					nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
					if (nRet == 1)
					{
						g_Logger.TraceInfo("CHeroMotarControl::threadProcRightMove 当前状态：静止！");
						m_bLeftRightMove = false;
						nCount = 0;
						break;
					}
					else
					{
						nIn0 = !smc_read_inbit(m_nConnectNo, 5);
						if (!nIn0)
						{
							nCount = 0;
							bMissStep = true;
							emgStop2();
							m_bLeftRightMove = false;
							break;
						}
					}
					Sleep(50);
					nCount++;
				}

				smc_get_encoder_unit(m_nConnectNo, PULP_OUT_AXIS, &dPos);
				g_Logger.TraceInfo("CHeroMotarControl::threadProcRightMove Pos = %f！", dPos);
				double dInterval1 = abs(abs(dPos) - dInterval);
				g_Logger.TraceInfo("CHeroMotarControl::threadProcRightMove Pos = %f！移动到的距离%f 目标位置间距%f", dPos, dInterval, dInterval1);
				if ((dInterval1 > IGNORE_LEN) && !bMissStep)
				{
					eStop();
					m_bEstop = false;
					m_bLeftRightMove = false;
					g_Logger.TraceInfo("CHeroMotarControl::threadProcRightMove Pos = %f！ 没有移动到目标位置", dPos);
					return -1;
				}
			}
		}
	}
	

	//if (!InitEndLeftRightData())
	//{
	//	return -1;
	//}
	return 0;
}

int CHeroMotarControl::threadProcPulpOut(UINT nValue)
{
	m_bPulpOut = false;
	doPulpOut(0);
	UINT nInterval = (nValue * HORIZONTAL_TRANSVERSE_300) / ONCE_MOVE_LEN;
	UINT nCount = 0;
	while (1)
	{
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return -1;
		}
		if (nCount >= nInterval)
		{
			smc_stop(m_nConnectNo, PULP_OUT_AXIS, 1);
			break;
		}
		Sleep(50);
		nCount++;
	}
	m_bPulpOut = true;
	return 0;
}

int CHeroMotarControl::threadProcRightVertical(UINT nValue)
{
#if(BLOCK_LEN == 600)
	if (!InitLeftRightData())
	{
		return -1;
	}
	doMoveLUp(6000, true);
	Sleep(100);
	short nRet = -1;
	UINT nCount = 0;
	while (1)
	{
		if (nCount >= EVENT_TIME_OUT)
		{
			eStop();
			m_bEstop = false;
			return -1;
		}
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return -1;
		}
		nRet = smc_check_done(m_nConnectNo, FRONT_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProcRightVertical 当前状态：静止！");
			nCount = 0;
			break;
		}
		Sleep(50);
		nCount ++;
	}
	doSingleMotion(600, 1);
	Sleep(100);
	short nIn0 = 1;
	while (1)
	{
		if (nCount >= EVENT_TIME_OUT)
		{
			eStop();
			m_bEstop = false;
			return -1;
		}
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return -1;
		}
		nRet = smc_check_done(m_nConnectNo, LEFT_RIGHT_MOTION_AXIS);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProcRightVertical 当前状态：静止！");
			nCount = 0;
			break;
		}
		else
		{
			nIn0 = !smc_read_inbit(m_nConnectNo, 5);
			if (!nIn0)
			{
				emgStop2();
				nCount = 0;
				break;
			}
		}
		Sleep(50);
		nCount++;
	}

	nIn0 = !smc_read_inbit(m_nConnectNo, 5);
	if (nIn0)
	{
		return -1;
	}

	thread t1(&CHeroMotarControl::threadProcPulpOut, this, nValue);
	t1.detach();

	doMoveLDown(80000, false);
	Sleep(200);
	while (1)
	{
		if (nCount >= VERTICAL_EVENT_TIME_OUT)
		{
			eStop();
			m_bEstop = false;
			return -1;
		}
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return -1;
		}
		nRet = smc_check_done(m_nConnectNo, BACK_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1 && m_bPulpOut)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			m_bPulpOut = false;
			nCount = 0;
			break;
		}
		Sleep(50);
		nCount++;
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
		if (nCount >= VERTICAL_EVENT_TIME_OUT)
		{
			eStop();
			m_bEstop = false;
			return -1;
		}
		if (m_bEstop)
		{
			eStop();
			m_bEstop = false;
			return -1;
		}
		nRet = smc_check_done(m_nConnectNo, BACK_UP_DOWN_MOTION_AXIS1);           //判断当前轴状态
		if (nRet == 1)
		{
			g_Logger.TraceInfo("CHeroMotarControl::threadProc 当前状态：静止！");
			break;
		}
		Sleep(50);
		nCount++;
	}

	if (!InitEndLeftRightData())
	{
		return -1;
	}

#endif
	return 0;
}