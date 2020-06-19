#include "HeroMotarTcp.h"
#include<windows.h>

CHeroMotarTcp::CHeroMotarTcp(unsigned short nPort)
{
    m_nSock = 0;
	m_nTry = 0;
    m_nPort = nPort;
    m_bLoop = true;
    m_bWSAStart = false;
    m_nSocket = INVALID_SOCKET;
    m_DealMsgCallback = nullptr;
    Init();
}

CHeroMotarTcp::~CHeroMotarTcp()
{
	Disconnect();
    if (m_bWSAStart)
    {
        WSACleanup();
    }
}

void CHeroMotarTcp::ThreadProc()
{
    fd_set fdRead, fdWrite, fdExcept;
    while (m_bLoop)
    {
        //分析conns中所有客户连接并把它们放到合适的fd_set中
        resetFDset(fdRead, fdWrite, fdExcept);
        //调用select，等待就绪的套接字I/O操作
        int nRet = select(0, &fdRead, &fdWrite, &fdExcept, NULL);
        if (nRet <= 0)
        {
            //cout << "select error" << WSAGetLastError() << endl;
            printf("CRobotTcpProtocol::ThreadProc select error\n");
            break;
        }
        //检查是否有新的客户连接请求
        nRet = checkAccept(fdRead, fdExcept);
        if (nRet == SOCKET_ERROR)
        {
            break;
        }
        //检查客户连接
        checkConn(fdRead, fdWrite, fdExcept);
    }
    //释放资源
    for (auto it = m_vConnection_Socket.begin(); it != m_vConnection_Socket.end(); ++it)
    {
        closesocket((*it)->hSocket);
        delete* it;
    }
    if (m_nSocket != INVALID_SOCKET)
        closesocket(m_nSocket);

}

//分析conns中的客户连接，根据分析结果填充各个fd_set
void CHeroMotarTcp::resetFDset(fd_set& fdRead, fd_set& fdWrite, fd_set& fdExcept)
{
    //首先清空各个fd_set
    FD_ZERO(&fdRead);
    FD_ZERO(&fdWrite);
    FD_ZERO(&fdExcept);
    //监听套接字每次都被放入fdRead，也就是说每次调用select都会去检测是否有新的连接请求
    FD_SET(m_nSocket, &fdRead);
    FD_SET(m_nSocket, &fdExcept);
    for (int i = 0; i < m_vConnection_Socket.size(); i++)
    {
        //客户连接的缓冲区还有空间，可以继续接收数据，就需要把其对应的套接字句柄放入fdRead中
        if (m_vConnection_Socket[i]->nBytes < BUF_LEN)
        {
            FD_SET(m_vConnection_Socket[i]->hSocket, &fdRead);
        }
        //客户连接的缓冲区还有数据需要发送，就需要把其对应的套接字句柄放入到fdWrite中
        if (m_vConnection_Socket[i]->nBytes > 0)
        {
            FD_SET(m_vConnection_Socket[i]->hSocket, &fdWrite);
        }
        //每个连接的套接字句柄都需要放到fd_Except中，以便select 能够检测其I/O错误
        FD_SET(m_vConnection_Socket[i]->hSocket, &fdExcept);
    }
}


//检查是否有新的客户连接
int CHeroMotarTcp::checkAccept(const fd_set& fdRead, const fd_set& fdExcept)
{
    int nErr = 0;
    //首先检查是否发生错误
    if (FD_ISSET(m_nSocket, &fdExcept))
    {
        int errlen = sizeof(nErr);
        getsockopt(m_nSocket, SOL_SOCKET, SO_ERROR, (char*)&nErr, &errlen);
        printf("CRobotTcpProtocol::checkAccept lastErr\n");
        return SOCKET_ERROR;
    }
    //检查fdRead中是否包含了监听套接字，如果是，则证明有新的连接请求，可以用accept
    if (FD_ISSET(m_nSocket, &fdRead))
    {
        //由于fd_set有大小限制（最多为FD_SETSIZE）,所以当已有客户连接到达这个限制时，不再接受连接请求
        if (m_vConnection_Socket.size() >= FD_SETSIZE - 1)
        {
            return 0;
        }
        //调用accept来接受连接请求
        sockaddr_in saRemote;
        int nSize = sizeof(sockaddr_in);
        SOCKET sSocket = accept(m_nSocket, (sockaddr*)&saRemote, &nSize);
        nErr = WSAGetLastError();
        //为了程序的健壮性，检查WSAEWOULDBLOCK错误
        if (sSocket == INVALID_SOCKET && nErr != WSAEWOULDBLOCK)
        {
            printf("CRobotTcpProtocol::checkAccept 接收错误: lastErr\n");
            return SOCKET_ERROR;
        }
        if (sSocket != INVALID_SOCKET)
        {
            //获取了新的客户连接套接字句柄，需要把它设为非阻塞模式，以便对其使用select函数
            u_long nNoBlock = 1;
            if (ioctlsocket(sSocket, FIONBIO, &nNoBlock) == SOCKET_ERROR)
            {
                printf("CRobotTcpProtocol::checkAccept ioctlsocket error\n");
                return SOCKET_ERROR;
            }
            //为新的客户连接创建一个Connection对象，并且加入到conns中去
            m_vConnection_Socket.push_back(new S_Connection_Socket(sSocket));
        }
    }
    return 0;
}

//检查当前所有客户连接，看看它们是否可读（recv），可写（send），还是I/O错误
void CHeroMotarTcp::checkConn(const fd_set& fdRead, const fd_set& fdWrite, const fd_set& fdExcept)
{
    auto iter = m_vConnection_Socket.begin();
    while (iter != m_vConnection_Socket.end())
    {
        bool bOk = true;
        //检查当前连接是否发生I/O错误
        S_Connection_Socket* pConn = *iter;
        if (FD_ISSET(pConn->hSocket, &fdExcept))
        {
            bOk = false;
            int nErr = 0;
            int nLen = sizeof(nErr);
            getsockopt(pConn->hSocket, SOL_SOCKET, SO_ERROR, (char*)&nErr, &nLen);
            printf("CRobotTcpProtocol::checkConn I/O error\n");
        }
        else
        {
            //检查当前连接是否可读
            if (FD_ISSET(pConn->hSocket, &fdRead))
            {
                bOk = tryRead(pConn);
            }
        }
        //发生了错误，关闭套接字并把其对应的Connection对象从conns中移除
        if (bOk == false)
        {
            closesocket(pConn->hSocket);
            delete pConn;
            iter = m_vConnection_Socket.erase(iter);
        }
        else
        {
            ++iter;
        }
    }
}

//select成功返回并标明recv就绪，调用recv接收数据
bool CHeroMotarTcp::tryRead(S_Connection_Socket * pConn)
{
    //pConn->buffer+pConn->nBytes表示缓冲区Buffer可用空间的开始位置
    //buf_len-pConn->nBytes表示缓冲区Buffer可用空间的大小
    int nRet = recv(pConn->hSocket, pConn->cBuf, BUF_LEN, 0);
    if (nRet > 0)
    {
        if (m_DealMsgCallback)
        {
            m_DealMsgCallback(pConn->cBuf, strlen(pConn->cBuf), pConn->hSocket);
        }
        memset(pConn->cBuf, 0, sizeof(pConn->cBuf));
        return true;
    }
    //对方关闭了连接，调用被动安全关闭连接PassiveShutdown函数
    else if (nRet == 0)
    {
        printf("CRobotTcpProtocol::tryRead 对方关闭连接\n");
        //shutConn(pConn->hSocket, pConn->cBuf, pConn->nBytes);
        closesocket(pConn->hSocket);
        pConn->hSocket = INVALID_SOCKET;
        return false;
    }
    //发生了错误。为了程序的健壮性，检查WSAEWOULDBLOCK错误
    else
    {
        int nErr = WSAGetLastError();
        if (nErr == WSAEWOULDBLOCK)
        {
            return true;
        }
        printf("CRobotTcpProtocol::tryRead 接收失败\n");
        return false;
    }
}

bool CHeroMotarTcp::tryWrite(char* pData, unsigned int nLen, unsigned int nSocket)
{
    int nSent = send(nSocket, pData, nLen, 0);
    if (nSent > 0)
    {
        //pConn->nBytes -= nSent;
        ////Buffer中还有数据尚未发送出去
        //if (pConn->nBytes > 0)
        //{
        //    //把尚未发送的数据从Buffer的尾部移动到Buffer头部
        //    memmove(pConn->cBuf, pConn->cBuf + nSent, pConn->nBytes);
        //}
        return true;
    }
    //对方关闭了连接，调用了被动安全关闭连接PassiveShutdown函数
    else if (nSent == 0)
    {
        printf("CRobotTcpProtocol::tryWrite 对方关闭连接\n");
        closesocket(nSocket);
        nSocket = INVALID_SOCKET;
        return false;
    }
    //发生了错误，为了程序的健壮性，检查WSAEWOULDBLOCK错误
    else
    {
        int lastErr = WSAGetLastError();
        if (lastErr == WSAEWOULDBLOCK)
        {
            return true;
        }
        return false;
    }
}

bool CHeroMotarTcp::tryWrite(S_Connection_Socket* pConn)
{
    int nSent = send(pConn->hSocket, pConn->cBuf, pConn->nBytes, 0);
    if (nSent > 0)
    {
        pConn->nBytes -= nSent;
        //Buffer中还有数据尚未发送出去
        if (pConn->nBytes > 0)
        {
            //把尚未发送的数据从Buffer的尾部移动到Buffer头部
            memmove(pConn->cBuf, pConn->cBuf + nSent, pConn->nBytes);
        }
        return true;
    }
    //对方关闭了连接，调用了被动安全关闭连接PassiveShutdown函数
    else if (nSent == 0)
    {
        printf("CRobotTcpProtocol::tryWrite 对方关闭连接\n");
        closesocket(pConn->hSocket);
        pConn->hSocket = INVALID_SOCKET;
        return false;
    }
    //发生了错误，为了程序的健壮性，检查WSAEWOULDBLOCK错误
    else
    {
        int lastErr = WSAGetLastError();
        if (lastErr == WSAEWOULDBLOCK)
        {
            return true;
        }
        return false;
    }
}

int CHeroMotarTcp::Init()
{
    //初始化套接字库
    WORD w_req = MAKEWORD(2, 2);//版本号
    WSADATA wsadata;
    int err = WSAStartup(w_req, &wsadata);
    if (err != 0)
    {
        printf("CRobotTcpProtocol::Connect WSAStartup failed\n");
        return -1;
    }
    //检测版本号
    if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2)
    {
        WSACleanup();
        printf("CRobotTcpProtocol::Connect check version failed\n");
        return -1;
    }
    m_bWSAStart = true;
    return 0;
}

int CHeroMotarTcp::createListen()
{
    if (m_nSocket != INVALID_SOCKET)
    {
        closesocket(m_nSocket);
        m_nSocket = INVALID_SOCKET;
    }

    m_nSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(m_nSocket == INVALID_SOCKET)
	{
		return -1;
	}
	
    SOCKADDR_IN si = { 0 };
    si.sin_family = AF_INET;
    si.sin_port = htons(TCP_PORT);
    si.sin_addr.s_addr = INADDR_ANY;
    if (bind(m_nSocket, (LPSOCKADDR)&si, sizeof(si)) == SOCKET_ERROR)
    {
        closesocket(m_nSocket);
        m_nSocket = INVALID_SOCKET;
        return -1;
    }
    if (listen(m_nSocket, SOMAXCONN) == SOCKET_ERROR)
    {
        closesocket(m_nSocket);
        m_nSocket = INVALID_SOCKET;
        return -1;
    }

    u_long nNoBlock = 1;
    if (ioctlsocket(m_nSocket, FIONBIO, &nNoBlock) == SOCKET_ERROR)
    {
        printf("CRobotTcpProtocol::createListen ioctlsocket 错误\n");
    }
    std::thread tThreadObj(&CHeroMotarTcp::ThreadProc, this);
    tThreadObj.detach();
	return 0;
}

int CHeroMotarTcp::Disconnect()
{
    m_bLoop = false;
    Sleep(1);
    if(m_nSock != 0)
	{
        closesocket(m_nSock);
        WSACleanup();
        m_nSock = 0;
	}
    printf("CRobotTcpProtocol::Disconnect AAAAAAAAAA\n");
	return 0;
}

int CHeroMotarTcp::LoopSend(char *pBuf, uint nSize, uint nSocket)
{
    if(nSocket <= 0)
    {
        printf("CRobotTcpProtocol::LoopSend tcp is disconnected AAAAAAAAAA\n");
        return -1;
    }
    g_Logger.TraceInfo("CHeroMotarTcp::LoopSend msg:%s", pBuf);
    int nRemian = nSize;
    int nSendlen = 0;
    int nRet = 0;
    while(nRemian > 0)
    {
        printf("CRobotTcpProtocol::LoopSend remian = %d m_nSock = %d\n", nRemian, m_nSock);
        nRet = send(nSocket, pBuf + nSendlen, nRemian, 0);
        if(nRet <= 0)
        {
            ++ m_nTry;
            continue;
        }
        else
        {
            m_nTry = 0;
        }
        nSendlen += nRet;
        nRemian -= nRet;
        if(m_nTry > 3)
        {
            printf("CRobotTcpProtocol::LoopSend connect break\n");
            break;
        }
    }
    return 0;
}
	
int CHeroMotarTcp::SetDealMsgCallback(OnDealMsgCallback msgCallback)
{
    m_DealMsgCallback = msgCallback;
    return 0;
}




