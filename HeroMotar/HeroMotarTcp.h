#ifndef _HERO_MOTAR_TCP_H_
#define _HERO_MOTAR_TCP_H_

//#include <string>
#include "HeroMotarTypes.h"
#include <thread>

typedef int (*OnDealMsgCallback)(char* pData, unsigned int nLen);

class CHeroMotarTcp
{
public:
    CHeroMotarTcp(ushort nPort);
    ~CHeroMotarTcp();
    void ThreadProc();
    int LoopSend(char *pBuf, uint nSize);
    int createListen();
    void resetFDset(fd_set& fdRead, fd_set& fdWrite, fd_set& fdExcept);
    int checkAccept(const fd_set& fdRead, const fd_set& fdExcept);
    void checkConn(const fd_set& fdRead, const fd_set& fdWrite, const fd_set& fdExcept);
    bool tryRead(S_Connection_Socket* pConn);
    bool tryWrite(S_Connection_Socket* pConn);
    int SetDealMsgCallback(OnDealMsgCallback msgCallback);
    //bool shutConn(SOCKET sSocket, const char* cbuff, int nLen);
private:
    
    int Disconnect();
    int Init();
private:
    uint m_nSock;
    int m_nTry;
    ushort m_nPort;
    HANDLE m_hThread;
    bool m_bLoop;
    bool m_bWSAStart;
    SOCKET m_nSocket;
    vector<S_Connection_Socket*> m_vConnection_Socket;
    OnDealMsgCallback  m_DealMsgCallback;
};

#endif
