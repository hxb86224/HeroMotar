
// HeroMotarDlg.h: 头文件
//

#pragma once


// CHeroMotarDlg 对话框
class CHeroMotarDlg : public CDialogEx
{
// 构造
public:
	CHeroMotarDlg(CWnd* pParent = nullptr);	// 标准构造函数

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_HEROMOTAR_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg void OnTimer(UINT nIDEvent);
	afx_msg void OnDestroy();
	afx_msg void OnButtonDo();
	afx_msg void OnButtonStop();
	afx_msg void OnButtonDo2();
	afx_msg void OnButtonDo3();
	afx_msg void OnButtonDo4();
	afx_msg void OnButtonDo6();
	afx_msg void OnButtonDo7();
	afx_msg void OnButtonDo8();
	afx_msg void OnButtonDo9();
	DECLARE_MESSAGE_MAP()
};
