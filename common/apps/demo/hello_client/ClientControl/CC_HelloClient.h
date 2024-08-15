
// ClientControl.h : main header file for the PROJECT_NAME application
//

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


// CClientControlApp:
// See ClientControl.cpp for the implementation of this class
//

class CClientControlApp : public CWinApp
{
public:
	CClientControlApp();

// Overrides
public:
	virtual BOOL InitInstance();

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern CClientControlApp theApp;