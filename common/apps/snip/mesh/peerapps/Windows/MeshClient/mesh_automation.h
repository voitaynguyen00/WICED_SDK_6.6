#pragma once

#include "afxwin.h"
#include "afxdlgs.h"

typedef struct
{
	int is_gatt_proxy;
	int is_friend;
	int is_relay;
	int send_net_beacon;
	int relay_xmit_count;
	int relay_xmit_interval;
	int default_ttl;
	int net_xmit_count;
	int net_xmit_interval;
	int publish_credential_flag;       ///< Value of the Friendship Credential Flag
	int publish_ttl;                   ///< Default TTL value for the outgoing messages
	int publish_period;                ///< Period for periodic status publishing
	int publish_retransmit_count;      ///< Number of retransmissions for each published message
	int publish_retransmit_interval;   ///< Interval in milliseconds between retransmissions
} device_config_params_t;


class CSocketWindow : public CWnd
{
    DECLARE_DYNAMIC(CSocketWindow)
public:
    afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
    BOOL Create();
protected:
    //{{AFX_MSG(COptions)
    //afx_msg void OnApplyNow();
    //afx_msg void OnOK();
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
    LRESULT OnSocketMessage(WPARAM wParam, LPARAM lParam);
};