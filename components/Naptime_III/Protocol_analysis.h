#ifndef _PROTOCOL_ANALYSIS_H_
#define _PROTOCOL_ANALYSIS_H_

#define Nap_Tool_Gotofactorytest      0x10
#define Nap_Tool_chargingnormal       0x11
#define Nap_Tool_chargeovernormal     0x12
#define Nap_Tool_chargingerror        0x13
#define Nap_Tool_chargeovererror      0x14
#define Nap_Tool_appconnectnap        0x15

#define Tool_Nap_charging             0x21
#define Tool_Nap_chargeover           0x22

#define Nap_App_send_deviceid         0x31
#define Nap_App_send_SN               0x32
#define Nap_App_send_userid           0x33
#define Nap_App_keyPress              0x34

#define App_Nap_write_deviceid        0x41
#define App_Nap_write_SN              0x42
#define App_Nap_useriddelete          0x43
#define App_Nap_Gotoledtest           0x44
#define App_Nap_Poweroff              0x45
#define App_Nap_Start1291             0x01
#define App_Nap_Stop1291              0x02
#define App_Nap_SystemTest            0x48
#define App_Nap_Disconnect            0x49

#define Tool_App_Toolconnectnap       0x50
#define Tool_App_signedout            0x51
#define Tool_App_charging_current     0x52
#define Tool_App_chargeover_current   0x53
#define Tool_App_RLG_vol              0x54
#define Tool_App_chargingnormal       0x55
#define Tool_App_chargeovernormal     0x56
#define Tool_App_chargingtesterror    0x57
#define Tool_App_advdata              0x58
#define Tool_App_Tooldisconnectnap    0x59

#define App_Tool_appconnectnap        0x60
#define App_Tool_lofftest             0x61

#endif
