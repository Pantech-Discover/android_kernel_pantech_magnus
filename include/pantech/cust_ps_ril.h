#ifndef _CUST_PS_RIL_
#define _CUST_PS_RIL_

/***************************************************************************
                                 TARGET
****************************************************************************/
#define FEATURE_PS_ATT_SPECIFIC /* USA - AT&T */

/***************************************************************************
                                 COMMON
****************************************************************************/
/* 2012.2.22. kdhyun : PS Telephony Manager 정의 및 Interface 정의
              - 수정 파일 : service_manager.c, GSMPhone.java, Android.mk
              - 추가 파일 : IPSTelephony.aidl, PSInterfaceManager.java
*/
#define FEATURE_PS_TELEPHONY_MGR

/* 20110324 PS10 SUNI : FTM Mode/Debug Screen in LINUX */
#define FEATURE_PS_DEBUG_MENU


/* 2011.08.10 PS1 Soobeen: SYSTEM DEBUG MENU */
#define FEATURE_PS_SYSTEM_DEBUGMODE

/* LTE band Locking */
#define FEATURE_LTE_BAND_LOCK

/*
2012.2.22. kdhyun : QMI_SAR를 이용하여 OEM QMI interface 사용하도록 함
                    - 수정 파일 : specific_absorption_rate_v01.c, qcril_other.c
*/
#define FEATURE_SKY_CP_OEM_QMI_ACCESS

/*
2012.2.22. kdhyun : QMI를 통한 NV Interface 추가 (QMI_SAR 이용)
                    - 수정 파일 : qcril_other.c, qcril_system_debug_nv.c, qmi_proxy.c, specific_absorption_rate_v01.c/h
*/
#define FEATURE_SKY_CP_NV_ACCESS_WITH_QMI

/*
2012.2.22. kdhyun : QMI를 통한 command 추가 (QMI_SAR 이용)
                    - 수정 파일 : qmi_proxy.c, specific_absorption_rate_v01.c/h
*/
#define FEATURE_SKY_CP_OEM_COMMANDS_WITH_QMI

/*
2012.4.6. kdhyun : CTS 테스트시 CDMA 제거
                   - 수정 파일 : LINUX/android/device/qcom/common/common.mk
*/
#define FEATURE_PS_CTS_CDMA_REMOVE

#ifdef T_SKY_CP_DEBUG_LOG_FUNC
/*
    SD i¹´e?e￥??μi? DM Logging
        - android\vendor\qcom\proprietary\Dm-monitor  - added folder - 
		- XXXX_EXE.sh
		- android\build\core\config.mk
		- android\system\core\rootdir\init_XXXX.rc
		- android\pantech\apps\Dialer\src\com\android\contacts\PCUSpecialCharSequenceMgr.java
		- android\kernel\drivers\char\diag\diagchar_core.c
		- android\frameworks\base\telephony\java\com\android\internal\telephony\SkyTelephonyIntents.java - added file- 
*/
#define FEATURE_SKY_CP_DM_LOG_STORE_TEMP_MEMORY
#endif/* T_SKY_CP_DEBUG_LOG_FUNC */

/***************************************************************************
                                 AS
****************************************************************************/
/* 20101101 PSTeam Sungoh AT&T RSSI Requirement implementation
13340 V_42 <CDR-RBP-1030>*/
#define FEATURE_ATNT_RSSI_BAR

/***************************************************************************
                                 MM
****************************************************************************/
/*
2010.11.09. kdhyun : Not to send network name in case of MCC/MNC 001/01, 001/02, 002/01
                     - modified file : qcril_qmi_nas2.c
*/
#define FEATURE_PS_NOT_USED_QUALCOMM_DEFAULT

/*
2012.11.28. kdhyun : Set initial RAT to requesting RAT in manual selection mode
                     - Eons.java, GSMPhone.java, NetworkSetting.java, OperatorInfo.java, qcrili.h, qcril_qmi_nas.c, RIL.java, ril_commands.h, SystemOperatorActivity.java
                     - GsmServiceStateTracker.java
*/
#define FEATURE_PS_MANUAL_SELECTION_RAT

/*
2011.11.22. kdhyun :  Not to consider DST about display of TIMEZONE information
                     - related TC : GSM-BTR-1-9275-2, LTE-BTR-1-4212
                     - modified file :DateTimeSettings.java
*/
#define FEATURE_PS_TIMEZONE_WITHOUT_DST

/*
2012.1.2. kdhyun : When EF_SPN value is null, null is returned
                   - modified file : IccRecords.java
*/
#define FEATURE_PS_EF_SPN_CHECK

/*
2012.1.17. kdhyun : MM reject cause display for AT&T requirement
2012.7.25. byungju : when reject is #13 and #15, not display emergency call only 
                    - modified file : GsmServiceStateTracker.java, strings.xml, public.xml
*/
#ifdef FEATURE_PS_ATT_SPECIFIC
#define FEATURE_PS_MM_REJECT_CAUSE_DISPLAY
#endif

/*
2012.1.17. kdhyun : To display separately "Out of service state" or "Emergency only state"
                    - modified file : MSimSubscriptionStatus.java, RadioInfo.java, Status.java
*/
#ifdef FEATURE_PS_ATT_SPECIFIC
#define FEATURE_PS_SEPERATE_OUT_OF_SERVIE_AND_EMERGENCY_ONLY
#endif

/*
2012.1.27. kdhyun : To modify error check of LAC range of EF_PNN
                    - modified file :OplRecords.java
*/
#define FEATURE_PS_PNN_LAC_RANGE_BUG_FIX

/*
2012.3.21. kdhyun : To change Android's default pref mode from W to GWL 
                    - modified file : RILConstants.java
2013.1.24. kdhyun : EVENT_GET_PREFERRED_NETWORK_TYPE
                    - modified file :GsmServiceStateTracker.java
2013.2.4. kdhyun : LTE on CDMA
                    - modified file : BaseCommands.java
*/
#define FEATURE_PS_DEFAULT_NETWORK_MODE_GWL

/*
2012.4.5. kdhyun : to use AlphaShort when AlphaLong is null
                   - modified file : GsmServiceStateTracker.java
2013.1.16. kdhyun : handled as an exception at Settings -> Status
                   - modified file : Status.java
*/
#define FEATURE_PS_ALT_OPERATOR_NAME_USE

/*
2012.4.5. kdhyun : Not to show Restricted notification
                   - modified file : GsmServiceStateTracker.java
*/
#define FEATURE_PS_RESTRICTED_NOTIFICATION_NOT_SUPPORT

/*
2012.4.27. kdhyun : After 30145 patch, android request_id is limited as QMI_RIL_FW_ANDROID_REQUEST_HNDL_MAX_EVT_ID (127)
                    Set bigger value than 132 by adding event_id of Ril.h
                    - modified file : qcril.c, ril.h
                    - 1048 version only
*/
#define FEATURE_SKY_CP_RIL_FW_ANDROID_REQUEST_HNDL_MAX_EVT_ID_FIX

/*
2012.4.27. kdhyun : After 30145 patch, there is problem of processing EVENT due to change of ITEM processing structure related to additional RIL
                    Remove RIL EVENT related to Voice & Data REG STATE, Auto & Malual Network Selection
                    from Table "qmi_ril_fw_dedicated_thrd_exec_android_requests_set"
                    -1048 version only
*/
#define FEATURE_SKY_CP_FW_DEDICATED_ANDROID_REQ_BUG_FIX

/*
2012.04.27. cmjung : In case of setting GWL mode and data disabled, network menu display GW mode not GWL mode 
                     Sync with Modem pref mode nv(00010) value
                    - modified file : GsmServiceStateTracker.java, ServiceStateTracker.java
*/
#define FEATURE_SKY_CP_GET_CM_PERSIST_PREF_MODE

/*
2012.5.30. kdhyun : Not to operate *#*#4636#*#* -> Phone information -> Set preferred network type 
                   - modified file : RadioInfo.java, radio_info.xml
*/
#define FEATURE_PS_BLOCK_GOOGLE_NETWORK_MODE_SETTING

/*
2012.06.05. cmjung : deleted 'No service' in Lock screen
                    - modification file : KeyGuardUpdateMonitor.java, CarrierLabel.java (by US3 Team)
*/
#define FEATURE_PS_NO_SVC_BLOCK

/*
2012.6.13. kdhyun : To get correct hard-coding operator name during mixed MCC 404, 405 in India
                   - modified file : qcril_qmi_nas2.c
*/
#define FEATURE_PS_REORDERING_MCC_404_405

/*
2012.6.29. cmjung : modified the network name  mismatch in notification after reject from network.
                    - modified file : GSMPhone.java, NotificationMgr.java
*/
#define FEATURE_PS_MATCH_NETWORK_NAME_IN_NOTIFICATION_AFTER_REJECT

// To modify display error of network name and manual search list by Function "centralized_eons_supported" (from 153613 version)
#define FEATURE_SKY_CP_CENTRALIZED_EONS_NOT_SUPPORTED

/*
2012.11.30. kdhyun : Network mode did not changed via QXDM tool after power cycling. (JB bug)
                     - modified file : GsmServiceStateTracker.java, RIL.java
*/
#define FEATURE_PS_BUG_FIX_NETWORK_MODE_IN_JB

/*
2013.06.10. BRKIM : (MAGNUS JB only) To block LTE and CS only mode 
                     - modified file : SystemDebugMode.java
*/
#define FEATURE_PS_BLOCK_LTE_CS_OF_DEBUG_MENU

/***************************************************************************
                                 CC
****************************************************************************/
/*ygkim 2012.1.3 AT&T Local CLIR
Added 10776 TC & Local CLIR for AT&T, for PTCRB CLIR TC, this feature have to be blocked.
*/
#define FEATURE_GET_CLIR_LOCAL
/*Eunseong, 2012.1.3 Error Cause by FDN Enabled
Processed USSD FDN Check Failure as Not Unsolicited res But OnRequestComplete
*/
#define FEATURE_PS_USSD_FDN_CHECK_ERROR_RELAY
/*ygkim 2012.1.3 Qualcomm bug fix
Shortened timing of airplane mode change (on/off), since PRESTO GB  
RADIO_POWER 에 대해서 MDM으로 부터 Callback message가 QMI interface에서 사라지는 현상이 확인 됨 
이 경우 처리를 위해  10초 timer 만료 후 RADIO_POWER complete를 보내고, Qcril 관련 state를 update 한다.
*/
//#define FEATURE_RADIO_POWER_CB_SUCCESS

/*hnlee 2012.1.3 UICC Check for call waiting
Blocked to access call forwarding and call waiting menu on without SIM.
*/
//#define FEATURE_SIM_PRESENT_CHECK_FOR_CALL_SETTING 
/*hnlee 2012.1.3 Call end Reason 
Added Call End raeson for Reject Indication
Added Emergency call fail cause
*/
#define FEATURE_ADD_CALL_END_REASON
#define FEATURE_PRESTO_ADD_ECC_FAIL_CAUSE

/*ygkim 2012.1.3 Qulaocmm bug fix
Ref> Presto GB DTMF issue related to critical section 
dial xxxx P(W) xxxxxxxx DTMF state machine 동작 오류 ; DTMF Start Resp 이후 token id  delete 후  DTMF Stop Req Event 처리 하면서 token id 새로 등록 되는 순서에 
unsync 현상 발생해서, token id delete를 하기전에 , DTMF Stop Req Event 가 먼저 처리 되면서, token id 등록 못하는 현상 발생
dial xxxx P(W) xxxxx 에서는  > DTMF 를 보내 준다. 
active call 상태에서 DTMF 를 보내는 것은 >DTMF Start ,  > DTMF Stop을 보내 주는 경우는 영향이 없다.
*/
//#define FEATURE_DTMF_EVENT_UNSYNC
/* SUNI 20120215 OSCAR PLM 317
  After the call was ended at local, the state 'ACTIVE' is returned in result to get current call 
  Eunseong, Juhyun 2012. 06.09, 
  modified call active state report symptom after call hangup, because this makes multicall-not-end issue.
 */
#define FEATURE_PS_CC_RETURN_NULL_DURING_CALL_DISCONNECTING
/* Eunseong, Juhyun, 20120320, Incoming call ring issue.
Fixed Qualcomm bug whichi is not sending UNSOL_CALL_RING
Blocked this feature due to release Qualcomm patch
*/
//#define FEATURE_INCOMING_ALERTING_UNSOL_REP
/*Eunseong, 20120330, SKY feature
Made call cause update at first call after registrataion,
*/
#define FEATURE_SKY_CP_LAST_CALL_FAILURE_FORCED_INIT
/*Eunseong, 20120418 
  USSD failure return in case of not getting respond of NW
*/
#define FEATURE_PS_USSD_FAIL_RETURN_NW_NOT_RESPONSE
/*Eunseong, SKY feature
When ps service is only in service state,  cs domain sevice will be no service, and no service icon without this feature
*/
#define FEATURE_SKY_CP_SUPPORT_PS_ONLY_MODE

/*
2012.06.13, From 8th MT missed call, MT call couldn't be forwarded to UI on qcril 1534 bug, it is a temporary code 
SKY feature,
Blocked this feature because of releasing CR370978 
*/
//#define FEATURE_SKY_CP_MISSED_CALL_BUG_FIX

/*Juhyun, Eunseong 2012.05.17 
Added not to use QCRIL.c Flow control 
becasue Qualcomm qcril flow control couldn't process same name-ril requests which occured fast continuosly
*/
#define FEATURE_PS_CC_DTMF_BLOCK_FLOW_CONTROL

/* juhyun 2021.10.10 MAGNUS 
Except  " timeout value( 2000 -->5000)"
ygkim 2012.05.31 OSCAR
we soemtimes have no return value to qmi_client_send_msg_sync(QMI_DMS_SET_OPERATING_MODE_REQ_V01) callback. but Modem's operating mode action is success.
this is unsync issue between modem and Android.
if this is failure by timeout for callback. we change failure to success. also change timeout value( 2000 -->5000) 
*/
#define FEATURE_PS_HANDLE_QMI_TIMEOUT_SET_MODEM_OP_MODE

/* 2013.01.18 Heesung : To fix CSFB immediate call end */
//#define FEATURE_SKY_CP_1X_CSFB_RETRY_BUG_FIX

/***************************************************************************
                                 Data
****************************************************************************/
/* cust_ps_data.h */

/***************************************************************************
                                 SIM
****************************************************************************/
/*ygkim 2012.1.3 PIN retry flag
SIM State change 정보에 SIM Retry 정보를 포함해서 전달.
*/
#define FEATURE_PS_PIN_RETRY_INIT

/*ygkim 2012.1.3 virtual sim flag
SIM State Change 정보에 Virtual sim 정보를 포함해서 전달한다.
SIM Manager 생성 및 Factory Command 과정에서 ECC List handling에 이용 
*/
#define FEATURE_PS_IS_VIRTUAL_SIM

#ifdef FEATURE_PS_IS_VIRTUAL_SIM
/*ygkim 2013.1.1 VIRTUAL_SIM for QCCI 
changed QMI legacy  to UIM over QCCI(QMI Common Clinet Interface)
:include virtual_sim member vlaue to IDL Service_X.c , .h file
*/
#define FEATURE_QCRIL_VIRTUAL_SIM_FLAG_BY_QCCI
#endif

/*ygkim 2012.4.16 CSIM or RUIM exception error  -->UiccCardApplication.java
CSIM or RUIM card type이 활성 화 될 경우 ,  Android Frameworks 작업에 exception handling이 발생함.
인식 안 하도록 수정. 
iccCardApplicationStatus returns App type , Ril.java do not update CSIM or RUIM app type status
*/
#define FEATURE_DISABLE_CSIM_RUIM

/*ygkim 2012.1.3 Sim State structure
Android 와 Qcril 사이의 SIM State Structure 정의 값 통일 
*/
#define FEATURE_PS_ILLEGAL_SIM_STATE_MISMATCH

/*<---Start UICC Card Power down handling */
/*ygkim 2012.1.3  Card Power down 
set Phone booting Sequence without Card power down action. 
1) ~1045 Patch Source: this feature Disable
2) 1048 Patch Source ~ 1053,~ : this feature Enabled 
*/
#define FEATURE_QMI_NO_CARD_POWER_DOWN_IN_AIRPLANE

/*ygkim 2012.1.3 Card Power down
Q8960: airplane on /off is handling without Card Power down/up action 
1)~1045 Patch Socure: This feature Enabled
2)1048~ 1053,~:this feature Enabled but code is modified
*/
#define FEATURE_RADIO_POWER_WITHOUT_CARD_PWDL

/*ygkim 2012.2.27 Phone power on without No Card Power down 
Q8960 :qcril_qmi_nas_is_apm_enabled() APM handling modifed
*/
#define FEAURE_APM_DISABLED
/*<---End UICC Card Power down  handling  */

/*ygkim 2012.2.27 Limited Card error 
정상 적인 Card Error 상태가 아닌대 Card error event가 trigger  되는 경우 android interface에서 check 해서
event 를 전달 하도록 수정.
*/
#define FEAURE_LIMITED_CARD_ERROR_TRIGGER

/*ygkim 2012.03.26  
Andorid 에서 보내는 STK RIL Ready 수신 후 First proactive command(SETUP) 전송을 위해 사용 되는 RESEND timer(5sec)를 수정
5sec delay 없이 바로 보내도록 수정.(0.001초 후 전달 )
*/
#define FEATURE_PS_CHG_RESEND_TIMER

/*<---Start UICC debug menu handling--->*/
/*ygkim 2012.1.3 Debug menu interface
Android OS ???? IMPU ???? ?о? ????? interface ???? 
*/
#define FEATURE_PS_IMPU_READ

/*ygkim 2012.1.3 Debug menu interface
Android OS???? ISIM ???? ?о? ???? interface ???? 
*/
#define FEATURE_PS_ISIM_READ

/*ygkim 2012.4.17 Card Status debug
IccCardApplicationStatus.java????????  Card Status Raw Data Trace??| A??C?? Radio Log???? PRINT??| CN??U.
*/
#define RIL_UICC_DEBUG_PRINT
/*<---End UICC Debug menu handling--->*/

/*hnlee 2013.4.15
Check sim type using getCardType(), and return hasIccCard value. If virtual sim or No sim then return false
*/
#define FEATURE_PS_CHECK_SIM_TYPE
#define FEATURE_PS_VIRTUAL_SIM_CHECK

/***************************************************************************
                                 SIM-WIFI
****************************************************************************/

/****************************************************************************
					NFC-BIP(ISIS) AT&T , VZW
***************************************************************************/
/*ygkim 2012.05-02 Android NFC interface
add NFC-BIP interfce_PS1
*/
#define FEATURE_QCRIL_UIM_QMI_APDU_ACCESS 
#ifdef FEATURE_QCRIL_UIM_QMI_APDU_ACCESS
/*NFC -BIP  RIL command handling
1)2012.05-02  RIL_REQUEST_SIM_APDU -->RIL_REQUEST_SIM_TRANSMIT_BASIC 
2)2012.05.14 Add_Rild reset Patch  
3)2012.05.20 include Smart card API request message for capability  handling to do flow control
*/
#define FEATURE_PANTECH_SMART_CARD_API

#endif
/**ygkim 2012.05-07 Android NFC interface
add NFC-BIP interfce 
info : Change error value from RIL_E_GENERIC_FAILURE 
       to RIL_E_MISSING_RESOURCE, 
          RIL_E_NO_SUCH_ELEMENT, 
          RIL_E_INVALID_PARAMETER
       or RIL_E_GENERIC_FAILURE 
when : 20120503
who  : JKA
*/
#define FEATURE_P_UICC_APDU_ACCESS_RESP_DETAIL
/*ygkim    2013.06.25
add send APDU qmi msg for NFC
for dividing into two send apu group( for NFC, for WIFI ).
#define FEATURE_PS_BUG_FIX_SEND_APDU_FOR_WIFI_AKA Modem
*/
#define FEATURE_PS_SEND_APDU_FOR_NFC
#endif //CUST_PS_RIL_H