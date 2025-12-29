#ifndef DRI_TEST_CMD
#define DRI_TEST_CMD

#include "fi_sdio.h"

#ifdef CONFIG_SDIO_IF

//================cmd=========================================

#define CMD_GET (0x80)
#define BCNInterval_Cmd 0x1
#define Bcn_Frm_Addr_Cmd  (CMD_GET|0x2)
#define Power_Save_Cmd 0x4
#define Send_NullData_Cmd 0x5
#define Keep_Alive_Cmd 0x6
#define Beacon_Miss_Cmd 0x7
#define CAM_Mode_Cmd 0x8
#define ARP_Agent_Cmd 0x9
#define Add_Pattern_Cmd 0xa
#define WoW_Enable_Cmd 0xb
#define AC_BE_Param_Cmd 0xc
#define AC_BK_Param_Cmd 0xd
#define AC_VI_Param_Cmd 0xe
#define AC_VO_Param_Cmd 0xf
#define CWmin_Cwmax_Cmd 0x10
#define LongRetryLimit_Cmd 0x11
#define ShortRetryLimit_Cmd 0x12
#define NDP_Anncmnt_Cmd 0x13
#define BmfmInfo_Cmd 0x15
#define UniCast_Key_Set_Cmd 0x16
#define Multicast_Key_Set_Cmd 0x17
#define Reset_Key_Cmd 0x18
#define Rekey_Data_Set_Cmd 0x19
#define MaxTxLifetime_Cmd 0x20
#define dhcp_start_cmd 0x21
#define vsdb_set_cmd 0x22
#define Beacon_Enable_Cmd 0x24
#define CHANNEL_SWITCH_CMD 0x25
#define DPD_MEMORY_CMD 0x26
#define FWLOG_MODE_CMD 0x27
#define CALI_PARAM_CMD 0x28
#define PHY_INTERFACE_CMD 0x29
#define ADDBA_OK_CMD 0x2a
#define WF2G_TXPWR_PARAM_CMD 0x2b
#define WF5G_TXPWR_PARAM_CMD 0x2c
#define CALI_PARAM_EX_CMD 0x2d
#define TXPWR_CFFC_CFG_CMD 0x2e
#define RegStaID_CMD 0x30
#define UnRegStaID_CMD 0x31
#define UnRegAllStaID_CMD 0x32
#define Del_BA_CMD 0x34
/*set_page_len_cmd*/
#define SET_PAGE_LEN_CMD 0x33
#define EXTERN_CHANNEL_STATUS_CMD (CMD_GET|0x35)
#define RD_SUPPORT_CMD 0x3a
#define MAC_SCAN_CMD 0x42
#define PHY_DEBUG (CMD_GET|0x44)
#define P2P_OPPPS_CWEND_IRQ_ENABLE_CMD 0x45
#define P2P_NoA_ENABLE_CMD 0x46
#define GET_TSF_CMD (CMD_GET|0x47)
#define GET_PN_CMD (CMD_GET|0x48)
#define GET_SDIO_PTR_CMD (CMD_GET|0x49)
#define Mac_Rtc_Cmd 0x4a
#define COEXIST_CMD 0x4B
#define EFUSE_CFG_CMD 0x4C
#define COEX_WF_ZGB_MODE_CMD 0X4D
#define TX_POWER_PT_CMD 0X4E
#define HOST_SHUTDOWN_REQ 0x4F
#define CF_END_CMD 0x50
#define KEY_ENTRY_READ_CMD 0x51
#define GET_QUEUE_DEBUG_INFO_CMD (CMD_GET | 0x52)
#define TXT_SHIFT_CFG_CMD 0x53
#define SCAN_SCH_START_CMD 0x54
/*coexist cmd1 comand*/
#define COEXIST_EN_CMD  BIT(0)
#define COEXIST_MAX_MISS_BCN_CNT  BIT(1)
#define COEXIST_REQ_TIMEOUT  BIT(2)
#define COEXIST_NOT_GRANT_WEIGHT  BIT(3)
#define COEXIST_FW_RETRY_WEIGHT  BIT(4)
#define COEXIST_MAX_NOT_GRANT_CNT  BIT(5)
#define COEXIST_SCAN_PRIORITY_RANGE  BIT(6)
#define COEXIST_BE_BK_NOQOS_PRI_RANGE  BIT(7)
#define COEXIST_INFOR_BT_WIFI_WORK_FREQ  BIT(8)
#define COEXIST_PARAM_CMD_CONFIG  BIT(9)

//Reset_Key_Cmd
#define ALL_KEY_RST 0xffff
#define MULTI_KEY_RST 0xeeee

//Reset_Key_Cmd
#define ALL_UNIKEY_RST 0xeeee
#define ALL_MULTIKEY_RST 0xdddd

//efuse_cfg_param flag
#define EFUSE_CFG_DISABLE 0
#define EFUSE_CFG_ENABLE 1

#pragma pack(1)

struct tx_trb_info
{
    /* The number of pages needed for a single transfer */
    unsigned int trb_num;
    /* Total length of transmission */
    unsigned int total_len;
    /* Actual size used for each page */
    unsigned short buffer_size[128];
};

//Set Page_Len Cmd
typedef struct PagelenCmd
{
    unsigned char Cmd;
    unsigned char reserve[3];
    unsigned int page_len;
} PagelenCmd;


//
///BCNInterval_Cmd
//
typedef struct BCNIntervalCmd
{
    unsigned char Cmd;
    unsigned char reserve[2];
    unsigned char vid;
    unsigned int interval;
} BCNIntervalCmd;
//
///Bcn_Frm_Addr_Cmd
//
typedef struct BcnFrmAddrCmd
{
    unsigned char Cmd;
    unsigned char reserve[2];
    unsigned char vid;
    unsigned int address;
} BcnFrmAddrCmd;
//
///Beacon_Enable_Cmd
//
typedef struct BeaconEnableCmd
{
    unsigned char Cmd;
    unsigned char reserve[2];
    unsigned char vid;
    unsigned char enable;
} BeaconEnableCmd;

typedef struct PowerSaveCmd
{
    unsigned char Cmd;
    unsigned char reserve[2];
    unsigned char vid;
    unsigned char psmode;
} PowerSaveCmd;
typedef struct CAMModeCmd
{
    unsigned char Cmd;
    unsigned char reserve[2];
    unsigned char vid;
    unsigned char cammode;
} CAMModeCmd;

typedef struct DHCPStartCmd
{
    unsigned char Cmd;
    unsigned char reserve[2];
    unsigned char vid;
    unsigned int ip;
} DHCPStartCmd;

//Del_BA_CMD
typedef struct
{
    unsigned char Cmd;
    unsigned char reserve[2];
    unsigned char STA_ID;
    unsigned char TID;
} Station;

//EXTERN_CHANNEL_STATUS_CMD
typedef struct Extern_Channel_Status
{
    unsigned char Cmd;
    unsigned char reserve[3];
    unsigned int Status;
}  Extern_Channel_Status;

typedef struct CommonCmdUC
{
    unsigned char Cmd;
    unsigned char reserve[2];
    unsigned char vid;
    unsigned char param[8];
} CommonCmdUC;

typedef struct CommonCmdUS
{
    unsigned char Cmd;
    unsigned char reserve[2];
    unsigned char vid;
    unsigned short param[4];
} CommonCmdUS;

typedef struct CommonCmdUL
{
    unsigned char Cmd;
    unsigned char reserve[2];
    unsigned char vid;
    unsigned long param[2];
} CommonCmdUL;

typedef struct CommonCmdULL
{
    unsigned char Cmd;
    unsigned char reserve[2];
    unsigned char vid;
    unsigned int rsrv2;
    unsigned long long param0;
} CommonCmdULL;

typedef struct NoACmd
{
    unsigned char Cmd;
    unsigned char reserve[2];
    unsigned char vid;
    unsigned int duration;
    unsigned int interval;
    unsigned int starttime;
    unsigned char count;
    unsigned char flag;
} NoACmd;

typedef struct KeyTableCmd
{
    unsigned char Cmd;
    unsigned char vid;
    unsigned char sta_id;
    unsigned char is_ukey;
} KeyTableCmd;

typedef struct Bcn_Frm_Addr
{
    unsigned int cmd;
    unsigned int address;
} Bcn_Frm_Addr;

typedef struct RegStaIdParam
{
    unsigned char Cmd;
    unsigned char vid;
    unsigned short StaId; //statin id
    unsigned char StaMac[6]; //station mac address
    unsigned char encrypt;
} RegStaIdParam;

typedef struct UnRegStaID
{
    unsigned char Cmd;
    unsigned char reserve[2];
    unsigned char vid; //statin id
    unsigned int StaId; //statin id
} UnRegStaID;

///UnRegAllStaID_CMD
typedef struct UnRegAllStaID
{
    unsigned char Cmd;
    unsigned char reserve[3];
    unsigned char vid; //statin id
} UnRegAllStaID;


//AC_BE_Param_Cmd
typedef struct SetEDCACmd
{
    unsigned char Cmd;
    unsigned char reserve[2];
    unsigned char vid;
    unsigned int data;
} SetEDCACmd;

///PHY_DEBUG
typedef struct phydebugcmd
{
    unsigned char Cmd;
    unsigned char len;
    unsigned short serve;
    unsigned int   offset;
    unsigned char buf[48];
} phydebugcmd;

#define WIFINET_ADDR_LEN    6
/*send (qos)null data command */
typedef struct NullDataCmd
{
    unsigned char Cmd;
    unsigned char vid;
    /*tid id useless when qos is 0 */
    unsigned char qos:1,
                /*1 is go to power save state, 0 is awake or keep alive*/
                tid:4,
                pwr_save:1,
                eosp:1,
                /* pwr save packet flag */
                pwr_flag:1;
    unsigned char dest_addr[WIFINET_ADDR_LEN];
    unsigned char bw;
    unsigned char rate;
    unsigned short sn;
    unsigned short staid;
    /*Indicates the length of the PSDU in octets*/
    unsigned short tv_l_length;
} NullDataCmd;

typedef struct KeepAliveCmd
{
    unsigned char Cmd;
    unsigned char enable;
    /* in second, need transform to ms in firmware */
    unsigned int period;
    struct NullDataCmd null_data_param;
} KeepAliveCmd;

#define IPV6_LEN 16
#define IPV4_LEN 4
#define IPV6_ADDR_BUF_LEN  (IPV6_LEN * 3 + 1)

#define WOW_PATTERN_SIZE 52
#define WOW_MAX_PATTERNS 4
#define WOW_PATTERN_MASK_SIZE 8
/* timeout for supend wifi, in jiffies */
#define WOW_TIMEOUT (200)

/* arp agent command */
typedef struct ArpAgentCmd
{
    unsigned char Cmd;
    unsigned char enable;
    unsigned char vid;
    unsigned char reserve;
    unsigned int ip_addr;
    unsigned char ip6_addr[IPV6_ADDR_BUF_LEN];
} ArpAgentCmd;

typedef struct PatchArpAgentCmd
{
    unsigned char Cmd;
    unsigned char enable;
    unsigned char vid;
    unsigned char reserve;
    unsigned int ip_addr;
    unsigned char ip6_addr[IPV6_ADDR_BUF_LEN];
    unsigned char dncp_server_mac[WIFINET_ADDR_LEN];
} PatchArpAgentCmd;


/* beacon miss monitor */
typedef struct BeaconMissCmd
{
    unsigned char Cmd;
    unsigned char vid;
    /*1 enable , 0 disable */
    unsigned char enable;
    unsigned char reserved;
    unsigned int period;
}BeaconMissCmd;

typedef struct vsdb_cmd
{
    unsigned char Cmd;
    unsigned char vid;
    /*1 enable , 0 disable */
    unsigned char enable;
    unsigned char reserved[5];
} vsdb_cmd;

/* suspend command */
typedef struct SuspendCmd
{
    unsigned char Cmd;
    unsigned char enable;
    unsigned char vid;
    unsigned char wow;
    unsigned char deepsleep;
    unsigned char reserved[3];
    unsigned int filters;
    unsigned char PN[16];
} SuspendCmd;

/* add patterns command for suspend*/
typedef struct AddPatternCmd
{
    unsigned char Cmd;
    unsigned char vid;
    /*offset from cfg layer, indicates where pattern
     *start to match with packets
     */
    unsigned char pattern_offset;
    unsigned char pattern_len;
    unsigned char pattern_id;
    unsigned char mask[WOW_PATTERN_MASK_SIZE];
    unsigned char pattern[WOW_PATTERN_SIZE];
} AddPatternCmd;

typedef struct NDPAnncmntCmd
{
    unsigned char Cmd;
    unsigned char vid;

    unsigned char rate;
    unsigned char bw;

    unsigned char dest_addr[WIFINET_ADDR_LEN];
    unsigned char reserved[2];

    unsigned short sn;
    unsigned short stainfo_num;

    unsigned short stainfo[WIFI_MAX_STA];
} NDPAnncmntCmd;

///TxBA_TYPE
#define  BA_DELAY  0
#define  BA_IMMIDIATE  1

///AuthRole
#define BA_INITIATOR  0
#define BA_RESPONSER  1

#define DPD_CHAN_SWITCH BIT(0)
#define DPD_FW_EVENT_START BIT(1)

typedef struct Add_BA_Struct
{
    unsigned char   Cmd;
    unsigned char   reserve[2];
    unsigned char   vid;
    unsigned char   TID;
    unsigned char   BA_TYPE;
    unsigned char   AuthRole;
    unsigned char   BA_Size;
    unsigned short   STA_ID;
    unsigned short   SeqNumStart;
} Add_BA_Struct;

typedef struct Delete_TxBA_Struct
{
    unsigned char   Cmd;
    unsigned char   reserve[2];
    unsigned char   vid;
    unsigned char   STA_ID;
    unsigned char   TID;
    unsigned char   AuthRole;
    unsigned char   reserve1;
} Delete_TxBA_Struct;

typedef struct MacRtcCmd
{
    unsigned char Cmd;
    unsigned char reserve[3];
    unsigned int enable;
} MacRtcCmd;

typedef struct Bmfm_Info_Cmd
{
    unsigned char Cmd;
    unsigned char vid;
    unsigned char reserved;

    unsigned char feedback_type;
    unsigned char group_id[8];
    unsigned char user_position[16];
}Bmfm_Info_Cmd;

typedef struct Coexist_Cmd
{
    unsigned char Cmd;
    unsigned char reserve[3];
    unsigned int coexist_id_bitmap;
    unsigned int req_timeout_value;
    unsigned int scan_priority_range;
    unsigned int be_bk_no_qos_priority_range;
    unsigned char coexist_enable;
    unsigned char max_miss_bcn_cnt;
    unsigned char not_grant_weight;
    unsigned char fw_retry_weight;
    unsigned char max_not_grant_cnt;
    unsigned char enable_infor_bt_wifi_work_freq;
    unsigned char reserve1[38];

} Coexist_Cmd;


struct New_Cmd
{
    unsigned char cmd_buffer[APP_CMD_PERFIFO_LEN];
};

/* channel switch flag */
enum {
    CHANNEL_RESTORE_FLAG = BIT(0),
    CHANNEL_CONNECT_FLAG = BIT(1),
    CHANNEL_NO_EVENT_FLAG = BIT(2),
    CHANNEL_RSSI_FLAG = BIT(3),
    CHANNEL_RSSI_PWR_FLAG = BIT(4),
    CHANNEL_HIGH_GAIN_FLAG = BIT(5),
};

typedef struct Channel_Switch
{
    unsigned char Cmd;
    unsigned char vid;
    unsigned char flag;
    unsigned char bw;
    unsigned char rssi;
    unsigned char duration;
    unsigned char res[2];
    unsigned int channel;
    /* center freq value */
    unsigned int pri_chan;
}Channel_Switch;
struct scanu_sched_plan
{
    //Interval between scheduled scan iterations. In seconds.
    unsigned int interval;
    //number of scan iterations in this scan plan. Zero means infinite loop.
    unsigned int iterations;
};


struct mac_addr
{
    /// Array of 16-bit words that make up the MAC address.
    unsigned short array[6/2];
};

/// Primary Channel definition
struct mac_chan_def
{
    /// Frequency of the channel (in MHz)
    unsigned short freq;
    /// RF band (@ref mac_chan_band)
    unsigned char band;
    /// Additional information (@ref mac_chan_flags)
    unsigned char flags;
    /// Max transmit power allowed on this channel (dBm)
    unsigned char tx_power;
};

/// SSID.
struct mac_ssid
{
    /// Actual length of the SSID.
    unsigned char length;
    /// Array containing the SSID name.
    unsigned char array[32];
};

#define SCAN_SSID_MAX   2
#define AML_2G_CHANNELS_NUM 14
#define AML_5G_CHANNELS_NUM 25

struct scanu_start_req
{
    struct mac_chan_def chan[AML_2G_CHANNELS_NUM + AML_5G_CHANNELS_NUM];
    struct mac_ssid ssid[SCAN_SSID_MAX];
    struct mac_addr bssid;
    uint32_t add_ies;
    unsigned short add_ie_len;
    unsigned char vif_idx;
    unsigned char chan_cnt;
    unsigned char ssid_cnt;
    unsigned int duration;
};

#define MAX_SCHED_SCAN_PLANS 2
#define MAX_MATCH_COUNT 2
struct scanu_macth_set
{
  struct mac_ssid ssId;
  unsigned char bssid[6];
  unsigned char    rssiThreshold;
  unsigned char per_band_rssi_thold[3];
};

typedef struct sched_scan_param
{
    struct scanu_start_req scanu_req;
    unsigned char match_count;
    struct scanu_macth_set match_sets[MAX_MATCH_COUNT];
    unsigned char min_rssi_thold;
    unsigned int delay;
    struct scanu_sched_plan scan_plans[MAX_SCHED_SCAN_PLANS];
    unsigned char n_scan_plans;
    unsigned char resv[3];
}sched_scan_ptr;


typedef struct sched_scan
{
    unsigned char Cmd;
    unsigned char enable;
    unsigned char vid;
    unsigned long long reqid;
    sched_scan_ptr * sched_param;
}sched_scan_cmd;

typedef struct DPD_Memory_Download
{
    unsigned char Cmd;
    unsigned char vid;
    unsigned char reserved[2];
} DPD_Memory_Download;

typedef struct Fwlog_Mode_Control
{
    unsigned char Cmd;
    unsigned char mode; //0x00:fw  0x01:host
    unsigned char fwlog_print;
    unsigned char reserved;
} Fwlog_Mode_Control;

// event
typedef struct channel_switch_event
{
    struct fw_event_basic_info basic_info;
    unsigned int channel;
    unsigned int bw;
    unsigned int done;
} channel_switch_event;

struct zgb_event_info
{
    unsigned char zgb_exist_flag;
    unsigned int  ampdu_num;
    unsigned int  del_ba_flag;
};

typedef struct zgb_exist_event
{
    struct fw_event_basic_info basic_info;
    struct zgb_event_info data_info;
} zgb_exist_event;

typedef struct tx_error_event
{
    struct fw_event_basic_info basic_info;
    unsigned int error_type;
    unsigned int frame_type;
} tx_error_event;

typedef struct dpd_calibration_event
{
    struct fw_event_basic_info basic_info;
    unsigned int dpd_state;
} dpd_calibration_event;

enum {
    dpd_calibration_start,
    dpd_calibration_end,
};


struct wow_wake_event
{
    struct fw_event_basic_info basic_info;
    unsigned int reason;
};

#define TSSI_5G_CAL_NUM 4

typedef struct txtssi_ratio_limit_param
{
    unsigned char enable;
    unsigned char wf2g_ratio_limit;
    unsigned char wf5g_ratio_limit;
    unsigned char reserved;
} txtssi_ratio_limit_param;

typedef struct digital_gain_limit_param
{
    unsigned char enable;
    unsigned char min_2g;
    unsigned char max_2g;
    unsigned char min_5g;
    unsigned char max_5g;
    unsigned char resv[3];
} digital_gain_limit_param;

typedef struct Cali_Param
{
    unsigned char Cmd;
    unsigned int version;
    unsigned short cali_config;
    unsigned char freq_offset;
    unsigned char htemp_freq_offset;
    unsigned char cca_ed_det;
    unsigned char tssi_2g_offset;
    unsigned char wf2g_spur_rmen;
    unsigned char rf_num;
    unsigned char tssi_5g_offset[6];
    unsigned char wftx_pwrtbl_en;
    unsigned char resv;
    txtssi_ratio_limit_param txtssi_ratio_limit;
    digital_gain_limit_param digital_gain_limit;
    unsigned short spur_freq;
    unsigned short platform_versionid;
} Cali_Param;

typedef struct WF2G_Txpwr_Param
{
    unsigned char Cmd;
    unsigned char reserved[3];
    unsigned char wf2g_pwr_tbl[2][16];
} WF2G_Txpwr_Param;

typedef struct WF5G_Txpwr_Param
{
    unsigned char Cmd;
    unsigned char reserved[3];
    unsigned char wf5g_pwr_tbl[3][16];
} WF5G_Txpwr_Param;

typedef struct Txpwr_Cffc_Cfg_Param
{
    unsigned char Cmd;
    unsigned char coefficient[57];
    unsigned char reserve[2];
    unsigned char band[4];
} Txpwr_Cffc_Cfg_Param;

typedef struct Efuse_Cfg_Param
{
    unsigned char Cmd;
    unsigned char flag;
    unsigned int efuse_9;
    unsigned int efuse_a;
    unsigned int efuse_b;
    unsigned int efuse_c;
    unsigned int efuse_d;
    unsigned int efuse_e;
} Efuse_Cfg_Param;

typedef struct Txt_Shift_Param
{
    unsigned char Cmd;
    unsigned char txt_shift_value[6];
} Txt_Shift_Param_T;

typedef struct Coex_Wf_Zgb_Mode_Param
{
    unsigned char Cmd;
    unsigned char  coex_work_mode;

}Coex_Wf_Zgb_Mode_Param;

typedef struct Tx_Power_Percentage_Param
{
    unsigned char Cmd;
    unsigned char  tx_power_percentage;

}Tx_Power_Percentage_Param;


typedef struct Phy_U_Interface_Param
{
    unsigned char Cmd;
    unsigned char vid;
    unsigned char interface_enable;
    unsigned char reserve[5];
} Phy_U_Interface_Param;

typedef struct Phy_Interface_Param
{
    unsigned char Cmd;
    unsigned char vid;
    unsigned char interface_enable;
    unsigned char reserve;
} Phy_Interface_Param;

typedef struct fwlog_print_event
{
    struct fw_event_basic_info basic_info;
    unsigned int fwlog_print;
} fwlog_print_event;

#pragma pack()

typedef union FI_CMDFIFO_PARAM
{
    struct RegStaIdParam StaIdParam;
    struct MultiCastKeyCmd MultiKeyCmd ;
    struct UniCastKeyCmd UniKeyCmd;
    struct RekeyDataCmd RekeyCmd;
    struct Extern_Channel_Status exchanstatus;
    struct UnRegStaID UnRegStaID;
    struct SetEDCACmd SetEDCACmd;
    struct PowerSaveCmd PowerSaveCmd;
    struct CAMModeCmd CAMModeCmd;
#ifdef DHCP_OFFLOAD
    struct DHCPStartCmd DHCPStartCmd;
#endif
#ifdef VSDB
    struct vsdb_cmd vsdb_enable_cmd;
#endif
    struct BCNIntervalCmd BCNIntervalCmd;
    struct BcnFrmAddrCmd BcnFrmAddrCmd;
    struct BeaconEnableCmd BeaconEnableCmd;
    struct UnRegAllStaID UnRegAllStaID;
    struct phydebugcmd Phydbgcmd;
    struct CommonCmdUC CommonCmdUC;
    struct CommonCmdUS CommonCmdUS;
    struct CommonCmdUL CommonCmdUL;
    struct CommonCmdULL CommonCmdULL;
    struct NoACmd NoACmd;
    struct MacRtcCmd  MacRtcCmd;
    struct NullDataCmd null_data_cmd;
    struct NDPAnncmntCmd ndp_anncmnt_data;
    struct KeepAliveCmd keep_alive_cmd;
    struct BeaconMissCmd beacon_miss_cmd;
    struct ArpAgentCmd arp_agent_cmd;
    struct PatchArpAgentCmd patch_arp_agent_cmd;
    struct SuspendCmd suspend_cmd;
    struct AddPatternCmd add_pattern_cmd;
    struct Bmfm_Info_Cmd bmfm_info_cmd;
    struct Coexist_Cmd coexist_cmd;
    struct Channel_Switch channel_switch_cmd;
    struct DPD_Memory_Download dpd_download_cmd;
    struct Fwlog_Mode_Control fwlog_mode_cmd;
    struct Cali_Param cali_param_cmd;
    struct Phy_Interface_Param phy_interface_cmd;
    struct WF2G_Txpwr_Param wf2g_txpwr_cmd;
    struct WF5G_Txpwr_Param wf5g_txpwr_cmd;
    struct PagelenCmd page_len_cmd;
    struct Txpwr_Cffc_Cfg_Param txpwr_cffc_cfg_cmd;
    struct New_Cmd new_cmd;
} FI_CMDFIFO_PARAM;

typedef struct Set_Cf_End
{
    unsigned char Cmd;
    unsigned char vid;
    unsigned char reserved;
    unsigned char enable;
} Set_Cf_End;

typedef struct Queue_Debug_Info
{
    unsigned int queue_idx:4;
    unsigned int state:4;
    unsigned int queue_cnt:8;
} Queue_Debug_Info;

typedef struct Common_Debug_Info
{
    unsigned int vid:2;
    unsigned int active_idx:4;
    unsigned int wifi_inactive_flag:1;
    unsigned int tx_error_flag:2;
    unsigned int flush_txframe_flag:2;
    unsigned int ps_state:2;
    unsigned int rsvd_0:3;
    unsigned int baqueue_cnt:8;
    unsigned int ba_cnt:8;
} Common_Debug_Info;

typedef struct Get_Queue_Debug_Info
{
    unsigned char Cmd;
    unsigned char reserve[2];
    unsigned char vid;
    Queue_Debug_Info param[QUEUE_AC_MAX];
    Common_Debug_Info common_debug_info;
    unsigned int queue_debug;
    unsigned int mac_irq_status;
} Get_Queue_Debug_Info;

typedef struct Get_Spec_Info
{
    unsigned char Cmd;
    unsigned char reserve[2];
    unsigned char vid;
    unsigned char param[60];
} Get_Spec_Info;


#endif// #ifdef CONFIG_SDIO_IF
#endif
