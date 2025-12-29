#ifndef __WIFI_IWPRIV_CMD_H
#define __WIFI_IWPRIV_CMD_H
#include <net/iw_handler.h>

extern struct iw_handler_def w1_iw_handle;

#define CALI_ADDR_NUM(x1,x2) (((x2) - (x1)) / 4 + 1)
#define RF_TOP_ADDR_START 0xff000000
#define RF_TOP_ADDR_END 0xff0000b4
#define RF_TOP_ADDR_NUM CALI_ADDR_NUM(RF_TOP_ADDR_START, RF_TOP_ADDR_END)
#define RF_SX_ADDR_START 0xff000400
#define RF_SX_ADDR_EDN 0xff0004c8
#define RF_SX_ADDR_NUM CALI_ADDR_NUM(RF_SX_ADDR_START, RF_SX_ADDR_EDN)
#define RF_TX_ADDR_START 0xff000800
#define RF_TX_ADDR_END 0xff000838
#define RF_TX_ADDR_NUM CALI_ADDR_NUM(RF_TX_ADDR_START, RF_TX_ADDR_END)
#define RF_RX_ADDR_START 0xff000c00
#define RF_RX_ADDR_END 0xff000d7c
#define RF_RX_ADDR_NUM CALI_ADDR_NUM(RF_RX_ADDR_START, RF_RX_ADDR_END)
#define ADDA_CORE_ADDR_START 0x00a0e000
#define ADDA_CORE_ADDR_END 0x00a0e01c
#define ADDA_CORE_ADDR_NUM CALI_ADDR_NUM(ADDA_CORE_ADDR_START, ADDA_CORE_ADDR_END)
#define ADDA_XMIT_ADDR_START 0x00a0e400
#define ADDA_XMIT_ADDR_END 0x00a0e630
#define ADDA_XMIT_ADDR_NUM CALI_ADDR_NUM(ADDA_XMIT_ADDR_START, ADDA_XMIT_ADDR_END)
#define ADDA_RECV_ADDR_START 0x00a0e800
#define ADDA_RECV_ADDR_END 0x00a0e868
#define ADDA_RECV_ADDR_NUM CALI_ADDR_NUM(ADDA_RECV_ADDR_START, ADDA_RECV_ADDR_END)
#define ADDA_ESTI_ADDR_START 0x00a0ec00
#define ADDA_ESTI_ADDR_END 0x00a0ee64
#define ADDA_ESTI_ADDR_NUM CALI_ADDR_NUM(ADDA_ESTI_ADDR_START, ADDA_ESTI_ADDR_END)
#define AGC_ADDR_START 0x00a08000
#define AGC_ADDR_END 0x00a08300
#define AGC_ADDR_NUM CALI_ADDR_NUM(AGC_ADDR_START, AGC_ADDR_END)
#define OFDM_ADDR_START 0x00a09000
#define OFDM_ADDR_END 0x00a09324
#define OFDM_ADDR_NUM CALI_ADDR_NUM(OFDM_ADDR_START, OFDM_ADDR_END)
#define PHY_ADDR_START 0x00a0b000
#define PHY_ADDR_END 0x00a0b248
#define PHY_ADDR_NUM CALI_ADDR_NUM(PHY_ADDR_START, PHY_ADDR_END)
#define AON_REG_ADDR_START 0x00f01000
#define AON_REG_ADDR_END 0x00f0107c
#define AON_REG_ADDR_NUM CALI_ADDR_NUM(AON_REG_ADDR_START, AON_REG_ADDR_END)
#define REG_DUMP_SIZE 2048
#define MEM_PKT_ADDR 0x00900000
#define MEM_PKT_lEN (112*1024*2)
#define REG_LEN 0xf000
#define MEM_ICCM_RAM_ADDR 0x00020000
#define EFUSE_ADDR_4 0x04
#define EFUSE_ADDR_5 0x05
#define EFUSE_ADDR_9 0x09
#define EFUSE_ADDR_A 0x0A
#define EFUSE_ADDR_B 0x0B
#define EFUSE_ADDR_C 0x0C
#define EFUSE_ADDR_D 0x0D
#define EFUSE_ADDR_E 0x0E
#define EFUSE_ADDR_F 0x0F
#define RF_VLD_ENABLE 0x01
#define EFUSE_ADDR_9_ILLEGAL_BIT 0xfff0ffff
#define EFUSE_ADDR_A_ILLEGAL_BIT 0x0000ff00
#define EFUSE_ADDR_A_LEGAL_BIT   0xffff00ff
#define BT_DIGITAL_GAIN_EFUSE_MASK 0xffff0000

enum
{
    RF_TOP_SEQ = 0,
    RF_SX_SEQ,
    RF_TX_SEQ,
    RF_RX_SEQ,
    ADDA_CORE_SEQ,
    ADDA_XMIT_SEQ,
    ADDA_RECV_SEQ,
    ADDA_ESTI_SEQ,
    AGC_SEQ,
    OFDM_SEQ,
    PHY_SEQ,
    AON_SEQ,
    DUMP_REG_SEQ_MAX,
};

enum DUMP_FLAG
{
    ICCROM_DUMP = BIT(0),
    ICCRAM_DUMP = BIT(1),
    DCCM_DUMP = BIT(2),
    SRAM_DUMP = BIT(3),
    PKT_DUMP = BIT(4),
    REG_DUMP = BIT(5),
    OTHERREG_DUMP = BIT(6),
};

typedef struct
{
    int addr_start;
    int addr_end;
    int num;
}reg_addr_attr_t;

struct pt_cali_bits {
    unsigned int channel:8,
                 bw:2,
                 rsvd1:6,
                 band:1,
                 rsvd2:14,
                 link_flag:1;
};

#define LOW_DAC_MCS_OFT (2)//mcs0-2 dac3, mcs3-9 dac5
#define LOW_MCS_GAIN_COMPENSATE (-2)//dBm

struct digital_gain_reg_bits {
    unsigned int rsvd1:16,
                 enable:1,
                 rsvd2:7,
                 gain:8;
};

struct key_gain_efuse_power_map {
    unsigned char param_name[30];
    unsigned char default_abs_power;
    unsigned char word_addr;
    unsigned char byte_oft;
};

enum efuse_domain {
    RF_VLD_XOSC_CTUNE  = 0x00,
    RF_VLD_PWR2G,
    RF_VLD_PWR5G,
    RF_VLD_PWR5G_BAND,
    RF_VLD_PWR5G_DELTA,
    RF_VLD_PWR5G_BAND_DELTA,
    RF_VLD_PWR2G_DELTA,

    RF_VLD,
    XOSC_CTUNE,
    PWR2G_20M,
    PWR2G_40M,
    PWR2G_11B,
    PWR5G_20M,
    PWR5G_40M,
    PWR5G_80M,
    PWR5G_BAND0,
    PWR5G_BAND1,
    PWR5G_BAND3,
    PWR5G_BAND4,

    PWR2G_20M_DELTA,
    PWR2G_40M_DALTA,
    PWR2G_11B_DELTA,
    PWR5G_20M_DELTA,
    PWR5G_40M_DELTA,
    PWR5G_80M_DELTA,
    PWR5G_BAND0_DELTA,
    PWR5G_BAND1_DELTA,
    PWR5G_BAND3_DELTA,
    PWR5G_BAND4_DELTA,
    XOSC_CTUNE_DELTA,

    EFUSE_MAX_DOMAIN
};

enum write_type {
    WRITE_FILE = 0,
    WRITE_FILE_NODE = 1,
};

enum aml_iwpriv_subcmd
{
    AML_IWP_ADDBA_REQ = 1,
    AML_IWP_AMSDU_STATE = 2,
    AML_IWP_AMPDU_STATE = 3,
    AML_IWP_UDP_INFO = 4,
    AML_IWP_COUNTRY = 5,
    AML_IWP_11H = 6,
    AML_IWP_ARP_RX = 7,
    AML_IWP_CHAN_LIST = 8,
    AML_IWP_ROAM_THRESH_2G = 9,
    AML_IWP_ROAM_THRESH_5G = 10,
    AML_IWP_ROAM_MODE = 11,
    AML_IWP_MARK_DFS_CHAN = 12,
    AML_IWP_UNMARK_DFS_CHAN = 13,
    AML_IWP_WEAK_THR_NARROW = 14,
    AML_IWP_WEAK_THR_WIDE = 15,
    AML_IWP_EAT_COUNT = 16,
    AML_IWP_AGGR_THRESH = 17,
    AML_IWP_HEART_INTERVAL = 18,
    AML_IWP_CHIP_ID = 19,
    AML_IWP_WIFI_MAC = 20,
    AML_IWP_AP_IP = 21,
    AML_IWP_ROAM_CHAN = 22,
    AML_IWP_BSS_COEX = 23,
    AML_IWP_TXAGGR_STA = 24,
    AML_IWP_SESSION = 25,
    AML_IWP_P2P_DEV_ID = 26,
    AML_IWP_WMM_AC_DELTS = 27,
    AML_IWP_SHORT_GI = 28,
    AML_IWP_DYNAMIC_BW = 29,
    AML_IWP_CCA_BUSY_CHECK = 30,
    AML_IWP_PHY_STATISTIC = 31,
    AML_IWP_PRINT_VERSION = 32,
    AML_IWP_SET_REG = 33,
    AML_IWP_GET_REG = 34,
    AML_IWP_GET_TX_STATUS = 35,
    AML_IWP_ENABLE_FW_LOG = 36,
    AML_IWP_LEGACY_SET_REG = 37,
    AML_IWP_LEGACY_GET_REG = 38,
    AML_IWP_SET_BCN_INTERVAL = 39,
    AML_IWP_SET_LDPC = 40,
    AML_IWP_SET_BEAMFORMING = 41,
    AML_IWP_SET_SCAN_TIME_IDLE = 42,
    AML_IWP_SET_SCAN_TIME_CONNECT = 43,
    AML_IWP_SET_SCAN_HANG = 44,
    AML_IWP_EN_BTWIFI_COEX = 45,
    AML_IWP_SET_COEXIST_MAX_MISS_BCN_CNT = 46,
    AML_IWP_SET_COEXIST_REQ_TIMEOUT = 47,
    AML_IWP_SET_COEXIST_NOT_GRANT_WEIGHT = 48,
    AML_IWP_SET_RATE_LEGACY = 49,
    AML_IWP_SET_RATE_HT = 50,
    AML_IWP_SET_RATE_VHT = 51,
    AML_IWP_SET_RATE_AUTO = 52,
    AML_IWP_SET_POWER = 53,
    AML_IWP_SET_CHL_RSSI = 54,
    AML_IWP_SET_BURST = 55,
    AML_IWP_SET_UAPSD = 56,
    AML_IWP_SET_PT_RX_START = 57,
    AML_IWP_SET_PT_RX_STOP = 58,
    AML_IWP_SET_SCAN_PRI = 59,
    AML_IWP_SET_BE_BK_NOQOS_PRI = 60,
    AML_IWP_SET_FETCH_PKT_METHOD = 61,
    AML_IWP_SET_FRAG_THRESHOLD = 62,
    AML_IWP_SET_PREAMBLE_TYPE = 63,
    AML_IWP_SET_FIX_BAND = 64,
    AML_IWP_SET_GAIN = 65,
    AML_IWP_SET_TPC = 66,
    AML_IWP_SET_DEV_SN = 67,
    AML_IWP_GET_DEV_SN = 68,
    AML_IWP_SET_WIFI_MAC = 69,
    AML_IWP_GET_WIFI_MAC = 70,
    AML_IWP_SET_BT_MAC = 71,
    AML_IWP_GET_BT_MAC = 72,
    AML_IWP_SET_TXPW_PLAN = 73,
    AML_IWP_SET_MAC_MODE = 74,
    AML_IWP_GET_HOST_LOG = 75,
    AML_IWP_GET_FW_LOG = 76,
    AML_IWP_SET_RECOVERY = 77,
    AML_IWP_GET_EFUSE = 78,
    AML_IWP_SET_EFUSE = 79,
    AML_IWP_GET_SPEC_REGS = 80,
    AML_IWP_GET_EN_RF_TEST = 81,
    AML_IWP_SET_RX_SIZE = 82,
    AML_IWP_MEM_DUMP = 83,
    AML_IWP_SET_PT_CALIBRATION = 84,
    AML_IWP_SET_TX_PATH = 85,
    AML_IWP_SET_TX_BW = 86,
    AML_IWP_SET_TX_MODE = 87,
    AML_IWP_SET_TX_RATE = 88,
    AML_IWP_SET_TX_TYPE = 89,
    AML_IWP_SET_TX_LEN = 90,
    AML_IWP_SET_TX_NUM = 91,
    AML_IWP_PT_TX_START = 92,
    AML_IWP_PT_TX_END = 93,
    AML_IWP_SET_RX_PATH = 94,
    AML_IWP_PT_RX_START = 95,
    AML_IWP_PT_RX_END = 96,
    AML_IWP_SET_STBC = 97,
    AML_IWP_SET_TX_PWR = 98,
    AML_IWP_SET_CF_END = 99,
    AML_IWP_SET_FLOW_CTRL = 100,
    AML_IWP_GET_KEY_ENTRY = 101,
    AML_IWP_SET_DELTA_TX_PWR = 102,
    AML_IWP_GET_CAPT_STATICS = 103,
    AML_IWP_GET_EFUSE_TIMES = 104,
    AML_IWP_GET_EFUSE_DOMAIN = 105,
    AML_IWP_SET_EFUSE_DOMAIN = 106,
    AML_IWP_SET_EFUSE_TMP = 107,
    AML_IWP_GET_EFUSE_TMP = 108,
    AML_IWP_SET_ANT_SWITCH = 109,
    AML_IWP_SET_WFA_MODE = 110,
    AML_IWP_SET_PT_EFUSE = 111,
    AML_IWP_GET_BT_DIGITAL_GAIN_EFUSE_TIMES = 112,
    AML_IWP_SET_BT_DIGITAL_GAIN_EFUSE = 113,
    AML_IWP_GET_BT_DIGITAL_GAIN_EFUSE = 114,
    AML_IWP_GET_WIFI_PERFORMANCE_INFO = 115,
};

extern void dump_spec_regs_val(struct wlan_net_vif *wnet_vif, int reg_domain);
extern unsigned char aml_iwpriv_get_band(void);
int aml_set_debug_modules(char *debug_str);
void aml_iwpriv_set_recovery(unsigned int set);

extern unsigned char g_iwpriv_get_spec_regs_flag;
void aml_get_rate_bw(unsigned char bw, unsigned char *name);
void aml_get_rate_idx(unsigned char rate, unsigned char *name);
void aml_get_rate_gi(unsigned char gi, unsigned char *name);
int aml_get_rvr_info(struct wlan_net_vif *wnet_vif, unsigned char *buf, unsigned char write_type);
#endif
