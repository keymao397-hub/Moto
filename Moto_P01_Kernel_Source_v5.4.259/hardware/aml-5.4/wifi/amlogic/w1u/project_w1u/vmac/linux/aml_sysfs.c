#include <linux/netdevice.h>
#include "wifi_mac_if.h"
#include "wifi_debug.h"
#include "wifi_mac_p2p.h"
#include "wifi_mac_chan.h"
#include "wifi_mac_concurrent.h"
#include "wifi_iwpriv_cmd.h"

struct wlan_net_vif *g_wnet_vif0;
struct wlan_net_vif *g_wnet_vif1;

extern void get_phy_stc_info(unsigned int *arr);
#define PROTO_802_11N  "n"
#define PROTO_802_11AC "ac"
#define BW_20M_STR "20M"
#define BW_40M_STR "40M"
#define BW_80M_STR "80M"
#define CCA_THRD_DEFAULT 0x002d8328
#define CCA_THRD_CE_FCC 0x002a42a4


static ssize_t show_hang_info(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    return 0;
}

static DEVICE_ATTR(hang_info, S_IRUGO, show_hang_info, NULL);

static ssize_t show_driver(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    return 0;
}

static DEVICE_ATTR(driver, S_IRUGO, show_driver, NULL);


static ssize_t show_band_info(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    return 0;
}

static DEVICE_ATTR(band_info, S_IRUGO, show_band_info, NULL);


static ssize_t show_band_query_flag(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    return 0;
}

static DEVICE_ATTR(band_query_flag, S_IRUGO, show_band_query_flag, NULL);


static ssize_t show_cur_channel(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned short cur_channel = 0;

    wnet_vif = g_wnet_vif0;
    if (wnet_vif->vm_curchan != WIFINET_CHAN_ERR) {
        cur_channel = g_wnet_vif0->vm_curchan->chan_pri_num;
    }
    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "cur_channel :%d\n", cur_channel);
}

static DEVICE_ATTR(cur_channel, S_IRUGO, show_cur_channel, NULL);


static ssize_t show_bandwidth(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    unsigned char bandwidth[32];
    struct wlan_net_vif *wnet_vif = g_wnet_vif0;

    if (!wnet_vif || !wnet_vif->vm_curchan) {
        AML_PRINT_LOG_ERR("wnet_vif/vm_curchan is null !!! \n");
        return -EFAULT;
    }

    switch (g_wnet_vif0->vm_curchan->chan_bw)
    {
        case WIFINET_BWC_WIDTH20:
            sprintf(bandwidth, "bandwidth :20\n");
            break;
        case WIFINET_BWC_WIDTH40:
            sprintf(bandwidth, "bandwidth :40\n");
            break;
        case WIFINET_BWC_WIDTH80:
            sprintf(bandwidth, "bandwidth :80\n");
            break;
        case WIFINET_BWC_WIDTH160:
            sprintf(bandwidth, "bandwidth :160\n");
            break;
        case WIFINET_BWC_WIDTH80P80:
            sprintf(bandwidth, "bandwidth :80+80\n");
            break;
        default:
            AML_PRINT_LOG_ERR("unsupported  bandwidth \n");
            break;
    }

    return sprintf(buf, "%s", bandwidth);
}

static DEVICE_ATTR(bandwidth, S_IRUGO, show_bandwidth, NULL);


static ssize_t show_rRssi(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned int Rssi;

    wnet_vif = g_wnet_vif0;

    if (!wnet_vif || !wnet_vif->vm_mainsta) {
        return sprintf(buf, "[WARN] vif or mainsta is null\n");
    }

    if ( wnet_vif->vm_opmode != WIFINET_M_STA) {
        return sprintf(buf, "[WARN] mode:%d\n",wnet_vif->vm_opmode);
    }

    if (wnet_vif->vm_state != WIFINET_S_CONNECTED) {
        return sprintf(buf, "[WARN] state:%d\n",wnet_vif->vm_state);
    }

    Rssi = wnet_vif->vm_mainsta->sta_avg_rssi;

    return sprintf(buf, "Rssi:%d\n", Rssi - 256);
}

static DEVICE_ATTR(rRssi, S_IRUGO, show_rRssi, NULL);


static ssize_t show_iSnr(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    int32_t Snr = 0;
    unsigned int arr[8] = {0};

    wnet_vif = g_wnet_vif0;

    get_phy_stc_info(arr);
    Snr = arr[1];

    if (!wnet_vif || !wnet_vif->vm_mainsta) {
        return sprintf(buf, "[WARN] vif or mainsta is null\n");
    }

    if ( wnet_vif->vm_opmode != WIFINET_M_STA) {
        return sprintf(buf, "[WARN] mode:%d\n",wnet_vif->vm_opmode);
    }

    if (wnet_vif->vm_state != WIFINET_S_CONNECTED) {
        return sprintf(buf, "[WARN] state:%d\n",wnet_vif->vm_state);
    }

    return sprintf(buf, "Snr:%d\n", wnet_vif->vm_mainsta->sta_avg_snr);
}

static DEVICE_ATTR(iSnr, S_IRUGO, show_iSnr, NULL);


static ssize_t show_i4RSSI0(struct device *d, struct device_attribute *attr,
                 char *buf)
{

    return 0;
}

static DEVICE_ATTR(i4RSSI0, S_IRUGO, show_i4RSSI0, NULL);


static ssize_t show_i4RSSI1(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    return 0;
}

static DEVICE_ATTR(i4RSSI1, S_IRUGO, show_i4RSSI1, NULL);


static ssize_t show_iSnrR0Phase2(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    return 0;
}

static DEVICE_ATTR(iSnrR0Phase2, S_IRUGO, show_iSnrR0Phase2, NULL);

static ssize_t show_iSnrR1Phase2(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    return 0;
}

static DEVICE_ATTR(iSnrR1Phase2, S_IRUGO, show_iSnrR1Phase2, NULL);


static ssize_t show_Glitch_Diff(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    static unsigned int last_glitch = 0;
    struct hw_interface* hif = hif_get_hw_interface();

    sprintf(buf, "%d \n", hif->HiStatus.tx_fail_num - last_glitch);

    last_glitch = hif->HiStatus.tx_fail_num;

    return strlen(buf);
}

static DEVICE_ATTR(Glitch_Diff, S_IRUGO, show_Glitch_Diff, NULL);


static ssize_t show_Glitch_Total(struct device *d, struct device_attribute *attr, char *buf)
{
    struct hw_interface* hif = hif_get_hw_interface();

    sprintf(buf, "%d \n", hif->HiStatus.tx_fail_num);

    return strlen(buf);
}

static DEVICE_ATTR(Glitch_Total, S_IRUGO, show_Glitch_Total, NULL);


static ssize_t show_TxRate(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned int Tx_rate;
    wnet_vif = g_wnet_vif0;

    Tx_rate = wnet_vif->vm_mainsta->sta_last_txrate >> 10;
    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "Tx_rate :%d\n", Tx_rate);
}

static DEVICE_ATTR(TxRate, S_IRUGO, show_TxRate, NULL);


static ssize_t show_TxPktNum(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned long TxPktNum;
    wnet_vif = g_wnet_vif0;

    TxPktNum = wnet_vif->vm_devstats.tx_packets;
    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "TxPktNum :%d\n", TxPktNum);
}

static DEVICE_ATTR(TxPktNum, S_IRUGO, show_TxPktNum, NULL);


static ssize_t show_TxFailNum(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned long TxFailNum;
    wnet_vif = g_wnet_vif0;

    TxFailNum = wnet_vif->vm_devstats.tx_errors;
    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "TxFailNum :%d\n", TxFailNum);

}


static DEVICE_ATTR(TxFailNum, S_IRUGO, show_TxFailNum, NULL);

static ssize_t show_Data_RxRate(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned int Rx_rate;
    wnet_vif = g_wnet_vif0;

    Rx_rate = wnet_vif->vm_mainsta->sta_last_rxrate >> 10;
    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "Rx_rate :%d\n", Rx_rate);
}

static DEVICE_ATTR(Data_RxRate, S_IRUGO, show_Data_RxRate, NULL);


static ssize_t show_Average_RxRate(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned int Rx_avg_rate;
    wnet_vif = g_wnet_vif0;

    Rx_avg_rate = wnet_vif->vm_rx_speed ;
    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "Rx_avg_rate :%d\n", Rx_avg_rate);

}

static DEVICE_ATTR(Average_RxRate, S_IRUGO, show_Average_RxRate, NULL);


static ssize_t show_RxPktNum(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned long RxPktNum;
    wnet_vif = g_wnet_vif0;

    RxPktNum = wnet_vif->vm_devstats.rx_packets;
    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "RxPktNum :%d\n", RxPktNum);
}

static DEVICE_ATTR(RxPktNum, S_IRUGO, show_RxPktNum, NULL);


static ssize_t show_RxFailNum(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned long RxFailNum;
    wnet_vif = g_wnet_vif0;

    RxFailNum = wnet_vif->vm_devstats.rx_errors;
    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "RxFailNum :%d\n", RxFailNum);

}

static DEVICE_ATTR(RxFailNum, S_IRUGO, show_RxFailNum, NULL);


static ssize_t show_pno_sec_mode(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    return 0;
}


static DEVICE_ATTR(pno_sec_mode, S_IRUGO, show_pno_sec_mode, NULL);


static ssize_t show_hal_spec(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;

    char *out = buf;

    wnet_vif = g_wnet_vif0;
    out += sprintf(out, "Tx_Nss:%d\n", 1);
    out += sprintf(out, "Rx_Nss:%d\n", 1);
    out += sprintf(out, "2gBW:%s\n", BW_40M_STR);
    out += sprintf(out, "5gBW:%s\n", BW_80M_STR);

    if (wnet_vif->vm_mac_mode == WIFINET_MODE_11BGN)
        out += sprintf(out, "max_proto:%s\n", PROTO_802_11N);
    else if (wnet_vif->vm_mac_mode == WIFINET_MODE_11GNAC)
        out += sprintf(out, "max_proto:%s\n", PROTO_802_11AC);
    return (out - buf);
}

static DEVICE_ATTR(hal_spec, S_IRUGO, show_hal_spec, NULL);


static ssize_t show_iNoise(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    int32_t Noise = 0;
    unsigned int arr[8] = {0};

    wnet_vif = g_wnet_vif0;

    get_phy_stc_info(arr);
    Noise = arr[4];

    if (!wnet_vif || !wnet_vif->vm_mainsta) {
        return sprintf(buf, "[WARN] vif or mainsta is null\n");
    }

    if ( wnet_vif->vm_opmode != WIFINET_M_STA) {
        return sprintf(buf, "[WARN] mode:%d\n",wnet_vif->vm_opmode);
    }

    if (wnet_vif->vm_state != WIFINET_S_CONNECTED) {
        return sprintf(buf, "[WARN] state:%d\n",wnet_vif->vm_state);
    }

    return sprintf(buf, "Noise:%d\n", (wnet_vif->vm_mainsta->sta_avg_rssi - 256 - wnet_vif->vm_mainsta->sta_avg_snr));
}


static DEVICE_ATTR(iNoise, S_IRUGO, show_iNoise, NULL);


static ssize_t show_bssid(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned char vm_bssid[WIFINET_ADDR_LEN];

    wnet_vif = g_wnet_vif0;
    memset(vm_bssid, 0, WIFINET_ADDR_LEN);
    memcpy(vm_bssid, wnet_vif->vm_des_bssid, WIFINET_ADDR_LEN);

    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "BSSID:%02x:%02x:%02x:%02x:%02x:%02x\n", vm_bssid[0],
    vm_bssid[1], vm_bssid[2], vm_bssid[3], vm_bssid[4], vm_bssid[5], vm_bssid);

}

static DEVICE_ATTR(bssid, S_IRUGO, show_bssid, NULL);


static ssize_t show_ssid(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    struct wifi_mac_ScanSSID wm_ssid;

    wnet_vif = g_wnet_vif0;
    memset(&wm_ssid, 0, sizeof(struct wifi_mac_ScanSSID));
    memcpy(wm_ssid.ssid, wnet_vif->vm_des_ssid[0].ssid, wnet_vif->vm_des_ssid[0].len);

    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "SSID:%s\n", wm_ssid.ssid);
}

static DEVICE_ATTR(ssid, S_IRUGO, show_ssid, NULL);


static ssize_t show_wakeup_reason(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    return 0;
}

static DEVICE_ATTR(wakeup_reason, S_IRUGO, show_wakeup_reason, NULL);


static ssize_t show_ap_info(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    struct wifi_mac_ScanSSID wm_ssid;

    wnet_vif = g_wnet_vif0;
    memset(&wm_ssid, 0, sizeof(struct wifi_mac_ScanSSID));
    memcpy(wm_ssid.ssid, wnet_vif->vm_des_ssid[0].ssid, wnet_vif->vm_des_ssid[0].len);

    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "AP_SSID:%s\n", wm_ssid.ssid);
}

static DEVICE_ATTR(ap_info, S_IRUGO, show_ap_info, NULL);


static ssize_t show_p2p_channel(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned short cur_channel = 0;

    wnet_vif = g_wnet_vif1;
    if (wnet_vif->vm_curchan != WIFINET_CHAN_ERR) {
        cur_channel = g_wnet_vif1->vm_curchan->chan_pri_num;
    }
    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "p2p_channel :%d\n", cur_channel);

}

static DEVICE_ATTR(p2p_channel, S_IRUGO, show_p2p_channel, NULL);

static ssize_t show_p2p_home_channel(struct device *d, struct device_attribute *attr,
                char *buf)
{
    struct wifi_mac *wifimac = NULL;

    wifimac = wifi_mac_get_mac_handle();

    return sprintf(buf, "p2p_home_ch: %d\n", wifimac->wm_p2p_home_channel);
}

extern unsigned char paramParseU32(char * * buf, unsigned int * pvalue);
unsigned char aml_parse_p2p_home_channel(const char *buf, unsigned int * channel)
{
    char ** pbuf = &buf;

    if (paramParseU32(pbuf, channel) != 0) {
        channel = 0;
        return 1;
    }
    return 0;
}

static ssize_t store_p2p_home_channel(struct device *d, struct device_attribute *attr,
                const char *buf, size_t len)
{
    struct wlan_net_vif *p2p_wnet_vif = NULL;
    struct wifi_mac *wifimac = NULL;
    unsigned int channel = 0;
    unsigned char error = 1;

    AML_PRINT_LOG_INFO("\n");

    wifimac = wifi_mac_get_mac_handle();
    p2p_wnet_vif = g_wnet_vif1;

    if (aml_parse_p2p_home_channel(buf, &channel) == 0) {
        if (wifi_mac_set_p2p_home_chan(wifimac, channel) == 0) {
            if (!wifi_mac_is_others_wnet_vif_running(p2p_wnet_vif)
                    && wifi_mac_p2p_home_channel_enabled(p2p_wnet_vif)
                    && p2p_wnet_vif->vm_curchan->chan_pri_num != channel) {
                channel_switch_announce_trigger(wifimac, channel, WIFINET_BWC_WIDTH20, channel);
            }
            error = 0;
        } else {
            AML_PRINT_LOG_INFO("channel %d isn't a legal channel\n", channel);
        }
    }

    if (error != 0) {
        AML_PRINT_LOG_INFO("p2p_home_channel config error\n");
    }
    return len;
}

DEVICE_ATTR(p2p_home_channel, S_IWUSR|S_IWGRP|S_IRUGO, show_p2p_home_channel, store_p2p_home_channel);

static ssize_t show_p2p_DevNum(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    struct wifi_station_tbl *nt;
    unsigned char DevNum = 0;
    struct wifi_station *sta = NULL;
    struct wifi_station *sta_next = NULL;

    wnet_vif = g_wnet_vif1;

    nt =  &wnet_vif->vm_sta_tbl;
    WIFINET_NODE_LOCK(nt);
    list_for_each_entry_safe(sta, sta_next, &nt->nt_nsta, sta_list) {

        if (sta != wnet_vif->vm_mainsta) {
            DevNum++;;
        }
    }

    WIFINET_NODE_UNLOCK(nt);
    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "DevNum :%d\n", DevNum);;
}

static DEVICE_ATTR(p2p_DevNum, S_IRUGO, show_p2p_DevNum, NULL);


static ssize_t show_p2p_bandwidth(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned short bandwidth;

    wnet_vif = g_wnet_vif1;
    bandwidth = g_wnet_vif1->vm_bandwidth;
    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "p2p_bandwidth :%d\n", bandwidth);
}

static DEVICE_ATTR(p2p_bandwidth, S_IRUGO, show_p2p_bandwidth, NULL);


static ssize_t show_p2p_rRssi(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned int Rssi;

    wnet_vif = g_wnet_vif1;

    if (!wnet_vif || !wnet_vif->vm_mainsta) {
        return sprintf(buf, "[WARN] vif or mainsta is null\n");
    }

    if ( wnet_vif->vm_opmode != WIFINET_M_STA) {
        return sprintf(buf, "[WARN] mode:%d\n",wnet_vif->vm_opmode);
    }

    if (wnet_vif->vm_state != WIFINET_S_CONNECTED) {
        return sprintf(buf, "[WARN] state:%d\n",wnet_vif->vm_state);
    }

    Rssi = wnet_vif->vm_mainsta->sta_avg_rssi;

    return sprintf(buf, "p2p_Rssi:%d\n", Rssi - 256);
}


static DEVICE_ATTR(p2p_rRssi, S_IRUGO, show_p2p_rRssi, NULL);


static ssize_t show_p2p_iNoise(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;

    wnet_vif = g_wnet_vif1;

    if (!wnet_vif || !wnet_vif->vm_mainsta) {
        return sprintf(buf, "[WARN] vif or mainsta is null\n");
    }

    if ( wnet_vif->vm_opmode != WIFINET_M_STA) {
        return sprintf(buf, "[WARN] mode:%d\n",wnet_vif->vm_opmode);
    }

    if (wnet_vif->vm_state != WIFINET_S_CONNECTED) {
        return sprintf(buf, "[WARN] state:%d\n",wnet_vif->vm_state);
    }

    return sprintf(buf, "p2p_Noise:%d\n", (wnet_vif->vm_mainsta->sta_avg_rssi - 256 - wnet_vif->vm_mainsta->sta_avg_snr));
}

static DEVICE_ATTR(p2p_iNoise , S_IRUGO, show_p2p_iNoise , NULL);


static ssize_t show_p2p_iSnr(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    int32_t Snr = 0;
    unsigned int arr[8] = {0};

    wnet_vif = g_wnet_vif1;

    get_phy_stc_info(arr);
    Snr = arr[1];

    if (!wnet_vif || !wnet_vif->vm_mainsta) {
        return sprintf(buf, "[WARN] vif or mainsta is null\n");
    }

    if ( wnet_vif->vm_opmode != WIFINET_M_STA) {
        return sprintf(buf, "[WARN] mode:%d\n",wnet_vif->vm_opmode);
    }

    if (wnet_vif->vm_state != WIFINET_S_CONNECTED) {
        return sprintf(buf, "[WARN] state:%d\n",wnet_vif->vm_state);
    }

    return sprintf(buf, "p2p_Snr:%d\n", wnet_vif->vm_mainsta->sta_avg_snr);
}

static DEVICE_ATTR(p2p_iSnr, S_IRUGO, show_p2p_iSnr, NULL);


static ssize_t show_p2p_i4RSSI0(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    return 0;
}

static DEVICE_ATTR(p2p_i4RSSI0, S_IRUGO, show_p2p_i4RSSI0, NULL);


static ssize_t show_p2p_i4RSSI1(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    return 0;
}

static DEVICE_ATTR(p2p_i4RSSI1, S_IRUGO, show_p2p_i4RSSI1, NULL);


static ssize_t show_p2p_iSnrR0Phase2(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    return 0;
}

static DEVICE_ATTR(p2p_iSnrR0Phase2, S_IRUGO, show_p2p_iSnrR0Phase2, NULL);


static ssize_t show_p2p_iSnrR1Phase2(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    return 0;
}

static DEVICE_ATTR(p2p_iSnrR1Phase2, S_IRUGO, show_p2p_iSnrR1Phase2, NULL);


static ssize_t show_p2p_TxPkt(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned long TxPktNum;
    wnet_vif = g_wnet_vif1;

    TxPktNum = wnet_vif->vm_devstats.tx_packets;
    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "p2p_TxPkt :%d\n", TxPktNum);
}

static DEVICE_ATTR(p2p_TxPkt, S_IRUGO, show_p2p_TxPkt, NULL);


static ssize_t show_p2p_TxFailPkt(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned long TxFailNum;
    wnet_vif = g_wnet_vif1;

    TxFailNum = wnet_vif->vm_devstats.tx_errors;
    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "p2p_TxFail :%d\n", TxFailNum);
}

static DEVICE_ATTR(p2p_TxFailPkt, S_IRUGO, show_p2p_TxFailPkt, NULL);


static ssize_t show_p2p_RxPkt(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned long RxPktNum;
    wnet_vif = g_wnet_vif1;

    RxPktNum = wnet_vif->vm_devstats.rx_packets;
    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "p2p_RxPkt :%d\n", RxPktNum);
}

static DEVICE_ATTR(p2p_RxPkt, S_IRUGO, show_p2p_RxPkt, NULL);


static ssize_t show_p2p_RxFailPkt(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned long RxFailNum;
    wnet_vif = g_wnet_vif1;

    RxFailNum = wnet_vif->vm_devstats.rx_errors;
    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "p2p_RxFail :%d\n", RxFailNum);
}

static DEVICE_ATTR(p2p_RxFailPkt, S_IRUGO, show_p2p_RxFailPkt, NULL);


static ssize_t show_p2p_Glitch(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    return 0;
}

static DEVICE_ATTR(p2p_Glitch, S_IRUGO, show_p2p_Glitch, NULL);


static ssize_t show_p2p_RxRate(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned int Rx_rate;
    wnet_vif = g_wnet_vif1;

    Rx_rate = wnet_vif->vm_mainsta->sta_last_rxrate >> 10;
    if (!wnet_vif)
        return -EFAULT;
    return sprintf(buf, "p2p_Rx_rate :%d\n", Rx_rate);
}

static DEVICE_ATTR(p2p_RxRate, S_IRUGO, show_p2p_RxRate, NULL);


static ssize_t show_p2p_Mac(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wifi_station *sta = NULL;
    struct wifi_station *sta_next = NULL;
    struct wlan_net_vif *wnet_vif = g_wnet_vif1;
    struct wifi_station_tbl *nt = &wnet_vif->vm_sta_tbl;
    unsigned char p2p0_vif_id = 1;
    unsigned int write_len = 0;
    unsigned char count = 0;

    if (!wnet_vif)
        return -EFAULT;

    if (vm_p2p_enabled(wnet_vif->vm_p2p)) {
        WIFINET_NODE_LOCK(nt);
        list_for_each_entry_safe(sta, sta_next, &nt->nt_nsta, sta_list) {
            if (sta->wnet_vif_id == p2p0_vif_id) {
                if (memcmp(sta->sta_macaddr, wnet_vif->vm_myaddr, WIFINET_ADDR_LEN)) {
                    write_len += sprintf(buf + write_len, "p2p[%d]: %02x:%02x:%02x:%02x:%02x:%02x\n",count++,
                            sta->sta_macaddr[0],sta->sta_macaddr[1],sta->sta_macaddr[2],
                            sta->sta_macaddr[3],sta->sta_macaddr[4],sta->sta_macaddr[5]);
                }
            }
        }
        WIFINET_NODE_UNLOCK(nt);
    }

    return write_len;
}

static DEVICE_ATTR(p2p_Mac, S_IRUGO, show_p2p_Mac, NULL);


static ssize_t show_bypass_dfs(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    return 0;
}

static DEVICE_ATTR(bypass_dfs, S_IRUGO, show_bypass_dfs, NULL);


static ssize_t show_go_hidden_mode(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = NULL;
    wnet_vif = g_wnet_vif1;

    if (!wnet_vif)
        return -EFAULT;

    return sprintf(buf, "go_hidden_mode: %d\n", wnet_vif->vm_p2p->go_hidden_mode);
}

static ssize_t store_go_hidden_mode(struct device *d, struct device_attribute *attr,
                char *buf, ssize_t len)
{
    unsigned char mode = *buf - 48;
    struct wlan_net_vif *wnet_vif = g_wnet_vif1;
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;

    if (!wnet_vif || (mode < 0) || (mode > 1)) {
        AML_PRINT_LOG_ERR("The input mode is error");
        return -EFAULT;
    }

    if (wnet_vif->vm_p2p->p2p_enable && (wnet_vif->vm_p2p->p2p_role == NET80211_P2P_ROLE_GO)) {
        if ((wifimac->wm_flags & WIFINET_F_DOTH) && (wifimac->wm_flags & WIFINET_F_CHANSWITCH)) {
            if (mode) {
                wnet_vif->vm_flags |= WIFINET_F_HIDESSID;
            } else {
                wnet_vif->vm_flags &= ~WIFINET_F_HIDESSID;
            }

        }
        AML_PRINT_LOG_INFO(" go_hidden_mode:%d \n",mode);
        wnet_vif->vm_p2p->go_hidden_mode = mode;
    }

    return len;
}

static DEVICE_ATTR(go_hidden_mode, S_IWUSR|S_IWGRP|S_IRUGO, show_go_hidden_mode, store_go_hidden_mode);

void cca_thrd_cfg_change_handle(struct wlan_net_vif *wnet_vif, unsigned char cfg)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;

    /*default cfg is disable*/
    AML_PRINT_LOG_INFO("%s\n", cfg ? "ENABLE" : "DISABLE");

    if (cfg) {
        aml_iwpriv_set_recovery(0);
        wifimac->drv_priv->drv_config.cfg_burst_ack = 0;
        wnet_vif->vif_ops.write_word(DF_AGC_REG_A29, CCA_THRD_CE_FCC);
    } else {
        aml_iwpriv_set_recovery(1);
        wifimac->drv_priv->drv_config.cfg_burst_ack = 1;
        wnet_vif->vif_ops.write_word(DF_AGC_REG_A29, CCA_THRD_DEFAULT);
    }

    return;
}

void cca_thrd_cfg_change_task(SYS_TYPE param1, SYS_TYPE param2,SYS_TYPE param3, SYS_TYPE param4,SYS_TYPE param5)
{
    struct wlan_net_vif *wnet_vif = (struct wlan_net_vif *)param1;
    unsigned char cfg = (unsigned char)param2;

    cca_thrd_cfg_change_handle(wnet_vif, cfg);

    return;
}

static ssize_t show_cca_thrd_cfg(struct device *d, struct device_attribute *attr,
                 char *buf)
{
    struct wlan_net_vif *wnet_vif = g_wnet_vif0;
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;

    if (!wnet_vif)
        return -EFAULT;

    return sprintf(buf, "cfg: %s\n",wifimac->cca_thrd_cfg ? "ENABLE" : "DISABLE");
}

static ssize_t store_cca_thrd_cfg(struct device *d, struct device_attribute *attr,
                char *buf, ssize_t len)
{
    struct wlan_net_vif *wnet_vif = g_wnet_vif0;
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    unsigned char cfg = (unsigned char)simple_strtoul(buf,NULL,0);

    if (!wnet_vif || (cfg < 0) || (cfg > 1)) {
        AML_PRINT_LOG_ERR("The input mode is error");
        return -EFAULT;
    }

    if (wifimac->cca_thrd_cfg != cfg) {
        if ( !cfg) {
            cca_thrd_cfg_change_handle(wnet_vif,DISABLE);
        } else if (cfg && (wnet_vif->vm_state == WIFINET_S_CONNECTED)) {
            cca_thrd_cfg_change_handle(wnet_vif,ENABLE);
        }
        wifimac->cca_thrd_cfg = cfg;
    }

    AML_PRINT_LOG_INFO("wifimac->cca_thrd_cfg:%d \n",wifimac->cca_thrd_cfg);

    return len;
}

static DEVICE_ATTR(cca_thrd_cfg, S_IWUSR|S_IWGRP|S_IRUGO, show_cca_thrd_cfg, store_cca_thrd_cfg);

static ssize_t rate_statics_show(struct device *d, struct device_attribute *attr, char *buf)
{
    struct wlan_net_vif *wnet_vif = g_wnet_vif0;

    return aml_get_rvr_info(wnet_vif, buf, WRITE_FILE_NODE);
}

static DEVICE_ATTR(rate_statics, S_IRUGO, rate_statics_show, NULL);

static struct attribute *aml_sysfs_entries[] = {
        &dev_attr_hang_info.attr,
        &dev_attr_driver.attr,
        &dev_attr_band_info.attr,
        &dev_attr_band_query_flag.attr,
        &dev_attr_cur_channel.attr,
        &dev_attr_bandwidth.attr,
        &dev_attr_rRssi.attr,
        &dev_attr_iSnr.attr,
        &dev_attr_i4RSSI0.attr,
        &dev_attr_i4RSSI1.attr,
        &dev_attr_iSnrR0Phase2.attr,
        &dev_attr_iSnrR1Phase2.attr,
        &dev_attr_Glitch_Diff.attr,
        &dev_attr_Glitch_Total.attr,
        &dev_attr_TxRate.attr,
        &dev_attr_Data_RxRate.attr,
        &dev_attr_TxPktNum.attr,
        &dev_attr_TxFailNum.attr,
        &dev_attr_RxPktNum.attr,
        &dev_attr_RxFailNum.attr,
        &dev_attr_Average_RxRate.attr,
        &dev_attr_pno_sec_mode.attr,
        &dev_attr_hal_spec.attr,
        &dev_attr_iNoise.attr,
        &dev_attr_ssid.attr,
        &dev_attr_bssid.attr,
        &dev_attr_wakeup_reason.attr,
        &dev_attr_ap_info.attr,
        &dev_attr_cca_thrd_cfg.attr,
        &dev_attr_rate_statics.attr,
        NULL,
};


static struct attribute *aml_p2p_sysfs_entries[] = {
        &dev_attr_p2p_home_channel.attr,
        &dev_attr_p2p_channel.attr,
        &dev_attr_p2p_DevNum.attr,
        &dev_attr_p2p_bandwidth.attr,
        &dev_attr_p2p_rRssi.attr,
        &dev_attr_p2p_iNoise.attr,
        &dev_attr_p2p_iSnr.attr,
        &dev_attr_p2p_i4RSSI0.attr,
        &dev_attr_p2p_i4RSSI1.attr,
        &dev_attr_p2p_iSnrR0Phase2.attr,
        &dev_attr_p2p_iSnrR1Phase2.attr,
        &dev_attr_p2p_TxPkt.attr,
        &dev_attr_p2p_TxFailPkt.attr,
        &dev_attr_p2p_RxPkt.attr,
        &dev_attr_p2p_RxFailPkt.attr,
        &dev_attr_p2p_Glitch.attr,
        &dev_attr_p2p_RxRate.attr,
        &dev_attr_p2p_Mac.attr,
        &dev_attr_bypass_dfs.attr,
        &dev_attr_go_hidden_mode.attr,
        NULL,
};


static struct attribute_group aml_attribute_group = {
        .attrs = aml_sysfs_entries,
};

static struct attribute_group aml_p2p_attribute_group = {
        .attrs = aml_p2p_sysfs_entries,
};


int32_t sysfsCreateSysFsEntry(void)
{
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    struct wlan_net_vif *wnet_vif0 = NULL;
    struct wlan_net_vif *wnet_vif1 = NULL;
    int32_t ret = 0;

    wnet_vif0 = wifi_mac_get_wnet_vif_by_vid(wifimac, 0);
    g_wnet_vif0 = wnet_vif0;

    ret = sysfs_create_group(&wnet_vif0->vm_ndev->dev.kobj, &aml_attribute_group);
    if (ret < 0) {
        AML_PRINT_LOG_ERR("ERROR init sysfs failed\n");
    } else {
        AML_PRINT_LOG_INFO(" init sysfs succeed!!\n");
    }

    wnet_vif1 = wifi_mac_get_wnet_vif_by_vid(wifimac, 1);
    g_wnet_vif1 = wnet_vif1;

    ret = sysfs_create_group(&wnet_vif1->vm_ndev->dev.kobj, &aml_p2p_attribute_group);
    if (ret < 0) {
        AML_PRINT_LOG_ERR("ERROR init p2p sysfs failed\n");
    } else {
        AML_PRINT_LOG_INFO("init p2p sysfs succeed!!\n");
    }
    return 0;
}
void sysRemovesysfs(void)
{
    sysfs_remove_group(&g_wnet_vif0->vm_ndev->dev.kobj, &aml_attribute_group);
    sysfs_remove_group(&g_wnet_vif1->vm_ndev->dev.kobj, &aml_p2p_attribute_group);
    return;
}




