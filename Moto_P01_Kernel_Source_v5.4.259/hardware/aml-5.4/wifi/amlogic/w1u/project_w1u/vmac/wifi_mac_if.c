/*
 ****************************************************************************************
 *
 * Copyright (C) Amlogic 2010-2014
 *
 * Project: 11N 80211 mac  layer Software
 *
 * Description:
 *     wifi_mac layer interface function/interface use by driver layer
 *
 *
 ****************************************************************************************
 */

#include "wifi_mac_com.h"
#include "wifi_debug.h"
#include "wifi_drv_config.h"
#include <linux/inetdevice.h>
#include <net/addrconf.h>
#include "wifi_pkt_desc.h"
#include "wifi_iwpriv_cmd.h"
#include "wifi_mac_sae.h"
#include "wifi_common.h"
#include "wifi_mac_action.h"
#include "wifi_mac_tx_reg.h"
#include "wifi_cfg80211.h"
#include "wifi_cmd_func.h"
#include "chip_intf_reg.h"
#include "wifi_mac_timer_reg.h"
#include "aml_interface.h"
#include "aml_regdom.h"

#define ANT_SEL_MEAS_PERIOD      (30*1000)
#define ANT_SEL_MEAS_DURATION    (1000)

unsigned char special_frame_status_trace[][25] = {
    "[TX], ",
    "[RX], ",
    "[TX SUC], ",
    "[TX FAIL], "
};

unsigned char tpc_mode = 0;
extern unsigned char  host_wake_w1_fail_cnt;

#ifdef SDIO_MODE_ON
extern unsigned char wifi_in_insmod;
extern unsigned char chip_en_access;
#endif

static struct net_device_stats *wifi_mac_getstats(struct net_device *);
static  int wifi_mac_change_mtu(struct net_device *, int);
static int wifi_set_mac_address(struct net_device *dev, void *addr);

const char *wifi_mac_state_name[WIFINET_S_MAX] =
{
    "INIT",
    "SCAN",
    "CONNECTING",
    "AUTH",
    "ASSOC",
    "CONNECTED"
};
int g_auto_gain_base = 0;

static struct wifi_mac wm_mac;

unsigned char g_ant_mode_force = 0;
unsigned char g_ant_index = 0; //0,1
unsigned int  g_rssi_total[2] = {0};
unsigned short g_rssi_cnt[2] = {0};
unsigned char g_rssi_val[2] = {0};
unsigned char g_ant_try[2] = {0};

unsigned long g_switch_stamp = 0;

void wifi_mac_reset_ant_measure(void)
{
    g_rssi_total[0] = 0;
    g_rssi_total[1] = 0;
    g_rssi_cnt[0] = 0;
    g_rssi_cnt[1] = 0;
    g_rssi_val[0] = 0;
    g_rssi_val[1] = 0;
    g_ant_try[0] = 0;
    g_ant_try[1] = 0;
    g_switch_stamp = 0;
}

unsigned char wifi_mac_get_cur_ant(void)
{
   return g_ant_index;
}

unsigned char wifi_mac_sync_using_ant(unsigned char ant)
{
   g_ant_index = ant;
   return g_ant_index;
}

void wifi_mac_set_ant_flag(unsigned char id)
{
    g_ant_try[id] = 1;
}

unsigned char wifi_mac_get_ant_flag(unsigned char id)
{
    return g_ant_try[id];
}

void wifi_mac_ant_init(struct hal_private * hal_priv)
{
    unsigned int temp;

    AML_PRINT_LOG_INFO("<running>\n");

    HAL_SPEC_BEGIN_LOCK();
    temp = hal_priv->hal_ops.hal_read_word(RG_AON_A1);
    temp &= ~BIT(10);
    hal_priv->hal_ops.hal_write_word(RG_AON_A1, temp);

    temp = hal_priv->hal_ops.hal_read_word(RG_AON_A2);
    temp &= ~BIT(10);
    hal_priv->hal_ops.hal_write_word(RG_AON_A2, temp);

    temp = hal_priv->hal_ops.hal_read_word(RG_AON_A5);
    temp |= BIT(10);
    hal_priv->hal_ops.hal_write_word(RG_AON_A5, temp);

    temp = hal_priv->hal_ops.hal_read_word(RG_AON_A11);
    temp &= ~BIT(10);
    hal_priv->hal_ops.hal_write_word(RG_AON_A11, temp);

    temp = hal_priv->hal_ops.hal_read_word(RG_AON_A10);
    temp &= ~(BIT(10));
    hal_priv->hal_ops.hal_write_word(RG_AON_A10, temp);
    HAL_SPEC_END_LOCK();
}

void wifi_mac_switch_ant_task(SYS_TYPE param1,SYS_TYPE param2, SYS_TYPE param3,SYS_TYPE param4,SYS_TYPE param5)
{
    struct hal_private * hal_priv = hal_get_priv();
    unsigned int temp;

    g_ant_index = (g_ant_index+1)%2;

    temp = hal_priv->hal_ops.hal_read_word(RG_AON_A10);
    if (g_ant_index) {
        temp |= BIT(10);
    }
    else {
        temp &= ~(BIT(10));
    }
    hal_priv->hal_ops.hal_write_word(RG_AON_A10, temp);
}

void wifi_mac_switch_ant(void *arg)
{
    struct wifi_mac *wifimac = (struct wifi_mac *)arg;

    wifi_mac_add_work_task(wifimac, wifi_mac_switch_ant_task, NULL, 0, 0, 0, 0, 0);

    return ;
}

void wifi_mac_rssi_sum(unsigned char rssi)
{

    if (g_ant_mode_force != 0)
        return ;

    if (g_switch_stamp == 0)
        g_switch_stamp = jiffies;

    if (g_switch_stamp > jiffies)
        return;

    if (g_switch_stamp + 500 > jiffies) {
        g_rssi_total[g_ant_index] += rssi;
        g_rssi_cnt[g_ant_index]++;
    }
}

unsigned char wifi_mac_cur_ant_rssi(unsigned char *rssi)
{
    if (g_rssi_cnt[g_ant_index] == 0) {
        return 0;
    }
    else
        g_rssi_val[g_ant_index] = g_rssi_total[g_ant_index] / g_rssi_cnt[g_ant_index];
    *rssi = g_rssi_val[g_ant_index];

    return 1;
}

unsigned char wifi_mac_div_ant_rssi(unsigned char *rssi)
{
    if (g_rssi_cnt[(g_ant_index+1)%2] == 0) {
        return 0;
    }
    else
        g_rssi_val[(g_ant_index+1)%2] = g_rssi_total[(g_ant_index+1)%2] / g_rssi_cnt[(g_ant_index+1)%2];
    *rssi = g_rssi_val[(g_ant_index+1)%2];

    return 1;
}

struct wifi_mac* wifi_mac_get_mac_handle(void)
{
    return &wm_mac;
}

unsigned char wifi_mac_get_host_wake_status(void)
{
    return host_wake_w1_fail_cnt;
}

unsigned char wifi_mac_clear_host_wake_status(void)
{
    host_wake_w1_fail_cnt = 0;
    return 0;
}

#ifdef SDIO_MODE_ON
unsigned char wifi_mac_set_wifi_insmod_status(void)
{
    wifi_in_insmod = 1;
    return 0;
}

unsigned char wifi_mac_get_chip_en_access(void)
{
    return chip_en_access;
}
#endif

enum hal_op_mode wifi_mac_opmode_2_halmode(enum wifi_mac_opmode opmode)
{
    enum hal_op_mode halmode;
    switch(opmode)
    {
        case WIFINET_M_STA:
            halmode = WIFI_M_STA;
            break;
        case WIFINET_M_IBSS:
            halmode = WIFI_M_IBSS;
            break;
        case WIFINET_M_HOSTAP:
            halmode = WIFI_M_HOSTAP;
            break;
        case WIFINET_M_MONITOR:
            halmode = WIFI_M_MONITOR;
            break;
        case WIFINET_M_WDS:
            halmode = WIFI_M_WDS;
            break;
        case WIFINET_M_P2P_CLIENT:
            halmode = WIFI_M_STA;
            break;
        case WIFINET_M_P2P_GO:
            halmode = WIFI_M_HOSTAP;
            break;
        default:
            halmode = WIFI_M_STA;
            break;
    }
    return halmode;
}

void *wifi_mac_alloc_ndev(unsigned int len)
{
    struct net_device *dev;
    struct wlan_net_vif *wnet_vif;

    dev = alloc_etherdev(len);
    if (dev == NULL)
        return NULL;

    wnet_vif = netdev_priv(dev);
    wnet_vif->vm_ndev = dev;

    AML_PRINT_LOG_INFO("<running>\n");
    return wnet_vif;
}

void wifi_mac_free_vmac(void *netif)
{
     AML_PRINT_LOG_INFO("<running>\n");
}

void
wifi_mac_vmac_detach(struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    int ret;

    if(wnet_vif->vm_state != WIFINET_S_INIT)
    {
        wifi_mac_stop(wnet_vif->vm_ndev);
    }
    KASSERT(wnet_vif->vm_state == WIFINET_S_INIT, ("wnet_vif not stopped"));

    wifi_mac_beacon_free( wnet_vif->vm_wmac , wnet_vif->wnet_vif_id);
    memset(&wifimac->wm_vendorinfo, 0, sizeof(struct wifi_mac_vendor_ie) * VENDOR_IE_MAX);
    ret = wifimac->drv_priv->drv_ops.remove_interface(wifimac->drv_priv, wnet_vif->wnet_vif_id);
    KASSERT(ret == 0, ("invalid interface id"));

    wnet_vif->wnet_vif_replaycounter++;

    wifi_mac_free_vmac(wnet_vif);
    vm_wlan_net_vif_detach(wnet_vif);
}

void wifi_mac_sta_free(struct wifi_station *sta)
{
    struct wifi_mac *wifimac = sta->sta_wmac;
    void * drv_sta = sta->drv_sta;

    if(drv_sta  == NULL) {
        AML_PRINT(AML_LOG_ID_NODE, AML_LOG_LEVEL_WARN, " \n");
        return;
    }
    wifi_mac_amsdu_nsta_free(wifimac, sta);

    wifimac->drv_priv->drv_ops.free_nsta(wifimac->drv_priv, drv_sta);
    sta->drv_sta = NULL;
}

void wifi_mac_sta_clean(struct wifi_station *sta)
{
    struct wifi_mac *wifimac = sta->sta_wmac;

    wifimac->drv_priv->drv_ops.cleanup_nsta(wifimac->drv_priv, sta->drv_sta);
}

int wifi_mac_reset(struct wifi_mac *wifimac)
{
    int error;

    error = wifimac->drv_priv->drv_ops.reset(wifimac->drv_priv);

    return error;
}

unsigned int wifi_mac_mhz2chan(unsigned int freq)
{
    unsigned int  chan = 1;

    if (freq>5000) {
        chan = (freq - 5000) / (5);

    } else if ((freq > 2407) && (freq < 2484)) {
        chan = (freq - 2407) / 5;

    } else if (freq == 2484) {
        chan = 14;

    } else {
        ASSERT(0);
    }

    return chan;
}

void wifi_mac_rate_nsta_update(void* ieee, void* nsta, int isnew)
{
    struct wifi_station *sta = (struct wifi_station *)nsta;

    wifi_mac_rate_newassoc(sta, isnew);
}

void
wifi_mac_new_assoc(struct wifi_station *sta, int isnew)
{
    struct wifi_mac *wifimac = sta->sta_wmac;

    wifi_mac_rate_nsta_update(wifimac, sta, isnew);
    wifimac->drv_priv->drv_ops.new_assoc( wifimac->drv_priv, sta->drv_sta, isnew,
            ((sta->sta_flags & WIFINET_NODE_UAPSD)? 1: 0) );
}

void wifi_mac_scan_start(struct wifi_mac *wifimac)
{
    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG,"\n");
    wifimac->wm_syncbeacon = 0;
    wifimac->drv_priv->drv_ops.scan_start(wifimac->drv_priv);
}

void
wifi_mac_scan_end(struct wifi_mac *wifimac)
{
    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG,"\n");
    wifimac->drv_priv->drv_ops.scan_end(wifimac->drv_priv);
}

void
wifi_mac_connect_start(struct wifi_mac *wifimac)
{
    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG,"\n");
    wifimac->drv_priv->drv_ops.connect_start(wifimac->drv_priv);
}

void
wifi_mac_connect_end(struct wifi_mac *wifimac)
{
    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG,"\n");
    wifimac->drv_priv->drv_ops.connect_end(wifimac->drv_priv);
}

void wifi_mac_scan_set_gain(struct wifi_mac *wifimac, unsigned char rssi)
{
    // AML_PRINT_LOG_INFO("scan rssi:%d\n", 256 - rssi);
    wifimac->drv_priv->drv_ops.set_channel_rssi(wifimac->drv_priv, rssi, 0);
}

void wifi_mac_set_channel_rssi(struct wifi_mac *wifimac, unsigned char rssi)
{
    unsigned char rssi_set = rssi;
    //struct hw_interface* hif = hif_get_hw_interface();
    if (wifimac->bt_lk && rssi >= 211 && WIFINET_IS_CHAN_2GHZ(wifimac->drv_priv->drv_wnet_vif_table[0]->vm_curchan)) {
        //set 3,3
        wifimac->drv_priv->drv_ops.set_channel_rssi(wifimac->drv_priv, 0xf8, 0);
        AML_PRINT_LOG_INFO("coexist set_gain %d-> %d\n", wifimac->bt_lk, 256 - 0xf8);

    } else if (wifimac->is_connect_set_gain) {
        wifimac->drv_priv->drv_ops.set_channel_rssi(wifimac->drv_priv, rssi_set, 0);
        AML_PRINT_LOG_INFO("is_connect_set_gain %d-> %d\n", wifimac->is_connect_set_gain, 256 - rssi_set);

    } else {
        //for high gain setting
        wifimac->drv_priv->drv_ops.set_channel_rssi(wifimac->drv_priv, rssi_set, 1);
        AML_PRINT_LOG_INFO("rvr_set_gain %d-> %d\n", wifimac->is_connect_set_gain, 256 - rssi_set);
    }

}

int wifi_mac_is_in_noisy_environment(struct wifi_mac *wifimac)
{
    int ret = 0;

    if ((wifimac->wm_nrunning == 0 && wifimac->wm_scan->scan_ssid_count >= wifimac->scan_gain_thresh_unconnect)
        || (wifimac->wm_nrunning != 0 && wifimac->wm_scan->scan_ssid_count >= wifimac->scan_gain_thresh_connect)) {
        ret = 1;
    }

    return ret;
}

int wifi_mac_is_in_clear_environment(struct wifi_mac *wifimac)
{
    int ret = 0;

    if (wifimac->wm_scan->scan_ssid_count < wifimac->scan_max_gain_thresh) {
        ret = 1;
    }

    return ret;
}


/* if wifimac->wm_scan->scan_ssid_count >= UNCONNECT_MIN_GIAN_THRESHOLD/CONNECT_MIN_GIAN_THRESHOLD always use MAC_MIN_GAIN
   if wifimac->wm_scan->scan_ssid_count <  MAX_GIAN_THRESHOLD always use MAC_MAX_GAIN
   if MAX_GIAN_THRESHOLD < wifimac->wm_scan->scan_ssid_count < UNCONNECT_MIN_GIAN_THRESHOLD/CONNECT_MIN_GIAN_THRESHOLD use MAC_MID_GAIN
*/
void wifi_mac_get_channel_rssi_before_scan(struct wifi_mac *wifimac, int *rssi)
{
    int channel_set_rssi = 0;

    AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_INFO, "last rssi:%d\n", *rssi);
    channel_set_rssi = MAC_MIN_GAIN;

    switch (*rssi) {
        case MAC_MAX_GAIN:
            if (wifimac->scan_noisy_status == WIFINET_S_SCAN_ENV_CLEAR) {
                channel_set_rssi = MAC_MAX_GAIN;

            } else {
                channel_set_rssi = MAC_MIN_GAIN;
            }
            break;

        case MAC_MID_GAIN:
            if (wifimac->scan_noisy_status == WIFINET_S_SCAN_ENV_CLEAR) {
                channel_set_rssi = MAC_MAX_GAIN;

            } else if (wifimac->scan_noisy_status == WIFINET_S_SCAN_ENV_NOISE) {
                channel_set_rssi = MAC_MIN_GAIN;

            } else {
                channel_set_rssi = MAC_MID_GAIN;
            }
            break;

        case MAC_MIN_GAIN:
            if (wifimac->scan_noisy_status == WIFINET_S_SCAN_ENV_NOISE) {
                channel_set_rssi = MAC_MIN_GAIN;

            } else {
                channel_set_rssi = MAC_MID_GAIN;
            }
            break;

        default:
            break;
    }

    AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_INFO, " current rssi:%d\n", channel_set_rssi);
    *rssi = channel_set_rssi;
}

void wifi_mac_set_tx_power_accord_rssi(struct wifi_mac *wifimac, unsigned char rssi)
{
    struct hal_channel hchan;
    unsigned char power_mode = 0;
    static unsigned char rssi_increase_num = 0;
    static unsigned char rssi_diminish_num = 0;

    if (rssi <= 186) { /* -70dbm */
        rssi_diminish_num++;
        rssi_increase_num = 0;

    } else if (rssi >= 196) { /* -60dbm */
        rssi_increase_num++;
        rssi_diminish_num = 0;

    } else {
        rssi_diminish_num = 0;
        rssi_increase_num = 0;
        return;
    }

    if (rssi_diminish_num == 10 && wifimac->wm_curchan->chan_bw == 2) {
        power_mode = 2;

    } else if (rssi_increase_num == 10) {
        power_mode = 1;
    }

    if (power_mode && (power_mode != tpc_mode)) {
        hchan.channel = wifi_mac_Mhz2ieee(wifimac->wm_curchan->chan_cfreq1, 0);
        hchan.chan_bw = wifimac->wm_curchan->chan_bw;
        AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_DEBUG, "tx_power_accord rssi:%d\n", rssi - 256);
        wifimac->drv_priv->drv_ops.set_tx_power_accord_rssi(wifimac->drv_priv, &hchan, rssi, power_mode);
        tpc_mode = power_mode;
    }

    if (rssi_diminish_num > 20) {
        rssi_diminish_num = 20;
    }

    if (rssi_increase_num > 20) {
        rssi_increase_num = 20;
    }
}


int
wifi_mac_wmm_update2hal(struct wifi_mac *wifimac, struct wlan_net_vif *wnet_vif)
{
    int ac;
    struct wmeParams *wmep;

    for (ac = 0; ac < WME_NUM_AC; ac++)
    {
        wmep = wifi_mac_wmm_chanparams(wnet_vif,wifimac, ac);
        if (wifimac->drv_priv->drv_ops.tx_wmm_queue_update(wifimac->drv_priv,
                    wnet_vif->wnet_vif_id, ac,
                    wmep->wmep_aifsn,wmep->wmep_logcwmin,
                    wmep->wmep_logcwmax,wmep->wmep_txopLimit) != 0)
        {
            return -EIO;
        }
    }
    return 0;
}


void
wifi_mac_com_ps_set_state(struct wifi_mac *wifimac,
    enum wifinet_ps_state newstate, int wnet_vif_id)
{
    AML_PRINT(AML_LOG_ID_PWR_SAVE,AML_LOG_LEVEL_DEBUG, "state-> %d\n", newstate);
    switch (newstate)
    {
        case WIFINET_PWRSAVE_AWAKE:
            wifimac->drv_priv->drv_ops.awake(wifimac->drv_priv, wnet_vif_id);
            break;
        case WIFINET_PWRSAVE_NETWORK_SLEEP:
            wifimac->drv_priv->drv_ops.netsleep(wifimac->drv_priv, wnet_vif_id);
            break;
        case WIFINET_PWRSAVE_FULL_SLEEP:
            wifimac->drv_priv->drv_ops.fullsleep(wifimac->drv_priv, wnet_vif_id);
            break;
        default:
            break;
    }
}

int wifi_mac_build_txinfo(struct wifi_mac *wifimac, struct sk_buff *skbbuf, struct wifi_mac_tx_info *txinfo)
{
    struct wifi_station *sta =  os_skb_get_nsta(skbbuf);
    struct wlan_net_vif *wnet_vif = sta->sta_wnet_vif;
    struct wifi_frame *wh;
    int key_index, pktlen;
    int type, subtype;
    enum hal_key_type key_type = HAL_KEY_TYPE_CLEAR;

    wh = (struct wifi_frame *)os_skb_data(skbbuf);

    txinfo->b_mcast = WIFINET_IS_MULTICAST(wh->i_addr1);
    txinfo->b_txfrag = (wh->i_fc[1] & WIFINET_FC1_MORE_FRAG) ||
        (((le16toh(*((unsigned short *)&(wh->i_seq[0]))) >> WIFINET_SEQ_FRAG_SHIFT) & WIFINET_SEQ_FRAG_MASK) > 0);

    type = wh->i_fc[0] & WIFINET_FC0_TYPE_MASK;
    subtype = wh->i_fc[0] & WIFINET_FC0_SUBTYPE_MASK;

    pktlen = os_skb_get_pktlen(skbbuf);
    txinfo->mpdulen= pktlen;

    if ((wh->i_fc[1] & WIFINET_FC1_WEP) && !txinfo->b_pmf)
    {
        const struct wifi_mac_security *cip;
        struct wifi_mac_key *k;

        k = wifi_mac_security_encap(sta, skbbuf);

        if (k == NULL)
        {
            WIFINET_DPRINTF(AML_LOG_ID_LOG, AML_LOG_LEVEL_DEBUG, "%s", "(k == NULL) err!!");
            return -EIO;
        }
        wh = (struct wifi_frame *)os_skb_data(skbbuf);

        cip = k->wk_cipher;
        pktlen += cip->wm_header + cip->wm_trailer;
        if ((k->wk_flags & WIFINET_KEY_SWMIC) == 0)
        {
            if ( ! txinfo->b_txfrag)
                pktlen += cip->wm_miclen;
            else
            {
                if (cip->wm_cipher != WIFINET_CIPHER_TKIP)
                    pktlen += cip->wm_miclen;
            }
        }

        switch (cip->wm_cipher)
        {
            case WIFINET_CIPHER_WEP:
                key_type = HAL_KEY_TYPE_WEP;
                break;
            case WIFINET_CIPHER_TKIP:
                key_type = HAL_KEY_TYPE_TKIP;
                pktlen += DP_SEC_TKIP_MIC_LEN;
                break;
            case WIFINET_CIPHER_AES_OCB:
            case WIFINET_CIPHER_AES_CCM:
                key_type = HAL_KEY_TYPE_AES;
                break;
#ifdef CONFIG_WAPI
            case WIFINET_CIPHER_WPI:
                key_type = HAL_KEY_TYPE_WPI;
                break;
#endif //#ifdef CONFIG_WAPI
            default:
                key_type = HAL_KEY_TYPE_CLEAR;
        }
        key_index = k->wk_keyix;
    }
    else
    {
        key_index = HAL_TXKEYIX_INVALID;
    }
    pktlen += WIFINET_CRC_LEN;

    txinfo->packetlen = pktlen;
    txinfo->key_index = key_index;
    txinfo->key_type = key_type;
    txinfo->tid_index =  os_skb_get_tid(skbbuf);

    if (wifi_mac_com_has_flag(wifimac, WIFINET_F_SHPREAMBLE) &&
        !wifi_mac_com_has_flag(wifimac, WIFINET_F_USEBARKER) &&
        wifi_macnsta_has_cap(sta, WIFINET_CAPINFO_SHORT_PREAMBLE))
    {
        txinfo->shortPreamble = 1;
    }

    switch (type)
    {
        case WIFINET_FC0_TYPE_MGT:
            txinfo->b_mgmt = 1;
            txinfo->queue_id = wifimac->wm_mng_qid;
            break;

        case WIFINET_FC0_TYPE_CTL:
            switch (subtype)
            {
                case WIFINET_FC0_SUBTYPE_PS_POLL:
                    txinfo->b_PsPoll = 1;
                    break;

                case WIFINET_FC0_SUBTYPE_BAR:
                    txinfo->b_BarPkt = 1;
                    break;

                default:
                    break;
            }
            txinfo->queue_id = wifimac->wm_mng_qid;;
            break;

        case WIFINET_FC0_TYPE_DATA:
            txinfo->b_datapkt = 1;
            txinfo->b_uapsd =  os_skb_is_uapsd(skbbuf);

            if (wh->i_fc[0] & WIFINET_FC0_SUBTYPE_QOS)
            {
                int ac =  os_skb_get_priority(skbbuf);
                txinfo->b_qosdata = 1;

                txinfo->queue_id = wifimac->wm_ac2q[ac & 0x3];
                if (wifi_mac_wmm_chanparams(wnet_vif,wifimac, ac)->wmep_noackPolicy)
                    txinfo->b_noack = 1;

            }
            else
            {
                txinfo->queue_id = wifimac->wm_noqos_legacy_qid;
                os_skb_set_priority(skbbuf, WME_AC_BE);
            }

            if (txinfo->b_uapsd)
            {
                txinfo->queue_id = wifimac->wm_uapsd_qid;
            }

            if (txinfo->b_mcast)
            {
                txinfo->queue_id = wifimac->wm_mcast_qnum;
            }

            if (!txinfo->b_mcast && WIFINET_NODE_USEAMPDU(sta))
            {
                if (WIFINET_QOS_HAS_SEQ(wh))
                {
                    txinfo->ht = 1;
                }
            }

            if ((subtype == WIFINET_FC0_SUBTYPE_NODATA)
                    || (subtype == WIFINET_FC0_SUBTYPE_QOS_NULL))
            {
                txinfo->queue_id = wifimac->wm_mng_qid;
                txinfo->ht = 0;
                txinfo->b_nulldata = 1;
            }

            if (M_FLAG_GET(skbbuf, M_CHECKSUMHW))
            {
                txinfo->b_hwchecksum = 1;
            }
            if (M_FLAG_GET(skbbuf, M_TKIPMICHW))
            {
                txinfo->b_hwtkipmic= 1;
            }
            break;
        default:
            return -EIO;
    }

    txinfo->wnet_vif_id = wnet_vif->wnet_vif_id;
    if (((wnet_vif->vm_opmode == WIFINET_M_HOSTAP) || (wnet_vif->vm_opmode == WIFINET_M_P2P_GO))
        && (wnet_vif->vm_ps_sta != 0))
        txinfo->ps = 1;

    txinfo->b_buffered = 0;
    if (txinfo->b_mcast)
    {
        txinfo->b_noack = 1;
    }
    else if (pktlen > wnet_vif->vm_rtsthreshold)
    {
        txinfo->b_rsten = 1;
    }

    return 0;
}

void wifi_mac_tx_addba_check(struct wifi_station *sta, unsigned char tid_index)
{
    struct wifi_mac *wifimac = sta->sta_wmac;
    struct wlan_net_vif *wnet_vif = sta->sta_wnet_vif;

    if (WIFINET_NODE_USEAMPDU(sta)
        && wifimac->drv_priv->drv_ops.check_aggr(wifimac->drv_priv, sta->drv_sta, tid_index)
        && (wnet_vif->vm_opmode != WIFINET_M_HOSTAP || softap_get_sta_num(wnet_vif) < 2))
    {
        struct wifi_mac_action_mgt_args actionargs;
        AML_PRINT(AML_LOG_ID_ADDBA, AML_LOG_LEVEL_INFO,"<running>  vif_id(%d), tid_index = %d \n", wnet_vif->wnet_vif_id, tid_index);

        actionargs.category = AML_CATEGORY_BACK;
        actionargs.action = WIFINET_ACTION_BA_ADDBA_REQUEST;
        actionargs.arg1 = tid_index;
        actionargs.arg2 = WME_MAX_BA;
        actionargs.arg3 = 0;
        wifi_mac_send_action(sta, (void *)&actionargs);
    }
}

int wifi_mac_tx_send(struct sk_buff *skbbuf)
{
    struct wifi_station *sta = os_skb_get_nsta(skbbuf);
    struct wlan_net_vif *wnet_vif = netdev_priv(skbbuf->dev);
    struct wifi_mac *wifimac = sta->sta_wmac;
    struct sk_buff *next_wbuf;
    unsigned char is_amsdu_support = 0;
    struct wifi_mac_tx_info *txinfo = NULL;
    struct wifi_mac_pkt_info *mac_pkt_info;
    struct wifi_frame *wh;
    struct ether_header eh;
    memcpy(&eh, os_skb_data(skbbuf), sizeof(struct ether_header));

    txinfo = (struct wifi_mac_tx_info *)os_skb_cb(skbbuf);
    ASSERT(sizeof(struct wifi_mac_tx_info) <= OS_SKB_CB_MAXLEN);

    mac_pkt_info= &txinfo->ptxdesc->drv_pkt_info.pkt_info[0];
    if ((mac_pkt_info->b_eap == 0) && (mac_pkt_info->b_arp == 0) && (mac_pkt_info->b_dhcp == 0) &&
         (!((sta->sta_wnet_vif->vm_opmode == WIFINET_M_HOSTAP) && (WIFINET_IS_MULTICAST(eh.ether_dhost))))) {
        /* For hostap/go, if DA is bcmc address, it's a broadcast/multicast 802.11 frame,
         * no need trigger to setup tx ba session. Otherwise, it will sendout a ToME addba
         * request frame.
         */
        wifi_mac_tx_addba_check(sta, os_skb_get_tid(skbbuf));
    }

    is_amsdu_support = wifimac->drv_priv->drv_ops.get_amsdu_supported(wifimac->drv_priv, sta->drv_sta, os_skb_get_tid(skbbuf));
    if (is_amsdu_support && !mac_pkt_info->b_dhcp && !mac_pkt_info->b_eap && !mac_pkt_info->b_arp) {
        skbbuf = wifi_mac_amsdu_send(skbbuf);
        if (skbbuf == NULL) {
            return NETDEV_TX_OK;
        }
    }

    wnet_vif->vif_sts.sts_tx_non_amsdu_in_msdu[os_skb_get_tid(skbbuf)]++;

    skbbuf = wifi_mac_encap(sta, skbbuf);
    if (skbbuf == NULL) {
        AML_PRINT(AML_LOG_ID_XMIT, AML_LOG_LEVEL_ERROR,"<running> \n");
        goto bad;
    }

    while (skbbuf != NULL) {
        txinfo = (struct wifi_mac_tx_info *)os_skb_cb(skbbuf);
        memset(&txinfo->wnet_vif_id, 0, sizeof(struct wifi_mac_tx_info)-sizeof(struct wifi_skb_callback)-sizeof(struct drv_txdesc *));
        next_wbuf = os_skb_next(skbbuf);

        if (wifi_mac_build_txinfo(wifimac, skbbuf, txinfo) != 0) {
            AML_PRINT_LOG_ERR("<running>\n");
            goto bad;
        }

        wh = (struct wifi_frame *)os_skb_data(skbbuf);
        if (((wnet_vif->pkt_ctrl.flag & PKT_STOP_SEND) && WIFINET_ADDR_EQ(wnet_vif->pkt_ctrl.addr, wh->i_addr1) && !mac_pkt_info->b_eap && !txinfo->b_mcast) || (sta->wnet_vif_id == 0 && sta->sta_is_adding_key == true)) {
            AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_DEBUG,"add key not complete drop the unicast data \n");
            goto bad;
        }

        wnet_vif->vif_sts.sts_tx_non_amsdu_out_msdu[os_skb_get_tid(skbbuf)]++;
        if (wifimac->drv_priv->drv_ops.tx_start(wifimac->drv_priv, skbbuf) != 0) {
            skbbuf = next_wbuf;
            while (skbbuf != NULL) {
                next_wbuf = os_skb_next(skbbuf);
                os_skb_next(skbbuf) = NULL;
                wifi_mac_complete_wbuf(skbbuf, 0);
                skbbuf = next_wbuf;
            }
            return NETDEV_TX_OK;
        }

        skbbuf = next_wbuf;
    }

    return NETDEV_TX_OK;

bad:
    AML_PRINT(AML_LOG_ID_XMIT, AML_LOG_LEVEL_ERROR,"<%s> drop frame\n", VMAC_DEV_NAME(sta->sta_wnet_vif));

    while (skbbuf != NULL) {
        next_wbuf =  os_skb_next(skbbuf);
        os_skb_next(skbbuf) = NULL;
        wifi_mac_complete_wbuf(skbbuf, 0);

        wnet_vif->vm_devstats.tx_dropped++;
        wnet_vif->vif_sts.sts_tx_non_amsdu_drop_msdu[os_skb_get_tid(skbbuf)]++;
        skbbuf = next_wbuf;
    }

    return NETDEV_TX_OK;
}

extern char dpp_pub_action_str[][50];
unsigned char wifi_mac_is_dpp_frame(struct sk_buff *skb, struct wifi_station *sta)
{
    struct wifi_mac_p2p_pub_act_frame *p2p_pub_act = NULL;
    unsigned char *buf = (unsigned char *)os_skb_data(skb);
    bool is_dpp_frame = false;

    p2p_pub_act = (struct wifi_mac_p2p_pub_act_frame *)(buf + sizeof(struct wifi_frame));

    if (p2p_pub_act && (p2p_pub_act->category == AML_CATEGORY_PUBLIC) && (p2p_pub_act->action == WIFINET_ACT_PUBLIC_P2P)) {
        if (p2p_pub_act->oui_type == OUI_TYPE_DPP) {
            unsigned char dpp_action_subtype = *(buf + sizeof(struct wifi_frame) + 7);
            if (dpp_action_subtype == 11) {
                concurrent_channel_protection(sta->sta_wnet_vif, 2000);
            }

            if (dpp_action_subtype < 19) {
                AML_PRINT_LOG_INFO("DPP->%s %d\n", dpp_pub_action_str[dpp_action_subtype], dpp_action_subtype);
            } else {
                AML_PRINT_LOG_INFO("subtype:%d\n", dpp_action_subtype);
            }
            is_dpp_frame = true;
        }
    }
    return is_dpp_frame;
}

int wifi_mac_tx_mgmt_frm(struct wifi_mac *wifimac, struct sk_buff *skbbuf)
{
    int error = 0;
    struct wifi_mac_tx_info *txinfo = NULL;
    struct wifi_frame *wh;
    struct wifi_station *sta;
    unsigned char pkt_len = 0;
    unsigned char is_robust_mgmt = 0;

    if (aml_wifi_is_enable_rf_test()) {
        wifi_mac_complete_wbuf(skbbuf, 0);
        return error;
    }

    txinfo = (struct wifi_mac_tx_info *)os_skb_cb(skbbuf);
    sta = txinfo->cb.sta;
    ASSERT(sizeof(struct wifi_mac_tx_info) <= OS_SKB_CB_MAXLEN);
    memset(&txinfo->wnet_vif_id, 0, sizeof(struct wifi_mac_tx_info)-sizeof(struct wifi_skb_callback)-sizeof(struct drv_txdesc *));
    wifi_mac_pwrsave_txpre(skbbuf);
    wh = (struct wifi_frame *)os_skb_data(skbbuf);
    is_robust_mgmt = is_robust_management_frame(wh);

    if (wifi_mac_is_dpp_frame(skbbuf, sta) == true) {
        txinfo->b_dpp = 1;
    }

    if ((sta->sta_flags_ext & WIFINET_NODE_MFP) && is_robust_mgmt && sta->pmf_key.wk_keylen) {
        if (WIFINET_IS_MULTICAST(wh->i_addr1)) {
            if (sta->pmf_key.wk_key_type == WIFINET_CIPHER_AES_CMAC) {
                pkt_len = MME_IE_LENGTH_CMAC;

            } else {
                pkt_len = MME_IE_LENGTH;
            }

            os_skb_put(skbbuf, pkt_len);
            if (wifi_mac_add_mmie(sta, skbbuf)) {
                goto fail;
            }
        }

        *(unsigned short *)&wh->i_seq[0] = htole16(sta->sta_txseqs[0] << WIFINET_SEQ_SEQ_SHIFT);
        sta->sta_txseqs[0]++;
        txinfo->b_pmf = 1;
        wh->i_fc[1] |= WIFINET_FC1_WEP;

        PN_LOCK();//for PN
        if (!wifi_mac_sw_encrypt(sta, skbbuf)) {
            PN_UNLOCK();
            goto fail;
        }
        PN_UNLOCK();
    }

    error = wifi_mac_build_txinfo(wifimac, skbbuf, txinfo);
    if (!error) {
        error = wifimac->drv_priv->drv_ops.tx_start(wifimac->drv_priv, skbbuf);
        return error;
    }

fail:
    wifi_mac_complete_wbuf(skbbuf, 0);
    return error;
}

unsigned int
wifi_mac_all_txq_all_qcnt(struct wifi_mac *wifimac)
{
    int ac, qqcnt = 0;

    for (ac = WME_AC_BE; ac <= WME_AC_VO; ac++)
    {
        qqcnt += wifimac->drv_priv->drv_ops.txlist_all_qcnt(wifimac->drv_priv, wifimac->wm_ac2q[ac]);
    }
    qqcnt += wifimac->drv_priv->drv_ops.txlist_all_qcnt(wifimac->drv_priv, wifimac->wm_uapsd_qid);
    qqcnt += wifimac->drv_priv->drv_ops.txlist_all_qcnt(wifimac->drv_priv, wifimac->wm_noqos_legacy_qid);
    qqcnt += wifimac->drv_priv->drv_ops.txlist_all_qcnt(wifimac->drv_priv, wifimac->wm_mng_qid);
    qqcnt += wifimac->drv_priv->drv_ops.txlist_all_qcnt(wifimac->drv_priv, wifimac->wm_mcast_qnum);
    return qqcnt;
}


static void
wifi_mac_tx_complete(struct wlan_net_vif *wnet_vif, struct sk_buff *skbbuf, struct wifi_mac_tx_status *ts)
{
    if (skbbuf == NULL) {
        return;
    }

    if ((wnet_vif != NULL) && (ts->ts_flags & WIFINET_TX_ERROR)) {
        wnet_vif->vm_devstats.tx_errors++;
    }

    wifi_mac_free_skb(skbbuf);

    if (wnet_vif->vm_wmac->txdesc_flag & TXDESC_FLOW_CTRL_EN) {
        wifi_mac_wake_queues(wnet_vif);
    }
}

static void
wifi_mac_tx_status(void* nsta, struct wifi_mac_tx_status * ts)
{
    struct wifi_station *sta = (struct wifi_station *)nsta;
    struct wlan_net_vif *wnet_vif;

    if (sta == NULL) {
        return;
    }
    wnet_vif = sta->sta_wnet_vif;

    if ((wnet_vif != NULL) && (wnet_vif->vm_mainsta != NULL))
    {
        sta->sta_rstamp = jiffies;

        if (ts->ts_ratekbps > 0)
        {
            sta->sta_last_txrate = ts->ts_ratekbps;
            wnet_vif->vm_mainsta->sta_last_txrate = ts->ts_ratekbps;
            sta->last_txrate_bw = (ts->ts_flags & IEEE80211_TX_RC_80_MHZ_WIDTH) ? BW_80 : (ts->ts_flags & IEEE80211_TX_RC_40_MHZ_WIDTH) ? BW_40 : BW_20;
        }
    }
}

static void
wifi_mac_updateslot_ex(SYS_TYPE param1,SYS_TYPE param2,SYS_TYPE param3,
        SYS_TYPE param4,SYS_TYPE param5)
{
    struct wifi_mac *wifimac = (struct wifi_mac *)param1;
    int slottime;

    slottime = (wifi_mac_com_has_flag(wifimac, WIFINET_F_SHSLOT)) ?
        HAL_SLOT_TIME_9 : HAL_SLOT_TIME_20;
    wifimac->drv_priv->drv_ops.set_slottime(wifimac->drv_priv, slottime);
}


void wifi_mac_updateslot(struct wifi_mac *wifimac)
{
    wifi_mac_add_work_task(wifimac,wifi_mac_updateslot_ex, NULL,
            (SYS_TYPE)wifimac, 0, 0, 0, 0);
}

void
wifi_mac_update_protmode(struct wifi_mac *wifimac)
{
    enum prot_mode mode = PROT_M_NONE;

    if (wifi_mac_com_has_flag(wifimac, WIFINET_F_USEPROT))
    {
        if (wifimac->wm_protmode == WIFINET_PROT_RTSCTS)
            mode = PROT_M_RTSCTS;
        else if (wifimac->wm_protmode == WIFINET_PROT_CTSONLY)
            mode = PROT_M_CTSONLY;
    }
    wifimac->drv_priv->drv_ops.set_protmode(wifimac->drv_priv, mode);
}

void wifi_mac_set_ampduparams(struct wifi_station *sta)
{
    struct wifi_mac *wifimac = sta->sta_wmac;

    wifimac->drv_priv->drv_ops.set_ampdu_params(wifimac->drv_priv, sta->drv_sta,
        sta->sta_maxampdu, sta->sta_mpdudensity);
}

void wifi_mac_addba_req_setup(struct wifi_station *sta, unsigned char tid_index,
    struct wifi_mac_ba_parameterset *baparamset, unsigned short *batimeout, struct wifi_mac_ba_seqctrl *basequencectrl,
    unsigned short buffersize)
{
    struct wifi_mac *wifimac = sta->sta_wmac;
    wifimac->drv_priv->drv_ops.addba_request_setup(wifimac->drv_priv, sta->drv_sta, tid_index, baparamset,
        batimeout, basequencectrl, buffersize);
}

void wifi_mac_addba_rsp_setup(struct wifi_station *sta, unsigned char tid_index)
{
    struct wifi_mac *wifimac = sta->sta_wmac;

    wifimac->drv_priv->drv_ops.addba_response_setup(wifimac->drv_priv, sta->drv_sta, tid_index);
}


int wifi_mac_addba_req(struct wifi_station *sta, unsigned char dialogtoken,
    struct wifi_mac_ba_parameterset *baparamset, unsigned short batimeout, struct wifi_mac_ba_seqctrl basequencectrl)
{
    struct wifi_mac *wifimac = sta->sta_wmac;
    return wifimac->drv_priv->drv_ops.addba_request_process(wifimac->drv_priv,
        sta->drv_sta, dialogtoken, baparamset, batimeout, basequencectrl);
}

void wifi_mac_addba_rsp(struct wifi_station *sta, unsigned short statuscode,
    struct wifi_mac_ba_parameterset *baparamset, unsigned short batimeout)
{
    struct wifi_mac *wifimac = sta->sta_wmac;
    wifimac->drv_priv->drv_ops.addba_response_process(wifimac->drv_priv,
        sta->drv_sta, statuscode, baparamset, batimeout);
}

void wifi_mac_delba(struct wifi_station *sta, struct wifi_mac_delba_parameterset *delbaparamset,
    unsigned short reasoncode)
{
    struct wifi_mac *wifimac = sta->sta_wmac;
    wifimac->drv_priv->drv_ops.delba_process(wifimac->drv_priv, sta->drv_sta, delbaparamset, reasoncode);
}

void wifi_mac_addba_set_rsp(struct wifi_station *sta, unsigned char tid_index, unsigned short statuscode)
{
    struct wifi_mac *wifimac = sta->sta_wmac;
    wifimac->drv_priv->drv_ops.set_addbaresponse(wifimac->drv_priv, sta->drv_sta, tid_index, statuscode);
}

int wifi_mac_set_country(struct wifi_mac *wifimac, char *isoName)
{
    int error;

    error = wifimac->drv_priv->drv_ops.set_country(wifimac->drv_priv, isoName);
    if (!error)
        wifi_mac_update_country_chan_list(wifimac);
    return error;
}

void wifi_mac_get_country(struct wifi_mac *wifimac, struct _wifi_mac_country_iso *ciso)
{
    wifimac->drv_priv->drv_ops.get_current_country(wifimac->drv_priv, (char *)ciso);
}

int wifi_mac_set_quiet(struct wifi_station *sta, unsigned char *quiet_elm)
{
    struct wifi_mac *wifimac = sta->sta_wmac;
    struct wifi_mac_quiet_ie *quiet = (struct wifi_mac_quiet_ie *)quiet_elm;

    return wifimac->drv_priv->drv_ops.set_quiet(wifimac->drv_priv, quiet->period, quiet->period,
        quiet->offset + quiet->tbttcount * sta->sta_wnet_vif->vm_bcn_intval, 1);
}

static void wifi_mac_update_txpower(void * ieee,
    unsigned short cfg_txpowlimit, unsigned short txpowlevel)
{
    struct wifi_mac *wifimac = NET80211_HANDLE(ieee);
    struct wlan_net_vif *wnet_vif;

    wifimac->wm_txpowlimit = cfg_txpowlimit;

    wifi_mac_enumerate_wnet_vifs(wnet_vif, wifimac)
    {
        struct wifi_station *sta;

        sta = wnet_vif->vm_mainsta;
        ASSERT(sta);
        if (sta)
        {
            wifi_macnsta_set_txpower(sta, txpowlevel);
        }
    }
}

struct wlan_net_vif *
wifi_mac_get_wnet_vif_by_vid(struct wifi_mac *wifimac, int wnet_vif_id)
{
    struct wlan_net_vif *wnet_vif = NULL;

    list_for_each_entry(wnet_vif, &wifimac->wm_wnet_vifs, vm_next)
    {
        if (wnet_vif->wnet_vif_id == wnet_vif_id)
        {
            break;
        }
    }
    return wnet_vif;
}

#if 0
static int16_t
wifi_mac_get_noisefloor(struct wifi_mac *wifimac, struct wifi_channel *chan)
{
    return wifimac->drv_priv->drv_ops.get_noisefloor(wifimac->drv_priv,
            chan->chan_cfreq1,  chan->chan_flags);
}
#endif

void
wifi_mac_config(struct wifi_mac *wifimac, int paramid,int data)
{
    wifimac->drv_priv->drv_ops.drv_set_config_param(wifimac->drv_priv,
            (enum cip_param_id)paramid,data);
}

int wifi_mac_get_config(struct wifi_mac *wifimac, int paramid)
{
    return wifimac->drv_priv->drv_ops.drv_get_config_param(wifimac->drv_priv, (enum cip_param_id)paramid);
}

static void
wifi_mac_notify_qstatus(void * ieee, unsigned short qqcnt)
{
    wifi_mac_notify_queue_status(NET80211_HANDLE(ieee), qqcnt);
}

void wifi_mac_roaming_trigger(struct wlan_net_vif * wnet_vif)
{
    int ret_startscan =0;

    if (wnet_vif->vm_chan_roaming_scan_flag == 1) {
        return;
    }

    if (wnet_vif->vm_wmac->wm_scan->roaming_candidate_chans_cnt < 1) {
        AML_PRINT_LOG_ERR("Can't find roaming candidate channel\n");
        return;
    }

    if (IS_APSTA_CONCURRENT(aml_wifi_get_con_mode()) && !concurrent_check_is_vmac_same_pri_channel(wnet_vif->vm_wmac)) {
        AML_PRINT_LOG_ERR("ap sta concurrent, not trigger roaming\n");
        return;
    }

    wnet_vif->vm_flags &= ~WIFINET_F_DESBSSID;
    wifi_mac_scan_forbidden(wnet_vif, FORBIDDEN_SCAN_FOR_ROAMING_TIMEOUT, FORBIDDEN_SCAN_FOR_ROAMING);
    wnet_vif->vm_chan_roaming_scan_flag = 1;
    wnet_vif->vm_chan_roaming_scan_count = 0;
    wnet_vif->vm_wmac->wm_scan->roaming_full_scan = 0;

    AML_PRINT_LOG_INFO("Trigger Roaming scan start!\n");
    ret_startscan = wifi_mac_start_scan(wnet_vif,WIFINET_SCANCFG_USERREQ|WIFINET_SCANCFG_ACTIVE| WIFINET_SCANCFG_FLUSH|WIFINET_SCANCFG_NOPICK, 1,
         &wnet_vif->vm_wmac->wm_scan->roaming_ssid);

    if (ret_startscan == 0) {
        wnet_vif->vm_chan_roaming_scan_flag = 0;
        wifi_mac_scan_access(wnet_vif);
        AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_INFO, " start scan fail, abort\n");
    }
    else {
        wnet_vif->vm_wmac->wm_roaming_state = ROAMING_SCAN;
    }
}

void wifi_mac_roaming_check(  struct wifi_mac *wifimac, struct wifi_station *sta)
{
    static int roaming_cnt = 0;
    int roaming_threshold = wifimac->roaming_threshold_5g;

    if (WIFINET_IS_CHAN_2GHZ(wifimac->wm_curchan)) {
        roaming_threshold = wifimac->roaming_threshold_2g;
    }

    if (sta->sta_avg_bcn_rssi  < roaming_threshold) {
        if (++roaming_cnt > ROAMING_TRIGER_COUNT && time_before((wifimac->wm_lastroaming + (5 * HZ)), jiffies)) {
            AML_PRINT_LOG_INFO("Rssi[%d] < Threshold[%d]  trigger roaming \n", sta->sta_avg_bcn_rssi, roaming_threshold);
            roaming_cnt = 0;
            wifimac->wm_lastroaming = jiffies;
            wifi_mac_roaming_trigger(sta->sta_wnet_vif);
        }
    }
    else {
        roaming_cnt = 0;
    }
}

int wifi_mac_cal_noise(int rssi,unsigned int snr)
{
    int noise = 0;
    unsigned char noisefigure;

    if (rssi <= -54 ) {
        noisefigure = 5;
    } else if (rssi <= -42) {
        noisefigure = 8;
    } else if (rssi <= -36) {
        noisefigure = 14;
    } else if (rssi <= -30) {
        noisefigure = 19;
    } else if (rssi <= -24) {
        noisefigure = 25;
    } else if (rssi <= -18) {
        noisefigure = 30;
    } else if (rssi <= -12) {
        noisefigure = 36;
    } else {
        noisefigure = 44;
    }

    noise = rssi - snr - noisefigure;

    return noise;
}

static int wifi_mac_average_rssi(int rssi,int i)
{
    int rssi_n = 0;
    int count = 0;
    int rssi_avg = 0;
    static int rssi_value[2][10] = {{0},{0}};
    static int rssi_num = 0;
    int idx = 0;

    idx = i;
    for (count = 0; count < 10; count++) {
        if (rssi_value[idx][count] == 0) {
            rssi_value[idx][count] = rssi;
            rssi_avg = (rssi_avg + rssi_value[idx][count]) / (count + 1);
            return rssi_avg;
        } else {
            rssi_avg += rssi_value[idx][count];
        }
    }

    rssi_avg = 0;
    for (rssi_n = 0; rssi_n < (count - 1); rssi_n++) {
        rssi_value[idx][rssi_n] = rssi_value[idx][rssi_n + 1];
        rssi_avg += rssi_value[idx][rssi_n];
    }

    if (rssi_num < 5) {
        if ((rssi - rssi_value[idx][count - 1]) > 10 || (rssi_value[idx][count - 1] - rssi) > 10) {
                rssi_num++;
        } else {
            rssi_num = 0;
            rssi_value[idx][count - 1] = rssi;
        }
    } else {
        rssi_num = 0;
        rssi_value[idx][count - 1] = rssi;
    }
    rssi_avg = (rssi_avg + rssi_value[idx][count - 1]) / count;

    return rssi_avg;
}
void rx_get_rate_index(struct wifi_mac_rx_status *rs, struct wifi_station *sta)
{
    unsigned char sta_vendor_rate_idx = rs->rs_vendor_rate_code & 0xf;

    if (IS_MCS_RATE(rs->rs_vendor_rate_code)) {
        if (sta->sta_flags & WIFINET_NODE_VHT) {
            if (rs->channel_bw == WIFINET_BWC_WIDTH80) {
                if(rs->rs_sgi) {
                    rs->rs_rate_index = sta_vendor_rate_idx + 92;
                } else {
                    rs->rs_rate_index = sta_vendor_rate_idx + 82;
                }
            } else if (rs->channel_bw == WIFINET_BWC_WIDTH40) {
                if(rs->rs_sgi) {
                    rs->rs_rate_index = sta_vendor_rate_idx + 72;
                } else {
                    rs->rs_rate_index = sta_vendor_rate_idx + 62;
                }
            } else if (rs->channel_bw == WIFINET_BWC_WIDTH20) {
                if(rs->rs_sgi) {
                    rs->rs_rate_index = sta_vendor_rate_idx + 53;
              } else {
                    rs->rs_rate_index = sta_vendor_rate_idx + 44;
              }
            }
        } else if ( sta->sta_flags & WIFINET_NODE_HT) {
            if (rs->channel_bw == WIFINET_BWC_WIDTH40) {
                if(rs->rs_sgi) {
                    rs->rs_rate_index = sta_vendor_rate_idx + 36;
                } else {
                    rs->rs_rate_index = sta_vendor_rate_idx + 28;
                }
            } else if (rs->channel_bw == WIFINET_BWC_WIDTH20) {
                if(rs->rs_sgi) {
                    rs->rs_rate_index = sta_vendor_rate_idx + 20;
                } else {
                    rs->rs_rate_index = sta_vendor_rate_idx + 12;
                }
            }
        } else {
            AML_PRINT_LOG_ERR("can not found rate: 0x%x\n", sta->sta_vendor_rate_code);
        }
    } else {
        rs->rs_rate_index = sta_vendor_rate_idx;
    }
}

int wifi_mac_rx_complete(void *ieee,struct sk_buff *skbbuf, struct wifi_mac_rx_status *rs)
{
    struct wifi_mac *wifimac = NET80211_HANDLE(ieee);
    struct wifi_station *sta;
    struct wlan_net_vif *wnet_vif;
    struct wifi_frame *wh;
    struct wifi_mac_rx_cb *rx_cb;
    int type;
    struct wifi_mac_ops *net_ops;
    unsigned char cur_snr;
    struct drv_private *drv_priv = wifimac->drv_priv;

    int frame_type, frame_subtype;
    static unsigned char recv_beacon_count = 0;

    if (os_skb_get_pktlen(skbbuf) < WIFINET_MIN_LEN)
    {
        AML_PRINT(AML_LOG_ID_RECV, AML_LOG_LEVEL_DEBUG, "short packet %d\n",os_skb_get_pktlen(skbbuf));
        {
            wh = (struct wifi_frame *)os_skb_data(skbbuf);
            frame_type = wh->i_fc[0] & WIFINET_FC0_TYPE_MASK;
            frame_subtype = wh->i_fc[0] & WIFINET_FC0_SUBTYPE_MASK;
            AML_PRINT(AML_LOG_ID_PWR_SAVE,AML_LOG_LEVEL_DEBUG, "len=%d too min, frame_type=0x%x frame_subtype=0x%x\n",
                os_skb_get_pktlen(skbbuf), frame_type, frame_subtype);
        }
        os_skb_free(skbbuf);
        return -1;
    }

    wh = (struct wifi_frame *)os_skb_data(skbbuf);
    frame_type = wh->i_fc[0] & WIFINET_FC0_TYPE_MASK;
    frame_subtype = wh->i_fc[0] & WIFINET_FC0_SUBTYPE_MASK;
    rx_cb = (struct wifi_mac_rx_cb *)os_skb_cb(skbbuf);
    rx_cb->keyid = rs->rs_keyid;
    rx_cb->wnet_vif_id = rs->rs_wnet_vif_id;

    sta = wifi_mac_find_rx_sta(wifimac, (struct wifi_mac_frame_min *)os_skb_data(skbbuf), rs->rs_wnet_vif_id);
    if (sta == NULL) {
        /*sta is null, this is first frame except probe req */
        return wifi_mac_input_all(wifimac, skbbuf, rs);
    }
    sta->cur_fratype = wh->i_fc[0];

    if ((frame_type == WIFINET_FC0_TYPE_MGT) && (frame_subtype == WIFINET_FC0_SUBTYPE_AUTH)
        && !WIFINET_ADDR_EQ(sta->sta_bssid, wh->i_addr3))
    {
        /*sta send auth to other ap, we shall free this sta buffer, if we have allocated. */
        AML_PRINT_LOG_INFO("<running> \n");
        return wifi_mac_input_all(wifimac, skbbuf, rs);
    }
    rx_get_rate_index(rs, sta);
    rs->rs_datarate = drv_priv->drv_currratetable->info[rs->rs_rate_index].rateKbps;
    wnet_vif = sta->sta_wnet_vif;

    if (wnet_vif->vm_wmac->txdesc_flag & TXDESC_FLOW_CTRL_EN) {
        if (wifimac->txdesc_flag & TXDESC_STOP_ALL_QUEUES)
        {
            tasklet_schedule(&drv_priv->ampdu_tasklet);
            tasklet_schedule(&drv_priv->retransmit_tasklet);
        }
        wifi_mac_wake_queues(wnet_vif);
    }

    if (WIFINET_IS_DATA(wh)) {
        if (rs->rs_datarate > 0) {
            sta->sta_last_rxrate = rs->rs_datarate;
            sta->sta_rxrate_index = rs->rs_rate_index;
            sta->last_rxrate_bw = rs->channel_bw;
            sta->last_rxrate_gi = rs->rs_sgi;
        }
    }
    sta->sta_avg_rssi = (sta->sta_avg_rssi + rs->rs_rssi * 3) / 4;
    if (WIFINET_IS_BEACON(wh))
    {
        int bcn_rssi = rs->rs_rssi;
        rs->rs_rssi = wifi_mac_average_rssi(rs->rs_rssi,0);
        sta->sta_avg_bcn_rssi =  (sta->sta_avg_bcn_rssi + (rs->rs_rssi -256) * 3) / 4; //dbm

        if ((wnet_vif->vm_opmode == WIFINET_M_STA) && (wnet_vif->vm_state == WIFINET_S_CONNECTED)) {
            if (!(wifimac->wm_flags & WIFINET_F_SCAN)) {
                recv_beacon_count++;
                if (recv_beacon_count == 20) {
                    recv_beacon_count = 0;
                    if ((sta->sta_avg_bcn_rssi - g_auto_gain_base) > 2 || (g_auto_gain_base - sta->sta_avg_bcn_rssi) > 2
                        || wifimac->force_set_gain) {
                        g_auto_gain_base = sta->sta_avg_bcn_rssi;
                        wifimac->force_set_gain = 0;
                        wifi_mac_set_channel_rssi(wifimac, (unsigned char)(g_auto_gain_base));
                    }
                }

                wifi_mac_rssi_sum(bcn_rssi);

                cur_snr = wifimac->drv_priv->drv_ops.get_snr() / 4;
                if (cur_snr != 0) {
                    sta->sta_avg_snr = (sta->sta_avg_snr * 3 + cur_snr) / 4;
                }
            }

            if (aml_txt_parameter.wftx_pwrtbl_en != 2) {
                wifi_mac_set_tx_power_accord_rssi(wifimac, (unsigned char)(sta->sta_avg_bcn_rssi));
            }
        }
    }else if(WIFINET_IS_DATA(wh)) {
        rs->rs_rssi = wifi_mac_average_rssi(rs->rs_rssi,1);
        sta->sta_avg_data_rssi =  (sta->sta_avg_data_rssi + (rs->rs_rssi -256) * 3) / 4; //dbm
    }

    net_ops = wifimac->drv_priv->net_ops;
    /* AP check triggers */
    net_ops->wifi_mac_ap_chk_uapsd_trig(wifimac->drv_priv->wmac, skbbuf, sta,rs);
    /* STA check more data and eosp */
    net_ops->wifi_mac_sta_check_ps_end(wifimac->drv_priv->wmac, skbbuf, sta, rs);

    /*check if need to roaming*/
    if (wifimac->wm_roaming == WIFINET_ROAMING_BASIC) {
        if ((wnet_vif->vm_opmode == WIFINET_M_STA) && (wnet_vif->vm_state == WIFINET_S_CONNECTED)
            && (sta == wnet_vif->vm_mainsta)){
            wifi_mac_roaming_check(wifimac,sta);
        }
    }

    /*process mpdu in ampdu  or pwr mgt frames*/
    type = wifimac->drv_priv->drv_ops.rx_proc_frame(wifimac->drv_priv, sta->drv_sta, skbbuf, rs);

    return type;
}

static void wifi_mac_free_txamsdu_queue(void * ieee, unsigned char vid)
{
    struct wifi_mac *wifimac = NET80211_HANDLE(ieee);
    wifi_mac_txamsdu_free_all(wifimac, vid);
}

unsigned int wifi_mac_sta_get_txrate(struct wifi_station *sta)
{
    unsigned int rate = 0;

    if (sta != NULL) {
        rate = sta->sta_last_txrate;
    }
    return rate;
}

unsigned int wifi_mac_register_behindTask(struct wifi_mac *wifimac, void *func)
{
    return wifimac->drv_priv->drv_ops.Low_register_behindTask(wifimac->drv_priv,func);
}

unsigned int wifi_mac_call_task(struct wifi_mac *wifimac,
        SYS_TYPE taskid,SYS_TYPE param1)
{
    return wifimac->drv_priv->drv_ops.Low_callRegisteredTask(wifimac->drv_priv, taskid,param1);
}

unsigned int wifi_mac_add_work_task(struct wifi_mac *wifimac,void *func,
    void *func_cb,SYS_TYPE param1, SYS_TYPE param2,SYS_TYPE param3,
    SYS_TYPE param4,SYS_TYPE param5)
{
    return wifimac->drv_priv->drv_ops.Low_addDHWorkTask(wifimac->drv_priv,
            func, func_cb, param1, param2, param3, param4, param5);
}

struct hal_work_task * wifi_mac_search_work_task(struct wifi_mac *wifimac,void *func,
    void *func_cb, SYS_TYPE param1, SYS_TYPE param2,SYS_TYPE param3,SYS_TYPE param4,SYS_TYPE param5)
{
    return drv_hal_search_workitem(func, func_cb, param1, param2, param3, param4, param5);
}


void wifi_mac_init_ops (struct wifi_mac* wmac)
{
    wmac->wmac_ops.wifi_mac_mac_entry = wifi_mac_entry;
    wmac->wmac_ops.wifi_mac_setup_channel_list = wifi_mac_chan_setup;
    wmac->wmac_ops.wifi_mac_update_txpower = wifi_mac_update_txpower;
    wmac->wmac_ops.wifi_mac_update_beacon = wifi_mac_update_beacon;
    wmac->wmac_ops.wifi_mac_notify_qstatus = wifi_mac_notify_qstatus;
    wmac->wmac_ops.wifi_mac_tx_complete = wifi_mac_tx_complete;
    wmac->wmac_ops.wifi_mac_tx_status = wifi_mac_tx_status;
    wmac->wmac_ops.wifi_mac_rx_complete = wifi_mac_rx_complete;
    wmac->wmac_ops.wifi_mac_input = wifi_mac_input;
    wmac->wmac_ops.wifi_mac_ChangeChannel = wifi_mac_ChangeChannel;
    wmac->wmac_ops.wifi_mac_rate_init = wifi_mac_rate_init;
    wmac->wmac_ops.wifi_mac_rate_nsta_update = wifi_mac_rate_nsta_update;
    wmac->wmac_ops.wifi_mac_free_txamsdu_queue = wifi_mac_free_txamsdu_queue;
    wmac->wmac_ops.wifi_mac_ap_chk_uapsd_trig = wifi_mac_pwrsave_chk_uapsd_trig;
    wmac->wmac_ops.wifi_mac_uapsd_eosp_indicate = wifi_mac_pwrsave_eosp_indicate;

    wmac->wmac_ops.wifi_mac_sta_check_ps_end = wifi_mac_pwrsave_check_ps_end;
    wmac->wmac_ops.wifi_mac_rate_ratmod_attach = wifi_mac_rate_ratmod_attach;
    wmac->wmac_ops.wifi_mac_rate_ratmod_detach = wifi_mac_rate_ratmod_detach;

    wmac->wmac_ops.wifi_mac_create_vmac = wifi_mac_create_vmac;
    wmac->wmac_ops.wifi_mac_device_ip_config = wifi_mac_device_ip_config;
    wmac->wmac_ops.wifi_mac_tbtt_handle = wifi_mac_tbtt_handle;
    wmac->wmac_ops.wifi_mac_channel_switch_complete = wifi_mac_channel_switch_complete;
    wmac->wmac_ops.wifi_mac_get_netif_cfg = wifi_mac_get_netif_cfg;
    wmac->wmac_ops.wifi_mac_mac_exit = wifi_mac_mac_exit;
    wmac->wmac_ops.wifi_mac_top_sm = wifi_mac_top_sm;
    wmac->wmac_ops.os_skb_get_nsta = os_skb_get_nsta;
    wmac->wmac_ops.os_skb_get_tid = os_skb_get_tid;
    wmac->wmac_ops.wifi_mac_notify_mic_fail = wifi_mac_notify_mic_fail;
    wmac->wmac_ops.wifi_mac_send_qosnulldata = wifi_mac_send_qosnulldata;

  //  wmac->wmac_ops.vm_p2p_client_cancel_noa = vm_p2p_client_cancel_noa,
  //  wmac->wmac_ops.vm_p2p_go_cancel_noa = vm_p2p_go_cancel_noa,

    wmac->wmac_ops.wifi_mac_pwrsave_fullsleep = wifi_mac_pwrsave_fullsleep;
    wmac->wmac_ops.wifi_mac_pwrsave_is_sta_fullsleep = wifi_mac_pwrsave_is_sta_fullsleep;
    wmac->wmac_ops.wifi_mac_pwrsave_is_sta_sleeping = wifi_mac_pwrsave_is_sta_sleeping;
    wmac->wmac_ops.wifi_mac_pwrsave_is_wnet_vif_fullsleep = wifi_mac_pwrsave_is_wnet_vif_fullsleep;
    wmac->wmac_ops.wifi_mac_pwrsave_is_wnet_vif_sleeping = wifi_mac_pwrsave_is_wnet_vif_sleeping;
    wmac->wmac_ops.wifi_mac_pwrsave_networksleep = wifi_mac_pwrsave_networksleep;
    wmac->wmac_ops.wifi_mac_pwrsave_psqueue_enqueue = wifi_mac_pwrsave_psqueue_enqueue;
    wmac->wmac_ops.wifi_mac_pwrsave_psqueue_flush = wifi_mac_pwrsave_psqueue_flush;
    wmac->wmac_ops.wifi_mac_pwrsave_sta_trigger = wifi_mac_pwrsave_sta_trigger;
    wmac->wmac_ops.wifi_mac_buffer_txq_enqueue = wifi_mac_buffer_txq_enqueue;
    wmac->wmac_ops.wifi_mac_buffer_txq_send_pre = wifi_mac_buffer_txq_send_pre;
    wmac->wmac_ops.wifi_mac_pwrsave_wakeup_for_tx = wifi_mac_pwrsave_wakeup_for_tx;
    wmac->wmac_ops.wifi_mac_pwrsave_wkup_and_NtfyAp = wifi_mac_pwrsave_wkup_and_NtfyAp;
    wmac->wmac_ops.wifi_mac_notify_ap_success = wifi_mac_notify_ap_success;

    wmac->wmac_ops.wifi_mac_get_sta = wifi_mac_get_sta;
    wmac->wmac_ops.wifi_mac_sta_attach = wifi_mac_sta_attach;

    wmac->wmac_ops.wifi_mac_beacon_miss = wifi_mac_process_beacon_miss;
    wmac->wmac_ops.wifi_mac_get_sts = wifi_mac_get_sts;
    wmac->wmac_ops.wifi_mac_process_tx_error = wifi_mac_process_tx_error;
    wmac->wmac_ops.wifi_mac_forward_txq_enqueue = wifi_mac_forward_txq_enqueue;

    wmac->wmac_ops.wifi_mac_recovery_record_vif = wifi_mac_recovery_record_vif;
    wmac->wmac_ops.wifi_mac_recovery_host_reset = wifi_mac_recovery_host_reset;
    wmac->wmac_ops.wifi_mac_vif_restore_end = wifi_mac_vif_restore_end;
    wmac->wmac_ops.wifi_mac_recovery_host_restore = wifi_mac_recovery_host_restore;
    wmac->wmac_ops.wifi_mac_process_recovery = wifi_mac_process_recovery;

}


void wifi_mac_set_txpower_limit(struct wifi_mac *wifimac, unsigned int limit,
                                unsigned short txpowerdb)
{
    wifimac->drv_priv->drv_ops.drv_set_txPwrLimit(wifimac->drv_priv, limit, txpowerdb);
}


int wifi_mac_get_netif_cfg(void * ieee)
{
    struct wifi_mac *wifimac = NET80211_HANDLE(ieee);

    return wifimac->wm_dev_flags;
}

int wifi_mac_netdev_open(void * ieee)
{
    struct wifi_mac *wifimac = NET80211_HANDLE(ieee);
    struct drv_private *drv_priv = wifimac->drv_priv;
    int error =0;

    /*call ha layer funct to set fpga info, init rf and set channel. */
    error = wifimac->drv_priv->drv_ops.drv_open(wifimac->drv_priv);
    if (error) {
        AML_PRINT_LOG_ERR("err= %d \n",error);
        return error;
    }

    wifimac->wm_dev_flags |= NETCOM_NETIF_IFF_UP_RUNNING;
    if (wifimac->wm_curchan != WIFINET_CHAN_ERR && wifimac->wm_curchanmaxpwr == 0)
        wifimac->wm_curchanmaxpwr = wifimac->wm_curchan->chan_maxpower;

    AML_PRINT_LOG_INFO("\n");

    chan_dbg(wifimac->wm_curchan, "X8",1332);
    if (wifimac->drv_priv->drv_config.cfg_txamsdu) {
        os_timer_ex_start_period(&wifimac->drv_priv->drv_txtimer, NET80211_AMSDU_TIMEOUT);
    }

    if (drv_priv->drv_config.cfg_recovery)
        os_timer_ex_start(&wifimac->wm_monitor_fw);
    os_timer_ex_start(&wifimac->wm_tp_rate);

    if (hal_ant_sel_en_get() == 2)
    {
        os_timer_ex_start(&wifimac->wm_ant_select);
    }

    return error;
}

const struct net_device_ops wifi_mac_netdev_ops =
{
    .ndo_init = wifi_mac_init,
    .ndo_uninit = wifi_mac_uninit,
    .ndo_get_stats = wifi_mac_getstats,
    .ndo_open = wifi_mac_open, // ifconfig up
    .ndo_stop = wifi_mac_stop, // if config down
    .ndo_start_xmit = wifi_mac_hardstart, // TCP/IP TX
    //netif_rx(skb);
    .ndo_change_mtu = wifi_mac_change_mtu,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
    .ndo_siocdevprivate     = wifi_mac_ioctrl_wrapper,
#else
    .ndo_do_ioctl           = wifi_mac_ioctrl,
#endif
    .ndo_set_mac_address = wifi_set_mac_address,
};

#define AMLWL_IPV4    1
#define AMLWL_IPV6    2

int aml_notify_ip(struct wifi_station *sta, u8 vif_ctx_id, u8 ip_type, u8 *ip_addr)
{
    if (ip_type != AMLWL_IPV4 && ip_type != AMLWL_IPV6)
        return -EINVAL;

    if (ip_type == AMLWL_IPV4) {
        memcpy(sta->sta_ap_ip, ip_addr, IPV4_LEN);
        sta->connect_status = CONNECT_DHCP_GET_ACK;
        sta->ip_acquired |= ip_type;
        sta->sta_arp_flag |= WIFI_ARP_GET_AP_IP;
        wifi_mac_scan_access(sta->sta_wnet_vif);

    } else {
        memcpy(sta->sta_ap_ipv6, ip_addr, IPV6_LEN);
    }
    return 0;
}


static int aml_inetaddr_event(struct notifier_block *this, unsigned long event, void *ptr)
{
    struct net_device *ndev;
    struct wlan_net_vif *wnet_vif = NULL;
    struct wifi_station *sta;
    struct wifi_station_tbl *nt = NULL;
    struct in_ifaddr *ifa = (struct in_ifaddr *)ptr;
    unsigned char *tmp = NULL;

    if (!ifa || !(ifa->ifa_dev->dev)) {
        return NOTIFY_DONE;
    }
    if (ifa->ifa_dev->dev->netdev_ops != &wifi_mac_netdev_ops) {
        return NOTIFY_DONE;
    }

    ndev = ifa->ifa_dev->dev;
    wnet_vif = netdev_priv(ndev);
    nt = &(wnet_vif->vm_sta_tbl);

    if (event == NETDEV_UP && wnet_vif->vm_state == WIFINET_S_CONNECTED) {
        struct wireless_dev *pwdev = wnet_vif->vm_wdev;
        struct vm_wdev_priv *pwdev_priv = wdev_to_priv(pwdev);

        sta = wifi_mac_get_sta(nt, wnet_vif->vm_des_bssid, wnet_vif->wnet_vif_id);

        if (sta != NULL) {
            if (sta->wnet_vif_id != wnet_vif->wnet_vif_id) {
                AML_PRINT_LOG_ERR("vid:%d mismatch:%d\n", sta->wnet_vif_id, wnet_vif->wnet_vif_id);
            }

        } else {
            AML_PRINT_LOG_ERR("failed to find sta\n");
            return NOTIFY_DONE;
        }
        tmp = (unsigned char *)(&ifa->ifa_address);
        AML_PRINT_LOG_INFO("[%s]get ip:%02d.%02d.%02d.%02d\n", ndev->name, tmp[0],tmp[1], tmp[2],tmp[3]);
        if (sta->sta_wnet_vif->vm_p2p->p2p_enable) {
            set_p2p_negotiation_status(sta, NET80211_P2P_STATE_GO_COMPLETE);
        }
        os_timer_ex_cancel(&pwdev_priv->connect_timeout, CANCEL_SLEEP);
        aml_notify_ip(sta, wnet_vif->wnet_vif_id, AMLWL_IPV4, (u8 *)&ifa->ifa_address);
    } else if (event == NETDEV_UP && wnet_vif->vm_state != WIFINET_S_CONNECTED) {
        wnet_vif->vm_use_static_ip = 1;
        tmp = (unsigned char *)(&ifa->ifa_address);
        AML_PRINT_LOG_INFO("config static IPv4 for %s: %02d.%02d.%02d.%02d", ndev->name, tmp[0],tmp[1], tmp[2],tmp[3]);
        memcpy(wnet_vif->vm_static_ipv4, &ifa->ifa_address, IPV4_LEN);
    }
    return NOTIFY_DONE;
}



void wifi_mac_set_ipv6_addr(SYS_TYPE param1, SYS_TYPE param2,SYS_TYPE param3, SYS_TYPE param4,SYS_TYPE param5)
{
    struct wifi_station *sta = (struct wifi_station *)param1;
    unsigned char vif_id = (unsigned char)param2;
    unsigned char type = (unsigned char)param3;
    unsigned char *ipv6_addr = (unsigned char *)param4;

    aml_notify_ip(sta, vif_id, type, ipv6_addr);
    return;
}

static int aml_inet6addr_event(struct notifier_block *this, unsigned long event, void *ptr)
{
    struct net_device *ndev;
    struct wlan_net_vif *wnet_vif = NULL;
    struct wifi_station *sta;
    struct wifi_station_tbl *nt = NULL;
    struct inet6_ifaddr *inet6_ifa = (struct inet6_ifaddr *)ptr;
    unsigned char *tmp = NULL;

    if (!inet6_ifa || !(inet6_ifa->idev->dev)) {
        return NOTIFY_DONE;
    }
    if (inet6_ifa->idev->dev->netdev_ops != &wifi_mac_netdev_ops) {
        return NOTIFY_DONE;
    }

    ndev = inet6_ifa->idev->dev;
    wnet_vif = netdev_priv(ndev);
    nt = &(wnet_vif->vm_sta_tbl);

    if (event == NETDEV_UP && wnet_vif->vm_state == WIFINET_S_CONNECTED) {
        sta = wifi_mac_get_sta(nt, wnet_vif->vm_des_bssid, wnet_vif->wnet_vif_id);

        if (sta != NULL) {
            if (sta->wnet_vif_id != wnet_vif->wnet_vif_id) {
                AML_PRINT_LOG_ERR("vid:%d mismatch:%d\n", sta->wnet_vif_id, wnet_vif->wnet_vif_id);
            }

        } else {
            AML_PRINT_LOG_ERR("failed to find sta\n");
            return NOTIFY_DONE;
        }
        tmp = (unsigned char *)(inet6_ifa->addr.in6_u.u6_addr8);
        AML_PRINT_LOG_INFO("get ipv6:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
            tmp[0],tmp[1], tmp[2],tmp[3], tmp[4],tmp[5], tmp[6],tmp[7], tmp[8],tmp[9], tmp[10],tmp[11], tmp[12],tmp[13], tmp[14],tmp[15]);

        wifi_mac_add_work_task(sta->sta_wmac, wifi_mac_set_ipv6_addr,
            NULL,(SYS_TYPE)sta, (SYS_TYPE)wnet_vif->wnet_vif_id, AMLWL_IPV6, (SYS_TYPE)&inet6_ifa->addr, 0);
    }

    return NOTIFY_DONE;
}

struct notifier_block aml_inetaddr_cb = {
    .notifier_call = aml_inetaddr_event
};

struct notifier_block aml_inet6addr_cb = {
    .notifier_call = aml_inet6addr_event
};

int wifi_mac_netdev_stop(void * ieee)
{
    struct wifi_mac *wifimac = NET80211_HANDLE(ieee);
    int ret;

    AML_PRINT_LOG_INFO("<running>++\n");
    if (NET80211_IF_RUNNING(wifimac)==0)
        return 0 ;

    wifimac->wm_dev_flags &= ~NETCOM_NETIF_IFF_RUNNING;
    ret = wifimac->drv_priv->drv_ops.drv_stop(wifimac->drv_priv);
    AML_PRINT_LOG_INFO("<running> wm_dev_flags  = %x \n",
        wifimac->wm_dev_flags );
    if (wifimac->drv_priv->drv_config.cfg_txamsdu) {
        os_timer_ex_cancel(&wifimac->drv_priv->drv_txtimer, CANCEL_SLEEP);
    }

    os_timer_ex_cancel(&wifimac->wm_monitor_fw, CANCEL_SLEEP);
    os_timer_ex_cancel(&wifimac->wm_tp_rate, CANCEL_SLEEP);

    if (hal_ant_sel_en_get() == 2) {
        os_timer_ex_cancel(&wifimac->wm_ant_select, CANCEL_SLEEP);
    }
    if (hal_ant_sel_en_get() != 0) {
        os_timer_ex_cancel(&wifimac->wm_ant_rssi_measure, CANCEL_SLEEP);
    }

    return ret;
}
/* For some platform using backport like ROKU, hrtimer_start is differently defined */
#ifndef BACKPORTS_VERSION
static ktime_t lock_kt;
#endif
static struct hrtimer hr_lock_timer;
int g_hr_lock_timer_valid = 0;

static enum hrtimer_restart wifi_mac_set_tx_lock_timeout(struct hrtimer *timer)
{
    struct drv_private *drv_priv = drv_get_drv_priv();

    if (drv_priv != NULL) {
        drv_priv->wait_mpdu_timeout = 1;
        tasklet_schedule(&drv_priv->ampdu_tasklet);
    }

    g_hr_lock_timer_valid = 0;
    return HRTIMER_NORESTART;
}

void wifi_mac_tx_lock_timer_attach(void)
{
    struct drv_private *drv_priv = drv_get_drv_priv();

    hrtimer_init(&hr_lock_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    hr_lock_timer.function = wifi_mac_set_tx_lock_timeout;

    /* For some platform using backport like ROKU, hrtimer_start is differently defined */
#ifdef BACKPORTS_VERSION
    hrtimer_start(&hr_lock_timer, 1500000 * drv_priv->drv_config.cfg_hrtimer_interval, HRTIMER_MODE_REL);
#else
    lock_kt = ktime_set(0, 1500000 * drv_priv->drv_config.cfg_hrtimer_interval);
    hrtimer_start(&hr_lock_timer, lock_kt, HRTIMER_MODE_REL);
#endif
    g_hr_lock_timer_valid = 1;
}

void wifi_mac_tx_lock_timer_cancel(void)
{
    if (g_hr_lock_timer_valid) {
        hrtimer_cancel(&hr_lock_timer);
        g_hr_lock_timer_valid = 0;
    }
}

void wifi_mac_set_fwlog_mode(unsigned char mode, struct wifi_mac *wifimac)
{
    if (mode == FWLOG_DISABLE) {
        //do nothing
    }
    else if (mode == FWLOG_BY_FILE) {
        wifimac->drv_priv->hal_priv->g_get_fw_log = true;
        wifimac->drv_priv->hal_priv->hal_ops.hal_set_fwlog_cmd(1);
        wifimac->drv_priv->hal_priv->hal_ops.hal_set_fwlog_cmd(4);
    }
    else if (mode == FWLOG_BY_UART) {
        //wifimac->drv_priv->hal_priv->hal_ops.hal_write_word(0x00f00004, 0x0ffbf0ff);
        //wifimac->drv_priv->hal_priv->hal_ops.hal_write_word(0x00f00008, 0x00040f00);
        //wifimac->drv_priv->hal_priv->hal_ops.hal_write_word(0x00f00020, 0x00000001);
    }
    AML_PRINT_LOG_INFO("mode:%d\n", mode);
}

int wifi_mac_cap_attach(struct wifi_mac *wifimac, struct drv_private* drv_priv)
{
    struct driver_ops  *ops = NULL;
    int vid = 0;

    wifimac->wm_mac_exit =0;
    wifimac->drv_priv = drv_priv;

    wifimac->wm_wnet_vif_mask = 0;
    wifimac->wm_caps = 0;
#if defined(SU_BF) || defined(MU_BF)
    wifimac->max_spatial = 2;
#endif

    ops = &drv_priv->drv_ops;
    wifi_mac_amsdu_attach(wifimac);
    // wifi_mac_com_clear_cap(wifimac, -1);

    wifi_mac_ComSetCap(wifimac, WIFINET_C_IBSS | WIFINET_C_HOSTAP | WIFINET_C_SHPREAMBLE
        | WIFINET_C_SHSLOT | WIFINET_C_WPA | WIFINET_C_BGSCAN);


    if (ops->drv_get_config_param(drv_priv, CHIP_PARAM_WMM))
        wifi_mac_ComSetCap(wifimac, WIFINET_C_WME);

    wifimac->wm_ac2q[WME_AC_BE] = HAL_WME_AC_BE;
    wifimac->wm_ac2q[WME_AC_BK] = HAL_WME_AC_BK;
    wifimac->wm_ac2q[WME_AC_VI ] = HAL_WME_AC_VI;
    wifimac->wm_ac2q[WME_AC_VO] = HAL_WME_AC_VO;

    wifimac->wm_mng_qid = HAL_WME_MGT;
    wifimac->wm_noqos_legacy_qid = HAL_WME_NOQOS;
    wifimac->wm_uapsd_qid = HAL_WME_UAPSD;

    /*get capabilities from 'drv_priv->drv_config'
      initiallize in aml_driv_attach()*/
    if (ops->drv_get_config_param(drv_priv, CHIP_PARAM_UAPSD))
    {
        wifi_mac_ComSetCap(wifimac, WIFINET_C_UAPSD);
        //WIFINET_COM_UAPSD_ENABLE(wifimac);
    }

    wifimac->wm_mcast_qnum = HAL_WME_MCAST;

    if (ops->drv_get_config_param(drv_priv, CHIP_PARAM_CIPHER_WEP))
        wifi_mac_ComSetCap(wifimac, WIFINET_C_WEP);
    if (ops->drv_get_config_param(drv_priv, CHIP_PARAM_CIPHER_WPI))
        wifi_mac_ComSetCap(wifimac, WIFINET_C_WAPI);
    if (ops->drv_get_config_param(drv_priv, CHIP_PARAM_CIPHER_AES_CCM))
    {
        wifi_mac_ComSetCap(wifimac, WIFINET_C_AES);
        wifi_mac_ComSetCap(wifimac, WIFINET_C_AES_CCM);
    }
    if (ops->drv_get_config_param(drv_priv, CHIP_PARAM_CIPHER_TKIP))
        wifi_mac_ComSetCap(wifimac, WIFINET_C_TKIP);

    if (ops->drv_get_config_param(drv_priv, CHIP_PARAM_HWTKIPMIC))
        wifi_mac_ComSetCap(wifimac, WIFINET_C_TKIPMIC);

    if (ops->drv_get_config_param(drv_priv, CHIP_PARAM_HWCSUM))
    {
        wifi_mac_ComSetCap(wifimac, WIFINET_C_HWCSUM);
    }
    if (ops->drv_get_config_param(drv_priv, CHIP_PARAM_BWAUTOCONTROL))
    {
        wifi_mac_ComSetCap(wifimac, WIFINET_C_BWCTRL);
    }

    wifi_mac_com_clear_flag(wifimac, WIFINET_F_DOTH);

    if (ops->drv_get_config_param(drv_priv, CHIP_PARAM_HT))
    {
        wifi_mac_ComSetCap(wifimac, WIFINET_C_HT);

        if (ops->drv_get_config_param(drv_priv, CHIP_PARAM_RDG))
        {
            wifi_mac_ComSetCap(wifimac, WIFINET_C_RDG);
        }
        wifi_mac_com_set_flag(wifimac, WIFINET_F_HT);
        wifi_mac_com_set_maxampdu(wifimac, WIFINET_HTCAP_MAXRXAMPDU_32768);
        wifi_mac_com_set_mpdudensity(wifimac, WIFINET_HTCAP_MPDUDENSITY_8);
        wifi_mac_com_set_extflag(wifimac, WIFINET_FEXT_AMPDU);
        wifi_mac_com_set_extflag(wifimac, WIFINET_FEXT_SHORTGI_ENABLE);
    }


    if (ops->drv_get_config_param(drv_priv, CHIP_PARAM_VHT))
    {
        if (ops->drv_get_config_param(drv_priv, CHIP_PARAM_RDG))
        {
            wifi_mac_ComSetCap(wifimac, WIFINET_C_RDG);
        }

        wifi_mac_com_set_flag(wifimac, WIFINET_F_HT);
        wifi_mac_com_set_maxampdu(wifimac, WIFINET_HTCAP_MAXRXAMPDU_32768);
        wifi_mac_com_set_mpdudensity(wifimac, WIFINET_HTCAP_MPDUDENSITY_8);
        wifi_mac_com_set_extflag(wifimac, WIFINET_FEXT_AMPDU);
    }

    /*set burst flag, burst frames in txop. */
    if (ops->drv_get_config_param(drv_priv, CHIP_PARAM_BURST_ACK))
    {
        wifi_mac_ComSetCap(wifimac, WIFINET_C_BURST);
    }

    if (ops->drv_get_config_param(drv_priv, CHIP_PARAM_MFP)) {
        wifi_mac_ComSetCap(wifimac, WIFINET_C_MFP);
    }

    memset(wifimac->wm_country.iso, 0, 3);

    /*set channel num and set current use channel by country code.*/
    wifi_mac_chan_attach(wifimac);
    wifi_mac_chan_overlapping_map_init(wifimac);

    if (wifimac->wm_caps & WIFINET_C_WME)
        wifimac->wm_flags |= WIFINET_F_WME;

    wifi_mac_reset_erp(wifimac, WIFINET_MODE_11BGN);
    wifi_mac_reset_ht(wifimac);

    wifimac->wm_suspend_mode = WIFI_SUSPEND_STATE_NONE;
    init_waitqueue_head(&wifimac->wm_suspend_wq);

    WIFINET_LOCK_INIT(wifimac, "wifi_mac");
    WIFINET_PSLOCK_INIT(wifimac, "wifi_mac_ps");
    WIFINET_QLOCK_INIT(wifimac, " wifi_mac_queue ");
    WIFINET_BEACONLOCK_INIT(wifimac, "wifi_mac_beacon");
    WIFINET_BEACONBUFLOCK_INIT(wifimac, "wifi_mac_beaconbuf");
    WIFINET_VMACS_LOCK_INIT(wifimac, "wlan_net_vif");

    /*init vmac list. */
    INIT_LIST_HEAD(&wifimac->wm_wnet_vifs);
    INIT_LIST_HEAD(&wifimac->wm_free_entryq);
    WIFINET_NODE_FREE_LOCK_INIT(wifimac);

    wifimac->scan_gain_thresh_unconnect = UNCONNECT_MIN_GIAN_THRESHOLD;
    wifimac->scan_gain_thresh_connect = CONNECT_MIN_GIAN_THRESHOLD;
    wifimac->scan_max_gain_thresh = MAX_GIAN_THRESHOLD;
    wifimac->scan_noisy_status = WIFINET_S_SCAN_ENV_CLEAR;
    wifimac->is_connect_set_gain = 1;
    wifimac->force_set_gain = 0;
    wifimac->scan_available = 1;

    wifimac->wm_txpowlimit = WIFINET_TXPOWER_MAX;
    wifimac->wm_rx_streams = WIFINET_RXSTREAM;
    wifimac->wm_tx_streams = WIFINET_TXSTREAM;
    wifimac->wm_flags_ext |= WIFINET_C_WDS_AUTODETECT;
    wifimac->wm_flags_ext |= WIFINET_FEXT_COUNTRYIE;
    /*protect type, no protect, rts/cts or cts only */
    wifimac->wm_protmode = WIFINET_PROT_RTSCTS;
    for(vid = 0; vid < DEFAULT_MAX_VMAC; vid++)
    {
        wifimac->wm_wme[vid].wme_hipri_switch_hysteresis = 3;
    }
    wifi_mac_pwrsave_attach();
    /*prepare encryption cipher and de/encrypt handler */
    wifi_mac_security_attach(wifimac);
    /* initialize inact_timer for ap&sta, sta will keep alive
    to ap (send nulldata to ap), ap will monitor if sta leave bss.*/

    /*initialize scan param and scan_timer */
    wifi_mac_scan_attach(wifimac);
    wifimac->wm_esco_en    = 0;
    wifimac->wm_bt_en = 0;

#ifdef CONFIG_CONCURRENT_MODE
    wifimac->wm_vsdb_slot = CONCURRENT_SLOT_NONE;
    wifimac->wm_vsdb_flags = 0;
    wifimac->wm_vsdb_switch_time = 0;
    wifimac->vsdb_mode_set_noa_enable = 0;

    wifimac->wm_p2p_home_channel = 0;

    /*init a concurrent timer for vmac channel concurrent function. */
    os_timer_ex_initialize(&wifimac->wm_concurrenttimer, COMCURRENT_CHANNCHANGE_TIME,
        concurrent_change_channel_timeout, wifimac);
#endif
    wifimac->wm_recovery_src = 0;
    wifimac->wm_recovery_level = WIFINET_RECOVERY_L_NORMAL;
    wifimac->wm_recovery_flags = 0;
#ifdef CHIP_RESET_SUPPORT
    wifimac->request_upper_recovery = 0;
    AML_PRINT_LOG_INFO("current plateform will not recovery by driver self, need to report uppper layer");
#endif
    os_timer_ex_initialize(&wifimac->wm_monitor_fw, 5000, wifi_mac_trigger_recovery, wifimac);
    os_timer_ex_initialize(&wifimac->wm_tp_rate, 1000, wifi_mac_monitor_tp_rate, wifimac);
    wifi_mac_set_fwlog_mode(get_fwlog_mode(), wifimac);

    wifimac->wm_alpha_set_forbid = 0;
    wifimac->wm_alpha_set_in_progress = 0;
    wifimac->wm_alpha_pending = 0;
    memset(wifimac->wm_alpha_target, 0x00, sizeof(wifimac->wm_alpha_target)/sizeof(wifimac->wm_alpha_target[0]));
    spin_lock_init(&wifimac->alpha_lock);

    if (hal_ant_sel_en_get() == 2) {
        os_timer_ex_initialize(&wifimac->wm_ant_select, ANT_SEL_MEAS_PERIOD, wifi_mac_ant_select, wifimac);
    }
    if (hal_ant_sel_en_get() != 0) {
        os_timer_ex_initialize(&wifimac->wm_ant_rssi_measure, ANT_SEL_MEAS_DURATION, wifi_mac_ant_rssi_measure, wifimac);
    }

    if (aml_bus_type == AML_BUS_TYPE_USB)
    {
        wifimac->txdesc_flag |= TXDESC_FLOW_CTRL_EN;
    }
    AML_PRINT_LOG_INFO("<running>\n");

    return 0;
}

int wifi_mac_cap_detach(struct wifi_mac *wifimac)
{
    struct wlan_net_vif *wnet_vif;
    wifi_mac_stop_running(wifimac);

    AML_PRINT_LOG_INFO("<running>\n");

    list_for_each_entry(wnet_vif, &wifimac->wm_wnet_vifs, vm_next) {
        wifi_mac_vmac_detach(wnet_vif);
    }

    wifi_mac_scan_detach(wifimac);
    wifi_mac_security_detach(wifimac);
    wifi_mac_pwrsave_detach();
    wifi_mac_StationDetach(wifimac);

    while (!list_empty(&wifimac->wm_wnet_vifs)) {
        wnet_vif = list_first_entry(&wifimac->wm_wnet_vifs,
                struct wlan_net_vif, vm_next);
        vm_wlan_net_vif_unregister(wnet_vif);
    }

    WIFINET_VMACS_LOCK_DESTROY(wifimac);
    WIFINET_LOCK_DESTROY(wifimac);

    wifi_mac_acl_deattach();
    wifi_mac_amsdu_detach(wifimac);

#ifdef CONFIG_CONCURRENT_MODE
    os_timer_ex_del(&wifimac->wm_concurrenttimer, CANCEL_SLEEP);
#endif

    wifi_mac_tx_lock_timer_cancel();
    os_timer_ex_del(&wifimac->wm_monitor_fw, CANCEL_SLEEP);
    os_timer_ex_del(&wifimac->wm_tp_rate, CANCEL_SLEEP);

    if (hal_ant_sel_en_get() != 0) {
        if (hal_ant_sel_en_get() == 2) {
            os_timer_ex_del(&wifimac->wm_ant_select, CANCEL_SLEEP);
        }
        os_timer_ex_del(&wifimac->wm_ant_rssi_measure, CANCEL_SLEEP);
    }

    AML_PRINT_LOG_INFO("<running>\n");
    wifimac->drv_priv = NULL;

    return 0;
}


unsigned char wifi_mac_tx_init(struct wifi_mac *wifimac) {
    struct list_head *head = &wifimac->txdesc_freequeue;
    struct drv_txdesc *ptxdesc;
    int i, bsize;

    TX_DESC_BUF_LOCK_INIT(wifimac);

    /* Setup tx descriptors */
    bsize = sizeof(struct drv_txdesc) * DRV_TXDESC_NUM;
#ifdef CONFIG_AML_USE_STATIC_BUF
    ptxdesc = wifi_mem_prealloc(AML_TX_DESC_BUF, bsize);
    memset(ptxdesc, 0, bsize);
#else
    ptxdesc = (struct drv_txdesc *)NET_MALLOC(bsize, GFP_KERNEL, "drv_tx_init.ptxdesc");
#endif
    if (ptxdesc == NULL)
    {
        return -ENOMEM;
    }
    wifimac->txdesc_bufptr = ptxdesc;

    INIT_LIST_HEAD(head);
    for (i = 0; i < DRV_TXDESC_NUM; i++, ptxdesc++)
    {
        list_add_tail(&ptxdesc->txdesc_queue, head);
        wifimac->txdesc_free_cnt++;
    }

    return wifimac->drv_priv->drv_ops.tx_init(wifimac->drv_priv, DRV_TXDESC_NUM);
}

void wifi_mac_tx_cleanup(struct wifi_mac *wifimac) {
    struct list_head *head = &wifimac->txdesc_freequeue;

    wifimac->drv_priv->drv_ops.tx_cleanup(wifimac->drv_priv);

    /* cleanup tx descriptors */
    if (wifimac->txdesc_bufptr != NULL)
    {
        INIT_LIST_HEAD(head);
#ifndef CONFIG_AML_USE_STATIC_BUF
        NET_FREE(wifimac->txdesc_bufptr, "drv_tx_init.ptxdesc");
#endif
        wifimac->txdesc_bufptr = NULL;
    }

    return;
}

int wifi_mac_entry(struct wifi_mac *wifimac, void *drv_priv)
{
    int error = 0;

    AML_PRINT_LOG_INFO("<running>++ \n");

    error = wifi_mac_cap_attach(wifimac,(struct drv_private*)drv_priv);
    if (error != 0)
        goto bad;

    error = wifi_mac_tx_init(wifimac);
    if (error != 0)
        goto bad1;

    error = wifimac->drv_priv->drv_ops.rx_init(drv_priv, DRV_RXDESC_NUM);
    if (error != 0)
        goto bad2;

    return 0;

bad2:
    wifi_mac_tx_cleanup(wifimac);
bad1:
    wifi_mac_cap_detach(wifimac);
bad:
    AML_PRINT_LOG_INFO("bad!\n");
    return error;
}


 void wifi_mac_get_sts(struct wifi_mac *wifimac,unsigned int op_code, unsigned int ctrl_code)
{
    int i ;
    struct wlan_net_vif *wnet_vif = NULL;
    ASSERT(wifimac != NULL);

    if((ctrl_code & STS_MOD_WMAC)  == STS_MOD_WMAC)
    {
            AML_PRINT_LOG_INFO("\n--------mac_statistic--------\n");
 /*vif tx path*/
    if ((ctrl_code & STS_TYP_TX) == STS_TYP_TX)
        {
            list_for_each_entry(wnet_vif,&wifimac->wm_wnet_vifs,vm_next)
                {
                    AML_PRINT_LOG_INFO("\n--------vif_id %d--------\n",  wnet_vif->wnet_vif_id);

                    AML_PRINT_LOG_INFO("tx_total_msdu %d\n", wnet_vif->vif_sts.sts_tx_total_msdu);
                    AML_PRINT_LOG_INFO("tx_total_out_msdu %d\n", wnet_vif->vif_sts.sts_tx_out_msdu);
                    AML_PRINT_LOG_INFO("tx_total_drop_msdu %d\n", wnet_vif->vif_sts.sts_tx_drop_msdu);

                    AML_PRINT_LOG_INFO("\ntx_total_tcp_msdu %d\n", wnet_vif->vif_sts.sts_tx_tcp_msdu);
                    AML_PRINT_LOG_INFO("tx_total_udp_msdu %d\n", wnet_vif->vif_sts.sts_tx_udp_msdu );
                    AML_PRINT_LOG_INFO("tx_total_arp_msdu %d\n",wnet_vif->vif_sts.sts_tx_arp_msdu);
                    AML_PRINT_LOG_INFO("tx_total_ns_msdu %d\n\n",wnet_vif->vif_sts.sts_tx_ns_msdu);


                    for(i = 0; i < WME_NUM_TID; i++)
                    {
                        AML_PRINT_LOG_INFO("tx_tid_in_msdu[%d] %d\n", i,wnet_vif->vif_sts.sts_tx_tid_in_msdu[i]);
                    }

                     for(i = 0; i < WME_NUM_TID; i++)
                    {
                        AML_PRINT_LOG_INFO("tx_tid_drop_msdu[%d] %d\n", i,wnet_vif->vif_sts.sts_tx_tid_drop_msdu[i]);
                    }

                    for(i = 0; i < WME_NUM_TID; i++)
                    {
                        AML_PRINT_LOG_INFO("tx_tid_out_msdu[%d] %d\n", i,wnet_vif->vif_sts.sts_tx_tid_out_msdu[i]);
                    }

                     for(i = 0; i < WME_NUM_TID; i++)
                    {
                        AML_PRINT_LOG_INFO("tx_tid_drop_bf_in_amsdu[%d] %d\n", i,wnet_vif->vif_sts.sts_tx_tid_drop_bf_in_msdu[i]);
                    }


                    for(i = 0; i < WME_NUM_TID; i++)
                    {
                        AML_PRINT_LOG_INFO("tx_amsdu_in_msdu[%d] %d\n", i,wnet_vif->vif_sts.sts_tx_amsdu_in_msdu[i]);
                    }

                    for(i = 0; i < WME_NUM_TID; i++)
                    {
                        AML_PRINT_LOG_INFO("tx_amsdu_drop_msdu[%d] %d\n", i,wnet_vif->vif_sts.sts_tx_amsdu_drop_msdu[i]);
                    }

                     for(i = 0; i < WME_NUM_TID; i++)
                    {
                        AML_PRINT_LOG_INFO("tx_asmdu_pend_msdu[%d] %d\n", i,   wifimac->msdu_cnt[i]);
                    }

                    for(i = 0; i < WME_NUM_TID; i++)
                    {
                        AML_PRINT_LOG_INFO("tx_amsdu_out_msdu[%d] %d\n", i,wnet_vif->vif_sts.sts_tx_amsdu_out_msdu[i]);
                    }

                    for(i = 0; i < WME_NUM_TID; i++)
                    {
                        AML_PRINT_LOG_INFO("tx_non_amsdu_in_msdu[%d] %d\n", i,wnet_vif->vif_sts.sts_tx_non_amsdu_in_msdu[i]);

                    }

                     for(i = 0; i < WME_NUM_TID; i++)
                    {
                        AML_PRINT_LOG_INFO("tx_non_amsdu_drop_msdu[%d] %d\n", i,wnet_vif->vif_sts.sts_tx_non_amsdu_drop_msdu[i]);
                    }

                    for(i = 0; i < WME_NUM_TID; i++)
                    {
                        AML_PRINT_LOG_INFO("tx_non_amsdu_out_msdu[%d] %d\n", i,wnet_vif->vif_sts.sts_tx_non_amsdu_out_msdu[i]);
                    }
                }
        }

/*vif rx path*/
           if ((ctrl_code & STS_TYP_RX) == STS_TYP_RX)
            {
                 list_for_each_entry(wnet_vif,&wifimac->wm_wnet_vifs,vm_next)
                {
                    AML_PRINT_LOG_INFO("\n--------vif_id %d--------\n",  wnet_vif->wnet_vif_id);
                    AML_PRINT_LOG_INFO("rx_bad_ver %d\n", wnet_vif->vif_sts.sts_rx_bad_ver);

                    AML_PRINT_LOG_INFO("rx_too_short %d\n",wnet_vif->vif_sts.sts_rx_too_short);

                    AML_PRINT_LOG_INFO("rx_wrong_bss %d\n",wnet_vif->vif_sts.sts_rx_wrong_bss);

                    AML_PRINT_LOG_INFO("rx_dup %d\n",wnet_vif->vif_sts.sts_rx_dup);

                    AML_PRINT_LOG_INFO("rx_wrong_dir %d\n",wnet_vif->vif_sts.sts_rx_wrong_dir);

                    AML_PRINT_LOG_INFO("rx_mcast_echo %d\n",wnet_vif->vif_sts.sts_rx_mcast_echo);

                    AML_PRINT_LOG_INFO("rx_not_assoc %d\n",wnet_vif->vif_sts.sts_rx_not_assoc);

                    AML_PRINT_LOG_INFO("rx_no_priv_ac %d\n",wnet_vif->vif_sts.sts_no_priv_ac);

                    AML_PRINT_LOG_INFO("rx_uncrypted %d\n",wnet_vif->vif_sts.sts_rx_uncrypted);

                    AML_PRINT_LOG_INFO("rx_decap %d\n",wnet_vif->vif_sts.sts_rx_decap);

                    AML_PRINT_LOG_INFO("rx_mng_discard %d\n",wnet_vif->vif_sts.sts_mng_discard);

                    AML_PRINT_LOG_INFO("rx_ctrl %d\n",wnet_vif->vif_sts.sts_rx_ctrl);

                    AML_PRINT_LOG_INFO("rx_bcn %d\n",wnet_vif->vif_sts.sts_rx_bcn);

                    AML_PRINT_LOG_INFO("rx_elem_miss %d\n",wnet_vif->vif_sts.sts_rx_elem_miss);

                    AML_PRINT_LOG_INFO("rx_elem_too_long %d\n",wnet_vif->vif_sts.sts_rx_elem_too_long);

                    AML_PRINT_LOG_INFO("rx_elem_too_short %d\n",wnet_vif->vif_sts.sts_rx_elem_too_short);

                    AML_PRINT_LOG_INFO("rx_elem_err %d\n",wnet_vif->vif_sts.sts_rx_elem_err);

                    AML_PRINT_LOG_INFO("rx_sta_all_fail %d\n",wnet_vif->vif_sts.sts_rx_sta_all_fail);

                    AML_PRINT_LOG_INFO("rx_ssid_mismatch %d\n", wnet_vif->vif_sts.sts_rx_ssid_mismatch);

                    AML_PRINT_LOG_INFO("rx_auth_unsprt %d\n", wnet_vif->vif_sts.sts_auth_unsprt);

                    AML_PRINT_LOG_INFO("rx_auth_fail %d\n",wnet_vif->vif_sts.sts_rx_auth_fail);

                    AML_PRINT_LOG_INFO("rx_ps_uncnnt %d\n", wnet_vif->vif_sts.sts_rx_ps_uncnnt);

                    AML_PRINT_LOG_INFO("rx_ps_aid_err %d\n",wnet_vif->vif_sts.sts_rx_ps_aid_err);

                    AML_PRINT_LOG_INFO("rx_assoc_err %d\n", wnet_vif->vif_sts.sts_rx_assoc_err);

                    AML_PRINT_LOG_INFO("rx_assoc_unauth %d\n",wnet_vif->vif_sts.sts_rx_assoc_unauth);

                    AML_PRINT_LOG_INFO("rx_assoc_cap_mismatch %d\n", wnet_vif->vif_sts.sts_rx_assoc_cap_mismatch);

                    AML_PRINT_LOG_INFO("rx_assoc_rate_mismatch %d\n", wnet_vif->vif_sts.sts_rx_assoc_rate_mismatch);

                    AML_PRINT_LOG_INFO("rx_assoc_wpa_mismatch %d\n",wnet_vif->vif_sts.sts_rx_assoc_wpa_mismatch);

                    AML_PRINT_LOG_INFO("rx_deauth %d\n", wnet_vif->vif_sts.sts_rx_deauth);

                    AML_PRINT_LOG_INFO("rx_dis_assoc %d\n",wnet_vif->vif_sts.sts_rx_dis_assoc);

                    AML_PRINT_LOG_INFO("rx_action %d\n", wnet_vif->vif_sts.sts_rx_action);

                    AML_PRINT_LOG_INFO("rx_buf_full %d\n",wnet_vif->vif_sts.sts_rx_buf_full);

                    AML_PRINT_LOG_INFO("rx_decrypt_err %d\n", wnet_vif->vif_sts.sts_rx_decrypt_err);

                    AML_PRINT_LOG_INFO("rx_auth_mismatch %d\n", wnet_vif->vif_sts.sts_rx_auth_mismatch);

                    AML_PRINT_LOG_INFO("rx_unauth %d\n",wnet_vif->vif_sts.sts_rx_unauth);

                    AML_PRINT_LOG_INFO("rx_tkip_mic_err %d\n", wnet_vif->vif_sts.sts_rx_tkip_mic_err);

                    AML_PRINT_LOG_INFO("rx_acl_mismatch %d\n", wnet_vif->vif_sts.sts_rx_acl_mismatch);

                    AML_PRINT_LOG_INFO("tx_no_buf %d\n", wnet_vif->vif_sts.sts_tx_no_buf);

                    AML_PRINT_LOG_INFO("tx_no_sta %d\n", wnet_vif->vif_sts.sts_tx_no_sta);

                    AML_PRINT_LOG_INFO("tx_unsprt_mngt %d\n",wnet_vif->vif_sts.sts_tx_unsprt_mngt);

                    AML_PRINT_LOG_INFO("tx_key_err %d\n",wnet_vif->vif_sts.sts_tx_key_err);

                    AML_PRINT_LOG_INFO("tx_headroom_err %d\n",wnet_vif->vif_sts.sts_headroom_err);

                    AML_PRINT_LOG_INFO("tx_sta_aged %d\n",wnet_vif->vif_sts.sts_tx_sta_aged);

                    AML_PRINT_LOG_INFO("tx_tkip_sw_mic_err %d\n",wnet_vif->vif_sts.sts_tx_tkip_sw_mic_err);

                    AML_PRINT_LOG_INFO("tx_ps_no_data %d\n",wnet_vif->vif_sts.sts_tx_ps_no_data);

                    AML_PRINT_LOG_INFO("rx_tkip_sw_mic_err %d\n", wnet_vif->vif_sts.sts_rx_tkip_sw_mic_err);

                    AML_PRINT_LOG_INFO("key_drop %d\n", wnet_vif->vif_sts.sts_key_drop);

                    AML_PRINT_LOG_INFO("key_id_err %d\n", wnet_vif->vif_sts.sts_key_id_err);

                    AML_PRINT_LOG_INFO("key_type_err %d\n",wnet_vif->vif_sts.sts_key_type_err);

                    AML_PRINT_LOG_INFO("rx_msdu %d\n",wnet_vif->vif_sts.sts_rx_msdu);

                    AML_PRINT_LOG_INFO("rx_msdu_time_stamp %ld\n",wnet_vif->vif_sts.sts_rx_msdu_time_stamp);
                }
            }
        }


        wifimac->drv_priv->drv_ops.drv_get_sts(wifimac->drv_priv, op_code, ctrl_code);
}


int wifi_mac_mac_exit(struct wifi_mac *wifimac)
{
    AML_PRINT_LOG_INFO("------>\n");
    wifimac->wm_mac_exit =1;
    wifi_mac_tx_cleanup(wifimac);

    return wifi_mac_cap_detach(wifimac);
}

void get_legacy_sta(void *arg, struct wifi_station *sta)
{
    struct wifi_mac *wifimac = arg;

    if (sta->sta_associd == 0)
        return;

    if (WIFINET_INCLUDE_11N(sta->sta_wnet_vif->vm_mac_mode)&&
        !(sta->sta_flags & WIFINET_NODE_HT))
    {
        wifimac->wm_non_ht_sta++;
    }
}

void get_ht20_sta(void *arg, struct wifi_station *sta)
{
    struct wifi_mac *wifimac = arg;

    if (sta->sta_associd == 0)
        return;

    if ((sta->sta_flags & WIFINET_NODE_HT) &&
        (sta->sta_chbw == WIFINET_BWC_WIDTH20))
    {
        wifimac->wm_ht20_sta++;
    }
}

/*
 * record sta op_mode(ht or legacy), for ht_info filed in
 * beacon/probe resp, refer to wifi_mac_add_htinfo(). This
 * is used to operate 20/40Mhz coexistence.
 */
void wifi_mac_ht_prot(struct wifi_mac *wifimac, struct wifi_station *sta, enum wifi_mac_ht_prot_update_flag flag)
{
    unsigned short legacy_stations = 0;
    unsigned short ht20_stations = 0;
    unsigned short legacy_ap = (wifimac->wm_flags_ext & WIFINET_F_NONHT_AP)?1:0;
    unsigned short prior_sm = wifimac->wm_ht_prot_sm;

    if (!NET80211_IF_RUN_UP(wifimac)) {
        return;
    }

    if (WIFINET_INCLUDE_11N(sta->sta_wnet_vif->vm_mac_mode) && !(sta->sta_flags & WIFINET_NODE_HT)) {
        if (flag == WIFINET_STA_CONNECT) {
            wifimac->wm_non_ht_sta++;

        } else if (flag == WIFINET_STA_DISCONNECT) {
            wifimac->wm_non_ht_sta--;
        }
    }

    if ((sta->sta_flags & WIFINET_NODE_HT) && (sta->sta_chbw == WIFINET_BWC_WIDTH20)) {
        if (flag == WIFINET_STA_CONNECT) {
            wifimac->wm_ht20_sta++;

        } else if (flag == WIFINET_STA_DISCONNECT) {
            wifimac->wm_ht20_sta--;
        }
    }
    legacy_stations = wifimac->wm_non_ht_sta;
    ht20_stations = wifimac->wm_ht20_sta;

    if (legacy_stations) {
        wifimac->wm_ht_prot_sm = WIFINET_HTINFO_OPMODE_MIXED_PROT_ALL;

    } else if (legacy_ap) {
        wifimac->wm_ht_prot_sm = WIFINET_HTINFO_OPMODE_MIXED_PROT_OPT;

    } else {
        wifimac->wm_ht_prot_sm = WIFINET_HTINFO_OPMODE_PURE;
    }

    if (prior_sm != wifimac->wm_ht_prot_sm)
        wifimac->wm_flags_ext |= WIFINET_FEXT_HTUPDATE;
}

void
wifi_mac_change_cbw(struct wifi_mac *wifimac, int to20)
{
    if (wifimac->wm_flags_ext & WIFINET_FEXT_COEXT_DISABLE)
        return;

    wifimac->wm_flags_ext |= WIFINET_FEXT_HTUPDATE;
}

void
wifi_mac_reset_vmac(struct wlan_net_vif *wnet_vif)
{
    wnet_vif->vm_pstxqueue_flags = 0;
}


void wifi_mac_reset_tspecs(struct wlan_net_vif *wnet_vif)
{
    unsigned char ac;
    unsigned char idx;

    for (ac = 0; ac < WME_AC_NUM; ac++) {
        for (idx = 0; idx < TS_DIR_IDX_COUNT; idx++) {
            memset(&wnet_vif->tspecs[ac][idx], 0, sizeof(struct wifi_mac_wmm_tspec_element));
        }
    }
}


void
wifi_mac_reset_ht(struct wifi_mac *wifimac)
{

    if (wifimac->wm_nopened > 1)
        return;

    wifimac->wm_non_ht_sta = 0;
    wifimac->wm_ht20_sta = 0;
    wifimac->wm_ht_prot_sm = WIFINET_HTINFO_OPMODE_PURE;
    wifimac->wm_flags_ext &= ~WIFINET_F_NONHT_AP;
}

void
wifi_mac_reset_vht(struct wifi_mac *wifimac)
{
}

void
wifi_mac_reset_erp(struct wifi_mac *wifimac, enum wifi_mac_macmode mode)
{
    wifimac->wm_flags &= ~WIFINET_F_USEPROT;

    wifimac->wm_nonerpsta = 0;
    wifimac->wm_longslotsta = 0;

    wifi_mac_set_shortslottime(wifimac, ( (WIFINET_MODE_11B != mode) && (wifimac->wm_caps & WIFINET_C_SHSLOT)));

    if (wifimac->wm_caps & WIFINET_C_SHPREAMBLE)
    {
        wifimac->wm_flags |= WIFINET_F_SHPREAMBLE;
        wifimac->wm_flags &= ~WIFINET_F_USEBARKER;
    }
    else
    {
        wifimac->wm_flags &= ~WIFINET_F_SHPREAMBLE;
        wifimac->wm_flags |= WIFINET_F_USEBARKER;
    }

}

void
wifi_mac_set_shortslottime(struct wifi_mac *wifimac, int onoff)
{

    if (onoff)
        wifimac->wm_flags |= WIFINET_F_SHSLOT;
    else
        wifimac->wm_flags &= ~WIFINET_F_SHSLOT;

    wifi_mac_updateslot(wifimac);
}

void
wifi_mac_wme_initparams(struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;

    WIFINET_BEACON_LOCK(wifimac);
    wifi_mac_wme_initparams_locked(wnet_vif);
    WIFINET_BEACON_UNLOCK(wifimac);
}

static void
wifi_mac_phyparam_2_wmeparam (struct wmeParams *ps_dst,
        const struct WmeParamDefault *ps_src)
{
    ps_dst->wmep_acm = ps_src->acm;
    ps_dst->wmep_aifsn = ps_src->aifsn;
    ps_dst->wmep_logcwmin = ps_src->logcwmin;
    ps_dst->wmep_logcwmax = ps_src->logcwmax;
    ps_dst->wmep_txopLimit = ps_src->txopLimit;
}

static void
wifi_mac_wmeparam_2_wmeparam (struct wmeParams *ps_dst,
        const struct wmeParams *ps_src)
{
    ps_dst->wmep_acm = ps_src->wmep_acm;
    ps_dst->wmep_aifsn = ps_src->wmep_aifsn;
    ps_dst->wmep_logcwmin = ps_src->wmep_logcwmin;
    ps_dst->wmep_logcwmax = ps_src->wmep_logcwmax;
    ps_dst->wmep_txopLimit = ps_src->wmep_txopLimit;
}

static enum
wifi_mac_wmm_phymode wifi_mac_wmm_phymode (enum wifi_mac_macmode mode)
{
    enum wifi_mac_wmm_phymode wmm_phymode = WIFINET_WMM_PHYMODE_G;

    if (mode == WIFINET_MODE_11B)
        wmm_phymode = WIFINET_WMM_PHYMODE_B;
    return wmm_phymode;
}

void
wifi_mac_wme_initparams_locked(struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_mac_wme_state *wme = &wifimac->wm_wme[wnet_vif->wnet_vif_id];
    enum wifi_mac_macmode mode;
    enum wifi_mac_wmm_phymode wmm_phymode;
    const struct WmeParamDefault *pApPhyParam, *pStaPhyParam;
    int i;

    AML_PRINT(AML_LOG_ID_WME, AML_LOG_LEVEL_DEBUG, "\n");
    WIFINET_BEACON_LOCK_ASSERT(wifimac);

    if ((wifimac->wm_caps & WIFINET_C_WME) == 0)
        return;

    mode = wnet_vif->vm_mac_mode;
    wmm_phymode = wifi_mac_wmm_phymode(mode);
    AML_PRINT(AML_LOG_ID_WME, AML_LOG_LEVEL_DEBUG, "macmode=%d, wmmphymode=%d\n",mode, wmm_phymode);
    for (i = 0; i < WME_NUM_AC; i++) {
        switch (i) {
            case WME_AC_BK:
                pApPhyParam = &ap_phy_param_AC_BK[wmm_phymode];
                pStaPhyParam = &sta_phy_param_AC_BK[wmm_phymode];
                break;
            case WME_AC_VI:
                pApPhyParam = &ap_phy_param_AC_VI[wmm_phymode];
                pStaPhyParam = &sta_phy_param_AC_VI[wmm_phymode];
                break;
            case WME_AC_VO:
                pApPhyParam = &ap_phy_param_AC_VO[wmm_phymode];
                pStaPhyParam = &sta_phy_param_AC_VO[wmm_phymode];
                break;
            case WME_AC_BE:
            default:
                pApPhyParam = &ap_phy_param_AC_BE[wmm_phymode];
                pStaPhyParam = &sta_phy_param_AC_BE[wmm_phymode];
                break;
        }

        if (wnet_vif->vm_opmode == WIFINET_M_HOSTAP) {
            wifi_mac_phyparam_2_wmeparam(&(wme->wme_wmeChanParams.cap_wmeParams[i]), pApPhyParam);
            wifi_mac_phyparam_2_wmeparam(&(wme->wme_wmeBssChanParams.cap_wmeParams[i]), pApPhyParam);

        } else {
            wifi_mac_phyparam_2_wmeparam(&(wme->wme_wmeChanParams.cap_wmeParams[i]), pStaPhyParam);
        }
        AML_PRINT(AML_LOG_ID_WME, AML_LOG_LEVEL_DEBUG, "wmep_logcwmax =%d\n",
                         wme->wme_wmeChanParams.cap_wmeParams[i].wmep_logcwmax);
    }

    wme->wme_wmeChanParams.cap_info_count = (unsigned char)(-1);
    wme->wme_wmeBssChanParams.cap_info_count = 0;

    if (wnet_vif->vm_mainsta != NULL) {
        wme->wme_hipri_switch_thresh = (10 * wnet_vif->vm_bcn_intval) / 100;
        wifi_mac_wme_updateparams_locked(wnet_vif);
    }
}

static void
wme_update_orig(SYS_TYPE param1,SYS_TYPE param2,
                    SYS_TYPE param3,SYS_TYPE param4,
                    SYS_TYPE param5)
{
    struct wifi_mac *wifimac = (struct wifi_mac *) param1;
    struct wlan_net_vif *wnet_vif = (struct wlan_net_vif *)param4;

    if (wnet_vif->wnet_vif_replaycounter != (int)param5)
        return ;
    wifi_mac_wmm_update2hal(wifimac,wnet_vif);
}

void
wme_update_ex(struct wifi_mac *wifimac,struct wlan_net_vif *wnet_vif )
{
    wifi_mac_add_work_task(wifimac, wme_update_orig,
            NULL, (SYS_TYPE)wifimac,0,0,
            (SYS_TYPE)wnet_vif,wnet_vif->wnet_vif_replaycounter);
}


void
wifi_mac_wme_updateparams_locked(struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_mac_wme_state *wme = &wifimac->wm_wme[wnet_vif->wnet_vif_id];
    enum wifi_mac_macmode mode;
    enum wifi_mac_wmm_phymode wmm_phymode;
    int i;

    AML_PRINT(AML_LOG_ID_WME, AML_LOG_LEVEL_DEBUG, "\n");

    for (i = 0; i < WME_NUM_AC; i++) {
        wifi_mac_wmeparam_2_wmeparam(&(wme->wme_chanParams.cap_wmeParams[i]),
                &(wme->wme_wmeChanParams.cap_wmeParams[i]));
        AML_PRINT(AML_LOG_ID_WME, AML_LOG_LEVEL_DEBUG, "wmep_logcwmax =%d\n",
                wme->wme_wmeChanParams.cap_wmeParams[i].wmep_logcwmax);

        wifi_mac_wmeparam_2_wmeparam(&(wme->wme_bssChanParams.cap_wmeParams[i]),
                &(wme->wme_wmeBssChanParams.cap_wmeParams[i]));
    }

    mode = wnet_vif->vm_mac_mode;
    wmm_phymode = wifi_mac_wmm_phymode(mode);

    if (((wnet_vif->vm_opmode == WIFINET_M_HOSTAP) && ((wme->wme_flags & WME_F_AGGRESSIVE) != 0))
        || ((wnet_vif->vm_opmode != WIFINET_M_HOSTAP) && (wnet_vif->vm_mainsta->sta_flags & WIFINET_NODE_QOS) == 0)
        || ((wnet_vif->vm_flags & WIFINET_F_WME) == 0)) {
        AML_PRINT(AML_LOG_ID_WME, AML_LOG_LEVEL_DEBUG, "use phy_param_BE_nonqos for BE\n");

        wifi_mac_phyparam_2_wmeparam(&(wme->wme_chanParams.cap_wmeParams[WME_AC_BE]),
                &phy_param_BE_nonqos[wmm_phymode]);

        wifi_mac_phyparam_2_wmeparam(&(wme->wme_bssChanParams.cap_wmeParams[WME_AC_BE]),
                &phy_param_BE_nonqos[wmm_phymode]);
    }

    if ((wnet_vif->vm_opmode == WIFINET_M_HOSTAP) && (wnet_vif->vm_sta_assoc < 2)
        && ((wme->wme_flags & WME_F_AGGRESSIVE) != 0)) {
        AML_PRINT(AML_LOG_ID_WME, AML_LOG_LEVEL_DEBUG, " use log_cwwin_BE_agg_log for BE\n");
        wme->wme_chanParams.cap_wmeParams[WME_AC_BE].wmep_logcwmin
            = wme->wme_bssChanParams.cap_wmeParams[WME_AC_BE].wmep_logcwmin
            = log_cwwin_BE_agg_log[wmm_phymode];
    }

    if (wnet_vif->vm_opmode == WIFINET_M_HOSTAP) {
        wme->wme_bssChanParams.cap_info_count = (wme->wme_bssChanParams.cap_info_count+1) & WME_QOSINFO_COUNT;
        wnet_vif->vm_flags |= WIFINET_F_WMEUPDATE;
        AML_PRINT(AML_LOG_ID_WME, AML_LOG_LEVEL_DEBUG, "update qosinfo count=%d\n",
                        wme->wme_bssChanParams.cap_info_count);
    }

    wme_update_ex(wifimac,wnet_vif);
}

void
wifi_mac_wme_updateparams(struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;

    if (wifimac->wm_caps & WIFINET_C_WME)
    {
        WIFINET_BEACON_LOCK(wifimac);
        wifi_mac_wme_updateparams_locked(wnet_vif);
        WIFINET_BEACON_UNLOCK(wifimac);
    }
}

int wifi_mac_check_main_sta(struct wlan_net_vif *wnet_vif)
{
    int return_val = 0;
    if (!wnet_vif->vm_mainsta) {
        AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_WARN, " vid:%d vm_mainsta is NULL, alloc new sta\n", wnet_vif->wnet_vif_id);

        /*sta side allocate a sta buffer to save ap capabilities and information */
        wnet_vif->vm_mainsta = wifi_mac_get_sta_node(&wnet_vif->vm_sta_tbl, wnet_vif, wnet_vif->vm_myaddr);
        if (wnet_vif->vm_mainsta == NULL)
        {
            AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_ERROR, "ERROR  alloc sta FAIL!!! \n");
            return_val = 1;
        }
    }
   return  return_val;
}


int wifi_mac_initial(struct net_device *dev, int forcescan)
{
    struct wlan_net_vif *wnet_vif = netdev_priv(dev);
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    int return_val = 0;
    struct hal_private *hal_priv = hal_get_priv();
    int total_delay = 0;

    AML_PRINT(AML_LOG_ID_INIT, AML_LOG_LEVEL_INFO,"<%s>:<running> \n", wnet_vif->vm_ndev->name);

    while ((total_delay < 1000) && (wnet_vif->vm_recovery_state != WIFINET_RECOVERY_END)) {
        msleep(10);
        total_delay +=10;
    }

    wifi_mac_preempt_scan(wifimac, 100, 100);

    if (hal_ant_sel_en_get() != 0) {
        wifi_mac_ant_init(hal_priv);
    }

    if (1 == wifi_mac_check_main_sta(wnet_vif)) {
        return 1;
    }

    hal_priv->powersave_init_flag = 0;
    wifimac->scan_noisy_status = WIFINET_S_SCAN_ENV_NOISE;
    wifimac->is_connect_set_gain = 1;

    /*dev/interface is not up. */
    if ((dev->flags & IFF_RUNNING) == 0) {
        AML_PRINT_LOG_INFO("<%s>wm_nopened =%d\n", wnet_vif->vm_ndev->name, wifimac->wm_nopened);
        /*set power save state */
        wifi_mac_pwrsave_wakeup(wnet_vif, WKUP_FROM_VMAC_UP);

        if ((wifimac->wm_nopened++ == 0) && (NET80211_IF_RUNNING(wifimac) == 0)) {
            AML_PRINT(AML_LOG_ID_INIT, AML_LOG_LEVEL_DEBUG, "<%s>: before calling wifi_mac_netdev_open\n", wnet_vif->vm_ndev->name);

            /*set running and up flag for wifimac*/
            return_val = wifi_mac_netdev_open(wifimac);

            if (return_val != 0)
                return return_val;
        }
        vm_cfg80211_up(wnet_vif);

        /*set running & up flag for dev/interface. */
        dev->flags |= IFF_RUNNING | IFF_UP;
#if (DEFAULT_INITIAL_POWERMODE == 1)
        wifi_mac_pwrsave_set_mode(wnet_vif, WIFINET_PWRSAVE_LOW);
#endif
    }

    if (NET80211_IF_RUN_UP(wifimac))
    {
        if (wnet_vif->vm_opmode == WIFINET_M_STA)
        {
            AML_PRINT(AML_LOG_ID_CONNECT, AML_LOG_LEVEL_DEBUG,"<%s>:<running> \n", wnet_vif->vm_ndev->name);
            if ((wnet_vif->vm_state != WIFINET_S_CONNECTED) || forcescan)
            {
                wifi_mac_top_sm(wnet_vif, WIFINET_S_SCAN, 0);
            }
            else
            {
                wifi_mac_top_sm(wnet_vif, WIFINET_S_ASSOC, 1);
            }
        }
        else
        {
            if (wnet_vif->vm_opmode == WIFINET_M_MONITOR
                || wnet_vif->vm_opmode == WIFINET_M_WDS) {
                wnet_vif->vm_state = WIFINET_S_INIT;
                wifi_mac_top_sm(wnet_vif, WIFINET_S_CONNECTED, -1);
            } else {
                wifi_mac_top_sm(wnet_vif, WIFINET_S_SCAN, 0);
            }
        }
    }
    return 0;
}

int wifi_mac_init(struct net_device *dev)
{
    struct wlan_net_vif *wnet_vif = netdev_priv(dev);
    AML_PRINT_LOG_INFO("dev = %p, vid = %d\n", dev, wnet_vif->wnet_vif_id);
    return 0;
}

void wifi_mac_uninit(struct net_device *dev)
{
    struct wlan_net_vif *wnet_vif = netdev_priv(dev);
    AML_PRINT_LOG_INFO("dev = %p, vid = %d\n", dev, wnet_vif->wnet_vif_id);
}

int wifi_mac_open(struct net_device *dev)
{
    if (!aml_insmod_flag) {
        AML_PRINT_LOG_ERR("<running> dev = %p, aml insmod not complete\n",dev);
        return -1;
    }

    AML_PRINT_LOG_INFO("<running> dev = %p\n",dev);
    return wifi_mac_initial(dev, 0);
}

int wifi_mac_stop(struct net_device *dev)
{
    struct wlan_net_vif *wnet_vif = netdev_priv(dev);
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    int index = 0;

    if (!aml_insmod_flag) {
        AML_PRINT_LOG_ERR("<running> dev = %p, aml insmod not complete\n", dev);
        return -1;
    }

    AML_PRINT_LOG_INFO("wm_nopened %d, wm_dev_flags:%08x\n", wifimac->wm_nopened, wifimac->wm_dev_flags);
    preempt_scan(dev, 100, 100);

    if (dev->flags & IFF_RUNNING)
    {
        dev->flags &= ~IFF_RUNNING;

        wifi_mac_top_sm(wnet_vif, WIFINET_S_INIT, -1);

        os_timer_ex_cancel(&wnet_vif->vm_mgtsend, CANCEL_SLEEP);
        os_timer_ex_cancel(&wnet_vif->vm_actsend, CANCEL_SLEEP);
#ifdef  AML_UNIT_TESTCASE
        aml_ifdown_unit_testcase_stop(dev);
#endif

        vm_cfg80211_down(wnet_vif);

        if ((--wifimac->wm_nopened == 0) && NET80211_IF_RUNNING(wifimac)) {
            WIFINET_DPRINTF(AML_LOG_ID_LOG, AML_LOG_LEVEL_DEBUG, "driver open ic_nopened %d\n", wifimac->wm_nopened);
            wifi_mac_netdev_stop(wifimac);
#if 0
            WIFINET_FW_STAT_LOCK(wifimac);
            if ((wifimac->wm_recovery_flags & WIFINET_RECOVERY_F_RUNNING) == 0) {
                wifimac->wm_recovery_flags |= WIFINET_RECOVERY_F_RUNNING;
                WIFINET_FW_STAT_UNLOCK(wifimac);

                msleep(200);
                wifimac->drv_priv->drv_ops.fw_repair(wifimac->drv_priv);
                WIFINET_FW_STAT_LOCK(wifimac);
                wifimac->wm_recovery_flags &= ~WIFINET_RECOVERY_F_RUNNING;
            }
            WIFINET_FW_STAT_UNLOCK(wifimac);
#endif
        }

        for (index=0; index<WIFINET_APPIE_NUM_OF_FRAME; index++) {
            wifi_mac_rm_app_ie(&wnet_vif->app_ie[index]);
        }
        wifi_mac_rm_app_ie(&wnet_vif->assocrsp_ie);
        wifi_mac_rm_app_ie(&wnet_vif->assocreq_ie);
        wnet_vif->vm_opt_ie_len =0;
    }

    return 0;
}


void
wifi_mac_stop_running(struct wifi_mac *wifimac)
{
    struct wlan_net_vif *wnet_vif = NULL, *wnet_vif_next = NULL;
    struct net_device *dev;

    list_for_each_entry_safe(wnet_vif,wnet_vif_next,&wifimac->wm_wnet_vifs,vm_next)
    {
        dev = wnet_vif->vm_ndev;
        if (dev->flags & IFF_RUNNING)
        {
            wifi_mac_stop(dev);
        }
    }
}
static void wifi_mac_tx_act_timeout_ex (SYS_TYPE param1,
        SYS_TYPE param2,SYS_TYPE param3,SYS_TYPE param4,SYS_TYPE param5)
{
    struct wlan_net_vif *wnet_vif = (struct wlan_net_vif *)param1;
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_mac_p2p *p2p = wnet_vif->vm_p2p;
    AML_PRINT_LOG_INFO("vid:%d, act_pkt_retry_count:%d, token:%d\n",
        wnet_vif->wnet_vif_id, wnet_vif->vm_p2p->act_pkt_retry_count, wnet_vif->vm_p2p->action_dialog_token);

    if (wnet_vif->vm_p2p->tx_status_flag == WIFINET_TX_STATUS_SUCC) {
        return;
    }

    if ((wnet_vif->vm_flags_ext2 & WIFINET_FEXT2_DPP_CONNECTION_STATUS_RETRY)) {
        if (wnet_vif->vm_p2p->act_pkt_retry_count < 10) {
            int delay = 0;
            if (p2p->action_chan && (wifimac->wm_curchan != p2p->action_chan)) {
                if (wifi_mac_is_wm_running(wifimac) == true) {
                    wifi_mac_scan_notify_leave_or_back(wnet_vif, 1);
                    while ((!wifimac->drv_priv->hal_priv->hal_ops.hal_tx_empty()) && delay < 500) {
                       msleep(10);
                       delay += 10;
                    }
                    wnet_vif->vm_flags_ext2 |= WIFINET_FEXT2_MGMT_RESTORE_CHANNEL;
                }
                wifi_mac_ChangeChannel(wifimac, p2p->action_chan, 0, wnet_vif->wnet_vif_id);
            }
            wnet_vif->vm_p2p->act_pkt_retry_count++;
            AML_PRINT_LOG_INFO("dpp status retry pri_chan:%d target chan:%d delay:%d flag:%x retry_cnt:%d\n", wifimac->wm_curchan->chan_pri_num,
                p2p->action_chan->chan_pri_num, delay, wnet_vif->vm_flags_ext2, wnet_vif->vm_p2p->act_pkt_retry_count);
            //need a lock
            vm_cfg80211_send_mgmt(wnet_vif, wnet_vif->vm_p2p->action_pkt, wnet_vif->vm_p2p->action_pkt_len);
        } else {
            AML_PRINT_LOG_INFO("dpp connection send fail\n");
            wnet_vif->vm_flags_ext2 &= ~WIFINET_FEXT2_DPP_CONNECTION_STATUS_RETRY;
        }
        return;
    }

    if ((wnet_vif->vm_p2p->tx_status_flag == WIFINET_TX_STATUS_FAIL && wnet_vif->vm_p2p->act_pkt_retry_count == DEFAULT_P2P_ACTION_RETRY_TIMES)) {
        wnet_vif->vm_p2p->send_tx_status_flag = 1;
        cfg80211_mgmt_tx_status(wnet_vif->vm_wdev, wnet_vif->vm_p2p->cookie, wnet_vif->vm_p2p->raw_action_pkt, wnet_vif->vm_p2p->raw_action_pkt_len, WIFINET_TX_STATUS_FAIL, GFP_KERNEL);
        return;
    }

    if (wnet_vif->vm_p2p_support == 1) {
        if ((wifimac->wm_curchan != WIFINET_CHAN_ERR) && (wnet_vif->vm_p2p->work_channel != WIFINET_CHAN_ERR)
            && (wnet_vif->vm_p2p->work_channel->chan_pri_num == wifimac->wm_curchan->chan_pri_num)
            && (wnet_vif->vm_p2p->action_dialog_token != 0
#ifdef CTS_VERIFIER_GAS
            || wnet_vif->vm_p2p->action_code == WIFINET_ACT_PUBLIC_GAS_REQ || wnet_vif->vm_p2p->action_code == WIFINET_ACT_PUBLIC_GAS_RSP
#endif
            ) && (wnet_vif->vm_p2p->act_pkt_retry_count < DEFAULT_P2P_ACTION_RETRY_TIMES)) {
            AML_PRINT_LOG_INFO("p2p retry pri_chan:%d\n", wnet_vif->vm_p2p->work_channel->chan_pri_num);
            //need a lock
            wnet_vif->vm_p2p->act_pkt_retry_count++;
            vm_cfg80211_send_mgmt(wnet_vif, wnet_vif->vm_p2p->action_pkt, wnet_vif->vm_p2p->action_pkt_len);
            os_timer_ex_start_period(&wnet_vif->vm_actsend, wnet_vif->vm_p2p->action_retry_time);
            return;
        }

        if (wnet_vif->vm_p2p->tx_status_flag == WIFINET_TX_STATUS_FAIL) {
            wnet_vif->vm_p2p->send_tx_status_flag = 1;
            cfg80211_mgmt_tx_status(wnet_vif->vm_wdev, wnet_vif->vm_p2p->cookie, wnet_vif->vm_p2p->raw_action_pkt, wnet_vif->vm_p2p->raw_action_pkt_len, WIFINET_TX_STATUS_FAIL, GFP_KERNEL);
        }

        if (wnet_vif->vm_state == WIFINET_S_SCAN) {
            return;
        }
    }
}

static void wifi_mac_tx_timeout_ex (SYS_TYPE param1,
        SYS_TYPE param2,SYS_TYPE param3,SYS_TYPE param4,SYS_TYPE param5)
{
    struct wlan_net_vif *wnet_vif = (struct wlan_net_vif *)param1;

    AML_PRINT_LOG_INFO("vid:%d, mgmt_pkt_retry_count:%d, token:%d\n",
        wnet_vif->wnet_vif_id, wnet_vif->mgmt_pkt_retry_count, wnet_vif->vm_p2p->action_dialog_token);

    if (wnet_vif->mgmt_pkt_retry_count++ < DEFAULT_MGMT_RETRY_TIMES) {
        if (wnet_vif->vm_state == WIFINET_S_AUTH) {
            /* swap the auth type between open and shared when auth timeout after sending auth frame to a wep AP */
            if (wnet_vif->mgmt_pkt_retry_count == DEFAULT_MGMT_RETRY_TIMES -1) {
                if ((wnet_vif->vm_flags & WIFINET_F_PRIVACY) && !(wnet_vif->vm_flags & WIFINET_F_WPA)) {
                    if (wnet_vif->vm_mainsta->sta_authmode == WIFINET_AUTH_SHARED) {
                        wnet_vif->vm_mainsta->sta_authmode = WIFINET_AUTH_OPEN;

                    } else {
                        wnet_vif->vm_mainsta->sta_authmode = WIFINET_AUTH_SHARED;
                    }
                }
            }
            wifi_mac_top_sm(wnet_vif, WIFINET_S_AUTH, WIFINET_FC0_SUBTYPE_DEAUTH);

        } else if (wnet_vif->vm_state == WIFINET_S_ASSOC) {
            wifi_mac_top_sm(wnet_vif, WIFINET_S_ASSOC, 0);

        } else if (wnet_vif->vm_phase_flags & PHASE_DISCONNECTING) {
            AML_PRINT_LOG_INFO("deauth tx tot vid:%d\n", wnet_vif->wnet_vif_id);
            wifi_mac_top_sm(wnet_vif, WIFINET_S_SCAN, 0);
        } else {
            wnet_vif->mgmt_pkt_retry_count = 0;
            return;
        }

        //AML_PRINT_LOG_INFO("wifi_mac->mgmt_pkt_retry_count:%d\n", wifimac->mgmt_pkt_retry_count);
        os_timer_ex_start_period(&wnet_vif->vm_mgtsend, wnet_vif->mgmt_pkt_retry_count * DEFAULT_MGMT_RETRY_INTERVAL);
        return;
    }

    wnet_vif->mgmt_pkt_retry_count = 0;
    if ((wnet_vif->vm_state == WIFINET_S_AUTH) || (wnet_vif->vm_state == WIFINET_S_ASSOC)) {
        wifi_mac_show_per_info();
        wifi_mac_top_sm(wnet_vif, WIFINET_S_SCAN, 0);
    }
}

static int wifi_mac_mgmt_tx_timeout(void* arg)
{
    struct wlan_net_vif *wnet_vif = (struct wlan_net_vif *) arg;

    AML_PRINT_LOG_INFO("vm_state %s, vid:%d\n", wifi_mac_state_name[wnet_vif->vm_state], wnet_vif->wnet_vif_id);
    if ((wnet_vif->vm_p2p_support == 1) || (wnet_vif->vm_state > WIFINET_S_CONNECTING)) {
        wifi_mac_add_work_task(wnet_vif->vm_wmac,wifi_mac_tx_timeout_ex, NULL,(SYS_TYPE)arg,0,0,0, 0);
    }

    return OS_TIMER_NOT_REARMED;
}

void wifi_mac_sm_switch (SYS_TYPE param1,
        SYS_TYPE param2,SYS_TYPE param3,SYS_TYPE param4,SYS_TYPE param5)
{
    struct wlan_net_vif *wnet_vif = (struct wlan_net_vif *)param1;
    enum wifi_mac_state state = (enum wifi_mac_state)param2;

    AML_PRINT_LOG_INFO("state:%d\n", state);

    if(state != WIFINET_S_INIT) {
        wifi_mac_top_sm(wnet_vif, state, 0);
    }
}

static int wifi_mac_sm_switch_timeout(void* arg)
{
    struct wlan_net_vif *wnet_vif = (struct wlan_net_vif *) arg;

    wifi_mac_add_work_task(wnet_vif->vm_wmac, wifi_mac_sm_switch, NULL, (SYS_TYPE)wnet_vif, WIFINET_S_SCAN, 0, 0, 0);
    return OS_TIMER_NOT_REARMED;
}

static int wifi_mac_act_tx_timeout(void* arg)
{
    struct wlan_net_vif *wnet_vif = (struct wlan_net_vif *)arg;
    struct wifi_station_tbl *nt = &wnet_vif->vm_sta_tbl;
    struct wifi_station *sta = NULL;
    struct wifi_station *sta_next = NULL;

    if (wnet_vif->vm_p2p_support == 1 || (wnet_vif->vm_flags_ext2 & WIFINET_FEXT2_DPP_CONNECTION_STATUS_RETRY)) {
        WIFINET_DPRINTF(AML_LOG_ID_ACTION, AML_LOG_LEVEL_INFO,"vm_state %s", wifi_mac_state_name[wnet_vif->vm_state]);
        wifi_mac_add_work_task(wnet_vif->vm_wmac,wifi_mac_tx_act_timeout_ex, NULL,(SYS_TYPE)arg,0,0,0, 0);
    }
#ifdef SUPPORT_80211W
    if (wnet_vif->vm_opmode == WIFINET_M_STA) {
        if (wnet_vif->vm_mainsta->sta_flags_ext & WIFINET_NODE_MFP_CONFIRM_DEAUTH) {
            if (wnet_vif->vm_mainsta->sta_wmac->wm_suspend_mode == WIFI_SUSPEND_STATE_NONE) {
                wifi_mac_send_sa_query(wnet_vif->vm_mainsta, WIFINET_ACTION_SA_QUERY_REQ, wnet_vif->vm_mainsta->sa_query_seq++);
            }
            else {
                AML_PRINT_LOG_WRAN("wait 100ms resume complete\n");
            }

            if (wnet_vif->vm_mainsta->sa_query_try_count++ < 20) {
                os_timer_ex_start_period(&wnet_vif->vm_actsend, 100);
            } else {
                AML_PRINT_LOG_INFO("sta not get pmf sa query rsp, just disconnect\n");
                wnet_vif->vm_chan_roaming_scan_flag = 0;
                wifi_mac_scan_access(wnet_vif);
                wifi_mac_add_work_task(wnet_vif->vm_wmac, wifi_mac_sm_switch, NULL, (SYS_TYPE)wnet_vif, WIFINET_S_SCAN, 0, 0, 0);
            }

        }

    } else if (wnet_vif->vm_opmode == WIFINET_M_HOSTAP) {
        WIFINET_NODE_LOCK(nt);
        list_for_each_entry_safe(sta, sta_next, &nt->nt_nsta, sta_list) {
            if (sta->sta_flags_ext & WIFINET_NODE_MFP_CONFIRM_DEAUTH) {
                if (wnet_vif->vm_mainsta->sta_wmac->wm_suspend_mode == WIFI_SUSPEND_STATE_NONE) {
                    wifi_mac_send_sa_query(sta, WIFINET_ACTION_SA_QUERY_REQ, sta->sa_query_seq++);
                }
                else {
                    AML_PRINT_LOG_WRAN("wait 100ms resume complete\n");
                }

                if (sta->sa_query_try_count++ < 10) {
                    os_timer_ex_start_period(&wnet_vif->vm_actsend, 100);

                } else {
                    AML_PRINT_LOG_INFO("sta:%p not get pmf sa query rsp, just disconnect\n", sta);
                    wifi_mac_sta_disconnect(sta);
                    wifi_mac_rm_sta_from_wds_by_addr(nt,sta->sta_macaddr);
                    list_del_init(&sta->sta_list);
                    WIFINET_NODE_UNLOCK(nt);
                    wifi_mac_free_sta(sta);
                    WIFINET_NODE_LOCK(nt);
                }
            }
        }
        WIFINET_NODE_UNLOCK(nt);
    }
#endif
    return OS_TIMER_NOT_REARMED;
}


static int wifi_mac_RocTimeout(void *arg)
{
    struct wlan_net_vif *wnet_vif = (struct wlan_net_vif *)arg;

    cfg80211_remain_on_channel_expired(wnet_vif->vm_wdev, wnet_vif->remain_on_ch_cookie,
        &wnet_vif->remain_on_ch_channel, GFP_KERNEL);

    return OS_TIMER_NOT_REARMED;
}


static void wifi_mac_check_special_ap(struct wlan_net_vif *wnet_vif)
{
    int i;
    char special_dev_manu_addr[][3]={
        {0x88, 0xc3, 0x97},
        {0x64, 0x64, 0x4a},
        {0x50, 0x64, 0x2b},
    };

    wnet_vif->vm_wmac->wm_signal_power_weak_thresh_narrow = WIFINET_SIGNAL_POWER_WEAK_THRESH_NARROW;
    wnet_vif->vm_wmac->wm_signal_power_weak_thresh_wide = WIFINET_SIGNAL_POWER_WEAK_THRESH_WIDE;

    wnet_vif->vm_wmac->wm_signal_power_bw_change_thresh_narrow = WIFINET_SIGNAL_POWER_BW_CHANGE_THRESH_NARROW;
    wnet_vif->vm_wmac->wm_signal_power_bw_change_thresh_wide = WIFINET_SIGNAL_POWER_BW_CHANGE_THRESH_WIDE;

    for (i = 0; i < ARRAY_SIZE(special_dev_manu_addr); i++) {
        if (!(memcmp(wnet_vif->vm_des_bssid, special_dev_manu_addr[i], 3))) {
            wnet_vif->vm_wmac->wm_signal_power_weak_thresh_narrow = WIFINET_SIGNAL_POWER_WEAK_THRESH_NARROW + 8;
            wnet_vif->vm_wmac->wm_signal_power_weak_thresh_wide = WIFINET_SIGNAL_POWER_WEAK_THRESH_WIDE + 31;

            break;
        }
    }
}

void wifi_mac_interface_down_ex(SYS_TYPE param1, SYS_TYPE param2, SYS_TYPE param3, SYS_TYPE param4, SYS_TYPE param5)
{
    struct wlan_net_vif *wnet_vif = (struct wlan_net_vif *)param1;

    AML_PRINT_LOG_INFO("interface down\n");
    if (wnet_vif->vm_recovery_state == WIFINET_RECOVERY_END) {
        wnet_vif->vm_curchan = WIFINET_CHAN_ERR;
        wnet_vif->vm_des_nssid = 0;
        memset(wnet_vif->vm_des_ssid, 0, IV_SSID_SCAN_AMOUNT*sizeof(struct wifi_mac_ScanSSID));
    } else {
        AML_PRINT_LOG_INFO("dont't clear ssid/chan info due to recovery in progress!\n");
    }
}

static int
wifi_mac_sub_sm(struct wlan_net_vif *wnet_vif, enum wifi_mac_state nstate, int arg)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_station *sta;
    enum wifi_mac_state prestate;
    int mgmt_arg;
    int ret = 0;

    prestate = wnet_vif->vm_state;
    wnet_vif->vm_state = nstate;

    if (prestate == WIFINET_S_CONNECTED && nstate != WIFINET_S_CONNECTED)
    {
        wifimac->wm_runningmask &= ~BIT(wnet_vif->wnet_vif_id);
        wifimac->wm_nrunning--;
    }

    WIFINET_DPRINTF(AML_LOG_ID_STATE, AML_LOG_LEVEL_INFO, "state %s->%s",
            wifi_mac_state_name[prestate],wifi_mac_state_name[nstate]);

    os_timer_ex_cancel(&wnet_vif->vm_mgtsend, CANCEL_NO_SLEEP);

    if (wnet_vif->vm_opmode != WIFINET_M_HOSTAP && prestate != WIFINET_S_SCAN)
    {
        wifi_mac_cancel_scan(wifimac);
    }

    if (wnet_vif->vm_mainsta == NULL) {
        AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_WARN, " vid:%d vm_mainsta is NULL, alloc new sta\n", wnet_vif->wnet_vif_id);

        /*sta side allocate a sta buffer to save ap capabilities and information */
        wnet_vif->vm_mainsta = wifi_mac_get_sta_node(&wnet_vif->vm_sta_tbl, wnet_vif, wnet_vif->vm_myaddr);
        if (wnet_vif->vm_mainsta == NULL) {
            AML_PRINT(AML_LOG_ID_LOG,AML_LOG_LEVEL_ERROR, "ERROR:: alloc sta FAIL!!! \n");
            return -EINVAL;
        }
    }
    sta = wnet_vif->vm_mainsta;

    switch (nstate)
    {
        case WIFINET_S_INIT:
            switch (prestate)
            {
                case WIFINET_S_INIT:
                    break;
                case WIFINET_S_CONNECTED:
                    switch (wnet_vif->vm_opmode)
                    {
                        case WIFINET_M_STA:
                            wifi_mac_sta_leave(sta, 0);
                            break;

                        case WIFINET_M_HOSTAP:
                            AML_PRINT_LOG_INFO("disassoc all sta\n");
                            wifi_mac_func_to_task(&wnet_vif->vm_sta_tbl, wifi_mac_sta_disassoc, wnet_vif, 1);
                            wifi_mac_add_work_task(wnet_vif->vm_wmac, wifi_mac_rst_bss_ex, NULL,(SYS_TYPE)wnet_vif, 0, 0, 0, 0);
                            break;

                        case WIFINET_M_IBSS:
                            wifi_mac_func_to_task(&wnet_vif->vm_sta_tbl,NULL,wnet_vif,0);
                            goto reset;

                        default:
                            goto reset;
                    }
                    break;

                case WIFINET_S_ASSOC:
                    switch (wnet_vif->vm_opmode)
                    {
                        case WIFINET_M_STA:
                            mgmt_arg = WIFINET_REASON_AUTH_LEAVE;
                            wifi_mac_send_mgmt(sta, WIFINET_FC0_SUBTYPE_DEAUTH, (void *)&mgmt_arg);
                            break;
                        case WIFINET_M_HOSTAP:
                            wifi_mac_func_to_task(&wnet_vif->vm_sta_tbl, wifi_mac_sta_deauth, wnet_vif,1);
                            break;
                        default:
                            break;
                    }
                    goto reset;

                case WIFINET_S_SCAN:
                    wifi_mac_cancel_scan(wifimac);
                    goto reset;

                case WIFINET_S_CONNECTING:
                case WIFINET_S_AUTH:
                reset:
                    AML_PRINT(AML_LOG_ID_CONNECT, AML_LOG_LEVEL_DEBUG,"\n");
                    wifi_mac_rst_bss(wnet_vif);
                    break;

                case WIFINET_S_MAX:
                    AML_PRINT_LOG_INFO("need to check twice\n");
                    break;
            }
            break;

        case WIFINET_S_SCAN:
#ifdef INTER_BEACON
            /* start inter-beacon */
            if ((wnet_vif->vm_opmode == WIFINET_M_STA) || (wnet_vif->vm_opmode == WIFINET_M_P2P_CLIENT)
                || (wnet_vif->vm_opmode == WIFINET_M_P2P_DEV) || (wnet_vif->vm_opmode == WIFINET_M_IBSS))
            {
                wnet_vif->vm_bcn_intval = WIFINET_INTER_BEACON_INTERVAL;
            }
#endif
            switch (prestate)
            {
                case WIFINET_S_INIT:
                createibss:
                    if (wnet_vif->vm_opmode == WIFINET_M_HOSTAP)
                    {
                        if ((wnet_vif->vm_curchan != WIFINET_CHAN_ERR && wnet_vif->vm_recovery_state == WIFINET_RECOVERY_END)
                            && (WIFINET_IS_CHAN_5GHZ(wnet_vif->vm_curchan) || (wnet_vif->vm_bandwidth == WIFINET_BWC_WIDTH20)))
                        {
                            WIFINET_DPRINTF(AML_LOG_ID_STATE, AML_LOG_LEVEL_INFO,"wifi_mac_create_ap channel = %d\n",
                                             wnet_vif->vm_curchan->chan_pri_num);
                            wifi_mac_create_wifi(wnet_vif, wnet_vif->vm_curchan);
                        } else {
                            WIFINET_DPRINTF(AML_LOG_ID_STATE, AML_LOG_LEVEL_INFO,"sm:ap mode, vm_des_nssid:%d",
                                            wnet_vif->vm_des_nssid);

                            if (wnet_vif->vm_des_nssid != 0) {
                                if (!wifi_mac_chk_scan(wnet_vif, WIFINET_SCANCFG_ACTIVE | WIFINET_SCANCFG_FLUSH |
                                    WIFINET_SCANCFG_CREATE, wnet_vif->vm_des_nssid, wnet_vif->vm_des_ssid)) {
                                        wifi_mac_chk_ap_chan(wifimac->wm_scan, wnet_vif);
                                    }
                            } else {
                                //back to WIFINET_S_INIT if not config ssid
                                wnet_vif->vm_state = prestate;
                            }
                        }
                    }
                    break;
                case WIFINET_S_SCAN:
                case WIFINET_S_CONNECTING:
                case WIFINET_S_AUTH:
                case WIFINET_S_ASSOC:
                    /* if (wifimac->wm_roaming == WIFINET_ROAMING_AUTO) */
                    if ((wnet_vif->vm_connecting_retry_cnt++ >= WIFINET_NODE_MAX_CONNECTING_CNT)
                        || (sta->connect_status == CONNECT_DISCONNECTING)) {
                        struct vm_wdev_priv *pwdev_priv = wdev_to_priv(wnet_vif->vm_wdev);
                        AML_PRINT_LOG_INFO("cancel connection and indicate disconenct\n");
                        os_timer_ex_cancel(&pwdev_priv->connect_timeout, CANCEL_SLEEP);
                        wifi_mac_notify_nsta_disconnect(sta, 0);
                        wnet_vif->vm_des_nssid = 0;
                        wnet_vif->vm_curchan = WIFINET_CHAN_ERR;
                        memset(wnet_vif->vm_des_ssid, 0, IV_SSID_SCAN_AMOUNT*sizeof(struct wifi_mac_ScanSSID));
                        wnet_vif->vm_connecting_retry_cnt = 0;
                    } else {
                        WIFINET_DPRINTF(AML_LOG_ID_STATE, AML_LOG_LEVEL_INFO,"sm:scan/connecting/auth/assoc:");
                        wifi_mac_chk_scan(wnet_vif, WIFINET_SCANCFG_ACTIVE| WIFINET_SCANCFG_FLUSH |arg,
                            wnet_vif->vm_des_nssid, wnet_vif->vm_des_ssid);
                        if (wnet_vif->vm_wmac->wm_roaming_state == ROAMING_CONNECTING) {
                            wnet_vif->vm_wmac->wm_roaming_state = ROAMING_STOP;
                            AML_PRINT_LOG_INFO("Roaming connect stop at %s\nRoaming quit!\n", wifi_mac_state_name[prestate]);
                            wifimac->wm_disconnect_code = DISCONNECT_APROAMFAIL;
                        }
                    }
                    break;

                case WIFINET_S_CONNECTED:
                    if (wnet_vif->vm_opmode == WIFINET_M_STA)
                    {
                        wifi_mac_sta_leave(sta, 0);
                        wnet_vif->vm_flags &= ~WIFINET_F_SIBSS;
                        if (wnet_vif->vm_wmac->wm_roaming_state == ROAMING_CONNECTING) {
                            wnet_vif->vm_wmac->wm_roaming_state = ROAMING_STOP;
                            AML_PRINT_LOG_INFO("Roaming connect stop\nRoaming quit!\n");
                            wifimac->wm_disconnect_code = DISCONNECT_APROAMFAIL;
                        }
                    }
                    else if (wnet_vif->vm_opmode == WIFINET_M_IBSS)
                    {
                        wifi_mac_func_to_task(&wnet_vif->vm_sta_tbl,NULL, wnet_vif,0);

                        WIFINET_DPRINTF(AML_LOG_ID_STATE, AML_LOG_LEVEL_INFO,,"sm:ibss");
                        wifi_mac_chk_scan(wnet_vif, WIFINET_SCANCFG_ACTIVE, wnet_vif->vm_des_nssid, wnet_vif->vm_des_ssid);
                    }
                    else
                    {
                        wifi_mac_func_to_task(&wnet_vif->vm_sta_tbl, wifi_mac_sta_disassoc, wnet_vif,1);
                        goto createibss;
                    }
                    break;
                case WIFINET_S_MAX:
                    AML_PRINT_LOG_INFO("need to check twice\n");
                    break;
            }
#ifdef INTER_BEACON
            /* start inter-beacon */
            /* WIFINET_INTER_BEACON_INTERVAL is also inter-beacon flag */
            if (wnet_vif->vm_bcn_intval == WIFINET_INTER_BEACON_INTERVAL)
            {
                wifi_mac_add_work_task(wifimac, wifi_mac_sta_beacon_init_ex,
                    NULL, (SYS_TYPE)wifimac,
                    (SYS_TYPE)wnet_vif->wnet_vif_id, 0,
                    (SYS_TYPE)wnet_vif,
                    (SYS_TYPE)wnet_vif->wnet_vif_replaycounter);
            }
#endif
            break;

        case WIFINET_S_AUTH:
            KASSERT(wnet_vif->vm_opmode == WIFINET_M_STA, ("switch to %s state when operating in mode %u",
                wifi_mac_state_name[nstate], wnet_vif->vm_opmode));

            if (prestate != nstate) {
                wnet_vif->mgmt_pkt_retry_count = 0;
            }
            if (sta->connect_status >= CONNECT_DISCONNECTING) {
                wifi_mac_add_work_task(wnet_vif->vm_wmac, wifi_mac_sm_switch, NULL, (SYS_TYPE)wnet_vif, WIFINET_S_SCAN, 0, 0, 0);
                break;
            }

            switch (prestate) {
                case WIFINET_S_SCAN:
                case WIFINET_S_CONNECTING:
#ifdef AML_WPA3
                    if (sta->sta_authmode == WIFINET_AUTH_SAE) {
#if (KERNEL_VERSION(4, 17, 0) <= CFG80211_VERSION_CODE)
                            wifi_mac_trigger_sae(sta);
#endif

                    } else
#endif //AML_WPA3
                    {
                        mgmt_arg = WIFINET_AUTH_SHARED_REQUEST;
                        wifi_mac_send_mgmt(sta, WIFINET_FC0_SUBTYPE_AUTH, (void *)&mgmt_arg);
                    }
                    break;

                case WIFINET_S_AUTH:
                case WIFINET_S_ASSOC:
#ifdef AML_WPA3
                    if ((prestate == WIFINET_S_ASSOC)
                        && (sta->sta_authmode == WIFINET_AUTH_SAE)
                        && (arg == WIFINET_STATUS_INVALID_PMKID)) {
#if (KERNEL_VERSION(4, 17, 0) <= CFG80211_VERSION_CODE)
                        wifi_mac_trigger_sae(sta);
#endif

                    } else
#endif //AML_WPA3
                    if (arg == WIFINET_FC0_SUBTYPE_AUTH) {
                        mgmt_arg = WIFINET_AUTH_SHARED_CHALLENGE;
                        WIFINET_DPRINTF(AML_LOG_ID_STATE, AML_LOG_LEVEL_INFO, "%d\n", mgmt_arg);
                        wifi_mac_send_mgmt(sta, WIFINET_FC0_SUBTYPE_AUTH, (void *)&mgmt_arg);

                    } else if (arg == WIFINET_FC0_SUBTYPE_DEAUTH) {
                        mgmt_arg = WIFINET_AUTH_SHARED_REQUEST;
                        WIFINET_DPRINTF(AML_LOG_ID_STATE, AML_LOG_LEVEL_INFO, "%d\n", mgmt_arg);
                        wifi_mac_send_mgmt(sta, WIFINET_FC0_SUBTYPE_AUTH, (void *)&mgmt_arg);

                    }

                    break;

                case WIFINET_S_CONNECTED:
                    switch (arg) {
                        case WIFINET_FC0_SUBTYPE_AUTH:
                            mgmt_arg = WIFINET_AUTH_SHARED_CHALLENGE;
                            wifi_mac_send_mgmt(sta, WIFINET_FC0_SUBTYPE_AUTH, (void *)&mgmt_arg);
                            WIFINET_DPRINTF(AML_LOG_ID_STATE, AML_LOG_LEVEL_INFO,"%d\n", mgmt_arg);
                            wnet_vif->vm_state = prestate;
                            break;

                        case WIFINET_FC0_SUBTYPE_DEAUTH:
                            mgmt_arg = WIFINET_REASON_ASSOC_LEAVE;
                            wifi_mac_send_mgmt(sta, WIFINET_FC0_SUBTYPE_DEAUTH, (void *)&mgmt_arg);
                            wifi_mac_sta_leave(sta, 0);
                            wifi_mac_add_work_task(wnet_vif->vm_wmac, wifi_mac_sm_switch, NULL, (SYS_TYPE)wnet_vif, WIFINET_S_SCAN, 0, 0, 0);
                            break;
                    }
                    break;

                default:
                    wifi_mac_add_work_task(wnet_vif->vm_wmac, wifi_mac_sm_switch, NULL, (SYS_TYPE)wnet_vif, WIFINET_S_SCAN, 0, 0, 0);
                    break;
            }
            break;

        case WIFINET_S_ASSOC:
            KASSERT(wnet_vif->vm_opmode == WIFINET_M_STA, ("switch to %s state when operating in mode %u",
                wifi_mac_state_name[nstate], wnet_vif->vm_opmode));

            if (prestate != nstate) {
                wnet_vif->mgmt_pkt_retry_count = 0;
            }

            if (sta->connect_status >= CONNECT_DISCONNECTING) {
                wifi_mac_add_work_task(wnet_vif->vm_wmac, wifi_mac_sm_switch, NULL, (SYS_TYPE)wnet_vif, WIFINET_S_SCAN, 0, 0, 0);
                break;
            }

            switch (prestate)
            {
                case WIFINET_S_AUTH:
                case WIFINET_S_ASSOC:
                    mgmt_arg = 0;
                    wifi_mac_send_mgmt(sta, WIFINET_FC0_SUBTYPE_ASSOC_REQ, (void *)&mgmt_arg);
                    break;

                case WIFINET_S_CONNECTED:
                    mgmt_arg = WIFINET_REASON_ASSOC_LEAVE;
                    wifi_mac_send_mgmt(sta, WIFINET_FC0_SUBTYPE_DISASSOC, (void *)&mgmt_arg);
                    wifi_mac_sta_leave(sta, arg);
                    wifi_mac_add_work_task(wnet_vif->vm_wmac, wifi_mac_sm_switch, NULL, (SYS_TYPE)wnet_vif, WIFINET_S_SCAN, 0, 0, 0);
                    break;

                default:
                    wifi_mac_add_work_task(wnet_vif->vm_wmac, wifi_mac_sm_switch, NULL, (SYS_TYPE)wnet_vif, WIFINET_S_SCAN, 0, 0, 0);
                    AML_PRINT_LOG_INFO("need to check twice\n");
                    break;
            }
            break;

        case WIFINET_S_CONNECTED:
            switch (prestate)
            {
                case WIFINET_S_INIT:
                    if (wnet_vif->vm_opmode == WIFINET_M_MONITOR ||
                        wnet_vif->vm_opmode == WIFINET_M_WDS ||
                        wnet_vif->vm_opmode == WIFINET_M_HOSTAP)
                    {
                        wifi_mac_create_wifi(wnet_vif, wnet_vif->vm_curchan);
                    }
                    break;
                case WIFINET_S_CONNECTING:
                    if (wnet_vif->vm_opmode == WIFINET_M_IBSS)
                    {
                        wifi_mac_notify_nsta_connect(sta, arg == WIFINET_FC0_SUBTYPE_ASSOC_RESP);
                    }
                    break;
                case WIFINET_S_MAX:
                    AML_PRINT_LOG_INFO("need to check twice\n");
                    break;
                default:
                    break;
            }

            if (prestate == WIFINET_S_ASSOC
                && ((wnet_vif->vm_opmode == WIFINET_M_STA) && (wnet_vif->vm_flags_ext & WIFINET_FEXT_SWBMISS)))
            {
                wifi_mac_add_work_task(wifimac,wifi_mac_set_beacon_miss, NULL, (SYS_TYPE)wifimac,
                    (SYS_TYPE)wnet_vif->wnet_vif_id, ENABLE, (SYS_TYPE)wnet_vif, WIFINET_BMISS_THRS/* period, ms*/);
                wifimac->wm_syncbeacon = 1;

                /*set Conect_AP beacon interval for fw */
                wifi_mac_add_work_task(wifimac, wifi_mac_sta_beacon_init_ex, NULL, (SYS_TYPE)wifimac,
                    (SYS_TYPE)wnet_vif->wnet_vif_id, 0, (SYS_TYPE)wnet_vif, (SYS_TYPE)wnet_vif->wnet_vif_replaycounter);

                /*Set keep alive(null data) period for fw*/
                wifi_mac_add_work_task(wifimac,wifi_mac_sta_keep_alive_ex, NULL, (SYS_TYPE)wifimac,
                    (SYS_TYPE)wnet_vif->wnet_vif_id, ENABLE, (SYS_TYPE)wnet_vif, WIFINET_KEEP_ALIVE/* period, ms*/);
                wifimac->wm_lastroaming = jiffies;
                sta->sta_bcn_num_connected = 0;

                wifi_mac_check_special_ap(wnet_vif);

                wnet_vif->vm_phase_flags &= ~PHASE_CONNECTING;

                if (!(wnet_vif->vm_flags & WIFINET_F_PRIVACY)
                    && ((wnet_vif->vm_recovery_state == WIFINET_RECOVERY_VIF_UP) || (wnet_vif->vm_use_static_ip == 1))) {
                    struct vm_wdev_priv *pwdev_priv = wdev_to_priv(wnet_vif->vm_wdev);

                    os_timer_ex_cancel(&pwdev_priv->connect_timeout, CANCEL_SLEEP);

                    if (wnet_vif->vm_recovery_state == WIFINET_RECOVERY_VIF_UP) {
                        sta->connect_status = CONNECT_DHCP_GET_ACK;
                        wifi_mac_vif_restore_end(wnet_vif);
                        wifi_mac_scan_access(wnet_vif);
                    }

                    if (wnet_vif->vm_use_static_ip == 1) {
                        unsigned char *ip_tmp = wnet_vif->vm_static_ipv4;
                        aml_notify_ip(sta, wnet_vif->wnet_vif_id, 1, wnet_vif->vm_static_ipv4);
                        AML_PRINT_LOG_INFO("set static ip: [%d.%d.%d.%d]", ip_tmp[0], ip_tmp[1], ip_tmp[2], ip_tmp[3]);
                    }
                }

                wifi_mac_set_channel_rssi(wifimac, (unsigned char)(wnet_vif->vm_mainsta->sta_avg_bcn_rssi));
                AML_PRINT_LOG_INFO("****************************************************\n");
                AML_PRINT_LOG_INFO("sta connect ok!!! AP CHANNEL:%d, CENTER_CHAN:%d, BW:%d, SSID:%s, BSSID:%02x:%02x:%02x:%02x:%02x:%02x\n",
                    wnet_vif->vm_curchan->chan_pri_num, wifi_mac_Mhz2ieee(wnet_vif->vm_curchan->chan_cfreq1, 0), sta->sta_chbw,
                    wnet_vif->vm_des_ssid[0].ssid, wnet_vif->vm_des_bssid[0], wnet_vif->vm_des_bssid[1], wnet_vif->vm_des_bssid[2],
                    wnet_vif->vm_des_bssid[3], wnet_vif->vm_des_bssid[4], wnet_vif->vm_des_bssid[5]);
                AML_PRINT_LOG_INFO("****************************************************\n");
                if (wnet_vif->vm_state  == WIFINET_S_CONNECTED && wifimac->wm_ant_rssi_measure.timer_handler)
                {
                    wifi_mac_reset_ant_measure();
                    os_timer_ex_start(&wifimac->wm_ant_rssi_measure);
                    printk("wm_ant_rssi_measure, wifi_mac_get_cur_ant:%d\n", wifi_mac_get_cur_ant()+1);
                }
                wifi_mac_show_per_info();
                g_auto_gain_base = 0;
                wifimac->wm_disconnect_code = DISCONNECT_GENERIC;
                sta->sta_is_adding_key = false;

                if (wifimac->cca_thrd_cfg) {
                    wifi_mac_add_work_task(wifimac,cca_thrd_cfg_change_task,NULL,(SYS_TYPE)wnet_vif, ENABLE, 0, 0, 0);
                }
                wifi_mac_sta_auth(sta);
            } else {
                if ((wnet_vif->vm_opmode == WIFINET_M_STA) && (prestate != WIFINET_S_CONNECTED)) {
                    AML_PRINT(AML_LOG_ID_CONNECT,AML_LOG_LEVEL_WARN, "vid[%d] unexpect switch to scan\n",wnet_vif->wnet_vif_id);
                    wifi_mac_add_work_task(wnet_vif->vm_wmac, wifi_mac_sm_switch, NULL, (SYS_TYPE)wnet_vif, WIFINET_S_SCAN, 0, 0, 0);
                    ret = -1;
                }
            }

            WIFI_ALPHA_LOCK(wifimac);
            if (wifimac->wm_alpha_set_forbid & CONNECT_FORBIDDEN) {
                wifi_mac_run_delayed_country_switch(wifimac);
                wifimac->wm_alpha_set_forbid &= ~CONNECT_FORBIDDEN;
            }
            WIFI_ALPHA_UNLOCK(wifimac);

            break;
        case WIFINET_S_MAX:
            AML_PRINT_LOG_INFO("-------------error: need to check twice-------------------\n");
            break;

        default:
            break;
    }

    if (nstate == WIFINET_S_CONNECTED && prestate != WIFINET_S_CONNECTED)
    {
        wifimac->wm_runningmask |= BIT(wnet_vif->wnet_vif_id);
        wifimac->wm_nrunning++;

        wifi_mac_set_scan_time(wnet_vif);
    #ifdef CONFIG_CONCURRENT_MODE
        if ((wifimac->wm_nrunning > 1) && !concurrent_check_is_vmac_same_pri_channel(wifimac)) {
            if ((wnet_vif->vm_p2p_support == 1) || (wnet_vif->vm_opmode == WIFINET_M_HOSTAP)) {
                wifimac->wm_vsdb_slot = CONCURRENT_SLOT_P2P;
            } else {
                wifimac->wm_vsdb_slot = CONCURRENT_SLOT_STA;
            }
#if 0
            if (IS_APSTA_CONCURRENT(aml_wifi_get_con_mode()) && concurrent_check_vmac_is_AP(wifimac)) {
                /*softap and sta concurrent as scc, no need vsdb*/
                wifimac->wm_vsdb_slot = CONCURRENT_SLOT_NONE;
            } else {
                wifi_mac_add_work_task(wifimac, wifi_mac_set_vsdb, NULL, (SYS_TYPE)wifimac, 0, ENABLE, (SYS_TYPE)wnet_vif, 0);
            }
#endif
            if (IS_APSTA_CONCURRENT(aml_wifi_get_con_mode()) && concurrent_check_vmac_is_AP(wifimac)) {
                struct wifi_channel *main_vmac_chan = wifi_mac_get_main_vmac_channel(wifimac);
                channel_switch_announce_trigger(wifimac, main_vmac_chan->chan_pri_num, main_vmac_chan->chan_bw, wifi_mac_Mhz2ieee(main_vmac_chan->chan_cfreq1, 0));
            }
            wifi_mac_add_work_task(wifimac, wifi_mac_set_vsdb, NULL, (SYS_TYPE)wifimac, 0, ENABLE, (SYS_TYPE)wnet_vif, 0);
        }
    #endif
    }
    else if (prestate == WIFINET_S_CONNECTED && nstate != WIFINET_S_CONNECTED)
    {
        wifi_mac_set_scan_time(wnet_vif);
    #ifdef CONFIG_CONCURRENT_MODE
        if ((wifimac->wm_nrunning == 1) && (wifimac->wm_vsdb_slot != CONCURRENT_SLOT_NONE)) {
            wifimac->wm_vsdb_slot = CONCURRENT_SLOT_NONE;
            wifimac->wm_vsdb_flags = 0;
            wifimac->vsdb_mode_set_noa_enable = 0;
            wifi_mac_restore_wnet_vif_channel_task(wnet_vif);

            wifi_mac_add_work_task(wifimac, wifi_mac_set_vsdb, NULL,(SYS_TYPE)wifimac, 0, DISABLE, (SYS_TYPE)wnet_vif, 0);
        }
    #endif
    }

    return ret;
}

int wifi_mac_top_sm(struct wlan_net_vif *wnet_vif,
        enum wifi_mac_state nstate, int arg)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    enum wifi_mac_opmode opmode = wnet_vif->vm_opmode;
    enum wifi_mac_state prestate = wnet_vif->vm_state;
    int rc = 0;

    if (aml_wifi_is_enable_rf_test())
        return 0;

    WIFINET_VMACS_LOCK(wifimac);

    switch (nstate) {
        case WIFINET_S_SCAN:
            rc = wifi_mac_sub_sm(wnet_vif, nstate, arg);
            break;

        case WIFINET_S_CONNECTED:
            if (opmode == WIFINET_M_HOSTAP) {
                if (wnet_vif->vm_recovery_state == WIFINET_RECOVERY_VIF_UP) {
                    wifi_mac_add_work_task(wifimac, wifi_mac_mkey_restore_task, NULL, (SYS_TYPE)wnet_vif, 0, 0, 0, 0);
                    wifi_mac_vif_restore_end(wnet_vif);
                    wifi_mac_scan_access(wnet_vif);
                }
                wifimac->wm_syncbeacon = 0;
                wifi_mac_add_work_task(wifimac, wifi_mac_beacon_alloc_ex, NULL, (SYS_TYPE)wifimac,
                    (SYS_TYPE)wnet_vif->wnet_vif_id,0, (SYS_TYPE)wnet_vif, (SYS_TYPE)wnet_vif->wnet_vif_replaycounter);

                AML_PRINT_LOG_INFO("****************************************************\n");
                AML_PRINT_LOG_INFO("start bss ok!!! AP CHANNEL:%d, BW:%d, SSID:%s, BSSID:%02x:%02x:%02x:%02x:%02x:%02x\n",
                    wnet_vif->vm_curchan->chan_pri_num, wnet_vif->vm_curchan->chan_bw, wnet_vif->vm_des_ssid[0].ssid,
                    wnet_vif->vm_des_bssid[0], wnet_vif->vm_des_bssid[1], wnet_vif->vm_des_bssid[2],
                    wnet_vif->vm_des_bssid[3], wnet_vif->vm_des_bssid[4], wnet_vif->vm_des_bssid[5]);
                AML_PRINT_LOG_INFO("****************************************************\n");

            }
            rc = wifi_mac_sub_sm(wnet_vif, nstate, arg);

            AML_PRINT_LOG_INFO("wm_running=%d, wnet_vif_mode %d vm_bandwidth:%d\n",
                wifimac->wm_nrunning, wnet_vif->vm_mac_mode, wnet_vif->vm_bandwidth);
            break;

        case WIFINET_S_INIT:
            rc = wifi_mac_sub_sm(wnet_vif, nstate, arg);
            if (opmode == WIFINET_M_HOSTAP) {
                wifi_mac_add_work_task(wnet_vif->vm_wmac, wifi_mac_interface_down_ex, NULL, (SYS_TYPE)wnet_vif, 0, 0, 0, 0);
                wifimac->drv_priv->drv_ops.down_interface(wifimac->drv_priv, wnet_vif->wnet_vif_id);
            }
            break;

        case WIFINET_S_CONNECTING:
            if ((opmode != WIFINET_M_STA) && (opmode != WIFINET_M_IBSS)) {
                WIFINET_DPRINTF(AML_LOG_ID_STATE, AML_LOG_LEVEL_WARN,"\n");
                rc= -EINVAL;
                goto __unlock;
            }
            rc = wifi_mac_sub_sm(wnet_vif, nstate, arg);
            break;

        default:
            rc = wifi_mac_sub_sm(wnet_vif, nstate, arg);
            break;
    }

__unlock:
    WIFINET_VMACS_UNLOCK(wifimac);

    if ((nstate == WIFINET_S_CONNECTED) && (prestate != WIFINET_S_CONNECTED)) {
        wifi_mac_pwrsave_wnet_vif_connect(wnet_vif);

    } else if ((prestate == WIFINET_S_CONNECTED) && (nstate != WIFINET_S_CONNECTED)) {
        wifi_mac_pwrsave_wnet_vif_disconnect(wnet_vif);
    }

    if (VM_WIFI_CONNECT_STATE(nstate)  && !VM_WIFI_CONNECT_STATE(prestate)) {
        wifi_mac_connect_start(wifimac);
        if ((opmode == WIFINET_M_STA) || (opmode == WIFINET_M_P2P_CLIENT)) {
            AML_PRINT_LOG_INFO(" connect_start \n");
        }

    } else if (VM_WIFI_CONNECT_STATE(prestate)  && !VM_WIFI_CONNECT_STATE(nstate)) {
        wifi_mac_connect_end(wifimac);
        AML_PRINT_LOG_INFO("connect end =%d, wnet_vif_mode %d\n", wifimac->wm_nrunning, wnet_vif->vm_mac_mode);
    }

    return rc;
}


void wifi_mac_com_vattach(struct wlan_net_vif *wnet_vif)
{
    wnet_vif->vm_rtsthreshold = WIFINET_RTS_MAX;
    wnet_vif->vm_fragthreshold = WIFINET_FRAGMT_THRESHOLD_MAX;

    os_timer_ex_initialize(&wnet_vif->vm_mgtsend, WIFINET_TRANS_WAIT, wifi_mac_mgmt_tx_timeout, wnet_vif);
    os_timer_ex_initialize(&wnet_vif->vm_sm_switch, WIFINET_TRANS_WAIT, wifi_mac_sm_switch_timeout, wnet_vif);
    os_timer_ex_initialize(&wnet_vif->vm_actsend, WIFINET_TRANS_WAIT, wifi_mac_act_tx_timeout, wnet_vif);
    os_timer_ex_initialize(&wnet_vif->vm_roc_timer, WIFINET_DPP_WAIT, wifi_mac_RocTimeout, wnet_vif);

    /*set tx rate(11n mcs0 or 11b/g rate) for mgmt or
      multicast frames, set default max retry times*/
    wifi_mac_rate_vattach(wnet_vif);
}

void wifi_mac_com_vdetach(struct wlan_net_vif *wnet_vif)
{
    os_timer_ex_del(&wnet_vif->vm_mgtsend, CANCEL_SLEEP);
    os_timer_ex_del(&wnet_vif->vm_sm_switch, CANCEL_SLEEP);
    os_timer_ex_del(&wnet_vif->vm_actsend, CANCEL_SLEEP);
    os_timer_ex_del(&wnet_vif->vm_roc_timer, CANCEL_SLEEP);

    if (wnet_vif->vm_aclop != NULL)
        wnet_vif->vm_aclop->iac_detach(wnet_vif);
}

const char *wifi_mac_opmode_name[] =
{
    "IBSS",
    "STA",
    "HOSTAP",
    "MONITOR",
    "WDS",
    "CLIENT ",
    "P2P_GO",
    "P2P_DEV",
};

static int
__wifi_mac_ioctrl(struct net_device *dev, void __user *data, int cmd)
{
    struct wlan_net_vif *wnet_vif = netdev_priv(dev);

    AML_PRINT(AML_LOG_ID_IOCTL, AML_LOG_LEVEL_DEBUG, "cmd 0x%x\n", cmd);
    switch (cmd) {
        case SIOCANDROID_PRIV:
            //return wifi_mac_stop(vmac->vm_dev);
            return  aml_android_priv_cmd(wnet_vif, data, cmd);

        case SIOCG80211STATS:
            return copy_to_user(data, &wnet_vif->vif_sts, sizeof (wnet_vif->vif_sts)) ? -EFAULT : 0;

        case SIOC80211IFDESTROY:
            if (!capable(CAP_NET_ADMIN))
                return -EPERM;

            wifi_mac_stop(wnet_vif->vm_ndev);
            wifi_mac_vmac_detach(wnet_vif);
            wifi_mac_delt_vmac_id(wnet_vif->vm_wmac,wnet_vif->wnet_vif_id);
            vm_wlan_net_vif_unregister(wnet_vif);
            return 0;
        default:
            break;
    }
    return -EOPNOTSUPP;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
int wifi_mac_ioctrl_wrapper(struct net_device *dev, struct ifreq *ifr,
    void __user *data, int cmd)
{
    return __wifi_mac_ioctrl(dev,data,cmd);
}
#else

int wifi_mac_ioctrl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
    return __wifi_mac_ioctrl(dev,ifr->ifr_data,cmd);
}

#endif

static int
wifi_set_mac_address(struct net_device *dev, void *addr)
{
    int ret = 0;
    struct wlan_net_vif *wnet_vif = netdev_priv(dev);
    struct sockaddr *sa = (struct sockaddr *)addr;
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;

    if (wnet_vif->vm_state > WIFINET_S_CONNECTING) {
        //other state need to reup interface
        return ret;
    }

    memcpy(wnet_vif->vm_myaddr, sa->sa_data, MAC_ADDR_LEN);
    if (wnet_vif->vm_mainsta != NULL) {
        memcpy(wnet_vif->vm_mainsta->sta_macaddr, sa->sa_data, MAC_ADDR_LEN);
    }
    dev_addr_mod(dev, 0, wnet_vif->vm_myaddr, WIFINET_ADDR_LEN);
    wnet_vif->vm_wdev->wiphy->addresses = (struct mac_address *)(wnet_vif->vm_myaddr);

    wifimac->drv_priv->drv_ops.set_macaddr(wifimac->drv_priv, wnet_vif->wnet_vif_id, wnet_vif->vm_myaddr);
    AML_PRINT_LOG_INFO("change mac_addr to:%02x:%02x:%02x:%02x:%02x:%02x\n", wnet_vif->vm_myaddr[0],
        wnet_vif->vm_myaddr[1], wnet_vif->vm_myaddr[2], wnet_vif->vm_myaddr[3], wnet_vif->vm_myaddr[4], wnet_vif->vm_myaddr[5]);
    return ret;
}

void wnet_vif_vht_cap_init( struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;

    KASSERT(wnet_vif,( "vht cap init should not be null\n"));

    wifimac->wm_flags_ext2 = WIFINET_VHTCAP_MAX_MPDU_LEN_7991 \
                | WIFINET_VHTCAP_RX_LDPC \
                /* can't process stbc pkt, just remove stbc */
                | WIFINET_VHTCAP_RX_STBC \
                | WIFINET_VHTCAP_MAX_AMPDU_LEN_EXP;
    wifimac->wm_flags |=WIFINET_F_LDPC;

    wnet_vif->vm_sgi = wifi_mac_com_has_extflag(wifimac, WIFINET_FEXT_SHORTGI_ENABLE);
    wnet_vif->vm_ldpc = GET_VHT_CAP_RX_LPDC(wifimac->wm_flags_ext2);
    wnet_vif->vm_tx_stbc = GET_VHT_CAP_TX_STBC(wifimac->wm_flags_ext2);
    wnet_vif->vm_rx_stbc = GET_VHT_CAP_RX_STBC(wifimac->wm_flags_ext2);

    AML_PRINT_LOG_INFO("vht cap init: sgi 0x%x, ldpc 0x%x, tx_stbc 0x%x, rx_stbc 0x%x \n",
        wnet_vif->vm_sgi, wnet_vif->vm_ldpc, wnet_vif->vm_tx_stbc, wnet_vif->vm_rx_stbc);

#ifdef SU_BF
    wifimac->wm_flags_ext2 |= WIFINET_VHTCAP_SU_BFMEE;
#endif
#ifdef MU_BF
    wifimac->wm_flags_ext2 |= WIFINET_VHTCAP_MU_BFMEE;
#endif
}

void wifi_mac_set_reg_val(unsigned int reg_addr, enum wifi_mac_bwc_width bw)
{
    unsigned int reg_val = 0;
    struct hw_interface* hw_if = hif_get_hw_interface();

    if (PHY_AGC_BUSY_FSM == reg_addr) {
        reg_val = hw_if->hif_ops.hi_read_word(PHY_AGC_BUSY_FSM);
        hw_if->hif_ops.hi_write_word(PHY_AGC_BUSY_FSM, reg_val | 0xf0);

    } else if (MAC_REG_BASE == reg_addr) {
        reg_val = hw_if->hif_ops.hi_read_word(MAC_REG_BASE);
        hw_if->hif_ops.hi_write_word(MAC_REG_BASE, reg_val & (~(1<<18)));

    } else if (MAC_RXPKT_CONTROL45 == reg_addr) {
        reg_val = hw_if->hif_ops.hi_read_word(MAC_RXPKT_CONTROL45);
        reg_val &= ~(1<<31);
        reg_val |= 0x800848;//bit23 & bit11 & bit6 & bit3 set to 1
        reg_val &= ~(1 << 17 | 1 << 18);
        hw_if->hif_ops.hi_write_word(MAC_RXPKT_CONTROL45, reg_val | (bw << 17));//bit17 & bit18 set ap's bw

    } else {
        AML_PRINT_LOG_INFO("No valid reg addr\n");
    }
}
static void wnet_vif_cap_init(struct wifi_mac *wifimac,struct wlan_net_vif *wnet_vif)
{
    wnet_vif->vm_caps = wifimac->wm_caps &
        ~(WIFINET_C_IBSS | WIFINET_C_HOSTAP  |WIFINET_C_MONITOR);

    wnet_vif->vm_htcap = WIFINET_HTCAP_SUPPORTCBW40 \
        /* can't process stbc pkt, just remove stbc */
        | WIFINET_HTCAP_C_RXSTBC_1SS \
        | WIFINET_HTCAP_USEDSSSCCK_40M \
        | WIFINET_HTCAP_DISABLE_SMPS \
        | WIFINET_HTCAP_C_MAXAMSDUSIZE;

    /*Init bandwidth and second channel offset. */
    wnet_vif->vm_curchan = WIFINET_CHAN_ERR;
    wnet_vif->scnd_chn_offset = WIFINET_HTINFO_EXTOFFSET_NA;
    wnet_vif->vm_bandwidth = WIFINET_BWC_WIDTH80;

}
int wifi_mac_setup(struct wifi_mac *wifimac,
                    struct wlan_net_vif *wnet_vif, int opmode)
{
    wnet_vif->vm_wmac = wifimac;
    wnet_vif->vm_flags = wifimac->wm_flags;
    wnet_vif->vm_flags_ext = wifimac->wm_flags_ext;

    AML_PRINT_LOG_INFO("<running> opmode=%d\n", opmode);
    wnet_vif_cap_init(wifimac,wnet_vif);
    switch (opmode)
    {
        case WIFINET_M_STA:
            /* set flag to start beacon miss timer for sta */
            wnet_vif->vm_flags_ext |= WIFINET_FEXT_SWBMISS;
            break;
        case WIFINET_M_IBSS:
            wnet_vif->vm_caps |= WIFINET_C_IBSS;
            wnet_vif->vm_flags &= ~WIFINET_F_RDG;
            break;
        case WIFINET_M_HOSTAP:
            wnet_vif->vm_caps |= WIFINET_C_HOSTAP;
            break;
        case WIFINET_M_MONITOR:
            wnet_vif->vm_caps |= WIFINET_C_MONITOR;
            wnet_vif->vm_flags &= ~WIFINET_F_RDG;
            break;
        case WIFINET_M_WDS:
            wnet_vif->vm_caps |= WIFINET_C_WDS;
            wnet_vif->vm_flags &= ~WIFINET_F_RDG;
            wnet_vif->vm_flags_ext |= WIFINET_FEXT_WDS;
            break;
    }
    wnet_vif->vm_opmode = opmode;
    wnet_vif->vm_chanchange_count = 0;
    wnet_vif->vm_dtim_period = WIFINET_DTIM_DEFAULT;
    wnet_vif->mgmt_pkt_retry_count = 0;
    wnet_vif->txtp_stat.tcp_tx_payload_total = 0;
    wnet_vif->txtp_stat.vm_tx_speed = 0;
    wnet_vif->txtp_stat.udp_tx_payload_total = 0;
    wnet_vif->txtp_stat.vm_udp_tx_speed = 0;
    wnet_vif->rxtp_stat.udp_rx_payload_total = 0;
    wnet_vif->rxtp_stat.vm_udp_rx_speed = 0;

    wifi_mac_security_vattach(wnet_vif);
    wifi_mac_pmkid_vattach(wnet_vif);
    wifi_mac_sta_vattach(wnet_vif);
    /*init power save para&state and timer for ap&sta and p2p*/
    wifi_mac_pwrsave_vattach(wnet_vif);
    wifi_mac_com_vattach(wnet_vif);
    /*set default time for scan operation */
    wifi_mac_scan_vattach(wnet_vif);
    wnet_vif_vht_cap_init(wnet_vif);

    /* Set the VHT  rate information
    *  11ac spec states it is mandatory to support
    *  MCS 0-7 and NSS=1 */
    wifi_mac_vht_rate_init(wnet_vif,VHT_MCSMAP_NSS1_MCS0_9,
        0, VHT_MCSMAP_NSS1_MCS0_9 );
    return 1;
}



int vm_wlan_net_vif_setup_forchvif(struct wifi_mac *wifimac,
        struct wlan_net_vif *wnet_vif, const char *name,  int opmode)
{
    struct net_device *dev = wnet_vif->vm_ndev;
    if (name != NULL)
        strncpy(dev->name, name, sizeof(dev->name) - 1);
    wnet_vif->vm_caps = wifimac->wm_caps;
    wnet_vif->vm_flags_ext &= ~WIFINET_FEXT_SWBMISS;
    wnet_vif->vm_flags = wifimac->wm_flags;

    AML_PRINT_LOG_INFO("<running> opmode=%d\n", opmode);
    switch (opmode)
    {
        case WIFINET_M_STA:
            /* set flag to start beacon miss timer for sta */
            wnet_vif_cap_init(wifimac,wnet_vif);
            wnet_vif->vm_flags_ext |= WIFINET_FEXT_SWBMISS;
            break;
        case WIFINET_M_IBSS:
            wnet_vif->vm_flags &= ~WIFINET_F_RDG;
            break;
        case WIFINET_M_HOSTAP:
            break;
        case WIFINET_M_MONITOR:
            wnet_vif->vm_flags &= ~WIFINET_F_RDG;
            break;
        case WIFINET_M_WDS:
            wnet_vif->vm_flags &= ~WIFINET_F_RDG;
            wnet_vif->vm_flags_ext |= WIFINET_FEXT_WDS;
            break;
    }
    wnet_vif->vm_opmode = opmode;
    wnet_vif->vm_chanchange_count = 0;
    wifi_mac_sta_vattach(wnet_vif);
    wifi_mac_pwrsave_vattach(wnet_vif);
    wifi_mac_com_vattach(wnet_vif);
    wifi_mac_scan_vattach(wnet_vif);
    wnet_vif_vht_cap_init(wnet_vif);
    return 1;
}

void aml_free_netdev(struct net_device * ndev)
{
    AML_PRINT_LOG_INFO("\n");
    free_netdev(ndev);
}

int
vm_wlan_net_vif_register(struct wlan_net_vif *wnet_vif, char* name)
{
    /* wireless net device */
    struct net_device *dev = wnet_vif->vm_ndev;
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;

    if (name != NULL)
        strncpy(dev->name, name, sizeof(dev->name) - 1);

    /*set net device operations, refer to 'wifi_mac_netdev_ops' for details.
        For example, if wlan0 up/down, tcp/ip_xmit_start ...*/
    dev->netdev_ops = &wifi_mac_netdev_ops;
    dev->hard_header_len = DEFAULT_HARD_HDR_LEN;
#ifdef CONFIG_WIRELESS_EXT
    dev->wireless_handlers = &w1_iw_handle;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
    dev->priv_destructor = aml_free_netdev;
#else
    dev->destructor = aml_free_netdev;
#endif

    if(wifimac->wm_caps & WIFINET_C_HWCSUM)
    {
        netdev_setcsum(dev,1);
    }
    else
    {
        netdev_setcsum(dev,0);
    }
    /*register a wireless net dev for cfg80211 */
    if (wifi_mac_alloc_wdev(wnet_vif, vm_cfg80211_get_parent_dev()) < 0)
        AML_PRINT_LOG_ERR("ERROR alloc wdev\n");

    AML_PRINT(AML_LOG_ID_CONNECT, AML_LOG_LEVEL_DEBUG, "\n");
    wifi_mac_pwrsave_latevattach(wnet_vif);

    WIFINET_QLOCK(wifimac);
    list_add_tail(&wnet_vif->vm_next, &wifimac->wm_wnet_vifs);
    wifimac->wnet_vif_num++;
    WIFINET_QUNLOCK(wifimac);

    dev_addr_mod(dev, 0, wnet_vif->vm_myaddr, WIFINET_ADDR_LEN);
    AML_PRINT_LOG_INFO("<running>\n");
    if (register_netdev(dev))
    {
        AML_PRINT_LOG_ERR("ERROR::%s: unable to register device\n", dev->name);
        return 0;
    }
    return 1;
}

void
vm_wlan_net_vif_detach(struct wlan_net_vif *wnet_vif)
{
    wifi_mac_com_vdetach(wnet_vif);
    wifi_mac_security_vdetach(wnet_vif);
    wifi_mac_pmkid_detach(wnet_vif);
    wifi_mac_pwrsave_vdetach(wnet_vif);

    wifi_mac_sta_vdetach(wnet_vif);

#ifdef CONFIG_P2P
    vm_p2p_dettach(wdev_to_priv(wnet_vif->vm_wdev));
#endif //CONFIG_P2P
}

void
vm_wlan_net_vif_unregister(struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct net_device *dev = wnet_vif->vm_ndev;
    struct wireless_dev *wdev = wnet_vif->vm_wdev;
    unsigned char vid = wnet_vif->wnet_vif_id;

    WIFINET_QLOCK(wifimac);
    list_del_init(&wnet_vif->vm_next);
    wifimac->wnet_vif_num--;
    WIFINET_QUNLOCK(wifimac);

    unregister_netdev(dev);
    vm_wdev_free(wdev, vid);
}

static struct net_device_stats *
wifi_mac_getstats(struct net_device *dev)
{
    struct wlan_net_vif *wnet_vif = netdev_priv(dev);
    return &wnet_vif->vm_devstats;
}


static int
wifi_mac_change_mtu(struct net_device *dev, int mtu)
{
    if (!(WIFINET_MTU_MIN < mtu && mtu <= WIFINET_MTU_MAX))
        return -EINVAL;
    dev->mtu = mtu;
    return 0;
}

void
wifi_mac_build_country_ie_2G(struct wifi_mac *wifimac)
{
    struct wifi_channel *c;
    int channel_num = 0;
    int first_channel = 0;
    int max_power = 0;
    int i = 0 ;

    WIFI_CHANNEL_LOCK(wifimac);
    for (i = 0; i < wifimac->wm_nchans; i++) {
        c = &wifimac->wm_channels[i];

        if (WIFINET_IS_CHAN_2GHZ(c) && (c->chan_bw == WIFINET_BWC_WIDTH20)) {
            if (!first_channel) {
                first_channel = c->chan_pri_num;
                max_power = c->chan_maxpower;
            }
            channel_num ++;
        }
    }
    WIFI_CHANNEL_UNLOCK(wifimac);

    wifimac->wm_countryinfo.country_triplet[0] = first_channel;
    wifimac->wm_countryinfo.country_triplet[1] = channel_num;
    wifimac->wm_countryinfo.country_triplet[2] = max_power;
    wifimac->wm_countryinfo.country_len += 3;
}

void
wifi_mac_build_country_ie_5G(struct wifi_mac *wifimac)
{
    struct wifi_channel *c;
    int i = 0 ;
    int triplet_index = 0;

    WIFI_CHANNEL_LOCK(wifimac);
    for (i = 0; i < wifimac->wm_nchans; i++) {
        c = &wifimac->wm_channels[i];

        if (WIFINET_IS_CHAN_5GHZ(c) && (c->chan_bw == WIFINET_BWC_WIDTH20)) {
            wifimac->wm_countryinfo.country_triplet[triplet_index++] = c->chan_pri_num;
            wifimac->wm_countryinfo.country_triplet[triplet_index++] = 1;
            wifimac->wm_countryinfo.country_triplet[triplet_index++] = c->chan_maxpower;
            wifimac->wm_countryinfo.country_len += 3;
        }
    }
    WIFI_CHANNEL_UNLOCK(wifimac);
}

extern bool g_DFS_on;
void
wifi_mac_build_country_ie_2G_and_5G(struct wifi_mac *wifimac)
{
    struct wifi_channel *c;
    int i = 0 ;
    int channel_num = 0;
    int first_channel = 0;
    int max_power = 0;
    int triplet_index = 0;

    WIFI_CHANNEL_LOCK(wifimac);
    for (i = 0; i < wifimac->wm_nchans; i++) {
        c = &wifimac->wm_channels[i];

        if (WIFINET_IS_CHAN_2GHZ(c) && (c->chan_bw == WIFINET_BWC_WIDTH20)) {
            if (!first_channel) {
                first_channel = c->chan_pri_num;
                max_power = c->chan_maxpower;
            }
            if (max_power < c->chan_maxpower) {
                max_power = c->chan_maxpower;
            }
            channel_num ++;
        }
    }
    if (channel_num != 0) {
        wifimac->wm_countryinfo.country_triplet[triplet_index++] = first_channel;
        wifimac->wm_countryinfo.country_triplet[triplet_index++] = channel_num;
        wifimac->wm_countryinfo.country_triplet[triplet_index++] = max_power;
        wifimac->wm_countryinfo.country_len += 4;
    }
    for (i = 0; i < wifimac->wm_nchans; i++) {
        c = &wifimac->wm_channels[i];

        if (WIFINET_IS_CHAN_5GHZ(c) && (c->chan_bw == WIFINET_BWC_WIDTH20)) {
            if (g_DFS_on || !(c->chan_flags & WIFINET_CHAN_DFS)) {
                wifimac->wm_countryinfo.country_triplet[triplet_index++] = c->chan_pri_num;
                wifimac->wm_countryinfo.country_triplet[triplet_index++] = 1;
                wifimac->wm_countryinfo.country_triplet[triplet_index++] = c->chan_maxpower;
                wifimac->wm_countryinfo.country_len += 3;
            }
        }
    }
    WIFI_CHANNEL_UNLOCK(wifimac);
}


void
wifi_mac_build_country_ie(struct wlan_net_vif * wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct _wifi_mac_country_iso country;
    memset(&wifimac->wm_countryinfo, 0, sizeof(wifimac->wm_countryinfo));
    wifimac->wm_countryinfo.country_id = WIFINET_ELEMID_COUNTRY;
    wifimac->wm_countryinfo.country_len = 0;

    wifi_mac_get_country(wifimac,&country);

    wifimac->wm_countryinfo.country_str[0] = country.iso[0];
    wifimac->wm_countryinfo.country_str[1] = country.iso[1];
    wifimac->wm_countryinfo.country_str[2] = 0x20; //environment: Any
    wifimac->wm_countryinfo.country_len += 3;

    wifi_mac_build_country_ie_2G_and_5G(wifimac);
    /*if (WIFINET_IS_CHAN_2GHZ(wnet_vif->vm_curchan)) {
        wifi_mac_build_country_ie_2G(wifimac);

    } else {
         wifi_mac_build_country_ie_5G(wifimac);
    }*/

    AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_DEBUG,"INFO::Country ie is %c%c%c\n", wifimac->wm_countryinfo.country_str[0],
        wifimac->wm_countryinfo.country_str[1], wifimac->wm_countryinfo.country_str[2]);
}

int wifi_mac_get_new_vmac_id(struct wifi_mac *wifimac)
{
    int id = 0;

    id = find_first_zero_bit(&wifimac->wm_wnet_vif_mask, WIFI_MAX_VID);
    if (id >= WIFI_MAX_VID)
    {
        id= -1;
        return id;
    }
    set_bit(id, (unsigned long *)&wifimac->wm_wnet_vif_mask);

    return id;
}

void wifi_mac_delt_vmac_id(struct wifi_mac *wifimac,int vid)
{
    ASSERT(vid < WIFI_MAX_VID);
    if (test_and_clear_bit(vid, (unsigned long *)&wifimac->wm_wnet_vif_mask))
    {
        AML_PRINT_LOG_INFO("<WARNING> vid %d free success\n", vid);
    }
    else
    {
        AML_PRINT_LOG_INFO("<WARNING> vid %d has been free\n", vid);
    }
}

void wifi_mac_set_fwlog_ex(SYS_TYPE param1,SYS_TYPE param2,
    SYS_TYPE param3,SYS_TYPE param4,SYS_TYPE param5)
{
    struct hal_private *halPriv = hal_get_priv();

    halPriv->hal_ops.hal_set_fwlog_cmd(1);
    halPriv->hal_ops.hal_set_fwlog_cmd(4);

    return;
}

void wifi_mac_run_delayed_country_switch(struct wifi_mac *wifimac)
{
    if (wifimac->wm_alpha_pending) {
        AML_PRINT_LOG_INFO("apply delayed country: %s\n", wifimac->wm_alpha_target);
        wifi_mac_set_country_regdom(wifimac->wm_alpha_target);
        wifimac->wm_alpha_pending = 0;
    }
}

void wifi_mac_channel_switch_complete(struct wlan_net_vif *wnet_vif)
{
    unsigned char vid = wnet_vif->wnet_vif_id;
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;
    struct hal_private *halPriv = hal_get_priv();

    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "vid:%d, ss->scan_StateFlags:%08x\n", vid, ss->scan_StateFlags);

    if (!(wnet_vif->vm_pstxqueue_flags & WIFINET_PSQUEUE_NOA) ) {
        wifimac->drv_priv->drv_ops.drv_hal_tx_frm_pause(wifimac->drv_priv, 0);
    }

    if ((halPriv->dpd_process_flag & DPD_CHAN_SWITCH) != 0) {

        if (wifimac->drv_priv->hal_priv->g_get_fw_log == true) {
            AML_PRINT_LOG_INFO("enable fwlog get again\n");
            wifi_mac_add_work_task(wifimac, wifi_mac_set_fwlog_ex, NULL,
                    (SYS_TYPE)0, (SYS_TYPE)0, 0, 0, (SYS_TYPE)0);
        }
        halPriv->dpd_process_flag &= ~DPD_CHAN_SWITCH;
    }

    if (wifimac->wm_vsdb_flags & CONCURRENT_SWITCH_TO_STA_CHANNEL) {
        /*switch to sta channel,we need notify ap*/
        wifimac->wm_vsdb_flags &= ~CONCURRENT_SWITCH_TO_STA_CHANNEL;
        wifi_mac_scan_notify_leave_or_back(wifimac->drv_priv->drv_wnet_vif_table[NET80211_MAIN_VMAC],0);
        AML_PRINT_LOG_INFO("channel switch complete and sta notify ap back done\n");
    }
    WIFI_SCAN_LOCK(ss);
    if (ss->scan_StateFlags & SCANSTATE_F_WAIT_CHANNEL_SWITCH) {
        ss->scan_StateFlags &= ~SCANSTATE_F_WAIT_CHANNEL_SWITCH;
        ss->scan_StateFlags |= SCANSTATE_F_CHANNEL_SWITCH_COMPLETE;
        WIFI_SCAN_UNLOCK(ss);
        wifi_mac_scan_timeout_ex(wifimac->wm_scan);
        WIFI_SCAN_LOCK(ss);
    }
    WIFI_SCAN_UNLOCK(ss);

    return;
}

void wifi_mac_set_noa(struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    unsigned long long tsf;
    unsigned int ltsf;
    unsigned int next_tbtt;

    if (wifimac->wm_nrunning == 1) {
        struct wlan_net_vif *connect_wnet = wifi_mac_running_wnet_vif(wifimac);
        struct p2p_noa noa = {0};

        if (connect_wnet == NULL) {
            return;
        }

        if (connect_wnet->vm_opmode == WIFINET_M_HOSTAP &&
            wnet_vif->vm_p2p->p2p_negotiation_state == NET80211_P2P_STATE_GO_COMPLETE &&
            softap_get_sta_num(wnet_vif)) {
            tsf = wifimac->drv_priv->drv_ops.drv_hal_get_tsf(wifimac->drv_priv, wnet_vif->wnet_vif_id);
            ltsf = (unsigned int)tsf;
            next_tbtt = roundup(ltsf, wnet_vif->vm_bcn_intval*TU_DURATION);
            noa.start = next_tbtt + WIFINET_SCAN_MOTHER_CHANNEL_TIME*TU_DURATION;
            noa.count = 255;
            noa.duration = 25*TU_DURATION;
            noa.interval = wnet_vif->vm_bcn_intval * TU_DURATION;
            wnet_vif->vm_p2p->noa_index++;
            wnet_vif->vm_p2p->ap_mode_set_noa_enable = 1;
            vm_p2p_noa_start(wnet_vif->vm_p2p, &noa);
            vm_p2p_update_noa_ie(wnet_vif->vm_p2p);
        }
    }
}

#ifndef FW_RF_CALIBRATION
void wifi_mac_tbtt_handle(struct wlan_net_vif *wnet_vif)
{
    unsigned char vid = wnet_vif->wnet_vif_id;
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;

    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "vid:%d, ss->scan_StateFlags:%08x\n", vid, ss->scan_StateFlags);

    WIFI_SCAN_LOCK(ss);
    if (ss->scan_StateFlags & SCANSTATE_F_WAIT_TBTT) {
        ss->scan_StateFlags &= ~SCANSTATE_F_WAIT_TBTT;

        //AML_PRINT_LOG_INFO("start timer\n");
        if (ss->scan_chan_wait <= 60) {
            os_timer_ex_start_period(&ss->ss_scan_timer,
                (WIFINET_SCAN_DEFAULT_INTERVAL - ss->scan_chan_wait - WIFINET_SCAN_CHANNEL_COST - WIFINET_SCAN_INTERVAL_LEFT));

        } else if (ss->scan_chan_wait <= 140) {
            os_timer_ex_start_period(&ss->ss_scan_timer,
                (2 * WIFINET_SCAN_DEFAULT_INTERVAL - ss->scan_chan_wait - WIFINET_SCAN_CHANNEL_COST));

        } else {
            //not suggest scan channel time lager than 140
            os_timer_ex_start_period(&ss->ss_scan_timer,
                (2 * WIFINET_SCAN_DEFAULT_INTERVAL - ss->scan_chan_wait - WIFINET_SCAN_CHANNEL_COST));
        }
    }
    WIFI_SCAN_UNLOCK(ss);

    return;
}
#else
void wifi_mac_tbtt_handle(struct wlan_net_vif *wnet_vif)
{
    unsigned char vid = wnet_vif->wnet_vif_id;
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;

    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG,"vid:%d, ss->scan_StateFlags:%08x, id=%d\n", vid, ss->scan_StateFlags, wnet_vif->wnet_vif_id);
    if ((ss->scan_StateFlags & SCANSTATE_F_WAIT_TBTT)
        && wnet_vif->vm_p2p->p2p_enable
        && !wnet_vif->vm_p2p->ap_mode_set_noa_enable) {
        wifi_mac_set_noa(wnet_vif);
    }
    WIFI_SCAN_LOCK(ss);
    if (ss->scan_StateFlags & SCANSTATE_F_WAIT_TBTT) {
        ss->scan_StateFlags &= ~SCANSTATE_F_WAIT_TBTT;
        //AML_PRINT_LOG_INFO("start timer\n");
        os_timer_ex_start_period(&ss->ss_scan_timer, WIFINET_SCAN_MOTHER_CHANNEL_TIME);
    }
    WIFI_SCAN_UNLOCK(ss);

    return;
}
#endif

int wifi_mac_device_ip_config(struct wlan_net_vif *wnet_vif, void *event)
{
    struct dhcp_offload_event *fw_event = (struct dhcp_offload_event *)event;
    struct in_device *in_dev;
    struct in_ifaddr *ifa_v4;
    unsigned int local_ip = 0;

    if (fw_event == NULL) {
        return 1;
    }

    local_ip = *(__be32 *)(&fw_event->local_ip);
    AML_PRINT(AML_LOG_ID_CONNECT, AML_LOG_LEVEL_DEBUG, " vid:%d\n", fw_event->basic_info.vid);
    in_dev = __in_dev_get_rtnl(wnet_vif->vm_ndev);
    if (in_dev) {
        ifa_v4 = in_dev->ifa_list;
        if (ifa_v4 != NULL && local_ip != 0) {
            ifa_v4->ifa_local = local_ip;
            AML_PRINT_LOG_INFO("local ip: %08x\n", local_ip);
        }
    }

    return 1;
}

int wifi_mac_create_vmac(struct wifi_mac *wifimac, void *ifr, int cmdFromwhr)
{
    struct vm_wlan_net_vif_params vmparam;
    char name[IFNAMSIZ];
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned char myaddr[WIFINET_ADDR_LEN] = { mac_addr0, mac_addr1, mac_addr2, mac_addr3, mac_addr4, mac_addr5 };
    int vm_opmode = 0, vid = 0;
    struct in_device *in_dev;
    struct in_ifaddr *ifa_v4;
    __be32 ipv4 = 0;

    if (cmdFromwhr ==1) {
        if (!capable(CAP_NET_ADMIN))
            return -EPERM;

        if (copy_from_user(&vmparam, ((struct ifreq *)ifr)->ifr_data, sizeof(vmparam)))
            return -EFAULT;

    } else {
        memcpy(&vmparam, ifr, sizeof(vmparam));
    }

    /* interface name 'wlan0', 'p2p0'.... for net device*/
    strncpy(name, vmparam.vm_param_name, sizeof(name) - 1);
    vid = wifi_mac_get_new_vmac_id(wifimac);
    if (vid < 0) {
       AML_PRINT_LOG_ERR("alloc wnet_vif id %d <%s>  < fail > \n",vid, vmparam.vm_param_name);
        return -EIO;
    }

    AML_PRINT_LOG_INFO("cp.vm_param_opmode=%d vm_param_name %s vid = %d \n",
	        vmparam.vm_param_opmode,vmparam.vm_param_name,vid);

    switch (vmparam.vm_param_opmode) {
        case WIFINET_M_STA:
            vm_opmode = vmparam.vm_param_opmode;
            break;
        case WIFINET_M_P2P_DEV:
            vm_opmode = WIFINET_M_STA;
            break;
        case WIFINET_M_IBSS:
        case WIFINET_M_MONITOR:
            if (wifimac->wnet_vif_num != 0)
            {
                AML_PRINT_LOG_ERR("Can't Create opmode=%d.\n",vmparam.vm_param_opmode);
                return   -EIO;
            }
            vm_opmode = vmparam.vm_param_opmode;
            break;
        case WIFINET_M_HOSTAP:
        case WIFINET_M_WDS:
            vm_opmode = WIFINET_M_HOSTAP;
            break;
        default:
            AML_PRINT_LOG_ERR("<running> opmode=%d\n",vmparam.vm_param_opmode);
            return  -EIO;
    }

    wnet_vif = (struct wlan_net_vif *)wifi_mac_alloc_ndev(sizeof(struct wlan_net_vif ));
    if (wnet_vif == NULL)
    {
        AML_PRINT_LOG_ERR("allocate wnet_vif fail \n");
        wifi_mac_delt_vmac_id(wifimac,vid);
        return  -EIO;
    }

#ifdef CONFIG_P2P
    if(vmparam.vm_param_opmode == WIFINET_M_P2P_DEV)
        wnet_vif->vm_p2p_support = 1;
    else
        wnet_vif->vm_p2p_support = 0;
#endif

    wnet_vif->wnet_vif_id = vid;
    wnet_vif->vm_beaconbuf = NULL;
    wnet_vif->wnet_vif_replaycounter = 0;
    wnet_vif->vm_bcn_intval = WIFINET_BINTVAL_DEFAULT;
    wnet_vif->vm_def_mgmt_txkey = WIFINET_KEYIX_NONE;
    wnet_vif->vm_recovery_state = WIFINET_RECOVERY_END;
    wnet_vif->vm_use_static_ip = 0;
    memset(wnet_vif->vm_static_ipv4, 0x00, IPV4_LEN);
    myaddr[2] += vid<<4;

    in_dev = __in_dev_get_rtnl(wnet_vif->vm_ndev);
    if (in_dev) {
      ifa_v4 = in_dev->ifa_list;
      if (ifa_v4 != NULL)
        ipv4 = ifa_v4->ifa_local;
    }

    if ( wifimac->drv_priv->drv_ops.add_interface(wifimac->drv_priv, vid, wnet_vif,
                wifi_mac_opmode_2_halmode(vm_opmode), myaddr, ipv4))
    {
        AML_PRINT_LOG_ERR("add interface < fail > \n");
        wifi_mac_free_vmac(wnet_vif);
        return  -EIO;
    }

    wifi_mac_setup(wifimac, wnet_vif, vm_opmode);
    WIFINET_ADDR_COPY(wnet_vif->vm_myaddr, myaddr);
    vm_wlan_net_vif_register(wnet_vif, name);

    /*set mac operation mode(11GN, 11GNAC) for sta or p2p dev. */
    wifi_mac_mode_vattach_ex(wnet_vif, vmparam.vm_param_opmode);

    wnet_vif->vif_ops.write_word = wifi_mac_write_word;
    wnet_vif->vif_ops.read_word = wifi_mac_read_word;
    wnet_vif->vif_ops.bt_write_word = wifi_mac_bt_write_word;
    wnet_vif->vif_ops.bt_read_word = wifi_mac_bt_read_word;
    wnet_vif->vif_ops.pt_rx_start = wifi_mac_pt_rx_start;
    wnet_vif->vif_ops.pt_rx_stop = wifi_mac_pt_rx_stop;

    AML_PRINT_LOG_INFO("wnet_vif->vm_mac_mode=%d, wnet_vif:%p\n", wnet_vif->vm_mac_mode, wnet_vif);
#if 0
    /* just for fpga test*/
    wifi_mac_write_word(0x00a0e4b4, 0x80801008);
    wifi_mac_write_word(0x00a0e4b8, 0x30010038);
    wifi_mac_write_word(0x00a0b078, 0x00040701);
    wifi_mac_write_word(0x00a0e82c, 0x00000a10);
    wifi_mac_write_word(0x00a0e808, 0x11100000);
    wifi_mac_write_word(0x00a08128, 0x00000211);
    /* just for fpga test*/
#endif
    return 0;
}


enum wifi_mac_macmode p2p_phy_mode_filter (enum wifi_mac_macmode in)
{
    enum wifi_mac_macmode out = in;
    int change = false;
    switch (in)
    {
        case WIFINET_MODE_11B:
            AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_WARN,"should not support 11b only\n");
            break;
        case WIFINET_MODE_11BG:
            out = WIFINET_MODE_11G;
            change = true;
            break;
        case WIFINET_MODE_11BGN:
            out = WIFINET_MODE_11GN;
            change = true;
            break;
        case WIFINET_MODE_11GNAC:
            out = WIFINET_MODE_11GNAC;
            change = true;
            break;
        default:
            // need to check twice
            break;
    }
    if (change == true)
    {
        AML_PRINT(AML_LOG_ID_P2P, AML_LOG_LEVEL_DEBUG,"rm 11b for p2p, macmode=%d\n", out);
    }
    return out;
}

int wifi_mac_get_ap_mode(unsigned char rates[])
{
    int i = 0;
    int j = 0;
    int flag = 0;
    const struct wifi_mac_rateset basic_rate[] =
    {
        { 4, { 0x82, 0x84, 0x8b, 0x96} },
        { 8, { 0x8c, 0x92, 0x98, 0xa4, 0xb0, 0xc8, 0xe0, 0xec} },
    };

    /*check if rates has 80211_B rate */
    for (i = 0; i < rates[1]; i++) {
        for (j = 0; j < basic_rate[0].dot11_rate_num; j++) {
            if (WIFINET_GET_RATE_VAL(rates[i+2]) == WIFINET_GET_RATE_VAL(basic_rate[0].dot11_rate[j])) {
                flag |= BIT(WIFINET_MODE_11B);
                break;
            }
        }
    }

    /*check if rates has 80211_G rate */
    for (i = 0; i < rates[1]; i++) {
        for (j = 0; j < basic_rate[1].dot11_rate_num; j++) {
            if (WIFINET_GET_RATE_VAL(rates[i+2]) == WIFINET_GET_RATE_VAL(basic_rate[1].dot11_rate[j])) {
                flag |= BIT(WIFINET_MODE_11G);
                break;
            }
        }
    }

    return flag;
}

enum wifi_mac_macmode wifi_mac_get_sta_mode(struct wifi_scan_info *se)
{
    enum wifi_mac_macmode macmode = WIFINET_MODE_AUTO;
    unsigned int flag = 0;
    unsigned char is_2g_channel;

    if (se == NULL)
    {
        AML_PRINT_LOG_ERR("se == NULL\n");
        return false;
    }

    is_2g_channel = WIFINET_IS_CHAN_2GHZ(se->SI_chan);

    /*check if se->SI_rates has 80211_B/G rate */
    flag = wifi_mac_get_ap_mode(se->SI_rates);

    /*check if se->SI_exrates has 80211_B/G rate */
    flag |= wifi_mac_get_ap_mode(se->SI_exrates);

    if ((flag & (BIT(WIFINET_MODE_11G)|BIT(WIFINET_MODE_11B)))
            == (BIT(WIFINET_MODE_11G)|BIT(WIFINET_MODE_11B)))
    {
        macmode = WIFINET_MODE_11BG;
    }
    else if ((flag & (BIT(WIFINET_MODE_11G)|BIT(WIFINET_MODE_11B))) == (BIT(WIFINET_MODE_11G)))
    {
        macmode = WIFINET_MODE_11G;
    }
    else if ((flag & (BIT(WIFINET_MODE_11G)|BIT(WIFINET_MODE_11B))) == (BIT(WIFINET_MODE_11B)))
    {
        macmode = WIFINET_MODE_11B;
    }

    if ((se->SI_htcap_ie[1] != 0) && !wifi_mac_is_ht_forbidden(se))
    {
        if(macmode == WIFINET_MODE_11BG)
        {
            macmode = WIFINET_MODE_11BGN;

        } else if(macmode == WIFINET_MODE_11G) {
            macmode = WIFINET_MODE_11GN;

        } else {
            macmode = WIFINET_MODE_11N;
        }

        if(!is_2g_channel && (se->ie_vht_cap[0] == WIFINET_ELEMID_VHTCAP)) {
            macmode = WIFINET_MODE_11GNAC;
        }

    } else {
        if(!is_2g_channel && (se->ie_vht_cap[0] == WIFINET_ELEMID_VHTCAP)) {
            macmode = WIFINET_MODE_11G;
        }
    }//wifi03-5G, add vht under TKIP, but use 11g rate.

    AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_DEBUG,"WARNING::macmode=%d, chan_flags=0x%x\n",
            macmode, se->SI_chan->chan_flags );

    if(macmode == WIFINET_MODE_AUTO)
    {
        if(WIFINET_IS_CHAN_5GHZ (se->SI_chan))
        {
            macmode = WIFINET_MODE_11GNAC;
         }
        else
        {
            macmode = WIFINET_MODE_11BGN;
        }

    }
    AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_DEBUG,"WARNING macmode=%d, need to check twice\n",macmode);
    return macmode;
}

int wifi_mac_mode_vattach_ex (struct wlan_net_vif *wnet_vif,
        enum wifi_mac_opmode opmode)
{
    if( opmode ==WIFINET_M_P2P_DEV) {
        wnet_vif->vm_flags_ext2 |= (WIFINET_FEXT2_PUREG | WIFINET_FEXT2_PUREN);
        wnet_vif->vm_mac_mode = WIFINET_MODE_11GN;

    } else {
        wnet_vif->vm_flags_ext2 |= (WIFINET_FEXT2_PUREB | WIFINET_FEXT2_PUREG | WIFINET_FEXT2_PUREN);
        wnet_vif->vm_mac_mode = WIFINET_MODE_11GNAC;
    }

    AML_PRINT_LOG_INFO("wnet_vif->vm_mac_mode=%d\n", wnet_vif->vm_mac_mode);
    return 1;
}

void wifi_mac_write_word(unsigned int addr,unsigned int data)
{
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    wifimac->drv_priv->drv_ops.drv_write_word(addr, data);
}

unsigned int wifi_mac_read_word(unsigned int addr)
{
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    return wifimac->drv_priv->drv_ops.drv_read_word(addr);
}

void wifi_mac_bt_write_word(unsigned int addr,unsigned int data)
{
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    wifimac->drv_priv->drv_ops.drv_bt_write_word(addr, data);
}

unsigned int wifi_mac_bt_read_word(unsigned int addr)
{
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    return wifimac->drv_priv->drv_ops.drv_bt_read_word(addr);
}


void wifi_mac_pt_rx_start(unsigned int qos)
{
     struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
     wifimac->drv_priv->drv_ops.drv_pt_rx_start(qos);
}

struct rx_statics_st wifi_mac_pt_rx_stop(void)
{
     struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
      return wifimac->drv_priv->drv_ops.drv_pt_rx_stop();
}


void wifi_mac_get_tx_info_task(SYS_TYPE param1,SYS_TYPE param2, SYS_TYPE param3,SYS_TYPE param4,SYS_TYPE param5)
{
    unsigned char i = 0;
    unsigned int reg_backoff_hold = 0;
    struct wlan_net_vif *wnet_vif = NULL;
    struct wifi_mac *wifimac = (struct wifi_mac *)param1;
    struct drv_private *drv_priv = wifimac->drv_priv;
    struct hal_private *hal_priv = drv_priv->hal_priv;

    hal_priv->hal_ops.hal_get_sts(0,STS_MOD_HAL|STS_TYP_TX|STS_MOD_HIF);
    reg_backoff_hold = drv_priv->drv_ops.drv_read_word(MAC_COUNT3);
    AML_PRINT_LOG_INFO("reg_backoff_hold:0x%08x \n", reg_backoff_hold);

    for (i = QUEUE_AC_MIN; i < QUEUE_AC_MAX; i++)
    {
        AML_PRINT_LOG_INFO("mac_txcontrol addr:0x%08x, val:0x%08x \n", MAC_TXCONTROL_NONQOS + i * 4, drv_priv->drv_ops.drv_read_word(MAC_TXCONTROL_NONQOS + i * 4));
    }

    list_for_each_entry(wnet_vif,&wifimac->wm_wnet_vifs, vm_next) {
        if (IS_UP(wnet_vif->vm_ndev)) {
            phy_get_queue_debug_info(wnet_vif->wnet_vif_id);
        }
    }

    return;
}

void wifi_mac_connect_repair_task(SYS_TYPE param1,SYS_TYPE param2, SYS_TYPE param3,SYS_TYPE param4,SYS_TYPE param5)
{
    struct wlan_net_vif *wnet_vif = NULL;
    struct wifi_mac *wifimac = (struct wifi_mac *)param1;
    int ret = 0;

#ifdef CHIP_RESET_SUPPORT
    if (aml_bus_type == 1) {
        AML_PRINT_LOG_INFO("recovery by upper\n");
        wifimac->request_upper_recovery = 1;
    } else
#endif
    {
#if defined(SDIO_BUILD_IN) && defined(SDIO_MODE_ON)
        wifi_mac_set_wifi_insmod_status();
#endif
        wifimac->wm_recovery_src &= ~WIFINET_RECOVERY_SRC_MASK;
        wifimac->wm_stavif_channel = WIFINET_CHAN_ERR;


        list_for_each_entry(wnet_vif,&wifimac->wm_wnet_vifs, vm_next) {
            if ((wnet_vif->vm_ndev->flags & IFF_RUNNING) == 0) {
                //current vif not running, no need to recovery
                AML_PRINT_LOG_INFO("skip vif[%d] due to it's not running\n", wnet_vif->wnet_vif_id);
                continue;
            }
            ret = wifi_mac_recovery_record_vif(wnet_vif);
        }

        //No support interface running, just stop.
        if ((wifimac->wm_recovery_flags & WIFINET_RECOVERY_F_VIF_MASK) == 0) {
            AML_PRINT_LOG_ERR("No support vif running, don't recovery\n");
            WIFINET_FW_STAT_LOCK(wifimac);
            wifimac->wm_recovery_flags = 0;
            WIFINET_FW_STAT_UNLOCK(wifimac);
            return;
        }

        wifimac->wmac_ops.wifi_mac_process_recovery(wifimac);
    }
    // host status restore is asynchronous, so that task end not mean recovery end
    return;
}

int wifi_mac_tx_fail_statistics(struct wifi_mac *wifimac)
{
    static unsigned int last_tx_fail = 0;
    struct hw_interface* hif = hif_get_hw_interface();

    if (gAmlTraceInfo[AML_LOG_ID_TX_REC].moduleTraceLevel >= AML_LOG_LEVEL_DEBUG) {
        struct drv_private *drv_priv = wifimac->drv_priv;
        if (!drv_priv->hal_priv->hal_ops.hal_tx_empty()) {
            unsigned int i = 0;
            unsigned long long time;
            bool data_send_detail = true;

            time = ktime_to_ms(ktime_get_boottime());
            for (i = 0; i < WIFI_MAX_TXFRAME; i++) {
                if ( tx_info_record[i].using && ((time - tx_info_record[i].timestamp) >= (WIFINET_TX_LIVE_TIME + 300))) {
                    if (data_send_detail) {
                        data_send_detail = false;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0) && !defined (LINUX_PLATFORM)
                        AML_PRINT(AML_LOG_ID_TX_REC, AML_LOG_LEVEL_DEBUG,"TX [send:%d done:%d free:%d] id map: %lx, %lx, %lx, %lx\n",
                            hif->HiStatus.Tx_Send_num,hif->HiStatus.Tx_Done_num,hif->HiStatus.Tx_Free_num,
                            drv_priv->hal_priv->tx_frames_map[0], drv_priv->hal_priv->tx_frames_map[1],
                            drv_priv->hal_priv->tx_frames_map[2],drv_priv->hal_priv->tx_frames_map[3]);
#else
                        AML_PRINT(AML_LOG_ID_TX_REC, AML_LOG_LEVEL_DEBUG,"TX [send:%d done:%d free:%d] id map: %lx, %lx, %lx, %lx\n",
                            atomic_read(&hif->HiStatus.Tx_Send_num),atomic_read(&hif->HiStatus.Tx_Done_num),atomic_read(&hif->HiStatus.Tx_Free_num),
                            drv_priv->hal_priv->tx_frames_map[0], drv_priv->hal_priv->tx_frames_map[1],
                            drv_priv->hal_priv->tx_frames_map[2],drv_priv->hal_priv->tx_frames_map[3]);
#endif

                    }

                    AML_PRINT(AML_LOG_ID_TX_REC, AML_LOG_LEVEL_DEBUG,"vid:%d tid:%d queue_id:%d seq:%d fc:%04x pkt_len:%d sn|id:%08x using:%d time:%llu\n",
                        tx_info_record[i].vid,
                        tx_info_record[i].tid,
                        tx_info_record[i].queue_id,
                        tx_info_record[i].seq,
                        tx_info_record[i].fc,
                        tx_info_record[i].packetlen,
                        (tx_info_record[i].seq << 8)|(tx_info_record[i].tx_id),
                        tx_info_record[i].using,
                        tx_info_record[i].timestamp);
                }
            }
        }
    }

    AML_PRINT(AML_LOG_ID_XMIT,AML_LOG_LEVEL_DEBUG,"current tx fail:%d  last tx_fail:%d  avg_tx_fail_num:%d \n",
                hif->HiStatus.tx_fail_num,last_tx_fail,hif->HiStatus.avg_tx_fail_num);

    hif->HiStatus.avg_tx_fail_num = hif->HiStatus.tx_fail_num - last_tx_fail;
    last_tx_fail = hif->HiStatus.tx_fail_num;

    return 0;
}

void wifi_mac_tp_test_report(struct wifi_mac *wifimac, unsigned short src, unsigned short dest)
{
/**5001 for iperf; 5201 for iperf3***/
    if ((src >= 5000 && src <= 5201) || (dest >= 5000 && dest <= 5201)) {
        wifimac->tp_pkt_flag = 1;
    }
}

int wifi_mac_monitor_tp_rate(void *arg)
{
    struct wifi_mac *wifimac = (struct wifi_mac *)arg;
    struct wlan_net_vif *wnet_vif = NULL;
    static unsigned short last_in_throughput = 0;
    static unsigned short count_n = 0;

    if (wifimac->tp_pkt_flag) {
        wifimac->in_throughput = BIT(4);
        wifimac->tp_pkt_flag = 0;
    } else {
        wifimac->in_throughput >>= 1;
    }

    last_in_throughput = wifimac->in_throughput;

    wifi_mac_tx_fail_statistics(wifimac);

    if ((count_n++ % 300) == 0 && (count_n != 0))
    {
        wifi_mac_show_per_info();
    }

    list_for_each_entry(wnet_vif,&wifimac->wm_wnet_vifs, vm_next) {
        /* tcp tx tp */
        wnet_vif->txtp_stat.vm_tx_speed = wnet_vif->txtp_stat.tcp_tx_payload_total >> 17;
        wnet_vif->txtp_stat.tcp_tx_payload_total = 0;
        /* udp tx and rx tp */
        wnet_vif->txtp_stat.vm_udp_tx_speed = wnet_vif->txtp_stat.udp_tx_payload_total >> 17;
        wnet_vif->txtp_stat.udp_tx_payload_total = 0;
        wnet_vif->rxtp_stat.vm_udp_rx_speed = wnet_vif->rxtp_stat.udp_rx_payload_total >> 17;
        wnet_vif->rxtp_stat.udp_rx_payload_total = 0;
        break;
    }

    list_for_each_entry(wnet_vif,&wifimac->wm_wnet_vifs, vm_next) {
        if (wifimac->in_throughput && wnet_vif->vm_scan_time_connect > WIFINET_SCAN_TIME_CONNECT_DEFAULT) {
            wnet_vif->vm_scan_time_connect = WIFINET_SCAN_TIME_CONNECT_DEFAULT;
            wifi_mac_set_scan_time(wnet_vif);
        } else if (!wifimac->in_throughput && wnet_vif->vm_scan_time_connect < WIFINET_SCAN_TIME_CONNECT_MAX) {
            wnet_vif->vm_scan_time_connect = WIFINET_SCAN_TIME_CONNECT_MAX;
            wifi_mac_set_scan_time(wnet_vif);
        }
    }

    return 0;
}

int wifi_mac_ant_select(void *arg)
{
    struct wifi_mac *wifimac = (struct wifi_mac *)arg;
    struct wlan_net_vif *wnet_vif = NULL;
    list_for_each_entry(wnet_vif,&wifimac->wm_wnet_vifs, vm_next) {
        if (wnet_vif->vm_opmode == WIFINET_M_STA) {
            if (g_ant_mode_force == 0) {
                wifi_mac_reset_ant_measure();
                wifi_mac_set_ant_flag(wifi_mac_get_cur_ant());
                printk("ant_select: start measure ant %d, time %ld\n", wifi_mac_get_cur_ant(),jiffies);
                os_timer_ex_start(&wifimac->wm_ant_rssi_measure);
            }
        }
        break;
    }
    return 0;

}

unsigned char ant_debug = 1;
#define DPRINTX( ...) do {         \
                if (ant_debug)                \
                        printk(__VA_ARGS__);      \
        } while (0)


int wifi_mac_ant_rssi_measure(void *arg)
{
    unsigned char cur_rssi = 0;
    unsigned char div_rssi = 0;
    DPRINTX("rssi_measure: timer time %ld\n", jiffies);
    if (wifi_mac_cur_ant_rssi(&cur_rssi)) {
        if (cur_rssi > 210) {
            DPRINTX("rssi_measure: current ant %d, cur_rssi %d, done.\n", wifi_mac_get_cur_ant(), cur_rssi - 256);
            return 1;
        }
        if (wifi_mac_get_ant_flag((wifi_mac_get_cur_ant()+1)%2)) {
            if (!wifi_mac_div_ant_rssi(&div_rssi)) {
                DPRINTX("rssi_measure: div ant no bcn received, don't change ant, cur_rssi %d, done.\n", cur_rssi - 256);
                return 1;
            } else {
                if (div_rssi > cur_rssi + 2) {
                    DPRINTX("rssi_measure: rssi gap, switch ant %d, %d, done.\n", div_rssi, cur_rssi);
                    wifi_mac_switch_ant(arg);
                }
                else
                {
                    DPRINTX("rssi_measure: cur_rssi %d, div_rssi %d, cur ant %d, done.\n", cur_rssi, div_rssi, wifi_mac_get_cur_ant());
                }
                return 1;
            }
        } else {
            wifi_mac_switch_ant(arg);
            DPRINTX("rssi_measure: ant is empty, switch to ant %d, org rssi %d\n", wifi_mac_get_cur_ant(), cur_rssi);
            wifi_mac_set_ant_flag((wifi_mac_get_cur_ant()+1)%2);
        }
        return 0;
    } else {
        if (wifi_mac_get_ant_flag((wifi_mac_get_cur_ant()+1)%2)) {
            if (wifi_mac_div_ant_rssi(&div_rssi)) {
                wifi_mac_switch_ant(arg);
                DPRINTX("rssi_measure: ant no bcn, switch to ant %d, div rssi %d, done.\n", wifi_mac_get_cur_ant(), div_rssi);
            }
            return 1;
        } else {
            wifi_mac_switch_ant(arg);
            DPRINTX("rssi_measure: 1st ant no bcn, switch to ant %d to measure\n", wifi_mac_get_cur_ant());
            wifi_mac_set_ant_flag((wifi_mac_get_cur_ant()+1)%2);
            return 0;
        }
    }

    return 1;
}

void wifi_mac_get_repair_level(void)
{
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();

    if (wifimac->wm_recovery_src & BIT(WIFINET_RECOVERY_SRC_SDIO_TIMEOUT)) {
        wifimac->wm_recovery_level = WIFINET_RECOVERY_L_THOROUGH;
    } else if (wifimac->wm_recovery_src & BIT(WIFINET_RECOVERY_SRC_CMD_CRASH)) {
        wifimac->wm_recovery_level = WIFINET_RECOVERY_L_NORMAL;
    } else if (wifimac->wm_recovery_src & BIT(WIFINET_RECOVERY_SRC_WAKE_FAIL)) {
        wifimac->wm_recovery_level = WIFINET_RECOVERY_L_NORMAL;
    } else if (wifimac->wm_recovery_src & BIT(WIFINET_RECOVERY_SRC_PAGE_ERROR)) {
        wifimac->wm_recovery_level = WIFINET_RECOVERY_L_NORMAL;
    } else if (wifimac->wm_recovery_src & BIT(WIFINET_RECOVERY_SRC_TX_FAIL)) {
        wifimac->wm_recovery_level = WIFINET_RECOVERY_L_NORMAL;
    } else if (wifimac->wm_recovery_src & BIT(WIFINET_RECOVERY_SRC_TX_STUCK)) {
        wifimac->wm_recovery_level = WIFINET_RECOVERY_L_NORMAL;
    }

}

tx_info_record_st tx_info_record[WIFI_MAX_TXFRAME];
void tx_done_record(struct fw_txdesc_fifo *pTxDescFiFo, unsigned char queue_id)
{
    unsigned int tx_id;
    struct hi_tx_desc *pTxDPape = NULL;
    struct drv_txdesc *p_drv_txdesc = NULL;

    if (aml_wifi_is_enable_rf_test()) {
        return;
    }

    if (gAmlTraceInfo[AML_LOG_ID_TX_REC].moduleTraceLevel < AML_LOG_LEVEL_DEBUG) {
        return;
    }

    if (pTxDescFiFo == NULL) {
        AML_PRINT(AML_LOG_ID_TX_REC,AML_LOG_LEVEL_WARN," pTxDescFiFo is NULL !\n");
        return ;
    }

    if (pTxDescFiFo->pTxDPape == NULL) {
        AML_PRINT(AML_LOG_ID_TX_REC,AML_LOG_LEVEL_WARN," pTxDPape is NULL !\n");
        return ;
    }

    p_drv_txdesc = (struct drv_txdesc *)pTxDescFiFo->callback;

    if (p_drv_txdesc == NULL) {
        AML_PRINT(AML_LOG_ID_TX_REC,AML_LOG_LEVEL_WARN," p_drv_txdesc is NULL !\n");
        return ;
    }

    if (p_drv_txdesc->txinfo == NULL) {
        AML_PRINT(AML_LOG_ID_TX_REC,AML_LOG_LEVEL_WARN," txinfo is NULL !\n");
        return ;
    }

    COMMON_LOCK();

    pTxDPape = pTxDescFiFo->pTxDPape;
    tx_id = pTxDPape->TxPriv.hostcallbackid;

    if (tx_info_record[tx_id].using) {
        AML_PRINT(AML_LOG_ID_TX_REC,AML_LOG_LEVEL_DEBUG,"vid:%d tid:%d queue_id:%d seq:%d fc:%04x pkt_len:%d sn|id:%08x using:%d time:%llu\n",
        tx_info_record[tx_id].vid,
        tx_info_record[tx_id].tid,
        tx_info_record[tx_id].queue_id,
        tx_info_record[tx_id].seq,
        tx_info_record[tx_id].fc,
        tx_info_record[tx_id].packetlen,
        (tx_info_record[tx_id].seq << 8)|(tx_info_record[tx_id].tx_id),
        tx_info_record[tx_id].using,
        tx_info_record[tx_id].timestamp);
    }

    tx_info_record[tx_id].using = true;
    tx_info_record[tx_id].vid = pTxDPape->TxPriv.vid;
    tx_info_record[tx_id].tid = pTxDPape->TxPriv.TID;
    tx_info_record[tx_id].seq = pTxDPape->TxPriv.SN;
    tx_info_record[tx_id].queue_id = queue_id;
    tx_info_record[tx_id].packetlen = p_drv_txdesc->txinfo->packetlen;
    tx_info_record[tx_id].tx_id = pTxDPape->TxPriv.hostcallbackid;
    tx_info_record[tx_id].fc = pTxDPape->TxVector.tv_FrameControl;
    tx_info_record[tx_id].timestamp = ktime_to_ms(ktime_get_boottime());

    COMMON_UNLOCK();

    return;

}

extern unsigned char aml_convert_tid(struct drv_txdesc *ptxdesc);
void tx_free_record(struct Tx_FrameDesc *pTxFrameDesc,unsigned char tx_id)
{
    struct drv_txdesc *ptxdesc ;

    if (aml_wifi_is_enable_rf_test()) {
        return;
    }

    if (gAmlTraceInfo[AML_LOG_ID_TX_REC].moduleTraceLevel < AML_LOG_LEVEL_DEBUG) {
        return;
    }

    if (pTxFrameDesc == NULL) {
        AML_PRINT(AML_LOG_ID_TX_REC,AML_LOG_LEVEL_WARN," pTxFrameDesc is NULL !\n");
        return;
    }

    ptxdesc = (struct drv_txdesc *)pTxFrameDesc->callback;

    if (ptxdesc == NULL) {
        AML_PRINT(AML_LOG_ID_TX_REC,AML_LOG_LEVEL_WARN," ptxdesc is NULL !\n");
        return;
    }

    //COMMON_LOCK();

    if (ptxdesc->txinfo) {

        if  ( (tx_info_record[tx_id].using)
            && (tx_info_record[tx_id].seq == ptxdesc->txinfo->seqnum)
            && (tx_info_record[tx_id].tid == aml_convert_tid(ptxdesc))
            && (tx_info_record[tx_id].vid == ptxdesc->txinfo->wnet_vif_id)
            && (tx_id == tx_info_record[tx_id].tx_id)) {
            tx_info_record[tx_id].using = 0;
        } else {
            AML_PRINT(AML_LOG_ID_TX_REC,AML_LOG_LEVEL_DEBUG,"vid:%d tid:%d queue_id:%d seq:%d:%04x fc:0x%02x flag:%02x tx_id:%02x \n",
                ptxdesc->txinfo->wnet_vif_id,
                aml_convert_tid(ptxdesc),
                ptxdesc->txinfo->queue_id,
                ptxdesc->txinfo->seqnum,
                ptxdesc->txinfo->seqnum,
                ((struct wifi_frame *)(ptxdesc->txdesc_ddraddr))->i_fc[0],
                ((struct wifi_frame *)(ptxdesc->txdesc_ddraddr))->i_fc[1],
                tx_id);

            AML_PRINT(AML_LOG_ID_TX_REC,AML_LOG_LEVEL_DEBUG,"vid:%d tid:%d queue_id:%d seq:%d fc:%04x pkt_len:%d sn|id:%08x using:%d time:%llu \n",
                tx_info_record[tx_id].vid,
                tx_info_record[tx_id].tid,
                tx_info_record[tx_id].queue_id,
                tx_info_record[tx_id].seq,
                tx_info_record[tx_id].fc,
                tx_info_record[tx_id].packetlen,
                (tx_info_record[tx_id].seq << 8)|(tx_info_record[tx_id].tx_id),
                tx_info_record[tx_id].using,
                tx_info_record[tx_id].timestamp);

            tx_info_record[tx_id].using = 0;
        }
    } else {
        tx_info_record[tx_id].using = 0;
        AML_PRINT(AML_LOG_ID_TX_REC,AML_LOG_LEVEL_DEBUG,"already free tx_id:%d \n",tx_id);
    }

    // COMMON_UNLOCK();

    return;
}

void tx_record_show()
{
    unsigned int i;
    struct hal_private* hal_priv = hal_get_priv();
    struct hw_interface* hif = hif_get_hw_interface();

    if (gAmlTraceInfo[AML_LOG_ID_TX_REC].moduleTraceLevel < AML_LOG_LEVEL_DEBUG) {
        return;
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0) && !defined (LINUX_PLATFORM)
    AML_PRINT(AML_LOG_ID_TX_REC,AML_LOG_LEVEL_INFO,"TX [send:%d done:%d free:%d] id map: %lx, %lx\n",
            hif->HiStatus.Tx_Send_num,hif->HiStatus.Tx_Done_num,hif->HiStatus.Tx_Free_num,
            hal_priv->tx_frames_map[0], hal_priv->tx_frames_map[1]);
#else
    AML_PRINT(AML_LOG_ID_TX_REC,AML_LOG_LEVEL_INFO,"TX [send:%d done:%d free:%d] id map: %lx, %lx\n",
            atomic_read(&hif->HiStatus.Tx_Send_num),atomic_read(&hif->HiStatus.Tx_Done_num),atomic_read(&hif->HiStatus.Tx_Free_num),
            hal_priv->tx_frames_map[0], hal_priv->tx_frames_map[1]);
#endif

    for (i = 0; i < WIFI_MAX_TXFRAME; i++) {
        if  (tx_info_record[i].using) {
            AML_PRINT(AML_LOG_ID_TX_REC,AML_LOG_LEVEL_INFO,"vid:%d tid:%d queue_id:%d seq:%d fc:%04x pkt_len:%d sn|id:%08x using:%d time:%llu",
                tx_info_record[i].vid,
                tx_info_record[i].tid,
                tx_info_record[i].queue_id,
                tx_info_record[i].seq,
                tx_info_record[i].fc,
                tx_info_record[i].packetlen,
                (tx_info_record[i].seq << 8)|(tx_info_record[i].tx_id),
                tx_info_record[i].using,
                tx_info_record[i].timestamp);
        }
    }

    return;

}

int wifi_mac_trigger_recovery(void *arg)
{
    struct wifi_mac *wifimac = (struct wifi_mac *)arg;
    static unsigned int free_page = 0;
    static unsigned int send = 0;
    static unsigned int done = 0;
    static unsigned int free = 0;
    static unsigned int more_check = 1;
    static unsigned int observe_period = 0;
    static unsigned int tx_ok_num = 0;
    static unsigned int tx_fail_num = 0;
    struct hal_private* hal_priv = hal_get_priv();
    struct hw_interface* hif = hif_get_hw_interface();

#if 0 //this is for test
    wifi_mac_add_work_task(wifimac, wifi_mac_connect_repair_task, NULL, (SYS_TYPE)wifimac, 0, 0, 0, 0);
    return 0;
#endif
    if (wifimac->wm_recovery_flags & WIFINET_RECOVERY_F_RUNNING) {
        AML_PRINT_LOG_INFO("recovery in progress\n");
        return 0;
    }
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0) && !defined (LINUX_PLATFORM)
    if ((free_page == hal_priv->txPageFreeNum)
            && (done == hif->HiStatus.Tx_Done_num) && (free == hif->HiStatus.Tx_Free_num)
            && (done != free))
#else
    if ((free_page == hal_priv->txPageFreeNum)
            && (done == atomic_read(&hif->HiStatus.Tx_Done_num)) && (free == atomic_read(&hif->HiStatus.Tx_Free_num))
            && (done != free))
#endif
    {
        wifimac->wm_recovery_src |= BIT(WIFINET_RECOVERY_SRC_TX_STUCK);
    }

    if ((hif->HiStatus.tx_ok_num == tx_ok_num) && (hif->HiStatus.tx_fail_num - tx_fail_num > 3000)) {
        wifimac->wm_recovery_src |= BIT(WIFINET_RECOVERY_SRC_TX_FAIL);
    }

    if (hal_priv->txPageFreeNum > hif->hw_config.txpagenum) {
        wifimac->wm_recovery_src |= BIT(WIFINET_RECOVERY_SRC_PAGE_ERROR);
    }

    if (wifi_mac_get_host_wake_status() > 0) {
        wifimac->wm_recovery_src |= BIT(WIFINET_RECOVERY_SRC_WAKE_FAIL);
    }

    if (wifi_mac_get_chip_en_access()) {
        wifimac->wm_recovery_src |= BIT(WIFINET_RECOVERY_SRC_SDIO_TIMEOUT);
    }

    /* no need recovery */
    if ((wifimac->wm_recovery_src & WIFINET_RECOVERY_SRC_MASK) == 0) {
        observe_period >>= 1;
        free_page = hal_priv->txPageFreeNum;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0) && !defined (LINUX_PLATFORM)
        send = hif->HiStatus.Tx_Send_num;
        done = hif->HiStatus.Tx_Done_num;
        free = hif->HiStatus.Tx_Free_num;
#else
        send = atomic_read(&hif->HiStatus.Tx_Send_num);
        done = atomic_read(&hif->HiStatus.Tx_Done_num);
        free = atomic_read(&hif->HiStatus.Tx_Free_num);
#endif
        tx_ok_num = hif->HiStatus.tx_ok_num;
        tx_fail_num = hif->HiStatus.tx_fail_num;
        more_check = 1;
        return 0;
    }

    if (IS_RECOVERY_GET_TX_INFO(wifimac->wm_recovery_src)) {
        tx_record_show();
        wifi_mac_add_work_task(wifimac, wifi_mac_get_tx_info_task, NULL, (SYS_TYPE)wifimac, 0, 0, 0, 0);
    }

    more_check = (IS_RECOVERY_CHECK_ONCE(wifimac->wm_recovery_src) ? 0 : more_check);

    wifi_mac_show_per_info();

    /* maybe need recovery */
    if (more_check != 0) {
        //need check once more
        AML_PRINT_LOG_INFO("recovery src 0x%x, need check again\n", wifimac->wm_recovery_src);
        wifimac->wm_recovery_src &= ~WIFINET_RECOVERY_SRC_MASK;
    } else {
        wifimac->wm_recovery_level = WIFINET_RECOVERY_L_THOROUGH;
        if (observe_period == 0) {
            //not in observation period, get repair level
            wifi_mac_get_repair_level();
        }
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0) && !defined (LINUX_PLATFORM)
        AML_PRINT_LOG_INFO("last send %d, current send %d, last done %d, current done %d, last free %d, current free %d\n",
            send, hif->HiStatus.Tx_Send_num, done, hif->HiStatus.Tx_Done_num, free, hif->HiStatus.Tx_Free_num);
#else
        AML_PRINT_LOG_INFO("last send %d, current send %d, last done %d, current done %d, last free %d, current free %d\n",
            send, atomic_read(&hif->HiStatus.Tx_Send_num), done, atomic_read(&hif->HiStatus.Tx_Done_num), free, atomic_read(&hif->HiStatus.Tx_Free_num));
#endif

        AML_PRINT_LOG_INFO("last free page %d, current free page %d, tx_ok %d, last tx_ok %d, tx_fail:%d, last_tx_fail:%d\n",
                free_page, hal_priv->txPageFreeNum, tx_ok_num, hif->HiStatus.tx_ok_num, tx_fail_num, hif->HiStatus.tx_fail_num);
        AML_PRINT_LOG_INFO("recovery src 0x%x, recovery level %d\n", wifimac->wm_recovery_src & WIFINET_RECOVERY_SRC_MASK, wifimac->wm_recovery_level);

        observe_period = BIT(4);//need check 5 times after recovery
        wifimac->wm_recovery_src &= ~WIFINET_RECOVERY_SRC_MASK;

        wifimac->wm_recovery_flags = WIFINET_RECOVERY_F_RUNNING;
        wifi_mac_add_work_task(wifimac, wifi_mac_connect_repair_task, NULL, (SYS_TYPE)wifimac, 0, 0, 0, 0);
    }

    more_check ^= 0x1;

    return 0;
}

int wifi_mac_recovery_record_vif(struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = NULL;
    unsigned char is_gc = 0, is_go = 0;
    int ret = 0;

    if (wnet_vif != NULL) {
        wifimac = wnet_vif->vm_wmac;
    }
#ifdef CONFIG_P2P
    AML_PRINT_LOG_INFO("recovery vif[%d], opmode = %d iftype = %d p2p_support = %d p2p_role = %d\n",
        wnet_vif->wnet_vif_id, wnet_vif->vm_opmode, wnet_vif->vm_wdev->iftype, wnet_vif->vm_p2p_support, vm_p2p_role(wnet_vif->vm_p2p));
#else
    AML_PRINT_LOG_INFO("recovery vif[%d], opmode = %d iftype = %d\n", wnet_vif->wnet_vif_id, wnet_vif->vm_opmode, wnet_vif->vm_wdev->iftype);
#endif
    switch (wnet_vif->vm_opmode)
    {
        case WIFINET_M_STA:
        case WIFINET_M_P2P_CLIENT://no use
#ifdef CONFIG_P2P
            //p2p group client mode
            is_gc = wnet_vif->vm_p2p_support && vm_p2p_role(wnet_vif->vm_p2p) == NET80211_P2P_ROLE_CLIENT;
            AML_PRINT_LOG_INFO("vif[%d] is_gc = %d\n", wnet_vif->wnet_vif_id, is_gc);
#endif
            wnet_vif->vm_recovery_state = WIFINET_RECOVERY_START;
            wifimac->wm_recovery_flags |= WIFINET_RECOVERY_F_VIF_MAP(wnet_vif->wnet_vif_id);
            if (wnet_vif->vm_state == WIFINET_S_CONNECTED) {
                if (is_gc) {
                    AML_PRINT_LOG_INFO("vif[%d] p2p client unable to reassoc to GO, need report disconnect event\n", wnet_vif->wnet_vif_id);
                } else {
#ifdef CONFIG_CONCURRENT_MODE
                    if (wifimac->wm_nrunning > 1 && !concurrent_check_is_vmac_same_pri_channel(wifimac)) {
                        AML_PRINT_LOG_INFO("vifs' channel are inconsistent, maybe need to switch channel\n");
                        wifimac->wm_stavif_channel = wnet_vif->vm_curchan;
                    }
#endif
                    wifimac->wm_recovery_flags |= WIFINET_RECOVERY_F_BSS_JOINED(wnet_vif->wnet_vif_id);
                    AML_PRINT_LOG_INFO("vif[%d] recovery with bss joined\n", wnet_vif->wnet_vif_id);
                }
            }
            break;
        case WIFINET_M_HOSTAP:
        case WIFINET_M_P2P_GO://no use
#ifdef CONFIG_P2P
            //p2p group owner mode
            is_go = wnet_vif->vm_p2p_support && vm_p2p_role(wnet_vif->vm_p2p) == NET80211_P2P_ROLE_GO;
            AML_PRINT_LOG_INFO("vif[%d] is_go = %d\n", wnet_vif->wnet_vif_id, is_go);
#endif
            wnet_vif->vm_recovery_state = WIFINET_RECOVERY_START;
            wifimac->wm_recovery_flags |= WIFINET_RECOVERY_F_VIF_MAP(wnet_vif->wnet_vif_id);
            wifimac->wm_recovery_flags |= WIFINET_RECOVERY_F_BSS_JOINED(wnet_vif->wnet_vif_id);
            break;
        default:
            ret = -EINVAL;
            break;
    }
    return ret;
}

/* @function: Reset host driver tx status/connect status */
void wifi_mac_recovery_host_reset(struct wifi_mac *wifimac)
{
    struct wlan_net_vif *wnet_vif = NULL;

    AML_PRINT_LOG_INFO("\n");

    list_for_each_entry(wnet_vif, &wifimac->wm_wnet_vifs, vm_next) {
        if ((wifimac->wm_recovery_flags & WIFINET_RECOVERY_F_VIF_MAP(wnet_vif->wnet_vif_id)) == 0) {
            continue;
        }
        AML_PRINT_LOG_INFO("reset vif[%d], vm_opmode:%d\n", wnet_vif->wnet_vif_id, wnet_vif->vm_opmode);
        switch (wnet_vif->vm_opmode) {
            case WIFINET_M_STA:
            case WIFINET_M_P2P_CLIENT:
                wifi_mac_top_sm(wnet_vif, WIFINET_S_SCAN, 0);
                break;
            case WIFINET_M_HOSTAP:
            case WIFINET_M_P2P_GO:
                wifi_mac_top_sm(wnet_vif, WIFINET_S_INIT, 0);
                wifi_mac_buffer_txq_flush(&wnet_vif->vm_forward_buffer_queue);
                break;
            default:
                AML_PRINT_LOG_ERR("unsupported vif opmode<%d>!\n", wnet_vif->vm_opmode);
                break;
        }
        //flush all pkts those pending in hal
        hal_tx_flush(wnet_vif->wnet_vif_id);
    }
}

void wifi_mac_mkey_restore_task(SYS_TYPE param1,SYS_TYPE param2, SYS_TYPE param3,SYS_TYPE param4,SYS_TYPE param5)
{
    struct wifi_mac_key *key = NULL;
    struct wlan_net_vif *wnet_vif = (struct wlan_net_vif *)param1;

    if ((wnet_vif->vm_def_txkey == WIFINET_KEYIX_NONE) ||
        (KEY_UNDEFINED(wnet_vif->vm_nw_keys[wnet_vif->vm_def_txkey]))) {

        AML_PRINT(AML_LOG_ID_KEY, AML_LOG_LEVEL_WARN,"get multicast key err\n");
        return;
    }

    key = &wnet_vif->vm_nw_keys[wnet_vif->vm_def_txkey];

    if (!wifi_mac_security_setkey(wnet_vif, key, wnet_vif->vm_myaddr, wnet_vif->vm_mainsta)) {
        AML_PRINT(AML_LOG_ID_KEY, AML_LOG_LEVEL_ERROR,"wifi_mac_security_setkey err\n");
        return;
    }

    AML_PRINT(AML_LOG_ID_KEY, AML_LOG_LEVEL_INFO,"multicast key restore success\n");

    return;
}

/* @function: called when a vif is restored end to set private status(wnet_vif) and public status(wifimac) */
int wifi_mac_vif_restore_end(struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;

    if (wnet_vif->vm_recovery_state == WIFINET_RECOVERY_END) {
        return -1;
    }

    if ((wifimac->wm_recovery_flags & WIFINET_RECOVERY_F_VIF_MAP(wnet_vif->wnet_vif_id)) == 0) {
        return -2;
    }

    AML_PRINT_LOG_INFO("vif[%d] restore end\n", wnet_vif->wnet_vif_id);
    wnet_vif->vm_recovery_state = WIFINET_RECOVERY_END;
    wifimac->wm_recovery_flags &= ~ WIFINET_RECOVERY_F_VIF_MAP(wnet_vif->wnet_vif_id);
    wifimac->wm_recovery_flags &= ~ WIFINET_RECOVERY_F_BSS_JOINED(wnet_vif->wnet_vif_id);

    if ((wifimac->wm_recovery_flags & WIFINET_RECOVERY_F_VIF_MASK) == 0) {
        wifimac->wm_recovery_src &= ~WIFINET_RECOVERY_SRC_MASK;
        //all running vifs restore end
        wifimac->wm_recovery_flags = 0;
        wifimac->wm_stavif_channel = WIFINET_CHAN_ERR;
        wifi_mac_scan_access(wnet_vif);
        AML_PRINT_LOG_INFO("!!!!!!!!! fw recovery end !!!!!!!!!\n");
    } else {
        wifi_mac_recovery_host_restore(wifimac);
    }

    return 0;
}

/* @function: Restore the host driver to the state it was in before the recovery was triggered, such as the connection state*/
void wifi_mac_recovery_host_restore_task(SYS_TYPE param1, SYS_TYPE param2, SYS_TYPE parma3, SYS_TYPE param4, SYS_TYPE param5)
{
    struct wifi_mac *wifimac = (struct wifi_mac *)param1;
    struct wlan_net_vif *wnet_vif = NULL;
    struct vm_wdev_priv *pwdev_priv = NULL;

    wifi_mac_clear_host_wake_status();

    AML_PRINT_LOG_INFO("\n");

    list_for_each_entry(wnet_vif, &wifimac->wm_wnet_vifs, vm_next) {
        if ((wifimac->wm_recovery_flags & WIFINET_RECOVERY_F_VIF_MAP(wnet_vif->wnet_vif_id)) == 0) {
            continue;
        }

        AML_PRINT_LOG_INFO("restore vif[%d] start\n", wnet_vif->wnet_vif_id);

        pwdev_priv = wdev_to_priv(wnet_vif->vm_wdev);

        wifimac->drv_priv->drv_ops.drv_interface_enable(ENABLE, wnet_vif->wnet_vif_id);
        wifimac->drv_priv->drv_ops.set_macaddr(wifimac->drv_priv, wnet_vif->wnet_vif_id, wnet_vif->vm_myaddr);

        if (wifimac->wm_recovery_flags & WIFINET_RECOVERY_F_BSS_JOINED(wnet_vif->wnet_vif_id)) {
            wnet_vif->vm_recovery_state = WIFINET_RECOVERY_VIF_UP;
            switch (wnet_vif->vm_opmode) {
                case WIFINET_M_STA:
                    //p2p gc won't set WIFINET_RECOVERY_F_BSS_JOINED flag, so will step into else branch
                    os_timer_ex_start(&pwdev_priv->connect_timeout);
                    if ((wifimac->wm_flags & WIFINET_F_SCAN) || (wifimac->wm_scan->scan_StateFlags & SCANSTATE_F_START)) {
                        wifi_mac_end_scan(wifimac->wm_scan);
                    }
                    //try to reconnect to BSS
                    wifi_mac_chk_scan(wnet_vif, WIFINET_SCANCFG_ACTIVE| WIFINET_SCANCFG_FLUSH, wnet_vif->vm_des_nssid, wnet_vif->vm_des_ssid);
                    break;
                case WIFINET_M_HOSTAP:
                case WIFINET_M_P2P_GO://no use
                    if (!WIFINET_IS_CHAN_ERR(wifimac->wm_stavif_channel)) {
                        AML_PRINT_LOG_INFO("need to switch channel\n");
                        wnet_vif->vm_curchan = wifimac->wm_stavif_channel;
                    }
                    wifimac->drv_priv->drv_ops.drv_set_bssid(wifimac->drv_priv, wnet_vif->wnet_vif_id, wnet_vif->vm_myaddr);
                    wifi_mac_top_sm(wnet_vif, WIFINET_S_SCAN, 0);
                    break;
                default:
                    AML_PRINT_LOG_ERR("unsupported vif opmode<%d>!\n", wnet_vif->vm_opmode);
                    break;
            }
        } else {
            wifi_mac_vif_restore_end(wnet_vif);
        }
        break;
    }
}

void wifi_mac_recovery_host_restore(struct wifi_mac *wifimac)
{
    /* This is called in recovery task
     * 1. For SAP mode, sta disassoc when host reset was called as a task, so that DISASSOC may not have been sent at the time.
     *    For consistency in timing, the restore action should also be run as a task
     * 2. For STA mode, it's similar to SAP mode, DEAUTH also run in task.
     **/
    wifi_mac_add_work_task(wifimac, wifi_mac_recovery_host_restore_task, NULL, (SYS_TYPE)wifimac, 0, 0, 0, 0);
}

void wifi_mac_process_recovery(struct wifi_mac *wifimac)
{
    struct wlan_net_vif *main_vmac = wifimac->drv_priv->drv_wnet_vif_table[NET80211_MAIN_VMAC];


    wifimac->drv_priv->hal_priv->fwRecoveryCnt++;
    wifimac->drv_priv->hal_priv->fwRecoveryStamp = jiffies;

    if (wifimac->wm_flags & WIFINET_F_SCAN) {
        AML_PRINT_LOG_INFO("recovery cancel scan\n");
        wifi_mac_cancel_scan(wifimac);
    }
    hi_get_irq_status();
    AML_PRINT_LOG_INFO("clear irq status done\n");
    wifi_mac_scan_forbidden(main_vmac, FORBIDDEN_SCAN_FOR_RECOVERY_TIMEOUT, FORBIDDEN_SCAN_FOR_RECOVERY);
    AML_PRINT_LOG_INFO("recovery forbid scan for all vifs!\n");

    //reset all running vifs to origin status
    wifimac->wmac_ops.wifi_mac_recovery_host_reset(wifimac);

    //power cycle and config fw
    wifimac->drv_priv->drv_ops.fw_repair(wifimac->drv_priv);

    //restore all running vifs to the status before recovery
    wifimac->wmac_ops.wifi_mac_recovery_host_restore(wifimac);

    if (aml_wifi_is_enable_rf_test()) {
        if (wifimac->rf_test_recover.ch_bw_link_val) {
             wifimac->drv_priv->drv_ops.drv_write_word(CH_BW_LINK_SRAM_ADDR,wifimac->rf_test_recover.ch_bw_link_val);
        }
        if (wifimac->rf_test_recover.primary_channel_val) {
            wifimac->drv_priv->drv_ops.drv_write_word(PHY_PRIMARY_CHANNEL,wifimac->rf_test_recover.primary_channel_val);
        }
        if (wifimac->rf_test_recover.power_tx_val) {
            wifimac->drv_priv->drv_ops.drv_write_word(RG_XMIT_A46,wifimac->rf_test_recover.power_tx_val);
        }

        gB2BTestCasePacket.packet_type = wifimac->rf_test_recover.packet_type;
        prepare_test_hal_layer_thr_init(gB2BTestCasePacket.packet_type);
    }
}

int aml_request_recovery(enum wifi_mac_recovery_source src)
{
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();

    if (os_timer_ex_active(&wifimac->wm_monitor_fw)
        && (BIT(src) & WIFINET_RECOVERY_SRC_MASK)
        && (!(wifimac->wm_recovery_flags & WIFINET_RECOVERY_F_RUNNING)
              || (src == WIFINET_RECOVERY_SRC_SDIO_TIMEOUT))) {
        wifimac->wm_recovery_src |= BIT(src);
        AML_PRINT_LOG_INFO("request recovery %d\n", src);
        return 0;
    }
    AML_PRINT_LOG_INFO("recovery flags %d, request bit %d\n", wifimac->wm_recovery_flags, src);
    return 1;
}

unsigned char wifi_mac_need_chip_reset(void)
{
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();

    return (wifimac->wm_recovery_level == WIFINET_RECOVERY_L_THOROUGH);
}

unsigned char is_in_recovery(struct wifi_mac *wifimac)
{
    if (wifimac->wm_recovery_flags & WIFINET_RECOVERY_F_RUNNING) {
        return 1;
    }

    return 0;
}

void wifi_mac_filter_special_data_frame(struct sk_buff *skb, SPECIAL_FRAME_STATUS_E frame_status)
{
    struct ethhdr *ethhdr = (struct ethhdr *)skb->data;
    char str[200] = {0};
    char *p_str = str;
    unsigned short offset = 0;

    //filter arp frame
    if (ethhdr->h_proto == htons(ETH_P_ARP)) {
        unsigned short op = (*(skb->data + ETHER_HDR_LEN + 6) << 8) | (*(skb->data + ETHER_HDR_LEN + 7));
        unsigned char *sender_mac = skb->data + ETHER_HDR_LEN + 8;
        unsigned char *sender_ip = skb->data + ETHER_HDR_LEN + 14;
        unsigned char *target_mac = skb->data + ETHER_HDR_LEN + 18;
        unsigned char *target_ip = skb->data + ETHER_HDR_LEN + 24;

        offset += sprintf(p_str + offset, special_frame_status_trace[frame_status]);
        if (op == 1) {
            offset += sprintf(p_str + offset, "arp req, ");
        }
        else if (op == 2) {
            offset += sprintf(p_str + offset, "arp rsp, ");
        }
        else {
            offset += sprintf(p_str + offset, "arp unknown:%x, ", op);
        }

        offset += sprintf(p_str + offset, "sender:[%02x:%02x:%02x:%02x:%02x:%02x, ",
                    sender_mac[0], sender_mac[1], sender_mac[2], sender_mac[3], sender_mac[4], sender_mac[5]);
        offset += sprintf(p_str + offset, "%d.%d.%d.%d], ",
                    sender_ip[0], sender_ip[1], sender_ip[2], sender_ip[3]);
        offset += sprintf(p_str + offset, "receiver:[%02x:%02x:%02x:%02x:%02x:%02x, ",
                    target_mac[0], target_mac[1], target_mac[2], target_mac[3], target_mac[4], target_mac[5]);
        offset += sprintf(p_str + offset, "%d.%d.%d.%d]",
                    target_ip[0], target_ip[1], target_ip[2], target_ip[3]);

        AML_PRINT(AML_LOG_ID_FILTER,AML_LOG_LEVEL_DEBUG, "%s\n", p_str);
    }
}

static void wifi_show_per_info(SYS_TYPE param1, SYS_TYPE param2,SYS_TYPE param3, SYS_TYPE param4,SYS_TYPE param5)
{
    struct hw_interface* hif = hif_get_hw_interface();
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    struct wlan_net_vif *wnet_vif = NULL;
    int avg_snr;

    AML_PRINT_LOG_INFO("ig0x%x,lpf0x%x,ct0x%x,at0x%x,fo0x%x,ag0x%x bs0x%x,0x%x,dpd0x%x,g0x%x,tf%d,to%d\n", hif->hif_ops.hi_read_word(0xa0816c), hif->hif_ops.hi_read_word(0xa081a4),
        hif->hif_ops.hi_read_word(DF_AGC_REG_A29), hif->hif_ops.hi_read_word(DF_AGC_REG_A30),
        hif->hif_ops.hi_read_word(0xf01024), hif->hif_ops.hi_read_word(DF_AGC_REG_A14),
        hif->hif_ops.hi_read_word(0xf02040), hif->hif_ops.hi_read_word(0xf03040),
        hif->hif_ops.hi_read_word(0xa0e4f0), hif->hif_ops.hi_read_word(0xa0e4b8),
        hif->HiStatus.tx_fail_num,  hif->HiStatus.tx_ok_num);
    if (wifimac->wm_curchan) {
        AML_PRINT_LOG_INFO("pc%d bw%d\n", wifimac->wm_curchan->chan_pri_num, wifimac->wm_curchan->chan_bw);
    }
    list_for_each_entry(wnet_vif, &wifimac->wm_wnet_vifs, vm_next) {
        if (wnet_vif->vm_curchan) {
            AML_PRINT_LOG_INFO("pc%d bw%d\n", wnet_vif->vm_curchan->chan_pri_num, wnet_vif->vm_curchan->chan_bw);
        }
        if (wnet_vif->vm_mainsta) {
            avg_snr = wnet_vif->vm_mainsta->sta_avg_snr;
            AML_PRINT_LOG_INFO("br%d,dr%d,c%d,s%d,n%d,tr%d,bw%d,rr%d,bw%d\n", wnet_vif->vm_mainsta->sta_avg_bcn_rssi,wnet_vif->vm_mainsta->sta_avg_data_rssi, wnet_vif->vm_wmac->drv_priv->drv_config.cfg_txpoweplan,
                avg_snr,wifi_mac_cal_noise(wnet_vif->vm_mainsta->sta_avg_bcn_rssi,avg_snr),
                wnet_vif->vm_mainsta->sta_last_txrate / 1000,wnet_vif->vm_mainsta->last_txrate_bw,wnet_vif->vm_mainsta->sta_last_rxrate / 1000,wnet_vif->vm_mainsta->last_rxrate_bw);
        }
        AML_PRINT_LOG_INFO("[%d]re%d,rp%d,te%d,tp%d\n", wnet_vif->wnet_vif_id,wnet_vif->vm_devstats.rx_errors,wnet_vif->vm_devstats.rx_packets,
            wnet_vif->vm_devstats.tx_errors,wnet_vif->vm_devstats.tx_packets);
    }
    phy_stc();
    cca_busy_check();
}

void wifi_mac_show_per_info(void)
{
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    wifi_mac_add_work_task(wifimac,wifi_show_per_info,NULL,0, 0, 0, 0, 0);
}

