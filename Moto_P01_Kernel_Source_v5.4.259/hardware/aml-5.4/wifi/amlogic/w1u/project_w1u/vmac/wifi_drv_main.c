/*
 ****************************************************************************************
 *
 * Copyright (C) Amlogic 2010-2014
 *
 * Project: 11N 80211 driver  layer Software
 *
 * Description:
 *  This file contains the main implementation of the DRIVER layer.  Most member
 *  functions of the DRIVER layer are defined here.
 *
 *
 ****************************************************************************************
 */
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <net/addrconf.h>
#include <net/sock.h>
#include <net/netlink.h>
#include "wifi_drv_power.h"
#include "wifi_drv_uapsd.h"
#include "wifi_drv_xmit.h"
#include "chip_bt_pmu_reg.h"
#include "wifi_drv_recv.h"
#include "wifi_mac_p2p.h"
#include "wifi_cfg80211.h"
#include "wifi_mac_concurrent.h"
#include "wifi_mac_if.h"
#include "wifi_mac_var.h"
#include "fi_cmd.h"
#include "wifi_mac_chan.h"
#include "wifi_hal_platform.h"
#include "wifi_mac_action.h"
#include "wifi_mac_main_reg.h"
#include "wifi_mac_tx_reg.h"
#include "wifi_mac_xmit.h"
#include "wifi_common.h"
#include "wifi_mac_amsdu.h"
#include "wifi_hal.h"

#define AML_TRACE_NL_PROTOCOL (28)


const unsigned char drv_bcast_mac[WIFINET_ADDR_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static unsigned int drv_phy_reg_sta_id( struct drv_private *drv_priv,
    unsigned char wnet_vif_id,unsigned short StaAid, unsigned char *pMac, unsigned char encrypt);

static struct drv_private drv_priv;
extern lp_shutdown_func g_lp_shutdown_func;
extern void hal_dpd_calibration(void);
extern struct notifier_block aml_inetaddr_cb;
extern struct notifier_block aml_inet6addr_cb;
extern int hal_tx_flush(unsigned char vid);
struct aml_trace_nl_info {
    struct sock * fw_log_sock;
    int user_pid;
    int enable;
};
enum {
    AML_TRACE_FW_LOG_START = 0xFF01,
    AML_TRACE_FW_LOG_STOP,
    AML_TRACE_FW_LOG_UPLOAD,
};
struct log_nl_msg_info {
    int msg_type;
    int msg_len;
};

struct aml_trace_nl_info g_trace_nl_info;

struct drv_private* drv_get_drv_priv(void)
{
    return &drv_priv;
}

int drv_rate_setup(struct drv_private *drv_priv, enum wifi_mac_macmode mode)
{

    struct drv_rate_table *rt = drv_hal_get_rate_tbl(mode);

    if (rt == NULL)
    {
        AML_PRINT_LOG_ERR("<running> error \n");
        return 0;
    }

    //do rate index and vendor_rate_code mapping.
    drv_hal_setupratetable(rt);
    DRV_MINSTREL_LOCK_INIT(drv_priv);
    drv_priv->drv_currratetable = rt;
    drv_priv->drv_protect_rateindex =  0;

    //do rate index and dot11Code mapping.
    if (drv_priv->net_ops->wifi_mac_rate_init)
    {
        drv_priv->net_ops->wifi_mac_rate_init(drv_priv->wmac, mode, rt);
    }
    return 1;
}



static void
drv_set_cfg_tx_power_limit( struct drv_private *drv_priv,
    unsigned short cfg_txpowlimit)
{

    drv_priv->drv_config.cfg_txpowlimit = cfg_txpowlimit;
}

void
drv_update_tx_pwr(struct drv_private *drv_priv, unsigned short txpowerdb)
{
    unsigned int txpow = 0, cfg_txpowlimit = 0;

    cfg_txpowlimit = drv_priv->drv_config.cfg_txpowlimit;

    if (drv_priv->drv_curtxpower != cfg_txpowlimit)
    {
        drv_priv->drv_curtxpower = txpow;
    }


    txpow = IW_MAX_TXPOWER;
    if (drv_priv->net_ops->wifi_mac_update_txpower)
        drv_priv->net_ops->wifi_mac_update_txpower(drv_priv->wmac,
            drv_priv->drv_curtxpower, txpow);
}

static void
drv_assoc_proc( struct drv_private *drv_priv,
    void * nsta, int isnew, int b_uapsd)
{
    struct aml_driver_nsta *drv_sta = DRIVER_NODE(nsta);
    int tid_index;

    if (!isnew)
    {
        for (tid_index = 0; tid_index < WME_NUM_TID; tid_index++)
        {
            if (drv_priv->drv_config.cfg_txaggr)
            {
                drv_txampdu_del(drv_priv,drv_sta,tid_index);
            }
            if (drv_priv->drv_config.cfg_rxaggr)
            {
                drv_rxampdu_del(drv_priv,drv_sta,tid_index);
            }
        }
    }
    drv_sta->sta_cleanup = 0;
    drv_sta->sta_powesave = 0;
    drv_sta->sta_isuapsd = 0;
    drv_sta->sta_isuapsd =  b_uapsd ;
}

int drv_channel_init(struct drv_private *drv_priv, unsigned int cc)
{
    unsigned int wMode;

    wMode = drv_priv->drv_config.cfg_modesupport;
    AML_PRINT_LOG_INFO("\n");
    if (drv_priv->net_ops->wifi_mac_setup_channel_list)
    {
        drv_priv->net_ops->wifi_mac_setup_channel_list(drv_priv->wmac, wMode,cc);
    }

    return 0;
}

static void
drv_update_slot( struct drv_private *drv_priv, int slottime)
{
    driv_ps_wakeup(drv_priv);
    drv_hal_setslottime( slottime);
    driv_ps_sleep(drv_priv);
}

static void rf_test_mode_recover(struct drv_private *drv_priv)
{
    struct wifi_mac *wifimac = drv_priv->wmac;
    unsigned int stop_tx = 0x08;
    AML_PRINT_LOG_INFO("rf_test mode is enable need stop tx ptk and recover\n");
    wifimac->rf_test_recover.packet_type = gB2BTestCasePacket.packet_type;
    gB2BTestCasePacket.packet_type = stop_tx;
    prepare_test_hal_layer_thr_init(stop_tx);
}

void aml_w1_fw_recovery(void *drv_priv)
{
    struct drv_private *p_drv_priv = (struct drv_private *)drv_priv;
    unsigned char res;

    if (aml_wifi_is_enable_rf_test()) {
        rf_test_mode_recover(p_drv_priv);
    }

    aml_disable_wifi();
    res = aml_enable_wifi();
    if ((aml_bus_type == AML_BUS_TYPE_SDIO) && !res) {
        aml_request_recovery(WIFINET_RECOVERY_SRC_SDIO_TIMEOUT);
        wifi_mac_get_repair_level();
        aml_disable_wifi();
        res = aml_enable_wifi();
        if (!res) {
            AML_PRINT_LOG_INFO("fw recovery fail!!!");
        }
    }

    if (p_drv_priv->hal_priv->hal_fw_log_flag) {
        hal_set_fwlog_cmd(FWLOG_AON_PIN_MUX_ENABLE);
    }
}

static void drv_scan_start_ex( SYS_TYPE param1,SYS_TYPE param2,
    SYS_TYPE param3,SYS_TYPE param4,SYS_TYPE param5)
{
    struct drv_private *drv_priv = (struct drv_private *)param1;
    unsigned int data = 0;

    if(drv_priv->drv_scanning == 0)
    {
        driv_ps_wakeup(drv_priv);
    }
    drv_priv->drv_scanning = 1;
    data = drv_priv->stop_noa_flag << 8 | drv_priv->drv_scanning;
    drv_hal_scancmd(data);
    //sleep at scan end
    // driv_ps_sleep(drv_priv);

}

static void drv_scan_start( struct drv_private *drv_priv)
{
    drv_hal_add_workitem((WorkHandler)drv_scan_start_ex, NULL, (SYS_TYPE)drv_priv, 0, 0, 0, 0);
}

static void drv_fw_recovery( struct drv_private *drv_priv)
{
    aml_w1_fw_recovery(drv_priv);
}


static void
drv_scan_end_ex( SYS_TYPE param1,SYS_TYPE param2,SYS_TYPE param3,
    SYS_TYPE param4,SYS_TYPE param5)
{
    struct drv_private *drv_priv = (struct drv_private *)param1;
    struct hal_private * hal_priv = hal_get_priv();
    unsigned char prescanning = drv_priv->drv_scanning;
    unsigned int data = 0;
    //wakeup at scan start
    // driv_ps_wakeup(drv_priv);
    drv_priv->drv_scanning = 0;
    data = drv_priv->stop_noa_flag << 8 | drv_priv->drv_scanning;
    drv_hal_scancmd(data);
    if (prescanning == 1) {
        driv_ps_sleep(drv_priv);
    }
    if ((aml_bus_type) && (hal_priv->dpd_delay_cail == 1)) {
        hal_dpd_calibration();
        hal_priv->dpd_delay_cail = 0;
    }
}

static void
drv_scan_end( struct drv_private *drv_priv)
{
    drv_hal_add_workitem((WorkHandler)drv_scan_end_ex,
        NULL, (SYS_TYPE)drv_priv, 0, 0, 0, 0);
}

// recv msg handl function
static void aml_recv_netlink(struct sk_buff *skb)
{
    struct nlmsghdr *nlh;
    struct log_nl_msg_info * nl_log_info = NULL;

    nlh = nlmsg_hdr(skb); // get msg body
    AML_PRINT_LOG_INFO("kernel rcv msg type: %d, pid: %d, len: %d, flag: %d, seq: %d\n",
        nlh->nlmsg_type, nlh->nlmsg_pid, nlh->nlmsg_len, nlh->nlmsg_flags, nlh->nlmsg_seq);
    nl_log_info = (struct nl_log_info*)NLMSG_DATA(nlh);
    switch (nl_log_info->msg_type) {
        case AML_TRACE_FW_LOG_START:
            g_trace_nl_info.user_pid = nlh->nlmsg_pid;
            g_trace_nl_info.enable = 1;
            AML_PRINT_LOG_INFO("user space process (pid: %d) start recv fw log !!!!\n", g_trace_nl_info.user_pid);
            break;
        case AML_TRACE_FW_LOG_STOP:
            g_trace_nl_info.enable = 0;
            AML_PRINT_LOG_INFO("user space process (pid: %d) stop recv fw log !!!!\n", g_trace_nl_info.user_pid);
            break;
        default:
            AML_PRINT_LOG_INFO("unknown msg (0x%x) from user space process (pid: %d), ignore !!!!\n",
                nl_log_info->msg_type, g_trace_nl_info.user_pid);
            break;
    }

    return;
}

int aml_log_nl_init(void)
{
    struct netlink_kernel_cfg cfg = {
        .input = aml_recv_netlink,
    };
    memset(&g_trace_nl_info, 0, sizeof(struct log_nl_msg_info));
    g_trace_nl_info.fw_log_sock = netlink_kernel_create(&init_net, AML_TRACE_NL_PROTOCOL, &cfg);
    if (!g_trace_nl_info.fw_log_sock) {
        AML_PRINT_LOG_INFO("aml trace netlink init failed");
        return -1;
    }

    AML_PRINT_LOG_INFO("aml trace netlink init OK!\n");
    return 0;
}
void aml_log_nl_deinit(void)
{
    if (g_trace_nl_info.fw_log_sock) {
        netlink_kernel_release(g_trace_nl_info.fw_log_sock);
    }
    AML_PRINT_LOG_INFO("fw log upload socket deinit!!!\n");

    return;
}
int aml_send_log_to_user(char *pbuf, uint16_t len, int msg_type)
{
    struct sk_buff *nl_skb;
    struct nlmsghdr *nlh = NULL;   //msg head
    struct log_nl_msg_info * nl_log_info = NULL;
    int ret;
    static int seq_num = 0;
    int buf_len = NLMSG_SPACE(len + sizeof(struct log_nl_msg_info));

    if (!g_trace_nl_info.fw_log_sock || !g_trace_nl_info.user_pid ||
        !g_trace_nl_info.enable) {
        AML_PRINT_LOG_INFO("kernel trace nl sock para invalid , can not upload msg to user\n");
        return -1;
    }
    //create sk_buff
    nl_skb = nlmsg_new(buf_len, GFP_ATOMIC);
    if (!nl_skb)
    {
        AML_PRINT_LOG_INFO("netlink alloc failure\n");
        return -1;
    }

    /* build netlink msg head */
    nlh = nlmsg_put(nl_skb, 0, 0, AML_TRACE_NL_PROTOCOL, buf_len, 0);
    if (nlh == NULL)
    {
        AML_PRINT_LOG_INFO("nlmsg_put failure \n");
        nlmsg_free(nl_skb);
        return -1;
    }
    NETLINK_CB(nl_skb).portid = 0;
    NETLINK_CB(nl_skb).dst_group = 0;
    nl_log_info = (struct nl_log_info*)nlmsg_data(nlh);
    nl_log_info->msg_len = len;
    nl_log_info->msg_type = msg_type;
    nlh->nlmsg_seq = seq_num++;

    /* copy data and send it */
    if (pbuf) {
        memcpy(nlmsg_data(nlh) + sizeof(struct log_nl_msg_info), pbuf, len);
    }
    ret = netlink_unicast(g_trace_nl_info.fw_log_sock, nl_skb, g_trace_nl_info.user_pid, 0);

    AML_PRINT_LOG_INFO("==== kernel upload msg to user result: %d, seq: %d\n", ret, seq_num - 1);
    return ret;

}

unsigned char print_ctl = 0;
extern unsigned char print_type;
static void drv_print_fwlog_ex( SYS_TYPE param1,SYS_TYPE param2,SYS_TYPE param3,
    SYS_TYPE param4,SYS_TYPE param5)
{
    unsigned char* logbuf_ptr = (unsigned char *)param1;
    int databyte = (int)param2;
    //unsigned char new_string[64] = {0};
    //int i = 0;

    if (print_type == 1) //auto
    {
        /*rotate print*/
        print_ctl = print_ctl ^ 1;
        databyte = databyte >> 1;
        if (print_ctl == 0)
        {
            logbuf_ptr += databyte;
        }
    }
    if (g_trace_nl_info.enable) {
        aml_send_log_to_user(logbuf_ptr, (int)param2, AML_TRACE_FW_LOG_UPLOAD);
    } else {
        storeFwlogToFile(logbuf_ptr, databyte);
    }
    /* print process */
    /*AML_PRINT_LOG_INFO("logbuf_ptr 0x%x\n", logbuf_ptr);
    AML_PRINT_LOG_INFO("fw_log:\n");
    while (databyte--)
    {
        if ((*logbuf_ptr == '\n') || (i == 63)) {
            new_string[i] = *logbuf_ptr;
            AML_PRINT_LOG_INFO("%s", new_string);
            memset(new_string, 0, 64);
            i = 0;
        } else {
            new_string[i] = *logbuf_ptr;
            i++;
        }

        logbuf_ptr++;
    }*/
    /* exit the loop and need to print the remaining characters */
    //AML_PRINT_LOG_INFO("%s", new_string);

    //AML_PRINT_LOG_INFO("\nfwlog_print end\n");
}

static void drv_print_fwlog(unsigned char *logbuf_ptr, int databyte)
{
    drv_hal_add_workitem((WorkHandler)drv_print_fwlog_ex,
        NULL, (SYS_TYPE)logbuf_ptr, (SYS_TYPE)databyte, 0, 0, 0);
}

static void
drv_connect_start_ex( SYS_TYPE param1,SYS_TYPE param2,
    SYS_TYPE param3,SYS_TYPE param4,SYS_TYPE param5)
{
    struct drv_private *drv_priv = (struct drv_private *)param1;

    if(drv_priv->drv_connetting == 0)
    {
        driv_ps_wakeup(drv_priv);
    }
    drv_priv->drv_connetting = 1;
}



static void
drv_connect_end_ex( SYS_TYPE param1,SYS_TYPE param2,
    SYS_TYPE param3,SYS_TYPE param4,SYS_TYPE param5)
{
    struct drv_private *drv_priv = (struct drv_private *)param1;

    unsigned char preconnetting =    drv_priv->drv_connetting ;

    drv_priv->drv_connetting = 0 ;
    if(preconnetting == 1)
    {
        driv_ps_sleep(drv_priv);

    }

}
static void
drv_connect_start( struct drv_private *drv_priv)
{
    drv_hal_add_workitem((WorkHandler)drv_connect_start_ex,
        NULL, (SYS_TYPE)drv_priv, 0, 0, 0, 0);
}

static void
drv_connect_end( struct drv_private *drv_priv)
{
    drv_hal_add_workitem((WorkHandler)drv_connect_end_ex,
        NULL, (SYS_TYPE)drv_priv, 0, 0, 0, 0);
}

static void drv_set_channel_rssi_ex( SYS_TYPE param1,SYS_TYPE param2,SYS_TYPE param3, SYS_TYPE param4,SYS_TYPE param5)
{
    struct hal_private* hal_priv = hal_get_priv();

    hal_priv->hal_ops.phy_set_channel_rssi(param1, param2);
}


static void drv_set_channel_rssi( struct drv_private *drv_priv, unsigned char rssi, unsigned char flag)
{
    static unsigned char pre_rssi = 0;

    if (pre_rssi != rssi) {
        pre_rssi = rssi;
        drv_hal_add_workitem((WorkHandler)drv_set_channel_rssi_ex, NULL, (SYS_TYPE)rssi, (SYS_TYPE)flag, 0, 0, 0);
    }
}

static void drv_set_tx_power_accord_rssi_ex( SYS_TYPE param1,SYS_TYPE param2,SYS_TYPE param3, SYS_TYPE param4,SYS_TYPE param5)
{
    struct hal_private* hal_priv = hal_get_priv();

    hal_priv->hal_ops.phy_set_tx_power_accord_rssi(param1, param2, param3, param4);
}


static void drv_set_tx_power_accord_rssi( struct drv_private *drv_priv, struct hal_channel *hchan, unsigned char rssi, unsigned char power_mode)
{
    drv_hal_add_workitem((WorkHandler)drv_set_tx_power_accord_rssi_ex, NULL, (SYS_TYPE)hchan->chan_bw, (SYS_TYPE)hchan->channel, (SYS_TYPE)rssi, (SYS_TYPE)power_mode, 0);
}

static void drv_cfg_txpwr_cffc_param_ex(SYS_TYPE param1,SYS_TYPE param2,SYS_TYPE param3, SYS_TYPE param4,SYS_TYPE param5)
{
    struct hal_private* hal_priv = hal_get_priv();

    hal_priv->hal_ops.hal_cfg_txpwr_cffc_param((void*)param1, (void*)param2);
}

static void drv_cfg_txpwr_cffc_param(struct wifi_channel * chan, struct tx_power_plan * txpwr_plan)
{
    drv_hal_add_workitem((WorkHandler)drv_cfg_txpwr_cffc_param_ex, NULL, (SYS_TYPE)chan, (SYS_TYPE)txpwr_plan, 0, 0, 0);
}


static void
drv_set_channel( struct drv_private *drv_priv, struct hal_channel *hchan, unsigned char flag, unsigned char vid)
{
#ifdef RF_T9026
    struct hal_private* hal_priv = hal_get_priv();
#endif
    AML_PRINT(AML_LOG_ID_BWC,AML_LOG_LEVEL_DEBUG,"ieee_chan=%d, pn:%d, cn:%d, bw=%d, drv_chan:%d, drv_pn:%d, drv_cn:%d, drv_bw:%d \n",
            hchan->pchan_num, hchan->pchan_num, hchan->cchan_num, hchan->chan_bw,
            drv_priv->drv_curchannel.channel, drv_priv->drv_curchannel.pchan_num,
            drv_priv->drv_curchannel.cchan_num, drv_priv->drv_curchannel.chan_bw);

#ifdef RF_T9026
    if (hal_priv->bRfInit) {
#endif
        driv_ps_wakeup(drv_priv);

        if (hchan != NULL) {
            static unsigned long switch_time;
            if ((OS_GET_TIMESTAMP() - switch_time) < MIN_DWELL_DUR) {
                AML_PRINT_LOG_ERR("chan switch time less,pchan_num:%d cur:%d,time:%d\n",
                    hchan->pchan_num,
                    drv_priv->drv_curchannel.pchan_num,
                    OS_GET_TIMESTAMP() - switch_time);
            }
            drv_hal_setchannel(hchan, flag, vid);
            switch_time = OS_GET_TIMESTAMP();
            drv_hal_set_chan_support(hchan);
            drv_priv->drv_curchannel = *hchan;

            AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "channel_freq %d pchan_num %d cchan_num %d chan_bw %d\n",
                hchan->channel, hchan->pchan_num, hchan->cchan_num, hchan->chan_bw);
        }
        driv_ps_sleep(drv_priv);
#ifdef RF_T9026
    }
#endif
}

static int drv_set_bssid(struct drv_private *drv_priv, unsigned char wnet_vif_id,unsigned char *bssid)
{
    int err;
    struct hal_private* hal_priv = hal_get_priv();

    memcpy((void *)(drv_priv->drv_bssid), bssid, WIFINET_ADDR_LEN);
    driv_ps_wakeup(drv_priv);
    err = hal_priv->hal_ops.phy_set_mac_bssid(wnet_vif_id,bssid);
    driv_ps_sleep(drv_priv);

    return err;
}

static void drv_set_mac_addr( struct drv_private *drv_priv, unsigned char wnet_vif_id, unsigned char *macaddr)
{
    driv_ps_wakeup(drv_priv);
    drv_hal_setmac(wnet_vif_id, macaddr);
    driv_ps_sleep(drv_priv);
}

static void
drv_bcn_init( struct drv_private *drv_priv, const  unsigned char wnet_vif_id,
    unsigned int _bperiod)
{
    AML_PRINT_LOG_INFO("<running>\n");
    driv_ps_wakeup(drv_priv);
    drv_hal_beaconinit( wnet_vif_id, _bperiod);
    driv_ps_sleep(drv_priv);
}
static void
drv_set_bcn_start( struct drv_private *drv_priv, unsigned char wnet_vif_id,
    unsigned short intval, unsigned char dtim_count,unsigned short bsstype)
{
    AML_PRINT_LOG_INFO("<running>\n");
    driv_ps_wakeup(drv_priv);
    drv_hal_set_bcn_start( wnet_vif_id, intval,  dtim_count, bsstype);
    driv_ps_sleep(drv_priv);
}

static void
drv_put_bcn_buf( struct drv_private *drv_priv, unsigned char wnet_vif_id,
    unsigned char *pBeacon, unsigned short len,unsigned char Rate,unsigned short Flag)
{
    //AML_PRINT_LOG_INFO("<running>\n");
    driv_ps_wakeup(drv_priv);
    drv_hal_put_bcn_buf(wnet_vif_id,pBeacon,  len, Rate, Flag);
    driv_ps_sleep(drv_priv);
}

static int
drv_hal_set_p2p_opps_cwend_enable(struct drv_private *drv_priv,
    unsigned char vid, unsigned char p2p_oppps_cw)
{

    AML_PRINT(AML_LOG_ID_HAL, AML_LOG_LEVEL_DEBUG, "vid=%d p2p_oppps_cw=%xTU\n",vid, p2p_oppps_cw);
    return drv_priv->hal_priv->hal_ops.phy_set_p2p_opps_cwend_enable(vid, p2p_oppps_cw);
}
static int
drv_hal_set_p2p_noa_enable(struct drv_private *drv_priv,
    unsigned char vid,unsigned int duration, unsigned int interval,
    unsigned int starttime, unsigned char count,unsigned char flag)
{
    unsigned char lflag = 0;
    AML_PRINT(AML_LOG_ID_HAL, AML_LOG_LEVEL_DEBUG,"vid=%d count=%d\n",vid, count);
    if (flag & P2P_NOA_START_MATCH_BEACON_HI)
    {
        lflag |= P2P_NOA_START_MATCH_BEACON;
    }
    else if (flag & P2P_NOA_END_MATCH_BEACON_HI)
    {
        lflag |= P2P_NOA_END_MATCH_BEACON;
    }
    return drv_priv->hal_priv->hal_ops.phy_set_p2p_noa_enable(vid, duration,
                interval, starttime, count, lflag);
}
static unsigned long long
drv_hal_get_tsf(struct drv_private *drv_priv, unsigned char vid)
{
    AML_PRINT(AML_LOG_ID_HAL, AML_LOG_LEVEL_DEBUG, "vid=%d\n", vid);
    return drv_priv->hal_priv->hal_ops.phy_get_tsf(vid);
}

static int drv_send_null_data(struct drv_private *drv_priv,
    struct NullDataCmd null_data , int len)
{
    AML_PRINT(AML_LOG_ID_HAL, AML_LOG_LEVEL_DEBUG, "vid=%d, ps=%d\n", null_data.vid, null_data.pwr_save);

    return drv_priv->hal_priv->hal_ops.phy_send_null_data(null_data, len);
}

static int drv_keep_alive(struct drv_private *drv_priv,
    struct NullDataCmd null_data,int len, int enable, int period)
{
    AML_PRINT(AML_LOG_ID_HAL, AML_LOG_LEVEL_DEBUG, "enable=%d, vid=%d, period=%d\n",
        enable, null_data.vid, period);

    return drv_priv->hal_priv->hal_ops.phy_keep_alive(null_data, len, enable, period);
}

static int drv_set_beacon_miss(struct drv_private *drv_priv,
    unsigned char wnet_vif_id,unsigned char enable, int period)
{
    AML_PRINT(AML_LOG_ID_HAL, AML_LOG_LEVEL_DEBUG, "vid=%d, enable=%d, period=%d\n",
        wnet_vif_id, enable, period);
    return drv_priv->hal_priv->hal_ops.phy_set_beacon_miss(wnet_vif_id, enable, period);
}

static int drv_set_vsdb(struct drv_private *drv_priv,
    unsigned char wnet_vif_id,unsigned char enable)
{
    AML_PRINT(AML_LOG_ID_HAL, AML_LOG_LEVEL_DEBUG, "vid=%d, enable=%d\n",
        wnet_vif_id, enable);
    return drv_priv->hal_priv->hal_ops.phy_set_vsdb(wnet_vif_id, enable);
}

static int
drv_set_arp_agent(struct drv_private *drv_priv, unsigned char wnet_vif_id,
    unsigned char enable, unsigned int ipv4, unsigned char *ipv6, unsigned char * dhcp_server_mac)
{
    AML_PRINT(AML_LOG_ID_HAL, AML_LOG_LEVEL_DEBUG, "vid=%d, enable=%d, ipv4=0x%x\n",
        wnet_vif_id, enable, ipv4);
    return drv_priv->hal_priv->hal_ops.phy_set_arp_agent(wnet_vif_id, enable, ipv4, ipv6, dhcp_server_mac);
}

static int
drv_set_pattern(struct drv_private *drv_priv, unsigned char vid,
    unsigned char offset,unsigned char len, unsigned char id,
    unsigned char *mask, unsigned char *pattern)
{
    AML_PRINT(AML_LOG_ID_HAL, AML_LOG_LEVEL_DEBUG, "vid=%d, pattern len=%d mask=0x%x\n",
        vid, len, *(unsigned int*)mask);
    return drv_priv->hal_priv->hal_ops.phy_set_pattern(vid, offset, len, id, mask, pattern);
}

static int drv_set_suspend(struct drv_private *drv_priv,
    unsigned char vid, unsigned char enable,
    unsigned char mode, unsigned int filters)
{
    AML_PRINT(AML_LOG_ID_HAL, AML_LOG_LEVEL_DEBUG, "vid=%d, enable=%d filters=0x%x\n",
        vid, enable, filters);

    if (enable == 1)
        drv_priv->dot11VmacPsState[vid] = DRV_PWRSAVE_FULL_SLEEP;
    else
        drv_priv->dot11VmacPsState[vid] = DRV_PWRSAVE_AWAKE;
    return drv_priv->hal_priv->hal_ops.phy_set_suspend(vid, enable, mode, filters);
}

int  drv_hal_tx_frm_pause(struct drv_private *drv_priv, int pause)
{
    AML_PRINT(AML_LOG_ID_HAL, AML_LOG_LEVEL_DEBUG, "pause=%d\n",pause);
    return drv_priv->hal_priv->hal_ops.hal_txframe_pause(pause);
}

static void
clear_staid_and_bssid_ex(SYS_TYPE param1,
    SYS_TYPE param2, SYS_TYPE param3,
    SYS_TYPE param4,SYS_TYPE param5)
{
    struct drv_private *drv_priv = (void *)param1;
    unsigned char wnet_vif_id = param2;
    unsigned short sta_id = param3;
    unsigned char bssid[6] = {0};
    struct wlan_net_vif *wnet_vif;

    wnet_vif = drv_priv->drv_wnet_vif_table[wnet_vif_id];
    if (wnet_vif == NULL)
    {
        return ;
    }

    driv_ps_wakeup(drv_priv);
    hal_phy_unregister_sta_id(wnet_vif_id,sta_id);
    if (wnet_vif->vm_hal_opmode == WIFI_M_STA) {
        drv_set_bssid(drv_priv, wnet_vif_id, bssid);
    }
    driv_ps_sleep(drv_priv);
    return ;
}

static int
drv_clear_staid_and_bssid(struct drv_private *drv_priv,
    unsigned char wnet_vif_id, unsigned short sta_id)
{
    drv_hal_add_workitem((WorkHandler)clear_staid_and_bssid_ex,NULL,
        (SYS_TYPE)drv_priv,(SYS_TYPE)wnet_vif_id,
        (SYS_TYPE)sta_id,(SYS_TYPE)0, (SYS_TYPE)0);
    return 0;
}

static void
drv_wnet_vif_disconnect_ex(SYS_TYPE param1,
    SYS_TYPE param2, SYS_TYPE param3,
    SYS_TYPE param4,SYS_TYPE param5)
{
    struct drv_private *drv_priv = (void *)param1;
    int wnet_vif_id = param2;

    driv_ps_wakeup(drv_priv);
    drv_hal_wnet_vifdisconnect(wnet_vif_id);
    driv_ps_sleep(drv_priv);
    return ;
}

static int
drv_wnet_vif_disconnect(struct drv_private *drv_priv,
    int wnet_vif_id)
{
    drv_hal_add_workitem((WorkHandler)drv_wnet_vif_disconnect_ex, NULL,
        (SYS_TYPE)drv_priv, (SYS_TYPE)wnet_vif_id, (SYS_TYPE)0, (SYS_TYPE)0,  (SYS_TYPE)0);
    return 0;
}

static int
drv_open( struct drv_private *drv_priv)
{
    struct wifi_mac *wifimac = NULL;
    int error = 0;
    wifimac = (struct wifi_mac *)(drv_priv->wmac);

    driv_ps_wakeup(drv_priv);
    drv_txlist_flushfree(drv_priv, 3);

    if (!drv_hal_reset())
    {
        AML_PRINT_LOG_ERR("unable to reset chip\n");
        error = -EIO;
        goto done;
    }

    drv_hal_enable();
    memset((void *)(&drv_priv->drv_curchannel), 0, sizeof(struct hal_channel));
    drv_priv->drv_not_init_flag = 0;

done:
    driv_ps_sleep(drv_priv);
    return error;
}


static
void drv_get_sts( struct drv_private *drv_priv,
    unsigned int op_code, unsigned int ctrl_code)
{
    int i;
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    struct drv_txlist *txlist = NULL;
    struct drv_tx_scoreboard *tid;
    struct wifi_station *sta = NULL;
    struct aml_driver_nsta *drv_sta = NULL;
    struct wlan_net_vif *connect_wnet = wifi_mac_running_wnet_vif(wifimac);

    if (connect_wnet != NULL) {
        sta = connect_wnet->vm_mainsta;
        drv_sta = sta->drv_sta;
        AML_PRINT_LOG_INFO("\n--------drv statistic--------\n");
        for (i=0; i< WME_NUM_AC;i++) {
            txlist = drv_txlist_initial(drv_priv,i);
            AML_PRINT_LOG_INFO("queen %d ac_queue_buffer_in_drv %d\n", i, txlist->txds_pending_cnt);

        }
        for (i=0; i< WME_NUM_TID;i++) {
            tid = &drv_sta->tx_agg_st.tid[i];
            AML_PRINT_LOG_INFO("tid %d tid_queue_buffer_in_drv %d, msdu_count %d\n", i,WIFINET_SAVEQ_QLEN(&tid->tid_tx_buffer_queue), wifimac->msdu_cnt[i]);

        }
        AML_PRINT_LOG_INFO("tx_buffer_queue_in_drv %d\n",WIFINET_SAVEQ_QLEN(&connect_wnet->vm_tx_buffer_queue));
    }

    if((ctrl_code & STS_MOD_DRV) == STS_MOD_DRV)
    {
        if ((ctrl_code & STS_TYP_TX) == STS_TYP_TX)
        {
            for(i = 0; i < HAL_NUM_TX_QUEUES; i++)
            {
                AML_PRINT_LOG_INFO("txlist_idx %d,  mpdu_to_hal %d,mpdu_pend %d, "
                        "mpdu_bkup %d,last_time %ld\n",
                        drv_priv->drv_txlist_table[i].txlist_qnum,
                        drv_priv->drv_txlist_table[i].txlist_qcnt,
                        drv_priv->drv_txlist_table[i].txds_pending_cnt,
                        drv_priv->drv_txlist_table[i].txlist_backup_qcnt,
                        drv_priv->drv_txlist_table[i].txlist_lasttime);
            }

            AML_PRINT_LOG_INFO("--**--tx_retry_cnt %d\n", drv_priv->drv_stats.tx_retry_cnt);
            AML_PRINT_LOG_INFO("--**--tx_short_retry_cnt %d\n", drv_priv->drv_stats.tx_short_retry_cnt);
            AML_PRINT_LOG_INFO("tx_ampdu_mutil %d\n", drv_priv->drv_stats.tx_pkts_cnt);
            AML_PRINT_LOG_INFO("tx_ampdu_uniq %d\n", drv_priv->drv_stats.tx_ampdu_one_frame_cnt);
            //AML_PRINT_LOG_INFO("tx_ampdu pend %d\n", drv_priv->drv_stats.txds_pending_cnt);
            AML_PRINT_LOG_INFO("tx_normal mpdu %d\n", drv_priv->drv_stats.tx_normal_cnt);

            AML_PRINT_LOG_INFO("tx_ampdu_fail %d\n", drv_priv->drv_stats.tx_end_fail_cnt);
            //AML_PRINT_LOG_INFO("tx_drops %d\n", drv_priv->drv_stats.tx_drops_cnt);

            AML_PRINT_LOG_INFO("tx_end_normal %d\n", drv_priv->drv_stats.tx_end_normal_cnt);
            //  AML_PRINT_LOG_INFO("tx_end_ampdu %d\n", drv_priv->drv_stats.tx_end_ampdu_cnt);
            //  AML_PRINT_LOG_INFO("tx_ampdu %d\n", drv_priv->drv_stats.tx_ampdu_cnt);
        }
        /*drv rx */

        if ((ctrl_code & STS_TYP_RX) == STS_TYP_RX)
        {
            AML_PRINT_LOG_INFO("rx_indicate %d\n", drv_priv->drv_stats.rx_indicate_cnt);
            AML_PRINT_LOG_INFO("rx_free_skb %d\n", drv_priv->drv_stats.rx_free_skb_cnt);
            AML_PRINT_LOG_INFO("rx_ampdu %d\n", drv_priv->drv_stats.rx_ampdu_cnt);
            AML_PRINT_LOG_INFO("rx_bar %d\n", drv_priv->drv_stats.rx_bar_cnt);
            AML_PRINT_LOG_INFO("rx_nonqos %d\n", drv_priv->drv_stats.rx_nonqos_cnt);
            AML_PRINT_LOG_INFO("rx_dup %d\n", drv_priv->drv_stats.rx_dup_cnt);
            AML_PRINT_LOG_INFO("rx_ok %d\n", drv_priv->drv_stats.rx_ok_cnt);
            AML_PRINT_LOG_INFO("rx_bardrop %d\n", drv_priv->drv_stats.rx_bardrop_cnt);
            AML_PRINT_LOG_INFO("rx_skipped %d\n", drv_priv->drv_stats.rx_skipped_cnt);
            AML_PRINT_LOG_INFO("--**--rx_ack_rssi %d\n", drv_priv->drv_stats.rx_ack_rssi);
        }

    }

    drv_priv->hal_priv->hal_ops.hal_get_sts(op_code, ctrl_code);
}


static int
drv_stop( struct drv_private *drv_priv)
{
    driv_ps_wakeup(drv_priv);
    drv_priv->drv_config.cfg_rtcenable = 0;
    driv_ps_sleep(drv_priv);
    return 0;
}

int drv_reset( void * dev)
{
    struct drv_private *drv_priv = ( struct drv_private *) dev;
    int error = 0;

    driv_ps_wakeup(drv_priv);

    if (!drv_hal_reset())
    {
        AML_PRINT_LOG_ERR("unable to reset chip\n");
        error = -EIO;
    }

    driv_ps_sleep(drv_priv);
    return error;
}

static int drv_suspend( void * dev)
{
    struct drv_private *drv_priv = ( struct drv_private *) dev;
    driv_ps_wakeup(drv_priv);

    drv_hal_disable();
    driv_ps_sleep(drv_priv);
    msleep(10);

    if (drv_priv->drv_not_init_flag)
        return -EIO;

    /* stop the hardware */
    drv_stop(drv_priv);

    drv_priv->drv_not_init_flag = 1;

    AML_PRINT_LOG_INFO("<running>\n");

    return 0;
}

static void
drv_set_chan_bw_mode_orig( SYS_TYPE param1,
    SYS_TYPE param2,SYS_TYPE param3,
    SYS_TYPE param4,SYS_TYPE param5)
{
    struct drv_private *drv_priv =(struct drv_private *)param1;
    int chanbw = (int) param2;
    driv_ps_wakeup(drv_priv);
    drv_hal_set11nmac2040( chanbw);
    driv_ps_sleep(drv_priv);
}

static void
drv_set_chan_bw_mode( struct drv_private *drv_priv,
    enum wifi_mac_chanbw  bw)
{
    drv_hal_add_workitem((WorkHandler)drv_set_chan_bw_mode_orig,
            NULL,(SYS_TYPE) drv_priv,(SYS_TYPE) bw,0,0,0);
}

static int
drv_get_scn_chan_busy( struct drv_private *drv_priv)
{
    int ret;
    driv_ps_wakeup(drv_priv);
    ret = drv_hal_get11nextbusy();
    driv_ps_sleep(drv_priv);
    return ret;
}

static unsigned int
drv_low_reg_behind_task( struct drv_private *drv_priv,void *func)
{
    return drv_low_register_behind_task(func);
}

static unsigned int
drv_low_call_task( struct drv_private *drv_priv,
    SYS_TYPE taskid,SYS_TYPE param1)
{
    return drv_low_call_register_task(taskid,param1);
}

unsigned int drv_low_add_worktask( struct drv_private *drv_priv,
    void *func,void *func_cb,SYS_TYPE param1,SYS_TYPE param2,
    SYS_TYPE param3,SYS_TYPE param4,SYS_TYPE param5)
{
    return drv_hal_add_workitem(func,func_cb,param1, param2,param3, param4,param5);
}

static unsigned int
drv_phy_reg_sta_id( struct drv_private *drv_priv,
    unsigned char wnet_vif_id,unsigned short StaAid,
    unsigned char *pMac, unsigned char encrypt)
{
    unsigned int ret;
    driv_ps_wakeup(drv_priv);
    ret = hal_phy_register_sta_id(wnet_vif_id,StaAid,pMac, encrypt);
    driv_ps_sleep(drv_priv);

    return ret;
}

static unsigned int drv_phy_unreg_all_sta_id(struct drv_private *drv_priv, unsigned char wnet_vif_id)
{
    unsigned int ret;
    driv_ps_wakeup(drv_priv);
    ret =  hal_phy_unregister_all_sta_id(wnet_vif_id);
    driv_ps_sleep(drv_priv);

    return ret;
}

int drv_add_wnet_vif(struct drv_private *drv_priv,
    int wnet_vif_id, void * if_data, enum hal_op_mode vm_opmode, unsigned char *myaddr, unsigned int ip)
{
    struct wlan_net_vif *wnet_vif;
    int ret = 0;

    if (wnet_vif_id >= DEFAULT_MAX_VMAC) {
        return -EINVAL;
    }

    if ((drv_priv->drv_wnet_vif_table[wnet_vif_id] != NULL)
        && (drv_priv->drv_wnet_vif_table[wnet_vif_id]->vm_recovery_state == WIFINET_RECOVERY_END)) {
        AML_PRINT_LOG_ERR("Invalid interface id = %u\n", wnet_vif_id);
        return -EINVAL;
    }

    driv_ps_wakeup(drv_priv);
    AML_PRINT_LOG_INFO("macaddr "MAC_FMT"\n", MAC_ARG(myaddr));

    if (vm_opmode == WIFI_M_HOSTAP) {
        /* copy nostabeacons - for WDS client */
        /* note: not right, beacon buffer is allocated on RUN trans */
        drv_set_bssid(drv_priv, wnet_vif_id, myaddr);
    }

    wnet_vif = (struct wlan_net_vif *)if_data;
    /* Set the VMAC opmode */
    wnet_vif->vm_hal_opmode = vm_opmode;
    AML_PRINT_LOG_INFO("<%s> hal_opmode=%d \n", VMAC_DEV_NAME(wnet_vif), wnet_vif->vm_hal_opmode);

    drv_priv->drv_wnet_vif_table[wnet_vif_id] = wnet_vif;
    drv_priv->drv_wnet_vif_num++;

#ifdef  AML_MCAST_QUEUE
    drv_tx_mcastq_init(drv_priv, wnet_vif_id);
#endif

    drv_hal_setmac( wnet_vif_id, myaddr);
    drv_hal_setopmode( wnet_vif_id, vm_opmode);
    drv_hal_setdhcp(wnet_vif_id, ip);
    driv_ps_sleep(drv_priv);
    return ret;
}
/*
TxBufLock
*/
static int drv_delete_wnet_vif(struct drv_private * drv_priv, int wnet_vif_id)
{
    int flags;
    int ret = 0;

    driv_ps_wakeup(drv_priv);

    flags = drv_priv->net_ops->wifi_mac_get_netif_cfg(drv_priv->wmac);

#ifdef  AML_MCAST_QUEUE
    drv_tx_mcastq_cleanup(drv_priv, wnet_vif_id);
#endif

    drv_priv->drv_wnet_vif_table[wnet_vif_id] = NULL;
    drv_priv->drv_wnet_vif_num--;

    if ((flags & NETCOM_NETIF_RUNNING) && drv_priv->drv_wnet_vif_num)
    {
        drv_hal_interset(1);
    }

    drv_hal_wnet_vifStop(wnet_vif_id);
    driv_ps_sleep(drv_priv);
    return ret ;
}
static int
drv_change_wnet_vif(struct drv_private * drv_priv, int wnet_vif_id,
    void * if_data, enum hal_op_mode vm_opmode,  unsigned char *myaddr, unsigned int ip)
{
    struct wlan_net_vif *wnet_vif;
    int ret = 0;

    if (wnet_vif_id >= DEFAULT_MAX_VMAC
        || drv_priv->drv_wnet_vif_table[wnet_vif_id] == NULL)
    {
        AML_PRINT_LOG_ERR("Invalid interface id = %u\n", wnet_vif_id);
        return -EINVAL;
    }
    driv_ps_wakeup(drv_priv);
    wnet_vif = drv_priv->drv_wnet_vif_table[wnet_vif_id];

    if (vm_opmode == WIFI_M_HOSTAP)
        drv_set_bssid(drv_priv, wnet_vif_id, myaddr);

    wnet_vif->vm_hal_opmode = vm_opmode;

    drv_hal_setmac(wnet_vif_id, myaddr);
    drv_hal_setopmode(wnet_vif_id, vm_opmode);
    drv_hal_setdhcp(wnet_vif_id, ip);
    driv_ps_sleep(drv_priv);
    return ret ;
}


static void *
drv_nsta_attach( struct drv_private *drv_priv, int wnet_vif_id, void* sta)
{
    struct aml_driver_nsta *drv_sta;

    drv_sta = (struct aml_driver_nsta *)NET_MALLOC(sizeof(struct aml_driver_nsta), GFP_ATOMIC, "drv_nsta_attach.drv_sta");
    if (drv_sta == NULL)
    {
        AML_PRINT_LOG_ERR("<running> drv_sta is NULL \n");
        return NULL;
    }

    drv_sta->net_nsta = sta;
    drv_sta->sta_drv_priv = drv_priv;
    drv_sta->sta_rc_nsta = ZMALLOC(drv_priv->drv_ratectrl_size,"drv_sta->sta_rc_nsta", GFP_ATOMIC);

    if (drv_sta->sta_rc_nsta == NULL)
    {
        AML_PRINT_LOG_ERR("<running> drv_sta->sta_rc_nsta == NULL\n");
        FREE(drv_sta, "drv_nsta_attach.drv_sta");
        return NULL;
    }

    AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_DEBUG, " drv_ratectrl_size = %d sta_rc_nsta = %p\n",
        drv_priv->drv_ratectrl_size, drv_sta->sta_rc_nsta);

    /* set up per-nsta tx/rx state */
    drv_txlist_init_for_sta(drv_priv, drv_sta, wnet_vif_id);
    drv_rx_nsta_init(drv_priv, drv_sta);
    drv_tx_uapsd_nsta_init(drv_priv, drv_sta);

#ifdef DRV_SUPPORT_TX_WITHDRAW
    drv_tx_withdraw_init(drv_priv, drv_sta);
#endif

    AML_PRINT(AML_LOG_ID_LOG,AML_LOG_LEVEL_DEBUG, "<running>  \n");

    return drv_sta;
}

static void
drv_nsta_detach(struct drv_private * drv_priv, void * nsta)
{
    struct aml_driver_nsta *drv_sta = DRIVER_NODE(nsta);

    drv_txlist_free_for_sta(drv_priv, drv_sta);
    drv_rx_nsta_free(drv_priv, drv_sta);

    AML_PRINT_LOG_INFO("drv_sta:%p\n", drv_sta);
    FREE(drv_sta->sta_rc_nsta, "drv_sta->sta_rc_nsta");
    FREE(drv_sta, "drv_nsta_attach.drv_sta");
}

static void
drv_nsta_cleanup( struct drv_private *drv_priv, void * nsta)
{
    struct aml_driver_nsta *drv_sta = DRIVER_NODE(nsta);

    drv_sta->sta_cleanup = 1;
    if (drv_sta->sta_isuapsd)
    {
        drv_sta->sta_isuapsd = 0;
        drv_tx_uapsd_nsta_cleanup(drv_priv, drv_sta);
    }

#ifdef DRV_SUPPORT_TX_WITHDRAW
    drv_tx_withdraw_cleanup(drv_priv, drv_sta);
#endif

    drv_txlist_cleanup_for_sta(drv_priv, drv_sta);
    drv_rx_nsta_clean(drv_priv, drv_sta);
}

static void
drv_nsta_update_pwrsave(struct drv_private *drv_priv,
    void * nsta, int pwrsave)
{
    struct aml_driver_nsta *drv_sta = DRIVER_NODE(nsta);
    if (pwrsave)
    {
        drv_sta->sta_powesave = 1;
        drv_txlist_pause_for_sta(drv_priv,drv_sta);
    }
    else
    {
        drv_sta->sta_powesave = 0;
        drv_txlist_resume_for_sta(drv_priv,drv_sta);
    }
}

static void
drv_key_rst_ex(SYS_TYPE param1,SYS_TYPE param2,
    SYS_TYPE param3,SYS_TYPE param4,SYS_TYPE param5)
{
    struct drv_private *drv_priv = (struct drv_private *)param1;
    unsigned char wnet_vif_id = (unsigned char)param2;
    unsigned short key_index = (unsigned short )param3;
    int staid = (int)param4;
    unsigned char group = (unsigned char)param5;
    struct wlan_net_vif *wnet_vif = drv_priv->drv_wnet_vif_table[wnet_vif_id];

    driv_ps_wakeup(drv_priv);

    AML_PRINT_LOG_INFO("vid:%d, aid:%d, kid:%d, vm_opmode:%d, vm_phase_flags:0x%x\n",
                wnet_vif_id, staid, key_index, wnet_vif->vm_opmode, wnet_vif->vm_phase_flags);

    if (staid == 0) {
        drv_hal_keyreset(wnet_vif_id, key_index);
    } else {
        drv_hal_keyclear(wnet_vif_id, staid);
    }
    wnet_vif->vm_key_bitmap &= ~(group ? BIT(key_index) : BIT(key_index + WIFINET_UKEY_BITMAP_OFT));
    if ((wnet_vif->vm_opmode == WIFINET_M_STA) &&
         (wnet_vif->vm_phase_flags & PHASE_DISCONNECTING) &&
         wnet_vif->vm_key_bitmap == 0) {
        wnet_vif->vm_phase_flags &= ~PHASE_DISCONNECTING;
        wifi_mac_scan_access(wnet_vif);
        AML_PRINT_LOG_INFO("disconnect complete!\n");
    }
    driv_ps_sleep(drv_priv);
}

static void
drv_key_rst( struct drv_private * drv_priv,unsigned char wnet_vif_id,
    unsigned short key_index, int staid, unsigned char group)
{
    drv_hal_add_workitem( (WorkHandler)drv_key_rst_ex,NULL,
                         (SYS_TYPE)drv_priv,(SYS_TYPE)wnet_vif_id,
                         (SYS_TYPE)key_index,(SYS_TYPE)staid,
                         (SYS_TYPE)group);
}

static int
drv_key_set(struct drv_private * drv_priv, unsigned char wnet_vif_id,
    unsigned short key_index, struct hal_key_val *hk,
    const unsigned char mac[WIFINET_ADDR_LEN])
{
    int status;
    struct wlan_net_vif *wnet_vif = drv_priv->drv_wnet_vif_table[wnet_vif_id];
    struct wifi_station *sta;

    driv_ps_wakeup(drv_priv);
    if ((mac[0] & 0x01) == 0x01)
    {
        status = drv_hal_keyset(wnet_vif_id, key_index, hk, (unsigned char *)mac, 0);
    }
    else
    {
        if (hk->kv_pad == WIFINET_M_HOSTAP)   //ap mode , send staid
        {
            sta = drv_priv->net_ops->wifi_mac_get_sta(&wnet_vif->vm_sta_tbl, mac,wnet_vif_id);
            status = drv_hal_keyset(wnet_vif_id, key_index, hk, (unsigned char *)mac,sta->sta_associd);
        }
        else
        {
            status = drv_hal_keyset(wnet_vif_id,key_index, hk, (unsigned char *)mac,1);
        }
    }
    driv_ps_sleep(drv_priv);
    return (status != 0);
}

static int
drv_rekey_data_set(struct drv_private *drv_priv,
    unsigned char wnet_vif_id, void  *rekey_data)
{
    int status;

    driv_ps_wakeup(drv_priv);
    status = drv_hal_rekey_data_set(wnet_vif_id, rekey_data, 1);
    driv_ps_sleep(drv_priv);

    return (status != 0);
}

/*
 * Set the 802.11D country
 */
static int
drv_set_country(struct drv_private * drv_priv, char *isoName)
{
    unsigned int wMode;
    int tmpi;

    tmpi = find_country_code((unsigned char *)isoName);
    if (tmpi == 0xff) {
        AML_PRINT_LOG_INFO("can't find country code \n");
        tmpi = 0;
    }

    drv_priv->drv_config.cfg_countrycode = tmpi;
    drv_priv->drv_config.cfg_txpoweplan = wifimac_get_tx_pwr_plan(tmpi);

    AML_PRINT_LOG_INFO("<running> %d\n", drv_priv->drv_config.cfg_countrycode );
    wMode = drv_priv->drv_config.cfg_modesupport;

    if (drv_priv->net_ops->wifi_mac_setup_channel_list)
    {
        drv_priv->net_ops->wifi_mac_setup_channel_list(drv_priv->wmac, wMode,drv_priv->drv_config.cfg_countrycode);
    }

    return 0;
}

/*
 * Return the current country and domain information
 */
extern struct country_chan_mapping  country_chan_mapping_list[];
static void
drv_get_curr_cntry( struct drv_private * drv_priv, unsigned char *iso_name)
{
    if (drv_priv->drv_config.cfg_countrycode != 0xff)
    {
        iso_name[0] = country_chan_mapping_list[drv_priv->drv_config.cfg_countrycode].country[0];
        iso_name[1] = country_chan_mapping_list[drv_priv->drv_config.cfg_countrycode].country[1];
        iso_name[2] = 0;
    }
    else
    {
        iso_name[0] = 'W';
        iso_name[1] = 'W';
        iso_name[2] = 0;
    }
}

static int
drv_set_quiet( struct drv_private *drv_priv,
    unsigned short period, unsigned short duration,
    unsigned short nextStart, unsigned short enabled)
{
//
///we not support this
//
    return 0;
}

void
drv_set_tx_pwr_limit(struct drv_private * drv_priv,
    unsigned int limit, unsigned short txpowerdb)
{
    drv_priv->drv_config.cfg_txpowlimit = (unsigned short)limit;

    drv_update_tx_pwr(drv_priv, txpowerdb);
}

static short
drv_get_noise_floor(struct drv_private * drv_priv,
    unsigned short  freq,  unsigned int chan_flags)
{
    return -85;
}

static void drv_write_word (unsigned int addr,unsigned int data)
{
      struct hal_private* hal_priv = hal_get_priv();

       hal_priv->hal_ops.hal_write_word(addr, data);
}

static unsigned int drv_read_word(unsigned int addr)
{
    struct hal_private* hal_priv = hal_get_priv();
    return hal_priv->hal_ops.hal_read_word(addr);
}


static void drv_bt_write_word (unsigned int addr,unsigned int data)
{
      struct hal_private* hal_priv = hal_get_priv();

       hal_priv->hal_ops.hal_bt_write_word(addr, data);
}

static unsigned int drv_bt_read_word(unsigned int addr)
{
    struct hal_private* hal_priv = hal_get_priv();
    return hal_priv->hal_ops.hal_bt_read_word(addr);
}


static void drv_pt_rx_start (unsigned int qos)
{
      struct hal_private* hal_priv = hal_get_priv();

       hal_priv->hal_ops.hal_pt_rx_start(qos);
}

static struct rx_statics_st drv_pt_rx_stop(void)
{
    struct hal_private* hal_priv = hal_get_priv();
    return hal_priv->hal_ops.hal_pt_rx_stop();
}

static void drv_set_bmfm_info(struct drv_private *drv_priv, int wnet_vif_id,
    unsigned char *group_id, unsigned char * user_position, unsigned char feedback_type)
{
    drv_hal_set_bmfm_info(wnet_vif_id, group_id, user_position, feedback_type);
}

static void drv_interface_enable(unsigned char enable, unsigned char vid)
{
    struct hal_private* hal_priv = hal_get_priv();

    hal_priv->hal_ops.phy_interface_enable(enable, vid);
}

static void drv_cfg_cali_param(void)
{
    struct hal_private* hal_priv = hal_get_priv();

    hal_priv->hal_ops.hal_cfg_cali_param();
}

static void drv_init_ops(struct drv_private *drv_priv)
{
    drv_priv->drv_ops.drv_open = drv_open;                   /* drv_open */
    drv_priv->drv_ops.drv_stop = drv_suspend;                /* drv_stop */
    drv_priv->drv_ops.add_interface = drv_add_wnet_vif;             /* add_interface */
    drv_priv->drv_ops.remove_interface = drv_delete_wnet_vif;             /* remove_interface */
    drv_priv->drv_ops.change_interface = drv_change_wnet_vif;             /* change_interface */
    drv_priv->drv_ops.down_interface = drv_wnet_vif_disconnect;               /* down_interface */
    drv_priv->drv_ops.alloc_nsta = drv_nsta_attach;            /* alloc_nsta */
    drv_priv->drv_ops.free_nsta = drv_nsta_detach;            /* free_nsta */
    drv_priv->drv_ops.cleanup_nsta = drv_nsta_cleanup;           /* cleanup_nsta */
    drv_priv->drv_ops.update_nsta_pwrsave = drv_nsta_update_pwrsave; /* update_nsta_pwrsave */
    drv_priv->drv_ops.new_assoc = drv_assoc_proc;               /* new_assoc */
    drv_priv->drv_ops.reset = drv_reset;                  /* reset */
    drv_priv->drv_ops.set_channel = drv_set_channel;            /* set_channel */
    drv_priv->drv_ops.scan_start = drv_scan_start;             /* scan_start */
    drv_priv->drv_ops.fw_repair = drv_fw_recovery;             /* fw_repair */
    drv_priv->drv_ops.scan_end = drv_scan_end;               /* scan_end */
    drv_priv->drv_ops.connect_start = drv_connect_start;             /*connect_start */
    drv_priv->drv_ops.connect_end = drv_connect_end;               /* connect_end */
    drv_priv->drv_ops.set_channel_rssi = drv_set_channel_rssi;    /* set_channel_rssi */
    drv_priv->drv_ops.set_tx_power_accord_rssi = drv_set_tx_power_accord_rssi;    /* set_tx_power_accord_rssi */
    drv_priv->drv_ops.tx_init = drv_tx_init;                /* tx_init */
    drv_priv->drv_ops.tx_cleanup = drv_txlist_cleanup;             /* tx_cleanup */
    drv_priv->drv_ops.tx_wmm_queue_update = drv_update_wmmq_param;             /* tx_wmm_queue_update */
    drv_priv->drv_ops.tx_start = drv_tx_start;               /* tx_start */
    drv_priv->drv_ops.txlist_qcnt_handle = drv_txlist_qcnt;              /* txlist_qcnt */
    drv_priv->drv_ops.txlist_all_qcnt = drv_txlist_all_qcnt;    /* txlist_all_qcnt */
    drv_priv->drv_ops.txlist_isfull = drv_txlist_isfull;         /* txlist_isfull*/
    drv_priv->drv_ops.rx_init = drv_rx_init;                /* rx_init */
    drv_priv->drv_ops.rx_proc_frame = drv_rx_input;               /* rx_proc_frame */
    drv_priv->drv_ops.check_aggr = drv_aggr_check;             /* check_aggr */
    drv_priv->drv_ops.aggr_tid_query = drv_aggr_tid;             /* aggr_tid_query */
    drv_priv->drv_ops.check_aggr_allow_to_send = drv_aggr_allow_to_send;      /* drv_aggr_allow_to_send */
    drv_priv->drv_ops.set_ampdu_params = drv_set_ampduparams;        /* set_ampdu_params */
    drv_priv->drv_ops.addba_request_setup = drv_addba_req_setup;     /* addba_request_setup */
    drv_priv->drv_ops.addba_response_setup = drv_rx_addbarsp;    /* addba_response_setup */
    drv_priv->drv_ops.addba_request_process = drv_rx_addbareq;   /* addba_request_process */
    drv_priv->drv_ops.addba_response_process = drv_addba_rsp_process;  /* addba_response_process */
    drv_priv->drv_ops.addba_clear = drv_addba_clear;            /* addba_clear */
    drv_priv->drv_ops.delba_process = drv_rx_delba;          /* delba_process */
    drv_priv->drv_ops.addba_status = drv_addba_status;           /* addba_status */
    drv_priv->drv_ops.drv_txrxampdu_del = drv_txrxampdu_del;          /* drv_txrxampdu_del */
    drv_priv->drv_ops.set_addbaresponse = drv_set_addba_rsp;      /* set_addbaresponse */
    drv_priv->drv_ops.clear_addbaresponsestatus = drv_clr_addba_rsp_status; /* clear_addbaresponsestatus */
    drv_priv->drv_ops.set_slottime = drv_update_slot;             /* set_slottime */
    drv_priv->drv_ops.set_protmode = drv_set_protmode;           /* set_protmode */
    drv_priv->drv_ops.set_cfg_txpowlimit = drv_set_cfg_tx_power_limit;         /* set_cfg_txpowlimit */
    drv_priv->drv_ops.key_delete = drv_key_rst;              /* key_delete */
    drv_priv->drv_ops.key_set = drv_key_set;                 /* key_set */
    drv_priv->drv_ops.rekey_data_set = drv_rekey_data_set;                 /* rekey_data_set */
    drv_priv->drv_ops.awake = drv_pwrsave_awake;          /* awake */
    drv_priv->drv_ops.netsleep = drv_pwrsave_netsleep;       /* netsleep */
    drv_priv->drv_ops.fullsleep = drv_pwrsave_fullsleep;      /* fullsleep */
    drv_priv->drv_ops.set_country = drv_set_country;            /* set_country */
    drv_priv->drv_ops.get_current_country = drv_get_curr_cntry;     /* get_current_country */
    drv_priv->drv_ops.set_quiet = drv_set_quiet;              /* set_quiet */
    drv_priv->drv_ops.set_bwmode = drv_set_chan_bw_mode;            /* set_bwmode */
    drv_priv->drv_ops.GetSecondChanBusy = drv_get_scn_chan_busy;         /* GetSecondChanBusy */
    drv_priv->drv_ops.drv_get_config_param = drv_get_config;             /* drv_get_config_param */
    drv_priv->drv_ops.drv_set_config_param = drv_set_config;             /* drv_set_config_param */
    drv_priv->drv_ops.get_noisefloor = drv_get_noise_floor;            /* get_noisefloor */
    drv_priv->drv_ops.drv_set_txPwrLimit = drv_set_tx_pwr_limit;          /* drv_set_txPwrLimit */
    drv_priv->drv_ops.get_amsdu_supported = drv_get_amsdu_supported;     /* get_amsdu_supported */
    drv_priv->drv_ops.process_uapsd_trigger = drv_process_uapsd_nsta_trigger;  /* process_uapsd_trigger */
    drv_priv->drv_ops.uapsd_qcnt = drv_tx_uapsd_nsta_qcnt;         /* uapsd_qcnt */
    drv_priv->drv_ops.set_macaddr = drv_set_mac_addr;        /* set_macaddr */
    drv_priv->drv_ops.Low_register_behindTask = drv_low_reg_behind_task; /* Low_register_behindTask */
    drv_priv->drv_ops.Low_callRegisteredTask = drv_low_call_task;  /* Low_callRegisteredTask */
    drv_priv->drv_ops.Low_addDHWorkTask = drv_low_add_worktask;   /* Low_addDHWorkTask */
    drv_priv->drv_ops.RegisterStationID = drv_phy_reg_sta_id;     /*RegisterStationID */
    drv_priv->drv_ops.clear_staid_and_bssid = drv_clear_staid_and_bssid;   /* clear_staid_and_bssid */
    drv_priv->drv_ops.UnRegisterAllStationID = drv_phy_unreg_all_sta_id; /* UnRegisterAllStationID */
    drv_priv->drv_ops.phy_setchannelsupport = NULL;       /* drv_set_channelsupport */
    drv_priv->drv_ops.Phy_beaconinit = drv_bcn_init;           /*Phy_beaconinit*/
    drv_priv->drv_ops.Phy_SetBeaconStart = drv_set_bcn_start;       /*Phy_SetBeaconStart*/
    drv_priv->drv_ops.Phy_PutBeaconBuf = drv_put_bcn_buf;       /*Phy_PutBeaconBuf*/
    /* drv_hal_set_p2p_opps_cwend_enable */
    drv_priv->drv_ops.drv_hal_set_p2p_opps_cwend_enable = drv_hal_set_p2p_opps_cwend_enable;
    /* drv_hal_set_p2p_noa_enable */
    drv_priv->drv_ops.drv_hal_set_p2p_noa_enable = drv_hal_set_p2p_noa_enable;
    drv_priv->drv_ops.drv_hal_get_tsf = drv_hal_get_tsf;/* drv_hal_get_tsf */
    drv_priv->drv_ops.drv_hal_tx_frm_pause = drv_hal_tx_frm_pause;   /* drv_hal_tx_frm_pause */
    drv_priv->drv_ops.drv_p2p_client_opps_cwend_may_sleep = drv_p2p_client_opps_cwend_may_sleep;
    drv_priv->drv_ops.drv_txq_backup_send = drv_txq_backup_send;
    drv_priv->drv_ops.phy_stc = phy_stc;
    drv_priv->drv_ops.get_snr = get_snr;
    drv_priv->drv_ops.cca_busy_check = cca_busy_check;
    drv_priv->drv_ops.drv_send_null_data = drv_send_null_data;
    drv_priv->drv_ops.drv_keep_alive = drv_keep_alive;
    drv_priv->drv_ops.drv_set_beacon_miss = drv_set_beacon_miss;
    drv_priv->drv_ops.drv_set_vsdb = drv_set_vsdb;
    drv_priv->drv_ops.drv_set_arp_agent = drv_set_arp_agent;
    drv_priv->drv_ops.drv_set_pattern = drv_set_pattern;
    drv_priv->drv_ops.drv_set_suspend = drv_set_suspend;
    drv_priv->drv_ops.drv_set_pkt_drop = drv_set_pkt_drop;
    drv_priv->drv_ops.drv_set_is_mother_channel = drv_set_is_mother_channel;
    drv_priv->drv_ops.drv_flush_normal_buffer_queue = drv_flush_normal_buffer_queue;
    drv_priv->drv_ops.drv_free_normal_buffer_queue = drv_free_normal_buffer_queue;
    drv_priv->drv_ops.drv_flush_txdata = drv_txlist_flushfree;
    drv_priv->drv_ops.drv_get_sts = drv_get_sts;
    drv_priv->drv_ops.drv_tx_pending_pkt = drv_tx_pending_pkt;
#ifdef DRV_SUPPORT_TX_WITHDRAW
    drv_priv->drv_ops.drv_tx_withdraw_legacyps_send= drv_tx_withdraw_legacyps_send;
#endif
    drv_priv->drv_ops.drv_write_word = drv_write_word;
    drv_priv->drv_ops.drv_read_word =  drv_read_word;
    drv_priv->drv_ops.drv_bt_write_word = drv_bt_write_word;
    drv_priv->drv_ops.drv_bt_read_word =  drv_bt_read_word;
    drv_priv->drv_ops.drv_pt_rx_start = drv_pt_rx_start;
    drv_priv->drv_ops.drv_pt_rx_stop = drv_pt_rx_stop;

    drv_priv->drv_ops.drv_set_bmfm_info = drv_set_bmfm_info;
    drv_priv->drv_ops.drv_interface_enable = drv_interface_enable;
    drv_priv->drv_ops.drv_cfg_cali_param = drv_cfg_cali_param;
    drv_priv->drv_ops.drv_cfg_txpwr_cffc_param = drv_cfg_txpwr_cffc_param;
    drv_priv->drv_ops.drv_print_fwlog = drv_print_fwlog;
    drv_priv->drv_ops.drv_set_bssid = drv_set_bssid;
    drv_priv->drv_ops.drv_set_tx_livetime = drv_hal_txlivetime;
}

void aml_set_mac_control_register(void)
{
    unsigned int read_tmp = 0;
    read_tmp = drv_read_word(MAC_CONTROL);
    read_tmp |= BIT(20); //enable qos null frame not update bitmap
    drv_write_word(MAC_CONTROL, read_tmp);
}

void aml_lp_shutdown_send_req(void)
{
    int value = 0;

    value = aml_send_me_shutdown();
    if (!value)
    {
        printk("shut_msg send fail! \n");
    }
}

void aml_interface_shutdown_init(void)
{
    g_lp_shutdown_func = aml_lp_shutdown_send_req;
}

int aml_drv_attach( struct drv_private *drv_priv, struct wifi_mac* wmac)
{
    int i ;
    //int error;
    char *country_code = NULL;
    int country_index = DEFAULT_COUNTRY;

    AML_PRINT_LOG_INFO("enter Here\n");

    drv_init_ops(drv_priv);

    drv_priv->wmac = wmac;
    drv_priv->net_ops = &wmac->wmac_ops;
    drv_priv->drv_not_init_flag  = 1;
    drv_priv->drv_connetting = 0;
    drv_priv->drv_scanning = 0;
    drv_priv->is_mother_channel[0] = 1;
    drv_priv->is_mother_channel[1] = 1;
    drv_priv->drv_config.cfg_cachelsz       = DEFAULT_CACHESIZE;        /* convert to bytes */
    drv_priv->wait_mpdu_timeout = 1;
    drv_priv->add_wakeup_work = 0;
    drv_priv->stop_noa_flag = 0;
    wmac->wm_disconnect_code = DISCONNECT_DRVINIT;

    drv_hal_attach(drv_priv, (void *)get_hal_call_back_table());    //attach hal

    drv_priv->hal_priv = hal_get_priv();

    drv_pwrsave_init(drv_priv);
    driv_ps_wakeup(drv_priv);

    country_code = aml_wifi_get_country_code();
    AML_PRINT_LOG_INFO("<running> insmod country: %s\n", country_code);

    country_index = find_country_code((unsigned char *)country_code);
    drv_priv->drv_config.cfg_countrycode = country_index == 0xff ? DEFAULT_COUNTRY : country_index;
    drv_priv->drv_config.cfg_txpoweplan = 0xff;

    drv_priv->drv_config.cfg_ampduackpolicy     = DEFAULT_AMPDUACKPOLICY;
    drv_priv->drv_config.cfg_htsupport      = DEFAULT_HT_ENABLE;
    drv_priv->drv_config.cfg_vhtsupport      = DEFAULT_VHT_ENABLE;
    drv_priv->drv_config.cfg_burst_ack        = DEFAULT_BURST_ENABLE;
    drv_priv->drv_config.cfg_txpowlimit             = DEFAULT_TXPOWER;
    drv_priv->drv_config.cfg_aggr_prot      = DEFAULT_TXAGG_PROT;
    drv_priv->drv_config.cfg_disratecontrol     = DEFAULT_RATECONTROL_DIS;
    drv_priv->drv_config.cfg_haswme         = DEFAULT_WMMSUPPORT;
    drv_priv->drv_config.cfg_dynamic_bw         = DEFAULT_SUPPORT_DYNAMIC_BW;
    drv_priv->drv_config.cfg_wifi_bt_coexist_support = DEFAULT_SUPPORT_WIFI_BT_COEXIST;

    /* 11n Capabilities */
    if (drv_priv->drv_config.cfg_htsupport)
    {
        drv_priv->drv_config.cfg_ampdu_limit        = DEFAULT_TXAMPDU_LEN_MAX;
        drv_priv->drv_config.cfg_txaggr         = DEFAULT_TXAMPDU_EN;
        drv_priv->drv_config.cfg_rxaggr         = DEFAULT_RXAMPDU_EN;
        drv_priv->drv_config.cfg_txamsdu        = DEFAULT_TXAMSDU_EN;
        drv_priv->drv_config.cfg_ampdu_subframes = DEFAULT_TXAMPDU_SUB_MAX;
        drv_priv->drv_config.cfg_40Msupport     = DEFAULT_11N40M_SUPPORT;
        drv_priv->drv_config.cfg_ampdu_oneframe = DEFAULT_TXAMPDU_ONEFRAME;
        drv_priv->drv_config.cfg_ampdu_livetime    = DEFAULT_TXAMPDU_ONEFRAME;
    }

    /* set short retry 100, long retry 100, firmware try more times to send out */
    drv_set_config((void *)drv_priv, CHIP_PARAM_RETRY_LIMIT, 100 << 8 | 100);

   /*11AC capabilities*/
    if (drv_priv->drv_config.cfg_vhtsupport)
    {
        // T.B.D
    }

    drv_priv->drv_config.cfg_txchainmask        = DEFAULT_TXCHAINMASK;              //chainmask: not support yet
    drv_priv->drv_config.cfg_rxchainmask        = DEFAULT_RXCHAINMASK;
    drv_priv->drv_config.cfg_txchainmasklegacy  = DEFAULT_TXCHAINMASKLEGACY;
    drv_priv->drv_config.cfg_rxchainmasklegacy  = DEFAULT_RXCHAINMASKLEGACY;
    drv_priv->drv_config.cfg_bw_ctrl            = DEFAULT_BWC_ENABLE;
    drv_priv->drv_config.cfg_aessupport         = 1;/*aes support*/
    drv_priv->drv_config.cfg_tkipsupport        = 1;/*tkip support*/
    drv_priv->drv_config.cfg_wepsupport         = 1;/*wep support*/
    drv_priv->drv_config.cfg_wapisupport        = 1;/*wapi support*/
    drv_priv->drv_config.cfg_tkipmicsupport     = DEFAULT_HW_TKIP_MIC;/*tkip mic support*/
    drv_priv->drv_config.cfg_rtcenable                   = DEFAULT_RTC_ENABLE;
    drv_priv->drv_config.cfg_retrynum       = DEFAULT_TXRETRY_MAX;
    drv_priv->drv_config.cfg_uapsdsupported     = DEFAULT_UAPSU_EN;
    drv_priv->drv_config.cfg_modesupport    = DEFAULT_SELECT_MODE;
    drv_priv->drv_config.cfg_eap_lowest_rate = DEFAULT_MCAST_EAPOL_NULLDATA_RATE_11G;
    drv_priv->drv_config.cfg_dssupport       = DEFAULT_DSSUPPORT;
    drv_priv->drv_config.cfg_eat_count_max = DEFAULT_EAT_COUNT_MAX;
    drv_priv->drv_config.cfg_aggr_thresh = DEFAULT_AGGR_THRESH;
    drv_priv->drv_config.cfg_no_aggr_thresh = DEFAULT_NO_AGGR_THRESH;
    drv_priv->drv_config.cfg_hrtimer_interval = DEFAULT_HRTIMER_INTERVAL;
    drv_priv->drv_config.cfg_mfp = DEFAULT_MFP_EN;//support mfp
    drv_priv->drv_config.cfg_mac_mode = DEFAULT_AUTO;
    drv_priv->drv_config.cfg_band = DEFAULT_BAND_ALL;
    drv_priv->drv_config.cfg_recovery = DEFAULT_SUPPORT_RECOVERY;

    if (aml_bus_type) {
        drv_priv->drv_config.cfg_checksumoffload    = USB_DEFAULT_HW_CSUM;
    }
#ifdef SDIO_MODE_ON
    else {
        drv_priv->drv_config.cfg_checksumoffload    = DEFAULT_HW_CSUM;
    }
#endif

    for (i = 0; i < DEFAULT_MAX_VMAC; i++)
    {
        drv_priv->dot11VmacPsState[i] = DRV_PWRSAVE_FULL_SLEEP;
    }
    drv_priv->dot11ComPsState = DRV_PWRSAVE_AWAKE;
    drv_priv->drv_PhyPsState = DRV_PWRSAVE_AWAKE;
    /*
     * The MAC has multi-rate retry support.
     */
    drv_priv->drv_ratectrl_mrr = 1;
/*    error = drv_channel_init(drv_priv, drv_priv->drv_config.cfg_countrycode);
    if (error != 0)
    {
        driv_ps_sleep(drv_priv);
         AML_PRINT_LOG_ERR("<running> error!!!\n");
        goto bad;
    }
    wifi_mac_set_tx_power_coefficient(drv_priv, NULL, drv_priv->drv_config.cfg_txpoweplan);
*/
    drv_priv->net_ops->wifi_mac_rate_ratmod_attach(drv_priv);
    AML_PRINT_LOG_INFO("<running> drv_priv->drv_ratectrl_size = %d\n",
        drv_priv->drv_ratectrl_size);

    /*
     * Setup rate tables for all potential media types.
     * In fact, we just initialize one mode, because all
     * of rates info are included in one mode table.
     */
    drv_rate_setup(drv_priv, WIFINET_MODE_11GNAC);
    drv_txlist_setup(drv_priv);
    driv_ps_sleep(drv_priv);
    aml_set_mac_control_register();

    drv_cfg_load_from_file();

    // init sdio/usb/pcie interface
    //aml_lp_shutdown_func_register
    aml_interface_shutdown_init();
    drv_priv->drv_agg_limit = drv_calc_agg_num(drv_priv, drv_priv->drv_config.cfg_ampdu_subframes);
    drv_priv->hal_priv->hal_max_mpdu_num = AMPDU_NUM_ONE_TIME_TX * drv_priv->drv_agg_limit;

    AML_PRINT_LOG_INFO("drv_agg_limit:%d, hal_max_mpdu_num:%d\n", drv_priv->drv_agg_limit, drv_priv->hal_priv->hal_max_mpdu_num);

    return 0;
/*
bad:
    return -ENODEV;
*/
}

void aml_drv_detach( struct drv_private * drv_priv)
{
    int i;
    g_lp_shutdown_func = NULL;

    drv_stop(drv_priv);
    if (drv_priv->net_ops) {
        drv_priv->net_ops->wifi_mac_rate_ratmod_detach(drv_priv);
    }

    /* cleanup tx queues */
    for (i = 0; i < HAL_NUM_TX_QUEUES; i++)
        if (DRV_TXQUEUE_VALUE(drv_priv, i))
            drv_txlist_destroy(drv_priv, &drv_priv->drv_txlist_table[i]);

    AML_PRINT_LOG_INFO("<running>\n");
    drv_hal_detach();
}


static int drv_read_manu_cfg(void *  drv_priv, void* cfg)
{
    AML_PRINT_LOG_INFO("maybe involve nand/nor flash operating function,to get manufactory cfg\n");
    return 0;
}

static int drv_get_default_cfg(void *  drv_priv, void* cfg)
{
    return drv_read_manu_cfg(drv_priv,cfg);
}


static void
drv_mic_error_event(void * dpriv,const void* frm,
    unsigned char* sa,unsigned char wnet_vif_id)
{
#if 1
    struct wifi_station *sta = NULL;
    struct wifi_frame *wh = (struct wifi_frame *)frm;
    struct drv_private *drv_priv = (struct drv_private *)dpriv;
    struct wlan_net_vif *wnet_vif = drv_priv->drv_wnet_vif_table[wnet_vif_id];
    struct wifi_station_tbl *nt = &wnet_vif->vm_sta_tbl;
    struct wifi_mac_Rsnparms *rsn;

    sta = drv_priv->net_ops->wifi_mac_get_sta(nt, sa, wnet_vif_id);
    if (sta == NULL)
    {
        AML_PRINT_LOG_INFO("mic_error_event find not error! \n");
        return ;
    }
    rsn = &sta->sta_rsn;

    if (WIFINET_IS_MULTICAST(wh->i_addr1))
    {
        if(rsn->rsn_mcastcipher == WIFINET_CIPHER_TKIP)
            drv_priv->net_ops->wifi_mac_notify_mic_fail(wnet_vif,wh, (int)wnet_vif->vm_def_txkey);
    }
    else
    {
        if(rsn->rsn_ucastcipher == WIFINET_CIPHER_TKIP)
            drv_priv->net_ops->wifi_mac_notify_mic_fail(wnet_vif,wh, 0);
    }
#endif
}

extern int g_hr_lock_timer_valid;
static void drv_tx_ok_timeout(void *drv_priv)
{
    struct drv_private *drv_private = (struct drv_private *)(drv_priv);

    if (drv_private != NULL) {
        DRV_TX_TIMEOUT_LOCK(drv_private);
        if (!g_hr_lock_timer_valid) {
            drv_private->wait_mpdu_timeout = 1;
        }
        AML_PRINT(AML_LOG_ID_XMIT,AML_LOG_LEVEL_DEBUG, "waiting_pkt_timeout:%d\n", drv_private->wait_mpdu_timeout);
        DRV_TX_TIMEOUT_UNLOCK(drv_private);
    }
}

static void drv_tx_pkt_clear(void *drv_priv)
{
    struct drv_private *drv_private = (struct drv_private *)(drv_priv);

    if (drv_private != NULL) {
        wifi_mac_notify_pkt_clear(drv_private->wmac);
    }
}

static void
drv_intr_rx_ok(void * dpriv,struct sk_buff *skb, unsigned long long PN, unsigned char encrypt, unsigned char Rssi,unsigned char vendor_rate_code,
    unsigned char channel, unsigned char aggr, unsigned char wnet_vif_id, unsigned char keyid, unsigned int channel_bw, unsigned int rx_sgi)
{
    struct wifi_mac *wifimac;
    struct drv_private *drv_priv = (struct drv_private *)dpriv;

    struct sk_buff *skbbuf = (struct sk_buff *)skb;
    struct wifi_frame *wh;
    struct wifi_mac_rx_status rxstatus = {0};
    wifimac = drv_priv->wmac;

    rxstatus.rs_pn = PN;
    rxstatus.rs_encrypt = encrypt;
    rxstatus.rs_flags = aggr;
    rxstatus.rs_channel = channel;
    rxstatus.rs_rssi = Rssi;
    rxstatus.rs_tstamp.tsf = jiffies;
    rxstatus.rs_wnet_vif_id = wnet_vif_id;
    rxstatus.rs_keyid = keyid;
    rxstatus.rs_vendor_rate_code = vendor_rate_code;
    rxstatus.channel_bw = channel_bw;
    rxstatus.rs_sgi = rx_sgi;

    wh = (struct wifi_frame *)os_skb_data(skb);
    if (!list_empty(&wifimac->wm_wnet_vifs))
    {
        drv_priv->drv_stats.rx_indicate_cnt++;
        drv_priv->net_ops->wifi_mac_rx_complete(wifimac, skbbuf, &rxstatus);
    }
    else
    {
        drv_priv->drv_stats.rx_free_skb_cnt++;
        os_skb_free(skb);
    }
}

static int pmf_encrypt_pkt_handle(void *dpriv, struct sk_buff *skb, unsigned char rssi,
    unsigned char vendor_rate_code, unsigned char channel, unsigned char aggr, unsigned char wnet_vif_id, unsigned char keyid,
    unsigned int channel_bw, unsigned int rx_sgi)
{

    struct wifi_mac *wifimac;
    struct drv_private *drv_priv= (struct drv_private *)dpriv;
    struct sk_buff *skbbuf = ( struct sk_buff *)skb;
    struct wifi_frame *wh;
    struct wifi_mac_rx_status rxstatus;
    unsigned char pkt_type;
    unsigned char is_protect;

    wh = (struct wifi_frame *)os_skb_data(skb);
    pkt_type = wh->i_fc[0] & WIFINET_FC0_TYPE_MASK;
    is_protect = wh->i_fc[1] & WIFINET_FC1_WEP;
    if (!((pkt_type == WIFINET_FC0_TYPE_MGT) && is_protect)) {
        return 1;

    } else {
    #if 0
        unsigned char i;

        AML_PRINT_LOG_INFO("pmf pkt, not drop\n");
        for (i = 0; i < 32; ++i) {
            AML_PRINT_LOG_INFO("%02x:\n", ((unsigned char *)wh)[i]);
        }
        AML_PRINT_LOG_INFO("\n");
    #endif
    }

    wifimac = drv_priv->wmac;
    rxstatus.rs_flags = aggr;
    rxstatus.rs_channel = channel;
    rxstatus.rs_rssi = rssi;
    rxstatus.rs_tstamp.tsf = jiffies;
    rxstatus.rs_wnet_vif_id = wnet_vif_id;
    rxstatus.rs_keyid = keyid ;
    rxstatus.rs_vendor_rate_code = vendor_rate_code;
    rxstatus.channel_bw = channel_bw;
    rxstatus.rs_sgi = rx_sgi;

    if (!list_empty(&wifimac->wm_wnet_vifs)) {
        drv_priv->drv_stats.rx_indicate_cnt++;
        drv_priv->net_ops->wifi_mac_rx_complete(wifimac, skbbuf, &rxstatus);

    } else {
        drv_priv->drv_stats.rx_free_skb_cnt++;
        os_skb_free(skb);
    }

    return 0;
}

#ifdef CONFIG_P2P
void p2p_noa_start_irq (struct wifi_mac_p2p *p2p, struct drv_private *drv_priv)
{
    unsigned short HiP2pNoaCountNow;
    struct wlan_net_vif *wnet_vif = p2p->wnet_vif;
    struct drv_txlist *txlist;
    struct wifi_mac *wifimac = drv_priv->wmac;
    struct wifi_station_tbl *nt = &wnet_vif->vm_sta_tbl;
    struct wifi_station *sta = NULL, *sta_next = NULL;
    struct wlan_net_vif *main_vmac = drv_priv->drv_wnet_vif_table[NET80211_MAIN_VMAC];
    struct wlan_net_vif *p2p_vmac = drv_priv->drv_wnet_vif_table[NET80211_P2P_VMAC];

    if (!(p2p->p2p_flag & P2P_NOA_START_FLAG_HI) || !p2p->HiP2pNoaCountNow) {
        return;
    }

    p2p->HiP2pNoaCountNow --;
    if ((p2p->HiP2pNoaCountNow == 0) && (p2p->noa.count == NET80211_P2P_SCHED_REPEAT)) {
        p2p->HiP2pNoaCountNow = (p2p->noa.count << 1);
    }
    HiP2pNoaCountNow = p2p->HiP2pNoaCountNow;

    if (P2P_NoA_START_FLAG(HiP2pNoaCountNow)) {
        /* if noa start */
        AML_PRINT(AML_LOG_ID_P2P, AML_LOG_LEVEL_DEBUG, "noa start HiP2pNoaCountNow=%d\n",
              HiP2pNoaCountNow);
        /*
            check power save precedence of GO as follow:
            1. Highest: Absence due to a non-periodic Notice of Absence (Count = 1).
            2. Presence from TBTT until the end of Beacon frame transmission.
            3. Presence during the CTWindow.
            4. Lowest: Absence for a periodic Notice of Absence (Count > 1).
        */
        if  ((p2p->noa.count > 1) && (p2p->p2p_flag & P2P_OPPPS_START_FLAG_HI)
            && (!(p2p->p2p_flag & P2P_OPPPS_CWEND_FLAG_HI) || (wnet_vif->vm_pstxqueue_flags & WIFINET_PSQUEUE_OPPS))) {
            /*noa count != 1,  opps is already start and sleep, just return; */
            return;
        }
        wnet_vif->vm_pstxqueue_flags |= WIFINET_PSQUEUE_NOA;

#ifdef CONFIG_CONCURRENT_MODE
        if ((wifimac->wm_nrunning > 1) &&
            (wifimac->wm_vsdb_slot == CONCURRENT_SLOT_P2P) &&
            (p2p_vmac->vm_curchan != WIFINET_CHAN_ERR)) {
                wifimac->wm_vsdb_slot = CONCURRENT_SLOT_STA;
                wifi_mac_restore_wnet_vif_channel(p2p_vmac);
        }
#endif

        if (drv_priv->net_ops->wifi_mac_pwrsave_is_wnet_vif_fullsleep(wnet_vif) != 0)
        {
            drv_priv->net_ops->wifi_mac_pwrsave_fullsleep(wnet_vif, SLEEP_AFTER_NOA_START);
        }
    }
    else
    {
        /* if noa end */
        AML_PRINT(AML_LOG_ID_P2P,AML_LOG_LEVEL_DEBUG, "noa end HiP2pNoaCountNow=%d\n",HiP2pNoaCountNow);

        wnet_vif->vm_pstxqueue_flags &= (~WIFINET_PSQUEUE_NOA);

        do
        {
            if (wnet_vif->vm_pstxqueue_flags & WIFINET_PSQUEUE_OPPS)
            {
                if (drv_priv->net_ops->wifi_mac_pwrsave_is_wnet_vif_fullsleep(wnet_vif) != 0)
                {
                    drv_priv->net_ops->wifi_mac_pwrsave_fullsleep(wnet_vif, SLEEP_AFTER_NOA_END);
                }
                break;
            }
#ifdef CONFIG_CONCURRENT_MODE
            if ((wifimac->wm_nrunning > 1) &&
                (wifimac->wm_vsdb_slot == CONCURRENT_SLOT_STA) &&
                (main_vmac->vm_curchan != WIFINET_CHAN_ERR)) {
                    wifimac->wm_vsdb_slot = CONCURRENT_SLOT_P2P;
                    wifi_mac_restore_wnet_vif_channel(main_vmac);
            }
#endif


            if (drv_priv->net_ops->wifi_mac_pwrsave_is_sta_sleeping(wnet_vif) == 0)
            {
                /* retry trigger. may drop response to last-time trigger */
                struct wifi_station *sta = wnet_vif->vm_mainsta;
                if (sta->sta_flags_ext & WIFINET_NODE_TRIGGER_WAIT_NOA_END)
                {
                  drv_priv->net_ops->wifi_mac_pwrsave_sta_trigger(wnet_vif);
                    sta->sta_flags_ext &= ~WIFINET_NODE_TRIGGER_WAIT_NOA_END;
                }
            }

            if (wnet_vif->vm_opmode == WIFINET_M_HOSTAP)
            {
                /* retry dtim */
                wnet_vif = drv_priv->drv_wnet_vif_table[wnet_vif->wnet_vif_id];
                txlist = &drv_priv->drv_txlist_table[HAL_WME_MCAST];
                if (wnet_vif->vm_mqueue_flag_send & MCAST_SEND_FLAG_NOA_END_RETRY)
                {
                    drv_txq_backup_send(drv_priv, txlist);
                    wnet_vif->vm_mqueue_flag_send &= (~MCAST_SEND_FLAG_NOA_END_RETRY);
                }
            }

            if (wnet_vif->vm_opmode == WIFINET_M_HOSTAP)
            {
                WIFINET_NODE_LOCK(nt);
                list_for_each_entry_safe(sta, sta_next, &nt->nt_nsta, sta_list)
                {
                    if (sta->sta_wnet_vif == wnet_vif)
                    {
                        if (sta->sta_flags_ext & WIFINET_NODE_PS_FLUSH_WAIT_NOA_END)
                        {
                            if ( drv_priv->net_ops->wifi_mac_pwrsave_psqueue_flush(sta) <= 0)
                            {
                                sta->sta_flags_ext &= ~WIFINET_NODE_PS_FLUSH_WAIT_NOA_END;
                            }
                        }

                        if (sta->sta_flags_ext & WIFINET_NODE_UAPSD_FLUSH_WAIT_NOA_END)
                        {
                            int uapsd_len = wifimac->drv_priv->drv_ops.process_uapsd_trigger(wifimac->drv_priv,
                                sta->drv_sta, WME_UAPSD_NODE_MAXQDEPTH, 0, 1);

                            if (uapsd_len <= 0)
                            {
                                sta->sta_flags_ext &= ~WIFINET_NODE_UAPSD_FLUSH_WAIT_NOA_END;
                            }
                        }
                    }
                }
                WIFINET_NODE_UNLOCK(nt);
            }
            drv_priv->net_ops->wifi_mac_buffer_txq_send_pre(wnet_vif);

            if (HiP2pNoaCountNow == 0)
            {
                /* noa need to be canceled */

                if (wnet_vif->vm_opmode == WIFINET_M_HOSTAP)
                {
                    vm_p2p_go_cancel_noa(p2p);
                }
                else if (wnet_vif->vm_opmode == WIFINET_M_STA)
                {
                    vm_p2p_client_cancel_noa(p2p);
                }
            }

            if (drv_priv->net_ops->wifi_mac_pwrsave_is_sta_sleeping(wnet_vif) == 0)
            {
                 drv_priv->net_ops->wifi_mac_pwrsave_fullsleep(wnet_vif, SLEEP_AFTER_NOA_END_DUMMY);
            }
        }
        while (0);
    }
}

static void p2p_opps_ctw_start_irq (struct wifi_mac_p2p *p2p)
{
    if (p2p->p2p_flag & P2P_OPPPS_START_FLAG_HI)
    {
        AML_PRINT(AML_LOG_ID_P2P, AML_LOG_LEVEL_DEBUG, "++\n");
        p2p->p2p_flag &= (~P2P_OPPPS_CWEND_FLAG_HI);

        /* wakeup and tx */
        p2p->wnet_vif->vm_pstxqueue_flags &= (~WIFINET_PSQUEUE_OPPS);
        /*
            if noa count == 1 and noa started, so we remain NoA ps state.
            And we need clear flags of Opps for subsequent NoA (count == 1 or periodic).
            if noa coun > 1 (periodically), the precedence of opps is higher than noa, so
            we clear noa ps flag to send frames.
        */
        if (p2p->wnet_vif->vm_p2p->noa.count > 1)
        {
            p2p->wnet_vif->vm_pstxqueue_flags &= (~WIFINET_PSQUEUE_NOA);
        }
        wifi_mac_buffer_txq_send_pre(p2p->wnet_vif);
    }
}

#endif

static void drv_intr_bcn_send_ok(void * dpriv,unsigned char vma_id)
{
    struct drv_private *drv_priv = (struct drv_private *)dpriv;
    struct wlan_net_vif  *wnet_vif = NULL;
    struct wifi_mac *wifimac;
    struct sk_buff *skb;
    unsigned char rate = 0;
    unsigned short flag = 0;

    wnet_vif  = drv_priv->drv_wnet_vif_table[vma_id];
    if((wnet_vif== NULL)||(wnet_vif->vm_state != WIFINET_S_CONNECTED))
        return;
    wifimac = wnet_vif->vm_wmac;

    if ((wnet_vif->vm_opmode == WIFINET_M_HOSTAP)||
        (wnet_vif->vm_opmode == WIFINET_M_IBSS))
    {
        drv_tx_get_mgmt_frm_rate(drv_priv, wnet_vif->vm_mac_mode,
            WIFINET_FC0_TYPE_MGT | WIFINET_FC0_SUBTYPE_BEACON, &rate, &flag);

        WIFINET_BEACONBUF_LOCK(wifimac);
        skb = wnet_vif->vm_beaconbuf;
        if(skb)
        {
            drv_priv->net_ops->wifi_mac_update_beacon(drv_priv->wmac, vma_id,skb, 0);
            drv_hal_put_bcn_buf(vma_id, os_skb_data(skb), os_skb_get_pktlen(skb), rate, flag);
        }
        else
        {
            AML_PRINT_LOG_INFO("Enter Function ERROR \n");
        }
        WIFINET_BEACONBUF_UNLOCK(wifimac);

#ifdef CONFIG_P2P
        if ((drv_priv->net_ops->wifi_mac_pwrsave_is_wnet_vif_fullsleep(wnet_vif) == 0)
            && (wnet_vif->vm_p2p->p2p_flag & P2P_OPPPS_START_FLAG_HI))
        {
            //drv_priv->net_ops->wifi_mac_pwrsave_networksleep(wnet_vif, NETSLEEP_AFTER_BEACON_SEND);
        }

        p2p_opps_ctw_start_irq(wnet_vif->vm_p2p);
        if (wnet_vif->vm_p2p->p2p_flag & (P2P_NOA_START_MATCH_BEACON_HI|P2P_NOA_END_MATCH_BEACON_HI)) {
            p2p_noa_start_irq(wnet_vif->vm_p2p, drv_priv);

        } else if (wnet_vif->vm_pstxqueue_flags & WIFINET_PSQUEUE_NOA) {
             drv_priv->net_ops->wifi_mac_pwrsave_fullsleep(wnet_vif, SLEEP_AFTER_NOA_START);
        }
#endif
    }
}


static void drv_intr_dtim_send_ok(void *drv_priv,unsigned char vma_id)
{

}


static void drv_intr_ba_recv_ok(void *drv_priv,unsigned char vma_id)
{
    AML_PRINT_LOG_INFO("Enter Function \n");
}

static int
drv_get_sta_id(void * dpriv,unsigned char *mac,
    unsigned char  wnet_vif_id,int * staid)
{

    struct wifi_station *sta = NULL;
    struct wifi_station_tbl *nt;
    struct drv_private    *drv_priv= (struct drv_private *)dpriv;
    struct wlan_net_vif *wnet_vif = NULL;

    wnet_vif = drv_priv->drv_wnet_vif_table[wnet_vif_id];
    ASSERT(wnet_vif);
    nt = &wnet_vif->vm_sta_tbl;

    sta = drv_priv->net_ops->wifi_mac_get_sta(nt, mac,wnet_vif_id);
    if (sta == NULL) {
        return -1;
    }

    if (sta->sta_wnet_vif->vm_opmode == WIFINET_M_STA) {
        *staid =  1;
    } else {
        *staid =  sta->sta_associd&0xff;
    }

    return 0;
}

/*rtc tmp wakeup*/
static void drv_goto_deep_sleep(void * dpriv)
{

}

static void drv_goto_wakeup(void * dpriv, unsigned char wnet_vif_id)
{
    struct drv_private *drv_priv = (struct drv_private *)dpriv;
    struct wlan_net_vif *wnet_vif = NULL;

    wnet_vif = drv_priv->drv_wnet_vif_table[wnet_vif_id];

    //if vmac not exist, may sta irq mistake
    if (wnet_vif)
    {
        struct wifi_mac_pwrsave_t        *ps = &wnet_vif->vm_pwrsave;

        //wifi_mac_pwrsave_show_state(wifimac);
        if (drv_priv->net_ops->wifi_mac_pwrsave_is_sta_fullsleep(wnet_vif) == 0)
        {
            WIFINET_PWRSAVE_LOCK(wnet_vif);
            //wait for beacon timeout
            ps->ips_sleep_wait_reason = SLEEP_AFTER_WAIT_BEACON_TIMEOUT;
            os_timer_ex_start_period(&ps->ips_timer_sleep_wait, ps->ips_ps_waitbeacon_timeout);
            ps->ips_flag_waitbeacon_timer_start = 1;
            WIFINET_PWRSAVE_UNLOCK(wnet_vif);

            if (wnet_vif->vm_p2p->p2p_enable == 1)
                p2p_opps_ctw_start_irq(wnet_vif->vm_p2p);

            drv_priv->net_ops->wifi_mac_pwrsave_networksleep(wnet_vif, NETSLEEP_AFTER_WAKEUP);
        }
    }
}

static void
drv_p2p_opps_cwend_sleep (struct wlan_net_vif *wnet_vif,
    struct drv_private *drv_priv)
{
    wnet_vif->vm_pstxqueue_flags |= WIFINET_PSQUEUE_OPPS;
    drv_priv->net_ops->wifi_mac_pwrsave_fullsleep(wnet_vif, SLEEP_AFTER_OPPS_END);
}

int
drv_p2p_go_opps_cwend_may_sleep (struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct drv_private *  drv_priv = wifimac->drv_priv;
    int ret = -1;

    AML_PRINT(AML_LOG_ID_PWR_SAVE,AML_LOG_LEVEL_DEBUG,"opmode %d vm_ps_sta %d "
            "vm_sta_assoc %d lgcy/mcast/uapsd %d/%d/%d\n",
            wnet_vif->vm_opmode,
            wnet_vif->vm_ps_sta,wnet_vif->vm_sta_assoc,
            !wnet_vif->vm_legacyps_txframes,
            !drv_txlist_all_qcnt(drv_priv, HAL_WME_MCAST),
            !drv_txlist_all_qcnt(drv_priv, HAL_WME_UAPSD));
    /*
    * go goto sleep when no frames are going to send or are sent,
    * all gc are in sleep state.
    */
    if ((wifi_mac_pwrsave_if_ap_can_opps (wnet_vif) == 0)
        && !drv_txlist_all_qcnt(drv_priv, HAL_WME_MCAST)
        && !wnet_vif->vm_legacyps_txframes
        && !drv_txlist_all_qcnt(drv_priv, HAL_WME_UAPSD))
    {
        drv_p2p_opps_cwend_sleep(wnet_vif, drv_priv);
        ret = 0;
    }
    return ret;
}

int drv_p2p_client_opps_cwend_may_sleep (struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct drv_private *  drv_priv = wifimac->drv_priv;
    int ret = -1;

    //for sta, if unicast trigger is in process, transfer to unicast
    if ((wnet_vif->vm_opmode==WIFINET_M_STA)
        && !wnet_vif->vm_pwrsave.ips_flag_send_ps_trigger)
    {
        drv_p2p_opps_cwend_sleep(wnet_vif, drv_priv);
        ret = 0;
    }
    return ret;
}

static void drv_intr_p2p_opps_cwend (void * dpriv,unsigned char wnet_vif_id)
{
    struct drv_private *drv_priv = (struct drv_private *)dpriv;
    struct wlan_net_vif *wnet_vif = NULL;

    wnet_vif = drv_priv->drv_wnet_vif_table[wnet_vif_id];
    if (wnet_vif)
    {
#ifdef CONFIG_P2P
        struct wifi_mac_p2p *p2p = wnet_vif->vm_p2p;
        if (p2p->p2p_flag & P2P_OPPPS_START_FLAG_HI)
        {
            wnet_vif->vm_p2p->p2p_flag |= P2P_OPPPS_CWEND_FLAG_HI;
            AML_PRINT(AML_LOG_ID_P2P, AML_LOG_LEVEL_DEBUG, "vma_id=%d\n", wnet_vif_id);

            do
            {
                if (drv_p2p_go_opps_cwend_may_sleep(wnet_vif) == 0)
                {
                    break;
                }
                if (drv_p2p_client_opps_cwend_may_sleep(wnet_vif) == 0)
                {
                    break;;
                }
                if (wnet_vif->vm_pstxqueue_flags & WIFINET_PSQUEUE_NOA)
                {
                    //if has trigger in process, just sleep. after noa end, trigger again
                     drv_priv->net_ops->wifi_mac_pwrsave_fullsleep(wnet_vif, SLEEP_AFTER_NOA_START);
                    break;
                }
            }
            while (0);
        }
#endif
    }
}

static void
drv_intr_p2p_noa_start (void *dpriv,unsigned char wnet_vif_id)
{
    struct drv_private *drv_priv = (struct drv_private *)dpriv;
    struct wlan_net_vif *wnet_vif = NULL;

    wnet_vif = drv_priv->drv_wnet_vif_table[wnet_vif_id];
    if (wnet_vif)
    {
#ifdef CONFIG_P2P
        if (drv_priv->net_ops->wifi_mac_pwrsave_is_wnet_vif_fullsleep(wnet_vif)==0)
        {
            //drv_priv->net_ops->wifi_mac_pwrsave_networksleep(wnet_vif, NETSLEEP_AFTER_NOA);
        }
        p2p_noa_start_irq(wnet_vif->vm_p2p, drv_priv);
#endif
    }
}
static void drv_trigger_send_delba(SYS_TYPE param1,
    SYS_TYPE param2,SYS_TYPE param3, SYS_TYPE param4,SYS_TYPE param5)
{
    struct drv_private *drv_priv = drv_get_drv_priv();
    unsigned int vid = 0;
    struct wlan_net_vif *wnet_vif = drv_priv->drv_wnet_vif_table[vid];   /*vmac 0*/
    struct hw_interface* hif = hif_get_hw_interface();
    struct wifi_station* sta = NULL;
    struct wifi_station* sta_next = NULL;
    struct aml_driver_nsta *drv_sta  = NULL;
    unsigned int reg_val = 0;
    unsigned int reg_val2 = 0;
    unsigned int reg_val3 = 0;
    struct wifi_mac_action_mgt_args actionargs;
    //struct hal_private *hal_priv = hal_get_priv();
    struct wifi_mac* wifi_mac =  wifi_mac_get_mac_handle();
    unsigned int tid_index = 0;
    struct drv_rx_scoreboard *RxTidState = NULL;
    struct wifi_station_tbl *vm_sta_tbl = &(wnet_vif->vm_sta_tbl);
    unsigned int j = 0;

    memset(&actionargs, 0, sizeof( struct wifi_mac_action_mgt_args));
#if 0
    reg_val = hif->hif_ops.hi_read_word(RG_COEX_BT_LOGIC_INFO);
    if (reg_val & COEX_EN_ESCO) {
        drv_priv->drv_config.cfg_ampdu_subframes = DEFAULT_TXAMPDU_SUB_MAX_COEX_ESCO;
        wifi_mac->wm_esco_en =1;

    } else {
        drv_priv->drv_config.cfg_ampdu_subframes = DEFAULT_TXAMPDU_SUB_MAX_COEX;
        wifi_mac->wm_esco_en = 0;
    }
#endif

#if 1
    reg_val3 = hif->hif_ops.hi_read_word(RG_PMU_A16);
    reg_val2 = hif->hif_ops.hi_read_word(RG_BT_PMU_A16);
    reg_val = hif->hif_ops.hi_read_word(RG_COEX_BT_LOGIC_INFO);
    if ((reg_val2 & BIT(31)) && (reg_val3 & BIT(31)))
    {
        wifi_mac->wm_bt_en = 1;
        if (reg_val & COEX_EN_ESCO) {
            drv_priv->drv_config.cfg_ampdu_subframes = DEFAULT_TXAMPDU_SUB_MAX_COEX_ESCO;
            wifi_mac->wm_esco_en =1;
            AML_PRINT_LOG_INFO("delba:coex change to work TDD ESCO\n");
        } else if (reg_val2 & BIT(24)){
            drv_priv->drv_config.cfg_ampdu_subframes = DEFAULT_TXAMPDU_SUB_MAX_COEX;
            wifi_mac->wm_esco_en = 0;
            AML_PRINT_LOG_INFO("delba:coex change to work TDD A2DP\n");
        }
    }
    else
    {
        wifi_mac->wm_bt_en = 0;
        wifi_mac->wm_esco_en = 0;
        drv_priv->drv_config.cfg_ampdu_subframes = DEFAULT_TXAMPDU_SUB_MAX;
        AML_PRINT_LOG_INFO("delba:coex change to not work\n");
    }
#endif

   /*two vid need consider*/
   for (j = 0; j < 2; ++j) {
      vm_sta_tbl = &drv_priv->drv_wnet_vif_table[j]->vm_sta_tbl;

        /*traverse all station in  one hash value*/
        WIFINET_NODE_LOCK(vm_sta_tbl);
        list_for_each_entry_safe(sta, sta_next, &vm_sta_tbl->nt_nsta, sta_list) {
            drv_sta = DRIVER_NODE(sta->drv_sta);

            /*traverse all tid */
            for (tid_index = 0; tid_index < WME_NUM_TID; tid_index++) {
                RxTidState = &drv_sta->rx_scb[tid_index];
#if 0

                AML_PRINT_LOG_INFO("delba2:%d\n",del_ba_flag);

                if (zgb_event->del_ba_flag) {
                    RxTidState->rx_addba_exchangecomplete = 1;
                }
#endif

                if (RxTidState->rx_addba_exchangecomplete) {
                    actionargs.category = AML_CATEGORY_BACK;
                    actionargs.action = WIFINET_ACTION_BA_DELBA;
                    actionargs.arg1 = tid_index;
                    /*
                    The Initiator subfield indicates if the originator or the recipient of the data is sending this frame. It is set to 1
                    to indicate the originator and is set to 0 to indicate the recipient. The TID subfield indicates the TSID or the
                    UP for which the block ack has been originally set up.
                    */
                    actionargs.arg2 = BA_INITIATOR;  /*reference protocol 802.11-2016.pdf  chapter 9.4.1.16 DELBA Parameter Set field*/
                    actionargs.arg3 = 1;   /* reference  hornor v9 phone, reference protocol 802.11-2016.pdf  chapter 9.4.1.9 Status Code field*/
                    wifi_mac_send_action(sta, (void *)&actionargs);
                    AML_PRINT_LOG_INFO("tid_index %d ->sta_macaddr=%s\n", tid_index, ether_sprintf(sta->sta_macaddr));
                }
            }
        }
        WIFINET_NODE_UNLOCK(vm_sta_tbl);
    }
}

static void drv_intr_fw_event(void *dpriv, void *event)
{
    struct drv_private *drv_priv = (struct drv_private *)dpriv;
    struct wlan_net_vif *wnet_vif = NULL;
    struct fw_event_no_data *fw_event = (struct fw_event_no_data *)event;
    struct wifi_mac *wifimac = NULL;

    wnet_vif = drv_priv->drv_wnet_vif_table[fw_event->basic_info.vid];
    if (wnet_vif == NULL) {
        return;
    }
    wifimac = wnet_vif->vm_wmac;

    //AML_PRINT_LOG_INFO("vid:%d, event:%d\n", fw_event->basic_info.vid, fw_event->basic_info.event);

    switch (fw_event->basic_info.event) {
        case DHCP_OFFLOAD_EVENT:
            drv_priv->net_ops->wifi_mac_device_ip_config(wnet_vif, event);
            break;

        case TBTT_EVENT:
            drv_priv->net_ops->wifi_mac_tbtt_handle(wnet_vif);
            break;

#ifdef  CONFIG_CONCURRENT_MODE
        case VSDB_SWITCH_PREPARE_EVENT:
            concurrent_vsdb_prepare_change_channel(wifimac);
            break;

        case VSDB_CHANNEL_SWITCH_EVENT:
            concurrent_vsdb_change_channel(wnet_vif->vm_wmac);
            break;
#endif

        case CHANNEL_SWITCH_EVENT:
            drv_priv->net_ops->wifi_mac_channel_switch_complete(wnet_vif);
            break;

        case DPD_CALIBRATION_EVENT:
        {
            struct hal_private * hal_priv = hal_get_priv();
            struct dpd_calibration_event *dpd_event = (struct dpd_calibration_event *)fw_event;
            AML_PRINT_LOG_INFO("dpd event state =%d\n",dpd_event->dpd_state);
            if (dpd_event->dpd_state == dpd_calibration_start) {
                hal_priv->dpd_process_flag |= DPD_FW_EVENT_START;
                hal_dpd_calibration();
            } else {
                if ((hal_priv->dpd_process_flag & DPD_FW_EVENT_START) != 0) {
                    if (wifimac->drv_priv->hal_priv->g_get_fw_log == true) {
                        AML_PRINT_LOG_INFO("enable fwlog get again\n");
                        wifi_mac_add_work_task(wifimac, wifi_mac_set_fwlog_ex, NULL,
                                (SYS_TYPE)0, (SYS_TYPE)0, 0, 0, (SYS_TYPE)0);
                    }
                    hal_priv->dpd_process_flag &= ~DPD_FW_EVENT_START;
                }

                if (aml_bus_type) {
                    hal_priv->dpd_suspend = 0;
                    if (wnet_vif->vm_opmode == WIFINET_M_STA) {
                        wifi_mac_scan_access(wnet_vif);
                    }
                }
            }
            break;
        }
        case TX_ERROR_EVENT:
        {
            struct tx_error_event *error_event = (struct tx_error_event *)fw_event;

            AML_PRINT_LOG_INFO("frame type %x, error type %x \n",
                error_event->frame_type, error_event->error_type);

            drv_priv->net_ops->wifi_mac_process_tx_error(wnet_vif);
            break;
        }

        case FWLOG_PRINT_EVENT:
            drv_priv->hal_priv->hal_ops.hal_set_fwlog_cmd(3); //3: print fwlog
            break;

        case ZGB_EXIST_EVENT:
        {
            struct zgb_exist_event *zgb_event = (struct zgb_exist_event *) fw_event;
            wifimac->wm_zgb_exist_flag = zgb_event->data_info.zgb_exist_flag;
            if (zgb_event->data_info.del_ba_flag)
            {
                drv_trigger_send_delba(0,0,0,0,0);
                wifimac->wm_manual_rx_bufsize = 1;
                AML_PRINT_LOG_INFO("set_rx_buff_size:%d\n",wifimac->wm_manual_rx_bufsize);
            }
            if (zgb_event->data_info.ampdu_num)
            {
                drv_priv->drv_config.cfg_ampdu_subframes = 1;
                AML_PRINT_LOG_INFO("change cfg_ampdu_subframes:%d\n",drv_priv->drv_config.cfg_ampdu_subframes);
            }
            break;
        }

        case WOW_WAKE_EVENT:
        {
            struct wow_wake_event *wow_event = (struct wow_wake_event *)fw_event;
            wifimac->wow_wakeup_reason = wow_event->reason;
            break ;
        }

        default:
            break;
    }
}

static void drv_intr_tx_null_data(void *dpriv, struct tx_nulldata_status * tx_null_status)
{
    struct drv_private *drv_priv = (struct drv_private *)dpriv;
    struct wlan_net_vif *wnet_vif = NULL;

    wnet_vif = drv_priv->drv_wnet_vif_table[tx_null_status->wnet_vif_id];

    //if send nulldata with ps=1 successfully, enter sleep
    //else wakeup
    //AML_PRINT_LOG_INFO("pwr_flag:%d, status:%d", tx_null_status->pwr_flag, tx_null_status->txstatus);
    if (tx_null_status->pwr_flag == 0)
        return;

    if ((tx_null_status->qos == 0) && (tx_null_status->ps == 1)) {
        if (tx_null_status->txstatus== TX_DESCRIPTOR_STATUS_NULL_DATA_OK) {
            if (!(wnet_vif->vm_wmac->wm_flags & WIFINET_F_SCAN)) {
                drv_priv->net_ops->wifi_mac_pwrsave_fullsleep(wnet_vif, SLEEP_AFTER_TX_NULL_WITH_PS);
            #ifdef USER_UAPSD_TRIGGER
                os_timer_ex_start(&(wnet_vif->vm_pwrsave.ips_timer_uapsd_trigger));
            #endif
            }
            drv_priv->net_ops->wifi_mac_notify_ap_success(wnet_vif);

        } else if (tx_null_status->txstatus == TX_DESCRIPTOR_STATUS_NULL_DATA_FAIL) {
            if (!(wnet_vif->vm_wmac->wm_flags & WIFINET_F_SCAN)) {
                drv_priv->net_ops->wifi_mac_pwrsave_wkup_and_NtfyAp(wnet_vif, WKUP_FROM_PSNULL_FAIL);

            }
            drv_priv->net_ops->wifi_mac_notify_ap_success(wnet_vif);
        }
    }
}

static void drv_intr_beacon_miss(void * dpriv, unsigned char wnet_vif_id)
{
    struct drv_private *drv_priv = (struct drv_private *)dpriv;
    drv_priv->net_ops->wifi_mac_beacon_miss(drv_priv->drv_wnet_vif_table[wnet_vif_id]);
}





/*when BT logic link information change or BT alive status change
WIFI need to change  aggregate number
and notify each other that our receiving window has become smaller
*/
static void drv_intr_bt_info_change(void * dpriv, unsigned char wnet_vif_id,unsigned char bt_lk_change )
{
    struct drv_private *drv_priv = (struct drv_private *)dpriv;
    struct wlan_net_vif *wnet_vif = NULL;
    struct hw_interface* hif = hif_get_hw_interface();
    unsigned int reg_val = 0;
    unsigned int reg_val2 = 0;
    unsigned int reg_val3 = 0;
    int agg_num = 0;
    struct wifi_mac* p_wifi_mac =  wifi_mac_get_mac_handle();
    reg_val = hif->hif_ops.hi_read_word(RG_BT_PMU_A16);
    reg_val2 = hif->hif_ops.hi_read_word(RG_PMU_A16);
    reg_val3 = hif->hif_ops.hi_read_word(RG_COEX_BT_LOGIC_INFO);

    wnet_vif = drv_priv->drv_wnet_vif_table[wnet_vif_id];
    AML_PRINT_LOG_INFO("vid %d \n", wnet_vif_id);

   WIFINET_VMACS_LOCK(p_wifi_mac);
   do {
       /*not open coexistence function*/
        if (drv_priv->drv_config.cfg_wifi_bt_coexist_support == 0) {
            break;
        }

        if ((wnet_vif->vm_opmode != WIFINET_M_STA) || wnet_vif->vm_state != WIFINET_S_CONNECTED || ( drv_priv->drv_config.cfg_txaggr == 0)) {
            break;
        }

        if ((p_wifi_mac->bt_lk != (!((reg_val2 & BIT(31)) >> 31) && (((reg_val & BIT(24)) >> 24) || ((reg_val & BIT(25)) >> 25))))
            || (bt_lk_change == 1))
        {
             p_wifi_mac->bt_lk = (!((reg_val2 & BIT(31)) >>   31) && (((reg_val & BIT(24)) >> 24) || ((reg_val & BIT(25)) >> 25)));
             wifi_mac_set_channel_rssi(p_wifi_mac, (unsigned char)(wnet_vif->vm_mainsta->sta_avg_bcn_rssi));
             AML_PRINT_LOG_INFO("p_wifi_mac->bt_lk,value=%d %d %d %d\n", p_wifi_mac->bt_lk, !((reg_val2 & BIT(31)) >> 31), ((reg_val & BIT(24)) >> 24), ((reg_val & BIT(25)) >> 25));
        }

        if (bt_lk_change == 1) {/*BT link info change*/
            if (wnet_vif->vm_state == WIFINET_S_CONNECTED)
            {
                //drv_trigger_send_delba(0, 0, 0, 0, 0);
            }

        } else {

             agg_num = p_wifi_mac->g_rs_baparamset_buffersize;
             AML_PRINT_LOG_INFO("drv_intr_bt_info_change reg addr=0x%x,value=0x%x reg addr=0x%x,value=0x%x \n", RG_BT_PMU_A16, reg_val, RG_PMU_A16, reg_val2);
             if ((reg_val & BIT(31)) && (reg_val2 & BIT(31)))
             {
                p_wifi_mac->wm_bt_en = 1;
                if (reg_val3 & COEX_EN_ESCO)
                {
                    p_wifi_mac->wm_esco_en = 1;
                    p_wifi_mac->wm_a2dp_en = 0;
                    agg_num = DEFAULT_TXAMPDU_SUB_MAX_COEX_ESCO;
                    drv_priv->drv_config.cfg_ampdu_subframes = DEFAULT_TXAMPDU_SUB_MAX_COEX_ESCO;
                    AML_PRINT_LOG_INFO("coex change to work TDD ESCO\n");
                }
                else if (reg_val & BIT(24))
                {
                    p_wifi_mac->wm_esco_en = 0;
                    p_wifi_mac->wm_a2dp_en = 1;
                    agg_num = DEFAULT_TXAMPDU_SUB_MAX_COEX;
                    drv_priv->drv_config.cfg_ampdu_subframes = DEFAULT_TXAMPDU_SUB_MAX_COEX;
                    AML_PRINT_LOG_INFO("coex change to work TDD A2DP\n");
                }
                else
                {
                    agg_num = DEFAULT_TXAMPDU_SUB_MAX;
                    AML_PRINT_LOG_INFO("coex change for abnormal case \n");
                }
             }
             else
             {
                agg_num = DEFAULT_TXAMPDU_SUB_MAX;
                drv_priv->drv_config.cfg_ampdu_subframes = DEFAULT_TXAMPDU_SUB_MAX;
                p_wifi_mac->wm_bt_en = 0;
                AML_PRINT_LOG_INFO("coex change to not work\n");
             }

             if ((p_wifi_mac->g_rs_baparamset_buffersize !=  0) && (agg_num != p_wifi_mac->g_rs_baparamset_buffersize))
             {
                AML_PRINT_LOG_INFO("delba now:num_now: %d; num_before: %d \n", agg_num, p_wifi_mac->g_rs_baparamset_buffersize);
                if (wnet_vif->vm_state == WIFINET_S_CONNECTED)
                {
                    //drv_trigger_send_delba(0, 0, 0, 0, 0);
                }
             }
    #if 0
             if ((reg_val & BIT(31)) && (reg_val2 & BIT(31))) { /* coex is work */
                drv_priv->drv_config.cfg_ampdu_subframes = DEFAULT_TXAMPDU_SUB_MAX_COEX_ESCO;
                p_wifi_mac->wm_bt_en = 1;
                AML_PRINT_LOG_INFO("coex change to work\n");

             } else { /* coex is not work */
                drv_priv->drv_config.cfg_ampdu_subframes = DEFAULT_TXAMPDU_SUB_MAX;
                p_wifi_mac->wm_bt_en = 0;
                AML_PRINT_LOG_INFO("coex change to not work\n");
             }
    #endif
        }
    } while (0);
    WIFINET_VMACS_UNLOCK(p_wifi_mac);
}

extern int32_t sysfsCreateSysFsEntry(void);
extern int32_t CreateProcEntry(void);
extern int32_t aml_proc_init(void);
extern void aml_proc_deinit(void);

static int
drv_dev_probe(void)
{
    struct vm_wlan_net_vif_params vm_param;
    struct hal_private *hal_priv = hal_get_priv();
    struct drv_private *drv_priv = drv_get_drv_priv();
    struct wifi_mac *wm_mac = wifi_mac_get_mac_handle();
    int ret = 0;
    int vif0opmode = -1;
    int vif1opmode = -1;
    char * vmac0 = NULL;
    char * vmac1 = NULL;
    int ifname_len = 0;

    /*2 init hal, download fw, host init */
    if ((hal_priv->hal_ops.hal_probe != NULL) && (!hal_priv->hal_ops.hal_probe())) {
        AML_PRINT_LOG_ERR("init hal error\n");
        goto err_ret;
    }

    if (efuse_manual_read(0xe) & BIT(26))
        AML_PRINT_LOG_INFO("auto calculate xosc");
    else
        AML_PRINT_LOG_INFO("not auto calculate xosc");

    memset(drv_priv, 0, sizeof(struct drv_private));
    memset(wm_mac, 0, sizeof(struct wifi_mac));

    wifi_mac_init_ops(wm_mac);

    aml_wifi_set_mac_addr();
    AML_PRINT_LOG_INFO("mac_addr set done."MAC_FMT"",
        mac_addr0,mac_addr1,mac_addr2, mac_addr3,mac_addr4,mac_addr5);

    /*3 init driver */
    if (aml_drv_attach(drv_priv, wm_mac)) {
        AML_PRINT_LOG_ERR("init driver error\n");
        goto err_ret;
    }

    /*4 init wifimac */
    if (wifi_mac_entry(wm_mac, drv_priv) != 0) {
        AML_PRINT_LOG_ERR( "init wifimac error\n");
        goto err_ret;
    }

    /*5 create vmac 'wlan0' and 'p2p0' */
    vmac0 = aml_wifi_get_vif0_name();
    ifname_len = strnlen(vmac0, IFNAMSIZ);
    if (vmac0 != NULL && ifname_len < IFNAMSIZ) {
        memcpy(&vm_param.vm_param_name, vmac0, ifname_len + 1);
    } else {
        AML_PRINT_LOG_ERR( "vmac0 == NULL or ifname_len >= IFNAMSIZ\n");
        goto err_ret;
    }

    vif0opmode = aml_wifi_get_vif0_opmode();
    if ((vif0opmode >= WIFINET_M_IBSS) && (vif0opmode <= WIFINET_M_P2P_DEV)) {
        vm_param.vm_param_opmode = vif0opmode;
    } else {
        vm_param.vm_param_opmode = WIFINET_M_STA;
    }
    ret = drv_priv->net_ops->wifi_mac_create_vmac(wm_mac, &vm_param,0);

    vmac1 = aml_wifi_get_vif1_name();
    ifname_len = strnlen(vmac1, IFNAMSIZ);
    if (vmac1 != NULL && ifname_len < IFNAMSIZ) {
    memcpy(&vm_param.vm_param_name, vmac1, ifname_len + 1);
    } else {
        AML_PRINT_LOG_ERR( "vmac1 == NULL or ifname_len >= IFNAMSIZ\n");
        goto err_ret;
    }

    vif1opmode = aml_wifi_get_vif1_opmode();
    if ((vif1opmode >= WIFINET_M_IBSS) && (vif1opmode <= WIFINET_M_P2P_DEV)) {
        vm_param.vm_param_opmode = vif1opmode;
    } else {
        vm_param.vm_param_opmode = WIFINET_M_P2P_DEV;
    }
    ret = drv_priv->net_ops->wifi_mac_create_vmac(wm_mac, &vm_param,0);

    /*6 init station */
    drv_priv->net_ops->wifi_mac_sta_attach(wm_mac);
    ret = register_inetaddr_notifier(&aml_inetaddr_cb);
    ret = register_inet6addr_notifier(&aml_inet6addr_cb);
    //should not be here
    drv_priv->drv_ops.drv_set_bmfm_info(drv_priv, 0, 0, 0, 0);
    drv_hal_enable_coexist(drv_priv->drv_config.cfg_wifi_bt_coexist_support );
    aml_proc_init();
    sysfsCreateSysFsEntry();
    CreateProcEntry();

    return ret;

err_ret:
    aml_drv_detach(drv_priv);
    return -ENODEV;
}


int drv_dev_remove(void)
{
    struct hal_private *hal_priv = hal_get_priv();
    struct drv_private* drv_priv = drv_get_drv_priv();
    struct wifi_mac* wm_mac = wifi_mac_get_mac_handle();

    hal_priv->hi_task_stop = 1;
    drv_priv->net_ops->wifi_mac_mac_exit(wm_mac);

    unregister_inetaddr_notifier(&aml_inetaddr_cb);
    unregister_inet6addr_notifier(&aml_inet6addr_cb);
    aml_drv_detach(drv_priv);
    aml_proc_deinit();

    AML_PRINT_LOG_INFO("<running>\n");
    return 0;
}

struct aml_hal_call_backs hal_call_back_table =
{
    /* Callback Functions */
    .get_defaultcfg = drv_get_default_cfg,
    .mic_error_event = drv_mic_error_event,
    .intr_tx_handle = drv_tx_irq_tasklet,
    .intr_tx_ok_timeout = drv_tx_ok_timeout,
    .intr_tx_pkt_clear = drv_tx_pkt_clear,
    .intr_rx_handle = drv_intr_rx_ok,
    .pmf_encrypt_pkt_handle = pmf_encrypt_pkt_handle,
    .intr_bcn_send = drv_intr_bcn_send_ok,
    .intr_dtim_send = drv_intr_dtim_send_ok,
    .intr_ba_recv = drv_intr_ba_recv_ok,
    .get_stationid = drv_get_sta_id,
    .intr_gotodeepsleep = drv_goto_deep_sleep,
    .intr_gotowakeup = drv_goto_wakeup,
    .dev_probe = drv_dev_probe,
    .dev_remove = drv_dev_remove,
    .intr_p2p_opps_cwend = drv_intr_p2p_opps_cwend,
    .intr_p2p_noa_start = drv_intr_p2p_noa_start,
    .intr_fw_event = drv_intr_fw_event,
    .intr_tx_nulldata_handle = drv_intr_tx_null_data,
    .intr_beacon_miss_handle = drv_intr_beacon_miss,
    .drv_intr_bt_info_change  = drv_intr_bt_info_change,
    .drv_pwrsave_wake_req = drv_pwrsave_wake_req,
    .drv_trace_nl_init = aml_log_nl_init,
    .drv_trace_nl_deinit = aml_log_nl_deinit,
};

struct aml_hal_call_backs* get_hal_call_back_table(void)
{
    return (struct aml_hal_call_backs *)&hal_call_back_table;
}

unsigned char drv_calc_agg_num(struct drv_private *drv_priv, unsigned char ampdu_subframe_num)
{
    unsigned char agg_num = 0;

    if (aml_bus_type == 1) {
        unsigned char tx_page_num = 0;
        unsigned char amsdu_page_num = 0;
        unsigned int efuse_val = 0;

        /* bit[24:22] indicate family revision, 5 means w1u usb revc*/
        efuse_val = drv_read_efuse_val(drv_priv, VID_PID_EFUSE_ADDR);
        amsdu_page_num = AMSDU_MAX_BUFFER_SIZE / PAGE_LEN_USB;

        if (((efuse_val >> FAMILY_REV_START) & FAMILY_REV_MASK) < W1U_USB_REV_C) {
            agg_num = AGG_NUM_PRE_USB;
        } else {
            tx_page_num = drv_get_tx_page_total_num(drv_priv);
            agg_num = tx_page_num / (2 * AMPDU_NUM_ONE_TIME_TX * amsdu_page_num);
        }
        agg_num = MIN(agg_num, ampdu_subframe_num);
    } else if (aml_bus_type == 0) {
        agg_num = ampdu_subframe_num;
    }

    return agg_num;
}

unsigned short drv_get_tx_page_total_num(struct drv_private *drv_priv)
{
    return drv_priv->hal_priv->hal_ops.hal_get_tx_page_total_num();
}

unsigned int drv_read_efuse_val(struct drv_private *drv_priv, unsigned int efuse_addr)
{
    return drv_priv->hal_priv->hal_ops.hal_read_efuse_val(efuse_addr);
}
