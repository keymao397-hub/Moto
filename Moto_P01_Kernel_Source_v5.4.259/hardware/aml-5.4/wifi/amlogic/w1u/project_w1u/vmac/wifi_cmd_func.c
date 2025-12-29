#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include "wifi_mac_com.h"
#include "wifi_cmd_func.h"
#include "wifi_drv_reg_ops.h"
#include "wifi_cfg80211.h"
#include "wifi_mac_if.h"
#include "wifi_iwpriv_cmd.h"


struct udp_info aml_udp_info[32];
int udp_cnt = 0;
struct udp_timer aml_udp_timer;
extern struct _B2B_Test_Case_Packet gB2BTestCasePacket;

cmd_to_func_table_t cmd_to_func[] =
{
    {"set_chip_id", aml_set_chip_id},
    {"get_chip_id", aml_get_chip_id},
    {"set_mac_addr", aml_set_mac_addr},
    {"get_mac_addr", aml_get_mac_addr},
    {"get_wifi_mac", aml_get_wifi_mac_addr},
    {"set_bt_dev_id", aml_set_bt_device_id},
    {"get_bt_dev_id", aml_get_bt_device_id},
    {"sta_send_bareq", aml_sta_send_addba_req},
    {"set_bss_coex", aml_sta_send_coexist_mgmt},
    {"get_txaggr_sta", aml_wpa_get_txaggr_status},
    {"sta_get_session", aml_sta_get_wfd_session},
    {"wmm_ac_addts",  aml_wmm_ac_addts},
    {"wmm_ac_delts",  aml_wmm_ac_delts},
#ifdef CONFIG_P2P
    {"get_p2p_dev_id", aml_get_p2p_device_addr},
#endif
    {"set_amsdu", aml_set_mac_amsdu},
    {"set_ampdu", aml_set_drv_ampdu},
    {"update_wmm_arg", aml_update_wmm_arg},
    {"set_dynamic_bw", aml_wpa_set_dynamic_bw},
    {"set_short_gi", aml_wpa_set_short_gi},
    {"set_eat_count", aml_set_eat_count_max},
    {"set_aggr_thr", aml_set_aggr_thresh},
    {"set_hrt_intv", aml_set_hrtimer_interval},
    {"get_ap_ip", aml_get_ap_ip},
    {"set_roam_2gthr", aml_set_roaming_threshold_2g},//wpa_cli driver set_roam_thr_2g -80
    {"set_roam_5gthr", aml_set_roaming_threshold_5g},
    {"get_roam_chan", aml_get_roaming_candidate_chans},
    {"set_roam_chan", aml_set_roaming_candidate_chans},
    {"set_roam_mode", aml_set_roaming_mode},
    {"set_udp_info", aml_set_udp_info},
    {"get_udp_info", aml_get_udp_info},
    {"set_dfs_mark",aml_mark_dfs_channel},
    {"set_dfs_unmark",aml_unmark_dfs_channel},
    {"set_dev_sn", aml_set_device_sn},
    {"get_dev_sn", aml_get_device_sn},
    {"set_nb_thr", aml_set_signal_power_weak_thresh_for_narrow_bandwidth},
    {"set_wb_thr", aml_set_signal_power_weak_thresh_for_wide_bandwidth},
    {"set_rate_legacy", aml_set_lagecy_bitrate_mask},
    {"set_rate_ht", aml_set_ht_bitrate_mask},
    {"set_rate_vht", aml_set_vht_bitrate_mask},
    {"set_rate_auto", aml_set_rate_auto},
    {"set_wifi_slcreq", aml_set_coex_req_timeslice_timeout},
    {"set_coex_bcnmis", aml_set_coex_max_miss_bcn},
    {"set_sc_hang", aml_set_scan_hang},
    {"set_sc_contime", aml_set_scan_connect_time},
    {"set_sc_idltime", aml_set_scan_idle_time},
    {"set_reg", aml_wpa_set_reg},
    {"get_reg", aml_wpa_get_reg},
    {"get_cca_stat", aml_wpa_get_cca_status},
    {"get_agc_stat", aml_wpa_get_agc_status},
    {"get_tx_stat", aml_wpa_get_latest_tx_status},
    {"get_drv_ver", aml_wpa_get_drv_ver},
    {"set_country", aml_wpa_set_country_code},
    {"get_country", aml_wpa_get_country_code},
    {"set_pwr_save", aml_wpa_set_power},
    {"set_chl_rssi", aml_wpa_set_channel_rssi},
    {"set_burst", aml_wpa_set_burst},
    {"set_coex_btwifi", aml_wpa_set_coex_btwifi},
    {"set_coex_bcnmis", aml_wpa_set_coex_bcnmis},
    {"set_bcn_intv", aml_wpa_set_bcn_intv},
    {"set_ldpc", aml_wpa_set_ldpc},
#if defined(SU_BF) || defined(MU_BF)
    {"set_beamforming", aml_wpa_set_beamforming},
#endif
    {"get_chan_list", aml_wpa_get_chan_list},
    {"set_uapsd", aml_wpa_set_uapsd},
    {"set_pt_rxstart", aml_wpa_set_pt_rxstart},
    {"set_pt_rxstop", aml_wpa_set_pt_rxstop},
    {"set_scan_pri", aml_wpa_set_scan_pri},
    {"set_bebk_pri", aml_wpa_set_bebk_pri},
    {"set_pkt_fetch", aml_wpa_set_pkt_fetch},
    {"set_frag_thr", aml_wpa_set_frag_thr},
    {"set_preamble", aml_wpa_set_preamble},
    {"set_band", aml_wpa_set_band},
    {"set_mac_mode", aml_wpa_set_mac_mode},
    {"set_gain_hang", aml_wpa_set_initial_gain_change_hang},
    {"set_tpc_hang", aml_wpa_set_tx_power_change_hang},
    {"set_tx_pw_plan", aml_set_tx_power_plan},
    {"set_debug", aml_wpa_set_debug},
    {"set_efuse", aml_wpa_set_efuse},
    {"get_efuse", aml_wpa_get_efuse},
    {"", NULL},
};

//Returns a char * arr [] and size is the length of the returned array
char **aml_cmd_char_parse(char sep, const char *str, int *size)
{
    int count = 0;
    int i;
    char **ret;
    int lastindex = -1;
    int j = 0;

    for (i = 0; i < strlen(str); i++) {
        if (str[i] == sep) {
            count++;
        }
    }

    ret = (char **)kzalloc((++count) * sizeof(char *), GFP_KERNEL);

    for (i = 0; i < strlen(str); i++) {
        if (str[i] == sep) {
            // kzalloc the memory space of substring length + 1
            ret[j] = (char *)kzalloc((i - lastindex) * sizeof(char), GFP_KERNEL);
            memcpy(ret[j], str + lastindex + 1, i - lastindex - 1);
            j++;
            lastindex = i;
        }
    }
    //Processing the last substring
    if (lastindex <= strlen(str) - 1) {
        ret[j] = (char *)kzalloc((strlen(str) - lastindex) * sizeof(char), GFP_KERNEL);
        memcpy(ret[j], str + lastindex + 1, strlen(str) - 1 - lastindex);
        j++;
    }

    *size = j;

    return ret;
}

/*
*  cmd: sta_send_bareq 11:22:33:44:55:66 5
*/
int aml_sta_send_addba_req(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    int skip = 0;
    char mac[18];
    int tid = 0;

    skip = strlen("sta_send_bareq") + 1;
    sscanf(buf + skip, "%s %d", mac, &tid);
    wifi_mac_send_addba_req(buf + skip, tid);
    return 0;
}

int aml_set_chip_id(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    int i,cmd_arg;
    char sep = ' ';

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg) {
        for (i = 0; i < 32; i++) {
            if (simple_strtoul(arg[2],NULL,16) & (1 << i)) {
                efuse_manual_write(i, 8);
            }
        }
        for (i = 0; i < 16; i++) {
            if (simple_strtoul(arg[1],NULL,16) & (1 << i)) {
                efuse_manual_write(i, 9);
            }
        }
        AML_PRINT_LOG_INFO("write chip_id is :%04x%08x\n", simple_strtoul(arg[1],NULL,16) & 0xffff,
            simple_strtoul(arg[2],NULL,16));

        FREE(arg, "cmd_arg");
    }

    return 0;
}

int aml_get_chip_id(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    int cmd_arg;
    char sep = ' ';
    unsigned int efuse_data_l = 0;
    unsigned int efuse_data_h = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg) {
        efuse_data_l = efuse_manual_read(simple_strtoul(arg[1],NULL,16));
        efuse_data_h = efuse_manual_read(simple_strtoul(arg[2],NULL,16));

        AML_PRINT_LOG_INFO("efuse addr:%08x,%08x, chip_id is :%04x%08x\n", simple_strtoul(arg[1],NULL,16),
            simple_strtoul(arg[2],NULL,16), efuse_data_h & 0xffff, efuse_data_l);
        FREE(arg, "cmd_arg");
    }

    return 0;
}

int aml_wpa_set_efuse(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    int i,cmd_arg;
    char sep = ' ';

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg) {
        if (arg[1] && arg[2]) {
            for (i = 0; i < 32; i++) {
                if (simple_strtoul(arg[2],NULL,16) & (1 << i)) {
                    efuse_manual_write(i, simple_strtoul(arg[1],NULL,16));
                }
            }
            printk("write addr is :%x, data :%08x\n", simple_strtoul(arg[1],NULL,16),
                simple_strtoul(arg[2],NULL,16));
        } else {
            printk("data error: %s", __func__);
        }
    }
    FREE(arg, "cmd_arg");

    return 0;

}

int aml_wpa_get_efuse(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    int cmd_arg;
    char sep = ' ';
    unsigned int efuse_data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg) {
        if (arg[1]) {
            efuse_data = efuse_manual_read(simple_strtoul(arg[1],NULL,16));

            printk("efuse addr:%x, data is :%08x\n", simple_strtoul(arg[1],NULL,16), efuse_data);

        } else {
            printk("data error: %s", __func__);
        }
    }
    FREE(arg, "cmd_arg");

    return 0;

}

int aml_set_mac_addr(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char **mac_cmd;
    int i,cmd_arg;
    char sep = ' ';
    unsigned int efuse_data_l = 0;
    unsigned int efuse_data_h = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg) {
        sep = ':';
        mac_cmd = aml_cmd_char_parse(sep, arg[1], &cmd_arg);
        if (mac_cmd) {
            efuse_data_l = (simple_strtoul(mac_cmd[2],NULL,16) << 24) | (simple_strtoul(mac_cmd[3],NULL,16) << 16)
                           | (simple_strtoul(mac_cmd[4],NULL,16) << 8) | simple_strtoul(mac_cmd[5],NULL,16);
            efuse_data_h = (simple_strtoul(mac_cmd[0],NULL,16) << 8) | (simple_strtoul(mac_cmd[1],NULL,16));
            for (i = 0; i < 32; i++) {
                if (efuse_data_l & (1 << i)) {
                    efuse_manual_write(i, 1);
                }
            }
            for (i = 0; i < 16; i++) {
                if (efuse_data_h & (1 << i)) {
                    efuse_manual_write(i, 2);
                }
            }
            AML_PRINT_LOG_INFO("write MAC addr is:  %02x:%02x:%02x:%02x:%02x:%02x\n",
                    (efuse_data_h & 0xff00) >> 8,efuse_data_h & 0x00ff, (efuse_data_l & 0xff000000) >> 24,
                    (efuse_data_l & 0x00ff0000) >> 16,(efuse_data_l & 0xff00) >> 8,efuse_data_l & 0xff);
        }

        kfree(mac_cmd);
        FREE(arg, "cmd_arg");
    }

    return 0;
}

int aml_get_mac_addr(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    int cmd_arg;
    char sep = ' ';
    unsigned int efuse_data_l = 0;
    unsigned int efuse_data_h = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg) {
        efuse_data_l = efuse_manual_read(simple_strtoul(arg[1],NULL,16));
        efuse_data_h = efuse_manual_read(simple_strtoul(arg[2],NULL,16));
        AML_PRINT_LOG_INFO("efuse addr:%08x,%08x, MAC addr is:  %02x:%02x:%02x:%02x:%02x:%02x\n",
                simple_strtoul(arg[1],NULL,16), simple_strtoul(arg[2],NULL,16),
                (efuse_data_h & 0xff00) >> 8,efuse_data_h & 0x00ff, (efuse_data_l & 0xff000000) >> 24,
                (efuse_data_l & 0x00ff0000) >> 16,(efuse_data_l & 0xff00) >> 8,efuse_data_l & 0xff);
        FREE(arg, "cmd_arg");
    }

    return 0;
}
int aml_get_wifi_mac_addr(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    int bytes_written = 0;

    bytes_written = snprintf(buf, len, MAC_FMT, MAC_ARG(wnet_vif->vm_myaddr));
    AML_PRINT_LOG_INFO("buf:  %s\n", buf);
    return bytes_written;
}

int aml_set_bt_device_id(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char **mac_cmd;
    int i,cmd_arg;
    char sep = ' ';
    unsigned int efuse_data_l = 0;
    unsigned int efuse_data_h = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg) {
        sep = ':';
        mac_cmd = aml_cmd_char_parse(sep, arg[1], &cmd_arg);
        if (mac_cmd) {
            efuse_data_h = (simple_strtoul(mac_cmd[0],NULL,16) << 24) | (simple_strtoul(mac_cmd[1],NULL,16) << 16)
                           | (simple_strtoul(mac_cmd[2],NULL,16) << 8) | simple_strtoul(mac_cmd[3],NULL,16);
            efuse_data_l = (simple_strtoul(mac_cmd[4],NULL,16) << 24) | (simple_strtoul(mac_cmd[5],NULL,16) << 16);
            for (i = 0; i < 32; i++) {
                if (efuse_data_h & (1 << i)) {
                    efuse_manual_write(i, 3);
                }
            }
            for (i = 16; i < 32; i++) {
                if (efuse_data_l & (1 << i)) {
                    efuse_manual_write(i, 2);
                }
            }
            AML_PRINT_LOG_INFO("write BT device id  is:  %02x:%02x:%02x:%02x:%02x:%02x\n",
                    (efuse_data_h & 0xff000000) >> 24, (efuse_data_h & 0x00ff0000) >> 16,
                    (efuse_data_h & 0xff00) >> 8,efuse_data_h & 0xff,
                    (efuse_data_l & 0xff000000) >> 24,(efuse_data_l & 0x00ff0000) >> 16);
        }

        kfree(mac_cmd);
        FREE(arg, "cmd_arg");
    }

    return 0;

}

int aml_get_bt_device_id(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    int cmd_arg;
    char sep = ' ';
    unsigned int efuse_data_l = 0;
    unsigned int efuse_data_h = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg) {
        efuse_data_l = efuse_manual_read(simple_strtoul(arg[1],NULL,16));
        efuse_data_h = efuse_manual_read(simple_strtoul(arg[2],NULL,16));
        AML_PRINT_LOG_INFO("efuse addr:%08x,%08x, get BT device id is:  %02x:%02x:%02x:%02x:%02x:%02x\n",
                simple_strtoul(arg[1],NULL,16), simple_strtoul(arg[2],NULL,16),
                (efuse_data_h & 0xff000000) >> 24, (efuse_data_h & 0x00ff0000) >> 16,
                (efuse_data_h & 0xff00) >> 8,efuse_data_h & 0xff,
                (efuse_data_l & 0xff000000) >> 24,(efuse_data_l & 0x00ff0000) >> 16);
        FREE(arg, "cmd_arg");
    }

    return 0;
}


int aml_sta_send_coexist_mgmt(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    int skip = 0;

    skip = strlen("set_bss_coex") + 1;
    wifi_mac_send_coexist_mgmt(buf + skip);
    return 0;
}

int aml_wpa_get_txaggr_status(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char* str[2] = {"all", "reset"};
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }

    data = simple_strtol(arg[1], NULL, 0);
    len = strlen("get_txaggr_sta ") + strlen(str[data]) + 1;
    snprintf(buf, len, "get_txaggr_sta %s", str[data]);
    AML_PRINT_LOG_INFO("buf %s\n", buf);
    aml_get_drv_txaggr_status(wnet_vif, buf, len);

    FREE(arg, "cmd_arg");
    return 0;
}


int aml_get_drv_txaggr_status(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int i, j, cmd_arg;
    unsigned int nframes;
    struct drv_private *drv_priv = drv_get_drv_priv();

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);

    //get queue-0 statistics
    if (likely(!arg[1])) {
        i = 0;
        for (j = 0; j < 16; j++) {
            nframes = drv_priv->drv_txlist_table[i].tx_aggr_status[j];
            AML_PRINT_LOG_INFO("Q:%d; aggr %d packets times: %d\n", i, j + 1, nframes);
        }
        AML_PRINT_LOG_INFO("\n");

    } else {
        //get all statistics
        if (strnicmp(arg[1], "all", strlen("all")) == 0) {
            for (i = 0; i < HAL_NUM_TX_QUEUES; i++) {
                for (j = 0; j < 16; j++) {
                    nframes = drv_priv->drv_txlist_table[i].tx_aggr_status[j];
                    AML_PRINT_LOG_INFO("Q:%d; aggr %d packets times: %d\n", i, j + 1, nframes);
                }
                AML_PRINT_LOG_INFO("\n");
            }
        }
        //reset
        if (strnicmp(arg[1], "reset", strlen("reset")) == 0) {
            for (i = 0; i < HAL_NUM_TX_QUEUES; i++) {
                for (j = 0; j < 16; j++) {
                    drv_priv->drv_txlist_table[i].tx_aggr_status[j] = 0;
                }
            }
        }
    }

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_sta_get_wfd_session(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    AML_PRINT_LOG_INFO("wfd_session_id=%s\n", wnet_vif->vm_p2p->wfd_session_id);
    return 0;
}

int aml_get_p2p_device_addr(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    int bytes_written = 0;

    memcpy(buf, wnet_vif->vm_ndev->dev_addr, ETH_ALEN);
    AML_PRINT_LOG_INFO("p2p_device_addr= %02x:%02x:%02x:%02x:%02x:%02x\n", *buf, *(buf + 1), *(buf + 2),
           *(buf + 3), *(buf + 4), *(buf + 5));

    bytes_written = ETH_ALEN;
    return bytes_written;

}


int aml_wmm_ac_addts(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    int skip = 0, cmd_arg;
    char **arg;
    char sep = ' ';
    skip = strlen("wmm_ac_addts") + 1;

    arg = aml_cmd_char_parse(sep, buf + skip, &cmd_arg);
    wifi_mac_send_wmm_ac_addts(arg);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_wmm_ac_delts(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    int skip = 0;
    skip = strlen("wmm_ac_delts") + 1;
    wifi_mac_send_wmm_ac_delts(buf + skip);
    return 0;
}

void aml_set_mac_amsdu_switch_state(char* arg)
{
    struct drv_private *drv_priv = drv_get_drv_priv();

    if (!arg) {
        if (drv_priv->drv_config.cfg_txamsdu) {
            AML_PRINT_LOG_INFO("AMSDU is ON.\n");
        }
        else {
            AML_PRINT_LOG_INFO("AMSDU is OFF.\n");
        }
    }
    else {
        if (strnicmp(arg, "on", strlen("on")) == 0) {
            drv_priv->drv_config.cfg_txamsdu = 1;
            AML_PRINT_LOG_INFO("AMSDU set ON!\n");

        }
        else if (strnicmp(arg, "off", strlen("off")) == 0) {
            drv_priv->drv_config.cfg_txamsdu = 0;
            AML_PRINT_LOG_INFO("AMSDU set OFF!\n");

        }
        else {
            if (drv_priv->drv_config.cfg_txamsdu) {
                AML_PRINT_LOG_INFO("AMSDU is ON.\n");
            }
            else {
                AML_PRINT_LOG_INFO("AMSDU is OFF.\n");
            }
        }
    }
}

void aml_set_drv_ampdu_switch_state(char* arg)
{
    struct drv_private *drv_priv = drv_get_drv_priv();

    if (!arg) {
        if (drv_priv->drv_config.cfg_txaggr) {
            AML_PRINT_LOG_INFO("AMPDU is ON.\n");
        }
        else {
            AML_PRINT_LOG_INFO("AMPDU is OFF.\n");
        }
    }
    else {
        if (strnicmp(arg, "on", strlen("on")) == 0) {
            drv_priv->drv_config.cfg_txaggr = 1;
            AML_PRINT_LOG_INFO("AMPDU set ON!\n");
        }
        else if (strnicmp(arg, "off", strlen("off")) == 0) {
            drv_priv->drv_config.cfg_txaggr = 0;
            AML_PRINT_LOG_INFO("AMPDU set OFF!\n");

        }
        else {
            if (drv_priv->drv_config.cfg_txaggr) {
                AML_PRINT_LOG_INFO("AMPDU is ON.\n");
            }
            else {
                AML_PRINT_LOG_INFO("AMPDU is OFF.\n");
            }
        }
    }
}

int aml_set_mac_amsdu(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);

    aml_set_mac_amsdu_switch_state(arg[1]);

    FREE(arg, "cmd_arg");
    return 0;
}

static struct wifi_channel * wifi_mac_find_chan_unlock( struct wifi_mac *wifimac, unsigned short cur_chan,
    enum wifi_mac_bwc_width cur_bw, unsigned short cur_freq)
{
    struct wifi_channel *c = NULL;
    int i = 0;

    for (i = 0; i < wifimac->wm_nchans; i++) {
        c = &wifimac->wm_channels[i];
        if ((c->chan_pri_num== cur_chan) && (c->chan_bw == cur_bw) && (c->chan_cfreq1 == cur_freq)) {
            AML_PRINT(AML_LOG_ID_BWC,AML_LOG_LEVEL_DEBUG, "find a chan=%d, bw=%d and freq=%d.\n",cur_chan, cur_bw, cur_freq);
            return c;
        }
    }

    AML_PRINT(AML_LOG_ID_BWC,AML_LOG_LEVEL_WARN, "NOT find a chan=%d, bw=%d and freq=%d.\n", cur_chan, cur_bw, cur_freq);
    return NULL;
}

void wifi_mac_set_country_regdom_task(SYS_TYPE param1, SYS_TYPE param2, SYS_TYPE param3, SYS_TYPE param4, SYS_TYPE param5)
{
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    struct drv_private* drv_priv = wifimac->drv_priv;
    struct wlan_net_vif *selected_wnet_vif = drv_priv->drv_wnet_vif_table[NET80211_MAIN_VMAC];
    struct wlan_net_vif *wnet_vif = NULL;//for iterator
    struct wifi_channel old_chans[WIFI_MAX_VID] = {0};
    struct wifi_channel saved_roaming_chans[ROAMING_CANDIDATE_CHAN_MAX] = {0};
    struct wifi_channel *pchan = NULL;
    struct wifi_channel *roaming_chan = NULL;
    unsigned char roaming_chan_cnt = 0;
    unsigned char alpha[3] = {param1, param2, 0};
    unsigned char cur_txpwrplan = 0;
    unsigned char idx = 0;

    if ((wifimac->wm_nrunning == 1) && (drv_priv->drv_wnet_vif_table[NET80211_P2P_VMAC]->vm_state == WIFINET_S_CONNECTED)) {
        selected_wnet_vif = drv_priv->drv_wnet_vif_table[NET80211_P2P_VMAC];
    }

    if ((alpha[0] == wifimac->wm_country.iso[0]) && (alpha[1] == wifimac->wm_country.iso[1])) {
        AML_PRINT_LOG_ERR("no need to set country code due to the same country code\n");
        goto end;
    }

    if (preempt_scan(selected_wnet_vif->vm_ndev, 100, 100) != 0) {
        AML_PRINT_LOG_INFO("delay country switch: target country %s\n", alpha);
        wifimac->wm_alpha_pending = 1;
        goto end;
    }

    WIFI_ALPHA_LOCK(wifimac);
    if ((alpha[0] != wifimac->wm_alpha_target[0]) || (alpha[1] != wifimac->wm_alpha_target[1])) {
        AML_PRINT_LOG_INFO("alpha [%s] has beed covered by target alpha [%s], waste country [%s]\n", alpha, wifimac->wm_alpha_target, alpha);
        alpha[0] = wifimac->wm_alpha_target[0];
        alpha[1] = wifimac->wm_alpha_target[1];
    } else {
        AML_PRINT_LOG_INFO("alpha=%s\n", alpha);
    }

    WIFI_CHANNEL_LOCK(wifimac);

    list_for_each_entry(wnet_vif, &wifimac->wm_wnet_vifs, vm_next) {
        pchan = &(old_chans[wnet_vif->wnet_vif_id]);
        if (!WIFINET_IS_CHAN_ERR(wnet_vif->vm_curchan)) {
            memcpy(pchan, wnet_vif->vm_curchan, sizeof(struct wifi_channel));
        }
    }
    cur_txpwrplan = drv_priv->drv_config.cfg_txpoweplan;

    // save roaming candidate channels and repointer after country set
    WIFI_ROAMING_CHANNEL_LOCK(wifimac->wm_scan);
    for (idx = 0; idx < wifimac->wm_scan->roaming_candidate_chans_cnt; idx ++) {
        roaming_chan = wifimac->wm_scan->roaming_candidate_chans[idx].channel;
        if (WIFINET_IS_CHAN_ERR(roaming_chan)) {
            break;
        }
        saved_roaming_chans[idx] = *roaming_chan;
        roaming_chan_cnt ++;
    }
    wifi_mac_set_country(wifimac, alpha);
    wifimac->wm_scan->roaming_candidate_chans_cnt = 0;
    for (idx = 0; idx < roaming_chan_cnt; idx ++) {
        pchan = saved_roaming_chans + idx;
        roaming_chan = wifi_mac_find_chan_unlock(wifimac, pchan->chan_pri_num, pchan->chan_bw, pchan->chan_cfreq1);
        if (roaming_chan == NULL) {
            continue;
        }
        wifimac->wm_scan->roaming_candidate_chans[idx].channel = roaming_chan;
        wifimac->wm_scan->roaming_candidate_chans_cnt ++;
    }
    WIFI_ROAMING_CHANNEL_UNLOCK(wifimac->wm_scan);

    list_for_each_entry(wnet_vif, &wifimac->wm_wnet_vifs, vm_next) {
        pchan = &(old_chans[wnet_vif->wnet_vif_id]);
        if (pchan->chan_cfreq1 != 0) {
            wnet_vif->vm_curchan = wifi_mac_find_chan_unlock(wifimac, pchan->chan_pri_num, pchan->chan_bw, pchan->chan_cfreq1);
            if (!wnet_vif->vm_curchan) {

                WIFI_CHANNEL_UNLOCK(wifimac);

                if ((wnet_vif->vm_opmode == WIFINET_M_HOSTAP) && (wnet_vif->vm_state == WIFINET_S_CONNECTED)) {
                    if (wifi_mac_p2p_home_channel_enabled(wnet_vif) && !wifi_mac_if_dfs_channel(wifimac, wifimac->wm_p2p_home_channel)) {
                        channel_switch_announce_trigger(wifimac, wifimac->wm_p2p_home_channel, WIFINET_BWC_WIDTH20, wifimac->wm_p2p_home_channel);
                    } else {
                        if (if_southamerica_country(wifimac->wm_country.iso)) {
                            channel_switch_announce_trigger(wifimac, 149, WIFINET_BWC_WIDTH20, 149);
                        } else {
                            channel_switch_announce_trigger(wifimac, 36, WIFINET_BWC_WIDTH20, 36);
                        }
                    }
                } else {
                    wifi_mac_top_sm(wnet_vif, WIFINET_S_INIT,0);
                }

                WIFI_CHANNEL_LOCK(wifimac);
            } else {
                AML_PRINT_LOG_INFO("vif[%d] now chan info => pri_num: %d, bw: %d, chan_cfreq1: %d\n", wnet_vif->wnet_vif_id,
                    wnet_vif->vm_curchan->chan_pri_num, wnet_vif->vm_curchan->chan_bw, wnet_vif->vm_curchan->chan_cfreq1);
            }
        }
    }

    if( cur_txpwrplan != drv_priv->drv_config.cfg_txpoweplan) {
        wifi_mac_set_tx_power_coefficient(drv_priv, selected_wnet_vif->vm_curchan ,drv_priv->drv_config.cfg_txpoweplan);
    }

    WIFI_CHANNEL_UNLOCK(wifimac);

    wifimac->wm_alpha_set_in_progress = 0;
    WIFI_ALPHA_UNLOCK(wifimac);

    return;
end:
    WIFI_ALPHA_LOCK(wifimac);
    wifimac->wm_alpha_set_in_progress = 0;
    WIFI_ALPHA_UNLOCK(wifimac);
    return;
}

void wifi_mac_set_country_regdom(char* arg)
{
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    wifi_mac_add_work_task(wifimac, wifi_mac_set_country_regdom_task, NULL, arg[0], arg[1], 0, 0, 0);
}

void wifi_mac_ap_set_11h(unsigned char channel)
{
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    struct drv_private* drv_priv = wifimac->drv_priv;
    struct wlan_net_vif *selected_wnet_vif = NULL;

    if (wifimac->wm_nrunning == 1) {
        if (drv_priv->drv_wnet_vif_table[NET80211_P2P_VMAC]->vm_state == WIFINET_S_CONNECTED) {
            selected_wnet_vif = drv_priv->drv_wnet_vif_table[NET80211_P2P_VMAC];
        } else {
            selected_wnet_vif = drv_priv->drv_wnet_vif_table[NET80211_MAIN_VMAC];
        }
    } else {
        AML_PRINT_LOG_ERR("no sta connected\n");
        return;
    }

    wifimac->wm_doth_tbtt = 255; //simple_strtoul(*buf, NULL, 0);
    //wifimac->wm_doth_channel = simple_strtoul(*(buf + 1), NULL, 0);
    wifimac->wm_doth_channel = channel;
    AML_PRINT_LOG_INFO("wm_dott_tbtt=%d, wm_doth_channel=%d\n", wifimac->wm_doth_tbtt, wifimac->wm_doth_channel);
    wifimac->wm_flags |= WIFINET_F_CHANSWITCH;
    wifimac->wm_flags |= WIFINET_F_DOTH;
    selected_wnet_vif->vm_chanchange_count = 0;

    return;
}

void wifi_mac_ap_set_arp_rx(char** buf)
{
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    struct drv_private* drv_priv = wifimac->drv_priv;
    struct wlan_net_vif *selected_wnet_vif = NULL;

    if (wifimac->wm_nrunning == 1) {
        if (drv_priv->drv_wnet_vif_table[NET80211_P2P_VMAC]->vm_state == WIFINET_S_CONNECTED) {
            selected_wnet_vif = drv_priv->drv_wnet_vif_table[NET80211_P2P_VMAC];
        } else {
            selected_wnet_vif = drv_priv->drv_wnet_vif_table[NET80211_MAIN_VMAC];
        }

    } else {
        AML_PRINT_LOG_ERR("no sta connected\n");
        return;
    }
    selected_wnet_vif->vm_arp_rx.src_mac_addr[0] = simple_strtoul(*buf, NULL, 0);
    selected_wnet_vif->vm_arp_rx.src_mac_addr[1] = simple_strtoul(*(buf+1), NULL, 0);
    selected_wnet_vif->vm_arp_rx.src_mac_addr[2] = simple_strtoul(*(buf+2), NULL, 0);
    selected_wnet_vif->vm_arp_rx.src_mac_addr[3] = simple_strtoul(*(buf+3), NULL, 0);
    selected_wnet_vif->vm_arp_rx.src_mac_addr[4] = simple_strtoul(*(buf+4), NULL, 0);
    selected_wnet_vif->vm_arp_rx.src_mac_addr[5] = simple_strtoul(*(buf+5), NULL, 0);
    selected_wnet_vif->vm_arp_rx.src_ip_addr[0] = simple_strtoul(*(buf+6), NULL, 0);
    selected_wnet_vif->vm_arp_rx.src_ip_addr[1] = simple_strtoul(*(buf+7), NULL, 0);
    selected_wnet_vif->vm_arp_rx.src_ip_addr[2] = simple_strtoul(*(buf+8), NULL, 0);
    selected_wnet_vif->vm_arp_rx.src_ip_addr[3] = simple_strtoul(*(buf+9), NULL, 0);
    selected_wnet_vif->vm_arp_rx.out = simple_strtoul(*(buf+10), NULL, 0);
    AML_PRINT_LOG_INFO("set mac=%02x:%02x:%02x:%02x:%02x:%02x\n", selected_wnet_vif->vm_arp_rx.src_mac_addr[0],selected_wnet_vif->vm_arp_rx.src_mac_addr[1],
           selected_wnet_vif->vm_arp_rx.src_mac_addr[2], selected_wnet_vif->vm_arp_rx.src_mac_addr[3], selected_wnet_vif->vm_arp_rx.src_mac_addr[4], selected_wnet_vif->vm_arp_rx.src_mac_addr[5]);
    AML_PRINT_LOG_INFO("set ip=%d.%d.%d.%d\n", selected_wnet_vif->vm_arp_rx.src_ip_addr[0], selected_wnet_vif->vm_arp_rx.src_ip_addr[1],
           selected_wnet_vif->vm_arp_rx.src_ip_addr[2],selected_wnet_vif->vm_arp_rx.src_ip_addr[3]);
    wifi_mac_set_arp_rsp(selected_wnet_vif);
    return;
}


int aml_set_drv_ampdu(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);

    aml_set_drv_ampdu_switch_state(arg[1]);

    FREE(arg, "cmd_arg");
    return 0;
}
int aml_update_wmm_arg(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char **param;
    int cmd_arg, cmd_param;
    char sep = ' ';
    int ac = 0;
    int i = 0;
    struct wifi_mac_wme_state *wme = &wnet_vif->vm_wmac->wm_wme[wnet_vif->wnet_vif_id];

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);

    if ((wme != NULL) && (arg[1] != NULL)) {
        ac = simple_strtoul(arg[1], NULL, 0);
        if ((ac < WME_NUM_AC) && (arg[2] != NULL)) {
            struct wmeParams *wmep = &wme->wme_wmeChanParams.cap_wmeParams[ac];

            for (i = 2; i < cmd_arg; i++) {
                sep = ':';
                param = aml_cmd_char_parse(sep, arg[i], &cmd_param);
                if (strnicmp(arg[i], "acm", strlen("acm")) == 0) {
                    wmep->wmep_acm = simple_strtoul(param[1], NULL, 0);

                } else if (strnicmp(arg[i], "aifsn", strlen("aifsn")) == 0) {
                    wmep->wmep_aifsn = simple_strtoul(param[1], NULL, 0);

                } else if (strnicmp(arg[i], "cwmin", strlen("cwmin")) == 0) {
                    wmep->wmep_logcwmin = simple_strtoul(param[1], NULL, 0);

                } else if (strnicmp(arg[i], "cwmax", strlen("cwmax")) == 0) {
                    wmep->wmep_logcwmax = simple_strtoul(param[1], NULL, 0);

                } else if (strnicmp(arg[i], "limit", strlen("limit")) == 0) {
                    wmep->wmep_txopLimit = simple_strtoul(param[1], NULL, 0);

                } else if (strnicmp(arg[i], "noackPolicy", strlen("noackPolicy")) == 0) {
                    wmep->wmep_noackPolicy = simple_strtoul(param[1], NULL, 0);
                } else {
                    AML_PRINT_LOG_ERR("Parameter [%s] is error.\n", arg[i]);
                }
                kfree(param);
            }

        } else {
            AML_PRINT_LOG_ERR("Parameter is error.\n");
            FREE(arg, "cmd_arg");
            return 0;
        }

        wifi_mac_wme_updateparams(wnet_vif);
    }

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_wpa_set_dynamic_bw(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char* str[2] = {"off", "on"};
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    len = strlen("get_txaggr_sta ") + strlen(str[data]) + 1;
    snprintf(buf, len, "get_txaggr_sta %s", str[data]);
    AML_PRINT_LOG_INFO("buf %s\n", buf);
    aml_set_dynamic_bw(wnet_vif, buf, len);

    FREE(arg, "cmd_arg");
    return 0;
}


int aml_set_dynamic_bw(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;
    struct drv_private *drv_priv = drv_get_drv_priv();

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if ((drv_priv != NULL) && (arg[1] != NULL)) {
        if (strnicmp(arg[1], "on", strlen("on")) == 0) {
            data = 1;
            AML_PRINT_LOG_INFO("dynamic_bw set ON!\n");

        } else if (strnicmp(arg[1], "off", strlen("off")) == 0) {
            data = 0;
            AML_PRINT_LOG_INFO("dynamic_bw set OFF!\n");

        } else {
            AML_PRINT_LOG_ERR("Parameter is error,now cfg_dynamic_bw=%d.\n", drv_priv->drv_config.cfg_dynamic_bw);
            FREE(arg, "cmd_arg");
            return 0;
        }
        drv_set_config((void *)drv_priv, CHIP_PARAM_DYNAMIC_BW, data);

    } else {
        AML_PRINT_LOG_ERR("Parameter is error.\n");
    }

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_wpa_set_short_gi(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char* str[2] = {"off", "on"};
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    len = strlen("get_txaggr_sta ") + strlen(str[data]) + 1;
    snprintf(buf, len, "get_txaggr_sta %s", str[data]);
    AML_PRINT_LOG_INFO("buf %s\n", buf);
    aml_set_short_gi(wnet_vif, buf, len);

    FREE(arg, "cmd_arg");
    return 0;
}


int aml_set_short_gi(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    int usr_data;
    char **arg;
    char sep = ' ';
    int cmd_arg;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (!arg[1]) {
        FREE(arg, "cmd_arg");
        AML_PRINT_LOG_ERR("--Bad parameter\n");
        return 0;
    }
    AML_PRINT_LOG_INFO("cmd param: %s, %s\n", arg[0], arg[1]);

    if (strnicmp(arg[1], "on", strlen("on")) == 0) {
        usr_data = 1;
        AML_PRINT_LOG_INFO("enable short GI.\n");

    } else if (strnicmp(arg[1], "off", strlen("off")) == 0) {
        usr_data = 0;
        AML_PRINT_LOG_INFO("disable short GI.\n");

    } else {
        AML_PRINT_LOG_INFO("--Bad parameter \n");
        FREE(arg, "cmd_arg");
        return 0;
    }

    if (aml_wifi_is_enable_rf_test())
        gB2BTestCasePacket.if_shortGI = usr_data;

    if (1 == usr_data) {
        wnet_vif->vm_wmac->wm_flags_ext |= WIFINET_FEXT_SHORTGI_ENABLE;
        AML_PRINT_LOG_INFO("enable short GI done\n");

    } else {
        wnet_vif->vm_wmac->wm_flags_ext &= ~WIFINET_FEXT_SHORTGI_ENABLE;
        AML_PRINT_LOG_INFO("disable short GI done.\n");
    }


    FREE(arg, "cmd_arg");
    return 0;
}

int aml_set_eat_count_max(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;
    struct drv_private *drv_priv = drv_get_drv_priv();

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if ((drv_priv != NULL) && (arg[1] != NULL)) {
        data = simple_strtoul(arg[1], NULL, 0);
        drv_set_config((void *)drv_priv, CHIP_PARAM_EAT_COUNT, data);

    } else {
        AML_PRINT_LOG_ERR("Parameter is error.\n");
    }

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_set_aggr_thresh(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;
    struct drv_private *drv_priv = drv_get_drv_priv();

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if ((drv_priv != NULL) && (arg[1] != NULL)) {
        data = simple_strtoul(arg[1], NULL, 0);
        drv_set_config((void *)drv_priv, CHIP_PARAM_AGGR_THRESH, data);

    } else {
        AML_PRINT_LOG_ERR("Parameter is error.\n");
    }

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_set_hrtimer_interval(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;
    struct drv_private *drv_priv = drv_get_drv_priv();

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if ((drv_priv != NULL) && (arg[1] != NULL)) {
        data = simple_strtoul(arg[1], NULL, 0);
        drv_set_config((void *)drv_priv, CHIP_PARAM_HRTIMER_INTERVAL, data);

    } else {
        AML_PRINT_LOG_ERR("Parameter is error.\n");
    }

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_get_ap_ip(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    if (!(wnet_vif->vm_mainsta->sta_arp_flag & WIFI_ARP_GET_AP_IP)) {
        wifi_mac_send_arp_req(wnet_vif);

    } else {
        AML_PRINT_LOG_INFO("ap ip is:%d:%d:%d:%d\n", wnet_vif->vm_mainsta->sta_ap_ip[0],
            wnet_vif->vm_mainsta->sta_ap_ip[1], wnet_vif->vm_mainsta->sta_ap_ip[2], wnet_vif->vm_mainsta->sta_ap_ip[3]);
    }

    return 0;
}


int aml_set_roaming_threshold_2g(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if ((wnet_vif != NULL)&& (wnet_vif->vm_wmac != NULL) && (arg[1] != NULL)) {
        data = simple_strtol(arg[1], NULL, 0);
        wnet_vif->vm_wmac->roaming_threshold_2g = data;
        AML_PRINT_LOG_INFO("vm_wmac->roaming_threshold_2g:%d\n", wnet_vif->vm_wmac->roaming_threshold_2g);

    } else {
        AML_PRINT_LOG_ERR("Parameter is error.\n");
    }

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_set_roaming_threshold_5g(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if ((wnet_vif != NULL) && (wnet_vif->vm_wmac != NULL) && (arg[1] != NULL)) {
        data = simple_strtol(arg[1], NULL, 0);
        wnet_vif->vm_wmac->roaming_threshold_5g = data;
        AML_PRINT_LOG_INFO("vm_wmac->roaming_threshold_5g:%d\n", wnet_vif->vm_wmac->roaming_threshold_5g);

    } else {
        AML_PRINT_LOG_ERR("Parameter is error.\n");
    }

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_get_roaming_candidate_chans(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    int i = 0;
    struct wifi_channel * chan = NULL;
    int chan_cnt = wnet_vif->vm_wmac->wm_scan->roaming_candidate_chans_cnt;

    AML_PRINT_LOG_INFO("get roaming candidate chans [%d]:\n", chan_cnt);

    for (i=0; i < chan_cnt; i++) {
        chan = wnet_vif->vm_wmac->wm_scan->roaming_candidate_chans[i].channel;

        if (chan == NULL) {
            AML_PRINT_LOG_ERR("erro pointer \n");
            return 0;
        }
        AML_PRINT_LOG_INFO("chan_cfreq1:%d chan_flags:%d chan_pri_num:%d chan_maxpower:%d chan_minpower:%d chan_bw:%d global_operating_class %d Rssi:%d\n",
            chan->chan_cfreq1, chan->chan_flags, chan->chan_pri_num, chan->chan_maxpower,
            chan->chan_minpower, chan->chan_bw, chan->global_operating_class,
            (wnet_vif->vm_wmac->wm_scan->roaming_candidate_chans[i].avg_rssi - 256));
    }

    return 0;
}

int aml_set_roaming_candidate_chans(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;
    int i = 0;
    int j = 0;
    struct wifi_channel *c;
    struct wifi_mac *wifimac = NULL;
    struct wifi_mac_scan_state *ss = NULL;

    if (wnet_vif == NULL) {
        return 0;
    }
    wifimac = wnet_vif->vm_wmac;
    ss = wifimac->wm_scan;

    WIFI_ROAMING_CHANNEL_LOCK(wnet_vif->vm_wmac->wm_scan);

    wnet_vif->vm_wmac->wm_scan->roaming_candidate_chans_cnt = 0;
    memset(wnet_vif->vm_wmac->wm_scan->roaming_candidate_chans, 0, sizeof(wnet_vif->vm_wmac->wm_scan->roaming_candidate_chans));

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    for (i = 1; i < ROAMING_CANDIDATE_CHAN_MAX; i++) {
        if (arg[i] != NULL) {
            data = simple_strtoul(arg[i], NULL, 0);
            AML_PRINT_LOG_INFO("get channel:%d\n",data);
            WIFI_CHANNEL_LOCK(wifimac);
            for (j = 0; j < wifimac->wm_nchans; j++) {
                c = &wifimac->wm_channels[j];
                if (c->chan_bw == WIFINET_BWC_WIDTH20 && c->chan_pri_num == data) {
                    ss->roaming_candidate_chans[ss->roaming_candidate_chans_cnt].channel = c;
                    ss->roaming_candidate_chans[ss->roaming_candidate_chans_cnt].avg_rssi = 156;
                    ss->roaming_candidate_chans_cnt++;
                }
            }
            WIFI_CHANNEL_UNLOCK(wifimac);

        } else {
           WIFI_ROAMING_CHANNEL_UNLOCK(wnet_vif->vm_wmac->wm_scan);
           FREE(arg, "cmd_arg");
           return 0;
        }
    }
    WIFI_ROAMING_CHANNEL_UNLOCK(wnet_vif->vm_wmac->wm_scan);

    FREE(arg, "cmd_arg");
    return 0;
}


int aml_set_roaming_mode(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if ((wnet_vif != NULL) && (wnet_vif->vm_wmac != NULL) && (arg[1] != NULL)) {
        data = simple_strtoul(arg[1], NULL, 0);
        wnet_vif->vm_wmac->wm_roaming = data;
        AML_PRINT_LOG_INFO("vm_wmac->wm_roaming:%d\n", wnet_vif->vm_wmac->wm_roaming);
    } else {
        AML_PRINT_LOG_ERR("Parameter is error.\n");
    }

    FREE(arg, "cmd_arg");
    return 0;
}

int wifi_mac_set_udp_info(char** buf)
{
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    struct drv_private* drv_priv = wifimac->drv_priv;
    struct wlan_net_vif *selected_wnet_vif = NULL;
    unsigned int param;
    if (wifimac->wm_nrunning == 1) {
        if (drv_priv->drv_wnet_vif_table[NET80211_P2P_VMAC]->vm_state == WIFINET_S_CONNECTED) {
            selected_wnet_vif = drv_priv->drv_wnet_vif_table[NET80211_P2P_VMAC];
        } else {
            selected_wnet_vif = drv_priv->drv_wnet_vif_table[NET80211_MAIN_VMAC];
        }
    } else {
        AML_PRINT_LOG_ERR("no sta connected\n");
        return -1;
    }

    aml_udp_info[udp_cnt].dst_port = simple_strtoul(*buf, NULL, 0);
    aml_udp_info[udp_cnt].src_port = simple_strtoul(*(buf + 1), NULL, 0);
    aml_udp_info[udp_cnt].dst_ip = simple_strtoul(*(buf + 2), NULL, 0);
    aml_udp_info[udp_cnt].src_ip = simple_strtoul(*(buf + 3), NULL, 0);
    aml_udp_info[udp_cnt].pkt_len = simple_strtoul(*(buf + 4), NULL, 0);
    param = simple_strtoul(*(buf + 5), NULL, 0);
    aml_udp_info[udp_cnt].out = param >> 8;
    aml_udp_info[udp_cnt].streamid = param & 0xff;
    aml_udp_info[udp_cnt].seq = 1000;
    aml_udp_info[udp_cnt].rx = 0;
    aml_udp_info[udp_cnt].tx = 0;
    aml_udp_info[udp_cnt].rx_totlen = 0;
    aml_udp_timer.udp_timer_stop = 0;

    AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_WARN,"dst_port=%04x, src_port=%04x, dst_ip=%08x, src_ip:%08x, pkt_len:%d, out:%d, streamid:%d, param=%d\n",
        aml_udp_info[udp_cnt].dst_port, aml_udp_info[udp_cnt].src_port, aml_udp_info[udp_cnt].dst_ip, aml_udp_info[udp_cnt].src_ip,
        aml_udp_info[udp_cnt].pkt_len, aml_udp_info[udp_cnt].out, aml_udp_info[udp_cnt].streamid, param);

    if (aml_udp_info[udp_cnt].out && aml_udp_timer.run_flag == 0) {
        os_timer_ex_initialize(&aml_udp_timer.udp_send_timeout, 0, wifi_mac_udp_send_timeout_ex, selected_wnet_vif);
        os_timer_ex_start_period(&aml_udp_timer.udp_send_timeout, 10);
        aml_udp_timer.run_flag = 1;
    }
    udp_cnt++;
    return 0;
}

//set_udp_info dst_port src_port dst_ip in/out
int aml_set_udp_info(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    int skip = 0, cmd_arg;
    char **arg;
    char sep = ' ';
    skip = strlen("set_udp_info") + 1;

    arg = aml_cmd_char_parse(sep, buf + skip, &cmd_arg);
    wifi_mac_set_udp_info(arg);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_get_udp_info(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    int i = 0;
    for (i = 0; i < udp_cnt; i++) {
        AML_PRINT_LOG_INFO("streamid=%d tx is %d, rx is %d, rx tot_len is %d\n", aml_udp_info[i].streamid, aml_udp_info[i].tx, aml_udp_info[i].rx, aml_udp_info[i].rx_totlen);
    }
    aml_udp_timer.udp_timer_stop = 1;
    aml_udp_timer.run_flag = 0;
    os_timer_ex_del(&aml_udp_timer.udp_send_timeout, CANCEL_SLEEP);
    return 0;
}

int aml_unmark_dfs_channel(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if ((wnet_vif != NULL) && (wnet_vif->vm_wmac != NULL)) {
        if (arg[1] != NULL) {
            data = simple_strtoul(arg[1], NULL, 0);
        }

        AML_PRINT_LOG_INFO("unmarked dfs channel :%d\n",data);
        wifi_mac_unmark_dfs_channel(wnet_vif, data);

   } else {
        AML_PRINT_LOG_ERR("Parameter is error.\n");
   }

   FREE(arg, "cmd_arg");
   return 0;
}

int aml_mark_dfs_channel(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
     char **arg;
     char sep = ' ';
     int cmd_arg;
     int data = 0;

     arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
     if ((wnet_vif != NULL) && (wnet_vif->vm_wmac != NULL)) {
         if (arg[1] != NULL) {
             data = simple_strtoul(arg[1], NULL, 0);
         }

         AML_PRINT_LOG_INFO("marked dfs channel :%d\n",data);
         wifi_mac_mark_dfs_channel(wnet_vif, data);

    } else {
         AML_PRINT_LOG_ERR("Parameter is error.\n");
    }

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_set_device_sn(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char **mac_cmd;
    int i,cmd_arg;
    char sep = ' ';
    unsigned int efuse_data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg) {
        sep = ':';
        mac_cmd = aml_cmd_char_parse(sep, arg[1], &cmd_arg);
        if (mac_cmd) {
            efuse_data = (simple_strtoul(mac_cmd[0],NULL,16) << 8) | (simple_strtoul(mac_cmd[1],NULL,16));
            for (i = 0; i < 16; i++) {
                if (efuse_data & (1 << i)) {
                    //AML_PRINT_LOG_INFO("set_dev_sn ===>>> efuse_manual_write: %d 0xf\n", i);
                    efuse_manual_write(i, 0xf);
                }
            }
            AML_PRINT_LOG_INFO("set_dev_sn ===>>> write SN/ID is: %02x %02x\n", ((efuse_data & 0xff00) >> 8), (efuse_data & 0x00ff));
        }
        kfree(mac_cmd);
    }

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_get_device_sn(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    int cmd_arg;
    char sep = ' ';
    unsigned int efuse_data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg) {
        efuse_data = efuse_manual_read(simple_strtoul(arg[1],NULL,16));
        AML_PRINT_LOG_INFO("get_dev_sn ===>>> efuse addr:%08x, get SN/ID is: %02x %02x\n", simple_strtoul(arg[1],NULL,16), ((efuse_data & 0xff00) >> 8), (efuse_data & 0x00ff));
        FREE(arg, "cmd_arg");
    }

    return 0;
}

int aml_set_signal_power_weak_thresh_for_narrow_bandwidth(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if ((wnet_vif != NULL) && (wnet_vif->vm_wmac != NULL) && (arg[1] != NULL)) {
        data = simple_strtol(arg[1], NULL, 0);
        wnet_vif->vm_wmac->wm_signal_power_weak_thresh_narrow = data;
        AML_PRINT_LOG_INFO("set signal power weak thresh to: %d\n", wnet_vif->vm_wmac->wm_signal_power_weak_thresh_narrow);

    } else {
        AML_PRINT_LOG_ERR("Parameter is error.\n");
    }

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_set_signal_power_weak_thresh_for_wide_bandwidth(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if ((wnet_vif != NULL) && (wnet_vif->vm_wmac != NULL) && (arg[1] != NULL)) {
        data = simple_strtol(arg[1], NULL, 0);
        wnet_vif->vm_wmac->wm_signal_power_weak_thresh_wide = data;
        AML_PRINT_LOG_INFO("set signal power weak thresh to: %d\n", wnet_vif->vm_wmac->wm_signal_power_weak_thresh_wide);

    } else {
        AML_PRINT_LOG_ERR("Parameter is error.\n");
    }

    FREE(arg, "cmd_arg");
    return 0;
}

extern int aml_iwpriv_set_lagecy_bitrate_mask(struct net_device *dev, unsigned int set);
int aml_set_lagecy_bitrate_mask(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    aml_iwpriv_set_lagecy_bitrate_mask(wnet_vif->vm_ndev, data);

    FREE(arg, "cmd_arg");
    return 0;
}

extern int aml_iwpriv_set_ht_bitrate_mask(struct net_device *dev, unsigned int set);
int aml_set_ht_bitrate_mask(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    aml_iwpriv_set_ht_bitrate_mask(wnet_vif->vm_ndev, data);

    FREE(arg, "cmd_arg");
    return 0;
}


extern int aml_iwpriv_set_vht_bitrate_mask(struct net_device *dev, unsigned int set);
int aml_set_vht_bitrate_mask(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    aml_iwpriv_set_vht_bitrate_mask(wnet_vif->vm_ndev, data);

    FREE(arg, "cmd_arg");
    return 0;
}

extern void aml_iwpriv_set_rate_auto(struct wlan_net_vif *wnet_vif);
int aml_set_rate_auto(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    aml_iwpriv_set_rate_auto(wnet_vif);
    return 0;
}

int aml_set_coex_req_timeslice_timeout(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    struct wifi_mac *wifimac = NULL;
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    wifimac = wifi_mac_get_mac_handle();
    wifimac->drv_priv->hal_priv->hal_ops.phy_set_coexist_req_timeslice_timeout_value(data);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_set_coex_max_miss_bcn(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    struct wifi_mac *wifimac = NULL;
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    wifimac = wifi_mac_get_mac_handle();
    wifimac->drv_priv->hal_priv->hal_ops.phy_set_coexist_max_miss_bcn(data);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_set_scan_hang(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    wnet_vif->vm_scan_hang = (unsigned char)data;
    AML_PRINT_LOG_INFO("vid:%d vm_scan_hang:%d\n", wnet_vif->wnet_vif_id, wnet_vif->vm_scan_hang);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_set_scan_connect_time(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    wnet_vif->vm_scan_time_connect = (unsigned char)data;
    wifi_mac_set_scan_time(wnet_vif);
    AML_PRINT_LOG_INFO("vid:%d vm_scan_time_connect:%d\n", wnet_vif->wnet_vif_id, wnet_vif->vm_scan_time_connect);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_set_scan_idle_time(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    wnet_vif->vm_scan_time_idle = (unsigned char)data;
    wifi_mac_set_scan_time(wnet_vif);
    AML_PRINT_LOG_INFO("vid:%d vm_scan_idle:%d\n", wnet_vif->wnet_vif_id, wnet_vif->vm_scan_time_idle);

    FREE(arg, "cmd_arg");
    return 0;
}

unsigned int get_reg(struct wlan_net_vif *wnet_vif, unsigned int set);
int aml_wpa_get_reg(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    get_reg(wnet_vif, data);

    FREE(arg, "cmd_arg");
    return 0;
}

unsigned int set_reg(struct wlan_net_vif *wnet_vif, unsigned int set1, unsigned int set2);
int aml_wpa_set_reg(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data1 = 0;
    int data2 = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data1 = simple_strtol(arg[1], NULL, 0);
    data2 = simple_strtol(arg[2], NULL, 0);
    set_reg(wnet_vif, data1, data2);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_wpa_get_cca_status(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    struct wifi_mac *wifimac = NULL;

    wifimac = wifi_mac_get_mac_handle();
    wifimac->drv_priv->drv_ops.cca_busy_check();

    return 0;
}

int aml_wpa_get_agc_status(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    struct wifi_mac *wifimac = NULL;

    wifimac = wifi_mac_get_mac_handle();
    wifimac->drv_priv->drv_ops.phy_stc();

    return 0;
}

unsigned int get_latest_tx_status(struct wifi_mac *wifimac);
int aml_wpa_get_latest_tx_status(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    struct wifi_mac *wifimac = NULL;

    wifimac = wifi_mac_get_mac_handle();
    get_latest_tx_status(wifimac);

    return 0;
}

extern void print_driver_version(void);
int aml_wpa_get_drv_ver(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    print_driver_version();
    return 0;
}

int aml_wpa_set_country_code(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);

    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }

    AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_INFO, "set country <%s> by supp-priv\n", arg[1]);

    // kernel has bug this operatin will let "iw reg set xx" not work
    // regulatory_hint(wnet_vif->vm_wdev->wiphy,arg[1]);

    wifi_mac_set_country_code(arg[1]);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_wpa_get_country_code(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    struct wifi_mac *wifimac = NULL;
    wifimac = wifi_mac_get_mac_handle();
    AML_PRINT_LOG_INFO("country code: %s\n", wifimac->wm_country.iso);

    return 0;
}

extern void wifi_mac_pwrsave_set_inactime(struct wlan_net_vif *wnet_vif, unsigned int time);
int aml_wpa_set_power(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;
    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        AML_PRINT_LOG_ERR("char_phrase fail\n");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    wifi_mac_pwrsave_set_inactime(wnet_vif, data);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_wpa_set_channel_rssi(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    struct wifi_mac *wifimac = NULL;
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    wifimac = wifi_mac_get_mac_handle();
    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    wifimac->drv_priv->drv_ops.set_channel_rssi(wifimac->drv_priv, data, 0);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_wpa_set_burst(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    struct wifi_mac *wifimac = NULL;
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    wifimac = wifi_mac_get_mac_handle();
    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    wifimac->drv_priv->drv_config.cfg_burst_ack = data;
    AML_PRINT_LOG_INFO("wpa cli set burst %d\n", data);

    FREE(arg, "cmd_arg");
    return 0;
}

extern void aml_iwpriv_set_uapsd(struct wlan_net_vif *wnet_vif, unsigned int set);
int aml_wpa_set_uapsd(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    struct wifi_mac *wifimac = NULL;
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    wifimac = wifi_mac_get_mac_handle();
    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    aml_iwpriv_set_uapsd(wnet_vif, data);
    AML_PRINT_LOG_INFO("wpa cli set uapsd %d\n", data);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_wpa_set_pt_rxstart(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    wnet_vif->vif_ops.pt_rx_start(data);
    AML_PRINT_LOG_INFO("wpa cli set rxstart %d\n", data);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_wpa_set_pt_rxstop(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    wnet_vif->vif_ops.pt_rx_stop();
    return 0;
}

int aml_wpa_set_scan_pri(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    struct wifi_mac *wifimac = NULL;
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    wifimac = wifi_mac_get_mac_handle();
    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    wifimac->drv_priv->hal_priv->hal_ops.phy_set_coexist_scan_priority_range(data);
    AML_PRINT_LOG_INFO("wpa cli set scan pri %d\n", data);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_wpa_set_bebk_pri(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    struct wifi_mac *wifimac = NULL;
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    wifimac = wifi_mac_get_mac_handle();
    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    wifimac->drv_priv->hal_priv->hal_ops.phy_set_coexist_be_bk_noqos_priority_range(data);
    AML_PRINT_LOG_INFO("wpa cli set bebk noqos pri %d\n", data);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_wpa_set_coex_btwifi(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    struct wifi_mac *wifimac = NULL;
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    wifimac = wifi_mac_get_mac_handle();
    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    AML_PRINT_LOG_INFO("coexist en= %d\n", data);
    wifimac->drv_priv->hal_priv->hal_ops.phy_set_coexist_en(data);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_wpa_set_coex_bcnmis(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    struct wifi_mac *wifimac = NULL;
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    wifimac = wifi_mac_get_mac_handle();
    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    AML_PRINT_LOG_INFO("coexist value=%d\n", data);
    wifimac->drv_priv->hal_priv->hal_ops.phy_set_coexist_max_miss_bcn(data);

    FREE(arg, "cmd_arg");
    return 0;
}

extern int aml_beacon_intvl_set(struct wlan_net_vif *wnet_vif, unsigned int set);
int aml_wpa_set_bcn_intv(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    AML_PRINT_LOG_INFO("coexist value=%d\n", data);
    aml_beacon_intvl_set(wnet_vif, data);

    FREE(arg, "cmd_arg");
    return 0;
}

extern int aml_set_ldpc(struct wlan_net_vif *wnet_vif, unsigned int set);
int aml_wpa_set_ldpc(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    AML_PRINT_LOG_INFO("coexist value=%d\n", data);
    aml_set_ldpc(wnet_vif, data);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_wpa_get_chan_list(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    struct wifi_mac *wifimac = NULL;
    struct wifi_channel *c = NULL;
    int i = 0;
    wifimac = wifi_mac_get_mac_handle();

    WIFI_CHANNEL_LOCK(wifimac);
    for (i = 0; i < wifimac->wm_nchans; i++) {
        c = &wifimac->wm_channels[i];
        AML_PRINT_LOG_INFO("channel:%d\t frequency:%d \t bandwidth:%dMHz\n", c->chan_pri_num, c->chan_cfreq1, ((1 << c->chan_bw) * 20));
    }
    WIFI_CHANNEL_UNLOCK(wifimac);

    return 0;
}

#if defined(SU_BF) || defined(MU_BF)
extern int aml_set_beamforming(struct wlan_net_vif *wnet_vif, unsigned int set1,unsigned int set2);
int aml_wpa_set_beamforming(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data1 = 0;
    int data2 = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data1 = simple_strtol(arg[1], NULL, 0);
    data2 = simple_strtol(arg[2], NULL, 0);
    aml_set_beamforming(wnet_vif, data1,data1);

    FREE(arg, "cmd_arg");
    return 0;
}
#endif

int aml_wpa_set_pkt_fetch(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    wnet_vif->vm_mainsta->sta_fetch_pkt_method = (unsigned char)data;
    AML_PRINT_LOG_INFO("wpa cli set pkt fetch %d\n", data);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_wpa_set_frag_thr(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    if ((unsigned short)data > 0) {
        wnet_vif->vm_fragthreshold = (unsigned short)data;
    }
    AML_PRINT_LOG_INFO("wpa cli set frag thr %d\n", data);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_wpa_set_preamble(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    phy_set_preamble_type(data);
    AML_PRINT_LOG_INFO("wpa cli set preamble type %d\n", data);

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_wpa_set_band(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;
    struct drv_private *drv_priv = drv_get_drv_priv();

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    drv_priv->drv_config.cfg_band = data;

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_wpa_set_mac_mode(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;
    struct drv_private *drv_priv = drv_get_drv_priv();

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    drv_priv->drv_config.cfg_mac_mode = data;

    FREE(arg, "cmd_arg");
    return 0;
}


int aml_wpa_set_initial_gain_change_hang(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    aml_txt_parameter.initial_gain_change_disable = data;

    if (aml_txt_parameter.initial_gain_change_disable) {
        AML_PRINT_LOG_ERR("initial_gain_change invalid\n ");

    } else {
        AML_PRINT_LOG_ERR("initial_gain_change valid\n ");
    }

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_wpa_set_tx_power_change_hang(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    data = simple_strtol(arg[1], NULL, 0);
    aml_txt_parameter.tx_power_change_disable = data;

    if (aml_txt_parameter.tx_power_change_disable) {
        AML_PRINT_LOG_ERR("tx_power_change invalid\n ");

    } else {
        AML_PRINT_LOG_ERR("tx_power_change valid\n ");
    }

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_set_tx_power_plan(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    int data = 0;

    struct drv_private *drv_priv = drv_get_drv_priv();

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if ((drv_priv != NULL) && (arg[1] != NULL)) {
        data = simple_strtoul(arg[1], NULL, 0);
        wifimac_set_tx_pwr_plan(data);
    } else {
        AML_PRINT_LOG_INFO("Parameter is error.\n");
    }

    FREE(arg, "cmd_arg");
    return 0;
}

int aml_set_debug_switch(char *switch_str)
{
    int debug_switch = 0;
    if(strstr(switch_str,"_off")!=NULL)
        debug_switch = 0;
    else if(strstr(switch_str,"_on")!=NULL)
        debug_switch = 1;
    else
        AML_PRINT_LOG_ERR("input error\n");
    return debug_switch;
}

int aml_wpa_set_debug(struct wlan_net_vif *wnet_vif, char* buf, int len)
{
    char **arg;
    char sep = ' ';
    int cmd_arg;
    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    if (arg[1] == NULL) {
        FREE(arg, "cmd_arg");
        return -EINVAL;
    }
    aml_set_debug_modules(arg[1]);

    FREE(arg, "cmd_arg");
    return 0;
}

void wifi_mac_set_cf_end(struct wlan_net_vif *wnet_vif, unsigned char is_enable)
{
    unsigned char vid = 0;

    vid = wnet_vif->wnet_vif_id;
    phy_set_cf_end(vid, is_enable);
}

