#include "wifi_mac_com.h"
#include "wifi_mac_rate.h"
#include "wifi_iwpriv_cmd.h"

extern int g_auto_gain_base;
extern bool g_DFS_on;

const char *forbidden_scan_reason_str[FORBIDDEN_SCAN_MAX] =
{
    "connecting",
    "connecting scan",
    "disconnecting",
    "roaming",
    "fw_recovery",
    "dpd_recail",
};


int saveie(unsigned char *iep, const unsigned char *ie)
{
    if (ie == NULL)
    {
        (iep)[1] = 0;
    }
    else
    {
        unsigned int ielen = ie[1]+2;
        if (iep != NULL)
            memcpy(iep, ie, ielen);
        return ielen;
    }
    return 0;
}

static int wifi_mac_scan_sta_compare(const struct scaninfo_entry *a, const struct scaninfo_entry *b)
{
    if ((a->scaninfo.SI_capinfo & WIFINET_CAPINFO_PRIVACY) &&
        (b->scaninfo.SI_capinfo & WIFINET_CAPINFO_PRIVACY) == 0)
        return 1;

    if ((a->scaninfo.SI_capinfo & WIFINET_CAPINFO_PRIVACY) == 0 &&
        (b->scaninfo.SI_capinfo & WIFINET_CAPINFO_PRIVACY))
        return -1;

    if (abs(b->connectcnt - a->connectcnt) > 1)
        return b->connectcnt - a->connectcnt;

    if (abs(b->scaninfo.SI_rssi - a->scaninfo.SI_rssi) < 5)
    {
        return wifi_mac_get_sta_mode((struct wifi_scan_info *)&a->scaninfo) - wifi_mac_get_sta_mode((struct wifi_scan_info *)&b->scaninfo);
    }
    return a->scaninfo.SI_rssi - b->scaninfo.SI_rssi;
}

static int match_ssid(const unsigned char *ie, int nssid, const struct wifi_mac_ScanSSID ssids[])
{
    int i;

    for (i = 0; i < nssid; i++) {
        if ((ie[1] == ssids[i].len) && (memcmp(ie+2, ssids[i].ssid, ie[1]) == 0)) {
            return 1;
        }
    }
    return 0;
}



static int match_bss(struct wlan_net_vif *wnet_vif,
    const struct wifi_mac_scan_state *ss, const struct scaninfo_entry *se)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    const struct wifi_scan_info *scaninfo = &se->scaninfo;
    unsigned int fail;

    fail = 0;
    if (wifi_mac_chan_num_avail(wifimac, wifi_mac_chan2ieee(wifimac, scaninfo->SI_chan)) == false) {
        fail |= STA_MATCH_ERR_CHAN;
        //AML_PRINT_LOG_INFO("fail 0x%x\n", fail);
    }

    if (wnet_vif->vm_opmode == WIFINET_M_IBSS) {
        if ((scaninfo->SI_capinfo & WIFINET_CAPINFO_IBSS) == 0) {
             fail |= STA_MATCH_ERR_BSS;
              //AML_PRINT_LOG_INFO("fail 0x%x\n", fail);
        }

    } else {
        if ((scaninfo->SI_capinfo & WIFINET_CAPINFO_ESS) == 0) {
            fail |= STA_MATCH_ERR_BSS;
            //AML_PRINT_LOG_INFO("fail 0x%x\n", fail);
        }
    }

    if (wnet_vif->vm_flags & WIFINET_F_PRIVACY) {
        if ((scaninfo->SI_capinfo & WIFINET_CAPINFO_PRIVACY) == 0) {
            fail |= STA_MATCH_ERR_PRIVACY;
             //AML_PRINT_LOG_INFO("fail 0x%x\n", fail);
        }

    } else {
        if (scaninfo->SI_capinfo & WIFINET_CAPINFO_PRIVACY) {
            fail |= STA_MATCH_ERR_PRIVACY;
             //AML_PRINT_LOG_INFO("fail 0x%x\n", fail);
        }
    }

    if (!check_rate(wnet_vif, scaninfo)) {
        fail |= STA_MATCH_ERR_RATE;
        //AML_PRINT_LOG_INFO("fail 0x%x\n", fail);
    }

    if ((wnet_vif->vm_fixed_rate.mode == WIFINET_FIXED_RATE_NONE)
        || (wnet_vif->vm_fixed_rate.rateinfo & WIFINET_RATE_MCS)) {
        if (!check_ht_rate(wnet_vif, scaninfo)) {
            fail |= STA_MATCH_ERR_HTRATE;
            //AML_PRINT_LOG_INFO("fail 0x%x\n", fail);
        }
    }

    /*get ssid, which we want to connect */
    if (!(wnet_vif->vm_flags & WIFINET_F_IGNORE_SSID)
        && ((ss->ss_nssid == 0) || (match_ssid(scaninfo->SI_ssid, ss->ss_nssid, ss->ss_ssid) == 0))) {
        fail |= STA_MATCH_ERR_SSID;
        //AML_PRINT_LOG_INFO("fail 0x%x, ss->ss_nssid:%d\n", fail, ss->ss_nssid);
    }

    if ((wnet_vif->vm_flags & WIFINET_F_DESBSSID)
        && !WIFINET_ADDR_EQ(wnet_vif->vm_des_bssid, scaninfo->SI_bssid)) {
        fail |= STA_MATCH_ERR_BSSID;
         //AML_PRINT_LOG_INFO("fail 0x%x\n", fail);
    }

    if (se->connectcnt >= WIFINET_CONNECT_FAILS) {
        fail |= STA_MATCH_ERR_STA_FAILS_MAX;
         //AML_PRINT_LOG_INFO("fail 0x%x\n", fail);
    }

    if (!(fail & STA_MATCH_ERR_SSID)) {
         AML_PRINT(AML_LOG_ID_SCAN,AML_LOG_LEVEL_WARN, "flag = 0x%x, ssid=%s, chan_pri_num=%d, SI_rssi =%d, vm_flags=%x bssid=%s chan_flags=0x%x \n",
            fail, ssidie_sprintf(scaninfo->SI_ssid), scaninfo->SI_chan->chan_pri_num, scaninfo->SI_rssi,
            wnet_vif->vm_flags, ether_sprintf(scaninfo->SI_bssid), scaninfo->SI_chan->chan_flags);
    }
    return fail;
}

static void wifi_mac_save_ssid(struct wlan_net_vif *wnet_vif, struct wifi_mac_scan_state *ss,
    int nssid, const struct wifi_mac_ScanSSID ssids[])
{
    if ((nssid == 0) || (nssid > WIFINET_SCAN_MAX_SSID)) {
        return;
    }

    memcpy(ss->ss_ssid, ssids, nssid * sizeof(ssids[0]));
    ss->ss_nssid = nssid;
}


int wifi_mac_scan_chk_11g_bss(struct wifi_mac_scan_state *ss, struct wlan_net_vif *wnet_vif)
{
        struct scaninfo_table *st = ss->ScanTablePriv;
        struct scaninfo_entry *se;
        enum wifi_mac_macmode macmode = WIFINET_MODE_AUTO;
        int ret = 0;

        WIFI_SCAN_SE_LIST_LOCK(st);
        list_for_each_entry(se,&st->st_entry,se_list) {
        macmode = wifi_mac_get_sta_mode(&se->scaninfo);
        /*a 5G 11g only found, ignore it*/
        if((macmode == WIFINET_MODE_11G) && WIFINET_IS_CHAN_2GHZ((&se->scaninfo)->SI_chan)) {

            ret = 1;
            AML_PRINT(AML_LOG_ID_SCAN,AML_LOG_LEVEL_WARN, "<running>  mac mode %d addr:%x:%x:%x:%x:%x:%x\n", macmode,
                se->scaninfo.SI_macaddr[0],se->scaninfo.SI_macaddr[1],se->scaninfo.SI_macaddr[2],
                se->scaninfo.SI_macaddr[3],se->scaninfo.SI_macaddr[4],se->scaninfo.SI_macaddr[5]);
            break;
        } else {

            ret = 0;
        }
    }
    WIFI_SCAN_SE_LIST_UNLOCK(st);
    return ret;
}


static int
wifi_mac_scan_get_best_node(struct wifi_mac_scan_state *ss, struct wlan_net_vif *wnet_vif, struct scaninfo_entry *bestNode)
{
    struct scaninfo_table *st = ss->ScanTablePriv;
    struct scaninfo_entry *se = NULL;
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    int roaming_threshold = wifimac->roaming_threshold_5g;

    WIFI_SCAN_SE_LIST_LOCK(st);
    list_for_each_entry(se,&st->st_entry,se_list) {
        if (wnet_vif->vm_chan_roaming_scan_flag == 1) {
            if (!memcmp(se->scaninfo.SI_bssid, wnet_vif->vm_mainsta->sta_bssid, WIFINET_ADDR_LEN)) {
                AML_PRINT_LOG_INFO("ignore node due to same bssid with current bss");
                continue;
            }

            if (WIFINET_IS_CHAN_2GHZ(se->scaninfo.SI_chan)) {
                roaming_threshold = wifimac->roaming_threshold_2g;
            } else if (WIFINET_IS_CHAN_5GHZ(se->scaninfo.SI_chan)) {
                roaming_threshold = wifimac->roaming_threshold_5g;
            }

            if ((se->se_avgrssi - 256) < roaming_threshold) {
                if ((ss->ss_nssid > 0) && (match_ssid(se->scaninfo.SI_ssid, ss->ss_nssid, ss->ss_ssid) == 1)) {
                    AML_PRINT_LOG_INFO("ignore node due to low rssi: ssid=%s, SI_rssi=%d (rssi_thr=%d), bssid=%s\n",
                        ssidie_sprintf(se->scaninfo.SI_ssid), se->scaninfo.SI_rssi, roaming_threshold,
                        ether_sprintf(se->scaninfo.SI_bssid));
                }
                continue;
            }
        }

        if (match_bss(wnet_vif, ss, se) == 0) {
            if (bestNode->se_valid == 0) {
                memcpy(bestNode, se, sizeof(struct scaninfo_entry));

            } else if (wifi_mac_scan_sta_compare(se, bestNode) > 0) {
                memcpy(bestNode, se, sizeof(struct scaninfo_entry));
            }
        }
    }

    WIFI_SCAN_SE_LIST_UNLOCK(st);
    return bestNode->se_valid;
}

static unsigned char
wifi_mac_scan_get_match_node(struct wifi_mac_scan_state *ss, struct wlan_net_vif *wnet_vif)
{
    struct scaninfo_table *st = ss->ScanTablePriv;
    struct scaninfo_entry *se = NULL;
    struct scaninfo_entry *se_next = NULL;
    unsigned char ret = 0;

    wnet_vif->vm_connchan.num = 0;
    WIFI_SCAN_SE_LIST_LOCK(st);
    list_for_each_entry_safe(se,se_next,&st->st_entry,se_list)
    {
        AML_PRINT(AML_LOG_ID_CONNECT, AML_LOG_LEVEL_DEBUG, "<running> se:%p st_entry:%p \n",se, &st->st_entry);
        if (match_bss(wnet_vif, ss, se) == 0)
        {
             wnet_vif->vm_scanchan_rssi = se->se_avgrssi - 256;
             wnet_vif->vm_connchan.conn_chan[wnet_vif->vm_connchan.num++] = se->scaninfo.SI_chan;
             WIFINET_ADDR_COPY(wnet_vif->vm_connchan.da, se->scaninfo.SI_macaddr);
             WIFINET_ADDR_COPY(wnet_vif->vm_connchan.bssid, se->scaninfo.SI_bssid);
             list_del_init(&se->se_list);
             list_del_init(&se->se_hash);
             FREE(se,"sta_add.se");
             ret = 1;
        }
    }
    WIFI_SCAN_SE_LIST_UNLOCK(st);
    return ret;
}

static unsigned char
wifi_mac_scan_set_match_node(struct wifi_mac_scan_state *ss, struct wlan_net_vif *wnet_vif)
{
    struct scaninfo_table *st = ss->ScanTablePriv;
    struct scaninfo_entry *se = NULL;
    struct scaninfo_entry *se_next = NULL;
    unsigned char ret = 0;

    wnet_vif->vm_connchan.num = 0;
    WIFI_SCAN_SE_LIST_LOCK(st);
    list_for_each_entry_safe(se,se_next,&st->st_entry,se_list) {
        if (match_bss(wnet_vif, ss, se) != 0) {
             list_del_init(&se->se_list);
             list_del_init(&se->se_hash);
             FREE(se,"sta_add.se");
             ret = 1;
        }
    }

    WIFI_SCAN_SE_LIST_UNLOCK(st);
    return ret;
}

static int
wifi_mac_scan_connect(struct wifi_mac_scan_state *ss, struct wlan_net_vif *wnet_vif)
{
    struct scaninfo_entry *best_node = &wnet_vif->vm_connect_scan_entry;
    struct wifi_channel *chan;
    struct vm_wdev_priv *pwdev_priv = wdev_to_priv(wnet_vif->vm_wdev);
    const unsigned char zero_bssid[WIFINET_ADDR_LEN] = {0};

    /*AP doesn't get ssid and connect, so just return. */
    if (ss->scan_CfgFlags & WIFINET_SCANCFG_NOPICK) {
        return 0;
    }

    if (pwdev_priv->connect_request == NULL) {
        if (wnet_vif->vm_recovery_state != WIFINET_RECOVERY_VIF_UP)
            return 0;
    }

    if (wnet_vif->vm_state > WIFINET_S_SCAN) {
        return 0;
    }

    if (wnet_vif->vm_mainsta && (wnet_vif->vm_mainsta->connect_status != CONNECT_CONNECTING)) {
        AML_PRINT(AML_LOG_ID_CONNECT,AML_LOG_LEVEL_WARN,"connection stop connect_status:%d\n", wnet_vif->vm_mainsta->connect_status);
        return 0;
    }

    KASSERT((wnet_vif->vm_opmode == WIFINET_M_IBSS)
        ||(wnet_vif->vm_opmode == WIFINET_M_STA)
        ||(wnet_vif->vm_opmode == WIFINET_M_P2P_DEV)
        ||(wnet_vif->vm_opmode == WIFINET_M_P2P_CLIENT),
        ("wifi_mac_scan_connect vmopmode error opmode=%d !!",wnet_vif->vm_opmode));

    memset(best_node, 0, sizeof(struct scaninfo_entry));
    /*get a best ap from scan list, by comparing rssi, connect times... */
    if (!wifi_mac_scan_get_best_node(ss, wnet_vif, best_node)) {
        AML_PRINT_LOG_ERR("no bss match, goto notfound\n");
        goto notfound;
    }

    if (wnet_vif->vm_wmac->wm_roaming_state == ROAMING_SCAN) {
        AML_PRINT_LOG_INFO("Roaming better bssid rssi:%d channel:%d bssid:%s ssid: %s\n",
            best_node->scaninfo.SI_rssi - 256, best_node->scaninfo.SI_chan->chan_pri_num,
            ether_sprintf(best_node->scaninfo.SI_bssid), ssidie_sprintf(best_node->scaninfo.SI_ssid));
    }

    if ((wnet_vif->vm_opmode == WIFINET_M_IBSS)
        && (WIFINET_ADDR_EQ(best_node->scaninfo.SI_bssid, &zero_bssid[0])))
    {
        /* get ap work mode. */
        if( WIFINET_IS_CHAN_2GHZ( best_node->scaninfo.SI_chan)  )
        {
            wnet_vif->vm_mac_mode = WIFINET_MODE_11BGN;
        }
        else
        {
            wnet_vif->vm_mac_mode = WIFINET_MODE_11GNAC;
        }

        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "<running> \n");
        wifi_mac_create_wifi(wnet_vif, best_node->scaninfo.SI_chan);
        return 1;

    } else {
        best_node->connectcnt++;
        best_node->ConnectTime = jiffies;
        if (wnet_vif->vm_wmac->wm_roaming_state == ROAMING_SCAN) {
            AML_PRINT_LOG_INFO("Roaming try to join ssid:%s bssid:%s channel:%d\n",
                ssidie_sprintf(best_node->scaninfo.SI_ssid), ether_sprintf(best_node->scaninfo.SI_bssid), best_node->scaninfo.SI_chan->chan_pri_num);
            wnet_vif->vm_wmac->wm_roaming_state = ROAMING_CONNECTING;
        }
        wifi_mac_connect(wnet_vif, &best_node->scaninfo);
        best_node->se_valid = 0;
        return 1;
    }

    AML_PRINT(AML_LOG_ID_CONNECT, AML_LOG_LEVEL_DEBUG, "<running> \n");
    return 1;

notfound:
    //if adhoc mode, not find we must create a new
    if (wnet_vif->vm_opmode == WIFINET_M_IBSS)
    {
         if (wnet_vif->vm_curchan != WIFINET_CHAN_ERR)
            chan = wnet_vif->vm_curchan;
        else
            chan = wifi_mac_get_wm_chan(wnet_vif->vm_wmac);
        wifi_mac_create_wifi(wnet_vif, chan);
    }
    AML_PRINT(AML_LOG_ID_CONNECT, AML_LOG_LEVEL_DEBUG, "entry=NULL, return\n");
    return 0;
}

int wifi_mac_chk_ap_chan(struct wifi_mac_scan_state *ss, struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    int channum;
    struct wifi_channel *c = NULL;

     //AP default work in 5G channel 149,wait hostapd configure after init
    channum = DEFAULT_CHANNEL;
    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG,
        "<%s> vm_opmode %d\n",wnet_vif->vm_ndev->name,wnet_vif->vm_opmode);

    if (wnet_vif->vm_curchan != WIFINET_CHAN_ERR)
        c = wnet_vif->vm_curchan;
    else
        c = wifi_mac_find_chan(wifimac, channum, WIFINET_BWC_WIDTH80, channum + 6);

    if(c == NULL)
    {
        AML_PRINT(AML_LOG_ID_BWC, AML_LOG_LEVEL_ERROR, " error channum=%d,  c=%p\n",channum, c);
        c  = ss->ss_chans[0];
    }

    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "channel=%d  chan_flags = 0x%x\n",
        channum, c->chan_flags);

    if (WIFINET_IS_CHAN_2GHZ(c)) {
        if (wifi_mac_scan_chk_11g_bss(ss, wnet_vif)) {
            wnet_vif->vm_htcap &= ~WIFINET_HTCAP_SUPPORTCBW40;
            wnet_vif->scnd_chn_offset = WIFINET_HTINFO_EXTOFFSET_NA;
            wnet_vif->vm_bandwidth = WIFINET_BWC_WIDTH20;
            c = wifi_mac_find_chan(wifimac, c->chan_pri_num, WIFINET_BWC_WIDTH20, c->chan_pri_num);
        }
    }

    wifi_mac_create_wifi(wnet_vif, c);
    return 0;
}

void get_ovlapping_chan_index(struct wifi_mac *wifimac, unsigned char center_chan, unsigned char bw,
    unsigned char *low, unsigned char *up) {
    unsigned char step = 0;
    unsigned char is_2g = 0;
    unsigned char i = 0;

    if ((0 < center_chan) && (center_chan < 15)) {
        step = 1;
        is_2g = 1;

    } else {
        step = 2;
    }

    for (i = 0; i < WIFINET_MAX_SCAN_CHAN; ++i) {
        if (wifimac->chan_overlapping_map[i].chan_index == center_chan) {
            break;
        }
    }

    if (i < WIFINET_MAX_SCAN_CHAN) {
        if (bw == WIFINET_BWC_WIDTH20) {
            step = 2 / step;

        } else if (bw == WIFINET_BWC_WIDTH40) {
            step = 4 / step;

        } else if (bw == WIFINET_BWC_WIDTH80) {
            step = 8 / step;
        }

        if (i < step) {
            *low = 0;
            *up = i + step;

        } else if (i + step > 63) {
            *low = i - step;
            *up = 63;

        } else {
            if (is_2g) {
                if (i + step > 14) {
                    *low = i - step;
                    *up = 13;

                } else {
                    *low = i - step;
                    *up = i + step;
                }

            } else {
                *low = i - step;
                *up = i + step;

                if (*low < 14) {
                    *low = 14;

                } else if ((*low < 30) && (i >= 30)) {
                    *low = 29;

                } else if ((*low < 53) && (i >= 53)) {
                    *low = 53;
                }

                if ((*up > 30) && (i <=30)) {
                    *up = 30;

                } else if ((*up > 53) && (i <= 53)) {
                    *up = 53;
                }
            }
        }

    } else {
        AML_PRINT_LOG_ERR("can't find center_chan:%d\n", center_chan);
    }

}

void wifi_mac_update_chan_overlapping_map(struct wlan_net_vif *wnet_vif) {
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;
    struct scaninfo_table *st = ss->ScanTablePriv;
    struct scaninfo_entry *se = NULL;
    struct scaninfo_entry *se_next = NULL;
    struct wifi_scan_info *lse = NULL;
    unsigned char bw = 0;
    unsigned char low_chan = 0;
    unsigned char up_chan = 0;
    unsigned char center_chan = 0;
    unsigned char i = 0;

    for (i = 0; i < WIFINET_MAX_SCAN_CHAN; ++i) {
        wifimac->chan_overlapping_map[i].overlapping = 0;
    }

    WIFI_SCAN_SE_LIST_LOCK(st);
    list_for_each_entry_safe(se, se_next, &st->st_entry, se_list) {
        lse = &se->scaninfo;
        center_chan = wifi_mac_Mhz2ieee(lse->SI_chan->chan_cfreq1, 0);
        bw = lse->SI_chan->chan_bw;

        get_ovlapping_chan_index(wifimac, center_chan, bw, &low_chan, &up_chan);
        for (i = low_chan; i <= up_chan; ++i) {
            wifimac->chan_overlapping_map[i].overlapping++;
        }

        //AML_PRINT_LOG_INFO("ssid:%s, center_chan:%d, bw:%d, low_chan:%d, up_chan:%d\n",
        //    lse->SI_ssid + 2, center_chan, bw, low_chan, up_chan);
    }
    WIFI_SCAN_SE_LIST_UNLOCK(st);

    //for (i = 0; i < WIFINET_MAX_SCAN_CHAN; ++i) {
    //    AML_PRINT_LOG_INFO("chan_index:%d, overlapping:%d\n",
    //        wifimac->chan_overlapping_map[i].chan_index, wifimac->chan_overlapping_map[i].overlapping);
    //}
}

void is_connect_need_set_gain(struct wlan_net_vif *wnet_vif) {
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    unsigned char overlapping_max = 0;
    unsigned char i = 0;
    unsigned char bw = wnet_vif->vm_curchan->chan_bw;
    unsigned char low_chan = 0;
    unsigned char up_chan = 0;
    unsigned char is_2g = 0;
    unsigned char center_chan = wifi_mac_Mhz2ieee(wnet_vif->vm_curchan->chan_cfreq1, 0);

    if ((0 < center_chan) && (center_chan < 15)) {
        is_2g = 1;
    }

    get_ovlapping_chan_index(wifimac, center_chan, bw, &low_chan, &up_chan);
    for (i = low_chan; i <= up_chan; ++i) {
        if (overlapping_max < wifimac->chan_overlapping_map[i].overlapping) {
            overlapping_max = wifimac->chan_overlapping_map[i].overlapping;
        }
    }

    if (is_2g && (overlapping_max > OVERLAPPING_24G_GIAN_THRESHOLD)) {
        wifimac->is_connect_set_gain = 1;

    } else if (!is_2g && (overlapping_max > OVERLAPPING_5G_GIAN_THRESHOLD)) {
        wifimac->is_connect_set_gain = 1;

    } else {
        wifimac->is_connect_set_gain = 0;
    }

    AML_PRINT_LOG_INFO("overlapping:%d, is_connect_set_gain:%d, low_chan:%d, up_chan:%d, bw:%d, center:%d\n",
        overlapping_max, wifimac->is_connect_set_gain, low_chan, up_chan, bw, center_chan);
}


int wifi_mac_scan_parse(struct wlan_net_vif *wnet_vif, wifi_mac_ScanIterFunc *f, void *arg)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;
    struct scaninfo_table *st = ss->ScanTablePriv;
    struct scaninfo_entry *se = NULL, *SI_next = NULL;
    unsigned char count = 0;

    WIFI_SCAN_SE_LIST_LOCK(st);
    list_for_each_entry_safe(se,SI_next,&st->st_entry,se_list)
    {
        se->scaninfo.SI_age = jiffies - se->LastUpdateTime;
        (*f)(arg, &se->scaninfo);
        count++;
        if (match_ssid(se->scaninfo.SI_ssid,  ss->ss_nssid, ss->ss_ssid))
        {
            AML_PRINT_LOG_INFO("scan find target ap ssid:%s bssid:%s\n", ssidie_sprintf(se->scaninfo.SI_ssid), ether_sprintf(se->scaninfo.SI_bssid));
        }
    }
    WIFI_SCAN_SE_LIST_UNLOCK(st);

    /* if scan all channels */
    if (!wnet_vif->vm_chan_switch_scan_flag && !wnet_vif->vm_scan_before_connect_flag
        && !wnet_vif->vm_chan_roaming_scan_flag && ss->scan_next_chan_index > 10) {
        wifi_mac_update_chan_overlapping_map(wnet_vif);
        ss->scan_ssid_count = count;

        if (wifi_mac_is_in_noisy_environment(wifimac)) {
            wifimac->scan_noisy_status = WIFINET_S_SCAN_ENV_NOISE;

        } else if (wifi_mac_is_in_clear_environment(wifimac)) {
            //clear environment, set to max gain
            wifimac->drv_priv->drv_ops.set_channel_rssi(wifimac->drv_priv, 174, 1);
            wifimac->scan_noisy_status = WIFINET_S_SCAN_ENV_CLEAR;

        } else {
            wifimac->scan_noisy_status = WIFINET_S_SCAN_ENV_MID;
        }
    }

    AML_PRINT_LOG_INFO("scan_res:%d, chan_count:%d, thresh_unconnect:%d, thresh_connect:%d, scan_noisy_status:%d\n",
        count, ss->scan_next_chan_index, wifimac->scan_gain_thresh_unconnect,
        wifimac->scan_gain_thresh_connect, wifimac->scan_noisy_status);
    return 0;
}

void wifi_mac_scan_flush(struct wifi_mac *wifimac)
{
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;
    struct scaninfo_table *st = ss->ScanTablePriv;
    struct scaninfo_entry *se=NULL, *next=NULL;

    AML_PRINT(AML_LOG_ID_SCAN,AML_LOG_LEVEL_INFO, "<running>  \n");

    WIFI_SCAN_SE_LIST_LOCK(st);
    list_for_each_entry_safe(se,next,&st->st_entry,se_list)
    {
        list_del_init(&se->se_list);
        list_del_init(&se->se_hash);
        FREE(se,"sta_add.se");
    }
    WIFI_SCAN_SE_LIST_UNLOCK(st);
}

void update_roaming_candidate_chan(struct wifi_mac_scan_state *ss, struct wifi_channel *apchan, int rssi)
{
    int i =0;
    struct wifi_candidate_channel * worst_chan = NULL;

    WIFI_ROAMING_CHANNEL_LOCK(ss);
    for (i = 0; i < ROAMING_CANDIDATE_CHAN_MAX; i++) {
        if (ss->roaming_candidate_chans[i].channel) {
            /*update rssi */
            if (ss->roaming_candidate_chans[i].channel->chan_pri_num == apchan->chan_pri_num) {
                ss->roaming_candidate_chans[i].avg_rssi = (ss->roaming_candidate_chans[i].avg_rssi + (rssi * 3)) >> 2;
                WIFI_ROAMING_CHANNEL_UNLOCK(ss);
                return ;
           }

            /*get worst channel*/
            if (!worst_chan) {
                worst_chan = &ss->roaming_candidate_chans[i];
            }
            else if (worst_chan->avg_rssi > ss->roaming_candidate_chans[i].avg_rssi) {
                worst_chan = &ss->roaming_candidate_chans[i];
            }
        }
    }

    if (ss->roaming_candidate_chans_cnt > 5) {
        if (rssi > worst_chan->avg_rssi) {
            worst_chan->channel = apchan;
            worst_chan->avg_rssi = rssi;
        }
    }
    else {
        ss->roaming_candidate_chans[ss->roaming_candidate_chans_cnt].channel = apchan;
        ss->roaming_candidate_chans[ss->roaming_candidate_chans_cnt].avg_rssi = rssi;
        ss->roaming_candidate_chans_cnt++;
    }
    WIFI_ROAMING_CHANNEL_UNLOCK(ss);

}

void wifi_mac_update_roaming_candidate_chan(struct wlan_net_vif *wnet_vif,const struct wifi_mac_scan_param *sp, int rssi)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;
    int nssid = sp->ssid[1];
    static struct wifi_channel * apchan = NULL;

    apchan= wifi_mac_scan_sta_get_ap_channel(wnet_vif, (struct wifi_mac_scan_param *)sp);
    if (apchan == NULL) {
         AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_DEBUG, " apchan = %p\n",apchan);
        return;
    }

    if ((sp->ssid[2] != 0) && ss->ss_ssid->len && nssid
        && !(memcmp(sp->ssid+2, ss->roaming_ssid.ssid, nssid))) {
        update_roaming_candidate_chan(ss, apchan, rssi);
//        AML_PRINT_LOG_INFO("\n\n[Roaming ssid:%s] len:%d chan:%d\n\n", ssidie_sprintf(sp->ssid), nssid, apchan->chan_pri_num);
    }

}

void wifi_mac_scan_rx(struct wlan_net_vif *wnet_vif, const struct wifi_mac_scan_param *sp,
    const struct wifi_frame *wh, int rssi, struct scaninfo_entry *oldse)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;
    struct scaninfo_table *st = ss->ScanTablePriv;
    const unsigned char *macaddr = wh->i_addr2;
    struct scaninfo_entry *se = NULL;
    struct wifi_scan_info *ise;
    static struct wifi_channel *apchan =NULL;
    int hash;
    int index = 0;
    struct wifi_mac_Rsnparms rsn = {0};
    int err;

    if (ss->scan_StateFlags & SCANSTATE_F_DISCARD) {
        if (wnet_vif->vm_mainsta->sta_bcn_num_connected) {
            wnet_vif->vm_mainsta->sta_bcn_num_connected = 0;
        }
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG,"drop \n");
        return;
    }

    if (sp->ssid) {
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "<running> %d %s chan %d \n", sp->ssid[1], ssidie_sprintf(sp->ssid), sp->chan);
    }

    apchan = wifi_mac_scan_sta_get_ap_channel(wnet_vif, (struct wifi_mac_scan_param *)sp);
    if (apchan == NULL) {
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "ssid %s ,apchan = %p\n", ssidie_sprintf(sp->ssid), apchan);
        return;
    }

    if (oldse != NULL) {
        se = oldse;
    } else {
        se = (struct scaninfo_entry *)NET_MALLOC(sizeof(struct scaninfo_entry), GFP_ATOMIC, "sta_add.se");
    }

    if (se == NULL) {
        return ;
    }

    ise = &se->scaninfo;
    ise->SI_frame_len = sp->frame_len;
    if (sp->rsn != NULL) {
        saveie(&ise->SI_rsn_ie[0], sp->rsn);

        err = wifi_mac_parse_counterpart_rsn(&rsn, ise->SI_rsn_ie, 0);
        if (err != 0) {
            goto fail;

        } else {
            ise->si_rsn_capa = rsn.rsn_caps;
        }
    }

    if (sp->rsnxe != NULL) {
        saveie(&ise->SI_rsnx_ie[0], sp->rsnxe);
    }

    WIFINET_ADDR_COPY(se->scaninfo.SI_macaddr, macaddr);
    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "<running> se %p se->se_list 0x%p, %s, mac:%02x:%02x:%02x:%02x:%02x:%02x\n",
        se, &se->se_list, ssidie_sprintf(sp->ssid), macaddr[0], macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]);

    if (sp->ssid && (sp->ssid[1] != 0) && (WIFINET_IS_PROBERSP(wh) || ise->SI_ssid[1] == 0)) {
       memcpy(ise->SI_ssid, sp->ssid, 2 + sp->ssid[1]);
    }
    else if (wnet_vif->vm_state == WIFINET_S_CONNECTING || wnet_vif->vm_state == WIFINET_S_SCAN){
       if (memcmp(se->scaninfo.SI_macaddr, wnet_vif->vm_des_bssid, ETH_ALEN) == 0) {
           AML_PRINT_LOG_INFO("drop hidden ssid AP beacon fc:0x%x\n", *(unsigned short *)wh->i_fc);
           goto fail;
       }
    }


    if (sp->rates) {
        KASSERT(sp->rates[1] <= WIFINET_RATE_MAXSIZE, ("rate set too large: %u", sp->rates[1]));
        memcpy(ise->SI_rates, sp->rates, 2 + sp->rates[1]);
    }

    if (sp->xrates != NULL) {
        KASSERT(sp->xrates[1] <= WIFINET_RATE_MAXSIZE, ("xrate set too large: %u", sp->xrates[1]));
        memcpy(ise->SI_exrates, sp->xrates, 2+sp->xrates[1]);

    } else {
        ise->SI_exrates[1] = 0;
    }

    WIFINET_ADDR_COPY(ise->SI_bssid, wh->i_addr3);
    if (se->LastUpdateTime == 0)
        se->se_avgrssi = rssi;
    else
        se->se_avgrssi = (se->se_avgrssi + rssi*3)>>2;
    se->scaninfo.SI_rssi = se->se_avgrssi;
    memcpy(ise->SI_tstamp.data, sp->tstamp, sizeof(ise->SI_tstamp));

    ise->SI_intval = sp->bintval;
    ise->SI_capinfo = sp->capinfo;
    ise->SI_chan = apchan;
    ise->SI_erp = sp->erp;
    ise->SI_timoff = sp->timoff;
    if (sp->tim != NULL) {
        ise->SI_dtimperiod =  ((const struct wifi_mac_tim_ie *) sp->tim)->tim_period;
    }

#ifdef CONFIG_WAPI
    saveie(&ise->SI_wai_ie[0], sp->wai);
#endif //#ifdef CONFIG_WAPI

#ifdef CONFIG_P2P
    for (index = 0; index < MAX_P2PIE_NUM; index++)
    {
        if (sp->p2p[index] != NULL) {
            saveie(&ise->SI_p2p_ie[index][0], sp->p2p[index]);
        }
    }
#endif //#ifdef CONFIG_P2P

#ifdef CONFIG_WFD
    saveie(&ise->SI_wfd_ie[0], sp->wfd);
#endif //#ifdef CONFIG_WFD

    saveie(&ise->SI_wme_ie[0], sp->wme);
    saveie(&ise->SI_htcap_ie[0], sp->htcap);
    saveie(&ise->SI_htinfo_ie[0], sp->htinfo);
    saveie(&ise->SI_country_ie[0], sp->country);
    saveie(&ise->SI_wpa_ie[0], sp->wpa);
    if (sp->wps != NULL) {
        saveie(&ise->SI_wps_ie[0], sp->wps);
    }

    if (se->connectcnt && ((jiffies - se->ConnectTime) > WIFINET_CONNECT_CNT_AGE*HZ)) {
        se->connectcnt = 0;
        WIFINET_DPRINTF_MACADDR( AML_LOG_ID_CONNECT, AML_LOG_LEVEL_DEBUG, macaddr,
            "fails %u HZ %d 0x%lx", se->connectcnt,HZ,jiffies);
    }

    se->LastUpdateTime = jiffies;
    se->se_valid = 1;

    saveie(ise->ie_vht_cap, sp->vht_cap);
    saveie(ise->ie_vht_opt, sp->vht_opt);
    saveie(ise->ie_vht_tx_pwr, sp->vht_tx_pwr);
    saveie(ise->ie_vht_ch_sw, sp->vht_ch_sw);
    saveie(ise->ie_vht_ext_bss_ld, sp->vht_ext_bss_ld);
    saveie(ise->ie_vht_quiet_ch, sp->vht_quiet_ch);
    saveie(ise->ie_vht_opt_md_ntf, sp->vht_opt_md_ntf);

    hash = STA_HASH(macaddr);

    WIFI_SCAN_SE_LIST_LOCK(st);
    if (oldse == NULL) {
        list_add(&se->se_hash, &st->st_hash[hash]);
        list_add_tail(&se->se_list, &st->st_entry);
    }
    WIFI_SCAN_SE_LIST_UNLOCK(st);

    if ((ss->scan_CfgFlags & WIFINET_SCANCFG_CONNECT)
        && (memcmp(se->scaninfo.SI_macaddr, wnet_vif->vm_des_bssid, ETH_ALEN) == 0)) {
        ss->scan_StateFlags |= SCANSTATE_F_GET_CONNECT_AP;
    }

    return;

fail:
    if (oldse == NULL) {
        FREE(se,"sta_add.se");
    }
    return;
}

static void quiet_intf (struct wlan_net_vif *wnet_vif, unsigned char enable)
{
    unsigned int qlen_real = WIFINET_SAVEQ_QLEN(&wnet_vif->vm_tx_buffer_queue);
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;

    if (wnet_vif->vm_opmode == WIFINET_M_STA) {
        if (wnet_vif->vm_state == WIFINET_S_CONNECTED) {
            AML_PRINT(AML_LOG_ID_PWR_SAVE, AML_LOG_LEVEL_DEBUG, " vid:%d, enable:%d, qlen_real:%d\n", wnet_vif->wnet_vif_id, enable, qlen_real);

            if (enable) {
                wnet_vif->vm_pstxqueue_flags |= WIFINET_PSQUEUE_PS4QUIET;
                wifimac->drv_priv->drv_ops.drv_set_is_mother_channel(wifimac->drv_priv, wnet_vif->wnet_vif_id, 0);
                wifi_mac_pwrsave_send_nulldata(wnet_vif->vm_mainsta, NULLDATA_PS, 1);

            } else {
                wnet_vif->vm_pstxqueue_flags &= ~WIFINET_PSQUEUE_PS4QUIET;

                if (qlen_real == 0) {
                    wifi_mac_pwrsave_send_nulldata(wnet_vif->vm_mainsta, NULLDATA_NONPS, 0);

                } else {
                    wifi_mac_buffer_txq_send_pre(wnet_vif);
                }
            }

        } else {
            wnet_vif->vm_pstxqueue_flags &= ~WIFINET_PSQUEUE_PS4QUIET;
        }
   }
   else if (wnet_vif->vm_opmode == WIFINET_M_HOSTAP) {
       if (enable) {
           wnet_vif->vm_pstxqueue_flags |= WIFINET_PSQUEUE_PS4QUIET;
           wifimac->drv_priv->drv_ops.drv_set_is_mother_channel(wifimac->drv_priv, wnet_vif->wnet_vif_id, 0);
       } else {
           wnet_vif->vm_pstxqueue_flags &= ~WIFINET_PSQUEUE_PS4QUIET;
           if (qlen_real == 0) {
                 //do nothing
           } else {
               wifi_mac_buffer_txq_send_pre(wnet_vif);
           }
       }
  }
}

void quiet_all_intf (struct wifi_mac *wifimac, unsigned char enable)
{
    struct wlan_net_vif *tmpwnet_vif = NULL, *tmpwnet_vif_next = NULL;

    list_for_each_entry_safe(tmpwnet_vif,tmpwnet_vif_next, &wifimac->wm_wnet_vifs, vm_next)
    {
        quiet_intf(tmpwnet_vif, enable);
    }
}

void wifi_mac_scan_notify_leave_or_back(struct wlan_net_vif *wnet_vif, unsigned char enable) {
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;

    if ((wifimac->wm_nrunning == 1)
        || ((wifimac->wm_nrunning == 2) && concurrent_check_is_vmac_same_pri_channel(wifimac))) {
        quiet_all_intf(wifimac, enable);

    } else if (wifimac->wm_nrunning == 2) {
        quiet_intf(wnet_vif, enable);
    }
}

void wifi_mac_set_scan_time(struct wlan_net_vif *wnet_vif) {
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;

    if (wnet_vif->vm_scan_duration_mandatory != 0 && wnet_vif->vm_scan_duration != 0) {
        ss->scan_chan_wait = wnet_vif->vm_scan_duration;
        ss->scan_passive_chan_wait = wnet_vif->vm_scan_duration;
        return;
    }

    if (wnet_vif->vm_chan_switch_scan_flag) {
        ss->scan_chan_wait = wnet_vif->vm_scan_time_chan_switch;
    } else {
        if (wnet_vif->vm_scan_before_connect_flag) {
            ss->scan_chan_wait = wnet_vif->vm_scan_time_before_connect;
        } else if (wifimac->wm_nrunning && !(wifimac->wm_nrunning == 1 && IS_APSTA_CONCURRENT(aml_wifi_get_con_mode()))) {
            ss->scan_chan_wait = wnet_vif->vm_scan_time_connect;
        } else {
            ss->scan_chan_wait = wnet_vif->vm_scan_time_idle;
        }
    }

    ss->scan_passive_chan_wait = WIFINET_SCAN_DEFAULT_INTERVAL;

    if (wnet_vif->vm_scan_duration != 0) {
        ss->scan_chan_wait = (ss->scan_chan_wait > wnet_vif->vm_scan_duration ? wnet_vif->vm_scan_duration : ss->scan_chan_wait);
        ss->scan_passive_chan_wait = (ss->scan_passive_chan_wait > wnet_vif->vm_scan_duration ? wnet_vif->vm_scan_duration : ss->scan_passive_chan_wait);
    }
    //AML_PRINT_LOG_INFO("change scan time to:%d, passive scan time:%d\n", ss->scan_chan_wait, ss->scan_passive_chan_wait);
    return;
}

static int vm_scan_setup_chan(struct wifi_mac_scan_state *ss, struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_channel *c;
    static unsigned char chan_aware_cnt = 0;
    int i = 0;
    unsigned char roaming_candidate_channel_cnt = 0;

#ifdef CONFIG_P2P
    struct wlan_net_vif *tmpwnet_vif;
    tmpwnet_vif = wifi_mac_get_wnet_vif_by_vid(wifimac, NET80211_P2P_VMAC);
#endif

    ss->scan_last_chan_index = 0;
    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "wifimac->wm_nchans=%d \n", wifimac->wm_nchans);

    if (wnet_vif->vm_chan_simulate_scan_flag) {
        AML_PRINT_LOG_ERR("scan for tx_error\n");

        WIFI_CHANNEL_LOCK(wifimac);
        for (i = 0; i < wifimac->wm_nchans; i++) {
            c = &wifimac->wm_channels[i];
            if (chan_aware_cnt >= 10) {
                c->chan_flags &= ~WIFINET_CHAN_AWARE;
                chan_aware_cnt = 0;
            }

            /* both 2.4G and 5G set WIFINET_BW_20 flag.*/
            if ((c->chan_bw == WIFINET_BWC_WIDTH20) && (wifimac->wm_curchan != c)) {
                if ((aml_iwpriv_get_band() == CFG_BAND_B) && (!WIFINET_IS_CHAN_2GHZ(c))) {
                    continue;
                } else if ((aml_iwpriv_get_band() == CFG_BAND_A) && (!WIFINET_IS_CHAN_5GHZ(c))) {
                    continue;
                }

                ss->ss_chans[ss->scan_last_chan_index++] = c;
                break;
            }
        }
        WIFI_CHANNEL_UNLOCK(wifimac);

        wnet_vif->vm_chan_simulate_scan_flag = 0;

    } else if (wnet_vif->vm_scan_before_connect_flag) {
        ss->scan_last_chan_index = wnet_vif->vm_connchan.num * 3;

        for (i = 0; i <  wnet_vif->vm_connchan.num; i++) {
            ss->ss_chans[i] = wnet_vif->vm_connchan.conn_chan[i];
            ss->ss_chans[i + wnet_vif->vm_connchan.num] = wnet_vif->vm_connchan.conn_chan[i];
            ss->ss_chans[i + (wnet_vif->vm_connchan.num * 2)] = wnet_vif->vm_connchan.conn_chan[i];
            AML_PRINT_LOG_INFO("add scan connect chans:%d\n", wnet_vif->vm_connchan.conn_chan[i]->chan_pri_num);
        }

    } else if (wnet_vif->vm_chan_roaming_scan_flag && !wnet_vif->vm_wmac->wm_scan->roaming_full_scan) {
        WIFI_ROAMING_CHANNEL_LOCK(ss);
        for (i = 0; i < wnet_vif->vm_wmac->wm_scan->roaming_candidate_chans_cnt; i++) {
            if (wnet_vif->vm_wmac->wm_scan->roaming_candidate_chans[i].channel->chan_cfreq1 != 0) {
                ss->ss_chans[roaming_candidate_channel_cnt++] = wnet_vif->vm_wmac->wm_scan->roaming_candidate_chans[i].channel;
            } else {
                AML_PRINT_LOG_ERR("error, roaming_candidate_chans center freq is zero!\n");
            }
        }
        AML_PRINT_LOG_INFO("scan roaming_candidate_chans, expect_num:%d, real_num:%d\n",
            wnet_vif->vm_wmac->wm_scan->roaming_candidate_chans_cnt, roaming_candidate_channel_cnt);
        ss->scan_last_chan_index = roaming_candidate_channel_cnt;
        WIFI_ROAMING_CHANNEL_UNLOCK(ss);
    } else if (wnet_vif->vm_chan_switch_scan_flag) {
        ss->scan_last_chan_index = 1;
        ss->ss_chans[0] = wnet_vif->vm_switchchan;

    } else {
        AML_PRINT_LOG_INFO("scan all chans \n");

        /*clean roamin candidate channel*/
        WIFI_ROAMING_CHANNEL_LOCK(wnet_vif->vm_wmac->wm_scan);
        wnet_vif->vm_wmac->wm_scan->roaming_candidate_chans_cnt = 0;
        memset(wnet_vif->vm_wmac->wm_scan->roaming_candidate_chans, 0, sizeof(wnet_vif->vm_wmac->wm_scan->roaming_candidate_chans));
        WIFI_ROAMING_CHANNEL_UNLOCK(wnet_vif->vm_wmac->wm_scan);

        WIFI_CHANNEL_LOCK(wifimac);
        for (i = 0; i < wifimac->wm_nchans; i++)
        {
            c = &wifimac->wm_channels[i];
            if (chan_aware_cnt >= 10) {
                c->chan_flags &= ~WIFINET_CHAN_AWARE;
                chan_aware_cnt = 0;
            }

            /* both 2.4G and 5G set WIFINET_BW_20 flag.*/
            if (c->chan_bw == WIFINET_BWC_WIDTH20)
            {
    #ifdef CONFIG_P2P
                if ((wnet_vif->vm_p2p != NULL) && (wnet_vif->vm_p2p->p2p_enable)
                    && (wnet_vif->vm_p2p->social_channel))
                {
                    if ((c->chan_cfreq1 != SOCIAL_CHAN_1)
                       && (c->chan_cfreq1 != SOCIAL_CHAN_2)
                       && (c->chan_cfreq1 != SOCIAL_CHAN_3))
                    {
                        continue;
                    }

                    if ((aml_iwpriv_get_band() == CFG_BAND_B) && (!WIFINET_IS_CHAN_2GHZ(c))) {
                        continue;
                    } else if ((aml_iwpriv_get_band() == CFG_BAND_A) && (!WIFINET_IS_CHAN_5GHZ(c))) {
                        continue;
                    }

                    ss->ss_chans[ss->scan_last_chan_index++] = c;
                }
                else
    #endif//CONFIG_P2P
                {
                    if ((aml_iwpriv_get_band() == CFG_BAND_B) && (!WIFINET_IS_CHAN_2GHZ(c))) {
                        continue;
                    } else if ((aml_iwpriv_get_band() == CFG_BAND_A) && (!WIFINET_IS_CHAN_5GHZ(c))) {
                        continue;
                    } else if (!g_DFS_on && c->chan_flags & WIFINET_CHAN_DFS) {
                        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "skip the DFS channel %d!\n",c->chan_pri_num);
                        continue;
                    }

                    ss->ss_chans[ss->scan_last_chan_index++] = c;
                    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG,"chan_index=%d,chan_pri_num=%d,flag=%x,freq %d %p\n",
                        i,c->chan_pri_num,c->chan_flags,c->chan_cfreq1,c);
                }
            }
        }
        WIFI_CHANNEL_UNLOCK(wifimac);
    }

    chan_aware_cnt++;
    wifi_mac_set_scan_time(wnet_vif);
    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG,"vid:%d ss->scan_next_chan_index=%d \
        ss->scan_last_chan_index=%d, ss->scan_chan_wait=0x%xms, HZ = %d LINUX_VERSION_CODE =%x\n",
        wnet_vif->wnet_vif_id, ss->scan_next_chan_index, ss->scan_last_chan_index,
        ss->scan_chan_wait, HZ, LINUX_VERSION_CODE);

    return 0;
}

#ifdef FW_RF_CALIBRATION
static void
wifi_mac_scan_send_probe_timeout(SYS_TYPE param1,SYS_TYPE param2,
    SYS_TYPE param3,SYS_TYPE param4,SYS_TYPE param5)
{
    struct wifi_mac_scan_state *ss = (struct wifi_mac_scan_state *)param1;
    struct wlan_net_vif *wnet_vif = ss->VMacPriv;
    unsigned char i;
    unsigned char j;

    os_timer_ex_cancel(&ss->ss_probe_timer, CANCEL_SLEEP);
    //AML_PRINT_LOG_INFO("ss->scan_StateFlags:%08x, ss->ss_nssid:%d\n", ss->scan_StateFlags, ss->ss_nssid);

    if (ss->scan_CfgFlags & WIFINET_SCANCFG_ACTIVE)
    {
        struct net_device *dev = wnet_vif->vm_ndev;

        for (i = 0; i < ss->ss_nssid; i++)
            for (j = 0; j < 1; ++j)
                wifi_mac_send_probereq(wnet_vif->vm_mainsta, wnet_vif->vm_myaddr, dev->broadcast,
                    dev->broadcast, ss->ss_ssid[i].ssid, ss->ss_ssid[i].len, wnet_vif->vm_opt_ie, wnet_vif->vm_opt_ie_len);

        if (!wnet_vif->vm_p2p->p2p_enable || (wnet_vif->vm_p2p->p2p_enable && ss->ss_nssid == 0)) {
            wifi_mac_send_probereq(wnet_vif->vm_mainsta, wnet_vif->vm_myaddr, dev->broadcast,
                dev->broadcast, "", 0, wnet_vif->vm_opt_ie, wnet_vif->vm_opt_ie_len);
        }
    }
}

int wifi_mac_scan_send_probe_timeout_ex(void *arg)
{
    struct wifi_mac_scan_state *ss = (struct wifi_mac_scan_state *) arg;
    struct wlan_net_vif *wnet_vif = ss->VMacPriv;
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;

    //AML_PRINT_LOG_INFO("ss->scan_StateFlags:%08x\n", ss->scan_StateFlags);

    WIFI_SCAN_LOCK(ss);
    if (ss->scan_StateFlags & SCANSTATE_F_SEND_PROBEREQ_AGAIN) {
        ss->scan_StateFlags &= ~SCANSTATE_F_SEND_PROBEREQ_AGAIN;
        wifi_mac_add_work_task(wifimac, wifi_mac_scan_send_probe_timeout, NULL,
            (SYS_TYPE)arg, (SYS_TYPE)wnet_vif, 0, 0, 0);

        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "ss->scan_next_chan_index = %d, ss->scan_StateFlags:%08x\n",
            ss->scan_next_chan_index, ss->scan_StateFlags);
    }
    WIFI_SCAN_UNLOCK(ss);

    return OS_TIMER_NOT_REARMED;
}


static enum hrtimer_restart
wifi_mac_scan_chk_leakap_done_process(struct hrtimer *timer)
{
    static unsigned char count = 0;
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();

    if (wifimac->wm_scan->scan_StateFlags & SCANSTATE_F_RX_LEAKAP_HAPPEN) {
        wifimac->wm_scan->scan_StateFlags &= ~SCANSTATE_F_RX_LEAKAP_HAPPEN;

        count ++;
        if (count < 5) {
            wifimac->wm_scan->scan_StateFlags |= SCANSTATE_F_RX_CHKING_LEAKAP_PKT;
            wifimac->wm_scan->scan_kt = ktime_set(0, 2000000);
            hrtimer_forward_now(&wifimac->wm_scan->scan_hr_timer, wifimac->wm_scan->scan_kt);
            return HRTIMER_RESTART;
        }
    }

    if (count != 0) {
        AML_PRINT_LOG_INFO("retry %d\n", count);
    }
    AML_PRINT(AML_LOG_ID_SCAN,AML_LOG_LEVEL_DEBUG, "retry %d\n", count);
    wifimac->wm_scan->scan_StateFlags &= ~SCANSTATE_F_RX_CHKING_LEAKAP_PKT;
    /*ss_scan_timer timeout and null notify happen at the same time, need check SCANSTATE_F_NOTIFY_AP again.*/
    if (wifimac->wm_scan->scan_StateFlags & SCANSTATE_F_NOTIFY_AP) {
        os_timer_ex_cancel(&wifimac->wm_scan->ss_scan_timer, 1);
        wifi_mac_scan_timeout_ex(wifimac->wm_scan);
    }
    count = 0;
    return HRTIMER_NORESTART;
}


void wifi_mac_scan_chk_leakap_hrtimer_attach(struct wifi_mac *wifimac)
{
    hrtimer_init(&wifimac->wm_scan->scan_hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    wifimac->wm_scan->scan_hr_timer.function = wifi_mac_scan_chk_leakap_done_process;
}


void wifi_mac_scan_chk_leakap_hrtimer_start(struct wifi_mac *wifimac)
{
    /* For some platform using backport like ROKU, hrtimer_start is differently defined */
#ifdef BACKPORTS_VERSION
    hrtimer_start(&wifimac->wm_scan->scan_hr_timer, 2000000, HRTIMER_MODE_REL);
#else
    wifimac->wm_scan->scan_kt = ktime_set(0, 2000000);
    hrtimer_start(&wifimac->wm_scan->scan_hr_timer, wifimac->wm_scan->scan_kt, HRTIMER_MODE_REL);
#endif
}

void wifi_mac_scan_chk_leakap_hrtimer_cancel(struct wifi_mac *wifimac)
{
    hrtimer_cancel(&wifimac->wm_scan->scan_hr_timer);
}

void wifi_mac_scan_chking_leakap(void * station, struct wifi_frame *wh)
{
        struct wifi_station *sta = (struct wifi_station *)station;
        struct wifi_mac *wifimac = sta->sta_wmac;
        struct wifi_mac_scan_state *ss = wifimac->wm_scan;

        if ((ss->scan_StateFlags & SCANSTATE_F_START) && (ss->scan_StateFlags & SCANSTATE_F_NOTIFY_AP)
            && (ss->scan_StateFlags & SCANSTATE_F_RX_CHKING_LEAKAP_PKT)) {
            if (WIFINET_ADDR_EQ(wh->i_addr1, sta->sta_wnet_vif->vm_myaddr)) {
                ss->scan_StateFlags |= SCANSTATE_F_RX_LEAKAP_HAPPEN;
            }
        }
}


void wifi_mac_scan_channel(struct wifi_mac *wifimac)
{
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;
    struct wlan_net_vif *wnet_vif = ss->VMacPriv;
    unsigned char i = 0;
    struct wifi_channel *chan;
    enum wifi_mac_macmode last_mac_mode;
    unsigned short dwell_time = 0;

    //AML_PRINT_LOG_INFO("ss->scan_StateFlags:%08x\n", ss->scan_StateFlags);
    if (!(ss->scan_StateFlags & SCANSTATE_F_CHANNEL_SWITCH_COMPLETE)) {
        return;
    }
    ss->scan_StateFlags &= ~SCANSTATE_F_WAIT_CHANNEL_SWITCH;
    ss->scan_StateFlags &= ~SCANSTATE_F_CHANNEL_SWITCH_COMPLETE;
    ss->scan_StateFlags &= ~SCANSTATE_F_DISCARD;

    chan = ss->ss_chans[ss->scan_next_chan_index++];

    if (!((wifimac->wm_nrunning == 2) && (!concurrent_check_is_vmac_same_pri_channel(wifimac)))) {
        if (chan->chan_flags & WIFINET_CHAN_DFS) {
            dwell_time = ss->scan_passive_chan_wait;
        } else {
            dwell_time = ss->scan_chan_wait;
        }
        os_timer_ex_start_period(&ss->ss_scan_timer, dwell_time);
        AML_PRINT_LOG_INFO("Scan CH %d, scan_ch_ms = %d\n", chan->chan_pri_num, dwell_time);
    }

    if ((wifimac->wm_nrunning == 1)
        || ((wifimac->wm_nrunning == 2) && (concurrent_check_is_vmac_same_pri_channel(wifimac)))) {
         if ((wifimac->drv_priv->drv_wnet_vif_table[NET80211_P2P_VMAC]->vm_curchan) &&
            (wifimac->drv_priv->drv_wnet_vif_table[NET80211_P2P_VMAC]->vm_curchan->chan_pri_num == chan->chan_pri_num)) {
               //the same chan no need to restore, if restore we will drop the scan frame in this chan
               AML_PRINT_LOG_INFO("scan the connected chan no need to restore\n");
        }
         else {
            ss->scan_StateFlags |= SCANSTATE_F_RESTORE;
            ss->scan_StateFlags &= ~SCANSTATE_F_TX_DONE;
            if ((wnet_vif->vm_opmode == WIFINET_M_STA) && (wnet_vif->vm_state == WIFINET_S_CONNECTED)) {
                wifi_mac_scan_set_gain(wifimac, wnet_vif->vm_scanchan_rssi);
            }
         }
    }


    /* vm_mac_mode default is 11GNAC */
    last_mac_mode = wnet_vif->vm_mac_mode;
    if (chan->chan_pri_num >= 1 && chan->chan_pri_num <= 14) {
        if (wnet_vif->vm_p2p->p2p_enable) {
            wnet_vif->vm_mac_mode = WIFINET_MODE_11GN;
        } else {
            wnet_vif->vm_mac_mode = WIFINET_MODE_11BGN;
        }

    } else {
        wnet_vif->vm_mac_mode = WIFINET_MODE_11GNAC;
    }

    if (last_mac_mode != wnet_vif->vm_mac_mode)
        wifi_mac_set_legacy_rates(&wnet_vif->vm_legacy_rates, wnet_vif);

    if (((ss->scan_CfgFlags & WIFINET_SCANCFG_ACTIVE) && !(chan->chan_flags & WIFINET_CHAN_DFS))
        ||((ss->scan_CfgFlags & WIFINET_SCANCFG_ACTIVE) && (chan->chan_flags & WIFINET_CHAN_AWARE))) {
        struct net_device *dev = wnet_vif->vm_ndev;

        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "vid:%d, next_chan_index = %d, chan=%d freq=%d, p2p_enable:%d\n",
            wnet_vif->wnet_vif_id, ss->scan_next_chan_index, chan->chan_pri_num, chan->chan_cfreq1, wnet_vif->vm_p2p->p2p_enable);

        if ((!wnet_vif->vm_p2p->p2p_enable || (wnet_vif->vm_p2p->p2p_enable && ss->ss_nssid == 0)) && !wnet_vif->vm_chan_switch_scan_flag) {
             wifi_mac_send_probereq(wnet_vif->vm_mainsta, wnet_vif->vm_myaddr, dev->broadcast,
                 dev->broadcast, "", 0, wnet_vif->vm_opt_ie, wnet_vif->vm_opt_ie_len);
        }

        for (i = 0; i < ss->ss_nssid; i++) {
            if (wnet_vif->vm_scan_before_connect_flag) {
                wifi_mac_send_probereq(wnet_vif->vm_mainsta, wnet_vif->vm_myaddr, wnet_vif->vm_connchan.da,
                    wnet_vif->vm_connchan.bssid, ss->ss_ssid[i].ssid, ss->ss_ssid[i].len, wnet_vif->vm_opt_ie, wnet_vif->vm_opt_ie_len);

            } else if (wnet_vif->vm_chan_switch_scan_flag) {
                wifi_mac_send_probereq(wnet_vif->vm_mainsta, wnet_vif->vm_myaddr, wnet_vif->vm_des_bssid,
                    wnet_vif->vm_des_bssid, ss->ss_ssid[i].ssid, ss->ss_ssid[i].len, wnet_vif->vm_opt_ie, wnet_vif->vm_opt_ie_len);

            } else {
                wifi_mac_send_probereq(wnet_vif->vm_mainsta, wnet_vif->vm_myaddr, dev->broadcast,
                    dev->broadcast, ss->ss_ssid[i].ssid, ss->ss_ssid[i].len, wnet_vif->vm_opt_ie, wnet_vif->vm_opt_ie_len);
            }
        }

        ss->scan_StateFlags |= SCANSTATE_F_SEND_PROBEREQ_AGAIN;
        if (ss->scan_chan_wait == wnet_vif->vm_scan_time_connect) {
            os_timer_ex_start_period(&ss->ss_probe_timer, 8);

        } else {
            os_timer_ex_start_period(&ss->ss_probe_timer, 20);
        }
    }

    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "OS_SET_TIMER = %d next_chn\n", ss->scan_chan_wait);
}

void wifi_mac_switch_scan_channel(struct wifi_mac *wifimac)
{
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;
    struct wifi_channel *chan;
    struct wlan_net_vif *wnet_vif = ss->VMacPriv;

    //AML_PRINT_LOG_INFO("ss->scan_StateFlags:%08x\n", ss->scan_StateFlags);
    if (ss->scan_next_chan_index >= ss->scan_last_chan_index) {
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_ERROR, "(scan_next_chan_index >= scan_last_chan_index) drop!!!\n");
        return;
    }

    ss->scan_StateFlags &= ~SCANSTATE_F_NOTIFY_AP;
    chan = ss->ss_chans[ss->scan_next_chan_index];
    if ((wifimac->wm_curchan != NULL)
        && (chan->chan_pri_num == wifimac->wm_curchan->chan_pri_num)
        && (wnet_vif->vm_recovery_state == WIFINET_RECOVERY_END)) {
        ss->scan_StateFlags |= SCANSTATE_F_CHANNEL_SWITCH_COMPLETE;
        wifi_mac_scan_channel(wifimac);
    } else {
        ss->scan_StateFlags |= SCANSTATE_F_WAIT_CHANNEL_SWITCH;
        wifi_mac_ChangeChannel(wifimac, chan, 0, wnet_vif->wnet_vif_id);
        os_timer_ex_start_period(&ss->ss_scan_timer, 30);
    }
}
#endif

void scan_next_chan(struct wifi_mac *wifimac)
{
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;
    struct wlan_net_vif *wnet_vif = ss->VMacPriv;
    unsigned char i = 0;
    unsigned char j = 0;
    struct wifi_channel *chan;
    enum wifi_mac_macmode last_mac_mode;
    unsigned int send_packet_num = 2;

    if (ss->scan_next_chan_index >= ss->scan_last_chan_index)
    {
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_ERROR, "  (scan_next_chan_index >= scan_last_chan_index) drop!!!\n");
        return;
    }

    if (!((wifimac->wm_nrunning == 2) && (!concurrent_check_is_vmac_same_pri_channel(wifimac)))) {
        //AML_PRINT_LOG_INFO("\n");
        os_timer_ex_start_period(&ss->ss_scan_timer, ss->scan_chan_wait);
    }

    if ((wifimac->wm_nrunning == 1)
        || ((wifimac->wm_nrunning == 2) && (concurrent_check_is_vmac_same_pri_channel(wifimac)))) {
        ss->scan_StateFlags |= SCANSTATE_F_RESTORE;
        ss->scan_StateFlags &= ~SCANSTATE_F_TX_DONE;
    }
    chan = ss->ss_chans[ss->scan_next_chan_index++];
    wifi_mac_ChangeChannel(wifimac, chan, 0, wnet_vif->wnet_vif_id);

    ss->scan_StateFlags &= ~SCANSTATE_F_DISCARD;
    /* vm_mac_mode default is 11GNAC */
    last_mac_mode = wnet_vif->vm_mac_mode;
    if (chan->chan_pri_num >= 1 && chan->chan_pri_num <= 14) {
        if (wnet_vif->vm_p2p->p2p_enable) {
            wnet_vif->vm_mac_mode = WIFINET_MODE_11GN;
        } else {
            wnet_vif->vm_mac_mode = WIFINET_MODE_11BGN;
        }

    } else {
        wnet_vif->vm_mac_mode = WIFINET_MODE_11GNAC;
    }

    if (last_mac_mode != wnet_vif->vm_mac_mode)
        wifi_mac_set_legacy_rates(&wnet_vif->vm_legacy_rates, wnet_vif);

    if (ss->scan_CfgFlags & WIFINET_SCANCFG_ACTIVE)
    {
        struct net_device *dev = wnet_vif->vm_ndev;

        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "vid:%d, next_chan_index = %d, chan=%d freq=%d, p2p_enable:%d\n",
            wnet_vif->wnet_vif_id, ss->scan_next_chan_index, chan->chan_pri_num, chan->chan_cfreq1, wnet_vif->vm_p2p->p2p_enable);

        if (!wnet_vif->vm_p2p->p2p_enable) {
            for (i = 0; i < send_packet_num; i++)
                wifi_mac_send_probereq(wnet_vif->vm_mainsta, wnet_vif->vm_myaddr, dev->broadcast,
                    dev->broadcast, "", 0, wnet_vif->vm_opt_ie, wnet_vif->vm_opt_ie_len);
            send_packet_num = 1;
        }

        for (i = 0; i < ss->ss_nssid; i++)
            for (j = 0; j < send_packet_num; j++)
                wifi_mac_send_probereq(wnet_vif->vm_mainsta, wnet_vif->vm_myaddr, dev->broadcast,
                    dev->broadcast, ss->ss_ssid[i].ssid, ss->ss_ssid[i].len, wnet_vif->vm_opt_ie, wnet_vif->vm_opt_ie_len);
    }

    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "OS_SET_TIMER = %d next_chn\n", ss->scan_chan_wait);
}


int wifi_mac_scan_buff_and_chk_tx(struct wifi_mac *wifimac)
{
    struct wlan_net_vif *tmpwnet_vif = NULL, *tmpwnet_vif_next = NULL;
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;
    struct drv_private *drv_priv = wifimac->drv_priv;

    list_for_each_entry_safe(tmpwnet_vif,tmpwnet_vif_next, &wifimac->wm_wnet_vifs, vm_next) {
        if (tmpwnet_vif->vm_state == WIFINET_S_CONNECTED) {
            tmpwnet_vif->vm_pstxqueue_flags |= WIFINET_PSQUEUE_PS4QUIET;
            wifimac->drv_priv->drv_ops.drv_set_is_mother_channel(wifimac->drv_priv, tmpwnet_vif->wnet_vif_id, 0);
        }
    }

    if (!drv_priv->hal_priv->hal_ops.hal_tx_empty()) {
        ss->scan_StateFlags |= SCANSTATE_F_WAIT_PKT_CLEAR;
        ss->scan_StateFlags &= ~SCANSTATE_F_TX_DONE;
        return 1;

    } else {
        ss->scan_StateFlags |= SCANSTATE_F_TX_DONE;
        return 0;
    }
}


void wifi_mac_scan_notify_ap(struct wifi_mac *wifimac)
{
    struct wlan_net_vif *tmpwnet_vif = NULL, *tmpwnet_vif_next = NULL;
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;

    list_for_each_entry_safe(tmpwnet_vif,tmpwnet_vif_next, &wifimac->wm_wnet_vifs, vm_next) {
        if ((tmpwnet_vif->vm_opmode == WIFINET_M_STA) && (tmpwnet_vif->vm_state == WIFINET_S_CONNECTED)) {
            wifi_mac_pwrsave_send_nulldata(tmpwnet_vif->vm_mainsta, NULLDATA_PS, 1);
            ss->scan_StateFlags |= SCANSTATE_F_NOTIFY_AP;
        }
    }
}


void wifi_mac_scan_timeout(SYS_TYPE param1,SYS_TYPE param2,
    SYS_TYPE param3,SYS_TYPE param4,SYS_TYPE param5)
{
    struct wifi_mac_scan_state *ss = (struct wifi_mac_scan_state *) param1;
    struct wlan_net_vif *wnet_vif = ss->VMacPriv;
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    unsigned char scandone;
    unsigned char need_notify_ap = 1;
    struct wifi_channel *chan = WIFINET_CHAN_ERR;

    os_timer_ex_cancel(&ss->ss_scan_timer, CANCEL_SLEEP);

    //check replayer count
    if (wifimac->wm_scanplayercnt != (unsigned long)param5) {
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_WARN, "wm_scanplayercnt %ld, ignore... \n", wifimac->wm_scanplayercnt);
        return;
    }

    if (!wnet_vif->vm_mainsta) {
        AML_PRINT(AML_LOG_ID_SCAN,AML_LOG_LEVEL_WARN, " vid:%d vm_mainsta is NULL, cancel scan\n", wnet_vif->wnet_vif_id);
        ss->scan_StateFlags |= SCANSTATE_F_CANCEL;
    }

    if (!(wifimac->wm_flags & WIFINET_F_SCAN)) {
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_WARN, " drop wm_flags  0x%x \n",wifimac->wm_flags );
        goto end;
    }

    if (ss->scan_StateFlags & SCANSTATE_F_DISCONNECT_REQ_CANCEL) {
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_WARN, " end scan due to disconnect, no need to restore channel\n");
        goto end;
    }

    if (ss->scan_StateFlags & SCANSTATE_F_GET_CONNECT_AP) {
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_WARN, " find connect ap\n");
        goto end;
    }

    if (!(ss->scan_StateFlags & SCANSTATE_F_START)) {
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_WARN, "SCANSTATE_F_START flag not set, end scan\n");
        goto end;
    }

    scandone = (ss->scan_next_chan_index >= ss->scan_last_chan_index) ||
        (ss->scan_StateFlags & SCANSTATE_F_CANCEL);

    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG,"nxt chn 0x%x, lst chn 0x%x ss_flag 0x%x scandone %d\n",
        ss->scan_next_chan_index, ss->scan_last_chan_index, ss->scan_StateFlags, scandone);

    if (ss->scan_next_chan_index < ss->scan_last_chan_index) {
        chan = ss->ss_chans[ss->scan_next_chan_index];
        if ((chan != WIFINET_CHAN_ERR) && (wifimac->wm_curchan != WIFINET_CHAN_ERR)
            && (chan->chan_pri_num == wifimac->wm_curchan->chan_pri_num) && (wifimac->wm_curchan->chan_bw == 0)) {
            need_notify_ap = 0;
        }
    }

    //AML_PRINT_LOG_INFO("ss->scan_StateFlags:%08x, scandone:%d, need_notify_ap:%d\n", ss->scan_StateFlags, scandone, need_notify_ap);
    if (wifimac->wm_nrunning != 0) {
        if (ss->scan_StateFlags & SCANSTATE_F_RESTORE) {
            wifi_mac_restore_wnet_vif_channel(wnet_vif);
            ss->scan_StateFlags &= ~SCANSTATE_F_RESTORE;
            ss->scan_StateFlags |= SCANSTATE_F_DISCARD;

            if (scandone) {
                AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_INFO, "scandone, end scan\n");
                goto end;
            }

            ss->scan_StateFlags |= SCANSTATE_F_WAIT_TBTT;
            os_timer_ex_start_period(&ss->ss_scan_timer, WIFINET_SCAN_DEFAULT_INTERVAL);
            return;

        } else if  (!scandone && need_notify_ap) {

            struct wlan_net_vif *connect_wnet = wifi_mac_running_wnet_vif(wifimac);

            if (!((connect_wnet != NULL) && (connect_wnet->vm_opmode == WIFINET_M_HOSTAP))) {
                if ((wifimac->wm_nrunning == 1) || ((wifimac->wm_nrunning == 2) && concurrent_check_is_vmac_same_pri_channel(wifimac))) {

                    if (!(ss->scan_StateFlags & SCANSTATE_F_TX_DONE)) {
                        if (wifi_mac_scan_buff_and_chk_tx(wifimac)) {
                            return;
                        }
                    }

                    if (!(ss->scan_StateFlags & SCANSTATE_F_NOTIFY_AP)) {
                        wifi_mac_scan_notify_ap(wifimac);
                        os_timer_ex_start_period(&wifimac->wm_scan->ss_scan_timer, LEAKY_AP_DET_WIN);
                        return;
                    }
                }
            } else {
                //add NOA in the near future
            }
        }
    }

    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG,"done 0x%x, ss_flag 0x%x\n", scandone, ss->scan_StateFlags);
    if (!scandone) {
        #ifndef FW_RF_CALIBRATION
            scan_next_chan(wifimac);
            return;
        #else
            if (ss->scan_StateFlags & SCANSTATE_F_CHANNEL_SWITCH_COMPLETE) {
                wifi_mac_scan_channel(wifimac);

            } else {
                wifi_mac_switch_scan_channel(wifimac);
            }
            return;
        #endif
    }

end:
    wifi_mac_end_scan(ss);
    return;
}

int wifi_mac_scan_timeout_ex(void *arg)
{
    struct wifi_mac_scan_state *ss = (struct wifi_mac_scan_state *) arg;
    struct wlan_net_vif *wnet_vif = ss->VMacPriv;
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;

    //AML_PRINT_LOG_INFO("ss->scan_StateFlags:%08x\n", ss->scan_StateFlags);

    WIFI_SCAN_LOCK(ss);
    if (ss->scan_StateFlags & SCANSTATE_F_WAIT_TBTT) {
        ss->scan_StateFlags &= ~SCANSTATE_F_WAIT_TBTT;
    }

    if (ss->scan_StateFlags & SCANSTATE_F_WAIT_CHANNEL_SWITCH) {
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_WARN,"switch channel timeout \n");
        ss->scan_StateFlags &= ~SCANSTATE_F_WAIT_CHANNEL_SWITCH;
        ss->scan_StateFlags |= SCANSTATE_F_CHANNEL_SWITCH_COMPLETE;
    }

    if (ss->scan_StateFlags & SCANSTATE_F_SEND_PROBEREQ_AGAIN) {
        ss->scan_StateFlags &= ~SCANSTATE_F_SEND_PROBEREQ_AGAIN;
    }

    wifimac->wm_scanplayercnt++;
    wifi_mac_add_work_task(wifimac, wifi_mac_scan_timeout, NULL,
        (SYS_TYPE)arg, (SYS_TYPE)wnet_vif, 0, 0, (SYS_TYPE)wifimac->wm_scanplayercnt);
    WIFI_SCAN_UNLOCK(ss);

    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "ss->scan_next_chan_index = %d, ss->scan_StateFlags:%08x\n",
        ss->scan_next_chan_index, ss->scan_StateFlags);

    return OS_TIMER_NOT_REARMED;
}

static void
scan_start_task(SYS_TYPE param1,SYS_TYPE param2,
    SYS_TYPE param3,SYS_TYPE param4,SYS_TYPE param5)
{
    struct wlan_net_vif *wnet_vif = (struct wlan_net_vif *)param4;
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;

    WIFI_SCAN_LOCK(ss);
    if ((wnet_vif->wnet_vif_replaycounter != (int)param5) || (wifimac->wm_scanplayercnt != (unsigned long)param3)
        || (ss->scan_StateFlags & SCANSTATE_F_START) || !(wifimac->wm_flags & WIFINET_F_SCAN)) {
        AML_PRINT_LOG_INFO("scan_StateFlags:%04x, wm_scanplayercnt:%ld\n", ss->scan_StateFlags, wifimac->wm_scanplayercnt);
        WIFI_SCAN_UNLOCK(ss);
        return;
    }

    ss->scan_StateFlags = 0;
    ss->scan_StateFlags |= SCANSTATE_F_START;
    WIFI_SCAN_UNLOCK(ss);
    os_timer_ex_cancel(&ss->ss_scan_timer, CANCEL_SLEEP);

    if ((ss->scan_CfgFlags & WIFINET_SCANCFG_CHANSET) == 0)
    {
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "\n");
        vm_scan_setup_chan(ss,wnet_vif);
    }
    wifimac->drv_priv->stop_noa_flag = 0;
    wifi_mac_scan_start(wifimac);

    //set up scan channel index to 0
    ss->scan_next_chan_index = 0;
    AML_PRINT_LOG_INFO("wm_nrunning:%d\n", wifimac->wm_nrunning);

    if (wifimac->wm_nrunning == 0) {
        wifi_mac_scan_timeout_ex(ss);

    } else {
        ss->scan_StateFlags |= SCANSTATE_F_WAIT_TBTT;
        os_timer_ex_start_period(&ss->ss_scan_timer, 110);
    }

    return;
}

void wifi_mac_check_switch_chan_result(struct wlan_net_vif * wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;
    struct scaninfo_table *st = ss->ScanTablePriv;
    struct scaninfo_entry *se = NULL, *SI_next = NULL;
    unsigned char find = 0;

    WIFI_SCAN_SE_LIST_LOCK(st);
    list_for_each_entry_safe(se,SI_next,&st->st_entry,se_list)
    {
        se->scaninfo.SI_age = jiffies - se->LastUpdateTime;
        if (!strcmp(se->scaninfo.SI_ssid + 2, wnet_vif->vm_des_ssid[0].ssid)
            && WIFINET_ADDR_EQ(wnet_vif->vm_des_bssid, se->scaninfo.SI_bssid)
            && wnet_vif->vm_switchchan->chan_pri_num == se->scaninfo.SI_chan->chan_pri_num) {
            find = 1;
        }
    }
    WIFI_SCAN_SE_LIST_UNLOCK(st);
    if (find == 1 || wnet_vif->vm_chan_switch_scan_count == 5) {
        if (wnet_vif->vm_mainsta) {
            wnet_vif->vm_mainsta->sta_channel_switch_mode = 0;
        }

        wnet_vif->vm_chan_switch_scan_count = 0;
        wnet_vif->vm_chan_switch_scan_flag = 0;
    }
}

void wifi_mac_end_scan( struct wifi_mac_scan_state *ss)
{
    struct wlan_net_vif *wnet_vif = ss->VMacPriv;
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    int find_roaming_node = 0, scan_roaming_node = 0;
    static int before_conn_scan_times = 0;
    struct scaninfo_entry *se = NULL, *SI_next = NULL;
    struct scaninfo_table *st = ss->ScanTablePriv;
    unsigned char encryption_type[5];

    if (!(wifimac->wm_flags & WIFINET_F_SCAN)) {
        AML_PRINT_LOG_ERR("not in scan status\n");
        if (ss->scan_CfgFlags & WIFINET_SCANCFG_USERREQ && !find_roaming_node) {
            wifi_mac_notify_scan_done(wnet_vif);
            ss->scan_CfgFlags &= (~WIFINET_SCANCFG_USERREQ);
        }
        return;
    }

    os_timer_ex_cancel(&ss->ss_scan_timer, CANCEL_SLEEP);
    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "chan_index = %d ,scan_CfgFlags 0x%x, wifimac->wm_nrunning is:%d\n",
        ss->scan_next_chan_index, ss->scan_CfgFlags, wifimac->wm_nrunning);

    if (wifimac->wm_nrunning == 1) {
        struct wlan_net_vif *connect_wnet = wifi_mac_running_wnet_vif(wifimac);
        if (connect_wnet != NULL) {
            if (connect_wnet->vm_opmode == WIFINET_M_HOSTAP &&
                connect_wnet->vm_p2p->noa_app_ie[WIFINET_APPIE_FRAME_BEACON].length &&
                connect_wnet->vm_p2p->p2p_enable == 1) {
                AML_PRINT_LOG_INFO("clear noa ie\n");
                connect_wnet->vm_p2p->ap_mode_set_noa_enable = 0;
                wifi_mac_rm_app_ie(&connect_wnet->vm_p2p->noa_app_ie[WIFINET_APPIE_FRAME_BEACON]);
                vm_p2p_update_beacon_app_ie(connect_wnet);
                wifimac->drv_priv->stop_noa_flag = 1;
                if (P2P_NoA_START_FLAG(connect_wnet->vm_p2p->HiP2pNoaCountNow)) {
                    p2p_noa_start_irq(connect_wnet->vm_p2p, wifimac->drv_priv);
                }
                connect_wnet->vm_p2p->HiP2pNoaCountNow = 0;
            }
        }

    } else {
        wifimac->drv_priv->stop_noa_flag = 0;
    }

    wifi_mac_scan_end(wifimac);
    wifimac->wm_lastscan = jiffies;
    ss->scan_StateFlags = 0;

    /* restore mac mode after scanning */
    if (wnet_vif->vm_mainsta) {
        wnet_vif->vm_mac_mode = wnet_vif->vm_mainsta->sta_bssmode;
        wifi_mac_set_legacy_rates(&wnet_vif->vm_legacy_rates, wnet_vif);
    }

    /*if in roaming scan, print the result*/
    if (wnet_vif->vm_chan_roaming_scan_flag || wifimac->wm_roaming_state == ROAMING_SCAN) {
        list_for_each_entry_safe(se,SI_next,&st->st_entry,se_list) {
            if (memcmp(ss->roaming_ssid.ssid, ssidie_sprintf(se->scaninfo.SI_ssid), ss->roaming_ssid.len) == 0) {
                memset(encryption_type, 0, sizeof(encryption_type));
                if (se->scaninfo.SI_rsn_ie[0])
                    memcpy(encryption_type, "WPA2", strlen("WPA2"));
                else if (se->scaninfo.SI_wpa_ie[0])
                    memcpy(encryption_type, "WPA", strlen("WPA"));
                else if (se->scaninfo.SI_wps_ie[0])
                    memcpy(encryption_type, "WPS", strlen("WPS"));
                else
                    memcpy(encryption_type, "ESS", strlen("ESS"));
                AML_PRINT_LOG_INFO("Roaming Scan rssi:%d channel:%d encryption:%s bssid:%s ssid: %s\n",
                    se->scaninfo.SI_rssi - 256, se->scaninfo.SI_chan->chan_pri_num, encryption_type,
                    ether_sprintf(se->scaninfo.SI_bssid), ssidie_sprintf(se->scaninfo.SI_ssid));
                scan_roaming_node++;
            }
        }
        if (scan_roaming_node == 0) {
            wifimac->wm_roaming_state = ROAMING_STOP;
            AML_PRINT_LOG_INFO("No Roaming bssid find!\nRoaming quit!\n");
        }
    }

    /*if in roaming mode ,check if can roaming*/
    if (wnet_vif->vm_mainsta != NULL && wnet_vif->vm_chan_roaming_scan_flag) {
        struct scaninfo_entry *best_node = &wnet_vif->vm_connect_scan_entry;

        if (wifi_mac_scan_get_best_node(ss, wnet_vif, best_node)) {
            best_node->se_valid = 0;
            if (memcmp(best_node->scaninfo.SI_bssid, wnet_vif->vm_mainsta->sta_bssid, WIFINET_ADDR_LEN)) {
                wifi_mac_scan_set_match_node(ss, wnet_vif);
                best_node->se_valid = 1;
                wifi_mac_top_sm(wnet_vif, WIFINET_S_SCAN, 0);
                find_roaming_node = 1;
                wnet_vif->vm_wmac->wm_scan->roaming_full_scan = 0;
                AML_PRINT_LOG_INFO("Roaming get better bssid rssi:%d channel:%d bssid:%s ssid: %s\n",
                    best_node->scaninfo.SI_rssi - 256, best_node->scaninfo.SI_chan->chan_pri_num,
                    ether_sprintf(best_node->scaninfo.SI_bssid), ssidie_sprintf(best_node->scaninfo.SI_ssid));
                wifimac->wm_roaming_state = ROAMING_CONNECTING;
                AML_PRINT_LOG_INFO("Roaming connect start!\n");
            }
        }
        else {
            wifimac->wm_roaming_state = ROAMING_STOP;
            AML_PRINT_LOG_INFO("Roaming get better bssid fail!\nRoaming quit!\n");
        }
    }

    if ((wnet_vif->vm_opmode != WIFINET_M_HOSTAP)
        && (wnet_vif->vm_opmode != WIFINET_M_P2P_GO)
        && (wnet_vif->vm_opmode != WIFINET_M_WDS)
        && (wnet_vif->vm_opmode != WIFINET_M_MONITOR)) {
        if (wifi_mac_scan_connect(ss, wnet_vif)) {
            before_conn_scan_times = 0;

        } else {
            if (wnet_vif->vm_scan_before_connect_flag) {
                before_conn_scan_times++;
            }
        }
    } else if ((wnet_vif->vm_opmode == WIFINET_M_HOSTAP)
                ||(wnet_vif->vm_opmode == WIFINET_M_P2P_GO)) {
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG,
                "vm_opmode %d scan_CfgFlags 0x%x\n",
                wnet_vif->vm_opmode,ss->scan_CfgFlags);
        if (ss->scan_CfgFlags  & WIFINET_SCANCFG_CREATE) {
            wifi_mac_chk_ap_chan(ss, wnet_vif);
        }
    }

    if (find_roaming_node) {
        wnet_vif->vm_chan_roaming_scan_flag = 0;
        wifi_mac_scan_access(wnet_vif);
    }

    /*upload scanning result and notify scanning done to upper
    after processing scan result. */
    if (ss->scan_CfgFlags & WIFINET_SCANCFG_USERREQ && !find_roaming_node) {
        wifi_mac_notify_scan_done(wnet_vif);
        ss->scan_CfgFlags &= (~WIFINET_SCANCFG_USERREQ);
    }

    if ((wnet_vif->vm_opmode == WIFINET_M_STA) && ((wnet_vif->vm_state == WIFINET_S_CONNECTED) ||
       (wnet_vif->vm_scan_before_connect_flag))) {
        if (g_auto_gain_base != 0) {
            wifi_mac_set_channel_rssi(wifimac, g_auto_gain_base);
        }
        else{
            wifi_mac_set_channel_rssi(wifimac, 174);
        }
    }
    wifi_mac_restore_wnet_vif_channel(wnet_vif);
    wifimac->wm_flags &= ~WIFINET_F_SCAN;
    ss->scan_CfgFlags = 0;
    wifimac->wm_p2p_connection_protect = 0;
    wnet_vif->vm_pstxqueue_flags &= ~WIFINET_PSQUEUE_PS4QUIET;
    wnet_vif->vm_scan_before_connect_flag = 0;
    //clear saved_ssids
    ss->ss_nssid = 0;
    memset(ss->ss_ssid,0,sizeof(ss->ss_ssid));
    wifi_mac_scan_access(wnet_vif);

    WIFI_ALPHA_LOCK(wifimac);
    if (wifimac->wm_alpha_set_forbid & SCAN_FORBIDDEN) {
        wifi_mac_run_delayed_country_switch(wifimac);
        wifimac->wm_alpha_set_forbid &= ~SCAN_FORBIDDEN;
    }
    WIFI_ALPHA_UNLOCK(wifimac);

    if (wnet_vif->vm_chan_switch_scan_flag) {
        wifi_mac_check_switch_chan_result(wnet_vif);
        if (wnet_vif->vm_mainsta && wnet_vif->vm_mainsta->sta_channel_switch_mode) {
            wifi_mac_start_scan(wnet_vif, WIFINET_SCANCFG_USERREQ | WIFINET_SCANCFG_ACTIVE | WIFINET_SCANCFG_FLUSH |
                                WIFINET_SCANCFG_CREATE, wnet_vif->vm_des_nssid, wnet_vif->vm_des_ssid);
            wnet_vif->vm_chan_switch_scan_count++;
        }
    }

    /*scan in roaming candidate channels 2 times,and full scan 2 times*/
    if (wnet_vif->vm_chan_roaming_scan_flag && !find_roaming_node) {
        wnet_vif->vm_chan_roaming_scan_count++;
        if (wnet_vif->vm_wmac->wm_scan->roaming_full_scan == 0) {
            if (wnet_vif->vm_chan_roaming_scan_count >= 2) {
                wnet_vif->vm_wmac->wm_scan->roaming_full_scan = 1;
                wnet_vif->vm_chan_roaming_scan_count = 0;
             }
            wifi_mac_start_scan(wnet_vif, WIFINET_SCANCFG_USERREQ | WIFINET_SCANCFG_ACTIVE | WIFINET_SCANCFG_FLUSH |
                                WIFINET_SCANCFG_CREATE, wnet_vif->vm_des_nssid, wnet_vif->vm_des_ssid);
        }
        else if (wnet_vif->vm_chan_roaming_scan_count < 2) {
            wifi_mac_start_scan(wnet_vif, WIFINET_SCANCFG_USERREQ | WIFINET_SCANCFG_ACTIVE | WIFINET_SCANCFG_FLUSH |
                                WIFINET_SCANCFG_CREATE, wnet_vif->vm_des_nssid, wnet_vif->vm_des_ssid);
        }
        else {
            wnet_vif->vm_chan_roaming_scan_flag = 0;
            wifi_mac_scan_access(wnet_vif);
        }
    }

    if (before_conn_scan_times == 1) {
        wnet_vif->vm_scan_before_connect_flag = 1;
        wifi_mac_start_scan(wnet_vif, WIFINET_SCANCFG_USERREQ | WIFINET_SCANCFG_ACTIVE | WIFINET_SCANCFG_FLUSH |
        WIFINET_SCANCFG_CREATE, wnet_vif->vm_des_nssid, wnet_vif->vm_des_ssid);

    } else if (before_conn_scan_times == 2 ) {
        wnet_vif->vm_scan_before_connect_flag = 0;
        wifi_mac_start_scan(wnet_vif, WIFINET_SCANCFG_USERREQ | WIFINET_SCANCFG_ACTIVE | WIFINET_SCANCFG_FLUSH |
        WIFINET_SCANCFG_CREATE, wnet_vif->vm_des_nssid, wnet_vif->vm_des_ssid);
        before_conn_scan_times = 0;

    } else {
        before_conn_scan_times = 0;
    }

    AML_PRINT_LOG_INFO("---> scan finish, vid:%d, clean vm_flags 0x%x\n", wnet_vif->wnet_vif_id, wifimac->wm_flags);
    os_timer_ex_start_period(&wnet_vif->vm_pwrsave.ips_timer_presleep, wnet_vif->vm_pwrsave.ips_inactivitytime);
}

void wifi_mac_notify_ap_success(struct wlan_net_vif *wnet_vif) {
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct drv_private *drv_priv = wifimac->drv_priv;
    struct wlan_net_vif *p2p_vmac = drv_priv->drv_wnet_vif_table[NET80211_P2P_VMAC];
    struct wlan_net_vif *sta_vmac = drv_priv->drv_wnet_vif_table[NET80211_MAIN_VMAC];

    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "vid:%d\n", wnet_vif->wnet_vif_id);

    //scan branch
    if ((wifimac->wm_nrunning > 0) && (wifimac->wm_flags & WIFINET_F_SCAN)
        && (wifimac->wm_scan->scan_StateFlags & SCANSTATE_F_NOTIFY_AP)) {
        if (drv_priv->hal_priv->hal_ops.hal_tx_empty()) {
            os_timer_ex_cancel(&wifimac->wm_scan->ss_scan_timer, 1);
            wifimac->wm_scan->scan_StateFlags |= SCANSTATE_F_RX_CHKING_LEAKAP_PKT;
            wifi_mac_scan_chk_leakap_hrtimer_start(wifimac);
            AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "notify success and start check leakap timer\n");
        }
    }

    //p2p branch
    if ((wifimac->wm_nrunning == 1)
        && (p2p_vmac->vm_p2p->p2p_flag & P2P_WAIT_SWITCH_CHANNEL)) {
        if (drv_priv->hal_priv->hal_ops.hal_tx_empty()) {
            wifi_mac_ChangeChannel(wifimac, p2p_vmac->vm_p2p->work_channel, 0, p2p_vmac->wnet_vif_id);
            p2p_vmac->vm_p2p->p2p_flag &= ~P2P_WAIT_SWITCH_CHANNEL;

        } else {
            //AML_PRINT_LOG_INFO("still have pkt in hal, wait\n");
            p2p_vmac->vm_p2p->p2p_flag &= ~P2P_WAIT_SWITCH_CHANNEL;
            p2p_vmac->vm_p2p->p2p_flag |= P2P_ALLOW_SWITCH_CHANNEL;
        }
    }

    if ((wifimac->wm_nrunning > 0)
            && (sta_vmac->vm_flags_ext2 & WIFINET_FEXT2_SWITCH_CHANNEL)) {
        if (drv_priv->hal_priv->hal_ops.hal_tx_empty()) {
            wifi_mac_ChangeChannel(wifimac, sta_vmac->vm_remainonchan, 0, sta_vmac->wnet_vif_id);
            sta_vmac->vm_flags_ext2 &= ~WIFINET_FEXT2_SWITCH_CHANNEL;
        } else {
            //printk("still have pkt in hal, wait\n");
            sta_vmac->vm_flags_ext2 &= ~WIFINET_FEXT2_SWITCH_CHANNEL;
            sta_vmac->vm_flags_ext2 |= WIFINET_FEXT2_ALLOW_SWITCH_CHANNEL;
        }
    }

    //vsdb channel switch branch
    if (wifimac->wm_nrunning == 2) {
        #ifdef  CONFIG_CONCURRENT_MODE
            if (wifimac->wm_vsdb_flags & CONCURRENT_NOTIFY_AP) {
                wifimac->wm_vsdb_flags &= ~CONCURRENT_NOTIFY_AP;
                if (drv_priv->hal_priv->hal_ops.hal_tx_empty()) {
                    concurrent_vsdb_do_channel_change(wifimac);

                } else {
                    //AML_PRINT_LOG_INFO("still have pkt in hal, wait\n");
                    wifimac->wm_vsdb_flags |= CONCURRENT_NOTIFY_AP_SUCCESS;
                }
            }
        #endif
    }
}

void wifi_mac_notify_pkt_clear(struct wifi_mac *wifimac) {
    struct drv_private *drv_priv = wifimac->drv_priv;
    struct wlan_net_vif *p2p_vmac = drv_priv->drv_wnet_vif_table[NET80211_P2P_VMAC];
    struct wlan_net_vif *sta_vmac = drv_priv->drv_wnet_vif_table[NET80211_MAIN_VMAC];

    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "\n");

    if ((wifimac->wm_nrunning > 0) && (wifimac->wm_flags & WIFINET_F_SCAN)
        && (wifimac->wm_scan->scan_StateFlags & SCANSTATE_F_WAIT_PKT_CLEAR)) {
        if (drv_priv->hal_priv->hal_ops.hal_tx_empty()) {
            wifimac->wm_scan->scan_StateFlags &= ~SCANSTATE_F_WAIT_PKT_CLEAR;
            wifimac->wm_scan->scan_StateFlags |= SCANSTATE_F_TX_DONE;
            os_timer_ex_cancel(&wifimac->wm_scan->ss_scan_timer, 1);
            wifi_mac_scan_timeout_ex(wifimac->wm_scan);
        }
    }

    //p2p branch
    if ((wifimac->wm_nrunning == 1)
        && (p2p_vmac->vm_p2p->p2p_flag & P2P_ALLOW_SWITCH_CHANNEL)) {
        if (drv_priv->hal_priv->hal_ops.hal_tx_empty()) {
            wifi_mac_ChangeChannel(wifimac, p2p_vmac->vm_p2p->work_channel, 0, p2p_vmac->wnet_vif_id);
            p2p_vmac->vm_p2p->p2p_flag &= ~P2P_ALLOW_SWITCH_CHANNEL;
        }
    }

    if ((wifimac->wm_nrunning > 0)
            && (sta_vmac->vm_flags_ext2 & WIFINET_FEXT2_ALLOW_SWITCH_CHANNEL)) {
        if (drv_priv->hal_priv->hal_ops.hal_tx_empty()) {
            wifi_mac_ChangeChannel(wifimac, sta_vmac->vm_remainonchan, 0, sta_vmac->wnet_vif_id);
            sta_vmac->vm_flags_ext2 &= ~WIFINET_FEXT2_ALLOW_SWITCH_CHANNEL;
        }
    }

    if (wifimac->wm_nrunning == 2) {
        #ifdef  CONFIG_CONCURRENT_MODE
            if ((wifimac->wm_vsdb_flags & CONCURRENT_NOTIFY_AP_SUCCESS)
                || (wifimac->wm_vsdb_flags & CONCURRENT_AP_SWITCH_CHANNEL)) {
                if (drv_priv->hal_priv->hal_ops.hal_tx_empty()) {
                    concurrent_vsdb_do_channel_change(wifimac);
                }
            }
        #endif
    }
}

void wifi_mac_cancel_scan(struct wifi_mac *wifimac)
{
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;
//    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "<%s> vm_opmode %d\n",
//        wnet_vif->vm_ndev->name,wnet_vif->vm_opmode);

    if (wifimac->wm_flags & WIFINET_F_SCAN)
    {
        ss->scan_StateFlags |= SCANSTATE_F_CANCEL;
//        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG,"<running> wnet_vif_id = %d\n", wnet_vif->wnet_vif_id);
        if (ss->scan_StateFlags & SCANSTATE_F_START)
        {
            os_timer_ex_cancel(&wifimac->wm_scan->ss_scan_timer, 1);
            wifi_mac_scan_timeout_ex(wifimac->wm_scan);
        }
    }
}

int vm_is_p2p_connect_scan(struct wlan_net_vif *wnet_vif, struct cfg80211_scan_request *request)
{
    return (request->n_channels == 1 && !wnet_vif->vm_scan_before_connect_flag && wnet_vif->vm_p2p_support);
}

int vm_scan_user_set_chan(struct wlan_net_vif *wnet_vif,
    struct cfg80211_scan_request *request)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;
    struct wifi_channel *c;
    int i=0,j=0;
    static unsigned char user_chan_aware_cnt = 0;

    ss->scan_last_chan_index = 0;
    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "wm_nchans=%d request_ch %d\n",
        wifimac->wm_nchans,request->n_channels);

    WIFI_CHANNEL_LOCK(wifimac);
    for (j = 0; j < request->n_channels; j++) {
        for (i = 0; i < wifimac->wm_nchans; i++) {
            c = &wifimac->wm_channels[i];

            if (user_chan_aware_cnt >= 10) {
                c->chan_flags &= ~WIFINET_CHAN_AWARE;
                user_chan_aware_cnt = 0;
            }

            if (c->chan_bw == WIFINET_BWC_WIDTH20) {
                if (wnet_vif->vm_p2p_support && wnet_vif->vm_p2p->social_channel) {
                    if ((c->chan_cfreq1 != SOCIAL_CHAN_1) && (c->chan_cfreq1 != SOCIAL_CHAN_2) && (c->chan_cfreq1 != SOCIAL_CHAN_3)) {
                        continue;
                    }
                }

                if (c->chan_cfreq1 != request->channels[j]->center_freq) {
                    continue;
                }

                if ((aml_iwpriv_get_band() == CFG_BAND_B) && (!WIFINET_IS_CHAN_2GHZ(c))) {
                    continue;
                } else if ((aml_iwpriv_get_band() == CFG_BAND_A) && (!WIFINET_IS_CHAN_5GHZ(c))) {
                    continue;
                } else if (!g_DFS_on && c->chan_flags & WIFINET_CHAN_DFS) {
                    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "skip the DFS channel %d!\n", c->chan_pri_num);
                    continue;
                }

                ss->ss_chans[ss->scan_last_chan_index++] = c;
            }
        }
    }
    WIFI_CHANNEL_UNLOCK(wifimac);

    user_chan_aware_cnt++;
    wifi_mac_set_scan_time(wnet_vif);
    AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_DEBUG, "vid:%d ss->scan_next_chan_index=%d \
        ss->scan_last_chan_index=%d, ss->scan_chan_wait=0x%xms, HZ = %d LINUX_VERSION_CODE =%x\n",
        wnet_vif->wnet_vif_id, ss->scan_next_chan_index, ss->scan_last_chan_index,
        ss->scan_chan_wait, HZ, LINUX_VERSION_CODE);

    return 0;
}

int wifi_mac_scan_before_connect(struct wifi_mac_scan_state *ss, struct wlan_net_vif *wnet_vif, int flags)
{
    /* if roaming find node to connect ,don't scan*/
    if (wnet_vif->vm_connect_scan_entry.se_valid) {
        wifi_mac_connect(wnet_vif, &wnet_vif->vm_connect_scan_entry.scaninfo);
        wnet_vif->vm_connect_scan_entry.se_valid = 0;
        return 1;
    }

    if (wifi_mac_scan_get_match_node(ss, wnet_vif) == 0) {
         AML_PRINT_LOG_INFO("not found bss in former scan results\n");

    } else {
        wnet_vif->vm_scan_before_connect_flag = 1;
    }

    return 0;
}

int wifi_mac_start_scan(struct wlan_net_vif *wnet_vif, int flags,
    unsigned int nssid, const struct wifi_mac_ScanSSID ssids[])
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;

    if (wifimac->wm_mac_exit) {
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_WARN, "<%s>  drop scan due to interface down\n", wnet_vif->vm_ndev->name);
        return 0;

    } else if (wifimac->wm_flags & WIFINET_F_SCAN) {
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_WARN, "<%s>  drop scan due to last scan not finish\n", wnet_vif->vm_ndev->name);
        return 0;

    } else if (wnet_vif->vm_scan_hang) {
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_WARN, "<%s>  drop scan due to scan hang\n", wnet_vif->vm_ndev->name);
        return 0;

    } else if (wnet_vif->vm_recovery_state == WIFINET_RECOVERY_START) {
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_WARN, "<%s>  drop scan due to fw recovering \n", wnet_vif->vm_ndev->name);
        return 0;
    }

#ifdef CONFIG_P2P
    if ((wifimac->wm_flags & WIFINET_F_NOSCAN) && (wnet_vif->vm_p2p->p2p_enable == 0)) {
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_WARN, "not allow start scan beacon p2p_enable!!!!! \n");
        return 0;
    }
#endif//CONFIG_P2P

    wifi_mac_save_ssid(wnet_vif, ss, nssid, ssids);
    if (ss->scan_CfgFlags & WIFINET_SCANCFG_CONNECT) {
        if (wnet_vif->vm_opmode == WIFINET_M_STA) {
            if (time_before(jiffies, wifimac->wm_lastscan + SCAN_VALID_DEFAULT)) {
                if (wifi_mac_scan_before_connect(ss, wnet_vif, flags)) {
                    ss->scan_CfgFlags = 0;
                    return 0;
                }
            } else {
                wnet_vif->vm_connect_scan_entry.se_valid = 0;
            }
        }
    }

    wifimac->wm_flags |= WIFINET_F_SCAN;

    if (wnet_vif->vm_opmode == WIFINET_M_STA) {
        AML_PRINT_LOG_INFO("vm_scanchan_rssi:%d \n", wnet_vif->vm_scanchan_rssi);
        if (wnet_vif->vm_scan_before_connect_flag) {
            if (wnet_vif->vm_scanchan_rssi > MAC_MIN_GAIN) {
                wnet_vif->vm_scanchan_rssi = MAC_MIN_GAIN;
            }

        } else {
            wifi_mac_get_channel_rssi_before_scan(wifimac, &wnet_vif->vm_scanchan_rssi);
        }

        if (wnet_vif->vm_state != WIFINET_S_CONNECTED) {
            wifi_mac_scan_set_gain(wifimac, wnet_vif->vm_scanchan_rssi);
        }
    }

    ss->scan_CfgFlags |= (flags & WIFINET_SCANCFG_MASK);
    wifi_mac_pwrsave_wakeup_for_tx(wnet_vif);

    //if opmode is AP or monitor ,we not need connect bss at scan end
    if ((wnet_vif->vm_opmode != WIFINET_M_IBSS) && (wnet_vif->vm_opmode != WIFINET_M_STA)) {
        ss->scan_CfgFlags |= WIFINET_SCANCFG_NOPICK;
    }

    if (!wnet_vif->vm_scan_before_connect_flag && ((ss->VMacPriv != wnet_vif) || (ss->scan_CfgFlags & WIFINET_SCANCFG_FLUSH))) {
        wifi_mac_scan_flush(wifimac);
    }

    ss->VMacPriv = wnet_vif;

    AML_PRINT_LOG_INFO("vid:%d---> scan start, CfgFlags is:%08x, ss->ss_nssid:%d\n", wnet_vif->wnet_vif_id, ss->scan_CfgFlags, ss->ss_nssid);
    wifimac->wm_scanplayercnt++;
    wifi_mac_add_work_task(wifimac, scan_start_task, NULL, (SYS_TYPE)ss, 0, (SYS_TYPE)wifimac->wm_scanplayercnt,
        (SYS_TYPE)wnet_vif, (SYS_TYPE)wnet_vif->wnet_vif_replaycounter);

    os_timer_ex_start_period(&ss->ss_scan_timer, WIFINET_SCAN_PROTECT_TIME);
    return 1;
}

int wifi_mac_chk_scan(struct wlan_net_vif *wnet_vif, int flags,
    unsigned int nssid, const struct wifi_mac_ScanSSID ssids[])
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;

    if (wifimac->wm_flags & WIFINET_F_SCAN) {
        return 0;
    }

    if (wnet_vif->vm_opmode == WIFINET_M_STA) {
        if (wnet_vif->vm_des_nssid == 0) {
            AML_PRINT(AML_LOG_ID_CONNECT, AML_LOG_LEVEL_INFO, "scan not trigger as vm_des_bssid is empty\n");
            return 0;
        }
        if ((wnet_vif->vm_mainsta != NULL) && (wnet_vif->vm_mainsta->connect_status != CONNECT_CONNECTING)) {
            AML_PRINT(AML_LOG_ID_CONNECT,AML_LOG_LEVEL_INFO,"scan not trigger as connect_status:%d\n",wnet_vif->vm_mainsta->connect_status);
            return 0;
        }
    }

    ss->scan_CfgFlags |= WIFINET_SCANCFG_CONNECT;
    wifi_mac_scan_forbidden(wnet_vif, FORBIDDEN_SCAN_FOR_CONNECT_SCAN_TIMEOUT, FORBIDDEN_SCAN_FOR_CONNECT_SCAN);
    AML_PRINT_LOG_INFO("flags:%08x\n", flags);
    return wifi_mac_start_scan(wnet_vif, flags,  nssid, ssids);
}

void wifi_mac_scan_vattach(struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;

    wnet_vif->vm_scan_time_idle = WIFINET_SCAN_TIME_IDLE_DEFAULT;
    wnet_vif->vm_scan_time_connect = WIFINET_SCAN_TIME_CONNECT_DEFAULT;
    wnet_vif->vm_scan_time_before_connect = WIFINET_SCAN_TIME_BEFORE_CONNECT;
    wnet_vif->vm_scan_time_chan_switch = WIFINET_SCAN_TIME_CHANNEL_SWITCH;
    ss->VMacPriv = wnet_vif;
    vm_scan_setup_chan(ss,wnet_vif);
}

void wifi_mac_scan_vdetach(struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;

    if (ss->VMacPriv == wnet_vif)
    {
        if (wifimac->wm_flags & WIFINET_F_SCAN)
        {
            os_timer_ex_cancel(&ss->ss_scan_timer, CANCEL_SLEEP);
            wifimac->wm_flags &= ~WIFINET_F_SCAN;
            AML_PRINT_LOG_INFO("-->clean vm_flags 0x%x\n", wifimac->wm_flags);
        }
        wifi_mac_scan_flush(wifimac);
    }
}

void wifi_mac_scan_attach(struct wifi_mac *wifimac)
{
    struct wifi_mac_scan_state *ss;
    struct scaninfo_table *st;
    int i=0;

#ifdef CONFIG_ROKU
    wifimac->wm_roaming = WIFINET_ROAMING_DISABLE;
#else
    wifimac->wm_roaming = WIFINET_ROAMING_BASIC;
#endif
    wifimac->roaming_threshold_2g = DEFAULT_ROAMING_THRESHOLD_2G;
    wifimac->roaming_threshold_5g = DEFAULT_ROAMING_THRESHOLD_5G;
    wifimac->wm_roaming_state = ROAMING_STOP;

    AML_PRINT_LOG_INFO("wifi_mac_scan_timeout is %p\n", wifi_mac_scan_timeout);
    ss = (struct wifi_mac_scan_state *)NET_MALLOC(sizeof(struct wifi_mac_scan_state),
        GFP_KERNEL, "wifi_mac_scan_attach.ss");
    if (ss != NULL)
    {
        wifimac->wm_scan = ss;
    }
    else
    {
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_ERROR, "<ERROR> \n");
        wifimac->wm_scan = NULL;
        return ;
    }
    st = (struct scaninfo_table *)NET_MALLOC(sizeof(struct scaninfo_table),
        GFP_KERNEL, "sta_attach.st");

    if (st == NULL)
    {
        FREE(wifimac->wm_scan, "wifi_mac_scan_attach.ss");
        wifimac->wm_scan = NULL;
        AML_PRINT(AML_LOG_ID_SCAN, AML_LOG_LEVEL_ERROR, "<ERROR> \n");
        return ;
    }
    spin_lock_init(&st->st_lock);
    spin_lock_init(&ss->scan_lock);
    spin_lock_init(&ss->roaming_chan_lock);

    INIT_LIST_HEAD(&st->st_entry);
    for(i = 0; i < STA_HASHSIZE; i++)
    {
        INIT_LIST_HEAD(&st->st_hash[i]);
    }
    os_timer_ex_initialize(&ss->ss_scan_timer, 0, wifi_mac_scan_timeout_ex, ss);
    os_timer_ex_initialize(&ss->ss_forbidden_timer, 0, wifi_mac_scan_forbidden_timeout, ss);
#ifdef FW_RF_CALIBRATION
    os_timer_ex_initialize(&ss->ss_probe_timer, 0, wifi_mac_scan_send_probe_timeout_ex, ss);
#endif
    wifi_mac_scan_chk_leakap_hrtimer_attach(wifimac);

    wifimac->wm_scan->ScanTablePriv = st;
}

void wifi_mac_scan_detach(struct wifi_mac *wifimac)
{
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;

    if (ss != NULL)
    {
        AML_PRINT(AML_LOG_ID_INIT, AML_LOG_LEVEL_INFO, "<running> \n");
        os_timer_ex_del(&ss->ss_scan_timer, CANCEL_SLEEP);
        os_timer_ex_del(&ss->ss_forbidden_timer, CANCEL_SLEEP);
        wifi_mac_scan_chk_leakap_hrtimer_cancel(wifimac);

#ifdef FW_RF_CALIBRATION
        os_timer_ex_del(&ss->ss_probe_timer, CANCEL_SLEEP);
#endif

        if (ss->ScanTablePriv != NULL)
        {
            wifi_mac_scan_flush(wifimac);
            FREE(ss->ScanTablePriv,"sta_attach.st");
        }

        wifimac->wm_flags &= ~WIFINET_F_SCAN;
        AML_PRINT_LOG_INFO("-->clean vm_flags 0x%x\n", wifimac->wm_flags);

        FREE(wifimac->wm_scan,"wifi_mac_scan_attach.ss");
        wifimac->wm_scan = NULL;
    }
}

static struct cfg80211_scan_request *request = NULL;
void wifi_mac_process_tx_error(struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct vm_wdev_priv *pwdev_priv = wdev_to_priv(wnet_vif->vm_wdev);
    int flag = WIFINET_SCANCFG_ACTIVE | WIFINET_SCANCFG_NOPICK
        | WIFINET_SCANCFG_USERREQ | WIFINET_SCANCFG_FLUSH;
    int cnt = 0;

    AML_PRINT_LOG_INFO("process tx error\n");
    // make sure scan cmd download to fw and reset phy/mac
    if (wifimac->wm_flags & WIFINET_F_SCAN) {
        OS_SPIN_LOCK(&pwdev_priv->scan_req_lock);
        if (pwdev_priv->scan_request != NULL) {
            request = pwdev_priv->scan_request;
            pwdev_priv->scan_request = NULL;
        }
        OS_SPIN_UNLOCK(&pwdev_priv->scan_req_lock);

        wifi_mac_cancel_scan(wifimac);
        /* waiting for completing scan process */
        while (wifimac->wm_flags & WIFINET_F_SCAN) {
            msleep(20);
            if (cnt++ > 20) {
                AML_PRINT_LOG_ERR("<%s>:wait scan end fail when process tx error \n", wnet_vif->vm_ndev->name);
                if (request != NULL) {
                    OS_SPIN_LOCK(&pwdev_priv->scan_req_lock);
                    pwdev_priv->scan_request = request;
                    request = NULL;
                    OS_SPIN_UNLOCK(&pwdev_priv->scan_req_lock);
                }
                return;
            }
        }

        if (request != NULL) {
            OS_SPIN_LOCK(&pwdev_priv->scan_req_lock);
            pwdev_priv->scan_request = request;
            request = NULL;
            OS_SPIN_UNLOCK(&pwdev_priv->scan_req_lock);
        }

        wifi_mac_start_scan(wnet_vif, flag, wnet_vif->vm_des_nssid, wnet_vif->vm_des_ssid);
    } else {
        AML_PRINT_LOG_INFO("simulate scan start\n");
        wifimac->wm_flags |= WIFINET_F_SCAN;
        wifi_mac_scan_start(wifimac);
        msleep(5);
        wifi_mac_scan_end(wifimac);
        wifimac->wm_flags &= ~WIFINET_F_SCAN;
        AML_PRINT_LOG_INFO("simulate scan end\n");
    }
}


int wifi_mac_scan_forbidden(struct wlan_net_vif *wnet_vif, int timeout, int reason)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_mac_scan_state *ss = wifimac->wm_scan;
    static unsigned long last_overtime = 0;
    unsigned long current_overtime = jiffies + timeout / 1000 * HZ;

    if (os_timer_ex_active(&ss->ss_forbidden_timer) && time_before(current_overtime, last_overtime)) {
        AML_PRINT_LOG_INFO("Remaining forbidden time %d seconds, waste this forbidden(%d seconds)\n",
            (last_overtime - jiffies)/HZ, timeout/1000);
        return 0;
    }
    last_overtime = current_overtime;
    AML_PRINT_LOG_INFO("scan forbidden for %s, timeout is %d ms\n", forbidden_scan_reason_str[reason], timeout);
    wifimac->scan_available = 0;
    os_timer_ex_start_period(&ss->ss_forbidden_timer, timeout);
    return 1;
}

int wifi_mac_scan_access(struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct hal_private *hal_priv = hal_get_priv();

    if (wnet_vif->vm_mainsta == NULL) {
        return 0;
    }

    if (((wnet_vif->vm_state >= WIFINET_S_CONNECTING) && (wnet_vif->vm_mainsta->connect_status != CONNECT_DHCP_GET_ACK))
        || ((wifimac->wm_flags & WIFINET_F_SCAN) && (wifimac->wm_scan->scan_CfgFlags & WIFINET_SCANCFG_CONNECT))
        || (wnet_vif->vm_phase_flags & (PHASE_CONNECTING|PHASE_DISCONNECTING))
        || (wifimac->wm_recovery_flags & WIFINET_RECOVERY_F_RUNNING)
        || (wnet_vif->vm_chan_roaming_scan_flag)
        || ((aml_bus_type) && (hal_priv->dpd_suspend == 1))) {
        AML_PRINT_LOG_INFO("vm_state:%d, connect_status:%d, wm_flags:0x%08x, scan_CfgFlags:0x%08x, vm_phase_flags:0x%02x, recovery_flags:0x%x, roaming_scan_flag:%d, dpd_suspend:%d\n",
            wnet_vif->vm_state, wnet_vif->vm_mainsta->connect_status,
            wifimac->wm_flags, wifimac->wm_scan->scan_CfgFlags,
            wnet_vif->vm_phase_flags,
             wifimac->wm_recovery_flags,
            wnet_vif->vm_chan_roaming_scan_flag,
            hal_priv->dpd_suspend);

        return 0;
    }

    os_timer_ex_cancel(&wifimac->wm_scan->ss_forbidden_timer, CANCEL_NO_SLEEP);
    wifimac->scan_available = 1;
    return 1;
}


int wifi_mac_scan_forbidden_timeout(void *arg)
{
    struct wifi_mac_scan_state *ss = (struct wifi_mac_scan_state *) arg;
    struct wlan_net_vif *wnet_vif = ss->VMacPriv;
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct hal_private *hal_priv = hal_get_priv();
    struct wlan_net_vif *tmp_wnet_vif = NULL;

    AML_PRINT_LOG_INFO("vm_state:%d, connect_status:%d, wm_flags:0x%08x, scan_CfgFlags:0x%08x, vm_phase_flags:0x%02x, recovery_flags:%x, roaming_scan_flag:%d, dpd_suspend:%d\n",
        wnet_vif->vm_state, wnet_vif->vm_mainsta->connect_status,
        wifimac->wm_flags, wifimac->wm_scan->scan_CfgFlags,
        wnet_vif->vm_phase_flags,
        wifimac->wm_recovery_flags,
        wnet_vif->vm_chan_roaming_scan_flag,
        hal_priv->dpd_suspend);


    if ((wnet_vif->vm_state >= WIFINET_S_CONNECTING) && (wnet_vif->vm_mainsta->connect_status != CONNECT_DHCP_GET_ACK)) {
        wifi_mac_add_work_task(wnet_vif->vm_wmac, wifi_mac_sm_switch, NULL, (SYS_TYPE)wnet_vif, WIFINET_S_SCAN, 0, 0, 0);
    }

    if ((wifimac->wm_flags & WIFINET_F_SCAN) && (wifimac->wm_scan->scan_CfgFlags & WIFINET_SCANCFG_CONNECT)) {
        wifimac->wm_flags &= ~WIFINET_F_SCAN;
        wifimac->wm_scan->scan_CfgFlags &= ~WIFINET_SCANCFG_CONNECT;
    }

    if (wnet_vif->vm_phase_flags & (PHASE_CONNECTING|PHASE_DISCONNECTING)) {
        wnet_vif->vm_phase_flags = 0;
    }

    if (wifimac->wm_recovery_flags != 0) {
        list_for_each_entry(tmp_wnet_vif, &wifimac->wm_wnet_vifs, vm_next) {
            wifi_mac_vif_restore_end(tmp_wnet_vif);
        }
    }

    if (wnet_vif->vm_chan_roaming_scan_flag) {
        wnet_vif->vm_chan_roaming_scan_flag = 0;
    }

    if ((aml_bus_type) && (hal_priv->dpd_suspend)) {
        hal_priv->dpd_delay_cail = 0;
        hal_priv->dpd_suspend = 0;
        hal_priv->dpd_wait_pkt_clear = 0;
    }

    wifimac->scan_available = 1;
    return OS_TIMER_NOT_REARMED;
}

unsigned char wifi_mac_scan_check_available(struct wlan_net_vif *wnet_vif)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;

    AML_PRINT_LOG_INFO("scan available %d\n", wifimac->scan_available);
    return wifimac->scan_available;
}

unsigned char wifi_mac_set_scan_dwell_time(struct wifi_mac * wifimac, unsigned short duration, unsigned char mandatory)
{
    struct wlan_net_vif *wnet_vif = NULL;

    if (duration == 0) {
        return 1;
    }
    list_for_each_entry(wnet_vif, &wifimac->wm_wnet_vifs, vm_next) {
        wnet_vif->vm_scan_duration = duration;
        wnet_vif->vm_scan_duration_mandatory = mandatory;
        if (wnet_vif->wnet_vif_id == NET80211_MAIN_VMAC) {
            wifi_mac_set_scan_time(wnet_vif);
        }
    }
    return 0;
}

static inline unsigned char get_chan_flags(unsigned int flags)
{
    unsigned char chan_flags = 0;
    if (flags & IEEE80211_CHAN_NO_IR)
        chan_flags |= CHAN_NO_IR;
    if (flags & IEEE80211_CHAN_RADAR)
        chan_flags |= CHAN_RADAR;
    return chan_flags;
}


struct mac_addr mac_addr_bcst = {{0xFFFF, 0xFFFF, 0xFFFF}};

int aml_get_sched_scan_req(struct wlan_net_vif *wnet_vif, struct cfg80211_sched_scan_request *request,
    sched_scan_ptr *host_sched_req)
{
    struct scanu_start_req *req = &(host_sched_req->scanu_req);
    int i;
    unsigned char chan_flags = 0;

    host_sched_req->min_rssi_thold = request->match_sets->rssi_thold;

     /* Set parameters */
    req->vif_idx = wnet_vif->wnet_vif_id;
    req->chan_cnt = (unsigned char)min_t(int, AML_2G_CHANNELS_NUM + AML_5G_CHANNELS_NUM, request->n_channels);
    req->ssid_cnt = (unsigned char)min_t(int, SCAN_SSID_MAX, request->n_ssids);
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,12,0)
    memcpy(&req->bssid, request->match_sets->bssid, 6);
#else
    req->bssid = mac_addr_bcst;
#endif
    if (req->ssid_cnt == 0)
        chan_flags |= CHAN_NO_IR;
    for (i = 0; i < req->ssid_cnt; i++) {
        int j;
        for (j = 0; j < request->ssids[i].ssid_len; j++)
            req->ssid[i].array[j] = request->ssids[i].ssid[j];
        req->ssid[i].length = request->ssids[i].ssid_len;
    }

/*
    if (request->ie) {
        if (aml_ipc_buf_a2e_alloc(aml_hw, &aml_hw->scan_ie,
                                   request->ie_len, request->ie)) {
            netdev_err(aml_vif->ndev, "Failed to allocate IPC buf for SCAN IEs\n");
            return -ENOMEM;
        }

        req->add_ie_len = request->ie_len;
        req->add_ies = aml_hw->scan_ie.dma_addr;
    } else */
    {
        req->add_ie_len = 0;
        req->add_ies = 0;
    }

    for (i = 0; i < req->chan_cnt; i++) {
        struct ieee80211_channel *chan = request->channels[i];

        req->chan[i].band = chan->band;
        req->chan[i].freq = chan->center_freq;
        req->chan[i].flags = chan_flags | get_chan_flags(chan->flags);
        //req->chan[i].tx_power = chan_to_fw_pwr(chan->max_reg_power);
    }

    host_sched_req->n_scan_plans = (u8)min_t(int, MAX_SCHED_SCAN_PLANS,request->n_scan_plans);
    host_sched_req->match_count = (u8)min_t(int, MAX_MATCH_COUNT,request->n_match_sets);

    for (i = 0; i < host_sched_req->n_scan_plans; i++) {
        host_sched_req->scan_plans[i].interval = request->scan_plans[i].interval;
        host_sched_req->scan_plans[i].iterations = request->scan_plans[i].iterations;
    }

     for (i = 0; i < host_sched_req->match_count; i++) {
        host_sched_req->match_sets[i].rssiThreshold = request->match_sets[i].rssi_thold;

#if LINUX_VERSION_CODE > KERNEL_VERSION(4,12,0)
        memcpy(&host_sched_req->match_sets[i].bssid, request->match_sets[i].bssid, ETH_ALEN);
#endif

        host_sched_req->match_sets[i].ssId.length = request->match_sets[i].ssid.ssid_len;
        memcpy(&host_sched_req->match_sets[i].ssId.array[0],
               request->match_sets[i].ssid.ssid,
               request->match_sets[i].ssid.ssid_len);
    }

    return 0;

}

#ifdef PNO_SUPPORT

#define FMAC_CHAN_2GHZ 0x4000  /* 2.4 GHz  channel. */
#define FMAC_CHAN_5GHZ 0x8000  /* 5 GHz  channel */
unsigned short fmac_wifi_Ieee2mhz(unsigned short chan, unsigned int flags)
{
    if (flags & FMAC_CHAN_2GHZ)
    {
        if (chan == 14)
            return 2484;

        if (chan < 14)
            return 2407 + chan*5;
        else
            return 2512 + ((chan-15)*20);
    } else if (flags & FMAC_CHAN_5GHZ) {
        return 5000 + (chan*5);
    } else {
        if (chan == 14)
            return 2484;

        if (chan < 14)
            return 2407 + chan*5;

        if (chan < 27)
            return 2512 + ((chan-15)*20);

        return 5000 + (chan*5);
    }
    return 0;
}


void init_sched_param(sched_scan_ptr * sched_param)
{
    sched_scan_ptr sched_ptr;
    unsigned char i = 0;

    memset(&sched_ptr,0,sizeof(sched_scan_ptr));

    sched_ptr.delay = 5;
    sched_ptr.n_scan_plans = 1;
    sched_ptr.scan_plans[0].interval = 10;
    sched_ptr.scanu_req.duration = 100*1000;

    for (i=0; i< 13; i++) {
        sched_ptr.scanu_req.chan[i].freq = fmac_wifi_Ieee2mhz(i + 1, FMAC_CHAN_2GHZ);
        sched_ptr.scanu_req.chan_cnt ++;
    }

    sched_ptr.scanu_req.chan[i++].freq = fmac_wifi_Ieee2mhz(36, FMAC_CHAN_5GHZ);
    sched_ptr.scanu_req.chan_cnt ++;
    sched_ptr.scanu_req.chan[i++].freq = fmac_wifi_Ieee2mhz(100, FMAC_CHAN_5GHZ);
    sched_ptr.scanu_req.chan[i].flags |= CHAN_RADAR;
    sched_ptr.scanu_req.chan_cnt ++;

    sched_ptr.min_rssi_thold = 256 - 65;

    sched_ptr.match_sets[0].bssid[0] = 0xd4;
    sched_ptr.match_sets[0].bssid[1] = 0x5d;
    sched_ptr.match_sets[0].bssid[2] = 0x64;
    sched_ptr.match_sets[0].bssid[3] = 0xc6;
    sched_ptr.match_sets[0].bssid[4] = 0xcd;
    sched_ptr.match_sets[0].bssid[5] = 0xf8;
    memcpy(sched_ptr.match_sets[0].ssId.array, "wifi100=24G",strlen("wifi100=24G"));
    sched_ptr.match_sets[0].ssId.length = strlen("wifi100=24G");
    sched_ptr.match_sets[0].rssiThreshold = 256 - 60;

    sched_ptr.match_sets[1].bssid[0] = 0xd4;
    sched_ptr.match_sets[1].bssid[1] = 0x5d;
    sched_ptr.match_sets[1].bssid[2] = 0x64;
    sched_ptr.match_sets[1].bssid[3] = 0xc6;
    sched_ptr.match_sets[1].bssid[4] = 0xcd;
    sched_ptr.match_sets[1].bssid[5] = 0xfc;

    memcpy(sched_ptr.match_sets[1].ssId.array, "wifi100=5G",strlen("wifi100=5G"));
    sched_ptr.match_sets[1].ssId.length = strlen("wifi100=5G");
    sched_ptr.match_sets[1].rssiThreshold = 256 - 60;
    sched_ptr.match_count = 2;

    memcpy(sched_param, &sched_ptr, sizeof(sched_scan_ptr));

}

int aml_send_sched_scan_req(struct wlan_net_vif *wnet_vif,
    struct cfg80211_sched_scan_request *request)
{
    sched_scan_cmd sched_start_req;
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    struct hw_interface *hif = hif_get_hw_interface();
    sched_scan_ptr *host_sched_req = NULL;
    sched_start_req.vid = wnet_vif->wnet_vif_id;
    sched_start_req.Cmd = SCAN_SCH_START_CMD;
    sched_start_req.enable = 1;
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,12,0)
    //sched_start_req.reqid = request->reqid;
#endif

    sched_start_req.sched_param = (sched_scan_ptr *)(SYS_TYPE)(MAC_SRAM_BASE + SRAM_LEN + 15*1024);
    host_sched_req = (sched_scan_ptr *)kmalloc(sizeof(sched_scan_ptr), GFP_KERNEL);
/*
    if (aml_get_sched_scan_req(wnet_vif, request, host_sched_req)) {
        return -1;
    } */

    init_sched_param(host_sched_req);
    hif->hif_ops.hi_write_reg32(RG_SCFG_SRAM_FUNC, MAC_REG_BASE);
    hif->hif_ops.hi_write_sram((unsigned char*)host_sched_req, (unsigned char*)(SYS_TYPE)sched_start_req.sched_param, sizeof(sched_scan_ptr));

    HAL_BEGIN_LOCK();
    hi_set_cmd((unsigned char*)&sched_start_req, sizeof(sched_scan_cmd));
    HAL_END_LOCK();

    kfree(host_sched_req);
    wifimac->sched_scan = 1;
    printk("---aml_send_sched_scan_req-hkk\n");
    return 0;
}


int aml_send_sched_scan_stop(struct wlan_net_vif *wnet_vif,
    unsigned long long reqid)
{
    sched_scan_cmd sched_start_req;
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    sched_start_req.vid = wnet_vif->wnet_vif_id;
    sched_start_req.Cmd = SCAN_SCH_START_CMD;
    sched_start_req.reqid = reqid;
    sched_start_req.enable = 0;
    sched_start_req.sched_param = (sched_scan_ptr *)(SYS_TYPE)(MAC_SRAM_BASE + SRAM_LEN + 5*1024);

    HAL_BEGIN_LOCK();
    hi_set_cmd((unsigned char*)&sched_start_req, sizeof(sched_scan_cmd));
    HAL_END_LOCK();
    wifimac->sched_scan = 0;
    return 0;
}
#endif



