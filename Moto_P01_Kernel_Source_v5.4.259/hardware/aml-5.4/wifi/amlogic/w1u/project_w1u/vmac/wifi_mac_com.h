/*
 ****************************************************************************************
 *
 * Copyright (C) Amlogic 2010-2014
 *
 * Project: 11N 80211 mac  layer Software
 *
 * Description:
 *     wifi_mac layer common need include *.h
 *
 *
 ****************************************************************************************
 */


#ifndef _NET80211_COMMON_H_
#define _NET80211_COMMON_H_


#include <linux/version.h>
#include <linux/kmod.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <net/ieee80211_radiotap.h>
#include <linux/wireless.h>
#include <linux/etherdevice.h>
#include <linux/random.h>
#include <linux/if_vlan.h>
#include <linux/time.h>
#include <net/iw_handler.h>
#include <linux/init.h>
#include <linux/sysctl.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <asm/uaccess.h>
#include <net/inet_ecn.h>

#include "wifi_drv_config.h"
#include "wifi_debug.h"
#include "wifi_hal_com.h"
#include "wifi_mac.h"
#include "wifi_mac_encrypt.h"
#include "wifi_mac_sta.h"
#include "wifi_mac_pmf.h"
#include "wifi_mac_action.h"
#include "wifi_mac_power.h"
#include "wifi_mac_scan.h"
#include "wifi_mac_var.h"
#include "wifi_mac_p2p.h"
#include "wifi_mac_xmit.h"
#include "wifi_mac_recv.h"
#include "wifi_drv_recv.h"
#include "wifi_mac_if.h"
#include "wifi_mac_acl.h"
#include "wifi_mac_amsdu.h"
#include "wifi_cfg80211.h"
#include "wifi_mac_monitor.h"
#include "wifi_mac_rate.h"
#include "wifi_mac_chan.h"
#include "wifi_android.h"
#include "wifi_rate_ctrl.h"
#include "rc80211_minstrel_init.h"
#include "wifi_hal_txdesc.h"
#include "wifi_mac_arp.h"

#include "wifi_hal.h"
#include "wifi_mac_beacon.h"
#include "wifi_mac_concurrent.h"
#include "wifi_pkt_desc.h"
#include "rf_d_adda_xmit_reg.h"
#include "aml_regdom.h"

extern const char *wifi_mac_state_name[WIFINET_S_MAX];

#define WIFINET_DPRINTF(_m, _level, _fmt, ...) do {         \
                AML_PRINT(_m,_level, "<%s> "_fmt"\n", wnet_vif->vm_ndev->name ,##__VA_ARGS__);       \
        } while (0)
#define WIFINET_DPRINTF_MACADDR( _m, _level, _mac, _fmt, ...) do {      \
                AML_PRINT(_m, _level, "<%s> mac[%s] "_fmt"\n",wnet_vif->vm_ndev->name ,ether_sprintf(_mac),##__VA_ARGS__);  \
        } while (0)
#define WIFINET_DPRINTF_STA( _m, _level, _sta, _fmt, ...)do {           \
                AML_PRINT(_m, _level, "<%s> mac[%s] "_fmt"\n", (_sta)->sta_wnet_vif->vm_ndev->name , ether_sprintf((_sta)->sta_macaddr),##__VA_ARGS__);  \
        } while (0)

#define BOOL2STR_EFFECT(_bool) (_bool?"enable":"disable")
#define BOOL2STR_SWITCH(_bool) (_bool?"on":"off")

#define OTHERS2BOOL(_var) (!!_var)

#define TXDESC_THRESHOLD_STOP 16
#define TXDESC_THRESHOLD_WAKE 64
#define TXDESC_FLOW_CTRL_EN BIT(0)
#define TXDESC_STOP_ALL_QUEUES BIT(1)
#endif //_NET80211_COMMON_H_
