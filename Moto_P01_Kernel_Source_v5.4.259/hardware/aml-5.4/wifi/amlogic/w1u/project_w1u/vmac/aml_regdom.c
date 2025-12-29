#include "aml_regdom.h"
#include "wifi_mac_com.h"
#include "wifi_debug.h"
#include "wifi_mac_if.h"
#include "wifi_cmd_func.h"
#include "wifi_mac_chan.h"

unsigned char regdom_scheme = REGDOM_CORE_MGMT;

static const struct ieee80211_regdomain regdom_global = {
    .n_reg_rules = 7,
    .alpha2 =  "00",
    .reg_rules = {
        /* channels 1..11 */
        REG_RULE(2412-10, 2462+10, 40, 6, 20, 0),
        /* channels 12..13. */
        REG_RULE(2467-10, 2472+10, 20, 6, 20, NL80211_RRF_AUTO_BW),
        /* channel 14. only JP enables this and for 802.11b only */
        REG_RULE(2484-10, 2484+10, 20, 6, 20, NL80211_RRF_NO_OFDM),
        /* channel 36..48 */
        REG_RULE(5180-10, 5240+10, 80, 6, 20, NL80211_RRF_AUTO_BW),
        /* channel 52..64 - DFS required */
        REG_RULE(5260-10, 5320+10, 80, 6, 20, NL80211_RRF_DFS |
                NL80211_RRF_AUTO_BW),
        /* channel 100..144 - DFS required */
        REG_RULE(5500-10, 5720+10, 80, 6, 20, NL80211_RRF_DFS),
        /* channel 149..165 */
        REG_RULE(5745-10, 5825+10, 80, 6, 20, 0),
    }
};

const struct ieee80211_regdomain regdom_cn = {
    .n_reg_rules = 4,
    .alpha2 = "CN",
    .reg_rules = {
        /* channels 1..13 */
        REG_RULE(2412-10, 2472+10, 40, 6, 20, 0),
        /* channel 36..48 */
        REG_RULE(5180-10, 5240+10, 80, 6, 20, NL80211_RRF_AUTO_BW),
        /* channel 52..64 - DFS required */
        REG_RULE(5260-10, 5320+10, 80, 6, 20, NL80211_RRF_DFS |
                NL80211_RRF_AUTO_BW),
        /* channels 149..165 */
        REG_RULE(5745-10, 5825+10, 80, 6, 20, 0),
    }
};

const struct ieee80211_regdomain regdom_us = {
    .n_reg_rules = 5,
    .alpha2 = "US",
    .reg_rules = {
        /* channels 1..11 */
        REG_RULE(2412-10, 2462+10, 40, 6, 20, 0),
        /* channel 36..48 */
        REG_RULE(5180-10, 5240+10, 80, 6, 20, NL80211_RRF_AUTO_BW),
        /* channel 52..64 - DFS required */
        REG_RULE(5260-10, 5320+10, 80, 6, 20, NL80211_RRF_DFS |
                NL80211_RRF_AUTO_BW),
        /* channel 100..140 - DFS required */
        REG_RULE(5500-10, 5720+10, 80, 6, 20, NL80211_RRF_AUTO_BW),
        /* channels 149..165 */
        REG_RULE(5745-10, 5825+10, 80, 6, 20, 0),
    }
};

const struct ieee80211_regdomain regdom_jp = {
    .n_reg_rules = 5,
    .alpha2 = "JP",
    .reg_rules = {
        /* channels 1..13 */
        REG_RULE(2412-10, 2472+10, 40, 6, 20, 0),
        /* channels 14 */
        REG_RULE(2484-10, 2484+10, 20, 6, 20, NL80211_RRF_NO_OFDM),
        /* channels 36..48 */
        REG_RULE(5180-10, 5240+10, 80, 6, 20, NL80211_RRF_AUTO_BW),
        /* channels 52..64 */
        REG_RULE(5260-10, 5320+10, 80, 6, 20, NL80211_RRF_DFS | NL80211_RRF_AUTO_BW),
        /* channels 100..140 */
        REG_RULE(5500-10, 5700+10, 80, 6, 20, NL80211_RRF_DFS)
    }
};

const struct aml_regdom aml_regdom_00 = {
    .country_code = "00",
    .regdom = &regdom_global
};

const struct aml_regdom aml_regdom_cn = {
    .country_code = "CN",
    .regdom = &regdom_cn
};

const struct aml_regdom aml_regdom_us = {
    .country_code = "US",
    .regdom = &regdom_us
};

const struct aml_regdom aml_regdom_jp = {
    .country_code = "JP",
    .regdom = &regdom_jp
};

const struct aml_regdom *aml_regdom_tbl[] = {
    &aml_regdom_00,
    &aml_regdom_cn,
    &aml_regdom_us,
    &aml_regdom_jp,
    NULL
};

unsigned int aml_regdom_bw_map_channel_flag[] = {
    IEEE80211_CHAN_NO_20MHZ, IEEE80211_CHAN_NO_HT40MINUS, IEEE80211_CHAN_NO_HT40PLUS, IEEE80211_CHAN_NO_80MHZ
};

#define IS_REGDOM_BW_NOT_ALLOWED(_regdom_bw, _chan_flags) (_chan_flags & aml_regdom_bw_map_channel_flag[_regdom_bw])

extern struct class_chan_set global_chan_set[];
struct wifi_channel* aml_match_channel(unsigned char pri_chan, enum aml_regdom_bw regdom_bw)
{
    unsigned int cfreq = 0;
    unsigned char bw = 0;
    unsigned int prifreq = wifi_mac_Ieee2mhz(pri_chan, 0);

    unsigned char ext_chan = 0;
    unsigned char op_class_oft = 0;

    struct class_chan_set *chan_set = NULL;
    unsigned char chan_set_idx = 0, chan_idx = 0;

    switch (regdom_bw) {
        case REGDOM_BW_20MHZ:
            cfreq = prifreq;
            bw = 20;
        break;
        case REGDOM_BW_HT40MINUS:
            cfreq = prifreq - 10;
            bw = 40;
        break;
        case REGDOM_BW_HT40PLUS:
            cfreq = prifreq + 10;
            bw = 40;
        break;
        case REGDOM_BW_80MHZ:
            if (pri_chan < 36 || pri_chan > 161) {
                return NULL;
            }
            ext_chan = pri_chan % 4;
            op_class_oft = (pri_chan - 36 - ext_chan) % 16;
            cfreq = prifreq - op_class_oft * 5 + 30;
            bw = 80;
        break;
        default:
        break;
    }

    if (bw == 0 || cfreq == 0) {
        return NULL;
    }

    for (chan_set_idx = 0; global_chan_set[chan_set_idx].opt_idx != 0; chan_set_idx ++) {
        chan_set = global_chan_set + chan_set_idx;
        if (chan_set->bw != bw) {
            continue;
        }
        for (chan_idx = 0; chan_idx < chan_set->sub_num; chan_idx ++) {
            if (chan_set->chan_sub_set[chan_idx].chan_pri_num != pri_chan ||
                    chan_set->chan_sub_set[chan_idx].chan_cfreq1 != cfreq) {
                continue;
            }
            return &chan_set->chan_sub_set[chan_idx];
        }
    }

    return NULL;
}


int aml_resolution_chan_info(struct wiphy *wiphy, int *chan_cnt, struct wifi_channel *chan_list)
{
    struct ieee80211_supported_band *band = NULL;
    struct ieee80211_channel *chan = NULL;
    unsigned char supported_band[] = {NL80211_BAND_2GHZ, NL80211_BAND_5GHZ};
    struct wifi_channel *tmp_channel = NULL;
    unsigned char band_idx = 0;
    unsigned int chan_idx = 0;
    unsigned char reg_bw = 0;

    for (band_idx = 0; band_idx < sizeof(supported_band)/sizeof(supported_band[0]); band_idx ++) {
        band = wiphy->bands[band_idx];
        if (band == NULL) {
            continue;
        }
        for (chan_idx = 0; chan_idx < band->n_channels; chan_idx ++) {
            chan = &band->channels[chan_idx];
            if (chan->flags & IEEE80211_CHAN_DISABLED) {
                continue;
            }
            for (reg_bw = 0; reg_bw < REGDOM_BW_MAX; reg_bw ++) {
                if (IS_REGDOM_BW_NOT_ALLOWED(reg_bw, chan->flags)) {
                    continue;
                }
                if ((tmp_channel = aml_match_channel(chan->hw_value, reg_bw)) == NULL) {
                    continue;
                }
                memcpy(&chan_list[*chan_cnt], tmp_channel, sizeof(struct wifi_channel));
                if (chan->flags & IEEE80211_CHAN_RADAR) {
                    chan_list[*chan_cnt].chan_flags |= WIFINET_CHAN_DFS;
                }
                /*AML_PRINT_LOG_INFO("CHANNEL %d BW %d CFREQ %d FLAGS 0x%x",
                    chan_list[*chan_cnt].chan_pri_num, chan_list[*chan_cnt].chan_bw, chan_list[*chan_cnt].chan_cfreq1, chan_list[*chan_cnt].chan_flags);*/
                (*chan_cnt) ++;
            }
        }
    }

    return 0;
}

void aml_regdom_str2alpha(unsigned char* country_str, unsigned char* alpha2)
{
    if (country_str[0] == 'W' && country_str[1] == 'W') {
        alpha2[0] = '0';
        alpha2[1] = '0';
    } else {
        alpha2[0] = country_str[0];
        alpha2[1] = country_str[1];
    }
}

void aml_regdom_alpha2str(unsigned char* alpha2, unsigned char* country_str)
{
    if (alpha2[0] == '0' && alpha2[1] == '0') {
        country_str[0] = 'W';
        country_str[1] = 'W';
    } else {
        country_str[0] = alpha2[0];
        country_str[1] = alpha2[1];
    }
}

/* Find driver customized regdomain by country */
const struct ieee80211_regdomain *aml_get_regdom(char *alpha2)
{
    const struct aml_regdom *regdom;
    int i = 0;

    while (aml_regdom_tbl[i]) {
        regdom = aml_regdom_tbl[i];
        if ((alpha2[0] == regdom->country_code[0]) && (alpha2[1] == regdom->country_code[1])) {
            return regdom->regdom;
        }
        i++;
    }
    return &regdom_global;
}

void aml_reset_wiphy_channels(struct wiphy * wiphy)
{
    struct ieee80211_supported_band *band = NULL;
    struct ieee80211_channel *chan = NULL;
    unsigned char supported_band[] = {NL80211_BAND_2GHZ, NL80211_BAND_5GHZ};
    unsigned char band_idx = 0, chan_idx = 0;

    for (band_idx = 0; band_idx < sizeof(supported_band)/sizeof(supported_band[0]); band_idx ++) {
        band = wiphy->bands[band_idx];
        if (band == NULL) {
            continue;
        }
        for (chan_idx = 0; chan_idx < band->n_channels; chan_idx ++) {
            chan = &band->channels[chan_idx];
            chan->flags = 0;
        }
    }
}

void aml_copy_regd(const struct ieee80211_regdomain *regd, struct ieee80211_regdomain *regd_copy)
{
    int idx = 0;
    memcpy(regd_copy, regd, sizeof(struct ieee80211_regdomain));
    for (idx = 0; idx < regd->n_reg_rules; idx ++) {
        memcpy(&regd_copy->reg_rules[idx], &regd->reg_rules[idx], sizeof(regd->reg_rules[idx]));
    }
}

void aml_regd_notify(struct wiphy *wiphy, char *alpha2)
{
    const struct ieee80211_regdomain *regd = NULL;
    struct ieee80211_regdomain *regd_copy = NULL;
    unsigned int regd_len = 0;

    regd = aml_get_regdom(alpha2);
    regd_len = sizeof(struct ieee80211_regdomain) + regd->n_reg_rules * sizeof(struct ieee80211_reg_rule);
    regd_copy = (struct ieee80211_regdomain *)ZMALLOC(regd_len, "regd_copy", GFP_KERNEL);
    if (!regd_copy) {
        AML_PRINT_LOG_ERR("failed to alloc regd_copy\n");
        goto exit;
    }

    aml_copy_regd(regd, regd_copy);
    if (memcmp(regd_copy->alpha2, alpha2, 2)) {
        AML_PRINT_LOG_INFO("country %s regdomain not found, now use %s as instead\n", alpha2, regd_copy->alpha2);
        memcpy(regd_copy->alpha2, alpha2, 2);
    }

    if (IS_REGD_CUST_BYDB()) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
        if (rtnl_is_locked()) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0))
            wiphy_lock(wiphy);
            regulatory_set_wiphy_regd_sync(wiphy, regd_copy);
            wiphy_unlock(wiphy);
#else
            regulatory_set_wiphy_regd_sync_rtnl(wiphy, regd_copy);
#endif /* LINUX_VERSION > 5.12.0 */
        } else {
            regulatory_set_wiphy_regd(wiphy, regd_copy);
        }
#else
        wiphy_apply_custom_regulatory(wiphy, regd_copy);
#endif /* LINUX_VERSION > 4.0.0 */
        AML_PRINT_LOG_INFO("sync wiphy regd done %s\n", wiphy->regd->alpha2);
    } else if (IS_REGD_CORE_MGMT()) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
        wiphy->regulatory_flags |= REGULATORY_CUSTOM_REG;
#else
        wiphy->regulatory_flags |= WIPHY_FLAG_CUSTOM_REGULATORY;
#endif
        wiphy_apply_custom_regulatory(wiphy, regd_copy);
    }

exit:
    FREE(regd_copy, "regd_copy");
    return;
}

void aml_apply_country(struct wiphy *wiphy, unsigned char* alpha2, unsigned char custom)
{
    struct wlan_net_vif *wnet_vif  = wiphy_to_adapter(wiphy);
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    //const struct ieee80211_regdomain *regdom = NULL;
    unsigned char country[3] = {'\0'};//Driver country code
    unsigned char copy_alpha2[3] = {'\0'};
    int ret = 0;

    aml_regdom_alpha2str(alpha2, country);
    aml_regdom_str2alpha(country, copy_alpha2);

    if (IS_REGD_USE_DB()) {
        if (custom) {
            aml_reset_wiphy_channels(wiphy);
            aml_regd_notify(wiphy, copy_alpha2);
        }
    }

    WIFI_ALPHA_LOCK(wifimac);

    if (IS_REGD_USE_DB()) {
        WIFI_NEW_CHANNEL_LOCK(wifimac);
        wifimac->wm_new_nchans = 0;
        memset(wifimac->wm_new_channels, 0x00, sizeof(wifimac->wm_new_channels));
        ret = aml_resolution_chan_info(wiphy, &(wifimac->wm_new_nchans), wifimac->wm_new_channels);
        WIFI_NEW_CHANNEL_UNLOCK(wifimac);
    }

    wifimac->wm_alpha_target[0] = country[0];
    wifimac->wm_alpha_target[1] = country[1];
    wifimac->wm_alpha_target[2] = '\0';

    if (wifimac->wm_alpha_set_forbid) {
        wifimac->wm_alpha_pending = 1;
        AML_PRINT_LOG_INFO("country switch forbidden, save pending country [%s]\n", country);
        WIFI_ALPHA_UNLOCK(wifimac);
        return;
    } else {
        wifimac->wm_alpha_set_in_progress = 1;
    }
    WIFI_ALPHA_UNLOCK(wifimac);

    wifi_mac_set_country_regdom(country);
}

static void aml_reg_notifier(struct wiphy *wiphy,
               struct regulatory_request *request)
{
    struct wlan_net_vif *wnet_vif = wiphy_to_adapter(wiphy);
    unsigned char source_code[5][10] = {"core", "user", "driver", "countryie"};
    unsigned char custom = IS_REGD_CUST_BYDB();

    if (!request)
        return;

    AML_PRINT_LOG_INFO("vid[%d] regdom set by %s: country <%s> \n", wnet_vif->wnet_vif_id, source_code[request->initiator], request->alpha2);

    AML_PRINT_LOG_INFO("wiphy alpha2 %s\n", wiphy->regd->alpha2);

    switch (request->initiator) {
        case NL80211_REGDOM_SET_BY_CORE:
            aml_apply_country(wiphy, request->alpha2, custom);
            break;
        case NL80211_REGDOM_SET_BY_DRIVER:
            /*
            * restore the driver regulatory flags since
            * regulatory_hint may have
            * changed them
            */
            aml_apply_country(wiphy, request->alpha2, custom);
            wiphy->regulatory_flags = wnet_vif->regulatory_flags;
            break;
        case NL80211_REGDOM_SET_BY_USER:
        case NL80211_REGDOM_SET_BY_COUNTRY_IE:
            if (wnet_vif->wnet_vif_id == NET80211_MAIN_VMAC) {
                aml_apply_country(wiphy, request->alpha2, custom);
                if (!IS_REGD_CUST_BYDB()) {
                    regulatory_hint(wnet_vif->vm_wdev->wiphy, request->alpha2);
                }
            }
            break;
    }
}

void aml_regd_init(struct wiphy *wiphy)
{
    struct wlan_net_vif *wnet_vif = wiphy_to_adapter(wiphy);
    char* alpha2 = NULL;

    if (wnet_vif->wnet_vif_id != NET80211_MAIN_VMAC) {
        return;
    }

    if (regdom_scheme >= REGDOM_SCHEME_MAX) {
        regdom_scheme = REGDOM_CUST_DRVDEF;
    }

    AML_PRINT_LOG_INFO("regdom scheme = %d\n", regdom_scheme);

    alpha2 = aml_wifi_get_country_code();

    wiphy->reg_notifier = aml_reg_notifier;

    /*
    *  To support GO working on DFS channel,
    *  enable REGULATORY_IGNORE_STALE_KICKOFF flag.
    *  It will not handle kernel regdomain change disconnect
    */

#if LINUX_VERSION_CODE <= KERNEL_VERSION(6, 3, 12)
    wiphy->regulatory_flags |= REGULATORY_IGNORE_STALE_KICKOFF;
#else
    wiphy->regulatory_flags |= (REGULATORY_WIPHY_SELF_MANAGED >> 1);
#endif

    if (IS_REGD_CORE_MGMT()) {
        wiphy->regulatory_flags |= REGULATORY_STRICT_REG;
    } else if (IS_REGD_CUST_BYDB()) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
        wiphy->regulatory_flags |= REGULATORY_WIPHY_SELF_MANAGED;
        return;
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
        wiphy->regulatory_flags |= REGULATORY_CUSTOM_REG;
#else
        wiphy->flags |= WIPHY_FLAG_CUSTOM_REGULATORY;
#endif
    }
    aml_apply_country(wiphy, alpha2, 1);
    wnet_vif->regulatory_flags = wiphy->regulatory_flags;
}

void aml_regd_cust(struct wiphy* wiphy)
{
    struct wlan_net_vif *wnet_vif = wiphy_to_adapter(wiphy);

    if (wnet_vif->wnet_vif_id != NET80211_MAIN_VMAC) {
        return;
    }

    if (!IS_REGDOM_SELF_MANAGED(wiphy)) {
        return;
    }
    AML_PRINT_LOG_INFO("custom regd\n");
    if (wiphy->regd == NULL) {
        rtnl_lock();
        aml_apply_country(wiphy, aml_wifi_get_country_code(), 1);
        rtnl_unlock();
    }
    wnet_vif->regulatory_flags = wiphy->regulatory_flags;

}

void wifi_mac_set_country_code(char* arg)
{
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    struct wlan_net_vif* wnet_vif = wifi_mac_get_wnet_vif_by_vid(wifimac, NET80211_MAIN_VMAC);
    unsigned char alpha2[3] = {'\0'};

    alpha2[0] = arg[0];
    alpha2[1] = arg[1];

    AML_PRINT_LOG_INFO("set country code to [%s]\n", alpha2);
    if (IS_REGD_CORE_MGMT()) {
        regulatory_hint(wnet_vif->vm_wdev->wiphy, alpha2);
    } else if (IS_REGD_CUST()) {
        aml_apply_country(wnet_vif->vm_wdev->wiphy, alpha2, 1);
    } else {
        AML_PRINT_LOG_ERR("error regdomain scheme: %d\n", regdom_scheme);
    }
}

