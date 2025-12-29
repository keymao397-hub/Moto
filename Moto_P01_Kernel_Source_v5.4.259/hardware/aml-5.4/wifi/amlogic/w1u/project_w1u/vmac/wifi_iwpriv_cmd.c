#include "wifi_mac_com.h"
#include "wifi_iwpriv_cmd.h"
#include "wifi_cmd_func.h"
#include "chip_intf_reg.h"
#include <net/cfg80211.h>
#include "version.h"
#include "wifi_drv_capture.h"
#include "wifi_csi.h"
#include "wifi_common.h"
#include "wifi_debug_key.h"


extern void print_driver_version(void);
extern char **aml_cmd_char_parse(char sep, const char *str, int *size);
extern struct udp_info aml_udp_info[];
extern struct udp_timer aml_udp_timer;
extern int udp_cnt;
extern struct WF2G_Txpwr_Param g_wf2g_txpwr_param;
extern struct WF5G_Txpwr_Param g_wf5g_txpwr_param;
extern Efuse_Cfg_Param efuse_cfg_param;
extern void wifi_mac_switch_ant(void* arg);
extern unsigned char wifi_mac_get_cur_ant(void);
extern void wifi_mac_reset_ant_measure(void);
extern unsigned char wifi_mac_sync_using_ant(unsigned char ant);


#if LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0)
extern int vm_cfg80211_set_bitrate_mask(struct wiphy *wiphy, struct net_device *dev,
    const unsigned char *peer, const struct cfg80211_bitrate_mask *mask);
#else
extern int vm_cfg80211_set_bitrate_mask(struct wiphy *wiphy, struct net_device *dev,
                unsigned int link_id, const unsigned char *peer,
                const struct cfg80211_bitrate_mask *mask);
#endif


void wifi_mac_pwrsave_set_inactime(struct wlan_net_vif *wnet_vif, unsigned int time);

unsigned char g_iwpriv_get_spec_regs_flag = 0;

struct wlan_net_vif *aml_iwpriv_get_vif(char *name)
{
    struct wlan_net_vif *wnet_vif = NULL;
    struct drv_private *drv_priv = NULL;
    struct wifi_mac *wifimac = NULL;
    int idx = 0;

    wifimac = wifi_mac_get_mac_handle();
    drv_priv = wifimac->drv_priv;

    for (idx = 0; idx < 2; idx++) {
        wnet_vif = drv_priv->drv_wnet_vif_table[idx];
        if (strncmp(wnet_vif->vm_ndev->name, name, sizeof(wnet_vif->vm_ndev->name)) == 0) {
            AML_PRINT_LOG_INFO("%s, %s\n", wnet_vif->vm_ndev->name, name);
            return wnet_vif;
        }
    }
    return NULL;
}


static int aml_ap_set_amsdu_state(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
    AML_PRINT_LOG_INFO("%s\n",extra);
    aml_set_mac_amsdu_switch_state(extra);

    return 0;
}

static int aml_ap_set_ampdu_state(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
    AML_PRINT_LOG_INFO("%s\n", extra);
    aml_set_drv_ampdu_switch_state(extra);

    return 0;
}


static int aml_ap_get_amsdu_state(void)
{
    AML_PRINT_LOG_INFO("\n");
    aml_set_mac_amsdu_switch_state(NULL);
    return 0;
}

static int aml_ap_get_ampdu_state(void)
{
    AML_PRINT_LOG_INFO("\n");
    aml_set_drv_ampdu_switch_state(NULL);
    return 0;
}

static int aml_ap_set_11h(unsigned char channel)
{
    wifi_mac_ap_set_11h(channel);

    return 0;
}


static int aml_ap_send_addba_req(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{

    int *param = (int *)extra;
    int val = 0;
    char addr[MAX_MAC_BUF_LEN]={0};

    AML_PRINT_LOG_INFO("%d, %d, %d, %d, %d, %d, %d\n",
            param[0], param[1], param[2],param[3],param[4],param[5],param[6]);

    snprintf(addr, MAX_MAC_BUF_LEN, "%02d:%02d:%02d:%02d:%02d:%02d",param[0],param[1],param[2],param[3],param[4],param[5]);
    val = param[6];
    wifi_mac_send_addba_req(addr, val);

    return 0;

}

unsigned int get_reg(struct wlan_net_vif *wnet_vif, unsigned int set)
{

    unsigned int usr_data = 0;
    unsigned int reg_val = 0;

    usr_data = set;
    AML_PRINT_LOG_INFO("Reg addr: 0x%08x\n", usr_data);

    if (((usr_data >> 24) & 0xff) == 0xff ) {
#ifdef USE_T902X_RF
        reg_val = rf_i2c_read(usr_data & 0x00ffffff);
#endif
    } else if (((usr_data >> 24) & 0xf0) == 0xf0 ) {
#ifdef USE_T902X_RF
        reg_val = rf_i2c_read(usr_data & 0xffffffff);
#endif
    } else {
        reg_val = wnet_vif->vif_ops.read_word(usr_data);
    }

    AML_PRINT_LOG_INFO("Drv info: Reg data=&0x%08x\n",reg_val);

    return reg_val;
}


unsigned int set_reg(struct wlan_net_vif *wnet_vif, unsigned int set1, unsigned int set2)
{

    unsigned int usr_data = 0;
    unsigned int usr_data_ext = 0;
    usr_data = set1;
    usr_data_ext = set2;

    AML_PRINT_LOG_INFO("Cfg80211: Reg addr: val:0x%08x,val:0x%08x\n",usr_data,usr_data_ext);
    if (((usr_data >> 24) & 0xff) == 0xff) {
#ifdef USE_T902X_RF
        rf_i2c_write( usr_data & 0x00ffffff,usr_data_ext );//access t902x rf reg
#endif
    } else if (((usr_data >> 24) & 0xf0) == 0xf0) {
#ifdef USE_T902X_RF
        rf_i2c_write( usr_data & 0xffffffff,usr_data_ext );//access t902x rf reg
#endif
    } else {
        wnet_vif->vif_ops.write_word(usr_data , usr_data_ext);
    }

    return 0;
}

reg_addr_attr_t reg_addr_attr[DUMP_REG_SEQ_MAX] =
{
    {RF_TOP_ADDR_START, RF_TOP_ADDR_END, RF_TOP_ADDR_NUM},
    {RF_SX_ADDR_START, RF_SX_ADDR_EDN, RF_SX_ADDR_NUM},
    {RF_TX_ADDR_START, RF_TX_ADDR_END, RF_TX_ADDR_NUM},
    {RF_RX_ADDR_START, RF_RX_ADDR_END, RF_RX_ADDR_NUM},
    {ADDA_CORE_ADDR_START, ADDA_CORE_ADDR_END, ADDA_CORE_ADDR_NUM},
    {ADDA_XMIT_ADDR_START, ADDA_XMIT_ADDR_END, ADDA_XMIT_ADDR_NUM},
    {ADDA_RECV_ADDR_START, ADDA_RECV_ADDR_END, ADDA_RECV_ADDR_NUM},
    {ADDA_ESTI_ADDR_START, ADDA_ESTI_ADDR_END, ADDA_ESTI_ADDR_NUM},
    {AGC_ADDR_START, AGC_ADDR_END, AGC_ADDR_NUM},
    {OFDM_ADDR_START, OFDM_ADDR_END, OFDM_ADDR_NUM},
    {PHY_ADDR_START, PHY_ADDR_END, PHY_ADDR_NUM},
    {AON_REG_ADDR_START, AON_REG_ADDR_END, AON_REG_ADDR_NUM},
};

void dump_spec_regs_val(struct wlan_net_vif *wnet_vif, int reg_domain)
{
    int i, j, reg_addr, reg_val;
    for (i = 0; i <= reg_domain; i++)
    {
        for (j = 0; j < reg_addr_attr[i].num; j++)
        {
            reg_addr = reg_addr_attr[i].addr_start + j*4;
            if (((reg_addr >> 24) & 0xff) == 0xff )
            {
#ifdef USE_T902X_RF
                reg_val = rf_i2c_read(reg_addr & 0x00ffffff);
#endif
            }
            else if (((reg_addr >> 24) & 0xf0) == 0xf0 )
            {
#ifdef USE_T902X_RF
                reg_val = rf_i2c_read(reg_addr & 0xffffffff);
#endif
            }
            else
            {
                reg_val = wnet_vif->vif_ops.read_word(reg_addr);
            }
            printk("reg_addr:0x%08x, reg_val:0x%08x\n", reg_addr, reg_val);
        }
    }
}

void aml_iwpriv_set_cf_end(struct wlan_net_vif *wnet_vif, unsigned char is_enable)
{
    AML_PRINT_LOG_INFO("vid:%d, enable:%d\n", wnet_vif->wnet_vif_id, is_enable);

    wifi_mac_set_cf_end(wnet_vif, is_enable);
}

void aml_iwpriv_set_flow_ctrl(struct wlan_net_vif *wnet_vif, unsigned char is_enable)
{
    struct wifi_mac * wifimac = wnet_vif->vm_wmac;

    TX_DESC_BUF_LOCK(wifimac);
    if (is_enable)
    {
        wifimac->txdesc_flag |= TXDESC_FLOW_CTRL_EN;
    }
    else if (!is_enable && !(wifimac->txdesc_flag & TXDESC_STOP_ALL_QUEUES))
    {
        wifimac->txdesc_flag &= ~TXDESC_FLOW_CTRL_EN;
    }
    else if (!is_enable && (wifimac->txdesc_flag & TXDESC_STOP_ALL_QUEUES))
    {
        AML_PRINT_LOG_INFO("wait wake queue, don't disable, please retry!!!\n");
    }
    TX_DESC_BUF_UNLOCK(wifimac);

    AML_PRINT_LOG_INFO("vid:%d, enable:%d, txdesc_flag:0x%x\n", wnet_vif->wnet_vif_id, is_enable, wifimac->txdesc_flag);
}

void aml_iwpriv_set_wfa_mode(struct wlan_net_vif *wnet_vif, unsigned char is_enable)
{
    struct wifi_mac * wifimac = wnet_vif->vm_wmac;
    wifimac->wm_wfa_enable = is_enable;
    AML_PRINT_LOG_INFO("vid:%d, enable_wfa:%d\n", wnet_vif->wnet_vif_id, wifimac->wm_wfa_enable);
}

#ifdef CONFIG_ROKU
char *sta_capt_statics_path = "/nvram/rate_statistics.txt";
#else
char *sta_capt_statics_path = "/vendor/etc/rate_statistics.txt";
#endif

void aml_get_rate_idx(unsigned char rate, unsigned char *name) {
    unsigned char *out = name;

    if (IS_HT_RATE(rate))
        out += sprintf(out, "11N_MCS%d", GET_HT_MCS(rate));
    else if (IS_VHT_RATE((rate)))
        out += sprintf(out, "11AC_MCS%d", GET_VHT_MCS(rate));
    else {
        switch (rate) {
            case WIFI_11B_1M:
                out += sprintf(out, "11B_1M");
                break;
            case WIFI_11B_2M:
                out += sprintf(out, "11B_2M");
                break;
            case WIFI_11B_5M:
                out += sprintf(out, "11B_5M");
                break;
            case WIFI_11B_11M:
                out += sprintf(out, "11B_11M");
                break;
            case WIFI_11G_6M:
                out += sprintf(out, "11G_6M");
                break;
            case WIFI_11G_9M:
                out += sprintf(out, "11G_9M");
                break;
            case WIFI_11G_12M:
                out += sprintf(out, "11G_12M");
                break;
            case WIFI_11G_18M:
                out += sprintf(out, "11G_18M");
                break;
            case WIFI_11G_24M:
                out += sprintf(out, "11G_24M");
                break;
            case WIFI_11G_36M:
                out += sprintf(out, "11G_36M");
                break;
            case WIFI_11G_48M:
                out += sprintf(out, "11G_48M");
                break;
            case WIFI_11G_54M:
                out += sprintf(out, "11G_54M");
                break;
        }
    }
}

void aml_get_rate_bw(unsigned char bw, unsigned char *name) {
    unsigned char *out = name;

    if (bw == BW_20)
        out += sprintf(out, "20");
    else if (bw == BW_40)
        out += sprintf(out, "40");
    else if(bw == BW_80)
        out += sprintf(out, "80");
    else
        AML_PRINT_LOG_ERR("not support bw %d \n", bw);
}

void aml_get_rate_gi(unsigned char gi, unsigned char *name) {
    unsigned char *out = name;

    if (gi == 1)
        out += sprintf(out, "short");
    else if(gi == 0)
        out += sprintf(out, "long");
    else
        AML_PRINT_LOG_ERR("not support gi %d \n", gi);
}

int aml_get_rvr_info(struct wlan_net_vif *wnet_vif, unsigned char *buf, unsigned char write_type)
{
    unsigned char tmp_buf_bw[10];
    unsigned char tmp_buf_gi[10];
    unsigned char tmp_buf_rate[10];
    unsigned int  signal_noise_info[8] = {0};

    struct drv_private *drv_priv = drv_get_drv_priv();
    struct hw_interface* hif = hif_get_hw_interface();

    if (!wnet_vif) {
        AML_PRINT_LOG_ERR("wnet_vif is null!\n");
        return -EFAULT;
    }

    if (wnet_vif->vm_state != WIFINET_S_CONNECTED) {
        AML_PRINT_LOG_ERR("vm_state is not in connection!\n");
        return -EFAULT;
    }

    if (wnet_vif->vm_mainsta == NULL) {
        AML_PRINT_LOG_ERR("vm_mainsta is null!\n");
        return -EFAULT;
    }

    if (write_type > WRITE_FILE_NODE) {
        AML_PRINT_LOG_ERR("error write type!\n");
        return -EFAULT;
    }

    get_phy_stc_info(signal_noise_info);

    if (write_type == WRITE_FILE) {
        sprintf(buf, "sta_avg_rssi:%d, sta_avg_bcn_rssi:%d, avg_snr:%d, avg_noise:%d, ",
                wnet_vif->vm_mainsta->sta_avg_rssi - 256,
                wnet_vif->vm_mainsta->sta_avg_bcn_rssi,
                wnet_vif->vm_mainsta->sta_avg_snr,
                (wnet_vif->vm_mainsta->sta_avg_rssi - 256 - wnet_vif->vm_mainsta->sta_avg_snr));
    } else if (write_type == WRITE_FILE_NODE) {
        sprintf(buf, "sta_avg_rssi:%d \nsta_avg_bcn_rssi:%d \navg_snr:%d \navg_noise:%d \n",
                wnet_vif->vm_mainsta->sta_avg_rssi - 256,
                wnet_vif->vm_mainsta->sta_avg_bcn_rssi,
                wnet_vif->vm_mainsta->sta_avg_snr,
                (wnet_vif->vm_mainsta->sta_avg_rssi - 256 - wnet_vif->vm_mainsta->sta_avg_snr));
    }

    aml_get_rate_idx(wnet_vif->vm_mainsta->sta_vendor_rate_code, tmp_buf_rate);
    aml_get_rate_bw(wnet_vif->vm_mainsta->sta_vendor_bw, tmp_buf_bw);
    aml_get_rate_gi(wnet_vif->vm_mainsta->sta_vendor_gi, tmp_buf_gi);
    if (write_type == WRITE_FILE) {
        sprintf(buf + strlen(buf), "txRate:%s, tx_bw:%s, tx_gi:%s, gbpps:%d, ",tmp_buf_rate, tmp_buf_bw, tmp_buf_gi, hif->HiStatus.avg_tx_fail_num);
    } else if (write_type == WRITE_FILE_NODE) {
        sprintf(buf + strlen(buf), "txRate:%s \ntx_bw:%s \ntx_gi:%s \ngbpps:%d \n",tmp_buf_rate, tmp_buf_bw, tmp_buf_gi, hif->HiStatus.avg_tx_fail_num);
    }

    memset(tmp_buf_rate, 0, sizeof(tmp_buf_rate));
    memset(tmp_buf_bw, 0, sizeof(tmp_buf_bw));
    memset(tmp_buf_gi, 0, sizeof(tmp_buf_gi));
    aml_get_rate_idx(drv_priv->drv_currratetable->info[wnet_vif->vm_mainsta->sta_rxrate_index].vendor_rate_code, tmp_buf_rate);
    aml_get_rate_bw(wnet_vif->vm_mainsta->last_rxrate_bw, tmp_buf_bw);
    aml_get_rate_gi(wnet_vif->vm_mainsta->last_rxrate_gi, tmp_buf_gi);
    if (write_type == WRITE_FILE) {
        sprintf(buf + strlen(buf), "rxRate:%s, rx_bw:%s, rx_gi:%s\n",tmp_buf_rate, tmp_buf_bw, tmp_buf_gi);
    } else if (write_type == WRITE_FILE_NODE) {
        sprintf(buf + strlen(buf), "rxRate:%s \nrx_bw:%s \nrx_gi:%s \n",tmp_buf_rate, tmp_buf_bw, tmp_buf_gi);
    }

    return strlen(buf);
}
#define CAPT_MAX_FILE_SIZE        1024 * 1024 * 2
void aml_save_sta_capture_statics(struct wlan_net_vif *wnet_vif)
{
#if defined (LINUX_PLATFORM)
    struct file *fp;
    unsigned int offset = 0;
    long file_size = -1;
    unsigned int offset_start = 0;
    char buf[512] = {0};
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
    struct kstat stat;
    mm_segment_t fs;
    int error = 0;
#endif
#endif

    if (wnet_vif->vm_state != WIFINET_S_CONNECTED) {
        AML_PRINT_LOG_ERR("not connect!!!\n");
        return;
    }
#if defined (LINUX_PLATFORM)
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
    fs = get_fs();
    set_fs(KERNEL_DS);
#endif
    fp = filp_open(sta_capt_statics_path, O_CREAT|O_RDWR, 0644);
    if (!IS_ERR(fp)) {
        offset = vfs_llseek(fp, 0, SEEK_END);
        offset_start = vfs_llseek(fp, 0, SEEK_SET);
        if (offset < 0 || offset_start < 0) {
            filp_close(fp, NULL);
            AML_PRINT_LOG_ERR("Failed to seek pos of file\n");
            return ;
        }
        file_size = offset - offset_start;

        if (file_size > CAPT_MAX_FILE_SIZE) {
            vfs_truncate(&fp->f_path, 0);
        } else {
            fp->f_pos = offset;
        }

        if (wnet_vif->vm_mainsta != NULL) {

            if (aml_get_rvr_info(wnet_vif, buf, WRITE_FILE) < 0) {
                filp_close(fp, NULL);
                goto err;
            }
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
            error = vfs_stat(sta_capt_statics_path, &stat);
            if (error) {
                filp_close(fp, NULL);
                goto err;
            }
            fp->f_pos = (int)stat.size;
            if ((int)stat.size < 0) {
                filp_close(fp, NULL);
                goto err;
            }
            vfs_write(fp, buf, strlen(buf), &fp->f_pos);
#else
            kernel_write(fp, buf, strlen(buf), &fp->f_pos);
#endif
            }
            filp_close(fp, NULL);
    }else {
        AML_PRINT_LOG_ERR("open file %s failed.\n", sta_capt_statics_path);
    }
err:
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
    set_fs(fs);
#else
    AML_PRINT_LOG_ERR("File operation err exit\n");
#endif
#else
        AML_PRINT_LOG_ERR("Android forbid open and write files\n");
#endif
}

int aml_iwpriv_get_spec_regs(struct wlan_net_vif *wnet_vif, int addr_range)
{
    int reg_domain = -1;

    AML_PRINT_LOG_INFO("addr_range:%d \n", addr_range);

    switch (addr_range) {
        case 1:
            reg_domain = RF_RX_SEQ;
            break;
        case 2:
            reg_domain = ADDA_ESTI_SEQ;
            break;
        case 3:
            reg_domain = PHY_SEQ;
            break;
        case 4:
            reg_domain = AON_SEQ;
            break;
        case 5:
            g_iwpriv_get_spec_regs_flag = 1;
            break;
        case 6:
            g_iwpriv_get_spec_regs_flag = 0;
            break;
        default:
            reg_domain = -1;
            AML_PRINT_LOG_INFO("input para invalid, addr_range:%d \n", addr_range);
    }

    dump_spec_regs_val(wnet_vif, reg_domain);

    return 0;
}

void aml_iwpriv_set_pt_calibration(unsigned int channel, unsigned char bw)
{
    struct hw_interface *hif = hif_get_hw_interface();
    struct pt_cali_bits *ptbits = NULL;
    unsigned int cali = 0;
    unsigned int bw_reg = 0;

    if (!aml_wifi_is_enable_rf_test()) {
        AML_PRINT_LOG_INFO("Now not in pt mode, exit\n");
        return;
    }

    if (channel < 0 || (channel > 14 && channel < 36) || channel > 165) {
        AML_PRINT_LOG_INFO("Calibration error channel!\n");
        return;
    } else if (bw != 20 && bw != 40 && bw != 80) {
        AML_PRINT_LOG_INFO("Calibration error bw!\n");
        return;
    }

    //20MHz:0 40MHz:1 80MHz:2
    gB2BTestCasePacket.channel_bw = (bw == 20 ? 0 : (bw == 40 ? 1 : 2));
    gB2BTestCasePacket.channel = channel;

    ptbits = (struct pt_cali_bits *)&cali;
    ptbits->link_flag = 1;
    ptbits->band = gB2BTestCasePacket.channel < 36 ? 0 : 1;
    ptbits->bw = gB2BTestCasePacket.channel_bw;
    ptbits->channel = gB2BTestCasePacket.channel;
    hif->hif_ops.hi_write_word(0x00a100f4, cali);

    bw_reg = BIT(31) | gB2BTestCasePacket.channel_bw << 28 | (gB2BTestCasePacket.channel_bw == 0 ? 0 : BIT(24));
    hif->hif_ops.hi_write_word(0x00a0b22c, bw_reg);

    AML_PRINT_LOG_INFO("PT tx calibration: 0x%x, band: %s, channel: %d, bw: %d0MHz\n",
        cali, ptbits->band ? "5GHz" : "2.4GHz", ptbits->channel, BIT(ptbits->bw + 1));

}

void aml_iwpriv_set_tx_path(struct wlan_net_vif *wnet_vif, unsigned int antenna, unsigned int channel)
{
    AML_PRINT_LOG_INFO("antenna = %d, channel = %d\n", antenna, channel);

    if ((channel > 14 && channel < 36) || channel > 165) {
        AML_PRINT_LOG_INFO("error channel!\n");
        return;
    }

    if (antenna != 0) {
        AML_PRINT_LOG_INFO("W1u just support one antenna, set antenna to 0!\n");
        antenna = 0;
    }

    if (aml_wifi_is_enable_rf_test()) {
        AML_PRINT_LOG_INFO("<PT> Set tx path to : antenna %d, channel %d\n", antenna, channel);
    }
}

void aml_iwpriv_set_tx_bw(unsigned int bw)
{
    AML_PRINT_LOG_INFO("bw = %d\n", bw);
    if (bw > 2) {
        AML_PRINT_LOG_INFO("Set TX BW Error: not support bw %d!\n", bw);
        AML_PRINT_LOG_INFO("  [0]:20MHz, [1]:40MHz, [2]:80MHz\n");
        return;
    }

    if (aml_wifi_is_enable_rf_test()) {
        AML_PRINT_LOG_INFO("<PT> Set bw to %d0M\n", BIT(bw + 1));
    }
}

static unsigned int pt_tx_mode = 0;
void aml_iwpriv_set_tx_mode(unsigned int mode)
{
    AML_PRINT_LOG_INFO("mode = %d\n", mode);
    if (mode > 3) {
        AML_PRINT_LOG_INFO("Now just support modes [0]:11b, [1]:11g, [2]:11n, [3]:11ac, input %d\n", mode);
        return;
    }
    if (aml_wifi_is_enable_rf_test()) {
        switch (mode) {
            case 0:
                pt_tx_mode = 0;
                break;
            case 1:
                pt_tx_mode = 0;
                break;
            case 2:
                pt_tx_mode = WIFI_11N_MASK;
                break;
            case 3:
                pt_tx_mode = WIFI_11AC_MASK;
                break;
        }
        AML_PRINT_LOG_INFO("<PT> Set mode to %d\n", mode);
    }
}

int legacy_mbps_rate2mask(unsigned int datarate)
{
    unsigned int map_list[] = {
            /* 11b rate mask */ WIFI_11B_1M, WIFI_11B_2M, WIFI_11B_5M, WIFI_11B_11M,
            /* 11g rate mask */ WIFI_11G_6M, WIFI_11G_9M, WIFI_11G_12M, WIFI_11G_18M, WIFI_11G_24M, WIFI_11G_36M, WIFI_11G_48M, WIFI_11G_54M
        };
    unsigned int datarate_list[] = {
            /* 11b rate mbps */ 1, 2, 5, 11,
            /* 11g rate mbps */ 6, 9, 12, 18, 24, 36, 48, 54
    };
    int map_index = 0;

    for (map_index = 0; map_index < sizeof(map_list)/sizeof(map_list[0]); map_index ++) {
        if (datarate_list[map_index] == datarate) {
            return map_list[map_index];
        }
    }
    return -1;
}

void aml_iwpriv_set_tx_rate(unsigned int rate)
{
    int map_rate = -1;

    AML_PRINT_LOG_INFO("rate = %d\n", rate);

    if (aml_wifi_is_enable_rf_test()) {
        switch (pt_tx_mode) {
            case 0:
                if (rate > 54) {
                    break;
                }
                map_rate = legacy_mbps_rate2mask(rate);
                break;
            case WIFI_11N_MASK:
                if (rate > WIFI_11N_MAX) {
                    break;
                }
                map_rate = WIFI_11N_MASK | rate;
                break;
            case WIFI_11AC_MASK:
                if (rate > WIFI_11AC_MAX) {
                    break;
                }
                map_rate = WIFI_11AC_MASK | rate;
                break;
        }
        if (map_rate == -1) {
            AML_PRINT_LOG_INFO("Set tx rate fail\n");
            AML_PRINT_LOG_INFO("  11b: [1/2/5/11 (mbps)], 11g: [6, 9, 12, 18, 24, 36, 48, 54 (mbps)]\n");
            AML_PRINT_LOG_INFO("  11n: [0-7 (mcs)], 11ac: [0-9 (mcs)]\n");
            return;
        }
        gB2BTestCasePacket.data_rate = map_rate;
        AML_PRINT_LOG_INFO("<PT> Set data rate to 0x%04x\n", map_rate);
    }

}
/* AMPDU or MPDU */
void aml_iwpriv_set_tx_type(unsigned int type)
{
    AML_PRINT_LOG_INFO("type = %d\n", type);
    if (aml_wifi_is_enable_rf_test()) {
        if (type <1 || type > 11) {
            AML_PRINT_LOG_INFO("Not support test type\n");
            return;
        }
        gB2BTestCasePacket.packet_type = type;
        AML_PRINT_LOG_INFO("<PT> Set pkt_type(mpdu/ampdu) to %d\n", type);
    }
}

void aml_iwpriv_set_tx_len(unsigned int len)
{
    AML_PRINT_LOG_INFO("len = %d\n", len);
    if (aml_wifi_is_enable_rf_test()) {
        gB2BTestCasePacket.pkt_length = len;
        AML_PRINT_LOG_INFO("<PT> Set pkt_len(mpdu_len) to %d\n", len);
    }
}

void aml_iwpriv_set_tx_num(unsigned int num)
{
    AML_PRINT_LOG_INFO("num = %d\n", num);
    if (aml_wifi_is_enable_rf_test()) {
        gB2BTestCasePacket.send_frame_num = num;
        AML_PRINT_LOG_INFO("<PT> Set frame num to %d\n", num);
    }
}

void aml_iwpriv_pt_tx_start(void)
{
    AML_PRINT_LOG_INFO("\n");
    if (!aml_wifi_is_enable_rf_test()) {
        return;
    }
    if (gB2BTestCasePacket.packet_type < 1 || gB2BTestCasePacket.packet_type > 11) {
        gB2BTestCasePacket.packet_type = TYPE_COMMON;
    }
    if (gB2BTestCasePacket.send_frame_num == 0) {
        gB2BTestCasePacket.send_frame_num = 0xffffffff;
    }
    AML_PRINT_LOG_INFO("<PT> To be send data packets and test type : %d\n", gB2BTestCasePacket.packet_type);
    AML_PRINT_LOG_INFO("<PT> Please make sure set bssid mac address as the same with dest.\n");
    AML_PRINT_LOG_INFO("<PT> Before calling prepare_test_hal_layer_thr_init.\n");
    prepare_test_hal_layer_thr_init(gB2BTestCasePacket.packet_type);
}

void aml_iwpriv_pt_tx_end(void)
{
    AML_PRINT_LOG_INFO("\n");
    if (!aml_wifi_is_enable_rf_test()) {
        return;
    }
    AML_PRINT_LOG_INFO("<PT> Before stop tx\n");
    Task_Schedule(TYPE_STOP_TX);
}

void aml_iwpriv_set_rx_path(struct wlan_net_vif *wnet_vif, unsigned char antenna, unsigned int channel)
{
    AML_PRINT_LOG_INFO("antenna = %d, channel = %d\n", antenna, channel);

    if ((channel > 14 && channel < 36) || channel > 165) {
        AML_PRINT_LOG_INFO("error channel!\n");
        return;
    }

    if (antenna != 0) {
        AML_PRINT_LOG_INFO("W1u just support one antenna, set antenna to 0!\n");
        antenna = 0;
    }

    if (aml_wifi_is_enable_rf_test()) {
        AML_PRINT_LOG_INFO("<PT> Set rx to : antenna %d, channel %d\n", antenna, channel);
    }

}

void aml_iwpriv_pt_rx_start(struct net_device *dev, unsigned char qos)
{
    struct wlan_net_vif *wnet_vif = aml_iwpriv_get_vif(dev->name);

    AML_PRINT_LOG_INFO("qos = %d\n", qos);
    if (!aml_wifi_is_enable_rf_test()) {
        return;
    }
    wnet_vif->vif_ops.pt_rx_start(!!qos);
}

int aml_iwpriv_pt_rx_end(struct net_device *dev, union iwreq_data *wrqu, char *extra)
{
    struct rx_statics_st rx_data;
    struct wlan_net_vif *wnet_vif = aml_iwpriv_get_vif(dev->name);

    AML_PRINT_LOG_INFO("\n");
    if (!aml_wifi_is_enable_rf_test()) {
        AML_PRINT_LOG_INFO("WARNING: not rf test mode\n");
        return -1;
    }
    rx_data = wnet_vif->vif_ops.pt_rx_stop();

    wrqu->data.length = scnprintf(extra, IW_PRIV_SIZE_MASK, " rx_ucast=0x%x, rx_other=0x%x, rx_mcast=0x%x, rx_fcserr=0x%x",
                                    (rx_data.rx_ucast<<3)>>3, (rx_data.rx_other<<2)>>2, (rx_data.rx_mcast<<2)>>2,rx_data.rx_fcserr);
    wrqu->data.length++;

    return 0;
}

void aml_iwpriv_set_stbc(struct net_device *dev, unsigned int enable)
{
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    struct wlan_net_vif *wnet_vif = aml_iwpriv_get_vif(dev->name);

    AML_PRINT_LOG_INFO("This action should be executed before connecting to ap or creating ap\n");
    if (1 == enable) {
        AML_PRINT_LOG_INFO("enable stbc\n");
        wifimac->wm_flags_ext2 |= WIFINET_VHTCAP_RX_STBC;
        wnet_vif->vm_htcap |= WIFINET_HTCAP_C_RXSTBC_1SS;
    } else if (0 == enable) {
        AML_PRINT_LOG_INFO("disable stbc\n");
        wifimac->wm_flags_ext2 &= ~WIFINET_VHTCAP_RX_STBC;
        wnet_vif->vm_htcap &= ~WIFINET_HTCAP_C_RXSTBC_1SS;
    }
    wnet_vif->vm_tx_stbc = GET_VHT_CAP_TX_STBC(wifimac->wm_flags_ext2);
    wnet_vif->vm_rx_stbc = GET_VHT_CAP_RX_STBC(wifimac->wm_flags_ext2);
}

static unsigned int delta_pwr_map[] = // Power change delta_P, digital gain need adjust delta_pwr_map[delta_P]/100 times.
        {
            1000, 1122, 1258, 1412, 1584, 1778, 1995, 2238, 2511, 2818,
            3162, 3548, 3981, 4466, 5011, 5623, 6309, 7079, 7943, 8912,
            10000, 11220, 12589, 14125, 15848, 17782, 19952, 22387, 25118, 28183,
            31622, 35481, 39810, 44668, 50118, 56234, 63095, 70794, 79432, 89125,
            100000, 112201, 125892, 141253, 158489, 177827, 199526, 223872, 251188// Gain max 0xff
        };

void aml_set_delta_pwr(int delta_pwr, int base_pwr, unsigned char base_gain)
{
    struct hw_interface *hif = hif_get_hw_interface();
    unsigned char map_size = sizeof(delta_pwr_map)/sizeof(delta_pwr_map[0]);
    unsigned int reg_data = 0;
    struct digital_gain_reg_bits *gain_bits = NULL;
    // Power increase or decrease
    unsigned char increase = delta_pwr > 0 ? 1 : 0;
    unsigned int abs_pwr = increase ? delta_pwr : (-1 * delta_pwr);
    // Original gain and gain after adjusting delta_pwr
    unsigned int old_gain, target_gain = 0;
    // The delta_pwr increase and decrease limit values which calculated based on the current gain
    int limit_delta_pwr_in, limit_delta_pwr_de = 0;
    unsigned int index = 0;
    unsigned char limit_find = 0;

    if (abs_pwr > sizeof(delta_pwr_map)) {
        AML_PRINT_LOG_INFO("Set tx power failed, arg should in [-%d, %d]!\n", map_size - 1, map_size - 1);
        return;
    }

    if (base_gain) {
        gain_bits = (struct digital_gain_reg_bits *)&reg_data;
        gain_bits->gain = base_gain;
    } else {
        reg_data = hif->hif_ops.hi_read_word(0x00a0e4b8);
        gain_bits = (struct digital_gain_reg_bits *)&reg_data;
    }
    old_gain = gain_bits->gain;

    if (increase) {
        target_gain = old_gain * delta_pwr_map[abs_pwr] / 1000;
    } else {
        target_gain = old_gain * 1000 / delta_pwr_map[abs_pwr];
    }

    if (target_gain == 0 || target_gain > 0xff) {
        //Find the adjustment range of gain
        for (index = 0; index < map_size; index ++) {
            if (delta_pwr_map[index] <= old_gain * 1000) {
                // The limit of old_gain can only be reduced to 0x01
                // Old_gain / _times_ >= 1
                limit_delta_pwr_de = index;
            } else {
                limit_find |= BIT(0);
            }
            if (delta_pwr_map[index] * old_gain / 1000 <= 0xff) {
                // The limit of old_gain can only be increased to 0xff
                // old_gain * _times_ <= 0xff
                limit_delta_pwr_in = index;
            } else {
                limit_find |= BIT(1);
            }
            if (limit_find == 0x3) {
                break;
            }
        }
        limit_delta_pwr_de = -1 * limit_delta_pwr_de;
        if (base_gain != 0) {
            limit_delta_pwr_de += base_pwr;
            limit_delta_pwr_in += base_pwr;
            limit_delta_pwr_de = limit_delta_pwr_de > 0 ? limit_delta_pwr_de : 1;
            AML_PRINT_LOG_INFO("    Tx pwr %d need digital gain 0x%x, tx pwr limit in [%d, %d]!\n",
                base_pwr, base_gain, limit_delta_pwr_de, limit_delta_pwr_in);
        } else {
            AML_PRINT_LOG_INFO("    Digital gain now 0x%x, delta tx pwr limit in [%d, %d]!\n",
                old_gain, limit_delta_pwr_de, limit_delta_pwr_in);
        }
        return;
    }

    reg_data = 0;
    gain_bits->enable = 1;
    gain_bits->gain = target_gain;
    hif->hif_ops.hi_write_word(0x00a0e4b8, reg_data);

    AML_PRINT_LOG_INFO("Digital gain reg set: 0x%08x\n", reg_data);
}

void aml_iwpriv_set_delta_tx_pwr(int delta_pwr)
{
    AML_PRINT_LOG_INFO("delta_pwr = %d\n", delta_pwr);
    aml_set_delta_pwr(delta_pwr, 0, 0);
}

int aml_get_efuse_gain_map_index(unsigned int rate, unsigned int channel, unsigned int bw)
{
    unsigned char band = 1;
    unsigned char band_5g = 0;

    if (channel <= 14) {
        band = 0;
    } else if (channel <= 48) {
        band_5g = 0;
    } else if (channel <= 64) {
        band_5g = 1;
    } else if (channel <= 112) {
        band_5g = 2;
    } else if (channel <= 144) {
        band_5g = 3;
    } else if (channel <= 165) {
        band_5g = 4;
    } else {
        //error channel
        return -1;
    }

    if (band == 0) {//2.4GHz
        if (WIFINET_BWC_WIDTH20 == bw) {
            if (rate <= WIFI_11BG_MAX) {//2.4GHz 20M legacy
                return 0;
            } else if (IS_HT_RATE(rate)) {//2.4GHz 20M 11N
                return 1;
            } else {
                return -1;
            }
        } else if (WIFINET_BWC_WIDTH40 == bw) {
            if (IS_HT_RATE(rate)) {//2.4GHz 40M 11N
                return 2;
            } else {
                return -1;
            }
        } else {
            return -1;
        }
    } else if (band == 1) {//5GHz
        if (band_5g == 0) {//5G_BAND0 20M/40M/80M
            return 3;
        } else if (band_5g == 1) {//5G_BAND1 20M/40M/80M
            return 4;
        } else if (band_5g == 2) {
            if (WIFINET_BWC_WIDTH20 == bw) {//5G_BAND2 20M
                return 5;
            } else if (WIFINET_BWC_WIDTH40 == bw) {//5G_BAND2 40M
                return 6;
            } else if (WIFINET_BWC_WIDTH80 == bw) {//5g_BAND2 80M
                return 7;
            }
        } else if (band_5g == 3) {//5G_BAND3 20M/40M/80M
            return 8;
        } else if (band_5g == 4) {//5G_BAND4 20M/40M/80M
            return 9;
        } else {
            return -1;
        }
    }
    return -1;
}

void aml_calc_code_rate_gain(unsigned int rate, unsigned int channel, unsigned char *key_gain)
{
    unsigned int increase = LOW_MCS_GAIN_COMPENSATE >= 0 ? 2 : 0;
    unsigned int ratio = delta_pwr_map[(increase - 1) * LOW_MCS_GAIN_COMPENSATE];
    unsigned int target_gain = *key_gain;

    if (channel <= 14) {
        return;
    }

    if (IS_HT_RATE(rate) || IS_VHT_RATE(rate)) {
        if ((rate & ~WIFI_RATE_MASK) <= LOW_DAC_MCS_OFT) {
            if (increase != 0) {
                target_gain = target_gain * ratio / 1000;
            } else {
                target_gain = target_gain * 1000 / ratio;
            }
            *key_gain = target_gain & 0xff;
        }
    }
}

extern unsigned char get_s8_item(char * varbuf, int len, char * item, char * item_value);
extern unsigned char get_s32_item(char *varbuf, int len, char *item, unsigned int *item_value);
extern unsigned int process_cali_content(char *varbuf, unsigned int len);

#define EFUSE_POWER_MAP(_map_index, _param_name, _power, _efuse_addr, _byte_oft) \
                                   {_param_name, _power, _efuse_addr, _byte_oft}

static struct key_gain_efuse_power_map kg_efuse_map[] =
{
   EFUSE_POWER_MAP(0, "pwr_band2_11b_20M",  18, 0x0b, 0), //11b/g 20M 2.4G
   EFUSE_POWER_MAP(1, "pwr_band2_11n_20M",  17, 0x0a, 2), //11n 20M 2.4G
   EFUSE_POWER_MAP(2, "pwr_band2_11n_40M",  17, 0x0a, 3), //11n 40M 2.4G
   EFUSE_POWER_MAP(3, "pwr_band5_5200",     14, 0x0c, 0), //band0{36, 40, 44, 48} 20M/40M/80M
   EFUSE_POWER_MAP(4, "pwr_band5_5300",     14, 0x0c, 1), //band1{52, 56, 60, 64} 20M/40M/80M
   EFUSE_POWER_MAP(5, "pwr_band5_5530_20M", 15, 0x0b, 1), //band2{100, 104, 108, 112} 20M
   EFUSE_POWER_MAP(6, "pwr_band5_5530_40M", 14, 0x0b, 2), //band2{100, 104, 108, 112} 40M
   EFUSE_POWER_MAP(7, "pwr_band5_5530_80M", 14, 0x0b, 3), //band2{100, 104, 108, 112} 80M
   EFUSE_POWER_MAP(8, "pwr_band5_5660",     14, 0x0c, 2), //band3{116, 120, 124, 128, 132, 136, 140, 144} 20M/40M/80M
   EFUSE_POWER_MAP(9, "pwr_band5_5780",     14, 0x0c, 3), //band4{149, 153, 157, 161, 165} 20M/40M/80M
};

extern char * conf_path;

void aml_get_key_power(unsigned int rate, unsigned int channel, unsigned int bw,
    unsigned char * key_gain, unsigned int * key_power)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
    struct file *fp;
    struct kstat stat;
    mm_segment_t fs;
#else
    const struct firmware *fw = NULL;
    struct device *dev = vm_cfg80211_get_parent_dev();
#endif

    int error = 0;
    int size, len = 0;
    char *content = NULL;
    unsigned int vendor_sn = 0;
    unsigned char chip_id_buf[100];

    unsigned char txt_efuse_name[10] = {0};
    unsigned char txt_efuse_invalid = 1;
    unsigned int efuse_param = 0;

    unsigned map_idx = 0;
    struct key_gain_efuse_power_map * pwr_map = NULL;

    map_idx = aml_get_efuse_gain_map_index(rate, channel, bw);
    if (map_idx < 0 || map_idx >= ARRAY_LENGTH(kg_efuse_map)) {
        AML_PRINT_LOG_INFO("ERROR, current tx_rate: %x, tx_channel: %d, tx_bw: %d\n", rate, channel, bw);
        return;
    }

    pwr_map = &(kg_efuse_map[map_idx]);
    *key_power = pwr_map->default_abs_power;

    vendor_sn = efuse_manual_read(0xf);
    vendor_sn &= 0xffff;
    sprintf(chip_id_buf, "%s/aml_wifi_rf_%04.txt", conf_path, vendor_sn);

    if (isFileReadable(chip_id_buf, NULL) != 0) {
        memset(chip_id_buf,'\0',sizeof(chip_id_buf));
        switch ((vendor_sn & 0xff00) >> 8) {
            case MODULE_ITON:
                sprintf(chip_id_buf, "%s/aml_wifi_rf_iton.txt", conf_path);
                break;
            case MODULE_AMPAK:
                sprintf(chip_id_buf, "%s/aml_wifi_rf_ampak.txt", conf_path);
                break;
            case MODULE_FN_LINK:
                sprintf(chip_id_buf, "%s/aml_wifi_rf_fn_link.txt", conf_path);
                break;
            default:
                if (aml_bus_type) {
                    sprintf(chip_id_buf, "%s/aml_wifi_rf_usb.txt", conf_path);
                }
#ifdef SDIO_MODE_ON
                else {
                    sprintf(chip_id_buf, "%s/aml_wifi_rf_sdio.txt", conf_path);
                }
#endif
        }
        AML_PRINT_LOG_INFO("aml wifi module SN:%04x  sn txt not found, the rf config: %s\n", vendor_sn, chip_id_buf);
    } else
        AML_PRINT_LOG_INFO("aml wifi module SN:%04x  the rf config: %s\n", vendor_sn, chip_id_buf);

    do {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
        fs = get_fs();
        set_fs(KERNEL_DS);
        fp = filp_open(chip_id_buf, O_RDONLY, 0);

        if (IS_ERR(fp)) {
            fp = NULL;
            break;
        }
        error = vfs_stat(chip_id_buf, &stat);
        if (error) {
            filp_close(fp, NULL);
            break;
        }

        size = (int)stat.size;
        if (size <= 0) {
            filp_close(fp, NULL);
            break;
        }

        content = ZMALLOC(size, chip_id_buf, GFP_KERNEL);

        if (content == NULL) {
            filp_close(fp, NULL);
            break;
        }
        if (vfs_read(fp, content, size, &fp->f_pos) != size) {
            FREE(content, "wifi_cali_param");
            filp_close(fp, NULL);
            break;
        }
#else
        error = request_firmware(&fw, chip_id_buf, dev);
        if (error) {
            // sn txt not found, the rf set default config
            sprintf(chip_id_buf, "aml_wifi_rf.txt");
            error = request_firmware(&fw, chip_id_buf, dev);
            if (error) {
                 break;
            }
        }
        size = fw->size;
        content = (char *)fw->data;
#endif
        len = process_cali_content(content, size);

        /* get key power from rf txt */
        get_s8_item(content, len, pwr_map->param_name, key_power);

        /* get key gain from rf txt */
        sprintf(txt_efuse_name, "efuse_%x", pwr_map->word_addr);
        txt_efuse_invalid = get_s32_item(content, len, txt_efuse_name, &efuse_param);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
        FREE(content, "wifi_cali_param");
        filp_close(fp, NULL);
        set_fs(fs);
#else
    release_firmware(fw);
#endif
    } while (0);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
    set_fs(fs);
#endif

    if (txt_efuse_invalid) {
        efuse_param = efuse_manual_read(pwr_map->word_addr);
        AML_PRINT_LOG_INFO("read key gain from efuse: 0x%x, byte_oft = %d\n", efuse_param, pwr_map->byte_oft);
    } else {
        AML_PRINT_LOG_INFO("read key gain from txt: 0x%x, byte_oft = %d\n", efuse_param, pwr_map->byte_oft);
    }

    *key_gain = efuse_param >> (8 * pwr_map->byte_oft);
    *key_gain &= 0x000000ff;
    aml_calc_code_rate_gain(rate, channel, key_gain);
}

void aml_iwpriv_set_tx_pwr(int pwr)
{
    unsigned int tx_rate = gB2BTestCasePacket.data_rate;
    unsigned int tx_channel = gB2BTestCasePacket.channel;
    unsigned char tx_bw = gB2BTestCasePacket.channel_bw;
    unsigned int key_gain, key_power = 0;

    AML_PRINT_LOG_INFO("power = %d\n", pwr);

    if (!aml_wifi_is_enable_rf_test()) {
        AML_PRINT_LOG_INFO("Now not in pt mode, exit\n");
        return;
    }

    aml_get_key_power(tx_rate, tx_channel, tx_bw, &key_gain, &key_power);
    if (key_gain == 0) {
        AML_PRINT_LOG_INFO("please check efuse_gain/txt_gain\n");
        return;
    }

    aml_set_delta_pwr(pwr - key_power, key_power, key_gain);
}


int aml_iwpriv_set_efuse(int addr, int value)
{
    int i = 0;

    for (i = 0; i < 32; i++) {
        if (value & (1 << i)) {
            efuse_manual_write(i, addr);
        }
    }
    AML_PRINT_LOG_INFO("write efuse addr is :0x%x, data :0x%08x\n", addr, value);

    return 0;

}

unsigned int aml_read_efuse_domain(enum efuse_domain domain)
{

    unsigned int read_value = 0;

    if (domain >= EFUSE_MAX_DOMAIN) {
        AML_PRINT_LOG_ERR("domain[%d] out of range !\n",domain);
        return 0;
    }

    switch (domain)
    {
        case RF_VLD_XOSC_CTUNE:
        case RF_VLD_PWR2G:
        case RF_VLD_PWR5G:
        case RF_VLD_PWR5G_BAND:
        case RF_VLD_PWR5G_DELTA:
        case RF_VLD_PWR5G_BAND_DELTA:
            read_value = (((efuse_manual_read(EFUSE_ADDR_9) & 0x00ff0000) >> 16) & BIT(domain));
            if (read_value) {
                read_value = RF_VLD_ENABLE;
            }
            break;
        case RF_VLD_PWR2G_DELTA:
            read_value = (((efuse_manual_read(EFUSE_ADDR_9) & 0x00ff0000) >> 16) & BIT(domain + 1));
            if (read_value) {
                read_value = RF_VLD_ENABLE;
            }
            break;
        case RF_VLD:
            read_value = ((efuse_manual_read(EFUSE_ADDR_9) & 0x00ff0000) >> 16);
            break;
        case XOSC_CTUNE:
            read_value = ((efuse_manual_read(EFUSE_ADDR_A) & 0x000000ff));
            break;
        case PWR2G_20M:
            read_value = ((efuse_manual_read(EFUSE_ADDR_A) & 0x00ff0000) >> 16);
            break;
        case PWR2G_40M:
            read_value = ((efuse_manual_read(EFUSE_ADDR_A) & 0xff000000) >> 24);
            break;
        case PWR2G_11B:
            read_value = ((efuse_manual_read(EFUSE_ADDR_B) & 0x000000ff));
            break;
        case PWR5G_20M:
            read_value = ((efuse_manual_read(EFUSE_ADDR_B) & 0x0000ff00) >> 8);
            break;
        case PWR5G_40M:
            read_value = ((efuse_manual_read(EFUSE_ADDR_B) & 0x00ff0000) >> 16);
            break;
        case PWR5G_80M:
            read_value = ((efuse_manual_read(EFUSE_ADDR_B) & 0xff000000) >> 24);
            break;
        case PWR5G_BAND0:
            read_value = ((efuse_manual_read(EFUSE_ADDR_C) & 0x000000ff));
            break;
        case PWR5G_BAND1:
            read_value = ((efuse_manual_read(EFUSE_ADDR_C) & 0x0000ff00) >> 8);
            break;
        case PWR5G_BAND3:
            read_value = ((efuse_manual_read(EFUSE_ADDR_C) & 0x00ff0000) >> 16);
            break;
        case PWR5G_BAND4:
            read_value = ((efuse_manual_read(EFUSE_ADDR_C) & 0xff000000) >> 24);
            break;
        //delta
        case PWR2G_20M_DELTA:
            read_value = ((efuse_manual_read(EFUSE_ADDR_D) & 0x0000000f));
            break;
        case PWR2G_40M_DALTA:
            read_value = ((efuse_manual_read(EFUSE_ADDR_D) & 0x000000f0) >> 4);
            break;
        case PWR2G_11B_DELTA:
            read_value = ((efuse_manual_read(EFUSE_ADDR_D) & 0x00000f00) >> 8);
            break;
        case PWR5G_20M_DELTA:
            read_value = ((efuse_manual_read(EFUSE_ADDR_D) & 0x0000f000) >> 12);
            break;
        case PWR5G_40M_DELTA:
            read_value = ((efuse_manual_read(EFUSE_ADDR_D) & 0x000f0000) >> 16);
            break;
        case PWR5G_80M_DELTA:
            read_value = ((efuse_manual_read(EFUSE_ADDR_D) & 0x00f00000) >> 20);
            break;
        case PWR5G_BAND0_DELTA:
            read_value = ((efuse_manual_read(EFUSE_ADDR_D) & 0x0f000000) >> 24);
            break;
        case PWR5G_BAND1_DELTA:
            read_value = ((efuse_manual_read(EFUSE_ADDR_D) & 0xf0000000) >> 28);
            break;
        case PWR5G_BAND3_DELTA:
            read_value = ((efuse_manual_read(EFUSE_ADDR_E) & 0x0000000f));
            break;
        case PWR5G_BAND4_DELTA:
            read_value = ((efuse_manual_read(EFUSE_ADDR_E) & 0x000000f0) >> 4);
            break;
        case XOSC_CTUNE_DELTA:
            read_value = ((efuse_manual_read(EFUSE_ADDR_E) & 0x00001f00) >> 8);
            break;
        default:
            AML_PRINT_LOG_ERR("domain[%d] out of range !\n",domain);
            return 0;
            break;
    }

    AML_PRINT_LOG_INFO("domain[%d] value:0x%x \n",domain,read_value);
    return read_value;
}


int aml_set_rf_vld(unsigned int set_value)
{
    unsigned int efuse_val = 0;

    AML_PRINT_LOG_INFO("set value 0x%08x\n",set_value);

    efuse_val = efuse_manual_read(EFUSE_ADDR_9);

    if (!(efuse_val & set_value)) {
        aml_iwpriv_set_efuse(EFUSE_ADDR_9,set_value);
    } else {
        AML_PRINT_LOG_INFO("already enabled RF_VLD[0x%08x]!\n",efuse_val);
    }

    return 0;
}

int aml_iwpriv_set_efuse_domain(enum efuse_domain domain, unsigned char value)
{
    unsigned int set_value = 0;
    unsigned int efuse_value = 0;

    set_value = value;

    AML_PRINT_LOG_INFO("domain[%d] value[0x%x]\n",domain,value);

    if (domain >= EFUSE_MAX_DOMAIN ) {
        AML_PRINT_LOG_ERR("domain[%d] out of range !\n",domain);
        return -1;
    }

    switch (domain)
    {
        case RF_VLD_XOSC_CTUNE:
            if (value == RF_VLD_ENABLE) {
                aml_set_rf_vld(BIT(16)); //bit0
            }
            break;
        case RF_VLD_PWR2G:
            if (value == RF_VLD_ENABLE) {
                aml_set_rf_vld(BIT(17)); //bit1
            }
            break;
        case RF_VLD_PWR5G:
            if (value == RF_VLD_ENABLE) {
                aml_set_rf_vld(BIT(18)); //bit2
            }
            break;
        case RF_VLD_PWR5G_BAND:
            if (value == RF_VLD_ENABLE) {
                aml_set_rf_vld(BIT(19)); //bit3
            }
            break;
        case RF_VLD_PWR5G_DELTA:
            if (value == RF_VLD_ENABLE) {
                aml_set_rf_vld(BIT(20)); //bit4
            }
            break;
        case RF_VLD_PWR5G_BAND_DELTA:
            if (value == RF_VLD_ENABLE) {
                aml_set_rf_vld(BIT(21)); //bit5
            }
            break;
        case RF_VLD_PWR2G_DELTA:
            if (value == RF_VLD_ENABLE) {
                aml_set_rf_vld(BIT(23)); //bit7
            }
            break;

        case RF_VLD:
            set_value = (set_value << 16) & 0x00ff0000;
            aml_iwpriv_set_efuse(EFUSE_ADDR_9,set_value);
            break;
        case XOSC_CTUNE:
            set_value = set_value & 0x000000ff;
            aml_iwpriv_set_efuse(EFUSE_ADDR_A,set_value);
            break;
        case PWR2G_20M:
            set_value = (set_value << 16) & 0x00ff0000;
            aml_iwpriv_set_efuse(EFUSE_ADDR_A,set_value);
            break;
        case PWR2G_40M:
            set_value = (set_value << 24) & 0xff000000;
            aml_iwpriv_set_efuse(EFUSE_ADDR_A,set_value);
            break;
        case PWR2G_11B:
            set_value = set_value & 0x000000ff;
            aml_iwpriv_set_efuse(EFUSE_ADDR_B,set_value);
            break;
        case PWR5G_20M:
            set_value = (set_value << 8) & 0x0000ff00;
            aml_iwpriv_set_efuse(EFUSE_ADDR_B,set_value);
            break;
        case PWR5G_40M:
            set_value = (set_value << 16) & 0x00ff0000;
            aml_iwpriv_set_efuse(EFUSE_ADDR_B,set_value);
            break;
        case PWR5G_80M:
            set_value = (set_value << 24) & 0xff000000;
            aml_iwpriv_set_efuse(EFUSE_ADDR_B,set_value);
            break;
        case PWR5G_BAND0:
            set_value = set_value & 0x000000ff;
            aml_iwpriv_set_efuse(EFUSE_ADDR_C,set_value);
            break;
        case PWR5G_BAND1:
            set_value = (set_value << 8) & 0x0000ff00;
            aml_iwpriv_set_efuse(EFUSE_ADDR_C,set_value);
            break;
        case PWR5G_BAND3:
            set_value = (set_value << 16) & 0x00ff0000;
            aml_iwpriv_set_efuse(EFUSE_ADDR_C,set_value);
            break;
        case PWR5G_BAND4:
            set_value = (set_value << 24) & 0xff000000;
            aml_iwpriv_set_efuse(EFUSE_ADDR_C,set_value);
            break;
        //delta
        case PWR2G_20M_DELTA:
            efuse_value = aml_read_efuse_domain(PWR2G_20M);
            if (value > efuse_value) {
                set_value = ((value - efuse_value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_D,set_value);
            } else {
                set_value = ((efuse_value - value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_D,0x10 - set_value);
            }
            break;
        case PWR2G_40M_DALTA:
            efuse_value = aml_read_efuse_domain(PWR2G_40M);
            if (value > efuse_value) {
                set_value = ((value - efuse_value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_D,set_value << 4);
            } else {
                set_value = ((efuse_value - value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_D,(0x10 - set_value) << 4);
            }
            break;
        case PWR2G_11B_DELTA:
            efuse_value = aml_read_efuse_domain(PWR2G_11B);
            if (value > efuse_value) {
                set_value = ((value - efuse_value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_D,set_value << 8);
            } else {
                set_value = ((efuse_value - value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_D,(0x10 - set_value) << 8);
            }
            break;
        case PWR5G_20M_DELTA:
            efuse_value = aml_read_efuse_domain(PWR5G_20M);
            if (value > efuse_value) {
                set_value = ((value - efuse_value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_D,set_value << 12);
            } else {
                set_value = ((efuse_value - value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_D,(0x10 - set_value) << 12);
            }
            break;
        case PWR5G_40M_DELTA:
            efuse_value = aml_read_efuse_domain(PWR5G_40M);
            if (value > efuse_value) {
                set_value = ((value - efuse_value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_D,set_value << 16);
            } else {
                set_value = ((efuse_value - value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_D,(0x10 - set_value) << 16);
            }
            break;
        case PWR5G_80M_DELTA:
            efuse_value = aml_read_efuse_domain(PWR5G_80M);
            if (value > efuse_value) {
                set_value = ((value - efuse_value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_D,set_value << 20);
            } else {
                set_value = ((efuse_value - value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_D,(0x10 - set_value) << 20);
            }
            break;
        case PWR5G_BAND0_DELTA:
            efuse_value = aml_read_efuse_domain(PWR5G_BAND0);
            if (value > efuse_value) {
                set_value = ((value - efuse_value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_D,set_value << 24);
            } else {
                set_value = ((efuse_value - value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_D,(0x10 - set_value) << 24);
            }
            break;
        case PWR5G_BAND1_DELTA:
            efuse_value = aml_read_efuse_domain(PWR5G_BAND1);
            if (value > efuse_value) {
                set_value = ((value - efuse_value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_D,set_value << 28);
            } else {
                set_value = ((efuse_value - value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_D,(0x10 - set_value) << 28);
            }
            break;
        case PWR5G_BAND3_DELTA:
            efuse_value = aml_read_efuse_domain(PWR5G_BAND3);
            if (value > efuse_value) {
                set_value = ((value - efuse_value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_E,set_value);
            } else {
                set_value = ((efuse_value - value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_E,(0x10 - set_value));
            }
            break;
        case PWR5G_BAND4_DELTA:
            efuse_value = aml_read_efuse_domain(PWR5G_BAND4);
            if (value > efuse_value) {
                set_value = ((value - efuse_value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_E,set_value << 4);
            } else {
                set_value = ((efuse_value - value) >> 2) & 0x0f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_E,(0x10 - set_value) << 4);
            }
            break;
        case XOSC_CTUNE_DELTA:
            efuse_value = aml_read_efuse_domain(XOSC_CTUNE);
            if (value > efuse_value) {
                set_value = (value - efuse_value) & 0x1f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_E,set_value << 8);
            } else {
                set_value = (efuse_value - value) & 0x1f;
                aml_iwpriv_set_efuse(EFUSE_ADDR_E,(0x20 - set_value) << 8);
            }
            break;

        default:
             AML_PRINT_LOG_ERR("domain[%d] out of range !\n",domain);
            return -4;
            break;
    }

    return 0;
}

int aml_iwpriv_set_pt_efuse(int addr, int value)
{
    if ((addr != EFUSE_ADDR_9) && (addr != EFUSE_ADDR_A) && (addr != EFUSE_ADDR_B) && (addr != EFUSE_ADDR_C) && (addr != EFUSE_ADDR_D) && (addr != EFUSE_ADDR_E)) {
        AML_PRINT_LOG_WRAN(" [FAIL] efuse addr illegal ! 0x%x\n", addr);
        return -1;
    }

    printk("set pt efuse addr:0x%x, data is :0x%08x efuse_value\n", addr, value, efuse_manual_read(addr));

    if ((addr == EFUSE_ADDR_9) && (value & EFUSE_ADDR_9_ILLEGAL_BIT)) {
        AML_PRINT_LOG_WRAN(" [FAIL] rf_vld illegal 0x%x\n", value);
        return -1;
    }

    if ((addr == EFUSE_ADDR_A) && (value & EFUSE_ADDR_A_ILLEGAL_BIT)) {
        AML_PRINT_LOG_WRAN(" [FAIL] efuse_a illegal value 0x%x\n ", value);
        return -1;
    }
    if ((addr == EFUSE_ADDR_A) && (efuse_manual_read(addr) & EFUSE_ADDR_A_LEGAL_BIT)) {
        AML_PRINT_LOG_WRAN(" [FAIL] efuse_a has already be written [0x%x]!\n", efuse_manual_read(addr));
        return -1;
    }

    if ((addr == EFUSE_ADDR_B) && efuse_manual_read(addr)) {
        AML_PRINT_LOG_WRAN(" [FAIL] efuse_b has already be written [0x%x] !\n", efuse_manual_read(addr));
        return -1;
    }

    if ((addr == EFUSE_ADDR_C) && efuse_manual_read(addr)) {
        AML_PRINT_LOG_WRAN(" [FAIL] efuse_c has already be written [0x%x]!\n", efuse_manual_read(addr));
        return -1;
    }

    if ((addr == EFUSE_ADDR_D) && efuse_manual_read(addr)) {
        AML_PRINT_LOG_WRAN(" [FAIL] efuse_d has already be written [0x%x]!\n", efuse_manual_read(addr));
        return -1;
    }

    if ((addr == EFUSE_ADDR_E) && efuse_manual_read(addr)) {
        AML_PRINT_LOG_WRAN(" [FAIL] efuse_e has already be written [0x%x]!\n", efuse_manual_read(addr));
        return -1;
    }

    aml_iwpriv_set_efuse(addr, value);
    return 0;

}

int aml_iwpriv_get_efuse(struct net_device *dev, char *str_addr, union iwreq_data *wrqu, char *extra)
{
    unsigned int addr, efuse_data = 0;

    addr = simple_strtol(str_addr, NULL, 0);
    efuse_data = efuse_manual_read(addr);
    printk("get efuse addr:0x%x, data is :0x%08x\n", addr, efuse_data);

    wrqu->data.length = scnprintf(extra, IW_PRIV_SIZE_MASK, "0x%08x", efuse_data);
    wrqu->data.length++;

    return 0;

}

int aml_iwpriv_get_efuse_domain(struct net_device *dev, char *str_domain, union iwreq_data *wrqu, char *extra)
{
    unsigned int domain, domain_data = 0;

    domain = simple_strtol(str_domain, NULL, 0);
    domain_data = aml_read_efuse_domain(domain);

    wrqu->data.length = scnprintf(extra, IW_PRIV_SIZE_MASK, "0x%02x", domain_data);
    wrqu->data.length++;

    return 0;
}

int aml_iwpriv_get_efuse_tmp(struct net_device *dev, char *str_addr, union iwreq_data *wrqu, char *extra)
{
    unsigned int efuse_addr, efuse_tmp_data = 0;

    efuse_addr = simple_strtol(str_addr, NULL, 0);

    if (efuse_addr == EFUSE_ADDR_9) {
        efuse_tmp_data = efuse_cfg_param.efuse_9;
    } else if (efuse_addr == EFUSE_ADDR_A) {
        efuse_tmp_data = efuse_cfg_param.efuse_a;
    }  else if (efuse_addr == EFUSE_ADDR_B) {
        efuse_tmp_data = efuse_cfg_param.efuse_b;
    }  else if (efuse_addr == EFUSE_ADDR_C) {
        efuse_tmp_data = efuse_cfg_param.efuse_c;
    }  else if (efuse_addr == EFUSE_ADDR_D) {
        efuse_tmp_data = efuse_cfg_param.efuse_d;
    }  else if (efuse_addr == EFUSE_ADDR_E) {
        efuse_tmp_data = efuse_cfg_param.efuse_e;
    }  else {
        AML_PRINT_LOG_INFO("WARNING: invalid efuse addr\n");
    }

    wrqu->data.length = scnprintf(extra, IW_PRIV_SIZE_MASK, "0x%08x", efuse_tmp_data);
    wrqu->data.length++;

    return 0;
}

int aml_iwpriv_get_sn(struct net_device *dev, union iwreq_data *wrqu, char *extra)
{
    unsigned int efuse_data;

    efuse_data = efuse_manual_read(0x0f);

    wrqu->data.length = scnprintf(extra, IW_PRIV_SIZE_MASK, "%02x:%02x", (( efuse_data & 0xff00) >> 8), (efuse_data & 0x00ff));
    wrqu->data.length++;

    return 0;
}

int aml_iwpriv_set_ant_switch(struct net_device *dev, char *parameter, union iwreq_data *wrqu, char *extra) {
    unsigned char using_ant;
    unsigned int ant_switch, reg_data;
    struct wlan_net_vif *wnet_vif = NULL;
    struct wifi_mac *wifimac = NULL;

    ant_switch = simple_strtol(parameter, NULL, 0);//0:auto ant sel   1:sel ant 1   2:set ant 2 other: only read
    wnet_vif = aml_iwpriv_get_vif(dev->name);
    wifimac = wnet_vif->vm_wmac;

    if (hal_ant_sel_en_get() == 0) {
        AML_PRINT_LOG_INFO("[WARNING] not suport ant switch\n");
        wrqu->data.length = scnprintf(extra, IW_PRIV_SIZE_MASK, " fail");
        wrqu->data.length++;
        return -1;
    }

    reg_data = get_reg(wnet_vif, RG_AON_A10);
    using_ant = ((reg_data & BIT(10)) >> 10) + 1;

    if ((using_ant - 1) != wifi_mac_get_cur_ant()) {
        AML_PRINT_LOG_INFO("sync using ant %d -> %d \n",wifi_mac_get_cur_ant(), (using_ant - 1));
        wifi_mac_sync_using_ant(using_ant -1);
    }

    if (aml_wifi_is_enable_rf_test()) {

        if (ant_switch == 1 || ant_switch == 2) {
            if (ant_switch != using_ant) {
                wifi_mac_switch_ant((void*)wifimac);
            }
        }

    } else {

        if (ant_switch == 0) {
            wifi_mac_reset_ant_measure();
            if (wnet_vif->vm_state  == WIFINET_S_CONNECTED && wifimac->wm_ant_rssi_measure.timer_handler) {
                os_timer_ex_start(&wifimac->wm_ant_rssi_measure);
            }
        } else if (ant_switch == 1 || ant_switch == 2) {
            if ((ant_switch + 1) % 2 != wifi_mac_get_cur_ant()) {
                wifi_mac_switch_ant((void*)wifimac);
            }
        }

    }

    msleep(5);

    reg_data = get_reg(wnet_vif, RG_AON_A10);
    using_ant = ((reg_data & BIT(10)) >> 10) + 1;
    wrqu->data.length = scnprintf(extra, IW_PRIV_SIZE_MASK, "%d", using_ant);
    wrqu->data.length++;

    AML_PRINT_LOG_INFO("set_ant: current is %d, set %d\n", wifi_mac_get_cur_ant() + 1, ant_switch);

    return 0;
}

int aml_iwpriv_get_bt_digital_gain_efuse_times(struct net_device *dev , union iwreq_data *wrqu, char *extra)
{
    unsigned int bt_efuse_time = 0;
    unsigned int times = 0;

    bt_efuse_time = efuse_manual_read(EFUSE_ADDR_4);

    //bt_digital_gain first times
    if ((bt_efuse_time & BIT(4)) == 0) {
        times = 1;

    } else {
        times = 0;
    }

    AML_PRINT_LOG_INFO("bt_pwr_dg_time: 0x%08x\n", times);
    wrqu->data.length = scnprintf(extra, IW_PRIV_SIZE_MASK, "bt_pwr_dg_time: 0x%02x", times);
    wrqu->data.length++;

    return times;
}

int aml_iwpriv_get_bt_digital_gain_efuse(struct net_device *dev , union iwreq_data *wrqu, char *extra)
{
    unsigned int bt_efuse_data = 0;

    bt_efuse_data = efuse_manual_read(EFUSE_ADDR_5);

    extra[0] = (bt_efuse_data & 0x00ff0000) >> 16;
    extra[1] = (bt_efuse_data & 0xff000000) >> 24;

    AML_PRINT_LOG_INFO("bt_pwr_dg_efuse:%08x, bdr:%02x edr:%02x\n", bt_efuse_data, extra[0], extra[1]);

    wrqu->data.length = scnprintf(extra, IW_PRIV_SIZE_MASK, "bdr:0x%02x, edr:0x%02x", extra[0], extra[1]);
    wrqu->data.length++;

    return 0;
}

int aml_iwpriv_set_bt_digital_gain_efuse(struct wlan_net_vif *wnet_vif, unsigned char bdr_gain, unsigned char edr_gain)
{
    unsigned int bt_efuse_data = 0;

    bt_efuse_data = efuse_manual_read(EFUSE_ADDR_5);
    if ((bt_efuse_data & BT_DIGITAL_GAIN_EFUSE_MASK) != 0) {
        AML_PRINT_LOG_INFO("bt efuse exist:%04x\n", (bt_efuse_data & BT_DIGITAL_GAIN_EFUSE_MASK));
        return -1;
    }

    bt_efuse_data = (((edr_gain << 8) | bdr_gain) << 16);
    aml_iwpriv_set_efuse(EFUSE_ADDR_5, bt_efuse_data);
    aml_iwpriv_set_efuse(EFUSE_ADDR_4, BIT(4));
    AML_PRINT_LOG_INFO("aml_set_bt_digital_gain_efuse:0x%8x\n", bt_efuse_data);

    return 0;
}

int aml_set_wmme(struct wlan_net_vif *wnet_vif, char *value)
{
    char buf[256] = {0};
    char *index_ptr = strstr(value, ",");

    while (index_ptr) {
        *index_ptr = ' ';
        index_ptr = strstr(value, ",");
    }

    sprintf(buf, "update_wmm_arg %s", value);

    AML_PRINT_LOG_INFO("buf:%s\n", buf);

    aml_update_wmm_arg(wnet_vif, buf, strlen(buf));
    return 0;

}

static int aml_iwpriv_set_wmme(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
    struct wlan_net_vif *wnet_vif = aml_iwpriv_get_vif(dev->name);

    AML_PRINT_LOG_INFO("%s\n", extra);
    aml_set_wmme(wnet_vif, extra);
    return 0;

}


static int aml_mem_dump(struct net_device *dev, int addr, int size)
{
    int len = 0, i = 0,ret = 0;
    unsigned int read_data = 0;
    unsigned char write_bin_flag = 0;
    char *la_buf = NULL;
    struct file *fp = NULL;
    char info_path[64];
    char *address = NULL;
    struct wlan_net_vif *wnet_vif = aml_iwpriv_get_vif(dev->name);

    if (size < 0) {
        AML_PRINT_LOG_ERR("size=%d is illegal !",size);
        goto err;
    }

    if ((addr & 0x00f00000) == 0x00100000) {
        addr &= ~BIT(20);
        AML_PRINT_LOG_INFO("The addr you put is fw addr need convert host addr=%#010x",addr);
    }
    else if ((addr & 0x00f00000) == 0x00800000) {
        addr &= ~0x00f0000;
        addr |= 0x00d00000;
        AML_PRINT_LOG_INFO("The addr you put is fw addr need convert host addr=%#010x",addr);
    }

    if ( (MAC_ICCM_AHB_BASE <= addr) && ((MAC_ICCM_AHB_BASE + ICCM_ROM_LEN) > addr) ) {
        if (size > (MAC_ICCM_AHB_BASE + ICCM_ROM_LEN - addr)) {
            AML_PRINT_LOG_ERR("size=%#x is bigger than ICCM_ROM_LEN(%#x) !",size,(MAC_ICCM_AHB_BASE + ICCM_ROM_LEN - addr));
            size = MAC_ICCM_AHB_BASE + ICCM_ROM_LEN - addr;
        }
        write_bin_flag = 1;
        scnprintf(info_path,64,"/data/dumpinfo_iccrom.bin");
    }
    else if ((MEM_ICCM_RAM_ADDR <= addr) && ((MEM_ICCM_RAM_ADDR + ICCM_RAM_LEN) > addr)) {
        if (size > (MEM_ICCM_RAM_ADDR + ICCM_RAM_LEN - addr)) {
            AML_PRINT_LOG_ERR("size=%#x is bigger than ICCM_RAM_LEN(%#x) !",size,(MEM_ICCM_RAM_ADDR + ICCM_RAM_LEN - addr));
            size = MEM_ICCM_RAM_ADDR + ICCM_RAM_LEN - addr;
        }
        write_bin_flag = 1;
        scnprintf(info_path,64,"/data/dumpinfo_iccram.bin");
    }
    else if ((MAC_DCCM_AHB_BASE <= addr) && ((MAC_DCCM_AHB_BASE + DCCM_LEN) > addr)) {
        if (size > (MAC_DCCM_AHB_BASE + DCCM_LEN - addr)) {
            AML_PRINT_LOG_ERR("size=%#x is bigger than DCCM_LEN(%#x) !",size,(MAC_DCCM_AHB_BASE + DCCM_LEN - addr));
            size = MAC_DCCM_AHB_BASE + DCCM_LEN - addr;
        }
        write_bin_flag = 1;
        scnprintf(info_path,64,"/data/dumpinfo_dccm.bin");
    }
    else if ((MEM_PKT_ADDR <= addr) && ((MEM_PKT_ADDR + MEM_PKT_lEN) > addr)) {
        if (size > (MEM_PKT_ADDR + MEM_PKT_lEN - addr)) {
            AML_PRINT_LOG_ERR("size=%#x is bigger than MEM_PKT_lEN(%#x) !",size,(MEM_PKT_ADDR + MEM_PKT_lEN- addr));
            size = MEM_PKT_ADDR + MEM_PKT_lEN - addr;
        }
        write_bin_flag = 1;
        scnprintf(info_path,64,"/data/dumpinfo_pkt.bin");
    }
    else if ((MAC_REG_BASE <= addr) && ((MAC_REG_BASE + REG_LEN) > addr)) {
        if (size > (MAC_REG_BASE + REG_LEN - addr)) {
            AML_PRINT_LOG_ERR("size=%#x is bigger than MAC_REG_LEN(%#x) !",size,(MAC_REG_BASE + REG_LEN - addr));
            size = MAC_REG_BASE + REG_LEN - addr;
        }
        scnprintf(info_path,64,"/data/dumpinfo_reg.txt");
    }
    else if ((MAC_SRAM_BASE <= addr) && ((MAC_SRAM_BASE + SRAM_LEN) > addr)) {
        if (size > (MAC_SRAM_BASE + SRAM_LEN - addr)) {
            AML_PRINT_LOG_ERR("size=%#x is bigger than SRAM_LEN(%#x) !",size,(MAC_SRAM_BASE + SRAM_LEN - addr));
            size = MAC_SRAM_BASE + SRAM_LEN - addr;
        }
        scnprintf(info_path,64,"/data/dumpinfo_sram.txt");
    }
    else if ((0x00f00000 <= addr) && ((0x00f00000 + 0x6000) > addr)) {
        if (size > (0x00f00000 + 0x6000 - addr)) {
            AML_PRINT_LOG_ERR("size=%#x is bigger than MAXLEN(%#x) !",size,(0x00f00000 + 0x6000 - addr));
            size = 0x00f00000 + 0x6000 - addr;
            }
        scnprintf(info_path,64,"/data/dumpinfo_00f0.txt");
    }
    else {
        scnprintf(info_path,64,"/data/dumpinfo.txt");
    }

    la_buf = kmalloc(REG_DUMP_SIZE, GFP_ATOMIC);
    if (!la_buf) {
         AML_PRINT_LOG_ERR("malloc buf erro\n");
         return 0;
    }
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0)) || defined (LINUX_PLATFORM)
    fp = filp_open(info_path, O_CREAT | O_WRONLY | O_TRUNC | O_SYNC, 0644);
#endif
    if (IS_ERR(fp)) {
        AML_PRINT_LOG_ERR("mactrace file(%s) open failed: PTR_ERR(fp) = %d\n", info_path, PTR_ERR(fp));
        goto err;
    }
    fp->f_pos = 0;
    memset(la_buf, 0, REG_DUMP_SIZE);
    address =(char *)(unsigned long)addr;
    if (!write_bin_flag) {
        len += scnprintf(&la_buf[len], (REG_DUMP_SIZE - len), "========dump range [0x%08x ---- 0x%08x], Size 0x%x========\n",
                        address, address + size, size);
    }
    else {
        AML_PRINT_LOG_INFO("========dump range [0x%08x ---- 0x%08x], Size 0x%x========\n", address, address + size, size);
    }

    for (i = 0; i < size / 4; i++) {
        if (write_bin_flag) {
            read_data = 0;
            read_data = wnet_vif->vif_ops.read_word((unsigned long)(address + i * 4));
            memcpy(&la_buf[len],&read_data,sizeof(unsigned int));
            len += sizeof(unsigned int);
        }
        else {
            if (((unsigned long)(address + i * 4) & 0x00fff000) == 0x00a02000 ||
                ((unsigned long)(address + i * 4) & 0x00fff000) == 0x00a03000 ||
                ((unsigned long)(address + i * 4) & 0x00fff000) == 0x00a04000 ||
                ((unsigned long)(address + i * 4) & 0x00fff000) == 0x00a06000 ||
                ((unsigned long)(address + i * 4) & 0x00fff000) == 0x00a0c000 ||
                ((unsigned long)(address + i * 4) & 0x00fff000) == 0x00a0f000 ||
                ((unsigned long)(address + i * 4) & 0x00fff000) == 0x00f05000 ) {
                continue;
            }
            len += scnprintf(&la_buf[len], (REG_DUMP_SIZE - len), "addr 0x%08x ----- value 0x%08x\n",
                             address + i * 4,  wnet_vif->vif_ops.read_word((unsigned long)(address + i * 4)));
        }

        if ((REG_DUMP_SIZE - len) < 38) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0)) || defined (LINUX_PLATFORM)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0))
            ret = kernel_write(fp, la_buf, len, &fp->f_pos);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0))
            ret = __vfs_write(fp, la_buf, len, &fp->f_pos);
#else
            ret = fp->f_op->write(fp, la_buf, len, &fp->f_pos);
#endif
#endif
            len = 0;
            memset(la_buf, 0, REG_DUMP_SIZE);
        }
    }
    if (len != 0) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0)) || defined (LINUX_PLATFORM)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0))
            ret = kernel_write(fp, la_buf, len, &fp->f_pos);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0))
            ret = __vfs_write(fp, la_buf, len, &fp->f_pos);
#else
            ret = fp->f_op->write(fp, la_buf, len, &fp->f_pos);
#endif
#endif
    }

    filp_close(fp, NULL);

err:
    if (la_buf != NULL) {
        kfree(la_buf);
    }
    return ret;
}

int aml_dump_info(struct net_device *dev, unsigned int dump_flag, unsigned int input_size)
{
    unsigned int size = 0 , addr = 0;

    if (input_size < 0) {
        AML_PRINT_LOG_ERR("input_size=%d is illegal !",input_size);
        return -1;
    }

    if (dump_flag & ICCROM_DUMP) {
        if (input_size == 0 || input_size > ICCM_ROM_LEN ) {
            size = ICCM_ROM_LEN;
        }
        else {
            size = input_size;
        }
        addr = MAC_ICCM_AHB_BASE;
        aml_mem_dump(dev, addr, size);
    }

    if (dump_flag & ICCRAM_DUMP) {
        if (input_size == 0 || input_size > ICCM_RAM_LEN ) {
            size = ICCM_RAM_LEN;
        }
        else {
            size = input_size;
        }
        addr = MAC_ICCM_AHB_BASE + ICCM_ROM_LEN;
        aml_mem_dump(dev, addr, size);
    }

    if (dump_flag & DCCM_DUMP) {
        if (input_size == 0 || input_size > DCCM_LEN ) {
            size = DCCM_LEN;
        }
        else {
            size = input_size;
        }
        addr = MAC_DCCM_AHB_BASE;
        aml_mem_dump(dev, addr, size);
    }

    if (dump_flag & SRAM_DUMP) {
        if (input_size == 0 || input_size > SRAM_LEN ) {
            size = SRAM_LEN;
        }
        else {
            size = input_size;
        }
        addr = MAC_SRAM_BASE;
        aml_mem_dump(dev, addr, size);
    }

    if (dump_flag & PKT_DUMP) {
        if (input_size == 0 || input_size > MEM_PKT_lEN ) {
            size = MEM_PKT_lEN;
        }
        else {
            size = input_size;
        }
        addr = MEM_PKT_ADDR;
        aml_mem_dump(dev, addr, size);
    }

    if (dump_flag & REG_DUMP) {
        if (input_size == 0 || input_size > REG_LEN ) {
            size = REG_LEN;
        }
        else {
            size = input_size;
        }
        addr = MAC_REG_BASE;
        aml_mem_dump(dev, addr, size);
    }

    if (dump_flag & OTHERREG_DUMP) {
        if (input_size == 0 || input_size > 0x6000 ) {
            size = 0x6000;
        }
        else {
            size = input_size;
        }
        /*ao_intf ao_ana ao_wf_pmu ao_bt_mpu efuse coexist */
        addr = 0x00f00000;
        aml_mem_dump(dev, addr, size);
    }

    return 0;
}

unsigned int get_latest_tx_status(struct wifi_mac *wifimac)
{
    unsigned int addr = 0x00000038;
    unsigned int val_con = 0;

    memcpy((unsigned char*)&val_con, (unsigned char*)&addr, sizeof(unsigned int));
    wifimac->drv_priv->net_ops->wifi_mac_get_sts(wifimac, 0, ntohl(val_con));
    return 0;
}

int aml_beacon_intvl_set(struct wlan_net_vif *wnet_vif, unsigned int set)
{
    unsigned int regdata = 0;
    int bcn_intvl =0;
    int usr_data = 0;

    usr_data = set;

    if ((wnet_vif != NULL) && (wnet_vif->vm_wmac != NULL)) {
        bcn_intvl = usr_data * 100;
        if (bcn_intvl == 0) {
            bcn_intvl = WIFINET_BINTVAL_DEFAULT;
        }
        AML_PRINT_LOG_INFO("bcn intvl %d\n", bcn_intvl);
        wnet_vif->vm_wmac->drv_priv->drv_ops.Phy_beaconinit(wnet_vif->vm_wmac->drv_priv,
                wnet_vif->wnet_vif_id, bcn_intvl);

            /* change max sleep time */
        if (usr_data != 0) {
           regdata = wnet_vif->vif_ops.read_word(RG_AON_A37);
           regdata &= ~0x00ff0000;
           regdata |= usr_data << 16;
           wnet_vif->vif_ops.write_word(RG_AON_A37, regdata);
        }
    }
    return 0;
}

void aml_iwpriv_set_recovery(unsigned int set)
{
    struct wifi_mac * wifimac = wifi_mac_get_mac_handle();

    AML_PRINT_LOG_INFO("wifi recovery now %s, set to %s", BOOL2STR_EFFECT(wifimac->drv_priv->drv_config.cfg_recovery), BOOL2STR_EFFECT(set));
    if (OTHERS2BOOL(wifimac->drv_priv->drv_config.cfg_recovery) == ENABLE && OTHERS2BOOL(set) == DISABLE) {
        wifimac->drv_priv->drv_config.cfg_recovery = DISABLE;
        os_timer_ex_cancel(&wifimac->wm_monitor_fw, CANCEL_SLEEP);
    } else if (OTHERS2BOOL(wifimac->drv_priv->drv_config.cfg_recovery) == DISABLE && OTHERS2BOOL(set) == ENABLE) {
        wifimac->drv_priv->drv_config.cfg_recovery = ENABLE;
        os_timer_ex_start(&wifimac->wm_monitor_fw);
    }
}

static void aml_iwpriv_enable_fw_log(struct wlan_net_vif *wnet_vif, unsigned int set)
{
    AML_PRINT_LOG_INFO("fw log enable:%d\n", set);

    if(set > 0)
    {
        set_reg(wnet_vif, 0x00f00004, 0x0ffbfaff);
        msleep(100);
        set_reg(wnet_vif, 0x00f00008, 0x00040500);
        msleep(100);
        set_reg(wnet_vif, 0x00f00020, 0x00000001);
        wnet_vif->vm_wmac->drv_priv->hal_priv->hal_fw_log_flag = 1;
    }
    else
    {
        set_reg(wnet_vif, 0x00f00004, 0x0ffffaff);
        msleep(100);
        set_reg(wnet_vif, 0x00f00008, 0x00000500);
        msleep(100);
        set_reg(wnet_vif, 0x00f00020, 0x00000000);
        wnet_vif->vm_wmac->drv_priv->hal_priv->hal_fw_log_flag = 0;
    }
}

int aml_set_ldpc(struct wlan_net_vif *wnet_vif, unsigned int set)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_station *sta = wnet_vif->vm_mainsta;

    if (aml_wifi_is_enable_rf_test()) {
        if (1 == set) {
            gB2BTestCasePacket.ldpc_enable = 1;
            AML_PRINT_LOG_INFO("Enable tx LDPC\n");
        } else if (0 == set){
            gB2BTestCasePacket.ldpc_enable = 0;
            AML_PRINT_LOG_INFO("Disable tx LDPC\n");
        } else {
            AML_PRINT_LOG_ERR("Invalid parameter\n");
        }
        return 0;
    }

    if (1 == set) {
        sta->sta_vhtcap |= WIFINET_VHTCAP_RX_LDPC;
        wifimac->wm_flags |=WIFINET_F_LDPC;
        AML_PRINT_LOG_INFO("Enable LDPC, if need to change, the action must be executed before connecting to ap or creating ap\n");
    } else if (0 == set) {
        sta->sta_vhtcap &= ~WIFINET_VHTCAP_RX_LDPC;
        wifimac->wm_flags &=~WIFINET_F_LDPC;
        AML_PRINT_LOG_INFO("Disable LDPC, if need to change, the action must be executed before connecting to ap or creating ap\n");
    } else {
        AML_PRINT_LOG_ERR("Invalid parameter\n");
    }
    return 0;
}

#if defined(SU_BF) || defined(MU_BF)
int aml_set_beamforming(struct wlan_net_vif *wnet_vif, unsigned int set1,unsigned int set2)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct hw_interface *hif = hif_get_hw_interface();
    int usr_data1 = 0;
    int usr_data2 = 0;
    unsigned int reg_data = 0;

    usr_data1 = set1;
    usr_data2 = set2;

    if (usr_data2 > 4) {
        AML_PRINT_LOG_INFO("support max spatial is 4 !\n");
        wifimac->max_spatial = 4;
    } else if (usr_data2 <= 0) {
        AML_PRINT_LOG_INFO("min spatial is 1 !\n");
        wifimac->max_spatial = 1;
    } else {
        wifimac->max_spatial = usr_data2;
    }
    AML_PRINT_LOG_INFO("set spatial %d \n", wifimac->max_spatial);

    if(usr_data1 < 4) {
        if (((usr_data1 & 0xF) & BIT(0)) == BIT(0)) {
            wifimac->wm_flags_ext2 |= WIFINET_VHTCAP_SU_BFMEE;
            AML_PRINT_LOG_INFO(" enable su mimo\n");
        } else {
            wifimac->wm_flags_ext2 &= ~WIFINET_VHTCAP_SU_BFMEE;
            AML_PRINT_LOG_INFO("disable su mimo\n");
        }

        if (((usr_data1 & 0xF) & BIT(1)) == BIT(1)) {
            wifimac->wm_flags_ext2 |= WIFINET_VHTCAP_MU_BFMEE;
                AML_PRINT_LOG_INFO("enable mu mimo\n");
        } else {
            wifimac->wm_flags_ext2 &= ~WIFINET_VHTCAP_MU_BFMEE;
            AML_PRINT_LOG_INFO("disable mu mimo\n");
        }

        if (usr_data1 != 0) {
            //enable bmfm
            reg_data = hif->hif_ops.hi_read_word(0x00a00224);
            reg_data &= ~ BIT(28);
            hif->hif_ops.hi_write_word(0x00a00224, reg_data);

            reg_data = hif->hif_ops.hi_read_word(0x00a092a4);
            reg_data &= ~ BIT(28);
            hif->hif_ops.hi_write_word(0x00a092a4, reg_data);
        } else {
            //disable bmfm
            reg_data = hif->hif_ops.hi_read_word(0x00a00224);
            reg_data |= BIT(28);
            hif->hif_ops.hi_write_word(0x00a00224, reg_data);

            reg_data = hif->hif_ops.hi_read_word(0x00a092a4);
            reg_data |= BIT(28);
            hif->hif_ops.hi_write_word(0x00a092a4, reg_data);
        }
    } else {
        AML_PRINT_LOG_ERR("initial parameter!\n");
    }
    return 0;

}
#endif


static unsigned int
aml_iwpriv_legacy_2g_rate_to_bitmap(int legacy)
{
    switch(legacy)
    {
        case 1:
            return 0x00;
        case 2:
            return 0x01;
        case 5:
            return 0x02;
        case 11:
            return 0x03;
        case 6:
            return 0x04;
        case 9:
            return 0x05;
        case 12:
            return 0x06;
        case 18:
            return 0x07;
        case 24:
            return 0x08;
        case 36:
            return 0x09;
        case 48:
            return 0x0a;
        case 54:
            return 0x0b;
        default:
            return 0;
    }
}

static unsigned int
aml_iwpriv_legacy_5g_rate_to_bitmap(int legacy)
{
    switch(legacy)
    {
        case 6:
            return 0x00;//6M;
        case 9:
            return 0x01;//9M;
        case 12:
            return 0x02;//12M;
        case 18:
            return 0x03;//18M;
        case 24:
            return 0x04;//24M;
        case 36:
            return 0x05;//36M;
        case 48:
            return 0x06;//48M;
        case 54:
            return 0x07;//54M;
        default:
            return 0;
    }
}


static unsigned int
aml_iwpriv_ht_rate_to_bitmap(int ht_mcs)
{
    switch(ht_mcs)
    {
        case 0:
            return 0x00;
        case 1:
            return 0x01;
        case 2:
            return 0x02;
        case 3:
            return 0x03;
        case 4:
            return 0x04;
        case 5:
            return 0x05;
        case 6:
            return 0x06;
        case 7:
            return 0x07;
        default:
            return 0;
    }
}


static unsigned int
aml_iwpriv_vm_vht_rate_to_bitmap(int vht_mcs)
{
    switch(vht_mcs)
    {
        case 0:
            return 0x00;
        case 1:
            return 0x01;
        case 2:
            return 0x02;
        case 3:
            return 0x03;
        case 4:
            return 0x04;
        case 5:
            return 0x05;
        case 6:
            return 0x06;
        case 7:
            return 0x07;
        case 8:
            return 0x08;
        case 9:
            return 0x09;
        default:
            return 0;
    }
}


int
aml_iwpriv_set_lagecy_bitrate_mask(struct net_device *dev, unsigned int set)
{
    int band = 0;
    struct cfg80211_bitrate_mask mask;
    struct wlan_net_vif *wnet_vif = netdev_priv(dev);
    memset(&mask, 0, sizeof(struct cfg80211_bitrate_mask));
    mask.control[band].legacy = (1<<aml_iwpriv_legacy_2g_rate_to_bitmap(set));
    AML_PRINT_LOG_INFO("opmode %d, band %d\n", wnet_vif->vm_opmode, band);
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0)
    vm_cfg80211_set_bitrate_mask(NULL, dev, NULL, &mask);
#else
    vm_cfg80211_set_bitrate_mask(NULL, dev, 0, NULL, &mask);
#endif

    return 0;
}


int
aml_iwpriv_set_ht_bitrate_mask(struct net_device *dev, unsigned int set)
{
    int band = 0;
    struct cfg80211_bitrate_mask mask;
    struct wlan_net_vif *wnet_vif = netdev_priv(dev);
    memset(&mask, 0, sizeof(struct cfg80211_bitrate_mask));

    if (wnet_vif->vm_state == WIFINET_S_CONNECTED) {
        band = WIFINET_IS_CHAN_5GHZ(wnet_vif->vm_curchan)? NL80211_BAND_5GHZ:NL80211_BAND_2GHZ;
    } else {
        band = NL80211_BAND_2GHZ;
    }

    AML_PRINT_LOG_INFO("opmode %d, band %d\n", wnet_vif->vm_opmode, band);
    mask.control[band].ht_mcs[0] = (1<<aml_iwpriv_ht_rate_to_bitmap(set));
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0)
    vm_cfg80211_set_bitrate_mask(NULL, dev, NULL, &mask);
#else
    vm_cfg80211_set_bitrate_mask(NULL, dev, 0, NULL, &mask);
#endif

    return 0;
}


int
aml_iwpriv_set_vht_bitrate_mask(struct net_device *dev, unsigned int set)
{
    int band = 0;
    struct cfg80211_bitrate_mask mask;
    struct wlan_net_vif *wnet_vif = netdev_priv(dev);
    memset(&mask, 0, sizeof(struct cfg80211_bitrate_mask));

    if (wnet_vif->vm_state == WIFINET_S_CONNECTED) {
        band = WIFINET_IS_CHAN_5GHZ(wnet_vif->vm_curchan)? NL80211_BAND_5GHZ:NL80211_BAND_2GHZ;
    } else {
        band = NL80211_BAND_2GHZ;
    }

    AML_PRINT_LOG_INFO("opmode %d, band %d\n", wnet_vif->vm_opmode, band);
    mask.control[band].vht_mcs[0] = (1<<aml_iwpriv_vm_vht_rate_to_bitmap(set));
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0)
    vm_cfg80211_set_bitrate_mask(NULL, dev, NULL, &mask);
#else
    vm_cfg80211_set_bitrate_mask(NULL, dev, 0, NULL, &mask);
#endif

    return 0;
}


void aml_iwpriv_set_rate_auto(struct wlan_net_vif *wnet_vif)
{
    wnet_vif->vm_fixed_rate.rateinfo = 0;
    wnet_vif->vm_fixed_rate.mode = WIFINET_FIXED_RATE_NONE;
    wnet_vif->vm_change_rate_enable = 1;
    AML_PRINT_LOG_INFO("enable autorate\n");
}

void aml_iwpriv_set_uapsd(struct wlan_net_vif *wnet_vif, unsigned int set)
{
    if ((unsigned char)set != 0) {
        WIFINET_VMAC_UAPSD_ENABLE(wnet_vif);
        AML_PRINT_LOG_INFO("enable ap uapsd\n ");

    } else {
        WIFINET_VMAC_UAPSD_DISABLE(wnet_vif);
        AML_PRINT_LOG_INFO("disable ap uapsd\n ");
    }
    wnet_vif->vm_flags |= WIFINET_F_WMEUPDATE;
}

unsigned char aml_iwpriv_set_band(unsigned int set)
{
    struct drv_private *drv_priv = drv_get_drv_priv();
    drv_priv->drv_config.cfg_band = set;
    AML_PRINT_LOG_INFO("band %d\n ", set);
    return 0;
}

unsigned char aml_iwpriv_set_initial_gain_change_hang(unsigned int set)
{
    aml_txt_parameter.initial_gain_change_disable = set;

    if (aml_txt_parameter.initial_gain_change_disable) {
        AML_PRINT_LOG_INFO("initial_gain_change invalid\n ");

    } else {
        AML_PRINT_LOG_INFO("initial_gain_change valid\n ");
    }
    return 0;
}

unsigned char aml_iwpriv_set_tx_power_change_hang(unsigned int set)
{
    aml_txt_parameter.tx_power_change_disable = set;

    if (aml_txt_parameter.tx_power_change_disable) {
        AML_PRINT_LOG_INFO("tx_power_change invalid \n");

    } else {
        AML_PRINT_LOG_INFO("tx_power_change valid \n");
    }
    return 0;
}

unsigned char aml_iwpriv_get_band(void)
{
    struct drv_private *drv_priv = drv_get_drv_priv();
    return drv_priv->drv_config.cfg_band;
}

unsigned char aml_iwpriv_set_mac_mode(unsigned int set)
{
    struct drv_private *drv_priv = drv_get_drv_priv();
    drv_priv->drv_config.cfg_mac_mode = set;
    return 0;
}

extern bool isFirstWrtFwlog;
extern unsigned short print_ctl;
static int aml_iwpriv_send_para1(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{

    struct wlan_net_vif *wnet_vif = NULL;
    struct wifi_mac *wifimac = NULL;

    int *param = (int *)extra;
    char* str[2] = {0};
    int sub_cmd = param[0];
    int set = param[1];
    char buf[30] = {0};
    int len = 0;


    AML_PRINT_LOG_INFO("sub_cmd %d, value %d\n", param[0], param[1]);

    wifimac = wifi_mac_get_mac_handle();
    wnet_vif = aml_iwpriv_get_vif(dev->name);

    switch (sub_cmd) {
        case AML_IWP_11H:
            aml_ap_set_11h(set);
            break;

        case AML_IWP_ARP_RX:
            break;

        case AML_IWP_ROAM_THRESH_2G:
            /*e.g '-80' need 3 char in string, added '\0', so need + 4 */
            len = strlen("set_roam_thr_2g ") + 4;
            snprintf(buf, len, "set_roam_thr_2g %d", set);
            AML_PRINT_LOG_INFO("buf %s\n", buf);
            aml_set_roaming_threshold_2g(wnet_vif, buf, len);
            break;

        case AML_IWP_ROAM_THRESH_5G:
            /*e.g '-80' need 3 char in string, added '\0', so need + 4 */
            len = strlen("set_roam_thr_5g ") + 4;
            snprintf(buf, len, "set_roam_thr_5g %d", set);
            AML_PRINT_LOG_INFO("buf %s\n", buf);
            aml_set_roaming_threshold_5g(wnet_vif, buf, len);
            break;

        case AML_IWP_ROAM_MODE:
            len = strlen("set_roam_mode ") + 4;
            snprintf(buf, len, "set_roam_mode %d", set);
            AML_PRINT_LOG_INFO("buf %s\n", buf);
            aml_set_roaming_mode(wnet_vif, buf, len);
            break;

        case AML_IWP_MARK_DFS_CHAN:
            len = strlen("mark_dfs_chan ") + 4;
            snprintf(buf, len, "mark_dfs_chan %d", set);
            AML_PRINT_LOG_INFO("buf %s\n", buf);
            aml_mark_dfs_channel(wnet_vif, buf, len);
            break;

        case AML_IWP_UNMARK_DFS_CHAN:
            len = strlen("unmark_dfs_chan ") + 4;
            snprintf(buf, len, "unmark_dfs_chan %d", set);
            AML_PRINT_LOG_INFO("buf %s\n", buf);
            aml_unmark_dfs_channel(wnet_vif, buf, len);
            break;

        case AML_IWP_WEAK_THR_NARROW:
            len = strlen("set_weak_thr_nb ") + 4;
            snprintf(buf, len, "set_weak_thr_nb %d", set);
            AML_PRINT_LOG_INFO("buf %s\n", buf);
            aml_set_signal_power_weak_thresh_for_narrow_bandwidth(wnet_vif, buf, len);
            break;

        case AML_IWP_WEAK_THR_WIDE:
            len = strlen("set_weak_thr_wb ") + 4;
            snprintf(buf, len, "set_weak_thr_wb %d", set);
            AML_PRINT_LOG_INFO("buf %s\n", buf);
            aml_set_signal_power_weak_thresh_for_wide_bandwidth(wnet_vif, buf, len);
            break;

        case AML_IWP_EAT_COUNT:
            len = strlen("set_eat_count ") + 4;
            snprintf(buf, len, "set_eat_count %d", set);
            AML_PRINT_LOG_INFO("buf %s\n", buf);
            aml_set_eat_count_max(wnet_vif, buf, len);
            break;

        case AML_IWP_AGGR_THRESH:
            len = strlen("set_aggr_thresh ") + 4;
            snprintf(buf, len, "set_aggr_thresh %d", set);
            AML_PRINT_LOG_INFO("buf %s\n", buf);
            aml_set_aggr_thresh(wnet_vif, buf, len);
            break;

        case AML_IWP_HEART_INTERVAL:
            len = strlen("set_hrt_int ") + 4;
            snprintf(buf, len, "set_hrt_int %d", set);
            AML_PRINT_LOG_INFO("buf %s\n", buf);
            aml_set_hrtimer_interval(wnet_vif, buf, len);
            break;

        case AML_IWP_BSS_COEX:
            len = strlen("set_bss_coex");
            snprintf(buf, len, "set_bss_coex");
            AML_PRINT_LOG_INFO("buf %s\n", buf);
            aml_sta_send_coexist_mgmt(wnet_vif, buf, len);
            break;

        case AML_IWP_WMM_AC_DELTS:
            len = strlen("wmm_ac_delts ") + 4;
            snprintf(buf, len, "wmm_ac_delts %d", set);
            AML_PRINT_LOG_INFO("buf %s\n", buf);
            aml_wmm_ac_delts(wnet_vif, buf, len);
            break;

        case AML_IWP_SHORT_GI:
            str[0] = "off";
            str[1] = "on";
            len = strlen("set_short_gi ") + strlen(str[set]) + 1;
            snprintf(buf, len, "set_short_gi %s", str[set]);
            AML_PRINT_LOG_INFO("buf %s\n", buf);
            aml_set_short_gi(wnet_vif, buf, len);
            break;

        case AML_IWP_DYNAMIC_BW:
            str[0] = "off";
            str[1] = "on";
            len = strlen("set_dynamic_bw ") + strlen(str[set]) + 1;
            snprintf(buf, len, "set_dynamic_bw %s", str[set]);
            AML_PRINT_LOG_INFO("buf %s\n", buf);
            aml_set_dynamic_bw(wnet_vif, buf, len);
            break;

        case AML_IWP_TXAGGR_STA:
            str[0] = "all";
            str[1] = "reset";
            len = strlen("get_txaggr_sta ") + strlen(str[set]) + 1;
            snprintf(buf, len, "get_txaggr_sta %s", str[set]);
            AML_PRINT_LOG_INFO("buf %s\n", buf);
            aml_get_drv_txaggr_status(wnet_vif, buf, len);
            break;

        case AML_IWP_SET_BCN_INTERVAL:
            aml_beacon_intvl_set(wnet_vif, set);
            break;

        case AML_IWP_SET_LDPC:
            aml_set_ldpc(wnet_vif, set);
            break;

        case AML_IWP_SET_SCAN_TIME_IDLE:
            if (set) {
                wnet_vif->vm_scan_time_idle = (unsigned char)set;
                wifi_mac_set_scan_time(wnet_vif);
            }
            AML_PRINT_LOG_INFO("vid:%d set scan_time_idle = %d\n ", wnet_vif->wnet_vif_id, wnet_vif->vm_scan_time_idle);
            break;

        case AML_IWP_SET_SCAN_TIME_CONNECT:
            if (set) {
                wnet_vif->vm_scan_time_connect = (unsigned char)set;
                wifi_mac_set_scan_time(wnet_vif);
            }
            AML_PRINT_LOG_INFO("vid:%d set scan_time_connect = %d\n ", wnet_vif->wnet_vif_id, wnet_vif->vm_scan_time_connect);
            break;

        case AML_IWP_SET_SCAN_HANG:
            wnet_vif->vm_scan_hang = (unsigned char)set;
            AML_PRINT_LOG_INFO("vid:%d vm_scan_hang:%d\n ", wnet_vif->wnet_vif_id, wnet_vif->vm_scan_hang);
            break;

        case AML_IWP_EN_BTWIFI_COEX:
            AML_PRINT_LOG_INFO("coexist en= %d\n ", set);
            wifimac->drv_priv->hal_priv->hal_ops.phy_set_coexist_en(set);
            break;

        case AML_IWP_SET_COEXIST_MAX_MISS_BCN_CNT:
            wifimac->drv_priv->hal_priv->hal_ops.phy_set_coexist_max_miss_bcn(set);
            break;

        case AML_IWP_SET_COEXIST_REQ_TIMEOUT:
            wifimac->drv_priv->hal_priv->hal_ops.phy_set_coexist_req_timeslice_timeout_value(set);
            AML_PRINT_LOG_INFO("set req timeout value= %d\n ", set);
            break;

        case AML_IWP_SET_COEXIST_NOT_GRANT_WEIGHT:
            wifimac->drv_priv->hal_priv->hal_ops.phy_set_coexist_not_grant_weight(set);
            AML_PRINT_LOG_INFO("set coexist_not_grant_weight= %d\n ", set);
            break;

        case AML_IWP_SET_RATE_LEGACY:
            aml_iwpriv_set_lagecy_bitrate_mask(dev, set);
            break;

        case AML_IWP_SET_RATE_HT:
            aml_iwpriv_set_ht_bitrate_mask(dev, set);
            break;

        case AML_IWP_SET_RATE_VHT:
            aml_iwpriv_set_vht_bitrate_mask(dev, set);
            break;

        case AML_IWP_SET_POWER:
            wifi_mac_pwrsave_set_inactime(wnet_vif, set);
            break;

        case AML_IWP_SET_CHL_RSSI:
            wifimac->drv_priv->drv_ops.set_channel_rssi(wifimac->drv_priv, set, 0);
            break;

        case AML_IWP_SET_BURST:
            wifimac->drv_priv->drv_config.cfg_burst_ack = set;
            AML_PRINT_LOG_INFO("iwpriv set burst %d\n", set);
            break;

        case AML_IWP_SET_UAPSD:
            aml_iwpriv_set_uapsd(wnet_vif, set);
            break;

        case AML_IWP_SET_PT_RX_START:
            wnet_vif->vif_ops.pt_rx_start(set);
            break;

        case AML_IWP_SET_SCAN_PRI:
            /* bit31-bit16 : minimal  priority
               bit15:bit0: max priority */
            wifimac->drv_priv->hal_priv->hal_ops.phy_set_coexist_scan_priority_range(set);
            break;

        case AML_IWP_SET_BE_BK_NOQOS_PRI:
            /* bit31-bit16 : minimal  priority
               bit15:bit0: max priority */
            wifimac->drv_priv->hal_priv->hal_ops.phy_set_coexist_be_bk_noqos_priority_range(set);
            break;

        case AML_IWP_SET_FETCH_PKT_METHOD:
            wnet_vif->vm_mainsta->sta_fetch_pkt_method = (unsigned char)set;
            AML_PRINT_LOG_INFO("iwpriv set pkt method %d\n", set);
            break;

        case AML_IWP_SET_FRAG_THRESHOLD:
            if ((unsigned short)set > 0) {
                wnet_vif->vm_fragthreshold = (unsigned short)set;
            }
            AML_PRINT_LOG_INFO("iwpriv set frag thr %d\n", wnet_vif->vm_fragthreshold);
            break;

        case AML_IWP_SET_PREAMBLE_TYPE:
            phy_set_preamble_type((unsigned char)set);
            AML_PRINT_LOG_INFO("iwpriv set preamble type %d\n", set);
            break;

        case AML_IWP_SET_FIX_BAND:
            aml_iwpriv_set_band(set);
            break;

        case AML_IWP_SET_GAIN:
            aml_iwpriv_set_initial_gain_change_hang(set);
            break;

        case AML_IWP_SET_TPC:
            aml_iwpriv_set_tx_power_change_hang(set);
            break;

        case AML_IWP_SET_TXPW_PLAN:
            wifimac_set_tx_pwr_plan(set);
            break;

        case AML_IWP_SET_MAC_MODE:
            aml_iwpriv_set_mac_mode(set);
            break;

        case AML_IWP_GET_HOST_LOG:
            AML_PRINT_LOG_INFO(" wifi host log %s\n",param[1]? "on":"off");
            break;

        case AML_IWP_GET_FW_LOG:
            AML_PRINT_LOG_INFO(" wifi fw log %s\n",param[1]? "on":"off");
            if (set == 1) {
                wifimac->drv_priv->hal_priv->hal_ops.hal_set_fwlog_cmd(WRITE_SRAM_MODE);
                wifimac->drv_priv->hal_priv->hal_ops.hal_set_fwlog_cmd(OPEN_AUTO_PRINT);
                wifimac->drv_priv->hal_priv->g_get_fw_log = true;
            } else if (set == 0) {
                wifimac->drv_priv->hal_priv->hal_ops.hal_set_fwlog_cmd(UART_MODE);
                wifimac->drv_priv->hal_priv->g_get_fw_log  = false;
                isFirstWrtFwlog = true;
                print_ctl = 0;
            }
            break;

        case AML_IWP_SET_RECOVERY:
            aml_iwpriv_set_recovery(set);
            break;
        case AML_IWP_ENABLE_FW_LOG:
            aml_iwpriv_enable_fw_log(wnet_vif,set);
            break;
        case AML_IWP_GET_SPEC_REGS:
            aml_iwpriv_get_spec_regs(wnet_vif, set);
            break;
        case AML_IWP_SET_RX_SIZE:
            wifimac->wm_manual_rx_bufsize = set;
            break;
        case AML_IWP_SET_TX_BW:
            aml_iwpriv_set_tx_bw(set);
            break;
        case AML_IWP_SET_TX_MODE:
            aml_iwpriv_set_tx_mode(set);
            break;
        case AML_IWP_SET_TX_RATE:
            aml_iwpriv_set_tx_rate(set);
            break;
        case AML_IWP_SET_TX_TYPE:
            aml_iwpriv_set_tx_type(set);
            break;
        case AML_IWP_SET_TX_LEN:
            aml_iwpriv_set_tx_len(set);
            break;
        case AML_IWP_SET_TX_NUM:
            aml_iwpriv_set_tx_num(set);
            break;
        case AML_IWP_PT_RX_START:
            aml_iwpriv_pt_rx_start(dev, set);
            break;
        case AML_IWP_SET_STBC:
            aml_iwpriv_set_stbc(dev, set);
            break;
        case AML_IWP_SET_TX_PWR:
            aml_iwpriv_set_tx_pwr(set);
            break;
        case AML_IWP_SET_DELTA_TX_PWR:
            aml_iwpriv_set_delta_tx_pwr(set);
            break;
        case AML_IWP_SET_CF_END:
            aml_iwpriv_set_cf_end(wnet_vif, set);
            break;
        case AML_IWP_SET_FLOW_CTRL:
            aml_iwpriv_set_flow_ctrl(wnet_vif, set);
            break;
        case AML_IWP_SET_WFA_MODE:
            aml_iwpriv_set_wfa_mode(wnet_vif, set);
            break;

    }

    return 0;
}

static int aml_iwpriv_send_para2(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{

    struct wlan_net_vif *wnet_vif = NULL;
    struct wifi_mac *wifimac = NULL;

    int *param = (int *)extra;
    int sub_cmd = param[0];
    int set1 = param[1];
    int set2 = param[2];


    AML_PRINT_LOG_INFO("sub_cmd %d, value %d %d\n", param[0], param[1], param[2]);

    wifimac = wifi_mac_get_mac_handle();
    wnet_vif = aml_iwpriv_get_vif(dev->name);

    switch (sub_cmd) {
        case AML_IWP_SET_REG:
            if ((set1 == 0x7fffffff) || (set2 == 0x7fffffff)) {
                AML_PRINT_LOG_INFO("********************************************************************************************************\n");
                AML_PRINT_LOG_INFO("You are using a legacy iwpriv tool, strongly suggest using a latest iwpriv tool\n");
                AML_PRINT_LOG_INFO("You also can using the legacy tool as below:\n iwpriv wlan0 set_reg 0xff000c80 0xff000c80 -> iwpriv wlan0 set_reg_legacy 0xff00 0x00c80 0xff0 0x00c80\n");
                AML_PRINT_LOG_INFO("********************************************************************************************************\n");
        }
        set_reg(wnet_vif, set1, set2);
        break;
#if defined(SU_BF) || defined(MU_BF)
        case AML_IWP_SET_BEAMFORMING:
            aml_set_beamforming(wnet_vif, set1,set2);
            break;
#endif
        case AML_IWP_SET_EFUSE:
            aml_iwpriv_set_efuse(set1,set2);
            break;
        case AML_IWP_SET_EFUSE_DOMAIN:
            aml_iwpriv_set_efuse_domain(set1,set2);
            break;
        case AML_IWP_SET_PT_EFUSE:
            aml_iwpriv_set_pt_efuse(set1,set2);
            break;
        case AML_IWP_MEM_DUMP:
            aml_mem_dump(dev, set1, set2);
            break;

        case AML_IWP_SET_PT_CALIBRATION:
            aml_iwpriv_set_pt_calibration(set1, set2);
            break;

        case AML_IWP_SET_TX_PATH:
            aml_iwpriv_set_tx_path(wnet_vif, set1, set2);
            break;

        case AML_IWP_SET_RX_PATH:
            aml_iwpriv_set_rx_path(wnet_vif, set1, set2);
            break;

        case AML_IWP_SET_BT_DIGITAL_GAIN_EFUSE:
            aml_iwpriv_set_bt_digital_gain_efuse(wnet_vif, set1, set2);
            break;
    }

    return 0;
}

static int aml_iwpriv_set_reg_legacy(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{

    struct wlan_net_vif *wnet_vif = NULL;
    struct wifi_mac *wifimac = NULL;

    int *param = (int *)extra;
    int sub_cmd = 0;
    int set0 = param[0];
    int set1 = param[1];
    int set2 = param[2];
    int set3 = param[3];
    int legacy_set1 = 0;
    int legacy_set2 = 0;


    AML_PRINT_LOG_INFO("sub_cmd %d, value1 %d, value2 %d, value3 %d\n", param[0], param[1], param[2], param[3]);

    wifimac = wifi_mac_get_mac_handle();
    wnet_vif = aml_iwpriv_get_vif(dev->name);

    sub_cmd = AML_IWP_LEGACY_SET_REG;
    switch (sub_cmd) {
        case AML_IWP_LEGACY_SET_REG:
            legacy_set1 = set1 | set0 << 16;
            legacy_set2 = set3 | set2 << 16;
            set_reg(wnet_vif, legacy_set1, legacy_set2);
            break;
    }
    return 0;
}

static void aml_iwpriv_get_key_entry(int set1, int set2, int set3)
{
    struct wifi_mac* wifimac = wifi_mac_get_mac_handle();
    struct wifi_station *sta = NULL;
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned char vid = (unsigned char) ((set1&0xffff0000) >> 16);
    unsigned char key_id = (unsigned char)set1&0xffff;
    unsigned short sta_associd = set2;
    unsigned char is_ukey = set3;


    AML_PRINT_LOG_INFO("vid:%d  key_id:%d  is_ukey:%d associd:0x%x\n", vid, key_id, is_ukey, sta_associd);
    wnet_vif = wifi_mac_get_wnet_vif_by_vid(wifimac, vid);
    if (!wnet_vif) {
        AML_PRINT_LOG_ERR("ERR wnet_vif\n");
        return;
    }

    if (is_ukey) {
        if (wnet_vif->vm_opmode == WIFINET_M_HOSTAP) {
             //phy_read_key_table_info(vid, sta_id, is_ukey);
             sta = wifi_mac_get_sta_by_staid(&wnet_vif->vm_sta_tbl, wnet_vif, sta_associd);

             if (sta) {
                 wifi_mac_key_ctrl_read_key_table_by_condition(vid, key_id, is_ukey, sta->sta_macaddr);
             }
             else {
                 AML_PRINT_LOG_ERR("no such sta\n");
             }
         } else if (wnet_vif->vm_opmode == WIFINET_M_STA && wnet_vif->vm_state == WIFINET_S_CONNECTED) {

             if (wnet_vif->vm_mainsta) {
                 wifi_mac_key_ctrl_read_key_table_by_condition(vid, key_id, is_ukey, wnet_vif->vm_mainsta->sta_macaddr);
             }
             else {
                 AML_PRINT_LOG_ERR("no such sta\n");
             }
         }
    } else if (!is_ukey){
        wifi_mac_key_ctrl_read_key_table_by_condition(vid, key_id, is_ukey, NULL);
    }

}

int aml_iwpriv_efuse_config_temporary(unsigned int efuse_addr, unsigned int efuse_data, unsigned char enable)
{
    struct WF5G_Txpwr_Param wf5g_txpwr_param = {0};

    if (!efuse_addr && !efuse_data && !enable) {

        AML_PRINT_LOG_INFO("disable efuse tmp config \n");

        //send cmd to fw to update efuse data
        efuse_cfg_param.Cmd = EFUSE_CFG_CMD;
        efuse_cfg_param.flag = EFUSE_CFG_DISABLE;
        hi_set_cmd((unsigned char *)&efuse_cfg_param, sizeof(struct Efuse_Cfg_Param));

        //send cmd to fw to start calculate
        memcpy(&wf5g_txpwr_param.wf5g_pwr_tbl, &g_wf5g_txpwr_param.wf5g_pwr_tbl, sizeof(wf5g_txpwr_param.wf5g_pwr_tbl));
        wf5g_txpwr_param.Cmd = WF5G_TXPWR_PARAM_CMD;
        hi_set_cmd((unsigned char *)&wf5g_txpwr_param, sizeof(struct WF5G_Txpwr_Param));

        return 0;
    }

    if (efuse_addr == EFUSE_ADDR_9) {
        efuse_cfg_param.efuse_9 = efuse_data;
    } else if (efuse_addr == EFUSE_ADDR_A) {
        efuse_cfg_param.efuse_a = efuse_data;
    }  else if (efuse_addr == EFUSE_ADDR_B) {
        efuse_cfg_param.efuse_b = efuse_data;
    }  else if (efuse_addr == EFUSE_ADDR_C) {
        efuse_cfg_param.efuse_c = efuse_data;
    }  else if (efuse_addr == EFUSE_ADDR_D) {
        efuse_cfg_param.efuse_d = efuse_data;
    }  else if (efuse_addr == EFUSE_ADDR_E) {
        efuse_cfg_param.efuse_e = efuse_data;
    }  else {
        AML_PRINT_LOG_INFO("WARNING: invalid efuse addr\n");
    }

    if (enable == EFUSE_CFG_ENABLE) {

        AML_PRINT_LOG_INFO("config efuse_9:%08x \n",efuse_cfg_param.efuse_9);
        AML_PRINT_LOG_INFO("config efuse_a:%08x  \n",efuse_cfg_param.efuse_a);
        AML_PRINT_LOG_INFO("config efuse_b:%08x  \n",efuse_cfg_param.efuse_b);
        AML_PRINT_LOG_INFO("config efuse_c:%08x  \n",efuse_cfg_param.efuse_c);
        AML_PRINT_LOG_INFO("config efuse_d:%08x  \n",efuse_cfg_param.efuse_d);
        AML_PRINT_LOG_INFO("config efuse_e:%08x  \n",efuse_cfg_param.efuse_e);

        //send cmd to fw to update efuse data
        efuse_cfg_param.Cmd = EFUSE_CFG_CMD;
        efuse_cfg_param.flag = EFUSE_CFG_ENABLE;
        hi_set_cmd((unsigned char *)&efuse_cfg_param, sizeof(struct Efuse_Cfg_Param));

        //send cmd to fw to start calculate
        memcpy(&wf5g_txpwr_param.wf5g_pwr_tbl, &g_wf5g_txpwr_param.wf5g_pwr_tbl, sizeof(wf5g_txpwr_param.wf5g_pwr_tbl));
        wf5g_txpwr_param.Cmd = WF5G_TXPWR_PARAM_CMD;
        hi_set_cmd((unsigned char *)&wf5g_txpwr_param, sizeof(struct WF5G_Txpwr_Param));
    }

    return 0;
}

static int aml_iwpriv_send_para3(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
    struct wlan_net_vif *wnet_vif = NULL;
    struct wifi_mac *wifimac = NULL;

    int *param = (int *)extra;
    int sub_cmd = param[0];
    int set1 = param[1];
    int set2 = param[2];
    int set3 = param[3];

    wifimac = wifi_mac_get_mac_handle();
    wnet_vif = aml_iwpriv_get_vif(dev->name);

    AML_PRINT_LOG_INFO("sub_cmd:[%d], param1:[%x], param2:[%x], param3:[%x]\n", param[0], param[1], param[2],param[3]);

    switch (sub_cmd) {
        case AML_IWP_GET_KEY_ENTRY:
            aml_iwpriv_get_key_entry(set1, set2, set3);
            break;
        case AML_IWP_SET_EFUSE_TMP:
            aml_iwpriv_efuse_config_temporary(set1, set2, set3);
            break;
        default:
            break;
    }

    return 0;
}

int aml_iwpriv_legacy_get_reg(struct net_device *dev, char *str_addr, union iwreq_data *wrqu, char *extra)
{
    char **cmd_addr;
    int param_count;
    char sep = ' ';
    unsigned int addr = 0;
    unsigned int reg_val = 0;
    struct wlan_net_vif *wnet_vif = NULL;

    wnet_vif = aml_iwpriv_get_vif(dev->name);

    cmd_addr = aml_cmd_char_parse(sep, str_addr, &param_count);

    if (param_count < 2) {
        AML_PRINT_LOG_ERR("param_count:%d input invalid!\n",param_count);

        wrqu->data.length = scnprintf(extra, IW_PRIV_SIZE_MASK, "   input invalid\n eg: iwpriv wlan0 get_reg_legacy \"0xff00 0x0c80\" ");
        wrqu->data.length++;

        kfree(cmd_addr);
        return -EINVAL;
    }

    addr = (simple_strtoul(cmd_addr[0],NULL,0) << 16) | simple_strtoul(cmd_addr[1],NULL,0);
    reg_val = get_reg(wnet_vif, addr);

    wrqu->data.length = scnprintf(extra, IW_PRIV_SIZE_MASK, "0x%08x", reg_val);
    wrqu->data.length++;

    kfree(cmd_addr);

    return 0;
}

int aml_iwpriv_get_reg(struct net_device *dev, char *str_addr, union iwreq_data *wrqu, char *extra)
{
    unsigned int addr = 0;
    unsigned int reg_val = 0;
    struct wlan_net_vif *wnet_vif = NULL;

    addr = simple_strtol(str_addr, NULL, 0);

    if (addr == 0x7fffffff) {
        AML_PRINT_LOG_INFO("******************************************************************************\n");
        AML_PRINT_LOG_INFO("You are using a legacy iwpriv tool, strongly suggest using a latest iwpriv one\n");
        AML_PRINT_LOG_INFO("You also can using the legacy tool as below:\n iwpriv wlan0 get_reg 0xff000c80 -> iwpriv wlan0 get_reg_legacy \"0xff00 0x00c80\" \n");
        AML_PRINT_LOG_INFO("******************************************************************************\n");
    }
    wnet_vif = aml_iwpriv_get_vif(dev->name);
    reg_val = get_reg(wnet_vif, addr);

    wrqu->data.length = scnprintf(extra, IW_PRIV_SIZE_MASK, "0x%08x", reg_val);
    wrqu->data.length++;

    return 0;
}

int aml_iwpriv_get_performance_info(struct net_device *dev , union iwreq_data *wrqu, char *extra)
{
    //unsigned int addr = 0;
    unsigned int reg_val1 = 0;
    unsigned int reg_val2 = 0;
    unsigned int len = 0;
    struct wlan_net_vif *wnet_vif = NULL;
    struct hw_interface* hif = hif_get_hw_interface();

    wnet_vif = aml_iwpriv_get_vif(dev->name);
    reg_val1 = get_reg(wnet_vif, 0xa0816c);
    reg_val2 = get_reg(wnet_vif, 0xa081a4);
    len += scnprintf(extra + len, IW_PRIV_SIZE_MASK, " ig0x%x,lpf0x%x\n",reg_val1,reg_val2);


    reg_val1 = get_reg(wnet_vif, DF_AGC_REG_A29);
    reg_val2 = get_reg(wnet_vif, DF_AGC_REG_A30);
    len += scnprintf(extra + len, IW_PRIV_SIZE_MASK, " ct0x%x,ant0x%x\n",reg_val1,reg_val2);

    reg_val1 = get_reg(wnet_vif, 0xf01024);
    reg_val2 = get_reg(wnet_vif, DF_AGC_REG_A14);
    len += scnprintf(extra + len, IW_PRIV_SIZE_MASK, " fo 0x%x,ag0x%x\n",reg_val1,reg_val2);

    reg_val1 = get_reg(wnet_vif, 0xf02040);
    reg_val2 = get_reg(wnet_vif, 0xf03040);
    len += scnprintf(extra + len, IW_PRIV_SIZE_MASK, " bs0x%x,0x%x\n",reg_val1,reg_val2);

    len += scnprintf(extra + len, IW_PRIV_SIZE_MASK, " tf%d,to%d\n",hif->HiStatus.tx_fail_num,  hif->HiStatus.tx_ok_num);

    if (wnet_vif && wnet_vif->vm_mainsta && wnet_vif->vm_wmac->wm_curchan) {
        len += scnprintf(extra + len, IW_PRIV_SIZE_MASK, " p_ch %d, bw%d\n",wnet_vif->vm_wmac->wm_curchan->chan_pri_num, wnet_vif->vm_wmac->wm_curchan->chan_bw);
        len += scnprintf(extra + len, IW_PRIV_SIZE_MASK, " br %d,dr %d,co%d\n",wnet_vif->vm_mainsta->sta_avg_bcn_rssi,wnet_vif->vm_mainsta->sta_avg_data_rssi, wnet_vif->vm_wmac->drv_priv->drv_config.cfg_txpoweplan);
    }

    wrqu->data.length += len;
    wrqu->data.length++;
    wifi_mac_show_per_info();
    return 0;
}

static int aml_iwpriv_get_efuse_times(struct net_device *dev, union iwreq_data *wrqu, char *extra)
{
    int *param = (int *)extra;
    unsigned char rf_vld_val = 0;
    int times = 0;

    rf_vld_val = (efuse_manual_read(0x09) & 0x00ff0000) >> 16;

    switch (rf_vld_val & 0xBF)
    {
        case 0xBF:
            times = 0;
            break;
        case 0x0F:
            times = 1;
            break;
        case 0xB0:
            times = 1;
            break;
        case 0x00:
            times = 2;
            break;
        default:
            AML_PRINT_LOG_INFO("WARNING RF_VLD[0x%02x]!\n",rf_vld_val);
            if ((rf_vld_val & 0x0F) == 0x0F) {
                times = 1;
            } else {
                times = 2;
            }
            break;
    }
    param[0] = times;
    wrqu->data.length = 1;

    return 0;
}

int aml_iwpriv_get_efuse_wifi_mac(struct net_device *dev, union iwreq_data *wrqu, char *extra)
{
    unsigned int efuse_data_l = 0;
    unsigned int efuse_data_h = 0;

    efuse_data_l = efuse_manual_read(0x1);
    efuse_data_h = efuse_manual_read(0x2);
    AML_PRINT_LOG_INFO("AML EFUSE WIFI MAC ADDR:  %02x:%02x:%02x:%02x:%02x:%02x\n",
            (efuse_data_h & 0xff00) >> 8,efuse_data_h & 0x00ff, (efuse_data_l & 0xff000000) >> 24,
            (efuse_data_l & 0x00ff0000) >> 16,(efuse_data_l & 0xff00) >> 8,efuse_data_l & 0xff);

    if (efuse_data_l != 0 || efuse_data_h != 0) {
        wrqu->data.length = scnprintf(extra, IW_PRIV_SIZE_MASK, " %02x:%02x:%02x:%02x:%02x:%02x\n",
                                        (efuse_data_h & 0xff00) >> 8,efuse_data_h & 0x00ff, (efuse_data_l & 0xff000000) >> 24,
                                        (efuse_data_l & 0x00ff0000) >> 16,(efuse_data_l & 0xff00) >> 8,efuse_data_l & 0xff);
        wrqu->data.length++;
    }

    return 0;
}

int aml_iwpriv_get_bt_efuse_mac(struct net_device *dev, union iwreq_data *wrqu, char *extra)
{
    unsigned int efuse_data_l = 0;
    unsigned int efuse_data_h = 0;

    efuse_data_l = efuse_manual_read(0x2);
    efuse_data_h = efuse_manual_read(0x3);
    AML_PRINT_LOG_INFO("AML EFUSE BT MAC ADDR:  %02x:%02x:%02x:%02x:%02x:%02x\n",
            (efuse_data_h & 0xff000000) >> 24, (efuse_data_h & 0x00ff0000) >> 16,
            (efuse_data_h & 0xff00) >> 8,efuse_data_h & 0xff,
            (efuse_data_l & 0xff000000) >> 24,(efuse_data_l & 0x00ff0000) >> 16);

    if (efuse_data_l != 0 || efuse_data_h != 0) {
        wrqu->data.length = scnprintf(extra, IW_PRIV_SIZE_MASK, " %02x:%02x:%02x:%02x:%02x:%02x\n",
                                        (efuse_data_h & 0xff000000) >> 24, (efuse_data_h & 0x00ff0000) >> 16,
                                        (efuse_data_h & 0xff00) >> 8,efuse_data_h & 0xff,
                                        (efuse_data_l & 0xff000000) >> 24,(efuse_data_l & 0x00ff0000) >> 16);
        wrqu->data.length++;
    }

    return 0;
}

static int aml_iwpriv_get(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{

    struct wlan_net_vif *wnet_vif = NULL;
    struct wifi_mac *wifimac = NULL;
    struct wifi_channel *c = NULL;
    int *param = (int *)extra;
    int sub_cmd = param[0];
    int i = 0;
    char buf[30] = {0};

    AML_PRINT_LOG_INFO("sub cmd %d\n", param[0]);

    wifimac = wifi_mac_get_mac_handle();
    wnet_vif = aml_iwpriv_get_vif(dev->name);

    /*if we need feed back the value to user space, we need these 2 lines code, this is a sample*/
    //wrqu->data.length = sizeof(int);
    //*param = 110;
    /*if we need feed back the value to user space, we need these 2 lines code, this is a sample*/

    switch (sub_cmd) {
        case AML_IWP_AMSDU_STATE:
            aml_ap_get_amsdu_state();
            break;

        case AML_IWP_AMPDU_STATE:
            aml_ap_get_ampdu_state();
            break;

        case AML_IWP_UDP_INFO:
            for (i = 0; i < udp_cnt; i++) {
                AML_PRINT_LOG_INFO("streamid=%d tx is %d, rx is %d, rx tot_len is %d\n", aml_udp_info[i].streamid, aml_udp_info[i].tx, aml_udp_info[i].rx, aml_udp_info[i].rx_totlen);
            }
            aml_udp_timer.udp_timer_stop = 1;
            aml_udp_timer.run_flag = 0;
            udp_cnt = 0;
            os_timer_ex_del(&aml_udp_timer.udp_send_timeout, CANCEL_SLEEP);
            break;

        case AML_IWP_COUNTRY:
            AML_PRINT_LOG_INFO("country code: %s\n", wifimac->wm_country.iso);
            break;

        case AML_IWP_CHAN_LIST:
            WIFI_CHANNEL_LOCK(wifimac);
            for (i = 0; i < wifimac->wm_nchans; i++) {
                c = &wifimac->wm_channels[i];
                if (!(hal_get_channel_2g_20Mhz_only() && c->chan_bw == CHAN_BW_40M && (c->chan_pri_num >= 1) && (c->chan_pri_num <= 14))) {
                    AML_PRINT_LOG_INFO("channel:%d\t frequency:%d \t bandwidth:%dMHz \n", c->chan_pri_num, c->chan_cfreq1, ((1 << c->chan_bw) * 20));
                }
            }
            WIFI_CHANNEL_UNLOCK(wifimac);
            break;

        case AML_IWP_CHIP_ID:
            snprintf(buf, 21, "get_chip_id 0x%x 0x%x",0x8, 0x9);
            AML_PRINT_LOG_INFO("buf %s\n", buf);
            aml_get_chip_id(NULL, buf, 0);
            break;

        case AML_IWP_AP_IP:
            aml_get_ap_ip(wnet_vif, NULL, 0);
            break;

        case AML_IWP_ROAM_CHAN:
            aml_get_roaming_candidate_chans(wnet_vif, NULL, 0);
            break;

        case AML_IWP_SESSION:
            aml_sta_get_wfd_session(wnet_vif, NULL, 0);
            break;
#ifdef CONFIG_P2P
        case AML_IWP_P2P_DEV_ID:
            aml_get_p2p_device_addr(wnet_vif, buf, 0);
            break;
#endif
        case AML_IWP_WIFI_MAC:
            aml_get_wifi_mac_addr(wnet_vif, buf, 30);
            break;

        case AML_IWP_CCA_BUSY_CHECK:
            wifimac->drv_priv->drv_ops.cca_busy_check();
            break;

        case AML_IWP_PHY_STATISTIC:
            wifimac->drv_priv->drv_ops.phy_stc();
            break;

        case AML_IWP_PRINT_VERSION:
            print_driver_version();
            AML_PRINT_LOG_INFO("driver version: %s\n", DRIVERVERSION);
            AML_PRINT_LOG_INFO("=============bus type:%s fw_chip:%s=============\n",aml_wifi_get_bus_type(),aml_wifi_get_fw_type());
            break;

        case AML_IWP_GET_TX_STATUS:
            get_latest_tx_status(wifimac);
            break;

        case AML_IWP_SET_RATE_AUTO:
            aml_iwpriv_set_rate_auto(wnet_vif);
            break;

        case AML_IWP_SET_PT_RX_STOP:
            wnet_vif->vif_ops.pt_rx_stop();
            break;

        case AML_IWP_GET_EN_RF_TEST:
            snprintf(buf, 13, "en_rf_test=%d", aml_wifi_is_enable_rf_test());
            AML_PRINT_LOG_INFO("%s\n",buf);
            break;

        case AML_IWP_PT_TX_START:
            aml_iwpriv_pt_tx_start();
            break;

        case AML_IWP_PT_TX_END:
            aml_iwpriv_pt_tx_end();
            break;

        case AML_IWP_GET_CAPT_STATICS:
            aml_save_sta_capture_statics(wnet_vif);
            break;
    }

    return 0;
}

static int aml_iwpriv_start_capture(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
    dut_start_capture(0x00005e00);//start capture on 0x5e
    return 0;
}


static int aml_iwpriv_get_csi_info(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
    struct wlan_net_vif *wnet_vif = NULL;
    struct wifi_mac *wifimac = NULL;
    csi_stream_t *csi_info = NULL;
    unsigned int band = 0;
    unsigned int csi_len = 0;
    unsigned int mac_mode = 0;
    short phase_incr = 0;
    unsigned int arr[8] = {0};
    static unsigned int pkg_idx = 0;

    AML_PRINT_LOG_INFO("start++\n");

    wifimac = wifi_mac_get_mac_handle();
    wnet_vif = aml_iwpriv_get_vif(dev->name);

    if ((wnet_vif->vm_state != WIFINET_S_CONNECTED)
        || (wnet_vif->vm_curchan == WIFINET_CHAN_ERR)
        || (wnet_vif->vm_mainsta == NULL)) {

        AML_PRINT_LOG_ERR("curchan or mainsta not available\n");
        return 0;
    }

    csi_info = ZMALLOC(sizeof(csi_stream_t), "csi_info", GFP_KERNEL);
    if (csi_info == NULL) {
        AML_PRINT_LOG_ERR("no memory!\n");
        return 0;
    }

    /*get band*/
    if (WIFINET_IS_CHAN_5GHZ(wnet_vif->vm_curchan)) {
        band = 2;
    } else if (WIFINET_IS_CHAN_2GHZ(wnet_vif->vm_curchan)) {
        band = 1;
    } else {
        band = 0;
    }

    /*get snr and noise*/
    get_phy_stc_info(arr);

    /*get protocol mode*/
    if (wnet_vif->vm_mac_mode >= WIFINET_MODE_11AC) {
        mac_mode = CSI_FRAME_TYPE_11AC;
    } else if (wnet_vif->vm_mac_mode >= WIFINET_MODE_11N) {
        mac_mode = CSI_FRAME_TYPE_11n;
    } else if (wnet_vif->vm_mac_mode >= WIFINET_MODE_11G) {
        mac_mode = CSI_FRAME_TYPE_11BA;
    }

    /*get csi len*/
    if (wnet_vif->vm_bandwidth == WIFINET_BWC_WIDTH20) {
        csi_len = 56;
    } else if (wnet_vif->vm_bandwidth == WIFINET_BWC_WIDTH40) {
        csi_len = 114;
    } else if (wnet_vif->vm_bandwidth == WIFINET_BWC_WIDTH80) {
        csi_len = 242;
    }

    /*get phase_incr from bit 23:12 of reg 0x00a092a0 */
    phase_incr = wnet_vif->vif_ops.read_word(0x00a092a0);
    phase_incr = (phase_incr >> 12) & 0x0fff;

    pkg_idx++;
    memset(csi_info, 0, sizeof(csi_stream_t));
    csi_info->time_stamp = wnet_vif->vm_mainsta->bcn_stamp;
    WIFINET_ADDR_COPY(csi_info->mac_ra, wnet_vif->vm_myaddr);
    WIFINET_ADDR_COPY(csi_info->mac_ta, wnet_vif->vm_des_bssid);
    csi_info->frequency_band = band;
    csi_info->bw = wnet_vif->vm_bandwidth;
    csi_info->rssi = wnet_vif->vm_mainsta->sta_avg_bcn_rssi;
    csi_info->protocol_mode = mac_mode;
    csi_info->frame_type = wnet_vif->vm_mainsta->cur_fratype;
    csi_info->chain_num = 1;
    csi_info->csi_len = csi_len;
    csi_info->snr = arr[1];
    csi_info->primary_channel_index = wifi_mac_Mhz2ieee(wnet_vif->vm_curchan->chan_cfreq1, 0);
    csi_info->noise = arr[4];
    csi_info->phyerr = 0;
    csi_info->rate = wnet_vif->vm_mainsta->sta_vendor_rate_code & 0xf;
    csi_info->extra_information = 0;
    csi_info->agc_code = 0;
    csi_info->phase_incr = phase_incr;
    csi_info->channel = wnet_vif->vm_curchan->chan_pri_num;
    //csi_info->reserved = 0;
    csi_info->packet_idx = pkg_idx;
    iwp_stop_tbus_to_get_sram(csi_info->csi);

    wrqu->data.length = sizeof(csi_stream_t);
    if (copy_to_user(wrqu->data.pointer,  (void*)csi_info, wrqu->data.length)) {
        AML_PRINT_LOG_ERR("copy to user failed \n");
        FREE(csi_info,"csi_info");
        return -EFAULT;
    }
    FREE(csi_info,"csi_info");
    AML_PRINT_LOG_INFO("end++\n");
    return 0;
}


static int aml_ap_set_udp_info(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
    char **arg;
    int cmd_arg;
    char sep = ',';

    char buf[41] = {0};
    if (copy_from_user(buf, wrqu->data.pointer, 40)) {
        return -EFAULT;
    }
    buf[40] = '\0';
    AML_PRINT_LOG_INFO("%s\n", buf);

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    wifi_mac_set_udp_info(arg);
    kfree(arg);

    return 0;

}

static int aml_ap_get_udp_info(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{

    int i = 0;
    for (i = 0; i < udp_cnt; i++) {
        AML_PRINT_LOG_INFO("streamid=%d tx is %d, rx is %d\n", aml_udp_info[i].streamid, aml_udp_info[i].tx, aml_udp_info[i].rx);
    }
    aml_udp_timer.udp_timer_stop = 1;
    aml_udp_timer.run_flag = 0;
    udp_cnt = 0;
    os_timer_ex_del(&aml_udp_timer.udp_send_timeout, CANCEL_SLEEP);

    return 0;
}

static int aml_iwpriv_get_int(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
    int parameter[16], i;
    int sub_cmd = wrqu->data.flags;

    AML_PRINT_LOG_INFO("sub_cmd:%d param_count:%d \n",sub_cmd,wrqu->data.length);

    if (wrqu->data.length) {
        if (copy_from_user(parameter, wrqu->data.pointer, sizeof(int) * wrqu->data.length)) {
            AML_PRINT_LOG_ERR("Copy from user failed!\n");
            return -EFAULT;
        }

    }

   for (i = 0; i < wrqu->data.length; i++) {
        AML_PRINT_LOG_INFO("parameter[%d]:%d \n", i, parameter[i]);
   }

    switch (sub_cmd) {
        case AML_IWP_GET_EFUSE_TIMES:
            aml_iwpriv_get_efuse_times(dev, wrqu, extra);
            break;

        default:
            break;
    }

    return 0;
}

static int aml_iwpriv_get_char(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
    int sub_cmd = wrqu->data.flags;
    char parameter[32];

    if (wrqu->data.length) {
        if (copy_from_user(parameter, wrqu->data.pointer, wrqu->data.length)) {
            AML_PRINT_LOG_ERR("Copy from user failed!\n");
            return -EFAULT;
        }
        parameter[wrqu->data.length] = '\0';
    }

    AML_PRINT_LOG_INFO("sub_cmd:%d param_len:%d param_data:'%s' \n",sub_cmd,wrqu->data.length,parameter);

    switch (sub_cmd) {
        case AML_IWP_GET_REG:
            aml_iwpriv_get_reg(dev, parameter, wrqu, extra);
            break;
        case AML_IWP_LEGACY_GET_REG:
            aml_iwpriv_legacy_get_reg(dev, parameter, wrqu, extra);
            break;
        case AML_IWP_GET_EFUSE:
            aml_iwpriv_get_efuse(dev, parameter, wrqu, extra);
            break;
        case AML_IWP_GET_EFUSE_DOMAIN:
            aml_iwpriv_get_efuse_domain(dev, parameter, wrqu, extra);
            break;
        case AML_IWP_GET_EFUSE_TMP:
            aml_iwpriv_get_efuse_tmp(dev, parameter, wrqu, extra);
            break;
        case AML_IWP_GET_WIFI_MAC:
            aml_iwpriv_get_efuse_wifi_mac(dev, wrqu, extra);
            break;
        case AML_IWP_GET_BT_MAC:
            aml_iwpriv_get_bt_efuse_mac(dev, wrqu, extra);
            break;
        case AML_IWP_PT_RX_END:
            aml_iwpriv_pt_rx_end(dev, wrqu, extra);
            break;
        case AML_IWP_GET_DEV_SN:
            aml_iwpriv_get_sn(dev, wrqu, extra);
            break;
        case AML_IWP_SET_ANT_SWITCH:
            aml_iwpriv_set_ant_switch(dev, parameter, wrqu, extra);
            break;
        case AML_IWP_GET_BT_DIGITAL_GAIN_EFUSE_TIMES:
            aml_iwpriv_get_bt_digital_gain_efuse_times(dev, wrqu, extra);
            break;
        case AML_IWP_GET_BT_DIGITAL_GAIN_EFUSE:
            aml_iwpriv_get_bt_digital_gain_efuse(dev, wrqu, extra);
            break;
        case AML_IWP_GET_WIFI_PERFORMANCE_INFO:
            aml_iwpriv_get_performance_info(dev, wrqu, extra);
            break;
        default:
            break;
    }

    return 0;
}

static int aml_set_country_code(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{

    AML_PRINT_LOG_INFO("set country <%s> by iwpriv\n",extra);

    // kernel has bug this operatin will let "iw reg set xx" not work
    // regulatory_hint(wnet_vif->vm_wdev->wiphy,extra);

    wifi_mac_set_country_code(extra);

    return 0;
}

void aml_iwpriv_set_dev_sn(char* arg_iw)
{
    char **mac_cmd;
    int i,cmd_arg;
    char sep = ':';
    unsigned int efuse_data = 0;

    mac_cmd = aml_cmd_char_parse(sep, arg_iw, &cmd_arg);
    if (mac_cmd) {
        efuse_data = (simple_strtoul(mac_cmd[0],NULL,16) << 8) | (simple_strtoul(mac_cmd[1],NULL,16));
        for (i = 0; i < 16; i++) {
            if (efuse_data & (1 << i)) {
                efuse_manual_write(i, 0xf);
            }
        }
        AML_PRINT_LOG_INFO("iwpriv write module SN is: %02x %02x\n", ((efuse_data & 0xff00) >> 8), (efuse_data & 0x00ff));
    }
    kfree(mac_cmd);

}

void aml_iwpriv_set_mac_addr(char* arg_iw)
{
    char **mac_cmd;
    int i,cmd_arg;
    char sep = ':';
    unsigned int efuse_data_l = 0;
    unsigned int efuse_data_h = 0;

    mac_cmd = aml_cmd_char_parse(sep, arg_iw, &cmd_arg);
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
        AML_PRINT_LOG_INFO("iwpriv write WIFI MAC addr is:  %02x:%02x:%02x:%02x:%02x:%02x\n",
                (efuse_data_h & 0xff00) >> 8,efuse_data_h & 0x00ff, (efuse_data_l & 0xff000000) >> 24,
                (efuse_data_l & 0x00ff0000) >> 16,(efuse_data_l & 0xff00) >> 8,efuse_data_l & 0xff);
    }
    kfree(mac_cmd);
}

void aml_iwpriv_set_bt_dev_id(char* arg_iw)
{
    char **mac_cmd;
    int i,cmd_arg;
    char sep = ':';
    unsigned int efuse_data_l = 0;
    unsigned int efuse_data_h = 0;

    mac_cmd = aml_cmd_char_parse(sep, arg_iw, &cmd_arg);
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
        AML_PRINT_LOG_INFO("iwpriv write BT MAC addr is:  %02x:%02x:%02x:%02x:%02x:%02x\n",
                (efuse_data_h & 0xff000000) >> 24, (efuse_data_h & 0x00ff0000) >> 16,
                (efuse_data_h & 0xff00) >> 8,efuse_data_h & 0xff,
                (efuse_data_l & 0xff000000) >> 24,(efuse_data_l & 0x00ff0000) >> 16);
    }
    kfree(mac_cmd);
}

static int aml_set_dev_sn(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
    AML_PRINT_LOG_INFO("%s\n", extra);
    aml_iwpriv_set_dev_sn(extra);

    return 0;
}

static int aml_set_efuse_wifi_mac(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
    char buf[19] = {0};
    if (copy_from_user(buf, wrqu->data.pointer, 18)) {
        return -EFAULT;
    }
    buf[18] = '\0';
    AML_PRINT_LOG_INFO("%s\n", buf);
    aml_iwpriv_set_mac_addr(buf);

    return 0;
}

static int aml_set_efuse_btmac(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
    char buf[19] = {0};

    if (copy_from_user(buf, wrqu->data.pointer, 18)) {
        return -EFAULT;
    }
    buf[18] = '\0';
    AML_PRINT_LOG_INFO("%s\n", buf);
    aml_iwpriv_set_bt_dev_id(buf);

    return 0;
}

int aml_iwpriv_set_debug_switch(char *switch_str)
{
    int debug_switch = 0;

    if (strstr(switch_str,"_off") != NULL) {
        debug_switch = 0;
    } else if (strstr(switch_str,"_on") != NULL) {
        debug_switch = 1;
    } else {
        AML_PRINT_LOG_ERR("input error\n");
    }

    return debug_switch;
}

int aml_set_debug_modules(char *debug_str)
{
    MODULE_ID module_id;
    DEBUG_LEVEL level;

    if (debug_str == NULL || strlen(debug_str) <= 0) {
        AML_PRINT(AML_LOG_ID_LOG,AML_LOG_LEVEL_ERROR,"debug modules is NULL\n");
        return -1;
    }

    if (strstr(debug_str, "ERR") != NULL) {
        level = AML_LOG_LEVEL_ERROR;
    } else if (strstr(debug_str, "WARN") != NULL) {
        level = AML_LOG_LEVEL_WARN;
    } else if (strstr(debug_str, "INFO") != NULL) {
        level = AML_LOG_LEVEL_INFO;
    } else if (strstr(debug_str, "DEBUG") != NULL) {
        level = AML_LOG_LEVEL_DEBUG;
    } else if (strstr(debug_str,"_on") != NULL) {
        level = AML_LOG_LEVEL_DEBUG;
    } else if (strstr(debug_str,"_off") != NULL) {
        level = AML_LOG_LEVEL_DEFAULT;
    } else {
        AML_PRINT(AML_LOG_ID_LOG,AML_LOG_LEVEL_ERROR,"debug level not found!\n");
        return -1;
    }

    if (strstr(debug_str, "ALL") == NULL ) {
        for (module_id = 0; module_id < AML_LOG_ID_MAX; module_id++) {
            if (strstr(debug_str, gAmlTraceInfo[module_id].moduleNameStr) != NULL) {
                break;
            }
        }
        if (module_id == AML_LOG_ID_MAX) {
            AML_PRINT(AML_LOG_ID_LOG,AML_LOG_LEVEL_ERROR,"module_id not found!\n");
            return -2;
        }
        aml_set_debug_level( module_id, level);
    } else {
        aml_set_all_debug_level(level);
    }

    return 0;
}

static int aml_iwpriv_set_debug(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
    AML_PRINT_LOG_INFO("%s\n", extra);
    aml_set_debug_modules(extra);
    return 0;

}

static int aml_get_country_code(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();

    AML_PRINT_LOG_INFO("country code: %s\n", wifimac->wm_country.iso);

    return 0;
}

static int aml_get_channel_list(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra) {
    struct wifi_mac *wifimac = wifi_mac_get_mac_handle();
    struct wifi_channel *c;
    int i = 0 ;

    WIFI_CHANNEL_LOCK(wifimac);
    for (i = 0; i < wifimac->wm_nchans; i++) {
        c = &wifimac->wm_channels[i];
        AML_PRINT_LOG_INFO("channel:%d\t frequency:%d \t bandwidth:%dMHz \n", c->chan_pri_num, c->chan_cfreq1, ((1 << c->chan_bw) * 20));
    }
    WIFI_CHANNEL_UNLOCK(wifimac);

     return 0;
}

static int aml_ap_set_arp_rx(struct net_device *dev,
    struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
    char **arg;
    int cmd_arg;
    char sep = ',';

    char buf[41] = {0};
    if (copy_from_user(buf, wrqu->data.pointer, 40)) {
        return -EFAULT;
    }
    buf[40] = '\0';
    AML_PRINT_LOG_INFO("%s\n", buf);

    arg = aml_cmd_char_parse(sep, buf, &cmd_arg);
    wifi_mac_ap_set_arp_rx(arg);
    kfree(arg);

    return 0;
}

int iw_standard_get_stats(struct net_device *dev, struct iw_request_info *info,
    union iwreq_data *wrqu, char *extra)
{
    struct wlan_net_vif *wnet_vif = NULL;
    struct iw_statistics stats = {0};
    unsigned int arr[8] = {0};

    AML_PRINT_LOG_INFO("\n");

    wnet_vif = aml_iwpriv_get_vif(dev->name);
    if ((wnet_vif->vm_opmode != WIFINET_M_STA) && (wnet_vif->vm_state != WIFINET_S_CONNECTED)) {
        return 0;
    }

    get_phy_stc_info(arr);
    stats.qual.level = wnet_vif->vm_mainsta->sta_avg_bcn_rssi;
    stats.qual.noise = arr[4];
    stats.qual.qual = arr[1];
    stats.qual.updated = IW_QUAL_ALL_UPDATED;

    /* Copy statistics to extra */
    memcpy(extra, &stats, sizeof(struct iw_statistics));
    wrqu->data.length = sizeof(struct iw_statistics);

    return 0;
}

void sap_change_channel(struct wlan_net_vif *wnet_vif, unsigned int channel)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct wifi_channel *c = NULL;
    int center_chan = 0;
    int i = 0;

    if (wnet_vif->vm_bandwidth == WIFINET_BWC_WIDTH20) {
        center_chan = channel;
    } else if (wnet_vif->vm_bandwidth == WIFINET_BWC_WIDTH40) {
        if (wnet_vif->scnd_chn_offset == WIFINET_HTINFO_EXTOFFSET_ABOVE)
            center_chan = channel + 2;
        else if (wnet_vif->scnd_chn_offset == WIFINET_HTINFO_EXTOFFSET_ABOVE)
            center_chan = channel - 2;
        else
            center_chan = channel;
    } else if (wnet_vif->vm_bandwidth == WIFINET_BWC_WIDTH80) {
        WIFI_CHANNEL_LOCK(wifimac);
        for (i = 0; i < wifimac->wm_nchans; i++) {
            c = &wifimac->wm_channels[i];
            if ((c->chan_pri_num== channel) && (c->chan_bw == WIFINET_BWC_WIDTH80)) {
                center_chan = wifi_mac_Ieee2mhz(c->chan_cfreq1, 0);
                break;
            }
        }
        WIFI_CHANNEL_UNLOCK(wifimac);
    }

    wifi_mac_set_wnet_vif_channel(wnet_vif, channel, wnet_vif->vm_bandwidth, center_chan);
}

int iw_standard_sap_set_freq(struct net_device *dev, struct iw_request_info *info,
    union iwreq_data *wrqu, char *extra)
{
    struct wlan_net_vif *wnet_vif = NULL;
    struct wifi_mac *wifimac = NULL;
    struct wifi_channel *c = NULL;
    unsigned int set_chl = 0;

    AML_PRINT_LOG_INFO("freq.m:%d, freq.e:%d\n", wrqu->freq.m, wrqu->freq.e);

    wifimac = wifi_mac_get_mac_handle();

    /* Settings by Frequency as input */
    if(wrqu->freq.e == 1) {
        unsigned int freq = wrqu->freq.m / 100000;

        if ((wrqu->freq.m < (unsigned int)2.412e8) || (wrqu->freq.m > (unsigned int)5.825e8))
            return -EINVAL;

        c = wifi_mac_find_chan(wifimac, wifi_mac_Mhz2ieee(freq, 0), WIFINET_BWC_WIDTH20, wifi_mac_Mhz2ieee(freq, 0));
    }

    /* Settings by Channel as input */
    if (wrqu->freq.e == 0) {
        unsigned int chan = wrqu->freq.m;

        if((wrqu->freq.m < 1) || (wrqu->freq.m > 165))
            return -EINVAL;

        c = wifi_mac_find_chan(wifimac, chan, WIFINET_BWC_WIDTH20, chan);
    }

    if (c == NULL) {
        return -EINVAL;
    }

    set_chl = c->chan_pri_num;
    wnet_vif = aml_iwpriv_get_vif(dev->name);

    AML_PRINT_LOG_INFO("opmode:%d, nrunning:%d\n", wnet_vif->vm_opmode, wifimac->wm_nrunning);
    if ((wnet_vif->vm_opmode == WIFINET_M_HOSTAP) && (wifimac->wm_nrunning == 1)) {
        sap_change_channel(wnet_vif, set_chl);
    }

    return 0;
}

void wifi_mac_pwrsave_set_inactime(struct wlan_net_vif *wnet_vif, unsigned int time)
{
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;

    if (wnet_vif->vm_opmode != WIFINET_M_STA) {
        return;
    }

    AML_PRINT_LOG_INFO("<running> time %d\n", time);

    if(time == 0) {
        wifi_mac_pwrsave_set_mode(wnet_vif, WIFINET_PWRSAVE_NONE);
        return;
    }

    wnet_vif->vm_pwrsave.ips_inactivitytime = time;

    if (wnet_vif->vm_state == WIFINET_S_CONNECTED && wifimac->wm_syncbeacon == 0) {
        AML_PRINT_LOG_INFO("<running>\n");
        wifi_mac_beacon_sync(wifimac->drv_priv->wmac, wnet_vif->wnet_vif_id);
    }

    if (time >= WIFINET_PS_LOW_INACTIVITYTIME) {
        wnet_vif->vm_pwrsave.ips_sta_psmode = WIFINET_PWRSAVE_LOW;
    } else {
        wnet_vif->vm_pwrsave.ips_sta_psmode = WIFINET_PWRSAVE_NORMAL;
    }
    os_timer_ex_start_period(&wnet_vif->vm_pwrsave.ips_timer_presleep, wnet_vif->vm_pwrsave.ips_inactivitytime);
}


int iw_standard_set_pwr(struct net_device *dev, struct iw_request_info *info,
    union iwreq_data *wrqu, char *extra)
{
    struct wlan_net_vif *wnet_vif = NULL;
    unsigned int time = 0;
    wnet_vif = aml_iwpriv_get_vif(dev->name);

    AML_PRINT_LOG_INFO("disable:%d\n", wrqu->power.disabled);
    if (wrqu->power.disabled) {
        wifi_mac_pwrsave_set_mode(wnet_vif, WIFINET_PWRSAVE_NONE);
        if (wnet_vif->vm_wdev) {
            wnet_vif->vm_wdev->ps = 0;
        }
        return 0;
    } else {
        if (wrqu->power.flags & IW_POWER_PERIOD) {
            time = wrqu->power.value / 1000;
            AML_PRINT_LOG_INFO("pwr perio value: %d\n", wrqu->power.value / 1000);
        }

        switch (wrqu->power.flags & IW_POWER_MODE) {
            case IW_POWER_UNICAST_R:
            case IW_POWER_MULTICAST_R:
            case IW_POWER_ALL_R:
                AML_PRINT_LOG_INFO("pwr mode value: %d\n", wrqu->power.value / 1000);
                break;

            case IW_POWER_ON:
                break;

            default:
                return 0;
        }

        wifi_mac_pwrsave_set_inactime(wnet_vif, time);
        if (wnet_vif->vm_wdev) {
            wnet_vif->vm_wdev->ps = 1;
        }
    }

    return 0;
}
#define IFNAMSIZ 16

int iw_standard_get_name(struct net_device *dev, struct iw_request_info *info,
     union iwreq_data *wrqu, char *extra)
{
    struct wlan_net_vif *wnet_vif = netdev_priv(dev);
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    enum nl80211_iftype iftype;

    list_for_each_entry(wnet_vif, &wifimac->wm_wnet_vifs, vm_next) {
        if (wnet_vif->vm_ndev == NULL || !(wnet_vif->vm_ndev->flags & IFF_RUNNING)) {
            continue;
        }
        iftype = wnet_vif->vm_wdev->iftype;

        if (iftype == NL80211_IFTYPE_STATION) {

            if (wnet_vif->vm_mainsta && wnet_vif->vm_state == WIFINET_S_CONNECTED) {
                switch (wnet_vif->vm_mainsta->sta_bssmode) {
                    case WIFINET_MODE_11B:
                        snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11b");
                        break;
                    case WIFINET_MODE_11G:
                        if (wnet_vif->vm_curchan && wnet_vif->vm_curchan->chan_pri_num >= 36) {
                            snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11a");
                        }
                        else {
                            snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11g");
                        }
                        break;
                    case WIFINET_MODE_11BG:
                        snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11bg");
                        break;
                    case WIFINET_MODE_11GN:
                        snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11gn");
                        break;
                    case WIFINET_MODE_11BGN:
                        snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11bgn");
                        break;
                    case WIFINET_MODE_11N:
                        snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11n");
                        break;
                    case WIFINET_MODE_11AC:
                    case WIFINET_MODE_11NAC:
                    case WIFINET_MODE_11GNAC:
                        snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11ac");
                        break;
                    default:
                        //snprintf(wrqu->name, IFNAMSIZ, "IEEE 802.11ax");
                        AML_PRINT_LOG_WRAN("no such mode %d\n", wnet_vif->vm_mainsta->sta_bssmode);
                        break;
                }
            } else {
                snprintf(wrqu->name, IFNAMSIZ, "unassociated");
            }
        }
    }
    return 0;
}

int iw_standard_get_range(struct net_device *dev, struct iw_request_info *info,
    union iwreq_data *wrqu, char *extra)
{
    struct iw_range * range = (struct iw_range *)extra;
    struct wlan_net_vif *wnet_vif = netdev_priv(dev);
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    enum nl80211_iftype iftype;
    unsigned int throughput = 20000000;

    list_for_each_entry(wnet_vif, &wifimac->wm_wnet_vifs, vm_next) {
        if (wnet_vif->vm_ndev == NULL || !(wnet_vif->vm_ndev->flags & IFF_RUNNING)) {
            continue;
        }
        iftype = wnet_vif->vm_wdev->iftype;

        if (iftype == NL80211_IFTYPE_STATION) {

            if (wnet_vif->vm_mainsta && wnet_vif->vm_state == WIFINET_S_CONNECTED && wnet_vif->vm_curchan) {
                switch (wnet_vif->vm_curchan->chan_bw) {
                    case CHAN_BW_20M:
                        throughput = 40000000;
                        break;
                    case CHAN_BW_40M:
                        throughput = 80000000;
                        break;
                    case CHAN_BW_80M:
                        throughput = 160000000;
                        break;
                    default:
                        AML_PRINT_LOG_ERR("no such bw:%d\n", wnet_vif->vm_curchan->chan_bw);
                        break;
                }
            } else {
                AML_PRINT_LOG_ERR("sta not connected\n");
            }
        }
    }

    memset(range, 0 , sizeof(*range));
    range->throughput = throughput;
    range->min_nwid = 0;
    range->max_nwid = 0;
    range->we_version_compiled  = WIRELESS_EXT;
    range->we_version_source = WIRELESS_EXT;

    return 0;
}


int iw_standard_get_ap(struct net_device *dev, struct iw_request_info *info,
    union iwreq_data *wrqu, char *extra)
{
    struct wlan_net_vif *wnet_vif = netdev_priv(dev);
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    enum nl80211_iftype iftype;
    struct sockaddr * addr = &(wrqu->ap_addr);

    list_for_each_entry(wnet_vif, &wifimac->wm_wnet_vifs, vm_next) {
         if (wnet_vif->vm_ndev == NULL || !(wnet_vif->vm_ndev->flags & IFF_RUNNING)) {
             continue;
         }
         iftype = wnet_vif->vm_wdev->iftype;

         if (iftype == NL80211_IFTYPE_STATION) {
            if (wnet_vif->vm_mainsta && wnet_vif->vm_state == WIFINET_S_CONNECTED) {
                memcpy(addr->sa_data, wnet_vif->vm_mainsta->sta_bssid, ETH_ALEN);
            }
         }
    }

    return 0;
}

int iw_standard_get_essid(struct net_device *dev, struct iw_request_info *info,
    union iwreq_data *wrqu, char *extra)
{
    struct iw_point * essid = &(wrqu->essid);
    struct wlan_net_vif *wnet_vif = netdev_priv(dev);
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    enum nl80211_iftype iftype;

    list_for_each_entry(wnet_vif, &wifimac->wm_wnet_vifs, vm_next) {
         if (wnet_vif->vm_ndev == NULL || !(wnet_vif->vm_ndev->flags & IFF_RUNNING)) {
             continue;
         }
         iftype = wnet_vif->vm_wdev->iftype;

         if (iftype == NL80211_IFTYPE_STATION) {
            if (wnet_vif->vm_mainsta && wnet_vif->vm_state == WIFINET_S_CONNECTED) {
                essid->length = wnet_vif->vm_mainsta->sta_esslen;
                memcpy(extra, wnet_vif->vm_mainsta->sta_essid, wnet_vif->vm_mainsta->sta_esslen);
                essid->flags = 1;
            }
         }
    }
    return 0;
}

static struct iw_statistics *aml_get_wireless_stats(struct net_device *dev)
{
    struct wlan_net_vif *wnet_vif = netdev_priv(dev);
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    struct iw_statistics *wstats = &wnet_vif->vm_iwstats;
    enum nl80211_iftype iftype;

    list_for_each_entry(wnet_vif, &wifimac->wm_wnet_vifs, vm_next) {
        if (wnet_vif->vm_ndev == NULL || !(wnet_vif->vm_ndev->flags & IFF_RUNNING)) {
             continue;
        }
        iftype = wnet_vif->vm_wdev->iftype;

        if (iftype == NL80211_IFTYPE_STATION) {
            if (wnet_vif->vm_mainsta && wnet_vif->vm_state == WIFINET_S_CONNECTED) {
                wstats->qual.qual = wnet_vif->vm_mainsta->sta_avg_snr;
                wstats->qual.level = wnet_vif->vm_mainsta->sta_avg_bcn_rssi;
                wstats->qual.noise = wifi_mac_cal_noise(wnet_vif->vm_mainsta->sta_avg_bcn_rssi,wnet_vif->vm_mainsta->sta_avg_snr);
                wstats->qual.updated = IW_QUAL_ALL_UPDATED | IW_QUAL_DBM;
            }
            else
            {
                wstats->qual.qual = 0;
                wstats->qual.level = 0;
                wstats->qual.noise = 0;
                wstats->qual.updated = IW_QUAL_ALL_UPDATED | IW_QUAL_DBM;
            }
        }
    }

    return wstats;
}

int iw_standard_get_freq(struct net_device *dev, struct iw_request_info *info,
    union iwreq_data *wrqu, char *extra)
{
    struct iw_freq * pr_iw_freq = &(wrqu->freq);
    struct wlan_net_vif *wnet_vif = netdev_priv(dev);
    struct wifi_mac *wifimac = wnet_vif->vm_wmac;
    enum nl80211_iftype iftype;


    list_for_each_entry(wnet_vif, &wifimac->wm_wnet_vifs, vm_next) {
        if (wnet_vif->vm_ndev == NULL || !(wnet_vif->vm_ndev->flags & IFF_RUNNING)) {
            continue;
        }
        iftype = wnet_vif->vm_wdev->iftype;

        if (iftype == NL80211_IFTYPE_STATION) {

            if (wnet_vif->vm_mainsta && wnet_vif->vm_state == WIFINET_S_CONNECTED) {
                pr_iw_freq->m = wnet_vif->vm_curchan->chan_cfreq1;
                pr_iw_freq->e = 6;
            }
        }
    }
     return 0;
}

static const iw_handler standard_handler[] = {
    IW_HANDLER(SIOCGIWSTATS,    (iw_handler)iw_standard_get_stats),
    IW_HANDLER(SIOCSIWFREQ,     (iw_handler)iw_standard_sap_set_freq),
    IW_HANDLER(SIOCSIWPOWER,    (iw_handler)iw_standard_set_pwr),
    IW_HANDLER(SIOCGIWNAME,    (iw_handler)iw_standard_get_name),
    IW_HANDLER(SIOCGIWRANGE,   (iw_handler)iw_standard_get_range),
    IW_HANDLER(SIOCGIWAP,      (iw_handler)iw_standard_get_ap),
    IW_HANDLER(SIOCGIWESSID,   (iw_handler)iw_standard_get_essid),
    IW_HANDLER(SIOCGIWFREQ,    (iw_handler)iw_standard_get_freq),

};


#if defined(CONFIG_WEXT_PRIV) || LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 32)
static iw_handler aml_iwpriv_private_handler[] = {
    aml_ap_send_addba_req, //index=0
    aml_iwpriv_send_para1, //index=1
    /*if we need feed back the value to user space, we need jump command for large buffer*/
    aml_iwpriv_get, //index=2
    NULL, //index=3
    aml_ap_set_udp_info, //index=4
    aml_iwpriv_get_int, //index=5
    aml_ap_set_arp_rx, //index=6
    aml_iwpriv_get_char, //index=7
    aml_ap_set_amsdu_state, //index=8
    aml_ap_set_ampdu_state, //index=9
    aml_set_country_code, //index=10
    aml_iwpriv_send_para2, //index=11
    aml_iwpriv_set_reg_legacy, //index=12
    aml_set_dev_sn, //index=13
    aml_set_efuse_wifi_mac, //index=14
    NULL, //necessary,reserve enough kernel space to copy data, index=15
    NULL, //index=16
    NULL, //index=17
    aml_set_efuse_btmac,//index=18
    NULL, //necessary,reserve enough kernel space to copy data, //index=19
    aml_iwpriv_set_debug,//index=20
    aml_iwpriv_start_capture,//index=21
    aml_iwpriv_get_csi_info,//index=22
    aml_iwpriv_send_para3,//index=23
    aml_iwpriv_set_wmme,//index=24
};

static const struct iw_priv_args aml_iwpriv_private_args[] = {
/*iwpriv set command, there is more parameters*/
{
    SIOCIWFIRSTPRIV,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 7, 0, "set_addba_req"},

/*iwpriv set command, there is one parameters*/
{
    SIOCIWFIRSTPRIV + 1,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, ""},
{
    AML_IWP_11H,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_11h"},
{
    AML_IWP_ROAM_THRESH_2G,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_roam_2gthr"},
{
    AML_IWP_ROAM_THRESH_5G,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_roam_5gthr"},
{
    AML_IWP_ROAM_MODE,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_roam_mode"},
{
    AML_IWP_MARK_DFS_CHAN,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_dfs_mark"},
{
    AML_IWP_UNMARK_DFS_CHAN,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_dfs_unmark"},
{
    AML_IWP_WEAK_THR_NARROW,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_nb_thr"},
{
    AML_IWP_WEAK_THR_WIDE,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_wb_thr"},
{
    AML_IWP_EAT_COUNT,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_eat_count"},
{
    AML_IWP_AGGR_THRESH,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_aggr_thr"},
{
    AML_IWP_HEART_INTERVAL,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_hrt_intv"},
{
    AML_IWP_BSS_COEX,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_bss_coex"},
{
    AML_IWP_TXAGGR_STA,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "get_txaggr_sta"},
{
    AML_IWP_WMM_AC_DELTS,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_wmm_delts"},
{
    AML_IWP_SHORT_GI,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_short_gi"},
{
    AML_IWP_DYNAMIC_BW,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_dynamic_bw"},
{
    AML_IWP_SET_BCN_INTERVAL,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_bcn_intv"},
{
    AML_IWP_SET_LDPC,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_ldpc"},
{
    AML_IWP_SET_SCAN_TIME_IDLE,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_sc_idltime"},
{
    AML_IWP_SET_SCAN_TIME_CONNECT,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_sc_contime"},
{
    AML_IWP_SET_SCAN_HANG,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_sc_hang"},
{
    AML_IWP_EN_BTWIFI_COEX,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_coex_btwifi"},
{
    AML_IWP_SET_COEXIST_MAX_MISS_BCN_CNT,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_coex_bcnmis"},
{
    AML_IWP_SET_COEXIST_REQ_TIMEOUT,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_wifi_slcreq"},
{
    AML_IWP_SET_COEXIST_NOT_GRANT_WEIGHT,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_wifi_grtwgt"},
{
    AML_IWP_SET_RATE_LEGACY,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_rate_legacy"},
{
    AML_IWP_SET_RATE_HT,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_rate_ht"},
{
    AML_IWP_SET_RATE_VHT,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_rate_vht"},
{
    AML_IWP_SET_POWER,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_pwr_save"},
{
    AML_IWP_SET_CHL_RSSI,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_chl_rssi"},
{
    AML_IWP_SET_BURST,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_burst"},
{
    AML_IWP_SET_UAPSD,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_uapsd"},
{
    AML_IWP_SET_PT_RX_START,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_pt_rxstart"},
{
    AML_IWP_SET_SCAN_PRI,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_scan_pri"},
{
    AML_IWP_SET_BE_BK_NOQOS_PRI,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_bebk_pri"},
{
    AML_IWP_SET_FETCH_PKT_METHOD,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_pkt_fetch"},
{
    AML_IWP_SET_FRAG_THRESHOLD,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_frag_thr"},
{
    AML_IWP_SET_PREAMBLE_TYPE,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_preamble"},
{
    AML_IWP_SET_FIX_BAND,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_band"},
{
    AML_IWP_SET_GAIN,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_gain_hang"},
{
    AML_IWP_SET_TPC,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_tpc_hang"},
{
    AML_IWP_SET_TXPW_PLAN,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_txpwr_plan"},
{
    AML_IWP_SET_MAC_MODE,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_mac_mode"},
{
    AML_IWP_GET_HOST_LOG,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "get_host_log"},
{
    AML_IWP_GET_FW_LOG,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "get_fw_log"},
{
    AML_IWP_SET_RECOVERY,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_recovery"},
{
    AML_IWP_ENABLE_FW_LOG,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_fwlog_en"},
{
    AML_IWP_GET_SPEC_REGS,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "get_spec_regs"},
{
    AML_IWP_SET_CF_END,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_cf_end"},
{
    AML_IWP_SET_FLOW_CTRL,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_flow_ctrl"},
{
    AML_IWP_SET_RX_SIZE,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_rx_buf"},
{
    AML_IWP_SET_TX_BW,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_tx_bw"},
{
    AML_IWP_SET_TX_MODE,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_tx_mode"},
{
    AML_IWP_SET_TX_RATE,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_tx_rate"},
{
    AML_IWP_SET_TX_TYPE,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_tx_type"},
{
    AML_IWP_SET_TX_LEN,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_tx_len"},
{
    AML_IWP_SET_TX_NUM,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_tx_num"},
{
    AML_IWP_PT_RX_START,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "pt_rx_start"},
{
    AML_IWP_SET_STBC,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_stbc"},
{
    AML_IWP_SET_TX_PWR,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_tx_pwr"},
{
    AML_IWP_SET_DELTA_TX_PWR,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_delta_pwr"},
{
    AML_IWP_SET_WFA_MODE,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_wfa"},


/*iwpriv get command*/
{
    /*if we need feed back the value to user space, we need jump command for large buffer*/
    SIOCIWFIRSTPRIV + 2,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, ""},
{
    AML_IWP_AMSDU_STATE,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_amsdu"},
{
    AML_IWP_AMPDU_STATE,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_ampdu"},
{
    AML_IWP_UDP_INFO,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_udp_info"},
{
    AML_IWP_COUNTRY,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_country"},
{
    AML_IWP_CHAN_LIST,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_chan_list"},
{
    AML_IWP_CHIP_ID,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_chip_id"},
{
    AML_IWP_AP_IP,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "set_arp_req"},
{
    AML_IWP_ROAM_CHAN,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_roam_chan"},
{
    AML_IWP_SESSION,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_wfd_session"},
#ifdef CONFIG_P2P
{
    AML_IWP_P2P_DEV_ID,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_p2p_dev_id"},
#endif
{
    AML_IWP_WIFI_MAC,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_wifi_mac"},
{
    AML_IWP_CCA_BUSY_CHECK,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_cca_stat"},
{
    AML_IWP_PHY_STATISTIC,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_agc_stat"},
{
    AML_IWP_PRINT_VERSION,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_drv_ver"},
{
    AML_IWP_GET_TX_STATUS,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_tx_stat"},
{
    AML_IWP_SET_RATE_AUTO,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "set_rate_auto"},
{
    AML_IWP_SET_PT_RX_STOP,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "set_pt_rxstop"},
{
    AML_IWP_GET_EN_RF_TEST,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_en_rf_test"},
{
    AML_IWP_PT_TX_START,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "pt_tx_start"},
{
    AML_IWP_PT_TX_END,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "pt_tx_end"},
{
    AML_IWP_GET_CAPT_STATICS,
    0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_rate_stat"},


{
    SIOCIWFIRSTPRIV + 4,
    IW_PRIV_TYPE_CHAR | 40, 0, "set_udp_info"},

{
    SIOCIWFIRSTPRIV + 5,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_MASK, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_MASK, ""},
{
    AML_IWP_GET_EFUSE_TIMES,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_MASK, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_MASK, "get_efuse_times"},


{
    SIOCIWFIRSTPRIV + 6,
    IW_PRIV_TYPE_CHAR | 40, 0, "set_arp_rx"},

{
    SIOCIWFIRSTPRIV + 7,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, ""},
{
    AML_IWP_GET_REG,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, "get_reg"},
{
    AML_IWP_LEGACY_GET_REG,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, "get_reg_legacy"},
{
    AML_IWP_GET_EFUSE,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, "get_efuse"},
{
    AML_IWP_GET_WIFI_MAC,
   IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, "get_mac_addr"},
{
    AML_IWP_GET_BT_MAC,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, "get_bt_dev_id"},
{
    AML_IWP_GET_EFUSE_DOMAIN,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, "get_efuse_dom"},
{
    AML_IWP_GET_EFUSE_TMP,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, "get_efuse_tmp"},
{
    AML_IWP_PT_RX_END,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, "pt_rx_end"},
{
    AML_IWP_GET_DEV_SN,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, "get_dev_sn"},
{
    AML_IWP_SET_ANT_SWITCH,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, "ant_switch"},
{
    AML_IWP_GET_BT_DIGITAL_GAIN_EFUSE_TIMES,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, "get_bt_dg_times"},
{
    AML_IWP_GET_BT_DIGITAL_GAIN_EFUSE,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, "get_bt_dg"},
{
    AML_IWP_GET_WIFI_PERFORMANCE_INFO,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, "get_per_info"},

{
    SIOCIWFIRSTPRIV + 8,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 3, 0, "set_amsdu"},
{
    SIOCIWFIRSTPRIV + 9,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 3, 0, "set_ampdu"},
{
    SIOCIWFIRSTPRIV + 10,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 3, 0, "set_country"},

/*iwpriv set command, there is 2 parameters*/
{
    SIOCIWFIRSTPRIV + 11,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, 0, ""},
{
    AML_IWP_SET_REG,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, 0, "set_reg"},
{
    AML_IWP_SET_EFUSE,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, 0, "set_efuse"},
{
    AML_IWP_SET_EFUSE_DOMAIN,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, 0, "set_efuse_dom"},
{
    AML_IWP_SET_PT_EFUSE,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, 0, "set_pt_efuse"},
{
    AML_IWP_MEM_DUMP,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, 0, "mem_dump"},
#if defined(SU_BF) || defined(MU_BF)
{
    AML_IWP_SET_BEAMFORMING,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, 0, "set_beamforming"},
#endif
{
    AML_IWP_SET_PT_CALIBRATION,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, 0, "set_pt_cali"},
{
    AML_IWP_SET_TX_PATH,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, 0, "set_tx_path"},
{
    AML_IWP_SET_RX_PATH,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, 0, "pt_set_rx"},
{
    AML_IWP_SET_BT_DIGITAL_GAIN_EFUSE,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, 0, "set_bt_dg"},

    /*iwpriv set command, there is 4 parameters*/
{
    SIOCIWFIRSTPRIV + 12,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 4, 0, "set_reg_legacy"},

{
    SIOCIWFIRSTPRIV + 13,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 6,0, "set_dev_sn"},

{
    SIOCIWFIRSTPRIV + 14,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 18,0, "set_mac_addr"},
{
    SIOCIWFIRSTPRIV + 18,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 18,0, "set_bt_dev_id"},
{
    SIOCIWFIRSTPRIV + 20,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_MASK, 0, "set_debug"},
{
     SIOCIWFIRSTPRIV + 21,
    0, IW_PRIV_TYPE_CHAR | 0, "set_capture"},
{
     SIOCIWFIRSTPRIV + 22,
    0, IW_PRIV_TYPE_BYTE | IW_PRIV_SIZE_MASK, "get_csi_info"},

    /*iwpriv set command, there is 3 parameters*/
{
    SIOCIWFIRSTPRIV + 23,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 3, 0, ""},
{
    AML_IWP_GET_KEY_ENTRY,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 3, 0, "get_key_entry"},
{
    AML_IWP_SET_EFUSE_TMP,
    IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 3, 0, "set_efuse_tmp"},
{
     SIOCIWFIRSTPRIV + 24,
    IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 34, 0, "set_wmme"},

};
#endif


#ifdef CONFIG_WIRELESS_EXT
struct iw_handler_def w1_iw_handle = {
#if defined(CONFIG_WEXT_PRIV) || LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 32)
    .num_standard = sizeof(standard_handler) / sizeof(standard_handler[0]),
    .num_private = ARRAY_SIZE(aml_iwpriv_private_handler),
    .num_private_args = ARRAY_SIZE(aml_iwpriv_private_args),
    .standard = (iw_handler *)standard_handler,
    .private = aml_iwpriv_private_handler,
    .private_args = aml_iwpriv_private_args,
    .get_wireless_stats = aml_get_wireless_stats,
#endif
};
#endif
