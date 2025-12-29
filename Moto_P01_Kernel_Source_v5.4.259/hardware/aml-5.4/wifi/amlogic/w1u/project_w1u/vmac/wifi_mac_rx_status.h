#ifndef __WIFI_MAC_RX_STATUS_H__
#define __WIFI_MAC_RX_STATUS_H__

struct wifi_mac_rx_status
{
    unsigned long long rs_pn;
    unsigned char rs_encrypt;
    int rs_flags;
    int rs_rssi;
    int rs_datarate;
    int rs_wnet_vif_id;
    int rs_channel;
    int rs_keyid;
    int rs_rate_index;
    unsigned int channel_bw;
    unsigned int rs_sgi;
    unsigned char rs_vendor_rate_code;

    union
    {
        unsigned char data[8];
        u_int64_t tsf;
    } rs_tstamp;
};

#endif//__WIFI_MAC_RX_STATUS_H__
