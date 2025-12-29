
#include "wifi_debug.h"
#include "wifi_mac_com.h"

const char * dbg_level_str[] = { "E", "W", "I", "D"};

moduleTraceInfo gAmlTraceInfo[ AML_LOG_ID_MAX ] =
{
    [AML_LOG_ID_LOG]           = { AML_LOG_LEVEL_DEFAULT, "LOG"  },
    [AML_LOG_ID_ACL]           = { AML_LOG_LEVEL_DEFAULT, "ACL"  },
    [AML_LOG_ID_XMIT]          = { AML_LOG_LEVEL_DEFAULT, "TX"   },
    [AML_LOG_ID_KEY]           = { AML_LOG_LEVEL_DEFAULT, "KEY"  },
    [AML_LOG_ID_STATE]         = { AML_LOG_LEVEL_DEFAULT, "SM"   },
    [AML_LOG_ID_RATE]          = { AML_LOG_LEVEL_DEFAULT, "RATE" },
    [AML_LOG_ID_RECV]          = { AML_LOG_LEVEL_DEFAULT, "RX"   },
    [AML_LOG_ID_P2P]           = { AML_LOG_LEVEL_DEFAULT, "P2P"  },
    [AML_LOG_ID_CFG80211]      = { AML_LOG_LEVEL_DEFAULT, "CFG"  },
    [AML_LOG_ID_SCAN]          = { AML_LOG_LEVEL_DEFAULT, "SCN"  },
    [AML_LOG_ID_LOCK]          = { AML_LOG_LEVEL_DEFAULT, "LOCK" },
    [AML_LOG_ID_INIT]          = { AML_LOG_LEVEL_DEFAULT, "INIT" },
    [AML_LOG_ID_ROAM]          = { AML_LOG_LEVEL_DEFAULT, "ROAM" },
    [AML_LOG_ID_NODE]          = { AML_LOG_LEVEL_DEFAULT, "NODE" },
    [AML_LOG_ID_ANDROID]       = { AML_LOG_LEVEL_DEFAULT, "DRID" },
    [AML_LOG_ID_ACTION]        = { AML_LOG_LEVEL_DEFAULT, "ACT"  },
    [AML_LOG_ID_IOCTL]         = { AML_LOG_LEVEL_DEFAULT, "IOCL" },
    [AML_LOG_ID_CONNECT]       = { AML_LOG_LEVEL_DEFAULT, "CNT"  },
    [AML_LOG_ID_TIMER]         = { AML_LOG_LEVEL_DEFAULT, "TMER" },
    [AML_LOG_ID_ADDBA]         = { AML_LOG_LEVEL_DEFAULT, "ADDR" },
    [AML_LOG_ID_NETDEV]        = { AML_LOG_LEVEL_DEFAULT, "NDEV" },
    [AML_LOG_ID_HAL]           = { AML_LOG_LEVEL_DEFAULT, "HAL"  },
    [AML_LOG_ID_BEACON]        = { AML_LOG_LEVEL_DEFAULT, "BCN"  },
    [AML_LOG_ID_UAPSD]         = { AML_LOG_LEVEL_DEFAULT, "APSD" },
    [AML_LOG_ID_BWC]           = { AML_LOG_LEVEL_DEFAULT, "BWC"  },
    [AML_LOG_ID_ELEMID]        = { AML_LOG_LEVEL_DEFAULT, "EID"  },
    [AML_LOG_ID_PWR_SAVE]      = { AML_LOG_LEVEL_DEFAULT, "PS"   },
    [AML_LOG_ID_WME]           = { AML_LOG_LEVEL_DEFAULT, "WME"  },
    [AML_LOG_ID_DOTH]          = { AML_LOG_LEVEL_DEFAULT, "11H"  },
    [AML_LOG_ID_RATE_CTR]      = { AML_LOG_LEVEL_DEFAULT, "RCTR" },
    [AML_LOG_ID_TX_MSDU]       = { AML_LOG_LEVEL_DEFAULT, "MSDU" },
    [AML_LOG_ID_HAL_TX]        = { AML_LOG_LEVEL_DEFAULT, "HLT"  },
    [AML_LOG_ID_FILTER]        = { AML_LOG_LEVEL_DEFAULT, "FIL"  },
    [AML_LOG_ID_TX_REC]        = { AML_LOG_LEVEL_DEFAULT, "REC"  },

};


void aml_set_debug_level(MODULE_ID module_id, DEBUG_LEVEL level)
{
    if (module_id >= AML_LOG_ID_MAX) {
        AML_PRINT(AML_LOG_ID_LOG,AML_LOG_LEVEL_ERROR,"module_id[%d] is out of range!\n",module_id);
        return;
    }

    if ((level >= AML_LOG_LEVEL_MAX) || (level < AML_LOG_LEVEL_ERROR)) {
        AML_PRINT(AML_LOG_ID_LOG,AML_LOG_LEVEL_ERROR,"level[%d] is out of range!\n",level);
        return;
    }

    if (gAmlTraceInfo[module_id].moduleTraceLevel == level) {
        AML_PRINT(AML_LOG_ID_LOG,AML_LOG_LEVEL_DEBUG,"module:%s skip \n",gAmlTraceInfo[module_id].moduleNameStr);
        return;
    }

    gAmlTraceInfo[module_id].moduleTraceLevel = level;

    AML_PRINT(AML_LOG_ID_LOG,AML_LOG_LEVEL_INFO,"module:%s level:%s \n",
                        gAmlTraceInfo[module_id].moduleNameStr, dbg_level_str[level]);
    return;
}

void aml_set_all_debug_level(DEBUG_LEVEL level)
{
    unsigned int id = 0;

    if ((level >= AML_LOG_LEVEL_MAX) || (level < AML_LOG_LEVEL_ERROR)) {
        AML_PRINT(AML_LOG_ID_LOG,AML_LOG_LEVEL_ERROR,"level[%d] is out of range!\n",level);
        return;
    }

    AML_PRINT(AML_LOG_ID_LOG,AML_LOG_LEVEL_INFO,"level:%s \n", dbg_level_str[level]);

    for (id = 0; id < AML_LOG_ID_MAX; id++) {

        if (gAmlTraceInfo[id].moduleTraceLevel != level) {
            gAmlTraceInfo[id].moduleTraceLevel = level;
        } else {
            AML_PRINT(AML_LOG_ID_LOG,AML_LOG_LEVEL_DEBUG,"module:%s skip \n",gAmlTraceInfo[id].moduleNameStr);
        }

    }

    return;
}

void address_print( unsigned char* address )
{
    PUTX8(1, address[0] );
    PUTC( ':' );
    PUTX8( 1, address[1] );
    PUTC( ':' );
    PUTX8( 1, address[2] );
    PUTC( ':' );
    PUTX8( 1, address[3] );
    PUTC( ':' );
    PUTX8( 1, address[4] );
    PUTC( ':' );
    PUTX8( 1, address[5] );
    PUTC( '\n' );
}

void IPv4_address_print( unsigned char* address )
{
    PRINT("IPv4 address=");
    PUTU8( address[0] );
    PUTC( '.' );
    PUTU8( address[1] );
    PUTC( '.' );
    PUTU8( address[2] );
    PUTC( '.' );
    PUTU8( address[3] );
    PUTC('\n');
}

  void dump_memory_internal(unsigned char *data,int len)
{

    unsigned char *cursor=data;
    char *xcursor = (char *)data;
    int i,j;
    printk("\n*********************************\n");
    for (  i = 0; i < len; i++ )
    {
        if (( i != 0)&&(( i & 0x0F ) == 0 ))
        {

#ifdef  ASCII_IN
            printk("|");
            for (j=0; j < len; j++)
            {
                if (((*xcursor)<' ')||((*xcursor)>'~'))
                {
                    printk(".");
                    xcursor++;
                }
                else
                {
                    printk("%c",*xcursor++);
                }
            }
#endif
            printk( "\n");
        }

        printk( "%02x",*cursor++ );
        printk(" ");
    }

#ifdef  ASCII_IN
    if (len>LINEBYTE)
    {
        for (i=0; i<LINEBYTE-(len%LINEBYTE); i++)
        {
            printk("   ");
        }
        printk("|");
        for (i=0; i<(len%LINEBYTE); i++)
        {
            if (((*xcursor)<' ')||((*xcursor)>'~'))
            {
                printk(".");
                xcursor++;
            }
            else
            {
                printk("%c",*xcursor++);
            }
        }

    }
#endif
    printk("\n*********************************\n");
}

void address_read( unsigned char* cursor, unsigned char* address )
{
    *address++ = *cursor++;
    *address++ = *cursor++;
    *address++ = *cursor++;
    *address++ = *cursor++;
    *address++ = *cursor++;
    *address++ = *cursor++;
}




 unsigned short READ_16L( const unsigned char* address )
{
#ifdef CTRL_BYTE
    return address[0] | ( address[1] << 8 );
#else
    return * (unsigned short *)address ;
#endif
}


 void WRITE_16L( unsigned char* address, unsigned short value )
{
#ifdef CTRL_BYTE
    address[0] = ( value >> 0 ) & 0xFF;
    address[1] = ( value >> 8 ) & 0xFF;
#else
    * (unsigned short *)address = value;
#endif
}


unsigned int READ_32L( const unsigned char* address )
{
#ifdef CTRL_BYTE
    return address[0] | ( address[1] << 8 ) | ( address[2] << 16 ) | ( address[3] << 24 );

#else
    return * (unsigned int *)address ;

#endif
}


 void WRITE_32L( unsigned char* address, unsigned int value )
{
#ifdef CTRL_BYTE
    address[0] = ( value >>  0 ) & 0xFF;
    address[1] = ( value >>  8 ) & 0xFF;
    address[2] = ( value >> 16 ) & 0xFF;
    address[3] = ( value >> 24 ) & 0xFF;

#else
    * (unsigned int *)address = value;
#endif
}

unsigned short READ_16B( const unsigned char* address )
{
    return address[1] | ( address[0] << 8 );
}


void WRITE_16B( unsigned char* address, unsigned short value )
{
    address[1] = ( value >> 0 ) & 0xFF;
    address[0] = ( value >> 8 ) & 0xFF;
}


 unsigned int READ_32B( const unsigned char* address )
{
    return address[3] | ( address[2] << 8 ) | ( address[1] << 16 ) | ( address[0] << 24 );
}

 void WRITE_32B( unsigned char* address, unsigned int value )
{
    address[3] = ( value >>  0 ) & 0xFF;
    address[2] = ( value >>  8 ) & 0xFF;
    address[1] = ( value >> 16 ) & 0xFF;
    address[0] = ( value >> 24 ) & 0xFF;
}


void ie_dbg(unsigned char *ie )
{
    int i = 0;
    AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_DEBUG, "ie \n");
    for(i = 0; i < ie[1] + 2; i++)
    {
        AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_DEBUG, "ie 0x%x\n", ie[i]);
    }
}

void wifi_debug_dump_data(unsigned char* data, unsigned int size, unsigned char bytes_per_line)
{
    unsigned char line_data[32 * 3 + 1];
    unsigned int offset = 0;
    unsigned int pos = 0;
    unsigned char i;

    if (bytes_per_line > 32)
    {
        printk("input_bytes_num:%d, not support", bytes_per_line);
        return;
    }

    while (offset < size)
    {
        pos = 0;

        for (i = 0; (i < bytes_per_line) && (offset + i) < size; i++)
        {
            pos += sprintf(&line_data[pos], "%02X ", data[offset + i]);
        }

        line_data[pos - 1] = '\n';
        line_data[pos] = '\0';
        offset += bytes_per_line;
        printk("%s", line_data);
    }
}

unsigned char wifi_debug_is_arp_pkt(struct sk_buff * skb)
{
    struct wifi_mac_tx_info *txinfo = (struct wifi_mac_tx_info *)os_skb_cb(skb);
    struct wifi_mac_pkt_info *mac_pkt_info = &txinfo->ptxdesc->drv_pkt_info.pkt_info[0];

    return mac_pkt_info->b_arp;
}

unsigned char wifi_debug_get_tid_in_qos_ctrl(void *mac_header)
{
    struct wifi_qos_frame *qh = (struct wifi_qos_frame *)mac_header;
    unsigned char qos_ctrl0;

    qos_ctrl0 = qh->i_qos[0];

    return qos_ctrl0 & 0xf;
}
