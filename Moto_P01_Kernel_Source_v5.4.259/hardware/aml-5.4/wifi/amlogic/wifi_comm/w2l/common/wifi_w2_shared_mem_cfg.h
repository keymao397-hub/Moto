#ifndef __WIFI_W2_SHARED_MEM_CFG_H__
#define __WIFI_W2_SHARED_MEM_CFG_H__

/* orignal patch_rxdesc bigger than mini patch_rxdesc 136 byte */
/* reference code patch_fwmain.c line 400 */
#define SHARED_MEM_BASE_ADDR             (0x60000000)
#define TXPAGE_DESC_ADDR                 (0x6000d07c) /* size 0x114  */
#define TXL_TBD_START                    (0x6000d190) /* size 0x2d00 */
#define TXLBUF_TAG_SDIO                  (0x6000fe90) /* size 0x3a00 */

#define HW_RXBUF2_START_ADDR             (0x60013890) /* size 0x2BC */
#define HW_RXBUF1_START_ADDR             (0x60013b4c)
/*
LA OFF: rx buffer large size 0x40000, small size: 0x10000;
LA ON: rx buffer large size 0x30000, small size: 0x20000
*/
#define RXBUF_START_ADDR                 (0x60013b4c)
#define RXBUF_END_ADDR_SMALL             (0x6001e000) /*rx buf size: (0xA4B4)*/
#define RXBUF_END_ADDR_LARGE             (0x60053b4c) /*rx buf size: (256K)*/

#define TXBUF_START_ADDR                 (RXBUF_END_ADDR_SMALL)
#define USB_RXBUF_END_ADDR_SMALL         (0x60029130) /*rx buf size:0x1D454 (117.08K)*/
#define USB_TXBUF_START_ADDR             (USB_RXBUF_END_ADDR_SMALL)

#if defined (USB_TX_USE_LARGE_PAGE) || defined (CONFIG_AML_USB_LARGE_PAGE)
#define USB_RXBUF_END_ADDR_LARGE         (0x600684b0) /*rx buf size:(256K)*/
#else
#define USB_RXBUF_END_ADDR_LARGE         (0x60053b4c) /*rx buf size:(256K)*/
#endif

#define RX_BUFFER_LEN_SMALL              (RXBUF_END_ADDR_SMALL - RXBUF_START_ADDR)
#define RX_BUFFER_LEN_LARGE              (RXBUF_END_ADDR_LARGE - RXBUF_START_ADDR)
#define USB_RX_BUFFER_LEN_SMALL          (USB_RXBUF_END_ADDR_SMALL - RXBUF_START_ADDR)
#define USB_RX_BUFFER_LEN_LARGE          (USB_RXBUF_END_ADDR_LARGE - RXBUF_START_ADDR)

#define TRX_BUF_SIZE        (0x60080000 - RXBUF_START_ADDR)
/* usb trace use dccm */
#define USB_TRACE_START_ADDR             (0x00d27800) /* trace size: 0x6800 */
#define USB_TRACE_END_ADDR               (0x00d2e000)
/* sdio trace use sram 27K size */
#define SDIO_TRACE_START_ADDR            (0xa10800)  /* trace size: 0x6800 */
#define SDIO_TRACE_END_ADDR              (0xa17000)

#define TRACE_TOTAL_SIZE    (0x6800)
#define TRACE_MAX_SIZE      (TRACE_TOTAL_SIZE >> 1) /* trace max size is total size 1/2 */

#define LA_START_ADDR       (0x60070000)
#define LA_LENGTH           (0x10000)

#define SDIO_USB_EXTEND_E2A_IRQ_STATUS CMD_DOWN_FIFO_FDN_ADDR

/* SDIO USB E2A EXTEND IRQ TYPE */
enum sdio_usb_e2a_irq_type {
    DYNAMIC_BUF_HOST_TX_STOP  = 1,
    DYNAMIC_BUF_HOST_TX_START,
    DYNAMIC_BUF_NOTIFY_FW_TX_STOP,
    DYNAMIC_BUF_LA_SWITCH_FINSH,
    EXCEPTION_IRQ,
};

struct sdio_buffer_control
{
    unsigned char flag;
    unsigned int tx_start_time;
    unsigned int tx_total_len;
    unsigned int rx_start_time;
    unsigned int rx_total_len;
    unsigned int tx_rate;
    unsigned int rx_rate;
    unsigned int buffer_status;
    unsigned int hwwr_switch_addr;
};
extern struct sdio_buffer_control sdio_buffer_ctrl;

#define BUFFER_TX_USED            BIT(0)
#define BUFFER_RX_USED            BIT(1)
#define BUFFER_TX_NEED_ENLARGE    BIT(2)
#define BUFFER_RX_NEED_ENLARGE    BIT(3)
#define BUFFER_RX_REDUCED          BIT(4)
#define BUFFER_RX_HOST_NOTIFY      BIT(5)
#define BUFFER_RX_REDUCE_FLAG      BIT(6)
#define BUFFER_RX_ENLARGE_FINSH    BIT(7)
#define BUFFER_RX_FORCE_REDUCE     BIT(8)
#define BUFFER_RX_FORCE_ENLARGE    BIT(9)
#define BUFFER_RX_FORBID_REDUCE    BIT(10)
#define BUFFER_RX_FORBID_ENLARGE   BIT(11)
#define BUFFER_RX_WAIT_READ_DATA   BIT(12)
#define BUFFER_TX_STOP_FLAG        BIT(13)
#define BUFFER_RX_ENLARGE_FLAG     BIT(14)

#define RX_ENLARGE_READ_RX_DATA_FINSH BIT(25)
#define HOST_RXBUF_ENLARGE_FINSH      BIT(26)
#define RX_REDUCE_READ_RX_DATA_FINSH  BIT(27)
#define HOST_RXBUF_REDUCE_FINSH       BIT(28)

#define FW_BUFFER_STATUS   (BIT(20) | BIT(21))
#define FW_BUFFER_NARROW   BIT(20)
#define FW_BUFFER_EXPAND   BIT(21)
#define RX_WRAP_FLAG       BIT(31)
#define RX_WRAP_TEMP_FLAG  BIT(19)

#define RX_HAS_DATA        BIT(0)

#define CHAN_SWITCH_IND_MSG_ADDR       (0xa17fc0)
#define EXCEPTION_INFO_ADDR            (0xa17fc8)

struct exceptinon_info
{
    uint8_t  type;
    uint32_t mstatus_mps_bits;
    uint32_t mepc;
    uint32_t mtval;
    uint32_t mcause;
    uint32_t sp;
};

#define SDIO_IRQ_E2A_CHAN_SWITCH_IND_MSG           CO_BIT(15)

#define UNWRAP_SIZE (56)

#endif
