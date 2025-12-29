#ifdef CONFIG_USB
#ifndef __WIFI_USB__
#define __WIFI_USB__

#include "wifi_hif.h"
#if defined (HAL_SIM_VER)
#include "usb_test.h"
#endif
#include <linux/usb.h>

#define W1u_PRODUCT  0x4c55
#define W1u_VENDOR  0x414D

#define W1u_VENDOR_AMLOGIC_EFUSE  0x1B8E
#define W1uu_C_PRODUCT_AMLOGIC_EFUSE  0x0541
#define W1uu_B_PRODUCT_AMLOGIC_EFUSE  0x0501
#define W1uu_A_PRODUCT_AMLOGIC_EFUSE  0x04C1

/*Macro for Write/Read reg via endpoin 0*/
//#define REG_CTRL_EP0

#define AML_USB_CONTROL_MSG_TIMEOUT 500

/* USB request types */
#define AML_USB_REQUEST          ( USB_TYPE_VENDOR | USB_RECIP_DEVICE )
#define AML_USB_REQUEST_IN       ( USB_DIR_IN | AML_USB_REQUEST )
#define AML_USB_REQUEST_OUT      ( USB_DIR_OUT | AML_USB_REQUEST )

#if defined (HAL_SIM_VER)
#define CRG_XFER_TO_DEVICE 	0
#define CRG_XFER_TO_HOST 	0x80
#endif
#define AML_SIG_CBW	0x43425355
#define AML_XFER_TO_DEVICE 	0
#define AML_XFER_TO_HOST 	0x80
#define USB_MAX_TRANS_SIZE (64 * 1024)
#define USB_CTRL_IN_REQTYPE (USB_DIR_IN | USB_TYPE_VENDOR | (USB_RECIP_ENDPOINT & 0x1f))
#define USB_CTRL_OUT_REQTYPE (USB_DIR_OUT | USB_TYPE_VENDOR | (USB_RECIP_ENDPOINT & 0x1f))

struct aml_hwif_usb {
    bool scatter_enabled;
    /* protects access to scat_req */
    OS_LOCK scat_lock;
    /* scatter request list head */
    struct amlw_hif_scatter_req *scat_req;
};


enum wifi_cmd {
    CMD_DOWNLOAD_WIFI = 0xC1,
    CMD_START_WIFI,
    CMD_STOP_WIFI,
    CMD_READ_REG,
    CMD_WRITE_REG,
    CMD_READ_PACKET,
    CMD_WRITE_PACKET,
    CMD_WRITE_SRAM,
    CMD_READ_SRAM,
    CMD_DOWNLOAD_BT,
    CMD_OTHER_CMD,
    CMD_USB_IRQ,
    CMD_READ_EFUSE = 0xCD,
    CMD_WRITE_EFUSE,
};

enum usb_endpoint_num{
    USB_EP0 = 0x0,
    USB_EP1,
    USB_EP2,
};

struct crg_msc_cbw {
    uint32_t sig;
    uint32_t tag;
    uint32_t data_len;
    uint8_t flag;
    uint8_t lun;
    uint8_t len;
    uint32_t cdb[4];
    uint8_t reserved; /* make sure 32 bype alined */
    uint8_t buf[480]; /* reserved */
//uint8_t resv;
}__attribute__ ((packed));

struct usb_hub {
	struct device		*intfdev;	/* the "interface" device */
	struct usb_device	*hdev;
};

int aml_usb_init(void);
void aml_usb_exit(void);
void aml_usb_disable_wifi(void);
void aml_usb_enable_wifi(void);
int start_wifi(void);
int stop_wifi(void);
int wifi_fw_download(void);
int wifi_iccm_download(unsigned char* addr, unsigned int len);
int wifi_dccm_download(unsigned char* addr, unsigned int len);
unsigned int aml_usb_read_word(unsigned int addr);
unsigned long aml_usb_read_byte(unsigned long addr);
void aml_usb_write_word(unsigned int addr,unsigned int data);
void aml_usb_write_byte(unsigned long addr, unsigned long data);
void aml_usb_write_sram(unsigned char *buf, unsigned char *sram_addr, SYS_TYPE len);
void aml_usb_read_sram(unsigned char *buf,unsigned char *sram_addr, SYS_TYPE len);
void hif_init_usb_ops(void);
int aml_usb_send_frame(struct amlw_hif_scatter_req * pframe);
int hal_tx_page_build(struct hi_tx_desc * pTxDPage);
int aml_usb_port_suspend(struct usb_device *udev);
int aml_usb_port_resume(struct usb_device *udev);
int aml_usb_send_packet(struct amlw_hif_scatter_req * scat_req);
void usb_stor_control_msg(unsigned long data );

#ifndef SDIO_MODE_ON
void aml_aon_write_reg(unsigned int addr,unsigned int data);
unsigned int aml_aon_read_reg(unsigned int addr);
void set_reg_fragment(unsigned int addr, unsigned int bit_end,
        unsigned int bit_start,unsigned int value);
#endif
#endif
#endif
