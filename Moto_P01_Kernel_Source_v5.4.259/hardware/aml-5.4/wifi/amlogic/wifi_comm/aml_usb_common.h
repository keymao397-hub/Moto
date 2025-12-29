#ifndef _AML_USB_COMMON_H_
#define _AML_USB_COMMON_H_
#include <linux/init.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/firmware.h>


#define PRINT(...)      do {printk("aml_usb_common->");printk( __VA_ARGS__ );}while(0)
#ifndef ASSERT
#define ASSERT(exp) do{    \
                if (!(exp)) {   \
                        printk("=>=>=>=>=>assert %s,%d\n",__func__,__LINE__);   \
                        /*BUG();        while(1);   */  \
                }                       \
        } while (0);
#endif

#define ERROR_DEBUG_OUT(format,...) do {    \
                 printk("FUNCTION: %s LINE: %d:"format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
        } while (0)

#define ZMALLOC(size, name, gfp) kzalloc(size, gfp)
#define FREE(a, name) kfree(a)

/*Macro for Write/Read reg via endpoin 0*/
//#define REG_CTRL_EP0
#define USB_DIR_OUT 0 /* to device */
#define USB_DIR_IN 0x80 /* to host */
#define USB_TYPE_VENDOR (0x02 << 5)
#define USB_RECIP_ENDPOINT 0x02
#define AML_USB_CONTROL_MSG_TIMEOUT 3000

#if 0
/* USB request types */
#define AML_USB_REQUEST          ( USB_TYPE_VENDOR | USB_RECIP_DEVICE )
#define AML_USB_REQUEST_IN       ( USB_DIR_IN | AML_USB_REQUEST )
#define AML_USB_REQUEST_OUT      ( USB_DIR_OUT | AML_USB_REQUEST )
#endif

#define USB_CTRL_IN_REQTYPE (USB_DIR_IN | USB_TYPE_VENDOR | (USB_RECIP_ENDPOINT & 0x1f))


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
    CMD_GET_TX_CFM,
    CMD_OTHER_CMD,
    CMD_USB_IRQ
};


enum usb_endpoint_num{
    USB_EP0 = 0x0,
    USB_EP1,
    USB_EP2,
    USB_EP3,
    USB_EP4,
    USB_EP5,
    USB_EP6,
    USB_EP7,
};
#if 0

struct crg_msc_cbw {
    uint32_t sig;
    uint32_t tag;
    uint32_t data_len;
    uint8_t flag;
    uint8_t lun;
    uint8_t len;
    uint32_t cdb[4];
    unsigned char resv[481];
}__attribute__ ((packed));

struct usb_hub {
    struct device       *intfdev;   /* the "interface" device */
    struct usb_device   *hdev;
};

struct aml_hwif_usb {
    bool scatter_enabled;
    /* protects access to scat_req */
    OS_LOCK scat_lock;
    /* scatter request list head */
    struct amlw_hif_scatter_req *scat_req;
};
#endif
int aml_wifi_usb_insmod(void);
void aml_wifi_usb_rmmod(void);

#endif
