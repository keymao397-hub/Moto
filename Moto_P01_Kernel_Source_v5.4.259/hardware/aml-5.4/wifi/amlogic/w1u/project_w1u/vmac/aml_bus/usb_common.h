#include <linux/init.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/kernel.h>

#ifdef CONFIG_USB
#define OS_LOCK spinlock_t

#define USB_MAXSG_SIZE 32
#define USB_MAX_SG_ENTRIES (USB_MAXSG_SIZE+2)
typedef unsigned long SYS_TYPE_U;

#define PRINT_U(...)      do {printk("aml_usb_common->");printk( __VA_ARGS__ );}while(0)
#ifndef ASSERT
#define ASSERT(exp) do{    \
                if (!(exp)) {   \
                        printk("=>=>=>=>=>assert %s,%d\n",__func__,__LINE__);   \
                        /*BUG();        while(1);   */  \
                }                       \
        } while (0);
#endif

#define AML_PRINT_LOG_ERR(format,...) do {    \
                 printk("FUNCTION: %s LINE: %d:"format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
        } while (0)


struct mutex auc_usb_mutex;

#define USB_BEGIN_LOCK() do {\
    mutex_lock(&auc_usb_mutex);\
} while (0)

#define USB_END_LOCK() do {\
    mutex_unlock(&auc_usb_mutex);\
} while (0)

#define USB_LOCK_INIT()  mutex_init(&auc_usb_mutex)
#define USB_LOCK_DESTROY() mutex_destroy(&auc_usb_mutex);

#define ZMALLOC(size, name, gfp) kzalloc(size, gfp)
#define FREE(a, name) kfree(a)


#define W1u_PRODUCT  0x4c55
#define W1u_VENDOR  0x414D

#define W1u_VENDOR_AMLOGIC_EFUSE  0x1B8E
#define W1uu_C_PRODUCT_AMLOGIC_EFUSE  0x0541
#define W1uu_B_PRODUCT_AMLOGIC_EFUSE  0x0501
#define W1uu_A_PRODUCT_AMLOGIC_EFUSE  0x04C1

/*Macro for Write/Read reg via endpoin 0*/
//#define REG_CTRL_EP0

#define AML_USB_CONTROL_MSG_TIMEOUT 3000

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

#ifdef CHIP_RESET_SUPPORT
struct auc_reset_ops {
    int (*probe_cb)(void);
    void (*disconnect_cb)(void);
};
#endif


struct amlw1u_hif_scatter_item {
    struct sk_buff *skbbuf;
    int len;
    int page_num;
    void *packet;
};


enum auc_cmd {
    CMD_DOWNLOAD_WIFI = 0xC1,
    CMD_START_WIFI,
    CMD_STOP_WIFI,
    CMD_READ_REG,
    CMD_WRITE_REG,
    CMD_READ_PACKET,
    CMD_WRITE_PACKET,
    CMD_WRITE_SRAM,
    CMD_READ_SRAM,
    CMD_OTHER_CMD,
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
    struct device       *intfdev;   /* the "interface" device */
    struct usb_device   *hdev;
};

/*auc--amlogic usb common for wifi usb*/
struct auc_hif_ops_for_wifi {
    int (*hi_bottom_write8)(unsigned char  func_num, int addr, unsigned char data);
    unsigned char (*hi_bottom_read8)(unsigned char  func_num, int addr);
    int (*hi_bottom_read)(unsigned char func_num, int addr, void *buf, size_t len, int incr_addr);
    int (*hi_bottom_write)(unsigned char func_num, int addr, void *buf, size_t len, int incr_addr);

    unsigned char (*hi_read8_func0)(unsigned long sram_addr);
    void (*hi_write8_func0)(unsigned long sram_addr, unsigned char sramdata);

    unsigned long (*hi_read_reg8)(unsigned long sram_addr);
    void (*hi_write_reg8)(unsigned long sram_addr, unsigned long sramdata);
    unsigned long (*hi_read_reg32)(unsigned long sram_addr);
    int (*hi_write_reg32)(unsigned long sram_addr, unsigned long sramdata);

    void (*hi_write_cmd)(unsigned long sram_addr, unsigned long sramdata);
    void (*hi_write_sram)(unsigned char*buf, unsigned char* addr, SYS_TYPE_U len);
    void (*hi_read_sram)(unsigned char* buf, unsigned char* addr, SYS_TYPE_U len);
    void (*hi_write_word)(unsigned int addr,unsigned int data);
    unsigned int (*hi_read_word)(unsigned int addr);

    void (*hi_rcv_frame)(unsigned char* buf, unsigned char* addr, SYS_TYPE_U len);

    int (*hi_enable_scat)(void);
    void (*hi_cleanup_scat)(void);
    struct amlw_hif_scatter_req * (*hi_get_scatreq)(void);
    int (*hi_scat_rw)(struct scatterlist *sg_list, unsigned int sg_num, unsigned int blkcnt,
        unsigned char func_num, unsigned int addr, unsigned char write);
    int (*hi_send_frame)(struct amlw_hif_scatter_req *scat_req);

    /*bt use*/
    void (*bt_hi_write_sram)(unsigned char* buf, unsigned char* addr, SYS_TYPE_U len);
    void (*bt_hi_read_sram)(unsigned char* buf, unsigned char* addr, SYS_TYPE_U len);
    void (*bt_hi_write_word)(unsigned int addr,unsigned int data);
    unsigned int (*bt_hi_read_word)(unsigned int addr);

    void (*hif_get_sts)(unsigned int op_code, unsigned int ctrl_code);
    void (*hif_pt_rx_start)(unsigned int qos);
    struct rx_statics_st (*hif_pt_rx_stop)(void);

    int (*hif_suspend)(unsigned int suspend_enable);
};

/*auc--amlogic usb common for bt*/
struct auc_hif_ops {
    int (*hi_send_cmd)(unsigned int addr, unsigned int len);
    void (*hi_write_word)(unsigned int addr,unsigned int data, unsigned int ep);
    unsigned int (*hi_read_word)(unsigned int addr, unsigned int ep);
    void (*hi_write_sram)(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep);
    void (*hi_read_sram)(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep);

    void (*hi_rx_buffer_read)(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep);

    /*bt use*/
    void (*hi_write_word_for_bt)(unsigned int addr,unsigned int data, unsigned int ep);
    unsigned int (*hi_read_word_for_bt)(unsigned int addr, unsigned int ep);
    void (*hi_write_sram_for_bt)(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep);
    void (*hi_read_sram_for_bt)(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep);

    int (*hi_enable_scat)(void);
    void (*hi_cleanup_scat)(void);
    struct amlw_hif_scatter_req * (*hi_get_scatreq)(void);
    int (*hi_scat_rw)(struct scatterlist *sg_list, unsigned int sg_num, unsigned int blkcnt,
        unsigned char func_num, unsigned int addr, unsigned char write);

    int (*hi_send_frame)(struct amlw_hif_scatter_req *scat_req);
    void (*hi_rcv_frame)(unsigned char* buf, unsigned char* addr, unsigned long len);
};

int aml_usb_insmod(void);
void aml_usb_rmmod(void);
#endif
