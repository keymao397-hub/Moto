#include "usb_common.h"
#include "chip_intf_reg.h"
#include "aml_interface.h"
unsigned char wifi_usb_access = 1;
typedef void (*lp_shutdown_func)(void);
lp_shutdown_func g_lp_shutdown_func;

#ifdef CONFIG_USB
struct auc_hif_ops_for_wifi g_auc_hif_ops_for_wifi;
struct auc_hif_ops g_auc_hif_ops;
struct usb_device *g_udev = NULL;
unsigned char auc_driver_insmoded;
unsigned char auc_wifi_in_insmod;
struct crg_msc_cbw *g_cmd_buf = NULL;
unsigned char *g_kmalloc_buf;
unsigned char auc_driver_probed = 0;

/*for bluetooth get read/write point*/
int bt_wt_ptr = 0;
int bt_rd_ptr = 0;
struct aml_pm_type g_wifi_pm;

typedef void (*bt_shutdown_func)(void);
bt_shutdown_func g_bt_shutdown_func = NULL;

void auc_build_cbw(struct crg_msc_cbw *cbw_buf,
                               unsigned char dir,
                               unsigned int len,
                               unsigned char cdb1,
                               unsigned int cdb2,
                               unsigned long cdb3,
                               SYS_TYPE_U cdb4)
{
    cbw_buf->sig = AML_SIG_CBW;
    cbw_buf->tag = 0x5da729a0;
    cbw_buf->data_len = len;
    cbw_buf->flag = dir; //direction
    cbw_buf->len = 16; //command length
    cbw_buf->lun = 0;

    cbw_buf->cdb[0] = cdb1;
    cbw_buf->cdb[1] = cdb2; // read or write addr
    cbw_buf->cdb[2] = (unsigned int)(unsigned long)cdb3;
    cbw_buf->cdb[3] = cdb4; //read or write data length
}


int auc_bulk_msg(struct usb_device *usb_dev, unsigned int pipe,
    void *data, int len, int *actual_length, int timeout)
{
    int ret;
    ret = usb_bulk_msg(usb_dev, pipe, data, len, actual_length, timeout);
    return ret;
}

int auc_send_cmd(unsigned int addr, unsigned int len)
{
    int ret;
    struct usb_device *udev = g_udev;
    unsigned int actual_length;

    USB_BEGIN_LOCK();
    auc_build_cbw(g_cmd_buf, AML_XFER_TO_HOST, len, CMD_OTHER_CMD, addr, 0, len);
    /* cmd stage */
    ret = auc_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1), g_cmd_buf, sizeof(*g_cmd_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        PRINT_U("%s:%d, Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", __func__, __LINE__, ret, addr, len);
        USB_END_LOCK();
        return ret;
    }

    USB_END_LOCK();
    return 0;
}

unsigned int auc_reg_read(unsigned int addr, unsigned int len)
{
    unsigned int reg_data;
    unsigned int actual_length;
    int ret;
    struct usb_device *udev = g_udev;

#if defined (HAL_FPGA_VER)
    USB_BEGIN_LOCK();
#if 0
    data = (unsigned char *)ZMALLOC(len,"reg tmp",GFP_DMA | GFP_ATOMIC);
    if (!data) {
        AML_PRINT_LOG_ERR("data malloc fail\n");
        return -ENOMEM;
    }
#endif
#ifdef REG_CTRL_EP0
    uint16_t addr_h;
    uint16_t addr_l;

    addr_h = (addr >> 16) & 0xffff;
    addr_l = addr & 0xffff;

    ret = aml_usb_control_msg(udev, usb_rcvctrlpipe(udev, USB_EP0), CMD_READ_REG, USB_CTRL_IN_REQTYPE,
                          addr_h, addr_l,g_kmalloc_buf, len, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret < 0)
    {
        AML_PRINT_LOG_ERR("Failed to usb_control_msg, ret %d\n", ret);

        return ret;
    }
#else


    auc_build_cbw(g_cmd_buf, AML_XFER_TO_HOST, len, CMD_READ_REG, addr, 0, len);

    /* cmd stage */
    ret = auc_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1),(void *)g_cmd_buf, sizeof(*g_cmd_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, addr, len);
        USB_END_LOCK();
        return ret;
    }
    /* data stage */
    ret = auc_bulk_msg(udev, usb_rcvbulkpipe(udev, USB_EP1), (void *)g_kmalloc_buf, len, &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, addr, len);

        USB_END_LOCK();
        return ret;
    }

#endif
    memcpy(&reg_data,g_kmalloc_buf,actual_length);
    USB_END_LOCK();
#elif defined (HAL_SIM_VER)

#ifdef REG_CTRL_EP0
    unsigned int req_buf = addr;
    aml_usb_control_transfer(0, &req_buf, 0, CMD_READ_REG,&reg_data, len);
    PRINT_U("[USB_HOST]reg_read value: %x\n", reg_data);
#else
    crg_msc_request(len, CRG_XFER_TO_HOST, CMD_READ_REG, addr, 0, len, &reg_data);
#endif
#endif

    return reg_data;
}

int auc_reg_write(unsigned int addr, unsigned int value, unsigned int len)
{
#if defined (HAL_FPGA_VER)
    int ret;
    struct usb_device *udev = g_udev;

#ifdef REG_CTRL_EP0
    unsigned int req_buf[2];
    req_buf[0] = addr;
    req_buf[1] = value;

    ret = aml_usb_control_msg(udev, usb_sndctrlpipe(udev, USB_EP0), CMD_WRITE_REG, USB_CTRL_OUT_REQTYPE,
                          0, 0, req_buf, sizeof(req_buf), AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret < 0)
    {
        PRINT_U("%s:%d, Failed to usb_control_msg, ret %d\n", __func__, __LINE__, ret);
        return ret;
    }
#else
    unsigned int actual_length;
    USB_BEGIN_LOCK();
    auc_build_cbw(g_cmd_buf, AML_XFER_TO_DEVICE, 0, CMD_WRITE_REG, addr, value, len);
    /* cmd stage */
    ret = auc_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1),(void *) g_cmd_buf, sizeof(*g_cmd_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, addr, len);
        USB_END_LOCK();
        return ret;
    }
    USB_END_LOCK();
#endif

    return actual_length; //bt write maybe use the value

#elif defined (HAL_SIM_VER)

#ifdef REG_CTRL_EP0
    uint32_t req_buf[2];
    req_buf[0] = addr;
    req_buf[1] = value;
    aml_usb_control_transfer(CMD_WRITE_REG, req_buf, 0x8, 0, 0, NULL);
    return 0;
#else
    return crg_msc_request(0, CRG_XFER_TO_DEVICE, CMD_WRITE_REG, addr, value, len, NULL);
#endif

#endif
}

void _auc_write_sram(unsigned int addr, unsigned char *pdata, unsigned int len)
{
#if defined (HAL_FPGA_VER)
    int ret;
    unsigned int actual_length;

    struct usb_device *udev = g_udev;

    USB_BEGIN_LOCK();
    auc_build_cbw(g_cmd_buf, AML_XFER_TO_DEVICE, len, CMD_WRITE_SRAM, addr, 0, len);
    /* cmd stage */
    ret = auc_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1), (void*)g_cmd_buf, sizeof(*g_cmd_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, addr, len);

        USB_END_LOCK();
        return;
    }
#if 0
    kmalloc_buf = (unsigned char *)ZMALLOC(len, "usb_write_sram", GFP_DMA | GFP_ATOMIC);//virt_to_phys(fwICCM);
    if (kmalloc_buf == NULL)
    {
        AML_PRINT_LOG_ERR("kmalloc buf fail\n");
        FREE(g_cmd_buf,"cmd stage");
        USB_END_LOCK();
        return;
    }
#endif
    if (len > 1024*20) {
        PRINT_U("write sram len is overflow %d\n",len);
        return;
    }
    memcpy(g_kmalloc_buf, pdata, len);
    /* data stage */
    ret = auc_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1), (void *)g_kmalloc_buf, len, &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, addr, len);
        USB_END_LOCK();
        return;
    }
    USB_END_LOCK();

#elif defined (HAL_SIM_VER)
    PRINT_U("[HOST]AML_USB_WRITE_SRAM: addr:0x%x,len:%d\n", addr, len);
    crg_msc_request(len, CRG_XFER_TO_DEVICE, CMD_WRITE_SRAM, addr, (unsigned long)pdata, len, NULL);
#endif
}

void _auc_read_sram(unsigned int addr, unsigned char *pdata, unsigned int len)
{
#if defined (HAL_FPGA_VER)
    int ret;
    unsigned int actual_length;
    struct usb_device *udev = g_udev;

    USB_BEGIN_LOCK();
    auc_build_cbw(g_cmd_buf,  AML_XFER_TO_HOST, len, CMD_READ_SRAM, addr, 0, len);
    /* cmd stage */
    ret = auc_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1), (void *)g_cmd_buf, sizeof(*g_cmd_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, addr, len);
        USB_END_LOCK();
        return;
    }
#if 0
    kmalloc_buf = (unsigned char *)ZMALLOC(len, "usb_read_sram", GFP_DMA|GFP_ATOMIC);
    if (kmalloc_buf == NULL)
    {
        AML_PRINT_LOG_ERR("kmalloc buf fail\n");
        FREE(g_cmd_buf,"cmd stage");
        USB_END_LOCK();
        return;
    }
#endif
    if (len > 1024*20) {
        PRINT_U("Read sram len is overflow %d\n",len);
        return;
    }

    /* data stage */
    ret = auc_bulk_msg(udev, usb_rcvbulkpipe(udev, USB_EP1),(void *)g_kmalloc_buf, len, &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, addr, len);
        USB_END_LOCK();
        return;
    }
    memcpy(pdata, g_kmalloc_buf, actual_length);
    USB_END_LOCK();
#elif defined (HAL_SIM_VER)
    PRINT_U("[HOST]AML_USB_READ_SRAM:addr:0x%x,len:%d\n", addr, len);
    crg_msc_request(len, CRG_XFER_TO_HOST, CMD_READ_SRAM, addr, 0, len, (unsigned int *)pdata);
#endif

}

void auc_write_sram(unsigned char *buf, unsigned char *sram_addr, SYS_TYPE_U len)
{
    unsigned int addr = (unsigned int)(unsigned long)sram_addr;
    _auc_write_sram(addr, buf, len);
}

void auc_write_sram_by_ep_for_bt(unsigned char *buf, unsigned char *sram_addr, unsigned int len, unsigned int ep)
{
    if (USB_EP2 == ep)
    {
        auc_write_sram(buf, sram_addr, len);
    }
    else if (USB_EP1 == ep)
    {
        auc_send_cmd((unsigned int)sram_addr, len);
    }
}

void auc_read_sram(unsigned char *buf,unsigned char *sram_addr, SYS_TYPE_U len)
{
    unsigned int addr = (unsigned int)(unsigned long)sram_addr;
    _auc_read_sram(addr, buf, len);
}

void auc_read_sram_by_ep_for_bt(unsigned char *buf, unsigned char *sram_addr, unsigned int len, unsigned int ep)
{
    auc_read_sram(buf, sram_addr, len);
}

void auc_write_word(unsigned int addr,unsigned int data)
{
    int len = 4;

    auc_reg_write(addr, data, len);
}

void auc_write_word_by_ep_for_bt(unsigned int addr, unsigned int data, unsigned int ep)
{
    auc_write_word(addr, data);
}

unsigned int auc_read_word(unsigned int addr)
{
    int len = 4;

    return auc_reg_read(addr, len);
}

unsigned int auc_read_word_by_ep_for_bt(unsigned int addr, unsigned int ep)
{
    return auc_read_word(addr);
}

void auc_ops_init(void)
{
    struct auc_hif_ops_for_wifi *ops = &g_auc_hif_ops_for_wifi;
    struct auc_hif_ops *ops_for_bt = &g_auc_hif_ops;

    ops->bt_hi_write_sram = auc_write_sram;
    ops->bt_hi_read_sram = auc_read_sram;
    ops->bt_hi_write_word = auc_write_word;
    ops->bt_hi_read_word = auc_read_word;

    ops_for_bt->hi_write_sram_for_bt = auc_write_sram_by_ep_for_bt;
    ops_for_bt->hi_read_sram_for_bt = auc_read_sram_by_ep_for_bt;
    ops_for_bt->hi_write_word_for_bt = auc_write_word_by_ep_for_bt;
    ops_for_bt->hi_read_word_for_bt = auc_read_word_by_ep_for_bt;

    auc_driver_insmoded = 1;
}

#ifdef CHIP_RESET_SUPPORT
struct drv_reset_ops {
    int (*enable_cb)(void);
    void (*disable_cb)(void);
};
extern struct drv_reset_ops g_drv_reset_ops;
#endif
static int auc_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
    PRINT_U("usb plug in!\n");
    g_udev = usb_get_dev(interface_to_usbdev(interface));

    USB_LOCK_INIT();

    g_cmd_buf = ZMALLOC(sizeof(*g_cmd_buf),"cmd stage",GFP_DMA | GFP_ATOMIC);
    if (!g_cmd_buf) {
        PRINT_U("g_cbw_buf malloc fail\n");
        return -ENOMEM;;
    }

    g_kmalloc_buf = (unsigned char *)ZMALLOC(20*1024,"reg tmp",GFP_DMA | GFP_ATOMIC);
    if (!g_kmalloc_buf) {
        AML_PRINT_LOG_ERR("data malloc fail\n");
        return -ENOMEM;
    }
    memset(g_kmalloc_buf,0,1024*20);
    memset(g_cmd_buf,0,sizeof(struct crg_msc_cbw ));

    auc_ops_init();
    auc_driver_probed = 1;

    PRINT_U("%s(%d)\n",__func__,__LINE__);
#ifdef CHIP_RESET_SUPPORT
    wifi_usb_access = 1;
    if (g_drv_reset_ops.enable_cb) {
        return g_drv_reset_ops.enable_cb();
    }
#endif
    return 0;
}

static void auc_disconnect(struct usb_interface *interface)
{
    PRINT_U("usb plug out!\n");
#ifdef CHIP_RESET_SUPPORT
    wifi_usb_access = 0;
    if (g_drv_reset_ops.disable_cb) {
        g_drv_reset_ops.disable_cb();
    }
#endif
    USB_LOCK_DESTROY();
    FREE(g_kmalloc_buf, "usb_read_sram");
    FREE(g_cmd_buf,"cmd stage");
    usb_set_intfdata(interface, NULL);
    usb_put_dev(g_udev);
    auc_driver_probed = 0;
    PRINT_U("--------aml_usb:disconnect-------\n");
}

#ifdef CONFIG_PM
static int auc_suspend(struct usb_interface *interface,pm_message_t state)
{
    return 0;
}

static int auc_resume(struct usb_interface *interface)
{
    return 0;
}
#endif

static int auc_reset_resume(struct usb_interface *intf)
{
    printk("--------W1U usb reset resume-------------- \n");
    return 0;
}

void auc_shutdown(struct device *dev)
{
    // notify fw to enter shutdown mode
      if (g_lp_shutdown_func != NULL)
      {
          g_lp_shutdown_func();
      }

      //notify bt wifi will shutdown
      auc_write_word(RG_AON_A56, auc_read_word(RG_AON_A56)|BIT(31));
}

static const struct usb_device_id auc_devices[] =
{
    {USB_DEVICE(W1u_VENDOR,W1u_PRODUCT)},
    {USB_DEVICE(W1u_VENDOR_AMLOGIC_EFUSE,W1uu_A_PRODUCT_AMLOGIC_EFUSE)},
    {USB_DEVICE(W1u_VENDOR_AMLOGIC_EFUSE,W1uu_B_PRODUCT_AMLOGIC_EFUSE)},
    {USB_DEVICE(W1u_VENDOR_AMLOGIC_EFUSE,W1uu_C_PRODUCT_AMLOGIC_EFUSE)},
    {}
};

MODULE_DEVICE_TABLE(usb, auc_devices);

static struct usb_driver aml_usb_common_driver = {

    .name = "aml_usb_common",
    .id_table = auc_devices,
    .probe = auc_probe,
    .disconnect = auc_disconnect,
#ifdef CONFIG_PM
    .suspend = auc_suspend,
    .resume = auc_resume,
#endif
    .reset_resume = auc_reset_resume,
    .drvwrap.driver.shutdown = auc_shutdown,
};



extern int aml_init_wlan_mem(void);
int aml_usb_insmod(void)
{
    int err = 0;

#ifdef NOT_AMLOGIC_PLATFORM
        err = aml_init_wlan_mem();
        if (err) {
            PRINT_U("aml_init_wlan_mem err: %d \n", err);
            return -ENOMEM;
        }
#endif
    auc_driver_insmoded = 1;
    auc_wifi_in_insmod = 0;
    err = usb_register(&aml_usb_common_driver);
    PRINT_U("%s(%d) aml usb driver insmod\n",__func__, __LINE__);

    if (err) {
        PRINT_U("failed to register usb driver: %d \n", err);
    }

    return err;

}


void aml_usb_rmmod(void)
{
    usb_deregister(&aml_usb_common_driver);
    auc_driver_insmoded = 0;
    PRINT_U("%s(%d) aml usb driver rmsmod\n",__func__, __LINE__);
}

//in order to satisfy bt to be able to recognize the symbol, no use in wifi
uint32_t aml_pci_read_for_bt(int base, u32 offset)
{
    //do nothing
    return 0;
}

void aml_pci_write_for_bt(u32 val, int base, u32 offset)
{
    ;//do nothing
}

EXPORT_SYMBOL(aml_usb_insmod);
EXPORT_SYMBOL(g_cmd_buf);
EXPORT_SYMBOL(g_auc_hif_ops_for_wifi);
EXPORT_SYMBOL(g_auc_hif_ops);
EXPORT_SYMBOL(g_udev);
EXPORT_SYMBOL(auc_driver_insmoded);
EXPORT_SYMBOL(auc_wifi_in_insmod);
EXPORT_SYMBOL(auc_usb_mutex);

EXPORT_SYMBOL(auc_driver_probed);

EXPORT_SYMBOL(bt_wt_ptr);
EXPORT_SYMBOL(bt_rd_ptr);
//in order to satisfy bt to be able to recognize the symbol, no use in wifi
EXPORT_SYMBOL(aml_pci_read_for_bt);
EXPORT_SYMBOL(aml_pci_write_for_bt);
EXPORT_SYMBOL(g_wifi_pm);
//module_init(aml_common_insmod);
//module_exit(aml_common_rmmod);
//MODULE_LICENSE("GPL");
#endif
EXPORT_SYMBOL(wifi_usb_access);
EXPORT_SYMBOL(g_lp_shutdown_func);
EXPORT_SYMBOL(g_bt_shutdown_func);
