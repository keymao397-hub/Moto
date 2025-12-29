#include "aml_usb_common.h"
#include "aml_wifi_bus.h"
#include <linux/delay.h>

#ifdef CONFIG_USB

#define W2_PRODUCT  0x4c55
#define W2_VENDOR  0x414D
#define W2u_VENDOR_AMLOGIC_EFUSE 0x1B8E
#define W2u_PRODUCT_A_AMLOGIC_EFUSE 0x0601
#define W2u_PRODUCT_B_AMLOGIC_EFUSE 0x0641

#define W1u_PRODUCT  0x4c55
#define W1u_VENDOR  0x414D
#define W1u_VENDOR_AMLOGIC_EFUSE  0x1B8E
#define W1uu_C_PRODUCT_AMLOGIC_EFUSE  0x0541
#define W1uu_B_PRODUCT_AMLOGIC_EFUSE  0x0501
#define W1uu_A_PRODUCT_AMLOGIC_EFUSE  0x04C1


struct usb_device *g_usb_dev = NULL;

unsigned int auc_reg_read_wifi_chip(unsigned int addr)
{
    unsigned int reg_data;
    int len = 4;
    int ret;
    unsigned char *data;
    struct usb_device *udev = g_usb_dev;
    unsigned short addr_h;
    unsigned short addr_l;

    data = (unsigned char *)ZMALLOC(len, "reg tmp", GFP_DMA | GFP_ATOMIC);
    if (!data) {
        ERROR_DEBUG_OUT("data malloc fail\n");
        return 0;
    }

    addr_h = (addr >> 16) & 0xffff;
    addr_l = addr & 0xffff;

    ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, USB_EP0), CMD_READ_REG, USB_CTRL_IN_REQTYPE,
                          addr_h, addr_l, data, len, AML_USB_CONTROL_MSG_TIMEOUT);

    if (ret < 0) {
        ERROR_DEBUG_OUT("Failed to usb_control_msg, ret %d\n", ret);
        return 0;
    }

    memcpy(&reg_data,data,len);
    FREE(data,"reg tmp");

    printk("%s: ********get chip_id:0x%08x \n", __func__, reg_data);

    return reg_data;
}

static int usb_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
    unsigned int chip1_id = 0, loop_cnt = 0;

    g_usb_dev = usb_get_dev(interface_to_usbdev(interface));

    PRINT("%s: usb probe idProduct 0x%x\n",__func__, id->idProduct);
    if (id->idProduct == W2u_PRODUCT_A_AMLOGIC_EFUSE
        || id->idProduct == W2u_PRODUCT_B_AMLOGIC_EFUSE) {
        aml_wifi_chip("aml_w2_u");
        aml_chipid_set(WIFI_CHIP_W2_USB);
        printk("%s: ********get chip type: aml_w2_u\n", __func__);
        return 0;
    } else if (id->idProduct == W1uu_A_PRODUCT_AMLOGIC_EFUSE
        || id->idProduct == W1uu_B_PRODUCT_AMLOGIC_EFUSE
        || id->idProduct == W1uu_C_PRODUCT_AMLOGIC_EFUSE) {
        aml_wifi_chip("aml_w1u");
        aml_chipid_set(WIFI_CHIP_W1U);
        printk("%s: ********get chip type: aml_w1u\n", __func__);
        return 0;
    } /* else if (id->idProduct == W2_PRODUCT || id->idProduct == W1u_PRODUCT) */

usb_chip_id:
    msleep(50);
    chip1_id = auc_reg_read_wifi_chip(WIFI_CHIP2_TYPE_ADDR);
    if (chip1_id == WIFI_CHIP_TYPE_W2_USB) {
        aml_wifi_chip("aml_w2_u");
        aml_chipid_set(WIFI_CHIP_W2_USB);
        printk("%s: ********get chip type: aml_w2_u\n", __func__);
    } else if (chip1_id == WIFI_CHIP_TYPE_W1U) {
        aml_wifi_chip("aml_w1u");
        aml_chipid_set(WIFI_CHIP_W1U);
        printk("%s: ********get chip type: aml_w1u\n", __func__);
    } else {
        printk("%s: wifi chip id check failed\n", __func__);
        if (loop_cnt++ < 20)
            goto usb_chip_id;
    }

    PRINT("%s: usb probe success\n",__func__);

    return 0;
}


static void usb_disconnect(struct usb_interface *interface)
{
    usb_set_intfdata(interface, NULL);
    usb_put_dev(g_usb_dev);
    PRINT("--------aml_usb:disconnect-------\n");
}

static const struct usb_device_id wifi_usb_devices[] =
{
    {USB_DEVICE(W2_VENDOR, W2_PRODUCT)},
    {USB_DEVICE(W2u_VENDOR_AMLOGIC_EFUSE, W2u_PRODUCT_A_AMLOGIC_EFUSE)},
    {USB_DEVICE(W2u_VENDOR_AMLOGIC_EFUSE, W2u_PRODUCT_B_AMLOGIC_EFUSE)},
    {USB_DEVICE(W1u_VENDOR, W1u_PRODUCT)},
    {USB_DEVICE(W1u_VENDOR_AMLOGIC_EFUSE, W1uu_A_PRODUCT_AMLOGIC_EFUSE)},
    {USB_DEVICE(W1u_VENDOR_AMLOGIC_EFUSE, W1uu_B_PRODUCT_AMLOGIC_EFUSE)},
    {USB_DEVICE(W1u_VENDOR_AMLOGIC_EFUSE, W1uu_C_PRODUCT_AMLOGIC_EFUSE)},
    {}
};

MODULE_DEVICE_TABLE(usb, wifi_usb_devices);

static struct usb_driver wifi_usb_common_driver = {

    .name = "wifi_usb_common",
    .id_table = wifi_usb_devices,
    .probe = usb_probe,
    .disconnect = usb_disconnect,
};


int aml_wifi_usb_insmod(void)
{
    int err = 0;

    err = usb_register(&wifi_usb_common_driver);
    PRINT("%s(%d) aml common driver insmod\n",__func__, __LINE__);

    return err;
}

void aml_wifi_usb_rmmod(void)
{
    usb_deregister(&wifi_usb_common_driver);
    PRINT("%s(%d) aml common driver rmmod\n",__func__, __LINE__);
}
#endif
