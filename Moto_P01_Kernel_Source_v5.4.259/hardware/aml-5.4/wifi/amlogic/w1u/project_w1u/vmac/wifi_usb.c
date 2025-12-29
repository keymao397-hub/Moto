/*
 ****************************************************************************************
 *
 * Copyright (C) Amlogic 2021-2022
 *
 * Project: 11N 80211 HAL  layer Software
 *
 * Description:
 *   usb interface function,used by HAL write/read usb function
 *
 *
 ****************************************************************************************
 */

#ifdef CONFIG_USB
#ifdef HAL_SIM_VER
#ifdef FW_NAME
namespace FW_NAME
{
#endif
#endif

#include <linux/usb.h>
#include "wifi_hal_com.h"
#include "wifi_hal_platform.h"
#include "wifi_hal.h"
#include "wifi_usb.h"
#include "wifi_mac_com.h"
#include "patch_fi_cmd.h"
#include <linux/usb/hcd.h>
#include <linux/usb.h>
//extern unsigned char buf_iccm_rd[ICCM_BUFFER_RD_LEN];
static unsigned char buf_iccm_rd[128*1024];
struct aml_hwif_usb g_hwif_usb;
struct mutex usb_mutex;
struct crg_msc_cbw *g_cbw_buf = NULL;
struct usb_ctrlrequest  *g_cr = NULL;
struct urb *g_urb;
unsigned char *g_buffer;
extern unsigned char wifi_usb_access;
void aml_usb_build_cbw(struct crg_msc_cbw *cbw_buf,
                               unsigned char dir,
                               unsigned int len,
                               unsigned char cdb1,
                               unsigned int cdb2,
                               unsigned long cdb3,
                               SYS_TYPE cdb4)
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

int aml_usb_bulk_msg(struct usb_device *usb_dev, unsigned int pipe,
    void *data, int len, int *actual_length, int timeout)
{
    int ret;
    if (!wifi_usb_access) {
        AML_PRINT_LOG_ERR("usb bulk disable\n");
        return -ENOMEM;
    }
    ret = usb_bulk_msg(usb_dev, pipe, data, len, actual_length, timeout);
    return ret;
}

int aml_usb_control_msg(struct usb_device *dev, unsigned int pipe, unsigned char request,
    unsigned char  requesttype, unsigned short value, unsigned short index, void *data,
    unsigned short size, int timeout)
{
    int ret;
    if (!wifi_usb_access) {
        AML_PRINT_LOG_ERR("usb control disable\n");
        return -ENOMEM;
    }
    ret = usb_control_msg(dev, pipe, request, requesttype, value, index, data, size, timeout);
    return ret;
}

void usb_stor_control_msg(unsigned long data)
{
    struct hw_interface *hif = hif_get_hw_interface();
    struct usb_device *udev = hif->udev;
    int ret;

    /* fill in the devrequest structure */
    g_cr->bRequestType = USB_CTRL_IN_REQTYPE;
    g_cr->bRequest = CMD_USB_IRQ;
    g_cr->wValue = 0;
    g_cr->wIndex = 0;
    g_cr->wLength = 2 * sizeof(int);

    /*fill a control urb*/
    usb_fill_control_urb(g_urb,
        udev,
        usb_rcvctrlpipe(udev, USB_EP0),
        (unsigned char *)g_cr,
        g_buffer, 2 * sizeof(int),
        (usb_complete_t)aml_usb_ctlread_complete,
        hif);

    /*submit urb*/
    ret = usb_submit_urb(g_urb, GFP_ATOMIC);
    if (ret < 0) {
        AML_PRINT_LOG_ERR("usb_submit_urb failed %d\n", ret);
    }
}

unsigned int reg_read(unsigned int addr, unsigned int len)
{
    unsigned int reg_data;
    unsigned char *data;
    unsigned int actual_length;
    int ret;
    struct hw_interface *hif = hif_get_hw_interface();
    struct usb_device *udev = hif->udev;
#if defined (HAL_FPGA_VER)
    data = (unsigned char *)ZMALLOC(len,"reg tmp",GFP_DMA | GFP_ATOMIC);
    if (!data) {
        AML_PRINT_LOG_ERR("data malloc fail\n");
        return -ENOMEM;
    }
#ifdef REG_CTRL_EP0
    uint16_t addr_h;
    uint16_t addr_l;

    addr_h = (addr >> 16) & 0xffff;
    addr_l = addr & 0xffff;

    ret = aml_usb_control_msg(udev, usb_rcvctrlpipe(udev, USB_EP0), CMD_READ_REG, USB_CTRL_IN_REQTYPE,
                          addr_h, addr_l,data, len, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret < 0)
    {
        AML_PRINT_LOG_ERR("Failed to usb_control_msg, ret %d\n", ret);
        FREE(data,"reg tmp");
        return ret;
    }
#else

    USB_BEGIN_LOCK();
    aml_usb_build_cbw(g_cbw_buf, AML_XFER_TO_HOST, len, CMD_READ_REG, addr, 0, len);

    /* cmd stage */
    ret = aml_usb_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1),(void *)g_cbw_buf, sizeof(*g_cbw_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, addr, len);
        FREE(data,"reg tmp");
        USB_END_LOCK();
        return ret;
    }
    /* data stage */
    ret = aml_usb_bulk_msg(udev, usb_rcvbulkpipe(udev, USB_EP1), (void *)data, len, &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, addr, len);
        FREE(data,"reg tmp");
        USB_END_LOCK();
        return ret;
    }
    USB_END_LOCK();
#endif
    memcpy(&reg_data,data,len);
    FREE(data,"reg tmp");

#elif defined (HAL_SIM_VER)

#ifdef REG_CTRL_EP0
    unsigned int req_buf = addr;
    aml_usb_control_transfer(0, &req_buf, 0, CMD_READ_REG,&reg_data, len);
    PRINT("[USB_HOST]reg_read value: %x\n", reg_data);
#else
    crg_msc_request(len, CRG_XFER_TO_HOST, CMD_READ_REG, addr, 0, len, &reg_data);
#endif
#endif
    return reg_data;
}

int reg_write(unsigned int addr, unsigned int value, unsigned int len)
{
#if defined (HAL_FPGA_VER)
    int ret;
    struct hw_interface *hif = hif_get_hw_interface();
    struct usb_device *udev = hif->udev;

#ifdef REG_CTRL_EP0
    unsigned int req_buf[2];
    req_buf[0] = addr;
    req_buf[1] = value;

    ret = aml_usb_control_msg(udev, usb_sndctrlpipe(udev, USB_EP0), CMD_WRITE_REG, USB_CTRL_OUT_REQTYPE,
                          0, 0, req_buf, sizeof(req_buf), AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret < 0)
    {
        AML_PRINT_LOG_INFO("Failed to usb_control_msg, ret %d\n", ret);
        return ret;
    }
#else
    unsigned int actual_length;
    USB_BEGIN_LOCK();
    aml_usb_build_cbw(g_cbw_buf, AML_XFER_TO_DEVICE, 0, CMD_WRITE_REG, addr, value, len);
    /* cmd stage */
    ret = aml_usb_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1),(void *) g_cbw_buf, sizeof(*g_cbw_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
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

unsigned int efuse_write(unsigned int addr, unsigned int efuse_data)
{
    int ret;
    unsigned int efuse_data_len = 4;
    struct hw_interface *hif = hif_get_hw_interface();
    struct usb_device *udev = hif->udev;
    unsigned int actual_length;

    USB_BEGIN_LOCK();
    aml_usb_build_cbw(g_cbw_buf, AML_XFER_TO_DEVICE, 0, CMD_WRITE_EFUSE, addr, efuse_data, efuse_data_len);
    /* cmd stage */
    ret = aml_usb_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1),(void *) g_cbw_buf, sizeof(*g_cbw_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, data: %d\n", ret, addr, efuse_data);
        USB_END_LOCK();
        return ret;
    }
    USB_END_LOCK();

    AML_PRINT_LOG_INFO("[WRITE] efuse[%x]:%08x",addr,efuse_data);

    return actual_length;
}

unsigned int efuse_read(unsigned int addr, unsigned int len)
{
    unsigned int reg_data = 0;
    unsigned char *data = 0;
    unsigned int actual_length = 0;
    int ret;
    struct hw_interface *hif = hif_get_hw_interface();
    struct usb_device *udev = hif->udev;
    data = (unsigned char *)ZMALLOC(len,"reg tmp",GFP_DMA | GFP_ATOMIC);
    if (!data) {
        AML_PRINT_LOG_ERR("data malloc fail\n");
        return -ENOMEM;
    }

    USB_BEGIN_LOCK();
    aml_usb_build_cbw(g_cbw_buf, AML_XFER_TO_HOST, len, CMD_READ_EFUSE, addr, 0, len);

    /* cmd stage */
    ret = aml_usb_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1),(void *)g_cbw_buf, sizeof(*g_cbw_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, addr, len);
        FREE(data,"reg tmp");
        USB_END_LOCK();
        return ret;
    }

    /* data stage */
    ret = aml_usb_bulk_msg(udev, usb_rcvbulkpipe(udev, USB_EP1), (void *)data, len, &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, addr, len);
        FREE(data,"reg tmp");
        USB_END_LOCK();
        return ret;
    }
    USB_END_LOCK();

    memcpy(&reg_data,data,len);
    FREE(data,"reg tmp");
    return reg_data;
}

unsigned long aml_usb_read_byte(unsigned long addr)
{
    int len = 1;

    return reg_read((unsigned int)addr, len);
}

void aml_usb_write_byte(unsigned long addr, unsigned long data)
{
    int len = 1;

    reg_write((unsigned int)addr, data, len);
}

unsigned int aml_usb_read_word(unsigned int addr)
{
    int len = 4;

    return reg_read(addr, len);
}

void aml_usb_write_word(unsigned int addr,unsigned int data)
{
    int len = 4;

    reg_write(addr, data, len);
}

unsigned int aml_usb_read_efuse(unsigned int addr)
{
    int len = 4;

    return efuse_read(addr, len);
}

void usb_write_sram(unsigned int addr, unsigned char *pdata, unsigned int len)
{
#if defined (HAL_FPGA_VER)
    int ret;
    unsigned int actual_length;
    void *kmalloc_buf;
    struct hw_interface *hif = hif_get_hw_interface();
    struct usb_device *udev = hif->udev;


    USB_BEGIN_LOCK();
    aml_usb_build_cbw(g_cbw_buf, AML_XFER_TO_DEVICE, len, CMD_WRITE_SRAM, addr, 0, len);
    /* cmd stage */
    ret = aml_usb_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1), (void*)g_cbw_buf, sizeof(*g_cbw_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, addr, len);
        USB_END_LOCK();
        return;
    }

    kmalloc_buf = (unsigned char *)ZMALLOC(len, "usb_write_sram", GFP_DMA | GFP_ATOMIC);//virt_to_phys(fwICCM);
    if (kmalloc_buf == NULL)
    {
        AML_PRINT_LOG_ERR("kmalloc buf fail\n");
        USB_END_LOCK();
        return;
    }
    memcpy(kmalloc_buf, pdata, len);
    /* data stage */
    ret = aml_usb_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1), (void *)kmalloc_buf, len, &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, addr, len);
        FREE(kmalloc_buf, "usb_write_sram");
        USB_END_LOCK();
        return;
    }
    USB_END_LOCK();
    FREE(kmalloc_buf, "usb_write_sram");

#elif defined (HAL_SIM_VER)
    PRINT("[HOST]AML_USB_WRITE_SRAM: addr:0x%x,len:%d\n", addr, len);
    crg_msc_request(len, CRG_XFER_TO_DEVICE, CMD_WRITE_SRAM, addr, (unsigned long)pdata, len, NULL);
#endif
}

void usb_read_sram(unsigned int addr, unsigned char *pdata, unsigned int len)
{
#if defined (HAL_FPGA_VER)
    int ret;
    unsigned int actual_length;
    void *kmalloc_buf;
    struct hw_interface* hif = hif_get_hw_interface();
    struct usb_device *udev = hif->udev;


    USB_BEGIN_LOCK();
    aml_usb_build_cbw(g_cbw_buf,  AML_XFER_TO_HOST, len, CMD_READ_SRAM, addr, 0, len);
    /* cmd stage */
    ret = aml_usb_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1), (void *)g_cbw_buf, sizeof(*g_cbw_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, addr, len);
        USB_END_LOCK();
        return;
    }

    kmalloc_buf = (unsigned char *)ZMALLOC(len, "usb_read_sram", GFP_DMA|GFP_ATOMIC);
    if (kmalloc_buf == NULL)
    {
        AML_PRINT_LOG_ERR("kmalloc buf fail\n");
        USB_END_LOCK();
        return;
    }
    /* data stage */
    ret = aml_usb_bulk_msg(udev, usb_rcvbulkpipe(udev, USB_EP1),(void *)kmalloc_buf, len, &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, addr, len);
        FREE(kmalloc_buf, "usb_read_sram");
        USB_END_LOCK();
        return;
    }
    USB_END_LOCK();
    memcpy(pdata, kmalloc_buf, len);
    FREE(kmalloc_buf, "usb_read_sram");
#elif defined (HAL_SIM_VER)
    AML_PRINT_LOG_INFO("[HOST]AML_USB_READ_SRAM:addr:0x%x,len:%d\n", addr, len);
    crg_msc_request(len, CRG_XFER_TO_HOST, CMD_READ_SRAM, addr, 0, len, (unsigned int *)pdata);
#endif
}

void aml_usb_write_sram(unsigned char *buf, unsigned char *sram_addr, SYS_TYPE len)
{
    unsigned int addr = (unsigned int)(unsigned long)sram_addr;
    usb_write_sram(addr, buf, len);
}
void aml_usb_read_sram(unsigned char *buf,unsigned char *sram_addr, SYS_TYPE len)
{
    unsigned int addr = (unsigned int)(unsigned long)sram_addr;
    usb_read_sram(addr, buf, len);
}

int wifi_iccm_download(unsigned char *src, unsigned int len)
{
    struct hw_interface* hif = hif_get_hw_interface();
#ifdef ICCM_ROM
    unsigned int base_addr = MAC_ICCM_AHB_BASE + ICCM_ROM_LEN;
#else
    unsigned int base_addr = MAC_ICCM_AHB_BASE;
#endif
    unsigned int offset = 0;
    unsigned int trans_len = len;

#if defined (HAL_FPGA_VER)
    int ret;
    unsigned int actual_length;
    struct usb_device *udev = hif->udev;

    USB_BEGIN_LOCK();
    aml_usb_build_cbw(g_cbw_buf, AML_XFER_TO_DEVICE, len, CMD_DOWNLOAD_WIFI, base_addr, 0, len);

    /* cmd stage */
    ret = aml_usb_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1), (void *)g_cbw_buf, sizeof(*g_cbw_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, base_addr, len);
        USB_END_LOCK();
        return 1;
    }

    while (offset < len) {
        if ((len - offset) > USB_MAX_TRANS_SIZE) {
            trans_len = USB_MAX_TRANS_SIZE;
        } else {
            trans_len = len - offset;
        }

        /* data stage */
        ret = aml_usb_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1), (void*)src+offset, trans_len, &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
        if (ret) {
            AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, base_addr, len);
            FREE(g_cbw_buf,"cmd stage");
            USB_END_LOCK();
            return 1;
        }

        AML_PRINT_LOG_INFO("wifi_iccm_download actual_length = 0x%x\n", actual_length);
        offset += actual_length;
    }

    USB_END_LOCK();

#ifdef ICCM_CHECK
    PRINT("start iccm check \n");
    offset = 0;
    len = ICCM_CHECK_LEN;
    do {
        SYS_TYPE databyte = len;
        /* NOTE:
         * if the len is not equal to USB_MAX_TRANS_SIZE,
         * need to open the following statement to check databyte
         */
        //databyte = (len > USB_MAX_TRANS_SIZE) ? USB_MAX_TRANS_SIZE : len;

        hif->hif_ops.hi_read_sram(buf_iccm_rd + offset,
                        (unsigned char*)(SYS_TYPE)(base_addr + offset), databyte);

        PRINT("read offset 0x%x,len 0x%lx \n",offset, databyte);
        offset += databyte;
        len -= databyte;
    } while(len > 0);

    if (memcmp(buf_iccm_rd, src, ICCM_CHECK_LEN)) {
        AML_PRINT_LOG_INFO("Host HAL: write ICCM ERROR!!!! \n");
    } else {
        AML_PRINT_LOG_INFO("Host HAL: write ICCM SUCCESS!!!! \n");
    }

    AML_PRINT_LOG_INFO("stop iccm check \n");
#endif

#elif defined (HAL_SIM_VER)
    PRINT("iccm_downld, addr 0x%x, len %d \n", base_addr, trans_len);
    while (offset < len)
    {
        if (offset + trans_len > len)
            trans_len = len - offset;
        PRINT("iccm_downld, addr+offset 0x%x, actual_len %d \n", base_addr + offset, trans_len);
        crg_msc_request(trans_len, CRG_XFER_TO_DEVICE, CMD_DOWNLOAD_WIFI,
            base_addr + offset/*dest*/, (unsigned long)src, trans_len, NULL);
        offset += trans_len;
    }
#endif

    return 0;
}

int wifi_dccm_download(unsigned char *src, unsigned int len)
{
    unsigned int base_addr = 0x00d00000;
    unsigned int offset = 0;
    unsigned int trans_len = len;

#if defined (HAL_FPGA_VER)
    int ret;
    unsigned int actual_length;
    struct hw_interface* hif = hif_get_hw_interface();
    struct usb_device *udev = hif->udev;

    AML_PRINT_LOG_INFO("dccm_downed, addr 0x%x, len %d \n", src, len);
    aml_usb_build_cbw(g_cbw_buf, AML_XFER_TO_DEVICE, len, CMD_DOWNLOAD_WIFI, base_addr, 0, len);
    USB_BEGIN_LOCK();
    /* cmd stage */
    ret = aml_usb_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1), (void*)g_cbw_buf,sizeof(*g_cbw_buf),&actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, base_addr, len);
        FREE(g_cbw_buf,"cmd stage");
        USB_END_LOCK();
        return 1;
    }

    while (offset < len) {
        if ((len - offset) > USB_MAX_TRANS_SIZE) {
            trans_len = USB_MAX_TRANS_SIZE;
        } else {
            trans_len = len - offset;
        }

        /* data stage */
        ret = aml_usb_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1),(void *)src+offset, trans_len, &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
        if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, base_addr, len);
            FREE(g_cbw_buf,"cmd stage");
            USB_END_LOCK();
            return 1;
        }

        AML_PRINT_LOG_INFO("wifi_dccm_download actual_length = 0x%x\n", actual_length);
        offset += actual_length;
    }

    USB_END_LOCK();
#elif defined (HAL_SIM_VER)
    PRINT("dccm_downed, addr 0x%x, len %d \n", addr, len);
    while (offset < len) {
        if (offset + trans_len > len)
            trans_len = len - offset;
        PRINT("dccm_downed, addr+offset 0x%x, actual_len %d \n", base_addr + offset, trans_len);
        crg_msc_request(trans_len, CRG_XFER_TO_DEVICE, CMD_DOWNLOAD_WIFI,
            base_addr + offset/*dest*/, (unsigned long)addr/*src*/, trans_len, NULL);
        offset += trans_len;
    }
#endif

    return 0;
}

int wifi_fw_download(void)
{
    const struct firmware *fw;
    struct device *dev = vm_cfg80211_get_parent_dev();
    unsigned char *src;
    int err = 0;
    unsigned int len = 0;
    void *kmalloc_buf = NULL;

    err = request_firmware(&fw, WIFI_FW_NAME, dev);
    if (err) {
        return err;
    }

    src = (unsigned char *)fw->data;
    if (fw->size < ICCM_RAM_LEN + DCCM_LEN) {
        AML_PRINT_LOG_INFO("fw size:0x%x is too short!\n", fw->size);
        release_firmware(fw);
        err = 2;
        return err;
    }

#ifdef ICCM_ROM
    /*
     * skip iccm rom code, 0 to ICCM_ROM_LEN-1 for iccm rom,
     * ICCM_ROM_LEN to ICCM_RAM_LEN-1 for iccm ram.
     */
    AML_PRINT_LOG_INFO("start download iccm ram\n");
#else
    AML_PRINT_LOG_INFO("start download iccm rom and ram\n");
#endif

#if defined (HAL_FPGA_VER)
    len = ICCM_RAM_LEN;
    PRINT("Sram size 0x%x\n",SRAM_LEN);
    kmalloc_buf = (unsigned char *)ZMALLOC(len, "usb_write", GFP_DMA | GFP_ATOMIC);//virt_to_phys(fwICCM);
    if (kmalloc_buf == NULL) {
        AML_PRINT_LOG_ERR("kmalloc buf fail\n");
        release_firmware(fw);
        err = 1;
        return err;
    }

    memcpy(kmalloc_buf, src, len);
    wifi_iccm_download(kmalloc_buf, len);

    src = (unsigned char *)fw->data + ICCM_RAM_LEN;
    len = ALIGN(DCCM_LEN, 4) - (6 * 1024/*stack*/ + 2 * 1024/*usb data*/);
    memset(kmalloc_buf, 0, len);
    memcpy(kmalloc_buf, src, len);
    wifi_dccm_download(kmalloc_buf, len);

    FREE(kmalloc_buf, "usb_write");
#elif defined (HAL_SIM_VER)
    AML_PRINT_LOG_INFO("len %d, addr 0x%x\n", len, src);
    len = ICCM_ALL_LEN;

    wifi_iccm_download(src, len);
    src = (unsigned char *)fw->data + ICCM_ALL_LEN;

    len = ALIGN(DCCM_LEN, 4) - (6 * 1024/*stack*/ + 2 * 1024/*usb data*/);
    wifi_dccm_download(src, len);
#endif
    release_firmware(fw);
    return err;
}

int start_wifi(void)
{
#ifdef HAL_FPGA_VER
    int ret;
    unsigned int actual_length;
    struct hw_interface* hif = hif_get_hw_interface();
    struct usb_device *udev = hif->udev;

#if 0 //kun
    bbpll_init();
    bbpll_start();
    hal_set_sys_clk_for_fpga();
#endif
    wifi_cpu_clk_switch(0x4f770033);

    aml_usb_build_cbw(g_cbw_buf, AML_XFER_TO_DEVICE, 0, CMD_START_WIFI, 0, 0, 0);
    USB_BEGIN_LOCK();
    /* cmd stage */
    ret = aml_usb_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1),(void *) g_cbw_buf, sizeof(*g_cbw_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    USB_END_LOCK();
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d\n", ret);
        return 1;
    }

    return 0;
#elif defined (HAL_SIM_VER)
    return crg_msc_request(0, CRG_XFER_TO_DEVICE, CMD_START_WIFI, 0, 0, 0, NULL);
#endif
}

int stop_wifi(void)
{
#ifdef HAL_FPGA_VER
    int ret;
    unsigned int actual_length;
    struct hw_interface* hif = hif_get_hw_interface();
    struct usb_device *udev = hif->udev;


    USB_BEGIN_LOCK();
    aml_usb_build_cbw(g_cbw_buf, AML_XFER_TO_DEVICE, 0, CMD_STOP_WIFI, 0, 0, 0);
    /* cmd stage */
    ret = aml_usb_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1), (void *)g_cbw_buf, sizeof(*g_cbw_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    USB_BEGIN_LOCK();
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d\n", ret);
        return 1;
    }

    return 0;
#elif defined (HAL_SIM_VER)
    return crg_msc_request(0, CRG_XFER_TO_DEVICE, CMD_STOP_WIFI, 0, 0, 0, NULL);
#endif
}

unsigned long aml_usb_read_reg32(unsigned long sram_addr)
{
    PRINT("##################### ERROR, %s should be replaced ##########################\n", __func__);
    return 0;
}

int aml_usb_write_reg32(unsigned long sram_addr, unsigned long sramdata)
{
    aml_usb_write_word((unsigned int)sram_addr, (unsigned int)sramdata);
    return 0;
}

struct aml_hwif_usb *aml_usb_priv(void)
{
    return &g_hwif_usb;
}

int aml_usb_enable_scatter(void)
{
    struct aml_hwif_usb *hif_usb = aml_usb_priv();
    struct amlw_hif_scatter_req *scat_req = NULL;

    ASSERT(hif_usb != NULL);

    if (hif_usb->scatter_enabled) {
        return 0;
    }

    hif_usb->scatter_enabled = true;

    /* allocate the scatter request */
    scat_req = ZMALLOC(sizeof(struct amlw_hif_scatter_req), "usb_alloc_prep_scat_req", GFP_ATOMIC|GFP_DMA);
    if (scat_req == NULL)
    {
        AML_PRINT_LOG_ERR("[usb sg alloc_scat_req]: no mem\n");
        return 1;
    }

    scat_req->free = true;
    hif_usb->scat_req = scat_req;

    return 0;

}

void aml_usb_cleanup_scatter(void)
{
    struct aml_hwif_usb *hif_usb = aml_usb_priv();
    PRINT("[usb sg cleanup]: enter\n");

    ASSERT(hif_usb != NULL);

    if (!hif_usb->scatter_enabled)
        return;

    hif_usb->scatter_enabled = false;

    /* empty the free list */
     FREE(hif_usb->scat_req, "usb_alloc_prep_scat_req");

    PRINT("[usb sg cleanup]: exit\n");

    return;
}

struct amlw_hif_scatter_req *aml_usb_scatter_req_get(void)
{
    struct aml_hwif_usb *hif_usb = aml_usb_priv();
    struct amlw_hif_scatter_req *scat_req = NULL;

    ASSERT(hif_usb != NULL);

    scat_req = hif_usb->scat_req;

    if (scat_req->free)
    {
        scat_req->free = false;
    }
    else if (scat_req->scat_count != 0) // get scat_req, but not build scatter list
    {
        scat_req = NULL;
    }

    return scat_req;
}

void aml_usb_scat_complete (struct amlw_hif_scatter_req * scat_req)
{
    int  i;
    struct hw_interface * hif = hif_get_hw_interface();
    struct aml_hwif_usb *hif_usb = aml_usb_priv();

    ASSERT(scat_req != NULL);
    ASSERT(hif != NULL);
    ASSERT(hif_usb != NULL);

    if (scat_req->complete)
    {
        for (i = 0; i < scat_req->scat_count; i++)
        {
            (scat_req->complete)(scat_req->scat_list[i].skbbuf);
            scat_req->scat_list[i].skbbuf = NULL;
        }
    }
    else
    {
        AML_PRINT_LOG_ERR("error: no complete function\n");
    }

    scat_req->free = true;
    scat_req->scat_count = 0;
    scat_req->len = 0;
    scat_req->addr = 0;
    memset(scat_req->sgentries, 0, MAX_SG_ENTRIES * sizeof(struct scatterlist));
    memset(scat_req->bufferinfo, 0, MAX_SG_ENTRIES * sizeof(struct HW_TxBufferInfo));
}

void usb_recv_frame(unsigned int addr, unsigned char *pdata, unsigned int len)
{
    int ret;
    unsigned int actual_length;
    struct hw_interface* hif = hif_get_hw_interface();
    struct usb_device *udev = hif->udev;

    USB_BEGIN_LOCK();

    aml_usb_build_cbw(g_cbw_buf,  AML_XFER_TO_HOST, len, CMD_READ_SRAM, addr, 0, len);

    /* cmd stage */
    ret = aml_usb_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1), (void *)g_cbw_buf,sizeof(*g_cbw_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, addr, len);
        USB_END_LOCK();
        return;
    }

    /* data stage */
    ret = aml_usb_bulk_msg(udev, usb_rcvbulkpipe(udev, USB_EP1), pdata, len, &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, addr: 0x%x, len: %d\n", ret, addr, len);
        USB_END_LOCK();
        return;
    }

    USB_END_LOCK();
}


void aml_usb_recv_frame(unsigned char *buf, unsigned char *addr, SYS_TYPE len)
{
    struct hw_interface* hif = hif_get_hw_interface();
    unsigned int rx_end = hif->hw_config.rxframeaddress + hif->hw_config.rxpagenum * hif->CommStaticParam.tx_page_len;

    if ((unsigned int)(unsigned long)(addr + len) > rx_end)
    {
        unsigned int remain_len = rx_end - (unsigned int)(unsigned long)addr;

        usb_recv_frame((unsigned int)(unsigned long)addr, buf, remain_len);
        usb_recv_frame(hif->hw_config.rxframeaddress, buf+remain_len, len - remain_len);
    }
    else
    {
        usb_recv_frame((unsigned int)(unsigned long)addr, buf, len);
    }
    return;
}


//#ifdef HOST_USB

void aml_show_txpage(void *pTxDPape, int len)
{
    int i;

    unsigned char * buf = (unsigned char *)pTxDPape;
    PRINT("Tx_Frame:: 0x%x\n", len);
    PRINT("**********************************************************************\n");
    for (i = 0; i < len; i++)
    {
        if (i % 16 == 0 && i !=0)
        {
            PRINT("\n");
        }
        PRINT("%02x ", *(buf++));
    }
    PRINT("\n");
    PRINT("*********************************************************************\n");
}

#if defined (HAL_FPGA_VER)
void aml_usb_build_tx_packet_info(struct crg_msc_cbw *cbw_buf, unsigned char cdb1,
    struct tx_trb_info_ex * trb_info)
{
    unsigned char remain_pkt_num = trb_info->packet_num - 14;

    cbw_buf->sig = trb_info->buffer_size[0] | trb_info->buffer_size[1] << 16;
    cbw_buf->tag = trb_info->buffer_size[2] | trb_info->buffer_size[3] << 16;
    cbw_buf->data_len = trb_info->buffer_size[4] | trb_info->buffer_size[5] << 16;
    cbw_buf->flag = trb_info->packet_num; //packet nums 1byte
    cbw_buf->len = trb_info->buffer_size[13] & 0xff;
    cbw_buf->lun = (trb_info->buffer_size[13] >> 8) & 0xff;

    cbw_buf->cdb[0] = cdb1 | trb_info->buffer_size[12] << 16;
    cbw_buf->cdb[1] = trb_info->buffer_size[6] | trb_info->buffer_size[7] << 16;
    cbw_buf->cdb[2] = trb_info->buffer_size[8] | trb_info->buffer_size[9] << 16;
    cbw_buf->cdb[3] = trb_info->buffer_size[10] | trb_info->buffer_size[11] << 16;

    if (trb_info->packet_num > 14)
    {
        unsigned char i = 0;
        unsigned short *p_buf = cbw_buf->buf;
        while (remain_pkt_num--)
        {
            *p_buf++ = trb_info->buffer_size[i++ + 14];
        }
    }
}

int aml_usb_send_frame(struct amlw_hif_scatter_req * pframe)
{
    int ret;
    int i;
    unsigned int actual_length;
    struct hw_interface* hif = hif_get_hw_interface();
    struct usb_device *udev = hif->udev;

    memset(&pframe->page, 0, sizeof(struct tx_trb_info_ex));

    USB_BEGIN_LOCK();
    /* build page_info array */
    pframe->page.packet_num = pframe->scat_count;
    for (i = 0; i < pframe->scat_count; i++)
    {
        pframe->page.buffer_size[i] = pframe->scat_list[i].len;
    }
    aml_usb_build_tx_packet_info(g_cbw_buf, CMD_WRITE_PACKET, &(pframe->page));
    /* cmd stage */
    ret = aml_usb_bulk_msg(udev, usb_sndbulkpipe(udev, USB_EP1),
        g_cbw_buf, sizeof(*g_cbw_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        AML_PRINT_LOG_ERR("Failed to usb_bulk_msg, ret %d, actual_len: %d\n", ret, actual_length);
        USB_END_LOCK();
        return 1;
    }
    aml_usb_send_packet(pframe);
    aml_usb_scat_complete(pframe);
    USB_END_LOCK();
    return 0;
}


#elif defined (HAL_SIM_VER) && defined (HOST_USB)

/* for simulation ver . */
int hi_tx_frame(struct hi_tx_desc * pTxDPage, unsigned int blk_num, unsigned int page_num)
{
    unsigned char *pdata = (unsigned char *)pTxDPage;

    crg_msc_request(blk_num, CRG_XFER_TO_DEVICE, CMD_WRITE_PACKET, 0, (unsigned long)pdata, page_num, NULL);
}
#endif

int hal_tx_page_build(struct hi_tx_desc * pTxDPage)
{
    unsigned int mpdu_len = HW_MPDU_LEN_GET(pTxDPage->MPDUBufFlag);
    struct hw_interface *hif = hif_get_hw_interface();
    unsigned int total_len = 0, page_index = 0, page_num = 0, page_len = 0;
    struct OtherTxPage *other_page = NULL;
    unsigned int data_len = 0, remain_len = 0;

    remain_len = mpdu_len + HI_TXDESC_DATAOFFSET;
    total_len = remain_len + hal_calc_block_in_mpdu(remain_len) * 12 - 4/* bufferflag in first page */;
    page_num = hal_calc_block_in_mpdu(total_len);
    page_len = hif->CommStaticParam.tx_page_len;
    if (page_num == 1)
    {
        memmove((void *)((unsigned char *)&(pTxDPage->TxVector) + 8), (void *)&(pTxDPage->TxVector), (remain_len - 4));
        memset((void *)&(pTxDPage->TxVector), 0, 8);
        return page_num;
    }

    for (page_index = 0; page_index < page_num; page_index++)
    {
        /* first page */
        if (page_index == 0)
        {
            unsigned int mpdubufflag = 0;

            data_len = page_len - (sizeof(struct TxDescPage) - 1/* txdata[1] */);

            // add 8 bytes
            memmove((void *)&(pTxDPage->TxVector) + 8, (void *)&(pTxDPage->TxVector), (remain_len - 4));
            //os_skb_push(8);//for chip
            remain_len -= page_len - 8;

            pTxDPage->MPDUBufFlag &= (HW_FIRST_AGG_FLAG | HW_LAST_AGG_FLAG);
            mpdubufflag = pTxDPage->MPDUBufFlag;

            memset((void *)pTxDPage, 0, sizeof(struct HW_TxBufferInfo));

            pTxDPage->MPDUBufFlag = mpdubufflag;
            pTxDPage->MPDUBufFlag |= HW_FIRST_MPDUBUF_FLAG;
            pTxDPage->MPDUBufFlag |= HW_MPDU_LEN_SET(mpdu_len);
            pTxDPage->MPDUBufFlag |= HW_BUFFER_LEN_SET(data_len);
            other_page = (struct OtherTxPage *)(((unsigned char *)pTxDPage) + page_len);
        }
        /* last page */
        else if (page_index == page_num - 1)
        {
            other_page->BufferInfo.MPDUBufFlag = pTxDPage->MPDUBufFlag;
            other_page->BufferInfo.MPDUBufFlag &= ~(HW_FIRST_MPDUBUF_FLAG | HW_LAST_MPDUBUF_FLAG);
            other_page->BufferInfo.MPDUBufFlag |= HW_LAST_MPDUBUF_FLAG;

            /* buffer len is 0 for last page */
            other_page->BufferInfo.MPDUBufFlag &= 0xfffff;
            break;
        }
        else
        {
            data_len = page_len - sizeof(struct HW_TxBufferInfo);
            if (remain_len < data_len)
                PRINT("%s, short len error\n", __func__);
            remain_len -= data_len;
            other_page->BufferInfo.MPDUBufFlag = pTxDPage->MPDUBufFlag;
            other_page->BufferInfo.MPDUBufFlag &= ~(HW_FIRST_MPDUBUF_FLAG | HW_LAST_MPDUBUF_FLAG);

            other_page->BufferInfo.MPDUBufFlag &= 0xfffff;
            other_page->BufferInfo.MPDUBufFlag |= HW_BUFFER_LEN_SET(data_len);

            other_page = (struct OtherTxPage *)(((unsigned char *)other_page) + page_len);
        }
        memmove((void *)other_page->txdata, (void *)other_page, remain_len);
        memset((void *)other_page, 0, sizeof(struct HW_TxBufferInfo));
    }
    return page_num;
}
struct usb_hub *aml_usb_hub_to_struct_hub(struct usb_device *hdev)
{
    if (!hdev || !hdev->actconfig || !hdev->maxchild)
        return NULL;
    return usb_get_intfdata(hdev->actconfig->interface[0]);
}
int aml_set_port_feature(struct usb_device *hdev, int port, int feature)
{
    return usb_control_msg(hdev, usb_sndctrlpipe(hdev, 0),
        USB_REQ_SET_FEATURE, USB_RT_PORT, feature, port,
        NULL, 0, 1000);
}
int aml_clear_port_feature(struct usb_device *hdev, int port, int feature)
{
    return usb_control_msg(hdev, usb_sndctrlpipe(hdev, 0),
        USB_REQ_CLEAR_FEATURE, USB_RT_PORT, feature, port,
        NULL, 0, 1000);
}

int aml_usb_port_suspend(struct usb_device *udev)
{
    int status;
    int port = udev->portnum;
    struct usb_hub *hub = (struct usb_hub *)aml_usb_hub_to_struct_hub(udev->parent);

    status = aml_set_port_feature(hub->hdev, port, USB_PORT_FEAT_SUSPEND);

    /* device has up to 10 msec to fully suspend */
    //usb_set_device_state(udev, USB_STATE_SUSPENDED);
    msleep(10);

    return status;
}

int aml_usb_port_resume(struct usb_device *udev)
{
    int  status;
    int  port = udev->portnum;
    struct usb_hub *hub = (struct usb_hub *)aml_usb_hub_to_struct_hub(udev->parent);

    status = aml_clear_port_feature(hub->hdev, port, USB_PORT_FEAT_SUSPEND);
    msleep(20);

    return status;
}

int aml_usb_send_packet(struct amlw_hif_scatter_req * scat_req)
{
    struct hw_interface *hif = hif_get_hw_interface();
    struct usb_device *udev = hif->udev;
    struct scatterlist *sg;
    struct usb_sg_request sgr;
    int sg_count, sgitem_count;
    unsigned int max_req_size;
    int ttl_len, pkt_offset, page_num;
    struct HW_TxBufferInfo *PbufferInfo;
    unsigned int data_len;
    unsigned int mpdubufflag = 0;
    unsigned int mpdu_len;
    unsigned int last_page_size ;
    unsigned int ttl_page_num = 0;
    int ret;
    int i;

    /* fill SG entries */
    sg = scat_req->sgentries;
    pkt_offset = 0; // reminder
    sgitem_count = 0; // count of scatterlist
    max_req_size = USB_MAX_TRANS_SIZE;
    udev->bus->sg_tablesize = MAXSG_SIZE;

    while (sgitem_count < scat_req->scat_count)
    {
        ttl_len = 0;
        sg_count = 0;
        page_num = 0;
        sg_init_table(sg, MAXSG_SIZE);

        /* assemble SG list */
        while (sgitem_count < scat_req->scat_count)
        {
            int packet_len = 0;
            unsigned char *pdata = NULL;
            packet_len = scat_req->scat_list[sgitem_count].len;

            pdata = scat_req->scat_list[sgitem_count].packet;
            page_num = scat_req->scat_list[sgitem_count].page_num;
            mpdu_len = HW_MPDU_LEN_GET(((struct hi_tx_desc *)pdata)->MPDUBufFlag);

            if (sg_count > (MAXSG_SIZE - page_num * 2))
            {
                AML_PRINT_LOG_INFO("sg_count > MAXSG_SIZE, sg_count:%d, page_num:%d, scat_count:%d\n", sg_count, page_num, scat_req->scat_count);
                break;
            }
            ttl_page_num += page_num;
            last_page_size = packet_len - (page_num - 1) * hif->CommStaticParam.tx_page_len;

            if (page_num == 1)
            {
                PbufferInfo = &(scat_req->bufferinfo[sg_count]);
                PbufferInfo->MPDUBufFlag = ((struct hi_tx_desc *)pdata)->MPDUBufFlag;
                sg_set_buf(&scat_req->sgentries[sg_count], (char *)PbufferInfo, sizeof(struct HW_TxBufferInfo));
                sg_count++;
                sg_set_buf(&scat_req->sgentries[sg_count], pdata + 4, packet_len - sizeof(struct HW_TxBufferInfo));
                sg_count++;
                ttl_len += packet_len;
            }
            else
            {
                for (i = 0; i < page_num; i++)
                {
                    if (i == 0)
                    {
                        PbufferInfo = &(scat_req->bufferinfo[sg_count]);
                        data_len = hif->CommStaticParam.tx_page_len - (sizeof(struct TxDescPage) - 1/* txdata[1] */);
                        PbufferInfo->MPDUBufFlag = ((struct hi_tx_desc *)pdata)->MPDUBufFlag;
                        PbufferInfo->MPDUBufFlag &= (HW_FIRST_AGG_FLAG | HW_LAST_AGG_FLAG);
                        PbufferInfo->MPDUBufFlag |= HW_FIRST_MPDUBUF_FLAG;
                        PbufferInfo->MPDUBufFlag |= HW_MPDU_LEN_SET(mpdu_len);
                        PbufferInfo->MPDUBufFlag |= HW_BUFFER_LEN_SET(data_len);
                        mpdubufflag = PbufferInfo->MPDUBufFlag;
                        sg_set_buf(&scat_req->sgentries[sg_count], (char *)PbufferInfo, sizeof(struct HW_TxBufferInfo));
                        sg_count++;
                        sg_set_buf(&scat_req->sgentries[sg_count], pdata + 4, hif->CommStaticParam.tx_page_len - sizeof(struct HW_TxBufferInfo));
                        sg_count++;
                        pkt_offset = hif->CommStaticParam.tx_page_len - 8;
                        ttl_len += hif->CommStaticParam.tx_page_len;
                    }
                    else if (i == page_num - 1)
                    {
                        PbufferInfo = &(scat_req->bufferinfo[sg_count]);
                        PbufferInfo->MPDUBufFlag = mpdubufflag;
                        PbufferInfo->MPDUBufFlag &= ~(HW_FIRST_MPDUBUF_FLAG | HW_LAST_MPDUBUF_FLAG);
                        PbufferInfo->MPDUBufFlag |= HW_LAST_MPDUBUF_FLAG;

                        /* buffer len is 0 for last page */
                        PbufferInfo->MPDUBufFlag &= 0xfffff;
                        sg_set_buf(&scat_req->sgentries[sg_count], (char *)PbufferInfo, sizeof(struct HW_TxBufferInfo));
                        sg_count++;
                        sg_set_buf(&scat_req->sgentries[sg_count], pdata + pkt_offset, last_page_size - sizeof(struct HW_TxBufferInfo));
                        sg_count++;
                        ttl_len += last_page_size;
                    }
                    else
                    {
                        PbufferInfo = &(scat_req->bufferinfo[sg_count]);
                        data_len = hif->CommStaticParam.tx_page_len - sizeof(struct HW_TxBufferInfo);
                        PbufferInfo->MPDUBufFlag = mpdubufflag;
                        PbufferInfo->MPDUBufFlag &= ~(HW_FIRST_MPDUBUF_FLAG | HW_LAST_MPDUBUF_FLAG);
                        PbufferInfo->MPDUBufFlag &= 0xfffff;
                        PbufferInfo->MPDUBufFlag |= HW_BUFFER_LEN_SET(data_len);

                        sg_set_buf(&scat_req->sgentries[sg_count], (char *)PbufferInfo, sizeof(struct HW_TxBufferInfo));
                        sg_count++;
                        sg_set_buf(&scat_req->sgentries[sg_count], pdata + pkt_offset, hif->CommStaticParam.tx_page_len - sizeof(struct HW_TxBufferInfo));
                        sg_count++;
                        pkt_offset += (hif->CommStaticParam.tx_page_len - sizeof(struct HW_TxBufferInfo));
                        ttl_len += hif->CommStaticParam.tx_page_len;
                    }
                }
                pkt_offset = 0;
            }
            sgitem_count++;
        }

        ret = usb_sg_init(&sgr, udev, usb_sndbulkpipe(udev, USB_EP1), 0, scat_req->sgentries,
            sg_count, 0, GFP_NOIO);

        if (ret)
        {
            AML_PRINT_LOG_INFO("usb_sg_init fail ret = %d\n", ret);
            return ret;
        }

        usb_sg_wait(&sgr);
        if (sgr.status != 0)
        {
            AML_PRINT_LOG_INFO("usb_sg_wait fail  %d\n", sgr.status);
            return -1;
        }
    }
    return 0;
}
extern struct auc_hif_ops_for_wifi g_auc_hif_ops_for_wifi;
extern void hif_get_sts(unsigned int op_code, unsigned int ctrl_code);
void hif_init_usb_ops(void)
{
    struct hw_interface* hif = hif_get_hw_interface();

#ifdef USB_BUILD_IN
    memcpy(&hif->hif_ops, &g_auc_hif_ops_for_wifi, sizeof(struct amlw_hif_ops));
#else
    hif->hif_ops.bt_hi_write_sram = aml_usb_write_sram;
    hif->hif_ops.bt_hi_read_sram = aml_usb_read_sram;
    hif->hif_ops.bt_hi_write_word = aml_usb_write_word;
    hif->hif_ops.bt_hi_read_word = aml_usb_read_word;
#endif

#if defined (HAL_FPGA_VER)

#if 0
    /* not use for usb interface */
    hif->hif_ops.hi_bottom_write8 = aml_usb_bottom_write8;
    hif->hif_ops.hi_bottom_read8 = aml_usb_bottom_read8;
    hif->hif_ops.hi_bottom_read = aml_usb_bottom_read;
    hif->hif_ops.hi_bottom_write = aml_usb_bottom_write;
    hif->hif_ops.hi_write8_func0 = aml_usb_bottom_write8_func0;
    hif->hif_ops.hi_read8_func0 = aml_usb_bottom_read8_func0;
    /* not use for usb interface */
#endif
    hif->hif_ops.hi_enable_scat = aml_usb_enable_scatter;
    hif->hif_ops.hi_cleanup_scat = aml_usb_cleanup_scatter;
    hif->hif_ops.hi_get_scatreq = aml_usb_scatter_req_get;

    hif->hif_ops.hi_send_frame = aml_usb_send_frame;
#elif defined (HAL_SIM_VER)
    hif->hif_ops.hi_send_frame = aml_usb_send_frame;//not use
#endif
    hif->hif_ops.hi_rcv_frame = aml_usb_recv_frame;

    hif->hif_ops.hi_read_reg8 = aml_usb_read_byte;
    hif->hif_ops.hi_write_reg8 = aml_usb_write_byte;
    hif->hif_ops.hi_read_reg32 = aml_usb_read_reg32;
    hif->hif_ops.hi_write_reg32 = aml_usb_write_reg32;

    hif->hif_ops.hi_write_word = aml_usb_write_word;
    hif->hif_ops.hi_read_word = aml_usb_read_word;
    hif->hif_ops.hi_read_efuse = aml_usb_read_efuse;
    hif->hif_ops.hi_write_sram = aml_usb_write_sram;
    hif->hif_ops.hi_read_sram = aml_usb_read_sram;

//    hif->hif_ops.hif_aon_write_reg = aml_usb_write_word;
//    hif->hif_ops.hif_aon_read_reg = aml_usb_read_word;

    hif->hif_ops.hif_get_sts = hif_get_sts;
    hif->hif_ops.hif_pt_rx_start = hif_pt_rx_start;
    hif->hif_ops.hif_pt_rx_stop = hif_pt_rx_stop;
}

#ifdef USB_BUILD_IN

extern struct usb_device *g_udev;
extern struct auc_hif_ops_for_wifi g_auc_hif_ops_for_wifi;
extern unsigned char auc_driver_insmoded;
extern int aml_usb_insmod(void);
#ifndef UBUNTU_PT_MODE
extern void set_usb_wifi_power(int is_on);
#endif
extern struct crg_msc_cbw *g_cmd_buf;
extern struct mutex auc_usb_mutex;
extern unsigned char auc_driver_probed;

int aml_usb_init(void)
{
    int ret = 0;
    struct hw_interface * hif = hif_get_hw_interface();
    struct hal_private * hal_priv = hal_get_priv();
    struct usb_hcd *p_usb_hcd;

    if (!auc_driver_insmoded) {
        aml_usb_insmod();
    }

    PRINT("usb build!!\n");

    if (auc_driver_probed == 0) {
        PRINT("There is no usb device was probed!!\n");
        return -1;
    }

    hif->udev = g_udev;
    g_cbw_buf = g_cmd_buf;

    g_buffer = ZMALLOC(2*sizeof(int), "fw_stat",GFP_DMA | GFP_ATOMIC);
    if (!g_buffer) {
        AML_PRINT_LOG_ERR("malloc fail!\n");
        return -ENOMEM;
    }

    g_cr =  ZMALLOC(sizeof(struct usb_ctrlrequest), "fw_stat",GFP_DMA | GFP_ATOMIC);
    if (!g_cr) {
        AML_PRINT_LOG_ERR("malloc fail!\n");
        return -ENOMEM;
    }

    g_urb = usb_alloc_urb(0, GFP_ATOMIC);
    if (!g_urb) {
        AML_PRINT_LOG_ERR("error,no urb!\n");
        return -ENOMEM;
    }
    tx_status_list_init(&(hif->tx_status_list), WIFI_MAX_TXFRAME*2);
    skb_queue_head_init(&hif->bcn_list_head);

    ret = hal_init_priv();
    if (ret != 0)
        goto create_thread_error;

    ret = hal_create_thread();
    if (ret != 0)
        goto create_thread_error;

    /*set parent dev for net dev. */
    vm_cfg80211_set_parent_dev(&hif->udev->dev);

    if ((hal_priv->hal_call_back != NULL)
        &&(hal_priv->hal_call_back->dev_probe != NULL))
    {
        AML_PRINT_LOG_INFO("hal_priv->hal_call_back->dev_probe\n");
        /*call driver probe to create vmac0 and vmac1 eventually*/
        ret = hal_priv->hal_call_back->dev_probe();
        if (ret < 0)
        {
            goto create_thread_error;
        }
    }
    hal_priv->powersave_init_flag = 0;

    PRINT("%s(%d): sg ops init\n", __func__, __LINE__);
    hif->hif_ops.hi_enable_scat();

    usb_stor_control_msg((unsigned long)hal_priv);
    hal_priv->hst_if_irq_en = 1;
    p_usb_hcd = container_of(hif->udev->bus, struct usb_hcd, self);
    irq_set_affinity_hint(p_usb_hcd->irq, cpumask_of(0));
    PRINT("aml_usb_init, usb_irq_num:%d, ret:%d\n", p_usb_hcd->irq, ret);

    if (aml_wifi_is_enable_rf_test()) {
        mib_init();
        driver_open();
    }
    return ret;

create_thread_error:
    hal_kill_thread();
    return ret;

}

extern unsigned char auc_driver_probed;

#ifndef SDIO_MODE_ON

void set_reg_fragment(unsigned int addr,unsigned int bit_end,
        unsigned int bit_start,unsigned int value)
{
    unsigned int tmp;
    unsigned int bitwidth = bit_end - bit_start + 1;
    int max_value = (bitwidth == 32)? -1 : (1 << (bitwidth)) - 1;
    struct hw_interface* hif = hif_get_hw_interface();

    ASSERT((bitwidth > 0)||(bit_start <= 31)||(bit_end <= 31));

    tmp = hif->hif_ops.hi_read_word(addr);
    tmp &= ~(max_value << bit_start); // clear [bit_end: bit_start]
    tmp |= ((value & max_value) << bit_start);

    hif->hif_ops.hi_write_word(addr,tmp);
}

/* aon module address from 0x00f00000, we need read/write by sdio func5 */
void aml_aon_write_reg(unsigned int addr,unsigned int data)
{
    struct hw_interface* hif = hif_get_hw_interface();
    hif->hif_ops.hi_write_word((addr), data);
}

unsigned int aml_aon_read_reg(unsigned int addr)
{
    unsigned int regdata = 0;
    struct hw_interface* hif = hif_get_hw_interface();
    regdata = hif->hif_ops.hi_read_word((addr));
    return regdata;
}

#endif

extern unsigned char recovery_notify_bt;
extern unsigned char recovery_done;
void aml_usb_disable_wifi(void)
{
    AML_PRINT_LOG_INFO("enter\n");
    wifi_usb_access = 0;

    recovery_notify_bt = 1;
    recovery_done = 0;

    /* 1.chip en off, usb disconnect */
#ifndef UBUNTU_PT_MODE
    set_usb_wifi_power(0);
#endif
    //waiting for usb disconnect
    while (auc_driver_probed == 1) {
        msleep(100);
    }

    /* 2.chip en on, usb probe */
#ifndef UBUNTU_PT_MODE
    set_usb_wifi_power(1);
#endif

    //waiting for usb probe
    while (auc_driver_probed == 0) {
        msleep(100);
    }
    AML_PRINT_LOG_INFO("usb power cycle ok\n");
}

void aml_usb_enable_wifi(void)
{
    struct hal_private *hal_priv = hal_get_priv();
    struct hw_interface * hif = hif_get_hw_interface();
    /* g_udev freed by auc_disconnect and realloc by auc_probe, old pointer reset to null,
       so hif->udev need to be reassigned */
    hif->udev = g_udev;
    /* same as hif->udev */
    g_cbw_buf = g_cmd_buf;
    hal_recovery_init_priv();
    wifi_usb_access = 1;
    hal_fw_repair();
    usb_stor_control_msg((unsigned long)hal_priv);
    recovery_done = 1;
}

void aml_usb_exit(void)
{
    struct hal_private * hal_priv = hal_get_priv();
    struct hw_interface * hif = hif_get_hw_interface();
    AML_PRINT_LOG_INFO("--------aml_usb:disconnect-------\n");
    hal_priv->powersave_init_flag = 1;

    if (wifi_usb_access) {
        hif->hif_ops.hi_write_word(RG_PMU_A22, 0x704);
        hif->hif_ops.hi_write_word(RG_PMU_A16, 0x0);
    }

    hal_free();
    g_cbw_buf = NULL;
    usb_kill_urb(g_urb);
    usb_free_urb(g_urb);
    FREE(g_cr,"fw_stat");
    FREE(g_buffer,"fw_stat");
    hal_kill_thread();
    vm_cfg80211_clear_parent_dev();

    hal_ops_detach();
#ifndef UBUNTU_PT_MODE
    set_usb_wifi_power(0);
#endif
#ifdef DRV_PT_SUPPORT
    b2b_tx_thread_remove();
#endif
    PRINT("aml_usb_exit++ \n");
}

#ifdef CONFIG_PM
static int aml_usb_suspend(struct usb_interface *interface,pm_message_t state)
{
    return 0;
}

static int aml_usb_resume(struct usb_interface *interface)
{
    return 0;
}
#endif

#else //USB_BUILD_IN

static const struct usb_device_id aml_usb_devices[] =
{
    {USB_DEVICE(W1u_VENDOR,W1u_PRODUCT)},
    {USB_DEVICE(W1u_VENDOR_AMLOGIC_EFUSE,W1uu_A_PRODUCT_AMLOGIC_EFUSE)},
    {USB_DEVICE(W1u_VENDOR_AMLOGIC_EFUSE,W1uu_B_PRODUCT_AMLOGIC_EFUSE)},
    {USB_DEVICE(W1u_VENDOR_AMLOGIC_EFUSE,W1uu_C_PRODUCT_AMLOGIC_EFUSE)},

    {}
};

MODULE_DEVICE_TABLE(usb, aml_usb_devices);

static int aml_usb_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
    int ret = 0;
    struct hw_interface * hif = hif_get_hw_interface();
    struct hal_private * hal_priv = hal_get_priv();
    struct usb_device *udev;

    udev = interface_to_usbdev(interface);
    udev = usb_get_dev(udev);
    hif->udev = udev;

    tx_status_list_init(&(hif->tx_status_list), WIFI_MAX_TXFRAME*2);
    skb_queue_head_init(&hif->bcn_list_head);

    USB_LOCK_INIT();

    usb_set_intfdata(interface,hif);

    g_buffer = ZMALLOC(2*sizeof(int), "fw_stat",GFP_DMA | GFP_ATOMIC);
    if (!g_buffer) {
        AML_PRINT_LOG_ERR("malloc fail!\n");
         return -ENOMEM;
    }

    g_cr =  ZMALLOC(sizeof(struct usb_ctrlrequest), "fw_stat",GFP_DMA | GFP_ATOMIC);
    if (!g_cr) {
        AML_PRINT_LOG_ERR("malloc fail!\n");
         return -ENOMEM;
    }

    g_urb = usb_alloc_urb(0, GFP_ATOMIC);
    if (!g_urb) {
        AML_PRINT_LOG_ERR("error,no urb!\n");
        return -ENOMEM;
    }
    ret = hal_init_priv();
    if (ret != 0)
        goto create_thread_error;

    ret = hal_create_thread();
    if (ret != 0)
        goto create_thread_error;

    /*set parent dev for net dev. */
    vm_cfg80211_set_parent_dev(&hif->udev->dev);

    if ((hal_priv->hal_call_back != NULL)
        &&(hal_priv->hal_call_back->dev_probe != NULL))
    {
        PRINT("hal_priv->hal_call_back->dev_probe\n");
        /*call driver probe to create vmac0 and vmac1 eventually*/
        ret = hal_priv->hal_call_back->dev_probe();
        if (ret < 0)
        {
            goto create_thread_error;
        }
    }
    hal_priv->powersave_init_flag = 0;

    PRINT("%s(%d): sg ops init\n", __func__, __LINE__);
    hif->hif_ops.hi_enable_scat();

    hal_priv->hst_if_irq_en = 1;
    usb_stor_control_msg((unsigned long)hal_priv);
    PRINT("aml_usb_probe-- ret %d\n", ret);

    if (aml_wifi_is_enable_rf_test()) {
        mib_init();
        driver_open();
    }

    return ret;

create_thread_error:
    hal_kill_thread();
    return ret;

}

static void aml_usb_disconnect(struct usb_interface *interface)
{
    struct hal_private * hal_priv = hal_get_priv();
    PRINT("--------aml_usb:disconnect-------\n");
    hal_priv->powersave_init_flag = 1;
    hal_free();

    hal_kill_thread();

    if (aml_wifi_is_enable_rf_test())
        b2b_tx_thread_remove();

    vm_cfg80211_clear_parent_dev();
    FREE(g_cbw_buf,"cmg stage");
    usb_free_urb(g_urb);
    usb_set_intfdata(interface, NULL);
    usb_put_dev(hal_priv->hif->udev);
}

#ifdef CONFIG_PM
static int aml_usb_suspend(struct usb_interface *interface,pm_message_t state)
{
    return 0;
}

static int aml_usb_resume(struct usb_interface *interface)
{
    return 0;
}
#endif



static struct usb_driver aml_usb_driver = {

    .name = "aml_w1u_usb",
    .id_table = aml_usb_devices,
    .probe = aml_usb_probe,
    .disconnect = aml_usb_disconnect,
#ifdef CONFIG_PM
    .suspend = aml_usb_suspend,
    .resume = aml_usb_resume,
#endif
};

int aml_usb_init(void)
{
    int err = 0;

    g_cbw_buf = ZMALLOC(sizeof(*g_cbw_buf),"cmd stage",GFP_DMA | GFP_ATOMIC);
    if (!g_cbw_buf) {
        AML_PRINT_LOG_ERR("g_cbw_buf malloc fail\n");
        return -ENOMEM;;
    }
    memset(g_cbw_buf,0,sizeof(struct crg_msc_cbw ));

    g_buffer = ZMALLOC(2*sizeof(int), "fw_stat",GFP_DMA | GFP_ATOMIC);
    if (!g_buffer) {
        AML_PRINT_LOG_ERR("malloc fail!\n");
         return -ENOMEM;
    }

    g_cr =  ZMALLOC(sizeof(struct usb_ctrlrequest), "fw_stat",GFP_DMA | GFP_ATOMIC);
    if (!g_cr) {
        AML_PRINT_LOG_ERR("malloc fail!\n");
         return -ENOMEM;
    }

    g_urb = usb_alloc_urb(0, GFP_ATOMIC);
    if (!g_urb) {
        AML_PRINT_LOG_ERR("error,no urb!\n");
        return -ENOMEM;
    }

    err = usb_register(&aml_usb_driver);
    PRINT("*****************aml usb driver init start...********************\n");

    if (err) {
        PRINT("failed to register usb driver: %d \n", err);
    }

    return err;
}

extern void set_usb_wifi_power(int is_on);

void aml_usb_exit(void)
{
    struct hw_interface * hif = hif_get_hw_interface();
    struct hal_private * hal_priv = hal_get_priv();

    hif->hif_ops.hi_write_word(RG_PMU_A22, 0x704);
    hif->hif_ops.hi_write_word(RG_PMU_A16, 0x0);

    PRINT("aml_usb_exit++ \n");
    usb_deregister(&aml_usb_driver);

    hal_ops_detach();
    set_usb_wifi_power(0);

    if (aml_wifi_is_enable_rf_test())
        b2b_tx_thread_remove();

    //aml_customer_gpio_wlan_ctrl(WLAN_POWER_OFF);
}
#endif
#ifdef HAL_SIM_VER
#ifdef FW_NAME
}
#endif
#endif
#endif


