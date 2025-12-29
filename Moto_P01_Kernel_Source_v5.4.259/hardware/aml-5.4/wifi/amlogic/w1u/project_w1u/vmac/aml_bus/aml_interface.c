#include <linux/init.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include "w1_sdio.h"
#include "aml_interface.h"

#ifdef SDIO_MODE_ON
char *bus_type = "sdio";

extern int aml_sdio_insmod(void);
extern int aml_sdio_rmmod(void);

#else
char *bus_type = "usb";
#endif

#ifdef CHIP_RESET_SUPPORT
struct drv_reset_ops {
    int (*enable_cb)(void);
    void (*disable_cb)(void);
} g_drv_reset_ops;

EXPORT_SYMBOL(g_drv_reset_ops);
#endif

unsigned int aml_bus_type;
struct aml_bus_state_detect bus_state_detect = {0};
unsigned char g_chip_function_ctrl;

EXPORT_SYMBOL(bus_type);
EXPORT_SYMBOL(aml_bus_type);
extern int aml_usb_insmod(void);
extern int aml_usb_rmmod(void);
EXPORT_SYMBOL(bus_state_detect);
EXPORT_SYMBOL(g_chip_function_ctrl);

#ifdef NOT_AMLOGIC_PLATFORM

#define AML_STATIC_VERSION_STR      "101.10.361.10 (wlan=r892223-20210623-1)"

#define DHD_SKB_1PAGE_BUFSIZE       (PAGE_SIZE * 1)
#define DHD_SKB_2PAGE_BUFSIZE       (PAGE_SIZE * 2)
#define DHD_SKB_4PAGE_BUFSIZE       (PAGE_SIZE * 4)

#define DHD_SKB_1PAGE_BUF_NUM   8
#define DHD_SKB_2PAGE_BUF_NUM   8
#define DHD_SKB_4PAGE_BUF_NUM   1

#define FW_VERBOSE_RING_SIZE            (256 * 1024)
#define DHD_PREALLOC_MEMDUMP_RAM_SIZE       (1290 * 1024)
#define NAN_EVENT_RING_SIZE     (64 * 1024)
#define WLAN_SKB_1_2PAGE_BUF_NUM ((DHD_SKB_1PAGE_BUF_NUM) + \
        (DHD_SKB_2PAGE_BUF_NUM))
#define WLAN_SKB_BUF_NUM ((WLAN_SKB_1_2PAGE_BUF_NUM) + (DHD_SKB_4PAGE_BUF_NUM))

void *wlan_static_dhd_memdump_ram_buf;
void *wlan_static_nan_event_ring_buf;
void *wlan_static_fw_verbose_ring_buf;

enum dhd_prealloc_index {
    DHD_PREALLOC_SKB_BUF = 4,
    DHD_PREALLOC_MEMDUMP_RAM = 11,
    DHD_PREALLOC_FW_VERBOSE_RING = 20,
    DHD_PREALLOC_NAN_EVENT_RING = 23,
    DHD_PREALLOC_MAX
};

void *aml_mem_prealloc(int section, unsigned long size)
{
    PRINT("sectoin %d, size %ld\n", section, size);

    if (section == DHD_PREALLOC_MEMDUMP_RAM) {
        if (size > DHD_PREALLOC_MEMDUMP_RAM_SIZE) {
            PRINT("request MEMDUMP_RAM(%lu) > %d\n",
                size, DHD_PREALLOC_MEMDUMP_RAM_SIZE);
            return NULL;
        }
        return wlan_static_dhd_memdump_ram_buf;
    }

    if (section == DHD_PREALLOC_FW_VERBOSE_RING) {
        if (size > FW_VERBOSE_RING_SIZE) {
            PRINT("request FW_VERBOSE_RING(%lu) > %d\n",
                size, FW_VERBOSE_RING_SIZE);
            return NULL;
        }
        return wlan_static_fw_verbose_ring_buf;
    }

    if (section < 0 || section > DHD_PREALLOC_MAX)
        PRINT("request section id(%d) is out of max %d\n",
            section, DHD_PREALLOC_MAX);

    PRINT("failed to alloc section %d, size=%ld\n",
        section, size);

    return NULL;
}
EXPORT_SYMBOL(aml_mem_prealloc);

int aml_init_wlan_mem(void)
{
    unsigned long size = 0;
    PRINT("%s\n", AML_STATIC_VERSION_STR);

    wlan_static_dhd_memdump_ram_buf = kmalloc(DHD_PREALLOC_MEMDUMP_RAM_SIZE, GFP_KERNEL);
    if (!wlan_static_dhd_memdump_ram_buf)
         goto err_mem_alloc;
    size += DHD_PREALLOC_MEMDUMP_RAM_SIZE;
    PRINT("sectoin %d, size=%d\n",
        DHD_PREALLOC_MEMDUMP_RAM, DHD_PREALLOC_MEMDUMP_RAM_SIZE);

    wlan_static_fw_verbose_ring_buf = kmalloc(FW_VERBOSE_RING_SIZE, GFP_KERNEL);
    if (!wlan_static_fw_verbose_ring_buf)
        goto err_mem_alloc;
    size += FW_VERBOSE_RING_SIZE;
    PRINT("sectoin %d, size=%d\n",
        DHD_PREALLOC_FW_VERBOSE_RING, FW_VERBOSE_RING_SIZE);

    PRINT("prealloc ok: %ld(%ldK)\n", size, size / 1024);
    return 0;

err_mem_alloc:
    kfree(wlan_static_dhd_memdump_ram_buf);
    kfree(wlan_static_fw_verbose_ring_buf);
    PRINT("Failed to mem_alloc for WLAN\n");

    return -ENOMEM;
}
EXPORT_SYMBOL(aml_init_wlan_mem);

int aml_deinit_wlan_mem(void) {
    int ret = 0;
    if (wlan_static_dhd_memdump_ram_buf) {
        kfree(wlan_static_dhd_memdump_ram_buf);
    }

    if (wlan_static_fw_verbose_ring_buf) {
        kfree(wlan_static_fw_verbose_ring_buf);
    }
    return ret;
}
#endif

int aml_bus_intf_insmod(void)
{
    int ret;
    if (strncmp(bus_type,"usb",3) == 0) {
        aml_bus_type = 1;
#ifdef CONFIG_USB
        ret = aml_usb_insmod();
        if (ret) {
            printk("aml usb bus init fail\n");
        }
#endif
    }
#ifdef SDIO_MODE_ON
    else if (strncmp(bus_type,"sdio",4) == 0) {
        aml_bus_type = 0;
        ret = aml_sdio_insmod();
        if (ret) {
            printk("aml sdio bus init fail\n");
        }
    }
#endif
    return 0;
}

void aml_bus_intf_rmmod(void)
{
    if (strncmp(bus_type,"usb",3) == 0) {
#ifdef CONFIG_USB
        aml_usb_rmmod();
#endif
    }
#ifdef SDIO_MODE_ON
    else if (strncmp(bus_type,"sdio",4) == 0) {
        aml_sdio_rmmod();
    }
#endif
#ifdef NOT_AMLOGIC_PLATFORM
    aml_deinit_wlan_mem();
#endif
}

module_param(bus_type, charp,S_IRUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(bus_type,"A string variable to adjust pci or sdio or usb bus interface");
module_init(aml_bus_intf_insmod);
module_exit(aml_bus_intf_rmmod);

MODULE_LICENSE("GPL");
