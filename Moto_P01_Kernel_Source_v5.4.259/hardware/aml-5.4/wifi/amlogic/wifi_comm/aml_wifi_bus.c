#include <linux/init.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include "aml_wifi_bus.h"
#include "aml_sdio_common.h"
#include "aml_usb_common.h"
#include "aml_pci_common.h"

static unsigned int g_chipid = WIFI_CHIP_W2_USB;
static char *g_aml_chipid_to_str[] = {
    [WIFI_CHIP_W1] = "aml_w1",
    [WIFI_CHIP_W1U] = "aml_w1u",
    [WIFI_CHIP_W1U_USB] = "aml_w1u_usb",
    [WIFI_CHIP_W1U_SDIO] = "aml_w1u_sdio",
    [WIFI_CHIP_W2_USB] = "aml_w2u",
    [WIFI_CHIP_W2_SDIO] = "aml_w2s",
    [WIFI_CHIP_W2_PCIE] = "aml_w2p",
    [WIFI_CHIP_UNKNOWN] = "unknown",
};

void aml_chipid_set(unsigned int chipid)
{
    if (chipid < WIFI_CHIP_UNKNOWN)
        g_chipid = chipid;
}

static ssize_t aml_proc_chipid_read(struct file *file,
                                    char __user *user_buf,
                                    size_t count, loff_t *ppos)
{
    int len = 0;
    char buf[50] = {0};
    ssize_t read;

    len += scnprintf(&buf[len], min_t(size_t, sizeof(buf) - 1, count),
                     "%s\n", g_aml_chipid_to_str[g_chipid]);

    read = simple_read_from_buffer(user_buf, count, ppos, buf, len);

    return read;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0)
static const struct file_operations aml_proc_chipid_ops = {
    .owner = THIS_MODULE,
    .read = aml_proc_chipid_read,
};
#else
static const struct proc_ops aml_proc_chipid_ops = {
    .proc_read = aml_proc_chipid_read,
};
#endif

struct proc_dir_entry *g_chipid_dir_entry;

int aml_wifi_bus_insmod(void)
{
    int ret;

#ifdef CONFIG_USB
    ret = aml_wifi_usb_insmod();
    if (ret) {
        printk("aml usb bus insmod fail\n");
    }
#endif
    ret = aml_wifi_sdio_insmod();
    if (ret) {
        printk("aml sdio bus insmod fail\n");
    }
#if 0
    ret = aml_wifi_pci_insmod();
    if (ret) {
        printk("aml pcie bus insmod fail\n");
    }
#endif

    g_chipid_dir_entry = proc_create("aml_chipid", 0444, NULL, &aml_proc_chipid_ops);

    return 0;
}

void aml_wifi_bus_rmmod(void)
{
    if (g_chipid_dir_entry)
        remove_proc_entry("aml_chipid", NULL);

#ifdef CONFIG_USB
    aml_wifi_usb_rmmod();
#endif
    aml_wifi_sdio_rmmod();
//    aml_wifi_pci_rmmod();
    printk("rmmod aml wifi comm");
}
module_init(aml_wifi_bus_insmod);
module_exit(aml_wifi_bus_rmmod);

MODULE_LICENSE("GPL");
