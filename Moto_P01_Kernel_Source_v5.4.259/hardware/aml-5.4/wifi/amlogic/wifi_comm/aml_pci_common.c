/**
 ******************************************************************************
 *
 * @file aml_pci.c
 *
 * Copyright (C) Amlogic 2012-2021
 *
 ******************************************************************************
 */
#include <linux/pci.h>
#include <linux/module.h>
#include <linux/init.h>
#include "aml_wifi_bus.h"

#define W2p_VENDOR_AMLOGIC_EFUSE 0x1F35
#define W2p_PRODUCT_AMLOGIC_EFUSE 0x0602
#define W2pRevB_PRODUCT_AMLOGIC_EFUSE 0x0642

static const struct pci_device_id aml_wifi_pci_ids[] =
{
    {PCI_DEVICE(0x0, 0x0)},
    {PCI_DEVICE(W2p_VENDOR_AMLOGIC_EFUSE, W2p_PRODUCT_AMLOGIC_EFUSE)},
    {PCI_DEVICE(W2p_VENDOR_AMLOGIC_EFUSE, W2pRevB_PRODUCT_AMLOGIC_EFUSE)},
    {0,}
};

static void aml_v7_platform_deinit(struct pci_dev *pci_dev)
{
    pci_disable_device(pci_dev);
    pci_release_regions(pci_dev);
    pci_clear_master(pci_dev);
    pci_disable_msi(pci_dev);

}

int aml_v7_platform_init(struct pci_dev *pci_dev)
{
    int ret = -ENOMEM;

    if ((ret = pci_enable_device(pci_dev))) {
        dev_err(&(pci_dev->dev), "pci_enable_device failed\n");
        goto out_enable;
    }

    pci_set_master(pci_dev);

    if ((ret = pci_request_regions(pci_dev, KBUILD_MODNAME))) {
        dev_err(&(pci_dev->dev), "pci_request_regions failed\n");
        goto out_request;
    }

    if (pci_enable_msi(pci_dev))
    {
        dev_err(&(pci_dev->dev), "pci_enable_msi failed\n");
        goto out_msi;
    }

    return 0;

  out_msi:
    pci_release_regions(pci_dev);
  out_request:
    pci_clear_master(pci_dev);
    pci_disable_device(pci_dev);
  out_enable:

    return ret;
}


/* Uncomment this for depmod to create module alias */
/* We don't want this on development platform */
//MODULE_DEVICE_TABLE(pci, aml_pci_ids);

static int aml_wifi_pci_probe(struct pci_dev *pci_dev,
                          const struct pci_device_id *pci_id)
{
    int ret = -ENODEV;

    printk("%s:%d %x\n", __func__, __LINE__, pci_id->vendor);
    ret = aml_v7_platform_init(pci_dev);
    if (ret)
        return ret;

    aml_wifi_chip("aml_w2_p");
    printk("%s: ********get chip type: aml_w2_p \n", __func__);
    return ret;
}

static void aml_wifi_pci_remove(struct pci_dev *pci_dev)
{
    printk("%s:%d \n", __func__, __LINE__);
    aml_v7_platform_deinit(pci_dev);
}

static struct pci_driver aml_wifi_pci_drv = {
    .name     = KBUILD_MODNAME,
    .id_table = aml_wifi_pci_ids,
    .probe    = aml_wifi_pci_probe,
    .remove   = aml_wifi_pci_remove,
};

int aml_wifi_pci_insmod(void)
{
    int err = 0;

    err = pci_register_driver(&aml_wifi_pci_drv);

    if (err) {
        printk("failed to register pci driver: %d \n", err);
    }

    printk("%s:%d \n", __func__, __LINE__);
    return err;
}

void aml_wifi_pci_rmmod(void)
{
    pci_unregister_driver(&aml_wifi_pci_drv);


    printk("%s(%d) aml common driver rmsmod\n",__func__, __LINE__);
}


