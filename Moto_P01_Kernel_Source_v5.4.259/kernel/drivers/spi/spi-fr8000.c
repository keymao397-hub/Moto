// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *    Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * Copyright (C) 2023 giantsemi
 *  giantsemi FR8000 UWB SPI driver
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/acpi.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/kobject.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/delay.h>

/*for kernel log*/
#define FR8000_LOG
#ifdef FR8000_LOG
#define FR_LOG(fmt,arg...)          do{printk("<FR8000>[%s:%d]"fmt"\n",__func__, __LINE__, ##arg);}while(0)
#else
#define FR_LOG(fmt,arg...)          do{}while(0)
#endif

#define DEV_NAME        "fr8000"
#define MAX_TRANS        2
#define WAIT_TIMEOUT    (HZ/10)

#define MAX_RETRY        3
#define MAX_WAIT        160
#define MAX_DATA        260

#define REG_ID_MISC_IRQ_STATUS    0x09
#define REG_ID_MISC_SOC_CTRL    0x27
#define REG_ID_MISC_TX_CMD_BUF    0x28
#define REG_ID_MISC_RX_CMD_BUF    0x29

#define SPI_MSG_CMD_GENERIC_READ    0x80
#define SPI_MSG_CMD_GENERIC_NO_DATA    0x81
#define SPI_MSG_CMD_UCI                0x90
#define SPI_MSG_CMD_UCI_ACK            0x91
#define SPI_MSG_CMD_UCI_DATA        0x92

#define SYS_IRQ_FLASH_RUNNING        0x0100
#define SYS_IRQ_CMD_2_HOST_READY    0x0200
#define SYS_IRQ_MCU_DATA_IND_READY    0x0400
#define SYS_IRQ_MCU_HALT            0x0040

/* Read / Write of the SPI device power field */
#define SPI_IOC_RD_POWER        _IOR(SPI_IOC_MAGIC, 6, __u32)
#define SPI_IOC_WR_POWER        _IOW(SPI_IOC_MAGIC, 6, __u32)

/* Read / Write of the SPI device wait field */
#define SPI_IOC_RD_WAIT        _IOR(SPI_IOC_MAGIC, 7, __u32)
#define SPI_IOC_WR_WAIT        _IOW(SPI_IOC_MAGIC, 7, __u32)

/* Read / Write of the SPI device reset field */
#define SPI_IOC_RD_RESET        _IOR(SPI_IOC_MAGIC, 8, __u32)
#define SPI_IOC_WR_RESET        _IOW(SPI_IOC_MAGIC, 8, __u32)

/* Read / Write of the SPI device wakup field */
#define SPI_IOC_RD_WAKEUP        _IOR(SPI_IOC_MAGIC, 9, __u32)
#define SPI_IOC_WR_WAKEUP        _IOW(SPI_IOC_MAGIC, 9, __u32)

/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *    is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK        ( SPI_MODE_3 | SPI_CS_HIGH \
                | SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
                | SPI_NO_CS | SPI_READY | SPI_TX_DUAL \
                | SPI_TX_QUAD | SPI_TX_OCTAL | SPI_RX_DUAL \
                | SPI_RX_QUAD | SPI_RX_OCTAL)

struct spidev_data {
    dev_t            devt;
    spinlock_t        spi_lock;
    spinlock_t        irq_lock;
    wait_queue_head_t    wait;
    struct cdev        cdev;
    struct device        *device;
    struct spi_device    *spi;
    struct list_head    device_entry;

    /* TX/RX buffers are NULL unless this device is open (users > 0) */
    struct mutex        buf_lock;
    struct mutex        wait_lock;
    struct mutex        wake_lock;
    unsigned int        users;
    u8                    *tx_buffer;
    u8                    *rx_buffer;
    u32                    speed_hz;
    u32                    power;
    volatile u32        wakeup;
    int                    aon_gpio;
    int                    rst_gpio;
    int                    wake_gpio;
    int                    irq_gpio;
    int                    irq;
};

static unsigned int spidev_major;
static struct class *spidev_class;
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

struct spidev_data    *g_spidev;

static unsigned int bufsiz = 2048;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

static int spidev_power(struct spidev_data *spidev, int on_off);
static int spidev_wait(struct spidev_data *spidev, unsigned int cmd, unsigned long arg);
static int spidev_reset(struct spidev_data *spidev);
static int spidev_wakeup(struct spidev_data *spidev);

/*-------------------------------------------------------------------------*/

static ssize_t
spidev_sync(struct spidev_data *spidev, struct spi_message *message)
{
    int status;
    struct spi_device *spi;

    spin_lock_irq(&spidev->spi_lock);
    spi = spidev->spi;
    spin_unlock_irq(&spidev->spi_lock);

    if (spi == NULL)
        status = -ESHUTDOWN;
    else
        status = spi_sync(spi, message);

    if (status == 0)
        status = message->actual_length;

    return status;
}

static inline ssize_t
spidev_sync_write(struct spidev_data *spidev, size_t len)
{
    struct spi_transfer    t = {
            .tx_buf        = spidev->tx_buffer,
            .len        = len,
            .speed_hz    = spidev->speed_hz,
        };
    struct spi_message    m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return spidev_sync(spidev, &m);
}

static inline ssize_t
spidev_sync_read(struct spidev_data *spidev, size_t len)
{
    struct spi_transfer    t = {
            .rx_buf        = spidev->rx_buffer,
            .len        = len,
            .speed_hz    = spidev->speed_hz,
        };
    struct spi_message    m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return spidev_sync(spidev, &m);
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t
spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct spidev_data    *spidev;
    ssize_t            status;

    /* chipselect only toggles at start or end of operation */
    if (count > bufsiz)
        return -EMSGSIZE;

    spidev = filp->private_data;

    mutex_lock(&spidev->buf_lock);
    status = spidev_sync_read(spidev, count);
    if (status > 0) {
        unsigned long    missing;

        missing = copy_to_user(buf, spidev->rx_buffer, status);
        if (missing == status)
            status = -EFAULT;
        else
            status = status - missing;
    }
    mutex_unlock(&spidev->buf_lock);

    return status;
}

/* Write-only message with current device setup */
static ssize_t
spidev_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *f_pos)
{
    struct spidev_data    *spidev;
    ssize_t            status;
    unsigned long        missing;

    /* chipselect only toggles at start or end of operation */
    if (count > bufsiz)
        return -EMSGSIZE;

    spidev = filp->private_data;

    mutex_lock(&spidev->buf_lock);
    missing = copy_from_user(spidev->tx_buffer, buf, count);
    if (missing == 0)
        status = spidev_sync_write(spidev, count);
    else
        status = -EFAULT;
    mutex_unlock(&spidev->buf_lock);

    return status;
}

static int spidev_message(struct spidev_data *spidev,
        struct spi_ioc_transfer *u_xfers, unsigned int n_xfers)
{
    struct spi_message    msg;
    struct spi_transfer    k_xfers[MAX_TRANS];
    struct spi_transfer    *k_tmp;
    struct spi_ioc_transfer *u_tmp;
    unsigned int        n, total, tx_total, rx_total;
    u8            *tx_buf, *rx_buf;
    int            status = -EFAULT;

    spi_message_init(&msg);
    /* k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
     * if (k_xfers == NULL)
     *    return -ENOMEM;
     */
    if (n_xfers > MAX_TRANS)
        n_xfers = MAX_TRANS;
    memset(k_xfers, 0, sizeof(k_xfers));

    /* Construct spi_message, copying any tx data to bounce buffer.
     * We walk the array of user-provided transfers, using each one
     * to initialize a kernel version of the same transfer.
     */
    tx_buf = spidev->tx_buffer;
    rx_buf = spidev->rx_buffer;
    total = 0;
    tx_total = 0;
    rx_total = 0;
    for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
            n;
            n--, k_tmp++, u_tmp++) {
        /* Ensure that also following allocations from rx_buf/tx_buf will meet
         * DMA alignment requirements.
         */
        unsigned int len_aligned = ALIGN(u_tmp->len, ARCH_KMALLOC_MINALIGN);

        k_tmp->len = u_tmp->len;

        total += k_tmp->len;
        /* Since the function returns the total length of transfers
         * on success, restrict the total to positive int values to
         * avoid the return value looking like an error.  Also check
         * each transfer length to avoid arithmetic overflow.
         */
        if (total > INT_MAX || k_tmp->len > INT_MAX) {
            status = -EMSGSIZE;
            goto done;
        }

        if (u_tmp->rx_buf) {
            /* this transfer needs space in RX bounce buffer */
            rx_total += len_aligned;
            if (rx_total > bufsiz) {
                status = -EMSGSIZE;
                goto done;
            }
            k_tmp->rx_buf = rx_buf;
            rx_buf += len_aligned;
        }
        if (u_tmp->tx_buf) {
            /* this transfer needs space in TX bounce buffer */
            tx_total += len_aligned;
            if (tx_total > bufsiz) {
                status = -EMSGSIZE;
                goto done;
            }
            k_tmp->tx_buf = tx_buf;
            if (copy_from_user(tx_buf, (const u8 __user *)
                        (uintptr_t) u_tmp->tx_buf,
                    u_tmp->len))
                goto done;
            tx_buf += len_aligned;
        }

        k_tmp->cs_change = !!u_tmp->cs_change;
        k_tmp->tx_nbits = u_tmp->tx_nbits;
        k_tmp->rx_nbits = u_tmp->rx_nbits;
        k_tmp->bits_per_word = u_tmp->bits_per_word;
        k_tmp->delay_usecs = u_tmp->delay_usecs;
        k_tmp->speed_hz = u_tmp->speed_hz;
        k_tmp->word_delay_usecs = u_tmp->word_delay_usecs;
        if (!k_tmp->speed_hz)
            k_tmp->speed_hz = spidev->speed_hz;
#ifdef VERBOSE
        dev_dbg(&spidev->spi->dev,
            "  xfer len %u %s%s%s%dbits %u usec %u usec %uHz\n",
            k_tmp->len,
            k_tmp->rx_buf ? "rx " : "",
            k_tmp->tx_buf ? "tx " : "",
            k_tmp->cs_change ? "cs " : "",
            k_tmp->bits_per_word ? : spidev->spi->bits_per_word,
            k_tmp->delay.value,
            k_tmp->word_delay.value,
            k_tmp->speed_hz ? : spidev->spi->max_speed_hz);
#endif
        spi_message_add_tail(k_tmp, &msg);
    }

    status = spidev_sync(spidev, &msg);
    if (status < 0)
        goto done;

    /* copy any rx data out of bounce buffer */
    for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
            n;
            n--, k_tmp++, u_tmp++) {
        if (u_tmp->rx_buf) {
            if (copy_to_user((u8 __user *)
                    (uintptr_t) u_tmp->rx_buf, k_tmp->rx_buf,
                    u_tmp->len)) {
                status = -EFAULT;
                goto done;
            }
        }
    }
    status = total;

done:
    /* kfree(k_xfers); */
    return status;
}

static struct spi_ioc_transfer *
spidev_get_ioc_message(unsigned int cmd, struct spi_ioc_transfer __user *u_ioc,
        unsigned int *n_ioc)
{
    u32    tmp;

    /* Check type, command number and direction */
    if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC
            || _IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
            || _IOC_DIR(cmd) != _IOC_WRITE)
        return ERR_PTR(-ENOTTY);

    tmp = _IOC_SIZE(cmd);
    if ((tmp % sizeof(struct spi_ioc_transfer)) != 0)
        return ERR_PTR(-EINVAL);
    *n_ioc = tmp / sizeof(struct spi_ioc_transfer);
    if (*n_ioc == 0)
        return NULL;

    /* copy into scratch area */
    return memdup_user(u_ioc, tmp);
}

static long
spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int            retval = 0;
    struct spidev_data    *spidev;
    struct spi_device    *spi;
    u32            tmp;
    unsigned int        n_ioc;
    struct spi_ioc_transfer    *ioc;

    /* Check type and command number */
    if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
        return -ENOTTY;

    /* guard against device removal before, or while,
     * we issue this ioctl.
     */
    spidev = filp->private_data;
    if (cmd == SPI_IOC_RD_WAIT || cmd == SPI_IOC_WR_WAIT)
        return spidev_wait(spidev, cmd, arg);
    spin_lock_irq(&spidev->spi_lock);
    spi = spi_dev_get(spidev->spi);
    spin_unlock_irq(&spidev->spi_lock);

    if (spi == NULL)
        return -ESHUTDOWN;

    /* use the buffer lock here for triple duty:
     *  - prevent I/O (from us) so calling spi_setup() is safe;
     *  - prevent concurrent SPI_IOC_WR_* from morphing
     *    data fields while SPI_IOC_RD_* reads them;
     *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
     */
    mutex_lock(&spidev->buf_lock);

    switch (cmd) {
    /* read requests */
    case SPI_IOC_RD_MODE:
        retval = put_user(spi->mode & SPI_MODE_MASK,
                    (__u8 __user *)arg);
        break;
    case SPI_IOC_RD_MODE32:
        retval = put_user(spi->mode & SPI_MODE_MASK,
                    (__u32 __user *)arg);
        break;
    case SPI_IOC_RD_LSB_FIRST:
        retval = put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
                    (__u8 __user *)arg);
        break;
    case SPI_IOC_RD_BITS_PER_WORD:
        retval = put_user(spi->bits_per_word, (__u8 __user *)arg);
        break;
    case SPI_IOC_RD_MAX_SPEED_HZ:
        retval = put_user(spidev->speed_hz, (__u32 __user *)arg);
        break;
    case SPI_IOC_RD_POWER:
        retval = put_user(spidev->power, (__u32 __user *)arg);
        break;
    case SPI_IOC_RD_RESET:
        retval = put_user(0, (__u32 __user *)arg);
        break;
    case SPI_IOC_RD_WAKEUP:
        retval = put_user(1, (__u32 __user *)arg);
        break;

    /* write requests */
    case SPI_IOC_WR_MODE:
    case SPI_IOC_WR_MODE32:
        if (cmd == SPI_IOC_WR_MODE)
            retval = get_user(tmp, (u8 __user *)arg);
        else
            retval = get_user(tmp, (u32 __user *)arg);
        if (retval == 0) {
            struct spi_controller *ctlr = spi->controller;
            u32    save = spi->mode;

            if (tmp & ~SPI_MODE_MASK) {
                retval = -EINVAL;
                break;
            }

            if (ctlr->use_gpio_descriptors && ctlr->cs_gpiods &&
                ctlr->cs_gpiods[spi->chip_select])
                tmp |= SPI_CS_HIGH;

            tmp |= spi->mode & ~SPI_MODE_MASK;
            spi->mode = (u16)tmp;
            retval = spi_setup(spi);
            if (retval < 0)
                spi->mode = save;
            else
                dev_dbg(&spi->dev, "spi mode %x\n", tmp);
        }
        break;
    case SPI_IOC_WR_LSB_FIRST:
        retval = get_user(tmp, (__u8 __user *)arg);
        if (retval == 0) {
            u32    save = spi->mode;

            if (tmp)
                spi->mode |= SPI_LSB_FIRST;
            else
                spi->mode &= ~SPI_LSB_FIRST;
            retval = spi_setup(spi);
            if (retval < 0)
                spi->mode = save;
            else
                dev_dbg(&spi->dev, "%csb first\n",
                        tmp ? 'l' : 'm');
        }
        break;
    case SPI_IOC_WR_BITS_PER_WORD:
        retval = get_user(tmp, (__u8 __user *)arg);
        if (retval == 0) {
            u8    save = spi->bits_per_word;

            spi->bits_per_word = tmp;
            retval = spi_setup(spi);
            if (retval < 0)
                spi->bits_per_word = save;
            else
                dev_dbg(&spi->dev, "%d bits per word\n", tmp);
        }
        break;
    case SPI_IOC_WR_MAX_SPEED_HZ:
        retval = get_user(tmp, (__u32 __user *)arg);
        if (retval == 0) {
            u32    save = spi->max_speed_hz;

            spi->max_speed_hz = tmp;
            retval = spi_setup(spi);
            if (retval == 0) {
                spidev->speed_hz = tmp;
                dev_dbg(&spi->dev, "%d Hz (max)\n",
                    spidev->speed_hz);
            }
            spi->max_speed_hz = save;
        }
        break;
    case SPI_IOC_WR_POWER:
        retval = get_user(tmp, (__u32 __user *)arg);
        if (retval == 0)
            spidev_power(spidev, tmp);
        break;
    case SPI_IOC_WR_RESET:
        retval = get_user(tmp, (__u32 __user *)arg);
        if (retval == 0 && tmp)
            spidev_reset(spidev);
        break;
    case SPI_IOC_WR_WAKEUP:
        retval = get_user(tmp, (__u32 __user *)arg);
        if (retval == 0 && tmp)
            spidev_wakeup(spidev);
        break;

    default:
        /* segmented and/or full-duplex I/O request */
        /* Check message and copy into scratch area */
        ioc = spidev_get_ioc_message(cmd,
                (struct spi_ioc_transfer __user *)arg, &n_ioc);
        if (IS_ERR(ioc)) {
            retval = PTR_ERR(ioc);
            break;
        }
        if (!ioc)
            break;    /* n_ioc is also 0 */

        /* translate to spi_message, execute */
        retval = spidev_message(spidev, ioc, n_ioc);
        kfree(ioc);
        break;
    }

    mutex_unlock(&spidev->buf_lock);
    spi_dev_put(spi);
    return retval;
}

#ifdef CONFIG_COMPAT
static long
spidev_compat_ioc_message(struct file *filp, unsigned int cmd,
        unsigned long arg)
{
    struct spi_ioc_transfer __user    *u_ioc;
    int                retval = 0;
    struct spidev_data        *spidev;
    struct spi_device        *spi;
    unsigned int            n_ioc, n;
    struct spi_ioc_transfer        *ioc;

    u_ioc = (struct spi_ioc_transfer __user *) compat_ptr(arg);

    /* guard against device removal before, or while,
     * we issue this ioctl.
     */
    spidev = filp->private_data;
    spin_lock_irq(&spidev->spi_lock);
    spi = spi_dev_get(spidev->spi);
    spin_unlock_irq(&spidev->spi_lock);

    if (spi == NULL)
        return -ESHUTDOWN;

    /* SPI_IOC_MESSAGE needs the buffer locked "normally" */
    mutex_lock(&spidev->buf_lock);

    /* Check message and copy into scratch area */
    ioc = spidev_get_ioc_message(cmd, u_ioc, &n_ioc);
    if (IS_ERR(ioc)) {
        retval = PTR_ERR(ioc);
        goto done;
    }
    if (!ioc)
        goto done;    /* n_ioc is also 0 */

    /* Convert buffer pointers */
    for (n = 0; n < n_ioc; n++) {
        ioc[n].rx_buf = (uintptr_t) compat_ptr(ioc[n].rx_buf);
        ioc[n].tx_buf = (uintptr_t) compat_ptr(ioc[n].tx_buf);
    }

    /* translate to spi_message, execute */
    retval = spidev_message(spidev, ioc, n_ioc);
    kfree(ioc);

done:
    mutex_unlock(&spidev->buf_lock);
    spi_dev_put(spi);
    return retval;
}

static long
spidev_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    if (_IOC_TYPE(cmd) == SPI_IOC_MAGIC
            && _IOC_NR(cmd) == _IOC_NR(SPI_IOC_MESSAGE(0))
            && _IOC_DIR(cmd) == _IOC_WRITE)
        return spidev_compat_ioc_message(filp, cmd, arg);

    return spidev_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spidev_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int spidev_open(struct inode *inode, struct file *filp)
{
    struct spidev_data    *spidev;
    int            status = -ENXIO;

    mutex_lock(&device_list_lock);

    list_for_each_entry(spidev, &device_list, device_entry) {
        if (spidev->devt == inode->i_rdev) {
            status = 0;
            break;
        }
    }

    if (status) {
        pr_debug("spidev: nothing for minor %d\n", iminor(inode));
        goto err_find_dev;
    }

    if (!spidev->tx_buffer) {
        spidev->tx_buffer = kmalloc(bufsiz, GFP_KERNEL);
        if (!spidev->tx_buffer) {
            dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
            status = -ENOMEM;
            goto err_find_dev;
        }
    }

    if (!spidev->rx_buffer) {
        spidev->rx_buffer = kmalloc(bufsiz, GFP_KERNEL);
        if (!spidev->rx_buffer) {
            dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
            status = -ENOMEM;
            goto err_alloc_rx_buf;
        }
    }

    spidev->users++;
    filp->private_data = spidev;
    stream_open(inode, filp);

    mutex_unlock(&device_list_lock);
    return 0;

err_alloc_rx_buf:
    kfree(spidev->tx_buffer);
    spidev->tx_buffer = NULL;
err_find_dev:
    mutex_unlock(&device_list_lock);
    return status;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
    struct spidev_data    *spidev;
    int            dofree;

    mutex_lock(&device_list_lock);
    spidev = filp->private_data;
    filp->private_data = NULL;

    spin_lock_irq(&spidev->spi_lock);
    /* ... after we unbound from the underlying device? */
    dofree = (spidev->spi == NULL);
    spin_unlock_irq(&spidev->spi_lock);

    /* last close? */
    spidev->users--;
    if (!spidev->users) {

        kfree(spidev->tx_buffer);
        spidev->tx_buffer = NULL;

        kfree(spidev->rx_buffer);
        spidev->rx_buffer = NULL;

        if (dofree)
            kfree(spidev);
        else
            spidev->speed_hz = spidev->spi->max_speed_hz;
    }
#ifdef CONFIG_SPI_SLAVE
    if (!dofree)
        spi_slave_abort(spidev->spi);
#endif
    mutex_unlock(&device_list_lock);

    return 0;
}

static const struct file_operations spidev_fops = {
    .owner =    THIS_MODULE,
    /* REVISIT switch to aio primitives, so that userspace
     * gets more complete API coverage.  It'll simplify things
     * too, except for the locking.
     */
    .write =    spidev_write,
    .read =        spidev_read,
    .unlocked_ioctl = spidev_ioctl,
    .compat_ioctl = spidev_compat_ioctl,
    .open =        spidev_open,
    .release =    spidev_release,
    .llseek =    no_llseek,
};

/*-------------------------------------------------------------------------*/

static irqreturn_t spidev_interrupt(int irq, void *dev_id)
{
    struct spidev_data    *spidev = dev_id;
    unsigned long        flags;

    spin_lock_irqsave(&spidev->irq_lock, flags);
    if (spidev->wakeup == 0)
        spidev->wakeup = 1;
    spin_unlock_irqrestore(&spidev->irq_lock, flags);
    wake_up_interruptible(&spidev->wait);
    return IRQ_HANDLED;
}

static int spidev_power(struct spidev_data *spidev, int on_off)
{
    FR_LOG("%s >>> power %s, spidev->power=%d.\n", __func__, on_off ? "on" : "off",spidev->power);
    if (on_off) {
        if (!spidev->power) {
            spidev->power = !spidev->power;
            /* TODO power on... */
            if (spidev->rst_gpio > 0) {
                gpio_set_value(spidev->rst_gpio, 0);
            }
            if (spidev->wake_gpio > 0) {
                gpio_set_value(spidev->wake_gpio, 0);
            }

            if (spidev->aon_gpio > 0) {
                msleep(3);
                gpio_set_value(spidev->aon_gpio, 1);
            }
            if (spidev->rst_gpio > 0) {
                gpio_set_value(spidev->rst_gpio, 0);
                msleep(20);
                gpio_set_value(spidev->rst_gpio, 1);
            }
        }
    } else {
        if (spidev->power) {
            spidev->power = !spidev->power;
            /* TODO power off... */
			if (spidev->wake_gpio > 0) {
				gpio_set_value(spidev->wake_gpio, 0);
            }
			if (spidev->rst_gpio > 0) {
				gpio_set_value(spidev->rst_gpio, 0);
			}
 
			if (spidev->aon_gpio > 0) {
				msleep(30);
				gpio_set_value(spidev->aon_gpio, 0);
            }



        }
    }

    return 0;
}

static int spidev_wait(struct spidev_data *spidev, unsigned int cmd, unsigned long arg)
{
    int            retval = 0;
    u32            tmp;
    unsigned long        flags;

    switch (cmd) {
    case SPI_IOC_RD_WAIT:
        retval = get_user(tmp, (__u32 __user *)arg);
        //FR_LOG("%s >>> SPI_IOC_RD_WAIT, wakeup:%u, tmp:%u, retval:%d, cnt:%d\n", __func__, spidev->wakeup, tmp, retval, cnt);
        mutex_lock(&spidev->wait_lock);
        if (spidev->irq_gpio > 0)
            retval = gpio_get_value(spidev->irq_gpio);
        spin_lock_irqsave(&spidev->irq_lock, flags);
        switch (spidev->wakeup) {
        case 0:
            if (retval) {
                spin_unlock_irqrestore(&spidev->irq_lock, flags);
            } else {
                spin_unlock_irqrestore(&spidev->irq_lock, flags);
                wait_event_interruptible(spidev->wait, spidev->wakeup);
        }
            break;
        case 1:
            spidev->wakeup = 0;
        spin_unlock_irqrestore(&spidev->irq_lock, flags);
            break;
        default:
            spin_unlock_irqrestore(&spidev->irq_lock, flags);
            wait_event_interruptible_timeout(spidev->wait, spidev->wakeup, WAIT_TIMEOUT);
            break;
        }
        mutex_unlock(&spidev->wait_lock);
        break;

    case SPI_IOC_WR_WAIT:
        retval = get_user(tmp, (__u32 __user *)arg);
        if (retval == 0) {
            //FR_LOG("%s >>> SPI_IOC_WR_WAIT, %u\n", __func__, tmp);
            mutex_lock(&spidev->wake_lock);
            spin_lock_irqsave(&spidev->irq_lock, flags);
            switch (tmp) {
            case 0:
                spidev->wakeup = 1;
                spin_unlock_irqrestore(&spidev->irq_lock, flags);
                wake_up_interruptible(&spidev->wait);
                break;
            case 1:
                if (spidev->wakeup == 1)
                    spidev->wakeup = 0;
                spin_unlock_irqrestore(&spidev->irq_lock, flags);
                break;
            default:
                spidev->wakeup = -1;
                spin_unlock_irqrestore(&spidev->irq_lock, flags);
                wake_up_interruptible(&spidev->wait);
                break;
            }
            mutex_unlock(&spidev->wake_lock);
            //FR_LOG("%s <<< SPI_IOC_WR_WAIT, %u\n", __func__, spidev->wakeup); 
        }
        break;
    }

    return retval;
}

static int spidev_reset(struct spidev_data *spidev)
{
    /* printk("%s >>> reset\n", __func__); */
    if (spidev->rst_gpio > 0) {
        gpio_set_value(spidev->rst_gpio, 0);
		msleep(1);
        gpio_set_value(spidev->rst_gpio, 1);
    }

    return 0;
}

static int
spidev_spi_readreg(struct spidev_data *spidev,
        unsigned int reg, unsigned int sub, unsigned char *data, size_t len)
{
    unsigned char *tx_buf;
    unsigned char *rx_buf;
    int ret;
    unsigned int tx_len = 0;
    struct spi_transfer t[2];
    struct spi_message m;
    tx_buf = kmalloc(20, GFP_KERNEL);
    rx_buf = kmalloc(len + 10, GFP_KERNEL);

    if ((!tx_buf) || (!rx_buf)) {
        if (tx_buf) kfree(tx_buf);
        if (rx_buf) kfree(rx_buf);
        return -1;
    }

    if (sub == 0) {
        tx_buf[tx_len++] = (0x00 | reg);
    } else {
        tx_buf[tx_len++] = (0x40 | reg);
        if (sub < 0x80) {
            tx_buf[tx_len++] = sub;
        } else {
            tx_buf[tx_len++] = (0x80 | (sub & 0x7f));
            tx_buf[tx_len++] = ((sub >> 7) & 0xff);
        }
    }
    tx_buf[tx_len++] = 0x00;
    memset(t, 0, sizeof(t));
    t[0].tx_buf = tx_buf;
    t[0].len = tx_len;
    t[0].speed_hz = spidev->speed_hz,
    t[1].rx_buf = rx_buf;
    t[1].len = len;
    t[1].speed_hz = spidev->speed_hz,
    spi_message_init(&m);
    spi_message_add_tail(&t[0], &m);
    spi_message_add_tail(&t[1], &m);

    ret =  spidev_sync(spidev, &m);
    printk("%s %d, ret is %d", __func__, __LINE__, ret);
    memcpy(data, rx_buf, len);

    kfree(tx_buf);
    kfree(rx_buf);
    return ret;
}

static int
spidev_spi_writereg(struct spidev_data *spidev,
        unsigned int reg, unsigned int sub, unsigned char *data, size_t len)
{
    unsigned char *tx_buf;
    unsigned char *tx_buf_2;
    int ret;
    unsigned int tx_len = 0;
    struct spi_transfer t[2];
    struct spi_message m;

    tx_buf = kmalloc(20, GFP_KERNEL);
    tx_buf_2 = kmalloc(len + 10, GFP_KERNEL);
    if ((!tx_buf) || (!tx_buf_2)) {
        if (tx_buf) kfree(tx_buf);
        if (tx_buf_2) kfree(tx_buf_2);
        return -1;
    }

    memcpy(tx_buf_2, data,  len);

    if (sub == 0) {
        tx_buf[tx_len++] = (0x80 | reg);
    } else {
        tx_buf[tx_len++] = (0x80 | 0x40 | reg);
        if (sub < 0x80) {
            tx_buf[tx_len++] = sub;
        } else {
            tx_buf[tx_len++] = (0x80 | (sub & 0x7f));
            tx_buf[tx_len++] = ((sub >> 7) & 0xff);
        }
    }
    memset(t, 0, sizeof(t));
    t[0].tx_buf = tx_buf;
    t[0].len = tx_len;
    t[0].speed_hz = spidev->speed_hz,
    t[1].tx_buf = tx_buf_2;
    t[1].len = len;
    t[1].speed_hz = spidev->speed_hz,
    spi_message_init(&m);
    spi_message_add_tail(&t[0], &m);
    spi_message_add_tail(&t[1], &m);
    ret = spidev_sync(spidev, &m);
    if (tx_buf) kfree(tx_buf);
    if (tx_buf_2) kfree(tx_buf_2);
    return ret;
}

static int
spidev_spi_generic_read(struct spidev_data *spidev)
{
    int ret;
    unsigned char soc_ctrl;
    unsigned char tx_buf[6] = { SPI_MSG_CMD_GENERIC_READ, 1, };

    /* Clear Tx Cmd */
    ret = spidev_spi_readreg(spidev, REG_ID_MISC_SOC_CTRL, 0, &soc_ctrl, 1);
    if (ret < 0) {
        dev_err(&spidev->spi->dev, "%s spi_readreg 1 failed\n", __func__);
        return ret;
    }
    soc_ctrl = (soc_ctrl & ~0x02) | (0 << 1);
    ret = spidev_spi_writereg(spidev, REG_ID_MISC_SOC_CTRL, 0, &soc_ctrl, 1);
    if (ret < 0) {
        dev_err(&spidev->spi->dev, "%s spi_writereg 1 failed\n", __func__);
        return ret;
    }
    /* Write data to TX Cmd Buf */
    ret = spidev_spi_writereg(spidev, REG_ID_MISC_TX_CMD_BUF, 0, tx_buf, sizeof(tx_buf));
    if (ret < 0) {
        dev_err(&spidev->spi->dev, "%s spi_writereg failed\n", __func__);
        return ret;
    }
    /* Set Tx Cmd Ready */
    ret = spidev_spi_readreg(spidev, REG_ID_MISC_SOC_CTRL, 0, &soc_ctrl, 1);
    if (ret < 0) {
        dev_err(&spidev->spi->dev, "%s spi_readreg 2 failed\n", __func__);
        return ret;
    }
    soc_ctrl = (soc_ctrl & ~0x02) | (1 << 1);
    ret = spidev_spi_writereg(spidev, REG_ID_MISC_SOC_CTRL, 0, &soc_ctrl, 1);
    if (ret < 0) {
        dev_err(&spidev->spi->dev, "%s spi_writereg 2 failed\n", __func__);
        return ret;
    }

    return 0;
}

static int
spidev_spi_generic_write(struct spidev_data *spidev,
        unsigned char *data, size_t len)
{
    int i, ret;
    unsigned char *tx_buf;
    unsigned char *tx_buf_2;
    unsigned int tx_len = 0;
    struct spi_transfer t[2];
    struct spi_message m;

    tx_buf = kmalloc(20, GFP_KERNEL);
    tx_buf_2 = kmalloc(len + 10, GFP_KERNEL);
    if ((!tx_buf) || (!tx_buf_2)) {
        if (tx_buf) kfree(tx_buf);
        if (tx_buf_2) kfree(tx_buf_2);
        return -1;
    }
    memcpy(tx_buf_2, data,  len);

    tx_buf[tx_len++] = (0x80 | REG_ID_MISC_TX_CMD_BUF);
    tx_buf[tx_len++] = SPI_MSG_CMD_UCI;
    tx_buf[tx_len++] = ((len >> 0) & 0xff);
    tx_buf[tx_len++] = ((len >> 8) & 0xff);
    tx_buf[tx_len++] = 0;
    tx_buf[tx_len++] = 0;
    for (i = 0; i < (int)len; i++) {
        /* chksum add */
        tx_buf[4] += data[i];
        /* chksum xor */
        tx_buf[5] ^= data[i];
    }
    memset(t, 0, sizeof(t));
    t[0].tx_buf = tx_buf;
    t[0].len = tx_len;
    t[0].speed_hz = spidev->speed_hz,
    t[1].tx_buf = tx_buf_2;
    t[1].len = len;
    t[1].speed_hz = spidev->speed_hz,
    spi_message_init(&m);
    spi_message_add_tail(&t[0], &m);
    spi_message_add_tail(&t[1], &m);

    ret = spidev_sync(spidev, &m);
    if (tx_buf) kfree(tx_buf);
    if (tx_buf_2) kfree(tx_buf_2);
    return ret;
}

static int
spidev_spi_read(struct spidev_data *spidev,
        unsigned char *data, size_t *len)
{
    int ret;
    unsigned char rx_buf[5];
    unsigned int rx_len;

    ret = spidev_spi_readreg(spidev, REG_ID_MISC_RX_CMD_BUF, 0, rx_buf, sizeof(rx_buf));
    if (ret < 0) {
        dev_err(&spidev->spi->dev, "%s spi_readreg 1 failed\n", __func__);
        return ret;
    }
    if (rx_buf[0] == SPI_MSG_CMD_UCI_DATA) {
        rx_len = (unsigned int)(rx_buf[1] << 0) + (unsigned int)(rx_buf[2] << 8);
        if (rx_len > *len)
            rx_len = *len;
        ret = spidev_spi_readreg(spidev, REG_ID_MISC_RX_CMD_BUF, sizeof(rx_buf), data, rx_len);
        if (ret < 0) {
            dev_err(&spidev->spi->dev, "%s spi_readreg 2 failed\n", __func__);
            return ret;
        }
        *len = rx_len;
    } else
        *len = 0;

    return (int)rx_buf[0];
}

static int
spidev_spi_write(struct spidev_data *spidev,
        unsigned char *data, size_t len)
{
    int ret;
    unsigned char soc_ctrl;

    /* Clear Tx Cmd */
    ret = spidev_spi_readreg(spidev, REG_ID_MISC_SOC_CTRL, 0, &soc_ctrl, 1);
    if (ret < 0) {
        dev_err(&spidev->spi->dev, "%s spi_readreg 1 failed\n", __func__);
        return ret;
    }
    soc_ctrl = (soc_ctrl & ~0x02) | (0 << 1);
    ret = spidev_spi_writereg(spidev, REG_ID_MISC_SOC_CTRL, 0, &soc_ctrl, 1);
    if (ret < 0) {
        dev_err(&spidev->spi->dev, "%s spi_writereg 1 failed\n", __func__);
        return ret;
    }
    /* Write data to TX Cmd Buf */
    ret = spidev_spi_generic_write(spidev, data, len);
    if (ret < 0) {
        dev_err(&spidev->spi->dev, "%s spi_generic_write failed\n", __func__);
        return ret;
    }
    /* Set Tx Cmd Ready */
    ret = spidev_spi_readreg(spidev, REG_ID_MISC_SOC_CTRL, 0, &soc_ctrl, 1);
    if (ret < 0) {
        dev_err(&spidev->spi->dev, "%s spi_readreg 2 failed\n", __func__);
        return ret;
    }
    soc_ctrl = (soc_ctrl & ~0x02) | (1 << 1);
    ret = spidev_spi_writereg(spidev, REG_ID_MISC_SOC_CTRL, 0, &soc_ctrl, 1);
    if (ret < 0) {
        dev_err(&spidev->spi->dev, "%s spi_writereg 2 failed\n", __func__);
        return ret;
    }

    return 0;
}

static int
spidev_spi_detect(struct spidev_data *spidev)
{
    int ret;
    unsigned char rx_buf[4];

    ret = spidev_spi_readreg(spidev, 0x00, 0, rx_buf, sizeof(rx_buf));
    if (ret < 0) {
        dev_err(&spidev->spi->dev, "%s spi_readreg failed\n", __func__);
        return ret;
    }

    return (rx_buf[2] == 0x01 && rx_buf[3] == 0x03) ? 0 : 1;
}

static int
spidev_spi_poll(struct spidev_data *spidev)
{
    int ret;
    unsigned char rx_buf[2];

    ret = spidev_spi_readreg(spidev, REG_ID_MISC_IRQ_STATUS, 0, rx_buf, sizeof(rx_buf));
    if (ret < 0) {
        dev_err(&spidev->spi->dev, "%s spi_readreg failed\n", __func__);
        return ret;
    }

    return ((int)rx_buf[0] << 0) | ((int)rx_buf[1] << 8);
}

static int spidev_wakeup(struct spidev_data *spidev)
{
    if (spidev->wake_gpio <= 0 || spidev->rst_gpio <= 0)
        return 0;

    gpio_set_value(spidev->rst_gpio, 0);
    msleep(1);
    gpio_set_value(spidev->wake_gpio, 1);
    msleep(3);
    gpio_set_value(spidev->wake_gpio, 0);
    msleep(20);
    gpio_set_value(spidev->rst_gpio, 1);

    /*if (spidev->wake_gpio <= 0)
        return 0;

    gpio_set_value(spidev->wake_gpio, 1);
    msleep(3);
    gpio_set_value(spidev->wake_gpio, 0);*/

    return 0;
}


static int spidev_request_gpios(struct spidev_data *spidev, struct spi_device *spi)
{
    int            status;

    spidev->aon_gpio = of_get_named_gpio(spi->dev.of_node, "aon-gpio", 0);
    if (spidev->aon_gpio > 0) {
        status = gpio_request(spidev->aon_gpio, "spidev-aon");
        if (status < 0) {
            spidev->aon_gpio = 0;
            return status;
        }
        gpio_direction_output(spidev->aon_gpio, 0);
    }
    spidev->rst_gpio = of_get_named_gpio(spi->dev.of_node, "rst-gpio", 0);
    if (spidev->rst_gpio > 0) {
        status = gpio_request(spidev->rst_gpio, "spidev-rst");
        if (status < 0) {
            spidev->rst_gpio = 0;
            return status;
        }
        gpio_direction_output(spidev->rst_gpio, 0);
    }

    spidev->wake_gpio = of_get_named_gpio(spi->dev.of_node, "wake-gpio", 0);
    if (spidev->wake_gpio > 0) {
        status = gpio_request(spidev->wake_gpio, "spidev-wake");
        if (status < 0) {
            spidev->wake_gpio = 0;
            return status;
        }
        gpio_direction_output(spidev->wake_gpio, 0);
    }

    spidev->irq_gpio = of_get_named_gpio(spi->dev.of_node, "irq-gpio", 0);
    if (spidev->irq_gpio > 0) {
        status = gpio_request(spidev->irq_gpio, "spidev-irq");
        if (status < 0) {
            spidev->irq_gpio = 0;
            return status;
        }
        gpio_direction_input(spidev->irq_gpio);
    }

    dev_info(&spi->dev, "aon-gpio=%d\n", spidev->aon_gpio);

    dev_info(&spi->dev, "rst-gpio=%d\n", spidev->rst_gpio);
    dev_info(&spi->dev, "wake-gpio=%d\n", spidev->wake_gpio);
    dev_info(&spi->dev, "irq-gpio=%d\n", spidev->irq_gpio);

    return 0;
}

static int spidev_release_gpios(struct spidev_data *spidev)
{
    if (spidev->irq > 0) {
        free_irq(spidev->irq, spidev);
        spidev->irq = 0;
    }
    if (spidev->irq_gpio > 0) {
        gpio_free(spidev->irq_gpio);
        spidev->irq_gpio = 0;
    }
    if (spidev->wake_gpio > 0) {
        gpio_free(spidev->wake_gpio);
        spidev->wake_gpio = 0;
    }
    if (spidev->rst_gpio > 0) {
        gpio_free(spidev->rst_gpio);
        spidev->rst_gpio = 0;
    }

    if (spidev->aon_gpio > 0) {
        gpio_free(spidev->aon_gpio);
        spidev->aon_gpio = 0;
    }

    return 0;
}

static ssize_t spidev_show_power(struct device *ddri, struct device_attribute *attr, char *buf)
{
    ssize_t length = 0;

    FR_LOG("power:%d\n", g_spidev->power);
    length = sprintf(buf, "power=%d\n", g_spidev->power);
    return length;
}
static ssize_t spidev_store_power(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    char *next;
    u8 power_flag  = simple_strtoul(buf, &next, 10);
    
    FR_LOG("power_flag %d\n", power_flag);
    if(power_flag) {
       spidev_power(g_spidev, 1);
    } else {
       spidev_power(g_spidev, 0);
    }

    return size;
}
static DEVICE_ATTR(powermode, 0664, spidev_show_power, spidev_store_power);

static ssize_t spidev_show_reset(struct device *ddri, struct device_attribute *attr, char *buf)
{
    u32 pin_val = -1;
    pin_val = gpio_get_value(g_spidev->rst_gpio);
    FR_LOG("reset pin_val=%d\n", pin_val);

    return sprintf(buf, "reset pin_val=%d\n", pin_val);
}
static ssize_t spidev_store_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    char *next;
    u8 reset_flag  = simple_strtoul(buf, &next, 10);
    FR_LOG("reset_flag %d\n", reset_flag);
    if(reset_flag) {
       gpio_set_value(g_spidev->rst_gpio, 1);
    } else {
       gpio_set_value(g_spidev->rst_gpio, 0);
    }

    return size;
}
static DEVICE_ATTR(reset, 0664, spidev_show_reset, spidev_store_reset);

static ssize_t spidev_show_irq(struct device *ddri, struct device_attribute *attr, char *buf)
{
    u32 pin_val = -1;
    pin_val = gpio_get_value(g_spidev->irq_gpio);
    FR_LOG("irq pin_val=%d\n", pin_val);

    return sprintf(buf, "irq pin_val=%d\n", pin_val);
}
static ssize_t spidev_store_irq(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    FR_LOG("spidev_store_irq\n");
    return size;
}
static DEVICE_ATTR(irq, 0664, spidev_show_irq, spidev_store_irq);

static ssize_t spidev_show_wake(struct device *ddri, struct device_attribute *attr, char *buf)
{
    int ret = 0;

    ret = spidev_spi_detect(g_spidev);
    FR_LOG("wake:%d\n", ret);
    return sprintf(buf, "wake=%d\n", ret);
}
static ssize_t spidev_store_wake(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    char *next;
    u8 wake_flag  = simple_strtoul(buf, &next, 10);
    FR_LOG("wake_flag %d\n", wake_flag);
    if(wake_flag) {
       spidev_wakeup(g_spidev);
    }

    return size;
}
static DEVICE_ATTR(wake, 0664, spidev_show_wake, spidev_store_wake);

static unsigned char readreg_buf[600] = {0};
static ssize_t spidev_show_readreg(struct device *ddri, struct device_attribute *attr, char *buf)
{
    ssize_t length = 0;

    FR_LOG("%s\n", readreg_buf);
    length = sprintf(buf, "%s\n", readreg_buf);
    return length;
}
static ssize_t spidev_store_readreg(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned int reg = 0;
    unsigned int sub_reg = 0;
    unsigned int len = 0;
    static unsigned char  __attribute__ ((aligned(4))) readreg_data[MAX_DATA] = {0};
    static unsigned char  __attribute__ ((aligned(4))) readreg_buffer[520] = {0};
    unsigned int i = 0;
    
    memset(readreg_buf, 0, sizeof(readreg_buf));
    if(3 == sscanf(buf, "%x %d %d", &reg, &sub_reg, &len) && len>0 && len<256){
        if (spidev_spi_readreg(g_spidev, reg, sub_reg, readreg_data, len) < 0) {
            sprintf(readreg_buf, "spidev_spi_readreg failed reg=%x, sub_reg=%d, len=%d\n", reg, sub_reg, len);
            FR_LOG("%s\n", readreg_buf);
            return size;
        }

        for (i = 0; i < len; i++) {
            sprintf(readreg_buffer+i*2, "%02X", readreg_data[i]);
        }
        sprintf(readreg_buf, "spidev_spi_readreg success reg=%x, sub_reg=%d, len=%d, data=%s\n", reg, sub_reg, len, readreg_buffer);
    }else{
        sprintf(readreg_buf, "%s\n", "params failed");
    }

    FR_LOG("%s\n", readreg_buf);
    return size;
}
static DEVICE_ATTR(readreg, 0664, spidev_show_readreg, spidev_store_readreg);

static ssize_t spidev_show_writereg(struct device *ddri, struct device_attribute *attr, char *buf)
{
    ssize_t length = 0;

    FR_LOG("%s\n", readreg_buf);
    length = sprintf(buf, "%s\n", readreg_buf);
    return length;
}
static ssize_t spidev_store_writereg(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned int reg = 0;
    unsigned int sub_reg = 0;
    unsigned int len = 0;
    unsigned char  __attribute__ ((aligned(4))) readreg_data[MAX_DATA] = {0}; 
    unsigned char __attribute__ ((aligned(4))) readreg_buffer[520] = {0};
    unsigned int i = 0;

    memset(readreg_buf, 0, sizeof(readreg_buf));
    if(3 == sscanf(buf, "%x %d %s", &reg, &sub_reg, readreg_buffer) && (strlen(readreg_buffer)>0) && (strlen(readreg_buffer)<513) && (strlen(readreg_buffer)%2==0)){
        len = strlen(readreg_buffer)/2;
        for (i = 0; i < len; i++) {
            //sprintf(readreg_buffer+i*2, "%02X", readreg_data[i]);
            if(1 != sscanf(readreg_buffer+i*2, "%02X", (unsigned int*)(readreg_data+i))){
                sprintf(readreg_buf, "%s\n", "params failed");
                FR_LOG("%s\n", readreg_buf);
                    return size;
            }
        }

        if (spidev_spi_writereg(g_spidev, reg, sub_reg, readreg_data, len) < 0) {
            sprintf(readreg_buf, "spidev_spi_writereg failed reg=%x, sub_reg=%d, len=%d\n", reg, sub_reg, len);
            FR_LOG("%s\n", readreg_buf);
            return size;
        }

        memset(readreg_data, 0, sizeof(readreg_data));
        if (spidev_spi_readreg(g_spidev, reg, sub_reg, readreg_data, len) < 0) {
            sprintf(readreg_buf, "spidev_spi_readreg failed reg=%x, sub_reg=%d, len=%d\n", reg, sub_reg, len);
            FR_LOG("%s\n", readreg_buf);
            return size;
        }

        for (i = 0; i < len; i++) {
            sprintf(readreg_buffer+i*2, "%02X", readreg_data[i]);
        }
        sprintf(readreg_buf, "spidev_spi_readreg success reg=%x, sub_reg=%d, len=%d, data=%s\n", reg, sub_reg, len, readreg_buffer);
    }else{
        sprintf(readreg_buf, "%s\n", "params failed");
    }

    FR_LOG("%s\n", readreg_buf);
    return size;
}
static DEVICE_ATTR(writereg, 0664, spidev_show_writereg, spidev_store_writereg);

static ssize_t spidev_show_spiread(struct device *ddri, struct device_attribute *attr, char *buf)
{
    ssize_t length = 0;
    int ret = 0;
    int value = 0;
    unsigned char data[MAX_DATA];
    size_t len = sizeof(data);
    unsigned int i = 0;

    value = spidev_spi_poll(g_spidev);
    if (value < 0) {
        FR_LOG("%s spi_poll failed\n", __func__);
        length = sprintf(buf, "%s spi_poll failed\n", __func__);
        return length;
    }
    

    if (value & SYS_IRQ_CMD_2_HOST_READY) {
        ret = spidev_spi_read(g_spidev, data, &len);
        if (ret < 0) {
            FR_LOG("%s spi_read failed\n", __func__);
            length = sprintf(buf, "%s spi_read failed\n", __func__);
            return length;
        }

        for (i = 0; i < len; i++) {
            sprintf(buf+i*2, "%02X", data[i]);
        }
        length = 2*len;
        length += sprintf(buf+length, "%s", "\n");
    }
    if (value & SYS_IRQ_MCU_DATA_IND_READY) {
        ret = spidev_spi_generic_read(g_spidev);
        if (ret < 0) {
            length += sprintf(buf+length, "%s spi_poll failed\n", __func__);
            FR_LOG("%s\n", buf);
            return length;
        }
        length += sprintf(buf+length, "%s\n", "spi_generic_read");
    }
    if (value & SYS_IRQ_MCU_HALT) {
        length += sprintf(buf+length, "%s\n", "SYS_IRQ_MCU_HALT");
    }

    FR_LOG("%s\n", buf);
    return length;
}
static ssize_t spidev_store_spiread(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    FR_LOG("spidev_store_spiread\n");
    return size;
}
static DEVICE_ATTR(spiread, 0664, spidev_show_spiread, spidev_store_spiread);

static ssize_t spidev_show_spiwrite(struct device *ddri, struct device_attribute *attr, char *buf)
{
    ssize_t length = 0;

    FR_LOG("%s\n", readreg_buf);
    length = sprintf(buf, "%s\n", readreg_buf);
    return length;
}
static ssize_t spidev_store_spiwrite(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned int len = 0;
    unsigned char __attribute__ ((aligned(4))) readreg_data[MAX_DATA] = {0};
    unsigned char __attribute__ ((aligned(4))) readreg_buffer[520] = {0};
    unsigned int i = 0;

    memset(readreg_buf, 0, sizeof(readreg_buf));
    if(1 == sscanf(buf, "%s", readreg_buffer) && (strlen(readreg_buffer)>0) && (strlen(readreg_buffer)<513) && (strlen(readreg_buffer)%2==0)){
        len = strlen(readreg_buffer)/2;
        for (i = 0; i < len; i++) {
            //sprintf(readreg_buffer+i*2, "%02X", readreg_data[i]);
            if(1 != sscanf(readreg_buffer+i*2, "%02X", (unsigned int*)(readreg_data+i))){
                sprintf(readreg_buf, "%s\n", "params failed");
                FR_LOG("%s\n", readreg_buf);
                    return size;
            }
        }

        if (spidev_spi_write(g_spidev, readreg_data, len) < 0) {
            sprintf(readreg_buf, "spidev_spi_write failed len=%d\n", len);
            FR_LOG("%s\n", readreg_buf);
            return size;
        }

        sprintf(readreg_buf, "spidev_spi_readreg success len=%d, data=%s\n", len, readreg_buffer);
    }else{
        sprintf(readreg_buf, "%s\n", "params failed");
    }

    FR_LOG("%s\n", readreg_buf);    
    return size;
}
static DEVICE_ATTR(spiwrite, 0664, spidev_show_spiwrite, spidev_store_spiwrite);

static ssize_t spidev_show_aon(struct device *ddri, struct device_attribute *attr, char *buf)
{
    u32 pin_val = -1;
    pin_val = gpio_get_value(g_spidev->aon_gpio);
    FR_LOG("aon pin_val=%d\n", pin_val);

    return sprintf(buf, "aon pin_val=%d\n", pin_val);
}
static ssize_t spidev_store_aon(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    char *next;
    u8 aon_flag  = simple_strtoul(buf, &next, 10);
    FR_LOG("aon_flag %d\n", aon_flag);
    if(aon_flag) {
       gpio_set_value(g_spidev->aon_gpio, 1);
    } else {
       gpio_set_value(g_spidev->aon_gpio, 0);
    }

    return size;
}
static DEVICE_ATTR(aon, 0664, spidev_show_aon, spidev_store_aon);

static int spidev_sysfs_init(struct spidev_data *spidev)
{
    int ret = 0;

    ret = sysfs_create_file(&spidev->device->kobj, &dev_attr_powermode.attr);
    ret = sysfs_create_file(&spidev->device->kobj, &dev_attr_reset.attr);
    ret = sysfs_create_file(&spidev->device->kobj, &dev_attr_irq.attr);
    ret = sysfs_create_file(&spidev->device->kobj, &dev_attr_wake.attr);
    ret = sysfs_create_file(&spidev->device->kobj, &dev_attr_readreg.attr);
    ret = sysfs_create_file(&spidev->device->kobj, &dev_attr_writereg.attr);
    ret = sysfs_create_file(&spidev->device->kobj, &dev_attr_spiread.attr);
    ret = sysfs_create_file(&spidev->device->kobj, &dev_attr_spiwrite.attr);
    ret = sysfs_create_file(&spidev->device->kobj, &dev_attr_aon.attr);

    return ret;
}

static int spidev_free(struct spidev_data *spidev)
{
    spidev_release_gpios(spidev);
    /* prevent new opens */
    mutex_lock(&device_list_lock);
    /* make sure ops on existing fds can abort cleanly */
    spin_lock_irq(&spidev->spi_lock);
    spidev->spi = NULL;
    spin_unlock_irq(&spidev->spi_lock);

    list_del(&spidev->device_entry);
    device_destroy(spidev_class, spidev->devt);
    cdev_del(&spidev->cdev);
    if (spidev->users == 0)
        kfree(spidev);
    mutex_unlock(&device_list_lock);

    return 0;
}

long int delayMs1=2;
long int delayMs2=2;
long int delayMs3=4;
long int delayMs4=12;
long int testCount=3000;
int testAonAlwaysOn = 0;
int testDigAlwaysOn = 0;
static ssize_t q845gpio_show(struct device *dev,
                      struct device_attribute *attr, char *buf)
{
	int i;
    printk("q845gpio_show testCount=%ld, delayMs1=%ld, delayMs2=%ld, delayMs3=%ld, delayMs4=%ld \n",testCount,delayMs1,delayMs2,delayMs3,delayMs4);
	for (i = 0; i < testCount; ++i) {
		gpio_set_value(411, 1);//412:dig, 411:aon
		mdelay(delayMs1);
		gpio_set_value(412, 1);
        if (!testDigAlwaysOn) {
            mdelay(delayMs2);
            gpio_set_value(412, 0);
        }
        if (!testAonAlwaysOn) {
            mdelay(delayMs3);
            gpio_set_value(411, 0);
        }
		mdelay(delayMs4);
	}
    return 0;
}
#if 1
static ssize_t q845gpio_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	char *end;
	char temp[100];
	long int num[100];
	int i;
	int j = 0,k=0;
    printk("q845gpio_store buf=%s, count=%zu",buf,count);
	memset(temp,0,sizeof(temp));
    if (sizeof(buf) >= 3) {
		for (i = 0; i < count; ++i) {
			if ((buf[i] != ' ') && (i != count-1)) {
				temp[j++] = buf[i];
			} else {
				num[k] = simple_strtol(temp, &end, 0);
				printk("q845gpio_store test temp=%s, num=%ld...\n",temp,num[k]);
				memset(temp,0,sizeof(temp));
				j = 0;
				k++;
			}
		}
		delayMs1 = num[0];
		delayMs2 = num[1];
		delayMs3 = num[2];
		delayMs4 = num[3];
		testCount = num[4];
		printk("q845gpio_store testCount=%ld, delayMs1=%ld, delayMs2=%ld, delayMs3=%ld, delayMs4=%ld \n",testCount,delayMs1,delayMs2,delayMs3,delayMs4);
        if (k > 6) {
            testAonAlwaysOn = num[5];
            testDigAlwaysOn = num[6];
            printk("q845gpio_store testAonAlwaysOn=%d, testDigAlwaysOn=%d\n", testAonAlwaysOn ,testDigAlwaysOn);
        }
    }
    return count;
}
#else
static ssize_t q845gpio_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	char *end;
	long int gpio_num;
	long int gpio_level;
	long int delayMs;
	char temp[100];
	long int num[100];
	int i;
	int j = 0,k=0;
    printk("buf=%s, count=%d",buf,count);
	memset(temp,0,sizeof(temp));
    if (sizeof(buf) >= 3) {
		for (i = 0; i < count; ++i) {
			if ((buf[i] != ' ') && (i != count-1)) {
				temp[j++] = buf[i];
			} else {
				num[k] = simple_strtol(temp, &end, 0);
				printk("test temp=%s, num=%ld...\n",temp,num[k]);
				memset(temp,0,sizeof(temp));
				j = 0;
				k++;
			}
		}
		gpio_num = num[0];
		gpio_level = num[1];
		delayMs = num[2];
        printk("test gpio_num=%ld, gpio_level=%ld, delayMs=%ld...",gpio_num,gpio_level,delayMs);
        gpio_set_value(361+gpio_num, gpio_level);
        msleep(delayMs);
    }
    return count;
}
#endif
static DEVICE_ATTR(q845gpio_rw, 0664, q845gpio_show, q845gpio_store);
static struct attribute *q845gpio_sysfs_entries[]={
	&dev_attr_q845gpio_rw.attr,
	NULL,
};
static struct attribute_group q845gpio_attr_group={
	.attrs = q845gpio_sysfs_entries,
};
/*-------------------------------------------------------------------------*/

static int spidev_probe(struct spi_device *spi)
{
    struct spidev_data    *spidev;
    int            status;
    unsigned long   flags;
	int ret_temp = 0;

    /*
     * spidev should never be referenced in DT without a specific
     * compatible string, it is a Linux implementation thing
     * rather than a description of the hardware.
     */
    WARN(spi->dev.of_node &&
         of_device_is_compatible(spi->dev.of_node, DEV_NAME),
         "%pOF: buggy DT: spidev listed directly in DT\n", spi->dev.of_node);

    /* Allocate driver data */
    spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
    if (!spidev)
        return -ENOMEM;

    /* Initialize the driver data */
    g_spidev = spidev;
    spidev->spi = spi;
    spin_lock_init(&spidev->spi_lock);
    spin_lock_init(&spidev->irq_lock);
    init_waitqueue_head(&spidev->wait);
    mutex_init(&spidev->buf_lock);
    mutex_init(&spidev->wait_lock);
    mutex_init(&spidev->wake_lock);

    INIT_LIST_HEAD(&spidev->device_entry);

    status = spidev_request_gpios(spidev, spi);
    if (status < 0) {
        dev_err(&spi->dev, "cann't request gpios\n");
        kfree(spidev);
        return status;
    }

    /*spidev_power(spidev, 1);
    if (spidev_spi_detect(spidev) != 0) {
        FR_LOG("spidev_spi_detect failed\n");
        spidev_power(spidev, 0);
        spidev_free(spidev);
        return -EINVAL;
        
    }
    spidev_power(spidev, 0);*/

    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
     */
    mutex_lock(&device_list_lock);
    if (spidev_major) {
        spidev->devt = MKDEV(spidev_major, 0);
        status = register_chrdev_region(spidev->devt, 1, DEV_NAME);
    } else {
        status = alloc_chrdev_region(&spidev->devt, 0, 1, DEV_NAME);
        spidev_major = MAJOR(spidev->devt);
    }
    if (status == 0) {
        cdev_init(&spidev->cdev, &spidev_fops);
        status = cdev_add(&spidev->cdev, spidev->devt, 1);
        if (status == 0) {
            spidev->device = device_create(spidev_class,
                            NULL, spidev->devt, NULL, DEV_NAME);
            if (IS_ERR(spidev->device)) {
                cdev_del(&spidev->cdev);
                unregister_chrdev_region(spidev->devt, 1);
                dev_err(&spi->dev, "create device failed\n");
                status = PTR_ERR(spidev->device);
            }
        } else {
            unregister_chrdev_region(spidev->devt, 1);
            dev_err(&spi->dev, "add cdev failed\n");
        }
    } else {
        dev_err(&spi->dev, "register chrdev region failed\n");
    }
    if (status == 0) {
        list_add(&spidev->device_entry, &device_list);
    }
    mutex_unlock(&device_list_lock);

    spidev->speed_hz = spi->max_speed_hz;

    if (status == 0) {
        spidev->irq = gpio_to_irq(spidev->irq_gpio);
        if (spidev->irq < 0) {
            dev_err(&spi->dev, "gpio to_irq failed %d<=>%d\n", spidev->irq_gpio, spidev->irq);
            status = spidev->irq;
        } else {
            spin_lock_irqsave(&spidev->irq_lock, flags);
            spidev->wakeup = 0;
            spin_unlock_irqrestore(&spidev->irq_lock, flags);
            status = request_irq(spidev->irq, spidev_interrupt,
                            IRQF_TRIGGER_RISING/*IRQF_TRIGGER_HIGH*/, DEV_NAME, spidev);
            if (status < 0) {
                dev_err(&spi->dev, "request_irq %d failed!\n", spidev->irq);
                spidev->irq = 0;
            }
        }
    }
    if (status == 0)
        spi_set_drvdata(spi, spidev);
    else {
        spidev_free(spidev);
        spidev = NULL;
    }
    status = spidev_sysfs_init(spidev);
    if(status) {
        dev_err(&spi->dev, "spidev_sysfs_init failed\n");
    }
	ret_temp = sysfs_create_group(&spi->dev.kobj, &q845gpio_attr_group);
	printk("##spidev_probe q845gpio_attr_group ret_temp=%d, spidev is %p",ret_temp, spidev);
    return status;
}

static int spidev_remove(struct spi_device *spi)
{
    struct spidev_data    *spidev = spi_get_drvdata(spi);
    spidev_free(spidev);

    return 0;
}

static const struct spi_device_id spidev_spi_ids[] = {
    { .name = "fr8000" },
    {},
};
MODULE_DEVICE_TABLE(spi, spidev_spi_ids);

static const struct of_device_id spidev_of_ids[] = {
    { .compatible = "giantsemi,fr8000" },
    {},
};
MODULE_DEVICE_TABLE(of, spidev_of_ids);

static struct spi_driver spidev_spi_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = DEV_NAME,
        .of_match_table = of_match_ptr(spidev_of_ids),
    },
    .probe =    spidev_probe,
    .remove =    spidev_remove,
    .id_table =    spidev_spi_ids,

    /* NOTE:  suspend/resume methods are not necessary here.
     * We don't do anything except pass the requests to/from
     * the underlying controller.  The refrigerator handles
     * most issues; the controller driver handles the rest.
     */
};

/*-------------------------------------------------------------------------*/

static int __init spidev_init(void)
{
    int status;

    spidev_class = class_create(THIS_MODULE, DEV_NAME);
    if (IS_ERR(spidev_class))
        return PTR_ERR(spidev_class);

    status = spi_register_driver(&spidev_spi_driver);
    if (status < 0)
        class_destroy(spidev_class);
    return status;
}
module_init(spidev_init);

static void __exit spidev_exit(void)
{
    spi_unregister_driver(&spidev_spi_driver);
    class_destroy(spidev_class);
}
module_exit(spidev_exit);

/*-------------------------------------------------------------------------*/

MODULE_AUTHOR("fr8000, <fr8000@giantsemi.com>");
MODULE_DESCRIPTION("giantsemi FR8000 UWB SPI driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:fr8000");
