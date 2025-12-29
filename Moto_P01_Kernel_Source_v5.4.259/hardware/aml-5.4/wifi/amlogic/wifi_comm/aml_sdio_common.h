#ifndef SDIO_COMMON_H
#define SDIO_COMMON_H

#include <linux/mmc/sdio_func.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>    /* udelay */
#include <linux/spinlock.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/moduleparam.h>
#include <linux/irqreturn.h>
#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h> /* printk() */
#include <linux/list.h>
#include <linux/netdevice.h>
#include <linux/version.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/gpio.h> //mach
#include <linux/timer.h>
#include <linux/string.h>
#include <linux/firmware.h>


#define OS_LOCK spinlock_t

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


//#define OS_LOCK spinlock_t

#define SDIO_ADDR_MASK (128 * 1024 - 1)
#define SDIO_OPMODE_INCREMENT 1
#define SDIO_OPMODE_FIXED 0

#define SDIO_WRITE 1
#define SDIO_READ 0

#define SDIOH_API_RC_SUCCESS (0x00)
#define SDIOH_API_RC_FAIL (0x01)

#define FUNCNUM_SDIO_LAST SDIO_FUNC7
#define SDIO_FUNCNUM_MAX (FUNCNUM_SDIO_LAST+1)

#define ZMALLOC(size, name, gfp) kzalloc(size, gfp)
#define FREE(a, name) kfree(a)


/*sdio max block count when we use scatter/gather list.*/

//#define MAX(a, b) ((a) > (b) ? (a) : (b))
//#define MIN(a, b) ((a) < (b) ? (a) : (b))

//typedef unsigned long SYS_TYPE;

enum SDIO_STD_FUNNUM {
    SDIO_FUNC0=0,
    SDIO_FUNC1,
    SDIO_FUNC2,
    SDIO_FUNC3,
    SDIO_FUNC4,
    SDIO_FUNC5,
    SDIO_FUNC6,
    SDIO_FUNC7,
};

struct aml_hwif_sdio {
    struct sdio_func * sdio_func_if[SDIO_FUNCNUM_MAX];
    bool scatter_enabled;

    /* protects access to scat_req */
    OS_LOCK scat_lock;

    /* scatter request list head */
    struct amlw_hif_scatter_req *scat_req;
};
int aml_wifi_sdio_insmod(void);
void aml_wifi_sdio_rmmod(void);

#endif
