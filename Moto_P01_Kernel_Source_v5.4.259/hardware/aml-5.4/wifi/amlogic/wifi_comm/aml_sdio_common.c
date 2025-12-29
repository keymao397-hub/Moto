#include <linux/mutex.h>
#include "aml_sdio_common.h"
#include "aml_wifi_bus.h"
#include <linux/delay.h>

#define W2_PRODUCT_AMLOGIC  0x8888
#define W2_VENDOR_AMLOGIC  0x8888
//sdio manufacturer code, usually vendor ID, 'a'=0x61, 'm'=0x6d
#define W2_VENDOR_AMLOGIC_EFUSE ('a'|('m'<<8))
//sdio manufacturer info, usually product ID
#define W2_PRODUCT_AMLOGIC_EFUSE (0x9007)
#define W2s_VENDOR_AMLOGIC_EFUSE 0x1B8E
#define W2s_A_PRODUCT_AMLOGIC_EFUSE 0x0600
#define W2s_B_PRODUCT_AMLOGIC_EFUSE 0x0640

#define W1_PRODUCT_AMLOGIC  0x8888
#define W1_VENDOR_AMLOGIC  0x8888
#define W1u_VENDOR_AMLOGIC_EFUSE  0x1B8E
#define W1us_C_PRODUCT_AMLOGIC_EFUSE  0x0540
#define W1us_B_PRODUCT_AMLOGIC_EFUSE  0x0500
#define W1us_A_PRODUCT_AMLOGIC_EFUSE  0x04C0
#define W1us_PRODUCT_AMLOGIC_EFUSE  0x0440
//sdio manufacturer code, usually vendor ID, 'a'=0x61, 'm'=0x6d
#define W1_VENDOR_AMLOGIC_EFUSE ('a'|('m'<<8))
//sdio manufacturer info, usually product ID
#define W1_PRODUCT_AMLOGIC_EFUSE (0x9007)

struct aml_hwif_sdio g_wifi_sdio;

struct sdio_func *aml_priv_to_sdio_func(int func_n)
{
    ASSERT(func_n >= 0 &&  func_n < SDIO_FUNCNUM_MAX);
    return g_wifi_sdio.sdio_func_if[func_n];
}

int wifi_sdio_request_buffer(unsigned char func_num,
    unsigned int fix_incr, unsigned char write, unsigned int addr, void *buf, unsigned int nbytes)
{
    int err_ret = 0;
    int align_nbytes = nbytes;
    struct sdio_func * func = aml_priv_to_sdio_func(func_num);
    bool fifo = (fix_incr == SDIO_OPMODE_FIXED);

    if (!func) {
        printk("func is NULL!\n");
        return -1;
    }

    ASSERT(fix_incr == SDIO_OPMODE_FIXED|| fix_incr == SDIO_OPMODE_INCREMENT);
    ASSERT(func->num == func_num);

    /* Claim host controller */
    sdio_claim_host(func);

    if (write && !fifo)
    {
        /* write, increment */
        align_nbytes = sdio_align_size(func, nbytes);
        err_ret = sdio_memcpy_toio(func, addr, buf, align_nbytes);
    }
    else if (write)
    {
        /* write, fifo */
        err_ret = sdio_writesb(func, addr, buf, align_nbytes);
    }
    else if (fifo)
    {
        /* read */
        err_ret = sdio_readsb(func, buf, addr, align_nbytes);
    }
    else
    {
        /* read */
        align_nbytes = sdio_align_size(func, nbytes);
        err_ret = sdio_memcpy_fromio(func, buf, addr, align_nbytes);
    }

    /* Release host controller */
    sdio_release_host(func);

    return (err_ret == 0) ? SDIOH_API_RC_SUCCESS : SDIOH_API_RC_FAIL;
}

unsigned int aml_sdio_read_chip_type_reg(unsigned int addr)
{
    int result;
    int w_len = sizeof(unsigned long);
    int r_len = sizeof(unsigned int);
    void *kmalloc_buf;
    unsigned char *r_kmalloc_buf = NULL;
    unsigned int w_addr, r_addr;
    unsigned int regdata = 0;
    unsigned char*buf;
    unsigned long sram_data;

    kmalloc_buf =  (unsigned char *)ZMALLOC(w_len, "sdio_write", GFP_DMA);
    r_kmalloc_buf = (unsigned char *)ZMALLOC(r_len, "sdio_read", GFP_DMA);
    if (kmalloc_buf == NULL || r_kmalloc_buf == NULL) {
        ERROR_DEBUG_OUT("kmalloc buf fail\n");
        return 0;
    }

    w_addr = RG_SCFG_FUNC2_BADDR_A & SDIO_ADDR_MASK;
    sram_data = ((unsigned long )(addr) & 0xfffe0000);
    memcpy(kmalloc_buf, &sram_data, w_len);
    result = wifi_sdio_request_buffer(SDIO_FUNC1, SDIO_OPMODE_INCREMENT, SDIO_WRITE, w_addr, kmalloc_buf, w_len);
    if (result < 0) {
        ERROR_DEBUG_OUT("_aml_sdio_request_buffer fail");
        return 0;
    }

    r_addr = (unsigned long )(addr) & SDIO_ADDR_MASK;
    buf = (unsigned char*)(&regdata);
    result = wifi_sdio_request_buffer(SDIO_FUNC2, SDIO_OPMODE_FIXED, SDIO_READ, r_addr, r_kmalloc_buf, r_len);
    if (result < 0) {
        ERROR_DEBUG_OUT("_aml_sdio_request_buffer fail");
        return 0;
    }

    if (r_kmalloc_buf != buf) {
        memcpy(buf, r_kmalloc_buf, r_len);
        FREE(r_kmalloc_buf, "sdio_read");
    }

    FREE(kmalloc_buf, "sdio_write");

    printk("%s: ********get chipid:0x%08x \n", __func__, regdata);
    return regdata;
}

int aml_wifi_sdio_probe(struct sdio_func *func, const struct sdio_device_id *id)
{
    int ret = 0;
    static struct sdio_func sdio_func_0;
    unsigned int chip1_id = 0, loop_cnt = 0;

    printk("%s: %d, sdio probe product id 0x%x\n", __func__, __LINE__, id->device);
    if (id->device == W2s_A_PRODUCT_AMLOGIC_EFUSE
        || id->device == W2s_B_PRODUCT_AMLOGIC_EFUSE) {
        aml_wifi_chip("aml_w2_s");
        printk("%s: ********get chip type: aml_w2_s \n", __func__);
        return ret;
    } else if (id->device == W1us_PRODUCT_AMLOGIC_EFUSE
        || id->device == W1us_A_PRODUCT_AMLOGIC_EFUSE
        || id->device == W1us_B_PRODUCT_AMLOGIC_EFUSE
        || id->device == W1us_C_PRODUCT_AMLOGIC_EFUSE) {
        aml_wifi_chip("aml_w1u_s");
        printk("%s: ********get chip type: aml_w1u_s \n", __func__);
        return ret;
    } else if (id->device == W1_PRODUCT_AMLOGIC_EFUSE) {
        aml_wifi_chip("aml_w1");
        printk("%s: ********get chip type: aml_w1 \n", __func__);
        return ret;
    }
    /* else if (id->device == W2_PRODUCT_AMLOGIC || id->device == W1_PRODUCT_AMLOGIC) */
    sdio_claim_host(func);
    ret = sdio_enable_func(func);
    if (ret)
        goto sdio_enable_error;

    if (func->num == 4)
        sdio_set_block_size(func, 512);
    else
        sdio_set_block_size(func, 512);

    printk("%s(%d): func->num %d sdio block size=%d, \n", __func__, __LINE__,
        func->num,  func->cur_blksize);

    if (func->num == 1)
    {
        sdio_func_0.num = 0;
        sdio_func_0.card = func->card;
        g_wifi_sdio.sdio_func_if[0] = &sdio_func_0;
    }
    g_wifi_sdio.sdio_func_if[func->num] = func;
    printk("%s(%d): func->num %d sdio_func=%p, \n", __func__, __LINE__,
        func->num,  func);

    sdio_release_host(func);
    sdio_set_drvdata(func, (void *)(&g_wifi_sdio));
    if (func->num != FUNCNUM_SDIO_LAST)
    {
        printk("%s(%d):func_num=%d, last func num=%d\n", __func__, __LINE__,
            func->num, FUNCNUM_SDIO_LAST);
        return 0;
    }
    printk("%s: %d, sdio probe success\n", __func__, __LINE__);
sdio_chip_id:
    msleep(50);
    chip1_id = aml_sdio_read_chip_type_reg(WIFI_CHIP2_TYPE_ADDR);

    if (chip1_id == WIFI_CHIP_TYPE_W2) {
        aml_wifi_chip("aml_w2_s");
        printk("%s: ********get chip type: aml_w2_s \n", __func__);
    } else if (chip1_id == WIFI_CHIP_TYPE_W1U) {
        aml_wifi_chip("aml_w1u_s");
        printk("%s: ********get chip type: aml_w1u_s \n", __func__);
    } else if (chip1_id == WIFI_CHIP_TYPE_W1) {
        aml_wifi_chip("aml_w1");
        printk("%s: ********get chip type: aml_w1 \n", __func__);
    } else {
        printk("%s: wifi chip id check failed\n", __func__);
        if (loop_cnt++ < 20)
            goto sdio_chip_id;
    }

    return ret;

sdio_enable_error:
    printk("sdio_enable_error:  line %d\n",__LINE__);
    sdio_release_host(func);

    return ret;
}

static void  aml_wifi_sdio_remove(struct sdio_func *func)
{
    if (func== NULL)
    {
        return ;
    }

    printk("\n==========================================\n");
    printk("aml_sdio_remove++ func->num =%d \n",func->num);
    printk("==========================================\n");

    sdio_claim_host(func);
    sdio_disable_func(func);
    sdio_release_host(func);
}


static const struct sdio_device_id aml_wifi_sdio[] =
{
    {SDIO_DEVICE(W2_VENDOR_AMLOGIC,W2_PRODUCT_AMLOGIC) },
    {SDIO_DEVICE(W2_VENDOR_AMLOGIC_EFUSE,W2_PRODUCT_AMLOGIC_EFUSE)},
    {SDIO_DEVICE(W2s_VENDOR_AMLOGIC_EFUSE,W2s_A_PRODUCT_AMLOGIC_EFUSE)},
    {SDIO_DEVICE(W2s_VENDOR_AMLOGIC_EFUSE,W2s_B_PRODUCT_AMLOGIC_EFUSE)},
    {SDIO_DEVICE(W1_VENDOR_AMLOGIC,W1_PRODUCT_AMLOGIC) },
    {SDIO_DEVICE(W1_VENDOR_AMLOGIC_EFUSE,W1_PRODUCT_AMLOGIC_EFUSE)},
    {SDIO_DEVICE(W1u_VENDOR_AMLOGIC_EFUSE,W1us_PRODUCT_AMLOGIC_EFUSE)},
    {SDIO_DEVICE(W1u_VENDOR_AMLOGIC_EFUSE,W1us_A_PRODUCT_AMLOGIC_EFUSE)},
    {SDIO_DEVICE(W1u_VENDOR_AMLOGIC_EFUSE,W1us_B_PRODUCT_AMLOGIC_EFUSE)},
    {SDIO_DEVICE(W1u_VENDOR_AMLOGIC_EFUSE,W1us_C_PRODUCT_AMLOGIC_EFUSE)},
    {}
};

static struct sdio_driver aml_wifi_sdio_driver =
{
    .name = "wifi_sdio",
    .id_table = aml_wifi_sdio,
    .probe = aml_wifi_sdio_probe,
    .remove = aml_wifi_sdio_remove,
};

int  aml_wifi_sdio_init(void)
{
    int err = 0;

    err = sdio_register_driver(&aml_wifi_sdio_driver);
    printk("*****************aml sdio common driver is insmoded********************\n");
    if (err)
        printk("failed to register sdio driver: %d \n", err);

    return err;
}

void  aml_wifi_sdio_exit(void)
{
    printk("aml_sdio_exit++ \n");
    sdio_unregister_driver(&aml_wifi_sdio_driver);
    printk("*****************aml sdio common driver is rmmoded********************\n");
}

int aml_wifi_sdio_insmod(void)
{

    aml_wifi_sdio_init();
    printk("%s(%d) start...\n",__func__, __LINE__);
    return 0;
}

void aml_wifi_sdio_rmmod(void)
{
    aml_wifi_sdio_exit();
}
