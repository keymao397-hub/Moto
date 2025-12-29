#ifndef __PLATFORM_WIFI_AML__
#define __PLATFORM_WIFI_AML__

#include <linux/usb.h>
#include <linux/firmware.h>


#ifdef HAL_FPGA_VER
#define OWNER_NAME "AML_WIFI"

#define GPIOX_6     234  //GET ( kernel dts print)
#define GPIOX_7     235
#define GPIOX_19    247
#define GPIOY_15    226 //IRQ TEST USE OK
#define GPIOX_20    248  //used to clk_sel,replace before GPIOY_15

#define INVALID_PARAM_VALUE -1

typedef struct version_info {
    char version_name[10];
    unsigned short version_id;
} version_info;

#ifdef CHIP_RESET_SUPPORT
struct drv_reset_ops {
    int (*enable_cb)(void);
    void (*disable_cb)(void);
};
#endif


void platform_wifi_reset_cpu(void);
int platform_wifi_request_gpio_irq (void *data);
void platform_wifi_free_gpio_irq (void *data);

void platform_wifi_clk_source_sel(int is_ssv_clk);
void platform_wifi_reset (void);

extern enum irqreturn  hal_irq_top(int irq, void *dev_id);

void hi_change_sram_concurrent_mode(void);
void set_wifi_baudrate (unsigned int apb_clk);
int amlhal_resetmac(void);
int amlhal_resetsdio(void);
unsigned char hal_set_sys_clk_for_fpga(void);
void wifi_cpu_clk_switch(unsigned int clk_cfg);
#endif
#define IS_APSTA_CONCURRENT(x) ((x & (1 << WIFINET_M_STA)) && (x & (1 << WIFINET_M_HOSTAP)))
#define WIFI_FW_NAME "wifi_fw_w1u.bin"

extern unsigned char aml_bus_type;

unsigned char hal_set_sys_clk(int clockdiv);
unsigned char hal_set_sys_clk_Core(unsigned int addr, unsigned int value);
int hal_download_sdio_fw_img(void);
#ifdef CONFIG_USB
int hal_download_usb_fw_img(void);
#endif
unsigned int bbpll_init(void);
unsigned int bbpll_start (void);
#ifdef CONFIG_USB
void aml_usb_ctlread_complete(struct urb *urb);
#endif
void aml_disable_wifi(void);
unsigned char aml_enable_wifi(void);

extern void aml_wifi_set_mac_addr(void);
extern char * aml_wifi_get_country_code(void);
extern char * aml_wifi_get_vif0_name(void);
extern char * aml_wifi_get_vif1_name(void);
extern int aml_wifi_get_vif0_opmode(void);
extern int aml_wifi_get_vif1_opmode(void);
extern unsigned int aml_wifi_get_con_mode(void);
extern void aml_wifi_set_con_mode(void *wifimac);
extern int aml_wifi_get_cali_proofing(void);
extern unsigned int aml_wifi_get_platform_verid(void);
extern char *aml_wifi_get_bus_type(void);
extern char *aml_wifi_get_fw_type(void);
extern unsigned int aml_wifi_is_enable_rf_test(void);
extern void aml_w1_fw_recovery(void *drv_priv);

#endif
