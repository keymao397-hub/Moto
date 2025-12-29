#ifndef _AML_DEBUG_H_
#define _AML_DEBUG_H_

#include <linux/bitops.h>
#include <linux/kernel.h>

#ifndef BIT
#define BIT(n)    (1UL << (n))
#endif //BIT

enum
{
    AML_DBG_MODULES_TX = BIT(0),        /* tx */
    AML_DBG_MODULES_RX = BIT(1),        /* rx */
    AML_DBG_MODULES_CMD = BIT(2),       /* cmd */
    AML_DBG_MODULES_TRACE = BIT(3),     /* trace */
    AML_DBG_MODULES_INTERFACE = BIT(4),     /* interface */
    AML_DBG_MODULES_IWPRIV = BIT(5),    /* iwpriv */
    AML_DBG_MODULES_MAIN = BIT(6),    /* main */
    AML_DBG_MODULES_MDNS = BIT(7),    /* mdns */
    AML_DBG_MODULES_MSG_RX = BIT(8),    /* msg_rx */
    AML_DBG_MODULES_MSG_TX = BIT(9),    /* msg_tx */
    AML_DBG_MODULES_PLATF = BIT(10),    /* platform */
    AML_DBG_MODULES_TESTM = BIT(11),    /* testmode */
    AML_DBG_MODULES_PCI = BIT(12),    /* pci */
    AML_DBG_MODULES_COMMON = BIT(13),    /* sdio/usb common */
    AML_DBG_MODULES_SDIO = BIT(14),    /* sdio */
    AML_DBG_MODULES_USB = BIT(15),    /* usb */
    AML_DBG_MODULES_UTILS = BIT(16),    /* utils */
    AML_DBG_MODULES_CSI = BIT(17),    /* csi */
};

enum
{
    AML_DBG_OFF = 0,
    AML_DBG_ON = 1,
};

extern int aml_debug;
extern unsigned long long g_dbg_modules;

#define AML_PRINT( _m,format,...) do { \
            if (g_dbg_modules & (_m)) \
            { \
                if(_m == AML_DBG_MODULES_TX) \
                printk("[TX] <%s> %d "format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
                if(_m == AML_DBG_MODULES_RX) \
                printk("[RX] <%s> %d "format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
                if(_m == AML_DBG_MODULES_CMD) \
                printk("[CMD] <%s> %d "format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
                if(_m == AML_DBG_MODULES_TRACE) \
                printk("[TRACE] <%s> %d "format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
                if(_m == AML_DBG_MODULES_INTERFACE) \
                printk("[INTERFACE] <%s> %d "format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
                if(_m == AML_DBG_MODULES_IWPRIV) \
                printk("[IWPRIV] <%s> %d "format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
                if(_m == AML_DBG_MODULES_MAIN) \
                printk("[MAIN] <%s> %d "format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
                if(_m == AML_DBG_MODULES_MDNS) \
                printk("[MDNS] <%s> %d "format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
                if(_m == AML_DBG_MODULES_MSG_RX) \
                printk("[MSG_RX] <%s> %d "format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
                if(_m == AML_DBG_MODULES_MSG_TX) \
                printk("[MSG_TX] <%s> %d "format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
                if(_m == AML_DBG_MODULES_PLATF) \
                printk("[PLATF] <%s> %d "format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
                if(_m == AML_DBG_MODULES_TESTM) \
                printk("[TESTM] <%s> %d "format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
                if(_m == AML_DBG_MODULES_PCI) \
                printk("[PCI] <%s> %d "format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
                if(_m == AML_DBG_MODULES_COMMON) \
                printk("[COMMON] <%s> %d "format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
                if(_m == AML_DBG_MODULES_SDIO) \
                printk("[SDIO] <%s> %d "format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
                if(_m == AML_DBG_MODULES_USB) \
                printk("[USB] <%s> %d "format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
                if(_m == AML_DBG_MODULES_UTILS) \
                printk("[UTILS] <%s> %d "format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
                if(_m == AML_DBG_MODULES_CSI) \
                printk("[CSI] "format"", ##__VA_ARGS__); \
            } \
        } while (0)

#define ERROR_DEBUG_OUT(format,...) do { \
                 printk("FUNCTION: %s LINE: %d:"format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
        } while (0)

#define AML_OUTPUT(format,...) do { \
                 printk("<%s> %d:"format"",__FUNCTION__, __LINE__, ##__VA_ARGS__); \
        } while (0)


#endif /* _DRV_AH_INTERAL_H_ */
