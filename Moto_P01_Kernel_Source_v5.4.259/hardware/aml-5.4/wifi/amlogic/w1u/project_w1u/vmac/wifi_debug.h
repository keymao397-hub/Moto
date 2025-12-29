#ifndef _AML_DEBUG_H_
#define _AML_DEBUG_H_

#include <linux/bitops.h>
#include <linux/kernel.h>

#ifndef BIT
#define BIT(n)    (1UL << (n))
#endif //BIT

#define  CTRL_BYTE
#define LINEBYTE 32
#define ASCII_IN  1

typedef enum
{
    AML_LOG_ID_LOG      = 0,
    AML_LOG_ID_ACL      = 1, /* reset module  */
    AML_LOG_ID_XMIT     = 2, /* xmit ampdu */       //AML_DEBUG_TX_AMPDU==>AML_DEBUG_XMIT
    AML_LOG_ID_KEY      = 3, /* key module  */
    AML_LOG_ID_STATE    = 4, /* state module  */
    AML_LOG_ID_RATE     = 5, /* rate control */
    AML_LOG_ID_RECV     = 6, /* rx module */
    AML_LOG_ID_P2P      = 7, /* p2p module */
    AML_LOG_ID_CFG80211 = 8, /* cfg80211 module */
    AML_LOG_ID_SCAN     = 9, /* scan module */
    AML_LOG_ID_LOCK     = 10,
    AML_LOG_ID_INIT     = 11, /* initial module */
    AML_LOG_ID_ROAM     = 12, /* exit module */
    AML_LOG_ID_NODE     = 13, /* nsta module */
    AML_LOG_ID_ANDROID  = 14, /* android module */
    AML_LOG_ID_ACTION   = 15, /* action module */
    AML_LOG_ID_IOCTL    = 16, /* ioctl module */
    AML_LOG_ID_CONNECT  = 17, /* Connect Operating */
    AML_LOG_ID_TIMER    = 18, /* TIMER Debug */
    AML_LOG_ID_ADDBA    = 19, /* ADDBA Debug */
    AML_LOG_ID_NETDEV   = 20, /* netdev module */
    AML_LOG_ID_HAL      = 21, /* hal module  */
    AML_LOG_ID_BEACON   = 22, /* beacon module  */
    AML_LOG_ID_UAPSD    = 23, /* uapsd */
    AML_LOG_ID_BWC      = 24, /* channel bw control */
    AML_LOG_ID_ELEMID   = 25, /* elemid */
    AML_LOG_ID_PWR_SAVE = 26, /* PS Poll and PS save */
    AML_LOG_ID_WME      = 27,
    AML_LOG_ID_DOTH     = 28,
    AML_LOG_ID_RATE_CTR = 29, /* minstrel rate */
    AML_LOG_ID_TX_MSDU  = 30,
    AML_LOG_ID_HAL_TX   = 31,
    AML_LOG_ID_FILTER   = 32,
    AML_LOG_ID_TX_REC   = 33,

    AML_LOG_ID_MAX,
}MODULE_ID;


typedef enum
{
    AML_LOG_LEVEL_ERROR = 0,
    AML_LOG_LEVEL_WARN,
    AML_LOG_LEVEL_INFO,
    AML_LOG_LEVEL_DEBUG,

    AML_LOG_LEVEL_MAX,
}DEBUG_LEVEL;

/*
** Define the "default" debug mask
*/
#define AML_LOG_LEVEL_DEFAULT AML_LOG_LEVEL_INFO

typedef struct
{
   DEBUG_LEVEL moduleTraceLevel;
   // 4 character string name for the module
   unsigned char moduleNameStr[ 5 ];
} moduleTraceInfo;
extern const char *dbg_level_str[];
extern moduleTraceInfo gAmlTraceInfo[ AML_LOG_ID_MAX ];

#define AML_PRINT(_module,_level,format,...) do {   \
    if ((_module < AML_LOG_ID_MAX) && (_level < AML_LOG_LEVEL_MAX)) {   \
        if (gAmlTraceInfo[_module].moduleTraceLevel >= _level) {  \
            if (_module == AML_LOG_ID_LOG) {    \
                printk("[wlan][%-4s][%s] <%s> %d:"format"",gAmlTraceInfo[_module].moduleNameStr, dbg_level_str[_level],\
                                                            __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            } else {    \
                printk("[wlan][%-4s][%s] <%s> %d:"format"",gAmlTraceInfo[_module].moduleNameStr, dbg_level_str[_level],\
                                                            __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            }   \
        }   \
    }   \
} while (0)

#define AML_PRINT_LOG_ERR(format,...) do {    \
            AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_ERROR,format, ##__VA_ARGS__); \
        } while (0)

#define AML_PRINT_LOG_WRAN(format,...) do {    \
            AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_WARN,format, ##__VA_ARGS__); \
        } while (0)

#define AML_PRINT_LOG_INFO(format,...) do {    \
            AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_INFO,format, ##__VA_ARGS__); \
        } while (0)

#define AML_PRINT_LOG_DEBUG(format,...) do {    \
            AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_DEBUG,format, ##__VA_ARGS__); \
        } while (0)

#define DBG_HAL_THR_ENTER()          //AML_PRINT_LOG_INFO("---xman debug---: ++ \n");
#define DBG_HAL_THR_EXIT()           //AML_PRINT_LOG_INFO("---xman debug---: -- \n");

#include "wifi_pt_init.h"
extern struct _B2B_Platform_Conf gB2BPlatformConf;


#define DEBUG_LOCK

#ifdef DEBUG_LOCK
#define OS_SPIN_LOCK_IRQ(a, b)        { AML_PRINT(AML_LOG_ID_LOCK, AML_LOG_LEVEL_DEBUG,"%p, #a ++\n",(a));spin_lock_irqsave((a), (b));}
#define OS_SPIN_UNLOCK_IRQ(a, b)   {AML_PRINT(AML_LOG_ID_LOCK, AML_LOG_LEVEL_DEBUG,"%p,#a --\n",(a));spin_unlock_irqrestore((a), (b));}


#define OS_SPIN_LOCK_BH(a)        {AML_PRINT(AML_LOG_ID_LOCK, AML_LOG_LEVEL_DEBUG,"%p,#a ++\n",(a));spin_lock_bh((a));}
#define OS_SPIN_UNLOCK_BH(a)   {AML_PRINT(AML_LOG_ID_LOCK, AML_LOG_LEVEL_DEBUG,"%p,#a --\n",(a));spin_unlock_bh((a));}


#define OS_SPIN_LOCK(a)        {AML_PRINT(AML_LOG_ID_LOCK, AML_LOG_LEVEL_DEBUG,"%p,#a ++\n",(a));spin_lock((a));}
#define OS_SPIN_UNLOCK(a)   {AML_PRINT(AML_LOG_ID_LOCK, AML_LOG_LEVEL_DEBUG,"%p,#a --\n",(a));spin_unlock((a));}


#define OS_WRITE_LOCK(a)        {AML_PRINT(AML_LOG_ID_LOCK, AML_LOG_LEVEL_DEBUG,"%p,#a ++\n",(a));write_lock((a));}
#define OS_WRITE_UNLOCK(a)   {AML_PRINT(AML_LOG_ID_LOCK, AML_LOG_LEVEL_DEBUG,"%p,#a --\n",(a));write_unlock((a));}

#define OS_WRITE_LOCK_BH(a)        {AML_PRINT(AML_LOG_ID_LOCK, AML_LOG_LEVEL_DEBUG,"%p,#a ++\n",(a));write_lock_bh((a));}
#define OS_WRITE_UNLOCK_BH(a)   {AML_PRINT(AML_LOG_ID_LOCK, AML_LOG_LEVEL_DEBUG,"%p,#a --\n",(a));write_unlock_bh((a));}

#define OS_WRITE_LOCK_IRQ(a, b)        {AML_PRINT(AML_LOG_ID_LOCK, AML_LOG_LEVEL_DEBUG,"%p,#a ++\n",(a));write_lock_irqsave((a), (b));}
#define OS_WRITE_UNLOCK_IRQ(a, b)   {AML_PRINT(AML_LOG_ID_LOCK, AML_LOG_LEVEL_DEBUG,"%p,#a --\n",(a));write_unlock_irqrestore((a), (b));}

#define OS_MUTEX_LOCK(a)        {/*AML_PRINT_LOG_INFO("%p, #a ++\n",(a));*/mutex_lock(a);}
#define OS_MUTEX_UNLOCK(a)        {/*AML_PRINT_LOG_INFO("%p, #a --\n",(a));*/mutex_unlock(a);}

#else
#define OS_SPIN_LOCK_IRQ(a, b)        {spin_lock_irqsave((a), (b));}
#define OS_SPIN_UNLOCK_IRQ(a, b)   {spin_unlock_irqrestore((a), (b));}


#define OS_SPIN_LOCK_BH(a)        {spin_lock_bh((a));}
#define OS_SPIN_UNLOCK_BH(a)   {spin_unlock_bh((a));}


#define OS_SPIN_LOCK(a)        {spin_lock((a));}
#define OS_SPIN_UNLOCK(a)   {spin_unlock((a));}


#define OS_WRITE_LOCK(a)        {write_lock((a));}
#define OS_WRITE_UNLOCK(a)   {write_unlock((a));}

#define OS_WRITE_LOCK_BH(a)        {write_lock_bh((a));}
#define OS_WRITE_UNLOCK_BH(a)   {write_unlock_bh((a));}

#define OS_WRITE_LOCK_IRQ(a, b)        {write_lock_irqsave((a), (b));}
#define OS_WRITE_UNLOCK_IRQ(a, b)   {write_unlock_irqrestore((a), (b));}


#define OS_MUTEX_LOCK(a)        {mutex_lock(a);}
#define OS_MUTEX_UNLOCK(a)        {mutex_unlock(a);}

#endif

#if defined (FPGA) ||defined (CHIP)
// #define PRINT(...)      do {printk( __VA_ARGS__ );}while(0)
// #define PRINT_ERR(...)      do {printk( __VA_ARGS__ );}while(0)
#define PRINT(format, ...)      do {AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_INFO,format,##__VA_ARGS__);}while(0)
#define PRINT_ERR(format, ...)      do {AML_PRINT(AML_LOG_ID_LOG, AML_LOG_LEVEL_ERROR,format,##__VA_ARGS__);}while(0)
#define PUTC( character )   printk("%c" ,character )
#define PUTX8(size, value)    printk("%02x", value);
#define PUTU8(value)            printk("%u", value);
#define PUTS( ... )    printk(__VA_ARGS__)
#define PUTU( number)   printk("%d\n",number)
#define DBG_ENTER()           //printk("--->%s ++ \n",__FUNCTION__);
#define DBG_EXIT()              //printk("--->%s -- \n",__FUNCTION__);
#endif


#ifndef ASSERT
#define ASSERT(exp) do{    \
                if (!(exp)) {   \
                        AML_PRINT_LOG_INFO("=>=>=>=>=>assert \n");   \
                        /*BUG();        while(1);   */  \
                }                       \
        } while (0);
#endif

#ifdef __KERNEL__
#include <asm/page.h>

#define KASSERT(exp, msg) do {          \
                if (unlikely(!(exp))) {         \
                        printk msg;         \
                        BUG();              \
                }                   \
        } while (0)

#endif /* __KERNEL__ */

void address_print( unsigned char* address );
void IPv4_address_print( unsigned char* address );
void dump_memory_internal(unsigned char *data,int len);
void address_read( unsigned char* cursor, unsigned char* address );

 unsigned short READ_16L( const unsigned char* address );
 void WRITE_16L( unsigned char* address, unsigned short value );
unsigned int READ_32L( const unsigned char* address );
void WRITE_32L( unsigned char* address, unsigned int value );

unsigned short READ_16B( const unsigned char* address );
void WRITE_16B( unsigned char* address, unsigned short value );
unsigned int READ_32B( const unsigned char* address );
void WRITE_32B( unsigned char* address, unsigned int value );

void ie_dbg(unsigned char *ie ) ;
void aml_set_all_debug_level(DEBUG_LEVEL level);
void aml_set_debug_level(MODULE_ID module_id, DEBUG_LEVEL level);
void wifi_debug_dump_data(unsigned char* data, unsigned int size, unsigned char bytes_per_line);
unsigned char wifi_debug_is_arp_pkt(struct sk_buff * skb);
unsigned char wifi_debug_get_tid_in_qos_ctrl(void *mac_header);
#endif
