#ifndef __AML_REGDOM_H__
#define __AML_REGDOM_H__

#include <net/cfg80211.h>
#include <linux/types.h>
#include "wifi_mac_com.h"

enum {
    REGDOM_CORE_MGMT = 0,   // Regulatory domain managed by core
    REGDOM_CUST_COREDB,     // Regulatory domain managed by driver with reg.db
    REGDOM_CUST_DRVDEF,     // Regulatory domain managed by driver with custom structure
    REGDOM_SCHEME_MAX
};

// Regulatory domain manage scheme
extern unsigned char regdom_scheme;

#define IS_REGD_CORE_MGMT() ((regdom_scheme) == REGDOM_CORE_MGMT)
#define IS_REGD_CUST_BYDB() ((regdom_scheme) == REGDOM_CUST_COREDB)
#define IS_REGD_CUST_BYDRV() ((regdom_scheme) == REGDOM_CUST_DRVDEF)

#define IS_REGD_CUST() (IS_REGD_CUST_BYDB() || IS_REGD_CUST_BYDRV())
#define IS_REGD_USE_DB() (IS_REGD_CORE_MGMT() || IS_REGD_CUST_BYDB())

#define SCAN_FORBIDDEN         BIT(0)
#define CONNECT_FORBIDDEN     BIT(1)

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
#define IS_REGDOM_SELF_MANAGED(wiphy)	\
	(wiphy->regulatory_flags & REGULATORY_WIPHY_SELF_MANAGED)
#else
#define IS_REGDOM_SELF_MANAGED(wiphy)	(false)
#endif /* KERNEL >= 4.0 */

struct aml_regdom {
    char country_code[2];
    const struct ieee80211_regdomain *regdom;
};

enum aml_regdom_bw {
    REGDOM_BW_20MHZ = 0,
    REGDOM_BW_HT40MINUS,
    REGDOM_BW_HT40PLUS,
    REGDOM_BW_80MHZ,
    REGDOM_BW_MAX
};

void aml_regd_init(struct wiphy * wiphy);
void aml_regd_cust(struct wiphy* wiphy);
void wifi_mac_set_country_code(char* arg);


#endif
