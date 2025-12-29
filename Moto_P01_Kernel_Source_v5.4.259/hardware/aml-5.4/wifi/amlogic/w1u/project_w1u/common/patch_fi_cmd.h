#ifndef __PATCH_FI_CMD__
#define __PATCH_FI_CMD__
#include "fi_sdio.h"
#include "fi_cmd.h"

#define TXPWR_CFFC_CFG_CMD 0x2e

struct tx_trb_info_ex
{
    /* The number of pages needed for a single transfer */
    unsigned int packet_num;
    /* Actual size used for each page */
    unsigned short buffer_size[128];
};

#endif
