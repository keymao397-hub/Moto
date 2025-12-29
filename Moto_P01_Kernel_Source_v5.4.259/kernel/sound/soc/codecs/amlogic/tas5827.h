#ifndef _TAS5827_H_
#define _TAS5827_H_

#include <linux/list.h>

#define TAS5827_RATES \
	(SNDRV_PCM_RATE_32000 | \
	 SNDRV_PCM_RATE_44100 | \
	 SNDRV_PCM_RATE_48000 | \
	 SNDRV_PCM_RATE_88200 | \
	 SNDRV_PCM_RATE_96000 | \
	 SNDRV_PCM_RATE_192000)

#define TAS5827_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE | \
	 SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE | \
	 SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_BE | \
	 SNDRV_PCM_FMTBIT_S32_LE)

struct REG_TAB_5827 {
        unsigned char Address;
        unsigned char Data;
};

static LIST_HEAD(fsm_dev_list);
#define fsm_list_init(fsm_dev) \
	do { \
		INIT_LIST_HEAD(&fsm_dev->list); \
		list_add(&fsm_dev->list, &fsm_dev_list); \
	} while (0)

#define fsm_list_entry(fsm_dev, ops) \
	do { \
		list_for_each_entry(fsm_dev, &fsm_dev_list, list) { \
			ops(fsm_dev); \
		} \
	} while(0)

#define fsm_list_func(fsm_dev, func) \
	do { \
		list_for_each_entry(fsm_dev, &fsm_dev_list, list) { \
			func(fsm_dev); \
		} \
	} while(0)

#define fsm_list_check(fsm_dev, type, ret) \
	do { \
		ret = 0; \
		list_for_each_entry(fsm_dev, &fsm_dev_list, list) { \
			ret |= fsm_stub_check_stable(fsm_dev, type); \
			if (ret) { \
				break; \
			} \
		} \
	} while(0)

#define fsm_list_return(fsm_dev, func, ret) \
	do { \
		ret = 0; \
		list_for_each_entry(fsm_dev, &fsm_dev_list, list) { \
			ret |= func(fsm_dev); \
		} \
	} while(0)

#define fsm_list_func_arg(fsm_dev, func, argv) \
	do { \
		list_for_each_entry(fsm_dev, &fsm_dev_list, list) { \
			func(fsm_dev, argv); \
		} \
	} while(0)

#define tas5827_log(type, fmt, args...) \
		pr_##type("%s: " fmt "\n",__func__, ##args)

#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
#define snd_soc_codec              snd_soc_component
#define snd_soc_add_codec_controls snd_soc_add_component_controls
#define snd_soc_codec_get_drvdata  snd_soc_component_get_drvdata
#define snd_soc_unregister_codec   snd_soc_unregister_component
#define snd_soc_register_codec     devm_snd_soc_register_component
#define snd_soc_codec_driver       snd_soc_component_driver
#endif

struct fsm_dev {
	struct i2c_client *i2c;
	struct mutex i2c_lock;
	struct regmap *regmap;

	int isbypass;
	int cld_mode;
	int dev_index;
	uint16_t l_vol;
	uint16_t r_vol;
	int sdz_pin;
	int state;

	struct list_head list;
};
typedef struct fsm_dev fsm_dev_t;

#include "tas5827_reg.h"
#include "tas5827_reg_ext.h"
//#define USE_GGEC_MUSIC_MODE_IMPLEMENT_CUSTOM_MODE 1
#include "tas5827_ggec_music_mode_reg.h"
#endif //_TAS5827_H_
