#ifndef _AML_INTERFACE_H_
#define _AML_INTERFACE_H_

enum AML_BUS_TYPE {
    AML_BUS_TYPE_SDIO = 0,
    AML_BUS_TYPE_USB,
    AML_BUS_TYPE_PCIE,
};

struct aml_bus_state_detect {
  unsigned char bus_err;
  unsigned char is_drv_load_finished;
  unsigned char bus_reset_ongoing;
  unsigned char is_load_by_timer;
  unsigned char is_recy_ongoing;
  struct timer_list timer;
  struct work_struct detect_work;
  int (*insmod_drv)(void);
};

struct aml_pm_type {
    atomic_t bus_suspend_cnt;
    atomic_t drv_suspend_cnt;
    atomic_t is_suht_down;
    atomic_t wifi_enable;
};
#endif
