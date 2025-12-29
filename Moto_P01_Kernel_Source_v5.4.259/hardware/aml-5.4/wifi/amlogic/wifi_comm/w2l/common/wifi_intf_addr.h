#ifndef __ADDR_INTF_H__
#define __ADDR_INTF_H__


#define WIFI_INTF         (0xa0d000)

#define RG_INTF_WT_CLK (WIFI_INTF + 0x7c)
#define RG_INTF_MACCORE_CLK (WIFI_INTF + 0x80)
/*coverity[bad_macro_redef]*/
#define RG_INTF_MPIF_CLK (WIFI_INTF + 0x84)
#define RG_INTF_LDPCRX_CLK (WIFI_INTF + 0x88)
#define RG_INTF_VTB_CLK (WIFI_INTF + 0x8c)

#define RG_INTF_CTRL_CLK (WIFI_INTF + 0xb0)
/*coverity[bad_macro_redef]*/
#define RG_INTF_CPU_CLK (WIFI_INTF + 0x090)
#define RG_INTF_APB_CLK (WIFI_INTF + 0x094)
#define RG_INTF_MAC_CLK (WIFI_INTF + 0x098)
#define RG_INTF_DAC_CLK (WIFI_INTF + 0x0a0)
#define RG_INTF_ADC_CLK (WIFI_INTF + 0x0a4)
#define RG_INTF_VIT_CLK (WIFI_INTF + 0x0ac)
#define RG_INTF_11BRX_CLK (WIFI_INTF + 0x0bc)
#define RG_INTF_320M_CLK (WIFI_INTF + 0x100)
#define RG_INTF_240M_CLK (WIFI_INTF + 0x104)
#define RG_INTF_BTCPU_CLK (WIFI_INTF + 0x108)
#define RG_INTF_BTMAC_CLK (WIFI_INTF + 0x10c)
#define RG_INTF_LDPC_CLK (WIFI_INTF + 0x118)
#define RG_INTF_PIN_MUX_REG1 (WIFI_INTF + 0x004)
#define RG_INTF_SHARE_64K_CAP (WIFI_INTF + 0x0e4)
#define RG_INTF_A42 (WIFI_INTF + 0x0a8)
#define RG_INTF_RTC_CLK_CTRL (WIFI_INTF + 0xc0)
#define RG_INTF_MAC_TIMER_CTRL1 (WIFI_INTF + 0xd0)

#define RG_PMU_A24                                (WIFI_INTF + 0x28)
// Bit 31  :0      rg_ram_pd_shutdown_hw          U     RW        default = 'h0
typedef union RG_PMU_A24_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_ram_pd_shutdown_hw : 32;
  } b;
} RG_PMU_A24_FIELD_T;

#define RG_PMU_A25                                (WIFI_INTF + 0x2c)
// Bit 31  :0      rg_ram_pd_disb_gate_hw         U     RW        default = 'h0
typedef union RG_PMU_A25_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_ram_pd_disb_gate_hw : 32;
  } b;
} RG_PMU_A25_FIELD_T;

#define RG_PMU_A20                                (WIFI_INTF + 0x30)
// Bit 31  :0      rg_ram_pd_shutdown_sw          U     RW        default = 'h0
typedef union RG_PMU_A20_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_ram_pd_shutdown_sw : 32;
  } b;
} RG_PMU_A20_FIELD_T;

#define RG_PMU_A21                                (WIFI_INTF + 0x34)
// Bit 31  :0      rg_ram_pd_disb_gate_sw         U     RW        default = 'h0
typedef union RG_PMU_A21_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_ram_pd_disb_gate_sw : 32;
  } b;
} RG_PMU_A21_FIELD_T;


#define RG_RAM_PD_SEL0_SW (WIFI_INTF + 0x38)
#define RG_RAM_PD_SEL1_SW (WIFI_INTF + 0x3c)
#define RG_RAM_PD_SEL0_HW (WIFI_INTF + 0x40)
#define RG_RAM_PD_SEL1_HW (WIFI_INTF + 0x44)

#endif
