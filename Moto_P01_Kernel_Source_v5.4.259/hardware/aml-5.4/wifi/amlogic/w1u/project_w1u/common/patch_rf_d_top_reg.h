#ifdef PATCH_RF_D_TOP_REG
#else
#define PATCH_RF_D_TOP_REG


#define RF_D_TOP_REG_BASE                         (0x0)


#define PATCH_RG_TOP_A12                                (RF_D_TOP_REG_BASE + 0x4c)
// Bit 4   :0      rg_wf_dac_rc_adj_q             U     RW        default = 'h8
// Bit 12  :5      rg_wf5g_dac_rsv                U     RW        default = 'h77
// Bit 15  :13     rg_wf_dac_vimc_i               U     RW        default = 'h0
// Bit 18  :16     rg_wf_dac_vimc_q               U     RW        default = 'h0
// Bit 20  :19     rg_wf_dac_pole_sel             U     RW        default = 'h0
// Bit 21          rg_wf_dac_tia_mux_sel_i        U     RW        default = 'h0
// Bit 22          rg_wf_dac_tia_mux_sel_q        U     RW        default = 'h0
// Bit 24  :23     rg_wf_radc_bs_sel              U     RW        default = 'h1
// Bit 25          rg_wf_radc_c_adj               U     RW        default = 'h0
// Bit 28  :26     rg_wf_radc_clk_delay_adj       U     RW        default = 'h1
// Bit 29          rg_wf_radc_ref_adj             U     RW        default = 'h0
typedef union PATCH_RG_TOP_A12_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_wf_dac_rc_adj_q : 5;
    unsigned int rg_wf5g_dac_rsv : 8;
    unsigned int rg_wf_dac_vimc_i : 3;
    unsigned int rg_wf_dac_vimc_q : 3;
    unsigned int rg_wf_dac_pole_sel : 2;
    unsigned int rg_wf_dac_tia_mux_sel_i : 1;
    unsigned int rg_wf_dac_tia_mux_sel_q : 1;
    unsigned int rg_wf_radc_bs_sel : 2;
    unsigned int rg_wf_radc_c_adj : 1;
    unsigned int rg_wf_radc_clk_delay_adj : 3;
    unsigned int rg_wf_radc_ref_adj : 1;
    unsigned int rsvd_0 : 2;
  } b;
} PATCH_RG_TOP_A12_FIELD_T;

#endif
