#ifdef PATCH_RF_D_TX_REG
#else
#define PATCH_RF_D_TX_REG

#define RF_D_TX_REG_BASE                          (0x800)

#define PATCH_RG_TX_A4                                  (RF_D_TX_REG_BASE + 0x10)
// Bit 2   :0      rg_wf2g_tx_pa_vgcg0            U     RW        default = 'h7
// Bit 10  :8      rg_wf2g_tx_mxr_vgcg1           U     RW        default = 'h7
// Bit 13  :11     rg_wf2g_tx_mxr_lo_vb           U     RW        default = 'h7
// Bit 16  :14     rg_wf2g_tx_mxr_lobuf_ict       U     RW        default = 'h1
// Bit 20  :17     rg_wf2g_tx_var_u               U     RW        default = 'h8
// Bit 24  :21     rg_wf2g_tx_var_d               U     RW        default = 'h8
typedef union PATCH_RG_TX_A4_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int RG_WF2G_TX_PA_VGCG0 : 3;
    unsigned int rsvd_0 : 5;
    unsigned int RG_WF2G_TX_MXR_VGCG1 : 3;
    unsigned int RG_WF2G_TX_MXR_LO_VB : 3;
    unsigned int RG_WF2G_TX_MXR_LOBUF_ICT : 3;
    unsigned int RG_WF2G_TX_VAR_U : 4;
    unsigned int RG_WF2G_TX_VAR_D : 4;
    unsigned int rsvd_1 : 7;
  } b;
} PATCH_RG_TX_A4_FIELD_T;


#define PATCH_RG_TX_A13                                 (RF_D_TX_REG_BASE + 0x34)
// Bit 4   :0      rg_wf5g_tx_mxr_cbank_f1        U     RW        default = 'h7
// Bit 9   :5      rg_wf5g_tx_mxr_cbank_f2        U     RW        default = 'h7
// Bit 14  :10     rg_wf5g_tx_mxr_cbank_f3        U     RW        default = 'h7
// Bit 19  :15     rg_wf5g_tx_mxr_cbank_f4        U     RW        default = 'h3
// Bit 24  :20     rg_wf5g_tx_mxr_cbank_man       U     RW        default = 'h4
// Bit 25          rg_wf5g_tx_mxr_cbank_man_mode     U     RW        default = 'h0
typedef union PATCH_RG_TX_A13_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int RG_WF5G_TX_MXR_CBANK_F1 : 5;
    unsigned int RG_WF5G_TX_MXR_CBANK_F2 : 5;
    unsigned int RG_WF5G_TX_MXR_CBANK_F3 : 5;
    unsigned int RG_WF5G_TX_MXR_CBANK_F4 : 5;
    unsigned int RG_WF5G_TX_MXR_CBANK_MAN : 5;
    unsigned int RG_WF5G_TX_MXR_CBANK_MAN_MODE : 1;
    unsigned int rsvd_0 : 6;
  } b;
} PATCH_RG_TX_A13_FIELD_T;

#endif
