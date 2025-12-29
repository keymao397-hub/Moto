#ifdef RF_D_ADDA_XMIT_REG
#else
#define RF_D_ADDA_XMIT_REG


#define RF_D_ADDA_XMIT_REG_BASE                   (0xa0e400)

#define RG_XMIT_A0                                (RF_D_ADDA_XMIT_REG_BASE + 0x0)
// Bit 31  :0      rg_xmit_cfg0                   U     RW        default = 'h0
typedef union RG_XMIT_A0_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_xmit_cfg0 : 32;
  } b;
} RG_XMIT_A0_FIELD_T;

#define RG_XMIT_A1                                (RF_D_ADDA_XMIT_REG_BASE + 0x4)
// Bit 31  :0      rg_xmit_cfg1                   U     RW        default = 'h0
typedef union RG_XMIT_A1_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_xmit_cfg1 : 32;
  } b;
} RG_XMIT_A1_FIELD_T;

#define RG_XMIT_A2                                (RF_D_ADDA_XMIT_REG_BASE + 0x8)
// Bit 24  :0      rg_tg1_f_sel                   U     RW        default = 'h200000
// Bit 28  :25     rg_tg1_tone_gain               U     RW        default = 'h0
// Bit 29          rg_tg1_enable                  U     RW        default = 'h0
// Bit 30          rg_tg1_tone_40                 U     RW        default = 'h0
// Bit 31          rg_tg1_tone_man_en             U     RW        default = 'h1
typedef union RG_XMIT_A2_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tg1_f_sel : 25;
    unsigned int rg_tg1_tone_gain : 4;
    unsigned int rg_tg1_enable : 1;
    unsigned int rg_tg1_tone_40 : 1;
    unsigned int rg_tg1_tone_man_en : 1;
  } b;
} RG_XMIT_A2_FIELD_T;

#define RG_XMIT_A3                                (RF_D_ADDA_XMIT_REG_BASE + 0xc)
// Bit 24  :0      rg_tg2_f_sel                   U     RW        default = 'h200000
// Bit 28  :25     rg_tg2_tone_gain               U     RW        default = 'h0
// Bit 29          rg_tg2_enable                  U     RW        default = 'h0
// Bit 30          rg_tg2_tone_40                 U     RW        default = 'h0
// Bit 31          rg_tg2_tone_man_en             U     RW        default = 'h1
typedef union RG_XMIT_A3_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tg2_f_sel : 25;
    unsigned int rg_tg2_tone_gain : 4;
    unsigned int rg_tg2_enable : 1;
    unsigned int rg_tg2_tone_40 : 1;
    unsigned int rg_tg2_tone_man_en : 1;
  } b;
} RG_XMIT_A3_FIELD_T;

#define RG_XMIT_A4                                (RF_D_ADDA_XMIT_REG_BASE + 0x10)
// Bit 1   :0      rg_tx_signal_sel               U     RW        default = 'h0
// Bit 14  :4      rg_tx_const_i                  U     RW        default = 'h0
// Bit 26  :16     rg_tx_const_q                  U     RW        default = 'h0
// Bit 31          rg_tx_const_v                  U     RW        default = 'h0
typedef union RG_XMIT_A4_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_signal_sel : 2;
    unsigned int rsvd_0 : 2;
    unsigned int rg_tx_const_i : 11;
    unsigned int rsvd_1 : 1;
    unsigned int rg_tx_const_q : 11;
    unsigned int rsvd_2 : 4;
    unsigned int rg_tx_const_v : 1;
  } b;
} RG_XMIT_A4_FIELD_T;

#define RG_XMIT_A5                                (RF_D_ADDA_XMIT_REG_BASE + 0x14)
// Bit 31  :0      rg_tim_ratio                   U     RW        default = 'h0
typedef union RG_XMIT_A5_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tim_ratio : 32;
  } b;
} RG_XMIT_A5_FIELD_T;

#define RG_XMIT_A6                                (RF_D_ADDA_XMIT_REG_BASE + 0x18)
// Bit 7   :0      rg_txirr_comp_theta0           U     RW        default = 'h0
// Bit 15  :8      rg_txirr_comp_theta1           U     RW        default = 'h0
// Bit 23  :16     rg_txirr_comp_theta2           U     RW        default = 'h0
// Bit 31  :24     rg_txirr_comp_theta3           U     RW        default = 'h0
typedef union RG_XMIT_A6_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txirr_comp_theta0 : 8;
    unsigned int rg_txirr_comp_theta1 : 8;
    unsigned int rg_txirr_comp_theta2 : 8;
    unsigned int rg_txirr_comp_theta3 : 8;
  } b;
} RG_XMIT_A6_FIELD_T;

#define RG_XMIT_A7                                (RF_D_ADDA_XMIT_REG_BASE + 0x1c)
// Bit 7   :0      rg_txirr_comp_theta4           U     RW        default = 'h0
// Bit 15  :8      rg_txirr_comp_theta5           U     RW        default = 'h0
// Bit 23  :16     rg_txirr_comp_theta6           U     RW        default = 'h0
// Bit 31  :24     rg_txirr_comp_theta7           U     RW        default = 'h0
typedef union RG_XMIT_A7_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txirr_comp_theta4 : 8;
    unsigned int rg_txirr_comp_theta5 : 8;
    unsigned int rg_txirr_comp_theta6 : 8;
    unsigned int rg_txirr_comp_theta7 : 8;
  } b;
} RG_XMIT_A7_FIELD_T;

#define RG_XMIT_A8                                (RF_D_ADDA_XMIT_REG_BASE + 0x20)
// Bit 7   :0      rg_txirr_comp_theta8           U     RW        default = 'h0
// Bit 15  :8      rg_txirr_comp_theta9           U     RW        default = 'h0
// Bit 23  :16     rg_txirr_comp_thetaa           U     RW        default = 'h0
// Bit 31  :24     rg_txirr_comp_thetab           U     RW        default = 'h0
typedef union RG_XMIT_A8_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txirr_comp_theta8 : 8;
    unsigned int rg_txirr_comp_theta9 : 8;
    unsigned int rg_txirr_comp_thetaa : 8;
    unsigned int rg_txirr_comp_thetab : 8;
  } b;
} RG_XMIT_A8_FIELD_T;

#define RG_XMIT_A9                                (RF_D_ADDA_XMIT_REG_BASE + 0x24)
// Bit 7   :0      rg_txirr_comp_thetac           U     RW        default = 'h0
// Bit 15  :8      rg_txirr_comp_thetad           U     RW        default = 'h0
// Bit 23  :16     rg_txirr_comp_thetae           U     RW        default = 'h0
// Bit 31  :24     rg_txirr_comp_thetaf           U     RW        default = 'h0
typedef union RG_XMIT_A9_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txirr_comp_thetac : 8;
    unsigned int rg_txirr_comp_thetad : 8;
    unsigned int rg_txirr_comp_thetae : 8;
    unsigned int rg_txirr_comp_thetaf : 8;
  } b;
} RG_XMIT_A9_FIELD_T;

#define RG_XMIT_A10                               (RF_D_ADDA_XMIT_REG_BASE + 0x28)
// Bit 7   :0      rg_txirr_comp_alpha0           U     RW        default = 'h0
// Bit 15  :8      rg_txirr_comp_alpha1           U     RW        default = 'h0
// Bit 23  :16     rg_txirr_comp_alpha2           U     RW        default = 'h0
// Bit 31  :24     rg_txirr_comp_alpha3           U     RW        default = 'h0
typedef union RG_XMIT_A10_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txirr_comp_alpha0 : 8;
    unsigned int rg_txirr_comp_alpha1 : 8;
    unsigned int rg_txirr_comp_alpha2 : 8;
    unsigned int rg_txirr_comp_alpha3 : 8;
  } b;
} RG_XMIT_A10_FIELD_T;

#define RG_XMIT_A11                               (RF_D_ADDA_XMIT_REG_BASE + 0x2c)
// Bit 7   :0      rg_txirr_comp_alpha4           U     RW        default = 'h0
// Bit 15  :8      rg_txirr_comp_alpha5           U     RW        default = 'h0
// Bit 23  :16     rg_txirr_comp_alpha6           U     RW        default = 'h0
// Bit 31  :24     rg_txirr_comp_alpha7           U     RW        default = 'h0
typedef union RG_XMIT_A11_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txirr_comp_alpha4 : 8;
    unsigned int rg_txirr_comp_alpha5 : 8;
    unsigned int rg_txirr_comp_alpha6 : 8;
    unsigned int rg_txirr_comp_alpha7 : 8;
  } b;
} RG_XMIT_A11_FIELD_T;

#define RG_XMIT_A12                               (RF_D_ADDA_XMIT_REG_BASE + 0x30)
// Bit 7   :0      rg_txirr_comp_alpha8           U     RW        default = 'h0
// Bit 15  :8      rg_txirr_comp_alpha9           U     RW        default = 'h0
// Bit 23  :16     rg_txirr_comp_alphaa           U     RW        default = 'h0
// Bit 31  :24     rg_txirr_comp_alphab           U     RW        default = 'h0
typedef union RG_XMIT_A12_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txirr_comp_alpha8 : 8;
    unsigned int rg_txirr_comp_alpha9 : 8;
    unsigned int rg_txirr_comp_alphaa : 8;
    unsigned int rg_txirr_comp_alphab : 8;
  } b;
} RG_XMIT_A12_FIELD_T;

#define RG_XMIT_A13                               (RF_D_ADDA_XMIT_REG_BASE + 0x34)
// Bit 7   :0      rg_txirr_comp_alphac           U     RW        default = 'h0
// Bit 15  :8      rg_txirr_comp_alphad           U     RW        default = 'h0
// Bit 23  :16     rg_txirr_comp_alphae           U     RW        default = 'h0
// Bit 31  :24     rg_txirr_comp_alphaf           U     RW        default = 'h0
typedef union RG_XMIT_A13_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txirr_comp_alphac : 8;
    unsigned int rg_txirr_comp_alphad : 8;
    unsigned int rg_txirr_comp_alphae : 8;
    unsigned int rg_txirr_comp_alphaf : 8;
  } b;
} RG_XMIT_A13_FIELD_T;

#define RG_XMIT_A18                               (RF_D_ADDA_XMIT_REG_BASE + 0x48)
// Bit 24  :0      rg_tx_pwrlvl0                  U     RW        default = 'h800000
typedef union RG_XMIT_A18_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_pwrlvl0 : 25;
    unsigned int rsvd_0 : 7;
  } b;
} RG_XMIT_A18_FIELD_T;

#define RG_XMIT_A19                               (RF_D_ADDA_XMIT_REG_BASE + 0x4c)
// Bit 24  :0      rg_tx_pwrlvl1                  U     RW        default = 'h800000
typedef union RG_XMIT_A19_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_pwrlvl1 : 25;
    unsigned int rsvd_0 : 7;
  } b;
} RG_XMIT_A19_FIELD_T;

#define RG_XMIT_A20                               (RF_D_ADDA_XMIT_REG_BASE + 0x50)
// Bit 24  :0      rg_tx_pwrlvl2                  U     RW        default = 'h800000
typedef union RG_XMIT_A20_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_pwrlvl2 : 25;
    unsigned int rsvd_0 : 7;
  } b;
} RG_XMIT_A20_FIELD_T;

#define RG_XMIT_A21                               (RF_D_ADDA_XMIT_REG_BASE + 0x54)
// Bit 24  :0      rg_tx_pwrlvl3                  U     RW        default = 'h800000
typedef union RG_XMIT_A21_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_pwrlvl3 : 25;
    unsigned int rsvd_0 : 7;
  } b;
} RG_XMIT_A21_FIELD_T;

#define RG_XMIT_A22                               (RF_D_ADDA_XMIT_REG_BASE + 0x58)
// Bit 24  :0      rg_tx_pwrlvl4                  U     RW        default = 'h800000
typedef union RG_XMIT_A22_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_pwrlvl4 : 25;
    unsigned int rsvd_0 : 7;
  } b;
} RG_XMIT_A22_FIELD_T;

#define RG_XMIT_A23                               (RF_D_ADDA_XMIT_REG_BASE + 0x5c)
// Bit 24  :0      rg_tx_pwrlvl5                  U     RW        default = 'h800000
typedef union RG_XMIT_A23_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_pwrlvl5 : 25;
    unsigned int rsvd_0 : 7;
  } b;
} RG_XMIT_A23_FIELD_T;

#define RG_XMIT_A24                               (RF_D_ADDA_XMIT_REG_BASE + 0x60)
// Bit 24  :0      rg_tx_pwrlvl6                  U     RW        default = 'h800000
typedef union RG_XMIT_A24_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_pwrlvl6 : 25;
    unsigned int rsvd_0 : 7;
  } b;
} RG_XMIT_A24_FIELD_T;

#define RG_XMIT_A25                               (RF_D_ADDA_XMIT_REG_BASE + 0x64)
// Bit 24  :0      rg_tx_pwrlvl7                  U     RW        default = 'h800000
typedef union RG_XMIT_A25_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_pwrlvl7 : 25;
    unsigned int rsvd_0 : 7;
  } b;
} RG_XMIT_A25_FIELD_T;

#define RG_XMIT_A26                               (RF_D_ADDA_XMIT_REG_BASE + 0x68)
// Bit 24  :0      rg_tx_pwrlvl8                  U     RW        default = 'h800000
typedef union RG_XMIT_A26_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_pwrlvl8 : 25;
    unsigned int rsvd_0 : 7;
  } b;
} RG_XMIT_A26_FIELD_T;

#define RG_XMIT_A27                               (RF_D_ADDA_XMIT_REG_BASE + 0x6c)
// Bit 24  :0      rg_tx_pwrlvl9                  U     RW        default = 'h800000
typedef union RG_XMIT_A27_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_pwrlvl9 : 25;
    unsigned int rsvd_0 : 7;
  } b;
} RG_XMIT_A27_FIELD_T;

#define RG_XMIT_A28                               (RF_D_ADDA_XMIT_REG_BASE + 0x70)
// Bit 24  :0      rg_tx_pwrlvla                  U     RW        default = 'h800000
typedef union RG_XMIT_A28_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_pwrlvla : 25;
    unsigned int rsvd_0 : 7;
  } b;
} RG_XMIT_A28_FIELD_T;

#define RG_XMIT_A29                               (RF_D_ADDA_XMIT_REG_BASE + 0x74)
// Bit 24  :0      rg_tx_pwrlvlb                  U     RW        default = 'h800000
typedef union RG_XMIT_A29_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_pwrlvlb : 25;
    unsigned int rsvd_0 : 7;
  } b;
} RG_XMIT_A29_FIELD_T;

#define RG_XMIT_A30                               (RF_D_ADDA_XMIT_REG_BASE + 0x78)
// Bit 24  :0      rg_tx_pwrlvlc                  U     RW        default = 'h800000
typedef union RG_XMIT_A30_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_pwrlvlc : 25;
    unsigned int rsvd_0 : 7;
  } b;
} RG_XMIT_A30_FIELD_T;

#define RG_XMIT_A31                               (RF_D_ADDA_XMIT_REG_BASE + 0x7c)
// Bit 24  :0      rg_tx_pwrlvld                  U     RW        default = 'h800000
typedef union RG_XMIT_A31_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_pwrlvld : 25;
    unsigned int rsvd_0 : 7;
  } b;
} RG_XMIT_A31_FIELD_T;

#define RG_XMIT_A32                               (RF_D_ADDA_XMIT_REG_BASE + 0x80)
// Bit 24  :0      rg_tx_pwrlvle                  U     RW        default = 'h800000
typedef union RG_XMIT_A32_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_pwrlvle : 25;
    unsigned int rsvd_0 : 7;
  } b;
} RG_XMIT_A32_FIELD_T;

#define RG_XMIT_A33                               (RF_D_ADDA_XMIT_REG_BASE + 0x84)
// Bit 24  :0      rg_tx_pwrlvlf                  U     RW        default = 'h800000
typedef union RG_XMIT_A33_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_pwrlvlf : 25;
    unsigned int rsvd_0 : 7;
  } b;
} RG_XMIT_A33_FIELD_T;

#define RG_XMIT_A34                               (RF_D_ADDA_XMIT_REG_BASE + 0x88)
// Bit 0           rg_tx_iqmm_bypass              U     RW        default = 'h1
// Bit 4           rg_tx_negate_msb               U     RW        default = 'h1
// Bit 8           rg_tx_iq_swap                  U     RW        default = 'h0
// Bit 13  :12     rg_tx_dout_sel                 U     RW        default = 'h0
// Bit 19  :16     rg_tx_async_wgap               U     RW        default = 'ha
typedef union RG_XMIT_A34_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_iqmm_bypass : 1;
    unsigned int rsvd_0 : 3;
    unsigned int rg_tx_negate_msb : 1;
    unsigned int rsvd_1 : 3;
    unsigned int rg_tx_iq_swap : 1;
    unsigned int rsvd_2 : 3;
    unsigned int rg_tx_dout_sel : 2;
    unsigned int rsvd_3 : 2;
    unsigned int rg_tx_async_wgap : 4;
    unsigned int rsvd_4 : 12;
  } b;
} RG_XMIT_A34_FIELD_T;

#define RG_XMIT_A45                               (RF_D_ADDA_XMIT_REG_BASE + 0xb4)
// Bit 24  :0      rg_man_tx_pwrlvl               U     RW        default = 'h800000
// Bit 31          rg_tx_pwr_sel_man_en           U     RW        default = 'h0
typedef union RG_XMIT_A45_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_man_tx_pwrlvl : 25;
    unsigned int rsvd_0 : 6;
    unsigned int rg_tx_pwr_sel_man_en : 1;
  } b;
} RG_XMIT_A45_FIELD_T;

#define RG_XMIT_A46                               (RF_D_ADDA_XMIT_REG_BASE + 0xb8)
// Bit 7   :0      rg_man_txirr_comp_theta        U     RW        default = 'h0
// Bit 15  :8      rg_man_txirr_comp_alpha        U     RW        default = 'h0
// Bit 16          rg_txpwc_comp_man_sel          U     RW        default = 'h1
// Bit 31  :24     rg_txpwc_comp_man              U     RW        default = 'h80
typedef union RG_XMIT_A46_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_man_txirr_comp_theta : 8;
    unsigned int rg_man_txirr_comp_alpha : 8;
    unsigned int rg_txpwc_comp_man_sel : 1;
    unsigned int rsvd_0 : 7;
    unsigned int rg_txpwc_comp_man : 8;
  } b;
} RG_XMIT_A46_FIELD_T;

#define RG_XMIT_A47                               (RF_D_ADDA_XMIT_REG_BASE + 0xbc)
// Bit 15  :0      rg_tx_ex_gain0                 U     RW        default = 'h9
// Bit 31  :16     rg_tx_ex_gain1                 U     RW        default = 'ha
typedef union RG_XMIT_A47_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_ex_gain0 : 16;
    unsigned int rg_tx_ex_gain1 : 16;
  } b;
} RG_XMIT_A47_FIELD_T;

#define RG_XMIT_A48                               (RF_D_ADDA_XMIT_REG_BASE + 0xd0)
// Bit 15  :0      rg_tx_ex_gain2                 U     RW        default = 'h9
// Bit 31  :16     rg_tx_ex_gain3                 U     RW        default = 'ha
typedef union RG_XMIT_A48_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_ex_gain2 : 16;
    unsigned int rg_tx_ex_gain3 : 16;
  } b;
} RG_XMIT_A48_FIELD_T;

#define RG_XMIT_A49                               (RF_D_ADDA_XMIT_REG_BASE + 0xd4)
// Bit 15  :0      rg_tx_ex_gain4                 U     RW        default = 'h9
// Bit 31  :16     rg_tx_ex_gain5                 U     RW        default = 'ha
typedef union RG_XMIT_A49_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_ex_gain4 : 16;
    unsigned int rg_tx_ex_gain5 : 16;
  } b;
} RG_XMIT_A49_FIELD_T;

#define RG_XMIT_A50                               (RF_D_ADDA_XMIT_REG_BASE + 0xd8)
// Bit 15  :0      rg_tx_ex_gain6                 U     RW        default = 'h9
// Bit 31  :16     rg_tx_ex_gain7                 U     RW        default = 'ha
typedef union RG_XMIT_A50_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_ex_gain6 : 16;
    unsigned int rg_tx_ex_gain7 : 16;
  } b;
} RG_XMIT_A50_FIELD_T;

#define RG_XMIT_A51                               (RF_D_ADDA_XMIT_REG_BASE + 0xdc)
// Bit 15  :0      rg_tx_ex_gain8                 U     RW        default = 'h9
// Bit 31  :16     rg_tx_ex_gain9                 U     RW        default = 'ha
typedef union RG_XMIT_A51_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_ex_gain8 : 16;
    unsigned int rg_tx_ex_gain9 : 16;
  } b;
} RG_XMIT_A51_FIELD_T;

#define RG_XMIT_A52                               (RF_D_ADDA_XMIT_REG_BASE + 0xe0)
// Bit 15  :0      rg_tx_ex_gaina                 U     RW        default = 'h9
// Bit 31  :16     rg_tx_ex_gainb                 U     RW        default = 'ha
typedef union RG_XMIT_A52_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_ex_gaina : 16;
    unsigned int rg_tx_ex_gainb : 16;
  } b;
} RG_XMIT_A52_FIELD_T;

#define RG_XMIT_A53                               (RF_D_ADDA_XMIT_REG_BASE + 0xe4)
// Bit 15  :0      rg_tx_ex_gainc                 U     RW        default = 'h9
// Bit 31  :16     rg_tx_ex_gaind                 U     RW        default = 'ha
typedef union RG_XMIT_A53_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_ex_gainc : 16;
    unsigned int rg_tx_ex_gaind : 16;
  } b;
} RG_XMIT_A53_FIELD_T;

#define RG_XMIT_A54                               (RF_D_ADDA_XMIT_REG_BASE + 0xe8)
// Bit 15  :0      rg_tx_ex_gaine                 U     RW        default = 'h9
// Bit 31  :16     rg_tx_ex_gainf                 U     RW        default = 'ha
typedef union RG_XMIT_A54_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_ex_gaine : 16;
    unsigned int rg_tx_ex_gainf : 16;
  } b;
} RG_XMIT_A54_FIELD_T;

#define RG_XMIT_A55                               (RF_D_ADDA_XMIT_REG_BASE + 0xec)
// Bit 15  :0      rg_tx_gain                     U     RW        default = 'h0
// Bit 16          rg_tx_set                      U     RW        default = 'h0
typedef union RG_XMIT_A55_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_gain : 16;
    unsigned int rg_tx_set : 1;
    unsigned int rsvd_0 : 15;
  } b;
} RG_XMIT_A55_FIELD_T;

#define RG_XMIT_A56                               (RF_D_ADDA_XMIT_REG_BASE + 0xf0)
// Bit 7   :0      rg_txdpd_comp_in_gain          U     RW        default = 'h80
// Bit 18  :8      rg_txdpd_comp_out_gain         U     RW        default = 'h80
// Bit 26          rg_txdpd_comp_bypass           U     RW        default = 'h1
// Bit 28          rg_txdpd_comp_bypass_man       U     RW        default = 'h1
// Bit 31          rg_txdpd_comp_coef_man_sel     U     RW        default = 'h0
typedef union RG_XMIT_A56_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_in_gain : 8;
    unsigned int rg_txdpd_comp_out_gain : 11;
    unsigned int rsvd_0 : 5;
    unsigned int rg_txdpd_comp_type : 1;
    unsigned int rsvd_1 : 1;
    unsigned int rg_txdpd_comp_bypass : 1;
    unsigned int rsvd_2 : 1;
    unsigned int rg_txdpd_comp_bypass_man : 1;
    unsigned int rsvd_3 : 2;
    unsigned int rg_txdpd_comp_coef_man_sel : 1;
  } b;
} RG_XMIT_A56_FIELD_T;

#define RG_XMIT_A57                               (RF_D_ADDA_XMIT_REG_BASE + 0xf4)
// Bit 11  :0      rg_txdpd_comp_coef_i_00_man     U     RW        default = 'hff9
// Bit 27  :16     rg_txdpd_comp_coef_q_00_man     U     RW        default = 'h46
typedef union RG_XMIT_A57_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_00_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_00_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A57_FIELD_T;

#define RG_XMIT_A58                               (RF_D_ADDA_XMIT_REG_BASE + 0xf8)
// Bit 11  :0      rg_txdpd_comp_coef_i_01_man     U     RW        default = 'hff8
// Bit 27  :16     rg_txdpd_comp_coef_q_01_man     U     RW        default = 'h13
typedef union RG_XMIT_A58_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_01_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_01_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A58_FIELD_T;

#define RG_XMIT_A59                               (RF_D_ADDA_XMIT_REG_BASE + 0xfc)
// Bit 11  :0      rg_txdpd_comp_coef_i_02_man     U     RW        default = 'h10
// Bit 27  :16     rg_txdpd_comp_coef_q_02_man     U     RW        default = 'h8
typedef union RG_XMIT_A59_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_02_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_02_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A59_FIELD_T;

#define RG_XMIT_A60                               (RF_D_ADDA_XMIT_REG_BASE + 0x100)
// Bit 11  :0      rg_txdpd_comp_coef_i_03_man     U     RW        default = 'h0
// Bit 27  :16     rg_txdpd_comp_coef_q_03_man     U     RW        default = 'h0
typedef union RG_XMIT_A60_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_03_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_03_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A60_FIELD_T;

#define RG_XMIT_A61                               (RF_D_ADDA_XMIT_REG_BASE + 0x104)
// Bit 11  :0      rg_txdpd_comp_coef_i_04_man     U     RW        default = 'he
// Bit 27  :16     rg_txdpd_comp_coef_q_04_man     U     RW        default = 'h93
typedef union RG_XMIT_A61_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_04_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_04_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A61_FIELD_T;

#define RG_XMIT_A62                               (RF_D_ADDA_XMIT_REG_BASE + 0x108)
// Bit 11  :0      rg_txdpd_comp_coef_i_05_man     U     RW        default = 'h16
// Bit 27  :16     rg_txdpd_comp_coef_q_05_man     U     RW        default = 'h1f
typedef union RG_XMIT_A62_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_05_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_05_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A62_FIELD_T;

#define RG_XMIT_A63                               (RF_D_ADDA_XMIT_REG_BASE + 0x10c)
// Bit 11  :0      rg_txdpd_comp_coef_i_06_man     U     RW        default = 'hfe1
// Bit 27  :16     rg_txdpd_comp_coef_q_06_man     U     RW        default = 'hff9
typedef union RG_XMIT_A63_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_06_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_06_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A63_FIELD_T;

#define RG_XMIT_A64                               (RF_D_ADDA_XMIT_REG_BASE + 0x110)
// Bit 11  :0      rg_txdpd_comp_coef_i_07_man     U     RW        default = 'h0
// Bit 27  :16     rg_txdpd_comp_coef_q_07_man     U     RW        default = 'h0
typedef union RG_XMIT_A64_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_07_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_07_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A64_FIELD_T;

#define RG_XMIT_A65                               (RF_D_ADDA_XMIT_REG_BASE + 0x114)
// Bit 11  :0      rg_txdpd_comp_coef_i_08_man     U     RW        default = 'hf79
// Bit 27  :16     rg_txdpd_comp_coef_q_08_man     U     RW        default = 'hc7f
typedef union RG_XMIT_A65_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_08_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_08_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A65_FIELD_T;

#define RG_XMIT_A66                               (RF_D_ADDA_XMIT_REG_BASE + 0x118)
// Bit 11  :0      rg_txdpd_comp_coef_i_09_man     U     RW        default = 'h9
// Bit 27  :16     rg_txdpd_comp_coef_q_09_man     U     RW        default = 'hf51
typedef union RG_XMIT_A66_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_09_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_09_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A66_FIELD_T;

#define RG_XMIT_A67                               (RF_D_ADDA_XMIT_REG_BASE + 0x11c)
// Bit 11  :0      rg_txdpd_comp_coef_i_10_man     U     RW        default = 'hff1
// Bit 27  :16     rg_txdpd_comp_coef_q_10_man     U     RW        default = 'h27
typedef union RG_XMIT_A67_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_10_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_10_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A67_FIELD_T;

#define RG_XMIT_A68                               (RF_D_ADDA_XMIT_REG_BASE + 0x120)
// Bit 11  :0      rg_txdpd_comp_coef_i_11_man     U     RW        default = 'h0
// Bit 27  :16     rg_txdpd_comp_coef_q_11_man     U     RW        default = 'h0
typedef union RG_XMIT_A68_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_11_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_11_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A68_FIELD_T;

#define RG_XMIT_A69                               (RF_D_ADDA_XMIT_REG_BASE + 0x124)
// Bit 11  :0      rg_txdpd_comp_coef_i_12_man     U     RW        default = 'hfeb
// Bit 27  :16     rg_txdpd_comp_coef_q_12_man     U     RW        default = 'hf4d
typedef union RG_XMIT_A69_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_12_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_12_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A69_FIELD_T;

#define RG_XMIT_A70                               (RF_D_ADDA_XMIT_REG_BASE + 0x128)
// Bit 11  :0      rg_txdpd_comp_coef_i_13_man     U     RW        default = 'h5
// Bit 27  :16     rg_txdpd_comp_coef_q_13_man     U     RW        default = 'hfe4
typedef union RG_XMIT_A70_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_13_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_13_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A70_FIELD_T;

#define RG_XMIT_A71                               (RF_D_ADDA_XMIT_REG_BASE + 0x12c)
// Bit 11  :0      rg_txdpd_comp_coef_i_14_man     U     RW        default = 'h6
// Bit 27  :16     rg_txdpd_comp_coef_q_14_man     U     RW        default = 'hfee
typedef union RG_XMIT_A71_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_14_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_14_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A71_FIELD_T;

#define RG_XMIT_A106                              (RF_D_ADDA_XMIT_REG_BASE + 0x1b8)
// Bit 7   :0      rg_tx_atten_factor             U     RW        default = 'h8
// Bit 8           rg_tx_atten_factor_man_mode     U     RW        default = 'h1
// Bit 31  :12     rg_txdpd_coef_update_en        U     RW        default = 'hddddd
typedef union RG_XMIT_A106_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_atten_factor : 8;
    unsigned int rg_tx_atten_factor_man_mode : 1;
    unsigned int rsvd_0 : 3;
    unsigned int rg_txdpd_coef_update_en : 20;
  } b;
} RG_XMIT_A106_FIELD_T;

#define RG_XMIT_A107                              (RF_D_ADDA_XMIT_REG_BASE + 0x1bc)
// Bit 7   :0      rg_tx_coeff_20m_0              U     RW        default = 'hff
// Bit 15  :8      rg_tx_coeff_20m_1              U     RW        default = 'hfe
// Bit 23  :16     rg_tx_coeff_20m_2              U     RW        default = 'hfe
// Bit 31  :24     rg_tx_coeff_20m_3              U     RW        default = 'hfe
typedef union RG_XMIT_A107_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_coeff_20m_0 : 8;
    unsigned int rg_tx_coeff_20m_1 : 8;
    unsigned int rg_tx_coeff_20m_2 : 8;
    unsigned int rg_tx_coeff_20m_3 : 8;
  } b;
} RG_XMIT_A107_FIELD_T;

#define RG_XMIT_A108                              (RF_D_ADDA_XMIT_REG_BASE + 0x1c0)
// Bit 7   :0      rg_tx_coeff_20m_4              U     RW        default = 'h0
// Bit 15  :8      rg_tx_coeff_20m_5              U     RW        default = 'h5
// Bit 23  :16     rg_tx_coeff_20m_6              U     RW        default = 'hd
typedef union RG_XMIT_A108_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_coeff_20m_4 : 8;
    unsigned int rg_tx_coeff_20m_5 : 8;
    unsigned int rg_tx_coeff_20m_6 : 8;
    unsigned int rsvd_0 : 8;
  } b;
} RG_XMIT_A108_FIELD_T;

#define RG_XMIT_A109                              (RF_D_ADDA_XMIT_REG_BASE + 0x1c4)
// Bit 7   :0      rg_tx_coeff_40m_0              U     RW        default = 'h1
// Bit 15  :8      rg_tx_coeff_40m_1              U     RW        default = 'h3
// Bit 23  :16     rg_tx_coeff_40m_2              U     RW        default = 'h4
// Bit 31  :24     rg_tx_coeff_40m_3              U     RW        default = 'h3
typedef union RG_XMIT_A109_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_coeff_40m_0 : 8;
    unsigned int rg_tx_coeff_40m_1 : 8;
    unsigned int rg_tx_coeff_40m_2 : 8;
    unsigned int rg_tx_coeff_40m_3 : 8;
  } b;
} RG_XMIT_A109_FIELD_T;

#define RG_XMIT_A110                              (RF_D_ADDA_XMIT_REG_BASE + 0x1c8)
// Bit 7   :0      rg_tx_coeff_40m_4              U     RW        default = 'hfd
// Bit 15  :8      rg_tx_coeff_40m_5              U     RW        default = 'hf4
// Bit 23  :16     rg_tx_coeff_40m_6              U     RW        default = 'hec
typedef union RG_XMIT_A110_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_coeff_40m_4 : 8;
    unsigned int rg_tx_coeff_40m_5 : 8;
    unsigned int rg_tx_coeff_40m_6 : 8;
    unsigned int rsvd_0 : 8;
  } b;
} RG_XMIT_A110_FIELD_T;

#define RG_XMIT_A111                              (RF_D_ADDA_XMIT_REG_BASE + 0x1cc)
// Bit 1   :0      rg_txirr_theta_step            U     RW        default = 'h0
// Bit 27  :8      rg_txdpd_powr_update_en        U     RW        default = 'h55555
typedef union RG_XMIT_A111_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txirr_theta_step : 2;
    unsigned int rsvd_0 : 6;
    unsigned int rg_txdpd_powr_update_en : 20;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A111_FIELD_T;

#define RG_XMIT_A112                              (RF_D_ADDA_XMIT_REG_BASE + 0x1d0)
// Bit 7   :0      rg_tx_atten_factor0            U     RW        default = 'h8
// Bit 15  :8      rg_tx_atten_factor1            U     RW        default = 'h8
// Bit 23  :16     rg_tx_atten_factor2            U     RW        default = 'h8
// Bit 31  :24     rg_tx_atten_factor3            U     RW        default = 'h8
typedef union RG_XMIT_A112_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_atten_factor0 : 8;
    unsigned int rg_tx_atten_factor1 : 8;
    unsigned int rg_tx_atten_factor2 : 8;
    unsigned int rg_tx_atten_factor3 : 8;
  } b;
} RG_XMIT_A112_FIELD_T;

#define RG_XMIT_A113                              (RF_D_ADDA_XMIT_REG_BASE + 0x1d4)
// Bit 7   :0      rg_tx_atten_factor4            U     RW        default = 'h8
// Bit 15  :8      rg_tx_atten_factor5            U     RW        default = 'h8
// Bit 23  :16     rg_tx_atten_factor6            U     RW        default = 'h8
// Bit 31  :24     rg_tx_atten_factor7            U     RW        default = 'h8
typedef union RG_XMIT_A113_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_atten_factor4 : 8;
    unsigned int rg_tx_atten_factor5 : 8;
    unsigned int rg_tx_atten_factor6 : 8;
    unsigned int rg_tx_atten_factor7 : 8;
  } b;
} RG_XMIT_A113_FIELD_T;

#define RG_XMIT_A114                              (RF_D_ADDA_XMIT_REG_BASE + 0x1d8)
// Bit 7   :0      rg_tx_atten_factor8            U     RW        default = 'h8
// Bit 15  :8      rg_tx_atten_factor9            U     RW        default = 'h8
// Bit 23  :16     rg_tx_atten_factora            U     RW        default = 'h8
// Bit 31  :24     rg_tx_atten_factorb            U     RW        default = 'h8
typedef union RG_XMIT_A114_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_atten_factor8 : 8;
    unsigned int rg_tx_atten_factor9 : 8;
    unsigned int rg_tx_atten_factora : 8;
    unsigned int rg_tx_atten_factorb : 8;
  } b;
} RG_XMIT_A114_FIELD_T;

#define RG_XMIT_A115                              (RF_D_ADDA_XMIT_REG_BASE + 0x1dc)
// Bit 7   :0      rg_tx_atten_factorc            U     RW        default = 'h8
// Bit 15  :8      rg_tx_atten_factord            U     RW        default = 'h8
// Bit 23  :16     rg_tx_atten_factore            U     RW        default = 'h8
// Bit 31  :24     rg_tx_atten_factorf            U     RW        default = 'h8
typedef union RG_XMIT_A115_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_atten_factorc : 8;
    unsigned int rg_tx_atten_factord : 8;
    unsigned int rg_tx_atten_factore : 8;
    unsigned int rg_tx_atten_factorf : 8;
  } b;
} RG_XMIT_A115_FIELD_T;

#define RG_XMIT_A119                              (RF_D_ADDA_XMIT_REG_BASE + 0x1ec)
// Bit 0           rg_tx_80m_ddc_bypass           U     RW        default = 'h0
// Bit 9   :8      rg_tx_ch_bw                    U     RW        default = 'h0
// Bit 12          rg_tx_ch_bw_man_mode           U     RW        default = 'h0
// Bit 16          rg_tx_bn_sel                   U     RW        default = 'h0
// Bit 20          rg_tx_bn_sel_man_mode          U     RW        default = 'h0
// Bit 25  :24     rg_tx_coeff_width_11b          U     RW        default = 'h0
// Bit 27  :26     rg_tx_coeff_width_20m          U     RW        default = 'h0
// Bit 29  :28     rg_tx_coeff_width_40m          U     RW        default = 'h0
// Bit 31  :30     rg_tx_coeff_width_80m          U     RW        default = 'h2
typedef union RG_XMIT_A119_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_80m_ddc_bypass : 1;
    unsigned int rsvd_0 : 7;
    unsigned int rg_tx_ch_bw : 2;
    unsigned int rsvd_1 : 2;
    unsigned int rg_tx_ch_bw_man_mode : 1;
    unsigned int rsvd_2 : 3;
    unsigned int rg_tx_bn_sel : 1;
    unsigned int rsvd_3 : 3;
    unsigned int rg_tx_bn_sel_man_mode : 1;
    unsigned int rsvd_4 : 3;
    unsigned int rg_tx_coeff_width_11b : 2;
    unsigned int rg_tx_coeff_width_20m : 2;
    unsigned int rg_tx_coeff_width_40m : 2;
    unsigned int rg_tx_coeff_width_80m : 2;
  } b;
} RG_XMIT_A119_FIELD_T;

#define RG_XMIT_A120                              (RF_D_ADDA_XMIT_REG_BASE + 0x1f0)
// Bit 7   :0      rg_tx_coeff_20m_7              U     RW        default = 'h18
// Bit 15  :8      rg_tx_coeff_20m_8              U     RW        default = 'h24
// Bit 23  :16     rg_tx_coeff_20m_9              U     RW        default = 'h31
// Bit 31  :24     rg_tx_coeff_20m_10             U     RW        default = 'h3c
typedef union RG_XMIT_A120_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_coeff_20m_7 : 8;
    unsigned int rg_tx_coeff_20m_8 : 8;
    unsigned int rg_tx_coeff_20m_9 : 8;
    unsigned int rg_tx_coeff_20m_10 : 8;
  } b;
} RG_XMIT_A120_FIELD_T;

#define RG_XMIT_A121                              (RF_D_ADDA_XMIT_REG_BASE + 0x1f4)
// Bit 7   :0      rg_tx_coeff_20m_11             U     RW        default = 'h44
// Bit 15  :8      rg_tx_coeff_20m_12             U     RW        default = 'h46
typedef union RG_XMIT_A121_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_coeff_20m_11 : 8;
    unsigned int rg_tx_coeff_20m_12 : 8;
    unsigned int rsvd_0 : 16;
  } b;
} RG_XMIT_A121_FIELD_T;

#define RG_XMIT_A122                              (RF_D_ADDA_XMIT_REG_BASE + 0x1f8)
// Bit 7   :0      rg_tx_coeff_40m_7              U     RW        default = 'hee
// Bit 15  :8      rg_tx_coeff_40m_8              U     RW        default = 'h0
// Bit 23  :16     rg_tx_coeff_40m_9              U     RW        default = 'h23
// Bit 31  :24     rg_tx_coeff_40m_10             U     RW        default = 'h4e
typedef union RG_XMIT_A122_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_coeff_40m_7 : 8;
    unsigned int rg_tx_coeff_40m_8 : 8;
    unsigned int rg_tx_coeff_40m_9 : 8;
    unsigned int rg_tx_coeff_40m_10 : 8;
  } b;
} RG_XMIT_A122_FIELD_T;

#define RG_XMIT_A123                              (RF_D_ADDA_XMIT_REG_BASE + 0x1fc)
// Bit 7   :0      rg_tx_coeff_40m_11             U     RW        default = 'h71
// Bit 15  :8      rg_tx_coeff_40m_12             U     RW        default = 'h7f
typedef union RG_XMIT_A123_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_coeff_40m_11 : 8;
    unsigned int rg_tx_coeff_40m_12 : 8;
    unsigned int rsvd_0 : 16;
  } b;
} RG_XMIT_A123_FIELD_T;

#define RG_XMIT_A124                              (RF_D_ADDA_XMIT_REG_BASE + 0x200)
// Bit 7   :0      rg_tx_coeff_11b_0              U     RW        default = 'hf9
// Bit 15  :8      rg_tx_coeff_11b_1              U     RW        default = 'hfe
// Bit 23  :16     rg_tx_coeff_11b_2              U     RW        default = 'h3
// Bit 31  :24     rg_tx_coeff_11b_3              U     RW        default = 'h8
typedef union RG_XMIT_A124_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_coeff_11b_0 : 8;
    unsigned int rg_tx_coeff_11b_1 : 8;
    unsigned int rg_tx_coeff_11b_2 : 8;
    unsigned int rg_tx_coeff_11b_3 : 8;
  } b;
} RG_XMIT_A124_FIELD_T;

#define RG_XMIT_A125                              (RF_D_ADDA_XMIT_REG_BASE + 0x204)
// Bit 7   :0      rg_tx_coeff_11b_4              U     RW        default = 'hd
// Bit 15  :8      rg_tx_coeff_11b_5              U     RW        default = 'h13
// Bit 23  :16     rg_tx_coeff_11b_6              U     RW        default = 'h18
// Bit 31  :24     rg_tx_coeff_11b_7              U     RW        default = 'h1d
typedef union RG_XMIT_A125_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_coeff_11b_4 : 8;
    unsigned int rg_tx_coeff_11b_5 : 8;
    unsigned int rg_tx_coeff_11b_6 : 8;
    unsigned int rg_tx_coeff_11b_7 : 8;
  } b;
} RG_XMIT_A125_FIELD_T;

#define RG_XMIT_A126                              (RF_D_ADDA_XMIT_REG_BASE + 0x208)
// Bit 7   :0      rg_tx_coeff_11b_8              U     RW        default = 'h21
// Bit 15  :8      rg_tx_coeff_11b_9              U     RW        default = 'h25
// Bit 23  :16     rg_tx_coeff_11b_10             U     RW        default = 'h27
// Bit 31  :24     rg_tx_coeff_11b_11             U     RW        default = 'h29
typedef union RG_XMIT_A126_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_coeff_11b_8 : 8;
    unsigned int rg_tx_coeff_11b_9 : 8;
    unsigned int rg_tx_coeff_11b_10 : 8;
    unsigned int rg_tx_coeff_11b_11 : 8;
  } b;
} RG_XMIT_A126_FIELD_T;

#define RG_XMIT_A127                              (RF_D_ADDA_XMIT_REG_BASE + 0x20c)
// Bit 7   :0      rg_tx_coeff_11b_12             U     RW        default = 'h29
typedef union RG_XMIT_A127_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_coeff_11b_12 : 8;
    unsigned int rsvd_0 : 24;
  } b;
} RG_XMIT_A127_FIELD_T;

#define RG_XMIT_A128                              (RF_D_ADDA_XMIT_REG_BASE + 0x210)
// Bit 7   :0      rg_tx_coeff_80m_0              U     RW        default = 'hff
// Bit 15  :8      rg_tx_coeff_80m_1              U     RW        default = 'hfe
// Bit 23  :16     rg_tx_coeff_80m_2              U     RW        default = 'h0
// Bit 31  :24     rg_tx_coeff_80m_3              U     RW        default = 'h2
typedef union RG_XMIT_A128_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_coeff_80m_0 : 8;
    unsigned int rg_tx_coeff_80m_1 : 8;
    unsigned int rg_tx_coeff_80m_2 : 8;
    unsigned int rg_tx_coeff_80m_3 : 8;
  } b;
} RG_XMIT_A128_FIELD_T;

#define RG_XMIT_A129                              (RF_D_ADDA_XMIT_REG_BASE + 0x214)
// Bit 7   :0      rg_tx_coeff_80m_4              U     RW        default = 'h0
// Bit 15  :8      rg_tx_coeff_80m_5              U     RW        default = 'hfc
// Bit 23  :16     rg_tx_coeff_80m_6              U     RW        default = 'hff
// Bit 31  :24     rg_tx_coeff_80m_7              U     RW        default = 'h7
typedef union RG_XMIT_A129_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_coeff_80m_4 : 8;
    unsigned int rg_tx_coeff_80m_5 : 8;
    unsigned int rg_tx_coeff_80m_6 : 8;
    unsigned int rg_tx_coeff_80m_7 : 8;
  } b;
} RG_XMIT_A129_FIELD_T;

#define RG_XMIT_A130                              (RF_D_ADDA_XMIT_REG_BASE + 0x218)
// Bit 7   :0      rg_tx_coeff_80m_8              U     RW        default = 'h1
// Bit 15  :8      rg_tx_coeff_80m_9              U     RW        default = 'hf3
// Bit 23  :16     rg_tx_coeff_80m_10             U     RW        default = 'hff
// Bit 31  :24     rg_tx_coeff_80m_11             U     RW        default = 'h28
typedef union RG_XMIT_A130_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_coeff_80m_8 : 8;
    unsigned int rg_tx_coeff_80m_9 : 8;
    unsigned int rg_tx_coeff_80m_10 : 8;
    unsigned int rg_tx_coeff_80m_11 : 8;
  } b;
} RG_XMIT_A130_FIELD_T;

#define RG_XMIT_A131                              (RF_D_ADDA_XMIT_REG_BASE + 0x21c)
// Bit 7   :0      rg_tx_coeff_80m_12             U     RW        default = 'h41
typedef union RG_XMIT_A131_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_tx_coeff_80m_12 : 8;
    unsigned int rsvd_0 : 24;
  } b;
} RG_XMIT_A131_FIELD_T;

#define RG_XMIT_A132                              (RF_D_ADDA_XMIT_REG_BASE + 0x220)
// Bit 11  :0      rg_txdpd_comp_coef_i_15_man     U     RW        default = 'h0
// Bit 27  :16     rg_txdpd_comp_coef_q_15_man     U     RW        default = 'h0
typedef union RG_XMIT_A132_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_15_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_15_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A132_FIELD_T;

#define RG_XMIT_A133                              (RF_D_ADDA_XMIT_REG_BASE + 0x224)
// Bit 11  :0      rg_txdpd_comp_coef_i_16_man     U     RW        default = 'h17
// Bit 27  :16     rg_txdpd_comp_coef_q_16_man     U     RW        default = 'ha3
typedef union RG_XMIT_A133_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_16_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_16_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A133_FIELD_T;

#define RG_XMIT_A134                              (RF_D_ADDA_XMIT_REG_BASE + 0x228)
// Bit 11  :0      rg_txdpd_comp_coef_i_17_man     U     RW        default = 'hffc
// Bit 27  :16     rg_txdpd_comp_coef_q_17_man     U     RW        default = 'h16
typedef union RG_XMIT_A134_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_17_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_17_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A134_FIELD_T;

#define RG_XMIT_A135                              (RF_D_ADDA_XMIT_REG_BASE + 0x22c)
// Bit 11  :0      rg_txdpd_comp_coef_i_18_man     U     RW        default = 'hfff
// Bit 27  :16     rg_txdpd_comp_coef_q_18_man     U     RW        default = 'h5
typedef union RG_XMIT_A135_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_18_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_18_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A135_FIELD_T;

#define RG_XMIT_A136                              (RF_D_ADDA_XMIT_REG_BASE + 0x230)
// Bit 11  :0      rg_txdpd_comp_coef_i_19_man     U     RW        default = 'h0
// Bit 27  :16     rg_txdpd_comp_coef_q_19_man     U     RW        default = 'h0
typedef union RG_XMIT_A136_FIELD
{
  unsigned int data;
  struct
  {
    unsigned int rg_txdpd_comp_coef_i_19_man : 12;
    unsigned int rsvd_0 : 4;
    unsigned int rg_txdpd_comp_coef_q_19_man : 12;
    unsigned int rsvd_1 : 4;
  } b;
} RG_XMIT_A136_FIELD_T;

#endif

