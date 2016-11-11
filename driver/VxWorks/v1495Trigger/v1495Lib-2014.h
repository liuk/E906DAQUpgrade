/**************************************************************************
*
*  v1495Lib.h  - Header for v1495 Library
*
*
*  Author: Shiuan-Hal , Shiu
*
*          August  2010
*
* Shift v1495_data{} up to 0x1000 and above.  REM - 2013-03-14
* Added struct for pulser data memory blocks. REM - 2013-06-10
*/


/* Define Structure for access to Local Memory map*/
/* $$ means don't using now! */

struct v1495_data {
  volatile unsigned short a_sta_l;        /* now is internal memory addr readout (0x1000)*/
  volatile unsigned short a_sta_h;        /* now ??? undefine(0x1002) */
  volatile unsigned short b_sta_l;        /* B port status (low)(0x1004) */
  volatile unsigned short b_sta_h;        /* B port status (high)(0x1006) */
  volatile unsigned short c_sta_l;        /* C port status (low)(0x1008) */
  volatile unsigned short c_sta_h;        /* C port status (high)(0x100A) */
  volatile unsigned short a_mask_l;       /* $$a mask register (low)(0x100C) */
  volatile unsigned short a_mask_h;       /* $$a mask register (high)(0x100E) */
  volatile unsigned short b_mask_l;       /* $$b mask register (low)(0x1010) */
  volatile unsigned short b_mask_h;       /* $$b mask register (high)(0x1012) */
  volatile unsigned short c_mask_l;       /* $$c mask register (low)(0x1014) */
  volatile unsigned short c_mask_h;       /* $$c mask register (high)(0x1016) */
  volatile unsigned short gatewidth;      /* $$gate width(0x1018) */
  volatile unsigned short c_ctrl_l;       /* now is internal memory addr ctrl (0x101A) */
  volatile unsigned short c_ctrl_h;       /* now is internal memory ctrl (0x101C) */
  volatile unsigned short mode;           /* $$mode(0x101E) */
  volatile unsigned short scratch;        /* scratch(0x1020) */
  volatile unsigned short g_ctrl;         /* $$g ctrl only bit 0 work(0x101E) */
  volatile unsigned short d_ctrl_l;       /* $$d ctrl (low)(0x1024) */
  volatile unsigned short d_ctrl_h;       /* $$d ctrl (high)(0x1026) */
  volatile unsigned short d_data_l;       /* $$d data (low)(0x1028) */
  volatile unsigned short d_data_h;       /* $$d data (high)(0x102A) */
  volatile unsigned short e_ctrl_l;       /* $$e ctrl (low)(0x102C) */
  volatile unsigned short e_ctrl_h;       /* $$e ctrl (high)(0x102E) */
  volatile unsigned short e_data_l;       /* $$e data (low)(0x1030) */
  volatile unsigned short e_data_h;       /* $$e data (high)(0x1032) */
  volatile unsigned short f_ctrl_l;       /* $$f ctrl (low)(0x1034) */
  volatile unsigned short f_ctrl_h;       /* $$f ctrl (high)(0x1036) */
  volatile unsigned short f_data_l;       /* $$f data(0x1038) */
  volatile unsigned short f_data_h;       /* $$f Set to 0x0001 to start pulser  data(0x103A) */
  volatile unsigned short revision;       /* revision(0x103C) */
  volatile unsigned short pdl_ctrl;       /* $$pdl control(0x103E) */
  volatile unsigned short pdl_data;       /* $$pdl data(0x1040) */
  volatile unsigned short d_id;           /* d id code(0x1042) */
  volatile unsigned short e_id;           /* e id code(0x1044) */
  volatile unsigned short f_id;           /* f id code(0x1046) */
  volatile unsigned short evtid_l;        /* event ID lower 16-bit */
  volatile unsigned short evtid_h;        /* event ID higher 16-bit */

};

struct v1495_vmestruct {
  volatile unsigned short ctrlr;        /* $$ control register (0x8000)*/
  volatile unsigned short statusr;      /* $$ status register (0x8002)*/
  volatile unsigned short int_lv;       /* $$ interrupt Level (0x8004)*/
  volatile unsigned short int_ID;       /* $$ interrupt Lv ID (0x8006)*/
  volatile unsigned short geo_add;      /* $$ geo address register (0x8008)*/
  volatile unsigned short mreset;       /* module reset (0x800A)*/
  volatile unsigned short firmware;     /* $$ firmware revision (0x800C)*/
  volatile unsigned short svmefpga;     /* $$ select vme fpga (0x800E)*/
  volatile unsigned short vmefpga;      /* $$ vme fpga flash (0x8010)*/
  volatile unsigned short suserfpga;    /* $$ select user fpga (0x8012)*/
  volatile unsigned short userfpga;     /* $$ user fpga flash (0x8014)*/
  volatile unsigned short fpgaconf;     /* user fpga configuration (0x8016)*/
  volatile unsigned short scratch16;    /* scratch16 (0x8018)*/
  volatile unsigned int   scratch32;    /* dcratch32 (0x8020)*/
};

struct v1495_memreadout {
  volatile unsigned short mem[128];     /* $$ memorr readout?(0x1100~0x11FF)*/
};

struct v1495_tdcreadout {
  volatile unsigned short tdc[256];     /* $$ tdc readout?(0x1200~0x13FF)*/
};

struct v1495_pulse {
  volatile unsigned short pulse1[256];  /*pulser data block #1 (0x1400-0x15ff) */
  volatile unsigned short pulse2[256];  /*pulser data block #2 (0x1600-0x17ff) */
  volatile unsigned short pulse3[256];  /*pulser data block #3 (0x1800-0x19ff) */
  volatile unsigned short pulse4[256];  /*pulser data block #4 (0x1a00-0x1bff) */
  volatile unsigned short pulse5[256];  /*pulser data block #5 (0x1c00-0x1dff) */
  volatile unsigned short pulse6[256];  /*pulser data block #6 (0x1e00-0x1fff) */
};

#define v1495_MAX_BOARDS 5

/* Define Bit Masks */
#define v1495_CHANNEL_L               0x0003
#define v1495_CHANNEL_H               0x3e00
#define v1495_SAMPLE                  0x01fc
#define v1495_READOUT                 0x00f0
