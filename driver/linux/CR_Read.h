/******************************************************************\
*                                                                  *
*  DslTdc.h  - Header for Da-Shung's Latch-TDC Card VME           *
*                   I/O Register Library                           *
*                                                                  *
*  Author: David Abbott                                            *
*          Jefferson Lab Data Acquisition Group                    *
*          April 2003                                              *
*                                                                  *
*  dsLatchCard.h - Header for Jin's Latch Card                *
*             Multi Event Latch Library                            *
*                                                                  *
*  Author:
*          Grass Wang					 	   *
*          Jun. 2012						   *
*          *                                                                  *
* This header file is modified from the dsLatchCard.h              *
\******************************************************************/
#ifdef VXWORKS
#include <vxWorks.h>
#include <logLib.h>
#else
#include <stdlib.h>
#include "jvme.h"
#endif

#include "stdio.h"
#include "string.h"
#include "unistd.h"
#include "time.h"

void CR_Reset(int id);
STATUS CR_Init (UINT32 addr, UINT32 addr_inc, int nmod);
void CR_Status(int id, int iword);
void CR_DataInit(int id, int csr);
void CR_FifoRead(int id, int ii);
void CR_WR_Reg(int id, int regaddr, int reg_value);
int  CR_RD_Reg(int id, int regaddr);
void CR_HeaderInit(int id, int csr);
void CR_TrigDisable(int id);
void CR_TrigEnable(int id);
void CR_FastTrigDisable(int id, unsigned int csr);
void CR_FastTrigEnable(int id, unsigned int csr);

void CR_ScalarInit(int id, int scalarID);
void CR_Scalar_Switch(int id, int scalarID);
void CR_ScalarDisplay(int id, int scalarID);
int  CR_BroadID(UINT32 addr, UINT32 addr_inc, int id);

void DP_Init(int id, int dp_start, int dp_end);
void DP_Write(int id, int val, int dp_start, int dp_end);
int  DP_Read(int id, int idx);

/* --- Define Structure for access to Local Memory map --- */
#define DSTDC_MAX_BOARDS 20
#define DATA_BUF_SIZE 1024
#define DP_BUF_SIZE 32767

struct Read_reg_struct{
  volatile unsigned int reg[64];             // 0x000  Control and Status Register
};

struct Read_dp_data {
  volatile unsigned int dp[DP_BUF_SIZE];
};

struct Read_scalar_struct{
  volatile unsigned int scalar[512];             // 0x000  Control and Status Register
};

struct Read_data{
  //  volatile unsigned int data[DATA_BUF_SIZE];  // 0x1000~0x1FFC  4MB FIFO memory space, (32Kx32)
  volatile unsigned int data[DATA_BUF_SIZE*2];  // 0x1000~0x1FFC +0x2000~0x2FFC(test) 8MB FIFO memory space, (32Kx32 + 32Kx32)
};
