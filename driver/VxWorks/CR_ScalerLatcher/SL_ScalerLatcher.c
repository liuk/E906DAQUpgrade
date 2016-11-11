/******************************************************************************
*
*  Dslatchcard -  Driver library for operation of DaShung's I/O Latch
*                 using a VxWorks 5.4 or later based Single Board computer.
*
*  Author: David Abbott
*          Jefferson Lab Data Acquisition Group
*          April 2003
*
*  Revision  1.0 - Initial Revision
*

*  Grass Wang, Aug. 2013
*  Revision  ScalerLatcher - New version for Da-Shung CR board Latch-TDC version
*
*/

#include "vxWorks.h"
#include "stdio.h"
#include "string.h"
#include "logLib.h"
#include "taskLib.h"
#include "intLib.h"
#include "iv.h"
#include "semLib.h"
#include "vxLib.h"
#include "unistd.h"
#include "time.h"

#include "SL_ScalerLatcher.h"

/* Define external Functions */
IMPORT STATUS sysBusToLocalAdrs(int, char *, char **);
IMPORT STATUS intDisconnect(int);
IMPORT STATUS sysIntEnable(int);
IMPORT STATUS sysIntDisable(int);
IMPORT STATUS sysVmeDmaSend(UINT32, UINT32, int, BOOL);

void writeFile(char* filename, char* InputWord);
void DelayLoop(int nloop);

/* Define global variables */
/* == pointers to TDC memory map == */
volatile struct SL_data* SL_d[2];   //data
volatile struct SL_csr*  SL_p;      //registers
volatile struct SL_DP_data* SL_DP;  //DP

/* == Number of ADCs in Crate == */
int M_Buf_size[6] = {1024, 512, 256, 128, 64, 32};
int M_Num_buf[6]  = {1, 2, 4, 8, 16, 32};
struct timespec tim, tim2;

/*******************************************************************************
*
* SL_Init - Initialize SIS 3600 Library.
*
* RETURNS: OK, or ERROR if the address is invalid or board is not present.
*
*******************************************************************************/
STATUS SL_Init(UINT32 addr)
{
  int ii = 0, res, resd, resdp, errFlag = 0;
  unsigned int laddr, laddrd, laddrdp;
  unsigned int addrd, addrdp;
  UINT32 addr_inc = 0x800;

  addrd = addr + 0x800;
  addrdp = addr + 0x20000;

  /* Check for valid address */
  if(addr == 0) {
    printf("SL_Init: ERROR: Must specify a Bus (VME-based A16) address for the ScalerLatcher\n");
    return(ERROR);
  } else if(addr > 0xffffffff) { /* A32 Addressing */
    printf("SL_Init: ERROR: A32/A24 Addressing not supported for the SIS 3600\n");
    return(ERROR);
  } else {
    /* get the ADC address (09=original, 0d=blt) */
    /* 0x09 is A32 non privileged data access    */
    res = sysBusToLocalAdrs(0x09, (char*)addr, (char**)&laddr);
    if(res != 0) {
      printf("SL_Init: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr) \n", addr);
      printf("SL_Init: ERROR res=%d\n", res);
      return(ERROR);
    }

    resd = sysBusToLocalAdrs(0x09, (char*)addrd, (char**)&laddrd);
    if(resd != 0) {
      printf("SL_Init: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddrd) \n", addrd);
      printf("SL_Init: ERROR resd=%d\n", resd);
      return(ERROR);
    }

    resdp = sysBusToLocalAdrs(0x09, (char*)addrdp, (char**)&laddrdp);
    if(resdp != 0) {
      printf("SL_Init: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddrd) \n", addrdp);
      printf("SL_Init: ERROR resdp=%d\n", resdp);
      return(ERROR);
    }
  } /* --- IF addr --- */

  SL_p = (struct SL_csr*)(laddr);
  SL_DP = (struct SL_DP_data *)(laddrdp);
  for(ii = 0; ii < 2; ii++) {
    SL_d[ii] = (struct SL_data*)(laddrd + ii*addr_inc);
  }
  printf("Initialized ScalerLatcher at address SL_p 0x%08x \n", (UINT32)SL_p);

  if(errFlag > 0) {
    printf("SL_Init: ERROR: Unable to initialize all Modules\n");
    return(ERROR);
  }

  return(OK);
}
/* ========================================================================== */

void SL_Scalar_Switch(int scalarID)
{
  int org_regcc = SL_p->csr[3];
  if(scalarID < 8) {
    SL_p->csr[3] = (org_regcc & 0x00ffffff) + (scalarID << 24);
  } else {
    printf("ERROR!!  scalarID =%d. it should less than 8!! \n", scalarID);
  }

  return;
}

void SL_WR_Reg(int regaddr, int reg_value)
{
  SL_p->csr[regaddr] = reg_value;
  return;
}

int SL_RD_Reg(int regaddr)
{
  return(SL_p->csr[regaddr]);
}

void SL_ShortDataInit(int NWordInit)
{
  int idata;
  for(idata = 0; idata < NWordInit; idata++) {
    SL_d[0]->data[idata] = 0;
  }

  return;
}

void SL_DataInit()
{
  int idata;
  for(idata = 0; idata < SL_DATA_BUF_SIZE; idata++) {
    SL_d[0]->data[idata] = 0;
    SL_d[1]->data[idata] = 0;
  }

  return;
}

void SL_ScalarDisplay()
{
  int iword;
  for(iword = 0; iword < SL_DATA_BUF_SIZE; iword++) {
    printf(" %8x:%8x ,", SL_d[0]->data[iword], SL_d[1]->data[iword]);
    if((iword+1) % 4 ==0) {
      printf("\n");
      printf("%4x: ", iword);
    }
  }
  return;
}

void SL_ShortDataDisplay(int part, int B_ID)
{
  int idata;
  for(idata = 0; idata < SL_DATA_BUF_SIZE; idata++) {
    if(idata%8 == 0) {
      printf("\n");
      printf("%4x: ", idata);
    }
    if((idata < (B_ID+1)*SL_DATA_BUF_SIZE/8) && (idata > B_ID*SL_DATA_BUF_SIZE/8)) printf("%8x ", SL_d[part]->data[idata]);
  }
  return;
}

void SL_DataDisplay(int part)
{
  int idata;
  for(idata = 0; idata < SL_DATA_BUF_SIZE; idata++) {
    if(idata % 8 == 0) {
      printf("\n");
      printf("%4x: ", idata);
    }
    printf("%8x ", SL_d[part]->data[idata]);
  }
  return;
}

/* ============Dual - port mem ============================================== */
void SL_DP_Init(int dp_start, int dp_end)
{
  int idata = 0;
  for(idata = dp_start; idata <= dp_end; ++idata)
  {
    SL_DP->dp[idata] = 0;
  }

  SL_p->csr[7] = 0;
}

int SL_DP_Read(int idata)
{
  return SL_DP->dp[idata];
}

void SL_DP_Write(int value, int dp_start, int dp_end)
{
  int idata;
  for(idata = dp_start; idata <= dp_end; ++idata)
  {
    SL_DP->dp[idata] = value;
  }
}

/* ========================================================================== */
void SL_Status()
{
  int ii;
  int iword = 4;
  if(SL_p == NULL) {
    printf("SL_Status: ERROR : SL not initialized \n");
    return;
  }

  /* --- Get info from Module --- */
  printf("================================================ \n");
  printf("SL STATUS  at base address 0x%x \n", (UINT32)SL_p);
  printf("------------------------------------------------ \n");
  for(ii = 0; ii < iword; ii++) {
    printf("SL_Status 0x%x                   : %x\n",ii*4,SL_p->csr[ii]);
  }
  printf("================================================ \n\n\n");
}

void writeFile(char* filename, char* InputWord)
{
  FILE* pFile = fopen (filename,"w");
  fprintf (pFile, InputWord);
  fclose (pFile);
}
