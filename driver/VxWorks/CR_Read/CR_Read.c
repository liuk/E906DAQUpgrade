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
*  Jia-Ye Chen, July 2010
*  Revision  03.0 - New version for Da-Shung CR board
*  Jia-Ye Chen, January 2011
*  Revision  04.0 - New version for Da-Shung CR board version 2
*
*  Changed by Jia-Ye's new code fix the timing problem
*
*  Grass Wang, June 2011
*  Revision  Latch-TDC - New version for Da-Shung CR board Latch-TDC version
*  Grass Wang, July 2016
*  Revision Dual-port memory storage
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

#include "CR_Read.h"

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
volatile struct Read_data* CR_d[DSTDC_MAX_BOARDS];
volatile struct Read_dp_data* CR_dp[DSTDC_MAX_BOARDS];
volatile struct Read_reg_struct* CR_p[DSTDC_MAX_BOARDS];
volatile struct Read_scalar_struct* CR_s[DSTDC_MAX_BOARDS];

/* == Number of ADCs in Crate == */
int M_Buf_size[6] = {1024, 512, 256, 128, 64, 32};
int M_Num_buf[6]  = {1, 2, 4, 8, 16, 32};
struct timespec tim, tim2;

/*******************************************************************************
*
* CR_Init - Initialize
*
* RETURNS: OK, or ERROR if the address is invalid or board is not present.
*
*******************************************************************************/
STATUS CR_Init(UINT32 addr, UINT32 addr_inc, int nmod)
{
  int ii = 0, res, ress, resd, resdp, errFlag = 0;
  unsigned int laddr, laddrd, laddrs, laddrdp;
  unsigned int addrd, addrs, addrdp;

  int NCR;
  addrs = addr + 0x800;
  addrd = addr + 0x1000;
  addrdp = addr + 0x20000;

  /* Check for valid address */
  if(addr == 0) {
    printf("CR_Init: ERROR: Must specify a Bus (VME-based A16) address for the DSTDC\n");
    return(ERROR);
  } else if(addr > 0xffffffff) { /* A32 Addressing */
    printf("CR_Init: ERROR: A32/A24 Addressing not supported for the SIS 3600\n");
    return(ERROR);
  } else {
    /* assume only one ADC to initialize */
    if(addr_inc == 0 || nmod==0) nmod = 1;

    /* get the ADC address (09=original, 0d=blt) */
    /* 0x09 is A32 non privileged data access    */
    res = sysBusToLocalAdrs(0x09, (char*)addr, (char**)&laddr);
    if(res != 0) {
      printf("CR_Init: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr) \n", addr);
      printf("CR_Init: ERROR res=%d\n", res);
      return(ERROR);
    }

    ress = sysBusToLocalAdrs(0x09, (char*)addrs, (char**)&laddrs);
    if(ress != 0) {
      printf("CR_Init: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddrs) \n", addrs);
      printf("CR_Init: ERROR ress=%d\n", ress);
      return(ERROR);
    }

    resd = sysBusToLocalAdrs(0x09, (char*)addrd, (char**)&laddrd);
    if(resd != 0) {
      printf("CR_Init: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddrd) \n", addrd);
      printf("CR_Init: ERROR resd=%d\n", resd);
      return(ERROR);
    }

    resdp = sysBusToLocalAdrs(0x09, (char*)addrdp, (char**)&laddrdp);
    if(resdp != 0) {
      printf("CR_Init: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddrd) \n", addrdp);
      printf("CR_Init: ERROR resd=%d\n", resdp);
      return(ERROR);
    }
  } /* --- IF addr --- */

  NCR = 0;
  for(ii = 0; ii < nmod; ii++) {
    CR_p[ii] = (struct Read_reg_struct*)(laddr + ii*addr_inc);  //registers
    CR_s[ii] = (struct Read_scalar_struct*)(laddrs + ii*addr_inc); //scalers
    CR_d[ii] = (struct Read_data*)(laddrd + ii*addr_inc); //data -- event buffer
    CR_dp[ii] = (struct Read_dp_data*)(laddrdp + ii*addr_inc); //dual-port mem

    ++NCR;
    printf("Initialized DSTDC ID %d at address CR_p 0x%08x \n", ii, (UINT32)CR_p[ii]);
  }

  if(errFlag > 0) {
    printf("CR_Init: ERROR: Unable to initialize all Modules\n");
    if(NCR > 0) printf("CR_Init: %d DSTDC(s) successfully initialized\n", NCR);

    return(ERROR);
  }

  return(OK);
}
/* ========================================================================== */

void CR_TrigDisable(int id)
{
  int tmp = CR_p[id]->reg[1];
  CR_p[id]->reg[1] = tmp & 0xfffff7ff;

  return;
}

void CR_TrigEnable(int id)
{
  int tmp = CR_p[id]->reg[1];
  tmp = tmp & 0xfffff7ff;
  CR_p[id]->reg[1] = tmp + 0x800;

  return;
}

void CR_FastTrigDisable(int id, unsigned int csr)
{
  CR_p[id]->reg[1] = csr & 0xfffff7ff;
  return;
}

void CR_FastTrigEnable(int id, unsigned int csr)
{
  csr = csr & 0xfffff7ff;
  CR_p[id]->reg[1] = csr + 0x800;
  return;
}

void CR_Scalar_Switch(int id, int scalarID)
{
  if(scalarID < 8) {
    int org_regcc = CR_p[id]->reg[3];
    CR_p[id]->reg[3] = (org_regcc & 0x00ffffff) + (scalarID << 24);
  } else {
    printf("ERROR!!  scalarID =%d. it should less than 8!! \n",scalarID);
  }

  return;
}

void CR_KMultiHitsON(int id, int update, int inhibit)
{
  int org_regcc = CR_p[id]->reg[4];
  if(inhibit < 64) {
    CR_p[id]->reg[4] = (org_regcc & 0xffc0ffff) + (inhibit << 16);
  } else {
    printf("ERROR!! inhibit = %d. it should less than 64!! \n", inhibit);
  }

  CR_p[id]->reg[4] |= 0x00800000;
  if(update == 1) {
    CR_p[id]->reg[4] |= 0x00600000;
  } else if (update == 0) {
    CR_p[id]->reg[4] &= (~0x00600000);
  } else {
    printf("ERROR!! update = %d. it should be 1 or 0!! \n", update);
  }

  printf("reg 0xc modified from %8x to %8x\n", org_regcc, CR_p[id]->reg[4]);
  return;
}

void CR_KMultiHitsOFF(int id)
{
  CR_p[id]->reg[4] &= (~0x00800000);
  return;
}

void CR_WR_Reg(int id, int regaddr, int reg_value)
{
  CR_p[id]->reg[regaddr] = reg_value;
  return;
}

int CR_RD_Reg(int id, int regaddr)
{
  return(CR_p[id]->reg[regaddr]);
}

void CR_Clear(int id)
{
  CR_p[id]->reg[3] = 1;

  tim.tv_sec = 0;
  tim.tv_nsec = 1000;
  nanosleep(&tim , &tim2);

  CR_p[id]->reg[3] = 0;
  return;
}

void CR_Reset(int id)
{
  CR_p[id]->reg[3] = 0x1001;

  tim.tv_sec = 0;
  tim.tv_nsec = 1000;
  nanosleep(&tim , &tim2);

  CR_p[id]->reg[3] = 0;
  return;
}

void DelayLoop(int nloop)
{
  int tmp, iloop;
  for(iloop = 0; iloop < nloop; iloop++) {
    tmp = CR_p[0]->reg[1];
  }
  return;
}

int CR_GetNBuf(int csr)
{
  int now_M_value = csr & 0xf;
  if(now_M_value > 5) {
    printf("check M value in csr, M = %d, it should between 0~5\n", now_M_value);
  }

  return(M_Num_buf[now_M_value]);
}

int CR_GetBufSize(int csr)
{
  int now_M_value = csr & 0xf;
  if(now_M_value > 5) {
    printf("check M value in csr,M=%d, it should between 0~5\n",now_M_value);
  }

  return(M_Buf_size[now_M_value]);
}

void CR_HeaderInit(int id, int csr)
{
  int idata, ibuffer;
  int now_M_value = csr & 0xf;
  if(now_M_value > 5) {
    printf("check M value in csr,M=%d, it should between 0~5\n", now_M_value);
  }

  for(ibuffer = 0; ibuffer < M_Num_buf[now_M_value]; ibuffer++) {
    idata = ibuffer*M_Buf_size[now_M_value];
    CR_d[id]->data[idata] = 0;
  }

  return;
}

void CR_ScalarInit(int id, int scalarID)
{
  int ich;
  for(ich = 0; ich < 64; ich++) {
    CR_s[id]->scalar[64*scalarID + ich] = 0;
  }

  return;
}

void DP_Init(int id, int dp_start, int dp_end)
{
  int idata;

  CR_p[id]->reg[6] = 0;
  CR_p[id]->reg[7] = 0;

  for(idata = dp_start; idata <= dp_end; idata++) {
    CR_dp[id]->dp[idata] = 0;
  }

  return;
}

void DP_Write(int id, int value, int dp_start, int dp_end)
{
  int idata;
  for(idata = dp_start; idata <= dp_end; idata++) {
    CR_dp[id]->dp[idata] = value;
  }
  return;
}

int DP_Read(int id, int idata)
{
  return CR_dp[id]->dp[idata];
}

void CR_DataInit(int id, int csr)
{
  int idata;
  for(idata = 0; idata < DATA_BUF_SIZE; idata++) {
    CR_d[id]->data[idata] = 0;
  }

  return;
}

void CR_ScalarDisplay(int id, int scalarID)
{
  int ich, ibuffer;
  for(ich = 0; ich < 64; ich++) {
    ibuffer = 64*scalarID + ich;
    printf("%2d: %d, ", ich, CR_s[id]->scalar[ibuffer]);
    if((ich+1) % 8 == 0) printf("\n");
  }

  return;
}

void CR_DataDisplay(int id)
{
  int idata;
  for(idata = 0; idata < DATA_BUF_SIZE; idata++) {
    if(idata%8==0) {
      printf("\n%4x: ", idata);
    }
    printf("%8x ", CR_d[id]->data[idata]);
  }

  return;
}

/* ========================================================================== */
void CR_Status(int id, int iword)
{
  int ii;
  if(id < 0 || CR_p[id] == NULL) {
    printf("CR_Status: ERROR : DSTDC id %d not initialized \n", id);
    return;
  }

  /* --- Get info from Module --- */
  printf("================================================ \n");
  printf("CR STATUS   id %d at base address 0x%x \n", id, (UINT32)CR_p[id]);
  printf("CR STATUS data %d at base address 0x%x \n", id, (UINT32)CR_d[id]);
  printf("------------------------------------------------ \n");
  for(ii = 0; ii < iword; ii++) {
    printf("CR_Status %d                   : %x\n", ii, CR_p[id]->reg[ii]);
  }
  printf("================================================ \n\n\n");
}

void CR_FifoRead(int id, int ii)
{
  unsigned int res[64];
  if(id < 0 || CR_p[id] == NULL) {
    logMsg("CR_ReadFifo ERROR : DSTDCCARD id %d not initialized \n", id, 0, 0, 0, 0, 0);
    return;
  }

  if(ii >= 64) {
    logMsg("CR_ReadFifo ERROR position on req can not greater than or equal to 64\n", id, 0, 0, 0, 0, 0);
    return;
  }

  printf("=== Reg Read ===\n");
  for(ii = 0; ii < 64; ii++) {
    res[ii] = CR_p[id]->reg[ii] & 0xffffffff;  /* --- CH 01~32 --- */
    printf("res[%3d] = 0x %8x\n", ii, res[ii]);
  }
  printf("\n");

  return;
}

int CR_BroadID(UINT32 addr, UINT32 addr_inc, int id)
{
  int broadid = addr + addr_inc*id;
  return broadid;
}

void writeFile(char* filename, char* InputWord)
{
  FILE* pFile = fopen (filename,"w");
  fprintf(pFile, InputWord);
  fclose(pFile);
}

void checkTDC(int nloop, int csr)
{
  int idata, iInit, iloop, iPrint;
  int id = 0;
  int fineTDC[8];
  for(iInit = 0; iInit < 8; iInit++) {
    fineTDC[iInit] = 0;
  }

  CR_TrigDisable(id);
  for(iloop = 0; iloop < nloop; iloop++) {
    CR_DataInit(id, csr);
    CR_TrigEnable(id);
    sleep(1);
    CR_TrigDisable(id);
    for(idata = 0; idata < DATA_BUF_SIZE; idata++) {
      int header = (CR_d[id]->data[idata] & 0x80000000) >> 31;
      if(CR_d[id]->data[idata] != 0 && header == 0) {
	    fineTDC[CR_d[id]->data[idata] & 0x7]++;
      }
    }///for idata
  }/// for iloop

  for(iPrint = 0; iPrint < 8; iPrint++) {
    printf("bit%d: %d\n", iPrint, fineTDC[iPrint]);
  }

  return;
}

/*
void checkTDC_Ch (int nloop,int csr){
  int idata,iInit,iCh,iloop,iPrint;
  int id=0;
  int fineTDC[16][9];
  for (iInit=0;  iInit<9;  iInit++){
    for (iCh=0; iCh<16;iCh++){
      fineTDC[iCh][iInit]=0;
    }
  }
  CR_TrigDisable(id);

  for (iloop=0; iloop<nloop; iloop++){
    CR_DataInit(id,csr);
    CR_TrigEnable(id);
    sleep(1);
    CR_TrigDisable(id);
    if (iloop%100 ==0) printf(" %d ",iloop);
    if (iloop%1000==0) printf("\n");
    for( idata=0; idata< DATA_BUF_SIZE; idata++){
      int tmpFine,tmpCh;
      int header= (CR_d[id]->data[idata]&0x80000000)>>31;
      if (CR_d[id]->data[idata]!=0 && header==0){
	tmpCh= (CR_d[id]->data[idata]&0x0f000000)>>24;
	tmpFine=CR_d[id]->data[idata]&0xf; ////fine tdc time
	//	printf("%d:org=0x%x,  fineTDC[%d]=%d\n",idata,CR_d[id]->data[idata], tmpFine, fineTDC[tmpFine]);
	fineTDC[tmpCh][tmpFine-1]++;
      }
    }///for idata
  }/// for iloop

  for (iCh=0; iCh<16;iCh++){
    printf("ch%2d: ",iCh);
    for (iPrint=0;  iPrint<9;  iPrint++){
      printf(" %1d:%5d ",iPrint,fineTDC[iCh][iPrint]);
    }
    printf("\n");
  }
  return;
}

void saveTDC (int nloop,int csr){
  int idata,iInit,iCh,iloop,iPrint;
  int id=0;
  int fineTDC[16][9];
  char *filename="tdctest.txt";
  FILE *pFile;

  for (iInit=0;  iInit<9;  iInit++){
    for (iCh=0; iCh<16;iCh++){
      fineTDC[iCh][iInit]=0;
    }
  }
  pFile = fopen (filename,"w+");

  CR_TrigDisable(id);

  for (iloop=0; iloop<nloop; iloop++){
    CR_DataInit(id,csr);
    CR_TrigEnable(id);
    tim.tv_sec = 0;
    tim.tv_nsec = 10000000;

    nanosleep(&tim , &tim2);
    CR_TrigDisable(id);
    //    DelayLoop(100);


    nanosleep(&tim , &tim2);
    if (iloop%100 ==0) printf(" %d ",iloop);
    if (iloop%1000==0) printf("\n");

    for( idata=0; idata< DATA_BUF_SIZE; idata++){
      int tmpFine,tmpCh;
      int tmpData=CR_d[id]->data[idata];
      int header= (tmpData&0x80000000)>>31;
      if (tmpData!=0 && header==0){
	tmpCh= (tmpData&0x0f000000)>>24;
	tmpFine=tmpData&0xf; ////fine tdc time
	//	printf("%d:org=0x%x,  fineTDC[%d]=%d\n",idata,CR_d[id]->data[idata], tmpFine, fineTDC[tmpFine]);
	fineTDC[tmpCh][tmpFine-1]++;
      }

      if (idata%32==0){
	fprintf(pFile,"\n");
	fprintf(pFile,"%4x  ",idata);
      }
      fprintf(pFile,"%8x ",tmpData);


      }///for idata



  }/// for iloop

  for (iCh=0; iCh<16;iCh++){
    printf("ch%2d: ",iCh);
    for (iPrint=0;  iPrint<9;  iPrint++){
      printf(" %1d:%5d ",iPrint,fineTDC[iCh][iPrint]);
    }
    printf("\n");
  }
  fclose (pFile);

  return;
}


void ReadTimeTest (int nloop,int csr){
  int idata,iInit,iCh,iloop;
  int id=0;
  int fineTDC[16][9];
  char *filename="tdctest.txt";
  FILE *pFile;
  int tmpArray[DATA_BUF_SIZE];

  for (iInit=0;  iInit<9;  iInit++){
    for (iCh=0; iCh<16;iCh++){
      fineTDC[iCh][iInit]=0;
    }
  }
  pFile = fopen (filename,"w+");

  CR_TrigDisable(id);

  for (iloop=0; iloop<nloop; iloop++){
    CR_DataInit(id,csr);
    CR_TrigEnable(id);
    tim.tv_sec = 0;
    tim.tv_nsec = 10000000;

    nanosleep(&tim , &tim2);
    CR_TrigDisable(id);
    //    DelayLoop(100);


    nanosleep(&tim , &tim2);
    if (iloop%100 ==0) printf(" %d ",iloop);
    if (iloop%1000==0) printf("\n");



    for( idata=0; idata< DATA_BUF_SIZE; idata++){
      tmpArray[idata]=CR_d[id]->data[idata];
    }

  }/// for iloop

  fclose (pFile);

  return;
}

void CR_Scaler_loop(UINT32 addr,int Nloop, int Nsec){
  int iScaler=0;
  tim.tv_sec = Nsec;

  if (Nloop>=7){
    printf("Nloop should not large than 7\n");
    return;
  }

  CR_Init(addr,0,0);
  CR_Status(0,5);

  CR_Scalar_Switch(0, 8);
  for ( iScaler=0; iScaler<8; iScaler++ ){
    CR_ScalarInit(0, iScaler);
  }

  for ( iScaler=0; iScaler<Nloop; iScaler++ ){
    CR_Scalar_Switch(0, iScaler);
    nanosleep(&tim , &tim2);
  }

  CR_Scalar_Switch(0, Nloop);
  for ( iScaler=0; iScaler<Nloop; iScaler++ ){
    printf("scaler %d:========================\n",iScaler);
    CR_ScalarDisplay(0, iScaler);
  }

  return;
}



/*

void checkTDC_Event (int nloop=1, int Debug=0){
  int idata,iInit,iCh,iloop,iPrint;
  int id=0;
  int fineTDC[16][8];
  int modeM= CR_p[id]->reg[1] & 0x7;

  int NEvent[6]={1,2,4,8,16,32};
  int NBufLength[6]={1024,512,256,128,64,32};
  int iEvent, iBuf, NBuf;
  int iShift;
  for (iInit=0;  iInit<8;  iInit++){
    for (iCh=0; iCh<16;iCh++){
      fineTDC[iCh][iInit]=0;
    }
  }

  CR_TrigDisable(id);

  for (iloop=0; iloop<nloop; iloop++){
    CR_DataInit(id);
    CR_TrigEnable(id);
    sleep(1);
    CR_TrigDisable(id);


    for (iEvent=0; iEvent<NEvent[modeM]; iEvent++){


      tmpM= iEvent << modeM;
      for (iShift=0; iShift < modeM; iShift++){


      }

      idata = iEvent *  NBufLength[modeM] ;

      NBuf=(CR_d[id]->data[idata]& 0x7ff00000)>20;
      if (NBuf>  NBufLength[modeM]){
	printf("ERROR!!! Nbuf is wrong, header = 0x%x, NBuf= %x, MaxNbuf= %x\n", CR_d[id]->data[idata],NBuf,  NBufLength[modeM]);
	return;
      }
      if (Debug==1) printf("event# %d, header : 0x%x\n",iEvent, (CR_d[id]->data[idata]);

      for( iBuf=1; iBuf<  NBuf ; iBuf++){

	idata = ( iEvent *  NBufLength[modeM] ) + iBuf;
	int tmpFine,tmpCh;
	int header= (CR_d[id]->data[idata]&0x80000000)>>31;
	if (CR_d[id]->data[idata]!=0 && header==0){
	  tmpCh= (CR_d[id]->data[idata]&0x0f000000)>>24;
	  tmpFine=CR_d[id]->data[idata]&0x7; ////fine tdc time
	  //	printf("%d:org=0x%x,  fineTDC[%d]=%d\n",idata,CR_d[id]->data[idata], tmpFine, fineTDC[tmpFine]);

	  if(Debug==1){
	    if (idata%8==0){
	      printf("\n");
	      printf("%4x: ",idata);
	    }
	    printf(" %8x ",CR_d[id]->data[idata]);
	  }

	  fineTDC[tmpCh][tmpFine]++;
	}
      }///for idata
    }
  }/// for iloop

  for (iCh=0; iCh<16;iCh++){
    printf("ch%2d: ",iCh);
    for (iPrint=0;  iPrint<8;  iPrint++){
      printf(" %1d:%5d ",iPrint,fineTDC[iCh][iPrint]);
    }
    printf("\n");
  }
  return;
}

*/
