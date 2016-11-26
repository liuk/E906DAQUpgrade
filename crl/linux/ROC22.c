/*************************************************************************
 *
 *  vme_list.c - Library of routines for readout and buffering of
 *                events using a JLAB Trigger Interface (TI) with
 *                a Linux VME controller.
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     400
#define MAX_EVENT_LENGTH   1024*40      /* Size in Bytes */



/* Define Interrupt source and address */
#define TIR_SOURCE
#define TIR_ADDR 0x0ed0
/* TIR_MODE:  0 : interrupt on trigger,
              1 : interrupt from Trigger Supervisor signal
              2 : polling for trigger
              3 : polling for Trigger Supervisor signal  */
#define TIR_MODE 3

#include <linuxvme_list.c> /* source required for CODA */
//#include "usrstrutils.c"   /* helper routines to pass data from mSQL to ROC */
#include "common_list.h"   /* OS independent calls */
#include <sys/time.h>

/* function prototype */
void rocTrigger(int arg);

#include "/home/e906daq/2.6.1/extensions/linuxvme/include/CR_Read.h"

extern struct Read_reg_struct    *CR_p[];
extern struct Read_data          *CR_d[];
extern struct Read_dp_data       *CR_dp[];
extern struct Read_scalar_struct *CR_s[];

#include "/usr/local/coda/2.6.1/extensions/e906/SL_ScalerLatcher.h"
extern struct SL_data *SL_d;
extern struct SL_csr  *SL_p;
int Clear64[64];

void Clear64Init()
{
  int iclean = 0;
  for(iclean = 0; iclean < 64; iclean++) {
    Clear64[iclean] = 0;
  }
}

#define NTDC 6

const int TDC_ScalarON = 1;
const int NFlushMax = 9000;
const int blkSize = 300;
const int PerEventRead = 2;
const int maxWaitCycle = 35;

int event_no;
int event_ty;
int ii;

int ARM_enable = 1;
int UP_Limit = 395;
int Low_Limit = 340;
int TDCHardID[6] = {36, 37, 38, 39, 40, 41};

int TDCBoardID = 0x09000000;
int csr = 0xffff0060;  // csr = 0xffff0060 /rising edge;  0xffff0160 /both edge / 2048ns /only 1 buffer
int MultiHitSetup = 0x0;
int TimeWindowOn = 1;
int LimitReg;
int TDCSetup;

int nFlushes = 0;
int BeamOn = 1;
int EOSIssued = 0;
int busyFlag[NTDC];

void rocDownload()
{

  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2, 2, 0);

  /*
  init_strings();
  string1 = getflag("string");
  string2 = getflag("string2");

  printf("usrstrutils configuration:\n");
  printf("\tstring1 = %d\n\tstring2 = %d\n",string1,string2);
  */
  printf("rocDownload: User Download Executed\n");

}

void rocPrestart()
{
  UEOPEN(132, BT_UI4, 0);

  /* Program/Init VME Modules Here */
  LimitReg = Low_Limit + (UP_Limit << 16) + (TimeWindowOn << 31) + (ARM_enable << 27);
  CR_Init(TDCBoardID, 0x1000000, NTDC);

  *rol->dabufp++ = rol->pid;
  for(ii = 0; ii < NTDC; ii++) {
    *rol->dabufp++ = 0xe906f011; //FEE event flag
    *rol->dabufp++ = TDCBoardID + (ii << 24) + TDCHardID[ii];  //Board ID

    CR_Reset(ii);
    *rol->dabufp++ = csr;  //csr (buffersize, timewindow length, etc)
    *rol->dabufp++ = MultiHitSetup;
    *rol->dabufp++ = LimitReg;
    CR_WR_Reg(ii, 5, LimitReg);
    CR_WR_Reg(ii, 3, MultiHitSetup); //turn off MultiHit Elimination
    CR_WR_Reg(ii, 6, 0);
    DP_Write(ii, 0xe906000f, 0x7ffe, 0x7ffe);   //reset ARM

    if(TDC_ScalarON == 1) {
      CR_Scalar_Switch(ii, 2);
      CR_ScalarInit(ii, 2);
      CR_ScalarInit(ii, 1);
    }
  }  //for ntdc

  UECLOSE;
  printf("rocPrestart: User Prestart Executed\n");
}

void rocGo()
{
  /* Enable modules, if needed, here */
  BeamOn = 1;
  for(ii = 0; ii < NTDC; ++ii) CR_WR_Reg(ii, 7, 0);
  for(ii = 0; ii < NTDC; ii++) {
    vmeWrite32(&CR_p[ii]->reg[1], csr);
    DP_Write(ii, 0xe9068000 + blkSize, 0x7ffe, 0x7ffe);
    CR_TrigEnable(ii);
    if(TDC_ScalarON == 1) {
      CR_ScalarDisplay(ii, 1);//show buffer0 (should be empty at go)
      CR_Scalar_Switch(ii, 1);//set pointer to buffer0 to start filling
    }
  }

  printf("rocGo: User Go Executed\n");
}

void rocEnd()
{
  for(ii = 0; ii < NTDC; ii++) {
    CR_FastTrigDisable(ii, csr);
    if(TDC_ScalarON == 1) {
      CR_Scalar_Switch(ii, 2); //using buffer0, so set pointer to buffer1 to stop writing to buffer0
      CR_ScalarDisplay(ii, 1); //show buffer0
    }
  }

  printf("rocEnd: Ended after %d events\n", tirGetIntCount());
}

void rocTrigger(int arg)
{
  int ii, jj, retVal, maxWords, nWords, remBytes;
  int Cnt, totalDMAwords;
  int ARMbusy, busyLoop;
  int DP_Bank;
  unsigned int* DMAaddr;
  long tmpaddr1, tmpaddr2;

  event_ty = EVTYPE;
  event_no = EVNUM;
  EVENTOPEN(event_ty, BT_UI4);
  extern DMANODE* the_event;
  extern unsigned int* dma_dabufp;

  *dma_dabufp++ = LSWAP(rol->pid);
  *dma_dabufp++ = LSWAP(0x000e0100);
  *dma_dabufp++ = LSWAP(0xe9060000);

  if(event_ty == 14 && BeamOn == 1) { //Physics event
    DP_Bank = ((event_no - 1) & 0xf) << 10;
    for(ii = 0; ii < NTDC; ii++){
      //CR_FastTrigDisable(ii, csr); //when trigger arrives at TDC, disable further trigger input
      busyFlag[ii] = 1;
    }

    for(ii = 0; ii < NTDC; ii++) {
      // data scaler flag=3, ignore = 0, latch=1, tdc=2,dsTDC2 flag=4, v1495=5,ZStdc=6,noZSWC=7,ZSWC=8,
      //  Run2TDC= 10, Run2TDC header = 11                                                            ,
      *dma_dabufp++ = LSWAP(0xe906f010); // run2 TDC
      *dma_dabufp++ = LSWAP(TDCBoardID + (ii << 24));
      // *dma_dabufp++ = LSWAP(event_no - 1);

      if(PerEventRead == 1) {
        maxWords = 257;
        DMAaddr = TDCBoardID + (ii << 24) + 0x20000 + (DP_Bank << 2);

        tmpaddr1 = dma_dabufp;
        tmpaddr2 = DMAaddr;
        if(((tmpaddr1 & 4) >> 2) != ((tmpaddr2 & 4) >> 2)) *dma_dabufp++ = LSWAP(0xe906e906);

	    retVal = vmeDmaSend(dma_dabufp, DMAaddr, maxWords << 2);
	    if(retVal < 0) {
	      logMsg("ERROR in DMA transfer Initialization 0x%x\n",retVal,0,0,0,0,0);
	      *dma_dabufp++ = LSWAP(0xda010bad);
	    } else {
	      remBytes = vmeDmaDone();
	      if(remBytes < 0) {                    //Error//
	        logMsg("ERROR during DMA transfer 0x%x\n",0,0,0,0,0,0);
	        *dma_dabufp++ = LSWAP(0xda020bad);
	      } else if(remBytes == 0) {        //Transfer completed //
	        dma_dabufp += maxWords;
	      } else {                            //Transfer Terminated //
	        nWords = maxWords - (remBytes >> 2);
	        dma_dabufp += nWords;
	      }
	    }//retVal <0
      } else if(PerEventRead == 2) {
        *dma_dabufp++ = LSWAP(vmeRead32(&CR_d[ii]->data[0]));
        //for(jj = 0; jj < 16; ++jj) *dma_dabufp++ = LSWAP(DP_Read(ii, jj << 10));
      }
    }//for NTDC

    //Wait for the interupt to be over
    ARMbusy = 1;
    busyLoop = 0;
    while(ARMbusy > 0 && busyLoop < maxWaitCycle) {
      ARMbusy = 0;
      for(ii = 0; ii < NTDC; ++ii) {
        //if(busyFlag[ii] == 0) continue;

        ++busyLoop;
        busyFlag[ii] = ((DP_Read(ii, DP_Bank) & 0x80000000) >> 31);
        ARMbusy = ARMbusy + busyFlag[ii];
      }
    }

    *dma_dabufp++ = LSWAP(busyLoop);
    *dma_dabufp++ = LSWAP(0xe906c0da);
  } else if(event_ty == 12) { //EOS event
    for(ii = 0; ii < NTDC; ++ii) DP_Write(ii, 0xe9060001, 0x7ffe, 0x7ffe);
    nFlushes = 0;
    BeamOn = 0;
    EOSIssued = 1;
    logMsg("Received EOS event! Will start off-beam transfer... \n");
  } else if(event_ty == 11) { //BOS event
    for(ii = 0; ii < NTDC; ++ii) DP_Write(ii, 0xe9060000, 0x7ffe, 0x7ffe);
    BeamOn = 1;
    EOSIssued = 0;
    logMsg("Received BOS event! Will start on-beam transfer... \n");
  } else if(event_ty == 10 && nFlushes < NFlushMax && BeamOn == 0) {
    ++nFlushes;

    for(ii = 0; ii < NTDC; ++ii) {
      Cnt = DP_Read(ii, 0);
      DMAaddr = TDCBoardID + (ii << 24) + 0x20000;
      *dma_dabufp++ = LSWAP(0xe906f018);
      *dma_dabufp++ = LSWAP(TDCBoardID + (ii << 24) + Cnt);

      if(EOSIssued == 1 && Cnt > 0 && Cnt < blkSize + 10) {
        //printf("Will transfer %d words from TDC %d\n", Cnt, ii);

        tmpaddr1 = dma_dabufp;
        tmpaddr2 = DMAaddr;
        if(((tmpaddr1 & 4) >> 2) != ((tmpaddr2 & 4) >> 2)) *dma_dabufp++ = LSWAP(0xe906e906);

  	    retVal = vmeDmaSend(dma_dabufp, DMAaddr, Cnt << 2);
  	    if(retVal < 0) {
  	      logMsg("ERROR in DMA transfer Initialization 0x%x\n",retVal,0,0,0,0,0);
  	      *dma_dabufp++ = LSWAP(0xda010bad);
  	    } else {
  	      remBytes = vmeDmaDone();
  	      if(remBytes < 0) {                    //Error//
  	        logMsg("ERROR during DMA transfer 0x%x\n",0,0,0,0,0,0);
  	        *dma_dabufp++ = LSWAP(0xda020bad);
  	      } else if(remBytes == 0) {        //Transfer completed //
  	        dma_dabufp += Cnt;
  	      } else {                            //Transfer Terminated //
  	        nWords = Cnt - (remBytes >> 2);
  	        dma_dabufp += nWords;
  	      }
        }
      }

      if(nFlushes < NFlushMax) {
        DP_Write(ii, 0xe9060002, 0x7ffe, 0x7ffe);
      } else {
        DP_Write(ii, 0xe9060003, 0x7ffe, 0x7ffe);
      }
    }

    if(nFlushes < NFlushMax) {
      EOSIssued = 1;
    } else {
      EOSIssued = 0;
    }

    *dma_dabufp++ = LSWAP(0xe906c0da);
  }

  EVENTCLOSE;
}

void rocDone()
{
  if(event_ty == 14 || event_ty == 11) {
    //for(ii = 0; ii < NTDC; ii++) CR_HeaderInit(ii, csr); //Clear header (trigger word)
    for(ii = 0; ii < NTDC; ii++) CR_WR_Reg(ii, 7, event_no);
    //for(ii = 0; ii < NTDC; ii++) CR_FastTrigEnable(ii,csr); //Re-initialize TDC trigger accept
  }
}

void rocCleanup()
{
  // Put any clean up code that should be done before the next readout list is downloaded
  commonCleanup();
}
