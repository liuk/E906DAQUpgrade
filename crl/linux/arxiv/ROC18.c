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
#include<sys/time.h>

struct timeval tv;

/* Globals to be filled by usrstrutils */
int string1=0, string2=0; /* defined when string is present in CODA database */
int ps1=0,ps2=0,ps3=0,ps4=0,ps5=0,ps6=0,ps7=0,ps8=0; /* defined in "ffile" */

/* function prototype */
void rocTrigger(int arg);

#include "/home/e906daq/2.6.1/extensions/linuxvme/include/CR_Read.h"

extern struct Read_reg_struct *CR_p[];
extern struct Read_data *CR_d[];
extern struct Read_scalar_struct*CR_s[];

#include "/usr/local/coda/2.6.1/extensions/e906/SL_ScalerLatcher.h"
extern struct SL_data *SL_d;
extern struct SL_csr *SL_p;
int Clear64[64];

void Clear64Init()
{
  int iclean=0;
  for (iclean=0;iclean<64;iclean++){
    Clear64[iclean]=0;
  }
}



const int  TDC_ScalarON=1;
const int NTDC=7;
int ii;

int TDCBoardID= 0x09000000;
// //  csr = 0xffff0060 /rising edge;  0xffff0160 /both edge         / 2048ns /only 1 buffer                  \

int csr=0xffff0060;
int MultiHitSetup=0x0;
int TimeWindowOn=1;
int UP_Limit=398;
int Low_Limit= 343;
int LimitReg;
int TDCSetup;
int TDCHardID[7]={28, 26, 30, 32, 33, 34, 128};



void
rocDownload()
{

  /* Setup Address and data modes for DMA transfers
   *   
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2,2,0); 



  /*
  
  init_strings();
  string1 = getflag("string");
  string2 = getflag("string2");

  printf("usrstrutils configuration:\n");
  printf("\tstring1 = %d\n\tstring2 = %d\n",string1,string2);
*/
  printf("rocDownload: User Download Executed\n");

}

void
rocPrestart()
{
  unsigned short iflag;
  int stat;

  /* Check ffile for changes (usrstrutils) 
     Useful for items that may change without re-download (e.g. prescale factors) */
  /*  init_strings();
  ps1 = getint("ps1");
  printf("usrstrutils configuration:\n");
  printf("\tps1 = %d\n",ps1);
  */
 UEOPEN(132,BT_UI4,0);    
 
  /* Program/Init VME Modules Here */
  LimitReg=  Low_Limit+ (UP_Limit<<16)+(TimeWindowOn<<31);  
  CR_Init(0x09000000,0x1000000,NTDC);
  //   tirIntInit(TIR_ADDR,TIR_TS_INT,0);

  //  *dma_dabufp++ =rol->pid;                                                                              
  *rol->dabufp++ =rol->pid;            
  for (ii=0;ii<NTDC;ii++){
    CR_Reset(ii);   
    *rol->dabufp++  =0xe906f011; //FEE event flag                                                                            
    *rol->dabufp++  =TDCBoardID+(0x1000000*ii)+TDCHardID[ii];  //Board ID                                                           
    
    *rol->dabufp++  =csr;  //csr (buffersize, timewindow length, etc)                                                                                 
    *rol->dabufp++ =MultiHitSetup;                                                                       
    *rol->dabufp++   =LimitReg;                                                                            
    CR_WR_Reg(ii,5,LimitReg);
    CR_WR_Reg(ii,3,MultiHitSetup); //turn off MultiHit Elimination                                       
    if (TDC_ScalarON==1){
      CR_Scalar_Switch(ii,2);                                                                                
      CR_ScalarInit(ii,2);                                                                                   
      CR_ScalarInit(ii,1);                                                                            
    }                                                                                                    
  }  //for ntdc                                                 

  printf("rocPrestart: User Prestart Executed\n");
  UECLOSE;            
}

void
rocGo()
{
  /* Enable modules, if needed, here */


  for (ii=0;ii<NTDC;ii++){                                                                                 
    vmeWrite32(&CR_p[ii]->reg[1],csr);
    CR_TrigEnable(ii);

    //    CR_FastTrigEnable(ii,csr);                                                                             
    if (TDC_ScalarON==1){                                                                                  
      CR_ScalarDisplay(ii,1);//show buffer0 (should be empty at go)                              
      CR_Scalar_Switch(ii,1);//set pointer to buffer0 to start filling                                 
    }                                                                                                      
  }                                                        

  /* Interrupts/Polling enabled after conclusion of rocGo() */
  //  tirIntEnable(1);

}

void
rocEnd()
{
  for (ii=0;ii<NTDC;ii++){                                                                                 
    CR_FastTrigDisable(ii,csr);                                                                          
    if (TDC_ScalarON==1){                                                                                
      CR_Scalar_Switch(ii,2); //using buffer0, so set pointer to buffer1 to stop writing to buffer0      
      CR_ScalarDisplay(ii,1); //show buffer0                                                            
    }                                                                                                    
  }       

  //      tirIntDisable();   
  printf("rocEnd: Ended after %d events\n",tirGetIntCount());
  
}

void
rocTrigger(int arg)
{
  int iii,data_addr,retVal,maxWords,nWords,remBytes;
    int TmpData;                                                                                             
  int iWord,iiWord,Cnt,totalDMAwords;                                                                     
  int firstDMAflag;                                                                                       
  int iHeader;                                                                                    
  unsigned int *DMAaddr;                                                                                  
  long tmpaddr1,tmpaddr2,tmpaddr3;           
                                                                      
  //  rol->dabufp = (long *) 0;                                                                               
  
  unsigned int event_ty=0, event_no=0;

  event_ty = EVTYPE;
  event_no = EVNUM;
  EVENTOPEN(event_ty, BT_UI4);


  extern DMANODE *the_event;
  extern unsigned int *dma_dabufp;
 
  *dma_dabufp++ =LSWAP(rol->pid);                                                                              
  *dma_dabufp++ =LSWAP(0X000e0100);
  *dma_dabufp++ = LSWAP(0xe9060000);

  //   Example: Raise the 0th (1<<0) and 2nd (1<<2) output level on the TI 
  //  tirIntOutput(1<<0 | 1<<2);
  
  if (event_ty ==14){                                                                                      
    for (ii=0;ii<NTDC;ii++){                                                                        
                CR_FastTrigDisable(ii,csr);//when trigger arrives at TDC, disable further trigger input             
    } 

                                                                                                         
    for (ii=0;ii<NTDC;ii++){                                                                               
                                                                                                         
      // data scaler flag=3, ignore = 0, latch=1, tdc=2,dsTDC2 flag=4, v1495=5,ZStdc=6,noZSWC=7,ZSWC=8,        
      //  Run2TDC= 10, Run2TDC header = 11                                                            ,        
      *dma_dabufp++ =LSWAP(0xe906f010); // run2 TDC                                                              
      //    *dma_dabufp++ = CR_s[0]->scalar[0];                                                                   
      totalDMAwords=0;                                                                                     
      iiWord=0;                                                                                            
      firstDMAflag=0;                                                                                      
                                                                                                         
      iWord= 0;                                                                         
      iHeader= vmeRead32(&CR_d[ii]->data[iWord]);//trigger word (including numwords in trigger block)               
      //Cnt=(iHeader& 0x7ff00000)>>20;//numwords                                                      
      Cnt=(iHeader& 0x7ff00000)>>20;//numwords                                                      
      *dma_dabufp++ = LSWAP(((TDCBoardID+(0x1000000*ii))&0xffff0000)+Cnt );//boardid, numwords                  
      //      *dma_dabufp++ = LSWAP(iHeader);
////////////Address only-mode
      /*
     int ihit;
      for (ihit=0; ihit<Cnt;ihit++){
	
	*dma_dabufp++ = LSWAP(vmeRead32(&CR_d[ii]->data[ihit]));
      }
      */
      
      
      //////////////////////////////////////////////
      if(Cnt>=2 ){ //if so, then there's data                         
                             

                                                                            
	////////// DMA loop////////////                                                                          
	maxWords =Cnt;  /// normal data taking        
	DMAaddr=TDCBoardID+(0x1000000*ii)+0x1000+((iWord)<<2);      /// normal data taking             

	tmpaddr1=dma_dabufp;                                                                              
	tmpaddr2=DMAaddr;      
	                                                                            
	if ((( tmpaddr1&4)>>2)!=((tmpaddr2&4)>>2)){                                                  
	  *dma_dabufp++ =LSWAP(0xe906e906);                                                           
	}      
	
	retVal = vmeDmaSend(dma_dabufp,DMAaddr,(maxWords<<2));                      

	if(retVal < 0) {                                                                             
	  logMsg("ERROR in DMA transfer Initialization 0x%x\n",retVal,0,0,0,0,0);    
	  *(dma_dabufp)++ = LSWAP(0xda010bad);                                               
	} else {                                                                                     
	  remBytes = vmeDmaDone();  
	  //      logMsg("DMA transfer Done 0x%x\n",nWords,0,0,0,0,0);                                  
	  if(remBytes < 0) {                    //Error//                             
	    logMsg("ERROR during DMA transfer 0x%x\n",0,0,0,0,0,0);             
	    *(dma_dabufp)++ = LSWAP(0xda020bad);                                         
	  }  else if(remBytes == 0) {        //Transfer completed //                    
	    dma_dabufp += maxWords;                                              
	  } else {                            //Transfer Terminated //                  
	    nWords = (remBytes>>2);                                   
	    dma_dabufp+= nWords; 
	  }                                                                                    
	}//retVal <0       

	///////////Final DMA loop/////////////////////////   
      }//if iCnt >=2                                   
      
   
                                                        
    }//for NTDC                                          
   
    *dma_dabufp++ =LSWAP(0xe906c0da);
    
  }//for evtype==14
 

    EVENTCLOSE;

}

void 
rocDone()
{
  int event_ty = EVTYPE;                                                                                     

  //  int event_ty=14;
  if(event_ty ==14){         

                                                       
    for (ii=0;ii<NTDC;ii++){                                                                               
      // dsTdcFifoClear(ii);                                                                                
      CR_HeaderInit(ii, csr); //Clear header (trigger word)                                                 
    }                                                                                                      
    for (ii=0;ii<NTDC;ii++){                                                                               
      CR_FastTrigEnable(ii,csr); //Re-initialize TDC trigger accept                                          
    }                                                                                                      
  }       

  //  tirIntAck(); 
  // tirIntEnable();
} 

void
rocCleanup()
{
  // Put any clean up code that should be done before the next readout list is downloaded 
  commonCleanup();

}


