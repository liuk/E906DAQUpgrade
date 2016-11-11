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

STATUS SL_Init (UINT32 addr);
void SL_Status();
void SL_DataInit(int csr);
void SL_Scalar_Switch(int scalarID);
void SL_WR_Reg(int regaddr, int reg_value);
int SL_RD_Reg(int regaddr);
void SL_ScalarDisplay();
void SL_DataDisplay(int part);


/* --- Define Structure for access to Local Memory map --- */

#define SL_DATA_BUF_SIZE 512
struct csr_struct{
  volatile unsigned int csr[4];             // 0x00c  Control and Status Register
};

struct Read_data{
  //  volatile unsigned int data[DATA_BUF_SIZE];  // 0x1000~0x1FFC  4MB FIFO memory space, (32Kx32)
  volatile unsigned int data[SL_DATA_BUF_SIZE];  // 0x800~0xFFC   0x1000~0x1800  4MB FIFO memory space, (32Kx32 + 32Kx32)
};



