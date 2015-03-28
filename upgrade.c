//==========================================================================
//
//      net/upgrade.c
//
//      upgrade support for Sernet
//
//==========================================================================

//#####DESCRIPTIONBEGIN####
//
// Author(s):    snowel
// Contributors: snowel
// Date:         2003-12-16
// Purpose:      
// Description:  
//              
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <redboot.h>
#include <net/net.h>
#include <cyg/io/flash.h>
//#include <cyg/hal/ixdp425.h>
//#include <cyg/hal/grg.h>
#include "strata.h"
#include <iob.h>

#ifdef CYGSEM_REDBOOT_FLASH_CONFIG
#error "define CYGSEM_REDBOOT_FLASH_CONFIG"
#endif
#ifdef CYGOPT_REDBOOT_FIS 
#error "define CYGOPT_REDBOOT_FIS "
#endif

/*enable this for remembering mac address
 when erase all*/
#define REMEMBER_MAC 

//#define DONGLE
//#define V6
#define EFG120V2
//#define EG808

#define FLASH_ADDR_BASE      0x50000000 //after remap

#if defined (EFG120V2) || defined (EG808) || defined (V6)
//#define FLASH_SIZE           0x1000000   //16M intel
#define FLASH_SIZE           0x100000   //1M only for test
#else
#define FLASH_SIZE           0x800000   //8M intel
#endif

#define BLOCK_SIZE           0x20000

#define REDBOOT_SIZE         0x40000

#ifdef EFG120V2
#define NVRAM_SIZE           0x40000
#else
#define NVRAM_SIZE           0x20000
#endif

#define NODE_ADDRESS         (REDBOOT_SIZE - 0x50)
#define PID_OFFSET           (REDBOOT_SIZE - 0x46)

#define PRODUCT_ID_OFFSET    (FLASH_SIZE - 0x10)
#define PROTOCOL_ID_OFFSET   (PRODUCT_ID_OFFSET + 0x02)
#define FW_VERSION_OFFSET    (PRODUCT_ID_OFFSET + 0x04)
#define SIGN_OFFSET          (PRODUCT_ID_OFFSET + 0x08)   /* eRcOmM */

#define KERNEL_CODE_OFFSET      (REDBOOT_SIZE + NVRAM_SIZE)         

#ifdef EFG120V2         
#define RAMDISK_OFFSET          (KERNEL_CODE_OFFSET + 0x200000)  /* 2M size kernel */
#else
#define RAMDISK_OFFSET          (KERNEL_CODE_OFFSET + 0x100000)  /* 1M size kernel */
#endif

#define UPGRADE_START_OFFSET  KERNEL_CODE_OFFSET
#define UPGRADE_END_OFFSET    FLASH_SIZE

#if defined (EG808) || defined (V6) //EG808 use grg kernel code
#define KERNEL_RAM_ADDRESS     0x01008000
#define RAMDISK_RAM_ADDRESS    0x00a00000
#else
#define KERNEL_RAM_ADDRESS     0x01d00000
#define RAMDISK_RAM_ADDRESS    0x01000000
#endif

typedef struct {
  unsigned char      DestAddress[6];
  unsigned char      SrcAddress[6];
  unsigned short     sap;
#define UPGRADE_HW_ETHER 0x8888
#define ASSIGN_HW_ETHER 0x0015
  unsigned short     wcmd;
#define  GET_VERSION_INFO 0x0000
#define  DOWN_REQUEST     0x0001
#define  DOWN_DATA        0x0002
#define  DOWN_RESET       0x0003
#define  DOWN_VERIFY      0x0004   //!!! 09/30/96 Add a new download comd
#define  DOWN_EALL        0x0005   //!!! 09/30/96 Add a new download comd

  unsigned short     wsequence;
  unsigned short     woffset;
  unsigned short     wsegment;
  unsigned short     wLen;
  unsigned char      bData[600];
} __attribute__((packed)) DLC;

#define DLC_LEN      10

typedef struct  VCI  {
    unsigned char     Prifix[7];
    unsigned short    VerControl;
    unsigned short    DownControl;
    unsigned char     Hid[32];
    unsigned short    Hver;
    unsigned short    ProdID;
    unsigned short    ProdIDmask;
    unsigned short    ProtID;
    unsigned short    ProtIDmask;
    unsigned short    FuncID;
    unsigned short    FuncIDmask;
    unsigned short    Fver;
    unsigned short    Cseg;
    unsigned short    Csize;
    unsigned char     Postfix[7];
}  __attribute__((packed)) VCI_TABLE;

VCI_TABLE vci;
#define  VCI_LEN         56 //sizeof(VCI_TABLE)-14

typedef  struct  DCB_TYPE {
    unsigned short   sequn;
    unsigned short   dest_seg;
    unsigned short   dest_off;
    unsigned short   src_seg;
    unsigned short   src_off;
    unsigned short   count;
    unsigned         erase;
    unsigned         no;
    unsigned short   tme_of_day;
    unsigned long    state;
#define  IDLE            0x00
#define  PROG            0x01
#define  PROGERR         0x02
#define  COM_SEQ_ERR     0x05
#define  SEQ_NUM_ERR     0x06
#define  PROG_ERR        0x07
#define  VERIFY_ERR      0x09

}   DCB_BUF;

DCB_BUF dcb;
unsigned long  ulAddress=0;
int eall_flag;
static unsigned char block_buffer[BLOCK_SIZE];
static int led_flash_flag=0;
static unsigned long time_now=0;
#ifdef REMEMBER_MAC
static unsigned char FlashNodeAddress[6];
#endif   

unsigned short xchg ( unsigned short dwData)
{
   unsigned short temp;
   temp = ((dwData & 0xff) << 8);
   temp |= ((dwData & 0xff00) >> 8);
   return temp;
                                                                                
}

#ifdef EFG120V2 
#define CS4_BASE_ADDRESS  0x54000000
#define CS4_LED_READY     0
#define CS4_LED_STATUS    1
#endif

void Led_Power(int value)
{
#ifdef EFG120V2 
  if(value)
    *(unsigned char *)(CS4_BASE_ADDRESS) &= ~(1<<CS4_LED_READY);
  else
    *(unsigned char *)(CS4_BASE_ADDRESS) |= (1<<CS4_LED_READY);
#else
  if(value)
    HAL_GPIO_OUTPUT_SET(GPIO_LED_READY);
  else
    HAL_GPIO_OUTPUT_CLEAR(GPIO_LED_READY);
#endif
}
void Led_Post(int value)
{
#ifdef EFG120V2 
  if(value)
    *(unsigned char *)(CS4_BASE_ADDRESS) &= ~(1<<CS4_LED_STATUS);
  else
    *(unsigned char *)(CS4_BASE_ADDRESS) |= (1<<CS4_LED_STATUS);
#else
  if(value)
    HAL_GPIO_OUTPUT_SET(GPIO_LED_STATUS);
  else
    HAL_GPIO_OUTPUT_CLEAR(GPIO_LED_STATUS);
#endif
}

#if 0
void Upgrade_Flash(void)
{
  if(MS_TICKS()-time_now>300)
    {
      if(led_flash_flag)
	{
	  Led_Power(0);
#if (HAL_PLATFORM_MACHINE_TYPE == 245) //ixdp425 ; 290 grg
	  Led_Post(1);
#else
	  Led_Post(0);
#endif
	  led_flash_flag=0;
	}
      else
	{
	  Led_Power(1);
	  Led_Post(0);
	  led_flash_flag=1;
	}     
      time_now=MS_TICKS();
    }
}

void Assign_Flash(void)
{
  if(MS_TICKS()-time_now>300)
    {
      if(led_flash_flag)
	{
#if (HAL_PLATFORM_MACHINE_TYPE == 245) //ixdp425 ; 290 grg
	  Led_Power(1);
#else
	  Led_Power(0);
#endif
	  Led_Post(1);
	  led_flash_flag=0;
	}
      else
	{
	  Led_Power(0);
	  Led_Post(0);
	  led_flash_flag=1;
	}     
      time_now=MS_TICKS();
    }
}
#endif

int PushButton(void)
{
  if(*IXP425_GPINR & (1 << GPIO_PUSHBUTTON))
    return 0;
  else
    return 1;
}


externC int flash_erase(volatile flash_t *block, unsigned int size);
externC int flash_program_buf(volatile flash_t *addr, flash_t *data, int len);

int FlashDriver(unsigned long flash_addr,char *mem_addr,unsigned long length,unsigned long dlFlag)
{
   int stat=0;
   unsigned long address;
   void *err_addr;

   switch (dlFlag) {
   case 0:
        if (eall_flag == 0) {
         if (( flash_addr >=  UPGRADE_START_OFFSET)&&(flash_addr < UPGRADE_END_OFFSET)) {
	   if ((stat = flash_program_buf((void *)(flash_addr|FLASH_ADDR_BASE), 
				     (void *)mem_addr, length)) != 0)
	     {
	       diag_printf("Can't program region at flash_addr %08x,stat=%d\n", flash_addr,stat);
	     }
         } else {
           stat = 0;
         } /* endif */
        } else {
	   if ((stat = flash_program_buf((void *)(flash_addr|FLASH_ADDR_BASE), 
				     (void *)mem_addr, length)) != 0)
	     {
	       diag_printf("Can't program region at flash_addr %08x,stat=%d\n", flash_addr,stat);
	     }
        }
        break;
   case 1:
        break;
   case 2:
        break;
   case 3: //erase all
     //        if ((stat = flash_unlock((void *)FLASH_ADDR_BASE, UPGRADE_END_OFFSET, (void **)&err_addr)) != 0) 
     //	  {
     //	    diag_printf("Error unlocking at %p: %s\n", err_addr, flash_errmsg(stat));
     //	  }
     for(address =0;address < UPGRADE_END_OFFSET;address += BLOCK_SIZE)
	if ((stat = flash_erase((void *)(FLASH_ADDR_BASE + address), BLOCK_SIZE)) != 0) 
	  {
	    diag_printf("Error erasing at %d\n", stat);
	  }

	if ((stat = flash_program_buf((void *)(flash_addr|FLASH_ADDR_BASE), 
				  (void *)mem_addr, length)) != 0)
	  {
	    diag_printf("Can't program region at flash_addr %08x,stat=%d\n", flash_addr,stat);
	  }
        eall_flag = 1;
        break;
   case 4://normal erase
     //        if ((stat = flash_unlock((void *)(FLASH_ADDR_BASE|UPGRADE_START_OFFSET), 
     //			      FLASH_SIZE-UPGRADE_START_OFFSET, (void **)&err_addr)) != 0) 
     //	  {
     //	    diag_printf("Error unlocking at %p: %s\n", err_addr, flash_errmsg(stat));
     //	  }
     for(address = UPGRADE_START_OFFSET ;address < UPGRADE_END_OFFSET;address += BLOCK_SIZE)     
	if ((stat = flash_erase((void *)(FLASH_ADDR_BASE + address), 
				BLOCK_SIZE)) != 0)
	  {
	    diag_printf("Error erasing at %d\n", stat);
	  }
	if (( flash_addr >=  UPGRADE_START_OFFSET)&&(flash_addr < UPGRADE_END_OFFSET))
	  {
	    if ((stat = flash_program_buf((void *)(flash_addr|FLASH_ADDR_BASE), 
				      (void *)mem_addr, length)) != 0)
	      {
		diag_printf("Can't program region at flash_addr %08x,stat=%d\n", flash_addr,stat);
	      }
	  } else {
	    stat = 0;
	  } /* endif */
        eall_flag = 0;
        break;
   case 5:
     if ((stat = flash_program_buf((void *)(flash_addr|FLASH_ADDR_BASE), 
			       (void *)mem_addr, length)) != 0)
       {
	 diag_printf("Can't program region at flash_addr %08x,stat=%d\n", flash_addr,stat);
       }
     break;
   } /* endswitch */
   return stat;
}

/*
 * Handle incoming upgrade packets.
 */

unsigned char NodeAddress[6];
int dl_Initialize(unsigned char *node)
{
  //  return Lan_Initialize1();
  return Lan_Initialize3(node);
}

void dl_GetAddr(unsigned char *node)
{
  //Lan_GetAddr1(node);
  Lan_GetAddr3(node);
}
IOB *dl_Receive(void)
{
  //  return Lan_Receive1();
  return Lan_Receive3();
}
int dl_Transmit(IOB *psIOB)
{
  //  Lan_Transmit1(psIOB);
  Lan_Transmit3(psIOB);
}

void Download(void){
	int i;
	IOB *psIOB,*psIOB1;
	DLC * psDLCHead;
        DLC * psDLCHead1;
	unsigned short * dwTemp;
        unsigned int  ulAddress;
        unsigned int  ulCount,blockOffset;
        unsigned char * ptr;
        unsigned char * psPID;
        unsigned long timeNow=0,timeOld=0;
	VCI_TABLE vci;
        //unsigned char NodeAddress[8]={0x00,0xc0,0x02,0x03,0x06,0x18};
        
        dl_GetAddr(NodeAddress);
    
        
        
        
	//        Led_Power(0);        
	//        Led_Post(1);
	//        Fsh_Power(0x83);        
	//        Fsh_Post(0x83); 
        diag_printf("In download function\r\n");

	/*for TFTP download init.*/
	//dl_c_init(NodeAddress);  

	timeOld = sys_times();
        while(1){
#if 1
	  //strange! don't know who mask RTC interrupt
	  cyg_drv_interrupt_unmask(CYGNUM_HAL_INTERRUPT_RTC);

	timeNow = sys_times();
	if(timeNow-timeOld == 4)
	  {
	    *(unsigned char *)(CS4_BASE_ADDRESS) = 0x1;
	    //Led_Post(0);
	    //Led_Power(1);
	  }
	else
	  if(timeNow-timeOld == 8)
	  {
	    *(unsigned char *)(CS4_BASE_ADDRESS) = 0x2;
	    //Led_Post(1);
	    //Led_Power(0);
	    timeOld = timeNow;
	  }
#endif
	  //if((psIOB=Lan_Receive2()) != 0)
	  //     freeIOB(psIOB);
            if ((psIOB=dl_Receive()) != 0) {
                   psDLCHead = (DLC *)pbIOBData(psIOB);
                   if (psDLCHead->sap == 0 ||
                       psDLCHead->sap == UPGRADE_HW_ETHER) {

		      if ((psIOB1 = LAN_DriverGetIOB(1024)) == NULL) 
			{
			  diag_printf("can't get iob in download\n");
			    }

                      psDLCHead1 = (DLC *)pbIOBData(psIOB1);
                      memcpy ((unsigned char *)psDLCHead1->DestAddress,(unsigned char *)psDLCHead->SrcAddress,6);
                      memcpy ((unsigned char *)psDLCHead1->SrcAddress,NodeAddress,6);
                      psDLCHead1->wcmd = psDLCHead->wcmd;
                      psDLCHead1->sap = psDLCHead->sap;
                      psDLCHead1->wsequence = psDLCHead->wsequence;
                      psDLCHead1->woffset = psDLCHead->woffset;
                      psDLCHead1->wsegment = psDLCHead->wsegment;
                      switch (xchg(psDLCHead->wcmd)) {
                      case GET_VERSION_INFO:               // get version
			diag_printf("download: GET_VERSION_INFO\n");
#if 0
 {
	    unsigned char *ptr=psIOB->dbData + psIOB->dwOffset;

	    diag_printf("psIOB->dwLength=%d,psIOB->dwOffset=%d\n",psIOB->dwLength,psIOB->dwOffset);
	    for(i=0;i<14;i++)
	      diag_printf("%02x.",*ptr++);
 }
#endif
			   psPID = (unsigned char *)(FLASH_ADDR_BASE + PID_OFFSET);
			   memcpy ((unsigned char *)&vci,psPID,sizeof(VCI_TABLE));
			   dwTemp = (unsigned short *)(FLASH_ADDR_BASE + PRODUCT_ID_OFFSET);
			   vci.ProdID = *dwTemp;
			   dwTemp = (unsigned short *)(FLASH_ADDR_BASE + PROTOCOL_ID_OFFSET);
			   vci.ProtID = *dwTemp;
			   dwTemp = (unsigned short *)(FLASH_ADDR_BASE + FW_VERSION_OFFSET);
			   vci.Fver = *dwTemp;

                           psDLCHead1->wLen=xchg(VCI_LEN);
                           memcpy((unsigned char *)psDLCHead1->bData,(unsigned char *)&vci.VerControl,VCI_LEN);
			   psIOB1->dwLength = VCI_LEN+24;
                           dl_Transmit(psIOB1);
                           break;
                      case DOWN_REQUEST:               // download
			diag_printf("download: DOWN_REQUEST normal\n");
                           dcb.state=PROG;
			   //                           ResetChip(0);
           //                 ResetChip(0x100000);
                           dcb.sequn=((xchg(psDLCHead->wsequence)+1) & 0xffff);
                           dcb.erase=4;
                           dcb.no = 0;
                           psDLCHead1->wLen=xchg(2);
			   psIOB1->dwLength = 64;
                           dl_Transmit(psIOB1);
                           break;
                      case DOWN_EALL:               // download            
			diag_printf("download: DOWN_REQUEST erase all\n");
                           dcb.state=PROG;
			   //                           ResetChip(0);
           //                 ResetChip(0x100000);
                           dcb.sequn=((xchg(psDLCHead->wsequence)+1) & 0xffff);
                           dcb.erase=3;
                           dcb.no = 0;
                           psDLCHead1->wLen=xchg(2);
			   psIOB1->dwLength = 64;
                           dl_Transmit(psIOB1);
                           break;
                      case DOWN_DATA :
                           switch (dcb.state) {
                           case PROG:
                                if (dcb.sequn==xchg(psDLCHead->wsequence)) {
                                   ulAddress = (xchg(psDLCHead->wsegment) << 4)+xchg(psDLCHead->woffset);
           
                                   if (ulAddress == 0) {
                                      dcb.no ++;
                                   }
                                   ulAddress += (0x100000*(dcb.no-1));
           
                                   ulCount = (unsigned int)(xchg(psDLCHead->wLen));
#ifdef    REMEMBER_MAC      
                                   if (( (ulAddress) <= NODE_ADDRESS) && ((ulAddress + ulCount) > NODE_ADDRESS)) {
                                      unsigned int tempOffset;
				      unsigned char *ptr = (unsigned char *)(FLASH_ADDR_BASE+NODE_ADDRESS);
           //                            tempOffset = DOMAIN_ADDRESS - ulAddress;
           //                            if (psDLCHead->bData[tempOffset] == 0)
           //                                psDLCHead->bData[tempOffset] = domain;
                                      
                                      if ((memcmp(ptr, "\x00\x00\x00\x00\x00\x00", 6) == 0) ||
                                          (memcmp(ptr, "\xFF\xFF\xFF\xFF\xFF\xFF", 6) == 0)) {
           
                                      } else {
                                          tempOffset = NODE_ADDRESS - ulAddress;
                                          if (memcmp(&psDLCHead->bData[tempOffset], "\x00\x00\x00\x00\x00\x00", 6) == 0)
                                              memcpy ((unsigned char *)&psDLCHead->bData[tempOffset],ptr,6);
                                      }    
                                   }                          
#endif        
				   /*New download */
				     
                                   if ( FlashDriver(ulAddress,(unsigned char *)psDLCHead->bData,ulCount,dcb.erase) == 0) {
                                      dcb.erase = 0;
                                      dcb.sequn += 1;
                                      *((unsigned short *) &psDLCHead1->bData[0])=0;
                                   } else {
                                      dcb.state=PROGERR;
                                      *((unsigned short *) &psDLCHead1->bData[0])=xchg(PROG_ERR);
                                   }
				    
                                } else {
                                  if (dcb.sequn >= xchg(psDLCHead->wsequence)) {
                                     *((unsigned short *) &psDLCHead1->bData[0])=0;
                                  } else {
                                     *((unsigned short *) &psDLCHead1->bData[0])=xchg(SEQ_NUM_ERR);
                                  } /* endif */
                                } /* endif */
                                psDLCHead1->wLen=xchg(2);
				psIOB1->dwLength = 64;
				dl_Transmit(psIOB1);
                                break;
                           case IDLE:
                                psDLCHead1->wLen=xchg(2);
                                *((unsigned short *) &psDLCHead1->bData[0])=xchg(COM_SEQ_ERR);
				psIOB1->dwLength = 64;
				dl_Transmit(psIOB1);
                                break;
                           }
                           break;
                      case DOWN_RESET:
			diag_printf("download: DOWN_RESET\n");
                           psDLCHead1->wLen=xchg(0);
                           *((unsigned short *) &psDLCHead1->bData[0])=0;
			   psIOB1->dwLength = 64;
			   dl_Transmit(psIOB1);
           	 /* write "eRcOmM" sign at the code end, before reset
           	  *   I don't know if recevicing the reset packet is mean
           	  *  that download completely. so here need to be checked
           	  */
           	 //		 writeFlash(sign,getNVRAMFlashAddr()-sizeof(sign),sizeof(sign));
           
			   //                           ResetChip(0);
                           //DbgUartPutStr("Down reset, after reset chip(0)\n");                           
                           for (i=0;i<300000;i++ ) {
                              ;
                           } /* endfor */
			   reset();
                           break;
                      case DOWN_VERIFY:
                           switch (dcb.state) {
                           case PROG:
                                if (dcb.sequn==xchg(psDLCHead->wsequence)) {
                                   if (dcb.erase == 0) {
                                      for (i=0;i<15000;i++ ) {
                                          ;
                                      } /* endfor */
				      //      ResetChip(0);           
                                      dcb.erase = 4;
                                      dcb.no = 0;
                                   } /* endif */
                                   ulAddress = (xchg(psDLCHead->wsegment) << 4)+xchg(psDLCHead->woffset);
				   if (ulAddress == 0) {
                                      dcb.no ++;
                                   } 
                                   ulAddress += 0x100000*(dcb.no-1);
           		 
                                   ptr = ( unsigned char *)(ulAddress|FLASH_ADDR_BASE);
                                   ulCount = (unsigned int)(xchg(psDLCHead->wLen));
                                   if (ulAddress >= UPGRADE_START_OFFSET) {
                                      if (memcmp ( ptr,(unsigned char *)&psDLCHead->bData[0],ulCount) == 0) {
                                         ++dcb.sequn;
                                         *((unsigned short *) &psDLCHead1->bData[0])=0;
                                      } else {
                                      	diag_printf("data verify error at %08x\n",ulAddress);
                                         dcb.state=PROGERR;
                                         *((unsigned short *) &psDLCHead1->bData[0])=xchg(VERIFY_ERR);
                                      }
                                   } else {
                                      ++dcb.sequn;
                                      *((unsigned short *) &psDLCHead1->bData[0])=0;
                                   } /* endif */
                                } else {
                                  if ((dcb.sequn-1)==xchg(psDLCHead->wsequence)) {
                                     *((unsigned short *) &psDLCHead1->bData[0])=0;
                                  } else {
                                     *((unsigned short *) &psDLCHead1->bData[0])=xchg(SEQ_NUM_ERR);
                                  } /* endif */
                                } /* endif */
				//for(i=0;i<100;i++);
				//diag_printf("psDLCHead1->wsegment=%04x,psDLCHead->wsegment=%04x\n",xchg(psDLCHead1->wsegment),xchg(psDLCHead->wsegment));
                                psDLCHead1->wLen=xchg(2);
				psIOB1->dwLength = 64;
				dl_Transmit(psIOB1);
                                break;
                           } /* endswitch */
                           break;
                      }
                   } /* psDLCHead->sap == UPGRADE_HW_ETHER  */
		   else
		     {

		       //dl_c_process(psIOB);
		       //if(1== DVARI.final)
		       // {
		       // }
		     }
		   freeIOB(psIOB);
            }  
	}// end while
}



//cdl_option CYGDAT_REDBOOT_DEFAULT_BOOT_SCRIPT {
//    user_value 1 "\"load redboot.srec; channel 2; go\n"\"
//};


void AssignHWAddress(unsigned char *psBuffer)
{
  //  unsigned char * psBuffer=(unsigned char *)pkt->buf;
  int stat=0;
  unsigned long offset;
  void *err_addr;

  diag_printf("assign mac address:%02x:%02x:%02x:%02x:%02x:%02x\n",
	      psBuffer[0],psBuffer[1],psBuffer[2],
	      psBuffer[3],psBuffer[4],psBuffer[5]);
  
  for(offset=0;offset<REDBOOT_SIZE;offset+=BLOCK_SIZE)
    {
      memcpy(block_buffer,(unsigned char *)(FLASH_ADDR_BASE+offset),BLOCK_SIZE);
      if(offset+BLOCK_SIZE>=REDBOOT_SIZE)
	memcpy(block_buffer+BLOCK_SIZE-0x50,psBuffer,6);

      //      if ((stat = flash_unlock((void *)(FLASH_ADDR_BASE+offset), BLOCK_SIZE, (void **)&err_addr)) != 0) 
      //	{
      //	  diag_printf("Error unlocking at %p: %s\n", err_addr, flash_errmsg(stat));
      //	}
      if ((stat = flash_erase((void *)(FLASH_ADDR_BASE+offset), BLOCK_SIZE)) != 0) 
	{
	  diag_printf("Error erasing stat=%d\n", stat);
	}
      if ((stat = flash_program_buf((void *)(FLASH_ADDR_BASE+offset), 
				(void *)block_buffer, BLOCK_SIZE)) != 0)
	{
	  diag_printf("Can't program region at flash_addr %08x,stat=%d\n", offset,stat);
	}
    }  

}


void Assign(void)
{
	IOB *psIOB;
	unsigned char   dbNodeaddress[10];
        unsigned long timeNow=0,timeOld=0;

	diag_printf("In Assign function\r\n");
	timeOld = sys_times();

        while (1) {

#if 1
	  //strange! don't know who mask RTC interrupt
	  cyg_drv_interrupt_unmask(CYGNUM_HAL_INTERRUPT_RTC);

	timeNow = sys_times();
	if(timeNow-timeOld == 2)
	  {
	    *(unsigned char *)(CS4_BASE_ADDRESS) = 0x3;
	    //Led_Power(0);
	    //Led_Post(0);
	  }
	else
	  if(timeNow-timeOld == 4)
	  {
	    *(unsigned char *)(CS4_BASE_ADDRESS) = 0x0;
	    //Led_Power(1);
	    //Led_Post(1);
	    timeOld = timeNow;
	  }
#endif
	  //if((psIOB=Lan_Receive2()) != 0)
	  //   freeIOB(psIOB);


           if ((psIOB=dl_Receive()) != 0) {
              unsigned char * psBuffer;
              psBuffer = pbIOBData(psIOB);	   
              if ((psBuffer[12] == 0) && (psBuffer[13] == ASSIGN_HW_ETHER)) {
                 memcpy(dbNodeaddress,&psBuffer[14],6);   
                 AssignHWAddress(dbNodeaddress);
		 reset();
              }
	      freeIOB (psIOB);
           } /* endif */
        } /* endwhile */ 
}

void do_move(void)
{
  unsigned long fileSize = *(unsigned long *)(FLASH_ADDR_BASE+KERNEL_CODE_OFFSET);
  unsigned char argv[4][15] = {"exec","-r","0x01000000","0x01d00000"};

  diag_printf("copy kernel code from flash to RAM\n");
  memcpy((unsigned char *)KERNEL_RAM_ADDRESS,(unsigned char *)
	 (FLASH_ADDR_BASE+KERNEL_CODE_OFFSET+0x10),fileSize);
  
  load_address = KERNEL_RAM_ADDRESS;
  load_address_end = KERNEL_RAM_ADDRESS + fileSize;
  entry_address = KERNEL_RAM_ADDRESS;         

  fileSize = *(unsigned long *)(FLASH_ADDR_BASE+RAMDISK_OFFSET);
  diag_printf("copy ramdisk file from flash to RAM\n");
  memcpy((unsigned char *)RAMDISK_RAM_ADDRESS,(unsigned char *)
	 (FLASH_ADDR_BASE+RAMDISK_OFFSET+0x10),fileSize);	    
  
  //  diag_sprintf(argv[2],"0x%08x",RAMDISK_RAM_ADDRESS);
  //  diag_sprintf(argv[3],"0x%08x",KERNEL_RAM_ADDRESS);
  //  do_exec(4,argv);
  do_exec(RAMDISK_RAM_ADDRESS,KERNEL_RAM_ADDRESS);
}
void do_boot(void)
{
  unsigned char *dbSign,*ptr;
  unsigned long fileSize;
  unsigned char myaddr[8]={0x00,0xc0,0x02,0x01,0x01,0x01};

  dbSign=(unsigned char *)(FLASH_ADDR_BASE+SIGN_OFFSET);       
  if ( memcmp (dbSign,"eRcOmM",6) == 0)
    {
      diag_printf("have eRcOmM \r\n");
      ptr = (unsigned char *)(FLASH_ADDR_BASE+NODE_ADDRESS);
      if ((memcmp(ptr,"\x00\x00\x00\x00\x00\x00",6) == 0) ||
	  (memcmp(ptr,"\xff\xff\xff\xff\xff\xff",6) == 0) )
	{
	  diag_printf("no node address \r\n");
	  dl_Initialize(myaddr);
	  Assign();
	} else {
	  fileSize = *(unsigned long *)(FLASH_ADDR_BASE+KERNEL_CODE_OFFSET);
	  if (fileSize == 0 || fileSize == 0xFFFFFFFF) {
	    dl_Initialize(myaddr);
	    Download();
	  } else {
#if 1
	    if(PushButton()==1)
	      {
		Led_Power(0);
		Led_Post(1);

		diag_printf("if release,go to upgrade mode\n");
		CYGACC_CALL_IF_DELAY_US(3000000); //wait 1s
		if(PushButton()==0)
		  {
		    dl_Initialize(ptr);
		    Download();		  
		  }

		Led_Power(1);
		Led_Post(0);

		diag_printf("if release,go to assign mode\n");
		CYGACC_CALL_IF_DELAY_US(3000000); //wait 3s
		if(PushButton()==0)
		  {
		    dl_Initialize(myaddr);
		    Assign();		  
		  }
	      }
#endif		 
	    do_move();
	    diag_printf("run kernel \r\n");
	    return ;
	    //	StartDownPgm((PROGRAM) (MAIN_CODE_OFFSET+0x10));
	  }  
	}
    } else {
      diag_printf("not have eRcOmM \r\n");
      dl_Initialize(ptr);
      Download();
  } /* endif */
  
}
