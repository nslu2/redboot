#include <iob.h>
#include <cyg/infra/cyg_type.h>

#define MAX_BUF_NO         400

static IOB RxBuf[MAX_BUF_NO] __attribute__((aligned(16)));
static int flag[MAX_BUF_NO];

void  initIOB(void)
{
	memset (flag, 0, sizeof (flag));
}


CB * getCBFromQ(CBQ *psCBQ, unsigned int zeroSize)
{
	CB * psCB = NULL;

   if (psCBQ->dwQCount != 0) 
   {
		/*  if there a CB available in this queue */
		psCB = psCBQ->psQHead;
		/*   get a CB to use, use the CB queue head        */
		psCBQ->psQHead = psCBQ->psQHead->psNextCB;
		/*   make new CB queue head              */
		psCB->psNextCB = NULL;
		
		if (--psCBQ->dwQCount==0) 
			psCBQ->psQTail=NULL;
			
		if (zeroSize) 
			memset((void *)psCB,0x00,zeroSize);
	}
	
   	return (psCB);
}

void putCBToQ(CBQ *psCBQ, CB *psCB, unsigned char dbOrder)

/*
 * dbOrder:
 *      HEAD : insert CB in the begining
 *      ADDRORDER: insert CB in queue so that queue is in address order
 *      TAIL : insert CB at the end
 */
{

  //  diag_printf("in putCBToQ:psCB=%08x\n",psCB);
	if (psCB==NULL) 
		return;
	
	if (psCBQ->dwQCount==0) 
	{
		/* if the psCB will be the only CB on queue make it the queue head */
		psCBQ->psQHead = psCB;
		psCBQ->psQTail = psCB;
		psCB->psNextCB = NULL;
	} 
	else 
	{
		/* 1 or more CB is in queue                                */
		if (dbOrder==TAIL) 
		{
			psCB->psNextCB = NULL;
			psCBQ->psQTail->psNextCB = psCB;
			psCBQ->psQTail = psCB;
		} 
		else if (dbOrder==HEAD || (dbOrder == ADDRORDER && psCB < psCBQ->psQHead)) 
		{
			/* if the CB address is smaller than queue head, make CB */
			/* the new queue head, and make the chain */
			psCB->psNextCB = psCBQ->psQHead;
			psCBQ->psQHead = psCB;
		} 
		else 
		{
			/* if CB address is bigger than free queue head         */
			/* find a CB in chain such that it's address is smaller */
			/* than THE CB. Put THE CB in chain so that the queue is*/
			/* is a sort-by-address sorted-queue                    */
			CB *aCB;
			aCB = psCBQ->psQHead;
		
			while ((psCB > aCB->psNextCB)
			&& aCB->psNextCB != NULL ) 
			{
				aCB = aCB->psNextCB;
			}
			
			psCB->psNextCB = aCB->psNextCB;
			aCB->psNextCB = psCB;
			
			if (aCB==psCBQ->psQTail) 
				psCBQ->psQTail = psCB;
		}
	}
	psCBQ->dwQCount++;
}

IOB * LAN_DriverGetIOB ( unsigned int dwSize )
{
	int i;
	IOB *psIOB = NULL;
	
	if (dwSize <= IOB_DATASZ)
	{
		for (i=0;i<MAX_BUF_NO ;i ++ ) 
		{      
			if (flag[i] == 0) 
			{
			flag[i] = 1;
			psIOB =  &RxBuf[i];
			psIOB->dwLength = 0;
			psIOB->dwOffset = 0;
			break;
			}
		}
	}
	
	//   printf ("LAN_DriverGetIOB: psIOB = %p\n", psIOB);
	return psIOB;
}

IOB * DriverGetIOB ( unsigned int dwSize )
{
	return LAN_DriverGetIOB (dwSize);
}

int freeIOB ( IOB * psIOB )
{
   int i;

//   printf ("freeIOB: psIOB = %p\n", psIOB);
   
	for (i=0;i<MAX_BUF_NO ;i++ ) 
	{
		if (psIOB == &RxBuf[i]) 
		{
			flag[i] = 0;
			return 0;
		} 
		
	}
   diag_printf ("IOB %p NOT FOUND for Freeing\n", psIOB);
   return 1;
}

