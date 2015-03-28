#ifndef ___IOB_H___ 
#define ___IOB_H___

#define HEAD 		0
#define ADDRORDER 	1
#define TAIL 		2

#define IOB_DATASZ		1600    /* buffer size */
#define IOB_OFFSET  	16      /* space reserved */
#define IOB_HDR_SZ		8

typedef void * LPVOID;

typedef struct {
   void * psNextIOB;
   unsigned short dwLength;
   unsigned short dwOffset;
   unsigned char dbData[IOB_DATASZ];
}__attribute__((packed)) IOB;

typedef IOB *  PIOB;
typedef IOB * LPIOB;

#define dwIOBDataLength(psData) (((psData))->dwLength)
#define pbIOBData(psData) &(((psData))->dbData[((psData))->dwOffset])
#define dwIOBDataOffset(psData) (((psData))->dwOffset)

#define IOBOFF(x) ((x)->dwOffset)
#define IOBLEN(x) ((x)->dwLength)
#define IOBDATA(x) (&((x)->dbData[(x)->dwOffset]))

typedef struct CBtype {
   struct CBtype *psNextCB;             /* pointer to next CB */
} CB;

typedef struct CBQtype {
   CB *psQHead;              /* pointer to Queue Head */
   CB *psQTail;              /* pointer to Queue Tail */
   unsigned int dwQCount;       /* Queue's element count */
} CBQ;

void putCBToQ(CBQ *psCBQ, CB *psCB, unsigned char dbOrder);
CB * getCBFromQ(CBQ *psCBQ, unsigned int zeroSize);
int freeIOB(IOB * psIOB);
IOB * LAN_DriverGetIOB ( unsigned int dwSize );
IOB * DriverGetIOB ( unsigned int dwSize );
void  initIOB(void);

#endif /* ___IOB_H___ */

