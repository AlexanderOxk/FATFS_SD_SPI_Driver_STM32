
#ifndef SD_Driver
#define SD_Driver

#include "diskio.h"
#include "stm32g0xx_hal.h"

/* microSD Commands */
#define CMD0	(0x40 + 0)		//GO_IDLE_STATE
#define CMD1	(0x40 + 1)		//SEND_OP_CND
#define CMD6	(0x40 + 6)		//SWITCH_FUNC
#define CMD8	(0x40 + 8)		//SEND_IF_COND
#define CMD9	(0x40 + 9)		//SEND_CSD
#define CMD10	(0x40 + 10)		//SEND_CID
#define CMD12	(0x40 + 12)		//STOP_TRANSMISSION
#define CMD13	(0x40 + 13)		//SEND_STATUS
#define CMD16	(0x40 + 16)		//SET_BLOCKLEN
#define CMD17	(0x40 + 17)		//READ_SINGLE_BLOCK
#define CMD18	(0x40 + 18)		//READ_MULTIPLE_BLOCK
#define CMD24	(0x40 + 24)		//WRITE_BLOCK
#define CMD25	(0x40 + 25)		//WRITE_MULTIPLE_BLOCK
#define CMD27	(0x40 + 27)		//PROGRAM_CSD
#define CMD32	(0x40 + 32)		//ERASE_WR_BLK_START_ADDR
#define CMD33	(0x40 + 33)		//ERASE_WR_BLK_END_ADDR
#define CMD38	(0x40 + 38)		//ERASE
#define CMD42	(0x40 + 42)		//LOCK_UNLOCK
#define CMD55	(0x40 + 55)		//APP_CMD
#define CMD58	(0x40 + 58)		//READ_OCR
#define CMD59	(0x40 + 59)		//CRC_ON_OFF
#define ACMD41	(0x40 + 41)		//APP_SEND_OP_COND

DSTATUS status_sd();
DSTATUS initialize_sd();
DRESULT read_sd(BYTE pdrv, BYTE *buff, uint32_t sector, UINT count);
DRESULT write_sd(BYTE pdrv, const BYTE *buff, uint32_t sector, UINT count);
DRESULT ioctl_sd(BYTE pdrv, BYTE cmd, void *buff);

#endif
