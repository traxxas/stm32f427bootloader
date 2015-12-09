/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2014        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include <sdio_sd.h>

typedef struct {
	DSTATUS status;
} DiskIoContext;

static DiskIoContext diskIoContext = {
	.status = STA_NOINIT,
};

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/
/* Physical drive number to identify the drive */
DSTATUS disk_status (BYTE pdrv) {
	if (pdrv) {
		return STA_NOINIT;
	} else {
		return diskIoContext.status;
	}
}

/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/*-----------------------------------------------------------------------*/
/* Physical drive number to identify the drive */
DSTATUS disk_initialize (BYTE pdrv) {
	if (pdrv) {
		return STA_NOINIT;
	} else {
		SDIO_NVIC_Configuration();

  	// Initialize drive
		if (SD_Init() == SD_OK) {
			diskIoContext.status &= ~STA_NOINIT;
		} else {
			if (SD_Detect() != SD_PRESENT) {
				diskIoContext.status |= STA_NODISK;
			}
		}

		return diskIoContext.status;
  }
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive number to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
	if (pdrv ||!count) return RES_PARERR;
	if (diskIoContext.status & STA_NOINIT) return RES_NOTRDY;

	if (SD_ReadMultiBlocks (buff, sector * 512, 512, count) != 0) {
		return RES_ERROR;
	}

	SD_WaitReadOperation();
	while (SD_GetStatus() != SD_TRANSFER_OK);

	return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive number to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address in LBA */
	UINT count			/* Number of sectors to write */
)
{
	if (pdrv ||!count) return RES_PARERR;
	if (diskIoContext.status & (STA_NOINIT | STA_NODISK)) return RES_NOTRDY;
	if (diskIoContext.status & STA_PROTECT) return RES_WRPRT;

#if defined (MSD_USE_MICROSD_MEM)
	if (SD_WriteMultiBlocks (buff, sector * 512, 512, count) != 0) {
		return RES_ERROR;
	}

	SD_WaitWriteOperation();
	while (SD_GetStatus() != SD_TRANSFER_OK);
#endif

	return RES_OK;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive number (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	SD_CardInfo cardInfo;

	DRESULT res = RES_ERROR;

	if (pdrv) return RES_PARERR;
	if (diskIoContext.status & (STA_NOINIT | STA_NODISK)) return RES_NOTRDY;

	switch (cmd)
	{
		case CTRL_SYNC :	  /* Make sure that no pending write process */
			res = RES_OK;
			break;

		case GET_SECTOR_COUNT :   /* Get number of sectors on the disk (DWORD) */
			if (SD_GetCardInfo(&cardInfo) == SD_OK) {
				*(uint32_t*)buff = cardInfo.CardCapacity / cardInfo.CardBlockSize;
				res = RES_OK;
			}
			break;

		case GET_SECTOR_SIZE :	  /* Get R/W sector size (DWORD) */
		case GET_BLOCK_SIZE : /* Get erase block size in unit of sector (DWORD) */
			if (SD_GetCardInfo(&cardInfo) == SD_OK) {
				*(uint32_t*)buff = cardInfo.CardBlockSize;
				res = RES_OK;
			}
		 	break;

		default:
		  	res = RES_PARERR;
			break;
	}

	return res;
}
#endif


DWORD get_fattime(void) {
  return 0;
}