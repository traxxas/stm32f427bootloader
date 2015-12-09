/*! \file upgrade.c
 *  \brief
 *
 *  Details
 *
 *  Copyright (c) 2015 Traxxas, All Rights Reserved
 */

// -----------------------------------------------------------------------------
// Includes
#include <upgrade.h>
#include <ff.h>
#include <sdio_sd.h>
#include <string.h>
#include <stm32f4xx_flash.h>
#include <stdbool.h>
#include <stm32f4xx_sdio.h>
#include <px4io_protocol.h>
#include <crc.h>
#include <serial.h>

// -----------------------------------------------------------------------------
// Definitions

/* extern */ const uint32_t Upgrade_AppBaseAddress = 0x08004000; /*!< Base address of the Application Image */

#define Upgrade_WriteBufferSize 512 /*!< bytes, matches microSD Sector Size */

/*! \brief Boot States
 *
 *  This enumeration is used to communicate the current bootloader state to the IO microcontroller.  This
 *  is used by the IO microcontroller to set the LED pattern.
 */
typedef enum {
  BootState_Normal,             /*!< Normal Application Boot */
  BootState_UpgradeInProgress,  /*!< Upgrade In Progress */
  BootState_BootErr,            /*!< Boot/Upgrade Error */

  BootState_Max                 /*!< Number of BootState, must be final entry */
} BootState;

/*! \brief Upgrade Context Data
 */
typedef struct {
  FATFS fatFs;              /*!< Fat File System Context */
  FIL fp;                   /*!< Upgrade File Handle */
  union {
    uint8_t byteArray[Upgrade_WriteBufferSize];
    uint32_t wordArray[Upgrade_WriteBufferSize / 4];
  } writeBuffer;
  uint32_t writeAddress;    /*!< Flash Write Address */
  uint32_t bytesRead;       /*!< Number of bytes read into buffer */
} UpgradeContext;

// -----------------------------------------------------------------------------
// Forward Declarations

static bool upgrade_mount(void);
static bool upgrade_openUpgradeFile(void);
static void upgrade_closeUpgradeFile(void);
static bool upgrade_erase(void);
static void upgrade_program(void);
static void upgrade_txBootState(BootState state);

// -----------------------------------------------------------------------------
// Globals

static const char* Upgrade_FileName = "UPGRADE.bin";      /*!< Firmware upgrade file */
static const char* Upgrade_RootDirectory = "";            /*!< Mount point of SD Card */
static UpgradeContext upgradeContext;

static const uint32_t upgrade_appFlashSectors[] = {
  /* FLASH_Sector_0 is reserved for the Boot Loader */
  FLASH_Sector_1,  /*!< Sector 1 - 16 Kbytes */
  FLASH_Sector_2,  /*!< Sector 2 - 16 Kbytes */
  FLASH_Sector_3,  /*!< Sector 3 - 16 Kbytes */
  FLASH_Sector_4,  /*!< Sector 4 - 64 Kbytes */
  FLASH_Sector_5,  /*!< Sector 5 - 128 Kbytes */
  FLASH_Sector_6,  /*!< Sector 6 - 128 Kbytes */
  FLASH_Sector_7,  /*!< Sector 7 - 128 Kbytes */
  FLASH_Sector_8,  /*!< Sector 8 - 128 Kbytes */
  FLASH_Sector_9,  /*!< Sector 9 - 128 Kbytes */
  FLASH_Sector_10, /*!< Sector 10 - 128 Kbytes */
  FLASH_Sector_11, /*!< Sector 11 - 128 Kbytes */
};

volatile uint8_t uartTxBuf[PX4IO_PKT_SIZE];

// -----------------------------------------------------------------------------
// Functions

/*! \brief Upgrade via MicroSD
 *
 *  This function will upgrade the application firmware if it is requested. This
 *  function:
 *    1. Mounts the SD card
 *    2. Looks for the existance of the firmware upgrade file
 *    3. Erases flash memory
 *    4. Loads the firmware upgrade file into flash
 *
 */
void upgrade(void) {
  uint8_t err = 0;
  volatile uint32_t uartDelay = 0;

  // Uart init
  serial_init();
  uartDelay = 0x1FFFFF;
  while(uartDelay--);
  upgrade_txBootState(BootState_UpgradeInProgress);

  if (upgrade_mount()) {
    if (upgrade_openUpgradeFile()) {
      err = 1; // default to error
      FLASH_Unlock();

      if (upgrade_erase()) {
        upgrade_txBootState(BootState_UpgradeInProgress);
        upgrade_program();
        err = 0;
      }

      FLASH_Lock();

      upgrade_closeUpgradeFile();
    }
  }

  if (err) {
    upgrade_txBootState(BootState_BootErr);
  } else {
    upgrade_txBootState(BootState_Normal);
  }

  uartDelay = 0xFFFF;
  while(uartDelay--);
}

/*! \brief Mount the MicroSD Card
 *
 *  This function checks for the presence of the microSD card.  If it
 *  is detected, it will attempt to mount the card.
 *
 *  \return bool, true if the microSD card was successfully mounted
 */
bool upgrade_mount(void) {

  bool mountSuccess = false;

  if (SD_Detect() == SD_PRESENT) {
    if (f_mount(&upgradeContext.fatFs, Upgrade_RootDirectory, 1) == FR_OK) {
      mountSuccess = true;
    }
  }

  return mountSuccess;
}

/*! \brief Open Upgrade File
 *
 *  Open the firmware upgrade file.  This file is at a well known name and
 *  location.  If the file exists, an upgrade has been requested and the
 *  file will be opened for reading.
 *
 *  If the file does not exist, this function will return 'false' indicating
 *  that an upgrade file was not found or could not be opened.
 */
bool upgrade_openUpgradeFile(void) {
  bool openSuccess = false;

  FRESULT fresult = f_open(&upgradeContext.fp, Upgrade_FileName, FA_READ);
  if (fresult == FR_OK) {
    openSuccess = true;
  }

  return openSuccess;
}

/*! \brief Close Upgrade File
 */
void upgrade_closeUpgradeFile(void) {
  f_close(&upgradeContext.fp);
}

/*! \brief Erase Flash
 *
 *  This function erases the application flash sections.
 */
bool upgrade_erase(void) {
  uint32_t eraseSuccess = true;

  // Erase the Application Flash sectors
  for (uint32_t i = 0; i < sizeof(upgrade_appFlashSectors) / sizeof(uint32_t); i++) {
    if (FLASH_EraseSector(upgrade_appFlashSectors[i], VoltageRange_3) != FLASH_COMPLETE) {
      eraseSuccess = false;
      break;
    }
  }

  return eraseSuccess;
}

/*! \brief Program Flash
 *
 *  Write the contents of the flash upgrade file into the Application Flash Sectors.
 */
void upgrade_program(void) {
  upgradeContext.writeAddress = Upgrade_AppBaseAddress;

  do {
    if (f_read(&upgradeContext.fp, upgradeContext.writeBuffer.byteArray, Upgrade_WriteBufferSize, &upgradeContext.bytesRead) == FR_OK) {
      if (upgradeContext.bytesRead % 4 == 0) {
        for (uint32_t i = 0; i < upgradeContext.bytesRead / 4; i++) {
          FLASH_ProgramWord(upgradeContext.writeAddress, upgradeContext.writeBuffer.wordArray[i]);

          // Advance to the next application address
          upgradeContext.writeAddress += 4;
        }
      } else {
        for (uint32_t i = 0; i < upgradeContext.bytesRead; i++) {
          FLASH_ProgramByte(upgradeContext.writeAddress, upgradeContext.writeBuffer.byteArray[i]);

          // Advance to the next application address
          upgradeContext.writeAddress += 1;
        }
     }
    }
  } while (upgradeContext.bytesRead == Upgrade_WriteBufferSize);
}


/*! \brief Compute the CRC for a received packet
 */
uint8_t crc_packet(__IO IOPacket* pkt) {
  uint8_t* end = (uint8_t*)(&pkt->regs[PKT_COUNT(*pkt)]);
  uint8_t* p = (uint8_t*)pkt;
  uint32_t length = end - p;

  return crc_calc8(p, length);
}

/*! \brief Boot State Packet Transmit
 *
 *  Send boot state to extern UART device.
 */
void upgrade_txBootState(BootState state) {
  // Prepare boot state packet
  IOPacket* txPacket = (IOPacket*) uartTxBuf;

  txPacket->page = 59 /* PX4IO_PAGE_CONTROL_OUTPUT */;
  txPacket->offset = 13 /* PX4IO_P_CONTROL_OUTPUT_FMU_BOOT_STATE */;
  txPacket->count_code = PKT_CODE_WRITE | 1;
  txPacket->regs[0] = state;

  // Compute CRC
  txPacket->crc = 0;
  txPacket->crc = crc_packet(txPacket);

  // Configure the serial transmission and send
  serial_txSend(uartTxBuf, PKT_SIZE(*txPacket));
}