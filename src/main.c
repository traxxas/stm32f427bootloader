/*! \file main.c
 *  \brief
 *
 *  Copyright (c) 2015 Traxxas, All Rights Reserved
 */

// -----------------------------------------------------------------------------
// Includes

#include <stm32_bsp.h>
#include <sdio_sd.h>
#include <upgrade.h>

// -----------------------------------------------------------------------------
// Definitions
typedef void (*pFunction)(void);

// -----------------------------------------------------------------------------
// Forward Declarations
static void app_launch(void);

// -----------------------------------------------------------------------------
// Globals

// -----------------------------------------------------------------------------
// Functions

/*! \brief Bootloader Main
 */
int32_t main(void) {
  // Initialize hardware clocks
  hardware_clockInit();

  // Initialize PWR device : needed for RTC BKP registers
  hardware_pwrInit();

  // Work around: The application crashes when the bootloader starts the App
  // after the FAT FS drivers are initialized.  As a work around, the boot loader
  // will set a flag and reset the micro.  If the flag is set, the bootloader
  // will jump immediately into the app without initializing the FAT FS drivers.
  if (hardware_isRebootToApp()) {
    // Initialize the SD Pins : needed for SD_Detect
    SD_LowLevel_Init();

    if (SD_Detect() == SD_PRESENT) {
      // Run the upgrade driver.  This will check to see if a firmware upgrade has
      // been requested
      upgrade();

      // Reboot into the Application
      hardware_rebootToApp();
     }
  }

  // Clear the reboot flag
  hardware_clearRebootFlag();

  // Start the application
  app_launch();
}

/*! \brief Start the Application
 */
void app_launch(void) {
  uint32_t JumpAddress;
  pFunction Jump_To_Application;

  /* Test if user code is programmed starting from address "APPLICATION_ADDRESS" */
  if (((*(__IO uint32_t*)Upgrade_AppBaseAddress) & 0x2FFE0000 ) == 0x20000000)
  {
    NVIC_SetVectorTable(Upgrade_AppBaseAddress, 0);

    /* Jump to user application */
    JumpAddress = *(__IO uint32_t*) (Upgrade_AppBaseAddress + 4);
    Jump_To_Application = (pFunction) JumpAddress;
    /* Initialize user application's Stack Pointer */
    __set_MSP(*(__IO uint32_t*) Upgrade_AppBaseAddress);

    /* Jump to application */
    Jump_To_Application();
  }

  // Block indefinitely on failure: user is required to power cycle to reattempt upgrade
  while (1);
}