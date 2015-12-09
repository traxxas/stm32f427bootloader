/*! \file stm32_bsp.c
 *  \brief
 *
 *  Details
 *
 *    See header file.
 *
 *  Copyright (c) 2015 Traxxas, All Rights Reserved
 */

// -----------------------------------------------------------------------------
// Includes

#include <stm32_bsp.h>
#include <system_stm32f4xx.h>
#include <stm32f4xx_sdio.h>

// -----------------------------------------------------------------------------
// Definitions

// -----------------------------------------------------------------------------
// Forward Declarations

// -----------------------------------------------------------------------------
// Globals
static const uint32_t Flag_RebootAfterUpgrade = 0xDEADBEEF;

// -----------------------------------------------------------------------------
// Functions

/*! \brief Initialize System Clock
 */
void hardware_clockInit(void) {
   /* Select fCPU = 168MHz */
  SystemInit();
}

/*! \brief Initialize PWR Peripheral
 */
void hardware_pwrInit(void) {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  PWR_BackupAccessCmd(ENABLE);
}

/*! \brief Reboot the Micro
 *
 *  Reboot the micro and set one of the backup registers to indicate
 *  that the reboot was triggered by the bootloader.  This will
 *  cause the bootloader to start the app without initializing
 *  any internal drivers
 */
void hardware_rebootToApp(void) {
  RTC_WriteBackupRegister(RTC_BKP_DR19, Flag_RebootAfterUpgrade);
  NVIC_SystemReset();
}

/*! \brief Clear Flag
 *
 *  Clear the backup register.  This will allow subsequent resets
 *  to behave normally
  */
void hardware_clearRebootFlag(void) {
  RTC_WriteBackupRegister(RTC_BKP_DR19, 0);
}

/*! \brief Determine if the reboot was triggered by the bootloader
 */
bool hardware_isRebootToApp(void) {
  return (RTC_ReadBackupRegister(RTC_BKP_DR19) != Flag_RebootAfterUpgrade);
}

/*!
 */
void SD_LowLevel_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOC and GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOD, ENABLE);

  GPIO_PinAFConfig(SDIO_DX_PORT, SDIO_D0_PINSOURCE, GPIO_AF_SDIO);
  GPIO_PinAFConfig(SDIO_DX_PORT, SDIO_D1_PINSOURCE, GPIO_AF_SDIO);
  GPIO_PinAFConfig(SDIO_DX_PORT, SDIO_D2_PINSOURCE, GPIO_AF_SDIO);
  GPIO_PinAFConfig(SDIO_DX_PORT, SDIO_D3_PINSOURCE, GPIO_AF_SDIO);
  GPIO_PinAFConfig(SDIO_CLK_PORT, SDIO_CLK_PINSOURCE, GPIO_AF_SDIO);
  GPIO_PinAFConfig(SDIO_CMD_PORT, SDIO_CMD_PINSOURCE, GPIO_AF_SDIO);

  /* Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins */
  GPIO_InitStructure.GPIO_Pin = SDIO_D0_PIN | SDIO_D1_PIN | SDIO_D2_PIN | SDIO_D3_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SDIO_DX_PORT, &GPIO_InitStructure);

  /* Configure PD.02 CMD line */
  GPIO_InitStructure.GPIO_Pin = SDIO_CMD_PIN;
  GPIO_Init(SDIO_CMD_PORT, &GPIO_InitStructure);

  /* Configure PC.12 pin: CLK pin */
  GPIO_InitStructure.GPIO_Pin = SDIO_CLK_PIN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(SDIO_CLK_PORT, &GPIO_InitStructure);

  /*!< Configure SD_SPI_DETECT_PIN pin: SD Card detect pin */
  GPIO_InitStructure.GPIO_Pin = SDIO_DETECT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SDIO_DETECT_PORT, &GPIO_InitStructure);

  /* Enable the SDIO APB2 Clock */
  RCC_APB2PeriphClockCmd(SDIO_RCC_PERIPH, ENABLE);

  /* Enable the DMA2 Clock */
  RCC_AHB1PeriphClockCmd(SDIO_DMA_PERIPH, ENABLE);
}

/*!
 */
void SD_LowLevel_DeInit(void) {
  GPIO_InitTypeDef  GPIO_InitStructure;

  /*!< Disable SDIO Clock */
  SDIO_ClockCmd(DISABLE);

  /*!< Set Power State to OFF */
  SDIO_SetPowerState(SDIO_PowerState_OFF);

  /*!< DeInitializes the SDIO peripheral */
  SDIO_DeInit();

  /* Disable the SDIO APB2 Clock */
  RCC_APB2PeriphClockCmd(SDIO_RCC_PERIPH, DISABLE);

  GPIO_PinAFConfig(SDIO_DX_PORT, SDIO_D0_PINSOURCE, GPIO_AF_MCO);
  GPIO_PinAFConfig(SDIO_DX_PORT, SDIO_D1_PINSOURCE, GPIO_AF_MCO);
  GPIO_PinAFConfig(SDIO_DX_PORT, SDIO_D2_PINSOURCE, GPIO_AF_MCO);
  GPIO_PinAFConfig(SDIO_DX_PORT, SDIO_D3_PINSOURCE, GPIO_AF_MCO);
  GPIO_PinAFConfig(SDIO_CLK_PORT, SDIO_CLK_PINSOURCE, GPIO_AF_MCO);
  GPIO_PinAFConfig(SDIO_CMD_PORT, SDIO_CMD_PINSOURCE, GPIO_AF_MCO);

  /* Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins */
  GPIO_InitStructure.GPIO_Pin = SDIO_D0_PIN | SDIO_D1_PIN | SDIO_D2_PIN | SDIO_D3_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(SDIO_DX_PORT, &GPIO_InitStructure);

  /* Configure PD.02 CMD line */
  GPIO_InitStructure.GPIO_Pin = SDIO_CMD_PIN;
  GPIO_Init(SDIO_CMD_PORT, &GPIO_InitStructure);

  /* Configure PC.12 pin: CLK pin */
  GPIO_InitStructure.GPIO_Pin = SDIO_CLK_PIN;
  GPIO_Init(SDIO_CLK_PORT, &GPIO_InitStructure);
}

/*!
 */
void SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize) {
  DMA_InitTypeDef SDDMA_InitStructure;

  DMA_ClearFlag(SDIO_DMA_STREAM, SDIO_DMA_FLAG_FEIF | SDIO_DMA_FLAG_DMEIF | SDIO_DMA_FLAG_TEIF | SDIO_DMA_FLAG_HTIF | SDIO_DMA_FLAG_TCIF);

  /* DMA2 Stream3 disable */
  DMA_Cmd(SDIO_DMA_STREAM, DISABLE);

  /* DMA2 Stream3 Config */
  DMA_DeInit(SDIO_DMA_STREAM);

  SDDMA_InitStructure.DMA_Channel = SDIO_DMA_CHANNEL;
  SDDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
  SDDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BufferSRC;
  SDDMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  SDDMA_InitStructure.DMA_BufferSize = BufferSize;
  SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  SDDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
  DMA_Init(SDIO_DMA_STREAM, &SDDMA_InitStructure);
  DMA_ITConfig(SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_FlowControllerConfig(SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);

  /* DMA2 Stream3 enable */
  DMA_Cmd(SDIO_DMA_STREAM, ENABLE);
}

/*!
 */
void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize) {
  DMA_InitTypeDef SDDMA_InitStructure;

  DMA_ClearFlag(SDIO_DMA_STREAM, SDIO_DMA_FLAG_FEIF | SDIO_DMA_FLAG_DMEIF | SDIO_DMA_FLAG_TEIF | SDIO_DMA_FLAG_HTIF | SDIO_DMA_FLAG_TCIF);

  /* DMA2 Stream3 disable */
  DMA_Cmd(SDIO_DMA_STREAM, DISABLE);

  /* DMA2 Stream3 Config */
  DMA_DeInit(SDIO_DMA_STREAM);

  SDDMA_InitStructure.DMA_Channel = SDIO_DMA_CHANNEL;
  SDDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
  SDDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BufferDST;
  SDDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  SDDMA_InitStructure.DMA_BufferSize = BufferSize;
  SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  SDDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
  DMA_Init(SDIO_DMA_STREAM, &SDDMA_InitStructure);
  DMA_ITConfig(SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_FlowControllerConfig(SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);

  /* DMA2 Stream3 enable */
  DMA_Cmd(SDIO_DMA_STREAM, ENABLE);
}

/**
  * @brief  Configures SDIO IRQ channel.
  * @param  None
  * @retval None
  */
void SDIO_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = SDIO_DMA_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_Init(&NVIC_InitStructure);
}
