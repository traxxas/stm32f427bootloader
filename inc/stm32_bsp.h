/*! \file stm32_bsp.h
 *  \brief
 *
 *  Details
 *
 *    This module contains the peripheral assignments.  The module contains only board level
 *    assigments and initialization.  There should be no application-specific development in
 *    this module.
 *
 *  Copyright (c) 2015 Traxxas, All Rights Reserved
 */

#ifndef __stm32_bsp_h
#define __stm32_bsp_h

// -----------------------------------------------------------------------------
// Includes

#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx_gpio.h>

// -----------------------------------------------------------------------------
// Definitions

#define SDIO_DX_PORT                GPIOC
#define SDIO_D0_PIN                 GPIO_Pin_8
#define SDIO_D0_PINSOURCE           GPIO_PinSource8
#define SDIO_D1_PIN                 GPIO_Pin_9
#define SDIO_D1_PINSOURCE           GPIO_PinSource9
#define SDIO_D2_PIN                 GPIO_Pin_10
#define SDIO_D2_PINSOURCE           GPIO_PinSource10
#define SDIO_D3_PIN                 GPIO_Pin_11
#define SDIO_D3_PINSOURCE           GPIO_PinSource11

#define SDIO_CLK_PORT               GPIOC
#define SDIO_CLK_PIN                GPIO_Pin_12
#define SDIO_CLK_PINSOURCE          GPIO_PinSource12

#define SDIO_DETECT_PORT            GPIOC
#define SDIO_DETECT_PIN             GPIO_Pin_13
#define SDIO_DETECT_PINSOURCE       GPIO_PinSource13

#define SDIO_CMD_PORT               GPIOD
#define SDIO_CMD_PIN                GPIO_Pin_2
#define SDIO_CMD_PINSOURCE          GPIO_PinSource2

#define SDIO_RCC_PERIPH             RCC_APB2Periph_SDIO
#define SDIO_DMA_PERIPH             RCC_AHB1Periph_DMA2

#define SDIO_FIFO_ADDRESS           ((uint32_t)0x40012C80)

#define SDIO_INIT_CLK_DIV           ((uint8_t)0x76)      /*!< SDIO Intialization Frequency (400KHz max) */
#define SDIO_TRANSFER_CLK_DIV       ((uint8_t)0x0)       /*!< SDIO Data Transfer Frequency (25MHz max)  */

#define SD_SDIO_DMA                 DMA2

#define SDIO_DMA_STREAM             DMA2_Stream3
#define SDIO_DMA_CHANNEL            DMA_Channel_4
#define SDIO_DMA_FLAG_FEIF          DMA_FLAG_FEIF3
#define SDIO_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF3
#define SDIO_DMA_FLAG_TEIF          DMA_FLAG_TEIF3
#define SDIO_DMA_FLAG_HTIF          DMA_FLAG_HTIF3
#define SDIO_DMA_FLAG_TCIF          DMA_FLAG_TCIF3
#define SDIO_DMA_IRQn               DMA2_Stream3_IRQn
#define SDIO_DMA_IRQHANDLER         DMA2_Stream3_IRQHandler

// -----------------------------------------------------------------------------
// Forward Declarations

// -----------------------------------------------------------------------------
// Globals

// -----------------------------------------------------------------------------
// Functions

extern void hardware_clockInit(void);

extern void hardware_pwrInit(void);
extern void hardware_rebootToApp(void);
extern bool hardware_isRebootToApp(void);
extern void hardware_clearRebootFlag(void);

extern void SDIO_NVIC_Configuration(void);
extern void SD_LowLevel_Init(void);
extern void SD_LowLevel_DeInit(void);
extern void SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize);
extern void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize);

#endif // __stm32_bsp_h