/*! \file serial.c
 *  \brief
 *
 *  Details
 *
 *    See header file
 *
 *  Copyright (c) 2015 Traxxas, All Rights Reserved
 */

// -----------------------------------------------------------------------------
// Includes
#include <serial.h>
#include <stm32f4xx_usart.h>

// -----------------------------------------------------------------------------
// Definitions

// -----------------------------------------------------------------------------
// Forward Declarations
static void serial_transmit(void);
static void serial_rccConfig(void);
static void serial_gpioConfig(void);
static void serial_usartConfig(void);

// -----------------------------------------------------------------------------
// Globals

static volatile SerialTxInitStructure userTxData = {
    NULL, // userTxBuffer
    0,    // userTxBufferLength
};

// -----------------------------------------------------------------------------
// Functions

/*! \brief Serial Driver Init Function
 *
 *  Initialize the driver and the associated peripherals.  This function
 *  enables receive packets immediately.
 */
void serial_init(void) {
  // Configure the peripheral clocks
  serial_rccConfig();

  // Configure the GPIO pins
  serial_gpioConfig();

  // Configure the USART
  serial_usartConfig();

  // Enable USART
  USART_Cmd(PX4IO_USART, ENABLE);
}

/*! \brief Configure transmit buffer and send
 */
uint32_t serial_txSend(volatile uint8_t* txPacket, uint32_t length) {
  // Store the user parameters (shallow memory copy)
  userTxData.transmitBuffer = txPacket;
  userTxData.transmitBufferLength = length;

  serial_transmit();

  return 0;
}

/*! \brief RCC Config
 *
 *  Configure the peripheral clocks needed by this driver
 */
void serial_rccConfig(void) {
  // Enable GPIO Clock
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  // Enable USART Clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
}

/*! \brief GPIO Config
 *
 *  Configure the GPIO devices needed by this driver
 */
void serial_gpioConfig(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure GPIO as AF pushpull
  GPIO_InitStructure.GPIO_Pin = PX4IO_TX_PIN | PX4IO_RX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(PX4IO_PORT, &GPIO_InitStructure);

  // Configure USART pins: Tx, Rx
  GPIO_PinAFConfig(PX4IO_PORT, PX4IO_TX_PIN_SOURCE, GPIO_AF_USART6);
  GPIO_PinAFConfig(PX4IO_PORT, PX4IO_RX_PIN_SOURCE, GPIO_AF_USART6);
}

/*! \brief USART Config
 *
 *  Configure the USART perpherals needed by this driver
 */
void serial_usartConfig(void) {
  USART_InitTypeDef USART_InitStructure;

  // Initialize USART
  USART_InitStructure.USART_BaudRate = PX4IO_BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = /*USART_Mode_Rx |*/ USART_Mode_Tx;
  USART_Init(PX4IO_USART, &USART_InitStructure);
}

/*! \brief Transmit Message
 */
void serial_transmit(void) {
  uint8_t i = 0;

  while (i < userTxData.transmitBufferLength) {
    while( !(PX4IO_USART->SR & 0x00000040) );
    USART_SendData(PX4IO_USART, userTxData.transmitBuffer[i]); i++;
  }
}
