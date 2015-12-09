/*! \file serial.h
 *  \brief
 *
 *  Details
 *
 *    Serial interface driver for the PX4FMU to PX4IO communication
 *    UART.  This driver manages the UART and DMA peripherals.
 *
 *  Copyright (c) 2015 Traxxas, All Rights Reserved
 */
#ifndef __serial_h
#define __serial_h

// -----------------------------------------------------------------------------
// Includes
#include <stdint.h>
#include <stddef.h>

// -----------------------------------------------------------------------------
// Definitions

// Resource Definitions
#define PX4IO_USART                 (USART6)
#define PX4IO_USART_IRQ             (USART6_IRQn)
#define PX4IO_PORT                  (GPIOC)
#define PX4IO_TX_PIN                (GPIO_Pin_6)
#define PX4IO_TX_PIN_SOURCE         (GPIO_PinSource6)
#define PX4IO_RX_PIN                (GPIO_Pin_7)
#define PX4IO_RX_PIN_SOURCE         (GPIO_PinSource7)

#define PX4IO_BAUDRATE              1500000

/*! \brief User supplied Receive Callback Function
 *
 *  The user is passed the received packet length.  The packet data
 *  is currently stored in the user supplied receive buffer.  The DMA
 *  peripheral has been disabled to prevent this data from being overwritten;
 *  however, it is the responsibility of the user callback to store any
 *  necessary information from the packet before yeilding control back to
 *  the driver.
 */
typedef void (*SerialRxPacketCallback)(uint32_t length);

/*! \brief Transmit Init Structure
 *
 *  Rethink this.  crude.
 */
typedef struct {
  /*! User supplied transmit buffer
   */
  volatile uint8_t* transmitBuffer;

  /*! Length of the user supplied transmit buffer
   */
  volatile uint32_t transmitBufferLength;
} SerialTxInitStructure;


// -----------------------------------------------------------------------------
// Globals

// -----------------------------------------------------------------------------
// Functions

extern void serial_init(void);
extern uint32_t serial_txSend(volatile uint8_t* txPacket, uint32_t length);

#endif // __serial_h