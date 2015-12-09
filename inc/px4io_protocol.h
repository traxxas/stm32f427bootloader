/*! \file px4io_protocol.h
 *  \brief PX4 FMU to PX4 IO Protocol
 *
 *  Details
 *
 *  Definitions extracted from PX4Firmware protocol.h
 */
#ifndef __px4io_protocol_h
#define __px4io_protocol_h

// -----------------------------------------------------------------------------
// Includes

// Standard Includes
#include <stdint.h>
#include <stddef.h>

// -----------------------------------------------------------------------------
// Definitions

#define PKT_MAX_REGS     32    /*!< by agreement w/FMU */
#define PKT_CODE_READ    0x00  /*!< FMU->IO read transaction */
#define PKT_CODE_WRITE   0x40  /*!< FMU->IO write transaction */
#define PKT_CODE_SUCCESS 0x00  /*!< IO->FMU success reply */
#define PKT_CODE_CORRUPT 0x40  /*!< IO->FMU bad packet reply */
#define PKT_CODE_ERROR   0x80  /*!< IO->FMU register op error reply */

#define PKT_CODE_MASK    0xc0
#define PKT_COUNT_MASK   0x3f

#define PKT_COUNT(_p)    ((_p).count_code & PKT_COUNT_MASK)
#define PKT_CODE(_p)     ((_p).count_code & PKT_CODE_MASK)
#define PKT_SIZE(_p)     ((size_t)((uint8_t *)&((_p).regs[PKT_COUNT(_p)]) - ((uint8_t *)&(_p))))

/*!
 */
#pragma pack(push, 1)
struct IOPacket {
  uint8_t  count_code;
  uint8_t  crc;
  uint8_t  page;
  uint8_t  offset;
  uint16_t regs[PKT_MAX_REGS];
};
#pragma pack(pop)

typedef struct IOPacket IOPacket;

#define PX4IO_PKT_SIZE   (sizeof(IOPacket)) /*!< bytes, sizeof(IOPacket) */

// -----------------------------------------------------------------------------
// Globals

// -----------------------------------------------------------------------------
// Functions

#endif // __px4io_protocol_h