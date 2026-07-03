#ifndef EEPROM_I2C_H
#define EEPROM_I2C_H

#include "stm32f1xx_hal.h"

/* ============================================================
   EXISTING SETTINGS EEPROM MAP (UNCHANGED)
   ============================================================ */

#define EE_ADDR_GAP_TIME        0x00   // uint16
#define EE_ADDR_RETRY_COUNT     0x02   // uint8
#define EE_ADDR_UV_LIMIT        0x03   // uint16
#define EE_ADDR_OV_LIMIT        0x05   // uint16
#define EE_ADDR_OVERLOAD       0x07   // int16 (2 bytes)
#define EE_ADDR_UNDERLOAD      0x09   // int16 (2 bytes)
#define EE_ADDR_MAXRUN         0x0B   // uint16 (2 bytes)

#define EE_ADDR_SIGNATURE      0x20
#define SETTINGS_SIGNATURE     0x55AA


/* ============================================================
   PROTECTED MODE STORAGE (NEW)
   ============================================================ */

#define EE_MODE_BLOCK_ADDR     0x30    // SAFE AREA (does not overlap settings)

#define EE_VALID_FLAG          0xA5
#define EE_COMMIT_FLAG         0x5A


/* ============================================================
   LOW LEVEL EEPROM API
   ============================================================ */

HAL_StatusTypeDef EEPROM_WriteByte(uint16_t addr, uint8_t data);
HAL_StatusTypeDef EEPROM_ReadByte(uint16_t addr, uint8_t *data);
HAL_StatusTypeDef EEPROM_WriteBuffer(uint16_t addr, uint8_t *buf, uint16_t len);
HAL_StatusTypeDef EEPROM_ReadBuffer(uint16_t addr, uint8_t *buf, uint16_t len);


/* ============================================================
   SAFE MODE STORAGE API
   ============================================================ */

void    EEPROM_SaveMode(uint8_t mode, uint8_t motor);
uint8_t EEPROM_LoadMode(uint8_t *mode, uint8_t *motor);


#endif
