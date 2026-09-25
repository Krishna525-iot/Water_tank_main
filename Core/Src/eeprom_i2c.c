#include "eeprom_i2c.h"
#include "stm32f1xx_hal.h"
#include <string.h>
extern I2C_HandleTypeDef hi2c2;
#define EEPROM_I2C_ADDR (0x50 << 1)
#define EEPROM_TIMEOUT     100
#define EEPROM_PAGE_SIZE   32

HAL_StatusTypeDef EEPROM_WriteByte(uint16_t addr, uint8_t data)
{
    HAL_StatusTypeDef r;
    r = HAL_I2C_Mem_Write(&hi2c2, EEPROM_I2C_ADDR,
                          addr, I2C_MEMADD_SIZE_16BIT,
                          &data, 1, EEPROM_TIMEOUT);
    HAL_Delay(5);
    return r;
}
HAL_StatusTypeDef EEPROM_ReadByte(uint16_t addr, uint8_t *data)
{
    return HAL_I2C_Mem_Read(&hi2c2, EEPROM_I2C_ADDR,
                            addr, I2C_MEMADD_SIZE_16BIT,
                            data, 1, EEPROM_TIMEOUT);
}
#define EEPROM_WRITE_CYCLE_MS  20U   /* datasheet max 5-10 ms, with margin */
#define EEPROM_WRITE_ATTEMPTS  5U

/* Acknowledge polling: the chip NACKs its address until its internal
 * write cycle has finished. */
static HAL_StatusTypeDef ee_wait_ready(void)
{
    uint32_t start = HAL_GetTick();
    while (HAL_I2C_IsDeviceReady(&hi2c2, EEPROM_I2C_ADDR, 1, 5) != HAL_OK)
    {
        if ((HAL_GetTick() - start) >= EEPROM_WRITE_CYCLE_MS) return HAL_TIMEOUT;
    }
    return HAL_OK;
}

/* Multi-byte HAL_I2C_Mem_Read on this STM32F1 returns a corrupt LAST
 * byte (a copy of the one before it) - measured on the board: bytes
 * written 55 B1 00 00 00 0F 30 read back as ...0F 0F in one read, but
 * correctly one byte at a time. The last byte of the mode block is the
 * power-restore setting, which is why it "turned OFF" after a reboot.
 * So every EEPROM read goes byte by byte (single-byte reads use the
 * HAL's protected 1-byte sequence). */
static HAL_StatusTypeDef ee_read_bytes(uint16_t addr, uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        HAL_StatusTypeDef st = HAL_ERROR;
        for (uint8_t attempt = 0; attempt < 3 && st != HAL_OK; attempt++)
        {
            st = HAL_I2C_Mem_Read(&hi2c2, EEPROM_I2C_ADDR,
                                  (uint16_t)(addr + i), I2C_MEMADD_SIZE_16BIT,
                                  &buf[i], 1, EEPROM_TIMEOUT);
        }
        if (st != HAL_OK) return st;
    }
    return HAL_OK;
}

/* A lock-up of the F1 I2C peripheral (BUSY stuck) fails every later
 * transfer; re-initialising it clears that. */
static void ee_recover_bus(void)
{
    HAL_I2C_DeInit(&hi2c2);
    HAL_I2C_Init(&hi2c2);
}

/* Page-aligned chunks, each written, then read back and compared.
 * Returns HAL_ERROR if any chunk could not be verified. */
HAL_StatusTypeDef EEPROM_WriteBuffer(uint16_t addr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef result = HAL_OK;

    while (len)
    {
        uint16_t room  = EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE);
        uint16_t chunk = (len < room) ? len : room;
        uint8_t  verified = 0;
        HAL_StatusTypeDef st = HAL_OK;
        uint8_t verifyBuf[EEPROM_PAGE_SIZE];

        for (uint8_t attempt = 0; attempt < EEPROM_WRITE_ATTEMPTS && !verified; attempt++)
        {
            if (attempt) HAL_Delay(5);

            /* A previous write may still be in its internal cycle */
            ee_wait_ready();

            st = HAL_I2C_Mem_Write(&hi2c2, EEPROM_I2C_ADDR,
                                   addr, I2C_MEMADD_SIZE_16BIT,
                                   data, chunk, 100);
            if (st != HAL_OK)
            {
                if (st == HAL_BUSY) ee_recover_bus();
                continue;
            }
            if (ee_wait_ready() != HAL_OK) { st = HAL_TIMEOUT; continue; }

            memset(verifyBuf, 0xEE, sizeof(verifyBuf));
            if (ee_read_bytes(addr, verifyBuf, chunk) == HAL_OK &&
                memcmp(verifyBuf, data, chunk) == 0)
            {
                verified = 1;
            }
        }

        if (!verified) result = HAL_ERROR;

        addr += chunk;
        data += chunk;
        len  -= chunk;
    }
    return result;
}
HAL_StatusTypeDef EEPROM_ReadBuffer(uint16_t addr, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    for (uint8_t attempt = 0; attempt < 3 && status != HAL_OK; attempt++)
    {
        if (attempt) HAL_Delay(5);
        ee_wait_ready();   /* a write may still be in its internal cycle */
        status = ee_read_bytes(addr, buf, len);
    }
    return status;
}
static uint8_t ee_crc(uint8_t m, uint8_t mot)
{
    return (uint8_t)(m ^ mot ^ EE_COMMIT_FLAG);
}

void EEPROM_SaveMode(uint8_t mode, uint8_t motor)
{
    uint8_t crc = ee_crc(mode, motor);
    uint8_t zero = 0;
    EEPROM_WriteByte(EE_MODE_BLOCK_ADDR, zero);
    EEPROM_WriteByte(EE_MODE_BLOCK_ADDR + 1, mode);
    EEPROM_WriteByte(EE_MODE_BLOCK_ADDR + 2, motor);
    EEPROM_WriteByte(EE_MODE_BLOCK_ADDR + 3, crc);
    EEPROM_WriteByte(EE_MODE_BLOCK_ADDR + 4, EE_COMMIT_FLAG);
    EEPROM_WriteByte(EE_MODE_BLOCK_ADDR, EE_VALID_FLAG);
}
uint8_t EEPROM_LoadMode(uint8_t *mode, uint8_t *motor)
{
    uint8_t v, c, m, mot, crc;
    EEPROM_ReadByte(EE_MODE_BLOCK_ADDR, &v);
    EEPROM_ReadByte(EE_MODE_BLOCK_ADDR + 4, &c);
    if (v != EE_VALID_FLAG || c != EE_COMMIT_FLAG)
        return 0;
    EEPROM_ReadByte(EE_MODE_BLOCK_ADDR + 1, &m);
    EEPROM_ReadByte(EE_MODE_BLOCK_ADDR + 2, &mot);
    EEPROM_ReadByte(EE_MODE_BLOCK_ADDR + 3, &crc);
    if (crc != ee_crc(m, mot))
        return 0;
    *mode = m;
    *motor = mot;
    return 1;
}
