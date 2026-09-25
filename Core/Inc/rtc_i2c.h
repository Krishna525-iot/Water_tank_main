#ifndef RTC_I2C_H
#define RTC_I2C_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// ========================
//   GLOBAL LIVE VARIABLES
// ========================
extern uint8_t g_i2c_rtc_addr;      // RTC detected address (7-bit)
extern uint8_t g_i2c_eeprom_addr;   // EEPROM detected address (7-bit)

// ========================
//   DATA STRUCTURES
// ========================
typedef struct {
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t dow;
    uint8_t dom;
    uint8_t month;
    uint16_t year;
} RTC_Time_t;

typedef struct {
    uint8_t mode;
    uint16_t countdownMin;
    uint16_t twistOn;
    uint16_t twistOff;
    uint16_t crc;      // CRC16
} RTC_PersistState;

// ========================
//   PUBLIC FUNCTIONS
// ========================
void RTC_Init(void);
void RTC_GetTimeDate(void);
void RTC_SetTimeDate(uint8_t sec, uint8_t min, uint8_t hour,
                     uint8_t dow, uint8_t dom, uint8_t month, uint16_t year);

// EEPROM
bool RTC_EEPROM_Write(uint16_t memAddr, const uint8_t *data, uint16_t len);
bool RTC_EEPROM_Read(uint16_t memAddr, uint8_t *data, uint16_t len);

// Persistent State
bool RTC_SavePersistentState(const RTC_PersistState *s);
bool RTC_LoadPersistentState(RTC_PersistState *s);

// Scanner
uint8_t RTC_I2C_ScanDevice(uint8_t start7, uint8_t end7);

// Expose Time Object
extern RTC_Time_t time;

// Set-clock request (PC bench tool over SWD): fields first, magic last.
// dow: 1 = Sunday .. 7 = Saturday (as shown on the LCD); yy = year - 2000
#define RTC_SET_REQUEST_MAGIC 0xA5
typedef struct {
    uint8_t sec, min, hour, dow;
    uint8_t dom, month, yy, magic;
} RTC_SetRequest_t;
extern volatile RTC_SetRequest_t g_rtcSetRequest;
void RTC_ServiceSetRequest(void);

#endif
