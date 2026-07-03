#include "rtc_i2c.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
extern I2C_HandleTypeDef hi2c2;
#define DS1307_7BIT_ADDR    0x68
#define DS1307_8BIT_ADDR    (DS1307_7BIT_ADDR << 1)
RTC_Time_t time;
static uint8_t dec2bcd(uint8_t v)
{
    return ((v / 10) << 4) | (v % 10);
}
static uint8_t bcd2dec(uint8_t v)
{
    return ((v >> 4) * 10) + (v & 0x0F);
}
void RTC_Init(void)
{
    uint8_t sec;
    if (HAL_I2C_IsDeviceReady(&hi2c2, DS1307_8BIT_ADDR, 3, 100) != HAL_OK)
    {
        printf("❌ DS1307 NOT found\r\n");
        return;
    }
    HAL_I2C_Mem_Read(&hi2c2, DS1307_8BIT_ADDR,
                     0x00, I2C_MEMADD_SIZE_8BIT,
                     &sec, 1, 100);
    if (sec & 0x80)
    {
        sec &= 0x7F;
        HAL_I2C_Mem_Write(&hi2c2, DS1307_8BIT_ADDR,
                          0x00, I2C_MEMADD_SIZE_8BIT,
                          &sec, 1, 100);
        HAL_Delay(50);
    }
    uint8_t sec2;
    HAL_Delay(1100);
    HAL_I2C_Mem_Read(&hi2c2, DS1307_8BIT_ADDR,
                     0x00, I2C_MEMADD_SIZE_8BIT,
                     &sec2, 1, 100);
    if ((sec2 & 0x7F) == (sec & 0x7F))
    {
        printf("❌ RTC OSCILLATOR NOT RUNNING\r\n");
    }
    else
    {
        printf("✅ RTC RUNNING (seconds ticking)\r\n");
    }
}
void RTC_SetTimeDate(uint8_t sec, uint8_t min, uint8_t hour,
                     uint8_t dow, uint8_t dom, uint8_t month, uint16_t year)
{
    uint8_t buf[7];

    buf[0] = dec2bcd(sec);
    buf[1] = dec2bcd(min);
    buf[2] = dec2bcd(hour);
    buf[3] = dec2bcd(dow);
    buf[4] = dec2bcd(dom);
    buf[5] = dec2bcd(month);
    buf[6] = dec2bcd(year - 2000);
    HAL_I2C_Mem_Write(&hi2c2, DS1307_8BIT_ADDR,
                      0x00, I2C_MEMADD_SIZE_8BIT,
                      buf, 7, 200);
    HAL_Delay(15);
}
void RTC_GetTimeDate(void)
{
    uint8_t buf[7];
    if (HAL_I2C_Mem_Read(&hi2c2, DS1307_8BIT_ADDR,
                         0x00, I2C_MEMADD_SIZE_8BIT,
                         buf, 7, 200) != HAL_OK)
    {
        printf("RTC READ FAIL\r\n");
        return;
    }
    time.sec   = bcd2dec(buf[0] & 0x7F);
    time.min   = bcd2dec(buf[1]);
    time.hour  = bcd2dec(buf[2] & 0x3F);
    time.dow   = bcd2dec(buf[3]);
    time.dom   = bcd2dec(buf[4]);
    time.month = bcd2dec(buf[5]);
    time.year  = 2000 + bcd2dec(buf[6]);
}
