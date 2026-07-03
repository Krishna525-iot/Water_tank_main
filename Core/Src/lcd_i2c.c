#include "lcd_i2c.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c2;
#define LCD_I2C_ADDR        0x4E
#define LCD_BACKLIGHT_BIT   0x08
#define LCD_ENABLE_BIT      0x04
#define LCD_RS_BIT          0x01
static uint8_t g_backlight = LCD_BACKLIGHT_BIT;
static HAL_StatusTypeDef lcd_i2c_write(uint8_t data)
{
    return HAL_I2C_Master_Transmit(&hi2c2, LCD_I2C_ADDR, &data, 1, 5);
}
static void lcd_pulse_enable(uint8_t data)
{
    lcd_i2c_write(data | LCD_ENABLE_BIT);
    HAL_Delay(2);
    lcd_i2c_write(data & ~LCD_ENABLE_BIT);
    HAL_Delay(2);
}
static void lcd_write4(uint8_t nibble, uint8_t rs)
{
    uint8_t data = (nibble & 0xF0);

    if (rs) data |= LCD_RS_BIT;
    data |= g_backlight;

    lcd_i2c_write(data);
    lcd_pulse_enable(data);
}
void lcd_send_cmd(uint8_t cmd)
{
    lcd_write4(cmd & 0xF0, 0);
    lcd_write4((cmd << 4) & 0xF0, 0);
    HAL_Delay(2);
}
void lcd_send_data(uint8_t data)
{
    lcd_write4(data & 0xF0, 1);
    lcd_write4((data << 4) & 0xF0, 1);
}
void lcd_backlight_on(void)
{
    g_backlight = LCD_BACKLIGHT_BIT;
    lcd_i2c_write(g_backlight);
}
void lcd_backlight_off(void)
{
    g_backlight = 0;
    lcd_i2c_write(g_backlight);
}
void lcd_clear(void)
{
    lcd_send_cmd(0x01);
    HAL_Delay(3);
}
void lcd_put_cur(uint8_t row, uint8_t col)
{
    uint8_t base = (row == 0 ? 0x80 : 0xC0);
    lcd_send_cmd(base + col);
}
void lcd_send_string(char *str)
{
    while (*str)
        lcd_send_data(*str++);
}
void lcd_init(void)
{
    HAL_Delay(50);
    lcd_backlight_on();
    for (int i = 0; i < 3; i++)
    {
        lcd_write4(0x30, 0);
        HAL_Delay(5);
    }
    lcd_write4(0x20, 0);
    HAL_Delay(5);
    lcd_send_cmd(0x28);
    lcd_send_cmd(0x08);
    lcd_clear();
    lcd_send_cmd(0x06);
    lcd_send_cmd(0x0C);
    HAL_Delay(5);
}
void lcd_self_test(void)
{
    lcd_init();
    lcd_clear();
    lcd_put_cur(0, 0);
    lcd_send_string("LCD OK");
    lcd_put_cur(1, 0);
    lcd_send_string("I2C READY");
}
