#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "main.h"

/* 8-bit I2C address for HAL (7-bit << 1):
   0x27 (7-bit) -> 0x4E (8-bit), 0x3F (7-bit) -> 0x7E (8-bit) */
#define SLAVE_ADDRESS_LCD 0x4E  // change to 0x7E if your scan shows 0x3F :contentReference[oaicite:2]{index=2}

/* Select your backpack pin mapping:
   A) Data D7..D4 on P7..P4, EN=P2, RW=P1, RS=P0, BL=P3  (your current code) :contentReference[oaicite:3]{index=3}
   B) Data D4..D7 on P0..P3, EN=P4, RW=P5, RS=P6, BL=P7  (alt common)
*/
#define LCD_PINMAP_A  0
#define LCD_PINMAP_B  1
#ifndef LCD_PINMAP
#define LCD_PINMAP LCD_PINMAP_A   // try A first; if no text, switch to B and rebuild
#endif

#ifdef __cplusplus
extern "C" {
#endif

void lcd_init(void);
void lcd_clear(void);
void lcd_put_cur(uint8_t row, uint8_t col);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_send_string(char *str);
void lcd_backlight_on(void);
void lcd_backlight_off(void);
void lcd_self_test(void);

#ifdef __cplusplus
}
#endif

#endif /* LCD_I2C_H */
