/* screen.c — complete receiver UI with Device Pairing integrated */
#include "screen.h"
#include "lcd_i2c.h"
#include "switches.h"
#include "model_handle.h"
#include "adc.h"
#include "rtc_i2c.h"
#include "acs712.h"
#include "lora.h"          /* LoRa_EnterPairingMode, LoRa_GetLastPairedDID … */
#include "device_id.h"     /* PairedDev_Count, PairedDev_Get, MAX_PAIRED     */
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef enum {
    UI_WELCOME = 0,
    UI_DASH,
    UI_MENU,
    UI_TIMER_SLOT_SELECT,
    UI_TIMER_EDIT_ON_TIME,
    UI_TIMER_EDIT_OFF_TIME,
    UI_TIMER_EDIT_DAYS,
    UI_TIMER_EDIT_GAP,
    UI_TIMER_EDIT_ENABLE,
    UI_TIMER_EDIT_SUMMARY,
    UI_AUTO_MENU,
    UI_AUTO_EDIT_GAP,
    UI_AUTO_EDIT_MAXRUN,
    UI_AUTO_EDIT_RETRY,
    UI_SEMI_AUTO,
    UI_TWIST,
    UI_TWIST_EDIT_ON,
    UI_TWIST_EDIT_OFF,
    UI_TWIST_EDIT_ON_H,
    UI_TWIST_EDIT_ON_M,
    UI_TWIST_EDIT_OFF_H,
    UI_TWIST_EDIT_OFF_M,
    UI_COUNTDOWN,
    UI_COUNTDOWN_EDIT_MIN,
    UI_DEVSET_MENU,
    UI_SETTINGS_GAP,
    UI_SETTINGS_RETRY,
    UI_SETTINGS_UV,
    UI_SETTINGS_OV,
    UI_SETTINGS_OL,
    UI_SETTINGS_UL,
    UI_SETTINGS_MAXRUN,
    UI_SETTINGS_PWRREST,
    UI_SETTINGS_FACTORY,
    UI_DEVSET_EDIT_DATE,
    UI_DEVSET_EDIT_TIME,
    UI_DEVSET_EDIT_DAY,
    UI_ADD_DEVICE_MENU,
    UI_ADD_DEVICE_PAIR,
    UI_ADD_DEVICE_REMOVE,
    UI_ADD_DEVICE_PAIR_DONE,
    UI_ADD_DEVICE_REMOVE_DONE,
    UI_RESET_CONFIRM,
    UI_NONE,
    UI_MAX_
} UiState;

typedef enum {
    BTN_NONE = 0,
    BTN_RESET,
    BTN_SELECT,
    BTN_UP,
    BTN_DOWN,
    BTN_RESET_LONG,
    BTN_SELECT_LONG,
    BTN_UP_LONG,
    BTN_DOWN_LONG
} UiButton;

static UiState ui      = UI_WELCOME;
static UiState last_ui = UI_NONE;
static bool     screenNeedsRefresh = false;
static uint32_t lastLcdUpdateTime  = 0;
static uint32_t lastUserAction     = 0;

extern void ModelHandle_StartAuto(uint16_t gap_s, uint16_t maxrun_min, uint8_t retry);
extern void ModelHandle_StartTimerNearestSlot(void);
extern void ModelHandle_StopTimer(void);
extern void ModelHandle_StopSemiAuto(void);
extern void ModelHandle_FactoryReset(void);

#define WELCOME_MS          2500
#define CURSOR_BLINK_MS     400
#define AUTO_BACK_MS        60000
#define LONG_PRESS_MS       2000
#define CONTINUOUS_STEP_MS  250
#define DASH_PAGE0_TIME     4500UL
#define DASH_PAGE1_TIME     2500UL
#define DRY_BLINK_MS        400UL
#define CD_EDIT_REPEAT_MS   3000UL   /* hold-DOWN auto-increment in countdown edit */

/* ── Pairing UI state ────────────────────────────────────────────── */
static bool     pairingInProgress   = false;
static bool     pairingComplete     = false;
static bool     pairingTimedOut     = false;
static uint32_t pairingStartTime    = 0;
static uint32_t pairingDoneDispTime = 0;

#define PAIRING_UI_TIMEOUT_MS   32000UL   /* slightly longer than LoRa 30 s */
#define PAIRING_DONE_HOLD_MS     3000UL   /* show result before auto-dismiss */

static bool reset_confirm_yes = false;
static uint8_t discoveredIndex = 0;
extern ADC_Data adcData;
extern TimerSlot timerSlots[5];
extern TwistSettings twistSettings;
extern RTC_Time_t time;
extern volatile bool     manualActive;
extern volatile bool     semiAutoActive;
extern volatile bool     timerActive;
extern volatile bool     countdownActive;
extern volatile bool     twistActive;
extern volatile bool     autoActive;
extern volatile uint32_t countdownDuration;
extern volatile bool     senseDryRun;
extern float g_voltageV;
extern float g_currentA;

static uint8_t edit_on_h = 0, edit_on_m = 0;
static uint8_t edit_off_h = 0, edit_off_m = 0;
static uint8_t time_edit_field = 0;
static uint8_t edit_day_mask   = 0x7F;
static uint8_t edit_day_index  = 0;
static uint8_t edit_gap_min    = 0;
static bool    edit_slot_enabled = true;
static uint8_t currentSlot = 0;
static uint8_t timer_page  = 0;

static uint16_t edit_auto_gap_s      = 60;
static uint16_t edit_auto_maxrun_min = 120;
static uint16_t edit_auto_retry      = 0;
static uint16_t edit_twist_on_s      = 5;
static uint16_t edit_twist_off_s     = 5;
static uint8_t  edit_twist_on_hh     = 6;
static uint8_t  edit_twist_on_mm     = 0;
static uint8_t  edit_twist_off_hh    = 18;
static uint8_t  edit_twist_off_mm    = 0;
static uint16_t edit_countdown_min   = 1;

static uint16_t edit_settings_gap_s     = 10;
static uint8_t  edit_settings_retry     = 3;
static uint16_t edit_settings_uv        = 180;
static uint16_t edit_settings_ov        = 260;
static int      edit_settings_ol        = 6;
static int      edit_settings_ul        = 0;
static uint16_t edit_settings_maxrun    = 120;
static uint8_t  edit_settings_pwrrest   = 0;
static bool     edit_settings_factory_yes = false;
static uint8_t  edit_settings_dry_en    = 1;

static uint8_t  edit_date_dd   = 1;
static uint8_t  edit_date_mm   = 1;
static uint16_t edit_date_yyyy = 2025;
static uint8_t  edit_date_field = 0;
static uint8_t  edit_time_hh   = 0;
static uint8_t  edit_time_min  = 0;
static uint8_t  edit_time_field = 0;
static uint8_t  edit_day_idx2  = 0;

static const char* const dowNames[7] = { "Sun","Mon","Tue","Wed","Thu","Fri","Sat" };

static uint8_t addDevMenuIndex = 0;   /* 0=Pair, 1=Remove */
static uint8_t addDevTypeIndex = 0;   /* DID index for remove screen */

extern uint8_t   ModelHandle_GetTankLevelPercent(void);
extern bool      ModelHandle_IsTankFull(void);
extern DryFSMState ModelHandle_GetDryState(void);
extern bool      ModelHandle_GetDryRunEnable(void);
extern uint16_t  ModelHandle_GetDryRunRetryGap(void);

static const char* const main_menu[] = { "Add New Device", "Device Setup", "Reset To Default" };
#define MAIN_MENU_COUNT 3

static uint8_t menu_idx      = 0;
static uint8_t menu_view_top = 0;

static const char* const devset_menu_items[] = {
    "Dry Run En","Test Time","Retry Gap","Low Volt","High Volt",
    "Over Load","Under Load","Max Run","Set Date","Set Time","Set Day",
    "Power Restore","Factory Reset","Back"
};
#define DEVSET_MENU_COUNT  (sizeof(devset_menu_items)/sizeof(devset_menu_items[0]))

#define DEBOUNCE_MS        15
#define REPEAT_START_MS    600
#define REPEAT_INTERVAL_MS 500

static uint8_t devset_idx      = 0;
static uint8_t devset_view_top = 0;

static uint8_t  dash_page        = 0;
static uint32_t dash_cycle_start = 0;
static uint8_t  dry_blink_slot   = 0;
static uint32_t dry_blink_start  = 0;
static bool     dash_semi_shown      = false;
static bool     dash_countdown_shown = false;

static bool dash_manual_off_shown = false;
static bool dash_auto_off_shown   = false;

/* ── Countdown edit hold-repeat state ────────────────────────────── */
static uint32_t cd_edit_repeat_time  = 0;
static bool     countdown_was_active = false;

static inline void clear_sticky_mode_flags(void)
{
    dash_semi_shown = false;
    dash_countdown_shown = false;
}

static inline void sticky_set_manual_off(void)
{
    dash_manual_off_shown = true;
    dash_auto_off_shown = false;
    dash_semi_shown = false;
    dash_countdown_shown = false;
}

static inline void sticky_set_auto_off(void)
{
    dash_auto_off_shown = true;
    dash_manual_off_shown = false;
    dash_semi_shown = false;
    dash_countdown_shown = false;
}

static inline void clear_all_dash_sticky_flags(void)
{
    dash_manual_off_shown = false;
    dash_auto_off_shown = false;
    clear_sticky_mode_flags();
}

static inline void sticky_set_countdown(void)
{
    dash_countdown_shown = true;
    dash_semi_shown = false;
}

static inline void sticky_set_semi(void)
{
    dash_semi_shown = true;
    dash_countdown_shown = false;
}
/* ════════════════════════════════════════════════════════════════════
 *  LCD HELPERS
 * ════════════════════════════════════════════════════════════════════ */
void Screen_Init(void)
{
    lcd_init(); lcd_clear();
    ui = UI_WELCOME; last_ui = UI_NONE;
    screenNeedsRefresh = true;
    lastUserAction = HAL_GetTick();
}
static inline void refreshInactivityTimer(void) { lastUserAction = HAL_GetTick(); }

static inline void lcd_line(uint8_t row, const char *s)
{
    char buf[17]; snprintf(buf, sizeof(buf), "%-16.16s", s);
    lcd_put_cur(row, 0); lcd_send_string(buf);
}
static inline void lcd_line0(const char *s) { lcd_line(0, s); }
static inline void lcd_line1(const char *s) { lcd_line(1, s); }

/* ════════════════════════════════════════════════════════════════════
 *  DISPLAY FUNCTIONS
 * ════════════════════════════════════════════════════════════════════ */
static void show_welcome(void)  { lcd_line0("   HELONIX"); lcd_line1(" IntelligentSys"); }

static void show_dash(void)
{
    char l0[17], l1[17];
    uint32_t now = HAL_GetTick();
    if (dash_cycle_start == 0) dash_cycle_start = now;

    uint32_t elapsed = now - dash_cycle_start;
    uint8_t  new_page;
    if      (elapsed < DASH_PAGE0_TIME)                     new_page = 0;
    else if (elapsed < (DASH_PAGE0_TIME + DASH_PAGE1_TIME)) new_page = 1;
    else { dash_cycle_start = now; new_page = 0; }

    if (new_page != dash_page)
    {
        dash_page = new_page; screenNeedsRefresh = true;
        dry_blink_slot = 0; dry_blink_start = now; return;
    }

    uint8_t     tankPercent    = ModelHandle_GetTankLevelPercent();
    bool        tankFull       = ModelHandle_IsTankFull();
    bool        motorOn        = Motor_GetStatus();
    DryFSMState dryFSMState    = ModelHandle_GetDryState();
    bool        dryEnabled     = ModelHandle_GetDryRunEnable();
    bool        sensorHasWater = ModelHandle_IsDryRunActive();
    const char *gw             = (adcData.voltages[4] <= 0.01f) ? "YES" : "NO ";

    if (semiAutoActive)  sticky_set_semi();
    if (countdownActive) sticky_set_countdown();
    if (timerActive || autoActive || twistActive || manualActive ||
        ModelHandle_IsRestartActive() || ModelHandle_IsVoltageFault() ||
        ModelHandle_IsOverload()      || ModelHandle_IsUnderload())
        { clear_all_dash_sticky_flags(); }

    const char *mode;
    if      (ModelHandle_IsRestartActive()) mode = "Refill ";
    else if (ModelHandle_IsVoltageFault())  mode = "VOLTERR";
    else if (ModelHandle_IsOverload())      mode = "OVERLD ";
    else if (ModelHandle_IsUnderload())     mode = "UNDERLD";
    else if (timerActive)                   mode = "TIMER";
    else if (autoActive)        mode = motorOn ? "AUTO   " : "AUTO W";
    else if (countdownActive)   mode = motorOn ? "COUNT  " : "CD WAIT";
    else if (twistActive)       mode = motorOn ? "TWIST  " : "TWIST W";
    else if (semiAutoActive)    mode = motorOn ? "SEMI   " : "SEMI";
    else if (manualActive)      mode = "MANUAL ";
    else if (tankFull)          mode = "FULL   ";
    else if (dash_countdown_shown)  mode = "COUNT  ";
    else if (dash_semi_shown)       mode = "SEMI   ";
    else if (dash_manual_off_shown) mode = "MANUAL ";
    else if (dash_auto_off_shown)   mode = "AUTO   ";
    else                            mode = "READY  ";

    if (dash_page == 0)
    {
        snprintf(l0, sizeof(l0), "%-7sM:%-3s%3d%%", mode, motorOn ? "ON " : "OFF", tankPercent);
        if (tankFull)
            snprintf(l1, sizeof(l1), "TANK FULL %02u:%02u", time.hour, time.min);
        else if (!senseDryRun)
            snprintf(l1, sizeof(l1), "G.W:%-3s DRY%02u:%02u", gw, time.hour, time.min);
        else
            snprintf(l1, sizeof(l1), "G.W:%-3s    %02u:%02u",  gw, time.hour, time.min);
        lcd_line0(l0); lcd_line1(l1);
    }
    else
    {
        /*
         * Dashboard Page 2:
         * - Time removed from second page.
         * - Error is shown only when actual error/fault occurs.
         * - Normal condition shows voltage/current and day.
         */

        const char *dow = "---";
        if (time.dow >= 1 && time.dow <= 7)
            dow = dowNames[(time.dow - 1) % 7];

        if (ModelHandle_IsVoltageFault())
        {
            lcd_line0("VOLTAGE ERROR");
            lcd_line1("Check Supply");
        }
        else if (ModelHandle_IsOverload())
        {
            lcd_line0("OVERLOAD ERROR");
            lcd_line1("Check Motor");
        }
        else if (ModelHandle_IsUnderload())
        {
            lcd_line0("UNDERLOAD ERROR");
            lcd_line1("Check Water");
        }
        else
        {
            snprintf(l0, sizeof(l0), "V:%3.0fV I:%4.1fA", g_voltageV, g_currentA);
            snprintf(l1, sizeof(l1), "%-12.12s", dow);

            lcd_line0(l0);
            lcd_line1(l1);
        }
    }
    (void)dryFSMState; (void)dryEnabled; (void)sensorHasWater;
}

static void draw_menu_cursor(void)
{
    if (ui != UI_MENU) return;
    uint8_t row = 255;
    if      (menu_idx == menu_view_top)     row = 0;
    else if (menu_idx == menu_view_top + 1) row = 1;
    if (row <= 1) { lcd_put_cur(row, 0); lcd_send_data('>'); }
}

static void show_menu(void)
{
    char l0[17], l1[17];
    if (menu_idx < menu_view_top)             menu_view_top = menu_idx;
    else if (menu_idx > menu_view_top + 1)    menu_view_top = menu_idx - 1;
    snprintf(l0, sizeof(l0), " %-15.15s", main_menu[menu_view_top]);
    if (menu_view_top + 1 < MAIN_MENU_COUNT)
        snprintf(l1, sizeof(l1), " %-15.15s", main_menu[menu_view_top + 1]);
    else
        snprintf(l1, sizeof(l1), "                ");
    lcd_line0(l0); lcd_line1(l1); draw_menu_cursor();
}

static void show_devset_menu(void)
{
    char l0[17], l1[17];
    if (devset_idx < devset_view_top)           devset_view_top = devset_idx;
    else if (devset_idx > devset_view_top + 1)  devset_view_top = devset_idx - 1;
    uint8_t idx0 = devset_view_top, idx1 = devset_view_top + 1;
    char star0 = ' ', star1 = ' ';
    for (int pass = 0; pass < 2; pass++)
    {
        uint8_t idx  = (pass == 0) ? idx0 : idx1;
        char   *star = (pass == 0) ? &star0 : &star1;
        switch (idx)
        {
            case 0: if (edit_settings_dry_en)       *star = '*'; break;
            case 1: if (edit_settings_gap_s > 0)    *star = '*'; break;
            case 2: if (edit_settings_retry > 0)    *star = '*'; break;
            case 3: if (edit_settings_uv > 0)       *star = '*'; break;
            case 4: if (edit_settings_ov > 0)       *star = '*'; break;
            case 5: if (edit_settings_ol > 0)       *star = '*'; break;
            case 6: if (edit_settings_ul > 0)       *star = '*'; break;
            case 7: if (edit_settings_maxrun > 0)   *star = '*'; break;
            default: break;
        }
    }
    if (idx0 < DEVSET_MENU_COUNT)
        snprintf(l0, sizeof(l0), "%c%c%-14.14s", (devset_idx==idx0?'>':' '), star0, devset_menu_items[idx0]);
    else snprintf(l0, sizeof(l0), "                ");
    if (idx1 < DEVSET_MENU_COUNT)
        snprintf(l1, sizeof(l1), "%c%c%-14.14s", (devset_idx==idx1?'>':' '), star1, devset_menu_items[idx1]);
    else snprintf(l1, sizeof(l1), "                ");
    lcd_line0(l0); lcd_line1(l1);
}

static void show_timer_slot_select(void)
{
    int item1 = timer_page * 2, item2 = item1 + 1;
    char l0[17], l1[17];
    if (item1 == 5) snprintf(l0, sizeof(l0), "%c Back", (currentSlot==5?'>':' '));
    else            snprintf(l0, sizeof(l0), "%c Timer %d", (currentSlot==item1?'>':' '), item1+1);
    if (item2 <= 5)
    {
        if (item2 == 5) snprintf(l1, sizeof(l1), "%c Back", (currentSlot==5?'>':' '));
        else            snprintf(l1, sizeof(l1), "%c Timer %d", (currentSlot==item2?'>':' '), item2+1);
    }
    else snprintf(l1, sizeof(l1), "                ");
    lcd_line0(l0); lcd_line1(l1);
}

static void show_edit_on_time(void)  {}
static void show_edit_off_time(void) {}

static const char* dayNames[] = {
    "Enable All","Disable All","Mon","Tue","Wed","Thu","Fri","Sat","Sun","Next>"
};

static void show_timer_days(void)
{
    lcd_line0("Timer Days"); char buf[17];
    if (edit_day_index >= 2 && edit_day_index <= 8)
    {
        uint8_t d = edit_day_index - 2, isOn = (edit_day_mask >> d) & 1;
        snprintf(buf, sizeof(buf), "> %s (%s)", dayNames[edit_day_index], isOn?"ON":"OFF");
    }
    else snprintf(buf, sizeof(buf), "> %s", dayNames[edit_day_index]);
    lcd_line1(buf);
}
static void show_timer_gap(void)
{
    lcd_line0("Timer Gap (min)"); char buf[17];
    snprintf(buf, sizeof(buf), ">T%u %3u min Next>", (unsigned)(currentSlot+1), (unsigned)edit_gap_min);
    lcd_line1(buf);
}
static void show_timer_enable(void)
{
    char title[17]; snprintf(title, sizeof(title), "T%u Enable?", (unsigned)(currentSlot+1));
    lcd_line0(title); lcd_line1(edit_slot_enabled ? "YES       Next>" : "NO        Next>");
}
static void show_timer_summary(void)
{
    char title[17]; snprintf(title, sizeof(title), "T%u Summary", (unsigned)(currentSlot+1));
    lcd_line0(title); lcd_line1(edit_slot_enabled ? "Enabled     Next>" : "Disabled    Next>");
}
static void show_auto_menu(void)    { lcd_line0("Auto Settings"); lcd_line1(">Gap/Max/Retry"); }
static void show_auto_gap(void)     { lcd_line0("DRY GAP (s)");  char b[17]; snprintf(b,sizeof(b),"val:%03u Next>",edit_auto_gap_s); lcd_line1(b); }
static void show_auto_maxrun(void)  { lcd_line0("MAX RUN (min)"); char b[17]; snprintf(b,sizeof(b),"val:%03u Next>",edit_auto_maxrun_min); lcd_line1(b); }
static void show_auto_retry(void)   { lcd_line0("RETRY COUNT");   char b[17]; snprintf(b,sizeof(b),"val:%03u Next>",edit_auto_retry); lcd_line1(b); }
static void show_semi_auto(void)    { lcd_line0("Semi-Auto"); lcd_line1(semiAutoActive?"val:Disable Next>":"val:Enable  Next>"); }

void ModelHandle_1SecondTask(void)
{
    if (countdownActive && countdownDuration > 0)
    { countdownDuration--; if (countdownDuration == 0) countdownActive = false; }
}

static void show_twist(void)
{
    char l0[17];
    snprintf(l0, sizeof(l0), "Tw %02us/%02us",
             (unsigned)twistSettings.onDurationSeconds,
             (unsigned)twistSettings.offDurationSeconds);
    lcd_line0(l0); lcd_line1(twistActive ? "val:STOP   Next>" : "val:START  Next>");
}

static void show_countdown(void)
{
    char l0[17], l1[17];
    if (!countdownActive)
    {
        uint8_t pct = ModelHandle_GetTankLevelPercent();
        const char *gw = (adcData.voltages[4] <= 0.01f) ? "YES" : "NO ";
        snprintf(l0, sizeof(l0), "COUNT M:OFF%3d%%", pct);
        snprintf(l1, sizeof(l1), "G.W:%-3s    %02u:%02u", gw, time.hour, time.min);
        lcd_line0(l0); lcd_line1(l1); return;
    }
    uint32_t sec = countdownDuration, min = sec / 60, s = sec % 60;
    snprintf(l0, sizeof(l0), "CD %02lu:%02lu RUN", (unsigned long)min, (unsigned long)s);
    lcd_line0(l0); lcd_line1("DOWN=STOP      ");
}

static void show_countdown_edit_min(void) { lcd_line0("SET MINUTES"); char b[17]; snprintf(b,sizeof(b),"val:%03u DOWN+>",edit_countdown_min); lcd_line1(b); }

static void show_settings_gap(void)
{
    lcd_line0("Test Time (min)"); char buf[17];
    if (edit_settings_gap_s == 0) snprintf(buf, sizeof(buf), "Disable    Next>");
    else                          snprintf(buf, sizeof(buf), "val:%2umin Next>", edit_settings_gap_s);
    lcd_line1(buf);
}
static void show_settings_retry(void)
{
    lcd_line0("Retry Gap (min)"); char buf[17];
    if (edit_settings_retry == 0) snprintf(buf, sizeof(buf), "Disable    Next>");
    else                          snprintf(buf, sizeof(buf), "val:%3umin Next>", edit_settings_retry);
    lcd_line1(buf);
}
static void show_settings_uv(void)
{
    lcd_line0("Low Volt"); char buf[17];
    if (edit_settings_uv == 0) snprintf(buf, sizeof(buf), "Disable    Next>");
    else                       snprintf(buf, sizeof(buf), "val:%3uV Next>", edit_settings_uv);
    lcd_line1(buf);
}
static void show_settings_ov(void)
{
    lcd_line0("High Volt"); char buf[17];
    if (edit_settings_ov == 0) snprintf(buf, sizeof(buf), "Disable    Next>");
    else                       snprintf(buf, sizeof(buf), "val:%3uV Next>", edit_settings_ov);
    lcd_line1(buf);
}
static void show_settings_ol(void)
{
    lcd_line0("Over Load (A)"); char buf[17];
    if (edit_settings_ol < 1) snprintf(buf, sizeof(buf), "Disable    Next>");
    else                      snprintf(buf, sizeof(buf), "val:%3d Next>", edit_settings_ol);
    lcd_line1(buf);
}
static void show_settings_ul(void)
{
    lcd_line0("Under Load (A)"); char buf[17];
    if (edit_settings_ul < 1) snprintf(buf, sizeof(buf), "Disable    Next>");
    else                      snprintf(buf, sizeof(buf), "val:%3d Next>", edit_settings_ul);
    lcd_line1(buf);
}
static void show_settings_maxrun(void)
{
    lcd_line0("Max Run"); char buf[17];
    if (edit_settings_maxrun == 0) snprintf(buf, sizeof(buf), "Disable    Next>");
    else                           snprintf(buf, sizeof(buf), "val:%3umin Next>", edit_settings_maxrun);
    lcd_line1(buf);
}
static void show_settings_pwrrest(void)
{
    lcd_line0("Power Restore");
    lcd_line1(edit_settings_pwrrest == 0 ? "YES       Next>" :
              edit_settings_pwrrest == 1 ? "NO        Next>" : "LAST      Next>");
}
static void show_settings_factory(void)
{
    lcd_line0("Factory Reset?");
    lcd_line1(edit_settings_factory_yes ? "YES       Next>" : "NO        Next>");
}
static void show_devset_edit_date(void)
{
    lcd_line0("Set Date"); uint8_t yy2 = (uint8_t)(edit_date_yyyy % 100); char buf[17];
    if      (edit_date_field == 0) snprintf(buf, sizeof(buf), "[%02u]-%02u-%02u", edit_date_dd, edit_date_mm, yy2);
    else if (edit_date_field == 1) snprintf(buf, sizeof(buf), "%02u-[%02u]-%02u", edit_date_dd, edit_date_mm, yy2);
    else                           snprintf(buf, sizeof(buf), "%02u-%02u-[%02u]", edit_date_dd, edit_date_mm, yy2);
    lcd_line1(buf);
}
static void show_devset_edit_time(void)
{
    lcd_line0("Set Time"); char buf[17];
    if (edit_time_field == 0) snprintf(buf, sizeof(buf), "[%02u]:%02u", edit_time_hh, edit_time_min);
    else                      snprintf(buf, sizeof(buf), "%02u:[%02u]", edit_time_hh, edit_time_min);
    lcd_line1(buf);
}
static void show_devset_edit_day(void)
{
    lcd_line0("Set Day"); char buf[17];
    snprintf(buf, sizeof(buf), "> %s", dowNames[edit_day_idx2 % 7]); lcd_line1(buf);
}

/* ── Add Device screens ──────────────────────────────────────────── */
static void show_add_device_menu(void)
{
    lcd_line0(addDevMenuIndex == 0 ? ">Pair Device"   : " Pair Device");
    lcd_line1(addDevMenuIndex == 1 ? ">Remove Device" : " Remove Device");
}

static void show_add_device_pair(void)
{
    char l0[17];
    char l1[17];

    if (pairingComplete)
    {
        uint32_t did = LoRa_GetLastPairedDID();

        lcd_line0("Paired OK!");
        snprintf(l1, sizeof(l1), "%08lX", (unsigned long)did);
        lcd_line1(l1);
        return;
    }

    if (pairingTimedOut)
    {
        lcd_line0("Pairing Failed");
        lcd_line1("SELECT=Back");
        return;
    }

    if (pairingInProgress)
    {
        uint8_t cnt = LoRa_GetDiscoveredCount();

        if (cnt == 0)
        {
            lcd_line0("Searching TX...");
            lcd_line1("RESET=Cancel");
            return;
        }

        if (discoveredIndex >= cnt)
            discoveredIndex = 0;

        LoRa_DiscoveredDevice_t d;

        if (LoRa_GetDiscoveredDevice(discoveredIndex, &d))
        {
            snprintf(l0, sizeof(l0), "TX %u/%u R:%d",
                     (unsigned)(discoveredIndex + 1),
                     (unsigned)cnt,
                     (int)d.rssi);

            snprintf(l1, sizeof(l1), "%08lX SEL",
                     (unsigned long)d.did);

            lcd_line0(l0);
            lcd_line1(l1);
            return;
        }

        lcd_line0("Device Error");
        lcd_line1("RESET=Back");
        return;
    }

    uint8_t pairedCnt = PairedDev_Count();

    snprintf(l0, sizeof(l0), "Pair TX %u/%u",
             (unsigned)pairedCnt,
             (unsigned)MAX_PAIRED);

    lcd_line0(l0);
    lcd_line1(pairedCnt >= MAX_PAIRED ? "Full Remove1st" : "SELECT=Start");
}


static void show_add_device_remove(void)
{
    uint8_t cnt = PairedDev_Count();
    char l0[17], l1[17];
    if (cnt == 0) { lcd_line0("No TX Paired"); lcd_line1("Back>"); return; }
    uint8_t idx = addDevTypeIndex % cnt;
    uint32_t did = PairedDev_Get(idx);
    char hex[9]; snprintf(hex, sizeof(hex), "%08lX", (unsigned long)did);
    snprintf(l0, sizeof(l0), "Remove %u/%u:", idx + 1, cnt);
    snprintf(l1, sizeof(l1), ">%.8s Del?>", hex);
    lcd_line0(l0); lcd_line1(l1);
}

static void show_add_device_pair_done(void)
{
    uint32_t did = LoRa_GetLastPairedDID();
    char hex[9]; snprintf(hex, sizeof(hex), "%08lX", (unsigned long)did);
    lcd_line0("Paired OK!"); lcd_line1(hex);
}

static void show_add_device_remove_done(void)
{
    uint8_t cnt = PairedDev_Count(); char l1[17];
    lcd_line0("TX Removed");
    snprintf(l1, sizeof(l1), "%u TX remaining", cnt); lcd_line1(l1);
}

static void show_reset_confirm(void)
{
    lcd_line0("Reset To Default?");
    lcd_line1(reset_confirm_yes ? "YES       Apply>" : "NO        Back>");
}

/* ════════════════════════════════════════════════════════════════════
 *  SETTINGS HELPERS
 * ════════════════════════════════════════════════════════════════════ */
static void apply_settings_core(void)
{
    uint16_t gap_s = 0;
    if (edit_settings_dry_en && edit_settings_gap_s > 0)
        gap_s = (uint16_t)(edit_settings_gap_s * 60U);
    ModelHandle_SetUserSettings(gap_s, ModelHandle_GetRetryCount(),
        edit_settings_uv, edit_settings_ov,
        edit_settings_ol, edit_settings_ul, edit_settings_maxrun);
    uint32_t retry_s = (edit_settings_retry > 0)
                       ? (uint32_t)edit_settings_retry * 60U : 10U;
    ModelHandle_SetDryRunTime(retry_s);
    ModelHandle_SetDryRun(edit_settings_dry_en && edit_settings_gap_s > 0);
}

static void start_settings_edit_flow(void)
{
    uint16_t gap_s = ModelHandle_GetGapTime();
    if (gap_s == 0) edit_settings_gap_s = 0;
    else
    {
        edit_settings_gap_s = gap_s / 60;
        if (edit_settings_gap_s < 1)   edit_settings_gap_s = 1;
        if (edit_settings_gap_s > 180) edit_settings_gap_s = 180;
    }
    edit_settings_dry_en = ModelHandle_GetDryRunEnable() ? 1 : 0;
    {
        uint16_t dry_s = ModelHandle_GetDryRunRetryGap();
        edit_settings_retry = (dry_s == 0) ? 5 : (uint8_t)(dry_s / 60);
        if (edit_settings_retry < 1)   edit_settings_retry = 1;
        if (edit_settings_retry > 180) edit_settings_retry = 180;
    }
    edit_settings_uv = ModelHandle_GetUnderVolt();
    if (edit_settings_uv && edit_settings_uv < 150) edit_settings_uv = 150;
    if (edit_settings_uv > 200) edit_settings_uv = 200;
    edit_settings_ov = ModelHandle_GetOverVolt();
    if (edit_settings_ov && edit_settings_ov < 250) edit_settings_ov = 250;
    if (edit_settings_ov > 300) edit_settings_ov = 300;
    edit_settings_ol = (int)ModelHandle_GetOverloadLimit();
    if (edit_settings_ol > 25) edit_settings_ol = 25;
    edit_settings_ul = (int)ModelHandle_GetUnderloadLimit();
    if (edit_settings_ul > 10) edit_settings_ul = 10;
    edit_settings_maxrun = ModelHandle_GetMaxRunTime();
    if (edit_settings_maxrun > 300) edit_settings_maxrun = 300;
    edit_settings_pwrrest = ModelHandle_GetPowerRestoreMode();
    edit_settings_factory_yes = false;
    RTC_GetTimeDate();
    edit_date_dd = time.dom; edit_date_mm = time.month; edit_date_yyyy = time.year;
    edit_date_field = 0;
    edit_time_hh = time.hour; edit_time_min = time.min; edit_time_field = 0;
    edit_day_idx2 = (time.dow >= 1 && time.dow <= 7) ? (uint8_t)((time.dow - 1) % 7) : 0;
    devset_idx = 0; devset_view_top = 0;
    ui = UI_DEVSET_MENU; screenNeedsRefresh = true;
}

static void goto_menu_top(void) { menu_idx = 0; menu_view_top = 0; }

/* ════════════════════════════════════════════════════════════════════
 *  MENU SELECT (SELECT button handler)
 * ════════════════════════════════════════════════════════════════════ */
static void menu_select(void)
{
    refreshInactivityTimer();

    if (ui == UI_WELCOME) { ui = UI_DASH; screenNeedsRefresh = true; return; }
    if (ui == UI_DASH)    { goto_menu_top(); ui = UI_MENU; screenNeedsRefresh = true; return; }

    if (ui == UI_MENU)
    {
        switch (menu_idx)
        {
            case 0:
                addDevMenuIndex = 0; addDevTypeIndex = 0;
                pairingInProgress = false; pairingComplete = false; pairingTimedOut = false;
                ui = UI_ADD_DEVICE_MENU; break;
            case 1: start_settings_edit_flow(); return;
            case 2: reset_confirm_yes = false; ui = UI_RESET_CONFIRM; break;
        }
        screenNeedsRefresh = true; return;
    }

    if (ui == UI_DEVSET_MENU)
    {
        switch (devset_idx)
        {
            case 0:  edit_settings_dry_en ^= 1; apply_settings_core(); break;
            case 1:  edit_settings_gap_s   = (edit_settings_gap_s   > 0) ? 0 : 5;  apply_settings_core(); break;
            case 2:  edit_settings_retry   = (edit_settings_retry   > 0) ? 0 : 5;  apply_settings_core(); break;
            case 3:  edit_settings_uv      = (edit_settings_uv      > 0) ? 0 : 180; apply_settings_core(); break;
            case 4:  edit_settings_ov      = (edit_settings_ov      > 0) ? 0 : 260; apply_settings_core(); break;
            case 5:  edit_settings_ol      = (edit_settings_ol      > 0) ? 0 : 6;  apply_settings_core(); break;
            case 6:  edit_settings_ul      = (edit_settings_ul      > 0) ? 0 : 2;  apply_settings_core(); break;
            case 7:  edit_settings_maxrun  = (edit_settings_maxrun  > 0) ? 0 : 60; apply_settings_core(); break;
            case 8:  ui = UI_DEVSET_EDIT_DATE; break;
            case 9:  ui = UI_DEVSET_EDIT_TIME; break;
            case 10: ui = UI_DEVSET_EDIT_DAY;  break;
            case 11:
                edit_settings_pwrrest = (edit_settings_pwrrest + 1) % 3;
                ModelHandle_SetPowerRestoreMode(edit_settings_pwrrest); break;
            case 12:
                edit_settings_factory_yes ^= 1;
                if (edit_settings_factory_yes)
                {
                    ModelHandle_FactoryReset();
                    edit_settings_dry_en  = ModelHandle_GetDryRunEnable() ? 1 : 0;
                    edit_settings_gap_s   = ModelHandle_GetGapTime() / 60;
                    edit_settings_retry   = ModelHandle_GetDryRunRetryGap() / 60;
                    edit_settings_uv      = ModelHandle_GetUnderVolt();
                    edit_settings_ov      = ModelHandle_GetOverVolt();
                    edit_settings_maxrun  = ModelHandle_GetMaxRunTime();
                    edit_settings_ol      = (int)ModelHandle_GetOverloadLimit();
                    edit_settings_ul      = (int)ModelHandle_GetUnderloadLimit();
                    edit_settings_pwrrest = ModelHandle_GetPowerRestoreMode();
                    edit_settings_factory_yes = false;
                    clear_sticky_mode_flags(); ui = UI_DASH;
                }
                break;
            case 13: ui = UI_MENU; break;
        }
        screenNeedsRefresh = true; return;
    }

    if (ui == UI_DEVSET_EDIT_DATE)
    {
        if (edit_date_field < 2) edit_date_field++;
        else
        {
            time.dom = edit_date_dd; time.month = edit_date_mm; time.year = edit_date_yyyy;
            RTC_SetTimeDate(time.sec, time.min, time.hour, time.dow, time.dom, time.month, time.year);
            ui = UI_DEVSET_MENU;
        }
        screenNeedsRefresh = true; return;
    }
    if (ui == UI_DEVSET_EDIT_TIME)
    {
        if (edit_time_field == 0) edit_time_field = 1;
        else
        {
            time.hour = edit_time_hh; time.min = edit_time_min; time.sec = 0;
            RTC_SetTimeDate(time.sec, time.min, time.hour, time.dow, time.dom, time.month, time.year);
            ui = UI_DEVSET_MENU;
        }
        screenNeedsRefresh = true; return;
    }
    if (ui == UI_DEVSET_EDIT_DAY)
    {
        time.dow = edit_day_idx2 + 1;
        RTC_SetTimeDate(time.sec, time.min, time.hour, time.dow, time.dom, time.month, time.year);
        ui = UI_DEVSET_MENU; screenNeedsRefresh = true; return;
    }
}

/* ════════════════════════════════════════════════════════════════════
 *  VALUE EDITING (UP / DOWN buttons in edit screens)
 * ════════════════════════════════════════════════════════════════════ */
void increase_edit_value(uint8_t step)
{
    switch (ui)
    {
        case UI_TIMER_EDIT_ON_TIME:
            if (time_edit_field==0){edit_on_h+=step;if(edit_on_h>23)edit_on_h=23;}
            else{edit_on_m+=step;if(edit_on_m>59)edit_on_m=59;} break;
        case UI_TIMER_EDIT_OFF_TIME:
            if (time_edit_field==0){edit_off_h+=step;if(edit_off_h>23)edit_off_h=23;}
            else{edit_off_m+=step;if(edit_off_m>59)edit_off_m=59;} break;
        case UI_TIMER_EDIT_GAP:
            edit_gap_min+=step; if(edit_gap_min>240)edit_gap_min=240; break;
        case UI_AUTO_EDIT_GAP:    edit_auto_gap_s+=step; break;
        case UI_AUTO_EDIT_MAXRUN: edit_auto_maxrun_min+=step; break;
        case UI_AUTO_EDIT_RETRY:  edit_auto_retry+=step; break;
        case UI_TWIST_EDIT_ON:    edit_twist_on_s+=step; break;
        case UI_TWIST_EDIT_OFF:   edit_twist_off_s+=step; break;
        case UI_TWIST_EDIT_ON_H:  edit_twist_on_hh+=step;  if(edit_twist_on_hh>23)edit_twist_on_hh=23; break;
        case UI_TWIST_EDIT_ON_M:  edit_twist_on_mm+=step;  if(edit_twist_on_mm>59)edit_twist_on_mm=59; break;
        case UI_TWIST_EDIT_OFF_H: edit_twist_off_hh+=step; if(edit_twist_off_hh>23)edit_twist_off_hh=23; break;
        case UI_TWIST_EDIT_OFF_M: edit_twist_off_mm+=step; if(edit_twist_off_mm>59)edit_twist_off_mm=59; break;
        case UI_COUNTDOWN_EDIT_MIN:
            if(edit_countdown_min<180)edit_countdown_min+=1;
            if(edit_countdown_min>180)edit_countdown_min=180; break;
        case UI_SETTINGS_GAP:
            edit_settings_gap_s+=step; if(edit_settings_gap_s>15)edit_settings_gap_s=15; break;
        case UI_SETTINGS_RETRY:
            edit_settings_retry+=step; if(edit_settings_retry>180)edit_settings_retry=180; break;
        case UI_SETTINGS_UV:
            if(edit_settings_uv==0)edit_settings_uv=150;
            else{edit_settings_uv+=step;if(edit_settings_uv>200)edit_settings_uv=200;} break;
        case UI_SETTINGS_OV:
            if(edit_settings_ov==0)edit_settings_ov=250;
            else{edit_settings_ov+=step;if(edit_settings_ov>300)edit_settings_ov=300;} break;
        case UI_SETTINGS_OL: edit_settings_ol+=step; if(edit_settings_ol>25)edit_settings_ol=25; break;
        case UI_SETTINGS_UL: edit_settings_ul+=step; if(edit_settings_ul>10)edit_settings_ul=10; break;
        case UI_SETTINGS_MAXRUN:
            if(edit_settings_maxrun==0)edit_settings_maxrun=10;
            else{edit_settings_maxrun+=step;if(edit_settings_maxrun>300)edit_settings_maxrun=300;} break;
        case UI_SETTINGS_PWRREST: edit_settings_pwrrest=(edit_settings_pwrrest+1)%3; break;
        case UI_SETTINGS_FACTORY: edit_settings_factory_yes^=1; break;
        case UI_DEVSET_EDIT_DATE:
            if(edit_date_field==0){edit_date_dd+=step;if(edit_date_dd>31)edit_date_dd=31;}
            else if(edit_date_field==1){edit_date_mm+=step;if(edit_date_mm>12)edit_date_mm=12;}
            else{edit_date_yyyy+=step;if(edit_date_yyyy>2099)edit_date_yyyy=2099;} break;
        case UI_DEVSET_EDIT_TIME:
            if(edit_time_field==0){edit_time_hh+=step;if(edit_time_hh>23)edit_time_hh=23;}
            else{edit_time_min+=step;if(edit_time_min>59)edit_time_min=59;} break;
        case UI_DEVSET_EDIT_DAY:
            edit_day_idx2=(uint8_t)((edit_day_idx2+1)%7); break;
        default: break;
    }
}

void decrease_edit_value(uint8_t step)
{
    switch (ui)
    {
        case UI_TIMER_EDIT_ON_TIME:
            if(time_edit_field==0){if(edit_on_h>step)edit_on_h-=step;else edit_on_h=0;}
            else{if(edit_on_m>step)edit_on_m-=step;else edit_on_m=0;} break;
        case UI_TIMER_EDIT_OFF_TIME:
            if(time_edit_field==0){if(edit_off_h>step)edit_off_h-=step;else edit_off_h=0;}
            else{if(edit_off_m>step)edit_off_m-=step;else edit_off_m=0;} break;
        case UI_TIMER_EDIT_GAP:
            if(edit_gap_min>step)edit_gap_min-=step;else edit_gap_min=0; break;
        case UI_AUTO_EDIT_GAP:
            if(edit_auto_gap_s>step)edit_auto_gap_s-=step;else edit_auto_gap_s=0; break;
        case UI_AUTO_EDIT_MAXRUN:
            if(edit_auto_maxrun_min>step)edit_auto_maxrun_min-=step;else edit_auto_maxrun_min=0; break;
        case UI_AUTO_EDIT_RETRY:
            if(edit_auto_retry>step)edit_auto_retry-=step;else edit_auto_retry=0; break;
        case UI_SETTINGS_GAP:
            if(edit_settings_gap_s>step)edit_settings_gap_s-=step;else edit_settings_gap_s=0; break;
        case UI_SETTINGS_RETRY:
            if(edit_settings_retry>step)edit_settings_retry-=step;else edit_settings_retry=0; break;
        case UI_SETTINGS_OL:
            if(edit_settings_ol>step)edit_settings_ol-=step;else edit_settings_ol=0; break;
        case UI_SETTINGS_UL:
            if(edit_settings_ul>step)edit_settings_ul-=step;else edit_settings_ul=0; break;
        case UI_SETTINGS_MAXRUN:
            if(edit_settings_maxrun>step)edit_settings_maxrun-=step;else edit_settings_maxrun=0; break;
        case UI_DEVSET_EDIT_DATE:
            if(edit_date_field==0){if(edit_date_dd>step)edit_date_dd-=step;else edit_date_dd=1;}
            else if(edit_date_field==1){if(edit_date_mm>step)edit_date_mm-=step;else edit_date_mm=1;}
            else{if(edit_date_yyyy>(2000+step))edit_date_yyyy-=step;else edit_date_yyyy=2000;} break;
        case UI_DEVSET_EDIT_TIME:
            if(edit_time_field==0){if(edit_time_hh>step)edit_time_hh-=step;else edit_time_hh=0;}
            else{if(edit_time_min>step)edit_time_min-=step;else edit_time_min=0;} break;
        case UI_DEVSET_EDIT_DAY:
            edit_day_idx2=(uint8_t)((edit_day_idx2+6)%7); break;
        default: break;
    }
}

/* ════════════════════════════════════════════════════════════════════
 *  BUTTON DECODER
 * ════════════════════════════════════════════════════════════════════ */
static UiButton decode_button_press(void)
{
    #define BTN_COUNT 4
    typedef struct {
        bool raw, stable; uint32_t lastChange, pressTime, lastRepeat; bool longSent;
    } BtnState;
    static BtnState btn[BTN_COUNT] = {0};
    uint32_t now = HAL_GetTick();

    for (int i = 0; i < BTN_COUNT; i++)
    {
        bool raw = Switch_IsPressed(i);
        if (raw != btn[i].raw) { btn[i].raw = raw; btn[i].lastChange = now; }
        if ((now - btn[i].lastChange) < DEBOUNCE_MS) continue;
        if (btn[i].stable != raw)
        {
            btn[i].stable = raw;
            if (raw) { btn[i].pressTime = now; btn[i].lastRepeat = now; btn[i].longSent = false; }
            else if (!btn[i].longSent)
            {
                switch (i) { case 0: return BTN_RESET; case 1: return BTN_SELECT;
                             case 2: return BTN_UP;    case 3: return BTN_DOWN; }
            }
        }
        if (btn[i].stable && !btn[i].longSent && (now - btn[i].pressTime) >= LONG_PRESS_MS)
        {
            btn[i].longSent = true;
            switch (i) { case 0: return BTN_RESET_LONG;  case 1: return BTN_SELECT_LONG;
                         case 2: return BTN_UP_LONG;      case 3: return BTN_DOWN_LONG; }
        }
        if ((i == 2 || i == 3) && btn[i].stable &&
            (now - btn[i].pressTime) >= REPEAT_START_MS &&
            (now - btn[i].lastRepeat) >= REPEAT_INTERVAL_MS)
        { btn[i].lastRepeat = now; return (i == 2) ? BTN_UP : BTN_DOWN; }
    }
    return BTN_NONE;
}
void Screen_HandleSwitches(void)
{
    UiButton b = decode_button_press();
    uint32_t now_sw = HAL_GetTick();

    {
        static uint32_t rep_start[2] = {0, 0};
        static uint32_t rep_last[2]  = {0, 0};

        /* Exclude UI_COUNTDOWN_EDIT_MIN from fast repeat –
           it uses its own 3-second hold-repeat below */
        bool in_menu = (ui != UI_DASH &&
                        ui != UI_WELCOME &&
                        ui != UI_COUNTDOWN &&
                        ui != UI_COUNTDOWN_EDIT_MIN &&
                        ui != UI_NONE);

        if (in_menu)
        {
            for (int ri = 0; ri < 2; ri++)
            {
                bool held = Switch_IsPressed(ri == 0 ? 2 : 3);

                if (held)
                {
                    if (rep_start[ri] == 0)
                        rep_start[ri] = now_sw;

                    if ((now_sw - rep_start[ri]) >= REPEAT_START_MS &&
                        (now_sw - rep_last[ri]) >= REPEAT_INTERVAL_MS)
                    {
                        rep_last[ri] = now_sw;

                        if (b == BTN_NONE)
                            b = (ri == 0) ? BTN_UP : BTN_DOWN;
                    }
                }
                else
                {
                    rep_start[ri] = 0;
                    rep_last[ri]  = 0;
                }
            }
        }
        else
        {
            rep_start[0] = rep_start[1] = 0;
            rep_last[0]  = rep_last[1]  = 0;
        }
    }

    if (b == BTN_NONE)
    {
        /* ── Countdown-edit: hold-DOWN 3-second auto-increment ────── */
        if (ui == UI_COUNTDOWN_EDIT_MIN && Switch_IsPressed(3))
        {
            if ((now_sw - cd_edit_repeat_time) >= CD_EDIT_REPEAT_MS)
            {
                if (edit_countdown_min < 180) edit_countdown_min++;
                cd_edit_repeat_time = now_sw;
                screenNeedsRefresh = true;
            }
        }
        return;
    }

    refreshInactivityTimer();

    /* ── Countdown running screen: DOWN stops it ─────────────────── */
    if (ui == UI_COUNTDOWN && b == BTN_DOWN)
    {
        ModelHandle_StopCountdown();
        countdown_was_active = false;
        sticky_set_countdown();
        ui = UI_DASH;
        screenNeedsRefresh = true;
        return;
    }

    if (b == BTN_RESET)
    {
        if (ui == UI_DASH)
        {
            if (!ModelHandle_IsRestartActive())
                ModelHandle_StartRestart();
            else
                ModelHandle_StopRestart();

            dash_page = 0;
            dash_cycle_start = HAL_GetTick();
            screenNeedsRefresh = true;
            return;
        }

        if (ui == UI_ADD_DEVICE_PAIR && pairingInProgress)
        {
            pairingInProgress = false;
            pairingComplete   = false;
            pairingTimedOut   = false;
            LoRa_ExitPairingMode();
        }

        switch (ui)
        {
            case UI_SETTINGS_GAP:
            case UI_SETTINGS_RETRY:
            case UI_SETTINGS_UV:
            case UI_SETTINGS_OV:
            case UI_SETTINGS_OL:
            case UI_SETTINGS_UL:
            case UI_SETTINGS_MAXRUN:
            case UI_SETTINGS_PWRREST:
            case UI_SETTINGS_FACTORY:
            case UI_DEVSET_EDIT_DATE:
            case UI_DEVSET_EDIT_TIME:
            case UI_DEVSET_EDIT_DAY:
                ui = UI_DEVSET_MENU;
                break;

            case UI_DEVSET_MENU:
            case UI_ADD_DEVICE_MENU:
            case UI_ADD_DEVICE_PAIR:
            case UI_ADD_DEVICE_REMOVE:
            case UI_ADD_DEVICE_PAIR_DONE:
            case UI_ADD_DEVICE_REMOVE_DONE:
                ui = UI_MENU;
                break;

            case UI_COUNTDOWN_EDIT_MIN:
                ui = UI_DASH;
                break;

            case UI_COUNTDOWN:
                ModelHandle_StopCountdown();
                countdown_was_active = false;
                sticky_set_countdown();
                ui = UI_DASH;
                break;

            default:
                ui = UI_DASH;
                break;
        }

        screenNeedsRefresh = true;
        return;
    }

    if (ui == UI_RESET_CONFIRM)
    {
        if (b == BTN_UP || b == BTN_DOWN)
        {
            reset_confirm_yes = !reset_confirm_yes;
        }
        else if (b == BTN_SELECT)
        {
            if (reset_confirm_yes)
            {
                ModelHandle_FactoryReset();

                edit_settings_dry_en  = ModelHandle_GetDryRunEnable() ? 1 : 0;
                edit_settings_gap_s   = ModelHandle_GetGapTime() / 60;
                edit_settings_retry   = ModelHandle_GetDryRunRetryGap() / 60;
                edit_settings_uv      = ModelHandle_GetUnderVolt();
                edit_settings_ov      = ModelHandle_GetOverVolt();
                edit_settings_maxrun  = ModelHandle_GetMaxRunTime();
                edit_settings_ol      = (int)ModelHandle_GetOverloadLimit();
                edit_settings_ul      = (int)ModelHandle_GetUnderloadLimit();
                edit_settings_pwrrest = ModelHandle_GetPowerRestoreMode();

                clear_sticky_mode_flags();
            }

            ui = UI_DASH;
        }

        screenNeedsRefresh = true;
        return;
    }

    if (ui == UI_DASH)
    {
        switch (b)
        {
            case BTN_RESET_LONG:
            {
                bool wasManual = manualActive;
                ModelHandle_ToggleManual();

                if (wasManual && !manualActive)
                    sticky_set_manual_off();
                else
                    clear_all_dash_sticky_flags();

                screenNeedsRefresh = true;
                break;
            }

            case BTN_SELECT:
            {
                bool wasAuto = autoActive;

                if (!autoActive)
                {
                    ModelHandle_StartAuto(edit_auto_gap_s, edit_auto_maxrun_min, edit_auto_retry);
                    clear_all_dash_sticky_flags();
                }
                else
                {
                    ModelHandle_StopAuto();

                    if (wasAuto && !autoActive)
                        sticky_set_auto_off();
                }

                screenNeedsRefresh = true;
                break;
            }

            case BTN_SELECT_LONG:
                ui = UI_MENU;
                menu_idx = 0;
                menu_view_top = 0;
                screenNeedsRefresh = true;
                return;

            case BTN_UP:
                if (autoActive)
                {
                    if (!timerActive)
                        ModelHandle_Button3_SinglePress();
                    else
                        ModelHandle_StopTimer();
                }
                screenNeedsRefresh = true;
                break;

            case BTN_UP_LONG:
                if (!semiAutoActive)
                {
                    ModelHandle_StartSemiAuto();
                    sticky_set_semi();
                }
                else
                {
                    ModelHandle_StopSemiAuto();
                }

                screenNeedsRefresh = true;
                break;

            case BTN_DOWN:
                if (!countdownActive)
                {
                    ModelHandle_StartCountdown(edit_countdown_min * 60);
                    countdown_was_active = true;
                    sticky_set_countdown();
                    ui = UI_COUNTDOWN;
                }
                else
                {
                    ModelHandle_StopCountdown();
                    countdown_was_active = false;
                    sticky_set_countdown();
                    ui = UI_DASH;
                }

                screenNeedsRefresh = true;
                return;

            case BTN_DOWN_LONG:
                if (!countdownActive)
                {
                    ui = UI_COUNTDOWN_EDIT_MIN;
                    edit_countdown_min = 1;
                    cd_edit_repeat_time = HAL_GetTick();
                    screenNeedsRefresh = true;
                }
                break;

            default:
                break;
        }

        return;
    }

    if (ui == UI_MENU)
    {
        if (b == BTN_SELECT || b == BTN_SELECT_LONG)
            menu_select();
        else if (b == BTN_DOWN && menu_idx < MAIN_MENU_COUNT - 1)
            menu_idx++;
        else if (b == BTN_UP && menu_idx > 0)
            menu_idx--;

        screenNeedsRefresh = true;
        return;
    }

    if (ui == UI_DEVSET_MENU)
    {
        if (b == BTN_SELECT || b == BTN_SELECT_LONG)
            menu_select();
        else if (b == BTN_DOWN && devset_idx < DEVSET_MENU_COUNT - 1)
            devset_idx++;
        else if (b == BTN_UP && devset_idx > 0)
            devset_idx--;

        screenNeedsRefresh = true;
        return;
    }

    if (ui == UI_ADD_DEVICE_MENU)
    {
        if (b == BTN_UP && addDevMenuIndex > 0)
            addDevMenuIndex--;

        if (b == BTN_DOWN && addDevMenuIndex < 1)
            addDevMenuIndex++;

        if (b == BTN_SELECT || b == BTN_SELECT_LONG)
        {
            if (addDevMenuIndex == 0)
            {
                pairingInProgress = false;
                pairingComplete   = false;
                pairingTimedOut   = false;
                discoveredIndex   = 0;
                addDevTypeIndex   = 0;
                ui = UI_ADD_DEVICE_PAIR;
            }
            else
            {
                addDevTypeIndex = 0;
                ui = UI_ADD_DEVICE_REMOVE;
            }
        }

        screenNeedsRefresh = true;
        return;
    }

    if (ui == UI_ADD_DEVICE_PAIR)
    {
        uint8_t cnt = LoRa_GetDiscoveredCount();

        if (pairingComplete || pairingTimedOut)
        {
            if (b == BTN_SELECT || b == BTN_SELECT_LONG)
            {
                pairingInProgress = false;
                pairingComplete   = false;
                pairingTimedOut   = false;
                LoRa_ExitPairingMode();
                ui = UI_ADD_DEVICE_MENU;
            }

            screenNeedsRefresh = true;
            return;
        }

        if (!pairingInProgress)
        {
            if (b == BTN_SELECT || b == BTN_SELECT_LONG)
            {
                if (PairedDev_Count() >= MAX_PAIRED)
                {
                    pairingTimedOut     = true;
                    pairingDoneDispTime = HAL_GetTick();
                }
                else
                {
                    discoveredIndex = 0;

                    LoRa_ClearDiscoveredDevices();
                    LoRa_EnterPairingMode();

                    pairingInProgress   = true;
                    pairingComplete     = false;
                    pairingTimedOut     = false;
                    pairingStartTime    = HAL_GetTick();
                    pairingDoneDispTime = 0;
                }

                screenNeedsRefresh = true;
            }

            return;
        }

        if ((b == BTN_UP || b == BTN_DOWN) && cnt > 1)
        {
            if (b == BTN_UP)
                discoveredIndex = (discoveredIndex == 0) ? (cnt - 1) : (discoveredIndex - 1);
            else
                discoveredIndex = (uint8_t)((discoveredIndex + 1) % cnt);

            screenNeedsRefresh = true;
            return;
        }

        if ((b == BTN_SELECT || b == BTN_SELECT_LONG) && cnt > 0)
        {
            if (discoveredIndex >= cnt)
                discoveredIndex = 0;

            if (LoRa_PairDiscoveredDevice(discoveredIndex))
            {
                pairingInProgress   = false;
                pairingComplete     = true;
                pairingTimedOut     = false;
                pairingDoneDispTime = HAL_GetTick();
            }
            else
            {
                pairingInProgress   = false;
                pairingComplete     = false;
                pairingTimedOut     = true;
                pairingDoneDispTime = HAL_GetTick();
                LoRa_ExitPairingMode();
            }

            screenNeedsRefresh = true;
            return;
        }

        return;
    }

    if (ui == UI_ADD_DEVICE_REMOVE)
    {
        uint8_t cnt = PairedDev_Count();

        if ((b == BTN_UP || b == BTN_DOWN) && cnt > 0)
        {
            addDevTypeIndex = (uint8_t)((addDevTypeIndex + 1) % cnt);
        }
        else if (b == BTN_SELECT || b == BTN_SELECT_LONG)
        {
            if (cnt > 0)
            {
                uint32_t removedDid = PairedDev_Get(addDevTypeIndex % cnt);

                PairedDev_Remove(removedDid);
                LoRa_OnPairingListChanged();

                addDevTypeIndex     = 0;
                pairingDoneDispTime = HAL_GetTick();
                ui = UI_ADD_DEVICE_REMOVE_DONE;
            }
            else
            {
                ui = UI_ADD_DEVICE_MENU;
            }
        }

        screenNeedsRefresh = true;
        return;
    }

    if (ui == UI_ADD_DEVICE_PAIR_DONE || ui == UI_ADD_DEVICE_REMOVE_DONE)
    {
        if (b == BTN_SELECT || b == BTN_SELECT_LONG)
        {
            ui = UI_ADD_DEVICE_MENU;
            screenNeedsRefresh = true;
        }

        return;
    }

    /* ════════════════════════════════════════════════════════════════
     *  Dedicated countdown-edit handler
     *  DOWN short  = +1
     *  DOWN hold   = auto +1 every 3 s  (handled in BTN_NONE block above)
     *  UP          = −1
     *  SELECT      = confirm & start countdown
     *  RESET       = cancel  (already handled in BTN_RESET switch above)
     * ════════════════════════════════════════════════════════════════ */
    if (ui == UI_COUNTDOWN_EDIT_MIN)
    {
        if (b == BTN_DOWN || b == BTN_DOWN_LONG)
        {
            if (edit_countdown_min < 180) edit_countdown_min++;
            cd_edit_repeat_time = HAL_GetTick();
            screenNeedsRefresh = true;
            return;
        }

//        if (b == BTN_UP || b == BTN_UP_LONG)
//        {
//            if (edit_countdown_min > 1) edit_countdown_min--;
//            screenNeedsRefresh = true;
//            return;
//        }

        if (b == BTN_SELECT || b == BTN_SELECT_LONG)
        {
            ModelHandle_StartCountdown(edit_countdown_min * 60);
            countdown_was_active = true;
            sticky_set_countdown();
            ui = UI_COUNTDOWN;
            screenNeedsRefresh = true;
            return;
        }

        screenNeedsRefresh = true;
        return;
    }

    if (ui != UI_DASH)
    {
        if (b == BTN_UP)
            increase_edit_value(1);
        else if (b == BTN_DOWN)
            decrease_edit_value(1);
        else if (b == BTN_UP_LONG)
            increase_edit_value(5);
        else if (b == BTN_DOWN_LONG)
            decrease_edit_value(5);
        else if (b == BTN_SELECT)
            menu_select();

        screenNeedsRefresh = true;
        return;
    }
}

void Screen_Update(void)
{
    uint32_t now = HAL_GetTick();

    if (ui >= UI_MAX_)
    {
        ui = UI_DASH;
        screenNeedsRefresh = true;
    }

    if (ui == UI_WELCOME && (now - lastLcdUpdateTime >= WELCOME_MS))
    {
        lastLcdUpdateTime = now;
        ui = UI_DASH;
        screenNeedsRefresh = true;
    }

    if (ui != UI_WELCOME &&
        ui != UI_DASH &&
        ui != UI_COUNTDOWN &&
        ui != UI_ADD_DEVICE_PAIR &&
        (now - lastUserAction >= AUTO_BACK_MS))
    {
        ui = UI_DASH;
        screenNeedsRefresh = true;
    }

    if (ui == UI_COUNTDOWN && (now - lastLcdUpdateTime) >= 1000UL)
    {
        lastLcdUpdateTime = now;
        screenNeedsRefresh = true;
    }

    /* ── Countdown completed: auto-transition to dash with sticky ── */
    if (ui == UI_COUNTDOWN)
    {
        if (countdownActive)
            countdown_was_active = true;
        else if (countdown_was_active)
        {
            countdown_was_active = false;
            sticky_set_countdown();
            ui = UI_DASH;
            screenNeedsRefresh = true;
        }
    }

    if (ui == UI_ADD_DEVICE_PAIR && pairingInProgress)
    {
        uint8_t cnt = LoRa_GetDiscoveredCount();

        if (cnt > 0 && discoveredIndex >= cnt)
        {
            discoveredIndex = 0;
            screenNeedsRefresh = true;
        }

        if ((now - pairingStartTime) >= PAIRING_UI_TIMEOUT_MS)
        {
            pairingInProgress   = false;
            pairingTimedOut     = true;
            pairingDoneDispTime = now;
            LoRa_ExitPairingMode();
            screenNeedsRefresh = true;
        }
        else if ((now - lastLcdUpdateTime) >= 1000UL)
        {
            lastLcdUpdateTime = now;
            screenNeedsRefresh = true;
        }
    }

    if (ui == UI_ADD_DEVICE_PAIR &&
        (pairingComplete || pairingTimedOut) &&
        (now - pairingDoneDispTime) >= PAIRING_DONE_HOLD_MS)
    {
        pairingComplete   = false;
        pairingTimedOut   = false;
        pairingInProgress = false;

        LoRa_ExitPairingMode();

        ui = UI_ADD_DEVICE_MENU;
        screenNeedsRefresh = true;
    }

    if (ui == UI_ADD_DEVICE_REMOVE_DONE &&
        (now - pairingDoneDispTime) >= PAIRING_DONE_HOLD_MS)
    {
        ui = UI_ADD_DEVICE_MENU;
        screenNeedsRefresh = true;
    }

    if (ui == UI_DASH && dash_cycle_start == 0)
        dash_cycle_start = now;

    if (ui == UI_DASH &&
        dash_page == 0 &&
        (now - dry_blink_start) >= DRY_BLINK_MS)
    {
        dry_blink_slot = dry_blink_slot ? 0 : 1;
        dry_blink_start = now;
        show_dash();
    }

    if (ui == UI_DASH &&
        dash_page == 1 &&
        (now - dry_blink_start) >= DRY_BLINK_MS)
    {
        dry_blink_start = now;
        show_dash();
    }

    if (screenNeedsRefresh || ui != last_ui)
    {
        if (ui != last_ui)
        {
            lcd_clear();
        }

        last_ui = ui;
        screenNeedsRefresh = false;

        if (ui == UI_DASH)
        {
            if (dash_cycle_start == 0)
                dash_cycle_start = now;

            dry_blink_slot  = 0;
            dry_blink_start = now;
        }

        switch (ui)
        {
            case UI_WELCOME:                show_welcome(); break;
            case UI_DASH:                   show_dash(); break;
            case UI_MENU:                   show_menu(); break;
            case UI_DEVSET_MENU:            show_devset_menu(); break;
            case UI_ADD_DEVICE_MENU:        show_add_device_menu(); break;
            case UI_ADD_DEVICE_PAIR:        show_add_device_pair(); break;
            case UI_ADD_DEVICE_REMOVE:      show_add_device_remove(); break;
            case UI_ADD_DEVICE_PAIR_DONE:   show_add_device_pair_done(); break;
            case UI_ADD_DEVICE_REMOVE_DONE: show_add_device_remove_done(); break;
            case UI_DEVSET_EDIT_DATE:       show_devset_edit_date(); break;
            case UI_DEVSET_EDIT_TIME:       show_devset_edit_time(); break;
            case UI_DEVSET_EDIT_DAY:        show_devset_edit_day(); break;
            case UI_RESET_CONFIRM:          show_reset_confirm(); break;
            case UI_COUNTDOWN:              show_countdown(); break;
            case UI_COUNTDOWN_EDIT_MIN:     show_countdown_edit_min(); break;
            case UI_TIMER_SLOT_SELECT:      show_timer_slot_select(); break;
            case UI_TIMER_EDIT_DAYS:        show_timer_days(); break;
            case UI_TIMER_EDIT_GAP:         show_timer_gap(); break;
            case UI_TIMER_EDIT_ENABLE:      show_timer_enable(); break;
            case UI_TIMER_EDIT_SUMMARY:     show_timer_summary(); break;
            case UI_AUTO_MENU:              show_auto_menu(); break;
            case UI_AUTO_EDIT_GAP:          show_auto_gap(); break;
            case UI_AUTO_EDIT_MAXRUN:       show_auto_maxrun(); break;
            case UI_AUTO_EDIT_RETRY:        show_auto_retry(); break;
            case UI_SEMI_AUTO:              show_semi_auto(); break;
            case UI_TWIST:                  show_twist(); break;
            case UI_SETTINGS_GAP:           show_settings_gap(); break;
            case UI_SETTINGS_RETRY:         show_settings_retry(); break;
            case UI_SETTINGS_UV:            show_settings_uv(); break;
            case UI_SETTINGS_OV:            show_settings_ov(); break;
            case UI_SETTINGS_OL:            show_settings_ol(); break;
            case UI_SETTINGS_UL:            show_settings_ul(); break;
            case UI_SETTINGS_MAXRUN:        show_settings_maxrun(); break;
            case UI_SETTINGS_PWRREST:       show_settings_pwrrest(); break;
            case UI_SETTINGS_FACTORY:       show_settings_factory(); break;

            default:
                lcd_line0("Unknown UI");
                lcd_line1("Check state");
                break;
        }
    }
}
