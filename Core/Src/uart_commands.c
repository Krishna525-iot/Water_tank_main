#include "uart_commands.h"
#include "uart.h"
#include "model_handle.h"
#include "relay.h"
#include "rtc_i2c.h"
#include "acs712.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

extern bool g_screenUpdatePending;
extern TimerSlot timerSlots[5];
extern volatile uint8_t motorStatus;
extern volatile bool manualActive;
extern volatile bool semiAutoActive;
extern volatile bool timerActive;
extern volatile bool countdownActive;
extern volatile bool twistActive;
extern volatile bool autoActive;
static inline void ack(const char *msg) { UART_TransmitPacket(msg); }
static inline void err(const char *msg) { UART_TransmitPacket(msg); }
typedef struct {
    uint8_t level;
    uint8_t motor;
    char    mode[16];
} StatusSnapshot;
static StatusSnapshot lastSent = {255, 255, "NONE"};
void UART_InitCommandSystem(void)
{
    lastSent.level = 255;
    lastSent.motor = 255;
    strcpy(lastSent.mode, "NONE");
}
void UART_SendStatusPacket(void)
{
    uint8_t level = ModelHandle_GetTankLevelPercent();
    const char *mode = "STANDBY";
    if      (ModelHandle_IsRestartActive()) mode = "RESTART";
    else if (manualActive)                  mode = "MANUAL";
    else if (semiAutoActive)                mode = "SEMIAUTO";
    else if (timerActive)                   mode = "TIMER";
    else if (countdownActive || countdownMode) mode = "COUNTDOWN";
    else if (twistActive)                  mode = "TWIST";
    else if (ModelHandle_IsAutoActive())    mode = "AUTO";
    bool changed = (lastSent.level != level) ||
                   (lastSent.motor != motorStatus) ||
                   (strcmp(lastSent.mode, mode) != 0);
    if (!changed) return;
    lastSent.level = level;
    lastSent.motor = motorStatus;
    strncpy(lastSent.mode, mode, sizeof(lastSent.mode) - 1);
    char buf[80];
    snprintf(buf, sizeof(buf),
             "@STATUS:%s:%d:%s#",
             motorStatus ? "ON" : "OFF",
             level,
             mode);
    UART_TransmitPacket(buf);
}

static char* next_token(char **ctx)
{
    if (!ctx || !*ctx) return NULL;
    char *start = *ctx;
    char *sep   = strchr(start, ':');
    if (sep) { *sep = '\0'; *ctx = sep + 1; }
    else       { *ctx = NULL; }
    return start;
}

static void parse_settings(char *ctx)
{
    if (!ctx) return;
    uint32_t gap_time_s = ModelHandle_GetGapTime();
    uint8_t  retry      = ModelHandle_GetRetryCount();
    uint16_t maxrun     = ModelHandle_GetMaxRunTime();
    uint16_t lowV       = ModelHandle_GetUnderVolt();
    uint16_t highV      = ModelHandle_GetOverVolt();
    int16_t  overL      = (int16_t)ModelHandle_GetOverloadLimit();
    int16_t  underL     = (int16_t)ModelHandle_GetUnderloadLimit();
    uint8_t  powerRest  = ModelHandle_GetPowerRestoreMode();
    uint32_t dry_time   = ModelHandle_GetDryRunRetryGap();
    uint8_t  dry_en     = ModelHandle_GetDryRunEnable() ? 1 : 0;
    uint8_t  buzz_pump  = ModelHandle_GetBuzzerPumpOnSound();
    uint8_t  buzz_full  = ModelHandle_GetBuzzerTankFullSound();
    uint8_t  buzz_empty = ModelHandle_GetBuzzerTankEmptySound();
    bool     dry_en_set = false;
    char *saveptr;
    char *pair = strtok_r(ctx, ";", &saveptr);
    while (pair)
    {
        char *eq = strchr(pair, '=');
        if (eq)
        {
            *eq = '\0';
            char *key = pair;
            char *val = eq + 1;
            int   v   = atoi(val);
            if      (!strcmp(key, "D"))  { gap_time_s = (uint32_t)v * 60UL; }
            else if (!strcmp(key, "RC")) { retry       = (uint8_t)v; }
            else if (!strcmp(key, "T"))  { dry_time    = (uint32_t)v * 60UL; }
            else if (!strcmp(key, "M"))  { maxrun      = (uint16_t)v; }
            else if (!strcmp(key, "LV")) { lowV        = (uint16_t)v; }
            else if (!strcmp(key, "HV")) { highV       = (uint16_t)v; }
            else if (!strcmp(key, "OL")) { overL       = (int16_t)v; }
            else if (!strcmp(key, "UL")) { underL      = (int16_t)v; }
            else if (!strcmp(key, "PR")) { powerRest   = (uint8_t)v; }
            else if (!strcmp(key, "DE")) { dry_en      = (uint8_t)(v ? 1 : 0); dry_en_set = true; }
            else if (!strcmp(key, "BZ")) { buzz_pump   = (uint8_t)(v ? 1 : 0); }
            else if (!strcmp(key, "BF")) { buzz_full   = (uint8_t)(v ? 1 : 0); }
            else if (!strcmp(key, "BE")) { buzz_empty  = (uint8_t)(v ? 1 : 0); }
        }
        pair = strtok_r(NULL, ";", &saveptr);
    }
    if (!dry_en_set && gap_time_s == 0) dry_en = 0;
    ModelHandle_SetUserSettings(gap_time_s, retry, lowV, highV, overL, underL, maxrun);
    ModelHandle_SetDryRunTime(dry_time);
    ModelHandle_SetPowerRestoreMode(powerRest);
    ModelHandle_SetDryRun(dry_en != 0);
    ModelHandle_SetBuzzerSettings(buzz_pump, buzz_full, buzz_empty);
    char resp[48];
    snprintf(resp, sizeof(resp), "@SOK:DR:%d#", ModelHandle_GetDryRunEnable() ? 1 : 0);
    ack(resp);
}
/* Same keys as SET, so the app can show what the device really holds */
static void send_settings(void)
{
    char buf[160];
    snprintf(buf, sizeof(buf),
        "@SETTINGS_DATA:D=%u;T=%u;RC=%u;M=%u;LV=%u;HV=%u;OL=%d;UL=%d;PR=%u;DE=%u;"
        "BZ=%u;BF=%u;BE=%u;CD=%u#",
        (unsigned)(ModelHandle_GetGapTime() / 60),
        (unsigned)(ModelHandle_GetDryRunRetryGap() / 60),
        (unsigned)ModelHandle_GetRetryCount(),
        (unsigned)ModelHandle_GetMaxRunTime(),
        (unsigned)ModelHandle_GetUnderVolt(),
        (unsigned)ModelHandle_GetOverVolt(),
        (int)ModelHandle_GetOverloadLimit(),
        (int)ModelHandle_GetUnderloadLimit(),
        (unsigned)ModelHandle_GetPowerRestoreMode(),
        ModelHandle_GetDryRunEnable() ? 1u : 0u,
        (unsigned)ModelHandle_GetBuzzerPumpOnSound(),
        (unsigned)ModelHandle_GetBuzzerTankFullSound(),
        (unsigned)ModelHandle_GetBuzzerTankEmptySound(),
        (unsigned)ModelHandle_GetCountdownDefaultMin());
    UART_TransmitPacket(buf);
}

static void send_twist_info(void)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "@TWIST_DATA:%u:%u:%02u:%02u:%02u:%02u:%u#",
             (unsigned)(twistSettings.onDurationSeconds  / 60U),
             (unsigned)(twistSettings.offDurationSeconds / 60U),
             twistSettings.onHour,  twistSettings.onMinute,
             twistSettings.offHour, twistSettings.offMinute,
             twistActive ? 1u : 0u);
    UART_TransmitPacket(buf);
}

/* Integer + tenths, no float printf needed */
static void send_calibration(void)
{
    char buf[80];
    int v  = (int)(g_voltageV * 10.0f + 0.5f);
    int a  = (int)(g_currentA * 100.0f + 0.5f);
    int vf = (int)(ACS712_GetVoltageFactor() * 10.0f + 0.5f);
    int ig = (int)(ACS712_GetCurrentGain()   * 1000.0f + 0.5f);
    snprintf(buf, sizeof(buf), "@CAL_DATA:V=%d.%d;I=%d.%02d;VF=%d.%d;IG=%d.%03d#",
             v / 10, v % 10, a / 100, a % 100, vf / 10, vf % 10, ig / 1000, ig % 1000);
    UART_TransmitPacket(buf);
}

/* App countdown value: minutes. Larger values are taken as seconds for
 * older app builds that sent seconds; either way the countdown is capped
 * at COUNTDOWN_MAX_MIN when it starts. */
static uint32_t countdown_arg_to_seconds(const char *s)
{
    uint32_t v = s ? (uint32_t)atoi(s) : 0;
    if (v == 0)   return (uint32_t)ModelHandle_GetCountdownDefaultMin() * 60UL;
    if (v <= 180) return v * 60UL;
    return v;
}

static void send_timer_info(void)
{
    char buf[80];
    const TimerSlot *slots = ModelHandle_GetTimerSlots();
    for (int i = 0; i < 5; i++)
    {
        const TimerSlot *t = &slots[i];
        snprintf(buf, sizeof(buf),
            "@TSLOT:%d:%02u:%02u:%02u:%02u:0x%02X:%d#",
            i + 1,
            t->onHour, t->onMinute,
            t->offHour, t->offMinute,
            t->dayMask,
            t->enabled);
        UART_TransmitPacket(buf);
    }
}
void UART_HandleCommand(const char *pkt)
{
    if (!pkt || !*pkt) return;
    char buf[UART_RX_BUFFER_SIZE];
    strncpy(buf, pkt, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    if (buf[0] == '@') memmove(buf, buf + 1, strlen(buf));
    char *end = strchr(buf, '#');
    if (end) *end = '\0';
    char *ctx = buf;
    char *cmd = next_token(&ctx);
    if (!cmd) return;
    if (!strcmp(cmd, "PING"))   { ack("@PONG#"); return; }
    if (!strcmp(cmd, "STATUS")) { UART_SendStatusPacket(); return; }
    if (!strcmp(cmd, "TINFO"))  { send_timer_info(); return; }
    if (!strcmp(cmd, "TIME"))
    {
        /* TIME:SET:HH:MM:SS:DD:MM:YYYY:DOW   (DOW 1 = Sunday .. 7 = Saturday)
         * TIME:GET */
        char *sub = next_token(&ctx);
        if (sub && !strcmp(sub, "SET"))
        {
            char *f[7];
            for (int i = 0; i < 7; i++) f[i] = next_token(&ctx);
            if (!f[6]) { err("@FORMAT#"); return; }
            int hh = atoi(f[0]), mi = atoi(f[1]), ss = atoi(f[2]);
            int dd = atoi(f[3]), mo = atoi(f[4]), yy = atoi(f[5]), dw = atoi(f[6]);
            if (hh > 23 || mi > 59 || ss > 59 || dd < 1 || dd > 31 ||
                mo < 1 || mo > 12 || yy < 2000 || yy > 2099 || dw < 1 || dw > 7)
                { err("@TIME_ERR#"); return; }
            RTC_SetTimeDate((uint8_t)ss, (uint8_t)mi, (uint8_t)hh,
                            (uint8_t)dw, (uint8_t)dd, (uint8_t)mo, (uint16_t)yy);
            RTC_GetTimeDate();
            ack("@TIME_SET_OK#");
        }
        else if (!sub || strcmp(sub, "GET")) { err("@FORMAT#"); return; }
        char resp[48];
        snprintf(resp, sizeof(resp), "@TIME_DATA:%02u:%02u:%02u:%02u:%02u:%04u:%u#",
                 time.hour, time.min, time.sec, time.dom, time.month, time.year, time.dow);
        ack(resp);
        return;
    }
    if (!strcmp(cmd, "GETSETTINGS")) { send_settings(); return; }
    if (!strcmp(cmd, "SET") || !strcmp(cmd, "SETTINGS"))
    {
        if (ctx && !strcmp(ctx, "GET")) { send_settings(); return; }
        parse_settings(ctx);
        send_settings();
        return;
    }
    if (!strcmp(cmd, "FACTORY"))
    {
        char *sub = next_token(&ctx);
        if (!sub || strcmp(sub, "RESET")) { err("@FORMAT#"); return; }
        ModelHandle_FactoryReset();
        ack("@FACTORY_RESET_OK#");
        send_settings();
        return;
    }
    if (!strcmp(cmd, "CAL"))
    {
        char *sub = next_token(&ctx);
        char *val = next_token(&ctx);
        if (!sub) { err("@FORMAT#"); return; }
        if (!strcmp(sub, "GET")) { send_calibration(); return; }
        if (!val) { err("@FORMAT#"); return; }
        bool ok = false;
        if      (!strcmp(sub, "V")) ok = ACS712_CalibrateVoltage((float)atof(val));
        else if (!strcmp(sub, "I")) ok = ACS712_CalibrateCurrent((float)atof(val));
        else { err("@FORMAT#"); return; }
        ack(ok ? "@CAL_OK#" : "@CAL_ERR#");
        send_calibration();
        return;
    }
    if (!strcmp(cmd, "BUZZER"))
    {
        char *pump  = next_token(&ctx);
        if (pump && !strcmp(pump, "GET")) { send_settings(); return; }
        char *full  = next_token(&ctx);
        char *empty = next_token(&ctx);
        if (!pump || !full || !empty) { err("@FORMAT#"); return; }
        ModelHandle_SetBuzzerSettings((uint8_t)atoi(pump), (uint8_t)atoi(full), (uint8_t)atoi(empty));
        ack("@BUZZER_OK#");
        return;
    }
    if (!strcmp(cmd, "DRYRUN"))
    {
        char *state = next_token(&ctx);
        if (!state) { err("@FORMAT#"); return; }
        if (!strcmp(state, "ON"))
        {
            ModelHandle_SetDryRun(true);
            char resp[32];
            snprintf(resp, sizeof(resp), "@DRYRUN_ON:GAP:%u#", ModelHandle_GetGapTime());
            ack(resp);
        }
        else if (!strcmp(state, "OFF"))
        {
            ModelHandle_SetDryRun(false);
            ack("@DRYRUN_OFF#");
        }
        else if (!strcmp(state, "GET"))
        {
            char resp[48];
            snprintf(resp, sizeof(resp),
                     "@DRYRUN:%s:GAP:%u:RETRY:%u#",
                     ModelHandle_GetDryRunEnable() ? "ON" : "OFF",
                     ModelHandle_GetGapTime(),
                     ModelHandle_GetMaxRunTime());
            ack(resp);
        }
        else err("@FORMAT#");
        return;
    }
    if (!strcmp(cmd, "MANUAL"))
    {
        char *state = next_token(&ctx);
        if (!state) { err("@FORMAT#"); return; }
        if (!strcmp(state, "ON"))
        {
            if (!ModelHandle_IsManualActive()) ModelHandle_ToggleManual();
            if (!Motor_GetStatus()) ModelHandle_ManualToggleMotor();
            ack("@MANUAL_ON#");
        }
        else if (!strcmp(state, "OFF"))
        {
            if (ModelHandle_IsManualActive()) ModelHandle_ToggleManual();
            ack("@MANUAL_OFF#");
        }
        else if (!strcmp(state, "MOTOR_ON"))
        {
            if (!Motor_GetStatus()) ModelHandle_ManualToggleMotor();
            ack("@MANUAL_MOTOR_ON#");
        }
        else if (!strcmp(state, "MOTOR_OFF"))
        {
            if (Motor_GetStatus()) ModelHandle_ManualToggleMotor();
            ack("@MANUAL_MOTOR_OFF#");
        }
        return;
    }
    if (!strcmp(cmd, "AUTO"))
    {
        char *state = next_token(&ctx);
        if (!state) { err("@FORMAT#"); return; }
        if (!strcmp(state, "ON"))
        {
            ModelHandle_ClearMaxRunFlag();
            ModelHandle_StartAuto(ModelHandle_GetGapTime(),
                                  ModelHandle_GetMaxRunTime(),
                                  ModelHandle_GetRetryCount());
            ack("@AUTO_ON#");
        }
        else if (!strcmp(state, "OFF"))
            { ModelHandle_StopAuto(); ack("@AUTO_OFF#"); }
        return;
    }
    if (!strcmp(cmd, "TIMER"))
    {
        char *sub = next_token(&ctx);
        if (!sub) { err("@FORMAT#"); return; }
        if (!strcmp(sub, "SET"))
        {
            char *slot_s = next_token(&ctx);
            char *days   = next_token(&ctx);
            char *onH_s  = next_token(&ctx);
            char *onM_s  = next_token(&ctx);
            char *offH_s = next_token(&ctx);
            char *offM_s = next_token(&ctx);
            char *en_s   = next_token(&ctx);
            if (!slot_s || !days || !onH_s || !onM_s || !offH_s || !offM_s || !en_s)
                { err("@FORMAT#"); return; }
            int slotNum = atoi(slot_s);
            if (slotNum < 1 || slotNum > 5) { err("@SLOT_ERR#"); return; }
            uint8_t s = slotNum - 1;
            timerSlots[s].onHour    = (uint8_t)atoi(onH_s);
            timerSlots[s].onMinute  = (uint8_t)atoi(onM_s);
            timerSlots[s].offHour   = (uint8_t)atoi(offH_s);
            timerSlots[s].offMinute = (uint8_t)atoi(offM_s);
            timerSlots[s].enabled   = (uint8_t)atoi(en_s);
            uint8_t dayMask = 0;
            char daybuf[32];
            strncpy(daybuf, days, sizeof(daybuf) - 1);
            daybuf[sizeof(daybuf)-1] = '\0';
            for (int i = 0; daybuf[i]; i++)
                if (daybuf[i] >= 'A' && daybuf[i] <= 'Z') daybuf[i] += 32;
            if (strstr(daybuf, "mon")) dayMask |= (1 << 0);
            if (strstr(daybuf, "tue")) dayMask |= (1 << 1);
            if (strstr(daybuf, "wed")) dayMask |= (1 << 2);
            if (strstr(daybuf, "thu")) dayMask |= (1 << 3);
            if (strstr(daybuf, "fri")) dayMask |= (1 << 4);
            if (strstr(daybuf, "sat")) dayMask |= (1 << 5);
            if (strstr(daybuf, "sun")) dayMask |= (1 << 6);
            timerSlots[s].dayMask = dayMask;
            ModelHandle_SaveTimerToEEPROM();
            send_timer_info();
            ack("@TIMER_SET_OK#");
            return;
        }
        if (!strcmp(sub, "ON"))
        {
            timerActive = true;
            ModelHandle_StartTimerNearestSlot();
            send_timer_info();
            ack("@TIMER_ON#");
            return;
        }
        if (!strcmp(sub, "OFF"))
        {
            ModelHandle_StopTimer();
            ack("@TIMER_OFF#");
            return;
        }
        if (!strcmp(sub, "GET"))
        {
            send_timer_info();
            return;
        }
        err("@FORMAT#");
        return;
    }
    if (!strcmp(cmd, "SEMIAUTO"))
    {
        char *state = next_token(&ctx);
        if (!state) { err("@FORMAT#"); return; }
        if (!strcmp(state, "ON"))
            { ModelHandle_StartSemiAuto(); ack("@SEMIAUTO_ON#"); }
        else if (!strcmp(state, "OFF"))
            { ModelHandle_StopSemiAuto(); ack("@SEMIAUTO_OFF#"); }
        return;
    }
    if (!strcmp(cmd, "COUNTDOWN"))
    {
        char *sub = next_token(&ctx);
        if (!sub) { err("@FORMAT#"); return; }
        if (!strcmp(sub, "ON") || atoi(sub) > 0)
        {
            const char *dur_s = !strcmp(sub, "ON") ? next_token(&ctx) : sub;
            uint32_t seconds = countdown_arg_to_seconds(dur_s);
            /* The app's duration also becomes the device button's duration */
            ModelHandle_SetCountdownDefaultMin((uint16_t)((seconds + 59UL) / 60UL));
            ModelHandle_StartCountdown(seconds);
            ack("@COUNTDOWN_ON#");
        }
        else if (!strcmp(sub, "SET"))
        {
            char *min_s = next_token(&ctx);
            if (!min_s) { err("@FORMAT#"); return; }
            ModelHandle_SetCountdownDefaultMin((uint16_t)(countdown_arg_to_seconds(min_s) / 60UL));
            ack("@COUNTDOWN_SET_OK#");
        }
        else if (!strcmp(sub, "GET"))
        {
            char resp[48];
            snprintf(resp, sizeof(resp), "@COUNTDOWN_DATA:%u:%lu:%u#",
                     (unsigned)ModelHandle_GetCountdownDefaultMin(),
                     countdownActive ? (unsigned long)countdownDuration : 0UL,
                     countdownActive ? 1u : 0u);
            ack(resp);
        }
        else if (!strcmp(sub, "OFF"))
            { ModelHandle_StopCountdown(); ack("@COUNTDOWN_OFF#"); }
        return;
    }
    if (!strcmp(cmd, "TWIST"))
    {
        char *state = next_token(&ctx);
        if (!state) { err("@FORMAT#"); return; }
        if (!strcmp(state, "ON"))
            { ModelHandle_ResumeTwist(); ack("@TWIST_ON#"); send_twist_info(); }
        else if (!strcmp(state, "GET"))
            { send_twist_info(); }
        else if (!strcmp(state, "SET"))
        {
            char *on_s  = next_token(&ctx);
            char *off_s = next_token(&ctx);
            char *onH   = next_token(&ctx);
            char *onM   = next_token(&ctx);
            char *offH  = next_token(&ctx);
            char *offM  = next_token(&ctx);
            if (on_s && off_s)
                ModelHandle_StartTwist(
                    (uint16_t)atoi(on_s), (uint16_t)atoi(off_s),
                    onH  ? (uint8_t)atoi(onH)  : 0,
                    onM  ? (uint8_t)atoi(onM)  : 0,
                    offH ? (uint8_t)atoi(offH) : 0,
                    offM ? (uint8_t)atoi(offM) : 0);
            else { err("@FORMAT#"); return; }
            ack("@TWIST_SET_OK#");
            send_twist_info();
        }
        else if (!strcmp(state, "OFF"))
            { ModelHandle_StopTwist(); ack("@TWIST_OFF#"); }
        return;
    }
    if (!strcmp(cmd, "RESTART"))
    {
        char *state = next_token(&ctx);
        if (!state) { err("@FORMAT#"); return; }
        if (!strcmp(state, "ON"))
            { ModelHandle_StartRestart(); ack("@RESTART_ON#"); }
        else if (!strcmp(state, "OFF"))
            { ModelHandle_StopRestart(); ack("@RESTART_OFF#"); }
        return;
    }
    g_screenUpdatePending = true;
}
