#include "uart_commands.h"
#include "uart.h"
#include "model_handle.h"
#include "relay.h"
#include "rtc_i2c.h"
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
    else if (countdownActive)               mode = "COUNTDOWN";
    else if (twistActive)                   mode = "TWIST";
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
    uint32_t dry_time   = 60;
    uint8_t  dry_en     = ModelHandle_GetDryRunEnable() ? 1 : 0;
    uint8_t  buzz_pump  = 1;
    uint8_t  buzz_full  = 1;
    uint8_t  buzz_empty = 1;
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
    if (!strcmp(cmd, "SET") || !strcmp(cmd, "SETTINGS"))
        { parse_settings(ctx); return; }
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
            { ModelHandle_ManualToggleMotor(); ack("@MANUAL_MOTOR_ON#"); }
        else if (!strcmp(state, "MOTOR_OFF"))
            { ModelHandle_ManualToggleMotor(); ack("@MANUAL_MOTOR_OFF#"); }
        return;
    }
    if (!strcmp(cmd, "AUTO"))
    {
        char *state = next_token(&ctx);
        if (!state) { err("@FORMAT#"); return; }
        if (!strcmp(state, "ON"))
        {
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
            uint32_t seconds = (uint32_t)atoi(sub);
            if (!strcmp(sub, "ON"))
            {
                char *dur_s = next_token(&ctx);
                seconds = dur_s ? (uint32_t)atoi(dur_s) : 600;
            }
            ModelHandle_StartCountdown(seconds);
            ack("@COUNTDOWN_ON#");
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
            { ModelHandle_StartTwist(5, 5, 0, 0, 0, 0); ack("@TWIST_ON#"); }
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
            ack("@TWIST_SET_OK#");
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
