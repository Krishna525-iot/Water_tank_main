#include "model_handle.h"
#include "relay.h"
#include "led.h"
#include "global.h"
#include "adc.h"
#include "rtc_i2c.h"
#include "uart_commands.h"
#include "stm32f1xx_hal.h"
#include "eeprom_i2c.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c2;
extern ADC_Data adcData;
extern RTC_Time_t time;
extern float g_currentA;
extern float g_voltageV;

#define TIMER_EE_SIGNATURE  0x544D
#define TIMER_EE_VERSION    1
#define EE_ADDR_TIMER_BLOCK 0x0600

typedef enum
{
    BUZZ_NONE = 0,
    BUZZ_TANK_FULL,
    BUZZ_TANK_EMPTY
} BuzzerEvent;

static BuzzerEvent activeBuzzEvent = BUZZ_NONE;

typedef struct {
    uint16_t signature;
    uint8_t  version;
    uint8_t  reserved;
    TimerSlot slots[5];
    uint16_t crc;
} TimerEEPROMBlock;

static uint16_t Timer_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

#define MOTOR_START_DELAY_MS   10000UL
#define TANK_FULL_DELAY_MS     10000UL

static uint32_t bootStartBlockUntil = 0;
static uint32_t tankFullDetectedAt  = 0;

TimerSlot timerSlots[5];

static void Buzzer_TankEmptyPattern(void);
static void Buzzer_TankFullPattern(void);

volatile bool manualActive    = false;
volatile bool semiAutoActive  = false;
volatile bool countdownActive = false;
volatile bool twistActive     = false;
volatile bool timerActive     = false;
volatile bool autoActive      = false;

static uint32_t dryDeadline = 0;

#define LOAD_FAULT_CONFIRM_MS 3000UL
#define LOAD_RETRY_RUN_MS     3000UL
#define LOAD_LOCK_DEFAULT_MS (20UL * 60UL * 1000UL)

static uint32_t motorOnStartMs = 0;
static uint32_t powerOnMs      = 0;
static DryFSMState dryState    = DRY_IDLE;

volatile uint8_t motorStatus = 0;
volatile bool senseDryRun         = false;
volatile bool groundWater         = false;
volatile bool senseOverLoad       = false;
volatile bool senseUnderLoad      = false;
volatile bool senseOverUnderVolt  = false;
volatile bool senseMaxRunReached  = false;
volatile bool manualOverride      = false;
volatile uint16_t auto_retry_counter = 0;
volatile bool     countdownMode      = false;
volatile uint32_t countdownDuration  = 0;
#define DEFAULT_DRY_GAP_S        120U   /* motor run/test time: 2 min */
#define DEFAULT_DRY_RETRY_S      300U   /* retry wait time: 5 min */
#define DEFAULT_AUTO_MAXRUN_MIN  300U
TwistSettings twistSettings;

typedef struct {
    bool    manual_on;
    bool    semi_on;
    bool    timer_on;
    bool    countdown_on;
    bool    twist_on;
    bool    auto_on;
    bool    motor_on;
    uint8_t power_restore_mode;
} ModeState;

static ModeState modeState;

#define EE_ADDR_COUNTDOWN_BLOCK 0x0040
#define EE_ADDR_MODE_BLOCK      0x0080
#define EE_ADDR_AUTO_BLOCK      0x00C0
#define CD_SIGNATURE            0xCD55

void ModelHandle_CheckDryRun(void);
void ModelHandle_CheckLoadFault(void);

typedef enum
{
    TIMER_RUN_TEST = 0,
    TIMER_WAIT_RETRY,
    TIMER_RUN_CONTINUOUS
} TimerState;

static TimerState timerState         = TIMER_RUN_TEST;
static uint32_t   timerStateDeadline = 0;

typedef struct {
    uint16_t sig;
    uint32_t remaining;
    uint8_t  active;
    uint16_t crc;
} CountdownBlock;

#define SEMI_TANK_FULL_DELAY_MS 10000UL

static uint8_t powerRestoreMode = 0;

SystemSettings sys = {
    .gap_time_s      = 0,
    .retry_count     = 0,
    .uv_limit        = 190,
    .ov_limit        = 270,
    .overload        = 0.0f,
    .underload       = 0.0f,
    .maxrun_min      = 300,
    .dry_run_time_s  = 300,
    .dry_run_enable  = 0
};

static inline void start_motor(void);
static inline void stop_motor(void);
static inline void clear_all_modes(void);
void ModelHandle_SoftDryRunHandler(void);
static void SaveCountdown(void);
static bool     timer_any_active_slot(void);
static uint16_t get_active_timer_gap_minutes(void);
static uint8_t  get_today_mask(void);
static bool     slot_is_active_now(const TimerSlot *t);
static bool     isTankFull(void);
static inline bool isAnyModeActive(void);
static void     auto_tick(void);
static void     twist_tick(void);
static void     leds_from_model(void);

typedef enum {
    MOTOR_OWNER_NONE = 0,
    MOTOR_OWNER_MANUAL,
    MOTOR_OWNER_SEMIAUTO,
    MOTOR_OWNER_TIMER,
    MOTOR_OWNER_COUNTDOWN,
    MOTOR_OWNER_TWIST,
    MOTOR_OWNER_AUTO,
    MOTOR_OWNER_RESTART
} MotorOwner;

static volatile MotorOwner motorOwner = MOTOR_OWNER_NONE;
static MotorOwner prevMotorOwner = MOTOR_OWNER_NONE;
#define BUZZ_SIG          0xB155
#define EE_ADDR_BUZZER_BLOCK 0x0500

typedef struct __attribute__((packed))
{
    uint16_t sig;
    uint8_t  pumpOnSound;
    uint8_t  tankFullSound;
    uint8_t  tankEmptySound;
    uint16_t crc;
} BuzzerEEPROMBlock;

typedef struct {
    uint8_t pumpOnSound;
    uint8_t tankFullSound;
    uint8_t tankEmptySound;
} BuzzerSettings;

static BuzzerSettings buzzerSettings = {1, 1, 1};

static uint32_t cd_deadline = 0;

typedef enum {
    LOAD_NORMAL = 0,
    LOAD_FAULT_WAIT,
    LOAD_FAULT_LOCK,
    LOAD_RETRY_RUN
} LoadFaultState;
#define COUNTDOWN_GAP_MS   15000UL
static LoadFaultState loadState = LOAD_NORMAL;
static MotorOwner previousOwner   = MOTOR_OWNER_NONE;
static bool previousManual    = false;
static bool previousSemi      = false;
static bool previousCountdown = false;
static bool previousAuto      = false;
static uint16_t auto_gap_s       = 120;
static uint16_t auto_maxrun_min  = 12;
static uint8_t  auto_retry_limit = 5;
static uint8_t  auto_retry_count = 0;

typedef enum {
    AUTO_IDLE = 0,
    AUTO_ON_WAIT,
    AUTO_DRY_CHECK,
    AUTO_OFF_WAIT
} AutoState;

static bool     modeSwitchPending = false;
static uint32_t modeSwitchTime    = 0;
static bool     restartActive     = false;
static MotorOwner pendingOwner    = MOTOR_OWNER_NONE;
static AutoState  autoState       = AUTO_IDLE;
static uint32_t   autoDeadline    = 0;
#define AUTO_SIG 0xA055
static bool     twist_on_phase = false;
static uint32_t bootBlockUntil = 0;
static uint32_t twist_deadline = 0;
static bool semiTankFullHold = false;
static bool cdTankFullHold   = false;
static bool autoBackgroundEnabled = true;
static bool autoUserLocked        = false;

typedef struct __attribute__((packed))
{
    uint16_t sig;
    uint8_t  active;
    uint8_t  state;
    uint8_t  retryCount;
    uint32_t remaining_ms;
    uint8_t  crc;
} AutoRuntimeBlock;

static bool     suppressAutoOneCycle  = false;
static uint32_t autoBootIgnoreUntil   = 0;

#define AUTO_START_LEVEL_PERCENT   50
#define AUTO_STOP_LEVEL_PERCENT    100

static uint32_t loadTimer      = 0;
static uint8_t  loadRetryCount = 0;

#ifndef EEPROM_PAGE_SIZE
#define EEPROM_PAGE_SIZE 32
#endif
#ifndef EEPROM_I2C_ADDR
#define EEPROM_I2C_ADDR (0x50 << 1)
#endif
#ifndef EEPROM_ADDR_SIZE
#define EEPROM_ADDR_SIZE I2C_MEMADD_SIZE_16BIT
#endif

typedef enum {
    SEMI_RUN_TEST = 0,
    SEMI_WAIT_GAP,
    SEMI_RUN_CONTINUOUS
} SemiState;

static SemiState semiState           = SEMI_RUN_TEST;
static uint32_t  semiDeadline        = 0;
static bool      semiWaterLossPending = false;

typedef struct __attribute__((packed))
{
    uint16_t sig;
    uint32_t gap;
    uint16_t dry_time;
    uint8_t  retry;
    uint16_t uv;
    uint16_t ov;
    uint16_t maxrun;
    uint16_t over10;
    uint16_t under10;
    uint8_t  dry_enable;
    uint16_t crc;
} SystemEEPROMBlock;

#define PROBE_THRESHOLD       0.50f
#define SENSOR_STABLE_TIME_MS 5000UL
#define EE_ADDR_SYS_BLOCK     0x0000
#define SYS_SIG               0x5A5B

typedef enum {
    RESTART_RUN_TEST = 0,
    RESTART_WAIT_GAP,
    RESTART_RUN_CONTINUOUS
} RestartState;

static RestartState restartState   = RESTART_RUN_TEST;
static uint32_t     restartDeadline = 0;
static uint32_t     stateDeadline   = 0;
static void ensure_dry_run_defaults_if_enabled(void)
{
    bool changed = false;

    /*
     * If dry-run is enabled but values are not configured,
     * use safe default values automatically.
     */
    if (sys.dry_run_enable != 0)
    {
        if (sys.gap_time_s == 0)
        {
            sys.gap_time_s = DEFAULT_DRY_GAP_S;
            changed = true;
        }

        if (sys.dry_run_time_s == 0)
        {
            sys.dry_run_time_s = DEFAULT_DRY_RETRY_S;
            changed = true;
        }

        if (sys.maxrun_min == 0)
        {
            sys.maxrun_min = DEFAULT_AUTO_MAXRUN_MIN;
            changed = true;
        }
    }

    if (changed)
    {
        ModelHandle_SaveSettingsToEEPROM();
    }
}
static inline bool dry_protection_enabled(void)
{
    if (sys.dry_run_enable == 0)
        return false;

    if (sys.gap_time_s == 0)
        sys.gap_time_s = DEFAULT_DRY_GAP_S;

    if (sys.dry_run_time_s == 0)
        sys.dry_run_time_s = DEFAULT_DRY_RETRY_S;

    return true;
}
uint16_t ModelHandle_GetGapTime(void)        { return sys.gap_time_s; }
uint8_t  ModelHandle_GetRetryCount(void)     { return sys.retry_count; }
uint16_t ModelHandle_GetUnderVolt(void)      { return sys.uv_limit; }
uint16_t ModelHandle_GetOverVolt(void)       { return sys.ov_limit; }
float    ModelHandle_GetOverloadLimit(void)  { return sys.overload; }
float    ModelHandle_GetUnderloadLimit(void) { return sys.underload; }
uint16_t ModelHandle_GetMaxRunTime(void)     { return sys.maxrun_min; }
uint16_t ModelHandle_GetDryRunRetryGap(void) { return sys.dry_run_time_s; }

void EEPROM_WriteBlockSafe(uint16_t addr, uint8_t *data, uint16_t len)
{
    while (len)
    {
        uint16_t page_space = EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE);
        uint16_t chunk = (len < page_space) ? len : page_space;
        HAL_I2C_Mem_Write(&hi2c2, EEPROM_I2C_ADDR, addr,
                          EEPROM_ADDR_SIZE, data, chunk, 1000);
        HAL_Delay(6);
        addr += chunk;
        data += chunk;
        len  -= chunk;
    }
}

static uint16_t SYS_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--)
    {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}
#define Buzzer_CRC16  SYS_CRC16

void ModelHandle_SaveSettingsToEEPROM(void)
{
    SystemEEPROMBlock b;
    memset(&b, 0, sizeof(b));
    b.sig        = SYS_SIG;
    b.gap        = sys.gap_time_s;
    b.dry_time   = sys.dry_run_time_s;
    b.retry      = sys.retry_count;
    b.uv         = sys.uv_limit;
    b.ov         = sys.ov_limit;
    b.maxrun     = sys.maxrun_min;
    b.over10     = (uint16_t)(sys.overload  * 10.0f);
    b.under10    = (uint16_t)(sys.underload * 10.0f);
    b.dry_enable = sys.dry_run_enable;
    b.crc        = SYS_CRC16((uint8_t*)&b, sizeof(b) - 2);
    EEPROM_WriteBlockSafe(EE_ADDR_SYS_BLOCK, (uint8_t*)&b, sizeof(b));
}

void ModelHandle_LoadSettingsFromEEPROM(void)
{
    SystemEEPROMBlock b;
    EEPROM_ReadBuffer(EE_ADDR_SYS_BLOCK, (uint8_t*)&b, sizeof(b));

    uint16_t crc = SYS_CRC16((uint8_t*)&b, sizeof(b) - 2);

    if (b.sig != SYS_SIG || crc != b.crc)
    {
        ModelHandle_SaveSettingsToEEPROM();
        return;
    }

    sys.gap_time_s     = b.gap;
    sys.dry_run_time_s = b.dry_time;
    sys.retry_count    = b.retry;
    sys.uv_limit       = b.uv;
    sys.ov_limit       = b.ov;
    sys.maxrun_min     = b.maxrun;
    sys.overload       = b.over10  / 10.0f;
    sys.underload      = b.under10 / 10.0f;
    sys.dry_run_enable = b.dry_enable;

    /*
     * Safety correction:
     * If dry run is enabled but old EEPROM has zero values,
     * restore defaults automatically.
     */
    ensure_dry_run_defaults_if_enabled();
}

void ModelHandle_SaveModeState(void)
{
    modeState.manual_on          = manualActive;
    modeState.semi_on            = semiAutoActive;
    modeState.timer_on           = timerActive;
    modeState.countdown_on       = countdownActive;
    modeState.twist_on           = twistActive;
    modeState.auto_on            = autoActive;
    modeState.motor_on           = (motorStatus == 1);
    modeState.power_restore_mode = powerRestoreMode;
    EEPROM_WriteBuffer(0x0200, (uint8_t*)&modeState, sizeof(modeState));
}

static void backup_current_mode(void)
{
    previousOwner     = motorOwner;
    previousManual    = manualActive;
    previousSemi      = semiAutoActive;
    previousCountdown = countdownActive;
    previousAuto      = autoActive;
}

static void restore_previous_mode(void)
{
    manualActive    = previousManual;
    semiAutoActive  = previousSemi;
    countdownActive = previousCountdown;
    autoActive      = previousAuto;

    if (previousManual)
        motorOwner = MOTOR_OWNER_MANUAL;
    else if (previousSemi)
        motorOwner = MOTOR_OWNER_SEMIAUTO;
    else if (previousCountdown)
        motorOwner = MOTOR_OWNER_COUNTDOWN;
    else if (previousAuto)
        motorOwner = MOTOR_OWNER_AUTO;
    else
    {
        motorOwner = MOTOR_OWNER_NONE;
        suppressAutoOneCycle = true;
        return;
    }

    if (!isTankFull() && isAnyModeActive())
        start_motor();
    else
        stop_motor();
}

void ModelHandle_StopRestart(void)
{
    if (!restartActive) return;
    restartActive = false;
    stop_motor();
    restore_previous_mode();
    ModelHandle_SaveModeState();
}

void ModelHandle_LoadModeState(void)
{
    EEPROM_ReadBuffer(0x0200, (uint8_t*)&modeState, sizeof(modeState));
    powerRestoreMode = modeState.power_restore_mode;
    if (powerRestoreMode > 2) powerRestoreMode = 0;

    if (powerRestoreMode == 1)
    {
        manualActive = semiAutoActive = timerActive =
        countdownActive = twistActive = autoActive = false;
        return;
    }
//    manualActive    = modeState.manual_on;
    semiAutoActive  = modeState.semi_on;
    timerActive     = modeState.timer_on;
    countdownActive = modeState.countdown_on;
    twistActive     = modeState.twist_on;

    if (modeState.auto_on)
    {
        uint8_t level = ModelHandle_GetTankLevelPercent();
        autoActive = (level <= AUTO_START_LEVEL_PERCENT);
    }
    else
    {
        autoActive = false;
    }
    if (timerActive && !autoActive)
        timerActive = false;
}

void ModelHandle_SaveTimerToEEPROM(void)
{
    TimerEEPROMBlock blk;
    memset(&blk, 0, sizeof(blk));
    blk.signature = TIMER_EE_SIGNATURE;
    blk.version   = TIMER_EE_VERSION;
    memcpy(blk.slots, timerSlots, sizeof(timerSlots));
    blk.crc = Timer_CRC16((uint8_t*)&blk, sizeof(blk) - sizeof(uint16_t));
    EEPROM_WriteBuffer(EE_ADDR_TIMER_BLOCK, (uint8_t*)&blk, sizeof(blk));
}

void ModelHandle_LoadTimerFromEEPROM(void)
{
    TimerEEPROMBlock blk;
    EEPROM_ReadBuffer(EE_ADDR_TIMER_BLOCK, (uint8_t*)&blk, sizeof(blk));
    if (blk.signature != TIMER_EE_SIGNATURE) return;
    uint16_t crc = Timer_CRC16((uint8_t*)&blk, sizeof(blk) - 2);
    if (blk.crc != crc) return;
    memcpy(timerSlots, blk.slots, sizeof(timerSlots));
}

void Timer_EEPROM_EnsureValid(void)
{
    TimerEEPROMBlock blk;
    EEPROM_ReadBuffer(EE_ADDR_TIMER_BLOCK, (uint8_t*)&blk, sizeof(blk));
    uint16_t crc = Timer_CRC16((uint8_t*)&blk, sizeof(blk) - 2);
    if (blk.signature != TIMER_EE_SIGNATURE || blk.crc != crc)
    {
        memset(timerSlots, 0, sizeof(timerSlots));
        ModelHandle_SaveTimerToEEPROM();
    }
}

void ModelHandle_SendTimerInfoUART(void)
{
    char buf[128];
    for (int i = 0; i < 5; i++)
    {
        const TimerSlot *t = &timerSlots[i];
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

static uint8_t read_raw_tank_level(void)
{
    uint8_t level = 0;
    if (adcData.voltages[3] < PROBE_THRESHOLD) level = 25;
    if (adcData.voltages[2] < PROBE_THRESHOLD) level = 50;
    if (adcData.voltages[1] < PROBE_THRESHOLD) level = 75;
    if (adcData.voltages[0] < PROBE_THRESHOLD) level = 100;
    return level;
}

static uint8_t get_tank_level_percent(void)
{
    static uint8_t  stableLevel    = 0;
    static uint8_t  candidateLevel = 0;
    static uint32_t changeTime     = 0;

    uint32_t now      = HAL_GetTick();
    uint8_t  rawLevel = read_raw_tank_level();

    if (rawLevel != stableLevel)
    {
        if (rawLevel != candidateLevel)
        {
            candidateLevel = rawLevel;
            changeTime     = now;
        }
        if ((now - changeTime) >= SENSOR_STABLE_TIME_MS)
            stableLevel = candidateLevel;
    }
    return stableLevel;
}

static bool isTankFull(void)
{
    return (get_tank_level_percent() == 100);
}

void ModelHandle_StartRestart(void)
{
    if (isTankFull()) return;

    backup_current_mode();
    clear_all_modes();

    restartActive       = true;
    restartState        = RESTART_RUN_CONTINUOUS;
    restartDeadline     = 0;
    motorOwner          = MOTOR_OWNER_RESTART;
    senseMaxRunReached  = false;
    dryState            = DRY_IDLE;
    bootStartBlockUntil = HAL_GetTick();

    start_motor();
    ModelHandle_SaveModeState();
}

void ModelHandle_TimerRecalculateNow(void)
{
    if (!timerActive) return;
    ModelHandle_ProcessTimerSlots();
}

void ModelHandle_ProcessDryRun(void)
{
    ModelHandle_SoftDryRunHandler();
}

uint8_t ModelHandle_GetPowerRestoreMode(void)    { return powerRestoreMode; }

void ModelHandle_SetPowerRestoreMode(uint8_t mode)
{
    if (mode > 2) mode = 0;
    powerRestoreMode = mode;
    ModelHandle_SaveModeState();
}

void ModelHandle_SetDryRunTime(uint32_t seconds)
{
    if (seconds < 10)   seconds = 10;
    if (seconds > 3600) seconds = 3600;
    sys.dry_run_time_s = seconds;
    ModelHandle_SaveSettingsToEEPROM();
}

void ModelHandle_ToggleManual(void)
{
    if (!manualActive)
    {
        restartActive   = false;
        timerActive     = false;
        semiAutoActive  = false;
        countdownActive = false;
        countdownMode   = false;
        twistActive     = false;

        if (autoActive)
        {
            autoActive = false;
            autoUserLocked = true;
        }

        manualActive       = true;
        manualOverride     = true;
        motorOwner         = MOTOR_OWNER_MANUAL;
        dryState           = DRY_IDLE;
        senseMaxRunReached = false;

        start_motor();
    }
    else
    {
        manualActive   = false;
        manualOverride = false;
        motorOwner     = MOTOR_OWNER_NONE;
        dryState       = DRY_IDLE;

        stop_motor();
    }

    ModelHandle_SaveModeState();
}

void ModelHandle_ManualToggleMotor(void)
{
    if (!manualActive) return;
    if (Motor_GetStatus()) stop_motor();
    else                   start_motor();
}

void ModelHandle_SetDryRunAndRetry(uint32_t retry_gap_sec, uint16_t dry_time_sec)
{
    if (retry_gap_sec < 10) retry_gap_sec = 10;
    if (dry_time_sec  < 10) dry_time_sec  = 10;
    sys.gap_time_s     = retry_gap_sec;
    sys.dry_run_time_s = dry_time_sec;
    ModelHandle_SaveSettingsToEEPROM();
}

void ModelHandle_Button3_SinglePress(void)
{
    if (!autoActive) return;

    if (!timerActive)
    {
        timerActive        = true;
        motorOwner         = MOTOR_OWNER_TIMER;
        timerState         = TIMER_RUN_TEST;
        timerStateDeadline = 0;
        dryState           = DRY_IDLE;
        ModelHandle_ProcessTimerSlots();
        ModelHandle_SendTimerInfoUART();
    }
    else
    {
        timerActive = false;
        motorOwner  = MOTOR_OWNER_AUTO;
        dryState    = DRY_IDLE;
        stop_motor();
    }
    ModelHandle_SaveModeState();
}

void ModelHandle_StopAllModesAndMotor(void)
{
    clear_all_modes();
    stop_motor();
    dryState = DRY_IDLE;
    ModelHandle_SaveModeState();
}

void ModelHandle_FactoryReset(void)
{
    sys.gap_time_s     = 0;
    sys.retry_count    = 0;
    sys.uv_limit       = 190;
    sys.ov_limit       = 270;
    sys.overload       = 0.0f;
    sys.underload      = 0.0f;
    sys.maxrun_min     = 300;
    sys.dry_run_time_s = 300;
    sys.dry_run_enable = 0;
    ModelHandle_SaveSettingsToEEPROM();
    powerRestoreMode = 0;
    ModelHandle_SaveModeState();
    buzzerSettings.pumpOnSound    = 1;
    buzzerSettings.tankFullSound  = 1;
    buzzerSettings.tankEmptySound = 1;
    ModelHandle_SaveBuzzerSettings();
}

void ModelHandle_StartTimerNearestSlot(void)
{
    if (!autoActive) return;
    timerActive        = true;
    motorOwner         = MOTOR_OWNER_TIMER;
    timerState         = TIMER_RUN_TEST;
    timerStateDeadline = 0;
    ModelHandle_ProcessTimerSlots();
    ModelHandle_SendTimerInfoUART();
}

static uint32_t buzzerPatternStart = 0;
static bool     buzzerState        = false;
static inline void Buzzer_SetPin(bool on)
{
    HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Buzzer_StartEvent(BuzzerEvent ev)
{
    if (activeBuzzEvent == ev) return;
    activeBuzzEvent    = ev;
    buzzerPatternStart = HAL_GetTick();
    buzzerState        = false;
}

#define TANK_FULL_BUZZ_MS 5000UL

static void Buzzer_TankEmptyPattern(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - buzzerPatternStart) >= 15000UL) { Buzzer_SetPin(false); activeBuzzEvent = BUZZ_NONE; return; }
    Buzzer_SetPin(true);
}

static void Buzzer_TankFullPattern(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - buzzerPatternStart) >= 15000UL) { Buzzer_SetPin(false); activeBuzzEvent = BUZZ_NONE; return; }
    Buzzer_SetPin(true);
}

static void Buzzer_Update(void)
{
    uint32_t now = HAL_GetTick();
    static uint32_t motorToggleTime          = 0;
    static bool     motorBuzzState           = false;
    static bool     manualTankFullBuzzActive = false;
    static uint32_t manualTankFullBuzzStart  = 0;
    static bool     prevTankFullManual       = false;
    bool motorOn  = Motor_GetStatus();
    bool tankFull = isTankFull();

    if (manualActive)
    {
        if (!motorOn)
        {
            Buzzer_SetPin(false);
            motorBuzzState           = false;
            motorToggleTime          = 0;
            manualTankFullBuzzActive = false;
            prevTankFullManual       = false;
            return;
        }
        if (tankFull && !prevTankFullManual)
        {
            manualTankFullBuzzActive = true;
            manualTankFullBuzzStart  = now;
            motorBuzzState           = false;
            motorToggleTime          = 0;
        }
        prevTankFullManual = tankFull;
        if (!tankFull && manualTankFullBuzzActive) manualTankFullBuzzActive = false;
        if (manualTankFullBuzzActive)
        {
            if ((now - manualTankFullBuzzStart) < TANK_FULL_BUZZ_MS) { Buzzer_SetPin(true); return; }
            else { manualTankFullBuzzActive = false; motorToggleTime = 0; motorBuzzState = false; }
        }
        if (buzzerSettings.pumpOnSound)
        {
            if ((now - motorToggleTime) >= 1000UL) { motorToggleTime = now; motorBuzzState = !motorBuzzState; }
            Buzzer_SetPin(motorBuzzState);
        }
        else Buzzer_SetPin(false);
        return;
    }
    manualTankFullBuzzActive = false;
    prevTankFullManual       = false;
    if (activeBuzzEvent == BUZZ_TANK_EMPTY) { Buzzer_TankEmptyPattern(); return; }
    if (activeBuzzEvent == BUZZ_TANK_FULL)  { Buzzer_TankFullPattern();  return; }

    if (motorOn)
    {
        if (buzzerSettings.pumpOnSound)
        {
            if ((now - motorToggleTime) >= 1000UL) { motorToggleTime = now; motorBuzzState = !motorBuzzState; }
            Buzzer_SetPin(motorBuzzState);
        }
        else Buzzer_SetPin(false);
        return;
    }
    Buzzer_SetPin(false);
}

static inline uint32_t now_ms(void) { return HAL_GetTick(); }

void ModelHandle_SetProtectionSettings(uint32_t retry_gap_sec,
                                        uint16_t dry_time_sec,
                                        uint8_t  dry_enable)
{
    if (retry_gap_sec < 60)    retry_gap_sec = 60;
    if (retry_gap_sec > 10800) retry_gap_sec = 10800;
    if (dry_time_sec  < 60)   dry_time_sec  = 60;
    if (dry_time_sec  > 900)  dry_time_sec  = 900;
    sys.gap_time_s     = retry_gap_sec;
    sys.dry_run_time_s = dry_time_sec;
    sys.dry_run_enable = dry_enable ? 1 : 0;
    ModelHandle_SaveSettingsToEEPROM();
}

static inline void clear_all_modes(void)
{
    manualActive    = false;
    semiAutoActive  = false;
    countdownActive = false;
    twistActive     = false;
    timerActive     = false;
    autoActive      = false;
    manualOverride  = false;
    countdownMode   = false;
}

void ModelHandle_OnPowerUp(void)
{
    powerOnMs           = HAL_GetTick();
    autoBootIgnoreUntil = powerOnMs + 15000UL;
    bootStartBlockUntil = powerOnMs + MOTOR_START_DELAY_MS;
    autoDeadline        = 0;
    dryState            = DRY_IDLE;
    ModelHandle_LoadBuzzerSettings();

    if (autoActive && timer_any_active_slot())
    {
        timerActive        = true;
        timerState         = TIMER_RUN_TEST;
        timerStateDeadline = 0;
        motorOwner         = MOTOR_OWNER_TIMER;
    }
    if (autoActive)
    {
        uint8_t level = get_tank_level_percent();
        if (level <= AUTO_START_LEVEL_PERCENT)
        {
            motorOwner    = MOTOR_OWNER_AUTO;
            autoState     = AUTO_ON_WAIT;
            stateDeadline = 0;
            /* Motor will start once bootStartBlockUntil expires
             * via auto_tick → AUTO_ON_WAIT → start_motor()      */
        }
        else
        {
            autoActive = false;
            motorOwner = MOTOR_OWNER_NONE;
        }
    }
}

static inline bool Motor_IsRelayOn(void)
{
    return HAL_GPIO_ReadPin(Relay1_GPIO_Port, Relay1_Pin) == GPIO_PIN_SET;
}

static bool Motor_StartAllowed(void)
{
    return !(senseOverLoad || senseUnderLoad || senseOverUnderVolt || senseMaxRunReached);
}

bool ModelHandle_IsRestartActive(void) { return restartActive; }

static inline void motor_apply(bool on)
{
    if (timerActive && timerState == TIMER_WAIT_RETRY) on = false;
    bool relayNow = Motor_IsRelayOn();
    if (on)
    {
        if (!Motor_StartAllowed()) return;
        if (!relayNow)
        {
            Relay_Set(1, true);
            motorOnStartMs = HAL_GetTick();
        }
        motorStatus = 1;
    }
    else
    {
        if (relayNow) Relay_Set(1, false);
        motorStatus = 0;
    }
    UART_SendStatusPacket();
}

bool Motor_GetStatus(void) { return Motor_IsRelayOn(); }

static inline void start_motor(void)
{
    if (HAL_GetTick() < bootStartBlockUntil) return;
    if (motorOwner == MOTOR_OWNER_NONE) return;
    if (dryState == DRY_FAULT && motorOwner != MOTOR_OWNER_MANUAL) return;
    motor_apply(true);
}

static inline void stop_motor(void) { motor_apply(false); }

static void check_max_run(void)
{
    if (sys.maxrun_min == 0 || !Motor_GetStatus()) return;
    if (countdownActive || restartActive) return;
    uint32_t limit = (uint32_t)sys.maxrun_min * 60000UL;
    if ((HAL_GetTick() - motorOnStartMs) >= limit)
    {
        senseMaxRunReached = true;
        clear_all_modes();
        stop_motor();
        dryState = DRY_IDLE;
        ModelHandle_SaveModeState();
    }
}

void ModelHandle_Button4_SinglePress(void)
{
    if (!countdownActive)
    {
        clear_all_modes();
        countdownActive  = true;
        countdownMode    = true;
        motorOwner       = MOTOR_OWNER_COUNTDOWN;
        uint32_t defaultSeconds = 600;
        cd_deadline       = HAL_GetTick() + defaultSeconds * 1000UL;
        countdownDuration = defaultSeconds;
        cdTankFullHold    = false;
        dryState          = DRY_IDLE;
        start_motor();
    }
    else
    {
        ModelHandle_StopCountdown();
    }
    ModelHandle_SaveModeState();
}

void ModelHandle_CheckDryRun(void)
{
    float v = adcData.voltages[5];
    senseDryRun = (v < 0.30f);
}

#define GROUND_SENSE_DELAY_MS 10000UL

void ModelHandle_CheckGroundWater(void)
{
    static uint32_t detectStart = 0;
    static bool     stableState = false;

    float    v        = adcData.voltages[4];
    bool     detected = (v < 0.30f);
    uint32_t now      = HAL_GetTick();

    if (detected)
    {
        if (!stableState)
        {
            if (detectStart == 0) detectStart = now;
            if ((now - detectStart) >= GROUND_SENSE_DELAY_MS)
                stableState = true;
        }
    }
    else
    {
        detectStart = 0;
        stableState = false;
    }
    groundWater = stableState;
}

static inline bool isAnyModeActive(void)
{
    return (manualActive || semiAutoActive || countdownActive ||
            twistActive || timerActive || autoActive);
}

void ModelHandle_SoftDryRunHandler(void)
{
    if (!dry_protection_enabled()) { dryState = DRY_IDLE; return; }
    if (motorOwner == MOTOR_OWNER_TIMER || motorOwner == MOTOR_OWNER_AUTO) return;
    if (motorOwner == MOTOR_OWNER_MANUAL || motorOwner == MOTOR_OWNER_NONE)
        { dryState = DRY_IDLE; return; }
    uint32_t now   = HAL_GetTick();
    uint32_t gapMs = (uint32_t)sys.gap_time_s * 1000UL;
    ModelHandle_CheckDryRun();
    bool motorOn = Motor_GetStatus();

    if (motorOn)
    {
        switch (dryState)
        {
            case DRY_IDLE:
                if (!senseDryRun) { dryState = DRY_WAITING; dryDeadline = now + gapMs; }
                break;

            case DRY_WAITING:
                if ((int32_t)(now - dryDeadline) < 0) break;
                if (!senseDryRun)
                {
                    stop_motor();
                    dryState = DRY_FAULT;
                    if (buzzerSettings.tankEmptySound) Buzzer_StartEvent(BUZZ_TANK_EMPTY);

                    switch (motorOwner)
                    {
                        case MOTOR_OWNER_SEMIAUTO:
                            semiAutoActive = false; motorOwner = MOTOR_OWNER_NONE;
                            ModelHandle_SaveModeState(); break;
                        case MOTOR_OWNER_COUNTDOWN:
                            ModelHandle_StopCountdown(); break;
                        case MOTOR_OWNER_TWIST:
                            ModelHandle_StopTwist(); break;
                        case MOTOR_OWNER_RESTART:
                            restartActive = false; motorOwner = MOTOR_OWNER_NONE;
                            restore_previous_mode(); ModelHandle_SaveModeState(); break;
                        default: break;
                    }
                }
                else dryState = DRY_IDLE;
                break;
            case DRY_FAULT:
                stop_motor(); break;

            default:
                dryState = DRY_IDLE; break;
        }
    }
    else
    {
        switch (dryState)
        {
            case DRY_FAULT:    if (senseDryRun) dryState = DRY_IDLE; break;
            case DRY_WAITING:  dryState = DRY_IDLE; break;
            default:           dryState = DRY_IDLE; break;
        }
    }
}

static inline uint32_t get_load_lock_duration_ms(void)
{
    return (sys.retry_count == 0) ? LOAD_LOCK_DEFAULT_MS
                                  : (uint32_t)sys.retry_count * 60000UL;
}

void ModelHandle_CheckLoadFault(void)
{
    if (!Motor_GetStatus()) return;
    float I = g_currentA;
    float V = g_voltageV;
    bool overload  = (sys.overload  > 0.1f)   && (I > sys.overload);
    bool underload = (sys.underload > 0.001f) && (I < sys.underload);
    bool voltFault = ((sys.uv_limit && V < sys.uv_limit) ||
                      (sys.ov_limit && V > sys.ov_limit));
    senseOverLoad      = overload;
    senseUnderLoad     = underload;
    senseOverUnderVolt = voltFault;
    bool fault = overload || underload || voltFault;

    uint32_t now    = HAL_GetTick();
    uint32_t lockMs = get_load_lock_duration_ms();

    switch (loadState)
    {
        case LOAD_NORMAL:
            loadRetryCount = 0;
            if (fault) { loadState = LOAD_FAULT_WAIT; loadTimer = now; }
            break;
        case LOAD_FAULT_WAIT:
            if (!fault) { loadState = LOAD_NORMAL; }
            else if ((now - loadTimer) >= LOAD_FAULT_CONFIRM_MS)
            {
                stop_motor(); loadState = LOAD_FAULT_LOCK; loadTimer = now;
                if (buzzerSettings.tankFullSound) Buzzer_StartEvent(BUZZ_TANK_FULL);
            }
            break;
        case LOAD_FAULT_LOCK:
            if (!fault) { loadState = LOAD_NORMAL; }
            else if ((now - loadTimer) >= lockMs && loadRetryCount < 1)
            {
                loadRetryCount++;
                if (isAnyModeActive()) start_motor();
                loadState = LOAD_RETRY_RUN; loadTimer = now;
            }
            break;
        case LOAD_RETRY_RUN:
            if ((now - loadTimer) >= LOAD_RETRY_RUN_MS)
            {
                if (fault)
                {
                    stop_motor(); loadState = LOAD_FAULT_LOCK; loadTimer = now;
                    if (buzzerSettings.tankFullSound) Buzzer_StartEvent(BUZZ_TANK_FULL);
                }
                else { loadState = LOAD_NORMAL; loadRetryCount = 0; }
            }
            break;
    }
}

static uint8_t get_today_mask(void)
{
    uint8_t d = time.dow;
    if (d < 1 || d > 7) d = 1;
    return (1U << (d - 1));
}

static bool slot_is_active_now(const TimerSlot *t)
{
    if (!t->enabled) return false;
    if (!(t->dayMask & get_today_mask())) return false;
    uint16_t now = time.hour * 60 + time.min;
    uint16_t on  = t->onHour  * 60 + t->onMinute;
    uint16_t off = t->offHour * 60 + t->offMinute;
    if (on == off) return false;
    if (on < off)  return (now >= on && now < off);
    return (now >= on || now < off);
}

static bool timer_any_active_slot(void)
{
    for (int i = 0; i < 5; i++)
        if (slot_is_active_now(&timerSlots[i])) return true;
    return false;
}

static uint16_t get_active_timer_gap_minutes(void)
{
    if (!timerActive) return 0;
    uint16_t now = time.hour * 60 + time.min;
    uint8_t dm = get_today_mask();
    for (int i = 0; i < 5; i++)
    {
        TimerSlot *t = &timerSlots[i];
        if (!t->enabled || !(t->dayMask & dm)) continue;
        uint16_t on  = t->onHour * 60 + t->onMinute;
        uint16_t off = t->offHour * 60 + t->offMinute;
        bool active = (on < off) ? (now >= on && now < off) : (now >= on || now < off);
        if (active) return (uint16_t)(sys.gap_time_s / 60);
    }
    return 0;
}

void ModelHandle_SaveBuzzerSettings(void)
{
    BuzzerEEPROMBlock b;
    memset(&b, 0, sizeof(b));
    b.sig           = BUZZ_SIG;
    b.pumpOnSound   = buzzerSettings.pumpOnSound   ? 1 : 0;
    b.tankFullSound = buzzerSettings.tankFullSound  ? 1 : 0;
    b.tankEmptySound= buzzerSettings.tankEmptySound ? 1 : 0;
    b.crc = Buzzer_CRC16((uint8_t*)&b, sizeof(b) - sizeof(uint16_t));
    EEPROM_WriteBlockSafe(EE_ADDR_BUZZER_BLOCK, (uint8_t*)&b, sizeof(b));
}

void ModelHandle_LoadBuzzerSettings(void)
{
    BuzzerEEPROMBlock b;
    EEPROM_ReadBuffer(EE_ADDR_BUZZER_BLOCK, (uint8_t*)&b, sizeof(b));

    uint16_t crc = Buzzer_CRC16((uint8_t*)&b, sizeof(b) - sizeof(uint16_t));

    if (b.sig != BUZZ_SIG || crc != b.crc)
    {
        buzzerSettings.pumpOnSound    = 1;
        buzzerSettings.tankFullSound  = 1;
        buzzerSettings.tankEmptySound = 1;
        ModelHandle_SaveBuzzerSettings();
        return;
    }
    buzzerSettings.pumpOnSound    = b.pumpOnSound    ? 1 : 0;
    buzzerSettings.tankFullSound  = b.tankFullSound  ? 1 : 0;
    buzzerSettings.tankEmptySound = b.tankEmptySound ? 1 : 0;
}

void ModelHandle_ProcessTimerSlots(void)
{
    if (!timerActive) return;
    if (!autoActive)
    {
        timerActive        = false;
        timerStateDeadline = 0;
        dryState           = DRY_IDLE;
        stop_motor();
        return;
    }
    if (motorOwner != MOTOR_OWNER_TIMER &&
        motorOwner != MOTOR_OWNER_AUTO  &&
        motorOwner != MOTOR_OWNER_NONE)
    {
        return;
    }
    uint32_t now        = HAL_GetTick();
    bool     slotActive = timer_any_active_slot();
    if (!slotActive)
    {
        stop_motor();
        dryState = DRY_IDLE;
        timerState = TIMER_RUN_TEST;
        timerStateDeadline = 0;
        return;
    }
    if (isTankFull())
    {
        stop_motor();
        timerState = TIMER_RUN_TEST;
        timerStateDeadline = 0;
        dryState = DRY_IDLE;
        return;
    }
    if (senseOverLoad || senseUnderLoad || senseOverUnderVolt || senseMaxRunReached)
    {
        stop_motor();
        return;
    }
    if (!dry_protection_enabled())
    {
        motorOwner         = MOTOR_OWNER_TIMER;
        timerState         = TIMER_RUN_CONTINUOUS;
        timerStateDeadline = 0;
        dryState           = DRY_IDLE;
        start_motor();
        return;
    }
    uint32_t dryTestMs  = (uint32_t)sys.gap_time_s     * 1000UL;
    uint32_t retryGapMs = (uint32_t)sys.dry_run_time_s * 1000UL;
    ModelHandle_CheckDryRun();
    switch (timerState)
    {
        case TIMER_RUN_TEST:
            motorOwner = MOTOR_OWNER_TIMER;
            start_motor();
            dryState = DRY_WAITING;
            if (timerStateDeadline == 0) timerStateDeadline = now + dryTestMs;
            if (now >= timerStateDeadline)
            {
                if (!senseDryRun)
                {
                    stop_motor();
                    timerState         = TIMER_WAIT_RETRY;
                    timerStateDeadline = now + retryGapMs;
                    dryState           = DRY_FAULT;
                }
                else
                {
                    timerState = TIMER_RUN_CONTINUOUS;
                    dryState   = DRY_IDLE;
                }
            }
            break;
        case TIMER_WAIT_RETRY:
            dryState = DRY_FAULT;
            if (now >= timerStateDeadline)
            {
                timerState         = TIMER_RUN_TEST;
                timerStateDeadline = 0;
            }
            break;
        case TIMER_RUN_CONTINUOUS:
            start_motor();
            dryState = DRY_IDLE;
            if (!senseDryRun)
            {
                stop_motor();
                timerState         = TIMER_WAIT_RETRY;
                timerStateDeadline = now + retryGapMs;
                dryState           = DRY_FAULT;
            }
            break;
    }
}

void ModelHandle_StartTimer(void)
{
    ModelHandle_SaveTimerToEEPROM();
    ModelHandle_SendTimerInfoUART();
}

void ModelHandle_StopTimer(void)
{
    timerActive        = false;
    timerStateDeadline = 0;
    dryState           = DRY_IDLE;
    if (autoActive)
        motorOwner = MOTOR_OWNER_AUTO;
    else
        stop_motor();
    ModelHandle_SaveModeState();
}

void ModelHandle_CheckAutoTimerActivation(void)
{
    if (!autoActive) return;
    if (timerActive) return;
    if (timer_any_active_slot())
    {
        timerActive = true;
        motorOwner  = MOTOR_OWNER_TIMER;
        timerState  = TIMER_RUN_TEST;
        timerStateDeadline = 0;
        start_motor();
    }
}

void ModelHandle_StartSemiAuto(void)
{
    restartActive = false;
    clear_all_modes();
    semiAutoActive       = true;
    motorOwner           = MOTOR_OWNER_SEMIAUTO;
    semiState            = SEMI_RUN_TEST;
    semiDeadline         = 0;
    semiWaterLossPending = false;
    senseMaxRunReached   = false;
    semiTankFullHold     = false;
    dryState             = DRY_IDLE;
    start_motor();
    ModelHandle_SaveModeState();
}

void ModelHandle_StopSemiAuto(void)
{
    semiAutoActive   = false;
    semiTankFullHold = false;
    dryState         = DRY_IDLE;
    stop_motor();
    ModelHandle_SaveModeState();
}

void ModelHandle_StartAuto(uint16_t gap_s, uint16_t maxrun_min, uint8_t retry)
{
    clear_all_modes();

    /*
     * If Sense Dry Run is enabled but no dry-run value is configured,
     * automatically use default settings.
     */
    ensure_dry_run_defaults_if_enabled();

    if (gap_s == 0)
        gap_s = sys.gap_time_s;

    if (gap_s == 0)
        gap_s = DEFAULT_DRY_GAP_S;

    if (maxrun_min == 0)
        maxrun_min = sys.maxrun_min;

    if (maxrun_min == 0)
        maxrun_min = DEFAULT_AUTO_MAXRUN_MIN;

    auto_gap_s       = gap_s;
    auto_maxrun_min  = maxrun_min;
    auto_retry_limit = retry;
    auto_retry_count = 0;
    autoUserLocked   = false;
    autoActive       = true;
    autoState        = AUTO_ON_WAIT;
    stateDeadline    = 0;
    motorOwner       = MOTOR_OWNER_AUTO;
    dryState         = DRY_IDLE;

    if (timer_any_active_slot())
    {
        timerActive        = true;
        timerState         = TIMER_RUN_TEST;
        timerStateDeadline = 0;
        motorOwner         = MOTOR_OWNER_TIMER;
    }

    if (motorOwner == MOTOR_OWNER_AUTO)
        start_motor();

    ModelHandle_SaveModeState();
}
void ModelHandle_StopAuto(void)
{
    timerActive        = false;
    timerStateDeadline = 0;

    autoActive       = false;
    autoUserLocked   = true;
    autoState        = AUTO_IDLE;
    stateDeadline    = 0;
    auto_retry_count = 0;
    autoDeadline     = 0;
    dryState         = DRY_IDLE;
    motorOwner       = MOTOR_OWNER_NONE;

    stop_motor();
    ModelHandle_SaveModeState();
}

void ModelHandle_LoadAutoSettings(void)
{
    EEPROM_ReadBuffer(0x0300, (uint8_t*)&auto_gap_s,       sizeof(auto_gap_s));
    EEPROM_ReadBuffer(0x0302, (uint8_t*)&auto_maxrun_min,  sizeof(auto_maxrun_min));
    EEPROM_ReadBuffer(0x0304, (uint8_t*)&auto_retry_limit, sizeof(auto_retry_limit));

    if (auto_gap_s < 30 || auto_gap_s > 3600)
        { auto_gap_s = 120; EEPROM_WriteBlockSafe(0x0300, (uint8_t*)&auto_gap_s, sizeof(auto_gap_s)); }
    if (auto_maxrun_min == 0xFFFF || auto_maxrun_min > 600)
        { auto_maxrun_min = 30; EEPROM_WriteBlockSafe(0x0302, (uint8_t*)&auto_maxrun_min, sizeof(auto_maxrun_min)); }
    if (auto_retry_limit == 0xFF || auto_retry_limit > 20)
        { auto_retry_limit = 3; EEPROM_WriteBlockSafe(0x0304, (uint8_t*)&auto_retry_limit, sizeof(auto_retry_limit)); }
}

static void auto_mode_background_control(void)
{
    if (!autoBackgroundEnabled) return;
    if (autoUserLocked) return;
    uint32_t now = HAL_GetTick();
    if (now < autoBootIgnoreUntil) return;
    if (now < bootBlockUntil) return;
    if (manualActive || semiAutoActive || timerActive || countdownActive || twistActive) return;

    uint8_t level = get_tank_level_percent();
    if (!autoActive &&
        level <= AUTO_START_LEVEL_PERCENT &&
        powerRestoreMode != 1)
    {
        ModelHandle_StartAuto(sys.gap_time_s, auto_maxrun_min, auto_retry_limit);
        return;
    }
    if (autoActive && level >= AUTO_STOP_LEVEL_PERCENT)
    {
        stop_motor();
        autoState     = AUTO_ON_WAIT;
        stateDeadline = 0;
        dryState      = DRY_IDLE;
        ModelHandle_SaveModeState();
    }
}

void ModelHandle_SetBuzzerSettings(uint8_t pump, uint8_t full, uint8_t empty)
{
    buzzerSettings.pumpOnSound    = pump  ? 1 : 0;
    buzzerSettings.tankFullSound  = full  ? 1 : 0;
    buzzerSettings.tankEmptySound = empty ? 1 : 0;
    ModelHandle_SaveBuzzerSettings();
}

static void auto_tick(void)
{
    if (!autoActive) return;

    uint32_t now = HAL_GetTick();

    ModelHandle_CheckDryRun();
    ModelHandle_CheckGroundWater();

    uint8_t level = get_tank_level_percent();

    bool protectionFault =
        senseOverLoad ||
        senseUnderLoad ||
        senseOverUnderVolt ||
        senseMaxRunReached;

    if (protectionFault)
    {
        stop_motor();
        autoState     = AUTO_IDLE;
        stateDeadline = 0;
        return;
    }

    if (level >= AUTO_STOP_LEVEL_PERCENT)
    {
        stop_motor();
        autoState     = AUTO_ON_WAIT;
        stateDeadline = 0;
        dryState      = DRY_IDLE;
        return;
    }

    /*
     * Auto mode final behavior:
     *
     * 1. If tank level <= start level, motor starts.
     * 2. During first dry-test gap, dry sensor is checked.
     * 3. If water is detected, motor keeps running continuously.
     * 4. If water is not detected, motor stops and waits retry time.
     * 5. After retry time, it tries again.
     *
     * This prevents unnecessary AUTO ON / AUTO W cycling.
     */

    if (!dry_protection_enabled())
    {
        /*
         * Dry protection disabled:
         * Auto mode should run motor continuously until tank full
         * or protection fault.
         */
        if (level <= AUTO_START_LEVEL_PERCENT)
        {
            motorOwner = MOTOR_OWNER_AUTO;
            dryState   = DRY_IDLE;
            autoState  = AUTO_OFF_WAIT;
            start_motor();
        }

        return;
    }

    uint32_t dryTestMs  = (uint32_t)sys.gap_time_s * 1000UL;
    uint32_t retryGapMs = (uint32_t)sys.dry_run_time_s * 1000UL;

    switch (autoState)
    {
        case AUTO_IDLE:
        case AUTO_ON_WAIT:
        {
            if (level > AUTO_START_LEVEL_PERCENT)
            {
                stop_motor();
                dryState      = DRY_IDLE;
                stateDeadline = 0;
                break;
            }

            motorOwner = MOTOR_OWNER_AUTO;
            start_motor();

            autoState     = AUTO_OFF_WAIT;
            stateDeadline = now + dryTestMs;
            dryState      = DRY_WAITING;
            break;
        }

        case AUTO_OFF_WAIT:
        {
            /*
             * Motor is ON during dry-test period.
             * After dry-test time, decide:
             * - water available     → keep motor ON continuously
             * - water not available → stop motor and wait retry
             */

            start_motor();

            if ((int32_t)(now - stateDeadline) >= 0)
            {
                ModelHandle_CheckDryRun();

                if (senseDryRun)
                {
                    /*
                     * Water available.
                     * Keep motor ON. Do not go to AUTO W.
                     */
                    dryState      = DRY_IDLE;
                    stateDeadline = 0;

                    /*
                     * Stay in AUTO_OFF_WAIT as continuous running state.
                     * No more ON/OFF cycling.
                     */
                    start_motor();
                }
                else
                {
                    /*
                     * Water not available.
                     * Now only stop motor and enter AUTO W / wait state.
                     */
                    stop_motor();
                    autoState     = AUTO_DRY_CHECK;
                    stateDeadline = now + retryGapMs;
                    dryState      = DRY_FAULT;

                    if (buzzerSettings.tankEmptySound)
                        Buzzer_StartEvent(BUZZ_TANK_EMPTY);
                }
            }

            break;
        }

        case AUTO_DRY_CHECK:
        {
            /*
             * Motor is OFF during retry wait.
             * After retry time, try again.
             */

            stop_motor();

            if ((int32_t)(now - stateDeadline) >= 0)
            {
                ModelHandle_CheckDryRun();
                ModelHandle_CheckGroundWater();

                if (senseDryRun || groundWater)
                {
                    autoState     = AUTO_ON_WAIT;
                    stateDeadline = 0;
                    dryState      = DRY_IDLE;
                }
                else
                {
                    /*
                     * Still no water.
                     * Keep waiting another retry cycle.
                     */
                    stateDeadline = now + retryGapMs;
                    dryState      = DRY_FAULT;

                    if (buzzerSettings.tankEmptySound)
                        Buzzer_StartEvent(BUZZ_TANK_EMPTY);
                }
            }

            break;
        }

        default:
        {
            autoState     = AUTO_ON_WAIT;
            stateDeadline = 0;
            dryState      = DRY_IDLE;
            break;
        }
    }
}
void ModelHandle_StartCountdown(uint32_t seconds)
{
    if (seconds < 60)    seconds = 60;
    if (seconds > 10800) seconds = 10800;

    clear_all_modes();

    senseMaxRunReached = false;
    countdownActive    = true;
    countdownMode      = true;

    uint32_t now       = HAL_GetTick();
    cd_deadline        = now + (seconds * 1000UL);
    countdownDuration  = seconds;
    motorOwner         = MOTOR_OWNER_COUNTDOWN;
    cdTankFullHold     = false;
    dryState           = DRY_IDLE;

    start_motor();
    ModelHandle_SaveModeState();
}

void ModelHandle_StopCountdown(void)
{
    countdownActive   = false;
    countdownMode     = false;
    cd_deadline       = 0;
    countdownDuration = 0;
    cdTankFullHold    = false;
    dryState          = DRY_IDLE;
    motorOwner        = MOTOR_OWNER_NONE;

    suppressAutoOneCycle = true;

    stop_motor();
    ModelHandle_SaveModeState();
}
static uint16_t CD_CRC(const uint8_t* d, uint16_t l)
{
    uint16_t c = 0xFFFF;
    while (l--) c = (c >> 1) ^ (*d++ + 0xA001);
    return c;
}

static void SaveCountdown(void)
{
    CountdownBlock b;
    memset(&b, 0, sizeof(b));
    b.sig    = CD_SIGNATURE;
    b.active = countdownActive;
    if (countdownActive)
    {
        uint32_t now = now_ms();
        b.remaining = (cd_deadline > now) ? (cd_deadline - now) / 1000UL : 0;
    }
    b.crc = CD_CRC((uint8_t*)&b, sizeof(b) - 2);
    EEPROM_WriteBuffer(EE_ADDR_COUNTDOWN_BLOCK, (uint8_t*)&b, sizeof(b));
}

void ModelHandle_LoadCountdown(void)
{
    CountdownBlock b;
    EEPROM_ReadBuffer(EE_ADDR_COUNTDOWN_BLOCK, (uint8_t*)&b, sizeof(b));
    if (b.sig != CD_SIGNATURE) return;
    if (CD_CRC((uint8_t*)&b, sizeof(b) - 2) != b.crc) return;
    if (b.active && b.remaining > 0)
    {
        countdownActive   = true;
        countdownMode     = true;
        countdownDuration = b.remaining;
        cd_deadline       = now_ms() + b.remaining * 1000UL;
        motorOwner        = MOTOR_OWNER_COUNTDOWN;
    }
}

void ModelHandle_StartTwist(uint16_t on_s, uint16_t off_s,
                            uint8_t onH, uint8_t onM,
                            uint8_t offH, uint8_t offM)
{
    clear_all_modes();
    uint32_t on_sec  = (on_s  ? on_s  : 1) * 60UL;
    uint32_t off_sec = (off_s ? off_s : 1) * 60UL;
    twistSettings.onDurationSeconds  = on_sec;
    twistSettings.offDurationSeconds = off_sec;
    twistSettings.onHour    = onH;
    twistSettings.onMinute  = onM;
    twistSettings.offHour   = offH;
    twistSettings.offMinute = offM;
    twistSettings.twistArmed  = true;
    twistSettings.twistActive = false;
    twistActive    = true;
    motorOwner     = MOTOR_OWNER_TWIST;
    twist_on_phase = true;
    twist_deadline = now_ms() + on_sec * 1000UL;
    dryState       = DRY_IDLE;
    start_motor();
}

void ModelHandle_StopTwist(void)
{
    twistActive               = false;
    twistSettings.twistActive = false;
    dryState                  = DRY_IDLE;
    stop_motor();
    ModelHandle_SaveModeState();
}

static void twist_tick(void)
{
    if (!twistActive) return;
    if (isTankFull()) { ModelHandle_StopTwist(); if (buzzerSettings.tankFullSound) Buzzer_StartEvent(BUZZ_TANK_FULL); return; }

    uint32_t now = now_ms();
    if (now >= twist_deadline)
    {
        twist_on_phase = !twist_on_phase;
        twist_deadline = now + (twist_on_phase ?
            twistSettings.onDurationSeconds :
            twistSettings.offDurationSeconds) * 1000UL;
    }
    if (twist_on_phase) start_motor(); else stop_motor();
}

static void leds_from_model(void)
{
    LED_ClearAllIntents();
    bool motorOn = Motor_GetStatus();
    if (dryState == DRY_FAULT)
    {
        LED_SetIntent(LED_COLOR_RED, LED_MODE_BLINK, 400);
        LED_ApplyIntents();
        return;
    }
    if (motorOn)          LED_SetIntent(LED_COLOR_GREEN,  LED_MODE_STEADY, 0);
    if (senseMaxRunReached) LED_SetIntent(LED_COLOR_RED,  LED_MODE_BLINK,  300);
    if (senseOverLoad || senseUnderLoad) LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 350);
    if (senseOverUnderVolt) LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_BLINK, 350);
    LED_ApplyIntents();
}
/* ─────────────────────────────────────────────────────────────────────────
 * Timer Mode Auto Transition
 *   • Any enabled slot's ON time reached → timerActive = true,
 *     motorOwner becomes TIMER, ProcessTimerSlots then drives the pump.
 *   • Slot's OFF time reached (no slot currently active) → timerActive
 *     = false, auto mode is re-armed and resumes level-based control.
 * Called every tick from ModelHandle_Process().
 * ─────────────────────────────────────────────────────────────────────────*/
static void timer_auto_transition(void)
{
    /* Foreground modes override timer — don't touch state while they run */
    if (manualActive || semiAutoActive || countdownActive ||
        twistActive  || restartActive)
        return;

    /* If the user explicitly turned auto off (ToggleManual sets this),
     * respect that and do not auto-enter timer mode.                    */
    if (!autoActive && !timerActive && autoUserLocked)
        return;

    bool slotActive = timer_any_active_slot();

    /* ── ON edge: a slot window just started ────────────────────────── */
    if (slotActive && !timerActive)
    {
        /* Timer runs "under" auto — make sure auto is enabled */
        if (!autoActive) { autoActive = true; autoUserLocked = false; }

        timerActive        = true;
        timerState         = TIMER_RUN_TEST;
        timerStateDeadline = 0;
        motorOwner         = MOTOR_OWNER_TIMER;
        dryState           = DRY_IDLE;
        ModelHandle_SaveModeState();
        return;
    }

    /* ── OFF edge: slot window just ended → fall back to auto mode ──── */
    if (!slotActive && timerActive)
    {
        timerActive        = false;
        timerState         = TIMER_RUN_TEST;
        timerStateDeadline = 0;
        stop_motor();

        /* Re-arm auto mode so level-based filling continues */
        autoActive     = true;
        autoUserLocked = false;
        autoState      = AUTO_ON_WAIT;
        stateDeadline  = 0;
        motorOwner     = MOTOR_OWNER_AUTO;
        dryState       = DRY_IDLE;
        ModelHandle_SaveModeState();
        return;
    }
}
void ModelHandle_Process(void)
{
    uint32_t now = HAL_GetTick();

    if (modeSwitchPending)
    {
        if (now >= modeSwitchTime) { modeSwitchPending = false; motorOwner = pendingOwner; }
        return;
    }

    ModelHandle_CheckGroundWater();
    ModelHandle_CheckDryRun();
    ModelHandle_CheckLoadFault();
    check_max_run();
    timer_auto_transition();
    bool protectionFault = senseOverLoad || senseUnderLoad || senseOverUnderVolt || senseMaxRunReached;
    bool tankFull        = isTankFull();

    MotorOwner newOwner;
    if      (restartActive)   newOwner = MOTOR_OWNER_RESTART;
    else if (timerActive)     newOwner = MOTOR_OWNER_TIMER;
    else if (manualActive)    newOwner = MOTOR_OWNER_MANUAL;
    else if (semiAutoActive)  newOwner = MOTOR_OWNER_SEMIAUTO;
    else if (countdownActive) newOwner = MOTOR_OWNER_COUNTDOWN;
    else if (twistActive)     newOwner = MOTOR_OWNER_TWIST;
    else if (autoActive)      newOwner = MOTOR_OWNER_AUTO;
    else                      newOwner = MOTOR_OWNER_NONE;
    motorOwner = newOwner;

    static bool prevTankFull      = false;
    static bool tankFullBuzzFired = false;

    if (motorOwner != prevMotorOwner)
    {
        prevTankFull      = false;
        tankFullBuzzFired = false;
        prevMotorOwner    = motorOwner;
    }

    if (tankFull && motorOwner != MOTOR_OWNER_MANUAL)
    {
        if (!prevTankFull)
        {
            tankFullDetectedAt = now;
            tankFullBuzzFired  = false;
        }
        if ((now - tankFullDetectedAt) >= TANK_FULL_DELAY_MS)
        {
            stop_motor();
            if (buzzerSettings.tankFullSound && !tankFullBuzzFired)
            {
                Buzzer_StartEvent(BUZZ_TANK_FULL);
                tankFullBuzzFired = true;
            }
        }
        prevTankFull = true;
    }
    else
    {
        prevTankFull = false;
        if (!tankFull) tankFullBuzzFired = false;
    }

    static uint8_t prevLevel = 100;
    uint8_t currentLevel = get_tank_level_percent();
    if (currentLevel == 0 && prevLevel > 0)
        if (buzzerSettings.tankEmptySound) Buzzer_StartEvent(BUZZ_TANK_EMPTY);
    prevLevel = currentLevel;

    switch (motorOwner)
    {
        case MOTOR_OWNER_RESTART:
            if (tankFull)
            {
                stop_motor(); restartActive = false;
                if (buzzerSettings.tankFullSound && !tankFullBuzzFired)
                    { Buzzer_StartEvent(BUZZ_TANK_FULL); tankFullBuzzFired = true; }
                dryState = DRY_IDLE;
                restore_previous_mode();
                ModelHandle_SaveModeState();
                break;
            }
            if (protectionFault) { stop_motor(); break; }
            start_motor();
            break;

        case MOTOR_OWNER_MANUAL:
            if (!protectionFault) start_motor(); else stop_motor();
            break;

        case MOTOR_OWNER_SEMIAUTO:
            if (protectionFault)
            {
                stop_motor();
            }
            else if (tankFull || semiTankFullHold)
            {
                semiTankFullHold = true;
                stop_motor();
            }
            else
            {
                start_motor();
            }
            break;

        case MOTOR_OWNER_TIMER:
            ModelHandle_ProcessTimerSlots();
            break;

        case MOTOR_OWNER_COUNTDOWN:
            if (!countdownActive) break;
            if (now >= cd_deadline) { ModelHandle_StopCountdown(); break; }
            countdownDuration = (cd_deadline - now) / 1000UL;
            if (protectionFault) { ModelHandle_StopCountdown(); break; }
            if (tankFull || cdTankFullHold)
            {
                cdTankFullHold = true;
                stop_motor();
            }
            else
            {
                start_motor();
            }
            break;

        case MOTOR_OWNER_TWIST:
            if (protectionFault) { stop_motor(); break; }
            twist_tick();
            break;

        case MOTOR_OWNER_AUTO:
            auto_tick();
            break;

        default:
            stop_motor();
            break;
    }

    ModelHandle_SoftDryRunHandler();

    if ((motorOwner == MOTOR_OWNER_NONE || motorOwner == MOTOR_OWNER_AUTO) &&
        !tankFull &&
        dryState != DRY_FAULT)
    {
        auto_mode_background_control();
    }

    leds_from_model();
    Buzzer_Update();
}

DryFSMState ModelHandle_GetDryState(void) { return dryState; }

void ModelHandle_ResetAll(void)
{
    clear_all_modes();
    stop_motor();
    senseDryRun        = false;
    senseOverLoad      = false;
    senseUnderLoad     = false;
    senseOverUnderVolt = false;
    senseMaxRunReached = false;
    countdownDuration  = 0;
    dryState           = DRY_IDLE;
    UART_SendStatusPacket();
}

void ModelHandle_SetUserSettings(uint32_t gap_seconds,
                                 uint8_t  retry,
                                 uint16_t uv_limit,
                                 uint16_t ov_limit,
                                 int16_t  overload,
                                 int16_t  underload,
                                 uint16_t maxrun_min)
{
    if (gap_seconds > 86400UL) gap_seconds = 86400UL;
    if (retry > 20)            retry       = 20;
    if (maxrun_min > 1440)     maxrun_min  = 1440;
    sys.gap_time_s  = gap_seconds;
    sys.retry_count = retry;
    sys.uv_limit    = uv_limit;
    sys.ov_limit    = ov_limit;
    sys.overload    = (float)overload;
    sys.underload   = (float)underload;
    sys.maxrun_min  = maxrun_min;
    if (gap_seconds == 0) sys.dry_run_enable = 0;
    ModelHandle_SaveSettingsToEEPROM();
    senseOverLoad = senseUnderLoad = senseOverUnderVolt = senseMaxRunReached = false;
    if (autoActive)
    {
        uint32_t gap = sys.gap_time_s ? sys.gap_time_s : 120;
        autoDeadline = HAL_GetTick() + (gap * 1000UL);
    }
    char dbg[80];
    snprintf(dbg, sizeof(dbg),
        "@SYS_UPDATE:GAP:%lu RET:%d UV:%d OV:%d MAX:%d DR:%d#",
        sys.gap_time_s, sys.retry_count, sys.uv_limit,
        sys.ov_limit, sys.maxrun_min, sys.dry_run_enable);
    UART_TransmitPacket(dbg);
}

void ModelHandle_SetAutoSettings(uint16_t gap_s, uint16_t maxrun_min, uint8_t retry)
{
    auto_gap_s       = (gap_s < 30) ? 120 : gap_s;
    auto_maxrun_min  = maxrun_min;
    auto_retry_limit = retry;
    EEPROM_WriteBlockSafe(0x0300, (uint8_t*)&auto_gap_s,       sizeof(auto_gap_s));
    EEPROM_WriteBlockSafe(0x0302, (uint8_t*)&auto_maxrun_min,  sizeof(auto_maxrun_min));
    EEPROM_WriteBlockSafe(0x0304, (uint8_t*)&auto_retry_limit, sizeof(auto_retry_limit));
}

bool ModelHandle_IsAutoActive(void) { return autoActive; }

void ModelHandle_SaveCurrentStateToEEPROM(void)
{
    RTC_PersistState s;
    memset(&s, 0, sizeof(s));
    if      (manualActive)    s.mode = 1;
    else if (semiAutoActive)  s.mode = 2;
    else if (timerActive)     s.mode = 3;
    else if (countdownActive) s.mode = 4;
    else if (twistActive)     s.mode = 5;
    else if (autoActive)      s.mode = 6;
    RTC_SavePersistentState(&s);
}

void ModelHandle_SetTimerSlot(uint8_t slot,
                              uint8_t onH, uint8_t onM,
                              uint8_t offH, uint8_t offM)
{
    timerSlots[slot].onHour    = onH;
    timerSlots[slot].onMinute  = onM;
    timerSlots[slot].offHour   = offH;
    timerSlots[slot].offMinute = offM;
    timerSlots[slot].enabled   = 1;
    ModelHandle_SendTimerInfoUART();
}

void ModelHandle_SetDryRun(bool on)
{
    sys.dry_run_enable = on ? 1 : 0;
    if (!on) dryState = DRY_IDLE;
    ModelHandle_SaveSettingsToEEPROM();
}

bool ModelHandle_GetDryRunEnable(void) { return dry_protection_enabled(); }

void ModelHandle_SetOverLoad(bool on)
{
    if (!on) sys.overload = 0.0f;
    else if (sys.overload < 0.1f) sys.overload = 9.0f;
}

void ModelHandle_SetOverUnderVolt(bool on)
{
    if (!on) { sys.uv_limit = 0; sys.ov_limit = 0; }
    else
    {
        if (!sys.uv_limit) sys.uv_limit = 190;
        if (!sys.ov_limit) sys.ov_limit = 270;
    }
}

void ModelHandle_ClearMaxRunFlag(void)  { senseMaxRunReached = false; }

const TimerSlot* ModelHandle_GetTimerSlots(void) { return timerSlots; }

void ModelHandle_ProcessUartCommand(const char* cmd) { (void)cmd; }

bool ModelHandle_IsOverload(void)       { return senseOverLoad; }
bool ModelHandle_IsUnderload(void)      { return senseUnderLoad; }
bool ModelHandle_IsDryRunActive(void)   { return senseDryRun; }
bool ModelHandle_IsVoltageFault(void)   { return senseOverUnderVolt; }
bool ModelHandle_IsMaxRunReached(void)  { return senseMaxRunReached; }
uint8_t ModelHandle_GetTankLevelPercent(void) { return get_tank_level_percent(); }
bool ModelHandle_IsTankFull(void)       { return isTankFull(); }
bool ModelHandle_IsManualActive(void)   { return manualActive; }
uint16_t ModelHandle_GetAutoGap(void)   { return auto_gap_s; }
uint16_t ModelHandle_GetAutoMaxRun(void){ return auto_maxrun_min; }
uint8_t  ModelHandle_GetAutoRetry(void) { return auto_retry_limit; }
