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
#include <stddef.h>

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

#define TWIST_EE_SIGNATURE  0x5457
#define TWIST_EE_VERSION    1
#define EE_ADDR_TWIST_BLOCK 0x0800

typedef struct {
    uint16_t      signature;
    uint8_t       version;
    uint8_t       reserved;
    TwistSettings settings;
    uint16_t      crc;
} TwistEEPROMBlock;

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
static bool     autoRestoreOverride = false;  /* power restore = independent on-key for Auto */
static bool     powerRestoreHold    = false;  /* restore OFF (or LAST with motor off): wait for next on-key */
static uint32_t powerRestoreHoldUntil = 0;    /* ...at most one testing gap (itself an on key) */
static bool     manualMotorOff      = false;  /* manual mode active but motor switched/held off */
static bool     manualFromRestore   = false;  /* manual restored at power-up, motor held off */
static bool     autoPausedByUser    = false;  /* Auto switched off with the button (shows "AUTO", motor off) */
static bool     manualPausedByUser  = false;  /* Manual switched off with the button */

/* Relay2 / Relay3 give a 2 s push to an external starter panel
 * (start push when the motor relay closes, stop push when it opens). */
#define STARTER_PULSE_MS 2000UL
static uint32_t starterOnPulseEnd  = 0;
static uint32_t starterOffPulseEnd = 0;

/* Factory defaults (testing report 24-09-26, item 7) */
#define FACTORY_DRY_TEST_S    120U     /* dry-run test window        */
#define FACTORY_TEST_GAP_S    1800U    /* testing gap: 30 min        */
#define FACTORY_MAXRUN_MIN    150U
#define FACTORY_RETRY_COUNT   5U
#define FACTORY_UV            180U
#define FACTORY_OV            280U
#define FACTORY_OVERLOAD_A    25.0f
#define FACTORY_PWR_RESTORE   0U       /* always restore (setting removed) */
#define FACTORY_COUNTDOWN_MIN 10U

#define GW_START_LEVEL_PERCENT 75      /* ground water may start the motor up to 75% */

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

static uint32_t motorOnStartMs = 0;
static uint32_t powerOnMs      = 0;
static DryFSMState dryState    = DRY_IDLE;

volatile uint8_t motorStatus = 0;
volatile bool senseDryRun         = false;
volatile bool groundWater         = false;
static bool   prevGroundWaterAuto  = false;  /* edge-detect for Auto's independent GW trigger */
static bool   autoGroundWaterRun   = false;  /* true while current Auto run was started by GW */
static bool   prevGroundWaterTimer = false;  /* edge-detect for Timer's independent GW trigger */
static bool   timerGroundWaterRun  = false;  /* true while current Timer run was started by GW */
volatile bool senseOverLoad       = false;
volatile bool senseUnderLoad      = false;
volatile bool senseOverUnderVolt  = false;
volatile bool senseMaxRunReached  = false;
volatile bool manualOverride      = false;
volatile uint16_t auto_retry_counter = 0;
volatile bool     countdownMode      = false;
volatile uint32_t countdownDuration  = 0;
#define DEFAULT_DRY_GAP_S        FACTORY_DRY_TEST_S   /* motor run/test time */
#define DEFAULT_DRY_RETRY_S      FACTORY_TEST_GAP_S   /* testing gap         */
#define DEFAULT_AUTO_MAXRUN_MIN  FACTORY_MAXRUN_MIN
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
static uint8_t    timer_retry_count  = 0;

typedef struct {
    uint16_t sig;
    uint32_t remaining;
    uint8_t  active;
    uint16_t crc;
} CountdownBlock;

#define SEMI_TANK_FULL_DELAY_MS 10000UL

static uint8_t powerRestoreMode = 0;

SystemSettings sys = {
    .gap_time_s      = FACTORY_DRY_TEST_S,
    .retry_count     = FACTORY_RETRY_COUNT,
    .uv_limit        = FACTORY_UV,
    .ov_limit        = FACTORY_OV,
    .overload        = FACTORY_OVERLOAD_A,
    .underload       = 0.0f,
    .maxrun_min      = FACTORY_MAXRUN_MIN,
    .dry_run_time_s  = FACTORY_TEST_GAP_S,
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
static bool     twist_window_active(void);
static void     twist_tick(void);
static void     twist_reset_sensors(void);
static bool     twist_load_run(void);
static void     twist_save_run(void);
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

static BuzzerSettings buzzerSettings = {0, 0, 0};

static uint32_t cd_deadline = 0;

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

/* Auto starts at 50% (client decision Q1-b), also after it has filled
 * the tank. Twist restarts after tank full only at 25%. */
#define AUTO_START_LEVEL_PERCENT   50
#define AUTO_REFILL_LEVEL_PERCENT  25   /* Twist: after tank full, restart only at <= 25% */

static bool autoFilledLatch = false;    /* Auto filled the tank: ground water alone does not restart it */
#define AUTO_STOP_LEVEL_PERCENT    100


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

/* sys.gap_time_s is the dry-run test window ("D" in the app),
 * sys.dry_run_time_s is the testing gap between retries ("T"). */
static uint32_t get_dry_test_ms(void)
{
    return (uint32_t)(sys.gap_time_s ? sys.gap_time_s : DEFAULT_DRY_GAP_S) * 1000UL;
}

static uint32_t get_test_gap_ms(void)
{
    return (uint32_t)(sys.dry_run_time_s ? sys.dry_run_time_s : DEFAULT_DRY_RETRY_S) * 1000UL;
}

static bool restore_hold_active(void)
{
    if (!powerRestoreHold) return false;
    if ((int32_t)(HAL_GetTick() - powerRestoreHoldUntil) >= 0)
        powerRestoreHold = false;
    return powerRestoreHold;
}

/* Retry count 0 = retry forever */
static bool retries_exhausted(uint8_t failedRuns)
{
    return sys.retry_count != 0 && failedRuns >= sys.retry_count;
}
uint16_t ModelHandle_GetGapTime(void)        { return sys.gap_time_s; }
uint8_t  ModelHandle_GetRetryCount(void)     { return sys.retry_count; }
uint16_t ModelHandle_GetUnderVolt(void)      { return sys.uv_limit; }
uint16_t ModelHandle_GetOverVolt(void)       { return sys.ov_limit; }
float    ModelHandle_GetOverloadLimit(void)  { return sys.overload; }
float    ModelHandle_GetUnderloadLimit(void) { return sys.underload; }
uint16_t ModelHandle_GetMaxRunTime(void)     { return sys.maxrun_min; }
uint16_t ModelHandle_GetDryRunRetryGap(void) { return sys.dry_run_time_s; }

/* Kept for existing callers; now the same verified, page-safe write */
void EEPROM_WriteBlockSafe(uint16_t addr, uint8_t *data, uint16_t len)
{
    EEPROM_WriteBuffer(addr, data, len);
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
    ModelHandle_LoadCountdownDefault();
}

/* Mode that was running before a one-time run (Refill, Countdown); it
 * continues when that run ends. */
static bool previousTimer       = false;
static bool previousTwist       = false;
static bool previousAutoPaused  = false;
static bool previousManualPaused = false;

void ModelHandle_SaveModeState(void)
{
    if (restartActive)
    {
        /* Refill is not a mode: during it the mode it was started from is
         * the last mode, so a power cut returns to that mode. */
        modeState.manual_on    = previousManual || previousManualPaused;
        modeState.semi_on      = previousSemi;
        modeState.timer_on     = previousTimer;
        modeState.countdown_on = previousCountdown;
        modeState.twist_on     = previousTwist;
        modeState.auto_on      = previousAuto || previousAutoPaused;
        modeState.motor_on     = false;
    }
    else
    {
        /* Auto / Manual switched off with the button still count as the
         * last mode for power restore (the dashboard keeps showing
         * "AUTO"/"MANUAL" with the motor off). */
        modeState.manual_on    = manualActive || manualPausedByUser;
        modeState.semi_on      = semiAutoActive;
        modeState.timer_on     = timerActive;
        modeState.countdown_on = countdownActive;
        modeState.twist_on     = twistActive;
        modeState.auto_on      = autoActive || autoPausedByUser;
        modeState.motor_on     = (HAL_GPIO_ReadPin(Relay1_GPIO_Port, Relay1_Pin) == GPIO_PIN_SET);
    }
    modeState.power_restore_mode = powerRestoreMode;
    EEPROM_WriteBuffer(0x0200, (uint8_t*)&modeState, sizeof(modeState));
}

/* The previous mode is also kept in EEPROM, so a countdown resumed after
 * a power cut still returns to the right mode when it ends. */
#define EE_ADDR_PREV_MODE  0x0070
#define PREV_MODE_SIG      0xB7

static void save_previous_mode(void)
{
    uint8_t flags = (previousManual       ? 0x01 : 0) | (previousSemi  ? 0x02 : 0) |
                    (previousCountdown    ? 0x04 : 0) | (previousAuto  ? 0x08 : 0) |
                    (previousTimer        ? 0x10 : 0) | (previousTwist ? 0x20 : 0) |
                    (previousAutoPaused   ? 0x40 : 0) | (previousManualPaused ? 0x80 : 0);
    uint8_t b[3] = { PREV_MODE_SIG, flags, (uint8_t)(PREV_MODE_SIG ^ flags) };
    EEPROM_WriteBuffer(EE_ADDR_PREV_MODE, b, sizeof(b));
}

static void load_previous_mode(void)
{
    uint8_t b[3];
    EEPROM_ReadBuffer(EE_ADDR_PREV_MODE, b, sizeof(b));
    uint8_t flags = (b[0] == PREV_MODE_SIG && b[2] == (uint8_t)(b[0] ^ b[1])) ? b[1] : 0;
    previousManual       = (flags & 0x01) != 0;
    previousSemi         = (flags & 0x02) != 0;
    previousCountdown    = (flags & 0x04) != 0;
    previousAuto         = (flags & 0x08) != 0;
    previousTimer        = (flags & 0x10) != 0;
    previousTwist        = (flags & 0x20) != 0;
    previousAutoPaused   = (flags & 0x40) != 0;
    previousManualPaused = (flags & 0x80) != 0;
}

static void backup_current_mode(void)
{
    previousOwner        = motorOwner;
    previousManual       = manualActive;
    previousSemi         = semiAutoActive;
    previousCountdown    = countdownActive;
    previousAuto         = autoActive;
    previousTimer        = timerActive;
    previousTwist        = twistActive;
    previousAutoPaused   = autoPausedByUser;
    previousManualPaused = manualPausedByUser;
    save_previous_mode();
}

static void restore_previous_mode(void)
{
    manualActive       = previousManual;
    semiAutoActive     = previousSemi;
    countdownActive    = previousCountdown;
    autoActive         = previousAuto;
    timerActive        = previousTimer;
    twistActive        = previousTwist;
    autoPausedByUser   = previousAutoPaused;
    manualPausedByUser = previousManualPaused;

    if (previousManual)
        motorOwner = MOTOR_OWNER_MANUAL;
    else if (previousSemi)
        motorOwner = MOTOR_OWNER_SEMIAUTO;
    else if (previousCountdown)
        motorOwner = MOTOR_OWNER_COUNTDOWN;
    else if (previousTimer)
    {
        /* Timer / Twist / Auto decide the motor themselves on their next tick */
        motorOwner         = MOTOR_OWNER_TIMER;
        timerState         = TIMER_RUN_TEST;
        timerStateDeadline = 0;
        dryState           = DRY_IDLE;
        return;
    }
    else if (previousTwist)
    {
        motorOwner     = MOTOR_OWNER_TWIST;
        twist_deadline = 0;
        twist_on_phase = true;
        twist_reset_sensors();
        return;
    }
    else if (previousAuto)
    {
        motorOwner    = MOTOR_OWNER_AUTO;
        autoState     = AUTO_ON_WAIT;
        stateDeadline = 0;
        dryState      = DRY_IDLE;
        return;
    }
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
    uint8_t raw[sizeof(modeState)];
    EEPROM_ReadBuffer(0x0200, raw, sizeof(raw));

    /* Blank / corrupt block (first boot): nothing to restore. Check the
     * raw bytes before treating them as bools. */
    bool valid = (raw[offsetof(ModeState, power_restore_mode)] <= 2);
    for (size_t i = 0; valid && i < offsetof(ModeState, power_restore_mode); i++)
        if (raw[i] > 1) valid = false;
    if (!valid)
    {
        memset(&modeState, 0, sizeof(modeState));
        modeState.power_restore_mode = FACTORY_PWR_RESTORE;
    }
    else memcpy(&modeState, raw, sizeof(modeState));
    powerRestoreMode = 0;   /* setting removed: last mode is always restored */

    /* The last mode is kept in every power-restore setting; whether it is
     * resumed and whether the motor may run straight away is decided in
     * ModelHandle_OnPowerUp(). Refill is never saved as a mode. */
    manualActive    = modeState.manual_on;
    timerActive     = modeState.timer_on;
    twistActive     = modeState.twist_on;
    autoActive      = modeState.auto_on;
    semiAutoActive  = modeState.semi_on;
    countdownActive = false;              /* resumed from its own block */
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

void ModelHandle_SaveTwistToEEPROM(void)
{
    TwistEEPROMBlock blk;
    memset(&blk, 0, sizeof(blk));
    blk.signature = TWIST_EE_SIGNATURE;
    blk.version   = TWIST_EE_VERSION;
    blk.settings  = twistSettings;
    blk.crc = Timer_CRC16((uint8_t*)&blk, sizeof(blk) - sizeof(uint16_t));
    EEPROM_WriteBuffer(EE_ADDR_TWIST_BLOCK, (uint8_t*)&blk, sizeof(blk));
}

void ModelHandle_LoadTwistFromEEPROM(void)
{
    TwistEEPROMBlock blk;
    EEPROM_ReadBuffer(EE_ADDR_TWIST_BLOCK, (uint8_t*)&blk, sizeof(blk));
    if (blk.signature != TWIST_EE_SIGNATURE) return;
    uint16_t crc = Timer_CRC16((uint8_t*)&blk, sizeof(blk) - 2);
    if (blk.crc != crc) return;
    twistSettings = blk.settings;
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

/* The Power Restore setting was removed (client decisions Q3/Q4): the last
 * mode is always restored. Kept so old app packets (PR=x) do no harm. */
void ModelHandle_SetPowerRestoreMode(uint8_t mode)
{
    (void)mode;
    powerRestoreMode = 0;
}

void ModelHandle_ForcePowerRestoreModeEarly(uint8_t mode)
{
    /* Corrects power_restore_mode directly in the raw EEPROM block,
     * BEFORE ModelHandle_LoadModeState() has run - so the corrected
     * value is what gets read on THIS boot, not just future ones.
     * Reads the current block first and only changes this one byte,
     * so manual_on/semi_on/timer_on/etc. (not yet loaded into RAM)
     * are preserved exactly as stored. */
    ModeState temp;
    EEPROM_ReadBuffer(0x0200, (uint8_t*)&temp, sizeof(temp));
    if (mode > 2) mode = 0;
    temp.power_restore_mode = mode;
    EEPROM_WriteBuffer(0x0200, (uint8_t*)&temp, sizeof(temp));
}

void ModelHandle_SetDryRunTime(uint32_t seconds)
{
    if (seconds < 10)    seconds = 10;
    if (seconds > 10800) seconds = 10800;
    sys.dry_run_time_s = seconds;
    ModelHandle_SaveSettingsToEEPROM();
}

void ModelHandle_ToggleManual(void)
{
    /* Manual restored after power-up with its motor held off: the
     * manual button is the on key, so first press starts the motor. */
    if (manualActive && manualMotorOff && manualFromRestore)
    {
        manualFromRestore  = false;
        manualMotorOff     = false;
        senseMaxRunReached = false;
        start_motor();
        ModelHandle_SaveModeState();
        return;
    }

    if (!manualActive)
    {
        manualMotorOff  = false;
        powerRestoreHold = false;
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
        autoPausedByUser   = false;
        manualPausedByUser = false;

        start_motor();
    }
    else
    {
        manualActive   = false;
        manualOverride = false;
        motorOwner     = MOTOR_OWNER_NONE;
        dryState       = DRY_IDLE;
        manualPausedByUser = true;   /* dashboard still shows MANUAL, motor off */

        stop_motor();
    }

    ModelHandle_SaveModeState();
}

void ModelHandle_ManualToggleMotor(void)
{
    if (!manualActive) return;
    manualFromRestore = false;
    manualMotorOff = Motor_GetStatus();
    if (manualMotorOff) stop_motor();
    else                start_motor();
    ModelHandle_SaveModeState();
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
    if (!timerActive)
    {
        clear_all_modes();
        timerActive        = true;
        motorOwner         = MOTOR_OWNER_TIMER;
        timerState         = TIMER_RUN_TEST;
        timerStateDeadline = 0;
        timer_retry_count  = 0;
        senseMaxRunReached = false;
        dryState           = DRY_IDLE;
        ModelHandle_ProcessTimerSlots();
        ModelHandle_SendTimerInfoUART();
    }
    else
    {
        timerActive = false;
        motorOwner  = MOTOR_OWNER_NONE;
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
    restartActive = false;
    clear_all_modes();
    stop_motor();
    dryState           = DRY_IDLE;
    senseOverLoad      = false;
    senseUnderLoad     = false;
    senseOverUnderVolt = false;
    senseMaxRunReached = false;
    manualMotorOff     = false;
    powerRestoreHold   = false;
    autoUserLocked     = false;

    sys.gap_time_s     = FACTORY_DRY_TEST_S;
    sys.retry_count    = FACTORY_RETRY_COUNT;
    sys.uv_limit       = FACTORY_UV;
    sys.ov_limit       = FACTORY_OV;
    sys.overload       = FACTORY_OVERLOAD_A;
    sys.underload      = 0.0f;
    sys.maxrun_min     = FACTORY_MAXRUN_MIN;
    sys.dry_run_time_s = FACTORY_TEST_GAP_S;
    sys.dry_run_enable = 0;
    ModelHandle_SaveSettingsToEEPROM();

    powerRestoreMode = FACTORY_PWR_RESTORE;
    ModelHandle_SaveModeState();

    buzzerSettings.pumpOnSound    = 0;
    buzzerSettings.tankFullSound  = 0;
    buzzerSettings.tankEmptySound = 0;
    ModelHandle_SaveBuzzerSettings();

    memset(timerSlots, 0, sizeof(timerSlots));
    ModelHandle_SaveTimerToEEPROM();
    memset(&twistSettings, 0, sizeof(twistSettings));
    ModelHandle_SaveTwistToEEPROM();
    ModelHandle_SetCountdownDefaultMin(FACTORY_COUNTDOWN_MIN);
}

void ModelHandle_StartTimerNearestSlot(void)
{
    clear_all_modes();
    timerActive        = true;
    motorOwner         = MOTOR_OWNER_TIMER;
    timerState         = TIMER_RUN_TEST;
    timerStateDeadline = 0;
    timer_retry_count  = 0;
    senseMaxRunReached = false;
    dryState           = DRY_IDLE;
    ModelHandle_ProcessTimerSlots();
    ModelHandle_SendTimerInfoUART();
    ModelHandle_SaveModeState();
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

#define TANK_FULL_BUZZ_MS 30000UL

static void Buzzer_TankEmptyPattern(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - buzzerPatternStart) >= 15000UL) { Buzzer_SetPin(false); activeBuzzEvent = BUZZ_NONE; return; }
    Buzzer_SetPin(true);
}

static void Buzzer_TankFullPattern(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - buzzerPatternStart) >= TANK_FULL_BUZZ_MS) { Buzzer_SetPin(false); activeBuzzEvent = BUZZ_NONE; return; }
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
        if (tankFull && !prevTankFullManual && buzzerSettings.tankFullSound)
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
    autoGroundWaterRun  = false;
    timerGroundWaterRun = false;
    manualMotorOff      = false;
    manualFromRestore   = false;
    powerRestoreHold    = false;   /* a user-started mode is an on key */
    autoPausedByUser    = false;   /* another mode is now the last mode */
    manualPausedByUser  = false;
}

void ModelHandle_OnPowerUp(void)
{
    powerOnMs           = HAL_GetTick();
    autoBootIgnoreUntil = powerOnMs + 15000UL;
    bootStartBlockUntil = powerOnMs + MOTOR_START_DELAY_MS;
    autoDeadline        = 0;
    dryState            = DRY_IDLE;
    ModelHandle_LoadBuzzerSettings();

    /* After a power cut the last mode always comes back (client Key
     * Story; the Power Restore setting was removed - decisions Q3/Q4):
     *   Auto: motor starts (unless the tank is full).
     *   Timer: starts if the clock is inside an active slot.
     *   Twist: continues the period it was in with the time left, if its
     *          end time has not passed; otherwise off.
     *   Countdown: motor on for the time left.
     *   Semi-Auto: runs again (until tank full) if its motor was running.
     *   Manual: comes back with the motor off (power off is its off key).
     *   Refill is not a mode: the mode it was started from applies. */
    bool wasRunning = modeState.motor_on;
    if (modeState.countdown_on)
    {
        ModelHandle_LoadCountdown();       /* resumes with the time left */
    }
    bool runNow = wasRunning;
    powerRestoreHold      = !runNow;
    powerRestoreHoldUntil = powerOnMs + get_test_gap_ms();

    if (countdownActive)
    {
        powerRestoreHold = false;          /* motor runs for the time left */
    }
    else if (manualActive)
    {
        motorOwner        = MOTOR_OWNER_MANUAL;
        manualOverride    = true;
        manualMotorOff    = true;
        manualFromRestore = true;
        powerRestoreHold  = false;
    }
    else if (semiAutoActive)
    {
        /* Semi-Auto runs again only if its motor was on at power loss */
        semiAutoActive   = wasRunning;
        motorOwner       = wasRunning ? MOTOR_OWNER_SEMIAUTO : MOTOR_OWNER_NONE;
        semiTankFullHold = false;
        powerRestoreHold = false;
    }
    else if (timerActive)
    {
        /* Inside an active slot the timer starts; outside it waits */
        motorOwner         = MOTOR_OWNER_TIMER;
        timerState         = TIMER_RUN_TEST;
        timerStateDeadline = 0;
        powerRestoreHold   = false;
    }
    else if (twistActive)
    {
        motorOwner = MOTOR_OWNER_TWIST;
        twist_reset_sensors();
        if (!twist_load_run())          /* continue the period with its time left */
        {
            twist_deadline = 0;
            twist_on_phase = runNow;
        }
        powerRestoreHold = false;
    }
    else if (autoActive)
    {
        /* Auto: power restore is an on key whatever the motor was doing -
         * start regardless of the 50% gate. auto_tick() still stops it
         * straight away if the tank is full. */
        motorOwner          = MOTOR_OWNER_AUTO;
        autoState           = AUTO_ON_WAIT;
        stateDeadline       = 0;
        autoRestoreOverride = true;
        powerRestoreHold    = false;
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
            Relay_Set(3, false);                 /* never push start and stop together */
            starterOffPulseEnd = 0;
            Relay_Set(2, true);
            starterOnPulseEnd = motorOnStartMs + STARTER_PULSE_MS;
        }
        motorStatus = 1;
    }
    else
    {
        if (relayNow)
        {
            Relay_Set(1, false);
            Relay_Set(2, false);
            starterOnPulseEnd = 0;
            Relay_Set(3, true);
            starterOffPulseEnd = HAL_GetTick() + STARTER_PULSE_MS;
        }
        motorStatus = 0;
    }
    /* Restore ON needs the motor state at the moment power fails */
    if (relayNow != on && powerRestoreMode == 0 && Motor_IsRelayOn() == on)
        ModelHandle_SaveModeState();
    UART_SendStatusPacket();
}

static void starter_relays_tick(void)
{
    uint32_t now = HAL_GetTick();
    if (starterOnPulseEnd && (int32_t)(now - starterOnPulseEnd) >= 0)
    {
        Relay_Set(2, false);
        starterOnPulseEnd = 0;
    }
    if (starterOffPulseEnd && (int32_t)(now - starterOffPulseEnd) >= 0)
    {
        Relay_Set(3, false);
        starterOffPulseEnd = 0;
    }
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

static uint32_t maxRunLockUntil = 0;

/* Max run is an off key in every mode. Refill and Countdown end (the
 * previous mode continues), Semi-Auto ends, Manual stays shown with the
 * motor off; Auto/Timer/Twist may run again after the testing gap. */
static void check_max_run(void)
{
    uint32_t now = HAL_GetTick();

    if (senseMaxRunReached && maxRunLockUntil && !Motor_GetStatus() &&
        (int32_t)(now - maxRunLockUntil) >= 0)
    {
        senseMaxRunReached = false;
        maxRunLockUntil    = 0;
    }

    if (sys.maxrun_min == 0 || !Motor_GetStatus()) return;
    uint32_t limit = (uint32_t)sys.maxrun_min * 60000UL;
    if ((now - motorOnStartMs) < limit) return;

    senseMaxRunReached = true;
    maxRunLockUntil    = now + get_test_gap_ms();
    if (maxRunLockUntil == 0) maxRunLockUntil = 1;
    stop_motor();
    dryState = DRY_IDLE;

    if (restartActive)   ModelHandle_StopRestart();
    if (countdownActive) ModelHandle_StopCountdown();
    if (manualActive)
    {
        manualActive       = false;
        manualPausedByUser = true;      /* still Manual, motor off */
    }
    manualOverride = false;
    manualMotorOff = false;
    semiAutoActive = false;
    ModelHandle_SaveModeState();
}

void ModelHandle_Button4_SinglePress(void)
{
    if (!countdownActive)
    {
        clear_all_modes();
        countdownActive  = true;
        countdownMode    = true;
        motorOwner       = MOTOR_OWNER_COUNTDOWN;
        uint32_t defaultSeconds = (uint32_t)ModelHandle_GetCountdownDefaultMin() * 60UL;
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

/* Ground-water and dry-run inputs: water pulls the input LOW (probe to
 * COM = 0 V; an open input floats 1.2-2.0 V). Small hysteresis. */
#define SENSOR_WET_V          0.30f
#define SENSOR_DRY_V          0.50f

/* senseDryRun = water present at the dry-run sensor */
void ModelHandle_CheckDryRun(void)
{
    float v = adcData.voltages[ADC_IDX_DRY_RUN];
    senseDryRun = senseDryRun ? (v < SENSOR_DRY_V) : (v < SENSOR_WET_V);
}

#define GROUND_SENSE_DELAY_MS 10000UL

void ModelHandle_CheckGroundWater(void)
{
    static uint32_t changeStart    = 0;
    static bool     candidateState = false;
    static bool     stableState    = false;

    /* The G.W terminal is wired to IN5 (it was read from IN4, which is
     * really the dry-run terminal - so G.W always showed NO). */
    float    v        = adcData.voltages[ADC_IDX_GROUND_W];
    bool     detected = stableState ? (v < SENSOR_DRY_V) : (v < SENSOR_WET_V);
    uint32_t now      = HAL_GetTick();

    if (detected != candidateState)
    {
        /* Reading just flipped direction - restart the qualification
         * window for this new candidate. A brief noise blip that
         * doesn't hold for the full delay never reaches stableState. */
        candidateState = detected;
        changeStart    = now;
    }

    if ((now - changeStart) >= GROUND_SENSE_DELAY_MS)
        stableState = candidateState;

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
    /* Timer, Auto and Twist run their own dry-run test */
    if (motorOwner == MOTOR_OWNER_TIMER || motorOwner == MOTOR_OWNER_AUTO ||
        motorOwner == MOTOR_OWNER_TWIST) return;
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

#define LOAD_INRUSH_IGNORE_MS 3000UL    /* ignore start-up current surge  */
#define VOLT_RECOVER_MS       10000UL   /* supply back in range this long */

static uint32_t loadBadSince  = 0;
static uint32_t loadLockUntil = 0;
static uint32_t voltBadSince  = 0;
static uint32_t voltGoodSince = 0;

/* Voltage is watched all the time, so a bad supply also blocks a start;
 * the fault clears once the supply is back in range for 10 s.
 * Over/under current is watched while running (after the inrush); it
 * trips after 3 s, and the motor may start again after the testing gap. */
void ModelHandle_CheckLoadFault(void)
{
    uint32_t now     = HAL_GetTick();
    bool     motorOn = Motor_GetStatus();
    float    I       = g_currentA;
    float    V       = g_voltageV;

    bool vBad = (sys.uv_limit && V < sys.uv_limit) ||
                (sys.ov_limit && V > sys.ov_limit);
    if (vBad)
    {
        voltGoodSince = 0;
        if (!voltBadSince) voltBadSince = now ? now : 1;
        if (!senseOverUnderVolt && (now - voltBadSince) >= LOAD_FAULT_CONFIRM_MS)
        {
            senseOverUnderVolt = true;
            stop_motor();
        }
    }
    else
    {
        voltBadSince = 0;
        if (senseOverUnderVolt)
        {
            if (!voltGoodSince) voltGoodSince = now ? now : 1;
            if ((now - voltGoodSince) >= VOLT_RECOVER_MS)
            {
                senseOverUnderVolt = false;
                voltGoodSince      = 0;
            }
        }
    }

    bool over = false, under = false;
    if (motorOn && (now - motorOnStartMs) >= LOAD_INRUSH_IGNORE_MS)
    {
        over  = (sys.overload  > 0.1f)   && (I > sys.overload);
        under = (sys.underload > 0.001f) && (I < sys.underload);
    }
    if (over || under)
    {
        if (!loadBadSince) loadBadSince = now ? now : 1;
        if ((now - loadBadSince) >= LOAD_FAULT_CONFIRM_MS)
        {
            senseOverLoad  = over;
            senseUnderLoad = under;
            stop_motor();
            loadLockUntil = now + get_test_gap_ms();
            loadBadSince  = 0;
        }
    }
    else loadBadSince = 0;

    if ((senseOverLoad || senseUnderLoad) && !motorOn &&
        (int32_t)(now - loadLockUntil) >= 0)
    {
        senseOverLoad  = false;
        senseUnderLoad = false;
    }
}

/* RTC day-of-week is 1=Sun..7=Sat (as shown on the LCD), while slot
 * dayMask from the app is bit0=Mon..bit6=Sun. */
static uint8_t get_today_mask(void)
{
    uint8_t d = time.dow;
    if (d < 1 || d > 7) d = 1;
    return (d == 1) ? (1U << 6) : (1U << (d - 2));
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
        buzzerSettings.pumpOnSound    = 0;
        buzzerSettings.tankFullSound  = 0;
        buzzerSettings.tankEmptySound = 0;
        ModelHandle_SaveBuzzerSettings();
        return;
    }
    buzzerSettings.pumpOnSound    = b.pumpOnSound    ? 1 : 0;
    buzzerSettings.tankFullSound  = b.tankFullSound  ? 1 : 0;
    buzzerSettings.tankEmptySound = b.tankEmptySound ? 1 : 0;
}

static void timer_begin_run(uint32_t now, bool dryEn)
{
    motorOwner         = MOTOR_OWNER_TIMER;
    timerState         = dryEn ? TIMER_RUN_TEST : TIMER_RUN_CONTINUOUS;
    timerStateDeadline = dryEn ? now + get_dry_test_ms() : 0;
    dryState           = dryEn ? DRY_WAITING : DRY_IDLE;
    start_motor();
}

void ModelHandle_ProcessTimerSlots(void)
{
    if (!timerActive) return;
    if (motorOwner != MOTOR_OWNER_TIMER)
    {
        return;
    }
    ModelHandle_CheckGroundWater();
    ModelHandle_CheckDryRun();
    uint32_t now     = HAL_GetTick();
    bool     motorOn = Motor_GetStatus();

    /* Edges are tracked every tick so a ground-water change outside a
     * slot (report example 4) is not replayed when the slot opens. */
    bool gwRising  = (groundWater && !prevGroundWaterTimer);
    bool gwFalling = (!groundWater && prevGroundWaterTimer);
    prevGroundWaterTimer = groundWater;

    if (!timer_any_active_slot())
    {
        stop_motor();
        dryState            = DRY_IDLE;
        timerState          = TIMER_RUN_TEST;
        timerStateDeadline  = 0;
        timer_retry_count   = 0;
        timerGroundWaterRun = false;
        powerRestoreHold    = false;   /* next slot start is a fresh on key */
        return;
    }
    if (isTankFull())
    {
        stop_motor();
        timerState          = TIMER_RUN_TEST;
        timerStateDeadline  = 0;
        dryState            = DRY_IDLE;
        timer_retry_count   = 0;
        timerGroundWaterRun = false;
        return;
    }
    if (senseOverLoad || senseUnderLoad || senseOverUnderVolt ||
        senseMaxRunReached)
    {
        stop_motor();
        timerGroundWaterRun = false;
        if (timerState != TIMER_WAIT_RETRY)
        {
            timerState         = TIMER_RUN_TEST;
            timerStateDeadline = 0;
        }
        return;
    }
    if (restore_hold_active())
    {
        if (!gwRising) { stop_motor(); return; }
        powerRestoreHold = false;
    }

    bool dryEn = dry_protection_enabled();

    /* Ground water is its own on/off key, whatever the dry-run setting */
    if (gwRising && !motorOn)
    {
        timerGroundWaterRun = true;
        timer_retry_count   = 0;
        timer_begin_run(now, dryEn);
        return;
    }
    /* Ground water lost is not an off key (client decision Q2-b) */
    (void)gwFalling;

    switch (timerState)
    {
        case TIMER_RUN_TEST:
            if (!dryEn || timerStateDeadline == 0)
            {
                timer_begin_run(now, dryEn);
                break;
            }
            start_motor();
            if ((int32_t)(now - timerStateDeadline) < 0) break;
            if (senseDryRun)
            {
                timerState          = TIMER_RUN_CONTINUOUS;
                dryState            = DRY_IDLE;
                timer_retry_count   = 0;
                timerGroundWaterRun = false;   /* confirmed by real sensor now */
            }
            else
            {
                timer_retry_count++;
                timerState         = TIMER_WAIT_RETRY;
                timerStateDeadline = retries_exhausted(timer_retry_count)
                                     ? 0 : now + get_test_gap_ms();
                dryState           = DRY_FAULT;
                stop_motor();
            }
            break;

        case TIMER_WAIT_RETRY:
            /* Testing gap: motor off, then test again unconditionally.
             * Deadline 0 = retries used up, wait for next slot / GW. */
            stop_motor();
            if (timerStateDeadline == 0) break;
            if ((int32_t)(now - timerStateDeadline) < 0) break;
            timerState         = TIMER_RUN_TEST;
            timerStateDeadline = 0;
            dryState           = DRY_IDLE;
            break;

        case TIMER_RUN_CONTINUOUS:
            dryState = DRY_IDLE;
            if (dryEn && !senseDryRun)
            {
                timerState         = TIMER_WAIT_RETRY;
                timerStateDeadline = now + get_test_gap_ms();
                dryState           = DRY_FAULT;
                stop_motor();
                break;
            }
            start_motor();
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
    motorOwner         = MOTOR_OWNER_NONE;
    timerGroundWaterRun = false;
    stop_motor();
    ModelHandle_SaveModeState();
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
    autoRestoreOverride = false;
    autoGroundWaterRun  = false;
    autoFilledLatch     = false;
    prevGroundWaterAuto = groundWater;
    autoActive       = true;
    autoState        = AUTO_ON_WAIT;
    stateDeadline    = 0;
    motorOwner       = MOTOR_OWNER_AUTO;
    dryState         = DRY_IDLE;

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
    autoGroundWaterRun = false;
    autoRestoreOverride = false;
    autoPausedByUser   = true;   /* dashboard still shows AUTO, motor off */
    manualPausedByUser = false;

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
    if (manualPausedByUser || restartActive) return;   /* still Manual (motor off) / Refill running */

    uint8_t level = get_tank_level_percent();
    if (!autoActive && restore_hold_active())
    {
        if (level > AUTO_START_LEVEL_PERCENT) powerRestoreHold = false;
        return;
    }
    if (!autoActive &&
        level <= AUTO_START_LEVEL_PERCENT)
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

uint8_t ModelHandle_GetBuzzerPumpOnSound(void)    { return buzzerSettings.pumpOnSound; }
uint8_t ModelHandle_GetBuzzerTankFullSound(void)  { return buzzerSettings.tankFullSound; }
uint8_t ModelHandle_GetBuzzerTankEmptySound(void) { return buzzerSettings.tankEmptySound; }

static void auto_begin_run(uint32_t now, bool dryEn)
{
    motorOwner    = MOTOR_OWNER_AUTO;
    autoState     = AUTO_OFF_WAIT;
    stateDeadline = dryEn ? now + get_dry_test_ms() : 0;   /* 0 = no dry test pending */
    dryState      = dryEn ? DRY_WAITING : DRY_IDLE;
    start_motor();
}

/*
 * Auto mode:
 *   AUTO_ON_WAIT   motor off; starts when level <= 50% (or <= 75% with
 *                  ground water present, or power restore override).
 *   AUTO_OFF_WAIT  motor running. With dry run enabled the first
 *                  dry-test window decides: water -> keep running,
 *                  no water -> stop and wait the testing gap.
 *   AUTO_DRY_CHECK motor off for the testing gap, then test again.
 *                  After "retry count" failed tests it waits for the
 *                  next on key (ground water, tank level cycle, user).
 * Ground water is an independent on/off key: detected -> start (up to
 * 75%), disconnected -> stop and wait the testing gap. Both edges are
 * already delayed 10 s by ModelHandle_CheckGroundWater().
 */
static void auto_tick(void)
{
    if (!autoActive) return;

    uint32_t now = HAL_GetTick();

    ModelHandle_CheckDryRun();
    ModelHandle_CheckGroundWater();

    uint8_t level   = get_tank_level_percent();
    bool    motorOn = Motor_GetStatus();

    bool gwRising  = (groundWater && !prevGroundWaterAuto);
    bool gwFalling = (!groundWater && prevGroundWaterAuto);
    prevGroundWaterAuto = groundWater;

    if (senseOverLoad || senseUnderLoad || senseOverUnderVolt ||
        senseMaxRunReached)
    {
        stop_motor();
        if (autoState != AUTO_DRY_CHECK)
        {
            autoState     = AUTO_ON_WAIT;
            stateDeadline = 0;
        }
        autoGroundWaterRun = false;
        return;
    }

    if (level >= AUTO_STOP_LEVEL_PERCENT)
    {
        stop_motor();
        autoState          = AUTO_ON_WAIT;
        stateDeadline      = 0;
        dryState           = DRY_IDLE;
        autoGroundWaterRun = false;
        auto_retry_count   = 0;
        autoFilledLatch    = true;
        return;
    }

    if (restore_hold_active())
    {
        /* Restore OFF: wait for a fresh on key - ground water, the
         * level rising above 50% so a later fall is a new trigger, or
         * the testing gap running out (restore_hold_active). */
        if (gwRising || level > AUTO_START_LEVEL_PERCENT)
            powerRestoreHold = false;
        else
        {
            stop_motor();
            return;
        }
    }

    bool dryEn = dry_protection_enabled();

    if (gwRising && !motorOn && level <= GW_START_LEVEL_PERCENT)
    {
        autoGroundWaterRun  = true;
        auto_retry_count    = 0;
        autoRestoreOverride = false;
        auto_begin_run(now, dryEn);
        return;
    }
    /* Ground water lost is not an off key (client decision Q2-b) */
    (void)gwFalling;

    switch (autoState)
    {
        case AUTO_OFF_WAIT:
        {
            start_motor();

            if (!dryEn || stateDeadline == 0)
            {
                stateDeadline = 0;
                dryState      = DRY_IDLE;
                break;
            }
            if ((int32_t)(now - stateDeadline) < 0) break;

            ModelHandle_CheckDryRun();
            if (senseDryRun)
            {
                /* Water confirmed: keep running until full */
                stateDeadline      = 0;
                dryState           = DRY_IDLE;
                autoGroundWaterRun = false;
                auto_retry_count   = 0;
            }
            else
            {
                stop_motor();
                auto_retry_count++;
                autoState     = AUTO_DRY_CHECK;
                stateDeadline = retries_exhausted(auto_retry_count)
                                ? 0 : now + get_test_gap_ms();
                dryState      = DRY_FAULT;
            }
            break;
        }

        case AUTO_DRY_CHECK:
        {
            /* Testing gap: motor off, then test again unconditionally.
             * Deadline 0 = retries used up, wait for next on key. */
            stop_motor();
            if (stateDeadline == 0) break;
            if ((int32_t)(now - stateDeadline) < 0) break;

            autoState     = AUTO_ON_WAIT;
            stateDeadline = 0;
            dryState      = DRY_IDLE;
            break;
        }

        case AUTO_IDLE:
        case AUTO_ON_WAIT:
        default:
        {
            /* After Auto has filled the tank it refills only once the level
             * is down to 25% (a fresh ground-water detection, handled
             * above, still starts it). Otherwise the start level is 50%. */
            /* After Auto has filled the tank, ground water that simply
             * stays connected does not restart it at 75% (a fresh
             * detection, handled above, still does). */
            bool gwStart = !autoFilledLatch && groundWater &&
                           level <= GW_START_LEVEL_PERCENT;
            if (level > AUTO_START_LEVEL_PERCENT && !autoRestoreOverride && !gwStart)
            {
                stop_motor();
                autoState     = AUTO_ON_WAIT;
                dryState      = DRY_IDLE;
                stateDeadline = 0;
                break;
            }
            autoRestoreOverride = false;
            autoFilledLatch     = false;
            auto_begin_run(now, dryEn);
            break;
        }
    }
}

void ModelHandle_StartCountdown(uint32_t seconds)
{
    if (seconds < 60)    seconds = 60;
    if (seconds > 10800) seconds = 10800;

    /* Remember the running mode; it continues when the countdown ends.
     * (Restarting a running countdown keeps the original one.) */
    if (!countdownActive) backup_current_mode();
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
    SaveCountdown();
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
    SaveCountdown();

    /* However it ends (time over, user stop, fault), the mode that was
     * running before the countdown continues. Never "return" into a
     * countdown (possible if a Refill was started during it). */
    previousCountdown = false;
    restore_previous_mode();
    ModelHandle_SaveModeState();
}

bool ModelHandle_IsAutoPausedByUser(void)   { return autoPausedByUser; }
bool ModelHandle_IsManualPausedByUser(void) { return manualPausedByUser; }
/* Countdown duration used by the device button (single press); set from
 * the long-press edit screen or the app, kept in EEPROM. */
#define EE_ADDR_CD_DEFAULT 0x0060
#define CD_DEFAULT_SIG     0xCD
static uint16_t cdDefaultMin = FACTORY_COUNTDOWN_MIN;

uint16_t ModelHandle_GetCountdownDefaultMin(void) { return cdDefaultMin; }

void ModelHandle_SetCountdownDefaultMin(uint16_t minutes)
{
    if (minutes < 1)   minutes = 1;
    if (minutes > 180) minutes = 180;
    cdDefaultMin = minutes;
    uint8_t b[4] = { CD_DEFAULT_SIG, (uint8_t)minutes, (uint8_t)(minutes >> 8), 0 };
    b[3] = (uint8_t)(b[0] ^ b[1] ^ b[2]);
    EEPROM_WriteBuffer(EE_ADDR_CD_DEFAULT, b, sizeof(b));
}

void ModelHandle_LoadCountdownDefault(void)
{
    uint8_t b[4];
    EEPROM_ReadBuffer(EE_ADDR_CD_DEFAULT, b, sizeof(b));
    uint16_t minutes = (uint16_t)(b[1] | (b[2] << 8));
    if (b[0] != CD_DEFAULT_SIG || b[3] != (uint8_t)(b[0] ^ b[1] ^ b[2]) ||
        minutes < 1 || minutes > 180)
    {
        ModelHandle_SetCountdownDefaultMin(FACTORY_COUNTDOWN_MIN);
        return;
    }
    cdDefaultMin = minutes;
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

/* Power restore ON: continue a countdown that was running at power loss
 * with the time it had left (saved every minute, so at most ~1 min is
 * repeated). The time starts counting when the motor may start again. */
void ModelHandle_LoadCountdown(void)
{
    CountdownBlock b;
    EEPROM_ReadBuffer(EE_ADDR_COUNTDOWN_BLOCK, (uint8_t*)&b, sizeof(b));
    if (b.sig != CD_SIGNATURE) return;
    if (CD_CRC((uint8_t*)&b, sizeof(b) - 2) != b.crc) return;
    if (b.active && b.remaining > 0 && b.remaining <= 10800UL)
    {
        countdownActive   = true;
        countdownMode     = true;
        countdownDuration = b.remaining;
        cd_deadline       = bootStartBlockUntil + b.remaining * 1000UL;
        cdTankFullHold    = false;
        motorOwner        = MOTOR_OWNER_COUNTDOWN;
        load_previous_mode();              /* mode to return to at the end */
    }
}

/* Countdown time is over (StopCountdown returns to the previous mode) */
static void countdown_finished(void)
{
    ModelHandle_StopCountdown();
}

/* Twist with dry run:
 *   ON period  - motor on for the on duration. With dry run enabled the
 *                first "dry run" minutes are a test: no water -> motor off
 *                for the rest of this ON period (twist keeps going; the
 *                next ON period tests again). If the ON period is shorter
 *                than the test, the check is made at its end.
 *   OFF period - motor off for the off duration.
 *   Ground water is not used in Twist.
 *   Tank full: paused until the level is down to 25%. */
static uint32_t twistDryDeadline     = 0;      /* end of this ON period's dry test, 0 = none */
static bool     twistDryFailed       = false;  /* no water: off for the rest of this ON period */
static bool     twistFilledLatch     = false;  /* tank filled: wait for <= 25% */

static void twist_begin_phase(uint32_t now, bool onPhase)
{
    uint32_t onMs  = (uint32_t)twistSettings.onDurationSeconds  * 1000UL;
    uint32_t offMs = (uint32_t)twistSettings.offDurationSeconds * 1000UL;

    twist_on_phase   = onPhase;
    twist_deadline   = now + (onPhase ? onMs : offMs);
    if (twist_deadline == 0) twist_deadline = 1;
    twistDryFailed   = false;
    twistDryDeadline = 0;
    dryState         = DRY_IDLE;

    if (onPhase && dry_protection_enabled())
    {
        uint32_t testMs = get_dry_test_ms();
        if (testMs > onMs) testMs = onMs;
        twistDryDeadline = now + testMs;
        if (twistDryDeadline == 0) twistDryDeadline = 1;
        dryState = DRY_WAITING;
    }
    twist_save_run();
}

/* Twist period (ON/OFF) and its time left, kept in EEPROM at every period
 * start and once a minute, so after a power cut twist continues the same
 * period with the time it had left (up to 1 min may be repeated). */
#define EE_ADDR_TWIST_RUN  0x0074
#define TWIST_RUN_SIG      0x7A

static void twist_save_run(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t left = 0;
    if (twist_deadline && (int32_t)(twist_deadline - now) > 0)
        left = (twist_deadline - now) / 1000UL;
    if (left > 0xFFFF) left = 0xFFFF;
    uint8_t b[5] = { TWIST_RUN_SIG, (uint8_t)(twist_on_phase ? 1 : 0),
                     (uint8_t)left, (uint8_t)(left >> 8), 0 };
    b[4] = (uint8_t)(b[0] ^ b[1] ^ b[2] ^ b[3]);
    EEPROM_WriteBuffer(EE_ADDR_TWIST_RUN, b, sizeof(b));
}

static bool twist_load_run(void)
{
    uint8_t b[5];
    EEPROM_ReadBuffer(EE_ADDR_TWIST_RUN, b, sizeof(b));
    if (b[0] != TWIST_RUN_SIG || b[4] != (uint8_t)(b[0] ^ b[1] ^ b[2] ^ b[3]) || b[1] > 1)
        return false;
    uint32_t left = (uint32_t)b[2] | ((uint32_t)b[3] << 8);
    if (left == 0) return false;
    twist_on_phase = (b[1] != 0);
    twist_deadline = bootStartBlockUntil + left * 1000UL;   /* time counts once the motor may start */
    if (twist_deadline == 0) twist_deadline = 1;
    return true;
}

static void twist_reset_sensors(void)
{
    twistDryDeadline     = 0;
    twistDryFailed       = false;
    twistFilledLatch     = false;   /* starting twist: normal cycle unless full */
}

void ModelHandle_StartTwist(uint16_t on_s, uint16_t off_s,
                            uint8_t onH, uint8_t onM,
                            uint8_t offH, uint8_t offM)
{
    twist_reset_sensors();
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
    twist_deadline = 0;
    dryState       = DRY_IDLE;
    ModelHandle_SaveTwistToEEPROM();
    ModelHandle_SaveModeState();
    twist_tick();
}

/* Start twist with the settings last saved (app TWIST:ON, device menu) */
void ModelHandle_ResumeTwist(void)
{
    uint16_t onMin  = twistSettings.onDurationSeconds  / 60U;
    uint16_t offMin = twistSettings.offDurationSeconds / 60U;
    if (onMin  == 0) onMin  = 5;
    if (offMin == 0) offMin = 5;
    ModelHandle_StartTwist(onMin, offMin,
                           twistSettings.onHour,  twistSettings.onMinute,
                           twistSettings.offHour, twistSettings.offMinute);
}

void ModelHandle_StopTwist(void)
{
    twistActive               = false;
    twistSettings.twistActive = false;
    dryState                  = DRY_IDLE;
    stop_motor();
    ModelHandle_SaveModeState();
}

/* A start time equal to the stop time means "no schedule restriction" -
 * keeps plain TWIST ON (which passes 0,0,0,0) behaving as always-on,
 * while TWIST SET with two different times gates it to that window. */
static bool twist_window_active(void)
{
    uint16_t onT  = (uint16_t)twistSettings.onHour  * 60U + twistSettings.onMinute;
    uint16_t offT = (uint16_t)twistSettings.offHour * 60U + twistSettings.offMinute;
    uint16_t now  = (uint16_t)time.hour * 60U + time.min;

    if (onT == offT) return true;
    if (onT < offT)  return (now >= onT && now < offT);
    return (now >= onT || now < offT);
}

static void twist_tick(void)
{
    if (!twistActive) return;
    if (motorOwner != MOTOR_OWNER_TWIST) return;

    ModelHandle_CheckDryRun();

    /* Tank full: twist pauses and restarts only once the level is down
     * to 25% (same rule as Auto). */
    uint8_t level = get_tank_level_percent();
    if (level >= AUTO_STOP_LEVEL_PERCENT)        twistFilledLatch = true;
    else if (level <= AUTO_REFILL_LEVEL_PERCENT) twistFilledLatch = false;

    /* Outside the twist time window, or waiting after tank full: motor
     * off. The cycle restarts with an ON period afterwards. (The full
     * buzzer is handled centrally.) */
    if (!twist_window_active() || twistFilledLatch)
    {
        stop_motor();
        twist_deadline   = 0;
        twist_on_phase   = true;
        twistDryDeadline = 0;
        twistDryFailed   = false;
        dryState         = DRY_IDLE;
        return;
    }

    uint32_t now = now_ms();

    /* Ground water is not used in Twist (client decision Q2-b and the
     * Key Story: twist on key is its own time only). */

    if (twist_deadline == 0)
        twist_begin_phase(now, twist_on_phase);

    /* Dry-run test at the start of each ON period (if enabled) */
    if (twist_on_phase && twistDryDeadline &&
        (int32_t)(now - twistDryDeadline) >= 0)
    {
        twistDryDeadline = 0;
        if (senseDryRun) dryState = DRY_IDLE;                        /* water: keep running */
        else { twistDryFailed = true; dryState = DRY_FAULT; }        /* no water: off till period ends */
    }

    if ((int32_t)(now - twist_deadline) >= 0)
        twist_begin_phase(now, !twist_on_phase);

    {
        static uint32_t lastTwistSave = 0;
        if ((now - lastTwistSave) >= 60000UL) { lastTwistSave = now; twist_save_run(); }
    }

    if (twist_on_phase && !twistDryFailed) start_motor(); else stop_motor();
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
            /* Voltage / current fault is an off key: Refill ends and the
             * mode it was started from continues. */
            if (protectionFault) { ModelHandle_StopRestart(); break; }
            start_motor();
            break;

        case MOTOR_OWNER_MANUAL:
            /* Protection trips are off keys for manual: the motor stops and
             * the device stays in Manual (shown as MANUAL, motor off); only
             * the user's press starts it again. */
            if (protectionFault)
            {
                stop_motor();
                manualActive       = false;
                manualOverride     = false;
                manualMotorOff     = false;
                manualPausedByUser = true;
                ModelHandle_SaveModeState();
            }
            else if (manualMotorOff) stop_motor();
            else                     start_motor();
            break;

        case MOTOR_OWNER_SEMIAUTO:
            if (protectionFault)
            {
                ModelHandle_StopSemiAuto();    /* fault is an off key: run ends */
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
            if ((int32_t)(now - cd_deadline) >= 0) { countdown_finished(); break; }
            countdownDuration = (cd_deadline - now) / 1000UL;
            {
                /* Time left is saved once a minute (EEPROM wear) so a
                 * power cut can resume it. */
                static uint32_t lastCdSave = 0;
                if ((now - lastCdSave) >= 60000UL) { lastCdSave = now; SaveCountdown(); }
            }
            /* Fault and tank full are off keys: the countdown ends and the
             * previous mode continues. */
            if (protectionFault || tankFull) { ModelHandle_StopCountdown(); break; }
            start_motor();
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

    starter_relays_tick();
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
