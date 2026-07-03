#ifndef MODEL_HANDLE_H
#define MODEL_HANDLE_H
#include <stdint.h>
#include <stdbool.h>
#include "adc.h"
#include "eeprom_i2c.h"
typedef struct __attribute__((packed)) {
    uint8_t onHour;
    uint8_t onMinute;
    uint8_t offHour;
    uint8_t offMinute;
    uint8_t dayMask;
    uint8_t enabled;
} TimerSlot;
typedef struct
{
    uint32_t gap_time_s;
    uint8_t  retry_count;
    uint16_t uv_limit;
    uint16_t ov_limit;
    float    overload;
    float    underload;
    uint16_t maxrun_min;
    uint16_t dry_run_time_s;
    uint8_t  dry_run_enable;
} SystemSettings;
extern SystemSettings sys;
extern TimerSlot timerSlots[5];
typedef struct {
    uint16_t onDurationSeconds;
    uint16_t offDurationSeconds;
    uint8_t  onHour;
    uint8_t  onMinute;
    uint8_t  offHour;
    uint8_t  offMinute;
    bool     twistActive;
    bool     twistArmed;
} TwistSettings;
typedef enum {
    DRY_IDLE = 0,
    DRY_WAITING,
    DRY_FAULT
} DryFSMState;
DryFSMState ModelHandle_GetDryState(void);
uint8_t ModelHandle_GetTankLevelPercent(void);
bool ModelHandle_IsTankFull(void);
extern volatile uint8_t  motorStatus;
extern volatile bool manualActive;
extern volatile bool countdownActive;
extern volatile bool twistActive;
extern volatile bool timerActive;
extern volatile bool semiAutoActive;
extern volatile bool autoActive;
extern volatile uint16_t auto_retry_counter;
extern volatile bool countdownMode;
extern volatile uint32_t countdownDuration;
extern TwistSettings  twistSettings;
extern volatile bool senseDryRun;
extern volatile bool senseOverLoad;
extern volatile bool senseOverUnderVolt;
extern volatile bool senseMaxRunReached;
extern volatile bool manualOverride;
uint32_t ModelHandle_TimeToSeconds(uint8_t hh, uint8_t mm);
void     ModelHandle_SecondsToTime(uint32_t sec, uint8_t* hh, uint8_t* mm);
void ModelHandle_SaveTimerToEEPROM(void);
void ModelHandle_LoadTimerFromEEPROM(void);
void ModelHandle_UpdateTimerFromScreen(void);
void ModelHandle_SetTimerSlot(uint8_t slot, uint8_t onH, uint8_t onM, uint8_t offH, uint8_t offM); // Set timer slot values
void ModelHandle_ProcessTimerSlots(void);
void ModelHandle_ToggleManual(void);
void ModelHandle_ManualLongPress(void);
void ModelHandle_SetMotor(bool on);
void ModelHandle_ClearManualOverride(void);
void Timer_EEPROM_EnsureValid(void);
void ModelHandle_StartSemiAuto(void);
uint8_t ModelHandle_GetPowerRestoreMode(void);
void ModelHandle_SetPowerRestoreMode(uint8_t mode);
void ModelHandle_StartAuto(uint16_t gap_s, uint16_t maxrun_min, uint8_t retry);
void ModelHandle_StopAuto(void);
void ModelHandle_StopAllModesAndMotor(void);
bool Motor_GetStatus(void);
bool ModelHandle_IsOverload(void);
bool ModelHandle_IsUnderload(void);
bool ModelHandle_IsDryRunActive(void);
bool ModelHandle_IsVoltageFault(void);
bool ModelHandle_IsMaxRunReached(void);
void ModelHandle_SetDryRun(bool on);
float    ModelHandle_GetOverloadLimit(void);
void ModelHandle_SetOverLoad(bool on);
void ModelHandle_SetOverUnderVolt(bool on);
void ModelHandle_ClearMaxRunFlag(void);
uint16_t ModelHandle_GetMaxRunTime(void);
uint16_t ModelHandle_GetOverVolt(void);
uint16_t ModelHandle_GetUnderVolt(void);
uint8_t  ModelHandle_GetRetryCount(void);
uint16_t ModelHandle_GetGapTime(void);
void ModelHandle_Button3_SinglePress(void);
float    ModelHandle_GetUnderloadLimit(void);
void ModelHandle_SetUserSettings(uint32_t gap_seconds,
                                 uint8_t  retry,
                                 uint16_t uv_limit,
                                 uint16_t ov_limit,
                                 int16_t  overload,
                                 int16_t  underload,
                                 uint16_t maxrun_min);
void ModelHandle_SetAutoSettings(uint16_t gap_s, uint16_t maxrun_min, uint8_t retry);
void ModelHandle_OnPowerUp(void);
void ModelHandle_LoadSettingsFromEEPROM(void);
void ModelHandle_SaveSettingsToEEPROM(void);
void ModelHandle_LoadTimerState(void);
void ModelHandle_SaveTimerState(void);
extern void ModelHandle_StartRestart(void);
extern bool ModelHandle_IsRestartActive(void);
void ModelHandle_ManualToggleMotor(void);
bool     ModelHandle_IsManualActive(void);
uint16_t ModelHandle_GetAutoGap(void);
uint16_t ModelHandle_GetAutoMaxRun(void);
uint8_t  ModelHandle_GetAutoRetry(void);

#endif /* MODEL_HANDLE_H */
