/* ====================================================================
 * adc.c  —  RECEIVER  (motor-controller node)
 *
 * NODE TYPE: LORA_RECEIVER_NODE
 *
 * ── Three operating modes — controlled by g_wireless_mode ───────────
 *
 *  g_wireless_mode is a FILE-SCOPE GLOBAL defined in main.c.
 *  This file reads it via  extern uint8_t g_wireless_mode.
 *  Change one line in main.c to switch the entire data source.
 *
 *    WIRELESS_MODE_LOCAL  (0) — Step 2 does nothing.
 *                               Local EMA ADC for all channels, always.
 *                               No radio ever queried here.
 *
 *    WIRELESS_MODE_LORA   (1) — Step 2 calls LoRa_IsWirelessDataValid().
 *                               Valid   → inject LoRa  data CH0–3, CH5.
 *                               Invalid → keep local ADC (link down).
 *                               RF433 not touched.
 *
 *    WIRELESS_MODE_RF433  (2) — Step 2 calls RF_IsWirelessDataValid().
 *                               Valid   → inject RF433 data CH0–3, CH5.
 *                               Invalid → keep local ADC (no packets).
 *                               LoRa not touched.
 *
 *  CH4 (ground water) is ALWAYS from local ADC in all modes.
 *
 * ── Voltage synthesis for CH0–CH3 (tank level probes) ───────────────
 *
 *   model_handle.c PROBE_THRESHOLD = 0.50 V
 *   voltage < 0.50 V → submerged     voltage ≥ 0.50 V → above water
 *
 *     voltages[0] = 100 % probe
 *     voltages[1] =  75 % probe
 *     voltages[2] =  50 % probe
 *     voltages[3] =  25 % probe
 *
 * ── Voltage synthesis for CH5 (dry-run / well-dry sensor) ───────────
 *
 *   model_handle.c: senseDryRun = (voltages[5] < 0.30 V)
 *
 *   WD=0 (well has water) → 1.0 V → senseDryRun = false
 *   WD=1 (well DRY)       → 0.0 V → senseDryRun = true → motor stops
 *
 * Fix v6.2:
 *   g_wireless_mode is now a proper file-scope global in main.c.
 *   The  extern uint8_t g_wireless_mode  declaration here resolves
 *   correctly at link time.  (Previously it was a local variable
 *   inside main() which has no linkage — linker error.)
 * ==================================================================== */

#include "adc.h"
#include "lora.h"    /* LoRa_IsWirelessDataValid / LoRa_GetWireless*() */
#include "rf.h"      /* RF_IsWirelessDataValid  / RF_GetWireless*()    */
#include "main.h"
#include "uart.h"
#include "global.h"
#include "led.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "model_handle.h"

/* ── Wireless mode constants — must match main.c ────────────────────── *
 *  Do NOT change the numeric values here.                               *
 *  Change g_wireless_mode in main.c to switch mode.                   */
#define WIRELESS_MODE_LOCAL   0u
#define WIRELESS_MODE_LORA    1u
#define WIRELESS_MODE_RF433   2u

/* ── g_wireless_mode — defined in main.c, read here via extern ──────── *
 *                                                                        *
 *  This MUST be an extern to a FILE-SCOPE global in main.c.            *
 *  If main.c defines it as a local variable inside main(), the linker  *
 *  will produce: "undefined reference to g_wireless_mode".             *
 *  Fix: declare it outside any function in main.c (file scope).        */
extern uint8_t g_wireless_mode;

/* ── ADC configuration ──────────────────────────────────────────────── */
#ifndef THR
#define THR                     1.0f
#endif
#define EMA_ALPHA               0.3f
#define HYST_DELTA              0.10f
#define GROUND_THRESHOLD        0.5f
#define DRY_VOLTAGE_THRESHOLD   0.05f
#define PRINT_DELTA             0.05f
#define VREF                    3.3f
#define ADC_RES                 4095.0f

/* ── Injected probe voltages ─────────────────────────────────────────
 *  PROBE_SUBMERGED  < PROBE_THRESHOLD (0.50 V) → water detected
 *  PROBE_DRY        > PROBE_THRESHOLD           → no water
 *  SENSOR_WATER    ≥ 0.30 V  →  senseDryRun = false  (motor allowed)
 *  SENSOR_DRY      < 0.30 V  →  senseDryRun = true   (motor blocked)
 * ──────────────────────────────────────────────────────────────────── */
#define PROBE_SUBMERGED  0.0f
#define PROBE_DRY        1.0f
#define SENSOR_WATER     1.0f
#define SENSOR_DRY       0.0f

/* ── Module state ───────────────────────────────────────────────────── */
float g_adcVoltages[ADC_CHANNEL_COUNT] = {0};
float g_acVoltage_raw = 0.0f;
float g_acCurrent_raw = 0.0f;
float g_acVoltage_avg = 0.0f;
float g_acCurrent_avg = 0.0f;
bool  g_overload      = false;

static float   s_filtered[ADC_CHANNEL_COUNT]    = {0};
static uint8_t s_level_flags[ADC_CHANNEL_COUNT] = {0};
static float   s_prev_volt[ADC_CHANNEL_COUNT]   = {0};

static const uint32_t adcChannels[ADC_CHANNEL_COUNT] = {
    ADC_CHANNEL_0,
    ADC_CHANNEL_1,
    ADC_CHANNEL_2,
    ADC_CHANNEL_3,
    ADC_CHANNEL_4,
    ADC_CHANNEL_5
};

static char dataPacketTx[16];

/* ── Source tracking — for UART transition log only ────────────────── */
typedef enum
{
    ADC_SRC_LOCAL = 0,
    ADC_SRC_LORA,
    ADC_SRC_RF433
} ADC_ActualSrc_t;

static ADC_ActualSrc_t s_activeSrc = ADC_SRC_LOCAL;

/* ── Low-level single-channel ADC read ─────────────────────────────── */
static float readChannelVoltage(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = channel;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) return 0.0f;
    if (HAL_ADC_Start(hadc)                   != HAL_OK) return 0.0f;

    float v = 0.0f;
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK)
    {
        uint32_t raw = HAL_ADC_GetValue(hadc);
        v = (raw * VREF) / ADC_RES;
    }
    HAL_ADC_Stop(hadc);
    return v;
}

/* ── ADC_Init ───────────────────────────────────────────────────────── */
void ADC_Init(ADC_HandleTypeDef *hadc)
{
    if (HAL_ADCEx_Calibration_Start(hadc) != HAL_OK)
        Error_Handler();
}

/* ── inject_wireless_level ──────────────────────────────────────────
 *
 *  Shared by WIRELESS_MODE_LORA and WIRELESS_MODE_RF433.
 *  Overwrites data->voltages[0..3] from lvlPct, voltages[5] from
 *  wellDry.  CH4 is intentionally NOT touched (always local ADC).
 *
 *  s_filtered[0..3,5] is also updated so EMA state stays in sync
 *  and does not snap when switching back to local ADC after a drop.
 * ──────────────────────────────────────────────────────────────────── */
static void inject_wireless_level(ADC_Data *data,
                                   uint8_t   lvlPct,
                                   uint8_t   wellDry)
{
    if (lvlPct > 100u) lvlPct = 100u;

    /* Tank level → probe voltages */
    float v0 = (lvlPct >= 100u) ? PROBE_SUBMERGED : PROBE_DRY;
    float v1 = (lvlPct >=  75u) ? PROBE_SUBMERGED : PROBE_DRY;
    float v2 = (lvlPct >=  50u) ? PROBE_SUBMERGED : PROBE_DRY;
    float v3 = (lvlPct >=  25u) ? PROBE_SUBMERGED : PROBE_DRY;

    data->voltages[0] = v0;  s_filtered[0] = v0;
    data->voltages[1] = v1;  s_filtered[1] = v1;
    data->voltages[2] = v2;  s_filtered[2] = v2;
    data->voltages[3] = v3;  s_filtered[3] = v3;

    for (int i = 0; i < 4; i++)
        data->rawValues[i] = (uint16_t)((data->voltages[i] * ADC_RES) / VREF);

    /* Well-dry → dry-run sensor voltage */
    float v5 = (wellDry != 0u) ? SENSOR_DRY : SENSOR_WATER;

    data->voltages[5]  = v5;
    s_filtered[5]      = v5;
    data->rawValues[5] = (uint16_t)((v5 * ADC_RES) / VREF);

    /* CH4 is NOT written — always local ADC */
}

/* ── log_source_change ──────────────────────────────────────────────
 *
 *  Prints one UART line when the actual data source changes.
 *  Suppresses logging while source is stable (no repeated prints).
 * ──────────────────────────────────────────────────────────────────── */
static void log_source_change(ADC_ActualSrc_t newSrc,
                               uint8_t lvl, uint8_t wd)
{
    if (newSrc == s_activeSrc) return;

    s_activeSrc = newSrc;

    char dbg[100];

    switch (newSrc)
    {
        case ADC_SRC_LORA:
            snprintf(dbg, sizeof(dbg),
                     "[ADC] Source → LoRa    Level:%u%%  WD:%u",
                     lvl, wd);
            break;

        case ADC_SRC_RF433:
            snprintf(dbg, sizeof(dbg),
                     "[ADC] Source → RF433   Level:%u%%  WD:%u  DID:%08lX",
                     lvl, wd,
                     (unsigned long)RF_GetLastPacketDID());
            break;

        case ADC_SRC_LOCAL:
        default:
            snprintf(dbg, sizeof(dbg),
                     "[ADC] Source → LOCAL ADC (wireless link down or LOCAL mode)");
            break;
    }

    HAL_UART_Transmit(&huart1,
                      (uint8_t *)dbg, (uint16_t)strlen(dbg), 500u);
    HAL_UART_Transmit(&huart1,
                      (uint8_t *)"\r\n", 2u, 500u);
}

/* ── ADC_ReadAllChannels ────────────────────────────────────────────
 *
 *  Step 1 : Read all 6 physical ADC channels through EMA.
 *           Runs unconditionally — keeps local values fresh.
 *
 *  Step 2 : Mode-controlled wireless override (reads g_wireless_mode):
 *
 *    WIRELESS_MODE_LOCAL  (0)
 *      No override.  Local EMA values used for all channels.
 *
 *    WIRELESS_MODE_LORA   (1)
 *      If LoRa link valid  → inject into CH0–3, CH5.
 *      If LoRa link down   → local ADC (no RF433 fallback).
 *
 *    WIRELESS_MODE_RF433  (2)
 *      If RF433 link valid → inject into CH0–3, CH5.
 *      If RF433 link down  → local ADC (no LoRa fallback).
 *
 *    CH4 is ALWAYS local ADC regardless of mode.
 *
 *  Step 3 : Level-flag events on FINAL voltages (after Step 2).
 * ──────────────────────────────────────────────────────────────────── */
void ADC_ReadAllChannels(ADC_HandleTypeDef *hadc, ADC_Data *data)
{
    char loraPacket[32];
    loraPacket[0] = '\0';

    /* ════════════════════════════════════════════════════════════════
     *  Step 1 — EMA-filter every local ADC channel
     * ════════════════════════════════════════════════════════════════ */
    for (uint8_t i = 0u; i < ADC_CHANNEL_COUNT; i++)
    {
        float v = readChannelVoltage(hadc, adcChannels[i]);

        s_filtered[i] = (s_filtered[i] == 0.0f)
                        ? v
                        : (EMA_ALPHA * v + (1.0f - EMA_ALPHA) * s_filtered[i]);

        v = s_filtered[i];
        data->voltages[i]   = v;
        data->rawValues[i]  = (uint16_t)((v * ADC_RES) / VREF);
        data->maxReached[i] = (v >= 3.2f);
        g_adcVoltages[i]    = v;

        if (fabsf(v - s_prev_volt[i]) > PRINT_DELTA)
            s_prev_volt[i] = v;
    }

    /* ════════════════════════════════════════════════════════════════
     *  Step 2 — Mode-controlled wireless override
     *
     *  The switch is driven by g_wireless_mode (global in main.c).
     *  Each case is completely independent — no cross-mode fallback.
     * ════════════════════════════════════════════════════════════════ */
    switch (g_wireless_mode)
    {
        /* ── Mode 0: LOCAL ────────────────────────────────────────── *
         *  Physical probes only.  No radio queried.                   *
         *  Use when the board is installed directly at the tank.      */
        case WIRELESS_MODE_LOCAL:
        {
            log_source_change(ADC_SRC_LOCAL, 0u, 0u);
            /* local values from Step 1 already in data — nothing to do */
        }
        break;

        /* ── Mode 1: LORA ─────────────────────────────────────────── *
         *  LoRa data when link is healthy; local ADC when link down.  *
         *  RF433 is NEVER checked in this mode.                       */
        case WIRELESS_MODE_LORA:
        {
            if (LoRa_IsWirelessDataValid())
            {
                uint8_t lvl = LoRa_GetWirelessTankLevel();
                uint8_t wd  = LoRa_GetWirelessWellDry();
                inject_wireless_level(data, lvl, wd);
                log_source_change(ADC_SRC_LORA, lvl, wd);
            }
            else
            {
                /* LoRa link down → local ADC fallback (Step 1 values) */
                log_source_change(ADC_SRC_LOCAL, 0u, 0u);
            }
        }
        break;

        /* ── Mode 2: RF433 ────────────────────────────────────────── *
         *  RF433 data when packets arriving; local ADC when silent.   *
         *  LoRa is NEVER checked in this mode.                        */
        case WIRELESS_MODE_RF433:
        {
            if (RF_IsWirelessDataValid())
            {
                uint8_t lvl = RF_GetWirelessTankLevel();
                uint8_t wd  = RF_GetWirelessWellDry();
                inject_wireless_level(data, lvl, wd);
                log_source_change(ADC_SRC_RF433, lvl, wd);
            }
            else
            {
                /* RF link silent → local ADC fallback (Step 1 values) */
                log_source_change(ADC_SRC_LOCAL, 0u, 0u);
            }
        }
        break;

        default:
        {
            /* Unknown mode — safe default: local ADC */
            log_source_change(ADC_SRC_LOCAL, 0u, 0u);
        }
        break;
    }

    /* ════════════════════════════════════════════════════════════════
     *  Step 3 — Level-flag events on FINAL voltages
     *
     *  Evaluated after Step 2 so events always match what
     *  model_handle.c actually sees, regardless of data source.
     * ════════════════════════════════════════════════════════════════ */
    for (uint8_t i = 0u; i < ADC_CHANNEL_COUNT; i++)
    {
        float v = data->voltages[i];

        /* CH0–CH3: tank level probes */
        if (i <= 3u)
        {
            if (!s_level_flags[i] && v >= THR)
            {
                s_level_flags[i] = 1u;
                switch (i)
                {
                    case 0:  snprintf(dataPacketTx, sizeof(dataPacketTx), "@L1#");   break;
                    case 1:  snprintf(dataPacketTx, sizeof(dataPacketTx), "@L2#");   break;
                    case 2:  snprintf(dataPacketTx, sizeof(dataPacketTx), "@L3#");   break;
                    case 3:  snprintf(dataPacketTx, sizeof(dataPacketTx), "@FULL#"); break;
                    default: dataPacketTx[0] = '\0'; break;
                }
                if (dataPacketTx[0])
                {
                    strncat(loraPacket, dataPacketTx,
                            sizeof(loraPacket) - strlen(loraPacket) - 1u);
                    strncat(loraPacket, ";",
                            sizeof(loraPacket) - strlen(loraPacket) - 1u);
                }
            }
            else if (s_level_flags[i] && v < (THR - HYST_DELTA))
            {
                s_level_flags[i] = 0u;
            }
            continue;
        }

        /* CH4: ground water — always local ADC */
        if (i == 4u)
        {
            if (!s_level_flags[i] && v >= GROUND_THRESHOLD)
            {
                s_level_flags[i] = 1u;
                snprintf(dataPacketTx, sizeof(dataPacketTx), "@GW#");
                strncat(loraPacket, dataPacketTx,
                        sizeof(loraPacket) - strlen(loraPacket) - 1u);
                strncat(loraPacket, ";",
                        sizeof(loraPacket) - strlen(loraPacket) - 1u);
            }
            else if (s_level_flags[i] && v < (GROUND_THRESHOLD - HYST_DELTA))
            {
                s_level_flags[i] = 0u;
            }
            continue;
        }

        /* CH5: dry-run sensor */
        if (i == 5u)
        {
            if (!s_level_flags[i] && v >= DRY_VOLTAGE_THRESHOLD)
            {
                s_level_flags[i] = 1u;
                snprintf(dataPacketTx, sizeof(dataPacketTx), "@DRY#");
                strncat(loraPacket, dataPacketTx,
                        sizeof(loraPacket) - strlen(loraPacket) - 1u);
                strncat(loraPacket, ";",
                        sizeof(loraPacket) - strlen(loraPacket) - 1u);
            }
            else if (s_level_flags[i] && v < (DRY_VOLTAGE_THRESHOLD - HYST_DELTA))
            {
                s_level_flags[i] = 0u;
            }
            continue;
        }
    }
}

/* ── ADC_CheckMaxVoltage (unchanged) ────────────────────────────────── */
uint8_t ADC_CheckMaxVoltage(ADC_Data *data, float threshold)
{
    for (uint8_t i = 0u; i < ADC_CHANNEL_COUNT; i++)
        if (data->voltages[i] >= threshold) return 1u;
    return 0u;
}

/* ── ADC_GetActiveSourceString — for LCD / status display ───────────── */
const char *ADC_GetActiveSourceString(void)
{
    switch (s_activeSrc)
    {
        case ADC_SRC_LORA:  return "LoRa";
        case ADC_SRC_RF433: return "RF433";
        default:            return "LOCAL";
    }
}
