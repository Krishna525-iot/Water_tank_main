/* ====================================================================
 * main.c  —  RECEIVER (motor-controller node)
 *
 * Pin assignments verified against main.h (IOC updated 2026-06-09):
 *   RF_connector_Pin  = GPIO_PIN_1, RF_connector_GPIO_Port = GPIOD (PD1)
 *   RF_DATA_Pin       = GPIO_PIN_7, RF_DATA_GPIO_Port       = GPIOB (PB7)
 *   LORA_SELECT_Pin   = GPIO_PIN_15, GPIOA (PA15)
 *   LORA_STATUS_Pin   = GPIO_PIN_6,  GPIOB (PB6)
 *   Relay1/2/3        = PB0 / PB1 / PB2
 *   LED1/2/3          = PA8 / PA11 / PA12
 *   LED4/5            = PB8 / PB9
 *   SWITCH1–4         = PB12 / PB13 / PB14 / PB15
 *
 * v6.8 — RF received-data refresh corrected to match rf.c v6.8:
 *   • rf.c now guarantees RF_GetRxPacketCount() advances ONLY on a
 *     packet that passed the preamble gate, the sync word, CRC-8, AND
 *     LoRa_ParsePacket().  So when the counter changes here, a genuinely
 *     valid packet just arrived — no longer a noise-decoded false frame.
 *   • RF_GetLastRawPacket() now returns the POST-CRC payload (rf.c writes
 *     s_rfLastRawPacket only after CRC passes).  g_rfRxData.raw is
 *     therefore a real last-good packet, not the previous pre-CRC buffer
 *     that produced the misleading "@TL:000…/0x01-prefix" live view.
 *   • g_rfRxData.level/.wellDry are refreshed from the accessors on each
 *     advance, but they are only MEANINGFUL when .valid is true (a fresh
 *     TANKLEVEL within the 90 s window).  On a HELLO/ACK/etc. the level
 *     fields hold their last known TANKLEVEL value and .valid reflects
 *     freshness via RF_IsWirelessDataValid().
 *   • Status line now reports RF_frames (decode attempts) alongside
 *     RF_pkts (validated) and RF_err, so a stuck/garbage link is visible:
 *     frames climbing while pkts flat == carrier present but no valid
 *     frames (alignment / CRC), which is the symptom v6.8 fixes.
 *
 * Three operating modes via g_wireless_mode:
 *   WIRELESS_MODE_LOCAL  (0) — ADC probes only
 *   WIRELESS_MODE_LORA   (1) — SX1278 + local ADC fallback
 *   WIRELESS_MODE_RF433  (2) — XY-MK-5V OOK on PD1 + local ADC fallback
 *
 * PB7 is LoRa DIO0 — used only in WIRELESS_MODE_LORA.
 * PD1 is RF_connector_Pin — used only in WIRELESS_MODE_RF433.
 * ==================================================================== */

#include "main.h"
#include "lcd_i2c.h"
#include "rtc_i2c.h"
#include "global.h"
#include "adc.h"
#include "lora.h"
#include "rf.h"
#include "uart.h"
#include "model_handle.h"
#include "screen.h"
#include "led.h"
#include "relay.h"
#include "acs712.h"
#include "device_id.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* ── Wireless mode constants ─────────────────────────────────────────── */
#define WIRELESS_MODE_LOCAL   0u
#define WIRELESS_MODE_LORA    1u
#define WIRELESS_MODE_RF433   2u

/* ── Cadences ────────────────────────────────────────────────────────── */
#define ADC_TOTAL_CHANNELS       8u
#define STATUS_UPDATE_INTERVAL   15000u

/* ── Peripheral handles ─────────────────────────────────────────────── */
ADC_HandleTypeDef  hadc1;
I2C_HandleTypeDef  hi2c2;
RTC_HandleTypeDef  hrtc;
SPI_HandleTypeDef  hspi1;
TIM_HandleTypeDef  htim3;
UART_HandleTypeDef huart1;

/* ── Application data ───────────────────────────────────────────────── */
uint16_t adcBuffer[ADC_TOTAL_CHANNELS];
float    g_adcAvg[ADC_TOTAL_CHANNELS] = {0};
float    g_vADC_ACS  = 0.0f;
ADC_Data adcData;

extern float g_currentA;
extern float g_voltageV;

char receivedUartPacket[UART_RX_BUFFER_SIZE];
bool g_screenUpdatePending = false;
extern uint8_t loraMode;

/* ── Dedicated RF received-data holder ───────────────────────────────
 *  RfRxData_t is now defined ONCE in global.h and shared by every file
 *  that touches g_rfRxData (adc.c, screen.c, …).  main.c only DEFINES
 *  the instance here; the type and the `extern` live in global.h.
 *
 *  This removes the layout-mismatch that let other files read/write the
 *  wrong byte offsets (the cause of the impossible level=176 reading).
 * ──────────────────────────────────────────────────────────────────── */
RfRxData_t g_rfRxData = {0};

/* ── Wireless mode — FILE-SCOPE GLOBAL ──────────────────────────────── *
 *  adc.c reads it via `extern uint8_t g_wireless_mode;`.                 */
uint8_t g_wireless_mode = WIRELESS_MODE_RF433;   /* ← CHANGE HERE */

/* ── Status timer ───────────────────────────────────────────────────── */
static uint32_t lastStatusUpdate = 0u;

/* ── External declarations ──────────────────────────────────────────── */
extern bool    Motor_GetStatus(void);
extern uint8_t g_loraConnected;

/* New rf.c v6.8 diagnostics accessors */
extern uint32_t    RF_GetRxFrameCount(void);
extern const char *RF_GetLastRejectRaw(void);

/* ── Private function prototypes ────────────────────────────────────── */
void SystemClock_Config(void);
static void MX_GPIO_Init       (void);
static void MX_ADC1_Init       (void);
static void MX_SPI1_Init       (void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init       (void);
static void MX_TIM3_Init       (void);

/* ── UART helpers ────────────────────────────────────────────────────── */
void Debug_Print(char *msg) { UART_TransmitString(&huart1, msg); }

void UART_PrintLn(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s,      (uint16_t)strlen(s), 1000u);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2u,                  1000u);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) { /* reserved */ }
}

/* ── Mode name helper ───────────────────────────────────────────────── */
static const char *mode_name(uint8_t mode)
{
    switch (mode)
    {
        case WIRELESS_MODE_LORA:  return "LORA";
        case WIRELESS_MODE_RF433: return "RF433";
        default:                  return "LOCAL-ADC";
    }
}

/* ====================================================================
 *  main()
 * ==================================================================== */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    MX_I2C2_Init();
    MX_TIM3_Init();

    HAL_Delay(100u);

    UART_PrintLn("\r\n\r\n");
    UART_PrintLn("=========================================");
    UART_PrintLn("  HELONIX - RECEIVER (MOTOR CONTROLLER)");
    UART_PrintLn("  Firmware : Three-Mode RX v6.8");
    UART_PrintLn("  PCB      : v1.0 — schematic verified");
    UART_PrintLn("  UART     : 115200 8N1");
    UART_PrintLn("=========================================");
    UART_PrintLn("  Pin map (from IOC / main.h):");
    UART_PrintLn("    RF_DATA (XY-MK-5V) : PD1  (PD01 remap active)");
    UART_PrintLn("    LORA DIO0          : PB7  (RF_DATA_Pin, INPUT float)");
    UART_PrintLn("    LORA_SELECT (NSS)  : PA15 (SWJ_NOJTAG remap)");
    UART_PrintLn("    LORA_STATUS (RST)  : PB6");
    UART_PrintLn("    SPI1 CLK/MISO/MOSI : PB3/PB4/PB5 (SPI1 remap)");
    UART_PrintLn("    RELAY1/2/3         : PB0/PB1/PB2");
    UART_PrintLn("    SW1/2/3/4          : PB12/PB13/PB14/PB15");
    UART_PrintLn("    LED1/2/3           : PA8/PA11/PA12");
    UART_PrintLn("    LED4/5             : PB8/PB9");

    {
        char buf[64];
        snprintf(buf, sizeof(buf), "  Mode     : %s (%u)",
                 mode_name(g_wireless_mode), (unsigned)g_wireless_mode);
        UART_PrintLn(buf);
    }
    UART_PrintLn("=========================================");

    /* ── Peripheral and subsystem init ───────────────────────────── */
    RTC_Init();
    lcd_init();
    ADC_Init(&hadc1);

    /* ── Radio init — ONLY for the selected mode ─────────────────── */
    switch (g_wireless_mode)
    {
        case WIRELESS_MODE_LORA:
        {
            LoRa_Init();
            loraMode = LORA_MODE_RECEIVER;
            UART_PrintLn("[INIT] LoRa: SX1278 on PB3/4/5 (SPI1 remapped)");
            UART_PrintLn("[INIT] LoRa: NSS=PA15  RST=PB6  DIO0=PB7");
            UART_PrintLn("[INIT] LoRa: RX continuous — awaiting TX packets");
            break;
        }
        case WIRELESS_MODE_RF433:
        {
            RF_Init();
            UART_PrintLn("[INIT] RF433: XY-MK-5V DATA on PD1 (PD01 remap)");
            UART_PrintLn("[INIT] RF433: TIM3 reconfigured to 1 MHz (1 us/count)");
            UART_PrintLn("[INIT] RF433: RF_Task() polled continuously in loop");
            break;
        }
        case WIRELESS_MODE_LOCAL:
        default:
        {
            UART_PrintLn("[INIT] LOCAL: no radio — ADC probes only");
            break;
        }
    }

    Screen_Init();
    UART_Init();
    Switches_Init();
    Relay_Init();
    LED_Init();
    ACS712_Init(&hadc1);

    HAL_Delay(100u);
    Timer_EEPROM_EnsureValid();
    ModelHandle_LoadSettingsFromEEPROM();
    ModelHandle_LoadAutoSettings();
    ModelHandle_LoadTimerFromEEPROM();
    ModelHandle_LoadModeState();
    ModelHandle_LoadCountdown();
    ModelHandle_LoadBuzzerSettings();
    HAL_Delay(50u);
    ModelHandle_OnPowerUp();
    RTC_GetTimeDate();

    UART_PrintLn("\r\n[MAIN] All systems initialised — entering main loop");

    /* ================================================================
     *  Main loop
     * ================================================================ */
    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* Step 1: LoRa */
        if (g_wireless_mode == WIRELESS_MODE_LORA)
            LoRa_Task();

        /* Step 2: RF433 */
        if (g_wireless_mode == WIRELESS_MODE_RF433)
            RF_Task();

        /* Step 3: new-data flags + dedicated RF received-data refresh */
        if (g_wireless_mode == WIRELESS_MODE_LORA && g_loraNewPacketFlag)
        {
            g_loraNewPacketFlag   = false;
            g_screenUpdatePending = true;
        }
        if (g_wireless_mode == WIRELESS_MODE_RF433)
        {
            static uint32_t s_rfPktSeen  = 0u;
            static bool      s_rfWasValid = false;

            /* (a) Refresh g_rfRxData on EVERY validated packet.  In v6.8
             *     RF_GetRxPacketCount() advances ONLY after preamble gate
             *     + sync + CRC + parse, so a change here is a genuine
             *     packet.  RF_GetLastRawPacket() is now the post-CRC
             *     payload, so .raw is a real last-good packet.            */
            uint32_t pkts = RF_GetRxPacketCount();
            if (pkts != s_rfPktSeen)
            {
                s_rfPktSeen        = pkts;
                g_rfRxData.did     = RF_GetLastPacketDID();
                g_rfRxData.seq     = RF_GetLastPacketSeq();
                g_rfRxData.level   = RF_GetWirelessTankLevel();
                g_rfRxData.wellDry = RF_GetWirelessWellDry();

                strncpy(g_rfRxData.raw, RF_GetLastRawPacket(), RF_MAX_PAYLOAD);
                g_rfRxData.raw[RF_MAX_PAYLOAD] = '\0';

                g_screenUpdatePending = true;
            }

            /* (b) .valid tracks TANKLEVEL data freshness (90 s window). */
            bool rfNow = RF_IsWirelessDataValid();
            g_rfRxData.valid = rfNow;

            if (!s_rfWasValid && rfNow)
                g_screenUpdatePending = true;
            s_rfWasValid = rfNow;
        }

        /* Step 4: ACS712 */
        ACS712_Update();

        /* Step 5: ADC */
        ADC_ReadAllChannels(&hadc1, &adcData);

        /* Step 6: RTC */
        RTC_GetTimeDate();

        /* Step 7: UART commands */
        if (UART_GetReceivedPacket(receivedUartPacket, sizeof(receivedUartPacket)))
        {
            UART_HandleCommand(receivedUartPacket);
            g_screenUpdatePending = true;
        }

        /* Step 8: model */
        ModelHandle_CheckAutoTimerActivation();
        ModelHandle_Process();

        /* Step 9: screen + LED */
        Screen_HandleSwitches();
        Screen_Update();
        LED_Task();

        /* Step 10: periodic status */
        if ((now - lastStatusUpdate) >= STATUS_UPDATE_INTERVAL)
        {
            lastStatusUpdate = now;
            uint8_t lvl = 0u, wd = 0u;
            const char *lnk = "N/A";

            switch (g_wireless_mode)
            {
                case WIRELESS_MODE_LORA:
                    if (LoRa_IsWirelessDataValid())
                    {
                        lvl = LoRa_GetWirelessTankLevel();
                        wd  = LoRa_GetWirelessWellDry();
                        lnk = "OK";
                    }
                    else { lnk = "DOWN"; }
                    break;
                case WIRELESS_MODE_RF433:
                    if (g_rfRxData.valid)
                    {
                        lvl = g_rfRxData.level;
                        wd  = g_rfRxData.wellDry;
                        lnk = "OK";
                    }
                    else { lnk = "DOWN"; }
                    break;
                default: break;
            }

            /* RF_frames = decode attempts, RF_pkts = validated packets.
             * frames climbing while pkts flat  => carrier present but no
             * valid frame (alignment/CRC) — the v6.8 fix target.          */
            char status[200];
            snprintf(status, sizeof(status),
                     "[STATUS] Mode:%-9s | Link:%-4s | TL:%3u%% | WD:%u | "
                     "RF_pkts:%lu | RF_frames:%lu | RF_err:%lu | Motor:%s",
                     mode_name(g_wireless_mode), lnk, lvl, wd,
                     (unsigned long)RF_GetRxPacketCount(),
                     (unsigned long)RF_GetRxFrameCount(),
                     (unsigned long)RF_GetRxErrorCount(),
                     Motor_GetStatus() ? "ON" : "OFF");
            UART_PrintLn(status);
        }

        /* Step 11: loop pace */
        HAL_Delay(10u);
    }
}

/* ====================================================================
 *  SystemClock_Config
 * ==================================================================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit     = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState            = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC;
    PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSI;
    PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

/* ====================================================================
 *  MX_GPIO_Init  (unchanged — AFIO remap order is load-bearing)
 * ==================================================================== */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
    __HAL_AFIO_REMAP_SPI1_ENABLE();
    __HAL_AFIO_REMAP_PD01_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, Relay1_Pin | Relay2_Pin | Relay3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, LED1_Pin | LED2_Pin | LED3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED4_Pin | LED5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_STATUS_GPIO_Port, LORA_STATUS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_SELECT_GPIO_Port, LORA_SELECT_Pin, GPIO_PIN_SET);

    /* Relays: PB0/PB1/PB2 */
    GPIO_InitStruct.Pin   = Relay1_Pin | Relay2_Pin | Relay3_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* LoRa RST: PB6 */
    GPIO_InitStruct.Pin   = LORA_STATUS_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LORA_STATUS_GPIO_Port, &GPIO_InitStruct);

    /* LoRa DIO0: PB7 — input, no pull */
    GPIO_InitStruct.Pin  = RF_DATA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(RF_DATA_GPIO_Port, &GPIO_InitStruct);

    /* XY-MK-5V DATA: PD1 — input, no pull */
    GPIO_InitStruct.Pin  = RF_connector_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(RF_connector_GPIO_Port, &GPIO_InitStruct);  /* GPIOD */

    /* LED4/LED5: PB8/PB9 */
    GPIO_InitStruct.Pin   = LED4_Pin | LED5_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Switches: PB12–PB15 — input pull-up, EXTI both edges */
    GPIO_InitStruct.Pin  = SWITCH1_Pin | SWITCH2_Pin | SWITCH3_Pin | SWITCH4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* LED1: PA8 */
    GPIO_InitStruct.Pin   = LED1_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* LED2/LED3: PA11/PA12 */
    GPIO_InitStruct.Pin   = LED2_Pin | LED3_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* LoRa NSS: PA15 — high speed */
    GPIO_InitStruct.Pin   = LORA_SELECT_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LORA_SELECT_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LORA_SELECT_GPIO_Port, LORA_SELECT_Pin, GPIO_PIN_SET);
}

/* ====================================================================
 *  MX_ADC1_Init  (unchanged)
 * ==================================================================== */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 8;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    uint32_t chList[] = {
        ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
        ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7
    };
    for (uint8_t r = 0u; r < 8u; r++)
    {
        sConfig.Channel      = chList[r];
        sConfig.Rank         = r + 1u;
        sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
    }
}

/* ====================================================================
 *  MX_I2C2_Init  (unchanged)
 * ==================================================================== */
static void MX_I2C2_Init(void)
{
    hi2c2.Instance             = I2C2;
    hi2c2.Init.ClockSpeed      = 100000;
    hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1     = 0;
    hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2     = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
}

/* ====================================================================
 *  MX_SPI1_Init  (unchanged)
 * ==================================================================== */
static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

/* ====================================================================
 *  MX_TIM3_Init  (unchanged — RF_Init() represcales to 1 MHz)
 * ==================================================================== */
static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 0xFFFF;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();
}

/* ====================================================================
 *  MX_USART1_UART_Init  (unchanged)
 * ==================================================================== */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

/* ====================================================================
 *  Error / assert handlers
 * ==================================================================== */
void Error_Handler(void)
{
    UART_PrintLn("[ERROR] Error_Handler — system halted!");
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
