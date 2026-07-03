/* ====================================================================
 * lora.h  —  RECEIVER public interface for SX1278 / Ra-02 433MHz
 *
 * Pin assignments verified against schematic SCH_Schematic1_2026-06-08.
 *
 * Ra-02 wiring (schematic page 2):
 *   NSS  (pin 15) → PA15  LORA_SELECT  (freed by SWJ_NOJTAG)
 *   RST  (pin 4)  → PB6   LORA_STATUS  (active-LOW pulse)
 *   DIO0 (pin 5)  → PB7   RF_DATA_Pin  (via R4 0Ω — same net as RF433 DATA)
 *   SCK  (pin 12) → PB3   SPI1_CLK     (SPI1 remapped from PA5)
 *   MISO (pin 13) → PB4   SPI1_MISO    (SPI1 remapped from PA6)
 *   MOSI (pin 14) → PB5   SPI1_MOSI    (SPI1 remapped from PA7)
 *
 * DIO0 shares PB7 with the RF433 XY-MK-5V DATA pin.
 * This is safe: only one radio is active at a time (g_wireless_mode).
 * Both functions require PB7 as INPUT, no pull — same config.
 *
 * AFIO remaps required in MX_GPIO_Init() (in this order):
 *   __HAL_AFIO_REMAP_SWJ_NOJTAG()  → frees PA15/PB3/PB4 from JTAG
 *   __HAL_AFIO_REMAP_SPI1_ENABLE() → moves SPI1 to PB3/PB4/PB5
 * ==================================================================== */

#ifndef LORA_RX_H
#define LORA_RX_H

#include "stm32f1xx_hal.h"
#include "main.h"
#include "lora_protocol.h"
#include <stdbool.h>
#include <stdint.h>

/* ── Pin definitions ───────────────────────────────────────────────── */
#define LORA_NSS_PORT    LORA_SELECT_GPIO_Port   /* GPIOA, PA15          */
#define LORA_NSS_PIN     LORA_SELECT_Pin         /* GPIO_PIN_15          */

#define LORA_RESET_PORT  LORA_STATUS_GPIO_Port   /* GPIOB, PB6           */
#define LORA_RESET_PIN   LORA_STATUS_Pin         /* GPIO_PIN_6           */

#define LORA_DIO0_PORT   RF_DATA_GPIO_Port       /* GPIOB, PB7           */
#define LORA_DIO0_PIN    RF_DATA_Pin             /* GPIO_PIN_7           */

/* ── LoRa mode ─────────────────────────────────────────────────────── */
#define LORA_MODE_TRANSMITTER  0
#define LORA_MODE_RECEIVER     1

/* ── SX1278 / Ra-02 433MHz RF config ────────────────────────────────
 * Must match transmitter lora.h exactly.
 */
#define LORA_FREQ_HZ              433000000UL

/* RegModemConfig1: BW=125kHz, CR=4/5, Explicit Header */
#define LORA_REG_MODEM_CFG1       0x72

/* RegModemConfig2: SF7, CRC ON */
#define LORA_REG_MODEM_CFG2       0x74

/* RegModemConfig3: LowDataRateOptimize OFF, AGC Auto ON */
#define LORA_REG_MODEM_CFG3       0x04

/* SX1278 detect settings */
#define LORA_REG_DETECT_OPT       0x03
#define LORA_REG_DETECT_OPT2      0xC3
#define LORA_REG_DETECTION_TH     0x0A

/* Private LoRa sync word — must match TX */
#define LORA_SYNC_WORD            0x12

/* Preamble = 8 symbols */
#define LORA_PREAMBLE_MSB         0x00
#define LORA_PREAMBLE_LSB         0x08

/* PA_BOOST */
#define LORA_REG_PA_CONFIG        0x8F

/* OCP ~100 mA */
#define LORA_REG_OCP              0x2B

/* Normal PA DAC */
#define LORA_REG_PA_DAC           0x84

/* ── Timing ────────────────────────────────────────────────────────── */
#define LORA_TX_TIMEOUT_MS              3000UL
#define RX_PEER_TIMEOUT_MS              30000UL
#define RX_SYNC_REQ_INTERVAL_MS         10000UL
#define RX_PAIRING_TIMEOUT_MS           30000UL

/* ── Buffer sizes ──────────────────────────────────────────────────── */
#ifndef LORA_BUF_SIZE
#define LORA_BUF_SIZE                   128
#endif

#ifndef MAX_PACKET_LEN
#define MAX_PACKET_LEN                  96
#endif

/* ── Debug variables ───────────────────────────────────────────────── */
extern volatile char g_lastRxPacket[MAX_PACKET_LEN];
extern volatile char g_lastTxPacket[MAX_PACKET_LEN];

/* ── Public globals ────────────────────────────────────────────────── */
extern uint8_t  loraMode;
extern uint8_t  g_loraConnected;
extern bool     g_loraNewPacketFlag;

extern uint32_t g_lora_tx_ok;
extern uint32_t g_lora_tx_retry;
extern uint32_t g_lora_tx_fail;

void LoRa_ClearWirelessData(void);

/* ── Core API ──────────────────────────────────────────────────────── */
void              LoRa_Init           (void);
void              LoRa_Task           (void);

LoRa_ConnState_t  LoRa_GetState       (void);
const char *      LoRa_GetStateString (void);

#define LORA_DISCOVERY_MAX 5

typedef struct
{
    uint32_t did;
    int16_t  rssi;
    uint32_t lastSeen;
    bool     valid;
} LoRa_DiscoveredDevice_t;

uint8_t  LoRa_GetDiscoveredCount(void);
bool     LoRa_GetDiscoveredDevice(uint8_t index, LoRa_DiscoveredDevice_t *out);
bool     LoRa_PairDiscoveredDevice(uint8_t index);
void     LoRa_ClearDiscoveredDevices(void);

/* ── Wireless data accessors ───────────────────────────────────────── */
uint8_t           LoRa_GetWirelessTankLevel(void);
uint8_t           LoRa_GetWirelessWellDry  (void);
uint32_t          LoRa_GetLastSequence     (void);
uint32_t          LoRa_GetPacketsLost      (void);
bool              LoRa_IsWirelessDataValid (void);
void              LoRa_OnPairingListChanged(void);

/* ── Pairing API ───────────────────────────────────────────────────── */
void     LoRa_EnterPairingMode (void);
void     LoRa_ExitPairingMode  (void);
bool     LoRa_IsPairingMode    (void);
bool     LoRa_IsPairingComplete(void);
uint32_t LoRa_GetLastPairedDID (void);

/* ── Low-level register access ─────────────────────────────────────── */
void    LoRa_WriteReg(uint8_t addr, uint8_t data);
uint8_t LoRa_ReadReg (uint8_t addr);

#endif /* LORA_RX_H */
