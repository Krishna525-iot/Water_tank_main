/* ====================================================================
 * rf.h  —  433 MHz OOK/ASK RF layer
 *
 * Shared between TRANSMITTER and RECEIVER firmware.
 * Copy this file into both projects unchanged.
 *
 * Hardware — TX board:
 *   FS1000A DATA  ← PD1  (RF_connector_Pin, GPIOD GPIO_PIN_1)
 *   GPIO config   : OUTPUT_PP, GPIO_SPEED_FREQ_HIGH
 *   AFIO remap    : __HAL_AFIO_REMAP_PD01_ENABLE() required in
 *                   MX_GPIO_Init() to release PD0/PD1 from OSC.
 *
 * Hardware — RX board:
 *   XY-MK-5V DATA → PD1  (RF_connector_Pin, GPIOD GPIO_PIN_1)
 *   GPIO config   : INPUT, no pull (module drives line actively)
 *   AFIO remap    : __HAL_AFIO_REMAP_PD01_ENABLE() required in
 *                   MX_GPIO_Init() to release PD0/PD1 from OSC.
 *
 * IMPORTANT: PD0 and PD1 are OSC_IN / OSC_OUT by default on STM32F1.
 * They cannot be used as GPIO unless the PD01 remap is enabled in AFIO.
 * Call __HAL_AFIO_REMAP_PD01_ENABLE() BEFORE HAL_GPIO_Init() for PD1.
 *
 * Wire format (byte-level, MSB first within each byte):
 *   [Preamble: RF_PREAMBLE_BYTES × 0xAA]
 *   [Sync1: 0x2D] [Sync2: 0xD4]
 *   [Length: 1 byte — count of payload bytes]
 *   [Payload: 1..RF_MAX_PAYLOAD bytes — same ASCII format as LoRa]
 *   [CRC-8: 1 byte — poly 0x07, init 0x00 over payload only]
 *
 * Bit encoding (Manchester-style OOK, 1200 µs / bit ≈ 833 bps):
 *   bit '1' → HIGH 300 µs  + LOW  900 µs
 *   bit '0' → HIGH 900 µs  + LOW  300 µs
 *
 * Timer dependency:
 *   RF_Init() reconfigures TIM3 → 1 MHz (prescaler = 63).
 *   TIM3 clock = 64 MHz (APB1=32 MHz, HAL doubles when APB1≠AHB).
 *   64 MHz / (63+1) = 1 MHz → 1 µs / count.
 *   TIM3 is used exclusively for µs timing; no conflict with SysTick.
 * ==================================================================== */

#ifndef RF_H
#define RF_H

#include <stdint.h>
#include <stdbool.h>

/* ── Wire-format constants (must be identical on TX and RX) ─────────── */
#define RF_PREAMBLE_BYTES           5u      /* 5 × 0xAA = 40 preamble bits  */
#define RF_SYNC_BYTE1               0x2Du
#define RF_SYNC_BYTE2               0xD4u
#define RF_MAX_PAYLOAD              48u     /* maximum payload bytes        */
#define RF_TX_REPEATS               3u      /* transmit each frame N times  */
#define RF_INTER_PKT_GAP_US         20000u  /* 20 ms quiet between repeats  */

/* ── Bit timing (µs) ────────────────────────────────────────────────── */
#define RF_BIT_ONE_HIGH_US          300u
#define RF_BIT_ONE_LOW_US           900u
#define RF_BIT_ZERO_HIGH_US         900u
#define RF_BIT_ZERO_LOW_US          300u
#define RF_BIT_PERIOD_US            1200u

/* ── RX decode thresholds: HIGH pulse duration (±33% tolerance) ─────── */
#define RF_THRESH_ONE_MIN_US        100u
#define RF_THRESH_ONE_MAX_US        550u
#define RF_THRESH_ZERO_MIN_US       600u
#define RF_THRESH_ZERO_MAX_US       1100u

/* ── TX cadence ─────────────────────────────────────────────────────── */
#define RF_HELLO_INTERVAL_MS        5000u   /* HELLO broadcast every 5 s    */
#define RF_DATA_INTERVAL_MS         12000u  /* DATA  broadcast every 12 s   */

/* ── RX data validity window ────────────────────────────────────────── */
#define RF_WIRELESS_TIMEOUT_MS      90000u  /* data expires after 90 s      */

/* ── RX robustness ──────────────────────────────────────────────────── */
#define RF_MIN_VALID_PREAMBLE_BITS  16u     /* min preamble bits before sync */
#define RF_SYNC_HUNT_MAX_BITS       120u    /* give up sync hunt after this  */

/* ── Shared API ─────────────────────────────────────────────────────── */
void    RF_Init (void);
uint8_t RF_CRC8 (const uint8_t *data, uint8_t len);

/* ── TX-only API ────────────────────────────────────────────────────── */
void RF_SendPacket (const char *payload, uint8_t len);
void RF_Service    (uint8_t tank_level, uint8_t well_dry);

/* ── RX-only API ────────────────────────────────────────────────────── */
void          RF_Task                (void);
bool          RF_IsWirelessDataValid (void);
uint8_t       RF_GetWirelessTankLevel(void);
uint8_t       RF_GetWirelessWellDry  (void);
void          RF_ClearWirelessData   (void);

bool          RF_HasReceivedPacket   (void);
const char   *RF_GetLastRawPacket    (void);
uint32_t      RF_GetLastPacketDID    (void);
uint32_t      RF_GetLastPacketSeq    (void);
uint8_t       RF_GetLastPacketType   (void);
uint8_t       RF_GetLastPacketCrcRx  (void);
uint8_t       RF_GetLastPacketCrcCalc(void);
uint32_t      RF_GetLastPacketAgeMs  (void);
uint32_t      RF_GetRxPacketCount    (void);
uint32_t      RF_GetRxErrorCount     (void);

#endif /* RF_H */
