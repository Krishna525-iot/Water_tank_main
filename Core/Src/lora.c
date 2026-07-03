/* ====================================================================
 * lora.c  —  RECEIVER LoRa driver  (WT1.1 motor-controller project)
 *
 * v3.4 — 433 MHz register fixes
 *
 * Bugs fixed vs v3.3:
 *
 *  BUG 1 (CRITICAL) — LowFrequencyModeOn missing from all RegOpMode writes
 *    SX1278 RegOpMode bit3 = LowFrequencyModeOn:
 *      1 = low-band PA/PLL path (< 525 MHz)   ← required for 433 MHz
 *      0 = high-band PA/PLL path (> 525 MHz)
 *    Without bit3, the PA and synthesiser use the wrong band, and the
 *    radio will not transmit or receive correctly at 433 MHz.
 *    All RegOpMode writes updated:
 *      sleep    : 0x80 → 0x88
 *      standby  : 0x81 → 0x89
 *      TX       : 0x83 → 0x8B
 *      RX cont  : 0x85 → 0x8D
 *    Also: LoRa_EnterRxContinuous() polls for 0x8D (was 0x85).
 *
 *  BUG 2 (MODERATE) — Wrong value written to RegModemConfig3 (reg 0x26)
 *    LORA_REG_DETECT_OPT (0x03) was written to register 0x26, which is
 *    RegModemConfig3, not RegDetectOptimize.
 *      Reg 0x26 = RegModemConfig3:  bit3=LowDataRateOpt, bit2=AgcAutoOn
 *      Reg 0x31 = RegDetectOptimize: bits[2:0] = 0x03 for SF7–SF12
 *    Writing 0x03 to reg 0x26: AgcAutoOn=0 (reduced sensitivity at range),
 *    bits[1:0] set (reserved → undefined behaviour).
 *    Fix: write LORA_REG_MODEM_CFG3 (0x04 = AgcAutoOn) to reg 0x26.
 *         write LORA_REG_DETECT_OPT (0x03) to reg 0x31.
 *    Reg 0x31 default is already 0xC3 and reg 0x37 default is 0x0A,
 *    both correct for SF7–SF12, so explicit writes are added for clarity
 *    and robustness after any unexpected reset.
 *
 *  BUG 3 (COSMETIC) — LoRa_Reset() redundant initial HIGH pulse
 *    The reset pin is already LOW (MX_GPIO_Init default).
 *    Sequence corrected to: LOW(assert) → delay → HIGH(release) → delay.
 *
 * v3.3 — FIFO base-address bug fix (retained)
 * ==================================================================== */

#include "lora.h"
#include "lora_parser.h"
#include "device_id.h"
#include "stm32f1xx_hal.h"
#include "led.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

extern SPI_HandleTypeDef  hspi1;
extern UART_HandleTypeDef huart1;

/* ── UART helpers ───────────────────────────────────────────────────── */
static void uart_print_now(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 1000);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 1000);
}

/* ── Deferred log ring — flushed AFTER ACK is on air ───────────────── */
#define DEFER_LOG_LINES   8
#define DEFER_LOG_LEN     128
static char    s_deferLog[DEFER_LOG_LINES][DEFER_LOG_LEN];
static uint8_t s_deferHead  = 0;
static uint8_t s_deferCount = 0;

static LoRa_DiscoveredDevice_t s_discovered[LORA_DISCOVERY_MAX];
static uint8_t s_discoveredCount = 0;

static void log_defer(const char *fmt, ...)
{
    if (s_deferCount >= DEFER_LOG_LINES) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_deferLog[s_deferHead], DEFER_LOG_LEN, fmt, ap);
    va_end(ap);
    s_deferHead = (uint8_t)((s_deferHead + 1u) % DEFER_LOG_LINES);
    s_deferCount++;
}

static void log_flush(void)
{
    while (s_deferCount > 0)
    {
        uint8_t idx = (uint8_t)((s_deferHead - s_deferCount + DEFER_LOG_LINES)
                                % DEFER_LOG_LINES);
        uart_print_now(s_deferLog[idx]);
        s_deferCount--;
    }
}

/* ── Debug buffers for Live Expressions ─────────────────────────────── */
volatile char g_lastRxPacket[MAX_PACKET_LEN] = {0};
volatile char g_lastTxPacket[MAX_PACKET_LEN] = {0};

/* ── Public globals ─────────────────────────────────────────────────── */
uint8_t  loraMode            = LORA_MODE_RECEIVER;
uint8_t  g_loraConnected     = 0;
bool     g_loraNewPacketFlag = false;
uint32_t g_lora_tx_ok = 0, g_lora_tx_retry = 0, g_lora_tx_fail = 0;

/* ── Private state ──────────────────────────────────────────────────── */
static LoRa_ConnState_t s_state = LORA_STATE_DISCONNECTED;

static uint8_t  s_rxBuf[LORA_BUF_SIZE];
static uint32_t s_rxPacketCount = 0;

static uint8_t  s_wirelessLevel      = 0;
static uint8_t  s_wirelessWellDry    = 0;
static uint32_t s_wirelessLastRxTick = 0;
static bool     s_wirelessDataValid  = false;

static uint32_t s_lastSeq           = 0;
static bool     s_seqInitialized    = false;
static uint32_t s_packetsLost       = 0;
static uint32_t s_duplicatePackets  = 0;
static uint32_t s_outOfOrderPackets = 0;

#define LORA_CLEAR_PAIRING_ONCE  0   /* Set 1 once to wipe EEPROM, then set 0 */
#define LORA_DIO_DEBUG_ENABLE    1   /* Periodic DIO0/IRQ/OpMode UART dump     */

static bool     s_pairingMode       = false;
static uint32_t s_pairingDeadline   = 0;
static bool     s_pairingDone       = false;
static uint32_t s_lastPairedDID     = 0;

static uint32_t s_lastSyncReqTick   = 0;
static uint32_t s_lastStatsTick     = 0;
static uint32_t s_lastPeerTick      = 0;

/* ── NSS helpers ────────────────────────────────────────────────────── */
#define NSS_LOW()   HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET)
#define NSS_HIGH()  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET)

/* ── SX1278 register addresses ──────────────────────────────────────── */
#define REG_OPMODE              0x01
#define REG_FIFO                0x00
#define REG_FIFO_ADDR_PTR       0x0D
#define REG_FIFO_TX_BASE_ADDR   0x0E
#define REG_FIFO_RX_BASE_ADDR   0x0F
#define REG_FIFO_RX_CURRENT     0x10
#define REG_IRQ_FLAGS           0x12
#define REG_RX_NB_BYTES         0x13
#define REG_PKT_RSSI_VALUE      0x1A
#define REG_MODEM_CONFIG3       0x26   /* LowDataRateOpt / AgcAutoOn           */
#define REG_DETECT_OPTIMIZE     0x31   /* 0x03 for SF7–12, 0x05 for SF6        */
#define REG_DETECTION_THRESHOLD 0x37   /* 0x0A for SF7–12, 0x0C for SF6        */
#define REG_SYNC_WORD           0x39
#define REG_VERSION             0x42

/* ── RegOpMode values for 433 MHz (LowFrequencyModeOn = bit3 = 1) ──── *
 *                                                                       *
 *  SX1278 RegOpMode bit layout:                                         *
 *    bit7 : LongRangeMode      1 = LoRa                                 *
 *    bit6 : AccessSharedReg    0 = LoRa register page                   *
 *    bit5 : (reserved)         0                                        *
 *    bit4 : (reserved)         0                                        *
 *    bit3 : LowFrequencyModeOn 1 = low-band (<525 MHz)  ← REQUIRED     *
 *    bit2:0: Mode                                                        *
 *                                                                       *
 *  Without bit3=1 at 433 MHz, the PA and PLL use the high-band path    *
 *  and the radio will not modulate/demodulate correctly.                *
 *                                                                       *
 *  0x88 = 1000_1000 = LoRa + LowFreq + Sleep                           *
 *  0x89 = 1000_1001 = LoRa + LowFreq + Standby                         *
 *  0x8B = 1000_1011 = LoRa + LowFreq + TX                              *
 *  0x8D = 1000_1101 = LoRa + LowFreq + RX Continuous                   */
#define OPMODE_LORA_SLEEP    0x88u
#define OPMODE_LORA_STANDBY  0x89u
#define OPMODE_LORA_TX       0x8Bu
#define OPMODE_LORA_RXCONT   0x8Du

/* ── SPI primitives ─────────────────────────────────────────────────── */
void LoRa_WriteReg(uint8_t addr, uint8_t data)
{
    uint8_t tx[2] = { (uint8_t)(addr | 0x80u), data };
    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    NSS_HIGH();
}

uint8_t LoRa_ReadReg(uint8_t addr)
{
    uint8_t tx[2] = { (uint8_t)(addr & 0x7Fu), 0x00u };
    uint8_t rx[2] = { 0x00u, 0x00u };
    NSS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    NSS_HIGH();
    return rx[1];
}

static void LoRa_WriteBuffer(uint8_t addr, const uint8_t *buf, uint8_t size)
{
    uint8_t a = addr | 0x80u;
    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &a, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, size, HAL_MAX_DELAY);
    NSS_HIGH();
}

static void LoRa_ReadBuffer(uint8_t addr, uint8_t *buf, uint8_t size)
{
    uint8_t a = addr & 0x7Fu;
    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &a, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buf, size, HAL_MAX_DELAY);
    NSS_HIGH();
}

/* ── Hardware reset ─────────────────────────────────────────────────── *
 *  SX1278 datasheet: pull RST LOW for ≥ 100 µs, then release HIGH.    *
 *  PB6 (LORA_STATUS_Pin) is initialised LOW by MX_GPIO_Init, so the   *
 *  module is already in reset at startup. Assert → hold → release.     */
static void LoRa_Reset(void)
{
    /* Assert reset (LOW) */
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);   /* 10 ms >> 100 µs minimum */

    /* Release reset (HIGH) */
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(20);   /* 10 ms oscillator start-up per datasheet */
}

/* ── RX continuous mode ─────────────────────────────────────────────── */
static void LoRa_EnterRxContinuous(void)
{
    LoRa_WriteReg(REG_IRQ_FLAGS,    0xFF);
    LoRa_WriteReg(REG_FIFO_ADDR_PTR, 0x00);

    for (uint8_t i = 0; i < 5; i++)
    {
        LoRa_WriteReg(REG_OPMODE, OPMODE_LORA_RXCONT);
        HAL_Delay(2);

        if (LoRa_ReadReg(REG_OPMODE) == OPMODE_LORA_RXCONT)
            return;
    }

    uart_print_now("[LORA RX] ERROR: failed to enter RX continuous (0x8D)");
}

/* ── Non-blocking packet poll ───────────────────────────────────────── */
static uint8_t LoRa_PollPacket(uint8_t *buffer, int16_t *rssi_out)
{
    uint8_t irq = LoRa_ReadReg(REG_IRQ_FLAGS);

    if ((irq & 0x40u) == 0u)   /* RxDone not set */
    {
        if (irq & 0x20u)
            LoRa_WriteReg(REG_IRQ_FLAGS, 0x20u);
        return 0;
    }

    if (irq & 0x20u)            /* CRC error */
    {
        LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
        return 0;
    }

    uint8_t len = LoRa_ReadReg(REG_RX_NB_BYTES);

    if (len == 0u || len >= (LORA_BUF_SIZE - 1u))
    {
        LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
        return 0;
    }

    uint8_t fifoAddr = LoRa_ReadReg(REG_FIFO_RX_CURRENT);
    LoRa_WriteReg(REG_FIFO_ADDR_PTR, fifoAddr);
    LoRa_ReadBuffer(REG_FIFO, buffer, len);
    buffer[len] = '\0';

    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);

    if (rssi_out != NULL)
        *rssi_out = (int16_t)(-157 + (int16_t)LoRa_ReadReg(REG_PKT_RSSI_VALUE));

    return len;
}

/* ── Fast TX reply → immediately back to RX ────────────────────────── */
static void LoRa_FastReply(const char *pkt, uint8_t len)
{
    if (pkt == NULL || len == 0u || len >= LORA_BUF_SIZE) return;

    if (len < MAX_PACKET_LEN)
    {
        memcpy((void *)g_lastTxPacket, pkt, len);
        g_lastTxPacket[len] = '\0';
    }

    LoRa_WriteReg(REG_OPMODE,         OPMODE_LORA_STANDBY);
    LoRa_WriteReg(REG_FIFO_ADDR_PTR,  0x00);
    LoRa_WriteBuffer(REG_FIFO, (const uint8_t *)pkt, len);
    LoRa_WriteReg(0x22, len);             /* RegPayloadLength */
    LoRa_WriteReg(REG_IRQ_FLAGS,      0xFF);
    LoRa_WriteReg(REG_OPMODE,         OPMODE_LORA_TX);

    uint32_t t0 = HAL_GetTick();
    while ((LoRa_ReadReg(REG_IRQ_FLAGS) & 0x08u) == 0u)
    {
        if ((HAL_GetTick() - t0) > LORA_TX_TIMEOUT_MS) break;
    }
    LoRa_WriteReg(REG_IRQ_FLAGS, 0x08u);

    LoRa_EnterRxContinuous();
}

/* ── Reply builders ─────────────────────────────────────────────────── */
static void send_ack(uint32_t did)
{
    char buf[24];
    int n = LoRa_BuildAck(buf, sizeof(buf), did);
    if (n > 0) LoRa_FastReply(buf, (uint8_t)n);
}

static void send_pong(uint32_t did, uint32_t seq)
{
    char buf[32];
    int n = LoRa_BuildPong(buf, sizeof(buf), did, seq);
    if (n > 0) LoRa_FastReply(buf, (uint8_t)n);
}

static void send_reject(uint32_t did)
{
    char buf[24];
    int n = LoRa_BuildReject(buf, sizeof(buf), did);
    if (n > 0) LoRa_FastReply(buf, (uint8_t)n);
}

static void send_sync_req(void)
{
    char buf[24];
    int n = LoRa_BuildSyncReq(buf, sizeof(buf), DeviceID_GetOwn());
    if (n > 0) LoRa_FastReply(buf, (uint8_t)n);
}

/* ── State helpers ──────────────────────────────────────────────────── */
const char *LoRa_GetStateString(void)
{
    switch (s_state)
    {
        case LORA_STATE_DISCONNECTED: return "WAITING";
        case LORA_STATE_HANDSHAKING:  return "HANDSHAKING";
        case LORA_STATE_CONNECTED:    return "CONNECTED";
        case LORA_STATE_STALE:        return "STALE";
        default:                      return "?";
    }
}

LoRa_ConnState_t LoRa_GetState(void) { return s_state; }

static void set_state(LoRa_ConnState_t ns)
{
    if (s_state == ns) return;
    s_state = ns;
    g_loraConnected = (ns == LORA_STATE_CONNECTED) ? 1u : 0u;
    log_defer("[LORA RX] STATE -> %s", LoRa_GetStateString());
}

/* ── Pairing helpers ────────────────────────────────────────────────── */
static bool already_in_list(uint32_t did)
{
    uint8_t cnt = PairedDev_Count();
    for (uint8_t i = 0; i < cnt; i++)
        if (PairedDev_Get(i) == did) return true;
    return false;
}

static bool should_allow(uint32_t did)
{
    if (!LoRa_DID_IsValid(did)) return false;
    if (s_pairingMode)          return true;
    return PairedDev_IsAllowed(did);
}

void LoRa_ClearDiscoveredDevices(void)
{
    memset(s_discovered, 0, sizeof(s_discovered));
    s_discoveredCount = 0;
}

static void discovery_add_or_update(uint32_t did, int16_t rssi)
{
    if (!LoRa_DID_IsValid(did)) return;
    if (already_in_list(did))   return;

    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < s_discoveredCount; i++)
    {
        if (s_discovered[i].valid && s_discovered[i].did == did)
        {
            s_discovered[i].rssi     = rssi;
            s_discovered[i].lastSeen = now;
            return;
        }
    }

    if (s_discoveredCount < LORA_DISCOVERY_MAX)
    {
        s_discovered[s_discoveredCount].did      = did;
        s_discovered[s_discoveredCount].rssi     = rssi;
        s_discovered[s_discoveredCount].lastSeen = now;
        s_discovered[s_discoveredCount].valid    = true;
        s_discoveredCount++;
    }
}

uint8_t LoRa_GetDiscoveredCount(void)           { return s_discoveredCount; }

bool LoRa_GetDiscoveredDevice(uint8_t index, LoRa_DiscoveredDevice_t *out)
{
    if (!out || index >= s_discoveredCount || !s_discovered[index].valid)
        return false;
    *out = s_discovered[index];
    return true;
}

static void pair_device(uint32_t did)
{
    if (!LoRa_DID_IsValid(did)) return;

    s_lastPairedDID = did;

    if (already_in_list(did))
    {
        s_pairingDone = s_pairingMode = false;
        s_seqInitialized = false;
        s_lastSeq = 0;
        return;
    }

    if (PairedDev_Add(did))
    {
        s_pairingDone = true;
        s_pairingMode = false;
        s_seqInitialized = false;
        s_lastSeq = 0;
        log_defer("[LORA RX] *** PAIRED DID %08lX (%u in list) ***",
                  (unsigned long)did, (unsigned)PairedDev_Count());
    }
    else
    {
        log_defer("[LORA RX] PairedDev_Add failed");
    }
}

bool LoRa_PairDiscoveredDevice(uint8_t index)
{
    if (index >= s_discoveredCount || !s_discovered[index].valid)
        return false;

    uint32_t did = s_discovered[index].did;
    pair_device(did);

    s_pairingDone   = true;
    s_pairingMode   = false;
    s_lastPairedDID = did;

    LoRa_ClearDiscoveredDevices();
    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
    LoRa_EnterRxContinuous();
    return true;
}

void LoRa_EnterPairingMode(void)
{
    LoRa_ClearDiscoveredDevices();

    s_pairingMode     = true;
    s_pairingDone     = false;
    s_lastPairedDID   = 0;
    s_pairingDeadline = HAL_GetTick() + RX_PAIRING_TIMEOUT_MS;

    s_wirelessDataValid = false;
    s_seqInitialized    = false;
    s_lastSeq           = 0;

    set_state(LORA_STATE_DISCONNECTED);

    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
    LoRa_EnterRxContinuous();

    uart_print_now("[LORA RX] *** MANUAL PAIRING DISCOVERY ACTIVE ***");
    LED_SetIntent(LED_COLOR_BLUE, LED_MODE_BLINK, 200);
}

void LoRa_ExitPairingMode(void)
{
    s_pairingMode = false;
    LoRa_ClearDiscoveredDevices();
    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
    LoRa_EnterRxContinuous();
}

bool     LoRa_IsPairingMode    (void) { return s_pairingMode;    }
bool     LoRa_IsPairingComplete(void) { return s_pairingDone;    }
uint32_t LoRa_GetLastPairedDID (void) { return s_lastPairedDID;  }

/* ── Wireless data accessors ────────────────────────────────────────── */
uint8_t  LoRa_GetWirelessTankLevel(void) { return s_wirelessLevel;   }
uint8_t  LoRa_GetWirelessWellDry  (void) { return s_wirelessWellDry; }
uint32_t LoRa_GetLastSequence     (void) { return s_lastSeq;         }
uint32_t LoRa_GetPacketsLost      (void) { return s_packetsLost;     }

bool LoRa_IsWirelessDataValid(void)
{
    if (!s_wirelessDataValid) return false;

    if (PairedDev_Count() == 0u)
    {
        LoRa_ClearWirelessData();
        return false;
    }

    if ((HAL_GetTick() - s_wirelessLastRxTick) > RX_PEER_TIMEOUT_MS)
    {
        s_wirelessDataValid = false;
        s_wirelessWellDry   = 0;

        if (s_state == LORA_STATE_CONNECTED)
        {
            set_state(LORA_STATE_STALE);
            uart_print_now("[LORA RX] !!! peer timeout — STALE");
            LED_SetIntent(LED_COLOR_RED, LED_MODE_BLINK, 500);
            s_seqInitialized = false;
        }
        return false;
    }

    return true;
}

void LoRa_OnPairingListChanged(void)
{
    LoRa_ClearWirelessData();

    s_pairingMode     = false;
    s_pairingDone     = false;
    s_lastPairedDID   = 0;
    s_lastPeerTick    = 0;
    s_lastSyncReqTick = 0;

    LoRa_ClearDiscoveredDevices();

    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
    LoRa_EnterRxContinuous();

    if (PairedDev_Count() == 0u)
        uart_print_now("[LORA RX] No TX paired — using LOCAL ADC only");
    else
        uart_print_now("[LORA RX] Pairing list changed — waiting for saved TX only");

    LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_STEADY, 1);
}

/* ── Sequence validation ────────────────────────────────────────────── */
static bool validate_sequence(uint32_t newSeq)
{
    if (!s_seqInitialized)
    {
        s_lastSeq        = newSeq;
        s_seqInitialized = true;
        return true;
    }

    if (newSeq == s_lastSeq)
    {
        s_duplicatePackets++;
        log_defer("[LORA RX] dup seq %08lX", (unsigned long)newSeq);
        return false;
    }

    if (newSeq < s_lastSeq)
    {
        if ((s_lastSeq - newSeq) > 0x10000UL)
        {
            log_defer("[LORA RX] seq jumped back — TX restarted, resync");
            s_lastSeq        = newSeq;
            s_seqInitialized = true;
            return true;
        }
        s_outOfOrderPackets++;
        return false;
    }

    if (newSeq > s_lastSeq + 1u)
    {
        uint32_t lost = newSeq - (s_lastSeq + 1u);
        s_packetsLost += lost;
        log_defer("[LORA RX] gap: lost %lu pkt(s)", (unsigned long)lost);
    }

    s_lastSeq = newSeq;
    return true;
}

static void accept_data(uint8_t lvl, uint8_t wd, uint32_t did)
{
    if (PairedDev_Count() == 0u || !PairedDev_IsAllowed(did))
    {
        LoRa_ClearWirelessData();
        log_defer("[LORA RX] blocked data from unpaired/removed DID=%08lX",
                  (unsigned long)did);
        return;
    }

    s_wirelessLevel      = lvl;
    s_wirelessWellDry    = wd;
    s_wirelessLastRxTick = HAL_GetTick();
    s_wirelessDataValid  = true;
    s_lastPeerTick       = HAL_GetTick();
    g_loraNewPacketFlag  = true;

    if (s_state != LORA_STATE_CONNECTED)
        set_state(LORA_STATE_CONNECTED);
}

/* ====================================================================
 *  LoRa_Init
 *  Full SX1278 configuration for 433 MHz LoRa RX.
 * ==================================================================== */
void LoRa_Init(void)
{
    char dbg[160];

    uart_print_now("");
    uart_print_now("=========================================");
    uart_print_now("  HELONIX RX  —  LoRa 433MHz init v3.4");
    uart_print_now("=========================================");

    DeviceID_Init();
    PairedDev_Init();

#if LORA_CLEAR_PAIRING_ONCE
    uart_print_now("[LORA RX] TEMP: clearing paired device EEPROM...");
    PairedDev_Clear();
    HAL_Delay(20);
    PairedDev_Init();
#endif

    char myDIDstr[9];
    DeviceID_GetHex(myDIDstr);
    snprintf(dbg, sizeof(dbg), "[LORA RX] My DID : %s", myDIDstr);
    uart_print_now(dbg);
    snprintf(dbg, sizeof(dbg), "[LORA RX] Paired : %u device(s) in EEPROM",
             (unsigned)PairedDev_Count());
    uart_print_now(dbg);

    if (PairedDev_Count() == 0u)
    {
        uart_print_now("[LORA RX] List empty — AUTO-PAIR first valid HELLO/TL");
    }
    else
    {
        for (uint8_t i = 0; i < PairedDev_Count(); i++)
        {
            snprintf(dbg, sizeof(dbg), "[LORA RX]   [%u] DID %08lX",
                     i, (unsigned long)PairedDev_Get(i));
            uart_print_now(dbg);
        }
    }

    /* ── Hardware reset ─────────────────────────────────────────────── */
    NSS_HIGH();
    HAL_Delay(20);

    LoRa_Reset();
    HAL_Delay(20);

    /* ── SPI diagnostic — read RegVersion three times ───────────────── */
    uint8_t ver1 = LoRa_ReadReg(REG_VERSION); HAL_Delay(2);
    uint8_t ver2 = LoRa_ReadReg(REG_VERSION); HAL_Delay(2);
    uint8_t ver3 = LoRa_ReadReg(REG_VERSION);
    uint8_t ver  = ver3;

    snprintf(dbg, sizeof(dbg),
             "[LORA RX] RegVersion reads: 0x%02X 0x%02X 0x%02X",
             ver1, ver2, ver3);
    uart_print_now(dbg);
    snprintf(dbg, sizeof(dbg),
             "[LORA RX] RegVersion(0x42) = 0x%02X  (expected 0x12)", ver);
    uart_print_now(dbg);

    if (ver != 0x12)
    {
        uart_print_now("[LORA RX] *** SPI LINK BROKEN — radio unreachable ***");
        uart_print_now("[LORA RX]   0xFF = NSS/MISO floating");
        uart_print_now("[LORA RX]   0x00 = MISO pulled low / power issue");
        uart_print_now("[LORA RX] Check: NSS=PA15  SCK=PB3  MISO=PB4  MOSI=PB5  RST=PB6");
        uart_print_now("[LORA RX]        SPI1 remap active? Ra-02 power 3.3V?");
    }

    /* ── Mode transition: FSK sleep → LoRa sleep ───────────────────── *
     *  Must go through sleep before switching LongRangeMode.           *
     *  Use LowFrequencyModeOn=1 from the very first write.             */
    LoRa_WriteReg(REG_OPMODE, 0x00);            /* FSK sleep (clear all)  */
    HAL_Delay(2);
    LoRa_WriteReg(REG_OPMODE, OPMODE_LORA_SLEEP); /* LoRa + LowFreq + sleep */
    HAL_Delay(10);

    /* ── Frequency: 433.000 MHz ─────────────────────────────────────── *
     *  Frf = Fstep * Frf_reg = (32e6/2^19) * Frf_reg                  *
     *  Frf_reg = 433e6 * 2^19 / 32e6 = 0x6C4000                       */
    uint64_t frf = ((uint64_t)433000000ULL << 19) / 32000000ULL;
    LoRa_WriteReg(0x06, (uint8_t)(frf >> 16));   /* RegFrMsb = 0x6C      */
    LoRa_WriteReg(0x07, (uint8_t)(frf >>  8));   /* RegFrMid = 0x40      */
    LoRa_WriteReg(0x08, (uint8_t)(frf      ));   /* RegFrLsb = 0x00      */

    /* ── Preamble length: 8 symbols ────────────────────────────────── */
    LoRa_WriteReg(0x20, LORA_PREAMBLE_MSB);
    LoRa_WriteReg(0x21, LORA_PREAMBLE_LSB);

    /* ── PA / OCP / DAC ────────────────────────────────────────────── */
    LoRa_WriteReg(0x09, LORA_REG_PA_CONFIG);     /* PA_BOOST, Pout=14dBm */
    LoRa_WriteReg(0x0B, LORA_REG_OCP);           /* OCP ~100 mA          */
    LoRa_WriteReg(0x4D, LORA_REG_PA_DAC);        /* default PA DAC       */

    /* ── Modem configuration ────────────────────────────────────────── */
    LoRa_WriteReg(0x1D, LORA_REG_MODEM_CFG1);    /* BW=125k CR=4/5 ExpHdr*/
    LoRa_WriteReg(0x1E, LORA_REG_MODEM_CFG2);    /* SF7, CRC on          */

    /* Reg 0x26 = RegModemConfig3: AgcAutoOn=1 (bit2), LowDataRateOpt=0 *
     *   LORA_REG_MODEM_CFG3 = 0x04 = 0b00000100                        *
     *   (was incorrectly written with LORA_REG_DETECT_OPT=0x03)        */
    LoRa_WriteReg(REG_MODEM_CONFIG3, LORA_REG_MODEM_CFG3);

    /* Reg 0x31 = RegDetectOptimize: 0x03 for SF7–SF12                 *
     *   (was incorrectly written to reg 0x26 in previous version)      */
    LoRa_WriteReg(REG_DETECT_OPTIMIZE, LORA_REG_DETECT_OPT);

    /* Reg 0x37 = RegDetectionThreshold: 0x0A for SF7–SF12             *
     *   Default is already 0x0A; explicit write for robustness.        */
    LoRa_WriteReg(REG_DETECTION_THRESHOLD, LORA_REG_DETECTION_TH);

    /* Reg 0x39 = RegSyncWord: private network sync word               */
    LoRa_WriteReg(REG_SYNC_WORD, LORA_SYNC_WORD);

    /* ── FIFO base addresses (v3.3 fix retained) ───────────────────── */
    LoRa_WriteReg(REG_FIFO_TX_BASE_ADDR, 0x00);
    LoRa_WriteReg(REG_FIFO_RX_BASE_ADDR, 0x00);
    LoRa_WriteReg(REG_FIFO_ADDR_PTR,     0x00);

    /* ── Clear all IRQ flags ────────────────────────────────────────── */
    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);

    /* ── Verify key registers ───────────────────────────────────────── */
    uint8_t v_tx  = LoRa_ReadReg(REG_FIFO_TX_BASE_ADDR);
    uint8_t v_rx  = LoRa_ReadReg(REG_FIFO_RX_BASE_ADDR);
    uint8_t v_op  = LoRa_ReadReg(REG_OPMODE);
    uint8_t v_mc3 = LoRa_ReadReg(REG_MODEM_CONFIG3);
    uint8_t v_dop = LoRa_ReadReg(REG_DETECT_OPTIMIZE);

    snprintf(dbg, sizeof(dbg),
             "[LORA RX] Verify: OpMode=0x%02X TxBase=0x%02X RxBase=0x%02X",
             v_op, v_tx, v_rx);
    uart_print_now(dbg);
    snprintf(dbg, sizeof(dbg),
             "[LORA RX] Verify: ModemCfg3=0x%02X(want 0x04) DetOpt=0x%02X(want 0x03)",
             v_mc3, v_dop);
    uart_print_now(dbg);

    /* ── Enter RX continuous ────────────────────────────────────────── */
    LoRa_EnterRxContinuous();
    HAL_Delay(5);

    uint8_t v_op2 = LoRa_ReadReg(REG_OPMODE);
    snprintf(dbg, sizeof(dbg),
             "[LORA RX] OpMode after EnterRxContinuous: 0x%02X (want 0x%02X)",
             v_op2, OPMODE_LORA_RXCONT);
    uart_print_now(dbg);

    /* ── Reset software state ───────────────────────────────────────── */
    s_state             = LORA_STATE_DISCONNECTED;
    g_loraConnected     = 0;
    g_loraNewPacketFlag = false;

    s_rxPacketCount     = 0;
    s_seqInitialized    = false;
    s_lastSeq           = 0;
    s_packetsLost       = 0;
    s_duplicatePackets  = 0;
    s_outOfOrderPackets = 0;

    s_wirelessDataValid  = false;
    s_wirelessLevel      = 0;
    s_wirelessWellDry    = 0;
    s_wirelessLastRxTick = 0;

    s_pairingMode    = false;
    s_pairingDone    = false;
    s_lastPairedDID  = 0;

    s_lastSyncReqTick = 0;
    s_lastStatsTick   = HAL_GetTick();
    s_lastPeerTick    = 0;

    memset((void *)g_lastRxPacket, 0, sizeof(g_lastRxPacket));
    memset((void *)g_lastTxPacket, 0, sizeof(g_lastTxPacket));

    uart_print_now("[LORA RX] Init complete — listening for transmitter...");
    LED_SetIntent(LED_COLOR_PURPLE, LED_MODE_STEADY, 1);
}

void LoRa_ClearWirelessData(void)
{
    s_wirelessLevel      = 0;
    s_wirelessWellDry    = 0;
    s_wirelessDataValid  = false;
    s_wirelessLastRxTick = 0;
    s_lastSeq            = 0;
    s_seqInitialized     = false;
    g_loraConnected      = 0;
    set_state(LORA_STATE_DISCONNECTED);
}

/* ── Periodic STALE handler ─────────────────────────────────────────── */
static void handle_stale_periodics(uint32_t now)
{
    if (s_state != LORA_STATE_STALE) return;

    if ((now - s_lastSyncReqTick) >= RX_SYNC_REQ_INTERVAL_MS)
    {
        s_lastSyncReqTick = now;
        send_sync_req();
        uart_print_now("[LORA RX] STALE — sent @SY (waiting for TX HELLO)");
    }
}

/* ── Per-cycle stats (every 30 s) ───────────────────────────────────── */
static void maybe_print_stats(uint32_t now)
{
    if ((now - s_lastStatsTick) < 30000UL) return;
    s_lastStatsTick = now;

    char dbg[160];
    snprintf(dbg, sizeof(dbg),
             "[LORA RX] stats rx=%lu lost=%lu dup=%lu lvl=%u%% wd=%u state=%s paired=%u",
             (unsigned long)s_rxPacketCount,
             (unsigned long)s_packetsLost,
             (unsigned long)s_duplicatePackets,
             (unsigned)s_wirelessLevel,
             (unsigned)s_wirelessWellDry,
             LoRa_GetStateString(),
             (unsigned)PairedDev_Count());
    uart_print_now(dbg);
}

/* ====================================================================
 *  LoRa_Task  — call from main loop every iteration
 * ==================================================================== */
void LoRa_Task(void)
{
    if (loraMode != LORA_MODE_RECEIVER) return;

    uint32_t now = HAL_GetTick();

#if LORA_DIO_DEBUG_ENABLE
    static uint32_t lastDioDebugTick = 0;

    if ((now - lastDioDebugTick) >= 2000UL)
    {
        lastDioDebugTick = now;

        char d[120];
        uint8_t dio0 = (uint8_t)HAL_GPIO_ReadPin(LORA_DIO0_PORT, LORA_DIO0_PIN);
        uint8_t irq  = LoRa_ReadReg(REG_IRQ_FLAGS);
        uint8_t op   = LoRa_ReadReg(REG_OPMODE);
        uint8_t rssi = LoRa_ReadReg(REG_PKT_RSSI_VALUE);

        snprintf(d, sizeof(d),
                 "[LORA RX] DIO0=%u IRQ=0x%02X OP=0x%02X(want 0x%02X) RSSI_REG=0x%02X",
                 dio0, irq, op, OPMODE_LORA_RXCONT, rssi);
        uart_print_now(d);

        if (op != OPMODE_LORA_RXCONT)
        {
            uart_print_now("[LORA RX] WARNING: OpMode changed — restoring RX continuous");
            LoRa_EnterRxContinuous();
        }
    }
#endif

    if (s_wirelessDataValid)
        (void)LoRa_IsWirelessDataValid();

    if (s_pairingMode && (now >= s_pairingDeadline))
    {
        s_pairingMode = false;
        uart_print_now("[LORA RX] pairing window expired — resuming RX continuous");
        LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
        LoRa_EnterRxContinuous();
    }

    handle_stale_periodics(now);
    maybe_print_stats(now);

    int16_t rssi = 0;
    uint8_t len = LoRa_PollPacket(s_rxBuf, &rssi);

    if (len == 0u)
    {
        log_flush();
        return;
    }

    s_rxBuf[len] = '\0';
    s_rxPacketCount++;

    if (len < MAX_PACKET_LEN)
        memcpy((void *)g_lastRxPacket, s_rxBuf, (size_t)(len + 1u));

    ParsedPacket_t p;
    bool ok = LoRa_ParsePacket((const char *)s_rxBuf, &p);

    log_defer("[LORA RX] rx#%lu rssi=%d state=%s raw=\"%s\"",
              (unsigned long)s_rxPacketCount, (int)rssi,
              LoRa_GetStateString(), (char *)s_rxBuf);

    if (!ok)
    {
        log_defer("[LORA RX] parse failed — ignored");
        log_flush();
        return;
    }

    switch (p.type)
    {
    case PKT_TYPE_HELLO:
    {
        if (s_pairingMode)
        {
            discovery_add_or_update(p.did, rssi);
            send_ack(p.did);
            log_defer("[LORA RX] DISCOVERED HELLO DID=%08lX RSSI=%d",
                      (unsigned long)p.did, (int)rssi);
            g_loraNewPacketFlag = true;
            break;
        }

        /* ── Auto-pair: accept first transmitter when list is empty ── *
         *  When no transmitter has been paired yet (fresh device or    *
         *  after clear), automatically pair the first valid HELLO.     *
         *  This matches the init banner "AUTO-PAIR first valid HELLO". *
         *  Once paired, normal should_allow() logic applies.           */
        if (PairedDev_Count() == 0u && LoRa_DID_IsValid(p.did))
        {
            log_defer("[LORA RX] AUTO-PAIR: empty list — pairing DID %08lX",
                      (unsigned long)p.did);
            pair_device(p.did);          /* saves to EEPROM              */
            s_lastPeerTick      = now;
            s_wirelessDataValid = false;
            s_seqInitialized    = false;
            s_lastSeq           = 0;
            send_ack(p.did);             /* ACK immediately              */
            set_state(LORA_STATE_CONNECTED);
            g_loraNewPacketFlag = true;
            LED_SetIntent(LED_COLOR_GREEN, LED_MODE_BLINK, 300);
            log_defer("[LORA RX] AUTO-PAIRED DID %08lX -> ACK CONNECTED",
                      (unsigned long)p.did);
            break;
        }

        if (should_allow(p.did))
        {
            s_lastPeerTick      = now;
            s_wirelessDataValid = false;
            s_seqInitialized    = false;
            s_lastSeq           = 0;
            send_ack(p.did);
            set_state(LORA_STATE_CONNECTED);
            log_defer("[LORA RX] HELLO from paired DID %08lX -> ACK CONNECTED",
                      (unsigned long)p.did);
            LED_SetIntent(LED_COLOR_GREEN, LED_MODE_BLINK, 300);
        }
        else
        {
            send_reject(p.did);
            log_defer("[LORA RX] HELLO from unpaired DID %08lX -> REJECT",
                      (unsigned long)p.did);
        }
        break;
    }

    case PKT_TYPE_TANKLEVEL:
    {
        if (s_pairingMode)
        {
            discovery_add_or_update(p.did, rssi);
            send_ack(p.did);
            log_defer("[LORA RX] DISCOVERED TL DID=%08lX RSSI=%d",
                      (unsigned long)p.did, (int)rssi);
            g_loraNewPacketFlag = true;
            break;
        }

        /* Auto-pair on TANKLEVEL too — transmitter may send data before HELLO */
        if (PairedDev_Count() == 0u && LoRa_DID_IsValid(p.did))
        {
            log_defer("[LORA RX] AUTO-PAIR on TL: empty list — pairing DID %08lX",
                      (unsigned long)p.did);
            pair_device(p.did);
        }

        if (!should_allow(p.did))
        {
            send_reject(p.did);
            log_defer("[LORA RX] TL from unpaired DID %08lX -> REJECT",
                      (unsigned long)p.did);
            break;
        }

        bool fresh = validate_sequence(p.seq);
        send_ack(p.did);   /* ACK even duplicates to stop TX retry */

        if (fresh)
        {
            accept_data(p.level, p.well_dry, p.did);
            set_state(LORA_STATE_CONNECTED);
            s_lastPeerTick = now;
            LED_SetIntent(LED_COLOR_GREEN, LED_MODE_STEADY, 1);
            log_defer("[LORA RX] TL=%u%% WD=%u seq=%08lX DID=%08lX -> ACCEPTED",
                      (unsigned)p.level, (unsigned)p.well_dry,
                      (unsigned long)p.seq, (unsigned long)p.did);
        }
        else
        {
            s_lastPeerTick = now;
            log_defer("[LORA RX] TL dup/OOO seq=%08lX -> re-ACKed",
                      (unsigned long)p.seq);
        }
        break;
    }

    case PKT_TYPE_PING:
    {
        if (!should_allow(p.did))
        {
            send_reject(p.did);
            log_defer("[LORA RX] PING from unpaired DID %08lX -> REJECT",
                      (unsigned long)p.did);
            break;
        }

        send_pong(p.did, p.seq);
        s_lastPeerTick = now;

        if (s_state != LORA_STATE_CONNECTED)
            set_state(LORA_STATE_CONNECTED);

        log_defer("[LORA RX] PING from %08lX -> PONG seq=%08lX",
                  (unsigned long)p.did, (unsigned long)p.seq);
        break;
    }

    case PKT_TYPE_BYE:
    {
        log_defer("[LORA RX] BYE from %08lX -> STALE", (unsigned long)p.did);
        s_wirelessDataValid = false;
        set_state(LORA_STATE_STALE);
        LED_SetIntent(LED_COLOR_RED, LED_MODE_BLINK, 500);
        break;
    }

    default:
    {
        log_defer("[LORA RX] unexpected packet type %d from DID=%08lX",
                  (int)p.type, (unsigned long)p.did);
        break;
    }
    }

    log_flush();
}
