/* ====================================================================
 * lora_protocol.h  —  SHARED PROTOCOL DEFINITIONS
 *
 * This header defines the complete packet protocol used between the
 * Transmitter (tank-level sensor node) and Receiver (motor controller).
 *
 * Protocol design goals:
 *   1. Reliable handshake before any data flows
 *   2. Stop sending DATA when disconnected — send HELLO instead
 *   3. Periodic PING keep-alive when no DATA changes
 *   4. Bidirectional ACK on every meaningful packet
 *   5. Sequence numbers for loss/duplicate/reorder detection
 *   6. Long-range LoRa configuration (SF10, +20 dBm)
 *   7. Graceful disconnect detection on both ends
 *
 * Packet alphabet (all framed with @ ... #):
 *
 *   ┌─────────────────┬──────────────────────────────────────────────┐
 *   │ Direction       │ Format                                       │
 *   ├─────────────────┼──────────────────────────────────────────────┤
 *   │ TX  → RX (init) │ @HI:<DID>#                                   │
 *   │ RX  → TX (resp) │ @ACK:<DID>#                                  │
 *   │ RX  → TX (resp) │ @REJ:<DID>#       (unknown / not paired)     │
 *   │ TX  → RX (data) │ @TL:<lvl>,WD:<dry>,SQ:<seq>,DID:<did>#       │
 *   │ TX  → RX (idle) │ @PI:<DID>,SQ:<seq>#                          │
 *   │ RX  → TX (resp) │ @PO:<DID>,SQ:<seq>#                          │
 *   │ RX  → TX (req)  │ @SY:<DID>#        (RX asks TX to re-HELLO)   │
 *   │ TX  → RX (info) │ @BY:<DID>#        (graceful goodbye)         │
 *   └─────────────────┴──────────────────────────────────────────────┘
 *
 *   <DID> = 8-hex-char Device ID (silicon UID derived)
 *   <lvl> = 3 ASCII digits, 000–100 (tank level %)
 *   <dry> = 1 digit, 0=water present, 1=dry
 *   <seq> = 8-hex-char monotonic sequence (resets on power cycle)
 *
 * State machines:
 *
 *   TX side:                      RX side:
 *   ┌──────────────┐              ┌──────────────┐
 *   │ DISCONNECTED │              │ WAITING      │
 *   └──────┬───────┘              └──────┬───────┘
 *          │ send @HI                    │ got @HI from paired DID
 *          │ wait @ACK                   │ → reply @ACK
 *          ▼                             ▼
 *   ┌──────────────┐              ┌──────────────┐
 *   │ HANDSHAKING  │ ──ack──►     │ CONNECTED    │
 *   └──────┬───────┘              └──────┬───────┘
 *          │ ack rx                      │ got @TL or @PI
 *          ▼                             │ → reply @ACK or @PO
 *   ┌──────────────┐                     │ no rx > TIMEOUT
 *   │ CONNECTED    │                     ▼
 *   └──────┬───────┘              ┌──────────────┐
 *          │ data ready: send @TL │ STALE        │
 *          │ idle > KEEPALIVE:    └──────────────┘
 *          │   send @PI                  on next packet, send @SY
 *          │ N consecutive no-ACK:       to force TX to re-HELLO
 *          │   → DISCONNECTED
 *          ▼
 *   (loops)
 *
 * ==================================================================== */

#ifndef LORA_PROTOCOL_H
#define LORA_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* ── Packet size limits ─────────────────────────────────────────────── */
#define LORA_BUF_SIZE                 96U
#define MAX_PACKET_LEN                64U   /* must fit longest packet */

/* Packet prefixes — kept short to fit timing budget */
#define PFX_HELLO                     "@HI:"
#define PFX_ACK                       "@ACK:"
#define PFX_REJECT                    "@REJ:"
#define PFX_TANKLEVEL                 "@TL:"
#define PFX_PING                      "@PI:"
#define PFX_PONG                      "@PO:"
#define PFX_SYNC_REQ                  "@SY:"
#define PFX_BYE                       "@BY:"

#define PKT_TERMINATOR                '#'
#define PKT_OPENER                    '@'

/* Packet type enum (parsed from prefix) */
typedef enum
{
    PKT_TYPE_UNKNOWN = 0,
    PKT_TYPE_HELLO,
    PKT_TYPE_ACK,
    PKT_TYPE_REJECT,
    PKT_TYPE_TANKLEVEL,
    PKT_TYPE_PING,
    PKT_TYPE_PONG,
    PKT_TYPE_SYNC_REQ,
    PKT_TYPE_BYE
} PacketType_t;

/* Parsed packet contents */
typedef struct
{
    PacketType_t type;
    uint32_t     did;       /* device id of sender */
    uint32_t     seq;       /* sequence (TANKLEVEL/PING/PONG only) */
    uint8_t      level;     /* tank level (TANKLEVEL only) */
    uint8_t      well_dry;  /* well dry flag (TANKLEVEL only) */
} ParsedPacket_t;

/* ── Connection state machine ───────────────────────────────────────── */
typedef enum
{
    LORA_STATE_DISCONNECTED = 0,  /* TX: no peer, send HELLO            */
    LORA_STATE_HANDSHAKING,       /* TX: HELLO sent, waiting first ACK  */
    LORA_STATE_CONNECTED,         /* TX/RX: peer alive, traffic flowing */
    LORA_STATE_STALE              /* RX: timeout, will request re-sync  */
} LoRa_ConnState_t;

/* ── Long-range LoRa radio configuration ────────────────────────────── *
 *
 * Default operating point optimised for "larger range" requirement.
 *
 * SF10 / BW125 / CR4/5 / +20 dBm gives:
 *   • Sensitivity:  ≈ -132 dBm (vs -123 dBm at SF7)
 *   • Symbol time:  8.192 ms
 *   • Data rate  :  ≈ 980 bps
 *   • Range gain over SF7: roughly 3–4× line-of-sight
 *
 * Time-on-air for a 40-byte payload at SF10/BW125/CR4/5/explicit hdr/
 * 12-symbol preamble ≈ 660 ms.  Add 20 ms RX-switch and your ACK round
 * trip is ≈ 1.4 s.  Hence ACK_WAIT_MS = 1800.
 *
 * If you need even more range, set LORA_LONG_RANGE_MAX = 1 to use SF12.
 * Be aware airtime then doubles and ACK_WAIT must rise to ~3500 ms.
 * ──────────────────────────────────────────────────────────────────── */

#define LORA_LONG_RANGE_MAX       0       /* 0 = SF10, 1 = SF12 */

#if LORA_LONG_RANGE_MAX
  /* SF12 / BW125 — maximum range, slowest                             */
  #define LORA_REG_MODEM_CFG2     0xC4    /* SF12 + CRC ON              */
  #define LORA_REG_SYMB_TIMEOUT   0x08    /* with msb in MODEM_CFG2[0:1]*/
  #define LORA_REG_DETECT_OPT     0x0C    /* LowDataRateOpt=1, AGC auto */
  #define LORA_ACK_WAIT_MS        3500UL
  #define LORA_TX_TIMEOUT_MS      3500UL
  #define LORA_HELLO_ACK_WAIT_MS  3500UL
#else
  /* SF10 / BW125 — long range, sensible airtime  (recommended)        */
  #define LORA_REG_MODEM_CFG2     0xA4    /* SF10 + CRC ON              */
  #define LORA_REG_SYMB_TIMEOUT   0x08
  #define LORA_REG_DETECT_OPT     0x0C    /* LowDataRateOpt=1 (SF10+)   */
  #define LORA_ACK_WAIT_MS        1800UL
  #define LORA_TX_TIMEOUT_MS      1500UL
  #define LORA_HELLO_ACK_WAIT_MS  2000UL
#endif

/* MODEM_CFG1: BW=125 kHz, CR=4/5, explicit header */
#define LORA_REG_MODEM_CFG1       0x72

/* PA: +20 dBm via PA_BOOST  (RegPaConfig=0xFF, RegPaDac=0x87)         */
#define LORA_REG_PA_CONFIG        0xFF
#define LORA_REG_PA_DAC           0x87
#define LORA_REG_OCP              0x3B    /* OCP on, ~240 mA            */

/* Preamble length: 12 symbols (longer = more reliable sync)           */
#define LORA_PREAMBLE_MSB         0x00
#define LORA_PREAMBLE_LSB         0x0C

/* Sync word — keep at 0x12 (private), avoid 0x34 (LoRaWAN public)     */
#define LORA_SYNC_WORD            0x12

/* ── Application-layer timing ───────────────────────────────────────── */

/* TX node */
#define TX_HELLO_RETRY_INTERVAL_MS    3000UL   /* HELLO every 3 s when DC */
#define TX_HELLO_MAX_RETRIES          0U       /* 0 = retry forever       */
#define TX_DATA_RETRY_MAX             3U
#define TX_NOACK_DISCONNECT_THRESH    3U       /* N data fails → DC       */
#define TX_KEEPALIVE_INTERVAL_MS      30000UL  /* PING every 30 s if idle */
#define TX_DATA_INTERVAL_MS           10000UL  /* normal data cadence     */

/* RX node */
#define RX_PEER_TIMEOUT_MS            120000UL /* no rx → STALE           */
#define RX_SYNC_REQ_INTERVAL_MS       5000UL   /* @SY rate when STALE     */
#define RX_PAIRING_TIMEOUT_MS         30000UL

/* Inter-packet gap to avoid TX/RX switching collision — measured for
 * SX1276 at SF10 it needs ≥ 8 ms quiet between RxDone and TX-start to
 * keep the modem reliable. Use 12 ms for safety margin.               */
#define LORA_RX_TO_TX_GAP_MS          12UL
#define LORA_TX_TO_RX_GAP_MS          5UL

/* ── Helper: validate parsed packet structure ───────────────────────── */
static inline bool LoRa_DID_IsValid(uint32_t did)
{
    return (did != 0U) && (did != 0xFFFFFFFFUL);
}

#endif /* LORA_PROTOCOL_H */
