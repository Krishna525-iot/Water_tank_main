/* ====================================================================
 * rf.c  —  RECEIVER  433 MHz OOK/ASK driver
 *
 * v6.8 — Preamble-gated sync + post-CRC raw buffer + validated counting
 *
 * WHY v6.8 (diagnosis from live debug of v6.7):
 *   Live Expressions showed:
 *     • s_rxPackets = 0x1000 (4096) — a round power of two, i.e. the
 *       counter was advancing on NOISE that the v6.7 "noise-tolerant"
 *       sync hunt stitched into a false 0x2DD4 lock.
 *     • g_rfRxData.raw held a CLEAN-looking "@TL:000,WD:1,SQ:..." string
 *       with a stray 0x01 in front, while .level/.wellDry = 0 and
 *       .valid = false.
 *
 *   Two distinct defects produced that picture:
 *
 *   DEFECT A — raw buffer written BEFORE CRC check (v6.6/6.7 Phase 4):
 *     s_rfLastRawPacket was strncpy'd from the decoded payload and THEN
 *     the CRC was compared. So the "raw" anyone read back was the last
 *     DECODE ATTEMPT, CRC-failed garbage included — never a guarantee of
 *     a real packet. That is why raw looked plausible while every
 *     validated field stayed zero/false.  FIX: write s_rfLastRawPacket
 *     ONLY after CRC passes.  A separate s_rfLastRejectRaw captures the
 *     failed bytes for debug without polluting the "last good" buffer.
 *
 *   DEFECT B — sync could lock on noise (v6.7 hunt accepted any 16 good
 *     bits whose shift register happened to equal 0x2DD4).  Superregen
 *     idle noise, once past the glitch filter, can form 0x2DD4 by chance.
 *     FIX: PREAMBLE GATE.  Before the sync word is accepted, the decoder
 *     must first observe a run of >= RF_MIN_VALID_PREAMBLE_BITS genuine
 *     alternating preamble bits (the 0xAA …1010… pattern).  Random noise
 *     does not hold a clean alternating run, so it can no longer reach
 *     the sync compare.  Only after the preamble gate is satisfied does
 *     the decoder look for 0x2D 0xD4.
 *
 *   DEFECT C (consequence) — s_rxPackets counted decode attempts that
 *     reached parse, not validated unique packets.  FIX: s_rxPackets is
 *     incremented ONLY after CRC pass AND successful parse.  A separate
 *     s_rxFrames counts raw frame attempts for diagnostics.
 *
 * RESULT: a frame is only counted, only updates raw, and only updates
 *   level/WD/valid when it (1) followed a real preamble, (2) carried a
 *   correct sync word, (3) passed CRC-8, and (4) parsed as a known type.
 *
 * ── Bit timing (unchanged, from rf.h) ───────────────────────────────
 *   bit '1': HIGH 300µs + LOW 900µs   (total 1200µs)
 *   bit '0': HIGH 900µs + LOW 300µs   (total 1200µs)
 *
 * Pin: PD1 (RF_connector_Pin = GPIO_PIN_1, RF_connector_GPIO_Port = GPIOD)
 *   PD01 AFIO remap MANDATORY in MX_GPIO_Init() before HAL_GPIO_Init().
 *   No pull resistor — XY-MK-5V drives DATA actively (a pull would
 *   distort the module's AGC envelope).
 *
 * WIRE FORMAT and TX firmware are UNCHANGED.  All edits are RX-only.
 * ==================================================================== */

#include "rf.h"
#include "lora_parser.h"
#include "device_id.h"
#include "stm32f1xx_hal.h"
#include "main.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/* ── TIM3: 64 MHz / (63+1) = 1 MHz -> 1 µs/count ──────────────────── */
#define RF_TIM3_PRESCALER    63u

/* ── Activity gate: how long RF_Task() waits for a preamble edge ────── */
#define RF_IDLE_GATE_US      5000u

/* ── read_bit() inter-bit timeout ───────────────────────────────────── */
#define RF_BIT_EDGE_TIMEOUT_US   2400u

/* ── Sync word ──────────────────────────────────────────────────────── */
#define RF_SYNC16  ((uint16_t)(((uint16_t)RF_SYNC_BYTE1 << 8u) | RF_SYNC_BYTE2))

/* ── Superregen (XY-MK-5V) hardening — RX decode tuning ─────────────── *
 *   RF_GLITCH_CONFIRM_US : HIGH must persist this long to be a real edge *
 *   RF_BIT_SPLIT_US      : midpoint between a 300µs '1' and 900µs '0'    *
 *   RF_BIT_WIDTH_MIN/MAX : hard sanity bounds on the HIGH pulse          */
#define RF_GLITCH_CONFIRM_US        60u
#define RF_BIT_WIDTH_MIN_US        150u
#define RF_BIT_WIDTH_MAX_US       1300u
#define RF_BIT_SPLIT_US            600u

/* ── Sync-hunt bounds ───────────────────────────────────────────────── */
#define RF_SYNC_BAD_RUN_MAX         10u   /* consecutive bad bits -> bail   */
#define RF_SYNC_HUNT_MAX_ATTEMPTS  400u   /* total read_bit() calls / hunt  */

/* ── Preamble gate (DEFECT B fix) ───────────────────────────────────── *
 * Number of consecutive ALTERNATING bits (…1010… i.e. the 0xAA pattern) *
 * that must be seen before the decoder is allowed to look for the sync  *
 * word.  rf.h provides RF_MIN_VALID_PREAMBLE_BITS (= 16).  Random noise  *
 * cannot sustain a clean alternating run of this length, so it can no    *
 * longer slip through to a false sync lock.                              */
#ifndef RF_MIN_VALID_PREAMBLE_BITS
#define RF_MIN_VALID_PREAMBLE_BITS  16u
#endif

extern TIM_HandleTypeDef  htim3;
extern UART_HandleTypeDef huart1;

static void uart_ln(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s,      (uint16_t)strlen(s), 500u);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2u,                  500u);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  MODULE STATE
 * ═══════════════════════════════════════════════════════════════════════ */
static uint8_t  s_rfLevel      = 0u;
static uint8_t  s_rfWellDry    = 0u;
static uint32_t s_rfLastRxTick = 0u;
static bool     s_rfDataValid  = false;

/* Last GOOD packet (written ONLY after CRC pass + parse) */
static char     s_rfLastRawPacket[RF_MAX_PAYLOAD + 1u] = {0};
static uint32_t s_rfLastDid      = 0u;
static uint32_t s_rfLastSeq      = 0u;
static uint8_t  s_rfLastType     = 0u;
static uint8_t  s_rfLastCrcRx    = 0u;
static uint8_t  s_rfLastCrcCalc  = 0u;
static uint32_t s_rfLastPacketMs = 0u;
static bool     s_rfPacketSeen   = false;

/* Last REJECTED raw bytes — debug only, never trusted as a real packet */
static char     s_rfLastRejectRaw[RF_MAX_PAYLOAD + 1u] = {0};

/* Dedup */
static uint32_t s_rfDupLastSeq  = 0u;
static bool     s_rfDupSeqInit  = false;

/* Counters: s_rxPackets = VALIDATED packets only; s_rxFrames = attempts */
static uint32_t s_rxPackets = 0u;
static uint32_t s_rxFrames  = 0u;
static uint32_t s_rxErrors  = 0u;
static bool     s_hwOk      = false;

/* ═══════════════════════════════════════════════════════════════════════
 *  CRC-8  poly 0x07, init 0x00
 * ═══════════════════════════════════════════════════════════════════════ */
uint8_t RF_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00u;
    while (len--)
    {
        crc ^= *data++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80u) ? ((crc << 1u) ^ 0x07u) : (crc << 1u);
    }
    return crc;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  PIN READER  (PD1; PD01 remap must be active or always returns 0)
 * ═══════════════════════════════════════════════════════════════════════ */
static inline uint8_t rf_pin(void)
{
    return (HAL_GPIO_ReadPin(RF_connector_GPIO_Port, RF_connector_Pin)
            == GPIO_PIN_SET) ? 1u : 0u;
}

/* ── TIM3 counter helpers ───────────────────────────────────────────── */
static inline void     tim_start(void) { __HAL_TIM_SET_COUNTER(&htim3, 0u); }
static inline uint32_t tim_now(void)   { return __HAL_TIM_GET_COUNTER(&htim3); }

/* ═══════════════════════════════════════════════════════════════════════
 *  READ ONE RF BIT  — glitch-confirmed edge + single-threshold classify
 *
 *  Returns: 1, 0, or -1 (timeout / out-of-range / noise).
 *  Unchanged from v6.7 — the bit-level decode itself was correct; the
 *  defects were above it (sync gate, raw buffer, counting).
 * ═══════════════════════════════════════════════════════════════════════ */
static int8_t read_bit(void)
{
    uint32_t t_rise = 0u, t_fall = 0u, w = 0u, tc = 0u;
    bool     glitch = false;

    /* Step 1: if HIGH on entry, wait out the current pulse first. */
    if (rf_pin() == 1u)
    {
        tim_start();
        while (rf_pin() == 1u)
            if (tim_now() > RF_BIT_WIDTH_MAX_US) return -1;
    }

    /* Step 2: hunt for a CONFIRMED rising edge. */
    tim_start();
    for (;;)
    {
        if (tim_now() > RF_BIT_EDGE_TIMEOUT_US) return -1;   /* no edge */
        if (rf_pin() == 0u) continue;

        t_rise = tim_now();
        tc     = t_rise;
        glitch = false;
        while ((tim_now() - tc) < RF_GLITCH_CONFIRM_US)
            if (rf_pin() == 0u) { glitch = true; break; }
        if (!glitch) break;                 /* confirmed real rising edge */
    }

    /* Step 3: measure HIGH width to the falling edge. */
    while (rf_pin() == 1u)
        if ((tim_now() - t_rise) > RF_BIT_WIDTH_MAX_US) return -1;
    t_fall = tim_now();

    /* Step 4: single-threshold classification with sanity bounds. */
    w = t_fall - t_rise;
    if (w < RF_BIT_WIDTH_MIN_US) return -1;
    return (w < RF_BIT_SPLIT_US) ? (int8_t)1 : (int8_t)0;
}

/* ── read_byte: MSB first, 8 bits ───────────────────────────────────── */
static bool read_byte(uint8_t *out)
{
    uint8_t b = 0u;
    for (int8_t i = 7; i >= 0; i--)
    {
        int8_t bit = read_bit();
        if (bit < 0) return false;
        b |= (uint8_t)((uint8_t)bit << (uint8_t)i);
    }
    *out = b;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  PREAMBLE-GATED SYNC HUNT  (DEFECT B fix)
 *
 *  Two stages:
 *    Stage 1 — PREAMBLE GATE: count consecutive ALTERNATING bits. The
 *      0xAA preamble is …1 0 1 0 1 0… so each accepted bit must differ
 *      from the previous one. A bit equal to the previous resets the run
 *      (noise / a real sync edge). The gate is satisfied once the run
 *      reaches RF_MIN_VALID_PREAMBLE_BITS. Random superregen noise cannot
 *      hold a clean alternating run that long, so it never reaches sync.
 *
 *    Stage 2 — SYNC MATCH: only after the gate, shift bits into a 16-bit
 *      register and accept the frame when it equals 0x2DD4. Because the
 *      preamble ends …1010 and sync starts 0x2D = 0010_1101, the first
 *      sync bit (0) breaks the alternation — which is exactly the signal
 *      that the preamble has ended and the sync word has begun.
 *
 *  Returns true and leaves the bit-stream positioned right after the
 *  sync word (next read_byte() reads the length byte).
 * ═══════════════════════════════════════════════════════════════════════ */
static bool hunt_preamble_then_sync(void)
{
    uint32_t attempts   = 0u;
    uint32_t bad_run    = 0u;

    /* ── Stage 1: preamble gate ─────────────────────────────────────── */
    uint32_t alt_run    = 0u;
    int8_t   prev_bit   = -2;          /* -2 = none yet */

    while (attempts < RF_SYNC_HUNT_MAX_ATTEMPTS)
    {
        attempts++;
        int8_t b = read_bit();

        if (b < 0)
        {
            alt_run  = 0u;
            prev_bit = -2;
            if (++bad_run >= RF_SYNC_BAD_RUN_MAX) return false;
            continue;
        }
        bad_run = 0u;

        if (prev_bit == -2)
        {
            alt_run  = 1u;             /* first valid bit */
        }
        else if (b != prev_bit)
        {
            alt_run++;                 /* alternation continues */
        }
        else
        {
            alt_run  = 1u;             /* alternation broken — restart run */
        }
        prev_bit = b;

        if (alt_run >= RF_MIN_VALID_PREAMBLE_BITS)
            break;                     /* genuine preamble confirmed */
    }

    if (alt_run < RF_MIN_VALID_PREAMBLE_BITS)
        return false;                  /* never saw a real preamble */

    /* ── Stage 2: sync match ────────────────────────────────────────── *
     * We are mid-preamble (alternating). Continue reading bits into a    *
     * shift register; the sync word breaks alternation and lands as      *
     * 0x2DD4. Keep going until sync is found or we run out of attempts.  */
    uint16_t sreg = 0u;

    /* Seed sreg with a fresh window. We do NOT carry preamble bits in,   *
     * because the preamble is all-alternating and would never equal      *
     * 0x2DD4; the sync bytes themselves provide the full 16-bit match.   */
    while (attempts < RF_SYNC_HUNT_MAX_ATTEMPTS)
    {
        attempts++;
        int8_t b = read_bit();

        if (b < 0)
        {
            if (++bad_run >= RF_SYNC_BAD_RUN_MAX) return false;
            sreg = 0u;                 /* don't stitch noise into sync */
            continue;
        }
        bad_run = 0u;

        sreg = (uint16_t)((sreg << 1u) | (uint8_t)b);

        if (sreg == RF_SYNC16)
            return true;               /* sync locked, real preamble seen */
    }

    return false;                      /* attempt budget exhausted */
}

/* ═══════════════════════════════════════════════════════════════════════
 *  TRY RECEIVE ONE COMPLETE RF FRAME
 *
 *  Frame: [5×0xAA preamble][0x2D][0xD4][len][payload][CRC8]
 *
 *  Order of operations is now strictly:
 *    1. preamble gate + sync   (hunt_preamble_then_sync)
 *    2. length                 (bounds-checked)
 *    3. payload                (len bytes)
 *    4. CRC                    (compared BEFORE touching the good buffer)
 *    5. parse                  (LoRa_ParsePacket)
 *  Only after 4 AND 5 succeed do we write s_rfLastRawPacket, bump
 *  s_rxPackets, and update level/WD/valid.  Failures touch ONLY
 *  s_rfLastRejectRaw and s_rxErrors.
 * ═══════════════════════════════════════════════════════════════════════ */
static bool try_receive_frame(void)
{
    s_rxFrames++;                      /* a decode attempt is starting */

    /* ── Phase 1: preamble-gated sync ──────────────────────────────── */
    if (!hunt_preamble_then_sync())
        return false;                  /* no real preamble+sync — silent */

    /* ── Phase 2: length byte ──────────────────────────────────────── */
    uint8_t len = 0u;
    if (!read_byte(&len))
        return false;

    if (len == 0u || len > RF_MAX_PAYLOAD)
    {
        s_rxErrors++;
        uart_ln("[RF RX] ERR: bad length");
        return false;
    }

    /* ── Phase 3: payload (into a LOCAL buffer, not the good buffer) ── */
    char payload[RF_MAX_PAYLOAD + 1u];
    for (uint8_t i = 0u; i < len; i++)
    {
        if (!read_byte((uint8_t *)&payload[i]))
            return false;
    }
    payload[len] = '\0';

    /* ── Phase 4: CRC ──────────────────────────────────────────────── */
    uint8_t rx_crc = 0u;
    if (!read_byte(&rx_crc))
        return false;

    uint8_t calc_crc = RF_CRC8((const uint8_t *)payload, len);

    if (rx_crc != calc_crc)
    {
        /* CRC failed — record to the REJECT buffer only, never the good one */
        s_rxErrors++;
        strncpy(s_rfLastRejectRaw, payload, RF_MAX_PAYLOAD);
        s_rfLastRejectRaw[RF_MAX_PAYLOAD] = '\0';

        char dbg[160];
        snprintf(dbg, sizeof(dbg),
                 "[RF RX] CRC FAIL raw=\"%s\" rx=0x%02X calc=0x%02X",
                 s_rfLastRejectRaw, (unsigned)rx_crc, (unsigned)calc_crc);
        uart_ln(dbg);
        return false;
    }

    /* ── Phase 5: parse ────────────────────────────────────────────── */
    ParsedPacket_t p;
    if (!LoRa_ParsePacket(payload, &p))
    {
        s_rxErrors++;
        strncpy(s_rfLastRejectRaw, payload, RF_MAX_PAYLOAD);
        s_rfLastRejectRaw[RF_MAX_PAYLOAD] = '\0';

        char dbg[160];
        snprintf(dbg, sizeof(dbg), "[RF RX] PARSE FAIL raw=\"%s\"",
                 s_rfLastRejectRaw);
        uart_ln(dbg);
        return false;
    }

    /* ════════════════════════════════════════════════════════════════
     *  VALIDATED PACKET — only here do we touch the "last good" state.
     * ════════════════════════════════════════════════════════════════ */
    s_rxPackets++;
    s_rfLastType     = (uint8_t)p.type;
    s_rfLastDid      = p.did;
    s_rfLastSeq      = p.seq;
    s_rfLastCrcRx    = rx_crc;
    s_rfLastCrcCalc  = calc_crc;
    s_rfLastPacketMs = HAL_GetTick();
    s_rfPacketSeen   = true;

    strncpy(s_rfLastRawPacket, payload, RF_MAX_PAYLOAD);
    s_rfLastRawPacket[RF_MAX_PAYLOAD] = '\0';

    {
        char dbg[200];
        snprintf(dbg, sizeof(dbg),
                 "[RF RX] OK #%lu type=%d DID=%08lX SEQ=%08lX \"%s\"",
                 (unsigned long)s_rxPackets, (int)p.type,
                 (unsigned long)p.did, (unsigned long)p.seq,
                 s_rfLastRawPacket);
        uart_ln(dbg);
    }

    /* ── TANKLEVEL ─────────────────────────────────────────────────── */
    if (p.type == PKT_TYPE_TANKLEVEL)
    {
        if (s_rfDupSeqInit && p.seq == s_rfDupLastSeq)
        {
            uart_ln("[RF RX] dup seq — discarded");
            return false;
        }
        s_rfDupLastSeq = p.seq;
        s_rfDupSeqInit = true;

        s_rfLevel      = p.level;
        s_rfWellDry    = p.well_dry;
        s_rfLastRxTick = HAL_GetTick();
        s_rfDataValid  = true;

        char dbg[200];
        snprintf(dbg, sizeof(dbg),
                 "[RF RX] ACCEPTED Level=%u%% WD=%u Seq=%08lX DID=%08lX",
                 (unsigned)s_rfLevel, (unsigned)s_rfWellDry,
                 (unsigned long)s_rfLastSeq, (unsigned long)s_rfLastDid);
        uart_ln(dbg);
        return true;
    }

    /* ── HELLO ─────────────────────────────────────────────────────── */
    if (p.type == PKT_TYPE_HELLO)
    {
        char dbg[160];
        snprintf(dbg, sizeof(dbg),
                 "[RF RX] HELLO DID=%08lX raw=\"%s\"",
                 (unsigned long)s_rfLastDid, s_rfLastRawPacket);
        uart_ln(dbg);
        s_rfLastRxTick = HAL_GetTick();
        return true;
    }

    /* Other known types (ACK/REJ/PI/PO/SY/BY) parse fine but carry no
     * tank data; they count as valid packets and refresh the link tick. */
    s_rfLastRxTick = HAL_GetTick();
    uart_ln("[RF RX] non-data packet accepted");
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  PUBLIC ACCESSORS
 * ═══════════════════════════════════════════════════════════════════════ */
bool RF_IsWirelessDataValid(void)
{
    if (!s_rfDataValid) return false;
    if ((HAL_GetTick() - s_rfLastRxTick) > RF_WIRELESS_TIMEOUT_MS)
    {
        s_rfDataValid = false;
        uart_ln("[RF RX] timeout — local ADC fallback");
        return false;
    }
    return true;
}

uint8_t       RF_GetWirelessTankLevel (void) { return s_rfLevel;        }
uint8_t       RF_GetWirelessWellDry   (void) { return s_rfWellDry;      }
bool          RF_HasReceivedPacket    (void) { return s_rfPacketSeen;   }
const char   *RF_GetLastRawPacket     (void) { return s_rfLastRawPacket;}
uint32_t      RF_GetLastPacketDID     (void) { return s_rfLastDid;      }
uint32_t      RF_GetLastPacketSeq     (void) { return s_rfLastSeq;      }
uint8_t       RF_GetLastPacketType    (void) { return s_rfLastType;     }
uint8_t       RF_GetLastPacketCrcRx   (void) { return s_rfLastCrcRx;    }
uint8_t       RF_GetLastPacketCrcCalc (void) { return s_rfLastCrcCalc;  }
uint32_t      RF_GetRxPacketCount     (void) { return s_rxPackets;      }
uint32_t      RF_GetRxErrorCount      (void) { return s_rxErrors;       }

/* New: frame-attempt counter (diagnostics) and reject-raw accessor */
uint32_t      RF_GetRxFrameCount      (void) { return s_rxFrames;       }
const char   *RF_GetLastRejectRaw     (void) { return s_rfLastRejectRaw;}

uint32_t RF_GetLastPacketAgeMs(void)
{
    if (!s_rfPacketSeen) return 0xFFFFFFFFUL;
    return HAL_GetTick() - s_rfLastPacketMs;
}

void RF_ClearWirelessData(void)
{
    s_rfLevel            = 0u;
    s_rfWellDry          = 0u;
    s_rfDataValid        = false;
    s_rfLastRxTick       = 0u;
    s_rfDupSeqInit       = false;
    s_rfDupLastSeq       = 0u;
    s_rfLastRawPacket[0] = '\0';
    s_rfLastRejectRaw[0] = '\0';
    s_rfLastDid          = 0u;
    s_rfLastSeq          = 0u;
    s_rfLastType         = 0u;
    s_rfLastCrcRx        = 0u;
    s_rfLastCrcCalc      = 0u;
    s_rfLastPacketMs     = 0u;
    s_rfPacketSeen       = false;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  RF_Init
 * ═══════════════════════════════════════════════════════════════════════ */
void RF_Init(void)
{
    uart_ln("[RF RX] Init v6.8 (preamble-gated, post-CRC raw)...");
    uart_ln("[RF RX] Pin     : PD1 (RF_connector_Pin = GPIO_PIN_1, GPIOD)");
    uart_ln("[RF RX] Remap   : __HAL_AFIO_REMAP_PD01_ENABLE() in MX_GPIO_Init");
    uart_ln("[RF RX] No pull : INPUT no pull (XY-MK-5V drives actively)");

    HAL_TIM_Base_Stop(&htim3);
    __HAL_TIM_SET_PRESCALER(&htim3, RF_TIM3_PRESCALER);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 0xFFFFu);
    htim3.Instance->EGR |= TIM_EGR_UG;   /* force prescaler load */
    HAL_TIM_Base_Start(&htim3);

    RF_ClearWirelessData();
    s_rxPackets = 0u;
    s_rxFrames  = 0u;
    s_rxErrors  = 0u;
    s_hwOk      = true;

    uart_ln("[RF RX] TIM3    : 1 MHz (prescaler=63)");
    uart_ln("[RF RX] Gate    : 5000 us idle gate, glitch-confirmed");
    uart_ln("[RF RX] Sync    : preamble-gated (>=16 alternating bits) + 0x2DD4");
    uart_ln("[RF RX] Raw     : written ONLY after CRC pass + parse");
    uart_ln("[RF RX] Count   : s_rxPackets = validated packets only");
    uart_ln("[RF RX] READY   : main loop must have NO HAL_Delay in RF433 mode");
}

/* ═══════════════════════════════════════════════════════════════════════
 *  RF_Task  — non-blocking idle gate, then blocking frame decode
 * ═══════════════════════════════════════════════════════════════════════ */
void RF_Task(void)
{
    if (!s_hwOk) return;

    /* Data expiry */
    if (s_rfDataValid &&
        (HAL_GetTick() - s_rfLastRxTick) > RF_WIRELESS_TIMEOUT_MS)
    {
        s_rfDataValid = false;
        uart_ln("[RF RX] data expired");
    }

    /* Activity gate — wait up to RF_IDLE_GATE_US for a CONFIRMED HIGH. */
    tim_start();
    for (;;)
    {
        if (tim_now() > RF_IDLE_GATE_US) return;     /* idle, no activity */
        if (rf_pin() == 0u) continue;

        uint32_t tc = tim_now();
        bool glitch = false;
        while ((tim_now() - tc) < RF_GLITCH_CONFIRM_US)
            if (rf_pin() == 0u) { glitch = true; break; }
        if (!glitch) break;                          /* real carrier */
    }

    /* Confirmed carrier — attempt one full frame decode. */
    (void)try_receive_frame();
}
