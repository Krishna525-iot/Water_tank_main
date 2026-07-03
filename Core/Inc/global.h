#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdint.h>
#include <stdbool.h>   /* bool type */
#include "rf.h"        /* RF_MAX_PAYLOAD — keeps raw[] size in one place */

/* ── Legacy externs ──────────────────────────────────────────────────── */
extern volatile uint8_t motorStatus;
extern volatile bool    packetReady;

/* ════════════════════════════════════════════════════════════════════════
 *  RF received-data holder — SINGLE SOURCE OF TRUTH
 *
 *  This struct MUST be defined in exactly one place and shared by every
 *  translation unit that touches g_rfRxData (main.c, adc.c, screen.c, …).
 *
 *  Previously main.c defined RfRxData_t locally while other files had no
 *  declaration at all.  Any file that referenced g_rfRxData therefore
 *  either failed to compile or — if it carried its own mismatched idea of
 *  the layout — read/wrote the WRONG byte offsets.  That is what produced
 *  the impossible g_rfRxData.level = 176 (0xB0) reading: the debugger and
 *  the writing code disagreed on where `level` lived.
 *
 *  Field order/types here are IDENTICAL to the original main.c definition,
 *  so no offsets shift.  main.c now includes this header instead of
 *  re-declaring the type.
 * ════════════════════════════════════════════════════════════════════════ */
typedef struct
{
    uint8_t  level;                    /* tank level 0..100 %         */
    uint8_t  wellDry;                  /* 1 = well dry, 0 = has water */
    uint32_t did;                      /* transmitter device ID       */
    uint32_t seq;                      /* packet sequence number      */
    bool     valid;                    /* RF link currently healthy   */
    char     raw[RF_MAX_PAYLOAD + 1u]; /* last GOOD raw packet string */
} RfRxData_t;

extern RfRxData_t g_rfRxData;

/* ── Wireless mode (defined in main.c; read by adc.c) ────────────────── */
extern uint8_t g_wireless_mode;

#endif /* GLOBAL_H */
