/* ====================================================================
 * lora_parser.c  —  PACKET PARSER + BUILDERS
 *
 * One source of truth for packet wire format.  Compile this file into
 * BOTH the TX firmware and the RX firmware so they cannot drift.
 * ==================================================================== */

#include "lora_parser.h"
#include <string.h>
#include <stdio.h>

/* ── Hex helpers ────────────────────────────────────────────────────── */

static bool parse_hex_n(const char *p, int n, uint32_t *out)
{
    uint32_t v = 0;
    int i;

    if (p == NULL || out == NULL || n <= 0 || n > 8)
        return false;

    for (i = 0; i < n; i++)
    {
        char c = p[i];
        uint8_t d;

        if      (c >= '0' && c <= '9') d = (uint8_t)(c - '0');
        else if (c >= 'A' && c <= 'F') d = (uint8_t)(c - 'A' + 10);
        else if (c >= 'a' && c <= 'f') d = (uint8_t)(c - 'a' + 10);
        else                            return false;

        v = (v << 4) | d;
    }

    *out = v;
    return true;
}

static bool parse_uint_3digit(const char *p, uint16_t max, uint16_t *out)
{
    uint16_t v = 0;
    int i;

    if (p == NULL || out == NULL)
        return false;

    /* Accept 1–3 ASCII digits, but our format always emits 3.  Be strict. */
    for (i = 0; i < 3; i++)
    {
        if (p[i] < '0' || p[i] > '9')
            return false;
        v = (uint16_t)((v * 10U) + (uint16_t)(p[i] - '0'));
    }

    if (v > max)
        return false;

    *out = v;
    return true;
}

/* ── Internal parser per packet type ────────────────────────────────── */

static bool parse_did_only(const char *p, uint32_t *did_out)
{
    /* Format: <8 hex DID>#  */
    if (p == NULL || did_out == NULL)
        return false;

    if (strlen(p) < 9 || p[8] != '#')
        return false;

    if (!parse_hex_n(p, 8, did_out))
        return false;

    return LoRa_DID_IsValid(*did_out);
}

static bool parse_did_and_seq(const char *p, uint32_t *did_out, uint32_t *seq_out)
{
    /* Format: <8 hex DID>,SQ:<8 hex SEQ>#   */
    if (p == NULL || did_out == NULL || seq_out == NULL)
        return false;

    if (strlen(p) < (size_t)(8 + 4 + 8 + 1))
        return false;

    if (!parse_hex_n(p, 8, did_out))
        return false;
    p += 8;

    if (strncmp(p, ",SQ:", 4) != 0)
        return false;
    p += 4;

    if (!parse_hex_n(p, 8, seq_out))
        return false;
    p += 8;

    if (*p != '#')
        return false;

    return LoRa_DID_IsValid(*did_out);
}

static bool parse_tanklevel_payload(const char *p, ParsedPacket_t *out)
{
    /* Format AFTER "@TL:" :   nnn,WD:d,SQ:hhhhhhhh,DID:hhhhhhhh#       */
    uint16_t lvl = 0;
    uint32_t did = 0;
    uint32_t seq = 0;

    if (!parse_uint_3digit(p, 100, &lvl))
        return false;
    p += 3;

    if (strncmp(p, ",WD:", 4) != 0)
        return false;
    p += 4;

    if (*p != '0' && *p != '1')
        return false;
    out->well_dry = (uint8_t)(*p - '0');
    p++;

    if (strncmp(p, ",SQ:", 4) != 0)
        return false;
    p += 4;

    if (!parse_hex_n(p, 8, &seq))
        return false;
    p += 8;

    if (strncmp(p, ",DID:", 5) != 0)
        return false;
    p += 5;

    if (!parse_hex_n(p, 8, &did))
        return false;
    p += 8;

    if (*p != '#')
        return false;

    if (!LoRa_DID_IsValid(did))
        return false;

    out->level = (uint8_t)lvl;
    out->did   = did;
    out->seq   = seq;
    return true;
}

/* ── Public parser ──────────────────────────────────────────────────── */

bool LoRa_ParsePacket(const char *pkt, ParsedPacket_t *out)
{
    if (out == NULL)
        return false;

    /* Always zero the output so caller never reads stale data on failure */
    out->type     = PKT_TYPE_UNKNOWN;
    out->did      = 0;
    out->seq      = 0;
    out->level    = 0;
    out->well_dry = 0;

    if (pkt == NULL)
        return false;

    if (pkt[0] != PKT_OPENER)
        return false;

    /* Hello:   @HI:<DID># */
    if (strncmp(pkt, PFX_HELLO, 4) == 0)
    {
        if (!parse_did_only(pkt + 4, &out->did))
            return false;
        out->type = PKT_TYPE_HELLO;
        return true;
    }

    /* Ack:     @ACK:<DID># */
    if (strncmp(pkt, PFX_ACK, 5) == 0)
    {
        if (!parse_did_only(pkt + 5, &out->did))
            return false;
        out->type = PKT_TYPE_ACK;
        return true;
    }

    /* Reject:  @REJ:<DID># */
    if (strncmp(pkt, PFX_REJECT, 5) == 0)
    {
        if (!parse_did_only(pkt + 5, &out->did))
            return false;
        out->type = PKT_TYPE_REJECT;
        return true;
    }

    /* Tank lvl: @TL:nnn,WD:d,SQ:h8,DID:h8# */
    if (strncmp(pkt, PFX_TANKLEVEL, 4) == 0)
    {
        if (!parse_tanklevel_payload(pkt + 4, out))
            return false;
        out->type = PKT_TYPE_TANKLEVEL;
        return true;
    }

    /* Ping:    @PI:<DID>,SQ:<SEQ># */
    if (strncmp(pkt, PFX_PING, 4) == 0)
    {
        if (!parse_did_and_seq(pkt + 4, &out->did, &out->seq))
            return false;
        out->type = PKT_TYPE_PING;
        return true;
    }

    /* Pong:    @PO:<DID>,SQ:<SEQ># */
    if (strncmp(pkt, PFX_PONG, 4) == 0)
    {
        if (!parse_did_and_seq(pkt + 4, &out->did, &out->seq))
            return false;
        out->type = PKT_TYPE_PONG;
        return true;
    }

    /* Sync req: @SY:<DID># */
    if (strncmp(pkt, PFX_SYNC_REQ, 4) == 0)
    {
        if (!parse_did_only(pkt + 4, &out->did))
            return false;
        out->type = PKT_TYPE_SYNC_REQ;
        return true;
    }

    /* Bye:     @BY:<DID># */
    if (strncmp(pkt, PFX_BYE, 4) == 0)
    {
        if (!parse_did_only(pkt + 4, &out->did))
            return false;
        out->type = PKT_TYPE_BYE;
        return true;
    }

    return false;
}

/* ── Builders ───────────────────────────────────────────────────────── */

int LoRa_BuildHello(char *out, int sz, uint32_t did)
{
    return snprintf(out, (size_t)sz, "@HI:%08lX#", (unsigned long)did);
}

int LoRa_BuildAck(char *out, int sz, uint32_t did)
{
    return snprintf(out, (size_t)sz, "@ACK:%08lX#", (unsigned long)did);
}

int LoRa_BuildReject(char *out, int sz, uint32_t did)
{
    return snprintf(out, (size_t)sz, "@REJ:%08lX#", (unsigned long)did);
}

int LoRa_BuildData(char *out, int sz, uint32_t did, uint32_t seq,
                   uint8_t level, uint8_t well_dry)
{
    if (level > 100) level = 100;
    well_dry = well_dry ? 1 : 0;
    return snprintf(out, (size_t)sz,
                    "@TL:%03u,WD:%u,SQ:%08lX,DID:%08lX#",
                    (unsigned)level, (unsigned)well_dry,
                    (unsigned long)seq, (unsigned long)did);
}

int LoRa_BuildPing(char *out, int sz, uint32_t did, uint32_t seq)
{
    return snprintf(out, (size_t)sz, "@PI:%08lX,SQ:%08lX#",
                    (unsigned long)did, (unsigned long)seq);
}

int LoRa_BuildPong(char *out, int sz, uint32_t did, uint32_t seq)
{
    return snprintf(out, (size_t)sz, "@PO:%08lX,SQ:%08lX#",
                    (unsigned long)did, (unsigned long)seq);
}

int LoRa_BuildSyncReq(char *out, int sz, uint32_t did)
{
    return snprintf(out, (size_t)sz, "@SY:%08lX#", (unsigned long)did);
}

int LoRa_BuildBye(char *out, int sz, uint32_t did)
{
    return snprintf(out, (size_t)sz, "@BY:%08lX#", (unsigned long)did);
}
