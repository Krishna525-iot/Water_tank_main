/* ====================================================================
 * lora_parser.h / .c  —  PACKET PARSER
 *
 * Centralised parsing logic so TX and RX use IDENTICAL packet validation.
 * No malloc, no statics — pure functional parsing into a typed struct.
 * ==================================================================== */

#ifndef LORA_PARSER_H
#define LORA_PARSER_H

#include "lora_protocol.h"

/* Parse any incoming packet.  Returns true if recognised + valid.
 * `out` is always written (zeroed on failure).                        */
bool LoRa_ParsePacket(const char *pkt, ParsedPacket_t *out);

/* Builders — write into caller-supplied buffer, return bytes written. */
int LoRa_BuildHello   (char *out, int sz, uint32_t did);
int LoRa_BuildAck     (char *out, int sz, uint32_t did);
int LoRa_BuildReject  (char *out, int sz, uint32_t did);
int LoRa_BuildData    (char *out, int sz, uint32_t did, uint32_t seq,
                       uint8_t level, uint8_t well_dry);
int LoRa_BuildPing    (char *out, int sz, uint32_t did, uint32_t seq);
int LoRa_BuildPong    (char *out, int sz, uint32_t did, uint32_t seq);
int LoRa_BuildSyncReq (char *out, int sz, uint32_t did);
int LoRa_BuildBye     (char *out, int sz, uint32_t did);

#endif /* LORA_PARSER_H */
