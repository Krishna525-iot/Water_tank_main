/* ====================================================================
 * device_id.h  —  Device ID generation & paired-device management
 *
 * Each STM32 node derives a 32-bit Device ID (DID) from its unique
 * 96-bit silicon UID on first boot and persists it in EEPROM.
 *
 * Transmitter : uses DeviceID_Init() + DeviceID_GetHex()
 *               stamps every outgoing packet with ,DID:XXXXXXXX
 *
 * Receiver    : uses DeviceID_Init() + PairedDev_*()
 *               stores up to MAX_PAIRED transmitter DIDs.
 *               Rejects packets whose DID is not in the list.
 *               If the list is EMPTY → open mode (accept all DID).
 *
 * EEPROM map (new, no overlap with existing layout):
 *   0x0700  —  own-DID block       ( 8 bytes)
 *   0x0720  —  paired-DID block    (22 bytes, up to 3 entries)
 * ==================================================================== */

#ifndef DEVICE_ID_H
#define DEVICE_ID_H

#include <stdint.h>
#include <stdbool.h>

#define DID_EE_ADDR   0x0700U
#define PAIR_EE_ADDR  0x0720U
#define MAX_PAIRED    3
#define DID_SIG       0xD1D0U
#define PAIR_SIG      0xDDAAU

/* ── Own Device ID (both nodes) ─────────────────────────────────── */
void     DeviceID_Init(void);          /* load or generate own DID  */
uint32_t DeviceID_GetOwn(void);
void     DeviceID_GetHex(char out[9]); /* "AABBCCDD\0"              */

/* ── Paired Device List (receiver node only) ─────────────────────── */
void     PairedDev_Init(void);
uint8_t  PairedDev_Count(void);
bool     PairedDev_IsAllowed(uint32_t did); /* true if list empty OR did found */
bool     PairedDev_Add(uint32_t did);
void     PairedDev_Remove(uint32_t did);
void     PairedDev_Clear(void);
uint32_t PairedDev_Get(uint8_t index);

#endif /* DEVICE_ID_H */
