/* ====================================================================
 * device_id.c  —  Device ID + Paired-Device implementation
 *
 * TRANSMITTER node (LORA_TRANSMITTER_NODE defined):
 *   • No EEPROM hardware on TX end.
 *   • DeviceID_Init() derives DID directly from the STM32 96-bit
 *     silicon UID (permanent, readable every boot — no storage needed).
 *   • All PairedDev_*() functions are stubs that do nothing.
 *
 * RECEIVER node (LORA_RECEIVER_NODE defined):
 *   • Has EEPROM (eeprom_i2c.h available).
 *   • DeviceID_Init() loads own DID from EEPROM (or generates +
 *     persists on first boot).
 *   • PairedDev_*() manage up to MAX_PAIRED TX entries in EEPROM.
 * ==================================================================== */

#include "device_id.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>

/* EEPROM header only needed on receiver */
#ifdef LORA_RECEIVER_NODE
#include "eeprom_i2c.h"
#endif

/* STM32F1 96-bit unique device ID register */
#define STM32_UID_BASE   0x1FFFF7E8UL

/* ── CRC-16/Modbus (shared) ─────────────────────────────────────── */
static uint16_t did_crc16(const uint8_t *d, uint16_t len)
{
    uint16_t c = 0xFFFF;
    while (len--)
    {
        c ^= *d++;
        for (int i = 0; i < 8; i++)
            c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
    }
    return c;
}

/* ── Silicon UID → 32-bit DID (available on both nodes) ─────────── */
static uint32_t uid_generate(void)
{
    volatile uint32_t *uid = (volatile uint32_t *)STM32_UID_BASE;
    uint32_t v = uid[0] ^ uid[1] ^ uid[2];
    if (v == 0 || v == 0xFFFFFFFFUL)
        v = 0xA1B2C3D4UL;   /* safe fallback */
    return v;
}

/* ════════════════════════════════════════════════════════════════════
 *  OWN DEVICE ID
 * ════════════════════════════════════════════════════════════════════ */
static uint32_t s_ownDID = 0;

#ifdef LORA_TRANSMITTER_NODE
/* ── TX: no EEPROM — derive from silicon UID every boot ─────────── */
void DeviceID_Init(void)
{
    s_ownDID = uid_generate();
}

#else /* LORA_RECEIVER_NODE */
/* ── RX: persist DID in EEPROM ───────────────────────────────────── */
typedef struct __attribute__((packed))
{
    uint16_t sig;   /* DID_SIG = 0xD1D0 */
    uint32_t did;
    uint16_t crc;
} DIDBlock;         /* 8 bytes */

void DeviceID_Init(void)
{
    DIDBlock b;
    EEPROM_ReadBuffer(DID_EE_ADDR, (uint8_t *)&b, sizeof(b));
    uint16_t crc = did_crc16((uint8_t *)&b, sizeof(b) - 2);

    if (b.sig == DID_SIG && crc == b.crc &&
        b.did != 0 && b.did != 0xFFFFFFFFUL)
    {
        s_ownDID = b.did;
        return;
    }

    /* First boot or corrupt — generate and persist */
    s_ownDID = uid_generate();
    memset(&b, 0, sizeof(b));
    b.sig = DID_SIG;
    b.did = s_ownDID;
    b.crc = did_crc16((uint8_t *)&b, sizeof(b) - 2);
    EEPROM_WriteBlockSafe(DID_EE_ADDR, (uint8_t *)&b, sizeof(b));
}
#endif /* LORA_RECEIVER_NODE */

uint32_t DeviceID_GetOwn(void) { return s_ownDID; }

void DeviceID_GetHex(char out[9])
{
    snprintf(out, 9, "%08lX", (unsigned long)s_ownDID);
}

/* ════════════════════════════════════════════════════════════════════
 *  PAIRED DEVICE LIST
 * ════════════════════════════════════════════════════════════════════ */

#ifdef LORA_TRANSMITTER_NODE
/* ── TX: stubs — pairing only lives on the receiver ─────────────── */
void     PairedDev_Init(void)              { }
uint8_t  PairedDev_Count(void)             { return 0; }
bool     PairedDev_IsAllowed(uint32_t did) { (void)did; return true; }
bool     PairedDev_Add(uint32_t did)       { (void)did; return false; }
void     PairedDev_Remove(uint32_t did)    { (void)did; }
void     PairedDev_Clear(void)             { }
uint32_t PairedDev_Get(uint8_t index)      { (void)index; return 0; }

#else /* LORA_RECEIVER_NODE */
/* ── RX: full EEPROM-backed implementation ───────────────────────── */

/*  EEPROM block layout (18 bytes total):
 *   uint16_t sig       PAIR_SIG = 0xDDAA
 *   uint8_t  count     number of paired DIDs (0..MAX_PAIRED)
 *   uint8_t  pad
 *   uint32_t dids[3]   the paired DIDs
 *   uint16_t crc
 */
typedef struct __attribute__((packed))
{
    uint16_t sig;
    uint8_t  count;
    uint8_t  pad;
    uint32_t dids[MAX_PAIRED];
    uint16_t crc;
} PairedBlock;

static uint32_t s_paired[MAX_PAIRED] = {0};
static uint8_t  s_pairedCount        = 0;

static void paired_save(void)
{
    PairedBlock b;
    memset(&b, 0, sizeof(b));
    b.sig   = PAIR_SIG;
    b.count = s_pairedCount;
    for (int i = 0; i < MAX_PAIRED; i++)
        b.dids[i] = s_paired[i];
    b.crc = did_crc16((uint8_t *)&b, sizeof(b) - 2);
    EEPROM_WriteBlockSafe(PAIR_EE_ADDR, (uint8_t *)&b, sizeof(b));
}

void PairedDev_Init(void)
{
    PairedBlock b;
    EEPROM_ReadBuffer(PAIR_EE_ADDR, (uint8_t *)&b, sizeof(b));
    uint16_t crc = did_crc16((uint8_t *)&b, sizeof(b) - 2);
    if (b.sig != PAIR_SIG || crc != b.crc)
    {
        s_pairedCount = 0;
        memset(s_paired, 0, sizeof(s_paired));
        return;
    }
    s_pairedCount = (b.count <= MAX_PAIRED) ? b.count : MAX_PAIRED;
    for (int i = 0; i < MAX_PAIRED; i++)
        s_paired[i] = b.dids[i];
}

uint8_t  PairedDev_Count(void)           { return s_pairedCount; }
uint32_t PairedDev_Get(uint8_t index)    { return (index < MAX_PAIRED) ? s_paired[index] : 0; }

bool PairedDev_IsAllowed(uint32_t did)
{
    if (s_pairedCount == 0) return true;   /* open mode */
    for (int i = 0; i < s_pairedCount; i++)
        if (s_paired[i] == did) return true;
    return false;
}

bool PairedDev_Add(uint32_t did)
{
    if (did == 0 || did == 0xFFFFFFFFUL) return false;
    for (int i = 0; i < s_pairedCount; i++)
        if (s_paired[i] == did) return true;   /* already paired */
    if (s_pairedCount >= MAX_PAIRED) return false;
    s_paired[s_pairedCount++] = did;
    paired_save();
    return true;
}

void PairedDev_Remove(uint32_t did)
{
    for (int i = 0; i < s_pairedCount; i++)
    {
        if (s_paired[i] == did)
        {
            for (int j = i; j < s_pairedCount - 1; j++)
                s_paired[j] = s_paired[j + 1];
            s_paired[--s_pairedCount] = 0;
            paired_save();
            return;
        }
    }
}

void PairedDev_Clear(void)
{
    memset(s_paired, 0, sizeof(s_paired));
    s_pairedCount = 0;
    paired_save();
}
#endif /* LORA_RECEIVER_NODE */
