/*
 * smp2p.h - SMP2P minimo con el ADSP (basado en drivers/soc/qcom/smp2p.c del
 * kernel sm6125). No usa interrupciones: la entrada se lee por polling.
 *
 * Recursos para el _CRS (ginkgo.dts, nodo qcom,smp2p-adsp):
 *   mailbox APCS  0x0F111000 (0x1000), registro de IPC en +8, bit 10
 *   items SMEM    entrante 0x1BB (443), saliente 0x1AD (429)
 *   local-pid 0 (apps), remote-pid 2 (ADSP)
 *
 * Orden de uso:
 *   SmemInit(...);  Smp2pOpen(...);       <- antes de arrancar el ADSP
 *   PilBootAdsp(...);  Smp2pWaitReady(...);
 */
#pragma once

#include "smem.h"

/* bits de la entrada entrante "slave-kernel" (orden de interrupts-extended) */
#define SMP2P_ADSP_ERR_FATAL     (1u << 0)
#define SMP2P_ADSP_ERR_READY     (1u << 1)
#define SMP2P_ADSP_PROXY_UNVOTE  (1u << 2)
#define SMP2P_ADSP_STOP_ACK      (1u << 3)

#pragma pack(push, 1)
typedef struct _SMP2P_ITEM_ENTRY {
    UINT8  name[16];
    UINT32 value;
} SMP2P_ITEM_ENTRY;

typedef struct _SMP2P_ITEM {
    UINT32 magic;
    UINT8  version;
    UINT8  features[3];       /* "unsigned features:24" del original */
    UINT16 local_pid;
    UINT16 remote_pid;
    UINT16 total_entries;
    UINT16 valid_entries;
    UINT32 flags;
    SMP2P_ITEM_ENTRY entries[16];
} SMP2P_ITEM;
#pragma pack(pop)

typedef struct _SMP2P_CONFIG {
    SMEM_CTX *Smem;
    UINT32    LocalPid;       /* 0                         */
    UINT32    RemotePid;      /* 2                         */
    UINT32    InItem;         /* 0x1BB                     */
    UINT32    OutItem;        /* 0x1AD                     */
    UINT64    MailboxPa;      /* 0x0F111000, alineada a 4K */
    UINT32    IpcOffset;      /* 8                         */
    UINT32    IpcBit;         /* 10                        */
} SMP2P_CONFIG;

typedef struct _SMP2P_CTX {
    SMP2P_CONFIG      Cfg;
    PVOID             MboxMap;
    volatile ULONG   *Ipc;
    volatile SMP2P_ITEM *In;
    volatile SMP2P_ITEM *Out;
    BOOLEAN           Open;
    BOOLEAN           SsrAckEnabled;
    BOOLEAN           SsrAck;
} SMP2P_CTX;

/* Reserva el item saliente, registra las entradas y avisa al ADSP. */
NTSTATUS Smp2pOpen(SMP2P_CTX *C, const SMP2P_CONFIG *Cfg);
VOID     Smp2pClose(SMP2P_CTX *C);

/*
 * Busca el item entrante y negocia. STATUS_SUCCESS cuando el canal esta
 * abierto; STATUS_DEVICE_NOT_READY mientras el ADSP no lo creo todavia.
 */
NTSTATUS Smp2pPoll(SMP2P_CTX *C);

/* Lee una entrada entrante por nombre ("slave-kernel", "rdbg", ...). */
NTSTATUS Smp2pReadIn(SMP2P_CTX *C, const char *Name, UINT32 *Value);

/* Cambia bits de una entrada saliente ("master-kernel", ...) y avisa. */
NTSTATUS Smp2pUpdateOut(SMP2P_CTX *C, const char *Name,
                        UINT32 Mask, UINT32 Value);

/* Espera err-ready de "slave-kernel". STATUS_UNSUCCESSFUL si hay err-fatal. */
NTSTATUS Smp2pWaitReady(SMP2P_CTX *C, ULONG TimeoutMs);
