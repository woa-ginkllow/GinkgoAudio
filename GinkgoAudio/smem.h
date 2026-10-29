/*
 * smem.h - acceso minimo a SMEM (memoria compartida de Qualcomm) desde Windows.
 *
 * Basado en drivers/soc/qcom/smem.c del kernel sm6125. Solo implementa las
 * particiones privadas y la global (version 12 del SBL); el heap global
 * antiguo (version 11) no esta soportado.
 *
 * Recursos para el _CRS (ginkgo.dts):
 *   SMEM        0x46000000, 0x200000   (smem@46000000, no-map)
 *   TCSR mutex  0x00340000 + 3 * 0x1000 = 0x343000 (hwlock numero 3)
 */
#pragma once

#include "scm.h"

#define SMEM_HOST_APPS    0
#define SMEM_HOST_COUNT   13

struct _SMEM_PTABLE_ENTRY;

typedef struct _SMEM_CTX {
    volatile UINT8  *Base;        /* SMEM mapeada (write-combined)          */
    UINT32           Size;
    PVOID            LockMap;     /* mapeo de la pagina del mutex           */
    volatile ULONG  *Lock;        /* registro del lock numero 3             */
    UINT32           LocalHost;
    UINT32           ItemCount;
    const struct _SMEM_PTABLE_ENTRY *Part[SMEM_HOST_COUNT];
    const struct _SMEM_PTABLE_ENTRY *GlobalPart;
} SMEM_CTX;

/* LockPa = direccion fisica del registro del lock (0x343000), alineada a 4 */
NTSTATUS SmemInit(SMEM_CTX *S, UINT64 SmemPa, UINT32 SmemSize, UINT64 LockPa);
VOID     SmemClose(SMEM_CTX *S);

/*
 * Reserva un item. Devuelve STATUS_OBJECT_NAME_COLLISION si ya existia (como
 * -EEXIST en Linux; el llamador suele tratarlo como exito).
 */
NTSTATUS SmemAlloc(SMEM_CTX *S, UINT32 Host, UINT32 Item, SIZE_T Size);

/* Busca un item. STATUS_NOT_FOUND si todavia no existe. */
NTSTATUS SmemGet(SMEM_CTX *S, UINT32 Host, UINT32 Item,
                 PVOID *Ptr, SIZE_T *Size);

/* Diagnostico: vuelca la lista de items de la particion con el host. */
VOID     SmemDumpPartition(SMEM_CTX *S, UINT32 Host);
