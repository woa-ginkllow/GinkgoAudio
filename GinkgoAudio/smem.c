/*
 * smem.c - acceso minimo a SMEM (ver smem.h).
 *
 * El protocolo del TCSR mutex (escribir 1 = "apps", releer, soltar con 0) esta
 * verificado contra qcom_hwspinlock.c del kernel sm6125 (mutex-id por defecto 1;
 * el nodo qcom,tcsr-mutex de ginkgo.dts no define "mutex-id").
 */
#include "smem.h"

#define SMEM_ITEM_COUNT           512
#define SMEM_ITEM_LAST_FIXED      8
#define SMEM_SBL_VERSION_INDEX    7
#define SMEM_GLOBAL_HEAP_VERSION  11
#define SMEM_GLOBAL_PART_VERSION  12
#define SMEM_GLOBAL_HOST          0xFFFE
#define SMEM_PRIVATE_CANARY       0xA5A5
#define SMEM_HWLOCK_APPS_ID       1u
#define SMEM_PTABLE_HDR_SIZE      32
#define SMEM_LOCK_TRIES           10000
#define SMEM_PAGE                 0x1000u

#define ALIGN_UP(x, a)  (((x) + ((a) - 1)) & ~((SIZE_T)(a) - 1))

#ifndef C_ASSERT
#define C_ASSERT(e) typedef char __C_ASSERT__[(e) ? 1 : -1]
#endif

/* header de la region principal (solo hasta "reserved") */
typedef struct _SMEM_HEADER {
    UINT32 proc_comm[16];
    UINT32 version[32];
    UINT32 initialized;
    UINT32 free_offset;
    UINT32 available;
    UINT32 reserved;
} SMEM_HEADER;

typedef struct _SMEM_PTABLE_ENTRY {
    UINT32 offset;
    UINT32 size;
    UINT32 flags;
    UINT16 host0;
    UINT16 host1;
    UINT32 cacheline;
    UINT32 reserved[7];
} SMEM_PTABLE_ENTRY;

typedef struct _SMEM_PART_HDR {
    UINT8  magic[4];
    UINT16 host0;
    UINT16 host1;
    UINT32 size;
    UINT32 offset_free_uncached;
    UINT32 offset_free_cached;
    UINT32 reserved[3];
} SMEM_PART_HDR;

typedef struct _SMEM_PRIV_ENTRY {
    UINT16 canary;
    UINT16 item;
    UINT32 size;           /* incluye padding_data */
    UINT16 padding_data;
    UINT16 padding_hdr;
    UINT32 reserved;
} SMEM_PRIV_ENTRY;

C_ASSERT(sizeof(SMEM_HEADER) == 208);
C_ASSERT(sizeof(SMEM_PTABLE_ENTRY) == 48);
C_ASSERT(sizeof(SMEM_PART_HDR) == 32);
C_ASSERT(sizeof(SMEM_PRIV_ENTRY) == 16);

static const UINT8 kPtableMagic[4] = { 0x24, 0x54, 0x4F, 0x43 };   /* "$TOC" */
static const UINT8 kPartMagic[4]   = { 0x24, 0x50, 0x52, 0x54 };   /* "$PRT" */
static const UINT8 kInfoMagic[4]   = { 0x53, 0x49, 0x49, 0x49 };   /* "SIII" */

/* ---- remote spinlock (TCSR mutex) ------------------------------------- */

static NTSTATUS LockAcquire(SMEM_CTX *S)
{
    ULONG i;

    for (i = 0; i < SMEM_LOCK_TRIES; i++) {
        WRITE_REGISTER_ULONG(S->Lock, SMEM_HWLOCK_APPS_ID);
        if (READ_REGISTER_ULONG(S->Lock) == SMEM_HWLOCK_APPS_ID) {
            KeMemoryBarrier();
            return STATUS_SUCCESS;
        }
        KeStallExecutionProcessor(100);
    }
    SCM_LOG("SMEM: no se pudo tomar el hwlock\n");
    return STATUS_IO_TIMEOUT;
}

static VOID LockRelease(SMEM_CTX *S)
{
    KeMemoryBarrier();
    WRITE_REGISTER_ULONG(S->Lock, 0);
}

/* ---- tabla de particiones --------------------------------------------- */

static const UINT8 *GetPtable(SMEM_CTX *S)
{
    const UINT8 *pt;

    if (S->Size < SMEM_PAGE) {
        return NULL;
    }
    pt = (const UINT8 *)S->Base + S->Size - SMEM_PAGE;
    if (!RtlEqualMemory(pt, kPtableMagic, 4)) {
        return NULL;
    }
    if (*(const UINT32 *)(pt + 4) != 1) {
        SCM_LOG("SMEM: version de ptable no soportada: %u\n",
                (unsigned)*(const UINT32 *)(pt + 4));
        return NULL;
    }
    return pt;
}

static const SMEM_PTABLE_ENTRY *PtableEntry(const UINT8 *pt, UINT32 i)
{
    return (const SMEM_PTABLE_ENTRY *)(pt + SMEM_PTABLE_HDR_SIZE +
                                       (SIZE_T)i * sizeof(SMEM_PTABLE_ENTRY));
}

static UINT32 GetItemCount(SMEM_CTX *S)
{
    const UINT8 *pt = GetPtable(S);
    UINT32 n;
    const UINT8 *info;

    if (pt == NULL) {
        return SMEM_ITEM_COUNT;
    }
    n = *(const UINT32 *)(pt + 8);
    info = pt + SMEM_PTABLE_HDR_SIZE + (SIZE_T)n * sizeof(SMEM_PTABLE_ENTRY);
    if (info + 18 > (const UINT8 *)S->Base + S->Size ||
        !RtlEqualMemory(info, kInfoMagic, 4)) {
        return SMEM_ITEM_COUNT;
    }
    return *(const UINT16 *)(info + 16);
}

static BOOLEAN PartHeaderOk(SMEM_CTX *S, const SMEM_PTABLE_ENTRY *e,
                            UINT32 hostA, UINT32 hostB)
{
    const SMEM_PART_HDR *h;

    if (e->offset == 0 || e->size == 0 ||
        (UINT64)e->offset + e->size > S->Size) {
        return FALSE;
    }
    h = (const SMEM_PART_HDR *)((const UINT8 *)S->Base + e->offset);
    if (!RtlEqualMemory(h->magic, kPartMagic, 4)) {
        return FALSE;
    }
    if (h->size != e->size || h->offset_free_uncached > h->size) {
        return FALSE;
    }
    return (h->host0 == hostA || h->host1 == hostA) &&
           (h->host0 == hostB || h->host1 == hostB);
}

static NTSTATUS SetGlobalPartition(SMEM_CTX *S)
{
    const UINT8 *pt = GetPtable(S);
    UINT32 i, n;

    if (pt == NULL) {
        return STATUS_DATA_ERROR;
    }
    n = *(const UINT32 *)(pt + 8);
    for (i = 0; i < n; i++) {
        const SMEM_PTABLE_ENTRY *e = PtableEntry(pt, i);

        if (e->host0 == SMEM_GLOBAL_HOST && e->host1 == SMEM_GLOBAL_HOST) {
            if (!PartHeaderOk(S, e, SMEM_GLOBAL_HOST, SMEM_GLOBAL_HOST)) {
                SCM_LOG("SMEM: la particion global es invalida\n");
                return STATUS_DATA_ERROR;
            }
            S->GlobalPart = e;
            return STATUS_SUCCESS;
        }
    }
    SCM_LOG("SMEM: falta la particion global\n");
    return STATUS_DATA_ERROR;
}

static NTSTATUS EnumeratePartitions(SMEM_CTX *S)
{
    const UINT8 *pt = GetPtable(S);
    UINT32 i, n;

    if (pt == NULL) {
        return STATUS_SUCCESS;
    }
    n = *(const UINT32 *)(pt + 8);
    for (i = 0; i < n; i++) {
        const SMEM_PTABLE_ENTRY *e = PtableEntry(pt, i);
        UINT32 remote;

        if (e->host0 != S->LocalHost && e->host1 != S->LocalHost) {
            continue;
        }
        if (e->offset == 0 || e->size == 0) {
            continue;
        }
        remote = (e->host0 == S->LocalHost) ? e->host1 : e->host0;
        if (remote >= SMEM_HOST_COUNT) {
            SCM_LOG("SMEM: host remoto invalido %u\n", (unsigned)remote);
            return STATUS_DATA_ERROR;
        }
        if (S->Part[remote] != NULL) {
            SCM_LOG("SMEM: particion duplicada para el host %u\n",
                    (unsigned)remote);
            return STATUS_DATA_ERROR;
        }
        if (!PartHeaderOk(S, e, S->LocalHost, remote)) {
            SCM_LOG("SMEM: la particion %u es invalida\n", (unsigned)i);
            return STATUS_DATA_ERROR;
        }
        S->Part[remote] = e;
    }
    return STATUS_SUCCESS;
}

/* ---- init / close ------------------------------------------------------ */

NTSTATUS SmemInit(SMEM_CTX *S, UINT64 SmemPa, UINT32 SmemSize, UINT64 LockPa)
{
    PHYSICAL_ADDRESS pa;
    const SMEM_HEADER *hdr;
    UINT32 version;
    NTSTATUS st;

    RtlZeroMemory(S, sizeof(*S));
    S->Size = SmemSize;
    S->LocalHost = SMEM_HOST_APPS;

    pa.QuadPart = (LONGLONG)SmemPa;
    S->Base = (volatile UINT8 *)MmMapIoSpace(pa, SmemSize, MmWriteCombined);
    if (S->Base == NULL) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    pa.QuadPart = (LONGLONG)(LockPa & ~(UINT64)(SMEM_PAGE - 1));
    S->LockMap = MmMapIoSpace(pa, SMEM_PAGE, MmNonCached);
    if (S->LockMap == NULL) {
        SmemClose(S);
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    S->Lock = (volatile ULONG *)((UINT8 *)S->LockMap +
                                 (LockPa & (SMEM_PAGE - 1)));

    hdr = (const SMEM_HEADER *)S->Base;
    if (hdr->initialized != 1 || hdr->reserved != 0) {
        SCM_LOG("SMEM: no fue inicializada por el SBL (init=%u res=%u)\n",
                (unsigned)hdr->initialized, (unsigned)hdr->reserved);
        SmemClose(S);
        return STATUS_DEVICE_NOT_READY;
    }

    version = hdr->version[SMEM_SBL_VERSION_INDEX];
    switch (version >> 16) {
    case SMEM_GLOBAL_PART_VERSION:
        st = SetGlobalPartition(S);
        if (!NT_SUCCESS(st)) {
            SmemClose(S);
            return st;
        }
        S->ItemCount = GetItemCount(S);
        break;
    case SMEM_GLOBAL_HEAP_VERSION:
        S->ItemCount = SMEM_ITEM_COUNT;     /* heap global: sin soporte */
        break;
    default:
        SCM_LOG("SMEM: version del SBL no soportada 0x%x\n", (unsigned)version);
        SmemClose(S);
        return STATUS_NOT_SUPPORTED;
    }

    st = EnumeratePartitions(S);
    if (!NT_SUCCESS(st)) {
        SmemClose(S);
        return st;
    }

    SCM_LOG("SMEM: version SBL 0x%x, items %u, particion ADSP=%d\n",
            (unsigned)version, (unsigned)S->ItemCount, (int)(S->Part[2] != NULL));
    return STATUS_SUCCESS;
}

VOID SmemClose(SMEM_CTX *S)
{
    if (S->LockMap != NULL) {
        MmUnmapIoSpace(S->LockMap, SMEM_PAGE);
    }
    if (S->Base != NULL) {
        MmUnmapIoSpace((PVOID)S->Base, S->Size);
    }
    S->LockMap = NULL;
    S->Lock = NULL;
    S->Base = NULL;
}

/* ---- items en una particion privada ------------------------------------ */

static const SMEM_PTABLE_ENTRY *SelectPartition(SMEM_CTX *S, UINT32 Host)
{
    if (Host < SMEM_HOST_COUNT && S->Part[Host] != NULL) {
        return S->Part[Host];
    }
    return S->GlobalPart;
}

static NTSTATUS AllocPrivate(SMEM_CTX *S, const SMEM_PTABLE_ENTRY *pe,
                             UINT32 item, SIZE_T size)
{
    volatile UINT8 *base = S->Base + pe->offset;
    volatile SMEM_PART_HDR *ph = (volatile SMEM_PART_HDR *)base;
    volatile UINT8 *pEnd = base + pe->size;
    volatile UINT8 *hdr = base + sizeof(SMEM_PART_HDR);
    volatile UINT8 *end = base + ph->offset_free_uncached;
    volatile UINT8 *cached = base + ph->offset_free_cached;
    volatile SMEM_PRIV_ENTRY *h;
    SIZE_T allocSize;

    if (end < base || end > cached || cached > pEnd) {
        return STATUS_DATA_ERROR;
    }

    while (hdr + sizeof(SMEM_PRIV_ENTRY) < end) {
        volatile UINT8 *next;

        h = (volatile SMEM_PRIV_ENTRY *)hdr;
        if (h->canary != SMEM_PRIVATE_CANARY) {
            SCM_LOG("SMEM: canary invalido en la particion\n");
            return STATUS_DATA_ERROR;
        }
        if (h->item == item) {
            return STATUS_OBJECT_NAME_COLLISION;
        }
        next = hdr + sizeof(SMEM_PRIV_ENTRY) + h->padding_hdr + h->size;
        if (next <= hdr) {
            return STATUS_DATA_ERROR;
        }
        hdr = next;
    }
    if (hdr > end) {
        return STATUS_DATA_ERROR;
    }

    allocSize = sizeof(SMEM_PRIV_ENTRY) + ALIGN_UP(size, 8);
    if (hdr + allocSize >= cached) {
        SCM_LOG("SMEM: sin espacio\n");
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    h = (volatile SMEM_PRIV_ENTRY *)hdr;
    h->canary = SMEM_PRIVATE_CANARY;
    h->item = (UINT16)item;
    h->size = (UINT32)ALIGN_UP(size, 8);
    h->padding_data = (UINT16)(h->size - size);
    h->padding_hdr = 0;

    /* el header tiene que verse completo antes de mover el offset libre */
    KeMemoryBarrier();
    ph->offset_free_uncached += (UINT32)allocSize;
    return STATUS_SUCCESS;
}

static NTSTATUS GetPrivate(SMEM_CTX *S, const SMEM_PTABLE_ENTRY *pe,
                           UINT32 item, PVOID *Ptr, SIZE_T *Size)
{
    const UINT8 *base = (const UINT8 *)S->Base + pe->offset;
    const SMEM_PART_HDR *ph = (const SMEM_PART_HDR *)base;
    UINT32 psize = pe->size;
    const UINT8 *pEnd = base + psize;
    const UINT8 *e = base + sizeof(SMEM_PART_HDR);
    const UINT8 *uncachedEnd = base + ph->offset_free_uncached;
    const UINT8 *cachedEnd = base + ph->offset_free_cached;
    UINT32 cl = pe->cacheline;

    if (uncachedEnd < base || uncachedEnd > cachedEnd || cachedEnd > pEnd) {
        return STATUS_DATA_ERROR;
    }

    /* region sin cache: se llena desde el principio */
    while (e + sizeof(SMEM_PRIV_ENTRY) < uncachedEnd) {
        const SMEM_PRIV_ENTRY *h = (const SMEM_PRIV_ENTRY *)e;
        const UINT8 *next;

        if (h->canary != SMEM_PRIVATE_CANARY) {
            return STATUS_DATA_ERROR;
        }
        if (h->item == item) {
            UINT32 esz = h->size, pad = h->padding_data;
            const UINT8 *p;

            if (!(esz < psize && pad < esz)) {
                return STATUS_DATA_ERROR;
            }
            p = e + sizeof(SMEM_PRIV_ENTRY) + h->padding_hdr;
            if (p + (esz - pad) > uncachedEnd) {
                return STATUS_DATA_ERROR;
            }
            *Ptr = (PVOID)p;
            if (Size != NULL) {
                *Size = esz - pad;
            }
            return STATUS_SUCCESS;
        }
        next = e + sizeof(SMEM_PRIV_ENTRY) + h->padding_hdr + h->size;
        if (next <= e) {
            return STATUS_DATA_ERROR;
        }
        e = next;
    }
    if (e > uncachedEnd) {
        return STATUS_DATA_ERROR;
    }

    /* region con cache: se llena desde el final hacia atras */
    if (cachedEnd == pEnd) {
        return STATUS_NOT_FOUND;
    }
    if (cl == 0) {
        return STATUS_DATA_ERROR;
    }
    e = base + psize - ALIGN_UP(sizeof(SMEM_PART_HDR), cl);

    while (e > cachedEnd) {
        const SMEM_PRIV_ENTRY *h = (const SMEM_PRIV_ENTRY *)e;
        const UINT8 *next;

        if (h->canary != SMEM_PRIVATE_CANARY) {
            return STATUS_DATA_ERROR;
        }
        if (h->item == item) {
            UINT32 esz = h->size, pad = h->padding_data;
            const UINT8 *p;

            if (!(esz < psize && pad < esz)) {
                return STATUS_DATA_ERROR;
            }
            p = e - esz;
            if (p < cachedEnd || p + (esz - pad) > e) {
                return STATUS_DATA_ERROR;
            }
            *Ptr = (PVOID)p;
            if (Size != NULL) {
                *Size = esz - pad;
            }
            return STATUS_SUCCESS;
        }
        next = e - h->size - ALIGN_UP(sizeof(SMEM_PRIV_ENTRY), cl);
        if (next >= e) {
            return STATUS_DATA_ERROR;
        }
        e = next;
    }
    return STATUS_NOT_FOUND;
}

NTSTATUS SmemAlloc(SMEM_CTX *S, UINT32 Host, UINT32 Item, SIZE_T Size)
{
    const SMEM_PTABLE_ENTRY *pe;
    NTSTATUS st;

    if (Item < SMEM_ITEM_LAST_FIXED || Item >= S->ItemCount) {
        return STATUS_INVALID_PARAMETER;
    }
    pe = SelectPartition(S, Host);
    if (pe == NULL) {
        return STATUS_NOT_SUPPORTED;     /* heap global sin implementar */
    }
    st = LockAcquire(S);
    if (!NT_SUCCESS(st)) {
        return st;
    }
    st = AllocPrivate(S, pe, Item, Size);
    LockRelease(S);
    return st;
}

NTSTATUS SmemGet(SMEM_CTX *S, UINT32 Host, UINT32 Item,
                 PVOID *Ptr, SIZE_T *Size)
{
    const SMEM_PTABLE_ENTRY *pe;
    NTSTATUS st;

    if (Item >= S->ItemCount) {
        return STATUS_INVALID_PARAMETER;
    }
    pe = SelectPartition(S, Host);
    if (pe == NULL) {
        return STATUS_NOT_SUPPORTED;
    }
    /* Como Linux (qcom_smem_get): leer en una particion privada no toma el
     * hwlock, asi no competimos con el ADSP mientras arranca. */
    st = GetPrivate(S, pe, Item, Ptr, Size);
    return st;
}

/* Diagnostico: lista los items de la particion privada con el host. */
VOID SmemDumpPartition(SMEM_CTX *S, UINT32 Host)
{
    const SMEM_PTABLE_ENTRY *pe;
    volatile UINT8 *base, *hdr, *end, *pEnd;
    volatile SMEM_PART_HDR *ph;
    UINT32 n = 0;

    if (S == NULL || (pe = SelectPartition(S, Host)) == NULL) {
        SCM_LOG("SMEM: dump: sin particion para el host %u\n", (unsigned)Host);
        return;
    }
    base = S->Base + pe->offset;
    ph = (volatile SMEM_PART_HDR *)base;
    pEnd = base + pe->size;
    hdr = base + sizeof(SMEM_PART_HDR);
    end = base + ph->offset_free_uncached;
    SCM_LOG("SMEM: particion host %u: tam=0x%x libre_uncached=0x%x libre_cached=0x%x\n",
            (unsigned)Host, (unsigned)pe->size, (unsigned)ph->offset_free_uncached,
            (unsigned)ph->offset_free_cached);
    if (end < base || end > pEnd) {
        return;
    }
    while (hdr + sizeof(SMEM_PRIV_ENTRY) < end && n < 64) {
        volatile SMEM_PRIV_ENTRY *h = (volatile SMEM_PRIV_ENTRY *)hdr;
        volatile UINT8 *next;

        if (h->canary != SMEM_PRIVATE_CANARY) {
            SCM_LOG("SMEM:   canary invalido en +0x%x\n", (unsigned)(hdr - base));
            return;
        }
        SCM_LOG("SMEM:   +0x%05x item %u tam=%u pad=%u\n",
                (unsigned)(hdr - base), (unsigned)h->item, (unsigned)h->size,
                (unsigned)h->padding_data);
        next = hdr + sizeof(SMEM_PRIV_ENTRY) + h->padding_hdr + h->size;
        if (next <= hdr) {
            return;
        }
        hdr = next;
        n++;
    }
}
