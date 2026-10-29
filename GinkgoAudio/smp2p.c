/*
 * smp2p.c - SMP2P minimo con el ADSP (ver smp2p.h).
 *
 * Diferencias con el kernel: sin interrupciones (se hace polling de la entrada)
 * y sin el interrupt controller virtual; solo se leen valores.
 *
 * El timbre (writel(BIT(idx), base + offset)) esta verificado contra
 * qcom-apcs-ipc-mailbox.c del kernel sm6125 (offset 8 para trinket).
 * A VERIFICAR: el orden y los nombres de las entradas salientes salen del DTS
 * (master-kernel, rdbg, sleepstate); el kernel las registra en el orden de los
 * nodos hijos.
 */
#include "smp2p.h"

#define SMP2P_MAGIC               0x504D5324u
#define SMP2P_VERSION             1
#define SMP2P_FEATURES            1u            /* SSR_ACK */
#define SMP2P_FEATURE_SSR_ACK     1u
#define SMP2P_MAX_ENTRY           16
#define SMP2P_NAME_LEN            16
#define SMP2P_FLAG_RESTART_DONE   (1u << 0)
#define SMP2P_FLAG_RESTART_ACK    (1u << 1)
#define SMP2P_PAGE                0x1000u

#ifndef C_ASSERT
#define C_ASSERT(e) typedef char __C_ASSERT__[(e) ? 1 : -1]
#endif
C_ASSERT(sizeof(SMP2P_ITEM_ENTRY) == 20);
C_ASSERT(sizeof(SMP2P_ITEM) == 340);

static const char *const kOutEntries[] = { "master-kernel", "rdbg", "sleepstate" };

static UINT32 FeatGet(volatile SMP2P_ITEM *i)
{
    return (UINT32)i->features[0] | ((UINT32)i->features[1] << 8) |
           ((UINT32)i->features[2] << 16);
}

static VOID FeatSet(volatile SMP2P_ITEM *i, UINT32 v)
{
    i->features[0] = (UINT8)(v & 0xFF);
    i->features[1] = (UINT8)((v >> 8) & 0xFF);
    i->features[2] = (UINT8)((v >> 16) & 0xFF);
}

static BOOLEAN NameEq(const volatile UINT8 *a, const char *b)
{
    UINT32 i;

    for (i = 0; i < SMP2P_NAME_LEN; i++) {
        UINT8 cb = (UINT8)b[i];

        if (a[i] != cb) {
            return FALSE;
        }
        if (cb == 0) {
            return TRUE;
        }
    }
    return TRUE;
}

static volatile SMP2P_ITEM_ENTRY *FindEntry(volatile SMP2P_ITEM *item,
                                            const char *name)
{
    UINT32 i, n = item->valid_entries;

    if (n > SMP2P_MAX_ENTRY) {
        n = SMP2P_MAX_ENTRY;
    }
    for (i = 0; i < n; i++) {
        if (NameEq(item->entries[i].name, name)) {
            return &item->entries[i];
        }
    }
    return NULL;
}

/* timbre al ADSP: los datos tienen que verse antes del aviso */
static VOID Kick(SMP2P_CTX *C)
{
    KeMemoryBarrier();
    WRITE_REGISTER_ULONG(C->Ipc, 1u << C->Cfg.IpcBit);
}

NTSTATUS Smp2pOpen(SMP2P_CTX *C, const SMP2P_CONFIG *Cfg)
{
    PHYSICAL_ADDRESS pa;
    PVOID p = NULL;
    SIZE_T sz = 0;
    volatile SMP2P_ITEM *out;
    NTSTATUS st;
    UINT32 i;

    RtlZeroMemory(C, sizeof(*C));
    C->Cfg = *Cfg;

    pa.QuadPart = (LONGLONG)Cfg->MailboxPa;
    C->MboxMap = MmMapIoSpace(pa, SMP2P_PAGE, MmNonCached);
    if (C->MboxMap == NULL) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    C->Ipc = (volatile ULONG *)((UINT8 *)C->MboxMap + Cfg->IpcOffset);

    st = SmemAlloc(Cfg->Smem, Cfg->RemotePid, Cfg->OutItem, sizeof(SMP2P_ITEM));
    if (!NT_SUCCESS(st) && st != STATUS_OBJECT_NAME_COLLISION) {
        SCM_LOG("SMP2P: no se pudo reservar el item saliente nt=0x%08x\n",
                (unsigned)st);
        Smp2pClose(C);
        return st;
    }
    st = SmemGet(Cfg->Smem, Cfg->RemotePid, Cfg->OutItem, &p, &sz);
    if (!NT_SUCCESS(st) || sz < sizeof(SMP2P_ITEM)) {
        SCM_LOG("SMP2P: no se pudo leer el item saliente nt=0x%08x\n",
                (unsigned)st);
        Smp2pClose(C);
        return NT_SUCCESS(st) ? STATUS_DATA_ERROR : st;
    }

    out = (volatile SMP2P_ITEM *)p;
    RtlZeroMemory((PVOID)out, sizeof(SMP2P_ITEM));
    out->magic = SMP2P_MAGIC;
    out->local_pid = (UINT16)Cfg->LocalPid;
    out->remote_pid = (UINT16)Cfg->RemotePid;
    out->total_entries = SMP2P_MAX_ENTRY;
    out->valid_entries = 0;
    FeatSet(out, SMP2P_FEATURES);

    /* el resto del header tiene que verse antes de validar con la version */
    KeMemoryBarrier();
    out->version = SMP2P_VERSION;
    C->Out = out;
    Kick(C);

    for (i = 0; i < sizeof(kOutEntries) / sizeof(kOutEntries[0]); i++) {
        volatile SMP2P_ITEM_ENTRY *e = &out->entries[out->valid_entries];
        BOOLEAN ended = FALSE;
        UINT32 k;

        /* nombre con relleno de ceros hasta 16 bytes (el item ya esta en cero) */
        for (k = 0; k < SMP2P_NAME_LEN; k++) {
            if (!ended && kOutEntries[i][k] == 0) {
                ended = TRUE;
            }
            e->name[k] = ended ? 0 : (UINT8)kOutEntries[i][k];
        }
        e->value = 0;
        out->valid_entries++;
    }

    /* avisar de nuevo despues de crear las entradas */
    Kick(C);

    SCM_LOG("SMP2P: item saliente 0x%x listo con %u entradas\n",
            (unsigned)Cfg->OutItem, (unsigned)out->valid_entries);
    return STATUS_SUCCESS;
}

VOID Smp2pClose(SMP2P_CTX *C)
{
    /* el kernel pone valid_entries = 0 al cerrar */
    if (C->Out != NULL) {
        C->Out->valid_entries = 0;
    }
    if (C->MboxMap != NULL) {
        MmUnmapIoSpace(C->MboxMap, SMP2P_PAGE);
    }
    C->MboxMap = NULL;
    C->Ipc = NULL;
}

NTSTATUS Smp2pPoll(SMP2P_CTX *C)
{
    volatile SMP2P_ITEM *in, *out = C->Out;

    if (out == NULL) {
        return STATUS_DEVICE_NOT_READY;
    }

    if (C->In == NULL) {
        PVOID p = NULL;
        SIZE_T sz = 0;
        NTSTATUS st = SmemGet(C->Cfg.Smem, C->Cfg.RemotePid, C->Cfg.InItem,
                              &p, &sz);

        if (!NT_SUCCESS(st)) {
            return STATUS_DEVICE_NOT_READY;      /* el ADSP todavia no lo creo */
        }
        if (sz < sizeof(SMP2P_ITEM)) {
            SCM_LOG("SMP2P: item entrante muy chico (%llu)\n",
                    (unsigned long long)sz);
            return STATUS_DATA_ERROR;
        }
        C->In = (volatile SMP2P_ITEM *)p;
    }

    in = C->In;
    if (in->magic != SMP2P_MAGIC) {
        return STATUS_DEVICE_NOT_READY;
    }

    if (!C->Open && in->version == out->version) {
        UINT32 f = FeatGet(in) & FeatGet(out);

        FeatSet(out, f);
        C->SsrAckEnabled = (f & SMP2P_FEATURE_SSR_ACK) != 0;
        C->Open = TRUE;
        SCM_LOG("SMP2P: canal abierto, ssr_ack=%d\n", (int)C->SsrAckEnabled);
    }
    if (!C->Open) {
        return STATUS_DEVICE_NOT_READY;
    }

    if (C->SsrAckEnabled) {
        BOOLEAN restart = (in->flags & SMP2P_FLAG_RESTART_DONE) != 0;

        if (restart != C->SsrAck) {
            UINT32 val;

            SCM_LOG("SMP2P: reinicio del ADSP detectado, enviando ack\n");
            C->SsrAck = (BOOLEAN)!C->SsrAck;
            val = out->flags & ~SMP2P_FLAG_RESTART_ACK;
            if (C->SsrAck) {
                val |= SMP2P_FLAG_RESTART_ACK;
            }
            out->flags = val;
            Kick(C);
        }
    }
    return STATUS_SUCCESS;
}

NTSTATUS Smp2pReadIn(SMP2P_CTX *C, const char *Name, UINT32 *Value)
{
    volatile SMP2P_ITEM_ENTRY *e;

    if (!C->Open) {
        return STATUS_DEVICE_NOT_READY;
    }
    e = FindEntry(C->In, Name);
    if (e == NULL) {
        return STATUS_NOT_FOUND;
    }
    *Value = e->value;
    return STATUS_SUCCESS;
}

NTSTATUS Smp2pUpdateOut(SMP2P_CTX *C, const char *Name,
                        UINT32 Mask, UINT32 Value)
{
    volatile SMP2P_ITEM_ENTRY *e;
    UINT32 orig, val;

    if (C->Out == NULL) {
        return STATUS_DEVICE_NOT_READY;
    }
    e = FindEntry(C->Out, Name);
    if (e == NULL) {
        return STATUS_NOT_FOUND;
    }
    orig = e->value;
    val = (orig & ~Mask) | Value;
    e->value = val;
    if (val != orig) {
        Kick(C);
    }
    return STATUS_SUCCESS;
}

NTSTATUS Smp2pWaitReady(SMP2P_CTX *C, ULONG TimeoutMs)
{
    LARGE_INTEGER delay;
    ULONG waited;
    UINT32 val = 0, last = 0xFFFFFFFFu;

    delay.QuadPart = -(LONGLONG)10 * 10000;      /* 10 ms */

    for (waited = 0; waited <= TimeoutMs; waited += 10) {
        if (NT_SUCCESS(Smp2pPoll(C)) &&
            NT_SUCCESS(Smp2pReadIn(C, "slave-kernel", &val))) {
            if (val != last) {
                SCM_LOG("SMP2P: slave-kernel = 0x%08x\n", (unsigned)val);
                last = val;
            }
            if (val & SMP2P_ADSP_ERR_FATAL) {
                return STATUS_UNSUCCESSFUL;
            }
            if (val & SMP2P_ADSP_ERR_READY) {
                return STATUS_SUCCESS;
            }
        }
        KeDelayExecutionThread(KernelMode, FALSE, &delay);
    }
    SCM_LOG("SMP2P: timeout esperando err-ready (abierto=%d)\n", (int)C->Open);
    return STATUS_IO_TIMEOUT;
}
