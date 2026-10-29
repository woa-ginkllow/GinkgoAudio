/*
 * glink.c - transporte GLINK nativo sobre SMEM (ver glink.h).
 *
 * Referencias en el kernel sm6125:
 *   drivers/rpmsg/qcom_glink_smem.c    FIFOs, descriptor, avail/peak/advance
 *   drivers/rpmsg/qcom_glink_native.c  comandos, intents, fragmentacion
 */
#include "glink.h"

/* ---- comandos (RPM_CMD_*) ------------------------------------------------ */
#define CMD_VERSION             0
#define CMD_VERSION_ACK         1
#define CMD_OPEN                2
#define CMD_CLOSE               3
#define CMD_OPEN_ACK            4
#define CMD_INTENT              5
#define CMD_RX_DONE             6
#define CMD_RX_INTENT_REQ       7
#define CMD_RX_INTENT_REQ_ACK   8
#define CMD_TX_DATA             9
#define CMD_CLOSE_ACK           11
#define CMD_TX_DATA_CONT        12
#define CMD_READ_NOTIF          13
#define CMD_RX_DONE_W_REUSE     14
#define CMD_SIGNALS             15

#define GLINK_VERSION_1         1
#define MSG_SIZE                8        /* struct glink_msg                  */
#define DATA_HDR_SIZE           16       /* glink_msg + chunk_size + left_size */
#define MAX_CHUNK               8192     /* SZ_8K del original                */
#define FIFO_FULL_RESERVE       8
#define TX_BLOCKED_CMD_RESERVE  8
#define SMEM_DESCRIPTOR         478
#define SMEM_FIFO_TX            479      /* FIFO_0 */
#define SMEM_FIFO_RX            480      /* FIFO_1 */
#define FIFO_TX_SIZE            16384
#define CTL_TIMEOUT_MS          1000
#define MAX_MSGS_PER_POLL       512
#define MAX_OPEN_NAME           128

#define ALIGN8(x) (((x) + 7u) & ~7u)

/* resultado de un handler del rx */
#define RX_OK     0
#define RX_WAIT   1     /* falta data en el FIFO: reintentar en el proximo poll */
#define RX_FAULT  2     /* FIFO corrupto: no se puede resincronizar             */

/* ---- utilidades ---------------------------------------------------------- */

static VOID P16(UINT8 *b, UINT32 o, UINT32 v)
{
    b[o]     = (UINT8)v;
    b[o + 1] = (UINT8)(v >> 8);
}

static VOID P32(UINT8 *b, UINT32 o, UINT32 v)
{
    b[o]     = (UINT8)v;
    b[o + 1] = (UINT8)(v >> 8);
    b[o + 2] = (UINT8)(v >> 16);
    b[o + 3] = (UINT8)(v >> 24);
}

static UINT32 G16(const UINT8 *b, UINT32 o)
{
    return (UINT32)b[o] | ((UINT32)b[o + 1] << 8);
}

static UINT32 G32(const UINT8 *b, UINT32 o)
{
    return (UINT32)b[o] | ((UINT32)b[o + 1] << 8) |
           ((UINT32)b[o + 2] << 16) | ((UINT32)b[o + 3] << 24);
}

typedef struct _GL_WAIT {
    ULONGLONG Start;
    ULONGLONG Limit;
} GL_WAIT;

static VOID WaitStart(GL_WAIT *W, ULONG Ms)
{
    W->Start = KeQueryInterruptTime();
    W->Limit = (ULONGLONG)Ms * 10000ull;          /* unidades de 100 ns */
}

static BOOLEAN WaitExpired(const GL_WAIT *W)
{
    return (BOOLEAN)((KeQueryInterruptTime() - W->Start) >= W->Limit);
}

/* Los primeros 2 ms se espera activamente; despues se cede la CPU. */
static VOID WaitPause(const GL_WAIT *W)
{
    ULONGLONG el = KeQueryInterruptTime() - W->Start;

    if (KeGetCurrentIrql() >= DISPATCH_LEVEL || el < 20000ull) {
        KeStallExecutionProcessor(50);
    } else {
        LARGE_INTEGER d;

        d.QuadPart = -10000;                      /* 1 ms */
        KeDelayExecutionThread(KernelMode, FALSE, &d);
    }
}

static VOID StrCopy(char *Dst, const char *Src, UINT32 Max)
{
    UINT32 i = 0;

    while (i + 1 < Max && Src[i] != 0) {
        Dst[i] = Src[i];
        i++;
    }
    while (i < Max) {
        Dst[i++] = 0;
    }
}

static BOOLEAN StrEq(const char *A, const char *B)
{
    UINT32 i;

    for (i = 0; i < GLINK_NAME_SIZE; i++) {
        if (A[i] != B[i]) {
            return FALSE;
        }
        if (A[i] == 0) {
            return TRUE;
        }
    }
    return TRUE;
}

/* ---- FIFOs ---------------------------------------------------------------- */

static VOID Kick(GLINK_CTX *G)
{
    KeMemoryBarrier();
    WRITE_REGISTER_ULONG(G->Cfg.Ipc, 1u << G->Cfg.IpcBit);
}

/* Intenta obtener el FIFO rx (lo crea el ADSP). */
static BOOLEAN RxReady(GLINK_CTX *G)
{
    PVOID p = NULL;
    SIZE_T sz = 0;

    if (G->RxFifo != NULL) {
        return TRUE;
    }
    /* SmemGet toma el hwlock: no se pide en cada vuelta del polling */
    if ((G->RxProbe++ % 20u) != 0) {
        return FALSE;
    }
    if (!NT_SUCCESS(SmemGet(G->Cfg.Smem, G->Cfg.RemotePid, SMEM_FIFO_RX,
                            &p, &sz)) || p == NULL || sz < 64) {
        return FALSE;
    }
    G->RxLen = (UINT32)sz;
    G->RxFifo = (volatile UINT8 *)p;
    SCM_LOG("GLINK: FIFO rx (item 480) encontrado, %u bytes\n",
            (unsigned)G->RxLen);
    return TRUE;
}

static UINT32 RxAvail(GLINK_CTX *G)
{
    UINT32 head, tail;

    if (!RxReady(G)) {
        return 0;
    }
    head = *G->RxHead;
    tail = *G->RxTail;
    if (head >= G->RxLen || tail >= G->RxLen) {
        if (!G->RxIdxBad) {
            G->RxIdxBad = TRUE;
            SCM_LOG("GLINK: indices rx invalidos head=%u tail=%u len=%u\n",
                    (unsigned)head, (unsigned)tail, (unsigned)G->RxLen);
        }
        return 0;
    }
    KeMemoryBarrier();           /* head primero, datos despues */
    return (head >= tail) ? (head - tail) : (G->RxLen - tail + head);
}

/* Copia sin consumir, con vuelta al principio del FIFO. */
static VOID RxPeek(GLINK_CTX *G, UINT32 Off, UINT8 *Dst, UINT32 Count)
{
    UINT32 pos = (*G->RxTail + Off) % G->RxLen;
    UINT32 i;

    for (i = 0; i < Count; i++) {
        Dst[i] = G->RxFifo[pos];
        if (++pos >= G->RxLen) {
            pos = 0;
        }
    }
}

static VOID RxAdvance(GLINK_CTX *G, UINT32 Count)
{
    UINT32 tail = *G->RxTail;

    KeMemoryBarrier();           /* termine de leer antes de liberar */
    tail += Count;
    if (tail >= G->RxLen) {
        tail %= G->RxLen;
    }
    *G->RxTail = tail;
}

static UINT32 TxAvail(GLINK_CTX *G)
{
    UINT32 head = *G->TxHead;
    UINT32 tail = *G->TxTail;
    UINT32 avail;

    if (head >= G->TxLen || tail >= G->TxLen) {
        return 0;
    }
    if (tail <= head) {
        avail = G->TxLen - head + tail;
    } else {
        avail = tail - head;
    }
    if (avail < FIFO_FULL_RESERVE + TX_BLOCKED_CMD_RESERVE) {
        return 0;
    }
    return avail - (FIFO_FULL_RESERVE + TX_BLOCKED_CMD_RESERVE);
}

static UINT32 FifoPut(volatile UINT8 *Fifo, UINT32 Len, UINT32 Head,
                      const UINT8 *Src, UINT32 N)
{
    UINT32 i;

    for (i = 0; i < N; i++) {
        Fifo[Head] = Src[i];
        if (++Head >= Len) {
            Head = 0;
        }
    }
    return Head;
}

/* Con TxLock tomado. */
static VOID TxWrite(GLINK_CTX *G, const UINT8 *Hdr, UINT32 HLen,
                    const UINT8 *Data, UINT32 DLen)
{
    UINT32 head = *G->TxHead;

    head = FifoPut(G->TxFifo, G->TxLen, head, Hdr, HLen);
    if (DLen != 0) {
        head = FifoPut(G->TxFifo, G->TxLen, head, Data, DLen);
    }
    head = ALIGN8(head);                  /* head siempre alineado a 8 */
    if (head >= G->TxLen) {
        head -= G->TxLen;
    }
    KeMemoryBarrier();                    /* FIFO visible antes que head */
    *G->TxHead = head;
}

static NTSTATUS TxMsg(GLINK_CTX *G, const UINT8 *Hdr, UINT32 HLen,
                      const UINT8 *Data, UINT32 DLen, ULONG TimeoutMs)
{
    UINT32 tlen = HLen + DLen;
    GL_WAIT w;

    if (!G->Active || G->TxFifo == NULL) {
        return STATUS_DEVICE_NOT_READY;
    }
    if (tlen >= G->TxLen) {
        return STATUS_INVALID_PARAMETER;
    }

    WaitStart(&w, TimeoutMs);
    for (;;) {
        KIRQL irql;

        KeAcquireSpinLock(&G->TxLock, &irql);
        if (TxAvail(G) >= tlen) {
            TxWrite(G, Hdr, HLen, Data, DLen);
            Kick(G);
            G->SentReadNotify = FALSE;
            G->TxMsgs++;
            KeReleaseSpinLock(&G->TxLock, irql);
            return STATUS_SUCCESS;
        }
        if (!G->SentReadNotify) {
            UINT8 rn[MSG_SIZE];

            /* pedir al remoto que avise cuando lea (cabe en la reserva) */
            G->SentReadNotify = TRUE;
            P16(rn, 0, CMD_READ_NOTIF);
            P16(rn, 2, 0);
            P32(rn, 4, 0);
            TxWrite(G, rn, MSG_SIZE, NULL, 0);
            Kick(G);
        }
        KeReleaseSpinLock(&G->TxLock, irql);

        if (WaitExpired(&w)) {
            SCM_LOG("GLINK: FIFO tx lleno, timeout (tlen=%u)\n", (unsigned)tlen);
            return STATUS_IO_TIMEOUT;
        }
        WaitPause(&w);
    }
}

/* ---- mensajes de control -------------------------------------------------- */

static NTSTATUS SendMsg(GLINK_CTX *G, UINT32 Cmd, UINT32 P1, UINT32 P2)
{
    UINT8 m[MSG_SIZE];

    P16(m, 0, Cmd);
    P16(m, 2, P1);
    P32(m, 4, P2);
    return TxMsg(G, m, MSG_SIZE, NULL, 0, CTL_TIMEOUT_MS);
}

static NTSTATUS SendVersion(GLINK_CTX *G)
{
    SCM_LOG("GLINK: -> VERSION v%u features=0x%x\n", GLINK_VERSION_1,
            (unsigned)G->Features);
    return SendMsg(G, CMD_VERSION, GLINK_VERSION_1, G->Features);
}

static NTSTATUS SendVersionAck(GLINK_CTX *G)
{
    SCM_LOG("GLINK: -> VERSION_ACK v%u features=0x%x\n", GLINK_VERSION_1,
            (unsigned)G->Features);
    return SendMsg(G, CMD_VERSION_ACK, GLINK_VERSION_1, G->Features);
}

static NTSTATUS SendOpenReq(GLINK_CTX *G, const GLINK_CHANNEL *Ch)
{
    UINT8 req[MSG_SIZE + GLINK_NAME_SIZE];
    UINT32 nameLen = 0;
    UINT32 reqLen;

    while (nameLen < GLINK_NAME_SIZE - 1 && Ch->Name[nameLen] != 0) {
        nameLen++;
    }
    nameLen++;                               /* incluye el NUL */
    reqLen = ALIGN8(MSG_SIZE + nameLen);

    RtlZeroMemory(req, sizeof(req));
    P16(req, 0, CMD_OPEN);
    P16(req, 2, Ch->Lcid);
    P32(req, 4, nameLen);
    RtlCopyMemory(req + MSG_SIZE, Ch->Name, nameLen - 1);
    return TxMsg(G, req, reqLen, NULL, 0, CTL_TIMEOUT_MS);
}

static NTSTATUS SendIntent(GLINK_CTX *G, const GLINK_CHANNEL *Ch,
                           const GLINK_LINTENT *In)
{
    UINT8 c[16];

    P16(c, 0, CMD_INTENT);
    P16(c, 2, Ch->Lcid);
    P32(c, 4, 1);                            /* count */
    P32(c, 8, In->Size);
    P32(c, 12, In->Id);
    SCM_LOG("GLINK: -> INTENT lcid=%u liid=%u size=%u\n",
            (unsigned)Ch->Lcid, (unsigned)In->Id, (unsigned)In->Size);
    return TxMsg(G, c, sizeof(c), NULL, 0, CTL_TIMEOUT_MS);
}

/* ---- canales / intents locales -------------------------------------------- */

static GLINK_CHANNEL *FindByName(GLINK_CTX *G, const char *Name)
{
    UINT32 i;

    for (i = 0; i < GLINK_MAX_CHANNELS; i++) {
        if (G->Ch[i].State != GL_CH_UNUSED && StrEq(G->Ch[i].Name, Name)) {
            return &G->Ch[i];
        }
    }
    return NULL;
}

static GLINK_CHANNEL *FindByRcid(GLINK_CTX *G, UINT32 Rcid)
{
    UINT32 i;

    if (Rcid == 0) {
        return NULL;
    }
    for (i = 0; i < GLINK_MAX_CHANNELS; i++) {
        GLINK_CHANNEL *c = &G->Ch[i];

        if (c->State > GL_CH_REGISTERED && c->Rcid == Rcid) {
            return c;
        }
    }
    return NULL;
}

static GLINK_CHANNEL *FindByLcid(GLINK_CTX *G, UINT32 Lcid)
{
    UINT32 i;

    if (Lcid == 0) {
        return NULL;
    }
    for (i = 0; i < GLINK_MAX_CHANNELS; i++) {
        GLINK_CHANNEL *c = &G->Ch[i];

        if (c->State > GL_CH_REGISTERED && c->Lcid == Lcid) {
            return c;
        }
    }
    return NULL;
}

/* Vuelve a dejar el canal "registrado, esperando OPEN". */
static VOID ChReset(GLINK_CTX *G, GLINK_CHANNEL *Ch)
{
    KIRQL irql;
    UINT32 i;

    KeAcquireSpinLock(&G->Lock, &irql);
    for (i = 0; i < GLINK_MAX_LINTENTS; i++) {
        Ch->L[i].Valid = FALSE;
    }
    for (i = 0; i < GLINK_MAX_RINTENTS; i++) {
        Ch->R[i].Valid = FALSE;
        Ch->R[i].InUse = FALSE;
    }
    for (i = 0; i < GLINK_BIG_SLOTS; i++) {
        Ch->BigUsed[i] = FALSE;
    }
    Ch->Lcid = 0;
    Ch->Rcid = 0;
    Ch->NextIid = 1;
    Ch->Rsigs = 0;
    Ch->IntentReqDone = 0;
    Ch->IntentReqGranted = FALSE;
    KeReleaseSpinLock(&G->Lock, irql);
}

/* idr_alloc_cyclic(1..) del original */
static UINT32 NextIid(GLINK_CHANNEL *Ch)
{
    UINT32 id = Ch->NextIid;

    Ch->NextIid = (Ch->NextIid == 0x7FFFFFFFu) ? 1 : Ch->NextIid + 1;
    return id;
}

static GLINK_LINTENT *AllocLIntent(GLINK_CHANNEL *Ch, UINT32 Size,
                                   UINT8 *Data, BOOLEAN Reuse, LONG Slot)
{
    UINT32 i;

    for (i = 0; i < GLINK_MAX_LINTENTS; i++) {
        GLINK_LINTENT *in = &Ch->L[i];

        if (!in->Valid) {
            in->Id = NextIid(Ch);
            in->Size = Size;
            in->Offset = 0;
            in->Data = Data;
            in->Reuse = Reuse;
            in->BigSlot = Slot;
            in->Valid = TRUE;
            return in;
        }
    }
    return NULL;
}

static GLINK_LINTENT *FindLIntent(GLINK_CHANNEL *Ch, UINT32 Id)
{
    UINT32 i;

    for (i = 0; i < GLINK_MAX_LINTENTS; i++) {
        if (Ch->L[i].Valid && Ch->L[i].Id == Id) {
            return &Ch->L[i];
        }
    }
    return NULL;
}

/* Publica los intents base del canal (announce_create del original). */
static VOID AdvertiseBase(GLINK_CTX *G, GLINK_CHANNEL *Ch)
{
    UINT32 i;

    for (i = 0; i < Ch->StdCount; i++) {
        GLINK_LINTENT *in = AllocLIntent(Ch, Ch->StdSize,
                                         Ch->StdPool + (size_t)i * Ch->StdSize,
                                         TRUE, -1);

        if (in == NULL) {
            SCM_LOG("GLINK: sin sitio para el intent %u\n", (unsigned)i);
            break;
        }
        (VOID)SendIntent(G, Ch, in);
    }
}

/* Remoto abre el canal: contestamos OPEN_ACK y abrimos el nuestro. */
static VOID StartOpen(GLINK_CTX *G, GLINK_CHANNEL *Ch, UINT32 Rcid)
{
    UINT32 lcid;

    if (Ch->State != GL_CH_REGISTERED) {
        SCM_LOG("GLINK: '%s' reabierto por el remoto (estado %ld), reinicio\n",
                Ch->Name, (long)Ch->State);
    }
    ChReset(G, Ch);
    Ch->Rcid = Rcid;

    lcid = G->NextLcid;
    G->NextLcid = (G->NextLcid >= 65535u) ? 1 : G->NextLcid + 1;
    Ch->Lcid = lcid;
    Ch->State = GL_CH_OPENING;

    SCM_LOG("GLINK: OPEN '%s' rcid=%u -> OPEN_ACK + OPEN lcid=%u\n",
            Ch->Name, (unsigned)Rcid, (unsigned)lcid);
    (VOID)SendMsg(G, CMD_OPEN_ACK, Rcid, 0);
    (VOID)SendOpenReq(G, Ch);
}

/* ---- API de canales ------------------------------------------------------- */

NTSTATUS GlinkRegister(GLINK_CTX *G, const char *Name, GLINK_RX_CB Cb,
                       PVOID CbCtx, UINT32 StdSize, UINT32 StdCount,
                       GLINK_CHANNEL **Out)
{
    UINT32 i;

    if (G == NULL || Name == NULL || Cb == NULL || Out == NULL ||
        StdSize == 0 || StdCount == 0 || StdCount > GLINK_MAX_STD_INTENTS ||
        (UINT64)StdSize * StdCount > GLINK_STD_POOL_BYTES) {
        return STATUS_INVALID_PARAMETER;
    }
    if (FindByName(G, Name) != NULL) {
        return STATUS_OBJECT_NAME_COLLISION;
    }
    for (i = 0; i < GLINK_MAX_CHANNELS; i++) {
        GLINK_CHANNEL *c = &G->Ch[i];

        if (c->State == GL_CH_UNUSED) {
            RtlZeroMemory(c, sizeof(*c));
            StrCopy(c->Name, Name, GLINK_NAME_SIZE);
            c->Cb = Cb;
            c->CbCtx = CbCtx;
            c->StdSize = StdSize;
            c->StdCount = StdCount;
            c->NextIid = 1;
            KeMemoryBarrier();
            c->State = GL_CH_REGISTERED;        /* el poll lo ve ya completo */
            *Out = c;
            return STATUS_SUCCESS;
        }
    }
    return STATUS_INSUFFICIENT_RESOURCES;
}

BOOLEAN GlinkIsOpen(const GLINK_CHANNEL *Ch)
{
    return (BOOLEAN)(Ch != NULL && Ch->State == GL_CH_OPEN);
}

NTSTATUS GlinkWaitOpen(GLINK_CTX *G, GLINK_CHANNEL *Ch, ULONG TimeoutMs)
{
    GL_WAIT w;

    WaitStart(&w, TimeoutMs);
    while (!GlinkIsOpen(Ch)) {
        (VOID)GlinkPoll(G);
        if (GlinkIsOpen(Ch)) {
            break;
        }
        if (WaitExpired(&w)) {
            return STATUS_IO_TIMEOUT;
        }
        WaitPause(&w);
    }
    return STATUS_SUCCESS;
}

/* ---- init / cierre -------------------------------------------------------- */

NTSTATUS GlinkInit(GLINK_CTX *G, const GLINK_CONFIG *Cfg)
{
    NTSTATUS st;
    PVOID p = NULL;
    SIZE_T sz = 0;
    volatile UINT32 *d;
    PVOID p0 = NULL;

    if (G == NULL || Cfg == NULL || Cfg->Smem == NULL || Cfg->Ipc == NULL) {
        return STATUS_INVALID_PARAMETER;
    }
    RtlZeroMemory(G, sizeof(*G));
    G->Cfg = *Cfg;
    KeInitializeSpinLock(&G->TxLock);
    KeInitializeSpinLock(&G->Lock);
    G->NextLcid = 1;
    G->Features = GLINK_FEATURE_INTENT_REUSE;

    /* descriptor: tx tail, tx head, rx tail, rx head (4 x u32 + relleno) */
    st = SmemAlloc(Cfg->Smem, Cfg->RemotePid, SMEM_DESCRIPTOR, 32);
    SCM_LOG("GLINK: item %u (descriptor): %s\n", SMEM_DESCRIPTOR,
            st == STATUS_OBJECT_NAME_COLLISION ? "ya existia (lo creo el ADSP)"
                                               : "creado por nosotros");
    if (!NT_SUCCESS(st) && st != STATUS_OBJECT_NAME_COLLISION) {
        SCM_LOG("GLINK: SmemAlloc(descriptor) fallo nt=0x%08x\n", (unsigned)st);
        return st;
    }
    st = SmemGet(Cfg->Smem, Cfg->RemotePid, SMEM_DESCRIPTOR, &p, &sz);
    if (!NT_SUCCESS(st)) {
        SCM_LOG("GLINK: SmemGet(descriptor) fallo nt=0x%08x\n", (unsigned)st);
        return st;
    }
    if (sz != 32) {
        SCM_LOG("GLINK: descriptor de tamano invalido (%u)\n", (unsigned)sz);
        return STATUS_INVALID_BUFFER_SIZE;
    }
    d = (volatile UINT32 *)p;
    p0 = p;
    G->TxTail = &d[0];
    G->TxHead = &d[1];
    G->RxTail = &d[2];
    G->RxHead = &d[3];

    st = SmemAlloc(Cfg->Smem, Cfg->RemotePid, SMEM_FIFO_TX, FIFO_TX_SIZE);
    SCM_LOG("GLINK: item %u (fifo tx): %s\n", SMEM_FIFO_TX,
            st == STATUS_OBJECT_NAME_COLLISION ? "ya existia (lo creo el ADSP)"
                                               : "creado por nosotros");
    if (!NT_SUCCESS(st) && st != STATUS_OBJECT_NAME_COLLISION) {
        SCM_LOG("GLINK: SmemAlloc(fifo tx) fallo nt=0x%08x\n", (unsigned)st);
        return st;
    }
    st = SmemGet(Cfg->Smem, Cfg->RemotePid, SMEM_FIFO_TX, &p, &sz);
    if (!NT_SUCCESS(st)) {
        SCM_LOG("GLINK: SmemGet(fifo tx) fallo nt=0x%08x\n", (unsigned)st);
        return st;
    }
    if (sz < 64 || sz > 0x100000) {
        return STATUS_INVALID_BUFFER_SIZE;
    }
    G->TxFifo = (volatile UINT8 *)p;
    G->TxLen = (UINT32)sz;

    /*
     * Se llama con el ADSP parado: se limpia todo el descriptor (restos de un
     * ciclo anterior) y el FIFO tx. El ADSP pone a cero sus propios indices.
     */
    if (Cfg->AdspStopped) {
        UINT32 i;

        for (i = 0; i < 8; i++) {
            ((volatile UINT32 *)p0)[i] = 0;
        }
    }
    *G->RxTail = 0;
    *G->TxHead = 0;
    KeMemoryBarrier();

    SCM_LOG("GLINK: descriptor y FIFO tx listos (tx %u bytes)\n",
            (unsigned)G->TxLen);
    G->Active = TRUE;

    if (!Cfg->DeferVersion) {
        st = GlinkLinkUp(G);
        if (!NT_SUCCESS(st)) {
            G->Active = FALSE;
            return st;
        }
    }
    return STATUS_SUCCESS;
}

NTSTATUS GlinkLinkUp(GLINK_CTX *G)
{
    NTSTATUS st;

    if (G == NULL || !G->Active) {
        return STATUS_INVALID_DEVICE_STATE;
    }
    st = SendVersion(G);
    if (!NT_SUCCESS(st)) {
        SCM_LOG("GLINK: no se pudo enviar VERSION nt=0x%08x\n", (unsigned)st);
    }
    return st;
}

VOID GlinkClose(GLINK_CTX *G)
{
    GL_WAIT w;

    if (G == NULL || !G->Active) {
        return;
    }
    G->Active = FALSE;
    WaitStart(&w, 200);
    while (G->RxBusy != 0 && !WaitExpired(&w)) {   /* esperar al poll en curso */
        WaitPause(&w);
    }
    G->TxFifo = NULL;
    G->RxFifo = NULL;
    G->TxTail = G->TxHead = G->RxTail = G->RxHead = NULL;
}

VOID GlinkResetShared(SMEM_CTX *Smem, UINT32 RemotePid)
{
    PVOID p = NULL;
    SIZE_T sz = 0;

    if (Smem == NULL) {
        return;
    }
    if (NT_SUCCESS(SmemGet(Smem, RemotePid, SMEM_DESCRIPTOR, &p, &sz)) &&
        p != NULL && sz == 32) {
        volatile UINT32 *d = (volatile UINT32 *)p;
        UINT32 i;

        for (i = 0; i < 8; i++) {
            d[i] = 0;
        }
        KeMemoryBarrier();
        SCM_LOG("GLINK: descriptor compartido puesto a cero\n");
    }
}

/* ---- recepcion ------------------------------------------------------------ */

static VOID OnVersion(GLINK_CTX *G, UINT32 Version, UINT32 Features)
{
    SCM_LOG("GLINK: <- VERSION v%u features=0x%x\n", (unsigned)Version,
            (unsigned)Features);
    switch (Version) {
    case 0:
        break;
    case GLINK_VERSION_1:
        G->Features &= Features;
        /* fallthrough */
    default:
        (VOID)SendVersionAck(G);
        break;
    }
}

static VOID OnVersionAck(GLINK_CTX *G, UINT32 Version, UINT32 Features)
{
    SCM_LOG("GLINK: <- VERSION_ACK v%u features=0x%x\n", (unsigned)Version,
            (unsigned)Features);
    switch (Version) {
    case 0:
        SCM_LOG("GLINK: el remoto rechazo la version\n");
        break;
    case GLINK_VERSION_1:
        if (Features == G->Features) {
            break;
        }
        G->Features &= Features;
        /* fallthrough */
    default:
        (VOID)SendVersion(G);
        break;
    }
}

static VOID OnRemoteOpen(GLINK_CTX *G, UINT32 Rcid, const char *Name)
{
    GLINK_CHANNEL *ch = FindByName(G, Name);
    UINT32 i;

    SCM_LOG("GLINK: <- OPEN '%s' rcid=%u\n", Name, (unsigned)Rcid);
    if (ch != NULL) {
        StartOpen(G, ch, Rcid);
        return;
    }
    /* sin driver todavia: se recuerda (si nadie lo registra el ADSP espera) */
    for (i = 0; i < GLINK_MAX_PENDING; i++) {
        if (G->Pending[i].Used && StrEq(G->Pending[i].Name, Name)) {
            G->Pending[i].Rcid = Rcid;
            return;
        }
    }
    for (i = 0; i < GLINK_MAX_PENDING; i++) {
        if (!G->Pending[i].Used) {
            StrCopy(G->Pending[i].Name, Name, GLINK_NAME_SIZE);
            G->Pending[i].Rcid = Rcid;
            G->Pending[i].Used = TRUE;
            return;
        }
    }
    SCM_LOG("GLINK: tabla de OPEN pendientes llena, se ignora '%s'\n", Name);
}

static VOID OnOpenAck(GLINK_CTX *G, UINT32 Lcid)
{
    GLINK_CHANNEL *ch = FindByLcid(G, Lcid);

    if (ch == NULL) {
        SCM_LOG("GLINK: OPEN_ACK invalido lcid=%u\n", (unsigned)Lcid);
        return;
    }
    SCM_LOG("GLINK: <- OPEN_ACK '%s' lcid=%u\n", ch->Name, (unsigned)Lcid);
    if (ch->State == GL_CH_OPENING) {
        AdvertiseBase(G, ch);
        ch->State = GL_CH_OPEN;
        SCM_LOG("GLINK: canal '%s' abierto\n", ch->Name);
    }
}

static VOID OnRemoteClose(GLINK_CTX *G, UINT32 Rcid)
{
    GLINK_CHANNEL *ch = FindByRcid(G, Rcid);

    SCM_LOG("GLINK: <- CLOSE rcid=%u\n", (unsigned)Rcid);
    (VOID)SendMsg(G, CMD_CLOSE_ACK, Rcid, 0);
    if (ch == NULL) {
        return;
    }
    ch->Rcid = 0;
    if (ch->Lcid != 0) {
        ch->State = GL_CH_CLOSING;
        SCM_LOG("GLINK: -> CLOSE lcid=%u\n", (unsigned)ch->Lcid);
        (VOID)SendMsg(G, CMD_CLOSE, ch->Lcid, 0);
    } else {
        ChReset(G, ch);
        ch->State = GL_CH_REGISTERED;
    }
}

static VOID OnCloseAck(GLINK_CTX *G, UINT32 Lcid)
{
    GLINK_CHANNEL *ch = FindByLcid(G, Lcid);

    SCM_LOG("GLINK: <- CLOSE_ACK lcid=%u\n", (unsigned)Lcid);
    if (ch == NULL) {
        return;
    }
    ChReset(G, ch);
    ch->State = GL_CH_REGISTERED;    /* queda listo para el proximo OPEN */
}

static VOID OnIntentReq(GLINK_CTX *G, UINT32 Rcid, UINT32 Size)
{
    GLINK_CHANNEL *ch = FindByRcid(G, Rcid);
    GLINK_LINTENT *in = NULL;
    UINT32 i;

    if (ch == NULL) {
        SCM_LOG("GLINK: RX_INTENT_REQ de canal desconocido rcid=%u\n",
                (unsigned)Rcid);
        return;
    }
    SCM_LOG("GLINK: <- RX_INTENT_REQ '%s' size=%u\n", ch->Name, (unsigned)Size);

    for (i = 0; i < GLINK_MAX_LINTENTS; i++) {
        if (ch->L[i].Valid && ch->L[i].Reuse && ch->L[i].Size >= Size) {
            in = &ch->L[i];
            break;
        }
    }
    if (in != NULL) {
        (VOID)SendMsg(G, CMD_RX_INTENT_REQ_ACK, ch->Lcid, 1);
        return;
    }
    if (Size <= GLINK_BIG_SIZE) {
        for (i = 0; i < GLINK_BIG_SLOTS; i++) {
            if (!ch->BigUsed[i]) {
                in = AllocLIntent(ch, GLINK_BIG_SIZE, ch->BigPool[i], FALSE,
                                  (LONG)i);
                if (in != NULL) {
                    ch->BigUsed[i] = TRUE;
                    (VOID)SendIntent(G, ch, in);
                }
                break;
            }
        }
    }
    (VOID)SendMsg(G, CMD_RX_INTENT_REQ_ACK, ch->Lcid, in != NULL ? 1 : 0);
}

static VOID OnIntentReqAck(GLINK_CTX *G, UINT32 Rcid, UINT32 Granted)
{
    GLINK_CHANNEL *ch = FindByRcid(G, Rcid);

    if (ch == NULL) {
        return;
    }
    ch->IntentReqGranted = (BOOLEAN)(Granted != 0);
    KeMemoryBarrier();
    InterlockedIncrement(&ch->IntentReqDone);
}

static VOID OnRxDone(GLINK_CTX *G, UINT32 Rcid, UINT32 Iid, BOOLEAN Reuse)
{
    GLINK_CHANNEL *ch = FindByRcid(G, Rcid);
    KIRQL irql;
    UINT32 i;

    if (ch == NULL) {
        SCM_LOG("GLINK: RX_DONE de canal desconocido rcid=%u\n", (unsigned)Rcid);
        return;
    }
    KeAcquireSpinLock(&G->Lock, &irql);
    for (i = 0; i < GLINK_MAX_RINTENTS; i++) {
        if (ch->R[i].Valid && ch->R[i].Id == Iid) {
            ch->R[i].InUse = FALSE;
            if (!Reuse) {
                ch->R[i].Valid = FALSE;
            }
            break;
        }
    }
    KeReleaseSpinLock(&G->Lock, irql);
    if (i == GLINK_MAX_RINTENTS) {
        SCM_LOG("GLINK: RX_DONE con intent desconocido iid=%u\n", (unsigned)Iid);
    }
}

/* El intent ya se uso: avisar al remoto de que lo puede volver a usar. */
static VOID RxDone(GLINK_CTX *G, GLINK_CHANNEL *Ch, GLINK_LINTENT *In)
{
    UINT32 cmd = In->Reuse ? CMD_RX_DONE_W_REUSE : CMD_RX_DONE;
    UINT32 iid = In->Id;

    if (!In->Reuse) {
        /* se libera antes de avisar: el remoto no puede volver a usarlo */
        if (In->BigSlot >= 0) {
            Ch->BigUsed[In->BigSlot] = FALSE;
        }
        In->Valid = FALSE;
    }
    (VOID)SendMsg(G, cmd, Ch->Lcid, iid);
}

static UINT32 OnData(GLINK_CTX *G, UINT32 Avail)
{
    UINT8 hdr[DATA_HDR_SIZE];
    GLINK_CHANNEL *ch;
    GLINK_LINTENT *in;
    UINT32 chunk, left, rcid, liid;

    if (Avail < DATA_HDR_SIZE) {
        return RX_WAIT;
    }
    RxPeek(G, 0, hdr, DATA_HDR_SIZE);
    rcid  = G16(hdr, 2);
    liid  = G32(hdr, 4);
    chunk = G32(hdr, 8);
    left  = G32(hdr, 12);

    if (chunk > G->RxLen || Avail < DATA_HDR_SIZE + chunk) {
        if (chunk > G->RxLen) {
            SCM_LOG("GLINK: chunk imposible (%u), FIFO corrupto\n",
                    (unsigned)chunk);
            return RX_FAULT;
        }
        return RX_WAIT;
    }

    ch = FindByRcid(G, rcid);
    if (ch == NULL) {
        SCM_LOG("GLINK: datos en canal inexistente rcid=%u, descartados\n",
                (unsigned)rcid);
        goto advance;
    }
    in = FindLIntent(ch, liid);
    if (in == NULL) {
        SCM_LOG("GLINK: sin intent %u en '%s', descartado\n", (unsigned)liid,
                ch->Name);
        goto advance;
    }
    if (in->Size - in->Offset < chunk) {
        SCM_LOG("GLINK: no cabe el chunk (%u) en el intent %u de '%s'\n",
                (unsigned)chunk, (unsigned)liid, ch->Name);
        in->Offset = 0;
        goto advance;
    }
    RxPeek(G, DATA_HDR_SIZE, in->Data + in->Offset, chunk);
    in->Offset += chunk;

    if (left == 0) {
        UINT32 len = in->Offset;

        in->Offset = 0;
        G->CbThread = KeGetCurrentThread();
        ch->Cb(ch->CbCtx, in->Data, len);
        G->CbThread = NULL;
        RxDone(G, ch, in);
    }
advance:
    RxAdvance(G, ALIGN8(DATA_HDR_SIZE + chunk));
    return RX_OK;
}

static UINT32 OnIntent(GLINK_CTX *G, UINT32 Rcid, UINT32 Count, UINT32 Avail)
{
    GLINK_CHANNEL *ch;
    UINT32 msgLen;
    UINT32 i, j;
    UINT8 buf[MSG_SIZE + 8 * GLINK_MAX_RINTENTS];

    if (Count > GLINK_MAX_RINTENTS) {
        SCM_LOG("GLINK: INTENT con %u entradas, FIFO corrupto\n", (unsigned)Count);
        return RX_FAULT;
    }
    msgLen = MSG_SIZE + 8 * Count;
    if (Avail < msgLen) {
        return RX_WAIT;
    }
    RxPeek(G, 0, buf, msgLen);

    ch = FindByRcid(G, Rcid);
    if (ch == NULL) {
        SCM_LOG("GLINK: INTENT para canal inexistente rcid=%u\n", (unsigned)Rcid);
    } else {
        KIRQL irql;

        KeAcquireSpinLock(&G->Lock, &irql);
        for (i = 0; i < Count; i++) {
            UINT32 size = G32(buf, MSG_SIZE + 8 * i);
            UINT32 iid  = G32(buf, MSG_SIZE + 8 * i + 4);
            BOOLEAN stored = FALSE;

            for (j = 0; j < GLINK_MAX_RINTENTS; j++) {
                if (ch->R[j].Valid && ch->R[j].Id == iid) {
                    stored = TRUE;            /* ya estaba */
                    break;
                }
            }
            for (j = 0; !stored && j < GLINK_MAX_RINTENTS; j++) {
                if (!ch->R[j].Valid) {
                    ch->R[j].Id = iid;
                    ch->R[j].Size = size;
                    ch->R[j].InUse = FALSE;
                    ch->R[j].Valid = TRUE;
                    stored = TRUE;
                }
            }
            if (!stored) {
                SCM_LOG("GLINK: tabla de intents remotos llena (iid=%u)\n",
                        (unsigned)iid);
            }
        }
        KeReleaseSpinLock(&G->Lock, irql);
    }
    RxAdvance(G, ALIGN8(msgLen));
    return RX_OK;
}

static UINT32 OnOpenCmd(GLINK_CTX *G, UINT32 Rcid, UINT32 NameLen, UINT32 Avail)
{
    char name[GLINK_NAME_SIZE];
    UINT8 raw[GLINK_NAME_SIZE];
    UINT32 total, n;

    if (NameLen == 0 || NameLen > MAX_OPEN_NAME) {
        SCM_LOG("GLINK: OPEN con nombre de %u bytes, FIFO corrupto\n",
                (unsigned)NameLen);
        return RX_FAULT;
    }
    total = MSG_SIZE + ALIGN8(NameLen);
    if (Avail < total) {
        return RX_WAIT;
    }
    n = NameLen < GLINK_NAME_SIZE ? NameLen : GLINK_NAME_SIZE;
    RtlZeroMemory(raw, sizeof(raw));
    RxPeek(G, MSG_SIZE, raw, n);
    RtlCopyMemory(name, raw, GLINK_NAME_SIZE);
    name[GLINK_NAME_SIZE - 1] = 0;
    RxAdvance(G, total);
    OnRemoteOpen(G, Rcid, name);
    return RX_OK;
}

/* OPEN que llegaron antes de registrar el canal. */
static VOID StartPending(GLINK_CTX *G)
{
    UINT32 i;

    for (i = 0; i < GLINK_MAX_PENDING; i++) {
        GLINK_PENDING *p = &G->Pending[i];
        GLINK_CHANNEL *ch;

        if (!p->Used) {
            continue;
        }
        ch = FindByName(G, p->Name);
        if (ch != NULL && ch->State == GL_CH_REGISTERED) {
            p->Used = FALSE;
            StartOpen(G, ch, p->Rcid);
        }
    }
}

NTSTATUS GlinkPoll(GLINK_CTX *G)
{
    UINT32 n = 0;

    if (G == NULL || !G->Active) {
        return STATUS_DEVICE_NOT_READY;
    }
    if (InterlockedCompareExchange(&G->RxBusy, 1, 0) != 0) {
        return STATUS_DEVICE_BUSY;
    }
    if (G->RxFault) {
        InterlockedExchange(&G->RxBusy, 0);
        return STATUS_DATA_ERROR;
    }

    StartPending(G);

    while (n++ < MAX_MSGS_PER_POLL) {
        UINT8 m[MSG_SIZE];
        UINT32 avail = RxAvail(G);
        UINT32 cmd, p1, p2, rc = RX_OK;

        if (avail < MSG_SIZE) {
            break;
        }
        RxPeek(G, 0, m, MSG_SIZE);
        cmd = G16(m, 0);
        p1  = G16(m, 2);
        p2  = G32(m, 4);
        G->RxMsgs++;

        switch (cmd) {
        case CMD_VERSION:
            RxAdvance(G, MSG_SIZE);
            OnVersion(G, p1, p2);
            break;
        case CMD_VERSION_ACK:
            RxAdvance(G, MSG_SIZE);
            OnVersionAck(G, p1, p2);
            break;
        case CMD_OPEN:
            rc = OnOpenCmd(G, p1, p2, avail);
            break;
        case CMD_OPEN_ACK:
            RxAdvance(G, MSG_SIZE);
            OnOpenAck(G, p1);
            break;
        case CMD_CLOSE:
            RxAdvance(G, MSG_SIZE);
            OnRemoteClose(G, p1);
            break;
        case CMD_CLOSE_ACK:
            RxAdvance(G, MSG_SIZE);
            OnCloseAck(G, p1);
            break;
        case CMD_TX_DATA:
        case CMD_TX_DATA_CONT:
            rc = OnData(G, avail);
            break;
        case CMD_READ_NOTIF:
            RxAdvance(G, MSG_SIZE);
            Kick(G);                     /* el remoto quiere saber que leimos */
            break;
        case CMD_INTENT:
            rc = OnIntent(G, p1, p2, avail);
            break;
        case CMD_RX_DONE:
            RxAdvance(G, MSG_SIZE);
            OnRxDone(G, p1, p2, FALSE);
            break;
        case CMD_RX_DONE_W_REUSE:
            RxAdvance(G, MSG_SIZE);
            OnRxDone(G, p1, p2, TRUE);
            break;
        case CMD_RX_INTENT_REQ:
            RxAdvance(G, MSG_SIZE);
            OnIntentReq(G, p1, p2);
            break;
        case CMD_RX_INTENT_REQ_ACK:
            RxAdvance(G, MSG_SIZE);
            OnIntentReqAck(G, p1, p2);
            break;
        case CMD_SIGNALS: {
            GLINK_CHANNEL *ch = FindByRcid(G, p1);

            RxAdvance(G, MSG_SIZE);
            if (ch != NULL) {
                ch->Rsigs = p2;
            }
            break;
        }
        default:
            SCM_LOG("GLINK: comando rx desconocido %u (p1=%u p2=%u), "
                    "se detiene el rx\n", (unsigned)cmd, (unsigned)p1,
                    (unsigned)p2);
            rc = RX_FAULT;
            break;
        }
        if (rc == RX_FAULT) {
            G->RxFault = TRUE;
            InterlockedExchange(&G->RxBusy, 0);
            return STATUS_DATA_ERROR;
        }
        if (rc == RX_WAIT) {
            G->RxMsgs--;                 /* se reintentara completo */
            break;
        }
    }
    InterlockedExchange(&G->RxBusy, 0);
    return STATUS_SUCCESS;
}

/* ---- envio ---------------------------------------------------------------- */

/* Elige el intent remoto libre mas pequeno que quepa (como Linux). */
static GLINK_RINTENT *TakeRIntent(GLINK_CTX *G, GLINK_CHANNEL *Ch, UINT32 Len)
{
    GLINK_RINTENT *best = NULL;
    KIRQL irql;
    UINT32 i;

    KeAcquireSpinLock(&G->Lock, &irql);
    for (i = 0; i < GLINK_MAX_RINTENTS; i++) {
        GLINK_RINTENT *r = &Ch->R[i];

        if (r->Valid && !r->InUse && r->Size >= Len) {
            if (best == NULL || best->Size > r->Size) {
                best = r;
            }
            if (best->Size == Len) {
                break;
            }
        }
    }
    if (best != NULL) {
        best->InUse = TRUE;
    }
    KeReleaseSpinLock(&G->Lock, irql);
    return best;
}

static VOID ReleaseRIntent(GLINK_CTX *G, GLINK_RINTENT *R)
{
    KIRQL irql;

    KeAcquireSpinLock(&G->Lock, &irql);
    R->InUse = FALSE;
    KeReleaseSpinLock(&G->Lock, irql);
}

NTSTATUS GlinkSend(GLINK_CTX *G, GLINK_CHANNEL *Ch, const VOID *Data,
                   UINT32 Len, ULONG TimeoutMs)
{
    GLINK_RINTENT *ri;
    const UINT8 *src = (const UINT8 *)Data;
    UINT32 iid, cmd = CMD_TX_DATA, left;
    NTSTATUS st;
    GL_WAIT w;

    if (G == NULL || Ch == NULL || Data == NULL || Len == 0) {
        return STATUS_INVALID_PARAMETER;
    }
    if (!G->Active || Ch->State != GL_CH_OPEN) {
        return STATUS_DEVICE_NOT_READY;
    }

    WaitStart(&w, TimeoutMs);
    for (;;) {
        ri = TakeRIntent(G, Ch, Len);
        if (ri != NULL) {
            break;
        }
        if (G->CbThread == KeGetCurrentThread()) {
            return STATUS_DEVICE_BUSY;      /* dentro de un callback: no espera */
        }
        if (WaitExpired(&w)) {
            return STATUS_IO_TIMEOUT;
        }

        /* pedir un intent y esperar a que llegue */
        Ch->IntentReqDone = 0;
        KeMemoryBarrier();
        SCM_LOG("GLINK: -> RX_INTENT_REQ '%s' size=%u\n", Ch->Name, (unsigned)Len);
        st = SendMsg(G, CMD_RX_INTENT_REQ, Ch->Lcid, Len);
        if (!NT_SUCCESS(st)) {
            return st;
        }
        while (Ch->IntentReqDone == 0) {
            (VOID)GlinkPoll(G);
            if (Ch->IntentReqDone != 0) {
                break;
            }
            if (!G->Active || Ch->State != GL_CH_OPEN) {
                return STATUS_DEVICE_NOT_READY;
            }
            if (WaitExpired(&w)) {
                SCM_LOG("GLINK: RX_INTENT_REQ sin respuesta\n");
                return STATUS_IO_TIMEOUT;
            }
            WaitPause(&w);
        }
        if (!Ch->IntentReqGranted) {
            SCM_LOG("GLINK: el remoto nego el intent\n");
            return STATUS_CANCELLED;
        }
    }
    iid = ri->Id;

    left = Len;
    while (left > 0) {
        UINT8 hdr[DATA_HDR_SIZE];
        UINT32 chunk = left > MAX_CHUNK ? MAX_CHUNK : left;

        left -= chunk;
        P16(hdr, 0, cmd);
        P16(hdr, 2, Ch->Lcid);
        P32(hdr, 4, iid);
        P32(hdr, 8, chunk);
        P32(hdr, 12, left);
        st = TxMsg(G, hdr, DATA_HDR_SIZE, src, chunk, CTL_TIMEOUT_MS);
        if (!NT_SUCCESS(st)) {
            ReleaseRIntent(G, ri);
            return st;
        }
        src += chunk;
        cmd = CMD_TX_DATA_CONT;
    }
    return STATUS_SUCCESS;
}

VOID GlinkDump(GLINK_CTX *G)
{
    UINT32 i;

    if (G == NULL || !G->Active) {
        SCM_LOG("GLINK: inactivo\n");
        return;
    }
    {
        PVOID p = NULL;
        SIZE_T sz = 0;
        volatile UINT32 *d = G->TxTail;     /* los 4 indices + relleno */

        SCM_LOG("GLINK: descriptor crudo %08x %08x %08x %08x %08x %08x %08x %08x\n",
                (unsigned)d[0], (unsigned)d[1], (unsigned)d[2], (unsigned)d[3],
                (unsigned)d[4], (unsigned)d[5], (unsigned)d[6], (unsigned)d[7]);
        if (NT_SUCCESS(SmemGet(G->Cfg.Smem, G->Cfg.RemotePid, SMEM_FIFO_RX,
                               &p, &sz)) && p != NULL && sz >= 16) {
            volatile UINT32 *r = (volatile UINT32 *)p;

            SCM_LOG("GLINK: item %u (fifo rx): EXISTE, %u bytes, [0..15]=%08x %08x %08x %08x\n",
                    SMEM_FIFO_RX, (unsigned)sz, (unsigned)r[0], (unsigned)r[1],
                    (unsigned)r[2], (unsigned)r[3]);
        } else {
            SCM_LOG("GLINK: item %u (fifo rx): no existe\n", SMEM_FIFO_RX);
        }
        SmemDumpPartition(G->Cfg.Smem, G->Cfg.RemotePid);
        if (G->TxFifo != NULL) {
            volatile UINT32 *f = (volatile UINT32 *)G->TxFifo;

            SCM_LOG("GLINK: fifo tx[0..15] = %08x %08x %08x %08x\n",
                    (unsigned)f[0], (unsigned)f[1], (unsigned)f[2], (unsigned)f[3]);
        }
    }
    SCM_LOG("GLINK: tx head=%u tail=%u/%u  rx head=%u tail=%u/%u  "
            "msgs rx=%u tx=%u fault=%d\n",
            (unsigned)*G->TxHead, (unsigned)*G->TxTail, (unsigned)G->TxLen,
            (unsigned)*G->RxHead, (unsigned)*G->RxTail, (unsigned)G->RxLen,
            (unsigned)G->RxMsgs, (unsigned)G->TxMsgs, (int)G->RxFault);
    for (i = 0; i < GLINK_MAX_CHANNELS; i++) {
        GLINK_CHANNEL *c = &G->Ch[i];

        if (c->State != GL_CH_UNUSED) {
            SCM_LOG("GLINK: canal '%s' estado=%ld lcid=%u rcid=%u\n", c->Name,
                    (long)c->State, (unsigned)c->Lcid, (unsigned)c->Rcid);
        }
    }
}

/* Vuelve a tocar el timbre (por si el ADSP no habia armado su interrupcion). */
VOID GlinkKick(GLINK_CTX *G)
{
    if (G != NULL && G->Active) {
        Kick(G);
    }
}
