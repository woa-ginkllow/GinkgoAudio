/*
 * glink.h - transporte GLINK nativo sobre SMEM con el ADSP (SM6125).
 *
 * Es una traduccion directa de drivers/rpmsg/qcom_glink_native.c y
 * qcom_glink_smem.c del kernel sm6125 (features = INTENT_REUSE, con intents).
 *
 * Diferencias con Linux:
 *   - Sin interrupcion: se hace polling con GlinkPoll() (un hilo cada 1-2 ms).
 *     Mas adelante el ISR/DPC de la SPI 391 (GSIV 423) puede llamar a la misma
 *     funcion.
 *   - Todo el protocolo (version, OPEN, intents...) se procesa dentro de
 *     GlinkPoll, sin colas diferidas ni hilos propios.
 *   - Solo canales abiertos por el remoto: el ADSP manda OPEN("apr_audio_svc")
 *     y nosotros contestamos (como hace Linux cuando aparece el rpdev). No se
 *     implementa el caso en que el apps abre primero.
 *
 * Recursos para el _CRS (ginkgo.dts, nodo glink-edge del adsp):
 *   SMEM items 478 (descriptor, 32 bytes), 479 (FIFO tx, 16 KiB), 480 (FIFO rx)
 *   mailbox APCS 0x0F111000, registro IPC en +8, bit 8 (mboxes = <&apcs 8>)
 *   interrupcion (todavia sin usar): SPI 391 (0x187), flanco de subida
 *
 * Uso:
 *   GlinkInit(&G, &cfg);                      // despues de err-ready
 *   GlinkRegister(&G, "apr_audio_svc", Cb, Ctx, 512, 20, &Ch);
 *   for (;;) GlinkPoll(&G);                   // hilo
 *   GlinkSend(&G, Ch, buf, len, 1000);
 */
#pragma once

#include "smem.h"

#define GLINK_NAME_SIZE        32
#define GLINK_MAX_CHANNELS     2
#define GLINK_MAX_LINTENTS     32      /* intents locales por canal          */
#define GLINK_MAX_RINTENTS     32      /* intents remotos recordados         */
#define GLINK_MAX_STD_INTENTS  24
#define GLINK_STD_POOL_BYTES   16384   /* 20 x 512 = 10240 cabe              */
#define GLINK_BIG_SLOTS        4       /* intents grandes bajo demanda       */
#define GLINK_BIG_SIZE         8192
#define GLINK_MAX_PENDING      8       /* OPEN remotos aun sin canal         */

#define GLINK_FEATURE_INTENT_REUSE  0x1u

typedef VOID (*GLINK_RX_CB)(PVOID Ctx, const UINT8 *Data, UINT32 Len);

typedef struct _GLINK_LINTENT {
    BOOLEAN Valid;
    BOOLEAN Reuse;
    UINT32  Id;
    UINT32  Size;
    UINT32  Offset;
    UINT8  *Data;
    LONG    BigSlot;               /* -1 si sale del pool estandar */
} GLINK_LINTENT;

typedef struct _GLINK_RINTENT {
    BOOLEAN Valid;
    BOOLEAN InUse;
    UINT32  Id;
    UINT32  Size;
} GLINK_RINTENT;

enum {
    GL_CH_UNUSED = 0,
    GL_CH_REGISTERED,              /* esperando el OPEN del remoto           */
    GL_CH_OPENING,                 /* OPEN_ACK + OPEN enviados, falta su ACK */
    GL_CH_OPEN,
    GL_CH_CLOSING
};

typedef struct _GLINK_CHANNEL {
    volatile LONG State;
    char          Name[GLINK_NAME_SIZE];
    UINT32        Lcid;            /* id local  (0 = ninguno) */
    UINT32        Rcid;            /* id remoto (0 = ninguno) */
    GLINK_RX_CB   Cb;
    PVOID         CbCtx;
    UINT32        StdSize;
    UINT32        StdCount;
    UINT32        NextIid;
    UINT32        Rsigs;
    volatile LONG IntentReqDone;
    BOOLEAN       IntentReqGranted;
    GLINK_LINTENT L[GLINK_MAX_LINTENTS];
    GLINK_RINTENT R[GLINK_MAX_RINTENTS];
    BOOLEAN       BigUsed[GLINK_BIG_SLOTS];
    UINT8         StdPool[GLINK_STD_POOL_BYTES];
    UINT8         BigPool[GLINK_BIG_SLOTS][GLINK_BIG_SIZE];
} GLINK_CHANNEL;

typedef struct _GLINK_CONFIG {
    SMEM_CTX         *Smem;
    UINT32            RemotePid;   /* 2 (ADSP)                     */
    volatile ULONG   *Ipc;         /* registro IPC ya mapeado      */
    UINT32            IpcBit;      /* 8                            */
    BOOLEAN           AdspStopped; /* TRUE: limpiar el descriptor entero    */
    BOOLEAN           DeferVersion;/* TRUE: no mandar VERSION en GlinkInit  */
} GLINK_CONFIG;

typedef struct _GLINK_PENDING {
    BOOLEAN Used;
    UINT32  Rcid;
    char    Name[GLINK_NAME_SIZE];
} GLINK_PENDING;

typedef struct _GLINK_CTX {
    GLINK_CONFIG     Cfg;
    BOOLEAN          Active;
    BOOLEAN          RxFault;
    BOOLEAN          RxIdxBad;
    BOOLEAN          SentReadNotify;
    UINT32           Features;
    UINT32           NextLcid;

    volatile UINT32 *TxTail;
    volatile UINT32 *TxHead;
    volatile UINT32 *RxTail;
    volatile UINT32 *RxHead;
    volatile UINT8  *TxFifo;
    volatile UINT8  *RxFifo;
    UINT32           TxLen;
    UINT32           RxLen;
    UINT32           RxProbe;      /* limita los intentos de buscar el item 480 */

    KSPIN_LOCK       TxLock;
    KSPIN_LOCK       Lock;         /* estado de intents remotos */
    volatile LONG    RxBusy;
    PVOID            CbThread;     /* hilo que esta dentro de un callback */

    GLINK_PENDING    Pending[GLINK_MAX_PENDING];
    GLINK_CHANNEL    Ch[GLINK_MAX_CHANNELS];

    /* contadores para el log */
    UINT32           RxMsgs;
    UINT32           TxMsgs;
} GLINK_CTX;

/* Reserva descriptor y FIFO tx, los inicializa y manda VERSION. */
NTSTATUS GlinkInit(GLINK_CTX *G, const GLINK_CONFIG *Cfg);
VOID     GlinkClose(GLINK_CTX *G);

/*
 * Pone a cero el descriptor compartido (item 478): deja SMEM como en un
 * arranque en frio. SOLO se llama con el ADSP ya detenido (despues de
 * PAS_SHUTDOWN) y antes de cerrar SMEM; si no, un re-arranque del ADSP
 * encontraria los indices del ciclo anterior.
 */
VOID     GlinkResetShared(SMEM_CTX *Smem, UINT32 RemotePid);

/*
 * Anuncia un canal que abrira el remoto. Al abrirse se publican StdCount
 * intents de StdSize bytes (qcom,intents = <0x200 0x14> -> 512 x 20).
 * Cb se llama (dentro de GlinkPoll) con cada mensaje completo; Data solo es
 * valida durante el callback.
 */
NTSTATUS GlinkRegister(GLINK_CTX *G, const char *Name, GLINK_RX_CB Cb,
                       PVOID CbCtx, UINT32 StdSize, UINT32 StdCount,
                       GLINK_CHANNEL **Out);

/* Procesa todo lo que haya en el FIFO rx. Seguro llamarla desde varios hilos. */
NTSTATUS GlinkPoll(GLINK_CTX *G);

BOOLEAN  GlinkIsOpen(const GLINK_CHANNEL *Ch);
NTSTATUS GlinkWaitOpen(GLINK_CTX *G, GLINK_CHANNEL *Ch, ULONG TimeoutMs);

/*
 * Envia un mensaje (fragmenta en trozos de 8 KiB). Si el remoto no tiene un
 * intent libre pide uno (RX_INTENT_REQ) y espera hasta TimeoutMs. Desde dentro
 * de un callback no espera: devuelve STATUS_DEVICE_BUSY. El llamador debe
 * serializar sus envios por canal.
 */
NTSTATUS GlinkSend(GLINK_CTX *G, GLINK_CHANNEL *Ch, const VOID *Data,
                   UINT32 Len, ULONG TimeoutMs);

VOID     GlinkDump(GLINK_CTX *G);

/* Toca el timbre del ADSP sin mandar nada nuevo (diagnostico/reintento). */
VOID     GlinkKick(GLINK_CTX *G);

/* Manda VERSION (y toca el timbre). Para usar con Cfg.DeferVersion = TRUE. */
NTSTATUS GlinkLinkUp(GLINK_CTX *G);
