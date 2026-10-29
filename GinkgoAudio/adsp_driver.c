/*
 * adsp_driver.c - driver KMDF que arranca el ADSP del SM6125 (Redmi Note 8).
 *
 * Une las piezas:  scm.c (TrustZone)  ->  pil_adsp.c (carga del firmware)
 *                  smem.c + smp2p.c    ->  deteccion de err-ready
 *
 * Flujo en EvtDeviceD0Entry:
 *   1. SmemInit      (SMEM 0x46000000 + hwlock 3 del TCSR mutex)
 *   2. Smp2pOpen     (item saliente 0x1AD, aviso por mailbox)  <- ANTES del ADSP
 *   3. PilBootAdsp   (adsp.mdt + adsp.bNN, INIT_IMAGE / MEM_SETUP / AUTH_AND_RESET)
 *   4. Smp2pWaitReady(err-ready de "slave-kernel")
 *   5. GLINK sobre SMEM (glink.c): se registra "apr_audio_svc" y un hilo hace
 *      polling; cuando el ADSP abre el canal se manda un primer comando APR
 *      (AVCS_CMD_GET_FWK_VERSION al servicio CORE) y se vuelca la respuesta.
 *
 * En EvtDeviceD0Exit: se para el hilo GLINK, PAS_SHUTDOWN y se cierran SMP2P y SMEM.
 *
 * Firmware: copialo (de la particion "modem" del telefono) a
 *   C:\Windows\System32\drivers\adsp\   -> adsp.mdt, adsp.b00, adsp.b01, ...
 * (ver ADSP_FW_DIR).
 */
#include <ntddk.h>
#include <wdf.h>
#include <ntstrsafe.h>

#include "pil_adsp.h"
#include "smp2p.h"
#include "glink.h"
#include "apr_min.h"
#include <initguid.h>
#include "adsp_audio.h"

/* solo esta en ntifs.h; ntoskrnl la exporta */
NTSYSAPI NTSTATUS NTAPI ZwWaitForSingleObject(HANDLE Handle, BOOLEAN Alertable,
                                              PLARGE_INTEGER Timeout);

/* ---- recursos fijos de ginkgo.dts (tienen que coincidir con el _CRS) ----- */
#define ADSP_SMEM_PA        0x46000000ull    /* smem@46000000                  */
#define ADSP_SMEM_SIZE      0x00200000u
#define ADSP_LOCK_PA        0x00343000ull    /* tcsr-mutex 0x340000 + 3*0x1000 */
#define ADSP_LOCK_SIZE      0x00001000u
#define ADSP_MBOX_PA        0x0F111000ull    /* trinket-apcs-hmss-global       */
#define ADSP_MBOX_SIZE      0x00001000u
#define ADSP_REGION_PA      0x55300000ull    /* pil_adsp_region@55300000       */
#define ADSP_REGION_SIZE    0x02200000ull

/*
 * Cuando se prepara GLINK respecto del arranque del ADSP (para aislar el fallo):
 *   0 = despues de err-ready, todo junto (levanta bien)  <-- usar este
 *   1 = reservar items 478/479 ANTES del arranque (el ADSP no levanto)
 *   2 = todo antes del arranque, con VERSION y timbre (esto colgo al ADSP)
 *   3 = como Android: todo antes del arranque Y el hilo de GLINK ya corriendo
 *       (responde al ADSP desde el primer momento, antes de err-ready)
 */
#define ADSP_GLINK_MODE         4

#define ADSP_TAG                'psDA'
#define ADSP_READY_TIMEOUT_MS   10000

/* GLINK / APR (ginkgo.dts: glink-edge adsp, mboxes = <&apcs 8>) */
#define ADSP_GLINK_IPC_BIT      8
#define ADSP_APR_CHANNEL        "apr_audio_svc"
#define ADSP_APR_INTENT_SIZE    0x200            /* qcom,intents = <0x200 0x14> */
#define ADSP_APR_INTENT_COUNT   0x14
#define ADSP_APR_TRIES          3
#define ADSP_APR_RETRY_100NS    (30ll * 1000 * 1000)   /* 3 s */
#define ADSP_GLINK_NOOPEN_100NS (150ll * 1000 * 1000)  /* 15 s */
#define ADSP_MAX_BLOB           (64ull * 1024 * 1024)

/* En WinPE \SystemRoot es la RAM disk: el log se pierde al reiniciar. */
#define ADSP_FW_DIR   L"\\SystemRoot\\System32\\drivers\\adsp\\"
#define ADSP_LOG_PATH L"\\SystemRoot\\adsp_log.txt"

typedef struct _ADSP_CONTEXT {
    WDFDEVICE   Device;
    SMEM_CTX    Smem;
    SMP2P_CTX   P2p;
    BOOLEAN     SmemOpen;
    BOOLEAN     P2pOpen;
    BOOLEAN     Booted;
    WDFMEMORY   MdtMem;      /* adsp.mdt: tiene que vivir durante todo el boot */
    WDFMEMORY   SegMem;      /* adsp.bNN en curso                              */

    /* GLINK + APR */
    WDFMEMORY       GlinkMem;     /* GLINK_CTX (~100 KB, no paginado)          */
    GLINK_CTX      *Glink;
    GLINK_CHANNEL  *AprCh;
    PKTHREAD        PollThread;
    volatile LONG   PollStop;
    /* motor de reproduccion (IOCTL de adsp_audio.h) */
    volatile LONG   PlayState;    /* ADSPAUDIO_STATE_*                        */
    volatile LONG   ReqOpen, ReqClose;
    volatile LONG   AfeFailed;
    NTSTATUS        OpenStatus;
    KEVENT          OpenDone, CloseDone;
    KSPIN_LOCK      RingLock;
    PUCHAR          Ring;
    ULONG           RingHead, RingTail, RingCount;
    ULONG           PlaySent;     /* trozos enviados al ADSP                  */
    ULONG           Underruns;
    BOOLEAN         PlayStarted;  /* ya llego el primer dato: desde aqui se rellena con silencio */
    ULONGLONG       BytesQueued;
    volatile LONG   FwkUp;        /* llego AVCS_CMDRSP_GET_FWK_VERSION (0x1292d) */
    ULONGLONG       AfeRetryAt;
    ULONG           AfeRetries;
    UINT32          PcmBytes;     /* tamano mapeado con el ADSP                */
    BOOLEAN         PcmInSmem;
    PVOID           PcmVa;        /* buffer PCM contiguo, sin cache            */
    ULONGLONG       PcmPa;
    volatile LONG   MemHandle;    /* handle del ASM_CMD_SHARED_MEM_MAP_REGIONS */
    volatile LONG   CoppId;       /* de ADM_CMDRSP_DEVICE_OPEN_V5              */
    volatile LONG   WrDone;       /* ASM_DATA_EVENT_WRITE_DONE_V2 recibidos    */
    volatile LONG   WrBadStatus;
    ULONGLONG       AfeToneAt;    /* 100 ns: cuando lanzar el tono            */
    volatile LONG   AprRsp;       /* APR_BASIC_RSP_RESULT recibidos             */
    volatile LONG   AprRspStatus; /* status del ultimo                         */
    volatile LONG   AprRspOp;     /* opcode al que responde                    */
    volatile LONG   AprRx;        /* paquetes APR recibidos                    */
} ADSP_CONTEXT, *PADSP_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(ADSP_CONTEXT, AdspGetContext)

DRIVER_INITIALIZE DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD       AdspEvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE AdspEvtPrepareHardware;
EVT_WDF_DEVICE_D0_ENTRY         AdspEvtD0Entry;
EVT_WDF_DEVICE_D0_EXIT          AdspEvtD0Exit;
EVT_WDF_IO_QUEUE_IO_DEVICE_CONTROL AdspEvtIoDeviceControl;

/* ---- log: DbgPrint + archivo -------------------------------------------- */

VOID AdspLog(const char *Fmt, ...)
{
    char buf[256];
    va_list ap;
    NTSTATUS st;
    size_t len = 0;

    va_start(ap, Fmt);
    st = RtlStringCbVPrintfA(buf, sizeof(buf), Fmt, ap);
    va_end(ap);
    if (!NT_SUCCESS(st) && st != STATUS_BUFFER_OVERFLOW) {
        return;
    }

    DbgPrint("%s", buf);

    if (KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return;
    }
    if (!NT_SUCCESS(RtlStringCbLengthA(buf, sizeof(buf), &len)) || len == 0) {
        return;
    }

    {
        UNICODE_STRING name;
        OBJECT_ATTRIBUTES oa;
        IO_STATUS_BLOCK iosb;
        HANDLE h = NULL;

        RtlInitUnicodeString(&name, ADSP_LOG_PATH);
        InitializeObjectAttributes(&oa, &name,
                                   OBJ_KERNEL_HANDLE | OBJ_CASE_INSENSITIVE,
                                   NULL, NULL);
        st = ZwCreateFile(&h, FILE_APPEND_DATA | SYNCHRONIZE, &oa, &iosb, NULL,
                          FILE_ATTRIBUTE_NORMAL, FILE_SHARE_READ, FILE_OPEN_IF,
                          FILE_SYNCHRONOUS_IO_NONALERT | FILE_NON_DIRECTORY_FILE,
                          NULL, 0);
        if (NT_SUCCESS(st)) {
            ZwWriteFile(h, NULL, NULL, NULL, &iosb, buf, (ULONG)len, NULL, NULL);
            ZwClose(h);
        }
    }
}

/* ---- lectura del firmware ----------------------------------------------- */

static NTSTATUS ReadFileToMemory(PCWSTR Path, WDFOBJECT Parent,
                                 WDFMEMORY *Mem, PVOID *Data, SIZE_T *Size)
{
    UNICODE_STRING name;
    OBJECT_ATTRIBUTES oa;
    IO_STATUS_BLOCK iosb;
    FILE_STANDARD_INFORMATION fsi;
    WDF_OBJECT_ATTRIBUTES attrs;
    HANDLE h = NULL;
    WDFMEMORY mem = NULL;
    PVOID buf = NULL;
    NTSTATUS st;

    RtlInitUnicodeString(&name, Path);
    InitializeObjectAttributes(&oa, &name,
                               OBJ_KERNEL_HANDLE | OBJ_CASE_INSENSITIVE,
                               NULL, NULL);

    st = ZwCreateFile(&h, GENERIC_READ | SYNCHRONIZE, &oa, &iosb, NULL,
                      FILE_ATTRIBUTE_NORMAL, FILE_SHARE_READ, FILE_OPEN,
                      FILE_SYNCHRONOUS_IO_NONALERT | FILE_NON_DIRECTORY_FILE,
                      NULL, 0);
    if (!NT_SUCCESS(st)) {
        AdspLog("ADSP: no se pudo abrir %ws nt=0x%08x\n", Path, (unsigned)st);
        return st;
    }

    st = ZwQueryInformationFile(h, &iosb, &fsi, sizeof(fsi),
                                FileStandardInformation);
    if (!NT_SUCCESS(st)) {
        goto done;
    }
    if (fsi.EndOfFile.QuadPart <= 0 ||
        (ULONGLONG)fsi.EndOfFile.QuadPart > ADSP_MAX_BLOB) {
        AdspLog("ADSP: tamano invalido (%lld) en %ws\n",
                (long long)fsi.EndOfFile.QuadPart, Path);
        st = STATUS_INVALID_IMAGE_FORMAT;
        goto done;
    }

    WDF_OBJECT_ATTRIBUTES_INIT(&attrs);
    attrs.ParentObject = Parent;
    st = WdfMemoryCreate(&attrs, NonPagedPoolNx, ADSP_TAG,
                         (size_t)fsi.EndOfFile.QuadPart, &mem, &buf);
    if (!NT_SUCCESS(st)) {
        goto done;
    }

    st = ZwReadFile(h, NULL, NULL, NULL, &iosb, buf,
                    (ULONG)fsi.EndOfFile.QuadPart, NULL, NULL);
    if (NT_SUCCESS(st) &&
        iosb.Information != (ULONG_PTR)fsi.EndOfFile.QuadPart) {
        st = STATUS_END_OF_FILE;
    }
    if (!NT_SUCCESS(st)) {
        AdspLog("ADSP: fallo la lectura de %ws nt=0x%08x\n", Path, (unsigned)st);
        WdfObjectDelete(mem);
        mem = NULL;
        goto done;
    }

    *Mem = mem;
    *Data = buf;
    *Size = (SIZE_T)fsi.EndOfFile.QuadPart;

done:
    ZwClose(h);
    return st;
}

static VOID FreeBlob(WDFMEMORY *Slot)
{
    if (*Slot != NULL) {
        WdfObjectDelete(*Slot);
        *Slot = NULL;
    }
}

/* callback que usa PilBootAdsp(): Index == PIL_BLOB_MDT -> adsp.mdt, si no adsp.bNN */
static NTSTATUS AdspGetBlob(PVOID Ctx, UINT32 Index,
                            const VOID **Data, SIZE_T *Size)
{
    PADSP_CONTEXT c = (PADSP_CONTEXT)Ctx;
    WCHAR path[160];
    WDFMEMORY *slot;
    PVOID p = NULL;
    NTSTATUS st;

    if (Index == PIL_BLOB_MDT) {
        slot = &c->MdtMem;
        st = RtlStringCbCopyW(path, sizeof(path), ADSP_FW_DIR L"adsp.mdt");
    } else {
        slot = &c->SegMem;
        st = RtlStringCbPrintfW(path, sizeof(path), ADSP_FW_DIR L"adsp.b%02u",
                                (unsigned)Index);
    }
    if (!NT_SUCCESS(st)) {
        return st;
    }

    FreeBlob(slot);          /* el segmento anterior ya se copio a la region */
    st = ReadFileToMemory(path, (WDFOBJECT)c->Device, slot, &p, Size);
    if (NT_SUCCESS(st)) {
        *Data = p;
    }
    return st;
}

/* ---- proxy vote ---------------------------------------------------------- */

/*
 * En Linux, pil-tz vota el reloj xo y el riel vdd_cx antes de arrancar el ADSP
 * para que no se apaguen hasta que el ADSP haga sus propios votos. En Windows
 * no hay un cliente de RPM que los apague, asi que se ASUME que siguen como los
 * dejo el firmware. Si el ADSP no arranca, este es el primer lugar donde mirar.
 */
static NTSTATUS AdspProxyVote(PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    AdspLog("ADSP: proxy vote omitido (se asume xo y vdd_cx encendidos)\n");
    return STATUS_SUCCESS;
}


/* ---- GLINK + APR ---------------------------------------------------------- */

/* Vuelca un paquete APR: cabecera decodificada y primeros dwords en crudo. */
static VOID AdspAprLog(const UINT8 *Data, UINT32 Len)
{
    UINT32 i, n = Len < 64 ? Len : 64;

    AdspLog("APR: rx %u bytes\n", (unsigned)Len);
    if (Len >= APR_HDR_SIZE) {
        APR_HDR h;

        RtlCopyMemory(&h, Data, sizeof(h));
        AdspLog("APR:   tipo=%u ver=%u hdr=%u pkt_size=%u\n",
                (unsigned)((h.HdrField >> 8) & 3), (unsigned)(h.HdrField & 0xF),
                (unsigned)(((h.HdrField >> 4) & 0xF) * 4), (unsigned)h.PktSize);
        AdspLog("APR:   src svc=0x%x dom=0x%x port=0x%x  dest svc=0x%x dom=0x%x port=0x%x\n",
                (unsigned)h.SrcSvc, (unsigned)h.SrcDomain, (unsigned)h.SrcPort,
                (unsigned)h.DestSvc, (unsigned)h.DestDomain, (unsigned)h.DestPort);
        AdspLog("APR:   token=0x%08x opcode=0x%08x\n",
                (unsigned)h.Token, (unsigned)h.Opcode);
    }
    for (i = 0; i + 4 <= n; i += 16) {
        UINT32 w[4] = { 0, 0, 0, 0 };
        UINT32 k;

        for (k = 0; k < 4 && i + 4 * k + 4 <= n; k++) {
            RtlCopyMemory(&w[k], Data + i + 4 * k, 4);
        }
        AdspLog("APR:   +%02x: %08x %08x %08x %08x\n", (unsigned)i,
                (unsigned)w[0], (unsigned)w[1], (unsigned)w[2], (unsigned)w[3]);
    }
}

/* Se llama desde GlinkPoll (hilo de polling): Data solo vale durante la llamada. */
static VOID AdspAprRx(PVOID Ctx, const UINT8 *Data, UINT32 Len)
{
    PADSP_CONTEXT c = (PADSP_CONTEXT)Ctx;
    APR_HDR h;
    UINT32 hl, w[4];

    InterlockedIncrement(&c->AprRx);
    if (Len < APR_HDR_SIZE) { AdspAprLog(Data, Len); return; }
    RtlCopyMemory(&h, Data, sizeof(h));
    hl = ((h.HdrField >> 4) & 0xF) * 4;          /* el ADSP declara 32 bytes */
    if (hl < APR_HDR_SIZE) hl = APR_HDR_SIZE;

    if (h.Opcode == ASM_DATA_EVENT_WRITE_DONE_V2 && Len >= hl + 16) {
        RtlCopyMemory(w, Data + hl, 16);          /* addr lsw/msw, handle, status */
        if (w[3] != 0) {
            InterlockedIncrement(&c->WrBadStatus);
            AdspAprLog(Data, Len);
        }
        InterlockedIncrement(&c->WrDone);
        return;
    }
    AdspAprLog(Data, Len);
    if (h.Opcode == AVCS_CMDRSP_GET_FWK_VERSION) {
        InterlockedExchange(&c->FwkUp, 1);
    }
    if (h.Opcode == APR_BASIC_RSP_RESULT && Len >= hl + 8) {
        RtlCopyMemory(w, Data + hl, 8);
        InterlockedExchange(&c->AprRspOp, (LONG)w[0]);
        InterlockedExchange(&c->AprRspStatus, (LONG)w[1]);
        InterlockedIncrement(&c->AprRsp);
    } else if (h.Opcode == ASM_CMDRSP_SHARED_MEM_MAP_REGIONS && Len >= hl + 4) {
        RtlCopyMemory(w, Data + hl, 4);
        InterlockedExchange(&c->MemHandle, (LONG)w[0]);
        InterlockedExchange(&c->AprRspOp, (LONG)h.Opcode);
        InterlockedExchange(&c->AprRspStatus, 0);
        InterlockedIncrement(&c->AprRsp);
    } else if (h.Opcode == ADM_CMDRSP_DEVICE_OPEN_V5 && Len >= hl + 8) {
        RtlCopyMemory(w, Data + hl, 8);           /* status, copp_id(u16) */
        InterlockedExchange(&c->CoppId, (LONG)(w[1] & 0xFFFF));
        InterlockedExchange(&c->AprRspOp, (LONG)h.Opcode);
        InterlockedExchange(&c->AprRspStatus, (LONG)w[0]);
        InterlockedIncrement(&c->AprRsp);
    }
}

static VOID AdspAprSendGetVersion(PADSP_CONTEXT c)
{
    UINT8 b[64];
    UINT32 n;
    NTSTATUS st;

    n = AprBuildPkt(b, APR_SVC_ADSP_CORE, NULL, 0, AVCS_CMD_GET_FWK_VERSION, 0x1234);
    AdspLog("APR: tx AVCS_CMD_GET_FWK_VERSION al servicio CORE (%u bytes)\n", (unsigned)n);
    st = GlinkSend(c->Glink, c->AprCh, b, n, 2000);
    AdspLog("APR: GlinkSend nt=0x%08x\n", (unsigned)st);
}

/* ---- AFE: reloj + I2S + arranque del puerto PRI_MI2S RX --------------------
 * Lineas SD del DT (qcom,msm-mi2s-rx-lines): ajustar si no es SD0. */
#define ADSP_AFE_SD_MODE        AFE_PORT_I2S_SD0

static NTSTATUS AdspAprSendSvc(PADSP_CONTEXT c, UINT8 Svc, UINT16 SrcPort, UINT16 DestPort,
                               UINT32 Token, UINT32 Opcode, const VOID *Pl, UINT32 Len, BOOLEAN Quiet)
{
    UINT8 buf[160];
    UINT32 n;

    if (32 + Len > sizeof(buf)) return STATUS_BUFFER_OVERFLOW;
    n = AprBuildPkt(buf, Svc, Pl, Len, Opcode, Token);
    RtlCopyMemory(buf + 6, &SrcPort, 2);          /* src_port  */
    RtlCopyMemory(buf + 10, &DestPort, 2);        /* dest_port */
    if (!Quiet) {
        AdspLog("APR: tx svc %u opcode 0x%08x len %u (puertos 0x%x/0x%x)\n", (unsigned)Svc,
                (unsigned)Opcode, (unsigned)n, (unsigned)SrcPort, (unsigned)DestPort);
    }
    return GlinkSend(c->Glink, c->AprCh, buf, n, 2000);
}

static NTSTATUS AdspAprSendRaw(PADSP_CONTEXT c, UINT32 Opcode, const VOID *Pl, UINT32 Len)
{
    return AdspAprSendSvc(c, APR_SVC_AFE, 0, 0, 0xAF00 | (Opcode & 0xFF), Opcode, Pl, Len, FALSE);
}

static NTSTATUS AdspAfeSetParam(PADSP_CONTEXT c, UINT32 Mod, UINT32 Param,
                                const VOID *Data, UINT32 DataLen)
{
    UINT8 pl[64];
    AFE_SET_PARAM_HDR hd = { 0 };

    if (Mod == AFE_MODULE_CLOCK_SET) {
        /* servicio: u32 payload_size, mem_hdr(12), param_hdr_v1(12), datos */
        UINT32 sz = 12 + DataLen, z = 0, w;
        UINT8 *q = pl;
        RtlCopyMemory(q, &sz, 4); q += 4;
        RtlZeroMemory(q, 12); q += 12;
        RtlCopyMemory(q, &Mod, 4); q += 4;
        RtlCopyMemory(q, &Param, 4); q += 4;
        w = DataLen;                       /* u16 size + u16 reservado */
        RtlCopyMemory(q, &w, 4); q += 4;
        RtlCopyMemory(q, Data, DataLen); q += DataLen;
        (VOID)z;
        return AdspAprSendRaw(c, AFE_SVC_CMD_SET_PARAM, pl, (UINT32)(q - pl));
    }
    hd.PortId = (UINT16)AFE_PORT_ID_PRIMARY_MI2S_RX;
    hd.PayloadSize = (UINT16)(sizeof(UINT32) * 2 + 4 + DataLen);  /* param_hdr_v1 = 12 */
    hd.ModuleId = Mod;
    hd.ParamId = Param;
    hd.ParamSize = (UINT16)DataLen;
    RtlCopyMemory(pl, &hd, sizeof(hd));
    RtlCopyMemory(pl + sizeof(hd), Data, DataLen);
    return AdspAprSendRaw(c, AFE_PORT_CMD_SET_PARAM_V2, pl, (UINT32)sizeof(hd) + DataLen);
}

/* Maquina de estados no bloqueante (la llama el hilo de GLINK). */
static VOID AdspAfeStep(PADSP_CONTEXT c, LONG *Step, LONG *WaitRsp)
{
    NTSTATUS st = STATUS_SUCCESS;

    if (*Step >= 99) { if (*Step == 99) { AdspLog("AFE: fallo, abortado\n"); InterlockedExchange(&c->AfeFailed, 1); *Step = 100; } return; }
    if (*WaitRsp >= 0) {
        if (c->AprRsp <= *WaitRsp) return;               /* aun sin respuesta */
        AdspLog("AFE: paso %d -> rsp op=0x%08x status=0x%x\n", (int)*Step,
                (unsigned)c->AprRspOp, (unsigned)c->AprRspStatus);
        if (c->AprRspStatus == 0x16 && c->AfeRetries < 40) {
            /* 0x16: el ADSP aun no tiene listo el servicio (tambien lo devolvio al
             * GET_FWK_VERSION en el arranque): se repite el mismo paso */
            c->AfeRetries++;
            c->AfeRetryAt = KeQueryInterruptTime() + 5ll * 1000 * 1000;   /* 0,5 s */
            AdspLog("AFE: status 0x16 (servicio no listo), reintento %u\n", (unsigned)c->AfeRetries);
            *WaitRsp = -1;
            return;
        }
        if (c->AprRspStatus != 0) { *Step = 99; return; }
        *WaitRsp = -1;
        (*Step)++;
        return;
    }
    if (KeQueryInterruptTime() < c->AfeRetryAt) return;
    switch (*Step) {
    case 0: {
        AFE_CLK_SET k = { 1, Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT,
                          1536000, Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_NO, 0, 1 };
        st = AdspAfeSetParam(c, AFE_MODULE_CLOCK_SET, AFE_PARAM_ID_CLOCK_SET, &k, sizeof(k));
        break; }
    case 1: {
        AFE_I2S_CFG i = { 1, 16, ADSP_AFE_SD_MODE, AFE_PORT_I2S_STEREO,
                          AFE_PORT_CONFIG_I2S_WS_SRC_INTERNAL, 48000,
                          AFE_LINEAR_PCM_DATA, 0 };
        st = AdspAfeSetParam(c, AFE_MODULE_AUDIO_DEV_INTERFACE, AFE_PARAM_ID_I2S_CONFIG, &i, sizeof(i));
        break; }
    case 2: {
        AFE_DEVICE_START d = { (UINT16)AFE_PORT_ID_PRIMARY_MI2S_RX, 0 };
        st = AdspAprSendRaw(c, AFE_PORT_CMD_DEVICE_START, &d, sizeof(d));
        break; }
#if !ADSP_AUTODEMO
    case 3:
        AdspLog("AFE: puerto PRI_MI2S RX arrancado; listo para IOCTL_ADSPAUDIO_OPEN\n");
        *Step = 100;
        return;
#else
    case 3: {
        AFE_DTMF t = { ADSP_TONE_MS, ADSP_TONE_FREQ, ADSP_TONE_FREQ, ADSP_TONE_GAIN,
                       1, (UINT16)AFE_PORT_ID_PRIMARY_MI2S_RX, 0 };
        /* dar tiempo a que ampi2c encienda el amplificador (espera 20 s) */
        if (KeQueryInterruptTime() < c->AfeToneAt) return;
        AdspLog("AFE: puerto PRI_MI2S RX arrancado; tono %d Hz %d ms\n",
                (int)ADSP_TONE_FREQ, (int)ADSP_TONE_MS);
        st = AdspAprSendRaw(c, AFE_PORTS_CMD_DTMF_CTL, &t, sizeof(t));
        break; }
#endif
    case 4:
        AdspLog("AFE: tono enviado\n");
        *Step = 100;
        return;
    default:
        if (*Step == 99) { AdspLog("AFE: fallo, abortado\n"); *Step = 100; }
        return;
    }
    AdspLog("AFE: paso %d GlinkSend nt=0x%08x\n", (int)*Step, (unsigned)st);
    if (!NT_SUCCESS(st)) { *Step = 99; return; }
    *WaitRsp = c->AprRsp;
}

/*
 * Hasta que paso llega la prueba de PCM (un BSOD pierde el log: se prueba por
 * etapas, subiendo de a una):
 *   9  = solo reservar el buffer en Windows (sin tocar el ADSP)
 *   10 = + mapa de memoria compartida
 *   11 = + ASM open write      12 = + formato PCM
 *   13 = + ADM open (COPP)     14 = + matriz
 *   15 = + RUN                 16 = + escrituras (suena)   17 = + close
 */
#define ADSP_ASM_LAST_STEP      17

/* 1 = al arrancar suena el DTMF y un seno por ASM (prueba); 0 = el ADSP queda
 * listo y solo se abre el stream cuando alguien hace IOCTL_ADSPAUDIO_OPEN */
#define ADSP_AUTODEMO           0

/* Interfaz de IOCTL, por partes (mascara de bits) para descubrir cual rompe el
 * arranque del ADSP:  1 = ACL (SDDL)   2 = cola de IOCTL   4 = interfaz de dispositivo
 * 0 = solo el motor interno y la autoprueba (ADSP_SELFTEST).  7 = todo. */
#define ADSP_AUDIO_IFACE        6

/* 1 = el propio driver prueba el motor de PCM: ADSP_SELFTEST_DELAY_S s despues de
 * que el AFE este listo hace OPEN, toca un seno ADSP_SELFTEST_SECONDS s por el anillo
 * (la misma ruta que los IOCTL), hace CLOSE y lo repite ADSP_SELFTEST_ROUNDS veces
 * (prueba la reapertura). Sirve cuando no se puede ejecutar audiotest.exe. */
#define ADSP_SELFTEST           0
#define ADSP_SELFTEST_DELAY_S   30
#define ADSP_SELFTEST_SECONDS   5
#define ADSP_SELFTEST_ROUNDS    2
#define ADSP_RING_SIZE          65536u

/* ---- reproduccion de PCM: ASM + ADM ----------------------------------------
 * Orden (como audio-kernel): mapa de memoria, open write, formato PCM, ADM
 * open, matriz, run, escrituras con ventana de ADSP_PCM_INFLIGHT, close. */
/*
 * El ADSP leyo la memoria normal de Windows y colgo el SoC (CLOCK_WATCHDOG):
 * solo puede leer lo que Android le da por el SMMU. SMEM si es suyo, asi que el
 * buffer va en un item de la particion privada (el seno de 9600 bytes son 50
 * periodos exactos, asi que 4 trozos se reutilizan en anillo).
 */
#define ADSP_PCM_IN_SMEM        1
#define ADSP_PCM_SMEM_ITEM      600
#define ADSP_PCM_RING           ADSP_PCM_INFLIGHT          /* trozos del anillo */

static BOOLEAN AdspPcmAlloc(PADSP_CONTEXT c)
{
    UINT32 i, frames;
    INT16 *p;

#if ADSP_PCM_IN_SMEM
    {
        NTSTATUS st;
        PVOID sp = NULL;
        SIZE_T sz = 0;

        /* el ADSP rechazo el mapa (EBADPARAM) con una direccion no alineada a 4 KiB:
         * el item lleva 4 KiB de sobra y se alinea la direccion fisica hacia arriba */
        c->PcmBytes = (ADSP_PCM_CHUNK * ADSP_PCM_RING + 0xFFFu) & ~0xFFFu;
        st = SmemAlloc(&c->Smem, 2, ADSP_PCM_SMEM_ITEM, c->PcmBytes + 0x1000);
        AdspLog("ASM: SmemAlloc item %u (%u bytes) nt=0x%08x\n", (unsigned)ADSP_PCM_SMEM_ITEM,
                (unsigned)(c->PcmBytes + 0x1000), (unsigned)st);
        if (!NT_SUCCESS(st) && st != STATUS_OBJECT_NAME_COLLISION) {
            return FALSE;
        }
        st = SmemGet(&c->Smem, 2, ADSP_PCM_SMEM_ITEM, &sp, &sz);
        if (!NT_SUCCESS(st) || sp == NULL || sz < c->PcmBytes + 0x1000) {
            AdspLog("ASM: SmemGet nt=0x%08x sz=%u\n", (unsigned)st, (unsigned)sz);
            return FALSE;
        }
        {
            ULONGLONG pa = ADSP_SMEM_PA + (ULONGLONG)((PUCHAR)sp - (PUCHAR)c->Smem.Base);
            ULONGLONG al = (pa + 0xFFFull) & ~0xFFFull;

            c->PcmVa = (PUCHAR)sp + (al - pa);
            c->PcmPa = al;
        }
        c->PcmInSmem = TRUE;
    }
#else
    {
        PHYSICAL_ADDRESS lo = { 0 }, hi, skip = { 0 };

        hi.QuadPart = 0xFFFFFFFFll;
        c->PcmBytes = ADSP_PCM_ALLOC;
        c->PcmVa = MmAllocateContiguousMemorySpecifyCache(ADSP_PCM_ALLOC, lo, hi, skip, MmNonCached);
        if (c->PcmVa == NULL) return FALSE;
        c->PcmPa = (ULONGLONG)MmGetPhysicalAddress(c->PcmVa).QuadPart;
        RtlZeroMemory(c->PcmVa, ADSP_PCM_ALLOC);
    }
#endif
    frames = (ADSP_PCM_CHUNK * ADSP_PCM_RING) / 4;
    p = (INT16 *)c->PcmVa;
    for (i = 0; i < frames; i++) {
        INT16 v = kSine1k48[i % 48];
        p[2 * i] = v;
        p[2 * i + 1] = v;
    }
    KeMemoryBarrier();
    return TRUE;
}

static VOID AdspAsmFree(PADSP_CONTEXT c)
{
    if (c->PcmVa != NULL && !c->PcmInSmem) {
        MmFreeContiguousMemorySpecifyCache(c->PcmVa, ADSP_PCM_ALLOC, MmNonCached);
        c->PcmVa = NULL;
    }
}

static VOID AdspAsmStep(PADSP_CONTEXT c, LONG *Step, LONG *WaitRsp, ULONGLONG *T0, ULONG *Sent)
{
    NTSTATUS st = STATUS_SUCCESS;
    ULONGLONG now = KeQueryInterruptTime();

#if !ADSP_AUTODEMO
    UNREFERENCED_PARAMETER(Sent);
#endif
    if (*Step >= 90) {
        if (*Step == 90) {
            AdspLog("ASM: fallo, abortado en el paso %d\n", (int)*WaitRsp);
            if (*WaitRsp <= 9 && c->MemHandle == 0) AdspAsmFree(c);   /* mapeado: no se libera */
            *Step = 100;
            c->OpenStatus = STATUS_UNSUCCESSFUL;
            InterlockedExchange(&c->PlayState, ADSPAUDIO_STATE_IDLE);
            KeSetEvent(&c->OpenDone, IO_NO_INCREMENT, FALSE);
        }
        return;
    }
    if (*WaitRsp >= 0) {                          /* esperando respuesta */
        if (c->AprRsp <= *WaitRsp) {
            if (now - *T0 > 100ll * 1000 * 1000) {            /* 10 s */
                AdspLog("ASM: timeout en el paso %d\n", (int)*Step);
                *WaitRsp = *Step;
                *Step = 90;
            }
            return;
        }
        AdspLog("ASM: paso %d -> rsp op=0x%08x status=0x%x\n", (int)*Step,
                (unsigned)c->AprRspOp, (unsigned)c->AprRspStatus);
        if (c->AprRspStatus != 0) { *WaitRsp = *Step; *Step = 90; return; }
        *WaitRsp = -1;
        (*Step)++;
        return;
    }
    if (*Step > ADSP_ASM_LAST_STEP) {
        AdspLog("ASM: ADSP_ASM_LAST_STEP=%d: me detengo aqui (buffer queda mapeado)\n",
                (int)ADSP_ASM_LAST_STEP);
        *Step = 100;
        return;
    }
    switch (*Step) {
    case 9:                                       /* solo Windows: reservar el buffer */
        if (c->PcmVa != NULL) { (*Step)++; return; }          /* ya reservado (reapertura) */
        if (!AdspPcmAlloc(c)) { AdspLog("ASM: sin memoria contigua\n"); *WaitRsp = 9; *Step = 90; return; }
        AdspLog("ASM: buffer PCM PA 0x%llx tamano 0x%x %s\n", c->PcmPa, (unsigned)c->PcmBytes, c->PcmInSmem ? "(en SMEM)" : "(memoria de Windows)");
        (*Step)++;
        return;
    case 10: {                                    /* memoria compartida */
        AVS_MAP_REGIONS m;
        if (c->MemHandle != 0) { (*Step)++; return; }         /* ya mapeada (reapertura) */
        m.PoolId = (UINT16)ADSP_MEMORY_MAP_SHMEM8_4K_POOL; m.NumRegions = 1; m.PropertyFlag = 0;
        m.AddrLsw = (UINT32)c->PcmPa; m.AddrMsw = (UINT32)(c->PcmPa >> 32); m.SizeBytes = c->PcmBytes;
        st = AdspAprSendSvc(c, APR_SVC_ASM, 0, 0, 0, ASM_CMD_SHARED_MEM_MAP_REGIONS, &m, sizeof(m), FALSE);
        break; }
    case 11: {                                    /* open write */
        ASM_OPEN_WRITE_V3 o = { 0, 0, 16, NULL_POPP_TOPOLOGY, ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V2 };
        st = AdspAprSendSvc(c, APR_SVC_ASM, ADSP_ASM_PORT, ADSP_ASM_PORT, ADSP_ASM_SESSION << 8,
                            ASM_STREAM_CMD_OPEN_WRITE_V3, &o, sizeof(o), FALSE);
        break; }
    case 12: {                                    /* formato PCM */
        ASM_PCM_FMT_V2 f = { 20, 2, 16, 48000, 1, 0, { PCM_CHANNEL_FL, PCM_CHANNEL_FR } };
        st = AdspAprSendSvc(c, APR_SVC_ASM, ADSP_ASM_PORT, ADSP_ASM_PORT, ADSP_ASM_SESSION << 8,
                            ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2, &f, sizeof(f), FALSE);
        break; }
    case 13: {                                    /* ADM: COPP hacia el puerto 0x1000 */
        ADM_OPEN_V5 a = { 0, 1, (UINT16)AFE_PORT_ID_PRIMARY_MI2S_RX, 0xFFFF,
                          ADM_TOPOLOGY_NONE_AUDIO_COPP, 2, 16, 48000, { PCM_CHANNEL_FL, PCM_CHANNEL_FR } };
        st = AdspAprSendSvc(c, APR_SVC_ADM, (UINT16)AFE_PORT_ID_PRIMARY_MI2S_RX, 0, 0,
                            ADM_CMD_DEVICE_OPEN_V5, &a, sizeof(a), FALSE);
        break; }
    case 14: {                                    /* matriz: sesion -> COPP */
        ADM_MATRIX_MAP_V5 r = { ADM_MATRIX_ID_AUDIO_RX, 1, ADSP_ASM_SESSION, 1,
                                (UINT16)c->CoppId, 0 };
        AdspLog("ASM: COPP id %d\n", (int)c->CoppId);
        st = AdspAprSendSvc(c, APR_SVC_ADM, 0, 0, 0, ADM_CMD_MATRIX_MAP_ROUTINGS_V5, &r, sizeof(r), FALSE);
        break; }
    case 15: {                                    /* run */
        ASM_RUN_V2 r = { 0, 0, 0 };
        st = AdspAprSendSvc(c, APR_SVC_ASM, ADSP_ASM_PORT, ADSP_ASM_PORT, ADSP_ASM_SESSION << 8,
                            ASM_SESSION_CMD_RUN_V2, &r, sizeof(r), FALSE);
        break; }
#if !ADSP_AUTODEMO
    case 16:                                      /* listo para reproducir */
        AdspLog("ASM: stream abierto y en RUN; el motor de PCM queda activo\n");
        c->OpenStatus = STATUS_SUCCESS;
        InterlockedExchange(&c->PlayState, ADSPAUDIO_STATE_RUNNING);
        KeSetEvent(&c->OpenDone, IO_NO_INCREMENT, FALSE);
        *Step = 100;
        return;
#else
    case 16: {                                    /* escrituras con ventana */
        if (*Sent < ADSP_PCM_CHUNKS && (LONG)*Sent - c->WrDone < (LONG)ADSP_PCM_INFLIGHT) {
            ASM_WRITE_V2 w;
            ULONGLONG pa = c->PcmPa + (ULONGLONG)(*Sent % ADSP_PCM_RING) * ADSP_PCM_CHUNK;

            w.AddrLsw = (UINT32)pa; w.AddrMsw = (UINT32)(pa >> 32);
            w.MemMapHandle = (UINT32)c->MemHandle; w.BufSize = ADSP_PCM_CHUNK;
            w.SeqId = *Sent; w.TsLsw = 0; w.TsMsw = 0; w.Flags = 0;
            st = AdspAprSendSvc(c, APR_SVC_ASM, ADSP_ASM_PORT, ADSP_ASM_PORT, *Sent,
                                ASM_DATA_CMD_WRITE_V2, &w, sizeof(w), *Sent > 1);
            if (!NT_SUCCESS(st)) { AdspLog("ASM: write %u nt=0x%08x\n", (unsigned)*Sent, (unsigned)st); *WaitRsp = 16; *Step = 90; return; }
            (*Sent)++;
            return;
        }
        {
            static LONG last = -1;

            if (*Sent == 1 && c->WrDone == 0) last = 0;      /* nueva corrida */
            if (last != c->WrDone) { last = c->WrDone; *T0 = now; }
        }
        if (c->WrDone >= (LONG)ADSP_PCM_CHUNKS) {
            AdspLog("ASM: %d escrituras confirmadas (%d con error)\n", (int)c->WrDone, (int)c->WrBadStatus);
            *Step = 17;
        } else if (now - *T0 > 100ll * 1000 * 1000 * 3) {
            AdspLog("ASM: sin write-done: enviadas %u, confirmadas %d\n", (unsigned)*Sent, (int)c->WrDone);
            *WaitRsp = 16; *Step = 90;
        }
        return; }
#endif
    case 17: {                                    /* cerrar stream */
        st = AdspAprSendSvc(c, APR_SVC_ASM, ADSP_ASM_PORT, ADSP_ASM_PORT, ADSP_ASM_SESSION << 8,
                            ASM_STREAM_CMD_CLOSE, NULL, 0, FALSE);
        break; }
    case 18:
        AdspLog("ASM: stream cerrado; prueba de PCM terminada\n");
        AdspAsmFree(c);
        *Step = 100;
        return;
    default:
        return;
    }
    AdspLog("ASM: paso %d GlinkSend nt=0x%08x\n", (int)*Step, (unsigned)st);
    if (!NT_SUCCESS(st)) { *WaitRsp = *Step; *Step = 90; return; }
    *WaitRsp = c->AprRsp;
    *T0 = now;
}

/* ---- motor de reproduccion bajo demanda (IOCTL) ----------------------------
 * WRITE copia al anillo (cualquier hilo); el hilo de GLINK saca trozos de
 * ADSPAUDIO_CHUNK_BYTES, los copia al slot de SMEM y envia ASM_DATA_CMD_WRITE_V2
 * con ventana de ADSP_PCM_INFLIGHT. Si falta dato una vez empezado, manda
 * silencio para que el stream no se corte (Underruns). */

static ULONG AdspRingWrite(PADSP_CONTEXT c, const UINT8 *Src, ULONG Len)
{
    KIRQL irql;
    ULONG n, first;

    if (c->Ring == NULL) return 0;
    KeAcquireSpinLock(&c->RingLock, &irql);
    n = ADSP_RING_SIZE - c->RingCount;
    if (n > Len) n = Len;
    first = ADSP_RING_SIZE - c->RingHead;
    if (first > n) first = n;
    RtlCopyMemory(c->Ring + c->RingHead, Src, first);
    if (n > first) RtlCopyMemory(c->Ring, Src + first, n - first);
    c->RingHead = (c->RingHead + n) % ADSP_RING_SIZE;
    c->RingCount += n;
    c->BytesQueued += n;
    if (n != 0) c->PlayStarted = TRUE;
    KeReleaseSpinLock(&c->RingLock, irql);
    return n;
}

/* saca Len bytes del anillo hacia Dst; devuelve FALSE si no hay tantos */
static BOOLEAN AdspRingRead(PADSP_CONTEXT c, PUCHAR Dst, ULONG Len)
{
    KIRQL irql;
    ULONG first;
    BOOLEAN ok = FALSE;

    KeAcquireSpinLock(&c->RingLock, &irql);
    if (c->RingCount >= Len) {
        first = ADSP_RING_SIZE - c->RingTail;
        if (first > Len) first = Len;
        RtlCopyMemory(Dst, c->Ring + c->RingTail, first);
        if (Len > first) RtlCopyMemory(Dst + first, c->Ring, Len - first);
        c->RingTail = (c->RingTail + Len) % ADSP_RING_SIZE;
        c->RingCount -= Len;
        ok = TRUE;
    }
    KeReleaseSpinLock(&c->RingLock, irql);
    return ok;
}

static VOID AdspPlayPump(PADSP_CONTEXT c)
{
    ASM_WRITE_V2 w;
    PUCHAR slot;
    ULONGLONG pa;
    NTSTATUS st;
    ULONG guard;

    for (guard = 0; guard < ADSP_PCM_INFLIGHT; guard++) {
        if ((LONG)c->PlaySent - c->WrDone >= (LONG)ADSP_PCM_INFLIGHT) return;
        if (!c->PlayStarted) return;
        slot = (PUCHAR)c->PcmVa + (c->PlaySent % ADSP_PCM_RING) * ADSP_PCM_CHUNK;
        if (!AdspRingRead(c, slot, ADSP_PCM_CHUNK)) {
            RtlZeroMemory(slot, ADSP_PCM_CHUNK);
            c->Underruns++;
        }
        KeMemoryBarrier();
        pa = c->PcmPa + (ULONGLONG)(c->PlaySent % ADSP_PCM_RING) * ADSP_PCM_CHUNK;
        w.AddrLsw = (UINT32)pa; w.AddrMsw = (UINT32)(pa >> 32);
        w.MemMapHandle = (UINT32)c->MemHandle; w.BufSize = ADSP_PCM_CHUNK;
        w.SeqId = c->PlaySent; w.TsLsw = 0; w.TsMsw = 0; w.Flags = 0;
        st = AdspAprSendSvc(c, APR_SVC_ASM, ADSP_ASM_PORT, ADSP_ASM_PORT, c->PlaySent,
                            ASM_DATA_CMD_WRITE_V2, &w, sizeof(w), TRUE);
        if (!NT_SUCCESS(st)) {
            AdspLog("PLAY: write nt=0x%08x\n", (unsigned)st);
            return;
        }
        c->PlaySent++;
    }
}

/* cierre: ASM close -> ADM close -> idle */
static VOID AdspPlayCloseStep(PADSP_CONTEXT c, LONG *Step, LONG *Wait, ULONGLONG *T0)
{
    ULONGLONG now = KeQueryInterruptTime();
    NTSTATUS st = STATUS_SUCCESS;

    if (*Wait >= 0) {
        if (c->AprRsp <= *Wait) {
            if (now - *T0 < 50ll * 1000 * 1000) return;           /* 5 s */
            AdspLog("PLAY: timeout cerrando (paso %d)\n", (int)*Step);
        } else {
            AdspLog("PLAY: cierre paso %d -> rsp op=0x%08x status=0x%x\n", (int)*Step,
                    (unsigned)c->AprRspOp, (unsigned)c->AprRspStatus);
        }
        *Wait = -1;
        (*Step)++;
        return;
    }
    switch (*Step) {
    case 0:
        st = AdspAprSendSvc(c, APR_SVC_ASM, ADSP_ASM_PORT, ADSP_ASM_PORT, ADSP_ASM_SESSION << 8,
                            ASM_STREAM_CMD_CLOSE, NULL, 0, FALSE);
        break;
    case 1:
        st = AdspAprSendSvc(c, APR_SVC_ADM, (UINT16)AFE_PORT_ID_PRIMARY_MI2S_RX,
                            (UINT16)c->CoppId, 0, ADM_CMD_DEVICE_CLOSE_V5, NULL, 0, FALSE);
        break;
    default:
        InterlockedExchange(&c->PlayState, ADSPAUDIO_STATE_IDLE);
        InterlockedExchange(&c->ReqClose, 0);
        KeSetEvent(&c->CloseDone, IO_NO_INCREMENT, FALSE);
        AdspLog("PLAY: stream cerrado\n");
        *Step = 100;
        return;
    }
    if (!NT_SUCCESS(st)) { *Wait = -1; (*Step)++; return; }
    *Wait = c->AprRsp;
    *T0 = now;
}

static VOID AdspPlayReset(PADSP_CONTEXT c)
{
    c->RingHead = c->RingTail = c->RingCount = 0;
    c->PlaySent = 0; c->Underruns = 0; c->PlayStarted = FALSE; c->BytesQueued = 0;
    InterlockedExchange(&c->WrDone, 0);
    c->OpenStatus = STATUS_TIMEOUT;
    KeClearEvent(&c->OpenDone);
}

static NTSTATUS AdspPlayOpen(PADSP_CONTEXT c)
{
    LARGE_INTEGER to;
    NTSTATUS w;

    if (c->PlayState == ADSPAUDIO_STATE_RUNNING) return STATUS_SUCCESS;
    if (c->AfeFailed) return STATUS_DEVICE_NOT_READY;
    if (c->Ring == NULL) return STATUS_INSUFFICIENT_RESOURCES;
    AdspPlayReset(c);
    InterlockedExchange(&c->ReqOpen, 1);
    to.QuadPart = -20ll * 1000 * 1000 * 10;                     /* 20 s */
    w = KeWaitForSingleObject(&c->OpenDone, Executive, KernelMode, FALSE, &to);
    if (w == STATUS_TIMEOUT) {
        InterlockedExchange(&c->ReqOpen, 0);
        return STATUS_TIMEOUT;
    }
    return c->OpenStatus;
}

static NTSTATUS AdspPlayClose(PADSP_CONTEXT c)
{
    LARGE_INTEGER to;

    if (c->PlayState != ADSPAUDIO_STATE_RUNNING) return STATUS_SUCCESS;
    KeClearEvent(&c->CloseDone);
    InterlockedExchange(&c->ReqClose, 1);
    to.QuadPart = -15ll * 1000 * 1000 * 10;
    if (KeWaitForSingleObject(&c->CloseDone, Executive, KernelMode, FALSE, &to) == STATUS_TIMEOUT) {
        return STATUS_TIMEOUT;
    }
    return STATUS_SUCCESS;
}

VOID AdspEvtIoDeviceControl(WDFQUEUE Queue, WDFREQUEST Request, size_t OutLen,
                            size_t InLen, ULONG Code)
{
    PADSP_CONTEXT c = AdspGetContext(WdfIoQueueGetDevice(Queue));
    NTSTATUS st = STATUS_INVALID_DEVICE_REQUEST;
    ULONG_PTR info = 0;

    UNREFERENCED_PARAMETER(OutLen);
    UNREFERENCED_PARAMETER(InLen);
    switch (Code) {
    case IOCTL_ADSPAUDIO_OPEN:
        st = AdspPlayOpen(c);
        break;
    case IOCTL_ADSPAUDIO_CLOSE:
        st = AdspPlayClose(c);
        break;
    case IOCTL_ADSPAUDIO_WRITE: {
        PVOID in = NULL, out = NULL;
        size_t inLen = 0, outLen = 0;
        ULONG acc;

        st = WdfRequestRetrieveInputBuffer(Request, 1, &in, &inLen);
        if (NT_SUCCESS(st)) st = WdfRequestRetrieveOutputBuffer(Request, sizeof(ULONG), &out, &outLen);
        if (!NT_SUCCESS(st) || in == NULL || out == NULL) { if (NT_SUCCESS(st)) st = STATUS_INVALID_PARAMETER; break; }
        if (c->PlayState != ADSPAUDIO_STATE_RUNNING) { st = STATUS_DEVICE_NOT_READY; break; }
        acc = AdspRingWrite(c, (const UINT8 *)in, (ULONG)inLen);
        *(ULONG *)out = acc;
        info = sizeof(ULONG);
        break; }
    case IOCTL_ADSPAUDIO_STATUS: {
        PVOID out = NULL;
        size_t outLen = 0;
        ADSPAUDIO_STATUS *s;

        st = WdfRequestRetrieveOutputBuffer(Request, sizeof(*s), &out, &outLen);
        if (!NT_SUCCESS(st) || out == NULL) { if (NT_SUCCESS(st)) st = STATUS_INVALID_PARAMETER; break; }
        s = (ADSPAUDIO_STATUS *)out;
        s->State = (ULONG)c->PlayState;
        s->RingSize = ADSP_RING_SIZE;
        s->RingFree = ADSP_RING_SIZE - c->RingCount;
        s->Underruns = c->Underruns;
        s->BytesQueued = c->BytesQueued;
        s->BytesPlayed = (ULONGLONG)(ULONG)c->WrDone * ADSP_PCM_CHUNK;
        info = sizeof(*s);
        break; }
    default:
        break;
    }
    WdfRequestCompleteWithInformation(Request, st, info);
}

static VOID AdspGlinkThread(PVOID Ctx)
{
    PADSP_CONTEXT c = (PADSP_CONTEXT)Ctx;
    LARGE_INTEGER d;
    ULONGLONG t0 = KeQueryInterruptTime();
    ULONGLONG tNext = 0;
    ULONGLONG tKick = t0 + 20ll * 1000 * 1000;
    ULONG kicks = 0;
    ULONG tries = 0;
    LONG afeStep = -1, afeWait = -1;
    LONG asmStep = -1, asmWait = -1;
    ULONGLONG asmT0 = 0, asmAt = 0;
    ULONG asmSent = 0;
    LONG clsStep = -1, clsWait = -1;
    ULONGLONG clsT0 = 0;
    LONG stStep = 0, stRound = 0;
    ULONGLONG stT = 0;
    BOOLEAN opened = FALSE, dumped = FALSE;

    d.QuadPart = -10000;                 /* 1 ms */
    while (c->PollStop == 0) {
        ULONGLONG now = KeQueryInterruptTime();

        (VOID)GlinkPoll(c->Glink);

        if (GlinkIsOpen(c->AprCh)) {
            if (!opened) {
                opened = TRUE;
                AdspLog("ADSP: el ADSP abrio '%s'\n", ADSP_APR_CHANNEL);
            }
            if (c->FwkUp && afeStep < 0) {
                afeStep = 0;                     /* version recibida: arrancar AFE */
                c->AfeToneAt = KeQueryInterruptTime() + (ULONGLONG)ADSP_SELFTEST_DELAY_S * 10ll * 1000 * 1000;
            }
            if (afeStep >= 0 && afeStep < 100) {
                AdspAfeStep(c, &afeStep, &afeWait);
                if (afeStep == 100) {
                    asmAt = KeQueryInterruptTime() + 50ll * 1000 * 1000;   /* +5 s: termina el DTMF */
                }
            }
#if ADSP_AUTODEMO
            if (afeStep == 100 && asmStep < 0 && asmAt != 0 && now >= asmAt) {
                asmStep = 9;
            }
#else
            if (c->ReqOpen && afeStep == 100 && asmStep < 0 && c->PlayState == ADSPAUDIO_STATE_IDLE) {
                InterlockedExchange(&c->ReqOpen, 0);
                InterlockedExchange(&c->PlayState, ADSPAUDIO_STATE_OPENING);
                asmWait = -1;
                asmStep = 9;
            }
            if (asmStep == 100 && c->PlayState == ADSPAUDIO_STATE_IDLE) {
                asmStep = -1;                         /* se puede volver a abrir */
            }
            if (c->PlayState == ADSPAUDIO_STATE_RUNNING) {
                if (c->ReqClose && clsStep < 0) {
                    InterlockedExchange(&c->PlayState, ADSPAUDIO_STATE_CLOSING);
                    clsStep = 0; clsWait = -1;
                } else {
                    AdspPlayPump(c);
                }
            }
            if (c->PlayState == ADSPAUDIO_STATE_CLOSING && clsStep >= 0 && clsStep < 100) {
                AdspPlayCloseStep(c, &clsStep, &clsWait, &clsT0);
                if (clsStep == 100) { clsStep = -1; asmStep = -1; }
            }
#if ADSP_SELFTEST
            switch (stStep) {
            case 0:
                if (afeStep == 100 && !c->AfeFailed && now >= c->AfeToneAt) {
                    AdspLog("TEST: ronda %d: OPEN\n", (int)(stRound + 1));
                    AdspPlayReset(c);
                    InterlockedExchange(&c->ReqOpen, 1);
                    stT = now;
                    stStep = 1;
                }
                break;
            case 1:
                if (c->PlayState == ADSPAUDIO_STATE_RUNNING) {
                    AdspLog("TEST: stream abierto, tocando %d s\n", (int)ADSP_SELFTEST_SECONDS);
                    stT = now;
                    stStep = 2;
                } else if (c->PlayState == ADSPAUDIO_STATE_IDLE && c->ReqOpen == 0 &&
                           KeReadStateEvent(&c->OpenDone) && !NT_SUCCESS(c->OpenStatus)) {
                    AdspLog("TEST: OPEN fallo nt=0x%08x\n", (unsigned)c->OpenStatus);
                    stStep = 9;
                } else if (now - stT > 300ll * 1000 * 1000) {
                    AdspLog("TEST: OPEN sin respuesta en 30 s\n");
                    stStep = 9;
                }
                break;
            case 2: {
                ULONG k;

                for (k = 0; k < 400 && c->RingCount + sizeof(kSine1k48) <= ADSP_RING_SIZE; k++) {   /* llenar el anillo: el hilo despierta cada ~15-40 ms */
                    (VOID)AdspRingWrite(c, (const UINT8 *)kSine1k48, sizeof(kSine1k48));
                }
                if (now - stT > (ULONGLONG)ADSP_SELFTEST_SECONDS * 10ll * 1000 * 1000) {
                    AdspLog("TEST: CLOSE (puestos %llu bytes, reproducidos %llu, subdesbordes %u)\n",
                            c->BytesQueued, (ULONGLONG)(ULONG)c->WrDone * ADSP_PCM_CHUNK,
                            (unsigned)c->Underruns);
                    InterlockedExchange(&c->ReqClose, 1);
                    stStep = 3;
                }
                break; }
            case 3:
                if (c->PlayState == ADSPAUDIO_STATE_IDLE) {
                    stRound++;
                    AdspLog("TEST: ronda %d terminada\n", (int)stRound);
                    if (stRound < ADSP_SELFTEST_ROUNDS) {
                        stStep = 0;
                        c->AfeToneAt = now + 20ll * 1000 * 1000;     /* 2 s de pausa */
                    } else {
                        stStep = 9;
                    }
                }
                break;
            default:
                break;
            }
#endif
#endif
            if (asmStep >= 9 && asmStep < 100 && c->PlayState != ADSPAUDIO_STATE_CLOSING) {
                AdspAsmStep(c, &asmStep, &asmWait, &asmT0, &asmSent);
            }
            if (!c->FwkUp && tries < 60 && now >= tNext) {
                tries++;
                tNext = now + 10ll * 1000 * 1000;      /* cada 1 s hasta que el ADSP conteste en serio */
                AdspAprSendGetVersion(c);
            }
        } else if (kicks < 30 && now >= tKick) {
            /* el ADSP aun no contesta: re-avisar cada 2 s y resumir cada 6 s */
            tKick = now + 20ll * 1000 * 1000;
            GlinkKick(c->Glink);
            if (++kicks % 3 == 0) {
                GlinkDump(c->Glink);
            }
        }
        if (!opened && !dumped && now - t0 > ADSP_GLINK_NOOPEN_100NS) {
            dumped = TRUE;
            AdspLog("ADSP: el ADSP no abrio '%s' en 15 s\n", ADSP_APR_CHANNEL);
            GlinkDump(c->Glink);
        }
        KeDelayExecutionThread(KernelMode, FALSE, &d);
    }
    PsTerminateSystemThread(STATUS_SUCCESS);
}

static VOID AdspGlinkStop(PADSP_CONTEXT c)
{
    if (c->PollThread != NULL) {
        InterlockedExchange(&c->PollStop, 1);
        KeWaitForSingleObject(c->PollThread, Executive, KernelMode, FALSE, NULL);
        ObDereferenceObject(c->PollThread);
        c->PollThread = NULL;
    }
    if (c->Glink != NULL) {
        GlinkDump(c->Glink);
        GlinkClose(c->Glink);
        c->Glink = NULL;
    }
    c->AprCh = NULL;
    /* el buffer PCM no se libera aqui: el ADSP puede tenerlo mapeado */
    FreeBlob(&c->GlinkMem);
}

/*
 * Se llama ANTES de arrancar el ADSP (como Linux, que registra el borde GLINK
 * mucho antes): el ADSP tiene que encontrar el item 479 al inicializarse.
 */
static NTSTATUS AdspGlinkPrepare(PADSP_CONTEXT c, BOOLEAN Stopped, BOOLEAN Defer)
{
    WDF_OBJECT_ATTRIBUTES attrs;
    GLINK_CONFIG cfg;
    PVOID buf = NULL;
    NTSTATUS st;

    WDF_OBJECT_ATTRIBUTES_INIT(&attrs);
    attrs.ParentObject = c->Device;
    st = WdfMemoryCreate(&attrs, NonPagedPoolNx, ADSP_TAG, sizeof(GLINK_CTX),
                         &c->GlinkMem, &buf);
    if (!NT_SUCCESS(st)) {
        return st;
    }
    c->Glink = (GLINK_CTX *)buf;

    RtlZeroMemory(&cfg, sizeof(cfg));
    cfg.Smem      = &c->Smem;
    cfg.RemotePid = 2;
    cfg.Ipc       = c->P2p.Ipc;          /* mismo mailbox APCS que SMP2P */
    cfg.IpcBit    = ADSP_GLINK_IPC_BIT;
    cfg.AdspStopped  = Stopped;
    cfg.DeferVersion = Defer;
    st = GlinkInit(c->Glink, &cfg);
    if (!NT_SUCCESS(st)) {
        goto fail;
    }
    st = GlinkRegister(c->Glink, ADSP_APR_CHANNEL, AdspAprRx, c,
                       ADSP_APR_INTENT_SIZE, ADSP_APR_INTENT_COUNT, &c->AprCh);
    if (!NT_SUCCESS(st)) {
        goto fail;
    }

    AdspLog("ADSP: GLINK preparado (antes del arranque del ADSP)\n");
    return STATUS_SUCCESS;

fail:
    AdspLog("ADSP: GLINK no se preparo nt=0x%08x\n", (unsigned)st);
    AdspGlinkStop(c);
    return st;
}

static NTSTATUS AdspGlinkStart(PADSP_CONTEXT c)
{
    OBJECT_ATTRIBUTES oa;
    HANDLE th = NULL;
    NTSTATUS st;

    if (c->Glink == NULL || c->AprCh == NULL) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    c->PollStop = 0;
    c->AprRx = 0;
    c->FwkUp = 0;
    c->AfeRetries = 0;
    c->AfeRetryAt = 0;
    c->AprRsp = 0;
    InitializeObjectAttributes(&oa, NULL, OBJ_KERNEL_HANDLE, NULL, NULL);
    st = PsCreateSystemThread(&th, THREAD_ALL_ACCESS, &oa, NULL, NULL,
                              AdspGlinkThread, c);
    if (!NT_SUCCESS(st)) {
        goto fail;
    }
    /* ObjectType = NULL esta permitido con KernelMode (PsThreadType solo se
     * declara en ntifs.h) */
    st = ObReferenceObjectByHandle(th, THREAD_ALL_ACCESS, NULL,
                                   KernelMode, (PVOID *)&c->PollThread, NULL);
    if (!NT_SUCCESS(st)) {
        /* el hilo ya corre: se para y se espera por el handle antes de liberar */
        InterlockedExchange(&c->PollStop, 1);
        ZwWaitForSingleObject(th, FALSE, NULL);
        ZwClose(th);
        goto fail;
    }
    ZwClose(th);
    AdspLog("ADSP: GLINK iniciado, esperando que el ADSP abra '%s'\n",
            ADSP_APR_CHANNEL);
    return STATUS_SUCCESS;

fail:
    AdspLog("ADSP: GLINK no arranco nt=0x%08x\n", (unsigned)st);
    AdspGlinkStop(c);
    return st;
}

/* ---- arranque y apagado -------------------------------------------------- */

static VOID AdspTeardown(PADSP_CONTEXT c)
{
    AdspGlinkStop(c);                 /* antes de apagar el ADSP y cerrar SMEM */
    if (c->Booted) {
        NTSTATUS st = PasShutdown(PAS_ID_ADSP);

        AdspLog("ADSP: PAS_SHUTDOWN nt=0x%08x\n", (unsigned)st);
        if (NT_SUCCESS(st) && c->SmemOpen) {
            /* ADSP parado: SMEM vuelve a estado de arranque en frio para que
             * el proximo arranque no vea los indices GLINK de este ciclo */
            GlinkResetShared(&c->Smem, 2);
        }
        c->Booted = FALSE;
    }
    if (c->P2pOpen) {
        Smp2pClose(&c->P2p);
        c->P2pOpen = FALSE;
    }
    if (c->SmemOpen) {
        SmemClose(&c->Smem);
        c->SmemOpen = FALSE;
    }
    FreeBlob(&c->SegMem);
    FreeBlob(&c->MdtMem);
}

static NTSTATUS AdspBoot(PADSP_CONTEXT c)
{
    SMP2P_CONFIG p2p;
    PIL_ADSP_CONFIG pil;
    NTSTATUS st;

    st = SmemInit(&c->Smem, ADSP_SMEM_PA, ADSP_SMEM_SIZE, ADSP_LOCK_PA);
    if (!NT_SUCCESS(st)) {
        AdspLog("ADSP: SmemInit fallo nt=0x%08x\n", (unsigned)st);
        return st;
    }
    c->SmemOpen = TRUE;

    RtlZeroMemory(&p2p, sizeof(p2p));
    p2p.Smem      = &c->Smem;
    p2p.LocalPid  = 0;
    p2p.RemotePid = 2;
    p2p.InItem    = 0x1BB;
    p2p.OutItem   = 0x1AD;
    p2p.MailboxPa = ADSP_MBOX_PA;
    p2p.IpcOffset = 8;
    p2p.IpcBit    = 10;
    st = Smp2pOpen(&c->P2p, &p2p);
    if (!NT_SUCCESS(st)) {
        AdspLog("ADSP: Smp2pOpen fallo nt=0x%08x\n", (unsigned)st);
        return st;
    }
    c->P2pOpen = TRUE;

#if ADSP_GLINK_MODE >= 1
    /* GLINK: descriptor y FIFOs en SMEM antes de que el ADSP los busque */
    (VOID)AdspGlinkPrepare(c, TRUE, ADSP_GLINK_MODE == 1);
#endif
#if ADSP_GLINK_MODE == 3
    (VOID)AdspGlinkStart(c);             /* hilo vivo durante el arranque */
#endif
#if ADSP_GLINK_MODE == 4
    /* Experimento: solo el descriptor (478) antes del arranque; el FIFO tx
     * (479) se crea despues de err-ready, como en el modo 0. */
    {
        NTSTATUS ds = SmemAlloc(&c->Smem, 2, 478, 32);
        PVOID dp = NULL;
        SIZE_T dsz = 0;

        AdspLog("ADSP: modo 4, item 478 pre-arranque nt=0x%08x\n", (unsigned)ds);
        if (NT_SUCCESS(ds) &&
            NT_SUCCESS(SmemGet(&c->Smem, 2, 478, &dp, &dsz)) && dsz == 32) {
            RtlZeroMemory(dp, 32);
            KeMemoryBarrier();
        }
    }
#endif

    RtlZeroMemory(&pil, sizeof(pil));
    pil.PasId      = PAS_ID_ADSP;
    pil.RegionBase = ADSP_REGION_PA;
    pil.RegionSize = ADSP_REGION_SIZE;
    pil.GetBlob    = AdspGetBlob;
    pil.Ctx        = c;
    pil.ProxyVote  = AdspProxyVote;

    st = PilBootAdsp(&pil);
    FreeBlob(&c->SegMem);
    FreeBlob(&c->MdtMem);
    if (!NT_SUCCESS(st)) {
        AdspLog("ADSP: PilBootAdsp fallo nt=0x%08x\n", (unsigned)st);
        return st;
    }
    c->Booted = TRUE;

    st = Smp2pWaitReady(&c->P2p, ADSP_READY_TIMEOUT_MS);
    if (NT_SUCCESS(st)) {
        AdspLog("ADSP: err-ready recibido, el ADSP esta corriendo\n");
        /* un fallo de GLINK no tumba el arranque: el log dira que paso */
#if ADSP_GLINK_MODE == 0 || ADSP_GLINK_MODE == 4
        (VOID)AdspGlinkPrepare(c, FALSE, FALSE);
#elif ADSP_GLINK_MODE == 1
        if (c->Glink != NULL) {
            (VOID)GlinkLinkUp(c->Glink);
        }
#endif
#if ADSP_GLINK_MODE != 3
        (VOID)AdspGlinkStart(c);
#endif
    } else {
        AdspLog("ADSP: sin err-ready nt=0x%08x (canal SMP2P abierto=%d)\n",
                (unsigned)st, (int)c->P2p.Open);
    }
    return st;
}

/* ---- callbacks de WDF ---------------------------------------------------- */

NTSTATUS AdspEvtPrepareHardware(WDFDEVICE Device, WDFCMRESLIST Raw,
                                WDFCMRESLIST Translated)
{
    struct { ULONGLONG Pa; ULONG Len; const char *Name; BOOLEAN Found; } need[] = {
        { ADSP_SMEM_PA,   ADSP_SMEM_SIZE,         "SMEM",    FALSE },
        { ADSP_LOCK_PA,   ADSP_LOCK_SIZE,         "TCSR",    FALSE },
        { ADSP_MBOX_PA,   ADSP_MBOX_SIZE,         "mailbox", FALSE },
        { ADSP_REGION_PA, (ULONG)ADSP_REGION_SIZE, "region", FALSE },
    };
    ULONG i, j, n;
    NTSTATUS st = STATUS_SUCCESS;

    UNREFERENCED_PARAMETER(Device);
    UNREFERENCED_PARAMETER(Raw);

    n = WdfCmResourceListGetCount(Translated);
    for (i = 0; i < n; i++) {
        PCM_PARTIAL_RESOURCE_DESCRIPTOR d =
            WdfCmResourceListGetDescriptor(Translated, i);

        if (d == NULL || d->Type != CmResourceTypeMemory) {
            continue;
        }
        for (j = 0; j < sizeof(need) / sizeof(need[0]); j++) {
            if ((ULONGLONG)d->u.Memory.Start.QuadPart == need[j].Pa &&
                d->u.Memory.Length >= need[j].Len) {
                need[j].Found = TRUE;
            }
        }
    }

    for (j = 0; j < sizeof(need) / sizeof(need[0]); j++) {
        if (!need[j].Found) {
            AdspLog("ADSP: falta el recurso %s (0x%llx) en el _CRS\n",
                    need[j].Name, (unsigned long long)need[j].Pa);
            st = STATUS_DEVICE_CONFIGURATION_ERROR;
        }
    }
    return st;
}

NTSTATUS AdspEvtD0Entry(WDFDEVICE Device, WDF_POWER_DEVICE_STATE PreviousState)
{
    PADSP_CONTEXT c = AdspGetContext(Device);
    NTSTATUS st;

    UNREFERENCED_PARAMETER(PreviousState);

    AdspLog("ADSP: D0Entry, arrancando\n");
    st = AdspBoot(c);
    if (!NT_SUCCESS(st)) {
        AdspTeardown(c);
    }
    return st;
}

NTSTATUS AdspEvtD0Exit(WDFDEVICE Device, WDF_POWER_DEVICE_STATE TargetState)
{
    UNREFERENCED_PARAMETER(TargetState);

    AdspTeardown(AdspGetContext(Device));
    return STATUS_SUCCESS;
}

NTSTATUS AdspEvtDeviceAdd(WDFDRIVER Driver, PWDFDEVICE_INIT DeviceInit)
{
    WDF_PNPPOWER_EVENT_CALLBACKS pnp;
    WDF_OBJECT_ATTRIBUTES attrs;
    WDFDEVICE device;
    NTSTATUS st;

    UNREFERENCED_PARAMETER(Driver);

    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnp);
    pnp.EvtDevicePrepareHardware = AdspEvtPrepareHardware;
    pnp.EvtDeviceD0Entry         = AdspEvtD0Entry;
    pnp.EvtDeviceD0Exit          = AdspEvtD0Exit;
    WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnp);

    /* cualquier usuario puede abrir la interfaz de audio (SYSTEM y admins todo;
     * el resto lectura/escritura), asi audiotest no necesita "administrador" */
#if (ADSP_AUDIO_IFACE & 1)
    {
        DECLARE_CONST_UNICODE_STRING(sddl, L"D:P(A;;GA;;;SY)(A;;GA;;;BA)(A;;GRGW;;;WD)");

        st = WdfDeviceInitAssignSDDLString(DeviceInit, &sddl);
        if (!NT_SUCCESS(st)) {
            AdspLog("ADSP: SDDL nt=0x%08x\n", (unsigned)st);
        }
    }
#endif

    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attrs, ADSP_CONTEXT);
    st = WdfDeviceCreate(&DeviceInit, &attrs, &device);
    if (!NT_SUCCESS(st)) {
        return st;
    }

    {
        PADSP_CONTEXT c = AdspGetContext(device);
        WDF_IO_QUEUE_CONFIG qc;
        WDF_OBJECT_ATTRIBUTES ma;
        WDFMEMORY mem;
        WDFQUEUE q;

        c->Device = device;
        KeInitializeEvent(&c->OpenDone, NotificationEvent, FALSE);
        KeInitializeEvent(&c->CloseDone, NotificationEvent, FALSE);
        KeInitializeSpinLock(&c->RingLock);
        c->PlayState = ADSPAUDIO_STATE_IDLE;

        WDF_OBJECT_ATTRIBUTES_INIT(&ma);
        ma.ParentObject = device;
        st = WdfMemoryCreate(&ma, NonPagedPoolNx, ADSP_TAG, ADSP_RING_SIZE, &mem, (PVOID *)&c->Ring);
        if (!NT_SUCCESS(st)) {
            AdspLog("ADSP: sin memoria para el anillo de audio nt=0x%08x\n", (unsigned)st);
            c->Ring = NULL;
        }

#if (ADSP_AUDIO_IFACE & 2)
        WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&qc, WdfIoQueueDispatchParallel);
        qc.EvtIoDeviceControl = AdspEvtIoDeviceControl;
        st = WdfIoQueueCreate(device, &qc, WDF_NO_OBJECT_ATTRIBUTES, &q);
        if (!NT_SUCCESS(st)) {
            AdspLog("ADSP: cola de IOCTL nt=0x%08x\n", (unsigned)st);
            return st;
        }
#else
        UNREFERENCED_PARAMETER(qc);
        UNREFERENCED_PARAMETER(q);
#endif
#if (ADSP_AUDIO_IFACE & 4)
        st = WdfDeviceCreateDeviceInterface(device, &GUID_DEVINTERFACE_ADSP_AUDIO, NULL);
        AdspLog("ADSP: DeviceAdd v6 ADSP_AUDIO_IFACE=%d, interfaz de audio nt=0x%08x\n",
                (int)ADSP_AUDIO_IFACE, (unsigned)st);
#endif
    }
    return STATUS_SUCCESS;
}

NTSTATUS DriverEntry(PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
    WDF_DRIVER_CONFIG cfg;

    WDF_DRIVER_CONFIG_INIT(&cfg, AdspEvtDeviceAdd);
    return WdfDriverCreate(DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES,
                           &cfg, WDF_NO_HANDLE);
}
