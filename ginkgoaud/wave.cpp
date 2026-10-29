/*
 * wave.cpp - filtro WaveRT del altavoz.
 *
 * Windows escribe PCM 48 kHz / 16 bits / estereo en un buffer ciclico (WaveRT).
 * No hay DMA real: un hilo de sistema copia del buffer al anillo de adspdrv con
 * IOCTL_ADSPAUDIO_WRITE (aplicando la ganancia de proteccion).
 *
 * Reloj: la posicion que se informa a Windows avanza con el contador de
 * rendimiento (QPC) a 192000 bytes/s mientras el stream esta en RUN. Los datos
 * se envian al ADSP con GKA_PREROLL_BYTES de silencio por delante; si el anillo
 * de adspdrv se llena (deriva de reloj) se descartan datos.
 */
#include "ginkgoaud.h"

/* ------------------------------------------------------------------ */
/* Descriptor del filtro                                              */
/* ------------------------------------------------------------------ */
static KSDATARANGE_AUDIO g_PinRange =
{
    {
        sizeof(KSDATARANGE_AUDIO), 0, 0, 0,
        STATICGUIDOF(KSDATAFORMAT_TYPE_AUDIO),
        STATICGUIDOF(KSDATAFORMAT_SUBTYPE_PCM),
        STATICGUIDOF(KSDATAFORMAT_SPECIFIER_WAVEFORMATEX)
    },
    GKA_CHANNELS,       /* MaximumChannels */
    GKA_BITS,           /* MinimumBitsPerSample */
    GKA_BITS,           /* MaximumBitsPerSample */
    GKA_RATE,           /* MinimumSampleFrequency */
    GKA_RATE            /* MaximumSampleFrequency */
};
static PKSDATARANGE g_PinRangePtrs[] = { (PKSDATARANGE)&g_PinRange };

static KSDATARANGE g_BridgeRange[] =
{
    {
        sizeof(KSDATARANGE), 0, 0, 0,
        STATICGUIDOF(KSDATAFORMAT_TYPE_AUDIO),
        STATICGUIDOF(KSDATAFORMAT_SUBTYPE_ANALOG),
        STATICGUIDOF(KSDATAFORMAT_SPECIFIER_NONE)
    }
};
static PKSDATARANGE g_BridgeRangePtrs[] = { &g_BridgeRange[0] };

static PCPIN_DESCRIPTOR g_WavePins[] =
{
    {   /* 0: pin de streaming (render), una instancia */
        1, 1, 0, NULL,
        {
            0, NULL, 0, NULL,
            SIZEOF_ARRAY(g_PinRangePtrs), g_PinRangePtrs,
            KSPIN_DATAFLOW_IN, KSPIN_COMMUNICATION_SINK,
            &KSCATEGORY_AUDIO, NULL, 0
        }
    },
    {   /* 1: puente hacia la topologia */
        0, 0, 0, NULL,
        {
            0, NULL, 0, NULL,
            SIZEOF_ARRAY(g_BridgeRangePtrs), g_BridgeRangePtrs,
            KSPIN_DATAFLOW_OUT, KSPIN_COMMUNICATION_NONE,
            &KSCATEGORY_AUDIO, NULL, 0
        }
    }
};

static PCCONNECTION_DESCRIPTOR g_WaveConnections[] =
{
    { PCFILTER_NODE, 0, PCFILTER_NODE, 1 }
};

static PCFILTER_DESCRIPTOR g_WaveFilter =
{
    0, NULL,
    sizeof(PCPIN_DESCRIPTOR), SIZEOF_ARRAY(g_WavePins), g_WavePins,
    sizeof(PCNODE_DESCRIPTOR), 0, NULL,
    SIZEOF_ARRAY(g_WaveConnections), g_WaveConnections,
    0, NULL
};

static LONG ReadGain(VOID);          /* definida junto a ApplyGain */

/* ------------------------------------------------------------------ */
/* Clases                                                             */
/* ------------------------------------------------------------------ */
class CMiniportWave : public IMiniportWaveRT, public CUnknown
{
public:
    DECLARE_STD_UNKNOWN();
    DEFINE_STD_CONSTRUCTOR(CMiniportWave);
    ~CMiniportWave() {}

    IMP_IMiniportWaveRT;

    LONG m_StreamActive = 0;      /* una sola instancia de stream */
};

#define GKA_TMP_BYTES 16384u      /* trozo maximo por IOCTL (multiplo de GKA_BLOCK) */

class CMiniportWaveStream : public IMiniportWaveRTStream, public CUnknown
{
public:
    DECLARE_STD_UNKNOWN();
    DEFINE_STD_CONSTRUCTOR(CMiniportWaveStream);
    ~CMiniportWaveStream();

    IMP_IMiniportWaveRTStream;

    NTSTATUS Init(_In_ CMiniportWave *Miniport, _In_ PPORTWAVERTSTREAM PortStream);

private:
    ULONGLONG PositionNowLocked(VOID);
    ULONGLONG PositionNow(VOID);
    NTSTATUS  StartThread(VOID);
    VOID      StopThread(VOID);
    VOID      Worker(VOID);
    VOID      Session(VOID);
    static VOID NTAPI ThreadThunk(_In_ PVOID Context);

    CMiniportWave      *m_Miniport = NULL;
    PPORTWAVERTSTREAM   m_PortStream = NULL;        /* referencia debil */

    PMDL                m_Mdl = NULL;
    PUCHAR              m_Buf = NULL;
    ULONG               m_BufSize = 0;

    KSSTATE             m_State = KSSTATE_STOP;
    KSPIN_LOCK          m_PosLock;
    LARGE_INTEGER       m_StartQpc = {};
    LARGE_INTEGER       m_Freq = {};
    ULONGLONG           m_BasePos = 0;              /* bytes acumulados hasta el ultimo cambio de estado */

    PKTHREAD            m_Thread = NULL;
    KEVENT              m_QuitEvt;                  /* terminar el hilo */
    KEVENT              m_RunEvt;                   /* el stream esta en RUN */
    volatile LONG       m_InSession = 0;
    ULONG               m_PosLogs = 0;
    ULONGLONG           m_Copied = 0;               /* bytes ya enviados al ADSP (solo el hilo) */
    PUCHAR              m_Tmp = NULL;
    LONG                m_Gain = GKA_GAIN_DEFAULT_Q15;
};

/* ------------------------------------------------------------------ */
/* CMiniportWave                                                      */
/* ------------------------------------------------------------------ */
STDMETHODIMP_(NTSTATUS)
CMiniportWave::NonDelegatingQueryInterface(_In_ REFIID Interface, _COM_Outptr_ PVOID *Object)
{
    if (IsEqualGUIDAligned(Interface, IID_IUnknown)) {
        *Object = PVOID(PUNKNOWN(PMINIPORTWAVERT(this)));
    } else if (IsEqualGUIDAligned(Interface, IID_IMiniport)) {
        *Object = PVOID(PMINIPORT(this));
    } else if (IsEqualGUIDAligned(Interface, IID_IMiniportWaveRT)) {
        *Object = PVOID(PMINIPORTWAVERT(this));
    } else {
        *Object = NULL;
    }

    if (*Object) {
        PUNKNOWN(*Object)->AddRef();
        return STATUS_SUCCESS;
    }
    return STATUS_INVALID_PARAMETER;
}

STDMETHODIMP_(NTSTATUS)
CMiniportWave::GetDescription(_Out_ PPCFILTER_DESCRIPTOR *OutFilterDescriptor)
{
    *OutFilterDescriptor = &g_WaveFilter;
    return STATUS_SUCCESS;
}

STDMETHODIMP_(NTSTATUS)
CMiniportWave::DataRangeIntersection(_In_ ULONG PinId, _In_ PKSDATARANGE ClientDataRange,
                                     _In_ PKSDATARANGE MyDataRange, _In_ ULONG OutputBufferLength,
                                     _Out_writes_bytes_to_opt_(OutputBufferLength, *ResultantFormatLength)
                                     PVOID ResultantFormat, _Out_ PULONG ResultantFormatLength)
{
    UNREFERENCED_PARAMETER(PinId);
    UNREFERENCED_PARAMETER(MyDataRange);
    UNREFERENCED_PARAMETER(OutputBufferLength);
    UNREFERENCED_PARAMETER(ResultantFormat);
    UNREFERENCED_PARAMETER(ResultantFormatLength);

    if (!IsEqualGUIDAligned(ClientDataRange->Specifier, KSDATAFORMAT_SPECIFIER_WAVEFORMATEX)) {
        return STATUS_NOT_IMPLEMENTED;
    }
    /* El resto lo resuelve el manejador de clase de PortCls con nuestro rango */
    return STATUS_NOT_IMPLEMENTED;
}

STDMETHODIMP_(NTSTATUS)
CMiniportWave::Init(_In_ PUNKNOWN UnknownAdapter, _In_ PRESOURCELIST ResourceList, _In_ PPORTWAVERT Port)
{
    UNREFERENCED_PARAMETER(UnknownAdapter);
    UNREFERENCED_PARAMETER(ResourceList);
    UNREFERENCED_PARAMETER(Port);
    return STATUS_SUCCESS;
}

STDMETHODIMP_(NTSTATUS)
CMiniportWave::GetDeviceDescription(_Out_ PDEVICE_DESCRIPTION DmaDeviceDescription)
{
    RtlZeroMemory(DmaDeviceDescription, sizeof(DEVICE_DESCRIPTION));
    DmaDeviceDescription->Master = TRUE;
    DmaDeviceDescription->ScatterGather = TRUE;
    DmaDeviceDescription->Dma32BitAddresses = TRUE;
    DmaDeviceDescription->InterfaceType = PCIBus;
    DmaDeviceDescription->MaximumLength = 0xFFFFFFFF;
    return STATUS_SUCCESS;
}

STDMETHODIMP_(NTSTATUS)
CMiniportWave::NewStream(_Out_ PMINIPORTWAVERTSTREAM *OutStream, _In_ PPORTWAVERTSTREAM OuterUnknown,
                         _In_ ULONG Pin, _In_ BOOLEAN Capture, _In_ PKSDATAFORMAT DataFormat)
{
    NTSTATUS st;
    CMiniportWaveStream *stream;
    PWAVEFORMATEX wfx;

    *OutStream = NULL;

    if (Pin != 0 || Capture) {
        return STATUS_INVALID_PARAMETER;
    }
    if (DataFormat->FormatSize < sizeof(KSDATAFORMAT_WAVEFORMATEX)) {
        return STATUS_INVALID_PARAMETER;
    }
    wfx = (PWAVEFORMATEX)(DataFormat + 1);
    if (wfx->nSamplesPerSec != GKA_RATE || wfx->nChannels != GKA_CHANNELS ||
        wfx->wBitsPerSample != GKA_BITS) {
        GkaLog("NewStream: formato no soportado (%lu Hz, %u ch, %u bits)\n",
               wfx->nSamplesPerSec, wfx->nChannels, wfx->wBitsPerSample);
        return STATUS_INVALID_PARAMETER;
    }
    if (InterlockedCompareExchange(&m_StreamActive, 1, 0) != 0) {
        return STATUS_INSUFFICIENT_RESOURCES;       /* ya hay un stream */
    }

    stream = new (POOL_FLAG_NON_PAGED, GKA_TAG) CMiniportWaveStream(NULL);
    if (!stream) {
        InterlockedExchange(&m_StreamActive, 0);
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    stream->AddRef();
    st = stream->Init(this, OuterUnknown);
    if (NT_SUCCESS(st)) {
        *OutStream = PMINIPORTWAVERTSTREAM(stream);
        (*OutStream)->AddRef();
        GkaLog("NewStream OK (pin %lu)\n", Pin);
    } else {
        InterlockedExchange(&m_StreamActive, 0);
    }
    stream->Release();      /* referencia privada */
    return st;
}

NTSTATUS CreateMiniportWave(_Out_ PUNKNOWN *Unknown)
{
    GkaLog("CreateMiniportWave  [wave.cpp %s]\n", GKA_BUILD_ID);
    CMiniportWave *obj = new (POOL_FLAG_NON_PAGED, GKA_TAG) CMiniportWave(NULL);
    if (!obj) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    obj->AddRef();
    *Unknown = static_cast<PUNKNOWN>(static_cast<PMINIPORTWAVERT>(obj));
    return STATUS_SUCCESS;
}

/* ------------------------------------------------------------------ */
/* CMiniportWaveStream                                                */
/* ------------------------------------------------------------------ */
STDMETHODIMP_(NTSTATUS)
CMiniportWaveStream::NonDelegatingQueryInterface(_In_ REFIID Interface, _COM_Outptr_ PVOID *Object)
{
    if (IsEqualGUIDAligned(Interface, IID_IUnknown)) {
        *Object = PVOID(PUNKNOWN(PMINIPORTWAVERTSTREAM(this)));
    } else if (IsEqualGUIDAligned(Interface, IID_IMiniportWaveRTStream)) {
        *Object = PVOID(PMINIPORTWAVERTSTREAM(this));
    } else {
        *Object = NULL;
    }

    if (*Object) {
        PUNKNOWN(*Object)->AddRef();
        return STATUS_SUCCESS;
    }
    return STATUS_INVALID_PARAMETER;
}

NTSTATUS CMiniportWaveStream::Init(_In_ CMiniportWave *Miniport, _In_ PPORTWAVERTSTREAM PortStream)
{
    m_Miniport = Miniport;
    m_Miniport->AddRef();                 /* el stream usa el miniport hasta destruirse */
    m_PortStream = PortStream;
    KeInitializeSpinLock(&m_PosLock);
    KeInitializeEvent(&m_QuitEvt, NotificationEvent, FALSE);
    KeInitializeEvent(&m_RunEvt, NotificationEvent, FALSE);
    m_Gain = ReadGain();
    GkaLog("ganancia %ld/32768 (maximo permitido %d)\n", m_Gain, GKA_GAIN_MAX_Q15);
    m_Tmp = (PUCHAR)ExAllocatePool2(POOL_FLAG_NON_PAGED, GKA_TMP_BYTES, GKA_TAG);
    if (!m_Tmp) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    return StartThread();           /* NewStream corre en PASSIVE_LEVEL */
}

CMiniportWaveStream::~CMiniportWaveStream()
{
    GkaLog("stream destruido\n");
    StopThread();
    if (m_Mdl) {
        FreeAudioBuffer(m_Mdl, m_BufSize);
        m_Mdl = NULL;
    }
    if (m_Tmp) {
        ExFreePoolWithTag(m_Tmp, GKA_TAG);
        m_Tmp = NULL;
    }
    if (m_Miniport) {
        InterlockedExchange(&m_Miniport->m_StreamActive, 0);
        m_Miniport->Release();
        m_Miniport = NULL;
    }
}

NTSTATUS CMiniportWaveStream::AllocateAudioBuffer(_In_ ULONG RequestedSize, _Out_ PMDL *AudioBufferMdl,
                                                  _Out_ ULONG *ActualSize, _Out_ ULONG *OffsetFromFirstPage,
                                                  _Out_ MEMORY_CACHING_TYPE *CacheType)
{
    PHYSICAL_ADDRESS high;
    PMDL mdl;

    if (RequestedSize < GKA_BLOCK || m_Mdl) {
        return STATUS_UNSUCCESSFUL;
    }
    RequestedSize -= RequestedSize % GKA_BLOCK;

    high.QuadPart = 0xFFFFFFFFull;
    mdl = m_PortStream->AllocatePagesForMdl(high, RequestedSize);
    if (!mdl) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    m_Buf = (PUCHAR)m_PortStream->MapAllocatedPages(mdl, MmCached);
    if (!m_Buf) {
        m_PortStream->FreePagesFromMdl(mdl);
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    m_Mdl = mdl;
    m_BufSize = RequestedSize;

    *AudioBufferMdl = mdl;
    *ActualSize = RequestedSize;
    *OffsetFromFirstPage = 0;
    *CacheType = MmCached;
    GkaLog("AllocateAudioBuffer: %lu bytes\n", RequestedSize);
    return STATUS_SUCCESS;
}

VOID CMiniportWaveStream::FreeAudioBuffer(_In_opt_ PMDL Mdl, _In_ ULONG Size)
{
    UNREFERENCED_PARAMETER(Size);
    GkaLog("FreeAudioBuffer\n");
    if (!Mdl) {
        return;
    }
    {
        PUCHAR buf = m_Buf;
        m_Buf = NULL;                   /* la sesion del hilo se corta sola */
        while (m_InSession) {
            LARGE_INTEGER ms;
            ms.QuadPart = -10000;
            KeDelayExecutionThread(KernelMode, FALSE, &ms);
        }
        if (buf) {
            m_PortStream->UnmapAllocatedPages(buf, Mdl);
        }
    }
    m_PortStream->FreePagesFromMdl(Mdl);
    if (Mdl == m_Mdl) {
        m_Mdl = NULL;
    }
    m_BufSize = 0;
}

VOID CMiniportWaveStream::GetHWLatency(_Out_ PKSRTAUDIO_HWLATENCY Latency)
{
    GkaLog("GetHWLatency\n");
    Latency->FifoSize = 0;
    Latency->ChipsetDelay = 0;
    Latency->CodecDelay = 0;
}

NTSTATUS CMiniportWaveStream::GetPositionRegister(_Out_ PKSRTAUDIO_HWREGISTER Reg)
{
    UNREFERENCED_PARAMETER(Reg);
    return STATUS_NOT_IMPLEMENTED;
}

NTSTATUS CMiniportWaveStream::GetClockRegister(_Out_ PKSRTAUDIO_HWREGISTER Reg)
{
    UNREFERENCED_PARAMETER(Reg);
    GkaLog("GetClockRegister\n");
    return STATUS_NOT_IMPLEMENTED;
}

NTSTATUS CMiniportWaveStream::SetFormat(_In_ KSDATAFORMAT *DataFormat)
{
    UNREFERENCED_PARAMETER(DataFormat);
    GkaLog("SetFormat\n");
    return STATUS_NOT_SUPPORTED;
}

/* Posicion en bytes desde el inicio del stream. Llamar con m_PosLock tomado. */
ULONGLONG CMiniportWaveStream::PositionNowLocked(VOID)
{
    ULONGLONG pos = m_BasePos;

    if (m_State == KSSTATE_RUN && m_Freq.QuadPart != 0) {
        LARGE_INTEGER now = KeQueryPerformanceCounter(NULL);
        ULONGLONG ticks = (ULONGLONG)(now.QuadPart - m_StartQpc.QuadPart);
        pos += (ticks * GKA_BYTES_PER_SEC) / (ULONGLONG)m_Freq.QuadPart;
    }
    return pos - (pos % GKA_BLOCK);
}

ULONGLONG CMiniportWaveStream::PositionNow(VOID)
{
    KIRQL irql;
    ULONGLONG pos;

    KeAcquireSpinLock(&m_PosLock, &irql);
    pos = PositionNowLocked();
    KeReleaseSpinLock(&m_PosLock, irql);
    return pos;
}

NTSTATUS CMiniportWaveStream::GetPosition(_Out_ KSAUDIO_POSITION *Position)
{
    if (m_PosLogs < 5) {
        m_PosLogs++;
    }
    ULONGLONG pos = PositionNow();
    Position->PlayOffset = pos;
    Position->WriteOffset = pos;
    return STATUS_SUCCESS;
}

NTSTATUS CMiniportWaveStream::SetState(_In_ KSSTATE State)
{
    KIRQL irql;

    GkaLog("SetState %d -> %d\n", (int)m_State, (int)State);

    KeAcquireSpinLock(&m_PosLock, &irql);
    if (State != m_State) {
        if (State == KSSTATE_RUN) {
            m_StartQpc = KeQueryPerformanceCounter(&m_Freq);
        } else if (m_State == KSSTATE_RUN) {
            m_BasePos = PositionNowLocked();     /* congelar la posicion */
        }
        if (State == KSSTATE_STOP) {
            m_BasePos = 0;                       /* STOP reinicia la posicion */
        }
        m_State = State;
    }
    KeReleaseSpinLock(&m_PosLock, irql);

    /* Valido en cualquier IRQL: el hilo (creado en NewStream) hace el resto */
    if (State == KSSTATE_RUN) {
        KeSetEvent(&m_RunEvt, 0, FALSE);
    } else {
        KeClearEvent(&m_RunEvt);
    }
    return STATUS_SUCCESS;
}

/* ------------------------------------------------------------------ */
/* Hilo de reproduccion                                               */
/* ------------------------------------------------------------------ */
VOID NTAPI CMiniportWaveStream::ThreadThunk(_In_ PVOID Context)
{
    ((CMiniportWaveStream *)Context)->Worker();
    PsTerminateSystemThread(STATUS_SUCCESS);
}

NTSTATUS CMiniportWaveStream::StartThread(VOID)
{
    HANDLE h = NULL;
    NTSTATUS st;

    if (m_Thread) {
        return STATUS_SUCCESS;
    }
    KeClearEvent(&m_QuitEvt);
    st = PsCreateSystemThread(&h, THREAD_ALL_ACCESS, NULL, NULL, NULL, ThreadThunk, this);
    if (!NT_SUCCESS(st)) {
        return st;
    }
    st = ObReferenceObjectByHandle(h, SYNCHRONIZE, *PsThreadType, KernelMode, (PVOID *)&m_Thread, NULL);
    ZwClose(h);
    if (!NT_SUCCESS(st)) {
        KeSetEvent(&m_QuitEvt, 0, FALSE);
        m_Thread = NULL;
    }
    return st;
}

VOID CMiniportWaveStream::StopThread(VOID)
{
    if (!m_Thread) {
        return;
    }
    KeSetEvent(&m_QuitEvt, 0, FALSE);
    KeWaitForSingleObject(m_Thread, Executive, KernelMode, FALSE, NULL);
    ObDereferenceObject(m_Thread);
    m_Thread = NULL;
}

static VOID ApplyGain(_In_reads_(Samples) const INT16 *In, _Out_writes_(Samples) INT16 *Out,
                      _In_ ULONG Samples, _In_ LONG Gain)
{
    for (ULONG i = 0; i < Samples; i++) {
        Out[i] = (INT16)(((INT32)In[i] * Gain) >> 15);     /* Gain <= 29000: sin desbordes */
    }
}

/* Ganancia desde el registro (Services\ginkgoaud\Parameters\GainQ15), con tope. */
static LONG ReadGain(VOID)
{
    ULONG val = GKA_GAIN_DEFAULT_Q15;
    ULONG def = GKA_GAIN_DEFAULT_Q15;
    RTL_QUERY_REGISTRY_TABLE q[2];

    RtlZeroMemory(q, sizeof(q));
    q[0].Flags         = RTL_QUERY_REGISTRY_DIRECT;
    q[0].Name          = (PWSTR)L"GainQ15";
    q[0].EntryContext  = &val;
    q[0].DefaultType   = REG_DWORD;
    q[0].DefaultData   = &def;
    q[0].DefaultLength = sizeof(def);
    if (!NT_SUCCESS(RtlQueryRegistryValues(RTL_REGISTRY_SERVICES, L"ginkgoaud\\Parameters",
                                           q, NULL, NULL))) {
        val = GKA_GAIN_DEFAULT_Q15;
    }
    if (val > GKA_GAIN_MAX_Q15) {
        val = GKA_GAIN_MAX_Q15;
    }
    return (LONG)val;
}

VOID CMiniportWaveStream::Session(VOID)
{
    LARGE_INTEGER tick;
    NTSTATUS st = STATUS_UNSUCCESSFUL;
    BOOLEAN ok = FALSE;
    BOOLEAN ampOn = FALSE;
    ULONG   attempts;
    ULONGLONG drops = 0;
    ULONGLONG sent = 0;
    ULONGLONG statT = KeQueryInterruptTime();

    GkaLog("Worker: sesion RUN\n");
    tick.QuadPart = -(LONGLONG)GKA_POLL_MS * 10000;       /* relativo, 100 ns */

    /* 1. conectar con adspdrv (puede no estar listo justo despues del arranque) */
    for (attempts = 0; attempts < 60; attempts++) {
        LARGE_INTEGER half;
        st = AdspClientConnect();
        if (NT_SUCCESS(st)) {
            break;
        }
        if ((attempts % 4) == 0) {
            GkaLog("Worker: conexion con adspdrv intento %lu -> 0x%08x\n", attempts, st);
        }
        half.QuadPart = -5000000;                          /* 500 ms */
        if (KeWaitForSingleObject(&m_QuitEvt, Executive, KernelMode, FALSE, &half) == STATUS_SUCCESS) {
            return;
        }
        if (m_State != KSSTATE_RUN) {
            return;
        }
    }
    if (!NT_SUCCESS(st)) {
        GkaLog("Worker: adspdrv no disponible (0x%08x), el stream queda mudo\n", st);
    } else {
        /* 2. abrir el stream ASM/ADM del ADSP (bloquea hasta ~20 s) */
        st = AdspClientOpen();
        if (!NT_SUCCESS(st)) {
            GkaLog("Worker: IOCTL_ADSPAUDIO_OPEN fallo 0x%08x\n", st);
        } else {
            ok = TRUE;
            GkaLog("Worker: stream del ADSP abierto\n");
            /* el reloj I2S del AFE ya corre: encender el amplificador */
            st = AmpClientEnable();
            if (NT_SUCCESS(st)) {
                ampOn = TRUE;
            } else {
                GkaLog("Worker: no se pudo encender el amplificador 0x%08x\n", st);
            }
        }
    }

    if (ok) {
        /* 3. colchon de silencio y arranque desde la posicion actual */
        RtlZeroMemory(m_Tmp, GKA_PREROLL_BYTES);
        {
            ULONG acc = 0;
            (VOID)AdspClientWrite(m_Tmp, GKA_PREROLL_BYTES, &acc);
        }
        m_Copied = PositionNow();
        GkaLog("Worker: preroll enviado, copiando desde %I64u\n", m_Copied);
    }

    /* 4. bucle: copiar lo que Windows "ya consumio" segun nuestro reloj */
    for (;;) {
        if (KeWaitForSingleObject(&m_QuitEvt, Executive, KernelMode, FALSE, &tick) == STATUS_SUCCESS) {
            break;
        }
        if (m_State != KSSTATE_RUN || !m_Buf) {
            break;                      /* PAUSE/STOP o buffer liberado */
        }
        if (!ok || m_BufSize == 0) {
            continue;
        }

        {
            static const ULONGLONG kStatEvery = 2ull * 10000000ull;   /* 2 s en 100 ns */
            ULONGLONG nowT = KeQueryInterruptTime();
            if (nowT - statT >= kStatEvery) {
                ADSPAUDIO_STATUS ss;
                statT = nowT;
                if (NT_SUCCESS(AdspClientStatus(&ss))) {
                    GkaLog("Worker: enviados %I64u B, descartes %I64u, anillo %lu/%lu, subdesbordes %lu\n",
                           sent, drops, ss.RingSize - ss.RingFree, ss.RingSize, ss.Underruns);
                }
            }
        }

        ULONGLONG pos = PositionNow();
        ULONGLONG avail = pos - m_Copied;
        if (avail < GKA_BLOCK) {
            continue;
        }
        if (avail > m_BufSize) {                 /* nos quedamos atras mas que el buffer */
            m_Copied = pos - m_BufSize;
            avail = m_BufSize;
            drops++;
        }

        ADSPAUDIO_STATUS stt;
        if (NT_SUCCESS(AdspClientStatus(&stt)) &&
            (stt.RingSize - stt.RingFree) > GKA_RING_MAX_BYTES) {
            m_Copied = pos;                      /* anillo demasiado lleno: descartar */
            drops++;
            continue;
        }

        while (avail >= GKA_BLOCK) {
            ULONG off = (ULONG)(m_Copied % m_BufSize);
            ULONG n = (avail < GKA_TMP_BYTES) ? (ULONG)avail : GKA_TMP_BYTES;
            ULONG acc = 0;

            if (n > m_BufSize - off) n = m_BufSize - off;
            n -= n % GKA_BLOCK;
            if (n == 0) {
                break;
            }
            ApplyGain((const INT16 *)(m_Buf + off), (INT16 *)m_Tmp, n / sizeof(INT16), m_Gain);
            (VOID)AdspClientWrite(m_Tmp, n, &acc);
            m_Copied += n;
            avail -= n;
            sent += n;
            if (acc < n) {                       /* anillo lleno: el resto se pierde */
                drops++;
            }
        }
    }

    if (ampOn) {
        (VOID)AmpClientDisable();        /* silenciar antes de parar el reloj */
    }
    if (ok) {
        (VOID)AdspClientClose();
        GkaLog("Worker: cerrado (enviados %I64u bytes, descartes %I64u)\n", sent, drops);
    }
}

VOID CMiniportWaveStream::Worker(VOID)
{
    PVOID objs[2];

    objs[0] = &m_QuitEvt;
    objs[1] = &m_RunEvt;
    GkaLog("Worker: hilo creado\n");
    KeSetPriorityThread(KeGetCurrentThread(), LOW_REALTIME_PRIORITY);

    for (;;) {
        NTSTATUS w = KeWaitForMultipleObjects(2, objs, WaitAny, Executive, KernelMode,
                                              FALSE, NULL, NULL);
        if (w == STATUS_WAIT_0) {
            break;                      /* terminar */
        }
        if (m_State == KSSTATE_RUN && m_Buf) {
            InterlockedExchange(&m_InSession, 1);
            Session();
            InterlockedExchange(&m_InSession, 0);
        } else {
            /* RunEvt activo pero sin estado RUN o sin buffer: evitar vuelta en vacio */
            LARGE_INTEGER ms;
            ms.QuadPart = -100000;
            KeDelayExecutionThread(KernelMode, FALSE, &ms);
        }
    }
    GkaLog("Worker: hilo termina\n");
}
