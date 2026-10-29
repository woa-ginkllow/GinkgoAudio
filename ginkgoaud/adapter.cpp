/*
 * adapter.cpp - DriverEntry / AddDevice / StartDevice del miniport de audio.
 *
 * Driver WDM de PortCls (sin KMDF). Se enumera por ACPI (ACPI\GNKAUD01, ver
 * ginkgoaud_acpi.asl) y publica un endpoint de render "Altavoz" cuyo audio
 * sale por adspdrv.sys.
 */
#define PUT_GUIDS_HERE
#include "ginkgoaud.h"

static PDRIVER_UNLOAD g_PcUnload = NULL;

/* ------------------------------------------------------------------ */
/* new / delete                                                       */
/* ------------------------------------------------------------------ */
PVOID operator new(size_t Size, POOL_FLAGS Flags, ULONG Tag)
{
    return ExAllocatePool2(Flags, Size, Tag);
}
/* operator delete(PVOID) ya lo define stdunk.lib (usa ExFreePool): no duplicarlo */
void __cdecl operator delete(PVOID P, size_t)           { if (P) ExFreePoolWithTag(P, GKA_TAG); }
void __cdecl operator delete[](PVOID P)                 { if (P) ExFreePoolWithTag(P, GKA_TAG); }
void __cdecl operator delete[](PVOID P, size_t)         { if (P) ExFreePoolWithTag(P, GKA_TAG); }

/* ------------------------------------------------------------------ */
/* Registro (funciona a cualquier IRQL)                               */
/*   PASSIVE_LEVEL: escribe directo al archivo.                       */
/*   IRQL alto: deja el texto en un buffer y un work item lo escribe. */
/* ------------------------------------------------------------------ */
static KSPIN_LOCK    g_LogLock;
static char          g_Pend[8192];
static ULONG         g_PendLen = 0;
static PIO_WORKITEM  g_LogWi = NULL;
static volatile LONG g_LogWiQueued = 0;

static VOID LogWriteFile(_In_reads_bytes_(Len) const char *Buf, _In_ ULONG Len)
{
    UNICODE_STRING name;
    OBJECT_ATTRIBUTES oa;
    HANDLE h = NULL;
    IO_STATUS_BLOCK iosb;

    RtlInitUnicodeString(&name, L"\\SystemRoot\\ginkgoaud_log.txt");
    InitializeObjectAttributes(&oa, &name, OBJ_KERNEL_HANDLE | OBJ_CASE_INSENSITIVE, NULL, NULL);
    if (NT_SUCCESS(ZwCreateFile(&h, FILE_APPEND_DATA | SYNCHRONIZE, &oa, &iosb, NULL,
                                FILE_ATTRIBUTE_NORMAL, FILE_SHARE_READ | FILE_SHARE_WRITE,
                                FILE_OPEN_IF,
                                FILE_SYNCHRONOUS_IO_NONALERT | FILE_NON_DIRECTORY_FILE | FILE_WRITE_THROUGH,
                                NULL, 0))) {
        (VOID)ZwWriteFile(h, NULL, NULL, NULL, &iosb, (PVOID)Buf, Len, NULL, NULL);
        ZwClose(h);
    }
}

static VOID LogDrain(VOID)         /* solo en PASSIVE_LEVEL */
{
    char chunk[512];

    for (;;) {
        KIRQL irql;
        ULONG n;

        KeAcquireSpinLock(&g_LogLock, &irql);
        n = g_PendLen < sizeof(chunk) ? g_PendLen : (ULONG)sizeof(chunk);
        if (n) {
            RtlCopyMemory(chunk, g_Pend, n);
            RtlMoveMemory(g_Pend, g_Pend + n, g_PendLen - n);
            g_PendLen -= n;
        }
        KeReleaseSpinLock(&g_LogLock, irql);
        if (!n) {
            break;
        }
        LogWriteFile(chunk, n);
    }
}

static IO_WORKITEM_ROUTINE LogWorkRoutine;
static VOID NTAPI LogWorkRoutine(_In_ PDEVICE_OBJECT DeviceObject, _In_opt_ PVOID Context)
{
    UNREFERENCED_PARAMETER(DeviceObject);
    UNREFERENCED_PARAMETER(Context);
    InterlockedExchange(&g_LogWiQueued, 0);
    LogDrain();
}

VOID GkaLogInit(_In_opt_ PDEVICE_OBJECT DeviceObject)
{
    static BOOLEAN lockInit = FALSE;
    if (!lockInit) {
        KeInitializeSpinLock(&g_LogLock);
        lockInit = TRUE;
    }
    if (DeviceObject && !g_LogWi) {
        g_LogWi = IoAllocateWorkItem(DeviceObject);
    }
}

VOID GkaLogFlush(VOID)
{
    if (KeGetCurrentIrql() == PASSIVE_LEVEL) {
        LogDrain();
    }
    if (g_LogWi) {
        IoFreeWorkItem(g_LogWi);
        g_LogWi = NULL;
    }
}

VOID GkaLog(_In_z_ const char *Fmt, ...)
{
    char buf[272];
    char msg[256];
    va_list ap;
    size_t len = 0;
    ULONGLONG t = KeQueryInterruptTime() / 10000;      /* ms desde el arranque */

    va_start(ap, Fmt);
    if (!NT_SUCCESS(RtlStringCbVPrintfA(msg, sizeof(msg), Fmt, ap))) {
        va_end(ap);
        return;
    }
    va_end(ap);
    if (!NT_SUCCESS(RtlStringCbPrintfA(buf, sizeof(buf), "[%I64u.%03I64u] %s",
                                       t / 1000, t % 1000, msg))) {
        return;
    }
    DbgPrint("GKA: %s", buf);
    if (!NT_SUCCESS(RtlStringCbLengthA(buf, sizeof(buf), &len))) {
        return;
    }

    if (KeGetCurrentIrql() == PASSIVE_LEVEL) {
        LogDrain();
        LogWriteFile(buf, (ULONG)len);
        return;
    }

    {
        KIRQL irql;
        KeAcquireSpinLock(&g_LogLock, &irql);
        if (g_PendLen + len <= sizeof(g_Pend)) {
            RtlCopyMemory(g_Pend + g_PendLen, buf, len);
            g_PendLen += (ULONG)len;
        }
        KeReleaseSpinLock(&g_LogLock, irql);
    }
    if (g_LogWi && InterlockedExchange(&g_LogWiQueued, 1) == 0) {
        IoQueueWorkItem(g_LogWi, LogWorkRoutine, DelayedWorkQueue, NULL);
    }
}

/* ------------------------------------------------------------------ */
/* StartDevice: crea y registra los dos filtros y los conecta         */
/* ------------------------------------------------------------------ */
static NTSTATUS InstallFilter(_In_ PDEVICE_OBJECT DeviceObject, _In_ PIRP Irp,
                              _In_ PRESOURCELIST ResourceList,
                              _In_ REFCLSID PortClass, _In_ PUNKNOWN Miniport,
                              _In_z_ PCWSTR Name, _Outptr_ PPORT *OutPort)
{
    PPORT port = NULL;
    NTSTATUS st;

    *OutPort = NULL;

    st = PcNewPort(&port, PortClass);
    if (!NT_SUCCESS(st)) {
        GkaLog("PcNewPort(%ws) fallo 0x%08x\n", Name, st);
        return st;
    }

    st = port->Init(DeviceObject, Irp, Miniport, NULL, ResourceList);
    if (!NT_SUCCESS(st)) {
        GkaLog("port->Init(%ws) fallo 0x%08x\n", Name, st);
        port->Release();
        return st;
    }

    st = PcRegisterSubdevice(DeviceObject, (PWSTR)Name, port);
    if (!NT_SUCCESS(st)) {
        GkaLog("PcRegisterSubdevice(%ws) fallo 0x%08x\n", Name, st);
        port->Release();
        return st;
    }

    *OutPort = port;      /* el llamador libera su referencia */
    return STATUS_SUCCESS;
}

static NTSTATUS StartDevice(_In_ PDEVICE_OBJECT DeviceObject, _In_ PIRP Irp,
                            _In_ PRESOURCELIST ResourceList)
{
    PUNKNOWN miniWave = NULL, miniTopo = NULL;
    PPORT portWave = NULL, portTopo = NULL;
    NTSTATUS st;

    GkaLogInit(DeviceObject);
    GkaLog("StartDevice\n");

    st = CreateMiniportWave(&miniWave);
    if (!NT_SUCCESS(st)) goto Done;
    st = CreateMiniportTopo(&miniTopo);
    if (!NT_SUCCESS(st)) goto Done;

    st = InstallFilter(DeviceObject, Irp, ResourceList, CLSID_PortWaveRT,
                       miniWave, L"WaveSpeaker", &portWave);
    if (!NT_SUCCESS(st)) goto Done;

    st = InstallFilter(DeviceObject, Irp, ResourceList, CLSID_PortTopology,
                       miniTopo, L"TopologySpeaker", &portTopo);
    if (!NT_SUCCESS(st)) goto Done;

    /* pin 1 (puente) del wave  ->  pin 0 de la topologia */
    st = PcRegisterPhysicalConnection(DeviceObject, (PUNKNOWN)portWave, 1,
                                      (PUNKNOWN)portTopo, 0);
    if (!NT_SUCCESS(st)) {
        GkaLog("PcRegisterPhysicalConnection fallo 0x%08x\n", st);
    } else {
        GkaLog("StartDevice OK: WaveSpeaker + TopologySpeaker registrados\n");
    }

Done:
    if (portWave) portWave->Release();
    if (portTopo) portTopo->Release();
    if (miniWave) miniWave->Release();
    if (miniTopo) miniTopo->Release();
    return st;
}

static NTSTATUS AddDevice(_In_ PDRIVER_OBJECT DriverObject,
                          _In_ PDEVICE_OBJECT PhysicalDeviceObject)
{
    /* 2 subdispositivos (wave y topologia); un par de reserva */
    return PcAddAdapterDevice(DriverObject, PhysicalDeviceObject,
                              PCPFNSTARTDEVICE(StartDevice), 4, 0);
}

static VOID DriverUnloadHook(_In_ PDRIVER_OBJECT DriverObject)
{
    GkaLogFlush();
    AdspClientDisconnect();
    if (g_PcUnload) {
        g_PcUnload(DriverObject);
    }
}

extern "C" DRIVER_INITIALIZE DriverEntry;
extern "C" NTSTATUS DriverEntry(_In_ PDRIVER_OBJECT DriverObject,
                                _In_ PUNICODE_STRING RegistryPathName)
{
    GkaLogInit(NULL);
    NTSTATUS st = PcInitializeAdapterDriver(DriverObject, RegistryPathName,
                                            (PDRIVER_ADD_DEVICE)AddDevice);
    if (NT_SUCCESS(st)) {
        g_PcUnload = DriverObject->DriverUnload;
        DriverObject->DriverUnload = DriverUnloadHook;
    }
    GkaLog("DriverEntry 0x%08x  [adapter.cpp %s]\n", st, GKA_BUILD_ID);
    return st;
}
