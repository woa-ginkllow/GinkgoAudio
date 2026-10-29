/*
 * adspclient.cpp - habla con adspdrv.sys y ampi2c.sys desde el kernel.
 *
 * Busca cada interfaz GUID, abre el dispositivo con IoGetDeviceObjectPointer
 * (acceso de kernel: no pasa por ninguna ACL) y manda los IOCTL de forma
 * sincrona. Todo en PASSIVE_LEVEL.
 */
#define GKA_INSTANTIATE_GUID           /* instancia los GUID de las interfaces */
#include "ginkgoaud.h"

typedef struct _LINK {
    const GUID    *Guid;
    const char    *Name;
    PFILE_OBJECT   File;
    PDEVICE_OBJECT Dev;
    LONG           LogCount;
} LINK;

static LINK g_Adsp = { &GUID_DEVINTERFACE_ADSP_AUDIO, "adspdrv", NULL, NULL, 0 };
static LINK g_Amp  = { &GUID_DEVINTERFACE_GKA_AMP,    "ampi2c",  NULL, NULL, 0 };

static FAST_MUTEX g_Lock;
static BOOLEAN    g_LockInit = FALSE;

static VOID EnsureLock(VOID)
{
    /* DriverEntry y el primer stream corren en PASSIVE; no hay carrera real */
    if (!g_LockInit) {
        ExInitializeFastMutex(&g_Lock);
        g_LockInit = TRUE;
    }
}

static NTSTATUS LinkConnect(LINK *L)
{
    PWSTR list = NULL;
    UNICODE_STRING name;
    NTSTATUS st;

    EnsureLock();
    ExAcquireFastMutex(&g_Lock);
    if (L->Dev) {
        ExReleaseFastMutex(&g_Lock);
        return STATUS_SUCCESS;
    }

    st = IoGetDeviceInterfaces(L->Guid, NULL, 0, &list);
    if (InterlockedIncrement(&L->LogCount) <= 3) {
        GkaLog("IoGetDeviceInterfaces(%s): st=0x%08x lista=%s\n", L->Name, st,
               (list && list[0]) ? "con interfaz" : "vacia");
    }
    if (!NT_SUCCESS(st) || list == NULL || list[0] == L'\0') {
        if (list) ExFreePool(list);
        ExReleaseFastMutex(&g_Lock);
        return STATUS_OBJECT_NAME_NOT_FOUND;
    }

    RtlInitUnicodeString(&name, list);
    st = IoGetDeviceObjectPointer(&name, FILE_ALL_ACCESS, &L->File, &L->Dev);
    ExFreePool(list);
    if (!NT_SUCCESS(st)) {
        L->File = NULL;
        L->Dev = NULL;
        GkaLog("IoGetDeviceObjectPointer(%s) fallo 0x%08x\n", L->Name, st);
    } else {
        GkaLog("conectado a %s\n", L->Name);
    }
    ExReleaseFastMutex(&g_Lock);
    return st;
}

static VOID LinkDisconnect(LINK *L)
{
    EnsureLock();
    ExAcquireFastMutex(&g_Lock);
    if (L->File) {
        ObDereferenceObject(L->File);
    }
    L->File = NULL;
    L->Dev = NULL;
    ExReleaseFastMutex(&g_Lock);
}

static NTSTATUS Ioctl(LINK *L, _In_ ULONG Code, _In_opt_ PVOID In, _In_ ULONG InLen,
                      _Out_opt_ PVOID Out, _In_ ULONG OutLen)
{
    KEVENT ev;
    IO_STATUS_BLOCK iosb;
    PIRP irp;
    NTSTATUS st;

    if (!L->Dev) {
        return STATUS_DEVICE_NOT_CONNECTED;
    }

    KeInitializeEvent(&ev, NotificationEvent, FALSE);
    irp = IoBuildDeviceIoControlRequest(Code, L->Dev, In, InLen, Out, OutLen,
                                        FALSE, &ev, &iosb);
    if (!irp) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    IoGetNextIrpStackLocation(irp)->FileObject = L->File;

    st = IoCallDriver(L->Dev, irp);
    if (st == STATUS_PENDING) {
        KeWaitForSingleObject(&ev, Executive, KernelMode, FALSE, NULL);
        st = iosb.Status;
    }
    return st;
}

/* ---- adspdrv ---- */
NTSTATUS AdspClientConnect(VOID) { return LinkConnect(&g_Adsp); }

VOID AdspClientDisconnect(VOID)
{
    LinkDisconnect(&g_Adsp);
    LinkDisconnect(&g_Amp);
}

NTSTATUS AdspClientOpen(VOID)  { return Ioctl(&g_Adsp, IOCTL_ADSPAUDIO_OPEN,  NULL, 0, NULL, 0); }
NTSTATUS AdspClientClose(VOID) { return Ioctl(&g_Adsp, IOCTL_ADSPAUDIO_CLOSE, NULL, 0, NULL, 0); }

NTSTATUS AdspClientWrite(_In_reads_bytes_(Len) const VOID *Data, _In_ ULONG Len, _Out_ ULONG *Accepted)
{
    ULONG acc = 0;
    NTSTATUS st = Ioctl(&g_Adsp, IOCTL_ADSPAUDIO_WRITE, (PVOID)Data, Len, &acc, sizeof(acc));
    *Accepted = NT_SUCCESS(st) ? acc : 0;
    return st;
}

NTSTATUS AdspClientStatus(_Out_ ADSPAUDIO_STATUS *Status)
{
    RtlZeroMemory(Status, sizeof(*Status));
    return Ioctl(&g_Adsp, IOCTL_ADSPAUDIO_STATUS, NULL, 0, Status, sizeof(*Status));
}

/* ---- ampi2c ---- */
NTSTATUS AmpClientEnable(VOID)
{
    NTSTATUS st = LinkConnect(&g_Amp);
    return NT_SUCCESS(st) ? Ioctl(&g_Amp, IOCTL_GKAAMP_ENABLE, NULL, 0, NULL, 0) : st;
}

NTSTATUS AmpClientDisable(VOID)
{
    NTSTATUS st = LinkConnect(&g_Amp);
    return NT_SUCCESS(st) ? Ioctl(&g_Amp, IOCTL_GKAAMP_DISABLE, NULL, 0, NULL, 0) : st;
}
