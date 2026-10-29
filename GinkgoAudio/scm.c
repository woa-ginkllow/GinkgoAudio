/*
 * scm.c - llamadas SCM a TrustZone y comandos PAS (ver scm.h).
 */
#include "scm.h"

#define SCM_V2_EBUSY         (-12)   /* esperar y reintentar          */
#define SCM_EBUSY_WAIT_MS    30
#define SCM_EBUSY_MAX_RETRY  67

NTSTATUS ScmCall2(UINT32 FnId, UINT64 ArgInfo,
                  UINT64 A0, UINT64 A1, UINT64 A2,
                  INT32 *Status, UINT64 *Ret0)
{
    LARGE_INTEGER delay;
    ULONG attempt;

    /* intervalo relativo en unidades de 100 ns */
    delay.QuadPart = -(LONGLONG)SCM_EBUSY_WAIT_MS * 10000;

    for (attempt = 0; attempt <= SCM_EBUSY_MAX_RETRY; attempt++) {
        SCM_SMC_REGS regs;
        INT32 status;

        RtlZeroMemory(&regs, sizeof(regs));
        regs.X[0] = (UINT64)FnId | SCM_SMC64_MASK;
        regs.X[1] = ArgInfo;
        regs.X[2] = A0;
        regs.X[3] = A1;
        regs.X[4] = A2;
        /* x5 = 0: no hay buffer de argumentos extra (<= 3 argumentos) */

        ScmSmcCall(&regs);

        status = (INT32)regs.X[0];
        if (status == SCM_V2_EBUSY) {
            KeDelayExecutionThread(KernelMode, FALSE, &delay);
            continue;
        }

        if (Status != NULL) {
            *Status = status;
        }
        if (Ret0 != NULL) {
            *Ret0 = regs.X[1];
        }
        return (status == 0) ? STATUS_SUCCESS : STATUS_UNSUCCESSFUL;
    }

    SCM_LOG("SCM: fn 0x%08x sigue ocupado tras %u reintentos\n",
            FnId, (unsigned)SCM_EBUSY_MAX_RETRY);
    return STATUS_TIMEOUT;
}

static NTSTATUS PasResult(const char *Name, NTSTATUS CallStatus,
                          INT32 Status, UINT64 Ret0)
{
    if (!NT_SUCCESS(CallStatus)) {
        SCM_LOG("PAS %s: fallo SMC nt=0x%08x x0=%d x1=0x%llx\n",
                Name, (unsigned)CallStatus, (int)Status,
                (unsigned long long)Ret0);
        return CallStatus;
    }
    if (Ret0 != 0) {
        SCM_LOG("PAS %s: TrustZone devolvio x1=0x%llx\n",
                Name, (unsigned long long)Ret0);
        return STATUS_UNSUCCESSFUL;
    }
    return STATUS_SUCCESS;
}

/*
 * PAS_INIT_IMAGE: copia el .mdt completo a un buffer fisicamente contiguo sin
 * cache (el kernel usa DMA_ATTR_STRONGLY_ORDERED) y le pasa su direccion
 * fisica a TrustZone.
 */
NTSTATUS PasInitImage(UINT32 PasId, const VOID *Metadata, SIZE_T Size)
{
    PHYSICAL_ADDRESS low, high, boundary, pa;
    PVOID buf;
    NTSTATUS st;
    INT32 status = 0;
    UINT64 ret0 = 0;

    low.QuadPart = 0;
    high.QuadPart = 0xFFFFFFFFLL;      /* por debajo de 4 GiB */
    boundary.QuadPart = 0;

    buf = MmAllocateContiguousMemorySpecifyCache(Size, low, high, boundary,
                                                 MmNonCached);
    if (buf == NULL) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    RtlCopyMemory(buf, Metadata, Size);
    KeMemoryBarrier();
    pa = MmGetPhysicalAddress(buf);

    st = ScmCall2(SCM_SIP_FNID(SCM_SVC_PIL, PAS_INIT_IMAGE_CMD),
                  SCM_ARGINFO_VAL_RW,
                  PasId, (UINT64)pa.QuadPart, 0,
                  &status, &ret0);

    MmFreeContiguousMemorySpecifyCache(buf, Size, MmNonCached);
    return PasResult("INIT_IMAGE", st, status, ret0);
}

/* PAS_MEM_SETUP: args = proc, base fisica, tamano de la region del firmware */
NTSTATUS PasMemSetup(UINT32 PasId, UINT64 Base, UINT64 Size)
{
    INT32 status = 0;
    UINT64 ret0 = 0;
    NTSTATUS st;

    st = ScmCall2(SCM_SIP_FNID(SCM_SVC_PIL, PAS_MEM_SETUP_CMD),
                  SCM_ARGINFO_3_VAL,
                  PasId, Base, Size,
                  &status, &ret0);
    return PasResult("MEM_SETUP", st, status, ret0);
}

/* PAS_AUTH_AND_RESET: autentica la imagen ya cargada y saca al ADSP de reset */
NTSTATUS PasAuthAndReset(UINT32 PasId)
{
    INT32 status = 0;
    UINT64 ret0 = 0;
    NTSTATUS st;

    st = ScmCall2(SCM_SIP_FNID(SCM_SVC_PIL, PAS_AUTH_AND_RESET_CMD),
                  SCM_ARGINFO_1_VAL,
                  PasId, 0, 0,
                  &status, &ret0);
    return PasResult("AUTH_AND_RESET", st, status, ret0);
}

NTSTATUS PasShutdown(UINT32 PasId)
{
    INT32 status = 0;
    UINT64 ret0 = 0;
    NTSTATUS st;

    st = ScmCall2(SCM_SIP_FNID(SCM_SVC_PIL, PAS_SHUTDOWN_CMD),
                  SCM_ARGINFO_1_VAL,
                  PasId, 0, 0,
                  &status, &ret0);
    return PasResult("SHUTDOWN", st, status, ret0);
}
