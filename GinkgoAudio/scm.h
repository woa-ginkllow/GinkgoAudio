/*
 * scm.h - llamadas SCM (SMC) a TrustZone y comandos PAS para arrancar
 * subsistemas (ADSP) desde un driver de Windows ARM64.
 *
 * Valores tomados del kernel sm6125:
 *   include/soc/qcom/scm.h, drivers/soc/qcom/scm.c,
 *   drivers/soc/qcom/subsys-pil-tz.c y el nodo qcom,lpass@ab00000 de ginkgo.dts.
 */
#pragma once

#include <ntddk.h>

/*
 * Log de todo el stack del ADSP. AdspLog() esta en adsp_driver.c: escribe por
 * DbgPrint y, a PASSIVE_LEVEL, tambien a un archivo (ADSP_LOG_PATH).
 * Si usas estos archivos fuera de ese driver, define SCM_LOG antes de incluirlo.
 */
#ifndef SCM_LOG
VOID AdspLog(const char *Fmt, ...);
#define SCM_LOG(...) AdspLog(__VA_ARGS__)
#endif

/* Servicio y comandos PIL (subsys-pil-tz.c) */
#define SCM_SVC_PIL              0x2
#define PAS_INIT_IMAGE_CMD       1
#define PAS_MEM_SETUP_CMD        2
#define PAS_AUTH_AND_RESET_CMD   5
#define PAS_SHUTDOWN_CMD         6

/* qcom,pas-id del nodo qcom,lpass@ab00000 (ADSP) en ginkgo.dts */
#define PAS_ID_ADSP              1

/* Igual que SCM_SIP_FNID() de include/soc/qcom/scm.h */
#define SCM_SIP_FNID(s, c) \
    (((((s) & 0xFF) << 8) | ((c) & 0xFF)) | 0x02000000)

/*
 * scm_version_mask en arm64 (SMC64_MASK de scm.c).
 * TODO: confirmar el valor real en tu scm.c (se espera 0x40000000).
 */
#define SCM_SMC64_MASK           0x40000000ULL

/*
 * arginfo = SCM_ARGS(n, tipos...) = n | (tipo_i << (4 + 2*i))
 * con SCM_VAL = 0, SCM_RO = 1, SCM_RW = 2, SCM_BUFVAL = 3.
 */
#define SCM_ARGINFO_1_VAL        0x1ULL    /* AUTH_AND_RESET, SHUTDOWN       */
#define SCM_ARGINFO_3_VAL        0x3ULL    /* MEM_SETUP                      */
#define SCM_ARGINFO_VAL_RW       0x82ULL   /* INIT_IMAGE: 2 | (SCM_RW << 6)  */

/* x0..x6 de entrada y de salida del SMC */
typedef struct _SCM_SMC_REGS {
    UINT64 X[7];
} SCM_SMC_REGS;

/* Implementada en scm_smc.asm */
VOID ScmSmcCall(SCM_SMC_REGS *Regs);

/*
 * Llamada estándar. Devuelve STATUS_SUCCESS si el SMC se ejecutó y TrustZone
 * respondió con x0 == 0; *Status recibe x0 y *Ret0 recibe x1.
 */
NTSTATUS ScmCall2(UINT32 FnId, UINT64 ArgInfo,
                  UINT64 A0, UINT64 A1, UINT64 A2,
                  INT32 *Status, UINT64 *Ret0);

/*
 * Comandos PAS. Todos devuelven STATUS_SUCCESS solo si x0 == 0 y x1 == 0
 * (como pil-tz: "if (ret) return ret; return scm_ret;").
 * Llamar a PASSIVE_LEVEL.
 */
NTSTATUS PasInitImage(UINT32 PasId, const VOID *Metadata, SIZE_T Size);
NTSTATUS PasMemSetup(UINT32 PasId, UINT64 Base, UINT64 Size);
NTSTATUS PasAuthAndReset(UINT32 PasId);
NTSTATUS PasShutdown(UINT32 PasId);
