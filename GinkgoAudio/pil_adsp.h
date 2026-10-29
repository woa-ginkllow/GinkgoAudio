/*
 * pil_adsp.h - cargador del ADSP (equivalente a pil_boot() de
 * drivers/soc/qcom/peripheral-loader.c del kernel sm6125).
 *
 * Secuencia (igual que pil_boot):
 *   1. validar adsp.mdt (ELF32) y calcular la region
 *   2. proxy vote (xo + vdd_cx), si el llamador lo provee
 *   3. PAS_INIT_IMAGE  con el .mdt completo
 *   4. PAS_MEM_SETUP   con (region_start, region_end - region_start)
 *   5. copiar cada segmento adsp.bNN y poner en cero hasta p_memsz
 *   6. PAS_AUTH_AND_RESET
 *
 * Despues de PilBootAdsp() el ADSP queda corriendo; la senal de exito es
 * "err-ready" por SMP2P (ese paso no esta aca).
 */
#pragma once

#include "scm.h"

/* Index = PIL_BLOB_MDT pide adsp.mdt; cualquier otro valor pide adsp.bNN */
#define PIL_BLOB_MDT   0xFFFFFFFFu

/*
 * Devuelve un puntero a los bytes del archivo y su tamano. El buffer tiene que
 * seguir valido hasta que PilBootAdsp() retorne. Como se leen los archivos de
 * la particion del telefono queda de tu lado.
 */
typedef NTSTATUS (*PIL_GET_BLOB)(PVOID Ctx, UINT32 Index,
                                 const VOID **Data, SIZE_T *Size);

typedef struct _PIL_ADSP_CONFIG {
    UINT32       PasId;        /* qcom,pas-id = 1                          */
    UINT64       RegionBase;   /* pil_adsp_region@55300000                 */
    UINT64       RegionSize;   /* 0x2200000                                */
    PIL_GET_BLOB GetBlob;
    PVOID        Ctx;
    /* Opcional: voto de xo y vdd_cx antes de INIT_IMAGE. */
    NTSTATUS   (*ProxyVote)(PVOID Ctx);
} PIL_ADSP_CONFIG;

NTSTATUS PilBootAdsp(const PIL_ADSP_CONFIG *Cfg);
