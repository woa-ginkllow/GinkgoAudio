/*
 * adsp_audio.h - interfaz entre adspdrv.sys (motor de PCM del ADSP) y quien
 * reproduce: la app de prueba o, despues, el miniport de audio.
 *
 * Formato fijo: PCM 16 bits, estereo, 48 kHz (lo que el AFE tiene configurado).
 * El dispositivo se publica como interfaz GUID_DEVINTERFACE_ADSP_AUDIO.
 */
#pragma once

/* En kernel incluir <ntddk.h> y <initguid.h> ANTES (initguid.h en un solo .c);
 * en usuario esta cabecera trae windows.h. */
#if !defined(_NTDDK_) && !defined(_WDMDDK_) && !defined(_WDM_INCLUDED_)
#include <windows.h>
#include <winioctl.h>      /* en kernel CTL_CODE ya viene de ntddk.h/wdm.h */
#endif

/* {D3A0C1B2-6A1F-4E2B-9C55-7B8A1E0F42D1} */
DEFINE_GUID(GUID_DEVINTERFACE_ADSP_AUDIO,
    0xd3a0c1b2, 0x6a1f, 0x4e2b, 0x9c, 0x55, 0x7b, 0x8a, 0x1e, 0x0f, 0x42, 0xd1);

/* OPEN: abre el stream y el camino ASM -> ADM -> AFE. Bloquea hasta ~20 s. */
#define IOCTL_ADSPAUDIO_OPEN   CTL_CODE(FILE_DEVICE_UNKNOWN, 0x800, METHOD_BUFFERED, FILE_ANY_ACCESS)
/* WRITE: entrada = bytes PCM; salida = ULONG con los bytes aceptados (no bloquea) */
#define IOCTL_ADSPAUDIO_WRITE  CTL_CODE(FILE_DEVICE_UNKNOWN, 0x801, METHOD_BUFFERED, FILE_ANY_ACCESS)
/* CLOSE: cierra stream y COPP */
#define IOCTL_ADSPAUDIO_CLOSE  CTL_CODE(FILE_DEVICE_UNKNOWN, 0x802, METHOD_BUFFERED, FILE_ANY_ACCESS)
/* STATUS: salida = ADSPAUDIO_STATUS */
#define IOCTL_ADSPAUDIO_STATUS CTL_CODE(FILE_DEVICE_UNKNOWN, 0x803, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ADSPAUDIO_STATE_IDLE     0
#define ADSPAUDIO_STATE_OPENING  1
#define ADSPAUDIO_STATE_RUNNING  2
#define ADSPAUDIO_STATE_CLOSING  3

#define ADSPAUDIO_CHUNK_BYTES    9600u      /* 50 ms: unidad de envio al ADSP */

typedef struct _ADSPAUDIO_STATUS {
    ULONG     State;            /* ADSPAUDIO_STATE_*                                  */
    ULONG     RingFree;         /* bytes libres en el anillo de entrada               */
    ULONG     RingSize;
    ULONG     Underruns;        /* trozos de silencio insertados por falta de datos   */
    ULONGLONG BytesQueued;      /* total aceptado por WRITE                           */
    ULONGLONG BytesPlayed;      /* total confirmado por el ADSP (incluye silencio)    */
} ADSPAUDIO_STATUS;
