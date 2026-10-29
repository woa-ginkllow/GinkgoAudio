/*
 * ginkgoaud.h - miniport de audio PortCls (WaveRT) para el altavoz interno del
 * Redmi Note 8. Reproduce a traves de adspdrv.sys (ASM/ADM/AFE + MAX98937).
 */
#pragma once

#define POOL_ZERO_DOWN_LEVEL_SUPPORT   /* ExAllocatePool2 tambien en Windows 10 anteriores a 2004 */
#define PC_IMPLEMENTATION
#define _NEW_DELETE_OPERATORS_   /* stdunk.h no define new/delete: los damos nosotros (adapter.cpp) */

#include <portcls.h>
#include <stdunk.h>
#include <ksdebug.h>
#include <ntstrsafe.h>
#include <stdarg.h>

#ifndef SIZEOF_ARRAY
#define SIZEOF_ARRAY(a) (sizeof(a) / sizeof((a)[0]))
#endif

/* Marca de version: subirla en cada entrega para ver en el log que binario corre. */
#define GKA_BUILD_ID           "v9-2026-10-28"

#define GKA_TAG                'aKNG'

/* Formato unico: el que tiene configurado el AFE del ADSP. */
#define GKA_RATE               48000u
#define GKA_CHANNELS           2u
#define GKA_BITS               16u
#define GKA_BLOCK              4u              /* bytes por frame */
#define GKA_BYTES_PER_SEC      192000u

/* Ganancia de proteccion del altavoz, Q15 (32768 = 0 dB, 16384 = -6 dB).
 * Se lee del registro al crear cada stream:
 *   HKLM\SYSTEM\CurrentControlSet\Services\ginkgoaud\Parameters\GainQ15 (REG_DWORD)
 * Si no existe se usa GKA_GAIN_DEFAULT_Q15. El valor leido se limita SIEMPRE a
 * GKA_GAIN_MAX_Q15 (~ -1 dB): el MAX98937 es potente y el altavoz del telefono
 * pequeno, asi que ningun valor del registro puede pasar de ahi sin recompilar. */
#define GKA_GAIN_DEFAULT_Q15   16384
#define GKA_GAIN_MAX_Q15       29000

/* Silencio que se manda al ADSP antes de los datos reales. Absorbe la
 * diferencia de reloj entre el temporizador de Windows y el del ADSP y que el
 * motor de PCM trabaja en trozos de 50 ms (9600 bytes). 19200 = 100 ms. */
#define GKA_PREROLL_BYTES      19200u

/* Si el anillo de adspdrv pasa de esto (~250 ms) se descartan datos para no
 * acumular retraso. */
#define GKA_RING_MAX_BYTES     48000u

/* Periodo del hilo de reproduccion. */
#define GKA_POLL_MS            5

/* Registro en \SystemRoot\ginkgoaud_log.txt (solo en PASSIVE_LEVEL). */
VOID GkaLogInit(_In_opt_ PDEVICE_OBJECT DeviceObject);
VOID GkaLog(_In_z_ const char *Fmt, ...);
VOID GkaLogFlush(VOID);       /* vuelca al archivo lo registrado en IRQL alto */

/* ---- new/delete con etiqueta (los pide stdunk.h) ---- */
PVOID operator new(size_t Size, POOL_FLAGS Flags, ULONG Tag);
void __cdecl operator delete(PVOID P, size_t Size);
void __cdecl operator delete[](PVOID P);
void __cdecl operator delete[](PVOID P, size_t Size);

/* ---- cliente de adspdrv (adspclient.cpp) ---- */
/* initguid.h va justo antes para que el GUID de adsp_audio.h se instancie
 * solo en el .cpp que define GKA_INSTANTIATE_GUID (adspclient.cpp). */
#ifdef GKA_INSTANTIATE_GUID
#include <initguid.h>
#endif
#include "adsp_audio.h"
#include "gka_amp.h"
NTSTATUS AdspClientConnect(VOID);
VOID     AdspClientDisconnect(VOID);
NTSTATUS AdspClientOpen(VOID);
NTSTATUS AdspClientClose(VOID);
NTSTATUS AdspClientWrite(_In_reads_bytes_(Len) const VOID *Data, _In_ ULONG Len, _Out_ ULONG *Accepted);
NTSTATUS AdspClientStatus(_Out_ ADSPAUDIO_STATUS *Status);
NTSTATUS AmpClientEnable(VOID);     /* enciende el MAX98937 (ampi2c) */
NTSTATUS AmpClientDisable(VOID);

/* ---- creadores de miniports (wave.cpp / topo.cpp) ---- */
NTSTATUS CreateMiniportWave(_Out_ PUNKNOWN *Unknown);
NTSTATUS CreateMiniportTopo(_Out_ PUNKNOWN *Unknown);
