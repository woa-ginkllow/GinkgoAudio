/*
 * gka_amp.h - interfaz entre ampi2c.sys (MAX98937) y ginkgoaud.sys.
 * ENABLE enciende el amplificador (el reloj I2S del AFE ya tiene que estar
 * corriendo); DISABLE lo apaga. Sin buffers.
 */
#pragma once

#if !defined(_NTDDK_) && !defined(_WDMDDK_) && !defined(_WDM_INCLUDED_)
#include <windows.h>
#include <winioctl.h>
#endif

/* {7C5A4E21-3B8D-4F62-A1D9-0E6B2C94F7A3} */
DEFINE_GUID(GUID_DEVINTERFACE_GKA_AMP,
    0x7c5a4e21, 0x3b8d, 0x4f62, 0xa1, 0xd9, 0x0e, 0x6b, 0x2c, 0x94, 0xf7, 0xa3);

#define IOCTL_GKAAMP_ENABLE   CTL_CODE(FILE_DEVICE_UNKNOWN, 0x810, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_GKAAMP_DISABLE  CTL_CODE(FILE_DEVICE_UNKNOWN, 0x811, METHOD_BUFFERED, FILE_ANY_ACCESS)
