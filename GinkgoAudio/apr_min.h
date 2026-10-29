/*
 * apr_min.h - lo minimo de APR para el primer intercambio con el ADSP.
 *
 * !!! VERIFICAR contra include/ipc/apr.h y dsp/q6core.h de tu kernel !!!
 * Estos valores estan escritos de memoria (audio-kernel de Qualcomm, APR v2);
 * si algun numero no coincide, el ADSP contestara con un error APR o no
 * contestara, pero el transporte GLINK sigue siendo valido. El log vuelca los
 * paquetes en crudo para poder compararlos.
 *
 * Formato del paquete (struct apr_hdr, 20 bytes, little endian):
 *   u16 hdr_field   = (msg_type & 3) << 8 | (hdr_len/4 & 0xF) << 4 | ver & 0xF
 *   u16 pkt_size    = cabecera + payload
 *   u8  src_svc, u8 src_domain, u16 src_port
 *   u8  dest_svc, u8 dest_domain, u16 dest_port
 *   u32 token, u32 opcode
 * apr_send_pkt() pone src_svc = dest_svc = id del servicio.
 */
#pragma once

#include <ntddk.h>

#pragma pack(push, 1)
typedef struct _APR_HDR {
    UINT16 HdrField;
    UINT16 PktSize;
    UINT8  SrcSvc;
    UINT8  SrcDomain;
    UINT16 SrcPort;
    UINT8  DestSvc;
    UINT8  DestDomain;
    UINT16 DestPort;
    UINT32 Token;
    UINT32 Opcode;
} APR_HDR;
#pragma pack(pop)

#define APR_HDR_SIZE            20u

/* tipos de mensaje (APR_MSG_TYPE_*) */
#define APR_MSG_TYPE_EVENT      0x0
#define APR_MSG_TYPE_CMD_RSP    0x1
#define APR_MSG_TYPE_SEQ_CMD    0x2
#define APR_MSG_TYPE_NSEQ_CMD   0x3
#define APR_PKT_VER             0x0

/* dominios (APR_DOMAIN_*) */
#define APR_DOMAIN_MODEM        0x3
#define APR_DOMAIN_ADSP         0x4
#define APR_DOMAIN_APPS         0x5

/* servicios (APR_SVC_*) */
#define APR_SVC_ADSP_CORE       0x3
#define APR_SVC_AFE             0x4
#define APR_SVC_ASM             0x7
#define APR_SVC_ADM             0x8

/* opcodes */
#define APR_BASIC_RSP_RESULT         0x000110E8u
#define AVCS_CMD_GET_FWK_VERSION     0x0001292Cu   /* apr_audio-v2.h:10898 */
#define AVCS_CMDRSP_GET_FWK_VERSION  0x0001292Du   /* (sin verificar) */

#define APR_HDR_FIELD(type, len, ver) \
    ((UINT16)((((type) & 0x3) << 8) | ((((len) / 4) & 0xF) << 4) | ((ver) & 0xF)))

static __forceinline VOID AprInitHdr(APR_HDR *H, UINT8 Svc, UINT32 PayloadLen,
                                     UINT32 Opcode, UINT32 Token)
{
    H->HdrField   = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD, APR_HDR_SIZE, APR_PKT_VER);
    H->PktSize    = (UINT16)(APR_HDR_SIZE + PayloadLen);
    H->SrcSvc     = Svc;
    H->SrcDomain  = APR_DOMAIN_APPS;
    H->SrcPort    = 0;
    H->DestSvc    = Svc;
    H->DestDomain = APR_DOMAIN_ADSP;
    H->DestPort   = 0;
    H->Token      = Token;
    H->Opcode     = Opcode;
}

/* ---- AFE (apr_audio-v2.h) ------------------------------------------------ */
#define AFE_PORT_CMD_SET_PARAM_V2    0x000100EFu   /* :1976 */
#define AFE_SVC_CMD_SET_PARAM        0x000100F3u
#define AFE_PORT_CMD_DEVICE_START    0x000100E5u   /* :4750 */
#define AFE_PORT_CMD_DEVICE_STOP     0x000100E6u
#define AFE_PORT_ID_PRIMARY_MI2S_RX  0x1000u       /* :1439 */

#define AFE_MODULE_AUDIO_DEV_INTERFACE 0x0001020Cu
#define AFE_PARAM_ID_I2S_CONFIG        0x0001020Du
#define AFE_MODULE_CLOCK_SET           0x0001028Fu
#define AFE_PARAM_ID_CLOCK_SET         0x00010290u
#define Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT 0x100u
#define Q6AFE_LPASS_IBIT_CLK_1_P536_MHZ  0x177000u
#define Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_NO 0x1u

#define AFE_PORT_I2S_SD0             0x1u
#define AFE_PORT_I2S_SD1             0x2u
#define AFE_PORT_I2S_QUAD01          0x5u
#define AFE_PORT_I2S_STEREO          0x1u
#define AFE_PORT_CONFIG_I2S_WS_SRC_INTERNAL 0x1u
#define AFE_LINEAR_PCM_DATA          0x0u

#pragma pack(push, 1)
typedef struct _AFE_SET_PARAM_HDR {      /* afe_port_cmd_set_param_v2 sin apr_hdr */
    UINT16 PortId;
    UINT16 PayloadSize;                  /* param_hdr_v1 + datos */
    UINT32 DataAddrLsw, DataAddrMsw, MemMapHandle;   /* 0 = in-band */
    UINT32 ModuleId;                     /* param_hdr_v1 */
    UINT32 ParamId;
    UINT16 ParamSize;
    UINT16 Reserved;
} AFE_SET_PARAM_HDR;

typedef struct _AFE_CLK_SET {            /* struct afe_clk_set (:11369) */
    UINT32 MinorVersion;                 /* 1 */
    UINT32 ClkId;
    UINT32 FreqHz;
    UINT16 Attri;
    UINT16 Root;
    UINT32 Enable;
} AFE_CLK_SET;

typedef struct _AFE_I2S_CFG {            /* afe_param_id_i2s_cfg (:2544) */
    UINT32 MinorVersion;                 /* 1 */
    UINT16 BitWidth;
    UINT16 ChannelMode;
    UINT16 MonoStereo;
    UINT16 WsSrc;
    UINT32 SampleRate;
    UINT16 DataFormat;
    UINT16 Reserved;
} AFE_I2S_CFG;

typedef struct _AFE_DEVICE_START {
    UINT16 PortId;
    UINT16 Reserved;
} AFE_DEVICE_START;
#pragma pack(pop)

/*
 * Cabecera de 32 bytes, version 1 (la que usa el ADSP en sus respuestas:
 * hdr_field 0x0181). Los 12 bytes extra van a cero. Cualquier comando con la
 * cabecera de 20 bytes/ver 0 recibio status 0x16 (ni siquiera GET_FWK_VERSION).
 * APR_TX_V1 = 0 vuelve a la cabecera vieja para comparar.
 */
#define APR_TX_V1   1

static __forceinline UINT32 AprBuildPkt(UINT8 *Out, UINT8 Svc, const VOID *Pl,
                                        UINT32 PlLen, UINT32 Opcode, UINT32 Token)
{
    APR_HDR h;
    UINT32 hl = APR_TX_V1 ? 32u : APR_HDR_SIZE;

    AprInitHdr(&h, Svc, PlLen, Opcode, Token);
    h.HdrField = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD, hl, APR_TX_V1 ? 1 : 0);
    h.PktSize  = (UINT16)(hl + PlLen);
    RtlZeroMemory(Out, hl);
    RtlCopyMemory(Out, &h, APR_HDR_SIZE);
    if (PlLen) RtlCopyMemory(Out + hl, Pl, PlLen);
    return hl + PlLen;
}

/* Generador de tonos del AFE (apr_audio-v2.h:11502): sale directo por el puerto,
 * sin ASM/ADM. duration_in_ms: -1 continuo, 0 apagar. */
#define AFE_PORTS_CMD_DTMF_CTL       0x00010102u
#pragma pack(push, 1)
typedef struct _AFE_DTMF {
    INT64  DurationMs;
    UINT16 HighFreq;
    UINT16 LowFreq;
    UINT16 Gain;
    UINT16 NumPorts;
    UINT16 PortId;
    UINT16 Reserved;
} AFE_DTMF;
#pragma pack(pop)

/* prueba: tono de 1 kHz, bajo, 3 s */
#define ADSP_TONE_FREQ   1000
#define ADSP_TONE_GAIN   0x1000
#define ADSP_TONE_MS     3000

/* ---- ASM / ADM: reproduccion de PCM (apr_audio-v2.h) ---------------------- */
#define ASM_CMD_SHARED_MEM_MAP_REGIONS    0x00010D92u
#define ASM_CMDRSP_SHARED_MEM_MAP_REGIONS 0x00010D93u
#define ASM_STREAM_CMD_OPEN_WRITE_V3      0x00010DB3u
#define ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2  0x00010D98u
#define ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V2 0x00010DA5u
#define ASM_SESSION_CMD_RUN_V2            0x00010DAAu
#define ASM_DATA_CMD_WRITE_V2             0x00010DABu
#define ASM_DATA_EVENT_WRITE_DONE_V2      0x00010D99u
#define ASM_STREAM_CMD_CLOSE              0x00010BCDu
#define ADM_CMD_DEVICE_OPEN_V5            0x00010326u
#define ADM_CMDRSP_DEVICE_OPEN_V5         0x00010329u
#define ADM_CMD_MATRIX_MAP_ROUTINGS_V5    0x00010325u
#define ADM_CMD_DEVICE_CLOSE_V5           0x00010327u
#define ADM_MATRIX_ID_AUDIO_RX            0u
#define ADSP_MEMORY_MAP_SHMEM8_4K_POOL    3u
#define NULL_POPP_TOPOLOGY                0x00010C68u
#define ADM_TOPOLOGY_NONE_AUDIO_COPP      0x00010312u
#define PCM_CHANNEL_FL                    1u
#define PCM_CHANNEL_FR                    2u

#pragma pack(push, 1)
typedef struct _AVS_MAP_REGIONS {          /* avs_cmd_shared_mem_map_regions + 1 region */
    UINT16 PoolId, NumRegions;
    UINT32 PropertyFlag;
    UINT32 AddrLsw, AddrMsw, SizeBytes;
} AVS_MAP_REGIONS;
typedef struct _ASM_OPEN_WRITE_V3 {
    UINT32 ModeFlags;
    UINT16 SinkEndpoint;
    UINT16 BitsPerSample;
    UINT32 PostprocTopo;
    UINT32 DecFmtId;
} ASM_OPEN_WRITE_V3;
typedef struct _ASM_PCM_FMT_V2 {           /* fmt_blk_size + cuerpo */
    UINT32 FmtBlkSize;
    UINT16 NumChannels, BitsPerSample;
    UINT32 SampleRate;
    UINT16 IsSigned, Reserved;
    UINT8  ChannelMap[8];
} ASM_PCM_FMT_V2;
typedef struct _ASM_RUN_V2 { UINT32 Flags, TimeLsw, TimeMsw; } ASM_RUN_V2;
typedef struct _ASM_WRITE_V2 {
    UINT32 AddrLsw, AddrMsw, MemMapHandle, BufSize, SeqId, TsLsw, TsMsw, Flags;
} ASM_WRITE_V2;
typedef struct _ADM_OPEN_V5 {
    UINT16 Flags, Mode, Endpoint1, Endpoint2;
    UINT32 Topology;
    UINT16 NumChannels, BitWidth;
    UINT32 SampleRate;
    UINT8  ChannelMap[8];
} ADM_OPEN_V5;
typedef struct _ADM_MATRIX_MAP_V5 {        /* + nodo + 1 copp (u16) + relleno */
    UINT32 MatrixId, NumSessions;
    UINT16 SessionId, NumCopps;
    UINT16 CoppId, Pad;
} ADM_MATRIX_MAP_V5;
#pragma pack(pop)

/* ADSP_ASM_SESSION: sesion 1, stream 1 -> puerto APR (sesion << 8 | stream) */
#define ADSP_ASM_SESSION     1
#define ADSP_ASM_PORT        ((UINT16)((ADSP_ASM_SESSION << 8) | 1))
#define ADSP_PCM_CHUNK       9600u          /* 50 ms a 48 kHz estereo 16 bits */
#define ADSP_PCM_CHUNKS      60u            /* 3 s */
#define ADSP_PCM_BYTES       (ADSP_PCM_CHUNK * ADSP_PCM_CHUNKS)
#define ADSP_PCM_ALLOC       0x90000u       /* >= ADSP_PCM_BYTES, multiplo de 4 KiB */
#define ADSP_PCM_INFLIGHT    4u

static const INT16 kSine1k48[48] = {
    0, 535, 1060, 1567, 2048, 2493, 2896, 3250, 3547, 3784, 3956, 4061, 4096, 4061, 3956, 3784, 3547, 3250, 2896, 2493, 2048, 1567, 1060, 535, 0, -535, -1060, -1567, -2048, -2493, -2896, -3250, -3547, -3784, -3956, -4061, -4096, -4061, -3956, -3784, -3547, -3250, -2896, -2493, -2048, -1567, -1060, -535
};
