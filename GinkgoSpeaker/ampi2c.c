/*
 * ampi2c.c - prueba del MAX98927 por I2C (GENI SE1 del SM6125, Redmi Note 8).
 *
 * Es un driver KMDF de PRUEBA, separado del driver del ADSP. En D0Entry:
 *   1. pinmux de gpio4/gpio5 (SDA/SCL de la SE1, funcion "qup01")
 *   2. vota los relojes de la QUPV3 wrap0 + SE1 (GCC+0x7900C bits 6,7,8,9,11)
 *      y comprueba que el reloj de la SE1 no esta parado (halt 0x1F274)
 *   3. pulso de reset del amplificador (gpio27: bajo, 2 ms, alto, 10 ms)
 *   4. comprueba que el firmware de la SE1 sea I2C (proto 3) e inicializa GENI
 *   5. lee REV_ID (0x01FF) del MAX98927 en 0x3a (y 0x38..0x3b para comparar)
 *
 * Solo toca el _CRS la SE1 (0x4A84000). TLMM y GCC se mapean con MmMapIoSpace
 * solo en las paginas necesarias, para no chocar con los recursos que ya
 * reclaman otros drivers (touch: TLMM/GCC completos).
 *
 * NUNCA se apagan los bits de reloj: el registro 0x7900C es compartido con la
 * SE2 (touch).
 *
 * Log: \SystemRoot\amp_log.txt (+ DbgPrint).
 */
#include <ntddk.h>
#include <wdf.h>
#include <ntstrsafe.h>
#include <initguid.h>
#include "gka_amp.h"

#define AMP_TAG             'ipmA'
#define AMP_LOG_PATH        L"\\SystemRoot\\amp_log.txt"

/* ---- direcciones fisicas (ginkgo.dts / gcc-trinket.c) ------------------- */
#define SE1_PA              0x04A84000ull
#define SE1_SIZE            0x4000u
#define TLMM_PA             0x00400000ull
/* pinctrl-trinket.c: ctl_reg = tile + 0x1000 * id. Los gpio 4, 5 y 27 son del
 * tile WEST (0x100000). (SOUTH 0x500000, EAST 0x900000; NORTH no tiene pines:
 * 0x404000 etc. es espacio inexistente y colgaba el arranque.)  */
#define TLMM_WEST           0x00100000ull
#define TLMM_PIN_STRIDE     0x1000u
#define GCC_PA              0x01400000ull
#define GCC_GATE_OFF        0x7900Cu         /* enable_reg QUPV3 wrap0         */
#define GCC_SE1_HALT_OFF    0x1F274u         /* gcc_qupv3_wrap0_s1_clk halt    */
#define GCC_SE1_RCG_CMD     0x1F278u
#define GCC_SE1_RCG_CFG     0x1F27Cu
/* bits: 6 m_ahb, 7 s_ahb, 8 core, 9 core_2x, 11 se1 (misma mascara que la SE2
 * 0x13C0 pero con el bit 11 en vez del 12) */
#define GCC_GATE_SE1_MASK   0x00000BC0u

#define TLMM_EAST           0x00900000ull
#define MI2S_PIN_FIRST      113     /* SCK 113, WS 114, DATA0 115, DATA1 116 */
#define MI2S_PIN_COUNT      4
#define MI2S_CTL            0xC4u   /* func 1 (pri_mi2s/_ws), drive 8 mA, sin pull */
#define PIN_SDA             4
#define PIN_SCL             5
#define PIN_RESET           27
/* funcion TLMM "qup01"; si no hay ACK, confirmar con pinctrl-trinket.c */
#define AMP_I2C_FUNC        1
#define AMP_PIN_CTL         ((AMP_I2C_FUNC << 2) | (2 << 6))   /* drive 2, sin pull */

/*
 * AMP_STAGE: hasta donde llega la prueba (el log se pierde si Windows se
 * cuelga, asi que si algo vuelve a colgar, bajar este numero):
 *   0 = no hace NADA (para separar este driver de otros problemas)
 *   1 = solo pinmux    2 = + relojes    3 = + reset del amplificador
 *   4 = + leer FW_REV de la SE1    5 = + inicializar GENI    6 = + lecturas I2C
 */
#define AMP_STAGE           6

/* 0 = no tocar gpio4/gpio5 (el firmware ya los deja en func 1 con pull-up:
 * ctl=0x7); 1 = escribir ctl=0x84 (bias-disable como el DTS) */
#define AMP_PIN_WRITE       0

#define ARRAY_SIZE_AMP(a)   (sizeof(a) / sizeof((a)[0]))
/* 1 = escribir la configuracion del MAX98937 (sin habilitar la salida) */
#define AMP_CONFIGURE       1
/* 1 = tras configurar, espera AMP_ENABLE_WAIT_MS (a que adsp_driver arranque el
 * AFE, que da BCLK/WS) y enciende el amplificador. */
#define AMP_ENABLE          0     /* 0: lo enciende ginkgoaud por IOCTL_GKAAMP_ENABLE */
#define AMP_ENABLE_WAIT_MS  20000

#define AMP_ADDR            0x3a
#define AMP_REG_REV_ID      0x0600   /* max98937.c: MAX98937_REV_ID; 0x01FF lee 0 */

/* ---- registros GENI (offset desde la base de la SE) ---------------------- */
#define GENI_FORCE_DEFAULT_REG   0x020
#define GENI_OUTPUT_CTRL         0x024
#define GENI_CGC_CTRL            0x028
#define GENI_STATUS              0x040
#define GENI_SER_M_CLK_CFG       0x048
#define GENI_FW_REVISION_RO      0x068
#define GENI_CLK_SEL             0x07C
#define GENI_DFS_IF_CFG          0x080
#define GENI_DMA_MODE_EN         0x258
#define GENI_TX_PACKING_CFG0     0x260
#define GENI_TX_PACKING_CFG1     0x264
#define I2C_TX_TRANS_LEN         0x26C
#define I2C_RX_TRANS_LEN         0x270
#define I2C_SCL_COUNTERS         0x278
#define GENI_RX_PACKING_CFG0     0x284
#define GENI_RX_PACKING_CFG1     0x288
#define GENI_M_CMD0              0x600
#define GENI_M_CMD_CTRL_REG      0x604
#define GENI_M_IRQ_STATUS        0x610
#define GENI_M_IRQ_EN            0x614
#define GENI_M_IRQ_CLEAR         0x618
#define GENI_S_IRQ_CLEAR         0x648
#define GENI_TX_FIFOn            0x700
#define GENI_RX_FIFOn            0x780
#define GENI_TX_WATERMARK_REG    0x80C
#define GENI_RX_WATERMARK_REG    0x810
#define GENI_RX_RFR_WATERMARK    0x814
#define GENI_RX_FIFO_STATUS      0x804
#define SE_DMA_TX_IRQ_CLR        0xC44
#define SE_DMA_RX_IRQ_CLR        0xD44
#define SE_IRQ_EN                0xE1C
#define SE_HW_PARAM_0            0xE24
#define SE_DMA_GENERAL_CFG       0xE30
#define SE_GSI_EVENT_EN          0xE18

#define M_CMD_DONE               (1u << 0)
#define M_RX_IRQ                 (1u << 7)
#define M_RX_FIFO_WATERMARK      (1u << 26)
#define M_RX_FIFO_LAST           (1u << 27)
#define M_TX_FIFO_WATERMARK      (1u << 30)
#define M_BENIGN                 (M_CMD_DONE | M_RX_IRQ | M_RX_FIFO_WATERMARK | \
                                  M_RX_FIFO_LAST | M_TX_FIFO_WATERMARK)
#define M_CMD_ABORT_BIT          (1u << 5)
#define M_GENI_CMD_ABORT         (1u << 1)
#define M_GENI_CMD_CANCEL        (1u << 2)

#define I2C_OPC_WRITE            1u
#define I2C_OPC_READ             2u
#define M_OPCODE_SHFT            27
#define SLV_ADDR_SHFT            9
#define STOP_STRETCH             (1u << 2)

#define GENI_PROTO_I2C           3u

/* M_IRQ_EN: M_COMMON_GENI_M_IRQ_EN (sin IO_DATA_*) + CMD_DONE + watermarks +
 * LAST + GP_IRQ0..5 (NACK = GP_IRQ1 = bit 10, BUS_PROTO = bit 12, ARB_LOST = 13) */
#define AMP_M_IRQ_EN             (0x7Eu | 0x1u | 0x7E00u | (1u << 24) | (1u << 25) | \
                                  (1u << 26) | (1u << 27) | (1u << 28) | (1u << 29) | \
                                  (1u << 30))
#define M_GP_IRQ_NACK            (1u << 10)

/* t: { freq, clk_div, t_high, t_low, t_cycle } con fuente de 19.2 MHz */
#define I2C_DIV                  7u        /* 100 kHz */
#define I2C_T_HIGH               10u
#define I2C_T_LOW                11u
#define I2C_T_CYCLE              26u

typedef struct _AMP_CONTEXT {
    PHYSICAL_ADDRESS SePa;
    ULONG            SeLen;
    volatile UCHAR  *Se;
    volatile UCHAR  *Pins;       /* paginas de gpio4 y gpio5 (2 x 0x1000)  */
    volatile UCHAR  *Reset;      /* pagina de gpio27                       */
    volatile UCHAR  *GccGate;    /* pagina 0x1479000                       */
    volatile UCHAR  *MiPins;     /* gpio113..116 (EAST), PRI_MI2S: 4 x 0x1000 */
    volatile UCHAR  *GccRcg;     /* pagina 0x141F000                       */
    ULONG            TxDepth;
    BOOLEAN          On;          /* salida del amplificador habilitada */
} AMP_CONTEXT, *PAMP_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(AMP_CONTEXT, AmpGetContext)

DRIVER_INITIALIZE DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD       AmpEvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE AmpEvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE AmpEvtReleaseHardware;
EVT_WDF_DEVICE_D0_ENTRY         AmpEvtD0Entry;
EVT_WDF_IO_QUEUE_IO_DEVICE_CONTROL AmpEvtIoDeviceControl;

/* ---- log ---------------------------------------------------------------- */

static VOID AmpLog(const char *Fmt, ...)
{
    char buf[256];
    va_list ap;
    NTSTATUS st;
    size_t len = 0;
    UNICODE_STRING name;
    OBJECT_ATTRIBUTES oa;
    IO_STATUS_BLOCK iosb;
    HANDLE h = NULL;

    va_start(ap, Fmt);
    st = RtlStringCbVPrintfA(buf, sizeof(buf), Fmt, ap);
    va_end(ap);
    if (!NT_SUCCESS(st) && st != STATUS_BUFFER_OVERFLOW) {
        return;
    }
    DbgPrint("%s", buf);
    if (KeGetCurrentIrql() != PASSIVE_LEVEL ||
        !NT_SUCCESS(RtlStringCbLengthA(buf, sizeof(buf), &len)) || len == 0) {
        return;
    }
    RtlInitUnicodeString(&name, AMP_LOG_PATH);
    InitializeObjectAttributes(&oa, &name, OBJ_KERNEL_HANDLE | OBJ_CASE_INSENSITIVE,
                               NULL, NULL);
    st = ZwCreateFile(&h, FILE_APPEND_DATA | SYNCHRONIZE, &oa, &iosb, NULL,
                      FILE_ATTRIBUTE_NORMAL, FILE_SHARE_READ, FILE_OPEN_IF,
                      FILE_SYNCHRONOUS_IO_NONALERT | FILE_NON_DIRECTORY_FILE,
                      NULL, 0);
    if (NT_SUCCESS(st)) {
        ZwWriteFile(h, NULL, NULL, NULL, &iosb, buf, (ULONG)len, NULL, NULL);
        ZwClose(h);
    }
}

/* ---- acceso a registros ------------------------------------------------- */

#define RD(base, off)      READ_REGISTER_ULONG((volatile ULONG *)((base) + (off)))
#define WR(base, off, v)   WRITE_REGISTER_ULONG((volatile ULONG *)((base) + (off)), (ULONG)(v))
#define SE_RD(c, off)      RD((c)->Se, off)
#define SE_WR(c, off, v)   WR((c)->Se, off, v)

static VOID SleepMs(LONG ms)
{
    LARGE_INTEGER t;

    t.QuadPart = -(LONGLONG)ms * 10000;
    KeDelayExecutionThread(KernelMode, FALSE, &t);
}

static volatile UCHAR *MapPa(ULONGLONG Pa, ULONG Len)
{
    PHYSICAL_ADDRESS pa;

    pa.QuadPart = (LONGLONG)Pa;
    return (volatile UCHAR *)MmMapIoSpace(pa, Len, MmNonCached);
}

/* ---- pines, relojes, reset ---------------------------------------------- */

static VOID PinsSetup(PAMP_CONTEXT c)
{
    ULONG sda0 = RD(c->Pins, 0), scl0 = RD(c->Pins, TLMM_PIN_STRIDE);

#if AMP_PIN_WRITE
    WR(c->Pins, 0, AMP_PIN_CTL);
    WR(c->Pins, TLMM_PIN_STRIDE, AMP_PIN_CTL);
    KeMemoryBarrier();
#endif
    AmpLog("AMP: gpio%d ctl 0x%08x -> 0x%08x | gpio%d ctl 0x%08x -> 0x%08x (func=%d)\n",
           PIN_SDA, (unsigned)sda0, (unsigned)RD(c->Pins, 0),
           PIN_SCL, (unsigned)scl0, (unsigned)RD(c->Pins, TLMM_PIN_STRIDE),
           AMP_I2C_FUNC);
}

static NTSTATUS ClocksOn(PAMP_CONTEXT c)
{
    ULONG gate = RD(c->GccGate, GCC_GATE_OFF & 0xFFF);
    ULONG halt, i;

    AmpLog("AMP: GCC+0x7900C antes=0x%08x\n", (unsigned)gate);
    WR(c->GccGate, GCC_GATE_OFF & 0xFFF, gate | GCC_GATE_SE1_MASK);
    KeMemoryBarrier();
    for (i = 0; i < 20; i++) {
        SleepMs(1);
        halt = RD(c->GccRcg, GCC_SE1_HALT_OFF & 0xFFF);
        if (!(halt & 0x80000000u)) {
            break;
        }
    }
    AmpLog("AMP: GCC gate=0x%08x halt SE1=0x%08x rcg cmd=0x%08x cfg=0x%08x\n",
           (unsigned)RD(c->GccGate, GCC_GATE_OFF & 0xFFF), (unsigned)halt,
           (unsigned)RD(c->GccRcg, GCC_SE1_RCG_CMD & 0xFFF),
           (unsigned)RD(c->GccRcg, GCC_SE1_RCG_CFG & 0xFFF));
    if (halt & 0x80000000u) {
        AmpLog("AMP: el reloj de la SE1 sigue parado (CLK_OFF), no se toca la SE\n");
        return STATUS_DEVICE_NOT_READY;
    }
    return STATUS_SUCCESS;
}

static VOID ResetAmp(PAMP_CONTEXT c)
{
    ULONG ctl = RD(c->Reset, 0);
    ULONG io  = RD(c->Reset, 4);

    AmpLog("AMP: gpio%d antes ctl=0x%08x io=0x%08x\n", PIN_RESET,
           (unsigned)ctl, (unsigned)io);
    ctl &= ~(0xFu << 2);          /* funcion 0 = GPIO */
    ctl |= (1u << 9);             /* salida habilitada */
    WR(c->Reset, 4, 0);           /* nivel bajo antes de habilitar la salida */
    WR(c->Reset, 0, ctl);
    KeMemoryBarrier();
    SleepMs(2);
    WR(c->Reset, 4, 2);           /* bit 1 = valor de salida */
    KeMemoryBarrier();
    SleepMs(10);
    AmpLog("AMP: reset del amplificador hecho, ctl=0x%08x io=0x%08x\n",
           (unsigned)RD(c->Reset, 0), (unsigned)RD(c->Reset, 4));
}

static VOID ResetAmpLong(PAMP_CONTEXT c)
{
    WR(c->Reset, 4, 0);
    KeMemoryBarrier();
    SleepMs(20);
    WR(c->Reset, 4, 2);
    KeMemoryBarrier();
    SleepMs(50);
    AmpLog("AMP: reset largo hecho\n");
}

/* ---- GENI I2C ----------------------------------------------------------- */

static VOID GeniInit(PAMP_CONTEXT c)
{
    /* geni_se_init + geni_se_io_init + io_set_mode (FIFO) */
    SE_WR(c, SE_GSI_EVENT_EN, 0);
    SE_WR(c, GENI_M_IRQ_CLEAR, 0xFFFFFFFF);
    SE_WR(c, GENI_S_IRQ_CLEAR, 0xFFFFFFFF);
    SE_WR(c, SE_DMA_TX_IRQ_CLR, 0xFFFFFFFF);
    SE_WR(c, SE_DMA_RX_IRQ_CLR, 0xFFFFFFFF);
    SE_WR(c, SE_IRQ_EN, 0xFFFFFFFF);
    SE_WR(c, GENI_CGC_CTRL, SE_RD(c, GENI_CGC_CTRL) | 0x7F);
    SE_WR(c, SE_DMA_GENERAL_CFG, SE_RD(c, SE_DMA_GENERAL_CFG) | 0xF);
    SE_WR(c, GENI_OUTPUT_CTRL, 0x7F);
    SE_WR(c, GENI_FORCE_DEFAULT_REG, 1);
    SE_WR(c, GENI_DMA_MODE_EN, SE_RD(c, GENI_DMA_MODE_EN) & ~1u);   /* FIFO */

    c->TxDepth = (SE_RD(c, SE_HW_PARAM_0) >> 16) & 0x3F;
    if (c->TxDepth < 2) {
        c->TxDepth = 16;
    }
    SE_WR(c, GENI_RX_WATERMARK_REG, c->TxDepth - 1);
    SE_WR(c, GENI_RX_RFR_WATERMARK, c->TxDepth);

    /* geni_se_config_packing(8 bits, 4 palabras, msb_to_lsb) */
    SE_WR(c, GENI_TX_PACKING_CFG0, 0x7F8FE);
    SE_WR(c, GENI_TX_PACKING_CFG1, 0xFFEFE);
    SE_WR(c, GENI_RX_PACKING_CFG0, 0x7F8FE);
    SE_WR(c, GENI_RX_PACKING_CFG1, 0xFFEFE);

    /* reloj: nivel DFS 0 (XO 19.2 MHz) y divisor/contadores de 100 kHz */
    /* igual que qcom_geni_i2c_conf(): CLK_SEL = 0 (dfs 0), sin tocar DFS_IF */
    SE_WR(c, GENI_M_IRQ_EN, AMP_M_IRQ_EN);
    SE_WR(c, GENI_CLK_SEL, 0);
    SE_WR(c, GENI_SER_M_CLK_CFG, (I2C_DIV << 4) | 1u);
    SE_WR(c, I2C_SCL_COUNTERS,
          (I2C_T_HIGH << 20) | (I2C_T_LOW << 10) | I2C_T_CYCLE);
    KeMemoryBarrier();
    AmpLog("AMP: GENI listo: tx_depth=%u m_irq_en=0x%08x clk_cfg=0x%08x scl=0x%08x\n",
           (unsigned)c->TxDepth, (unsigned)SE_RD(c, GENI_M_IRQ_EN),
           (unsigned)SE_RD(c, GENI_SER_M_CLK_CFG),
           (unsigned)SE_RD(c, I2C_SCL_COUNTERS));
}

static NTSTATUS WaitM(PAMP_CONTEXT c, ULONG *Status, ULONG Ms)
{
    ULONG i, s;

    for (i = 0; i < Ms * 20; i++) {
        s = SE_RD(c, GENI_M_IRQ_STATUS);
        if (s & (M_CMD_DONE | ~M_BENIGN)) {
            *Status = s;
            return STATUS_SUCCESS;
        }
        KeStallExecutionProcessor(50);
    }
    *Status = SE_RD(c, GENI_M_IRQ_STATUS);
    return STATUS_IO_TIMEOUT;
}

static VOID Abort(PAMP_CONTEXT c)
{
    ULONG i, s = 0;

    SE_WR(c, GENI_M_CMD_CTRL_REG, M_GENI_CMD_CANCEL);
    for (i = 0; i < 2000; i++) {
        s = SE_RD(c, GENI_M_IRQ_STATUS);
        if (s & (1u << 4)) {          /* M_CMD_CANCEL */
            break;
        }
        KeStallExecutionProcessor(50);
    }
    if (!(s & (1u << 4))) {
        SE_WR(c, GENI_M_CMD_CTRL_REG, M_GENI_CMD_ABORT);
        for (i = 0; i < 2000; i++) {
            s = SE_RD(c, GENI_M_IRQ_STATUS);
            if (s & M_CMD_ABORT_BIT) {
                break;
            }
            KeStallExecutionProcessor(50);
        }
    }
    SE_WR(c, GENI_M_IRQ_CLEAR, 0xFFFFFFFF);
}

/* lectura de un registro (RegBytes = 1 o 2 bytes de direccion) y valor de 8 bits */
static ULONG g_dbgReads;

static NTSTATUS AmpReadRegN(PAMP_CONTEXT c, ULONG Addr, ULONG Reg, ULONG RegBytes,
                            ULONG *Val)
{
    NTSTATUS nt;
    ULONG st = 0, rxst, word, rst = 0;

    SE_WR(c, GENI_M_IRQ_CLEAR, 0xFFFFFFFF);

    /* write [reg] sin STOP (repeated start) */
    SE_WR(c, I2C_TX_TRANS_LEN, RegBytes);
    SE_WR(c, GENI_TX_WATERMARK_REG, 1);
    SE_WR(c, GENI_M_CMD0, (I2C_OPC_WRITE << M_OPCODE_SHFT) |
                          (Addr << SLV_ADDR_SHFT) | STOP_STRETCH);
    if (RegBytes == 2) {
        SE_WR(c, GENI_TX_FIFOn, ((Reg >> 8) & 0xFF) | ((Reg & 0xFF) << 8));
    } else {
        SE_WR(c, GENI_TX_FIFOn, Reg & 0xFF);
    }
    SE_WR(c, GENI_TX_WATERMARK_REG, 0);
    KeMemoryBarrier();
    nt = WaitM(c, &st, 100);
    if (!NT_SUCCESS(nt) || (st & ~M_BENIGN)) {
        AmpLog("AMP:   addr 0x%02x: write de registro fallo m_irq=0x%08x "
               "geni_status=0x%08x (%s)\n", (unsigned)Addr, (unsigned)st,
               (unsigned)SE_RD(c, GENI_STATUS),
               NT_SUCCESS(nt) ? ((st & M_GP_IRQ_NACK) ? "NACK: el esclavo no responde"
                                                      : "error de GENI") : "timeout");
        Abort(c);
        return NT_SUCCESS(nt) ? STATUS_UNSUCCESSFUL : nt;
    }
    SE_WR(c, GENI_M_IRQ_CLEAR, st);

    SE_WR(c, I2C_RX_TRANS_LEN, 1);
    SE_WR(c, GENI_M_CMD0, (I2C_OPC_READ << M_OPCODE_SHFT) | (Addr << SLV_ADDR_SHFT));
    KeMemoryBarrier();
    nt = WaitM(c, &rst, 100);
    if (!NT_SUCCESS(nt) || (rst & ~M_BENIGN)) {
        AmpLog("AMP:   addr 0x%02x: read fallo m_irq=0x%08x geni_status=0x%08x (%s)\n",
               (unsigned)Addr, (unsigned)rst, (unsigned)SE_RD(c, GENI_STATUS),
               NT_SUCCESS(nt) ? "error de GENI" : "timeout");
        Abort(c);
        return NT_SUCCESS(nt) ? STATUS_UNSUCCESSFUL : nt;
    }
    rxst = SE_RD(c, GENI_RX_FIFO_STATUS);
    word = SE_RD(c, GENI_RX_FIFOn);
    SE_WR(c, GENI_M_IRQ_CLEAR, rst);
    if (g_dbgReads < 4) {
        g_dbgReads++;
        AmpLog("AMP:   [dbg] write m_irq=0x%08x, read m_irq=0x%08x rx_fifo_status=0x%08x "
               "palabra cruda=0x%08x\n", (unsigned)st, (unsigned)rst,
               (unsigned)rxst, (unsigned)word);
    }
    if ((rxst & 0x1FFFFFF) == 0) {
        AmpLog("AMP:   addr 0x%02x: read termino sin datos (rx_fifo_status=0x%08x)\n",
               (unsigned)Addr, (unsigned)rxst);
        return STATUS_DEVICE_DATA_ERROR;
    }
    *Val = word & 0xFF;
    return STATUS_SUCCESS;
}

#define AmpReadReg(c, a, r, v)   AmpReadRegN((c), (a), (r), 2, (v))

/* escritura de un registro de 16 bits de direccion y valor de 8 bits */
static NTSTATUS AmpWriteReg(PAMP_CONTEXT c, ULONG Addr, ULONG Reg, ULONG Val)
{
    ULONG st = 0;
    NTSTATUS nt;

    SE_WR(c, GENI_M_IRQ_CLEAR, 0xFFFFFFFF);
    SE_WR(c, I2C_TX_TRANS_LEN, 3);
    SE_WR(c, GENI_TX_WATERMARK_REG, 1);
    SE_WR(c, GENI_M_CMD0, (I2C_OPC_WRITE << M_OPCODE_SHFT) | (Addr << SLV_ADDR_SHFT));
    SE_WR(c, GENI_TX_FIFOn, ((Reg >> 8) & 0xFF) | ((Reg & 0xFF) << 8) |
                            ((Val & 0xFF) << 16));
    SE_WR(c, GENI_TX_WATERMARK_REG, 0);
    KeMemoryBarrier();
    nt = WaitM(c, &st, 100);
    if (!NT_SUCCESS(nt) || (st & ~M_BENIGN)) {
        AmpLog("AMP:   write 0x%04x=0x%02x fallo m_irq=0x%08x (%s)\n", (unsigned)Reg,
               (unsigned)Val, (unsigned)st, NT_SUCCESS(nt) ? "NACK/error" : "timeout");
        Abort(c);
        return NT_SUCCESS(nt) ? STATUS_UNSUCCESSFUL : nt;
    }
    SE_WR(c, GENI_M_IRQ_CLEAR, st);
    return STATUS_SUCCESS;
}

static NTSTATUS AmpUpdate(PAMP_CONTEXT c, ULONG Reg, ULONG Mask, ULONG Val)
{
    ULONG old = 0;
    NTSTATUS nt = AmpReadReg(c, AMP_ADDR, Reg, &old);

    if (!NT_SUCCESS(nt)) {
        return nt;
    }
    return AmpWriteReg(c, AMP_ADDR, Reg, (old & ~Mask) | (Val & Mask));
}

/*
 * Configuracion del MAX98937 (la que hace max98927_i2c_probe en el kernel de
 * Xiaomi con CONFIG_MACH_XIAOMI_C3J: canal mono + mapa comun) + formato I2S
 * esclavo, 16 bits, 48 kHz (max98927_dai_set_fmt / hw_params / set_clock).
 * NO habilita el amplificador (Global_Enable / AMP_enables): hace falta que
 * el BCLK/WS del PRI_MI2S ya este corriendo.
 */
typedef struct { USHORT Reg; UCHAR Val; } AMP_REGVAL;

static const AMP_REGVAL kAmpInit[] = {
    /* canal mono (max98937_reg_channel_map[0]) */
    {0x0040,0x01},{0x001e,0x01},{0x0018,0x03},{0x001a,0x03},{0x001c,0xFC},
    {0x0028,0x80},{0x0029,0x01},
    /* mapa comun (max98937_reg_common_map, variante C3J) */
    {0x0011,0x03},{0x0053,0x00},{0x0055,0x00},
    {0x005e,0x16},{0x005f,0x00},{0x0060,0x0c},{0x0061,0x0c},
    {0x0062,0x12},{0x0063,0x00},{0x0064,0x0c},{0x0065,0x0c},
    {0x0066,0x0e},{0x0067,0x03},{0x0068,0x18},{0x0069,0x18},
    {0x006a,0x0a},{0x006b,0x00},{0x006c,0x80},{0x006d,0x00},
    {0x005a,0x00},{0x005b,0x00},{0x005c,0x1f},{0x005d,0x00},
    {0x0042,0x07},{0x0014,0x75},{0x0015,0x8c},{0x0017,0x55},{0x003e,0xF7},
    {0x0023,0x40},{0x003a,0x00},{0x001b,0x00},{0x0019,0x00},{0x001f,0x03},
    {0x001d,0xFF},{0x003d,0x03},{0x0035,0x00},{0x0036,0x38},{0x0037,0x0f},
    {0x003b,0x55},{0x003c,0xA0},{0x003f,0x1c},{0x0041,0x40},{0x0043,0x00},
    {0x0044,0xFF},{0x0016,0x08},{0x0073,0x0f},{0x0077,0x01},{0x0052,0x02},
    {0x0020,0x00},{0x0021,0x00},{0x0022,0x00},{0x004e,0x10},{0x0079,0x45},
    {0x007a,0x2B},{0x007b,0x04},
};

static VOID ConfigAmp(PAMP_CONTEXT c)
{
    ULONG i, bad = 0, fail = 0, v = 0;
    NTSTATUS nt;

    for (i = 0; i < ARRAY_SIZE_AMP(kAmpInit); i++) {
        nt = AmpWriteReg(c, AMP_ADDR, kAmpInit[i].Reg, kAmpInit[i].Val);
        if (!NT_SUCCESS(nt)) {
            fail++;
        }
    }
    /* esclavo I2S, BCLK = 32*fs (16 bits), 48 kHz */
    AmpUpdate(c, 0x0024, 0x03, 0x00);        /* PCM_Master_Mode: esclavo        */
    AmpUpdate(c, 0x0025, 0x0F, 0x02);        /* PCM_Clock_setup: BSEL = 32*fs   */
    AmpUpdate(c, 0x0026, 0x0F, 0x08);        /* DIG_IF_SR = 48000               */
    AmpUpdate(c, 0x0027, 0xF0, 0x80);        /* SPK_SR   = 48000                */
    AmpUpdate(c, 0x0027, 0x0F, 0x08);        /* IVADC_SR = 48000                */

    for (i = 0; i < ARRAY_SIZE_AMP(kAmpInit); i++) {
        nt = AmpReadReg(c, AMP_ADDR, kAmpInit[i].Reg, &v);
        if (!NT_SUCCESS(nt) || v != kAmpInit[i].Val) {
            bad++;
            AmpLog("AMP:   reg 0x%04x escrito 0x%02x leido %s0x%02x\n",
                   (unsigned)kAmpInit[i].Reg, (unsigned)kAmpInit[i].Val,
                   NT_SUCCESS(nt) ? "" : "(fallo) ", (unsigned)v);
        }
    }
    AmpLog("AMP: configuracion: %u registros, %u escrituras fallidas, %u lecturas distintas\n",
           (unsigned)ARRAY_SIZE_AMP(kAmpInit), (unsigned)fail, (unsigned)bad);
    {
        static const ULONG chk[] = { 0x0023, 0x0024, 0x0025, 0x0026, 0x0027,
                                     0x0500, 0x0039, 0x0004, 0x0005, 0x0006 };

        for (i = 0; i < ARRAY_SIZE_AMP(chk); i++) {
            nt = AmpReadReg(c, AMP_ADDR, chk[i], &v);
            AmpLog("AMP:   reg 0x%04x = 0x%02x\n", (unsigned)chk[i], (unsigned)v);
        }
    }
}

/*
 * Experimento de polaridad/estado del pin de reset: deja gpio27 en un estado y
 * lee REV_ID con direccion de 16 y de 8 bits.
 */
static VOID TryResetState(PAMP_CONTEXT c, const char *Name, ULONG Ctl, ULONG Io)
{
    ULONG v = 0, v8 = 0;
    NTSTATUS n16, n8;

    WR(c->Reset, 4, Io);
    WR(c->Reset, 0, Ctl);
    KeMemoryBarrier();
    SleepMs(50);
    n16 = AmpReadRegN(c, AMP_ADDR, AMP_REG_REV_ID, 2, &v);
    n8  = AmpReadRegN(c, AMP_ADDR, 0x00, 1, &v8);
    AmpLog("AMP: reset=%s (ctl=0x%08x io=0x%08x): REV_ID16 %s 0x%02x, reg0(8b) %s 0x%02x\n",
           Name, (unsigned)RD(c->Reset, 0), (unsigned)RD(c->Reset, 4),
           NT_SUCCESS(n16) ? "=" : "fallo", (unsigned)v,
           NT_SUCCESS(n8) ? "=" : "fallo", (unsigned)v8);
}

static VOID AmpDumpStatus(PAMP_CONTEXT c, const char *Tag)
{
    static const ULONG r[] = { 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006,
                               0x0007, 0x0008, 0x0009, 0x000A, 0x0039, 0x0500 };
    ULONG i, v;

    for (i = 0; i < ARRAY_SIZE_AMP(r); i++) {
        v = 0xFF;
        (VOID)AmpReadReg(c, AMP_ADDR, r[i], &v);
        AmpLog("AMP: %s reg 0x%04x = 0x%02x\n", Tag, (unsigned)r[i], (unsigned)v);
    }
}

/* Enciende el amplificador (con BCLK/WS ya presentes). */
static VOID MiPinsSetup(PAMP_CONTEXT c)
{
    ULONG i;

    /* el TLMM es del AP: sin esto el ADSP genera BCLK/WS pero no salen al pad */
    for (i = 0; i < MI2S_PIN_COUNT; i++) {
        ULONG off = i * TLMM_PIN_STRIDE, a = RD(c->MiPins, off);

        WR(c->MiPins, off, MI2S_CTL);
        KeMemoryBarrier();
        AmpLog("AMP: gpio%u ctl 0x%08x -> 0x%08x (pri_mi2s)\n",
               (unsigned)(MI2S_PIN_FIRST + i), (unsigned)a,
               (unsigned)RD(c->MiPins, off));
    }
}

/* Enciende la salida (con BCLK/WS ya presentes). Mismo orden que la prueba que sono. */
static NTSTATUS AmpOn(PAMP_CONTEXT c)
{
    NTSTATUS nt;

    if (c->On) {
        return STATUS_SUCCESS;
    }
    MiPinsSetup(c);
    nt = AmpUpdate(c, 0x0039, 0x01, 0x01);       /* AMP_enables bit0 */
    if (!NT_SUCCESS(nt)) {
        AmpLog("AMP: AmpOn: AMP_enables fallo 0x%08x\n", (unsigned)nt);
        return nt;
    }
    SleepMs(5);
    nt = AmpUpdate(c, 0x0500, 0x01, 0x01);       /* Global_Enable bit0 */
    if (!NT_SUCCESS(nt)) {
        AmpLog("AMP: AmpOn: Global_Enable fallo 0x%08x\n", (unsigned)nt);
        return nt;
    }
    /* desbloqueo del C3J (max98937.c); un NACK suelto aqui no es fatal */
    (VOID)AmpWriteReg(c, AMP_ADDR, 0x0600, 0x54);
    (VOID)AmpWriteReg(c, AMP_ADDR, 0x0600, 0x4D);
    (VOID)AmpWriteReg(c, AMP_ADDR, 0x030D, 0x40);
    SleepMs(50);
    AmpDumpStatus(c, "despues");
    c->On = TRUE;
    AmpLog("AMP: amplificador ENCENDIDO\n");
    return STATUS_SUCCESS;
}

static NTSTATUS AmpOff(PAMP_CONTEXT c)
{
    NTSTATUS n1, n2;

    if (!c->On) {
        return STATUS_SUCCESS;
    }
    n1 = AmpUpdate(c, 0x0500, 0x01, 0x00);       /* Global_Enable = 0 */
    n2 = AmpUpdate(c, 0x0039, 0x01, 0x00);       /* AMP_enables = 0 */
    c->On = FALSE;
    AmpLog("AMP: amplificador APAGADO (0x%08x 0x%08x)\n", (unsigned)n1, (unsigned)n2);
    return NT_SUCCESS(n1) ? n2 : n1;
}

VOID AmpEvtIoDeviceControl(WDFQUEUE Queue, WDFREQUEST Request, size_t OutLen,
                           size_t InLen, ULONG Code)
{
    PAMP_CONTEXT c = AmpGetContext(WdfIoQueueGetDevice(Queue));
    NTSTATUS st = STATUS_INVALID_DEVICE_REQUEST;

    UNREFERENCED_PARAMETER(OutLen);
    UNREFERENCED_PARAMETER(InLen);
    if (c->Se == NULL) {
        st = STATUS_DEVICE_NOT_READY;
    } else if (Code == IOCTL_GKAAMP_ENABLE) {
        st = AmpOn(c);
    } else if (Code == IOCTL_GKAAMP_DISABLE) {
        st = AmpOff(c);
    }
    WdfRequestComplete(Request, st);
}

#if AMP_ENABLE
static VOID AmpEnable(PAMP_CONTEXT c)
{
    MiPinsSetup(c);
    AmpLog("AMP: esperando %u ms al AFE...\n", (unsigned)AMP_ENABLE_WAIT_MS);
    SleepMs(AMP_ENABLE_WAIT_MS);
    AmpDumpStatus(c, "antes");
    AmpUpdate(c, 0x0039, 0x01, 0x01);        /* AMP_enables bit0 */
    SleepMs(5);
    AmpUpdate(c, 0x0500, 0x01, 0x01);        /* Global_Enable bit0 */
    /* desbloqueo del C3J (max98937.c) */
    AmpWriteReg(c, AMP_ADDR, 0x0600, 0x54);
    AmpWriteReg(c, AMP_ADDR, 0x0600, 0x4D);
    AmpWriteReg(c, AMP_ADDR, 0x030D, 0x40);
    SleepMs(50);
    AmpDumpStatus(c, "despues");
    AmpLog("AMP: amplificador encendido\n");
}
#endif

static VOID RunTest(PAMP_CONTEXT c)
{
    ULONG fw = SE_RD(c, GENI_FW_REVISION_RO);
    ULONG proto = (fw >> 8) & 0xFF, a, v = 0;
    NTSTATUS nt;

    AmpLog("AMP: SE1 FW_REV=0x%08x proto=%u (3 = I2C) geni_status=0x%08x\n",
           (unsigned)fw, (unsigned)proto, (unsigned)SE_RD(c, GENI_STATUS));
    if (proto != GENI_PROTO_I2C) {
        AmpLog("AMP: la SE1 no tiene firmware I2C; hay que cargarlo (no soportado)\n");
        return;
    }
#if AMP_STAGE >= 5
    GeniInit(c);
#endif
#if AMP_STAGE < 6
    AmpLog("AMP: AMP_STAGE=%d, sin lecturas I2C\n", AMP_STAGE);
    return;
#endif

    /* como check_max98927_presence(): reintentos con espera tras el reset */
    for (a = 0; a < 3; a++) {
        nt = AmpReadReg(c, AMP_ADDR, AMP_REG_REV_ID, &v);
        AmpLog("AMP: intento %u: addr 0x%02x REV_ID(0x%04x) %s 0x%02x%s\n",
               (unsigned)a, AMP_ADDR, AMP_REG_REV_ID,
               NT_SUCCESS(nt) ? "=" : "fallo",
               (unsigned)v,
               (NT_SUCCESS(nt) && (v & 0x40))
                   ? "  <- bit 0x40 activo: chip presente (max98937.c)" : "");
        if (NT_SUCCESS(nt) && (v & 0x40)) {
            break;
        }
        SleepMs(20 + (LONG)a * 20);
    }

    /* segundo intento con un reset mas largo (20 ms bajo, 50 ms alto) */
    if (!(v & 0x40)) {
        ResetAmpLong(c);
        for (a = 0; a < 4; a++) {
            nt = AmpReadReg(c, AMP_ADDR, AMP_REG_REV_ID, &v);
            AmpLog("AMP: tras reset largo, intento %u: REV_ID %s 0x%02x\n",
                   (unsigned)a, NT_SUCCESS(nt) ? "=" : "fallo", (unsigned)v);
            if (NT_SUCCESS(nt) && (v & 0x40)) {
                break;
            }
            SleepMs(50);
        }
    }
    /* barrido: la direccion de REV_ID cambia segun la version del driver
     * (0x01FF en el de Xiaomi, 0x21FF en el mainline) */
    {
        static const ULONG regs[] = { 0x0001, 0x0002, 0x0003, 0x0004, 0x01FF,
                                      0x05FF, 0x0600, 0x0601, 0x0602, 0x06FF,
                                      0x2000, 0x2001, 0x2002, 0x21FF };
        ULONG i;

        for (i = 0; i < ARRAY_SIZE_AMP(regs); i++) {
            nt = AmpReadReg(c, AMP_ADDR, regs[i], &v);
            AmpLog("AMP: reg 0x%04x %s 0x%02x\n", (unsigned)regs[i],
                   NT_SUCCESS(nt) ? "=" : "fallo", (unsigned)v);
        }
    }
    /* estados del pin de reset (gpio27): bajo, hi-Z sin pull, pull-up, alto */
    {
        ULONG base = RD(c->Reset, 0) & ~((0xFu << 2) | 3u | (1u << 9));

        TryResetState(c, "BAJO", base | (1u << 9), 0);
        TryResetState(c, "ENTRADA sin pull", base, 0);
        TryResetState(c, "ENTRADA pull-up", base | 3u, 0);
        TryResetState(c, "ENTRADA pull-down", base | 1u, 0);
        TryResetState(c, "ALTO", base | (1u << 9), 2);
    }

    /* control del camino de lectura: otros esclavos del mismo bus (DTS: ktd3136
     * en 0x36 con registros de 8 bits; pm8008 en 0x08/0x09 no se toca) */
    {
        ULONG r;

        for (r = 0; r < 8; r++) {
            nt = AmpReadRegN(c, 0x36, r, 1, &v);
            AmpLog("AMP: control 0x36 reg 0x%02x %s 0x%02x\n", (unsigned)r,
                   NT_SUCCESS(nt) ? "=" : "fallo", (unsigned)v);
        }
        /* y el amplificador con direccion de registro de 8 bits, por si
         * el chip no usa 16 bits */
        for (r = 0; r < 4; r++) {
            nt = AmpReadRegN(c, AMP_ADDR, r, 1, &v);
            AmpLog("AMP: 0x3a (8 bits) reg 0x%02x %s 0x%02x\n", (unsigned)r,
                   NT_SUCCESS(nt) ? "=" : "fallo", (unsigned)v);
        }
    }
#if AMP_CONFIGURE
    ConfigAmp(c);
#if AMP_ENABLE
    AmpEnable(c);
#endif
#endif
    AmpLog("AMP: prueba terminada\n");
}

/* ---- WDF ---------------------------------------------------------------- */

NTSTATUS AmpEvtPrepareHardware(WDFDEVICE Device, WDFCMRESLIST Raw,
                               WDFCMRESLIST Translated)
{
    PAMP_CONTEXT c = AmpGetContext(Device);
    ULONG i, n = WdfCmResourceListGetCount(Translated);

    UNREFERENCED_PARAMETER(Raw);
    for (i = 0; i < n; i++) {
        PCM_PARTIAL_RESOURCE_DESCRIPTOR d =
            WdfCmResourceListGetDescriptor(Translated, i);

        if (d->Type == CmResourceTypeMemory &&
            (ULONGLONG)d->u.Memory.Start.QuadPart == SE1_PA) {
            c->SePa  = d->u.Memory.Start;
            c->SeLen = d->u.Memory.Length;
        }
    }
    if (c->SeLen < SE1_SIZE) {
        AmpLog("AMP: falta el recurso de la SE1 (0x%llx) en el _CRS\n", SE1_PA);
        return STATUS_DEVICE_CONFIGURATION_ERROR;
    }
    c->Se       = MapPa(SE1_PA, SE1_SIZE);
    c->Pins     = MapPa(TLMM_PA + TLMM_WEST + (ULONGLONG)PIN_SDA * TLMM_PIN_STRIDE,
                        2 * TLMM_PIN_STRIDE);
    c->Reset    = MapPa(TLMM_PA + TLMM_WEST + (ULONGLONG)PIN_RESET * TLMM_PIN_STRIDE,
                        TLMM_PIN_STRIDE);
    c->MiPins   = MapPa(TLMM_PA + TLMM_EAST + (ULONGLONG)MI2S_PIN_FIRST * TLMM_PIN_STRIDE,
                        MI2S_PIN_COUNT * TLMM_PIN_STRIDE);
    c->GccGate  = MapPa(GCC_PA + (GCC_GATE_OFF & ~0xFFFu), 0x1000);
    c->GccRcg   = MapPa(GCC_PA + (GCC_SE1_HALT_OFF & ~0xFFFu), 0x1000);
    if (!c->MiPins || !c->Se || !c->Pins || !c->Reset || !c->GccGate || !c->GccRcg) {
        AmpLog("AMP: MmMapIoSpace fallo\n");
        return STATUS_INSUFFICIENT_RESOURCES;       /* ReleaseHardware libera */
    }
    return STATUS_SUCCESS;
}

NTSTATUS AmpEvtReleaseHardware(WDFDEVICE Device, WDFCMRESLIST Translated)
{
    PAMP_CONTEXT c = AmpGetContext(Device);

    UNREFERENCED_PARAMETER(Translated);
    if (c->Se)      { MmUnmapIoSpace((PVOID)c->Se, SE1_SIZE); c->Se = NULL; }
    if (c->Pins)    { MmUnmapIoSpace((PVOID)c->Pins, 2 * TLMM_PIN_STRIDE); c->Pins = NULL; }
    if (c->Reset)   { MmUnmapIoSpace((PVOID)c->Reset, TLMM_PIN_STRIDE); c->Reset = NULL; }
    if (c->MiPins)  { MmUnmapIoSpace((PVOID)c->MiPins, MI2S_PIN_COUNT * TLMM_PIN_STRIDE); c->MiPins = NULL; }
    if (c->GccGate) { MmUnmapIoSpace((PVOID)c->GccGate, 0x1000); c->GccGate = NULL; }
    if (c->GccRcg)  { MmUnmapIoSpace((PVOID)c->GccRcg, 0x1000); c->GccRcg = NULL; }
    return STATUS_SUCCESS;
}

NTSTATUS AmpEvtD0Entry(WDFDEVICE Device, WDF_POWER_DEVICE_STATE PreviousState)
{
    PAMP_CONTEXT c = AmpGetContext(Device);

    UNREFERENCED_PARAMETER(PreviousState);
    AmpLog("AMP: D0Entry, prueba del MAX98927 por I2C (SE1) AMP_STAGE=%d\n",
           AMP_STAGE);
#if AMP_STAGE >= 1
    PinsSetup(c);
#endif
#if AMP_STAGE >= 2
    if (NT_SUCCESS(ClocksOn(c))) {
#if AMP_STAGE >= 3
        ResetAmp(c);
#endif
#if AMP_STAGE >= 4
        RunTest(c);
#endif
    }
#endif
    return STATUS_SUCCESS;      /* un fallo de la prueba no tumba el dispositivo */
}

NTSTATUS AmpEvtDeviceAdd(WDFDRIVER Driver, PWDFDEVICE_INIT Init)
{
    WDF_PNPPOWER_EVENT_CALLBACKS pnp;
    WDF_OBJECT_ATTRIBUTES attrs;
    WDFDEVICE dev;
    WDF_IO_QUEUE_CONFIG qc;
    WDFQUEUE q;
    NTSTATUS st;

    UNREFERENCED_PARAMETER(Driver);
    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnp);
    pnp.EvtDevicePrepareHardware = AmpEvtPrepareHardware;
    pnp.EvtDeviceReleaseHardware = AmpEvtReleaseHardware;
    pnp.EvtDeviceD0Entry         = AmpEvtD0Entry;
    WdfDeviceInitSetPnpPowerEventCallbacks(Init, &pnp);

    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attrs, AMP_CONTEXT);
    st = WdfDeviceCreate(&Init, &attrs, &dev);
    if (!NT_SUCCESS(st)) {
        return st;
    }

    WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&qc, WdfIoQueueDispatchSequential);
    qc.EvtIoDeviceControl = AmpEvtIoDeviceControl;
    st = WdfIoQueueCreate(dev, &qc, WDF_NO_OBJECT_ATTRIBUTES, &q);
    if (!NT_SUCCESS(st)) {
        return st;
    }
    st = WdfDeviceCreateDeviceInterface(dev, &GUID_DEVINTERFACE_GKA_AMP, NULL);
    AmpLog("AMP: interfaz GKA_AMP nt=0x%08x\n", (unsigned)st);
    return STATUS_SUCCESS;
}

NTSTATUS DriverEntry(PDRIVER_OBJECT Drv, PUNICODE_STRING Reg)
{
    WDF_DRIVER_CONFIG cfg;

    WDF_DRIVER_CONFIG_INIT(&cfg, AmpEvtDeviceAdd);
    return WdfDriverCreate(Drv, Reg, WDF_NO_OBJECT_ATTRIBUTES, &cfg,
                           WDF_NO_HANDLE);
}
