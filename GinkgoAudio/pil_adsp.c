/*
 * pil_adsp.c - cargador del ADSP (ver pil_adsp.h).
 *
 * SUPUESTOS A VERIFICAR contra pil_init_mmap() de tu peripheral-loader.c
 * (esa funcion no estaba en lo que pegaste):
 *   - segmento cargable = PT_LOAD, p_memsz != 0 y tipo distinto de "hash"
 *     ((p_flags & 0x07000000) != 0x02000000)
 *   - region_start = menor p_paddr, region_end = mayor (p_paddr + p_memsz),
 *     alineado hacia arriba a 4 KiB
 *   - imagen relocatable si algun segmento tiene el bit 27 en p_flags; en ese
 *     caso los segmentos van a RegionBase + (p_paddr - menor p_paddr)
 */
#include "pil_adsp.h"

typedef struct _ELF32_EHDR {
    UINT8  e_ident[16];
    UINT16 e_type;
    UINT16 e_machine;
    UINT32 e_version;
    UINT32 e_entry;
    UINT32 e_phoff;
    UINT32 e_shoff;
    UINT32 e_flags;
    UINT16 e_ehsize;
    UINT16 e_phentsize;
    UINT16 e_phnum;
    UINT16 e_shentsize;
    UINT16 e_shnum;
    UINT16 e_shstrndx;
} ELF32_EHDR;

typedef struct _ELF32_PHDR {
    UINT32 p_type;
    UINT32 p_offset;
    UINT32 p_vaddr;
    UINT32 p_paddr;
    UINT32 p_filesz;
    UINT32 p_memsz;
    UINT32 p_flags;
    UINT32 p_align;
} ELF32_PHDR;

#define PT_LOAD_TYPE        1u
#define MDT_TYPE_MASK       0x07000000u
#define MDT_TYPE_HASH       0x02000000u
#define MDT_RELOCATABLE     0x08000000u    /* BIT(27) */
#define PAGE_ALIGN_4K(x)    (((x) + 0xFFFull) & ~0xFFFull)

static BOOLEAN SegIsLoadable(const ELF32_PHDR *p)
{
    return p->p_type == PT_LOAD_TYPE &&
           (p->p_flags & MDT_TYPE_MASK) != MDT_TYPE_HASH &&
           p->p_memsz != 0;
}

/*
 * Copia Src (SrcLen bytes) a la direccion fisica Pa y rellena con ceros hasta
 * Len. Se mapea sin cache en tramos de 1 MiB.
 *
 * OJO: se usa MmWriteCombined (memoria Normal-NC en ARM64) para que RtlCopyMemory
 * pueda hacer accesos no alineados. Si ves un fallo de alineacion, cambialo por
 * un bucle de UINT32/UINT64 alineados.
 */
#define MAP_CHUNK  (1ull << 20)

static NTSTATUS FillPhys(UINT64 Pa, UINT64 Len, const UINT8 *Src, UINT64 SrcLen)
{
    UINT64 done = 0;

    while (done < Len) {
        UINT64 n = Len - done;
        PHYSICAL_ADDRESS pa;
        UINT8 *va;
        UINT64 copy = 0;

        if (n > MAP_CHUNK) {
            n = MAP_CHUNK;
        }

        pa.QuadPart = (LONGLONG)(Pa + done);
        va = (UINT8 *)MmMapIoSpace(pa, (SIZE_T)n, MmWriteCombined);
        if (va == NULL) {
            SCM_LOG("PIL: no se pudo mapear pa=0x%llx len=0x%llx\n",
                    (unsigned long long)(Pa + done), (unsigned long long)n);
            return STATUS_INSUFFICIENT_RESOURCES;
        }

        if (done < SrcLen) {
            copy = SrcLen - done;
            if (copy > n) {
                copy = n;
            }
            RtlCopyMemory(va, Src + done, (SIZE_T)copy);
        }
        if (copy < n) {
            RtlZeroMemory(va + copy, (SIZE_T)(n - copy));
        }

        KeMemoryBarrier();
        MmUnmapIoSpace(va, (SIZE_T)n);
        done += n;
    }
    return STATUS_SUCCESS;
}

NTSTATUS PilBootAdsp(const PIL_ADSP_CONFIG *Cfg)
{
    const VOID *mdtData = NULL;
    SIZE_T mdtSize = 0;
    const ELF32_EHDR *eh;
    const ELF32_PHDR *ph;
    UINT64 minPa = ~0ull, maxPa = 0, regionStart, regionEnd, delta;
    BOOLEAN relocatable = FALSE;
    UINT32 i;
    NTSTATUS st;

    if (Cfg == NULL || Cfg->GetBlob == NULL) {
        return STATUS_INVALID_PARAMETER;
    }

    /* 1. adsp.mdt */
    st = Cfg->GetBlob(Cfg->Ctx, PIL_BLOB_MDT, &mdtData, &mdtSize);
    if (!NT_SUCCESS(st)) {
        SCM_LOG("PIL: no se pudo leer adsp.mdt nt=0x%08x\n", (unsigned)st);
        return st;
    }
    if (mdtSize < sizeof(ELF32_EHDR)) {
        SCM_LOG("PIL: el mdt es mas chico que un header ELF\n");
        return STATUS_INVALID_IMAGE_FORMAT;
    }

    eh = (const ELF32_EHDR *)mdtData;
    if (eh->e_ident[0] != 0x7F || eh->e_ident[1] != 'E' ||
        eh->e_ident[2] != 'L'  || eh->e_ident[3] != 'F') {
        SCM_LOG("PIL: el mdt no es un ELF\n");
        return STATUS_INVALID_IMAGE_FORMAT;
    }
    if (eh->e_phnum == 0 || eh->e_phentsize != sizeof(ELF32_PHDR) ||
        (UINT64)eh->e_phoff + (UINT64)eh->e_phnum * sizeof(ELF32_PHDR) > mdtSize) {
        SCM_LOG("PIL: program headers fuera del mdt (phnum=%u)\n",
                (unsigned)eh->e_phnum);
        return STATUS_INVALID_IMAGE_FORMAT;
    }
    ph = (const ELF32_PHDR *)((const UINT8 *)mdtData + eh->e_phoff);

    /* 2. region que ocupan los segmentos cargables */
    for (i = 0; i < eh->e_phnum; i++) {
        if (!SegIsLoadable(&ph[i])) {
            continue;
        }
        if (ph[i].p_flags & MDT_RELOCATABLE) {
            relocatable = TRUE;
        }
        if ((UINT64)ph[i].p_paddr < minPa) {
            minPa = ph[i].p_paddr;
        }
        if ((UINT64)ph[i].p_paddr + ph[i].p_memsz > maxPa) {
            maxPa = (UINT64)ph[i].p_paddr + ph[i].p_memsz;
        }
    }
    if (minPa == ~0ull) {
        SCM_LOG("PIL: no hay segmentos cargables\n");
        return STATUS_INVALID_IMAGE_FORMAT;
    }

    if (relocatable) {
        delta = Cfg->RegionBase - minPa;     /* aritmetica modular a proposito */
        regionStart = Cfg->RegionBase;
        regionEnd = PAGE_ALIGN_4K(Cfg->RegionBase + (maxPa - minPa));
    } else {
        delta = 0;
        regionStart = minPa;
        regionEnd = PAGE_ALIGN_4K(maxPa);
    }

    SCM_LOG("PIL: relocatable=%d region 0x%llx..0x%llx (%llu KiB)\n",
            (int)relocatable, (unsigned long long)regionStart,
            (unsigned long long)regionEnd,
            (unsigned long long)((regionEnd - regionStart) >> 10));

    if (regionStart < Cfg->RegionBase ||
        regionEnd > Cfg->RegionBase + Cfg->RegionSize) {
        SCM_LOG("PIL: la imagen no entra en la region reservada\n");
        return STATUS_INVALID_IMAGE_FORMAT;
    }

    /* 3. proxy vote (xo + vdd_cx) */
    if (Cfg->ProxyVote != NULL) {
        st = Cfg->ProxyVote(Cfg->Ctx);
        if (!NT_SUCCESS(st)) {
            SCM_LOG("PIL: fallo el proxy vote nt=0x%08x\n", (unsigned)st);
            return st;
        }
    }

    /* 4. INIT_IMAGE con el .mdt completo */
    st = PasInitImage(Cfg->PasId, mdtData, mdtSize);
    if (!NT_SUCCESS(st)) {
        return st;
    }

    /* 5. MEM_SETUP con la region calculada */
    st = PasMemSetup(Cfg->PasId, regionStart, regionEnd - regionStart);
    if (!NT_SUCCESS(st)) {
        goto fail;
    }

    /* 6. segmentos: adsp.bNN, NN = indice del program header */
    for (i = 0; i < eh->e_phnum; i++) {
        const VOID *blob = NULL;
        SIZE_T blobSize = 0;
        UINT64 dst;

        if (!SegIsLoadable(&ph[i])) {
            continue;
        }

        dst = (UINT64)ph[i].p_paddr + delta;

        if (ph[i].p_filesz != 0) {
            st = Cfg->GetBlob(Cfg->Ctx, i, &blob, &blobSize);
            if (!NT_SUCCESS(st)) {
                SCM_LOG("PIL: falta adsp.b%02u nt=0x%08x\n",
                        (unsigned)i, (unsigned)st);
                goto fail;
            }
            if (blobSize != ph[i].p_filesz) {
                SCM_LOG("PIL: adsp.b%02u mide %llu y el mdt dice %u\n",
                        (unsigned)i, (unsigned long long)blobSize,
                        (unsigned)ph[i].p_filesz);
                st = STATUS_INVALID_IMAGE_FORMAT;
                goto fail;
            }
        }

        st = FillPhys(dst, ph[i].p_memsz, (const UINT8 *)blob, ph[i].p_filesz);
        if (!NT_SUCCESS(st)) {
            goto fail;
        }
    }

    /* 7. sacar al ADSP de reset */
    st = PasAuthAndReset(Cfg->PasId);
    if (!NT_SUCCESS(st)) {
        goto fail;
    }

    SCM_LOG("PIL: ADSP fuera de reset\n");
    return STATUS_SUCCESS;

fail:
    /* pil_boot llama a deinit_image; aca solo dejamos el resultado */
    SCM_LOG("PIL: fallo la carga del ADSP nt=0x%08x\n", (unsigned)st);
    return st;
}
