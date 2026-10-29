; scm_smc.asm - envoltorio de "smc #0" para llamadas SCM hacia TrustZone.
; Version autocontenida para armasm64: sin #include, sin macros de ksarm64.h,
; asi que no necesita pasar por el preprocesador de C.
;
; VOID ScmSmcCall(SCM_SMC_REGS *Regs);
;
;   Regs->X[0..6] son x0..x6 de entrada y de salida.
;   Repite el SMC mientras x0 == 1 (SCM_INTERRUPTED), igual que
;   __scm_call_armv8_64() en drivers/soc/qcom/scm.c del kernel sm6125.
;
; x19 es callee-saved en el ABI de Windows ARM64: se guarda en la pila y guarda
; el puntero Regs, porque SMCCC permite que el SMC pise x4-x17. lr no se toca.
;
; Sin directivas de unwind: si una excepcion cae justo dentro de este stub, el
; desenrollado de pila saldria mal. El stub solo hace SMC y accesos a Regs.
;
; Ensamblar con:  armasm64.exe -nologo -o scm_smc.obj scm_smc.asm

        AREA    |.text|, CODE, READONLY, ALIGN=2
        EXPORT  ScmSmcCall

ScmSmcCall PROC
        str     x19, [sp, #-16]!
        mov     x19, x0

        ldp     x0, x1, [x19, #0]
        ldp     x2, x3, [x19, #16]
        ldp     x4, x5, [x19, #32]
        ldr     x6, [x19, #48]

ScmSmcRetry
        smc     #0
        cmp     x0, #1
        b.eq    ScmSmcRetry

        stp     x0, x1, [x19, #0]
        stp     x2, x3, [x19, #16]
        stp     x4, x5, [x19, #32]
        str     x6, [x19, #48]

        ldr     x19, [sp], #16
        ret
        ENDP

        END
