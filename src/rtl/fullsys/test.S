.MEMORYMAP
    DEFAULTSLOT 0
    SLOTSIZE $10000
    SLOT 0 $0000
.ENDME

.ROMBANKMAP
    BANKSTOTAL 1
    BANKSIZE $10000
    BANKS 1
.ENDRO

.ORG $0000
INIT:
    SEI
    CSH
    LDA     #$FF
    TAM     #$01 ;MPR0 = FF (I/O Segment)
    LDA     #$F8
    TAM     #$02 ;MPR1 = F8 (RAM)
    LDX     #$FF
    TXS          ;SP init
MAIN:
    CLY
    STY     $0402 ;CTA[7:0] = 0
    STY     $0403 ;CTA[8] = 0
    TIA     PALDATA+$E000, $0404, 1024 ;write palette to VCE
    STY     $0402 ;CTA[7:0] = 0
    STY     $0403 ;CTA[8] = 0
    TAI     $0404, $2000, 1024 ;dump first 256 bytes of palette for verification
    CLX
LOOP:
    LDA     PALDATA+$E000, Y
    CMP     $00, X
    BNE     BAD
    TXA
    CMP     #$FF
    BEQ     GOOD
    INX
    INY
    BRA     LOOP
BAD:
    TXA
    LDX     #$BA
    LDY     #$AD
    BRA     DONE
GOOD:
    LDX     #$BE
    LDY     #$EF
DONE:
    BRA     DONE
PALDATA:
    .INCDIR ""
    .INCBIN "PAL.bin"
    
.ORG $1FFC                 
    .DW     $FFFF ;NMI Vector
    .DW     $E000 ;RST vector
    
