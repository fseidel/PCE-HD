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
    LDA     #$3
    STA     $02
    LDX     #$02
    CLA
    CLC
    SET
    ADC     #2
    LDA     $02
    PHA
MAIN:
    ST0     #0 ;zero MAWR
    ST1     #0 
    ST2     #0
    ST0     #2 ;put VDC into read/write mode
    CLX
    CLY
WRITELOOP:
    STY     $00, x
    INX
    INY
    BNE     WRITELOOP
    TIA     $2000, $0002, 256
    ST0     #1 ;zero MARR
    ST1     #0
    ST2     #0
    ST0     #2 ;put VDC into read/write mode
    TAI     $0002, $2200, 256
    ;TIA     GFXDATA+$E000, $0002, 256 ;write graphics to VDC
    ;TAI     $0002, $2000, 256         ;read them back into zeropage
    CLX
    CLY
LOOP:
    LDA     $00, x
    ;CMP     GFXDATA+$E000, y
    CMP     $2200, y
    BNE     BAD
    INX
    INY
    BEQ     GOOD
    BRA     LOOP
GOOD:
    TXA
    LDX     #$13
    LDY     #$37
    BRA     DONE
BAD:
    TXA
    LDX     #$BA
    LDY     #$AD
DONE:
    ;PLA
DONELOOP:   
    BRA     DONELOOP
GFXDATA:
    .INCDIR ""
    .INCBIN "GFX_256.bin"
    
.ORG $1FFC                 
    .DW     $FFFF ;NMI Vector
    .DW     $E000 ;RST vector
    
