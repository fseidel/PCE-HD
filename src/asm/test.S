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

.ORG $1000
INIT:
    LDA     #0
    TAM     #$01 ;MPR0 = 0
    LDA     #$F8
    TAM     #$02 ;MPR1 = F8
    LDA     #2
    TAM     #$04 ;MPR2 = 2
    
    CLA
    CLX
    CLY
    LDX     #$FF
    TXS
    LDA     #1
    LDX     #2
    LDY     #3 ;123
    SXY     ;132
    SAX     ;312
    SAY     ;213
    TII     INIT, $2200, $100
    JMP     LBL+$1200
    
LBL:    
    SMB0    $00
    LDX     $00 ;x = 1
    SMB1    $00
    LDY     $00 ;y = 3
    RMB0    $00
    LDA     $00 ;a = 2
    BBS0    $00, NOT_TAKEN
    BBR0    $00, TAKEN
LOOP:
    TMA     #$02  ;A = 1
    LDA     $BEEF ;magic number to halt test
    BRA     LOOP
NOT_TAKEN:
    LDA     #$FF
    BRA     LOOP
TAKEN2:
    LDX     #$80
    STX     $20
    LDX     #$41
    STX     $21
    LDX     #$1
    TST     #1, $20
    PHP
    PLX
    STX     $20
    LDX     #$1
    TST     #1, $20,x
    BEQ     NOT_TAKEN2
    PHP
    PLX
    STX     $21
    LDX     #$FF
    BRA     LOOP
TAKEN:
    LDA     #$0F
    BBR0    $00, TAKEN2
NOT_TAKEN2:
    LDX     #$BA
    LDY     #$D2
    LDA     $BEEF
    
.ORG $1FFC                 
    .DW     $FFFF ;NMI Vector
    .DW     $F000 ;RST vector
    
