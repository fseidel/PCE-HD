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
    SMB0    $00
    LDX     $00
    SMB1    $00
    LDY     $00
    RMB0    $00
    LDA     $00
    
LOOP:   
    BRA     LOOP
    
.ORG $FFFA                 
    .DW     $FFFF ;NMI Vector
    .DW     $1000 ;RST vector
    .DW     $FFFF ;IRQ vctor
    
