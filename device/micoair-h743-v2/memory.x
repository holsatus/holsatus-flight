MEMORY
{
    FLASH    : ORIGIN = 0x08000000, LENGTH = 1536K
    STORAGE  : ORIGIN = 0x08180000, LENGTH = 512K
    RAM      : ORIGIN = 0x24000000, LENGTH = 512K
    RAM_D3   : ORIGIN = 0x38000000, LENGTH = 64K
}

SECTIONS
{
    /* Place DMA buffers here to separate them from cached logic */
    .ram_d3 : {
        *(.ram_d3)
    } > RAM_D3
}
