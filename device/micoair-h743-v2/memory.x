MEMORY
{
    FLASH    : ORIGIN = 0x08000000, LENGTH = 1536K
    STORAGE  : ORIGIN = 0x08180000, LENGTH = 512K
    RAM      : ORIGIN = 0x24000000, LENGTH = 512K
    RAM_D3   : ORIGIN = 0x38000000, LENGTH = 64K
}

SECTIONS
{
    /* DMA buffers that must stay out of the cached domains. SRAM4 lives in the
     * D3 domain and is not cached by the CPU D-cache, so DMA can use it without
     * cache maintenance. NOLOAD because it cannot participate in the
     * `__sdata..__edata` copy (that would make `__edata` jump into D3 and the
     * reset handler would try to copy ~320 MB). Contents are undefined at boot;
     * DMA buffers are fully written before each transfer. */
    .ram_d3 (NOLOAD) : ALIGN(4)
    {
        KEEP(*(.ram_d3 .ram_d3.*))
    } > RAM_D3
}

/* NOTE: keep this in its own `SECTIONS` block. `INSERT AFTER` applies to every
 * section in the block, so mixing it with `.ram_d3` would drag that section in
 * front of `__edata`. */
SECTIONS
{
    /* `linkme` emits its distributed-slice contents into orphan sections named
     * `linkme_<SLICE>`. Without an explicit placement the linker puts them in
     * RAM with LMA == VMA, so the reset code never loads their initializers and
     * the slice contains garbage. Fold them into the `.data` load mechanism. */
    linkme_PARAM_TABLES : ALIGN(4)
    {
        KEEP(*(linkme_PARAM_TABLES));
    } > RAM AT> FLASH
} INSERT AFTER .data;
