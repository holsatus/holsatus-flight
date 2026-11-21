MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 768K
    STORAGE : ORIGIN = 0x080C0000, LENGTH = 256K
    RAM : ORIGIN = 0x20000000, LENGTH = 128K
}

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
