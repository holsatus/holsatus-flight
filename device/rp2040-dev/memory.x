MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    /* ### Boot loader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;

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
