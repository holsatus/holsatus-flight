/* STM32H743VIT6 memory layout
   Flash: 2MB starting at 0x08000000 (dual bank, 1MB each)
   AXI SRAM: 512KB at 0x24000000 (used as main RAM)
*/
MEMORY
{
    FLASH  : ORIGIN = 0x08000000, LENGTH = 2048K
    RAM    : ORIGIN = 0x24000000, LENGTH = 512K
}
