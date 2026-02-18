

INCLUDE "cmsis/cmsis_f1/Source/Templates/gcc/linker/STM32F103XB_FLASH.ld"

MEMORY
{
PMA (W)    : ORIGIN = 0x40006000, LENGTH = 512 /* 256 x 16bit */
}

SECTIONS {
    /* USB/CAN Packet Memory Area (PMA) */
    .pma :
    {
    _pma_start = .; /* Start of PMA in real memory space */
    . = ALIGN(2);
    *(.pma)			/* Any variable defined the section .pma should add*/
    . = ALIGN(2);
    _pma_end = .; 	/* End of PMA in PMA space */
    } > PMA
}

