/* Memory layout for RP2350 (Pico 2 W) */
MEMORY
{
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 4096K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 520K
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS {
    .boot2 : {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;
