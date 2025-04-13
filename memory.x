MEMORY
{
  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  CCMRAM (xrw) : ORIGIN = 0x10000000, LENGTH = 64K
  /* The bootloader occupies 0x8000000 - 0x8004000 */
  FLASH (rx) : ORIGIN = 0x8004000, LENGTH = 1008K
  /* NOTE 1 K = 1 KiB = 1024 bytes */
}

/* Length of the contiguous stack placed at the beginning of the RAM region.
   The value must match the one in Hopter configuration parameters. */
_contiguous_stack_length = 0x1000;
