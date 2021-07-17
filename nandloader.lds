ENTRY(__start);

SECTIONS
{
	. = 0x9d0003F0;

	.nandheader : {
	  LONG(ADDR(.text) | 0x20000000) /* load addr in kseg1 */
	  LONG(ADDR(.text) | 0x20000000) /* entry addr in kseg1 */
	  LONG(__end - ADDR(.nandheader)) /* length including 16 byte header */
	  LONG(0xdeadbeef)  /* xor of all uint32s, excluding the 16 byte header */
	}

	.text : {
		*(.boot)
		*(.text*)
		*(.rodata*)
		*(.data*)
	}

        .bss : {
                _bstart = .;
                *(.bss*);
                . = ALIGN(4);
        }
        .bsize : {
                _bsize = .;
                LONG(SIZEOF(.bss));
        }

	__end = .;

	/DISCARD/ : {
		*(.MIPS.abiflags)
		*(.note.*)
		*(.reginfo)
	}
}
