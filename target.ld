STARTUP(vectors.o)
ENTRY(reset_vector)
INPUT(libtarget.a)


GROUP(libtarget.a libgcc.a libsupc++.a)




OUTPUT_FORMAT(elf32-bigarm)
MEMORY
{
    ram : ORIGIN = 0, LENGTH = 0x04000000
}

SECTIONS
{
   .debug_aranges 0 : { *(.debug_aranges) } 
   .debug_pubnames 0 : { *(.debug_pubnames) } 
.debug_info 0 : { *(.debug_info) } 
.debug_abbrev 0 : { *(.debug_abbrev) } 
.debug_line 0 : { *(.debug_line) } 
.debug_frame 0 : { *(.debug_frame) } 
.debug_str 0 : { *(.debug_str) } 
.debug_loc 0 : { *(.debug_loc) } 
.debug_macinfo 0 : { *(.debug_macinfo) } 
.note.arm.ident 0 : { KEEP (*(.note.arm.ident)) }
    .rom_vectors 0x0 : 
	{ __rom_vectors_vma = ABSOLUTE(.); 
	. = .; 
	KEEP (*(.vectors)) } > ram __rom_vectors_lma = LOADADDR(.rom_vectors);
        .fixed_vectors 0x20 : { . = .; KEEP (*(.fixed_vectors)) } > ram
        
	.text ALIGN (0x4) : 
        { _stext = ABSOLUTE(.); 
	  PROVIDE (__stext = ABSOLUTE(.)); 
	*(.text*) *(.gnu.warning) 
	*(.gnu.linkonce.t.*) *(.init) 
	*(.glue_7) *(.glue_7t) } > ram _etext = .;
	
	 PROVIDE (__etext = .);
    .fini ALIGN (0x4) : { . = .; *(.fini) } > ram
    .rodata ALIGN (0x4) : { . = .; *(.rodata*) *(.gnu.linkonce.r.*) } > ram
    .rodata1 ALIGN (0x4) : { . = .; *(.rodata1) } > ram
    .fixup ALIGN (0x4) : { . = .; *(.fixup) } > ram
    .gcc_except_table ALIGN (0x4) : { . = .; *(.gcc_except_table) } > ram
    
    .data ALIGN (0x4) : 
	{ __ram_data_start = ABSOLUTE (.); 
	*(.data*) *(.data1) *(.gnu.linkonce.d.*) 
	. = ALIGN (4); 
	KEEP(*( SORT (.ecos.table.*))) ; 
	. = ALIGN (4); __CTOR_LIST__ = ABSOLUTE (.); 
	KEEP (*(SORT (.ctors*))) __CTOR_END__ = ABSOLUTE (.); 
	__DTOR_LIST__ = ABSOLUTE (.); 
	KEEP (*(SORT (.dtors*))) __DTOR_END__ = ABSOLUTE (.); 
	*(.dynamic) *(.sdata*) *(.gnu.linkonce.s.*) . = ALIGN (4); *(.2ram.*) } > ram 
	
	__rom_data_start = LOADADDR (.data); 
	__ram_data_end = .; 
	PROVIDE (__ram_data_end = .); 
	_edata = .; PROVIDE (edata = .); 
	PROVIDE (__rom_data_end = LOADADDR (.data) + SIZEOF(.data));

    .bss ALIGN (0x4) : 
	{ __bss_start = ABSOLUTE (.); 
	*(.scommon) *(.dynsbss) *(.sbss*) *(.gnu.linkonce.sb.*) 
	*(.dynbss) *(.bss*) *(.gnu.linkonce.b.*) 
	*(COMMON) __bss_end = ABSOLUTE (.); } > ram

    __heap1 = ALIGN (0x8);
    . = ALIGN(4); _end = .; PROVIDE (end = .);
}
