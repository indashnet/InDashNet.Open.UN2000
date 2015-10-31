OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)
SECTIONS
{
 . = (0x4A000000);
 . = ALIGN(4);
 .head :
 {
  arch/arm/cpu/armv7/sun7i/spare_head.o (.data)
 }
 . = ALIGN(1);
 .text :
 {
  __int_vector_start = .;
  arch/arm/cpu/armv7/start.o (.text*)
  __int_vector_end = .;
  *(EXCLUDE_FILE(board/sunxi/sun7i/standby/libstandby.o ).text)
 }
 . = ALIGN(16);
 .rodata :
 {
  *(EXCLUDE_FILE(board/sunxi/sun7i/standby/libstandby.o ).rodata*)
 }
 . = ALIGN(16);
 .data :
 {
  *(EXCLUDE_FILE(board/sunxi/sun7i/standby/libstandby.o ).data)
 }
 . = ALIGN(4);
 . = .;
 __u_boot_cmd_start = .;
 .u_boot_cmd : { *(.u_boot_cmd) }
 __u_boot_cmd_end = .;
 . = ALIGN(4);
 __image_copy_end = .;
 .rel.dyn : {
  __rel_dyn_start = .;
  *(.rel*)
  __rel_dyn_end = .;
 }
 .dynsym : {
  __dynsym_start = .;
  *(.dynsym)
 }
 _end = .;
 _standby_start_lma = .;
 .boot_standby (0x1000) :
 AT ( ADDR (.dynsym) + SIZEOF (.dynsym) )
 {
  _standby_start = .;
  board/sunxi/sun7i/standby/libstandby.o (.text .data .bss .rodata)
  _standby_end = .;
 }
 .bss __rel_dyn_start (OVERLAY) : {
  __bss_start = .;
  *(.bss)
   . = ALIGN(4);
  __bss_end__ = .;
 }
 /DISCARD/ : { *(.dynstr*) }
 /DISCARD/ : { *(.dynamic*) }
 /DISCARD/ : { *(.plt*) }
 /DISCARD/ : { *(.interp*) }
 /DISCARD/ : { *(.gnu*) }
}
