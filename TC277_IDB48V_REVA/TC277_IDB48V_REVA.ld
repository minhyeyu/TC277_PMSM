OUTPUT_FORMAT("elf32-tricore")
OUTPUT_ARCH(tricore)
ENTRY(_RESET)


__ISTACK_SIZE = DEFINED (__ISTACK_SIZE) ? __ISTACK_SIZE : 8K;
__USTACK_SIZE = DEFINED (__USTACK_SIZE) ? __USTACK_SIZE : 8K;
__HEAP_SIZE = DEFINED (__HEAP_SIZE) ? __HEAP_SIZE : 4K ;
__CSA_SIZE = DEFINED (__CSA_SIZE) ? __CSA_SIZE : 4K ;

__INTTAB_START_ADDR        = 0x800F0000 ;


MEMORY
{

    PMU_PFLASH0_BMI (rx!p): org = 0x80000000, len = 32  /* boot header */
    PMU_STARTUP (rx!p)    : org = 0x80000020, len = 0xE0  /* startup code */
    PMU_TRAPTAB (rx!p): org = 0x80000100, len = 0x300 /* trap vector tables */
    PMU_INTTAB_TC0 (rx!p):  org = 0x800F0000, len = 0x2000 /* interrupt vector table */
    PMU_USER_FUNCS (rx!p):  org = 0x8002E000, len = 0x2000

    PMU_INTTAB_TC1 (rx!p):  org = 0x80001500, len = 0xC00 /* interrupt vector table */
    PMU_INTTAB_TC2 (rx!p):  org = 0x80002100, len = 0xC00 /* interrupt vector table */
        
    PMU_PFLASH0 (rx!p)  : org = 0x80030000, len = 320K /*Program Flash Memory (PFLASH0)*/
    PMU_PFLASH0_CONST (rx!p): org = 0x80080000, len = 128K    
    PMU_PFLASH1 (rx!p)  : org = 0x80200000, len = 2M  /*Program Flash Memory (PFLASH1)*/
    CONFIG_DATA (rx!p): org = 0xA0100000, len = 20K  
    CONFIG_DIO_DATA (rx!p): org = 0xA0140000, len = 512  /*Program Flash Memory (PFLASH1)*/ 
    

    CPU0_PMI_PSPR (wx!p): org = 0x70100000, len = 24K   /*Scratch-Pad RAM (PSPR)*/
    CPU1_PMI_PSPR (wx!p): org = 0x60100000, len = 32K   /*Scratch-Pad RAM (PSPR)*/
    CPU2_PMI_PSPR (wx!p): org = 0x50100000, len = 32K   /*Scratch-Pad RAM (PSPR)*/
    /* 0x100 bytes of DSPR is reserved for sbst_tcx_dspr */
    CPU0_DMI_DSPR (w!xp): org = 0x70000100, len = 112K - 12K  /*Local Data RAM (DSPR)*/
    CPU1_DMI_DSPR (w!xp): org = 0x60000100, len = 120K - 12K  /*Local Data RAM (DSPR)*/
    CPU2_DMI_DSPR (w!xp): org = 0x50000100, len = 120K - 12K  /*Local Data RAM (DSPR)*/
    DMI_DSPR (w!xp)     : org = 0xD0019000, len = 12K /*Local Data RAM (DSPR)*/


    LMU_SRAM (w!xp)     : org = 0xB0000000, len = 32K  /*Local Data RAM (DSPR)*/

    /* reserve memory for flash routines */
    FLS_ERASE_PFLASH0 (rx!p): org = 0x80001100, len = 0x200  /*Program Flash Memory (PFLASH0)*/
    FLS_WRITE_PFLASH0 (rx!p): org = 0x80001300, len = 0x200  /*Program Flash Memory (PFLASH0)*/
    FLSLOADER_RAM_CODE (w!xp): org = 0x70101000, len = 20k /* PSPR for Flsloader demo */
    FLS_ERASE_SRAM (w!xp):  org = 0x701011C8, len = 0x200  /*Global Scratch-Pad RAM (PSPR)*/
    FLS_WRITE_SRAM (w!xp):  org = 0x70101100, len = 0x200  /*Global Scratch-Pad RAM (PSPR)*/

       
}


SECTIONS
{

  /* Code Allocation */
  .BMD_HDR_CONST_FAR_UNSPECIFIED   :
  {
    KEEP(*(.rodata.BMD_HDR_CONST_FAR_UNSPECIFIED))
  } > PMU_PFLASH0_BMI

  /*
   * The startup code should be placed where the CPU expects it after a reset,
   * so we try to locate it first, no matter where it appears in the list of
   * objects and libraries (note: because the wildcard pattern doesn't match
   * directories, we'll try to find crt0.o in various (sub)directories).
   */
  .startup :
  {
    KEEP (*(.startup_code))
    . = ALIGN(8);
  } > PMU_STARTUP
  


  /*
  * Allocate trap and interrupt vector tables.
  */

  .CPU0_TRAP_HANDLER 0x80000100:  ALIGN(32) FLAGS(axl)
  {
    . = ALIGN(32) ;
    PROVIDE(__CPU0_TRAP_HANDLER_CODE_ROM_start = .);
    KEEP(*(.text.CPU0_TRAP_HANDLER_CODE_ROM))
    . = ALIGN(32) ;
    PROVIDE(__CPU0_TRAP_HANDLER_CODE_ROM_end = .);
  } > PMU_TRAPTAB

  .CPU1_TRAP_HANDLER 0x80000200:  ALIGN(32) FLAGS(axl)
  {
    . = ALIGN(32) ;
    PROVIDE(__CPU1_TRAP_HANDLER_CODE_ROM_start = .);
    KEEP(*(.text.CPU1_TRAP_HANDLER_CODE_ROM))
    . = ALIGN(32) ;
    PROVIDE(__CPU1_TRAP_HANDLER_CODE_ROM_end = .);
  } > PMU_TRAPTAB
  .CPU2_TRAP_HANDLER 0x80000300:  ALIGN(32) FLAGS(axl)
  {
    . = ALIGN(32) ;
    PROVIDE(__CPU2_TRAP_HANDLER_CODE_ROM_start = .);
    KEEP(*(.text.CPU2_TRAP_HANDLER_CODE_ROM))
    . = ALIGN(32) ;
    PROVIDE(__CPU2_TRAP_HANDLER_CODE_ROM_end = .);
  } > PMU_TRAPTAB

   /* Section for interrupt table */
  .inttab_tc0_000 (__INTTAB_START_ADDR + 0x0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_0)); } > PMU_INTTAB_TC0
  .inttab_tc0_001 (__INTTAB_START_ADDR + 0x20) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_1)); } > PMU_INTTAB_TC0
  .inttab_tc0_002 (__INTTAB_START_ADDR + 0x40) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_2)); } > PMU_INTTAB_TC0
  .inttab_tc0_003 (__INTTAB_START_ADDR + 0x60) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_3)); } > PMU_INTTAB_TC0
  .inttab_tc0_004 (__INTTAB_START_ADDR + 0x80) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_4)); } > PMU_INTTAB_TC0
  .inttab_tc0_005 (__INTTAB_START_ADDR + 0xA0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_5)); } > PMU_INTTAB_TC0
  .inttab_tc0_006 (__INTTAB_START_ADDR + 0xC0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_6)); } > PMU_INTTAB_TC0
  .inttab_tc0_007 (__INTTAB_START_ADDR + 0xE0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_7)); } > PMU_INTTAB_TC0
  .inttab_tc0_008 (__INTTAB_START_ADDR + 0x100) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_8)); } > PMU_INTTAB_TC0
  .inttab_tc0_009 (__INTTAB_START_ADDR + 0x120) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_9)); } > PMU_INTTAB_TC0
  .inttab_tc0_00A (__INTTAB_START_ADDR + 0x140) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_10)); } > PMU_INTTAB_TC0
  .inttab_tc0_00B (__INTTAB_START_ADDR + 0x160) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_11)); } > PMU_INTTAB_TC0
  .inttab_tc0_00C (__INTTAB_START_ADDR + 0x180) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_12)); } > PMU_INTTAB_TC0
  .inttab_tc0_00D (__INTTAB_START_ADDR + 0x1A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_13)); } > PMU_INTTAB_TC0
  .inttab_tc0_00E (__INTTAB_START_ADDR + 0x1C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_14)); } > PMU_INTTAB_TC0
  .inttab_tc0_00F (__INTTAB_START_ADDR + 0x1E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_15)); } > PMU_INTTAB_TC0
  .inttab_tc0_010 (__INTTAB_START_ADDR + 0x200) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_16)); } > PMU_INTTAB_TC0
  .inttab_tc0_011 (__INTTAB_START_ADDR + 0x220) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_17)); } > PMU_INTTAB_TC0
  .inttab_tc0_012 (__INTTAB_START_ADDR + 0x240) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_18)); } > PMU_INTTAB_TC0
  .inttab_tc0_013 (__INTTAB_START_ADDR + 0x260) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_19)); } > PMU_INTTAB_TC0
  .inttab_tc0_014 (__INTTAB_START_ADDR + 0x280) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_20)); } > PMU_INTTAB_TC0
  .inttab_tc0_015 (__INTTAB_START_ADDR + 0x2A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_21)); } > PMU_INTTAB_TC0
  .inttab_tc0_016 (__INTTAB_START_ADDR + 0x2C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_22)); } > PMU_INTTAB_TC0
  .inttab_tc0_017 (__INTTAB_START_ADDR + 0x2E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_23)); } > PMU_INTTAB_TC0
  .inttab_tc0_018 (__INTTAB_START_ADDR + 0x300) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_24)); } > PMU_INTTAB_TC0
  .inttab_tc0_019 (__INTTAB_START_ADDR + 0x320) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_25)); } > PMU_INTTAB_TC0
  .inttab_tc0_01A (__INTTAB_START_ADDR + 0x340) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_26)); } > PMU_INTTAB_TC0
  .inttab_tc0_01B (__INTTAB_START_ADDR + 0x360) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_27)); } > PMU_INTTAB_TC0
  .inttab_tc0_01C (__INTTAB_START_ADDR + 0x380) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_28)); } > PMU_INTTAB_TC0
  .inttab_tc0_01D (__INTTAB_START_ADDR + 0x3A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_29)); } > PMU_INTTAB_TC0
  .inttab_tc0_01E (__INTTAB_START_ADDR + 0x3C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_30)); } > PMU_INTTAB_TC0
  .inttab_tc0_01F (__INTTAB_START_ADDR + 0x3E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_31)); } > PMU_INTTAB_TC0
  .inttab_tc0_020 (__INTTAB_START_ADDR + 0x400) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_32)); } > PMU_INTTAB_TC0
  .inttab_tc0_021 (__INTTAB_START_ADDR + 0x420) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_33)); } > PMU_INTTAB_TC0
  .inttab_tc0_022 (__INTTAB_START_ADDR + 0x440) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_34)); } > PMU_INTTAB_TC0
  .inttab_tc0_023 (__INTTAB_START_ADDR + 0x460) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_35)); } > PMU_INTTAB_TC0
  .inttab_tc0_024 (__INTTAB_START_ADDR + 0x480) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_36)); } > PMU_INTTAB_TC0
  .inttab_tc0_025 (__INTTAB_START_ADDR + 0x4A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_37)); } > PMU_INTTAB_TC0
  .inttab_tc0_026 (__INTTAB_START_ADDR + 0x4C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_38)); } > PMU_INTTAB_TC0
  .inttab_tc0_027 (__INTTAB_START_ADDR + 0x4E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_39)); } > PMU_INTTAB_TC0
  .inttab_tc0_028 (__INTTAB_START_ADDR + 0x500) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_40)); } > PMU_INTTAB_TC0
  .inttab_tc0_029 (__INTTAB_START_ADDR + 0x520) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_41)); } > PMU_INTTAB_TC0
  .inttab_tc0_02A (__INTTAB_START_ADDR + 0x540) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_42)); } > PMU_INTTAB_TC0
  .inttab_tc0_02B (__INTTAB_START_ADDR + 0x560) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_43)); } > PMU_INTTAB_TC0
  .inttab_tc0_02C (__INTTAB_START_ADDR + 0x580) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_44)); } > PMU_INTTAB_TC0
  .inttab_tc0_02D (__INTTAB_START_ADDR + 0x5A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_45)); } > PMU_INTTAB_TC0
  .inttab_tc0_02E (__INTTAB_START_ADDR + 0x5C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_46)); } > PMU_INTTAB_TC0
  .inttab_tc0_02F (__INTTAB_START_ADDR + 0x5E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_47)); } > PMU_INTTAB_TC0
  .inttab_tc0_030 (__INTTAB_START_ADDR + 0x600) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_48)); } > PMU_INTTAB_TC0
  .inttab_tc0_031 (__INTTAB_START_ADDR + 0x620) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_49)); } > PMU_INTTAB_TC0
  .inttab_tc0_032 (__INTTAB_START_ADDR + 0x640) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_50)); } > PMU_INTTAB_TC0
  .inttab_tc0_033 (__INTTAB_START_ADDR + 0x660) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_51)); } > PMU_INTTAB_TC0
  .inttab_tc0_034 (__INTTAB_START_ADDR + 0x680) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_52)); } > PMU_INTTAB_TC0
  .inttab_tc0_035 (__INTTAB_START_ADDR + 0x6A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_53)); } > PMU_INTTAB_TC0
  .inttab_tc0_036 (__INTTAB_START_ADDR + 0x6C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_54)); } > PMU_INTTAB_TC0
  .inttab_tc0_037 (__INTTAB_START_ADDR + 0x6E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_55)); } > PMU_INTTAB_TC0
  .inttab_tc0_038 (__INTTAB_START_ADDR + 0x700) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_56)); } > PMU_INTTAB_TC0
  .inttab_tc0_039 (__INTTAB_START_ADDR + 0x720) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_57)); } > PMU_INTTAB_TC0
  .inttab_tc0_03A (__INTTAB_START_ADDR + 0x740) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_58)); } > PMU_INTTAB_TC0
  .inttab_tc0_03B (__INTTAB_START_ADDR + 0x760) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_59)); } > PMU_INTTAB_TC0
  .inttab_tc0_03C (__INTTAB_START_ADDR + 0x780) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_60)); } > PMU_INTTAB_TC0
  .inttab_tc0_03D (__INTTAB_START_ADDR + 0x7A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_61)); } > PMU_INTTAB_TC0
  .inttab_tc0_03E (__INTTAB_START_ADDR + 0x7C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_62)); } > PMU_INTTAB_TC0
  .inttab_tc0_03F (__INTTAB_START_ADDR + 0x7E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_63)); } > PMU_INTTAB_TC0
  .inttab_tc0_040 (__INTTAB_START_ADDR + 0x800) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_64)); } > PMU_INTTAB_TC0
  .inttab_tc0_041 (__INTTAB_START_ADDR + 0x820) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_65)); } > PMU_INTTAB_TC0
  .inttab_tc0_042 (__INTTAB_START_ADDR + 0x840) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_66)); } > PMU_INTTAB_TC0
  .inttab_tc0_043 (__INTTAB_START_ADDR + 0x860) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_67)); } > PMU_INTTAB_TC0
  .inttab_tc0_044 (__INTTAB_START_ADDR + 0x880) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_68)); } > PMU_INTTAB_TC0
  .inttab_tc0_045 (__INTTAB_START_ADDR + 0x8A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_69)); } > PMU_INTTAB_TC0
  .inttab_tc0_046 (__INTTAB_START_ADDR + 0x8C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_70)); } > PMU_INTTAB_TC0
  .inttab_tc0_047 (__INTTAB_START_ADDR + 0x8E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_71)); } > PMU_INTTAB_TC0
  .inttab_tc0_048 (__INTTAB_START_ADDR + 0x900) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_72)); } > PMU_INTTAB_TC0
  .inttab_tc0_049 (__INTTAB_START_ADDR + 0x920) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_73)); } > PMU_INTTAB_TC0
  .inttab_tc0_04A (__INTTAB_START_ADDR + 0x940) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_74)); } > PMU_INTTAB_TC0
  .inttab_tc0_04B (__INTTAB_START_ADDR + 0x960) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_75)); } > PMU_INTTAB_TC0
  .inttab_tc0_04C (__INTTAB_START_ADDR + 0x980) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_76)); } > PMU_INTTAB_TC0
  .inttab_tc0_04D (__INTTAB_START_ADDR + 0x9A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_77)); } > PMU_INTTAB_TC0
  .inttab_tc0_04E (__INTTAB_START_ADDR + 0x9C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_78)); } > PMU_INTTAB_TC0
  .inttab_tc0_04F (__INTTAB_START_ADDR + 0x9E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_79)); } > PMU_INTTAB_TC0
  .inttab_tc0_050 (__INTTAB_START_ADDR + 0xA00) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_80)); } > PMU_INTTAB_TC0
  .inttab_tc0_051 (__INTTAB_START_ADDR + 0xA20) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_81)); } > PMU_INTTAB_TC0
  .inttab_tc0_052 (__INTTAB_START_ADDR + 0xA40) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_82)); } > PMU_INTTAB_TC0
  .inttab_tc0_053 (__INTTAB_START_ADDR + 0xA60) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_83)); } > PMU_INTTAB_TC0
  .inttab_tc0_054 (__INTTAB_START_ADDR + 0xA80) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_84)); } > PMU_INTTAB_TC0
  .inttab_tc0_055 (__INTTAB_START_ADDR + 0xAA0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_85)); } > PMU_INTTAB_TC0
  .inttab_tc0_056 (__INTTAB_START_ADDR + 0xAC0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_86)); } > PMU_INTTAB_TC0
  .inttab_tc0_057 (__INTTAB_START_ADDR + 0xAE0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_87)); } > PMU_INTTAB_TC0
  .inttab_tc0_058 (__INTTAB_START_ADDR + 0xB00) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_88)); } > PMU_INTTAB_TC0
  .inttab_tc0_059 (__INTTAB_START_ADDR + 0xB20) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_89)); } > PMU_INTTAB_TC0
  .inttab_tc0_05A (__INTTAB_START_ADDR + 0xB40) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_90)); } > PMU_INTTAB_TC0
  .inttab_tc0_05B (__INTTAB_START_ADDR + 0xB60) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_91)); } > PMU_INTTAB_TC0
  .inttab_tc0_05C (__INTTAB_START_ADDR + 0xB80) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_92)); } > PMU_INTTAB_TC0
  .inttab_tc0_05D (__INTTAB_START_ADDR + 0xBA0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_93)); } > PMU_INTTAB_TC0
  .inttab_tc0_05E (__INTTAB_START_ADDR + 0xBC0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_94)); } > PMU_INTTAB_TC0
  .inttab_tc0_05F (__INTTAB_START_ADDR + 0xBE0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_95)); } > PMU_INTTAB_TC0
  .inttab_tc0_060 (__INTTAB_START_ADDR + 0xC00) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_96)); } > PMU_INTTAB_TC0
  .inttab_tc0_061 (__INTTAB_START_ADDR + 0xC20) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_97)); } > PMU_INTTAB_TC0
  .inttab_tc0_062 (__INTTAB_START_ADDR + 0xC40) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_98)); } > PMU_INTTAB_TC0
  .inttab_tc0_063 (__INTTAB_START_ADDR + 0xC60) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_99)); } > PMU_INTTAB_TC0
  .inttab_tc0_064 (__INTTAB_START_ADDR + 0xC80) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_100)); } > PMU_INTTAB_TC0
  .inttab_tc0_065 (__INTTAB_START_ADDR + 0xCA0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_101)); } > PMU_INTTAB_TC0
  .inttab_tc0_066 (__INTTAB_START_ADDR + 0xCC0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_102)); } > PMU_INTTAB_TC0
  .inttab_tc0_067 (__INTTAB_START_ADDR + 0xCE0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_103)); } > PMU_INTTAB_TC0
  .inttab_tc0_068 (__INTTAB_START_ADDR + 0xD00) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_104)); } > PMU_INTTAB_TC0
  .inttab_tc0_069 (__INTTAB_START_ADDR + 0xD20) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_105)); } > PMU_INTTAB_TC0
  .inttab_tc0_06A (__INTTAB_START_ADDR + 0xD40) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_106)); } > PMU_INTTAB_TC0
  .inttab_tc0_06B (__INTTAB_START_ADDR + 0xD60) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_107)); } > PMU_INTTAB_TC0
  .inttab_tc0_06C (__INTTAB_START_ADDR + 0xD80) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_108)); } > PMU_INTTAB_TC0
  .inttab_tc0_06D (__INTTAB_START_ADDR + 0xDA0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_109)); } > PMU_INTTAB_TC0
  .inttab_tc0_06E (__INTTAB_START_ADDR + 0xDC0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_110)); } > PMU_INTTAB_TC0
  .inttab_tc0_06F (__INTTAB_START_ADDR + 0xDE0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_111)); } > PMU_INTTAB_TC0
  .inttab_tc0_070 (__INTTAB_START_ADDR + 0xE00) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_112)); } > PMU_INTTAB_TC0
  .inttab_tc0_071 (__INTTAB_START_ADDR + 0xE20) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_113)); } > PMU_INTTAB_TC0
  .inttab_tc0_072 (__INTTAB_START_ADDR + 0xE40) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_114)); } > PMU_INTTAB_TC0
  .inttab_tc0_073 (__INTTAB_START_ADDR + 0xE60) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_115)); } > PMU_INTTAB_TC0
  .inttab_tc0_074 (__INTTAB_START_ADDR + 0xE80) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_116)); } > PMU_INTTAB_TC0
  .inttab_tc0_075 (__INTTAB_START_ADDR + 0xEA0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_117)); } > PMU_INTTAB_TC0
  .inttab_tc0_076 (__INTTAB_START_ADDR + 0xEC0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_118)); } > PMU_INTTAB_TC0
  .inttab_tc0_077 (__INTTAB_START_ADDR + 0xEE0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_119)); } > PMU_INTTAB_TC0
  .inttab_tc0_078 (__INTTAB_START_ADDR + 0xF00) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_120)); } > PMU_INTTAB_TC0
  .inttab_tc0_079 (__INTTAB_START_ADDR + 0xF20) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_121)); } > PMU_INTTAB_TC0
  .inttab_tc0_07A (__INTTAB_START_ADDR + 0xF40) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_122)); } > PMU_INTTAB_TC0
  .inttab_tc0_07B (__INTTAB_START_ADDR + 0xF60) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_123)); } > PMU_INTTAB_TC0
  .inttab_tc0_07C (__INTTAB_START_ADDR + 0xF80) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_124)); } > PMU_INTTAB_TC0
  .inttab_tc0_07D (__INTTAB_START_ADDR + 0xFA0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_125)); } > PMU_INTTAB_TC0
  .inttab_tc0_07E (__INTTAB_START_ADDR + 0xFC0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_126)); } > PMU_INTTAB_TC0
  .inttab_tc0_07F (__INTTAB_START_ADDR + 0xFE0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_127)); } > PMU_INTTAB_TC0
  .inttab_tc0_080 (__INTTAB_START_ADDR + 0x1000) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_128)); } > PMU_INTTAB_TC0
  .inttab_tc0_081 (__INTTAB_START_ADDR + 0x1020) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_129)); } > PMU_INTTAB_TC0
  .inttab_tc0_082 (__INTTAB_START_ADDR + 0x1040) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_130)); } > PMU_INTTAB_TC0
  .inttab_tc0_083 (__INTTAB_START_ADDR + 0x1060) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_131)); } > PMU_INTTAB_TC0
  .inttab_tc0_084 (__INTTAB_START_ADDR + 0x1080) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_132)); } > PMU_INTTAB_TC0
  .inttab_tc0_085 (__INTTAB_START_ADDR + 0x10A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_133)); } > PMU_INTTAB_TC0
  .inttab_tc0_086 (__INTTAB_START_ADDR + 0x10C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_134)); } > PMU_INTTAB_TC0
  .inttab_tc0_087 (__INTTAB_START_ADDR + 0x10E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_135)); } > PMU_INTTAB_TC0
  .inttab_tc0_088 (__INTTAB_START_ADDR + 0x1100) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_136)); } > PMU_INTTAB_TC0
  .inttab_tc0_089 (__INTTAB_START_ADDR + 0x1120) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_137)); } > PMU_INTTAB_TC0
  .inttab_tc0_08A (__INTTAB_START_ADDR + 0x1140) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_138)); } > PMU_INTTAB_TC0
  .inttab_tc0_08B (__INTTAB_START_ADDR + 0x1160) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_139)); } > PMU_INTTAB_TC0
  .inttab_tc0_08C (__INTTAB_START_ADDR + 0x1180) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_140)); } > PMU_INTTAB_TC0
  .inttab_tc0_08D (__INTTAB_START_ADDR + 0x11A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_141)); } > PMU_INTTAB_TC0
  .inttab_tc0_08E (__INTTAB_START_ADDR + 0x11C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_142)); } > PMU_INTTAB_TC0
  .inttab_tc0_08F (__INTTAB_START_ADDR + 0x11E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_143)); } > PMU_INTTAB_TC0
  .inttab_tc0_090 (__INTTAB_START_ADDR + 0x1200) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_144)); } > PMU_INTTAB_TC0
  .inttab_tc0_091 (__INTTAB_START_ADDR + 0x1220) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_145)); } > PMU_INTTAB_TC0
  .inttab_tc0_092 (__INTTAB_START_ADDR + 0x1240) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_146)); } > PMU_INTTAB_TC0
  .inttab_tc0_093 (__INTTAB_START_ADDR + 0x1260) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_147)); } > PMU_INTTAB_TC0
  .inttab_tc0_094 (__INTTAB_START_ADDR + 0x1280) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_148)); } > PMU_INTTAB_TC0
  .inttab_tc0_095 (__INTTAB_START_ADDR + 0x12A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_149)); } > PMU_INTTAB_TC0
  .inttab_tc0_096 (__INTTAB_START_ADDR + 0x12C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_150)); } > PMU_INTTAB_TC0
  .inttab_tc0_097 (__INTTAB_START_ADDR + 0x12E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_151)); } > PMU_INTTAB_TC0
  .inttab_tc0_098 (__INTTAB_START_ADDR + 0x1300) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_152)); } > PMU_INTTAB_TC0
  .inttab_tc0_099 (__INTTAB_START_ADDR + 0x1320) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_153)); } > PMU_INTTAB_TC0
  .inttab_tc0_09A (__INTTAB_START_ADDR + 0x1340) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_154)); } > PMU_INTTAB_TC0
  .inttab_tc0_09B (__INTTAB_START_ADDR + 0x1360) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_155)); } > PMU_INTTAB_TC0
  .inttab_tc0_09C (__INTTAB_START_ADDR + 0x1380) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_156)); } > PMU_INTTAB_TC0
  .inttab_tc0_09D (__INTTAB_START_ADDR + 0x13A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_157)); } > PMU_INTTAB_TC0
  .inttab_tc0_09E (__INTTAB_START_ADDR + 0x13C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_158)); } > PMU_INTTAB_TC0
  .inttab_tc0_09F (__INTTAB_START_ADDR + 0x13E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_159)); } > PMU_INTTAB_TC0
  .inttab_tc0_0A0 (__INTTAB_START_ADDR + 0x1400) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_160)); } > PMU_INTTAB_TC0
  .inttab_tc0_0A1 (__INTTAB_START_ADDR + 0x1420) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_161)); } > PMU_INTTAB_TC0
  .inttab_tc0_0A2 (__INTTAB_START_ADDR + 0x1440) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_162)); } > PMU_INTTAB_TC0
  .inttab_tc0_0A3 (__INTTAB_START_ADDR + 0x1460) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_163)); } > PMU_INTTAB_TC0
  .inttab_tc0_0A4 (__INTTAB_START_ADDR + 0x1480) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_164)); } > PMU_INTTAB_TC0
  .inttab_tc0_0A5 (__INTTAB_START_ADDR + 0x14A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_165)); } > PMU_INTTAB_TC0
  .inttab_tc0_0A6 (__INTTAB_START_ADDR + 0x14C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_166)); } > PMU_INTTAB_TC0
  .inttab_tc0_0A7 (__INTTAB_START_ADDR + 0x14E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_167)); } > PMU_INTTAB_TC0
  .inttab_tc0_0A8 (__INTTAB_START_ADDR + 0x1500) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_168)); } > PMU_INTTAB_TC0
  .inttab_tc0_0A9 (__INTTAB_START_ADDR + 0x1520) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_169)); } > PMU_INTTAB_TC0
  .inttab_tc0_0AA (__INTTAB_START_ADDR + 0x1540) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_170)); } > PMU_INTTAB_TC0
  .inttab_tc0_0AB (__INTTAB_START_ADDR + 0x1560) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_171)); } > PMU_INTTAB_TC0
  .inttab_tc0_0AC (__INTTAB_START_ADDR + 0x1580) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_172)); } > PMU_INTTAB_TC0
  .inttab_tc0_0AD (__INTTAB_START_ADDR + 0x15A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_173)); } > PMU_INTTAB_TC0
  .inttab_tc0_0AE (__INTTAB_START_ADDR + 0x15C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_174)); } > PMU_INTTAB_TC0
  .inttab_tc0_0AF (__INTTAB_START_ADDR + 0x15E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_175)); } > PMU_INTTAB_TC0
  .inttab_tc0_0B0 (__INTTAB_START_ADDR + 0x1600) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_176)); } > PMU_INTTAB_TC0
  .inttab_tc0_0B1 (__INTTAB_START_ADDR + 0x1620) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_177)); } > PMU_INTTAB_TC0
  .inttab_tc0_0B2 (__INTTAB_START_ADDR + 0x1640) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_178)); } > PMU_INTTAB_TC0
  .inttab_tc0_0B3 (__INTTAB_START_ADDR + 0x1660) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_179)); } > PMU_INTTAB_TC0
  .inttab_tc0_0B4 (__INTTAB_START_ADDR + 0x1680) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_180)); } > PMU_INTTAB_TC0
  .inttab_tc0_0B5 (__INTTAB_START_ADDR + 0x16A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_181)); } > PMU_INTTAB_TC0
  .inttab_tc0_0B6 (__INTTAB_START_ADDR + 0x16C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_182)); } > PMU_INTTAB_TC0
  .inttab_tc0_0B7 (__INTTAB_START_ADDR + 0x16E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_183)); } > PMU_INTTAB_TC0
  .inttab_tc0_0B8 (__INTTAB_START_ADDR + 0x1700) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_184)); } > PMU_INTTAB_TC0
  .inttab_tc0_0B9 (__INTTAB_START_ADDR + 0x1720) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_185)); } > PMU_INTTAB_TC0
  .inttab_tc0_0BA (__INTTAB_START_ADDR + 0x1740) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_186)); } > PMU_INTTAB_TC0
  .inttab_tc0_0BB (__INTTAB_START_ADDR + 0x1760) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_187)); } > PMU_INTTAB_TC0
  .inttab_tc0_0BC (__INTTAB_START_ADDR + 0x1780) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_188)); } > PMU_INTTAB_TC0
  .inttab_tc0_0BD (__INTTAB_START_ADDR + 0x17A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_189)); } > PMU_INTTAB_TC0
  .inttab_tc0_0BE (__INTTAB_START_ADDR + 0x17C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_190)); } > PMU_INTTAB_TC0
  .inttab_tc0_0BF (__INTTAB_START_ADDR + 0x17E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_191)); } > PMU_INTTAB_TC0
  .inttab_tc0_0C0 (__INTTAB_START_ADDR + 0x1800) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_192)); } > PMU_INTTAB_TC0
  .inttab_tc0_0C1 (__INTTAB_START_ADDR + 0x1820) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_193)); } > PMU_INTTAB_TC0
  .inttab_tc0_0C2 (__INTTAB_START_ADDR + 0x1840) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_194)); } > PMU_INTTAB_TC0
  .inttab_tc0_0C3 (__INTTAB_START_ADDR + 0x1860) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_195)); } > PMU_INTTAB_TC0
  .inttab_tc0_0C4 (__INTTAB_START_ADDR + 0x1880) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_196)); } > PMU_INTTAB_TC0
  .inttab_tc0_0C5 (__INTTAB_START_ADDR + 0x18A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_197)); } > PMU_INTTAB_TC0
  .inttab_tc0_0C6 (__INTTAB_START_ADDR + 0x18C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_198)); } > PMU_INTTAB_TC0
  .inttab_tc0_0C7 (__INTTAB_START_ADDR + 0x18E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_199)); } > PMU_INTTAB_TC0
  .inttab_tc0_0C8 (__INTTAB_START_ADDR + 0x1900) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_200)); } > PMU_INTTAB_TC0
  .inttab_tc0_0C9 (__INTTAB_START_ADDR + 0x1920) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_201)); } > PMU_INTTAB_TC0
  .inttab_tc0_0CA (__INTTAB_START_ADDR + 0x1940) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_202)); } > PMU_INTTAB_TC0
  .inttab_tc0_0CB (__INTTAB_START_ADDR + 0x1960) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_203)); } > PMU_INTTAB_TC0
  .inttab_tc0_0CC (__INTTAB_START_ADDR + 0x1980) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_204)); } > PMU_INTTAB_TC0
  .inttab_tc0_0CD (__INTTAB_START_ADDR + 0x19A0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_205)); } > PMU_INTTAB_TC0
  .inttab_tc0_0CE (__INTTAB_START_ADDR + 0x19C0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_206)); } > PMU_INTTAB_TC0
  .inttab_tc0_0CF (__INTTAB_START_ADDR + 0x19E0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_207)); } > PMU_INTTAB_TC0
  .inttab_tc0_0D0 (__INTTAB_START_ADDR + 0x1A00) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_208)); } > PMU_INTTAB_TC0
  .inttab_tc0_0D1 (__INTTAB_START_ADDR + 0x1A20) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_209)); } > PMU_INTTAB_TC0
  .inttab_tc0_0D2 (__INTTAB_START_ADDR + 0x1A40) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_210)); } > PMU_INTTAB_TC0
  .inttab_tc0_0D3 (__INTTAB_START_ADDR + 0x1A60) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_211)); } > PMU_INTTAB_TC0
  .inttab_tc0_0D4 (__INTTAB_START_ADDR + 0x1A80) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_212)); } > PMU_INTTAB_TC0
  .inttab_tc0_0D5 (__INTTAB_START_ADDR + 0x1AA0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_213)); } > PMU_INTTAB_TC0
  .inttab_tc0_0D6 (__INTTAB_START_ADDR + 0x1AC0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_214)); } > PMU_INTTAB_TC0
  .inttab_tc0_0D7 (__INTTAB_START_ADDR + 0x1AE0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_215)); } > PMU_INTTAB_TC0
  .inttab_tc0_0D8 (__INTTAB_START_ADDR + 0x1B00) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_216)); } > PMU_INTTAB_TC0
  .inttab_tc0_0D9 (__INTTAB_START_ADDR + 0x1B20) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_217)); } > PMU_INTTAB_TC0
  .inttab_tc0_0DA (__INTTAB_START_ADDR + 0x1B40) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_218)); } > PMU_INTTAB_TC0
  .inttab_tc0_0DB (__INTTAB_START_ADDR + 0x1B60) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_219)); } > PMU_INTTAB_TC0
  .inttab_tc0_0DC (__INTTAB_START_ADDR + 0x1B80) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_220)); } > PMU_INTTAB_TC0
  .inttab_tc0_0DD (__INTTAB_START_ADDR + 0x1BA0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_221)); } > PMU_INTTAB_TC0
  .inttab_tc0_0DE (__INTTAB_START_ADDR + 0x1BC0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_222)); } > PMU_INTTAB_TC0
  .inttab_tc0_0DF (__INTTAB_START_ADDR + 0x1BE0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_223)); } > PMU_INTTAB_TC0
  .inttab_tc0_0E0 (__INTTAB_START_ADDR + 0x1C00) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_224)); } > PMU_INTTAB_TC0
  .inttab_tc0_0E1 (__INTTAB_START_ADDR + 0x1C20) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_225)); } > PMU_INTTAB_TC0
  .inttab_tc0_0E2 (__INTTAB_START_ADDR + 0x1C40) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_226)); } > PMU_INTTAB_TC0
  .inttab_tc0_0E3 (__INTTAB_START_ADDR + 0x1C60) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_227)); } > PMU_INTTAB_TC0
  .inttab_tc0_0E4 (__INTTAB_START_ADDR + 0x1C80) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_228)); } > PMU_INTTAB_TC0
  .inttab_tc0_0E5 (__INTTAB_START_ADDR + 0x1CA0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_229)); } > PMU_INTTAB_TC0
  .inttab_tc0_0E6 (__INTTAB_START_ADDR + 0x1CC0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_230)); } > PMU_INTTAB_TC0
  .inttab_tc0_0E7 (__INTTAB_START_ADDR + 0x1CE0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_231)); } > PMU_INTTAB_TC0
  .inttab_tc0_0E8 (__INTTAB_START_ADDR + 0x1D00) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_232)); } > PMU_INTTAB_TC0
  .inttab_tc0_0E9 (__INTTAB_START_ADDR + 0x1D20) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_233)); } > PMU_INTTAB_TC0
  .inttab_tc0_0EA (__INTTAB_START_ADDR + 0x1D40) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_234)); } > PMU_INTTAB_TC0
  .inttab_tc0_0EB (__INTTAB_START_ADDR + 0x1D60) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_235)); } > PMU_INTTAB_TC0
  .inttab_tc0_0EC (__INTTAB_START_ADDR + 0x1D80) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_236)); } > PMU_INTTAB_TC0
  .inttab_tc0_0ED (__INTTAB_START_ADDR + 0x1DA0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_237)); } > PMU_INTTAB_TC0
  .inttab_tc0_0EE (__INTTAB_START_ADDR + 0x1DC0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_238)); } > PMU_INTTAB_TC0
  .inttab_tc0_0EF (__INTTAB_START_ADDR + 0x1DE0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_239)); } > PMU_INTTAB_TC0
  .inttab_tc0_0F0 (__INTTAB_START_ADDR + 0x1E00) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_240)); } > PMU_INTTAB_TC0
  .inttab_tc0_0F1 (__INTTAB_START_ADDR + 0x1E20) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_241)); } > PMU_INTTAB_TC0
  .inttab_tc0_0F2 (__INTTAB_START_ADDR + 0x1E40) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_242)); } > PMU_INTTAB_TC0
  .inttab_tc0_0F3 (__INTTAB_START_ADDR + 0x1E60) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_243)); } > PMU_INTTAB_TC0
  .inttab_tc0_0F4 (__INTTAB_START_ADDR + 0x1E80) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_244)); } > PMU_INTTAB_TC0
  .inttab_tc0_0F5 (__INTTAB_START_ADDR + 0x1EA0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_245)); } > PMU_INTTAB_TC0
  .inttab_tc0_0F6 (__INTTAB_START_ADDR + 0x1EC0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_246)); } > PMU_INTTAB_TC0
  .inttab_tc0_0F7 (__INTTAB_START_ADDR + 0x1EE0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_247)); } > PMU_INTTAB_TC0
  .inttab_tc0_0F8 (__INTTAB_START_ADDR + 0x1F00) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_248)); } > PMU_INTTAB_TC0
  .inttab_tc0_0F9 (__INTTAB_START_ADDR + 0x1F20) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_249)); } > PMU_INTTAB_TC0
  .inttab_tc0_0FA (__INTTAB_START_ADDR + 0x1F40) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_250)); } > PMU_INTTAB_TC0
  .inttab_tc0_0FB (__INTTAB_START_ADDR + 0x1F60) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_251)); } > PMU_INTTAB_TC0
  .inttab_tc0_0FC (__INTTAB_START_ADDR + 0x1F80) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_252)); } > PMU_INTTAB_TC0
  .inttab_tc0_0FD (__INTTAB_START_ADDR + 0x1FA0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_253)); } > PMU_INTTAB_TC0
  .inttab_tc0_0FE (__INTTAB_START_ADDR + 0x1FC0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_254)); } > PMU_INTTAB_TC0
  .inttab_tc0_0FF (__INTTAB_START_ADDR + 0x1FE0) : { . = ALIGN(8) ;  KEEP (*(.intvec_tc0_255)); } > PMU_INTTAB_TC0
  __INTTAB_CPU0 = __INTTAB_START_ADDR;
  __INTTAB_CPU1 = __INTTAB_START_ADDR; /*Single interrupt table for all CPUs*/
  __INTTAB_CPU2 = __INTTAB_START_ADDR; /*Single interrupt table for all CPUs*/



  
  .CONFIGURATION_PB 0xa0140000   :
  {
    KEEP(*(.rodata.CONFIGURATION_PB))          
  }> CONFIG_DIO_DATA

  .PwmNotifFunc 0x8002e000 :
  {
    *Pwm_17_Gtm_Demo.o(.text.Pwm_NotifCh00 )
  } > PMU_USER_FUNCS

  .SpiCmdSeqFunc 0x8002e050 :
  {
    *Spi_Demo.o(.text.EepTest_EndOfCmdSeq )
  } > PMU_USER_FUNCS

  .SpiStatSeqFunc 0x8002e060 :
  {
    *Spi_Demo.o(.text.EepTest_EndOfStatSeq )
  } > PMU_USER_FUNCS


  .SpiDataSeqFunc 0x8002e070 :
  {
    *Spi_Demo.o(.text.EepTest_EndOfDataSeq )
  } > PMU_USER_FUNCS

  .GptNotifFunc 0x8002e080 :
  {
    *Gpt_Demo.o(.text.Gpt_Notification )
  } > PMU_USER_FUNCS

  .GptMcNotifFunc 0x8002e0e0 :
  {
    *Gpt_Demo.o(.text.Gpt_MulticoreNotif )
  } > PMU_USER_FUNCS

  .GptWdgNotifFunc  0x8002e110:
  {
    *Gpt_Demo.o(.text.Gpt_WdgNotification )
  } > PMU_USER_FUNCS
  
  .PWM_NOTIFICATION 0x8002e200 :
  {
    PROVIDE(__PWM_NOTIFICATION_start = 0x8002e200);
    KEEP(*(.text.PWM_NOTIFICATION))
    PROVIDE(__PWM_NOTIFICATION_end = 0x8002e2ff);
  } > PMU_USER_FUNCS
    
  .ICU_NOTIFICATION  0x8002e300:
  {
    PROVIDE(__ICU_NOTIFICATION_start = 0x8002e300);
    KEEP(*(.text.ICU_NOTIFICATION))
    PROVIDE(__ICU_NOTIFICATION_end = 0x8002e4ff);
  } > PMU_USER_FUNCS

  .GPT_NOTIFICATION  0x8002e500:
  {
    PROVIDE(__GPT_NOTIFICATION_start = 0x8002e500);
    KEEP(*(.text.GPT_NOTIFICATION))
    PROVIDE(__GPT_NOTIFICATION_end = 0x8002e6ff);
  } > PMU_USER_FUNCS

  
  .WDG_NOTIFICATION  0x8002e700:
  {
    PROVIDE(__WDG_NOTIFICATION_start = 0x8002e700);
    KEEP(*(.text.WDG_NOTIFICATION))
    PROVIDE(__WDG_NOTIFICATION_end = 0x8002e8ff);
  } > PMU_USER_FUNCS
  
  .FEE_JOBENDNOTIF_CODE  0x8002e900:
  {
    PROVIDE(__FEE_JOBENDNOTIF_CODE_start = 0x8002e900);
    KEEP(*(.text.FEE_JOBENDNOTIF_CODE))
    PROVIDE(__FEE_JOBENDNOTIF_CODE_end = 0x8002eAff);
  } > PMU_USER_FUNCS

  .FEE_JOBERRNOTIF_CODE  0x8002eB00:
  {
    PROVIDE(__FEE_JOBERRNOTIF_CODE_start = 0x8002eB00);
    KEEP(*(.text.FEE_JOBERRNOTIF_CODE))
    PROVIDE(__FEE_JOBERRNOTIF_CODE_end = 0x8002eCff);
  } > PMU_USER_FUNCS

  .FEE_ILLEGALNOTIF_CODE  0x8002eD00:
  {
    PROVIDE(__FEE_ILLEGALNOTIF_CODE_start = 0x8002eD00);
    KEEP(*(.text.FEE_ILLEGALNOTIF_CODE))
    PROVIDE(__FEE_ILLEGALNOTIF_CODE_end = 0x8002eDff);
  } > PMU_USER_FUNCS

  .FLS_JOBENDNOTIF_CODE  0x8002eE00:
  {
    KEEP(*(.text.DemoNotifJobEnd))
  } > PMU_USER_FUNCS

  .FLS_JOBERRNOTIF_CODE  0x8002f000:
  {
    KEEP(*(.text.DemoNotifJobError))
  } > PMU_USER_FUNCS

  .SENT_CALLOUT_CODE_0 0x8002f200:
  {
    PROVIDE(__SENT_CALLOUT_CODE_0_start = 0x8002f200);
    *Sent_Callout.o(.text.Sent_Callout_Chan0);
    PROVIDE(__SENT_CALLOUT_CODE_0_end = 0x8002f2ff);
  } > PMU_USER_FUNCS

  .SENT_CALLOUT_CODE_5 0x8002f300:
  {
    PROVIDE(__SENT_CALLOUT_CODE_5_start = 0x8002f300);
    *Sent_Callout.o(.text.Sent_Callout_Chan5);
    PROVIDE(__SENT_CALLOUT_CODE_5_end = 0x8002f3ff);
  } > PMU_USER_FUNCS

  .FLS_AC_ERASE_SOURCE :
  {
    PROVIDE(FLS_AC_ERASE_SOURCE_START = .);
    KEEP(*(.text.FLS_AC_ERASE_SOURCE))          
    PROVIDE(FLS_AC_ERASE_SOURCE_END = .);
  }> FLS_ERASE_PFLASH0
  
  .FLS_AC_WRITE_SOURCE :
  {
    PROVIDE(FLS_AC_WRITE_SOURCE_START = .);
    KEEP(*(.text.FLS_AC_WRITE_SOURCE))          
    PROVIDE(FLS_AC_WRITE_SOURCE_END = .);
  }> FLS_WRITE_PFLASH0

  .FLS_AC_ERASE :
  {
    IROM_BASE_ERASE = .;
    *(.FLS_AC_ERASE)
    *(.FLS_AC_ERASE.*)
    . = ALIGN(8);
  } > FLS_ERASE_SRAM AT > FLS_ERASE_PFLASH0 = 0

  .FLS_AC_WRITE :
  {
    IROM_BASE_WRITE = .;
    *(.FLS_AC_WRITE)
    *(.FLS_AC_WRITE.*)
    . = ALIGN(8);
  } > FLS_WRITE_SRAM AT > FLS_WRITE_PFLASH0 = 0

  
  .CPU0_PRIVATE_CODE :
  {
    PROVIDE(__CPU0_PRIVATE_CODE_start = .);
    KEEP(*(.text.CPU0.Private*))
    PROVIDE(__CPU0_PRIVATE_CODE_end = .);
  } > PMU_PFLASH0

  .CPU1_PRIVATE_CODE :
  {
    PROVIDE(__CPU1_PRIVATE_CODE_start = .);
    KEEP(*(.text.CPU1.Private*))
    PROVIDE(__CPU1_PRIVATE_CODE_end = .);
  } > PMU_PFLASH0   

  .CPU2_PRIVATE_CODE :
  {
    PROVIDE(__CPU2_PRIVATE_CODE_start = .);
    KEEP(*(.text.CPU2.Private*))
    PROVIDE(__CPU2_PRIVATE_CODE_end = .);
  } > PMU_PFLASH0   

  .SHARED_CODE : FLAGS(axl)
  {
    PROVIDE(__SHARED_CODE_start = .);
    KEEP(*(.text.Shared*))
    *(.text)
    *(.text*)
    *(.gnu.linkonce.t.*)  
    KEEP(*(.init)) /*Code executed before calling main*/
    KEEP(*(.init*))     
    KEEP(*(.fini)) /*Code executed before exiting program*/
    KEEP(*(.fini*)) 
    *(.eh_frame) /*Exception handling frame for C++ exceptions*/
    *(.eh_frame*)
    KEEP(*(.ctors)) /*Section for constructors*/
    KEEP(*(.ctors*))
    KEEP(*(.dtors)) /*Section for destructors*/
    KEEP(*(.dtors*))        
    . = ALIGN(8);

    PROVIDE(__SHARED_CODE_end = .);
  } > PMU_PFLASH0



  .RODATA   :
  {
    PROVIDE(__RODATA_start = .);
    *(.rodata)
    *(.rodata*)
    *(.rodata.*)  
    *(.rodata.Shared.DEFAULT_CONST_FAR_UNSPECIFIED) 
    *(.gnu.linkonce.r.*)
    *(.jcr.*)
      
    PROVIDE(__RODATA_end = .);
  } > PMU_PFLASH0_CONST



  /* Data allocation */
  /* Initialized data section */
  .CPU0_PRIVATE_DATA  :
  {
    PROVIDE(__CPU0_PRIVATE_DATA_start = .);
    KEEP(*(.data.CPU0.Private*))
    PROVIDE(__CPU0_PRIVATE_DATA_end = .);
  } > CPU0_DMI_DSPR AT > PMU_PFLASH0

  
  .CPU1_PRIVATE_DATA  :
  {
    PROVIDE(__CPU1_PRIVATE_DATA_start = .);
    KEEP(*(.data.CPU1.Private*))
    PROVIDE(__CPU1_PRIVATE_DATA_end = .);
  } > CPU1_DMI_DSPR AT > PMU_PFLASH0


  .CPU2_PRIVATE_DATA  :
  {
    PROVIDE(__CPU2_PRIVATE_DATA_start = .);
    KEEP(*(.data.CPU2.Private*))
    PROVIDE(__CPU2_PRIVATE_DATA_end = .);
  } > CPU2_DMI_DSPR AT > PMU_PFLASH0


  .SHARED_DATA  :
  {
    PROVIDE(__SHARED_DATA_start = .);
    KEEP(*(.data.Shared*))
    *(.data)
    *(.data*)
    *(.gnu.linkonce.d.*)      
    PROVIDE(__SHARED_DATA_end = .);
  } > LMU_SRAM AT > PMU_PFLASH0


  /* Initialized sdata section */
  .CPU0_PRIVATE_SDATA  :
  {
    PROVIDE(__CPU0_SMALL_DATA = . + 0x8000);
        KEEP(*(.sdata.CPU0.Private*))
        KEEP(*(.sdata.Shared*))
    *(.sdata)
    *(.sdata*)
    *(.gnu.linkonce.s.*)      

    PROVIDE(__CPU0_SMALL_DATA_end = .);
  } > CPU0_DMI_DSPR AT > PMU_PFLASH0

  
  .CPU1_PRIVATE_SDATA  :
  {
    PROVIDE(__CPU1_SMALL_DATA = . + 0x8000);
    KEEP(*(.sdata.CPU1.Private*))
    PROVIDE(__CPU1_SMALL_DATA_end = .);
  } > CPU1_DMI_DSPR AT > PMU_PFLASH0


  .CPU2_PRIVATE_SDATA  :
  {
    PROVIDE(__CPU2_SMALL_DATA = . + 0x8000);
    KEEP(*(.sdata.CPU2.Private*))
    PROVIDE(__CPU2_SMALL_DATA_end = .);
  } > CPU2_DMI_DSPR AT > PMU_PFLASH0



  /* Initialized sdata section */
  .CPU0_PRIVATE_SDATA2  :
  {
    PROVIDE(__CPU0_SMALL_DATA2 = . + 0x8000);
        KEEP(*(.sdata.rodata.CPU0.Private*))
        KEEP(*(.sdata.rodata.Shared*))
    *(.sdata.rodata)
    *(.sdata.rodata*)
    *(.gnu.linkonce.sr.*)   

    PROVIDE(__CPU0_SMALL_DATA2_end = .);
  } > CPU0_DMI_DSPR AT > PMU_PFLASH0

  
  .CPU1_PRIVATE_SDATA2  :
  {
    PROVIDE(__CPU1_SMALL_DATA2 = . + 0x8000);
    KEEP(*(.sdata.rodata.CPU1.Private*))
    PROVIDE(__CPU1_SMALL_DATA2_end = .);
  } > CPU1_DMI_DSPR AT > PMU_PFLASH0


  .CPU2_PRIVATE_SDATA2  :
  {
    PROVIDE(__CPU2_SMALL_DATA2 = . + 0x8000);
    KEEP(*(.sdata.rodata.CPU2.Private*))
    PROVIDE(__CPU2_SMALL_DATA2_end = .);
  } > CPU2_DMI_DSPR AT > PMU_PFLASH0




  /* Initialized zdata section */
  .CPU0_PRIVATE_ZDATA  :
  {
    PROVIDE(__CPU0_PRIVATE_ZDATA_start = .);
    KEEP(*(.zdata.CPU0.Private*))
    PROVIDE(__CPU0_PRIVATE_ZDATA_end = .);
  } > CPU0_DMI_DSPR AT > PMU_PFLASH0

  
  .CPU1_PRIVATE_ZDATA  :
  {
    PROVIDE(__CPU1_PRIVATE_ZDATA_start = .);
    KEEP(*(.zdata.CPU1.Private*))
    PROVIDE(__CPU1_PRIVATE_ZDATA_end = .);
  } > CPU1_DMI_DSPR AT > PMU_PFLASH0


  .CPU2_PRIVATE_ZDATA  :
  {
    PROVIDE(__CPU2_PRIVATE_ZDATA_start = .);
    KEEP(*(.zdata.CPU2.Private*))
    PROVIDE(__CPU2_PRIVATE_ZDATA_end = .);
  } > CPU2_DMI_DSPR AT > PMU_PFLASH0


  .SHARED_ZDATA  :
  {
    PROVIDE(__SHARED_ZDATA_start = .);
    KEEP(*(.zdata.Shared*))
    *(.zdata)
    *(.zdata*)
    *(.gnu.linkonce.zb.*)
    PROVIDE(__SHARED_ZDATA_end = .);
  } > LMU_SRAM AT > PMU_PFLASH0



  /* Initialized bit data section */
  .CPU0_PRIVATE_BDATA  :
  {
    PROVIDE(__CPU0_PRIVATE_BDATA_start = .);
    KEEP(*(.bdata.CPU0.Private*))
    PROVIDE(__CPU0_PRIVATE_BDATA_end = .);
  } > CPU0_DMI_DSPR AT > PMU_PFLASH0

  
  .CPU1_PRIVATE_BDATA  :
  {
    PROVIDE(__CPU1_PRIVATE_BDATA_start = .);
    KEEP(*(.bdata.CPU1.Private*))
    PROVIDE(__CPU1_PRIVATE_BDATA_end = .);
  } > CPU1_DMI_DSPR AT > PMU_PFLASH0


  .CPU2_PRIVATE_BDATA  :
  {
    PROVIDE(__CPU2_PRIVATE_BDATA_start = .);
    KEEP(*(.bdata.CPU2.Private*))
    PROVIDE(__CPU2_PRIVATE_BDATA_end = .);
  } > CPU2_DMI_DSPR AT > PMU_PFLASH0


  .SHARED_BDATA  :
  {
    PROVIDE(__SHARED_BDATA_start = .);
    KEEP(*(.bdata.Shared*))
    *(.bdata)
    *(.bdata*)
    PROVIDE(__SHARED_BDATA_end = .);
  } > LMU_SRAM AT > PMU_PFLASH0



  /* BSS section */
  .CPU0_PRIVATE_BSS  :
  {
    PROVIDE(__CPU0_PRIVATE_BSS_start = .);
    KEEP(*(.bss.CPU0.Private*))
    PROVIDE(__CPU0_PRIVATE_BSS_end = .);
  } > CPU0_DMI_DSPR

  
  .CPU1_PRIVATE_BSS  :
  {
    PROVIDE(__CPU1_PRIVATE_BSS_start = .);
    KEEP(*(.bss.CPU1.Private*))
    PROVIDE(__CPU1_PRIVATE_BSS_end = .);
  } > CPU1_DMI_DSPR


  .CPU2_PRIVATE_BSS  :
  {
    PROVIDE(__CPU2_PRIVATE_BSS_start = .);
    KEEP(*(.bss.CPU2.Private*))
    PROVIDE(__CPU2_PRIVATE_BSS_end = .);
  } > CPU2_DMI_DSPR


  .SHARED_BSS  :
  {
    PROVIDE(__SHARED_BSS_start = .);
    KEEP(*(.bss.Shared*))
    *(.bss)
    *(.bss*)
    *(.gnu.linkonce.b.*)
    PROVIDE(__SHARED_BSS_end = .);
  } > LMU_SRAM


  /* SBSS section */
  .CPU0_PRIVATE_SBSS  :
  {
    PROVIDE(__CPU0_PRIVATE_SBSS_start = .);
    KEEP(*(.sbss.CPU0.Private*))
    PROVIDE(__CPU0_PRIVATE_SBSS_end = .);
  } > CPU0_DMI_DSPR

  
  .CPU1_PRIVATE_SBSS  :
  {
    PROVIDE(__CPU1_PRIVATE_SBSS_start = .);
    KEEP(*(.sbss.CPU1.Private*))
    PROVIDE(__CPU1_PRIVATE_SBSS_end = .);
  } > CPU1_DMI_DSPR


  .CPU2_PRIVATE_SBSS  :
  {
    PROVIDE(__CPU2_PRIVATE_SBSS_start = .);
    KEEP(*(.sbss.CPU2.Private*))
    PROVIDE(__CPU2_PRIVATE_SBSS_end = .);
  } > CPU2_DMI_DSPR


  .SHARED_SBSS  :
  {
    PROVIDE(__SHARED_SBSS_start = .);
    KEEP(*(.sbss.Shared*))
    *(.bss)
    *(.bss*)
    *(.gnu.linkonce.b.*)
    PROVIDE(__SHARED_SBSS_end = .);
  } > LMU_SRAM


  /* ZBSS section */
  .CPU0_PRIVATE_ZBSS  :
  {
    PROVIDE(__CPU0_PRIVATE_ZBSS_start = .);
    KEEP(*(.zbss.CPU0.Private*))
    PROVIDE(__CPU0_PRIVATE_ZBSS_end = .);
  } > CPU0_DMI_DSPR

  
  .CPU1_PRIVATE_ZBSS  :
  {
    PROVIDE(__CPU1_PRIVATE_ZBSS_start = .);
    KEEP(*(.zbss.CPU1.Private*))
    PROVIDE(__CPU1_PRIVATE_ZBSS_end = .);
  } > CPU1_DMI_DSPR


  .CPU2_PRIVATE_ZBSS  :
  {
    PROVIDE(__CPU2_PRIVATE_ZBSS_start = .);
    KEEP(*(.zbss.CPU2.Private*))
    PROVIDE(__CPU2_PRIVATE_ZBSS_end = .);
  } > CPU2_DMI_DSPR


  .SHARED_ZBSS  :
  {
    PROVIDE(__SHARED_ZBSS_start = .);
    KEEP(*(.zbss.Shared*))
    *(.zbss) 
    *(.zbss*)
    *(.gnu.linkonce.zb.*)
    PROVIDE(__SHARED_ZBSS_end = .);
  } > LMU_SRAM



  /* BBSS section */
  .CPU0_PRIVATE_BBSS  :
  {
    PROVIDE(__CPU0_PRIVATE_BBSS_start = .);
    KEEP(*(.bbss.CPU0.Private*))
    PROVIDE(__CPU0_PRIVATE_BBSS_end = .);
  } > CPU0_DMI_DSPR

  
  .CPU1_PRIVATE_BBSS  :
  {
    PROVIDE(__CPU1_PRIVATE_BBSS_start = .);
    KEEP(*(.bbss.CPU1.Private*))
    PROVIDE(__CPU1_PRIVATE_BBSS_end = .);
  } > CPU1_DMI_DSPR


  .CPU2_PRIVATE_BBSS  :
  {
    PROVIDE(__CPU2_PRIVATE_BBSS_start = .);
    KEEP(*(.bbss.CPU2.Private*))
    PROVIDE(__CPU2_PRIVATE_BBSS_end = .);
  } > CPU2_DMI_DSPR


  .SHARED_BBSS  :
  {
    PROVIDE(__SHARED_BBSS_start = .);
    KEEP(*(.bbss.Shared*))
    *(.bbss) 
    *(.bbss*)
    PROVIDE(__SHARED_BBSS_end = .);
  } > LMU_SRAM



  .clear_sec :
  {
    . = ALIGN(8);
    PROVIDE(__shared_clear_table = .) ;

    LONG(0 + ADDR(.SHARED_BSS));  LONG(SIZEOF(.SHARED_BSS));
    LONG(0 + ADDR(.SHARED_SBSS)); LONG(SIZEOF(.SHARED_SBSS));
    LONG(0 + ADDR(.SHARED_ZBSS)); LONG(SIZEOF(.SHARED_ZBSS));
    LONG(0 + ADDR(.SHARED_BBSS)); LONG(SIZEOF(.SHARED_BBSS));

    PROVIDE(__cpu0_clear_table = .) ;

    LONG(0 + ADDR(.CPU0_PRIVATE_BSS)); LONG(SIZEOF(.CPU0_PRIVATE_BSS));
    LONG(0 + ADDR(.CPU0_PRIVATE_SBSS)); LONG(SIZEOF(.CPU0_PRIVATE_SBSS));
    LONG(0 + ADDR(.CPU0_PRIVATE_ZBSS)); LONG(SIZEOF(.CPU0_PRIVATE_ZBSS));
    LONG(0 + ADDR(.CPU0_PRIVATE_BBSS)); LONG(SIZEOF(.CPU0_PRIVATE_BBSS));

    PROVIDE(__cpu1_clear_table = .) ;

    LONG(0 + ADDR(.CPU1_PRIVATE_BSS)); LONG(SIZEOF(.CPU1_PRIVATE_BSS));
    LONG(0 + ADDR(.CPU1_PRIVATE_SBSS)); LONG(SIZEOF(.CPU1_PRIVATE_SBSS));
    LONG(0 + ADDR(.CPU1_PRIVATE_ZBSS)); LONG(SIZEOF(.CPU1_PRIVATE_ZBSS));
    LONG(0 + ADDR(.CPU1_PRIVATE_BBSS)); LONG(SIZEOF(.CPU1_PRIVATE_BBSS));

    PROVIDE(__cpu2_clear_table = .) ;

    LONG(0 + ADDR(.CPU2_PRIVATE_BSS)); LONG(SIZEOF(.CPU2_PRIVATE_BSS));
    LONG(0 + ADDR(.CPU2_PRIVATE_SBSS)); LONG(SIZEOF(.CPU2_PRIVATE_SBSS));
    LONG(0 + ADDR(.CPU2_PRIVATE_ZBSS)); LONG(SIZEOF(.CPU2_PRIVATE_ZBSS));
    LONG(0 + ADDR(.CPU2_PRIVATE_BBSS)); LONG(SIZEOF(.CPU2_PRIVATE_BBSS));


    LONG(-1);                 LONG(-1);

  } > PMU_PFLASH0




  .copy_sec :
  {
    . = ALIGN(8);
    PROVIDE(__shared_copy_table = .) ;

    LONG(LOADADDR(.SHARED_DATA)); LONG(0 + ADDR(.SHARED_DATA)); LONG(SIZEOF(.SHARED_DATA));
    LONG(LOADADDR(.SHARED_ZDATA));  LONG(0 + ADDR(.SHARED_ZDATA));  LONG(SIZEOF(.SHARED_ZDATA));
    LONG(LOADADDR(.SHARED_BDATA));  LONG(0 + ADDR(.SHARED_BDATA));  LONG(SIZEOF(.SHARED_BDATA));


    PROVIDE(__cpu0_copy_table = .) ;

    LONG(LOADADDR(.CPU0_PRIVATE_DATA)); LONG(0 + ADDR(.CPU0_PRIVATE_DATA)); LONG(SIZEOF(.CPU0_PRIVATE_DATA));
    LONG(LOADADDR(.CPU0_PRIVATE_SDATA));  LONG(0 + ADDR(.CPU0_PRIVATE_SDATA));  LONG(SIZEOF(.CPU0_PRIVATE_SDATA));
    LONG(LOADADDR(.CPU0_PRIVATE_ZDATA));  LONG(0 + ADDR(.CPU0_PRIVATE_ZDATA));  LONG(SIZEOF(.CPU0_PRIVATE_ZDATA));
    LONG(LOADADDR(.CPU0_PRIVATE_BDATA));  LONG(0 + ADDR(.CPU0_PRIVATE_BDATA));  LONG(SIZEOF(.CPU0_PRIVATE_BDATA));
    LONG(LOADADDR(.FLSLOADERRAMCODE));      LONG(0 + ADDR(.FLSLOADERRAMCODE)); LONG(SIZEOF(.FLSLOADERRAMCODE));

    PROVIDE(__cpu1_copy_table = .) ;

    LONG(LOADADDR(.CPU1_PRIVATE_DATA)); LONG(0 + ADDR(.CPU1_PRIVATE_DATA)); LONG(SIZEOF(.CPU1_PRIVATE_DATA));
    LONG(LOADADDR(.CPU1_PRIVATE_SDATA));  LONG(0 + ADDR(.CPU1_PRIVATE_SDATA));  LONG(SIZEOF(.CPU1_PRIVATE_SDATA));
    LONG(LOADADDR(.CPU1_PRIVATE_ZDATA));  LONG(0 + ADDR(.CPU1_PRIVATE_ZDATA));  LONG(SIZEOF(.CPU1_PRIVATE_ZDATA));
    LONG(LOADADDR(.CPU1_PRIVATE_BDATA));  LONG(0 + ADDR(.CPU1_PRIVATE_BDATA));  LONG(SIZEOF(.CPU1_PRIVATE_BDATA));

    PROVIDE(__cpu2_copy_table = .) ;

    LONG(LOADADDR(.CPU2_PRIVATE_DATA)); LONG(0 + ADDR(.CPU2_PRIVATE_DATA)); LONG(SIZEOF(.CPU2_PRIVATE_DATA));
    LONG(LOADADDR(.CPU2_PRIVATE_SDATA));  LONG(0 + ADDR(.CPU2_PRIVATE_SDATA));  LONG(SIZEOF(.CPU2_PRIVATE_SDATA));
    LONG(LOADADDR(.CPU2_PRIVATE_ZDATA));  LONG(0 + ADDR(.CPU2_PRIVATE_ZDATA));  LONG(SIZEOF(.CPU2_PRIVATE_ZDATA));
    LONG(LOADADDR(.CPU2_PRIVATE_BDATA));  LONG(0 + ADDR(.CPU2_PRIVATE_BDATA));  LONG(SIZEOF(.CPU2_PRIVATE_BDATA));


    /*PROTECTED REGION ID(Protection: iROM copy section) ENABLED START*/
      /*Protection-Area for your own LDF-Code*/
    /*PROTECTED REGION END*/
    LONG(-1);                 LONG(-1);                  LONG(-1);
  } > PMU_PFLASH0

  /* Flsloader code executed from RAM */
  .FLSLOADERRAMCODE :
  {
         _code_ram_start = ABSOLUTE (.);
         *(.FLSLOADERRAMCODE*)
         . = ALIGN(4);
         _code_ram_end = ABSOLUTE (.);
         _code_rom_start = LOADADDR (.FLSLOADERRAMCODE);
  } > FLSLOADER_RAM_CODE AT> PMU_PFLASH0


  .cpu0_csa :
  {
     . = ALIGN(64);
     __CPU0_CSA_BEGIN = . ;
     . += __CSA_SIZE;
     __CPU0_CSA_END = .;
  } > CPU0_DMI_DSPR

  .heap :
  {
    . =  ALIGN(8);
    __HEAP = .;
    . += __HEAP_SIZE;
    . =  ALIGN(8);
    __HEAP_END = .;   
  } > DMI_DSPR

  .cpu0_istack :
  {
    . =  ALIGN(8);
    . += __ISTACK_SIZE;
    __CPU0_ISTACK = .;
  } > CPU0_DMI_DSPR

  .cpu0_ustack :
  {
    . =  ALIGN(8);
    . += __USTACK_SIZE;
    __CPU0_USTACK = .;
  } > CPU0_DMI_DSPR


  .cpu1_csa :
  {
     . = ALIGN(64);
     __CPU1_CSA_BEGIN = . ;
     . += __CSA_SIZE;
     . = ALIGN(64);    
     __CPU1_CSA_END = .;
  } > CPU1_DMI_DSPR


  .cpu1_istack :
  {
    . =  ALIGN(8);
    . += __ISTACK_SIZE;
    __CPU1_ISTACK = .;
  } > CPU1_DMI_DSPR

  .cpu1_ustack :
  {
    . =  ALIGN(8);
    . += __USTACK_SIZE;
    __CPU1_USTACK = .;
  } > CPU1_DMI_DSPR


  .cpu2_csa :
  {
     . = ALIGN(64);
     __CPU2_CSA_BEGIN = . ;
     . += __CSA_SIZE;
     . = ALIGN(64);    
     __CPU2_CSA_END = .;
  } > CPU2_DMI_DSPR


  .cpu2_istack :
  {
    . =  ALIGN(8);
    . += __ISTACK_SIZE;
    __CPU2_ISTACK = .;
  } > CPU2_DMI_DSPR

  .cpu2_ustack :
  {
    . =  ALIGN(8);
    . += __USTACK_SIZE;
    __CPU2_USTACK = .;
  } > CPU2_DMI_DSPR


}

