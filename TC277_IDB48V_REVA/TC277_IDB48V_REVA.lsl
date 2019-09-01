
// Include the CPU specific .lsl file
// The CPU is specified by the __CPU__ macro
#ifndef __CPU__
# error No CPU defined, please link with -D__CPU__=<cpu>
#endif

#define ISTACK_TC0 8k
#define ISTACK_TC1 8k
#define ISTACK_TC2 8k
#define USTACK_TC0 16k
#define USTACK_TC1 8k
#define USTACK_TC2 8k

#define PMU_PFLASH0 0x80030000
#define PMU_PFLASH1 0x80200000
#define CPU0_PMI_PSPR 0x70100000
#define CPU1_PMI_PSPR 0x60100000
#define CPU2_PMI_PSPR 0x50100000
// 100 bytes of DSPR is reserved for sbst_tc0_dspr
#define CPU0_DMI_DSPR 0x70000100
#define CPU1_DMI_DSPR 0x60000100
#define CPU2_DMI_DSPR 0x50000100
#define LMU_SRAM 0xB0000000


#define __QUOTE_ME2(x) #x
#define __QUOTE_ME(x) __QUOTE_ME2(x)
#define CPULSL_FILE __QUOTE_ME(__CPU__.lsl)

#include CPULSL_FILE

// Define default external memory
// These memory definitions can be replaced by the Debug Target Configuration Wizard

#ifndef HAS_ON_CHIP_FLASH       // On-chip flash is located at 0x[8a]0000000
memory xrom_a (tag="External ROM",tag="dtc")
{
        mau = 8;
        size = 1M;
        type = rom;
         map     cached (dest=bus:sri, dest_offset=0x80000000,           size=1M);
         map not_cached (dest=bus:sri, dest_offset=0xa0000000, reserved, size=1M);
}
#endif

/* trap tab is on the reset address of BTV (cached area) */
#define IFXTRAPTAB 0x80000100 /* this is the default address (only cached area) of BTV after reset */

section_layout :vtc:linear
{
  "_lc_u_trap_tab" = (IFXTRAPTAB);
  "_lc_u_trap_tab_tc0" = (IFXTRAPTAB);
  "_lc_u_trap_tab_tc1" = (IFXTRAPTAB+0x0100);
  "_lc_u_trap_tab_tc2" = (IFXTRAPTAB+0x0200);

              // trapvector table for CPU0
  group (ordered, contiguous, align = 1<<5, run_addr=(IFXTRAPTAB)+0x0000)
  {
    select ".text.CPU0_TRAP_HANDLER_CODE_ROM";
  }

              // trapvector table for CPU1
  group (ordered, contiguous, align = 1<<5, run_addr=(IFXTRAPTAB)+0x0100)
  {
    select ".text.CPU1_TRAP_HANDLER_CODE_ROM";
  }

              // trapvector table for CPU2
  group (ordered, contiguous, align = 1<<5, run_addr=(IFXTRAPTAB)+0x0200)
  {
    select ".text.CPU2_TRAP_HANDLER_CODE_ROM";
  }
  
}

section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=0x80000000)
  {
    select ".rodata.BMD_HDR_CONST_FAR_UNSPECIFIED";
  }
  
  group  (ordered, run_addr=0x80000400)
  {
    select ".rodata*";
  }
}


section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=0x8002e000)
  {
    select ".text.Pwm_17_Gtm_Demo.Pwm_NotifCh00";
  }
}

section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=0x8002e050)
  {
    select ".text.Spi_Demo.EepTest_EndOfCmdSeq";
  }
}
section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=0x8002e060)
  {
    select ".text.Spi_Demo.EepTest_EndOfStatSeq";
  }
}
section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=0x8002e070)
  {
    select ".text.Spi_Demo.EepTest_EndOfDataSeq";
  }
}
section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=0x8002e080)
  {
    select ".text.Gpt_Demo.Gpt_Notification";
  }
}

section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=0x8002e0e0)
  {
    select ".text.Gpt_Demo.Gpt_MulticoreNotif";
  }
}

section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=0x8002e110)
  {
    select ".text.Gpt_Demo.Gpt_WdgNotification";
  }
}

section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=0x8002e200)
  {
    select ".text.PWM_NOTIFICATION";
  }
}

section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=0x8002e300)
  {
    select ".text.ICU_NOTIFICATION";
  }
}


section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=0x8002e500)
  {
    select ".text.GPT_NOTIFICATION";
  }
}

section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=0x8002e700)
  {
    select ".text.WDG_NOTIFICATION";
  }
}

section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=0x8002e900)
  {
    select ".text.FEE_JOBENDNOTIF_CODE";
  }
}

section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=0x8002eB00)
  {
    select ".text.FEE_JOBERRNOTIF_CODE";
  }
}

section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=0x8002eD00)
  {
    select ".text.FEE_ILLEGALNOTIF_CODE";
  }
}

section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=0x8002eE00)
  {
    select ".text.Fls_17_Pmu_Demo.DemoNotifJobEnd";
  }
}

section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=0x8002f000)
  {
    select ".text.Fls_17_Pmu_Demo.DemoNotifJobError";
  }
}

section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=0x8002f200)
  {
    select ".text.Sent_Callout.Sent_Callout_Chan0";
  }
  group  (ordered, run_addr=0x8002f300)
  {
    select ".text.Sent_Callout.Sent_Callout_Chan5";
  }
}


section_layout :tc0:linear
{
  group FLS_AC_WRITE (ordered, run_addr = 0x70101100)
  {
    reserved "FLS_AC_WRITE" (alloc_allowed = absolute, size = 200);
  }
}
section_layout :tc0:linear
{
  group FLS_AC_ERASE (ordered, run_addr = 0x701011C8)
  {
    reserved "FLS_AC_ERASE" (alloc_allowed = absolute, size = 200);
  }
}

section_layout :tc0:linear
{
  group FLS_STATE_VAR (ordered, run_addr = 0x70010000)
  {
    reserved "FLS_STATE_VAR" (alloc_allowed = absolute, size = 48);
  }
}

section_layout mpe:vtc:linear
{
  group FLS_AC_ERASE_SOURCE (ordered, contiguous, run_addr=0x80008100)
  {
    select ".text.FLS_AC_ERASE_SOURCE";
  }
}

section_layout mpe:vtc:linear
{
  group FLS_AC_WRITE_SOURCE (ordered, contiguous, run_addr=0x80008300)
  {
    select ".text.FLS_AC_WRITE_SOURCE";
  }
}


section_setup mpe:vtc:linear
{
  modify input (space = mpe:tc0:linear, copy)
  {
    select "(.text.FLSLOADERRAMCODE*)";
  }
}

section_layout mpe:tc0:linear
{
  group FLSLOADER_CODE ( ordered, run_addr = mem:mpe:dspr0 )
  {
    select "(.text.FLSLOADERRAMCODE*)";
  }
}

section_layout :tc0:abs18
{
  group NEAR_SECTION_16K (ordered, run_addr = 0xd0001000, align = 4)
  {
    select "(.zbss.DEFAULT_RAM_FAST_32BIT)";
  }
}

section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=mem:mpe:pspr0)
  {
  }
}

section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=mem:mpe:pspr1)
  {
  }
}

section_layout mpe:vtc:linear
{
  group  (ordered, run_addr=mem:mpe:pspr2)
  {
  }
}


// CODE
section_layout mpe:vtc:linear
{
  select ".text.libc.reset";
  group SHARED_CODE (ordered, run_addr=PMU_PFLASH0)
  {
    select "(.text.Shared*)";
    select "(.text*)";
  } 
}
section_layout mpe:vtc:linear
{
  group CPU0_PRIVATE_CODE (ordered, run_addr=PMU_PFLASH0)
  {
    select "(.text.CPU0.Private*)";
  }
}
section_layout mpe:vtc:linear
{
  group CPU1_PRIVATE_CODE (ordered, run_addr=PMU_PFLASH0)
  {
    select "(.text.CPU1.Private*)";
  }
}

section_layout mpe:vtc:linear
{
  group CPU2_PRIVATE_CODE (ordered, run_addr=PMU_PFLASH0)
  {
    select "(.text.CPU2.Private*)";
  }
}


//CORE SPECIFIC DATA

section_layout :tc0:linear
{
  group  (ordered, align = 64, attributes=rw, run_addr=(0xd0000000)) 
          reserved "sbst_tc0_dspr" (size = 0x100);                
  group  (ordered, align = 64, attributes=rw, run_addr=(0xc0000000)) 
          reserved "sbst_tc0_pspr" (size = 0x40);                
}



section_layout :tc1:linear
{
  group  (ordered, align = 64, attributes=rw, run_addr=(0xd0000000)) 
          reserved "sbst_tc1_dspr" (size = 0x100);                
  group  (ordered, align = 64, attributes=rw, run_addr=(0xc0000000)) 
          reserved "sbst_tc1_pspr" (size = 0x40);                
}


section_layout :tc2:linear
{
  group  (ordered, align = 64, attributes=rw, run_addr=(0xd0000000)) 
          reserved "sbst_tc2_dspr" (size = 0x100);                
  group  (ordered, align = 64, attributes=rw, run_addr=(0xc0000000)) 
          reserved "sbst_tc2_pspr" (size = 0x40);                
}

section_layout :tc0:linear
{
  group  (ordered, run_addr=0xD0001080)
  {
    select ".data.FLS_RESERVED";
  }
}


section_layout mpe:vtc:linear
{
  group CPU0_PRIVATE_DATA (ordered, run_addr=mem:mpe:dspr0)
  {
    select "(.data.CPU0.Private*)";
    select "(.zdata.CPU0.Private*)";
    select "(.zdata*)";
    select "(.bss.CPU0.Private*)";
  }
}

section_layout mpe:vtc:linear
{
  group CPU1_PRIVATE_DATA (ordered, run_addr=mem:mpe:dspr1)
  {
    select "(.data.CPU1.Private*)";
    select "(.bss.CPU1.Private.*)";
  }
}

section_layout mpe:vtc:linear
{
  group CPU2_PRIVATE_DATA (ordered, run_addr=mem:mpe:dspr2)
  {
    select "(.data.CPU2.Private*)";
    select "(.bss.CPU2.Private*)";    
  }
}


section_layout mpe:vtc:abs18
{
  group SHARED_ZBSS (ordered, run_addr=mem:mpe:lmuram/not_cached)
  {
    select "(.zbss*)";
  }
}

section_layout mpe:vtc:linear
{
  select ".bss.Dma_Demo.ExtBuffer";
  group SHARED_DATA (ordered, run_addr=mem:mpe:lmuram/not_cached)
  {
    select "(.data*)";
    select "(.zdata*)";
    select "(.bss*)";
  }
}
