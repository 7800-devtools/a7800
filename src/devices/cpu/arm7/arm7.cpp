// license:BSD-3-Clause
// copyright-holders:Steve Ellenoff,R. Belmont,Ryan Holtz
/*****************************************************************************
 *
 *   arm7.c
 *   Portable CPU Emulator for 32-bit ARM v3/4/5/6
 *
 *   Copyright Steve Ellenoff, all rights reserved.
 *   Thumb, DSP, and MMU support and many bugfixes by R. Belmont and Ryan Holtz.
 *
 *  This work is based on:
 *  #1) 'Atmel Corporation ARM7TDMI (Thumb) Datasheet - January 1999'
 *  #2) Arm 2/3/6 emulator By Bryan McPhail (bmcphail@tendril.co.uk) and Phil Stroffolino (MAME CORE 0.76)
 *
 *****************************************************************************/

/******************************************************************************
 *  Notes:

    ** This is a plain vanilla implementation of an ARM7 cpu which incorporates my ARM7 core.
       It can be used as is, or used to demonstrate how to utilize the arm7 core to create a cpu
       that uses the core, since there are numerous different mcu packages that incorporate an arm7 core.

       See the notes in the arm7core.inc file itself regarding issues/limitations of the arm7 core.
    **

TODO:
- Cleanups
- Fix and finish the DRC code, or remove it entirely

*****************************************************************************/
#include "emu.h"
#include "debugger.h"
#include "arm7.h"
#include "arm7core.h"   //include arm7 core
#include "arm7help.h"


/* prototypes of coprocessor functions */
void arm7_dt_r_callback(arm_state *arm, uint32_t insn, uint32_t *prn, uint32_t (*read32)(arm_state *arm, uint32_t addr));
void arm7_dt_w_callback(arm_state *arm, uint32_t insn, uint32_t *prn, void (*write32)(arm_state *arm, uint32_t addr, uint32_t data));

// holder for the co processor Data Transfer Read & Write Callback funcs
void (*arm7_coproc_dt_r_callback)(arm_state *arm, uint32_t insn, uint32_t *prn, uint32_t (*read32)(arm_state *arm, uint32_t addr));
void (*arm7_coproc_dt_w_callback)(arm_state *arm, uint32_t insn, uint32_t *prn, void (*write32)(arm_state *arm, uint32_t addr, uint32_t data));


DEFINE_DEVICE_TYPE(ARM7,    arm7_cpu_device,    "arm7_le", "ARM7 (little)")
DEFINE_DEVICE_TYPE(ARM7_BE, arm7_be_cpu_device, "arm7_be", "ARM7 (big)")
DEFINE_DEVICE_TYPE(ARM7500, arm7500_cpu_device, "arm7500", "ARM7500")
DEFINE_DEVICE_TYPE(ARM9,    arm9_cpu_device,    "arm9",    "ARM9")
DEFINE_DEVICE_TYPE(ARM920T, arm920t_cpu_device, "arm920t", "ARM920T")
DEFINE_DEVICE_TYPE(PXA255,  pxa255_cpu_device,  "pxa255",  "Intel XScale PXA255")
DEFINE_DEVICE_TYPE(SA1110,  sa1110_cpu_device,  "sa1110",  "Intel StrongARM SA-1110")


arm7_cpu_device::arm7_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: arm7_cpu_device(mconfig, ARM7, tag, owner, clock, 4, eARM_ARCHFLAGS_T, ENDIANNESS_LITTLE)
{
}


arm7_cpu_device::arm7_cpu_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, uint8_t archRev, uint8_t archFlags, endianness_t endianness)
	: cpu_device(mconfig, type, tag, owner, clock)
	, m_program_config("program", endianness, 32, 32, 0)
	, m_endian(endianness)
	, m_archRev(archRev)
	, m_archFlags(archFlags)
	, m_copro_id(0x41 | (1 << 23) | (7 << 12))  // <-- where did this come from?
	, m_pc(0)
{
	memset(m_r, 0x00, sizeof(m_r));
}


arm7_be_cpu_device::arm7_be_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: arm7_cpu_device(mconfig, ARM7_BE, tag, owner, clock, 4, eARM_ARCHFLAGS_T, ENDIANNESS_BIG)
{
}


arm7500_cpu_device::arm7500_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: arm7_cpu_device(mconfig, ARM7500, tag, owner, clock, 3, eARM_ARCHFLAGS_MODE26, ENDIANNESS_LITTLE)
{
	m_copro_id = (0x41 << 24) | (0 << 20) | (1 << 16) | (0x710 << 4) | (0 << 0);
}


arm9_cpu_device::arm9_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: arm7_cpu_device(mconfig, ARM9, tag, owner, clock, 5, eARM_ARCHFLAGS_T | eARM_ARCHFLAGS_E, ENDIANNESS_LITTLE)
	// ARMv5
	// has TE extensions
{
}


arm920t_cpu_device::arm920t_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: arm7_cpu_device(mconfig, ARM920T, tag, owner, clock, 4, eARM_ARCHFLAGS_T, ENDIANNESS_LITTLE)
	// ARMv4
	// has T extension
{
	m_copro_id = (0x41 << 24) | (1 << 20) | (2 << 16) | (0x920 << 4) | (0 << 0);
}


pxa255_cpu_device::pxa255_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: arm7_cpu_device(mconfig, PXA255, tag, owner, clock, 5, eARM_ARCHFLAGS_T | eARM_ARCHFLAGS_E | eARM_ARCHFLAGS_XSCALE, ENDIANNESS_LITTLE)
	// ARMv5
	// has TE and XScale extensions
{
}


sa1110_cpu_device::sa1110_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: arm7_cpu_device(mconfig, SA1110, tag, owner, clock, 4, eARM_ARCHFLAGS_SA, ENDIANNESS_LITTLE)
	// ARMv4
	// has StrongARM, no Thumb, no Enhanced DSP
{
}

device_memory_interface::space_config_vector arm7_cpu_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM, &m_program_config)
	};
}

void arm7_cpu_device::update_reg_ptr()
{
	m_reg_group = sRegisterTable[GET_MODE];
}

void arm7_cpu_device::set_cpsr(uint32_t val)
{
	uint8_t old_mode = GET_CPSR & MODE_FLAG;
	if (m_archFlags & eARM_ARCHFLAGS_MODE26)
	{
		if ((val & 0x10) != (m_r[eCPSR] & 0x10))
		{
			if (val & 0x10)
			{
				// 26 -> 32
				val = (val & 0x0FFFFF3F) | (m_r[eR15] & 0xF0000000) /* N Z C V */ | ((m_r[eR15] & 0x0C000000) >> (26 - 6)) /* I F */;
				m_r[eR15] = m_r[eR15] & 0x03FFFFFC;
			}
			else
			{
				// 32 -> 26
				m_r[eR15] = (m_r[eR15] & 0x03FFFFFC) /* PC */ | (val & 0xF0000000) /* N Z C V */ | ((val & 0x000000C0) << (26 - 6)) /* I F */ | (val & 0x00000003) /* M1 M0 */;
			}
		}
		else
		{
			if (!(val & 0x10))
			{
				// mirror bits in pc
				m_r[eR15] = (m_r[eR15] & 0x03FFFFFF) | (val & 0xF0000000) /* N Z C V */ | ((val & 0x000000C0) << (26 - 6)) /* I F */;
			}
		}
	}
	else
	{
		val |= 0x10; // force valid mode
	}
	m_r[eCPSR] = val;
	if ((GET_CPSR & MODE_FLAG) != old_mode)
	{
		update_reg_ptr();
	}
}


/**************************************************************************
 * ARM TLB IMPLEMENTATION
 **************************************************************************/

enum
{
	TLB_COARSE = 0,
	TLB_FINE
};

enum
{
	FAULT_NONE = 0,
	FAULT_DOMAIN,
	FAULT_PERMISSION
};


// COARSE, desc_level1, vaddr
uint32_t arm7_cpu_device::arm7_tlb_get_second_level_descriptor( uint32_t granularity, uint32_t first_desc, uint32_t vaddr )
{
	uint32_t desc_lvl2 = vaddr;

	switch( granularity )
	{
		case TLB_COARSE:
			desc_lvl2 = (first_desc & COPRO_TLB_CFLD_ADDR_MASK) | ((vaddr & COPRO_TLB_VADDR_CSLTI_MASK) >> COPRO_TLB_VADDR_CSLTI_MASK_SHIFT);
			break;
		case TLB_FINE:
			desc_lvl2 = (first_desc & COPRO_TLB_FPTB_ADDR_MASK) | ((vaddr & COPRO_TLB_VADDR_FSLTI_MASK) >> COPRO_TLB_VADDR_FSLTI_MASK_SHIFT);
			break;
		default:
			// We shouldn't be here
			LOG( ( "ARM7: Attempting to get second-level TLB descriptor of invalid granularity (%d)\n", granularity ) );
			break;
	}

	return m_program->read_dword( desc_lvl2 );
}


int arm7_cpu_device::detect_fault(int desc_lvl1, int ap, int flags)
{
	switch (m_decoded_access_control[(desc_lvl1 >> 5) & 0xf])
	{
		case 0 : // "No access - Any access generates a domain fault"
		{
			return FAULT_DOMAIN;
		}
		case 1 : // "Client - Accesses are checked against the access permission bits in the section or page descriptor"
		{
			if ((ap & 3) == 3)
			{
				return FAULT_NONE;
			}
			else if (ap & 2)
			{
				if (((m_r[eCPSR] & MODE_FLAG) == eARM7_MODE_USER) && (flags & ARM7_TLB_WRITE))
				{
					return FAULT_PERMISSION;
				}
			}
			else if (ap & 1)
			{
				if ((m_r[eCPSR] & MODE_FLAG) == eARM7_MODE_USER)
				{
					return FAULT_PERMISSION;
				}
			}
			else
			{
				int s = (m_control & COPRO_CTRL_SYSTEM) ? 1 : 0;
				int r = (m_control & COPRO_CTRL_ROM) ? 1 : 0;
				if (s == 0)
				{
					if (r == 0) // "Any access generates a permission fault"
					{
						return FAULT_PERMISSION;
					}
					else // "Any write generates a permission fault"
					{
						if (flags & ARM7_TLB_WRITE)
						{
							return FAULT_PERMISSION;
						}
					}
				}
				else
				{
					if (r == 0) // "Only Supervisor read permitted"
					{
						if (((m_r[eCPSR] & MODE_FLAG) == eARM7_MODE_USER) || (flags & ARM7_TLB_WRITE))
						{
							return FAULT_PERMISSION;
						}
					}
					else // "Reserved" -> assume same behaviour as S=0/R=0 case
					{
						return FAULT_PERMISSION;
					}
				}
			}
		}
		break;
		case 2 : // "Reserved - Reserved. Currently behaves like the no access mode"
		{
			return FAULT_DOMAIN;
		}
		case 3 : // "Manager - Accesses are not checked against the access permission bits so a permission fault cannot be generated"
		{
			return FAULT_NONE;
		}
	}
	return FAULT_NONE;
}


bool arm7_cpu_device::arm7_tlb_translate(offs_t &addr, int flags)
{
	if (addr < 0x2000000)
	{
		addr += m_pid_offset;
	}

	uint32_t desc_lvl1 = m_program->read_dword(m_tlb_base_mask | ((addr & COPRO_TLB_VADDR_FLTI_MASK) >> COPRO_TLB_VADDR_FLTI_MASK_SHIFT));

#if ARM7_MMU_ENABLE_HACK
	if ((m_r[eR15] == (m_mmu_enable_addr + 4)) || (m_r[eR15] == (m_mmu_enable_addr + 8)))
	{
		LOG( ( "ARM7: fetch flat, PC = %08x, vaddr = %08x\n", m_r[eR15], addr ) );
		return true;
	}
	else
	{
		m_mmu_enable_addr = 1;
	}
#endif

	uint8_t tlb_type = desc_lvl1 & 3;
	if (tlb_type == COPRO_TLB_SECTION_TABLE)
	{
		// Entry is a section
		int fault = detect_fault(desc_lvl1, (desc_lvl1 >> 10) & 3, flags);
		if (fault == FAULT_NONE)
		{
			addr = ( desc_lvl1 & COPRO_TLB_SECTION_PAGE_MASK ) | ( addr & ~COPRO_TLB_SECTION_PAGE_MASK );
		}
		else
		{
			if (flags & ARM7_TLB_ABORT_D)
			{
				uint8_t domain = (desc_lvl1 >> 5) & 0xF;
				LOG( ( "ARM7: Section Table, Section %s fault on virtual address, vaddr = %08x, PC = %08x\n", (fault == FAULT_DOMAIN) ? "domain" : "permission", addr, m_r[eR15] ) );
				m_faultStatus[0] = ((fault == FAULT_DOMAIN) ? (9 << 0) : (13 << 0)) | (domain << 4); // 9 = section domain fault, 13 = section permission fault
				m_faultAddress = addr;
				m_pendingAbtD = true;
				update_irq_state();
				LOG( ( "vaddr %08X desc_lvl1 %08X domain %d permission %d ap %d s %d r %d mode %d read %d write %d\n",
					addr, desc_lvl1, domain, (m_domainAccessControl >> ((desc_lvl1 >> 4) & 0x1e)) & 3, (desc_lvl1 >> 10) & 3, (m_control & COPRO_CTRL_SYSTEM) ? 1 : 0, (m_control & COPRO_CTRL_ROM) ? 1 : 0,
					m_r[eCPSR] & MODE_FLAG, flags & ARM7_TLB_READ ? 1 : 0,  flags & ARM7_TLB_WRITE ? 1 : 0) );
			}
			else if (flags & ARM7_TLB_ABORT_P)
			{
				LOG( ( "ARM7: Section Table, Section %s fault on virtual address, vaddr = %08x, PC = %08x\n", (fault == FAULT_DOMAIN) ? "domain" : "permission", addr, m_r[eR15] ) );
				m_pendingAbtP = true;
				update_irq_state();
			}
			return false;
		}
	}
	else if (tlb_type == COPRO_TLB_UNMAPPED)
	{
		// Unmapped, generate a translation fault
		if (flags & ARM7_TLB_ABORT_D)
		{
			LOG( ( "ARM7: Translation fault on unmapped virtual address, PC = %08x, vaddr = %08x\n", m_r[eR15], addr ) );
			m_faultStatus[0] = (5 << 0); // 5 = section translation fault
			m_faultAddress = addr;
			m_pendingAbtD = true;
			update_irq_state();
		}
		else if (flags & ARM7_TLB_ABORT_P)
		{
			LOG( ( "ARM7: Translation fault on unmapped virtual address, PC = %08x, vaddr = %08x\n", m_r[eR15], addr ) );
			m_pendingAbtP = true;
			update_irq_state();
		}
		return false;
	}
	else
	{
		// Entry is the physical address of a coarse second-level table
		uint8_t permission = (m_domainAccessControl >> ((desc_lvl1 >> 4) & 0x1e)) & 3;
		uint32_t desc_lvl2 = arm7_tlb_get_second_level_descriptor( (desc_lvl1 & 3) == COPRO_TLB_COARSE_TABLE ? TLB_COARSE : TLB_FINE, desc_lvl1, addr );
		if ((permission != 1) && (permission != 3))
		{
			uint8_t domain = (desc_lvl1 >> 5) & 0xF;
			fatalerror("ARM7: Not Yet Implemented: Coarse Table, Section Domain fault on virtual address, vaddr = %08x, domain = %08x, PC = %08x\n", addr, domain, m_r[eR15]);
		}

		switch( desc_lvl2 & 3 )
		{
			case COPRO_TLB_UNMAPPED:
				// Unmapped, generate a translation fault
				if (flags & ARM7_TLB_ABORT_D)
				{
					uint8_t domain = (desc_lvl1 >> 5) & 0xF;
					LOG( ( "ARM7: Translation fault on unmapped virtual address, vaddr = %08x, PC %08X\n", addr, m_r[eR15] ) );
					m_faultStatus[0] = (7 << 0) | (domain << 4); // 7 = page translation fault
					m_faultAddress = addr;
					m_pendingAbtD = true;
					update_irq_state();
				}
				else if (flags & ARM7_TLB_ABORT_P)
				{
					LOG( ( "ARM7: Translation fault on unmapped virtual address, vaddr = %08x, PC %08X\n", addr, m_r[eR15] ) );
					m_pendingAbtP = true;
					update_irq_state();
				}
				return false;
			case COPRO_TLB_LARGE_PAGE:
				// Large page descriptor
				addr = ( desc_lvl2 & COPRO_TLB_LARGE_PAGE_MASK ) | ( addr & ~COPRO_TLB_LARGE_PAGE_MASK );
				break;
			case COPRO_TLB_SMALL_PAGE:
				// Small page descriptor
				{
					uint8_t ap = ((((desc_lvl2 >> 4) & 0xFF) >> (((addr >> 10) & 3) << 1)) & 3);
					int fault = detect_fault(desc_lvl1, ap, flags);
					if (fault == FAULT_NONE)
					{
						addr = ( desc_lvl2 & COPRO_TLB_SMALL_PAGE_MASK ) | ( addr & ~COPRO_TLB_SMALL_PAGE_MASK );
					}
					else
					{
						if (flags & ARM7_TLB_ABORT_D)
						{
							uint8_t domain = (desc_lvl1 >> 5) & 0xF;
							// hapyfish expects a data abort when something tries to write to a read-only memory location from user mode
							LOG( ( "ARM7: Page Table, Section %s fault on virtual address, vaddr = %08x, PC = %08x\n", (fault == FAULT_DOMAIN) ? "domain" : "permission", addr, m_r[eR15] ) );
							m_faultStatus[0] = ((fault == FAULT_DOMAIN) ? (11 << 0) : (15 << 0)) | (domain << 4); // 11 = page domain fault, 15 = page permission fault
							m_faultAddress = addr;
							m_pendingAbtD = true;
							update_irq_state();
							LOG( ( "vaddr %08X desc_lvl2 %08X domain %d permission %d ap %d s %d r %d mode %d read %d write %d\n",
								addr, desc_lvl2, domain, permission, ap, (m_control & COPRO_CTRL_SYSTEM) ? 1 : 0, (m_control & COPRO_CTRL_ROM) ? 1 : 0,
								m_r[eCPSR] & MODE_FLAG, flags & ARM7_TLB_READ ? 1 : 0,  flags & ARM7_TLB_WRITE ? 1 : 0) );
						}
						else if (flags & ARM7_TLB_ABORT_P)
						{
							LOG( ( "ARM7: Page Table, Section %s fault on virtual address, vaddr = %08x, PC = %08x\n", (fault == FAULT_DOMAIN) ? "domain" : "permission", addr, m_r[eR15] ) );
							m_pendingAbtP = true;
							update_irq_state();
						}
						return false;
					}
				}
				break;
			case COPRO_TLB_TINY_PAGE:
				// Tiny page descriptor
				if( ( desc_lvl1 & 3 ) == 1 )
				{
					LOG( ( "ARM7: It would appear that we're looking up a tiny page from a coarse TLB lookup.  This is bad. vaddr = %08x\n", addr ) );
				}
				addr = ( desc_lvl2 & COPRO_TLB_TINY_PAGE_MASK ) | ( addr & ~COPRO_TLB_TINY_PAGE_MASK );
				break;
		}
	}

	return true;
}


bool arm7_cpu_device::memory_translate(int spacenum, int intention, offs_t &address)
{
	/* only applies to the program address space and only does something if the MMU's enabled */
	if( spacenum == AS_PROGRAM && ( m_control & COPRO_CTRL_MMU_EN ) )
	{
		return arm7_tlb_translate(address, 0);
	}
	return true;
}


/* include the arm7 core */
#include "arm7core.hxx"

/***************************************************************************
 * CPU SPECIFIC IMPLEMENTATIONS
 **************************************************************************/

void arm7_cpu_device::device_start()
{
	m_program = &space(AS_PROGRAM);
	m_direct = &m_program->direct();

	save_item(NAME(m_r));
	save_item(NAME(m_pendingIrq));
	save_item(NAME(m_pendingFiq));
	save_item(NAME(m_pendingAbtD));
	save_item(NAME(m_pendingAbtP));
	save_item(NAME(m_pendingUnd));
	save_item(NAME(m_pendingSwi));
	save_item(NAME(m_pending_interrupt));
	save_item(NAME(m_fcsePID));
	save_item(NAME(m_pid_offset));

	m_icountptr = &m_icount;

	state_add( ARM7_PC,    "PC", m_pc).callexport().formatstr("%08X");
	state_add(STATE_GENPC, "GENPC", m_pc).callexport().noshow();
	state_add(STATE_GENPCBASE, "CURPC", m_pc).callexport().noshow();
	/* registers shared by all operating modes */
	state_add( ARM7_R0,    "R0",   m_r[ 0]).formatstr("%08X");
	state_add( ARM7_R1,    "R1",   m_r[ 1]).formatstr("%08X");
	state_add( ARM7_R2,    "R2",   m_r[ 2]).formatstr("%08X");
	state_add( ARM7_R3,    "R3",   m_r[ 3]).formatstr("%08X");
	state_add( ARM7_R4,    "R4",   m_r[ 4]).formatstr("%08X");
	state_add( ARM7_R5,    "R5",   m_r[ 5]).formatstr("%08X");
	state_add( ARM7_R6,    "R6",   m_r[ 6]).formatstr("%08X");
	state_add( ARM7_R7,    "R7",   m_r[ 7]).formatstr("%08X");
	state_add( ARM7_R8,    "R8",   m_r[ 8]).formatstr("%08X");
	state_add( ARM7_R9,    "R9",   m_r[ 9]).formatstr("%08X");
	state_add( ARM7_R10,   "R10",  m_r[10]).formatstr("%08X");
	state_add( ARM7_R11,   "R11",  m_r[11]).formatstr("%08X");
	state_add( ARM7_R12,   "R12",  m_r[12]).formatstr("%08X");
	state_add( ARM7_R13,   "R13",  m_r[13]).formatstr("%08X");
	state_add( ARM7_R14,   "R14",  m_r[14]).formatstr("%08X");
	state_add( ARM7_R15,   "R15",  m_r[15]).formatstr("%08X");
	/* FIRQ Mode Shadowed Registers */
	state_add( ARM7_FR8,   "FR8",  m_r[eR8_FIQ]  ).formatstr("%08X");
	state_add( ARM7_FR9,   "FR9",  m_r[eR9_FIQ]  ).formatstr("%08X");
	state_add( ARM7_FR10,  "FR10", m_r[eR10_FIQ] ).formatstr("%08X");
	state_add( ARM7_FR11,  "FR11", m_r[eR11_FIQ] ).formatstr("%08X");
	state_add( ARM7_FR12,  "FR12", m_r[eR12_FIQ] ).formatstr("%08X");
	state_add( ARM7_FR13,  "FR13", m_r[eR13_FIQ] ).formatstr("%08X");
	state_add( ARM7_FR14,  "FR14", m_r[eR14_FIQ] ).formatstr("%08X");
	state_add( ARM7_FSPSR, "FR16", m_r[eSPSR_FIQ]).formatstr("%08X");
	/* IRQ Mode Shadowed Registers */
	state_add( ARM7_IR13,  "IR13", m_r[eR13_IRQ] ).formatstr("%08X");
	state_add( ARM7_IR14,  "IR14", m_r[eR14_IRQ] ).formatstr("%08X");
	state_add( ARM7_ISPSR, "IR16", m_r[eSPSR_IRQ]).formatstr("%08X");
	/* Supervisor Mode Shadowed Registers */
	state_add( ARM7_SR13,  "SR13", m_r[eR13_SVC] ).formatstr("%08X");
	state_add( ARM7_SR14,  "SR14", m_r[eR14_SVC] ).formatstr("%08X");
	state_add( ARM7_SSPSR, "SR16", m_r[eSPSR_SVC]).formatstr("%08X");
	/* Abort Mode Shadowed Registers */
	state_add( ARM7_AR13,  "AR13", m_r[eR13_ABT] ).formatstr("%08X");
	state_add( ARM7_AR14,  "AR14", m_r[eR14_ABT] ).formatstr("%08X");
	state_add( ARM7_ASPSR, "AR16", m_r[eSPSR_ABT]).formatstr("%08X");
	/* Undefined Mode Shadowed Registers */
	state_add( ARM7_UR13,  "UR13", m_r[eR13_UND] ).formatstr("%08X");
	state_add( ARM7_UR14,  "UR14", m_r[eR14_UND] ).formatstr("%08X");
	state_add( ARM7_USPSR, "UR16", m_r[eSPSR_UND]).formatstr("%08X");

	state_add(STATE_GENFLAGS, "GENFLAGS", m_r[eCPSR]).formatstr("%13s").noshow();
}


void arm7_cpu_device::state_export(const device_state_entry &entry)
{
	switch (entry.index())
	{
		case STATE_GENPC:
		case STATE_GENPCBASE:
			m_pc = GET_PC;
			break;
	}
}


void arm7_cpu_device::state_string_export(const device_state_entry &entry, std::string &str) const
{
	switch (entry.index())
	{
		case STATE_GENFLAGS:
			str = string_format("%c%c%c%c%c%c%c%c %s",
				(ARM7REG(eCPSR) & N_MASK) ? 'N' : '-',
				(ARM7REG(eCPSR) & Z_MASK) ? 'Z' : '-',
				(ARM7REG(eCPSR) & C_MASK) ? 'C' : '-',
				(ARM7REG(eCPSR) & V_MASK) ? 'V' : '-',
				(ARM7REG(eCPSR) & Q_MASK) ? 'Q' : '-',
				(ARM7REG(eCPSR) & I_MASK) ? 'I' : '-',
				(ARM7REG(eCPSR) & F_MASK) ? 'F' : '-',
				(ARM7REG(eCPSR) & T_MASK) ? 'T' : '-',
				GetModeText(ARM7REG(eCPSR)));
		break;
	}
}

void arm7_cpu_device::device_reset()
{
	memset(m_r, 0, sizeof(m_r));
	m_pendingIrq = false;
	m_pendingFiq = false;
	m_pendingAbtD = false;
	m_pendingAbtP = false;
	m_pendingUnd = false;
	m_pendingSwi = false;
	m_pending_interrupt = false;
	m_control = 0;
	m_tlbBase = 0;
	m_tlb_base_mask = 0;
	m_faultStatus[0] = 0;
	m_faultStatus[1] = 0;
	m_faultAddress = 0;
	m_fcsePID = 0;
	m_pid_offset = 0;
	m_domainAccessControl = 0;
	memset(m_decoded_access_control, 0, sizeof(uint8_t) * 16);

	/* start up in SVC mode with interrupts disabled. */
	m_r[eCPSR] = I_MASK | F_MASK | 0x10;
	SwitchMode(eARM7_MODE_SVC);
	m_r[eR15] = 0;

	m_impstate.cache_dirty = true;
}


#define UNEXECUTED() \
	m_r[eR15] += 4; \
	m_icount +=2; /* Any unexecuted instruction only takes 1 cycle (page 193) */

void arm7_cpu_device::execute_run()
{
	uint32_t insn;

	do
	{
		uint32_t pc = GET_PC;

		debugger_instruction_hook(this, pc);

		/* handle Thumb instructions if active */
		if (T_IS_SET(m_r[eCPSR]))
		{
			offs_t raddr;

			pc = m_r[eR15];

			// "In Thumb state, bit [0] is undefined and must be ignored. Bits [31:1] contain the PC."
			raddr = pc & (~1);

			if ( m_control & COPRO_CTRL_MMU_EN )
			{
				if (!arm7_tlb_translate(raddr, ARM7_TLB_ABORT_P | ARM7_TLB_READ))
				{
					goto skip_exec;
				}
			}

			insn = m_direct->read_word(raddr);
			(this->*thumb_handler[(insn & 0xffc0) >> 6])(pc, insn);

		}
		else
		{
			offs_t raddr;

			/* load 32 bit instruction */

			// "In ARM state, bits [1:0] of r15 are undefined and must be ignored. Bits [31:2] contain the PC."
			raddr = pc & (~3);

			if ( m_control & COPRO_CTRL_MMU_EN )
			{
				if (!arm7_tlb_translate(raddr, ARM7_TLB_ABORT_P | ARM7_TLB_READ))
				{
					goto skip_exec;
				}
			}

#if 0
			if (MODE26)
			{
				uint32_t temp1, temp2;
				temp1 = GET_CPSR & 0xF00000C3;
				temp2 = (R15 & 0xF0000000) | ((R15 & 0x0C000000) >> (26 - 6)) | (R15 & 0x00000003);
				if (temp1 != temp2) fatalerror( "%08X: 32-bit and 26-bit modes are out of sync (%08X %08X)\n", pc, temp1, temp2);
			}
#endif

			insn = m_direct->read_dword(raddr);

			int op_offset = 0;
			/* process condition codes for this instruction */
			if ((insn >> INSN_COND_SHIFT) != COND_AL)
			{
				switch (insn >> INSN_COND_SHIFT)
				{
					case COND_EQ:
						if (Z_IS_CLEAR(m_r[eCPSR]))
							{ UNEXECUTED();  goto skip_exec; }
						break;
					case COND_NE:
						if (Z_IS_SET(m_r[eCPSR]))
							{ UNEXECUTED();  goto skip_exec; }
						break;
					case COND_CS:
						if (C_IS_CLEAR(m_r[eCPSR]))
							{ UNEXECUTED();  goto skip_exec; }
						break;
					case COND_CC:
						if (C_IS_SET(m_r[eCPSR]))
							{ UNEXECUTED();  goto skip_exec; }
						break;
					case COND_MI:
						if (N_IS_CLEAR(m_r[eCPSR]))
							{ UNEXECUTED();  goto skip_exec; }
						break;
					case COND_PL:
						if (N_IS_SET(m_r[eCPSR]))
							{ UNEXECUTED();  goto skip_exec; }
						break;
					case COND_VS:
						if (V_IS_CLEAR(m_r[eCPSR]))
							{ UNEXECUTED();  goto skip_exec; }
						break;
					case COND_VC:
						if (V_IS_SET(m_r[eCPSR]))
							{ UNEXECUTED();  goto skip_exec; }
						break;
					case COND_HI:
						if (C_IS_CLEAR(m_r[eCPSR]) || Z_IS_SET(m_r[eCPSR]))
							{ UNEXECUTED();  goto skip_exec; }
						break;
					case COND_LS:
						if (C_IS_SET(m_r[eCPSR]) && Z_IS_CLEAR(m_r[eCPSR]))
							{ UNEXECUTED();  goto skip_exec; }
						break;
					case COND_GE:
						if (!(m_r[eCPSR] & N_MASK) != !(m_r[eCPSR] & V_MASK)) /* Use x ^ (x >> ...) method */
							{ UNEXECUTED();  goto skip_exec; }
						break;
					case COND_LT:
						if (!(m_r[eCPSR] & N_MASK) == !(m_r[eCPSR] & V_MASK))
							{ UNEXECUTED();  goto skip_exec; }
						break;
					case COND_GT:
						if (Z_IS_SET(m_r[eCPSR]) || (!(m_r[eCPSR] & N_MASK) != !(m_r[eCPSR] & V_MASK)))
							{ UNEXECUTED();  goto skip_exec; }
						break;
					case COND_LE:
						if (Z_IS_CLEAR(m_r[eCPSR]) && (!(m_r[eCPSR] & N_MASK) == !(m_r[eCPSR] & V_MASK)))
							{ UNEXECUTED();  goto skip_exec; }
						break;
					case COND_NV:
						if (m_archRev < 5)
						  { UNEXECUTED();  goto skip_exec; }
						else
							op_offset = 0x10;
						break;
				}
			}
			/*******************************************************************/
			/* If we got here - condition satisfied, so decode the instruction */
			/*******************************************************************/
			(this->*ops_handler[((insn & 0xF000000) >> 24) + op_offset])(insn);
		}

skip_exec:

		arm7_check_irq_state();

		/* All instructions remove 3 cycles.. Others taking less / more will have adjusted this # prior to here */
		m_icount -= 3;
	} while (m_icount > 0);
}


void arm7_cpu_device::execute_set_input(int irqline, int state)
{
	switch (irqline) {
	case ARM7_IRQ_LINE: /* IRQ */
		m_pendingIrq = state ? true : false;
		break;

	case ARM7_FIRQ_LINE: /* FIRQ */
		m_pendingFiq = state ? true : false;
		break;

	case ARM7_ABORT_EXCEPTION:
		m_pendingAbtD = state ? true : false;
		break;
	case ARM7_ABORT_PREFETCH_EXCEPTION:
		m_pendingAbtP = state ? true : false;
		break;

	case ARM7_UNDEFINE_EXCEPTION:
		m_pendingUnd = state ? true : false;
		break;
	}

	update_irq_state();
	arm7_check_irq_state();
}


offs_t arm7_cpu_device::disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options)
{
	extern CPU_DISASSEMBLE( arm7arm );
	extern CPU_DISASSEMBLE( arm7thumb );
	extern CPU_DISASSEMBLE( arm7arm_be );
	extern CPU_DISASSEMBLE( arm7thumb_be );

	if (T_IS_SET(m_r[eCPSR]))
	{
		if ( m_endian == ENDIANNESS_BIG )
			return CPU_DISASSEMBLE_NAME(arm7thumb_be)(this, stream, pc, oprom, opram, options);
		else
			return CPU_DISASSEMBLE_NAME(arm7thumb)(this, stream, pc, oprom, opram, options);
	}
	else
	{
		if ( m_endian == ENDIANNESS_BIG )
			return CPU_DISASSEMBLE_NAME(arm7arm_be)(this, stream, pc, oprom, opram, options);
		else
			return CPU_DISASSEMBLE_NAME(arm7arm)(this, stream, pc, oprom, opram, options);
	}
}


/* ARM system coprocessor support */

WRITE32_MEMBER( arm7_cpu_device::arm7_do_callback )
{
	m_pendingUnd = true;
	update_irq_state();
}

READ32_MEMBER( arm7_cpu_device::arm7_rt_r_callback )
{
	uint32_t opcode = offset;
	uint8_t cReg = ( opcode & INSN_COPRO_CREG ) >> INSN_COPRO_CREG_SHIFT;
	uint8_t op2 =  ( opcode & INSN_COPRO_OP2 )  >> INSN_COPRO_OP2_SHIFT;
	uint8_t op3 =    opcode & INSN_COPRO_OP3;
	uint8_t cpnum = (opcode & INSN_COPRO_CPNUM) >> INSN_COPRO_CPNUM_SHIFT;
	uint32_t data = 0;

//    printf("cpnum %d cReg %d op2 %d op3 %d (%x)\n", cpnum, cReg, op2, op3, GET_REGISTER(arm, 15));

	// we only handle system copro here
	if (cpnum != 15)
	{
		if (m_archFlags & eARM_ARCHFLAGS_XSCALE)
	{
		// handle XScale specific CP14
		if (cpnum == 14)
		{
			switch( cReg )
			{
				case 1: // clock counter
					data = (uint32_t)total_cycles();
					break;

				default:
					break;
			}
		}
		else
		{
			fatalerror("XScale: Unhandled coprocessor %d (archFlags %x)\n", cpnum, m_archFlags);
		}

		return data;
	}
	else
	{
		LOG( ("ARM7: Unhandled coprocessor %d (archFlags %x)\n", cpnum, m_archFlags) );
		m_pendingUnd = true;
		update_irq_state();
		return 0;
	}
	}

	switch( cReg )
	{
		case 4:
		case 7:
		case 8:
		case 9:
		case 10:
		case 11:
		case 12:
			// RESERVED
			LOG( ( "arm7_rt_r_callback CR%d, RESERVED\n", cReg ) );
			break;
		case 0:             // ID
		switch(op2)
		{
			case 0:
			switch (m_archRev)
			{
				case 3: // ARM6 32-bit
				data = 0x41;
				break;

				case 4: // ARM7/SA11xx
					if (m_archFlags & eARM_ARCHFLAGS_SA)
					{
						// ARM Architecture Version 4
						// Part Number 0xB11 (SA1110)
						// Stepping B5
							data = 0x69 | ( 0x01 << 16 ) | ( 0xB11 << 4 ) | 0x9;
					}
					else
					{
						data = m_copro_id;
					}
					break;

				case 5: // ARM9/10/XScale
					data = 0x41 | (9 << 12);
					if (m_archFlags & eARM_ARCHFLAGS_T)
					{
						if (m_archFlags & eARM_ARCHFLAGS_E)
						{
							if (m_archFlags & eARM_ARCHFLAGS_J)
							{
								data |= (6<<16);    // v5TEJ
							}
							else
							{
								data |= (5<<16);    // v5TE
							}
						}
						else
						{
							data |= (4<<16);    // v5T
						}
					}

					/* ID from PXA-250 manual */
					//data = 0x69052100;
					break;

				case 6: // ARM11
					data = 0x41 | (10<< 12) | (7<<16);  // v6
					break;
			}
			break;
		case 1: // cache type
			data = 0x0f0d2112;  // HACK: value expected by ARMWrestler (probably Nintendo DS ARM9's value)
			//data = (6 << 25) | (1 << 24) | (0x172 << 12) | (0x172 << 0); // ARM920T (S3C24xx)
			break;
		case 2: // TCM type
			data = 0;
			break;
		case 3: // TLB type
			data = 0;
			break;
		case 4: // MPU type
			data = 0;
			break;
		}
			LOG( ( "arm7_rt_r_callback, ID %02x (%02x) -> %08x (PC=%08x)\n",op2,m_archRev,data,GET_PC ) );
			break;
		case 1:             // Control
			data = COPRO_CTRL | 0x70;   // bits 4-6 always read back as "1" (bit 3 too in XScale)
			break;
		case 2:             // Translation Table Base
			data = COPRO_TLB_BASE;
			break;
		case 3:             // Domain Access Control
			LOG( ( "arm7_rt_r_callback, Domain Access Control\n" ) );
			data = COPRO_DOMAIN_ACCESS_CONTROL;
			break;
		case 5:             // Fault Status
			LOG( ( "arm7_rt_r_callback, Fault Status\n" ) );
			switch (op3)
			{
				case 0: data = COPRO_FAULT_STATUS_D; break;
				case 1: data = COPRO_FAULT_STATUS_P; break;
			}
			break;
		case 6:             // Fault Address
			LOG( ( "arm7_rt_r_callback, Fault Address\n" ) );
			data = COPRO_FAULT_ADDRESS;
			break;
		case 13:            // Read Process ID (PID)
			LOG( ( "arm7_rt_r_callback, Read PID\n" ) );
			data = COPRO_FCSE_PID;
			break;
		case 14:            // Read Breakpoint
			LOG( ( "arm7_rt_r_callback, Read Breakpoint\n" ) );
			break;
		case 15:            // Test, Clock, Idle
			LOG( ( "arm7_rt_r_callback, Test / Clock / Idle \n" ) );
			break;
	}

	return data;
}

WRITE32_MEMBER( arm7_cpu_device::arm7_rt_w_callback )
{
	uint32_t opcode = offset;
	uint8_t cReg = ( opcode & INSN_COPRO_CREG ) >> INSN_COPRO_CREG_SHIFT;
	uint8_t op2 =  ( opcode & INSN_COPRO_OP2 )  >> INSN_COPRO_OP2_SHIFT;
	uint8_t op3 =    opcode & INSN_COPRO_OP3;
	uint8_t cpnum = (opcode & INSN_COPRO_CPNUM) >> INSN_COPRO_CPNUM_SHIFT;

	// handle XScale specific CP14 - just eat writes for now
	if (cpnum != 15)
	{
		if (cpnum == 14)
		{
			LOG( ("arm7_rt_w_callback: write %x to XScale CP14 reg %d\n", data, cReg) );
			return;
		}
		else
		{
			LOG( ("ARM7: Unhandled coprocessor %d\n", cpnum) );
			m_pendingUnd = true;
			update_irq_state();
			return;
		}
	}

	switch( cReg )
	{
		case 0:
		case 4:
		case 10:
		case 11:
		case 12:
			// RESERVED
			LOG( ( "arm7_rt_w_callback CR%d, RESERVED = %08x\n", cReg, data) );
			break;
		case 1:             // Control
			LOG( ( "arm7_rt_w_callback Control = %08x (%d) (%d)\n", data, op2, op3 ) );
			LOG( ( "    MMU:%d, Address Fault:%d, Data Cache:%d, Write Buffer:%d\n",
					data & COPRO_CTRL_MMU_EN, ( data & COPRO_CTRL_ADDRFAULT_EN ) >> COPRO_CTRL_ADDRFAULT_EN_SHIFT,
					( data & COPRO_CTRL_DCACHE_EN ) >> COPRO_CTRL_DCACHE_EN_SHIFT,
					( data & COPRO_CTRL_WRITEBUF_EN ) >> COPRO_CTRL_WRITEBUF_EN_SHIFT ) );
			LOG( ( "    Endianness:%d, System:%d, ROM:%d, Instruction Cache:%d\n",
					( data & COPRO_CTRL_ENDIAN ) >> COPRO_CTRL_ENDIAN_SHIFT,
					( data & COPRO_CTRL_SYSTEM ) >> COPRO_CTRL_SYSTEM_SHIFT,
					( data & COPRO_CTRL_ROM ) >> COPRO_CTRL_ROM_SHIFT,
					( data & COPRO_CTRL_ICACHE_EN ) >> COPRO_CTRL_ICACHE_EN_SHIFT ) );
			LOG( ( "    Int Vector Adjust:%d\n", ( data & COPRO_CTRL_INTVEC_ADJUST ) >> COPRO_CTRL_INTVEC_ADJUST_SHIFT ) );
#if ARM7_MMU_ENABLE_HACK
			if (((data & COPRO_CTRL_MMU_EN) != 0) && ((COPRO_CTRL & COPRO_CTRL_MMU_EN) == 0))
			{
				>m_mmu_enable_addr = R15;
			}
			if (((data & COPRO_CTRL_MMU_EN) == 0) && ((COPRO_CTRL & COPRO_CTRL_MMU_EN) != 0))
			{
				if (!arm7_tlb_translate( R15, 0))
				{
					fatalerror("ARM7_MMU_ENABLE_HACK translate failed\n");
				}
			}
#endif
			COPRO_CTRL = data & COPRO_CTRL_MASK;
			break;
		case 2:             // Translation Table Base
			LOG( ( "arm7_rt_w_callback TLB Base = %08x (%d) (%d)\n", data, op2, op3 ) );
			COPRO_TLB_BASE = data;
			m_tlb_base_mask = data & COPRO_TLB_BASE_MASK;
			break;
		case 3:             // Domain Access Control
			LOG( ( "arm7_rt_w_callback Domain Access Control = %08x (%d) (%d)\n", data, op2, op3 ) );
			COPRO_DOMAIN_ACCESS_CONTROL = data;
			for (int i = 0; i < 32; i += 2)
			{
				m_decoded_access_control[i >> 1] = (COPRO_DOMAIN_ACCESS_CONTROL >> i) & 3;
			}
			break;
		case 5:             // Fault Status
			LOG( ( "arm7_rt_w_callback Fault Status = %08x (%d) (%d)\n", data, op2, op3 ) );
			switch (op3)
			{
				case 0: COPRO_FAULT_STATUS_D = data; break;
				case 1: COPRO_FAULT_STATUS_P = data; break;
			}
			break;
		case 6:             // Fault Address
			LOG( ( "arm7_rt_w_callback Fault Address = %08x (%d) (%d)\n", data, op2, op3 ) );
			COPRO_FAULT_ADDRESS = data;
			break;
		case 7:             // Cache Operations
//            LOG( ( "arm7_rt_w_callback Cache Ops = %08x (%d) (%d)\n", data, op2, op3 ) );
			break;
		case 8:             // TLB Operations
			LOG( ( "arm7_rt_w_callback TLB Ops = %08x (%d) (%d)\n", data, op2, op3 ) );
			break;
		case 9:             // Read Buffer Operations
			LOG( ( "arm7_rt_w_callback Read Buffer Ops = %08x (%d) (%d)\n", data, op2, op3 ) );
			break;
		case 13:            // Write Process ID (PID)
			LOG( ( "arm7_rt_w_callback Write PID = %08x (%d) (%d)\n", data, op2, op3 ) );
			COPRO_FCSE_PID = data;
			m_pid_offset = (((COPRO_FCSE_PID >> 25) & 0x7F)) * 0x2000000;
			break;
		case 14:            // Write Breakpoint
			LOG( ( "arm7_rt_w_callback Write Breakpoint = %08x (%d) (%d)\n", data, op2, op3 ) );
			break;
		case 15:            // Test, Clock, Idle
			LOG( ( "arm7_rt_w_callback Test / Clock / Idle = %08x (%d) (%d)\n", data, op2, op3 ) );
			break;
	}
}


void arm7_cpu_device::arm7_dt_r_callback(uint32_t insn, uint32_t *prn)
{
	uint8_t cpn = (insn >> 8) & 0xF;
	if ((m_archFlags & eARM_ARCHFLAGS_XSCALE) && (cpn == 0))
	{
		LOG( ( "arm7_dt_r_callback: DSP Coprocessor 0 (CP0) not yet emulated (PC %08x)\n", GET_PC ) );
	}
	else
	{
		m_pendingUnd = true;
		update_irq_state();
	}
}


void arm7_cpu_device::arm7_dt_w_callback(uint32_t insn, uint32_t *prn)
{
	uint8_t cpn = (insn >> 8) & 0xF;
	if ((m_archFlags & eARM_ARCHFLAGS_XSCALE) && (cpn == 0))
	{
		LOG( ( "arm7_dt_w_callback: DSP Coprocessor 0 (CP0) not yet emulated (PC %08x)\n", GET_PC ) );
	}
	else
	{
		m_pendingUnd = true;
		update_irq_state();
	}
}


/***************************************************************************
 * Default Memory Handlers
 ***************************************************************************/
void arm7_cpu_device::arm7_cpu_write32(uint32_t addr, uint32_t data)
{
	if( COPRO_CTRL & COPRO_CTRL_MMU_EN )
	{
		if (!arm7_tlb_translate( addr, ARM7_TLB_ABORT_D | ARM7_TLB_WRITE ))
		{
			return;
		}
	}

	addr &= ~3;
	m_program->write_dword(addr, data);
}


void arm7_cpu_device::arm7_cpu_write16(uint32_t addr, uint16_t data)
{
	if( COPRO_CTRL & COPRO_CTRL_MMU_EN )
	{
		if (!arm7_tlb_translate( addr, ARM7_TLB_ABORT_D | ARM7_TLB_WRITE ))
		{
			return;
		}
	}

	addr &= ~1;
	m_program->write_word(addr, data);
}

void arm7_cpu_device::arm7_cpu_write8(uint32_t addr, uint8_t data)
{
	if( COPRO_CTRL & COPRO_CTRL_MMU_EN )
	{
		if (!arm7_tlb_translate( addr, ARM7_TLB_ABORT_D | ARM7_TLB_WRITE ))
		{
			return;
		}
	}

	m_program->write_byte(addr, data);
}

uint32_t arm7_cpu_device::arm7_cpu_read32(uint32_t addr)
{
	uint32_t result;

	if( COPRO_CTRL & COPRO_CTRL_MMU_EN )
	{
		if (!arm7_tlb_translate( addr, ARM7_TLB_ABORT_D | ARM7_TLB_READ ))
		{
			return 0;
		}
	}

	if (addr & 3)
	{
		result = m_program->read_dword(addr & ~3);
		result = (result >> (8 * (addr & 3))) | (result << (32 - (8 * (addr & 3))));
	}
	else
	{
		result = m_program->read_dword(addr);
	}

	return result;
}

uint16_t arm7_cpu_device::arm7_cpu_read16(uint32_t addr)
{
	uint16_t result;

	if( COPRO_CTRL & COPRO_CTRL_MMU_EN )
	{
		if (!arm7_tlb_translate( addr, ARM7_TLB_ABORT_D | ARM7_TLB_READ ))
		{
			return 0;
		}
	}

	result = m_program->read_word(addr & ~1);

	if (addr & 1)
	{
		result = ((result >> 8) & 0xff) | ((result & 0xff) << 8);
	}

	return result;
}

uint8_t arm7_cpu_device::arm7_cpu_read8(uint32_t addr)
{
	if( COPRO_CTRL & COPRO_CTRL_MMU_EN )
	{
		if (!arm7_tlb_translate( addr, ARM7_TLB_ABORT_D | ARM7_TLB_READ ))
		{
			return 0;
		}
	}

	// Handle through normal 8 bit handler (for 32 bit cpu)
	return m_program->read_byte(addr);
}

#include "arm7drc.hxx"
