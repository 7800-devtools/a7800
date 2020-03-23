// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    mips3.c
    Core implementation for the portable MIPS III/IV emulator.
    Written by Aaron Giles

***************************************************************************/

#include "emu.h"
#include "debugger.h"
#include "mips3.h"
#include "mips3com.h"


#define ENABLE_OVERFLOWS    0

/***************************************************************************
    HELPER MACROS
***************************************************************************/

#define RSVAL32     ((uint32_t)m_core->r[RSREG])
#define RTVAL32     ((uint32_t)m_core->r[RTREG])
#define RDVAL32     ((uint32_t)m_core->r[RDREG])

#define RSVAL64     (m_core->r[RSREG])
#define RTVAL64     (m_core->r[RTREG])
#define RDVAL64     (m_core->r[RDREG])

#define FRVALS_FR0  (((float *)&m_core->cpr[1][FRREG & 0x1E])[BYTE_XOR_LE(FRREG & 1)])
#define FTVALS_FR0  (((float *)&m_core->cpr[1][FTREG & 0x1E])[BYTE_XOR_LE(FTREG & 1)])
#define FSVALS_FR0  (((float *)&m_core->cpr[1][FSREG & 0x1E])[BYTE_XOR_LE(FSREG & 1)])
#define FDVALS_FR0  (((float *)&m_core->cpr[1][FDREG & 0x1E])[BYTE_XOR_LE(FDREG & 1)])
#define FSVALW_FR0  (((uint32_t *)&m_core->cpr[1][FSREG & 0x1E])[BYTE_XOR_LE(FSREG & 1)])
#define FDVALW_FR0  (((uint32_t *)&m_core->cpr[1][FDREG & 0x1E])[BYTE_XOR_LE(FDREG & 1)])

#define FRVALD_FR0  (*(double *)&m_core->cpr[1][FRREG & 0x1E])
#define FTVALD_FR0  (*(double *)&m_core->cpr[1][FTREG & 0x1E])
#define FSVALD_FR0  (*(double *)&m_core->cpr[1][FSREG & 0x1E])
#define FDVALD_FR0  (*(double *)&m_core->cpr[1][FDREG & 0x1E])
#define FSVALL_FR0  (*(uint64_t *)&m_core->cpr[1][FSREG & 0x1E])
#define FDVALL_FR0  (*(uint64_t *)&m_core->cpr[1][FDREG & 0x1E])

#define FRVALS_FR1  (((float *)&m_core->cpr[1][FRREG])[BYTE_XOR_LE(0)])
#define FTVALS_FR1  (((float *)&m_core->cpr[1][FTREG])[BYTE_XOR_LE(0)])
#define FSVALS_FR1  (((float *)&m_core->cpr[1][FSREG])[BYTE_XOR_LE(0)])
#define FDVALS_FR1  (((float *)&m_core->cpr[1][FDREG])[BYTE_XOR_LE(0)])
#define FSVALW_FR1  (((uint32_t *)&m_core->cpr[1][FSREG])[BYTE_XOR_LE(0)])
#define FDVALW_FR1  (((uint32_t *)&m_core->cpr[1][FDREG])[BYTE_XOR_LE(0)])

#define FRVALD_FR1  (*(double *)&m_core->cpr[1][FRREG])
#define FTVALD_FR1  (*(double *)&m_core->cpr[1][FTREG])
#define FSVALD_FR1  (*(double *)&m_core->cpr[1][FSREG])
#define FDVALD_FR1  (*(double *)&m_core->cpr[1][FDREG])
#define FSVALL_FR1  (*(uint64_t *)&m_core->cpr[1][FSREG])
#define FDVALL_FR1  (*(uint64_t *)&m_core->cpr[1][FDREG])

#define ADDPC(x)    m_nextpc = m_core->pc + ((x) << 2)
#define ABSPC(x)    m_nextpc = (m_core->pc & 0xf0000000) | ((x) << 2)
#define ABSPCL(x,l) { m_nextpc = (m_core->pc & 0xf0000000) | ((x) << 2); m_core->r[l] = (int32_t)(m_core->pc + 4); }
#define SETPC(x)    m_nextpc = (x)
#define SETPCL(x,l) { m_nextpc = (x); m_core->r[l] = (int32_t)(m_core->pc + 4); }

#define HIVAL       (uint32_t)m_core->r[REG_HI]
#define LOVAL       (uint32_t)m_core->r[REG_LO]
#define HIVAL64     m_core->r[REG_HI]
#define LOVAL64     m_core->r[REG_LO]
#define SR          m_core->cpr[0][COP0_Status]
#define CAUSE       m_core->cpr[0][COP0_Cause]

#define GET_FCC(n)  (m_cf[1][n])
#define SET_FCC(n,v) (m_cf[1][n] = (v))

#define IS_FR0      (!(SR & SR_FR))
#define IS_FR1      (SR & SR_FR)

/* size of the execution code cache */
#define CACHE_SIZE                      (32 * 1024 * 1024)



static const uint8_t fcc_shift[8] = { 23, 25, 26, 27, 28, 29, 30, 31 };

/* lookup table for FP modes */
static const uint8_t fpmode_source[4] =
{
	uml::ROUND_ROUND,
	uml::ROUND_TRUNC,
	uml::ROUND_CEIL,
	uml::ROUND_FLOOR
};

/***************************************************************************
    MEMORY ACCESSORS
***************************************************************************/

#define ROPCODE(pc)     direct->read_dword(pc)


DEFINE_DEVICE_TYPE(VR4300BE,  vr4300be_device,  "vr4300be",  "VR4300 (big)")
DEFINE_DEVICE_TYPE(VR4300LE,  vr4300le_device,  "vr4300le",  "VR4300 (little)")
DEFINE_DEVICE_TYPE(VR4310BE,  vr4310be_device,  "vr4310be",  "VR4310 (big)")
DEFINE_DEVICE_TYPE(VR4310LE,  vr4310le_device,  "vr4310le",  "VR4310 (little)")
DEFINE_DEVICE_TYPE(R4600BE,   r4600be_device,   "r4600be",   "R4600 (big)")
DEFINE_DEVICE_TYPE(R4600LE,   r4600le_device,   "r4600le",   "R4600 (little)")
DEFINE_DEVICE_TYPE(R4650BE,   r4650be_device,   "r4650be",   "IDT R4650 (big)")
DEFINE_DEVICE_TYPE(R4650LE,   r4650le_device,   "r4650le",   "IDT R4650 (little)")
DEFINE_DEVICE_TYPE(R4700BE,   r4700be_device,   "r4700be",   "R4700 (big)")
DEFINE_DEVICE_TYPE(R4700LE,   r4700le_device,   "r4700le",   "R4700 (little)")
DEFINE_DEVICE_TYPE(TX4925BE,  tx4925be_device,  "tx4925be",  "TX4925 (big)")
DEFINE_DEVICE_TYPE(TX4925LE,  tx4925le_device,  "tx4925le",  "TX4925 (little)")
DEFINE_DEVICE_TYPE(R5000BE,   r5000be_device,   "r5000be",   "R5000 (big)")
DEFINE_DEVICE_TYPE(R5000LE,   r5000le_device,   "r5000le",   "R5000 (little)")
DEFINE_DEVICE_TYPE(VR5500BE,  vr5500be_device,  "vr5500be",  "VR5500 (big)")
DEFINE_DEVICE_TYPE(VR5500LE,  vr5500le_device,  "vr5500le",  "VR5500 (little)")
DEFINE_DEVICE_TYPE(QED5271BE, qed5271be_device, "qed5271be", "QED5271 (big)")
DEFINE_DEVICE_TYPE(QED5271LE, qed5271le_device, "qed5271le", "QED5271 (little)")
DEFINE_DEVICE_TYPE(RM7000BE,  rm7000be_device,  "rm7000be",  "RM7000 (big)")
DEFINE_DEVICE_TYPE(RM7000LE,  rm7000le_device,  "rm7000le",  "RM7000 (little)")


// VR4300 and VR5432 have 4 fewer PFN bits, and only 32 TLB entries
mips3_device::mips3_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, mips3_flavor flavor, endianness_t endianness)
	: cpu_device(mconfig, type, tag, owner, clock)
	, device_vtlb_interface(mconfig, *this, AS_PROGRAM)
	, m_program_config("program", endianness, 32, 32, 0, 32, MIPS3_MIN_PAGE_SHIFT)
	, m_flavor(flavor)
	, m_core(nullptr)
	, m_ppc(0)
	, m_nextpc(0)
	, m_pcbase(0)
	, m_delayslot(false)
	, m_op(0)
	, m_interrupt_cycles(0)
	, m_ll_value(0)
	, m_lld_value(0)
	, m_badcop_value(0)
	, m_lwl(endianness == ENDIANNESS_BIG ? &mips3_device::lwl_be : &mips3_device::lwl_le)
	, m_lwr(endianness == ENDIANNESS_BIG ? &mips3_device::lwr_be : &mips3_device::lwr_le)
	, m_swl(endianness == ENDIANNESS_BIG ? &mips3_device::swl_be : &mips3_device::swl_le)
	, m_swr(endianness == ENDIANNESS_BIG ? &mips3_device::swr_be : &mips3_device::swr_le)
	, m_ldl(endianness == ENDIANNESS_BIG ? &mips3_device::ldl_be : &mips3_device::ldl_le)
	, m_ldr(endianness == ENDIANNESS_BIG ? &mips3_device::ldr_be : &mips3_device::ldr_le)
	, m_sdl(endianness == ENDIANNESS_BIG ? &mips3_device::sdl_be : &mips3_device::sdl_le)
	, m_sdr(endianness == ENDIANNESS_BIG ? &mips3_device::sdr_be : &mips3_device::sdr_le)
	, c_system_clock(0)
	, m_pfnmask(flavor == MIPS3_TYPE_VR4300 ? 0x000fffff : 0x00ffffff)
	, m_tlbentries(flavor == MIPS3_TYPE_VR4300 ? 32 : MIPS3_MAX_TLB_ENTRIES)
	, m_bigendian(endianness == ENDIANNESS_BIG)
	, m_byte_xor(m_bigendian ? BYTE4_XOR_BE(0) : BYTE4_XOR_LE(0))
	, m_word_xor(m_bigendian ? WORD_XOR_BE(0) : WORD_XOR_LE(0))
	, c_icache_size(0)
	, c_dcache_size(0)
	, m_fastram_select(0)
	, m_debugger_temp(0)
	, m_cache(CACHE_SIZE + sizeof(internal_mips3_state))
	, m_drcuml(nullptr)
	, m_drcfe(nullptr)
	, m_drcoptions(0)
	, m_cache_dirty(0)
	, m_entry(nullptr)
	, m_nocode(nullptr)
	, m_out_of_cycles(nullptr)
	, m_tlb_mismatch(nullptr)
	, m_hotspot_select(0)
{
	memset(m_fpmode, 0, sizeof(m_fpmode));

	for (int i = 0; i < 3; i++)
	{
		m_read8[i] = nullptr;
		m_write8[i] = nullptr;
		m_read16[i] = nullptr;
		m_write16[i] = nullptr;
		m_read32[i] = nullptr;
		m_read32mask[i] = nullptr;
		m_write32[i] = nullptr;
		m_write32mask[i] = nullptr;
		m_read64[i] = nullptr;
		m_read64mask[i] = nullptr;
		m_write64[i] = nullptr;
		m_write64mask[i] = nullptr;
	}

	for (int i = 0; i < 18; i++)
	{
		m_exception[i] = nullptr;
		m_exception_norecover[i] = nullptr;
	}
	memset(m_fastram, 0, sizeof(m_fastram));
	memset(m_hotspot, 0, sizeof(m_hotspot));

	// configure the virtual TLB
	set_vtlb_fixed_entries(2 * m_tlbentries + 2);
}

device_memory_interface::space_config_vector mips3_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM, &m_program_config)
	};
}


void mips3_device::device_stop()
{
	if (m_drcfe != nullptr)
	{
		m_drcfe = nullptr;
	}
	if (m_drcuml != nullptr)
	{
		m_drcuml = nullptr;
	}
}

/***************************************************************************
    EXECEPTION HANDLING
***************************************************************************/

void mips3_device::generate_exception(int exception, int backup)
{
	uint32_t offset = 0x180;
/*
    useful for catching exceptions:

    if (exception != 0)
    {
        fprintf(stderr, "Exception: PC=%08X, PPC=%08X\n", m_core->pc, m_ppc);
        machine().debug_break();
    }
*/

	/* back up if necessary */
	if (backup)
		m_core->pc = m_ppc;

	/* translate our fake fill exceptions into real exceptions */
	if (exception == EXCEPTION_TLBLOAD_FILL || exception == EXCEPTION_TLBSTORE_FILL)
	{
		offset = 0;
		exception = (exception - EXCEPTION_TLBLOAD_FILL) + EXCEPTION_TLBLOAD;
	}

	/* set the exception PC */
	m_core->cpr[0][COP0_EPC] = m_core->pc;

	/* put the cause in the low 8 bits and clear the branch delay flag */
	CAUSE = (CAUSE & ~0x800000ff) | (exception << 2);

	/* set the appropriate bits for coprocessor exceptions */
	if(exception == EXCEPTION_BADCOP)
	{
		CAUSE |= m_badcop_value << 28;
	}

	/* if we were in a branch delay slot, adjust */
	if ((m_nextpc != ~0) || (m_delayslot))
	{
		m_delayslot = false;
		m_nextpc = ~0;
		m_core->cpr[0][COP0_EPC] -= 4;
		CAUSE |= 0x80000000;
	}

	/* set the exception level */
	SR |= SR_EXL;

	/* based on the BEV bit, we either go to ROM or RAM */
	m_core->pc = (SR & SR_BEV) ? 0xbfc00200 : 0x80000000;

	/* most exceptions go to offset 0x180, except for TLB stuff */
	if (exception >= EXCEPTION_TLBMOD && exception <= EXCEPTION_TLBSTORE)
	{
		osd_printf_debug("TLB miss @ %08X\n", (uint32_t)m_core->cpr[0][COP0_BadVAddr]);
	}
	m_core->pc += offset;

/*
    useful for tracking interrupts

    if ((CAUSE & 0x7f) == 0)
        logerror("Took interrupt -- Cause = %08X, PC =  %08X\n", (uint32_t)CAUSE, m_core->pc);
*/
}


void mips3_device::generate_tlb_exception(int exception, offs_t address)
{
	m_core->cpr[0][COP0_BadVAddr] = address;
	if(exception == EXCEPTION_TLBLOAD || exception == EXCEPTION_TLBSTORE || exception == EXCEPTION_TLBLOAD_FILL || exception == EXCEPTION_TLBSTORE_FILL)
	{
		m_core->cpr[0][COP0_Context] = (m_core->cpr[0][COP0_Context] & 0xff800000) | ((address >> 9) & 0x007ffff0);
		m_core->cpr[0][COP0_EntryHi] = (address & 0xffffe000) | (m_core->cpr[0][COP0_EntryHi] & 0xff);
	}
	generate_exception(exception, 1);
}


void mips3_device::invalid_instruction(uint32_t op)
{
	generate_exception(EXCEPTION_INVALIDOP, 1);
}



/***************************************************************************
    IRQ HANDLING
***************************************************************************/

void mips3_device::check_irqs()
{
	if ((CAUSE & SR & 0xfc00) && (SR & SR_IE) && !(SR & SR_EXL) && !(SR & SR_ERL))
		generate_exception(EXCEPTION_INTERRUPT, 0);
}



/***************************************************************************
    CORE CALLBACKS
***************************************************************************/

void mips3_device::device_start()
{
	m_isdrc = allow_drc();

	/* allocate the implementation-specific state from the full cache */
	m_core = (internal_mips3_state *)m_cache.alloc_near(sizeof(internal_mips3_state));

	/* initialize based on the config */
	memset(m_core, 0, sizeof(internal_mips3_state));

	m_cpu_clock = clock();
	m_program = &space(AS_PROGRAM);
	m_direct = &m_program->direct();

	/* set up the endianness */
	m_program->accessors(m_memory);

	/* allocate a timer for the compare interrupt */
	m_compare_int_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(mips3_device::compare_int_callback), this));

	uint32_t flags = 0;
	/* initialize the UML generator */
	m_drcuml = std::make_unique<drcuml_state>(*this, m_cache, flags, 8, 32, 2);

	/* add symbols for our stuff */
	m_drcuml->symbol_add(&m_core->pc, sizeof(m_core->pc), "pc");
	m_drcuml->symbol_add(&m_core->icount, sizeof(m_core->icount), "icount");
	for (int regnum = 0; regnum < 32; regnum++)
	{
		char buf[10];
		sprintf(buf, "r%d", regnum);
		m_drcuml->symbol_add(&m_core->r[regnum], sizeof(m_core->r[regnum]), buf);
		sprintf(buf, "f%d", regnum);
		m_drcuml->symbol_add(&m_core->cpr[1][regnum], sizeof(m_core->cpr[1][regnum]), buf);
	}
	m_drcuml->symbol_add(&m_core->r[REG_LO], sizeof(m_core->r[REG_LO]), "lo");
	m_drcuml->symbol_add(&m_core->r[REG_HI], sizeof(m_core->r[REG_LO]), "hi");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_Index], sizeof(m_core->cpr[0][COP0_Index]), "Index");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_Random], sizeof(m_core->cpr[0][COP0_Random]), "Random");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_EntryLo0], sizeof(m_core->cpr[0][COP0_EntryLo0]), "EntryLo0");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_EntryLo1], sizeof(m_core->cpr[0][COP0_EntryLo1]), "EntryLo1");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_Context], sizeof(m_core->cpr[0][COP0_Context]), "Context");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_PageMask], sizeof(m_core->cpr[0][COP0_PageMask]), "PageMask");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_Wired], sizeof(m_core->cpr[0][COP0_Wired]), "Wired");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_BadVAddr], sizeof(m_core->cpr[0][COP0_BadVAddr]), "BadVAddr");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_Count], sizeof(m_core->cpr[0][COP0_Count]), "Count");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_EntryHi], sizeof(m_core->cpr[0][COP0_EntryHi]), "EntryHi");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_Compare], sizeof(m_core->cpr[0][COP0_Compare]), "Compare");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_Status], sizeof(m_core->cpr[0][COP0_Status]), "Status");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_Cause], sizeof(m_core->cpr[0][COP0_Cause]), "Cause");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_EPC], sizeof(m_core->cpr[0][COP0_EPC]), "EPC");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_PRId], sizeof(m_core->cpr[0][COP0_PRId]), "PRId");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_Config], sizeof(m_core->cpr[0][COP0_Config]), "Config");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_LLAddr], sizeof(m_core->cpr[0][COP0_LLAddr]), "LLAddr");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_XContext], sizeof(m_core->cpr[0][COP0_XContext]), "XContext");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_ECC], sizeof(m_core->cpr[0][COP0_ECC]), "ECC");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_CacheErr], sizeof(m_core->cpr[0][COP0_CacheErr]), "CacheErr");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_TagLo], sizeof(m_core->cpr[0][COP0_TagLo]), "TagLo");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_TagHi], sizeof(m_core->cpr[0][COP0_TagHi]), "TagHi");
	m_drcuml->symbol_add(&m_core->cpr[0][COP0_ErrorPC], sizeof(m_core->cpr[0][COP0_ErrorPC]), "ErrorPC");
	m_drcuml->symbol_add(&m_core->ccr[1][31], sizeof(m_core->cpr[1][31]), "fcr31");
	m_drcuml->symbol_add(&m_core->mode, sizeof(m_core->mode), "mode");
	m_drcuml->symbol_add(&m_core->arg0, sizeof(m_core->arg0), "arg0");
	m_drcuml->symbol_add(&m_core->arg1, sizeof(m_core->arg1), "arg1");
	m_drcuml->symbol_add(&m_core->numcycles, sizeof(m_core->numcycles), "numcycles");
	m_drcuml->symbol_add(&m_fpmode, sizeof(m_fpmode), "fpmode");

	/* initialize the front-end helper */
	m_drcfe = std::make_unique<mips3_frontend>(this, COMPILE_BACKWARDS_BYTES, COMPILE_FORWARDS_BYTES, SINGLE_INSTRUCTION_MODE ? 1 : COMPILE_MAX_SEQUENCE);

	/* allocate memory for cache-local state and initialize it */
	memcpy(m_fpmode, fpmode_source, sizeof(fpmode_source));

	/* compute the register parameters */
	for (int regnum = 0; regnum < 34; regnum++)
	{
		m_regmap[regnum] = (regnum == 0) ? uml::parameter(0) : uml::parameter::make_memory(&m_core->r[regnum]);
		m_regmaplo[regnum] = (regnum == 0) ? uml::parameter(0) : uml::parameter::make_memory(LOPTR(&m_core->r[regnum]));
	}

	/* if we have registers to spare, assign r2, r3, r4 to leftovers */
	if (!DISABLE_FAST_REGISTERS)
	{
		drcbe_info beinfo;

		m_drcuml->get_backend_info(beinfo);
		if (beinfo.direct_iregs > 4)
		{
			m_regmap[2] = uml::I4;
			m_regmaplo[2] = uml::I4;
		}
		if (beinfo.direct_iregs > 5)
		{
			m_regmap[3] = uml::I5;
			m_regmaplo[3] = uml::I5;
		}
		if (beinfo.direct_iregs > 6)
		{
			m_regmap[4] = uml::I6;
			m_regmaplo[4] = uml::I6;
		}
	}

	/* mark the cache dirty so it is updated on next execute */
	m_cache_dirty = true;


	/* register for save states */
	save_item(NAME(m_core->pc));
	save_item(NAME(m_core->r));
	save_item(NAME(m_core->cpr));
	save_item(NAME(m_core->ccr));
	save_item(NAME(m_core->llbit));
	save_item(NAME(m_core->count_zero_time));
	for (int tlbindex = 0; tlbindex < m_tlbentries; tlbindex++)
	{
		save_item(NAME(m_tlb[tlbindex].page_mask), tlbindex);
		save_item(NAME(m_tlb[tlbindex].entry_hi), tlbindex);
		save_item(NAME(m_tlb[tlbindex].entry_lo), tlbindex);
	}

	// Register state with debugger
	state_add( MIPS3_PC,           "PC", m_core->pc).formatstr("%08X");

#if USE_ABI_REG_NAMES
	state_add( MIPS3_R0,           "zero", m_core->r[0]).callimport().formatstr("%016X");   // Can't change R0
	state_add( MIPS3_R1,           "at", m_core->r[1]).formatstr("%016X");
	state_add( MIPS3_R2,           "v0", m_core->r[2]).formatstr("%016X");
	state_add( MIPS3_R3,           "v1", m_core->r[3]).formatstr("%016X");
	state_add( MIPS3_R4,           "a0", m_core->r[4]).formatstr("%016X");
	state_add( MIPS3_R5,           "a1", m_core->r[5]).formatstr("%016X");
	state_add( MIPS3_R6,           "a2", m_core->r[6]).formatstr("%016X");
	state_add( MIPS3_R7,           "a3", m_core->r[7]).formatstr("%016X");
	state_add( MIPS3_R8,           "t0", m_core->r[8]).formatstr("%016X");
	state_add( MIPS3_R9,           "t1", m_core->r[9]).formatstr("%016X");
	state_add( MIPS3_R10,          "t2", m_core->r[10]).formatstr("%016X");
	state_add( MIPS3_R11,          "t3", m_core->r[11]).formatstr("%016X");
	state_add( MIPS3_R12,          "t4", m_core->r[12]).formatstr("%016X");
	state_add( MIPS3_R13,          "t5", m_core->r[13]).formatstr("%016X");
	state_add( MIPS3_R14,          "t6", m_core->r[14]).formatstr("%016X");
	state_add( MIPS3_R15,          "t7", m_core->r[15]).formatstr("%016X");
	state_add( MIPS3_R16,          "s0", m_core->r[16]).formatstr("%016X");
	state_add( MIPS3_R17,          "s1", m_core->r[17]).formatstr("%016X");
	state_add( MIPS3_R18,          "s2", m_core->r[18]).formatstr("%016X");
	state_add( MIPS3_R19,          "s3", m_core->r[19]).formatstr("%016X");
	state_add( MIPS3_R20,          "s4", m_core->r[20]).formatstr("%016X");
	state_add( MIPS3_R21,          "s5", m_core->r[21]).formatstr("%016X");
	state_add( MIPS3_R22,          "s6", m_core->r[22]).formatstr("%016X");
	state_add( MIPS3_R23,          "s7", m_core->r[23]).formatstr("%016X");
	state_add( MIPS3_R24,          "t8", m_core->r[24]).formatstr("%016X");
	state_add( MIPS3_R25,          "t9", m_core->r[25]).formatstr("%016X");
	state_add( MIPS3_R26,          "k0", m_core->r[26]).formatstr("%016X");
	state_add( MIPS3_R27,          "k1", m_core->r[27]).formatstr("%016X");
	state_add( MIPS3_R28,          "gp", m_core->r[28]).formatstr("%016X");
	state_add( MIPS3_R29,          "sp", m_core->r[29]).formatstr("%016X");
	state_add( MIPS3_R30,          "fp", m_core->r[30]).formatstr("%016X");
	state_add( MIPS3_R31,          "ra", m_core->r[31]).formatstr("%016X");
#else
	state_add( MIPS3_R0,           "R0", m_core->r[0]).callimport().formatstr("%016X");   // Can't change R0
	state_add( MIPS3_R1,           "R1", m_core->r[1]).formatstr("%016X");
	state_add( MIPS3_R2,           "R2", m_core->r[2]).formatstr("%016X");
	state_add( MIPS3_R3,           "R3", m_core->r[3]).formatstr("%016X");
	state_add( MIPS3_R4,           "R4", m_core->r[4]).formatstr("%016X");
	state_add( MIPS3_R5,           "R5", m_core->r[5]).formatstr("%016X");
	state_add( MIPS3_R6,           "R6", m_core->r[6]).formatstr("%016X");
	state_add( MIPS3_R7,           "R7", m_core->r[7]).formatstr("%016X");
	state_add( MIPS3_R8,           "R8", m_core->r[8]).formatstr("%016X");
	state_add( MIPS3_R9,           "R9", m_core->r[9]).formatstr("%016X");
	state_add( MIPS3_R10,          "R10", m_core->r[10]).formatstr("%016X");
	state_add( MIPS3_R11,          "R11", m_core->r[11]).formatstr("%016X");
	state_add( MIPS3_R12,          "R12", m_core->r[12]).formatstr("%016X");
	state_add( MIPS3_R13,          "R13", m_core->r[13]).formatstr("%016X");
	state_add( MIPS3_R14,          "R14", m_core->r[14]).formatstr("%016X");
	state_add( MIPS3_R15,          "R15", m_core->r[15]).formatstr("%016X");
	state_add( MIPS3_R16,          "R16", m_core->r[16]).formatstr("%016X");
	state_add( MIPS3_R17,          "R17", m_core->r[17]).formatstr("%016X");
	state_add( MIPS3_R18,          "R18", m_core->r[18]).formatstr("%016X");
	state_add( MIPS3_R19,          "R19", m_core->r[19]).formatstr("%016X");
	state_add( MIPS3_R20,          "R20", m_core->r[20]).formatstr("%016X");
	state_add( MIPS3_R21,          "R21", m_core->r[21]).formatstr("%016X");
	state_add( MIPS3_R22,          "R22", m_core->r[22]).formatstr("%016X");
	state_add( MIPS3_R23,          "R23", m_core->r[23]).formatstr("%016X");
	state_add( MIPS3_R24,          "R24", m_core->r[24]).formatstr("%016X");
	state_add( MIPS3_R25,          "R25", m_core->r[25]).formatstr("%016X");
	state_add( MIPS3_R26,          "R26", m_core->r[26]).formatstr("%016X");
	state_add( MIPS3_R27,          "R27", m_core->r[27]).formatstr("%016X");
	state_add( MIPS3_R28,          "R28", m_core->r[28]).formatstr("%016X");
	state_add( MIPS3_R29,          "R29", m_core->r[29]).formatstr("%016X");
	state_add( MIPS3_R30,          "R30", m_core->r[30]).formatstr("%016X");
	state_add( MIPS3_R31,          "R31", m_core->r[31]).formatstr("%016X");
#endif
	state_add( MIPS3_HI,           "HI", m_core->r[REG_HI]).formatstr("%016X");
	state_add( MIPS3_LO,           "LO", m_core->r[REG_LO]).formatstr("%016X");

	state_add( MIPS3_CCR1_31,      "CCR31", m_core->ccr[1][31]).formatstr("%08X");

	state_add( MIPS3_FPR0,         "FPR0", m_core->cpr[1][0]).formatstr("%016X");
	state_add( MIPS3_FPS0,         "FPS0", m_core->cpr[1][0]).formatstr("%17s");
	state_add( MIPS3_FPD0,         "FPD0", m_core->cpr[1][0]).formatstr("%17s");
	state_add( MIPS3_FPR1,         "FPR1", m_core->cpr[1][1]).formatstr("%016X");
	state_add( MIPS3_FPS1,         "FPS1", m_core->cpr[1][1]).formatstr("%17s");
	state_add( MIPS3_FPD1,         "FPD1", m_core->cpr[1][1]).formatstr("%17s");
	state_add( MIPS3_FPR2,         "FPR2", m_core->cpr[1][2]).formatstr("%016X");
	state_add( MIPS3_FPS2,         "FPS2", m_core->cpr[1][2]).formatstr("%17s");
	state_add( MIPS3_FPD2,         "FPD2", m_core->cpr[1][2]).formatstr("%17s");
	state_add( MIPS3_FPR3,         "FPR3", m_core->cpr[1][3]).formatstr("%016X");
	state_add( MIPS3_FPS3,         "FPS3", m_core->cpr[1][3]).formatstr("%17s");
	state_add( MIPS3_FPD3,         "FPD3", m_core->cpr[1][3]).formatstr("%17s");
	state_add( MIPS3_FPR4,         "FPR4", m_core->cpr[1][4]).formatstr("%016X");
	state_add( MIPS3_FPS4,         "FPS4", m_core->cpr[1][4]).formatstr("%17s");
	state_add( MIPS3_FPD4,         "FPD4", m_core->cpr[1][4]).formatstr("%17s");
	state_add( MIPS3_FPR5,         "FPR5", m_core->cpr[1][5]).formatstr("%016X");
	state_add( MIPS3_FPS5,         "FPS5", m_core->cpr[1][5]).formatstr("%17s");
	state_add( MIPS3_FPD5,         "FPD5", m_core->cpr[1][5]).formatstr("%17s");
	state_add( MIPS3_FPR6,         "FPR6", m_core->cpr[1][6]).formatstr("%016X");
	state_add( MIPS3_FPS6,         "FPS6", m_core->cpr[1][6]).formatstr("%17s");
	state_add( MIPS3_FPD6,         "FPD6", m_core->cpr[1][6]).formatstr("%17s");
	state_add( MIPS3_FPR7,         "FPR7", m_core->cpr[1][7]).formatstr("%016X");
	state_add( MIPS3_FPS7,         "FPS7", m_core->cpr[1][7]).formatstr("%17s");
	state_add( MIPS3_FPD7,         "FPD7", m_core->cpr[1][7]).formatstr("%17s");
	state_add( MIPS3_FPR8,         "FPR8", m_core->cpr[1][8]).formatstr("%016X");
	state_add( MIPS3_FPS8,         "FPS8", m_core->cpr[1][8]).formatstr("%17s");
	state_add( MIPS3_FPD8,         "FPD8", m_core->cpr[1][8]).formatstr("%17s");
	state_add( MIPS3_FPR9,         "FPR9", m_core->cpr[1][9]).formatstr("%016X");
	state_add( MIPS3_FPS9,         "FPS9", m_core->cpr[1][9]).formatstr("%17s");
	state_add( MIPS3_FPD9,         "FPD9", m_core->cpr[1][9]).formatstr("%17s");
	state_add( MIPS3_FPR10,        "FPR10", m_core->cpr[1][10]).formatstr("%016X");
	state_add( MIPS3_FPS10,        "FPS10", m_core->cpr[1][10]).formatstr("%17s");
	state_add( MIPS3_FPD10,        "FPD10", m_core->cpr[1][10]).formatstr("%17s");
	state_add( MIPS3_FPR11,        "FPR11", m_core->cpr[1][11]).formatstr("%016X");
	state_add( MIPS3_FPS11,        "FPS11", m_core->cpr[1][11]).formatstr("%17s");
	state_add( MIPS3_FPD11,        "FPD11", m_core->cpr[1][11]).formatstr("%17s");
	state_add( MIPS3_FPR12,        "FPR12", m_core->cpr[1][12]).formatstr("%016X");
	state_add( MIPS3_FPS12,        "FPS12", m_core->cpr[1][12]).formatstr("%17s");
	state_add( MIPS3_FPD12,        "FPD12", m_core->cpr[1][12]).formatstr("%17s");
	state_add( MIPS3_FPR13,        "FPR13", m_core->cpr[1][13]).formatstr("%016X");
	state_add( MIPS3_FPS13,        "FPS13", m_core->cpr[1][13]).formatstr("%17s");
	state_add( MIPS3_FPD13,        "FPD13", m_core->cpr[1][13]).formatstr("%17s");
	state_add( MIPS3_FPR14,        "FPR14", m_core->cpr[1][14]).formatstr("%016X");
	state_add( MIPS3_FPS14,        "FPS14", m_core->cpr[1][14]).formatstr("%17s");
	state_add( MIPS3_FPD14,        "FPD14", m_core->cpr[1][14]).formatstr("%17s");
	state_add( MIPS3_FPR15,        "FPR15", m_core->cpr[1][15]).formatstr("%016X");
	state_add( MIPS3_FPS15,        "FPS15", m_core->cpr[1][15]).formatstr("%17s");
	state_add( MIPS3_FPD15,        "FPD15", m_core->cpr[1][15]).formatstr("%17s");
	state_add( MIPS3_FPR16,        "FPR16", m_core->cpr[1][16]).formatstr("%016X");
	state_add( MIPS3_FPS16,        "FPS16", m_core->cpr[1][16]).formatstr("%17s");
	state_add( MIPS3_FPD16,        "FPD16", m_core->cpr[1][16]).formatstr("%17s");
	state_add( MIPS3_FPR17,        "FPR17", m_core->cpr[1][17]).formatstr("%016X");
	state_add( MIPS3_FPS17,        "FPS17", m_core->cpr[1][17]).formatstr("%17s");
	state_add( MIPS3_FPD17,        "FPD17", m_core->cpr[1][17]).formatstr("%17s");
	state_add( MIPS3_FPR18,        "FPR18", m_core->cpr[1][18]).formatstr("%016X");
	state_add( MIPS3_FPS18,        "FPS18", m_core->cpr[1][18]).formatstr("%17s");
	state_add( MIPS3_FPD18,        "FPD18", m_core->cpr[1][18]).formatstr("%17s");
	state_add( MIPS3_FPR19,        "FPR19", m_core->cpr[1][19]).formatstr("%016X");
	state_add( MIPS3_FPS19,        "FPS19", m_core->cpr[1][19]).formatstr("%17s");
	state_add( MIPS3_FPD19,        "FPD19", m_core->cpr[1][19]).formatstr("%17s");
	state_add( MIPS3_FPR20,        "FPR20", m_core->cpr[1][20]).formatstr("%016X");
	state_add( MIPS3_FPS20,        "FPS20", m_core->cpr[1][20]).formatstr("%17s");
	state_add( MIPS3_FPD20,        "FPD20", m_core->cpr[1][20]).formatstr("%17s");
	state_add( MIPS3_FPR21,        "FPR21", m_core->cpr[1][21]).formatstr("%016X");
	state_add( MIPS3_FPS21,        "FPS21", m_core->cpr[1][21]).formatstr("%17s");
	state_add( MIPS3_FPD21,        "FPD21", m_core->cpr[1][21]).formatstr("%17s");
	state_add( MIPS3_FPR22,        "FPR22", m_core->cpr[1][22]).formatstr("%016X");
	state_add( MIPS3_FPS22,        "FPS22", m_core->cpr[1][22]).formatstr("%17s");
	state_add( MIPS3_FPD22,        "FPD22", m_core->cpr[1][22]).formatstr("%17s");
	state_add( MIPS3_FPR23,        "FPR23", m_core->cpr[1][23]).formatstr("%016X");
	state_add( MIPS3_FPS23,        "FPS23", m_core->cpr[1][23]).formatstr("%17s");
	state_add( MIPS3_FPD23,        "FPD23", m_core->cpr[1][23]).formatstr("%17s");
	state_add( MIPS3_FPR24,        "FPR24", m_core->cpr[1][24]).formatstr("%016X");
	state_add( MIPS3_FPS24,        "FPS24", m_core->cpr[1][24]).formatstr("%17s");
	state_add( MIPS3_FPD24,        "FPD24", m_core->cpr[1][24]).formatstr("%17s");
	state_add( MIPS3_FPR25,        "FPR25", m_core->cpr[1][25]).formatstr("%016X");
	state_add( MIPS3_FPS25,        "FPS25", m_core->cpr[1][25]).formatstr("%17s");
	state_add( MIPS3_FPD25,        "FPD25", m_core->cpr[1][25]).formatstr("%17s");
	state_add( MIPS3_FPR26,        "FPR26", m_core->cpr[1][26]).formatstr("%016X");
	state_add( MIPS3_FPS26,        "FPS26", m_core->cpr[1][26]).formatstr("%17s");
	state_add( MIPS3_FPD26,        "FPD26", m_core->cpr[1][26]).formatstr("%17s");
	state_add( MIPS3_FPR27,        "FPR27", m_core->cpr[1][27]).formatstr("%016X");
	state_add( MIPS3_FPS27,        "FPS27", m_core->cpr[1][27]).formatstr("%17s");
	state_add( MIPS3_FPD27,        "FPD27", m_core->cpr[1][27]).formatstr("%17s");
	state_add( MIPS3_FPR28,        "FPR28", m_core->cpr[1][28]).formatstr("%016X");
	state_add( MIPS3_FPS28,        "FPS28", m_core->cpr[1][28]).formatstr("%17s");
	state_add( MIPS3_FPD28,        "FPD28", m_core->cpr[1][28]).formatstr("%17s");
	state_add( MIPS3_FPR29,        "FPR29", m_core->cpr[1][29]).formatstr("%016X");
	state_add( MIPS3_FPS29,        "FPS29", m_core->cpr[1][29]).formatstr("%17s");
	state_add( MIPS3_FPD29,        "FPD29", m_core->cpr[1][29]).formatstr("%17s");
	state_add( MIPS3_FPR30,        "FPR30", m_core->cpr[1][30]).formatstr("%016X");
	state_add( MIPS3_FPS30,        "FPS30", m_core->cpr[1][30]).formatstr("%17s");
	state_add( MIPS3_FPD30,        "FPD30", m_core->cpr[1][30]).formatstr("%17s");
	state_add( MIPS3_FPR31,        "FPR31", m_core->cpr[1][31]).formatstr("%016X");
	state_add( MIPS3_FPS31,        "FPS31", m_core->cpr[1][31]).formatstr("%17s");
	state_add( MIPS3_FPD31,        "FPD31", m_core->cpr[1][31]).formatstr("%17s");

	state_add( MIPS3_SR,           "SR", m_core->cpr[0][COP0_Status]).formatstr("%08X");
	state_add( MIPS3_EPC,          "EPC", m_core->cpr[0][COP0_EPC]).formatstr("%08X");
	state_add( MIPS3_CAUSE,        "Cause", m_core->cpr[0][COP0_Cause]).formatstr("%08X");
	state_add( MIPS3_COUNT,        "Count", m_debugger_temp).callexport().formatstr("%08X");
	state_add( MIPS3_COMPARE,      "Compare", m_core->cpr[0][COP0_Compare]).formatstr("%08X");
	state_add( MIPS3_INDEX,        "Index", m_core->cpr[0][COP0_Index]).formatstr("%08X");
	state_add( MIPS3_RANDOM,       "Random", m_core->cpr[0][COP0_Random]).formatstr("%08X");
	state_add( MIPS3_ENTRYHI,      "EntryHi", m_core->cpr[0][COP0_EntryHi]).formatstr("%016X");
	state_add( MIPS3_ENTRYLO0,     "EntryLo0", m_core->cpr[0][COP0_EntryLo0]).formatstr("%016X");
	state_add( MIPS3_ENTRYLO1,     "EntryLo1", m_core->cpr[0][COP0_EntryLo1]).formatstr("%016X");
	state_add( MIPS3_PAGEMASK,     "PageMask", m_core->cpr[0][COP0_PageMask]).formatstr("%016X");
	state_add( MIPS3_WIRED,        "Wired", m_core->cpr[0][COP0_Wired]).formatstr("%08X");
	state_add( MIPS3_BADVADDR,     "BadVAddr", m_core->cpr[0][COP0_BadVAddr]).formatstr("%08X");

	state_add( STATE_GENPCBASE, "CURPC", m_core->pc).noshow();
	state_add( STATE_GENSP, "CURSP", m_core->r[31]).noshow();
	state_add( STATE_GENFLAGS, "CURFLAGS", m_debugger_temp).formatstr("%1s").noshow();

	m_icountptr = &m_core->icount;
}


void mips3_device::state_export(const device_state_entry &entry)
{
	switch (entry.index())
	{
		case MIPS3_COUNT:
			m_debugger_temp = (total_cycles() - m_core->count_zero_time) / 2;
			break;
	}
}


void mips3_device::state_string_export(const device_state_entry &entry, std::string &str) const
{
	switch (entry.index())
	{
		case MIPS3_FPS0:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][0]);
			break;

		case MIPS3_FPD0:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][0]);
			break;

		case MIPS3_FPS1:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][1]);
			break;

		case MIPS3_FPD1:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][1]);
			break;

		case MIPS3_FPS2:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][2]);
			break;

		case MIPS3_FPD2:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][2]);
			break;

		case MIPS3_FPS3:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][3]);
			break;

		case MIPS3_FPD3:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][3]);
			break;

		case MIPS3_FPS4:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][4]);
			break;

		case MIPS3_FPD4:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][4]);
			break;

		case MIPS3_FPS5:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][5]);
			break;

		case MIPS3_FPD5:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][5]);
			break;

		case MIPS3_FPS6:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][6]);
			break;

		case MIPS3_FPD6:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][6]);
			break;

		case MIPS3_FPS7:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][7]);
			break;

		case MIPS3_FPD7:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][7]);
			break;

		case MIPS3_FPS8:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][8]);
			break;

		case MIPS3_FPD8:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][8]);
			break;

		case MIPS3_FPS9:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][9]);
			break;

		case MIPS3_FPD9:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][9]);
			break;

		case MIPS3_FPS10:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][10]);
			break;

		case MIPS3_FPD10:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][10]);
			break;

		case MIPS3_FPS11:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][11]);
			break;

		case MIPS3_FPD11:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][11]);
			break;

		case MIPS3_FPS12:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][12]);
			break;

		case MIPS3_FPD12:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][12]);
			break;

		case MIPS3_FPS13:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][13]);
			break;

		case MIPS3_FPD13:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][13]);
			break;

		case MIPS3_FPS14:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][14]);
			break;

		case MIPS3_FPD14:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][14]);
			break;

		case MIPS3_FPS15:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][15]);
			break;

		case MIPS3_FPD15:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][15]);
			break;

		case MIPS3_FPS16:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][16]);
			break;

		case MIPS3_FPD16:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][16]);
			break;

		case MIPS3_FPS17:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][17]);
			break;

		case MIPS3_FPD17:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][17]);
			break;

		case MIPS3_FPS18:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][18]);
			break;

		case MIPS3_FPD18:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][18]);
			break;

		case MIPS3_FPS19:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][19]);
			break;

		case MIPS3_FPD19:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][19]);
			break;

		case MIPS3_FPS20:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][20]);
			break;

		case MIPS3_FPD20:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][20]);
			break;

		case MIPS3_FPS21:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][21]);
			break;

		case MIPS3_FPD21:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][21]);
			break;

		case MIPS3_FPS22:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][22]);
			break;

		case MIPS3_FPD22:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][22]);
			break;

		case MIPS3_FPS23:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][23]);
			break;

		case MIPS3_FPD23:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][23]);
			break;

		case MIPS3_FPS24:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][24]);
			break;

		case MIPS3_FPD24:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][24]);
			break;

		case MIPS3_FPS25:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][25]);
			break;

		case MIPS3_FPD25:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][25]);
			break;

		case MIPS3_FPS26:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][26]);
			break;

		case MIPS3_FPD26:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][26]);
			break;

		case MIPS3_FPS27:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][27]);
			break;

		case MIPS3_FPD27:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][27]);
			break;

		case MIPS3_FPS28:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][28]);
			break;

		case MIPS3_FPD28:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][28]);
			break;

		case MIPS3_FPS29:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][29]);
			break;

		case MIPS3_FPD29:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][29]);
			break;

		case MIPS3_FPS30:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][30]);
			break;

		case MIPS3_FPD30:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][30]);
			break;

		case MIPS3_FPS31:
			str = string_format("!%16g", *(float *)&m_core->cpr[1][31]);
			break;

		case MIPS3_FPD31:
			str = string_format("!%16g", *(double *)&m_core->cpr[1][31]);
			break;

		case STATE_GENFLAGS:
			str = " ";
			break;
	}
}


void mips3_device::device_reset()
{
	/* common reset */
	m_nextpc = ~0;
	memset(m_cf, 0, sizeof(m_cf));

	/* initialize the state */
	m_core->pc = 0xbfc00000;
	m_core->cpr[0][COP0_Status] = SR_BEV | SR_ERL;
	m_core->cpr[0][COP0_Wired] = 0;
	m_core->cpr[0][COP0_Compare] = 0xffffffff;
	m_core->cpr[0][COP0_Count] = 0;
	m_core->cpr[0][COP0_Config] = compute_config_register();
	m_core->cpr[0][COP0_PRId] = compute_prid_register();
	m_core->count_zero_time = total_cycles();

	/* initialize the TLB state */
	for (int tlbindex = 0; tlbindex < m_tlbentries; tlbindex++)
	{
		mips3_tlb_entry *entry = &m_tlb[tlbindex];
		entry->page_mask = 0;
		entry->entry_hi = 0xffffffff;
		entry->entry_lo[0] = 0xfffffff8;
		entry->entry_lo[1] = 0xfffffff8;
		vtlb_load(2 * tlbindex + 0, 0, 0, 0);
		vtlb_load(2 * tlbindex + 1, 0, 0, 0);
	}

	/* load the fixed TLB range */
	vtlb_load(2 * m_tlbentries + 0, (0xa0000000 - 0x80000000) >> MIPS3_MIN_PAGE_SHIFT, 0x80000000, 0x00000000 | VTLB_READ_ALLOWED | VTLB_WRITE_ALLOWED | VTLB_FETCH_ALLOWED | VTLB_FLAG_VALID);
	vtlb_load(2 * m_tlbentries + 1, (0xc0000000 - 0xa0000000) >> MIPS3_MIN_PAGE_SHIFT, 0xa0000000, 0x00000000 | VTLB_READ_ALLOWED | VTLB_WRITE_ALLOWED | VTLB_FETCH_ALLOWED | VTLB_FLAG_VALID);

	m_core->mode = (MODE_KERNEL << 1) | 0;
	m_cache_dirty = true;
	m_interrupt_cycles = 0;
}


bool mips3_device::memory_translate(int spacenum, int intention, offs_t &address)
{
	/* only applies to the program address space */
	if (spacenum == AS_PROGRAM)
	{
		const vtlb_entry *table = vtlb_table();
		vtlb_entry entry = table[address >> MIPS3_MIN_PAGE_SHIFT];
		if ((entry & (1 << (intention & (TRANSLATE_TYPE_MASK | TRANSLATE_USER_MASK)))) == 0)
			return false;
		address = (entry & ~MIPS3_MIN_PAGE_MASK) | (address & MIPS3_MIN_PAGE_MASK);
	}
	return true;
}


offs_t mips3_device::disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options)
{
	uint32_t op = *(uint32_t *)oprom;
	if (m_bigendian)
		op = big_endianize_int32(op);
	else
		op = little_endianize_int32(op);
	return dasmmips3(stream, pc, op);
}



/***************************************************************************
    TLB HANDLING
***************************************************************************/

inline bool mips3_device::RBYTE(offs_t address, uint32_t *result)
{
	if ((m_flavor == MIPS3_TYPE_TX4925) && ((address & 0xffff0000) == 0xff1f0000))
	{
		*result = (*m_memory.read_byte)(*m_program, address);
		return true;
	}

	const uint32_t tlbval = vtlb_table()[address >> 12];
	if (tlbval & VTLB_READ_ALLOWED)
	{
		const uint32_t tlbaddress = (tlbval & ~0xfff) | (address & 0xfff);
		for (int ramnum = 0; ramnum < m_fastram_select; ramnum++)
		{
			if (tlbaddress < m_fastram[ramnum].start || tlbaddress > m_fastram[ramnum].end)
			{
				continue;
			}
			*result = m_fastram[ramnum].offset_base8[tlbaddress ^ m_byte_xor];
			return true;
		}
		*result = (*m_memory.read_byte)(*m_program, tlbaddress);
	}
	else
	{
		if(tlbval & VTLB_FLAG_FIXED)
		{
			generate_tlb_exception(EXCEPTION_TLBLOAD, address);
		}
		else
		{
			generate_tlb_exception(EXCEPTION_TLBLOAD_FILL, address);
		}
		*result = 0;
		return false;
	}
	return true;
}

inline bool mips3_device::RHALF(offs_t address, uint32_t *result)
{
	if ((m_flavor == MIPS3_TYPE_TX4925) && ((address & 0xffff0000) == 0xff1f0000))
	{
		*result = (*m_memory.read_word)(*m_program, address);
		return true;
	}

	const uint32_t tlbval = vtlb_table()[address >> 12];
	if (tlbval & VTLB_READ_ALLOWED)
	{
		const uint32_t tlbaddress = (tlbval & ~0xfff) | (address & 0xfff);
		for (int ramnum = 0; ramnum < m_fastram_select; ramnum++)
		{
			if (tlbaddress < m_fastram[ramnum].start || tlbaddress > m_fastram[ramnum].end)
			{
				continue;
			}
			*result = m_fastram[ramnum].offset_base16[(tlbaddress ^ m_word_xor) >> 1];
			return true;
		}
		*result = (*m_memory.read_word)(*m_program, tlbaddress);
	}
	else
	{
		if(tlbval & VTLB_FLAG_FIXED)
		{
			generate_tlb_exception(EXCEPTION_TLBLOAD, address);
		}
		else
		{
			generate_tlb_exception(EXCEPTION_TLBLOAD_FILL, address);
		}
		*result = 0;
		return false;
	}
	return true;
}

inline bool mips3_device::RWORD(offs_t address, uint32_t *result)
{
	if ((m_flavor == MIPS3_TYPE_TX4925) && ((address & 0xffff0000) == 0xff1f0000))
	{
		*result = (*m_memory.read_dword)(*m_program, address);
		return true;
	}

	const uint32_t tlbval = vtlb_table()[address >> 12];
	if (tlbval & VTLB_READ_ALLOWED)
	{
		const uint32_t tlbaddress = (tlbval & ~0xfff) | (address & 0xfff);
		for (int ramnum = 0; ramnum < m_fastram_select; ramnum++)
		{
			if (tlbaddress < m_fastram[ramnum].start || tlbaddress > m_fastram[ramnum].end)
			{
				continue;
			}
			*result = m_fastram[ramnum].offset_base32[tlbaddress >> 2];
			return true;
		}
		*result = (*m_memory.read_dword)(*m_program, tlbaddress);
	}
	else
	{
		if(tlbval & VTLB_FLAG_FIXED)
		{
			generate_tlb_exception(EXCEPTION_TLBLOAD, address);
		}
		else
		{
			generate_tlb_exception(EXCEPTION_TLBLOAD_FILL, address);
		}
		*result = 0;
		return false;
	}
	return true;
}

inline bool mips3_device::RWORD_MASKED(offs_t address, uint32_t *result, uint32_t mem_mask)
{
	if ((m_flavor == MIPS3_TYPE_TX4925) && ((address & 0xffff0000) == 0xff1f0000))
	{
		*result = (*m_memory.read_dword_masked)(*m_program, address, mem_mask);
		return true;
	}

	const uint32_t tlbval = vtlb_table()[address >> 12];
	if (tlbval & VTLB_READ_ALLOWED)
	{
		*result = (*m_memory.read_dword_masked)(*m_program, (tlbval & ~0xfff) | (address & 0xfff), mem_mask);
	}
	else
	{
		if(tlbval & VTLB_FLAG_FIXED)
		{
			generate_tlb_exception(EXCEPTION_TLBLOAD, address);
		}
		else
		{
			generate_tlb_exception(EXCEPTION_TLBLOAD_FILL, address);
		}
		*result = 0;
		return false;
	}
	return true;
}

inline bool mips3_device::RDOUBLE(offs_t address, uint64_t *result)
{
	if ((m_flavor == MIPS3_TYPE_TX4925) && ((address & 0xffff0000) == 0xff1f0000))
	{
		*result = (*m_memory.read_qword)(*m_program, address);
		return true;
	}
	const uint32_t tlbval = vtlb_table()[address >> 12];
	if (tlbval & VTLB_READ_ALLOWED)
	{
		*result = (*m_memory.read_qword)(*m_program, (tlbval & ~0xfff) | (address & 0xfff));
	}
	else
	{
		if(tlbval & VTLB_FLAG_FIXED)
		{
			generate_tlb_exception(EXCEPTION_TLBLOAD, address);
		}
		else
		{
			generate_tlb_exception(EXCEPTION_TLBLOAD_FILL, address);
		}
		*result = 0;
		return false;
	}
	return true;
}

inline bool mips3_device::RDOUBLE_MASKED(offs_t address, uint64_t *result, uint64_t mem_mask)
{
	if ((m_flavor == MIPS3_TYPE_TX4925) && ((address & 0xffff0000) == 0xff1f0000))
	{
		*result = (*m_memory.read_qword_masked)(*m_program, address, mem_mask);
		return true;
	}

	const uint32_t tlbval = vtlb_table()[address >> 12];
	if (tlbval & VTLB_READ_ALLOWED)
	{
		*result = (*m_memory.read_qword_masked)(*m_program, (tlbval & ~0xfff) | (address & 0xfff), mem_mask);
	}
	else
	{
		if(tlbval & VTLB_FLAG_FIXED)
		{
			generate_tlb_exception(EXCEPTION_TLBLOAD, address);
		}
		else
		{
			generate_tlb_exception(EXCEPTION_TLBLOAD_FILL, address);
		}
		*result = 0;
		return false;
	}
	return true;
}

inline void mips3_device::WBYTE(offs_t address, uint8_t data)
{
	if ((m_flavor == MIPS3_TYPE_TX4925) && ((address & 0xffff0000) == 0xff1f0000))
	{
		(*m_memory.write_byte)(*m_program, address, data);
		return;
	}

	const uint32_t tlbval = vtlb_table()[address >> 12];
	if (tlbval & VTLB_WRITE_ALLOWED)
	{
		const uint32_t tlbaddress = (tlbval & ~0xfff) | (address & 0xfff);
		for (int ramnum = 0; ramnum < m_fastram_select; ramnum++)
		{
			if (m_fastram[ramnum].readonly == true || tlbaddress < m_fastram[ramnum].start || tlbaddress > m_fastram[ramnum].end)
			{
				continue;
			}
			m_fastram[ramnum].offset_base8[tlbaddress ^ m_byte_xor] = data;
			return;
		}
		(*m_memory.write_byte)(*m_program, tlbaddress, data);
	}
	else
	{
		if(tlbval & VTLB_READ_ALLOWED)
		{
			generate_tlb_exception(EXCEPTION_TLBMOD, address);
		}
		else if(tlbval & VTLB_FLAG_FIXED)
		{
			generate_tlb_exception(EXCEPTION_TLBSTORE, address);
		}
		else
		{
			generate_tlb_exception(EXCEPTION_TLBSTORE_FILL, address);
		}
	}
}

inline void mips3_device::WHALF(offs_t address, uint16_t data)
{
	if ((m_flavor == MIPS3_TYPE_TX4925) && ((address & 0xffff0000) == 0xff1f0000))
	{
		(*m_memory.write_word)(*m_program, address, data);
		return;
	}
	const uint32_t tlbval = vtlb_table()[address >> 12];
	if (tlbval & VTLB_WRITE_ALLOWED)
	{
		const uint32_t tlbaddress = (tlbval & ~0xfff) | (address & 0xfff);
		for (int ramnum = 0; ramnum < m_fastram_select; ramnum++)
		{
			if (m_fastram[ramnum].readonly == true || tlbaddress < m_fastram[ramnum].start || tlbaddress > m_fastram[ramnum].end)
			{
				continue;
			}
			m_fastram[ramnum].offset_base16[(tlbaddress ^ m_word_xor) >> 1] = data;
			return;
		}
		(*m_memory.write_word)(*m_program, tlbaddress, data);
	}
	else
	{
		if(tlbval & VTLB_READ_ALLOWED)
		{
			generate_tlb_exception(EXCEPTION_TLBMOD, address);
		}
		else if(tlbval & VTLB_FLAG_FIXED)
		{
			generate_tlb_exception(EXCEPTION_TLBSTORE, address);
		}
		else
		{
			generate_tlb_exception(EXCEPTION_TLBSTORE_FILL, address);
		}
	}
}

inline void mips3_device::WWORD(offs_t address, uint32_t data)
{
	if ((m_flavor == MIPS3_TYPE_TX4925) && ((address & 0xffff0000) == 0xff1f0000))
	{
		(*m_memory.write_dword)(*m_program, address, data);
		return;
	}

	const uint32_t tlbval = vtlb_table()[address >> 12];
	if (tlbval & VTLB_WRITE_ALLOWED)
	{
		const uint32_t tlbaddress = (tlbval & ~0xfff) | (address & 0xfff);
		for (int ramnum = 0; ramnum < m_fastram_select; ramnum++)
		{
			if (m_fastram[ramnum].readonly == true || tlbaddress < m_fastram[ramnum].start || tlbaddress > m_fastram[ramnum].end)
			{
				continue;
			}
			m_fastram[ramnum].offset_base32[tlbaddress >> 2] = data;
			return;
		}
		(*m_memory.write_dword)(*m_program, tlbaddress, data);
	}
	else
	{
		if(tlbval & VTLB_READ_ALLOWED)
		{
			generate_tlb_exception(EXCEPTION_TLBMOD, address);
		}
		else if(tlbval & VTLB_FLAG_FIXED)
		{
			generate_tlb_exception(EXCEPTION_TLBSTORE, address);
		}
		else
		{
			generate_tlb_exception(EXCEPTION_TLBSTORE_FILL, address);
		}
	}
}

inline void mips3_device::WWORD_MASKED(offs_t address, uint32_t data, uint32_t mem_mask)
{
	if ((m_flavor == MIPS3_TYPE_TX4925) && ((address & 0xffff0000) == 0xff1f0000))
	{
		(*m_memory.write_dword_masked)(*m_program, address, data, mem_mask);
		return;
	}

	const uint32_t tlbval = vtlb_table()[address >> 12];
	if (tlbval & VTLB_WRITE_ALLOWED)
	{
		(*m_memory.write_dword_masked)(*m_program, (tlbval & ~0xfff) | (address & 0xfff), data, mem_mask);
	}
	else
	{
		if(tlbval & VTLB_READ_ALLOWED)
		{
			generate_tlb_exception(EXCEPTION_TLBMOD, address);
		}
		else if(tlbval & VTLB_FLAG_FIXED)
		{
			generate_tlb_exception(EXCEPTION_TLBSTORE, address);
		}
		else
		{
			generate_tlb_exception(EXCEPTION_TLBSTORE_FILL, address);
		}
	}
}

inline void mips3_device::WDOUBLE(offs_t address, uint64_t data)
{
	if ((m_flavor == MIPS3_TYPE_TX4925) && ((address & 0xffff0000) == 0xff1f0000))
	{
		(*m_memory.write_qword)(*m_program, address, data);
		return;
	}

	const uint32_t tlbval = vtlb_table()[address >> 12];
	if (tlbval & VTLB_WRITE_ALLOWED)
	{
		(*m_memory.write_qword)(*m_program, (tlbval & ~0xfff) | (address & 0xfff), data);
	}
	else
	{
		if(tlbval & VTLB_READ_ALLOWED)
		{
			generate_tlb_exception(EXCEPTION_TLBMOD, address);
		}
		else if(tlbval & VTLB_FLAG_FIXED)
		{
			generate_tlb_exception(EXCEPTION_TLBSTORE, address);
		}
		else
		{
			generate_tlb_exception(EXCEPTION_TLBSTORE_FILL, address);
		}
	}
}

inline void mips3_device::WDOUBLE_MASKED(offs_t address, uint64_t data, uint64_t mem_mask)
{
	if ((m_flavor == MIPS3_TYPE_TX4925) && ((address & 0xffff0000) == 0xff1f0000))
	{
		(*m_memory.write_qword_masked)(*m_program, address, data, mem_mask);
		return;
	}

	const uint32_t tlbval = vtlb_table()[address >> 12];
	if (tlbval & VTLB_WRITE_ALLOWED)
	{
		(*m_memory.write_qword_masked)(*m_program, (tlbval & ~0xfff)  | (address & 0xfff), data, mem_mask);
	}
	else
	{
		if(tlbval & VTLB_READ_ALLOWED)
		{
			generate_tlb_exception(EXCEPTION_TLBMOD, address);
		}
		else if(tlbval & VTLB_FLAG_FIXED)
		{
			generate_tlb_exception(EXCEPTION_TLBSTORE, address);
		}
		else
		{
			generate_tlb_exception(EXCEPTION_TLBSTORE_FILL, address);
		}
	}
}



/***************************************************************************
    COP0 (SYSTEM) EXECUTION HANDLING
***************************************************************************/

uint64_t mips3_device::get_cop0_reg(int idx)
{
	if (idx == COP0_Count)
	{
		/* it doesn't really take 250 cycles to read this register, but it helps speed */
		/* up loops that hammer on it */
		if (m_core->icount >= MIPS3_COUNT_READ_CYCLES)
			m_core->icount -= MIPS3_COUNT_READ_CYCLES;
		else
			m_core->icount = 0;
		return (uint32_t)((total_cycles() - m_core->count_zero_time) / 2);
	}
	else if (idx == COP0_Cause)
	{
		/* it doesn't really take 250 cycles to read this register, but it helps speed */
		/* up loops that hammer on it */
		if (m_core->icount >= MIPS3_CAUSE_READ_CYCLES)
			m_core->icount -= MIPS3_CAUSE_READ_CYCLES;
		else
			m_core->icount = 0;
	}
	else if (idx == COP0_Random)
	{
		int wired = m_core->cpr[0][COP0_Wired] & 0x3f;
		int range = 48 - wired;
		if (range > 0)
			return ((total_cycles() - m_core->count_zero_time) % range + wired) & 0x3f;
		else
			return 47;
	}
	return m_core->cpr[0][idx];
}

void mips3_device::set_cop0_reg(int idx, uint64_t val)
{
	switch (idx)
	{
		case COP0_Cause:
			CAUSE = (CAUSE & 0xfc00) | (val & ~0xfc00);
			if (CAUSE & 0x300)
			{
				/* if we're in a delay slot, propogate the target PC before generating the exception */
				if (m_nextpc != ~0)
				{
					m_core->pc = m_nextpc;
					m_nextpc = ~0;
				}
				generate_exception(EXCEPTION_INTERRUPT, 0);
			}
			break;

		case COP0_Status:
		{
			/* update interrupts and cycle counting */
			uint32_t diff = m_core->cpr[0][idx] ^ val;
//          if (val & 0xe0)
//              fatalerror("System set 64-bit addressing mode, SR=%08X\n", val);
			m_core->cpr[0][idx] = val;
			if (diff & 0x8000)
				mips3com_update_cycle_counting();
			check_irqs();
			break;
		}

		case COP0_Count:
			m_core->cpr[0][idx] = val;
			m_core->count_zero_time = total_cycles() - ((uint64_t)(uint32_t)val * 2);
			mips3com_update_cycle_counting();
			break;

		case COP0_Compare:
			m_core->compare_armed = 1;
			CAUSE &= ~0x8000;
			m_core->cpr[0][idx] = val & 0xffffffff;
			mips3com_update_cycle_counting();
			break;

		case COP0_PRId:
			break;

		case COP0_Config:
			m_core->cpr[0][idx] = (m_core->cpr[0][idx] & ~7) | (val & 7);
			break;

		case COP0_EntryHi:
			/* if the ASID changes, remap */
			if ((m_core->cpr[0][idx] ^ val) & 0xff)
			{
				m_core->cpr[0][idx] = val;
				mips3com_asid_changed();
			}
			m_core->cpr[0][idx] = val;
			break;

		default:
			m_core->cpr[0][idx] = val;
			break;
	}
}

inline uint64_t mips3_device::get_cop0_creg(int idx)
{
	return m_core->ccr[0][idx];
}

inline void mips3_device::set_cop0_creg(int idx, uint64_t val)
{
	m_core->ccr[0][idx] = val;
}

void mips3_device::handle_cop0(uint32_t op)
{
	if ((SR & SR_KSU_MASK) != SR_KSU_KERNEL && !(SR & SR_COP0))
	{
		m_badcop_value = 0;
		generate_exception(EXCEPTION_BADCOP, 1);
		return;
	}

	switch (RSREG)
	{
		case 0x00:  /* MFCz */      if (RTREG) RTVAL64 = (int32_t)get_cop0_reg(RDREG);        break;
		case 0x01:  /* DMFCz */     if (RTREG) RTVAL64 = get_cop0_reg(RDREG);               break;
		case 0x02:  /* CFCz */      if (RTREG) RTVAL64 = (int32_t)get_cop0_creg(RDREG);       break;
		case 0x04:  /* MTCz */      set_cop0_reg(RDREG, RTVAL32);                           break;
		case 0x05:  /* DMTCz */     set_cop0_reg(RDREG, RTVAL64);                           break;
		case 0x06:  /* CTCz */      set_cop0_creg(RDREG, RTVAL32);                          break;
		case 0x08:  /* BC */
			switch (RTREG)
			{
				case 0x00:  /* BCzF */  if (!m_cf[0]) ADDPC(SIMMVAL);               break;
				case 0x01:  /* BCzF */  if (m_cf[0]) ADDPC(SIMMVAL);                break;
				case 0x02:  /* BCzFL */ invalid_instruction(op);                            break;
				case 0x03:  /* BCzTL */ invalid_instruction(op);                            break;
				default:    invalid_instruction(op);                                        break;
			}
			break;
		case 0x10:
		case 0x11:
		case 0x12:
		case 0x13:
		case 0x14:
		case 0x15:
		case 0x16:
		case 0x17:
		case 0x18:
		case 0x19:
		case 0x1a:
		case 0x1b:
		case 0x1c:
		case 0x1d:
		case 0x1e:
		case 0x1f:  /* COP */
			switch (op & 0x01ffffff)
			{
				case 0x01:  /* TLBR */
					mips3com_tlbr();
					break;

				case 0x02:  /* TLBWI */
					mips3com_tlbwi();
					break;

				case 0x06:  /* TLBWR */
					mips3com_tlbwr();
					break;

				case 0x08:  /* TLBP */
					mips3com_tlbp();
					break;

				case 0x10:  /* RFE */   invalid_instruction(op);                            break;
				case 0x18:  /* ERET   logerror("ERET\n"); */ m_core->pc = m_core->cpr[0][COP0_EPC]; SR &= ~SR_EXL; check_irqs(); m_lld_value ^= 0xffffffff; m_ll_value ^= 0xffffffff;  break;
				case 0x20:  /* WAIT */                                                      break;
				default:    invalid_instruction(op);                                        break;
			}
			break;
		default:    invalid_instruction(op);                                                break;
	}
}



/***************************************************************************
    COP1 (FPU) EXECUTION HANDLING
***************************************************************************/

inline uint32_t mips3_device::get_cop1_reg32(int idx)
{
	if (IS_FR0)
		return ((uint32_t *)&m_core->cpr[1][idx & 0x1E])[idx & 1];
	else
		return m_core->cpr[1][idx];
}

inline uint64_t mips3_device::get_cop1_reg64(int idx)
{
	if (IS_FR0)
		idx &= 0x1E;
	return m_core->cpr[1][idx];
}

inline void mips3_device::set_cop1_reg32(int idx, uint32_t val)
{
	if (IS_FR0)
		((uint32_t *)&m_core->cpr[1][idx & 0x1E])[idx & 1] = val;
	else
		m_core->cpr[1][idx] = val;
}

inline void mips3_device::set_cop1_reg64(int idx, uint64_t val)
{
	if (IS_FR0)
		idx &= 0x1E;
	m_core->cpr[1][idx] = val;
}

inline uint64_t mips3_device::get_cop1_creg(int idx)
{
	if (idx == 31)
	{
		uint32_t result = m_core->ccr[1][31] & ~0xfe800000;
		int i;

		for (i = 0; i < 8; i++)
			if (m_cf[1][i])
				result |= 1 << fcc_shift[i];
		return result;
	}
	return m_core->ccr[1][idx];
}

inline void mips3_device::set_cop1_creg(int idx, uint64_t val)
{
	m_core->ccr[1][idx] = val;
	if (idx == 31)
	{
		int i;

		for (i = 0; i < 8; i++)
			m_cf[1][i] = (val >> fcc_shift[i]) & 1;
	}
}

void mips3_device::handle_cop1_fr0(uint32_t op)
{
	double dtemp;

	/* note: additional condition codes available on R5000 only */

	if (!(SR & SR_COP1))
	{
		m_badcop_value = 1;
		generate_exception(EXCEPTION_BADCOP, 1);
		return;
	}

	switch (RSREG)
	{
		case 0x00:  /* MFCz */      if (RTREG) RTVAL64 = (int32_t)get_cop1_reg32(RDREG);      break;
		case 0x01:  /* DMFCz */     if (RTREG) RTVAL64 = get_cop1_reg64(RDREG);             break;
		case 0x02:  /* CFCz */      if (RTREG) RTVAL64 = (int32_t)get_cop1_creg(RDREG);       break;
		case 0x04:  /* MTCz */      set_cop1_reg32(RDREG, RTVAL32);                         break;
		case 0x05:  /* DMTCz */     set_cop1_reg64(RDREG, RTVAL64);                         break;
		case 0x06:  /* CTCz */      set_cop1_creg(RDREG, RTVAL32);                          break;
		case 0x08:  /* BC */
			switch ((op >> 16) & 3)
			{
				case 0x00:  /* BCzF */  if (!GET_FCC((op >> 18) & 7)) ADDPC(SIMMVAL);   break;
				case 0x01:  /* BCzT */  if (GET_FCC((op >> 18) & 7)) ADDPC(SIMMVAL);    break;
				case 0x02:  /* BCzFL */ if (!GET_FCC((op >> 18) & 7)) ADDPC(SIMMVAL); else m_core->pc += 4;  break;
				case 0x03:  /* BCzTL */ if (GET_FCC((op >> 18) & 7)) ADDPC(SIMMVAL); else m_core->pc += 4;   break;
			}
			break;
		default:
			switch (op & 0x3f)
			{
				case 0x00:
					if (IS_SINGLE(op))  /* ADD.S */
						FDVALS_FR0 = FSVALS_FR0 + FTVALS_FR0;
					else                /* ADD.D */
						FDVALD_FR0 = FSVALD_FR0 + FTVALD_FR0;
					break;

				case 0x01:
					if (IS_SINGLE(op))  /* SUB.S */
						FDVALS_FR0 = FSVALS_FR0 - FTVALS_FR0;
					else                /* SUB.D */
						FDVALD_FR0 = FSVALD_FR0 - FTVALD_FR0;
					break;

				case 0x02:
					if (IS_SINGLE(op))  /* MUL.S */
						FDVALS_FR0 = FSVALS_FR0 * FTVALS_FR0;
					else                /* MUL.D */
						FDVALD_FR0 = FSVALD_FR0 * FTVALD_FR0;
					break;

				case 0x03:
					if (IS_SINGLE(op))  /* DIV.S */
						FDVALS_FR0 = FSVALS_FR0 / FTVALS_FR0;
					else                /* DIV.D */
						FDVALD_FR0 = FSVALD_FR0 / FTVALD_FR0;
					break;

				case 0x04:
					if (IS_SINGLE(op))  /* SQRT.S */
						FDVALS_FR0 = sqrt(FSVALS_FR0);
					else                /* SQRT.D */
						FDVALD_FR0 = sqrt(FSVALD_FR0);
					break;

				case 0x05:
					if (IS_SINGLE(op))  /* ABS.S */
						FDVALS_FR0 = fabs(FSVALS_FR0);
					else                /* ABS.D */
						FDVALD_FR0 = fabs(FSVALD_FR0);
					break;

				case 0x06:
					if (IS_SINGLE(op))  /* MOV.S */
						FDVALS_FR0 = FSVALS_FR0;
					else                /* MOV.D */
						FDVALD_FR0 = FSVALD_FR0;
					break;

				case 0x07:
					if (IS_SINGLE(op))  /* NEG.S */
						FDVALS_FR0 = -FSVALS_FR0;
					else                /* NEG.D */
						FDVALD_FR0 = -FSVALD_FR0;
					break;

				case 0x08:
					if (IS_SINGLE(op))  /* ROUND.L.S */
					{
						double temp = FSVALS_FR0;
						if (temp < 0)
							temp = ceil(temp - 0.5);
						else
							temp = floor(temp + 0.5);
						FDVALL_FR0 = (int64_t)temp;
					}
					else                /* ROUND.L.D */
					{
						double temp = FSVALD_FR0;
						if (temp < 0)
							temp = ceil(temp - 0.5);
						else
							temp = floor(temp + 0.5);
						FDVALL_FR0 = (int64_t)temp;
					}
					break;

				case 0x09:
					if (IS_SINGLE(op))  /* TRUNC.L.S */
					{
						double temp = FSVALS_FR0;
						if (temp < 0)
							temp = ceil(temp);
						else
							temp = floor(temp);
						FDVALL_FR0 = (int64_t)temp;
					}
					else                /* TRUNC.L.D */
					{
						double temp = FSVALD_FR0;
						if (temp < 0)
							temp = ceil(temp);
						else
							temp = floor(temp);
						FDVALL_FR0 = (int64_t)temp;
					}
					break;

				case 0x0a:
					if (IS_SINGLE(op))  /* CEIL.L.S */
						dtemp = ceil(FSVALS_FR0);
					else                /* CEIL.L.D */
						dtemp = ceil(FSVALD_FR0);
					FDVALL_FR0 = (int64_t)dtemp;
					break;

				case 0x0b:
					if (IS_SINGLE(op))  /* FLOOR.L.S */
						dtemp = floor(FSVALS_FR0);
					else                /* FLOOR.L.D */
						dtemp = floor(FSVALD_FR0);
					FDVALL_FR0 = (int64_t)dtemp;
					break;

				case 0x0c:
					if (IS_SINGLE(op))  /* ROUND.W.S */
					{
						dtemp = FSVALS_FR0;
						if (dtemp < 0)
							dtemp = ceil(dtemp - 0.5);
						else
							dtemp = floor(dtemp + 0.5);
						FDVALW_FR0 = (int32_t)dtemp;
					}
					else                /* ROUND.W.D */
					{
						dtemp = FSVALD_FR0;
						if (dtemp < 0)
							dtemp = ceil(dtemp - 0.5);
						else
							dtemp = floor(dtemp + 0.5);
						FDVALW_FR0 = (int32_t)dtemp;
					}
					break;

				case 0x0d:
					if (IS_SINGLE(op))  /* TRUNC.W.S */
					{
						dtemp = FSVALS_FR0;
						if (dtemp < 0)
							dtemp = ceil(dtemp);
						else
							dtemp = floor(dtemp);
						FDVALW_FR0 = (int32_t)dtemp;
					}
					else                /* TRUNC.W.D */
					{
						dtemp = FSVALD_FR0;
						if (dtemp < 0)
							dtemp = ceil(dtemp);
						else
							dtemp = floor(dtemp);
						FDVALW_FR0 = (int32_t)dtemp;
					}
					break;

				case 0x0e:
					if (IS_SINGLE(op))  /* CEIL.W.S */
						dtemp = ceil(FSVALS_FR0);
					else                /* CEIL.W.D */
						dtemp = ceil(FSVALD_FR0);
					FDVALW_FR0 = (int32_t)dtemp;
					break;

				case 0x0f:
					if (IS_SINGLE(op))  /* FLOOR.W.S */
						dtemp = floor(FSVALS_FR0);
					else                /* FLOOR.W.D */
						dtemp = floor(FSVALD_FR0);
					FDVALW_FR0 = (int32_t)dtemp;
					break;

				case 0x11:  /* R5000 */
					if (GET_FCC((op >> 18) & 7) == ((op >> 16) & 1))
					{
						if (IS_SINGLE(op))  /* MOVT/F.S */
							FDVALS_FR0 = FSVALS_FR0;
						else                /* MOVT/F.D */
							FDVALD_FR0 = FSVALD_FR0;
					}
					break;

				case 0x12:  /* R5000 */
					if (RTVAL64 == 0)
					{
						if (IS_SINGLE(op))  /* MOVZ.S */
							FDVALS_FR0 = FSVALS_FR0;
						else                /* MOVZ.D */
							FDVALD_FR0 = FSVALD_FR0;
					}
					break;

				case 0x13:  /* R5000 */
					if (RTVAL64 != 0)
					{
						if (IS_SINGLE(op))  /* MOVN.S */
							FDVALS_FR0 = FSVALS_FR0;
						else                /* MOVN.D */
							FDVALD_FR0 = FSVALD_FR0;
					}
					break;

				case 0x15:  /* R5000 */
					if (IS_SINGLE(op))  /* RECIP.S */
						FDVALS_FR0 = 1.0f / FSVALS_FR0;
					else                /* RECIP.D */
						FDVALD_FR0 = 1.0 / FSVALD_FR0;
					break;

				case 0x16:  /* R5000 */
					if (IS_SINGLE(op))  /* RSQRT.S */
						FDVALS_FR0 = 1.0f / sqrt(FSVALS_FR0);
					else                /* RSQRT.D */
						FDVALD_FR0 = 1.0 / sqrt(FSVALD_FR0);
					break;

				case 0x20:
					if (IS_INTEGRAL(op))
					{
						if (IS_SINGLE(op))  /* CVT.S.W */
							FDVALS_FR0 = (int32_t)FSVALW_FR0;
						else                /* CVT.S.L */
							FDVALS_FR0 = (int64_t)FSVALL_FR0;
					}
					else                    /* CVT.S.D */
						FDVALS_FR0 = FSVALD_FR0;
					break;

				case 0x21:
					if (IS_INTEGRAL(op))
					{
						if (IS_SINGLE(op))  /* CVT.D.W */
							FDVALD_FR0 = (int32_t)FSVALW_FR0;
						else                /* CVT.D.L */
							FDVALD_FR0 = (int64_t)FSVALL_FR0;
					}
					else                    /* CVT.D.S */
						FDVALD_FR0 = FSVALS_FR0;
					break;

				case 0x24:
					if (IS_SINGLE(op))  /* CVT.W.S */
						FDVALW_FR0 = (int32_t)FSVALS_FR0;
					else
						FDVALW_FR0 = (int32_t)FSVALD_FR0;
					break;

				case 0x25:
					if (IS_SINGLE(op))  /* CVT.L.S */
						FDVALL_FR0 = (int64_t)FSVALS_FR0;
					else                /* CVT.L.D */
						FDVALL_FR0 = (int64_t)FSVALD_FR0;
					break;

				case 0x30:
				case 0x38:
					if (IS_SINGLE(op))  /* C.F.S */
						SET_FCC((op >> 8) & 7, 0);
					else                /* C.F.D */
						SET_FCC((op >> 8) & 7, 0);
					break;

				case 0x31:
				case 0x39:
					if (IS_SINGLE(op))  /* C.UN.S */
						SET_FCC((op >> 8) & 7, 0);
					else                /* C.UN.D */
						SET_FCC((op >> 8) & 7, 0);
					break;

				case 0x32:
				case 0x3a:
					if (IS_SINGLE(op))  /* C.EQ.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR0 == FTVALS_FR0));
					else                /* C.EQ.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR0 == FTVALD_FR0));
					break;

				case 0x33:
				case 0x3b:
					if (IS_SINGLE(op))  /* C.UEQ.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR0 == FTVALS_FR0));
					else                /* C.UEQ.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR0 == FTVALD_FR0));
					break;

				case 0x34:
				case 0x3c:
					if (IS_SINGLE(op))  /* C.OLT.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR0 < FTVALS_FR0));
					else                /* C.OLT.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR0 < FTVALD_FR0));
					break;

				case 0x35:
				case 0x3d:
					if (IS_SINGLE(op))  /* C.ULT.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR0 < FTVALS_FR0));
					else                /* C.ULT.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR0 < FTVALD_FR0));
					break;

				case 0x36:
				case 0x3e:
					if (IS_SINGLE(op))  /* C.OLE.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR0 <= FTVALS_FR0));
					else                /* C.OLE.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR0 <= FTVALD_FR0));
					break;

				case 0x37:
				case 0x3f:
					if (IS_SINGLE(op))  /* C.ULE.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR0 <= FTVALS_FR0));
					else                /* C.ULE.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR0 <= FTVALD_FR0));
					break;

				default:
					fprintf(stderr, "cop1 %X\n", op);
					break;
			}
			break;
	}
}


void mips3_device::handle_cop1_fr1(uint32_t op)
{
	double dtemp;

	/* note: additional condition codes available on R5000 only */

	if (!(SR & SR_COP1))
	{
		m_badcop_value = 1;
		generate_exception(EXCEPTION_BADCOP, 1);
		return;
	}

	switch (RSREG)
	{
		case 0x00:  /* MFCz */      if (RTREG) RTVAL64 = (int32_t)get_cop1_reg32(RDREG);      break;
		case 0x01:  /* DMFCz */     if (RTREG) RTVAL64 = get_cop1_reg64(RDREG);             break;
		case 0x02:  /* CFCz */      if (RTREG) RTVAL64 = (int32_t)get_cop1_creg(RDREG);       break;
		case 0x04:  /* MTCz */      set_cop1_reg32(RDREG, RTVAL32);                         break;
		case 0x05:  /* DMTCz */     set_cop1_reg64(RDREG, RTVAL64);                         break;
		case 0x06:  /* CTCz */      set_cop1_creg(RDREG, RTVAL32);                          break;
		case 0x08:  /* BC */
			switch ((op >> 16) & 3)
			{
				case 0x00:  /* BCzF */  if (!GET_FCC((op >> 18) & 7)) ADDPC(SIMMVAL);   break;
				case 0x01:  /* BCzT */  if (GET_FCC((op >> 18) & 7)) ADDPC(SIMMVAL);    break;
				case 0x02:  /* BCzFL */ if (!GET_FCC((op >> 18) & 7)) ADDPC(SIMMVAL); else m_core->pc += 4;  break;
				case 0x03:  /* BCzTL */ if (GET_FCC((op >> 18) & 7)) ADDPC(SIMMVAL); else m_core->pc += 4;   break;
			}
			break;
		default:
			switch (op & 0x3f)
			{
				case 0x00:
					if (IS_SINGLE(op))  /* ADD.S */
						FDVALS_FR1 = FSVALS_FR1 + FTVALS_FR1;
					else                /* ADD.D */
						FDVALD_FR1 = FSVALD_FR1 + FTVALD_FR1;
					break;

				case 0x01:
					if (IS_SINGLE(op))  /* SUB.S */
						FDVALS_FR1 = FSVALS_FR1 - FTVALS_FR1;
					else                /* SUB.D */
						FDVALD_FR1 = FSVALD_FR1 - FTVALD_FR1;
					break;

				case 0x02:
					if (IS_SINGLE(op))  /* MUL.S */
						FDVALS_FR1 = FSVALS_FR1 * FTVALS_FR1;
					else                /* MUL.D */
						FDVALD_FR1 = FSVALD_FR1 * FTVALD_FR1;
					break;

				case 0x03:
					if (IS_SINGLE(op))  /* DIV.S */
						FDVALS_FR1 = FSVALS_FR1 / FTVALS_FR1;
					else                /* DIV.D */
						FDVALD_FR1 = FSVALD_FR1 / FTVALD_FR1;
					break;

				case 0x04:
					if (IS_SINGLE(op))  /* SQRT.S */
						FDVALS_FR1 = sqrt(FSVALS_FR1);
					else                /* SQRT.D */
						FDVALD_FR1 = sqrt(FSVALD_FR1);
					break;

				case 0x05:
					if (IS_SINGLE(op))  /* ABS.S */
						FDVALS_FR1 = fabs(FSVALS_FR1);
					else                /* ABS.D */
						FDVALD_FR1 = fabs(FSVALD_FR1);
					break;

				case 0x06:
					if (IS_SINGLE(op))  /* MOV.S */
						FDVALS_FR1 = FSVALS_FR1;
					else                /* MOV.D */
						FDVALD_FR1 = FSVALD_FR1;
					break;

				case 0x07:
					if (IS_SINGLE(op))  /* NEG.S */
						FDVALS_FR1 = -FSVALS_FR1;
					else                /* NEG.D */
						FDVALD_FR1 = -FSVALD_FR1;
					break;

				case 0x08:
					if (IS_SINGLE(op))  /* ROUND.L.S */
					{
						double temp = FSVALS_FR1;
						if (temp < 0)
							temp = ceil(temp - 0.5);
						else
							temp = floor(temp + 0.5);
						FDVALL_FR1 = (int64_t)temp;
					}
					else                /* ROUND.L.D */
					{
						double temp = FSVALD_FR1;
						if (temp < 0)
							temp = ceil(temp - 0.5);
						else
							temp = floor(temp + 0.5);
						FDVALL_FR1 = (int64_t)temp;
					}
					break;

				case 0x09:
					if (IS_SINGLE(op))  /* TRUNC.L.S */
					{
						double temp = FSVALS_FR1;
						if (temp < 0)
							temp = ceil(temp);
						else
							temp = floor(temp);
						FDVALL_FR1 = (int64_t)temp;
					}
					else                /* TRUNC.L.D */
					{
						double temp = FSVALD_FR1;
						if (temp < 0)
							temp = ceil(temp);
						else
							temp = floor(temp);
						FDVALL_FR1 = (int64_t)temp;
					}
					break;

				case 0x0a:
					if (IS_SINGLE(op))  /* CEIL.L.S */
						dtemp = ceil(FSVALS_FR1);
					else                /* CEIL.L.D */
						dtemp = ceil(FSVALD_FR1);
					FDVALL_FR1 = (int64_t)dtemp;
					break;

				case 0x0b:
					if (IS_SINGLE(op))  /* FLOOR.L.S */
						dtemp = floor(FSVALS_FR1);
					else                /* FLOOR.L.D */
						dtemp = floor(FSVALD_FR1);
					FDVALL_FR1 = (int64_t)dtemp;
					break;

				case 0x0c:
					if (IS_SINGLE(op))  /* ROUND.W.S */
					{
						dtemp = FSVALS_FR1;
						if (dtemp < 0)
							dtemp = ceil(dtemp - 0.5);
						else
							dtemp = floor(dtemp + 0.5);
						FDVALW_FR1 = (int32_t)dtemp;
					}
					else                /* ROUND.W.D */
					{
						dtemp = FSVALD_FR1;
						if (dtemp < 0)
							dtemp = ceil(dtemp - 0.5);
						else
							dtemp = floor(dtemp + 0.5);
						FDVALW_FR1 = (int32_t)dtemp;
					}
					break;

				case 0x0d:
					if (IS_SINGLE(op))  /* TRUNC.W.S */
					{
						dtemp = FSVALS_FR1;
						if (dtemp < 0)
							dtemp = ceil(dtemp);
						else
							dtemp = floor(dtemp);
						FDVALW_FR1 = (int32_t)dtemp;
					}
					else                /* TRUNC.W.D */
					{
						dtemp = FSVALD_FR1;
						if (dtemp < 0)
							dtemp = ceil(dtemp);
						else
							dtemp = floor(dtemp);
						FDVALW_FR1 = (int32_t)dtemp;
					}
					break;

				case 0x0e:
					if (IS_SINGLE(op))  /* CEIL.W.S */
						dtemp = ceil(FSVALS_FR1);
					else                /* CEIL.W.D */
						dtemp = ceil(FSVALD_FR1);
					FDVALW_FR1 = (int32_t)dtemp;
					break;

				case 0x0f:
					if (IS_SINGLE(op))  /* FLOOR.W.S */
						dtemp = floor(FSVALS_FR1);
					else                /* FLOOR.W.D */
						dtemp = floor(FSVALD_FR1);
					FDVALW_FR1 = (int32_t)dtemp;
					break;

				case 0x11:  /* R5000 */
					if (GET_FCC((op >> 18) & 7) == ((op >> 16) & 1))
					{
						if (IS_SINGLE(op))  /* MOVT/F.S */
							FDVALS_FR1 = FSVALS_FR1;
						else                /* MOVT/F.D */
							FDVALD_FR1 = FSVALD_FR1;
					}
					break;

				case 0x12:  /* R5000 */
					if (RTVAL64 == 0)
					{
						if (IS_SINGLE(op))  /* MOVZ.S */
							FDVALS_FR1 = FSVALS_FR1;
						else                /* MOVZ.D */
							FDVALD_FR1 = FSVALD_FR1;
					}
					break;

				case 0x13:  /* R5000 */
					if (RTVAL64 != 0)
					{
						if (IS_SINGLE(op))  /* MOVN.S */
							FDVALS_FR1 = FSVALS_FR1;
						else                /* MOVN.D */
							FDVALD_FR1 = FSVALD_FR1;
					}
					break;

				case 0x15:  /* R5000 */
					if (IS_SINGLE(op))  /* RECIP.S */
						FDVALS_FR1 = 1.0f / FSVALS_FR1;
					else                /* RECIP.D */
						FDVALD_FR1 = 1.0 / FSVALD_FR1;
					break;

				case 0x16:  /* R5000 */
					if (IS_SINGLE(op))  /* RSQRT.S */
						FDVALS_FR1 = 1.0f / sqrt(FSVALS_FR1);
					else                /* RSQRT.D */
						FDVALD_FR1 = 1.0 / sqrt(FSVALD_FR1);
					break;

				case 0x20:
					if (IS_INTEGRAL(op))
					{
						if (IS_SINGLE(op))  /* CVT.S.W */
							FDVALS_FR1 = (int32_t)FSVALW_FR1;
						else                /* CVT.S.L */
							FDVALS_FR1 = (int64_t)FSVALL_FR1;
					}
					else                    /* CVT.S.D */
						FDVALS_FR1 = FSVALD_FR1;
					break;

				case 0x21:
					if (IS_INTEGRAL(op))
					{
						if (IS_SINGLE(op))  /* CVT.D.W */
							FDVALD_FR1 = (int32_t)FSVALW_FR1;
						else                /* CVT.D.L */
							FDVALD_FR1 = (int64_t)FSVALL_FR1;
					}
					else                    /* CVT.D.S */
						FDVALD_FR1 = FSVALS_FR1;
					break;

				case 0x24:
					if (IS_SINGLE(op))  /* CVT.W.S */
						FDVALW_FR1 = (int32_t)FSVALS_FR1;
					else
						FDVALW_FR1 = (int32_t)FSVALD_FR1;
					break;

				case 0x25:
					if (IS_SINGLE(op))  /* CVT.L.S */
						FDVALL_FR1 = (int64_t)FSVALS_FR1;
					else                /* CVT.L.D */
						FDVALL_FR1 = (int64_t)FSVALD_FR1;
					break;

				case 0x30:
				case 0x38:
					if (IS_SINGLE(op))  /* C.F.S */
						SET_FCC((op >> 8) & 7, 0);
					else                /* C.F.D */
						SET_FCC((op >> 8) & 7, 0);
					break;

				case 0x31:
				case 0x39:
					if (IS_SINGLE(op))  /* C.UN.S */
						SET_FCC((op >> 8) & 7, 0);
					else                /* C.UN.D */
						SET_FCC((op >> 8) & 7, 0);
					break;

				case 0x32:
				case 0x3a:
					if (IS_SINGLE(op))  /* C.EQ.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR1 == FTVALS_FR1));
					else                /* C.EQ.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR1 == FTVALD_FR1));
					break;

				case 0x33:
				case 0x3b:
					if (IS_SINGLE(op))  /* C.UEQ.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR1 == FTVALS_FR1));
					else                /* C.UEQ.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR1 == FTVALD_FR1));
					break;

				case 0x34:
				case 0x3c:
					if (IS_SINGLE(op))  /* C.OLT.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR1 < FTVALS_FR1));
					else                /* C.OLT.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR1 < FTVALD_FR1));
					break;

				case 0x35:
				case 0x3d:
					if (IS_SINGLE(op))  /* C.ULT.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR1 < FTVALS_FR1));
					else                /* C.ULT.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR1 < FTVALD_FR1));
					break;

				case 0x36:
				case 0x3e:
					if (IS_SINGLE(op))  /* C.OLE.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR1 <= FTVALS_FR1));
					else                /* C.OLE.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR1 <= FTVALD_FR1));
					break;

				case 0x37:
				case 0x3f:
					if (IS_SINGLE(op))  /* C.ULE.S */
						SET_FCC((op >> 8) & 7, (FSVALS_FR1 <= FTVALS_FR1));
					else                /* C.ULE.D */
						SET_FCC((op >> 8) & 7, (FSVALD_FR1 <= FTVALD_FR1));
					break;

				default:
					fprintf(stderr, "cop1 %X\n", op);
					break;
			}
			break;
	}
}



/***************************************************************************
    COP1X (FPU EXTRA) EXECUTION HANDLING
***************************************************************************/

void mips3_device::handle_cop1x_fr0(uint32_t op)
{
	uint64_t temp64;
	uint32_t temp;

	if (!(SR & SR_COP1))
	{
		m_badcop_value = 1;
		generate_exception(EXCEPTION_BADCOP, 1);
		return;
	}

	switch (op & 0x3f)
	{
		case 0x00:      /* LWXC1 */
			if (RWORD(RSVAL32 + RTVAL32, &temp)) FDVALW_FR0 = temp;
			break;

		case 0x01:      /* LDXC1 */
			if (RDOUBLE(RSVAL32 + RTVAL32, &temp64)) FDVALL_FR0 = temp64;
			break;

		case 0x08:      /* SWXC1 */
			WWORD(RSVAL32 + RTVAL32, get_cop1_reg32(FDREG));
			break;

		case 0x09:      /* SDXC1 */
			WDOUBLE(RSVAL32 + RTVAL32, get_cop1_reg64(FDREG));
			break;

		case 0x0f:      /* PREFX */
			break;

		case 0x20:      /* MADD.S */
			FDVALS_FR0 = FSVALS_FR0 * FTVALS_FR0 + FRVALS_FR0;
			break;

		case 0x21:      /* MADD.D */
			FDVALD_FR0 = FSVALD_FR0 * FTVALD_FR0 + FRVALD_FR0;
			break;

		case 0x28:      /* MSUB.S */
			FDVALS_FR0 = FSVALS_FR0 * FTVALS_FR0 - FRVALS_FR0;
			break;

		case 0x29:      /* MSUB.D */
			FDVALD_FR0 = FSVALD_FR0 * FTVALD_FR0 - FRVALD_FR0;
			break;

		case 0x30:      /* NMADD.S */
			FDVALS_FR0 = -(FSVALS_FR0 * FTVALS_FR0 + FRVALS_FR0);
			break;

		case 0x31:      /* NMADD.D */
			FDVALD_FR0 = -(FSVALD_FR0 * FTVALD_FR0 + FRVALD_FR0);
			break;

		case 0x38:      /* NMSUB.S */
			FDVALS_FR0 = -(FSVALS_FR0 * FTVALS_FR0 - FRVALS_FR0);
			break;

		case 0x39:      /* NMSUB.D */
			FDVALD_FR0 = -(FSVALD_FR0 * FTVALD_FR0 - FRVALD_FR0);
			break;

		case 0x24:      /* MADD.W */
		case 0x25:      /* MADD.L */
		case 0x2c:      /* MSUB.W */
		case 0x2d:      /* MSUB.L */
		case 0x34:      /* NMADD.W */
		case 0x35:      /* NMADD.L */
		case 0x3c:      /* NMSUB.W */
		case 0x3d:      /* NMSUB.L */
		default:
			fprintf(stderr, "cop1x %X\n", op);
			break;
	}
}

void mips3_device::handle_cop1x_fr1(uint32_t op)
{
	uint64_t temp64;
	uint32_t temp;

	if (!(SR & SR_COP1))
	{
		m_badcop_value = 1;
		generate_exception(EXCEPTION_BADCOP, 1);
		return;
	}

	switch (op & 0x3f)
	{
		case 0x00:      /* LWXC1 */
			if (RWORD(RSVAL32 + RTVAL32, &temp)) FDVALW_FR1 = temp;
			break;

		case 0x01:      /* LDXC1 */
			if (RDOUBLE(RSVAL32 + RTVAL32, &temp64)) FDVALL_FR1 = temp64;
			break;

		case 0x08:      /* SWXC1 */
			WWORD(RSVAL32 + RTVAL32, get_cop1_reg32(FDREG));
			break;

		case 0x09:      /* SDXC1 */
			WDOUBLE(RSVAL32 + RTVAL32, get_cop1_reg64(FDREG));
			break;

		case 0x0f:      /* PREFX */
			break;

		case 0x20:      /* MADD.S */
			FDVALS_FR1 = FSVALS_FR1 * FTVALS_FR1 + FRVALS_FR1;
			break;

		case 0x21:      /* MADD.D */
			FDVALD_FR1 = FSVALD_FR1 * FTVALD_FR1 + FRVALD_FR1;
			break;

		case 0x28:      /* MSUB.S */
			FDVALS_FR1 = FSVALS_FR1 * FTVALS_FR1 - FRVALS_FR1;
			break;

		case 0x29:      /* MSUB.D */
			FDVALD_FR1 = FSVALD_FR1 * FTVALD_FR1 - FRVALD_FR1;
			break;

		case 0x30:      /* NMADD.S */
			FDVALS_FR1 = -(FSVALS_FR1 * FTVALS_FR1 + FRVALS_FR1);
			break;

		case 0x31:      /* NMADD.D */
			FDVALD_FR1 = -(FSVALD_FR1 * FTVALD_FR1 + FRVALD_FR1);
			break;

		case 0x38:      /* NMSUB.S */
			FDVALS_FR1 = -(FSVALS_FR1 * FTVALS_FR1 - FRVALS_FR1);
			break;

		case 0x39:      /* NMSUB.D */
			FDVALD_FR1 = -(FSVALD_FR1 * FTVALD_FR1 - FRVALD_FR1);
			break;

		case 0x24:      /* MADD.W */
		case 0x25:      /* MADD.L */
		case 0x2c:      /* MSUB.W */
		case 0x2d:      /* MSUB.L */
		case 0x34:      /* NMADD.W */
		case 0x35:      /* NMADD.L */
		case 0x3c:      /* NMSUB.W */
		case 0x3d:      /* NMSUB.L */
		default:
			fprintf(stderr, "cop1x %X\n", op);
			break;
	}
}



/***************************************************************************
    COP2 (CUSTOM) EXECUTION HANDLING
***************************************************************************/

inline uint64_t mips3_device::get_cop2_reg(int idx)
{
	return m_core->cpr[2][idx];
}

inline void mips3_device::set_cop2_reg(int idx, uint64_t val)
{
	m_core->cpr[2][idx] = val;
}

inline uint64_t mips3_device::get_cop2_creg(int idx)
{
	return m_core->ccr[2][idx];
}

inline void mips3_device::set_cop2_creg(int idx, uint64_t val)
{
	m_core->ccr[2][idx] = val;
}

void mips3_device::handle_cop2(uint32_t op)
{
	if (!(SR & SR_COP2))
	{
		m_badcop_value = 2;
		generate_exception(EXCEPTION_BADCOP, 1);
		return;
	}

	switch (RSREG)
	{
		case 0x00:  /* MFCz */      if (RTREG) RTVAL64 = (int32_t)get_cop2_reg(RDREG);        break;
		case 0x01:  /* DMFCz */     if (RTREG) RTVAL64 = get_cop2_reg(RDREG);               break;
		case 0x02:  /* CFCz */      if (RTREG) RTVAL64 = (int32_t)get_cop2_creg(RDREG);       break;
		case 0x04:  /* MTCz */      set_cop2_reg(RDREG, RTVAL32);                           break;
		case 0x05:  /* DMTCz */     set_cop2_reg(RDREG, RTVAL64);                           break;
		case 0x06:  /* CTCz */      set_cop2_creg(RDREG, RTVAL32);                          break;
		case 0x08:  /* BC */
			switch (RTREG)
			{
				case 0x00:  /* BCzF */  if (!m_cf[2]) ADDPC(SIMMVAL);               break;
				case 0x01:  /* BCzF */  if (m_cf[2]) ADDPC(SIMMVAL);                break;
				case 0x02:  /* BCzFL */ invalid_instruction(op);                            break;
				case 0x03:  /* BCzTL */ invalid_instruction(op);                            break;
				default:    invalid_instruction(op);                                        break;
			}
			break;
		case 0x10:
		case 0x11:
		case 0x12:
		case 0x13:
		case 0x14:
		case 0x15:
		case 0x16:
		case 0x17:
		case 0x18:
		case 0x19:
		case 0x1a:
		case 0x1b:
		case 0x1c:
		case 0x1d:
		case 0x1e:
		case 0x1f:  /* COP */       invalid_instruction(op);                                break;
		default:    invalid_instruction(op);                                                break;
	}
}



/***************************************************************************
    CORE EXECUTION LOOP
***************************************************************************/

void mips3_device::handle_regimm(uint32_t op)
{
	switch (RTREG)
	{
		case 0x00:  /* BLTZ */      if ((int64_t)RSVAL64 < 0) ADDPC(SIMMVAL);                         break;
		case 0x01:  /* BGEZ */      if ((int64_t)RSVAL64 >= 0) ADDPC(SIMMVAL);                        break;
		case 0x02:  /* BLTZL */     if ((int64_t)RSVAL64 < 0) ADDPC(SIMMVAL); else m_core->pc += 4;        break;
		case 0x03:  /* BGEZL */     if ((int64_t)RSVAL64 >= 0) ADDPC(SIMMVAL); else m_core->pc += 4;   break;
		case 0x08:  /* TGEI */      if ((int64_t)RSVAL64 >= SIMMVAL) generate_exception(EXCEPTION_TRAP, 1);   break;
		case 0x09:  /* TGEIU */     if (RSVAL64 >= UIMMVAL) generate_exception(EXCEPTION_TRAP, 1);  break;
		case 0x0a:  /* TLTI */      if ((int64_t)RSVAL64 < SIMMVAL) generate_exception(EXCEPTION_TRAP, 1);    break;
		case 0x0b:  /* TLTIU */     if (RSVAL64 >= UIMMVAL) generate_exception(EXCEPTION_TRAP, 1);  break;
		case 0x0c:  /* TEQI */      if (RSVAL64 == UIMMVAL) generate_exception(EXCEPTION_TRAP, 1);  break;
		case 0x0e:  /* TNEI */      if (RSVAL64 != UIMMVAL) generate_exception(EXCEPTION_TRAP, 1);  break;
		case 0x10:  /* BLTZAL */    m_core->r[31] = (int32_t)(m_core->pc + 4); if ((int64_t)RSVAL64 < 0) ADDPC(SIMMVAL);                     break;
		case 0x11:  /* BGEZAL */    m_core->r[31] = (int32_t)(m_core->pc + 4); if ((int64_t)RSVAL64 >= 0) ADDPC(SIMMVAL);                    break;
		case 0x12:  /* BLTZALL */   m_core->r[31] = (int32_t)(m_core->pc + 4); if ((int64_t)RSVAL64 < 0) ADDPC(SIMMVAL); else m_core->pc += 4; break;
		case 0x13:  /* BGEZALL */   m_core->r[31] = (int32_t)(m_core->pc + 4); if ((int64_t)RSVAL64 >= 0) ADDPC(SIMMVAL); else m_core->pc += 4;    break;
		default:    /* ??? */       invalid_instruction(op);                                        break;
	}
}

void mips3_device::handle_special(uint32_t op)
{
	switch (op & 63)
	{
		case 0x00:  /* SLL */       if (RDREG) RDVAL64 = (int32_t)(RTVAL32 << SHIFT);                 break;
		case 0x01:  /* MOVF - R5000*/if (RDREG && GET_FCC((op >> 18) & 7) == ((op >> 16) & 1)) RDVAL64 = RSVAL64;   break;
		case 0x02:  /* SRL */       if (RDREG) RDVAL64 = (int32_t)(RTVAL32 >> SHIFT);                 break;
		case 0x03:  /* SRA */       if (RDREG) RDVAL64 = (int32_t)RTVAL32 >> SHIFT;                   break;
		case 0x04:  /* SLLV */      if (RDREG) RDVAL64 = (int32_t)(RTVAL32 << (RSVAL32 & 31));        break;
		case 0x06:  /* SRLV */      if (RDREG) RDVAL64 = (int32_t)(RTVAL32 >> (RSVAL32 & 31));        break;
		case 0x07:  /* SRAV */      if (RDREG) RDVAL64 = (int32_t)RTVAL32 >> (RSVAL32 & 31);          break;
		case 0x08:  /* JR */        SETPC(RSVAL32);                                                 break;
		case 0x09:  /* JALR */      SETPCL(RSVAL32,RDREG);                                          break;
		case 0x0a:  /* MOVZ - R5000 */if (RTVAL64 == 0) { if (RDREG) RDVAL64 = RSVAL64; }           break;
		case 0x0b:  /* MOVN - R5000 */if (RTVAL64 != 0) { if (RDREG) RDVAL64 = RSVAL64; }           break;
		case 0x0c:  /* SYSCALL */   generate_exception(EXCEPTION_SYSCALL, 1);                       break;
		case 0x0d:  /* BREAK */     generate_exception(EXCEPTION_BREAK, 1);                         break;
		case 0x0f:  /* SYNC */      /* effective no-op */                                           break;
		case 0x10:  /* MFHI */      if (RDREG) RDVAL64 = HIVAL64;                                   break;
		case 0x11:  /* MTHI */      HIVAL64 = RSVAL64;                                              break;
		case 0x12:  /* MFLO */      if (RDREG) RDVAL64 = LOVAL64;                                   break;
		case 0x13:  /* MTLO */      LOVAL64 = RSVAL64;                                              break;
		case 0x14:  /* DSLLV */     if (RDREG) RDVAL64 = RTVAL64 << (RSVAL32 & 63);                 break;
		case 0x16:  /* DSRLV */     if (RDREG) RDVAL64 = RTVAL64 >> (RSVAL32 & 63);                 break;
		case 0x17:  /* DSRAV */     if (RDREG) RDVAL64 = (int64_t)RTVAL64 >> (RSVAL32 & 63);          break;
		case 0x18:  /* MULT */
		{
			uint64_t temp64 = (int64_t)(int32_t)RSVAL32 * (int64_t)(int32_t)RTVAL32;
			LOVAL64 = (int32_t)temp64;
			HIVAL64 = (int32_t)(temp64 >> 32);
			m_core->icount -= 3;
			break;
		}
		case 0x19:  /* MULTU */
		{
			uint64_t temp64 = (uint64_t)RSVAL32 * (uint64_t)RTVAL32;
			LOVAL64 = (int32_t)temp64;
			HIVAL64 = (int32_t)(temp64 >> 32);
			m_core->icount -= 3;
			break;
		}
		case 0x1a:  /* DIV */
			if (RTVAL32)
			{
				LOVAL64 = (int32_t)((int32_t)RSVAL32 / (int32_t)RTVAL32);
				HIVAL64 = (int32_t)((int32_t)RSVAL32 % (int32_t)RTVAL32);
			}
			m_core->icount -= 35;
			break;
		case 0x1b:  /* DIVU */
			if (RTVAL32)
			{
				LOVAL64 = (int32_t)(RSVAL32 / RTVAL32);
				HIVAL64 = (int32_t)(RSVAL32 % RTVAL32);
			}
			m_core->icount -= 35;
			break;
		case 0x1c:  /* DMULT */
		{
			int64_t rshi = (int32_t)(RSVAL64 >> 32);
			int64_t rthi = (int32_t)(RTVAL64 >> 32);
			int64_t rslo = (uint32_t)RSVAL64;
			int64_t rtlo = (uint32_t)RTVAL64;
			int64_t mid_prods = (rshi * rtlo) + (rslo * rthi);
			uint64_t lo_prod = (rslo * rtlo);
			int64_t hi_prod = (rshi * rthi);
			mid_prods += lo_prod >> 32;

			HIVAL64 = hi_prod + (mid_prods >> 32);
			LOVAL64 = (uint32_t)lo_prod + (mid_prods << 32);
			m_core->icount -= 7;
			break;
		}
		case 0x1d:  /* DMULTU */
		{
			uint64_t rshi = (int32_t)(RSVAL64 >> 32);
			uint64_t rthi = (int32_t)(RTVAL64 >> 32);
			uint64_t rslo = (uint32_t)RSVAL64;
			uint64_t rtlo = (uint32_t)RTVAL64;
			uint64_t mid_prods = (rshi * rtlo) + (rslo * rthi);
			uint64_t lo_prod = (rslo * rtlo);
			uint64_t hi_prod = (rshi * rthi);
			mid_prods += lo_prod >> 32;

			HIVAL64 = hi_prod + (mid_prods >> 32);
			LOVAL64 = (uint32_t)lo_prod + (mid_prods << 32);
			m_core->icount -= 7;
			break;
		}
		case 0x1e:  /* DDIV */
			if (RTVAL64)
			{
				LOVAL64 = (int64_t)RSVAL64 / (int64_t)RTVAL64;
				HIVAL64 = (int64_t)RSVAL64 % (int64_t)RTVAL64;
			}
			m_core->icount -= 67;
			break;
		case 0x1f:  /* DDIVU */
			if (RTVAL64)
			{
				LOVAL64 = RSVAL64 / RTVAL64;
				HIVAL64 = RSVAL64 % RTVAL64;
			}
			m_core->icount -= 67;
			break;
		case 0x20:  /* ADD */
			if (ENABLE_OVERFLOWS && RSVAL32 > ~RTVAL32) generate_exception(EXCEPTION_OVERFLOW, 1);
			else if (RDREG) RDVAL64 = (int32_t)(RSVAL32 + RTVAL32);
			break;
		case 0x21:  /* ADDU */      if (RDREG) RDVAL64 = (int32_t)(RSVAL32 + RTVAL32);                break;
		case 0x22:  /* SUB */
			if (ENABLE_OVERFLOWS && RSVAL32 < RTVAL32) generate_exception(EXCEPTION_OVERFLOW, 1);
			else if (RDREG) RDVAL64 = (int32_t)(RSVAL32 - RTVAL32);
			break;
		case 0x23:  /* SUBU */      if (RDREG) RDVAL64 = (int32_t)(RSVAL32 - RTVAL32);                break;
		case 0x24:  /* AND */       if (RDREG) RDVAL64 = RSVAL64 & RTVAL64;                         break;
		case 0x25:  /* OR */        if (RDREG) RDVAL64 = RSVAL64 | RTVAL64;                         break;
		case 0x26:  /* XOR */       if (RDREG) RDVAL64 = RSVAL64 ^ RTVAL64;                         break;
		case 0x27:  /* NOR */       if (RDREG) RDVAL64 = ~(RSVAL64 | RTVAL64);                      break;
		case 0x2a:  /* SLT */       if (RDREG) RDVAL64 = (int64_t)RSVAL64 < (int64_t)RTVAL64;           break;
		case 0x2b:  /* SLTU */      if (RDREG) RDVAL64 = (uint64_t)RSVAL64 < (uint64_t)RTVAL64;         break;
		case 0x2c:  /* DADD */
			if (ENABLE_OVERFLOWS && RSVAL64 > ~RTVAL64) generate_exception(EXCEPTION_OVERFLOW, 1);
			else if (RDREG) RDVAL64 = RSVAL64 + RTVAL64;
			break;
		case 0x2d:  /* DADDU */     if (RDREG) RDVAL64 = RSVAL64 + RTVAL64;                         break;
		case 0x2e:  /* DSUB */
			if (ENABLE_OVERFLOWS && RSVAL64 < RTVAL64) generate_exception(EXCEPTION_OVERFLOW, 1);
			else if (RDREG) RDVAL64 = RSVAL64 - RTVAL64;
			break;
		case 0x2f:  /* DSUBU */     if (RDREG) RDVAL64 = RSVAL64 - RTVAL64;                         break;
		case 0x30:  /* TGE */       if ((int64_t)RSVAL64 >= (int64_t)RTVAL64) generate_exception(EXCEPTION_TRAP, 1); break;
		case 0x31:  /* TGEU */      if (RSVAL64 >= RTVAL64) generate_exception(EXCEPTION_TRAP, 1);  break;
		case 0x32:  /* TLT */       if ((int64_t)RSVAL64 < (int64_t)RTVAL64) generate_exception(EXCEPTION_TRAP, 1); break;
		case 0x33:  /* TLTU */      if (RSVAL64 < RTVAL64) generate_exception(EXCEPTION_TRAP, 1);   break;
		case 0x34:  /* TEQ */       if (RSVAL64 == RTVAL64) generate_exception(EXCEPTION_TRAP, 1);  break;
		case 0x36:  /* TNE */       if (RSVAL64 != RTVAL64) generate_exception(EXCEPTION_TRAP, 1);  break;
		case 0x38:  /* DSLL */      if (RDREG) RDVAL64 = RTVAL64 << SHIFT;                          break;
		case 0x3a:  /* DSRL */      if (RDREG) RDVAL64 = RTVAL64 >> SHIFT;                          break;
		case 0x3b:  /* DSRA */      if (RDREG) RDVAL64 = (int64_t)RTVAL64 >> SHIFT;                   break;
		case 0x3c:  /* DSLL32 */    if (RDREG) RDVAL64 = RTVAL64 << (SHIFT + 32);                   break;
		case 0x3e:  /* DSRL32 */    if (RDREG) RDVAL64 = RTVAL64 >> (SHIFT + 32);                   break;
		case 0x3f:  /* DSRA32 */    if (RDREG) RDVAL64 = (int64_t)RTVAL64 >> (SHIFT + 32);            break;
		default:    /* ??? */       invalid_instruction(op);                                        break;
	}
}

void mips3_device::burn_cycles(int32_t cycles)
{
	execute_burn(cycles);
}

void mips3_device::execute_run()
{
	if (m_isdrc)
	{
		int execute_result;

		/* reset the cache if dirty */
		if (m_cache_dirty)
			code_flush_cache();
		m_cache_dirty = false;

		/* execute */
		do
		{
			/* run as much as we can */
			execute_result = m_drcuml->execute(*m_entry);

			/* if we need to recompile, do it */
			if (execute_result == EXECUTE_MISSING_CODE)
			{
				code_compile_block(m_core->mode, m_core->pc);
			}
			else if (execute_result == EXECUTE_UNMAPPED_CODE)
			{
				fatalerror("Attempted to execute unmapped code at PC=%08X\n", m_core->pc);
			}
			else if (execute_result == EXECUTE_RESET_CACHE)
			{
				code_flush_cache();
			}

		} while (execute_result != EXECUTE_OUT_OF_CYCLES);

		return;
	}

	/* count cycles and interrupt cycles */
	m_core->icount -= m_interrupt_cycles;
	m_interrupt_cycles = 0;

	/* update timers & such */
	mips3com_update_cycle_counting();

	/* check for IRQs */
	check_irqs();

	/* core execution loop */
	do
	{
		uint32_t op;
		uint64_t temp64 = 0;
		uint32_t temp;

		/* debugging */
		m_ppc = m_core->pc;
		debugger_instruction_hook(this, m_core->pc);

		/* instruction fetch */
		if(!RWORD(m_core->pc, &op))
		{
			continue;
		}

		/* adjust for next PC */
		if (m_nextpc != ~0)
		{
			/* Exceptions need to be able to see delayslot, since nextpc gets cleared before instruction execution */
			m_delayslot = true;
			m_core->pc = m_nextpc;
			m_nextpc = ~0;
		}
		else
		{
			m_delayslot = false;
			m_core->pc += 4;
		}
		/* parse the instruction */
		const int switch_val = (op >> 26) & 0x3f;

		switch (switch_val)
		{
			case 0x00:  /* SPECIAL */
				handle_special(op);
				break;

			case 0x01:  /* REGIMM */
				handle_regimm(op);
				break;

			case 0x02:  /* J */         ABSPC(LIMMVAL);                                                         break;
			case 0x03:  /* JAL */       ABSPCL(LIMMVAL,31);                                                     break;
			case 0x04:  /* BEQ */       if (RSVAL64 == RTVAL64) ADDPC(SIMMVAL);                                 break;
			case 0x05:  /* BNE */       if (RSVAL64 != RTVAL64) ADDPC(SIMMVAL);                                 break;
			case 0x06:  /* BLEZ */      if ((int64_t)RSVAL64 <= 0) ADDPC(SIMMVAL);                                break;
			case 0x07:  /* BGTZ */      if ((int64_t)RSVAL64 > 0) ADDPC(SIMMVAL);                                 break;
			case 0x08:  /* ADDI */
				if (ENABLE_OVERFLOWS && RSVAL32 > ~SIMMVAL) generate_exception(EXCEPTION_OVERFLOW, 1);
				else if (RTREG) RTVAL64 = (int32_t)(RSVAL32 + SIMMVAL);
				break;
			case 0x09:  /* ADDIU */     if (RTREG) RTVAL64 = (int32_t)(RSVAL32 + SIMMVAL);                        break;
			case 0x0a:  /* SLTI */      if (RTREG) RTVAL64 = (int64_t)RSVAL64 < (int64_t)SIMMVAL;                   break;
			case 0x0b:  /* SLTIU */     if (RTREG) RTVAL64 = (uint64_t)RSVAL64 < (uint64_t)SIMMVAL;                 break;
			case 0x0c:  /* ANDI */      if (RTREG) RTVAL64 = RSVAL64 & UIMMVAL;                                 break;
			case 0x0d:  /* ORI */       if (RTREG) RTVAL64 = RSVAL64 | UIMMVAL;                                 break;
			case 0x0e:  /* XORI */      if (RTREG) RTVAL64 = RSVAL64 ^ UIMMVAL;                                 break;
			case 0x0f:  /* LUI */       if (RTREG) RTVAL64 = (int32_t)(UIMMVAL << 16);                            break;
			case 0x10:  /* COP0 */      handle_cop0(op);                                                        break;
			case 0x11:  /* COP1 */
				if (IS_FR0)
					handle_cop1_fr0(op);
				else
					handle_cop1_fr1(op);
				break;
			case 0x12:  /* COP2 */      handle_cop2(op);                                                        break;
			case 0x13:  /* COP1X - R5000 */
				if (IS_FR0)
					handle_cop1x_fr0(op);
				else
					handle_cop1x_fr1(op);
				break;
			case 0x14:  /* BEQL */      if (RSVAL64 == RTVAL64) ADDPC(SIMMVAL); else m_core->pc += 4;                break;
			case 0x15:  /* BNEL */      if (RSVAL64 != RTVAL64) ADDPC(SIMMVAL); else m_core->pc += 4;                break;
			case 0x16:  /* BLEZL */     if ((int64_t)RSVAL64 <= 0) ADDPC(SIMMVAL); else m_core->pc += 4;           break;
			case 0x17:  /* BGTZL */     if ((int64_t)RSVAL64 > 0) ADDPC(SIMMVAL); else m_core->pc += 4;                break;
			case 0x18:  /* DADDI */
				if (ENABLE_OVERFLOWS && (int64_t)RSVAL64 > ~SIMMVAL) generate_exception(EXCEPTION_OVERFLOW, 1);
				else if (RTREG) RTVAL64 = RSVAL64 + (int64_t)SIMMVAL;
				break;
			case 0x19:  /* DADDIU */    if (RTREG) RTVAL64 = RSVAL64 + (uint64_t)SIMMVAL;                         break;
			case 0x1a:  /* LDL */       (this->*m_ldl)(op);                                                       break;
			case 0x1b:  /* LDR */       (this->*m_ldr)(op);                                                       break;
			case 0x1c:  /* IDT-specific opcodes: mad/madu/mul on R4640/4650, msub on RC32364 */
				switch (op & 0x1f)
				{
					case 2: /* MUL */
						RDVAL64 = (int32_t)((int32_t)RSVAL32 * (int32_t)RTVAL32);
						m_core->icount -= 3;
						break;
					default: invalid_instruction(op);
				}
				break;
			case 0x20:  /* LB */        if (RBYTE(SIMMVAL+RSVAL32, &temp) && RTREG) RTVAL64 = (int8_t)temp;       break;
			case 0x21:  /* LH */        if (RHALF(SIMMVAL+RSVAL32, &temp) && RTREG) RTVAL64 = (int16_t)temp;      break;
			case 0x22:  /* LWL */       (this->*m_lwl)(op);                                                       break;
			case 0x23:  /* LW */        if (RWORD(SIMMVAL+RSVAL32, &temp) && RTREG) RTVAL64 = (int32_t)temp;      break;
			case 0x24:  /* LBU */       if (RBYTE(SIMMVAL+RSVAL32, &temp) && RTREG) RTVAL64 = (uint8_t)temp;      break;
			case 0x25:  /* LHU */       if (RHALF(SIMMVAL+RSVAL32, &temp) && RTREG) RTVAL64 = (uint16_t)temp;     break;
			case 0x26:  /* LWR */       (this->*m_lwr)(op);                                                       break;
			case 0x27:  /* LWU */       if (RWORD(SIMMVAL+RSVAL32, &temp) && RTREG) RTVAL64 = (uint32_t)temp;     break;
			case 0x28:  /* SB */        WBYTE(SIMMVAL+RSVAL32, RTVAL32);                                        break;
			case 0x29:  /* SH */        WHALF(SIMMVAL+RSVAL32, RTVAL32);                                        break;
			case 0x2a:  /* SWL */       (this->*m_swl)(op);                                                       break;
			case 0x2b:  /* SW */        WWORD(SIMMVAL+RSVAL32, RTVAL32);                                        break;
			case 0x2c:  /* SDL */       (this->*m_sdl)(op);                                                       break;
			case 0x2d:  /* SDR */       (this->*m_sdr)(op);                                                       break;
			case 0x2e:  /* SWR */       (this->*m_swr)(op);                                                       break;
			case 0x2f:  /* CACHE */     /* effective no-op */                                                   break;
			case 0x30:  /* LL */        if (RWORD(SIMMVAL+RSVAL32, &temp) && RTREG) RTVAL64 = (uint32_t)temp; m_ll_value = RTVAL32;       break;
			case 0x31:  /* LWC1 */
				if (!(SR & SR_COP1))
				{
					m_badcop_value = 1;
					generate_exception(EXCEPTION_BADCOP, 1);
					break;
				}
				if (RWORD(SIMMVAL+RSVAL32, &temp))
					set_cop1_reg32(RTREG, temp);
				break;
			case 0x32:  /* LWC2 */      if (RWORD(SIMMVAL+RSVAL32, &temp)) set_cop2_reg(RTREG, temp);           break;
			case 0x33:  /* PREF */      /* effective no-op */                                                   break;
			case 0x34:  /* LLD */       if (RDOUBLE(SIMMVAL+RSVAL32, &temp64) && RTREG) RTVAL64 = temp64; m_lld_value = temp64;     break;
			case 0x35:  /* LDC1 */
			if (!(SR & SR_COP1))
			{
				m_badcop_value = 1;
				generate_exception(EXCEPTION_BADCOP, 1);
				break;
			}
			if (RDOUBLE(SIMMVAL+RSVAL32, &temp64))
				set_cop1_reg64(RTREG, temp64);
			break;
			case 0x36:  /* LDC2 */      if (RDOUBLE(SIMMVAL+RSVAL32, &temp64)) set_cop2_reg(RTREG, temp64);     break;
			case 0x37:  /* LD */        if (RDOUBLE(SIMMVAL+RSVAL32, &temp64) && RTREG) RTVAL64 = temp64;       break;
			case 0x38:  /* SC */        if (RWORD(SIMMVAL+RSVAL32, &temp) && RTREG)
			{
				if (temp == m_ll_value)
				{
					WWORD(SIMMVAL+RSVAL32, RTVAL32);
					RTVAL64 = (uint32_t)1;
				}
				else
				{
					RTVAL64 = (uint32_t)0;
				}
			}
			break;
			case 0x39:  /* SWC1 */
				if (!(SR & SR_COP1))
				{
					m_badcop_value = 1;
					generate_exception(EXCEPTION_BADCOP, 1);
					break;
				}
				WWORD(SIMMVAL+RSVAL32, get_cop1_reg32(RTREG));
				break;
			case 0x3a:  /* SWC2 */      WWORD(SIMMVAL+RSVAL32, get_cop2_reg(RTREG));                            break;
			case 0x3b:  /* SWC3 */      invalid_instruction(op);                                                break;
			case 0x3c:  /* SCD */       if (RDOUBLE(SIMMVAL+RSVAL32, &temp64) && RTREG)
			{
				if (temp64 == m_lld_value)
				{
					WDOUBLE(SIMMVAL+RSVAL32, RTVAL64);
					RTVAL64 = 1;
				}
				else
				{
					RTVAL64 = 0;
				}
			}
			break;
			case 0x3d:  /* SDC1 */
				if (!(SR & SR_COP1))
				{
					m_badcop_value = 1;
					generate_exception(EXCEPTION_BADCOP, 1);
					break;
				}
				WDOUBLE(SIMMVAL+RSVAL32, get_cop1_reg64(RTREG));
				break;
			case 0x3e:  /* SDC2 */      WDOUBLE(SIMMVAL+RSVAL32, get_cop2_reg(RTREG));                          break;
			case 0x3f:  /* SD */        WDOUBLE(SIMMVAL+RSVAL32, RTVAL64);                                      break;
			default:    /* ??? */       invalid_instruction(op);                                                break;
		}
		/* Clear this flag once instruction execution is finished, will interfere with interrupt exceptions otherwise */
		m_delayslot = false;
		m_core->icount--;

	} while (m_core->icount > 0 || m_nextpc != ~0);

	m_core->icount -= m_interrupt_cycles;
	m_interrupt_cycles = 0;
}



/***************************************************************************
    COMPLEX OPCODE IMPLEMENTATIONS
***************************************************************************/

void mips3_device::lwl_be(uint32_t op)
{
	offs_t offs = SIMMVAL + RSVAL32;
	int shift = 8 * (offs & 3);
	uint32_t mask = 0xffffffffUL << shift;
	uint32_t temp;

	if (RWORD_MASKED(offs & ~3, &temp, mask >> shift) && RTREG)
		RTVAL64 = (int32_t)((RTVAL32 & ~mask) | (temp << shift));
}

void mips3_device::lwr_be(uint32_t op)
{
	offs_t offs = SIMMVAL + RSVAL32;
	int shift = 8 * (~offs & 3);
	uint32_t mask = 0xffffffffUL >> shift;
	uint32_t temp;

	if (RWORD_MASKED(offs & ~3, &temp, mask << shift) && RTREG)
		RTVAL64 = (int32_t)((RTVAL32 & ~mask) | (temp >> shift));
}

void mips3_device::ldl_be(uint32_t op)
{
	offs_t offs = SIMMVAL + RSVAL32;
	int shift = 8 * (offs & 7);
	uint64_t mask = 0xffffffffffffffffU << shift;
	uint64_t temp;

	if (RDOUBLE_MASKED(offs & ~7, &temp, mask >> shift) && RTREG)
		RTVAL64 = (RTVAL64 & ~mask) | (temp << shift);
}

void mips3_device::ldr_be(uint32_t op)
{
	offs_t offs = SIMMVAL + RSVAL32;
	int shift = 8 * (~offs & 7);
	uint64_t mask = 0xffffffffffffffffU >> shift;
	uint64_t temp;

	if (RDOUBLE_MASKED(offs & ~7, &temp, mask << shift) && RTREG)
		RTVAL64 = (RTVAL64 & ~mask) | (temp >> shift);
}

void mips3_device::swl_be(uint32_t op)
{
	offs_t offs = SIMMVAL + RSVAL32;
	int shift = 8 * (offs & 3);
	uint32_t mask = 0xffffffffUL >> shift;
	WWORD_MASKED(offs & ~3, RTVAL32 >> shift, mask);
}

void mips3_device::swr_be(uint32_t op)
{
	offs_t offs = SIMMVAL + RSVAL32;
	int shift = 8 * (~offs & 3);
	uint32_t mask = 0xffffffffUL << shift;
	WWORD_MASKED(offs & ~3, RTVAL32 << shift, mask);
}

void mips3_device::sdl_be(uint32_t op)
{
	offs_t offs = SIMMVAL + RSVAL32;
	int shift = 8 * (offs & 7);
	uint64_t mask = 0xffffffffffffffffU >> shift;
	WDOUBLE_MASKED(offs & ~7, RTVAL64 >> shift, mask);
}

void mips3_device::sdr_be(uint32_t op)
{
	offs_t offs = SIMMVAL + RSVAL32;
	int shift = 8 * (~offs & 7);
	uint64_t mask = 0xffffffffffffffffU << shift;
	WDOUBLE_MASKED(offs & ~7, RTVAL64 << shift, mask);
}



void mips3_device::lwl_le(uint32_t op)
{
	offs_t offs = SIMMVAL + RSVAL32;
	int shift = 8 * (~offs & 3);
	uint32_t mask = 0xffffffffUL << shift;
	uint32_t temp;

	if (RWORD_MASKED(offs & ~3, &temp, mask >> shift) && RTREG)
		RTVAL64 = (int32_t)((RTVAL32 & ~mask) | (temp << shift));
}

void mips3_device::lwr_le(uint32_t op)
{
	offs_t offs = SIMMVAL + RSVAL32;
	int shift = 8 * (offs & 3);
	uint32_t mask = 0xffffffffUL >> shift;
	uint32_t temp;

	if (RWORD_MASKED(offs & ~3, &temp, mask << shift) && RTREG)
		RTVAL64 = (int32_t)((RTVAL32 & ~mask) | (temp >> shift));
}

void mips3_device::ldl_le(uint32_t op)
{
	offs_t offs = SIMMVAL + RSVAL32;
	int shift = 8 * (~offs & 7);
	uint64_t mask = 0xffffffffffffffffU << shift;
	uint64_t temp;

	if (RDOUBLE_MASKED(offs & ~7, &temp, mask >> shift) && RTREG)
		RTVAL64 = (RTVAL64 & ~mask) | (temp << shift);
}

void mips3_device::ldr_le(uint32_t op)
{
	offs_t offs = SIMMVAL + RSVAL32;
	int shift = 8 * (offs & 7);
	uint64_t mask = 0xffffffffffffffffU >> shift;
	uint64_t temp;

	if (RDOUBLE_MASKED(offs & ~7, &temp, mask << shift) && RTREG)
		RTVAL64 = (RTVAL64 & ~mask) | (temp >> shift);
}

void mips3_device::swl_le(uint32_t op)
{
	offs_t offs = SIMMVAL + RSVAL32;
	int shift = 8 * (~offs & 3);
	uint32_t mask = 0xffffffffUL >> shift;
	WWORD_MASKED(offs & ~3, RTVAL32 >> shift, mask);
}

void mips3_device::swr_le(uint32_t op)
{
	offs_t offs = SIMMVAL + RSVAL32;
	int shift = 8 * (offs & 3);
	uint32_t mask = 0xffffffffUL << shift;
	WWORD_MASKED(offs & ~3, RTVAL32 << shift, mask);
}

void mips3_device::sdl_le(uint32_t op)
{
	offs_t offs = SIMMVAL + RSVAL32;
	int shift = 8 * (~offs & 7);
	uint64_t mask = 0xffffffffffffffffU >> shift;
	WDOUBLE_MASKED(offs & ~7, RTVAL64 >> shift, mask);
}

void mips3_device::sdr_le(uint32_t op)
{
	offs_t offs = SIMMVAL + RSVAL32;
	int shift = 8 * (offs & 7);
	uint64_t mask = 0xffffffffffffffffU << shift;
	WDOUBLE_MASKED(offs & ~7, RTVAL64 << shift, mask);
}
