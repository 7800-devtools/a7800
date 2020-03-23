// license:BSD-3-Clause
// copyright-holders:Steve Ellenoff,R. Belmont,Ryan Holtz
/*****************************************************************************
 *
 *   arm7.h
 *   Portable ARM7TDMI CPU Emulator
 *
 *   Copyright Steve Ellenoff, all rights reserved.
 *
 *  This work is based on:
 *  #1) 'Atmel Corporation ARM7TDMI (Thumb) Datasheet - January 1999'
 *  #2) Arm 2/3/6 emulator By Bryan McPhail (bmcphail@tendril.co.uk) and Phil Stroffolino (MAME CORE 0.76)
 *
 *****************************************************************************

 This file contains everything related to the arm7 cpu specific implementation.
 Anything related to the arm7 core itself is defined in arm7core.h instead.

 ******************************************************************************/

#ifndef MAME_CPU_ARM7_ARM7_H
#define MAME_CPU_ARM7_ARM7_H

#pragma once

#include "cpu/drcfe.h"
#include "cpu/drcuml.h"
#include "cpu/drcumlsh.h"


#define ARM7_MAX_FASTRAM       4
#define ARM7_MAX_HOTSPOTS      16


/***************************************************************************
    COMPILER-SPECIFIC OPTIONS
***************************************************************************/

#define ARM7DRC_STRICT_VERIFY      0x0001          /* verify all instructions */
#define ARM7DRC_FLUSH_PC           0x0008          /* flush the PC value before each memory access */

#define ARM7DRC_COMPATIBLE_OPTIONS (ARM7DRC_STRICT_VERIFY | ARM7DRC_FLUSH_PC)
#define ARM7DRC_FASTEST_OPTIONS    (0)

/****************************************************************************************************
 *  PUBLIC FUNCTIONS
 ***************************************************************************************************/

class arm7_cpu_device : public cpu_device
{
public:
	// construction/destruction
	arm7_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	arm7_cpu_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, uint8_t archRev, uint8_t archFlags, endianness_t endianness);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_execute_interface overrides
	virtual uint32_t execute_min_cycles() const override { return 3; }
	virtual uint32_t execute_max_cycles() const override { return 4; }
	virtual uint32_t execute_input_lines() const override { return 4; } /* There are actually only 2 input lines: we use 3 variants of the ABORT line while there is only 1 real one */
	virtual void execute_run() override;
	virtual void execute_set_input(int inputnum, int state) override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;
	virtual bool memory_translate(int spacenum, int intention, offs_t &address) override;

	// device_state_interface overrides
	virtual void state_export(const device_state_entry &entry) override;
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	// device_disasm_interface overrides
	virtual uint32_t disasm_min_opcode_bytes() const override { return 2; }
	virtual uint32_t disasm_max_opcode_bytes() const override { return 4; }
	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;

	address_space_config m_program_config;

	uint32_t m_r[/*NUM_REGS*/37];
	bool m_pendingIrq;
	bool m_pendingFiq;
	bool m_pendingAbtD;
	bool m_pendingAbtP;
	bool m_pendingUnd;
	bool m_pendingSwi;
	bool m_pending_interrupt;
	int m_icount;
	endianness_t m_endian;
	address_space *m_program;
	direct_read_data *m_direct;

	/* Coprocessor Registers */
	uint32_t m_control;
	uint32_t m_tlbBase;
	uint32_t m_tlb_base_mask;
	uint32_t m_faultStatus[2];
	uint32_t m_faultAddress;
	uint32_t m_fcsePID;
	uint32_t m_pid_offset;
	uint32_t m_domainAccessControl;
	uint8_t m_decoded_access_control[16];

	uint8_t m_archRev;          // ARM architecture revision (3, 4, and 5 are valid)
	uint8_t m_archFlags;        // architecture flags

//#if ARM7_MMU_ENABLE_HACK
//  uint32_t mmu_enable_addr; // workaround for "MMU is enabled when PA != VA" problem
//#endif

	uint32_t m_copro_id;

	// For debugger
	uint32_t m_pc;

	int64_t saturate_qbit_overflow(int64_t res);
	void SwitchMode(uint32_t cpsr_mode_val);
	uint32_t decodeShift(uint32_t insn, uint32_t *pCarry);
	int loadInc(uint32_t pat, uint32_t rbv, uint32_t s, int mode);
	int loadDec(uint32_t pat, uint32_t rbv, uint32_t s, int mode);
	int storeInc(uint32_t pat, uint32_t rbv, int mode);
	int storeDec(uint32_t pat, uint32_t rbv, int mode);
	void HandleCoProcDO(uint32_t insn);
	void HandleCoProcRT(uint32_t insn);
	void HandleCoProcDT(uint32_t insn);
	void HandleBranch(uint32_t insn, bool h_bit);
	void HandleMemSingle(uint32_t insn);
	void HandleHalfWordDT(uint32_t insn);
	void HandleSwap(uint32_t insn);
	void HandlePSRTransfer(uint32_t insn);
	void HandleALU(uint32_t insn);
	void HandleMul(uint32_t insn);
	void HandleSMulLong(uint32_t insn);
	void HandleUMulLong(uint32_t insn);
	void HandleMemBlock(uint32_t insn);

	void arm7ops_0123(uint32_t insn);
	void arm7ops_4567(uint32_t insn);
	void arm7ops_89(uint32_t insn);
	void arm7ops_ab(uint32_t insn);
	void arm7ops_cd(uint32_t insn);
	void arm7ops_e(uint32_t insn);
	void arm7ops_f(uint32_t insn);

	void arm9ops_undef(uint32_t insn);
	void arm9ops_1(uint32_t insn);
	void arm9ops_57(uint32_t insn);
	void arm9ops_89(uint32_t insn);
	void arm9ops_ab(uint32_t insn);
	void arm9ops_c(uint32_t insn);
	void arm9ops_e(uint32_t insn);

	void set_cpsr(uint32_t val);
	bool arm7_tlb_translate(offs_t &addr, int flags);
	uint32_t arm7_tlb_get_second_level_descriptor( uint32_t granularity, uint32_t first_desc, uint32_t vaddr );
	int detect_fault(int desc_lvl1, int ap, int flags);
	void arm7_check_irq_state();
	void update_irq_state();
	void arm7_cpu_write32(uint32_t addr, uint32_t data);
	void arm7_cpu_write16(uint32_t addr, uint16_t data);
	void arm7_cpu_write8(uint32_t addr, uint8_t data);
	uint32_t arm7_cpu_read32(uint32_t addr);
	uint16_t arm7_cpu_read16(uint32_t addr);
	uint8_t arm7_cpu_read8(uint32_t addr);

	// Coprocessor support
	DECLARE_WRITE32_MEMBER( arm7_do_callback );
	DECLARE_READ32_MEMBER( arm7_rt_r_callback );
	DECLARE_WRITE32_MEMBER( arm7_rt_w_callback );
	void arm7_dt_r_callback(uint32_t insn, uint32_t *prn);
	void arm7_dt_w_callback(uint32_t insn, uint32_t *prn);

	void tg00_0(uint32_t pc, uint32_t insn);
	void tg00_1(uint32_t pc, uint32_t insn);
	void tg01_0(uint32_t pc, uint32_t insn);
	void tg01_10(uint32_t pc, uint32_t insn);
	void tg01_11(uint32_t pc, uint32_t insn);
	void tg01_12(uint32_t pc, uint32_t insn);
	void tg01_13(uint32_t pc, uint32_t insn);
	void tg02_0(uint32_t pc, uint32_t insn);
	void tg02_1(uint32_t pc, uint32_t insn);
	void tg03_0(uint32_t pc, uint32_t insn);
	void tg03_1(uint32_t pc, uint32_t insn);
	void tg04_00_00(uint32_t pc, uint32_t insn);
	void tg04_00_01(uint32_t pc, uint32_t insn);
	void tg04_00_02(uint32_t pc, uint32_t insn);
	void tg04_00_03(uint32_t pc, uint32_t insn);
	void tg04_00_04(uint32_t pc, uint32_t insn);
	void tg04_00_05(uint32_t pc, uint32_t insn);
	void tg04_00_06(uint32_t pc, uint32_t insn);
	void tg04_00_07(uint32_t pc, uint32_t insn);
	void tg04_00_08(uint32_t pc, uint32_t insn);
	void tg04_00_09(uint32_t pc, uint32_t insn);
	void tg04_00_0a(uint32_t pc, uint32_t insn);
	void tg04_00_0b(uint32_t pc, uint32_t insn);
	void tg04_00_0c(uint32_t pc, uint32_t insn);
	void tg04_00_0d(uint32_t pc, uint32_t insn);
	void tg04_00_0e(uint32_t pc, uint32_t insn);
	void tg04_00_0f(uint32_t pc, uint32_t insn);
	void tg04_01_00(uint32_t pc, uint32_t insn);
	void tg04_01_01(uint32_t pc, uint32_t insn);
	void tg04_01_02(uint32_t pc, uint32_t insn);
	void tg04_01_03(uint32_t pc, uint32_t insn);
	void tg04_01_10(uint32_t pc, uint32_t insn);
	void tg04_01_11(uint32_t pc, uint32_t insn);
	void tg04_01_12(uint32_t pc, uint32_t insn);
	void tg04_01_13(uint32_t pc, uint32_t insn);
	void tg04_01_20(uint32_t pc, uint32_t insn);
	void tg04_01_21(uint32_t pc, uint32_t insn);
	void tg04_01_22(uint32_t pc, uint32_t insn);
	void tg04_01_23(uint32_t pc, uint32_t insn);
	void tg04_01_30(uint32_t pc, uint32_t insn);
	void tg04_01_31(uint32_t pc, uint32_t insn);
	void tg04_01_32(uint32_t pc, uint32_t insn);
	void tg04_01_33(uint32_t pc, uint32_t insn);
	void tg04_0203(uint32_t pc, uint32_t insn);
	void tg05_0(uint32_t pc, uint32_t insn);
	void tg05_1(uint32_t pc, uint32_t insn);
	void tg05_2(uint32_t pc, uint32_t insn);
	void tg05_3(uint32_t pc, uint32_t insn);
	void tg05_4(uint32_t pc, uint32_t insn);
	void tg05_5(uint32_t pc, uint32_t insn);
	void tg05_6(uint32_t pc, uint32_t insn);
	void tg05_7(uint32_t pc, uint32_t insn);
	void tg06_0(uint32_t pc, uint32_t insn);
	void tg06_1(uint32_t pc, uint32_t insn);
	void tg07_0(uint32_t pc, uint32_t insn);
	void tg07_1(uint32_t pc, uint32_t insn);
	void tg08_0(uint32_t pc, uint32_t insn);
	void tg08_1(uint32_t pc, uint32_t insn);
	void tg09_0(uint32_t pc, uint32_t insn);
	void tg09_1(uint32_t pc, uint32_t insn);
	void tg0a_0(uint32_t pc, uint32_t insn);
	void tg0a_1(uint32_t pc, uint32_t insn);
	void tg0b_0(uint32_t pc, uint32_t insn);
	void tg0b_1(uint32_t pc, uint32_t insn);
	void tg0b_2(uint32_t pc, uint32_t insn);
	void tg0b_3(uint32_t pc, uint32_t insn);
	void tg0b_4(uint32_t pc, uint32_t insn);
	void tg0b_5(uint32_t pc, uint32_t insn);
	void tg0b_6(uint32_t pc, uint32_t insn);
	void tg0b_7(uint32_t pc, uint32_t insn);
	void tg0b_8(uint32_t pc, uint32_t insn);
	void tg0b_9(uint32_t pc, uint32_t insn);
	void tg0b_a(uint32_t pc, uint32_t insn);
	void tg0b_b(uint32_t pc, uint32_t insn);
	void tg0b_c(uint32_t pc, uint32_t insn);
	void tg0b_d(uint32_t pc, uint32_t insn);
	void tg0b_e(uint32_t pc, uint32_t insn);
	void tg0b_f(uint32_t pc, uint32_t insn);
	void tg0c_0(uint32_t pc, uint32_t insn);
	void tg0c_1(uint32_t pc, uint32_t insn);
	void tg0d_0(uint32_t pc, uint32_t insn);
	void tg0d_1(uint32_t pc, uint32_t insn);
	void tg0d_2(uint32_t pc, uint32_t insn);
	void tg0d_3(uint32_t pc, uint32_t insn);
	void tg0d_4(uint32_t pc, uint32_t insn);
	void tg0d_5(uint32_t pc, uint32_t insn);
	void tg0d_6(uint32_t pc, uint32_t insn);
	void tg0d_7(uint32_t pc, uint32_t insn);
	void tg0d_8(uint32_t pc, uint32_t insn);
	void tg0d_9(uint32_t pc, uint32_t insn);
	void tg0d_a(uint32_t pc, uint32_t insn);
	void tg0d_b(uint32_t pc, uint32_t insn);
	void tg0d_c(uint32_t pc, uint32_t insn);
	void tg0d_d(uint32_t pc, uint32_t insn);
	void tg0d_e(uint32_t pc, uint32_t insn);
	void tg0d_f(uint32_t pc, uint32_t insn);
	void tg0e_0(uint32_t pc, uint32_t insn);
	void tg0e_1(uint32_t pc, uint32_t insn);
	void tg0f_0(uint32_t pc, uint32_t insn);
	void tg0f_1(uint32_t pc, uint32_t insn);

	typedef void ( arm7_cpu_device::*arm7thumb_ophandler ) (uint32_t, uint32_t);
	static const arm7thumb_ophandler thumb_handler[0x40*0x10];

	typedef void ( arm7_cpu_device::*arm7ops_ophandler )(uint32_t);
	static const arm7ops_ophandler ops_handler[0x20];

	//
	// DRC
	//

	/* fast RAM info */
	struct fast_ram_info
	{
		offs_t              start;                      /* start of the RAM block */
		offs_t              end;                        /* end of the RAM block */
		bool                readonly;                   /* true if read-only */
		void *              base;                       /* base in memory where the RAM lives */
	};

	struct hotspot_info
	{
		uint32_t             pc;
		uint32_t             opcode;
		uint32_t             cycles;
	};

	/* internal compiler state */
	struct compiler_state
	{
		uint32_t              cycles;                     /* accumulated cycles */
		uint8_t               checkints;                  /* need to check interrupts before next instruction */
		uint8_t               checksoftints;              /* need to check software interrupts before next instruction */
		uml::code_label  labelnum;                   /* index for local labels */
	};

	/* ARM7 registers */
	struct arm7imp_state
	{
		/* core state */
		drc_cache *         cache;                      /* pointer to the DRC code cache */
		drcuml_state *      drcuml;                     /* DRC UML generator state */
		//arm7_frontend *     drcfe;                      /* pointer to the DRC front-end state */
		uint32_t              drcoptions;                 /* configurable DRC options */

		/* internal stuff */
		uint8_t               cache_dirty;                /* true if we need to flush the cache */
		uint32_t              jmpdest;                    /* destination jump target */

		/* parameters for subroutines */
		uint64_t              numcycles;                  /* return value from gettotalcycles */
		uint32_t              mode;                       /* current global mode */
		const char *        format;                     /* format string for print_debug */
		uint32_t              arg0;                       /* print_debug argument 1 */
		uint32_t              arg1;                       /* print_debug argument 2 */

		/* register mappings */
		uml::parameter   regmap[/*NUM_REGS*/37];               /* parameter to register mappings for all 16 integer registers */

		/* subroutines */
		uml::code_handle *   entry;                      /* entry point */
		uml::code_handle *   nocode;                     /* nocode exception handler */
		uml::code_handle *   out_of_cycles;              /* out of cycles exception handler */
		uml::code_handle *   tlb_translate;              /* tlb translation handler */
		uml::code_handle *   detect_fault;               /* tlb fault detection handler */
		uml::code_handle *   check_irq;                  /* irq check handler */
		uml::code_handle *   read8;                      /* read byte */
		uml::code_handle *   write8;                     /* write byte */
		uml::code_handle *   read16;                     /* read half */
		uml::code_handle *   write16;                    /* write half */
		uml::code_handle *   read32;                     /* read word */
		uml::code_handle *   write32;                    /* write word */

		/* fast RAM */
		uint32_t              fastram_select;
		fast_ram_info       fastram[ARM7_MAX_FASTRAM];

		/* hotspots */
		uint32_t              hotspot_select;
		hotspot_info        hotspot[ARM7_MAX_HOTSPOTS];
	} m_impstate;

	typedef void ( arm7_cpu_device::*arm7thumb_drcophandler)(drcuml_block*, compiler_state*, const opcode_desc*);
	static const arm7thumb_drcophandler drcthumb_handler[0x40*0x10];

	void drctg00_0(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* Shift left */
	void drctg00_1(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* Shift right */
	void drctg01_0(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg01_10(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg01_11(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* SUB Rd, Rs, Rn */
	void drctg01_12(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* ADD Rd, Rs, #imm */
	void drctg01_13(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* SUB Rd, Rs, #imm */
	void drctg02_0(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg02_1(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg03_0(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* ADD Rd, #Offset8 */
	void drctg03_1(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* SUB Rd, #Offset8 */
	void drctg04_00_00(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* AND Rd, Rs */
	void drctg04_00_01(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* EOR Rd, Rs */
	void drctg04_00_02(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* LSL Rd, Rs */
	void drctg04_00_03(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* LSR Rd, Rs */
	void drctg04_00_04(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* ASR Rd, Rs */
	void drctg04_00_05(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* ADC Rd, Rs */
	void drctg04_00_06(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);  /* SBC Rd, Rs */
	void drctg04_00_07(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* ROR Rd, Rs */
	void drctg04_00_08(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* TST Rd, Rs */
	void drctg04_00_09(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* NEG Rd, Rs */
	void drctg04_00_0a(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* CMP Rd, Rs */
	void drctg04_00_0b(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* CMN Rd, Rs - check flags, add dasm */
	void drctg04_00_0c(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* ORR Rd, Rs */
	void drctg04_00_0d(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* MUL Rd, Rs */
	void drctg04_00_0e(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* BIC Rd, Rs */
	void drctg04_00_0f(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* MVN Rd, Rs */
	void drctg04_01_00(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg04_01_01(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* ADD Rd, HRs */
	void drctg04_01_02(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* ADD HRd, Rs */
	void drctg04_01_03(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* Add HRd, HRs */
	void drctg04_01_10(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);  /* CMP Rd, Rs */
	void drctg04_01_11(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* CMP Rd, Hs */
	void drctg04_01_12(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* CMP Hd, Rs */
	void drctg04_01_13(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* CMP Hd, Hs */
	void drctg04_01_20(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* MOV Rd, Rs (undefined) */
	void drctg04_01_21(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* MOV Rd, Hs */
	void drctg04_01_22(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* MOV Hd, Rs */
	void drctg04_01_23(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* MOV Hd, Hs */
	void drctg04_01_30(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg04_01_31(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg04_01_32(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg04_01_33(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg04_0203(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg05_0(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);  /* STR Rd, [Rn, Rm] */
	void drctg05_1(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);  /* STRH Rd, [Rn, Rm] */
	void drctg05_2(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);  /* STRB Rd, [Rn, Rm] */
	void drctg05_3(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);  /* LDSB Rd, [Rn, Rm] todo, add dasm */
	void drctg05_4(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);  /* LDR Rd, [Rn, Rm] */
	void drctg05_5(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);  /* LDRH Rd, [Rn, Rm] */
	void drctg05_6(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);  /* LDRB Rd, [Rn, Rm] */
	void drctg05_7(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);  /* LDSH Rd, [Rn, Rm] */
	void drctg06_0(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* Store */
	void drctg06_1(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* Load */
	void drctg07_0(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* Store */
	void drctg07_1(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);  /* Load */
	void drctg08_0(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* Store */
	void drctg08_1(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* Load */
	void drctg09_0(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* Store */
	void drctg09_1(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* Load */
	void drctg0a_0(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);  /* ADD Rd, PC, #nn */
	void drctg0a_1(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* ADD Rd, SP, #nn */
	void drctg0b_0(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* ADD SP, #imm */
	void drctg0b_1(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg0b_2(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg0b_3(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg0b_4(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* PUSH {Rlist} */
	void drctg0b_5(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* PUSH {Rlist}{LR} */
	void drctg0b_6(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg0b_7(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg0b_8(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg0b_9(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg0b_a(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg0b_b(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg0b_c(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* POP {Rlist} */
	void drctg0b_d(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* POP {Rlist}{PC} */
	void drctg0b_e(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg0b_f(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg0c_0(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* Store */
	void drctg0c_1(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* Load */
	void drctg0d_0(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); // COND_EQ:
	void drctg0d_1(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); // COND_NE:
	void drctg0d_2(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); // COND_CS:
	void drctg0d_3(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); // COND_CC:
	void drctg0d_4(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); // COND_MI:
	void drctg0d_5(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); // COND_PL:
	void drctg0d_6(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); // COND_VS:
	void drctg0d_7(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); // COND_VC:
	void drctg0d_8(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); // COND_HI:
	void drctg0d_9(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); // COND_LS:
	void drctg0d_a(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); // COND_GE:
	void drctg0d_b(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); // COND_LT:
	void drctg0d_c(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); // COND_GT:
	void drctg0d_d(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); // COND_LE:
	void drctg0d_e(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); // COND_AL:
	void drctg0d_f(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); // SWI (this is sort of a "hole" in the opcode encoding)
	void drctg0e_0(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg0e_1(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg0f_0(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void drctg0f_1(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc); /* BL */

	void update_reg_ptr();
	const int* m_reg_group;
	void load_fast_iregs(drcuml_block *block);
	void save_fast_iregs(drcuml_block *block);
	void arm7_drc_init();
	void arm7_drc_exit();
	void execute_run_drc();
	void arm7drc_set_options(uint32_t options);
	void arm7drc_add_fastram(offs_t start, offs_t end, uint8_t readonly, void *base);
	void arm7drc_add_hotspot(offs_t pc, uint32_t opcode, uint32_t cycles);
	void code_flush_cache();
	void code_compile_block(uint8_t mode, offs_t pc);
	void cfunc_get_cycles();
	void cfunc_unimplemented();
	void static_generate_entry_point();
	void static_generate_check_irq();
	void static_generate_nocode_handler();
	void static_generate_out_of_cycles();
	void static_generate_detect_fault(uml::code_handle **handleptr);
	void static_generate_tlb_translate(uml::code_handle **handleptr);
	void static_generate_memory_accessor(int size, bool istlb, bool iswrite, const char *name, uml::code_handle **handleptr);
	void generate_update_cycles(drcuml_block *block, compiler_state *compiler, uml::parameter param);
	void generate_checksum_block(drcuml_block *block, compiler_state *compiler, const opcode_desc *seqhead, const opcode_desc *seqlast);
	void generate_sequence_instruction(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void generate_delay_slot_and_branch(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc, uint8_t linkreg);

	typedef bool ( arm7_cpu_device::*drcarm7ops_ophandler)(drcuml_block*, compiler_state*, const opcode_desc*, uint32_t);
	static const drcarm7ops_ophandler drcops_handler[0x10];

	void saturate_qbit_overflow(drcuml_block *block);
	bool drcarm7ops_0123(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc, uint32_t op);
	bool drcarm7ops_4567(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc, uint32_t op);
	bool drcarm7ops_89(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc, uint32_t op);
	bool drcarm7ops_ab(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc, uint32_t op);
	bool drcarm7ops_cd(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc, uint32_t op);
	bool drcarm7ops_e(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc, uint32_t op);
	bool drcarm7ops_f(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc, uint32_t op);
	bool generate_opcode(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);

};


class arm7_be_cpu_device : public arm7_cpu_device
{
public:
	// construction/destruction
	arm7_be_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


class arm7500_cpu_device : public arm7_cpu_device
{
public:
	// construction/destruction
	arm7500_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


class arm9_cpu_device : public arm7_cpu_device
{
public:
	// construction/destruction
	arm9_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


class arm920t_cpu_device : public arm7_cpu_device
{
public:
	// construction/destruction
	arm920t_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


class pxa255_cpu_device : public arm7_cpu_device
{
public:
	// construction/destruction
	pxa255_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


class sa1110_cpu_device : public arm7_cpu_device
{
public:
	// construction/destruction
	sa1110_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


DECLARE_DEVICE_TYPE(ARM7,    arm7_cpu_device)
DECLARE_DEVICE_TYPE(ARM7_BE, arm7_be_cpu_device)
DECLARE_DEVICE_TYPE(ARM7500, arm7500_cpu_device)
DECLARE_DEVICE_TYPE(ARM9,    arm9_cpu_device)
DECLARE_DEVICE_TYPE(ARM920T, arm920t_cpu_device)
DECLARE_DEVICE_TYPE(PXA255,  pxa255_cpu_device)
DECLARE_DEVICE_TYPE(SA1110,  sa1110_cpu_device)

#endif // MAME_CPU_ARM7_ARM7_H
