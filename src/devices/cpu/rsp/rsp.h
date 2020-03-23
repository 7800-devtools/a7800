// license:BSD-3-Clause
// copyright-holders:Ville Linde, Ryan Holtz
/***************************************************************************

    rsp.h

    Interface file for the universal machine language-based
    Reality Signal Processor (RSP) emulator.

***************************************************************************/

#ifndef MAME_CPU_RSP_RSP_H
#define MAME_CPU_RSP_RSP_H

#pragma once

#include "cpu/drcfe.h"
#include "cpu/drcuml.h"

/***************************************************************************
    REGISTER ENUMERATION
***************************************************************************/

enum
{
	RSP_PC = 1,
	RSP_R0,
	RSP_R1,
	RSP_R2,
	RSP_R3,
	RSP_R4,
	RSP_R5,
	RSP_R6,
	RSP_R7,
	RSP_R8,
	RSP_R9,
	RSP_R10,
	RSP_R11,
	RSP_R12,
	RSP_R13,
	RSP_R14,
	RSP_R15,
	RSP_R16,
	RSP_R17,
	RSP_R18,
	RSP_R19,
	RSP_R20,
	RSP_R21,
	RSP_R22,
	RSP_R23,
	RSP_R24,
	RSP_R25,
	RSP_R26,
	RSP_R27,
	RSP_R28,
	RSP_R29,
	RSP_R30,
	RSP_R31,
	RSP_SR,
	RSP_NEXTPC,
	RSP_STEPCNT,
	RSP_V0,  RSP_V1,  RSP_V2,  RSP_V3,  RSP_V4,  RSP_V5,  RSP_V6,  RSP_V7,
	RSP_V8,  RSP_V9,  RSP_V10, RSP_V11, RSP_V12, RSP_V13, RSP_V14, RSP_V15,
	RSP_V16, RSP_V17, RSP_V18, RSP_V19, RSP_V20, RSP_V21, RSP_V22, RSP_V23,
	RSP_V24, RSP_V25, RSP_V26, RSP_V27, RSP_V28, RSP_V29, RSP_V30, RSP_V31
};

#define RSP_STATUS_HALT          0x0001
#define RSP_STATUS_BROKE         0x0002
#define RSP_STATUS_DMABUSY       0x0004
#define RSP_STATUS_DMAFULL       0x0008
#define RSP_STATUS_IOFULL        0x0010
#define RSP_STATUS_SSTEP         0x0020
#define RSP_STATUS_INTR_BREAK    0x0040
#define RSP_STATUS_SIGNAL0       0x0080
#define RSP_STATUS_SIGNAL1       0x0100
#define RSP_STATUS_SIGNAL2       0x0200
#define RSP_STATUS_SIGNAL3       0x0400
#define RSP_STATUS_SIGNAL4       0x0800
#define RSP_STATUS_SIGNAL5       0x1000
#define RSP_STATUS_SIGNAL6       0x2000
#define RSP_STATUS_SIGNAL7       0x4000

#define RSPDRC_STRICT_VERIFY    0x0001          /* verify all instructions */

#define MCFG_RSP_DP_REG_R_CB(_devcb) \
	devcb = &rsp_device::static_set_dp_reg_r_callback(*device, DEVCB_##_devcb);

#define MCFG_RSP_DP_REG_W_CB(_devcb) \
	devcb = &rsp_device::static_set_dp_reg_w_callback(*device, DEVCB_##_devcb);

#define MCFG_RSP_SP_REG_R_CB(_devcb) \
	devcb = &rsp_device::static_set_sp_reg_r_callback(*device, DEVCB_##_devcb);

#define MCFG_RSP_SP_REG_W_CB(_devcb) \
	devcb = &rsp_device::static_set_sp_reg_w_callback(*device, DEVCB_##_devcb);

#define MCFG_RSP_SP_SET_STATUS_CB(_devcb) \
	devcb = &rsp_device::static_set_status_callback(*device, DEVCB_##_devcb);


class rsp_frontend;
class rsp_cop2;

class rsp_device : public cpu_device
{
	friend class rsp_frontend;
	friend class rsp_cop2;
	friend class rsp_cop2_drc;

public:
	// construction/destruction
	rsp_device(const machine_config &mconfig, const char *_tag, device_t *_owner, uint32_t _clock);

	void resolve_cb();
	template <class Object> static devcb_base &static_set_dp_reg_r_callback(device_t &device, Object &&cb) { return downcast<rsp_device &>(device).m_dp_reg_r_func.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &static_set_dp_reg_w_callback(device_t &device, Object &&cb) { return downcast<rsp_device &>(device).m_dp_reg_w_func.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &static_set_sp_reg_r_callback(device_t &device, Object &&cb) { return downcast<rsp_device &>(device).m_sp_reg_r_func.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &static_set_sp_reg_w_callback(device_t &device, Object &&cb) { return downcast<rsp_device &>(device).m_sp_reg_w_func.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &static_set_status_callback(device_t &device, Object &&cb) { return downcast<rsp_device &>(device).m_sp_set_status_func.set_callback(std::forward<Object>(cb)); }

	void rspdrc_flush_drc_cache();
	void rspdrc_set_options(uint32_t options);
	void rsp_add_dmem(uint32_t *base);
	void rsp_add_imem(uint32_t *base);

	void ccfunc_read8();
	void ccfunc_read16();
	void ccfunc_read32();
	void ccfunc_write8();
	void ccfunc_write16();
	void ccfunc_write32();
	void ccfunc_get_cop0_reg();
	void ccfunc_set_cop0_reg();
	void ccfunc_sp_set_status_cb();
	void ccfunc_unimplemented();

	uint8_t* get_dmem() { return m_dmem8; }

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_stop() override;

	// device_execute_interface overrides
	virtual uint32_t execute_min_cycles() const override { return 1; }
	virtual uint32_t execute_max_cycles() const override { return 1; }
	virtual uint32_t execute_input_lines() const override { return 1; }
	virtual uint32_t execute_default_irq_vector() const override { return 0; }
	virtual void execute_run() override;
	virtual void execute_set_input(int inputnum, int state) override { }

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// device_state_interface overrides
	virtual void state_import(const device_state_entry &entry) override;
	virtual void state_export(const device_state_entry &entry) override;
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	// device_disasm_interface overrides
	virtual uint32_t disasm_min_opcode_bytes() const override { return 4; }
	virtual uint32_t disasm_max_opcode_bytes() const override { return 4; }
	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;

	void unimplemented_opcode(uint32_t op);

	/* internal compiler state */
	struct compiler_state
	{
		uint32_t              cycles;                   /* accumulated cycles */
		uint8_t               checkints;                /* need to check interrupts before next instruction */
		uint8_t               checksoftints;            /* need to check software interrupts before next instruction */
		uml::code_label     labelnum;                 /* index for local labels */
	};

private:
	address_space_config m_program_config;

	/* fast RAM info */
	struct fast_ram_info
	{
		offs_t              start;                      /* start of the RAM block */
		offs_t              end;                        /* end of the RAM block */
		bool                readonly;                   /* true if read-only */
		void *              base;                       /* base in memory where the RAM lives */
	};

	/* core state */
	drc_cache           m_cache;                      /* pointer to the DRC code cache */
	std::unique_ptr<drcuml_state>      m_drcuml;                     /* DRC UML generator state */
	std::unique_ptr<rsp_frontend>      m_drcfe;                      /* pointer to the DRC front-end state */
	uint32_t              m_drcoptions;                 /* configurable DRC options */

	/* internal stuff */
	uint8_t               m_cache_dirty;                /* true if we need to flush the cache */

	/* parameters for subroutines */
	uint64_t              m_numcycles;                  /* return value from gettotalcycles */
	const char *        m_format;                     /* format string for print_debug */
	uint32_t              m_arg2;                       /* print_debug argument 3 */
	uint32_t              m_arg3;                       /* print_debug argument 4 */

	/* register mappings */
	uml::parameter   m_regmap[34];                 /* parameter to register mappings for all 32 integer registers */

	/* subroutines */
	uml::code_handle *   m_entry;                      /* entry point */
	uml::code_handle *   m_nocode;                     /* nocode exception handler */
	uml::code_handle *   m_out_of_cycles;              /* out of cycles exception handler */
	uml::code_handle *   m_read8;                      /* read byte */
	uml::code_handle *   m_write8;                     /* write byte */
	uml::code_handle *   m_read16;                     /* read half */
	uml::code_handle *   m_write16;                    /* write half */
	uml::code_handle *   m_read32;                     /* read word */
	uml::code_handle *   m_write32;                    /* write word */

	struct internal_rsp_state
	{
		uint32_t pc;
		uint32_t r[35];
		uint32_t arg0;
		uint32_t arg1;
		uint32_t jmpdest;
		int icount;
	};

	internal_rsp_state *m_rsp_state;

	FILE *m_exec_output;

	uint32_t m_sr;
	uint32_t m_step_count;

	uint32_t m_ppc;
	uint32_t m_nextpc;

	address_space *m_program;
protected:
	direct_read_data *m_direct;

private:
	std::unique_ptr<rsp_cop2>    m_cop2;

	uint32_t *m_dmem32;
	uint16_t *m_dmem16;
	uint8_t *m_dmem8;

	uint32_t *m_imem32;
	uint16_t *m_imem16;
	uint8_t *m_imem8;

	uint32_t m_debugger_temp;
	bool m_isdrc;

	devcb_read32 m_dp_reg_r_func;
	devcb_write32 m_dp_reg_w_func;
	devcb_read32 m_sp_reg_r_func;
	devcb_write32 m_sp_reg_w_func;
	devcb_write32 m_sp_set_status_func;

	uint8_t READ8(uint32_t address);
	uint16_t READ16(uint32_t address);
	uint32_t READ32(uint32_t address);
	void WRITE8(uint32_t address, uint8_t data);
	void WRITE16(uint32_t address, uint16_t data);
	void WRITE32(uint32_t address, uint32_t data);
	uint32_t get_cop0_reg(int reg);
	void set_cop0_reg(int reg, uint32_t data);
	void load_fast_iregs(drcuml_block *block);
	void save_fast_iregs(drcuml_block *block);
	uint8_t DM_READ8(uint32_t address);
	uint16_t DM_READ16(uint32_t address);
	uint32_t DM_READ32(uint32_t address);
	void DM_WRITE8(uint32_t address, uint8_t data);
	void DM_WRITE16(uint32_t address, uint16_t data);
	void DM_WRITE32(uint32_t address, uint32_t data);
	void rspcom_init();
	void execute_run_drc();
	void code_flush_cache();
	void code_compile_block(offs_t pc);
	void static_generate_entry_point();
	void static_generate_nocode_handler();
	void static_generate_out_of_cycles();
	void static_generate_memory_accessor(int size, int iswrite, const char *name, uml::code_handle *&handleptr);
	void generate_update_cycles(drcuml_block *block, compiler_state *compiler, uml::parameter param, bool allow_exception);
	void generate_checksum_block(drcuml_block *block, compiler_state *compiler, const opcode_desc *seqhead, const opcode_desc *seqlast);
	void generate_sequence_instruction(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void generate_delay_slot_and_branch(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc, uint8_t linkreg);
	void generate_branch(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	bool generate_opcode(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	bool generate_special(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	bool generate_regimm(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	bool generate_cop0(drcuml_block *block, compiler_state *compiler, const opcode_desc *desc);
	void log_add_disasm_comment(drcuml_block *block, uint32_t pc, uint32_t op);
};


DECLARE_DEVICE_TYPE(RSP, rsp_device)

extern offs_t rsp_dasm_one(std::ostream &stream, offs_t pc, uint32_t op);


#endif // MAME_CPU_RSP_RSP_H
