// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/*
    Manchester Small-Scale Experimental Machine (SSEM) emulator

    Written by Ryan Holtz
*/

#ifndef MAME_CPU_SSEM_SSEM_H
#define MAME_CPU_SSEM_SSEM_H

#pragma once

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> ssem_device

// Used by core CPU interface
class ssem_device : public cpu_device
{
public:
	// construction/destruction
	ssem_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_stop() override;

	// device_execute_interface overrides
	virtual uint32_t execute_min_cycles() const override;
	virtual uint32_t execute_max_cycles() const override;
	virtual uint32_t execute_input_lines() const override;
	virtual void execute_run() override;
	virtual void execute_set_input(int inputnum, int state) override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// device_disasm_interface overrides
	virtual uint32_t disasm_min_opcode_bytes() const override;
	virtual uint32_t disasm_max_opcode_bytes() const override;
	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;

	// device_state_interface overrides
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	// address spaces
	const address_space_config m_program_config;

	// memory access
	inline uint32_t program_read32(uint32_t addr);
	inline void program_write32(uint32_t addr, uint32_t data);

	// CPU registers
	uint32_t m_pc;
	uint32_t m_shifted_pc;
	uint32_t m_a;
	uint32_t m_halt;

	// other internal states
	int m_icount;

	// address spaces
	address_space *m_program;
};

// device type definition
DECLARE_DEVICE_TYPE(SSEMCPU, ssem_device)

/***************************************************************************
    REGISTER ENUMERATION
***************************************************************************/

enum
{
	SSEM_PC = 1,
	SSEM_A,
	SSEM_HALT
};

CPU_DISASSEMBLE( ssem );

#endif // MAME_CPU_SSEM_SSEM_H
