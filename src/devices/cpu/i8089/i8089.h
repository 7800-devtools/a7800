// license:GPL-2.0+
// copyright-holders:Dirk Best,Carl
/***************************************************************************

    Intel 8089 I/O Processor

***************************************************************************/

#ifndef MAME_CPU_I8089_I8089_H
#define MAME_CPU_I8089_I8089_H

#pragma once

#ifdef _MSC_VER
// MSVC seems to want to actually instantiate templates when it gets an extern template declaration, effectively defeating the purpose of extern template declatations altogether
// In this case it causes a problem because the required_device template can't be instantiated for the incomplete i8089_channel_device type
#include "i8089_channel.h"
#endif


//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_I8089_DATABUS_WIDTH(_databus_width) \
	i8089_device::set_databus_width(*device, _databus_width);

#define MCFG_I8089_SINTR1(_sintr1) \
	devcb = &downcast<i8089_device *>(device)->set_sintr1_callback(DEVCB_##_sintr1);

#define MCFG_I8089_SINTR2(_sintr2) \
	devcb = &downcast<i8089_device *>(device)->set_sintr2_callback(DEVCB_##_sintr2);


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// forward declaration
class i8089_channel_device;

// ======================> i8089_device

class i8089_device : public cpu_device
{
	friend class i8089_channel_device;

public:
	// construction/destruction
	i8089_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// callbacks
	template <class Object> devcb_base &set_sintr1_callback(Object &&sintr1) { return m_write_sintr1.set_callback(std::forward<Object>(sintr1)); }
	template <class Object> devcb_base &set_sintr2_callback(Object &&sintr2) { return m_write_sintr2.set_callback(std::forward<Object>(sintr2)); }

	// static configuration helpers
	static void set_databus_width(device_t &device, uint8_t databus_width) { downcast<i8089_device &>(device).m_databus_width = databus_width; }

	// input lines
	DECLARE_WRITE_LINE_MEMBER( ca_w );
	DECLARE_WRITE_LINE_MEMBER( sel_w ) { m_sel = state; }
	DECLARE_WRITE_LINE_MEMBER( drq1_w );
	DECLARE_WRITE_LINE_MEMBER( drq2_w );
	DECLARE_WRITE_LINE_MEMBER( ext1_w );
	DECLARE_WRITE_LINE_MEMBER( ext2_w );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_config_complete() override;
	virtual void device_reset() override;

	// device_execute_interface overrides
	virtual void execute_run() override;

	int m_icount;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	address_space_config m_program_config;
	address_space_config m_io_config;

	// device_disasm_interface overrides
	virtual uint32_t disasm_min_opcode_bytes() const override { return 1; }
	virtual uint32_t disasm_max_opcode_bytes() const override { return 7; }
	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;

	// device_state_interface overrides
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;

private:
	bool sysbus_width() const { return BIT(m_sysbus, 0); }
	bool remotebus_width() const { return BIT(m_soc, 0); }
	bool request_grant() const { return BIT(m_soc, 1); }

	// internal communication
	DECLARE_WRITE_LINE_MEMBER( ch1_sintr_w ) { m_write_sintr1(state); }
	DECLARE_WRITE_LINE_MEMBER( ch2_sintr_w ) { m_write_sintr2(state); }

	uint8_t read_byte(bool space, offs_t address);
	uint16_t read_word(bool space, offs_t address);
	void write_byte(bool space, offs_t address, uint8_t data);
	void write_word(bool space, offs_t address, uint16_t data);

	required_device<i8089_channel_device> m_ch1;
	required_device<i8089_channel_device> m_ch2;

	devcb_write_line m_write_sintr1;
	devcb_write_line m_write_sintr2;

	void initialize();

	uint8_t m_databus_width;
	address_space *m_mem;
	address_space *m_io;

	// register indexes for the debugger state
	enum
	{
		SYSBUS,
		SCB,
		SOC,
		DIVIDER1,
		CH1_GA, CH1_GB, CH1_GC,
		CH1_TP, CH1_BC, CH1_IX,
		CH1_CC, CH1_MC, CH1_CP,
		CH1_PP, CH1_PSW,
		DIVIDER2,
		CH2_GA, CH2_GB, CH2_GC,
		CH2_TP, CH2_BC, CH2_IX,
		CH2_CC, CH2_MC, CH2_CP,
		CH2_PP, CH2_PSW
	};

	// system configuration
	uint8_t m_sysbus;
	offs_t m_scb;
	uint8_t m_soc;

	bool m_initialized;
	bool m_master;

	// task pointer for the currently executing channel
	offs_t m_current_tp;

	// state of input pins
	int m_ca;
	int m_sel;
	bool m_last_chan;
};


// device type definition
DECLARE_DEVICE_TYPE(I8089, i8089_device)

#endif // MAME_CPU_I8089_I8089_H
