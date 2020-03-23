// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/***************************************************************************

    m6510.h

    6502 with 6 i/o pins, also known as 8500

***************************************************************************/
#ifndef MAME_CPU_M6502_M6510_H
#define MAME_CPU_M6502_M6510_H

#pragma once

#include "m6502.h"

#define MCFG_M6510_PORT_CALLBACKS(_read, _write) \
	downcast<m6510_device *>(device)->set_callbacks(DEVCB_##_read, DEVCB_##_write);

#define MCFG_M6510_PORT_PULLS(_up, _down) \
	downcast<m6510_device *>(device)->set_pulls(_up, _down);

class m6510_device : public m6502_device {
public:
	m6510_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	uint8_t get_port();
	void set_pulls(uint8_t pullup, uint8_t pulldown);

	template<class _read, class _write> void set_callbacks(_read rd, _write wr) {
		read_port.set_callback(rd);
		write_port.set_callback(wr);
	}

	static const disasm_entry disasm_entries[0x100];

	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;
	virtual void do_exec_full() override;
	virtual void do_exec_partial() override;

protected:
	m6510_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	class mi_6510_normal : public memory_interface {
	public:
		m6510_device *base;

		mi_6510_normal(m6510_device *base);
		virtual ~mi_6510_normal() {}
		virtual uint8_t read(uint16_t adr) override;
		virtual uint8_t read_sync(uint16_t adr) override;
		virtual uint8_t read_arg(uint16_t adr) override;
		virtual void write(uint16_t adr, uint8_t val) override;
	};

	class mi_6510_nd : public mi_6510_normal {
	public:
		mi_6510_nd(m6510_device *base);
		virtual ~mi_6510_nd() {}
		virtual uint8_t read_sync(uint16_t adr) override;
		virtual uint8_t read_arg(uint16_t adr) override;
	};

	devcb_read8  read_port;
	devcb_write8 write_port;

	uint8_t pullup, floating, dir, port, drive;

	virtual void device_start() override;
	virtual void device_reset() override;

	uint8_t dir_r();
	void dir_w(uint8_t data);
	uint8_t port_r();
	void port_w(uint8_t data);

	void update_port();

#define O(o) void o ## _full(); void o ## _partial()

	// 6510 undocumented instructions in a C64 context
	// implementation follows what the test suites expect (usually an extra and)
	O(anc_10_imm);
	O(ane_10_imm);
	O(arr_10_imm);
	O(asr_10_imm);
	O(las_10_aby);
	O(lxa_10_imm);

#undef O
};

enum {
	M6510_IRQ_LINE = m6502_device::IRQ_LINE,
	M6510_NMI_LINE = m6502_device::NMI_LINE
};

DECLARE_DEVICE_TYPE(M6510, m6510_device)

#endif // MAME_CPU_M6502_M6510_H
