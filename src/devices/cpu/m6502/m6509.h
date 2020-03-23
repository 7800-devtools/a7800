// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/***************************************************************************

    m6509.h

    6502 with banking and extended address bus

***************************************************************************/

#ifndef MAME_CPU_M6502_M6509_H
#define MAME_CPU_M6502_M6509_H

#include "m6502.h"

class m6509_device : public m6502_device {
public:
	m6509_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static const disasm_entry disasm_entries[0x100];

	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;
	virtual void do_exec_full() override;
	virtual void do_exec_partial() override;

protected:
	class mi_6509_normal : public memory_interface {
	public:
		m6509_device *base;

		mi_6509_normal(m6509_device *base);
		virtual ~mi_6509_normal() {}
		virtual uint8_t read(uint16_t adr) override;
		virtual uint8_t read_9(uint16_t adr) override;
		virtual uint8_t read_sync(uint16_t adr) override;
		virtual uint8_t read_arg(uint16_t adr) override;
		virtual void write(uint16_t adr, uint8_t val) override;
		virtual void write_9(uint16_t adr, uint8_t val) override;
	};

	class mi_6509_nd : public mi_6509_normal {
	public:
		mi_6509_nd(m6509_device *base);
		virtual ~mi_6509_nd() {}
		virtual uint8_t read_sync(uint16_t adr) override;
		virtual uint8_t read_arg(uint16_t adr) override;
	};

	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void state_export(const device_state_entry &entry) override;

	uint32_t XPC;

	uint8_t bank_i, bank_y;

	uint8_t bank_i_r() { return bank_i; }
	uint8_t bank_y_r() { return bank_y; }
	void bank_i_w(uint8_t data) { bank_i = data; }
	void bank_y_w(uint8_t data) { bank_y = data; }

	uint32_t adr_in_bank_i(uint16_t adr) { return adr | ((bank_i & 0xf) << 16); }
	uint32_t adr_in_bank_y(uint16_t adr) { return adr | ((bank_y & 0xf) << 16); }

#define O(o) void o ## _full(); void o ## _partial()

	// 6509 opcodes
	O(lda_9_idy);
	O(sta_9_idy);

#undef O
};

enum {
	M6509_IRQ_LINE = m6502_device::IRQ_LINE,
	M6509_NMI_LINE = m6502_device::NMI_LINE,
	M6509_SET_OVERFLOW = m6502_device::V_LINE
};

enum {
	M6509_BI = M6502_IR+1,
	M6509_BY
};

DECLARE_DEVICE_TYPE(M6509, m6509_device)

#endif // MAME_CPU_M6502_M6509_H
