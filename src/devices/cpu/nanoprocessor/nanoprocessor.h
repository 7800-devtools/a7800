// license:BSD-3-Clause
// copyright-holders:F. Ulivi
//
// *****************************
// Emulator for HP nanoprocessor
// *****************************
//
// Nanoprocessor is a very simple microcontroller developed by HP in 1974 for
// its own internal use. It's used, for example, in HP9845 built-in printer, in
// 98034 & 98035 modules, etc.
// Here's a summary of its features:
// * 8-bit word size
// * 1 Accumulator
// * 16 general purpose registers
// * 11-bit Program Counter
// * 2048 bytes of program space
// * 16 input 8-bit ports (DS0-DS15)
// * 15 output 8-bit ports (DS0-DS14)
// * 8 "direct control" I/O lines that can be individually tested/set/cleared
// * 1 interrupt input with 8-bit vector
// * One level nesting of subroutine call through a 11-bit subroutine stack register (SSR)
// * One level nesting of interruption through a 11-bit interrupt stack register (ISR)
// * No built-in ROM or RAM
// * All instructions execute in 2 clock cycles
// * NO arithmetic capabilities, except binary/BCD increment/decrement, bit
//   manipulation, left/right shift and comparison between A and R0
// * Register access can be indexed to use GP registers as a "poor man's RAM"
//                            _____   _____
//                   PA0   1 |*    \_/     | 40  VGG
//                   PA1   2 |             | 39  VDD
//                   PA2   3 |             | 38  VBG
//                   PA3   4 |             | 37  DC0
//                   PA4   5 |             | 36  DC1
//                   PA5   6 |             | 35  DC2
//                   PA6   7 |             | 34  DC3
//                   PA7   8 |             | 33  DC4
//                   PA8   9 |             | 32  DC5
//                   PA9  10 |Nanoprocessor| 31  DC6
//                  PA10  11 |             | 30  DC7/INT ENA
//                   DS3  12 |             | 29  _INT
//                   DS2  13 |             | 28  ACK
//                   DS1  14 |             | 27  CLK
//                   DS0  15 |             | 26  STM
//                  _R/W  16 |             | 25  DB7
//                   GND  17 |             | 24  DB6
//                   DB0  18 |             | 23  DB5
//                   DB1  19 |             | 22  DB4
//                   DB2  20 |_____________| 21  DB3
//
// The HP manual of Nanoprocessor is available here:
// http://www.hp9845.net/9845/downloads/manuals/Nanoprocessor.pdf
// Thanks to anyone who made the manual available.
#ifndef MAME_CPU_NANOPROCESSOR_NANOPROCESSOR_H
#define MAME_CPU_NANOPROCESSOR_NANOPROCESSOR_H

#pragma once

constexpr unsigned HP_NANO_IE_DC   = 7;   // DC line used as interrupt enable/mask (DC7)

// DC changed callback
// The callback receives a 8-bit word holding the state of all DC lines.
// DC0 is in bit 0, DC1 in bit 1 and so on.
// Keep in mind that DC7 usually masks the interrupt signal.
#define MCFG_HP_NANO_DC_CHANGED(_devcb)                                 \
	devcb = &hp_nanoprocessor_device::set_dc_changed_func(*device , DEVCB_##_devcb);

// Callback to read the input state of DC lines
// All lines that are not in input are to be reported at "1"
#define MCFG_HP_NANO_READ_DC_CB(_devcb)                                 \
	devcb = &hp_nanoprocessor_device::set_read_dc_func(*device , DEVCB_##_devcb);

class hp_nanoprocessor_device : public cpu_device
{
public:
	hp_nanoprocessor_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_dc_changed_func(device_t &device, Object &&cb) { return downcast<hp_nanoprocessor_device &>(device).m_dc_changed_func.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_read_dc_func(device_t &device, Object &&cb) { return downcast<hp_nanoprocessor_device &>(device).m_read_dc_func.set_callback(std::forward<Object>(cb)); }

	// device_execute_interface overrides
	virtual uint32_t execute_min_cycles() const override { return 2; }
	// 3 cycles is for int. acknowledge + 1 instruction
	virtual uint32_t execute_max_cycles() const override { return 3; }
	virtual uint32_t execute_input_lines() const override { return 1; }
	virtual uint32_t execute_default_irq_vector() const override { return 0xff; }

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// device_state_interface overrides
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	// device_disasm_interface overrides
	virtual uint32_t disasm_min_opcode_bytes() const override { return 1; }
	virtual uint32_t disasm_max_opcode_bytes() const override { return 2; }
	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;

private:
	static constexpr unsigned HP_NANO_REGS    = 16;  // Number of GP registers
	static constexpr unsigned HP_NANO_PC_MASK = 0x7ff;   // Mask of PC meaningful bits: 11 bits available
	static constexpr unsigned HP_NANO_DC_NO   = 8;   // Number of direct control lines (DC7 is typically used as interrupt mask)

	devcb_write8 m_dc_changed_func;
	devcb_read8 m_read_dc_func;
	int m_icount;

	// State of processor
	uint8_t  m_reg_A;   // Accumulator
	uint8_t  m_reg_R[ HP_NANO_REGS ];   // General purpose registers
	uint16_t m_reg_PA;  // Program counter ("Program Address" in HP doc)
	uint16_t m_reg_SSR; // Subroutine stack register
	uint16_t m_reg_ISR; // Interrupt stack register
	uint16_t m_flags;   // Flags: extend flag (E) & direct control lines (DC0-7)

	address_space_config m_program_config;
	address_space_config m_io_config;

	address_space *m_program;
	direct_read_data *m_direct;
	address_space *m_io;

	// device_t overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	virtual void execute_run() override;
	virtual void execute_set_input(int linenum, int state) override;

	void execute_one(uint8_t opcode);
	uint16_t pa_offset(unsigned off) const;
	uint8_t fetch(void);
	void skip(void);
	void dc_update(void);
	void dc_set(unsigned bit_no);
	void dc_clr(unsigned bit_no);
};

extern const device_type HP_NANOPROCESSOR;

#endif /* _NANOPROCESSOR_H_ */
