// license:BSD-3-Clause
// copyright-holders:Aaron Giles

#ifndef MAME_CPU_M6800_M6801_H
#define MAME_CPU_M6800_M6801_H

#pragma once

#include "cpu/m6800/m6800.h"


enum
{
	M6801_IRQ_LINE = M6800_IRQ_LINE,
	M6801_TIN_LINE,                 /* P20/Tin Input Capture line (eddge sense)     */
									/* Active eddge is selecrable by internal reg.  */
									/* raise eddge : CLEAR_LINE  -> ASSERT_LINE     */
									/* fall  eddge : ASSERT_LINE -> CLEAR_LINE      */
									/* it is usuali to use PULSE_LINE state         */
	M6801_SC1_LINE
};

enum
{
	M6803_IRQ_LINE = M6800_IRQ_LINE
};

enum
{
	HD6301_IRQ_LINE = M6800_IRQ_LINE
};

enum
{
	M6801_MODE_0 = 0,
	M6801_MODE_1,
	M6801_MODE_2,
	M6801_MODE_3,
	M6801_MODE_4,
	M6801_MODE_5,
	M6801_MODE_6,
	M6801_MODE_7
};

enum
{
	M6801_PORT1 = 0x100,
	M6801_PORT2,
	M6801_PORT3,
	M6801_PORT4
};


#define MCFG_M6801_SC2(_devcb) \
	devcb = &m6801_cpu_device::set_out_sc2_func(*device, DEVCB_##_devcb);
#define MCFG_M6801_SER_TX(_devcb) \
	devcb = &m6801_cpu_device::set_out_sertx_func(*device, DEVCB_##_devcb);


class m6801_cpu_device : public m6800_cpu_device
{
public:
	m6801_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration helpers
	template<class _Object> static devcb_base &set_out_sc2_func(device_t &device, _Object object) { return downcast<m6801_cpu_device &>(device).m_out_sc2_func.set_callback(object); }
	template<class _Object> static devcb_base &set_out_sertx_func(device_t &device, _Object object) { return downcast<m6801_cpu_device &>(device).m_out_sertx_func.set_callback(object); }

	DECLARE_READ8_MEMBER( m6801_io_r );
	DECLARE_WRITE8_MEMBER( m6801_io_w );

	void m6801_clock_serial();

protected:
	m6801_cpu_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, const m6800_cpu_device::op_func *insn, const uint8_t *cycles, address_map_constructor internal = nullptr);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_execute_interface overrides
	virtual uint64_t execute_clocks_to_cycles(uint64_t clocks) const override { return (clocks + 4 - 1) / 4; }
	virtual uint64_t execute_cycles_to_clocks(uint64_t cycles) const override { return (cycles * 4); }
	virtual void execute_set_input(int inputnum, int state) override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// device_disasm_interface overrides
	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;

	address_space_config m_io_config;

	devcb_write_line m_out_sc2_func;
	devcb_write_line m_out_sertx_func;

	address_space *m_io;

	/* internal registers */
	uint8_t   m_port1_ddr;
	uint8_t   m_port2_ddr;
	uint8_t   m_port3_ddr;
	uint8_t   m_port4_ddr;
	uint8_t   m_port1_data;
	uint8_t   m_port2_data;
	uint8_t   m_port3_data;
	uint8_t   m_port4_data;
	uint8_t   m_p3csr;          // Port 3 Control/Status Register
	uint8_t   m_tcsr;           /* Timer Control and Status Register */
	uint8_t   m_pending_tcsr;   /* pending IRQ flag for clear IRQflag process */
	uint8_t   m_irq2;           /* IRQ2 flags */
	uint8_t   m_ram_ctrl;
	PAIR    m_counter;        /* free running counter */
	PAIR    m_output_compare; /* output compare       */
	uint16_t  m_input_capture;  /* input capture        */
	int     m_p3csr_is3_flag_read;
	int     m_port3_latched;

	uint8_t   m_trcsr, m_rmcr, m_rdr, m_tdr, m_rsr, m_tsr;
	int     m_rxbits, m_txbits, m_txstate, m_trcsr_read_tdre, m_trcsr_read_orfe, m_trcsr_read_rdrf, m_tx, m_ext_serclock;
	bool    m_use_ext_serclock;
	int     m_port2_written;

	int     m_latch09;

	PAIR    m_timer_over;
	emu_timer *m_sci_timer;

	/* point of next timer event */
	uint32_t m_timer_next;

	int     m_sc1_state;

	static const uint8_t cycles_6803[256];
	static const uint8_t cycles_63701[256];
	static const op_func m6803_insn[256];
	static const op_func hd63701_insn[256];

	virtual void m6800_check_irq2() override;
	virtual void increment_counter(int amount) override;
	virtual void EAT_CYCLES() override;
	virtual void CLEANUP_COUNTERS() override;

	void check_timer_event();
	void set_rmcr(uint8_t data);
	void write_port2();
	int m6800_rx();
	void serial_transmit();
	void serial_receive();
	TIMER_CALLBACK_MEMBER( sci_tick );
	void set_os3(int state);
};


class m6803_cpu_device : public m6801_cpu_device
{
public:
	m6803_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;
};


class hd6301_cpu_device : public m6801_cpu_device
{
public:
	hd6301_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	hd6301_cpu_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;
};


class hd63701_cpu_device : public m6801_cpu_device
{
public:
	hd63701_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;

	virtual void TAKE_TRAP() override;
};


// DP-40 package: HD6303RP,  HD63A03RP,  HD63B03RP,
// FP-54 package: HD6303RF,  HD63A03RF,  HD63B03RF,
// CG-40 package: HD6303RCG, HD63A03RCG, HD63B03RCG,
// Not fully emulated yet
class hd6303r_cpu_device : public hd6301_cpu_device
{
public:
	hd6303r_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


// DP-64S package: HD6303YP,  HD63A03YP,  HD63B03YP,  HD63C03YP
// FP-64  package: HD6303YF,  HD63A03YF,  HD63B03YF,  HD63C03YF
// FP-64A package: HD6303YH,  HD63A03YH,  HD63B03YH,  HD63C03YH
// CP-68  package: HD6303YCP, HD63A03YCP, HD63B03YCP, HD63C03YCP
// Not fully emulated yet
class hd6303y_cpu_device : public hd6301_cpu_device
{
public:
	hd6303y_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


DECLARE_DEVICE_TYPE(M6801, m6801_cpu_device)
DECLARE_DEVICE_TYPE(M6803, m6803_cpu_device)
DECLARE_DEVICE_TYPE(HD6301, hd6301_cpu_device)
DECLARE_DEVICE_TYPE(HD63701, hd63701_cpu_device)
DECLARE_DEVICE_TYPE(HD6303R, hd6303r_cpu_device)
DECLARE_DEVICE_TYPE(HD6303Y, hd6303y_cpu_device)

#endif // MAME_CPU_M6800_M6801_H
