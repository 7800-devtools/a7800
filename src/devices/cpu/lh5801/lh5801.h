// license:BSD-3-Clause
// copyright-holders:Peter Trauner
/*****************************************************************************
 *
 *   cpustate->h
 *   portable lh5801 emulator interface
 *
 *
 *****************************************************************************/
#ifndef MAME_CPU_LH5801_LH5801_H
#define MAME_CPU_LH5801_LH5801_H

#pragma once

/*
lh5801

little endian

ph, pl p
sh, sl s

xh, xl x
yh, yl y
uh, ul u

a A

0 0 0 H V Z IE C

TM 9bit polynomial?

pu pv disp flipflops

bf flipflop (break key connected)

    me0, me1 chip select for 2 64kb memory blocks

in0-in7 input pins

    mi maskable interrupt input (fff8/9)
    timer fffa/b
    nmi non .. (fffc/d)
    reset fffe/f
e ?



lh5811 chip
pa 8bit io
pb 8bit io
pc 8bit
*/


// input lines
enum
{
	LH5801_LINE_MI     //maskable interrupt
};


#define MCFG_LH5801_IN(_devcb) \
	devcb = &lh5801_cpu_device::set_in_func(*device, DEVCB_##_devcb);


class lh5801_cpu_device :  public cpu_device
{
public:
	// construction/destruction
	lh5801_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration helpers
	template <class Object> static devcb_base &set_in_func(device_t &device, Object &&cb) { return downcast<lh5801_cpu_device &>(device).m_in_func.set_callback(std::forward<Object>(cb)); }

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_execute_interface overrides
	virtual uint32_t execute_min_cycles() const override { return 2; }
	virtual uint32_t execute_max_cycles() const override { return 19; }
	virtual uint32_t execute_input_lines() const override { return 2; }
	virtual uint32_t execute_default_irq_vector() const override { return 0; }
	virtual void execute_run() override;
	virtual void execute_set_input(int inputnum, int state) override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// device_state_interface overrides
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	// device_disasm_interface overrides
	virtual uint32_t disasm_min_opcode_bytes() const override { return 1; }
	virtual uint32_t disasm_max_opcode_bytes() const override { return 5; }
	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;

private:
	address_space_config m_program_config;
	address_space_config m_io_config;

	devcb_read8 m_in_func;

	address_space *m_program;         //ME0
	address_space *m_io;              //ME1
	direct_read_data *m_direct;

	PAIR m_s;
	PAIR m_p;
	PAIR m_u;
	PAIR m_x;
	PAIR m_y;
	int m_tm; //9 bit

	uint8_t m_t, m_a;

	int m_bf;
	int m_dp;
	int m_pu;
	int m_pv;

	uint16_t m_oldpc;

	int m_irq_state;

	uint8_t m_ir_flipflop[3];   //interrupt request flipflop: IR0, IR1, IR2
	int m_lines_status[2];    //MI and NMI lines status

	int m_idle;
	int m_icount;

	void check_irq();
	void lh5801_instruction_fd();
	void lh5801_instruction();
	uint8_t lh5801_add_generic(int left, int right, int carry);
	uint16_t lh5801_readop_word();
	void lh5801_adc(uint8_t data);
	void lh5801_add_mem(address_space &space, int addr, uint8_t data);
	void lh5801_adr(PAIR *reg);
	void lh5801_sbc(uint8_t data);
	void lh5801_cpa(uint8_t a, uint8_t b);
	uint8_t lh5801_decimaladd_generic(int left, int right, int carry);
	void lh5801_dca(uint8_t data);
	void lh5801_dcs(uint8_t data);
	void lh5801_and(uint8_t data);
	void lh5801_and_mem(address_space &space, int addr, uint8_t data);
	void lh5801_bit(uint8_t a, uint8_t b);
	void lh5801_eor(uint8_t data);
	void lh5801_ora(uint8_t data);
	void lh5801_ora_mem(address_space &space, int addr, uint8_t data);
	void lh5801_lda(uint8_t data);
	void lh5801_lde(PAIR *reg);
	void lh5801_sde(PAIR *reg);
	void lh5801_lin(PAIR *reg);
	void lh5801_sin(PAIR *reg);
	void lh5801_dec(uint8_t *adr);
	void lh5801_inc(uint8_t *adr);
	void lh5801_pop();
	void lh5801_pop_word(PAIR *reg);
	void lh5801_rtn();
	void lh5801_rti();
	void lh5801_push(uint8_t data);
	void lh5801_push_word(uint16_t data);
	void lh5801_jmp(uint16_t adr);
	void lh5801_branch_plus(int doit);
	void lh5801_branch_minus(int doit);
	void lh5801_lop();
	void lh5801_sjp();
	void lh5801_vector(int doit, int nr);
	void lh5801_aex();
	void lh5801_drl(address_space &space, int adr);
	void lh5801_drr(address_space &space, int adr);
	void lh5801_rol();
	void lh5801_ror();
	void lh5801_shl();
	void lh5801_shr();
	void lh5801_am(int value);
	void lh5801_ita();

};


DECLARE_DEVICE_TYPE(LH5801, lh5801_cpu_device)


#endif // MAME_CPU_LH5801_LH5801_H
