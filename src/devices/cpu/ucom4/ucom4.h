// license:BSD-3-Clause
// copyright-holders:hap
/*

  NEC uCOM-4 MCU family cores

*/

#ifndef MAME_CPU_UCOM4_UCOM4_H
#define MAME_CPU_UCOM4_UCOM4_H

#pragma once


// I/O ports setup
#define MCFG_UCOM4_READ_A_CB(_devcb) \
	devcb = &ucom4_cpu_device::set_read_a_callback(*device, DEVCB_##_devcb);

#define MCFG_UCOM4_READ_B_CB(_devcb) \
	devcb = &ucom4_cpu_device::set_read_b_callback(*device, DEVCB_##_devcb);

#define MCFG_UCOM4_READ_C_CB(_devcb) \
	devcb = &ucom4_cpu_device::set_read_c_callback(*device, DEVCB_##_devcb);
#define MCFG_UCOM4_WRITE_C_CB(_devcb) \
	devcb = &ucom4_cpu_device::set_write_c_callback(*device, DEVCB_##_devcb);

#define MCFG_UCOM4_READ_D_CB(_devcb) \
	devcb = &ucom4_cpu_device::set_read_d_callback(*device, DEVCB_##_devcb);
#define MCFG_UCOM4_WRITE_D_CB(_devcb) \
	devcb = &ucom4_cpu_device::set_write_d_callback(*device, DEVCB_##_devcb);

#define MCFG_UCOM4_WRITE_E_CB(_devcb) \
	devcb = &ucom4_cpu_device::set_write_e_callback(*device, DEVCB_##_devcb);

#define MCFG_UCOM4_WRITE_F_CB(_devcb) \
	devcb = &ucom4_cpu_device::set_write_f_callback(*device, DEVCB_##_devcb);

#define MCFG_UCOM4_WRITE_G_CB(_devcb) \
	devcb = &ucom4_cpu_device::set_write_g_callback(*device, DEVCB_##_devcb);

#define MCFG_UCOM4_WRITE_H_CB(_devcb) \
	devcb = &ucom4_cpu_device::set_write_h_callback(*device, DEVCB_##_devcb);

#define MCFG_UCOM4_WRITE_I_CB(_devcb) \
	devcb = &ucom4_cpu_device::set_write_i_callback(*device, DEVCB_##_devcb);

enum
{
	NEC_UCOM4_PORTA = 0,
	NEC_UCOM4_PORTB,
	NEC_UCOM4_PORTC,
	NEC_UCOM4_PORTD,
	NEC_UCOM4_PORTE,
	NEC_UCOM4_PORTF,
	NEC_UCOM4_PORTG,
	NEC_UCOM4_PORTH,
	NEC_UCOM4_PORTI
};

enum
{
	NEC_UCOM43 = 0,
	NEC_UCOM44,
	NEC_UCOM45
};


// pinout reference

/*
            _______   _______
    CL1  1 |*      \_/       | 42 CL0
    PC0  2 |                 | 41 Vgg
    PC1  3 |                 | 40 PB3
    PC2  4 |                 | 39 PB2
    PC3  5 |                 | 38 PB1
   /INT  6 |                 | 37 PB0
  RESET  7 |                 | 36 PA3
    PD0  8 |                 | 35 PA2
    PD1  9 |     uPD546      | 34 PA1
    PD2 10 |     uPD552      | 33 PA0
    PD3 11 |     uPD553      | 32 PI2
    PE0 12 |     uPD650*     | 31 PI1
    PE1 13 |                 | 30 PI0
    PE2 14 |                 | 29 PH3
    PE3 15 |                 | 28 PH2
    PF0 16 |                 | 27 PH1
    PF1 17 |                 | 26 PH0
    PF2 18 |                 | 25 PG3
    PF3 19 |                 | 24 PG2
   TEST 20 |                 | 23 PG1
    Vss 21 |_________________| 22 PG0

  *: pin 21 is Vcc, pin 41 is Vss

*/


class ucom4_cpu_device : public cpu_device
{
public:
	// static configuration helpers
	template <class Object> static devcb_base &set_read_a_callback(device_t &device, Object &&cb) { return downcast<ucom4_cpu_device &>(device).m_read_a.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_read_b_callback(device_t &device, Object &&cb) { return downcast<ucom4_cpu_device &>(device).m_read_b.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_read_c_callback(device_t &device, Object &&cb) { return downcast<ucom4_cpu_device &>(device).m_read_c.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_read_d_callback(device_t &device, Object &&cb) { return downcast<ucom4_cpu_device &>(device).m_read_d.set_callback(std::forward<Object>(cb)); }

	template <class Object> static devcb_base &set_write_c_callback(device_t &device, Object &&cb) { return downcast<ucom4_cpu_device &>(device).m_write_c.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_write_d_callback(device_t &device, Object &&cb) { return downcast<ucom4_cpu_device &>(device).m_write_d.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_write_e_callback(device_t &device, Object &&cb) { return downcast<ucom4_cpu_device &>(device).m_write_e.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_write_f_callback(device_t &device, Object &&cb) { return downcast<ucom4_cpu_device &>(device).m_write_f.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_write_g_callback(device_t &device, Object &&cb) { return downcast<ucom4_cpu_device &>(device).m_write_g.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_write_h_callback(device_t &device, Object &&cb) { return downcast<ucom4_cpu_device &>(device).m_write_h.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_write_i_callback(device_t &device, Object &&cb) { return downcast<ucom4_cpu_device &>(device).m_write_i.set_callback(std::forward<Object>(cb)); }

protected:
	// construction/destruction
	ucom4_cpu_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock, int family, int stack_levels, int prgwidth, address_map_constructor program, int datawidth, address_map_constructor data);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_execute_interface overrides
	virtual u64 execute_clocks_to_cycles(u64 clocks) const override { return (clocks + 4 - 1) / 4; } // 4 cycles per machine cycle
	virtual u64 execute_cycles_to_clocks(u64 cycles) const override { return (cycles * 4); } // "
	virtual u32 execute_min_cycles() const override { return 1; }
	virtual u32 execute_max_cycles() const override { return 2; }
	virtual u32 execute_input_lines() const override { return 1; }
	virtual void execute_set_input(int line, int state) override;
	virtual void execute_run() override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// device_disasm_interface overrides
	virtual u32 disasm_min_opcode_bytes() const override { return 1; }
	virtual u32 disasm_max_opcode_bytes() const override { return 2; }
	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const u8 *oprom, const u8 *opram, u32 options) override;

	// device_state_interface overrides
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	address_space_config m_program_config;
	address_space_config m_data_config;
	address_space *m_program;
	address_space *m_data;

	int m_prgwidth;
	int m_datawidth;
	int m_prgmask;
	int m_datamask;
	int m_family;        // MCU family (43/44/45)
	int m_stack_levels;  // number of callstack levels
	u16 m_stack[3];      // max 3
	u8 m_port_out[0x10]; // last value written to output port
	u8 m_op;
	u8 m_prev_op;        // previous opcode
	u8 m_arg;            // opcode argument for 2-byte opcodes
	u8 m_bitmask;        // opcode bit argument
	bool m_skip;         // skip next opcode
	int m_icount;
	emu_timer *m_timer;

	u16 m_pc;            // program counter
	u16 m_prev_pc;
	u8 m_acc;            // 4-bit accumulator
	u8 m_dpl;            // 4-bit data pointer low (RAM x)
	u8 m_dph;            // 4-bit(?) data pointer high (RAM y)
	u8 m_dph_mask;
	u8 m_carry_f;        // carry flag
	u8 m_carry_s_f;      // carry save flag
	u8 m_timer_f;        // timer out flag
	u8 m_int_f;          // interrupt flag
	u8 m_inte_f;         // interrupt enable flag
	int m_int_line;      // interrupt pin state

	// i/o handlers
	devcb_read8 m_read_a;
	devcb_read8 m_read_b;
	devcb_read8 m_read_c;
	devcb_read8 m_read_d;

	devcb_write8 m_write_c;
	devcb_write8 m_write_d;
	devcb_write8 m_write_e;
	devcb_write8 m_write_f;
	devcb_write8 m_write_g;
	devcb_write8 m_write_h;
	devcb_write8 m_write_i;

	virtual u8 input_r(int index);
	virtual void output_w(int index, u8 data);

	// misc internal helpers
	void increment_pc();
	void fetch_arg();
	void do_interrupt();

	u8 ram_r();
	void ram_w(u8 data);
	void pop_stack();
	void push_stack();

	bool check_op_43();
	TIMER_CALLBACK_MEMBER( simple_timer_cb );
	u8 ucom43_reg_r(int index);
	void ucom43_reg_w(int index, u8 data);

	// opcode handlers
	void op_illegal();

	void op_li();
	void op_lm();
	void op_ldi();
	void op_ldz();
	void op_s();
	void op_tal();
	void op_tla();

	void op_xm();
	void op_xmi();
	void op_xmd();
	void op_ad();
	void op_adc();
	void op_ads();
	void op_daa();
	void op_das();

	void op_exl();
	void op_cma();
	void op_cia();
	void op_clc();
	void op_stc();
	void op_tc();
	void op_inc();
	void op_dec();
	void op_ind();
	void op_ded();

	void op_rmb();
	void op_smb();
	void op_reb();
	void op_seb();
	void op_rpb();
	void op_spb();
	void op_jmpcal();
	void op_jcp();
	void op_jpa();
	void op_czp();
	void op_rt();
	void op_rts();

	void op_ci();
	void op_cm();
	void op_cmb();
	void op_tab();
	void op_cli();
	void op_tmb();
	void op_tpa();
	void op_tpb();

	void op_tit();
	void op_ia();
	void op_ip();
	void op_oe();
	void op_op();
	void op_ocd();
	void op_nop();

	void op_taw();
	void op_taz();
	void op_thx();
	void op_tly();
	void op_xaw();
	void op_xaz();
	void op_xhr();
	void op_xhx();
	void op_xls();
	void op_xly();
	void op_xc();

	void op_sfb();
	void op_rfb();
	void op_fbt();
	void op_fbf();
	void op_rar();
	void op_inm();
	void op_dem();
	void op_stm();
	void op_ttm();
	void op_ei();
	void op_di();
};


class upd546_cpu_device : public ucom4_cpu_device
{
public:
	upd546_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);
};


class upd553_cpu_device : public ucom4_cpu_device
{
public:
	upd553_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);
};


class upd557l_cpu_device : public ucom4_cpu_device
{
public:
	upd557l_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	virtual u8 input_r(int index) override;
	virtual void output_w(int index, u8 data) override;
};


class upd650_cpu_device : public ucom4_cpu_device
{
public:
	upd650_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);
};


class upd552_cpu_device : public ucom4_cpu_device
{
public:
	upd552_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);
};



DECLARE_DEVICE_TYPE(NEC_D546,  upd546_cpu_device)
DECLARE_DEVICE_TYPE(NEC_D553,  upd553_cpu_device)
DECLARE_DEVICE_TYPE(NEC_D557L, upd557l_cpu_device)
DECLARE_DEVICE_TYPE(NEC_D650,  upd650_cpu_device)
DECLARE_DEVICE_TYPE(NEC_D552,  upd552_cpu_device)

#endif // MAME_CPU_UCOM4_UCOM4_H
