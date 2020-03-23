// license:BSD-3-Clause
// copyright-holders:hap
/*

  Mitsubishi MELPS 4 MCU family cores

*/

#ifndef MAME_CPU_MELPS4_MELPS4_H
#define MAME_CPU_MELPS4_MELPS4_H

#pragma once


// I/O ports setup

// K input or A/D input port, up to 16 pins
#define MCFG_MELPS4_READ_K_CB(_devcb) \
	devcb = &melps4_cpu_device::set_read_k_callback(*device, DEVCB_##_devcb);

// D discrete I/O port, up to 16 pins - offset 0-15 for bit, 16 for all pins clear
#define MCFG_MELPS4_READ_D_CB(_devcb) \
	devcb = &melps4_cpu_device::set_read_d_callback(*device, DEVCB_##_devcb);
#define MCFG_MELPS4_WRITE_D_CB(_devcb) \
	devcb = &melps4_cpu_device::set_write_d_callback(*device, DEVCB_##_devcb);

// 8-bit S generic I/O port
#define MCFG_MELPS4_READ_S_CB(_devcb) \
	devcb = &melps4_cpu_device::set_read_s_callback(*device, DEVCB_##_devcb);
#define MCFG_MELPS4_WRITE_S_CB(_devcb) \
	devcb = &melps4_cpu_device::set_write_s_callback(*device, DEVCB_##_devcb);

// 4-bit F generic I/O port
#define MCFG_MELPS4_READ_F_CB(_devcb) \
	devcb = &melps4_cpu_device::set_read_f_callback(*device, DEVCB_##_devcb);
#define MCFG_MELPS4_WRITE_F_CB(_devcb) \
	devcb = &melps4_cpu_device::set_write_f_callback(*device, DEVCB_##_devcb);

// 4-bit G generic output port
#define MCFG_MELPS4_WRITE_G_CB(_devcb) \
	devcb = &melps4_cpu_device::set_write_g_callback(*device, DEVCB_##_devcb);

// 1-bit U generic output port
#define MCFG_MELPS4_WRITE_U_CB(_devcb) \
	devcb = &melps4_cpu_device::set_write_u_callback(*device, DEVCB_##_devcb);

// T timer I/O pin (use execute_set_input for reads)
#define MCFG_MELPS4_WRITE_T_CB(_devcb) \
	devcb = &melps4_cpu_device::set_write_t_callback(*device, DEVCB_##_devcb);


#define MELPS4_PORTD_CLR 16

// only generic ports here
enum
{
	MELPS4_PORTS = 0,
	MELPS4_PORTF,
	MELPS4_PORTG,
	MELPS4_PORTU
};

enum
{
	MELPS4_INPUT_LINE_INT = 0,
	MELPS4_INPUT_LINE_T
};



// pinout reference

/*
            ______   ______
    D9   1 |*     \_/      | 42 D8
    D10  2 |               | 41 D7
    D11  3 |               | 40 D6
  RESET  4 |               | 39 D5
      T  5 |               | 38 D4
     K0  6 |               | 37 D3
     K1  7 |               | 36 D2
     K2  8 |               | 35 D1
     K3  9 |               | 34 D0
     G0 10 |               | 33 Xin
     G1 11 |     M58846    | 32 Xout
     G2 12 |               | 31 S7
     G3 13 |               | 30 S6
      U 14 |               | 29 S5
     F0 15 |               | 28 S4
     F1 16 |               | 27 S3
     F2 17 |               | 26 S2
     F3 18 |               | 25 S1
    INT 19 |               | 24 S0
  CNVss 20 |               | 23 Vp
    Vss 21 |_______________| 22 Vdd

*/


class melps4_cpu_device : public cpu_device
{
public:
	// static configuration helpers
	template <class Object> static devcb_base &set_read_k_callback(device_t &device, Object &&cb) { return downcast<melps4_cpu_device &>(device).m_read_k.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_read_d_callback(device_t &device, Object &&cb) { return downcast<melps4_cpu_device &>(device).m_read_d.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_read_s_callback(device_t &device, Object &&cb) { return downcast<melps4_cpu_device &>(device).m_read_s.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_read_f_callback(device_t &device, Object &&cb) { return downcast<melps4_cpu_device &>(device).m_read_f.set_callback(std::forward<Object>(cb)); }

	template <class Object> static devcb_base &set_write_d_callback(device_t &device, Object &&cb) { return downcast<melps4_cpu_device &>(device).m_write_d.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_write_s_callback(device_t &device, Object &&cb) { return downcast<melps4_cpu_device &>(device).m_write_s.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_write_f_callback(device_t &device, Object &&cb) { return downcast<melps4_cpu_device &>(device).m_write_f.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_write_g_callback(device_t &device, Object &&cb) { return downcast<melps4_cpu_device &>(device).m_write_g.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_write_u_callback(device_t &device, Object &&cb) { return downcast<melps4_cpu_device &>(device).m_write_u.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_write_t_callback(device_t &device, Object &&cb) { return downcast<melps4_cpu_device &>(device).m_write_t.set_callback(std::forward<Object>(cb)); }

protected:
	// construction/destruction
	melps4_cpu_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, int prgwidth, address_map_constructor program, int datawidth, address_map_constructor data, int d_pins, uint8_t sm_page, uint8_t int_page);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_execute_interface overrides
	virtual uint64_t execute_clocks_to_cycles(uint64_t clocks) const override { return (clocks + 6 - 1) / 6; } // 6 t-states per machine cycle
	virtual uint64_t execute_cycles_to_clocks(uint64_t cycles) const override { return (cycles * 6); } // "
	virtual uint32_t execute_min_cycles() const override { return 1; }
	virtual uint32_t execute_max_cycles() const override { return 1+1; } // max opcode cycles + interrupt duration
	virtual uint32_t execute_input_lines() const override { return 3; } // up to 3 (some internal)
	virtual void execute_set_input(int line, int state) override;
	virtual void execute_run() override;
	virtual void execute_one();

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// device_disasm_interface overrides
	virtual uint32_t disasm_min_opcode_bytes() const override { return 2; }
	virtual uint32_t disasm_max_opcode_bytes() const override { return 2; }
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	address_space_config m_program_config;
	address_space_config m_data_config;
	address_space *m_program;
	address_space *m_data;

	int m_icount;

	// fixed settings or mask options that differ between MCU type
	int m_prgwidth;         // number of bits and bitmask for ROM/RAM size: see melps4.c for info
	int m_datawidth;        // "
	int m_prgmask;          // "
	int m_datamask;         // "
	int m_d_pins;           // number of D port pins and bitmask: 11 on '40,'41,'42,'44, 8 on '43, 12 on '45,'46, 16 on '47
	int m_d_mask;           // "

	uint8_t m_sm_page;      // subroutine default page: 14 on '40 to '44, 2 on '45,'46, 0 on '47
	uint8_t m_int_page;     // interrupt routine page: 12 on '40 to '44, 1 on '45,'46, 2 on '47
	uint8_t m_xami_mask;    // mask option for XAMI opcode on '40,'41,'45 (0xf for others)
	uint16_t m_sp_mask;     // SP opcode location(middle 4 bits): 7 on '40 to '46, 3 on '47
	uint16_t m_ba_op;       // BA opcode location: 1 on '40 to '46, N/A on '47
	uint8_t m_stack_levels; // 3 levels on MELPS 4, 12 levels on MELPS 41/42

	// internal state, misc regs
	uint16_t m_pc;          // program counter (11 or 10-bit)
	uint16_t m_prev_pc;
	uint16_t m_stack[12];   // callstack (SK0-SKx, same size as PC)
	uint16_t m_op;
	uint16_t m_prev_op;
	uint8_t m_bitmask;      // opcode bit argument

	uint16_t m_port_d;      // last written port data
	uint8_t m_port_s;       // "
	uint8_t m_port_f;       // "
	uint8_t m_port_t;       // "

	bool m_sm, m_sms;       // subroutine mode flag + irq stack
	bool m_ba_flag;         // temp flag indicates BA opcode was executed
	uint8_t m_sp_param;     // temp register holding SP opcode parameter
	uint8_t m_cps;          // DP,CY or DP',CY' selected
	bool m_skip;            // skip next opcode
	uint8_t m_inte;         // interrupt enable flag
	int m_intp;             // external interrupt polarity ('40 to '44)
	bool m_irqflag[3];      // irq flags: exf, 1f, 2f (external, timer 1, timer 2)
	int m_int_state;        // INT pin state
	int m_t_in_state;       // T input pin state
	bool m_prohibit_irq;    // interrupt is prohibited during certain opcodes
	bool m_possible_irq;    // indicate that irq needs to be rechecked

	uint8_t m_tmr_count[2]; // timer active count
	uint8_t m_tmr_reload;   // timer(2) auto reload
	bool m_tmr_irq_enabled[2];

	// work registers (unless specified, each is 4-bit)
	uint8_t m_a;            // accumulator
	uint8_t m_b;            // generic
	uint8_t m_e;            // 8-bit register, hold data for S output
	uint8_t m_y, m_y2;      // RAM index Y, Y' (Z.XX.YYYY is DP aka Data Pointer)
	uint8_t m_x, m_x2;      // RAM index X, X', 2-bit
	uint8_t m_z, m_z2;      // RAM index Z, Z', 1-bit, optional
	uint8_t m_cy, m_cy2;    // carry flag(s)

	uint8_t m_h;            // A/D converter H or generic
	uint8_t m_l;            // A/D converter L or generic
	uint8_t m_c;            // A/D converter counter
	uint8_t m_v;            // timer control V
	uint8_t m_w;            // timer control W

	// i/o handlers
	devcb_read16 m_read_k;
	devcb_read16 m_read_d;
	devcb_read8 m_read_s;
	devcb_read8 m_read_f;

	devcb_write16 m_write_d;
	devcb_write8 m_write_s;
	devcb_write8 m_write_f;
	devcb_write8 m_write_g;
	devcb_write8 m_write_u;
	devcb_write_line m_write_t;

	virtual void write_t_in(int state) { m_t_in_state = state; }
	virtual void write_v(uint8_t data) { m_v = data; }
	virtual void write_w(uint8_t data) { m_w = data; }
	virtual void do_interrupt(int which);
	virtual void check_interrupt();

	uint8_t read_gen_port(int port);
	void write_gen_port(int port, uint8_t data);
	int read_d_pin(int bit);
	void write_d_pin(int bit, int state);

	// misc internal helpers
	uint8_t ram_r();
	void ram_w(uint8_t data);
	void pop_pc();
	void push_pc();

	// opcode handlers
	void op_tab();
	void op_tba();
	void op_tay();
	void op_tya();
	void op_teab();
	void op_tabe();
	void op_tepa();
	void op_txa();
	void op_tax();

	void op_lxy();
	void op_lz();
	void op_iny();
	void op_dey();
	void op_lcps();
	void op_sadr();

	void op_tam();
	void op_xam();
	void op_xamd();
	void op_xami();

	void op_la();
	void op_am();
	void op_amc();
	void op_amcs();
	void op_a();
	void op_sc();
	void op_rc();
	void op_szc();
	void op_cma();
	void op_rl();
	void op_rr();

	void op_sb();
	void op_rb();
	void op_szb();

	void op_seam();
	void op_sey();

	void op_tla();
	void op_tha();
	void op_taj();
	void op_xal();
	void op_xah();
	void op_lc7();
	void op_dec();
	void op_shl();
	void op_rhl();
	void op_cpa();
	void op_cpas();
	void op_cpae();
	void op_szj();

	void op_t1ab();
	void op_trab();
	void op_t2ab();
	void op_tab1();
	void op_tabr();
	void op_tab2();
	void op_tva();
	void op_twa();
	void op_snz1();
	void op_snz2();

	void op_ba();
	void op_sp();
	void op_b();
	void op_bm();

	void op_rt();
	void op_rts();
	void op_rti();

	void op_cld();
	void op_cls();
	void op_clds();
	void op_sd();
	void op_rd();
	void op_szd();
	void op_osab();
	void op_ospa();
	void op_ose();
	void op_ias();
	void op_ofa();
	void op_iaf();
	void op_oga();
	void op_iak();
	void op_szk();
	void op_su();

	void op_ei();
	void op_di();
	void op_inth();
	void op_intl();

	void op_nop();
	void op_illegal();
};


#endif // MAME_CPU_MELPS4_MELPS4_H
