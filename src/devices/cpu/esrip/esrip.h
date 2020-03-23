// license:BSD-3-Clause
// copyright-holders:Philip Bennett
/***************************************************************************

    esrip.h
    Interface file for the Entertainment Sciences RIP
    Written by Phil Bennett

***************************************************************************/

#ifndef MAME_CPU_ESRIP_ESRIP_H
#define MAME_CPU_ESRIP_ESRIP_H

#pragma once


/***************************************************************************
    INTERFACE CONFIGURATION MACROS
***************************************************************************/
#define ESRIP_DRAW(name) int name(int l, int r, int fig, int attr, int addr, int col, int x_scale, int bank)

#define MCFG_ESRIP_FDT_R_CALLBACK(_read) \
	devcb = &esrip_device::static_set_fdt_r_callback(*device, DEVCB_##_read);

#define MCFG_ESRIP_FDT_W_CALLBACK(_write) \
	devcb = &esrip_device::static_set_fdt_w_callback(*device, DEVCB_##_write);

#define MCFG_ESRIP_STATUS_IN_CALLBACK(_read) \
	devcb = &esrip_device::static_set_status_in_callback(*device, DEVCB_##_read);

#define MCFG_ESRIP_DRAW_CALLBACK_OWNER(_class, _method) \
	esrip_device::static_set_draw_callback(*device, esrip_device::draw_delegate(&_class::_method, #_class "::" #_method, downcast<_class *>(owner)));

#define MCFG_ESRIP_LBRM_PROM(_tag) \
	esrip_device::static_lbrm_prom(*device, _tag);


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// device type definition
DECLARE_DEVICE_TYPE(ESRIP, esrip_device)

// ======================> esrip_device

// Used by core CPU interface
class esrip_device : public cpu_device
{
public:
	typedef device_delegate<int (int l, int r, int fig, int attr, int addr, int col, int x_scale, int bank)> draw_delegate;

	// construction/destruction
	esrip_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// inline configuration helpers
	template <class Object> static devcb_base &static_set_fdt_r_callback(device_t &device, Object &&cb) { return downcast<esrip_device &>(device).m_fdt_r.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &static_set_fdt_w_callback(device_t &device, Object &&cb) { return downcast<esrip_device &>(device).m_fdt_w.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &static_set_status_in_callback(device_t &device, Object &&cb) { return downcast<esrip_device &>(device).m_status_in.set_callback(std::forward<Object>(cb)); }
	static void static_set_draw_callback(device_t &device, draw_delegate func) { downcast<esrip_device &>(device).m_draw = func; }
	static void static_lbrm_prom(device_t &device, const char *name) { downcast<esrip_device &>(device).m_lbrm_prom = name; }

	// public interfaces
	uint8_t get_rip_status();

protected:
	// register enumeration
	enum
	{
		ESRIP_PC = 1,
		ESRIP_ACC,
		ESRIP_DLATCH,
		ESRIP_ILATCH,
		ESRIP_RAM00,
		ESRIP_RAM01,
		ESRIP_RAM02,
		ESRIP_RAM03,
		ESRIP_RAM04,
		ESRIP_RAM05,
		ESRIP_RAM06,
		ESRIP_RAM07,
		ESRIP_RAM08,
		ESRIP_RAM09,
		ESRIP_RAM0A,
		ESRIP_RAM0B,
		ESRIP_RAM0C,
		ESRIP_RAM0D,
		ESRIP_RAM0E,
		ESRIP_RAM0F,
		ESRIP_RAM10,
		ESRIP_RAM11,
		ESRIP_RAM12,
		ESRIP_RAM13,
		ESRIP_RAM14,
		ESRIP_RAM15,
		ESRIP_RAM16,
		ESRIP_RAM17,
		ESRIP_RAM18,
		ESRIP_RAM19,
		ESRIP_RAM1A,
		ESRIP_RAM1B,
		ESRIP_RAM1C,
		ESRIP_RAM1D,
		ESRIP_RAM1E,
		ESRIP_RAM1F,
		ESRIP_STATW,
		ESRIP_FDTC,
		ESRIP_IPTC,
		ESRIP_XSCALE,
		ESRIP_YSCALE,
		ESRIP_BANK,
		ESRIP_LINE,
		ESRIP_FIG,
		ESRIP_ATTR,
		ESRIP_ADRL,
		ESRIP_ADRR,
		ESRIP_COLR,
		ESRIP_IADDR
	};

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_stop() override;

	void make_ops();

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

	// CPU registers
	uint16_t  m_ram[32];
	uint16_t  m_acc;
	uint16_t  m_d_latch;
	uint16_t  m_i_latch;
	uint16_t  m_result;
	uint8_t   m_new_status;
	uint8_t   m_status;
	uint16_t  m_inst;
	uint8_t   m_immflag;
	uint8_t   m_ct;
	uint8_t   m_t;

	/* Instruction latches - current and previous values */
	uint8_t   m_l1, m_pl1;
	uint8_t   m_l2, m_pl2;
	uint8_t   m_l3, m_pl3;
	uint8_t   m_l4, m_pl4;
	uint8_t   m_l5, m_pl5;
	uint8_t   m_l6, m_pl6;
	uint8_t   m_l7, m_pl7;

	uint8_t   m_pc;
	uint16_t  m_rip_pc;
	uint8_t   m_status_out;

	uint8_t   m_x_scale;
	uint8_t   m_y_scale;
	uint8_t   m_img_bank;
	uint8_t   m_line_latch;
	uint16_t  m_fig_latch;
	uint16_t  m_attr_latch;
	uint16_t  m_adl_latch;
	uint16_t  m_adr_latch;
	uint16_t  m_iaddr_latch;
	uint8_t   m_c_latch;

	uint16_t  m_fdt_cnt;
	uint16_t  m_ipt_cnt;

	uint8_t   m_fig;
	uint16_t  m_fig_cycles;

	uint8_t   m_optable[65536];

	std::vector<uint16_t> m_ipt_ram;
	uint8_t   *m_lbrm;

	address_space *m_program;
	direct_read_data *m_direct;

	int     m_icount;

	devcb_read16  m_fdt_r;
	devcb_write16 m_fdt_w;
	devcb_read8 m_status_in;
	draw_delegate m_draw;
	const char *m_lbrm_prom;

	typedef void (esrip_device::*ophandler)(uint16_t inst);

	ophandler m_opcode[24];

	static const ophandler s_opcodetable[24];

private:
	int get_hblank() const;
	int get_lbrm() const;
	int check_jmp(uint8_t jmp_ctrl) const;

	// flags
	void calc_z_flag(uint16_t res);
	void calc_c_flag_add(uint16_t a, uint16_t b);
	void calc_c_flag_sub(uint16_t a, uint16_t b);
	void calc_n_flag(uint16_t res);
	void calc_v_flag_add(uint16_t a, uint16_t b, uint32_t r);
	void calc_v_flag_sub(uint16_t a, uint16_t b, uint32_t r);

	// opcodes
	uint16_t sor_op(uint16_t r, uint16_t opcode);
	void sor(uint16_t inst);
	void sonr(uint16_t inst);

	uint16_t tor_op(uint16_t r, uint16_t s, int opcode);
	void tonr(uint16_t inst);
	void tor1(uint16_t inst);
	void tor2(uint16_t inst);

	void bonr(uint16_t inst);
	void bor1(uint16_t inst);
	void bor2(uint16_t inst);

	void rotr1(uint16_t inst);
	void rotr2(uint16_t inst);
	void rotnr(uint16_t inst);
	void rotc(uint16_t inst);
	void rotm(uint16_t inst);

	void prt(uint16_t inst);
	void prtnr(uint16_t inst);

	void crcf(uint16_t inst);
	void crcr(uint16_t inst);

	uint16_t shift_op(uint16_t u, int opcode);
	void shftr(uint16_t inst);
	void shftnr(uint16_t inst);

	void svstr(uint16_t inst);

	void rstst(uint16_t inst);
	void setst(uint16_t inst);

	void test(uint16_t inst);

	void nop(uint16_t inst);

	void am29116_execute(uint16_t inst, int _sre);
};


CPU_DISASSEMBLE( esrip );

#endif // MAME_CPU_ESRIP_ESRIP_H
