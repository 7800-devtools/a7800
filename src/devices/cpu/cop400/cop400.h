// license:BSD-3-Clause
// copyright-holders:Curt Coder
/***************************************************************************

    cop400.h

    National Semiconductor COPS Emulator.

***************************************************************************/

#ifndef MAME_DEVICES_CPU_COP400_H
#define MAME_DEVICES_CPU_COP400_H

#pragma once

// i/o pins

// L pins: 8-bit bi-directional
#define MCFG_COP400_READ_L_CB(_devcb) \
	devcb = &cop400_cpu_device::set_read_l_callback(*device, DEVCB_##_devcb);
#define MCFG_COP400_WRITE_L_CB(_devcb) \
	devcb = &cop400_cpu_device::set_write_l_callback(*device, DEVCB_##_devcb);
// output state when pins are in tri-state, default 0
#define MCFG_COP400_READ_L_TRISTATE_CB(_devcb) \
	devcb = &cop400_cpu_device::set_read_l_tristate_callback(*device, DEVCB_##_devcb);

// G pins: 4-bit bi-directional
#define MCFG_COP400_READ_G_CB(_devcb) \
	devcb = &cop400_cpu_device::set_read_g_callback(*device, DEVCB_##_devcb);
#define MCFG_COP400_WRITE_G_CB(_devcb) \
	devcb = &cop400_cpu_device::set_write_g_callback(*device, DEVCB_##_devcb);

// D outputs: 4-bit general purpose output
#define MCFG_COP400_WRITE_D_CB(_devcb) \
	devcb = &cop400_cpu_device::set_write_d_callback(*device, DEVCB_##_devcb);

// IN inputs: 4-bit general purpose input
#define MCFG_COP400_READ_IN_CB(_devcb) \
	devcb = &cop400_cpu_device::set_read_in_callback(*device, DEVCB_##_devcb);

// SI/SO lines: serial in/out or counter/gen.purpose
#define MCFG_COP400_READ_SI_CB(_devcb) \
	devcb = &cop400_cpu_device::set_read_si_callback(*device, DEVCB_##_devcb);
#define MCFG_COP400_WRITE_SO_CB(_devcb) \
	devcb = &cop400_cpu_device::set_write_so_callback(*device, DEVCB_##_devcb);

// SK output line: logic-controlled clock or gen.purpose
#define MCFG_COP400_WRITE_SK_CB(_devcb) \
	devcb = &cop400_cpu_device::set_write_sk_callback(*device, DEVCB_##_devcb);

// CKI/CKO lines: only CKO input here
#define MCFG_COP400_READ_CKO_CB(_devcb) \
	devcb = &cop400_cpu_device::set_read_cko_callback(*device, DEVCB_##_devcb);


/***************************************************************************
    CONSTANTS
***************************************************************************/

/* register access indexes */
enum
{
	COP400_PC,
	COP400_SA,
	COP400_SB,
	COP400_SC,
	COP400_A,
	COP400_B,
	COP400_C,
	COP400_G,
	COP400_Q,
	COP400_EN,
	COP400_SIO,
	COP400_SKL,
	COP400_T
};

/* input lines */
enum
{
	/* COP420 */
	COP400_IN0 = 0,
	COP400_IN1,
	COP400_IN2,
	COP400_IN3,

	/* COP404L */
	COP400_MB,
	COP400_DUAL,
	COP400_SEL10,
	COP400_SEL20
};

/* CKI bonding options */
enum cop400_cki_bond {
	COP400_CKI_DIVISOR_4 = 4,
	COP400_CKI_DIVISOR_8 = 8,
	COP400_CKI_DIVISOR_16 = 16,
	COP400_CKI_DIVISOR_32 = 32
};

/* CKO bonding options */
enum cop400_cko_bond {
	COP400_CKO_OSCILLATOR_OUTPUT = 0,
	COP400_CKO_RAM_POWER_SUPPLY,
	COP400_CKO_HALT_IO_PORT,
	COP400_CKO_SYNC_INPUT,
	COP400_CKO_GENERAL_PURPOSE_INPUT
};


#define MCFG_COP400_CONFIG(_cki, _cko, _microbus) \
	cop400_cpu_device::set_cki(*device, _cki); \
	cop400_cpu_device::set_cko(*device, _cko); \
	cop400_cpu_device::set_microbus(*device, _microbus);


class cop400_cpu_device : public cpu_device
{
public:
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// static configuration helpers
	template<class _Object> static devcb_base &set_read_l_callback(device_t &device, _Object object) { return downcast<cop400_cpu_device &>(device).m_read_l.set_callback(object); }
	template<class _Object> static devcb_base &set_read_l_tristate_callback(device_t &device, _Object object) { return downcast<cop400_cpu_device &>(device).m_read_l_tristate.set_callback(object); }
	template<class _Object> static devcb_base &set_write_l_callback(device_t &device, _Object object) { return downcast<cop400_cpu_device &>(device).m_write_l.set_callback(object); }
	template<class _Object> static devcb_base &set_read_g_callback(device_t &device, _Object object) { return downcast<cop400_cpu_device &>(device).m_read_g.set_callback(object); }
	template<class _Object> static devcb_base &set_write_g_callback(device_t &device, _Object object) { return downcast<cop400_cpu_device &>(device).m_write_g.set_callback(object); }
	template<class _Object> static devcb_base &set_write_d_callback(device_t &device, _Object object) { return downcast<cop400_cpu_device &>(device).m_write_d.set_callback(object); }
	template<class _Object> static devcb_base &set_read_in_callback(device_t &device, _Object object) { return downcast<cop400_cpu_device &>(device).m_read_in.set_callback(object); }
	template<class _Object> static devcb_base &set_read_si_callback(device_t &device, _Object object) { return downcast<cop400_cpu_device &>(device).m_read_si.set_callback(object); }
	template<class _Object> static devcb_base &set_write_so_callback(device_t &device, _Object object) { return downcast<cop400_cpu_device &>(device).m_write_so.set_callback(object); }
	template<class _Object> static devcb_base &set_write_sk_callback(device_t &device, _Object object) { return downcast<cop400_cpu_device &>(device).m_write_sk.set_callback(object); }
	template<class _Object> static devcb_base &set_read_cko_callback(device_t &device, _Object object) { return downcast<cop400_cpu_device &>(device).m_read_cko.set_callback(object); }

	static void set_cki(device_t &device, cop400_cki_bond cki) { downcast<cop400_cpu_device &>(device).m_cki = cki; }
	static void set_cko(device_t &device, cop400_cko_bond cko) { downcast<cop400_cpu_device &>(device).m_cko = cko; }
	static void set_microbus(device_t &device, bool has_microbus) { downcast<cop400_cpu_device &>(device).m_has_microbus = has_microbus; }

	DECLARE_READ8_MEMBER( microbus_rd );
	DECLARE_WRITE8_MEMBER( microbus_wr );

protected:
	// construction/destruction
	cop400_cpu_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, uint8_t program_addr_bits, uint8_t data_addr_bits, uint8_t featuremask, uint8_t g_mask, uint8_t d_mask, uint8_t in_mask, bool has_counter, bool has_inil, address_map_constructor internal_map_program, address_map_constructor internal_map_data);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_execute_interface overrides
	virtual uint64_t execute_clocks_to_cycles(uint64_t clocks) const override { return (clocks + m_cki - 1) / m_cki; }
	virtual uint64_t execute_cycles_to_clocks(uint64_t cycles) const override { return (cycles * m_cki); }
	virtual uint32_t execute_min_cycles() const override { return 1; }
	virtual uint32_t execute_max_cycles() const override { return 2; }
	virtual uint32_t execute_input_lines() const override { return 0; }
	virtual void execute_run() override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// device_state_interface overrides
	virtual void state_import(const device_state_entry &entry) override;
	virtual void state_export(const device_state_entry &entry) override;
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	// device_disasm_interface overrides
	virtual uint32_t disasm_min_opcode_bytes() const override { return 1; }
	virtual uint32_t disasm_max_opcode_bytes() const override { return 2; }
	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;

	address_space_config m_program_config;
	address_space_config m_data_config;

	// i/o handlers
	devcb_read8 m_read_l;
	devcb_read8 m_read_l_tristate;
	devcb_write8 m_write_l;
	devcb_read8 m_read_g;
	devcb_write8 m_write_g;
	devcb_write8 m_write_d;
	devcb_read8 m_read_in;
	devcb_read_line m_read_si;
	devcb_write_line m_write_so;
	devcb_write_line m_write_sk;
	devcb_read_line m_read_cko;

	enum {
		COP410_FEATURE = 0x01,
		COP420_FEATURE = 0x02,
		COP444L_FEATURE = 0x04,
		COP424C_FEATURE = 0x08
	};

	enum {
		TIMER_SERIAL,
		TIMER_COUNTER,
		TIMER_COUNTER_T,
		TIMER_INIL
	};

	cop400_cki_bond m_cki;
	cop400_cko_bond m_cko;
	bool m_has_microbus;

	bool m_has_counter;
	bool m_has_inil;

	address_space *m_program;
	direct_read_data *m_direct;
	address_space *m_data;

	uint8_t m_featuremask;

	/* registers */
	uint16_t  m_pc;             /* 9/10/11-bit ROM address program counter */
	uint16_t  m_prevpc;         /* previous value of program counter */
	uint8_t   m_a;              /* 4-bit accumulator */
	uint8_t   m_b;              /* 5/6/7-bit RAM address register */
	int     m_c;              /* 1-bit carry register */
	uint8_t   m_en;             /* 4-bit enable register */
	uint8_t   m_g;              /* 4-bit general purpose I/O port */
	uint8_t   m_q;              /* 8-bit latch for L port */
	uint16_t  m_sa, m_sb, m_sc; /* subroutine save registers */
	uint8_t   m_sio;            /* 4-bit shift register and counter */
	int     m_skl;            /* 1-bit latch for SK output */
	uint8_t   m_flags;          // used for I/O only

	/* counter */
	uint8_t   m_t;              /* 8-bit timer */
	int     m_skt_latch;      /* timer overflow latch */

	/* input/output ports */
	uint8_t   m_g_mask;         /* G port mask */
	uint8_t   m_d_mask;         /* D port mask */
	uint8_t   m_in_mask;        /* IN port mask */
	uint8_t   m_il;             /* IN latch */
	uint8_t   m_in[4];          /* IN port shift register */
	uint8_t   m_si;             /* serial input */

	/* skipping logic */
	bool m_skip;               /* skip next instruction */
	int m_skip_lbi;           /* skip until next non-LBI instruction */
	bool m_last_skip;          /* last value of skip */
	bool m_halt;               /* halt mode */
	bool m_idle;               /* idle mode */

	/* execution logic */
	int m_InstLen[256];       /* instruction length in bytes */
	int m_icount;             /* instruction counter */

	/* timers */
	emu_timer *m_serial_timer;
	emu_timer *m_counter_timer;
	emu_timer *m_inil_timer;

	typedef void ( cop400_cpu_device::*cop400_opcode_func ) (uint8_t opcode);

	const cop400_opcode_func *m_opcode_map;

	static const cop400_opcode_func COP410_OPCODE_23_MAP[256];
	static const cop400_opcode_func COP410_OPCODE_33_MAP[256];
	static const cop400_opcode_func COP410_OPCODE_MAP[256];
	static const cop400_opcode_func COP420_OPCODE_23_MAP[256];
	static const cop400_opcode_func COP420_OPCODE_33_MAP[256];
	static const cop400_opcode_func COP420_OPCODE_MAP[256];
	static const cop400_opcode_func COP444L_OPCODE_23_MAP[256];
	static const cop400_opcode_func COP444L_OPCODE_33_MAP[256];
	static const cop400_opcode_func COP444L_OPCODE_MAP[256];
	static const cop400_opcode_func COP424C_OPCODE_23_MAP[256];
	static const cop400_opcode_func COP424C_OPCODE_33_MAP[256];
	static const cop400_opcode_func COP424C_OPCODE_MAP[256];

	void serial_tick();
	void counter_tick();
	void inil_tick();

	void PUSH(uint16_t data);
	void POP();
	void WRITE_Q(uint8_t data);
	void WRITE_G(uint8_t data);

	uint8_t fetch();

	void illegal(uint8_t opcode);
	void asc(uint8_t opcode);
	void add(uint8_t opcode);
	void aisc(uint8_t opcode);
	void clra(uint8_t opcode);
	void comp(uint8_t opcode);
	void nop(uint8_t opcode);
	void rc(uint8_t opcode);
	void sc(uint8_t opcode);
	void xor_(uint8_t opcode);
	void adt(uint8_t opcode);
	void casc(uint8_t opcode);
	void jid(uint8_t opcode);
	void jmp(uint8_t opcode);
	void jp(uint8_t opcode);
	void jsr(uint8_t opcode);
	void ret(uint8_t opcode);
	void cop420_ret(uint8_t opcode);
	void retsk(uint8_t opcode);
	void halt(uint8_t opcode);
	void it(uint8_t opcode);
	void camq(uint8_t opcode);
	void ld(uint8_t opcode);
	void lqid(uint8_t opcode);
	void rmb0(uint8_t opcode);
	void rmb1(uint8_t opcode);
	void rmb2(uint8_t opcode);
	void rmb3(uint8_t opcode);
	void smb0(uint8_t opcode);
	void smb1(uint8_t opcode);
	void smb2(uint8_t opcode);
	void smb3(uint8_t opcode);
	void stii(uint8_t opcode);
	void x(uint8_t opcode);
	void xad(uint8_t opcode);
	void xds(uint8_t opcode);
	void xis(uint8_t opcode);
	void cqma(uint8_t opcode);
	void ldd(uint8_t opcode);
	void camt(uint8_t opcode);
	void ctma(uint8_t opcode);
	void cab(uint8_t opcode);
	void cba(uint8_t opcode);
	void lbi(uint8_t opcode);
	void lei(uint8_t opcode);
	void xabr(uint8_t opcode);
	void cop444l_xabr(uint8_t opcode);
	void skc(uint8_t opcode);
	void ske(uint8_t opcode);
	void skgz(uint8_t opcode);
	void skgbz0(uint8_t opcode);
	void skgbz1(uint8_t opcode);
	void skgbz2(uint8_t opcode);
	void skgbz3(uint8_t opcode);
	void skmbz0(uint8_t opcode);
	void skmbz1(uint8_t opcode);
	void skmbz2(uint8_t opcode);
	void skmbz3(uint8_t opcode);
	void skt(uint8_t opcode);
	void ing(uint8_t opcode);
	void inl(uint8_t opcode);
	void obd(uint8_t opcode);
	void omg(uint8_t opcode);
	void xas(uint8_t opcode);
	void inin(uint8_t opcode);
	void cop402m_inin(uint8_t opcode);
	void inil(uint8_t opcode);
	void ogi(uint8_t opcode);
	void cop410_op23(uint8_t opcode);
	void cop410_op33(uint8_t opcode);
	void cop420_op23(uint8_t opcode);
	void cop420_op33(uint8_t opcode);
	void cop444l_op23(uint8_t opcode);
	void cop444l_op33(uint8_t opcode);
	void cop424c_op23(uint8_t opcode);
	void cop424c_op33(uint8_t opcode);
	void skgbz(int bit);
	void skmbz(int bit);
};


/* COP410 family */
// COP401 is a ROMless version of the COP410
class cop401_cpu_device : public cop400_cpu_device
{
public:
	// construction/destruction
	cop401_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


class cop410_cpu_device : public cop400_cpu_device
{
public:
	// construction/destruction
	cop410_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


// COP411 is a 20-pin package version of the COP410, missing D2/D3/G3/CKO
class cop411_cpu_device : public cop400_cpu_device
{
public:
	// construction/destruction
	cop411_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


/* COP420 family */
// COP402 is a ROMless version of the COP420
class cop402_cpu_device : public cop400_cpu_device
{
public:
	// construction/destruction
	cop402_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


class cop420_cpu_device : public cop400_cpu_device
{
public:
	// construction/destruction
	cop420_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


// COP421 is a 24-pin package version of the COP420, lacking the IN ports
class cop421_cpu_device : public cop400_cpu_device
{
public:
	// construction/destruction
	cop421_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


// COP422 is a 20-pin package version of the COP420, lacking G0/G1, D0/D1, and the IN ports
class cop422_cpu_device : public cop400_cpu_device
{
public:
	// construction/destruction
	cop422_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


/* COP444 family */
// COP404L is a ROMless version of the COP444, which can emulate a COP410/COP411, COP424/COP425, or a COP444/COP445
class cop404l_cpu_device : public cop400_cpu_device
{
public:
	// construction/destruction
	cop404l_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


class cop444l_cpu_device : public cop400_cpu_device
{
public:
	// construction/destruction
	cop444l_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


// COP445 is a 24-pin package version of the COP444, lacking the IN ports
class cop445l_cpu_device : public cop400_cpu_device
{
public:
	// construction/destruction
	cop445l_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


// COP404C
class cop404c_cpu_device : public cop400_cpu_device
{
public:
	// construction/destruction
	cop404c_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


// COP424C is functionally equivalent to COP444C, with only 1K ROM and 64x4 bytes RAM
class cop424c_cpu_device : public cop400_cpu_device
{
public:
	// construction/destruction
	cop424c_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


// COP425C is a 24-pin package version of the COP424C, lacking the IN ports
class cop425c_cpu_device : public cop400_cpu_device
{
public:
	// construction/destruction
	cop425c_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


// COP426C is a 20-pin package version of the COP424C, with only L0-L7, G2-G3, D2-D3 ports
class cop426c_cpu_device : public cop400_cpu_device
{
public:
	// construction/destruction
	cop426c_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


// COP444C
class cop444c_cpu_device : public cop400_cpu_device
{
public:
	// construction/destruction
	cop444c_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


// COP445C
class cop445c_cpu_device : public cop400_cpu_device
{
public:
	// construction/destruction
	cop445c_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


// COP446C
class cop446c_cpu_device : public cop400_cpu_device
{
public:
	// construction/destruction
	cop446c_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


DECLARE_DEVICE_TYPE(COP401, cop401_cpu_device)
DECLARE_DEVICE_TYPE(COP410, cop410_cpu_device)
DECLARE_DEVICE_TYPE(COP411, cop411_cpu_device)
DECLARE_DEVICE_TYPE(COP402, cop402_cpu_device)
DECLARE_DEVICE_TYPE(COP420, cop420_cpu_device)
DECLARE_DEVICE_TYPE(COP421, cop421_cpu_device)
DECLARE_DEVICE_TYPE(COP422, cop422_cpu_device)
DECLARE_DEVICE_TYPE(COP404L, cop404l_cpu_device)
DECLARE_DEVICE_TYPE(COP444L, cop444l_cpu_device)
DECLARE_DEVICE_TYPE(COP445L, cop445l_cpu_device)
DECLARE_DEVICE_TYPE(COP404C, cop404c_cpu_device)
DECLARE_DEVICE_TYPE(COP424C, cop424c_cpu_device)
DECLARE_DEVICE_TYPE(COP425C, cop425c_cpu_device)
DECLARE_DEVICE_TYPE(COP426C, cop426c_cpu_device)
DECLARE_DEVICE_TYPE(COP444C, cop444c_cpu_device)
DECLARE_DEVICE_TYPE(COP445C, cop445c_cpu_device)
DECLARE_DEVICE_TYPE(COP446C, cop446c_cpu_device)

#endif  // MAME_DEVICES_CPU_COP400_H
