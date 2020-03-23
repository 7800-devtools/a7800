// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    MOS 6526/8520 Complex Interface Adapter emulation

**********************************************************************
                            _____   _____
                   Vss   1 |*    \_/     | 40  CNT
                   PA0   2 |             | 39  SP
                   PA1   3 |             | 38  RS0
                   PA2   4 |             | 37  RS1
                   PA3   5 |             | 36  RS2
                   PA4   6 |             | 35  RS3
                   PA5   7 |             | 34  _RES
                   PA6   8 |             | 33  DB0
                   PA7   9 |             | 32  DB1
                   PB0  10 |   MOS6526   | 31  DB2
                   PB1  11 |   MOS8520   | 30  DB3
                   PB2  12 |             | 29  DB4
                   PB3  13 |             | 28  DB5
                   PB4  14 |             | 27  DB6
                   PB5  15 |             | 26  DB7
                   PB6  16 |             | 25  phi2
                   PB7  17 |             | 24  _FLAG
                   _PC  18 |             | 23  _CS
                   TOD  19 |             | 22  R/W
                   Vcc  20 |_____________| 21  _IRQ

                            _____   _____
                  FCO*   1 |*    \_/     | 48  FDO*
                   TED   2 |             | 47  FCI*
                  phi0   3 |             | 46  FDI*
                 CLKIN   4 |             | 45  IRQ
                 CTRLO   5 |             | 44  RSET
                 CTRLI   6 |             | 43
                  phi2   7 |             | 42
                    D7   8 |             | 41  INDEX*
                    D6   9 |             | 40  WG2*
                    D5  10 |             | 39  WPRT*
                    D4  11 |             | 38  RPULSE
                   GND  12 |   MOS5710   | 37  Q
                   Vcc  13 |             | 36  Vcc
                    D3  14 |             | 35  GND
                    D2  15 |             | 34  CS3*
                    D1  16 |             | 33  CS2*
                    D0  17 |             | 32  CS1*
                   A15  18 |             | 31  R/W*
                   A14  19 |             | 30  OSC
                   A13  20 |             | 29  XTL1
                   A12  21 |             | 28  XTL2
                   A10  22 |             | 27  A0
                    A4  23 |             | 26  A1
                    A3  24 |_____________| 25  A2

**********************************************************************/

#ifndef MAME_MACHINE_MOS6526_H
#define MAME_MACHINE_MOS6526_H

#pragma once




//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_MOS6526_TOD(_clock) \
	mos6526_device::static_set_tod_clock(*device, _clock);

#define MCFG_MOS6526_IRQ_CALLBACK(_write) \
	devcb = &mos6526_device::set_irq_wr_callback(*device, DEVCB_##_write);

#define MCFG_MOS6526_CNT_CALLBACK(_write) \
	devcb = &mos6526_device::set_cnt_wr_callback(*device, DEVCB_##_write);

#define MCFG_MOS6526_SP_CALLBACK(_write) \
	devcb = &mos6526_device::set_sp_wr_callback(*device, DEVCB_##_write);

#define MCFG_MOS6526_PA_INPUT_CALLBACK(_read) \
	devcb = &mos6526_device::set_pa_rd_callback(*device, DEVCB_##_read);

#define MCFG_MOS6526_PA_OUTPUT_CALLBACK(_write) \
	devcb = &mos6526_device::set_pa_wr_callback(*device, DEVCB_##_write);

#define MCFG_MOS6526_PB_INPUT_CALLBACK(_read) \
	devcb = &mos6526_device::set_pb_rd_callback(*device, DEVCB_##_read);

#define MCFG_MOS6526_PB_OUTPUT_CALLBACK(_write) \
	devcb = &mos6526_device::set_pb_wr_callback(*device, DEVCB_##_write);

#define MCFG_MOS6526_PC_CALLBACK(_write) \
	devcb = &mos6526_device::set_pc_wr_callback(*device, DEVCB_##_write);



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> mos6526_device

class mos6526_device :  public device_t,
						public device_execute_interface
{
public:
	// construction/destruction
	mos6526_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void static_set_tod_clock(device_t &device, int clock) { downcast<mos6526_device &>(device).m_tod_clock = clock; }

	template <class Object> static devcb_base &set_irq_wr_callback(device_t &device, Object &&cb) { return downcast<mos6526_device &>(device).m_write_irq.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_cnt_wr_callback(device_t &device, Object &&cb) { return downcast<mos6526_device &>(device).m_write_cnt.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_sp_wr_callback(device_t &device, Object &&cb) { return downcast<mos6526_device &>(device).m_write_sp.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_pa_rd_callback(device_t &device, Object &&cb) { return downcast<mos6526_device &>(device).m_read_pa.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_pa_wr_callback(device_t &device, Object &&cb) { return downcast<mos6526_device &>(device).m_write_pa.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_pb_rd_callback(device_t &device, Object &&cb) { return downcast<mos6526_device &>(device).m_read_pb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_pb_wr_callback(device_t &device, Object &&cb) { return downcast<mos6526_device &>(device).m_write_pb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_pc_wr_callback(device_t &device, Object &&cb) { return downcast<mos6526_device &>(device).m_write_pc.set_callback(std::forward<Object>(cb)); }

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );

	uint8_t pa_r() { return m_pa; }
	DECLARE_READ8_MEMBER( pa_r ) { return pa_r(); }
	uint8_t pb_r() { return m_pb; }
	DECLARE_READ8_MEMBER( pb_r ) { return pb_r(); }

	DECLARE_READ_LINE_MEMBER( sp_r ) { return m_sp; }
	DECLARE_WRITE_LINE_MEMBER( sp_w );
	DECLARE_READ_LINE_MEMBER( cnt_r ) { return m_cnt; }
	DECLARE_WRITE_LINE_MEMBER( cnt_w );
	DECLARE_WRITE_LINE_MEMBER( flag_w );
	DECLARE_READ_LINE_MEMBER( irq_r ) { return m_irq; }
	DECLARE_WRITE_LINE_MEMBER( tod_w );

protected:
	enum
	{
		TYPE_6526,
		TYPE_6526A,
		TYPE_8520,
		TYPE_5710
	};

	mos6526_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, uint32_t variant);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	virtual void execute_run() override;

	int m_icount;
	const int m_variant;
	int m_tod_clock;

	inline void update_interrupt();
	inline void update_pa();
	inline void update_pb();
	inline void set_cra(uint8_t data);
	inline void set_crb(uint8_t data);
	inline void serial_input();
	inline void serial_output();
	inline void clock_ta();
	inline void clock_tb();
	inline void clock_pipeline();
	inline uint8_t bcd_increment(uint8_t value);
	virtual inline void clock_tod();
	inline uint8_t read_tod(int offset);
	inline void write_tod(int offset, uint8_t data);
	inline void synchronize();

	devcb_write_line   m_write_irq;
	devcb_write_line   m_write_pc;
	devcb_write_line   m_write_cnt;
	devcb_write_line   m_write_sp;
	devcb_read8        m_read_pa;
	devcb_write8       m_write_pa;
	devcb_read8        m_read_pb;
	devcb_write8       m_write_pb;

	// interrupts
	bool m_irq;
	int m_ir0;
	int m_ir1;
	uint8_t m_icr;
	uint8_t m_imr;
	bool m_icr_read;

	// peripheral ports
	int m_pc;
	int m_flag;
	uint8_t m_pra;
	uint8_t m_prb;
	uint8_t m_ddra;
	uint8_t m_ddrb;
	uint8_t m_pa;
	uint8_t m_pb;
	uint8_t m_pa_in;
	uint8_t m_pb_in;

	// serial
	int m_sp;
	int m_cnt;
	uint8_t m_sdr;
	uint8_t m_shift;
	bool m_sdr_empty;
	int m_bits;

	// timers
	int m_ta_out;
	int m_tb_out;
	int m_ta_pb6;
	int m_tb_pb7;
	int m_count_a0;
	int m_count_a1;
	int m_count_a2;
	int m_count_a3;
	int m_load_a0;
	int m_load_a1;
	int m_load_a2;
	int m_oneshot_a0;
	int m_count_b0;
	int m_count_b1;
	int m_count_b2;
	int m_count_b3;
	int m_load_b0;
	int m_load_b1;
	int m_load_b2;
	int m_oneshot_b0;
	uint16_t m_ta;
	uint16_t m_tb;
	uint16_t m_ta_latch;
	uint16_t m_tb_latch;
	uint8_t m_cra;
	uint8_t m_crb;

	// time-of-day
	int m_tod_count;
	uint32_t m_tod;
	uint32_t m_tod_latch;
	uint32_t m_alarm;
	bool m_tod_stopped;
	bool m_tod_latched;
	emu_timer *m_tod_timer;
};


// ======================> mos6526a_device

class mos6526a_device : public mos6526_device
{
public:
	mos6526a_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


// ======================> mos8520_device

class mos8520_device : public mos6526_device
{
public:
	mos8520_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );

protected:
	virtual inline void clock_tod() override;
};


// ======================> mos5710_device

class mos5710_device : public mos6526_device
{
public:
	mos5710_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	//DECLARE_READ8_MEMBER( read );
	//DECLARE_WRITE8_MEMBER( write );
};


// device type definition
DECLARE_DEVICE_TYPE(MOS6526,  mos6526_device)
DECLARE_DEVICE_TYPE(MOS6526A, mos6526a_device)
DECLARE_DEVICE_TYPE(MOS8520,  mos8520_device)
DECLARE_DEVICE_TYPE(MOS5710,  mos5710_device)

#endif // MAME_MACHINE_MOS6526_H
