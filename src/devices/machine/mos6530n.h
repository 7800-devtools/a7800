// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    MOS Technology 6530 Memory, I/O, Timer Array emulation
    MOS Technology 6532 RAM, I/O, Timer Array emulation

**********************************************************************
                            _____   _____
                   Vss   1 |*    \_/     | 40  PA1
                   PA0   2 |             | 39  PA2
                  phi2   3 |             | 38  PA3
                   RS0   4 |             | 37  PA4
                    A9   5 |             | 36  PA5
                    A8   6 |             | 35  PA6
                    A7   7 |             | 34  PA7
                    A6   8 |             | 33  DB0
                   R/W   9 |             | 32  DB1
                    A5  10 |   MCS6530   | 31  DB2
                    A4  11 |             | 30  DB3
                    A3  12 |             | 29  DB4
                    A2  13 |             | 28  DB5
                    A1  14 |             | 27  DB6
                    A0  15 |             | 26  DB7
                  _RES  16 |             | 25  PB0
               IRQ/PB7  17 |             | 24  PB1
               CS1/PB6  18 |             | 23  PB2
               CS2/PB5  19 |             | 22  PB3
                   Vcc  20 |_____________| 21  PB4

                            _____   _____
                   Vss   1 |*    \_/     | 40  A6
                    A5   2 |             | 39  phi2
                    A4   3 |             | 38  CS1
                    A3   4 |             | 37  _CS2
                    A2   5 |             | 36  _RS
                    A1   6 |             | 35  R/W
                    A0   7 |             | 34  _RES
                   PA0   8 |             | 33  D0
                   PA1   9 |             | 32  D1
                   PA2  10 |   MCS6532   | 31  D2
                   PA3  11 |             | 30  D3
                   PA4  12 |             | 29  D4
                   PA5  13 |             | 28  D5
                   PA6  14 |             | 27  D6
                   PA7  15 |             | 26  D7
                   PB7  16 |             | 25  _IRQ
                   PB6  17 |             | 24  PB0
                   PB5  18 |             | 23  PB1
                   PB4  19 |             | 22  PB2
                   Vcc  20 |_____________| 21  PB3

**********************************************************************/

#ifndef MAME_MACHINE_MOS6530N_H
#define MAME_MACHINE_MOS6530N_H

#pragma once



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_MOS6530n_IRQ_CB(_write) \
	devcb = &mos6530_device_base::set_irq_wr_callback(*device, DEVCB_##_write);

#define MCFG_MOS6530n_IN_PA_CB(_read) \
	devcb = &mos6530_device_base::set_pa_rd_callback(*device, DEVCB_##_read);

#define MCFG_MOS6530n_OUT_PA_CB(_write) \
	devcb = &mos6530_device_base::set_pa_wr_callback(*device, DEVCB_##_write);

#define MCFG_MOS6530n_IN_PB_CB(_read) \
	devcb = &mos6530_device_base::set_pb_rd_callback(*device, DEVCB_##_read);

#define MCFG_MOS6530n_OUT_PB_CB(_write) \
	devcb = &mos6530_device_base::set_pb_wr_callback(*device, DEVCB_##_write);

#define MCFG_MOS6530n_IN_PA0_CB(_read) \
	devcb = &mos6530_device_base::set_pa_rd_callback<0>(*device, DEVCB_##_read);

#define MCFG_MOS6530n_IN_PA1_CB(_read) \
	devcb = &mos6530_device_base::set_pa_rd_callback<1>(*device, DEVCB_##_read);

#define MCFG_MOS6530n_IN_PA2_CB(_read) \
	devcb = &mos6530_device_base::set_pa_rd_callback<2>(*device, DEVCB_##_read);

#define MCFG_MOS6530n_IN_PA3_CB(_read) \
	devcb = &mos6530_device_base::set_pa_rd_callback<3>(*device, DEVCB_##_read);

#define MCFG_MOS6530n_IN_PA4_CB(_read) \
	devcb = &mos6530_device_base::set_pa_rd_callback<4>(*device, DEVCB_##_read);

#define MCFG_MOS6530n_IN_PA5_CB(_read) \
	devcb = &mos6530_device_base::set_pa_rd_callback<5>(*device, DEVCB_##_read);

#define MCFG_MOS6530n_IN_PA6_CB(_read) \
	devcb = &mos6530_device_base::set_pa_rd_callback<6>(*device, DEVCB_##_read);

#define MCFG_MOS6530n_IN_PA7_CB(_read) \
	devcb = &mos6530_device_base::set_pa_rd_callback<7>(*device, DEVCB_##_read);

#define MCFG_MOS6530n_OUT_PA0_CB(_write) \
	devcb = &mos6530_device_base::set_pa_wr_callback<0>(*device, DEVCB_##_write);

#define MCFG_MOS6530n_OUT_PA1_CB(_write) \
	devcb = &mos6530_device_base::set_pa_wr_callback<1>(*device, DEVCB_##_write);

#define MCFG_MOS6530n_OUT_PA2_CB(_write) \
	devcb = &mos6530_device_base::set_pa_wr_callback<2>(*device, DEVCB_##_write);

#define MCFG_MOS6530n_OUT_PA3_CB(_write) \
	devcb = &mos6530_device_base::set_pa_wr_callback<3>(*device, DEVCB_##_write);

#define MCFG_MOS6530n_OUT_PA4_CB(_write) \
	devcb = &mos6530_device_base::set_pa_wr_callback<4>(*device, DEVCB_##_write);

#define MCFG_MOS6530n_OUT_PA5_CB(_write) \
	devcb = &mos6530_device_base::set_pa_wr_callback<5>(*device, DEVCB_##_write);

#define MCFG_MOS6530n_OUT_PA6_CB(_write) \
	devcb = &mos6530_device_base::set_pa_wr_callback<6>(*device, DEVCB_##_write);

#define MCFG_MOS6530n_OUT_PA7_CB(_write) \
	devcb = &mos6530_device_base::set_pa_wr_callback<7>(*device, DEVCB_##_write);

#define MCFG_MOS6530n_IN_PB0_CB(_read) \
	devcb = &mos6530_device_base::set_pb_rd_callback<0>(*device, DEVCB_##_read);

#define MCFG_MOS6530n_IN_PB1_CB(_read) \
	devcb = &mos6530_device_base::set_pb_rd_callback<1>(*device, DEVCB_##_read);

#define MCFG_MOS6530n_IN_PB2_CB(_read) \
	devcb = &mos6530_device_base::set_pb_rd_callback<2>(*device, DEVCB_##_read);

#define MCFG_MOS6530n_IN_PB3_CB(_read) \
	devcb = &mos6530_device_base::set_pb_rd_callback<3>(*device, DEVCB_##_read);

#define MCFG_MOS6530n_IN_PB4_CB(_read) \
	devcb = &mos6530_device_base::set_pb_rd_callback<4>(*device, DEVCB_##_read);

#define MCFG_MOS6530n_IN_PB5_CB(_read) \
	devcb = &mos6530_device_base::set_pb_rd_callback<5>(*device, DEVCB_##_read);

#define MCFG_MOS6530n_IN_PB6_CB(_read) \
	devcb = &mos6530_device_base::set_pb_rd_callback<6>(*device, DEVCB_##_read);

#define MCFG_MOS6530n_IN_PB7_CB(_read) \
	devcb = &mos6530_device_base::set_pb_rd_callback<7>(*device, DEVCB_##_read);

#define MCFG_MOS6530n_OUT_PB0_CB(_write) \
	devcb = &mos6530_device_base::set_pb_wr_callback<0>(*device, DEVCB_##_write);

#define MCFG_MOS6530n_OUT_PB1_CB(_write) \
	devcb = &mos6530_device_base::set_pb_wr_callback<1>(*device, DEVCB_##_write);

#define MCFG_MOS6530n_OUT_PB2_CB(_write) \
	devcb = &mos6530_device_base::set_pb_wr_callback<2>(*device, DEVCB_##_write);

#define MCFG_MOS6530n_OUT_PB3_CB(_write) \
	devcb = &mos6530_device_base::set_pb_wr_callback<3>(*device, DEVCB_##_write);

#define MCFG_MOS6530n_OUT_PB4_CB(_write) \
	devcb = &mos6530_device_base::set_pb_wr_callback<4>(*device, DEVCB_##_write);

#define MCFG_MOS6530n_OUT_PB5_CB(_write) \
	devcb = &mos6530_device_base::set_pb_wr_callback<5>(*device, DEVCB_##_write);

#define MCFG_MOS6530n_OUT_PB6_CB(_write) \
	devcb = &mos6530_device_base::set_pb_wr_callback<6>(*device, DEVCB_##_write);

#define MCFG_MOS6530n_OUT_PB7_CB(_write) \
	devcb = &mos6530_device_base::set_pb_wr_callback<7>(*device, DEVCB_##_write);



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> mos6530_device_base

class mos6530_device_base :  public device_t
{
public:
	template <class Object> static devcb_base &set_irq_wr_callback(device_t &device, Object &&cb) { return downcast<mos6530_device_base &>(device).m_irq_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_pa_rd_callback(device_t &device, Object &&cb) { return downcast<mos6530_device_base &>(device).m_in8_pa_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_pa_wr_callback(device_t &device, Object &&cb) { return downcast<mos6530_device_base &>(device).m_out8_pa_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_pb_rd_callback(device_t &device, Object &&cb) { return downcast<mos6530_device_base &>(device).m_in8_pb_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_pb_wr_callback(device_t &device, Object &&cb) { return downcast<mos6530_device_base &>(device).m_out8_pb_cb.set_callback(std::forward<Object>(cb)); }
	template <unsigned N, class Object> static devcb_base &set_pa_rd_callback(device_t &device, Object &&cb) { return downcast<mos6530_device_base &>(device).m_in_pa_cb[N].set_callback(std::forward<Object>(cb)); }
	template <unsigned N, class Object> static devcb_base &set_pa_wr_callback(device_t &device, Object &&cb) { return downcast<mos6530_device_base &>(device).m_out_pa_cb[N].set_callback(std::forward<Object>(cb)); }
	template <unsigned N, class Object> static devcb_base &set_pb_rd_callback(device_t &device, Object &&cb) { return downcast<mos6530_device_base &>(device).m_in_pb_cb[N].set_callback(std::forward<Object>(cb)); }
	template <unsigned N, class Object> static devcb_base &set_pb_wr_callback(device_t &device, Object &&cb) { return downcast<mos6530_device_base &>(device).m_out_pb_cb[N].set_callback(std::forward<Object>(cb)); }

	DECLARE_WRITE_LINE_MEMBER( pa0_w ) { pa_w(0, state); }
	DECLARE_WRITE_LINE_MEMBER( pa1_w ) { pa_w(1, state); }
	DECLARE_WRITE_LINE_MEMBER( pa2_w ) { pa_w(2, state); }
	DECLARE_WRITE_LINE_MEMBER( pa3_w ) { pa_w(3, state); }
	DECLARE_WRITE_LINE_MEMBER( pa4_w ) { pa_w(4, state); }
	DECLARE_WRITE_LINE_MEMBER( pa5_w ) { pa_w(5, state); }
	DECLARE_WRITE_LINE_MEMBER( pa6_w ) { pa_w(6, state); }
	DECLARE_WRITE_LINE_MEMBER( pa7_w ) { pa_w(7, state); }

	DECLARE_WRITE_LINE_MEMBER( pb0_w ) { pb_w(0, state); }
	DECLARE_WRITE_LINE_MEMBER( pb1_w ) { pb_w(1, state); }
	DECLARE_WRITE_LINE_MEMBER( pb2_w ) { pb_w(2, state); }
	DECLARE_WRITE_LINE_MEMBER( pb3_w ) { pb_w(3, state); }
	DECLARE_WRITE_LINE_MEMBER( pb4_w ) { pb_w(4, state); }
	DECLARE_WRITE_LINE_MEMBER( pb5_w ) { pb_w(5, state); }
	DECLARE_WRITE_LINE_MEMBER( pb6_w ) { pb_w(6, state); }
	DECLARE_WRITE_LINE_MEMBER( pb7_w ) { pb_w(7, state); }

protected:
	// construction/destruction
	mos6530_device_base(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	enum
	{
		IRQ_EDGE  = 0x40,
		IRQ_TIMER = 0x80
	};

	void update_pa();
	virtual void update_pb();
	virtual void update_irq();
	virtual uint8_t get_irq_flags();
	void edge_detect();
	void pa_w(int bit, int state);
	void pb_w(int bit, int state);
	void timer_w(offs_t offset, uint8_t data, bool ie);
	uint8_t timer_r(bool ie);

	DECLARE_READ8_MEMBER( rom_r ) { return m_rom[offset]; }
	DECLARE_READ8_MEMBER( ram_r ) { return m_ram[offset]; }
	DECLARE_WRITE8_MEMBER( ram_w ) { m_ram[offset] = data; }
	DECLARE_READ8_MEMBER( pa_data_r );
	DECLARE_WRITE8_MEMBER( pa_data_w );
	DECLARE_READ8_MEMBER( pb_data_r );
	DECLARE_WRITE8_MEMBER( pb_data_w );
	DECLARE_READ8_MEMBER( pa_ddr_r ) { return m_pa_ddr; }
	DECLARE_WRITE8_MEMBER( pa_ddr_w );
	DECLARE_READ8_MEMBER( pb_ddr_r ) { return m_pb_ddr; }
	DECLARE_WRITE8_MEMBER( pb_ddr_w );
	DECLARE_READ8_MEMBER( timer_off_r );
	DECLARE_READ8_MEMBER( timer_on_r );
	DECLARE_READ8_MEMBER( irq_r );
	DECLARE_WRITE8_MEMBER( timer_off_w );
	DECLARE_WRITE8_MEMBER( timer_on_w );
	DECLARE_WRITE8_MEMBER( edge_w );

	optional_shared_ptr<uint8_t> m_ram;
	optional_region_ptr<uint8_t> m_rom;

	devcb_write_line m_irq_cb;
	devcb_read8    m_in8_pa_cb;
	devcb_write8   m_out8_pa_cb;
	devcb_read8    m_in8_pb_cb;
	devcb_write8   m_out8_pb_cb;
	devcb_read_line m_in_pa_cb[8];
	devcb_write_line m_out_pa_cb[8];
	devcb_read_line m_in_pb_cb[8];
	devcb_write_line m_out_pb_cb[8];

	uint8_t m_pa_in;
	uint8_t m_pa_out;
	uint8_t m_pa_ddr;
	int m_pa7;
	int m_pa7_dir;

	uint8_t m_pb_in;
	uint8_t m_pb_out;
	uint8_t m_pb_ddr;

	bool m_ie_timer;
	bool m_irq_timer;
	bool m_ie_edge;
	bool m_irq_edge;

	int m_prescale;
	uint8_t m_timer;

	enum {
		IDLE,
		RUNNING,
		RUNNING_SYNCPOINT,
		RUNNING_AFTER_INTERRUPT
	};

	struct live_info {
		attotime tm, tm_irq;
		attotime period;
		int state, next_state;
		uint8_t value;
	};

	live_info cur_live, checkpoint_live;
	emu_timer *t_gen;

	void live_start();
	void checkpoint();
	void rollback();
	void live_delay(int state);
	void live_sync();
	void live_abort();
	void live_run(const attotime &limit = attotime::never);
};


class mos6530_new_device :  public mos6530_device_base
{
public:
	// construction/destruction
	mos6530_new_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_ADDRESS_MAP(rom_map, 8);
	virtual DECLARE_ADDRESS_MAP(ram_map, 8);
	virtual DECLARE_ADDRESS_MAP(io_map, 8);

protected:
	// device-level overrides
	virtual void device_start() override;

	void update_pb() override;
	void update_irq() override;
	uint8_t get_irq_flags() override;
};


class mos6532_new_device :  public mos6530_device_base
{
public:
	// construction/destruction
	mos6532_new_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_ADDRESS_MAP(ram_map, 8);
	virtual DECLARE_ADDRESS_MAP(io_map, 8);

	// is there a better way to access the memory map when not using AM_DEVICE?
	DECLARE_READ8_MEMBER(io_r);
	DECLARE_WRITE8_MEMBER(io_w);

protected:
	// device-level overrides
	virtual void device_start() override;
};


// device type definition
DECLARE_DEVICE_TYPE(MOS6530_NEW, mos6530_new_device)
DECLARE_DEVICE_TYPE(MOS6532_NEW, mos6532_new_device)

#endif // MAME_MACHINE_MOS6530N_H
