// license:BSD-3-Clause
// copyright-holders:James Wallace
/***************************************************************************

    Motorola 6840 (PTM)

    Programmable Timer Module

***************************************************************************/

#ifndef MAME_MACHINE_6840PTM_H
#define MAME_MACHINE_6840PTM_H

#pragma once



//**************************************************************************
//  DEVICE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_PTM6840_EXTERNAL_CLOCKS(_clk0, _clk1, _clk2) \
	ptm6840_device::set_external_clocks(*device, _clk0, _clk1, _clk2);

#define MCFG_PTM6840_OUT0_CB(_devcb) \
	devcb = &ptm6840_device::set_out_callback(*device, 0, DEVCB_##_devcb);

#define MCFG_PTM6840_OUT1_CB(_devcb) \
	devcb = &ptm6840_device::set_out_callback(*device, 1, DEVCB_##_devcb);

#define MCFG_PTM6840_OUT2_CB(_devcb) \
	devcb = &ptm6840_device::set_out_callback(*device, 2, DEVCB_##_devcb);

#define MCFG_PTM6840_IRQ_CB(_devcb) \
	devcb = &ptm6840_device::set_irq_callback(*device, DEVCB_##_devcb);

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> ptm6840_device

class ptm6840_device :  public device_t
{
public:
	// construction/destruction
	ptm6840_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void set_external_clocks(device_t &device, double clock0, double clock1, double clock2) { downcast<ptm6840_device &>(device).m_external_clock[0] = clock0; downcast<ptm6840_device &>(device).m_external_clock[1] = clock1; downcast<ptm6840_device &>(device).m_external_clock[2] = clock2; }
	template <class Object> static devcb_base &set_out_callback(device_t &device, int index, Object &&cb) { return downcast<ptm6840_device &>(device).m_out_cb[index].set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_irq_callback(device_t &device, Object &&cb) { return downcast<ptm6840_device &>(device).m_irq_cb.set_callback(std::forward<Object>(cb)); }

	int status(int clock) const { return m_enabled[clock]; } // get whether timer is enabled
	int irq_state() const { return m_irq; }                 // get IRQ state
	uint16_t count(int counter) const { return compute_counter(counter); }    // get counter value
	void set_ext_clock(int counter, double clock);  // set clock frequency
	int ext_clock(int counter) const { return m_external_clock[counter]; }  // get clock frequency

	DECLARE_WRITE8_MEMBER( write );
	void write(offs_t offset, uint8_t data) { write(machine().dummy_space(), offset, data); }
	DECLARE_READ8_MEMBER( read );
	uint8_t read(offs_t offset) { return read(machine().dummy_space(), offset); }

	void set_gate(int idx, int state);
	DECLARE_WRITE_LINE_MEMBER( set_g1 );
	DECLARE_WRITE_LINE_MEMBER( set_g2 );
	DECLARE_WRITE_LINE_MEMBER( set_g3 );

	void set_clock(int idx, int state);
	DECLARE_WRITE_LINE_MEMBER( set_c1 );
	DECLARE_WRITE_LINE_MEMBER( set_c2 );
	DECLARE_WRITE_LINE_MEMBER( set_c3 );

	void update_interrupts();

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	void subtract_from_counter(int counter, int count);
	void tick(int counter, int count);
	void timeout(int idx);

	uint16_t compute_counter(int counter) const;
	void reload_count(int idx);

	enum
	{
		PTM_6840_CTRL1   = 0,
		PTM_6840_CTRL2   = 1,
		PTM_6840_STATUS  = 1,
		PTM_6840_MSBBUF1 = 2,
		PTM_6840_LSB1    = 3,
		PTM_6840_MSBBUF2 = 4,
		PTM_6840_LSB2    = 5,
		PTM_6840_MSBBUF3 = 6,
		PTM_6840_LSB3    = 7
	};

	enum
	{
		RESET_TIMERS    = 0x01,
		CR1_SELECT      = 0x01,
		T3_PRESCALE_EN  = 0x01,
		INTERNAL_CLK_EN = 0x02,
		COUNT_MODE_8BIT = 0x04,
		INTERRUPT_EN    = 0x40,
		COUNT_OUT_EN    = 0x80
	};

	enum
	{
		TIMER1_IRQ  = 0x01,
		TIMER2_IRQ  = 0x02,
		TIMER3_IRQ  = 0x04,
		ANY_IRQ     = 0x80
	};

	double m_external_clock[3];

	devcb_write_line m_out_cb[3];
	devcb_write_line m_irq_cb;

	uint8_t m_control_reg[3];
	uint8_t m_output[3]; // Output states
	uint8_t m_gate[3];   // Input gate states
	uint8_t m_clk[3];  // Clock states
	uint8_t m_enabled[3];
	uint8_t m_mode[3];
	uint8_t m_fired[3];
	uint8_t m_t3_divisor;
	uint8_t m_t3_scaler;
	uint8_t m_irq;
	uint8_t m_status_reg;
	uint8_t m_status_read_since_int;
	uint8_t m_lsb_buffer;
	uint8_t m_msb_buffer;

	// Each PTM has 3 timers
	emu_timer *m_timer[3];

	uint16_t m_latch[3];
	uint16_t m_counter[3];

	static const char *const opmode[];
};


// device type definition
DECLARE_DEVICE_TYPE(PTM6840, ptm6840_device)

#endif // MAME_MACHINE_6840PTM_H
