// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Intel 8255(A) Programmable Peripheral Interface emulation

**********************************************************************
                            _____   _____
                   PA3   1 |*    \_/     | 40  PA4
                   PA2   2 |             | 39  PA5
                   PA1   3 |             | 38  PA6
                   PA0   4 |             | 37  PA7
                   _RD   5 |             | 36  WR
                   _CS   6 |             | 35  RESET
                   GND   7 |             | 34  D0
                    A1   8 |             | 33  D1
                    A0   9 |             | 32  D2
                   PC7  10 |    8255     | 31  D3
                   PC6  11 |    8255A    | 30  D4
                   PC5  12 |             | 29  D5
                   PC4  13 |             | 28  D6
                   PC0  14 |             | 27  D7
                   PC1  15 |             | 26  Vcc
                   PC2  16 |             | 25  PB7
                   PC3  17 |             | 24  PB6
                   PB0  18 |             | 23  PB5
                   PB1  19 |             | 22  PB4
                   PB2  20 |_____________| 21  PB3

**********************************************************************/

#ifndef MAME_MACHINE_I8255_H
#define MAME_MACHINE_I8255_H

#pragma once


//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_I8255_IN_PORTA_CB(_devcb) \
	devcb = &i8255_device::set_in_pa_callback(*device, DEVCB_##_devcb);

#define MCFG_I8255_IN_PORTB_CB(_devcb) \
	devcb = &i8255_device::set_in_pb_callback(*device, DEVCB_##_devcb);

#define MCFG_I8255_IN_PORTC_CB(_devcb) \
	devcb = &i8255_device::set_in_pc_callback(*device, DEVCB_##_devcb);

#define MCFG_I8255_OUT_PORTA_CB(_devcb) \
	devcb = &i8255_device::set_out_pa_callback(*device, DEVCB_##_devcb);

#define MCFG_I8255_OUT_PORTB_CB(_devcb) \
	devcb = &i8255_device::set_out_pb_callback(*device, DEVCB_##_devcb);

#define MCFG_I8255_OUT_PORTC_CB(_devcb) \
	devcb = &i8255_device::set_out_pc_callback(*device, DEVCB_##_devcb);

// output state when pins are in tri-state, default 0xff
#define MCFG_I8255_TRISTATE_PORTA_CB(_devcb) \
	devcb = &i8255_device::set_tri_pa_callback(*device, DEVCB_##_devcb);

#define MCFG_I8255_TRISTATE_PORTB_CB(_devcb) \
	devcb = &i8255_device::set_tri_pb_callback(*device, DEVCB_##_devcb);

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> i8255_device

class i8255_device : public device_t
{
public:
	// construction/destruction
	i8255_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_in_pa_callback(device_t &device, Object &&cb)  { return downcast<i8255_device &>(device).m_in_pa_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_in_pb_callback(device_t &device, Object &&cb)  { return downcast<i8255_device &>(device).m_in_pb_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_in_pc_callback(device_t &device, Object &&cb)  { return downcast<i8255_device &>(device).m_in_pc_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_pa_callback(device_t &device, Object &&cb) { return downcast<i8255_device &>(device).m_out_pa_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_pb_callback(device_t &device, Object &&cb) { return downcast<i8255_device &>(device).m_out_pb_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_pc_callback(device_t &device, Object &&cb) { return downcast<i8255_device &>(device).m_out_pc_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_tri_pa_callback(device_t &device, Object &&cb) { return downcast<i8255_device &>(device).m_tri_pa_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_tri_pb_callback(device_t &device, Object &&cb) { return downcast<i8255_device &>(device).m_tri_pb_cb.set_callback(std::forward<Object>(cb)); }

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );

	DECLARE_READ8_MEMBER( pa_r );
	uint8_t pa_r();

	DECLARE_READ8_MEMBER( pb_r );
	uint8_t pb_r();

	DECLARE_WRITE_LINE_MEMBER( pc2_w );
	DECLARE_WRITE_LINE_MEMBER( pc4_w );
	DECLARE_WRITE_LINE_MEMBER( pc6_w );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	inline void check_interrupt(int port);
	inline void set_ibf(int port, int state);
	inline void set_obf(int port, int state);
	inline void set_inte(int port, int state);
	inline void set_inte1(int state);
	inline void set_inte2(int state);
	inline void set_intr(int port, int state);
	inline int group_mode(int group);
	inline int port_mode(int port);
	inline int port_c_lower_mode();
	inline int port_c_upper_mode();

	uint8_t read_mode0(int port);
	uint8_t read_mode1(int port);
	uint8_t read_mode2();
	uint8_t read_pc();
	void write_mode0(int port, uint8_t data);
	void write_mode1(int port, uint8_t data);
	void write_mode2(uint8_t data);
	void output_pc();
	void set_mode(uint8_t data);
	void set_pc_bit(int bit, int state);

	devcb_read8        m_in_pa_cb;
	devcb_read8        m_in_pb_cb;
	devcb_read8        m_in_pc_cb;

	devcb_write8       m_out_pa_cb;
	devcb_write8       m_out_pb_cb;
	devcb_write8       m_out_pc_cb;

	devcb_read8        m_tri_pa_cb;
	devcb_read8        m_tri_pb_cb;

	uint8_t m_control;            // mode control word
	uint8_t m_output[3];          // output latch
	uint8_t m_input[3];           // input latch

	int m_ibf[2];               // input buffer full flag
	int m_obf[2];               // output buffer full flag, negative logic
	int m_inte[2];              // interrupt enable
	int m_inte1;                // interrupt enable
	int m_inte2;                // interrupt enable
	int m_intr[2];              // interrupt
};


// device type definition
DECLARE_DEVICE_TYPE(I8255, i8255_device)
extern const device_type I8255A;

#endif // MAME_MACHINE_I8255_H
