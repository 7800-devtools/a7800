// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Intel 8155/8156 - 2048-Bit Static MOS RAM with I/O Ports and Timer emulation
    8156 is the same as 8155, except that chip enable is active high instead of low

**********************************************************************
                            _____   _____
                   PC3   1 |*    \_/     | 40  Vcc
                   PC4   2 |             | 39  PC2
              TIMER IN   3 |             | 38  PC1
                 RESET   4 |             | 37  PC0
                   PC5   5 |             | 36  PB7
            _TIMER OUT   6 |             | 35  PB6
                 IO/_M   7 |             | 34  PB5
             CE or _CE   8 |             | 33  PB4
                   _RD   9 |             | 32  PB3
                   _WR  10 |    8155     | 31  PB2
                   ALE  11 |    8156     | 30  PB1
                   AD0  12 |             | 29  PB0
                   AD1  13 |             | 28  PA7
                   AD2  14 |             | 27  PA6
                   AD3  15 |             | 26  PA5
                   AD4  16 |             | 25  PA4
                   AD5  17 |             | 24  PA3
                   AD6  18 |             | 23  PA2
                   AD7  19 |             | 22  PA1
                   Vss  20 |_____________| 21  PA0

**********************************************************************/

#ifndef MAME_MACHINE_I8155_H
#define MAME_MACHINE_I8155_H

#pragma once



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_I8155_IN_PORTA_CB(_devcb) \
	devcb = &i8155_device::set_in_pa_callback(*device, DEVCB_##_devcb);

#define MCFG_I8155_IN_PORTB_CB(_devcb) \
	devcb = &i8155_device::set_in_pb_callback(*device, DEVCB_##_devcb);

#define MCFG_I8155_IN_PORTC_CB(_devcb) \
	devcb = &i8155_device::set_in_pc_callback(*device, DEVCB_##_devcb);

#define MCFG_I8155_OUT_PORTA_CB(_devcb) \
	devcb = &i8155_device::set_out_pa_callback(*device, DEVCB_##_devcb);

#define MCFG_I8155_OUT_PORTB_CB(_devcb) \
	devcb = &i8155_device::set_out_pb_callback(*device, DEVCB_##_devcb);

#define MCFG_I8155_OUT_PORTC_CB(_devcb) \
	devcb = &i8155_device::set_out_pc_callback(*device, DEVCB_##_devcb);

#define MCFG_I8155_OUT_TIMEROUT_CB(_devcb) \
	devcb = &i8155_device::set_out_to_callback(*device, DEVCB_##_devcb);


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> i8155_device

class i8155_device :    public device_t,
						public device_memory_interface
{
public:
	// construction/destruction
	i8155_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_in_pa_callback(device_t &device, Object &&cb)  { return downcast<i8155_device &>(device).m_in_pa_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_in_pb_callback(device_t &device, Object &&cb)  { return downcast<i8155_device &>(device).m_in_pb_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_in_pc_callback(device_t &device, Object &&cb)  { return downcast<i8155_device &>(device).m_in_pc_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_pa_callback(device_t &device, Object &&cb) { return downcast<i8155_device &>(device).m_out_pa_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_pb_callback(device_t &device, Object &&cb) { return downcast<i8155_device &>(device).m_out_pb_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_pc_callback(device_t &device, Object &&cb) { return downcast<i8155_device &>(device).m_out_pc_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_to_callback(device_t &device, Object &&cb) { return downcast<i8155_device &>(device).m_out_to_cb.set_callback(std::forward<Object>(cb)); }

	DECLARE_READ8_MEMBER( io_r );
	DECLARE_WRITE8_MEMBER( io_w );

	DECLARE_READ8_MEMBER( memory_r );
	DECLARE_WRITE8_MEMBER( memory_w );

	DECLARE_WRITE8_MEMBER( ale_w );
	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	virtual space_config_vector memory_space_config() const override;

	inline uint8_t get_timer_mode();
	inline void timer_output();
	inline void pulse_timer_output();
	inline int get_port_mode(int port);
	inline uint8_t read_port(int port);
	inline void write_port(int port, uint8_t data);

	void register_w(int offset, uint8_t data);

private:
	devcb_read8        m_in_pa_cb;
	devcb_read8        m_in_pb_cb;
	devcb_read8        m_in_pc_cb;

	devcb_write8       m_out_pa_cb;
	devcb_write8       m_out_pb_cb;
	devcb_write8       m_out_pc_cb;

	// this gets called for each change of the TIMER OUT pin (pin 6)
	devcb_write_line   m_out_to_cb;

	// CPU interface
	int m_io_m;                 // I/O or memory select
	uint8_t m_ad;                 // address

	// registers
	uint8_t m_command;            // command register
	uint8_t m_status;             // status register
	uint8_t m_output[3];          // output latches

	// counter
	uint16_t m_count_length;      // count length register
	uint16_t m_counter;           // counter register
	int m_to;                   // timer output

	// timers
	emu_timer *m_timer;         // counter timer

	const address_space_config      m_space_config;
};


// device type definition
DECLARE_DEVICE_TYPE(I8155, i8155_device)
extern const device_type I8156;


#endif
