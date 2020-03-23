// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Intel 8355 - 16,384-Bit ROM with I/O emulation

**********************************************************************
                            _____   _____
                  _CE1   1 |*    \_/     | 40  Vcc
                   CE2   2 |             | 39  PB7
                   CLK   3 |             | 38  PB6
                 RESET   4 |             | 37  PB5
                  N.C.   5 |             | 36  PB4
                 READY   6 |             | 35  PB3
                 IO/_M   7 |             | 34  PB2
                  _IOR   8 |             | 33  PB1
                   _RD   9 |             | 32  PB0
                  _IOW  10 |    8355     | 31  PA7
                   ALE  11 |    8355-2   | 30  PA6
                   AD0  12 |             | 29  PA5
                   AD1  13 |             | 28  PA4
                   AD2  14 |             | 27  PA3
                   AD3  15 |             | 26  PA2
                   AD4  16 |             | 25  PA1
                   AD5  17 |             | 24  PA0
                   AD6  18 |             | 23  A10
                   AD7  19 |             | 22  A9
                   Vss  20 |_____________| 21  A8

**********************************************************************/

#ifndef MAME_MACHINE_I8355_H
#define MAME_MACHINE_I8355_H

#pragma once




///*************************************************************************
//  MACROS / CONSTANTS
///*************************************************************************




///*************************************************************************
//  INTERFACE CONFIGURATION MACROS
///*************************************************************************

#define MCFG_I8355_IN_PA_CB(_devcb) \
	devcb = &i8355_device::set_in_pa_callback(*device, DEVCB_##_devcb);

#define MCFG_I8355_OUT_PA_CB(_devcb) \
	devcb = &i8355_device::set_out_pa_callback(*device, DEVCB_##_devcb);

#define MCFG_I8355_IN_PB_CB(_devcb) \
	devcb = &i8355_device::set_in_pb_callback(*device, DEVCB_##_devcb);

#define MCFG_I8355_OUT_PB_CB(_devcb) \
	devcb = &i8355_device::set_out_pb_callback(*device, DEVCB_##_devcb);


///*************************************************************************
//  TYPE DEFINITIONS
///*************************************************************************

// ======================> i8355_device

class i8355_device : public device_t
{
public:
	// construction/destruction
	i8355_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_in_pa_callback(device_t &device, Object &&cb) { return downcast<i8355_device &>(device).m_in_pa_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_pa_callback(device_t &device, Object &&cb) { return downcast<i8355_device &>(device).m_out_pa_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_in_pb_callback(device_t &device, Object &&cb) { return downcast<i8355_device &>(device).m_in_pb_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_pb_callback(device_t &device, Object &&cb) { return downcast<i8355_device &>(device).m_out_pb_cb.set_callback(std::forward<Object>(cb)); }

	DECLARE_READ8_MEMBER( io_r );
	DECLARE_WRITE8_MEMBER( io_w );

	DECLARE_READ8_MEMBER( memory_r );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	inline uint8_t read_port(int port);
	inline void write_port(int port, uint8_t data);

private:
	devcb_read8             m_in_pa_cb;
	devcb_write8            m_out_pa_cb;

	devcb_read8             m_in_pb_cb;
	devcb_write8            m_out_pb_cb;

	// registers
	uint8_t m_output[2];          // output latches
	uint8_t m_ddr[2];             // DDR latches

	// internal ROM
	required_region_ptr<uint8_t> m_rom;
};


// device type definition
DECLARE_DEVICE_TYPE(I8355, i8355_device)

#endif // MAME_MACHINE_I8355_H
