// license:BSD-3-Clause
// copyright-holders: R. Belmont
/**********************************************************************

    MM5740 Keyboard Encoder emulation

**********************************************************************
                            _____   _____
                    B3   1 |*    \_/     | 40  B4
                   Vll   2 |             | 39  B9
                 Clock   3 |             | 38  B2
                    X9   4 |             | 37  B1
                    X8   5 |             | 36  B8
                    X7   6 |             | 35  B7
                    X6   7 |             | 34  B6
                    X5   8 |             | 33  B5
                    X4   9 |             | 32  Vss
                    X3  10 |    MM5740   | 31  Y9
                    X2  11 |             | 30  Y8
                    X1  12 |             | 29  Y7
    Data Strobe Output  13 |             | 28  Y6
   Data Strobe Control  14 |             | 27  Y5
         Output Enable  15 |             | 26  Y4
                Repeat  16 |             | 25  Y3
       Key Bounce Mask  17 |             | 24  Y2
                   Vgg  18 |             | 23  Y1
               Control  19 |             | 22  Y0
        Shift Lock I/O  20 |_____________| 21  Shift

Name                 Pin No.     Function
----------------------------------------------------------------------

X1-X9                4-12        Output - Drives the key switch matrix.

Y1-Y10               22-31       Inputs - connect to the X drive lines with
                 the key switch matrix.

B1-B9                1,33-40     Tri-stated data outputs.

Data Strobe Output   13          Output to indicate key pressed.

Data Strobe Control  14          Input to control data strobe output pulse width.

Output Enable        15          Input to control the chip's TRI-STATE output

Repeat               16          Each cycle of this signal will issue
                 a new data strobe for the pressed key.

Key-Bounce Mask      17          Use capacitor on this chip to provide
                 key debouncing

Shift                21          Shift key pressed

Control              19          Control key pressed

Shift Lock I/O       20          Togglable input to signify shift (NOT caps) lock.

Clock                3           A TTL compatible clock signal

Vss                  32          +5.0V

Vll                  2           Ground

Vgg                  18          -12V


**********************************************************************/

/* TODO:
    Support Key-bounce mask
    Support Repeat function
    Support shift lock
    Support additional internal ROMs
*/

#ifndef MAME_MACHINE_MM5740_H
#define MAME_MACHINE_MM5740_H

#pragma once


//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_MM5740_MATRIX_X1(_cb)       devcb = &mm5740_device::set_x_cb<0>(*device, DEVCB_##_cb);
#define MCFG_MM5740_MATRIX_X2(_cb)       devcb = &mm5740_device::set_x_cb<1>(*device, DEVCB_##_cb);
#define MCFG_MM5740_MATRIX_X3(_cb)       devcb = &mm5740_device::set_x_cb<2>(*device, DEVCB_##_cb);
#define MCFG_MM5740_MATRIX_X4(_cb)       devcb = &mm5740_device::set_x_cb<3>(*device, DEVCB_##_cb);
#define MCFG_MM5740_MATRIX_X5(_cb)       devcb = &mm5740_device::set_x_cb<4>(*device, DEVCB_##_cb);
#define MCFG_MM5740_MATRIX_X6(_cb)       devcb = &mm5740_device::set_x_cb<5>(*device, DEVCB_##_cb);
#define MCFG_MM5740_MATRIX_X7(_cb)       devcb = &mm5740_device::set_x_cb<6>(*device, DEVCB_##_cb);
#define MCFG_MM5740_MATRIX_X8(_cb)       devcb = &mm5740_device::set_x_cb<7>(*device, DEVCB_##_cb);
#define MCFG_MM5740_MATRIX_X9(_cb)       devcb = &mm5740_device::set_x_cb<8>(*device, DEVCB_##_cb);
#define MCFG_MM5740_SHIFT_CB(_cb)        devcb = &mm5740_device::set_shift_cb(*device, DEVCB_##_cb);
#define MCFG_MM5740_CONTROL_CB(_cb)      devcb = &mm5740_device::set_control_cb(*device, DEVCB_##_cb);
#define MCFG_MM5740_DATA_READY_CB(_cb)   devcb = &mm5740_device::set_data_ready_cb(*device, DEVCB_##_cb);

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> mm5740_device

class mm5740_device : public device_t
{
public:
	// construction/destruction
	mm5740_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// public interface
	uint16_t b_r();

	template <unsigned N, typename Object> static devcb_base &set_x_cb(device_t &device, Object &&cb)
	{
		return downcast<mm5740_device &>(device).m_read_x[N].set_callback(std::forward<Object>(cb));
	}
	template <typename Object> static devcb_base &set_shift_cb(device_t &device, Object &&cb)
	{
		return downcast<mm5740_device &>(device).m_read_shift.set_callback(std::forward<Object>(cb));
	}
	template <typename Object> static devcb_base &set_control_cb(device_t &device, Object &&cb)
	{
		return downcast<mm5740_device &>(device).m_read_control.set_callback(std::forward<Object>(cb));
	}
	template <typename Object> static devcb_base &set_data_ready_cb(device_t &device, Object &&cb)
	{
		return downcast<mm5740_device &>(device).m_write_data_ready.set_callback(std::forward<Object>(cb));
	}
	static uint32_t calc_effective_clock_key_debounce(uint32_t capacitance);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	virtual const tiny_rom_entry *device_rom_region() const override;

private:
	devcb_read16 m_read_x[9];
	devcb_read_line m_read_shift, m_read_control;
	devcb_write_line m_write_data_ready;

	required_memory_region m_rom;                   // Internal ROM

	int m_b;                    // output buffer

	int m_x_mask[9];            // mask of what keys are down

	// timers
	emu_timer *m_scan_timer;    // keyboard scan timer
};


// device type definition
DECLARE_DEVICE_TYPE(MM5740, mm5740_device)

#endif // MAME_MACHINE_MM5740_H
