// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Nokia MikroMikko 1 keyboard emulation

*********************************************************************/
#ifndef MAME_MACHINE_MM1KB_H
#define MAME_MACHINE_MM1KB_H

#pragma once


#include "sound/samples.h"



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_MM1_KEYBOARD_KBST_CALLBACK(_write) \
	devcb = &mm1_keyboard_device::set_kbst_wr_callback(*device, DEVCB_##_write);



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> mm1_keyboard_device

class mm1_keyboard_device :  public device_t
{
public:
	// construction/destruction
	mm1_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_kbst_wr_callback(device_t &device, Object &&cb) { return downcast<mm1_keyboard_device &>(device).m_write_kbst.set_callback(std::forward<Object>(cb)); }

	DECLARE_READ8_MEMBER( read ) { return m_data; }

	DECLARE_WRITE_LINE_MEMBER( bell_w )
	{
		if (state == 1)
		{
			if (first_time)
			{
				m_samples->start(1, 1); // power switch
				first_time = false;
			}
			if (!m_samples->playing(0)) m_samples->start(0, 0); // beep; during boot, the second beep is in real HW very short (just before floppy seeks) but that's NYI
		}
		else if (m_samples->playing(0)) m_samples->stop(0); // happens only once during boot, no effect on output
	}
	void shut_down_mm1();

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual ioport_constructor device_input_ports() const override;

private:
	devcb_write_line m_write_kbst;

	required_device<samples_device> m_samples;
	required_memory_region m_rom;
	required_ioport_array<10> m_y;
	required_ioport m_special;

	int m_sense;
	int m_drive;
	uint8_t m_data;

	static bool first_time;                 // for power switch sound
	emu_timer *m_scan_timer;                // scan timer
};


// device type definition
DECLARE_DEVICE_TYPE(MM1_KEYBOARD, mm1_keyboard_device)



#endif // MAME_MACHINE_MM1KB_H
