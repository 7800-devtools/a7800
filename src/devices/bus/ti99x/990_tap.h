// license:GPL-2.0+
// copyright-holders:Raphael Nabet
/*
    990_tap.h: include file for 990_tap.c
*/
#ifndef MAME_BUS_TI99X_990_TAP_H
#define MAME_BUS_TI99X_990_TAP_H

#pragma once

DECLARE_DEVICE_TYPE(TI990_TAPE_CTRL, tap_990_device)

class tap_990_device : public device_t
{
public:
	tap_990_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	template <class Object> static devcb_base &static_set_int_callback(device_t &device, Object &&cb)
	{
		return downcast<tap_990_device &>(device).m_int_line.set_callback(std::forward<Object>(cb));
	}

	DECLARE_READ16_MEMBER( read );
	DECLARE_WRITE16_MEMBER( write );

	void set_tape(int id, device_image_interface *img, bool bot, bool eot, bool wp)
	{
		m_tape[id].img = img;
		m_tape[id].bot = bot;
		m_tape[id].eot = eot;
		m_tape[id].wp = wp;
	}

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	static constexpr unsigned MAX_TAPE_UNIT = 4;

	struct tape_unit_t
	{
		device_image_interface *img;        // image descriptor
		bool bot;   // true if we are at the beginning of tape
		bool eot;   // true if we are at the end of tape
		bool wp;    // true if tape is write-protected
	};

	int     cur_tape_unit();
	void    update_interrupt();
	void    cmd_read_binary_forward();
	void    cmd_record_skip_forward();
	void    cmd_record_skip_reverse();
	void    cmd_rewind();
	void    cmd_rewind_and_offline();
	void    read_transport_status();
	void    execute_command();

	devcb_write_line m_int_line;

	uint16_t m_w[8];

	tape_unit_t m_tape[MAX_TAPE_UNIT];
};

#define MCFG_TI990_TAPE_INT_HANDLER( _intcallb )  \
	devcb = &tap_990_device::static_set_int_callback( *device, DEVCB_##_intcallb );

#endif // MAME_BUS_TI99X_990_TAP_H
