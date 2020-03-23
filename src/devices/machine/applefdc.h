// license:BSD-3-Clause
// copyright-holders:Nathan Woods, Raphael Nabet, R. Belmont
/*********************************************************************

    applefdc.h

    Implementation of various Apple Floppy Disk Controllers, including
    the classic Apple controller and the IWM (Integrated Woz Machine)
    chip

    Nate Woods
    Raphael Nabet
    R. Belmont

*********************************************************************/

#ifndef MAME_MACHINE_APPLEFDC_H
#define MAME_MACHINE_APPLEFDC_H

#pragma once



/***************************************************************************
    CONSTANTS
***************************************************************************/

#define APPLEFDC_PH0    0x01
#define APPLEFDC_PH1    0x02
#define APPLEFDC_PH2    0x04
#define APPLEFDC_PH3    0x08

DECLARE_DEVICE_TYPE(APPLEFDC, applefdc_device)
DECLARE_DEVICE_TYPE(IWM, iwm_device)



/***************************************************************************
    INTERFACE
***************************************************************************/

struct applefdc_interface
{
	void (*set_lines)(device_t *device, uint8_t lines);
	void (*set_enable_lines)(device_t *device, int enable_mask);

	uint8_t (*read_data)(device_t *device);
	void (*write_data)(device_t *device, uint8_t data);
	int (*read_status)(device_t *device);
};



/***************************************************************************
    BASE DEVICE
***************************************************************************/

class applefdc_base_device : public device_t
{
public:
	// configuration helpers
	static void static_set_config(device_t &device, const applefdc_interface *intrf) { downcast<applefdc_base_device &>(device).m_interface = intrf; }

	// read/write handlers
	virtual uint8_t read(uint8_t offset);
	virtual void write(uint8_t offset, uint8_t data);

	// read/write handlers overloads
	uint8_t read(offs_t offset)               { return read((uint8_t) offset); }
	void write(offs_t offset, uint8_t data)   { write((uint8_t) offset, data); }
	DECLARE_READ8_MEMBER( read )            { return read((uint8_t) offset); }
	DECLARE_WRITE8_MEMBER( write )          { write((uint8_t) offset, data); }

	// accessor
	uint8_t get_lines();

protected:
	enum applefdc_t
	{
		APPLEFDC_APPLE2,    /* classic Apple II disk controller (pre-IWM) */
		APPLEFDC_IWM,       /* Integrated Woz Machine */
		APPLEFDC_SWIM       /* Sander/Woz Integrated Machine */
	};

	// constructor
	applefdc_base_device(applefdc_t fdc_type, const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// other protecteds
	virtual void iwm_modereg_w(uint8_t data);

private:
	// data that is constant for the lifetime of the emulation
	emu_timer * m_motor_timer;
	applefdc_t  m_type;
	const applefdc_interface *m_interface;

	// data that changes at emulation time
	uint8_t       m_write_byte;
	uint8_t       m_lines;                    /* flags from IWM_MOTOR - IWM_Q7 */
	uint8_t       m_mode;                     /* 0-31; see above */
	uint8_t       m_handshake_hack;           /* not sure what this is for */

	// functions
	const applefdc_interface *get_interface();
	int iwm_enable2();
	uint8_t iwm_readenable2handshake();
	uint8_t statusreg_r();
	uint8_t read_reg(int lines);
	void write_reg(uint8_t data);
	void turn_motor_onoff(bool status);
	void iwm_access(int offset);
};



/***************************************************************************
    APPLE FDC - Used on Apple II
***************************************************************************/

class applefdc_device : public applefdc_base_device
{
public:
	applefdc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};



/***************************************************************************
    IWM - Used on early Macs
***************************************************************************/

class iwm_device : public applefdc_base_device
{
public:
	iwm_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};



/***************************************************************************
    DEVICE CONFIGURATION MACROS
***************************************************************************/

#define MCFG_APPLEFDC_CONFIG(_intrf) \
	applefdc_base_device::static_set_config(*device, &(_intrf));

#define MCFG_APPLEFDC_ADD(_tag, _intrf) \
	MCFG_DEVICE_ADD(_tag, APPLEFDC, 0) \
	MCFG_APPLEFDC_CONFIG(_intrf)

#define MCFG_APPLEFDC_MODIFY(_tag, _intrf) \
	MCFG_DEVICE_MODIFY(_tag)          \
	MCFG_APPLEFDC_CONFIG(_intrf)

#define MCFG_IWM_ADD(_tag, _intrf) \
	MCFG_DEVICE_ADD(_tag, IWM, 0) \
	MCFG_APPLEFDC_CONFIG(_intrf)

#define MCFG_IWM_MODIFY(_tag, _intrf) \
	MCFG_DEVICE_MODIFY(_tag)          \
	MCFG_APPLEFDC_CONFIG(_intrf)


#endif // MAME_MACHINE_APPLEFDC_H
