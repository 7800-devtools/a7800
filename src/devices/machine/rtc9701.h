// license:BSD-3-Clause
// copyright-holders:Angelo Salese, David Haywood
/***************************************************************************

    rtc9701.h

    Serial rtc9701s.

***************************************************************************/

#ifndef MAME_MACHINE_RTC9701_H
#define MAME_MACHINE_RTC9701_H

#pragma once



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_RTC9701_ADD(tag) \
		MCFG_DEVICE_ADD((tag), RTC9701, XTAL_32_768kHz)

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************


// ======================> rtc9701_device

class rtc9701_device :  public device_t,
						public device_nvram_interface
{
public:
	// construction/destruction
	rtc9701_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);


	// I/O operations
	DECLARE_WRITE_LINE_MEMBER( write_bit );
	DECLARE_READ_LINE_MEMBER( read_bit );
	DECLARE_WRITE_LINE_MEMBER( set_cs_line );
	DECLARE_WRITE_LINE_MEMBER( set_clock_line );
	TIMER_CALLBACK_MEMBER(timer_callback);

protected:
	enum class state_t : u8
	{
		CMD_WAIT = 0,
		RTC_READ,
		RTC_WRITE,
		EEPROM_READ,
		EEPROM_WRITE,
		AFTER_WRITE_ENABLE

	};

	struct regs_t
	{
		uint8_t sec, min, hour, day, wday, month, year;
	};

	// device-level overrides
	virtual void device_validity_check(validity_checker &valid) const override;
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_nvram_interface overrides
	virtual void nvram_default() override;
	virtual void nvram_read(emu_file &file) override;
	virtual void nvram_write(emu_file &file) override;
	inline uint8_t rtc_read(uint8_t offset);
	inline void rtc_write(uint8_t offset,uint8_t data);

	int                     m_latch;
	int                     m_reset_line;
	int                     m_clock_line;


	state_t rtc_state;
	int cmd_stream_pos;
	int current_cmd;

	int rtc9701_address_pos;
	int rtc9701_current_address;

	uint16_t rtc9701_current_data;
	int rtc9701_data_pos;

	uint16_t rtc9701_data[0x100];

	regs_t m_rtc;

	emu_timer *m_timer;
};


// device type definition
DECLARE_DEVICE_TYPE(RTC9701, rtc9701_device)

#endif // MAME_MACHINE_RTC9701_H
