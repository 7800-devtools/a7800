// license:GPL-2.0+
// copyright-holders:Fabio Priuli, byuu
/***********************************************************************************************************

 Super NES/Famicom (HiROM) cartridge emulation (for SNES/SFC)

 ***********************************************************************************************************/


#include "emu.h"
#include "rom21.h"


//-------------------------------------------------
//  sns_rom_device - constructor
//-------------------------------------------------

DEFINE_DEVICE_TYPE(SNS_HIROM,      sns_rom21_device,      "sns_rom21",     "SNES Cart (HiROM)")
DEFINE_DEVICE_TYPE(SNS_HIROM_SRTC, sns_rom21_srtc_device, "sns_rom21_rtc", "SNES Cart (HiROM) + S-RTC")


sns_rom21_device::sns_rom21_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock), device_sns_cart_interface( mconfig, *this )
{
}

sns_rom21_device::sns_rom21_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: sns_rom21_device(mconfig, SNS_HIROM, tag, owner, clock)
{
}

sns_rom21_srtc_device::sns_rom21_srtc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: sns_rom21_device(mconfig, SNS_HIROM_SRTC, tag, owner, clock), m_mode(0), m_index(0)
{
}


void sns_rom21_device::device_start()
{
}

void sns_rom21_device::device_reset()
{
}

void sns_rom21_srtc_device::device_start()
{
	save_item(NAME(m_mode));
	save_item(NAME(m_index));
//  save_item(NAME(m_rtc_ram));
}

void sns_rom21_srtc_device::device_reset()
{
	m_mode = RTCM_Read;
	m_index = -1;
//  memset(m_rtc_ram, 0, sizeof(m_rtc_ram));

// at this stage, rtc_ram is not yet allocated. this will be fixed when converting RTC to be a separate device.
//  update_time();
}

/*-------------------------------------------------
 mapper specific handlers
 -------------------------------------------------*/

// low and hi reads are not the same! (different ROM banks are accessed)

READ8_MEMBER(sns_rom21_device::read_l)
{
	// here ROM banks from 128 to 255, mirrored twice
	int bank = (offset & 0x3fffff) / 0x8000;
	return m_rom[rom_bank_map[bank + 0x80] * 0x8000 + (offset & 0x7fff)];
}

READ8_MEMBER(sns_rom21_device::read_h)
{
	// here ROM banks from 0 to 127, mirrored twice
	int bank = (offset & 0x3fffff) / 0x8000;
	return m_rom[rom_bank_map[bank] * 0x8000 + (offset & 0x7fff)];
}


// Hi-ROM + S-RTC (used by Daikaijuu Monogatari II)
// same as above but additional read/write handling for the RTC
/***************************************************************************

 Based on C++ implementation by Byuu in BSNES.

 Byuu's code is released under GNU General Public License
 version 2 as published by the Free Software Foundation.

 ***************************************************************************/

static const uint8_t srtc_months[12] =
{
	31, 28, 31,
	30, 31, 30,
	31, 31, 30,
	31, 30, 31
};

void sns_rom21_srtc_device::update_time()
{
	system_time curtime, *systime = &curtime;
	machine().current_datetime(curtime);
	m_rtc_ram[0] = systime->local_time.second % 10;
	m_rtc_ram[1] = systime->local_time.second / 10;
	m_rtc_ram[2] = systime->local_time.minute % 10;
	m_rtc_ram[3] = systime->local_time.minute / 10;
	m_rtc_ram[4] = systime->local_time.hour % 10;
	m_rtc_ram[5] = systime->local_time.hour / 10;
	m_rtc_ram[6] = systime->local_time.mday % 10;
	m_rtc_ram[7] = systime->local_time.mday / 10;
	m_rtc_ram[8] = systime->local_time.month;
	m_rtc_ram[9] = (systime->local_time.year - 1000) % 10;
	m_rtc_ram[10] = ((systime->local_time.year - 1000) / 10) % 10;
	m_rtc_ram[11] = (systime->local_time.year - 1000) / 100;
	m_rtc_ram[12] = systime->local_time.weekday % 7;
}

// Returns day-of-week for specified date
// e.g. 0 = Sunday, 1 = Monday, ... 6 = Saturday
// Usage: weekday(2008, 1, 1) returns the weekday of January 1st, 2008
uint8_t sns_rom21_srtc_device::srtc_weekday( uint32_t year, uint32_t month, uint32_t day )
{
	uint32_t y = 1900, m = 1; // Epoch is 1900-01-01
	uint32_t sum = 0;         // Number of days passed since epoch

	year = std::max(1900U, year);
	month = std::max(1U, std::min(12U, month));
	day = std::max(1U, std::min(31U, day));

	while (y < year)
	{
		uint8_t leapyear = 0;
		if ((y % 4) == 0)
		{
			leapyear = 1;
			if ((y % 100) == 0 && (y % 400) != 0)
			{
				leapyear = 0;
			}
		}
		sum += leapyear ? 366 : 365;
		y++;
	}

	while (m < month)
	{
		uint32_t days = srtc_months[m - 1];
		if (days == 28)
		{
			uint8_t leapyear = 0;
			if ((y % 4) == 0)
			{
				leapyear = 1;
				if ((y % 100) == 0 && (y % 400) != 0)
				{
					leapyear = 0;
				}
			}
			days += leapyear ? 1 : 0;
		}
		sum += days;
		m++;
	}

	sum += day - 1;
	return (sum + 1) % 7; // 1900-01-01 was a Monday
}


// this gets called only for accesses at 0x2800,
// because for 0x2801 open bus gets returned...
READ8_MEMBER(sns_rom21_srtc_device::chip_read)
{
	if (m_mode != RTCM_Read)
		return 0x00;

	if (m_index < 0)
	{
		update_time();
		m_index++;
		return 0x0f;
	}
	else if (m_index > 12)
	{
		m_index = -1;
		return 0x0f;
	}
	else
		return m_rtc_ram[m_index++];
}

// this gets called only for accesses at 0x2801
WRITE8_MEMBER(sns_rom21_srtc_device::chip_write)
{
	data &= 0x0f;   // Only the low four bits are used

	if (data == 0x0d)
	{
		m_mode = RTCM_Read;
		m_index = -1;
		return;
	}

	if (data == 0x0e)
	{
		m_mode = RTCM_Command;
		return;
	}

	if (data == 0x0f)
		return; // Unknown behaviour

	if (m_mode == RTCM_Write)
	{
		if (m_index >= 0 && m_index < 12)
		{
			m_rtc_ram[m_index++] = data;

			if (m_index == 12)
			{
				// Day of week is automatically calculated and written
				uint32_t day   = m_rtc_ram[6] + m_rtc_ram[7] * 10;
				uint32_t month = m_rtc_ram[8];
				uint32_t year  = m_rtc_ram[9] + m_rtc_ram[10] * 10 + m_rtc_ram[11] * 100;
				year += 1000;

				m_rtc_ram[m_index++] = srtc_weekday(year, month, day);
			}
		}
	}
	else if (m_mode == RTCM_Command)
	{
		if (data == 0)
		{
			m_mode = RTCM_Write;
			m_index = 0;
		}
		else if (data == 4)
		{
			uint8_t i;
			m_mode = RTCM_Ready;
			m_index = -1;
			for(i = 0; i < 13; i++)
				m_rtc_ram[i] = 0;
		}
		else
		{
			// Unknown behaviour
			m_mode = RTCM_Ready;
		}
	}
}
