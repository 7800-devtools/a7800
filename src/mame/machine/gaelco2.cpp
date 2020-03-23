// license:BSD-3-Clause
// copyright-holders:Manuel Abadia
/***************************************************************************

    Gaelco CG-1V/GAE1 based games

    Functions to emulate general aspects of the machine (RAM, ROM, interrupts,
    I/O ports)

***************************************************************************/

#include "emu.h"
#include "includes/gaelco2.h"

#include "machine/eepromser.h"
#include "chd.h"


/***************************************************************************

    Split even/odd bytes from ROMs in 16 bit mode to different memory areas

***************************************************************************/

void gaelco2_state::gaelco2_ROM16_split_gfx(const char *src_reg, const char *dst_reg, int start, int length, int dest1, int dest2)
{
	int i;

	/* get a pointer to the source data */
	uint8_t *src = (uint8_t *)memregion(src_reg)->base();

	/* get a pointer to the destination data */
	uint8_t *dst = (uint8_t *)memregion(dst_reg)->base();

	/* fill destination areas with the proper data */
	for (i = 0; i < length/2; i++){
		dst[dest1 + i] = src[start + i*2 + 0];
		dst[dest2 + i] = src[start + i*2 + 1];
	}
}


/***************************************************************************

    Driver init routines

***************************************************************************/

DRIVER_INIT_MEMBER(gaelco2_state,alighunt)
{
	/*
	For "gfx2" we have this memory map:
	    0x0000000-0x03fffff ROM u48
	    0x0400000-0x07fffff ROM u47
	    0x0800000-0x0bfffff ROM u50
	    0x0c00000-0x0ffffff ROM u49

	and we are going to construct this one for "gfx1":
	    0x0000000-0x01fffff ROM u48 even bytes
	    0x0200000-0x03fffff ROM u47 even bytes
	    0x0400000-0x05fffff ROM u48 odd bytes
	    0x0600000-0x07fffff ROM u47 odd bytes
	    0x0800000-0x09fffff ROM u50 even bytes
	    0x0a00000-0x0bfffff ROM u49 even bytes
	    0x0c00000-0x0dfffff ROM u50 odd bytes
	    0x0e00000-0x0ffffff ROM u49 odd bytes
	*/

	/* split ROM u48 */
	gaelco2_ROM16_split_gfx("gfx2", "gfx1", 0x0000000, 0x0400000, 0x0000000, 0x0400000);

	/* split ROM u47 */
	gaelco2_ROM16_split_gfx("gfx2", "gfx1", 0x0400000, 0x0400000, 0x0200000, 0x0600000);

	/* split ROM u50 */
	gaelco2_ROM16_split_gfx("gfx2", "gfx1", 0x0800000, 0x0400000, 0x0800000, 0x0c00000);

	/* split ROM u49 */
	gaelco2_ROM16_split_gfx("gfx2", "gfx1", 0x0c00000, 0x0400000, 0x0a00000, 0x0e00000);
}


DRIVER_INIT_MEMBER(gaelco2_state,touchgo)
{
	/*
	For "gfx2" we have this memory map:
	    0x0000000-0x03fffff ROM ic65
	    0x0400000-0x05fffff ROM ic66
	    0x0800000-0x0bfffff ROM ic67

	and we are going to construct this one for "gfx1":
	    0x0000000-0x01fffff ROM ic65 even bytes
	    0x0200000-0x02fffff ROM ic66 even bytes
	    0x0400000-0x05fffff ROM ic65 odd bytes
	    0x0600000-0x06fffff ROM ic66 odd bytes
	    0x0800000-0x09fffff ROM ic67 even bytes
	    0x0c00000-0x0dfffff ROM ic67 odd bytes
	*/

	/* split ROM ic65 */
	gaelco2_ROM16_split_gfx("gfx2", "gfx1", 0x0000000, 0x0400000, 0x0000000, 0x0400000);

	/* split ROM ic66 */
	gaelco2_ROM16_split_gfx("gfx2", "gfx1", 0x0400000, 0x0200000, 0x0200000, 0x0600000);

	/* split ROM ic67 */
	gaelco2_ROM16_split_gfx("gfx2", "gfx1", 0x0800000, 0x0400000, 0x0800000, 0x0c00000);
}


DRIVER_INIT_MEMBER(gaelco2_state,snowboar)
{
	/*
	For "gfx2" we have this memory map:
	    0x0000000-0x03fffff ROM sb44
	    0x0400000-0x07fffff ROM sb45
	    0x0800000-0x0bfffff ROM sb46

	and we are going to construct this one for "gfx1":
	    0x0000000-0x01fffff ROM sb44 even bytes
	    0x0200000-0x03fffff ROM sb45 even bytes
	    0x0400000-0x05fffff ROM sb44 odd bytes
	    0x0600000-0x07fffff ROM sb45 odd bytes
	    0x0800000-0x09fffff ROM sb46 even bytes
	    0x0c00000-0x0dfffff ROM sb46 odd bytes
	*/

	/* split ROM sb44 */
	gaelco2_ROM16_split_gfx("gfx2", "gfx1", 0x0000000, 0x0400000, 0x0000000, 0x0400000);

	/* split ROM sb45 */
	gaelco2_ROM16_split_gfx("gfx2", "gfx1", 0x0400000, 0x0400000, 0x0200000, 0x0600000);

	/* split ROM sb46 */
	gaelco2_ROM16_split_gfx("gfx2", "gfx1", 0x0800000, 0x0400000, 0x0800000, 0x0c00000);
}


/***************************************************************************

    MCU communication

***************************************************************************/

WRITE8_MEMBER(gaelco2_state::shareram_w)
{
	// why isn't there an AM_SOMETHING macro for this?
	reinterpret_cast<u8 *>(m_shareram.target())[BYTE_XOR_BE(offset)] = data;
}

READ8_MEMBER(gaelco2_state::shareram_r)
{
	// why isn't there an AM_SOMETHING macro for this?
	return reinterpret_cast<u8 const *>(m_shareram.target())[BYTE_XOR_BE(offset)];
}


/***************************************************************************

    Coin counters/lockouts

***************************************************************************/

WRITE16_MEMBER(gaelco2_state::gaelco2_coin_w)
{
	/* Coin Lockouts */
	machine().bookkeeping().coin_lockout_w(0, ~data & 0x01);
	machine().bookkeeping().coin_lockout_w(1, ~data & 0x02);

	/* Coin Counters */
	machine().bookkeeping().coin_counter_w(0, data & 0x04);
	machine().bookkeeping().coin_counter_w(1, data & 0x08);
}

WRITE16_MEMBER(gaelco2_state::gaelco2_coin2_w)
{
	/* coin counters */
	machine().bookkeeping().coin_counter_w(offset & 0x01,  data & 0x01);
}

WRITE16_MEMBER(wrally2_state::wrally2_coin_w)
{
	/* coin counters */
	machine().bookkeeping().coin_counter_w((offset >> 3) & 0x01,  data & 0x01);
}

WRITE16_MEMBER(gaelco2_state::touchgo_coin_w)
{
	if ((offset >> 2) == 0){
		machine().bookkeeping().coin_counter_w(0, data & 0x01);
		machine().bookkeeping().coin_counter_w(1, data & 0x02);
		machine().bookkeeping().coin_counter_w(2, data & 0x04);
		machine().bookkeeping().coin_counter_w(3, data & 0x08);
	}
}

/***************************************************************************

    Bang

***************************************************************************/


DRIVER_INIT_MEMBER(bang_state,bang)
{
	m_clr_gun_int = 0;
}

WRITE16_MEMBER(bang_state::bang_clr_gun_int_w)
{
	m_clr_gun_int = 1;
}

TIMER_DEVICE_CALLBACK_MEMBER(bang_state::bang_irq)
{
	int scanline = param;

	if (scanline == 256){
		m_maincpu->set_input_line(2, HOLD_LINE);
		m_clr_gun_int = 0;
	}

	if ((scanline % 64) == 0 && m_clr_gun_int)
		m_maincpu->set_input_line(4, HOLD_LINE);
}

/***************************************************************************

    World Rally 2 analog controls
    - added by Mirko Mattioli <els@fastwebnet.it>
    ---------------------------------------------------------------
    WR2 pcb has two ADC, one for each player. The ADCs have in common
    the clock signal line (adc_clk) and the chip enable signal line
    (adc_cs) and, of course,  two different data out signal lines.
    When "Pot Wheel" option is selected via dip-switch, then the gear
    is enabled (low/high shifter); the gear is disabled in joy mode by
    the CPU program code. No brakes are present in this game.
    Analog controls routines come from modified code wrote by Aaron
    Giles for gaelco3d driver.

***************************************************************************/


CUSTOM_INPUT_MEMBER(wrally2_state::wrally2_analog_bit_r)
{
	int which = (uintptr_t)param;
	return (m_analog_ports[which] >> 7) & 0x01;
}


WRITE16_MEMBER(wrally2_state::wrally2_adc_clk)
{
	/* a zero/one combo is written here to clock the next analog port bit */
	if (ACCESSING_BITS_0_7)
	{
		if (!(data & 0xff))
		{
			m_analog_ports[0] <<= 1;
			m_analog_ports[1] <<= 1;
		}
	}
	else
		logerror("%06X:analog_port_clock_w(%02X) = %08X & %08X\n", space.device().safe_pc(), offset, data, mem_mask);
}


WRITE16_MEMBER(wrally2_state::wrally2_adc_cs)
{
	/* a zero is written here to read the analog ports, and a one is written when finished */
	if (ACCESSING_BITS_0_7)
	{
		if (!(data & 0xff))
		{
			m_analog_ports[0] = m_analog0->read();
			m_analog_ports[1] = m_analog1->read();
		}
	}
	else
		logerror("%06X:analog_port_latch_w(%02X) = %08X & %08X\n", space.device().safe_pc(), offset, data, mem_mask);
}

/***************************************************************************

    EEPROM (93C66)

***************************************************************************/

WRITE16_MEMBER(gaelco2_state::gaelco2_eeprom_cs_w)
{
	/* bit 0 is CS (active low) */
	m_eeprom->cs_write((data & 0x01) ? ASSERT_LINE : CLEAR_LINE);
}

WRITE16_MEMBER(gaelco2_state::gaelco2_eeprom_sk_w)
{
	/* bit 0 is SK (active high) */
	m_eeprom->clk_write((data & 0x01) ? ASSERT_LINE : CLEAR_LINE);
}

WRITE16_MEMBER(gaelco2_state::gaelco2_eeprom_data_w)
{
	/* bit 0 is EEPROM data (DIN) */
	m_eeprom->di_write(data & 0x01);
}

/***************************************************************************

    Protection

***************************************************************************/

static uint32_t rol(uint32_t x, unsigned int c)
{
	return (x << c) | (x >> (32 - c));
}

static uint16_t get_lo(uint32_t x)
{
	return ((x & 0x00000010) <<  1) |
			((x & 0x00000800) <<  3) |
			((x & 0x40000000) >> 27) |
			((x & 0x00000005) <<  6) |
			((x & 0x00000008) <<  8) |
			rol(x & 0x00800040, 9)   |
			((x & 0x04000000) >> 16) |
			((x & 0x00008000) >> 14) |
			((x & 0x00002000) >> 11) |
			((x & 0x00020000) >> 10) |
			((x & 0x00100000) >>  8) |
			((x & 0x00044000) >>  5) |
			((x & 0x00000020) >>  1);
}

static uint16_t get_hi(uint32_t x)
{
	return ((x & 0x00001400) >>  0) |
			((x & 0x10000000) >> 26) |
			((x & 0x02000000) >> 24) |
			((x & 0x08000000) >> 21) |
			((x & 0x00000002) << 12) |
			((x & 0x01000000) >> 19) |
			((x & 0x20000000) >> 18) |
			((x & 0x80000000) >> 16) |
			((x & 0x00200000) >> 13) |
			((x & 0x00010000) >> 12) |
			((x & 0x00080000) >> 10) |
			((x & 0x00000200) >>  9) |
			((x & 0x00400000) >>  8) |
			((x & 0x00000080) >>  4) |
			((x & 0x00000100) >>  1);
}

static uint16_t get_out(uint16_t x)
{
	return ((x & 0xc840) <<  0) |
			((x & 0x0080) <<  2) |
			((x & 0x0004) <<  3) |
			((x & 0x0008) <<  5) |
			((x & 0x0010) <<  8) |
			((x & 0x0002) <<  9) |
			((x & 0x0001) << 13) |
			((x & 0x0200) >>  9) |
			((x & 0x1400) >>  8) |
			((x & 0x0100) >>  7) |
			((x & 0x2000) >>  6) |
			((x & 0x0020) >>  2);
}

uint16_t mangle(uint32_t x)
{
	uint16_t a = get_lo(x);
	uint16_t b = get_hi(x);
	return get_out(((a ^ 0x0010) - (b ^ 0x0024)) ^ 0x5496);
}

READ16_MEMBER(gaelco2_state::snowboar_protection_r)
{
	uint16_t ret  = mangle(snowboard_latch);
	ret = ((ret & 0xff00) >> 8) | ((ret & 0x00ff) << 8);
	return ret;

}

WRITE16_MEMBER(gaelco2_state::snowboar_protection_w)
{
	COMBINE_DATA(&m_snowboar_protection[offset]);

	snowboard_latch = (snowboard_latch << 16) | data;

	logerror("%06x: protection write %04x to %04x\n", space.device().safe_pc(), data, offset*2);

}
