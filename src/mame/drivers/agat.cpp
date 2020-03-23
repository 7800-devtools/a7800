// license:BSD-3-Clause
// copyright-holders:R. Belmont, Sergey Svishchev
/***************************************************************************

    agat.c
    Skeleton driver for Agat series of Soviet Apple II non-clones

    These are similar to Apple II (same bus architecture, keyboard and
    floppy interface), but video controller is completely different.

    To do:
    - native keyboards (at least two variants)
    - 840K floppy controller (MFM encoding, but track layout is unique)
    - agat7: 64K and 128K onboard memory configurations
    - agat9
    - 3rd party slot devices

************************************************************************/

#include "emu.h"
#include "includes/apple2.h"
#include "video/agat7.h"

#include "machine/bankdev.h"
#include "imagedev/flopdrv.h"

#include "bus/a2bus/a2diskii.h"
#include "bus/a2bus/agat7langcard.h"
#include "bus/a2bus/agat7ram.h"

#include "screen.h"
#include "softlist.h"
#include "speaker.h"

#include "formats/ap2_dsk.h"


#define A7_CPU_TAG "maincpu"
#define A7_KBDC_TAG "ay3600"
#define A7_BUS_TAG "a2bus"
#define A7_SPEAKER_TAG "speaker"
#define A7_CASSETTE_TAG "tape"
#define A7_UPPERBANK_TAG "inhbank"
#define A7_VIDEO_TAG "a7video"

class agat7_state : public driver_device
{
public:
	agat7_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, A7_CPU_TAG)
		, m_ram(*this, RAM_TAG)
		, m_ay3600(*this, A7_KBDC_TAG)
		, m_video(*this, A7_VIDEO_TAG)
		, m_a2bus(*this, A7_BUS_TAG)
		, m_joy1x(*this, "joystick_1_x")
		, m_joy1y(*this, "joystick_1_y")
		, m_joy2x(*this, "joystick_2_x")
		, m_joy2y(*this, "joystick_2_y")
		, m_joybuttons(*this, "joystick_buttons")
		, m_kbspecial(*this, "keyb_special")
		, m_kbrepeat(*this, "keyb_repeat")
		, m_speaker(*this, A7_SPEAKER_TAG)
		, m_cassette(*this, A7_CASSETTE_TAG)
		, m_upperbank(*this, A7_UPPERBANK_TAG)
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<ram_device> m_ram;
	required_device<ay3600_device> m_ay3600;
	required_device<agat7video_device> m_video;
	required_device<a2bus_device> m_a2bus;
	required_ioport m_joy1x, m_joy1y, m_joy2x, m_joy2y, m_joybuttons;
	required_ioport m_kbspecial;
	required_ioport m_kbrepeat;
	required_device<speaker_sound_device> m_speaker;
	required_device<cassette_image_device> m_cassette;
	required_device<address_map_bank_device> m_upperbank;

	TIMER_DEVICE_CALLBACK_MEMBER(ay3600_repeat);
	TIMER_DEVICE_CALLBACK_MEMBER(agat_timer);
	TIMER_DEVICE_CALLBACK_MEMBER(agat_vblank);

	virtual void machine_start() override;
	virtual void machine_reset() override;

	DECLARE_READ8_MEMBER(c000_r);
	DECLARE_WRITE8_MEMBER(c000_w);
	DECLARE_READ8_MEMBER(c080_r);
	DECLARE_WRITE8_MEMBER(c080_w);
	DECLARE_READ8_MEMBER(c100_r);
	DECLARE_WRITE8_MEMBER(c100_w);
	DECLARE_READ8_MEMBER(c800_r);
	DECLARE_WRITE8_MEMBER(c800_w);
	DECLARE_READ8_MEMBER(inh_r);
	DECLARE_WRITE8_MEMBER(inh_w);
	DECLARE_WRITE_LINE_MEMBER(a2bus_irq_w);
	DECLARE_WRITE_LINE_MEMBER(a2bus_nmi_w);
	DECLARE_WRITE_LINE_MEMBER(a2bus_inh_w);
	DECLARE_READ_LINE_MEMBER(ay3600_shift_r);
	DECLARE_READ_LINE_MEMBER(ay3600_control_r);
	DECLARE_WRITE_LINE_MEMBER(ay3600_data_ready_w);
	DECLARE_WRITE_LINE_MEMBER(ay3600_ako_w);

	DECLARE_READ8_MEMBER(agat7_membank_r);
	DECLARE_WRITE8_MEMBER(agat7_membank_w);
	DECLARE_READ8_MEMBER(agat7_ram_r);
	DECLARE_WRITE8_MEMBER(agat7_ram_w);

private:
	int m_speaker_state;
	int m_cassette_state;

	double m_joystick_x1_time;
	double m_joystick_y1_time;
	double m_joystick_x2_time;
	double m_joystick_y2_time;

	uint16_t m_lastchar, m_strobe;
	uint8_t m_transchar;
	bool m_anykeydown;

	int m_inh_slot;
	int m_cnxx_slot;

	bool m_page2;
	bool m_an0, m_an1, m_an2, m_an3;

	uint8_t *m_ram_ptr;
	int m_ram_size;

	int m_inh_bank;

	double m_x_calibration, m_y_calibration;

	device_a2bus_card_interface *m_slotdevice[8];

	bool m_agat7_interrupts;
	int m_agat7_membank;
	int m_agat7_ram_slot;

	void do_io(address_space &space, int offset);
	uint8_t read_floatingbus();
};

/***************************************************************************
    PARAMETERS
***************************************************************************/

#define JOYSTICK_DELTA          80
#define JOYSTICK_SENSITIVITY    50
#define JOYSTICK_AUTOCENTER     80

WRITE_LINE_MEMBER(agat7_state::a2bus_irq_w)
{
	m_maincpu->set_input_line(M6502_IRQ_LINE, state);
}

WRITE_LINE_MEMBER(agat7_state::a2bus_nmi_w)
{
	m_maincpu->set_input_line(INPUT_LINE_NMI, state);
}

// This code makes a ton of assumptions because we can guarantee a pre-IIe machine!
WRITE_LINE_MEMBER(agat7_state::a2bus_inh_w)
{
	if (state == ASSERT_LINE)
	{
		// assume no cards are pulling /INH
		m_inh_slot = -1;
		m_agat7_ram_slot = -1;

		// scan the slots to figure out which card(s) are INHibiting stuff
		for (int i = 0; i <= 7; i++)
		{
			if (m_slotdevice[i])
			{
				// this driver only can inhibit from 0xd000-0xffff
				if ((m_slotdevice[i]->inh_start() == 0xd000) &&
					(m_slotdevice[i]->inh_end() == 0xffff))
				{
					if ((m_slotdevice[i]->inh_type() & INH_READ) == INH_READ)
					{
						if (m_inh_bank != 1)
						{
							m_upperbank->set_bank(1);
							m_inh_bank = 1;
						}
					}
					else
					{
						if (m_inh_bank != 0)
						{
							m_upperbank->set_bank(0);
							m_inh_bank = 0;
						}
					}

					m_inh_slot = i;
					// break;
				}

				if ((m_slotdevice[i]->inh_start() == 0x8000) &&
					(m_slotdevice[i]->inh_end() == 0xbfff))
				{
					if ((m_slotdevice[i]->inh_type() & INH_READ) == INH_READ)
					{
						m_agat7_ram_slot = i;
					}
				}
			}
		}

		// if no slots are inhibiting, make sure ROM is fully switched in
		if ((m_inh_slot == -1) && (m_inh_bank != 0))
		{
			m_upperbank->set_bank(0);
			m_inh_bank = 0;
		}
	}
}

/***************************************************************************
    START/RESET
***************************************************************************/

void agat7_state::machine_start()
{
	m_ram_ptr = m_ram->pointer();
	m_ram_size = m_ram->size();
	m_speaker_state = 0;
	m_speaker->level_w(m_speaker_state);
	m_cassette_state = 0;
	m_cassette->output(-1.0f);
	m_upperbank->set_bank(0);
	m_inh_bank = 0;

	// precalculate joystick time constants
	m_x_calibration = attotime::from_usec(12).as_double();
	m_y_calibration = attotime::from_usec(13).as_double();

	// cache slot devices
	for (int i = 0; i <= 7; i++)
	{
		m_slotdevice[i] = m_a2bus->get_a2bus_card(i);
	}

	// setup save states
	save_item(NAME(m_speaker_state));
	save_item(NAME(m_cassette_state));
	save_item(NAME(m_joystick_x1_time));
	save_item(NAME(m_joystick_y1_time));
	save_item(NAME(m_joystick_x2_time));
	save_item(NAME(m_joystick_y2_time));
	save_item(NAME(m_lastchar));
	save_item(NAME(m_strobe));
	save_item(NAME(m_transchar));
	save_item(NAME(m_inh_slot));
	save_item(NAME(m_inh_bank));
	save_item(NAME(m_cnxx_slot));
	save_item(NAME(m_page2));
	save_item(NAME(m_an0));
	save_item(NAME(m_an1));
	save_item(NAME(m_an2));
	save_item(NAME(m_an3));
	save_item(NAME(m_anykeydown));

	// setup video pointers
	m_video->m_ram_dev = machine().device<ram_device>(RAM_TAG);
	m_video->m_char_ptr = memregion("gfx1")->base();
	m_video->m_char_size = memregion("gfx1")->bytes();
}

void agat7_state::machine_reset()
{
	m_inh_slot = -1;
	m_cnxx_slot = -1;
	m_page2 = false;
	m_an0 = m_an1 = m_an2 = m_an3 = false;
	m_anykeydown = false;
	m_upperbank->set_bank(0);
	m_agat7_interrupts = false;
	m_agat7_membank = 0;
	m_agat7_ram_slot = -1;
}

/***************************************************************************
    VIDEO
***************************************************************************/

/***************************************************************************
    I/O
***************************************************************************/
// most softswitches don't care about read vs write, so handle them here
void agat7_state::do_io(address_space &space, int offset)
{
	if (machine().side_effect_disabled())
	{
		return;
	}

	switch (offset & 0xf0)
	{
		case 0x20:
			m_cassette_state ^= 1;
			m_cassette->output(m_cassette_state ? 1.0f : -1.0f);
			break;

		case 0x30:
			m_speaker_state ^= 1;
			m_speaker->level_w(m_speaker_state);
			break;

		// XXX agat7: 0x4N -- enable timer interrupts, 0x5N -- disable (or vice versa depending on hw rev)
		case 0x40:
			m_agat7_interrupts = true;
			break;

		case 0x50:
			m_agat7_interrupts = false;

		case 0x70:
			m_joystick_x1_time = machine().time().as_double() + m_x_calibration * m_joy1x->read();
			m_joystick_y1_time = machine().time().as_double() + m_y_calibration * m_joy1y->read();
			m_joystick_x2_time = machine().time().as_double() + m_x_calibration * m_joy2x->read();
			m_joystick_y2_time = machine().time().as_double() + m_y_calibration * m_joy2y->read();
			break;
	}
}

READ8_MEMBER(agat7_state::c000_r)
{
	if (offset)
	logerror("%s: c000_r %04X == %02X\n", machine().describe_context(), offset+0xc000, 0);
	else if (m_strobe)
	logerror("%s: c000_r %04X == %02X\n", machine().describe_context(), offset+0xc000, m_transchar | m_strobe);

	switch (offset)
	{
		// keyboard latch.  NB: agat7 returns 0 if latch is clear, agat9 returns char code.
		case 0x00: case 0x01: case 0x02: case 0x03: case 0x04: case 0x05: case 0x06: case 0x07:
		case 0x08: case 0x09: case 0x0a: case 0x0b: case 0x0c: case 0x0d: case 0x0e: case 0x0f:
			return m_strobe ? (m_transchar | m_strobe) : 0;

		case 0x10:  // reads any key down, clears strobe
			{
				uint8_t rv = m_transchar | (m_anykeydown ? 0x80 : 0x00);
				m_strobe = 0;
				return rv;
			}

		case 0x60: // cassette in
		case 0x68:
			return m_cassette->input() > 0.0 ? 0x80 : 0;

		case 0x61:  // button 0
		case 0x69:
			return (m_joybuttons->read() & 0x10) ? 0x80 : 0;

		case 0x62:  // button 1
		case 0x6a:
			return (m_joybuttons->read() & 0x20) ? 0x80 : 0;

		case 0x63:  // button 2
		case 0x6b:
			return ((m_joybuttons->read() & 0x40) || (m_kbspecial->read() & 0x06)) ? 0 : 0x80;

		case 0x64:  // joy 1 X axis
		case 0x6c:
			return (space.machine().time().as_double() < m_joystick_x1_time) ? 0x80 : 0;

		case 0x65:  // joy 1 Y axis
		case 0x6d:
			return (space.machine().time().as_double() < m_joystick_y1_time) ? 0x80 : 0;

		case 0x66: // joy 2 X axis
		case 0x6e:
			return (space.machine().time().as_double() < m_joystick_x2_time) ? 0x80 : 0;

		case 0x67: // joy 2 Y axis
		case 0x6f:
			return (space.machine().time().as_double() < m_joystick_y2_time) ? 0x80 : 0;

		default:
			do_io(space, offset);
			break;
	}

	return read_floatingbus();
}

WRITE8_MEMBER(agat7_state::c000_w)
{
	logerror("%s: c000_w %04X <- %02X\n", machine().describe_context(), offset + 0xc000, data);

	switch (offset)
	{
		// clear keyboard latch
		case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17:
		case 0x18: case 0x19: case 0x1a: case 0x1b: case 0x1c: case 0x1d: case 0x1e: case 0x1f:
			m_strobe = 0;
			break;

		default:
			do_io(space, offset);
			break;
	}
}

READ8_MEMBER(agat7_state::c080_r)
{
	if (!machine().side_effect_disabled())
	{
		int slot;

		offset &= 0x7F;
		slot = offset / 0x10;

		logerror("%s: c080_r %04X (slot %d) == %02X\n", machine().describe_context(), offset + 0xc080, slot, 0);

		if (m_slotdevice[slot] != nullptr)
		{
			return m_slotdevice[slot]->read_c0nx(space, offset % 0x10);
		}
	}

	return read_floatingbus();
}

WRITE8_MEMBER(agat7_state::c080_w)
{
	int slot;

	offset &= 0x7F;
	slot = offset / 0x10;

	logerror("%s: c080_w %04X (slot %d) <- %02X\n", machine().describe_context(), offset + 0xc080, slot, data);

	if (m_slotdevice[slot] != nullptr)
	{
		m_slotdevice[slot]->write_c0nx(space, offset % 0x10, data);
	}
}

READ8_MEMBER(agat7_state::c100_r)
{
	int slotnum;
	uint8_t data = 0;

	slotnum = ((offset >> 8) & 0xf) + 1;

	if (m_slotdevice[slotnum] != nullptr)
	{
		if ((m_slotdevice[slotnum]->take_c800()) && (!machine().side_effect_disabled()))
		{
			m_cnxx_slot = slotnum;
		}

		logerror("%s: c100_r %04X (slot %d) == %02X\n", machine().describe_context(), offset+0xc100, slotnum, data);

		return m_slotdevice[slotnum]->read_cnxx(space, offset & 0xff);
	}

	return read_floatingbus();
}

WRITE8_MEMBER(agat7_state::c100_w)
{
	int slotnum;

	slotnum = ((offset >> 8) & 0xf) + 1;

	logerror("%s: c100_w %04X (slot %d) <- %02X\n", machine().describe_context(), offset + 0xc100, slotnum, data);

	if (m_slotdevice[slotnum] != nullptr)
	{
		if ((m_slotdevice[slotnum]->take_c800()) && (!machine().side_effect_disabled()))
		{
			m_cnxx_slot = slotnum;
		}

		m_slotdevice[slotnum]->write_cnxx(space, offset&0xff, data);
	}
}

READ8_MEMBER(agat7_state::c800_r)
{
//  logerror("%s: c800_r %04X (slot %d) == %02X\n", machine().describe_context(), offset+0xc800, m_cnxx_slot, 0);

	if (offset == 0x7ff)
	{
		if (!machine().side_effect_disabled())
		{
			m_cnxx_slot = -1;
		}

		return 0xff;
	}

	if ((m_cnxx_slot != -1) && (m_slotdevice[m_cnxx_slot] != nullptr))
	{
		return m_slotdevice[m_cnxx_slot]->read_c800(space, offset & 0xfff);
	}

	return read_floatingbus();
}

WRITE8_MEMBER(agat7_state::c800_w)
{
//  logerror("%s: c800_w %04X <- %02X\n", machine().describe_context(), offset+0xc800, data);

	if (offset == 0x7ff)
	{
		if (!machine().side_effect_disabled())
		{
			m_cnxx_slot = -1;
		}

		return;
	}

	if ((m_cnxx_slot != -1) && (m_slotdevice[m_cnxx_slot] != nullptr))
	{
		m_slotdevice[m_cnxx_slot]->write_c800(space, offset & 0xfff, data);
	}
}

READ8_MEMBER(agat7_state::inh_r)
{
	if (m_inh_slot != -1)
	{
		return m_slotdevice[m_inh_slot]->read_inh_rom(space, offset + 0xd000);
	}

	assert(0); // hitting inh_r with invalid m_inh_slot should not be possible
	return read_floatingbus();
}

WRITE8_MEMBER(agat7_state::inh_w)
{
	if (m_inh_slot != -1)
	{
		m_slotdevice[m_inh_slot]->write_inh_rom(space, offset + 0xd000, data);
	}
}

// XXX what does Agat do here?
uint8_t agat7_state::read_floatingbus()
{
	return 0xff;
}

/***************************************************************************
    ADDRESS MAP
***************************************************************************/

/* onboard memory banking on Agat-7 */

READ8_MEMBER(agat7_state::agat7_membank_r)
{
	logerror("%s: c0f0_r %04X == %02X\n", machine().describe_context(), offset + 0xc0f0, 0xff);

	m_agat7_membank = offset;

	return 0xff;
}

WRITE8_MEMBER(agat7_state::agat7_membank_w)
{
	logerror("%s: c0f0_w %04X <- %02X\n", machine().describe_context(), offset + 0xc0f0, data);

	m_agat7_membank = offset;
}

READ8_MEMBER(agat7_state::agat7_ram_r)
{
	if (offset < 32768)
	{
		if (offset < m_ram_size)
		{
			return m_ram_ptr[offset];
		}
	}
	else
	{
		if (m_agat7_ram_slot != -1)
			return m_slotdevice[m_agat7_ram_slot]->read_inh_rom(space, offset + 0x8000);
		if (offset < m_ram_size)
		{
			if (m_agat7_membank == 0)
				return m_ram_ptr[offset];
			else if (m_agat7_membank == 1)
				return m_ram_ptr[offset + 16384];
		}
	}

	return 0xff;
}

WRITE8_MEMBER(agat7_state::agat7_ram_w)
{
//  if (offset > 0x7fff)
//  logerror("%s: ram %04X (bank %d slot %d) <- %02X\n", machine().describe_context(), offset, m_agat7_membank, m_agat7_ram_slot, data);

	if (offset < 32768)
	{
		if (offset < m_ram_size)
		{
			m_ram_ptr[offset] = data;
		}
	}
	else
	{
		if (m_agat7_ram_slot != -1)
			m_slotdevice[m_agat7_ram_slot]->write_inh_rom(space, offset + 0x8000, data);
		else if (offset < m_ram_size)
		{
			if (m_agat7_membank == 0)
				m_ram_ptr[offset] = data;
			else if (m_agat7_membank == 1)
				m_ram_ptr[offset + 16384] = data;
		}
	}
}

/*
 * onboard RAM is at least 32K, up to 128K.
 * first 32K of onboard RAM are always mapped at 0.
 * standard add-on RAM cards hold 32K (possibly up to 128K?)
 * and are supported only on motherboards with 32K onboard.
 * all extra RAM (onboard or addon) is accessible via 16K window at 0x8000.
 */
static ADDRESS_MAP_START( agat7_map, AS_PROGRAM, 8, agat7_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0xbfff) AM_READWRITE(agat7_ram_r, agat7_ram_w)
	AM_RANGE(0xc000, 0xc07f) AM_READWRITE(c000_r, c000_w)
	AM_RANGE(0xc080, 0xc0ef) AM_READWRITE(c080_r, c080_w)
	AM_RANGE(0xc0f0, 0xc0ff) AM_READWRITE(agat7_membank_r, agat7_membank_w)
	AM_RANGE(0xc100, 0xc6ff) AM_READWRITE(c100_r, c100_w)
	AM_RANGE(0xc700, 0xc7ff) AM_DEVREADWRITE("a7video", agat7video_device, read, write)
	AM_RANGE(0xc800, 0xcfff) AM_READWRITE(c800_r, c800_w)
	AM_RANGE(0xd000, 0xffff) AM_DEVICE(A7_UPPERBANK_TAG, address_map_bank_device, amap8)
ADDRESS_MAP_END

static ADDRESS_MAP_START( inhbank_map, AS_PROGRAM, 8, agat7_state )
	AM_RANGE(0x0000, 0x2fff) AM_ROM AM_REGION("maincpu", 0x1000) AM_WRITE(inh_w)
	AM_RANGE(0x3000, 0x5fff) AM_READWRITE(inh_r, inh_w)
ADDRESS_MAP_END

/***************************************************************************
    KEYBOARD
***************************************************************************/

READ_LINE_MEMBER(agat7_state::ay3600_shift_r)
{
	// either shift key
	if (m_kbspecial->read() & 0x06)
	{
		return ASSERT_LINE;
	}

	return CLEAR_LINE;
}

READ_LINE_MEMBER(agat7_state::ay3600_control_r)
{
	if (m_kbspecial->read() & 0x08)
	{
		return ASSERT_LINE;
	}

	return CLEAR_LINE;
}

static const uint8_t a2_key_remap[0x40][4] =
{
/*    norm shft ctrl both */
	{ 0x33,0x23,0x33,0x23 },    /* 3 #     00     */
	{ 0x34,0x24,0x34,0x24 },    /* 4 $     01     */
	{ 0x35,0x25,0x35,0x25 },    /* 5 %     02     */
	{ 0x36,'&', 0x35,'&'  },    /* 6 &     03     */
	{ 0x37,0x27,0x37,0x27 },    /* 7 '     04     */
	{ 0x38,0x28,0x38,0x28 },    /* 8 (     05     */
	{ 0x39,0x29,0x39,0x29 },    /* 9 )     06     */
	{ 0x30,0x30,0x30,0x30 },    /* 0       07     */
	{ 0x3a,0x2a,0x3b,0x2a },    /* : *     08     */
	{ 0x2d,0x3d,0x2d,0x3d },    /* - =     09     */
	{ 0x51,0x51,0x11,0x11 },    /* q Q     0a     */
	{ 0x57,0x57,0x17,0x17 },    /* w W     0b     */
	{ 0x45,0x45,0x05,0x05 },    /* e E     0c     */
	{ 0x52,0x52,0x12,0x12 },    /* r R     0d     */
	{ 0x54,0x54,0x14,0x14 },    /* t T     0e     */
	{ 0x59,0x59,0x19,0x19 },    /* y Y     0f     */
	{ 0x55,0x55,0x15,0x15 },    /* u U     10     */
	{ 0x49,0x49,0x09,0x09 },    /* i I     11     */
	{ 0x4f,0x4f,0x0f,0x0f },    /* o O     12     */
	{ 0x50,0x50,0x10,0x10 },    /* p P     13     */
	{ 0x44,0x44,0x04,0x04 },    /* d D     14     */
	{ 0x46,0x46,0x06,0x06 },    /* f F     15     */
	{ 0x47,0x47,0x07,0x07 },    /* g G     16     */
	{ 0x48,0x48,0x08,0x08 },    /* h H     17     */
	{ 0x4a,0x4a,0x0a,0x0a },    /* j J     18     */
	{ 0x4b,0x4b,0x0b,0x0b },    /* k K     19     */
	{ 0x4c,0x4c,0x0c,0x0c },    /* l L     1a     */
	{ ';' ,0x2b,';' ,0x2b },    /* ; +     1b     */
	{ 0x08,0x08,0x08,0x08 },    /* Left    1c     */
	{ 0x15,0x15,0x15,0x15 },    /* Right   1d     */
	{ 0x5a,0x5a,0x1a,0x1a },    /* z Z     1e     */
	{ 0x58,0x58,0x18,0x18 },    /* x X     1f     */
	{ 0x43,0x43,0x03,0x03 },    /* c C     20     */
	{ 0x56,0x56,0x16,0x16 },    /* v V     21     */
	{ 0x42,0x42,0x02,0x02 },    /* b B     22     */
	{ 0x4e,0x4e,0x0e,0x0e },    /* n N     23     */
	{ 0x4d,0x4d,0x0d,0x0d },    /* m M     24     */
	{ 0x2c,0x3c,0x2c,0x3c },    /* , <     25     */
	{ 0x2e,0x3e,0x2e,0x3e },    /* . >     26     */
	{ 0x2f,0x3f,0x2f,0x3f },    /* / ?     27     */
	{ 0x53,0x53,0x13,0x13 },    /* s S     28     */
	{ 0x32,0x22,0x32,0x00 },    /* 2 "     29     */
	{ 0x31,0x21,0x31,0x31 },    /* 1 !     2a     */
	{ 0x1b,0x1b,0x1b,0x1b },    /* Escape  2b     */
	{ 0x41,0x41,0x01,0x01 },    /* a A     2c     */
	{ 0x20,0x20,0x20,0x20 },    /* Space   2d     */
	{ 0x19,0x19,0x19,0x19 },    /* Up */
	{ 0x1a,0x1a,0x1a,0x1a },    /* Down */
	{ 0x10,0x10,0x10,0x10 },    /* KP1 */
	{ 0x0d,0x0d,0x0d,0x0d },    /* Enter   31     */
	{ 0x11,0x11,0x11,0x11 },
	{ 0x12,0x12,0x12,0x12 },
	{ 0x13,0x13,0x13,0x13 },
	{ 0x14,0x14,0x14,0x14 },
	{ 0x1c,0x1c,0x1c,0x1c },
	{ 0x1d,0x1d,0x1d,0x1d },
	{ 0x1e,0x1e,0x1e,0x1e },
	{ 0x1f,0x1f,0x1f,0x1f },
	{ 0x01,0x01,0x01,0x01 },
	{ 0x02,0x02,0x02,0x02 },
	{ 0x03,0x03,0x03,0x03 },
	{ 0x04,0x04,0x04,0x04 },
	{ 0x05,0x05,0x05,0x05 },
	{ 0x06,0x06,0x06,0x06 },
};

WRITE_LINE_MEMBER(agat7_state::ay3600_data_ready_w)
{
	if (state == ASSERT_LINE)
	{
		int mod = 0;
		m_lastchar = m_ay3600->b_r();

		mod = (m_kbspecial->read() & 0x06) ? 0x01 : 0x00;
		mod |= (m_kbspecial->read() & 0x08) ? 0x02 : 0x00;

		m_transchar = a2_key_remap[m_lastchar & 0x3f][mod];

		if (m_transchar != 0)
		{
			m_strobe = 0x80;
		}
	}
}

WRITE_LINE_MEMBER(agat7_state::ay3600_ako_w)
{
	m_anykeydown = (state == ASSERT_LINE) ? true : false;
}

TIMER_DEVICE_CALLBACK_MEMBER(agat7_state::ay3600_repeat)
{
	// is the key still down?
	if (m_anykeydown)
	{
		if (m_kbrepeat->read() & 1)
		{
			m_strobe = 0x80;
		}
	}
}

TIMER_DEVICE_CALLBACK_MEMBER(agat7_state::agat_timer)
{
	if (m_agat7_interrupts)
	{
		m_maincpu->set_input_line(M6502_IRQ_LINE, ASSERT_LINE);
	}
}

TIMER_DEVICE_CALLBACK_MEMBER(agat7_state::agat_vblank)
{
	if (m_agat7_interrupts)
	{
		m_maincpu->set_input_line(M6502_NMI_LINE, ASSERT_LINE);
	}
}

/***************************************************************************
    INPUT PORTS
***************************************************************************/

static INPUT_PORTS_START( agat7_joystick )
	PORT_START("joystick_1_x")      /* Joystick 1 X Axis */
	PORT_BIT( 0xff, 0x80, IPT_AD_STICK_X) PORT_NAME("P1 Joystick X")
	PORT_SENSITIVITY(JOYSTICK_SENSITIVITY)
	PORT_KEYDELTA(JOYSTICK_DELTA)
	PORT_CENTERDELTA(JOYSTICK_AUTOCENTER)
	PORT_MINMAX(0,0xff) PORT_PLAYER(1)
	PORT_CODE_DEC(KEYCODE_4_PAD)    PORT_CODE_INC(KEYCODE_6_PAD)
	PORT_CODE_DEC(JOYCODE_X_LEFT_SWITCH)    PORT_CODE_INC(JOYCODE_X_RIGHT_SWITCH)

	PORT_START("joystick_1_y")      /* Joystick 1 Y Axis */
	PORT_BIT( 0xff, 0x80, IPT_AD_STICK_Y) PORT_NAME("P1 Joystick Y")
	PORT_SENSITIVITY(JOYSTICK_SENSITIVITY)
	PORT_KEYDELTA(JOYSTICK_DELTA)
	PORT_CENTERDELTA(JOYSTICK_AUTOCENTER)
	PORT_MINMAX(0,0xff) PORT_PLAYER(1)
	PORT_CODE_DEC(KEYCODE_8_PAD)    PORT_CODE_INC(KEYCODE_2_PAD)
	PORT_CODE_DEC(JOYCODE_Y_UP_SWITCH)      PORT_CODE_INC(JOYCODE_Y_DOWN_SWITCH)

	PORT_START("joystick_2_x")      /* Joystick 2 X Axis */
	PORT_BIT( 0xff, 0x80, IPT_AD_STICK_X) PORT_NAME("P2 Joystick X")
	PORT_SENSITIVITY(JOYSTICK_SENSITIVITY)
	PORT_KEYDELTA(JOYSTICK_DELTA)
	PORT_CENTERDELTA(JOYSTICK_AUTOCENTER)
	PORT_MINMAX(0,0xff) PORT_PLAYER(2)
	PORT_CODE_DEC(JOYCODE_X_LEFT_SWITCH)    PORT_CODE_INC(JOYCODE_X_RIGHT_SWITCH)

	PORT_START("joystick_2_y")      /* Joystick 2 Y Axis */
	PORT_BIT( 0xff, 0x80, IPT_AD_STICK_Y) PORT_NAME("P2 Joystick Y")
	PORT_SENSITIVITY(JOYSTICK_SENSITIVITY)
	PORT_KEYDELTA(JOYSTICK_DELTA)
	PORT_CENTERDELTA(JOYSTICK_AUTOCENTER)
	PORT_MINMAX(0,0xff) PORT_PLAYER(2)
	PORT_CODE_DEC(JOYCODE_Y_UP_SWITCH)      PORT_CODE_INC(JOYCODE_Y_DOWN_SWITCH)

	PORT_START("joystick_buttons")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_BUTTON1)  PORT_PLAYER(1)            PORT_CODE(KEYCODE_0_PAD)    PORT_CODE(JOYCODE_BUTTON1)
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_BUTTON2)  PORT_PLAYER(1)            PORT_CODE(KEYCODE_ENTER_PAD)PORT_CODE(JOYCODE_BUTTON2)
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_BUTTON1)  PORT_PLAYER(2)            PORT_CODE(JOYCODE_BUTTON1)
INPUT_PORTS_END

	/*
	  Apple II / II Plus key matrix (from "The Apple II Circuit Description")

	      | Y0  | Y1  | Y2  | Y3  | Y4  | Y5  | Y6  | Y7  | Y8  | Y9  |
	      |     |     |     |     |     |     |     |     |     |     |
	  ----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----|
	  X0  |  3  |  4  |  5  |  6  |  7  |  8  |  9  |  0  | :*  |  -  |
	  ----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----|
	  X1  |  Q  |  W  |  E  |  R  |  T  |  Y  |  U  |  I  |  O  |  P  |
	  ----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----|
	  X2  |  D  |  F  |  G  |  H  |  J  |  K  |  L  | ;+  |LEFT |RIGHT|
	  ----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----|
	  X3  |  Z  |  X  |  C  |  V  |  B  |  N  |  M  | ,<  | .>  |  /? |
	  ----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----|
	  X4  |  S  |  2  |  1  | ESC |  A  |SPACE|     |     |     |ENTER|
	  ----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----|
	*/

static INPUT_PORTS_START( agat7_common )
	PORT_START("X0")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_3)  PORT_CHAR('3') PORT_CHAR('#')
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_4)  PORT_CHAR('4') PORT_CHAR('$')
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_5)  PORT_CHAR('5') PORT_CHAR('%')
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_6)  PORT_CHAR('6') PORT_CHAR('&')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_7)  PORT_CHAR('7') PORT_CHAR('\'')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_8)  PORT_CHAR('8') PORT_CHAR('(')
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_9)  PORT_CHAR('9') PORT_CHAR(')')
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_0)  PORT_CHAR('0')
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS)  PORT_CHAR(':') PORT_CHAR('*')
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_EQUALS) PORT_CHAR('-') PORT_CHAR('=')

	PORT_START("X1")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Q)  PORT_CHAR('Q') PORT_CHAR('q')
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_W)  PORT_CHAR('W') PORT_CHAR('w')
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_E)  PORT_CHAR('E') PORT_CHAR('e')
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_R)  PORT_CHAR('R') PORT_CHAR('r')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_T)  PORT_CHAR('T') PORT_CHAR('t')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Y)  PORT_CHAR('Y') PORT_CHAR('y')
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_U)  PORT_CHAR('U') PORT_CHAR('u')
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_I)  PORT_CHAR('I') PORT_CHAR('i')
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_O)  PORT_CHAR('O') PORT_CHAR('o')
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_P)  PORT_CHAR('P') PORT_CHAR('@')

	PORT_START("X2")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_D)  PORT_CHAR('D') PORT_CHAR('d')
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F)  PORT_CHAR('F') PORT_CHAR('f')
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_G)  PORT_CHAR('G') PORT_CHAR('g')
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_H)  PORT_CHAR('H') PORT_CHAR('h')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_J)  PORT_CHAR('J') PORT_CHAR('j')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_K)  PORT_CHAR('K') PORT_CHAR('k')
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_L)  PORT_CHAR('L') PORT_CHAR('l')
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COLON) PORT_CHAR(';') PORT_CHAR('+')
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(UTF8_LEFT)      PORT_CODE(KEYCODE_LEFT)
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(UTF8_RIGHT)     PORT_CODE(KEYCODE_RIGHT)

	PORT_START("X3")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Z)  PORT_CHAR('Z') PORT_CHAR('z')
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_X)  PORT_CHAR('X') PORT_CHAR('x')
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_C)  PORT_CHAR('C') PORT_CHAR('c')
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_V)  PORT_CHAR('V') PORT_CHAR('v')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_B)  PORT_CHAR('B') PORT_CHAR('b')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_N)  PORT_CHAR('N') PORT_CHAR('n')
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_M)  PORT_CHAR('M') PORT_CHAR('m')
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA)  PORT_CHAR(',') PORT_CHAR('<')
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_STOP)   PORT_CHAR('.') PORT_CHAR('>')
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH)  PORT_CHAR('/') PORT_CHAR('?')

	PORT_START("X4")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_S)  PORT_CHAR('S') PORT_CHAR('s')
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_2)  PORT_CHAR('2') PORT_CHAR('\"')
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_1)  PORT_CHAR('1') PORT_CHAR('!')
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Esc")      PORT_CODE(KEYCODE_ESC)      PORT_CHAR(27)
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_A)          PORT_CHAR('A') PORT_CHAR('a')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SPACE)  PORT_CHAR(' ')
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(UTF8_UP)    PORT_CODE(KEYCODE_UP)
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(UTF8_DOWN)  PORT_CODE(KEYCODE_DOWN)
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Num 1") PORT_CODE(KEYCODE_7_PAD) PORT_CHAR(UCHAR_MAMEKEY(7_PAD))
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Return")   PORT_CODE(KEYCODE_ENTER)    PORT_CHAR(13)

	PORT_START("X5")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 2") PORT_CODE(KEYCODE_8_PAD) PORT_CHAR(UCHAR_MAMEKEY(8_PAD))
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 3") PORT_CODE(KEYCODE_9_PAD) PORT_CHAR(UCHAR_MAMEKEY(9_PAD))
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 4") PORT_CODE(KEYCODE_4_PAD) PORT_CHAR(UCHAR_MAMEKEY(4_PAD))
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 5") PORT_CODE(KEYCODE_5_PAD) PORT_CHAR(UCHAR_MAMEKEY(5_PAD))
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 6") PORT_CODE(KEYCODE_6_PAD) PORT_CHAR(UCHAR_MAMEKEY(6_PAD))
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 7") PORT_CODE(KEYCODE_1_PAD) PORT_CHAR(UCHAR_MAMEKEY(1_PAD))
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 8") PORT_CODE(KEYCODE_2_PAD) PORT_CHAR(UCHAR_MAMEKEY(2_PAD))
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 9") PORT_CODE(KEYCODE_3_PAD) PORT_CHAR(UCHAR_MAMEKEY(3_PAD))
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num 0") PORT_CODE(KEYCODE_0_PAD) PORT_CHAR(UCHAR_MAMEKEY(0_PAD))
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num .") PORT_CODE(KEYCODE_DEL_PAD) PORT_CHAR(UCHAR_MAMEKEY(DEL_PAD))

	PORT_START("X6")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Num =") PORT_CODE(KEYCODE_ENTER_PAD) PORT_CHAR(UCHAR_MAMEKEY(ENTER_PAD))
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("PF1") PORT_CODE(KEYCODE_SLASH_PAD) PORT_CHAR(UCHAR_MAMEKEY(SLASH_PAD))
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("PF2") PORT_CODE(KEYCODE_ASTERISK)
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("PF3") PORT_CODE(KEYCODE_MINUS_PAD)
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("X7")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("X8")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("keyb_special")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Left Shift")   PORT_CODE(KEYCODE_LSHIFT)   PORT_CHAR(UCHAR_SHIFT_1)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Right Shift")  PORT_CODE(KEYCODE_RSHIFT)   PORT_CHAR(UCHAR_SHIFT_1)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Control")      PORT_CODE(KEYCODE_LCONTROL) PORT_CHAR(UCHAR_SHIFT_2)
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("RESET")        PORT_CODE(KEYCODE_F12)
INPUT_PORTS_END

static INPUT_PORTS_START(agat7)
	PORT_INCLUDE(agat7_common)

	PORT_START("keyb_repeat")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("REPT")         PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR('\\')

	/* other devices */
	PORT_INCLUDE(agat7_joystick)
INPUT_PORTS_END

static SLOT_INTERFACE_START(agat7_cards)
	// Standard cards

	SLOT_INTERFACE("a7lang", A2BUS_AGAT7LANGCARD) // Agat-7 RAM Language Card -- decimal 3.089.119
	SLOT_INTERFACE("a7ram", A2BUS_AGAT7RAM) // Agat-7 32K RAM Card -- decimal 3.089.119-01, KR565RU6D chips
	SLOT_INTERFACE("a7fdc", A2BUS_AGAT7_FDC) // Disk II clone -- decimal 3.089.105
	// 840K floppy controller -- decimal 7.104.351
	// Serial-parallel card -- decimal 3.089.106
	// Printer card (agat9) -- decimal 3.089.174

	// 3rd party cards

	// Programmable i/o (8035 + 8251 + 8253 + 8255)
	// Card-93 floppy controller (KR1818VG93-based)
	// Nippel ADC (digital oscilloscope)
	// Nippel Clock (mc146818)
	// Nippel Co-processor (R65C02 clone + dual-ported RAM)
SLOT_INTERFACE_END

static MACHINE_CONFIG_START( agat7 )
	MCFG_CPU_ADD("maincpu", M6502, XTAL_14_3MHz / 14)
	MCFG_CPU_PROGRAM_MAP(agat7_map)

	MCFG_TIMER_DRIVER_ADD_PERIODIC("agat7irq", agat7_state, agat_timer, attotime::from_hz(500))

	MCFG_DEVICE_ADD(A7_VIDEO_TAG, AGAT7VIDEO, 0)

	MCFG_RAM_ADD(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("32K")
//  MCFG_RAM_EXTRA_OPTIONS("64K,128K")
	MCFG_RAM_DEFAULT_VALUE(0x00)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD(A7_SPEAKER_TAG, SPEAKER_SOUND, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.00)

	/* /INH banking */
	MCFG_DEVICE_ADD(A7_UPPERBANK_TAG, ADDRESS_MAP_BANK, 0)
	MCFG_DEVICE_PROGRAM_MAP(inhbank_map)
	MCFG_ADDRESS_MAP_BANK_ENDIANNESS(ENDIANNESS_LITTLE)
	MCFG_ADDRESS_MAP_BANK_DATABUS_WIDTH(8)
	MCFG_ADDRESS_MAP_BANK_STRIDE(0x3000)

	/* keyboard controller -- XXX must be replaced */
	MCFG_DEVICE_ADD(A7_KBDC_TAG, AY3600, 0)
	MCFG_AY3600_MATRIX_X0(IOPORT("X0"))
	MCFG_AY3600_MATRIX_X1(IOPORT("X1"))
	MCFG_AY3600_MATRIX_X2(IOPORT("X2"))
	MCFG_AY3600_MATRIX_X3(IOPORT("X3"))
	MCFG_AY3600_MATRIX_X4(IOPORT("X4"))
	MCFG_AY3600_MATRIX_X5(IOPORT("X5"))
	MCFG_AY3600_MATRIX_X6(IOPORT("X6"))
	MCFG_AY3600_MATRIX_X7(IOPORT("X7"))
	MCFG_AY3600_MATRIX_X8(IOPORT("X8"))
	MCFG_AY3600_SHIFT_CB(READLINE(agat7_state, ay3600_shift_r))
	MCFG_AY3600_CONTROL_CB(READLINE(agat7_state, ay3600_control_r))
	MCFG_AY3600_DATA_READY_CB(WRITELINE(agat7_state, ay3600_data_ready_w))
	MCFG_AY3600_AKO_CB(WRITELINE(agat7_state, ay3600_ako_w))

	/* repeat timer.  10 Hz per Mymrin's book */
	MCFG_TIMER_DRIVER_ADD_PERIODIC("repttmr", agat7_state, ay3600_repeat, attotime::from_hz(10))

	/*
	 * slot 0 is reserved for SECAM encoder or Apple II compat card.
	 * slot 1 always holds the CPU card.
	 */
	MCFG_DEVICE_ADD(A7_BUS_TAG, A2BUS, 0)
	MCFG_A2BUS_CPU(A7_CPU_TAG)
	MCFG_A2BUS_OUT_IRQ_CB(WRITELINE(agat7_state, a2bus_irq_w))
	MCFG_A2BUS_OUT_NMI_CB(WRITELINE(agat7_state, a2bus_nmi_w))
	MCFG_A2BUS_OUT_INH_CB(WRITELINE(agat7_state, a2bus_inh_w))
	MCFG_A2BUS_SLOT_ADD(A7_BUS_TAG, "sl2", agat7_cards, "a7lang")
	MCFG_A2BUS_SLOT_ADD(A7_BUS_TAG, "sl3", agat7_cards, "a7fdc")
	MCFG_A2BUS_SLOT_ADD(A7_BUS_TAG, "sl4", agat7_cards, nullptr)
	MCFG_A2BUS_SLOT_ADD(A7_BUS_TAG, "sl5", agat7_cards, nullptr)
	MCFG_A2BUS_SLOT_ADD(A7_BUS_TAG, "sl6", agat7_cards, "a7ram")

	MCFG_CASSETTE_ADD(A7_CASSETTE_TAG)
	MCFG_CASSETTE_DEFAULT_STATE(CASSETTE_STOPPED)
MACHINE_CONFIG_END


/***************************************************************************

  Game driver(s)

***************************************************************************/

ROM_START( agat7 )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASEFF )
	ROM_DEFAULT_BIOS("v1")

	ROM_SYSTEM_BIOS( 0, "v1", "Version 1" ) // original?
	ROMX_LOAD( "monitor7.rom", 0x3800, 0x0800, CRC(071fda0b) SHA1(6089d46b7addc4e2ae096b2cf81124681bd2b27a), ROM_BIOS(1))
	ROM_SYSTEM_BIOS( 1, "v2", "Version 2" ) // modded by author of agatcomp.ru
	ROMX_LOAD( "agat_pzu.bin", 0x3800, 0x0800, CRC(c605163d) SHA1(b30fd1b264a347a9de69bb9e3105483254994d06), ROM_BIOS(2))
	ROM_SYSTEM_BIOS( 2, "debug", "Debug" )  // written by author of agatcomp.ru
	ROMX_LOAD( "debug-sysmon7.bin", 0x3800, 0x0800, CRC(d26f18a4) SHA1(2862c13a82e2f4dfc757aa2eeab11fe71c570c12), ROM_BIOS(3))

	// 140KB floppy controller
	ROM_LOAD( "shugart7.rom", 0x4500, 0x0100, CRC(c6e4850c) SHA1(71626d3d2d4bbeeac2b77585b45a5566d20b8d34))
	// 840KB floppy controller
	ROM_LOAD( "teac.rom",     0x4500, 0x0100, CRC(94266928) SHA1(5d369bad6cdd6a70b0bb16480eba69640de87a2e))

	ROM_REGION(0x0800,"gfx1",0)
	ROM_LOAD( "agathe7.fnt", 0x0000, 0x0800, CRC(fcffb490) SHA1(0bda26ae7ad75f74da835c0cf6d9928f9508844c))
ROM_END

ROM_START( agat9 )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASEFF )
	ROM_SYSTEM_BIOS( 0, "v1", "Version 1" )
	ROMX_LOAD( "monitor9.rom", 0x3800, 0x0800, CRC(b90bb66a) SHA1(02217f0785913b41fc25eabcff70fa814799c69a), ROM_BIOS(1))
	ROM_SYSTEM_BIOS( 1, "v2", "Version 2" )
	ROMX_LOAD( "monitor91.rom", 0x3800, 0x0800, CRC(89b10fc1) SHA1(7fe1ede32b5525255f82597ca9c3c2034c5996fa), ROM_BIOS(2))
	// Floppy controllers
	ROM_LOAD( "shugart9.rom", 0x4500, 0x0100, CRC(964a0ce2) SHA1(bf955189ebffe874c20ef649a3db8177dc16af61))
	ROM_LOAD( "teac.rom",     0x4500, 0x0100, CRC(94266928) SHA1(5d369bad6cdd6a70b0bb16480eba69640de87a2e))
	// Printer card
	ROM_LOAD( "cm6337.rom", 0x8000, 0x0100, CRC(73be16ec) SHA1(ead1abbef5b86f1def0b956147d5b267f0d544b5))
	ROM_LOAD( "cm6337p.rom", 0x8100, 0x0800, CRC(9120f11f) SHA1(78107653491e88d5ea12e07367c4c028771a4aca))
	ROM_REGION(0x0800,"gfx1",0)
	ROM_LOAD( "agathe9.fnt", 0x0000, 0x0800, CRC(8c55c984) SHA1(5a5a202000576b88b4ae2e180dd2d1b9b337b594))
ROM_END

//    YEAR  NAME      PARENT    COMPAT    MACHINE      INPUT    STATE         INIT      COMPANY            FULLNAME  FLAGS
COMP( 1983, agat7,    apple2,   0,        agat7,       agat7,   agat7_state,  0,        "Agat",            "Agat-7", MACHINE_NOT_WORKING)
COMP( 1984, agat9,    apple2,   0,        agat7,       agat7,   agat7_state,  0,        "Agat",            "Agat-9", MACHINE_NOT_WORKING)
