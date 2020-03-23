// license:BSD-3-Clause
// copyright-holders:Bryan McPhail, R. Belmont
/***************************************************************************

    Seibu Sound System v1.02, designed 1986 by Seibu Kaihatsu

    The Seibu sound system comprises of a Z80A, a YM3812, a YM3931*, and
    an Oki MSM6295.  As well as sound the Z80 can controls coins and pass
    data to the main cpu.  There are a few little quirks that make it
    worthwhile emulating in a separate file:

    * The YM3812 generates interrupt RST10, by asserting the interrupt line,
    and placing 0xd7 on the data bus.

    * The main cpu generates interrupt RST18, by asserting the interrupt line,
    and placing 0xdf on the data bus.

    A problem can occur if both the YM3812 and the main cpu try to assert
    the interrupt line at the same time.  The effect in the old Mame
    emulation would be for sound to stop playing - this is because a RST18
    cancelled out a RST10, and if a single RST10 is dropped sound stops
    as the YM3812 timer is not reset.  The problem occurs because even
    if both interrupts happen at the same time, there can only be one value
    on the data bus.  Obviously the real hardware must have some circuit
    to prevent this.  It is emulated by user timers to control the z80
    interrupt vector.

    * The YM3931 is the main/sub cpu interface, similar to Konami's K054986A
      or Taito's TC0140SYT.  It also provides the Z80 memory map and
      interrupt control.  It's not a Yamaha chip :-)

    Emulation by Bryan McPhail, mish@tendril.co.uk
    ADPCM by R. Belmont and Jarek Burczynski

***************************************************************************/

#include "emu.h"
#include "audio/seibu.h"
#include "sound/okiadpcm.h"
#include "sound/okim6295.h"


/***************************************************************************
    Seibu Sound System
***************************************************************************/

/*
    Games using encrypted sound cpu:

    Air Raid         1987   "START UP PROGRAM V1.02 (C)1986 SEIBU KAIHATSU INC."
    Cabal            1988   "Michel/Seibu    sound 11/04/88"
    Dead Angle       1988?  "START UP PROGRAM V1.02 (C)1986 SEIBU KAIHATSU INC."
    Dynamite Duke    1989   "START UP PROGRAM V1.02 (C)1986 SEIBU KAIHATSU INC."
    Toki             1989   "START UP PROGRAM V1.02 (C)1986 SEIBU KAIHATSU INC."
    Raiden (alt)     1990   "START UP PROGRAM V1.02 (C)1986 SEIBU KAIHATSU INC."

    raiden and the decrypted raidena are not identical, there are vast sections of different data.
    However, there are a few different bytes in the middle of identical data, suggesting a possible
    error in the decryption scheme: they all require an additional XOR with 0x20 and are located at
    similar addresses.
    00002422: 03 23
    000024A1: 00 20
    000024A2: 09 29
    00002822: 48 68
    000028A1: 06 26
    00002A21: 17 37
    00002A22: 00 20
    00002AA1: 12 32
    00002C21: 02 22
    00002CA1: 02 22
    00002CA2: 17 37
*/

DEFINE_DEVICE_TYPE(SEIBU_SOUND, seibu_sound_device, "seibu_sound", "Seibu Sound System")

seibu_sound_device::seibu_sound_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SEIBU_SOUND, tag, owner, clock),
		m_ym_read_cb(*this),
		m_ym_write_cb(*this),
		m_sound_cpu(*this, finder_base::DUMMY_TAG),
		m_sound_rom(*this, finder_base::DUMMY_TAG),
		m_rom_bank(*this, finder_base::DUMMY_TAG),
		m_main2sub_pending(0),
		m_sub2main_pending(0),
		m_rst10_irq(0xff),
		m_rst18_irq(0xff)
{
}

void seibu_sound_device::static_set_cpu_tag(device_t &device, const char *tag)
{
	downcast<seibu_sound_device &>(device).m_sound_cpu.set_tag(tag);
	downcast<seibu_sound_device &>(device).m_sound_rom.set_tag(tag);
}

void seibu_sound_device::static_set_rombank_tag(device_t &device, const char *tag)
{
	downcast<seibu_sound_device &>(device).m_rom_bank.set_tag(tag);
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void seibu_sound_device::device_start()
{
	m_ym_read_cb.resolve_safe(0);
	m_ym_write_cb.resolve_safe();

	if (m_sound_rom.found() && m_rom_bank.found())
	{
		if (m_sound_rom.length() > 0x10000)
		{
			m_rom_bank->configure_entries(0, (m_sound_rom.length() - 0x10000) / 0x8000, &m_sound_rom[0x10000], 0x8000);

			/* Denjin Makai definitely needs this at start-up, it never writes to the bankswitch */
			m_rom_bank->set_entry(0);
		} else
			m_rom_bank->set_base(&m_sound_rom[0x8000]);
	}

	m_main2sub[0] = m_main2sub[1] = 0;
	m_sub2main[0] = m_sub2main[1] = 0;

	save_item(NAME(m_main2sub));
	save_item(NAME(m_sub2main));
	save_item(NAME(m_main2sub_pending));
	save_item(NAME(m_sub2main_pending));
	save_item(NAME(m_rst10_irq));
	save_item(NAME(m_rst18_irq));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void seibu_sound_device::device_reset()
{
	update_irq_lines(VECTOR_INIT);
}

void seibu_sound_device::update_irq_lines(int param)
{
	// note: we use 0xff here for inactive irqline

	switch (param)
	{
		case VECTOR_INIT:
			m_rst10_irq = m_rst18_irq = 0xff;
			break;

		case RST10_ASSERT:
			m_rst10_irq = 0xd7;
			break;

		case RST10_CLEAR:
			m_rst10_irq = 0xff;
			break;

		case RST18_ASSERT:
			m_rst18_irq = 0xdf;
			break;

		case RST18_CLEAR:
			m_rst18_irq = 0xff;
			break;
	}

	if (m_sound_cpu.found())
	{
		if ((m_rst10_irq & m_rst18_irq) == 0xff) /* no IRQs pending */
			m_sound_cpu->set_input_line(0, CLEAR_LINE);
		else /* IRQ pending */
			m_sound_cpu->set_input_line_and_vector(0, ASSERT_LINE, m_rst10_irq & m_rst18_irq);
	}
	else
		return;
}


WRITE8_MEMBER( seibu_sound_device::irq_clear_w )
{
	/* Denjin Makai and SD Gundam doesn't like this, it's tied to the rst18 ack ONLY so it could be related to it. */
	//update_irq_lines(VECTOR_INIT);
}

WRITE8_MEMBER( seibu_sound_device::rst10_ack_w )
{
	/* Unused for now */
}

WRITE8_MEMBER( seibu_sound_device::rst18_ack_w )
{
	update_irq_lines(RST18_CLEAR);
}

WRITE_LINE_MEMBER( seibu_sound_device::fm_irqhandler )
{
	update_irq_lines(state ? RST10_ASSERT : RST10_CLEAR);
}

READ8_MEMBER( seibu_sound_device::ym_r )
{
	return m_ym_read_cb(offset);
}

WRITE8_MEMBER( seibu_sound_device::ym_w )
{
	m_ym_write_cb(offset, data);
}

WRITE8_MEMBER( seibu_sound_device::bank_w )
{
	if (m_rom_bank.found())
		m_rom_bank->set_entry(data & 1);
}

WRITE8_MEMBER( seibu_sound_device::coin_w )
{
	space.machine().bookkeeping().coin_counter_w(0, data & 1);
	space.machine().bookkeeping().coin_counter_w(1, data & 2);
}

READ8_MEMBER( seibu_sound_device::soundlatch_r )
{
	return m_main2sub[offset];
}

READ8_MEMBER( seibu_sound_device::main_data_pending_r )
{
	return m_sub2main_pending ? 1 : 0;
}

WRITE8_MEMBER( seibu_sound_device::main_data_w )
{
	m_sub2main[offset] = data;
}

WRITE8_MEMBER( seibu_sound_device::pending_w )
{
	/* just a guess */
	m_main2sub_pending = 0;
	m_sub2main_pending = 1;
}

READ16_MEMBER( seibu_sound_device::main_word_r )
{
	//logerror("%06x: seibu_main_word_r(%x)\n",space.device().safe_pc(),offset);
	switch (offset)
	{
		case 2:
		case 3:
			return m_sub2main[offset-2];
		case 5:
			return m_main2sub_pending ? 1 : 0;
		default:
			//logerror("%06x: seibu_main_word_r(%x)\n",space.device().safe_pc(),offset);
			return 0xffff;
	}
}

WRITE16_MEMBER( seibu_sound_device::main_word_w )
{
	//printf("%06x: seibu_main_word_w(%x,%02x)\n",space.device().safe_pc(),offset,data);
	if (ACCESSING_BITS_0_7)
	{
		switch (offset)
		{
			case 0:
			case 1:
				m_main2sub[offset] = data;
				break;
			case 4:
				update_irq_lines(RST18_ASSERT);
				break;
			case 2: //Sengoku Mahjong writes here
			case 6:
				/* just a guess */
				m_sub2main_pending = 0;
				m_main2sub_pending = 1;
				break;
			default:
				//logerror("%06x: seibu_main_word_w(%x,%02x)\n",space.device().safe_pc(),offset,data);
				break;
		}
	}
}

WRITE16_MEMBER( seibu_sound_device::main_mustb_w )
{
	if (ACCESSING_BITS_0_7)
		m_main2sub[0] = data & 0xff;
	if (ACCESSING_BITS_8_15)
		m_main2sub[1] = data >> 8;

//  logerror("seibu_main_mustb_w: %x -> %x %x\n", data, main2sub[0], main2sub[1]);

	update_irq_lines(RST18_ASSERT);
}

/***************************************************************************/

ADDRESS_MAP_START( seibu_sound_map, AS_PROGRAM, 8, seibu_sound_device )
	AM_RANGE(0x0000, 0x1fff) AM_ROM
	AM_RANGE(0x2000, 0x27ff) AM_RAM
	AM_RANGE(0x4000, 0x4000) AM_DEVWRITE("seibu_sound", seibu_sound_device, pending_w)
	AM_RANGE(0x4001, 0x4001) AM_DEVWRITE("seibu_sound", seibu_sound_device, irq_clear_w)
	AM_RANGE(0x4002, 0x4002) AM_DEVWRITE("seibu_sound", seibu_sound_device, rst10_ack_w)
	AM_RANGE(0x4003, 0x4003) AM_DEVWRITE("seibu_sound", seibu_sound_device, rst18_ack_w)
	AM_RANGE(0x4007, 0x4007) AM_DEVWRITE("seibu_sound", seibu_sound_device, bank_w)
	AM_RANGE(0x4008, 0x4009) AM_DEVREADWRITE("seibu_sound", seibu_sound_device, ym_r, ym_w)
	AM_RANGE(0x4010, 0x4011) AM_DEVREAD("seibu_sound", seibu_sound_device, soundlatch_r)
	AM_RANGE(0x4012, 0x4012) AM_DEVREAD("seibu_sound", seibu_sound_device, main_data_pending_r)
	AM_RANGE(0x4013, 0x4013) AM_READ_PORT("COIN")
	AM_RANGE(0x4018, 0x4019) AM_DEVWRITE("seibu_sound", seibu_sound_device, main_data_w)
	AM_RANGE(0x401b, 0x401b) AM_DEVWRITE("seibu_sound", seibu_sound_device, coin_w)
	AM_RANGE(0x6000, 0x6000) AM_DEVREADWRITE("oki", okim6295_device, read, write)
	AM_RANGE(0x8000, 0xffff) AM_ROMBANK("seibu_bank1")
ADDRESS_MAP_END

/***************************************************************************/

DEFINE_DEVICE_TYPE(SEI80BU, sei80bu_device, "sei80bu", "SEI80BU Encrypted Z80 Interface")

sei80bu_device::sei80bu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SEI80BU, tag, owner, clock),
		device_rom_interface(mconfig, *this, 16)
{
}

READ8_MEMBER(sei80bu_device::data_r)
{
	u16 a = offset;
	u8 src = read_byte(offset);

	if ( BIT(a,9)  &  BIT(a,8))             src ^= 0x80;
	if ( BIT(a,11) &  BIT(a,4) &  BIT(a,1)) src ^= 0x40;
	if ( BIT(a,11) & ~BIT(a,8) &  BIT(a,1)) src ^= 0x04;
	if ( BIT(a,13) & ~BIT(a,6) &  BIT(a,4)) src ^= 0x02;
	if (~BIT(a,11) &  BIT(a,9) &  BIT(a,2)) src ^= 0x01;

	if (BIT(a,13) &  BIT(a,4)) src = BITSWAP8(src,7,6,5,4,3,2,0,1);
	if (BIT(a, 8) &  BIT(a,4)) src = BITSWAP8(src,7,6,5,4,2,3,1,0);

	return src;
}

READ8_MEMBER(sei80bu_device::opcode_r)
{
	u16 a = offset;
	u8 src = read_byte(offset);

	if ( BIT(a,9)  &  BIT(a,8))             src ^= 0x80;
	if ( BIT(a,11) &  BIT(a,4) &  BIT(a,1)) src ^= 0x40;
	if (~BIT(a,13) & BIT(a,12))             src ^= 0x20;
	if (~BIT(a,6)  &  BIT(a,1))             src ^= 0x10;
	if (~BIT(a,12) &  BIT(a,2))             src ^= 0x08;
	if ( BIT(a,11) & ~BIT(a,8) &  BIT(a,1)) src ^= 0x04;
	if ( BIT(a,13) & ~BIT(a,6) &  BIT(a,4)) src ^= 0x02;
	if (~BIT(a,11) &  BIT(a,9) &  BIT(a,2)) src ^= 0x01;

	if (BIT(a,13) &  BIT(a,4)) src = BITSWAP8(src,7,6,5,4,3,2,0,1);
	if (BIT(a, 8) &  BIT(a,4)) src = BITSWAP8(src,7,6,5,4,2,3,1,0);
	if (BIT(a,12) &  BIT(a,9)) src = BITSWAP8(src,7,6,4,5,3,2,1,0);
	if (BIT(a,11) & ~BIT(a,6)) src = BITSWAP8(src,6,7,5,4,3,2,1,0);

	return src;
}

/***************************************************************************
    Seibu ADPCM device
    (MSM5205 with interface to sample ROM provided by YM3931)

    FIXME: hook up an actual MSM5205 in place of this custom implementation
***************************************************************************/

DEFINE_DEVICE_TYPE(SEIBU_ADPCM, seibu_adpcm_device, "seibu_adpcm", "Seibu ADPCM (MSM5205)")

seibu_adpcm_device::seibu_adpcm_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SEIBU_ADPCM, tag, owner, clock),
		device_sound_interface(mconfig, *this),
		m_stream(nullptr),
		m_current(0),
		m_end(0),
		m_nibble(0),
		m_playing(0),
		m_base(*this, DEVICE_SELF)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void seibu_adpcm_device::device_start()
{
	m_playing = 0;
	m_stream = machine().sound().stream_alloc(*this, 0, 1, clock());
	m_adpcm.reset();

	save_item(NAME(m_current));
	save_item(NAME(m_end));
	save_item(NAME(m_nibble));
	save_item(NAME(m_playing));
}

// "decrypt" is a bit flowery here, as it's probably just line-swapping to
// simplify PCB layout/routing rather than intentional protection, but it
// still fits, especially since the Z80s for all these games are truly encrypted.

void seibu_adpcm_device::decrypt()
{
	for (int i = 0; i < m_base.length(); i++)
	{
		m_base[i] = BITSWAP8(m_base[i], 7, 5, 3, 1, 6, 4, 2, 0);
	}
}

WRITE8_MEMBER( seibu_adpcm_device::adr_w )
{
	if (m_stream)
		m_stream->update();

	if (offset)
	{
		m_end = data<<8;
	}
	else
	{
		m_current = data<<8;
		m_nibble = 4;
	}
}

WRITE8_MEMBER( seibu_adpcm_device::ctl_w )
{
	// sequence is 00 02 01 each time.
	if (m_stream)
		m_stream->update();

	switch (data)
	{
		case 0:
			m_playing = 0;
			break;
		case 2:
			break;
		case 1:
			m_playing = 1;
			break;
	}
}


//-------------------------------------------------
//  sound_stream_update - handle a stream update
//-------------------------------------------------

void seibu_adpcm_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	stream_sample_t *dest = outputs[0];

	while (m_playing && samples > 0)
	{
		int val = (m_base[m_current] >> m_nibble) & 15;

		m_nibble ^= 4;
		if (m_nibble == 4)
		{
			m_current++;
			if (m_current >= m_end)
				m_playing = 0;
		}

		*dest++ = m_adpcm.clock(val) << 4;
		samples--;
	}
	while (samples > 0)
	{
		*dest++ = 0;
		samples--;
	}
}
