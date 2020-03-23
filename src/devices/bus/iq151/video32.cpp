// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/***************************************************************************

    IQ151 video32 cartridge emulation

***************************************************************************/

#include "emu.h"
#include "video32.h"
#include "screen.h"


/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

ROM_START( iq151_video32 )
	ROM_REGION(0x0400, "chargen", ROMREGION_INVERT)
	ROM_LOAD( "iq151_video32font.rom", 0x0000, 0x0400, CRC(395567a7) SHA1(18800543daf4daed3f048193c6ae923b4b0e87db))

	ROM_REGION(0x0400, "videoram", ROMREGION_ERASE)
ROM_END


/* F4 Character Displayer */
static const gfx_layout iq151_video32_charlayout =
{
	8, 8,                   /* 8 x 8 characters */
	128,                    /* 128 characters */
	1,                  /* 1 bits per pixel */
	{ 0 },                  /* no bitplanes */
	/* x offsets */
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	/* y offsets */
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8 },
	8*8                 /* every char takes 8 bytes */
};

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(IQ151_VIDEO32, iq151_video32_device, "iq151_video32", "IQ151 video32")

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  iq151_video32_device - constructor
//-------------------------------------------------

iq151_video32_device::iq151_video32_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, IQ151_VIDEO32, tag, owner, clock)
	, device_gfx_interface(mconfig, *this, nullptr, "^^palette")
	, device_iq151cart_interface(mconfig, *this)
	, m_videoram(*this, "videoram")
	, m_chargen(*this, "chargen")
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void iq151_video32_device::device_start()
{
	set_gfx(0, std::make_unique<gfx_element>(&palette(), iq151_video32_charlayout, m_chargen, 0, 1, 0));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void iq151_video32_device::device_reset()
{
	screen_device *screen = machine().first_screen();

	// if required adjust screen size
	if (screen->visible_area().max_x < 32*8 - 1)
		screen->set_visible_area(0, 32*8-1, 0, 32*8-1);
}

//-------------------------------------------------
//  device_rom_region
//-------------------------------------------------

const tiny_rom_entry *iq151_video32_device::device_rom_region() const
{
	return ROM_NAME( iq151_video32 );
}

//-------------------------------------------------
//  read
//-------------------------------------------------

void iq151_video32_device::read(offs_t offset, uint8_t &data)
{
	// videoram is mapped at 0xec00-0xefff
	if (offset >= 0xec00 && offset < 0xf000)
		data = m_videoram[offset & 0x3ff];
}

//-------------------------------------------------
//  write
//-------------------------------------------------

void iq151_video32_device::write(offs_t offset, uint8_t data)
{
	if (offset >= 0xec00 && offset < 0xf000)
		m_videoram[offset & 0x3ff] = data;
}

//-------------------------------------------------
//  video update
//-------------------------------------------------

void iq151_video32_device::video_update(bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	uint16_t ma = 0, sy = 0;

	for (int y = 0; y < 32; y++)
	{
		for (int ra = 0; ra < 8; ra++)
		{
			uint16_t *p = &bitmap.pix16(sy++);

			for (int x = ma; x < ma + 32; x++)
			{
				uint8_t chr = m_videoram[x] & 0x7f; // rom only has 128 characters
				uint8_t gfx = m_chargen[(chr<<3) | ra ];

				// chars above 0x7f have colors inverted
				if (m_videoram[x] > 0x7f)
					gfx = ~gfx;

				/* Display a scanline of a character */
				*p++ |= BIT(gfx, 7);
				*p++ |= BIT(gfx, 6);
				*p++ |= BIT(gfx, 5);
				*p++ |= BIT(gfx, 4);
				*p++ |= BIT(gfx, 3);
				*p++ |= BIT(gfx, 2);
				*p++ |= BIT(gfx, 1);
				*p++ |= BIT(gfx, 0);
			}
		}
		ma += 32;
	}
}
