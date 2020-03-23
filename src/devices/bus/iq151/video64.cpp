// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/***************************************************************************

    IQ151 video64 cartridge emulation

***************************************************************************/

#include "emu.h"
#include "video64.h"

#include "screen.h"


/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

ROM_START( iq151_video64 )
	ROM_REGION(0x0800, "chargen", ROMREGION_INVERT)
	ROM_LOAD( "iq151_video64font.rom", 0x0000, 0x0800, CRC(cb6f43c0) SHA1(4b2c1d41838d569228f61568c1a16a8d68b3dadf))

	ROM_REGION(0x0800, "videoram", ROMREGION_ERASE)
ROM_END


/* F4 Character Displayer */
static const gfx_layout iq151_video64_charlayout =
{
	6, 8,                   /* 6 x 8 characters */
	256,                    /* 256 characters */
	1,                  /* 1 bits per pixel */
	{ 0 },                  /* no bitplanes */
	/* x offsets */
	{ 2, 3, 4, 5, 6, 7 },
	/* y offsets */
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8 },
	8*8                 /* every char takes 8 bytes */
};

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(IQ151_VIDEO64, iq151_video64_device, "iq151_video64", "IQ151 video64")

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  iq151_video64_device - constructor
//-------------------------------------------------

iq151_video64_device::iq151_video64_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, IQ151_VIDEO64, tag, owner, clock)
	, device_gfx_interface(mconfig, *this, nullptr, "^^palette")
	, device_iq151cart_interface( mconfig, *this )
	, m_videoram(*this, "videoram")
	, m_chargen(*this, "chargen")
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void iq151_video64_device::device_start()
{
	set_gfx(0, std::make_unique<gfx_element>(&palette(), iq151_video64_charlayout, m_chargen, 0, 1, 0));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void iq151_video64_device::device_reset()
{
	screen_device *screen = machine().first_screen();

	// if required adjust screen size
	if (screen->visible_area().max_x < 64*6 - 1)
		screen->set_visible_area(0, 64*6-1, 0, 32*8-1);
}

//-------------------------------------------------
//  device_rom_region
//-------------------------------------------------

const tiny_rom_entry *iq151_video64_device::device_rom_region() const
{
	return ROM_NAME( iq151_video64 );
}

//-------------------------------------------------
//  read
//-------------------------------------------------

void iq151_video64_device::read(offs_t offset, uint8_t &data)
{
	// videoram is mapped at 0xe800-0xefff
	if (offset >= 0xe800 && offset < 0xf000)
		data = m_videoram[offset & 0x7ff];
}

//-------------------------------------------------
//  write
//-------------------------------------------------

void iq151_video64_device::write(offs_t offset, uint8_t data)
{
	if (offset >= 0xe800 && offset < 0xf000)
		m_videoram[offset & 0x7ff] = data;
}

//-------------------------------------------------
//  IO read
//-------------------------------------------------

void iq151_video64_device::io_read(offs_t offset, uint8_t &data)
{
	if (offset >= 0xfc && offset < 0x100)
	{
		// this value is used by the IQ151 for detect if the installed
		// cart is video64 or video32
		data = 0xfe;
	}
}

//-------------------------------------------------
//  video update
//-------------------------------------------------

void iq151_video64_device::video_update(bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	uint16_t ma = 0, sy = 0;

	for (int y = 0; y < 32; y++)
	{
		for (int ra = 0; ra < 8; ra++)
		{
			uint16_t *p = &bitmap.pix16(sy++);

			for (int x = ma; x < ma + 64; x++)
			{
				uint8_t chr = m_videoram[x];
				uint8_t gfx = m_chargen[(chr<<3) | ra ];

				/* Display a scanline of a character */
				*p++ |= BIT(gfx, 5);
				*p++ |= BIT(gfx, 4);
				*p++ |= BIT(gfx, 3);
				*p++ |= BIT(gfx, 2);
				*p++ |= BIT(gfx, 1);
				*p++ |= BIT(gfx, 0);
			}
		}
		ma += 64;
	}
}
