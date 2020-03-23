// license:BSD-3-Clause
// copyright-holders:Sergey Svishchev
/*********************************************************************

    agat7video.cpp

    Implementation of Agat-7 onboard video.

    5 video modes:
    - 32x32 color text
    - 64x32 mono text with reverse video
    - 64x64 color graphics
    - 128x128 color graphics
    - 256x256 mono graphics

    Character generator ROM could have 128 or 256 chars.

    C7xx: video mode select

*********************************************************************/

#include "emu.h"
#include "video/agat7.h"

#include "screen.h"


/***************************************************************************
    PARAMETERS
***************************************************************************/

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(AGAT7VIDEO, agat7video_device, "agat7video", "Agat-7 Video")

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER(agat7video_device::device_add_mconfig)
	MCFG_SCREEN_ADD("a7screen", RASTER)
	MCFG_SCREEN_RAW_PARAMS(XTAL_10_5MHz, 672, 0, 512, 312, 0, 256)
	MCFG_SCREEN_UPDATE_DRIVER(agat7video_device, screen_update)
	MCFG_SCREEN_PALETTE("a7palette")

	MCFG_PALETTE_ADD("a7palette", 16)
	MCFG_PALETTE_INIT_OWNER(agat7video_device, agat7)
MACHINE_CONFIG_END


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

agat7video_device::agat7video_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, AGAT7VIDEO, tag, owner, clock),
	m_palette(*this, "a7palette")
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void agat7video_device::device_start()
{
//  save_item(NAME(m_video_mode));
	save_item(NAME(m_start_address));
}

void agat7video_device::device_reset()
{
	// XXX to be confirmed
	m_video_mode = TEXT_LORES;
	m_start_address = 0x7800;
}


READ8_MEMBER(agat7video_device::read)
{
	do_io(offset);
	return 0;
}

WRITE8_MEMBER(agat7video_device::write)
{
	do_io(offset);
}


void agat7video_device::do_io(int offset)
{
	switch (offset & 3)
	{
	case 0:
		m_video_mode = GRAPHICS_LORES;
		m_start_address = (offset) << 9;
		logerror("offset %04X, video mode 0 (GRAPHICS_LORES)\n", m_start_address);
		break;

	case 1:
		m_video_mode = GRAPHICS_HIRES;
		m_start_address = ((offset & 0x3f) - 0x01) << 9;
		logerror("offset %04X, video mode 1 (GRAPHICS_HIRES)\n", m_start_address);
		break;

	case 2:
		if (offset > 0x80) {
			m_video_mode = TEXT_HIRES;
			m_start_address = (offset - 0x82) << 9;
			logerror("offset %04X, video mode 2 (TEXT_HIRES)\n", m_start_address);
		} else {
			m_video_mode = TEXT_LORES;
			m_start_address = (offset - 0x02) << 9;
			logerror("offset %04X, video mode 2 (TEXT_LORES)\n", m_start_address);
		}
		break;

	case 3:
		m_video_mode = GRAPHICS_MONO;
		m_start_address = ((offset & 0x3f) - 0x03) << 9;
		logerror("offset %04X, video mode 3 (GRAPHICS_MONO)\n", m_start_address);
		break;
	}
}


void agat7video_device::plot_text_character(bitmap_ind16 &bitmap, int xpos, int ypos, int xscale, uint32_t code,
	const uint8_t *textgfx_data, uint32_t textgfx_datalen, int fg, int bg)
{
	int x, y, i;
	const uint8_t *chardata;
	uint16_t color;

	/* look up the character data */
	chardata = &textgfx_data[(code * 8)];

	for (y = 0; y < 8; y++)
	{
		for (x = 0; x < 8; x++)
		{
			color = (chardata[y] & (1 << (7-x))) ? fg : bg;

			for (i = 0; i < xscale; i++)
			{
				bitmap.pix16(ypos + y, xpos + (x * xscale) + i) = color;
			}
		}
	}
}

void agat7video_device::text_update_lores(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int beginrow, int endrow)
{
	int row, col;
	uint32_t address;
	uint8_t ch, attr;
	int fg = 0;
	int bg = 0;

	beginrow = std::max(beginrow, cliprect.min_y - (cliprect.min_y % 8));
	endrow = std::min(endrow, cliprect.max_y - (cliprect.max_y % 8) + 7);

	for (row = beginrow; row <= endrow; row += 8)
	{
		for (col = 0; col < 32; col++)
		{
			/* calculate address */
			address = m_start_address + (col * 2) + (row * 8);
			ch = m_ram_dev->read(address);
			attr = m_ram_dev->read(address + 1);
			if (BIT(attr, 5)) {
				fg = BITSWAP8(attr,7,6,5,3,4,2,1,0) & 15;
				bg = 0;
			} else {
				fg = 0;
				bg = BITSWAP8(attr,7,6,5,3,4,2,1,0) & 15;
			}
			plot_text_character(bitmap, col * 16, row, 2, ch, m_char_ptr, m_char_size, fg, bg);
		}
	}
}

void agat7video_device::text_update_hires(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int beginrow, int endrow)
{
	int row, col;
	uint32_t address;
	uint8_t ch;
	int fg, bg;

	beginrow = std::max(beginrow, cliprect.min_y - (cliprect.min_y % 8));
	endrow = std::min(endrow, cliprect.max_y - (cliprect.max_y % 8) + 7);

	if (m_start_address & 0x800) {
		fg = 7; bg = 0;
	} else {
		fg = 0; bg = 7;
	}

	for (row = beginrow; row <= endrow; row += 8)
	{
		for (col = 0; col < 64; col++)
		{
			/* calculate address */
			address = m_start_address + col + (row * 8);
			ch = m_ram_dev->read(address);
			plot_text_character(bitmap, col * 8, row, 1, ch, m_char_ptr, m_char_size, fg, bg);
		}
	}
}

void agat7video_device::graph_update_mono(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int beginrow, int endrow)
{
	int row, col, b;
	uint32_t address;
	uint16_t *p;
	uint8_t gfx, v;
	int fg = 7, bg = 0;

	beginrow = std::max(beginrow, cliprect.min_y - (cliprect.min_y % 8));
	endrow = std::min(endrow, cliprect.max_y - (cliprect.max_y % 8) + 7);

	for (row = beginrow; row <= endrow; row++)
	{
		p = &bitmap.pix16(row);
		for (col = 0; col < 32; col++)
		{
			address = m_start_address + col + (row * 0x20);
			gfx = m_ram_dev->read(address);

			for (b = 0; b < 8; b++)
			{
				v = (gfx & 0x80);
				gfx <<= 1;
				*(p++) = v ? fg : bg;
				*(p++) = v ? fg : bg;
			}
		}
	}
}

void agat7video_device::graph_update_hires(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int beginrow, int endrow)
{
	int row, col, b;
	uint32_t address;
	uint16_t *p;
	uint8_t gfx, v;

	beginrow = std::max(beginrow, cliprect.min_y - (cliprect.min_y % 8));
	endrow = std::min(endrow, cliprect.max_y - (cliprect.max_y % 8) + 7);

	for (row = beginrow; row <= endrow; row++)
	{
		p = &bitmap.pix16(row);
		for (col = 0; col < 0x40; col++)
		{
			address = m_start_address + col + ((row/2) * 0x40);
			gfx = m_ram_dev->read(address);

			for (b = 0; b < 2; b++)
			{
				v = (gfx & 0xf0) >> 4;
				gfx <<= 4;
				*(p++) = v;
				*(p++) = v;
				*(p++) = v;
				*(p++) = v;
			}
		}
	}
}

void agat7video_device::graph_update_lores(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int beginrow, int endrow)
{
	int row, col, b;
	uint32_t address;
	uint16_t *p;
	uint8_t gfx, v;

	beginrow = std::max(beginrow, cliprect.min_y - (cliprect.min_y % 8));
	endrow = std::min(endrow, cliprect.max_y - (cliprect.max_y % 8) + 7);

	for (row = beginrow; row <= endrow; row++)
	{
		p = &bitmap.pix16(row);
		for (col = 0; col < 0x20; col++)
		{
			address = m_start_address + col + ((row/4) * 0x20);
			gfx = m_ram_dev->read(address);

			for (b = 0; b < 2; b++)
			{
				v = (gfx & 0xf0) >> 4;
				gfx <<= 4;
				*(p++) = v;
				*(p++) = v;
				*(p++) = v;
				*(p++) = v;
				*(p++) = v;
				*(p++) = v;
				*(p++) = v;
				*(p++) = v;
			}
		}
	}
}


uint32_t agat7video_device::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	switch (m_video_mode)
	{
	case TEXT_LORES:
		text_update_lores(screen, bitmap, cliprect, 0, 255);
		break;

	case TEXT_HIRES:
		text_update_hires(screen, bitmap, cliprect, 0, 255);
		break;

	case GRAPHICS_MONO:
		graph_update_mono(screen, bitmap, cliprect, 0, 255);
		break;

	case GRAPHICS_LORES:
		graph_update_lores(screen, bitmap, cliprect, 0, 255);
		break;

	case GRAPHICS_HIRES:
		graph_update_hires(screen, bitmap, cliprect, 0, 255);
		break;

	default:
		graph_update_mono(screen, bitmap, cliprect, 0, 255);
		break;
	}

	return 0;
}

// per http://agatcomp.ru/Reading/IiO/87-2-077.djvu
static const rgb_t agat7_palette[] =
{
	rgb_t::black(),
	rgb_t(0xFF, 0x00, 0x00),  /* White */
	rgb_t(0x00, 0xFF, 0x00),  /* White */
	rgb_t(0xFF, 0xFF, 0x00),  /* White */
	rgb_t(0x00, 0x00, 0xFF),  /* White */
	rgb_t(0xFF, 0x00, 0xFF),  /* White */
	rgb_t(0xFF, 0xFF, 0x00),  /* White */
	rgb_t(0xFF, 0xFF, 0xFF),  /* White */
	rgb_t::black(),
	rgb_t(0x7F, 0x00, 0x00),  /* White */
	rgb_t(0x00, 0x7F, 0x00),  /* White */
	rgb_t(0x7F, 0x7F, 0x00),  /* White */
	rgb_t(0x00, 0x00, 0x7F),  /* White */
	rgb_t(0x7F, 0x00, 0x7F),  /* White */
	rgb_t(0x7F, 0x7F, 0x00),  /* White */
	rgb_t(0x7F, 0x7F, 0x7F)   /* White */
};

PALETTE_INIT_MEMBER(agat7video_device, agat7)
{
	palette.set_pen_colors(0, agat7_palette, ARRAY_LENGTH(agat7_palette));
}
