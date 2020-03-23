// license:LGPL-2.1+
// copyright-holders:Ville Linde, Angelo Salese, hap
/*************************************************************************

  Taito JC System

  functions to emulate the video hardware

*************************************************************************/

#include "emu.h"
#include "video/poly.h"
#include "includes/taitojc.h"

static const gfx_layout taitojc_char_layout =
{
	16,16,
	0x80,
	4,
	{ 0,1,2,3 },
	{ 24,28,16,20,8,12,0,4, 56,60,48,52,40,44,32,36 },
	{ 0*64, 1*64, 2*64, 3*64, 4*64, 5*64, 6*64, 7*64,8*64,9*64,10*64,11*64,12*64,13*64,14*64,15*64 },
	16*64
};

TILE_GET_INFO_MEMBER(taitojc_state::taitojc_tile_info)
{
	uint32_t val = m_tile_ram[tile_index];
	int color = (val >> 22) & 0xff;
	int tile = (val >> 2) & 0x7f;
	SET_TILE_INFO_MEMBER(m_gfx_index, tile, color, 0);
}

READ32_MEMBER(taitojc_state::taitojc_palette_r)
{
	return m_palette_ram[offset];
}

WRITE32_MEMBER(taitojc_state::taitojc_palette_w)
{
	int r, g, b;
	uint32_t color;

	COMBINE_DATA( m_palette_ram + offset );

	color = m_palette_ram[offset];
	r = (color >>  8) & 0xff;
	g = (color >> 16) & 0xff;
	b = (color >>  0) & 0xff;

	m_palette->set_pen_color(offset, rgb_t(r, g, b));
}

READ32_MEMBER(taitojc_state::taitojc_tile_r)
{
	return m_tile_ram[offset];
}

READ32_MEMBER(taitojc_state::taitojc_char_r)
{
	return m_char_ram[offset];
}

WRITE32_MEMBER(taitojc_state::taitojc_tile_w)
{
	COMBINE_DATA(m_tile_ram.get() + offset);
	m_tilemap->mark_tile_dirty(offset);
}

WRITE32_MEMBER(taitojc_state::taitojc_char_w)
{
	COMBINE_DATA(m_char_ram.get() + offset);
	m_gfxdecode->gfx(m_gfx_index)->mark_dirty(offset/32);
}

// Object data format:
//
// 0x00:   xxxxxx-- -------- -------- --------   Height
// 0x00:   ------xx xxxxxxxx -------- --------   Y
// 0x00:   -------- -------- xxxxxx-- --------   Width
// 0x00:   -------- -------- ------xx xxxxxxxx   X
// 0x01:   ---xxxxx xx------ -------- --------   Palette
// 0x01:   -------- --x----- -------- --------   Priority (0 = below 3D, 1 = above 3D)
// 0x01:   -------- -------x -------- --------   Color depth (0) 4bpp / (1) 8bpp
// 0x01:   -------- -------- -xxxxxxx xxxxxxxx   VRAM data address

/*
    Object RAM is grouped in three different banks (0-0x400 / 0x400-0x800 / 0x800-0xc00),
    Initial 6 dwords aren't surely for object stuff (setting global object flags?)
    0xd00-0xdff seems to be a per-bank vregister. Usage of this is mostly unknown, the only
    clue we have so far is this config change in dendego:
    0x2000db3f 0x3f3f3f3f 0xfec00090 0x403f00ff 0xd20-0xd2f on Taito logo
    0x2000db3f 0x3f3f3f3f 0xff600090 0x207f00ff 0xd20-0xd2f on intro FMV

    dword 0 bits 14-15 looks up to the object RAM for the given bank. (it's mostly fixed to 0,
    1 and 2 for each bank). Then dwords 2 and 3 should presumably configure bank 1 to a bigger
    (doubled?) height and width and a different x/y start point.


    0xfc0-0xfff is global vregs. 0xfc4 bit 13 is used to swap between bank 0 and bank 1?
    It's unknown at current time how bank 2 should show up.

        fc0 00000000   always
        fc4 c01f0000   boot-up, testmode, sidebs always, sidebs2 always
            c0100000   landgear in-game, dendego2 in-game
            c0310000   dendego in-game
            c0312000   dendego intro 3d parts
            c031f000   dendego disclaimer screen (only for a few frames)

        fc4 11000000 00------ ----0000 00000000   always 0/1
            -------- --xx---- -------- --------   ?
            -------- ----xxxx -------- --------   one of these probably disables textlayer, unknown function otherwise
            -------- -------- xxxx---- --------   object bank related

        fc8 40000000   always
        fcc 00000000   always
        fd0 c0000000   always
        ...
        ffc c0000000   always

*/

void taitojc_state::draw_object(bitmap_ind16 &bitmap, const rectangle &cliprect, uint32_t w1, uint32_t w2, uint8_t bank_type)
{
	int x, y, width, height, palette;
	int i, j;
	int x1, x2, y1, y2;
	int ix, iy;
	uint32_t address;
	uint8_t *v;
	uint8_t color_depth;
	uint8_t mask_screen;

	color_depth = (w2 & 0x10000) >> 16;
	mask_screen = (w2 & 0x20000) >> 17;

	address     = (w2 & 0x7fff) * 0x20;
	if (w2 & 0x4000)
		address |= 0x40000;

	x           = ((w1 >>  0) & 0x3ff);
	if (x & 0x200)
		x |= ~0x1ff;        // sign-extend

	y           = ((w1 >> 16) & 0x3ff);
	if (y & 0x200)
		y |= ~0x1ff;        // sign-extend

	width       = ((w1 >> 10) & 0x3f) * 16;
	height      = ((w1 >> 26) & 0x3f) * 16;
	palette     = ((w2 >> 22) & 0x7f) << 8;

	/* TODO: untangle this! */
	if(address >= 0xff000)
		v = (uint8_t*)&m_objlist[(address-0xff000)/4];
	if(address >= 0xfc000)
		v = (uint8_t*)&m_char_ram[(address-0xfc000)/4];
	else if(address >= 0xf8000)
		v = (uint8_t*)&m_tile_ram[(address-0xf8000)/4];
	else
		v = (uint8_t*)&m_vram[address/4];

	/* guess, but it's probably doable via a vreg ... */
	if ((width == 0 || height == 0) && bank_type == 2)
		width = height = 16;

	if(width == 0 || height == 0)
		return;

	x1 = x;
	x2 = x + width;
	y1 = y;
	y2 = y + height;

	// trivial rejection
	if (x1 > cliprect.max_x || x2 < cliprect.min_x || y1 > cliprect.max_y || y2 < cliprect.min_y)
	{
		return;
	}

//  osd_printf_debug("draw_object: %08X %08X, X: %d, Y: %d, W: %d, H: %d\n", w1, w2, x, y, width, height);

	ix = 0;
	iy = 0;

	// clip
	if (x1 < cliprect.min_x)
	{
		ix = abs(cliprect.min_x - x1);
		x1 = cliprect.min_x;
	}
	if (x2 > cliprect.max_x)
	{
		x2 = cliprect.max_x;
	}
	if (y1 < cliprect.min_y)
	{
		iy = abs(cliprect.min_y - y1);
		y1 = cliprect.min_y;
	}
	if (y2 > cliprect.max_y)
	{
		y2 = cliprect.max_y;
	}

	/* this bit seems to set up border at left/right of screen (reads at 0xffc00) */
	if(mask_screen)
	{
		if(address != 0xffc00)
		{
			popmessage("mask screen with %08x, contact MAMEdev",address);
			return;
		}

		for (j=y1; j < y2; j++)
		{
			uint16_t *d = &bitmap.pix16(j);

			for (i=x1; i < x2; i++)
			{
				d[i] = 0x78; //TODO: black

				//index++;
			}

			//iy++;
		}
	}
	else if(!color_depth) // Densha de Go 2/2X "credit text", 4bpp
	{
		for (j=y1; j < y2; j++)
		{
			uint16_t *d = &bitmap.pix16(j);
			int index = (iy * (width / 2)) + ix;

			for (i=x1; i < x2; i+=2)
			{
				uint8_t pen = (v[BYTE4_XOR_BE(index)] & 0xf0) >> 4;
				if (pen != 0)
					d[i] = palette + pen;

				pen = (v[BYTE4_XOR_BE(index)] & 0x0f);
				if (pen != 0)
					d[i+1] = palette + pen;

				index++;
			}

			iy++;
		}
	}
	else // 8bpp
	{
		{
			for (j=y1; j < y2; j++)
			{
				uint16_t *d = &bitmap.pix16(j);
				int index = (iy * width) + ix;

				for (i=x1; i < x2; i++)
				{
					uint8_t pen = v[BYTE4_XOR_BE(index)];
					if (pen != 0)
					{
						d[i] = palette + pen;
					}

					index++;
				}

				iy++;
			}
		}
	}
}

void taitojc_state::draw_object_bank(bitmap_ind16 &bitmap, const rectangle &cliprect, uint8_t bank_type, uint8_t pri)
{
	uint16_t start_offs;
//  uint8_t double_xy;
	int i;

	start_offs = ((bank_type+1)*0x400)/4;
//  double_xy = (m_objlist[(0xd1c+bank_type*0x10)/4] & 0x20000000) >> 29;

	/* probably a core bug in there (otherwise objects sticks on screen in Densha de Go) */
	if(bank_type == 1 && (!(m_objlist[0xfc4/4] & 0x2000)))
		return;

	for (i=start_offs-2; i >= (start_offs-0x400/4); i-=2)
	{
		uint32_t w1 = m_objlist[i + 0];
		uint32_t w2 = m_objlist[i + 1];

		if (((w2 & 0x200000) >> 21) == pri)
		{
			draw_object(bitmap, cliprect, w1, w2, bank_type);
		}
	}
}


void taitojc_state::video_start()
{
	/* find first empty slot to decode gfx */
	for (m_gfx_index = 0; m_gfx_index < MAX_GFX_ELEMENTS; m_gfx_index++)
		if (m_gfxdecode->gfx(m_gfx_index) == nullptr)
			break;

	assert(m_gfx_index != MAX_GFX_ELEMENTS);

	m_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(taitojc_state::taitojc_tile_info),this), TILEMAP_SCAN_ROWS, 16, 16, 64, 64);

	m_tilemap->set_transparent_pen(0);

	m_char_ram = make_unique_clear<uint32_t[]>(0x4000/4);
	m_tile_ram = make_unique_clear<uint32_t[]>(0x4000/4);

	/* create the char set (gfx will then be updated dynamically from RAM) */
	m_gfxdecode->set_gfx(m_gfx_index, std::make_unique<gfx_element>(m_palette, taitojc_char_layout, (uint8_t *)m_char_ram.get(), 0, m_palette->entries() / 16, 0));
}

uint32_t taitojc_state::screen_update_taitojc(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	bitmap.fill(0, cliprect);

	// low priority objects
	draw_object_bank(bitmap, cliprect, 0, 0);
	draw_object_bank(bitmap, cliprect, 1, 0);
	draw_object_bank(bitmap, cliprect, 2, 0);

	// 3D layer
	m_tc0780fpa->draw(bitmap, cliprect);

	// high priority objects
	draw_object_bank(bitmap, cliprect, 0, 1);
	draw_object_bank(bitmap, cliprect, 1, 1);
	draw_object_bank(bitmap, cliprect, 2, 1);

	// text layer
	if (m_objlist[0xfc4/4] & 0x10000)
		m_tilemap->draw(screen, bitmap, cliprect, 0, 0);

	return 0;
}

uint32_t taitojc_state::screen_update_dendego(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	// update controller state in artwork
	static const uint8_t dendego_mascon_table[6] = { 0x76, 0x67, 0x75, 0x57, 0x73, 0x37 };
	static const uint8_t dendego_brake_table[11] = { 0x00, 0x05, 0x1d, 0x35, 0x4d, 0x65, 0x7d, 0x95, 0xad, 0xc5, 0xd4 };

	uint8_t btn = (ioport("BUTTONS")->read() & 0x77);
	int level;
	for (level = 5; level > 0; level--)
		if (btn == dendego_mascon_table[level]) break;

	if (level != output().get_value("counter0"))
		output().set_value("counter0", level);

	btn = m_analog_ports[0]->read() & 0xff;
	for (level = 10; level > 0; level--)
		if (btn >= dendego_brake_table[level]) break;

	if (level != output().get_value("counter1"))
		output().set_value("counter1", level);

	return screen_update_taitojc(screen, bitmap, cliprect);
}
