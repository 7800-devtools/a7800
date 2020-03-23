// license:BSD-3-Clause
// copyright-holders:Luca Elia
/***************************************************************************

    ms1_tmap.cpp

    8x8/16x16 tilemap generator for Jaleco's Mega System 1 and driving
    and mahjong games from the same period.

***************************************************************************/

#include "emu.h"
#include "video/ms1_tmap.h"

static constexpr int TILES_PER_PAGE_X = 0x20;
static constexpr int TILES_PER_PAGE_Y = 0x20;
static constexpr int TILES_PER_PAGE = TILES_PER_PAGE_X * TILES_PER_PAGE_Y;

/*

  A page is 256x256, approximately the visible screen size. Each layer is
  made up of 8 pages (8x8 tiles) or 32 pages (16x16 tiles). The number of
  horizontal  pages and the tiles size  is selectable, using the  layer's
  control register. I think that when tiles are 16x16 a layer can be made
  of 16x2, 8x4, 4x8 or 2x16 pages (see below). When tile size is 8x8 we
  have two examples to guide the choice:

  the copyright screen of p47j (0x12) should be 4x2 (unless it's been hacked :)
  the ending sequence of 64th street (0x13) should be 2x4.

                                                       Mega System 1
        Tile Format:            Cisco Heat              F1 GP Star

                Colour      fedc b--- ---- ----     fedc ---- ---- ----
                Code        ---- -a98 7654 3210     ---- ba98 7654 3210

*/

DEFINE_DEVICE_TYPE(MEGASYS1_TILEMAP, megasys1_tilemap_device, "ms1_tmap", "Mega System 1 Tilemap")

megasys1_tilemap_device::megasys1_tilemap_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, MEGASYS1_TILEMAP, tag, owner, clock),
		device_gfx_interface(mconfig, *this),
		m_scrollram(*this, DEVICE_SELF),
		m_8x8_scroll_factor(1),
		m_16x16_scroll_factor(4),
		m_bits_per_color_code(4),
		m_colorbase(0)
{
}

void megasys1_tilemap_device::static_set_8x8_scroll_factor(device_t &device, int scroll_factor)
{
	downcast<megasys1_tilemap_device &>(device).m_8x8_scroll_factor = scroll_factor;
}

void megasys1_tilemap_device::static_set_16x16_scroll_factor(device_t &device, int scroll_factor)
{
	downcast<megasys1_tilemap_device &>(device).m_16x16_scroll_factor = scroll_factor;
}

void megasys1_tilemap_device::static_set_bits_per_color_code(device_t &device, int bits)
{
	downcast<megasys1_tilemap_device &>(device).m_bits_per_color_code = bits;
}

void megasys1_tilemap_device::static_set_colorbase(device_t &device, uint16_t colorbase)
{
	downcast<megasys1_tilemap_device &>(device).m_colorbase = colorbase;
}



/*************************************
 *
 *  Graphics definitions
 *
 *************************************/

static const gfx_layout tilelayout =
{
	8,8,
	RGN_FRAC(1,1),
	4,
	{ STEP4(0,1)   },
	{ STEP8(0,4)   },
	{ STEP8(0,4*8) },
	8*8*4
};

GFXDECODE_MEMBER(megasys1_tilemap_device::gfxinfo)
	GFXDECODE_DEVICE(DEVICE_SELF, 0, tilelayout, 0, 16)
GFXDECODE_END

//-------------------------------------------------
//  device_start: Start up the device
//-------------------------------------------------

void megasys1_tilemap_device::device_start()
{
	// decode our graphics
	decode_gfx(gfxinfo);
	gfx(0)->set_colorbase(m_colorbase);
	gfx(0)->set_colors(1 << m_bits_per_color_code);

	// create 16x16 tilemaps
	m_tilemap[0][0] = &machine().tilemap().create(
			*this, tilemap_get_info_delegate(FUNC(megasys1_tilemap_device::get_scroll_tile_info_16x16),this), tilemap_mapper_delegate(FUNC(megasys1_tilemap_device::scan_16x16),this),
			8,8, TILES_PER_PAGE_X * 16, TILES_PER_PAGE_Y * 2);
	m_tilemap[0][1] = &machine().tilemap().create(
			*this, tilemap_get_info_delegate(FUNC(megasys1_tilemap_device::get_scroll_tile_info_16x16),this), tilemap_mapper_delegate(FUNC(megasys1_tilemap_device::scan_16x16),this),
			8,8, TILES_PER_PAGE_X * 8, TILES_PER_PAGE_Y * 4);
	m_tilemap[0][2] = &machine().tilemap().create(
			*this, tilemap_get_info_delegate(FUNC(megasys1_tilemap_device::get_scroll_tile_info_16x16),this), tilemap_mapper_delegate(FUNC(megasys1_tilemap_device::scan_16x16),this),
			8,8, TILES_PER_PAGE_X * 4, TILES_PER_PAGE_Y * 8);
	m_tilemap[0][3] = &machine().tilemap().create(
			*this, tilemap_get_info_delegate(FUNC(megasys1_tilemap_device::get_scroll_tile_info_16x16),this), tilemap_mapper_delegate(FUNC(megasys1_tilemap_device::scan_16x16),this),
			8,8, TILES_PER_PAGE_X * 2, TILES_PER_PAGE_Y * 16);

	// create 8x8 tilemaps
	m_tilemap[1][0] = &machine().tilemap().create(
			*this, tilemap_get_info_delegate(FUNC(megasys1_tilemap_device::get_scroll_tile_info_8x8),this), tilemap_mapper_delegate(FUNC(megasys1_tilemap_device::scan_8x8),this),
			8,8, TILES_PER_PAGE_X * 8, TILES_PER_PAGE_Y * 1);
	m_tilemap[1][1] = &machine().tilemap().create(
			*this, tilemap_get_info_delegate(FUNC(megasys1_tilemap_device::get_scroll_tile_info_8x8),this), tilemap_mapper_delegate(FUNC(megasys1_tilemap_device::scan_8x8),this),
			8,8, TILES_PER_PAGE_X * 4, TILES_PER_PAGE_Y * 2);
	m_tilemap[1][2] = &machine().tilemap().create(
			*this, tilemap_get_info_delegate(FUNC(megasys1_tilemap_device::get_scroll_tile_info_8x8),this), tilemap_mapper_delegate(FUNC(megasys1_tilemap_device::scan_8x8),this),
			8,8, TILES_PER_PAGE_X * 4, TILES_PER_PAGE_Y * 2);
	m_tilemap[1][3] = &machine().tilemap().create(
			*this, tilemap_get_info_delegate(FUNC(megasys1_tilemap_device::get_scroll_tile_info_8x8),this), tilemap_mapper_delegate(FUNC(megasys1_tilemap_device::scan_8x8),this),
			8,8, TILES_PER_PAGE_X * 2, TILES_PER_PAGE_Y * 4);

	// set transparency
	for (int i = 0; i < 8; i++)
		m_tilemap[i/4][i%4]->set_transparent_pen(15);

	m_tmap = m_tilemap[0][0];
	m_scroll_flag = m_scrollx = m_scrolly = 0;

	save_item(NAME(m_scrollx));
	save_item(NAME(m_scrolly));
	save_item(NAME(m_scroll_flag));
}

void megasys1_tilemap_device::device_post_load()
{
	m_tmap = m_tilemap[(m_scroll_flag >> 4) & 1][m_scroll_flag & 3];
}

/***************************************************************************

                            Layers declarations:

                    * Read and write handlers for the layer
                    * Callbacks for the TileMap code

***************************************************************************/

WRITE16_MEMBER(megasys1_tilemap_device::write)
{
	COMBINE_DATA(&m_scrollram[offset]);
	if (offset < 0x40000/2)
	{
		if (m_scroll_flag & 0x10)
		{
			// tiles are 8x8
			m_tmap->mark_tile_dirty(offset);
		}
		else
		{
			// tiles are 16x16
			m_tmap->mark_tile_dirty(offset*4 + 0);
			m_tmap->mark_tile_dirty(offset*4 + 1);
			m_tmap->mark_tile_dirty(offset*4 + 2);
			m_tmap->mark_tile_dirty(offset*4 + 3);
		}
	}
}

/***************************************************************************

                            Video registers access

***************************************************************************/


/*      Tilemap Size (PagesX x PagesY)

        Reg. Value          16          8       <- Tile Size

            0               16 x  2     8 x 1
            1                8 x  4     4 x 2
            2                4 x  8     4 x 2
            3                2 x 16     2 x 4
*/

TILEMAP_MAPPER_MEMBER(megasys1_tilemap_device::scan_8x8)
{
	return (col * TILES_PER_PAGE_Y) +
			(row / TILES_PER_PAGE_Y) * TILES_PER_PAGE * (num_cols / TILES_PER_PAGE_X) +
			(row % TILES_PER_PAGE_Y);
}

TILEMAP_MAPPER_MEMBER(megasys1_tilemap_device::scan_16x16)
{
	return ( ((col / 2) * (TILES_PER_PAGE_Y / 2)) +
				((row / 2) / (TILES_PER_PAGE_Y / 2)) * (TILES_PER_PAGE / 4) * (num_cols / TILES_PER_PAGE_X) +
				((row / 2) % (TILES_PER_PAGE_Y / 2)) )*4 + (row&1) + (col&1)*2;
}

/*
    The tile code of a specific layer is multiplied for a constant
    depending on the tile mode (8x8 or 16x16)

    The most reasonable arrangement seems a 1:1 mapping (meaning we
    must multiply by 4 the tile code in 16x16 mode, since we decode
    the graphics like 8x8)

    However, this is probably a game specific thing, as Soldam uses
    layer 1 in both modes, and even with 8x8 tiles the tile code must
    be multiplied by 4! (for the High Score table)

    AFAIK, the other games use a layer in one mode only (always 8x8 or
    16x16) so it could be that the multiplication factor is constant
    for each layer and hardwired to 1x or 4x for both tile sizes
*/

TILE_GET_INFO_MEMBER(megasys1_tilemap_device::get_scroll_tile_info_8x8)
{
	uint16_t code = m_scrollram[tile_index];
	SET_TILE_INFO_MEMBER(0, (code & 0xfff) * m_8x8_scroll_factor, code >> (16 - m_bits_per_color_code), 0);
}

TILE_GET_INFO_MEMBER(megasys1_tilemap_device::get_scroll_tile_info_16x16)
{
	uint16_t code = m_scrollram[tile_index/4];
	SET_TILE_INFO_MEMBER(0, (code & 0xfff) * m_16x16_scroll_factor + (tile_index & 3), code >> (16 - m_bits_per_color_code), 0);
}

READ16_MEMBER(megasys1_tilemap_device::scroll_r)
{
	switch (offset)
	{
		case 0: return m_scrollx;
		case 1: return m_scrolly;
		case 2: return m_scroll_flag;
		default: return 0xffff;
	}
}

WRITE16_MEMBER(megasys1_tilemap_device::scroll_w)
{
	switch (offset)
	{
		case 0:
		{
			COMBINE_DATA(&m_scrollx);
			break;
		}
		case 1:
		{
			COMBINE_DATA(&m_scrolly);
			break;
		}
		case 2:
		{
			if (((m_scroll_flag ^ data) & mem_mask) != 0)
			{
				COMBINE_DATA(&m_scroll_flag);
				logerror("Setting scroll flag: %02X\n", m_scroll_flag);
				m_tmap = m_tilemap[(m_scroll_flag >> 4) & 1][m_scroll_flag & 3];
				m_tmap->mark_all_dirty();
			}
			break;
		}
		default:
		{
			logerror("vreg %04X <- %04X", offset, data);
			break;
		}
	}
}

/***************************************************************************


                                Tilemap Drawing


***************************************************************************/

void megasys1_tilemap_device::draw(screen_device &screen, bitmap_ind16 &dest, const rectangle &cliprect, uint32_t flags, uint8_t priority, uint8_t priority_mask)
{
	m_tmap->set_scrollx(0, m_scrollx);
	m_tmap->set_scrolly(0, m_scrolly);
	m_tmap->draw(screen, dest, cliprect, flags, priority, priority_mask);
}

void megasys1_tilemap_device::enable(bool enable)
{
	m_tmap->enable(enable);
}

void megasys1_tilemap_device::set_flip(uint32_t attributes)
{
	m_tmap->set_flip(attributes);
}
