// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/*
  Sega system24 hardware

System 24      68000x2  315-5292   315-5293  315-5294  315-5242        ym2151 dac           315-5195(x3) 315-5296(IO)

  System24:
    The odd one out.  Medium resolution. Entirely ram-based, no
    graphics roms.  4-layer tilemap hardware in two pairs, selection
    on a 8-pixels basis.  Tile-based sprites(!) organised as a linked
    list.  The tilemap chip has been reused for model1 and model2,
    probably because they had it handy and it handles medium res.

*/

#include "emu.h"
#include "segaic24.h"
#include "screen.h"


DEFINE_DEVICE_TYPE(S24TILE,   segas24_tile_device,   "segas24_tile",   "Sega System 24 Tilemap")
DEFINE_DEVICE_TYPE(S24SPRITE, segas24_sprite_device, "segas24_sprite", "Sega System 24 Sprites")
DEFINE_DEVICE_TYPE(S24MIXER,  segas24_mixer_device,  "segas24_mixer",  "Sega System 24 Mixer")


segas24_tile_device::segas24_tile_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, S24TILE, tag, owner, clock)
	, device_gfx_interface(mconfig, *this)
	, char_gfx_index(0)
{
}


void segas24_tile_device::static_set_tile_mask(device_t &device, uint16_t _tile_mask)
{
	segas24_tile_device &dev = downcast<segas24_tile_device &>(device);
	dev.tile_mask = _tile_mask;
}

const gfx_layout segas24_tile_device::char_layout = {
	8, 8,
	SYS24_TILES,
	4,
	{ 0, 1, 2, 3 },
	{ STEP8(0, 4) },
	{ STEP8(0, 32) },
	8*32
};

void segas24_tile_device::tile_info(int offset, tile_data &tileinfo, tilemap_memory_index tile_index)
{
	uint16_t val = tile_ram[tile_index|offset];
	SET_TILE_INFO_MEMBER(char_gfx_index, val & tile_mask, (val >> 7) & 0xff, 0);
	tileinfo.category = (val & 0x8000) != 0;
}

TILE_GET_INFO_MEMBER(segas24_tile_device::tile_info_0s)
{
	tile_info(0x0000, tileinfo, tile_index);
}

TILE_GET_INFO_MEMBER( segas24_tile_device::tile_info_0w)
{
	tile_info(0x1000, tileinfo, tile_index);
}

TILE_GET_INFO_MEMBER(segas24_tile_device::tile_info_1s)
{
	tile_info(0x2000, tileinfo, tile_index);
}

TILE_GET_INFO_MEMBER(segas24_tile_device::tile_info_1w)
{
	tile_info(0x3000, tileinfo, tile_index);
}

void segas24_tile_device::device_start()
{
	if (!palette().device().started())
		throw device_missing_dependencies();

	char_ram = std::make_unique<uint16_t[]>(0x80000/2);
	tile_ram = std::make_unique<uint16_t[]>(0x10000/2);

	tile_layer[0] = &machine().tilemap().create(*this, tilemap_get_info_delegate(FUNC(segas24_tile_device::tile_info_0s),this), TILEMAP_SCAN_ROWS,  8, 8, 64, 64);
	tile_layer[1] = &machine().tilemap().create(*this, tilemap_get_info_delegate(FUNC(segas24_tile_device::tile_info_0w),this), TILEMAP_SCAN_ROWS,  8, 8, 64, 64);
	tile_layer[2] = &machine().tilemap().create(*this, tilemap_get_info_delegate(FUNC(segas24_tile_device::tile_info_1s),this), TILEMAP_SCAN_ROWS,  8, 8, 64, 64);
	tile_layer[3] = &machine().tilemap().create(*this, tilemap_get_info_delegate(FUNC(segas24_tile_device::tile_info_1w),this), TILEMAP_SCAN_ROWS,  8, 8, 64, 64);

	tile_layer[0]->set_transparent_pen(0);
	tile_layer[1]->set_transparent_pen(0);
	tile_layer[2]->set_transparent_pen(0);
	tile_layer[3]->set_transparent_pen(0);

	memset(char_ram.get(), 0, 0x80000);
	memset(tile_ram.get(), 0, 0x10000);

	set_gfx(char_gfx_index, std::make_unique<gfx_element>(&palette(), char_layout, (uint8_t *)char_ram.get(), NATIVE_ENDIAN_VALUE_LE_BE(8,0), palette().entries() / 16, 0));

	save_pointer(NAME(tile_ram.get()), 0x10000/2);
	save_pointer(NAME(char_ram.get()), 0x80000/2);
}

void segas24_tile_device::draw_rect(screen_device &screen, bitmap_ind16 &bm, bitmap_ind8 &tm, bitmap_ind16 &dm, const uint16_t *mask,
								uint16_t tpri, uint8_t lpri, int win, int sx, int sy, int xx1, int yy1, int xx2, int yy2)
{
	int y;
	const uint16_t *source  = &bm.pix16(sy, sx);
	const uint8_t  *trans = &tm.pix8(sy, sx);
	uint8_t        *prib = &screen.priority().pix8(0);
	uint16_t       *dest = &dm.pix16(0);

	tpri |= TILEMAP_PIXEL_LAYER0;

	dest += yy1*dm.rowpixels() + xx1;
	prib += yy1*screen.priority().rowpixels() + xx1;
	mask += yy1*4;
	yy2 -= yy1;

	while(xx1 >= 128) {
		xx1 -= 128;
		xx2 -= 128;
		mask++;
	}

	for(y=0; y<yy2; y++) {
		const uint16_t *src   = source;
		const uint8_t  *srct  = trans;
		uint16_t *dst         = dest;
		uint8_t *pr           = prib;
		const uint16_t *mask1 = mask;
		int llx = xx2;
		int cur_x = xx1;

		while(llx > 0) {
			uint16_t m = *mask1++;

			if(win)
				m = ~m;

			if(!cur_x && llx>=128) {
				// Fast paths for the 128-pixels without side clipping case

				if(!m) {
					// 1- 128 pixels from this layer
					int x;
					for(x=0; x<128; x++) {
						if(*srct++ == tpri) {
							*dst = *src;
							*pr |= lpri;
						}
						src++;
						dst++;
						pr++;
					}

				} else if(m == 0xffff) {
					// 2- 128 pixels from the other layer
					src += 128;
					srct += 128;
					dst += 128;
					pr += 128;

				} else {
					// 3- 128 pixels from both layers
					int x;
					for(x=0; x<128; x+=8) {
						if(!(m & 0x8000)) {
							int xx;
							for(xx=0; xx<8; xx++)
								if(srct[xx] == tpri) {
									dst[xx] = src[xx];
									pr[xx] |= lpri;
								}
						}
						src += 8;
						srct += 8;
						dst += 8;
						pr += 8;
						m <<= 1;
					}
				}
			} else {
				// Clipped path
				int llx1 = llx >= 128 ? 128 : llx;

				if(!m) {
					// 1- 128 pixels from this layer
					int x;
					for(x = cur_x; x<llx1; x++) {
						if(*srct++ == tpri) {
							*dst = *src;
							*pr |= lpri;
						}
						src++;
						dst++;
						pr++;
					}

				} else if(m == 0xffff) {
					// 2- 128 pixels from the other layer
					src += 128 - cur_x;
					srct += 128 - cur_x;
					dst += 128 - cur_x;
					pr += 128 - cur_x;

				} else {
					// 3- 128 pixels from both layers
					int x;
					for(x=cur_x; x<llx1; x++) {
						if(*srct++ == tpri && !(m & (0x8000 >> (x >> 3)))) {
							*dst = *src;
							*pr |= lpri;
						}

						src++;
						dst++;
						pr++;
					}
				}
			}
			llx -= 128;
			cur_x = 0;
		}
		source += bm.rowpixels();
		trans  += tm.rowpixels();
		dest   += dm.rowpixels();
		prib   += screen.priority().rowpixels();
		mask   += 4;
	}
}


// The rgb version is used by model 1 & 2 which do not need to care
// about sprite priority hence the lack of support for the
// priority_bitmap

void segas24_tile_device::draw_rect(screen_device &screen, bitmap_ind16 &bm, bitmap_ind8 &tm, bitmap_rgb32 &dm, const uint16_t *mask,
								uint16_t tpri, uint8_t lpri, int win, int sx, int sy, int xx1, int yy1, int xx2, int yy2)
{
	int y;
	const uint16_t *source  = &bm.pix16(sy, sx);
	const uint8_t  *trans = &tm.pix8(sy, sx);
	uint32_t       *dest = &dm.pix32(0);
	const pen_t  *pens   = palette().pens();

	tpri |= TILEMAP_PIXEL_LAYER0;

	dest += yy1*dm.rowpixels() + xx1;
	mask += yy1*4;
	yy2 -= yy1;

	while(xx1 >= 128) {
		xx1 -= 128;
		xx2 -= 128;
		mask++;
	}

	for(y=0; y<yy2; y++) {
		const uint16_t *src   = source;
		const uint8_t  *srct  = trans;
		uint32_t *dst         = dest;
		const uint16_t *mask1 = mask;
		int llx = xx2;
		int cur_x = xx1;

		while(llx > 0) {
			uint16_t m = *mask1++;

			if(win)
				m = ~m;

			if(!cur_x && llx>=128) {
				// Fast paths for the 128-pixels without side clipping case

				if(!m) {
					// 1- 128 pixels from this layer
					int x;
					for(x=0; x<128; x++) {
						if(*srct++ == tpri)
							*dst = pens[*src];
						src++;
						dst++;
					}

				} else if(m == 0xffff) {
					// 2- 128 pixels from the other layer
					src += 128;
					srct += 128;
					dst += 128;

				} else {
					// 3- 128 pixels from both layers
					int x;
					for(x=0; x<128; x+=8) {
						if(!(m & 0x8000)) {
							int xx;
							for(xx=0; xx<8; xx++)
								if(srct[xx] == tpri)
									dst[xx] = pens[src[xx]];
						}
						src += 8;
						srct += 8;
						dst += 8;
						m <<= 1;
					}
				}
			} else {
				// Clipped path
				int llx1 = llx >= 128 ? 128 : llx;

				if(!m) {
					// 1- 128 pixels from this layer
					int x;
					for(x = cur_x; x<llx1; x++) {
						if(*srct++ == tpri)
							*dst = pens[*src];
						src++;
						dst++;
					}

				} else if(m == 0xffff) {
					// 2- 128 pixels from the other layer
					src += 128 - cur_x;
					srct += 128 - cur_x;
					dst += 128 - cur_x;

				} else {
					// 3- 128 pixels from both layers
					int x;
					for(x=cur_x; x<llx1; x++) {
						if(*srct++ == tpri && !(m & (0x8000 >> (x >> 3))))
							*dst = pens[*src];

						src++;
						dst++;
					}
				}
			}
			llx -= 128;
			cur_x = 0;
		}
		source += bm.rowpixels();
		trans  += tm.rowpixels();
		dest   += dm.rowpixels();
		mask   += 4;
	}
}

template<class _BitmapClass>
void segas24_tile_device::draw_common(screen_device &screen, _BitmapClass &bitmap, const rectangle &cliprect, int layer, int lpri, int flags)
{
	uint16_t hscr = tile_ram[0x5000+(layer >> 1)];
	uint16_t vscr = tile_ram[0x5004+(layer >> 1)];
	uint16_t ctrl = tile_ram[0x5004+((layer >> 1) & 2)];
	uint16_t *mask = tile_ram.get() + (layer & 4 ? 0x6800 : 0x6000);
	uint16_t tpri = layer & 1;

	lpri = 1 << lpri;
	layer >>= 1;

	// Layer disable
	if(vscr & 0x8000)
		return;

	if(ctrl & 0x6000) {
		// Special window/scroll modes
		if(layer & 1)
			return;

		tile_layer[layer]->set_scrolly(0, vscr & 0x1ff);
		tile_layer[layer|1]->set_scrolly(0, vscr & 0x1ff);

		if(hscr & 0x8000) {
			uint16_t *hscrtb = tile_ram.get() + 0x4000 + 0x200*layer;

			switch((ctrl & 0x6000) >> 13) {
			case 1: {
				int y;
				uint16_t v = (-vscr) & 0x1ff;
				if(!((-vscr) & 0x200))
					layer ^= 1;
				for(y=cliprect.min_y; y<=cliprect.max_y; y++) {
					uint16_t h;
					rectangle c = cliprect;
					int l1 = layer;
					if(y >= v)
						l1 ^= 1;

					c.min_y = c.max_y = y;

					hscr = hscrtb[y];

					h = hscr & 0x1ff;
					tile_layer[l1]->set_scrollx(0, -h);
					tile_layer[l1]->draw(screen, bitmap, c, tpri, lpri);
				}
				break;
			}
			case 2: case 3: {
				int y;
				for(y=cliprect.min_y; y<=cliprect.max_y; y++) {
					uint16_t h;
					rectangle c1 = cliprect;
					rectangle c2 = cliprect;
					int l1 = layer;

					hscr = hscrtb[y];

					h = hscr & 0x1ff;
					tile_layer[layer]->set_scrollx(0, -h);
					tile_layer[layer|1]->set_scrollx(0, -h);

					if(c1.max_x >= h)
						c1.max_x = h-1;
					if(c2.min_x < h)
						c2.min_x = h;
					if(!(hscr & 0x200))
						l1 ^= 1;

					c1.min_y = c1.max_y = c2.min_y = c2.max_y = y;

					tile_layer[l1]->draw(screen, bitmap, c1, tpri, lpri);
					tile_layer[l1^1]->draw(screen, bitmap, c2, tpri, lpri);
				}
				break;
			}
			}

		} else {
			tile_layer[layer]->set_scrollx(0, -(hscr & 0x1ff));
			tile_layer[layer|1]->set_scrollx(0, -(hscr & 0x1ff));

			switch((ctrl & 0x6000) >> 13) {
			case 1: {
				rectangle c1 = cliprect;
				rectangle c2 = cliprect;
				uint16_t v;
				v = (-vscr) & 0x1ff;
				if(c1.max_y >= v)
					c1.max_y = v-1;
				if(c2.min_y < v)
					c2.min_y = v;
				if(!((-vscr) & 0x200))
					layer ^= 1;

				tile_layer[layer]->draw(screen, bitmap, c1, tpri, lpri);
				tile_layer[layer^1]->draw(screen, bitmap, c2, tpri, lpri);
				break;
			}
			case 2: case 3: {
				rectangle c1 = cliprect;
				rectangle c2 = cliprect;
				uint16_t h;
				h = (+hscr) & 0x1ff;
				if(c1.max_x >= h)
					c1.max_x = h-1;
				if(c2.min_x < h)
					c2.min_x = h;
				if(!((+hscr) & 0x200))
					layer ^= 1;

				tile_layer[layer]->draw(screen, bitmap, c1, tpri, lpri);
				tile_layer[layer^1]->draw(screen, bitmap, c2, tpri, lpri);
				break;
			}
			}
		}

	} else {
		int win = layer & 1;

		bitmap_ind16 &bm = tile_layer[layer]->pixmap();
		bitmap_ind8 &tm = tile_layer[layer]->flagsmap();

		if(hscr & 0x8000) {
			int y;
			uint16_t *hscrtb = tile_ram.get() + 0x4000 + 0x200*layer;
			vscr &= 0x1ff;

			for(y=0; y<384; y++) {
				hscr = (-hscrtb[y]) & 0x1ff;
				if(hscr + 496 <= 512) {
					// Horizontal split unnecessary
					draw_rect(screen, bm, tm, bitmap, mask, tpri, lpri, win, hscr, vscr,        0,        y,      496,      y+1);
				} else {
					// Horizontal split necessary
					draw_rect(screen, bm, tm, bitmap, mask, tpri, lpri, win, hscr, vscr,        0,        y, 512-hscr,      y+1);
					draw_rect(screen, bm, tm, bitmap, mask, tpri, lpri, win,    0, vscr, 512-hscr,        y,      496,      y+1);
				}
				vscr = (vscr + 1) & 0x1ff;
			}
		} else {
			hscr = (-hscr) & 0x1ff;
			vscr = (+vscr) & 0x1ff;

			if(hscr + 496 <= 512) {
				// Horizontal split unnecessary
				if(vscr + 384 <= 512) {
					// Vertical split unnecessary
					draw_rect(screen, bm, tm, bitmap, mask, tpri, lpri, win, hscr, vscr,        0,        0,      496,      384);
				} else {
					// Vertical split necessary
					draw_rect(screen, bm, tm, bitmap, mask, tpri, lpri, win, hscr, vscr,        0,        0,      496, 512-vscr);
					draw_rect(screen, bm, tm, bitmap, mask, tpri, lpri, win, hscr,    0,        0, 512-vscr,      496,      384);

				}
			} else {
				// Horizontal split necessary
				if(vscr + 384 <= 512) {
					// Vertical split unnecessary
					draw_rect(screen, bm, tm, bitmap, mask, tpri, lpri, win, hscr, vscr,        0,        0, 512-hscr,      384);
					draw_rect(screen, bm, tm, bitmap, mask, tpri, lpri, win,    0, vscr, 512-hscr,        0,      496,      384);
				} else {
					// Vertical split necessary
					draw_rect(screen, bm, tm, bitmap, mask, tpri, lpri, win, hscr, vscr,        0,        0, 512-hscr, 512-vscr);
					draw_rect(screen, bm, tm, bitmap, mask, tpri, lpri, win,    0, vscr, 512-hscr,        0,      496, 512-vscr);
					draw_rect(screen, bm, tm, bitmap, mask, tpri, lpri, win, hscr,    0,        0, 512-vscr, 512-hscr,      384);
					draw_rect(screen, bm, tm, bitmap, mask, tpri, lpri, win,    0,    0, 512-hscr, 512-vscr,      496,      384);
				}
			}
		}
	}
}

void segas24_tile_device::draw(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int layer, int lpri, int flags)
{ draw_common(screen, bitmap, cliprect, layer, lpri, flags); }

void segas24_tile_device::draw(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect, int layer, int lpri, int flags)
{ draw_common(screen, bitmap, cliprect, layer, lpri, flags); }

READ16_MEMBER(segas24_tile_device::tile_r)
{
	return tile_ram[offset];
}

READ16_MEMBER(segas24_tile_device::char_r)
{
	return char_ram[offset];
}

WRITE16_MEMBER(segas24_tile_device::tile_w)
{
	COMBINE_DATA(tile_ram.get() + offset);
	if(offset < 0x4000)
		tile_layer[offset >> 12]->mark_tile_dirty(offset & 0xfff);
}

WRITE16_MEMBER(segas24_tile_device::char_w)
{
	uint16_t old = char_ram[offset];
	COMBINE_DATA(char_ram.get() + offset);
	if(old != char_ram[offset])
		gfx(char_gfx_index)->mark_dirty(offset / 16);
}

READ32_MEMBER(segas24_tile_device::tile32_r)
{
	return tile_r(space, offset*2, mem_mask&0xffff) | tile_r(space, (offset*2)+1, mem_mask>>16)<<16;
}

READ32_MEMBER(segas24_tile_device::char32_r)
{
	return char_r(space, offset*2, mem_mask&0xffff) | char_r(space, (offset*2)+1, mem_mask>>16)<<16;
}

WRITE32_MEMBER(segas24_tile_device::tile32_w)
{
	tile_w(space, offset*2, data&0xffff, mem_mask&0xffff);
	tile_w(space, (offset*2)+1, data>>16, mem_mask>>16);
}

WRITE32_MEMBER(segas24_tile_device::char32_w)
{
	char_w(space, offset*2, data&0xffff, mem_mask&0xffff);
	char_w(space, (offset*2)+1, data>>16, mem_mask>>16);
}


segas24_sprite_device::segas24_sprite_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, S24SPRITE, tag, owner, clock)
{
}

void segas24_sprite_device::device_start()
{
	sprite_ram = std::make_unique<uint16_t[]>(0x40000/2);

	save_pointer(NAME(sprite_ram.get()), 0x40000/2);
}

/* System24 sprites
      Normal sprite:
    0   00Z--nnn    nnnnnnnn    zoom mode (1 = separate x and y), next sprite
    1   xxxxxxxx    yyyyyyyy    zoom x, zoom y (zoom y is used for both when mode = 0)
    2   --TTTTTT    TTTTTTTT    sprite number
    3   PPPPCCCC    CCCCCCCC    priority, indirect palette base
    4   FSSSYYYY    YYYYYYYY    flipy, y size, top
    5   FSSSXXXX    XXXXXXXX    flipx, x size, left

      Clip?
    0   01---nnn    nnnnnnnn    next sprite
    1   hVH-----    --------    hide/vflip/hflip
    2   -------y    yyyyyyyy    Clip top
    2   -------x    xxxxxxxx    Clip left
    2   -------y    yyyyyyyy    Clip bottom
    2   -------x    xxxxxxxx    Clip right

      Skipped entry
    0   10---nnn    nnnnnnnn    next sprite

      End of sprite list
    0   11------    --------
*/

void segas24_sprite_device::draw(bitmap_ind16 &bitmap, const rectangle &cliprect, bitmap_ind8 &priority_bitmap, const int *spri)
{
	uint16_t curspr = 0;
	int countspr = 0;
	int seen;
	uint8_t pmt[4];
	int i;
	uint16_t *sprd[0x2000], *clip[0x2000];
	uint16_t *cclip = nullptr;

	for(i=0; i<4; i++)
		pmt[i] = 0xff << (1+spri[3-i]);

	for(seen = 0; seen < 0x2000; seen++) {
		uint16_t *source;
		uint16_t type;

		source = sprite_ram.get() + (curspr << 3);

		if(curspr == 0 && source[0] == 0)
			break;

		curspr = source[0];
		type = curspr & 0xc000;
		curspr &= 0x1fff;

		if(type == 0xc000)
			break;

		if(type == 0x8000)
			continue;

		if(type == 0x4000) {
			cclip = source;
			continue;
		}

		sprd[countspr] = source;
		clip[countspr] = cclip;

		countspr++;
		if(!curspr)
			break;
	}

	for(countspr--; countspr >= 0; countspr--) {
		uint16_t *source, *pix;
		int x, y, sx, sy;
		int px, py;
		uint16_t colors[16];
		int flipx, flipy;
		int zoomx, zoomy;
		uint8_t pm[16];
		//      int dump;
		int xmod, ymod;
		int min_x, min_y, max_x, max_y;

		uint32_t addoffset;
		uint32_t newoffset;
		uint32_t offset;

		source = sprd[countspr];
		cclip = clip[countspr];

		if(cclip) {
			min_y = (cclip[2] & 511);
			min_x = (cclip[3] & 511) - 8;
			max_y = (cclip[4] & 511);
			max_x = (cclip[5] & 511) - 8;
		} else {
			min_x = 0;
			max_x = 495;
			min_y = 0;
			max_y = 383;
		}


		if(min_x < cliprect.min_x)
			min_x = cliprect.min_x;
		if(min_y < cliprect.min_y)
			min_y = cliprect.min_y;
		if(max_x > cliprect.max_x)
			max_x = cliprect.max_x;
		if(max_y > cliprect.max_y)
			max_y = cliprect.max_y;

		if(!(source[0] & 0x2000))
			zoomx = zoomy = source[1] & 0xff;
		else {
			zoomx = source[1] >> 8;
			zoomy = source[1] & 0xff;
		}
		if(!zoomx)
			zoomx = 0x3f;
		if(!zoomy)
			zoomy = 0x3f;

		zoomx++;
		zoomy++;

		x = source[5] & 0xfff;
		flipx = source[5] & 0x8000;
		if(x & 0x800)
			x -= 0x1000;
		sx = 1 << ((source[5] & 0x7000) >> 12);

		x -= 8;

		y = source[4] & 0xfff;
		if(y & 0x800)
			y -= 0x1000;
		flipy = source[4] & 0x8000;
		sy = 1 << ((source[4] & 0x7000) >> 12);

		pix = &sprite_ram[(source[3] & 0x3fff)* 0x8];
		for(px=0; px<8; px++) {
			int c;
			c              = pix[px] >> 8;
			pm[px*2]       = pmt[c>>6];
			if(c>1)
				c |= 0x1000;
			colors[px*2]   = c;

			c              = pix[px] & 0xff;
			pm[px*2+1]     = pmt[c>>6];
			if(c>1)
				c |= 0x1000;
			colors[px*2+1] = c;
		}

		offset = (source[2] & 0x7fff) * 0x10;

		xmod = 0x20;
		ymod = 0x20;
		for(py=0; py<sy; py++) {
			int xmod1 = xmod;
			int xpos1 = x;
			int ypos1 = y, ymod1 = ymod;
			for(px=0; px<sx; px++) {
				int xmod2 = xmod1, xpos2 = xpos1;
				int zy;
				addoffset = 0x10*(flipx ? sx-px-1 : px) + 0x10*sx*(flipy ? sy-py-1 : py) + (flipy ? 7*2 : 0);
				newoffset = offset + addoffset;

				ymod1 = ymod;
				ypos1 = y;
				for(zy=0; zy<8; zy++) {
					ymod1 += zoomy;
					while(ymod1 >= 0x40) {
						if(ypos1 >= min_y && ypos1 <= max_y) {
							int zx;
							xmod2 = xmod1;
							xpos2 = xpos1;

							for(zx=0; zx<8; zx++) {
								xmod2 += zoomx;
								while(xmod2 >= 0x40) {
									if(xpos2 >= min_x && xpos2 <= max_x) {
										int zx1 = flipx ? 7-zx : zx;
										uint32_t neweroffset = (newoffset+(zx1>>2))&0x1ffff; // crackdown sometimes attempts to use data past the end of spriteram
										int c = (sprite_ram[neweroffset] >> (((~zx1) & 3) << 2)) & 0xf;
										uint8_t *pri = &priority_bitmap.pix8(ypos1, xpos2);
										if(!(*pri & pm[c])) {
											c = colors[c];
											if(c) {
												uint16_t *dst = &bitmap.pix16(ypos1, xpos2);
												if(c==1)
													*dst = (*dst) | 0x2000;
												else
													*dst = c;
												*pri = 0xff;
											}
										}
									}
									xmod2 -= 0x40;
									xpos2++;
								}
							}
						}
						ymod1 -= 0x40;
						ypos1++;
					}
					if(flipy)
						newoffset -= 2;
					else
						newoffset += 2;
				}

				xpos1 = xpos2;
				xmod1 = xmod2;
			}
			y    = ypos1;
			ymod = ymod1;
		}
	}
}


WRITE16_MEMBER(segas24_sprite_device::write)
{
	COMBINE_DATA(sprite_ram.get() + offset);
}

READ16_MEMBER(segas24_sprite_device::read)
{
	return sprite_ram[offset];
}


segas24_mixer_device::segas24_mixer_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, S24MIXER, tag, owner, clock)
{
}

void segas24_mixer_device::device_start()
{
	memset(mixer_reg, 0, sizeof(mixer_reg));
	save_item(NAME(mixer_reg));
}

WRITE16_MEMBER(segas24_mixer_device::write)
{
	COMBINE_DATA(mixer_reg + offset);
}

READ16_MEMBER(segas24_mixer_device::read)
{
	return mixer_reg[offset];
}

uint16_t segas24_mixer_device::get_reg(int reg)
{
	return mixer_reg[reg];
}
