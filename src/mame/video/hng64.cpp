// license:LGPL-2.1+
// copyright-holders:David Haywood, Angelo Salese, ElSemi, Andrew Gardner
#include "emu.h"
#include "includes/hng64.h"

#define BLEND_TEST 0

#define HNG64_VIDEO_DEBUG 0


void hng64_state::hng64_mark_all_tiles_dirty( int tilemap )
{
	m_tilemap[tilemap].m_tilemap_8x8->mark_all_dirty();
	m_tilemap[tilemap].m_tilemap_16x16->mark_all_dirty();
	m_tilemap[tilemap].m_tilemap_16x16_alt->mark_all_dirty();
}

void hng64_state::hng64_mark_tile_dirty( int tilemap, int tile_index )
{
	m_tilemap[tilemap].m_tilemap_8x8->mark_tile_dirty(tile_index);
	m_tilemap[tilemap].m_tilemap_16x16->mark_tile_dirty(tile_index);
	m_tilemap[tilemap].m_tilemap_16x16_alt->mark_tile_dirty(tile_index);
}


// make this a function!
// pppppppp ff--atttt tttttttt tttttttt
#define HNG64_GET_TILE_INFO                                                     \
{                                                                               \
	uint16_t tilemapinfo = (m_videoregs[reg]>>shift)&0xffff;                      \
	int tileno,pal, flip;                                                       \
																				\
	tileno = m_videoram[tile_index+(offset/4)];                                 \
																				\
	pal = (tileno&0xff000000)>>24;                                              \
	flip =(tileno&0x00c00000)>>22;                                              \
																				\
	if (tileno&0x200000)                                                        \
	{                                                                           \
		tileno = (tileno & m_videoregs[0x0b]) | m_videoregs[0x0c];              \
	}                                                                           \
																				\
	tileno &= 0x1fffff;                                                         \
																				\
	if (size==0)                                                                \
	{                                                                           \
		if (tilemapinfo&0x400)                                                  \
		{                                                                       \
			SET_TILE_INFO_MEMBER(1,tileno>>1,pal>>4,TILE_FLIPYX(flip));         \
		}                                                                       \
		else                                                                    \
		{                                                                       \
			SET_TILE_INFO_MEMBER(0,tileno, pal,TILE_FLIPYX(flip));              \
		}                                                                       \
	}                                                                           \
	else                                                                        \
	{                                                                           \
		if (tilemapinfo&0x400)                                                  \
		{                                                                       \
			SET_TILE_INFO_MEMBER(3,tileno>>3,pal>>4,TILE_FLIPYX(flip));         \
		}                                                                       \
		else                                                                    \
		{                                                                       \
			SET_TILE_INFO_MEMBER(2,tileno>>2, pal,TILE_FLIPYX(flip));           \
		}                                                                       \
	}                                                                           \
}

TILE_GET_INFO_MEMBER(hng64_state::get_hng64_tile0_8x8_info)
{
	int offset = 0x00000;
	int size = 0;
	int reg = 0x02;
	int shift = 16;

	HNG64_GET_TILE_INFO
}

TILE_GET_INFO_MEMBER(hng64_state::get_hng64_tile0_16x16_info)
{
	int offset = 0x00000;
	int size = 1;
	int reg = 0x02;
	int shift = 16;

	HNG64_GET_TILE_INFO
}

TILE_GET_INFO_MEMBER(hng64_state::get_hng64_tile1_8x8_info)
{
	int offset = 0x10000;
	int size = 0;
	int reg = 0x02;
	int shift = 0;

	HNG64_GET_TILE_INFO
}

TILE_GET_INFO_MEMBER(hng64_state::get_hng64_tile1_16x16_info)
{
	int offset = 0x10000;
	int size = 1;
	int reg = 0x02;
	int shift = 0;

	HNG64_GET_TILE_INFO
}

TILE_GET_INFO_MEMBER(hng64_state::get_hng64_tile2_8x8_info)
{
	int offset = 0x20000;
	int size = 0;
	int reg = 0x03;
	int shift = 16;

	HNG64_GET_TILE_INFO
}

TILE_GET_INFO_MEMBER(hng64_state::get_hng64_tile2_16x16_info)
{
	int offset = 0x20000;
	int size = 1;
	int reg = 0x03;
	int shift = 16;

	HNG64_GET_TILE_INFO
}

TILE_GET_INFO_MEMBER(hng64_state::get_hng64_tile3_8x8_info)
{
	int offset = 0x30000;
	int size = 0;
	int reg = 0x03;
	int shift = 0;

	HNG64_GET_TILE_INFO
}

TILE_GET_INFO_MEMBER(hng64_state::get_hng64_tile3_16x16_info)
{
	int offset = 0x30000;
	int size = 1;
	int reg = 0x03;
	int shift = 0;

	HNG64_GET_TILE_INFO
}


WRITE32_MEMBER(hng64_state::hng64_videoram_w)
{
	const int realoff = (offset * 4);
	COMBINE_DATA(&m_videoram[offset]);

	if ((realoff>=0) && (realoff<0x10000))
	{
		hng64_mark_tile_dirty(0, offset&0x3fff);
	}
	else if ((realoff>=0x10000) && (realoff<0x20000))
	{
		hng64_mark_tile_dirty(1, offset&0x3fff);
	}
	else if ((realoff>=0x20000) && (realoff<0x30000))
	{
		hng64_mark_tile_dirty(2, offset&0x3fff);
	}
	else if ((realoff>=0x30000) && (realoff<0x40000))
	{
		hng64_mark_tile_dirty(3, offset&0x3fff);
	}
	// Offsets 0x40000 - 0x58000 are for "floor" scanline control

	/* 400000 - 7fffff is scroll regs etc. */
}

/* internal set of transparency states for rendering */


static void hng64_configure_blit_parameters(blit_parameters *blit, tilemap_t *tmap, bitmap_rgb32 &dest, const rectangle &cliprect, uint32_t flags, uint8_t priority, uint8_t priority_mask, hng64trans_t drawformat)
{
	/* start with nothing */
	memset(blit, 0, sizeof(*blit));

	/* set the target bitmap */
	blit->bitmap = &dest;

	/* if we have a cliprect, copy */
	blit->cliprect = cliprect;

	/* set the priority code and alpha */
	//blit->tilemap_priority_code = priority | (priority_mask << 8) | (tmap->palette_offset << 16); // fixit
	blit->alpha = (flags & TILEMAP_DRAW_ALPHA_FLAG) ? (flags >> 24) : 0xff;

	blit->drawformat = drawformat;

	/* tile priority; unless otherwise specified, draw anything in layer 0 */
	blit->mask = TILEMAP_PIXEL_CATEGORY_MASK;
	blit->value = flags & TILEMAP_PIXEL_CATEGORY_MASK;

	/* if no layers specified, draw layer 0 */
	if ((flags & (TILEMAP_DRAW_LAYER0 | TILEMAP_DRAW_LAYER1 | TILEMAP_DRAW_LAYER2)) == 0)
		flags |= TILEMAP_DRAW_LAYER0;

	/* OR in the bits from the draw masks */
	blit->mask |= flags & (TILEMAP_DRAW_LAYER0 | TILEMAP_DRAW_LAYER1 | TILEMAP_DRAW_LAYER2);
	blit->value |= flags & (TILEMAP_DRAW_LAYER0 | TILEMAP_DRAW_LAYER1 | TILEMAP_DRAW_LAYER2);

	/* for all-opaque rendering, don't check any of the layer bits */
	if (flags & TILEMAP_DRAW_OPAQUE)
	{
		blit->mask &= ~(TILEMAP_PIXEL_LAYER0 | TILEMAP_PIXEL_LAYER1 | TILEMAP_PIXEL_LAYER2);
		blit->value &= ~(TILEMAP_PIXEL_LAYER0 | TILEMAP_PIXEL_LAYER1 | TILEMAP_PIXEL_LAYER2);
	}

	/* don't check category if requested */
	if (flags & TILEMAP_DRAW_ALL_CATEGORIES)
	{
		blit->mask &= ~TILEMAP_PIXEL_CATEGORY_MASK;
		blit->value &= ~TILEMAP_PIXEL_CATEGORY_MASK;
	}
}

static inline uint32_t alpha_additive_r32(uint32_t d, uint32_t s, uint8_t level)
{
	uint32_t add;
	add = (s & 0x00ff0000) + (d & 0x00ff0000);
	if (add & 0x01000000) d = (d & 0xff00ffff) | (0x00ff0000);
	else d = (d & 0xff00ffff) | (add & 0x00ff0000);
	add = (s & 0x000000ff) + (d & 0x000000ff);
	if (add & 0x00000100) d = (d & 0xffffff00) | (0x000000ff);
	else d = (d & 0xffffff00) | (add & 0x000000ff);
	add = (s & 0x0000ff00) + (d & 0x0000ff00);
	if (add & 0x00010000) d = (d & 0xffff00ff) | (0x0000ff00);
	else d = (d & 0xffff00ff) | (add & 0x0000ff00);
	return d;
}


/*-------------------------------------------------
    tilemap_draw_roz_core - render the tilemap's
    pixmap to the destination with rotation
    and zoom
-------------------------------------------------*/

#define HNG64_ROZ_PLOT_PIXEL(INPUT_VAL)                                                 \
do {                                                                                    \
	if (blit->drawformat == HNG64_TILEMAP_NORMAL)                                       \
		*(uint32_t *)dest = clut[INPUT_VAL];                                              \
	else if (blit->drawformat == HNG64_TILEMAP_ADDITIVE)                                \
		*(uint32_t *)dest = alpha_additive_r32(*(uint32_t *)dest, clut[INPUT_VAL], alpha);  \
	else if (blit->drawformat == HNG64_TILEMAP_ALPHA)                                   \
		*(uint32_t *)dest = alpha_blend_r32(*(uint32_t *)dest, clut[INPUT_VAL], alpha);     \
} while (0)

void hng64_state::hng64_tilemap_draw_roz_core(screen_device &screen, tilemap_t *tmap, const blit_parameters *blit,
		uint32_t startx, uint32_t starty, int incxx, int incxy, int incyx, int incyy, int wraparound)
{
	const pen_t *clut = &m_palette->pen(blit->tilemap_priority_code >> 16);
	bitmap_ind8 &priority_bitmap = screen.priority();
	bitmap_rgb32 &destbitmap = *blit->bitmap;
	bitmap_ind16 &srcbitmap = tmap->pixmap();
	bitmap_ind8 &flagsmap = tmap->flagsmap();
	const int xmask = srcbitmap.width()-1;
	const int ymask = srcbitmap.height()-1;
	const int widthshifted = srcbitmap.width() << 16;
	const int heightshifted = srcbitmap.height() << 16;
	uint32_t priority = blit->tilemap_priority_code;
	uint8_t mask = blit->mask;
	uint8_t value = blit->value;
	uint8_t alpha = blit->alpha;
	uint32_t cx;
	uint32_t cy;
	int x;
	int sx;
	int sy;
	int ex;
	int ey;
	uint32_t *dest;
	uint8_t *pri;
	const uint16_t *src;
	const uint8_t *maskptr;

	/* pre-advance based on the cliprect */
	startx += blit->cliprect.min_x * incxx + blit->cliprect.min_y * incyx;
	starty += blit->cliprect.min_x * incxy + blit->cliprect.min_y * incyy;

	/* extract start/end points */
	sx = blit->cliprect.min_x;
	sy = blit->cliprect.min_y;
	ex = blit->cliprect.max_x;
	ey = blit->cliprect.max_y;

	/* optimized loop for the not rotated case */
	if (incxy == 0 && incyx == 0 && !wraparound)
	{
		/* skip without drawing until we are within the bitmap */
		while (startx >= widthshifted && sx <= ex)
		{
			startx += incxx;
			sx++;
		}

		/* early exit if we're done already */
		if (sx > ex)
			return;

		/* loop over rows */
		while (sy <= ey)
		{
			/* only draw if Y is within range */
			if (starty < heightshifted)
			{
				/* initialize X counters */
				x = sx;
				cx = startx;
				cy = starty >> 16;

				/* get source and priority pointers */
				pri = &priority_bitmap.pix8(sy, sx);
				src = &srcbitmap.pix16(cy);
				maskptr = &flagsmap.pix8(cy);
				dest = &destbitmap.pix32(sy, sx);

				/* loop over columns */
				while (x <= ex && cx < widthshifted)
				{
					/* plot if we match the mask */
					if ((maskptr[cx >> 16] & mask) == value)
					{
						HNG64_ROZ_PLOT_PIXEL(src[cx >> 16]);
						*pri = (*pri & (priority >> 8)) | priority;
					}

					/* advance in X */
					cx += incxx;
					x++;
					dest++;
					pri++;
				}
			}

			/* advance in Y */
			starty += incyy;
			sy++;
		}
	}

	/* wraparound case */
	else if (wraparound)
	{
		/* loop over rows */
		while (sy <= ey)
		{
			/* initialize X counters */
			x = sx;
			cx = startx;
			cy = starty;

			/* get dest and priority pointers */
			dest = &destbitmap.pix32(sy, sx);
			pri = &priority_bitmap.pix8(sy, sx);

			/* loop over columns */
			while (x <= ex)
			{
				/* plot if we match the mask */
				if ((flagsmap.pix8((cy >> 16) & ymask, (cx >> 16) & xmask) & mask) == value)
				{
					HNG64_ROZ_PLOT_PIXEL(srcbitmap.pix16((cy >> 16) & ymask, (cx >> 16) & xmask));
					*pri = (*pri & (priority >> 8)) | priority;
				}

				/* advance in X */
				cx += incxx;
				cy += incxy;
				x++;
				dest++;
				pri++;
			}

			/* advance in Y */
			startx += incyx;
			starty += incyy;
			sy++;
		}
	}

	/* non-wraparound case */
	else
	{
		/* loop over rows */
		while (sy <= ey)
		{
			/* initialize X counters */
			x = sx;
			cx = startx;
			cy = starty;

			/* get dest and priority pointers */
			dest = &destbitmap.pix32(sy, sx);
			pri = &priority_bitmap.pix8(sy, sx);

			/* loop over columns */
			while (x <= ex)
			{
				/* plot if we're within the bitmap and we match the mask */
				if (cx < widthshifted && cy < heightshifted)
					if ((flagsmap.pix8(cy >> 16, cx >> 16) & mask) == value)
					{
						HNG64_ROZ_PLOT_PIXEL(srcbitmap.pix16(cy >> 16, cx >> 16));
						*pri = (*pri & (priority >> 8)) | priority;
					}

				/* advance in X */
				cx += incxx;
				cy += incxy;
				x++;
				dest++;
				pri++;
			}

			/* advance in Y */
			startx += incyx;
			starty += incyy;
			sy++;
		}
	}
}



void hng64_state::hng64_tilemap_draw_roz_primask(screen_device &screen, bitmap_rgb32 &dest, const rectangle &cliprect, tilemap_t *tmap,
		uint32_t startx, uint32_t starty, int incxx, int incxy, int incyx, int incyy,
		int wraparound, uint32_t flags, uint8_t priority, uint8_t priority_mask, hng64trans_t drawformat)
{
	blit_parameters blit;

	// notes:
	// - startx and starty MUST be uint32_t for calculations to work correctly
	// - srcbitmap->width and height are assumed to be a power of 2 to speed up wraparound

	// skip if disabled
	//if (!tmap->enable)
	//  return;

g_profiler.start(PROFILER_TILEMAP_DRAW_ROZ);
	/* configure the blit parameters */
	hng64_configure_blit_parameters(&blit, tmap, dest, cliprect, flags, priority, priority_mask, drawformat);

	/* get the full pixmap for the tilemap */
	tmap->pixmap();

	/* then do the roz copy */
	hng64_tilemap_draw_roz_core(screen, tmap, &blit, startx, starty, incxx, incxy, incyx, incyy, wraparound);
g_profiler.stop();
}


inline void hng64_state::hng64_tilemap_draw_roz(screen_device &screen, bitmap_rgb32 &dest, const rectangle &cliprect, tilemap_t *tmap,
		uint32_t startx, uint32_t starty, int incxx, int incxy, int incyx, int incyy,
		int wraparound, uint32_t flags, uint8_t priority, hng64trans_t drawformat)
{
	hng64_tilemap_draw_roz_primask(screen, dest, cliprect, tmap, startx, starty, incxx, incxy, incyx, incyy, wraparound, flags, priority, 0xff, drawformat);
}



/*
 * Video Regs Format (appear to just be tilemap regs)
 * --------------------------------------------------
 *
 * uint32_t | Bits                                    | Use
 *        | 3322 2222 2222 1111 1111 11             |
 * -------+-1098-7654-3210-9876-5432-1098-7654-3210-+----------------
 *   0    | ---- -Cdd ---- -??Z ---- ---- ---- ---- |  C = global complex zoom
          | 0000 0011  - road edge alt 1            | dd = global tilemap dimension selector
          | 0000 0111  - road edge alt 2            |  ? = Always Set?
          |                                         |  Z = Global Zoom Disable?
 *   1    | oooo oooo oooo oooo ---- ---- ---- ---- | unknown - 0001 is a popular value.  Explore.
 *   1    | ---- ---- ---- ---- oooo oooo oooo oooo | unknown - untouched in sams64 games, initialized elsewhere
 *   2    | xxxx xxxx xxxx xxxx ---- ---- ---- ---- | tilemap0 per layer flags
 *   2    | ---- ---- ---- ---- xxxx xxxx xxxx xxxx | tilemap1 per layer flags
 *   3    | xxxx xxxx xxxx xxxx ---- ---- ---- ---- | tilemap2 per layer flags
 *   3    | ---- ---- ---- ---- xxxx xxxx xxxx xxxx | tilemap3 per layer flags
 *   4    | xxxx xxxx xxxx xxxx ---- ---- ---- ---- | tilemap0 scrollbase when not floor, lineram offset when floor
 *   4    | ---- ---- ---- ---- xxxx xxxx xxxx xxxx | tilemap1 scrollbase when not floor, lineram offset when floor
 *   5    | xxxx xxxx xxxx xxxx ---- ---- ---- ---- | tilemap3 scrollbase when not floor, lineram offset when floor
 *   5    | ---- ---- ---- ---- xxxx xxxx xxxx xxxx | tilemap4 scrollbase when not floor, lineram offset when floor
 *   6    | oooo oooo oooo oooo oooo oooo oooo oooo | unknown - always seems to be 000001ff (fatfurwa)
 *   7    | oooo oooo oooo oooo oooo oooo oooo oooo | unknown - always seems to be 000001ff (fatfurwa)
 *   8    | oooo oooo oooo oooo oooo oooo oooo oooo | unknown - always seems to be 80008000 (fatfurwa)
 *   9    | oooo oooo oooo oooo oooo oooo oooo oooo | unknown - always seems to be 00000000 (fatfurwa)
 *   a    | oooo oooo oooo oooo oooo oooo oooo oooo | unknown - always seems to be 00000000 (fatfurwa)
 *   b    | mmmm mmmm mmmm mmmm mmmm mmmm mmmm mmmm | auto animation mask for tilemaps
 *   c    | xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx | auto animation bits for tilemaps
 *   d    | oooo oooo oooo oooo oooo oooo oooo oooo | not used ??
 *   e    | oooo oooo oooo oooo oooo oooo oooo oooo | not used ??

    // tilemap0 per layer flags
    // 0840 - startup tests, 8x8x4 layer
    // 0cc0 - beast busters 2, 8x8x8 layer
    // 0860 - fatal fury wa
    // 08e0 - fatal fury wa during transitions
    // 0940 - samurai shodown 64
    // 0880 - buriki

    // Individual tilemap regs format
    // ------------------------------
    // mmmm dbr? ??ez zzzz
    // m = Tilemap mosaic level [0-15] - confirmed in sams64 demo mode
    //  -- they seem to enable mosaic at the same time as rowscroll in several cases (floor in buriki / ff)
    //     and also on the rotating logo in buriki.. does it cause some kind of aliasing side-effect, or.. ?
    // d = line (floor) mode - buriki, fatafurwa, some backgrounds in ss64_2
    // b = 4bpp/8bpp (seems correct) (beast busters, samsh64, sasm64 2, xrally switch it for some screens)
    // r = tile size (seems correct)
    // e = tilemap enable bit according to sams64_2
    // z = z depth/priority? tilemaps might also be affected by min / max clip values somewhere?
    //              (debug layer on buriki has priority 0x020, which would be highest)
 */


void hng64_state::hng64_drawtilemap(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect, int tm)
{
	// Useful bits from the global tilemap flags
	const uint32_t& global_tileregs = m_videoregs[0x00];
	const int global_dimensions = (global_tileregs & 0x03000000) >> 24;
	const int global_alt_scroll_register_format = global_tileregs & 0x04000000;
	const int global_zoom_disable = global_tileregs & 0x00010000;

	// Debug blending on/off based on m_additive_tilemap_debug
	int debug_blend_enabled = 0;
	if ((m_additive_tilemap_debug&(1 << tm)))
		debug_blend_enabled = 1;

#if HNG64_VIDEO_DEBUG
	if ((global_dimensions != 0) && (global_dimensions != 3))
		popmessage("unsupported global_dimensions on tilemaps");
#endif

	// Determine which tilemap registers and scroll base this tilemap uses
	uint16_t tileregs = 0;
	uint16_t scrollbase = 0;
	if (tm==0)
	{
		scrollbase = (m_videoregs[0x04]&0x3fff0000)>>16;
		tileregs   = (m_videoregs[0x02]&0xffff0000)>>16;
	}
	else if (tm==1)
	{
		scrollbase = (m_videoregs[0x04]&0x00003fff)>>0;
		tileregs   = (m_videoregs[0x02]&0x0000ffff)>>0;
	}
	else if (tm==2)
	{
		scrollbase = (m_videoregs[0x05]&0x3fff0000)>>16;
		tileregs   = (m_videoregs[0x03]&0xffff0000)>>16;
	}
	else if (tm==3)
	{
		scrollbase = (m_videoregs[0x05]&0x00003fff)>>0;
		tileregs   = (m_videoregs[0x03]&0x0000ffff)>>0;
	}

	// Useful bits from the tilemap registers
	const uint8_t mosaicValueBits  = (tileregs & 0xf000) >> 12; (void)mosaicValueBits;
	const uint8_t floorModeBit     = (tileregs & 0x0800) >> 11;
	const uint8_t bppBit           = (tileregs & 0x0400) >> 10;
	const uint8_t bigTilemapBit    = (tileregs & 0x0200) >>  9;
	const uint8_t tilemapEnableBit = (tileregs & 0x0040) >>  6; (void)tilemapEnableBit;

	// Tilemap drawing enable (sams64_2 demo mode says this is legit)
	//if (!tilemapEnableBit)
	//{
	//    return;
	//}

	// Select the proper tilemap size
	tilemap_t* tilemap = nullptr;
	if (global_dimensions==0)
	{
		if (bigTilemapBit) tilemap = m_tilemap[tm].m_tilemap_16x16;
		else tilemap = m_tilemap[tm].m_tilemap_8x8;
	}
	else
	{
		if (bigTilemapBit) tilemap = m_tilemap[tm].m_tilemap_16x16_alt;
		else tilemap = m_tilemap[tm].m_tilemap_8x8; // _alt
	}

	// Set the transmask so our manual copy is correct
	int transmask = 0x00;
	if (bppBit)
		transmask = 0xff;
	else
		transmask = 0xf;

	if (floorModeBit == 0x0000)
	{
		// floor mode
		// life would be easier if the roz we're talking about for complex zoom wasn't setting this as well

		// fprintf(stderr, "Tilemap %d is a floor using :\n", tm);
		const uint32_t floorAddress = 0x40000 + (scrollbase << 4);

		// TODO: The row count is correct, but how is this layer clipped? m_tcram?

		// See how many lines we have in the data region
		// DEBUG: Change this to a loop that goes over each line and draws them - it's just for visualization now
		//int lineCount = 0;
		//for (int ii = 0; ii < 0x2000/4; ii += 4)
		//{
		//    const int realAddress = floorAddress/4;
		//    if (m_videoram[realAddress+ii] == 0xffffff00 && m_videoram[realAddress+ii+1] == 0xffffff00)
		//        continue;
		//    if (m_videoram[realAddress+ii] == 0x00000000 && m_videoram[realAddress+ii+1] == 0x00000000)
		//        continue;
		//
		//    lineCount++;
		//}
		//printf("lines %d\n", lineCount);

		// Buriki uses a 2x mosaic effect on its floor, so its line count is half
		// (but so does fatfurwa - maybe it overdraws a bunch of pixels?)
		//if (m_mcu_type == BURIKI_MCU)
		//    lineCount *= 2;

		// DEBUG - draw a horizontal green line where the uppermost line of the floor is drawn
		const rectangle &visarea = screen.visible_area();
		//if (lineCount < visarea.height())
		//{
		//    for (int ii = 0; ii < visarea.width(); ii++)
		//        bitmap.pix32((visarea.height()-lineCount), ii) = 0xff00ff00;
		//}

		// HACK : Clear RAM - this is "needed" in fatfurwa since it doesn't clear its own ram (buriki does)
		//        Figure out what the difference between the two programs is.  It's possible writing to
		//        the linescroll ram fills a buffer and it's cleared automatically between frames?
		for (int ii = 0; ii < 0x2000/4; ii++)
		{
			const int realAddress = floorAddress/4;
			m_videoram[realAddress+ii] = 0x00000000;
		}


		// Floor mode - per pixel simple / complex modes? -- every other line?
		//  (there doesn't seem to be enough data in Buriki for every line at least)
		rectangle clip = visarea;

		if (global_alt_scroll_register_format) // globally selects alt scroll register layout???
		{
			// Logic would dictate that this should be the 'complex' scroll register layout,
			// but per-line.  That doesn't work however.
			//
			// You only have line data for the number of lines on the screen, not enough for
			// the complex register layout
			//
			// HOWEVER, using the code below doesn't work either.  This might be because
			// they have mosaic turned on, and it adopts a new meaning in linescroll modes?
			//
			// The code below could also be wrong, and rowscroll simply acts the same in all
			// modes, this is hard to know because ss64_2 barely uses it.
			//
			// buriki line data is at 20146000 (physical)

#if HNG64_VIDEO_DEBUG
			popmessage("Unhandled rowscroll %02x", tileregs>>12);
#endif
		}
		else // 'simple' mode with linescroll, used in some ss64_2 levels (assumed to be correct, but doesn't do much with it.. so could be wrong)
		{
			int32_t xtopleft, xmiddle;
			int32_t ytopleft, ymiddle;

			for (int line=0; line < 448; line++)
			{
				clip.min_y = clip.max_y = line;

				if (global_zoom_disable) // disable all scrolling / zoom (test screen) (maybe)
				{
					// If this bit is active the scroll registers don't seem valid at all?
					// It either disables zooming, or disables use of the scroll registers completely
					// - used at startup

					xtopleft = 0;
					xmiddle = 256<<16;

					ytopleft = 0;
					ymiddle = 256<<16;
				}
				else
				{
					xtopleft = (m_videoram[(0x40000+(line*0x10)+(scrollbase<<4))/4]);
					xmiddle  = (m_videoram[(0x40004+(line*0x10)+(scrollbase<<4))/4]); // middle screen point
					ytopleft = (m_videoram[(0x40008+(line*0x10)+(scrollbase<<4))/4]);
					ymiddle  = (m_videoram[(0x4000c+(line*0x10)+(scrollbase<<4))/4]); // middle screen point
				}

				const int xinc = (xmiddle - xtopleft) / 512;
				const int yinc = (ymiddle - ytopleft) / 512;

				hng64_tilemap_draw_roz(screen, bitmap,clip,tilemap,xtopleft,ytopleft,
						xinc<<1,0,0,yinc<<1,
						1,
						0,0, debug_blend_enabled?HNG64_TILEMAP_ADDITIVE:HNG64_TILEMAP_NORMAL);
			}
		}
	}
	else
	{
		// 0x1000 is set up the buriki 2nd title screen with rotating logo and in fatal fury at various times?

		if (global_alt_scroll_register_format) // globally selects alt scroll register layout???
		{
			/* complex zoom mode? */
			/* with this scroll register layout rotation effects are possible
			   the most obvious use of rotation is the Buriki One logo after
			   attract mode; the text around the outside of the logo is rotated
			   onto the screen

			   see 1:32 in http://www.youtube.com/watch?v=PoYaHOILuGs

			   Xtreme Rally seems to have an issue with this mode on the communication check
			   screen at startup, but according to videos that should scroll, and no scroll
			   values are updated, so it might be an unrelated bug.

			*/

			int32_t xtopleft,xmiddle, xalt;
			int32_t ytopleft,ymiddle, yalt;
			int xinc, xinc2, yinc, yinc2;

#if HNG64_VIDEO_DEBUG
			if (0)
				if (tm==2)
					popmessage("X %08x X %08x X %08x Y %08x Y %08x Y %08x",
						m_videoram[(0x40000+(scrollbase<<4))/4],
						m_videoram[(0x40004+(scrollbase<<4))/4],
						m_videoram[(0x40010+(scrollbase<<4))/4],
						/*m_videoram[(0x40014+(scrollbase<<4))/4],*/  // unused? (dupe value on fatfurwa, 00 on rest)

						m_videoram[(0x40008+(scrollbase<<4))/4],
						m_videoram[(0x40018+(scrollbase<<4))/4],
						m_videoram[(0x4000c+(scrollbase<<4))/4]);
						/*m_videoram[(0x4001c+(scrollbase<<4))/4]);*/ // unused? (dupe value on fatfurwa, 00 on rest)
#endif


			xtopleft  = (m_videoram[(0x40000+(scrollbase<<4))/4]);
			xalt      = (m_videoram[(0x40004+(scrollbase<<4))/4]); // middle screen point
			xmiddle   = (m_videoram[(0x40010+(scrollbase<<4))/4]);

			ytopleft     = (m_videoram[(0x40008+(scrollbase<<4))/4]);
			yalt         = (m_videoram[(0x40018+(scrollbase<<4))/4]); // middle screen point
			ymiddle      = (m_videoram[(0x4000c+(scrollbase<<4))/4]);

			xinc = (xmiddle - xtopleft) / 512;
			yinc = (ymiddle - ytopleft) / 512;
			xinc2 = (xalt-xtopleft) / 512;
			yinc2 = (yalt-ytopleft) /512;


			/* manual copy = slooow */
			if (BLEND_TEST)
			{
				bitmap_ind16 &bm = tilemap->pixmap();
				int bmheight = bm.height();
				int bmwidth = bm.width();
				const pen_t *paldata = m_palette->pens();
				uint32_t* dstptr;
				uint16_t* srcptr;
				int xx,yy;


				int tmp = xtopleft;
				int tmp2 = ytopleft;
				//printf("start %08x end %08x start %08x end %08x\n", xtopleft, xmiddle, ytopleft, ymiddle);

				for (yy=0;yy<448;yy++)
				{
					dstptr = &bitmap.pix32(yy);

					tmp = xtopleft;
					tmp2 = ytopleft;

					for (xx=0;xx<512;xx++)
					{
						int realsrcx = (xtopleft>>16)&(bmwidth-1);
						int realsrcy = (ytopleft>>16)&(bmheight-1);
						uint16_t pen;

						srcptr = &bm.pix16(realsrcy);

						pen = srcptr[realsrcx];

						if (pen&transmask)
							*dstptr = paldata[pen];

						xtopleft+= xinc<<1;
						ytopleft+= yinc2<<1;
						++dstptr;
					}

					ytopleft = tmp2 + (yinc<<1);
					xtopleft = tmp + (xinc2<<1);
				}
			}
			else
			{
				hng64_tilemap_draw_roz(screen, bitmap,cliprect,tilemap,xtopleft,ytopleft,
						xinc<<1,yinc2<<1,xinc2<<1,yinc<<1,
						1,
						0,0, debug_blend_enabled?HNG64_TILEMAP_ADDITIVE:HNG64_TILEMAP_NORMAL);
			}

		}
		else
		{
			/* simple zoom mode? - only 4 regs? */
			/* in this mode they can only specify the top left and middle screen points for each tilemap,
			   this allows simple zooming, but not rotation */

			int32_t xtopleft,xmiddle;
			int32_t ytopleft,ymiddle;
			int xinc,yinc;

#if HNG64_VIDEO_DEBUG
			if (0)
				if (tm==2)
					popmessage("%08x %08x %08x %08x",
						m_videoram[(0x40010+(scrollbase<<4))/4],
						m_videoram[(0x40014+(scrollbase<<4))/4],
						m_videoram[(0x40018+(scrollbase<<4))/4],
						m_videoram[(0x4001c+(scrollbase<<4))/4]);
#endif

			if (global_zoom_disable) // disable all scrolling / zoom (test screen) (maybe)
			{
				/* If this bit is active the scroll registers don't seem valid at all?
				   It either disables zooming, or disables use of the scroll registers completely
				   - used at startup
				*/

				xtopleft = 0;
				xmiddle = 256<<16;

				ytopleft = 0;
				ymiddle = 256<<16;
			}
			else
			{
				xtopleft = (m_videoram[(0x40000+(scrollbase<<4))/4]);
				xmiddle   = (m_videoram[(0x40004+(scrollbase<<4))/4]); // middle screen point
				ytopleft = (m_videoram[(0x40008+(scrollbase<<4))/4]);
				ymiddle   = (m_videoram[(0x4000c+(scrollbase<<4))/4]); // middle screen point
			}

			xinc = (xmiddle - xtopleft) / 512;
			yinc = (ymiddle - ytopleft) / 512;

			/* manual copy = slooow */
			if (BLEND_TEST)
			{
				bitmap_ind16 &bm = tilemap->pixmap();
				int bmheight = bm.height();
				int bmwidth = bm.width();
				const pen_t *paldata = m_palette->pens();
				uint32_t* dstptr;
				uint16_t* srcptr;
				int xx,yy;

				int tmp = xtopleft;

				//printf("start %08x end %08x start %08x end %08x\n", xtopleft, xmiddle, ytopleft, ymiddle);

				for (yy=0;yy<448;yy++)
				{
					int realsrcy = (ytopleft>>16)&(bmheight-1);

					dstptr = &bitmap.pix32(yy);
					srcptr = &bm.pix16(realsrcy);

					xtopleft = tmp;

					for (xx=0;xx<512;xx++)
					{
						int realsrcx = (xtopleft>>16)&(bmwidth-1);

						uint16_t pen;

						pen = srcptr[realsrcx];

						if (pen&transmask)
							*dstptr = paldata[pen];

						xtopleft+= xinc<<1;
						++dstptr;
					}

					ytopleft+= yinc<<1;
				}
			}
			else
			{
				hng64_tilemap_draw_roz(screen, bitmap,cliprect,tilemap,xtopleft,ytopleft,
						xinc<<1,0,0,yinc<<1,
						1,
						0,0, debug_blend_enabled?HNG64_TILEMAP_ADDITIVE:HNG64_TILEMAP_NORMAL);
			}
		}
	}
}


uint32_t hng64_state::screen_update_hng64(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
#if 1
	// press in sams64_2 attract mode for a nice debug screen from the game
	// not sure how functional it is, and it doesn't appear to test everything (rowscroll modes etc.)
	// but it could be useful
	if ( machine().input().code_pressed_once(KEYCODE_L) )
	{
		address_space &space = m_maincpu->space(AS_PROGRAM);

		if (!strcmp(machine().system().name, "sams64_2"))
		{
			space.write_byte(0x2f27c8, 0x2);
		}
		else if (!strcmp(machine().system().name, "roadedge")) // hack to get test mode (useful for sound test)
		{
			space.write_byte(0xcfb53, 0x1);
		}
		else if (!strcmp(machine().system().name, "xrally")) // hack to get test mode (useful for sound test)
		{
			space.write_byte(0xa2363, 0x1);
		}

	}
#endif


	// Initialize some buffers
	bitmap.fill(m_tcram[0x50/4] & 0x10000 ? m_palette->black_pen() : m_palette->pen(0), cliprect); //FIXME: Is the register correct? check with HW tests
	screen.priority().fill(0x00, cliprect);

	// If the screen is disabled, don't draw anything (m_screen_dis is a shady variable at best)
	if (m_screen_dis)
		return 0;

	// If the auto-animation mask or bits have changed search for tiles using them and mark as dirty
	const uint32_t animmask = m_videoregs[0x0b];
	const uint32_t animbits = m_videoregs[0x0c];
	if ((m_old_animmask != animmask) || (m_old_animbits != animbits))
	{
		int tile_index;
		for (tile_index=0;tile_index<128*128;tile_index++)
		{
			if (m_videoram[tile_index+(0x00000/4)] & 0x200000)
			{
				hng64_mark_tile_dirty(0, tile_index);
			}
			if (m_videoram[tile_index+(0x10000/4)] & 0x200000)
			{
				hng64_mark_tile_dirty(1, tile_index);
			}
			if (m_videoram[tile_index+(0x20000/4)] & 0x200000)
			{
				hng64_mark_tile_dirty(2, tile_index);
			}
			if (m_videoram[tile_index+(0x30000/4)] & 0x200000)
			{
				hng64_mark_tile_dirty(3, tile_index);
			}
		}

		m_old_animmask = animmask;
		m_old_animbits = animbits;
	}

	// If any magic bits have been touched, mark every tilemap dirty
	uint16_t tileflags[4];
	tileflags[0] = m_videoregs[0x02] >> 16;
	tileflags[1] = m_videoregs[0x02] & 0xffff;
	tileflags[2] = m_videoregs[0x03] >> 16;
	tileflags[3] = m_videoregs[0x03] & 0xffff;
	const uint16_t IMPORTANT_DIRTY_TILEFLAG_MASK = 0x0600;
	for (int i = 0; i < 4; i++)
	{
		if ((m_old_tileflags[i] & IMPORTANT_DIRTY_TILEFLAG_MASK) != (tileflags[i] & IMPORTANT_DIRTY_TILEFLAG_MASK))
		{
			hng64_mark_all_tiles_dirty(i);
			m_old_tileflags[i] = tileflags[i];
		}
	}

	// Draw the four tilemaps
	hng64_drawtilemap(screen,bitmap,cliprect, 3);
	hng64_drawtilemap(screen,bitmap,cliprect, 2);
	hng64_drawtilemap(screen,bitmap,cliprect, 1);
	hng64_drawtilemap(screen,bitmap,cliprect, 0);

	// 3d gets drawn next
	if(!(m_3dregs[0] & 0x1000000))
	{
		int x, y;

		// Blit the color buffer into the primary bitmap
		for (y = cliprect.min_y; y <= cliprect.max_y; y++)
		{
			uint32_t *src = &m_poly_renderer->colorBuffer3d().pix32(y, cliprect.min_x);
			uint32_t *dst = &bitmap.pix32(y, cliprect.min_x);

			for (x = cliprect.min_x; x <= cliprect.max_x; x++)
			{
				if(*src & 0xff000000)
					*dst = *src;

				dst++;
				src++;
			}
		}
	}

	// Draw the sprites on top of everything
	draw_sprites(screen, bitmap, cliprect);

	// Layer the global frame buffer operations on top of everything
	// transition_control(bitmap, cliprect);


#if HNG64_VIDEO_DEBUG
	if (0)
		popmessage("%08x %08x %08x %08x %08x", m_spriteregs[0], m_spriteregs[1], m_spriteregs[2], m_spriteregs[3], m_spriteregs[4]);

	if (1)
	popmessage("%08x %08x TR(%04x %04x %04x %04x) SB(%04x %04x %04x %04x) %08x %08x %08x %08x %08x AA(%08x %08x) %08x",
		m_videoregs[0x00],
		m_videoregs[0x01],
		(m_videoregs[0x02]>>16)&0xffff,
		(m_videoregs[0x02]>>0)&0xffff,  //       ss64_2 debug mode indicates that 0x0040 is enable!
		(m_videoregs[0x03]>>16)&0xffff, //       buriki agrees (debug data on text layer) xrally agress (pink layer)
		(m_videoregs[0x03]>>0)&0xffff,  //       fatal fury doesn't (all backgrounds have it set) joy
		(m_videoregs[0x04]>>16)&0xffff,
		(m_videoregs[0x04]>>0)&0xffff,
		(m_videoregs[0x05]>>16)&0xffff,
		(m_videoregs[0x05]>>0)&0xffff,
		m_videoregs[0x06],
		m_videoregs[0x07],
		m_videoregs[0x08],
		m_videoregs[0x09],
		m_videoregs[0x0a],
		m_videoregs[0x0b],
		m_videoregs[0x0c],
		m_videoregs[0x0d]);

	if (0)
	popmessage("3D: %08x %08x %08x %08x : %08x %08x %08x %08x : %08x %08x %08x %08x",
		m_3dregs[0x00/4], m_3dregs[0x04/4], m_3dregs[0x08/4], m_3dregs[0x0c/4],
		m_3dregs[0x10/4], m_3dregs[0x14/4], m_3dregs[0x18/4], m_3dregs[0x1c/4],
		m_3dregs[0x20/4], m_3dregs[0x24/4], m_3dregs[0x28/4], m_3dregs[0x2c/4]);

	if (0)
		popmessage("TC: %08x %08x %08x %08x : %08x %08x %08x %08x : %08x %08x %08x %08x : %08x %08x %08x %08x : %08x %08x %08x %08x : %08x %08x %08x %08x",
		m_tcram[0x00/4],
		m_tcram[0x04/4],
		m_tcram[0x08/4], // tilemaps 0/1 ?
		m_tcram[0x0c/4], // ss64_2 debug 04000000 = 'half' on tm1   00000004 = 'half' on tm3  (used in transitions?)
		m_tcram[0x10/4],
		m_tcram[0x14/4],
		m_tcram[0x18/4],
		m_tcram[0x1c/4],
		m_tcram[0x20/4],
		m_tcram[0x24/4],
		m_tcram[0x28/4],
		m_tcram[0x2c/4],
		m_tcram[0x30/4],
		m_tcram[0x34/4],
		m_tcram[0x38/4],
		m_tcram[0x3c/4],
		m_tcram[0x40/4],
		m_tcram[0x44/4],
		m_tcram[0x48/4],
		m_tcram[0x4c/4],
		m_tcram[0x50/4],
		m_tcram[0x54/4],
		m_tcram[0x58/4],
		m_tcram[0x5c/4]);

	if ( machine().input().code_pressed_once(KEYCODE_T) )
	{
		m_additive_tilemap_debug ^= 1;
		popmessage("blend changed %02x", m_additive_tilemap_debug);
	}
	if ( machine().input().code_pressed_once(KEYCODE_Y) )
	{
		m_additive_tilemap_debug ^= 2;
		popmessage("blend changed %02x", m_additive_tilemap_debug);
	}
	if ( machine().input().code_pressed_once(KEYCODE_U) )
	{
		m_additive_tilemap_debug ^= 4;
		popmessage("blend changed %02x", m_additive_tilemap_debug);
	}
	if ( machine().input().code_pressed_once(KEYCODE_I) )
	{
		m_additive_tilemap_debug ^= 8;
		popmessage("blend changed %02x", m_additive_tilemap_debug);
	}
#endif

	return 0;
}

WRITE_LINE_MEMBER(hng64_state::screen_vblank_hng64)
{
	// rising edge
	if (state)
		clear3d();
}


/* Transition Control Video Registers
 * ----------------------------------
 *
 * uint32_t | Bits                                    | Use
 *        | 3322 2222 2222 1111 1111 11             |
 * -------+-1098-7654-3210-9876-5432-1098-7654-3210-+----------------
 *      0 |                                         |
 *      1 | xxxx xxxx xxxx xxxx yyyy yyyy yyyy yyyy | Min X / Min Y visible area rectangle values
 *      2 | xxxx xxxx xxxx xxxx yyyy yyyy yyyy yyyy | Max X / Max Y visible area rectangle values (added up with the Min X / Min Y)
 *      3 |                                         |
 *      4 |                                         |
 *      5 | ---- ---- ---- ---? ---- --?? ???? ???? | Global Fade In/Fade Out control
 *      6 |                                         |
 *      7 | ---- ---- xxxx xxxx xxxx xxxx xxxx xxxx | Port A of RGB fade (subtraction)
 *      8 |                                         |
 *      9 | ---- ---- ---- ---? ---- ---- ---- ???? | Per-layer Fade In/Fade Out control
 *     10 | ---- ---- xxxx xxxx xxxx xxxx xxxx xxxx | Port B of RGB fade (additive)
 *     11 | xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx | Unknown - looks like an ARGB value - it seems to change when the scene changes
 *     12 |                                         |
 *     13 |                                         |
 *     14 |                                         |
 *     15 |                                         |
 *     16 |                                         |
 *     17 |                                         |
 *     18 | xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx | V-Blank related stuff
 *     19 |                                         |
 *     20 | ---- ---- ---- ---x ---- ---- ---- ---- | Back layer control register?
 *     21 |                                         |
 *     22 |                                         |
 *     23 |                                         |
 *     24 |                                         |
 *
 *  Various bits change depending on what is happening in the scene.
 *  These bits may set which 'layer' is affected by the blending.
 *  Or maybe they adjust the scale of the lightening and darkening...
 *  Or maybe it switches from fading by scaling to fading using absolute addition and subtraction...
 *  Or maybe they set transition type (there seems to be a cute scaling-squares transition in there somewhere)...
 */

// Very much a work in progress - no hard testing has been done
void hng64_state::transition_control( bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	int i, j;

//  float colorScaleR, colorScaleG, colorScaleB;
	int32_t finR, finG, finB;

	int32_t darkR, darkG, darkB;
	int32_t brigR, brigG, brigB;

	// If either of the fading memory regions is non-zero...
	if (m_tcram[0x00000007] != 0x00000000 || m_tcram[0x0000000a] != 0x00000000)
	{
		darkR = (int32_t)( m_tcram[0x00000007]        & 0xff);
		darkG = (int32_t)((m_tcram[0x00000007] >> 8)  & 0xff);
		darkB = (int32_t)((m_tcram[0x00000007] >> 16) & 0xff);

		brigR = (int32_t)( m_tcram[0x0000000a]        & 0xff);
		brigG = (int32_t)((m_tcram[0x0000000a] >> 8)  & 0xff);
		brigB = (int32_t)((m_tcram[0x0000000a] >> 16) & 0xff);

		for (i = cliprect.min_x; i < cliprect.max_x; i++)
		{
			for (j = cliprect.min_y; j < cliprect.max_y; j++)
			{
				rgb_t* thePixel = reinterpret_cast<rgb_t *>(&bitmap.pix32(j, i));

				finR = (int32_t)thePixel->r();
				finG = (int32_t)thePixel->g();
				finB = (int32_t)thePixel->b();

#if 0
				// Apply the darkening pass (0x07)...
				colorScaleR = 1.0f - (float)( m_tcram[0x00000007] & 0xff)        / 255.0f;
				colorScaleG = 1.0f - (float)((m_tcram[0x00000007] >> 8)  & 0xff) / 255.0f;
				colorScaleB = 1.0f - (float)((m_tcram[0x00000007] >> 16) & 0xff) / 255.0f;

				finR = ((float)thePixel->r()   * colorScaleR);
				finG = ((float)thePixel->g() * colorScaleG);
				finB = ((float)thePixel->b()  * colorScaleB);


				// Apply the lightening pass (0x0a)...
				colorScaleR = 1.0f + (float)( m_tcram[0x0000000a] & 0xff)        / 255.0f;
				colorScaleG = 1.0f + (float)((m_tcram[0x0000000a] >> 8)  & 0xff) / 255.0f;
				colorScaleB = 1.0f + (float)((m_tcram[0x0000000a] >> 16) & 0xff) / 255.0f;

				finR *= colorScaleR;
				finG *= colorScaleG;
				finB *= colorScaleB;


				// Clamp
				if (finR > 255.0f) finR = 255.0f;
				if (finG > 255.0f) finG = 255.0f;
				if (finB > 255.0f) finB = 255.0f;
#endif


				// Subtractive fading
				if (m_tcram[0x00000007] != 0x00000000)
				{
					finR -= darkR;
					finG -= darkG;
					finB -= darkB;
				}

				// Additive fading
				if (m_tcram[0x0000000a] != 0x00000000)
				{
					finR += brigR;
					finG += brigG;
					finB += brigB;
				}

				// Clamp the high end
				if (finR > 255) finR = 255;
				if (finG > 255) finG = 255;
				if (finB > 255) finB = 255;

				// Clamp the low end
				if (finR < 0) finR = 0;
				if (finG < 0) finG = 0;
				if (finB < 0) finB = 0;

				*thePixel = rgb_t(255, (uint8_t)finR, (uint8_t)finG, (uint8_t)finB);
			}
		}
	}
}

void hng64_state::video_start()
{
	m_old_animmask = -1;
	m_old_animbits = -1;
	m_old_tileflags[0] = -1;
	m_old_tileflags[1] = -1;
	m_old_tileflags[2] = -1;
	m_old_tileflags[3] = -1;

	m_tilemap[0].m_tilemap_8x8       = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(hng64_state::get_hng64_tile0_8x8_info),this),   TILEMAP_SCAN_ROWS,  8,   8, 128, 128); /* 128x128x4 = 0x10000 */
	m_tilemap[0].m_tilemap_16x16     = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(hng64_state::get_hng64_tile0_16x16_info),this), TILEMAP_SCAN_ROWS,  16, 16, 128, 128); /* 128x128x4 = 0x10000 */
	m_tilemap[0].m_tilemap_16x16_alt = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(hng64_state::get_hng64_tile0_16x16_info),this), TILEMAP_SCAN_ROWS,  16, 16, 256,  64); /* 128x128x4 = 0x10000 */

	m_tilemap[1].m_tilemap_8x8       = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(hng64_state::get_hng64_tile1_8x8_info),this),   TILEMAP_SCAN_ROWS,  8,   8, 128, 128); /* 128x128x4 = 0x10000 */
	m_tilemap[1].m_tilemap_16x16     = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(hng64_state::get_hng64_tile1_16x16_info),this), TILEMAP_SCAN_ROWS,  16, 16, 128, 128); /* 128x128x4 = 0x10000 */
	m_tilemap[1].m_tilemap_16x16_alt = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(hng64_state::get_hng64_tile1_16x16_info),this), TILEMAP_SCAN_ROWS,  16, 16, 256,  64); /* 128x128x4 = 0x10000 */

	m_tilemap[2].m_tilemap_8x8       = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(hng64_state::get_hng64_tile2_8x8_info),this),   TILEMAP_SCAN_ROWS,  8,   8, 128, 128); /* 128x128x4 = 0x10000 */
	m_tilemap[2].m_tilemap_16x16     = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(hng64_state::get_hng64_tile2_16x16_info),this), TILEMAP_SCAN_ROWS,  16, 16, 128, 128); /* 128x128x4 = 0x10000 */
	m_tilemap[2].m_tilemap_16x16_alt = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(hng64_state::get_hng64_tile2_16x16_info),this), TILEMAP_SCAN_ROWS,  16, 16, 256,  64); /* 128x128x4 = 0x10000 */

	m_tilemap[3].m_tilemap_8x8       = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(hng64_state::get_hng64_tile3_8x8_info),this),   TILEMAP_SCAN_ROWS,  8,   8, 128, 128); /* 128x128x4 = 0x10000 */
	m_tilemap[3].m_tilemap_16x16     = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(hng64_state::get_hng64_tile3_16x16_info),this), TILEMAP_SCAN_ROWS,  16, 16, 128, 128); /* 128x128x4 = 0x10000 */
	m_tilemap[3].m_tilemap_16x16_alt = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(hng64_state::get_hng64_tile3_16x16_info),this), TILEMAP_SCAN_ROWS,  16, 16, 256,  64); /* 128x128x4 = 0x10000 */

	for (auto & elem : m_tilemap)
	{
		elem.m_tilemap_8x8->set_transparent_pen(0);
		elem.m_tilemap_16x16->set_transparent_pen(0);
		elem.m_tilemap_16x16_alt->set_transparent_pen(0);
	}

	// Debug switch, turn on / off additive blending on a per-tilemap basis
	m_additive_tilemap_debug = 0;

	// Rasterizer creation
	m_poly_renderer = std::make_unique<hng64_poly_renderer>(*this);

	// 3d information
	m_dl = std::make_unique<uint16_t[]>(0x100);
	m_polys.resize(HNG64_MAX_POLYGONS);

	m_texturerom = memregion("textures")->base();
	m_vertsrom = (uint16_t*)memregion("verts")->base();
	m_vertsrom_size = memregion("verts")->bytes();
}

#include "video/hng64_3d.hxx"
#include "video/hng64_sprite.hxx"
