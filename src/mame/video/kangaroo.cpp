// license:BSD-3-Clause
// copyright-holders:Ville Laitinen, Aaron Giles
/***************************************************************************

    Sun Electronics Kangaroo hardware

***************************************************************************/

#include "emu.h"
#include "includes/kangaroo.h"

/*************************************
 *
 *  Video setup
 *
 *************************************/

void kangaroo_state::video_start()
{
	/* video RAM is accessed 32 bits at a time (two planes, 4bpp each, 4 pixels) */
	m_videoram = std::make_unique<uint32_t[]>(256 * 64);
	save_pointer(NAME(m_videoram.get()), 256 * 64);
}



/*************************************
 *
 *  Video RAM accesses
 *
 *************************************/

void kangaroo_state::videoram_write( uint16_t offset, uint8_t data, uint8_t mask )
{
	uint32_t expdata, layermask;

	/* data contains 4 2-bit values packed as DCBADCBA; expand these into 4 8-bit values */
	expdata = 0;
	if (data & 0x01) expdata |= 0x00000055;
	if (data & 0x10) expdata |= 0x000000aa;
	if (data & 0x02) expdata |= 0x00005500;
	if (data & 0x20) expdata |= 0x0000aa00;
	if (data & 0x04) expdata |= 0x00550000;
	if (data & 0x40) expdata |= 0x00aa0000;
	if (data & 0x08) expdata |= 0x55000000;
	if (data & 0x80) expdata |= 0xaa000000;

	/* determine which layers are enabled */
	layermask = 0;
	if (mask & 0x08) layermask |= 0x30303030;
	if (mask & 0x04) layermask |= 0xc0c0c0c0;
	if (mask & 0x02) layermask |= 0x03030303;
	if (mask & 0x01) layermask |= 0x0c0c0c0c;

	/* update layers */
	m_videoram[offset] = (m_videoram[offset] & ~layermask) | (expdata & layermask);
}


WRITE8_MEMBER(kangaroo_state::kangaroo_videoram_w)
{
	videoram_write(offset, data, m_video_control[8]);
}



/*************************************
 *
 *  Video control writes
 *
 *************************************/

WRITE8_MEMBER(kangaroo_state::kangaroo_video_control_w)
{
	m_video_control[offset] = data;

	switch (offset)
	{
		case 5: /* blitter start */
			blitter_execute();
			break;

		case 8: /* bank select */
			membank("bank1")->set_entry((data & 0x05) ? 0 : 1);
			break;
	}
}



/*************************************
 *
 *  DMA blitter
 *
 *************************************/

void kangaroo_state::blitter_execute(  )
{
	uint32_t gfxhalfsize = memregion("gfx1")->bytes() / 2;
	const uint8_t *gfxbase = memregion("gfx1")->base();
	uint16_t src = m_video_control[0] + 256 * m_video_control[1];
	uint16_t dst = m_video_control[2] + 256 * m_video_control[3];
	uint8_t height = m_video_control[5];
	uint8_t width = m_video_control[4];
	uint8_t mask = m_video_control[8];
	int x, y;

	/* during DMA operations, the top 2 bits are ORed together, as well as the bottom 2 bits */
	/* adjust the mask to account for this */
	if (mask & 0x0c) mask |= 0x0c;
	if (mask & 0x03) mask |= 0x03;

	/* loop over height, then width */
	for (y = 0; y <= height; y++, dst += 256)
		for (x = 0; x <= width; x++)
		{
			uint16_t effdst = (dst + x) & 0x3fff;
			uint16_t effsrc = src++ & (gfxhalfsize - 1);
			videoram_write(effdst, gfxbase[0 * gfxhalfsize + effsrc], mask & 0x05);
			videoram_write(effdst, gfxbase[1 * gfxhalfsize + effsrc], mask & 0x0a);
		}
}



/*************************************
 *
 *  Video updater
 *
 *************************************/

uint32_t kangaroo_state::screen_update_kangaroo(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	uint8_t scrolly = m_video_control[6];
	uint8_t scrollx = m_video_control[7];
	uint8_t maska = (m_video_control[10] & 0x28) >> 3;
	uint8_t maskb = (m_video_control[10] & 0x07) >> 0;
	uint8_t xora = (m_video_control[9] & 0x20) ? 0xff : 0x00;
	uint8_t xorb = (m_video_control[9] & 0x10) ? 0xff : 0x00;
	uint8_t enaa = (m_video_control[9] & 0x08);
	uint8_t enab = (m_video_control[9] & 0x04);
	uint8_t pria = (~m_video_control[9] & 0x02);
	uint8_t prib = (~m_video_control[9] & 0x01);
	int x, y;

	/* iterate over pixels */
	for (y = cliprect.min_y; y <= cliprect.max_y; y++)
	{
		uint32_t *dest = &bitmap.pix32(y);

		for (x = cliprect.min_x; x <= cliprect.max_x; x += 2)
		{
			uint8_t effxa = scrollx + ((x / 2) ^ xora);
			uint8_t effya = scrolly + (y ^ xora);
			uint8_t effxb = (x / 2) ^ xorb;
			uint8_t effyb = y ^ xorb;
			uint8_t pixa = (m_videoram[effya + 256 * (effxa / 4)] >> (8 * (effxa % 4) + 0)) & 0x0f;
			uint8_t pixb = (m_videoram[effyb + 256 * (effxb / 4)] >> (8 * (effxb % 4) + 4)) & 0x0f;
			uint8_t finalpens;

			/* for each layer, contribute bits if (a) enabled, and (b) either has priority or the opposite plane is 0 */
			finalpens = 0;
			if (enaa && (pria || pixb == 0))
				finalpens |= pixa;
			if (enab && (prib || pixa == 0))
				finalpens |= pixb;

			/* store the first of two pixels, which is always full brightness */
			dest[x + 0] = m_palette->pen_color(finalpens & 7);

			/* KOS1 alternates at 5MHz, offset from the pixel clock by 1/2 clock */
			/* when 0, it enables the color mask for pixels with Z = 0 */
			finalpens = 0;
			if (enaa && (pria || pixb == 0))
			{
				if (!(pixa & 0x08)) pixa &= maska;
				finalpens |= pixa;
			}
			if (enab && (prib || pixa == 0))
			{
				if (!(pixb & 0x08)) pixb &= maskb;
				finalpens |= pixb;
			}

			/* store the second of two pixels, which is affected by KOS1 and the A/B masks */
			dest[x + 1] = m_palette->pen_color(finalpens & 7);
		}
	}

	return 0;
}
