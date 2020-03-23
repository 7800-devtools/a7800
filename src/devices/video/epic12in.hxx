// license:BSD-3-Clause
// copyright-holders:David Haywood
/* blitter function */

void epic12_device::FUNCNAME(BLIT_PARAMS)
{
	uint32_t* gfx2;
	int y, yf;

#if REALLY_SIMPLE == 0
	colour_t s_clr;
#endif

#if BLENDED == 1
	colour_t d_clr;

#if _SMODE == 2
#if _DMODE != 0
	colour_t clr0;
#endif
#elif _SMODE == 0
#if _DMODE != 0
#if _DMODE != 5
#if _DMODE != 1
	colour_t clr0;
#endif
#endif
#endif
#else
	colour_t clr0;
#endif


#endif

#if REALLY_SIMPLE == 1
#if TRANSPARENT == 1
	uint32_t pen;
#endif
#else
	uint32_t pen;
#endif
	uint32_t *bmp;

#if FLIPX == 1
	src_x += (dimx-1);
#endif

	if (flipy)  {   yf = -1;    src_y += (dimy-1);  }
	else        {   yf = +1;                        }

	int starty = 0;
	const int dst_y_end = dst_y_start+dimy;

	if (dst_y_start < clip->min_y)
		starty = clip->min_y - dst_y_start;

	if (dst_y_end > clip->max_y)
		dimy -= (dst_y_end-1) - clip->max_y;

	// check things are safe to draw (note, if the source would wrap round an edge of the 0x2000*0x1000 vram we don't draw.. not sure what the hw does anyway)
	// ddpdfk triggers this on boss explosions so it needs fixing
#if FLIPX == 1
	if ((src_x &0x1fff) < ((src_x-(dimx-1))&0x1fff))
	{
	//  popmessage("sprite gets clipped off src_x %04x dimx %04x\n", src_x, dimx);
		return;
	}
#else
	if ((src_x &0x1fff) > ((src_x+(dimx-1))&0x1fff))
	{
	//  popmessage("sprite gets clipped off src_x %04x dimx %04x\n", src_x, dimx);
		return;
	}
#endif

	int startx = 0;
	const int dst_x_end = dst_x_start+dimx;

	if (dst_x_start < clip->min_x)
		startx = clip->min_x - dst_x_start;

	if (dst_x_end > clip->max_x)
		dimx -= (dst_x_end-1) - clip->max_x;

// wrong/unsafe slowdown sim
	if (dimy > starty && dimx > startx)
	{
		blit_delay += (dimy - starty)*(dimx - startx);

		//printf("delay is now %d\n", blit_delay);
	}

#if BLENDED == 1
#if _SMODE == 0
#if _DMODE == 0
	const uint8_t* salpha_table = colrtable[s_alpha];
	const uint8_t* dalpha_table = colrtable[d_alpha];
#endif

#if _DMODE == 5
	const uint8_t* salpha_table = colrtable[s_alpha];
#endif
#if _DMODE == 1
	const uint8_t* salpha_table = colrtable[s_alpha];
#endif

#endif

#if _SMODE == 2
#if _DMODE == 0

	const uint8_t* dalpha_table = colrtable[d_alpha];
#endif
#endif
#endif



	for (y = starty; y < dimy; y++)
	{
		bmp = &bitmap->pix(dst_y_start + y, dst_x_start+startx);
		const int ysrc_index =  ((src_y + yf * y) & 0x0fff) * 0x2000;
		gfx2 = gfx + ysrc_index;

		#if FLIPX == 1
			gfx2 += (src_x-startx);
		#else
			gfx2 += (src_x+startx);
		#endif

#if 1
		const uint32_t* end = bmp+(dimx-startx);
#else
		// maybe we can do some SSE type optimizations on larger blocks? right now this just results in more code and slower compiling tho.

		const int width = dimx-startx;
		const uint32_t* end = bmp+(width);

		if (width<0) return;

		int bigblocks = width>>3;

		while (bigblocks)
		{
			#include "epic12pixel.hxx"
			#include "epic12pixel.hxx"
			#include "epic12pixel.hxx"
			#include "epic12pixel.hxx"
			#include "epic12pixel.hxx"
			#include "epic12pixel.hxx"
			#include "epic12pixel.hxx"
			#include "epic12pixel.hxx"

			bigblocks--;
		}
#endif
		while (bmp<end)
		{
			#include "epic12pixel.hxx"
		}

	}

//  g_profiler.stop();
}

#undef LOOP_INCREMENTS
