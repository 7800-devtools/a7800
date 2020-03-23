// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    Atari Jaguar object processor

****************************************************************************/

#ifndef INCLUDE_OBJECT_PROCESSOR
#error jagobj.c is not directly compilable!
#endif


#define LOG_OBJECTS         0


/*************************************
 *
 *  Object processor init
 *
 *************************************/

void jaguar_state::jagobj_init()
{
	int i;

	/* fill tables */
	for (i = 0; i < 256 * 256; i++)
	{
		int y = (i >> 8) & 0xff;
		int dy = (int8_t)i;
		int c1 = (i >> 8) & 0x0f;
		int dc1 = (int8_t)(i << 4) >> 4;
		int c2 = (i >> 12) & 0x0f;
		int dc2 = (int8_t)(i & 0xf0) >> 4;

		y += dy;
		if (y < 0) y = 0;
		else if (y > 0xff) y = 0xff;
		m_blend_y[i] = y;

		c1 += dc1;
		if (c1 < 0) c1 = 0;
		else if (c1 > 0x0f) c1 = 0x0f;
		c2 += dc2;
		if (c2 < 0) c2 = 0;
		else if (c2 > 0x0f) c2 = 0x0f;
		m_blend_cc[i] = (c2 << 4) | c1;
	}
}



/*************************************
 *
 *  Blending function
 *
 *************************************/

#define BLEND(dst, src)     \
	(dst) = (m_blend_cc[((dst) & 0xff00) | (((src) >> 8) & 0xff)] << 8) | m_blend_y[(((dst) & 0xff) << 8) | ((src) & 0xff)];



/*************************************
 *
 *  4bpp bitmap renderers
 *
 *************************************/

inline void jaguar_state::bitmap_4_draw(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint8_t flags, int32_t dxpos, uint16_t *clutbase)
{
	if (firstpix & 7)
	{
		uint32_t pixsrc = src[firstpix >> 3];
		while (firstpix & 7)
		{
			int pix = (pixsrc >> ((~firstpix & 7) << 2)) & 0x0f;
			if ((!(flags & 4) || pix) && (uint32_t)xpos < 760)
			{
				if (!(flags & 2))
					scanline[xpos] = clutbase[BYTE_XOR_BE(pix)];
				else
					BLEND(scanline[xpos], clutbase[BYTE_XOR_BE(pix)]);
			}
			xpos += dxpos;
			firstpix++;
		}
	}

	firstpix >>= 3;
	iwidth >>= 3;
	iwidth -= firstpix;

	while (iwidth-- > 0)
	{
		uint32_t pix = src[firstpix++];
		if (!(flags & 4) || pix)
		{
			if ((!(flags & 4) || (pix & 0xf0000000)) && (uint32_t)xpos < 760)
			{
				if (!(flags & 2))
					scanline[xpos] = clutbase[BYTE_XOR_BE(pix >> 28)];
				else
					BLEND(scanline[xpos], clutbase[BYTE_XOR_BE(pix >> 28)]);
			}
			xpos += dxpos;

			if ((!(flags & 4) || (pix & 0x0f000000)) && (uint32_t)xpos < 760)
			{
				if (!(flags & 2))
					scanline[xpos] = clutbase[BYTE_XOR_BE((pix >> 24) & 0x0f)];
				else
					BLEND(scanline[xpos], clutbase[BYTE_XOR_BE((pix >> 24) & 0x0f)]);
			}
			xpos += dxpos;

			if ((!(flags & 4) || (pix & 0x00f00000)) && (uint32_t)xpos < 760)
			{
				if (!(flags & 2))
					scanline[xpos] = clutbase[BYTE_XOR_BE((pix >> 20) & 0x0f)];
				else
					BLEND(scanline[xpos], clutbase[BYTE_XOR_BE((pix >> 20) & 0x0f)]);
			}
			xpos += dxpos;

			if ((!(flags & 4) || (pix & 0x000f0000)) && (uint32_t)xpos < 760)
			{
				if (!(flags & 2))
					scanline[xpos] = clutbase[BYTE_XOR_BE((pix >> 16) & 0x0f)];
				else
					BLEND(scanline[xpos], clutbase[BYTE_XOR_BE((pix >> 16) & 0x0f)]);
			}
			xpos += dxpos;

			if ((!(flags & 4) || (pix & 0x0000f000)) && (uint32_t)xpos < 760)
			{
				if (!(flags & 2))
					scanline[xpos] = clutbase[BYTE_XOR_BE((pix >> 12) & 0x0f)];
				else
					BLEND(scanline[xpos], clutbase[BYTE_XOR_BE((pix >> 12) & 0x0f)]);
			}
			xpos += dxpos;

			if ((!(flags & 4) || (pix & 0x00000f00)) && (uint32_t)xpos < 760)
			{
				if (!(flags & 2))
					scanline[xpos] = clutbase[BYTE_XOR_BE((pix >> 8) & 0x0f)];
				else
					BLEND(scanline[xpos], clutbase[BYTE_XOR_BE((pix >> 8) & 0x0f)]);
			}
			xpos += dxpos;

			if ((!(flags & 4) || (pix & 0x000000f0)) && (uint32_t)xpos < 760)
			{
				if (!(flags & 2))
					scanline[xpos] = clutbase[BYTE_XOR_BE((pix >> 4) & 0x0f)];
				else
					BLEND(scanline[xpos], clutbase[BYTE_XOR_BE((pix >> 4) & 0x0f)]);
			}
			xpos += dxpos;

			if ((!(flags & 4) || (pix & 0x0000000f)) && (uint32_t)xpos < 760)
			{
				if (!(flags & 2))
					scanline[xpos] = clutbase[BYTE_XOR_BE(pix & 0x0f)];
				else
					BLEND(scanline[xpos], clutbase[BYTE_XOR_BE(pix & 0x0f)]);
			}
			xpos += dxpos;
		}
		else
			xpos += dxpos << 3;
	}
}

void jaguar_state::bitmap_4_0(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint16_t *clutbase)
{
	bitmap_4_draw(scanline, firstpix, iwidth, src, xpos, 0, 1, clutbase);
}

void jaguar_state::bitmap_4_1(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint16_t *clutbase)
{
	bitmap_4_draw(scanline, firstpix, iwidth, src, xpos, 1, -1, clutbase);
}

void jaguar_state::bitmap_4_2(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint16_t *clutbase)
{
	bitmap_4_draw(scanline, firstpix, iwidth, src, xpos, 2, 1, clutbase);
}

void jaguar_state::bitmap_4_3(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint16_t *clutbase)
{
	bitmap_4_draw(scanline, firstpix, iwidth, src, xpos, 3, -1, clutbase);
}

void jaguar_state::bitmap_4_4(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint16_t *clutbase)
{
	bitmap_4_draw(scanline, firstpix, iwidth, src, xpos, 4, 1, clutbase);
}

void jaguar_state::bitmap_4_5(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint16_t *clutbase)
{
	bitmap_4_draw(scanline, firstpix, iwidth, src, xpos, 5, -1, clutbase);
}

void jaguar_state::bitmap_4_6(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint16_t *clutbase)
{
	bitmap_4_draw(scanline, firstpix, iwidth, src, xpos, 6, 1, clutbase);
}

void jaguar_state::bitmap_4_7(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint16_t *clutbase)
{
	bitmap_4_draw(scanline, firstpix, iwidth, src, xpos, 7, -1, clutbase);
}

void (jaguar_state::*const jaguar_state::bitmap4[8])(uint16_t *, int32_t, int32_t, uint32_t *, int32_t, uint16_t *) =
{
	&jaguar_state::bitmap_4_0,
	&jaguar_state::bitmap_4_1,
	&jaguar_state::bitmap_4_2,
	&jaguar_state::bitmap_4_3,
	&jaguar_state::bitmap_4_4,
	&jaguar_state::bitmap_4_5,
	&jaguar_state::bitmap_4_6,
	&jaguar_state::bitmap_4_7
};



/*************************************
 *
 *  8bpp bitmap renderers
 *
 *************************************/

inline void jaguar_state::bitmap_8_draw(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint8_t flags, int32_t dxpos, uint16_t *clutbase)
{
	if (firstpix & 3)
	{
		uint32_t pixsrc = src[firstpix >> 2];
		while (firstpix & 3)
		{
			uint8_t pix = pixsrc >> ((~firstpix & 3) << 3);
			if ((!(flags & 4) || pix) && (uint32_t)xpos < 760)
			{
				if (!(flags & 2))
					scanline[xpos] = clutbase[BYTE_XOR_BE(pix)];
				else
					BLEND(scanline[xpos], clutbase[BYTE_XOR_BE(pix)]);
			}
			xpos += dxpos;
			firstpix++;
		}
	}

	firstpix >>= 2;
	iwidth >>= 2;
	iwidth -= firstpix;

	while (iwidth-- > 0)
	{
		uint32_t pix = src[firstpix++];
		if (!(flags & 4) || pix)
		{
			if ((!(flags & 4) || (pix & 0xff000000)) && (uint32_t)xpos < 760)
			{
				if (!(flags & 2))
					scanline[xpos] = clutbase[BYTE_XOR_BE(pix >> 24)];
				else
					BLEND(scanline[xpos], clutbase[BYTE_XOR_BE(pix >> 24)]);
			}
			xpos += dxpos;

			if ((!(flags & 4) || (pix & 0x00ff0000)) && (uint32_t)xpos < 760)
			{
				if (!(flags & 2))
					scanline[xpos] = clutbase[BYTE_XOR_BE((pix >> 16) & 0xff)];
				else
					BLEND(scanline[xpos], clutbase[BYTE_XOR_BE((pix >> 16) & 0xff)]);
			}
			xpos += dxpos;

			if ((!(flags & 4) || (pix & 0x0000ff00)) && (uint32_t)xpos < 760)
			{
				if (!(flags & 2))
					scanline[xpos] = clutbase[BYTE_XOR_BE((pix >> 8) & 0xff)];
				else
					BLEND(scanline[xpos], clutbase[BYTE_XOR_BE((pix >> 8) & 0xff)]);
			}
			xpos += dxpos;

			if ((!(flags & 4) || (pix & 0x000000ff)) && (uint32_t)xpos < 760)
			{
				if (!(flags & 2))
					scanline[xpos] = clutbase[BYTE_XOR_BE(pix & 0xff)];
				else
					BLEND(scanline[xpos], clutbase[BYTE_XOR_BE(pix & 0xff)]);
			}
			xpos += dxpos;
		}
		else
			xpos += dxpos << 2;
	}
}

void jaguar_state::bitmap_8_0(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint16_t *clutbase)
{
	bitmap_8_draw(scanline, firstpix, iwidth, src, xpos, 0, 1, clutbase);
}

void jaguar_state::bitmap_8_1(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint16_t *clutbase)
{
	bitmap_8_draw(scanline, firstpix, iwidth, src, xpos, 1, -1, clutbase);
}

void jaguar_state::bitmap_8_2(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint16_t *clutbase)
{
	bitmap_8_draw(scanline, firstpix, iwidth, src, xpos, 2, 1, clutbase);
}

void jaguar_state::bitmap_8_3(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint16_t *clutbase)
{
	bitmap_8_draw(scanline, firstpix, iwidth, src, xpos, 3, -1, clutbase);
}

void jaguar_state::bitmap_8_4(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint16_t *clutbase)
{
	bitmap_8_draw(scanline, firstpix, iwidth, src, xpos, 4, 1, clutbase);
}

void jaguar_state::bitmap_8_5(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint16_t *clutbase)
{
	bitmap_8_draw(scanline, firstpix, iwidth, src, xpos, 5, -1, clutbase);
}

void jaguar_state::bitmap_8_6(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint16_t *clutbase)
{
	bitmap_8_draw(scanline, firstpix, iwidth, src, xpos, 6, 1, clutbase);
}

void jaguar_state::bitmap_8_7(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint16_t *clutbase)
{
	bitmap_8_draw(scanline, firstpix, iwidth, src, xpos, 7, -1, clutbase);
}

void (jaguar_state::*const jaguar_state::bitmap8[8])(uint16_t *, int32_t, int32_t, uint32_t *, int32_t, uint16_t *) =
{
	&jaguar_state::bitmap_8_0,
	&jaguar_state::bitmap_8_1,
	&jaguar_state::bitmap_8_2,
	&jaguar_state::bitmap_8_3,
	&jaguar_state::bitmap_8_4,
	&jaguar_state::bitmap_8_5,
	&jaguar_state::bitmap_8_6,
	&jaguar_state::bitmap_8_7
};



/*************************************
 *
 *  16bpp bitmap renderers
 *
 *************************************/

inline void jaguar_state::bitmap_16_draw(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint8_t flags, int32_t dxpos)
{
	if (firstpix & 1)
	{
		uint16_t pix = src[firstpix >> 1];
		if ((!(flags & 4) || pix) && (uint32_t)xpos < 760)
		{
			if (!(flags & 2))
				scanline[xpos] = pix;
			else
				BLEND(scanline[xpos], pix);
		}
		xpos += dxpos;
	}

	firstpix >>= 1;
	iwidth >>= 1;
	iwidth -= firstpix;

	while (iwidth-- > 0)
	{
		uint32_t pix = src[firstpix++];
		if (!(flags & 4) || pix)
		{
			if ((!(flags & 4) || (pix >> 16)) && (uint32_t)xpos < 760)
			{
				if (!(flags & 2))
					scanline[xpos] = pix >> 16;
				else
					BLEND(scanline[xpos], pix >> 16);
			}
			xpos += dxpos;

			if ((!(flags & 4) || (pix & 0xffff)) && (uint32_t)xpos < 760)
			{
				if (!(flags & 2))
					scanline[xpos] = pix;
				else
					BLEND(scanline[xpos], pix);
			}
			xpos += dxpos;
		}
		else
			xpos += dxpos << 1;
	}
}

void jaguar_state::bitmap_16_0(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos)
{
	bitmap_16_draw(scanline, firstpix, iwidth, src, xpos, 0, 1);
}

void jaguar_state::bitmap_16_1(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos)
{
	bitmap_16_draw(scanline, firstpix, iwidth, src, xpos, 1, -1);
}

void jaguar_state::bitmap_16_2(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos)
{
	bitmap_16_draw(scanline, firstpix, iwidth, src, xpos, 2, 1);
}

void jaguar_state::bitmap_16_3(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos)
{
	bitmap_16_draw(scanline, firstpix, iwidth, src, xpos, 3, -1);
}

void jaguar_state::bitmap_16_4(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos)
{
	bitmap_16_draw(scanline, firstpix, iwidth, src, xpos, 4, 1);
}

void jaguar_state::bitmap_16_5(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos)
{
	bitmap_16_draw(scanline, firstpix, iwidth, src, xpos, 5, -1);
}

void jaguar_state::bitmap_16_6(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos)
{
	bitmap_16_draw(scanline, firstpix, iwidth, src, xpos, 6, 1);
}

void jaguar_state::bitmap_16_7(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos)
{
	bitmap_16_draw(scanline, firstpix, iwidth, src, xpos, 7, -1);
}

void (jaguar_state::*const jaguar_state::bitmap16[8])(uint16_t *, int32_t, int32_t, uint32_t *, int32_t) =
{
	&jaguar_state::bitmap_16_0,
	&jaguar_state::bitmap_16_1,
	&jaguar_state::bitmap_16_2,
	&jaguar_state::bitmap_16_3,
	&jaguar_state::bitmap_16_4,
	&jaguar_state::bitmap_16_5,
	&jaguar_state::bitmap_16_6,
	&jaguar_state::bitmap_16_7
};





/*************************************
 *
 *  32bpp bitmap renderers - needs to be verified
 *
 *************************************/

inline void jaguar_state::bitmap_32_draw(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos, uint8_t flags, int32_t dxpos)
{
	iwidth -= firstpix;

	while (iwidth-- > 0)
	{
		uint32_t pix = src[firstpix++];

		if (xpos < 760)
		{
			scanline[xpos++] = (pix&0xffff0000)>>16;
			scanline[xpos++] = pix&0xffff;
		}
	}
}

void jaguar_state::bitmap_32_0(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos)
{
	bitmap_32_draw(scanline, firstpix, iwidth, src, xpos, 0, 1);
}

void jaguar_state::bitmap_32_1(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos)
{
	bitmap_32_draw(scanline, firstpix, iwidth, src, xpos, 1, -1);
}

void jaguar_state::bitmap_32_2(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos)
{
	bitmap_32_draw(scanline, firstpix, iwidth, src, xpos, 2, 1);
}

void jaguar_state::bitmap_32_3(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos)
{
	bitmap_32_draw(scanline, firstpix, iwidth, src, xpos, 3, -1);
}

void jaguar_state::bitmap_32_4(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos)
{
	bitmap_32_draw(scanline, firstpix, iwidth, src, xpos, 4, 1);
}

void jaguar_state::bitmap_32_5(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos)
{
	bitmap_32_draw(scanline, firstpix, iwidth, src, xpos, 5, -1);
}

void jaguar_state::bitmap_32_6(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos)
{
	bitmap_32_draw(scanline, firstpix, iwidth, src, xpos, 6, 1);
}

void jaguar_state::bitmap_32_7(uint16_t *scanline, int32_t firstpix, int32_t iwidth, uint32_t *src, int32_t xpos)
{
	bitmap_32_draw(scanline, firstpix, iwidth, src, xpos, 7, -1);
}

void (jaguar_state::*const jaguar_state::bitmap32[8])(uint16_t *, int32_t, int32_t, uint32_t *, int32_t) =
{
	&jaguar_state::bitmap_32_0,
	&jaguar_state::bitmap_32_1,
	&jaguar_state::bitmap_32_2,
	&jaguar_state::bitmap_32_3,
	&jaguar_state::bitmap_32_4,
	&jaguar_state::bitmap_32_5,
	&jaguar_state::bitmap_32_6,
	&jaguar_state::bitmap_32_7
};



static inline uint8_t lookup_pixel(const uint32_t *src, int i, int pitch, int depth)
{
	int ppl     = 32 / depth;
	uint32_t data = src[((i & ppl) / ppl) + ((i / (ppl<<1)) * (pitch<<1))];
	uint8_t pix   = (data >> ((~i & (ppl-1)) * depth)) & ((1 << depth) - 1);
	return pix;
}



/*************************************
 *
 *  Standard bitmap processor
 *
 *************************************/

uint32_t *jaguar_state::process_bitmap(uint16_t *scanline, uint32_t *objdata, int vc, bool logit)
{
	/* extract minimal data */
	uint32_t upper = objdata[0];
	uint32_t lower = objdata[1];
	uint32_t ypos = (lower >> 3) & 0x7ff;
	uint32_t height = (lower >> 14) & 0x3ff;
	uint32_t link = (lower >> 24) | ((upper & 0x7ff) << 8);
	uint32_t data = (upper >> 11);
	uint32_t *src = (uint32_t *)memory_base(data << 3);

	if (logit)
	{
		/* second phrase */
		uint32_t upper2 = objdata[2];
		uint32_t lower2 = objdata[3];

		/* extract data */
		int32_t xpos = (int32_t)(lower2 << 20) >> 20;
		uint8_t depth = 1 << ((lower2 >> 12) & 7);
		uint8_t pitch = (lower2 >> 15) & 7;
		uint32_t dwidth = (lower2 >> 18) & 0x3ff;
		int32_t iwidth = (lower2 >> 28) | ((upper2 & 0x3f) << 4);
		uint8_t _index = (upper2 >> 6) & 0x3f;
		uint8_t flags = (upper2 >> 13) & 0x0f;
		uint8_t firstpix = (upper2 >> 17) & 0x3f;

		logerror("        ypos=%X height=%X link=%06X data=%06X\n", ypos, height, link << 3, data << 3);
		logerror("        xpos=%X depth=%X pitch=%X dwidth=%X iwidth=%X index=%X flags=%X firstpix=%X\n", xpos, depth, pitch, dwidth, iwidth, _index, flags, firstpix);
	}

	/* only render if valid */
	if (vc >= ypos && height > 0 && src)
	{
		/* second phrase */
		uint32_t upper2 = objdata[2];
		uint32_t lower2 = objdata[3];

		/* extract data */
		int32_t xpos = (int32_t)(lower2 << 20) >> 20;
		uint8_t depthlog = (lower2 >> 12) & 7;
		uint8_t pitch = (lower2 >> 15) & 7;
		uint32_t dwidth = (lower2 >> 18) & 0x3ff;
		uint32_t iwidth = ((lower2 >> 28) | ((upper2 & 0x3f) << 4)) << (6 - depthlog);
		uint8_t _index = (upper2 >> 5) & 0xfe;
		uint8_t flags = (upper2 >> 13) & 0x07;
		uint8_t firstpix = ((upper2 >> 17) & 0x3f) >> depthlog;
		int i, dxpos = (flags & 1) ? -1 : 1;

		/* preadjust for firstpix */
		xpos += firstpix * dxpos;

		/* switch off the depth */
		switch (depthlog)
		{
			/* 1bpp case */
			case 0:
			{
				uint16_t *clut = (uint16_t *)&m_gpu_clut[0] + _index;

				/* non-blending */
				if (!(flags & 2))
				{
					for (i = firstpix; i < iwidth; i++)
					{
						uint8_t pix = lookup_pixel(src, i, pitch, 1);

						if (xpos >= 0 && xpos < 760 && (pix || !(flags & 4)))
							scanline[xpos] = clut[BYTE_XOR_BE(pix)];
						xpos += dxpos;
					}
				}

				/* blending */
				else
				{
					for (i = firstpix; i < iwidth; i++)
					{
						uint8_t pix = lookup_pixel(src, i, pitch, 1);

						if (xpos >= 0 && xpos < 760 && (pix || !(flags & 4)))
							BLEND(scanline[xpos], clut[BYTE_XOR_BE(pix)]);
						xpos += dxpos;
					}
				}
				break;
			}

			/* 2bpp case */
			case 1:
			{
				uint16_t *clut = (uint16_t *)&m_gpu_clut[0] + (_index & 0xfc);

				/* non-blending */
				if (!(flags & 2))
				{
					for (i = firstpix; i < iwidth; i++)
					{
						uint8_t pix = lookup_pixel(src, i, pitch, 2);

						if (xpos >= 0 && xpos < 760 && (pix || !(flags & 4)))
							scanline[xpos] = clut[BYTE_XOR_BE(pix)];
						xpos += dxpos;
					}
				}

				/* blending */
				else
				{
					for (i = firstpix; i < iwidth; i++)
					{
						uint8_t pix = lookup_pixel(src, i, pitch, 2);

						if (xpos >= 0 && xpos < 760 && (pix || !(flags & 4)))
							BLEND(scanline[xpos], clut[BYTE_XOR_BE(pix)]);
						xpos += dxpos;
					}
				}
				break;
			}

			/* 4bpp case */
			case 2:
				/* only handle pitch=1 for now */
				if (pitch != 1)
					logerror("Unhandled pitch = %d\n", pitch);

				(this->*bitmap4[flags])(scanline, firstpix, iwidth, src, xpos, (uint16_t *)&m_gpu_clut[0] + (_index & 0xf8));
				break;

			/* 8bpp case */
			case 3:
				/* only handle pitch=1 for now */
				if (pitch != 1)
					logerror("Unhandled pitch = %d\n", pitch);

				(this->*bitmap8[flags])(scanline, firstpix, iwidth, src, xpos, (uint16_t *)&m_gpu_clut[0]);
				break;

			/* 16bpp case */
			case 4:
				/* only handle pitch=1 for now */
				if (pitch != 1)
					logerror("Unhandled pitch = %d\n", pitch);

				(this->*bitmap16[flags])(scanline, firstpix, iwidth, src, xpos);
				break;

			/* 32bpp case */
			case 5:
				/* only handle pitch=1 for now */
				if (pitch != 1)
					logerror("Unhandled pitch = %d\n", pitch);

				(this->*bitmap32[flags])(scanline, firstpix, iwidth, src, xpos);
				break;

			default:
				fprintf(stderr, "Unhandled bitmap source depth = %d\n", depthlog);
				break;
		}

		/* decrement the height and add to the source data offset */
		objdata[0] = upper + (dwidth << 11);
		objdata[1] = lower - (1 << 14);
	}

	return (uint32_t *)memory_base(link << 3);
}



/*************************************
 *
 *  Scaled bitmap object processor
 *
 *************************************/

uint32_t *jaguar_state::process_scaled_bitmap(uint16_t *scanline, uint32_t *objdata, int vc, bool logit)
{
	/* extract data */
	uint32_t upper = objdata[0];
	uint32_t lower = objdata[1];
	uint32_t ypos = (lower >> 3) & 0x7ff;
	uint32_t height = (lower >> 14) & 0x3ff;
	uint32_t link = (lower >> 24) | ((upper & 0x7ff) << 8);
	uint32_t data = (upper >> 11);
	uint32_t *src = (uint32_t *)memory_base(data << 3);

	/* third phrase */
	uint32_t lower3 = objdata[5];
	int32_t remainder = (lower3 >> 16) & 0xff;

	if (logit)
	{
		/* second phrase */
		uint32_t upper2 = objdata[2];
		uint32_t lower2 = objdata[3];

		/* extract data */
		int32_t xpos = (int32_t)(lower2 << 20) >> 20;
		uint8_t depth = 1 << ((lower2 >> 12) & 7);
		uint8_t pitch = (lower2 >> 15) & 7;
		uint32_t dwidth = (lower2 >> 18) & 0x3ff;
		int32_t iwidth = (lower2 >> 28) | ((upper2 & 0x3f) << 4);
		uint8_t _index = (upper2 >> 6) & 0x3f;
		uint8_t flags = (upper2 >> 13) & 0x0f;
		uint8_t firstpix = (upper2 >> 17) & 0x3f;

		int32_t hscale = lower3 & 0xff;
		int32_t vscale = (lower3 >> 8) & 0xff;

		logerror("        ypos=%X height=%X link=%06X data=%06X\n", ypos, height, link << 3, data << 3);
		logerror("        xpos=%X depth=%X pitch=%X dwidth=%X iwidth=%X index=%X flags=%X firstpix=%X\n", xpos, depth, pitch, dwidth, iwidth, _index, flags, firstpix);
		logerror("        hscale=%X vscale=%X remainder=%X\n", hscale, vscale, remainder);
	}

	/* only render if valid */
	if (vc >= ypos && (height > 0 || remainder > 0) && src)
	{
		/* second phrase */
		uint32_t upper2 = objdata[2];
		uint32_t lower2 = objdata[3];

		/* extract data */
		int32_t xpos = (int32_t)(lower2 << 20) >> 20;
		uint8_t depthlog = (lower2 >> 12) & 7;
		uint8_t pitch = (lower2 >> 15) & 7;
		uint32_t dwidth = (lower2 >> 18) & 0x3ff;
		int32_t iwidth = ((lower2 >> 28) | ((upper2 & 0x3f) << 4)) << (6 - depthlog);
		uint8_t _index = (upper2 >> 5) & 0xfe;
		uint8_t flags = (upper2 >> 13) & 0x07;
		uint8_t firstpix = ((upper2 >> 17) & 0x3f) >> depthlog;

		int32_t hscale = lower3 & 0xff;
		int32_t vscale = (lower3 >> 8) & 0xff;
		int32_t xleft = hscale;
		int dxpos = (flags & 1) ? -1 : 1;
		int xpix = firstpix, yinc;

		/* only handle pitch=1 (sequential data) for now */
		if (pitch != 1)
			logerror("Unhandled pitch = %d\n", pitch);
		if (flags & 2)
		{
			osd_printf_debug("Unhandled blend mode in scaled bitmap case\n");
			logerror("Unhandled blend mode in scaled bitmap case\n");
		}

		/* preadjust for firstpix */
		xpos += firstpix * dxpos;

		/* ignore hscale = 0 */
		if (hscale != 0)
		{
			/* switch off the depth */
			switch (depthlog)
			{
				case 0:
				{
					uint16_t *clut = (uint16_t *)&m_gpu_clut[0] + _index;

					/* render in phrases */
					while (xpix < iwidth)
					{
						uint16_t pix = (src[xpix >> 5] >> (~xpix & 31)) & 0x01;

						while (xleft > 0)
						{
							if (xpos >= 0 && xpos < 760 && (pix || !(flags & 4)))
								scanline[xpos] = clut[BYTE_XOR_BE(pix)];
							xpos += dxpos;
							xleft -= 0x20;
						}
						while (xleft <= 0)
							xleft += hscale, xpix++;
					}
					break;
				}

				case 1:
				{
					uint16_t *clut = (uint16_t *)&m_gpu_clut[0] + (_index & 0xfc);

					/* render in phrases */
					while (xpix < iwidth)
					{
						uint16_t pix = (src[xpix >> 4] >> ((~xpix & 15) << 1)) & 0x03;

						while (xleft > 0)
						{
							if (xpos >= 0 && xpos < 760 && (pix || !(flags & 4)))
								scanline[xpos] = clut[BYTE_XOR_BE(pix)];
							xpos += dxpos;
							xleft -= 0x20;
						}
						while (xleft <= 0)
							xleft += hscale, xpix++;
					}
					break;
				}

				case 2:
				{
					uint16_t *clut = (uint16_t *)&m_gpu_clut[0] + (_index & 0xf8);

					/* render in phrases */
					while (xpix < iwidth)
					{
						uint16_t pix = (src[xpix >> 3] >> ((~xpix & 7) << 2)) & 0x0f;

						while (xleft > 0)
						{
							if (xpos >= 0 && xpos < 760 && (pix || !(flags & 4)))
								scanline[xpos] = clut[BYTE_XOR_BE(pix)];
							xpos += dxpos;
							xleft -= 0x20;
						}
						while (xleft <= 0)
							xleft += hscale, xpix++;
					}
					break;
				}

				case 3:
				{
					uint16_t *clut = (uint16_t *)&m_gpu_clut[0];

					/* render in phrases */
					while (xpix < iwidth)
					{
						uint16_t pix = (src[xpix >> 2] >> ((~xpix & 3) << 3)) & 0xff;

						while (xleft > 0)
						{
							if (xpos >= 0 && xpos < 760 && (pix || !(flags & 4)))
								scanline[xpos] = clut[BYTE_XOR_BE(pix)];
							xpos += dxpos;
							xleft -= 0x20;
						}
						while (xleft <= 0)
							xleft += hscale, xpix++;
					}
					break;
				}

				case 4:
					while (xpix < iwidth)
					{
						uint16_t pix = src[xpix >> 1] >> ((~xpix & 1) << 4);

						while (xleft > 0)
						{
							if (xpos >= 0 && xpos < 760 && (pix || !(flags & 4)))
								scanline[xpos] = pix;
							xpos += dxpos;
							xleft -= 0x20;
						}
						while (xleft <= 0)
							xleft += hscale, xpix++;
					}
					break;

				default:
					fprintf(stderr, "Unhandled scaled bitmap source depth = %d\n", depthlog);
					break;
			}
		}

		/* handle Y scale */
		remainder -= 0x20;
		yinc = 0;
		while (remainder <= 0 && vscale != 0)
			remainder += vscale, yinc++;
		if (yinc > height)
			yinc = height, remainder = 0;

		/* decrement the height and add to the source data offset */
		objdata[0] = upper + yinc * (dwidth << 11);
		objdata[1] = lower - yinc * (1 << 14);
		objdata[5] = (lower3 & ~0xff0000) | ((remainder & 0xff) << 16);
	}

	return (uint32_t *)memory_base(link << 3);
}



/*************************************
 *
 *  Branch object processor
 *
 *************************************/

uint32_t *jaguar_state::process_branch(uint32_t *objdata, int vc, bool logit)
{
	uint32_t upper = objdata[0];
	uint32_t lower = objdata[1];
	uint32_t ypos = (lower >> 3) & 0x7ff;
	uint32_t cc = (lower >> 14) & 7;
	uint32_t link = (lower >> 24) | ((upper & 0x7ff) << 8);
	int taken = 0;

//  if ((ypos & 1) && ypos != 0x7ff)
//      fprintf(stderr, "        branch cc=%d ypos=%X link=%06X - \n", cc, ypos, link << 3);

	switch (cc)
	{
		/* 0: branch if ypos == vc or ypos == 0x7ff */
		case 0:
			if (logit) logerror("        branch if %X == vc or %X == 0x7ff to %06X\n", ypos, ypos, link << 3);
			taken = (ypos == vc) || (ypos == 0x7ff);
			break;

		/* 1: branch if ypos > vc */
		case 1:
			if (logit) logerror("        branch if %X > vc to %06X\n", ypos, link << 3);
			taken = (ypos > vc);
			break;

		/* 2: branch if ypos < vc */
		case 2:
			if (logit) logerror("        branch if %X < vc to %06X\n", ypos, link << 3);
			taken = (ypos < vc);
			break;

		/* 3: branch if object processor flag is set */
		case 3:
			if (logit) logerror("        branch if object flag set to %06X\n", link << 3);
			taken = m_gpu_regs[OBF] & 1;
			break;

		/* 4: branch on second half of display line */
		case 4:
			if (logit) logerror("        branch if second half of line to %06X\n", link << 3);
			taken = (vc & 1);
			break;

		default:
			fprintf(stderr, "Invalid branch!\n");
			link = 0; taken = 1;
			break;
	}

	/* handle the branch */
	return taken ? (uint32_t *)memory_base(link << 3) : (objdata + 2);
}



/*************************************
 *
 *  Process object list
 *
 *************************************/

void jaguar_state::process_object_list(int vc, uint16_t *scanline)
{
	int done = 0, count = 0;
	uint32_t *objdata;
	bool logit;
	int x;

	/* erase the scanline first */
	for (x = 0; x < 760; x++)
		scanline[x] = m_gpu_regs[BG];

	logit = LOG_OBJECTS;

	/* fetch the object pointer */
	objdata = (uint32_t *)memory_base((m_gpu_regs[OLP_H] << 16) | m_gpu_regs[OLP_L]);
	while (!done && objdata && count++ < 100)
	{
		/* the low 3 bits determine the command */
		switch (objdata[1] & 7)
		{
			/* bitmap object */
			case 0:
				if (logit)
					logerror("bitmap = %08X-%08X %08X-%08X\n", objdata[0], objdata[1], objdata[2], objdata[3]);
				objdata = process_bitmap(scanline, objdata, vc, logit);
				break;

			/* scaled bitmap object */
			case 1:
				if (logit)
					logerror("scaled = %08X-%08X %08X-%08X %08X-%08X\n", objdata[0], objdata[1], objdata[2], objdata[3], objdata[4], objdata[5]);
				objdata = process_scaled_bitmap(scanline, objdata, vc, logit);
				break;


			/* GPU interrupt */
			case 2:
				m_gpu_regs[OB_HH]=(objdata[1]&0xffff0000)>>16;
				m_gpu_regs[OB_HL]=objdata[1]&0xffff;
				m_gpu_regs[OB_LH]=(objdata[0]&0xffff0000)>>16;
				m_gpu_regs[OB_LL]=objdata[0]&0xffff;
				m_cpu_irq_state |= 2;
				update_cpu_irq();
				done=1;
				break;

			/* branch */
			case 3:
				if (logit)
					logerror("branch = %08X-%08X\n", objdata[0], objdata[1]);
				objdata = process_branch(objdata, vc, logit);
				break;

			/* stop */
			case 4:
			{
				int interrupt = (objdata[1] >> 3) & 1;
				done = 1;

				if (logit)
					logerror("stop   = %08X-%08X\n", objdata[0], objdata[1]);
				if (interrupt)
				{
//                  fprintf(stderr, "stop int=%d\n", interrupt);
					m_cpu_irq_state |= 4;
					update_cpu_irq();
				}
				break;
			}

			default:
				fprintf(stderr, "%08X %08X\n", objdata[0], objdata[1]);
				done = 1;
				break;
		}
	}
}
