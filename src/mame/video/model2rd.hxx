// license:BSD-3-Clause
// copyright-holders:R. Belmont, Olivier Galibert, ElSemi, Angelo Salese
/********************************************************************

    Sega Model 2 3D rasterization functions

********************************************************************/

#undef MODEL2_CHECKER
#undef MODEL2_TEXTURED
#undef MODEL2_TRANSLUCENT

#ifndef MODEL2_FUNC
#error "Model 2 renderer: No function defined!"
#endif

#ifndef MODEL2_FUNC_NAME
#error "Model 2 renderer: No function name defined!"
#endif

#if MODEL2_FUNC == 0
#undef MODEL2_CHECKER
#undef MODEL2_TEXTURED
#undef MODEL2_TRANSLUCENT
#elif MODEL2_FUNC == 1
#undef MODEL2_CHECKER
#undef MODEL2_TEXTURED
#define MODEL2_TRANSLUCENT
#elif MODEL2_FUNC == 2
#undef MODEL2_CHECKER
#define MODEL2_TEXTURED
#undef MODEL2_TRANSLUCENT
#elif MODEL2_FUNC == 3
#undef MODEL2_CHECKER
#define MODEL2_TEXTURED
#define MODEL2_TRANSLUCENT
#elif MODEL2_FUNC == 4
#define MODEL2_CHECKER
#undef MODEL2_TEXTURED
#undef MODEL2_TRANSLUCENT
#elif MODEL2_FUNC == 5
#define MODEL2_CHECKER
#undef MODEL2_TEXTURED
#define MODEL2_TRANSLUCENT
#elif MODEL2_FUNC == 6
#define MODEL2_CHECKER
#define MODEL2_TEXTURED
#undef MODEL2_TRANSLUCENT
#elif MODEL2_FUNC == 7
#define MODEL2_CHECKER
#define MODEL2_TEXTURED
#define MODEL2_TRANSLUCENT
#else
#error "Model 2 renderer: Invalid function selected!"
#endif

#ifndef MODEL2_TEXTURED
/* non-textured render path */
void MODEL2_FUNC_NAME(int32_t scanline, const extent_t& extent, const m2_poly_extra_data& object, int threadid)
{
#if !defined( MODEL2_TRANSLUCENT)
	model2_state *state = object.state;
	bitmap_rgb32 *destmap = (bitmap_rgb32 *)&m_destmap;
	uint32_t *p = &destmap->pix32(scanline);

	/* extract color information */
	const uint16_t *colortable_r = (const uint16_t *)&state->m_colorxlat[0x0000/4];
	const uint16_t *colortable_g = (const uint16_t *)&state->m_colorxlat[0x4000/4];
	const uint16_t *colortable_b = (const uint16_t *)&state->m_colorxlat[0x8000/4];
	const uint16_t *lumaram = (const uint16_t *)state->m_lumaram.target();
	uint32_t  lumabase = object.lumabase;
	uint32_t  color = object.colorbase;
	uint8_t   luma;
	uint32_t  tr, tg, tb;
	int     x;
#endif
	/* if it's translucent, there's nothing to render */
#if defined( MODEL2_TRANSLUCENT)
	return;
#else

	luma = lumaram[BYTE_XOR_LE(lumabase + (0xf << 3))] & 0x3F;

	color = state->m_palram[BYTE_XOR_LE(color + 0x1000)] & 0x7fff;

	colortable_r += ((color >>  0) & 0x1f) << 8;
	colortable_g += ((color >>  5) & 0x1f) << 8;
	colortable_b += ((color >> 10) & 0x1f) << 8;

	/* we have the 6 bits of luma information along with 5 bits per color component */
	/* now build and index into the master color lookup table and extract the raw RGB values */

	tr = colortable_r[BYTE_XOR_LE(luma)] & 0xff;
	tg = colortable_g[BYTE_XOR_LE(luma)] & 0xff;
	tb = colortable_b[BYTE_XOR_LE(luma)] & 0xff;

	/* build the final color */
	color = rgb_t(tr, tg, tb);

	for(x = extent.startx; x < extent.stopx; x++)
#if defined(MODEL2_CHECKER)
		if ((x^scanline) & 1) p[x] = color;
#else
		p[x] = color;
#endif
#endif
}

#else
/* textured render path */
void MODEL2_FUNC_NAME(int32_t scanline, const extent_t& extent, const m2_poly_extra_data& object, int threadid)
{
	model2_state *state = object.state;
	bitmap_rgb32 *destmap = (bitmap_rgb32 *)&m_destmap;
	uint32_t *p = &destmap->pix32(scanline);

	uint32_t  tex_width = object.texwidth;
	uint32_t  tex_height = object.texheight;

	/* extract color information */
	const uint16_t *colortable_r = (const uint16_t *)&state->m_colorxlat[0x0000/4];
	const uint16_t *colortable_g = (const uint16_t *)&state->m_colorxlat[0x4000/4];
	const uint16_t *colortable_b = (const uint16_t *)&state->m_colorxlat[0x8000/4];
	const uint16_t *lumaram = (const uint16_t *)state->m_lumaram.target();
	uint32_t  colorbase = object.colorbase;
	uint32_t  lumabase = object.lumabase;
	uint32_t  tex_x = object.texx;
	uint32_t  tex_y = object.texy;
	uint32_t  tex_x_mask, tex_y_mask;
	uint32_t  tex_mirr_x = object.texmirrorx;
	uint32_t  tex_mirr_y = object.texmirrory;
	uint32_t *sheet = object.texsheet;
	float ooz = extent.param[0].start;
	float uoz = extent.param[1].start;
	float voz = extent.param[2].start;
	float dooz = extent.param[0].dpdx;
	float duoz = extent.param[1].dpdx;
	float dvoz = extent.param[2].dpdx;
	int     x;

	tex_x_mask  = tex_width - 1;
	tex_y_mask  = tex_height - 1;

	colorbase = state->m_palram[BYTE_XOR_LE(colorbase + 0x1000)] & 0x7fff;

	colortable_r += ((colorbase >>  0) & 0x1f) << 8;
	colortable_g += ((colorbase >>  5) & 0x1f) << 8;
	colortable_b += ((colorbase >> 10) & 0x1f) << 8;

	for(x = extent.startx; x < extent.stopx; x++, uoz += duoz, voz += dvoz, ooz += dooz)
	{
		float z = recip_approx(ooz) * 256.0f;
		int32_t u = uoz * z;
		int32_t v = voz * z;
		uint32_t  tr, tg, tb;
		uint16_t  t;
		uint8_t luma;
		int u2;
		int v2;

#if defined(MODEL2_CHECKER)
		if ( ((x^scanline) & 1) == 0 )
			continue;
#endif
		u2 = (u >> 8) & tex_x_mask;
		v2 = (v >> 8) & tex_y_mask;

		if ( tex_mirr_x )
			u2 = ( tex_width - 1 ) - u2;

		if ( tex_mirr_y )
			v2 = ( tex_height - 1 ) - v2;

		t = get_texel( tex_x, tex_y, u2, v2, sheet );

#if defined(MODEL2_TRANSLUCENT)
		if ( t == 0x0f )
			continue;
#endif
		luma = lumaram[BYTE_XOR_LE(lumabase + (t << 3))] & 0x3f;

		/* we have the 6 bits of luma information along with 5 bits per color component */
		/* now build and index into the master color lookup table and extract the raw RGB values */

		tr = colortable_r[BYTE_XOR_LE(luma)] & 0xff;
		tg = colortable_g[BYTE_XOR_LE(luma)] & 0xff;
		tb = colortable_b[BYTE_XOR_LE(luma)] & 0xff;

		p[x] = rgb_t(tr, tg, tb);
	}
}

#endif
