// license:BSD-3-Clause
// copyright-holders: Couriersud, Olivier Galibert, R. Belmont
//============================================================
//
//  draw13.c - SDL 2.0 drawing implementation
//
//  SDLMAME by Olivier Galibert and R. Belmont
//
//  SDL 2.0 renderer by Couriersud
//
//============================================================

// standard C headers
#include <math.h>
#include <stdio.h>

// MAME headers
#include "emu.h"
#include "options.h"

// OSD headers
#include "osdsdl.h"
#include "window.h"

#include "draw13.h"

//============================================================
//  DEBUGGING
//============================================================

//============================================================
//  CONSTANTS
//============================================================

#define STAT_PIXEL_THRESHOLD (150*150)

enum
{
	TEXTURE_TYPE_NONE,
	TEXTURE_TYPE_PLAIN,
	TEXTURE_TYPE_SURFACE
};


//============================================================
//  Inline functions
//============================================================

static inline bool is_opaque(const float &a)
{
	return (a >= 1.0f);
}

static inline bool is_transparent(const float &a)
{
	return (a <  0.0001f);
}

//============================================================
//  CONSTRUCTOR & DESTRUCTOR
//============================================================

renderer_sdl2::renderer_sdl2(std::shared_ptr<osd_window> window, int extra_flags)
	: osd_renderer(window,  FLAG_NEEDS_OPENGL | extra_flags)
	, m_sdl_renderer(nullptr)
	, m_blittimer(0)
	, m_last_hofs(0)
	, m_last_vofs(0)
	, m_width(0)
	, m_height(0)
	, m_blit_dim(0, 0)
	, m_last_blit_time(0)
	, m_last_blit_pixels(0)
{
	for (int i = 0; i < 30; i++)
	{
		fmt_support[i].format = 0;
		fmt_support[i].status = 0;
	}

	if (!s_blit_info_initialized)
	{
		/* On OSX, calling this from drawsdl2_init will
		 * prohibit fullscreen toggling. It is than not possible
		 * to toggle from fullscreen to window mode.
		 */
		expand_copy_info(s_blit_info_default);
		s_blit_info_initialized = true;
	}
}


//============================================================
//  STATIC VARIABLES
//============================================================

#define BM_ALL (UINT32_MAX)
//( SDL_BLENDMODE_MASK | SDL_BLENDMODE_BLEND | SDL_BLENDMODE_ADD | SDL_BLENDMODE_MOD)

#define ENTRY(a,b,f) { SDL_TEXFORMAT_ ## a, SDL_PIXELFORMAT_ ## b, &texcopy_ ## f, BM_ALL, #a, #b, 0, 0, 0, 0}
#define ENTRY_BM(a,b,f,bm) { SDL_TEXFORMAT_ ## a, SDL_PIXELFORMAT_ ## b, &texcopy_ ## f, bm, #a, #b, 0, 0, 0, 0}
#define ENTRY_LR(a,b,f) { SDL_TEXFORMAT_ ## a, SDL_PIXELFORMAT_ ## b, &texcopy_ ## f, BM_ALL, #a, #b, 0, 0, 0, -1}

const copy_info_t renderer_sdl2::s_blit_info_default[] =
{
	/* no rotation */
	ENTRY(ARGB32,           ARGB8888,   argb32_argb32),
	ENTRY_LR(ARGB32,        RGB888,     argb32_rgb32),
	/* Entry primarily for directfb */
	ENTRY_BM(ARGB32,        RGB888,     argb32_rgb32, SDL_BLENDMODE_ADD),
	ENTRY_BM(ARGB32,        RGB888,     argb32_rgb32, SDL_BLENDMODE_MOD),
	ENTRY_BM(ARGB32,        RGB888,     argb32_rgb32, SDL_BLENDMODE_NONE),

	ENTRY(RGB32,            ARGB8888,   rgb32_argb32),
	ENTRY(RGB32,            RGB888,     rgb32_rgb32),

	ENTRY(RGB32_PALETTED,   ARGB8888,   rgb32pal_argb32),
	ENTRY(RGB32_PALETTED,   RGB888,     rgb32pal_argb32),

	ENTRY(YUY16,            UYVY,       yuv16_uyvy),
	ENTRY(YUY16,            YUY2,       yuv16_yuy2),
	ENTRY(YUY16,            YVYU,       yuv16_yvyu),
	ENTRY(YUY16,            ARGB8888,   yuv16_argb32),
	ENTRY(YUY16,            RGB888,     yuv16_argb32),

	ENTRY(YUY16_PALETTED,   UYVY,       yuv16pal_uyvy),
	ENTRY(YUY16_PALETTED,   YUY2,       yuv16pal_yuy2),
	ENTRY(YUY16_PALETTED,   YVYU,       yuv16pal_yvyu),
	ENTRY(YUY16_PALETTED,   ARGB8888,   yuv16pal_argb32),
	ENTRY(YUY16_PALETTED,   RGB888,     yuv16pal_argb32),

	ENTRY(PALETTE16,        ARGB8888,   pal16_argb32),
	ENTRY(PALETTE16,        RGB888,     pal16_argb32),

	ENTRY(RGB15,            RGB555,     rgb15_rgb555),
	ENTRY(RGB15,            ARGB1555,   rgb15_argb1555),
	ENTRY(RGB15,            ARGB8888,   rgb15_argb32),
	ENTRY(RGB15,            RGB888,     rgb15_argb32),

	ENTRY(RGB15_PALETTED,   ARGB8888,   rgb15pal_argb32),
	ENTRY(RGB15_PALETTED,   RGB888,     rgb15pal_argb32),

	ENTRY(PALETTE16A,       ARGB8888,   pal16a_argb32),
	ENTRY(PALETTE16A,       RGB888,     pal16a_rgb32),

	/* rotation */
	ENTRY(ARGB32,           ARGB8888,   rot_argb32_argb32),
	ENTRY_LR(ARGB32,        RGB888,     rot_argb32_rgb32),
	/* Entry primarily for directfb */
	ENTRY_BM(ARGB32,        RGB888,     rot_argb32_rgb32, SDL_BLENDMODE_ADD),
	ENTRY_BM(ARGB32,        RGB888,     rot_argb32_rgb32, SDL_BLENDMODE_MOD),
	ENTRY_BM(ARGB32,        RGB888,     rot_argb32_rgb32, SDL_BLENDMODE_NONE),

	ENTRY(RGB32,            ARGB8888,   rot_rgb32_argb32),
	ENTRY(RGB32,            RGB888,     rot_argb32_argb32),

	ENTRY(RGB32_PALETTED,   ARGB8888,   rot_rgb32pal_argb32),
	ENTRY(RGB32_PALETTED,   RGB888,     rot_rgb32pal_argb32),

	ENTRY(YUY16,            ARGB8888,   rot_yuv16_argb32rot),
	ENTRY(YUY16,            RGB888,     rot_yuv16_argb32rot),

	ENTRY(YUY16_PALETTED,   ARGB8888,   rot_yuv16pal_argb32rot),
	ENTRY(YUY16_PALETTED,   RGB888,     rot_yuv16pal_argb32rot),

	ENTRY(PALETTE16,        ARGB8888,   rot_pal16_argb32),
	ENTRY(PALETTE16,        RGB888,     rot_pal16_argb32),

	ENTRY(RGB15,            RGB555,     rot_rgb15_argb1555),
	ENTRY(RGB15,            ARGB1555,   rot_rgb15_argb1555),
	ENTRY(RGB15,            ARGB8888,   rot_rgb15_argb32),
	ENTRY(RGB15,            RGB888,     rot_rgb15_argb32),

	ENTRY(RGB15_PALETTED,   ARGB8888,   rot_rgb15pal_argb32),
	ENTRY(RGB15_PALETTED,   RGB888,     rot_rgb15pal_argb32),

	ENTRY(PALETTE16A,       ARGB8888,   rot_pal16a_argb32),
	ENTRY(PALETTE16A,       RGB888,     rot_pal16a_rgb32),

{ -1 },
};

copy_info_t* renderer_sdl2::s_blit_info[SDL_TEXFORMAT_LAST+1] = { nullptr };
bool renderer_sdl2::s_blit_info_initialized = false;

//============================================================
//  INLINES
//============================================================


static inline float round_nearest(float f)
{
	return floor(f + 0.5f);
}

static inline HashT texture_compute_hash(const render_texinfo &texture, const uint32_t flags)
{
	return (HashT)texture.base ^ (flags & (PRIMFLAG_BLENDMODE_MASK | PRIMFLAG_TEXFORMAT_MASK));
}

static inline SDL_BlendMode map_blendmode(const int blendmode)
{
	switch (blendmode)
	{
		case BLENDMODE_NONE:
			return SDL_BLENDMODE_NONE;
		case BLENDMODE_ALPHA:
			return SDL_BLENDMODE_BLEND;
		case BLENDMODE_RGB_MULTIPLY:
			return SDL_BLENDMODE_MOD;
		case BLENDMODE_ADD:
			return SDL_BLENDMODE_ADD;
		default:
			osd_printf_warning("Unknown Blendmode %d", blendmode);
	}
	return SDL_BLENDMODE_NONE;
}

void texture_info::set_coloralphamode(SDL_Texture *texture_id, const render_color *color)
{
	uint32_t sr = (uint32_t)(255.0f * color->r);
	uint32_t sg = (uint32_t)(255.0f * color->g);
	uint32_t sb = (uint32_t)(255.0f * color->b);
	uint32_t sa = (uint32_t)(255.0f * color->a);


	if (color->r >= 1.0f && color->g >= 1.0f && color->b >= 1.0f && is_opaque(color->a))
	{
		SDL_SetTextureColorMod(texture_id, 0xFF, 0xFF, 0xFF);
		SDL_SetTextureAlphaMod(texture_id, 0xFF);
	}
	/* coloring-only case */
	else if (is_opaque(color->a))
	{
		SDL_SetTextureColorMod(texture_id, sr, sg, sb);
		SDL_SetTextureAlphaMod(texture_id, 0xFF);
	}
	/* alpha and/or coloring case */
	else if (!is_transparent(color->a))
	{
		SDL_SetTextureColorMod(texture_id, sr, sg, sb);
		SDL_SetTextureAlphaMod(texture_id, sa);
	}
	else
	{
		SDL_SetTextureColorMod(texture_id, 0xFF, 0xFF, 0xFF);
		SDL_SetTextureAlphaMod(texture_id, 0x00);
	}
}

void texture_info::render_quad(const render_primitive &prim, const int x, const int y)
{
	SDL_Rect target_rect;

	target_rect.x = x;
	target_rect.y = y;
	target_rect.w = round_nearest(prim.bounds.x1) - round_nearest(prim.bounds.x0);
	target_rect.h = round_nearest(prim.bounds.y1) - round_nearest(prim.bounds.y0);

	SDL_SetTextureBlendMode(m_texture_id, m_sdl_blendmode);
	set_coloralphamode(m_texture_id, &prim.color);
	//printf("%d %d %d %d\n", target_rect.x, target_rect.y, target_rect.w, target_rect.h);
	// Arghhh .. Just another bug. SDL_RenderCopy has severe issues with scaling ...
	SDL_RenderCopy(m_renderer->m_sdl_renderer,  m_texture_id, nullptr, &target_rect);
	//SDL_RenderCopyEx(m_renderer->m_sdl_renderer,  m_texture_id, nullptr, &target_rect, 0, nullptr, SDL_FLIP_NONE);
	//SDL_RenderCopyEx(m_renderer->m_sdl_renderer,  m_texture_id, nullptr, nullptr, 0, nullptr, SDL_FLIP_NONE);
}

void renderer_sdl2::render_quad(texture_info *texture, const render_primitive &prim, const int x, const int y)
{
	SDL_Rect target_rect;

	target_rect.x = x;
	target_rect.y = y;
	target_rect.w = round_nearest(prim.bounds.x1 - prim.bounds.x0);
	target_rect.h = round_nearest(prim.bounds.y1 - prim.bounds.y0);

	if (texture)
	{
		copy_info_t *copyinfo = texture->m_copyinfo;
		copyinfo->time -= osd_ticks();
		texture->render_quad(prim, x, y);
		copyinfo->time += osd_ticks();

		copyinfo->pixel_count += std::max(STAT_PIXEL_THRESHOLD , (texture->raw_width() * texture->raw_height()));
		if (m_last_blit_pixels)
		{
			copyinfo->time += (m_last_blit_time * (int64_t) (texture->raw_width() * texture->raw_height())) / (int64_t) m_last_blit_pixels;
		}
		copyinfo->samples++;
		copyinfo->perf = ( texture->m_copyinfo->pixel_count * (osd_ticks_per_second()/1000)) / texture->m_copyinfo->time;
	}
	else
	{
		uint32_t sr = (uint32_t)(255.0f * prim.color.r);
		uint32_t sg = (uint32_t)(255.0f * prim.color.g);
		uint32_t sb = (uint32_t)(255.0f * prim.color.b);
		uint32_t sa = (uint32_t)(255.0f * prim.color.a);

		SDL_SetRenderDrawBlendMode(m_sdl_renderer, map_blendmode(PRIMFLAG_GET_BLENDMODE(prim.flags)));
		SDL_SetRenderDrawColor(m_sdl_renderer, sr, sg, sb, sa);
		SDL_RenderFillRect(m_sdl_renderer, &target_rect);
	}
}

int renderer_sdl2::RendererSupportsFormat(Uint32 format, Uint32 access, const char *sformat)
{
	int i;
	for (i = 0; fmt_support[i].format != 0; i++)
	{
		if (format == fmt_support[i].format)
		{
			return fmt_support[i].status;
		}
	}
	/* not tested yet */
	fmt_support[i].format = format;
	fmt_support[i + 1].format = 0;
	SDL_Texture *texid = SDL_CreateTexture(m_sdl_renderer, format, access, 16, 16);
	if (texid)
	{
		fmt_support[i].status = 1;
		SDL_DestroyTexture(texid);
		return 1;
	}
	osd_printf_verbose("Pixelformat <%s> error %s \n", sformat, SDL_GetError());
	osd_printf_verbose("Pixelformat <%s> not supported\n", sformat);
	fmt_support[i].status = 0;
	return 0;
}

//============================================================
//  drawsdl_init
//============================================================

void renderer_sdl2::add_list(copy_info_t **head, const copy_info_t *element, Uint32 bm)
{
	copy_info_t *newci = global_alloc(copy_info_t);
	*newci = *element;

	newci->bm_mask = bm;
	newci->next = *head;
	*head = newci;
}

void renderer_sdl2::expand_copy_info(const copy_info_t *list)
{
	for (const copy_info_t *bi = list; bi->src_fmt != -1; bi++)
	{
		if (bi->bm_mask == BM_ALL)
		{
			add_list(&s_blit_info[bi->src_fmt], bi, SDL_BLENDMODE_NONE);
			add_list(&s_blit_info[bi->src_fmt], bi, SDL_BLENDMODE_ADD);
			add_list(&s_blit_info[bi->src_fmt], bi, SDL_BLENDMODE_MOD);
			add_list(&s_blit_info[bi->src_fmt], bi, SDL_BLENDMODE_BLEND);
		}
		else
		{
			add_list(&s_blit_info[bi->src_fmt], bi, bi->bm_mask);
		}
	}
}

// FIXME: machine only used to access options.
void renderer_sdl2::init(running_machine &machine)
{
	osd_printf_verbose("Using SDL native texturing driver (SDL 2.0+)\n");

#if USE_OPENGL
	// Load the GL library now - else MT will fail
	const char *stemp = downcast<sdl_options &>(machine.options()).gl_lib();
#else
	const char *stemp = nullptr;
#endif
	if (stemp != nullptr && strcmp(stemp, OSDOPTVAL_AUTO) == 0)
		stemp = nullptr;

	// No fatalerror here since not all video drivers support GL !
	if (SDL_GL_LoadLibrary(stemp) != 0) // Load library (default for e==nullptr
		osd_printf_warning("Warning: Unable to load opengl library: %s\n", stemp ? stemp : "<default>");
	else
		osd_printf_verbose("Loaded opengl shared library: %s\n", stemp ? stemp : "<default>");
}


//============================================================
//  sdl_info::create
//============================================================

static void drawsdl_show_info(struct SDL_RendererInfo *render_info)
{
#define RF_ENTRY(x) {x, #x }
	static struct {
		int flag;
		const char *name;
	} rflist[] =
		{
#if 0
			RF_ENTRY(SDL_RENDERER_SINGLEBUFFER),
			RF_ENTRY(SDL_RENDERER_PRESENTCOPY),
			RF_ENTRY(SDL_RENDERER_PRESENTFLIP2),
			RF_ENTRY(SDL_RENDERER_PRESENTFLIP3),
			RF_ENTRY(SDL_RENDERER_PRESENTDISCARD),
#endif
			RF_ENTRY(SDL_RENDERER_SOFTWARE),
			RF_ENTRY(SDL_RENDERER_PRESENTVSYNC),
			RF_ENTRY(SDL_RENDERER_ACCELERATED),
			RF_ENTRY(SDL_RENDERER_TARGETTEXTURE),
			{-1, nullptr}
		};
	int i;

	osd_printf_verbose("window: using renderer %s\n", render_info->name ? render_info->name : "<unknown>");
	for (i = 0; rflist[i].name != nullptr; i++)
		if (render_info->flags & rflist[i].flag)
			osd_printf_verbose("renderer: flag %s\n", rflist[i].name);
}


int renderer_sdl2::create()
{
	// create renderer

	/* Enable bilinear filtering in case it is supported.
	 * This applies to all texture operations. However, artwort is pre-scaled
	 * and thus shouldn't be affected.
	 */
	if (video_config.filter)
	{
		SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
	}
	else
	{
		SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0");
	}

	auto win = assert_window();

	if (video_config.waitvsync)
		m_sdl_renderer = SDL_CreateRenderer(std::dynamic_pointer_cast<sdl_window_info>(win)->platform_window(), -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
	else
		m_sdl_renderer = SDL_CreateRenderer(std::dynamic_pointer_cast<sdl_window_info>(win)->platform_window(), -1, SDL_RENDERER_ACCELERATED);

	if (!m_sdl_renderer)
	{
		fatalerror("Error on creating renderer: %s\n", SDL_GetError());
	}

	//SDL_SelectRenderer(window().sdl_window);

	m_blittimer = 3;

	//SDL_RenderPresent(m_sdl_renderer);
	osd_printf_verbose("Leave renderer_sdl2::create\n");

	struct SDL_RendererInfo render_info;

	SDL_GetRendererInfo(m_sdl_renderer, &render_info);
	drawsdl_show_info(&render_info);

	return 0;
}


//============================================================
//  drawsdl_xy_to_render_target
//============================================================

int renderer_sdl2::xy_to_render_target(int x, int y, int *xt, int *yt)
{
	*xt = x - m_last_hofs;
	*yt = y - m_last_vofs;
	if (*xt<0 || *xt >= m_blit_dim.width())
		return 0;
	if (*yt<0 || *yt >= m_blit_dim.height())
		return 0;
	return 1;
}

//============================================================
//  drawsdl_destroy_all_textures
//============================================================

void renderer_sdl2::destroy_all_textures()
{
	auto win = assert_window();
	if (win == nullptr)
		return;

	if(win->m_primlist)
	{
		win->m_primlist->acquire_lock();
		m_texlist.reset();
		win->m_primlist->release_lock();
	}
	else
		m_texlist.reset();
}

//============================================================
//  sdl_info::draw
//============================================================

int renderer_sdl2::draw(int update)
{
	texture_info *texture=nullptr;
	float vofs, hofs;
	int blit_pixels = 0;

	if (video_config.novideo)
	{
		return 0;
	}

	auto win = assert_window();
	osd_dim wdim = win->get_size();

	if (has_flags(FI_CHANGED) || (wdim.width() != m_width) || (wdim.height() != m_height))
	{
		destroy_all_textures();
		m_width = wdim.width();
		m_height = wdim.height();
		SDL_RenderSetViewport(m_sdl_renderer, nullptr);
		m_blittimer = 3;
		clear_flags(FI_CHANGED);
	}

	//SDL_SelectRenderer(window().sdl_window);

	if (m_blittimer > 0)
	{
		/* SDL Underlays need alpha = 0 ! */
		SDL_SetRenderDrawBlendMode(m_sdl_renderer, SDL_BLENDMODE_NONE);
		//SDL_SetRenderDrawColor(0,0,0,255);
		SDL_SetRenderDrawColor(m_sdl_renderer, 0,0,0,0);
		SDL_RenderFillRect(m_sdl_renderer, nullptr);
		m_blittimer--;
	}

	// compute centering parameters
	vofs = hofs = 0.0f;

	if (video_config.centerv || video_config.centerh)
	{
		int ch, cw;

		ch = wdim.height();
		cw = wdim.width();

		if (video_config.centerv)
		{
			vofs = (ch - m_blit_dim.height()) / 2.0f;
		}
		if (video_config.centerh)
		{
			hofs = (cw - m_blit_dim.width()) / 2.0f;
		}
	}

	m_last_hofs = hofs;
	m_last_vofs = vofs;

	win->m_primlist->acquire_lock();

	// now draw
	for (render_primitive &prim : *win->m_primlist)
	{
		Uint8 sr, sg, sb, sa;

		switch (prim.type)
		{
			case render_primitive::LINE:
				sr = (int)(255.0f * prim.color.r);
				sg = (int)(255.0f * prim.color.g);
				sb = (int)(255.0f * prim.color.b);
				sa = (int)(255.0f * prim.color.a);

				SDL_SetRenderDrawBlendMode(m_sdl_renderer, map_blendmode(PRIMFLAG_GET_BLENDMODE(prim.flags)));
				SDL_SetRenderDrawColor(m_sdl_renderer, sr, sg, sb, sa);
				SDL_RenderDrawLine(m_sdl_renderer, prim.bounds.x0 + hofs, prim.bounds.y0 + vofs,
						prim.bounds.x1 + hofs, prim.bounds.y1 + vofs);
				break;
			case render_primitive::QUAD:
				texture = texture_update(prim);
				if (texture)
					blit_pixels += (texture->raw_height() * texture->raw_width());
				render_quad(texture, prim,
						round_nearest(hofs + prim.bounds.x0),
						round_nearest(vofs + prim.bounds.y0));
				break;
			default:
				throw emu_fatalerror("Unexpected render_primitive type\n");
		}
	}

	win->m_primlist->release_lock();

	m_last_blit_pixels = blit_pixels;
	m_last_blit_time = -osd_ticks();
	SDL_RenderPresent(m_sdl_renderer);
	m_last_blit_time += osd_ticks();

	return 0;
}


//============================================================
//  texture handling
//============================================================

//============================================================
//  texture_compute_size and type
//============================================================

copy_info_t *texture_info::compute_size_type()
{
	copy_info_t *result = nullptr;
	int maxperf = 0;

	for (copy_info_t *bi = renderer_sdl2::s_blit_info[m_format]; bi != nullptr; bi = bi->next)
	{
		if ((m_is_rotated == bi->blitter->m_is_rot)
				&& (m_sdl_blendmode == bi->bm_mask))
		{
			if (m_renderer->RendererSupportsFormat(bi->dst_fmt, m_sdl_access, bi->dstname))
			{
				int perf = bi->perf;
				if (perf == 0)
					return bi;
				else if (perf > (maxperf * 102) / 100)
				{
					result = bi;
					maxperf = perf;
				}
			}
		}
	}

	if (result)
		return result;

	/* try last resort handlers */
	for (copy_info_t *bi = renderer_sdl2::s_blit_info[m_format]; bi != nullptr; bi = bi->next)
	{
		if ((m_is_rotated == bi->blitter->m_is_rot)
			&& (m_sdl_blendmode == bi->bm_mask))
			if (m_renderer->RendererSupportsFormat(bi->dst_fmt, m_sdl_access, bi->dstname))
				return bi;
	}
	//FIXME: crash implement a -do nothing handler */
	return nullptr;
}

// FIXME:
bool texture_info::is_pixels_owned() const
{ // do we own / allocated it ?
	return ((m_sdl_access == SDL_TEXTUREACCESS_STATIC)
			&& (m_copyinfo->blitter->m_is_passthrough));
}

//============================================================
//  texture_info::matches
//============================================================

bool texture_info::matches(const render_primitive &prim, const quad_setup_data &setup)
{
	return  texinfo().base == prim.texture.base &&
			texinfo().width == prim.texture.width &&
			texinfo().height == prim.texture.height &&
			texinfo().rowpixels == prim.texture.rowpixels &&
			m_setup.dudx == setup.dudx &&
			m_setup.dvdx == setup.dvdx &&
			m_setup.dudy == setup.dudy &&
			m_setup.dvdy == setup.dvdy &&
			m_setup.startu == setup.startu &&
			m_setup.startv == setup.startv &&
			((flags() ^ prim.flags) & (PRIMFLAG_BLENDMODE_MASK | PRIMFLAG_TEXFORMAT_MASK)) == 0;
}

//============================================================
//  texture_create
//============================================================

texture_info::texture_info(renderer_sdl2 *renderer, const render_texinfo &texsource, const quad_setup_data &setup, uint32_t flags)
{
	// fill in the core data
	m_renderer = renderer;
	m_hash = texture_compute_hash(texsource, flags);
	m_flags = flags;
	m_texinfo = texsource;
	m_texinfo.seqid = -1; // force set data
	m_is_rotated = false;
	m_setup = setup;
	m_sdl_blendmode = map_blendmode(PRIMFLAG_GET_BLENDMODE(flags));
	m_pitch = 0;

	switch (PRIMFLAG_GET_TEXFORMAT(flags))
	{
		case TEXFORMAT_ARGB32:
			m_format = SDL_TEXFORMAT_ARGB32;
			break;
		case TEXFORMAT_RGB32:
			m_format = texsource.palette ? SDL_TEXFORMAT_RGB32_PALETTED : SDL_TEXFORMAT_RGB32;
			break;
		case TEXFORMAT_PALETTE16:
			m_format = SDL_TEXFORMAT_PALETTE16;
			break;
		case TEXFORMAT_PALETTEA16:
			m_format = SDL_TEXFORMAT_PALETTE16A;
			break;
		case TEXFORMAT_YUY16:
			m_format = texsource.palette ? SDL_TEXFORMAT_YUY16_PALETTED : SDL_TEXFORMAT_YUY16;
			break;

		default:
			osd_printf_error("Unknown textureformat %d\n", PRIMFLAG_GET_TEXFORMAT(flags));
	}

	if (setup.rotwidth != m_texinfo.width || setup.rotheight != m_texinfo.height
			|| setup.dudx < 0 || setup.dvdy < 0 || (PRIMFLAG_GET_TEXORIENT(flags) != 0))
		m_is_rotated = true;
	else
		m_is_rotated = false;

	//m_sdl_access = SDL_TEXTUREACCESS_STATIC;
	m_sdl_access = SDL_TEXTUREACCESS_STREAMING;

	// Watch out for 0x0 textures ...
	if (!m_setup.rotwidth || !m_setup.rotheight)
		osd_printf_warning("Trying to create texture with zero dim\n");

	// set copy_info

	m_copyinfo = compute_size_type();

	m_texture_id = SDL_CreateTexture(m_renderer->m_sdl_renderer, m_copyinfo->dst_fmt, m_sdl_access,
			m_setup.rotwidth, m_setup.rotheight);

	if (!m_texture_id)
		osd_printf_error("Error creating texture: %d x %d, pixelformat %s error: %s\n", m_setup.rotwidth, m_setup.rotheight,
				m_copyinfo->dstname, SDL_GetError());

	if (m_sdl_access == SDL_TEXTUREACCESS_STATIC)
	{
		if (m_copyinfo->blitter->m_is_passthrough)
			m_pixels = nullptr;
		else
			m_pixels = malloc(m_setup.rotwidth * m_setup.rotheight * m_copyinfo->blitter->m_dest_bpp);
	}
	m_last_access = osd_ticks();

}

texture_info::~texture_info()
{
	if ( is_pixels_owned() && (m_pixels != nullptr) )
		free(m_pixels);
	SDL_DestroyTexture(m_texture_id);
}

//============================================================
//  texture_set_data
//============================================================

void texture_info::set_data(const render_texinfo &texsource, const uint32_t flags)
{
	m_copyinfo->time -= osd_ticks();
	if (m_sdl_access == SDL_TEXTUREACCESS_STATIC)
	{
		if ( m_copyinfo->blitter->m_is_passthrough )
		{
			m_pixels = texsource.base;
			m_pitch = m_texinfo.rowpixels * m_copyinfo->blitter->m_dest_bpp;
		}
		else
		{
			m_pitch = m_setup.rotwidth * m_copyinfo->blitter->m_dest_bpp;
			m_copyinfo->blitter->texop(this, &texsource);
		}
		SDL_UpdateTexture(m_texture_id, nullptr, m_pixels, m_pitch);
	}
	else
	{
		SDL_LockTexture(m_texture_id, nullptr, (void **) &m_pixels, &m_pitch);
		if ( m_copyinfo->blitter->m_is_passthrough )
		{
			uint8_t *src = (uint8_t *) texsource.base;
			uint8_t *dst = (uint8_t *) m_pixels;
			int spitch = texsource.rowpixels * m_copyinfo->blitter->m_dest_bpp;
			int num = texsource.width * m_copyinfo->blitter->m_dest_bpp;
			int h = texsource.height;
			while (h--) {
				memcpy(dst, src, num);
				src += spitch;
				dst += m_pitch;
			}
		}
		else
			m_copyinfo->blitter->texop(this, &texsource);
		SDL_UnlockTexture(m_texture_id);
	}
	m_copyinfo->time += osd_ticks();
}

//============================================================
//  compute rotation setup
//============================================================

inline float signf(const float a)
{
	return (0.0f < a) - (a < 0.0f);
}

void quad_setup_data::compute(const render_primitive &prim, const int prescale)
{
	const render_quad_texuv *texcoords = &prim.texcoords;
	int texwidth = prim.texture.width;
	int texheight = prim.texture.height;
	float fdudx, fdvdx, fdudy, fdvdy;
	float width, height;
	float fscale;
	/* determine U/V deltas */
	if ((PRIMFLAG_GET_SCREENTEX(prim.flags)))
		fscale = (float) prescale;
	else
		fscale = 1.0f;

	fdudx = (texcoords->tr.u - texcoords->tl.u); // a a11
	fdvdx = (texcoords->tr.v - texcoords->tl.v); // c a21
	fdudy = (texcoords->bl.u - texcoords->tl.u); // b a12
	fdvdy = (texcoords->bl.v - texcoords->tl.v); // d a22

	width = fabsf(( fdudx * (float) (texwidth) + fdvdx * (float) (texheight)) ) * fscale;
	height = fabsf((fdudy * (float) (texwidth) + fdvdy * (float) (texheight)) ) * fscale;

	fdudx = signf(fdudx) / fscale;
	fdvdy = signf(fdvdy) / fscale;
	fdvdx = signf(fdvdx) / fscale;
	fdudy = signf(fdudy) / fscale;

#if 0
	printf("tl.u %f tl.v %f\n", texcoords->tl.u, texcoords->tl.v);
	printf("tr.u %f tr.v %f\n", texcoords->tr.u, texcoords->tr.v);
	printf("bl.u %f bl.v %f\n", texcoords->bl.u, texcoords->bl.v);
	printf("br.u %f br.v %f\n", texcoords->br.u, texcoords->br.v);
	/* compute start and delta U,V coordinates now */
#endif

	dudx = round_nearest(65536.0f * fdudx);
	dvdx = round_nearest(65536.0f * fdvdx);
	dudy = round_nearest(65536.0f * fdudy);
	dvdy = round_nearest(65536.0f * fdvdy);
	startu = round_nearest(65536.0f * (float) texwidth * texcoords->tl.u);
	startv = round_nearest(65536.0f * (float) texheight * texcoords->tl.v);

	/* clamp to integers */

	rotwidth = round_nearest(width);
	rotheight = round_nearest(height);

	//printf("%d %d rot %d %d\n", texwidth, texheight, rotwidth, rotheight);

	startu += (dudx + dudy) / 2;
	startv += (dvdx + dvdy) / 2;

}

//============================================================
//  texture_find
//============================================================

texture_info *renderer_sdl2::texture_find(const render_primitive &prim, const quad_setup_data &setup)
{
	HashT texhash = texture_compute_hash(prim.texture, prim.flags);
	texture_info *texture;
	osd_ticks_t now = osd_ticks();

	// find a match
	for (texture = m_texlist.first(); texture != nullptr; )
		if (texture->hash() == texhash &&
			texture->matches(prim, setup))
		{
			/* would we choose another blitter based on performance ? */
			if ((texture->m_copyinfo->samples & 0x7f) == 0x7f)
			{
				if (texture->m_copyinfo != texture->compute_size_type())
					return nullptr;
			}
			texture->m_last_access = now;
			return texture;
		}
		else
		{
			/* free resources not needed any longer? */
			texture_info *expire = texture;
			texture = texture->next();
			if (now - expire->m_last_access > osd_ticks_per_second())
				m_texlist.remove(*expire);
		}

	// nothing found
	return nullptr;
}

//============================================================
//  exit
//============================================================

void renderer_sdl2::exit()
{
	if (s_blit_info_initialized)
	{
		for (int i = 0; i <= SDL_TEXFORMAT_LAST; i++)
		{
			for (copy_info_t *bi = s_blit_info[i]; bi != nullptr; )
			{
				if (bi->pixel_count)
					osd_printf_verbose("%s -> %s %s blendmode 0x%02x, %d samples: %d KPixel/sec\n", bi->srcname, bi->dstname,
							bi->blitter->m_is_rot ? "rot" : "norot", bi->bm_mask, bi->samples,
							(int) bi->perf);
				copy_info_t *freeme = bi;
				bi = bi->next;
				global_free(freeme);
			}
			s_blit_info[i] = nullptr;
		}
		s_blit_info_initialized = false;
	}
}

//============================================================
//  texture_update
//============================================================

texture_info * renderer_sdl2::texture_update(const render_primitive &prim)
{
	quad_setup_data setup;
	texture_info *texture;

	auto win = assert_window();
	setup.compute(prim, win->prescale());

	texture = texture_find(prim, setup);

	// if we didn't find one, create a new texture
	if (texture == nullptr && prim.texture.base != nullptr)
	{
		texture = global_alloc(texture_info(this, prim.texture, setup, prim.flags));
		/* add us to the texture list */
		m_texlist.prepend(*texture);
	}

	if (texture != nullptr)
	{
		if (prim.texture.base != nullptr && texture->texinfo().seqid != prim.texture.seqid)
		{
			texture->texinfo().seqid = prim.texture.seqid;
			// if we found it, but with a different seqid, copy the data
			texture->set_data(prim.texture, prim.flags);
		}

	}
	return texture;
}

render_primitive_list *renderer_sdl2::get_primitives()
{
	auto win = assert_window();
	if (win == nullptr)
		return nullptr;

	osd_dim nd = win->get_size();
	if (nd != m_blit_dim)
	{
		m_blit_dim = nd;
		notify_changed();
	}
	win->target()->set_bounds(m_blit_dim.width(), m_blit_dim.height(), win->pixel_aspect());
	return &win->target()->get_primitives();
}
