// license:BSD-3-Clause
// copyright-holders:Couriersud, Olivier Galibert, R. Belmont
//============================================================
//
//  draw13.h - SDL 2.0 drawing implementation
//
//  SDLMAME by Olivier Galibert and R. Belmont
//
//============================================================

#pragma once

#ifndef __DRAW20__
#define __DRAW20__

// OSD headers
#ifndef OSD_WINDOWS
#include "osdsdl.h"
#include "window.h"
#else
#include "../windows/window.h"
typedef uint64_t HashT;
#endif

// standard SDL headers
#include <SDL2/SDL.h>

struct quad_setup_data
{
	quad_setup_data()
		: dudx(0)
		, dvdx(0)
		, dudy(0)
		, dvdy(0)
		, startu(0)
		, startv(0)
		, rotwidth(0)
		, rotheight(0)
	{
	}

	void compute(const render_primitive &prim, const int prescale);

	int32_t   dudx, dvdx, dudy, dvdy;
	int32_t   startu, startv;
	int32_t   rotwidth, rotheight;
};

//============================================================
//  Textures
//============================================================

class renderer_sdl2;
struct copy_info_t;

/* texture_info holds information about a texture */
class texture_info
{
	friend class simple_list<texture_info>;
public:
	texture_info(renderer_sdl2 *renderer, const render_texinfo &texsource, const quad_setup_data &setup, const uint32_t flags);
	~texture_info();

	void set_data(const render_texinfo &texsource, const uint32_t flags);
	void render_quad(const render_primitive &prim, const int x, const int y);
	bool matches(const render_primitive &prim, const quad_setup_data &setup);

	copy_info_t *compute_size_type();

	void                *m_pixels;            // pixels for the texture
	int                 m_pitch;

	copy_info_t         *m_copyinfo;
	quad_setup_data     m_setup;

	osd_ticks_t         m_last_access;

	int raw_width() const { return m_texinfo.width; }
	int raw_height() const { return m_texinfo.height; }

	texture_info *next() { return m_next; }
	const render_texinfo &texinfo() const { return m_texinfo; }
	render_texinfo &texinfo() { return m_texinfo; }

	HashT hash() const { return m_hash; }
	uint32_t flags() const { return m_flags; }
	// FIXME:
	bool is_pixels_owned() const;

private:
	void set_coloralphamode(SDL_Texture *texture_id, const render_color *color);

	Uint32              m_sdl_access;
	renderer_sdl2 *     m_renderer;
	render_texinfo      m_texinfo;            // copy of the texture info
	HashT               m_hash;               // hash value for the texture (must be >= pointer size)
	uint32_t              m_flags;              // rendering flags

	SDL_Texture *       m_texture_id;
	bool                m_is_rotated;

	int                 m_format;             // texture format
	SDL_BlendMode       m_sdl_blendmode;

	texture_info *      m_next;               // next texture in the list
};

//============================================================
//  TEXCOPY FUNCS
//============================================================

enum SDL_TEXFORMAT_E
{
	SDL_TEXFORMAT_ARGB32 = 0,
	SDL_TEXFORMAT_RGB32,
	SDL_TEXFORMAT_RGB32_PALETTED,
	SDL_TEXFORMAT_YUY16,
	SDL_TEXFORMAT_YUY16_PALETTED,
	SDL_TEXFORMAT_PALETTE16,
	SDL_TEXFORMAT_RGB15,
	SDL_TEXFORMAT_RGB15_PALETTED,
	SDL_TEXFORMAT_PALETTE16A,
	SDL_TEXFORMAT_PALETTE16_ARGB1555,
	SDL_TEXFORMAT_RGB15_ARGB1555,
	SDL_TEXFORMAT_RGB15_PALETTED_ARGB1555,
	SDL_TEXFORMAT_LAST = SDL_TEXFORMAT_RGB15_PALETTED_ARGB1555
};

#include "blit13.h"

struct copy_info_t
{
	int                 src_fmt;
	Uint32              dst_fmt;
	const blit_base     *blitter;
	Uint32              bm_mask;
	const char          *srcname;
	const char          *dstname;
	/* Statistics */
	uint64_t              pixel_count;
	int64_t               time;
	int                 samples;
	int                 perf;
	/* list */
	copy_info_t           *next;
};

/* sdl_info is the information about SDL for the current screen */
class renderer_sdl2 : public osd_renderer
{
public:
	renderer_sdl2(std::shared_ptr<osd_window> window, int extra_flags);

	virtual ~renderer_sdl2()
	{
		destroy_all_textures();
		SDL_DestroyRenderer(m_sdl_renderer);
		m_sdl_renderer = nullptr;
	}

	static void init(running_machine &machine);
	static void exit();

	virtual int create() override;
	virtual int draw(const int update) override;
	virtual int xy_to_render_target(const int x, const int y, int *xt, int *yt) override;
	virtual render_primitive_list *get_primitives() override;

	int RendererSupportsFormat(Uint32 format, Uint32 access, const char *sformat);

	SDL_Renderer *  m_sdl_renderer;

	static copy_info_t* s_blit_info[SDL_TEXFORMAT_LAST+1];

private:
	void expand_copy_info(const copy_info_t *list);
	void add_list(copy_info_t **head, const copy_info_t *element, Uint32 bm);

	void render_quad(texture_info *texture, const render_primitive &prim, const int x, const int y);

	texture_info *texture_find(const render_primitive &prim, const quad_setup_data &setup);
	texture_info *texture_update(const render_primitive &prim);

	void destroy_all_textures();

	int32_t           m_blittimer;


	simple_list<texture_info>  m_texlist;                // list of active textures

	float           m_last_hofs;
	float           m_last_vofs;

	int             m_width;
	int             m_height;

	osd_dim         m_blit_dim;

	struct
	{
		Uint32  format;
		int     status;
	} fmt_support[30];

	// Stats
	int64_t           m_last_blit_time;
	int64_t           m_last_blit_pixels;

	static bool s_blit_info_initialized;
	static const copy_info_t s_blit_info_default[];
};

#endif // __DRAW20__
