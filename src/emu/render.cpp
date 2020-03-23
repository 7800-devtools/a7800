// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    render.c

    Core rendering system.

****************************************************************************

    Windows-specific to-do:
        * no fallback if we run out of video memory

    Longer-term to do: (once old renderer is gone)
        * make vector updates asynchronous

****************************************************************************

    Overview of objects:

        render_target -- This represents a final rendering target. It
            is specified using integer width/height values, can have
            non-square pixels, and you can specify its rotation. It is
            what really determines the final rendering details. The OSD
            layer creates one or more of these to encapsulate the
            rendering process. Each render_target holds a list of
            layout_files that it can use for drawing. When rendering, it
            makes use of both layout_files and render_containers.

        render_container -- Containers are the top of a hierarchy that is
            not directly related to the objects above. Containers hold
            high level primitives that are generated at runtime by the
            video system. They are used currently for each screen and
            the user interface. These high-level primitives are broken down
            into low-level primitives at render time.

***************************************************************************/

#include "emu.h"
#include "emuopts.h"
#include "render.h"
#include "rendfont.h"
#include "rendlay.h"
#include "rendutil.h"
#include "config.h"
#include "drivenum.h"
#include "xmlfile.h"
#include "ui/uimain.h"
#include <zlib.h>



//**************************************************************************
//  CONSTANTS
//**************************************************************************

#define INTERNAL_FLAG_CHAR      0x00000001

enum
{
	COMPONENT_TYPE_IMAGE = 0,
	COMPONENT_TYPE_RECT,
	COMPONENT_TYPE_DISK,
	COMPONENT_TYPE_MAX
};


enum
{
	CONTAINER_ITEM_LINE = 0,
	CONTAINER_ITEM_QUAD,
	CONTAINER_ITEM_MAX
};



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// an object_transform is used to track transformations when building an object list
struct object_transform
{
	float               xoffs, yoffs;       // offset transforms
	float               xscale, yscale;     // scale transforms
	render_color        color;              // color transform
	int                 orientation;        // orientation transform
	bool                no_center;          // center the container?
};



//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

// precomputed UV coordinates for various orientations
static const render_quad_texuv oriented_texcoords[8] =
{
	{ { 0,0 }, { 1,0 }, { 0,1 }, { 1,1 } },     // 0
	{ { 1,0 }, { 0,0 }, { 1,1 }, { 0,1 } },     // ORIENTATION_FLIP_X
	{ { 0,1 }, { 1,1 }, { 0,0 }, { 1,0 } },     // ORIENTATION_FLIP_Y
	{ { 1,1 }, { 0,1 }, { 1,0 }, { 0,0 } },     // ORIENTATION_FLIP_X | ORIENTATION_FLIP_Y
	{ { 0,0 }, { 0,1 }, { 1,0 }, { 1,1 } },     // ORIENTATION_SWAP_XY
	{ { 0,1 }, { 0,0 }, { 1,1 }, { 1,0 } },     // ORIENTATION_SWAP_XY | ORIENTATION_FLIP_X
	{ { 1,0 }, { 1,1 }, { 0,0 }, { 0,1 } },     // ORIENTATION_SWAP_XY | ORIENTATION_FLIP_Y
	{ { 1,1 }, { 1,0 }, { 0,1 }, { 0,0 } }      // ORIENTATION_SWAP_XY | ORIENTATION_FLIP_X | ORIENTATION_FLIP_Y
};

// layer orders
static const int layer_order_standard[] = { ITEM_LAYER_SCREEN, ITEM_LAYER_OVERLAY, ITEM_LAYER_BACKDROP, ITEM_LAYER_BEZEL, ITEM_LAYER_CPANEL, ITEM_LAYER_MARQUEE };
static const int layer_order_alternate[] = { ITEM_LAYER_BACKDROP, ITEM_LAYER_SCREEN, ITEM_LAYER_OVERLAY, ITEM_LAYER_BEZEL, ITEM_LAYER_CPANEL, ITEM_LAYER_MARQUEE };



//**************************************************************************
//  INLINE FUNCTIONS
//**************************************************************************

//-------------------------------------------------
//  apply_orientation - apply orientation to a
//  set of bounds
//-------------------------------------------------

inline void apply_orientation(render_bounds &bounds, int orientation)
{
	// swap first
	if (orientation & ORIENTATION_SWAP_XY)
	{
		std::swap(bounds.x0, bounds.y0);
		std::swap(bounds.x1, bounds.y1);
	}

	// apply X flip
	if (orientation & ORIENTATION_FLIP_X)
	{
		bounds.x0 = 1.0f - bounds.x0;
		bounds.x1 = 1.0f - bounds.x1;
	}

	// apply Y flip
	if (orientation & ORIENTATION_FLIP_Y)
	{
		bounds.y0 = 1.0f - bounds.y0;
		bounds.y1 = 1.0f - bounds.y1;
	}
}


//-------------------------------------------------
//  normalize_bounds - normalize bounds so that
//  x0/y0 are less than x1/y1
//-------------------------------------------------

inline void normalize_bounds(render_bounds &bounds)
{
	if (bounds.x0 > bounds.x1)
		std::swap(bounds.x0, bounds.x1);
	if (bounds.y0 > bounds.y1)
		std::swap(bounds.y0, bounds.y1);
}


//-------------------------------------------------
//  get_layer_and_blendmode - return the
//  appropriate layer index and blendmode
//-------------------------------------------------

inline item_layer get_layer_and_blendmode(const layout_view &view, int index, int &blendmode)
{
	//  if we have multiple backdrop pieces and no overlays, render:
	//      backdrop (add) + screens (add) + bezels (alpha) + cpanels (alpha) + marquees (alpha)
	//  else render:
	//      screens (add) + overlays (RGB multiply) + backdrop (add) + bezels (alpha) + cpanels (alpha) + marquees (alpha)

	const int *layer_order = layer_order_standard;
	if (view.items(ITEM_LAYER_BACKDROP).count() > 1 && view.items(ITEM_LAYER_OVERLAY).empty())
		layer_order = layer_order_alternate;

	// select the layer
	int layer = layer_order[index];

	// pick a blendmode
	if (layer == ITEM_LAYER_SCREEN && layer_order == layer_order_standard)
		blendmode = -1;
	else if (layer == ITEM_LAYER_SCREEN || (layer == ITEM_LAYER_BACKDROP && layer_order == layer_order_standard))
		blendmode = BLENDMODE_ADD;
	else if (layer == ITEM_LAYER_OVERLAY)
		blendmode = BLENDMODE_RGB_MULTIPLY;
	else
		blendmode = BLENDMODE_ALPHA;

	return item_layer(layer);
}

//**************************************************************************
//  RENDER PRIMITIVE
//**************************************************************************

//-------------------------------------------------
//  reset - reset the state of a primitive after
//  it is re-allocated
//-------------------------------------------------

void render_primitive::reset()
{
	// do not clear m_next!
	memset(&type, 0, uintptr_t(&texcoords + 1) - uintptr_t(&type));
}



//**************************************************************************
//  RENDER PRIMITIVE LIST
//**************************************************************************

//-------------------------------------------------
//  render_primitive_list - constructor
//-------------------------------------------------

render_primitive_list::render_primitive_list()
{
}


//-------------------------------------------------
//  ~render_primitive_list - destructor
//-------------------------------------------------

render_primitive_list::~render_primitive_list()
{
	release_all();
}


//-------------------------------------------------
//  add_reference - add a new reference
//-------------------------------------------------

inline void render_primitive_list::add_reference(void *refptr)
{
	// skip if we already have one
	if (has_reference(refptr))
		return;

	// set the refptr and link us into the list
	reference *ref = m_reference_allocator.alloc();
	ref->m_refptr = refptr;
	m_reflist.append(*ref);
}


//-------------------------------------------------
//  has_reference - find a refptr in a reference
//  list
//-------------------------------------------------

inline bool render_primitive_list::has_reference(void *refptr) const
{
	// skip if we already have one
	for (reference &ref : m_reflist)
		if (ref.m_refptr == refptr)
			return true;
	return false;
}


//-------------------------------------------------
//  alloc - allocate a new empty primitive
//-------------------------------------------------

inline render_primitive *render_primitive_list::alloc(render_primitive::primitive_type type)
{
	render_primitive *result = m_primitive_allocator.alloc();
	result->reset();
	result->type = type;
	return result;
}


//-------------------------------------------------
//  release_all - release the contents of
//  a render list
//-------------------------------------------------

void render_primitive_list::release_all()
{
	// release all the live items while under the lock
	m_primitive_allocator.reclaim_all(m_primlist);
	m_reference_allocator.reclaim_all(m_reflist);
}


//-------------------------------------------------
//  append_or_return - append a primitive to the
//  end of the list, or return it to the free
//  list, based on a flag
//-------------------------------------------------

void render_primitive_list::append_or_return(render_primitive &prim, bool clipped)
{
	if (!clipped)
		m_primlist.append(prim);
	else
		m_primitive_allocator.reclaim(prim);
}



//**************************************************************************
//  RENDER TEXTURE
//**************************************************************************

//-------------------------------------------------
//  render_texture - constructor
//-------------------------------------------------

render_texture::render_texture()
	: m_manager(nullptr),
		m_next(nullptr),
		m_bitmap(nullptr),
		m_format(TEXFORMAT_ARGB32),
		m_osddata(~0L),
		m_scaler(nullptr),
		m_param(nullptr),
		m_curseq(0)
{
	m_sbounds.set(0, -1, 0, -1);
	memset(m_scaled, 0, sizeof(m_scaled));
}


//-------------------------------------------------
//  ~render_texture - destructor
//-------------------------------------------------

render_texture::~render_texture()
{
	release();
}


//-------------------------------------------------
//  reset - reset the state of a texture after
//  it has been re-allocated
//-------------------------------------------------

void render_texture::reset(render_manager &manager, texture_scaler_func scaler, void *param)
{
	m_manager = &manager;
	if (scaler != nullptr)
	{
		assert(m_format == TEXFORMAT_ARGB32);
		m_scaler = scaler;
		m_param = param;
	}
	m_osddata = ~0L;
}


//-------------------------------------------------
//  release - release resources when we are freed
//-------------------------------------------------

void render_texture::release()
{
	// free all scaled versions
	for (auto & elem : m_scaled)
	{
		m_manager->invalidate_all(elem.bitmap);
		global_free(elem.bitmap);
		elem.bitmap = nullptr;
		elem.seqid = 0;
	}

	// invalidate references to the original bitmap as well
	m_manager->invalidate_all(m_bitmap);
	m_bitmap = nullptr;
	m_sbounds.set(0, -1, 0, -1);
	m_format = TEXFORMAT_ARGB32;
	m_curseq = 0;
}


//-------------------------------------------------
//  set_bitmap - set a new source bitmap
//-------------------------------------------------

void render_texture::set_bitmap(bitmap_t &bitmap, const rectangle &sbounds, texture_format format)
{
	assert(bitmap.cliprect().contains(sbounds));

	// ensure we have a valid palette for palettized modes
	if (format == TEXFORMAT_PALETTE16 || format == TEXFORMAT_PALETTEA16)
		assert(bitmap.palette() != nullptr);

	// invalidate references to the old bitmap
	if (&bitmap != m_bitmap && m_bitmap != nullptr)
		m_manager->invalidate_all(m_bitmap);

	// set the new bitmap/palette
	m_bitmap = &bitmap;
	m_sbounds = sbounds;
	m_format = format;

	// invalidate all scaled versions
	for (auto & elem : m_scaled)
	{
		if (elem.bitmap != nullptr)
		{
			m_manager->invalidate_all(elem.bitmap);
			global_free(elem.bitmap);
		}
		elem.bitmap = nullptr;
		elem.seqid = 0;
	}
}


//-------------------------------------------------
//  hq_scale - generic high quality resampling
//  scaler
//-------------------------------------------------

void render_texture::hq_scale(bitmap_argb32 &dest, bitmap_argb32 &source, const rectangle &sbounds, void *param)
{
	render_color color = { 1.0f, 1.0f, 1.0f, 1.0f };
	bitmap_argb32 sourcesub(source, sbounds);
	render_resample_argb_bitmap_hq(dest, sourcesub, color);
}


//-------------------------------------------------
//  get_scaled - get a scaled bitmap (if we can)
//-------------------------------------------------

void render_texture::get_scaled(u32 dwidth, u32 dheight, render_texinfo &texinfo, render_primitive_list &primlist, u32 flags)
{
	// source width/height come from the source bounds
	int swidth = m_sbounds.width();
	int sheight = m_sbounds.height();

	// ensure height/width are non-zero
	if (dwidth < 1) dwidth = 1;
	if (dheight < 1) dheight = 1;

	texinfo.osddata = m_osddata;

	// are we scaler-free? if so, just return the source bitmap
	if (m_scaler == nullptr || (m_bitmap != nullptr && swidth == dwidth && sheight == dheight))
	{
		if (m_bitmap == nullptr) return;

		// add a reference and set up the source bitmap
		primlist.add_reference(m_bitmap);
		texinfo.base = m_bitmap->raw_pixptr(m_sbounds.min_y, m_sbounds.min_x);
		texinfo.rowpixels = m_bitmap->rowpixels();
		texinfo.width = swidth;
		texinfo.height = sheight;
		// palette will be set later
		texinfo.seqid = ++m_curseq;
	}
	else
	{
		// make sure we can recover the original argb32 bitmap
		bitmap_argb32 dummy;
		bitmap_argb32 &srcbitmap = (m_bitmap != nullptr) ? downcast<bitmap_argb32 &>(*m_bitmap) : dummy;

		// is it a size we already have?
		scaled_texture *scaled = nullptr;
		int scalenum;
		for (scalenum = 0; scalenum < ARRAY_LENGTH(m_scaled); scalenum++)
		{
			scaled = &m_scaled[scalenum];

			// we need a non-NULL bitmap with matching dest size
			if (scaled->bitmap != nullptr && dwidth == scaled->bitmap->width() && dheight == scaled->bitmap->height())
				break;
		}

		// did we get one?
		if (scalenum == ARRAY_LENGTH(m_scaled))
		{
			int lowest = -1;

			// didn't find one -- take the entry with the lowest seqnum
			for (scalenum = 0; scalenum < ARRAY_LENGTH(m_scaled); scalenum++)
				if ((lowest == -1 || m_scaled[scalenum].seqid < m_scaled[lowest].seqid) && !primlist.has_reference(m_scaled[scalenum].bitmap))
					lowest = scalenum;
			assert_always(lowest != -1, "Too many live texture instances!");

			// throw out any existing entries
			scaled = &m_scaled[lowest];
			if (scaled->bitmap != nullptr)
			{
				m_manager->invalidate_all(scaled->bitmap);
				global_free(scaled->bitmap);
			}

			// allocate a new bitmap
			scaled->bitmap = global_alloc(bitmap_argb32(dwidth, dheight));
			scaled->seqid = ++m_curseq;

			// let the scaler do the work
			(*m_scaler)(*scaled->bitmap, srcbitmap, m_sbounds, m_param);
		}

		// finally fill out the new info
		primlist.add_reference(scaled->bitmap);
		texinfo.base = &scaled->bitmap->pix32(0);
		texinfo.rowpixels = scaled->bitmap->rowpixels();
		texinfo.width = dwidth;
		texinfo.height = dheight;
		// palette will be set later
		texinfo.seqid = scaled->seqid;
	}
}


//-------------------------------------------------
//  get_adjusted_palette - return the adjusted
//  palette for a texture
//-------------------------------------------------

const rgb_t *render_texture::get_adjusted_palette(render_container &container)
{
	// override the palette with our adjusted palette
	switch (m_format)
	{
		case TEXFORMAT_PALETTE16:
		case TEXFORMAT_PALETTEA16:

			assert(m_bitmap->palette() != nullptr);

			// return our adjusted palette
			return container.bcg_lookup_table(m_format, m_bitmap->palette());

		case TEXFORMAT_RGB32:
		case TEXFORMAT_ARGB32:
		case TEXFORMAT_YUY16:

			// if no adjustment necessary, return nullptr
			if (!container.has_brightness_contrast_gamma_changes())
				return nullptr;
			return container.bcg_lookup_table(m_format);

		default:
			assert(false);
	}

	return nullptr;
}



//**************************************************************************
//  RENDER CONTAINER
//**************************************************************************

//-------------------------------------------------
//  render_container - constructor
//-------------------------------------------------

render_container::render_container(render_manager &manager, screen_device *screen)
	: m_next(nullptr),
		m_manager(manager),
		m_screen(screen),
		m_overlaybitmap(nullptr),
		m_overlaytexture(nullptr)
{
	// make sure it is empty
	empty();

	// if we have a screen, read and apply the options
	if (m_screen != nullptr)
	{
		// set the initial orientation and brightness/contrast/gamma
		m_user.m_orientation = manager.machine().system().flags & ORIENTATION_MASK;
		m_user.m_brightness = manager.machine().options().brightness();
		m_user.m_contrast = manager.machine().options().contrast();
		m_user.m_gamma = manager.machine().options().gamma();
		// palette client will be allocated later
	}

	recompute_lookups();
}


//-------------------------------------------------
//  ~render_container - destructor
//-------------------------------------------------

render_container::~render_container()
{
	// free all the container items
	empty();

	// free the overlay texture
	m_manager.texture_free(m_overlaytexture);
}


//-------------------------------------------------
//  set_overlay - set the overlay bitmap for the
//  container
//-------------------------------------------------

void render_container::set_overlay(bitmap_argb32 *bitmap)
{
	// free any existing texture
	m_manager.texture_free(m_overlaytexture);

	// set the new data and allocate the texture
	m_overlaybitmap = bitmap;
	if (m_overlaybitmap != nullptr)
	{
		m_overlaytexture = m_manager.texture_alloc(render_container::overlay_scale);
		m_overlaytexture->set_bitmap(*bitmap, bitmap->cliprect(), TEXFORMAT_ARGB32);
	}
}


//-------------------------------------------------
//  set_user_settings - set the current user
//  settings for a container
//-------------------------------------------------

void render_container::set_user_settings(const user_settings &settings)
{
	m_user = settings;
	recompute_lookups();
}


//-------------------------------------------------
//  add_line - add a line item to this container
//-------------------------------------------------

void render_container::add_line(float x0, float y0, float x1, float y1, float width, rgb_t argb, u32 flags)
{
	item &newitem = add_generic(CONTAINER_ITEM_LINE, x0, y0, x1, y1, argb);
	newitem.m_width = width;
	newitem.m_flags = flags;
}


//-------------------------------------------------
//  add_quad - add a quad item to this container
//-------------------------------------------------

void render_container::add_quad(float x0, float y0, float x1, float y1, rgb_t argb, render_texture *texture, u32 flags)
{
	item &newitem = add_generic(CONTAINER_ITEM_QUAD, x0, y0, x1, y1, argb);
	newitem.m_texture = texture;
	newitem.m_flags = flags;
}


//-------------------------------------------------
//  add_char - add a char item to this container
//-------------------------------------------------

void render_container::add_char(float x0, float y0, float height, float aspect, rgb_t argb, render_font &font, u16 ch)
{
	// compute the bounds of the character cell and get the texture
	render_bounds bounds;
	bounds.x0 = x0;
	bounds.y0 = y0;
	render_texture *texture = font.get_char_texture_and_bounds(height, aspect, ch, bounds);

	// add it like a quad
	item &newitem = add_generic(CONTAINER_ITEM_QUAD, bounds.x0, bounds.y0, bounds.x1, bounds.y1, argb);
	newitem.m_texture = texture;
	newitem.m_flags = PRIMFLAG_TEXORIENT(ROT0) | PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA) | PRIMFLAG_PACKABLE;
	newitem.m_internal = INTERNAL_FLAG_CHAR;
}


//-------------------------------------------------
//  apply_brightness_contrast_gamma - apply the
//  container's brightess, contrast, and gamma to
//  an 8-bit value
//-------------------------------------------------

u8 render_container::apply_brightness_contrast_gamma(u8 value)
{
	return ::apply_brightness_contrast_gamma(value, m_user.m_brightness, m_user.m_contrast, m_user.m_gamma);
}


//-------------------------------------------------
//  apply_brightness_contrast_gamma_fp - apply the
//  container's brightess, contrast, and gamma to
//  a floating-point value
//-------------------------------------------------

float render_container::apply_brightness_contrast_gamma_fp(float value)
{
	return ::apply_brightness_contrast_gamma_fp(value, m_user.m_brightness, m_user.m_contrast, m_user.m_gamma);
}


//-------------------------------------------------
//  bcg_lookup_table - return the appropriate
//  brightness/contrast/gamma lookup table for a
//  given texture mode
//-------------------------------------------------

const rgb_t *render_container::bcg_lookup_table(int texformat, palette_t *palette)
{
	switch (texformat)
	{
		case TEXFORMAT_PALETTE16:
		case TEXFORMAT_PALETTEA16:
			if (m_palclient == nullptr) // if adjusted palette hasn't been created yet, create it
			{
				m_palclient = std::make_unique<palette_client>(*palette);
				m_bcglookup.resize(palette->max_index());
				recompute_lookups();
			}
			assert (palette == &m_palclient->palette());
			return &m_bcglookup[0];

		case TEXFORMAT_RGB32:
		case TEXFORMAT_ARGB32:
		case TEXFORMAT_YUY16:
			return m_bcglookup256;

		default:
			return nullptr;
	}
}


//-------------------------------------------------
//  overlay_scale - scaler for an overlay
//-------------------------------------------------

void render_container::overlay_scale(bitmap_argb32 &dest, bitmap_argb32 &source, const rectangle &sbounds, void *param)
{
	// simply replicate the source bitmap over the target
	for (int y = 0; y < dest.height(); y++)
	{
		u32 *src = &source.pix32(y % source.height());
		u32 *dst = &dest.pix32(y);
		int sx = 0;

		// loop over columns
		for (int x = 0; x < dest.width(); x++)
		{
			*dst++ = src[sx++];
			if (sx >= source.width())
				sx = 0;
		}
	}
}


//-------------------------------------------------
//  add_generic - add a generic item to a
//  container
//-------------------------------------------------

render_container::item &render_container::add_generic(u8 type, float x0, float y0, float x1, float y1, rgb_t argb)
{
	item *newitem = m_item_allocator.alloc();

	assert(x0 == x0);
	assert(x1 == x1);
	assert(y0 == y0);
	assert(y1 == y1);

	// copy the data into the new item
	newitem->m_type = type;
	newitem->m_bounds.x0 = x0;
	newitem->m_bounds.y0 = y0;
	newitem->m_bounds.x1 = x1;
	newitem->m_bounds.y1 = y1;
	newitem->m_color.r = (float)argb.r() * (1.0f / 255.0f);
	newitem->m_color.g = (float)argb.g() * (1.0f / 255.0f);
	newitem->m_color.b = (float)argb.b() * (1.0f / 255.0f);
	newitem->m_color.a = (float)argb.a() * (1.0f / 255.0f);
	newitem->m_flags = 0;
	newitem->m_internal = 0;
	newitem->m_width = 0;
	newitem->m_texture = nullptr;

	// add the item to the container
	return m_itemlist.append(*newitem);
}


//-------------------------------------------------
//  recompute_lookups - recompute the lookup table
//  for the render container
//-------------------------------------------------

void render_container::recompute_lookups()
{
	// recompute the 256 entry lookup table
	for (int i = 0; i < 0x100; i++)
	{
		u8 adjustedval = apply_brightness_contrast_gamma(i);
		m_bcglookup256[i + 0x000] = adjustedval << 0;
		m_bcglookup256[i + 0x100] = adjustedval << 8;
		m_bcglookup256[i + 0x200] = adjustedval << 16;
		m_bcglookup256[i + 0x300] = adjustedval << 24;
	}

	// recompute the palette entries
	if (m_palclient != nullptr)
	{
		palette_t &palette = m_palclient->palette();
		const rgb_t *adjusted_palette = palette.entry_list_adjusted();
		int colors = palette.max_index();

		if (has_brightness_contrast_gamma_changes())
		{
			for (int i = 0; i < colors; i++)
			{
				rgb_t newval = adjusted_palette[i];
				m_bcglookup[i] = (newval & 0xff000000) |
										m_bcglookup256[0x200 + newval.r()] |
										m_bcglookup256[0x100 + newval.g()] |
										m_bcglookup256[0x000 + newval.b()];
			}
		}
		else
			memcpy(&m_bcglookup[0], adjusted_palette, colors * sizeof(rgb_t));
	}
}


//-------------------------------------------------
//  update_palette - update any dirty palette
//  entries
//-------------------------------------------------

void render_container::update_palette()
{
	// skip if no client
	if (m_palclient == nullptr)
		return;

	// get the dirty list
	u32 mindirty, maxdirty;
	const u32 *dirty = m_palclient->dirty_list(mindirty, maxdirty);

	// iterate over dirty items and update them
	if (dirty != nullptr)
	{
		palette_t &palette = m_palclient->palette();
		const rgb_t *adjusted_palette = palette.entry_list_adjusted();

		if (has_brightness_contrast_gamma_changes())
		{
			// loop over chunks of 32 entries, since we can quickly examine 32 at a time
			for (u32 entry32 = mindirty / 32; entry32 <= maxdirty / 32; entry32++)
			{
				u32 dirtybits = dirty[entry32];
				if (dirtybits != 0)

					// this chunk of 32 has dirty entries; fix them up
					for (u32 entry = 0; entry < 32; entry++)
						if (dirtybits & (1 << entry))
						{
							u32 finalentry = entry32 * 32 + entry;
							rgb_t newval = adjusted_palette[finalentry];
							m_bcglookup[finalentry] = (newval & 0xff000000) |
														m_bcglookup256[0x200 + newval.r()] |
														m_bcglookup256[0x100 + newval.g()] |
														m_bcglookup256[0x000 + newval.b()];
						}
			}
		}
		else
			memcpy(&m_bcglookup[mindirty], &adjusted_palette[mindirty], (maxdirty - mindirty + 1) * sizeof(rgb_t));
	}
}


//-------------------------------------------------
//  user_settings - constructor
//-------------------------------------------------

render_container::user_settings::user_settings()
	: m_orientation(0),
		m_brightness(1.0f),
		m_contrast(1.0f),
		m_gamma(1.0f),
		m_xscale(1.0f),
		m_yscale(1.0f),
		m_xoffset(0.0f),
		m_yoffset(0.0f)
{
}



//**************************************************************************
//  RENDER TARGET
//**************************************************************************

//-------------------------------------------------
//  render_target - constructor
//-------------------------------------------------

render_target::render_target(render_manager &manager, const internal_layout *layoutfile, u32 flags)
	: m_next(nullptr),
		m_manager(manager),
		m_curview(nullptr),
		m_flags(flags),
		m_listindex(0),
		m_width(640),
		m_height(480),
		m_pixel_aspect(0.0f),
		m_max_refresh(0),
		m_orientation(0),
		m_base_view(nullptr),
		m_base_orientation(ROT0),
		m_maxtexwidth(65536),
		m_maxtexheight(65536),
		m_transform_container(true)
{
	// determine the base layer configuration based on options
	m_base_layerconfig.set_backdrops_enabled(manager.machine().options().use_backdrops());
	m_base_layerconfig.set_overlays_enabled(manager.machine().options().use_overlays());
	m_base_layerconfig.set_bezels_enabled(manager.machine().options().use_bezels());
	m_base_layerconfig.set_cpanels_enabled(manager.machine().options().use_cpanels());
	m_base_layerconfig.set_marquees_enabled(manager.machine().options().use_marquees());
	m_base_layerconfig.set_zoom_to_screen(manager.machine().options().artwork_crop());

	// aspect and scale options
	m_keepaspect = (manager.machine().options().keep_aspect() && !(flags & RENDER_CREATE_HIDDEN));
	m_int_overscan = manager.machine().options().int_overscan();
	m_int_scale_x = manager.machine().options().int_scale_x();
	m_int_scale_y = manager.machine().options().int_scale_y();
	if (m_manager.machine().options().auto_stretch_xy())
		m_scale_mode = SCALE_FRACTIONAL_AUTO;
	else if (manager.machine().options().uneven_stretch_x())
		m_scale_mode = SCALE_FRACTIONAL_X;
	else if (manager.machine().options().uneven_stretch_y())
		m_scale_mode = SCALE_FRACTIONAL_Y;
	else if (manager.machine().options().uneven_stretch())
		m_scale_mode = SCALE_FRACTIONAL;
	else
		m_scale_mode = SCALE_INTEGER;

	// determine the base orientation based on options
	if (!manager.machine().options().rotate())
		m_base_orientation = orientation_reverse(manager.machine().system().flags & ORIENTATION_MASK);

	// rotate left/right
	if (manager.machine().options().ror() || (manager.machine().options().auto_ror() && (manager.machine().system().flags & ORIENTATION_SWAP_XY)))
		m_base_orientation = orientation_add(ROT90, m_base_orientation);
	if (manager.machine().options().rol() || (manager.machine().options().auto_rol() && (manager.machine().system().flags & ORIENTATION_SWAP_XY)))
		m_base_orientation = orientation_add(ROT270, m_base_orientation);

	// flip X/Y
	if (manager.machine().options().flipx())
		m_base_orientation ^= ORIENTATION_FLIP_X;
	if (manager.machine().options().flipy())
		m_base_orientation ^= ORIENTATION_FLIP_Y;

	// set the orientation and layerconfig equal to the base
	m_orientation = m_base_orientation;
	m_layerconfig = m_base_layerconfig;

	// load the layout files
	load_layout_files(layoutfile, flags & RENDER_CREATE_SINGLE_FILE);

	// set the current view to the first one
	set_view(0);

	// make us the UI target if there is none
	if (!hidden() && manager.m_ui_target == nullptr)
		manager.set_ui_target(*this);
}


//-------------------------------------------------
//  ~render_target - destructor
//-------------------------------------------------

render_target::~render_target()
{
}


//-------------------------------------------------
//  is_ui_target - return true if this is the
//  UI target
//-------------------------------------------------

bool render_target::is_ui_target() const
{
	return (this == &m_manager.ui_target());
}


//-------------------------------------------------
//  index - return the index of this target
//-------------------------------------------------

int render_target::index() const
{
	return m_manager.m_targetlist.indexof(*this);
}


//-------------------------------------------------
//  set_bounds - set the bounds and pixel aspect
//  of a target
//-------------------------------------------------

void render_target::set_bounds(s32 width, s32 height, float pixel_aspect)
{
	m_width = width;
	m_height = height;
	m_bounds.x0 = m_bounds.y0 = 0;
	m_bounds.x1 = (float)width;
	m_bounds.y1 = (float)height;
	m_pixel_aspect = pixel_aspect != 0.0? pixel_aspect : 1.0;
}


//-------------------------------------------------
//  set_view - dynamically change the view for
//  a target
//-------------------------------------------------

void render_target::set_view(int viewindex)
{
	layout_view *view = view_by_index(viewindex);
	if (view != nullptr)
	{
		m_curview = view;
		view->recompute(m_layerconfig);
	}
}


//-------------------------------------------------
//  set_max_texture_size - set the upper bound on
//  the texture size
//-------------------------------------------------

void render_target::set_max_texture_size(int maxwidth, int maxheight)
{
	m_maxtexwidth = maxwidth;
	m_maxtexheight = maxheight;
}


//-------------------------------------------------
//  configured_view - select a view for this
//  target based on the configuration parameters
//-------------------------------------------------

int render_target::configured_view(const char *viewname, int targetindex, int numtargets)
{
	layout_view *view = nullptr;
	int viewindex;

	// auto view just selects the nth view
	if (strcmp(viewname, "auto") != 0)
	{
		// scan for a matching view name
		size_t viewlen = strlen(viewname);
		for (view = view_by_index(viewindex = 0); view != nullptr; view = view_by_index(++viewindex))
			if (core_strnicmp(view->name(), viewname, viewlen) == 0)
				break;
	}

	// if we don't have a match, default to the nth view
	screen_device_iterator iter(m_manager.machine().root_device());
	int scrcount = iter.count();
	if (view == nullptr && scrcount > 0)
	{
		// if we have enough targets to be one per screen, assign in order
		if (numtargets >= scrcount)
		{
			int ourindex = index() % scrcount;
			screen_device *screen = iter.byindex(ourindex);

			// find the first view with this screen and this screen only
			for (view = view_by_index(viewindex = 0); view != nullptr; view = view_by_index(++viewindex))
			{
				const render_screen_list &viewscreens = view->screens();
				if (viewscreens.count() == 0)
				{
					view = nullptr;
					break;
				}
				else if (viewscreens.count() == viewscreens.contains(*screen))
					break;
			}
		}

		// otherwise, find the first view that has all the screens
		if (view == nullptr)
		{
			for (view = view_by_index(viewindex = 0); view != nullptr; view = view_by_index(++viewindex))
			{
				const render_screen_list &viewscreens = view->screens();
				if (viewscreens.count() == 0)
					break;
				if (viewscreens.count() >= scrcount)
				{
					bool has_screen = false;
					for (screen_device &screen : iter)
						if (!viewscreens.contains(screen))
						{
							has_screen = true;
							break;
						}
					if (!has_screen)
						break;
				}
			}
		}
	}

	// make sure it's a valid view
	return (view != nullptr) ? view_index(*view) : 0;
}


//-------------------------------------------------
//  view_name - return the name of the given view
//-------------------------------------------------

const char *render_target::view_name(int viewindex)
{
	layout_view *view = view_by_index(viewindex);
	return (view != nullptr) ? view->name() : nullptr;
}


//-------------------------------------------------
//  render_target_get_view_screens - return a
//  bitmask of which screens are visible on a
//  given view
//-------------------------------------------------

const render_screen_list &render_target::view_screens(int viewindex)
{
	layout_view *view = view_by_index(viewindex);
	return (view != nullptr) ? view->screens() : s_empty_screen_list;
}


//-------------------------------------------------
//  compute_visible_area - compute the visible
//  area for the given target with the current
//  layout and proposed new parameters
//-------------------------------------------------

void render_target::compute_visible_area(s32 target_width, s32 target_height, float target_pixel_aspect, int target_orientation, s32 &visible_width, s32 &visible_height)
{
	switch (m_scale_mode)
	{
		case SCALE_FRACTIONAL:
		{
			float width, height;
			float scale;

			// constrained case
			if (m_keepaspect)
			{
				// start with the aspect ratio of the square pixel layout
				width = m_curview->effective_aspect(m_layerconfig);
				height = 1.0f;

				// first apply target orientation
				if (target_orientation & ORIENTATION_SWAP_XY)
					std::swap(width, height);

				// apply the target pixel aspect ratio
				height *= target_pixel_aspect;

				// based on the height/width ratio of the source and target, compute the scale factor
				if (width / height > (float)target_width / (float)target_height)
					scale = (float)target_width / width;
				else
					scale = (float)target_height / height;
			}

			// stretch-to-fit case
			else
			{
				width = (float)target_width;
				height = (float)target_height;
				scale = 1.0f;
			}

			// set the final width/height
			visible_width = render_round_nearest(width * scale);
			visible_height = render_round_nearest(height * scale);
			break;
		}

		default:
		{
			// get source size and aspect
			s32 src_width, src_height;
			compute_minimum_size(src_width, src_height);
			float src_aspect = m_curview->effective_aspect(m_layerconfig);

			// apply orientation if required
			if (target_orientation & ORIENTATION_SWAP_XY)
				src_aspect = 1.0 / src_aspect;

			// get target aspect
			float target_aspect = (float)target_width / (float)target_height * target_pixel_aspect;
			bool target_is_portrait = (target_aspect < 1.0f);

			// apply automatic axial stretching if required
			int scale_mode = m_scale_mode;
			if (m_scale_mode == SCALE_FRACTIONAL_AUTO)
			{
				bool is_rotated = (m_manager.machine().system().flags & ORIENTATION_SWAP_XY) ^ (target_orientation & ORIENTATION_SWAP_XY);
				scale_mode = is_rotated ^ target_is_portrait ? SCALE_FRACTIONAL_Y : SCALE_FRACTIONAL_X;
			}

			// determine the scale mode for each axis
			bool x_is_integer = !((!target_is_portrait && scale_mode == SCALE_FRACTIONAL_X) || (target_is_portrait && scale_mode == SCALE_FRACTIONAL_Y));
			bool y_is_integer = !((target_is_portrait && scale_mode == SCALE_FRACTIONAL_X) || (!target_is_portrait && scale_mode == SCALE_FRACTIONAL_Y));

			// first compute scale factors to fit the screen
			float xscale = (float)target_width / src_width;
			float yscale = (float)target_height / src_height;
			float maxxscale = std::max(1.0f, float(m_int_overscan ? render_round_nearest(xscale) : floor(xscale)));
			float maxyscale = std::max(1.0f, float(m_int_overscan ? render_round_nearest(yscale) : floor(yscale)));

			// now apply desired scale mode and aspect correction
			if (m_keepaspect && target_aspect > src_aspect) xscale *= src_aspect / target_aspect * (maxyscale / yscale);
			if (m_keepaspect && target_aspect < src_aspect) yscale *= target_aspect / src_aspect * (maxxscale / xscale);
			if (x_is_integer) xscale = std::min(maxxscale, std::max(1.0f, render_round_nearest(xscale)));
			if (y_is_integer) yscale = std::min(maxyscale, std::max(1.0f, render_round_nearest(yscale)));

			// check if we have user defined scale factors, if so use them instead
			int user_scale_x = target_is_portrait? m_int_scale_y : m_int_scale_x;
			int user_scale_y = target_is_portrait? m_int_scale_x : m_int_scale_y;
			xscale = user_scale_x > 0 ? user_scale_x : xscale;
			yscale = user_scale_y > 0 ? user_scale_y : yscale;

			// set the final width/height
			visible_width = render_round_nearest(src_width * xscale);
			visible_height = render_round_nearest(src_height * yscale);
			break;
		}
	}
}


//-------------------------------------------------
//  compute_minimum_size - compute the "minimum"
//  size of a target, which is the smallest bounds
//  that will ensure at least 1 target pixel per
//  source pixel for all included screens
//-------------------------------------------------

void render_target::compute_minimum_size(s32 &minwidth, s32 &minheight)
{
	float maxxscale = 1.0f, maxyscale = 1.0f;
	int screens_considered = 0;

	// early exit in case we are called between device teardown and render teardown
	if (m_manager.machine().phase() == machine_phase::EXIT)
	{
		minwidth = 640;
		minheight = 480;
		return;
	}

	if (m_curview == nullptr)
		throw emu_fatalerror("Mandatory artwork is missing");

	// scan the current view for all screens
	for (item_layer layer = ITEM_LAYER_FIRST; layer < ITEM_LAYER_MAX; ++layer)

		// iterate over items in the layer
		for (layout_view::item &curitem : m_curview->items(layer))
			if (curitem.screen() != nullptr)
			{
				// use a hard-coded default visible area for vector screens
				screen_device *screen = curitem.screen();
				const rectangle vectorvis(0, 639, 0, 479);
				const rectangle &visarea = (screen->screen_type() == SCREEN_TYPE_VECTOR) ? vectorvis : screen->visible_area();

				// apply target orientation to the bounds
				render_bounds bounds = curitem.bounds();
				apply_orientation(bounds, m_orientation);
				normalize_bounds(bounds);

				// based on the orientation of the screen container, check the bitmap
				float xscale, yscale;
				if (!(orientation_add(m_orientation, screen->container().orientation()) & ORIENTATION_SWAP_XY))
				{
					xscale = float(visarea.width()) / bounds.width();
					yscale = float(visarea.height()) / bounds.height();
				}
				else
				{
					xscale = float(visarea.height()) / bounds.width();
					yscale = float(visarea.width()) / bounds.height();
				}

				// pick the greater
				maxxscale = std::max(xscale, maxxscale);
				maxyscale = std::max(yscale, maxyscale);
				screens_considered++;
			}

	// if there were no screens considered, pick a nominal default
	if (screens_considered == 0)
	{
		maxxscale = 640.0f;
		maxyscale = 480.0f;
	}

	// round up
	minwidth = render_round_nearest(maxxscale);
	minheight = render_round_nearest(maxyscale);
}


//-------------------------------------------------
//  get_primitives - return a list of primitives
//  for a given render target
//-------------------------------------------------

render_primitive_list &render_target::get_primitives()
{
	// remember the base values if this is the first frame
	if (m_base_view == nullptr)
		m_base_view = m_curview;

	// switch to the next primitive list
	render_primitive_list &list = m_primlist[m_listindex];
	m_listindex = (m_listindex + 1) % ARRAY_LENGTH(m_primlist);
	list.acquire_lock();

	// free any previous primitives
	list.release_all();

	// compute the visible width/height
	s32 viswidth, visheight;
	compute_visible_area(m_width, m_height, m_pixel_aspect, m_orientation, viswidth, visheight);

	// create a root transform for the target
	object_transform root_xform;
	root_xform.xoffs = (float)(m_width - viswidth) / 2;
	root_xform.yoffs = (float)(m_height - visheight) / 2;
	root_xform.xscale = (float)viswidth;
	root_xform.yscale = (float)visheight;
	root_xform.color.r = root_xform.color.g = root_xform.color.b = root_xform.color.a = 1.0f;
	root_xform.orientation = m_orientation;
	root_xform.no_center = false;

	// iterate over layers back-to-front, but only if we're running
	if (m_manager.machine().phase() >= machine_phase::RESET)
		for (item_layer layernum = ITEM_LAYER_FIRST; layernum < ITEM_LAYER_MAX; ++layernum)
		{
			int blendmode;
			item_layer layer = get_layer_and_blendmode(*m_curview, layernum, blendmode);
			if (m_curview->layer_enabled(layer))
			{
				// iterate over items in the layer
				for (layout_view::item &curitem : m_curview->items(layer))
				{
					// first apply orientation to the bounds
					render_bounds bounds = curitem.bounds();
					apply_orientation(bounds, root_xform.orientation);
					normalize_bounds(bounds);

					// apply the transform to the item
					object_transform item_xform;
					item_xform.xoffs = root_xform.xoffs + bounds.x0 * root_xform.xscale;
					item_xform.yoffs = root_xform.yoffs + bounds.y0 * root_xform.yscale;
					item_xform.xscale = (bounds.x1 - bounds.x0) * root_xform.xscale;
					item_xform.yscale = (bounds.y1 - bounds.y0) * root_xform.yscale;
					item_xform.color.r = curitem.color().r * root_xform.color.r;
					item_xform.color.g = curitem.color().g * root_xform.color.g;
					item_xform.color.b = curitem.color().b * root_xform.color.b;
					item_xform.color.a = curitem.color().a * root_xform.color.a;
					item_xform.orientation = orientation_add(curitem.orientation(), root_xform.orientation);
					item_xform.no_center = false;

					// if there is no associated element, it must be a screen element
					if (curitem.screen() != nullptr)
						add_container_primitives(list, root_xform, item_xform, curitem.screen()->container(), blendmode);
					else
						add_element_primitives(list, item_xform, *curitem.element(), curitem.state(), blendmode);
				}
			}
		}

	// if we are not in the running stage, draw an outer box
	else
	{
		render_primitive *prim = list.alloc(render_primitive::QUAD);
		set_render_bounds_xy(&prim->bounds, 0.0f, 0.0f, (float)m_width, (float)m_height);
		prim->full_bounds = prim->bounds;
		set_render_color(&prim->color, 1.0f, 1.0f, 1.0f, 1.0f);
		prim->texture.base = nullptr;
		prim->flags = PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA);
		list.append(*prim);

		if (m_width > 1 && m_height > 1)
		{
			prim = list.alloc(render_primitive::QUAD);
			set_render_bounds_xy(&prim->bounds, 1.0f, 1.0f, (float)(m_width - 1), (float)(m_height - 1));
			prim->full_bounds = prim->bounds;
			set_render_color(&prim->color, 1.0f, 0.0f, 0.0f, 0.0f);
			prim->texture.base = nullptr;
			prim->flags = PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA);
			list.append(*prim);
		}
	}

	// process the debug containers
	for (render_container &debug : m_debug_containers)
	{
		object_transform ui_xform;
		ui_xform.xoffs = 0;
		ui_xform.yoffs = 0;
		ui_xform.xscale = (float)m_width;
		ui_xform.yscale = (float)m_height;
		ui_xform.color.r = ui_xform.color.g = ui_xform.color.b = 1.0f;
		ui_xform.color.a = 0.9f;
		ui_xform.orientation = m_orientation;
		ui_xform.no_center = true;

		// add UI elements
		add_container_primitives(list, root_xform, ui_xform, debug, BLENDMODE_ALPHA);
	}

	// process the UI if we are the UI target
	if (is_ui_target())
	{
		// compute the transform for the UI
		object_transform ui_xform;
		ui_xform.xoffs = 0;
		ui_xform.yoffs = 0;
		ui_xform.xscale = (float) m_width;
		ui_xform.yscale = (float) m_height;
		ui_xform.color.r = ui_xform.color.g = ui_xform.color.b = ui_xform.color.a = 1.0f;
		ui_xform.orientation = m_orientation;
		ui_xform.no_center = false;

		// add UI elements
		add_container_primitives(list, root_xform, ui_xform, m_manager.ui_container(), BLENDMODE_ALPHA);
	}

	// optimize the list before handing it off
	add_clear_and_optimize_primitive_list(list);
	list.release_lock();
	return list;
}


//-------------------------------------------------
//  map_point_container - attempts to map a point
//  on the specified render_target to the
//  specified container, if possible
//-------------------------------------------------

bool render_target::map_point_container(s32 target_x, s32 target_y, render_container &container, float &container_x, float &container_y)
{
	ioport_port *input_port;
	ioport_value input_mask;
	return map_point_internal(target_x, target_y, &container, container_x, container_y, input_port, input_mask);
}


//-------------------------------------------------
//  map_point_input - attempts to map a point on
//  the specified render_target to the specified
//  container, if possible
//-------------------------------------------------

bool render_target::map_point_input(s32 target_x, s32 target_y, ioport_port *&input_port, ioport_value &input_mask, float &input_x, float &input_y)
{
	return map_point_internal(target_x, target_y, nullptr, input_x, input_y, input_port, input_mask);;
}


//-------------------------------------------------
//  invalidate_all - if any of our primitive lists
//  contain a reference to the given pointer,
//  clear them
//-------------------------------------------------

void render_target::invalidate_all(void *refptr)
{
	// iterate through all our primitive lists
	for (auto & list : m_primlist)
	{
		// if we have a reference to this object, release our list
		list.acquire_lock();
		if (list.has_reference(refptr))
			list.release_all();
		list.release_lock();
	}
}


//-------------------------------------------------
//  debug_alloc - allocate a container for a debug
//  view
//-------------------------------------------------

render_container *render_target::debug_alloc()
{
	return &m_debug_containers.append(*m_manager.container_alloc());
}


//-------------------------------------------------
//  debug_free - free a container for a debug view
//-------------------------------------------------

void render_target::debug_free(render_container &container)
{
	m_debug_containers.remove(container);
}


//-------------------------------------------------
//  debug_append - move a debug view container to
//  the end of the list
//-------------------------------------------------

void render_target::debug_append(render_container &container)
{
	m_debug_containers.append(m_debug_containers.detach(container));
}


//-------------------------------------------------
//  resolve_tags - resolve tag lookups
//-------------------------------------------------

void render_target::resolve_tags()
{
	for (layout_file &file : m_filelist)
	{
		for (layout_view &view : file.views())
		{
			view.resolve_tags();
		}
	}
}


//-------------------------------------------------
//  update_layer_config - recompute after a layer
//  config change
//-------------------------------------------------

void render_target::update_layer_config()
{
	m_curview->recompute(m_layerconfig);
}


//-------------------------------------------------
//  load_layout_files - load layout files for a
//  given render target
//-------------------------------------------------

void render_target::load_layout_files(const internal_layout *layoutfile, bool singlefile)
{
	bool have_default = false;
	// if there's an explicit file, load that first
	const char *basename = m_manager.machine().basename();
	if (layoutfile != nullptr)
		have_default |= load_layout_file(basename, layoutfile);

	// if we're only loading this file, we know our final result
	if (singlefile)
		return;

	// try to load a file based on the driver name
	const game_driver &system = m_manager.machine().system();
	if (!load_layout_file(basename, system.name))
		have_default |= load_layout_file(basename, "default");
	else
		have_default |= true;

	// if a default view has been specified, use that as a fallback
	if (system.default_layout != nullptr)
		have_default |= load_layout_file(nullptr, system.default_layout);
	if (m_manager.machine().config().m_default_layout != nullptr)
		have_default |= load_layout_file(nullptr, m_manager.machine().config().m_default_layout);

	// try to load another file based on the parent driver name
	int cloneof = driver_list::clone(system);
	if (cloneof != -1) {
		if (!load_layout_file(driver_list::driver(cloneof).name, driver_list::driver(cloneof).name))
			have_default |= load_layout_file(driver_list::driver(cloneof).name, "default");
		else
			have_default |= true;
	}
	screen_device_iterator iter(m_manager.machine().root_device());
	int screens = iter.count();
	// now do the built-in layouts for single-screen games
	if (screens == 1)
	{
		if (system.flags & ORIENTATION_SWAP_XY)
			load_layout_file(nullptr, &layout_vertical);
		else
			load_layout_file(nullptr, &layout_horizont);
		assert_always(m_filelist.count() > 0, "Couldn't parse default layout??");
	}
	if (!have_default)
	{
		if (screens == 0)
		{
			load_layout_file(nullptr, &layout_noscreens);
			assert_always(m_filelist.count() > 0, "Couldn't parse default layout??");
		}
		else
		if (screens == 2)
		{
			load_layout_file(nullptr, &layout_dualhsxs);
			assert_always(m_filelist.count() > 0, "Couldn't parse default layout??");
		}
		else
		if (screens == 3)
		{
			load_layout_file(nullptr, &layout_triphsxs);
			assert_always(m_filelist.count() > 0, "Couldn't parse default layout??");
		}
		else
		if (screens == 4)
		{
			load_layout_file(nullptr, &layout_quadhsxs);
			assert_always(m_filelist.count() > 0, "Couldn't parse default layout??");
		}
	}
}


//-------------------------------------------------
//  load_layout_file - load a single layout file
//  and append it to our list
//-------------------------------------------------


bool render_target::load_layout_file(const char *dirname, const internal_layout *layout_data)
{
	// +1 to ensure data is terminated for XML parser
	auto tempout = make_unique_clear<u8[]>(layout_data->decompressed_size+1);

	z_stream stream;
	int zerr;

	/* initialize the stream */
	memset(&stream, 0, sizeof(stream));
	stream.next_out = tempout.get();
	stream.avail_out = layout_data->decompressed_size;


	zerr = inflateInit(&stream);
	if (zerr != Z_OK)
	{
		fatalerror("could not inflateInit");
		return false;
	}

	/* decompress this chunk */
	stream.next_in = (unsigned char*)layout_data->data;
	stream.avail_in = layout_data->compressed_size;
	zerr = inflate(&stream, Z_NO_FLUSH);

	/* stop at the end of the stream */
	if (zerr == Z_STREAM_END)
	{
		// OK
	}
	else if (zerr != Z_OK)
	{
		fatalerror("decompression error\n");
		return false;
	}

	/* clean up */
	zerr = inflateEnd(&stream);
	if (zerr != Z_OK)
	{
		fatalerror("inflateEnd error\n");
		return false;
	}

	return load_layout_file(dirname, (const char*)tempout.get());
}

bool render_target::load_layout_file(const char *dirname, const char *filename)
{
	util::xml::file::ptr rootnode;
	if (filename[0] == '<')
	{
		// if the first character of the "file" is an open brace, assume it is an XML string
		rootnode = util::xml::file::string_read(filename, nullptr);
	}
	else
	{
		// otherwise, assume it is a file

		// build the path and optionally prepend the directory
		std::string fname = std::string(filename).append(".lay");
		if (dirname != nullptr)
			fname.insert(0, PATH_SEPARATOR).insert(0, dirname);

		// attempt to open the file; bail if we can't
		emu_file layoutfile(m_manager.machine().options().art_path(), OPEN_FLAG_READ);
		osd_file::error filerr = layoutfile.open(fname.c_str());
		if (filerr != osd_file::error::NONE)
			return false;

		// read the file
		rootnode = util::xml::file::read(layoutfile, nullptr);
	}

	// if we didn't get a properly-formatted XML file, record a warning and exit
	if (rootnode == nullptr)
	{
		if (filename[0] != '<')
			osd_printf_warning("Improperly formatted XML file '%s', ignoring\n", filename);
		else
			osd_printf_warning("Improperly formatted XML string, ignoring\n");
		return false;
	}

	// parse and catch any errors
	bool result = true;
	try
	{
		m_filelist.append(*global_alloc(layout_file(m_manager.machine(), *rootnode, dirname)));
	}
	catch (emu_fatalerror &err)
	{
		if (filename[0] != '<')
			osd_printf_warning("Error in XML file '%s': %s\n", filename, err.string());
		else
			osd_printf_warning("Error in XML string: %s\n", err.string());
		result = false;
	}

	emulator_info::layout_file_cb(*rootnode);

	// free the root node
	return result;
}


//-------------------------------------------------
//  add_container_primitives - add primitives
//  based on the container
//-------------------------------------------------

void render_target::add_container_primitives(render_primitive_list &list, const object_transform &root_xform, const object_transform &xform, render_container &container, int blendmode)
{
	// first update the palette for the container, if it is dirty
	container.update_palette();

	// compute the clip rect
	render_bounds cliprect;
	cliprect.x0 = xform.xoffs;
	cliprect.y0 = xform.yoffs;
	cliprect.x1 = xform.xoffs + xform.xscale;
	cliprect.y1 = xform.yoffs + xform.yscale;
	sect_render_bounds(&cliprect, &m_bounds);

	float root_xoffs = root_xform.xoffs + fabsf(root_xform.xscale - xform.xscale) * 0.5f;
	float root_yoffs = root_xform.yoffs + fabsf(root_xform.yscale - xform.yscale) * 0.5f;

	render_bounds root_cliprect;
	root_cliprect.x0 = root_xoffs;
	root_cliprect.y0 = root_yoffs;
	root_cliprect.x1 = root_xoffs + root_xform.xscale;
	root_cliprect.y1 = root_yoffs + root_xform.yscale;
	sect_render_bounds(&root_cliprect, &m_bounds);

	// compute the container transform
	object_transform container_xform;
	container_xform.orientation = orientation_add(container.orientation(), xform.orientation);
	{
		float xscale = (container_xform.orientation & ORIENTATION_SWAP_XY) ? container.yscale() : container.xscale();
		float yscale = (container_xform.orientation & ORIENTATION_SWAP_XY) ? container.xscale() : container.yscale();
		float xoffs = (container_xform.orientation & ORIENTATION_SWAP_XY) ? container.yoffset() : container.xoffset();
		float yoffs = (container_xform.orientation & ORIENTATION_SWAP_XY) ? container.xoffset() : container.yoffset();
		if (container_xform.orientation & ORIENTATION_FLIP_X) xoffs = -xoffs;
		if (container_xform.orientation & ORIENTATION_FLIP_Y) yoffs = -yoffs;
		if (!m_transform_container)
		{
			xscale = 1.0f;
			yscale = 1.0f;
			xoffs = 0.0f;
			yoffs = 0.0f;
		}
		container_xform.xscale = xform.xscale * xscale;
		container_xform.yscale = xform.yscale * yscale;
		if (xform.no_center)
		{
			container_xform.xoffs = xform.xscale * (xoffs) + xform.xoffs;
			container_xform.yoffs = xform.yscale * (yoffs) + xform.yoffs;
		}
		else
		{
			container_xform.xoffs = xform.xscale * (0.5f - 0.5f * xscale + xoffs) + xform.xoffs;
			container_xform.yoffs = xform.yscale * (0.5f - 0.5f * yscale + yoffs) + xform.yoffs;
		}
		container_xform.color = xform.color;
	}

	// iterate over elements
	for (render_container::item &curitem : container.items())
	{
		// compute the oriented bounds
		render_bounds bounds = curitem.bounds();
		apply_orientation(bounds, container_xform.orientation);

		float xscale = container_xform.xscale;
		float yscale = container_xform.yscale;
		float xoffs = container_xform.xoffs;
		float yoffs = container_xform.yoffs;
		if (!m_transform_container && PRIMFLAG_GET_VECTOR(curitem.flags()))
		{
			xoffs = root_xoffs;
			yoffs = root_yoffs;
		}

		// allocate the primitive and set the transformed bounds/color data
		render_primitive *prim = list.alloc(render_primitive::INVALID);

		prim->container = &container; /* pass the container along for access to user_settings */

		prim->bounds.x0 = render_round_nearest(xoffs + bounds.x0 * xscale);
		prim->bounds.y0 = render_round_nearest(yoffs + bounds.y0 * yscale);
		if (curitem.internal() & INTERNAL_FLAG_CHAR)
		{
			prim->bounds.x1 = prim->bounds.x0 + render_round_nearest((bounds.x1 - bounds.x0) * xscale);
			prim->bounds.y1 = prim->bounds.y0 + render_round_nearest((bounds.y1 - bounds.y0) * yscale);
		}
		else
		{
			prim->bounds.x1 = render_round_nearest(xoffs + bounds.x1 * xscale);
			prim->bounds.y1 = render_round_nearest(yoffs + bounds.y1 * yscale);
		}

		// compute the color of the primitive
		prim->color.r = container_xform.color.r * curitem.color().r;
		prim->color.g = container_xform.color.g * curitem.color().g;
		prim->color.b = container_xform.color.b * curitem.color().b;
		prim->color.a = container_xform.color.a * curitem.color().a;

		// copy unclipped bounds
		prim->full_bounds = prim->bounds;

		// now switch off the type
		bool clipped = true;
		switch (curitem.type())
		{
			case CONTAINER_ITEM_LINE:
				// adjust the color for brightness/contrast/gamma
				prim->color.a = container.apply_brightness_contrast_gamma_fp(prim->color.a);
				prim->color.r = container.apply_brightness_contrast_gamma_fp(prim->color.r);
				prim->color.g = container.apply_brightness_contrast_gamma_fp(prim->color.g);
				prim->color.b = container.apply_brightness_contrast_gamma_fp(prim->color.b);

				// set the line type
				prim->type = render_primitive::LINE;
				prim->flags |= PRIMFLAG_TYPE_LINE;

				// scale the width by the minimum of X/Y scale factors
				prim->width = curitem.width() * std::min(container_xform.xscale, container_xform.yscale);
				prim->flags |= curitem.flags();

				// clip the primitive
				if (!m_transform_container && PRIMFLAG_GET_VECTOR(curitem.flags()))
				{
					clipped = render_clip_line(&prim->bounds, &root_cliprect);
				}
				else
				{
					clipped = render_clip_line(&prim->bounds, &cliprect);
				}
				break;

			case CONTAINER_ITEM_QUAD:
				// set the quad type
				prim->type = render_primitive::QUAD;
				prim->flags |= PRIMFLAG_TYPE_QUAD;

				// normalize the bounds
				normalize_bounds(prim->bounds);

				// get the scaled bitmap and set the resulting palette
				if (curitem.texture() != nullptr)
				{
					// determine the final orientation
					int finalorient = orientation_add(PRIMFLAG_GET_TEXORIENT(curitem.flags()), container_xform.orientation);

					// based on the swap values, get the scaled final texture
					int width = (finalorient & ORIENTATION_SWAP_XY) ? (prim->bounds.y1 - prim->bounds.y0) : (prim->bounds.x1 - prim->bounds.x0);
					int height = (finalorient & ORIENTATION_SWAP_XY) ? (prim->bounds.x1 - prim->bounds.x0) : (prim->bounds.y1 - prim->bounds.y0);
					width = std::min(width, m_maxtexwidth);
					height = std::min(height, m_maxtexheight);

					curitem.texture()->get_scaled(width, height, prim->texture, list, curitem.flags());

					// set the palette
					prim->texture.palette = curitem.texture()->get_adjusted_palette(container);

					// determine UV coordinates
					prim->texcoords = oriented_texcoords[finalorient];

					// apply clipping
					clipped = render_clip_quad(&prim->bounds, &cliprect, &prim->texcoords);

					// apply the final orientation from the quad flags and then build up the final flags
					prim->flags |= (curitem.flags() & ~(PRIMFLAG_TEXORIENT_MASK | PRIMFLAG_BLENDMODE_MASK | PRIMFLAG_TEXFORMAT_MASK))
						| PRIMFLAG_TEXORIENT(finalorient)
						| PRIMFLAG_TEXFORMAT(curitem.texture()->format());
					prim->flags |= blendmode != -1
						? PRIMFLAG_BLENDMODE(blendmode)
						: PRIMFLAG_BLENDMODE(PRIMFLAG_GET_BLENDMODE(curitem.flags()));
				}
				else
				{
					// adjust the color for brightness/contrast/gamma
					prim->color.r = container.apply_brightness_contrast_gamma_fp(prim->color.r);
					prim->color.g = container.apply_brightness_contrast_gamma_fp(prim->color.g);
					prim->color.b = container.apply_brightness_contrast_gamma_fp(prim->color.b);

					// no texture
					prim->texture.base = nullptr;

					if (PRIMFLAG_GET_VECTORBUF(curitem.flags()))
					{
						// flags X(1) flip-x, Y(2) flip-y, S(4) swap-xy
						//
						// X  Y  S   e.g.       flips
						// 0  0  0   asteroid   !X !Y
						// 0  0  1   -           X  Y
						// 0  1  0   speedfrk   !X  Y
						// 0  1  1   tempest    !X  Y
						// 1  0  0   -           X !Y
						// 1  0  1   -           x !Y
						// 1  1  0   solarq      X  Y
						// 1  1  1   barrier    !X !Y

						bool flip_x = (m_manager.machine().system().flags & ORIENTATION_FLIP_X) == ORIENTATION_FLIP_X;
						bool flip_y = (m_manager.machine().system().flags & ORIENTATION_FLIP_Y) == ORIENTATION_FLIP_Y;
						bool swap_xy = (m_manager.machine().system().flags & ORIENTATION_SWAP_XY) == ORIENTATION_SWAP_XY;

						int vectororient = 0;
						if (flip_x)
						{
							vectororient |= ORIENTATION_FLIP_X;
						}
						if (flip_y)
						{
							vectororient |= ORIENTATION_FLIP_Y;
						}
						if ((flip_x && flip_y && swap_xy) || (!flip_x && !flip_y && swap_xy))
						{
							vectororient ^= ORIENTATION_FLIP_X;
							vectororient ^= ORIENTATION_FLIP_Y;
						}

						// determine the final orientation (textures are up-side down, so flip axis for vectors to immitate that behavior)
						int finalorient = orientation_add(vectororient, container_xform.orientation);

						// determine UV coordinates
						prim->texcoords = oriented_texcoords[finalorient];

						// apply clipping
						clipped = render_clip_quad(&prim->bounds, &cliprect, &prim->texcoords);

						// apply the final orientation from the quad flags and then build up the final flags
						prim->flags |= (curitem.flags() & ~(PRIMFLAG_TEXORIENT_MASK | PRIMFLAG_BLENDMODE_MASK | PRIMFLAG_TEXFORMAT_MASK))
							| PRIMFLAG_TEXORIENT(finalorient);
						prim->flags |= blendmode != -1
							? PRIMFLAG_BLENDMODE(blendmode)
							: PRIMFLAG_BLENDMODE(PRIMFLAG_GET_BLENDMODE(curitem.flags()));
					}
					else
					{
						// set the basic flags
						prim->flags |= (curitem.flags() & ~PRIMFLAG_BLENDMODE_MASK)
							| PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA);

						// apply clipping
						clipped = render_clip_quad(&prim->bounds, &cliprect, nullptr);
					}
				}
				break;
		}

		// add to the list or free if we're clipped out
		list.append_or_return(*prim, clipped);
	}

	// add the overlay if it exists
	if (container.overlay() != nullptr && m_layerconfig.screen_overlay_enabled())
	{
		s32 width, height;

		// allocate a primitive
		render_primitive *prim = list.alloc(render_primitive::QUAD);
		set_render_bounds_wh(&prim->bounds, xform.xoffs, xform.yoffs, xform.xscale, xform.yscale);
		prim->full_bounds = prim->bounds;
		prim->color = container_xform.color;
		width = render_round_nearest(prim->bounds.x1) - render_round_nearest(prim->bounds.x0);
		height = render_round_nearest(prim->bounds.y1) - render_round_nearest(prim->bounds.y0);

		container.overlay()->get_scaled(
			(container_xform.orientation & ORIENTATION_SWAP_XY) ? height : width,
			(container_xform.orientation & ORIENTATION_SWAP_XY) ? width : height, prim->texture, list);

		// determine UV coordinates
		prim->texcoords = oriented_texcoords[container_xform.orientation];

		// set the flags and add it to the list
		prim->flags = PRIMFLAG_TEXORIENT(container_xform.orientation)
			| PRIMFLAG_BLENDMODE(BLENDMODE_RGB_MULTIPLY)
			| PRIMFLAG_TEXFORMAT(container.overlay()->format())
			| PRIMFLAG_TEXSHADE(1);

		list.append_or_return(*prim, false);
	}
}


//-------------------------------------------------
//  add_element_primitives - add the primitive
//  for an element in the current state
//-------------------------------------------------

void render_target::add_element_primitives(render_primitive_list &list, const object_transform &xform, layout_element &element, int state, int blendmode)
{
	// if we're out of range, bail
	if (state > element.maxstate())
		return;
	if (state < 0)
		state = 0;

	// get a pointer to the relevant texture
	render_texture *texture = element.state_texture(state);
	if (texture != nullptr)
	{
		render_primitive *prim = list.alloc(render_primitive::QUAD);

		// configure the basics
		prim->color = xform.color;
		prim->flags = PRIMFLAG_TEXORIENT(xform.orientation) | PRIMFLAG_BLENDMODE(blendmode) | PRIMFLAG_TEXFORMAT(texture->format());

		// compute the bounds
		s32 width = render_round_nearest(xform.xscale);
		s32 height = render_round_nearest(xform.yscale);
		set_render_bounds_wh(&prim->bounds, render_round_nearest(xform.xoffs), render_round_nearest(xform.yoffs), (float) width, (float) height);
		prim->full_bounds = prim->bounds;
		if (xform.orientation & ORIENTATION_SWAP_XY)
			std::swap(width, height);
		width = std::min(width, m_maxtexwidth);
		height = std::min(height, m_maxtexheight);

		// get the scaled texture and append it

		texture->get_scaled(width, height, prim->texture, list, prim->flags);

		// compute the clip rect
		render_bounds cliprect;
		cliprect.x0 = render_round_nearest(xform.xoffs);
		cliprect.y0 = render_round_nearest(xform.yoffs);
		cliprect.x1 = render_round_nearest(xform.xoffs + xform.xscale);
		cliprect.y1 = render_round_nearest(xform.yoffs + xform.yscale);
		sect_render_bounds(&cliprect, &m_bounds);

		// determine UV coordinates and apply clipping
		prim->texcoords = oriented_texcoords[xform.orientation];
		bool clipped = render_clip_quad(&prim->bounds, &cliprect, &prim->texcoords);

		// add to the list or free if we're clipped out
		list.append_or_return(*prim, clipped);
	}
}


//-------------------------------------------------
//  map_point_internal - internal logic for
//  mapping points
//-------------------------------------------------

bool render_target::map_point_internal(s32 target_x, s32 target_y, render_container *container, float &mapped_x, float &mapped_y, ioport_port *&mapped_input_port, ioport_value &mapped_input_mask)
{
	// compute the visible width/height
	s32 viswidth, visheight;
	compute_visible_area(m_width, m_height, m_pixel_aspect, m_orientation, viswidth, visheight);

	// create a root transform for the target
	object_transform root_xform;
	root_xform.xoffs = (float)(m_width - viswidth) / 2;
	root_xform.yoffs = (float)(m_height - visheight) / 2;

	// default to point not mapped
	mapped_x = -1.0;
	mapped_y = -1.0;
	mapped_input_port = nullptr;
	mapped_input_mask = 0;

	// convert target coordinates to float
	float target_fx = (float)(target_x - root_xform.xoffs) / viswidth;
	float target_fy = (float)(target_y - root_xform.yoffs) / visheight;
	if (m_manager.machine().ui().is_menu_active())
	{
		target_fx = (float)target_x / m_width;
		target_fy = (float)target_y / m_height;
	}
	// explicitly check for the UI container
	if (container != nullptr && container == &m_manager.ui_container())
	{
		// this hit test went against the UI container
		if (target_fx >= 0.0f && target_fx < 1.0f && target_fy >= 0.0f && target_fy < 1.0f)
		{
			// this point was successfully mapped
			mapped_x = (float)target_x / m_width;
			mapped_y = (float)target_y / m_height;
			return true;
		}
		return false;
	}

	// loop through each layer
	for (item_layer layernum = ITEM_LAYER_FIRST; layernum < ITEM_LAYER_MAX; ++layernum)
	{
		int blendmode;
		item_layer layer = get_layer_and_blendmode(*m_curview, layernum, blendmode);
		if (m_curview->layer_enabled(layer))
		{
			// iterate over items in the layer
			for (layout_view::item &item : m_curview->items(layer))
			{
				bool checkit;

				// if we're looking for a particular container, verify that we have the right one
				if (container != nullptr)
					checkit = (item.screen() != nullptr && &item.screen()->container() == container);

				// otherwise, assume we're looking for an input
				else
					checkit = item.has_input();

				// this target is worth looking at; now check the point
				if (checkit && target_fx >= item.bounds().x0 && target_fx < item.bounds().x1 && target_fy >= item.bounds().y0 && target_fy < item.bounds().y1)
				{
					// point successfully mapped
					mapped_x = (target_fx - item.bounds().x0) / (item.bounds().x1 - item.bounds().x0);
					mapped_y = (target_fy - item.bounds().y0) / (item.bounds().y1 - item.bounds().y0);
					mapped_input_port = item.input_tag_and_mask(mapped_input_mask);
					return true;
				}
			}
		}
	}
	return false;
}


//-------------------------------------------------
//  view_name - return the name of the indexed
//  view, or nullptr if it doesn't exist
//-------------------------------------------------

layout_view *render_target::view_by_index(int index) const
{
	// scan the list of views within each layout, skipping those that don't apply
	for (layout_file &file : m_filelist)
		for (layout_view &view : file.views())
			if (!(m_flags & RENDER_CREATE_NO_ART) || !view.has_art())
				if (index-- == 0)
					return &view;
	return nullptr;
}


//-------------------------------------------------
//  view_index - return the index of the given
//  view
//-------------------------------------------------

int render_target::view_index(layout_view &targetview) const
{
	// find the first named match
	int index = 0;

	// scan the list of views within each layout, skipping those that don't apply
	for (layout_file &file : m_filelist)
		for (layout_view &view : file.views())
			if (!(m_flags & RENDER_CREATE_NO_ART) || !view.has_art())
			{
				if (&targetview == &view)
					return index;
				index++;
			}
	return 0;
}


//-------------------------------------------------
//  config_load - process config information
//-------------------------------------------------

void render_target::config_load(util::xml::data_node const &targetnode)
{
	// find the view
	const char *viewname = targetnode.get_attribute_string("view", nullptr);
	if (viewname != nullptr)
		for (int viewnum = 0; viewnum < 1000; viewnum++)
		{
			const char *testname = view_name(viewnum);
			if (testname == nullptr)
				break;
			if (!strcmp(viewname, testname))
			{
				set_view(viewnum);
				break;
			}
		}

	// modify the artwork config
	int tmpint = targetnode.get_attribute_int("backdrops", -1);
	if (tmpint == 0 || tmpint == 1)
		set_backdrops_enabled(tmpint);

	tmpint = targetnode.get_attribute_int("overlays", -1);
	if (tmpint == 0 || tmpint == 1)
		set_overlays_enabled(tmpint);

	tmpint = targetnode.get_attribute_int("bezels", -1);
	if (tmpint == 0 || tmpint == 1)
		set_bezels_enabled(tmpint);

	tmpint = targetnode.get_attribute_int("cpanels", -1);
	if (tmpint == 0 || tmpint == 1)
		set_cpanels_enabled(tmpint);

	tmpint = targetnode.get_attribute_int("marquees", -1);
	if (tmpint == 0 || tmpint == 1)
		set_marquees_enabled(tmpint);

	tmpint = targetnode.get_attribute_int("zoom", -1);
	if (tmpint == 0 || tmpint == 1)
		set_zoom_to_screen(tmpint);

	// apply orientation
	tmpint = targetnode.get_attribute_int("rotate", -1);
	if (tmpint != -1)
	{
		if (tmpint == 90)
			tmpint = ROT90;
		else if (tmpint == 180)
			tmpint = ROT180;
		else if (tmpint == 270)
			tmpint = ROT270;
		else
			tmpint = ROT0;
		set_orientation(orientation_add(tmpint, orientation()));

		// apply the opposite orientation to the UI
		if (is_ui_target())
		{
			render_container::user_settings settings;
			render_container &ui_container = m_manager.ui_container();

			ui_container.get_user_settings(settings);
			settings.m_orientation = orientation_add(orientation_reverse(tmpint), settings.m_orientation);
			ui_container.set_user_settings(settings);
		}
	}
}


//-------------------------------------------------
//  config_save - save our configuration, or
//  return false if we are the same as the default
//-------------------------------------------------

bool render_target::config_save(util::xml::data_node &targetnode)
{
	bool changed = false;

	// output the basics
	targetnode.set_attribute_int("index", index());

	// output the view
	if (m_curview != m_base_view)
	{
		targetnode.set_attribute("view", m_curview->name());
		changed = true;
	}

	// output the layer config
	if (m_layerconfig != m_base_layerconfig)
	{
		targetnode.set_attribute_int("backdrops", m_layerconfig.backdrops_enabled());
		targetnode.set_attribute_int("overlays", m_layerconfig.overlays_enabled());
		targetnode.set_attribute_int("bezels", m_layerconfig.bezels_enabled());
		targetnode.set_attribute_int("cpanels", m_layerconfig.cpanels_enabled());
		targetnode.set_attribute_int("marquees", m_layerconfig.marquees_enabled());
		targetnode.set_attribute_int("zoom", m_layerconfig.zoom_to_screen());
		changed = true;
	}

	// output rotation
	if (m_orientation != m_base_orientation)
	{
		int rotate = 0;
		if (orientation_add(ROT90, m_base_orientation) == m_orientation)
			rotate = 90;
		else if (orientation_add(ROT180, m_base_orientation) == m_orientation)
			rotate = 180;
		else if (orientation_add(ROT270, m_base_orientation) == m_orientation)
			rotate = 270;
		assert(rotate != 0);
		targetnode.set_attribute_int("rotate", rotate);
		changed = true;
	}

	return changed;
}


//-------------------------------------------------
//  init_clear_extents - reset the extents list
//-------------------------------------------------

void render_target::init_clear_extents()
{
	m_clear_extents[0] = -m_height;
	m_clear_extents[1] = 1;
	m_clear_extents[2] = m_width;
	m_clear_extent_count = 3;
}


//-------------------------------------------------
//  remove_clear_extent - remove a quad from the
//  list of stuff to clear, unless it overlaps
//  a previous quad
//-------------------------------------------------

bool render_target::remove_clear_extent(const render_bounds &bounds)
{
	s32 *max = &m_clear_extents[MAX_CLEAR_EXTENTS];
	s32 *last = &m_clear_extents[m_clear_extent_count];
	s32 *ext = &m_clear_extents[0];
	s32 boundsx0 = ceil(bounds.x0);
	s32 boundsx1 = floor(bounds.x1);
	s32 boundsy0 = ceil(bounds.y0);
	s32 boundsy1 = floor(bounds.y1);
	s32 y0, y1 = 0;

	// loop over Y extents
	while (ext < last)
	{
		s32 *linelast;

		// first entry of each line should always be negative
		assert(ext[0] < 0.0f);
		y0 = y1;
		y1 = y0 - ext[0];

		// do we intersect this extent?
		if (boundsy0 < y1 && boundsy1 > y0)
		{
			s32 *xext;
			s32 x0, x1 = 0;

			// split the top
			if (y0 < boundsy0)
			{
				int diff = boundsy0 - y0;

				// make a copy of this extent
				memmove(&ext[ext[1] + 2], &ext[0], (last - ext) * sizeof(*ext));
				last += ext[1] + 2;
				assert_always(last < max, "Ran out of clear extents!\n");

				// split the extent between pieces
				ext[ext[1] + 2] = -(-ext[0] - diff);
				ext[0] = -diff;

				// advance to the new extent
				y0 -= ext[0];
				ext += ext[1] + 2;
				y1 = y0 - ext[0];
			}

			// split the bottom
			if (y1 > boundsy1)
			{
				int diff = y1 - boundsy1;

				// make a copy of this extent
				memmove(&ext[ext[1] + 2], &ext[0], (last - ext) * sizeof(*ext));
				last += ext[1] + 2;
				assert_always(last < max, "Ran out of clear extents!\n");

				// split the extent between pieces
				ext[ext[1] + 2] = -diff;
				ext[0] = -(-ext[0] - diff);

				// recompute y1
				y1 = y0 - ext[0];
			}

			// now remove the X extent
			linelast = &ext[ext[1] + 2];
			xext = &ext[2];
			while (xext < linelast)
			{
				x0 = x1;
				x1 = x0 + xext[0];

				// do we fully intersect this extent?
				if (boundsx0 >= x0 && boundsx1 <= x1)
				{
					// yes; split it
					memmove(&xext[2], &xext[0], (last - xext) * sizeof(*xext));
					last += 2;
					linelast += 2;
					assert_always(last < max, "Ran out of clear extents!\n");

					// split this extent into three parts
					xext[0] = boundsx0 - x0;
					xext[1] = boundsx1 - boundsx0;
					xext[2] = x1 - boundsx1;

					// recompute x1
					x1 = boundsx1;
					xext += 2;
				}

				// do we partially intersect this extent?
				else if (boundsx0 < x1 && boundsx1 > x0)
					goto abort;

				// advance
				xext++;

				// do we partially intersect the next extent (which is a non-clear extent)?
				if (xext < linelast)
				{
					x0 = x1;
					x1 = x0 + xext[0];
					if (boundsx0 < x1 && boundsx1 > x0)
						goto abort;
					xext++;
				}
			}

			// update the count
			ext[1] = linelast - &ext[2];
		}

		// advance to the next row
		ext += 2 + ext[1];
	}

	// update the total count
	m_clear_extent_count = last - &m_clear_extents[0];
	return true;

abort:
	// update the total count even on a failure as we may have split extents
	m_clear_extent_count = last - &m_clear_extents[0];
	return false;
}


//-------------------------------------------------
//  add_clear_extents - add the accumulated
//  extents as a series of quads to clear
//-------------------------------------------------

void render_target::add_clear_extents(render_primitive_list &list)
{
	simple_list<render_primitive> clearlist;
	s32 *last = &m_clear_extents[m_clear_extent_count];
	s32 *ext = &m_clear_extents[0];
	s32 y0, y1 = 0;

	// loop over all extents
	while (ext < last)
	{
		s32 *linelast = &ext[ext[1] + 2];
		s32 *xext = &ext[2];
		s32 x0, x1 = 0;

		// first entry should always be negative
		assert(ext[0] < 0);
		y0 = y1;
		y1 = y0 - ext[0];

		// now remove the X extent
		while (xext < linelast)
		{
			x0 = x1;
			x1 = x0 + *xext++;

			// only add entries for non-zero widths
			if (x1 - x0 > 0)
			{
				render_primitive *prim = list.alloc(render_primitive::QUAD);
				set_render_bounds_xy(&prim->bounds, (float)x0, (float)y0, (float)x1, (float)y1);
				prim->full_bounds = prim->bounds;
				set_render_color(&prim->color, 1.0f, 0.0f, 0.0f, 0.0f);
				prim->texture.base = nullptr;
				prim->flags = PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA);
				clearlist.append(*prim);
			}

			// skip the non-clearing extent
			x0 = x1;
			x1 = x0 + *xext++;
		}

		// advance to the next part
		ext += 2 + ext[1];
	}

	// we know that the first primitive in the list will be the global clip
	// so we insert the clears immediately after
	list.m_primlist.prepend_list(clearlist);
}


//-------------------------------------------------
//  add_clear_and_optimize_primitive_list -
//  optimize the primitive list
//-------------------------------------------------

void render_target::add_clear_and_optimize_primitive_list(render_primitive_list &list)
{
	// start with the assumption that we need to clear the whole screen
	init_clear_extents();

	// scan the list until we hit an intersection quad or a line
	for (render_primitive &prim : list)
	{
		// switch off the type
		switch (prim.type)
		{
			case render_primitive::LINE:
				goto done;

			case render_primitive::QUAD:
			{
				// stop when we hit an alpha texture
				if (PRIMFLAG_GET_TEXFORMAT(prim.flags) == TEXFORMAT_ARGB32 || PRIMFLAG_GET_TEXFORMAT(prim.flags) == TEXFORMAT_PALETTEA16)
					goto done;

				// if this quad can't be cleanly removed from the extents list, we're done
				if (!remove_clear_extent(prim.bounds))
					goto done;

				// change the blendmode on the first primitive to be NONE
				if (PRIMFLAG_GET_BLENDMODE(prim.flags) == BLENDMODE_RGB_MULTIPLY)
				{
					// RGB multiply will multiply against 0, leaving nothing
					set_render_color(&prim.color, 1.0f, 0.0f, 0.0f, 0.0f);
					prim.texture.base = nullptr;
					prim.flags = (prim.flags & ~PRIMFLAG_BLENDMODE_MASK) | PRIMFLAG_BLENDMODE(BLENDMODE_NONE);
				}
				else
				{
					// for alpha or add modes, we will blend against 0 or add to 0; treat it like none
					prim.flags = (prim.flags & ~PRIMFLAG_BLENDMODE_MASK) | PRIMFLAG_BLENDMODE(BLENDMODE_NONE);
				}

				// since alpha is disabled, premultiply the RGB values and reset the alpha to 1.0
				prim.color.r *= prim.color.a;
				prim.color.g *= prim.color.a;
				prim.color.b *= prim.color.a;
				prim.color.a = 1.0f;
				break;
			}

			default:
				throw emu_fatalerror("Unexpected primitive type");
		}
	}

done:
	// now add the extents to the clear list
	add_clear_extents(list);
}



//**************************************************************************
//  CORE IMPLEMENTATION
//**************************************************************************

//-------------------------------------------------
//  render_manager - constructor
//-------------------------------------------------

render_manager::render_manager(running_machine &machine)
	: m_machine(machine),
		m_ui_target(nullptr),
		m_live_textures(0),
		m_ui_container(global_alloc(render_container(*this)))
{
	// register callbacks
	machine.configuration().config_register("video", config_load_delegate(&render_manager::config_load, this), config_save_delegate(&render_manager::config_save, this));

	// create one container per screen
	for (screen_device &screen : screen_device_iterator(machine.root_device()))
		screen.set_container(*container_alloc(&screen));
}


//-------------------------------------------------
//  ~render_manager - destructor
//-------------------------------------------------

render_manager::~render_manager()
{
	// free all the containers since they may own textures
	container_free(m_ui_container);
	m_screen_container_list.reset();

	// better not be any outstanding textures when we die
	assert(m_live_textures == 0);
}


//-------------------------------------------------
//  is_live - return if the screen is 'live'
//-------------------------------------------------

bool render_manager::is_live(screen_device &screen) const
{
	// iterate over all live targets and or together their screen masks
	for (render_target &target : m_targetlist)
		if (!target.hidden() && target.view_screens(target.view()).contains(screen))
			return true;
	return false;
}


//-------------------------------------------------
//  max_update_rate - return the smallest maximum
//  update rate across all targets
//-------------------------------------------------

float render_manager::max_update_rate() const
{
	// iterate over all live targets and or together their screen masks
	float minimum = 0;
	for (render_target &target : m_targetlist)
		if (target.max_update_rate() != 0)
		{
			if (minimum == 0)
				minimum = target.max_update_rate();
			else
				minimum = std::min(target.max_update_rate(), minimum);
		}

	return minimum;
}


//-------------------------------------------------
//  target_alloc - allocate a new target
//-------------------------------------------------

render_target *render_manager::target_alloc(const internal_layout *layoutfile, u32 flags)
{
	return &m_targetlist.append(*global_alloc(render_target(*this, layoutfile, flags)));
}


//-------------------------------------------------
//  target_free - free a target
//-------------------------------------------------

void render_manager::target_free(render_target *target)
{
	if (target != nullptr)
		m_targetlist.remove(*target);
}


//-------------------------------------------------
//  target_by_index - get a render_target by index
//-------------------------------------------------

render_target *render_manager::target_by_index(int index) const
{
	// count up the targets until we hit the requested index
	for (render_target &target : m_targetlist)
		if (!target.hidden())
			if (index-- == 0)
				return &target;
	return nullptr;
}


//-------------------------------------------------
//  ui_aspect - return the aspect ratio for UI
//  fonts
//-------------------------------------------------

float render_manager::ui_aspect(render_container *rc)
{
	int orient;
	float aspect;

	if (rc == m_ui_container || rc == nullptr) {
		// ui container, aggregated multi-screen target

		orient = orientation_add(m_ui_target->orientation(), m_ui_container->orientation());
		// based on the orientation of the target, compute height/width or width/height
		if (!(orient & ORIENTATION_SWAP_XY))
				aspect = (float)m_ui_target->height() / (float)m_ui_target->width();
		else
				aspect = (float)m_ui_target->width() / (float)m_ui_target->height();

		// if we have a valid pixel aspect, apply that and return
		if (m_ui_target->pixel_aspect() != 0.0f)
		{
			float pixel_aspect = m_ui_target->pixel_aspect();

			if (orient & ORIENTATION_SWAP_XY)
				pixel_aspect = 1.0f / pixel_aspect;

			return aspect /= pixel_aspect;
		}

	} else {
		// single screen container

		orient = rc->orientation();
		// based on the orientation of the target, compute height/width or width/height
		if (!(orient & ORIENTATION_SWAP_XY))
			aspect = (float)rc->screen()->visible_area().height() / (float)rc->screen()->visible_area().width();
		else
			aspect = (float)rc->screen()->visible_area().width() / (float)rc->screen()->visible_area().height();
	}

	// clamp for extreme proportions
	if (aspect < 0.66f)
		aspect = 0.66f;
	if (aspect > 1.5f)
		aspect = 1.5f;

	return aspect;
}


//-------------------------------------------------
//  texture_alloc - allocate a new texture
//-------------------------------------------------

render_texture *render_manager::texture_alloc(texture_scaler_func scaler, void *param)
{
	// allocate a new texture and reset it
	render_texture *tex = m_texture_allocator.alloc();
	tex->reset(*this, scaler, param);
	m_live_textures++;
	return tex;
}


//-------------------------------------------------
//  texture_free - release a texture
//-------------------------------------------------

void render_manager::texture_free(render_texture *texture)
{
	if (texture != nullptr)
	{
		m_live_textures--;
		texture->release();
	}
	m_texture_allocator.reclaim(texture);
}


//-------------------------------------------------
//  font_alloc - allocate a new font instance
//-------------------------------------------------

render_font *render_manager::font_alloc(const char *filename)
{
	return global_alloc(render_font(*this, filename));
}


//-------------------------------------------------
//  font_free - release a font instance
//-------------------------------------------------

void render_manager::font_free(render_font *font)
{
	global_free(font);
}


//-------------------------------------------------
//  invalidate_all - remove all refs to a
//  particular reference pointer
//-------------------------------------------------

void render_manager::invalidate_all(void *refptr)
{
	// permit nullptr
	if (refptr == nullptr)
		return;

	// loop over targets
	for (render_target &target : m_targetlist)
		target.invalidate_all(refptr);
}


//-------------------------------------------------
//  resolve_tags - resolve tag lookups
//-------------------------------------------------

void render_manager::resolve_tags()
{
	for (render_target &target : m_targetlist)
		target.resolve_tags();
}


//-------------------------------------------------
//  container_alloc - allocate a new container
//-------------------------------------------------

render_container *render_manager::container_alloc(screen_device *screen)
{
	auto container = global_alloc(render_container(*this, screen));
	if (screen != nullptr)
		m_screen_container_list.append(*container);
	return container;
}


//-------------------------------------------------
//  container_free - release a container
//-------------------------------------------------

void render_manager::container_free(render_container *container)
{
	m_screen_container_list.remove(*container);
}


//-------------------------------------------------
//  config_load - read and apply data from the
//  configuration file
//-------------------------------------------------

void render_manager::config_load(config_type cfg_type, util::xml::data_node const *parentnode)
{
	// we only care about game files
	if (cfg_type != config_type::GAME)
		return;

	// might not have any data
	if (parentnode == nullptr)
		return;

	// check the UI target
	util::xml::data_node const *const uinode = parentnode->get_child("interface");
	if (uinode != nullptr)
	{
		render_target *target = target_by_index(uinode->get_attribute_int("target", 0));
		if (target != nullptr)
			set_ui_target(*target);
	}

	// iterate over target nodes
	for (util::xml::data_node const *targetnode = parentnode->get_child("target"); targetnode; targetnode = targetnode->get_next_sibling("target"))
	{
		render_target *target = target_by_index(targetnode->get_attribute_int("index", -1));
		if (target != nullptr)
			target->config_load(*targetnode);
	}

	// iterate over screen nodes
	for (util::xml::data_node const *screennode = parentnode->get_child("screen"); screennode; screennode = screennode->get_next_sibling("screen"))
	{
		int index = screennode->get_attribute_int("index", -1);
		render_container *container = m_screen_container_list.find(index);
		render_container::user_settings settings;

		// fetch current settings
		container->get_user_settings(settings);

		// fetch color controls
		settings.m_brightness = screennode->get_attribute_float("brightness", settings.m_brightness);
		settings.m_contrast = screennode->get_attribute_float("contrast", settings.m_contrast);
		settings.m_gamma = screennode->get_attribute_float("gamma", settings.m_gamma);

		// fetch positioning controls
		settings.m_xoffset = screennode->get_attribute_float("hoffset", settings.m_xoffset);
		settings.m_xscale = screennode->get_attribute_float("hstretch", settings.m_xscale);
		settings.m_yoffset = screennode->get_attribute_float("voffset", settings.m_yoffset);
		settings.m_yscale = screennode->get_attribute_float("vstretch", settings.m_yscale);

		// set the new values
		container->set_user_settings(settings);
	}
}


//-------------------------------------------------
//  config_save - save data to the configuration
//  file
//-------------------------------------------------

void render_manager::config_save(config_type cfg_type, util::xml::data_node *parentnode)
{
	// we only care about game files
	if (cfg_type != config_type::GAME)
		return;

	// write out the interface target
	if (m_ui_target->index() != 0)
	{
		// create a node for it
		util::xml::data_node *const uinode = parentnode->add_child("interface", nullptr);
		if (uinode != nullptr)
			uinode->set_attribute_int("target", m_ui_target->index());
	}

	// iterate over targets
	for (int targetnum = 0; targetnum < 1000; targetnum++)
	{
		// get this target and break when we fail
		render_target *target = target_by_index(targetnum);
		if (target == nullptr)
			break;

		// create a node
		util::xml::data_node *const targetnode = parentnode->add_child("target", nullptr);
		if (targetnode != nullptr && !target->config_save(*targetnode))
			targetnode->delete_node();
	}

	// iterate over screen containers
	int scrnum = 0;
	for (render_container *container = m_screen_container_list.first(); container != nullptr; container = container->next(), scrnum++)
	{
		// create a node
		util::xml::data_node *const screennode = parentnode->add_child("screen", nullptr);
		if (screennode != nullptr)
		{
			bool changed = false;

			// output the basics
			screennode->set_attribute_int("index", scrnum);

			render_container::user_settings settings;
			container->get_user_settings(settings);

			// output the color controls
			if (settings.m_brightness != machine().options().brightness())
			{
				screennode->set_attribute_float("brightness", settings.m_brightness);
				changed = true;
			}

			if (settings.m_contrast != machine().options().contrast())
			{
				screennode->set_attribute_float("contrast", settings.m_contrast);
				changed = true;
			}

			if (settings.m_gamma != machine().options().gamma())
			{
				screennode->set_attribute_float("gamma", settings.m_gamma);
				changed = true;
			}

			// output the positioning controls
			if (settings.m_xoffset != 0.0f)
			{
				screennode->set_attribute_float("hoffset", settings.m_xoffset);
				changed = true;
			}

			if (settings.m_xscale != 1.0f)
			{
				screennode->set_attribute_float("hstretch", settings.m_xscale);
				changed = true;
			}

			if (settings.m_yoffset != 0.0f)
			{
				screennode->set_attribute_float("voffset", settings.m_yoffset);
				changed = true;
			}

			if (settings.m_yscale != 1.0f)
			{
				screennode->set_attribute_float("vstretch", settings.m_yscale);
				changed = true;
			}

			// if nothing changed, kill the node
			if (!changed)
				screennode->delete_node();
		}
	}
}
