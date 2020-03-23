// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria, Aaron Giles, Nathan Woods
/*********************************************************************

    ui/widgets.cpp

    Internal MAME widgets for the user interface.

*********************************************************************/

#include "emu.h"

#include "widgets.h"


namespace ui {
/***************************************************************************
    WIDGETS
***************************************************************************/

//-------------------------------------------------
//  ctor
//-------------------------------------------------

widgets_manager::widgets_manager(running_machine &machine)
	: m_hilight_bitmap(std::make_unique<bitmap_argb32>(256, 1))
	, m_hilight_texture()
	, m_hilight_main_bitmap(std::make_unique<bitmap_argb32>(1, 128))
	, m_hilight_main_texture()
{
	render_manager &render(machine.render());
	auto const texture_free([&render](render_texture *texture) { render.texture_free(texture); });

	// create a texture for hilighting items
	for (unsigned x = 0; x < 256; ++x)
	{
		unsigned const alpha((x < 25) ? (0xff * x / 25) : (x >(256 - 25)) ? (0xff * (255 - x) / 25) : 0xff);
		m_hilight_bitmap->pix32(0, x) = rgb_t(alpha, 0xff, 0xff, 0xff);
	}
	m_hilight_texture = texture_ptr(render.texture_alloc(), texture_free);
	m_hilight_texture->set_bitmap(*m_hilight_bitmap, m_hilight_bitmap->cliprect(), TEXFORMAT_ARGB32);

	// create a texture for hilighting items in main menu
	for (unsigned y = 0; y < 128; ++y)
	{
		constexpr unsigned r1(0), g1(169), b1 = (255); // any start color
		constexpr unsigned r2(0), g2(39), b2 = (130); // any stop color
		unsigned const r = r1 + (y * (r2 - r1) / 128);
		unsigned const g = g1 + (y * (g2 - g1) / 128);
		unsigned const b = b1 + (y * (b2 - b1) / 128);
		m_hilight_main_bitmap->pix32(y, 0) = rgb_t(r, g, b);
	}
	m_hilight_main_texture = texture_ptr(render.texture_alloc(), texture_free);
	m_hilight_main_texture->set_bitmap(*m_hilight_main_bitmap, m_hilight_main_bitmap->cliprect(), TEXFORMAT_ARGB32);

	// create a texture for arrow icons
	m_arrow_texture = texture_ptr(render.texture_alloc(render_triangle), texture_free);
}


//-------------------------------------------------
//  render_triangle - render a triangle that
//  is used for up/down arrows and left/right
//  indicators
//-------------------------------------------------

void widgets_manager::render_triangle(bitmap_argb32 &dest, bitmap_argb32 &source, const rectangle &sbounds, void *param)
{
	int halfwidth = dest.width() / 2;
	int height = dest.height();
	int x, y;

	// start with all-transparent
	dest.fill(rgb_t(0x00, 0x00, 0x00, 0x00));

	// render from the tip to the bottom
	for (y = 0; y < height; y++)
	{
		int linewidth = (y * (halfwidth - 1) + (height / 2)) * 255 * 2 / height;
		uint32_t *target = &dest.pix32(y, halfwidth);

		// don't antialias if height < 12
		if (dest.height() < 12)
		{
			int pixels = (linewidth + 254) / 255;
			if (pixels % 2 == 0) pixels++;
			linewidth = pixels * 255;
		}

		// loop while we still have data to generate
		for (x = 0; linewidth > 0; x++)
		{
			int dalpha;

			// first column we only consume one pixel
			if (x == 0)
			{
				dalpha = std::min(0xff, linewidth);
				target[x] = rgb_t(dalpha, 0xff, 0xff, 0xff);
			}

			// remaining columns consume two pixels, one on each side
			else
			{
				dalpha = std::min(0x1fe, linewidth);
				target[x] = target[-x] = rgb_t(dalpha / 2, 0xff, 0xff, 0xff);
			}

			// account for the weight we consumed */
			linewidth -= dalpha;
		}
	}
}

} // namespace ui
