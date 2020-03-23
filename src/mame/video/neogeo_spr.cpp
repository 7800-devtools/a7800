// license:BSD-3-Clause
// copyright-holders:Bryan McPhail,Ernesto Corvi,Andrew Prime,Zsolt Vasvari
// thanks-to:Fuzz
/* NeoGeo sprites (and fixed text layer) */

#include "emu.h"
#include "neogeo_spr.h"
#include "screen.h"


// pure virtual functions
//const device_type NEOGEO_SPRITE_BASE = device_creator<neosprite_base_device>;

neosprite_base_device::neosprite_base_device(
		const machine_config &mconfig,
		device_type type,
		const char *tag,
		device_t *owner,
		uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, m_bppshift(4)
{
}

void neosprite_base_device::device_start()
{
	m_videoram = std::make_unique<uint16_t[]>(0x8000 + 0x800);
	m_videoram_drawsource = m_videoram.get();

	/* clear allocated memory */
	memset(m_videoram.get(), 0x00, (0x8000 + 0x800) * sizeof(uint16_t));

	create_sprite_line_timer();
	create_auto_animation_timer();

	/* initialize values that are not modified on a reset */
	m_vram_offset = 0;
	m_vram_read_buffer = 0;
	m_vram_modulo = 0;
	m_auto_animation_speed = 0;
	m_auto_animation_disabled = 0;
	m_auto_animation_counter = 0;
	m_auto_animation_frame_counter = 0;

	/* register for state saving */
	save_pointer(NAME(m_videoram.get()), 0x8000 + 0x800);
	save_item(NAME(m_vram_offset));
	save_item(NAME(m_vram_read_buffer));
	save_item(NAME(m_vram_modulo));
	save_item(NAME(m_fixed_layer_source));

	save_item(NAME(m_auto_animation_speed));
	save_item(NAME(m_auto_animation_disabled));
	save_item(NAME(m_auto_animation_counter));
	save_item(NAME(m_auto_animation_frame_counter));


	m_region_zoomy = memregion(":zoomy")->base();
}

void neosprite_base_device::device_reset()
{
	//m_sprite_gfx_address_mask = 0;
	optimize_sprite_data();

	start_sprite_line_timer();
	start_auto_animation_timer();
}



/*************************************
 *
 *  Video RAM access
 *
 *************************************/

void neosprite_base_device::set_videoram_offset(uint16_t data)
{
	m_vram_offset = (data & 0x8000 ? data & 0x87ff : data);

	/* the read happens right away */
	m_vram_read_buffer = m_videoram[m_vram_offset];
}


uint16_t neosprite_base_device::get_videoram_data()
{
	return m_vram_read_buffer;
}


void neosprite_base_device::set_videoram_data(uint16_t data)
{
	m_videoram[m_vram_offset] = data;

	/* auto increment/decrement the current offset - A15 is NOT affected */
	set_videoram_offset((m_vram_offset & 0x8000) | ((m_vram_offset + m_vram_modulo) & 0x7fff));
}


void neosprite_base_device::set_videoram_modulo(uint16_t data)
{
	m_vram_modulo = data;
}


uint16_t neosprite_base_device::get_videoram_modulo()
{
	return m_vram_modulo;
}



/*************************************
 *
 *  Auto animation
 *
 *************************************/

void neosprite_base_device::set_auto_animation_speed(uint8_t data)
{
	m_auto_animation_speed = data;
}


void neosprite_base_device::set_auto_animation_disabled(uint8_t data)
{
	m_auto_animation_disabled = data;
}


uint8_t neosprite_base_device::neogeo_get_auto_animation_counter()
{
	return m_auto_animation_counter;
}


TIMER_CALLBACK_MEMBER(neosprite_base_device::auto_animation_timer_callback)
{
	if (m_auto_animation_frame_counter == 0)
	{
		m_auto_animation_frame_counter = m_auto_animation_speed;
		m_auto_animation_counter += 1;
	}
	else
		m_auto_animation_frame_counter = m_auto_animation_frame_counter - 1;

	m_auto_animation_timer->adjust(m_screen->time_until_pos(NEOGEO_VSSTART));
}


void neosprite_base_device::create_auto_animation_timer()
{
	m_auto_animation_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(neosprite_base_device::auto_animation_timer_callback),this));
}


void neosprite_base_device::start_auto_animation_timer()
{
	m_auto_animation_timer->adjust(m_screen->time_until_pos(NEOGEO_VSSTART));
}



/*************************************
 *
 *  Fixed layer
 *
 *************************************/

void neosprite_base_device::neogeo_set_fixed_layer_source(uint8_t data)
{
	m_fixed_layer_source = data;
}


void neosprite_base_device::draw_fixed_layer(bitmap_rgb32 &bitmap, int scanline)
{
	int x;

	uint8_t* gfx_base = m_fixed_layer_source ? m_region_fixed : m_region_fixedbios->base();
	uint32_t addr_mask = ( m_fixed_layer_source ? m_region_fixed_size : m_region_fixedbios->bytes() ) - 1;
	uint16_t *video_data = &m_videoram_drawsource[0x7000 | (scanline >> 3)];
	uint32_t *pixel_addr = &bitmap.pix32(scanline, NEOGEO_HBEND);

	int garouoffsets[32];
	int banked = m_fixed_layer_source && (addr_mask > 0x1ffff);

	/* thanks to Mr K for the garou & kof2000 banking info */
	/* Build line banking table for Garou & MS3 before starting render */
	if (banked && m_fixed_layer_bank_type == 1)
	{
		int garoubank = 0;
		int k = 0;
		int y = 0;
		while (y < 32)
		{
			if (m_videoram_drawsource[0x7500 + k] == 0x0200 && (m_videoram_drawsource[0x7580 + k] & 0xff00) == 0xff00)
			{
				garoubank = m_videoram_drawsource[0x7580 + k] & 3;
				garouoffsets[y++] = garoubank;
			}
			garouoffsets[y++] = garoubank;
			k += 2;
		}
	}

	for (x = 0; x < 40; x++)
	{
		uint16_t code_and_palette = *video_data;
		uint16_t code = code_and_palette & 0x0fff;

		if (banked)
		{
			int y = scanline >> 3;
			switch (m_fixed_layer_bank_type)
			{
			case 1:
				/* Garou, MSlug 3 */
				code += 0x1000 * (garouoffsets[(y - 2) & 31] ^ 3);
				break;
			case 2:
				code += 0x1000 * (((m_videoram_drawsource[0x7500 + ((y - 1) & 31) + 32 * (x / 6)] >> (5 - (x % 6)) * 2) & 3) ^ 3);
				break;
			}
		}

		{
			int i;
			int gfx_offset = ((code << 5) | (scanline & 0x07)) & addr_mask;

			const pen_t *char_pens;

			char_pens = &m_pens[code_and_palette >> 12 << m_bppshift];


			static const uint32_t pix_offsets[] = { 0x10, 0x18, 0x00, 0x08 };

			for (i = 0; i < 4; i++)
			{
				draw_fixed_layer_2pixels(pixel_addr, gfx_offset + pix_offsets[i], gfx_base, char_pens);
			}
		}

		video_data = video_data + 0x20;
	}
}


inline void neosprite_base_device::draw_fixed_layer_2pixels(uint32_t*&pixel_addr, int offset, uint8_t* gfx_base, const pen_t* char_pens)
{
	uint8_t data = gfx_base[offset];

	if (data & 0x0f)
		*pixel_addr = char_pens[data & 0x0f];
	pixel_addr++;

	if (data & 0xf0)
		*pixel_addr = char_pens[(data & 0xf0) >> 4];
	pixel_addr++;

}

/*************************************
 *
 *  Sprite hardware
 *
 *************************************/

#define MAX_SPRITES_PER_SCREEN    (381)
#define MAX_SPRITES_PER_LINE      (96)


/* horizontal zoom table - verified on real hardware */
static const int zoom_x_tables[][16] =
{
	{ 0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0 },
	{ 0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0 },
	{ 0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0 },
	{ 0,0,1,0,1,0,0,0,1,0,0,0,1,0,0,0 },
	{ 0,0,1,0,1,0,0,0,1,0,0,0,1,0,1,0 },
	{ 0,0,1,0,1,0,1,0,1,0,0,0,1,0,1,0 },
	{ 0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0 },
	{ 1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0 },
	{ 1,0,1,0,1,0,1,0,1,1,1,0,1,0,1,0 },
	{ 1,0,1,1,1,0,1,0,1,1,1,0,1,0,1,0 },
	{ 1,0,1,1,1,0,1,0,1,1,1,0,1,0,1,1 },
	{ 1,0,1,1,1,0,1,1,1,1,1,0,1,0,1,1 },
	{ 1,0,1,1,1,0,1,1,1,1,1,0,1,1,1,1 },
	{ 1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1 },
	{ 1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1 },
	{ 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 }
};



inline bool neosprite_base_device::sprite_on_scanline(int scanline, int y, int rows)
{
	return (rows == 0) || (rows >= 0x20) || ((scanline - y) & 0x1ff) < (rows * 0x10);
}




void neosprite_base_device::draw_sprites(bitmap_rgb32 &bitmap, int scanline)
{
	int sprite_index;
	int max_sprite_index;

	int y = 0;
	int x = 0;
	int rows = 0;
	int zoom_y = 0;
	int zoom_x = 0;
	uint16_t *sprite_list;

	/* select the active list */
	if (scanline & 0x01)
		sprite_list = &m_videoram_drawsource[0x8680];
	else
		sprite_list = &m_videoram_drawsource[0x8600];

	/* optimization -- find last non-zero entry and only draw that many +1
	   sprite.  This is not 100% correct as the hardware will keep drawing
	   the #0 sprite over and over, but we need the speed */
	for (max_sprite_index = (MAX_SPRITES_PER_LINE - 1); max_sprite_index >= 0; max_sprite_index--)
	{
		if (sprite_list[max_sprite_index] != 0)
			break;
	}

	/* add the +1 now, just in case the 0 at the end is real sprite */
	if (max_sprite_index != (MAX_SPRITES_PER_LINE - 1))
		max_sprite_index = max_sprite_index + 1;

	for (sprite_index = 0; sprite_index <= max_sprite_index; sprite_index++)
	{
		uint16_t sprite_number = sprite_list[sprite_index] & 0x01ff;
		uint16_t y_control = m_videoram_drawsource[0x8200 | sprite_number];
		uint16_t zoom_control = m_videoram_drawsource[0x8000 | sprite_number];

		/* if chained, go to next X coordinate and get new X zoom */
		if (y_control & 0x40)
		{
			x = (x + zoom_x + 1) & 0x01ff;
			zoom_x = (zoom_control >> 8) & 0x0f;
		}
		/* new block */
		else
		{
			y = 0x200 - (y_control >> 7);
			x = m_videoram_drawsource[0x8400 | sprite_number] >> 7;

			zoom_y = (zoom_control & 0xff);

			zoom_x = (zoom_control >> 8) & 0x0f;
			rows = y_control & 0x3f;
		}

		/* skip if falls completely outside the screen */
		if ((x >= 0x140) && (x <= 0x1f0))
			continue;

		/* double check the Y coordinate, in case somebody modified the sprite coordinate
		   since we buffered it */
		if (sprite_on_scanline(scanline, y, rows))
		{
			int sprite_y;
			int tile;
			uint8_t sprite_y_and_tile;
			offs_t attr_and_code_offs;
			uint16_t attr;
			uint32_t code;
			const int *zoom_x_table;
			const pen_t *line_pens;
			int x_inc;

			int sprite_line = (scanline - y) & 0x1ff;
			int zoom_line = sprite_line & 0xff;
			int invert = sprite_line & 0x100;

			if (invert)
				zoom_line ^= 0xff;

			if (rows > 0x20)
			{
				zoom_line = zoom_line % ((zoom_y + 1) << 1);

				if (zoom_line > zoom_y)
				{
					zoom_line = ((zoom_y + 1) << 1) - 1 - zoom_line;
					invert = !invert;
				}
			}

			sprite_y_and_tile = m_region_zoomy[(zoom_y << 8) | zoom_line];

			sprite_y = sprite_y_and_tile & 0x0f;
			tile = sprite_y_and_tile >> 4;

			if (invert)
			{
				sprite_y ^= 0x0f;
				tile ^= 0x1f;
			}

			attr_and_code_offs = (sprite_number << 6) | (tile << 1);
			attr = m_videoram_drawsource[attr_and_code_offs + 1];
			code = ((attr << 12) & 0xf0000) | m_videoram_drawsource[attr_and_code_offs];

			/* substitute auto animation bits */
			if (!m_auto_animation_disabled)
			{
				if (attr & 0x0008)
					code = (code & ~0x07) | (m_auto_animation_counter & 0x07);
				else if (attr & 0x0004)
					code = (code & ~0x03) | (m_auto_animation_counter & 0x03);
			}

			/* vertical flip? */
			if (attr & 0x0002)
				sprite_y ^= 0x0f;

			zoom_x_table = zoom_x_tables[zoom_x];

			/* compute offset in gfx ROM and mask it to the number of bits available */
			int gfx_base = ((code << 8) | (sprite_y << 4)) & m_sprite_gfx_address_mask;


			line_pens = &m_pens[attr >> 8 << m_bppshift];


			/* horizontal flip? */
			if (attr & 0x0001)
			{
				gfx_base = gfx_base + 0x0f;
				x_inc = -1;
			}
			else
				x_inc = 1;

			/* draw the line - no wrap-around */
			if (x <= 0x01f0)
			{
				int i;

				uint32_t *pixel_addr = &bitmap.pix32(scanline, x + NEOGEO_HBEND);

				for (i = 0; i < 0x10; i++)
				{
					if (*zoom_x_table)
					{
						draw_pixel(gfx_base, pixel_addr, line_pens);

						pixel_addr++;
					}

					zoom_x_table++;
					gfx_base += x_inc;
				}
			}
			/* wrap-around */
			else
			{
				int i;

				int x_save = x;
				uint32_t *pixel_addr = &bitmap.pix32(scanline, NEOGEO_HBEND);

				for (i = 0; i < 0x10; i++)
				{
					if (*zoom_x_table)
					{
						if (x >= 0x200)
						{
							draw_pixel(gfx_base, pixel_addr, line_pens);

							pixel_addr++;
						}

						x++;
					}

					zoom_x_table++;
					gfx_base += x_inc;
				}

				x = x_save;
			}
		}
	}
}


void neosprite_base_device::parse_sprites(int scanline)
{
	uint16_t sprite_number;
	int y = 0;
	int rows = 0;
	uint16_t *sprite_list;

	int active_sprite_count = 0;

	/* select the active list */
	if (scanline & 0x01)
		sprite_list = &m_videoram_drawsource[0x8680];
	else
		sprite_list = &m_videoram_drawsource[0x8600];

	/* scan all sprites */
	for (sprite_number = 0; sprite_number < MAX_SPRITES_PER_SCREEN; sprite_number++)
	{
		uint16_t y_control = m_videoram_drawsource[0x8200 | sprite_number];

		/* if not chained, get Y position and height, otherwise use previous values */
		if (~y_control & 0x40)
		{
			y = 0x200 - (y_control >> 7);
			rows = y_control & 0x3f;
		}

		/* skip sprites with 0 rows */
		if (rows == 0)
			continue;

		if (!sprite_on_scanline(scanline, y, rows))
			continue;

		/* sprite is on this scanline, add it to active list */
		*sprite_list = sprite_number;

		sprite_list++;

		/* increment sprite count, and if we reached the max, bail out */
		active_sprite_count++;

		if (active_sprite_count == MAX_SPRITES_PER_LINE)
			break;
	}

	/* fill the rest of the sprite list with 0, including one extra entry */
	memset(sprite_list, 0, sizeof(sprite_list[0]) * (MAX_SPRITES_PER_LINE - active_sprite_count + 1));
}


TIMER_CALLBACK_MEMBER(neosprite_base_device::sprite_line_timer_callback)
{
	int scanline = param;

	/* we are at the beginning of a scanline -
	   we need to draw the previous scanline and parse the sprites on the current one */
	if (scanline != 0)
		m_screen->update_partial(scanline - 1);

	parse_sprites(scanline);

	/* let's come back at the beginning of the next line */
	scanline = (scanline + 1) % NEOGEO_VTOTAL;

	m_sprite_line_timer->adjust(m_screen->time_until_pos(scanline), scanline);
}


void neosprite_base_device::create_sprite_line_timer()
{
	m_sprite_line_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(neosprite_base_device::sprite_line_timer_callback),this));
}


void neosprite_base_device::start_sprite_line_timer()
{
	m_sprite_line_timer->adjust(m_screen->time_until_pos(0));
}


uint32_t neosprite_base_device::get_region_mask(uint8_t* rgn, uint32_t rgn_size)
{
	/* convert the sprite graphics data into a format that
	   allows faster blitting */
	uint32_t mask;
	uint32_t len;
	uint32_t bit;

	/* get mask based on the length rounded up to the nearest
	   power of 2 */
	mask = 0xffffffff;

	len = rgn_size;

	for (bit = 0x80000000; bit != 0; bit >>= 1)
	{
		if ((len * 2 - 1) & bit)
			break;

		mask >>= 1;
	}

	return mask;
}

void neosprite_base_device::optimize_sprite_data()
{
	// this does nothing in this implementation, it's used by neosprite_optimized_device
	// m_sprite_gfx_address_mask gets set when the GFX region is assigned
	return;
}

void neosprite_base_device::set_optimized_sprite_data(uint8_t* sprdata, uint32_t mask)
{
	return;
}

// these are for passing in pointers from the main system
void neosprite_base_device::set_sprite_region(uint8_t* region_sprites, uint32_t region_sprites_size)
{
	m_region_sprites = region_sprites;
	m_region_sprites_size = region_sprites_size;
}

void neosprite_base_device::set_fixed_regions(uint8_t* fix_cart, uint32_t fix_cart_size, memory_region* fix_bios)
{
	m_region_fixed = fix_cart;
	m_region_fixed_size = fix_cart_size;
	m_region_fixedbios = fix_bios;
}

void neosprite_base_device::set_screen(screen_device* screen)
{
	m_screen = screen;
}

void neosprite_base_device::set_pens(const pen_t* pens)
{
	m_pens = pens;
}



/*********************************************************************************************************************************/
/* Regular NeoGeo sprite handling - drawing directly from ROM (or RAM on NeoCD)                                                  */
/*                                                                                                                               */
/* note, we don't currently use this implementation, reference only                                                              */
/* if we do it will be important to ensure that all sprite regions are ^2 sizes - the optimized routine automatically allocates  */
/* ^2 sized regions when pre-decoding, but obviously we don't here, so if we want to be safe we'll have to adjust the actual     */
/* regions          (alternatively I could add an additional size check in the draw routine, but that would be slower)           */
/*********************************************************************************************************************************/

DEFINE_DEVICE_TYPE(NEOGEO_SPRITE_REGULAR, neosprite_regular_device, "neosprite_reg", "Neo-Geo Sprites (regular)")

neosprite_regular_device::neosprite_regular_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: neosprite_base_device(mconfig, NEOGEO_SPRITE_REGULAR, tag, owner, clock)
{
}

void neosprite_regular_device::set_sprite_region(uint8_t* region_sprites, uint32_t region_sprites_size)
{
	m_region_sprites = region_sprites;
	m_region_sprites_size = region_sprites_size;

	uint32_t mask = get_region_mask(m_region_sprites, m_region_sprites_size);
	uint32_t proper_size = (mask + 1) >>1;

	printf("lengths %08x %08x m_region_sprites", region_sprites_size, proper_size);

	if (m_region_sprites_size != proper_size)
	{
		fatalerror("please use power of 2 region sizes with neosprite_base_device to ensure masking works correctly");
	}

	m_sprite_gfx_address_mask = mask;
}

inline void neosprite_regular_device::draw_pixel(int romaddr, uint32_t* dst, const pen_t *line_pens)
{
	const uint8_t* src = m_region_sprites + (((romaddr &~0xff)>>1) | (((romaddr&0x8)^0x8)<<3) | ((romaddr & 0xf0)  >> 2));
	const int x = romaddr & 0x7;

	const uint8_t gfx = (((src[0x3] >> x) & 0x01) << 3) |
						(((src[0x1] >> x) & 0x01) << 2) |
						(((src[0x2] >> x) & 0x01) << 1) |
						(((src[0x0] >> x) & 0x01) << 0);

	if (gfx)
		*dst = line_pens[gfx];
}



/*********************************************************************************************************************************/
/* Regular NeoGeo sprite handling with pre-decode optimization                                                                   */
/*                                                                                                                               */
/* this is closer to the old MAME implementation where the 4bpp graphics have been expanded to an easier to draw 8bpp format     */
/* for additional speed                                                                                                          */
/*********************************************************************************************************************************/

DEFINE_DEVICE_TYPE(NEOGEO_SPRITE_OPTIMZIED, neosprite_optimized_device, "neosprite_opt", "Neo-Geo Sprites (optimized)")

neosprite_optimized_device::neosprite_optimized_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: neosprite_base_device(mconfig, NEOGEO_SPRITE_OPTIMZIED, tag, owner, clock)
	, m_spritegfx8(nullptr)
{
}

uint32_t neosprite_optimized_device::optimize_helper(std::vector<uint8_t> &spritegfx, uint8_t* region_sprites, uint32_t region_sprites_size)
{
	// convert the sprite graphics data into a format that allows faster blitting
	uint8_t *src;
	uint8_t *dest;

	uint32_t mask = get_region_mask(region_sprites, region_sprites_size);

	spritegfx.resize(mask + 1);
	uint32_t spritegfx_address_mask = mask;

	src = region_sprites;
	dest = &spritegfx[0];

	for (unsigned i = 0; i < region_sprites_size; i += 0x80, src += 0x80)
	{
		for (unsigned y = 0; y < 0x10; y++)
		{
			for (unsigned x = 0; x < 8; x++)
			{
				*(dest++) = (((src[0x43 | (y << 2)] >> x) & 0x01) << 3) |
				(((src[0x41 | (y << 2)] >> x) & 0x01) << 2) |
				(((src[0x42 | (y << 2)] >> x) & 0x01) << 1) |
				(((src[0x40 | (y << 2)] >> x) & 0x01) << 0);
			}

			for (unsigned x = 0; x < 8; x++)
			{
				*(dest++) = (((src[0x03 | (y << 2)] >> x) & 0x01) << 3) |
				(((src[0x01 | (y << 2)] >> x) & 0x01) << 2) |
				(((src[0x02 | (y << 2)] >> x) & 0x01) << 1) |
				(((src[0x00 | (y << 2)] >> x) & 0x01) << 0);
			}
		}
	}

	return spritegfx_address_mask;
}


void neosprite_optimized_device::optimize_sprite_data()
{
	m_sprite_gfx_address_mask = optimize_helper(m_sprite_gfx, m_region_sprites, m_region_sprites_size);
	m_spritegfx8 = &m_sprite_gfx[0];
}

void neosprite_optimized_device::set_optimized_sprite_data(uint8_t* sprdata, uint32_t mask)
{
	m_sprite_gfx_address_mask = mask;
	m_spritegfx8 = &sprdata[0];
}

inline void neosprite_optimized_device::draw_pixel(int romaddr, uint32_t* dst, const pen_t *line_pens)
{
	const uint8_t gfx = m_spritegfx8[romaddr];

	if (gfx)
		*dst = line_pens[gfx];
}


/*********************************************************************************************************************************/
/* MIDAS specific sprite handling                                                                                                */
/*                                                                                                                               */
/* this is used by the midas.c hardware which is a reengineered NeoGeo, it has 8bbp tiles instead of 4bpp tiles                  */
/* and uploads the zoom table.  The additional videoram buffering is a guess because 'hammer' is very glitchy without it         */
/*********************************************************************************************************************************/

DEFINE_DEVICE_TYPE(NEOGEO_SPRITE_MIDAS, neosprite_midas_device, "midassprite", "MIDAS Sprites")


neosprite_midas_device::neosprite_midas_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: neosprite_base_device(mconfig, NEOGEO_SPRITE_MIDAS, tag, owner, clock)
{
	m_bppshift = 8;
}


inline void neosprite_midas_device::draw_pixel(int romaddr, uint32_t* dst, const pen_t *line_pens)
{
	const uint8_t* src = m_region_sprites + (((romaddr &~0xff)) | (((romaddr&0x8)^0x8)<<4) | ((romaddr & 0xf0)  >> 1));
	const int x = romaddr & 0x7;

	const uint8_t gfx =   (((src[0x7] >> x) & 0x01) << 7) |
						(((src[0x6] >> x) & 0x01) << 6) |
						(((src[0x5] >> x) & 0x01) << 5) |
						(((src[0x4] >> x) & 0x01) << 4) |
						(((src[0x3] >> x) & 0x01) << 3) |
						(((src[0x2] >> x) & 0x01) << 2) |
						(((src[0x1] >> x) & 0x01) << 1) |
						(((src[0x0] >> x) & 0x01) << 0);

	if (gfx)
		*dst = line_pens[gfx];
}


void neosprite_midas_device::device_start()
{
	neosprite_base_device::device_start();

	m_videoram_buffer = std::make_unique<uint16_t[]>(0x8000 + 0x800);
	m_videoram_drawsource = m_videoram_buffer.get();

	memset(m_videoram_buffer.get(), 0x00, (0x8000 + 0x800) * sizeof(uint16_t));

}

void neosprite_midas_device::buffer_vram()
{
	memcpy(m_videoram_buffer.get(), m_videoram.get(), (0x8000 + 0x800) * sizeof(uint16_t));
}

inline void neosprite_midas_device::draw_fixed_layer_2pixels(uint32_t*&pixel_addr, int offset, uint8_t* gfx_base, const pen_t* char_pens)
{
	uint8_t data;

	data = ((gfx_base[(offset * 2)+0] & 0x0f)<<0) | ((gfx_base[(offset * 2)+1] & 0x0f)<<4);
	if (data)
		*pixel_addr = char_pens[data];
	pixel_addr++;

	data = ((gfx_base[(offset * 2)+0] & 0xf0)>>4) | ((gfx_base[(offset * 2)+1] & 0xf0)<<0);
	if (data)
		*pixel_addr = char_pens[data];
	pixel_addr++;
}

void neosprite_midas_device::set_sprite_region(uint8_t* region_sprites, uint32_t region_sprites_size)
{
	m_region_sprites = region_sprites;
	m_region_sprites_size = region_sprites_size;
	uint32_t mask = get_region_mask(m_region_sprites, m_region_sprites_size);
	m_sprite_gfx_address_mask = mask;
}
