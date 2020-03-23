// license:BSD-3-Clause
// copyright-holders:Nathan Woods,Frank Palazzolo
/*****************************************************************************
 *
 * includes/stic.h
 *
 ****************************************************************************/

#ifndef MAME_VIDEO_STIC_H
#define MAME_VIDEO_STIC_H

// the Intellivision emulation scales to match the output format at the last
// step. The Intellivision keyboard component appears to be 320x96, but can
// also run Intellivision carts, so x-coordinates are conditionally scaled
// by 2.
#define INTV_X_SCALE                1
#define INTV_Y_SCALE                1
#define INTVKBD_X_SCALE             2
#define INTVKBD_Y_SCALE             INTV_Y_SCALE

struct intv_sprite_type
{
	int visible;
	int xpos;
	int ypos;
	int coll;
	int collision;
	int doublex;
	int doubley;
	int quady;
	int xflip;
	int yflip;
	int behind_foreground;
	int grom;
	int card;
	int color;
	int doubleyres;
	int dirty;
};


// ======================> stic_device

class stic_device :  public device_t
{
public:
	// GROM/GRAM cards are 8x8
	static constexpr unsigned   CARD_WIDTH              = 8;
	static constexpr unsigned   CARD_HEIGHT             = 8;

	// Intellivision resolution is 20x12 BACKTAB CARDs, minus the rightmost column,
	// for an effective resolution of (19 * 8 + 1 * 7) x (12 * 8) == 159x96.
	//
	// MOB scanline height can be half of a card scanline height, so y-coordinates
	// are scaled by 2.
	static constexpr unsigned   X_SCALE                 = 1;
	static constexpr unsigned   Y_SCALE                 = 2;

	// overscan sizes in intv pixels
	// these values are approximate.
	static constexpr unsigned   OVERSCAN_LEFT_WIDTH     = 13;
	static constexpr unsigned   OVERSCAN_RIGHT_WIDTH    = 17;
	static constexpr unsigned   OVERSCAN_TOP_HEIGHT     = 12;
	static constexpr unsigned   OVERSCAN_BOTTOM_HEIGHT  = 12;

	//Timing constants based on  Joe Zbiciak's documentation
	static constexpr int        CYCLES_PER_SCANLINE     = 57;
	static constexpr int        ROW_BUSRQ               = 110; // CPU paused during backtab row buffering
	static constexpr int        FRAME_BUSRQ             = 42; // CPU paused just after end of vblank and before first row fetch (approximate)
	static constexpr int        VBLANK_END              = 3790;
	static constexpr int        FIRST_FETCH             = 3933;

	/*** BACKTAB ****************************************************************/

	// BACKTAB is made up of 20x12 cards
	// (the first 19 columns are 8x8, the 20th column is 7x8)
	static constexpr unsigned   BACKTAB_WIDTH           = 20;
	static constexpr unsigned   BACKTAB_HEIGHT          = 12;

	static constexpr unsigned   SCREEN_WIDTH            = (OVERSCAN_LEFT_WIDTH + (BACKTAB_WIDTH * CARD_WIDTH) - 1 + OVERSCAN_RIGHT_WIDTH) * X_SCALE;
	static constexpr unsigned   SCREEN_HEIGHT           = (OVERSCAN_TOP_HEIGHT + (BACKTAB_HEIGHT * CARD_HEIGHT) + OVERSCAN_BOTTOM_HEIGHT) * Y_SCALE;

	// construction/destruction
	stic_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~stic_device();

	DECLARE_READ16_MEMBER(read);
	DECLARE_READ16_MEMBER(gram_read);
	DECLARE_READ16_MEMBER(grom_read) { if (offset > 0x800) printf("help! %X\n", offset); return (0xff00 | m_grom[offset]); }
	DECLARE_WRITE16_MEMBER(write);
	DECLARE_WRITE16_MEMBER(gram_write);

	void write_to_btb(int h, int w, uint16_t data) { m_backtab_buffer[h][w] = data; }
	int read_row_delay() { return m_row_delay; }
	int read_stic_handshake() { return m_stic_handshake; }
	void set_x_scale(int val) { m_x_scale = val; }
	void set_y_scale(int val) { m_y_scale = val; }

	// device-level overrides
	virtual void device_start() override;
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_reset() override;

	void screenrefresh();
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

private:
	/*** STIC registers *********************************************************/

	// number of STIC registers
	static constexpr unsigned   STIC_REGISTERS  = 0x33;

	// STIC MOBs (Moveable OBjects)
	enum
	{
		MOB0,
		MOB1,
		MOB2,
		MOB3,
		MOB4,
		MOB5,
		MOB6,
		MOB7,

		MOBS
	};

	// STIC Color Stack
	enum
	{
		CSR0,
		CSR1,
		CSR2,
		CSR3,

		CSRS
	};


	required_region_ptr<uint8_t> m_grom;

	void intv_set_pixel(bitmap_ind16 &bitmap, int x, int y, uint32_t color);
	uint32_t intv_get_pixel(bitmap_ind16 &bitmap, int x, int y);
	void intv_plot_box(bitmap_ind16 &bm, int x, int y, int w, int h, int color);
	bool sprites_collide(int spriteNum1, int spriteNum2);
	void determine_sprite_collisions();
	void render_sprites();
	void render_line(bitmap_ind16 &bitmap, uint8_t nextByte, uint16_t x, uint16_t y, uint8_t fgcolor, uint8_t bgcolor);
	void render_colored_squares(bitmap_ind16 &bitmap, uint16_t x, uint16_t y, uint8_t color0, uint8_t color1, uint8_t color2, uint8_t color3);
	void render_color_stack_mode(bitmap_ind16 &bitmap);
	void render_fg_bg_mode(bitmap_ind16 &bitmap);
	void copy_sprites_to_background(bitmap_ind16 &bitmap);
	void render_background(bitmap_ind16 &bitmap);
	void draw_borders(bitmap_ind16 &bitmap);

#ifdef UNUSED_CODE
	void draw_background(bitmap_ind16 &bitmap, int transparency);
	void draw_sprites(bitmap_ind16 &bitmap, int behind_foreground);
#endif

	bitmap_ind16 m_bitmap;

	intv_sprite_type m_sprite[MOBS];
	uint8_t m_sprite_buffers[MOBS][CARD_WIDTH * 2][CARD_HEIGHT * 4 * 2 * 2];
	uint16_t m_backtab_buffer[BACKTAB_HEIGHT][BACKTAB_WIDTH];
	int m_color_stack_mode;
	int m_stic_registers[STIC_REGISTERS];
	int m_color_stack_offset;
	int m_stic_handshake;
	int m_border_color;
	int m_col_delay;
	int m_row_delay;
	int m_left_edge_inhibit;
	int m_top_edge_inhibit;
	int m_x_scale;
	int m_y_scale;

	uint8_t m_gramdirty;
	uint8_t m_gram[512];
	uint8_t m_gramdirtybytes[512];
};

// device type definition
DECLARE_DEVICE_TYPE(STIC, stic_device)


/***************************************************************************
 DEVICE CONFIGURATION MACROS
 ***************************************************************************/

#define MCFG_STIC_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, STIC, 0)


#endif // MAME_VIDEO_STIC_H
