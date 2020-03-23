// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    atarirle.h

    Common RLE-based motion object management functions for early 90's
    Atari raster games.

***************************************************************************/

#ifndef MAME_VIDEO_ATARIRLE_H
#define MAME_VIDEO_ATARIRLE_H


//**************************************************************************
//  DEVICE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_ATARIRLE_ADD(_tag, _interface) \
	MCFG_DEVICE_ADD(_tag, ATARI_RLE_OBJECTS, 0) \
	atari_rle_objects_device::static_set_config(*device, _interface);



//**************************************************************************
//  CONSTANTS
//**************************************************************************

#define ATARIRLE_PRIORITY_SHIFT     12
#define ATARIRLE_BANK_SHIFT         15
#define ATARIRLE_PRIORITY_MASK      ((0xffff << ATARIRLE_PRIORITY_SHIFT) & 0xffff)
#define ATARIRLE_DATA_MASK          (ATARIRLE_PRIORITY_MASK ^ 0xffff)

#define ATARIRLE_CONTROL_MOGO       1
#define ATARIRLE_CONTROL_ERASE      2
#define ATARIRLE_CONTROL_FRAME      4

#define ATARIRLE_COMMAND_NOP        0
#define ATARIRLE_COMMAND_DRAW       1
#define ATARIRLE_COMMAND_CHECKSUM   2



//**************************************************************************
//  TYPES & STRUCTURES
//**************************************************************************

// description of the motion objects
struct atari_rle_objects_config
{
	struct entry { uint16_t data[8]; };

	uint16_t          m_leftclip;           // left clip coordinate
	uint16_t          m_rightclip;          // right clip coordinate
	uint16_t          m_palettebase;        // base palette entry

	entry           m_code_entry;           // mask for the code index
	entry           m_color_entry;          // mask for the color
	entry           m_xpos_entry;           // mask for the X position
	entry           m_ypos_entry;           // mask for the Y position
	entry           m_scale_entry;          // mask for the scale factor
	entry           m_hflip_entry;          // mask for the horizontal flip
	entry           m_order_entry;          // mask for the order
	entry           m_priority_entry;       // mask for the priority
	entry           m_vram_entry;           // mask for the VRAM target
};


// ======================> atari_rle_objects_device

// device type definition
DECLARE_DEVICE_TYPE(ATARI_RLE_OBJECTS, atari_rle_objects_device)

class atari_rle_objects_device : public device_t,
									public device_video_interface,
									public atari_rle_objects_config
{
public:
	// construction/destruction
	atari_rle_objects_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration helpers
	static void static_set_config(device_t &device, const atari_rle_objects_config &config);

	// control handlers
	DECLARE_WRITE8_MEMBER(control_write);
	DECLARE_WRITE8_MEMBER(command_write);

	// render helpers
	void vblank_callback(screen_device &screen, bool state);

	// getters
	bitmap_ind16 &vram(int idx) { return m_vram[idx][(m_control_bits & ATARIRLE_CONTROL_FRAME) >> 2]; }

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	// a sprite parameter, which is a word index + shift + mask
	class sprite_parameter
	{
	public:
		sprite_parameter();
		bool set(const atari_rle_objects_config::entry &input) { return set(input.data); }
		bool set(const uint16_t input[8]);
		uint16_t extract(memory_array &array, int offset) const { return (array.read(offset + m_word) >> m_shift) & m_mask; }
		uint16_t shift() const { return m_shift; }
		uint16_t mask() const { return m_mask; }

	private:
		uint16_t          m_word;             // word index
		uint16_t          m_shift;            // shift amount
		uint16_t          m_mask;             // final mask
	};

	// internal structure describing each object in the ROMs
	struct object_info
	{
		int16_t           width;
		int16_t           height;
		int16_t           xoffs;
		int16_t           yoffs;
		uint8_t           bpp;
		const uint16_t *  table;
		const uint16_t *  data;
	};

	// internal helpers
	inline int round_to_powerof2(int value);
	void build_rle_tables();
	int count_objects();
	void prescan_rle(int which);
	void compute_checksum();
	void sort_and_render();
	void draw_rle(bitmap_ind16 &bitmap, const rectangle &clip, int code, int color, int hflip, int vflip, int x, int y, int xscale, int yscale);
	void draw_rle_zoom(bitmap_ind16 &bitmap, const rectangle &clip, const object_info &info, uint32_t palette, int sx, int sy, int scalex, int scaley);
	void draw_rle_zoom_hflip(bitmap_ind16 &bitmap, const rectangle &clip, const object_info &info, uint32_t palette, int sx, int sy, int scalex, int scaley);
	void hilite_object(bitmap_ind16 &bitmap, int hilite);

	// derived state
	int             m_bitmapwidth;        // width of the full playfield bitmap
	int             m_bitmapheight;       // height of the full playfield bitmap
	int             m_bitmapxmask;        // x coordinate mask for the playfield bitmap
	int             m_bitmapymask;        // y coordinate mask for the playfield bitmap
	rectangle       m_cliprect;           // clipping rectangle

	// masks
	sprite_parameter    m_codemask;           // mask for the code index
	sprite_parameter    m_colormask;          // mask for the color
	sprite_parameter    m_xposmask;           // mask for the X position
	sprite_parameter    m_yposmask;           // mask for the Y position
	sprite_parameter    m_scalemask;          // mask for the scale factor
	sprite_parameter    m_hflipmask;          // mask for the horizontal flip
	sprite_parameter    m_ordermask;          // mask for the order
	sprite_parameter    m_prioritymask;       // mask for the priority
	sprite_parameter    m_vrammask;           // mask for the VRAM target

	// ROM information
	required_region_ptr<uint16_t> m_rombase;    // pointer to the base of the GFX ROM
	int                 m_objectcount;        // number of objects in the ROM
	std::vector<object_info> m_info;               // list of info records

	// rendering state
	bitmap_ind16        m_vram[2][2];         // pointers to VRAM bitmaps and backbuffers
	int                 m_partial_scanline;   // partial update scanline

	// control state
	uint8_t               m_control_bits;       // current control bits
	uint8_t               m_command;            // current command
	uint16_t              m_checksums[256];     // checksums for each 0x40000 bytes
	memory_array        m_ram;

	// tables
	uint8_t               m_rle_bpp[8];
	uint16_t *            m_rle_table[8];
	uint16_t              m_rle_table_data[0x500];
};

#endif // MAME_VIDEO_ATARIRLE_H
