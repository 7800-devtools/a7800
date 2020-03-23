// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/***************************************************************************

    IBM PC junior
    Tandy 1000 Graphics Adapter (T1T) section

    Note that in the IBM PC Junior world, the term 'vga' is not the 'vga' that
    most people think of

***************************************************************************/

#include "emu.h"
#include "pc_t1t.h"

#include "machine/pic8259.h"
#include "machine/ram.h"
#include "screen.h"


enum
{
	T1000_TEXT_INTEN = 0,
	T1000_TEXT_BLINK,
	T1000_GFX_1BPP,
	T1000_GFX_2BPP,
	T1000_GFX_4BPP,
	T1000_GFX_2BPP_TGA,
	PCJX_TEXT,
	PCJR_GFX_2BPP_HIGH
};


DEFINE_DEVICE_TYPE(PCVIDEO_T1000, pcvideo_t1000_device, "tandy_1000_graphics", "Tandy 1000 Graphics Adapter")
DEFINE_DEVICE_TYPE(PCVIDEO_PCJR,  pcvideo_pcjr_device,  "pcjr_graphics",       "PC Jr Graphics Adapter")

pc_t1t_device::pc_t1t_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock),
	device_video_interface(mconfig, *this),
	m_mc6845(*this, T1000_MC6845_NAME),
	m_mode_control(0),
	m_color_select(0),
	m_status(0),
	m_bank(0),
	m_pc_framecnt(0),
	m_displayram(nullptr),
	m_chr_gen(nullptr),
	m_chr_size(0),
	m_ra_offset(0),
	m_address_data_ff(0),
	m_update_row_type(-1),
	m_display_enable(0),
	m_vsync(0),
	m_palette_base(0),
	m_palette(*this,"palette"),
	m_ram(*this, ":" RAM_TAG),
	m_vram(*this, "vram")
{
}

pcvideo_t1000_device::pcvideo_t1000_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: pc_t1t_device(mconfig, PCVIDEO_T1000, tag, owner, clock)
{
}

pcvideo_pcjr_device::pcvideo_pcjr_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: pc_t1t_device(mconfig, PCVIDEO_PCJR, tag, owner, clock),
	m_jxkanji(nullptr)
{
}


void pcvideo_t1000_device::device_start()
{
	if(!m_ram->started())
		throw device_missing_dependencies();
	m_chr_gen = machine().root_device().memregion("gfx1")->base();
	m_bank = 0;
	m_chr_size = 1;
	m_ra_offset = 256;
	m_vram->space(0).install_ram(0, 128*1024 - 1, m_ram->pointer());
}


void pcvideo_pcjr_device::device_start()
{
	if(!m_ram->started())
		throw device_missing_dependencies();
	m_chr_gen = machine().root_device().memregion("gfx1")->base();
	m_bank = 0;
	m_mode_control = 0x08;
	m_chr_size = 8;
	m_ra_offset = 1;
	if(!strncmp(machine().system().name, "ibmpcjx", 7))
	{
		m_jxkanji = machine().root_device().memregion("kanji")->base();
		m_vram->space(0).install_ram(0, 128*1024 - 1, memshare(":vram")->ptr()); // TODO: fix when this is really understood
	}
	else
	{
		m_jxkanji = nullptr;
		m_vram->space(0).install_ram(0, 128*1024 - 1, m_ram->pointer());
	}
}


/***************************************************************************

    Static declarations

***************************************************************************/

static ADDRESS_MAP_START(vram_map, 0, 8, pcvideo_t1000_device)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x20000, 0x3ffff) AM_NOP
ADDRESS_MAP_END

MACHINE_CONFIG_MEMBER( pcvideo_t1000_device::device_add_mconfig )
	MCFG_SCREEN_ADD(T1000_SCREEN_NAME, RASTER)
	MCFG_SCREEN_RAW_PARAMS(XTAL_14_31818MHz,912,0,640,262,0,200)
	MCFG_SCREEN_UPDATE_DEVICE( T1000_MC6845_NAME, mc6845_device, screen_update )

	MCFG_PALETTE_ADD( "palette", 32 )
	MCFG_PALETTE_INIT_OWNER(pc_t1t_device, pcjr)

	MCFG_MC6845_ADD(T1000_MC6845_NAME, MC6845, T1000_SCREEN_NAME, XTAL_14_31818MHz/8)
	MCFG_MC6845_SHOW_BORDER_AREA(false)
	MCFG_MC6845_CHAR_WIDTH(8)
	MCFG_MC6845_UPDATE_ROW_CB(pc_t1t_device, crtc_update_row)
	MCFG_MC6845_OUT_DE_CB(WRITELINE(pc_t1t_device, t1000_de_changed))
	MCFG_MC6845_OUT_VSYNC_CB(WRITELINE(pcvideo_t1000_device, t1000_vsync_changed))

	MCFG_DEVICE_ADD("vram", ADDRESS_MAP_BANK, 0)
	MCFG_DEVICE_PROGRAM_MAP(vram_map)
	MCFG_ADDRESS_MAP_BANK_ENDIANNESS(ENDIANNESS_LITTLE)
	MCFG_ADDRESS_MAP_BANK_DATABUS_WIDTH(8)
	MCFG_ADDRESS_MAP_BANK_ADDRBUS_WIDTH(18)
	MCFG_ADDRESS_MAP_BANK_STRIDE(0x4000)
MACHINE_CONFIG_END


MACHINE_CONFIG_MEMBER( pcvideo_pcjr_device::device_add_mconfig )
	MCFG_SCREEN_ADD(T1000_SCREEN_NAME, RASTER)
	MCFG_SCREEN_RAW_PARAMS(XTAL_14_31818MHz, 912, 0, 640, 262, 0, 200)
	MCFG_SCREEN_UPDATE_DEVICE( T1000_MC6845_NAME, mc6845_device, screen_update )

	MCFG_PALETTE_ADD( "palette", 32 )
	MCFG_PALETTE_INIT_OWNER(pc_t1t_device, pcjr)

	MCFG_MC6845_ADD(T1000_MC6845_NAME, MC6845, T1000_SCREEN_NAME, XTAL_14_31818MHz/16)
	MCFG_MC6845_SHOW_BORDER_AREA(false)
	MCFG_MC6845_CHAR_WIDTH(8)
	MCFG_MC6845_UPDATE_ROW_CB(pcvideo_pcjr_device, crtc_update_row)
	MCFG_MC6845_OUT_DE_CB(WRITELINE(pc_t1t_device, t1000_de_changed))
	MCFG_MC6845_OUT_VSYNC_CB(WRITELINE(pcvideo_pcjr_device, pcjr_vsync_changed))
	MCFG_DEVICE_ADD("vram", ADDRESS_MAP_BANK, 0)
	MCFG_DEVICE_PROGRAM_MAP(vram_map)
	MCFG_ADDRESS_MAP_BANK_ENDIANNESS(ENDIANNESS_LITTLE)
	MCFG_ADDRESS_MAP_BANK_DATABUS_WIDTH(8)
	MCFG_ADDRESS_MAP_BANK_ADDRBUS_WIDTH(18)
	MCFG_ADDRESS_MAP_BANK_STRIDE(0x4000)
MACHINE_CONFIG_END


/***************************************************************************

    Methods

***************************************************************************/

/* Initialise the cga palette */
PALETTE_INIT_MEMBER( pc_t1t_device, pcjr )
{
	const static unsigned char tga_palette[16][3] =
	{
		{ 0x00, 0x00, 0x00 }, { 0x00, 0x00, 0xaa }, { 0x00, 0xaa, 0x00 }, { 0x00, 0xaa, 0xaa },
		{ 0xaa, 0x00, 0x00 }, { 0xaa, 0x00, 0xaa }, { 0xaa, 0x55, 0x00 }, { 0xaa, 0xaa, 0xaa },
		{ 0x55, 0x55, 0x55 }, { 0x55, 0x55, 0xff }, { 0x55, 0xff, 0x55 }, { 0x55, 0xff, 0xff },
		{ 0xff, 0x55, 0x55 }, { 0xff, 0x55, 0xff }, { 0xff, 0xff, 0x55 }, { 0xff, 0xff, 0xff }
	};
	int i;

	/* colors */
	for(i = 0; i < 16; i++)
		palette.set_pen_color(i, tga_palette[i][0], tga_palette[i][1], tga_palette[i][2]);

	/* b/w mode shades */
	for(i = 0; i < 16; i++)
		palette.set_pen_color(16+i, ( i << 4 ) | i, ( i << 4 ) | i, ( i << 4 ) | i );
}


MC6845_UPDATE_ROW( pc_t1t_device::t1000_text_inten_update_row )
{
	const rgb_t *palette = m_palette->palette()->entry_list_raw();
	uint32_t  *p = &bitmap.pix32(y);
	int i;

	if ( y == 0 ) logerror("t1000_text_inten_update_row\n");
	for ( i = 0; i < x_count; i++ )
	{
		uint16_t offset = ( ( ma + i ) << 1 ) & 0x3fff;
		uint8_t chr = m_displayram[ offset ];
		uint8_t attr = m_displayram[ offset +1 ];
		uint8_t data = m_chr_gen[ chr * m_chr_size + ra * m_ra_offset ];
		uint16_t fg = m_palette_base + ( attr & 0x0F );
		uint16_t bg = m_palette_base + ( ( attr >> 4 ) & 0x07 );

		if ( i == cursor_x && ( m_pc_framecnt & 0x08 ) )
		{
			data = 0xFF;
		}

		*p = palette[( data & 0x80 ) ? fg : bg]; p++;
		*p = palette[( data & 0x40 ) ? fg : bg]; p++;
		*p = palette[( data & 0x20 ) ? fg : bg]; p++;
		*p = palette[( data & 0x10 ) ? fg : bg]; p++;
		*p = palette[( data & 0x08 ) ? fg : bg]; p++;
		*p = palette[( data & 0x04 ) ? fg : bg]; p++;
		*p = palette[( data & 0x02 ) ? fg : bg]; p++;
		*p = palette[( data & 0x01 ) ? fg : bg]; p++;
	}
}


MC6845_UPDATE_ROW( pc_t1t_device::t1000_text_blink_update_row )
{
	const rgb_t *palette = m_palette->palette()->entry_list_raw();
	uint32_t  *p = &bitmap.pix32(y);
	int i;

	for ( i = 0; i < x_count; i++ )
	{
		uint16_t offset = ( ( ma + i ) << 1 ) & 0x3fff;
		uint8_t chr = m_displayram[ offset ];
		uint8_t attr = m_displayram[ offset +1 ];
		uint8_t data = m_chr_gen[ chr * m_chr_size + ra * m_ra_offset ];
		uint16_t fg = m_palette_base + ( attr & 0x0F );
		uint16_t bg = m_palette_base + ( ( attr >> 4 ) & 0x07 );

		if ( i == cursor_x )
		{
			if ( m_pc_framecnt & 0x08 )
			{
				data = 0xFF;
			}
		}
		else
		{
			if ( ( attr & 0x80 ) && ( m_pc_framecnt & 0x10 ) )
			{
				data = 0x00;
			}
		}

		*p = palette[( data & 0x80 ) ? fg : bg]; p++;
		*p = palette[( data & 0x40 ) ? fg : bg]; p++;
		*p = palette[( data & 0x20 ) ? fg : bg]; p++;
		*p = palette[( data & 0x10 ) ? fg : bg]; p++;
		*p = palette[( data & 0x08 ) ? fg : bg]; p++;
		*p = palette[( data & 0x04 ) ? fg : bg]; p++;
		*p = palette[( data & 0x02 ) ? fg : bg]; p++;
		*p = palette[( data & 0x01 ) ? fg : bg]; p++;
	}
}

MC6845_UPDATE_ROW( pcvideo_pcjr_device::pcjx_text_update_row )
{
	const rgb_t *palette = m_palette->palette()->entry_list_raw();
	uint32_t  *p = &bitmap.pix32(y);
	int i;

	for ( i = 0; i < x_count; i++ )
	{
		uint16_t offset = ( ( ma + i ) << 1 ) & 0x3fff;
		uint8_t chr = m_displayram[ offset ];
		uint8_t attr = m_displayram[ offset +1 ];
		uint16_t fg = m_palette_base + ( attr & 0x07 );
		uint16_t bg = m_palette_base + ( ( attr >> 4 ) & 0x07 );
		uint16_t code = chr & 0x1f;
		if((attr & 0x88) == 0x88)
		{
			code = m_displayram[ offset - 2 ] & 0x1f;
			code = (code << 8) + chr;
		}
		else if(attr & 0x80)
			code = (code << 8) + m_displayram[ offset + 2 ];
		else
			code = chr;

		uint8_t data;
		if(ra < 16)
			data = m_jxkanji[code * 16 * 2 + (ra * 2) + ((attr & 8)?1:0)];
		else
			data = ((i == cursor_x) && (m_pc_framecnt & 8)) ? 0xff: 0;

		*p = palette[( data & 0x80 ) ? fg : bg]; p++;
		*p = palette[( data & 0x40 ) ? fg : bg]; p++;
		*p = palette[( data & 0x20 ) ? fg : bg]; p++;
		*p = palette[( data & 0x10 ) ? fg : bg]; p++;
		*p = palette[( data & 0x08 ) ? fg : bg]; p++;
		*p = palette[( data & 0x04 ) ? fg : bg]; p++;
		*p = palette[( data & 0x02 ) ? fg : bg]; p++;
		*p = palette[( data & 0x01 ) ? fg : bg]; p++;
	}
}

MC6845_UPDATE_ROW( pc_t1t_device::t1000_gfx_4bpp_update_row )
{
	const rgb_t *palette = m_palette->palette()->entry_list_raw();
	uint32_t  *p = &bitmap.pix32(y);
	uint8_t   *vid = m_displayram + ( ra << 13 );
	int i;

	for ( i = 0; i < x_count; i++ )
	{
		uint16_t offset = ( ( ma + i ) << 1 ) & 0x1fff;
		uint8_t data = vid[ offset ];

		*p = palette[m_palette_base + m_reg.data[0x10 + ( data >> 4 )]]; p++;
		*p = palette[m_palette_base + m_reg.data[0x10 + ( data >> 4 )]]; p++;
		*p = palette[m_palette_base + m_reg.data[0x10 + ( data & 0x0F )]]; p++;
		*p = palette[m_palette_base + m_reg.data[0x10 + ( data & 0x0F )]]; p++;

		data = vid[ offset + 1 ];

		*p = palette[m_palette_base + m_reg.data[0x10 + ( data >> 4 )]]; p++;
		*p = palette[m_palette_base + m_reg.data[0x10 + ( data >> 4 )]]; p++;
		*p = palette[m_palette_base + m_reg.data[0x10 + ( data & 0x0F )]]; p++;
		*p = palette[m_palette_base + m_reg.data[0x10 + ( data & 0x0F )]]; p++;
	}
}


MC6845_UPDATE_ROW( pc_t1t_device::t1000_gfx_2bpp_update_row )
{
	const rgb_t *palette = m_palette->palette()->entry_list_raw();
	uint32_t  *p = &bitmap.pix32(y);
	uint8_t   *vid = m_displayram + ( ra << 13 );
	int i;

	for ( i = 0; i < x_count; i++ )
	{
		uint16_t offset = ( ( ma + i ) << 1 ) & 0x1fff;
		uint8_t data = vid[ offset ];

		*p = palette[m_palette_base + m_reg.data[ 0x10 + ( ( data >> 6 ) & 0x03 ) ]]; p++;
		*p = palette[m_palette_base + m_reg.data[ 0x10 + ( ( data >> 4 ) & 0x03 ) ]]; p++;
		*p = palette[m_palette_base + m_reg.data[ 0x10 + ( ( data >> 2 ) & 0x03 ) ]]; p++;
		*p = palette[m_palette_base + m_reg.data[ 0x10 + (   data        & 0x03 ) ]]; p++;

		data = vid[ offset+1 ];

		*p = palette[m_palette_base + m_reg.data[ 0x10 + ( ( data >> 6 ) & 0x03 ) ]]; p++;
		*p = palette[m_palette_base + m_reg.data[ 0x10 + ( ( data >> 4 ) & 0x03 ) ]]; p++;
		*p = palette[m_palette_base + m_reg.data[ 0x10 + ( ( data >> 2 ) & 0x03 ) ]]; p++;
		*p = palette[m_palette_base + m_reg.data[ 0x10 + (   data        & 0x03 ) ]]; p++;
	}
}


MC6845_UPDATE_ROW( pcvideo_pcjr_device::pcjr_gfx_2bpp_high_update_row )
{
	const rgb_t *palette = m_palette->palette()->entry_list_raw();
	uint32_t  *p = &bitmap.pix32(y);
	uint8_t   *vid = m_displayram + ( ra << 13 );
	int i;

	for ( i = 0; i < x_count; i++ )
	{
		uint16_t offset = ( ( ma + i ) << 1 ) & 0x1fff;
		uint8_t data0 = vid[ offset ];
		uint8_t data1 = vid[ offset + 1 ];

		*p = palette[m_palette_base + m_reg.data[ 0x10 + ( ( ( data0 & 0x80 ) >> 7 ) | ( ( data1 & 0x80 ) >> 6 ) ) ]]; p++;
		*p = palette[m_palette_base + m_reg.data[ 0x10 + ( ( ( data0 & 0x40 ) >> 6 ) | ( ( data1 & 0x40 ) >> 5 ) ) ]]; p++;
		*p = palette[m_palette_base + m_reg.data[ 0x10 + ( ( ( data0 & 0x20 ) >> 5 ) | ( ( data1 & 0x20 ) >> 4 ) ) ]]; p++;
		*p = palette[m_palette_base + m_reg.data[ 0x10 + ( ( ( data0 & 0x10 ) >> 4 ) | ( ( data1 & 0x10 ) >> 3 ) ) ]]; p++;
		*p = palette[m_palette_base + m_reg.data[ 0x10 + ( ( ( data0 & 0x08 ) >> 3 ) | ( ( data1 & 0x08 ) >> 2 ) ) ]]; p++;
		*p = palette[m_palette_base + m_reg.data[ 0x10 + ( ( ( data0 & 0x04 ) >> 2 ) | ( ( data1 & 0x04 ) >> 1 ) ) ]]; p++;
		*p = palette[m_palette_base + m_reg.data[ 0x10 + ( ( ( data0 & 0x02 ) >> 1 ) | ( ( data1 & 0x02 )      ) ) ]]; p++;
		*p = palette[m_palette_base + m_reg.data[ 0x10 + ( ( ( data0 & 0x01 )      ) | ( ( data1 & 0x01 ) << 1 ) ) ]]; p++;
	}
}


MC6845_UPDATE_ROW( pc_t1t_device::t1000_gfx_2bpp_tga_update_row )
{
	const rgb_t *palette = m_palette->palette()->entry_list_raw();
	uint32_t  *p = &bitmap.pix32(y);
	uint8_t   *vid = m_displayram + ( ra << 13 );
	int i;

	if ( y == 0 ) logerror("t1000_gfx_2bpp_tga_update_row\n");
	for ( i = 0; i < x_count; i++ )
	{
		uint16_t offset = ( ( ma + i ) << 1 ) & 0x1fff;
		uint8_t data = vid[ offset ];
		uint16_t data2 = ( vid[ offset + 1 ] ) << 1;

		*p = palette[m_reg.data[ 0x10 + ( ( ( data2 & 0x100 ) | ( data & 0x80 ) ) >> 7 ) ]]; p++;
		*p = palette[m_reg.data[ 0x10 + ( ( ( data2 & 0x80 ) | ( data & 0x40 ) ) >> 6 ) ]]; p++;
		*p = palette[m_reg.data[ 0x10 + ( ( ( data2 & 0x40 ) | ( data & 0x20 ) ) >> 5 ) ]]; p++;
		*p = palette[m_reg.data[ 0x10 + ( ( ( data2 & 0x20 ) | ( data & 0x10 ) ) >> 4 ) ]]; p++;

		*p = palette[m_reg.data[ 0x10 + ( ( ( data2 & 0x10 ) | ( data & 0x08 ) ) >> 3 ) ]]; p++;
		*p = palette[m_reg.data[ 0x10 + ( ( ( data2 & 0x08 ) | ( data & 0x04 ) ) >> 2 ) ]]; p++;
		*p = palette[m_reg.data[ 0x10 + ( ( ( data2 & 0x04 ) | ( data & 0x02 ) ) >> 1 ) ]]; p++;
		*p = palette[m_reg.data[ 0x10 + (   ( data2 & 0x02 ) | ( data & 0x01 )        ) ]]; p++;
	}
}


MC6845_UPDATE_ROW( pc_t1t_device::t1000_gfx_1bpp_update_row )
{
	const rgb_t *palette = m_palette->palette()->entry_list_raw();
	uint32_t  *p = &bitmap.pix32(y);
	uint8_t   *vid = m_displayram + ( ra << 13 );
	uint8_t   fg = m_palette_base + m_reg.data[0x11];
	uint8_t   bg = m_palette_base + m_reg.data[0x10];
	int i;

	if ( y == 0 ) logerror("t1000_gfx_1bpp_update_row\n");
	for ( i = 0; i < x_count; i++ )
	{
		uint16_t offset = ( ( ma + i ) << 1 ) & 0x1fff;
		uint8_t data = vid[ offset ];

		*p = palette[( data & 0x80 ) ? fg : bg]; p++;
		*p = palette[( data & 0x40 ) ? fg : bg]; p++;
		*p = palette[( data & 0x20 ) ? fg : bg]; p++;
		*p = palette[( data & 0x10 ) ? fg : bg]; p++;
		*p = palette[( data & 0x08 ) ? fg : bg]; p++;
		*p = palette[( data & 0x04 ) ? fg : bg]; p++;
		*p = palette[( data & 0x02 ) ? fg : bg]; p++;
		*p = palette[( data & 0x01 ) ? fg : bg]; p++;

		data = vid[ offset + 1 ];

		*p = palette[( data & 0x80 ) ? fg : bg]; p++;
		*p = palette[( data & 0x40 ) ? fg : bg]; p++;
		*p = palette[( data & 0x20 ) ? fg : bg]; p++;
		*p = palette[( data & 0x10 ) ? fg : bg]; p++;
		*p = palette[( data & 0x08 ) ? fg : bg]; p++;
		*p = palette[( data & 0x04 ) ? fg : bg]; p++;
		*p = palette[( data & 0x02 ) ? fg : bg]; p++;
		*p = palette[( data & 0x01 ) ? fg : bg]; p++;
	}
}

MC6845_UPDATE_ROW( pc_t1t_device::crtc_update_row )
{
	if (m_update_row_type == -1)
		return;

	switch (m_update_row_type)
	{
		case T1000_TEXT_INTEN:
			t1000_text_inten_update_row(bitmap, cliprect, ma, ra, y, x_count, cursor_x, de, hbp, vbp);
			break;
		case T1000_TEXT_BLINK:
			t1000_text_blink_update_row(bitmap, cliprect, ma, ra, y, x_count, cursor_x, de, hbp, vbp);
			break;
		case T1000_GFX_1BPP:
			t1000_gfx_1bpp_update_row(bitmap, cliprect, ma, ra, y, x_count, cursor_x, de, hbp, vbp);
			break;
		case T1000_GFX_2BPP:
			t1000_gfx_2bpp_update_row(bitmap, cliprect, ma, ra, y, x_count, cursor_x, de, hbp, vbp);
			break;
		case T1000_GFX_2BPP_TGA:
			t1000_gfx_2bpp_tga_update_row(bitmap, cliprect, ma, ra, y, x_count, cursor_x, de, hbp, vbp);
			break;
		case T1000_GFX_4BPP:
			t1000_gfx_4bpp_update_row(bitmap, cliprect, ma, ra, y, x_count, cursor_x, de, hbp, vbp);
			break;
	}
}


MC6845_UPDATE_ROW( pcvideo_pcjr_device::crtc_update_row )
{
	if (m_update_row_type == -1)
		return;

	switch (m_update_row_type)
	{
		case PCJX_TEXT:
			pcjx_text_update_row(bitmap, cliprect, ma, ra, y, x_count, cursor_x, de, hbp, vbp);
			break;
		case PCJR_GFX_2BPP_HIGH:
			pcjr_gfx_2bpp_high_update_row(bitmap, cliprect, ma, ra, y, x_count, cursor_x, de, hbp, vbp);
			break;
		default:
			pc_t1t_device::crtc_update_row(bitmap, cliprect, ma, ra, y, x_count, cursor_x, de, hbp, vbp);
			break;
	}
}


void pcvideo_t1000_device::mode_switch( void )
{
	switch( m_mode_control & 0x3B )
	{
	case 0x08: case 0x09:
		m_update_row_type = T1000_TEXT_INTEN;
		break;
	case 0x28: case 0x29:
		m_update_row_type = T1000_TEXT_BLINK;
		break;
	case 0x0A: case 0x0B: case 0x2A: case 0x2B:
		switch( m_bank & 0xc0 )
		{
		case 0x00:
		case 0x40:
			//logerror("t1t_gfx_2bpp - 1\n");
			m_update_row_type = T1000_GFX_2BPP;
			if ( m_color_select )
			{
				m_reg.data[0x10] = 0x00;
				m_reg.data[0x11] = 0x0B;
				m_reg.data[0x12] = 0x0D;
				m_reg.data[0x13] = 0x0F;
			}
			else
			{
				m_reg.data[0x10] = 0x00;
				m_reg.data[0x11] = 0x0A;
				m_reg.data[0x12] = 0x0C;
				m_reg.data[0x13] = 0x0E;
			}
			break;
		case 0x80:
		case 0xc0:
			//logerror("t1t_gfx_4bpp\n");
			m_update_row_type = T1000_GFX_4BPP;
			break;
		}
		break;
	case 0x18: case 0x19: case 0x1A: case 0x1B:
	case 0x38: case 0x39: case 0x3A: case 0x3B:
		switch( m_bank & 0xc0 )
		{
		case 0x00:
		case 0x40:
			//logerror("t1t_gfx_1bpp\n");
			m_update_row_type = T1000_GFX_1BPP;
			break;
		case 0x80:
		case 0xc0:
			//logerror("t1t_gfx_2bpp - 2\n");
			m_update_row_type = T1000_GFX_2BPP_TGA;
			break;
		}
		break;
	default:
		m_update_row_type = -1;
		break;
	}
}


/* mode control 1 ( m_reg.data[0] ) */
/* bit0 - 0 = low bandwidth, 1 = high bandwidth */
/* bit1 - 0 = alpha, 1 = graphics */
/* bit2 - 0 = color, 1 = b/w */
/* bit3 - 0 = video disable, 1 = video enable */
/* bit4 - 1 = 16 color graphics */
/* mode control 2 ( m_reg.data[3] ) */
/* bit1 - 1 = enable blink */
/* bit3 - 1 = 2 color graphics */

void pcvideo_pcjr_device::pc_pcjr_mode_switch()
{
	switch( m_reg.data[0] & 0x1A )
	{
	case 0x08:      /* 01x0x */
		if(m_jxkanji)
		{
			m_update_row_type = PCJX_TEXT;
			break;
		}
		if ( m_reg.data[3] & 0x02 )
		{
			m_update_row_type = T1000_TEXT_BLINK;
		}
		else
		{
			m_update_row_type = T1000_TEXT_INTEN;
		}
		break;
	case 0x0A:      /* 01x1x */
		/* By default use medium resolution mode */
		m_update_row_type = T1000_GFX_2BPP;

		/* Check for high resolution mode */
		if ( ( m_bank & 0xc0 ) == 0xc0 )
			m_update_row_type = PCJR_GFX_2BPP_HIGH;

		/* Check for 640x200 b/w 2 shades mode */
		if ( ( m_reg.data[0] & 0x04 ) && ( m_reg.data[3] & 0x08 ) )
		{
			m_update_row_type = T1000_GFX_1BPP;
		}
		break;
	case 0x18:      /* 11x0x - invalid?? */
		m_update_row_type = -1;
		break;
	case 0x1A:      /* 11x1x */
		m_update_row_type = T1000_GFX_4BPP;
		break;
	default:
		m_update_row_type = -1;
		break;
	}

	/* Determine mc6845 input clock */
	if ( m_reg.data[0] & 0x01 )
	{
		m_mc6845->set_clock( XTAL_14_31818MHz/8 );
	}
	else
	{
		m_mc6845->set_clock( XTAL_14_31818MHz/16 );
	}

	/* color or b/w? */
	m_palette_base = ( m_reg.data[0] & 0x04 ) ? 16 : 0;
}


/*
 * 3d8 rW   T1T mode control register (see #P138)
 */
void pcvideo_t1000_device::mode_control_w(int data)
{
	m_mode_control = data;

	mode_switch();
}

int pc_t1t_device::mode_control_r(void)
{
	int data = m_mode_control;
	return data;
}

/*
 * 3d9 ?W   color select register on color adapter
 */
void pc_t1t_device::color_select_w(int data)
{
	if (m_color_select == data)
		return;
	m_color_select = data;
}

int pc_t1t_device::color_select_r(void)
{
	int data = m_color_select;
	return data;
}

/*  Bitfields for T1T status register:
 *  Bit(s)  Description (Table P179)
 *  7-6 not used
 *  5-4 color EGA, color ET4000: diagnose video display feedback, select
 *      from color plane enable
 *  4   holds current dot being displayed
 *  3   in vertical retrace
 *  2   (CGA,color EGA) light pen switch is off
 *      (MCGA,color ET4000) reserved (0)
 *  1   (CGA,color EGA) positive edge from light pen has set trigger
 *      (MCGA,color ET4000) reserved (0)
 *  0   display enabled
 *      =0  do not use memory
 *      =1  memory access without interfering with display
 *      (Genoa SuperEGA) horizontal or vertical retrace
 */
int pc_t1t_device::status_r(void)
{
	int data = m_vsync | m_status | m_display_enable;
	/* HACK HACK HACK */
	data |= ( m_display_enable ? 0x10 : 0x00 );
	/* end HACK */
	return data;
}

/*
 * 3db -W   light pen strobe reset (on any value)
 */
void pc_t1t_device::lightpen_strobe_w(int data)
{
//  pc_port[0x3db] = data;
}


/*
 * 3da -W   (mono EGA/mono VGA) feature control register
 *          (see PORT 03DAh-W for details; VGA, see PORT 03CAh-R)
 */
void pc_t1t_device::vga_index_w(int data)
{
	m_reg.index = data;
}

void pcvideo_t1000_device::vga_data_w(int data)
{
	m_reg.data[m_reg.index] = data;

	switch (m_reg.index)
	{
		case 0x00: /* mode control 1 */
			break;
		case 0x01: /* palette mask (bits 3-0) */
			break;
		case 0x02: /* border color (bits 3-0) */
			break;
		case 0x03: /* mode control 2 */
			break;
		case 0x04: /* reset register */
			break;
		/* palette array */
		case 0x10: case 0x11: case 0x12: case 0x13:
		case 0x14: case 0x15: case 0x16: case 0x17:
		case 0x18: case 0x19: case 0x1a: case 0x1b:
		case 0x1c: case 0x1d: case 0x1e: case 0x1f:
			m_reg.data[m_reg.index] = data & 0x0F;
			break;
	}
}


void pcvideo_pcjr_device::pc_pcjr_vga_data_w(int data)
{
	m_reg.data[m_reg.index] = data;

	switch (m_reg.index)
	{
		case 0x00:  /* mode control 1 */
					/* bit0 - 0 = low bandwidth, 1 = high bandwidth */
					/* bit1 - 0 = alpha, 1 = graphics */
					/* bit2 - 0 = color, 1 = b/w */
					/* bit3 - 0 = video disable, 1 = video enable */
					/* bit4 - 1 = 16 color graphics */
			pc_pcjr_mode_switch();
			break;
		case 0x01:  /* palette mask (bits 3-0) */
			break;
		case 0x02:  /* border color (bits 3-0) */
			break;
		case 0x03:  /* mode control 2 */
					/* bit1 - 1 = enable blink */
					/* bit3 - 1 = 2 color graphics */
			pc_pcjr_mode_switch();
			break;
		case 0x04:  /* reset register */
			break;
					/* palette array */
		case 0x10: case 0x11: case 0x12: case 0x13:
		case 0x14: case 0x15: case 0x16: case 0x17:
		case 0x18: case 0x19: case 0x1a: case 0x1b:
		case 0x1c: case 0x1d: case 0x1e: case 0x1f:
			m_reg.data[m_reg.index] = data & 0x0F;
			break;
	}
}


int pc_t1t_device::vga_data_r(void)
{
	int data = m_reg.data[m_reg.index];

	switch (m_reg.index)
	{
		case 0x00: /* mode control 1 */
			break;
		case 0x01: /* palette mask (bits 3-0) */
			break;
		case 0x02: /* border color (bits 3-0) */
			break;
		case 0x03: /* mode control 2 */
			break;
		case 0x04: /* reset register */
			break;
		/* palette array */
		case 0x10: case 0x11: case 0x12: case 0x13:
		case 0x14: case 0x15: case 0x16: case 0x17:
		case 0x18: case 0x19: case 0x1a: case 0x1b:
		case 0x1c: case 0x1d: case 0x1e: case 0x1f:
			break;
	}
	return data;
}

/*
 * 3df RW   display bank, access bank, mode
 * bit 0-2  Identifies the page of main memory being displayed in units of 16K.
 *          0: 0K, 1: 16K...7: 112K. In 32K modes (bits 6-7 = 2) only 0,2,4 and
 *          6 are valid, as the next page will also be used.
 *     3-5  Identifies the page of main memory that can be read/written at B8000h
 *          in units of 16K. 0: 0K, 1: 16K...7: 112K. In 32K modes (bits 6-7 = 2)
 *          only 0,2,4 and 6 are valid, as the next page will also be used.
 *     6-7  Display mode. 0: Text, 1: 16K graphics mode (4,5,6,8)
 *          2: 32K graphics mode (9,Ah)
 */
void pcvideo_t1000_device::bank_w(int data)
{
	int dram, vram;
	if ((data&0xc0)==0xc0) /* needed for lemmings */
	{
		dram = (m_bank & 0x06);// | ((m_bank & 0x1800) >> 8);
		vram = ((m_bank & 0x30) >> 3);// | ((m_bank & 0x6000) >> 10);
	}
	else
	{
		dram = (m_bank & 0x07);// | ((m_bank & 0x1800) >> 8);
		vram = ((m_bank & 0x38) >> 3);// | ((m_bank & 0x6000) >> 10);
	}
	m_displayram = m_ram->pointer() + (dram << 14);
	if(m_disable)
		return;
	m_vram->set_bank(vram);
	if((m_bank & 0xc0) != (data & 0xc0))
		mode_switch();
}


void pcvideo_pcjr_device::pc_pcjr_bank_w(int data)
{
	int dram, vram;
	m_bank = data;
	/* it seems the video ram is mapped to the last 128K of main memory */
	if ((data&0xc0)==0xc0) /* needed for lemmings */
	{
		dram = (data & 0x06);
		vram = (data & 0x30) >> 3;
	}
	else
	{
		dram = (data & 0x07);
		vram = (data & 0x38) >> 3;
	}
	m_vram->set_bank(vram);
	m_displayram = m_ram->pointer() + (dram << 14);
	if((m_bank & 0xc0) != (data & 0xc0))
		pc_pcjr_mode_switch();
}

void pcvideo_pcjr_device::pc_pcjx_bank_w(int data)
{
	int dram, vram;
	m_bank = data;
	if ((data&0xc0)==0xc0) /* needed for lemmings */
	{
		dram = (data & 0x06);
		vram = (data & 0x30) >> 3;
	}
	else
	{
		dram = (data & 0x07);
		vram = (data & 0x38) >> 3;
	}
	m_vram->set_bank(vram);
	/* this certainly isn't correct but otherwise the memory test stomps on the vram */
	m_displayram = (uint8_t *)memshare(":vram")->ptr() + (dram << 14);
	if((m_bank & 0xc0) != (data & 0xc0))
		pc_pcjr_mode_switch();
}

int pc_t1t_device::bank_r(void)
{
	return m_bank;
}

/*************************************************************************
 *
 *      T1T
 *      Tandy 1000 / PCjr
 *
 *************************************************************************/

WRITE8_MEMBER( pcvideo_t1000_device::write )
{
	switch( offset )
	{
		case 0: case 2: case 4: case 6:
			m_mc6845->address_w( space, offset, data );
			break;
		case 1: case 3: case 5: case 7:
			m_mc6845->register_w( space, offset, data );
			break;
		case 8:
			mode_control_w(data);
			break;
		case 9:
			color_select_w(data);
			break;
		case 10:
			vga_index_w(data);
			break;
		case 11:
			lightpen_strobe_w(data);
			break;
		case 12:
			break;
		case 13:
			m_bank = (data << 8) | (m_bank & 0xff);
			bank_w(m_bank);
			break;
		case 14:
			vga_data_w(data);
			break;
		case 15:
			m_bank = (m_bank & 0xff00) | data;
			bank_w(data);
			break;
	}
}

WRITE8_MEMBER( pcvideo_pcjr_device::write )
{
	switch( offset )
	{
		case 0: case 4:
			m_mc6845->address_w( space, offset, data );
			break;
		case 1: case 5:
			m_mc6845->register_w( space, offset, data );
			break;
		case 10:
			if ( m_address_data_ff & 0x01 )
			{
				pc_pcjr_vga_data_w( data );
			}
			else
			{
				vga_index_w( data );
			}
			m_address_data_ff ^= 0x01;
			break;
		case 11:
			lightpen_strobe_w(data);
			break;
		case 12:
			break;
		case 15:
			if(m_jxkanji)
				pc_pcjx_bank_w(data);
			else
				pc_pcjr_bank_w(data);
			break;

		default:
			break;
	}
}


READ8_MEMBER( pc_t1t_device::read )
{
	int             data = 0xff;

	switch( offset )
	{
		case 0: case 2: case 4: case 6:
			/* return last written mc6845 address value here? */
			break;

		case 1: case 3: case 5: case 7:
			data = m_mc6845->register_r( space, offset );
			break;

		case 8:
			data = mode_control_r();
			break;

		case 9:
			data = color_select_r();
			break;

		case 10:
			m_address_data_ff = 0;
			data = status_r();
			break;

		case 11:
			/* -W lightpen strobe reset */
			break;

		case 12:
		case 13:
			break;

		case 14:
			data = vga_data_r();
			break;

		case 15:
			data = bank_r();
			break;
	}
	return data;
}

WRITE_LINE_MEMBER( pc_t1t_device::t1000_de_changed )
{
	m_display_enable = state ? 1 : 0;
}


WRITE_LINE_MEMBER( pcvideo_t1000_device::t1000_vsync_changed )
{
	m_vsync = state ? 8 : 0;
	if ( state )
	{
		m_pc_framecnt++;
	}
}

WRITE_LINE_MEMBER( pcvideo_t1000_device::disable_w )
{
	if(state)
		m_vram->set_bank(8);
	else
		bank_w(m_bank);
	m_disable = state ? true : false;
}

WRITE_LINE_MEMBER( pcvideo_pcjr_device::pcjr_vsync_changed )
{
	m_vsync = state ? 8 : 0;
	if ( state )
	{
		m_pc_framecnt++;
	}
	machine().device<pic8259_device>("pic8259")->ir5_w(state);
}
