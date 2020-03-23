// license:BSD-3-Clause
// copyright-holders:David Graves, Angelo Salese, David Haywood, Tomasz Slanina
/***************************************************************************

    Legionnaire / Heated Barrel video hardware (derived from D-Con)

    priority test (used by Legionnaire, front to bottom):
    - OBJ 0
    - TXT
    - OBJ 1
    - BK3
    - OBJ 2
    - MBK
    - OBJ 3
    - LBK
    TODO: Anything else doesn't match this scheme (most notably Denjin Makai),
          guess it's selectable by PROM, CRTC or COP ...

***************************************************************************/

#include "emu.h"
#include "includes/legionna.h"
#include "screen.h"


/******************************************************************************/


WRITE16_MEMBER(legionna_state::tilemap_enable_w)
{
	COMBINE_DATA(&m_layer_disable);
}

WRITE16_MEMBER(legionna_state::tile_scroll_w)
{
	COMBINE_DATA(scrollvals + offset);
	data = scrollvals[offset];

	tilemap_t *tm = nullptr;
	switch(offset/2) {
	case 0: tm = m_background_layer; break;
	case 1: tm = m_midground_layer; break;
	case 2: tm = m_foreground_layer; break;
	}
	if(offset & 1)
		tm->set_scrolly(0, data);
	else
		tm->set_scrollx(0, data);
}

WRITE16_MEMBER(legionna_state::tile_vreg_1a_w)
{
	flip_screen_set(data & 1);
	// TODO: other bits ...
}

WRITE16_MEMBER(legionna_state::tile_scroll_base_w)
{
	// TODO: specific for Godzilla, needs visible area changes.
	if(offset == 7)
		m_text_layer->set_scrolldy(0x1ef - data,0x1ef - data);

	printf("%02x %04x\n",offset,data);
}

WRITE16_MEMBER(legionna_state::heatbrl_setgfxbank)
{
	m_back_gfx_bank = (data &0x4000) >> 2;
}

/*xxx- --- ---- ---- banking*/
WRITE16_MEMBER(legionna_state::denjinmk_setgfxbank)
{
	m_fore_gfx_bank = (data &0x2000) >> 1;//???
	m_back_gfx_bank = (data &0x4000) >> 2;
	m_mid_gfx_bank  = (data &0x8000) >> 3;//???

	m_background_layer->mark_all_dirty();
	m_foreground_layer->mark_all_dirty();
	m_midground_layer->mark_all_dirty();
	m_text_layer->mark_all_dirty();
}

WRITE16_MEMBER(legionna_state::videowrite_cb_w)
{
	//  AM_RANGE(0x101000, 0x1017ff) AM_RAM // _WRITE(legionna_background_w) AM_SHARE("back_data")
	//  AM_RANGE(0x101800, 0x101fff) AM_RAM // _WRITE(legionna_foreground_w) AM_SHARE("fore_data")
	//  AM_RANGE(0x102000, 0x1027ff) AM_RAM // _WRITE(legionna_midground_w) AM_SHARE("mid_data")
	//  AM_RANGE(0x102800, 0x1037ff) AM_RAM // _WRITE(legionna_text_w) AM_SHARE("textram")

	if (offset < 0x800 / 2)
	{
		legionna_background_w(space, offset, data, 0xffff);
	}
	else if (offset < 0x1000 /2)
	{
		offset -= 0x800 / 2;
		legionna_foreground_w(space, offset, data, 0xffff);
	}
	else if (offset < 0x1800/2)
	{
		offset -= 0x1000 / 2;
		legionna_midground_w(space, offset, data, 0xffff);
	}
	else if (offset < 0x2800/2)
	{
		offset -= 0x1800 / 2;
		legionna_text_w(space, offset, data, 0xffff);
	}
}

// TODO: move to COP device
WRITE16_MEMBER(legionna_state::grainbow_layer_config_w)
{
	// (0x8000|0x1ff), 0x200, 0x1ff, 0x200 written in sequence at startup
	COMBINE_DATA(&m_layer_config[offset]);
}

WRITE16_MEMBER(legionna_state::legionna_background_w)
{
	COMBINE_DATA(&m_back_data[offset]);
	m_background_layer->mark_tile_dirty(offset);
}

WRITE16_MEMBER(legionna_state::legionna_midground_w)
{
	COMBINE_DATA(&m_mid_data[offset]);
	m_midground_layer->mark_tile_dirty(offset);
}

WRITE16_MEMBER(legionna_state::legionna_foreground_w)
{
	COMBINE_DATA(&m_fore_data[offset]);
	m_foreground_layer->mark_tile_dirty(offset);
}

WRITE16_MEMBER(legionna_state::legionna_text_w)
{
	COMBINE_DATA(&m_textram[offset]);
	m_text_layer->mark_tile_dirty(offset);
}

TILE_GET_INFO_MEMBER(legionna_state::get_back_tile_info)
{
	int tile=m_back_data[tile_index];
	int color=(tile>>12)&0xf;

	tile &= 0xfff;
	tile |= m_back_gfx_bank;        /* Heatbrl uses banking */

	SET_TILE_INFO_MEMBER(1,tile,color,0);
}

TILE_GET_INFO_MEMBER(legionna_state::get_mid_tile_info)
{
	int tile=m_mid_data[tile_index];
	int color=(tile>>12)&0xf;

	tile &= 0xfff;

	SET_TILE_INFO_MEMBER(5,tile,color,0);
}

TILE_GET_INFO_MEMBER(legionna_state::get_mid_tile_info_denji)
{
	int tile=m_mid_data[tile_index];
	int color=(tile>>12)&0xf;

	tile &= 0xfff;
	tile |= m_mid_gfx_bank;

	SET_TILE_INFO_MEMBER(5,tile,color,0);
}

TILE_GET_INFO_MEMBER(legionna_state::get_mid_tile_info_cupsoc)
{
	int tile=m_mid_data[tile_index];
	int color=(tile>>12)&0xf;

	tile &= 0xfff;

	tile |= 0x1000;
	color += 0x10;

	SET_TILE_INFO_MEMBER(1,tile,color,0);
}

TILE_GET_INFO_MEMBER(legionna_state::get_fore_tile_info)
{
	int tile=m_fore_data[tile_index];
	int color=(tile>>12)&0xf;

	tile &= 0xfff;

	SET_TILE_INFO_MEMBER(4,tile,color,0);
}

TILE_GET_INFO_MEMBER(legionna_state::get_fore_tile_info_denji)
{
	int tile=m_fore_data[tile_index];
	int color=(tile>>12)&0xf;

	tile &= 0xfff;
	tile |= m_fore_gfx_bank;

	SET_TILE_INFO_MEMBER(4,tile,color,0);
}

TILE_GET_INFO_MEMBER(legionna_state::get_text_tile_info)
{
	int tile = m_textram[tile_index];
	int color=(tile>>12)&0xf;

	tile &= 0xfff;

	SET_TILE_INFO_MEMBER(0,tile,color,0);
}

void legionna_state::common_video_allocate_ptr()
{
	m_back_data = make_unique_clear<uint16_t[]>(0x800/2);
	m_fore_data =  make_unique_clear<uint16_t[]>(0x800/2);
	m_mid_data =  make_unique_clear<uint16_t[]>(0x800/2);
	m_textram =  make_unique_clear<uint16_t[]>(0x1000/2);
	m_scrollram16 = std::make_unique<uint16_t[]>(0x60/2);
	m_sprite_xoffs = 0;
	m_sprite_yoffs = 0;

	save_pointer(NAME(m_back_data.get()), 0x800/2);
	save_pointer(NAME(m_fore_data.get()), 0x800/2);
	save_pointer(NAME(m_mid_data.get()), 0x800/2);
	save_pointer(NAME(m_textram.get()), 0x1000/2);
	save_pointer(NAME(m_scrollram16.get()), 0x60/2);

	save_item(NAME(m_back_gfx_bank));
	save_item(NAME(m_mid_gfx_bank));
	save_item(NAME(m_fore_gfx_bank));
	save_item(NAME(m_layer_disable));
}

void legionna_state::common_video_start()
{
	common_video_allocate_ptr();

	m_background_layer = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(legionna_state::get_back_tile_info),this),TILEMAP_SCAN_ROWS,16,16,32,32);
	m_midground_layer =  &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(legionna_state::get_mid_tile_info),this), TILEMAP_SCAN_ROWS,16,16,32,32);
	m_foreground_layer = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(legionna_state::get_fore_tile_info),this),TILEMAP_SCAN_ROWS,16,16,32,32);
	m_text_layer =       &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(legionna_state::get_text_tile_info),this),TILEMAP_SCAN_ROWS,  8,8,64,32);

	m_has_extended_banking = 0;
	m_has_extended_priority = 0;

	m_background_layer->set_transparent_pen(15);
	m_midground_layer->set_transparent_pen(15);
	m_foreground_layer->set_transparent_pen(15);
	m_text_layer->set_transparent_pen(15);
}

VIDEO_START_MEMBER(legionna_state,legionna)
{
	common_video_start();

	m_sprite_pri_mask[0] = 0x0000;
	m_sprite_pri_mask[1] = 0xfff0;
	m_sprite_pri_mask[2] = 0xfffc;
	m_sprite_pri_mask[3] = 0xfffe;
}

VIDEO_START_MEMBER(legionna_state,heatbrl)
{
	common_video_start();

	m_sprite_pri_mask[0] = 0xfff0;
	m_sprite_pri_mask[1] = 0xfffc;
	m_sprite_pri_mask[2] = 0xfffe;
	// TODO: not shown?
	m_sprite_pri_mask[3] = 0xffff;
}

VIDEO_START_MEMBER(legionna_state,godzilla)
{
	VIDEO_START_CALL_MEMBER(legionna);

	m_has_extended_banking = 1;
	m_has_extended_priority = 0;

	m_sprite_pri_mask[0] = 0xfff0;
	m_sprite_pri_mask[1] = 0xfffc;
	m_sprite_pri_mask[2] = 0xfffe;
	// TODO: not shown?
	m_sprite_pri_mask[3] = 0xffff;
}

VIDEO_START_MEMBER(legionna_state,denjinmk)
{
	common_video_allocate_ptr();

	m_background_layer = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(legionna_state::get_back_tile_info),this),TILEMAP_SCAN_ROWS,16,16,32,32);
	m_midground_layer =  &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(legionna_state::get_mid_tile_info_denji),this), TILEMAP_SCAN_ROWS,16,16,32,32);
	m_foreground_layer = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(legionna_state::get_fore_tile_info_denji),this),TILEMAP_SCAN_ROWS,16,16,32,32);
	m_text_layer =       &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(legionna_state::get_text_tile_info),this),TILEMAP_SCAN_ROWS,  8,8,64,32);

	m_has_extended_banking = 1;
	m_has_extended_priority = 0;

	m_sprite_pri_mask[0] = 0xfff0; // normal sprites
	m_sprite_pri_mask[1] = 0xfffc; // luna park horse rides
	m_sprite_pri_mask[2] = 0xfffe; // door at the end of sewers part in level 1
	m_sprite_pri_mask[3] = 0x0000; // briefing guy in pre-stage and portraits before a boss fight

//  m_background_layer->set_transparent_pen(15);
	m_midground_layer->set_transparent_pen(15);
	m_foreground_layer->set_transparent_pen(15);
	m_text_layer->set_transparent_pen(7);//?
}

VIDEO_START_MEMBER(legionna_state,cupsoc)
{
	common_video_allocate_ptr();

	m_background_layer = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(legionna_state::get_back_tile_info),this),TILEMAP_SCAN_ROWS,16,16,32,32);
	m_midground_layer =  &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(legionna_state::get_mid_tile_info_cupsoc),this), TILEMAP_SCAN_ROWS,16,16,32,32);
	m_foreground_layer = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(legionna_state::get_fore_tile_info),this),TILEMAP_SCAN_ROWS,16,16,32,32);
	m_text_layer =       &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(legionna_state::get_text_tile_info),this),TILEMAP_SCAN_ROWS,  8,8,64,32);

	m_has_extended_banking = 0;
	m_has_extended_priority = 1;

	m_background_layer->set_transparent_pen(15);
	m_midground_layer->set_transparent_pen(15);
	m_foreground_layer->set_transparent_pen(15);
	m_text_layer->set_transparent_pen(15);
}

VIDEO_START_MEMBER(legionna_state,grainbow)
{
	VIDEO_START_CALL_MEMBER(legionna);
	m_sprite_xoffs = m_sprite_yoffs = 16;

	m_has_extended_banking = 0;
	m_has_extended_priority = 1;

	m_layer_config = std::make_unique<uint16_t[]>(0x8/2);
}

/*************************************************************************

    Legionnaire Spriteram (similar to Dcon)
    ---------------------

    It has "big sprites" created by setting width or height >0. Tile
    numbers are read consecutively.

    +0   x....... ........  Sprite enable
    +0   .x...... ........  Flip x
    +0   ..x..... ........  Flip y ???
    +0   ...xxx.. ........  Width: do this many tiles horizontally
    +0   ......xx x.......  Height: do this many tiles vertically
    +0   ........ .x......  Tile bank,used in Denjin Makai / extra Priority in Grainbow (to external pin?)
    +0   ........ ..xxxxxx  Color bank

    +1   xx...... ........  Priority? (1=high?)
    +1   ..xxxxxx xxxxxxxx  Tile number

    +2   ----xxxx xxxxxxxx  X coordinate (signed)

    +3   b------- --------  more tile banking used by Denjin Makai
    +3   ----xxxx xxxxxxxx  Y coordinate (signed)

*************************************************************************/

void legionna_state::draw_sprites(screen_device &screen, bitmap_ind16 &bitmap,const rectangle &cliprect)
{
	uint16_t *spriteram16 = m_spriteram;
	int offs,fx,fy,x,y,color,sprite,cur_pri;
	int dx,dy,ax,ay;
	int pri_mask;

	for (offs = 0;offs < 0x400;offs += 4)
	{
		uint16_t data = spriteram16[offs];
		if (!(data &0x8000)) continue;

		pri_mask = 0;

		if (m_has_extended_priority)
		{
			cur_pri = (spriteram16[offs+1] & 0xc000) >> 14;

			if(data & 0x0040)
			{
				cur_pri |= 0x4; // definitely seems to be needed by grainbow
			}

			//
			// -4 behind bg? (mask sprites)
			// -32 behind mid
			// -256 behind tx
			// 0    above all

			// is the low bit REALLY priority?

			switch (cur_pri)
			{
				case 0: pri_mask = -256; break; // gumdam swamp monster l2
				case 1: pri_mask = -256; break; // cupsoc
				case 2: pri_mask = -4; break; // masking effect for gundam l2 monster
				case 3: pri_mask = -4; break; // cupsoc (not sure what..)
				case 4: pri_mask = -32; break; // gundam level 2/3 player
				//case 5: pri_mask = 0; break;
				case 6: pri_mask = 0; break; // insert coin in gundam
				//case 7: pri_mask = 0; break;

				default: printf("unhandled pri %d\n",cur_pri); pri_mask=0;
			}

		}
		else
		{
			cur_pri = (spriteram16[offs+1] & 0xc000) >> 14;
			pri_mask = m_sprite_pri_mask[cur_pri];
			#if 0
			static uint8_t pri_test;

			if(machine().input().code_pressed_once(KEYCODE_A))
				pri_test++;

			if(machine().input().code_pressed_once(KEYCODE_A))
				pri_test--;

			pri_test&=3;
			popmessage("%02x",pri_test);

			// quick and dirty priority tester
			if(cur_pri == pri_test)
			{
				static uint16_t test = 0xffff;

				if(machine().input().code_pressed_once(KEYCODE_Q))
					test^=1;

				if(machine().input().code_pressed_once(KEYCODE_W))
					test^=2;

				if(machine().input().code_pressed_once(KEYCODE_E))
					test^=4;

				if(machine().input().code_pressed_once(KEYCODE_R))
					test^=8;

				if(machine().input().code_pressed_once(KEYCODE_T))
					test^=0x10;

				if(machine().input().code_pressed_once(KEYCODE_Y))
					test^=0x20;

				if(machine().input().code_pressed_once(KEYCODE_U))
					test^=0x40;

				if(machine().input().code_pressed_once(KEYCODE_I))
					test^=0x80;

				pri_mask = 0xffff & test;
				data = (data & 0xffc0) | (machine().rand() & 0x3f);
				popmessage("%04x %04x %d",pri_mask,test,pri_test);
				//pri_mask = test;
			}
			#endif
		}

		sprite = spriteram16[offs+1];

		sprite &= 0x3fff;

		if (m_has_extended_banking)
		{
			if(data & 0x0040)
			{
				sprite |= 0x4000;//tile banking,used in Denjin Makai
			}
			if(spriteram16[offs+3] & 0x8000)
			{
				sprite |= 0x8000;//tile banking?,used in Denjin Makai
			}
		}


		y = spriteram16[offs+3];
		x = spriteram16[offs+2];

		/* heated barrel hardware seems to need 0x1ff with 0x100 sign bit for sprite warp,
		   this doesn't work on denjin makai as the visible area is larger */
		if (cliprect.max_x<(320-1))
		{
			x&=0x1ff;
			y&=0x1ff;

			if (x&0x100) x-=0x200;
			if (y&0x100) y-=0x200;
		}
		else
		{
			x&=0xfff;
			y&=0xfff;

			if (x&0x800) x-=0x1000;
			if (y&0x800) y-=0x1000;

		}


		color = (data &0x3f) + 0x40;
		fx =  (data &0x4000) >> 14;
		fy =  (data &0x2000) >> 13;
		dy = ((data &0x0380) >> 7)  + 1;
		dx = ((data &0x1c00) >> 10) + 1;

		if (!fx)
		{
			if(!fy)
			{
				for (ax=0; ax<dx; ax++)
					for (ay=0; ay<dy; ay++)
					{
						m_gfxdecode->gfx(3)->prio_transpen(bitmap,cliprect,
						sprite++,
						color,fx,fy,(x+ax*16)+m_sprite_xoffs,y+ay*16+m_sprite_yoffs,
						screen.priority(),pri_mask, 15);
					}
			}
			else
			{
				for (ax=0; ax<dx; ax++)
					for (ay=0; ay<dy; ay++)
					{
						m_gfxdecode->gfx(3)->prio_transpen(bitmap,cliprect,
						sprite++,
						color,fx,fy,(x+ax*16)+m_sprite_xoffs,y+(dy-ay-1)*16+m_sprite_yoffs,
						screen.priority(),pri_mask,15);
					}
			}
		}
		else
		{
			if(!fy)
			{
				for (ax=0; ax<dx; ax++)
					for (ay=0; ay<dy; ay++)
					{
						m_gfxdecode->gfx(3)->prio_transpen(bitmap,cliprect,
						sprite++,
						color,fx,fy,(x+(dx-ax-1)*16)+m_sprite_xoffs,y+ay*16+m_sprite_yoffs,
						screen.priority(),pri_mask,15);
					}
			}
			else
			{
				for (ax=0; ax<dx; ax++)
					for (ay=0; ay<dy; ay++)
					{
						m_gfxdecode->gfx(3)->prio_transpen(bitmap,cliprect,
						sprite++,
						color,fx,fy,(x+(dx-ax-1)*16)+m_sprite_xoffs,y+(dy-ay-1)*16+m_sprite_yoffs,
						screen.priority(),pri_mask, 15);
					}
			}
		}
	}
}

uint32_t legionna_state::screen_update_legionna(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	/* Setup the tilemaps */
	screen.priority().fill(0, cliprect);
	bitmap.fill(m_palette->black_pen(), cliprect);    /* wrong color? */

	if (!(m_layer_disable&0x0001)) m_midground_layer->draw(screen, bitmap, cliprect, 0, 0);
	if (!(m_layer_disable&0x0002)) m_background_layer->draw(screen, bitmap, cliprect, 0, 1);
	if (!(m_layer_disable&0x0004)) m_foreground_layer->draw(screen, bitmap, cliprect, 0, 2);
	if (!(m_layer_disable&0x0008)) m_text_layer->draw(screen, bitmap, cliprect, 0, 4);

	if (!(m_layer_disable&0x0010))
		draw_sprites(screen,bitmap,cliprect);

	if (machine().input().code_pressed_once(KEYCODE_Z))
		if (m_raiden2cop) m_raiden2cop->dump_table();

	return 0;
}

uint32_t legionna_state::screen_update_heatbrl(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	/* Setup the tilemaps */
	screen.priority().fill(0, cliprect);
	bitmap.fill(m_palette->black_pen(), cliprect);    /* wrong color? */

	// TODO: priority order is different than anything else?
	if (!(m_layer_disable&0x0004)) m_foreground_layer->draw(screen, bitmap, cliprect, 0, 0);
	if (!(m_layer_disable&0x0002)) m_midground_layer->draw(screen, bitmap, cliprect, 0, 1);
	if (!(m_layer_disable&0x0001)) m_background_layer->draw(screen, bitmap, cliprect, 0, 2);
	if (!(m_layer_disable&0x0008)) m_text_layer->draw(screen, bitmap, cliprect, 0, 4);

	if (!(m_layer_disable&0x0010))
		draw_sprites(screen,bitmap,cliprect);

	if (machine().input().code_pressed_once(KEYCODE_Z))
		if (m_raiden2cop) m_raiden2cop->dump_table();

	return 0;
}


uint32_t legionna_state::screen_update_godzilla(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	bitmap.fill(0x0200, cliprect);
	screen.priority().fill(0, cliprect);

	if (!(m_layer_disable&0x0001)) m_background_layer->draw(screen, bitmap, cliprect, 0, 0);
	if (!(m_layer_disable&0x0002)) m_midground_layer->draw(screen, bitmap, cliprect, 0, 1);
	if (!(m_layer_disable&0x0004)) m_foreground_layer->draw(screen, bitmap, cliprect, 0, 2);
	if (!(m_layer_disable&0x0008)) m_text_layer->draw(screen, bitmap, cliprect, 0, 4);

	if (!(m_layer_disable&0x0010))
		draw_sprites(screen,bitmap,cliprect);

	if (machine().input().code_pressed_once(KEYCODE_Z))
		if (m_raiden2cop) m_raiden2cop->dump_table();


	return 0;
}

uint32_t legionna_state::screen_update_grainbow(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	bitmap.fill(m_palette->black_pen(), cliprect);
	screen.priority().fill(0, cliprect);

	if(!(m_layer_disable & 1))
		m_background_layer->draw(screen, bitmap, cliprect, 0,1);

	if(!(m_layer_disable & 2))
		m_midground_layer->draw(screen, bitmap, cliprect, 0,2);

	if(!(m_layer_disable & 4))
		m_foreground_layer->draw(screen, bitmap, cliprect, 0,4);

	if(!(m_layer_disable & 8))
		m_text_layer->draw(screen, bitmap, cliprect, 0,8);

	if (!(m_layer_disable&0x0010))
		draw_sprites(screen,bitmap,cliprect);

	if (machine().input().code_pressed_once(KEYCODE_Z))
		if (m_raiden2cop) m_raiden2cop->dump_table();

	return 0;
}
