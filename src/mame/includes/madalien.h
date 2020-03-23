// license:GPL-2.0+
// copyright-holders:Norbert Kehrer
/***************************************************************************

    Mad Alien (c) 1980 Data East Corporation

    Original driver by Norbert Kehrer (February 2004)

***************************************************************************/

#include "machine/gen_latch.h"
#include "sound/discrete.h"


#define MADALIEN_MAIN_CLOCK     XTAL_10_595MHz


class madalien_state : public driver_device
{
public:
	madalien_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_videoram(*this, "videoram"),
		m_charram(*this, "charram"),
		m_video_control(*this, "video_control"),
		m_shift_hi(*this, "shift_hi"),
		m_shift_lo(*this, "shift_lo"),
		m_video_flags(*this, "video_flags"),
		m_headlight_pos(*this, "headlight_pos"),
		m_edge1_pos(*this, "edge1_pos"),
		m_edge2_pos(*this, "edge2_pos"),
		m_scroll(*this, "scroll"),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_discrete(*this, "discrete"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_soundlatch(*this, "soundlatch"),
		m_soundlatch2(*this, "soundlatch2") { }

	required_shared_ptr<uint8_t> m_videoram;
	required_shared_ptr<uint8_t> m_charram;
	required_shared_ptr<uint8_t> m_video_control;
	required_shared_ptr<uint8_t> m_shift_hi;
	required_shared_ptr<uint8_t> m_shift_lo;
	required_shared_ptr<uint8_t> m_video_flags;
	required_shared_ptr<uint8_t> m_headlight_pos;
	required_shared_ptr<uint8_t> m_edge1_pos;
	required_shared_ptr<uint8_t> m_edge2_pos;
	required_shared_ptr<uint8_t> m_scroll;

	tilemap_t *m_tilemap_fg;
	tilemap_t *m_tilemap_edge1[4];
	tilemap_t *m_tilemap_edge2[4];
	std::unique_ptr<bitmap_ind16> m_headlight_bitmap;
	DECLARE_READ8_MEMBER(shift_r);
	DECLARE_READ8_MEMBER(shift_rev_r);
	DECLARE_WRITE8_MEMBER(madalien_output_w);
	DECLARE_WRITE8_MEMBER(madalien_sound_command_w);
	DECLARE_READ8_MEMBER(madalien_sound_command_r);
	DECLARE_WRITE8_MEMBER(madalien_videoram_w);
	DECLARE_WRITE8_MEMBER(madalien_charram_w);
	DECLARE_INPUT_CHANGED_MEMBER(coin_inserted);
	DECLARE_WRITE8_MEMBER(madalien_portA_w);
	DECLARE_WRITE8_MEMBER(madalien_portB_w);
	TILEMAP_MAPPER_MEMBER(scan_mode0);
	TILEMAP_MAPPER_MEMBER(scan_mode1);
	TILEMAP_MAPPER_MEMBER(scan_mode2);
	TILEMAP_MAPPER_MEMBER(scan_mode3);
	TILE_GET_INFO_MEMBER(get_tile_info_BG_1);
	TILE_GET_INFO_MEMBER(get_tile_info_BG_2);
	TILE_GET_INFO_MEMBER(get_tile_info_FG);
	DECLARE_VIDEO_START(madalien);
	DECLARE_PALETTE_INIT(madalien);
	uint32_t screen_update_madalien(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	inline int scan_helper(int col, int row, int section);
	void draw_edges(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int flip, int scroll_mode);
	void draw_headlight(bitmap_ind16 &bitmap, const rectangle &cliprect, int flip);
	void draw_foreground(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int flip);
	inline uint8_t shift_common(uint8_t hi, uint8_t lo);
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	required_device<discrete_device> m_discrete;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<generic_latch_8_device> m_soundlatch;
	required_device<generic_latch_8_device> m_soundlatch2;
};
/*----------- defined in video/madalien.c -----------*/

MACHINE_CONFIG_EXTERN( madalien_video );

/*----------- defined in audio/madalien.c -----------*/

DISCRETE_SOUND_EXTERN( madalien );

/* Discrete Sound Input Nodes */
#define MADALIEN_8910_PORTA         NODE_01
#define MADALIEN_8910_PORTB         NODE_02
