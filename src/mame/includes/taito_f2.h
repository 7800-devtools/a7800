// license:BSD-3-Clause
// copyright-holders:David Graves, Bryan McPhail, Brad Oliver, Andrew Prime, Brian Troha, Nicola Salmoria
#ifndef MAME_INCLUDES_TAITO_F2_H
#define MAME_INCLUDES_TAITO_F2_H

#pragma once


#include "machine/taitocchip.h"
#include "machine/taitoio.h"

#include "sound/okim6295.h"
#include "video/tc0100scn.h"
#include "video/tc0110pcr.h"
#include "video/tc0280grd.h"
#include "video/tc0360pri.h"
#include "video/tc0480scp.h"


class taitof2_state : public driver_device
{
public:
	enum
	{
		TIMER_TAITOF2_INTERRUPT6
	};

	struct f2_tempsprite
	{
		int code, color;
		int flipx, flipy;
		int x, y;
		int zoomx, zoomy;
		int primask;
	};

	taitof2_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
			m_sprite_extension(*this, "sprite_ext"),
			m_spriteram(*this, "spriteram"),
			m_maincpu(*this, "maincpu"),
			m_audiocpu(*this, "audiocpu"),
			m_cchip(*this, "cchip"),
			m_oki(*this, "oki"),
			m_tc0100scn(*this, "tc0100scn"),
			m_tc0100scn_1(*this, "tc0100scn_1"),
			m_tc0100scn_2(*this, "tc0100scn_2"),
			m_tc0110pcr(*this, "tc0110pcr"),
			m_tc0360pri(*this, "tc0360pri"),
			m_tc0280grd(*this, "tc0280grd"),
			m_tc0430grw(*this, "tc0430grw"),
			m_tc0480scp(*this, "tc0480scp"),
			m_tc0220ioc(*this, "tc0220ioc"),
			m_tc0510nio(*this, "tc0510nio"),
			m_gfxdecode(*this, "gfxdecode"),
			m_palette(*this, "palette")
			{ }

	/* memory pointers */
	optional_shared_ptr<uint16_t> m_sprite_extension;
	required_shared_ptr<uint16_t> m_spriteram;
	std::unique_ptr<uint16_t[]>        m_spriteram_buffered;
	std::unique_ptr<uint16_t[]>        m_spriteram_delayed;

	/* video-related */
	std::unique_ptr<struct f2_tempsprite[]> m_spritelist;
	int             m_sprite_type;

	uint16_t          m_spritebank[8];
//  uint16_t          m_spritebank_eof[8];
	uint16_t          m_spritebank_buffered[8];

	int32_t           m_sprites_disabled;
	int32_t           m_sprites_active_area;
	int32_t           m_sprites_master_scrollx;
	int32_t           m_sprites_master_scrolly;
	/* remember flip status over frames because driftout can fail to set it */
	int32_t           m_sprites_flipscreen;

	/* On the left hand screen edge (assuming horiz screen, no
	   screenflip: in screenflip it is the right hand edge etc.)
	   there may be 0-3 unwanted pixels in both tilemaps *and*
	   sprites. To erase this we use f2_hide_pixels (0 to +3). */

	int32_t           m_hide_pixels;
	int32_t           m_flip_hide_pixels; /* Different in some games */

	int32_t           m_pivot_xdisp;  /* Needed in games with a pivot layer */
	int32_t           m_pivot_ydisp;

	int32_t           m_game;

	uint8_t           m_tilepri[6]; // todo - move into taitoic.c
	uint8_t           m_spritepri[6]; // todo - move into taitoic.c
	uint8_t           m_spriteblendmode; // todo - move into taitoic.c

	int             m_prepare_sprites;

	/* misc */
	int32_t           m_mjnquest_input;
	int             m_last[2];
	int             m_nibble;
	int32_t           m_driveout_sound_latch;
	int32_t           m_oki_bank;
	emu_timer       *m_int6_timer;

	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	optional_device<taito_cchip_device> m_cchip;
	optional_device<okim6295_device> m_oki;
	optional_device<tc0100scn_device> m_tc0100scn;
	optional_device<tc0100scn_device> m_tc0100scn_1;
	optional_device<tc0100scn_device> m_tc0100scn_2;
	optional_device<tc0110pcr_device> m_tc0110pcr;
	optional_device<tc0360pri_device> m_tc0360pri;
	optional_device<tc0280grd_device> m_tc0280grd;
	optional_device<tc0280grd_device> m_tc0430grw;
	optional_device<tc0480scp_device> m_tc0480scp;
	optional_device<tc0220ioc_device> m_tc0220ioc;
	optional_device<tc0510nio_device> m_tc0510nio;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;

	DECLARE_WRITE8_MEMBER(coin_nibble_w);
	DECLARE_WRITE16_MEMBER(growl_coin_word_w);
	DECLARE_WRITE8_MEMBER(taitof2_4p_coin_word_w);
	DECLARE_READ16_MEMBER(cameltry_paddle_r);
	DECLARE_READ16_MEMBER(mjnquest_dsw_r);
	DECLARE_READ16_MEMBER(mjnquest_input_r);
	DECLARE_WRITE16_MEMBER(mjnquest_inputselect_w);
	DECLARE_WRITE8_MEMBER(sound_bankswitch_w);
	DECLARE_READ8_MEMBER(driveout_sound_command_r);
	DECLARE_WRITE8_MEMBER(oki_bank_w);
	DECLARE_WRITE16_MEMBER(driveout_sound_command_w);
	DECLARE_WRITE16_MEMBER(taitof2_sprite_extension_w);
	DECLARE_WRITE16_MEMBER(taitof2_spritebank_w);
	DECLARE_WRITE16_MEMBER(koshien_spritebank_w);
	DECLARE_WRITE8_MEMBER(cameltrya_porta_w);
	DECLARE_DRIVER_INIT(driveout);
	DECLARE_DRIVER_INIT(cameltry);
	DECLARE_DRIVER_INIT(mjnquest);
	DECLARE_DRIVER_INIT(finalb);
	DECLARE_MACHINE_START(f2);
	DECLARE_VIDEO_START(taitof2_default);
	DECLARE_MACHINE_START(common);
	DECLARE_VIDEO_START(taitof2_dondokod);
	DECLARE_VIDEO_START(taitof2_driftout);
	DECLARE_VIDEO_START(taitof2_finalb);
	DECLARE_VIDEO_START(taitof2_megab);
	DECLARE_VIDEO_START(taitof2_thundfox);
	DECLARE_VIDEO_START(taitof2_ssi);
	DECLARE_VIDEO_START(taitof2_gunfront);
	DECLARE_VIDEO_START(taitof2_growl);
	DECLARE_VIDEO_START(taitof2_mjnquest);
	DECLARE_VIDEO_START(taitof2_footchmp);
	DECLARE_VIDEO_START(taitof2_hthero);
	DECLARE_VIDEO_START(taitof2_koshien);
	DECLARE_VIDEO_START(taitof2_yuyugogo);
	DECLARE_VIDEO_START(taitof2_ninjak);
	DECLARE_VIDEO_START(taitof2_solfigtr);
	DECLARE_VIDEO_START(taitof2_pulirula);
	DECLARE_VIDEO_START(taitof2_metalb);
	DECLARE_VIDEO_START(taitof2_qzchikyu);
	DECLARE_VIDEO_START(taitof2_yesnoj);
	DECLARE_VIDEO_START(taitof2_deadconx);
	DECLARE_VIDEO_START(taitof2_deadconxj);
	DECLARE_VIDEO_START(taitof2_dinorex);
	DECLARE_VIDEO_START(taitof2_quiz);
	uint32_t screen_update_taitof2(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_taitof2_pri_roz(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_taitof2_pri(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_taitof2_thundfox(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_taitof2_ssi(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_taitof2_deadconx(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_taitof2_yesnoj(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_taitof2_metalb(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	DECLARE_WRITE_LINE_MEMBER(screen_vblank_no_buffer);
	DECLARE_WRITE_LINE_MEMBER(screen_vblank_partial_buffer_delayed);
	DECLARE_WRITE_LINE_MEMBER(screen_vblank_partial_buffer_delayed_thundfox);
	DECLARE_WRITE_LINE_MEMBER(screen_vblank_full_buffer_delayed);
	DECLARE_WRITE_LINE_MEMBER(screen_vblank_partial_buffer_delayed_qzchikyu);
	INTERRUPT_GEN_MEMBER(taitof2_interrupt);
	void reset_driveout_sound_region();
	void taitof2_core_vh_start (int sprite_type, int hide, int flip_hide );
	void draw_sprites( screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int *primasks, int uses_tc360_mixer );
	void update_spritebanks(  );
	void taitof2_handle_sprite_buffering(  );
	void taitof2_update_sprites_active_area(  );
	void draw_roz_layer( screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, uint32_t priority);
	void taito_f2_tc360_spritemixdraw(screen_device &screen, bitmap_ind16 &dest_bmp, const rectangle &clip, gfx_element *gfx,
	uint32_t code, uint32_t color, int flipx, int flipy, int sx, int sy, int scalex, int scaley );

protected:
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
};

#endif // MAME_INCLUDES_TAITO_F2_H
