// license:BSD-3-Clause
// copyright-holders:David Graves
/*************************************************************************

    World Grand Prix

*************************************************************************/

#include "audio/taitosnd.h"
#include "machine/taitoio.h"
#include "video/tc0100scn.h"


class wgp_state : public driver_device
{
public:
	enum
	{
		TIMER_WGP_INTERRUPT4,
		TIMER_WGP_INTERRUPT6,
		TIMER_WGP_CPUB_INTERRUPT6
	};

	wgp_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_spritemap(*this, "spritemap"),
		m_spriteram(*this, "spriteram"),
		m_pivram(*this, "pivram"),
		m_piv_ctrlram(*this, "piv_ctrlram"),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_subcpu(*this, "sub"),
		m_tc0100scn(*this, "tc0100scn"),
		m_tc0140syt(*this, "tc0140syt"),
		m_tc0220ioc(*this, "tc0220ioc"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_z80bank(*this, "z80bank"),
		m_steer(*this, "STEER"),
		m_unknown(*this, "UNKNOWN"),
		m_fake(*this, "FAKE")
	{ }

	/* memory pointers */
	required_shared_ptr<uint16_t> m_spritemap;
	required_shared_ptr<uint16_t> m_spriteram;
	required_shared_ptr<uint16_t> m_pivram;
	required_shared_ptr<uint16_t> m_piv_ctrlram;

	/* video-related */
	tilemap_t   *m_piv_tilemap[3];
	uint16_t      m_piv_ctrl_reg;
	uint16_t      m_piv_zoom[3];
	uint16_t      m_piv_scrollx[3];
	uint16_t      m_piv_scrolly[3];
	uint16_t      m_rotate_ctrl[8];
	int         m_piv_xoffs;
	int         m_piv_yoffs;
	uint8_t       m_dislayer[4];

	/* misc */
	uint16_t      m_cpua_ctrl;
	uint16_t      m_port_sel;

	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	required_device<cpu_device> m_subcpu;
	required_device<tc0100scn_device> m_tc0100scn;
	required_device<tc0140syt_device> m_tc0140syt;
	required_device<tc0220ioc_device> m_tc0220ioc;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_memory_bank m_z80bank;
	optional_ioport m_steer;
	optional_ioport m_unknown;
	optional_ioport m_fake;

	DECLARE_WRITE8_MEMBER(coins_w);
	DECLARE_WRITE16_MEMBER(cpua_ctrl_w);
	DECLARE_READ16_MEMBER(lan_status_r);
	DECLARE_WRITE16_MEMBER(rotate_port_w);
	DECLARE_READ16_MEMBER(wgp_adinput_r);
	DECLARE_WRITE16_MEMBER(wgp_adinput_w);
	DECLARE_WRITE8_MEMBER(sound_bankswitch_w);
	DECLARE_WRITE16_MEMBER(wgp_sound_w);
	DECLARE_READ16_MEMBER(wgp_sound_r);
	DECLARE_READ16_MEMBER(wgp_pivram_word_r);
	DECLARE_WRITE16_MEMBER(wgp_pivram_word_w);
	DECLARE_READ16_MEMBER(wgp_piv_ctrl_word_r);
	DECLARE_WRITE16_MEMBER(wgp_piv_ctrl_word_w);
	DECLARE_DRIVER_INIT(wgp);
	DECLARE_DRIVER_INIT(wgp2);
	TILE_GET_INFO_MEMBER(get_piv0_tile_info);
	TILE_GET_INFO_MEMBER(get_piv1_tile_info);
	TILE_GET_INFO_MEMBER(get_piv2_tile_info);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	DECLARE_VIDEO_START(wgp2);
	uint32_t screen_update_wgp(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	INTERRUPT_GEN_MEMBER(wgp_cpub_interrupt);
	void wgp_postload();
	inline void common_get_piv_tile_info( tile_data &tileinfo, int tile_index, int num );
	void wgp_core_vh_start( int piv_xoffs, int piv_yoffs );
	void draw_sprites( screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int y_offs );
	void wgp_piv_layer_draw( screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int layer, int flags, uint32_t priority );
	void parse_control();

protected:
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
};
