// license:BSD-3-Clause
// copyright-holders:Carlos A. Lozano
#include "audio/seibu.h"

class bloodbro_state : public driver_device
{
public:
	bloodbro_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_audiocpu(*this, "audiocpu"),
		m_seibu_sound(*this, "seibu_sound"),
		m_spriteram(*this, "spriteram"),
		m_bgvideoram(*this, "bgvideoram"),
		m_fgvideoram(*this, "fgvideoram"),
		m_txvideoram(*this, "txvideoram") { }

	required_device<cpu_device> m_maincpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<cpu_device> m_audiocpu;
	required_device<seibu_sound_device> m_seibu_sound;

	required_shared_ptr<uint16_t> m_spriteram;
	required_shared_ptr<uint16_t> m_bgvideoram;
	required_shared_ptr<uint16_t> m_fgvideoram;
	required_shared_ptr<uint16_t> m_txvideoram;

	uint16_t m_scrollram[6];
	uint16_t m_layer_en;

	tilemap_t *m_bg_tilemap;
	tilemap_t *m_fg_tilemap;
	tilemap_t *m_tx_tilemap;

	DECLARE_WRITE16_MEMBER(bgvideoram_w);
	DECLARE_WRITE16_MEMBER(fgvideoram_w);
	DECLARE_WRITE16_MEMBER(txvideoram_w);
	DECLARE_WRITE16_MEMBER(layer_en_w);
	DECLARE_WRITE16_MEMBER(layer_scroll_w);
	DECLARE_WRITE16_MEMBER(weststry_layer_scroll_w);
	DECLARE_WRITE8_MEMBER(weststry_soundlatch_w);

	TILE_GET_INFO_MEMBER(get_bg_tile_info);
	TILE_GET_INFO_MEMBER(get_fg_tile_info);
	TILE_GET_INFO_MEMBER(get_tx_tile_info);

	virtual void video_start() override;

	uint32_t screen_update_bloodbro(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_weststry(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_skysmash(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void bloodbro_draw_sprites(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void weststry_draw_sprites(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	DECLARE_DRIVER_INIT(weststry);
};
