// license:BSD-3-Clause
// copyright-holders:Manuel Abadia
#include "sound/namco.h"

class baraduke_state : public driver_device
{
public:
	baraduke_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_spriteram(*this, "spriteram"),
		m_videoram(*this, "videoram"),
		m_textram(*this, "textram"),
		m_maincpu(*this, "maincpu"),
		m_cus30(*this, "namco"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette") { }

	int m_inputport_selected;
	int m_counter;
	required_shared_ptr<uint8_t> m_spriteram;
	required_shared_ptr<uint8_t> m_videoram;
	required_shared_ptr<uint8_t> m_textram;
	required_device<cpu_device> m_maincpu;
	required_device<namco_cus30_device> m_cus30;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	tilemap_t *m_tx_tilemap;
	tilemap_t *m_bg_tilemap[2];
	int m_xscroll[2];
	int m_yscroll[2];
	int m_copy_sprites;
	DECLARE_WRITE8_MEMBER(inputport_select_w);
	DECLARE_READ8_MEMBER(inputport_r);
	DECLARE_WRITE8_MEMBER(baraduke_lamps_w);
	DECLARE_WRITE8_MEMBER(baraduke_irq_ack_w);
	DECLARE_READ8_MEMBER(soundkludge_r);
	DECLARE_READ8_MEMBER(readFF);
	DECLARE_READ8_MEMBER(baraduke_videoram_r);
	DECLARE_WRITE8_MEMBER(baraduke_videoram_w);
	DECLARE_READ8_MEMBER(baraduke_textram_r);
	DECLARE_WRITE8_MEMBER(baraduke_textram_w);
	DECLARE_WRITE8_MEMBER(baraduke_scroll0_w);
	DECLARE_WRITE8_MEMBER(baraduke_scroll1_w);
	DECLARE_READ8_MEMBER(baraduke_spriteram_r);
	DECLARE_WRITE8_MEMBER(baraduke_spriteram_w);
	DECLARE_DRIVER_INIT(baraduke);
	TILEMAP_MAPPER_MEMBER(tx_tilemap_scan);
	TILE_GET_INFO_MEMBER(tx_get_tile_info);
	TILE_GET_INFO_MEMBER(get_tile_info0);
	TILE_GET_INFO_MEMBER(get_tile_info1);
	virtual void video_start() override;
	DECLARE_PALETTE_INIT(baraduke);
	uint32_t screen_update_baraduke(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	DECLARE_WRITE_LINE_MEMBER(screen_vblank_baraduke);
	void scroll_w(address_space &space, int layer, int offset, int data);
	void draw_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect, int sprite_priority);
	void set_scroll(int layer);
};
