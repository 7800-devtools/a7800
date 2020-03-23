// license:BSD-3-Clause
// copyright-holders:Tatsuyuki Satoh
#include "sound/msm5205.h"

class appoooh_state : public driver_device
{
public:
	appoooh_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_spriteram(*this, "spriteram"),
		m_fg_videoram(*this, "fg_videoram"),
		m_fg_colorram(*this, "fg_colorram"),
		m_spriteram_2(*this, "spriteram_2"),
		m_bg_videoram(*this, "bg_videoram"),
		m_bg_colorram(*this, "bg_colorram"),
		m_decrypted_opcodes(*this, "decrypted_opcodes"),
		m_maincpu(*this, "maincpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_msm(*this, "msm") { }

	/* memory pointers */
	required_shared_ptr<uint8_t> m_spriteram;
	required_shared_ptr<uint8_t> m_fg_videoram;
	required_shared_ptr<uint8_t> m_fg_colorram;
	required_shared_ptr<uint8_t> m_spriteram_2;
	required_shared_ptr<uint8_t> m_bg_videoram;
	required_shared_ptr<uint8_t> m_bg_colorram;
	optional_shared_ptr<uint8_t> m_decrypted_opcodes;

	/* video-related */
	tilemap_t  *m_fg_tilemap;
	tilemap_t  *m_bg_tilemap;
	int m_scroll_x;
	int m_priority;

	/* sound-related */
	uint32_t   m_adpcm_data;
	uint32_t   m_adpcm_address;

	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<msm5205_device> m_msm;

	uint8_t m_nmi_mask;
	DECLARE_WRITE8_MEMBER(adpcm_w);
	DECLARE_WRITE8_MEMBER(scroll_w);
	DECLARE_WRITE8_MEMBER(fg_videoram_w);
	DECLARE_WRITE8_MEMBER(fg_colorram_w);
	DECLARE_WRITE8_MEMBER(bg_videoram_w);
	DECLARE_WRITE8_MEMBER(bg_colorram_w);
	DECLARE_WRITE8_MEMBER(out_w);
	DECLARE_DRIVER_INIT(robowresb);
	TILE_GET_INFO_MEMBER(get_fg_tile_info);
	TILE_GET_INFO_MEMBER(get_bg_tile_info);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	DECLARE_VIDEO_START(appoooh);
	DECLARE_PALETTE_INIT(appoooh);
	DECLARE_PALETTE_INIT(robowres);
	uint32_t screen_update_appoooh(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_robowres(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	INTERRUPT_GEN_MEMBER(vblank_irq);
	void appoooh_draw_sprites( bitmap_ind16 &dest_bmp, const rectangle &cliprect, gfx_element *gfx, uint8_t *sprite );
	void robowres_draw_sprites( bitmap_ind16 &dest_bmp, const rectangle &cliprect, gfx_element *gfx, uint8_t *sprite );
	DECLARE_WRITE_LINE_MEMBER(adpcm_int);
};

#define CHR1_OFST   0x00  /* palette page of char set #1 */
#define CHR2_OFST   0x10  /* palette page of char set #2 */
