// license:BSD-3-Clause
// copyright-holders:Pierpaolo Prazzoli
class rollrace_state : public driver_device
{
public:
	rollrace_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_videoram(*this, "videoram"),
		m_colorram(*this, "colorram"),
		m_spriteram(*this, "spriteram") { }

	required_device<cpu_device> m_maincpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_shared_ptr<uint8_t> m_videoram;
	required_shared_ptr<uint8_t> m_colorram;
	required_shared_ptr<uint8_t> m_spriteram;

	tilemap_t *m_fg_tilemap;
	int m_charbank[2];
	int m_bkgpage;
	int m_bkgflip;
	int m_chrbank;
	int m_bkgpen;
	int m_bkgcol;
	int m_flipy;
	int m_flipx;
	int m_spritebank;

	uint8_t m_nmi_mask;
	uint8_t m_sound_nmi_mask;

	DECLARE_READ8_MEMBER(fake_d800_r);
	DECLARE_WRITE8_MEMBER(fake_d800_w);
	DECLARE_WRITE8_MEMBER(nmi_mask_w);
	DECLARE_WRITE8_MEMBER(sound_nmi_mask_w);
	DECLARE_WRITE8_MEMBER(coin_w);
	DECLARE_WRITE8_MEMBER(charbank_w);
	DECLARE_WRITE8_MEMBER(bkgpen_w);
	DECLARE_WRITE8_MEMBER(spritebank_w);
	DECLARE_WRITE8_MEMBER(backgroundpage_w);
	DECLARE_WRITE8_MEMBER(backgroundcolor_w);
	DECLARE_WRITE8_MEMBER(flipy_w);
	DECLARE_WRITE8_MEMBER(flipx_w);
	DECLARE_WRITE8_MEMBER(vram_w);
	DECLARE_WRITE8_MEMBER(cram_w);
	TILE_GET_INFO_MEMBER(get_fg_tile_info);
	void tilemap_refresh_flip();

	DECLARE_PALETTE_INIT(rollrace);
	virtual void machine_start() override;
	virtual void video_start() override;


	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	INTERRUPT_GEN_MEMBER(vblank_irq);
	INTERRUPT_GEN_MEMBER(sound_timer_irq);
};
