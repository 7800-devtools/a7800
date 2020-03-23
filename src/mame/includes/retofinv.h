// license:BSD-3-Clause
// copyright-holders:Jarek Parchanski, Andrea Mazzoleni

#include "machine/taito68705interface.h"

#include "machine/gen_latch.h"


class retofinv_state : public driver_device
{
public:
	retofinv_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_audiocpu(*this, "audiocpu")
		, m_subcpu(*this, "sub")
		, m_68705(*this, "68705")
		, m_gfxdecode(*this, "gfxdecode")
		, m_palette(*this, "palette")
		, m_soundlatch(*this, "soundlatch")
		, m_fg_videoram(*this, "fg_videoram")
		, m_sharedram(*this, "sharedram")
		, m_bg_videoram(*this, "bg_videoram")
	{
	}

	DECLARE_WRITE8_MEMBER(cpu1_reset_w);
	DECLARE_WRITE8_MEMBER(cpu2_reset_w);
	DECLARE_WRITE8_MEMBER(mcu_reset_w);
	DECLARE_WRITE8_MEMBER(cpu2_m6000_w);
	DECLARE_READ8_MEMBER(cpu0_mf800_r);
	DECLARE_WRITE8_MEMBER(soundcommand_w);
	DECLARE_WRITE8_MEMBER(irq0_ack_w);
	DECLARE_WRITE8_MEMBER(irq1_ack_w);
	DECLARE_WRITE8_MEMBER(coincounter_w);
	DECLARE_WRITE8_MEMBER(coinlockout_w);
	DECLARE_READ8_MEMBER(mcu_status_r);
	DECLARE_WRITE8_MEMBER(bg_videoram_w);
	DECLARE_WRITE8_MEMBER(fg_videoram_w);
	DECLARE_WRITE8_MEMBER(gfx_ctrl_w);

	TILEMAP_MAPPER_MEMBER(tilemap_scan);
	TILE_GET_INFO_MEMBER(bg_get_tile_info);
	TILE_GET_INFO_MEMBER(fg_get_tile_info);

	DECLARE_PALETTE_INIT(retofinv);
	DECLARE_PALETTE_INIT(retofinv_bl);

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	INTERRUPT_GEN_MEMBER(main_vblank_irq);
	INTERRUPT_GEN_MEMBER(sub_vblank_irq);

protected:
	virtual void machine_start() override;
	virtual void video_start() override;

	void draw_sprites(bitmap_ind16 &bitmap);

	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	required_device<cpu_device> m_subcpu;
	optional_device<taito68705_mcu_device> m_68705;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<generic_latch_8_device> m_soundlatch;

	required_shared_ptr<uint8_t> m_fg_videoram;
	required_shared_ptr<uint8_t> m_sharedram;
	required_shared_ptr<uint8_t> m_bg_videoram;

	uint8_t m_main_irq_mask;
	uint8_t m_sub_irq_mask;
	uint8_t m_cpu2_m6000;
	int m_fg_bank;
	int m_bg_bank;
	tilemap_t *m_bg_tilemap;
	tilemap_t *m_fg_tilemap;
};
