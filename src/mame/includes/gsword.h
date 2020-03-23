// license:BSD-3-Clause
// copyright-holders:Steve Ellenoff,Jarek Parchanski

#include "machine/gen_latch.h"
#include "sound/ay8910.h"
#include "sound/msm5205.h"

class gsword_state_base : public driver_device
{
public:
	gsword_state_base(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_audiocpu(*this, "audiocpu")
		, m_subcpu(*this, "sub")
		, m_ay0(*this, "ay1")
		, m_ay1(*this, "ay2")
		, m_gfxdecode(*this, "gfxdecode")
		, m_palette(*this, "palette")
		, m_spritetile_ram(*this, "spritetile_ram")
		, m_spritexy_ram(*this, "spritexy_ram")
		, m_spriteattrib_ram(*this, "spriteattram")
		, m_videoram(*this, "videoram")
		, m_cpu2_ram(*this, "cpu2_ram")
	{
	}

	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	optional_device<cpu_device> m_subcpu;
	required_device<ay8910_device> m_ay0;
	required_device<ay8910_device> m_ay1;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;

	required_shared_ptr<uint8_t> m_spritetile_ram;
	required_shared_ptr<uint8_t> m_spritexy_ram;
	required_shared_ptr<uint8_t> m_spriteattrib_ram;
	required_shared_ptr<uint8_t> m_videoram;
	required_shared_ptr<uint8_t> m_cpu2_ram;

	int m_fake8910_0;
	int m_fake8910_1;
	int m_charbank;
	int m_charpalbank;
	int m_flipscreen;
	tilemap_t *m_bg_tilemap;

	// common
	DECLARE_WRITE8_MEMBER(videoram_w);
	DECLARE_WRITE8_MEMBER(charbank_w);
	DECLARE_WRITE8_MEMBER(videoctrl_w);
	DECLARE_WRITE8_MEMBER(scroll_w);
	DECLARE_WRITE8_MEMBER(ay8910_control_port_0_w);
	DECLARE_WRITE8_MEMBER(ay8910_control_port_1_w);
	DECLARE_READ8_MEMBER(fake_0_r);
	DECLARE_READ8_MEMBER(fake_1_r);

	TILE_GET_INFO_MEMBER(get_bg_tile_info);

	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;

	uint32_t screen_update_gsword(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void draw_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect);
};


class gsword_state : public gsword_state_base
{
public:
	gsword_state(const machine_config &mconfig, device_type type, const char *tag)
		: gsword_state_base(mconfig, type, tag)
		, m_soundlatch(*this, "soundlatch")
		, m_msm(*this, "msm")
		, m_protect_hack(false)
		, m_nmi_enable(false)
	{
	}

	DECLARE_READ8_MEMBER(hack_r);
	DECLARE_WRITE8_MEMBER(nmi_set_w);
	DECLARE_WRITE8_MEMBER(sound_command_w);
	DECLARE_WRITE8_MEMBER(adpcm_data_w);
	DECLARE_READ8_MEMBER(i8741_2_r);
	DECLARE_READ8_MEMBER(i8741_3_r);

	INTERRUPT_GEN_MEMBER(sound_interrupt);

	DECLARE_DRIVER_INIT(gsword);
	DECLARE_DRIVER_INIT(gsword2);

	DECLARE_PALETTE_INIT(gsword);

	virtual void machine_start() override;
	virtual void machine_reset() override;

private:
	required_device<generic_latch_8_device> m_soundlatch;
	required_device<msm5205_device>         m_msm;

	bool    m_protect_hack;
	bool    m_nmi_enable;
};


class josvolly_state : public gsword_state_base
{
public:
	josvolly_state(const machine_config &mconfig, device_type type, const char *tag)
		: gsword_state_base(mconfig, type, tag)
		, m_cpu2_nmi_enable(false)
		, m_mcu1_p1(0xffU)
		, m_mcu1_p2(0xffU)
		, m_mcu2_p1(0xffU)
	{
	}

	DECLARE_READ8_MEMBER(mcu1_p1_r);
	DECLARE_READ8_MEMBER(mcu1_p2_r);
	DECLARE_READ8_MEMBER(mcu2_p1_r);
	DECLARE_READ8_MEMBER(mcu2_p2_r);

	DECLARE_WRITE8_MEMBER(cpu2_nmi_enable_w);
	DECLARE_WRITE8_MEMBER(cpu2_irq_clear_w);
	DECLARE_WRITE8_MEMBER(mcu1_p1_w);
	DECLARE_WRITE8_MEMBER(mcu1_p2_w);
	DECLARE_WRITE8_MEMBER(mcu2_p1_w);
	DECLARE_WRITE8_MEMBER(mcu2_p2_w);

	DECLARE_PALETTE_INIT(josvolly);

	virtual void machine_start() override;
	virtual void machine_reset() override;

private:
	bool    m_cpu2_nmi_enable;
	u8      m_mcu1_p1;
	u8      m_mcu1_p2;
	u8      m_mcu2_p1;
};
