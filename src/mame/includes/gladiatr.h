// license:BSD-3-Clause
// copyright-holders:Victor Trucco,Steve Ellenoff,Phil Stroffolino,Tatsuyuki Satoh,Tomasz Slanina,Nicola Salmoria,Vas Crabb

#include "machine/gen_latch.h"
#include "sound/msm5205.h"


class gladiatr_state_base : public driver_device
{
public:
	DECLARE_WRITE8_MEMBER(videoram_w);
	DECLARE_WRITE8_MEMBER(colorram_w);
	DECLARE_WRITE8_MEMBER(textram_w);
	DECLARE_WRITE8_MEMBER(paletteram_w);
	DECLARE_WRITE8_MEMBER(spritebuffer_w);

protected:
	gladiatr_state_base(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_subcpu(*this, "sub")
		, m_audiocpu(*this, "audiocpu")
		, m_cctl(*this, "cctl")
		, m_ccpu(*this, "ccpu")
		, m_ucpu(*this, "ucpu")
		, m_csnd(*this, "csnd")
		, m_gfxdecode(*this, "gfxdecode")
		, m_palette(*this, "palette")
		, m_msm(*this, "msm")
		, m_soundlatch(*this, "soundlatch")
		, m_videoram(*this, "videoram")
		, m_colorram(*this, "colorram")
		, m_textram(*this, "textram")
		, m_paletteram(*this, "paletteram")
		, m_spriteram(*this, "spriteram")
		, m_video_attributes(0)
		, m_fg_scrolly(0)
		, m_fg_tile_bank(0)
		, m_bg_tile_bank(0)
		, m_sprite_bank(0)
		, m_sprite_buffer(0)
		, m_fg_tilemap(nullptr)
		, m_bg_tilemap(nullptr)
	{
	}

	TILE_GET_INFO_MEMBER(bg_get_tile_info);
	TILE_GET_INFO_MEMBER(fg_get_tile_info);

	void draw_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect);

	required_device<cpu_device>             m_maincpu;
	required_device<cpu_device>             m_subcpu;
	required_device<cpu_device>             m_audiocpu;
	optional_device<cpu_device>             m_cctl;
	optional_device<cpu_device>             m_ccpu;
	optional_device<cpu_device>             m_ucpu;
	optional_device<cpu_device>             m_csnd;
	required_device<gfxdecode_device>       m_gfxdecode;
	required_device<palette_device>         m_palette;
	required_device<msm5205_device>         m_msm;
	required_device<generic_latch_8_device> m_soundlatch;

	required_shared_ptr<uint8_t>            m_videoram;
	required_shared_ptr<uint8_t>            m_colorram;
	required_shared_ptr<uint8_t>            m_textram;
	required_shared_ptr<uint8_t>            m_paletteram;
	required_shared_ptr<uint8_t>            m_spriteram;

	int         m_video_attributes;
	int         m_fg_scrolly;
	int         m_fg_tile_bank;
	int         m_bg_tile_bank;
	int         m_sprite_bank;
	int         m_sprite_buffer;

	tilemap_t   *m_fg_tilemap;
	tilemap_t   *m_bg_tilemap;
};

class gladiatr_state : public gladiatr_state_base
{
public:
	gladiatr_state(const machine_config &mconfig, device_type type, const char *tag)
		: gladiatr_state_base(mconfig, type, tag)
		, m_dsw1(*this, "DSW1")
		, m_dsw2(*this, "DSW2")
		, m_in0(*this, "IN0")
		, m_in1(*this, "IN1")
		, m_in2(*this, "IN2")
		, m_coins(*this, "COINS")
		, m_tclk_val(false)
		, m_cctl_p1(0xff)
		, m_cctl_p2(0xff)
		, m_ucpu_p1(0xff)
		, m_csnd_p1(0xff)
		, m_fg_scrollx(0)
		, m_bg_scrollx(0)
		, m_bg_scrolly(0)
	{
	}

	DECLARE_WRITE8_MEMBER(gladiatr_spritebank_w);
	DECLARE_WRITE8_MEMBER(gladiatr_video_registers_w);

	DECLARE_WRITE8_MEMBER(gladiatr_bankswitch_w);
	DECLARE_WRITE8_MEMBER(gladiator_cpu_sound_command_w);
	DECLARE_READ8_MEMBER(gladiator_cpu_sound_command_r);
	DECLARE_WRITE8_MEMBER(gladiatr_flipscreen_w);
	DECLARE_WRITE8_MEMBER(gladiatr_irq_patch_w);
	DECLARE_WRITE8_MEMBER(gladiator_int_control_w);
	DECLARE_WRITE8_MEMBER(gladiator_adpcm_w);
	DECLARE_WRITE_LINE_MEMBER(gladiator_ym_irq);

	DECLARE_WRITE_LINE_MEMBER(tclk_w);
	DECLARE_READ8_MEMBER(cctl_p1_r);
	DECLARE_READ8_MEMBER(cctl_p2_r);
	DECLARE_WRITE8_MEMBER(ccpu_p2_w);
	DECLARE_READ_LINE_MEMBER(tclk_r);
	DECLARE_READ_LINE_MEMBER(ucpu_t1_r);
	DECLARE_READ8_MEMBER(ucpu_p1_r);
	DECLARE_WRITE8_MEMBER(ucpu_p1_w);
	DECLARE_READ8_MEMBER(ucpu_p2_r);
	DECLARE_READ_LINE_MEMBER(csnd_t1_r);
	DECLARE_READ8_MEMBER(csnd_p1_r);
	DECLARE_WRITE8_MEMBER(csnd_p1_w);
	DECLARE_READ8_MEMBER(csnd_p2_r);

	DECLARE_INPUT_CHANGED_MEMBER(p1_s1);
	DECLARE_INPUT_CHANGED_MEMBER(p1_s2);
	DECLARE_INPUT_CHANGED_MEMBER(p2_s1);
	DECLARE_INPUT_CHANGED_MEMBER(p2_s2);

	DECLARE_DRIVER_INIT(gladiatr);

	DECLARE_MACHINE_RESET(gladiator);
	DECLARE_VIDEO_START(gladiatr);

	uint32_t screen_update_gladiatr(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void swap_block(uint8_t *src1,uint8_t *src2,int len);

private:
	required_ioport m_dsw1, m_dsw2;
	required_ioport m_in0, m_in1, m_in2;
	required_ioport m_coins;

	bool    m_tclk_val;
	u8      m_cctl_p1, m_cctl_p2;
	u8      m_ucpu_p1, m_csnd_p1;

	int     m_fg_scrollx;
	int     m_bg_scrollx;
	int     m_bg_scrolly;
};

class ppking_state : public gladiatr_state_base
{
public:
	ppking_state(const machine_config &mconfig, device_type type, const char *tag)
		: gladiatr_state_base(mconfig, type, tag)
		, m_nvram(*this, "nvram")
		, m_data1(0)
		, m_data2(0)
		, m_flag1(0)
		, m_flag2(0)
	{
	}

	DECLARE_READ8_MEMBER(ppking_f1_r);
	DECLARE_READ8_MEMBER(ppking_f6a3_r);
	DECLARE_WRITE8_MEMBER(ppking_qx0_w);
	DECLARE_WRITE8_MEMBER(ppking_qx1_w);
	DECLARE_WRITE8_MEMBER(ppking_qx2_w);
	DECLARE_WRITE8_MEMBER(ppking_qx3_w);
	DECLARE_READ8_MEMBER(ppking_qx2_r);
	DECLARE_READ8_MEMBER(ppking_qx3_r);
	DECLARE_READ8_MEMBER(ppking_qx0_r);
	DECLARE_READ8_MEMBER(ppking_qx1_r);
	DECLARE_WRITE8_MEMBER(ppking_video_registers_w);

	DECLARE_DRIVER_INIT(ppking);

	DECLARE_MACHINE_RESET(ppking);
	DECLARE_VIDEO_START(ppking);

	uint32_t screen_update_ppking(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

private:
	required_shared_ptr<uint8_t>    m_nvram;

	u8  m_data1;
	u8  m_data2;
	u8  m_flag1;
	u8  m_flag2;
};
