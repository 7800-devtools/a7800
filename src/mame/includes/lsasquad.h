// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria
#include "machine/taito68705interface.h"

class lsasquad_state : public driver_device
{
public:
	lsasquad_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_videoram(*this, "videoram"),
		m_scrollram(*this, "scrollram"),
		m_spriteram(*this, "spriteram"),
		m_priority_prom(*this, "prio_prom"),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_bmcu(*this, "bmcu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette") { }

	/* memory pointers */
	required_shared_ptr<uint8_t> m_videoram;
	required_shared_ptr<uint8_t> m_scrollram;
	required_shared_ptr<uint8_t> m_spriteram;
	required_region_ptr<uint8_t> m_priority_prom;

	/* misc */
	int m_sound_pending;
	int m_sound_nmi_enable;
	int m_pending_nmi;
	int m_sound_cmd;
	int m_sound_result;


	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	optional_device<taito68705_mcu_device> m_bmcu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;

	DECLARE_WRITE8_MEMBER(lsasquad_bankswitch_w);
	DECLARE_WRITE8_MEMBER(lsasquad_sh_nmi_disable_w);
	DECLARE_WRITE8_MEMBER(lsasquad_sh_nmi_enable_w);
	DECLARE_WRITE8_MEMBER(lsasquad_sound_command_w);
	DECLARE_READ8_MEMBER(lsasquad_sh_sound_command_r);
	DECLARE_WRITE8_MEMBER(lsasquad_sh_result_w);
	DECLARE_READ8_MEMBER(lsasquad_sound_result_r);
	DECLARE_READ8_MEMBER(lsasquad_sound_status_r);
	DECLARE_READ8_MEMBER(daikaiju_sh_sound_command_r);
	DECLARE_READ8_MEMBER(daikaiju_sound_status_r);

	DECLARE_READ8_MEMBER(lsasquad_mcu_status_r);
	DECLARE_READ8_MEMBER(daikaiju_mcu_status_r);
	DECLARE_WRITE8_MEMBER(unk);
	DECLARE_MACHINE_START(lsasquad);
	DECLARE_MACHINE_RESET(lsasquad);
	uint32_t screen_update_lsasquad(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_daikaiju(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	TIMER_CALLBACK_MEMBER(nmi_callback);
	void draw_layer( bitmap_ind16 &bitmap, const rectangle &cliprect, uint8_t *scrollram );
	int draw_layer_daikaiju( bitmap_ind16 &bitmap, const rectangle &cliprect, int offs, int  * previd, int type );
	void drawbg( bitmap_ind16 &bitmap, const rectangle &cliprect, int type );
	void draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect, uint8_t priority );
};
