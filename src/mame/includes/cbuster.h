// license:BSD-3-Clause
// copyright-holders:Bryan McPhail
/*************************************************************************

    Crude Buster

*************************************************************************/

#include "machine/gen_latch.h"
#include "video/decospr.h"
#include "video/deco16ic.h"

class cbuster_state : public driver_device
{
public:
	cbuster_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_ram(*this, "ram"),
		m_pf1_rowscroll(*this, "pf1_rowscroll"),
		m_pf2_rowscroll(*this, "pf2_rowscroll"),
		m_pf3_rowscroll(*this, "pf3_rowscroll"),
		m_pf4_rowscroll(*this, "pf4_rowscroll"),
		m_spriteram16(*this, "spriteram16"),
		m_paletteram(*this, "palette"),
		m_paletteram_ext(*this, "palette_ext"),
		m_sprgen(*this, "spritegen"),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_deco_tilegen1(*this, "tilegen1"),
		m_deco_tilegen2(*this, "tilegen2"),
		m_palette(*this, "palette"),
		m_soundlatch(*this, "soundlatch")
	{ }

	/* memory pointers */
	required_shared_ptr<uint16_t> m_ram;
	required_shared_ptr<uint16_t> m_pf1_rowscroll;
	required_shared_ptr<uint16_t> m_pf2_rowscroll;
	required_shared_ptr<uint16_t> m_pf3_rowscroll;
	required_shared_ptr<uint16_t> m_pf4_rowscroll;
	required_shared_ptr<uint16_t> m_spriteram16;
	required_shared_ptr<uint16_t> m_paletteram;
	required_shared_ptr<uint16_t> m_paletteram_ext;
	optional_device<decospr_device> m_sprgen;

	uint16_t    m_spriteram16_buffer[0x400];

	/* misc */
	uint16_t    m_prot;
	int       m_pri;

	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	required_device<deco16ic_device> m_deco_tilegen1;
	required_device<deco16ic_device> m_deco_tilegen2;
	required_device<palette_device> m_palette;
	required_device<generic_latch_8_device> m_soundlatch;

	DECLARE_WRITE16_MEMBER(twocrude_control_w);
	DECLARE_READ16_MEMBER(twocrude_control_r);
	DECLARE_DRIVER_INIT(twocrude);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	uint32_t screen_update_twocrude(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	DECO16IC_BANK_CB_MEMBER(bank_callback);
	DECLARE_WRITE16_MEMBER(cbuster_palette_w);
	DECLARE_WRITE16_MEMBER(cbuster_palette_ext_w);
	void update_palette(int offset);
};
