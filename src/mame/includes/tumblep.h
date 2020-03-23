// license:BSD-3-Clause
// copyright-holders:David Haywood,Bryan McPhail
/*************************************************************************

    Tumble Pop

*************************************************************************/

#include "machine/gen_latch.h"
#include "video/decospr.h"
#include "video/deco16ic.h"

class tumblep_state : public driver_device
{
public:
	tumblep_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_spriteram(*this, "spriteram"),
		m_pf1_rowscroll(*this, "pf1_rowscroll"),
		m_pf2_rowscroll(*this, "pf2_rowscroll"),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_deco_tilegen1(*this, "tilegen1"),
		m_sprgen(*this, "spritegen"),
		m_soundlatch(*this, "soundlatch")
	{ }

	/* memory pointers */
	required_shared_ptr<uint16_t> m_spriteram;
	required_shared_ptr<uint16_t> m_pf1_rowscroll;
	required_shared_ptr<uint16_t> m_pf2_rowscroll;

	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	required_device<deco16ic_device> m_deco_tilegen1;
	required_device<decospr_device> m_sprgen;
	required_device<generic_latch_8_device> m_soundlatch;

	DECLARE_WRITE16_MEMBER(tumblep_sound_w);
	DECLARE_READ16_MEMBER(tumblepop_controls_r);
	DECLARE_DRIVER_INIT(tumblep);
	virtual void machine_start() override;
	uint32_t screen_update_tumblep(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
};
