// license:BSD-3-Clause
// copyright-holders:Luca Elia

#include "machine/gen_latch.h"
#include "screen.h"

class suna16_state : public driver_device
{
public:
	suna16_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this,"maincpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_screen(*this, "screen"),
		m_palette(*this, "palette"),
		m_soundlatch(*this, "soundlatch"),
		m_spriteram(*this, "spriteram"),
		m_spriteram2(*this, "spriteram2"),
		m_bank1(*this, "bank1"),
		m_bank2(*this, "bank2")


	{ }

	required_device<cpu_device> m_maincpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
	required_device<generic_latch_8_device> m_soundlatch;

	required_shared_ptr<uint16_t> m_spriteram;
	optional_shared_ptr<uint16_t> m_spriteram2;

	optional_memory_bank m_bank1;
	optional_memory_bank m_bank2;


	std::unique_ptr<uint16_t[]> m_paletteram;
	int m_color_bank;
	uint8_t m_prot;

	// common
	DECLARE_WRITE16_MEMBER(soundlatch_w);
	DECLARE_READ16_MEMBER(paletteram_r);
	DECLARE_WRITE16_MEMBER(paletteram_w);
	DECLARE_WRITE16_MEMBER(flipscreen_w);

	// bestbest specific
	DECLARE_WRITE16_MEMBER(bestbest_flipscreen_w);
	DECLARE_WRITE16_MEMBER(bestbest_coin_w);
	DECLARE_READ8_MEMBER(bestbest_prot_r);
	DECLARE_WRITE8_MEMBER(bestbest_prot_w);
	DECLARE_WRITE8_MEMBER(bestbest_ay8910_port_a_w);

	// bssoccer specific
	DECLARE_WRITE16_MEMBER(bssoccer_leds_w);
	DECLARE_WRITE8_MEMBER(bssoccer_pcm_1_bankswitch_w);
	DECLARE_WRITE8_MEMBER(bssoccer_pcm_2_bankswitch_w);

	// uballoon specific
	DECLARE_WRITE16_MEMBER(uballoon_leds_w);
	DECLARE_WRITE8_MEMBER(uballoon_pcm_1_bankswitch_w);
	DECLARE_READ8_MEMBER(uballoon_prot_r);
	DECLARE_WRITE8_MEMBER(uballoon_prot_w);

	TIMER_DEVICE_CALLBACK_MEMBER(bssoccer_interrupt);

	DECLARE_DRIVER_INIT(uballoon);
	virtual void video_start() override;
	DECLARE_MACHINE_START(bestbest);
	DECLARE_MACHINE_START(bssoccer);
	DECLARE_MACHINE_START(uballoon);
	DECLARE_MACHINE_RESET(uballoon);

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_bestbest(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void draw_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect, uint16_t *sprites, int gfx);
};
