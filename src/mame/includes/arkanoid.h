// license:BSD-3-Clause
// copyright-holders:Brad Oliver,Stephane Humbert
#include "machine/taito68705interface.h"

/* This it the best way to allow game specific kludges until the system is fully understood */
enum {
	ARKUNK = 0,  /* unknown bootlegs for inclusion of possible new sets */
	ARKANGC,
	ARKANGC2,
	BLOCK2,
	ARKBLOCK,
	ARKBLOC2,
	ARKGCBL,
	PADDLE2
};

class arkanoid_state : public driver_device
{
public:


	arkanoid_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_videoram(*this, "videoram")
		, m_spriteram(*this, "spriteram")
		, m_protram(*this, "protram")
		, m_muxports(*this, "P%u", 1)
		, m_maincpu(*this, "maincpu")
		, m_mcuintf(*this, "mcu")
		, m_gfxdecode(*this, "gfxdecode")
		, m_palette(*this, "palette")
	{
	}

	/* memory pointers */
	required_shared_ptr<uint8_t> m_videoram;
	optional_shared_ptr<uint8_t> m_spriteram;
	optional_shared_ptr<uint8_t> m_protram;

	/* video-related */
	tilemap_t  *m_bg_tilemap;
	uint8_t    m_gfxbank;
	uint8_t    m_palettebank;

	/* input-related */
	uint8_t    m_paddle_select;   // selected by d008 bit 2

	/* bootleg related */
	int      m_bootleg_id;
	uint8_t    m_bootleg_cmd;

	/* hexaa */
	uint8_t m_hexaa_from_main;
	uint8_t m_hexaa_from_sub;

	/* devices */
	optional_ioport_array<2> m_muxports;
	required_device<cpu_device> m_maincpu;
	optional_device<arkanoid_mcu_device_base> m_mcuintf;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;


	DECLARE_READ8_MEMBER(arkanoid_bootleg_f000_r);
	DECLARE_READ8_MEMBER(arkanoid_bootleg_f002_r);
	DECLARE_WRITE8_MEMBER(arkanoid_bootleg_d018_w);
	DECLARE_READ8_MEMBER(arkanoid_bootleg_d008_r);
	DECLARE_WRITE8_MEMBER(arkanoid_videoram_w);
	DECLARE_WRITE8_MEMBER(arkanoid_d008_w);
	DECLARE_WRITE8_MEMBER(tetrsark_d008_w);
	DECLARE_WRITE8_MEMBER(brixian_d008_w);
	DECLARE_WRITE8_MEMBER(hexa_d008_w);
	DECLARE_READ8_MEMBER(hexaa_f000_r);
	DECLARE_WRITE8_MEMBER(hexaa_f000_w);
	DECLARE_WRITE8_MEMBER(hexaa_sub_80_w);
	DECLARE_READ8_MEMBER(hexaa_sub_90_r);
	DECLARE_CUSTOM_INPUT_MEMBER(arkanoid_semaphore_input_r);
	DECLARE_CUSTOM_INPUT_MEMBER(arkanoid_input_mux);
	DECLARE_DRIVER_INIT(block2);
	DECLARE_DRIVER_INIT(arkblock);
	DECLARE_DRIVER_INIT(hexa);
	DECLARE_DRIVER_INIT(hexaa);
	DECLARE_DRIVER_INIT(paddle2);
	DECLARE_DRIVER_INIT(tetrsark);
	DECLARE_DRIVER_INIT(arkgcbl);
	DECLARE_DRIVER_INIT(arkangc2);
	DECLARE_DRIVER_INIT(arkbloc2);
	DECLARE_DRIVER_INIT(arkangc);
	DECLARE_DRIVER_INIT(brixian);
	TILE_GET_INFO_MEMBER(get_bg_tile_info);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	uint32_t screen_update_arkanoid(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_hexa(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect );
	void arkanoid_bootleg_init(  );

};
