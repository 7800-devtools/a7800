// license:BSD-3-Clause
// copyright-holders:David Graves, R. Belmont

#include "machine/74157.h"
#include "machine/eepromser.h"
#include "sound/okim6295.h"
#include "sound/msm5205.h"
#include "video/excellent_spr.h"

class gcpinbal_state : public driver_device
{
public:
	enum
	{
		TIMER_GCPINBAL_INTERRUPT1
	};

	gcpinbal_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_eeprom(*this, "eeprom"),
		m_oki(*this, "oki"),
		m_msm(*this, "msm"),
		m_adpcm_select(*this, "adpcm_select"),
		m_tilemapram(*this, "tilemapram"),
		m_d80010_ram(*this, "d80010"),
		m_d80060_ram(*this, "d80060"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_sprgen(*this, "spritegen")
	{ }

	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<eeprom_serial_93cxx_device> m_eeprom;
	required_device<okim6295_device> m_oki;
	required_device<msm6585_device> m_msm;
	required_device<hct157_device> m_adpcm_select;

	/* memory pointers */
	required_shared_ptr<uint16_t> m_tilemapram;
	required_shared_ptr<uint16_t> m_d80010_ram;
	required_shared_ptr<uint16_t> m_d80060_ram;

	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;

	emu_timer *m_int1_timer;

	/* video-related */
	tilemap_t     *m_tilemap[3];
	uint16_t      m_scrollx[3];
	uint16_t      m_scrolly[3];
	uint16_t      m_bg0_gfxset;
	uint16_t      m_bg1_gfxset;
#ifdef MAME_DEBUG
	uint8_t       m_dislayer[4];
#endif

	/* sound-related */
	uint32_t      m_msm_start;
	uint32_t      m_msm_end;
	uint32_t      m_msm_bank;
	uint32_t      m_adpcm_start;
	uint32_t      m_adpcm_end;
	uint32_t      m_adpcm_idle;
	uint8_t       m_adpcm_trigger;

	DECLARE_WRITE16_MEMBER(d80010_w);
	DECLARE_WRITE8_MEMBER(d80040_w);
	DECLARE_WRITE16_MEMBER(d80060_w);
	DECLARE_WRITE8_MEMBER(bank_w);
	DECLARE_WRITE8_MEMBER(eeprom_w);
	DECLARE_WRITE8_MEMBER(es8712_ack_w);
	DECLARE_WRITE8_MEMBER(es8712_w);
	DECLARE_READ16_MEMBER(gcpinbal_tilemaps_word_r);
	DECLARE_WRITE16_MEMBER(gcpinbal_tilemaps_word_w);
	TILE_GET_INFO_MEMBER(get_bg0_tile_info);
	TILE_GET_INFO_MEMBER(get_bg1_tile_info);
	TILE_GET_INFO_MEMBER(get_fg_tile_info);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	uint32_t screen_update_gcpinbal(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	INTERRUPT_GEN_MEMBER(gcpinbal_interrupt);
	void gcpinbal_core_vh_start(  );
	DECLARE_WRITE_LINE_MEMBER(gcp_adpcm_int);
	required_device<excellent_spr_device> m_sprgen;

protected:
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
};
