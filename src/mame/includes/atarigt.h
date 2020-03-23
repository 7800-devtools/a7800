// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/*************************************************************************

    Atari GT hardware

*************************************************************************/

#include "audio/cage.h"
#include "machine/atarigen.h"
#include "video/atarirle.h"

#define CRAM_ENTRIES        0x4000
#define TRAM_ENTRIES        0x4000
#define MRAM_ENTRIES        0x8000

#define ADDRSEQ_COUNT   4

class atarigt_state : public atarigen_state
{
public:
	atarigt_state(const machine_config &mconfig, device_type type, const char *tag)
		: atarigen_state(mconfig, type, tag),
			m_colorram(*this, "colorram", 32),
			m_playfield_tilemap(*this, "playfield"),
			m_alpha_tilemap(*this, "alpha"),
			m_rle(*this, "rle"),
			m_mo_command(*this, "mo_command"),
			m_cage(*this, "cage") { }

	uint8_t           m_is_primrage;
	required_shared_ptr<uint16_t> m_colorram;

	required_device<tilemap_device> m_playfield_tilemap;
	required_device<tilemap_device> m_alpha_tilemap;
	required_device<atari_rle_objects_device> m_rle;

	bitmap_ind16    m_pf_bitmap;
	bitmap_ind16    m_an_bitmap;

	uint8_t           m_playfield_tile_bank;
	uint8_t           m_playfield_color_bank;
	uint16_t          m_playfield_xscroll;
	uint16_t          m_playfield_yscroll;

	uint32_t          m_tram_checksum;

	uint32_t          m_expanded_mram[MRAM_ENTRIES * 3];

	required_shared_ptr<uint32_t> m_mo_command;
	optional_device<atari_cage_device> m_cage;

	void            (atarigt_state::*m_protection_w)(address_space &space, offs_t offset, uint16_t data);
	void            (atarigt_state::*m_protection_r)(address_space &space, offs_t offset, uint16_t *data);

	bool            m_ignore_writes;
	offs_t          m_protaddr[ADDRSEQ_COUNT];
	uint8_t           m_protmode;
	uint16_t          m_protresult;
	uint8_t           m_protdata[0x800];

	virtual void update_interrupts() override;
	virtual void scanline_update(screen_device &screen, int scanline) override;
	DECLARE_READ32_MEMBER(special_port2_r);
	DECLARE_READ32_MEMBER(special_port3_r);
	DECLARE_READ32_MEMBER(analog_port0_r);
	DECLARE_READ32_MEMBER(analog_port1_r);
	DECLARE_WRITE32_MEMBER(latch_w);
	DECLARE_WRITE32_MEMBER(mo_command_w);
	DECLARE_WRITE32_MEMBER(led_w);
	DECLARE_READ32_MEMBER(sound_data_r);
	DECLARE_WRITE32_MEMBER(sound_data_w);
	DECLARE_READ32_MEMBER(colorram_protection_r);
	DECLARE_WRITE32_MEMBER(colorram_protection_w);
	DECLARE_WRITE32_MEMBER(tmek_pf_w);

	DECLARE_WRITE8_MEMBER(cage_irq_callback);

	void atarigt_colorram_w(offs_t address, uint16_t data, uint16_t mem_mask);
	uint16_t atarigt_colorram_r(offs_t address);
	DECLARE_DRIVER_INIT(primrage);
	DECLARE_DRIVER_INIT(tmek);
	TILE_GET_INFO_MEMBER(get_alpha_tile_info);
	TILE_GET_INFO_MEMBER(get_playfield_tile_info);
	TILEMAP_MAPPER_MEMBER(atarigt_playfield_scan);
	DECLARE_MACHINE_START(atarigt);
	DECLARE_MACHINE_RESET(atarigt);
	DECLARE_VIDEO_START(atarigt);
	uint32_t screen_update_atarigt(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
private:
	void tmek_update_mode(offs_t offset);
	void tmek_protection_w(address_space &space, offs_t offset, uint16_t data);
	void tmek_protection_r(address_space &space, offs_t offset, uint16_t *data);
	void primrage_update_mode(offs_t offset);
	void primrage_protection_w(address_space &space, offs_t offset, uint16_t data);
	void primrage_protection_r(address_space &space, offs_t offset, uint16_t *data);
	void compute_fake_pots(int *pots);
};
