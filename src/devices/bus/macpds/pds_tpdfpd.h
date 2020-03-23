// license:BSD-3-Clause
// copyright-holders:R. Belmont
#ifndef MAME_BUS_PDS_TPDFPD_H
#define MAME_BUS_PDS_TPDFPD_H

#pragma once

#include "macpds.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> macpds_sedisplay_device

class macpds_sedisplay_device :
		public device_t,
		public device_video_interface,
		public device_macpds_card_interface
{
public:
	// construction/destruction
	macpds_sedisplay_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	macpds_sedisplay_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

private:
	DECLARE_READ16_MEMBER(sedisplay_r);
	DECLARE_WRITE16_MEMBER(sedisplay_w);
	DECLARE_READ16_MEMBER(ramdac_r);
	DECLARE_WRITE16_MEMBER(ramdac_w);

	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	std::unique_ptr<uint8_t[]> m_vram;
	uint32_t m_vbl_disable;
	uint32_t m_palette[256], m_colors[3], m_count, m_clutoffs;
	emu_timer *m_timer;
	const std::string m_assembled_tag;
};


// device type definition
DECLARE_DEVICE_TYPE(PDS_SEDISPLAY, macpds_sedisplay_device)

#endif // MAME_BUS_PDS_TPDFPD_H
