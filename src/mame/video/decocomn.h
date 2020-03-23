// license:BSD-3-Clause
// copyright-holders:Bryan McPhail
/*************************************************************************

    decocomn.h

**************************************************************************/
#ifndef MAME_VIDEO_DECOCOMN_H
#define MAME_VIDEO_DECOCOMN_H

#pragma once


/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/


class decocomn_device : public device_t,
						public device_video_interface
{
public:
	decocomn_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration
	static void static_set_palette_tag(device_t &device, const char *tag);

	DECLARE_WRITE16_MEMBER( nonbuffered_palette_w );
	DECLARE_WRITE16_MEMBER( buffered_palette_w );
	DECLARE_WRITE16_MEMBER( palette_dma_w );
	DECLARE_WRITE16_MEMBER( priority_w );
	DECLARE_READ16_MEMBER( priority_r );
	DECLARE_READ16_MEMBER( d_71_r );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	// internal state
	std::unique_ptr<uint8_t[]> m_dirty_palette;
	uint16_t m_priority;
	required_device<palette_device> m_palette;
	required_shared_ptr<uint16_t> m_generic_paletteram_16;
};

DECLARE_DEVICE_TYPE(DECOCOMN, decocomn_device)



/***************************************************************************
    DEVICE CONFIGURATION MACROS
***************************************************************************/

#define MCFG_DECOCOMN_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, DECOCOMN, 0)

#define MCFG_DECOCOMN_PALETTE(_palette_tag) \
	decocomn_device::static_set_palette_tag(*device, "^" _palette_tag);

#endif // MAME_VIDEO_DECOCOMN_H
