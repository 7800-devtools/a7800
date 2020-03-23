// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Wang PC PM-001B Medium-Resolution Video Controller emulation

**********************************************************************/

#ifndef MAME_BUS_WANGPC_MVC_H
#define MAME_BUS_WANGPC_MVC_H

#pragma once

#include "wangpc.h"
#include "video/mc6845.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> wangpc_mvc_device

class wangpc_mvc_device : public device_t,
							public device_wangpcbus_card_interface
{
public:
	// construction/destruction
	wangpc_mvc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;

	// device_wangpcbus_card_interface overrides
	virtual uint16_t wangpcbus_mrdc_r(address_space &space, offs_t offset, uint16_t mem_mask) override;
	virtual void wangpcbus_amwc_w(address_space &space, offs_t offset, uint16_t mem_mask, uint16_t data) override;
	virtual uint16_t wangpcbus_iorc_r(address_space &space, offs_t offset, uint16_t mem_mask) override;
	virtual void wangpcbus_aiowc_w(address_space &space, offs_t offset, uint16_t mem_mask, uint16_t data) override;

private:
	MC6845_UPDATE_ROW( crtc_update_row );
	DECLARE_WRITE_LINE_MEMBER( vsync_w );

	inline void set_irq(int state);

	required_device<mc6845_device> m_crtc;
	optional_shared_ptr<uint16_t> m_video_ram;
	optional_shared_ptr<uint16_t> m_char_ram;
	optional_shared_ptr<uint16_t> m_bitmap_ram;

	uint8_t m_option;
	int m_irq;
};


// device type definition
DECLARE_DEVICE_TYPE(WANGPC_MVC, wangpc_mvc_device)

#endif // MAME_BUS_WANGPC_MVC_H
