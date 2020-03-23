// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
#ifndef MAME_BUS_IQ151_GRAFIK_H
#define MAME_BUS_IQ151_GRAFIK_H

#pragma once

#include "iq151.h"
#include "machine/i8255.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> iq151_grafik_device

class iq151_grafik_device :
		public device_t,
		public device_iq151cart_interface
{
public:
	// construction/destruction
	iq151_grafik_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;

	// iq151cart_interface overrides
	virtual void io_read(offs_t offset, uint8_t &data) override;
	virtual void io_write(offs_t offset, uint8_t data) override;
	virtual void video_update(bitmap_ind16 &bitmap, const rectangle &cliprect) override;

private:
	// ppi8255 callback
	DECLARE_WRITE8_MEMBER(x_write);
	DECLARE_WRITE8_MEMBER(y_write);
	DECLARE_WRITE8_MEMBER(control_w);

	required_device<i8255_device> m_ppi8255;

	uint8_t       m_posx;     // horizontal position
	uint8_t       m_posy;     // vertical position
	uint8_t       m_all;      // 0: bit mode 1: byte mode
	uint8_t       m_pen;
	uint8_t       m_fast;
	uint8_t       m_ev;       // enable video out
	uint8_t       m_ex;
	uint8_t       m_sel;      // enable vram access
	uint8_t       m_videoram[0x4000];
};


// device type definition
extern const device_type IQ151_GRAFIK;
DECLARE_DEVICE_TYPE(IQ151_GRAFIK, iq151_grafik_device)

#endif // MAME_BUS_IQ151_GRAFIK_H
