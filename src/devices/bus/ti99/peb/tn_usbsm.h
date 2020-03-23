// license:LGPL-2.1+
// copyright-holders:Michael Zapf
/****************************************************************************

    Nouspikel USB / SmartMedia interface card
    See tn_usbsm.c for documentation

    Michael Zapf

    February 2012: Rewritten as class

*****************************************************************************/

#ifndef MAME_BUS_TI99_PEB_TN_USBSM_H
#define MAME_BUS_TI99_PEB_TN_USBSM_H

#pragma once

#include "peribox.h"
#include "machine/smartmed.h"
#include "machine/strata.h"
#include "machine/ram.h"

namespace bus { namespace ti99 { namespace peb {

class nouspikel_usb_smartmedia_device : public device_t, public device_ti99_peribox_card_interface
{
public:
	nouspikel_usb_smartmedia_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	DECLARE_READ8Z_MEMBER(readz) override;
	DECLARE_WRITE8_MEMBER(write) override;

	DECLARE_READ8Z_MEMBER(crureadz) override;
	DECLARE_WRITE8_MEMBER(cruwrite) override;

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual ioport_constructor device_input_ports() const override;

private:

	int         m_feeprom_page;
	int         m_sram_page;
	int         m_cru_register;
	bool        m_tms9995_mode;

	bool        m_enable_io;
	bool        m_enable_int;
	bool        m_enable_sm;
	bool        m_write_flash;

	uint16_t      m_input_latch;
	uint16_t      m_output_latch;

	required_device<ram_device> m_ram_lb;
	required_device<ram_device> m_ram_hb;
	required_device<smartmedia_image_device> m_smartmedia;
	required_device<strataflash_device> m_flash;
};
} } } // end namespace bus::ti99::peb

DECLARE_DEVICE_TYPE_NS(TI99_USBSM, bus::ti99::peb, nouspikel_usb_smartmedia_device)

#endif // MAME_BUS_TI99_PEB_TN_USBSM_H
