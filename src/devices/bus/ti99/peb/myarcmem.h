// license:LGPL-2.1+
// copyright-holders:Michael Zapf
/****************************************************************************

    TI-99 Myarc memory expansion
    See myarcmem.c for documentation

    Michael Zapf, September 2010
    February 2012: Rewritten as class

*****************************************************************************/

#ifndef MAME_BUS_TI99_PEB_MYARCMEM_H
#define MAME_BUS_TI99_PEB_MYARCMEM_H

#pragma once

#include "peribox.h"
#include "machine/ram.h"

namespace bus { namespace ti99 { namespace peb {

class myarc_memory_expansion_device : public device_t, public device_ti99_peribox_card_interface
{
public:
	myarc_memory_expansion_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	DECLARE_READ8Z_MEMBER(readz) override;
	DECLARE_WRITE8_MEMBER(write) override;

	DECLARE_READ8Z_MEMBER(crureadz) override;
	DECLARE_WRITE8_MEMBER(cruwrite) override;

protected:
	void device_start() override;
	void device_reset() override;
	const tiny_rom_entry *device_rom_region() const override;
	ioport_constructor device_input_ports() const override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	int     get_base(int offset);
	required_device<ram_device> m_ram;
	uint8_t*  m_dsrrom;
	int     m_bank;
	int     m_size;
};

} } } // end namespace bus::ti99::peb

DECLARE_DEVICE_TYPE_NS(TI99_MYARCMEM, bus::ti99::peb, myarc_memory_expansion_device)

#endif // MAME_BUS_TI99_PEB_MYARCMEM_H
