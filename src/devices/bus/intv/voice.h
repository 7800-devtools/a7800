// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
#ifndef __INTV_VOICE_H
#define __INTV_VOICE_H

#include "slot.h"
#include "rom.h"
#include "sound/sp0256.h"


// ======================> intv_voice_device

class intv_voice_device : public intv_rom_device
{
public:
	// construction/destruction
	intv_voice_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// reading and writing
	// actual IntelliVoice access
	virtual DECLARE_READ16_MEMBER(read_speech) override;
	virtual DECLARE_WRITE16_MEMBER(write_speech) override;

	// passthru access
	virtual DECLARE_READ16_MEMBER(read_rom04) override { return m_subslot->read_rom04(space, offset, mem_mask); }
	virtual DECLARE_READ16_MEMBER(read_rom20) override { return m_subslot->read_rom20(space, offset, mem_mask); }
	virtual DECLARE_READ16_MEMBER(read_rom40) override { return m_subslot->read_rom40(space, offset, mem_mask); }
	virtual DECLARE_READ16_MEMBER(read_rom48) override { return m_subslot->read_rom48(space, offset, mem_mask); }
	virtual DECLARE_READ16_MEMBER(read_rom50) override { return m_subslot->read_rom50(space, offset, mem_mask); }
	virtual DECLARE_READ16_MEMBER(read_rom60) override { return m_subslot->read_rom60(space, offset, mem_mask); }
	virtual DECLARE_READ16_MEMBER(read_rom70) override { return m_subslot->read_rom70(space, offset, mem_mask); }
	virtual DECLARE_READ16_MEMBER(read_rom80) override;
	virtual DECLARE_READ16_MEMBER(read_rom90) override { return m_subslot->read_rom90(space, offset, mem_mask); }
	virtual DECLARE_READ16_MEMBER(read_roma0) override { return m_subslot->read_roma0(space, offset, mem_mask); }
	virtual DECLARE_READ16_MEMBER(read_romb0) override { return m_subslot->read_romb0(space, offset, mem_mask); }
	virtual DECLARE_READ16_MEMBER(read_romc0) override { return m_subslot->read_romc0(space, offset, mem_mask); }
	virtual DECLARE_READ16_MEMBER(read_romd0) override;
	virtual DECLARE_READ16_MEMBER(read_rome0) override { return m_subslot->read_rome0(space, offset, mem_mask); }
	virtual DECLARE_READ16_MEMBER(read_romf0) override { return m_subslot->read_romf0(space, offset, mem_mask); }

	// RAM passthru write
	virtual DECLARE_WRITE16_MEMBER(write_88) override { if (m_ram88_enabled) m_subslot->write_ram(space, offset, data, mem_mask); }
	virtual DECLARE_WRITE16_MEMBER(write_d0) override { if (m_ramd0_enabled) m_subslot->write_ram(space, offset, data, mem_mask); }
	virtual DECLARE_READ16_MEMBER(read_ram) override { return m_subslot->read_ram(space, offset, mem_mask); }
	virtual DECLARE_WRITE16_MEMBER(write_ram) override { m_subslot->write_ram(space, offset, data, mem_mask); }

	virtual void late_subslot_setup() override;

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

private:
	required_device<sp0256_device> m_speech;
	required_device<intv_cart_slot_device> m_subslot;
	bool m_ramd0_enabled, m_ram88_enabled;
};


// device type definition
DECLARE_DEVICE_TYPE(INTV_ROM_VOICE, intv_voice_device)

#endif // MAME_BUS_INTV_SLOT_H
