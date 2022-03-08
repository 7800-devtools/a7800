// license:BSD-3-Clause
// copyright-holders:Mike Saarna

#ifndef MAME_BUS_A7800_BANKSET_H
#define MAME_BUS_A7800_BANKSET_H

#pragma once

#include "a78_slot.h"
#include "rom.h"
#include "sound/pokey.h"


// ======================> a78_bankset_sg_device

class a78_bankset_sg_device : public a78_rom_sg_device
{
public:
	// construction/destruction
	a78_bankset_sg_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_40xx) override;
	virtual DECLARE_WRITE8_MEMBER(write_40xx) override;

protected:
	a78_bankset_sg_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

};

// ======================> a78_bankset_sg_p450_device

class a78_bankset_sg_p450_device : public a78_rom_sg_device
{
public:
	// construction/destruction
	a78_bankset_sg_p450_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_40xx) override;
	virtual DECLARE_WRITE8_MEMBER(write_40xx) override;

       virtual DECLARE_READ8_MEMBER(read_04xx) override { if (offset >= 0x50 && offset < 0x60) return m_pokey450->read(space, offset & 0x0f); else return 0xff; }
        virtual DECLARE_WRITE8_MEMBER(write_04xx) override { if (offset >= 0x50 && offset < 0x60) m_pokey450->write(space, offset & 0x0f, data); }

protected:
	a78_bankset_sg_p450_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);
        virtual void device_add_mconfig(machine_config &config) override;
        required_device<pokey_device> m_pokey450;

};


// ======================> a78_bankset_sg_bankram_device

class a78_bankset_sg_bankram_device : public a78_rom_sg_device
{
public:
	// construction/destruction
	a78_bankset_sg_bankram_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_40xx) override;
	virtual DECLARE_WRITE8_MEMBER(write_40xx) override;

protected:
	a78_bankset_sg_bankram_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

};

// ======================> a78_bankset_sg_bankram_device

class a78_bankset_sg_bankram_p450_device : public a78_rom_sg_device
{
public:
	// construction/destruction
	a78_bankset_sg_bankram_p450_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_40xx) override;
	virtual DECLARE_WRITE8_MEMBER(write_40xx) override;

       virtual DECLARE_READ8_MEMBER(read_04xx) override { if (offset >= 0x50 && offset < 0x60) return m_pokey450->read(space, offset & 0x0f); else return 0xff; }
        virtual DECLARE_WRITE8_MEMBER(write_04xx) override { if (offset >= 0x50 && offset < 0x60) m_pokey450->write(space, offset & 0x0f, data); }


protected:
	a78_bankset_sg_bankram_p450_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);
        virtual void device_add_mconfig(machine_config &config) override;
        required_device<pokey_device> m_pokey450;

};


// ======================> a78_bankset_rom_device

class a78_bankset_rom_device : public a78_rom_device
{
public:
	// construction/destruction
	a78_bankset_rom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_40xx) override;

protected:
	a78_bankset_rom_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

};

// ======================> a78_bankset_rom_52k_device

class a78_bankset_rom_52k_device : public a78_rom_device
{
public:
	// construction/destruction
	a78_bankset_rom_52k_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_40xx) override;
	virtual DECLARE_READ8_MEMBER(read_30xx) override;

protected:
	a78_bankset_rom_52k_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

};

// ======================> a78_bankset_rom_52k_p4000_device

class a78_bankset_rom_52k_p4000_device : public a78_rom_device
{
public:
	// construction/destruction
	a78_bankset_rom_52k_p4000_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_40xx) override;
	virtual DECLARE_READ8_MEMBER(read_30xx) override;
	virtual DECLARE_WRITE8_MEMBER(write_40xx) override { if (offset < 0x4000) m_pokey4000->write(space, offset & 0x0f, data); }

protected:
	a78_bankset_rom_52k_p4000_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_add_mconfig(machine_config &config) override;
	required_device<pokey_device> m_pokey4000;

};


// ======================> a78_bankset_rom_pokey450_device

class a78_bankset_rom_p450_device : public a78_rom_device
{
public:
	// construction/destruction
	a78_bankset_rom_p450_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_40xx) override;

	virtual DECLARE_READ8_MEMBER(read_04xx) override { if (offset >= 0x50 && offset < 0x60) return m_pokey450->read(space, offset & 0x0f); else return 0xff; }
	virtual DECLARE_WRITE8_MEMBER(write_04xx) override { if (offset >= 0x50 && offset < 0x60) m_pokey450->write(space, offset & 0x0f, data); }

protected:
	a78_bankset_rom_p450_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_add_mconfig(machine_config &config) override;
	required_device<pokey_device> m_pokey450;

};


// ======================> a78_bankset_rom_pokey4000_device

class a78_bankset_rom_p4000_device : public a78_rom_device
{
public:
	// construction/destruction
	a78_bankset_rom_p4000_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_40xx) override;
	virtual DECLARE_WRITE8_MEMBER(write_40xx) override { if (offset >= 0 && offset < 0x10) m_pokey4000->write(space, offset & 0x0f, data); }

protected:
	a78_bankset_rom_p4000_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_add_mconfig(machine_config &config) override;
	required_device<pokey_device> m_pokey4000;

};


// ======================> a78_bankset_rom_device

class a78_bankset_bankram_device : public a78_rom_device
{
public:
	// construction/destruction
	a78_bankset_bankram_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_40xx) override;
	virtual DECLARE_WRITE8_MEMBER(write_40xx) override;


protected:
	a78_bankset_bankram_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

};

// ======================> a78_bankset_bankram_p450_device

class a78_bankset_bankram_p450_device : public a78_rom_device
{
public:
	// construction/destruction
	a78_bankset_bankram_p450_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_40xx) override;
	virtual DECLARE_WRITE8_MEMBER(write_40xx) override;

       virtual DECLARE_READ8_MEMBER(read_04xx) override { if (offset >= 0x50 && offset < 0x60) return m_pokey450->read(space, offset & 0x0f); else return 0xff; }
        virtual DECLARE_WRITE8_MEMBER(write_04xx) override { if (offset >= 0x50 && offset < 0x60) m_pokey450->write(space, offset & 0x0f, data); }




protected:
	a78_bankset_bankram_p450_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

        virtual void device_add_mconfig(machine_config &config) override;
        required_device<pokey_device> m_pokey450;

};




// device type definitions
DECLARE_DEVICE_TYPE(A78_ROM_BANKSET_ROM, a78_bankset_rom_device)
DECLARE_DEVICE_TYPE(A78_ROM_BANKSET_SG, a78_bankset_sg_device)
DECLARE_DEVICE_TYPE(A78_ROM_BANKSET_SG_BANKRAM, a78_bankset_sg_bankram_device)
DECLARE_DEVICE_TYPE(A78_ROM_BANKSET_BANKRAM, a78_bankset_bankram_device)
DECLARE_DEVICE_TYPE(A78_ROM_BANKSET_ROM_52K, a78_bankset_rom_52k_device)

DECLARE_DEVICE_TYPE(A78_ROM_BANKSET_ROM_POK450, a78_bankset_rom_p450_device)
DECLARE_DEVICE_TYPE(A78_ROM_BANKSET_ROM_POK4000, a78_bankset_rom_p4000_device)
DECLARE_DEVICE_TYPE(A78_ROM_BANKSET_SG_POK450, a78_bankset_sg_p450_device)
DECLARE_DEVICE_TYPE(A78_ROM_BANKSET_SG_BANKRAM_POK450, a78_bankset_sg_bankram_p450_device)
DECLARE_DEVICE_TYPE(A78_ROM_BANKSET_BANKRAM_POK450, a78_bankset_bankram_p450_device)
DECLARE_DEVICE_TYPE(A78_ROM_BANKSET_ROM_52K_POK4000, a78_bankset_rom_52k_p4000_device)

#endif // MAME_BUS_A7800_BANKSET_H
