// license:BSD-3-Clause
// copyright-holders:smf
#ifndef MAME_MACHINE_ATAFLASH_H
#define MAME_MACHINE_ATAFLASH_H

#pragma once

#include "pccard.h"
#include "machine/idehd.h"

extern const device_type ATA_FLASH_PCCARD;
DECLARE_DEVICE_TYPE(ATA_FLASH_PCCARD, ata_flash_pccard_device)

class ata_flash_pccard_device : public ide_hdd_device, public pccard_interface
{
public:
	ata_flash_pccard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ16_MEMBER(read_memory) override;
	virtual DECLARE_WRITE16_MEMBER(write_memory) override;
	virtual DECLARE_READ16_MEMBER(read_reg) override;
	virtual DECLARE_WRITE16_MEMBER(write_reg) override;

protected:
	ata_flash_pccard_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_reset() override;

	virtual attotime seek_time() override;

private:
	uint8_t m_cis[512];
	uint8_t m_configuration_option;
	uint8_t m_configuration_and_status;
	uint8_t m_pin_replacement;
};

DECLARE_DEVICE_TYPE(TAITO_PCCARD1, taito_pccard1_device)

class taito_pccard1_device : public ata_flash_pccard_device
{
public:
	taito_pccard1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ16_MEMBER(read_reg) override;
	virtual DECLARE_WRITE16_MEMBER(write_reg) override;

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void process_command() override;
	virtual bool is_ready() override;

private:
	uint8_t m_key[5];
	uint16_t m_locked;
};

DECLARE_DEVICE_TYPE(TAITO_PCCARD2, taito_pccard2_device)

class taito_pccard2_device : public ata_flash_pccard_device
{
public:
	taito_pccard2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void process_command() override;
	virtual void process_buffer() override;
	virtual bool is_ready() override;

	static const int IDE_COMMAND_TAITO_GNET_UNLOCK_1 = 0xfe;
	static const int IDE_COMMAND_TAITO_GNET_UNLOCK_2 = 0xfc;

private:
	uint8_t m_key[5];
	bool m_locked;
};

DECLARE_DEVICE_TYPE(TAITO_COMPACT_FLASH, taito_compact_flash_device)

class taito_compact_flash_device : public ata_flash_pccard_device
{
public:
	taito_compact_flash_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void process_command() override;
	virtual bool is_ready() override;

	static constexpr int IDE_COMMAND_TAITO_COMPACT_FLASH_UNLOCK = 0x0f;

private:
	uint8_t m_key[5];
	bool m_locked;
};

#endif // MAME_MACHINE_ATAFLASH_H
