// license:BSD-3-Clause
// copyright-holders:smf
#include "emu.h"
#include "ataflash.h"

DEFINE_DEVICE_TYPE(ATA_FLASH_PCCARD, ata_flash_pccard_device, "ataflash", "ATA Flash PC Card")

ata_flash_pccard_device::ata_flash_pccard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	ata_flash_pccard_device(mconfig, ATA_FLASH_PCCARD, tag, owner, clock)
{
}

ata_flash_pccard_device::ata_flash_pccard_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: ide_hdd_device(mconfig, type, tag, owner, clock)
{
}

void ata_flash_pccard_device::device_reset()
{
	ide_hdd_device::device_reset();

	uint32_t metalength;
	memset(m_cis, 0xff, 512);

	if (m_handle != nullptr)
	{
		m_handle->read_metadata(PCMCIA_CIS_METADATA_TAG, 0, m_cis, 512, metalength);
	}

	m_configuration_option = 0;
	m_configuration_and_status = 0;
	m_pin_replacement = 0x002e;
}

READ16_MEMBER( ata_flash_pccard_device::read_memory )
{
	if(offset <= 7)
	{
		m_8bit_data_transfers = !ACCESSING_BITS_8_15; // HACK
		return read_cs0(space, offset, mem_mask);
	}
	else if(offset <= 15)
	{
		return read_cs1(space, offset & 7, mem_mask);
	}
	else
	{
		return 0xffff;
	}
}

WRITE16_MEMBER( ata_flash_pccard_device::write_memory )
{
	if(offset <= 7)
	{
		m_8bit_data_transfers = !ACCESSING_BITS_8_15; // HACK
		write_cs0(space, offset, data, mem_mask);
	}
	else if( offset <= 15)
	{
		write_cs1(space, offset & 7, data, mem_mask);
	}
}

READ16_MEMBER( ata_flash_pccard_device::read_reg )
{
	switch (offset)
	{
	case 0x100:
		return m_configuration_option;

	case 0x101:
		return m_configuration_and_status;

	case 0x102:
		return m_pin_replacement;

	default:
		if (offset < 0x100)
			return m_cis[offset];
	}

	return pccard_interface::read_reg(space, offset, mem_mask);
}

WRITE16_MEMBER( ata_flash_pccard_device::write_reg )
{
	/// TODO: get offsets from CIS
	switch (offset)
	{
	case 0x07:
		// TODO: figure out what this is
		/// taito type 1: 0x0e, 0x0a (after unlock)
		/// taito type 2: 0xdf, 0xdf (before unlock)
		/// taito compact 0x0c, 0x0c, 0x0c, 0x0c (before unlock)
		// logerror("unknown reg 0x07 %02x\n", data);
		break;

	case 0x100:
		m_configuration_option = data;
		break;

	case 0x101:
		// TODO: irq ack
		m_configuration_and_status = data;
		break;

	default:
		pccard_interface::write_reg(space, offset, data, mem_mask);
		break;
	}
}

attotime ata_flash_pccard_device::seek_time()
{
	return attotime::zero;
}


DEFINE_DEVICE_TYPE(TAITO_PCCARD1, taito_pccard1_device, "taito_pccard1", "Taito PC Card (Type 1)")

taito_pccard1_device::taito_pccard1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	ata_flash_pccard_device(mconfig, TAITO_PCCARD1, tag, owner, clock),
	m_locked(0)
{
}

void taito_pccard1_device::device_start()
{
	ata_flash_pccard_device::device_start();

	save_item(NAME(m_locked));
}

void taito_pccard1_device::device_reset()
{
	ata_flash_pccard_device::device_reset();

	uint32_t metalength;
	memset(m_key, 0, sizeof(m_key));

	if (m_handle != nullptr && m_handle->read_metadata(HARD_DISK_KEY_METADATA_TAG, 0, m_key, 5, metalength) == CHDERR_NONE)
	{
		m_locked = 0x1ff;
	}
}

READ16_MEMBER(taito_pccard1_device::read_reg)
{
	switch (offset)
	{
	case 0x201:
		return m_locked != 0;

	default:
		return ata_flash_pccard_device::read_reg(space, offset, mem_mask);
	}
}

WRITE16_MEMBER(taito_pccard1_device::write_reg)
{
	if (offset >= 0x280 && offset <= 0x288)
	{
		uint8_t v = data;
		int pos = offset - 0x280;
		uint8_t k = pos < sizeof(m_key) ? m_key[pos] : 0;

		// TODO: find out if unlocking the key then using an incorrect key will re-lock the card.
		if (v == k)
		{
			m_locked &= ~(1 << pos);
		}
		else
		{
			m_locked |= 1 << pos;
		}

		// logerror("unlock %d %02x %04x\n", pos, data, m_locked);
	}
	else
	{
		ata_flash_pccard_device::write_reg(space, offset, data, mem_mask);
	}
}

void taito_pccard1_device::process_command()
{
	m_buffer_size = IDE_DISK_SECTOR_SIZE;

	switch (m_command)
	{
	default:
		if (m_locked != 0)
		{
			m_status |= IDE_STATUS_ERR;
			m_error = IDE_ERROR_NONE;
			m_status &= ~IDE_STATUS_DRDY;
			return;
		}
		break;
	}

	ata_flash_pccard_device::process_command();
}

bool taito_pccard1_device::is_ready()
{
	return m_locked == 0;
}

DEFINE_DEVICE_TYPE(TAITO_PCCARD2, taito_pccard2_device, "taito_pccard2", "Taito PC Card (Type 2)")

taito_pccard2_device::taito_pccard2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	ata_flash_pccard_device(mconfig, TAITO_PCCARD2, tag, owner, clock),
	m_locked(false)
{
}

void taito_pccard2_device::device_start()
{
	ata_flash_pccard_device::device_start();

	save_item(NAME(m_locked));
}

void taito_pccard2_device::device_reset()
{
	ata_flash_pccard_device::device_reset();

	uint32_t metalength;
	memset(m_key, 0, sizeof(m_key));

	if (m_handle != nullptr && m_handle->read_metadata(HARD_DISK_KEY_METADATA_TAG, 0, m_key, 5, metalength) == CHDERR_NONE)
	{
		m_locked = true;
	}
}

void taito_pccard2_device::process_command()
{
	m_buffer_size = IDE_DISK_SECTOR_SIZE;

	switch (m_command)
	{
	case IDE_COMMAND_TAITO_GNET_UNLOCK_1:
		//LOGPRINT(("IDE GNET Unlock 1\n"));

		m_sector_count = 1;
		m_status |= IDE_STATUS_DRDY;

		set_irq(ASSERT_LINE);
		return;

	case IDE_COMMAND_TAITO_GNET_UNLOCK_2:
		//LOGPRINT(("IDE GNET Unlock 2\n"));

		/* mark the buffer ready */
		m_status |= IDE_STATUS_DRQ;

		set_irq(ASSERT_LINE);
		return;

	default:
		if (m_locked)
		{
			m_status |= IDE_STATUS_ERR;
			m_error = IDE_ERROR_NONE;
			m_status &= ~IDE_STATUS_DRDY;
			return;
		}
		break;
	}

	ata_flash_pccard_device::process_command();
}

void taito_pccard2_device::process_buffer()
{
	if (m_command == IDE_COMMAND_TAITO_GNET_UNLOCK_2)
	{
		int i, bad = 0;

		for (i = 0; !bad && i<512; i++)
			bad = ((i < 2 || i >= 7) && m_buffer[i]) || ((i >= 2 && i < 7) && m_buffer[i] != m_key[i - 2]);

		if (bad)
		{
			// TODO: find out if unlocking the key then using an incorrect key will re-lock the card.
			m_status |= IDE_STATUS_ERR;
			m_error = IDE_ERROR_NONE;
		}
		else
		{
			m_locked = false;
		}

		// logerror("unlock %02x %02x %02x %02x %02x %d\n", m_buffer[2], m_buffer[3], m_buffer[4], m_buffer[5], m_buffer[6], m_locked);
	}
	else
	{
		ata_flash_pccard_device::process_buffer();
	}
}

bool taito_pccard2_device::is_ready()
{
	return !m_locked;
}

DEFINE_DEVICE_TYPE(TAITO_COMPACT_FLASH, taito_compact_flash_device, "taito_cf", "Taito Compact Flash")

taito_compact_flash_device::taito_compact_flash_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	ata_flash_pccard_device(mconfig, TAITO_COMPACT_FLASH, tag, owner, clock),
	m_locked(false)
{
}

void taito_compact_flash_device::device_start()
{
	ata_flash_pccard_device::device_start();

	save_item(NAME(m_locked));
}

void taito_compact_flash_device::device_reset()
{
	ata_flash_pccard_device::device_reset();

	uint32_t metalength;
	memset(m_key, 0, sizeof(m_key));

	if (m_handle != nullptr && m_handle->read_metadata(HARD_DISK_KEY_METADATA_TAG, 0, m_key, 5, metalength) == CHDERR_NONE)
	{
		m_locked = true;
	}
}

void taito_compact_flash_device::process_command()
{
	m_buffer_size = IDE_DISK_SECTOR_SIZE;

	switch (m_command)
	{
	case IDE_COMMAND_TAITO_COMPACT_FLASH_UNLOCK:
		/* key check */
		if (m_feature != m_key[0] || m_sector_count != m_key[1] || m_sector_number != m_key[2] || m_cylinder_low != m_key[3] || m_cylinder_high != m_key[4])
		{
			// TODO: find out if unlocking the key then using an incorrect key will lock the re-card.
			m_status &= ~IDE_STATUS_DRDY; // TODO: check if this is used as a flag to the unlock code, would it already be set this early?
		}
		else
		{
			m_locked = false;
		}

		// logerror("unlock %02x %02x %02x %02x %02x %d\n", m_feature, m_sector_count, m_sector_number, m_cylinder_low, m_cylinder_high, m_locked);

		set_irq(ASSERT_LINE);
		return;

	default:
		if (m_locked)
		{
			m_status |= IDE_STATUS_ERR;
			m_error = IDE_ERROR_NONE;
			m_status &= ~IDE_STATUS_DRDY;
			return;
		}
		break;
	}

	ata_flash_pccard_device::process_command();
}

bool taito_compact_flash_device::is_ready()
{
	return !m_locked;
}
