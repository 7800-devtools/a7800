// license:BSD-3-Clause
// copyright-holders:Frank Palazzolo, Aaron Giles, Jonathan Gevaryahu, Raphael Nabet, Couriersud, Michael Zapf
/*
 * Voice Synthesis Memory
 *
 */

#ifndef MAME_MACHINE_SPCHROM_H
#define MAME_MACHINE_SPCHROM_H

#pragma once


class speechrom_device : public device_t
{
public:
	// construction/destruction
	speechrom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	/// TODO: implement bus behaviour
	int read(int count);
	void load_address(int data);
	void read_and_branch();
	void set_reverse_bit_order(bool reverse) { m_reverse = reverse; }

protected:
	// device-level overrides
	virtual void device_start() override;

private:
	uint8_t *m_speechrom_data;           /* pointer to speech ROM data */
	unsigned int m_speechROMlen;       /* length of data pointed by speechrom_data, from 0 to 2^18 */
	unsigned int m_speechROMaddr;      /* 18 bit pointer in ROM */
	int m_load_pointer;                /* which 4-bit nibble will be affected by load address */
	int m_ROM_bits_count;              /* current bit position in ROM */
	bool m_reverse;
};


// device type definition
DECLARE_DEVICE_TYPE(SPEECHROM, speechrom_device)

#endif // MAME_MACHINE_SPCHROM_H
