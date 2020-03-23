// license:BSD-3-Clause
// copyright-holders:David Haywood
/* 68307 MBUS module */
/* all ports on this are 8-bit? */

#include "emu.h"
#include "68307bus.h"


#define m68307BUS_MADR (0x01)
#define m68307BUS_MFDR (0x03)
#define m68307BUS_MBCR (0x05)
#define m68307BUS_MBSR (0x07)
#define m68307BUS_MBDR (0x09)

READ8_MEMBER( m68307_cpu_device::m68307_internal_mbus_r )
{
	assert(m68307MBUS);
	m68307_mbus &mbus = *m68307MBUS;
	uint8_t retval;

	int pc = space.device().safe_pc();


	switch (offset)
	{
		case m68307BUS_MADR:
			logerror("%08x m68307_internal_mbus_r %08x (MADR - M-Bus Address Register)\n", pc, offset);
			return space.machine().rand();

		case m68307BUS_MFDR:
			logerror("%08x m68307_internal_mbus_r %08x (MFDR - M-Bus Frequency Divider Register)\n", pc, offset);
			return space.machine().rand();

		case m68307BUS_MBCR:
			logerror("%08x m68307_internal_mbus_r %08x (MFCR - M-Bus Control Register)\n", pc, offset);
			return mbus.m_MFCR;//space.machine().rand();

		case m68307BUS_MBSR:
			logerror("%08x m68307_internal_mbus_r %08x (MBSR - M-Bus Status Register)\n", pc, offset);
			retval = 0;
			if (mbus.m_busy) retval |= 0x20;
			if (mbus.m_intpend) retval |= 0x02;

			return retval;

		case m68307BUS_MBDR:
			logerror("%08x m68307_internal_mbus_r %08x (MBDR - M-Bus Data I/O Register)\n", pc, offset);
			mbus.m_intpend = true;
			return 0xff;//space.machine().rand();

		default:
			logerror("%08x m68307_internal_mbus_r %08x (UNKNOWN / ILLEGAL)\n", pc, offset);
			return 0x00;
	}

	return 0xff;
}

WRITE8_MEMBER( m68307_cpu_device::m68307_internal_mbus_w )
{
	assert(m68307MBUS);
	m68307_mbus &mbus = *m68307MBUS;

	int pc = space.device().safe_pc();

	switch (offset)
	{
		case m68307BUS_MADR:
			logerror("%08x m68307_internal_mbus_w %08x, %02x (MADR - M-Bus Address Register)\n", pc, offset,data);
			break;

		case m68307BUS_MFDR:
			logerror("%08x m68307_internal_mbus_w %08x, %02x (MFDR - M-Bus Frequency Divider Register)\n", pc, offset,data);
			break;

		case m68307BUS_MBCR:
			logerror("%08x m68307_internal_mbus_w %08x, %02x (MFCR - M-Bus Control Register)\n", pc, offset,data);

			mbus.m_MFCR = data;
			if (data & 0x80)
			{
				mbus.m_busy = false;
				mbus.m_intpend = false;
			}
			if (data & 0x20) mbus.m_busy = true;

			break;

		case m68307BUS_MBSR:
			logerror("%08x m68307_internal_mbus_w %08x, %02x (MBSR - M-Bus Status Register)\n", pc, offset,data);
			break;

		case m68307BUS_MBDR:
			logerror("%08x m68307_internal_mbus_w %08x, %02x (MBDR - M-Bus Data I/O Register)\n", pc, offset,data);

			mbus.m_intpend = true;

			break;

		default:
			logerror("%08x m68307_internal_mbus_w %08x, %02x (UNKNOWN / ILLEGAL)\n", pc, offset,data);
			break;
	}
}

void m68307_cpu_device::m68307_mbus::reset()
{
	m_busy = false;
}
