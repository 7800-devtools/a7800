// license:BSD-3-Clause
// copyright-holders:Angelo Salese
/***************************************************************************

  SH7604 BUS Controller

***************************************************************************/

#ifndef MAME_CPU_SH2_SH7604_BUS_H
#define MAME_CPU_SH2_SH7604_BUS_H

#pragma once



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_SH7604_BUS_ADD(_tag,_freq) \
	MCFG_DEVICE_ADD(_tag, SH7604_BUS, _freq)

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> sh7604_bus_device

class sh7604_bus_device : public device_t
{
public:
	// construction/destruction
	sh7604_bus_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// I/O operations
	DECLARE_ADDRESS_MAP( bus_regs, 16 );

	DECLARE_WRITE32_MEMBER( write );
	DECLARE_READ32_MEMBER( read );
	DECLARE_READ16_MEMBER( bus_control_1_r );
	DECLARE_WRITE16_MEMBER( bus_control_1_w );
	DECLARE_READ16_MEMBER( bus_control_2_r );
	DECLARE_WRITE16_MEMBER( bus_control_2_w );
	DECLARE_READ16_MEMBER( wait_control_r );
	DECLARE_WRITE16_MEMBER( wait_control_w );
	DECLARE_READ16_MEMBER( memory_control_r );
	DECLARE_WRITE16_MEMBER( memory_control_w );
	DECLARE_READ16_MEMBER( refresh_timer_status_r );
	DECLARE_WRITE16_MEMBER( refresh_timer_control_w );
	DECLARE_READ16_MEMBER( refresh_timer_counter_r );
	DECLARE_WRITE16_MEMBER( refresh_timer_counter_w );
	DECLARE_READ16_MEMBER( refresh_timer_constant_r );
	DECLARE_WRITE16_MEMBER( refresh_timer_constant_w );

protected:
	// device-level overrides
	//virtual void device_validity_check(validity_checker &valid) const;
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	bool m_is_slave;
	const address_space_config      m_space_config;

	uint16_t m_bcr1;
	uint16_t m_bcr2;
	uint16_t m_wcr;
	uint16_t m_mcr;
	uint16_t m_rtcsr;
	uint16_t m_rtcor;
};


// device type definition
DECLARE_DEVICE_TYPE(SH7604_BUS, sh7604_bus_device)

#endif // MAME_CPU_SH2_SH7604_BUS_H
