// license:GPL-2.0+
// copyright-holders:Dirk Best
/***************************************************************************

    EACA Colour Genie Parallel Slot

    20-pin slot

     1  GND        11  IOB2
     2  +12V       12  IOB1
     3  IOA3       13  IOB0
     4  IOA4       14  IOB3
     5  IOA0       15  IOB4
     6  IOA5       16  IOB5
     7  IOA1       17  IOB7
     8  IOA2       18  IOB6
     9  IOA7       19  +5V
    10  IOA6       20  -12V

***************************************************************************/

#ifndef MAME_BUS_CGENIE_PARALLEL_PARALLEL_H
#define MAME_BUS_CGENIE_PARALLEL_PARALLEL_H

#pragma once



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_CG_PARALLEL_SLOT_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, CG_PARALLEL_SLOT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(cg_parallel_slot_carts, nullptr, false)


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class device_cg_parallel_interface;

class cg_parallel_slot_device : public device_t, public device_slot_interface
{
public:
	// construction/destruction
	cg_parallel_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~cg_parallel_slot_device();

	// IOA
	DECLARE_READ8_MEMBER(pa_r);
	DECLARE_WRITE8_MEMBER(pa_w);

	// IOB
	DECLARE_READ8_MEMBER(pb_r);
	DECLARE_WRITE8_MEMBER(pb_w);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	device_cg_parallel_interface *m_cart;
};

// class representing interface-specific live parallel device
class device_cg_parallel_interface : public device_slot_card_interface
{
public:
	// construction/destruction
	virtual ~device_cg_parallel_interface();

	virtual uint8_t pa_r() { return 0xff; }
	virtual void pa_w(uint8_t data) { }

	virtual uint8_t pb_r() { return 0xff; }
	virtual void pb_w(uint8_t data) { }

protected:
	device_cg_parallel_interface(const machine_config &mconfig, device_t &device);

	cg_parallel_slot_device *m_slot;
};

// device type definition
DECLARE_DEVICE_TYPE(CG_PARALLEL_SLOT, cg_parallel_slot_device)

// include here so drivers don't need to
#include "carts.h"

#endif // MAME_BUS_CGENIE_PARALLEL_PARALLEL_H
