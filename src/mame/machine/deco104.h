// license:BSD-3-Clause
// copyright-holders:David Haywood
#ifndef MAME_MACHINE_DECO104_H
#define MAME_MACHINE_DECO104_H

#pragma once

#include "deco146.h"


/* Data East 104 protection chip */

class deco104_device : public deco_146_base_device
{
public:
	deco104_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
};

DECLARE_DEVICE_TYPE(DECO104PROT, deco104_device)


#define MCFG_DECO104_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, DECO104PROT, 0)

#endif // MAME_MACHINE_DECO104_H
