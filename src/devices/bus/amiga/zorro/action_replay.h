// license:GPL-2.0+
// copyright-holders:Dirk Best
/***************************************************************************

    Datel Action Replay

    Freezer cartridge for Amiga 500 and Amiga 2000

***************************************************************************/

#ifndef MAME_BUS_AMIGA_ZORRO_ACTION_REPLAY_H
#define MAME_BUS_AMIGA_ZORRO_ACTION_REPLAY_H

#pragma once

#include "zorro.h"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> action_replay_device

class action_replay_device : public device_t, public device_exp_card_interface
{
public:
	// optional information overrides
	virtual ioport_constructor device_input_ports() const override;

	DECLARE_INPUT_CHANGED_MEMBER( freeze );

protected:
	// construction/destruction
	action_replay_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	required_ioport m_button;
};

class action_replay_mk1_device : public action_replay_device
{
public:
	// construction/destruction
	action_replay_mk1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
};

class action_replay_mk2_device : public action_replay_device
{
public:
	// construction/destruction
	action_replay_mk2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
};

class action_replay_mk3_device : public action_replay_device
{
public:
	// construction/destruction
	action_replay_mk3_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
};

// device type definition
DECLARE_DEVICE_TYPE(ACTION_REPLAY_MK1, action_replay_mk1_device)
DECLARE_DEVICE_TYPE(ACTION_REPLAY_MK2, action_replay_mk2_device)
DECLARE_DEVICE_TYPE(ACTION_REPLAY_MK3, action_replay_mk3_device)

#endif // MAME_BUS_AMIGA_ZORRO_ACTION_REPLAY_H
