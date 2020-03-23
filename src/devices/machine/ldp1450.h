// license:BSD-3-Clause
// copyright-holders:Angelo Salese
/***************************************************************************

    Sony LDP-1450 laserdisc emulation.

***************************************************************************/

#ifndef MAME_MACHINE_LDP1450_H
#define MAME_MACHINE_LDP1450_H

#pragma once

#include "laserdsc.h"


//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_LASERDISC_LDP1450_ADD(_tag, clock) \
	MCFG_DEVICE_ADD(_tag, SONY_LDP1450, clock)

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// device type definition
DECLARE_DEVICE_TYPE(SONY_LDP1450, sony_ldp1450_device)

// ======================> sony_ldp1450_device

class sony_ldp1450_device : public laserdisc_device
{
public:
	// construction/destruction
	sony_ldp1450_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// I/O operations TODO: both actually protected
	void command_w(uint8_t data);
	uint8_t status_r() const { return m_status; }

protected:
	// device-level overrides
	virtual void device_validity_check(validity_checker &valid) const override;
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual const tiny_rom_entry *device_rom_region() const override;

	virtual void player_vsync(const vbi_metadata &vbi, int fieldnum, const attotime &curtime) override;
	virtual int32_t player_update(const vbi_metadata &vbi, int fieldnum, const attotime &curtime) override;
	virtual void player_overlay(bitmap_yuy16 &bitmap) override { }

	uint8_t m_ld_frame_index;
	uint8_t m_ld_frame[5];
	uint8_t m_ld_command_current_byte;
	uint8_t m_ld_command_to_send[5];
	uint8_t m_ld_command_total_bytes;

	enum LD_INPUT_STATE
	{
		LD_INPUT_GET_COMMAND = 0,
		LD_INPUT_TEXT_COMMAND,
		LD_INPUT_TEXT_GET_X,
		LD_INPUT_TEXT_GET_Y,
		LD_INPUT_TEXT_GET_MODE,
		LD_INPUT_TEXT_GET_STRING,
		LD_INPUT_TEXT_GET_SET_WINDOW
	} m_ld_input_state;

	enum ldp1450_player_state {
		player_standby = 0,
		player_search,
		player_search_clr,
		player_play,
		player_stop,
		player_repeat
	};

private:
	uint8_t m_command;
	uint8_t m_status;
	ldp1450_player_state m_player_state;
	bool m_audio_enable[2];
	void set_new_player_state(ldp1450_player_state which);
	void set_new_player_bcd(uint8_t data);
	uint32_t bcd_to_raw();
	void exec_enter_cmd();
	uint8_t m_internal_bcd[0x10];
	uint8_t m_index_state;

};

#endif // MAME_MACHINE_LDP1450_H
