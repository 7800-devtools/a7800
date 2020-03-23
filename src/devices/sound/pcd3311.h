// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    PCD3311 DTMF/modem/musical tone generator emulation

**********************************************************************
                            _____   _____
                  OSCI   1 |*    \_/     | 16  Vdd
                  OSCO   2 |             | 15  Vss
                  MODE   3 |             | 14  D4
                    D5   4 |  PCD3311T   | 13  N/C
                   N/C   5 |             | 12  D3
                STROBE   6 |             | 11  D2
                  TONE   7 |             | 10  D1/SDA
                    A0   8 |_____________| 9   D0/SCL

**********************************************************************/

#ifndef MAME_SOUND_PCD3311_H
#define MAME_SOUND_PCD3311_H

#pragma once




//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> pcd3311_device

class pcd3311_device :  public device_t,
				   public device_sound_interface
{
public:
	// construction/destruction
	pcd3311_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_WRITE8_MEMBER( write ) { m_data = data; }
	DECLARE_WRITE_LINE_MEMBER( strobe_w ) { m_strobe = state; }
	DECLARE_WRITE_LINE_MEMBER( mode_w ) { m_mode = state; }
	DECLARE_WRITE_LINE_MEMBER( a0_w ) { m_a0 = state; }

protected:
	// device-level overrides
	virtual void device_start() override;

	// internal callbacks
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

private:
	int m_a0;
	int m_mode;
	int m_strobe;
	uint8_t m_data;
};


// device type definition
DECLARE_DEVICE_TYPE(PCD3311, pcd3311_device)

#endif // MAME_SOUND_PCD3311_H
