// license:BSD-3-Clause
// copyright-holders:Tim Schuerewegen
/*

    NVIDIA GoForce 4500

    (c) 2010 Tim Schuerewegen

*/

#ifndef MAME_VIDEO_GF4500_H
#define MAME_VIDEO_GF4500_H

#pragma once


class gf4500_device : public device_t
{
public:
	gf4500_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);


	DECLARE_READ32_MEMBER( read );
	DECLARE_WRITE32_MEMBER( write );

	uint32_t screen_update(screen_device &device, bitmap_rgb32 &bitmap, const rectangle &cliprect);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	// internal state

	void vram_write16(uint16_t data);

	std::unique_ptr<uint32_t[]> m_data;
	int m_screen_x;
	int m_screen_y;
	int m_screen_x_max;
	int m_screen_y_max;
	int m_screen_x_min;
	int m_screen_y_min;
};



#define MCFG_GF4500_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, GF4500, 0)


DECLARE_DEVICE_TYPE(GF4500, gf4500_device)

#endif // MAME_VIDEO_GF4500_H
