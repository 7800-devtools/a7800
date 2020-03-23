// license:BSD-3-Clause
// copyright-holders:David Haywood
/* Inder / Dinamic Video */


/* */


#ifndef MAME_MACHINE_INDER_VID_H
#define MAME_MACHINE_INDER_VID_H

#pragma once


#include "video/ramdac.h"
#include "cpu/tms34010/tms34010.h"

DECLARE_DEVICE_TYPE(INDER_VIDEO, inder_vid_device)

#define MCFG_INDER_VIDEO_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, INDER_VIDEO, 0)


class inder_vid_device : public device_t
/*  public device_video_interface */
{
public:
	// construction/destruction
	inder_vid_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	required_shared_ptr<uint16_t> m_vram;
	required_device<palette_device> m_palette;
	required_device<tms34010_device> m_tms;

	int m_shiftfull; // this might be a driver specific hack for a TMS bug.

	DECLARE_WRITE_LINE_MEMBER(m68k_gen_int);

	TMS340X0_TO_SHIFTREG_CB_MEMBER(to_shiftreg);
	TMS340X0_FROM_SHIFTREG_CB_MEMBER(from_shiftreg);
	TMS340X0_SCANLINE_RGB32_CB_MEMBER(scanline);
};

#endif // MAME_MACHINE_INDER_VID_H
