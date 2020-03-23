// license:BSD-3-Clause
// copyright-holders:Nathan Woods, Raphael Nabet, R. Belmont
/*********************************************************************

    sonydriv.h

    Apple/Sony 3.5" floppy drive emulation (to be interfaced with applefdc.c)

*********************************************************************/

#ifndef MAME_MACHINE_SONYDRIV_H
#define MAME_MACHINE_SONYDRIV_H

#pragma once

#include "imagedev/flopdrv.h"

#if 0
enum
{
	SONY_FLOPPY_ALLOW400K           = 0x0001,
	SONY_FLOPPY_ALLOW800K           = 0x0002,

	SONY_FLOPPY_SUPPORT2IMG         = 0x4000,
	SONY_FLOPPY_EXT_SPEED_CONTROL   = 0x8000    // means the speed is controlled by computer
};
#endif

void sony_set_lines(device_t *device, uint8_t lines);
void sony_set_enable_lines(device_t *device, int enable_mask);
void sony_set_sel_line(device_t *device, int sel);

void sony_set_speed(int speed);

uint8_t sony_read_data(device_t *device);
void sony_write_data(device_t *device, uint8_t data);
int sony_read_status(device_t *device);

class sonydriv_floppy_image_device : public legacy_floppy_image_device
{
public:
	// construction/destruction
	sonydriv_floppy_image_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void call_unload() override;

protected:
	virtual void device_start() override;
};

// device type definition
DECLARE_DEVICE_TYPE(FLOPPY_SONY, sonydriv_floppy_image_device)

#define MCFG_LEGACY_FLOPPY_SONY_2_DRIVES_ADD(_config)   \
	MCFG_DEVICE_ADD(FLOPPY_0, FLOPPY_SONY, 0)       \
	MCFG_LEGACY_FLOPPY_CONFIG(_config) \
	MCFG_DEVICE_ADD(FLOPPY_1, FLOPPY_SONY, 0)       \
	MCFG_LEGACY_FLOPPY_CONFIG(_config)

#define MCFG_LEGACY_FLOPPY_SONY_2_DRIVES_ADDITIONAL_ADD(_config)    \
	MCFG_DEVICE_ADD(FLOPPY_2, FLOPPY_SONY, 0)       \
	MCFG_LEGACY_FLOPPY_CONFIG(_config) \
	MCFG_DEVICE_ADD(FLOPPY_3, FLOPPY_SONY, 0)       \
	MCFG_LEGACY_FLOPPY_CONFIG(_config)

#define MCFG_LEGACY_FLOPPY_SONY_2_DRIVES_MODIFY(_config)    \
	MCFG_DEVICE_MODIFY(FLOPPY_0)        \
	MCFG_LEGACY_FLOPPY_CONFIG(_config) \
	MCFG_DEVICE_MODIFY(FLOPPY_1)        \
	MCFG_LEGACY_FLOPPY_CONFIG(_config)

#endif // MAME_MACHINE_SONYDRIV_H
