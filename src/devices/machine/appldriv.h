// license:BSD-3-Clause
// copyright-holders:R. Belmont
/*********************************************************************

    appldriv.h

    Apple 5.25" floppy drive emulation (to be interfaced with applefdc.c)

*********************************************************************/

#ifndef MAME_MACHINE_APPLDRIV_H
#define MAME_MACHINE_APPLDRIV_H

#pragma once

#include "imagedev/flopdrv.h"
#include "formats/ap2_dsk.h"

void apple525_set_lines(device_t *device, uint8_t lines);
void apple525_set_enable_lines(device_t *device, int enable_mask);

uint8_t apple525_read_data(device_t *device);
void apple525_write_data(device_t *device, uint8_t data);
int apple525_read_status(device_t *device);
int apple525_get_count(running_machine &machine);

class apple525_floppy_image_device : public legacy_floppy_image_device
{
public:
	// construction/destruction
	apple525_floppy_image_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual image_init_result call_load() override;
	virtual void call_unload() override;
	void set_params(int dividend, int divisor) { m_dividend = dividend; m_divisor = divisor; }

	int get_dividend() { return m_dividend; }
	int get_divisor() { return m_divisor; }

	// these elements should be private, but are not yet
	unsigned int state : 4;     /* bits 0-3 are the phase */
	unsigned int tween_tracks : 1;
	unsigned int track_loaded : 1;
	unsigned int track_dirty : 1;
	int position;
	int spin_count;         /* simulate drive spin to fool RWTS test at $BD34 */
	uint8_t track_data[APPLE2_NIBBLE_SIZE * APPLE2_SECTOR_COUNT];

protected:
	virtual void device_start() override;

private:
	int m_dividend;
	int m_divisor;
};

// device type definition
DECLARE_DEVICE_TYPE(FLOPPY_APPLE, apple525_floppy_image_device)

#define MCFG_LEGACY_FLOPPY_APPLE_PARAMS(_dividend,_divisor) \
	downcast<apple525_floppy_image_device *>(device)->set_params(_dividend,_divisor);

#define MCFG_LEGACY_FLOPPY_APPLE_2_DRIVES_ADD(_config,_dividend,_divisor)   \
	MCFG_DEVICE_ADD(FLOPPY_0, FLOPPY_APPLE, 0)      \
	MCFG_LEGACY_FLOPPY_CONFIG(_config) \
	MCFG_LEGACY_FLOPPY_APPLE_PARAMS(_dividend,_divisor) \
	MCFG_DEVICE_ADD(FLOPPY_1, FLOPPY_APPLE, 0)      \
	MCFG_LEGACY_FLOPPY_CONFIG(_config) \
	MCFG_LEGACY_FLOPPY_APPLE_PARAMS(_dividend,_divisor)

#define MCFG_LEGACY_FLOPPY_APPLE_4_DRIVES_ADD(_config,_dividend,_divisor)   \
	MCFG_DEVICE_ADD(FLOPPY_0, FLOPPY_APPLE, 0)      \
	MCFG_LEGACY_FLOPPY_CONFIG(_config) \
	MCFG_LEGACY_FLOPPY_APPLE_PARAMS(_dividend,_divisor) \
	MCFG_DEVICE_ADD(FLOPPY_1, FLOPPY_APPLE, 0)      \
	MCFG_LEGACY_FLOPPY_CONFIG(_config) \
	MCFG_LEGACY_FLOPPY_APPLE_PARAMS(_dividend,_divisor) \
	MCFG_DEVICE_ADD(FLOPPY_2, FLOPPY_APPLE, 0)      \
	MCFG_LEGACY_FLOPPY_CONFIG(_config) \
	MCFG_LEGACY_FLOPPY_APPLE_PARAMS(_dividend,_divisor) \
	MCFG_DEVICE_ADD(FLOPPY_3, FLOPPY_APPLE, 0)      \
	MCFG_LEGACY_FLOPPY_CONFIG(_config) \
	MCFG_LEGACY_FLOPPY_APPLE_PARAMS(_dividend,_divisor)

#define MCFG_LEGACY_FLOPPY_APPLE_2_DRIVES_REMOVE()  \
	MCFG_DEVICE_REMOVE(FLOPPY_0)        \
	MCFG_DEVICE_REMOVE(FLOPPY_1)

#endif // MAME_MACHINE_APPLDRIV_H
