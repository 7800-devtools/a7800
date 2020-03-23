// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    harddisk.h

    Generic MAME hard disk implementation, with differencing files

***************************************************************************/

#pragma once

#ifndef __HARDDISK_H__
#define __HARDDISK_H__

#include "osdcore.h"
#include "chd.h"


/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

struct hard_disk_file;

struct hard_disk_info
{
	uint32_t          cylinders;
	uint32_t          heads;
	uint32_t          sectors;
	uint32_t          sectorbytes;
};



/***************************************************************************
    FUNCTION PROTOTYPES
***************************************************************************/

hard_disk_file *hard_disk_open(chd_file *chd);
void hard_disk_close(hard_disk_file *file);

chd_file *hard_disk_get_chd(hard_disk_file *file);
hard_disk_info *hard_disk_get_info(hard_disk_file *file);

uint32_t hard_disk_read(hard_disk_file *file, uint32_t lbasector, void *buffer);
uint32_t hard_disk_write(hard_disk_file *file, uint32_t lbasector, const void *buffer);

#endif  /* __HARDDISK_H__ */
