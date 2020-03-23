// license:BSD-3-Clause
// copyright-holders:smf
/***************************************************************************

t10mmc.h

***************************************************************************/

#ifndef MAME_MACHINE_T10MMC_H
#define MAME_MACHINE_T10MMC_H

#pragma once

#include "t10spc.h"
#include "imagedev/chd_cd.h"
#include "sound/cdda.h"

class t10mmc : public virtual t10spc
{
public:
	t10mmc()
		: t10spc(), m_image(nullptr), m_cdda(nullptr), m_cdrom(nullptr), m_lba(0), m_blocks(0), m_last_lba(0), m_num_subblocks(0), m_cur_subblock(0), m_audio_sense(0), m_device(nullptr)
	{
	}

	virtual void SetDevice( void *device ) override;
	virtual void GetDevice( void **device ) override;
	virtual void ExecCommand() override;
	virtual void WriteData( uint8_t *data, int dataLength ) override;
	virtual void ReadData( uint8_t *data, int dataLength ) override;

protected:
	virtual void t10_start(device_t &device) override;
	virtual void t10_reset() override;

	enum
	{
		T10MMC_CMD_READ_SUB_CHANNEL = 0x42,
		T10MMC_CMD_READ_TOC_PMA_ATIP = 0x43,
		T10MMC_CMD_PLAY_AUDIO_10 = 0x45,
		T10MMC_CMD_PLAY_AUDIO_MSF = 0x47,
		T10MMC_CMD_PLAY_AUDIO_TRACK_INDEX = 0x48,
		T10MMC_CMD_PAUSE_RESUME = 0x4b,
		T10MMC_CMD_STOP_PLAY_SCAN = 0x4e,
		T10MMC_CMD_PLAY_AUDIO_12 = 0xa5,
		T10MMC_CMD_SET_CD_SPEED = 0xbb
	};

	enum toc_format_t
	{
		TOC_FORMAT_TRACKS = 0,
		TOC_FORMAT_SESSIONS = 1
	};

	void abort_audio();
	toc_format_t toc_format();
	int toc_tracks();

	cdrom_image_device *m_image;
	cdda_device *m_cdda;
	cdrom_file *m_cdrom;

	uint32_t m_lba;
	uint32_t m_blocks;
	uint32_t m_last_lba;
	uint32_t m_num_subblocks;
	uint32_t m_cur_subblock;
	int m_audio_sense;
	bool m_sotc;

	device_t *m_device;
};

#endif // MAME_MACHINE_T10MMC_H
