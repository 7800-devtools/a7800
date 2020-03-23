// license:LGPL-2.1+
// copyright-holders:Michael Zapf
/****************************************************************************

    MFM hard disk emulation

    See mfmhd.c for documentation

    Michael Zapf
    August 2015

*****************************************************************************/

#ifndef MAME_DEVICES_IMAGEDEV_MFMHD_H
#define MAME_DEVICES_IMAGEDEV_MFMHD_H

#pragma once

#include "imagedev/harddriv.h"
#include "formats/mfm_hd.h"

class mfm_harddisk_device;

class mfmhd_trackimage
{
public:
	bool    dirty;
	int     cylinder;
	int     head;
	uint16_t* encdata;            // MFM encoding per byte
	mfmhd_trackimage* next;
};

class mfmhd_trackimage_cache
{
public:
	mfmhd_trackimage_cache(running_machine &machine);
	~mfmhd_trackimage_cache();
	void        init(mfm_harddisk_device* mfmhd, int tracksize, int trackslots);
	uint16_t*     get_trackimage(int cylinder, int head);
	void        mark_current_as_dirty();
	void        cleanup();
	void        write_back_one();

private:
	mfm_harddisk_device*        m_mfmhd;
	mfmhd_trackimage*           m_tracks;
	running_machine &           m_machine;
};

class mfm_harddisk_device : public harddisk_image_device,
							public device_slot_card_interface
{
public:
	~mfm_harddisk_device();

	typedef delegate<void (mfm_harddisk_device*, int)> index_pulse_cb;
	typedef delegate<void (mfm_harddisk_device*, int)> ready_cb;
	typedef delegate<void (mfm_harddisk_device*, int)> seek_complete_cb;

	void setup_index_pulse_cb(index_pulse_cb cb);
	void setup_ready_cb(ready_cb cb);
	void setup_seek_complete_cb(seek_complete_cb cb);

	// Configuration
	void set_encoding(mfmhd_enc_t encoding) { m_encoding = encoding; }
	void set_spinup_time(int spinupms) { m_spinupms = spinupms; }
	void set_cache_size(int tracks) { m_cachelines = tracks;    }
	void set_format(mfmhd_image_format_t* format) { m_format = format; }

	mfmhd_enc_t get_encoding() { return m_encoding; }

	// Active low lines. We're using ASSERT=0 / CLEAR=1
	line_state      ready_r() { return m_ready? ASSERT_LINE : CLEAR_LINE; }
	line_state      seek_complete_r() { return m_seek_complete? ASSERT_LINE : CLEAR_LINE; } ;
	line_state      trk00_r() { return m_current_cylinder==0? ASSERT_LINE : CLEAR_LINE; }

	// Data output towards controller
	bool            read(attotime &from_when, const attotime &limit, uint16_t &data);

	// Data input from controller
	bool            write(attotime &from_when, const attotime &limit, uint16_t cdata, bool wpcom=false, bool reduced_wc=false);

	// Step
	void            step_w(line_state line);
	void            direction_in_w(line_state line);

	// Head select
	void            headsel_w(int head) { m_current_head = head & 0x0f; }

	image_init_result            call_load() override;
	void            call_unload() override;

	// Tells us the time when the track ends (next index pulse). Needed by the controller.
	attotime        track_end_time();

	// Access the tracks on the image. Used as a callback from the cache.
	chd_error       load_track(uint16_t* data, int cylinder, int head);
	void            write_track(uint16_t* data, int cylinder, int head);

	// Delivers the number of heads according to the loaded image
	int             get_actual_heads();

protected:
	mfm_harddisk_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	void                device_start() override;
	void                device_stop() override;
	void                device_reset() override;
	void                device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	std::string         tts(const attotime &t);

	emu_timer           *m_index_timer, *m_spinup_timer, *m_seek_timer, *m_cache_timer;
	index_pulse_cb      m_index_pulse_cb;
	ready_cb            m_ready_cb;
	seek_complete_cb    m_seek_complete_cb;

	int         m_max_cylinders;
	int         m_phys_cylinders;
	int         m_actual_cylinders;  // after reading the CHD
	int         m_max_heads;
	int         m_landing_zone;
	int         m_precomp_cyl;
	int         m_redwc_cyl;

	int         m_maxseek_time;
	int         m_seeknext_time;

private:
	mfmhd_enc_t m_encoding;
	int         m_cell_size;    // nanoseconds
	int         m_trackimage_size;  // number of 16-bit cell blocks (data bytes)
	int         m_spinupms;
	int         m_rpm;
	int         m_interleave;
	int         m_cachelines;
	bool        m_ready;
	int         m_current_cylinder;
	int         m_current_head;
	int         m_track_delta;
	int         m_step_phase;
	bool        m_seek_complete;
	bool        m_seek_inward;
	bool        m_autotruncation;
	bool        m_recalibrated;
	line_state  m_step_line;    // keep the last state

	attotime    m_spinup_time;
	attotime    m_revolution_start_time;
	attotime    m_rev_time;

	attotime    m_settle_time;
	attotime    m_step_time;

	mfmhd_trackimage_cache* m_cache;
	mfmhd_image_format_t*   m_format;

	void        head_move();
	void        recalibrate();

	// Common routine for read/write
	bool            find_position(attotime &from_when, const attotime &limit, int &bytepos, int &bitpos);
};

/*
    The Generic drive is a MFM drive that has just enough heads and cylinders
    to handle the CHD image.

    Specific Seagate models:

    ST-213: 10 MB
    ST-225: 20 MB
    ST-251: 40 MB
*/
class mfm_hd_generic_device : public mfm_harddisk_device
{
public:
	mfm_hd_generic_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

DECLARE_DEVICE_TYPE(MFMHD_GENERIC, mfm_hd_generic_device)

class mfm_hd_st213_device : public mfm_harddisk_device
{
public:
	mfm_hd_st213_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

DECLARE_DEVICE_TYPE(MFMHD_ST213, mfm_hd_st213_device)

class mfm_hd_st225_device : public mfm_harddisk_device
{
public:
	mfm_hd_st225_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

DECLARE_DEVICE_TYPE(MFMHD_ST225, mfm_hd_st225_device)

class mfm_hd_st251_device : public mfm_harddisk_device
{
public:
	mfm_hd_st251_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

DECLARE_DEVICE_TYPE(MFMHD_ST251, mfm_hd_st251_device)


/* Connector for a MFM hard disk. See also floppy.c */
class mfm_harddisk_connector : public device_t,
								public device_slot_interface
{
public:
	mfm_harddisk_connector(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	~mfm_harddisk_connector();

	mfm_harddisk_device *get_device();

	void configure(mfmhd_enc_t encoding, int spinupms, int cache, mfmhd_format_type format);

protected:
	void device_start() override;
	void device_config_complete() override;

private:
	mfmhd_enc_t m_encoding;
	int m_spinupms;
	int m_cachesize;
	mfmhd_image_format_t* m_format;
};

DECLARE_DEVICE_TYPE(MFM_HD_CONNECTOR, mfm_harddisk_connector)

/*
    Add a harddisk connector.
    Parameters:
    _tag = Tag of the connector
    _slot_intf = Selection of hard drives
    _def_slot = Default hard drive
    _enc = Encoding (see comments in mfm_hd.c)
    _spinupms = Spinup time in milliseconds (some configurations assume that the
    user has turned on the hard disk before turning on the system. We cannot
    emulate this, so we allow for shorter times)
    _cache = number of cached MFM tracks
*/
#define MCFG_MFM_HARDDISK_CONN_ADD(_tag, _slot_intf, _def_slot, _enc, _spinupms, _cache, _format)  \
	MCFG_DEVICE_ADD(_tag, MFM_HD_CONNECTOR, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false) \
	static_cast<mfm_harddisk_connector *>(device)->configure(_enc, _spinupms, _cache, _format);


#endif // MAME_DEVICES_IMAGEDEV_MFMHD_H
