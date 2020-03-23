// license:GPL-2.0+
// copyright-holders:Raphael Nabet
/*************************************************************************

    video/crt.h

    CRT video emulation for TX-0 and PDP-1

*************************************************************************/

#ifndef MAME_VIDEO_CRT_H
#define MAME_VIDEO_CRT_H

#pragma once


//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_CRT_NUM_LEVELS(_lev) \
	crt_device::set_num_levels(*device, _lev);

#define MCFG_CRT_OFFSETS(_xoffs, _yoffs) \
	crt_device::set_offsets(*device, _xoffs, _yoffs);

#define MCFG_CRT_SIZE(_width, _height) \
	crt_device::set_size(*device, _width, _height);


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> crt_device

class crt_device : public device_t
{
public:
	crt_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void set_num_levels(device_t &device, int levels) { downcast<crt_device &>(device).m_num_intensity_levels = levels; }
	static void set_offsets(device_t &device, int x_offset, int y_offset)
	{
		crt_device &dev = downcast<crt_device &>(device);
		dev.m_window_offset_x = x_offset;
		dev.m_window_offset_y = y_offset;
	}
	static void set_size(device_t &device, int width, int height)
	{
		crt_device &dev = downcast<crt_device &>(device);
		dev.m_window_width = width;
		dev.m_window_height = height;
	}

	void plot(int x, int y);
	void eof();
	void update(bitmap_ind16 &bitmap);

protected:
	// device-level overrides
	virtual void device_start() override;

private:
	struct crt_point
	{
		crt_point() { }

		int intensity = 0;  /* current intensity of the pixel */
							/* a node is not in the list when (intensity == -1) */
		int next = 0;       /* index of next pixel in list */
	};

	crt_point *m_list; /* array of (crt_window_width*crt_window_height) point */
	std::unique_ptr<int[]> m_list_head;  /* head of the list of lit pixels (index in the array) */
						/* keep a separate list for each display line (makes the video code slightly faster) */

	int m_decay_counter;  /* incremented each frame (tells for how many frames the CRT has decayed between two screen refresh) */

	/* CRT window */
	int m_num_intensity_levels;
	int m_window_offset_x;
	int m_window_offset_y;
	int m_window_width;
	int m_window_height;
};

DECLARE_DEVICE_TYPE(CRT, crt_device)



#endif // MAME_VIDEO_CRT_H
