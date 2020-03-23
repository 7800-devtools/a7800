// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    divideo.c

    Device video interfaces.

***************************************************************************/

#include "emu.h"
#include "screen.h"


const char device_video_interface::s_unconfigured_screen_tag[] = "!!UNCONFIGURED!!";



//**************************************************************************
//  DEVICE VIDEO INTERFACE
//**************************************************************************

//-------------------------------------------------
//  device_video_interface - constructor
//-------------------------------------------------

device_video_interface::device_video_interface(const machine_config &mconfig, device_t &device, bool screen_required)
	: device_interface(device, "video"),
		m_screen_required(screen_required),
		m_screen_tag(s_unconfigured_screen_tag),
		m_screen(nullptr)
{
}


//-------------------------------------------------
//  ~device_video_interface - destructor
//-------------------------------------------------

device_video_interface::~device_video_interface()
{
}


//-------------------------------------------------
//  static_add_route - configuration helper to add
//  a new route to the device
//-------------------------------------------------

void device_video_interface::static_set_screen(device_t &device, const char *tag)
{
	// find our video interface
	device_video_interface *video;
	if (!device.interface(video))
		throw emu_fatalerror("MCFG_VIDEO_SET_SCREEN called on device '%s' with no video interface", device.tag());
	video->m_screen_tag = tag;
}


//-------------------------------------------------
//  interface_validity_check - validation for a
//  device after the configuration has been
//  constructed
//-------------------------------------------------

void device_video_interface::interface_validity_check(validity_checker &valid) const
{
	// only look up screens if we haven't explicitly requested no screen
	screen_device *screen = nullptr;
	if (m_screen_tag != nullptr)
	{
		// find the screen device if explicitly configured
		if (strcmp(m_screen_tag, s_unconfigured_screen_tag) != 0)
		{
			screen = device().siblingdevice<screen_device>(m_screen_tag);
			if (screen == nullptr)
				osd_printf_error("Screen '%s' not found, explicitly set for device '%s'\n", m_screen_tag, device().tag());
		}

		// otherwise, look for a single match
		else
		{
			screen_device_iterator iter(device().mconfig().root_device());
			screen = iter.first();
			if (iter.count() > 1)
				osd_printf_error("No screen specified for device '%s', but multiple screens found\n", device().tag());
		}
	}

	// error if no screen is found
	if (screen == nullptr && m_screen_required)
		osd_printf_error("Device '%s' requires a screen\n", device().tag());
}


//-------------------------------------------------
//  interface_pre_start - make sure all our input
//  devices are started
//-------------------------------------------------

void device_video_interface::interface_pre_start()
{
	// only look up screens if we haven't explicitly requested no screen
	if (m_screen_tag != nullptr)
	{
		// find the screen device if explicitly configured
		if (strcmp(m_screen_tag, s_unconfigured_screen_tag) != 0)
		{
			m_screen = device().siblingdevice<screen_device>(m_screen_tag);
			if (m_screen == nullptr)
				throw emu_fatalerror("Screen '%s' not found, explicitly set for device '%s'", m_screen_tag, device().tag());
		}

		// otherwise, look for a single match
		else
		{
			screen_device_iterator iter(device().machine().root_device());
			m_screen = iter.first();
			if (iter.count() > 1)
				throw emu_fatalerror("No screen specified for device '%s', but multiple screens found", device().tag());
		}
	}

	// fatal error if no screen is found
	if (m_screen == nullptr && m_screen_required)
		throw emu_fatalerror("Device '%s' requires a screen", device().tag());

	// if we have a screen and it's not started, wait for it
	device_palette_interface *palintf;
	if (m_screen != nullptr && !m_screen->started())
	{
		// avoid circular dependency if we are also a palette device
		if (!device().interface(palintf))
			throw device_missing_dependencies();
		else
		{
			// resolve the palette for the sake of register_screen_bitmap
			m_screen->resolve_palette();

			// no other palette may be specified (FIXME: breaks meritm.cpp)
			if (0 && m_screen->has_palette() && palintf != &m_screen->palette())
				throw emu_fatalerror("Device '%s' cannot control screen '%s' with palette '%s'", device().tag(), m_screen_tag, m_screen->palette().device().tag());
		}
	}
}
