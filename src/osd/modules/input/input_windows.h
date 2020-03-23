// license:BSD-3-Clause
// copyright-holders:Aaron Giles, Brad Hughes
//============================================================
//
//  input_windows.h - Common code used by Windows input modules
//
//============================================================

#ifndef INPUT_WIN_H_
#define INPUT_WIN_H_

// standard windows headers
#include <windows.h>
#undef interface

#include "window.h"
#include "winmain.h"

//============================================================
//  TYPEDEFS
//============================================================

// state information for a keyboard
struct keyboard_state
{
	uint8_t                   state[MAX_KEYS];
	int8_t                    oldkey[MAX_KEYS];
	int8_t                    currkey[MAX_KEYS];
};

// state information for a mouse (matches DIMOUSESTATE exactly)
struct mouse_state
{
	LONG                    lX;
	LONG                    lY;
	LONG                    lZ;
	BYTE                    rgbButtons[8];
};

class wininput_module : public input_module_base
{
protected:
	bool  m_global_inputs_enabled;

public:
	wininput_module(const char * type, const char * name)
		: input_module_base(type, name),
			m_global_inputs_enabled(false)
	{
	}

	virtual ~wininput_module() { }

	virtual bool should_hide_mouse()
	{
		if (winwindow_has_focus()  // has focus
			&& (!video_config.windowed || !osd_common_t::s_window_list.front()->win_has_menu()) // not windowed or doesn't have a menu
			&& (input_enabled() && !input_paused()) // input enabled and not paused
			&& (mouse_enabled() || lightgun_enabled())) // either mouse or lightgun enabled in the core
		{
			return true;
		}

		return false;
	}

	virtual bool handle_input_event(input_event eventid, void* data)
	{
		return false;
	}

protected:

	void before_poll(running_machine& machine) override
	{
		// periodically process events, in case they're not coming through
		// this also will make sure the mouse state is up-to-date
		winwindow_process_events_periodic(machine);
	}

	bool should_poll_devices(running_machine &machine) override
	{
		return input_enabled() && (m_global_inputs_enabled || winwindow_has_focus());
	}
};

#endif
