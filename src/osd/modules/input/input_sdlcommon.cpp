// license:BSD-3-Clause
// copyright-holders:Olivier Galibert, R. Belmont, Brad Hughes
//============================================================
//
//  input_sdlcommon.cpp - SDL Common code shared by SDL modules
//
//    Note: this code is also used by the X11 input modules
//
//============================================================

#include "input_module.h"
#include "modules/osdmodule.h"

#if defined(OSD_SDL)

// standard sdl header
#include <SDL2/SDL.h>
#include <ctype.h>
#include <stddef.h>
#include <mutex>
#include <memory>
#include <algorithm>

// MAME headers
#include "emu.h"
#include "osdepend.h"
#include "ui/uimain.h"
#include "uiinput.h"
#include "window.h"
#include "strconv.h"

#include "../../sdl/osdsdl.h"
#include "input_common.h"
#include "input_sdlcommon.h"

#define GET_WINDOW(ev) window_from_id((ev)->windowID)
//#define GET_WINDOW(ev) ((ev)->windowID)

static std::shared_ptr<sdl_window_info> window_from_id(Uint32 windowID)
{
	SDL_Window *sdl_window = SDL_GetWindowFromID(windowID);

	auto& windows = osd_common_t::s_window_list;
	auto window = std::find_if(windows.begin(), windows.end(), [sdl_window](std::shared_ptr<osd_window> w)
	{
		return std::static_pointer_cast<sdl_window_info>(w)->platform_window() == sdl_window;
	});

	if (window == windows.end())
		return nullptr;

	return std::static_pointer_cast<sdl_window_info>(*window);
}

void sdl_event_manager::process_events(running_machine &machine)
{
	std::lock_guard<std::mutex> scope_lock(m_lock);
	SDL_Event sdlevent;
	while (SDL_PollEvent(&sdlevent))
	{
		// process window events if they come in
		if (sdlevent.type == SDL_WINDOWEVENT)
			process_window_event(machine, sdlevent);

		// Find all subscribers for the event type
		auto subscribers = m_subscription_index.equal_range(sdlevent.type);

		// Dispatch the events
		std::for_each(subscribers.first, subscribers.second, [&sdlevent](auto sub)
		{
			sub.second->handle_event(sdlevent);
		});
	}
}

void sdl_event_manager::process_window_event(running_machine &machine, SDL_Event &sdlevent)
{
	std::shared_ptr<sdl_window_info> window = GET_WINDOW(&sdlevent.window);

	if (window == nullptr)
	{
		osd_printf_warning("Skipped window event due to missing window param from SDL\n");
		return;
	}

	switch (sdlevent.window.event)
	{
	case SDL_WINDOWEVENT_SHOWN:
		m_has_focus = true;
		break;

	case SDL_WINDOWEVENT_CLOSE:
		machine.schedule_exit();
		break;

	case SDL_WINDOWEVENT_LEAVE:
		machine.ui_input().push_mouse_leave_event(window->target());
		m_mouse_over_window = 0;
		break;

	case SDL_WINDOWEVENT_MOVED:
		window->notify_changed();
		m_focus_window = window;
		m_has_focus = true;
		break;

	case SDL_WINDOWEVENT_RESIZED:
#ifdef SDLMAME_LINUX
		/* FIXME: SDL2 sends some spurious resize events on Ubuntu
		* while in fullscreen mode. Ignore them for now.
		*/
		if (!window->fullscreen())
#endif
		{
			//printf("event data1,data2 %d x %d %ld\n", event.window.data1, event.window.data2, sizeof(SDL_Event));
			window->resize(sdlevent.window.data1, sdlevent.window.data2);
		}
		m_focus_window = window;
		m_has_focus = true;
		break;

	case SDL_WINDOWEVENT_ENTER:
		m_mouse_over_window = 1;
		/* fall through */
	case SDL_WINDOWEVENT_FOCUS_GAINED:
	case SDL_WINDOWEVENT_EXPOSED:
	case SDL_WINDOWEVENT_MAXIMIZED:
	case SDL_WINDOWEVENT_RESTORED:
		m_focus_window = window;
		m_has_focus = true;
		break;

	case SDL_WINDOWEVENT_MINIMIZED:
	case SDL_WINDOWEVENT_FOCUS_LOST:
		m_has_focus = false;
		break;
	}
}

//============================================================
//  customize_input_type_list
//============================================================

void sdl_osd_interface::customize_input_type_list(simple_list<input_type_entry> &typelist)
{
	input_item_id mameid_code;
	input_code ui_code;
	const char* uimode;
	char fullmode[64];

	// loop over the defaults
	for (input_type_entry &entry : typelist)
	{
		switch (entry.type())
		{
			// configurable UI mode switch
		case IPT_UI_TOGGLE_UI:
			uimode = options().ui_mode_key();
			if (!strcmp(uimode, "auto"))
			{
#if defined(__APPLE__) && defined(__MACH__)
				mameid_code = keyboard_trans_table::instance().lookup_mame_code("ITEM_ID_INSERT");
#else
				mameid_code = keyboard_trans_table::instance().lookup_mame_code("ITEM_ID_SCRLOCK");
#endif
			}
			else
			{
				snprintf(fullmode, 63, "ITEM_ID_%s", uimode);
				mameid_code = keyboard_trans_table::instance().lookup_mame_code(fullmode);
			}
			ui_code = input_code(DEVICE_CLASS_KEYBOARD, 0, ITEM_CLASS_SWITCH, ITEM_MODIFIER_NONE, input_item_id(mameid_code));
			entry.defseq(SEQ_TYPE_STANDARD).set(ui_code);
			break;
			// alt-enter for fullscreen
		case IPT_OSD_1:
			entry.configure_osd("TOGGLE_FULLSCREEN", "Toggle Fullscreen");
			entry.defseq(SEQ_TYPE_STANDARD).set(KEYCODE_ENTER, KEYCODE_LALT);
			break;

			// disable UI_SELECT when LALT is down, this stops selecting
			// things in the menu when toggling fullscreen with LALT+ENTER
			/*          case IPT_UI_SELECT:
			entry.defseq(SEQ_TYPE_STANDARD).set(KEYCODE_ENTER, input_seq::not_code, KEYCODE_LALT);
			break;*/

			// page down for fastforward (must be OSD_3 as per src/emu/ui.c)
		case IPT_UI_FAST_FORWARD:
			entry.defseq(SEQ_TYPE_STANDARD).set(KEYCODE_PGDN);
			break;

			// OSD hotkeys use LCTRL and start at F3, they start at
			// F3 because F1-F2 are hardcoded into many drivers to
			// various dipswitches, and pressing them together with
			// LCTRL will still press/toggle these dipswitches.

			// LCTRL-F3 to toggle fullstretch
		case IPT_OSD_2:
			entry.configure_osd("TOGGLE_FULLSTRETCH", "Toggle Uneven stretch");
			entry.defseq(SEQ_TYPE_STANDARD).set(KEYCODE_F3, KEYCODE_LCONTROL);
			break;
			// add a Not lcrtl condition to the reset key
		case IPT_UI_SOFT_RESET:
			entry.defseq(SEQ_TYPE_STANDARD).set(KEYCODE_F3, input_seq::not_code, KEYCODE_LCONTROL, input_seq::not_code, KEYCODE_LSHIFT);
			break;

			// LCTRL-F4 to toggle keep aspect
		case IPT_OSD_4:
			entry.configure_osd("TOGGLE_KEEP_ASPECT", "Toggle Keepaspect");
			entry.defseq(SEQ_TYPE_STANDARD).set(KEYCODE_F4, KEYCODE_LCONTROL);
			break;
			// add a Not lcrtl condition to the show gfx key
		case IPT_UI_SHOW_GFX:
			entry.defseq(SEQ_TYPE_STANDARD).set(KEYCODE_F4, input_seq::not_code, KEYCODE_LCONTROL);
			break;

			// LCTRL-F5 to toggle OpenGL filtering
		case IPT_OSD_5:
			entry.configure_osd("TOGGLE_FILTER", "Toggle Filter");
			entry.defseq(SEQ_TYPE_STANDARD).set(KEYCODE_F5, KEYCODE_LCONTROL);
			break;
			// add a Not lcrtl condition to the toggle debug key
		case IPT_UI_TOGGLE_DEBUG:
			entry.defseq(SEQ_TYPE_STANDARD).set(KEYCODE_F5, input_seq::not_code, KEYCODE_LCONTROL);
			break;

			// LCTRL-F6 to decrease OpenGL prescaling
		case IPT_OSD_6:
			entry.configure_osd("DECREASE_PRESCALE", "Decrease Prescaling");
			entry.defseq(SEQ_TYPE_STANDARD).set(KEYCODE_F6, KEYCODE_LCONTROL);
			break;
			// add a Not lcrtl condition to the toggle cheat key
		case IPT_UI_TOGGLE_CHEAT:
			entry.defseq(SEQ_TYPE_STANDARD).set(KEYCODE_F6, input_seq::not_code, KEYCODE_LCONTROL);
			break;

			// LCTRL-F7 to increase OpenGL prescaling
		case IPT_OSD_7:
			entry.configure_osd("INCREASE_PRESCALE", "Increase Prescaling");
			entry.defseq(SEQ_TYPE_STANDARD).set(KEYCODE_F7, KEYCODE_LCONTROL);
			break;

		// lshift-lalt-F12 for fullscreen video (BGFX)
		case IPT_OSD_8:
			entry.configure_osd("RENDER_AVI", "Record Rendered Video");
			entry.defseq(SEQ_TYPE_STANDARD).set(KEYCODE_F12, KEYCODE_LSHIFT, KEYCODE_LALT);
			break;

		// add a Not lcrtl condition to the load state key
		case IPT_UI_LOAD_STATE:
			entry.defseq(SEQ_TYPE_STANDARD).set(KEYCODE_F7, input_seq::not_code, KEYCODE_LCONTROL, input_seq::not_code, KEYCODE_LSHIFT);
			break;

			// add a Not lcrtl condition to the throttle key
		case IPT_UI_THROTTLE:
			entry.defseq(SEQ_TYPE_STANDARD).set(KEYCODE_F10, input_seq::not_code, KEYCODE_LCONTROL);
			break;

			// disable the config menu if the ALT key is down
			// (allows ALT-TAB to switch between apps)
		case IPT_UI_CONFIGURE:
			entry.defseq(SEQ_TYPE_STANDARD).set(KEYCODE_TAB, input_seq::not_code, KEYCODE_LALT, input_seq::not_code, KEYCODE_RALT);
			break;

#if defined(__APPLE__) && defined(__MACH__)
		// 78-key Apple MacBook & Bluetooth keyboards have no right control key
		case IPT_MAHJONG_SCORE:
			if (entry.player() == 0)
				entry.defseq(SEQ_TYPE_STANDARD).set(KEYCODE_SLASH);
			break;
#endif

			// leave everything else alone
		default:
			break;
		}
	}
}

void sdl_osd_interface::poll_inputs(running_machine &machine)
{
	m_keyboard_input->poll_if_necessary(machine);
	m_mouse_input->poll_if_necessary(machine);
	m_lightgun_input->poll_if_necessary(machine);
	m_joystick_input->poll_if_necessary(machine);
}

void sdl_osd_interface::release_keys()
{
	auto keybd = dynamic_cast<input_module_base*>(m_keyboard_input);
	if (keybd != nullptr)
		keybd->devicelist()->reset_devices();
}

bool sdl_osd_interface::should_hide_mouse()
{
	// if we are paused, no
	if (machine().paused())
		return false;

	// if neither mice nor lightguns enabled in the core, then no
	if (!options().mouse() && !options().lightgun())
		return false;

	if (!sdl_event_manager::instance().mouse_over_window())
		return false;

	// otherwise, yes
	return true;
}

void sdl_osd_interface::process_events_buf()
{
	SDL_PumpEvents();
}

#endif
