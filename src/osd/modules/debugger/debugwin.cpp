// license:BSD-3-Clause
// copyright-holders:Aaron Giles, Vas Crabb
//============================================================
//
//  debugwin.c - Win32 debug window handling
//
//============================================================

#include "emu.h"
#include "debug_module.h"
#include "modules/osdmodule.h"

#if defined(OSD_WINDOWS) /*|| defined(SDLMAME_WIN32)*/

#include "win/debugwin.h"

#include "win/consolewininfo.h"
#include "win/debugwininfo.h"
#include "win/disasmwininfo.h"
#include "win/logwininfo.h"
#include "win/memorywininfo.h"
#include "win/pointswininfo.h"
#include "win/uimetrics.h"

#include "debugger.h"
#include "debug/debugcpu.h"

#include "window.h"
#include "../input/input_common.h"
#include "../input/input_windows.h"


class debugger_windows : public osd_module, public debug_module, protected debugger_windows_interface
{
public:
	debugger_windows() :
		osd_module(OSD_DEBUG_PROVIDER, "windows"),
		debug_module(),
		m_machine(nullptr),
		m_metrics(),
		m_waiting_for_debugger(false),
		m_window_list(),
		m_main_console(nullptr)
	{
	}

	virtual ~debugger_windows() { }

	virtual int init(const osd_options &options) override { return 0; }
	virtual void exit() override;

	virtual void init_debugger(running_machine &machine) override;
	virtual void wait_for_debugger(device_t &device, bool firststop) override;
	virtual void debugger_update() override;

protected:
	virtual running_machine &machine() const override { return *m_machine; }

	virtual ui_metrics &metrics() const override { return *m_metrics; }

	virtual bool const &waiting_for_debugger() const override { return m_waiting_for_debugger; }
	virtual bool seq_pressed() const override;

	virtual void create_memory_window() override { create_window<memorywin_info>(); }
	virtual void create_disasm_window() override { create_window<disasmwin_info>(); }
	virtual void create_log_window() override { create_window<logwin_info>(); }
	virtual void create_points_window() override { create_window<pointswin_info>(); }
	virtual void remove_window(debugwin_info &info) override;

	virtual void show_all() override;
	virtual void hide_all() override;

private:
	template <typename T> T *create_window();

	running_machine             *m_machine;
	std::unique_ptr<ui_metrics>    m_metrics;
	bool                        m_waiting_for_debugger;
	std::vector<std::unique_ptr<debugwin_info>>  m_window_list;
	consolewin_info             *m_main_console;
};


void debugger_windows::exit()
{
	// loop over windows and free them
	while (!m_window_list.empty())
		m_window_list.front()->destroy();

	m_main_console = nullptr;
	m_metrics.reset();
	m_machine = nullptr;
}


void debugger_windows::init_debugger(running_machine &machine)
{
	m_machine = &machine;
	m_metrics = std::make_unique<ui_metrics>(downcast<osd_options &>(m_machine->options()));
}


void debugger_windows::wait_for_debugger(device_t &device, bool firststop)
{
	// create a console window
	if (m_main_console == nullptr)
		m_main_console = create_window<consolewin_info>();

	// update the views in the console to reflect the current CPU
	if (m_main_console != nullptr)
		m_main_console->set_cpu(device);

	// when we are first stopped, adjust focus to us
	if (firststop && (m_main_console != nullptr))
	{
		m_main_console->set_foreground();
		if (winwindow_has_focus())
			m_main_console->set_default_focus();
	}

	// make sure the debug windows are visible
	m_waiting_for_debugger = true;
	show_all();

	// run input polling to ensure that our status is in sync
	downcast<windows_osd_interface&>(machine().osd()).poll_input(*m_machine);

	// get and process messages
	MSG message;
	GetMessage(&message, nullptr, 0, 0);

	switch (message.message)
	{
	// check for F10 -- we need to capture that ourselves
	case WM_SYSKEYDOWN:
	case WM_SYSKEYUP:
		if (message.wParam == VK_F4 && message.message == WM_SYSKEYDOWN)
			SendMessage(GetAncestor(GetFocus(), GA_ROOT), WM_CLOSE, 0, 0);
		if (message.wParam == VK_F10)
			SendMessage(GetAncestor(GetFocus(), GA_ROOT), (message.message == WM_SYSKEYDOWN) ? WM_KEYDOWN : WM_KEYUP, message.wParam, message.lParam);
		break;

	// process everything else
	default:
		winwindow_dispatch_message(*m_machine, &message);
		break;
	}

	// mark the debugger as active
	m_waiting_for_debugger = false;
}


void debugger_windows::debugger_update()
{
	// if we're running live, do some checks
	if (!winwindow_has_focus() && m_machine && !m_machine->debugger().cpu().is_stopped() && (m_machine->phase() == machine_phase::RUNNING))
	{
		// see if the interrupt key is pressed and break if it is
		if (seq_pressed())
		{
			HWND const focuswnd = GetFocus();

			m_machine->debugger().cpu().get_visible_cpu()->debug()->halt_on_next_instruction("User-initiated break\n");

			// if we were focused on some window's edit box, reset it to default
			for (auto &info : m_window_list)
				info->restore_field(focuswnd);
		}
	}
}


bool debugger_windows::seq_pressed() const
{
	input_seq const &seq = m_machine->ioport().type_seq(IPT_UI_DEBUG_BREAK);
	bool result = false;
	bool invert = false;
	bool first = true;

	// iterate over all of the codes
	for (int codenum = 0, length = seq.length(); codenum < length; codenum++)
	{
		input_code code = seq[codenum];

		if (code == input_seq::not_code)
		{
			// handle NOT
			invert = true;
		}
		else if (code == input_seq::or_code || code == input_seq::end_code)
		{
			// handle OR and END

			// if we have a positive result from the previous set, we're done
			if (result || code == input_seq::end_code)
				break;

			// otherwise, reset our state
			result = false;
			invert = false;
			first = true;
		}
		else
		{
			// handle everything else as a series of ANDs
			int const vkey = keyboard_trans_table::instance().vkey_for_mame_code(code);
			bool const pressed = (vkey != 0) && ((GetAsyncKeyState(vkey) & 0x8000) != 0);

			if (first)          // if this is the first in the sequence, result is set equal
				result = pressed ^ invert;
			else if (result)    // further values are ANDed
				result = result && (pressed ^ invert);

			// no longer first, and clear the invert flag
			first = invert = false;
		}
	}

	// return the result if we queried at least one switch
	return result;
}


void debugger_windows::remove_window(debugwin_info &info)
{
	for (auto it = m_window_list.begin(); it != m_window_list.end(); ++it)
		if (it->get() == &info) {
			m_window_list.erase(it);
			return;
		}
}


void debugger_windows::show_all()
{
	for (auto &info : m_window_list)
		info->show();
}


void debugger_windows::hide_all()
{
	SetForegroundWindow(std::static_pointer_cast<win_window_info>(osd_common_t::s_window_list.front())->platform_window());
	for (auto &info : m_window_list)
		info->hide();
}


template <typename T> T *debugger_windows::create_window()
{
	// allocate memory
	std::unique_ptr<T> info = std::make_unique<T>(static_cast<debugger_windows_interface &>(*this));
	if (info->is_valid())
	{
		m_window_list.push_back(std::move(info));
		T *ptr = dynamic_cast<T*>(m_window_list.back().get());
		return ptr;
	}
	return nullptr;
}


#else /* not windows */
MODULE_NOT_SUPPORTED(debugger_windows, OSD_DEBUG_PROVIDER, "windows")
#endif

MODULE_DEFINITION(DEBUG_WINDOWS, debugger_windows)
