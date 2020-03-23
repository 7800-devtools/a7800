// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/*********************************************************************

    dvbpoints.h

    Breakpoint debugger view.

***************************************************************************/

#ifndef MAME_EMU_DEBUG_DVBPOINTS_H
#define MAME_EMU_DEBUG_DVBPOINTS_H

#pragma once

#include "debugvw.h"
#include "debugcpu.h"


//**************************************************************************
//  CONSTANTS
//**************************************************************************


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// debug view for breakpoints
class debug_view_breakpoints : public debug_view
{
	friend class debug_view_manager;

	// construction/destruction
	debug_view_breakpoints(running_machine &machine, debug_view_osd_update_func osdupdate, void *osdprivate);
	virtual ~debug_view_breakpoints();

protected:
	// view overrides
	virtual void view_update() override;
	virtual void view_click(const int button, const debug_view_xy& pos) override;

private:
	// internal helpers
	void enumerate_sources();
	void pad_ostream_to_length(std::ostream& str, int len);
	void gather_breakpoints();


	// internal state
	int (*m_sortType)(void const *, void const *);
	std::vector<device_debug::breakpoint *> m_buffer;
};

#endif // MAME_EMU_DEBUG_DVBPOINTS_H
