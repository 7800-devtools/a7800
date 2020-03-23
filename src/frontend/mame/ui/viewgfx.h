// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria, Aaron Giles, Nathan Woods
/***************************************************************************

    ui/viewgfx.h

    Internal graphics viewer.

***************************************************************************/

#pragma once

#ifndef __UI_VIEWGFX_H__
#define __UI_VIEWGFX_H__



/***************************************************************************
    FUNCTION PROTOTYPES
***************************************************************************/

// initialization
void ui_gfx_init(running_machine &machine);

// returns 'true' if the internal graphics viewer has relevance
bool ui_gfx_is_relevant(running_machine &machine);

// master handler
uint32_t ui_gfx_ui_handler(render_container &container, mame_ui_manager &mui, bool uistate);


#endif  /* __UI_VIEWGFX_H__ */
