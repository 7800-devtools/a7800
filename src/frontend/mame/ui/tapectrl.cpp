// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria, Aaron Giles, Nathan Woods
/***************************************************************************

    ui/tapectrl.cpp

    Tape control

***************************************************************************/

#include "emu.h"

#include "ui/tapectrl.h"

namespace ui {
/***************************************************************************
    CONSTANTS
***************************************************************************/

#define TAPECMD_NULL            ((void *) 0x0000)
#define TAPECMD_STOP            ((void *) 0x0001)
#define TAPECMD_PLAY            ((void *) 0x0002)
#define TAPECMD_RECORD          ((void *) 0x0003)
#define TAPECMD_REWIND          ((void *) 0x0004)
#define TAPECMD_FAST_FORWARD    ((void *) 0x0005)
#define TAPECMD_SLIDER          ((void *) 0x0006)
#define TAPECMD_SELECT          ((void *) 0x0007)


/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

//-------------------------------------------------
//  ctor
//-------------------------------------------------

menu_tape_control::menu_tape_control(mame_ui_manager &mui, render_container &container, cassette_image_device *device)
	: menu_device_control<cassette_image_device>(mui, container, device)
{
}


//-------------------------------------------------
//  dtor
//-------------------------------------------------

menu_tape_control::~menu_tape_control()
{
}


//-------------------------------------------------
//  populate - populates the main tape control menu
//-------------------------------------------------

void menu_tape_control::populate(float &customtop, float &custombottom)
{
	if (current_device())
	{
		// name of tape
		item_append(current_display_name(), current_device()->exists() ? current_device()->filename() : "No Tape Image loaded", current_display_flags(), TAPECMD_SELECT);

		if (current_device()->exists())
		{
			std::string timepos;
			cassette_state state;
			double t0 = current_device()->get_position();
			double t1 = current_device()->get_length();
			uint32_t tapeflags = 0;

			// state
			if (t1 > 0)
			{
				if (t0 > 0)
					tapeflags |= FLAG_LEFT_ARROW;
				if (t0 < t1)
					tapeflags |= FLAG_RIGHT_ARROW;
			}

			get_time_string(timepos, current_device(), nullptr, nullptr);
			state = current_device()->get_state();
			item_append(
						(state & CASSETTE_MASK_UISTATE) == CASSETTE_STOPPED
						?   _("stopped")
						:   ((state & CASSETTE_MASK_UISTATE) == CASSETTE_PLAY
								? ((state & CASSETTE_MASK_MOTOR) == CASSETTE_MOTOR_ENABLED ? _("playing") : _("(playing)"))
								: ((state & CASSETTE_MASK_MOTOR) == CASSETTE_MOTOR_ENABLED ? _("recording") : _("(recording)"))
								),
								timepos.c_str(),
						tapeflags,
						TAPECMD_SLIDER);

			// pause or stop
			item_append(_("Pause/Stop"), "", 0, TAPECMD_STOP);

			// play
			item_append(_("Play"), "", 0, TAPECMD_PLAY);

			// record
			item_append(_("Record"), "", 0, TAPECMD_RECORD);

			// rewind
			item_append(_("Rewind"), "", 0, TAPECMD_REWIND);

			// fast forward
			item_append(_("Fast Forward"), "", 0, TAPECMD_FAST_FORWARD);
		}
	}
}


//-------------------------------------------------
//  handle - main tape control menu
//-------------------------------------------------

void menu_tape_control::handle()
{
	// rebuild the menu (so to update the selected device, if the user has pressed L or R, and the tape counter)
	repopulate(reset_options::REMEMBER_POSITION);

	// process the menu
	const event *event = process(PROCESS_LR_REPEAT);
	if (event != nullptr)
	{
		switch(event->iptkey)
		{
		case IPT_UI_LEFT:
			if (event->itemref == TAPECMD_SLIDER)
				current_device()->seek(-1, SEEK_CUR);
			else if (event->itemref == TAPECMD_SELECT)
				previous();
			break;

		case IPT_UI_RIGHT:
			if (event->itemref == TAPECMD_SLIDER)
				current_device()->seek(+1, SEEK_CUR);
			else if (event->itemref == TAPECMD_SELECT)
				next();
			break;

		case IPT_UI_SELECT:
			if (event->itemref == TAPECMD_STOP)
				current_device()->change_state(CASSETTE_STOPPED, CASSETTE_MASK_UISTATE);
			else if (event->itemref == TAPECMD_PLAY)
				current_device()->change_state(CASSETTE_PLAY, CASSETTE_MASK_UISTATE);
			else if (event->itemref == TAPECMD_RECORD)
				current_device()->change_state(CASSETTE_RECORD, CASSETTE_MASK_UISTATE);
			else if (event->itemref == TAPECMD_REWIND)
				current_device()->seek(-30, SEEK_CUR);
			else if (event->itemref == TAPECMD_FAST_FORWARD)
				current_device()->seek(30, SEEK_CUR);
			else if (event->itemref == TAPECMD_SLIDER)
				current_device()->seek(0, SEEK_SET);
			break;
		}
	}
}


//-------------------------------------------------
//  get_time_string - returns a textual
//  representation of the time
//-------------------------------------------------

void menu_tape_control::get_time_string(std::string &dest, cassette_image_device *cassette, int *curpos, int *endpos)
{
	double t0, t1;

	t0 = cassette->get_position();
	t1 = cassette->get_length();

	if (t1)
		dest = string_format("%04d/%04d", (int)t0, (int)t1);
	else
		dest = string_format("%04d/%04d", 0, (int)t1);

	if (curpos != nullptr)
		*curpos = t0;
	if (endpos != nullptr)
		*endpos = t1;
}

} // namespace ui
