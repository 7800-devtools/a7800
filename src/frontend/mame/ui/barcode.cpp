// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/***************************************************************************

    ui/barcode.cpp

    "Barcode Reader" control

***************************************************************************/

#include "emu.h"

#include "ui/barcode.h"
#include "ui/ui.h"
#include "ui/utils.h"

namespace ui {
// itemrefs for key menu items
#define ITEMREF_NEW_BARCODE    ((void *) 0x0001)
#define ITEMREF_ENTER_BARCODE  ((void *) 0x0002)
#define ITEMREF_SELECT_READER  ((void *) 0x0003)


/**************************************************

 BARCODE READER MENU

 **************************************************/


//-------------------------------------------------
//  ctor
//-------------------------------------------------

menu_barcode_reader::menu_barcode_reader(mame_ui_manager &mui, render_container &container, barcode_reader_device *device)
	: menu_device_control<barcode_reader_device>(mui, container, device)
{
}


//-------------------------------------------------
//  dtor
//-------------------------------------------------

menu_barcode_reader::~menu_barcode_reader()
{
}

//-------------------------------------------------
//  populate - populates the barcode input menu
//-------------------------------------------------

void menu_barcode_reader::populate(float &customtop, float &custombottom)
{
	if (current_device())
	{
		std::string buffer;
		const char *new_barcode;

		// selected device
		item_append(current_display_name(), "", current_display_flags(), ITEMREF_SELECT_READER);

		// append the "New Barcode" item
		if (get_selection_ref() == ITEMREF_NEW_BARCODE)
		{
			buffer.append(m_barcode_buffer);
			new_barcode = buffer.c_str();
		}
		else
		{
			new_barcode = m_barcode_buffer.c_str();
		}

		item_append(_("New Barcode:"), new_barcode, 0, ITEMREF_NEW_BARCODE);

		// finish up the menu
		item_append(menu_item_type::SEPARATOR);
		item_append(_("Enter Code"), "", 0, ITEMREF_ENTER_BARCODE);

		customtop = ui().get_line_height() + 3.0f * UI_BOX_TB_BORDER;
	}
}


//-------------------------------------------------
//  handle - manages inputs in the barcode input menu
//-------------------------------------------------

void menu_barcode_reader::handle()
{
	// rebuild the menu (so to update the selected device, if the user has pressed L or R)
	repopulate(reset_options::REMEMBER_POSITION);

	// process the menu
	const event *event = process(PROCESS_LR_REPEAT);

	// process the event
	if (event)
	{
		// handle selections
		switch (event->iptkey)
		{
		case IPT_UI_LEFT:
			if (event->itemref == ITEMREF_SELECT_READER)
				previous();
			break;

		case IPT_UI_RIGHT:
			if (event->itemref == ITEMREF_SELECT_READER)
				next();
			break;

		case IPT_UI_SELECT:
			if (event->itemref == ITEMREF_ENTER_BARCODE)
			{
				std::string tmp_file(m_barcode_buffer);
				//printf("code %s\n", m_barcode_buffer);
				if (!current_device()->is_valid(tmp_file.length()))
					ui().popup_time(5, "%s", _("Barcode length invalid!"));
				else
				{
					current_device()->write_code(tmp_file.c_str(), tmp_file.length());
					// if sending was successful, reset char buffer
					m_barcode_buffer.clear();
					reset(reset_options::REMEMBER_POSITION);
				}
			}
			break;

		case IPT_SPECIAL:
			if (get_selection_ref() == ITEMREF_NEW_BARCODE)
			{
				if (input_character(m_barcode_buffer, event->unichar, uchar_is_digit))
					reset(reset_options::REMEMBER_POSITION);
			}
			break;

		case IPT_UI_CANCEL:
			// reset the char buffer also in this case
			m_barcode_buffer.clear();
			break;
		}
	}
}

} // namespace ui
