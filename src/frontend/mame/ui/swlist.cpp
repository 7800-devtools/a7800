// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria, Aaron Giles, Nathan Woods
/*********************************************************************

    ui/swlist.cpp

    Internal MAME user interface for software list.

 *********************************************************************/

#include "emu.h"

#include "ui/ui.h"
#include "ui/swlist.h"
#include "ui/utils.h"

#include "softlist_dev.h"


namespace ui {
/***************************************************************************
    CONSTANTS
***************************************************************************/

// time (in seconds) to display errors
#define ERROR_MESSAGE_TIME      5

// item reference for "Switch Item Ordering"
#define ITEMREF_SWITCH_ITEM_ORDERING    ((void *)1)


/***************************************************************************
    SOFTWARE PARTS
***************************************************************************/

//-------------------------------------------------
//  is_valid_softlist_part_char - returns whether
//  this character is a valid char for a softlist
//  part
//-------------------------------------------------

static bool is_valid_softlist_part_char(char32_t ch)
{
	return (ch == (char)ch) && isalnum(ch);
}


//-------------------------------------------------
//  ctor
//-------------------------------------------------

menu_software_parts::menu_software_parts(mame_ui_manager &mui, render_container &container, const software_info *info, const char *interface, const software_part **part, bool other_opt, result &result)
	: menu(mui, container),
		m_result(result)
{
	m_info = info;
	m_interface = interface;
	m_selected_part = part;
	m_other_opt = other_opt;
}


//-------------------------------------------------
//  dtor
//-------------------------------------------------

menu_software_parts::~menu_software_parts()
{
}


//-------------------------------------------------
//  populate
//-------------------------------------------------

void menu_software_parts::populate(float &customtop, float &custombottom)
{
	if (m_other_opt)
	{
		software_part_menu_entry *entry1 = (software_part_menu_entry *) m_pool_alloc(sizeof(*entry1));
		entry1->type = result::EMPTY;
		entry1->part = nullptr;
		//item_append(_("[empty slot]"), "", 0, entry1);

		software_part_menu_entry *entry2 = (software_part_menu_entry *) m_pool_alloc(sizeof(*entry2));
		entry2->type = result::FMGR;
		entry2->part = nullptr;
		item_append(_("[file manager]"), "", 0, entry2);


		software_part_menu_entry *entry3 = (software_part_menu_entry *) m_pool_alloc(sizeof(*entry3));
		entry3->type = result::SWLIST;
		entry3->part = nullptr;
		//item_append(_("[software list]"), "", 0, entry3);
		//item_append(_("[Add Hardware]"), "", 0, entry3);
	}

	for (const software_part &swpart : m_info->parts())
	{
		if (swpart.matches_interface(m_interface))
		{
			software_part_menu_entry *entry = (software_part_menu_entry *) m_pool_alloc(sizeof(*entry));
			// check if the available parts have specific part_id to be displayed (e.g. "Map Disc", "Bonus Disc", etc.)
			// if not, we simply display "part_name"; if yes we display "part_name (part_id)"
			std::string menu_part_name(swpart.name());
			if (swpart.feature("part_id") != nullptr)
				menu_part_name.append(" (").append(swpart.feature("part_id")).append(")");
			entry->type = result::ENTRY;
			entry->part = &swpart;
			item_append(m_info->shortname(), menu_part_name, 0, entry);
		}
	}
}


//-------------------------------------------------
//  handle
//-------------------------------------------------

void menu_software_parts::handle()
{
	// process the menu
	const event *event = process(0);

	if (event != nullptr && event->iptkey == IPT_UI_SELECT && event->itemref != nullptr)
	{
		software_part_menu_entry *entry = (software_part_menu_entry *) event->itemref;
		m_result = entry->type;
		*m_selected_part = entry->part;
		stack_pop();
	}
}


/***************************************************************************
    SOFTWARE LIST
***************************************************************************/

//-------------------------------------------------
//  ctor
//-------------------------------------------------

menu_software_list::menu_software_list(mame_ui_manager &mui, render_container &container, software_list_device *swlist, const char *interface, std::string &result)
	: menu(mui, container), m_result(result)
{
	m_swlist = swlist;
	m_interface = interface;
	m_ordered_by_shortname = true;
}


//-------------------------------------------------
//  dtor
//-------------------------------------------------

menu_software_list::~menu_software_list()
{
}


//-------------------------------------------------
//  compare_entries
//-------------------------------------------------

int menu_software_list::compare_entries(const entry_info &e1, const entry_info &e2, bool shortname)
{
	int result;
	const char *e1_basename = shortname ? e1.short_name.c_str() : e1.long_name.c_str();
	const char *e2_basename = shortname ? e2.short_name.c_str() : e2.long_name.c_str();

	result = core_stricmp(e1_basename, e2_basename);
	if (result == 0)
	{
		result = strcmp(e1_basename, e2_basename);
	}

	return result;
}


//-------------------------------------------------
//  append_software_entry - populate a specific list
//-------------------------------------------------

void menu_software_list::append_software_entry(const software_info &swinfo)
{
	entry_info entry;
	bool entry_updated = false;

	// check if at least one of the parts has the correct interface and add a menu entry only in this case
	for (const software_part &swpart : swinfo.parts())
	{
		if (swpart.matches_interface(m_interface) && m_swlist->is_compatible(swpart) == SOFTWARE_IS_COMPATIBLE)
		{
			entry_updated = true;
			entry.short_name.assign(swinfo.shortname());
			entry.long_name.assign(swinfo.longname());
			break;
		}
	}

	// skip this if no new entry has been allocated (e.g. if the software has no matching interface for this image device)
	if (entry_updated)
	{
		// find the end of the list
		auto iter = m_entrylist.begin();
		while (iter != m_entrylist.end() && compare_entries(entry, *iter, m_ordered_by_shortname) >= 0)
			++iter;

		m_entrylist.emplace(iter, std::move(entry));
	}
}


//-------------------------------------------------
//  populate
//-------------------------------------------------

void menu_software_list::populate(float &customtop, float &custombottom)
{
	// clear all entries before populating
	m_entrylist.clear();

	// build up the list of entries for the menu
	for (const software_info &swinfo : m_swlist->get_info())
		append_software_entry(swinfo);

	// add an entry to change ordering
	//item_append(_("Switch Item Ordering"), "", 0, ITEMREF_SWITCH_ITEM_ORDERING);

	// append all of the menu entries
	for (auto &entry : m_entrylist)
		item_append(entry.short_name, entry.long_name, 0, &entry);
}


//-------------------------------------------------
//  handle
//-------------------------------------------------

void menu_software_list::handle()
{
	const entry_info *selected_entry = nullptr;
	int bestmatch = 0;

	// process the menu
	const event *event = process(0);

	if (event && event->itemref)
	{
		if (event->itemref == ITEMREF_SWITCH_ITEM_ORDERING && event->iptkey == IPT_UI_SELECT)
		{
			m_ordered_by_shortname = !m_ordered_by_shortname;

			// reset the char buffer if we change ordering criterion
			m_filename_buffer.clear();

			// reload the menu with the new order
			reset(reset_options::REMEMBER_REF);
			machine().popmessage(_("Switched Order: entries now ordered by %s"), m_ordered_by_shortname ? _("shortname") : _("description"));
		}
		// handle selections
		else if (event->iptkey == IPT_UI_SELECT)
		{
			entry_info *info = (entry_info *) event->itemref;
			m_result = info->short_name;
			stack_pop();
		}
		else if (event->iptkey == IPT_SPECIAL)
		{
			if (input_character(m_filename_buffer, event->unichar, &is_valid_softlist_part_char))
			{
				// display the popup
				ui().popup_time(ERROR_MESSAGE_TIME, "%s", m_filename_buffer);

				// identify the selected entry
				entry_info const *const cur_selected = (uintptr_t(event->itemref) != 1)
						? reinterpret_cast<entry_info const *>(get_selection_ref())
						: nullptr;

				// loop through all entries
				for (auto &entry : m_entrylist)
				{
					// is this entry the selected entry?
					if (cur_selected != &entry)
					{
						auto &compare_name = m_ordered_by_shortname ? entry.short_name : entry.long_name;

						int match = 0;
						for (int i = 0; i < m_filename_buffer.size() + 1; i++)
						{
							if (core_strnicmp(compare_name.c_str(), m_filename_buffer.c_str(), i) == 0)
								match = i;
						}

						if (match > bestmatch)
						{
							bestmatch = match;
							selected_entry = &entry;
						}
					}
				}

				if (selected_entry != nullptr && selected_entry != cur_selected)
				{
					set_selection((void *)selected_entry);
					centre_selection();
				}
			}
		}
		else if (event->iptkey == IPT_UI_CANCEL)
		{
			// reset the char buffer also in this case
			m_filename_buffer.clear();
			m_result = m_filename_buffer;
			stack_pop();
		}
	}
}


/***************************************************************************
    SOFTWARE MENU - list of available software lists - i.e. cartridges,
    floppies
***************************************************************************/

//-------------------------------------------------
//  ctor
//-------------------------------------------------

menu_software::menu_software(mame_ui_manager &mui, render_container &container, const char *interface, software_list_device **result)
	: menu(mui, container)
{
	m_interface = interface;
	m_result = result;
}


//-------------------------------------------------
//  dtor
//-------------------------------------------------

menu_software::~menu_software()
{
}


//-------------------------------------------------
//  populate
//-------------------------------------------------

void menu_software::populate(float &customtop, float &custombottom)
{
	bool have_compatible = false;

	// Add original software lists for this system
	software_list_device_iterator iter(machine().config().root_device());
	for (software_list_device &swlistdev : iter)
		if (swlistdev.list_type() == SOFTWARE_LIST_ORIGINAL_SYSTEM)
			if (!swlistdev.get_info().empty() && m_interface != nullptr)
			{
				bool found = false;
				for (const software_info &swinfo : swlistdev.get_info())
					for (const software_part &swpart : swinfo.parts())
						if (swpart.matches_interface(m_interface))
						{
							found = true;
							break;
						}
				if (found)
					item_append(swlistdev.description(), "", 0, (void *)&swlistdev);
			}

	// add compatible software lists for this system
	for (software_list_device &swlistdev : iter)
		if (swlistdev.list_type() == SOFTWARE_LIST_COMPATIBLE_SYSTEM)
			if (!swlistdev.get_info().empty() && m_interface != nullptr)
			{
				bool found = false;
				for (const software_info &swinfo : swlistdev.get_info())
					for (const software_part &swpart : swinfo.parts())
						if (swpart.matches_interface(m_interface))
						{
							found = true;
							break;
						}
				if (found)
				{
					if (!have_compatible)
						item_append(_("[compatible lists]"), "", FLAG_DISABLE, nullptr);
					item_append(swlistdev.description(), "", 0, (void *)&swlistdev);
				}
				have_compatible = true;
			}
}


//-------------------------------------------------
//  handle
//-------------------------------------------------

void menu_software::handle()
{
	// process the menu
	const event *event = process(0);

	if (event != nullptr && event->iptkey == IPT_UI_SELECT)
	{
		//menu::stack_push<menu_software_list>(ui(), container(), (software_list_config *)event->itemref, image);
		*m_result = reinterpret_cast<software_list_device *>(event->itemref);
		stack_pop();
	}
}

} // namespace ui
