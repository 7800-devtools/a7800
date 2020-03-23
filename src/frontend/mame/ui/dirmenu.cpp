// license:BSD-3-Clause
// copyright-holders:Maurizio Petrarota
/*********************************************************************

    ui/dirmenu.cpp

    Internal UI user interface.

*********************************************************************/

#include "emu.h"

#include "ui/ui.h"
#include "ui/dirmenu.h"
#include "ui/utils.h"
#include "ui/optsmenu.h"

#include "emuopts.h"
#include "mame.h"


namespace ui {
static int ADDING = 1;
static int CHANGE = 2;

struct folders_entry
{
	const char *name;
	const char *option;
	const int   action;
};

static const folders_entry s_folders[] =
{
	{ __("ROMs"),                OPTION_MEDIAPATH,          ADDING },
	{ __("Software Media"),      OPTION_SWPATH,             CHANGE },
	{ __("UI"),                  OPTION_UI_PATH,            CHANGE },
	{ __("Language"),            OPTION_LANGUAGEPATH,       CHANGE },
	{ __("Samples"),             OPTION_SAMPLEPATH,         ADDING },
	{ __("DATs"),                OPTION_HISTORY_PATH,       ADDING },
	{ __("INIs"),                OPTION_INIPATH,            ADDING },
	{ __("Extra INIs"),          OPTION_EXTRAINI_PATH,      CHANGE },
	{ __("Icons"),               OPTION_ICONS_PATH,         ADDING },
	{ __("Cheats"),              OPTION_CHEATPATH,          ADDING },
	{ __("Snapshots"),           OPTION_SNAPSHOT_DIRECTORY, ADDING },
	{ __("Cabinets"),            OPTION_CABINETS_PATH,      ADDING },
	{ __("Flyers"),              OPTION_FLYERS_PATH,        ADDING },
	{ __("Titles"),              OPTION_TITLES_PATH,        ADDING },
	{ __("Ends"),                OPTION_ENDS_PATH,          ADDING },
	{ __("PCBs"),                OPTION_PCBS_PATH,          ADDING },
	{ __("Marquees"),            OPTION_MARQUEES_PATH,      ADDING },
	{ __("Controls Panels"),     OPTION_CPANELS_PATH,       ADDING },
	{ __("Crosshairs"),          OPTION_CROSSHAIRPATH,      ADDING },
	{ __("Artworks"),            OPTION_ARTPATH,            ADDING },
	{ __("Bosses"),              OPTION_BOSSES_PATH,        ADDING },
	{ __("Artworks Preview"),    OPTION_ARTPREV_PATH,       ADDING },
	{ __("Select"),              OPTION_SELECT_PATH,        ADDING },
	{ __("GameOver"),            OPTION_GAMEOVER_PATH,      ADDING },
	{ __("HowTo"),               OPTION_HOWTO_PATH,         ADDING },
	{ __("Logos"),               OPTION_LOGOS_PATH,         ADDING },
	{ __("Scores"),              OPTION_SCORES_PATH,        ADDING },
	{ __("Versus"),              OPTION_VERSUS_PATH,        ADDING },
	{ __("Covers"),              OPTION_COVER_PATH,         ADDING }
};


/**************************************************
    MENU DIRECTORY
**************************************************/
//-------------------------------------------------
//  ctor / dtor
//-------------------------------------------------

menu_directory::menu_directory(mame_ui_manager &mui, render_container &container) : menu(mui, container)
{
}

menu_directory::~menu_directory()
{
	ui().save_ui_options();
	ui_globals::reset = true;
}

//-------------------------------------------------
//  handle
//-------------------------------------------------

void menu_directory::handle()
{
	// process the menu
	const event *menu_event = process(0);

	if (menu_event != nullptr && menu_event->itemref != nullptr && menu_event->iptkey == IPT_UI_SELECT)
		menu::stack_push<menu_display_actual>(ui(), container(), selected_index());
}

//-------------------------------------------------
//  populate
//-------------------------------------------------

void menu_directory::populate(float &customtop, float &custombottom)
{
	for (auto & elem : s_folders)
		item_append(_(elem.name), "", 0, (void *)(uintptr_t)elem.action);

	item_append(menu_item_type::SEPARATOR);
	customtop = ui().get_line_height() + 3.0f * UI_BOX_TB_BORDER;
}

//-------------------------------------------------
//  perform our special rendering
//-------------------------------------------------

void menu_directory::custom_render(void *selectedref, float top, float bottom, float origx1, float origy1, float origx2, float origy2)
{
	float width;

	// get the size of the text
	ui().draw_text_full(container(), _("Folders Setup"), 0.0f, 0.0f, 1.0f, ui::text_layout::CENTER, ui::text_layout::TRUNCATE,
		mame_ui_manager::NONE, rgb_t::white(), rgb_t::black(), &width, nullptr);
	width += (2.0f * UI_BOX_LR_BORDER) + 0.01f;
	float maxwidth = std::max(width, origx2 - origx1);

	// compute our bounds
	float x1 = 0.5f - 0.5f * maxwidth;
	float x2 = x1 + maxwidth;
	float y1 = origy1 - top;
	float y2 = origy1 - UI_BOX_TB_BORDER;

	// draw a box
	ui().draw_outlined_box(container(), x1, y1, x2, y2, UI_GREEN_COLOR);

	// take off the borders
	x1 += UI_BOX_LR_BORDER;
	x2 -= UI_BOX_LR_BORDER;
	y1 += UI_BOX_TB_BORDER;

	// draw the text within it
	ui().draw_text_full(container(), _("Folders Setup"), x1, y1, x2 - x1, ui::text_layout::CENTER, ui::text_layout::TRUNCATE,
		mame_ui_manager::NORMAL, UI_TEXT_COLOR, UI_TEXT_BG_COLOR, nullptr, nullptr);
}

/**************************************************
    MENU DISPLAY PATH
**************************************************/
//-------------------------------------------------
//  ctor / dtor
//-------------------------------------------------

menu_display_actual::menu_display_actual(mame_ui_manager &mui, render_container &container, int ref)
	: menu(mui, container), m_ref(ref)
{
}

menu_display_actual::~menu_display_actual()
{
}

//-------------------------------------------------
//  handle
//-------------------------------------------------

void menu_display_actual::handle()
{
	// process the menu
	const event *menu_event = process(0);
	if (menu_event != nullptr && menu_event->itemref != nullptr && menu_event->iptkey == IPT_UI_SELECT)
		switch ((uintptr_t)menu_event->itemref)
		{
		case REMOVE:
			menu::stack_push<menu_remove_folder>(ui(), container(), m_ref);
			break;

		case ADD_CHANGE:
			menu::stack_push<menu_add_change_folder>(ui(), container(), m_ref);
			break;
		}
}

//-------------------------------------------------
//  populate
//-------------------------------------------------

void menu_display_actual::populate(float &customtop, float &custombottom)
{
	m_tempbuf = string_format(_("Current %1$s Folders"), _(s_folders[m_ref].name));
	if (ui().options().exists(s_folders[m_ref].option))
		m_searchpath.assign(ui().options().value(s_folders[m_ref].option));
	else
		m_searchpath.assign(machine().options().value(s_folders[m_ref].option));

	path_iterator path(m_searchpath.c_str());
	std::string curpath;
	m_folders.clear();
	while (path.next(curpath, nullptr))
		m_folders.push_back(curpath);

	item_append((s_folders[m_ref].action == CHANGE) ? _("Change Folder") : _("Add Folder"), "", 0, (void *)ADD_CHANGE);

	if (m_folders.size() > 1)
		item_append(_("Remove Folder"), "", 0, (void *)REMOVE);

	item_append(menu_item_type::SEPARATOR);
	customtop = (m_folders.size() + 1) * ui().get_line_height() + 6.0f * UI_BOX_TB_BORDER;
}

//-------------------------------------------------
//  perform our special rendering
//-------------------------------------------------

void menu_display_actual::custom_render(void *selectedref, float top, float bottom, float origx1, float origy1, float origx2, float origy2)
{
	float width, maxwidth = origx2 - origx1;
	float lineh = ui().get_line_height();

	for (auto & elem : m_folders)
	{
		ui().draw_text_full(container(), elem.c_str(), 0.0f, 0.0f, 1.0f, ui::text_layout::LEFT, ui::text_layout::TRUNCATE, mame_ui_manager::NONE, rgb_t::white(), rgb_t::black(), &width, nullptr);
		width += (2.0f * UI_BOX_LR_BORDER) + 0.01f;
		maxwidth = std::max(maxwidth, width);
	}

	// get the size of the text
	ui().draw_text_full(container(), m_tempbuf.c_str(), 0.0f, 0.0f, 1.0f, ui::text_layout::CENTER, ui::text_layout::TRUNCATE, mame_ui_manager::NONE, rgb_t::white(), rgb_t::black(), &width, nullptr);
	width += (2.0f * UI_BOX_LR_BORDER) + 0.01f;
	maxwidth = std::max(width, maxwidth);

	// compute our bounds
	float x1 = 0.5f - 0.5f * maxwidth;
	float x2 = x1 + maxwidth;
	float y1 = origy1 - top;
	float y2 = y1 + lineh + 2.0f * UI_BOX_TB_BORDER;

	// draw a box
	ui().draw_outlined_box(container(), x1, y1, x2, y2, UI_GREEN_COLOR);

	// take off the borders
	x1 += UI_BOX_LR_BORDER;
	x2 -= UI_BOX_LR_BORDER;
	y1 += UI_BOX_TB_BORDER;

	// draw the text within it
	ui().draw_text_full(container(), m_tempbuf.c_str(), x1, y1, x2 - x1, ui::text_layout::CENTER, ui::text_layout::TRUNCATE,
		mame_ui_manager::NORMAL, UI_TEXT_COLOR, UI_TEXT_BG_COLOR, nullptr, nullptr);

	// compute our bounds
	x1 = 0.5f - 0.5f * maxwidth;
	x2 = x1 + maxwidth;
	y1 = y2 + 2.0f * UI_BOX_TB_BORDER;
	y2 = origy1 - UI_BOX_TB_BORDER;

	// draw a box
	ui().draw_outlined_box(container(), x1, y1, x2, y2, UI_BACKGROUND_COLOR);

	// take off the borders
	x1 += UI_BOX_LR_BORDER;
	x2 -= UI_BOX_LR_BORDER;
	y1 += UI_BOX_TB_BORDER;

	// draw the text within it
	for (auto & elem : m_folders)
	{
		ui().draw_text_full(container(), elem.c_str(), x1, y1, x2 - x1, ui::text_layout::LEFT, ui::text_layout::TRUNCATE,
			mame_ui_manager::NORMAL, UI_TEXT_COLOR, UI_TEXT_BG_COLOR, nullptr, nullptr);
		y1 += lineh;
	}

}

/**************************************************
MENU ADD FOLDER
**************************************************/
//-------------------------------------------------
//  ctor / dtor
//-------------------------------------------------

menu_add_change_folder::menu_add_change_folder(mame_ui_manager &mui, render_container &container, int ref) : menu(mui, container)
{
	m_ref = ref;
	m_change = (s_folders[ref].action == CHANGE);
	m_search.clear();

	// configure the starting path
	osd_get_full_path(m_current_path, ".");

	std::string searchpath;
	if (mui.options().exists(s_folders[m_ref].option))
		searchpath = mui.options().value(s_folders[m_ref].option);
	else
		searchpath = mui.machine().options().value(s_folders[m_ref].option);

	path_iterator path(searchpath.c_str());
	std::string curpath;
	while (path.next(curpath, nullptr))
		m_folders.push_back(curpath);
}

menu_add_change_folder::~menu_add_change_folder()
{
}

//-------------------------------------------------
//  handle
//-------------------------------------------------

void menu_add_change_folder::handle()
{
	// process the menu
	const event *menu_event = process(0);

	if (menu_event != nullptr && menu_event->itemref != nullptr)
	{
		if (menu_event->iptkey == IPT_UI_SELECT)
		{
			int index = (uintptr_t)menu_event->itemref - 1;
			const menu_item &pitem = item[index];

			// go up to the parent path
			if (!strcmp(pitem.text.c_str(), ".."))
			{
				size_t first_sep = m_current_path.find_first_of(PATH_SEPARATOR[0]);
				size_t last_sep = m_current_path.find_last_of(PATH_SEPARATOR[0]);
				if (first_sep != last_sep)
					m_current_path.erase(++last_sep);
			}
			else
			{
				// if isn't a drive, appends the directory
				if (strcmp(pitem.subtext.c_str(), "[DRIVE]") != 0)
				{
					if (m_current_path[m_current_path.length() - 1] == PATH_SEPARATOR[0])
						m_current_path.append(pitem.text);
					else
						m_current_path.append(PATH_SEPARATOR).append(pitem.text);
				}
				else
					m_current_path = pitem.text;
			}

			// reset the char buffer also in this case
			m_search.clear();
			reset(reset_options::SELECT_FIRST);
		}
		else if (menu_event->iptkey == IPT_SPECIAL)
		{
			bool update_selected = false;

			if (menu_event->unichar == 0x09)
			{
				// Tab key, save current path
				std::string error_string;
				if (m_change)
				{
					if (ui().options().exists(s_folders[m_ref].option))
						ui().options().set_value(s_folders[m_ref].option, m_current_path.c_str(), OPTION_PRIORITY_CMDLINE);
					else if (strcmp(machine().options().value(s_folders[m_ref].option), m_current_path.c_str()) != 0)
					{
						machine().options().set_value(s_folders[m_ref].option, m_current_path.c_str(), OPTION_PRIORITY_CMDLINE);
					}
				}
				else
				{
					m_folders.push_back(m_current_path);
					std::string tmppath;
					for (int x = 0; x < m_folders.size(); ++x)
					{
						tmppath.append(m_folders[x]);
						if (x != m_folders.size() - 1)
							tmppath.append(";");
					}

					if (ui().options().exists(s_folders[m_ref].option))
						ui().options().set_value(s_folders[m_ref].option, tmppath.c_str(), OPTION_PRIORITY_CMDLINE);
					else if (strcmp(machine().options().value(s_folders[m_ref].option), tmppath.c_str()) != 0)
					{
						machine().options().set_value(s_folders[m_ref].option, tmppath.c_str(), OPTION_PRIORITY_CMDLINE);
					}
				}

				reset_parent(reset_options::SELECT_FIRST);
				stack_pop();
			}
			else
			{
				// if it's any other key and we're not maxed out, update
				update_selected = input_character(m_search, menu_event->unichar, uchar_is_printable);
			}

			// check for entries which matches our search buffer
			if (update_selected)
			{
				const int cur_selected = selected_index();
				int entry, bestmatch = 0;

				// from current item to the end
				for (entry = cur_selected; entry < item.size(); entry++)
					if (item[entry].ref != nullptr && !m_search.empty())
					{
						int match = 0;
						for (int i = 0; i < m_search.size() + 1; i++)
						{
							if (core_strnicmp(item[entry].text.c_str(), m_search.data(), i) == 0)
								match = i;
						}

						if (match > bestmatch)
						{
							bestmatch = match;
							selected = entry;
						}
					}

				// and from the first item to current one
				for (entry = 0; entry < cur_selected; entry++)
				{
					if (item[entry].ref != nullptr && !m_search.empty())
					{
						int match = 0;
						for (int i = 0; i < m_search.size() + 1; i++)
						{
							if (core_strnicmp(item[entry].text.c_str(), m_search.data(), i) == 0)
								match = i;
						}

						if (match > bestmatch)
						{
							bestmatch = match;
							selected = entry;
						}
					}
				}
				centre_selection();
			}
		}
		else if (menu_event->iptkey == IPT_UI_CANCEL)
		{
			// reset the char buffer also in this case
			m_search.clear();
		}
	}
}

//-------------------------------------------------
//  populate
//-------------------------------------------------

void menu_add_change_folder::populate(float &customtop, float &custombottom)
{
	// open a path
	const char *volume_name = nullptr;
	file_enumerator path(m_current_path.c_str());
	const osd::directory::entry *dirent;
	int folders_count = 0;

	// add the drives
	for (int i = 0; (volume_name = osd_get_volume_name(i)) != nullptr; ++i)
		item_append(volume_name, "[DRIVE]", 0, (void *)(uintptr_t)++folders_count);

	// add the directories
	while ((dirent = path.next()) != nullptr)
	{
		if (dirent->type == osd::directory::entry::entry_type::DIR && strcmp(dirent->name, ".") != 0)
			item_append(dirent->name, "[DIR]", 0, (void *)(uintptr_t)++folders_count);
	}

	item_append(menu_item_type::SEPARATOR);

	// configure the custom rendering
	customtop = 2.0f * ui().get_line_height() + 3.0f * UI_BOX_TB_BORDER;
	custombottom = 1.0f * ui().get_line_height() + 3.0f * UI_BOX_TB_BORDER;
}

//-------------------------------------------------
//  perform our special rendering
//-------------------------------------------------

void menu_add_change_folder::custom_render(void *selectedref, float top, float bottom, float origx1, float origy1, float origx2, float origy2)
{
	float width, maxwidth = origx2 - origx1;
	std::string tempbuf[2];
	tempbuf[0] = string_format(
			(m_change)
				? _("Change %1$s Folder - Search: %2$s_")
				: _("Add %1$s Folder - Search: %2$s_"),
			_(s_folders[m_ref].name),
			m_search);
	tempbuf[1] = m_current_path;

	// get the size of the text
	for (auto & elem : tempbuf)
	{
		ui().draw_text_full(container(), elem.c_str(), 0.0f, 0.0f, 1.0f, ui::text_layout::CENTER, ui::text_layout::NEVER,
			mame_ui_manager::NONE, rgb_t::white(), rgb_t::black(), &width, nullptr);
		width += (2.0f * UI_BOX_LR_BORDER) + 0.01f;
		maxwidth = std::max(width, maxwidth);
	}

	// compute our bounds
	float x1 = 0.5f - 0.5f * maxwidth;
	float x2 = x1 + maxwidth;
	float y1 = origy1 - top;
	float y2 = origy1 - UI_BOX_TB_BORDER;

	// draw a box
	ui().draw_outlined_box(container(), x1, y1, x2, y2, UI_GREEN_COLOR);

	// take off the borders
	x1 += UI_BOX_LR_BORDER;
	x2 -= UI_BOX_LR_BORDER;
	y1 += UI_BOX_TB_BORDER;

	// draw the text within it
	for (auto & elem : tempbuf)
	{
		ui().draw_text_full(container(), elem.c_str(), x1, y1, x2 - x1, ui::text_layout::CENTER, ui::text_layout::NEVER,
			mame_ui_manager::NORMAL, UI_TEXT_COLOR, UI_TEXT_BG_COLOR, nullptr, nullptr);
		y1 = y1 + ui().get_line_height();
	}

	// bottom text
	tempbuf[0] = _("Press TAB to set");

	ui().draw_text_full(container(), tempbuf[0].c_str(), 0.0f, 0.0f, 1.0f, ui::text_layout::CENTER, ui::text_layout::TRUNCATE,
		mame_ui_manager::NONE, rgb_t::white(), rgb_t::black(), &width, nullptr);
	width += 2 * UI_BOX_LR_BORDER;
	maxwidth = std::max(maxwidth, width);

	// compute our bounds
	x1 = 0.5f - 0.5f * maxwidth;
	x2 = x1 + maxwidth;
	y1 = origy2 + UI_BOX_TB_BORDER;
	y2 = origy2 + bottom;

	// draw a box
	ui().draw_outlined_box(container(), x1, y1, x2, y2, UI_RED_COLOR);

	// take off the borders
	x1 += UI_BOX_LR_BORDER;
	x2 -= UI_BOX_LR_BORDER;
	y1 += UI_BOX_TB_BORDER;

	// draw the text within it
	ui().draw_text_full(container(), tempbuf[0].c_str(), x1, y1, x2 - x1, ui::text_layout::CENTER, ui::text_layout::TRUNCATE,
		mame_ui_manager::NORMAL, UI_TEXT_COLOR, UI_TEXT_BG_COLOR, nullptr, nullptr);

}

/**************************************************
    MENU REMOVE FOLDER
**************************************************/
//-------------------------------------------------
//  ctor / dtor
//-------------------------------------------------

menu_remove_folder::menu_remove_folder(mame_ui_manager &mui, render_container &container, int ref) : menu(mui, container)
{
	m_ref = ref;
	if (mui.options().exists(s_folders[m_ref].option))
		m_searchpath.assign(mui.options().value(s_folders[m_ref].option));
	else
		m_searchpath.assign(mui.machine().options().value(s_folders[m_ref].option));

	path_iterator path(m_searchpath.c_str());
	std::string curpath;
	while (path.next(curpath, nullptr))
		m_folders.push_back(curpath);
}

menu_remove_folder::~menu_remove_folder()
{
}

//-------------------------------------------------
//  handle
//-------------------------------------------------

void menu_remove_folder::handle()
{
	// process the menu
	const event *menu_event = process(0);
	if (menu_event != nullptr && menu_event->itemref != nullptr && menu_event->iptkey == IPT_UI_SELECT)
	{
		std::string tmppath, error_string;
		m_folders.erase(m_folders.begin() + selected_index());
		for (int x = 0; x < m_folders.size(); ++x)
		{
			tmppath.append(m_folders[x]);
			if (x < m_folders.size() - 1)
				tmppath.append(";");
		}

		if (ui().options().exists(s_folders[m_ref].option))
			ui().options().set_value(s_folders[m_ref].option, tmppath.c_str(), OPTION_PRIORITY_CMDLINE);
		else if (strcmp(machine().options().value(s_folders[m_ref].option),tmppath.c_str())!=0)
		{
			machine().options().set_value(s_folders[m_ref].option, tmppath.c_str(), OPTION_PRIORITY_CMDLINE);
		}

		reset_parent(reset_options::REMEMBER_REF);
		stack_pop();
	}
}

//-------------------------------------------------
//  populate menu
//-------------------------------------------------

void menu_remove_folder::populate(float &customtop, float &custombottom)
{
	int folders_count = 0;
	for (auto & elem : m_folders)
		item_append(elem, "", 0, (void *)(uintptr_t)++folders_count);

	item_append(menu_item_type::SEPARATOR);
	customtop = ui().get_line_height() + 3.0f * UI_BOX_TB_BORDER;
}

//-------------------------------------------------
//  perform our special rendering
//-------------------------------------------------

void menu_remove_folder::custom_render(void *selectedref, float top, float bottom, float origx1, float origy1, float origx2, float origy2)
{
	float width;
	std::string tempbuf = string_format(_("Remove %1$s Folder"), _(s_folders[m_ref].name));

	// get the size of the text
	ui().draw_text_full(container(), tempbuf.c_str(), 0.0f, 0.0f, 1.0f, ui::text_layout::CENTER, ui::text_layout::NEVER, mame_ui_manager::NONE, rgb_t::white(), rgb_t::black(), &width, nullptr);
	width += (2.0f * UI_BOX_LR_BORDER) + 0.01f;
	float maxwidth = std::max(width, origx2 - origx1);

	// compute our bounds
	float x1 = 0.5f - 0.5f * maxwidth;
	float x2 = x1 + maxwidth;
	float y1 = origy1 - top;
	float y2 = origy1 - UI_BOX_TB_BORDER;

	// draw a box
	ui().draw_outlined_box(container(), x1, y1, x2, y2, UI_GREEN_COLOR);

	// take off the borders
	x1 += UI_BOX_LR_BORDER;
	x2 -= UI_BOX_LR_BORDER;
	y1 += UI_BOX_TB_BORDER;

	// draw the text within it
	ui().draw_text_full(container(), tempbuf.c_str(), x1, y1, x2 - x1, ui::text_layout::CENTER, ui::text_layout::NEVER, mame_ui_manager::NORMAL,
		UI_TEXT_COLOR, UI_TEXT_BG_COLOR, nullptr, nullptr);
}

} // namespace ui
