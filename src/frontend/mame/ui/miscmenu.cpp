// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria, Aaron Giles, Nathan Woods, Maurizio Petrarota
/*********************************************************************

    ui/miscmenu.cpp

    Internal MAME menus for the user interface.

*********************************************************************/

#include "emu.h"
#include "mame.h"
#include "osdnet.h"
#include "mameopts.h"
#include "pluginopts.h"
#include "drivenum.h"
#include "natkeyboard.h"

#include "uiinput.h"
#include "ui/ui.h"
#include "ui/menu.h"
#include "ui/miscmenu.h"
#include "../info.h"
#include "ui/inifile.h"
#include "ui/submenu.h"

namespace ui {
/***************************************************************************
    MENU HANDLERS
***************************************************************************/

/*-------------------------------------------------
    menu_keyboard_mode - menu that
-------------------------------------------------*/

menu_keyboard_mode::menu_keyboard_mode(mame_ui_manager &mui, render_container &container) : menu(mui, container)
{
}

void menu_keyboard_mode::populate(float &customtop, float &custombottom)
{
	bool natural = machine().ioport().natkeyboard().in_use();
	item_append(_("Keyboard Mode:"), natural ? _("Natural") : _("Emulated"), natural ? FLAG_LEFT_ARROW : FLAG_RIGHT_ARROW, nullptr);
}

menu_keyboard_mode::~menu_keyboard_mode()
{
}

void menu_keyboard_mode::handle()
{
	bool natural = machine().ioport().natkeyboard().in_use();

	/* process the menu */
	const event *menu_event = process(0);

	if (menu_event != nullptr)
	{
		if (menu_event->iptkey == IPT_UI_LEFT || menu_event->iptkey == IPT_UI_RIGHT)
		{
			machine().ioport().natkeyboard().set_in_use(!natural);
			reset(reset_options::REMEMBER_REF);
		}
	}
}


/*-------------------------------------------------
    menu_bios_selection - populates the main
    bios selection menu
-------------------------------------------------*/

menu_bios_selection::menu_bios_selection(mame_ui_manager &mui, render_container &container) : menu(mui, container)
{
}

void menu_bios_selection::populate(float &customtop, float &custombottom)
{
	/* cycle through all devices for this system */
	for (device_t &device : device_iterator(machine().root_device()))
	{
		if (device.rom_region() != nullptr && !ROMENTRY_ISEND(device.rom_region()))
		{
			const char *val = "default";
			for (const rom_entry *rom = device.rom_region(); !ROMENTRY_ISEND(rom); rom++)
			{
				if (ROMENTRY_ISSYSTEM_BIOS(rom) && ROM_GETBIOSFLAGS(rom) == device.system_bios())
				{
					val = ROM_GETHASHDATA(rom);
				}
			}
			item_append(device.owner() == nullptr ? "driver" : device.tag()+1, val, FLAG_LEFT_ARROW | FLAG_RIGHT_ARROW, (void *)&device);
		}
	}

	item_append(menu_item_type::SEPARATOR);
	item_append(_("Reset"), "", 0, (void *)1);
}

menu_bios_selection::~menu_bios_selection()
{
}

/*-------------------------------------------------
    menu_bios_selection - menu that
-------------------------------------------------*/

void menu_bios_selection::handle()
{
	/* process the menu */
	const event *menu_event = process(0);

	if (menu_event != nullptr && menu_event->itemref != nullptr)
	{
		if ((uintptr_t)menu_event->itemref == 1 && menu_event->iptkey == IPT_UI_SELECT)
			machine().schedule_hard_reset();
		else if (menu_event->iptkey == IPT_UI_LEFT || menu_event->iptkey == IPT_UI_RIGHT)
		{
			device_t *dev = (device_t *)menu_event->itemref;
			int cnt = 0;
			for (const rom_entry &rom : dev->rom_region_vector())
			{
				if (ROMENTRY_ISSYSTEM_BIOS(&rom)) cnt ++;
			}
			int val = dev->system_bios() + ((menu_event->iptkey == IPT_UI_LEFT) ? -1 : +1);
			if (val<1) val=cnt;
			if (val>cnt) val=1;
			dev->set_system_bios(val);
			if (strcmp(dev->tag(),":")==0) {
				machine().options().set_value("bios", val-1, OPTION_PRIORITY_CMDLINE);
			} else {
				const char *slot_option_name = dev->owner()->tag() + 1;
				machine().options().slot_option(slot_option_name).set_bios(string_format("%d", val - 1));
			}
			reset(reset_options::REMEMBER_REF);
		}
	}
}



menu_network_devices::menu_network_devices(mame_ui_manager &mui, render_container &container) : menu(mui, container)
{
}

menu_network_devices::~menu_network_devices()
{
}

/*-------------------------------------------------
    menu_network_devices_populate - populates the main
    network device menu
-------------------------------------------------*/

void menu_network_devices::populate(float &customtop, float &custombottom)
{
	/* cycle through all devices for this system */
	for (device_network_interface &network : network_interface_iterator(machine().root_device()))
	{
		int curr = network.get_interface();
		const char *title = nullptr;
		for(auto &entry : get_netdev_list())
		{
			if(entry->id==curr) {
				title = entry->description;
				break;
			}
		}

		item_append(network.device().tag(),  (title) ? title : "------", FLAG_LEFT_ARROW | FLAG_RIGHT_ARROW, (void *)network);
	}
}

/*-------------------------------------------------
    menu_network_devices - menu that
-------------------------------------------------*/

void menu_network_devices::handle()
{
	/* process the menu */
	const event *menu_event = process(0);

	if (menu_event != nullptr && menu_event->itemref != nullptr)
	{
		if (menu_event->iptkey == IPT_UI_LEFT || menu_event->iptkey == IPT_UI_RIGHT) {
			device_network_interface *network = (device_network_interface *)menu_event->itemref;
			int curr = network->get_interface();
			if (menu_event->iptkey == IPT_UI_LEFT) curr--; else curr++;
			if (curr==-2) curr = netdev_count() - 1;
			network->set_interface(curr);
			reset(reset_options::REMEMBER_REF);
		}
	}
}


/*-------------------------------------------------
    menu_bookkeeping - handle the bookkeeping
    information menu
-------------------------------------------------*/

void menu_bookkeeping::handle()
{
	attotime curtime;

	/* if the time has rolled over another second, regenerate */
	curtime = machine().time();
	if (prevtime.seconds() != curtime.seconds())
	{
		prevtime = curtime;
		repopulate(reset_options::SELECT_FIRST);
	}

	/* process the menu */
	process(0);
}


/*-------------------------------------------------
    menu_bookkeeping - handle the bookkeeping
    information menu
-------------------------------------------------*/
menu_bookkeeping::menu_bookkeeping(mame_ui_manager &mui, render_container &container) : menu(mui, container)
{
}

menu_bookkeeping::~menu_bookkeeping()
{
}

void menu_bookkeeping::populate(float &customtop, float &custombottom)
{
	int tickets = machine().bookkeeping().get_dispensed_tickets();
	std::ostringstream tempstring;
	int ctrnum;

	/* show total time first */
	if (prevtime.seconds() >= (60 * 60))
		util::stream_format(tempstring, _("Uptime: %1$d:%2$02d:%3$02d\n\n"), prevtime.seconds() / (60 * 60), (prevtime.seconds() / 60) % 60, prevtime.seconds() % 60);
	else
		util::stream_format(tempstring, _("Uptime: %1$d:%2$02d\n\n"), (prevtime.seconds() / 60) % 60, prevtime.seconds() % 60);

	/* show tickets at the top */
	if (tickets > 0)
		util::stream_format(tempstring, _("Tickets dispensed: %1$d\n\n"), tickets);

	/* loop over coin counters */
	for (ctrnum = 0; ctrnum < bookkeeping_manager::COIN_COUNTERS; ctrnum++)
	{
		int count = machine().bookkeeping().coin_counter_get_count(ctrnum);

		/* display the coin counter number */
		/* display how many coins */
		/* display whether or not we are locked out */
		util::stream_format(tempstring,
				(count == 0) ? _("Coin %1$c: NA%3$s\n") : _("Coin %1$c: %2$d%3$s\n"),
				ctrnum + 'A',
				count,
				machine().bookkeeping().coin_lockout_get_state(ctrnum) ? _(" (locked)") : "");
	}

	/* append the single item */
	item_append(tempstring.str(), "", FLAG_MULTILINE, nullptr);
}

/*-------------------------------------------------
    menu_crosshair - handle the crosshair settings
    menu
-------------------------------------------------*/

void menu_crosshair::handle()
{
	/* process the menu */
	const event *menu_event = process(PROCESS_LR_REPEAT);

	/* handle events */
	if (menu_event != nullptr && menu_event->itemref != nullptr)
	{
		crosshair_item_data *data = (crosshair_item_data *)menu_event->itemref;
		bool changed = false;
		//int set_def = false;
		int newval = data->cur;

		/* retreive the user settings */
		render_crosshair &crosshair = machine().crosshair().get_crosshair(data->player);

		switch (menu_event->iptkey)
		{
			/* if selected, reset to default value */
			case IPT_UI_SELECT:
				newval = data->defvalue;
				//set_def = true;
				break;

			/* left decrements */
			case IPT_UI_LEFT:
				newval -= machine().input().code_pressed(KEYCODE_LSHIFT) ? 10 : 1;
				break;

			/* right increments */
			case IPT_UI_RIGHT:
				newval += machine().input().code_pressed(KEYCODE_LSHIFT) ? 10 : 1;
				break;
		}

		/* clamp to range */
		if (newval < data->min)
			newval = data->min;
		if (newval > data->max)
			newval = data->max;

		/* if things changed, update */
		if (newval != data->cur)
		{
			switch (data->type)
			{
				/* visibility state */
				case CROSSHAIR_ITEM_VIS:
					crosshair.set_mode(newval);
					// set visibility as specified by mode - auto mode starts with visibility off
					crosshair.set_visible(newval == CROSSHAIR_VISIBILITY_ON);
					changed = true;
					break;

				/* auto time */
				case CROSSHAIR_ITEM_AUTO_TIME:
					machine().crosshair().set_auto_time(newval);
					changed = true;
					break;
			}
		}

		/* crosshair graphic name */
		if (data->type == CROSSHAIR_ITEM_PIC)
		{
			switch (menu_event->iptkey)
			{
				case IPT_UI_SELECT:
					crosshair.set_default_bitmap();
					changed = true;
					break;

				case IPT_UI_LEFT:
					crosshair.set_bitmap_name(data->last_name);
					changed = true;
					break;

				case IPT_UI_RIGHT:
					crosshair.set_bitmap_name(data->next_name);
					changed = true;
					break;
			}
		}

		if (changed)
		{
			/* rebuild the menu */
			reset(reset_options::REMEMBER_POSITION);
		}
	}
}


/*-------------------------------------------------
    menu_crosshair_populate - populate the
    crosshair settings menu
-------------------------------------------------*/

menu_crosshair::menu_crosshair(mame_ui_manager &mui, render_container &container) : menu(mui, container)
{
}

void menu_crosshair::populate(float &customtop, float &custombottom)
{
	crosshair_item_data *data;
	char temp_text[16];
	int player;
	uint8_t use_auto = false;
	uint32_t flags = 0;

	/* loop over player and add the manual items */
	for (player = 0; player < MAX_PLAYERS; player++)
	{
		/* get the user settings */
		render_crosshair &crosshair = machine().crosshair().get_crosshair(player);

		/* add menu items for usable crosshairs */
		if (crosshair.is_used())
		{
			/* Make sure to keep these matched to the CROSSHAIR_VISIBILITY_xxx types */
			static const char *const vis_text[] = { "Off", "On", "Auto" };

			/* track if we need the auto time menu */
			if (crosshair.mode() == CROSSHAIR_VISIBILITY_AUTO) use_auto = true;

			/* CROSSHAIR_ITEM_VIS - allocate a data item and fill it */
			data = (crosshair_item_data *)m_pool_alloc(sizeof(*data));
			data->type = CROSSHAIR_ITEM_VIS;
			data->player = player;
			data->min = CROSSHAIR_VISIBILITY_OFF;
			data->max = CROSSHAIR_VISIBILITY_AUTO;
			data->defvalue = CROSSHAIR_VISIBILITY_DEFAULT;
			data->cur = crosshair.mode();

			/* put on arrows */
			if (data->cur > data->min)
				flags |= FLAG_LEFT_ARROW;
			if (data->cur < data->max)
				flags |= FLAG_RIGHT_ARROW;

			/* add CROSSHAIR_ITEM_VIS menu */
			sprintf(temp_text, "P%d Visibility", player + 1);
			item_append(temp_text, vis_text[crosshair.mode()], flags, data);

			/* CROSSHAIR_ITEM_PIC - allocate a data item and fill it */
			data = (crosshair_item_data *)m_pool_alloc(sizeof(*data));
			data->type = CROSSHAIR_ITEM_PIC;
			data->player = player;
			data->last_name[0] = 0;
			/* other data item not used by this menu */

			/* search for crosshair graphics */

			/* open a path to the crosshairs */
			file_enumerator path(machine().options().crosshair_path());
			const osd::directory::entry *dir;
			/* reset search flags */
			bool using_default = false;
			bool finished = false;
			bool found = false;

			/* if we are using the default, then we just need to find the first in the list */
			if (*crosshair.bitmap_name() == '\0')
				using_default = true;

			/* look for the current name, then remember the name before */
			/* and find the next name */
			while (((dir = path.next()) != nullptr) && !finished)
			{
				int length = strlen(dir->name);

				/* look for files ending in .png with a name not larger then 9 chars*/
				if ((length > 4) && (length <= CROSSHAIR_PIC_NAME_LENGTH + 4) && core_filename_ends_with(dir->name, ".png"))
				{
					/* remove .png from length */
					length -= 4;

					if (found || using_default)
					{
						/* get the next name */
						strncpy(data->next_name, dir->name, length);
						data->next_name[length] = 0;
						finished = true;
					}
					else if (!strncmp(dir->name, crosshair.bitmap_name(), length))
					{
						/* we found the current name */
						/* so loop once more to find the next name */
						found = true;
					}
					else
						/* remember last name */
						/* we will do it here in case files get added to the directory */
					{
						strncpy(data->last_name, dir->name, length);
						data->last_name[length] = 0;
					}
				}
			}
			/* if name not found then next item is DEFAULT */
			if (!found && !using_default)
			{
				data->next_name[0] = 0;
				finished = true;
			}
			/* setup the selection flags */
			flags = 0;
			if (finished)
				flags |= FLAG_RIGHT_ARROW;
			if (found)
				flags |= FLAG_LEFT_ARROW;

			/* add CROSSHAIR_ITEM_PIC menu */
			sprintf(temp_text, "P%d Crosshair", player + 1);
			item_append(temp_text, using_default ? "DEFAULT" : crosshair.bitmap_name(), flags, data);
		}
	}
	if (use_auto)
	{
		/* CROSSHAIR_ITEM_AUTO_TIME - allocate a data item and fill it */
		data = (crosshair_item_data *)m_pool_alloc(sizeof(*data));
		data->type = CROSSHAIR_ITEM_AUTO_TIME;
		data->min = CROSSHAIR_VISIBILITY_AUTOTIME_MIN;
		data->max = CROSSHAIR_VISIBILITY_AUTOTIME_MAX;
		data->defvalue = CROSSHAIR_VISIBILITY_AUTOTIME_DEFAULT;
		data->cur = machine().crosshair().auto_time();

		/* put on arrows in visible menu */
		if (data->cur > data->min)
			flags |= FLAG_LEFT_ARROW;
		if (data->cur < data->max)
			flags |= FLAG_RIGHT_ARROW;

		/* add CROSSHAIR_ITEM_AUTO_TIME menu */
		sprintf(temp_text, "%d", machine().crosshair().auto_time());
		item_append(_("Visible Delay"), temp_text, flags, data);
	}
//  else
//      /* leave a blank filler line when not in auto time so size does not rescale */
//      item_append("", "", nullptr, nullptr);
}

menu_crosshair::~menu_crosshair()
{
}

/*-------------------------------------------------
    menu_quit_game - handle the "menu" for
    quitting the game
-------------------------------------------------*/

menu_quit_game::menu_quit_game(mame_ui_manager &mui, render_container &container) : menu(mui, container)
{
}

menu_quit_game::~menu_quit_game()
{
}

void menu_quit_game::populate(float &customtop, float &custombottom)
{
}

void menu_quit_game::handle()
{
	/* request a reset */
	machine().schedule_exit();

	/* reset the menu stack */
	stack_reset();
}

//-------------------------------------------------
//  ctor / dtor
//-------------------------------------------------

menu_export::menu_export(mame_ui_manager &mui, render_container &container, std::vector<const game_driver *> &&drvlist)
	: menu(mui, container), m_list(std::move(drvlist))
{
}

menu_export::~menu_export()
{
}

//-------------------------------------------------
//  handlethe options menu
//-------------------------------------------------

void menu_export::handle()
{
	// process the menu
	process_parent();
	const event *menu_event = process(PROCESS_NOIMAGE);
	if (menu_event != nullptr && menu_event->itemref != nullptr)
	{
		switch (uintptr_t(menu_event->itemref))
		{
		case 1:
		case 3:
			if (menu_event->iptkey == IPT_UI_SELECT)
			{
				std::string filename("exported");
				emu_file infile(ui().options().ui_path(), OPEN_FLAG_READ);
				if (infile.open(filename.c_str(), ".xml") == osd_file::error::NONE)
					for (int seq = 0; ; ++seq)
					{
						std::string seqtext = string_format("%s_%04d", filename, seq);
						if (infile.open(seqtext.c_str(), ".xml") != osd_file::error::NONE)
						{
							filename = seqtext;
							break;
						}
					}

				// attempt to open the output file
				emu_file file(ui().options().ui_path(), OPEN_FLAG_WRITE | OPEN_FLAG_CREATE | OPEN_FLAG_CREATE_PATHS);
				if (file.open(filename.c_str(), ".xml") == osd_file::error::NONE)
				{
					FILE *pfile;
					std::string fullpath(file.fullpath());
					file.close();
					pfile = fopen(fullpath.c_str(), "w");

					// create the XML and save to file
					driver_enumerator drvlist(machine().options());
					drvlist.exclude_all();
					for (auto & elem : m_list)
						drvlist.include(driver_list::find(*elem));

					info_xml_creator creator(machine().options());
					creator.output(pfile, drvlist, (uintptr_t(menu_event->itemref) == 1) ? false : true);
					fclose(pfile);
					machine().popmessage(_("%s.xml saved under ui folder."), filename.c_str());
				}
			}
			break;
		case 2:
			if (menu_event->iptkey == IPT_UI_SELECT)
			{
				std::string filename("exported");
				emu_file infile(ui().options().ui_path(), OPEN_FLAG_READ);
				if (infile.open(filename.c_str(), ".txt") == osd_file::error::NONE)
					for (int seq = 0; ; ++seq)
					{
						std::string seqtext = string_format("%s_%04d", filename, seq);
						if (infile.open(seqtext.c_str(), ".txt") != osd_file::error::NONE)
						{
							filename = seqtext;
							break;
						}
					}

				// attempt to open the output file
				emu_file file(ui().options().ui_path(), OPEN_FLAG_WRITE | OPEN_FLAG_CREATE | OPEN_FLAG_CREATE_PATHS);
				if (file.open(filename.c_str(), ".txt") == osd_file::error::NONE)
				{
					// print the header
					std::ostringstream buffer;
					buffer << _("Name:             Description:\n");
					driver_enumerator drvlist(machine().options());
					drvlist.exclude_all();
					for (auto & elem : m_list)
						drvlist.include(driver_list::find(*elem));

					// iterate through drivers and output the info
					while (drvlist.next())
						if ((drvlist.driver().flags & MACHINE_NO_STANDALONE) == 0)
							util::stream_format(buffer, "%-18s\"%s\"\n", drvlist.driver().name, drvlist.driver().type.fullname());
					file.puts(buffer.str().c_str());
					file.close();
					machine().popmessage(_("%s.txt saved under ui folder."), filename.c_str());
				}
			}
			break;
		default:
			break;
		}
	}
}

//-------------------------------------------------
//  populate
//-------------------------------------------------

void menu_export::populate(float &customtop, float &custombottom)
{
	// add options items
	item_append(_("Export list in XML format (like -listxml)"), "", 0, (void *)(uintptr_t)1);
	item_append(_("Export list in XML format (like -listxml, but exclude devices)"), "", 0, (void *)(uintptr_t)3);
	item_append(_("Export list in TXT format (like -listfull)"), "", 0, (void *)(uintptr_t)2);
	item_append(menu_item_type::SEPARATOR);
}

//-------------------------------------------------
//  ctor / dtor
//-------------------------------------------------

menu_machine_configure::menu_machine_configure(mame_ui_manager &mui, render_container &container, const game_driver *prev, float _x0, float _y0)
	: menu(mui, container)
	, m_drv(prev)
	, x0(_x0)
	, y0(_y0)
	, m_curbios(0)
	, m_fav_reset(false)
{
	// parse the INI file
	std::ostringstream error;
	mame_options::parse_standard_inis(m_opts, error, m_drv);
	setup_bios();
}

menu_machine_configure::~menu_machine_configure()
{
	if (m_fav_reset)
		reset_topmost(reset_options::SELECT_FIRST);
}

//-------------------------------------------------
//  handlethe options menu
//-------------------------------------------------

void menu_machine_configure::handle()
{
	// process the menu
	process_parent();
	const event *menu_event = process(PROCESS_NOIMAGE, x0, y0);
	if (menu_event != nullptr && menu_event->itemref != nullptr)
	{
		if (menu_event->iptkey == IPT_UI_SELECT)
		{
			switch ((uintptr_t)menu_event->itemref)
			{
				case SAVE:
				{
					std::string filename(m_drv->name);
					emu_file file(machine().options().ini_path(), OPEN_FLAG_WRITE | OPEN_FLAG_CREATE);
					osd_file::error filerr = file.open(filename.c_str(), ".ini");
					if (filerr == osd_file::error::NONE)
					{
						std::string inistring = m_opts.output_ini();
						file.puts(inistring.c_str());
						ui().popup_time(2, "%s", _("\n    Configuration saved    \n\n"));
					}
					break;
				}
				case ADDFAV:
					mame_machine_manager::instance()->favorite().add_favorite_game(m_drv);
					reset(reset_options::REMEMBER_POSITION);
					break;
				case DELFAV:
					mame_machine_manager::instance()->favorite().remove_favorite_game();
					if (main_filters::actual == FILTER_FAVORITE)
					{
						m_fav_reset = true;
						menu::stack_pop();
					}
					else
						reset(reset_options::REMEMBER_POSITION);
					break;
				case CONTROLLER:
					if (menu_event->iptkey == IPT_UI_SELECT)
						menu::stack_push<submenu>(ui(), container(), submenu::control_options, m_drv, &m_opts);
					break;
				case VIDEO:
					if (menu_event->iptkey == IPT_UI_SELECT)
						menu::stack_push<submenu>(ui(), container(), submenu::video_options, m_drv, &m_opts);
					break;
				case ADVANCED:
					if (menu_event->iptkey == IPT_UI_SELECT)
						menu::stack_push<submenu>(ui(), container(), submenu::advanced_options, m_drv, &m_opts);
					break;
				default:
					break;
			}
		}
		else if (menu_event->iptkey == IPT_UI_LEFT || menu_event->iptkey == IPT_UI_RIGHT)
		{
			(menu_event->iptkey == IPT_UI_LEFT) ? --m_curbios : ++m_curbios;
			m_opts.set_value(OPTION_BIOS, m_bios[m_curbios].second, OPTION_PRIORITY_CMDLINE);
			reset(reset_options::REMEMBER_POSITION);
		}
	}
}

//-------------------------------------------------
//  populate
//-------------------------------------------------

void menu_machine_configure::populate(float &customtop, float &custombottom)
{
	// add options items
	item_append(_("Bios"), "", FLAG_DISABLE | FLAG_UI_HEADING, nullptr);
	if (!m_bios.empty())
	{
		uint32_t arrows = get_arrow_flags(std::size_t(0), m_bios.size() - 1, m_curbios);
		item_append(_("Driver"), m_bios[m_curbios].first, arrows, (void *)(uintptr_t)BIOS);
	}
	else
		item_append(_("This machine has no bios."), "", FLAG_DISABLE, nullptr);

	item_append(menu_item_type::SEPARATOR);
	item_append(_(submenu::advanced_options[0].description), "", 0, (void *)(uintptr_t)ADVANCED);
	item_append(_(submenu::video_options[0].description), "", 0, (void *)(uintptr_t)VIDEO);
	item_append(_(submenu::control_options[0].description), "", 0, (void *)(uintptr_t)CONTROLLER);
	item_append(menu_item_type::SEPARATOR);

	if (!mame_machine_manager::instance()->favorite().isgame_favorite(m_drv))
		item_append(_("Add To Favorites"), "", 0, (void *)ADDFAV);
	else
		item_append(_("Remove From Favorites"), "", 0, (void *)DELFAV);

	item_append(menu_item_type::SEPARATOR);
	item_append(_("Save machine configuration"), "", 0, (void *)(uintptr_t)SAVE);
	item_append(menu_item_type::SEPARATOR);
	customtop = 2.0f * ui().get_line_height() + 3.0f * UI_BOX_TB_BORDER;
}

//-------------------------------------------------
//  perform our special rendering
//-------------------------------------------------

void menu_machine_configure::custom_render(void *selectedref, float top, float bottom, float origx1, float origy1, float origx2, float origy2)
{
	float width;
	std::string text[2];
	float maxwidth = origx2 - origx1;

	text[0] = _("Configure machine:");
	text[1] = m_drv->type.fullname();

	for (auto & elem : text)
	{
		ui().draw_text_full(container(), elem.c_str(), 0.0f, 0.0f, 1.0f, ui::text_layout::CENTER, ui::text_layout::TRUNCATE,
			mame_ui_manager::NONE, rgb_t::white(), rgb_t::black(), &width, nullptr);
		width += 2 * UI_BOX_LR_BORDER;
		maxwidth = std::max(maxwidth, width);
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
	for (auto & elem : text)
	{
		ui().draw_text_full(container(), elem.c_str(), x1, y1, x2 - x1, ui::text_layout::CENTER, ui::text_layout::TRUNCATE,
			mame_ui_manager::NORMAL, UI_TEXT_COLOR, UI_TEXT_BG_COLOR, nullptr, nullptr);
		y1 += ui().get_line_height();
	}
}

void menu_machine_configure::setup_bios()
{
	if (m_drv->rom == nullptr)
		return;

	auto entries = rom_build_entries(m_drv->rom);

	std::string specbios(m_opts.bios());
	std::string default_name;
	for (const rom_entry &rom : entries)
		if (ROMENTRY_ISDEFAULT_BIOS(&rom))
			default_name = ROM_GETNAME(&rom);

	std::size_t bios_count = 0;
	for (const rom_entry &rom : entries)
	{
		if (ROMENTRY_ISSYSTEM_BIOS(&rom))
		{
			std::string name(ROM_GETHASHDATA(&rom));
			std::string biosname(ROM_GETNAME(&rom));
			int bios_flags = ROM_GETBIOSFLAGS(&rom);
			std::string bios_number = std::to_string(bios_flags - 1);

			// check biosnumber and name
			if (bios_number == specbios || biosname == specbios)
				m_curbios = bios_count;

			if (biosname == default_name)
			{
				name.append(_(" (default)"));
				if (specbios == "default")
					m_curbios = bios_count;
			}

			m_bios.emplace_back(name, bios_flags - 1);
			bios_count++;
		}
	}

}

//-------------------------------------------------
//  ctor / dtor
//-------------------------------------------------

menu_plugins_configure::menu_plugins_configure(mame_ui_manager &mui, render_container &container)
	: menu(mui, container)
{
}

menu_plugins_configure::~menu_plugins_configure()
{
	emu_file file_plugin(machine().options().ini_path(), OPEN_FLAG_WRITE | OPEN_FLAG_CREATE | OPEN_FLAG_CREATE_PATHS);
	if (file_plugin.open("plugin.ini") != osd_file::error::NONE)
		// Can't throw in a destructor, so let's ignore silently for
		// now.  We shouldn't write files in a destructor in any case.
		//
		// throw emu_fatalerror("Unable to create file plugin.ini\n");
		return;
	// generate the updated INI
	file_plugin.puts(mame_machine_manager::instance()->plugins().output_ini().c_str());
}

//-------------------------------------------------
//  handlethe options menu
//-------------------------------------------------

void menu_plugins_configure::handle()
{
	// process the menu
	bool changed = false;
	plugin_options& plugins = mame_machine_manager::instance()->plugins();
	process_parent();
	const event *menu_event = process(PROCESS_NOIMAGE);
	if (menu_event != nullptr && menu_event->itemref != nullptr)
	{
		if (menu_event->iptkey == IPT_UI_LEFT || menu_event->iptkey == IPT_UI_RIGHT || menu_event->iptkey == IPT_UI_SELECT)
		{
			int oldval = plugins.int_value((const char*)menu_event->itemref);
			plugins.set_value((const char*)menu_event->itemref, oldval == 1 ? 0 : 1, OPTION_PRIORITY_CMDLINE);
			changed = true;
		}
	}
	if (changed)
		reset(reset_options::REMEMBER_REF);
}

//-------------------------------------------------
//  populate
//-------------------------------------------------

void menu_plugins_configure::populate(float &customtop, float &custombottom)
{
	plugin_options& plugins = mame_machine_manager::instance()->plugins();

	for (auto &curentry : plugins.entries())
	{
		if (curentry->type() != OPTION_HEADER)
		{
			auto enabled = !strcmp(curentry->value(), "1");
			item_append(curentry->description(), enabled ? _("On") : _("Off"),
				enabled ? FLAG_RIGHT_ARROW : FLAG_LEFT_ARROW, (void *)(uintptr_t)curentry->name().c_str());
		}
	}
	item_append(menu_item_type::SEPARATOR);
	customtop = ui().get_line_height() + (3.0f * UI_BOX_TB_BORDER);
}

//-------------------------------------------------
//  perform our special rendering
//-------------------------------------------------

void menu_plugins_configure::custom_render(void *selectedref, float top, float bottom, float origx1, float origy1, float origx2, float origy2)
{
	float width;

	ui().draw_text_full(container(), _("Plugins"), 0.0f, 0.0f, 1.0f, ui::text_layout::CENTER, ui::text_layout::TRUNCATE,
		mame_ui_manager::NONE, rgb_t::white(), rgb_t::black(), &width, nullptr);
	width += 2 * UI_BOX_LR_BORDER;
	float maxwidth = std::max(origx2 - origx1, width);

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
	ui().draw_text_full(container(), _("Plugins"), x1, y1, x2 - x1, ui::text_layout::CENTER, ui::text_layout::TRUNCATE,
		mame_ui_manager::NORMAL, UI_TEXT_COLOR, UI_TEXT_BG_COLOR, nullptr, nullptr);
}

} // namespace ui
