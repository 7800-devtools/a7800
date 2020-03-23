// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria, Aaron Giles, Nathan Woods
/*********************************************************************

    ui/inputmap.cpp

    Internal menus for input mappings.

*********************************************************************/

#include "emu.h"

#include "uiinput.h"
#include "ui/ui.h"
#include "ui/menu.h"
#include "ui/inputmap.h"


namespace ui {
/***************************************************************************
    CONSTANTS
***************************************************************************/

#define MAX_PHYSICAL_DIPS       10
#define MAX_INPUT_PORTS         32
#define MAX_BITS_PER_PORT       32

/* DIP switch rendering parameters */
#define DIP_SWITCH_HEIGHT       0.05f
#define DIP_SWITCH_SPACING      0.01f
#define SINGLE_TOGGLE_SWITCH_FIELD_WIDTH 0.025f
#define SINGLE_TOGGLE_SWITCH_WIDTH 0.020f
/* make the switch 80% of the width space and 1/2 of the switch height */
#define PERCENTAGE_OF_HALF_FIELD_USED 0.80f
#define SINGLE_TOGGLE_SWITCH_HEIGHT ((DIP_SWITCH_HEIGHT / 2) * PERCENTAGE_OF_HALF_FIELD_USED)



/*-------------------------------------------------
    menu_input_groups_populate - populate the
    input groups menu
-------------------------------------------------*/

menu_input_groups::menu_input_groups(mame_ui_manager &mui, render_container &container) : menu(mui, container)
{
}

void menu_input_groups::populate(float &customtop, float &custombottom)
{
	//int player;

	/* build up the menu */
	item_append(_("User Interface"), "", 0, (void *)(IPG_UI + 1));
	//for (player = 0; player < MAX_PLAYERS; player++)
	//{
	//	auto s = string_format("Player %d Controls", player + 1);
	//	item_append(s, "", 0, (void *)(uintptr_t)(IPG_PLAYER1 + player + 1));
	//}
	//item_append(_("Other Controls"), "", 0, (void *)(uintptr_t)(IPG_OTHER + 1));
}

menu_input_groups::~menu_input_groups()
{
}

/*-------------------------------------------------
    menu_input_groups - handle the input groups
    menu
-------------------------------------------------*/

void menu_input_groups::handle()
{
	/* process the menu */
	const event *menu_event = process(0);
	if (menu_event != nullptr && menu_event->iptkey == IPT_UI_SELECT)
		menu::stack_push<menu_input_general>(ui(), container(), int((long long)(menu_event->itemref)-1));
}



/*-------------------------------------------------
    menu_input_general - handle the general
    input menu
-------------------------------------------------*/

menu_input_general::menu_input_general(mame_ui_manager &mui, render_container &container, int _group) : menu_input(mui, container)
{
	group = _group;
}

void menu_input_general::populate(float &customtop, float &custombottom)
{
	input_item_data *itemlist = nullptr;
	int suborder[SEQ_TYPE_TOTAL];
	int sortorder = 1;

	/* create a mini lookup table for sort order based on sequence type */
	suborder[SEQ_TYPE_STANDARD] = 0;
	suborder[SEQ_TYPE_DECREMENT] = 1;
	suborder[SEQ_TYPE_INCREMENT] = 2;

	/* iterate over the input ports and add menu items */
	for (input_type_entry &entry : machine().ioport().types())

		/* add if we match the group and we have a valid name */
		if (entry.group() == group && entry.name() != nullptr && entry.name()[0] != 0)
		{
			input_seq_type seqtype;

			/* loop over all sequence types */
			sortorder++;
			for (seqtype = SEQ_TYPE_STANDARD; seqtype < SEQ_TYPE_TOTAL; ++seqtype)
			{
				/* build an entry for the standard sequence */
				input_item_data *item = (input_item_data *)m_pool_alloc(sizeof(*item));
				memset(item, 0, sizeof(*item));
				item->ref = &entry;
				if(pollingitem && pollingref == &entry && pollingseq == seqtype)
					pollingitem = item;
				item->seqtype = seqtype;
				item->seq = machine().ioport().type_seq(entry.type(), entry.player(), seqtype);
				item->defseq = &entry.defseq(seqtype);
				item->sortorder = sortorder * 4 + suborder[seqtype];
				item->type = ioport_manager::type_is_analog(entry.type()) ? (INPUT_TYPE_ANALOG + seqtype) : INPUT_TYPE_DIGITAL;
				item->is_optional = false;
				item->name = entry.name();
				item->owner_name = nullptr;
				item->next = itemlist;
				itemlist = item;

				/* stop after one, unless we're analog */
				if (item->type == INPUT_TYPE_DIGITAL)
					break;
			}
		}

	/* sort and populate the menu in a standard fashion */
	populate_and_sort(itemlist);
}

menu_input_general::~menu_input_general()
{
}

/*-------------------------------------------------
    menu_input_specific - handle the game-specific
    input menu
-------------------------------------------------*/

menu_input_specific::menu_input_specific(mame_ui_manager &mui, render_container &container) : menu_input(mui, container)
{
}

void menu_input_specific::populate(float &customtop, float &custombottom)
{
	input_item_data *itemlist = nullptr;
	int suborder[SEQ_TYPE_TOTAL];
	int port_count = 0;

	/* create a mini lookup table for sort order based on sequence type */
	suborder[SEQ_TYPE_STANDARD] = 0;
	suborder[SEQ_TYPE_DECREMENT] = 1;
	suborder[SEQ_TYPE_INCREMENT] = 2;

	/* iterate over the input ports and add menu items */
	for (auto &port : machine().ioport().ports())
	{
		port_count++;
		for (ioport_field &field : port.second->fields())
		{
			ioport_type_class type_class = field.type_class();

			/* add if we match the group and we have a valid name */
			if (field.enabled() && (type_class == INPUT_CLASS_CONTROLLER || type_class == INPUT_CLASS_MISC || type_class == INPUT_CLASS_KEYBOARD))
			{
				input_seq_type seqtype;
				uint32_t sortorder;

				/* determine the sorting order */
				if (type_class == INPUT_CLASS_CONTROLLER)
				{
					sortorder = (field.type() << 2) | (field.player() << 12);
					if (field.device().owner() != nullptr)
						sortorder |= (port_count & 0xfff) * 0x10000;
				}
				else
					sortorder = field.type() | 0xf000;

				/* loop over all sequence types */
				for (seqtype = SEQ_TYPE_STANDARD; seqtype < SEQ_TYPE_TOTAL; ++seqtype)
				{
					/* build an entry for the standard sequence */
					input_item_data *item = (input_item_data *)m_pool_alloc(sizeof(*item));
					memset(item, 0, sizeof(*item));
					item->ref = &field;
					item->seqtype = seqtype;
					if(pollingitem && pollingref == item->ref && pollingseq == seqtype)
						pollingitem = item;
					item->seq = field.seq(seqtype);
					item->defseq = &field.defseq(seqtype);
					item->sortorder = sortorder + suborder[seqtype];
					item->type = field.is_analog() ? (INPUT_TYPE_ANALOG + seqtype) : INPUT_TYPE_DIGITAL;
					item->is_optional = field.optional();
					item->name = field.name();
					item->owner_name = field.device().tag();
					item->next = itemlist;
					itemlist = item;

					/* stop after one, unless we're analog */
					if (item->type == INPUT_TYPE_DIGITAL)
						break;
				}
			}
		}
	}

	/* sort and populate the menu in a standard fashion */
	populate_and_sort(itemlist);
}

menu_input_specific::~menu_input_specific()
{
}

/*-------------------------------------------------
    menu_input - display a menu for inputs
-------------------------------------------------*/
menu_input::menu_input(mame_ui_manager &mui, render_container &container) : menu(mui, container), last_sortorder(0), record_next(false)
{
	pollingitem = nullptr;
	pollingref = nullptr;
	pollingseq = SEQ_TYPE_STANDARD;
}

menu_input::~menu_input()
{
}

/*-------------------------------------------------
    toggle_none_default - toggle between "NONE"
    and the default item
-------------------------------------------------*/

void menu_input::toggle_none_default(input_seq &selected_seq, input_seq &original_seq, const input_seq &selected_defseq)
{
	/* if we used to be "none", toggle to the default value */
	if (original_seq.length() == 0)
		selected_seq = selected_defseq;

	/* otherwise, toggle to "none" */
	else
		selected_seq.reset();
}

void menu_input::handle()
{
	input_item_data *seqchangeditem = nullptr;
	const event *menu_event;
	int invalidate = false;

	/* process the menu */
	menu_event = process((pollingitem != nullptr) ? PROCESS_NOKEYS : 0);

	/* if we are polling, handle as a special case */
	if (pollingitem != nullptr)
	{
		input_item_data *item = pollingitem;

		/* if UI_CANCEL is pressed, abort */
		if (machine().ui_input().pressed(IPT_UI_CANCEL))
		{
			pollingitem = nullptr;
			record_next = false;
			toggle_none_default(item->seq, starting_seq, *item->defseq);
			seqchangeditem = item;
		}

		/* poll again; if finished, update the sequence */
		if (machine().input().seq_poll())
		{
			pollingitem = nullptr;
			record_next = true;
			item->seq = machine().input().seq_poll_final();
			seqchangeditem = item;
		}
	}

	/* otherwise, handle the events */
	else if (menu_event != nullptr && menu_event->itemref != nullptr)
	{
		input_item_data *item = (input_item_data *)menu_event->itemref;
		switch (menu_event->iptkey)
		{
			/* an item was selected: begin polling */
			case IPT_UI_SELECT:
				pollingitem = item;
				last_sortorder = item->sortorder;
				starting_seq = item->seq;
				machine().input().seq_poll_start((item->type == INPUT_TYPE_ANALOG) ? ITEM_CLASS_ABSOLUTE : ITEM_CLASS_SWITCH, record_next ? &item->seq : nullptr);
				invalidate = true;
				break;

			/* if the clear key was pressed, reset the selected item */
			case IPT_UI_CLEAR:
				toggle_none_default(item->seq, item->seq, *item->defseq);
				record_next = false;
				seqchangeditem = item;
				break;
		}

		/* if the selection changed, reset the "record next" flag */
		if (item->sortorder != last_sortorder)
			record_next = false;
		last_sortorder = item->sortorder;
	}

	/* if the sequence changed, update it */
	if (seqchangeditem != nullptr)
	{
		update_input(seqchangeditem);

		/* invalidate the menu to force an update */
		invalidate = true;
	}

	/* if the menu is invalidated, clear it now */
	if (invalidate)
	{
		pollingref = nullptr;
		if (pollingitem != nullptr)
		{
			pollingref = pollingitem->ref;
			pollingseq = pollingitem->seqtype;
		}
		reset(reset_options::REMEMBER_POSITION);
	}
}

void menu_input_general::update_input(struct input_item_data *seqchangeditem)
{
	const input_type_entry *entry = (const input_type_entry *)seqchangeditem->ref;
	machine().ioport().set_type_seq(entry->type(), entry->player(), seqchangeditem->seqtype, seqchangeditem->seq);
}

void menu_input_specific::update_input(struct input_item_data *seqchangeditem)
{
	ioport_field::user_settings settings;

	((ioport_field *)seqchangeditem->ref)->get_user_settings(settings);
	settings.seq[seqchangeditem->seqtype] = seqchangeditem->seq;
	((ioport_field *)seqchangeditem->ref)->set_user_settings(settings);
}


/*-------------------------------------------------
    menu_input_compare_items - compare two
    items for quicksort
-------------------------------------------------*/

int menu_input::compare_items(const void *i1, const void *i2)
{
	const input_item_data * const *data1 = (const input_item_data * const *)i1;
	const input_item_data * const *data2 = (const input_item_data * const *)i2;
	if ((*data1)->sortorder < (*data2)->sortorder)
		return -1;
	if ((*data1)->sortorder > (*data2)->sortorder)
		return 1;
	return 0;
}


/*-------------------------------------------------
    menu_input_populate_and_sort - take a list
    of input_item_data objects and build up the
    menu from them
-------------------------------------------------*/

void menu_input::populate_and_sort(input_item_data *itemlist)
{
	const char *nameformat[INPUT_TYPE_TOTAL] = { nullptr };
	input_item_data **itemarray, *item;
	int numitems = 0, curitem;
	std::string subtext;
	std::string prev_owner;
	bool first_entry = true;

	/* create a mini lookup table for name format based on type */
	nameformat[INPUT_TYPE_DIGITAL] = "%s";
	nameformat[INPUT_TYPE_ANALOG] = "%s Analog";
	nameformat[INPUT_TYPE_ANALOG_INC] = "%s Analog Inc";
	nameformat[INPUT_TYPE_ANALOG_DEC] = "%s Analog Dec";

	/* first count the number of items */
	for (item = itemlist; item != nullptr; item = item->next)
		numitems++;

	/* now allocate an array of items and fill it up */
	itemarray = (input_item_data **)m_pool_alloc(sizeof(*itemarray) * numitems);
	for (item = itemlist, curitem = 0; item != nullptr; item = item->next)
		itemarray[curitem++] = item;

	/* sort it */
	qsort(itemarray, numitems, sizeof(*itemarray), compare_items);

	/* build the menu */
	for (curitem = 0; curitem < numitems; curitem++)
	{
		uint32_t flags = 0;

		/* generate the name of the item itself, based off the base name and the type */
		item = itemarray[curitem];
		assert(nameformat[item->type] != nullptr);

		if (item->owner_name && strcmp(item->owner_name, prev_owner.c_str()) != 0)
		{
			if (first_entry)
				first_entry = false;
			else
				item_append(menu_item_type::SEPARATOR);
			item_append(string_format("[root%s]", item->owner_name), "", 0, nullptr);
			prev_owner.assign(item->owner_name);
		}

		std::string text = string_format(nameformat[item->type], item->name);
		if (item->is_optional)
			text = "(" + text + ")";

		/* if we're polling this item, use some spaces with left/right arrows */
		if (pollingref == item->ref && pollingseq == item->seqtype)
		{
			subtext.assign("   ");
			flags |= FLAG_LEFT_ARROW | FLAG_RIGHT_ARROW;
		}

		/* otherwise, generate the sequence name and invert it if different from the default */
		else
		{
			subtext = machine().input().seq_name(item->seq);
			flags |= (item->seq != *item->defseq) ? FLAG_INVERT : 0;
		}

		/* add the item */
		item_append(text, subtext, flags, item);
	}
}


/*-------------------------------------------------
    menu_settings_dip_switches - handle the DIP
    switches menu
-------------------------------------------------*/

menu_settings_dip_switches::menu_settings_dip_switches(mame_ui_manager &mui, render_container &container) : menu_settings(mui, container, IPT_DIPSWITCH)
{
}

menu_settings_dip_switches::~menu_settings_dip_switches()
{
}

/*-------------------------------------------------
    menu_settings_driver_config - handle the
    driver config menu
-------------------------------------------------*/

menu_settings_driver_config::menu_settings_driver_config(mame_ui_manager &mui, render_container &container) : menu_settings(mui, container, IPT_CONFIG)
{
}

menu_settings_driver_config::~menu_settings_driver_config()
{
}

/*-------------------------------------------------
    menu_settings_common - handle one of the
    switches menus
-------------------------------------------------*/

void menu_settings::handle()
{
	// process the menu
	const event *menu_event = process(0);

	// handle events
	if (menu_event != nullptr && menu_event->itemref != nullptr)
	{
		// reset
		if ((uintptr_t)menu_event->itemref == 1)
		{
			if (menu_event->iptkey == IPT_UI_SELECT)
				machine().schedule_hard_reset();
		}
		// actual settings
		else
		{
			ioport_field *field = (ioport_field *)menu_event->itemref;
			ioport_field::user_settings settings;
			int changed = false;

			switch (menu_event->iptkey)
			{
			/* if selected, reset to default value */
			case IPT_UI_SELECT:
				field->get_user_settings(settings);
				settings.value = field->defvalue();
				field->set_user_settings(settings);
				changed = true;
				break;

			/* left goes to previous setting */
			case IPT_UI_LEFT:
				field->select_previous_setting();
				changed = true;
				break;

			/* right goes to next setting */
			case IPT_UI_RIGHT:
				field->select_next_setting();
				changed = true;
				break;
			}

			/* if anything changed, rebuild the menu, trying to stay on the same field */
			if (changed)
				reset(reset_options::REMEMBER_REF);
		}
	}
}


/*-------------------------------------------------
    menu_settings_populate - populate one of the
    switches menus
-------------------------------------------------*/

menu_settings::menu_settings(mame_ui_manager &mui, render_container &container, uint32_t _type) : menu(mui, container), diplist(nullptr), dipcount(0)
{
	type = _type;
}

void menu_settings::populate(float &customtop, float &custombottom)
{
	dip_descriptor **diplist_tailptr;
	std::string prev_owner;
	bool first_entry = true;

	/* reset the dip switch tracking */
	dipcount = 0;
	diplist = nullptr;
	diplist_tailptr = &diplist;

	/* loop over input ports and set up the current values */
	for (auto &port : machine().ioport().ports())
		for (ioport_field &field : port.second->fields())
			if (field.type() == type && field.enabled())
			{
				uint32_t flags = 0;

				/* set the left/right flags appropriately */
				if (field.has_previous_setting())
					flags |= FLAG_LEFT_ARROW;
				if (field.has_next_setting())
					flags |= FLAG_RIGHT_ARROW;

				/* add the menu item */
				if (strcmp(field.device().tag(), prev_owner.c_str()) != 0)
				{
					if (first_entry)
						first_entry = false;
					else
						item_append(menu_item_type::SEPARATOR);
					string_format("[root%s]", field.device().tag());
					item_append(string_format("[root%s]", field.device().tag()), "", 0, nullptr);
					prev_owner.assign(field.device().tag());
				}

				item_append(field.name(), field.setting_name(), flags, (void *)&field);

				/* for DIP switches, build up the model */
				if (type == IPT_DIPSWITCH && !field.diplocations().empty())
				{
					ioport_field::user_settings settings;
					uint32_t accummask = field.mask();

					/* get current settings */
					field.get_user_settings(settings);

					/* iterate over each bit in the field */
					for (const ioport_diplocation &diploc : field.diplocations())
					{
						uint32_t mask = accummask & ~(accummask - 1);
						dip_descriptor *dip;

						/* find the matching switch name */
						for (dip = diplist; dip != nullptr; dip = dip->next)
							if (strcmp(dip->name, diploc.name()) == 0)
								break;

						/* allocate new if none */
						if (dip == nullptr)
						{
							dip = (dip_descriptor *)m_pool_alloc(sizeof(*dip));
							dip->next = nullptr;
							dip->name = diploc.name();
							dip->mask = dip->state = 0;
							*diplist_tailptr = dip;
							diplist_tailptr = &dip->next;
							dipcount++;
						}

						/* apply the bits */
						dip->mask |= 1 << (diploc.number() - 1);
						if (((settings.value & mask) != 0 && !diploc.inverted()) || ((settings.value & mask) == 0 && diploc.inverted()))
							dip->state |= 1 << (diploc.number() - 1);

						/* clear the relevant bit in the accumulated mask */
						accummask &= ~mask;
					}
				}
			}
	if (type == IPT_DIPSWITCH)
		custombottom = dipcount ? dipcount * (DIP_SWITCH_HEIGHT + DIP_SWITCH_SPACING) + DIP_SWITCH_SPACING : 0;

	item_append(menu_item_type::SEPARATOR);
	item_append(_("Reset"), "", 0, (void *)1);
}

menu_settings::~menu_settings()
{
}

/*-------------------------------------------------
    menu_settings_custom_render - perform our special
    rendering
-------------------------------------------------*/

void menu_settings_dip_switches::custom_render(void *selectedref, float top, float bottom, float x1, float y1, float x2, float y2)
{
	// catch if no diploc has to be drawn
	if (bottom == 0)
		return;

	// add borders
	y1 = y2 + UI_BOX_TB_BORDER;
	y2 = y1 + bottom;

	// draw extra menu area
	ui().draw_outlined_box(container(), x1, y1, x2, y2, UI_BACKGROUND_COLOR);
	y1 += (float)DIP_SWITCH_SPACING;

	// iterate over DIP switches
	for (dip_descriptor *dip = diplist; dip != nullptr; dip = dip->next)
	{
		uint32_t selectedmask = 0;

		// determine the mask of selected bits
		if ((uintptr_t)selectedref != 1)
		{
			ioport_field *field = (ioport_field *)selectedref;

			if (field != nullptr && !field->diplocations().empty())
				for (const ioport_diplocation &diploc : field->diplocations())
					if (strcmp(dip->name, diploc.name()) == 0)
						selectedmask |= 1 << (diploc.number() - 1);
		}

		// draw one switch
		custom_render_one(x1, y1, x2, y1 + DIP_SWITCH_HEIGHT, dip, selectedmask);
		y1 += (float)(DIP_SWITCH_SPACING + DIP_SWITCH_HEIGHT);
	}
}


/*-------------------------------------------------
    menu_settings_custom_render_one - draw a single
    DIP switch
-------------------------------------------------*/

void menu_settings_dip_switches::custom_render_one(float x1, float y1, float x2, float y2, const dip_descriptor *dip, uint32_t selectedmask)
{
	float switch_field_width = SINGLE_TOGGLE_SWITCH_FIELD_WIDTH * container().manager().ui_aspect();
	float switch_width = SINGLE_TOGGLE_SWITCH_WIDTH * container().manager().ui_aspect();
	int numtoggles, toggle;
	float switch_toggle_gap;
	float y1_off, y1_on;

	/* determine the number of toggles in the DIP */
	numtoggles = 32 - count_leading_zeros(dip->mask);

	/* center based on the number of switches */
	x1 += (x2 - x1 - numtoggles * switch_field_width) / 2;

	/* draw the dip switch name */
	ui().draw_text_full(container(),
						dip->name,
						0,
						y1 + (DIP_SWITCH_HEIGHT - UI_TARGET_FONT_HEIGHT) / 2,
						x1 - ui().get_string_width(" "),
						ui::text_layout::RIGHT,
						ui::text_layout::NEVER,
						mame_ui_manager::NORMAL,
						UI_TEXT_COLOR,
						PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA),
						nullptr ,
						nullptr);

	/* compute top and bottom for on and off positions */
	switch_toggle_gap = ((DIP_SWITCH_HEIGHT/2) - SINGLE_TOGGLE_SWITCH_HEIGHT)/2;
	y1_off = y1 + UI_LINE_WIDTH + switch_toggle_gap;
	y1_on = y1 + DIP_SWITCH_HEIGHT/2 + switch_toggle_gap;

	/* iterate over toggles */
	for (toggle = 0; toggle < numtoggles; toggle++)
	{
		float innerx1;

		/* first outline the switch */
		ui().draw_outlined_box(container(), x1, y1, x1 + switch_field_width, y2, UI_BACKGROUND_COLOR);

		/* compute x1/x2 for the inner filled in switch */
		innerx1 = x1 + (switch_field_width - switch_width) / 2;

		/* see if the switch is actually used */
		if (dip->mask & (1 << toggle))
		{
			float innery1 = (dip->state & (1 << toggle)) ? y1_on : y1_off;
			container().add_rect(innerx1, innery1, innerx1 + switch_width, innery1 + SINGLE_TOGGLE_SWITCH_HEIGHT,
								(selectedmask & (1 << toggle)) ? UI_DIPSW_COLOR : UI_TEXT_COLOR,
								PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA));
		}
		else
		{
			container().add_rect(innerx1, y1_off, innerx1 + switch_width, y1_on + SINGLE_TOGGLE_SWITCH_HEIGHT,
								UI_UNAVAILABLE_COLOR,
								PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA));
		}

		/* advance to the next switch */
		x1 += switch_field_width;
	}
}


/*-------------------------------------------------
    menu_analog - handle the analog settings menu
-------------------------------------------------*/

void menu_analog::handle()
{
	/* process the menu */
	const event *menu_event = process(PROCESS_LR_REPEAT);

	/* handle events */
	if (menu_event != nullptr && menu_event->itemref != nullptr)
	{
		analog_item_data *data = (analog_item_data *)menu_event->itemref;
		int newval = data->cur;

		switch (menu_event->iptkey)
		{
			/* if selected, reset to default value */
			case IPT_UI_SELECT:
				newval = data->defvalue;
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
			ioport_field::user_settings settings;

			/* get the settings and set the new value */
			data->field->get_user_settings(settings);
			switch (data->type)
			{
				case ANALOG_ITEM_KEYSPEED:      settings.delta = newval;        break;
				case ANALOG_ITEM_CENTERSPEED:   settings.centerdelta = newval;  break;
				case ANALOG_ITEM_REVERSE:       settings.reverse = newval;      break;
				case ANALOG_ITEM_SENSITIVITY:   settings.sensitivity = newval;  break;
			}
			data->field->set_user_settings(settings);

			/* rebuild the menu */
			reset(reset_options::REMEMBER_POSITION);
		}
	}
}


/*-------------------------------------------------
    menu_analog_populate - populate the analog
    settings menu
-------------------------------------------------*/

menu_analog::menu_analog(mame_ui_manager &mui, render_container &container) : menu(mui, container)
{
}

void menu_analog::populate(float &customtop, float &custombottom)
{
	std::string prev_owner;
	bool first_entry = true;

	/* loop over input ports and add the items */
	for (auto &port : machine().ioport().ports())
		for (ioport_field &field : port.second->fields())
			if (field.is_analog() && field.enabled())
			{
				ioport_field::user_settings settings;
				int use_autocenter = false;
				int type;

				/* based on the type, determine if we enable autocenter */
				switch (field.type())
				{
					case IPT_POSITIONAL:
					case IPT_POSITIONAL_V:
						if (field.analog_wraps())
							break;

					case IPT_AD_STICK_X:
					case IPT_AD_STICK_Y:
					case IPT_AD_STICK_Z:
					case IPT_PADDLE:
					case IPT_PADDLE_V:
					case IPT_PEDAL:
					case IPT_PEDAL2:
					case IPT_PEDAL3:
						use_autocenter = true;
						break;

					default:
						break;
				}

				/* get the user settings */
				field.get_user_settings(settings);

				/* iterate over types */
				for (type = 0; type < ANALOG_ITEM_COUNT; type++)
					if (type != ANALOG_ITEM_CENTERSPEED || use_autocenter)
					{
						analog_item_data *data;
						uint32_t flags = 0;
						std::string text;
						std::string subtext;
						if (strcmp(field.device().tag(), prev_owner.c_str()) != 0)
						{
							if (first_entry)
								first_entry = false;
							else
								item_append(menu_item_type::SEPARATOR);
							item_append(string_format("[root%s]", field.device().tag()), "", 0, nullptr);
							prev_owner.assign(field.device().tag());
						}

						/* allocate a data item for tracking what this menu item refers to */
						data = (analog_item_data *)m_pool_alloc(sizeof(*data));
						data->field = &field;
						data->type = type;

						/* determine the properties of this item */
						switch (type)
						{
							default:
							case ANALOG_ITEM_KEYSPEED:
								text = string_format("%s Digital Speed", field.name());
								subtext = string_format("%d", settings.delta);
								data->min = 0;
								data->max = 255;
								data->cur = settings.delta;
								data->defvalue = field.delta();
								break;

							case ANALOG_ITEM_CENTERSPEED:
								text = string_format("%s Autocenter Speed", field.name());
								subtext = string_format("%d", settings.centerdelta);
								data->min = 0;
								data->max = 255;
								data->cur = settings.centerdelta;
								data->defvalue = field.centerdelta();
								break;

							case ANALOG_ITEM_REVERSE:
								text = string_format("%s Reverse", field.name());
								subtext.assign(settings.reverse ? "On" : "Off");
								data->min = 0;
								data->max = 1;
								data->cur = settings.reverse;
								data->defvalue = field.analog_reverse();
								break;

							case ANALOG_ITEM_SENSITIVITY:
								text = string_format("%s Sensitivity", field.name());
								subtext = string_format("%d", settings.sensitivity);
								data->min = 1;
								data->max = 255;
								data->cur = settings.sensitivity;
								data->defvalue = field.sensitivity();
								break;
						}

						/* put on arrows */
						if (data->cur > data->min)
							flags |= FLAG_LEFT_ARROW;
						if (data->cur < data->max)
							flags |= FLAG_RIGHT_ARROW;

						/* append a menu item */
						item_append(std::move(text), std::move(subtext), flags, data);
					}
			}
}

menu_analog::~menu_analog()
{
}

} // namespace ui
