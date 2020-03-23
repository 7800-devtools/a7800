// license:BSD-3-Clause
// copyright-holders:Nathan Woods
/*********************************************************************

    ui/devopt.cpp

    Internal menu for the device configuration.

*********************************************************************/

#include "emu.h"
#include "ui/ui.h"
#include "ui/devopt.h"


namespace ui {
/*-------------------------------------------------
 device_config - handle the game information
 menu
 -------------------------------------------------*/

menu_device_config::menu_device_config(mame_ui_manager &mui, render_container &container, device_slot_interface *slot, device_slot_option *option) : menu(mui, container)
{
	m_option = option;
	m_owner = slot;
	m_mounted = slot->device().subdevice(option->name()) != nullptr;
}

void menu_device_config::populate(float &customtop, float &custombottom)
{
	std::ostringstream str;
	device_t *dev;

	util::stream_format(str, "[This option is%s currently mounted in the running system]\n\n", m_mounted ? "" : " NOT");
	util::stream_format(str, "Option: %s\n", m_option->name());

	dev = const_cast<machine_config &>(machine().config()).device_add(&machine().config().root_device(), m_option->name(), m_option->devtype(), 0);

	util::stream_format(str, "Device: %s\n", dev->name());
	if (!m_mounted)
		str << "\nIf you select this option, the following items will be enabled:\n";
	else
		str << "\nThe selected option enables the following items:\n";

	// loop over all CPUs
	execute_interface_iterator execiter(*dev);
	if (execiter.count() > 0)
	{
		str << "* CPU:\n";
		std::unordered_set<std::string> exectags;
		for (device_execute_interface &exec : execiter)
		{
			if (!exectags.insert(exec.device().tag()).second)
				continue;

			// get cpu specific clock that takes internal multiplier/dividers into account
			int clock = exec.device().clock();

			// count how many identical CPUs we have
			int count = 1;
			const char *name = exec.device().name();
			for (device_execute_interface &scan : execiter)
			{
				if (exec.device().type() == scan.device().type() && strcmp(name, scan.device().name()) == 0 && exec.device().clock() == scan.device().clock())
					if (exectags.insert(scan.device().tag()).second)
						count++;
			}

			// if more than one, prepend a #x in front of the CPU name
			if (count > 1)
				util::stream_format(str, "  %d" UTF8_MULTIPLY, count);
			else
				str << "  ";
			str << name;

			// display clock in kHz or MHz
			if (clock >= 1000000)
				util::stream_format(str, " %d.%06d" UTF8_NBSP "MHz\n", clock / 1000000, clock % 1000000);
			else
				util::stream_format(str, " %d.%03d" UTF8_NBSP "kHz\n", clock / 1000, clock % 1000);
		}
	}

	// display screen information
	screen_device_iterator scriter(*dev);
	if (scriter.count() > 0)
	{
		str << "* Video:\n";
		for (screen_device &screen : scriter)
		{
			util::stream_format(str, "  Screen '%s': ", screen.tag());

			if (screen.screen_type() == SCREEN_TYPE_VECTOR)
				str << "Vector\n";
			else
			{
				const rectangle &visarea = screen.visible_area();

				util::stream_format(str, "%d " UTF8_MULTIPLY " %d (%s) %f" UTF8_NBSP "Hz\n",
									visarea.width(), visarea.height(),
									(machine().system().flags & ORIENTATION_SWAP_XY) ? "V" : "H",
									ATTOSECONDS_TO_HZ(screen.frame_period().attoseconds()));
			}
		}
	}

	// loop over all sound chips
	sound_interface_iterator snditer(*dev);
	if (snditer.count() > 0)
	{
		str << "* Sound:\n";
		std::unordered_set<std::string> soundtags;
		for (device_sound_interface &sound : snditer)
		{
			if (!sound.issound() || !soundtags.insert(sound.device().tag()).second)
				continue;

			// count how many identical sound chips we have
			int count = 1;
			for (device_sound_interface &scan : snditer)
			{
				if (sound.device().type() == scan.device().type() && sound.device().clock() == scan.device().clock())
					if (soundtags.insert(scan.device().tag()).second)
						count++;
			}
			// if more than one, prepend a #x in front of the CPU name
			if (count > 1)
				util::stream_format(str,"  %d" UTF8_MULTIPLY, count);
			else
				str << "  ";
			str << sound.device().name();

			// display clock in kHz or MHz
			int clock = sound.device().clock();
			if (clock >= 1000000)
				util::stream_format(str," %d.%06d" UTF8_NBSP "MHz\n", clock / 1000000, clock % 1000000);
			else if (clock != 0)
				util::stream_format(str," %d.%03d" UTF8_NBSP "kHz\n", clock / 1000, clock % 1000);
			else
				str << '\n';
		}
	}

	// scan for BIOS settings
	int bios = 0;
	if (dev->rom_region())
	{
		std::string bios_str;
		// first loop through roms in search of default bios (shortname)
		for (const rom_entry &rom : dev->rom_region_vector())
			if (ROMENTRY_ISDEFAULT_BIOS(&rom))
				bios_str.assign(ROM_GETNAME(&rom));

		// then loop again to count bios options and to get the default bios complete name
		for (const rom_entry &rom : dev->rom_region_vector())
		{
			if (ROMENTRY_ISSYSTEM_BIOS(&rom))
			{
				bios++;
				if (bios_str.compare(ROM_GETNAME(&rom))==0)
					bios_str.assign(ROM_GETHASHDATA(&rom));
			}
		}

		if (bios)
			util::stream_format(str, "* BIOS settings:\n  %d options    [default: %s]\n", bios, bios_str.c_str());
	}

	int input = 0, input_mj = 0, input_hana = 0, input_gamble = 0, input_analog = 0, input_adjust = 0;
	int input_keypad = 0, input_keyboard = 0, dips = 0, confs = 0;
	std::string errors;
	std::ostringstream dips_opt, confs_opt;
	ioport_list portlist;
	for (device_t &iptdev : device_iterator(*dev))
		portlist.append(iptdev, errors);

	// check if the device adds inputs to the system
	for (auto &port : portlist)
		for (ioport_field &field : port.second->fields())
		{
			if (field.type() >= IPT_MAHJONG_FIRST && field.type() < IPT_MAHJONG_LAST)
				input_mj++;
			else if (field.type() >= IPT_HANAFUDA_FIRST && field.type() < IPT_HANAFUDA_LAST)
				input_hana++;
			else if (field.type() >= IPT_GAMBLING_FIRST && field.type() < IPT_GAMBLING_LAST)
				input_gamble++;
			else if (field.type() >= IPT_ANALOG_FIRST && field.type() < IPT_ANALOG_LAST)
				input_analog++;
			else if (field.type() == IPT_ADJUSTER)
				input_adjust++;
			else if (field.type() == IPT_KEYPAD)
				input_keypad++;
			else if (field.type() == IPT_KEYBOARD)
				input_keyboard++;
			else if (field.type() >= IPT_START1 && field.type() < IPT_UI_FIRST)
				input++;
			else if (field.type() == IPT_DIPSWITCH)
			{
				dips++;
				dips_opt << "  " << field.name();
				for (ioport_setting &setting : field.settings())
				{
					if (setting.value() == field.defvalue())
					{
						util::stream_format(dips_opt, "    [default: %s]\n", setting.name());
						break;
					}
				}
			}
			else if (field.type() == IPT_CONFIG)
			{
				confs++;
				confs_opt << "  " << field.name();
				for (ioport_setting &setting : field.settings())
				{
					if (setting.value() == field.defvalue())
					{
						util::stream_format(confs_opt, "    [default: %s]\n", setting.name());
						break;
					}
				}
			}
		}

	if (dips)
		str << "* Dispwitch settings:\n" << dips_opt.str();
	if (confs)
		str << "* Configuration settings:\n" << confs_opt.str();
	if (input + input_mj + input_hana + input_gamble + input_analog + input_adjust + input_keypad + input_keyboard)
		str << "* Input device(s):\n";
	if (input)
		util::stream_format(str, "  User inputs    [%d inputs]\n", input);
	if (input_mj)
		util::stream_format(str, "  Mahjong inputs    [%d inputs]\n", input_mj);
	if (input_hana)
		util::stream_format(str, "  Hanafuda inputs    [%d inputs]\n", input_hana);
	if (input_gamble)
		util::stream_format(str, "  Gambling inputs    [%d inputs]\n", input_gamble);
	if (input_analog)
		util::stream_format(str, "  Analog inputs    [%d inputs]\n", input_analog);
	if (input_adjust)
		util::stream_format(str, "  Adjuster inputs    [%d inputs]\n", input_adjust);
	if (input_keypad)
		util::stream_format(str, "  Keypad inputs    [%d inputs]\n", input_keypad);
	if (input_keyboard)
		util::stream_format(str, "  Keyboard inputs    [%d inputs]\n", input_keyboard);

	image_interface_iterator imgiter(*dev);
	if (imgiter.count() > 0)
	{
		str << "* Media Options:\n";
		for (const device_image_interface &imagedev : imgiter)
			util::stream_format(str, "  %s    [tag: %s]\n", imagedev.image_type_name(), imagedev.device().tag());
	}

	slot_interface_iterator slotiter(*dev);
	if (slotiter.count() > 0)
	{
		str << "* Slot Options:\n";
		for (const device_slot_interface &slot : slotiter)
			util::stream_format(str, "  %s    [default: %s]\n", slot.device().tag(), slot.default_option() ? slot.default_option() : "----");
	}

	if ((execiter.count() + scriter.count() + snditer.count() + imgiter.count() + slotiter.count() + bios + dips + confs
			+ input + input_mj + input_hana + input_gamble + input_analog + input_adjust + input_keypad + input_keyboard) == 0)
			str << "[None]\n";

	const_cast<machine_config &>(machine().config()).device_remove(&machine().config().root_device(), m_option->name());
	item_append(str.str(), "", FLAG_MULTILINE, nullptr);
}

void menu_device_config::handle()
{
	/* process the menu */
	process(0);
}

menu_device_config::~menu_device_config()
{
}

} // namespace ui
