// license:BSD-3-Clause
// copyright-holders:Aaron Giles,Paul Priest
/***************************************************************************

    info.c

    Dumps the MAME internal data as an XML file.

***************************************************************************/

#include "emu.h"

#include "info.h"
#include "mameopts.h"

#include "machine/ram.h"
#include "sound/samples.h"

#include "config.h"
#include "drivenum.h"
#include "screen.h"
#include "softlist_dev.h"
#include "speaker.h"

#include "xmlfile.h"

#include <ctype.h>


#define XML_ROOT                "mame"
#define XML_TOP                 "machine"


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

// DTD string describing the data
const char info_xml_creator::s_dtd_string[] =
"<!DOCTYPE __XML_ROOT__ [\n"
"<!ELEMENT __XML_ROOT__ (__XML_TOP__+)>\n"
"\t<!ATTLIST __XML_ROOT__ build CDATA #IMPLIED>\n"
"\t<!ATTLIST __XML_ROOT__ debug (yes|no) \"no\">\n"
"\t<!ATTLIST __XML_ROOT__ mameconfig CDATA #REQUIRED>\n"
"\t<!ELEMENT __XML_TOP__ (description, year?, manufacturer?, biosset*, rom*, disk*, device_ref*, sample*, chip*, display*, sound?, input?, dipswitch*, configuration*, port*, adjuster*, driver?, device*, slot*, softwarelist*, ramoption*)>\n"
"\t\t<!ATTLIST __XML_TOP__ name CDATA #REQUIRED>\n"
"\t\t<!ATTLIST __XML_TOP__ sourcefile CDATA #IMPLIED>\n"
"\t\t<!ATTLIST __XML_TOP__ isbios (yes|no) \"no\">\n"
"\t\t<!ATTLIST __XML_TOP__ isdevice (yes|no) \"no\">\n"
"\t\t<!ATTLIST __XML_TOP__ ismechanical (yes|no) \"no\">\n"
"\t\t<!ATTLIST __XML_TOP__ runnable (yes|no) \"yes\">\n"
"\t\t<!ATTLIST __XML_TOP__ cloneof CDATA #IMPLIED>\n"
"\t\t<!ATTLIST __XML_TOP__ romof CDATA #IMPLIED>\n"
"\t\t<!ATTLIST __XML_TOP__ sampleof CDATA #IMPLIED>\n"
"\t\t<!ELEMENT description (#PCDATA)>\n"
"\t\t<!ELEMENT year (#PCDATA)>\n"
"\t\t<!ELEMENT manufacturer (#PCDATA)>\n"
"\t\t<!ELEMENT biosset EMPTY>\n"
"\t\t\t<!ATTLIST biosset name CDATA #REQUIRED>\n"
"\t\t\t<!ATTLIST biosset description CDATA #REQUIRED>\n"
"\t\t\t<!ATTLIST biosset default (yes|no) \"no\">\n"
"\t\t<!ELEMENT rom EMPTY>\n"
"\t\t\t<!ATTLIST rom name CDATA #REQUIRED>\n"
"\t\t\t<!ATTLIST rom bios CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST rom size CDATA #REQUIRED>\n"
"\t\t\t<!ATTLIST rom crc CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST rom sha1 CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST rom merge CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST rom region CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST rom offset CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST rom status (baddump|nodump|good) \"good\">\n"
"\t\t\t<!ATTLIST rom optional (yes|no) \"no\">\n"
"\t\t<!ELEMENT disk EMPTY>\n"
"\t\t\t<!ATTLIST disk name CDATA #REQUIRED>\n"
"\t\t\t<!ATTLIST disk sha1 CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST disk merge CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST disk region CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST disk index CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST disk writable (yes|no) \"no\">\n"
"\t\t\t<!ATTLIST disk status (baddump|nodump|good) \"good\">\n"
"\t\t\t<!ATTLIST disk optional (yes|no) \"no\">\n"
"\t\t<!ELEMENT device_ref EMPTY>\n"
"\t\t\t<!ATTLIST device_ref name CDATA #REQUIRED>\n"
"\t\t<!ELEMENT sample EMPTY>\n"
"\t\t\t<!ATTLIST sample name CDATA #REQUIRED>\n"
"\t\t<!ELEMENT chip EMPTY>\n"
"\t\t\t<!ATTLIST chip name CDATA #REQUIRED>\n"
"\t\t\t<!ATTLIST chip tag CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST chip type (cpu|audio) #REQUIRED>\n"
"\t\t\t<!ATTLIST chip clock CDATA #IMPLIED>\n"
"\t\t<!ELEMENT display EMPTY>\n"
"\t\t\t<!ATTLIST display tag CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST display type (raster|vector|lcd|unknown) #REQUIRED>\n"
"\t\t\t<!ATTLIST display rotate (0|90|180|270) #IMPLIED>\n"
"\t\t\t<!ATTLIST display flipx (yes|no) \"no\">\n"
"\t\t\t<!ATTLIST display width CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST display height CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST display refresh CDATA #REQUIRED>\n"
"\t\t\t<!ATTLIST display pixclock CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST display htotal CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST display hbend CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST display hbstart CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST display vtotal CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST display vbend CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST display vbstart CDATA #IMPLIED>\n"
"\t\t<!ELEMENT sound EMPTY>\n"
"\t\t\t<!ATTLIST sound channels CDATA #REQUIRED>\n"
"\t\t<!ELEMENT input (control*)>\n"
"\t\t\t<!ATTLIST input service (yes|no) \"no\">\n"
"\t\t\t<!ATTLIST input tilt (yes|no) \"no\">\n"
"\t\t\t<!ATTLIST input players CDATA #REQUIRED>\n"
"\t\t\t<!ATTLIST input coins CDATA #IMPLIED>\n"
"\t\t\t<!ELEMENT control EMPTY>\n"
"\t\t\t\t<!ATTLIST control type CDATA #REQUIRED>\n"
"\t\t\t\t<!ATTLIST control player CDATA #IMPLIED>\n"
"\t\t\t\t<!ATTLIST control buttons CDATA #IMPLIED>\n"
"\t\t\t\t<!ATTLIST control reqbuttons CDATA #IMPLIED>\n"
"\t\t\t\t<!ATTLIST control minimum CDATA #IMPLIED>\n"
"\t\t\t\t<!ATTLIST control maximum CDATA #IMPLIED>\n"
"\t\t\t\t<!ATTLIST control sensitivity CDATA #IMPLIED>\n"
"\t\t\t\t<!ATTLIST control keydelta CDATA #IMPLIED>\n"
"\t\t\t\t<!ATTLIST control reverse (yes|no) \"no\">\n"
"\t\t\t\t<!ATTLIST control ways CDATA #IMPLIED>\n"
"\t\t\t\t<!ATTLIST control ways2 CDATA #IMPLIED>\n"
"\t\t\t\t<!ATTLIST control ways3 CDATA #IMPLIED>\n"
"\t\t<!ELEMENT dipswitch (dipvalue*)>\n"
"\t\t\t<!ATTLIST dipswitch name CDATA #REQUIRED>\n"
"\t\t\t<!ATTLIST dipswitch tag CDATA #REQUIRED>\n"
"\t\t\t<!ATTLIST dipswitch mask CDATA #REQUIRED>\n"
"\t\t\t<!ELEMENT dipvalue EMPTY>\n"
"\t\t\t\t<!ATTLIST dipvalue name CDATA #REQUIRED>\n"
"\t\t\t\t<!ATTLIST dipvalue value CDATA #REQUIRED>\n"
"\t\t\t\t<!ATTLIST dipvalue default (yes|no) \"no\">\n"
"\t\t<!ELEMENT configuration (confsetting*)>\n"
"\t\t\t<!ATTLIST configuration name CDATA #REQUIRED>\n"
"\t\t\t<!ATTLIST configuration tag CDATA #REQUIRED>\n"
"\t\t\t<!ATTLIST configuration mask CDATA #REQUIRED>\n"
"\t\t\t<!ELEMENT confsetting EMPTY>\n"
"\t\t\t\t<!ATTLIST confsetting name CDATA #REQUIRED>\n"
"\t\t\t\t<!ATTLIST confsetting value CDATA #REQUIRED>\n"
"\t\t\t\t<!ATTLIST confsetting default (yes|no) \"no\">\n"
"\t\t<!ELEMENT port (analog*)>\n"
"\t\t\t<!ATTLIST port tag CDATA #REQUIRED>\n"
"\t\t\t<!ELEMENT analog EMPTY>\n"
"\t\t\t\t<!ATTLIST analog mask CDATA #REQUIRED>\n"
"\t\t<!ELEMENT adjuster EMPTY>\n"
"\t\t\t<!ATTLIST adjuster name CDATA #REQUIRED>\n"
"\t\t\t<!ATTLIST adjuster default CDATA #REQUIRED>\n"
"\t\t<!ELEMENT driver EMPTY>\n"
"\t\t\t<!ATTLIST driver status (good|imperfect|preliminary) #REQUIRED>\n"
"\t\t\t<!ATTLIST driver emulation (good|imperfect|preliminary) #REQUIRED>\n"
"\t\t\t<!ATTLIST driver color (good|imperfect|preliminary) #REQUIRED>\n"
"\t\t\t<!ATTLIST driver sound (good|imperfect|preliminary) #REQUIRED>\n"
"\t\t\t<!ATTLIST driver graphic (good|imperfect|preliminary) #REQUIRED>\n"
"\t\t\t<!ATTLIST driver cocktail (good|imperfect|preliminary) #IMPLIED>\n"
"\t\t\t<!ATTLIST driver protection (good|imperfect|preliminary) #IMPLIED>\n"
"\t\t\t<!ATTLIST driver savestate (supported|unsupported) #REQUIRED>\n"
"\t\t<!ELEMENT device (instance*, extension*)>\n"
"\t\t\t<!ATTLIST device type CDATA #REQUIRED>\n"
"\t\t\t<!ATTLIST device tag CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST device fixed_image CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST device mandatory CDATA #IMPLIED>\n"
"\t\t\t<!ATTLIST device interface CDATA #IMPLIED>\n"
"\t\t\t<!ELEMENT instance EMPTY>\n"
"\t\t\t\t<!ATTLIST instance name CDATA #REQUIRED>\n"
"\t\t\t\t<!ATTLIST instance briefname CDATA #REQUIRED>\n"
"\t\t\t<!ELEMENT extension EMPTY>\n"
"\t\t\t\t<!ATTLIST extension name CDATA #REQUIRED>\n"
"\t\t<!ELEMENT slot (slotoption*)>\n"
"\t\t\t<!ATTLIST slot name CDATA #REQUIRED>\n"
"\t\t\t<!ELEMENT slotoption EMPTY>\n"
"\t\t\t\t<!ATTLIST slotoption name CDATA #REQUIRED>\n"
"\t\t\t\t<!ATTLIST slotoption devname CDATA #REQUIRED>\n"
"\t\t\t\t<!ATTLIST slotoption default (yes|no) \"no\">\n"
"\t\t<!ELEMENT softwarelist EMPTY>\n"
"\t\t\t<!ATTLIST softwarelist name CDATA #REQUIRED>\n"
"\t\t\t<!ATTLIST softwarelist status (original|compatible) #REQUIRED>\n"
"\t\t\t<!ATTLIST softwarelist filter CDATA #IMPLIED>\n"
"\t\t<!ELEMENT ramoption (#PCDATA)>\n"
"\t\t\t<!ATTLIST ramoption default CDATA #IMPLIED>\n"
"]>";


//**************************************************************************
//  INFO XML CREATOR
//**************************************************************************

//-------------------------------------------------
//  info_xml_creator - constructor
//-------------------------------------------------

info_xml_creator::info_xml_creator(emu_options const &options, bool dtd)
	: m_output(nullptr),
		m_dtd(dtd)
{
}


//-------------------------------------------------
//  output_mame_xml - print the XML information
//  for all known games
//-------------------------------------------------

void info_xml_creator::output(FILE *out, std::vector<std::string> const &patterns)
{
	m_output = out;

	std::unique_ptr<device_type_set> devfilter(patterns.empty() ? nullptr : new device_type_set);

	// track which patterns match machines
	driver_enumerator drivlist(m_lookup_options);
	std::vector<bool> matched(patterns.size(), false);
	size_t exact_matches = 0;
	auto const included = [&patterns, &drivlist, &matched, &exact_matches] (char const *const name) -> bool
	{
		if (patterns.empty())
			return true;

		bool result = false;
		auto it = matched.begin();
		for (std::string const &pat : patterns)
		{
			if (!core_strwildcmp(pat.c_str(), name))
			{
				result = true;
				if (!*it)
				{
					*it = true;
					if (!core_iswildstr(pat.c_str()))
						++exact_matches;
				}
			}
			++it;
		}
		return result;
	};

	// iterate through the drivers, outputting one at a time
	bool first = true;
	while (drivlist.next())
	{
		if (included(drivlist.driver().name))
		{
			if (first)
			{
				output_header();
				first = false;
			}
			output_one(drivlist, devfilter.get());

			// stop looking if we found everything specified
			if (!patterns.empty() && exact_matches == patterns.size())
				break;
		}
	}

	// iterate through the device types if not everything matches a driver
	if (!patterns.empty() && exact_matches != patterns.size())
	{
		for (device_type type : registered_device_types)
		{
			if (included(type.shortname()))
			{
				devfilter->insert(&type);
				if (exact_matches == patterns.size())
					break;
			}
		}
	}

	// output devices (both devices with roms and slot devices)
	if (!devfilter || !devfilter->empty())
	{
		if (first)
		{
			output_header();
			first = false;
		}
		output_devices(devfilter.get());
	}

	if (!first)
		output_footer();

	// throw an error if there were unmatched patterns
	auto it = matched.begin();
	for (std::string const &pat : patterns)
	{
		if (!*it)
			throw emu_fatalerror(EMU_ERR_NO_SUCH_GAME, "No matching machines found for '%s'", pat.c_str());

		++it;
	}
}


//-------------------------------------------------
//  output_mame_xml - print the XML information
//  for a subset of games
//-------------------------------------------------

void info_xml_creator::output(FILE *out, driver_enumerator &drivlist, bool nodevices)
{
	m_output = out;

	device_type_set devfilter;

	output_header();

	// iterate through the drivers, outputting one at a time
	while (drivlist.next())
		output_one(drivlist, &devfilter);

	// output devices (both devices with roms and slot devices)
	if (!nodevices)
		output_devices(&devfilter);

	output_footer();
}


//-------------------------------------------------
//  output_header - print the XML DTD and open
//  the root element
//-------------------------------------------------

void info_xml_creator::output_header()
{
	if (m_dtd)
	{
		// output the DTD
		fprintf(m_output, "<?xml version=\"1.0\"?>\n");
		std::string dtd(s_dtd_string);
		strreplace(dtd, "__XML_ROOT__", XML_ROOT);
		strreplace(dtd, "__XML_TOP__", XML_TOP);

		fprintf(m_output, "%s\n\n", dtd.c_str());
	}

	// top-level tag
	fprintf(m_output, "<%s build=\"%s\" debug=\""
#ifdef MAME_DEBUG
		"yes"
#else
		"no"
#endif
		"\" mameconfig=\"%d\">\n",
		XML_ROOT,
		util::xml::normalize_string(emulator_info::get_build_version()),
		CONFIG_VERSION
	);
}


//-------------------------------------------------
//  output_header - close the root element
//-------------------------------------------------

void info_xml_creator::output_footer()
{
	// close the top level tag
	fprintf(m_output, "</%s>\n", XML_ROOT);
}


//-------------------------------------------------
//  output_one - print the XML information
//  for one particular game driver
//-------------------------------------------------

void info_xml_creator::output_one(driver_enumerator &drivlist, device_type_set *devtypes)
{
	// no action if not a game
	const game_driver &driver = drivlist.driver();
	if (driver.flags & MACHINE_NO_STANDALONE)
		return;

	std::shared_ptr<machine_config> const config(drivlist.config());
	device_iterator iter(config->root_device());

	// allocate input ports
	ioport_list portlist;
	std::string errors;
	for (device_t &device : iter)
	{
		portlist.append(device, errors);

		if (devtypes && device.owner() && device.shortname() && *device.shortname())
			devtypes->insert(&device.type());
	}

	// renumber player numbers for controller ports
	int player_offset = 0;
	// but treat keyboard count separately from players' number
	int kbd_offset = 0;
	for (device_t &device : iter)
	{
		int nplayers = 0;
		bool new_kbd = false;
		for (auto &port : portlist)
			if (&port.second->device() == &device)
				for (ioport_field &field : port.second->fields())
					if (field.type() >= IPT_START && field.type() < IPT_ANALOG_LAST)
					{
						if (field.type() == IPT_KEYBOARD)
						{
							if (!new_kbd) new_kbd = true;
							field.set_player(field.player() + kbd_offset);
						}
						else
						{
							nplayers = std::max(nplayers, field.player() + 1);
							field.set_player(field.player() + player_offset);
						}
					}
		player_offset += nplayers;
		if (new_kbd) kbd_offset++;
	}

	// print the header and the machine name
	fprintf(m_output, "\t<%s",XML_TOP);
	fprintf(m_output, " name=\"%s\"", util::xml::normalize_string(driver.name));

	// strip away any path information from the source_file and output it
	const char *start = strrchr(driver.type.source(), '/');
	if (!start)
		start = strrchr(driver.type.source(), '\\');
	start = start ? (start + 1) : driver.type.source();
	fprintf(m_output, " sourcefile=\"%s\"", util::xml::normalize_string(start));

	// append bios and runnable flags
	if (driver.flags & MACHINE_IS_BIOS_ROOT)
		fprintf(m_output, " isbios=\"yes\"");
	if (driver.flags & MACHINE_NO_STANDALONE)
		fprintf(m_output, " runnable=\"no\"");
	if (driver.flags & MACHINE_MECHANICAL)
		fprintf(m_output, " ismechanical=\"yes\"");

	// display clone information
	int clone_of = drivlist.find(driver.parent);
	if (clone_of != -1 && !(drivlist.driver(clone_of).flags & MACHINE_IS_BIOS_ROOT))
		fprintf(m_output, " cloneof=\"%s\"", util::xml::normalize_string(drivlist.driver(clone_of).name));
	if (clone_of != -1)
		fprintf(m_output, " romof=\"%s\"", util::xml::normalize_string(drivlist.driver(clone_of).name));

	// display sample information and close the game tag
	output_sampleof(config->root_device());
	fprintf(m_output, ">\n");

	// output game description
	if (driver.type.fullname() != nullptr)
		fprintf(m_output, "\t\t<description>%s</description>\n", util::xml::normalize_string(driver.type.fullname()));

	// print the year only if is a number or another allowed character (? or +)
	if (driver.year != nullptr && strspn(driver.year, "0123456789?+") == strlen(driver.year))
		fprintf(m_output, "\t\t<year>%s</year>\n", util::xml::normalize_string(driver.year));

	// print the manufacturer information
	if (driver.manufacturer != nullptr)
		fprintf(m_output, "\t\t<manufacturer>%s</manufacturer>\n", util::xml::normalize_string(driver.manufacturer));

	// now print various additional information
	output_bios(driver);
	output_rom(&drivlist, config->root_device());
	output_device_roms(config->root_device());
	output_sample(config->root_device());
	output_chips(config->root_device(), "");
	output_display(config->root_device(), &drivlist.driver().flags, "");
	output_sound(config->root_device());
	output_input(portlist);
	output_switches(portlist, "", IPT_DIPSWITCH, "dipswitch", "dipvalue");
	output_switches(portlist, "", IPT_CONFIG, "configuration", "confsetting");
	output_ports(portlist);
	output_adjusters(portlist);
	output_driver(driver);
	output_images(config->root_device(), "");
	output_slots(*config, config->root_device(), "", devtypes);
	output_software_list(config->root_device());
	output_ramoptions(config->root_device());

	// close the topmost tag
	fprintf(m_output, "\t</%s>\n",XML_TOP);
}


//-------------------------------------------------
//  output_one_device - print the XML info for
//  a single device
//-------------------------------------------------

void info_xml_creator::output_one_device(machine_config &config, device_t &device, const char *devtag)
{
	bool has_speaker = false, has_input = false;
	// check if the device adds speakers to the system
	sound_interface_iterator snditer(device);
	if (snditer.first() != nullptr)
		has_speaker = true;
	// generate input list
	ioport_list portlist;
	std::string errors;
	for (device_t &dev : device_iterator(device))
		portlist.append(dev, errors);
	// check if the device adds player inputs (other than dsw and configs) to the system
	for (auto &port : portlist)
		for (ioport_field &field : port.second->fields())
			if (field.type() >= IPT_START1 && field.type() < IPT_UI_FIRST)
			{
				has_input = true;
				break;
			}

	// start to output info
	fprintf(m_output, "\t<%s", XML_TOP);
	fprintf(m_output, " name=\"%s\"", util::xml::normalize_string(device.shortname()));
	std::string src(device.source());
	strreplace(src,"../", "");
	fprintf(m_output, " sourcefile=\"%s\"", util::xml::normalize_string(src.c_str()));
	fprintf(m_output, " isdevice=\"yes\"");
	fprintf(m_output, " runnable=\"no\"");
	output_sampleof(device);
	fprintf(m_output, ">\n");
	fprintf(m_output, "\t\t<description>%s</description>\n", util::xml::normalize_string(device.name()));

	output_rom(nullptr, device);

	if (device.type().type() != typeid(samples_device)) // ignore samples_device itself
		output_sample(device);

	output_chips(device, devtag);
	output_display(device, nullptr, devtag);
	if (has_speaker)
		output_sound(device);
	if (has_input)
		output_input(portlist);
	output_switches(portlist, devtag, IPT_DIPSWITCH, "dipswitch", "dipvalue");
	output_switches(portlist, devtag, IPT_CONFIG, "configuration", "confsetting");
	output_adjusters(portlist);
	output_images(device, devtag);
	output_slots(config, device, devtag, nullptr);
	fprintf(m_output, "\t</%s>\n", XML_TOP);
}


//-------------------------------------------------
//  output_devices - print the XML info for devices
//  with roms and for devices that can be mounted
//  in slots
//  The current solution works to some extent, but
//  it is limited by the fact that devices are only
//  acknowledged when attached to a driver (so that
//  for instance sub-sub-devices could never appear
//  in the xml input if they are not also attached
//  directly to a driver as device or sub-device)
//-------------------------------------------------

void info_xml_creator::output_devices(device_type_set const *filter)
{
	// get config for empty machine
	machine_config config(GAME_NAME(___empty), m_lookup_options);

	auto const action = [this, &config] (device_type type)
			{
				// add it at the root of the machine config
				device_t *const dev = config.device_add(&config.root_device(), "_tmp", type, 0);

				// notify this device and all its subdevices that they are now configured
				for (device_t &device : device_iterator(*dev))
					if (!device.configured())
						device.config_complete();

				// print details and remove it
				output_one_device(config, *dev, dev->tag());
				config.device_remove(&config.root_device(), "_tmp");
			};

	// run through devices
	if (filter)
	{
		for (std::add_pointer_t<device_type> type : *filter) action(*type);
	}
	else
	{
		for (device_type type : registered_device_types) action(type);
	}
}


//------------------------------------------------
//  output_device_roms - when a driver uses roms
//  included in a device set, print a reference
//-------------------------------------------------

void info_xml_creator::output_device_roms(device_t &root)
{
	for (device_t &device : device_iterator(root))
		if (device.owner())
			fprintf(m_output, "\t\t<device_ref name=\"%s\"/>\n", util::xml::normalize_string(device.shortname()));
}


//------------------------------------------------
//  output_sampleof - print the 'sampleof'
//  attribute, if appropriate
//-------------------------------------------------

void info_xml_creator::output_sampleof(device_t &device)
{
	// iterate over sample devices
	for (samples_device &samples : samples_device_iterator(device))
	{
		samples_iterator sampiter(samples);
		if (sampiter.altbasename() != nullptr)
		{
			fprintf(m_output, " sampleof=\"%s\"", util::xml::normalize_string(sampiter.altbasename()));

			// must stop here, as there can only be one attribute of the same name
			return;
		}
	}
}


//-------------------------------------------------
//  output_bios - print the BIOS set for a
//  game
//-------------------------------------------------

void info_xml_creator::output_bios(game_driver const &driver)
{
	// skip if no ROMs
	if (driver.rom)
	{
		auto rom_entries = rom_build_entries(driver.rom);

		// first determine the default BIOS name
		std::string defaultname;
		for (const rom_entry &rom : rom_entries)
			if (ROMENTRY_ISDEFAULT_BIOS(&rom))
				defaultname = ROM_GETNAME(&rom);

		// iterate over ROM entries and look for BIOSes
		for (const rom_entry &rom : rom_entries)
			if (ROMENTRY_ISSYSTEM_BIOS(&rom))
			{
				// output extracted name and descriptions
				fprintf(m_output, "\t\t<biosset");
				fprintf(m_output, " name=\"%s\"", util::xml::normalize_string(ROM_GETNAME(&rom)));
				fprintf(m_output, " description=\"%s\"", util::xml::normalize_string(ROM_GETHASHDATA(&rom)));
				if (defaultname == ROM_GETNAME(&rom))
					fprintf(m_output, " default=\"yes\"");
				fprintf(m_output, "/>\n");
			}
	}
}


//-------------------------------------------------
//  output_rom - print the roms section of
//  the XML output
//-------------------------------------------------

void info_xml_creator::output_rom(driver_enumerator *drivlist, device_t &device)
{
	// iterate over 3 different ROM "types": BIOS, ROMs, DISKs
	bool const do_merge_name = drivlist && dynamic_cast<driver_device *>(&device);
	for (int rom_type = 0; rom_type < 3; rom_type++)
		for (const rom_entry *region = rom_first_region(device); region != nullptr; region = rom_next_region(region))
		{
			bool const is_disk = ROMREGION_ISDISKDATA(region);

			// disk regions only work for disks
			if ((is_disk && rom_type != 2) || (!is_disk && rom_type == 2))
				continue;

			// iterate through ROM entries
			std::string bios_name;
			for (const rom_entry *rom = rom_first_file(region); rom != nullptr; rom = rom_next_file(rom))
			{
				// BIOS ROMs only apply to bioses
				bool const is_bios = ROM_GETBIOSFLAGS(rom);
				if ((is_bios && rom_type != 0) || (!is_bios && rom_type == 0))
					continue;

				// if we have a valid ROM and we are a clone, see if we can find the parent ROM
				util::hash_collection hashes(ROM_GETHASHDATA(rom));
				const char *const merge_name = (do_merge_name && !hashes.flag(util::hash_collection::FLAG_NO_DUMP)) ? get_merge_name(*drivlist, hashes) : nullptr;

				// scan for a BIOS name
				bios_name.clear();
				if (!is_disk && is_bios)
				{
					// scan backwards through the ROM entries
					for (const rom_entry *brom = rom - 1; brom != device.rom_region(); brom--)
					{
						if (ROMENTRY_ISSYSTEM_BIOS(brom))
						{
							bios_name = ROM_GETNAME(brom);
							break;
						}
					}
				}

				std::ostringstream output;

				// opening tag
				if (!is_disk)
					output << "\t\t<rom";
				else
					output << "\t\t<disk";

				// add name, merge, bios, and size tags */
				const char *const name = ROM_GETNAME(rom);
				if (name && name[0])
					util::stream_format(output, " name=\"%s\"", util::xml::normalize_string(name));
				if (merge_name)
					util::stream_format(output, " merge=\"%s\"", util::xml::normalize_string(merge_name));
				if (!bios_name.empty())
					util::stream_format(output, " bios=\"%s\"", util::xml::normalize_string(bios_name.c_str()));
				if (!is_disk)
					util::stream_format(output, " size=\"%u\"", rom_file_size(rom));

				// dump checksum information only if there is a known dump
				if (!hashes.flag(util::hash_collection::FLAG_NO_DUMP))
				{
					// iterate over hash function types and print m_output their values
					output << " " << hashes.attribute_string();
				}
				else
					output << " status=\"nodump\"";

				// append a region name
				util::stream_format(output, " region=\"%s\"", ROMREGION_GETTAG(region));

				if (!is_disk)
				{
					// for non-disk entries, print offset
					util::stream_format(output, " offset=\"%x\"", ROM_GETOFFSET(rom));
				}
				else
				{
					// for disk entries, add the disk index
					util::stream_format(output, " index=\"%x\"", DISK_GETINDEX(rom));
					util::stream_format(output, " writable=\"%s\"", DISK_ISREADONLY(rom) ? "no" : "yes");
				}

				// add optional flag
				if (ROM_ISOPTIONAL(rom))
					output << " optional=\"yes\"";

				output << "/>\n";

				fprintf(m_output, "%s", output.str().c_str());
			}
		}
}


//-------------------------------------------------
//  output_sample - print a list of all
//  samples referenced by a game_driver
//-------------------------------------------------

void info_xml_creator::output_sample(device_t &device)
{
	// iterate over sample devices
	for (samples_device &samples : samples_device_iterator(device))
	{
		samples_iterator iter(samples);
		std::unordered_set<std::string> already_printed;
		for (const char *samplename = iter.first(); samplename != nullptr; samplename = iter.next())
		{
			// filter out duplicates
			if (!already_printed.insert(samplename).second)
				continue;

			// output the sample name
			fprintf(m_output, "\t\t<sample name=\"%s\"/>\n", util::xml::normalize_string(samplename));
		}
	}
}


/*-------------------------------------------------
    output_chips - print a list of CPU and
    sound chips used by a game
-------------------------------------------------*/

void info_xml_creator::output_chips(device_t &device, const char *root_tag)
{
	// iterate over executable devices
	for (device_execute_interface &exec : execute_interface_iterator(device))
	{
		if (strcmp(exec.device().tag(), device.tag()))
		{
			std::string newtag(exec.device().tag()), oldtag(":");
			newtag = newtag.substr(newtag.find(oldtag.append(root_tag)) + oldtag.length());

			fprintf(m_output, "\t\t<chip");
			fprintf(m_output, " type=\"cpu\"");
			fprintf(m_output, " tag=\"%s\"", util::xml::normalize_string(newtag.c_str()));
			fprintf(m_output, " name=\"%s\"", util::xml::normalize_string(exec.device().name()));
			fprintf(m_output, " clock=\"%d\"", exec.device().clock());
			fprintf(m_output, "/>\n");
		}
	}

	// iterate over sound devices
	for (device_sound_interface &sound : sound_interface_iterator(device))
	{
		if (strcmp(sound.device().tag(), device.tag()) != 0 && sound.issound())
		{
			std::string newtag(sound.device().tag()), oldtag(":");
			newtag = newtag.substr(newtag.find(oldtag.append(root_tag)) + oldtag.length());

			fprintf(m_output, "\t\t<chip");
			fprintf(m_output, " type=\"audio\"");
			fprintf(m_output, " tag=\"%s\"", util::xml::normalize_string(newtag.c_str()));
			fprintf(m_output, " name=\"%s\"", util::xml::normalize_string(sound.device().name()));
			if (sound.device().clock() != 0)
				fprintf(m_output, " clock=\"%d\"", sound.device().clock());
			fprintf(m_output, "/>\n");
		}
	}
}


//-------------------------------------------------
//  output_display - print a list of all the
//  displays
//-------------------------------------------------

void info_xml_creator::output_display(device_t &device, u32 const *flags, const char *root_tag)
{
	// iterate over screens
	for (const screen_device &screendev : screen_device_iterator(device))
	{
		if (strcmp(screendev.tag(), device.tag()))
		{
			std::string newtag(screendev.tag()), oldtag(":");
			newtag = newtag.substr(newtag.find(oldtag.append(root_tag)) + oldtag.length());

			fprintf(m_output, "\t\t<display");
			fprintf(m_output, " tag=\"%s\"", util::xml::normalize_string(newtag.c_str()));

			switch (screendev.screen_type())
			{
				case SCREEN_TYPE_RASTER:    fprintf(m_output, " type=\"raster\"");  break;
				case SCREEN_TYPE_VECTOR:    fprintf(m_output, " type=\"vector\"");  break;
				case SCREEN_TYPE_LCD:       fprintf(m_output, " type=\"lcd\"");     break;
				default:                    fprintf(m_output, " type=\"unknown\""); break;
			}

			// output the orientation as a string
			if (flags)
			{
				switch (*flags & ORIENTATION_MASK)
				{
				case ORIENTATION_FLIP_X:
					fprintf(m_output, " rotate=\"0\" flipx=\"yes\"");
					break;
				case ORIENTATION_FLIP_Y:
					fprintf(m_output, " rotate=\"180\" flipx=\"yes\"");
					break;
				case ORIENTATION_FLIP_X|ORIENTATION_FLIP_Y:
					fprintf(m_output, " rotate=\"180\"");
					break;
				case ORIENTATION_SWAP_XY:
					fprintf(m_output, " rotate=\"90\" flipx=\"yes\"");
					break;
				case ORIENTATION_SWAP_XY|ORIENTATION_FLIP_X:
					fprintf(m_output, " rotate=\"90\"");
					break;
				case ORIENTATION_SWAP_XY|ORIENTATION_FLIP_Y:
					fprintf(m_output, " rotate=\"270\"");
					break;
				case ORIENTATION_SWAP_XY|ORIENTATION_FLIP_X|ORIENTATION_FLIP_Y:
					fprintf(m_output, " rotate=\"270\" flipx=\"yes\"");
					break;
				default:
					fprintf(m_output, " rotate=\"0\"");
					break;
				}
			}

			// output width and height only for games that are not vector
			if (screendev.screen_type() != SCREEN_TYPE_VECTOR)
			{
				const rectangle &visarea = screendev.visible_area();
				fprintf(m_output, " width=\"%d\"", visarea.width());
				fprintf(m_output, " height=\"%d\"", visarea.height());
			}

			// output refresh rate
			fprintf(m_output, " refresh=\"%f\"", ATTOSECONDS_TO_HZ(screendev.refresh_attoseconds()));

			// output raw video parameters only for games that are not vector
			// and had raw parameters specified
			if (screendev.screen_type() != SCREEN_TYPE_VECTOR && !screendev.oldstyle_vblank_supplied())
			{
				int pixclock = screendev.width() * screendev.height() * ATTOSECONDS_TO_HZ(screendev.refresh_attoseconds());

				fprintf(m_output, " pixclock=\"%d\"", pixclock);
				fprintf(m_output, " htotal=\"%d\"", screendev.width());
				fprintf(m_output, " hbend=\"%d\"", screendev.visible_area().min_x);
				fprintf(m_output, " hbstart=\"%d\"", screendev.visible_area().max_x+1);
				fprintf(m_output, " vtotal=\"%d\"", screendev.height());
				fprintf(m_output, " vbend=\"%d\"", screendev.visible_area().min_y);
				fprintf(m_output, " vbstart=\"%d\"", screendev.visible_area().max_y+1);
			}
			fprintf(m_output, " />\n");
		}
	}
}


//-------------------------------------------------
//  output_sound - print a list of all the
//  speakers
//------------------------------------------------

void info_xml_creator::output_sound(device_t &device)
{
	speaker_device_iterator spkiter(device);
	int speakers = spkiter.count();

	// if we have no sound, zero m_output the speaker count
	sound_interface_iterator snditer(device);
	if (snditer.first() == nullptr)
		speakers = 0;

	fprintf(m_output, "\t\t<sound channels=\"%d\"/>\n", speakers);
}


//-------------------------------------------------
//  output_input - print a summary of a game's
//  input
//-------------------------------------------------

void info_xml_creator::output_input(const ioport_list &portlist)
{
	// enumerated list of control types
	// NOTE: the order is chosen so that 'spare' button inputs are assigned to the
	// most-likely-correct input device when info is output (you can think of it as
	// a sort of likelihood order of having buttons)
	enum
	{
		CTRL_DIGITAL_BUTTONS,
		CTRL_DIGITAL_JOYSTICK,
		CTRL_ANALOG_JOYSTICK,
		CTRL_ANALOG_LIGHTGUN,
		CTRL_ANALOG_DIAL,
		CTRL_ANALOG_POSITIONAL,
		CTRL_ANALOG_TRACKBALL,
		CTRL_ANALOG_MOUSE,
		CTRL_ANALOG_PADDLE,
		CTRL_ANALOG_PEDAL,
		CTRL_DIGITAL_KEYPAD,
		CTRL_DIGITAL_KEYBOARD,
		CTRL_DIGITAL_MAHJONG,
		CTRL_DIGITAL_HANAFUDA,
		CTRL_DIGITAL_GAMBLING,
		CTRL_COUNT
	};

	enum
	{
		CTRL_P1,
		CTRL_P2,
		CTRL_P3,
		CTRL_P4,
		CTRL_P5,
		CTRL_P6,
		CTRL_P7,
		CTRL_P8,
		CTRL_P9,
		CTRL_P10,
		CTRL_PCOUNT
	};

	// directions
	const uint8_t DIR_UP = 0x01;
	const uint8_t DIR_DOWN = 0x02;
	const uint8_t DIR_LEFT = 0x04;
	const uint8_t DIR_RIGHT = 0x08;

	// initialize the list of control types
	struct
	{
		const char *    type;           // general type of input
		int             player;         // player which the input belongs to
		int             nbuttons;       // total number of buttons
		int             reqbuttons;     // total number of non-optional buttons
		int             maxbuttons;     // max index of buttons (using IPT_BUTTONn) [probably to be removed soonish]
		int             ways;           // directions for joystick
		bool            analog;         // is analog input?
		uint8_t           helper[3];      // for dual joysticks [possibly to be removed soonish]
		int32_t           min;            // analog minimum value
		int32_t           max;            // analog maximum value
		int32_t           sensitivity;    // default analog sensitivity
		int32_t           keydelta;       // default analog keydelta
		bool            reverse;        // default analog reverse setting
	} control_info[CTRL_COUNT * CTRL_PCOUNT];

	memset(&control_info, 0, sizeof(control_info));

	// tracking info as we iterate
	int nplayer = 0;
	int ncoin = 0;
	bool service = false;
	bool tilt = false;

	// iterate over the ports
	for (auto &port : portlist)
	{
		int ctrl_type = CTRL_DIGITAL_BUTTONS;
		bool ctrl_analog = false;
		for (ioport_field &field : port.second->fields())
		{
			// track the highest player number
			if (nplayer < field.player() + 1)
				nplayer = field.player() + 1;

			// switch off of the type
			switch (field.type())
			{
				// map joysticks
				case IPT_JOYSTICK_UP:
					ctrl_type = CTRL_DIGITAL_JOYSTICK;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "joy";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].ways = field.way();
					control_info[field.player() * CTRL_COUNT + ctrl_type].helper[0] |= DIR_UP;
					break;
				case IPT_JOYSTICK_DOWN:
					ctrl_type = CTRL_DIGITAL_JOYSTICK;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "joy";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].ways = field.way();
					control_info[field.player() * CTRL_COUNT + ctrl_type].helper[0] |= DIR_DOWN;
					break;
				case IPT_JOYSTICK_LEFT:
					ctrl_type = CTRL_DIGITAL_JOYSTICK;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "joy";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].ways = field.way();
					control_info[field.player() * CTRL_COUNT + ctrl_type].helper[0] |= DIR_LEFT;
					break;
				case IPT_JOYSTICK_RIGHT:
					ctrl_type = CTRL_DIGITAL_JOYSTICK;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "joy";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].ways = field.way();
					control_info[field.player() * CTRL_COUNT + ctrl_type].helper[0] |= DIR_RIGHT;
					break;

				case IPT_JOYSTICKLEFT_UP:
					ctrl_type = CTRL_DIGITAL_JOYSTICK;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "joy";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].ways = field.way();
					control_info[field.player() * CTRL_COUNT + ctrl_type].helper[1] |= DIR_UP;
					break;
				case IPT_JOYSTICKLEFT_DOWN:
					ctrl_type = CTRL_DIGITAL_JOYSTICK;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "joy";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].ways = field.way();
					control_info[field.player() * CTRL_COUNT + ctrl_type].helper[1] |= DIR_DOWN;
					break;
				case IPT_JOYSTICKLEFT_LEFT:
					ctrl_type = CTRL_DIGITAL_JOYSTICK;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "joy";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].ways = field.way();
					control_info[field.player() * CTRL_COUNT + ctrl_type].helper[1] |= DIR_LEFT;
					break;
				case IPT_JOYSTICKLEFT_RIGHT:
					ctrl_type = CTRL_DIGITAL_JOYSTICK;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "joy";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].ways = field.way();
					control_info[field.player() * CTRL_COUNT + ctrl_type].helper[1] |= DIR_RIGHT;
					break;

				case IPT_JOYSTICKRIGHT_UP:
					ctrl_type = CTRL_DIGITAL_JOYSTICK;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "joy";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].ways = field.way();
					control_info[field.player() * CTRL_COUNT + ctrl_type].helper[2] |= DIR_UP;
					break;
				case IPT_JOYSTICKRIGHT_DOWN:
					ctrl_type = CTRL_DIGITAL_JOYSTICK;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "joy";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].ways = field.way();
					control_info[field.player() * CTRL_COUNT + ctrl_type].helper[2] |= DIR_DOWN;
					break;
				case IPT_JOYSTICKRIGHT_LEFT:
					ctrl_type = CTRL_DIGITAL_JOYSTICK;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "joy";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].ways = field.way();
					control_info[field.player() * CTRL_COUNT + ctrl_type].helper[2] |= DIR_LEFT;
					break;
				case IPT_JOYSTICKRIGHT_RIGHT:
					ctrl_type = CTRL_DIGITAL_JOYSTICK;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "joy";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].ways = field.way();
					control_info[field.player() * CTRL_COUNT + ctrl_type].helper[2] |= DIR_RIGHT;
					break;

				// map analog inputs
				case IPT_AD_STICK_X:
				case IPT_AD_STICK_Y:
				case IPT_AD_STICK_Z:
					ctrl_analog = true;
					ctrl_type = CTRL_ANALOG_JOYSTICK;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "stick";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].analog = true;
					break;

				case IPT_PADDLE:
				case IPT_PADDLE_V:
					ctrl_analog = true;
					ctrl_type = CTRL_ANALOG_PADDLE;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "paddle";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].analog = true;
					break;

				case IPT_PEDAL:
				case IPT_PEDAL2:
				case IPT_PEDAL3:
					ctrl_analog = true;
					ctrl_type = CTRL_ANALOG_PEDAL;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "pedal";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].analog = true;
					break;

				case IPT_LIGHTGUN_X:
				case IPT_LIGHTGUN_Y:
					ctrl_analog = true;
					ctrl_type = CTRL_ANALOG_LIGHTGUN;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "lightgun";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].analog = true;
					break;

				case IPT_POSITIONAL:
				case IPT_POSITIONAL_V:
					ctrl_analog = true;
					ctrl_type = CTRL_ANALOG_POSITIONAL;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "positional";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].analog = true;
					break;

				case IPT_DIAL:
				case IPT_DIAL_V:
					ctrl_analog = true;
					ctrl_type = CTRL_ANALOG_DIAL;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "dial";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].analog = true;
					break;

				case IPT_TRACKBALL_X:
				case IPT_TRACKBALL_Y:
					ctrl_analog = true;
					ctrl_type = CTRL_ANALOG_TRACKBALL;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "trackball";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].analog = true;
					break;

				case IPT_MOUSE_X:
				case IPT_MOUSE_Y:
					ctrl_analog = true;
					ctrl_type = CTRL_ANALOG_MOUSE;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "mouse";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].analog = true;
					break;

				// map buttons
				case IPT_BUTTON1:
				case IPT_BUTTON2:
				case IPT_BUTTON3:
				case IPT_BUTTON4:
				case IPT_BUTTON5:
				case IPT_BUTTON6:
				case IPT_BUTTON7:
				case IPT_BUTTON8:
				case IPT_BUTTON9:
				case IPT_BUTTON10:
				case IPT_BUTTON11:
				case IPT_BUTTON12:
				case IPT_BUTTON13:
				case IPT_BUTTON14:
				case IPT_BUTTON15:
				case IPT_BUTTON16:
					ctrl_analog = false;
					if (control_info[field.player() * CTRL_COUNT + ctrl_type].type == nullptr)
					{
						control_info[field.player() * CTRL_COUNT + ctrl_type].type = "only_buttons";
						control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
						control_info[field.player() * CTRL_COUNT + ctrl_type].analog = false;
					}
					control_info[field.player() * CTRL_COUNT + ctrl_type].maxbuttons = std::max(control_info[field.player() * CTRL_COUNT + ctrl_type].maxbuttons, field.type() - IPT_BUTTON1 + 1);
					control_info[field.player() * CTRL_COUNT + ctrl_type].nbuttons++;
					if (!field.optional())
						control_info[field.player() * CTRL_COUNT + ctrl_type].reqbuttons++;
					break;

				// track maximum coin index
				case IPT_COIN1:
				case IPT_COIN2:
				case IPT_COIN3:
				case IPT_COIN4:
				case IPT_COIN5:
				case IPT_COIN6:
				case IPT_COIN7:
				case IPT_COIN8:
				case IPT_COIN9:
				case IPT_COIN10:
				case IPT_COIN11:
				case IPT_COIN12:
					ncoin = std::max(ncoin, field.type() - IPT_COIN1 + 1);
					break;

				// track presence of keypads and keyboards
				case IPT_KEYPAD:
					ctrl_type = CTRL_DIGITAL_KEYPAD;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "keypad";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].nbuttons++;
					if (!field.optional())
						control_info[field.player() * CTRL_COUNT + ctrl_type].reqbuttons++;
					break;

				case IPT_KEYBOARD:
					ctrl_type = CTRL_DIGITAL_KEYBOARD;
					control_info[field.player() * CTRL_COUNT + ctrl_type].type = "keyboard";
					control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
					control_info[field.player() * CTRL_COUNT + ctrl_type].nbuttons++;
					if (!field.optional())
						control_info[field.player() * CTRL_COUNT + ctrl_type].reqbuttons++;
					break;

				// additional types
				case IPT_SERVICE:
					service = true;
					break;

				case IPT_TILT:
					tilt = true;
					break;

				default:
					if (field.type() > IPT_MAHJONG_FIRST && field.type() < IPT_MAHJONG_LAST)
					{
						ctrl_type = CTRL_DIGITAL_MAHJONG;
						control_info[field.player() * CTRL_COUNT + ctrl_type].type = "mahjong";
						control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
						control_info[field.player() * CTRL_COUNT + ctrl_type].nbuttons++;
						if (!field.optional())
							control_info[field.player() * CTRL_COUNT + ctrl_type].reqbuttons++;
					}
					else if (field.type() > IPT_HANAFUDA_FIRST && field.type() < IPT_HANAFUDA_LAST)
					{
						ctrl_type = CTRL_DIGITAL_HANAFUDA;
						control_info[field.player() * CTRL_COUNT + ctrl_type].type = "hanafuda";
						control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
						control_info[field.player() * CTRL_COUNT + ctrl_type].nbuttons++;
						if (!field.optional())
							control_info[field.player() * CTRL_COUNT + ctrl_type].reqbuttons++;
					}
					else if (field.type() > IPT_GAMBLING_FIRST && field.type() < IPT_GAMBLING_LAST)
					{
						ctrl_type = CTRL_DIGITAL_GAMBLING;
						control_info[field.player() * CTRL_COUNT + ctrl_type].type = "gambling";
						control_info[field.player() * CTRL_COUNT + ctrl_type].player = field.player() + 1;
						control_info[field.player() * CTRL_COUNT + ctrl_type].nbuttons++;
						if (!field.optional())
							control_info[field.player() * CTRL_COUNT + ctrl_type].reqbuttons++;
					}
					break;
			}

			if (ctrl_analog)
			{
				// get the analog stats
				if (field.minval() != 0)
					control_info[field.player() * CTRL_COUNT + ctrl_type].min = field.minval();
				if (field.maxval() != 0)
					control_info[field.player() * CTRL_COUNT + ctrl_type].max = field.maxval();
				if (field.sensitivity() != 0)
					control_info[field.player() * CTRL_COUNT + ctrl_type].sensitivity = field.sensitivity();
				if (field.delta() != 0)
					control_info[field.player() * CTRL_COUNT + ctrl_type].keydelta = field.delta();
				if (field.analog_reverse() != 0)
					control_info[field.player() * CTRL_COUNT + ctrl_type].reverse = true;
			}
		}
	}

	// Clean-up those entries, if any, where buttons were defined in a separate port than the actual controller they belong to.
	// This is quite often the case, especially for arcades where controls can be easily mapped to separate input ports on PCB.
	// If such situation would only happen for joystick, it would be possible to work it around by initializing differently
	// ctrl_type above, but it is quite common among analog inputs as well (for instance, this is the tipical situation
	// for lightguns) and therefore we really need this separate loop.
	for (int i = 0; i < CTRL_PCOUNT; i++)
	{
		bool fix_done = false;
		for (int j = 1; j < CTRL_COUNT; j++)
			if (control_info[i * CTRL_COUNT].type != nullptr && control_info[i * CTRL_COUNT + j].type != nullptr && !fix_done)
			{
				control_info[i * CTRL_COUNT + j].nbuttons += control_info[i * CTRL_COUNT].nbuttons;
				control_info[i * CTRL_COUNT + j].reqbuttons += control_info[i * CTRL_COUNT].reqbuttons;
				control_info[i * CTRL_COUNT + j].maxbuttons = std::max(control_info[i * CTRL_COUNT + j].maxbuttons, control_info[i * CTRL_COUNT].maxbuttons);

				memset(&control_info[i * CTRL_COUNT], 0, sizeof(control_info[0]));
				fix_done = true;
			}
	}

	// Output the input info
	// First basic info
	fprintf(m_output, "\t\t<input");
	fprintf(m_output, " players=\"%d\"", nplayer);
	if (ncoin != 0)
		fprintf(m_output, " coins=\"%d\"", ncoin);
	if (service)
		fprintf(m_output, " service=\"yes\"");
	if (tilt)
		fprintf(m_output, " tilt=\"yes\"");
	fprintf(m_output, ">\n");

	// Then controller specific ones
	for (auto & elem : control_info)
		if (elem.type != nullptr)
		{
			//printf("type %s - player %d - buttons %d\n", elem.type, elem.player, elem.nbuttons);
			if (elem.analog)
			{
				fprintf(m_output, "\t\t\t<control type=\"%s\"", util::xml::normalize_string(elem.type));
				if (nplayer > 1)
					fprintf(m_output, " player=\"%d\"", elem.player);
				if (elem.nbuttons > 0)
				{
					fprintf(m_output, " buttons=\"%d\"", strcmp(elem.type, "stick") ? elem.nbuttons : elem.maxbuttons);
					if (elem.reqbuttons < elem.nbuttons)
						fprintf(m_output, " reqbuttons=\"%d\"", elem.reqbuttons);
				}
				if (elem.min != 0 || elem.max != 0)
				{
					fprintf(m_output, " minimum=\"%d\"", elem.min);
					fprintf(m_output, " maximum=\"%d\"", elem.max);
				}
				if (elem.sensitivity != 0)
					fprintf(m_output, " sensitivity=\"%d\"", elem.sensitivity);
				if (elem.keydelta != 0)
					fprintf(m_output, " keydelta=\"%d\"", elem.keydelta);
				if (elem.reverse)
					fprintf(m_output, " reverse=\"yes\"");

				fprintf(m_output, "/>\n");
			}
			else
			{
				if (elem.helper[1] == 0 && elem.helper[2] != 0) { elem.helper[1] = elem.helper[2]; elem.helper[2] = 0; }
				if (elem.helper[0] == 0 && elem.helper[1] != 0) { elem.helper[0] = elem.helper[1]; elem.helper[1] = 0; }
				if (elem.helper[1] == 0 && elem.helper[2] != 0) { elem.helper[1] = elem.helper[2]; elem.helper[2] = 0; }
				const char *joys = (elem.helper[2] != 0) ? "triple" : (elem.helper[1] != 0) ? "double" : "";
				fprintf(m_output, "\t\t\t<control type=\"%s%s\"", joys, util::xml::normalize_string(elem.type));
				if (nplayer > 1)
					fprintf(m_output, " player=\"%d\"", elem.player);
				if (elem.nbuttons > 0)
				{
					fprintf(m_output, " buttons=\"%d\"", strcmp(elem.type, "joy") ? elem.nbuttons : elem.maxbuttons);
					if (elem.reqbuttons < elem.nbuttons)
						fprintf(m_output, " reqbuttons=\"%d\"", elem.reqbuttons);
				}
				for (int lp = 0; lp < 3 && elem.helper[lp] != 0; lp++)
				{
					const char *plural = (lp==2) ? "3" : (lp==1) ? "2" : "";
					const char *ways;
					std::string helper;
					switch (elem.helper[lp] & (DIR_UP | DIR_DOWN | DIR_LEFT | DIR_RIGHT))
					{
						case DIR_UP | DIR_DOWN | DIR_LEFT | DIR_RIGHT:
							helper = string_format("%d", (elem.ways == 0) ? 8 : elem.ways);
							ways = helper.c_str();
							break;
						case DIR_LEFT | DIR_RIGHT:
							ways = "2";
							break;
						case DIR_UP | DIR_DOWN:
							ways = "vertical2";
							break;
						case DIR_UP:
						case DIR_DOWN:
						case DIR_LEFT:
						case DIR_RIGHT:
							ways = "1";
							break;
						case DIR_UP | DIR_DOWN | DIR_LEFT:
						case DIR_UP | DIR_DOWN | DIR_RIGHT:
						case DIR_UP | DIR_LEFT | DIR_RIGHT:
						case DIR_DOWN | DIR_LEFT | DIR_RIGHT:
							ways = (elem.ways == 4) ? "3 (half4)" : "5 (half8)";
							break;
						default:
							ways = "strange2";
							break;
					}
					fprintf(m_output, " ways%s=\"%s\"", plural, ways);
				}
				fprintf(m_output, "/>\n");
			}
		}

	fprintf(m_output, "\t\t</input>\n");
}


//-------------------------------------------------
//  output_switches - print the configurations or
//  DIP switch settings
//-------------------------------------------------

void info_xml_creator::output_switches(const ioport_list &portlist, const char *root_tag, int type, const char *outertag, const char *innertag)
{
	// iterate looking for DIP switches
	for (auto &port : portlist)
		for (ioport_field &field : port.second->fields())
			if (field.type() == type)
			{
				std::ostringstream output;

				std::string newtag(port.second->tag()), oldtag(":");
				newtag = newtag.substr(newtag.find(oldtag.append(root_tag)) + oldtag.length());

				// output the switch name information
				std::string normalized_field_name(util::xml::normalize_string(field.name()));
				std::string normalized_newtag(util::xml::normalize_string(newtag.c_str()));
				util::stream_format(output,"\t\t<%s name=\"%s\" tag=\"%s\" mask=\"%u\">\n", outertag, normalized_field_name.c_str(), normalized_newtag.c_str(), field.mask());

				// loop over settings
				for (ioport_setting &setting : field.settings())
				{
					util::stream_format(output,"\t\t\t<%s name=\"%s\" value=\"%u\"%s/>\n", innertag, util::xml::normalize_string(setting.name()), setting.value(), setting.value() == field.defvalue() ? " default=\"yes\"" : "");
				}

				// terminate the switch entry
				util::stream_format(output,"\t\t</%s>\n", outertag);

				fprintf(m_output, "%s", output.str().c_str());
			}
}

//-------------------------------------------------
//  output_ports - print the structure of input
//  ports in the driver
//-------------------------------------------------
void info_xml_creator::output_ports(const ioport_list &portlist)
{
	// cycle through ports
	for (auto &port : portlist)
	{
		fprintf(m_output,"\t\t<port tag=\"%s\">\n", util::xml::normalize_string(port.second->tag()));
		for (ioport_field &field : port.second->fields())
		{
			if(field.is_analog())
				fprintf(m_output,"\t\t\t<analog mask=\"%u\"/>\n", field.mask());
		}
		// close element
		fprintf(m_output,"\t\t</port>\n");
	}

}

//-------------------------------------------------
//  output_adjusters - print the Analog
//  Adjusters for a game
//-------------------------------------------------

void info_xml_creator::output_adjusters(const ioport_list &portlist)
{
	// iterate looking for Adjusters
	for (auto &port : portlist)
		for (ioport_field &field : port.second->fields())
			if (field.type() == IPT_ADJUSTER)
				fprintf(m_output, "\t\t<adjuster name=\"%s\" default=\"%d\"/>\n", util::xml::normalize_string(field.name()), field.defvalue());
}


//-------------------------------------------------
//  output_driver - print driver status
//-------------------------------------------------

void info_xml_creator::output_driver(game_driver const &driver)
{
	fprintf(m_output, "\t\t<driver");

	/* The status entry is an hint for frontend authors */
	/* to select working and not working games without */
	/* the need to know all the other status entries. */
	/* Games marked as status=good are perfectly emulated, games */
	/* marked as status=imperfect are emulated with only */
	/* some minor issues, games marked as status=preliminary */
	/* don't work or have major emulation problems. */

	u32 const flags = driver.flags;
	if (flags & (MACHINE_NOT_WORKING | MACHINE_UNEMULATED_PROTECTION | MACHINE_NO_SOUND | MACHINE_WRONG_COLORS | MACHINE_MECHANICAL))
		fprintf(m_output, " status=\"preliminary\"");
	else if (flags & (MACHINE_IMPERFECT_COLORS | MACHINE_IMPERFECT_SOUND | MACHINE_IMPERFECT_GRAPHICS))
		fprintf(m_output, " status=\"imperfect\"");
	else
		fprintf(m_output, " status=\"good\"");

	if (flags & MACHINE_NOT_WORKING)
		fprintf(m_output, " emulation=\"preliminary\"");
	else
		fprintf(m_output, " emulation=\"good\"");

	if (flags & MACHINE_WRONG_COLORS)
		fprintf(m_output, " color=\"preliminary\"");
	else if (flags & MACHINE_IMPERFECT_COLORS)
		fprintf(m_output, " color=\"imperfect\"");
	else
		fprintf(m_output, " color=\"good\"");

	if (flags & MACHINE_NO_SOUND)
		fprintf(m_output, " sound=\"preliminary\"");
	else if (flags & MACHINE_IMPERFECT_SOUND)
		fprintf(m_output, " sound=\"imperfect\"");
	else
		fprintf(m_output, " sound=\"good\"");

	if (flags & MACHINE_IMPERFECT_GRAPHICS)
		fprintf(m_output, " graphic=\"imperfect\"");
	else
		fprintf(m_output, " graphic=\"good\"");

	if (flags & MACHINE_NO_COCKTAIL)
		fprintf(m_output, " cocktail=\"preliminary\"");

	if (flags & MACHINE_UNEMULATED_PROTECTION)
		fprintf(m_output, " protection=\"preliminary\"");

	if (flags & MACHINE_SUPPORTS_SAVE)
		fprintf(m_output, " savestate=\"supported\"");
	else
		fprintf(m_output, " savestate=\"unsupported\"");

	fprintf(m_output, "/>\n");
}


//-------------------------------------------------
//  output_images - prints m_output all info on
//  image devices
//-------------------------------------------------

void info_xml_creator::output_images(device_t &device, const char *root_tag)
{
	for (const device_image_interface &imagedev : image_interface_iterator(device))
	{
		if (strcmp(imagedev.device().tag(), device.tag()))
		{
			bool loadable = imagedev.user_loadable();
			std::string newtag(imagedev.device().tag()), oldtag(":");
			newtag = newtag.substr(newtag.find(oldtag.append(root_tag)) + oldtag.length());

			// print m_output device type
			fprintf(m_output, "\t\t<device type=\"%s\"", util::xml::normalize_string(imagedev.image_type_name()));

			// does this device have a tag?
			if (imagedev.device().tag())
				fprintf(m_output, " tag=\"%s\"", util::xml::normalize_string(newtag.c_str()));

			// is this device available as media switch?
			if (!loadable)
				fprintf(m_output, " fixed_image=\"1\"");

			// is this device mandatory?
			if (imagedev.must_be_loaded())
				fprintf(m_output, " mandatory=\"1\"");

			if (imagedev.image_interface() && imagedev.image_interface()[0])
				fprintf(m_output, " interface=\"%s\"", util::xml::normalize_string(imagedev.image_interface()));

			// close the XML tag
			fprintf(m_output, ">\n");

			if (loadable)
			{
				const char *name = imagedev.instance_name().c_str();
				const char *shortname = imagedev.brief_instance_name().c_str();

				fprintf(m_output, "\t\t\t<instance");
				fprintf(m_output, " name=\"%s\"", util::xml::normalize_string(name));
				fprintf(m_output, " briefname=\"%s\"", util::xml::normalize_string(shortname));
				fprintf(m_output, "/>\n");

				std::string extensions(imagedev.file_extensions());

				char *ext = strtok((char *)extensions.c_str(), ",");
				while (ext != nullptr)
				{
					fprintf(m_output, "\t\t\t<extension");
					fprintf(m_output, " name=\"%s\"", util::xml::normalize_string(ext));
					fprintf(m_output, "/>\n");
					ext = strtok(nullptr, ",");
				}
			}
			fprintf(m_output, "\t\t</device>\n");
		}
	}
}


//-------------------------------------------------
//  output_images - prints all info about slots
//-------------------------------------------------

void info_xml_creator::output_slots(machine_config &config, device_t &device, const char *root_tag, device_type_set *devtypes)
{
	for (device_slot_interface &slot : slot_interface_iterator(device))
	{
		// shall we list fixed slots as non-configurable?
		bool const listed(!slot.fixed() && strcmp(slot.device().tag(), device.tag()));

		if (devtypes || listed)
		{
			std::string newtag(slot.device().tag()), oldtag(":");
			newtag = newtag.substr(newtag.find(oldtag.append(root_tag)) + oldtag.length());

			// print m_output device type
			if (listed)
				fprintf(m_output, "\t\t<slot name=\"%s\">\n", util::xml::normalize_string(newtag.c_str()));

			/*
			 if (listed && slot.slot_interface()[0])
			 fprintf(m_output, " interface=\"%s\"", util::xml::normalize_string(slot.slot_interface()));
			 */

			for (auto &option : slot.option_list())
			{
				if (devtypes || (listed && option.second->selectable()))
				{
					device_t *const dev = config.device_add(&slot.device(), "_dummy", option.second->devtype(), option.second->clock());
					if (!dev->configured())
						dev->config_complete();

					if (devtypes)
						for (device_t &subdevice : device_iterator(*dev)) devtypes->insert(&subdevice.type());

					if (listed && option.second->selectable())
					{
						fprintf(m_output, "\t\t\t<slotoption");
						fprintf(m_output, " name=\"%s\"", util::xml::normalize_string(option.second->name()));
						fprintf(m_output, " devname=\"%s\"", util::xml::normalize_string(dev->shortname()));
						if (slot.default_option() != nullptr && strcmp(slot.default_option(), option.second->name())==0)
							fprintf(m_output, " default=\"yes\"");
						fprintf(m_output, "/>\n");
					}

					config.device_remove(&slot.device(), "_dummy");
				}
			}

			if (listed)
				fprintf(m_output, "\t\t</slot>\n");
		}
	}
}


//-------------------------------------------------
//  output_software_list - print the information
//  for all known software lists for this system
//-------------------------------------------------

void info_xml_creator::output_software_list(device_t &root)
{
	for (const software_list_device &swlist : software_list_device_iterator(root))
	{
		fprintf(m_output, "\t\t<softwarelist name=\"%s\" ", swlist.list_name().c_str());
		fprintf(m_output, "status=\"%s\" ", (swlist.list_type() == SOFTWARE_LIST_ORIGINAL_SYSTEM) ? "original" : "compatible");
		if (swlist.filter())
			fprintf(m_output, "filter=\"%s\" ", swlist.filter());
		fprintf(m_output, "/>\n");
	}
}



//-------------------------------------------------
//  output_ramoptions - prints m_output all RAM
//  options for this system
//-------------------------------------------------

void info_xml_creator::output_ramoptions(device_t &root)
{
	for (const ram_device &ram : ram_device_iterator(root))
	{
		for (uint32_t option : ram.extra_options())
		{
			if (option == ram.default_size())
				fprintf(m_output, "\t\t<ramoption default=\"1\">%u</ramoption>\n", option);
			else
				fprintf(m_output, "\t\t<ramoption>%u</ramoption>\n", option);
		}
	}
}


//-------------------------------------------------
//  get_merge_name - get the rom name from a
//  parent set
//-------------------------------------------------

const char *info_xml_creator::get_merge_name(driver_enumerator &drivlist, util::hash_collection const &romhashes)
{
	// walk the parent chain
	const char *merge_name = nullptr;
	for (int clone_of = drivlist.find(drivlist.driver().parent); clone_of != -1; clone_of = drivlist.find(drivlist.driver(clone_of).parent))
	{
		// look in the parent's ROMs
		device_t *device = &drivlist.config(clone_of, m_lookup_options)->root_device();
		for (const rom_entry *pregion = rom_first_region(*device); pregion != nullptr; pregion = rom_next_region(pregion))
			for (const rom_entry *prom = rom_first_file(pregion); prom != nullptr; prom = rom_next_file(prom))
			{
				util::hash_collection phashes(ROM_GETHASHDATA(prom));
				if (!phashes.flag(util::hash_collection::FLAG_NO_DUMP) && romhashes == phashes)
				{
					// stop when we find a match
					merge_name = ROM_GETNAME(prom);
					break;
				}
			}
	}

	return merge_name;
}
