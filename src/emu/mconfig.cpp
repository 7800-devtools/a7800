// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    mconfig.c

    Machine configuration macros and functions.

***************************************************************************/

#include "emu.h"
#include "emuopts.h"
#include "screen.h"

#include <ctype.h>


//**************************************************************************
//  MACHINE CONFIGURATIONS
//**************************************************************************

//-------------------------------------------------
//  machine_config - constructor
//-------------------------------------------------

machine_config::machine_config(const game_driver &gamedrv, emu_options &options)
	: m_minimum_quantum(attotime::zero),
		m_default_layout(nullptr),
		m_gamedrv(gamedrv),
		m_options(options)
{
	// add the root device
	device_add(nullptr, "root", gamedrv.type, 0);

	// intialize slot devices - make sure that any required devices have been allocated
	for (device_slot_interface &slot : slot_interface_iterator(root_device()))
	{
		device_t &owner = slot.device();
		const char *slot_option_name = owner.tag() + 1;

		// figure out which device goes into this slot
		bool has_option = options.has_slot_option(slot_option_name);
		const char *selval;
		bool is_default;
		if (!has_option)
		{
			// The only time we should be getting here is when emuopts.cpp is invoking
			// us to evaluate slot/image options, and the internal state of emuopts.cpp has
			// not caught up yet
			selval = slot.default_option();
			is_default = true;
		}
		else
		{
			const slot_option &opt = options.slot_option(slot_option_name);
			selval = opt.value().c_str();
			is_default = !opt.specified();
		}

		if (selval && *selval)
		{
			const device_slot_option *option = slot.option(selval);

			if (option && (is_default || option->selectable()))
			{
				// create the device
				device_t *new_dev = device_add(&owner, option->name(), option->devtype(), option->clock());
				slot.set_card_device(new_dev);

				const char *default_bios = option->default_bios();
				if (default_bios != nullptr)
					device_t::static_set_default_bios_tag(*new_dev, default_bios);

				machine_config_constructor additions = option->machine_config();
				if (additions != nullptr)
					(*additions)(*this, new_dev, new_dev);

				const input_device_default *input_device_defaults = option->input_device_defaults();
				if (input_device_defaults)
					device_t::static_set_input_default(*new_dev, input_device_defaults);
			}
			else
				throw emu_fatalerror("Unknown slot option '%s' in slot '%s'", selval, owner.tag()+1);
		}
	}

	// then notify all devices that their configuration is complete
	for (device_t &device : device_iterator(root_device()))
		if (!device.configured())
			device.config_complete();
}


//-------------------------------------------------
//  ~machine_config - destructor
//-------------------------------------------------

machine_config::~machine_config()
{
}


//-------------------------------------------------
//  first_screen - return a pointer to the first
//  screen device
//-------------------------------------------------

screen_device *machine_config::first_screen() const
{
	return screen_device_iterator(root_device()).first();
}


//-------------------------------------------------
//  device_add - configuration helper to add a
//  new device
//-------------------------------------------------

device_t *machine_config::device_add(device_t *owner, const char *tag, device_type type, u32 clock)
{
	const char *orig_tag = tag;

	// if the device path is absolute, start from the root
	if (tag[0] == ':')
	{
		tag++;
		owner = m_root_device.get();
	}

	// go down the path until we're done with it
	while (strchr(tag, ':'))
	{
		const char *next = strchr(tag, ':');
		assert(next != tag);
		std::string part(tag, next-tag);
		owner = owner->subdevices().find(part);
		if (owner == nullptr)
			throw emu_fatalerror("Could not find %s when looking up path for device %s\n", part.c_str(), orig_tag);
		tag = next+1;
	}
	assert(tag[0] != '\0');

	if (owner != nullptr)
	{
		// allocate the new device and append it to the owner's list
		device_t *const device = &owner->subdevices().m_list.append(*type(*this, tag, owner, clock).release());
		device->add_machine_configuration(*this);
		return device;
	}
	else
	{
		// allocate the root device directly
		assert(!m_root_device);
		m_root_device = type(*this, tag, nullptr, clock);
		driver_device *driver = dynamic_cast<driver_device *>(m_root_device.get());
		if (driver)
			driver->set_game_driver(m_gamedrv);
		m_root_device->add_machine_configuration(*this);
		return m_root_device.get();
	}
}


//-------------------------------------------------
//  device_replace - configuration helper to
//  replace one device with a new device
//-------------------------------------------------

device_t *machine_config::device_replace(device_t *owner, const char *tag, device_type type, u32 clock)
{
	// find the original device by relative tag (must exist)
	assert(owner != nullptr);
	device_t *old_device = owner->subdevice(tag);
	if (old_device == nullptr)
	{
		osd_printf_warning("Warning: attempting to replace non-existent device '%s'\n", tag);
		return device_add(owner, tag, type, clock);
	}
	else
	{
		// make sure we have the old device's actual owner
		owner = old_device->owner();
		assert(owner != nullptr);

		// remove references to the old device
		remove_references(*old_device);

		// allocate the new device and substitute it for the old one in the owner's list
		device_t *const new_device = &owner->subdevices().m_list.replace_and_remove(*type(*this, tag, owner, clock).release(), *old_device);
		new_device->add_machine_configuration(*this);
		return new_device;
	}
}


//-------------------------------------------------
//  device_remove - configuration helper to
//  remove a device
//-------------------------------------------------

device_t *machine_config::device_remove(device_t *owner, const char *tag)
{
	// find the original device by relative tag (must exist)
	assert(owner != nullptr);
	device_t *device = owner->subdevice(tag);
	if (device == nullptr)
	{
		osd_printf_warning("Warning: attempting to remove non-existent device '%s'\n", tag);
		return nullptr;
	}

	// make sure we have the old device's actual owner
	owner = device->owner();
	assert(owner != nullptr);

	// remove references to the old device
	remove_references(*device);

	// let the device's owner do the work
	owner->subdevices().m_list.remove(*device);
	return nullptr;
}


//-------------------------------------------------
//  device_find - configuration helper to
//  locate a device
//-------------------------------------------------

device_t *machine_config::device_find(device_t *owner, const char *tag)
{
	// find the original device by relative tag (must exist)
	assert(owner != nullptr);
	device_t *device = owner->subdevice(tag);
	if (device == nullptr)
		throw emu_fatalerror("Unable to find device '%s'\n", tag);

	// return the device
	return device;
}


//-------------------------------------------------
//  remove_references - globally remove references
//  to a device about to be removed from the tree
//-------------------------------------------------

void machine_config::remove_references(ATTR_UNUSED device_t &device)
{
	// iterate over all devices and remove any references
	for (device_t &scan : device_iterator(root_device()))
		scan.subdevices().m_tagmap.clear(); //remove(&device);
}
