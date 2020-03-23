// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    distate.c

    Device state interfaces.

***************************************************************************/

#include "emu.h"


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

const u64 device_state_entry::k_decimal_divisor[] =
{
	1U,
	10U,
	100U,
	1000U,
	10000U,
	100000U,
	1000000U,
	10000000U,
	100000000U,
	1000000000U,
	10000000000U,
	100000000000U,
	1000000000000U,
	10000000000000U,
	100000000000000U,
	1000000000000000U,
	10000000000000000U,
	100000000000000000U,
	1000000000000000000U,
	10000000000000000000U
};



//**************************************************************************
//  DEVICE STATE ENTRY
//**************************************************************************

//-------------------------------------------------
//  device_state_entry - constructor
//-------------------------------------------------

device_state_entry::device_state_entry(int index, const char *symbol, void *dataptr, u8 size, device_state_interface *dev)
	: m_device_state(dev),
		m_index(index),
		m_dataptr(dataptr),
		m_datamask(0),
		m_datasize(size),
		m_flags(0),
		m_symbol(symbol),
		m_default_format(true),
		m_sizemask(0)
{
	// convert the size to a mask
	assert(size == 1 || size == 2 || size == 4 || size == 8);
	if (size == 1)
		m_sizemask = 0xff;
	else if (size == 2)
		m_sizemask = 0xffff;
	else if (size == 4)
		m_sizemask = 0xffffffff;
	else
		m_sizemask = ~u64(0);

	// default the data mask to the same
	m_datamask = m_sizemask;
	format_from_mask();

	// override well-known symbols
	if (index == STATE_GENPCBASE)
		m_symbol.assign("CURPC");
	else if (index == STATE_GENSP)
		m_symbol.assign("CURSP");
	else if (index == STATE_GENFLAGS)
		m_symbol.assign("CURFLAGS");
}

device_state_entry::device_state_entry(int index, device_state_interface *dev)
	: m_device_state(dev),
		m_index(index),
		m_dataptr(nullptr),
		m_datamask(0),
		m_datasize(0),
		m_flags(DSF_DIVIDER),
		m_symbol(),
		m_default_format(true),
		m_sizemask(0)
{
}


//-------------------------------------------------
//  formatstr - specify a format string
//-------------------------------------------------

device_state_entry &device_state_entry::formatstr(const char *_format)
{
	m_format.assign(_format);
	m_default_format = false;

	// set the DSF_CUSTOM_STRING flag by formatting with a nullptr string
	m_flags &= ~DSF_CUSTOM_STRING;
	format(nullptr);

	return *this;
}


//-------------------------------------------------
//  format_from_mask - make a format based on
//  the data mask
//-------------------------------------------------

void device_state_entry::format_from_mask()
{
	// skip if we have a user-provided format
	if (!m_default_format)
		return;

	// make up a format based on the mask
	int width = 0;
	for (u64 tempmask = m_datamask; tempmask != 0; tempmask >>= 4)
		width++;
	m_format = string_format("%%0%dX", width);
}


//-------------------------------------------------
//  value - return the current value as a u64
//-------------------------------------------------

u64 device_state_entry::value() const
{
	// pick up the value
	u64 result;
	switch (m_datasize)
	{
		default:
		case 1: result = *static_cast<u8  *>(m_dataptr); break;
		case 2: result = *static_cast<u16 *>(m_dataptr); break;
		case 4: result = *static_cast<u32 *>(m_dataptr); break;
		case 8: result = *static_cast<u64 *>(m_dataptr); break;
	}
	return result & m_datamask;
}


//-------------------------------------------------
//  format - return the value of the given
//  pieces of indexed state as a string
//-------------------------------------------------

std::string device_state_entry::format(const char *string, bool maxout) const
{
	std::string dest;
	u64 result = value();

	// parse the format
	bool leadzero = false;
	bool percent = false;
	bool explicitsign = false;
	bool hitnonzero;
	bool reset = true;
	int width = 0;
	for (const char *fptr = m_format.c_str(); *fptr != 0; fptr++)
	{
		// reset any accumulated state
		if (reset)
		{
			leadzero = maxout;
			percent = explicitsign = reset = false;
			width = 0;
		}

		// if we're not within a format, then anything other than a % outputs directly
		if (!percent && *fptr != '%')
		{
			dest.append(fptr, 1);
			continue;
		}

		// handle each character in turn
		switch (*fptr)
		{
			// % starts a format; %% outputs a single %
			case '%':
				if (!percent)
					percent = true;
				else
				{
					dest.append(fptr, 1);
					percent = false;
				}
				break;

			// 0 means insert leading 0s, unless it follows another width digit
			case '0':
				if (width == 0)
					leadzero = true;
				else
					width *= 10;
				break;

			// 1-9 accumulate into the width
			case '1':   case '2':   case '3':   case '4':   case '5':
			case '6':   case '7':   case '8':   case '9':
				width = width * 10 + (*fptr - '0');
				break;

			// + means explicit sign
			case '+':
				explicitsign = true;
				break;

			// X outputs as hexadecimal
			case 'X':
				if (width == 0)
					throw emu_fatalerror("Width required for %%X formats\n");
				hitnonzero = false;
				while (leadzero && width > 16)
				{
					dest.append(" ");
					width--;
				}
				for (int digitnum = 15; digitnum >= 0; digitnum--)
				{
					int digit = (result >> (4 * digitnum)) & 0x0f;
					if (digit != 0)
					{
						static const char hexchars[] = "0123456789ABCDEF";
						dest.append(&hexchars[digit], 1);
						hitnonzero = true;
					}
					else if (hitnonzero || (leadzero && digitnum < width) || digitnum == 0)
						dest.append("0");
				}
				reset = true;
				break;

			// O outputs as octal
			case 'O':
				if (width == 0)
					throw emu_fatalerror("Width required for %%O formats\n");
				hitnonzero = false;
				while (leadzero && width > 22)
				{
					dest.append(" ");
					width--;
				}
				for (int digitnum = 21; digitnum >= 0; digitnum--)
				{
					int digit = (result >> (3 * digitnum)) & 07;
					if (digit != 0)
					{
						static const char octchars[] = "01234567";
						dest.append(&octchars[digit], 1);
						hitnonzero = true;
					}
					else if (hitnonzero || (leadzero && digitnum < width) || digitnum == 0)
						dest.append("0");
				}
				reset = true;
				break;

			// d outputs as signed decimal
			case 'd':
				if (width == 0)
					throw emu_fatalerror("Width required for %%d formats\n");
				if ((result & m_datamask) > (m_datamask >> 1))
				{
					result = -result & m_datamask;
					dest.append("-");
					width--;
				}
				else if (explicitsign)
				{
					dest.append("+");
					width--;
				}
				// fall through to unsigned case

			// u outputs as unsigned decimal
			case 'u':
				if (width == 0)
					throw emu_fatalerror("Width required for %%u formats\n");
				hitnonzero = false;
				while (leadzero && width > ARRAY_LENGTH(k_decimal_divisor))
				{
					dest.append(" ");
					width--;
				}
				for (int digitnum = ARRAY_LENGTH(k_decimal_divisor) - 1; digitnum >= 0; digitnum--)
				{
					int digit = (result >= k_decimal_divisor[digitnum]) ? (result / k_decimal_divisor[digitnum]) % 10 : 0;
					if (digit != 0)
					{
						static const char decchars[] = "0123456789";
						dest.append(&decchars[digit], 1);
						hitnonzero = true;
					}
					else if (hitnonzero || (leadzero && digitnum < width) || digitnum == 0)
						dest.append("0");
				}
				reset = true;
				break;

			// s outputs a custom string
			case 's':
				if (width == 0)
					throw emu_fatalerror("Width required for %%s formats\n");
				if (string == nullptr)
				{
					const_cast<device_state_entry *>(this)->m_flags |= DSF_CUSTOM_STRING;
					return dest;
				}
				if (strlen(string) <= width)
				{
					dest.append(string);
					width -= strlen(string);
					while (width-- != 0)
						dest.append(" ");
				}
				else
					dest.append(string, width);
				reset = true;
				break;

			// other formats unknown
			default:
				throw emu_fatalerror("Unknown format character '%c'\n", *fptr);
		}
	}
	return dest;
}


//-------------------------------------------------
//  set_value - set the value from a u64
//-------------------------------------------------

void device_state_entry::set_value(u64 value) const
{
	// apply the mask
	value &= m_datamask;

	// sign-extend if necessary
	if ((m_flags & DSF_IMPORT_SEXT) != 0 && value > (m_datamask >> 1))
		value |= ~m_datamask;

	// store the value
	switch (m_datasize)
	{
		default:
		case 1: *static_cast<u8  *>(m_dataptr) = value; break;
		case 2: *static_cast<u16 *>(m_dataptr) = value; break;
		case 4: *static_cast<u32 *>(m_dataptr) = value; break;
		case 8: *static_cast<u64 *>(m_dataptr) = value; break;
	}
}


//-------------------------------------------------
//  set_value - set the value from a string
//-------------------------------------------------

void device_state_entry::set_value(const char *string) const
{
	// not implemented
}



//**************************************************************************
//  DEVICE STATE INTERFACE
//**************************************************************************

//-------------------------------------------------
//  device_state_interface - constructor
//-------------------------------------------------

device_state_interface::device_state_interface(const machine_config &mconfig, device_t &device)
	: device_interface(device, "state")
{
	memset(m_fast_state, 0, sizeof(m_fast_state));

	// configure the fast accessor
	device.interfaces().m_state = this;
}


//-------------------------------------------------
//  ~device_state_interface - destructor
//-------------------------------------------------

device_state_interface::~device_state_interface()
{
}


//-------------------------------------------------
//  state_int - return the value of the given piece
//  of indexed state as a u64
//-------------------------------------------------

u64 device_state_interface::state_int(int index)
{
	// nullptr or out-of-range entry returns 0
	const device_state_entry *entry = state_find_entry(index);
	if (entry == nullptr)
		return 0;

	// call the exporter before we do anything
	if (entry->needs_export())
		state_export(*entry);

	// pick up the value
	return entry->value();
}


//-------------------------------------------------
//  state_string - return the value of the given
//  pieces of indexed state as a string
//-------------------------------------------------

std::string device_state_interface::state_string(int index) const
{
	// nullptr or out-of-range entry returns bogus string
	const device_state_entry *entry = state_find_entry(index);
	if (entry == nullptr)
		return std::string("???");

	// get the custom string if needed
	std::string custom;
	if (entry->needs_custom_string())
		state_string_export(*entry, custom);

	// ask the entry to format itself
	return entry->format(custom.c_str());
}


//-------------------------------------------------
//  state_string_max_length - return the maximum
//  length of the given state string
//-------------------------------------------------

int device_state_interface::state_string_max_length(int index)
{
	// nullptr or out-of-range entry returns bogus string
	const device_state_entry *entry = state_find_entry(index);
	if (entry == nullptr)
		return 3;

	// ask the entry to format itself maximally
	return entry->format("", true).length();
}


//-------------------------------------------------
//  set_state_int - set the value of the given
//  piece of indexed state from a u64
//-------------------------------------------------

void device_state_interface::set_state_int(int index, u64 value)
{
	// nullptr or out-of-range entry is a no-op
	const device_state_entry *entry = state_find_entry(index);
	if (entry == nullptr)
		return;

	// set the value
	entry->set_value(value);

	// call the importer to finish up
	if (entry->needs_import())
		state_import(*entry);
}


//-------------------------------------------------
//  set_state - set the value of the given piece
//  of indexed state from a string
//-------------------------------------------------

void device_state_interface::set_state_string(int index, const char *string)
{
	// nullptr or out-of-range entry is a no-op
	const device_state_entry *entry = state_find_entry(index);
	if (entry == nullptr)
		return;

	// set the value
	entry->set_value(string);

	// call the importer to finish up
	if (entry->needs_import())
		state_import(*entry);
}


//-------------------------------------------------
//  state_add - return the value of the given
//  pieces of indexed state as a u64
//-------------------------------------------------

device_state_entry &device_state_interface::state_add(int index, const char *symbol, void *data, u8 size)
{
	// assert validity of incoming parameters
	assert(size == 1 || size == 2 || size == 4 || size == 8);
	assert(symbol != nullptr);

	// append to the end of the list
	m_state_list.push_back(std::make_unique<device_state_entry>(index, symbol, data, size, this));

	// set the fast entry if applicable
	if (index >= FAST_STATE_MIN && index <= FAST_STATE_MAX)
		m_fast_state[index - FAST_STATE_MIN] = m_state_list.back().get();

	return *m_state_list.back().get();
}

//-------------------------------------------------
//  state_add_divider - add a simple divider
//  entry
//-------------------------------------------------

device_state_entry &device_state_interface::state_add_divider(int index)
{
	// append to the end of the list
	m_state_list.push_back(std::make_unique<device_state_entry>(index, this));

	return *m_state_list.back().get();
}

//-------------------------------------------------
//  state_import - called after new state is
//  written to perform any post-processing
//-------------------------------------------------

void device_state_interface::state_import(const device_state_entry &entry)
{
	// do nothing by default
}


//-------------------------------------------------
//  state_export - called prior to new state
//  reading the state
//-------------------------------------------------

void device_state_interface::state_export(const device_state_entry &entry)
{
	// do nothing by default
}


//-------------------------------------------------
//  state_string_import - called after new state is
//  written to perform any post-processing
//-------------------------------------------------

void device_state_interface::state_string_import(const device_state_entry &entry, std::string &str)
{
	// do nothing by default
}


//-------------------------------------------------
//  state_string_export - called after new state is
//  written to perform any post-processing
//-------------------------------------------------

void device_state_interface::state_string_export(const device_state_entry &entry, std::string &str) const
{
	// do nothing by default
}


//-------------------------------------------------
//  interface_post_start - verify that state was
//  properly set up
//-------------------------------------------------

void device_state_interface::interface_post_start()
{
	// make sure we got something during startup
	if (m_state_list.size() == 0)
		throw emu_fatalerror("No state registered for device '%s' that supports it!", m_device.tag());
}


//-------------------------------------------------
//  state_find_entry - return a pointer to the
//  state entry for the given index
//-------------------------------------------------

const device_state_entry *device_state_interface::state_find_entry(int index) const
{
	// use fast lookup if possible
	if (index >= FAST_STATE_MIN && index <= FAST_STATE_MAX)
		return m_fast_state[index - FAST_STATE_MIN];

	// otherwise, scan the first
	for (auto &entry : m_state_list)
		if (entry->m_index == index)
			return entry.get();

	// handle failure by returning nullptr
	return nullptr;
}
