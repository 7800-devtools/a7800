// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    input.c

    Handle input from the user.

****************************************************************************

    To do:
        * auto-selecting joystick configs
        * per-joystick configs?
        * test half-axis selections
        * add input test menu
        * get rid of osd_customize_inputport_list

***************************************************************************/

#include "emu.h"
#include "inputdev.h"



//**************************************************************************
//  CONSTANTS
//**************************************************************************

// invalid memory value for axis polling
const s32 INVALID_AXIS_VALUE      = 0x7fffffff;

// additional expanded input codes for sequences
const input_code input_seq::end_code(DEVICE_CLASS_INTERNAL, 0, ITEM_CLASS_INVALID, ITEM_MODIFIER_NONE, ITEM_ID_SEQ_END);
const input_code input_seq::default_code(DEVICE_CLASS_INTERNAL, 0, ITEM_CLASS_INVALID, ITEM_MODIFIER_NONE, ITEM_ID_SEQ_DEFAULT);
const input_code input_seq::not_code(DEVICE_CLASS_INTERNAL, 0, ITEM_CLASS_INVALID, ITEM_MODIFIER_NONE, ITEM_ID_SEQ_NOT);
const input_code input_seq::or_code(DEVICE_CLASS_INTERNAL, 0, ITEM_CLASS_INVALID, ITEM_MODIFIER_NONE, ITEM_ID_SEQ_OR);

// constant sequences
const input_seq input_seq::empty_seq;



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> code_string_table

// simple class to match codes to strings
struct code_string_table
{
	u32 operator[](const char *string) const
	{
		for (const code_string_table *current = this; current->m_code != ~0; current++)
			if (strcmp(current->m_string, string) == 0)
				return current->m_code;
		return ~0;
	}

	const char *operator[](u32 code) const
	{
		for (const code_string_table *current = this; current->m_code != ~0; current++)
			if (current->m_code == code)
				return current->m_string;
		return nullptr;
	}

	u32                     m_code;
	const char *            m_string;
};



//**************************************************************************
//  TOKEN/STRING TABLES
//**************************************************************************

// token strings for device classes
static const code_string_table devclass_token_table[] =
{
	{ DEVICE_CLASS_KEYBOARD, "KEYCODE" },
	{ DEVICE_CLASS_MOUSE,    "MOUSECODE" },
	{ DEVICE_CLASS_LIGHTGUN, "GUNCODE" },
	{ DEVICE_CLASS_JOYSTICK, "JOYCODE" },
	{ ~0U,                   "UNKCODE" }
};

// friendly strings for device classes
static const code_string_table devclass_string_table[] =
{
	{ DEVICE_CLASS_KEYBOARD, "Kbd" },
	{ DEVICE_CLASS_MOUSE,    "Mouse" },
	{ DEVICE_CLASS_LIGHTGUN, "Gun" },
	{ DEVICE_CLASS_JOYSTICK, "Joy" },
	{ ~0U,                   "Unk" }
};

// token strings for item modifiers
static const code_string_table modifier_token_table[] =
{
	{ ITEM_MODIFIER_POS,     "POS" },
	{ ITEM_MODIFIER_NEG,     "NEG" },
	{ ITEM_MODIFIER_LEFT,    "LEFT" },
	{ ITEM_MODIFIER_RIGHT,   "RIGHT" },
	{ ITEM_MODIFIER_UP,      "UP" },
	{ ITEM_MODIFIER_DOWN,    "DOWN" },
	{ ~0U,                   "" }
};

// friendly strings for item modifiers
static const code_string_table modifier_string_table[] =
{
	{ ITEM_MODIFIER_POS,     "+" },
	{ ITEM_MODIFIER_NEG,     "-" },
	{ ITEM_MODIFIER_LEFT,    "Left" },
	{ ITEM_MODIFIER_RIGHT,   "Right" },
	{ ITEM_MODIFIER_UP,      "Up" },
	{ ITEM_MODIFIER_DOWN,    "Down" },
	{ ~0U,                   "" }
};

// token strings for item classes
static const code_string_table itemclass_token_table[] =
{
	{ ITEM_CLASS_SWITCH,     "SWITCH" },
	{ ITEM_CLASS_ABSOLUTE,   "ABSOLUTE" },
	{ ITEM_CLASS_RELATIVE,   "RELATIVE" },
	{ ~0U,                   "" }
};

// token strings for standard item ids
static const code_string_table itemid_token_table[] =
{
	// standard keyboard codes
	{ ITEM_ID_A,             "A" },
	{ ITEM_ID_B,             "B" },
	{ ITEM_ID_C,             "C" },
	{ ITEM_ID_D,             "D" },
	{ ITEM_ID_E,             "E" },
	{ ITEM_ID_F,             "F" },
	{ ITEM_ID_G,             "G" },
	{ ITEM_ID_H,             "H" },
	{ ITEM_ID_I,             "I" },
	{ ITEM_ID_J,             "J" },
	{ ITEM_ID_K,             "K" },
	{ ITEM_ID_L,             "L" },
	{ ITEM_ID_M,             "M" },
	{ ITEM_ID_N,             "N" },
	{ ITEM_ID_O,             "O" },
	{ ITEM_ID_P,             "P" },
	{ ITEM_ID_Q,             "Q" },
	{ ITEM_ID_R,             "R" },
	{ ITEM_ID_S,             "S" },
	{ ITEM_ID_T,             "T" },
	{ ITEM_ID_U,             "U" },
	{ ITEM_ID_V,             "V" },
	{ ITEM_ID_W,             "W" },
	{ ITEM_ID_X,             "X" },
	{ ITEM_ID_Y,             "Y" },
	{ ITEM_ID_Z,             "Z" },
	{ ITEM_ID_0,             "0" },
	{ ITEM_ID_1,             "1" },
	{ ITEM_ID_2,             "2" },
	{ ITEM_ID_3,             "3" },
	{ ITEM_ID_4,             "4" },
	{ ITEM_ID_5,             "5" },
	{ ITEM_ID_6,             "6" },
	{ ITEM_ID_7,             "7" },
	{ ITEM_ID_8,             "8" },
	{ ITEM_ID_9,             "9" },
	{ ITEM_ID_F1,            "F1" },
	{ ITEM_ID_F2,            "F2" },
	{ ITEM_ID_F3,            "F3" },
	{ ITEM_ID_F4,            "F4" },
	{ ITEM_ID_F5,            "F5" },
	{ ITEM_ID_F6,            "F6" },
	{ ITEM_ID_F7,            "F7" },
	{ ITEM_ID_F8,            "F8" },
	{ ITEM_ID_F9,            "F9" },
	{ ITEM_ID_F10,           "F10" },
	{ ITEM_ID_F11,           "F11" },
	{ ITEM_ID_F12,           "F12" },
	{ ITEM_ID_F13,           "F13" },
	{ ITEM_ID_F14,           "F14" },
	{ ITEM_ID_F15,           "F15" },
	{ ITEM_ID_F16,           "F16" },
	{ ITEM_ID_F17,           "F17" },
	{ ITEM_ID_F18,           "F18" },
	{ ITEM_ID_F19,           "F19" },
	{ ITEM_ID_F20,           "F20" },
	{ ITEM_ID_ESC,           "ESC" },
	{ ITEM_ID_TILDE,         "TILDE" },
	{ ITEM_ID_MINUS,         "MINUS" },
	{ ITEM_ID_EQUALS,        "EQUALS" },
	{ ITEM_ID_BACKSPACE,     "BACKSPACE" },
	{ ITEM_ID_TAB,           "TAB" },
	{ ITEM_ID_OPENBRACE,     "OPENBRACE" },
	{ ITEM_ID_CLOSEBRACE,    "CLOSEBRACE" },
	{ ITEM_ID_ENTER,         "ENTER" },
	{ ITEM_ID_COLON,         "COLON" },
	{ ITEM_ID_QUOTE,         "QUOTE" },
	{ ITEM_ID_BACKSLASH,     "BACKSLASH" },
	{ ITEM_ID_BACKSLASH2,    "BACKSLASH2" },
	{ ITEM_ID_COMMA,         "COMMA" },
	{ ITEM_ID_STOP,          "STOP" },
	{ ITEM_ID_SLASH,         "SLASH" },
	{ ITEM_ID_SPACE,         "SPACE" },
	{ ITEM_ID_INSERT,        "INSERT" },
	{ ITEM_ID_DEL,           "DEL" },
	{ ITEM_ID_HOME,          "HOME" },
	{ ITEM_ID_END,           "END" },
	{ ITEM_ID_PGUP,          "PGUP" },
	{ ITEM_ID_PGDN,          "PGDN" },
	{ ITEM_ID_LEFT,          "LEFT" },
	{ ITEM_ID_RIGHT,         "RIGHT" },
	{ ITEM_ID_UP,            "UP" },
	{ ITEM_ID_DOWN,          "DOWN" },
	{ ITEM_ID_0_PAD,         "0PAD" },
	{ ITEM_ID_1_PAD,         "1PAD" },
	{ ITEM_ID_2_PAD,         "2PAD" },
	{ ITEM_ID_3_PAD,         "3PAD" },
	{ ITEM_ID_4_PAD,         "4PAD" },
	{ ITEM_ID_5_PAD,         "5PAD" },
	{ ITEM_ID_6_PAD,         "6PAD" },
	{ ITEM_ID_7_PAD,         "7PAD" },
	{ ITEM_ID_8_PAD,         "8PAD" },
	{ ITEM_ID_9_PAD,         "9PAD" },
	{ ITEM_ID_SLASH_PAD,     "SLASHPAD" },
	{ ITEM_ID_ASTERISK,      "ASTERISK" },
	{ ITEM_ID_MINUS_PAD,     "MINUSPAD" },
	{ ITEM_ID_PLUS_PAD,      "PLUSPAD" },
	{ ITEM_ID_DEL_PAD,       "DELPAD" },
	{ ITEM_ID_ENTER_PAD,     "ENTERPAD" },
	{ ITEM_ID_BS_PAD,        "BSPAD" },
	{ ITEM_ID_TAB_PAD,       "TABPAD" },
	{ ITEM_ID_00_PAD,        "00PAD" },
	{ ITEM_ID_000_PAD,       "000PAD" },
	{ ITEM_ID_PRTSCR,        "PRTSCR" },
	{ ITEM_ID_PAUSE,         "PAUSE" },
	{ ITEM_ID_LSHIFT,        "LSHIFT" },
	{ ITEM_ID_RSHIFT,        "RSHIFT" },
	{ ITEM_ID_LCONTROL,      "LCONTROL" },
	{ ITEM_ID_RCONTROL,      "RCONTROL" },
	{ ITEM_ID_LALT,          "LALT" },
	{ ITEM_ID_RALT,          "RALT" },
	{ ITEM_ID_SCRLOCK,       "SCRLOCK" },
	{ ITEM_ID_NUMLOCK,       "NUMLOCK" },
	{ ITEM_ID_CAPSLOCK,      "CAPSLOCK" },
	{ ITEM_ID_LWIN,          "LWIN" },
	{ ITEM_ID_RWIN,          "RWIN" },
	{ ITEM_ID_MENU,          "MENU" },
	{ ITEM_ID_CANCEL,        "CANCEL" },

	// standard mouse/joystick/gun codes
	{ ITEM_ID_XAXIS,         "XAXIS" },
	{ ITEM_ID_YAXIS,         "YAXIS" },
	{ ITEM_ID_ZAXIS,         "ZAXIS" },
	{ ITEM_ID_RXAXIS,        "RXAXIS" },
	{ ITEM_ID_RYAXIS,        "RYAXIS" },
	{ ITEM_ID_RZAXIS,        "RZAXIS" },
	{ ITEM_ID_SLIDER1,       "SLIDER1" },
	{ ITEM_ID_SLIDER2,       "SLIDER2" },
	{ ITEM_ID_BUTTON1,       "BUTTON1" },
	{ ITEM_ID_BUTTON2,       "BUTTON2" },
	{ ITEM_ID_BUTTON3,       "BUTTON3" },
	{ ITEM_ID_BUTTON4,       "BUTTON4" },
	{ ITEM_ID_BUTTON5,       "BUTTON5" },
	{ ITEM_ID_BUTTON6,       "BUTTON6" },
	{ ITEM_ID_BUTTON7,       "BUTTON7" },
	{ ITEM_ID_BUTTON8,       "BUTTON8" },
	{ ITEM_ID_BUTTON9,       "BUTTON9" },
	{ ITEM_ID_BUTTON10,      "BUTTON10" },
	{ ITEM_ID_BUTTON11,      "BUTTON11" },
	{ ITEM_ID_BUTTON12,      "BUTTON12" },
	{ ITEM_ID_BUTTON13,      "BUTTON13" },
	{ ITEM_ID_BUTTON14,      "BUTTON14" },
	{ ITEM_ID_BUTTON15,      "BUTTON15" },
	{ ITEM_ID_BUTTON16,      "BUTTON16" },
	{ ITEM_ID_BUTTON17,      "BUTTON17" },
	{ ITEM_ID_BUTTON18,      "BUTTON18" },
	{ ITEM_ID_BUTTON19,      "BUTTON19" },
	{ ITEM_ID_BUTTON20,      "BUTTON20" },
	{ ITEM_ID_BUTTON21,      "BUTTON21" },
	{ ITEM_ID_BUTTON22,      "BUTTON22" },
	{ ITEM_ID_BUTTON23,      "BUTTON23" },
	{ ITEM_ID_BUTTON24,      "BUTTON24" },
	{ ITEM_ID_BUTTON25,      "BUTTON25" },
	{ ITEM_ID_BUTTON26,      "BUTTON26" },
	{ ITEM_ID_BUTTON27,      "BUTTON27" },
	{ ITEM_ID_BUTTON28,      "BUTTON28" },
	{ ITEM_ID_BUTTON29,      "BUTTON29" },
	{ ITEM_ID_BUTTON30,      "BUTTON30" },
	{ ITEM_ID_BUTTON31,      "BUTTON31" },
	{ ITEM_ID_BUTTON32,      "BUTTON32" },
	{ ITEM_ID_START,         "START" },
	{ ITEM_ID_SELECT,        "SELECT" },

	// Hats
	{ ITEM_ID_HAT1UP,        "HAT1UP" },
	{ ITEM_ID_HAT1DOWN,      "HAT1DOWN" },
	{ ITEM_ID_HAT1LEFT,      "HAT1LEFT" },
	{ ITEM_ID_HAT1RIGHT,     "HAT1RIGHT" },
	{ ITEM_ID_HAT2UP,        "HAT2UP" },
	{ ITEM_ID_HAT2DOWN,      "HAT2DOWN" },
	{ ITEM_ID_HAT2LEFT,      "HAT2LEFT" },
	{ ITEM_ID_HAT2RIGHT,     "HAT2RIGHT" },
	{ ITEM_ID_HAT3UP,        "HAT3UP" },
	{ ITEM_ID_HAT3DOWN,      "HAT3DOWN" },
	{ ITEM_ID_HAT3LEFT,      "HAT3LEFT" },
	{ ITEM_ID_HAT3RIGHT,     "HAT3RIGHT" },
	{ ITEM_ID_HAT4UP,        "HAT4UP" },
	{ ITEM_ID_HAT4DOWN,      "HAT4DOWN" },
	{ ITEM_ID_HAT4LEFT,      "HAT4LEFT" },
	{ ITEM_ID_HAT4RIGHT,     "HAT4RIGHT" },

	// Additional IDs
	{ ITEM_ID_ADD_SWITCH1,   "ADDSW1" },
	{ ITEM_ID_ADD_SWITCH2,   "ADDSW2" },
	{ ITEM_ID_ADD_SWITCH3,   "ADDSW3" },
	{ ITEM_ID_ADD_SWITCH4,   "ADDSW4" },
	{ ITEM_ID_ADD_SWITCH5,   "ADDSW5" },
	{ ITEM_ID_ADD_SWITCH6,   "ADDSW6" },
	{ ITEM_ID_ADD_SWITCH7,   "ADDSW7" },
	{ ITEM_ID_ADD_SWITCH8,   "ADDSW8" },
	{ ITEM_ID_ADD_SWITCH9,   "ADDSW9" },
	{ ITEM_ID_ADD_SWITCH10,  "ADDSW10" },
	{ ITEM_ID_ADD_SWITCH11,  "ADDSW11" },
	{ ITEM_ID_ADD_SWITCH12,  "ADDSW12" },
	{ ITEM_ID_ADD_SWITCH13,  "ADDSW13" },
	{ ITEM_ID_ADD_SWITCH14,  "ADDSW14" },
	{ ITEM_ID_ADD_SWITCH15,  "ADDSW15" },
	{ ITEM_ID_ADD_SWITCH16,  "ADDSW16" },
	{ ITEM_ID_ADD_ABSOLUTE1, "ADDAXIS1" },
	{ ITEM_ID_ADD_ABSOLUTE2, "ADDAXIS2" },
	{ ITEM_ID_ADD_ABSOLUTE3, "ADDAXIS3" },
	{ ITEM_ID_ADD_ABSOLUTE4, "ADDAXIS4" },
	{ ITEM_ID_ADD_ABSOLUTE5, "ADDAXIS5" },
	{ ITEM_ID_ADD_ABSOLUTE6, "ADDAXIS6" },
	{ ITEM_ID_ADD_ABSOLUTE7, "ADDAXIS7" },
	{ ITEM_ID_ADD_ABSOLUTE8, "ADDAXIS8" },
	{ ITEM_ID_ADD_ABSOLUTE9, "ADDAXIS9" },
	{ ITEM_ID_ADD_ABSOLUTE10,"ADDAXIS10" },
	{ ITEM_ID_ADD_ABSOLUTE11,"ADDAXIS11" },
	{ ITEM_ID_ADD_ABSOLUTE12,"ADDAXIS12" },
	{ ITEM_ID_ADD_ABSOLUTE13,"ADDAXIS13" },
	{ ITEM_ID_ADD_ABSOLUTE14,"ADDAXIS14" },
	{ ITEM_ID_ADD_ABSOLUTE15,"ADDAXIS15" },
	{ ITEM_ID_ADD_ABSOLUTE16,"ADDAXIS16" },
	{ ITEM_ID_ADD_RELATIVE1, "ADDREL1" },
	{ ITEM_ID_ADD_RELATIVE2, "ADDREL2" },
	{ ITEM_ID_ADD_RELATIVE3, "ADDREL3" },
	{ ITEM_ID_ADD_RELATIVE4, "ADDREL4" },
	{ ITEM_ID_ADD_RELATIVE5, "ADDREL5" },
	{ ITEM_ID_ADD_RELATIVE6, "ADDREL6" },
	{ ITEM_ID_ADD_RELATIVE7, "ADDREL7" },
	{ ITEM_ID_ADD_RELATIVE8, "ADDREL8" },
	{ ITEM_ID_ADD_RELATIVE9, "ADDREL9" },
	{ ITEM_ID_ADD_RELATIVE10,"ADDREL10" },
	{ ITEM_ID_ADD_RELATIVE11,"ADDREL11" },
	{ ITEM_ID_ADD_RELATIVE12,"ADDREL12" },
	{ ITEM_ID_ADD_RELATIVE13,"ADDREL13" },
	{ ITEM_ID_ADD_RELATIVE14,"ADDREL14" },
	{ ITEM_ID_ADD_RELATIVE15,"ADDREL15" },
	{ ITEM_ID_ADD_RELATIVE16,"ADDREL16" },

	{ ~0U,                   nullptr }
};



//**************************************************************************
//  INPUT SEQ
//**************************************************************************

//-------------------------------------------------
//  operator+= - append a code to the end of an
//  input sequence
//-------------------------------------------------

input_seq &input_seq::operator+=(input_code code)
{
	// if not enough room, return false
	int curlength = length();
	if (curlength < ARRAY_LENGTH(m_code) - 1)
	{
		m_code[curlength++] = code;
		m_code[curlength] = end_code;
	}
	return *this;
}


//-------------------------------------------------
//  operator|= - append a code to a sequence; if
//  the sequence is non-empty, insert an OR
//  before the new code
//-------------------------------------------------

input_seq &input_seq::operator|=(input_code code)
{
	// overwrite end/default with the new code
	if (m_code[0] == end_code || m_code[0] == default_code)
		m_code[0] = code;

	// otherwise, append an OR token and then the new code
	else
	{
		*this += or_code;
		*this += code;
	}
	return *this;
}


//-------------------------------------------------
//  length - return the length of the sequence
//-------------------------------------------------

int input_seq::length() const
{
	// find the end token; error if none found
	for (int seqnum = 0; seqnum < ARRAY_LENGTH(m_code); seqnum++)
		if (m_code[seqnum] == end_code)
			return seqnum;
	return ARRAY_LENGTH(m_code);
}


//-------------------------------------------------
//  is_valid - return true if a given sequence is
//  valid
//-------------------------------------------------

bool input_seq::is_valid() const
{
	// "default" can only be of length 1
	if (m_code[0] == default_code)
		return (length() == 1);

	// scan the sequence for valid codes
	input_item_class lastclass = ITEM_CLASS_INVALID;
	input_code lastcode = INPUT_CODE_INVALID;
	int positive_code_count = 0;
	for (auto code : m_code)
	{
		// invalid codes are never permitted

		if (code == INPUT_CODE_INVALID)
			return false;

		// if we hit an OR or the end, validate the previous chunk
		if (code == or_code || code == end_code)
		{
			// must be at least one positive code
			if (positive_code_count == 0)
				return false;

			// last code must not have been an internal code
			if (lastcode.internal())
				return false;

			// if this is the end, we're ok
			if (code == end_code)
				return true;

			// reset the state for the next chunk
			positive_code_count = 0;
			lastclass = ITEM_CLASS_INVALID;
		}

		// if we hit a NOT, make sure we don't have a double
		else if (code == not_code)
		{
			if (lastcode == not_code)
				return false;
		}

		// anything else
		else
		{
			// count positive codes
			if (lastcode != not_code)
				positive_code_count++;

			// non-switch items can't have a NOT
			input_item_class itemclass = code.item_class();
			if (itemclass != ITEM_CLASS_SWITCH && lastcode == not_code)
				return false;

			// absolute/relative items must all be the same class
			if ((lastclass == ITEM_CLASS_ABSOLUTE && itemclass != ITEM_CLASS_ABSOLUTE) ||
				(lastclass == ITEM_CLASS_RELATIVE && itemclass != ITEM_CLASS_RELATIVE))
				return false;
		}

		// remember the last code
		lastcode = code;
	}

	// if we got here, we were missing an END token; fail
	return false;
}


//-------------------------------------------------
//  set - directly set up to the first 7 codes
//-------------------------------------------------

void input_seq::set(input_code code0, input_code code1, input_code code2, input_code code3, input_code code4, input_code code5, input_code code6)
{
	m_code[0] = code0;
	m_code[1] = code1;
	m_code[2] = code2;
	m_code[3] = code3;
	m_code[4] = code4;
	m_code[5] = code5;
	m_code[6] = code6;
	for (int codenum = 7; codenum < ARRAY_LENGTH(m_code); codenum++)
		m_code[codenum] = end_code;
}


//-------------------------------------------------
//  backspace - "backspace" over the last entry in
//  a sequence
//-------------------------------------------------

void input_seq::backspace()
{
	// if we have at least one entry, remove it
	int curlength = length();
	if (curlength > 0)
		m_code[curlength - 1] = end_code;
}


//-------------------------------------------------
//  replace - replace all instances of oldcode
//  with newcode in a sequence
//-------------------------------------------------

void input_seq::replace(input_code oldcode, input_code newcode)
{
	for (auto & elem : m_code)
		if (elem == oldcode)
			elem = newcode;
}



//**************************************************************************
//  INPUT MANAGER
//**************************************************************************

//-------------------------------------------------
//  input_manager - constructor
//-------------------------------------------------

input_manager::input_manager(running_machine &machine)
	: m_machine(machine),
		m_poll_seq_last_ticks(0),
		m_poll_seq_class(ITEM_CLASS_SWITCH)
{
	// reset code memory
	reset_memory();

	// create pointers for the classes
	m_class[DEVICE_CLASS_KEYBOARD] = std::make_unique<input_class_keyboard>(*this);
	m_class[DEVICE_CLASS_MOUSE] = std::make_unique<input_class_mouse>(*this);
	m_class[DEVICE_CLASS_LIGHTGUN] = std::make_unique<input_class_lightgun>(*this);
	m_class[DEVICE_CLASS_JOYSTICK] = std::make_unique<input_class_joystick>(*this);

#ifdef MAME_DEBUG
	for (input_device_class devclass = DEVICE_CLASS_FIRST_VALID; devclass <= DEVICE_CLASS_LAST_VALID; ++devclass)
	{
		assert(m_class[devclass] != nullptr);
		assert(m_class[devclass]->devclass() == devclass);
	}
#endif
}


//-------------------------------------------------
//  input_manager - destructor
//-------------------------------------------------

input_manager::~input_manager()
{
}


//-------------------------------------------------
//  code_value - return the value of a given
//  input code
//-------------------------------------------------

s32 input_manager::code_value(input_code code)
{
	g_profiler.start(PROFILER_INPUT);
	s32 result = 0;

	// dummy loop to allow clean early exits
	do
	{
		// return 0 for any invalid devices
		input_device *device = device_from_code(code);
		if (device == nullptr)
			break;

		// also return 0 if the device class is disabled
		input_class &devclass = *m_class[code.device_class()];
		if (!devclass.enabled())
			break;

		// if this is not a multi device, only return data for item 0 and iterate over all
		int startindex = code.device_index();
		int stopindex = startindex;
		if (!devclass.multi())
		{
			if (startindex != 0)
				break;
			stopindex = devclass.maxindex();
		}

		// iterate over all device indices
		input_item_class targetclass = code.item_class();
		for (int curindex = startindex; curindex <= stopindex; curindex++)
		{
			// lookup the item for the appropriate index
			code.set_device_index(curindex);
			input_device_item *item = item_from_code(code);
			if (item == nullptr)
				continue;

			// process items according to their native type
			switch (targetclass)
			{
				case ITEM_CLASS_ABSOLUTE:
					if (result == 0)
						result = item->read_as_absolute(code.item_modifier());
					break;

				case ITEM_CLASS_RELATIVE:
					result += item->read_as_relative(code.item_modifier());
					break;

				case ITEM_CLASS_SWITCH:
					result |= item->read_as_switch(code.item_modifier());
					break;

				default:
					break;
			}
		}
	} while (0);

	// stop the profiler before exiting
	g_profiler.stop();
	return result;
}


//-------------------------------------------------
//  code_pressed_once - return non-zero if a given
//  input code has transitioned from off to on
//  since the last call
//-------------------------------------------------

bool input_manager::code_pressed_once(input_code code)
{
	// look for the code in the memory
	bool curvalue = code_pressed(code);
	int empty = -1;
	for (int memnum = 0; memnum < ARRAY_LENGTH(m_switch_memory); memnum++)
	{
		// were we previous pressed on the last time through here?
		if (m_switch_memory[memnum] == code)
		{
			// if no longer pressed, clear entry
			if (curvalue == false)
				m_switch_memory[memnum] = INPUT_CODE_INVALID;

			// always return false
			return false;
		}

		// remember the first empty entry
		if (empty == -1 && m_switch_memory[memnum] == INPUT_CODE_INVALID)
			empty = memnum;
	}

	// if we get here, we were not previously pressed; if still not pressed, return 0
	if (curvalue == false)
		return false;

	// otherwise, add ourself to the memory and return 1
	assert(empty != -1);
	if (empty != -1)
		m_switch_memory[empty] = code;
	return true;
}


//-------------------------------------------------
//  reset_polling - reset memories in preparation
//  for polling
//-------------------------------------------------

void input_manager::reset_polling()
{
	// reset switch memory
	reset_memory();

	// iterate over device classes and devices
	for (input_device_class devclass = DEVICE_CLASS_FIRST_VALID; devclass <= DEVICE_CLASS_LAST_VALID; ++devclass)
		for (int devnum = 0; devnum <= m_class[devclass]->maxindex(); devnum++)
		{
			// fetch the device; ignore if nullptr
			input_device *device = m_class[devclass]->device(devnum);
			if (device == nullptr)
				continue;

			// iterate over items within each device
			for (input_item_id itemid = ITEM_ID_FIRST_VALID; itemid <= device->maxitem(); ++itemid)
			{
				// for any non-switch items, set memory equal to the current value
				input_device_item *item = device->item(itemid);
				if (item != nullptr && item->itemclass() != ITEM_CLASS_SWITCH)
					item->set_memory(code_value(item->code()));
			}
		}
}


//-------------------------------------------------
//  poll_switches - poll for any input
//-------------------------------------------------

input_code input_manager::poll_switches()
{
	// iterate over device classes and devices
	for (input_device_class devclass = DEVICE_CLASS_FIRST_VALID; devclass <= DEVICE_CLASS_LAST_VALID; ++devclass)
		for (int devnum = 0; devnum <= m_class[devclass]->maxindex(); devnum++)
		{
			// fetch the device; ignore if nullptr
			input_device *device = m_class[devclass]->device(devnum);
			if (device == nullptr)
				continue;

			// iterate over items within each device
			for (input_item_id itemid = ITEM_ID_FIRST_VALID; itemid <= device->maxitem(); ++itemid)
			{
				input_device_item *item = device->item(itemid);
				if (item != nullptr)
				{
					input_code code = item->code();

					// if the item is natively a switch, poll it
					if (item->itemclass() == ITEM_CLASS_SWITCH)
					{
						if (code_pressed_once(code))
							return code;
						else
							continue;
					}

					// skip if there is not enough axis movement
					if (!code_check_axis(*item, code))
						continue;

					// otherwise, poll axes digitally
					code.set_item_class(ITEM_CLASS_SWITCH);

					// if this is a joystick X axis, check with left/right modifiers
					if (devclass == DEVICE_CLASS_JOYSTICK && code.item_id() == ITEM_ID_XAXIS)
					{
						code.set_item_modifier(ITEM_MODIFIER_LEFT);
						if (code_pressed_once(code))
							return code;
						code.set_item_modifier(ITEM_MODIFIER_RIGHT);
						if (code_pressed_once(code))
							return code;
					}

					// if this is a joystick Y axis, check with up/down modifiers
					else if (devclass == DEVICE_CLASS_JOYSTICK && code.item_id() == ITEM_ID_YAXIS)
					{
						code.set_item_modifier(ITEM_MODIFIER_UP);
						if (code_pressed_once(code))
							return code;
						code.set_item_modifier(ITEM_MODIFIER_DOWN);
						if (code_pressed_once(code))
							return code;
					}

					// any other axis, check with pos/neg modifiers
					else
					{
						code.set_item_modifier(ITEM_MODIFIER_POS);
						if (code_pressed_once(code))
							return code;
						code.set_item_modifier(ITEM_MODIFIER_NEG);
						if (code_pressed_once(code))
							return code;
					}
				}
			}
		}

	// if nothing, return an invalid code
	return INPUT_CODE_INVALID;
}


//-------------------------------------------------
//  poll_keyboard_switches - poll for any
//  keyboard-specific input
//-------------------------------------------------

input_code input_manager::poll_keyboard_switches()
{
	// iterate over devices within each class
	input_class &keyboard_class = device_class(DEVICE_CLASS_KEYBOARD);
	for (int devnum = 0; devnum < keyboard_class.maxindex(); devnum++)
	{
		// fetch the device; ignore if nullptr
		input_device *device = keyboard_class.device(devnum);
		if (device == nullptr)
			continue;

		// iterate over items within each device
		for (input_item_id itemid = ITEM_ID_FIRST_VALID; itemid <= device->maxitem(); ++itemid)
		{
			input_device_item *item = device->item(itemid);
			if (item != nullptr && item->itemclass() == ITEM_CLASS_SWITCH)
			{
				input_code code = item->code();
				if (code_pressed_once(code))
					return code;
			}
		}
	}

	// if nothing, return an invalid code
	return INPUT_CODE_INVALID;
}


//-------------------------------------------------
//  code_check_axis - see if axis has moved far
//  enough to trigger a read when polling
//-------------------------------------------------

bool input_manager::code_check_axis(input_device_item &item, input_code code)
{
	// if we've already reported this one, don't bother
	if (item.memory() == INVALID_AXIS_VALUE)
		return false;

	// ignore min/max for lightguns
	// so the selection will not be affected by a gun going out of range
	s32 curval = code_value(code);
	if (code.device_class() == DEVICE_CLASS_LIGHTGUN &&
		(code.item_id() == ITEM_ID_XAXIS || code.item_id() == ITEM_ID_YAXIS) &&
		(curval == INPUT_ABSOLUTE_MAX || curval == INPUT_ABSOLUTE_MIN))
		return false;

	// compute the diff against memory
	s32 diff = curval - item.memory();
	if (diff < 0)
		diff = -diff;

	// for absolute axes, look for 25% of maximum
	if (item.itemclass() == ITEM_CLASS_ABSOLUTE && diff > (INPUT_ABSOLUTE_MAX - INPUT_ABSOLUTE_MIN) / 4)
	{
		item.set_memory(INVALID_AXIS_VALUE);
		return true;
	}

	// for relative axes, look for ~20 pixels movement
	if (item.itemclass() == ITEM_CLASS_RELATIVE && diff > 20 * INPUT_RELATIVE_PER_PIXEL)
	{
		item.set_memory(INVALID_AXIS_VALUE);
		return true;
	}
	return false;
}


//-------------------------------------------------
//  poll_axes - poll for any input
//-------------------------------------------------

input_code input_manager::poll_axes()
{
	// iterate over device classes and devices
	for (input_device_class devclass = DEVICE_CLASS_FIRST_VALID; devclass <= DEVICE_CLASS_LAST_VALID; ++devclass)
		for (int devnum = 0; devnum <= m_class[devclass]->maxindex(); devnum++)
		{
			// fetch the device; ignore if nullptr
			input_device *device = m_class[devclass]->device(devnum);
			if (device == nullptr)
				continue;

			// iterate over items within each device
			for (input_item_id itemid = ITEM_ID_FIRST_VALID; itemid <= device->maxitem(); ++itemid)
			{
				input_device_item *item = device->item(itemid);
				if (item != nullptr && item->itemclass() != ITEM_CLASS_SWITCH)
				{
					input_code code = item->code();
					if (code_check_axis(*item, code))
						return code;
				}
			}
		}

	// if nothing, return an invalid code
	return INPUT_CODE_INVALID;
}


//-------------------------------------------------
//  device_from_code - given an input_code return
//  a pointer to the associated device
//-------------------------------------------------

input_device *input_manager::device_from_code(input_code code) const
{
	// if the class is valid, return the appropriate device pointer
	input_device_class devclass = code.device_class();
	if (devclass >= DEVICE_CLASS_FIRST_VALID && devclass <= DEVICE_CLASS_LAST_VALID)
		return m_class[devclass]->device(code.device_index());

	// otherwise, return nullptr
	return nullptr;
}


//-------------------------------------------------
//  item_from_code - given an input_code return
//  a pointer to the appropriate input_device_item
//-------------------------------------------------

input_device_item *input_manager::item_from_code(input_code code) const
{
	// first get the device; if none, then we don't have an item
	input_device *device = device_from_code(code);
	if (device == nullptr)
		return nullptr;

	// then return the device's item
	return device->item(code.item_id());
}


//-------------------------------------------------
//  reset_memory - reset the array of memory for
//  pressed switches
//-------------------------------------------------

void input_manager::reset_memory()
{
	// reset all entries in switch memory to invalid
	for (auto & elem : m_switch_memory)
		elem = INPUT_CODE_INVALID;
}


//-------------------------------------------------
//  code_from_itemid - translates an input_item_id
//  to an input_code
//-------------------------------------------------

input_code input_manager::code_from_itemid(input_item_id itemid) const
{
	// iterate over device classes and devices
	for (input_device_class devclass = DEVICE_CLASS_FIRST_VALID; devclass <= DEVICE_CLASS_LAST_VALID; ++devclass)
		for (int devnum = 0; devnum <= m_class[devclass]->maxindex(); devnum++)
		{
			input_device *device = m_class[devclass]->device(devnum);
			if (device != nullptr)
			{
				input_device_item *item = device->item(itemid);
				if (item != nullptr)
					return item->code();
			}
		}

	return INPUT_CODE_INVALID;
}


//-------------------------------------------------
//  code_name - convert an input code into a
//  friendly name
//-------------------------------------------------

std::string input_manager::code_name(input_code code) const
{
	// if nothing there, return an empty string
	input_device_item *item = item_from_code(code);
	if (item == nullptr)
		return std::string();

	// determine the devclass part
	const char *devclass = (*devclass_string_table)[code.device_class()];

	// determine the devindex part
	std::string devindex = string_format("%d", code.device_index() + 1);

	// if we're unifying all devices, don't display a number
	if (!m_class[code.device_class()]->multi())
		devindex.clear();

	// keyboard 0 doesn't show a class or index if it is the only one
	input_device_class device_class = item->device().devclass();
	if (device_class == DEVICE_CLASS_KEYBOARD && m_class[device_class]->maxindex() == 0)
	{
		devclass = "";
		devindex.clear();
	}

	// devcode part comes from the item name
	const char *devcode = item->name();

	// determine the modifier part
	const char *modifier = (*modifier_string_table)[code.item_modifier()];

	// devcode is redundant with joystick switch left/right/up/down
	if (device_class == DEVICE_CLASS_JOYSTICK && code.item_class() == ITEM_CLASS_SWITCH)
		if (code.item_modifier() >= ITEM_MODIFIER_LEFT && code.item_modifier() <= ITEM_MODIFIER_DOWN)
			devcode = "";

	// concatenate the strings
	std::string str(devclass);
	if (!devindex.empty())
		str.append(" ").append(devindex);
	if (devcode[0] != 0)
		str.append(" ").append(devcode);
	if (modifier != nullptr)
		str.append(" ").append(modifier);

	// delete any leading spaces
	strtrimspace(str);
	return str;
}


//-------------------------------------------------
//  code_to_token - create a token for a given code
//-------------------------------------------------

std::string input_manager::code_to_token(input_code code) const
{
	// determine the devclass part
	const char *devclass = (*devclass_token_table)[code.device_class()];

	// determine the devindex part; keyboard 0 doesn't show an index
	std::string devindex = string_format("%d", code.device_index() + 1);
	if (code.device_class() == DEVICE_CLASS_KEYBOARD && code.device_index() == 0)
		devindex.clear();

	// determine the itemid part; look up in the table if we don't have a token
	input_device_item *item = item_from_code(code);
	const char *devcode = (item != nullptr) ? item->token() : "UNKNOWN";

	// determine the modifier part
	const char *modifier = (*modifier_token_table)[code.item_modifier()];

	// determine the itemclass part; if we match the native class, we don't include this
	const char *itemclass = "";
	if (item == nullptr || item->itemclass() != code.item_class())
		itemclass = (*itemclass_token_table)[code.item_class()];

	// concatenate the strings
	std::string str(devclass);
	if (!devindex.empty())
		str.append("_").append(devindex);
	if (devcode[0] != 0)
		str.append("_").append(devcode);
	if (modifier != nullptr)
		str.append("_").append(modifier);
	if (itemclass[0] != 0)
		str.append("_").append(itemclass);
	return str;
}


//-------------------------------------------------
//  code_from_token - extract an input code from a
//  token
//-------------------------------------------------

input_code input_manager::code_from_token(const char *_token)
{
	// copy the token and break it into pieces
	std::string token[6];
	int numtokens;
	for (numtokens = 0; numtokens < ARRAY_LENGTH(token); )
	{
		// make a token up to the next underscore
		char *score = (char *)strchr(_token, '_');
		token[numtokens++].assign(_token, (score == nullptr) ? strlen(_token) : (score - _token));

		// if we hit the end, we're done, else advance our pointer
		if (score == nullptr)
			break;
		_token = score + 1;
	}

	// first token should be the devclass
	int curtok = 0;
	input_device_class devclass = input_device_class((*devclass_token_table)[token[curtok++].c_str()]);
	if (devclass == ~input_device_class(0))
		return INPUT_CODE_INVALID;

	// second token might be index; look for number
	int devindex = 0;
	if (numtokens > 2 && sscanf(token[curtok].c_str(), "%d", &devindex) == 1)
	{
		curtok++;
		devindex--;
	}
	if (curtok >= numtokens)
		return INPUT_CODE_INVALID;

	// next token is the item ID
	input_item_id itemid = input_item_id((*itemid_token_table)[token[curtok].c_str()]);
	bool standard = (itemid != ~input_item_id(0));

	// if we're a standard code, default the itemclass based on it
	input_item_class itemclass = ITEM_CLASS_INVALID;
	if (standard)
		itemclass = m_class[devclass]->standard_item_class(itemid);

	// otherwise, keep parsing
	else
	{
		// if this is an invalid device, we have nothing to look up
		input_device *device = m_class[devclass]->device(devindex);
		if (device == nullptr)
			return INPUT_CODE_INVALID;

		// if not a standard code, look it up in the device specific codes
		for (itemid = ITEM_ID_FIRST_VALID; itemid <= device->maxitem(); ++itemid)
		{
			input_device_item *item = device->item(itemid);
			if (item != nullptr && token[curtok].compare(item->token()) == 0)
			{
				// take the itemclass from the item
				itemclass = item->itemclass();
				break;
			}
		}

		// bail on fail
		if (itemid > device->maxitem())
			return INPUT_CODE_INVALID;
	}
	curtok++;

	// if we have another token, it is probably a modifier
	input_item_modifier modifier = ITEM_MODIFIER_NONE;
	if (curtok < numtokens)
	{
		modifier = input_item_modifier((*modifier_token_table)[token[curtok].c_str()]);
		if (modifier != ~input_item_modifier(0))
			curtok++;
		else
			modifier = ITEM_MODIFIER_NONE;
	}

	// if we have another token, it is the item class
	if (curtok < numtokens)
	{
		u32 temp = (*itemclass_token_table)[token[curtok].c_str()];
		if (temp != ~0)
		{
			curtok++;
			itemclass = input_item_class(temp);
		}
	}

	// we should have consumed all tokens
	if (curtok != numtokens)
		return INPUT_CODE_INVALID;

	// assemble the final code
	return input_code(devclass, devindex, itemclass, modifier, itemid);
}


//-------------------------------------------------
//  standard_token - return the standard token for
//  the given input item ID
//-------------------------------------------------

const char *input_manager::standard_token(input_item_id itemid) const
{
	return itemid <= ITEM_ID_MAXIMUM ? (*itemid_token_table)[itemid] : nullptr;
}


//-------------------------------------------------
//  seq_pressed - return true if the given sequence
//  of switch inputs is "pressed"
//-------------------------------------------------

bool input_manager::seq_pressed(const input_seq &seq)
{
	// iterate over all of the codes
	bool result = false;
	bool invert = false;
	bool first = true;
	for (int codenum = 0; ; codenum++)
	{
		// handle NOT
		input_code code = seq[codenum];
		if (code == input_seq::not_code)
			invert = true;

		// handle OR and END
		else if (code == input_seq::or_code || code == input_seq::end_code)
		{
			// if we have a positive result from the previous set, we're done
			if (result || code == input_seq::end_code)
				break;

			// otherwise, reset our state
			result = false;
			invert = false;
			first = true;
		}

		// handle everything else as a series of ANDs
		else
		{
			// if this is the first in the sequence, result is set equal
			if (first)
				result = code_pressed(code) ^ invert;

			// further values are ANDed
			else if (result)
				result &= code_pressed(code) ^ invert;

			// no longer first, and clear the invert flag
			first = invert = false;
		}
	}

	// return the result if we queried at least one switch
	return result;
}


//-------------------------------------------------
//  seq_axis_value - return the value of an axis
//  defined in an input sequence
//-------------------------------------------------

s32 input_manager::seq_axis_value(const input_seq &seq, input_item_class &itemclass)
{
	// start with no valid classes
	input_item_class itemclasszero = ITEM_CLASS_INVALID;
	itemclass = ITEM_CLASS_INVALID;

	// iterate over all of the codes
	s32 result = 0;
	bool invert = false;
	bool enable = true;
	for (int codenum = 0; ; codenum++)
	{
		// handle NOT
		input_code code = seq[codenum];
		if (code == input_seq::not_code)
			invert = true;

		// handle OR and END
		else if (code == input_seq::or_code || code == input_seq::end_code)
		{
			// if we have a positive result from the previous set, we're done
			if (itemclass != ITEM_CLASS_INVALID || code == input_seq::end_code)
				break;

			// otherwise, reset our state
			result = 0;
			invert = false;
			enable = true;
		}

		// handle everything else only if we're still enabled
		else if (enable)
		{
			// switch codes serve as enables
			if (code.item_class() == ITEM_CLASS_SWITCH)
			{
				// AND against previous digital codes
				if (enable)
					enable &= code_pressed(code) ^ invert;
			}

			// non-switch codes are analog values
			else
			{
				s32 value = code_value(code);

				// if we got a 0 value, don't do anything except remember the first type
				if (value == 0)
				{
					if (itemclasszero == ITEM_CLASS_INVALID)
						itemclasszero = code.item_class();
				}

				// non-zero absolute values stick
				else if (code.item_class() == ITEM_CLASS_ABSOLUTE)
				{
					itemclass = ITEM_CLASS_ABSOLUTE;
					result = value;
				}

				// non-zero relative values accumulate
				else if (code.item_class() == ITEM_CLASS_RELATIVE)
				{
					itemclass = ITEM_CLASS_RELATIVE;
					result += value;
				}
			}

			// clear the invert flag
			invert = false;
		}
	}

	// if the caller wants to know the type, provide it
	if (result == 0)
		itemclass = itemclasszero;
	return result;
}


//-------------------------------------------------
//  seq_poll_start - begin polling for a new
//  sequence of the given itemclass
//-------------------------------------------------

void input_manager::seq_poll_start(input_item_class itemclass, const input_seq *startseq)
{
	assert(itemclass == ITEM_CLASS_SWITCH || itemclass == ITEM_CLASS_ABSOLUTE || itemclass == ITEM_CLASS_RELATIVE);

	// reset the recording count and the clock
	m_poll_seq_last_ticks = 0;
	m_poll_seq_class = itemclass;
	m_poll_seq.reset();

	// grab the starting sequence to append to, and append an OR
	if (startseq != nullptr)
	{
		m_poll_seq = *startseq;
		if (m_poll_seq.length() > 0)
			m_poll_seq += input_seq::or_code;
	}

	// flush out any goobers
	reset_polling();
	input_code dummycode = KEYCODE_ENTER;
	while (dummycode != INPUT_CODE_INVALID)
		dummycode = (m_poll_seq_class == ITEM_CLASS_SWITCH) ? poll_switches() : poll_axes();
}


//-------------------------------------------------
//  input_seq_poll - continue polling
//-------------------------------------------------

bool input_manager::seq_poll()
{
	int curlen = m_poll_seq.length();
	input_code lastcode = m_poll_seq[curlen - 1];

	// switch case: see if we have a new code to process
	input_code newcode;
	if (m_poll_seq_class == ITEM_CLASS_SWITCH)
	{
		newcode = poll_switches();
		if (newcode != INPUT_CODE_INVALID)
		{
			// if code is duplicate, toggle the NOT state on the code
			if (curlen > 0 && newcode == lastcode)
			{
				// back up over the existing code
				m_poll_seq.backspace();

				// if there was a NOT preceding it, delete it as well, otherwise append a fresh one
				if (m_poll_seq[curlen - 2] == input_seq::not_code)
					m_poll_seq.backspace();
				else
					m_poll_seq += input_seq::not_code;
			}
		}
	}

	// absolute/relative case: see if we have an analog change of sufficient amount
	else
	{
		bool has_or = false;
		if (lastcode == input_seq::or_code)
		{
			lastcode = m_poll_seq[curlen - 2];
			has_or = true;
		}
		newcode = poll_axes();

		// if the last code doesn't match absolute/relative of this code, ignore the new one
		if ((lastcode.item_class() == ITEM_CLASS_ABSOLUTE && newcode.item_class() != ITEM_CLASS_ABSOLUTE) ||
			(lastcode.item_class() == ITEM_CLASS_RELATIVE && newcode.item_class() != ITEM_CLASS_RELATIVE))
			newcode = INPUT_CODE_INVALID;

		// if the new code is valid, check for half-axis toggles on absolute controls
		if (newcode != INPUT_CODE_INVALID && curlen > 0 && newcode.item_class() == ITEM_CLASS_ABSOLUTE)
		{
			input_code last_nomodifier = lastcode;
			last_nomodifier.set_item_modifier(ITEM_MODIFIER_NONE);
			if (newcode == last_nomodifier)
			{
				// increment the modifier, wrapping back to none
				switch (lastcode.item_modifier())
				{
					case ITEM_MODIFIER_NONE:    newcode.set_item_modifier(ITEM_MODIFIER_POS);   break;
					case ITEM_MODIFIER_POS:     newcode.set_item_modifier(ITEM_MODIFIER_NEG);   break;
					default:
					case ITEM_MODIFIER_NEG:     newcode.set_item_modifier(ITEM_MODIFIER_NONE);  break;
				}

				// back up over the previous code so we can re-append
				if (has_or)
					m_poll_seq.backspace();
				m_poll_seq.backspace();
			}
		}
	}

	// if we got a new code to append it, append it and reset the timer
	if (newcode != INPUT_CODE_INVALID)
	{
		m_poll_seq += newcode;
		m_poll_seq_last_ticks = osd_ticks();
	}

	// if we're recorded at least one item and 2/3 of a second has passed, we're done
	if (m_poll_seq_last_ticks != 0 && osd_ticks() > m_poll_seq_last_ticks + osd_ticks_per_second() * 2 / 3)
	{
		// if the final result is invalid, reset to nothing
		if (!m_poll_seq.is_valid())
			m_poll_seq.reset();

		// return true to indicate that we are finished
		return true;
	}

	// return false to indicate we are still polling
	return false;
}


//-------------------------------------------------
//  seq_name - generate the friendly name of a
//  sequence
//-------------------------------------------------

std::string input_manager::seq_name(const input_seq &seq) const
{
	// make a copy of our sequence, removing any invalid bits
	input_code clean_codes[sizeof(seq) / sizeof(input_code)];
	int clean_index = 0;
	for (int codenum = 0; seq[codenum] != input_seq::end_code; codenum++)
	{
		// if this is a code item which is not valid, don't copy it and remove any preceding ORs/NOTs
		input_code code = seq[codenum];
		if (!code.internal() && code_name(code).empty())
		{
			while (clean_index > 0 && clean_codes[clean_index - 1].internal())
				clean_index--;
		}
		else if (clean_index > 0 || !code.internal())
			clean_codes[clean_index++] = code;
	}

	// special case: empty
	if (clean_index == 0)
		return std::string((seq.length() == 0) ? "None" : "n/a");

	// start with an empty buffer
	std::string str;

	// loop until we hit the end
	for (int codenum = 0; codenum < clean_index; codenum++)
	{
		// append a space if not the first code
		if (codenum != 0)
			str.append(" ");

		// handle OR/NOT codes here
		input_code code = clean_codes[codenum];
		if (code == input_seq::or_code)
			str.append("or");
		else if (code == input_seq::not_code)
			str.append("not");

		// otherwise, assume it is an input code and ask the input system to generate it
		else
			str.append(code_name(code));
	}
	return str;
}


//-------------------------------------------------
//  seq_to_tokens - generate the tokenized form of
//  a sequence
//-------------------------------------------------

std::string input_manager::seq_to_tokens(const input_seq &seq) const
{
	// start with an empty buffer
	std::string str;

	// loop until we hit the end
	for (int codenum = 0; seq[codenum] != input_seq::end_code; codenum++)
	{
		// append a space if not the first code
		if (codenum != 0)
			str.append(" ");

		// handle OR/NOT codes here
		input_code code = seq[codenum];
		if (code == input_seq::or_code)
			str.append("OR");
		else if (code == input_seq::not_code)
			str.append("NOT");
		else if (code == input_seq::default_code)
			str.append("DEFAULT");

		// otherwise, assume it is an input code and ask the input system to generate it
		else
			str.append(code_to_token(code));
	}
	return str;
}


//-------------------------------------------------
//  seq_from_tokens - generate the tokenized form
//  of a sequence
//-------------------------------------------------

void input_manager::seq_from_tokens(input_seq &seq, const char *string)
{
	// start with a blank sequence
	seq.reset();

	// loop until we're done
	std::vector<char> strcopy(string, string + std::strlen(string) + 1);
	char *str = &strcopy[0];
	unsigned operators = 0;
	while (1)
	{
		// trim any leading spaces
		while (*str != 0 && isspace(u8(*str)))
			str++;

		// bail if we're done
		if (*str == 0)
			return;

		// find the end of the token and make it upper-case along the way
		char *strtemp;
		for (strtemp = str; *strtemp != 0 && !isspace(u8(*strtemp)); strtemp++)
			*strtemp = toupper(u8(*strtemp));
		char origspace = *strtemp;
		*strtemp = 0;

		// look for common stuff
		input_code code;
		bool is_operator;
		if (strcmp(str, "OR") == 0)
		{
			code = input_seq::or_code;
			is_operator = true;
		}
		else if (strcmp(str, "NOT") == 0)
		{
			code = input_seq::not_code;
			is_operator = true;
		}
		else if (strcmp(str, "DEFAULT") == 0)
		{
			code = input_seq::default_code;
			is_operator = false;
		}
		else
		{
			code = code_from_token(str);
			is_operator = false;
		}

		// translate and add to the sequence
		if (is_operator)
		{
			++operators;
			seq += code;
		}
		else
		{
			if (code.device_class() < DEVICE_CLASS_FIRST_VALID)
			{
				osd_printf_warning("Input: Dropping invalid input token %s\n", str);
				while (operators)
				{
					seq.backspace();
					--operators;
				}
			}
			else
			{
				operators = 0;
				seq += code;
			}
		}

		// advance
		if (origspace == 0)
			return;
		str = strtemp + 1;
	}
}

//-------------------------------------------------
//  map_device_to_controller - map device to
//  controller based on device map table
//-------------------------------------------------

bool input_manager::map_device_to_controller(const devicemap_table_type *devicemap_table)
{
	if (nullptr == devicemap_table)
		return true;

	for (devicemap_table_type::const_iterator it = devicemap_table->begin(); it != devicemap_table->end(); it++)
	{
		const char *deviceid = it->first.c_str();
		const char *controllername = it->second.c_str();

		// tokenize the controller name into device class and index (i.e. controller name should be of the form "GUNCODE_1")
		std::string token[2];
		int numtokens;
		const char *_token = controllername;
		for (numtokens = 0; numtokens < ARRAY_LENGTH(token); )
		{
			// make a token up to the next underscore
			char *score = (char *)strchr(_token, '_');
			token[numtokens++].assign(_token, (score == nullptr) ? strlen(_token) : (score - _token));

			// if we hit the end, we're done, else advance our pointer
			if (score == nullptr)
				break;
			_token = score + 1;
		}
		if (2 != numtokens)
			return false;

		// first token should be the devclass
		input_device_class devclass = input_device_class((*devclass_token_table)[strmakeupper(token[0]).c_str()]);
		if (devclass == ~input_device_class(0))
			return false;

		// second token should be the devindex
		int devindex = 0;
		if (1 != sscanf(token[1].c_str(), "%d", &devindex))
			return false;
		devindex--;

		if (devindex >= DEVICE_INDEX_MAXIMUM)
			return false;

		// enumerate through devices and look for a match
		input_class *input_devclass = m_class[devclass].get();
		for (int devnum = 0; devnum <= input_devclass->maxindex(); devnum++)
		{
			input_device *device = input_devclass->device(devnum);
			if (device != nullptr && device->match_device_id(deviceid))
			{
				// remap devindex
				input_devclass->remap_device_index(device->devindex(), devindex);
				osd_printf_verbose("Input: Remapped %s #%d: %s (device id: %s)\n", input_devclass->name(), devindex, device->name(), device->id());

				break;
			}
		}
	}

	return true;
}
