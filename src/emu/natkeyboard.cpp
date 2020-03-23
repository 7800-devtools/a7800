// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    natkeyboard.cpp

    Natural keyboard input support.

***************************************************************************/

#include "emu.h"
#include "natkeyboard.h"
#include "emuopts.h"


//**************************************************************************
//  DEBUGGING
//**************************************************************************

#define LOG_NATURAL_KEYBOARD    0



//**************************************************************************
//  CONSTANTS
//**************************************************************************

const int KEY_BUFFER_SIZE = 4096;
const char32_t INVALID_CHAR = '?';



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// character information
struct char_info
{
	char32_t ch;
	const char *alternate;  // alternative string, in UTF-8

	static const char_info *find(char32_t target);
};


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************


// master character info table
const char_info charinfo[] =
{
	{ 0x0009,                   "    " },   // Tab
	{ 0x0061,                   "A" },      // a
	{ 0x0062,                   "B" },      // b
	{ 0x0063,                   "C" },      // c
	{ 0x0064,                   "D" },      // d
	{ 0x0065,                   "E" },      // e
	{ 0x0066,                   "F" },      // f
	{ 0x0067,                   "G" },      // g
	{ 0x0068,                   "H" },      // h
	{ 0x0069,                   "I" },      // i
	{ 0x006a,                   "J" },      // j
	{ 0x006b,                   "K" },      // k
	{ 0x006c,                   "L" },      // l
	{ 0x006d,                   "M" },      // m
	{ 0x006e,                   "N" },      // n
	{ 0x006f,                   "O" },      // o
	{ 0x0070,                   "P" },      // p
	{ 0x0071,                   "Q" },      // q
	{ 0x0072,                   "R" },      // r
	{ 0x0073,                   "S" },      // s
	{ 0x0074,                   "T" },      // t
	{ 0x0075,                   "U" },      // u
	{ 0x0076,                   "V" },      // v
	{ 0x0077,                   "W" },      // w
	{ 0x0078,                   "X" },      // x
	{ 0x0079,                   "Y" },      // y
	{ 0x007a,                   "Z" },      // z
	{ 0x00a0,                   " " },      // non breaking space
	{ 0x00a1,                   "!" },      // inverted exclamation mark
	{ 0x00a6,                   "|" },      // broken bar
	{ 0x00a9,                   "(c)" },    // copyright sign
	{ 0x00ab,                   "<<" },     // left pointing double angle
	{ 0x00ae,                   "(r)" },    // registered sign
	{ 0x00bb,                   ">>" },     // right pointing double angle
	{ 0x00bc,                   "1/4" },    // vulgar fraction one quarter
	{ 0x00bd,                   "1/2" },    // vulgar fraction one half
	{ 0x00be,                   "3/4" },    // vulgar fraction three quarters
	{ 0x00bf,                   "?" },      // inverted question mark
	{ 0x00c0,                   "A" },      // 'A' grave
	{ 0x00c1,                   "A" },      // 'A' acute
	{ 0x00c2,                   "A" },      // 'A' circumflex
	{ 0x00c3,                   "A" },      // 'A' tilde
	{ 0x00c4,                   "A" },      // 'A' diaeresis
	{ 0x00c5,                   "A" },      // 'A' ring above
	{ 0x00c6,                   "AE" },     // 'AE' ligature
	{ 0x00c7,                   "C" },      // 'C' cedilla
	{ 0x00c8,                   "E" },      // 'E' grave
	{ 0x00c9,                   "E" },      // 'E' acute
	{ 0x00ca,                   "E" },      // 'E' circumflex
	{ 0x00cb,                   "E" },      // 'E' diaeresis
	{ 0x00cc,                   "I" },      // 'I' grave
	{ 0x00cd,                   "I" },      // 'I' acute
	{ 0x00ce,                   "I" },      // 'I' circumflex
	{ 0x00cf,                   "I" },      // 'I' diaeresis
	{ 0x00d0,                   "D" },      // 'ETH'
	{ 0x00d1,                   "N" },      // 'N' tilde
	{ 0x00d2,                   "O" },      // 'O' grave
	{ 0x00d3,                   "O" },      // 'O' acute
	{ 0x00d4,                   "O" },      // 'O' circumflex
	{ 0x00d5,                   "O" },      // 'O' tilde
	{ 0x00d6,                   "O" },      // 'O' diaeresis
	{ 0x00d7,                   "X" },      // multiplication sign
	{ 0x00d8,                   "O" },      // 'O' stroke
	{ 0x00d9,                   "U" },      // 'U' grave
	{ 0x00da,                   "U" },      // 'U' acute
	{ 0x00db,                   "U" },      // 'U' circumflex
	{ 0x00dc,                   "U" },      // 'U' diaeresis
	{ 0x00dd,                   "Y" },      // 'Y' acute
	{ 0x00df,                   "SS" },     // sharp S
	{ 0x00e0,                   "a" },      // 'a' grave
	{ 0x00e1,                   "a" },      // 'a' acute
	{ 0x00e2,                   "a" },      // 'a' circumflex
	{ 0x00e3,                   "a" },      // 'a' tilde
	{ 0x00e4,                   "a" },      // 'a' diaeresis
	{ 0x00e5,                   "a" },      // 'a' ring above
	{ 0x00e6,                   "ae" },     // 'ae' ligature
	{ 0x00e7,                   "c" },      // 'c' cedilla
	{ 0x00e8,                   "e" },      // 'e' grave
	{ 0x00e9,                   "e" },      // 'e' acute
	{ 0x00ea,                   "e" },      // 'e' circumflex
	{ 0x00eb,                   "e" },      // 'e' diaeresis
	{ 0x00ec,                   "i" },      // 'i' grave
	{ 0x00ed,                   "i" },      // 'i' acute
	{ 0x00ee,                   "i" },      // 'i' circumflex
	{ 0x00ef,                   "i" },      // 'i' diaeresis
	{ 0x00f0,                   "d" },      // 'eth'
	{ 0x00f1,                   "n" },      // 'n' tilde
	{ 0x00f2,                   "o" },      // 'o' grave
	{ 0x00f3,                   "o" },      // 'o' acute
	{ 0x00f4,                   "o" },      // 'o' circumflex
	{ 0x00f5,                   "o" },      // 'o' tilde
	{ 0x00f6,                   "o" },      // 'o' diaeresis
	{ 0x00f8,                   "o" },      // 'o' stroke
	{ 0x00f9,                   "u" },      // 'u' grave
	{ 0x00fa,                   "u" },      // 'u' acute
	{ 0x00fb,                   "u" },      // 'u' circumflex
	{ 0x00fc,                   "u" },      // 'u' diaeresis
	{ 0x00fd,                   "y" },      // 'y' acute
	{ 0x00ff,                   "y" },      // 'y' diaeresis
	{ 0x2010,                   "-" },      // hyphen
	{ 0x2011,                   "-" },      // non-breaking hyphen
	{ 0x2012,                   "-" },      // figure dash
	{ 0x2013,                   "-" },      // en dash
	{ 0x2014,                   "-" },      // em dash
	{ 0x2015,                   "-" },      // horizontal dash
	{ 0x2018,                   "\'" },     // left single quotation mark
	{ 0x2019,                   "\'" },     // right single quotation mark
	{ 0x201a,                   "\'" },     // single low quotation mark
	{ 0x201b,                   "\'" },     // single high reversed quotation mark
	{ 0x201c,                   "\"" },     // left double quotation mark
	{ 0x201d,                   "\"" },     // right double quotation mark
	{ 0x201e,                   "\"" },     // double low quotation mark
	{ 0x201f,                   "\"" },     // double high reversed quotation mark
	{ 0x2024,                   "." },      // one dot leader
	{ 0x2025,                   ".." },     // two dot leader
	{ 0x2026,                   "..." },    // horizontal ellipsis
	{ 0x2047,                   "??" },     // double question mark
	{ 0x2048,                   "?!" },     // question exclamation mark
	{ 0x2049,                   "!?" },     // exclamation question mark
	{ 0xff01,                   "!" },      // fullwidth exclamation point
	{ 0xff02,                   "\"" },     // fullwidth quotation mark
	{ 0xff03,                   "#" },      // fullwidth number sign
	{ 0xff04,                   "$" },      // fullwidth dollar sign
	{ 0xff05,                   "%" },      // fullwidth percent sign
	{ 0xff06,                   "&" },      // fullwidth ampersand
	{ 0xff07,                   "\'" },     // fullwidth apostrophe
	{ 0xff08,                   "(" },      // fullwidth left parenthesis
	{ 0xff09,                   ")" },      // fullwidth right parenthesis
	{ 0xff0a,                   "*" },      // fullwidth asterisk
	{ 0xff0b,                   "+" },      // fullwidth plus
	{ 0xff0c,                   "," },      // fullwidth comma
	{ 0xff0d,                   "-" },      // fullwidth minus
	{ 0xff0e,                   "." },      // fullwidth period
	{ 0xff0f,                   "/" },      // fullwidth slash
	{ 0xff10,                   "0" },      // fullwidth zero
	{ 0xff11,                   "1" },      // fullwidth one
	{ 0xff12,                   "2" },      // fullwidth two
	{ 0xff13,                   "3" },      // fullwidth three
	{ 0xff14,                   "4" },      // fullwidth four
	{ 0xff15,                   "5" },      // fullwidth five
	{ 0xff16,                   "6" },      // fullwidth six
	{ 0xff17,                   "7" },      // fullwidth seven
	{ 0xff18,                   "8" },      // fullwidth eight
	{ 0xff19,                   "9" },      // fullwidth nine
	{ 0xff1a,                   ":" },      // fullwidth colon
	{ 0xff1b,                   ";" },      // fullwidth semicolon
	{ 0xff1c,                   "<" },      // fullwidth less than sign
	{ 0xff1d,                   "=" },      // fullwidth equals sign
	{ 0xff1e,                   ">" },      // fullwidth greater than sign
	{ 0xff1f,                   "?" },      // fullwidth question mark
	{ 0xff20,                   "@" },      // fullwidth at sign
	{ 0xff21,                   "A" },      // fullwidth 'A'
	{ 0xff22,                   "B" },      // fullwidth 'B'
	{ 0xff23,                   "C" },      // fullwidth 'C'
	{ 0xff24,                   "D" },      // fullwidth 'D'
	{ 0xff25,                   "E" },      // fullwidth 'E'
	{ 0xff26,                   "F" },      // fullwidth 'F'
	{ 0xff27,                   "G" },      // fullwidth 'G'
	{ 0xff28,                   "H" },      // fullwidth 'H'
	{ 0xff29,                   "I" },      // fullwidth 'I'
	{ 0xff2a,                   "J" },      // fullwidth 'J'
	{ 0xff2b,                   "K" },      // fullwidth 'K'
	{ 0xff2c,                   "L" },      // fullwidth 'L'
	{ 0xff2d,                   "M" },      // fullwidth 'M'
	{ 0xff2e,                   "N" },      // fullwidth 'N'
	{ 0xff2f,                   "O" },      // fullwidth 'O'
	{ 0xff30,                   "P" },      // fullwidth 'P'
	{ 0xff31,                   "Q" },      // fullwidth 'Q'
	{ 0xff32,                   "R" },      // fullwidth 'R'
	{ 0xff33,                   "S" },      // fullwidth 'S'
	{ 0xff34,                   "T" },      // fullwidth 'T'
	{ 0xff35,                   "U" },      // fullwidth 'U'
	{ 0xff36,                   "V" },      // fullwidth 'V'
	{ 0xff37,                   "W" },      // fullwidth 'W'
	{ 0xff38,                   "X" },      // fullwidth 'X'
	{ 0xff39,                   "Y" },      // fullwidth 'Y'
	{ 0xff3a,                   "Z" },      // fullwidth 'Z'
	{ 0xff3b,                   "[" },      // fullwidth left bracket
	{ 0xff3c,                   "\\" },     // fullwidth backslash
	{ 0xff3d,                   "]" },      // fullwidth right bracket
	{ 0xff3e,                   "^" },      // fullwidth caret
	{ 0xff3f,                   "_" },      // fullwidth underscore
	{ 0xff40,                   "`" },      // fullwidth backquote
	{ 0xff41,                   "a" },      // fullwidth 'a'
	{ 0xff42,                   "b" },      // fullwidth 'b'
	{ 0xff43,                   "c" },      // fullwidth 'c'
	{ 0xff44,                   "d" },      // fullwidth 'd'
	{ 0xff45,                   "e" },      // fullwidth 'e'
	{ 0xff46,                   "f" },      // fullwidth 'f'
	{ 0xff47,                   "g" },      // fullwidth 'g'
	{ 0xff48,                   "h" },      // fullwidth 'h'
	{ 0xff49,                   "i" },      // fullwidth 'i'
	{ 0xff4a,                   "j" },      // fullwidth 'j'
	{ 0xff4b,                   "k" },      // fullwidth 'k'
	{ 0xff4c,                   "l" },      // fullwidth 'l'
	{ 0xff4d,                   "m" },      // fullwidth 'm'
	{ 0xff4e,                   "n" },      // fullwidth 'n'
	{ 0xff4f,                   "o" },      // fullwidth 'o'
	{ 0xff50,                   "p" },      // fullwidth 'p'
	{ 0xff51,                   "q" },      // fullwidth 'q'
	{ 0xff52,                   "r" },      // fullwidth 'r'
	{ 0xff53,                   "s" },      // fullwidth 's'
	{ 0xff54,                   "t" },      // fullwidth 't'
	{ 0xff55,                   "u" },      // fullwidth 'u'
	{ 0xff56,                   "v" },      // fullwidth 'v'
	{ 0xff57,                   "w" },      // fullwidth 'w'
	{ 0xff58,                   "x" },      // fullwidth 'x'
	{ 0xff59,                   "y" },      // fullwidth 'y'
	{ 0xff5a,                   "z" },      // fullwidth 'z'
	{ 0xff5b,                   "{" },      // fullwidth left brace
	{ 0xff5c,                   "|" },      // fullwidth vertical bar
	{ 0xff5d,                   "}" },      // fullwidth right brace
	{ 0xff5e,                   "~" },      // fullwidth tilde
	{ 0xff5f,                   "((" },     // fullwidth double left parenthesis
	{ 0xff60,                   "))" },     // fullwidth double right parenthesis
	{ 0xffe0,                   "\xC2\xA2" },       // fullwidth cent sign
	{ 0xffe1,                   "\xC2\xA3" },       // fullwidth pound sign
	{ 0xffe4,                   "\xC2\xA4" },       // fullwidth broken bar
	{ 0xffe5,                   "\xC2\xA5" },       // fullwidth yen sign
	{ 0xffe6,                   "\xE2\x82\xA9" },   // fullwidth won sign
	{ 0xffe9,                   "\xE2\x86\x90" },   // fullwidth left arrow
	{ 0xffea,                   "\xE2\x86\x91" },   // fullwidth up arrow
	{ 0xffeb,                   "\xE2\x86\x92" },   // fullwidth right arrow
	{ 0xffec,                   "\xE2\x86\x93" },   // fullwidth down arrow
	{ 0xffed,                   "\xE2\x96\xAA" },   // fullwidth solid box
	{ 0xffee,                   "\xE2\x97\xA6" },   // fullwidth open circle
	{ UCHAR_MAMEKEY(ESC),       "\033" },   // Esc key
	{ UCHAR_MAMEKEY(DEL),       "\010" },   // Delete key
	{ UCHAR_MAMEKEY(HOME),      "\014" },   // Home key
	{ UCHAR_MAMEKEY(0_PAD),     "0" },      // 0 on the numeric keypad
	{ UCHAR_MAMEKEY(1_PAD),     "1" },      // 1 on the numeric keypad
	{ UCHAR_MAMEKEY(2_PAD),     "2" },      // 2 on the numeric keypad
	{ UCHAR_MAMEKEY(3_PAD),     "3" },      // 3 on the numeric keypad
	{ UCHAR_MAMEKEY(4_PAD),     "4" },      // 4 on the numeric keypad
	{ UCHAR_MAMEKEY(5_PAD),     "5" },      // 5 on the numeric keypad
	{ UCHAR_MAMEKEY(6_PAD),     "6" },      // 6 on the numeric keypad
	{ UCHAR_MAMEKEY(7_PAD),     "7" },      // 7 on the numeric keypad
	{ UCHAR_MAMEKEY(8_PAD),     "8" },      // 8 on the numeric keypad
	{ UCHAR_MAMEKEY(9_PAD),     "9" },      // 9 on the numeric keypad
	{ UCHAR_MAMEKEY(SLASH_PAD), "/" },      // / on the numeric keypad
	{ UCHAR_MAMEKEY(ASTERISK),  "*" },      // * on the numeric keypad
	{ UCHAR_MAMEKEY(MINUS_PAD), "-" },      // - on the numeric Keypad
	{ UCHAR_MAMEKEY(PLUS_PAD),  "+" },      // + on the numeric Keypad
	{ UCHAR_MAMEKEY(DEL_PAD),   "." },      // . on the numeric keypad
	{ UCHAR_MAMEKEY(ENTER_PAD), "\015" },   // Enter on the numeric keypad
	{ UCHAR_MAMEKEY(BS_PAD),    "\010" },   // Backspace on the numeric keypad
	{ UCHAR_MAMEKEY(TAB_PAD),   "\011" },   // Tab on the numeric keypad
	{ UCHAR_MAMEKEY(00_PAD),    "00" },     // 00 on the numeric keypad
	{ UCHAR_MAMEKEY(000_PAD),   "000" }     // 000 on the numeric keypad
};



//**************************************************************************
//  NATURAL KEYBOARD
//**************************************************************************

//-------------------------------------------------
//  natural_keyboard - constructor
//-------------------------------------------------

natural_keyboard::natural_keyboard(running_machine &machine)
	: m_machine(machine)
	, m_in_use(false)
	, m_bufbegin(0)
	, m_bufend(0)
	, m_fieldnum(0)
	, m_status_keydown(false)
	, m_last_cr(false)
	, m_timer(nullptr)
	, m_current_rate(attotime::zero)
	, m_queue_chars()
	, m_accept_char()
	, m_charqueue_empty()
{
	// try building a list of keycodes; if none are available, don't bother
	build_codes(machine.ioport());
	if (!m_keycode_map.empty())
	{
		m_buffer.resize(KEY_BUFFER_SIZE);
		m_timer = machine.scheduler().timer_alloc(timer_expired_delegate(FUNC(natural_keyboard::timer), this));
	}

	// retrieve option setting
	set_in_use(machine.options().natural_keyboard());
}


//-------------------------------------------------
//  configure - configure callbacks for full-
//  featured keyboard support
//-------------------------------------------------

void natural_keyboard::configure(ioport_queue_chars_delegate queue_chars, ioport_accept_char_delegate accept_char, ioport_charqueue_empty_delegate charqueue_empty)
{
	// set the callbacks
	m_queue_chars = queue_chars;
	m_accept_char = accept_char;
	m_charqueue_empty = charqueue_empty;
}


//-------------------------------------------------
//  set_in_use - specify whether the natural
//  keyboard is active
//-------------------------------------------------

void natural_keyboard::set_in_use(bool usage)
{
	if (m_in_use != usage)
	{
		// update active usage
		m_in_use = usage;
		machine().options().set_value(OPTION_NATURAL_KEYBOARD, usage, OPTION_PRIORITY_CMDLINE);

		// lock out (or unlock) all keyboard inputs
		for (auto &port : machine().ioport().ports())
			for (ioport_field &field : port.second->fields())
				if (field.type() == IPT_KEYBOARD)
				{
					field.live().lockout = usage;

					// clear pressed status when going out of use
					if (!usage)
						field.set_value(0);
				}
	}
}


//-------------------------------------------------
//  post - post a single character
//-------------------------------------------------

void natural_keyboard::post(char32_t ch)
{
	// ignore any \n that are preceded by \r
	if (m_last_cr && ch == '\n')
	{
		m_last_cr = false;
		return;
	}

	// change all eolns to '\r'
	if (ch == '\n')
		ch = '\r';
	else
		m_last_cr = (ch == '\r');

	// logging
	if (LOG_NATURAL_KEYBOARD)
	{
		const keycode_map_entry *code = find_code(ch);
		machine().logerror("natural_keyboard::post(): code=%i (%s) field.name='%s'\n", int(ch), unicode_to_string(ch).c_str(), (code != nullptr && code->field[0] != nullptr) ? code->field[0]->name() : "<null>");
	}

	// can we post this key in the queue directly?
	if (can_post_directly(ch))
		internal_post(ch);

	// can we post this key with an alternate representation?
	else if (can_post_alternate(ch))
	{
		const char_info *info = char_info::find(ch);
		assert(info != nullptr && info->alternate != nullptr);
		const char *altstring = info->alternate;
		while (*altstring != 0)
		{
			altstring += uchar_from_utf8(&ch, altstring, strlen(altstring));
			internal_post(ch);
		}
	}
}


//-------------------------------------------------
//  post - post a unicode encoded string
//-------------------------------------------------

void natural_keyboard::post(const char32_t *text, size_t length, const attotime &rate)
{
	// set the fixed rate
	m_current_rate = rate;

	// 0 length means strlen
	if (length == 0)
		for (const char32_t *scan = text; *scan != 0; scan++)
			length++;

	// iterate over characters or until the buffer is full up
	while (length > 0 && !full())
	{
		// fetch next character
		post(*text++);
		length--;
	}
}


//-------------------------------------------------
//  post_utf8 - post a UTF-8 encoded string
//-------------------------------------------------

void natural_keyboard::post_utf8(const char *text, size_t length, const attotime &rate)
{
	// set the fixed rate
	m_current_rate = rate;

	// 0-length means strlen
	if (length == 0)
		length = strlen(text);

	// iterate until out of characters
	while (length > 0)
	{
		// decode the next character
		char32_t uc;
		int count = uchar_from_utf8(&uc, text, length);
		if (count < 0)
		{
			count = 1;
			uc = INVALID_CHAR;
		}

		// append to the buffer
		post(uc);
		text += count;
		length -= count;
	}
}


//-------------------------------------------------
//  post_coded - post a coded string
//-------------------------------------------------

void natural_keyboard::post_coded(const char *text, size_t length, const attotime &rate)
{
	static const struct
	{
		const char *key;
		char32_t code;
	} codes[] =
	{
		{ "BACKSPACE",  8 },
		{ "BS",         8 },
		{ "BKSP",       8 },
		{ "DEL",        UCHAR_MAMEKEY(DEL) },
		{ "DELETE",     UCHAR_MAMEKEY(DEL) },
		{ "END",        UCHAR_MAMEKEY(END) },
		{ "ENTER",      13 },
		{ "ESC",        '\033' },
		{ "HOME",       UCHAR_MAMEKEY(HOME) },
		{ "INS",        UCHAR_MAMEKEY(INSERT) },
		{ "INSERT",     UCHAR_MAMEKEY(INSERT) },
		{ "PGDN",       UCHAR_MAMEKEY(PGDN) },
		{ "PGUP",       UCHAR_MAMEKEY(PGUP) },
		{ "SPACE",      32 },
		{ "TAB",        9 },
		{ "F1",         UCHAR_MAMEKEY(F1) },
		{ "F2",         UCHAR_MAMEKEY(F2) },
		{ "F3",         UCHAR_MAMEKEY(F3) },
		{ "F4",         UCHAR_MAMEKEY(F4) },
		{ "F5",         UCHAR_MAMEKEY(F5) },
		{ "F6",         UCHAR_MAMEKEY(F6) },
		{ "F7",         UCHAR_MAMEKEY(F7) },
		{ "F8",         UCHAR_MAMEKEY(F8) },
		{ "F9",         UCHAR_MAMEKEY(F9) },
		{ "F10",        UCHAR_MAMEKEY(F10) },
		{ "F11",        UCHAR_MAMEKEY(F11) },
		{ "F12",        UCHAR_MAMEKEY(F12) },
		{ "QUOTE",      '\"' }
	};

	// set the fixed rate
	m_current_rate = rate;

	// 0-length means strlen
	if (length == 0)
		length = strlen(text);

	// iterate through the source string
	size_t curpos = 0;
	while (curpos < length)
	{
		// extract next character
		char32_t ch = text[curpos];
		size_t increment = 1;

		// look for escape characters
		if (ch == '{')
			for (auto & code : codes)
			{
				size_t keylen = strlen(code.key);
				if (curpos + keylen + 2 <= length)
					if (core_strnicmp(code.key, &text[curpos + 1], keylen) == 0 && text[curpos + keylen + 1] == '}')
					{
						ch = code.code;
						increment = keylen + 2;
					}
			}

		// if we got a code, post it
		if (ch != 0)
			post(ch);
		curpos += increment;
	}
}


//-------------------------------------------------
//  build_codes - given an input port table, create
//  an input code table useful for mapping unicode
//  chars
//-------------------------------------------------

void natural_keyboard::build_codes(ioport_manager &manager)
{
	// find all shfit keys
	unsigned mask = 0;
	ioport_field *shift[SHIFT_COUNT];
	std::fill(std::begin(shift), std::end(shift), nullptr);
	for (auto const &port : manager.ports())
	{
		for (ioport_field &field : port.second->fields())
		{
			if (field.type() == IPT_KEYBOARD)
			{
				char32_t const code = field.keyboard_code(0);
				if ((code >= UCHAR_SHIFT_BEGIN) && (code <= UCHAR_SHIFT_END))
				{
					mask |= 1U << (code - UCHAR_SHIFT_BEGIN);
					shift[code - UCHAR_SHIFT_BEGIN] = &field;
				}
			}
		}
	}

	// iterate over ports and fields
	for (auto const &port : manager.ports())
	{
		for (ioport_field &field : port.second->fields())
		{
			if (field.type() == IPT_KEYBOARD)
			{
				// iterate over all shift states
				for (unsigned curshift = 0; curshift < SHIFT_STATES; ++curshift)
				{
					if (!(curshift & ~mask))
					{
						// fetch the code, ignoring 0 and shiters
						char32_t const code = field.keyboard_code(curshift);
						if (((code < UCHAR_SHIFT_BEGIN) || (code > UCHAR_SHIFT_END)) && (code != 0))
						{
							keycode_map_entry newcode;
							newcode.ch = code;
							std::fill(std::begin(newcode.field), std::end(newcode.field), nullptr);

							unsigned fieldnum = 0;
							for (unsigned i = 0, bits = curshift; (i < SHIFT_COUNT) && bits; ++i, bits >>= 1)
							{
								if (BIT(bits, 0))
									newcode.field[fieldnum++] = shift[i];
							}

							assert(fieldnum < ARRAY_LENGTH(newcode.field));
							newcode.field[fieldnum] = &field;
							m_keycode_map.push_back(std::move(newcode));

							if (LOG_NATURAL_KEYBOARD)
							{
								machine().logerror("natural_keyboard: code=%u (%s) port=%p field.name='%s'\n",
										code, unicode_to_string(code), (void *)&port, field.name());
							}
						}
					}
				}
			}
		}
	}
}


//-------------------------------------------------
//  can_post_directly - determine if the given
//  unicode character can be directly posted
//-------------------------------------------------

bool natural_keyboard::can_post_directly(char32_t ch)
{
	// if we have a queueing callback, then it depends on whether we can accept the character
	if (!m_queue_chars.isnull())
		return m_accept_char.isnull() ? true : m_accept_char(ch);

	// otherwise, it depends on the input codes
	const keycode_map_entry *code = find_code(ch);
	return (code != nullptr && code->field[0] != nullptr);
}


//-------------------------------------------------
//  can_post_alternate - determine if the given
//  unicode character can be posted via translation
//-------------------------------------------------

bool natural_keyboard::can_post_alternate(char32_t ch)
{
	const char_info *info = char_info::find(ch);
	if (info == nullptr)
		return false;

	const char *altstring = info->alternate;
	if (altstring == nullptr)
		return false;

	while (*altstring != 0)
	{
		char32_t uchar;
		int count = uchar_from_utf8(&uchar, altstring, strlen(altstring));
		if (count <= 0)
			return false;
		if (!can_post_directly(uchar))
			return false;
		altstring += count;
	}
	return true;
}


//-------------------------------------------------
//  choose_delay - determine the delay between
//  posting keyboard events
//-------------------------------------------------

attotime natural_keyboard::choose_delay(char32_t ch)
{
	// if we have a live rate, just use that
	if (m_current_rate != attotime::zero)
		return m_current_rate;

	// systems with queue_chars can afford a much smaller delay
	if (!m_queue_chars.isnull())
		return attotime::from_msec(10);

	// otherwise, default to constant delay with a longer delay on CR
	return attotime::from_msec((ch == '\r') ? 200 : 50);
}


//-------------------------------------------------
//  internal_post - post a keyboard event
//-------------------------------------------------

void natural_keyboard::internal_post(char32_t ch)
{
	// need to start up the timer?
	if (empty())
	{
		m_timer->adjust(choose_delay(ch));
		m_fieldnum = 0;
		m_status_keydown = false;
	}

	// add to the buffer, resizing if necessary
	m_buffer[m_bufend++] = ch;
	if ((m_bufend + 1) % m_buffer.size() == m_bufbegin)
		m_buffer.resize(m_buffer.size() + KEY_BUFFER_SIZE);
	m_bufend %= m_buffer.size();
}


//-------------------------------------------------
//  timer - timer callback to keep things flowing
//  when posting a string of characters
//-------------------------------------------------

void natural_keyboard::timer(void *ptr, int param)
{
	if (!m_queue_chars.isnull())
	{
		// the driver has a queue_chars handler
		while (!empty() && m_queue_chars(&m_buffer[m_bufbegin], 1))
		{
			m_bufbegin = (m_bufbegin + 1) % m_buffer.size();
			if (m_current_rate != attotime::zero)
				break;
		}
	}
	else
	{
		// the driver does not have a queue_chars handler

		// loop through this character's component codes
		const keycode_map_entry *const code = find_code(m_buffer[m_bufbegin]);
		bool advance;
		if (code)
		{
			do
			{
				assert(m_fieldnum < ARRAY_LENGTH(code->field));

				ioport_field *const field = code->field[m_fieldnum];
				if (field)
				{
					// special handling for toggle fields
					if (!field->live().toggle)
						field->set_value(!m_status_keydown);
					else if (!m_status_keydown)
						field->set_value(!field->digital_value());
				}
			}
			while (code->field[m_fieldnum] && (++m_fieldnum < ARRAY_LENGTH(code->field)) && m_status_keydown);
			advance = (m_fieldnum >= ARRAY_LENGTH(code->field)) || !code->field[m_fieldnum];
		}
		else
		{
			advance = true;
		}

		if (advance)
		{
			m_fieldnum = 0;
			m_status_keydown = !m_status_keydown;

			// proceed to next character when keydown expires
			if (!m_status_keydown)
				m_bufbegin = (m_bufbegin + 1) % m_buffer.size();
		}
	}

	// need to make sure timerproc is called again if buffer not empty
	if (!empty())
		m_timer->adjust(choose_delay(m_buffer[m_bufbegin]));
}


//-------------------------------------------------
//  unicode_to_string - obtain a string
//  representation of a given code; used for
//  logging and debugging
//-------------------------------------------------

std::string natural_keyboard::unicode_to_string(char32_t ch) const
{
	std::string buffer;
	switch (ch)
	{
	// check some magic values
	case '\0':  buffer.assign("\\0");      break;
	case '\r':  buffer.assign("\\r");      break;
	case '\n':  buffer.assign("\\n");      break;
	case '\t':  buffer.assign("\\t");      break;

	default:
		// seven bit ASCII is easy
		if (ch >= 32 && ch < 128)
		{
			char temp[2] = { char(ch), 0 };
			buffer.assign(temp);
		}
		else if (ch >= UCHAR_MAMEKEY_BEGIN)
		{
			// try to obtain a codename with code_name(); this can result in an empty string
			input_code code(DEVICE_CLASS_KEYBOARD, 0, ITEM_CLASS_SWITCH, ITEM_MODIFIER_NONE, input_item_id(ch - UCHAR_MAMEKEY_BEGIN));
			buffer = machine().input().code_name(code);
		}

		// did we fail to resolve? if so, we have a last resort
		if (buffer.empty())
			buffer = string_format("U+%04X", unsigned(ch));
		break;
	}
	return buffer;
}


//-------------------------------------------------
//  find_code - find a code in our lookup table
//-------------------------------------------------

const natural_keyboard::keycode_map_entry *natural_keyboard::find_code(char32_t ch) const
{
	for (auto & elem : m_keycode_map)
	{
		if (elem.ch == ch)
			return &elem;
	}
	return nullptr;
}


//-------------------------------------------------
//  dump - dumps info to stream
//-------------------------------------------------

void natural_keyboard::dump(std::ostream &str) const
{
	constexpr size_t left_column_width = 24;

	// loop through all codes
	for (auto &code : m_keycode_map)
	{
		// describe the character code
		std::string description = string_format("%08X (%s) ", code.ch, unicode_to_string(code.ch).c_str());

		// pad with spaces
		util::stream_format(str, "%-*s", left_column_width, description);

		// identify the keys used
		for (int field = 0; field < ARRAY_LENGTH(code.field) && code.field[field] != nullptr; field++)
			util::stream_format(str, "%s'%s'", (field > 0) ? ", " : "", code.field[field]->name());

		// carriage return
		str << '\n';
	}
}


//-------------------------------------------------
//  dump - dumps info to string
//-------------------------------------------------

std::string natural_keyboard::dump() const
{
	std::ostringstream buffer;
	dump(buffer);
	return buffer.str();
}


/***************************************************************************
    MISCELLANEOUS
***************************************************************************/

//-------------------------------------------------
//  find - look up information about a particular
//  character
//-------------------------------------------------

const char_info *char_info::find(char32_t target)
{
	// perform a simple binary search to find the proper alternate
	int low = 0;
	int high = ARRAY_LENGTH(charinfo);
	while (high > low)
	{
		int middle = (high + low) / 2;
		char32_t ch = charinfo[middle].ch;
		if (ch < target)
			low = middle + 1;
		else if (ch > target)
			high = middle;
		else
			return &charinfo[middle];
	}
	return nullptr;
}


//-------------------------------------------------
//  validate_natural_keyboard_statics -
//  validates natural keyboard static data
//-------------------------------------------------

/*
bool validate_natural_keyboard_statics(void)
{
    int i;
    bool error = false;
    char32_t last_char = 0;
    const char_info *ci;

    // check to make sure that charinfo is in order
    for (i = 0; i < ARRAY_LENGTH(charinfo); i++)
    {
        if (last_char >= charinfo[i].ch)
        {
            osd_printf_error("inputx: charinfo is out of order; 0x%08x should be higher than 0x%08x\n", charinfo[i].ch, last_char);
            error = true;
        }
        last_char = charinfo[i].ch;
    }

    // check to make sure that I can look up everything on alternate_charmap
    for (i = 0; i < ARRAY_LENGTH(charinfo); i++)
    {
        ci = char_info::find(charinfo[i].ch);
        if (ci != &charinfo[i])
        {
            osd_printf_error("ioport: expected char_info::find(0x%08x) to work properly\n", charinfo[i].ch);
            error = true;
        }
    }
    return error;
}
*/
