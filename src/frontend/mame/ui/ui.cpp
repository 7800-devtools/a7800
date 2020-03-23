// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria, Aaron Giles, Nathan Woods
/*********************************************************************

    ui.cpp

    Functions used to handle MAME's user interface.

*********************************************************************/

#include "emu.h"
#include "mame.h"
#include "emuopts.h"
#include "mameopts.h"
#include "video/vector.h"
#include "machine/laserdsc.h"
#include "drivenum.h"
#include "natkeyboard.h"
#include "render.h"
#include "luaengine.h"
#include "cheat.h"
#include "rendfont.h"
#include "uiinput.h"
#include "ui/ui.h"
#include "ui/info.h"
#include "ui/menu.h"
#include "ui/mainmenu.h"
#include "ui/filemngr.h"
#include "ui/sliders.h"
#include "ui/state.h"
#include "ui/viewgfx.h"
#include "imagedev/cassette.h"


/***************************************************************************
    CONSTANTS
***************************************************************************/

enum
{
	LOADSAVE_NONE,
	LOADSAVE_LOAD,
	LOADSAVE_SAVE
};


/***************************************************************************
    LOCAL VARIABLES
***************************************************************************/

// list of natural keyboard keys that are not associated with UI_EVENT_CHARs
static const input_item_id non_char_keys[] =
{
	ITEM_ID_ESC,
	ITEM_ID_F1,
	ITEM_ID_F2,
	ITEM_ID_F3,
	ITEM_ID_F4,
	ITEM_ID_F5,
	ITEM_ID_F6,
	ITEM_ID_F7,
	ITEM_ID_F8,
	ITEM_ID_F9,
	ITEM_ID_F10,
	ITEM_ID_F11,
	ITEM_ID_F12,
	ITEM_ID_NUMLOCK,
	ITEM_ID_0_PAD,
	ITEM_ID_1_PAD,
	ITEM_ID_2_PAD,
	ITEM_ID_3_PAD,
	ITEM_ID_4_PAD,
	ITEM_ID_5_PAD,
	ITEM_ID_6_PAD,
	ITEM_ID_7_PAD,
	ITEM_ID_8_PAD,
	ITEM_ID_9_PAD,
	ITEM_ID_DEL_PAD,
	ITEM_ID_PLUS_PAD,
	ITEM_ID_MINUS_PAD,
	ITEM_ID_INSERT,
	ITEM_ID_DEL,
	ITEM_ID_HOME,
	ITEM_ID_END,
	ITEM_ID_PGUP,
	ITEM_ID_PGDN,
	ITEM_ID_UP,
	ITEM_ID_DOWN,
	ITEM_ID_LEFT,
	ITEM_ID_RIGHT,
	ITEM_ID_PAUSE,
	ITEM_ID_CANCEL
};

static const char *s_color_list[] = {
	OPTION_UI_BORDER_COLOR,
	OPTION_UI_BACKGROUND_COLOR,
	OPTION_UI_GFXVIEWER_BG_COLOR,
	OPTION_UI_UNAVAILABLE_COLOR,
	OPTION_UI_TEXT_COLOR,
	OPTION_UI_TEXT_BG_COLOR,
	OPTION_UI_SUBITEM_COLOR,
	OPTION_UI_CLONE_COLOR,
	OPTION_UI_SELECTED_COLOR,
	OPTION_UI_SELECTED_BG_COLOR,
	OPTION_UI_MOUSEOVER_COLOR,
	OPTION_UI_MOUSEOVER_BG_COLOR,
	OPTION_UI_MOUSEDOWN_COLOR,
	OPTION_UI_MOUSEDOWN_BG_COLOR,
	OPTION_UI_DIPSW_COLOR,
	OPTION_UI_SLIDER_COLOR
};

/***************************************************************************
    GLOBAL VARIABLES
***************************************************************************/

// messagebox buffer
std::string mame_ui_manager::messagebox_text;
std::string mame_ui_manager::messagebox_poptext;
rgb_t mame_ui_manager::messagebox_backcolor;

// slider info
std::vector<ui::menu_item> mame_ui_manager::slider_list;
slider_state *mame_ui_manager::slider_current;


/***************************************************************************
    CORE IMPLEMENTATION
***************************************************************************/

static const uint32_t mouse_bitmap[32*32] =
{
	0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x09a46f30,0x81ac7c43,0x24af8049,0x00ad7d45,0x00a8753a,0x00a46f30,0x009f6725,0x009b611c,0x00985b14,0x0095560d,0x00935308,0x00915004,0x00904e02,0x008f4e01,0x008f4d00,0x008f4d00,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x00a16a29,0xa2aa783d,0xffbb864a,0xc0b0824c,0x5aaf7f48,0x09ac7b42,0x00a9773c,0x00a67134,0x00a26b2b,0x009e6522,0x009a5e19,0x00965911,0x0094550b,0x00925207,0x00915004,0x008f4e01,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x009a5e18,0x39a06827,0xffb97c34,0xffe8993c,0xffc88940,0xedac7c43,0x93ad7c44,0x2dac7c43,0x00ab793f,0x00a87438,0x00a46f30,0x00a06827,0x009c611d,0x00985c15,0x0095570e,0x00935309,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x00935308,0x00965810,0xcc9a5e19,0xffe78a21,0xfffb9929,0xfff49931,0xffd88e39,0xffb9813f,0xc9ac7c43,0x66ad7c44,0x0cac7a41,0x00a9773c,0x00a67134,0x00a26b2b,0x009e6522,0x009a5e19,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4e01,0x00904e02,0x60925106,0xffba670a,0xfff88b11,0xfff98f19,0xfff99422,0xfff9982b,0xffe89434,0xffc9883c,0xf3ac7a41,0x9cad7c44,0x39ac7c43,0x00ab7a40,0x00a87539,0x00a56f31,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008e4d00,0x008e4d00,0x098e4d00,0xea8f4d00,0xffee7f03,0xfff68407,0xfff6870d,0xfff78b15,0xfff78f1d,0xfff79426,0xfff49730,0xffd98d38,0xffbc823f,0xd2ac7c43,0x6fad7c44,0x12ac7b42,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008e4d00,0x008e4d00,0x008e4c00,0x8a8e4c00,0xffc46800,0xfff37e00,0xfff37f02,0xfff38106,0xfff3830a,0xfff48711,0xfff48b19,0xfff58f21,0xfff5942b,0xffe79134,0xffcb863b,0xf9ac7a41,0xa5ac7c43,0x3fac7c43,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008e4d00,0x008e4d00,0x008e4c00,0x218d4c00,0xfc8e4c00,0xffee7a00,0xfff07c00,0xfff17c00,0xfff17d02,0xfff17e04,0xfff18008,0xfff2830d,0xfff28614,0xfff38a1c,0xfff38f25,0xfff2932e,0xffd98b37,0xffbc813e,0xdbac7c43,0x78ad7c44,0x15ac7b42,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008e4d00,0x008e4d00,0x008e4d00,0x008e4c00,0xb18d4c00,0xffcf6b00,0xffed7900,0xffed7900,0xffee7900,0xffee7a01,0xffee7a01,0xffee7b03,0xffee7c06,0xffef7e0a,0xffef8110,0xfff08618,0xfff08a20,0xfff18f2a,0xffe78f33,0xffcc863b,0xfcab7a40,0xaeac7c43,0x4bac7c43,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008e4d00,0x008e4d00,0x008e4c00,0x488d4c00,0xffa85800,0xffe97500,0xffea7600,0xffea7600,0xffeb7600,0xffeb7600,0xffeb7600,0xffeb7701,0xffeb7702,0xffeb7804,0xffec7a07,0xffec7d0d,0xffec8013,0xffed851c,0xffee8a25,0xffee8f2e,0xffd98937,0xffbe813d,0xe4ab7a40,0x81ab7a40,0x1ba9763b,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008e4d00,0x008e4d00,0x008e4c00,0x008d4c00,0xdb8d4c00,0xffd86c00,0xffe77300,0xffe77300,0xffe87300,0xffe87300,0xffe87300,0xffe87300,0xffe87300,0xffe87401,0xffe87401,0xffe87503,0xffe97606,0xffe9780a,0xffe97c10,0xffea7f16,0xffeb831d,0xffeb8623,0xffe48426,0xffc67725,0xffa5661f,0xb7985c15,0x54935309,0x038e4d00,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008e4d00,0x008e4d00,0x008e4d00,0x008e4c00,0x6f8d4c00,0xffb25b00,0xffe36f00,0xffe47000,0xffe47000,0xffe57000,0xffe57000,0xffe57000,0xffe57000,0xffe57000,0xffe57000,0xffe57000,0xffe57000,0xffe57101,0xffe57000,0xffe47000,0xffe16e00,0xffde6c00,0xffd86900,0xffd06600,0xffc76200,0xffaa5500,0xff8a4800,0xea743f00,0x5a7a4200,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008f4d00,0x008e4d00,0x008e4d00,0x008e4c00,0x0f8d4c00,0xf38d4c00,0xffdc6a00,0xffe16d00,0xffe16d00,0xffe26d00,0xffe26d00,0xffe26d00,0xffe26d00,0xffe26d00,0xffe16d00,0xffe06c00,0xffde6b00,0xffd96900,0xffd16500,0xffc76000,0xffb95900,0xffab5200,0xff9c4b00,0xff894300,0xff6b3600,0xf9512c00,0xa5542d00,0x3c5e3200,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008f4d00,0x008e4d00,0x008e4d00,0x008e4c00,0x008d4c00,0x968d4c00,0xffbc5d00,0xffde6a00,0xffde6a00,0xffde6a00,0xffdf6a00,0xffdf6a00,0xffdf6a00,0xffde6a00,0xffdc6800,0xffd66600,0xffcc6100,0xffbf5b00,0xffaf5300,0xff9d4a00,0xff8a4200,0xff6d3500,0xff502900,0xe7402300,0x7b3f2200,0x15442500,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008f4d00,0x008f4d00,0x008e4d00,0x008e4d00,0x008e4c00,0x2a8d4c00,0xff9b5000,0xffda6600,0xffdb6700,0xffdb6700,0xffdc6700,0xffdc6700,0xffdb6700,0xffd96500,0xffd16200,0xffc25b00,0xffad5100,0xff974700,0xff7f3c00,0xff602f00,0xff472500,0xbd3d2100,0x513d2100,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008f4d00,0x008f4d00,0x008e4d00,0x008e4d00,0x008e4c00,0x008e4c00,0xc08d4c00,0xffc35c00,0xffd76300,0xffd76300,0xffd86300,0xffd86300,0xffd76300,0xffd06000,0xffc05800,0xffa54c00,0xff7f3b00,0xff582c00,0xf03f2200,0x903c2000,0x2a3e2100,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008e4d00,0x008e4d00,0x008e4c00,0x548d4c00,0xffa55200,0xffd35f00,0xffd46000,0xffd46000,0xffd46000,0xffd25e00,0xffc65900,0xffac4e00,0xff833c00,0xe7472600,0x693c2000,0x0c3d2100,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008e4d00,0x008e4d00,0x008e4c00,0x038d4c00,0xe48d4c00,0xffc95a00,0xffd15d00,0xffd15d00,0xffd15d00,0xffcb5a00,0xffb95200,0xff984300,0xff5f2e00,0x723f2200,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008e4d00,0x008e4d00,0x008e4d00,0x008e4c00,0x7b8d4c00,0xffad5200,0xffce5a00,0xffce5a00,0xffcd5900,0xffc35500,0xffaa4a00,0xff853a00,0xf9472600,0x15432400,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008e4d00,0x008e4d00,0x008e4c00,0x188d4c00,0xf98e4c00,0xffc95600,0xffcb5700,0xffc75500,0xffb94f00,0xff9b4200,0xff6c3100,0xab442500,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008e4d00,0x008e4d00,0x008e4d00,0x008e4c00,0xa58d4c00,0xffb35000,0xffc75300,0xffc05000,0xffac4800,0xff8b3a00,0xff542a00,0x45462500,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008e4d00,0x008e4d00,0x008e4c00,0x398d4c00,0xff994d00,0xffc24f00,0xffb74b00,0xff9e4000,0xff763200,0xde472600,0x03492800,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008e4d00,0x008e4d00,0x008e4c00,0x008e4c00,0xcf8d4c00,0xffb24b00,0xffab4500,0xff8d3900,0xff5e2b00,0x7e452500,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008e4d00,0x008e4d00,0x008e4d00,0x008e4c00,0x638d4c00,0xff984800,0xffa03f00,0xff7e3200,0xfc492800,0x1b472600,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008e4d00,0x008e4d00,0x008e4c00,0x098b4b00,0xed824600,0xff903800,0xff692c00,0xb4462600,0x004c2900,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008e4d00,0x008e4d00,0x008e4c00,0x008a4a00,0x8a7e4400,0xff793500,0xff572900,0x51472600,0x00542d00,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008e4d00,0x008d4c00,0x00884900,0x247a4200,0xfc633500,0xe74f2a00,0x034d2900,0x005e3300,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008e4d00,0x008d4c00,0x00884900,0x00794100,0xb4643600,0x87552e00,0x00593000,0x006b3900,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008f4d00,0x008d4c00,0x00884900,0x007c4300,0x486d3b00,0x24643600,0x00693800,0x00774000,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,
	0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff,0x00ffffff
};


//-------------------------------------------------
//  ctor - set up the user interface
//-------------------------------------------------

mame_ui_manager::mame_ui_manager(running_machine &machine)
	: ui_manager(machine)
	, m_font(nullptr)
	, m_handler_callback(nullptr)
	, m_handler_callback_type(ui_callback_type::GENERAL)
	, m_handler_param(0)
	, m_single_step(false)
	, m_showfps(false)
	, m_showfps_end(0)
	, m_show_profiler(false)
	, m_popup_text_end(0)
	, m_mouse_arrow_texture(nullptr)
	, m_mouse_show(false) {}

mame_ui_manager::~mame_ui_manager()
{
}

void mame_ui_manager::init()
{
	load_ui_options();
	// initialize the other UI bits
	ui::menu::init(machine(), options());
	ui_gfx_init(machine());

	get_font_rows(&machine());
	decode_ui_color(0, &machine());

	// more initialization
	using namespace std::placeholders;
	set_handler(ui_callback_type::GENERAL, std::bind(&mame_ui_manager::handler_messagebox, this, _1));
	m_non_char_keys_down = std::make_unique<uint8_t[]>((ARRAY_LENGTH(non_char_keys) + 7) / 8);
	m_mouse_show = machine().system().flags & MACHINE_CLICKABLE_ARTWORK ? true : false;

	// request a callback upon exiting
	machine().add_notifier(MACHINE_NOTIFY_EXIT, machine_notify_delegate(&mame_ui_manager::exit, this));

	// create mouse bitmap
	bitmap_argb32 *ui_mouse_bitmap = auto_alloc(machine(), bitmap_argb32(32, 32));
	uint32_t *dst = &ui_mouse_bitmap->pix32(0);
	memcpy(dst,mouse_bitmap,32*32*sizeof(uint32_t));
	m_mouse_arrow_texture = machine().render().texture_alloc();
	m_mouse_arrow_texture->set_bitmap(*ui_mouse_bitmap, ui_mouse_bitmap->cliprect(), TEXFORMAT_ARGB32);
}


//-------------------------------------------------
//  exit - clean up ourselves on exit
//-------------------------------------------------

void mame_ui_manager::exit()
{
	// free the mouse texture
	machine().render().texture_free(m_mouse_arrow_texture);
	m_mouse_arrow_texture = nullptr;

	// free the font
	if (m_font != nullptr)
	{
		machine().render().font_free(m_font);
		m_font = nullptr;
	}
}


//-------------------------------------------------
//  initialize - initialize ui lists
//-------------------------------------------------

void mame_ui_manager::initialize(running_machine &machine)
{
	m_machine_info = std::make_unique<ui::machine_info>(machine);

	// initialize the on-screen display system
	slider_list = slider_init(machine);
	if (slider_list.size() > 0)
	{
		slider_current = reinterpret_cast<slider_state *>(slider_list[0].ref);
	}
	else
	{
		slider_current = nullptr;
	}

	// if no test switch found, assign its input sequence to a service mode DIP
	if (!m_machine_info->has_test_switch() && m_machine_info->has_dips())
	{
		const char *const service_mode_dipname = ioport_configurer::string_from_token(DEF_STR(Service_Mode));
		for (auto &port : machine.ioport().ports())
			for (ioport_field &field : port.second->fields())
				if (field.type() == IPT_DIPSWITCH && strcmp(field.name(), service_mode_dipname) == 0)
					field.set_defseq(machine.ioport().type_seq(IPT_SERVICE));
	}
}


//-------------------------------------------------
//  set_handler - set a callback/parameter
//  pair for the current UI handler
//-------------------------------------------------

void mame_ui_manager::set_handler(ui_callback_type callback_type, const std::function<uint32_t (render_container &)> &&callback)
{
	m_handler_callback = std::move(callback);
	m_handler_callback_type = callback_type;
}


//-------------------------------------------------
//  display_startup_screens - display the
//  various startup screens
//-------------------------------------------------

void mame_ui_manager::display_startup_screens(bool first_time)
{
	const int maxstate = 3;
	int str = machine().options().seconds_to_run();
	bool show_gameinfo = !machine().options().skip_gameinfo();
	bool show_warnings = true, show_mandatory_fileman = true;
	int state;

	// disable everything if we are using -str for 300 or fewer seconds, or if we're the empty driver,
	// or if we are debugging
	if (!first_time || (str > 0 && str < 60*5) || &machine().system() == &GAME_NAME(___empty) || (machine().debug_flags & DEBUG_FLAG_ENABLED) != 0)
		show_gameinfo = show_warnings = show_mandatory_fileman = false;

	#if defined(EMSCRIPTEN)
	// also disable for the JavaScript port since the startup screens do not run asynchronously
	show_gameinfo = show_warnings = false;
	#endif

	// loop over states
	using namespace std::placeholders;
	set_handler(ui_callback_type::GENERAL, std::bind(&mame_ui_manager::handler_ingame, this, _1));
	for (state = 0; state < maxstate && !machine().scheduled_event_pending() && !ui::menu::stack_has_special_main_menu(machine()); state++)
	{
		// default to standard colors
		messagebox_backcolor = UI_BACKGROUND_COLOR;
		messagebox_text.clear();

		// pick the next state
		switch (state)
		{
			case 0:
				if (show_warnings)
					messagebox_text = machine_info().warnings_string();
				if (!messagebox_text.empty())
				{
					set_handler(ui_callback_type::MODAL, std::bind(&mame_ui_manager::handler_messagebox_anykey, this, _1));
					// TODO: don't think BTANB should be marked yellow? Also move this snippet to specific getter
					if (machine().system().flags & (MACHINE_WARNING_FLAGS|MACHINE_BTANB_FLAGS))
						messagebox_backcolor = UI_YELLOW_COLOR;
					if (machine().system().flags & (MACHINE_FATAL_FLAGS))
						messagebox_backcolor = UI_RED_COLOR;
				}
				break;

			case 1:
				if (show_gameinfo)
					messagebox_text = machine_info().game_info_string();
				if (!messagebox_text.empty())
					set_handler(ui_callback_type::MODAL, std::bind(&mame_ui_manager::handler_messagebox_anykey, this, _1));
				break;

			case 2:
				if (show_mandatory_fileman)
					messagebox_text = machine_info().mandatory_images();
				if (!messagebox_text.empty())
				{
					std::string warning = std::string(_("This driver requires images to be loaded in the following device(s): ")) + messagebox_text;
					ui::menu_file_manager::force_file_manager(*this, machine().render().ui_container(), warning.c_str());
				}
				break;
		}

		// clear the input memory
		machine().input().reset_polling();
		while (machine().input().poll_switches() != INPUT_CODE_INVALID) { }

		// loop while we have a handler
		while (m_handler_callback_type == ui_callback_type::MODAL && !machine().scheduled_event_pending() && !ui::menu::stack_has_special_main_menu(machine()))
		{
			machine().video().frame_update();
		}

		// clear the handler and force an update
		set_handler(ui_callback_type::GENERAL, std::bind(&mame_ui_manager::handler_ingame, this, _1));
		machine().video().frame_update();
	}

	// if we're the empty driver, force the menus on
	if (ui::menu::stack_has_special_main_menu(machine()))
		show_menu();
}


//-------------------------------------------------
//  set_startup_text - set the text to display
//  at startup
//-------------------------------------------------

void mame_ui_manager::set_startup_text(const char *text, bool force)
{
	static osd_ticks_t lastupdatetime = 0;
	osd_ticks_t curtime = osd_ticks();

	// copy in the new text
	messagebox_text.assign(text);
	messagebox_backcolor = UI_BACKGROUND_COLOR;

	// don't update more than 4 times/second
	if (force || (curtime - lastupdatetime) > osd_ticks_per_second() / 4)
	{
		lastupdatetime = curtime;
		machine().video().frame_update();
	}
}


//-------------------------------------------------
//  update_and_render - update the UI and
//  render it; called by video.c
//-------------------------------------------------

void mame_ui_manager::update_and_render(render_container &container)
{
	// always start clean
	container.empty();

	// if we're paused, dim the whole screen
	if (machine().phase() >= machine_phase::RESET && (single_step() || machine().paused()))
	{
		int alpha = (1.0f - machine().options().pause_brightness()) * 255.0f;
		if (ui::menu::stack_has_special_main_menu(machine()))
			alpha = 255;
		if (alpha > 255)
			alpha = 255;
		if (alpha >= 0)
			container.add_rect(0.0f, 0.0f, 1.0f, 1.0f, rgb_t(alpha,0x00,0x00,0x00), PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA));
	}

	// render any cheat stuff at the bottom
	if (machine().phase() >= machine_phase::RESET)
		mame_machine_manager::instance()->cheat().render_text(*this, container);

	// call the current UI handler
	m_handler_param = m_handler_callback(container);

	// display any popup messages
	if (osd_ticks() < m_popup_text_end)
		draw_text_box(container, messagebox_poptext.c_str(), ui::text_layout::CENTER, 0.5f, 0.9f, messagebox_backcolor);
	else
		m_popup_text_end = 0;

	// display the internal mouse cursor
	if (m_mouse_show || (is_menu_active() && machine().options().ui_mouse()))
	{
		int32_t mouse_target_x, mouse_target_y;
		bool mouse_button;
		render_target *mouse_target = machine().ui_input().find_mouse(&mouse_target_x, &mouse_target_y, &mouse_button);

		if (mouse_target != nullptr)
		{
			float mouse_y=-1,mouse_x=-1;
			if (mouse_target->map_point_container(mouse_target_x, mouse_target_y, container, mouse_x, mouse_y))
			{
				const float cursor_size = 0.6 * get_line_height();
				container.add_quad(mouse_x, mouse_y, mouse_x + cursor_size * container.manager().ui_aspect(&container), mouse_y + cursor_size, UI_TEXT_COLOR, m_mouse_arrow_texture, PRIMFLAG_ANTIALIAS(1) | PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA));
			}
		}
	}

	// cancel takes us back to the ingame handler
	if (m_handler_param == UI_HANDLER_CANCEL)
	{
		using namespace std::placeholders;
		set_handler(ui_callback_type::GENERAL, std::bind(&mame_ui_manager::handler_ingame, this, _1));
	}
}


//-------------------------------------------------
//  get_font - return the UI font
//-------------------------------------------------

render_font *mame_ui_manager::get_font()
{
	// allocate the font and messagebox string
	if (m_font == nullptr)
		m_font = machine().render().font_alloc(machine().options().ui_font());
	return m_font;
}


//-------------------------------------------------
//  get_line_height - return the current height
//  of a line
//-------------------------------------------------

float mame_ui_manager::get_line_height()
{
	int32_t raw_font_pixel_height = get_font()->pixel_height();
	render_target &ui_target = machine().render().ui_target();
	int32_t target_pixel_height = ui_target.height();
	float one_to_one_line_height;
	float scale_factor;

	// compute the font pixel height at the nominal size
	one_to_one_line_height = (float)raw_font_pixel_height / (float)target_pixel_height;

	// determine the scale factor
	scale_factor = UI_TARGET_FONT_HEIGHT / one_to_one_line_height;

	// if our font is small-ish, do integral scaling
	if (raw_font_pixel_height < 24)
	{
		// do we want to scale smaller? only do so if we exceed the threshold
		if (scale_factor <= 1.0f)
		{
			if (one_to_one_line_height < UI_MAX_FONT_HEIGHT || raw_font_pixel_height < 12)
				scale_factor = 1.0f;
		}

		// otherwise, just ensure an integral scale factor
		else
			scale_factor = floor(scale_factor);
	}

	// otherwise, just make sure we hit an even number of pixels
	else
	{
		int32_t height = scale_factor * one_to_one_line_height * (float)target_pixel_height;
		scale_factor = (float)height / (one_to_one_line_height * (float)target_pixel_height);
	}

	return scale_factor * one_to_one_line_height;
}


//-------------------------------------------------
//  get_char_width - return the width of a
//  single character
//-------------------------------------------------

float mame_ui_manager::get_char_width(char32_t ch)
{
	return get_font()->char_width(get_line_height(), machine().render().ui_aspect(), ch);
}


//-------------------------------------------------
//  get_string_width - return the width of a
//  character string
//-------------------------------------------------

float mame_ui_manager::get_string_width(const char *s, float text_size)
{
	return get_font()->utf8string_width(get_line_height() * text_size, machine().render().ui_aspect(), s);
}


//-------------------------------------------------
//  draw_outlined_box - add primitives to draw
//  an outlined box with the given background
//  color
//-------------------------------------------------

void mame_ui_manager::draw_outlined_box(render_container &container, float x0, float y0, float x1, float y1, rgb_t backcolor)
{
	draw_outlined_box(container, x0, y0, x1, y1, UI_BORDER_COLOR, backcolor);
}


//-------------------------------------------------
//  draw_outlined_box - add primitives to draw
//  an outlined box with the given background
//  color
//-------------------------------------------------

void mame_ui_manager::draw_outlined_box(render_container &container, float x0, float y0, float x1, float y1, rgb_t fgcolor, rgb_t bgcolor)
{
	container.add_rect(x0, y0, x1, y1, bgcolor, PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA));
	container.add_line(x0, y0, x1, y0, UI_LINE_WIDTH, fgcolor, PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA));
	container.add_line(x1, y0, x1, y1, UI_LINE_WIDTH, fgcolor, PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA));
	container.add_line(x1, y1, x0, y1, UI_LINE_WIDTH, fgcolor, PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA));
	container.add_line(x0, y1, x0, y0, UI_LINE_WIDTH, fgcolor, PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA));
}


//-------------------------------------------------
//  draw_text - simple text renderer
//-------------------------------------------------

void mame_ui_manager::draw_text(render_container &container, const char *buf, float x, float y)
{
	draw_text_full(container, buf, x, y, 1.0f - x, ui::text_layout::LEFT, ui::text_layout::WORD, mame_ui_manager::NORMAL, UI_TEXT_COLOR, UI_TEXT_BG_COLOR, nullptr, nullptr);
}


//-------------------------------------------------
//  draw_text_full - full featured text
//  renderer with word wrapping, justification,
//  and full size computation
//-------------------------------------------------

void mame_ui_manager::draw_text_full(render_container &container, const char *origs, float x, float y, float origwrapwidth, ui::text_layout::text_justify justify, ui::text_layout::word_wrapping wrap, draw_mode draw, rgb_t fgcolor, rgb_t bgcolor, float *totalwidth, float *totalheight, float text_size)
{
	// create the layout
	auto layout = create_layout(container, origwrapwidth, justify, wrap);

	// append text to it
	layout.add_text(
			origs,
			fgcolor,
			draw == OPAQUE_ ? bgcolor : rgb_t::transparent(),
			text_size);

	// and emit it (if we are asked to do so)
	if (draw != NONE)
		layout.emit(container, x, y);

	// return width/height
	if (totalwidth)
		*totalwidth = layout.actual_width();
	if (totalheight)
		*totalheight = layout.actual_height();
}


//-------------------------------------------------
//  draw_text_box - draw a multiline text
//  message with a box around it
//-------------------------------------------------

void mame_ui_manager::draw_text_box(render_container &container, const char *text, ui::text_layout::text_justify justify, float xpos, float ypos, rgb_t backcolor)
{
	// cap the maximum width
	float maximum_width = 1.0f - UI_BOX_LR_BORDER * 2;

	// create a layout
	ui::text_layout layout = create_layout(container, maximum_width, justify);

	// add text to it
	layout.add_text(text);

	// and draw the result
	draw_text_box(container, layout, xpos, ypos, backcolor);
}


//-------------------------------------------------
//  draw_text_box - draw a multiline text
//  message with a box around it
//-------------------------------------------------

void mame_ui_manager::draw_text_box(render_container &container, ui::text_layout &layout, float xpos, float ypos, rgb_t backcolor)
{
	// xpos and ypos are where we want to "pin" the layout, but we need to adjust for the actual size of the payload
	auto actual_left = layout.actual_left();
	auto actual_width = layout.actual_width();
	auto actual_height = layout.actual_height();
	auto x = std::min(std::max(xpos - actual_width / 2, UI_BOX_LR_BORDER), 1.0f - actual_width - UI_BOX_LR_BORDER);
	auto y = std::min(std::max(ypos - actual_height / 2, UI_BOX_TB_BORDER), 1.0f - actual_height - UI_BOX_TB_BORDER);

	// add a box around that
	draw_outlined_box(container,
			x - UI_BOX_LR_BORDER,
			y - UI_BOX_TB_BORDER,
			x + actual_width + UI_BOX_LR_BORDER,
			y + actual_height + UI_BOX_TB_BORDER, backcolor);

	// emit the text
	layout.emit(container, x - actual_left, y);
}


//-------------------------------------------------
//  draw_message_window - draw a multiline text
//  message with a box around it
//-------------------------------------------------

void mame_ui_manager::draw_message_window(render_container &container, const char *text)
{
	draw_text_box(container, text, ui::text_layout::text_justify::LEFT, 0.5f, 0.5f, UI_BACKGROUND_COLOR);
}


//-------------------------------------------------
//  show_fps_temp - show the FPS counter for
//  a specific period of time
//-------------------------------------------------

void mame_ui_manager::show_fps_temp(double seconds)
{
	if (!m_showfps)
		m_showfps_end = osd_ticks() + seconds * osd_ticks_per_second();
}


//-------------------------------------------------
//  set_show_fps - show/hide the FPS counter
//-------------------------------------------------

void mame_ui_manager::set_show_fps(bool show)
{
	m_showfps = show;
	if (!show)
	{
		m_showfps = 0;
		m_showfps_end = 0;
	}
}


//-------------------------------------------------
//  show_fps - return the current FPS
//  counter visibility state
//-------------------------------------------------

bool mame_ui_manager::show_fps() const
{
	return m_showfps || (m_showfps_end != 0);
}


//-------------------------------------------------
//  show_fps_counter
//-------------------------------------------------

bool mame_ui_manager::show_fps_counter()
{
	bool result = m_showfps || osd_ticks() < m_showfps_end;
	if (!result)
		m_showfps_end = 0;
	return result;
}


//-------------------------------------------------
//  set_show_profiler - show/hide the profiler
//-------------------------------------------------

void mame_ui_manager::set_show_profiler(bool show)
{
	m_show_profiler = show;
	g_profiler.enable(show);
}


//-------------------------------------------------
//  show_profiler - return the current
//  profiler visibility state
//-------------------------------------------------

bool mame_ui_manager::show_profiler() const
{
	return m_show_profiler;
}


//-------------------------------------------------
//  show_menu - show the menus
//-------------------------------------------------

void mame_ui_manager::show_menu()
{
	using namespace std::placeholders;
	set_handler(ui_callback_type::MENU, std::bind(&ui::menu::ui_handler, _1, std::ref(*this)));
}


//-------------------------------------------------
//  show_mouse - change mouse status
//-------------------------------------------------

void mame_ui_manager::show_mouse(bool status)
{
	m_mouse_show = status;
}


//-------------------------------------------------
//  is_menu_active - return true if the menu
//  UI handler is active
//-------------------------------------------------

bool mame_ui_manager::is_menu_active(void)
{
	return m_handler_callback_type == ui_callback_type::MENU
		|| m_handler_callback_type == ui_callback_type::VIEWER;
}



/***************************************************************************
    UI HANDLERS
***************************************************************************/

//-------------------------------------------------
//  handler_messagebox - displays the current
//  messagebox_text string but handles no input
//-------------------------------------------------

uint32_t mame_ui_manager::handler_messagebox(render_container &container)
{
	draw_text_box(container, messagebox_text.c_str(), ui::text_layout::LEFT, 0.5f, 0.5f, messagebox_backcolor);
	return 0;
}


//-------------------------------------------------
//  handler_messagebox_anykey - displays the
//  current messagebox_text string and waits for
//  any keypress
//-------------------------------------------------

uint32_t mame_ui_manager::handler_messagebox_anykey(render_container &container)
{
	uint32_t state = 0;

	// draw a standard message window
	draw_text_box(container, messagebox_text.c_str(), ui::text_layout::LEFT, 0.5f, 0.5f, messagebox_backcolor);

	// if the user cancels, exit out completely
	if (machine().ui_input().pressed(IPT_UI_CANCEL))
	{
		machine().schedule_exit();
		state = UI_HANDLER_CANCEL;
	}

	// if any key is pressed, just exit
	else if (machine().input().poll_switches() != INPUT_CODE_INVALID)
		state = UI_HANDLER_CANCEL;

	return state;
}


//-------------------------------------------------
//  process_natural_keyboard - processes any
//  natural keyboard input
//-------------------------------------------------

void mame_ui_manager::process_natural_keyboard()
{
	ui_event event;
	int i, pressed;
	input_item_id itemid;
	input_code code;
	uint8_t *key_down_ptr;
	uint8_t key_down_mask;

	// loop while we have interesting events
	while (machine().ui_input().pop_event(&event))
	{
		// if this was a UI_EVENT_CHAR event, post it
		if (event.event_type == UI_EVENT_CHAR)
			machine().ioport().natkeyboard().post(event.ch);
	}

	// process natural keyboard keys that don't get UI_EVENT_CHARs
	for (i = 0; i < ARRAY_LENGTH(non_char_keys); i++)
	{
		// identify this keycode
		itemid = non_char_keys[i];
		code = machine().input().code_from_itemid(itemid);

		// ...and determine if it is pressed
		pressed = machine().input().code_pressed(code);

		// figure out whey we are in the key_down map
		key_down_ptr = &m_non_char_keys_down[i / 8];
		key_down_mask = 1 << (i % 8);

		if (pressed && !(*key_down_ptr & key_down_mask))
		{
			// this key is now down
			*key_down_ptr |= key_down_mask;

			// post the key
			machine().ioport().natkeyboard().post(UCHAR_MAMEKEY_BEGIN + code.item_id());
		}
		else if (!pressed && (*key_down_ptr & key_down_mask))
		{
			// this key is now up
			*key_down_ptr &= ~key_down_mask;
		}
	}
}


//-------------------------------------------------
//  increase_frameskip
//-------------------------------------------------

void mame_ui_manager::increase_frameskip()
{
	// get the current value and increment it
	int newframeskip = machine().video().frameskip() + 1;
	if (newframeskip > MAX_FRAMESKIP)
		newframeskip = -1;
	machine().video().set_frameskip(newframeskip);

	// display the FPS counter for 2 seconds
	show_fps_temp(2.0);
}


//-------------------------------------------------
//  decrease_frameskip
//-------------------------------------------------

void mame_ui_manager::decrease_frameskip()
{
	// get the current value and decrement it
	int newframeskip = machine().video().frameskip() - 1;
	if (newframeskip < -1)
		newframeskip = MAX_FRAMESKIP;
	machine().video().set_frameskip(newframeskip);

	// display the FPS counter for 2 seconds
	show_fps_temp(2.0);
}


//-------------------------------------------------
//  can_paste
//-------------------------------------------------

bool mame_ui_manager::can_paste()
{
	// retrieve the clipboard text
	char *text = osd_get_clipboard_text();

	// free the string if allocated
	if (text != nullptr)
		free(text);

	// did we have text?
	return text != nullptr;
}


//-------------------------------------------------
//  paste - does a paste from the keyboard
//-------------------------------------------------

void mame_ui_manager::paste()
{
	// retrieve the clipboard text
	char *text = osd_get_clipboard_text();

	// was a result returned?
	if (text != nullptr)
	{
		// post the text
		machine().ioport().natkeyboard().post_utf8(text);

		// free the string
		free(text);
	}
}


//-------------------------------------------------
//  draw_fps_counter
//-------------------------------------------------

void mame_ui_manager::draw_fps_counter(render_container &container)
{
	draw_text_full(container, machine().video().speed_text().c_str(), 0.0f, 0.0f, 1.0f,
		ui::text_layout::RIGHT, ui::text_layout::WORD, OPAQUE_, rgb_t::white(), rgb_t::black(), nullptr, nullptr);
}


//-------------------------------------------------
//  draw_timecode_counter
//-------------------------------------------------

void mame_ui_manager::draw_timecode_counter(render_container &container)
{
	std::string tempstring;
	draw_text_full(container, machine().video().timecode_text(tempstring).c_str(), 0.0f, 0.0f, 1.0f,
		ui::text_layout::RIGHT, ui::text_layout::WORD, OPAQUE_, rgb_t(0xf0, 0xf0, 0x10, 0x10), rgb_t::black(), nullptr, nullptr);
}


//-------------------------------------------------
//  draw_timecode_total
//-------------------------------------------------

void mame_ui_manager::draw_timecode_total(render_container &container)
{
	std::string tempstring;
	draw_text_full(container, machine().video().timecode_total_text(tempstring).c_str(), 0.0f, 0.0f, 1.0f,
		ui::text_layout::LEFT, ui::text_layout::WORD, OPAQUE_, rgb_t(0xf0, 0x10, 0xf0, 0x10), rgb_t::black(), nullptr, nullptr);
}


//-------------------------------------------------
//  draw_profiler
//-------------------------------------------------

void mame_ui_manager::draw_profiler(render_container &container)
{
	const char *text = g_profiler.text(machine());
	draw_text_full(container, text, 0.0f, 0.0f, 1.0f, ui::text_layout::LEFT, ui::text_layout::WORD, OPAQUE_, rgb_t::white(), rgb_t::black(), nullptr, nullptr);
}


//-------------------------------------------------
//  start_save_state
//-------------------------------------------------

void mame_ui_manager::start_save_state()
{
	ui::menu::stack_reset(machine());
	show_menu();
	ui::menu::stack_push<ui::menu_save_state>(*this, machine().render().ui_container());
}


//-------------------------------------------------
//  start_load_state
//-------------------------------------------------

void mame_ui_manager::start_load_state()
{
	ui::menu::stack_reset(machine());
	show_menu();
	ui::menu::stack_push<ui::menu_load_state>(*this, machine().render().ui_container());
}


//-------------------------------------------------
//  image_handler_ingame - execute display
//  callback function for each image device
//-------------------------------------------------

void mame_ui_manager::image_handler_ingame()
{
	// run display routine for devices
	if (machine().phase() == machine_phase::RUNNING)
	{
		auto layout = create_layout(machine().render().ui_container());

		// loop through all devices, build their text into the layout
		for (device_image_interface &image : image_interface_iterator(machine().root_device()))
		{
			std::string str = image.call_display();
			if (!str.empty())
			{
				layout.add_text(str.c_str());
				layout.add_text("\n");
			}
		}

		// did we actually create anything?
		if (!layout.empty())
		{
			float x = 0.2f;
			float y = 0.5f * get_line_height() + 2.0f * UI_BOX_TB_BORDER;
			draw_text_box(machine().render().ui_container(), layout, x, y, UI_BACKGROUND_COLOR);
		}
	}
}

//-------------------------------------------------
//  handler_ingame - in-game handler takes care
//  of the standard keypresses
//-------------------------------------------------

uint32_t mame_ui_manager::handler_ingame(render_container &container)
{
	bool is_paused = machine().paused();

	// first draw the FPS counter
	if (show_fps_counter())
		draw_fps_counter(container);

	// Show the duration of current part (intro or gameplay or extra)
	if (show_timecode_counter())
		draw_timecode_counter(container);

	// Show the total time elapsed for the video preview (all parts intro, gameplay, extras)
	if (show_timecode_total())
		draw_timecode_total(container);

	// draw the profiler if visible
	if (show_profiler())
		draw_profiler(container);

	// if we're single-stepping, pause now
	if (single_step())
	{
		machine().pause();
		set_single_step(false);
	}

	// determine if we should disable the rest of the UI
	bool has_keyboard = machine_info().has_keyboard();
	bool ui_disabled = (has_keyboard && !machine().ui_active());

	// is ScrLk UI toggling applicable here?
	if (has_keyboard)
	{
		// are we toggling the UI with ScrLk?
		if (machine().ui_input().pressed(IPT_UI_TOGGLE_UI))
		{
			// toggle the UI
			machine().set_ui_active(!machine().ui_active());

			// display a popup indicating the new status
			if (machine().ui_active())
			{
				popup_time(2, "%s\n%s\n%s\n%s\n%s\n%s\n",
					_("Keyboard Emulation Status"),
					"-------------------------",
					_("Mode: PARTIAL Emulation"),
					_("UI:   Enabled"),
					"-------------------------",
					_("**Use ScrLock to toggle**"));
			}
			else
			{
				popup_time(2, "%s\n%s\n%s\n%s\n%s\n%s\n",
					_("Keyboard Emulation Status"),
					"-------------------------",
					_("Mode: FULL Emulation"),
					_("UI:   Disabled"),
					"-------------------------",
					_("**Use ScrLock to toggle**"));
			}
		}
	}

	// is the natural keyboard enabled?
	if (machine().ioport().natkeyboard().in_use() && (machine().phase() == machine_phase::RUNNING))
		process_natural_keyboard();

	if (!ui_disabled)
	{
		// paste command
		if (machine().ui_input().pressed(IPT_UI_PASTE))
			paste();
	}

	image_handler_ingame();

	// handle a save input timecode request
	if (machine().ui_input().pressed(IPT_UI_TIMECODE))
		machine().video().save_input_timecode();

	if (ui_disabled) return ui_disabled;

	if (machine().ui_input().pressed(IPT_UI_CANCEL))
	{
		request_quit();
		return 0;
	}

	// turn on menus if requested
	if (machine().ui_input().pressed(IPT_UI_CONFIGURE))
	{
		show_menu();
		return 0;
	}

	// if the on-screen display isn't up and the user has toggled it, turn it on
	if ((machine().debug_flags & DEBUG_FLAG_ENABLED) == 0 && machine().ui_input().pressed(IPT_UI_ON_SCREEN_DISPLAY))
	{
		using namespace std::placeholders;
		set_handler(ui_callback_type::MENU, std::bind(&ui::menu_sliders::ui_handler, _1, std::ref(*this)));
		return 1;
	}

	// handle a reset request
	if (machine().ui_input().pressed(IPT_UI_RESET_MACHINE))
		machine().schedule_hard_reset();
	if (machine().ui_input().pressed(IPT_UI_SOFT_RESET))
		machine().schedule_soft_reset();

	// handle a request to display graphics/palette
	if (machine().ui_input().pressed(IPT_UI_SHOW_GFX))
	{
		if (!is_paused)
			machine().pause();
		using namespace std::placeholders;
		set_handler(ui_callback_type::VIEWER, std::bind(&ui_gfx_ui_handler, _1, std::ref(*this), is_paused));
		return is_paused ? 1 : 0;
	}

	// handle a tape control key
	if (machine().ui_input().pressed(IPT_UI_TAPE_START))
	{
		for (cassette_image_device &cass : cassette_device_iterator(machine().root_device()))
		{
			cass.change_state(CASSETTE_PLAY, CASSETTE_MASK_UISTATE);
			return 0;
		}
	}
	if (machine().ui_input().pressed(IPT_UI_TAPE_STOP))
	{
		for (cassette_image_device &cass : cassette_device_iterator(machine().root_device()))
		{
			cass.change_state(CASSETTE_STOPPED, CASSETTE_MASK_UISTATE);
			return 0;
		}
	}

	// handle a save state request
	if (machine().ui_input().pressed(IPT_UI_SAVE_STATE))
	{
		start_save_state();
		return LOADSAVE_SAVE;
	}

	// handle a load state request
	if (machine().ui_input().pressed(IPT_UI_LOAD_STATE))
	{
		start_load_state();
		return LOADSAVE_LOAD;
	}

	// handle a save snapshot request
	if (machine().ui_input().pressed(IPT_UI_SNAPSHOT))
		machine().video().save_active_screen_snapshots();

	// toggle pause
	if (machine().ui_input().pressed(IPT_UI_PAUSE))
		machine().toggle_pause();

	// pause single step
	if (machine().ui_input().pressed(IPT_UI_PAUSE_SINGLE))
	{
		set_single_step(true);
		machine().resume();
	}

	// handle a toggle cheats request
	if (machine().ui_input().pressed(IPT_UI_TOGGLE_CHEAT))
		mame_machine_manager::instance()->cheat().set_enable(!mame_machine_manager::instance()->cheat().enabled());

	// toggle movie recording
	if (machine().ui_input().pressed(IPT_UI_RECORD_MOVIE))
		machine().video().toggle_record_movie();

	// toggle profiler display
	if (machine().ui_input().pressed(IPT_UI_SHOW_PROFILER))
		set_show_profiler(!show_profiler());

	// toggle FPS display
	if (machine().ui_input().pressed(IPT_UI_SHOW_FPS))
		set_show_fps(!show_fps());

	// increment frameskip?
	if (machine().ui_input().pressed(IPT_UI_FRAMESKIP_INC))
		increase_frameskip();

	// decrement frameskip?
	if (machine().ui_input().pressed(IPT_UI_FRAMESKIP_DEC))
		decrease_frameskip();

	// toggle throttle?
	if (machine().ui_input().pressed(IPT_UI_THROTTLE))
		machine().video().toggle_throttle();

	// toggle autofire
	if (machine().ui_input().pressed(IPT_UI_TOGGLE_AUTOFIRE))
	{
		if (!machine().options().cheat())
		{
			machine().popmessage(_("Autofire can't be enabled"));
		}
		else
		{
			bool autofire_toggle = machine().ioport().get_autofire_toggle();
			machine().ioport().set_autofire_toggle(!autofire_toggle);
			machine().popmessage("Autofire %s", autofire_toggle ? _("Enabled") : _("Disabled"));
		}
	}

	// check for fast forward
	if (machine().ioport().type_pressed(IPT_UI_FAST_FORWARD))
	{
		machine().video().set_fastforward(true);
		show_fps_temp(0.5);
	}
	else
		machine().video().set_fastforward(false);

	return 0;
}


//-------------------------------------------------
//  request_quit
//-------------------------------------------------

void mame_ui_manager::request_quit()
{
	using namespace std::placeholders;
	if (!machine().options().confirm_quit())
		machine().schedule_exit();
	else
		set_handler(ui_callback_type::GENERAL, std::bind(&mame_ui_manager::handler_confirm_quit, this, _1));
}


//-------------------------------------------------
//  handler_confirm_quit - leads the user through
//  confirming quit emulation
//-------------------------------------------------

uint32_t mame_ui_manager::handler_confirm_quit(render_container &container)
{
	uint32_t state = 0;

	// get the text for 'UI Select'
	std::string ui_select_text = machine().input().seq_name(machine().ioport().type_seq(IPT_UI_SELECT, 0, SEQ_TYPE_STANDARD));

	// get the text for 'UI Cancel'
	std::string ui_cancel_text = machine().input().seq_name(machine().ioport().type_seq(IPT_UI_CANCEL, 0, SEQ_TYPE_STANDARD));

	// assemble the quit message
	std::string quit_message = string_format(_("Are you sure you want to quit?\n\n"
			"Press ''%1$s'' to quit,\n"
			"Press ''%2$s'' to return to emulation."),
			ui_select_text,
			ui_cancel_text);

	draw_text_box(container, quit_message.c_str(), ui::text_layout::CENTER, 0.5f, 0.5f, UI_RED_COLOR);
	machine().pause();

	// if the user press ENTER, quit the game
	if (machine().ui_input().pressed(IPT_UI_SELECT))
		machine().schedule_exit();

	// if the user press ESC, just continue
	else if (machine().ui_input().pressed(IPT_UI_CANCEL))
	{
		machine().resume();
		state = UI_HANDLER_CANCEL;
	}

	return state;
}


/***************************************************************************
    SLIDER CONTROLS
***************************************************************************/

//-------------------------------------------------
//  ui_get_slider_list - get the list of sliders
//-------------------------------------------------

std::vector<ui::menu_item>& mame_ui_manager::get_slider_list(void)
{
	return slider_list;
}


//-------------------------------------------------
//  slider_alloc - allocate a new slider entry
//-------------------------------------------------

slider_state* mame_ui_manager::slider_alloc(running_machine &machine, int id, const char *title, int32_t minval, int32_t defval, int32_t maxval, int32_t incval, void *arg)
{
	int size = sizeof(slider_state) + strlen(title);
	slider_state *state = (slider_state *)auto_alloc_array_clear(machine, uint8_t, size);

	state->minval = minval;
	state->defval = defval;
	state->maxval = maxval;
	state->incval = incval;

	using namespace std::placeholders;
	state->update = std::bind(&mame_ui_manager::slider_changed, this, _1, _2, _3, _4, _5);

	state->arg = arg;
	state->id = id;
	strcpy(state->description, title);

	return state;
}


//----------------------------------------------------------
//  mame_ui_manager::slider_init - initialize the list of slider
//  controls
//----------------------------------------------------------

std::vector<ui::menu_item> mame_ui_manager::slider_init(running_machine &machine)
{
	std::vector<slider_state *> sliders;

	// add overall volume
	sliders.push_back(slider_alloc(machine, SLIDER_ID_VOLUME, _("Master Volume"), -32, 0, 0, 1, nullptr));

	// add per-channel volume
	mixer_input info;
	for (int item = 0; machine.sound().indexed_mixer_input(item, info); item++)
	{
		int32_t maxval = 2000;
		int32_t defval = 1000;

		std::string str = string_format(_("%1$s Volume"), info.stream->input_name(info.inputnum));
		sliders.push_back(slider_alloc(machine, SLIDER_ID_MIXERVOL + item, str.c_str(), 0, defval, maxval, 20, (void *)(uintptr_t)item));
	}

	// add analog adjusters
	int slider_index = 0;
	for (auto &port : machine.ioport().ports())
	{
		for (ioport_field &field : port.second->fields())
		{
			if (field.type() == IPT_ADJUSTER)
			{
				sliders.push_back(slider_alloc(machine, SLIDER_ID_ADJUSTER + slider_index++, field.name(), field.minval(), field.defvalue(), field.maxval(), 1, (void *)&field));
			}
		}
	}

	// add CPU overclocking (cheat only)
	slider_index = 0;
	if (machine.options().cheat())
	{
		for (device_execute_interface &exec : execute_interface_iterator(machine.root_device()))
		{
			void *param = (void *)&exec.device();
			std::string str = string_format(_("Overclock CPU %1$s"), exec.device().tag());
			sliders.push_back(slider_alloc(machine, SLIDER_ID_OVERCLOCK + slider_index++, str.c_str(), 10, 1000, 2000, 1, param));
		}
		for (device_sound_interface &snd : sound_interface_iterator(machine.root_device()))
		{
			device_execute_interface *exec;
			if (!snd.device().interface(exec) && snd.device().unscaled_clock() != 0)
			{
				void *param = (void *)&snd.device();
				std::string str = string_format(_("Overclock %1$s sound"), snd.device().tag());
				sliders.push_back(slider_alloc(machine, SLIDER_ID_OVERCLOCK + slider_index++, str.c_str(), 10, 1000, 2000, 1, param));
			}
		}
	}

	// add screen parameters
	screen_device_iterator scriter(machine.root_device());
	slider_index = 0;
	for (screen_device &screen : scriter)
	{
		int defxscale = floor(screen.xscale() * 1000.0f + 0.5f);
		int defyscale = floor(screen.yscale() * 1000.0f + 0.5f);
		int defxoffset = floor(screen.xoffset() * 1000.0f + 0.5f);
		int defyoffset = floor(screen.yoffset() * 1000.0f + 0.5f);
		void *param = (void *)&screen;
		std::string screen_desc = machine_info().get_screen_desc(screen);

		// add refresh rate tweaker
		if (machine.options().cheat())
		{
			std::string str = string_format(_("%1$s Refresh Rate"), screen_desc);
			sliders.push_back(slider_alloc(machine, SLIDER_ID_REFRESH + slider_index, str.c_str(), -10000, 0, 10000, 1000, param));
		}

		// add standard brightness/contrast/gamma controls per-screen
		std::string str = string_format(_("%1$s Brightness"), screen_desc);
		sliders.push_back(slider_alloc(machine, SLIDER_ID_BRIGHTNESS + slider_index, str.c_str(), 100, 1000, 2000, 10, param));
		str = string_format(_("%1$s Contrast"), screen_desc);
		sliders.push_back(slider_alloc(machine, SLIDER_ID_CONTRAST + slider_index, str.c_str(), 100, 1000, 2000, 50, param));
		str = string_format(_("%1$s Gamma"), screen_desc);
		sliders.push_back(slider_alloc(machine, SLIDER_ID_GAMMA + slider_index, str.c_str(), 100, 1000, 3000, 50, param));

		// add scale and offset controls per-screen
		str = string_format(_("%1$s Horiz Stretch"), screen_desc);
		sliders.push_back(slider_alloc(machine, SLIDER_ID_XSCALE + slider_index, str.c_str(), 500, defxscale, 1500, 2, param));
		str = string_format(_("%1$s Horiz Position"), screen_desc);
		sliders.push_back(slider_alloc(machine, SLIDER_ID_XOFFSET + slider_index, str.c_str(), -500, defxoffset, 500, 2, param));
		str = string_format(_("%1$s Vert Stretch"), screen_desc);
		sliders.push_back(slider_alloc(machine, SLIDER_ID_YSCALE + slider_index, str.c_str(), 500, defyscale, 1500, 2, param));
		str = string_format(_("%1$s Vert Position"), screen_desc);
		sliders.push_back(slider_alloc(machine, SLIDER_ID_YOFFSET + slider_index, str.c_str(), -500, defyoffset, 500, 2, param));
		slider_index++;
	}

	slider_index = 0;
	for (laserdisc_device &laserdisc : laserdisc_device_iterator(machine.root_device()))
	{
		if (laserdisc.overlay_configured())
		{
			laserdisc_overlay_config config;
			laserdisc.get_overlay_config(config);
			int defxscale = floor(config.m_overscalex * 1000.0f + 0.5f);
			int defyscale = floor(config.m_overscaley * 1000.0f + 0.5f);
			int defxoffset = floor(config.m_overposx * 1000.0f + 0.5f);
			int defyoffset = floor(config.m_overposy * 1000.0f + 0.5f);
			void *param = (void *)&laserdisc;

			// add scale and offset controls per-overlay
			std::string str = string_format(_("Laserdisc '%1$s' Horiz Stretch"), laserdisc.tag());
			sliders.push_back(slider_alloc(machine, SLIDER_ID_OVERLAY_XSCALE + slider_index, str.c_str(), 500, (defxscale == 0) ? 1000 : defxscale, 1500, 2, param));
			str = string_format(_("Laserdisc '%1$s' Horiz Position"), laserdisc.tag());
			sliders.push_back(slider_alloc(machine, SLIDER_ID_OVERLAY_YSCALE + slider_index, str.c_str(), -500, defxoffset, 500, 2, param));
			str = string_format(_("Laserdisc '%1$s' Vert Stretch"), laserdisc.tag());
			sliders.push_back(slider_alloc(machine, SLIDER_ID_OVERLAY_XOFFSET + slider_index, str.c_str(), 500, (defyscale == 0) ? 1000 : defyscale, 1500, 2, param));
			str = string_format(_("Laserdisc '%1$s' Vert Position"), laserdisc.tag());
			sliders.push_back(slider_alloc(machine, SLIDER_ID_OVERLAY_YOFFSET + slider_index, str.c_str(), -500, defyoffset, 500, 2, param));
			slider_index++;
		}
	}

	slider_index = 0;
	for (screen_device &screen : scriter)
	{
		if (screen.screen_type() == SCREEN_TYPE_VECTOR)
		{
			// add vector control
			sliders.push_back(slider_alloc(machine, SLIDER_ID_FLICKER + slider_index, _("Vector Flicker"), 0, 0, 1000, 10, nullptr));
			sliders.push_back(slider_alloc(machine, SLIDER_ID_BEAM_WIDTH_MIN + slider_index, _("Beam Width Minimum"), 100, 100, 1000, 1, nullptr));
			sliders.push_back(slider_alloc(machine, SLIDER_ID_BEAM_WIDTH_MAX + slider_index, _("Beam Width Maximum"), 100, 100, 1000, 1, nullptr));
			sliders.push_back(slider_alloc(machine, SLIDER_ID_BEAM_INTENSITY + slider_index, _("Beam Intensity Weight"), -1000, 0, 1000, 10, nullptr));
			slider_index++;
			break;
		}
	}

#ifdef MAME_DEBUG
	slider_index = 0;
	// add crosshair adjusters
	for (auto &port : machine.ioport().ports())
	{
		for (ioport_field &field : port.second->fields())
		{
			if (field.crosshair_axis() != CROSSHAIR_AXIS_NONE && field.player() == 0)
			{
				std::string str = string_format(_("Crosshair Scale %1$s"), (field.crosshair_axis() == CROSSHAIR_AXIS_X) ? _("X") : _("Y"));
				sliders.push_back(slider_alloc(machine, SLIDER_ID_CROSSHAIR_SCALE + slider_index, str.c_str(), -3000, 1000, 3000, 100, (void *)&field));
				str = string_format(_("Crosshair Offset %1$s"), (field.crosshair_axis() == CROSSHAIR_AXIS_X) ? _("X") : _("Y"));
				sliders.push_back(slider_alloc(machine, SLIDER_ID_CROSSHAIR_OFFSET + slider_index, str.c_str(), -3000, 0, 3000, 100, (void *)&field));
			}
		}
	}
#endif

	std::vector<ui::menu_item> items;
	for (slider_state *slider : sliders)
	{
		ui::menu_item item;
		item.text = slider->description;
		item.subtext = "";
		item.flags = 0;
		item.ref = slider;
		item.type = ui::menu_item_type::SLIDER;
		items.push_back(item);
	}

	return items;
}

//----------------------------------------------------
//  slider_changed - global slider-modified callback
//----------------------------------------------------

int32_t mame_ui_manager::slider_changed(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	if (id == SLIDER_ID_VOLUME)
		return slider_volume(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_MIXERVOL && id <= SLIDER_ID_MIXERVOL_LAST)
		return slider_mixervol(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_ADJUSTER && id <= SLIDER_ID_ADJUSTER_LAST)
			return slider_adjuster(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_OVERCLOCK && id <= SLIDER_ID_OVERCLOCK_LAST)
			return slider_overclock(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_REFRESH && id <= SLIDER_ID_REFRESH_LAST)
			return slider_refresh(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_BRIGHTNESS && id <= SLIDER_ID_BRIGHTNESS_LAST)
			return slider_brightness(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_CONTRAST && id <= SLIDER_ID_CONTRAST_LAST)
			return slider_contrast(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_GAMMA && id <= SLIDER_ID_GAMMA_LAST)
			return slider_gamma(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_XSCALE && id <= SLIDER_ID_XSCALE_LAST)
			return slider_xscale(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_YSCALE && id <= SLIDER_ID_YSCALE_LAST)
			return slider_yscale(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_XOFFSET && id <= SLIDER_ID_XOFFSET_LAST)
			return slider_xoffset(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_YOFFSET && id <= SLIDER_ID_YOFFSET_LAST)
			return slider_yoffset(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_OVERLAY_XSCALE && id <= SLIDER_ID_OVERLAY_XSCALE_LAST)
			return slider_overxscale(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_OVERLAY_YSCALE && id <= SLIDER_ID_OVERLAY_YSCALE_LAST)
			return slider_overyscale(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_OVERLAY_XOFFSET && id <= SLIDER_ID_OVERLAY_XOFFSET_LAST)
			return slider_overxoffset(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_OVERLAY_YOFFSET && id <= SLIDER_ID_OVERLAY_YOFFSET_LAST)
			return slider_overyoffset(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_FLICKER && id <= SLIDER_ID_FLICKER_LAST)
			return slider_flicker(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_BEAM_WIDTH_MIN && id <= SLIDER_ID_BEAM_WIDTH_MIN_LAST)
			return slider_beam_width_min(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_BEAM_WIDTH_MAX && id <= SLIDER_ID_BEAM_WIDTH_MAX_LAST)
			return slider_beam_width_max(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_BEAM_INTENSITY && id <= SLIDER_ID_BEAM_INTENSITY_LAST)
			return slider_beam_intensity_weight(machine, arg, id, str, newval);
#ifdef MAME_DEBUG
	else if (id >= SLIDER_ID_CROSSHAIR_SCALE && id <= SLIDER_ID_CROSSHAIR_SCALE_LAST)
			return slider_crossscale(machine, arg, id, str, newval);
	else if (id >= SLIDER_ID_CROSSHAIR_OFFSET && id <= SLIDER_ID_CROSSHAIR_OFFSET_LAST)
			return slider_crossoffset(machine, arg, id, str, newval);
#endif

	return 0;
}


//-------------------------------------------------
//  slider_volume - global volume slider callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_volume(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	if (newval != SLIDER_NOCHANGE)
		machine.sound().set_attenuation(newval);
	if (str)
		*str = string_format(_("%1$3ddB"), machine.sound().attenuation());
	return machine.sound().attenuation();
}


//-------------------------------------------------
//  slider_mixervol - single channel volume
//  slider callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_mixervol(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	mixer_input info;
	if (!machine.sound().indexed_mixer_input((uintptr_t)arg, info))
		return 0;
	if (newval != SLIDER_NOCHANGE)
	{
		int32_t curval = floor(info.stream->user_gain(info.inputnum) * 1000.0f + 0.5f);
		if (newval > curval && (newval - curval) <= 4) newval += 4; // round up on increment
		info.stream->set_user_gain(info.inputnum, (float)newval * 0.001f);
	}
	if (str)
		*str = string_format("%4.2f", info.stream->user_gain(info.inputnum));
	return floorf(info.stream->user_gain(info.inputnum) * 1000.0f + 0.5f);
}


//-------------------------------------------------
//  slider_adjuster - analog adjuster slider
//  callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_adjuster(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	ioport_field *field = (ioport_field *)arg;
	ioport_field::user_settings settings;

	field->get_user_settings(settings);
	if (newval != SLIDER_NOCHANGE)
	{
		settings.value = newval;
		field->set_user_settings(settings);
	}
	if (str)
		*str = string_format(_("%1$d%%"), settings.value);
	return settings.value;
}


//-------------------------------------------------
//  slider_overclock - CPU overclocker slider
//  callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_overclock(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	device_t *cpu = (device_t *)arg;
	if (newval != SLIDER_NOCHANGE)
		cpu->set_clock_scale((float)newval * 0.001f);
	if (str)
		*str = string_format(_("%1$3.0f%%"), floor(cpu->clock_scale() * 100.0 + 0.5));
	return floor(cpu->clock_scale() * 1000.0 + 0.5);
}


//-------------------------------------------------
//  slider_refresh - refresh rate slider callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_refresh(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	screen_device *screen = reinterpret_cast<screen_device *>(arg);
	double defrefresh = ATTOSECONDS_TO_HZ(screen->refresh_attoseconds());
	double refresh;

	if (newval != SLIDER_NOCHANGE)
	{
		int width = screen->width();
		int height = screen->height();
		const rectangle &visarea = screen->visible_area();
		screen->configure(width, height, visarea, HZ_TO_ATTOSECONDS(defrefresh + (double)newval * 0.001));
	}
	if (str)
		*str = string_format(_("%1$.3ffps"), ATTOSECONDS_TO_HZ(machine.first_screen()->frame_period().attoseconds()));
	refresh = ATTOSECONDS_TO_HZ(machine.first_screen()->frame_period().attoseconds());
	return floor((refresh - defrefresh) * 1000.0 + 0.5);
}


//-------------------------------------------------
//  slider_brightness - screen brightness slider
//  callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_brightness(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	screen_device *screen = reinterpret_cast<screen_device *>(arg);
	render_container::user_settings settings;

	screen->container().get_user_settings(settings);
	if (newval != SLIDER_NOCHANGE)
	{
		settings.m_brightness = (float)newval * 0.001f;
		screen->container().set_user_settings(settings);
	}
	if (str)
		*str = string_format(_("%1$.3f"), settings.m_brightness);
	return floor(settings.m_brightness * 1000.0f + 0.5f);
}


//-------------------------------------------------
//  slider_contrast - screen contrast slider
//  callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_contrast(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	screen_device *screen = reinterpret_cast<screen_device *>(arg);
	render_container::user_settings settings;

	screen->container().get_user_settings(settings);
	if (newval != SLIDER_NOCHANGE)
	{
		settings.m_contrast = (float)newval * 0.001f;
		screen->container().set_user_settings(settings);
	}
	if (str)
		*str = string_format(_("%1$.3f"), settings.m_contrast);
	return floor(settings.m_contrast * 1000.0f + 0.5f);
}


//-------------------------------------------------
//  slider_gamma - screen gamma slider callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_gamma(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	screen_device *screen = reinterpret_cast<screen_device *>(arg);
	render_container::user_settings settings;

	screen->container().get_user_settings(settings);
	if (newval != SLIDER_NOCHANGE)
	{
		settings.m_gamma = (float)newval * 0.001f;
		screen->container().set_user_settings(settings);
	}
	if (str)
		*str = string_format(_("%1$.3f"), settings.m_gamma);
	return floor(settings.m_gamma * 1000.0f + 0.5f);
}


//-------------------------------------------------
//  slider_xscale - screen horizontal scale slider
//  callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_xscale(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	screen_device *screen = reinterpret_cast<screen_device *>(arg);
	render_container::user_settings settings;

	screen->container().get_user_settings(settings);
	if (newval != SLIDER_NOCHANGE)
	{
		settings.m_xscale = (float)newval * 0.001f;
		screen->container().set_user_settings(settings);
	}
	if (str)
		*str = string_format(_("%1$.3f"), settings.m_xscale);
	return floor(settings.m_xscale * 1000.0f + 0.5f);
}


//-------------------------------------------------
//  slider_yscale - screen vertical scale slider
//  callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_yscale(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	screen_device *screen = reinterpret_cast<screen_device *>(arg);
	render_container::user_settings settings;

	screen->container().get_user_settings(settings);
	if (newval != SLIDER_NOCHANGE)
	{
		settings.m_yscale = (float)newval * 0.001f;
		screen->container().set_user_settings(settings);
	}
	if (str)
		*str = string_format(_("%1$.3f"), settings.m_yscale);
	return floor(settings.m_yscale * 1000.0f + 0.5f);
}


//-------------------------------------------------
//  slider_xoffset - screen horizontal position
//  slider callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_xoffset(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	screen_device *screen = reinterpret_cast<screen_device *>(arg);
	render_container::user_settings settings;

	screen->container().get_user_settings(settings);
	if (newval != SLIDER_NOCHANGE)
	{
		settings.m_xoffset = (float)newval * 0.001f;
		screen->container().set_user_settings(settings);
	}
	if (str)
		*str = string_format(_("%1$.3f"), settings.m_xoffset);
	return floor(settings.m_xoffset * 1000.0f + 0.5f);
}


//-------------------------------------------------
//  slider_yoffset - screen vertical position
//  slider callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_yoffset(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	screen_device *screen = reinterpret_cast<screen_device *>(arg);
	render_container::user_settings settings;

	screen->container().get_user_settings(settings);
	if (newval != SLIDER_NOCHANGE)
	{
		settings.m_yoffset = (float)newval * 0.001f;
		screen->container().set_user_settings(settings);
	}
	if (str)
		*str = string_format(_("%1$.3f"), settings.m_yoffset);
	return floor(settings.m_yoffset * 1000.0f + 0.5f);
}


//-------------------------------------------------
//  slider_overxscale - screen horizontal scale slider
//  callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_overxscale(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	laserdisc_device *laserdisc = (laserdisc_device *)arg;
	laserdisc_overlay_config settings;

	laserdisc->get_overlay_config(settings);
	if (newval != SLIDER_NOCHANGE)
	{
		settings.m_overscalex = (float)newval * 0.001f;
		laserdisc->set_overlay_config(settings);
	}
	if (str)
		*str = string_format(_("%1$.3f"), settings.m_overscalex);
	return floor(settings.m_overscalex * 1000.0f + 0.5f);
}


//-------------------------------------------------
//  slider_overyscale - screen vertical scale slider
//  callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_overyscale(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	laserdisc_device *laserdisc = (laserdisc_device *)arg;
	laserdisc_overlay_config settings;

	laserdisc->get_overlay_config(settings);
	if (newval != SLIDER_NOCHANGE)
	{
		settings.m_overscaley = (float)newval * 0.001f;
		laserdisc->set_overlay_config(settings);
	}
	if (str)
		*str = string_format(_("%1$.3f"), settings.m_overscaley);
	return floor(settings.m_overscaley * 1000.0f + 0.5f);
}


//-------------------------------------------------
//  slider_overxoffset - screen horizontal position
//  slider callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_overxoffset(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	laserdisc_device *laserdisc = (laserdisc_device *)arg;
	laserdisc_overlay_config settings;

	laserdisc->get_overlay_config(settings);
	if (newval != SLIDER_NOCHANGE)
	{
		settings.m_overposx = (float)newval * 0.001f;
		laserdisc->set_overlay_config(settings);
	}
	if (str)
		*str = string_format(_("%1$.3f"), settings.m_overposx);
	return floor(settings.m_overposx * 1000.0f + 0.5f);
}


//-------------------------------------------------
//  slider_overyoffset - screen vertical position
//  slider callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_overyoffset(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	laserdisc_device *laserdisc = (laserdisc_device *)arg;
	laserdisc_overlay_config settings;

	laserdisc->get_overlay_config(settings);
	if (newval != SLIDER_NOCHANGE)
	{
		settings.m_overposy = (float)newval * 0.001f;
		laserdisc->set_overlay_config(settings);
	}
	if (str)
		*str = string_format(_("%1$.3f"), settings.m_overposy);
	return floor(settings.m_overposy * 1000.0f + 0.5f);
}


//-------------------------------------------------
//  slider_flicker - vector flicker slider
//  callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_flicker(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	if (newval != SLIDER_NOCHANGE)
		vector_options::s_flicker = (float)newval * 0.001f;
	if (str)
		*str = string_format(_("%1$1.2f"), vector_options::s_flicker);
	return floor(vector_options::s_flicker * 1000.0f + 0.5f);
}


//-------------------------------------------------
//  slider_beam_width_min - minimum vector beam width slider
//  callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_beam_width_min(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	if (newval != SLIDER_NOCHANGE)
		vector_options::s_beam_width_min = std::min((float)newval * 0.01f, vector_options::s_beam_width_max);
	if (str != nullptr)
		*str = string_format(_("%1$1.2f"), vector_options::s_beam_width_min);
	return floor(vector_options::s_beam_width_min * 100.0f + 0.5f);
}


//-------------------------------------------------
//  slider_beam_width_max - maximum vector beam width slider
//  callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_beam_width_max(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	if (newval != SLIDER_NOCHANGE)
		vector_options::s_beam_width_max = std::max((float)newval * 0.01f, vector_options::s_beam_width_min);
	if (str != nullptr)
		*str = string_format(_("%1$1.2f"), vector_options::s_beam_width_max);
	return floor(vector_options::s_beam_width_max * 100.0f + 0.5f);
}


//-------------------------------------------------
//  slider_beam_intensity_weight - vector beam intensity weight slider
//  callback
//-------------------------------------------------

int32_t mame_ui_manager::slider_beam_intensity_weight(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	if (newval != SLIDER_NOCHANGE)
		vector_options::s_beam_intensity_weight = (float)newval * 0.001f;
	if (str != nullptr)
		*str = string_format(_("%1$1.2f"), vector_options::s_beam_intensity_weight);
	return floor(vector_options::s_beam_intensity_weight * 1000.0f + 0.5f);
}


//-------------------------------------------------
//  slider_crossscale - crosshair scale slider
//  callback
//-------------------------------------------------

#ifdef MAME_DEBUG
int32_t mame_ui_manager::slider_crossscale(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	ioport_field *field = (ioport_field *)arg;

	if (newval != SLIDER_NOCHANGE)
		field->set_crosshair_scale(float(newval) * 0.001);
	if (str)
		*str = string_format((field->crosshair_axis() == CROSSHAIR_AXIS_X) ? _("Crosshair Scale X %1$1.3f") :  _("Crosshair Scale Y %1$1.3f"), float(newval) * 0.001f);
	return floor(field->crosshair_scale() * 1000.0f + 0.5f);
}
#endif


//-------------------------------------------------
//  slider_crossoffset - crosshair scale slider
//  callback
//-------------------------------------------------

#ifdef MAME_DEBUG
int32_t mame_ui_manager::slider_crossoffset(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	ioport_field *field = (ioport_field *)arg;

	if (newval != SLIDER_NOCHANGE)
		field->set_crosshair_offset(float(newval) * 0.001f);
	if (str)
		*str = string_format((field->crosshair_axis() == CROSSHAIR_AXIS_X) ? _("Crosshair Offset X %1$1.3f") :  _("Crosshair Offset Y %1$1.3f"), float(newval) * 0.001f);
	return field->crosshair_offset();
}
#endif


//-------------------------------------------------
//  wrap_text
//-------------------------------------------------

ui::text_layout mame_ui_manager::create_layout(render_container &container, float width, ui::text_layout::text_justify justify, ui::text_layout::word_wrapping wrap)
{
	// determine scale factors
	float yscale = get_line_height();
	float xscale = yscale * machine().render().ui_aspect(&container);

	// create the layout
	return ui::text_layout(*get_font(), xscale, yscale, width, justify, wrap);
}


//-------------------------------------------------
//  wrap_text
//-------------------------------------------------

int mame_ui_manager::wrap_text(render_container &container, const char *origs, float x, float y, float origwrapwidth, std::vector<int> &xstart, std::vector<int> &xend, float text_size)
{
	// create the layout
	auto layout = create_layout(container, origwrapwidth, ui::text_layout::LEFT, ui::text_layout::WORD);

	// add the text
	layout.add_text(
			origs,
			rgb_t::black(),
			rgb_t::black(),
			text_size);

	// and get the wrapping info
	return layout.get_wrap_info(xstart, xend);
}

//-------------------------------------------------
//  draw_textured_box - add primitives to
//  draw an outlined box with the given
//  textured background and line color
//-------------------------------------------------

void mame_ui_manager::draw_textured_box(render_container &container, float x0, float y0, float x1, float y1, rgb_t backcolor, rgb_t linecolor, render_texture *texture, uint32_t flags)
{
	container.add_quad(x0, y0, x1, y1, backcolor, texture, flags);
	container.add_line(x0, y0, x1, y0, UI_LINE_WIDTH, linecolor, PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA));
	container.add_line(x1, y0, x1, y1, UI_LINE_WIDTH, linecolor, PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA));
	container.add_line(x1, y1, x0, y1, UI_LINE_WIDTH, linecolor, PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA));
	container.add_line(x0, y1, x0, y0, UI_LINE_WIDTH, linecolor, PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA));
}

//-------------------------------------------------
//  decode UI color options
//-------------------------------------------------

rgb_t decode_ui_color(int id, running_machine *machine)
{
	static rgb_t color[ARRAY_LENGTH(s_color_list)];

	if (machine != nullptr) {
		ui_options option;
		for (int x = 0; x < ARRAY_LENGTH(s_color_list); ++x) {
			const char *o_default = option.value(s_color_list[x]);
			const char *s_option = mame_machine_manager::instance()->ui().options().value(s_color_list[x]);
			int len = strlen(s_option);
			if (len != 8)
				color[x] = rgb_t((uint32_t)strtoul(o_default, nullptr, 16));
			else
				color[x] = rgb_t((uint32_t)strtoul(s_option, nullptr, 16));
		}
	}
	return color[id];
}

//-------------------------------------------------
//  get font rows from options
//-------------------------------------------------

int get_font_rows(running_machine *machine)
{
	static int value;

	return ((machine != nullptr) ? value = mame_machine_manager::instance()->ui().options().font_rows() : value);
}

void mame_ui_manager::popup_time_string(int seconds, std::string message)
{
	// extract the text
	messagebox_poptext = message;
	messagebox_backcolor = UI_BACKGROUND_COLOR;

	// set a timer
	m_popup_text_end = osd_ticks() + osd_ticks_per_second() * seconds;
}


/***************************************************************************
    LOADING AND SAVING OPTIONS
***************************************************************************/

//-------------------------------------------------
//  load ui options
//-------------------------------------------------

void mame_ui_manager::load_ui_options()
{
	// parse the file
	// attempt to open the output file
	emu_file file(machine().options().ini_path(), OPEN_FLAG_READ);
	if (file.open("ui.ini") == osd_file::error::NONE)
	{
		try
		{
			options().parse_ini_file((util::core_file&)file, OPTION_PRIORITY_MAME_INI, OPTION_PRIORITY_MAME_INI < OPTION_PRIORITY_DRIVER_INI, true);
		}
		catch (options_exception &)
		{
			osd_printf_error("**Error loading ui.ini**\n");
		}
	}
}

//-------------------------------------------------
//  save ui options
//-------------------------------------------------

void mame_ui_manager::save_ui_options()
{
	// attempt to open the output file
	emu_file file(machine().options().ini_path(), OPEN_FLAG_WRITE | OPEN_FLAG_CREATE | OPEN_FLAG_CREATE_PATHS);
	if (file.open("ui.ini") == osd_file::error::NONE)
	{
		// generate the updated INI
		std::string initext = options().output_ini();
		file.puts(initext.c_str());
		file.close();
	}
	else
		machine().popmessage(_("**Error saving ui.ini**"));
}

//-------------------------------------------------
//  save main option
//-------------------------------------------------

void mame_ui_manager::save_main_option()
{
	// parse the file
	std::string error;
	emu_options options(emu_options::option_support::GENERAL_ONLY); // This way we make sure that all OSD parts are in

	options.copy_from(machine().options());

	// attempt to open the main ini file
	{
		emu_file file(machine().options().ini_path(), OPEN_FLAG_READ);
		if (file.open(emulator_info::get_configname(), ".ini") == osd_file::error::NONE)
		{
			try
			{
				options.parse_ini_file((util::core_file&)file, OPTION_PRIORITY_MAME_INI, OPTION_PRIORITY_MAME_INI < OPTION_PRIORITY_DRIVER_INI, true);
			}
			catch(options_error_exception &)
			{
				osd_printf_error("**Error loading %s.ini**\n", emulator_info::get_configname());
				return;
			}
			catch (options_exception &)
			{
				// ignore other exceptions related to options
			}
		}
	}

	for (const auto &f_entry : machine().options().entries())
	{
		const char *value = f_entry->value();
		if (value && options.exists(f_entry->name()) && strcmp(value, options.value(f_entry->name().c_str())))
		{
			options.set_value(f_entry->name(), *f_entry->value(), OPTION_PRIORITY_CMDLINE);
		}
	}

	// attempt to open the output file
	{
		emu_file file(machine().options().ini_path(), OPEN_FLAG_WRITE | OPEN_FLAG_CREATE | OPEN_FLAG_CREATE_PATHS);
		if (file.open(emulator_info::get_configname(), ".ini") == osd_file::error::NONE)
		{
			// generate the updated INI
			std::string initext = options.output_ini();
			file.puts(initext.c_str());
			file.close();
		}
		else {
			machine().popmessage(_("**Error saving %s.ini**"), emulator_info::get_configname());
			return;
		}
	}
	popup_time(3, "%s", _("\n    Configuration saved    \n\n"));
}

void mame_ui_manager::menu_reset()
{
	ui::menu::stack_reset(machine());
}
