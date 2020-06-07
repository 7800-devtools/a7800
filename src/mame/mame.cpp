// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    mame.c

    Specific (per target) constants

****************************************************************************/

#include "emu.h"

#define APPNAME                 "A7800"
#define APPNAME_LOWER           "a7800"
#define CONFIGNAME              "a7800"
#define COPYRIGHT               "Copyright Mike Saarna\nMAME Team\nand Team A7800\nhttp://7800.8bitdev.org/index.php/A7800_Emulator"
#define COPYRIGHT_INFO          "Copyright Mike Saarna, MAME Team, and Team A7800"

const char * emulator_info::get_appname() { return APPNAME;}
const char * emulator_info::get_appname_lower() { return APPNAME_LOWER;}
const char * emulator_info::get_configname() { return CONFIGNAME;}
const char * emulator_info::get_copyright() { return COPYRIGHT;}
const char * emulator_info::get_copyright_info() { return COPYRIGHT_INFO;}
