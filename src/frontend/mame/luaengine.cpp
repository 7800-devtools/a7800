// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic,Luca Bruno
/***************************************************************************

    luaengine.c

    Controls execution of the core MAME system.

***************************************************************************/

#include <thread>
#include <lua.hpp>
#include "emu.h"
#include "mame.h"
#include "drivenum.h"
#include "emuopts.h"
#include "ui/ui.h"
#include "ui/pluginopt.h"
#include "luaengine.h"
#include "natkeyboard.h"
#include "uiinput.h"
#include "pluginopts.h"

#ifdef __clang__
#pragma clang diagnostic ignored "-Wshift-count-overflow"
#endif
#if defined(_MSC_VER)
#pragma warning(disable:4503)
#endif

//**************************************************************************
//  LUA ENGINE
//**************************************************************************

extern "C" {
	int luaopen_zlib(lua_State *L);
	int luaopen_lfs(lua_State *L);
	int luaopen_linenoise(lua_State *L);
	int luaopen_lsqlite3(lua_State *L);
}

namespace sol
{
	class buffer
	{
	public:
		// sol does lua_settop(0), save userdata buffer in registry if necessary
		buffer(int size, lua_State *L)
		{
			ptr = luaL_buffinitsize(L, &buff, size);
			len = size;
			if(buff.b != buff.initb)
			{
				lua_pushvalue(L, -1);
				lua_setfield(L, LUA_REGISTRYINDEX, "sol::buffer_temp");
			}
		}
		~buffer()
		{
			lua_State *L = buff.L;
			if(lua_getfield(L, LUA_REGISTRYINDEX, "sol::buffer_temp") != LUA_TNIL)
			{
				lua_pushnil(L);
				lua_setfield(L, LUA_REGISTRYINDEX, "sol::buffer_temp");
			}
			else
				lua_pop(L, -1);

			luaL_pushresultsize(&buff, len);
		}
		void set_len(int size) { len = size; }
		int get_len() { return len; }
		char *get_ptr() { return ptr; }
	private:
		luaL_Buffer buff;
		int len;
		char *ptr;
	};
	template<>
	struct is_container<core_options> : std::false_type {}; // don't convert core_optons to a table directly
	namespace stack
	{
		template <>
		struct pusher<osd_file::error>
		{
			static int push(lua_State *L, osd_file::error error)
			{
				const char *strerror;
				switch(error)
				{
					case osd_file::error::NONE:
						return stack::push(L, sol::nil);
					case osd_file::error::FAILURE:
						strerror = "failure";
						break;
					case osd_file::error::OUT_OF_MEMORY:
						strerror = "out_of_memory";
						break;
					case osd_file::error::NOT_FOUND:
						strerror = "not_found";
						break;
					case osd_file::error::ACCESS_DENIED:
						strerror = "access_denied";
						break;
					case osd_file::error::ALREADY_OPEN:
						strerror = "already_open";
						break;
					case osd_file::error::TOO_MANY_FILES:
						strerror = "too_many_files";
						break;
					case osd_file::error::INVALID_DATA:
						strerror = "invalid_data";
						break;
					case osd_file::error::INVALID_ACCESS:
						strerror = "invalid_access";
						break;
					default:
						strerror = "unknown_error";
						break;
				}
				return stack::push(L, strerror);
			}
		};
		template <>
		struct checker<sol::buffer *>
		{
			template <typename Handler>
			static bool check (lua_State* L, int index, Handler&& handler, record& tracking)
			{
				return stack::check<int>(L, index, handler);
			}
		};
		template <>
		struct getter<sol::buffer *>
		{
			static sol::buffer *get(lua_State* L, int index, record& tracking)
			{
				return new sol::buffer(stack::get<int>(L, index), L);
			}
		};
		template <>
		struct checker<input_item_class>
		{
			template <typename Handler>
			static bool check (lua_State* L, int index, Handler&& handler, record& tracking)
			{
				return stack::check<const std::string &>(L, index, handler);
			}
		};
		template <>
		struct getter<input_item_class>
		{
			static input_item_class get(lua_State* L, int index, record& tracking)
			{
				const std::string item_class =  stack::get<const std::string &>(L, index);
				if(item_class == "switch")
					return ITEM_CLASS_SWITCH;
				else if(item_class == "absolute" || item_class == "abs")
					return ITEM_CLASS_ABSOLUTE;
				else if(item_class == "relative" || item_class == "rel")
					return ITEM_CLASS_RELATIVE;
				else if(item_class == "maximum" || item_class == "max")
					return ITEM_CLASS_MAXIMUM;
				else
					return ITEM_CLASS_INVALID;
			}
		};
		template <>
		struct pusher<sol::buffer *>
		{
			static int push(lua_State* L, sol::buffer *buff)
			{
				delete buff;
				return 1;
			}
		};
		template <>
		struct pusher<map_handler_type>
		{
			static int push(lua_State *L, map_handler_type type)
			{
				const char *typestr;
				switch(type)
				{
					case AMH_NONE:
						typestr = "none";
						break;
					case AMH_RAM:
						typestr = "ram";
						break;
					case AMH_ROM:
						typestr = "rom";
						break;
					case AMH_NOP:
						typestr = "nop";
						break;
					case AMH_UNMAP:
						typestr = "unmap";
						break;
					case AMH_DEVICE_DELEGATE:
						typestr = "delegate";
						break;
					case AMH_PORT:
						typestr = "port";
						break;
					case AMH_BANK:
						typestr = "bank";
						break;
					case AMH_DEVICE_SUBMAP:
						typestr = "submap";
						break;
					default:
						typestr = "unknown";
						break;
				}
				return stack::push(L, typestr);
			}
		};
	}
}

//-------------------------------------------------
//  mem_read - templated memory readers for <sign>,<size>
//  -> manager:machine().devices[":maincpu"].spaces["program"]:read_i8(0xC000)
//-------------------------------------------------

template <typename T>
T lua_engine::addr_space::mem_read(offs_t address, sol::object shift)
{
	T mem_content = 0;
	if(!shift.as<bool>())
		address = space.address_to_byte(address);
	switch(sizeof(mem_content) * 8) {
		case 8:
			mem_content = space.read_byte(address);
			break;
		case 16:
			if (WORD_ALIGNED(address)) {
				mem_content = space.read_word(address);
			} else {
				mem_content = space.read_word_unaligned(address);
			}
			break;
		case 32:
			if (DWORD_ALIGNED(address)) {
				mem_content = space.read_dword(address);
			} else {
				mem_content = space.read_dword_unaligned(address);
			}
			break;
		case 64:
			if (QWORD_ALIGNED(address)) {
				mem_content = space.read_qword(address);
			} else {
				mem_content = space.read_qword_unaligned(address);
			}
			break;
		default:
			break;
	}

	return mem_content;
}

//-------------------------------------------------
//  mem_write - templated memory writer for <sign>,<size>
//  -> manager:machine().devices[":maincpu"].spaces["program"]:write_u16(0xC000, 0xF00D)
//-------------------------------------------------

template <typename T>
void lua_engine::addr_space::mem_write(offs_t address, T val, sol::object shift)
{
	if(!shift.as<bool>())
		address = space.address_to_byte(address);
	switch(sizeof(val) * 8) {
		case 8:
			space.write_byte(address, val);
			break;
		case 16:
			if (WORD_ALIGNED(address)) {
				space.write_word(address, val);
			} else {
				space.write_word_unaligned(address, val);
			}
			break;
		case 32:
			if (DWORD_ALIGNED(address)) {
				space.write_dword(address, val);
			} else {
				space.write_dword_unaligned(address, val);
			}
			break;
		case 64:
			if (QWORD_ALIGNED(address)) {
				space.write_qword(address, val);
			} else {
				space.write_qword_unaligned(address, val);
			}
			break;
		default:
			break;
	}
}

//-------------------------------------------------
//  log_mem_read - templated logical memory readers for <sign>,<size>
//  -> manager:machine().devices[":maincpu"].spaces["program"]:read_log_i8(0xC000)
//-------------------------------------------------

template <typename T>
T lua_engine::addr_space::log_mem_read(offs_t address)
{
	T mem_content = 0;
	if(!dev.translate(space.spacenum(), TRANSLATE_READ_DEBUG, address))
		return 0;
	address = space.address_to_byte(address);

	switch(sizeof(mem_content) * 8) {
		case 8:
			mem_content = space.read_byte(address);
			break;
		case 16:
			if (WORD_ALIGNED(address)) {
				mem_content = space.read_word(address);
			} else {
				mem_content = space.read_word_unaligned(address);
			}
			break;
		case 32:
			if (DWORD_ALIGNED(address)) {
				mem_content = space.read_dword(address);
			} else {
				mem_content = space.read_dword_unaligned(address);
			}
			break;
		case 64:
			if (QWORD_ALIGNED(address)) {
				mem_content = space.read_qword(address);
			} else {
				mem_content = space.read_qword_unaligned(address);
			}
			break;
		default:
			break;
	}

	return mem_content;
}

//-------------------------------------------------
//  log_mem_write - templated logical memory writer for <sign>,<size>
//  -> manager:machine().devices[":maincpu"].spaces["program"]:write_log_u16(0xC000, 0xF00D)
//-------------------------------------------------

template <typename T>
void lua_engine::addr_space::log_mem_write(offs_t address, T val)
{
	if(!dev.translate(space.spacenum(), TRANSLATE_WRITE_DEBUG, address))
		return;
	address = space.address_to_byte(address);

	switch(sizeof(val) * 8) {
		case 8:
			space.write_byte(address, val);
			break;
		case 16:
			if (WORD_ALIGNED(address)) {
				space.write_word(address, val);
			} else {
				space.write_word_unaligned(address, val);
			}
			break;
		case 32:
			if (DWORD_ALIGNED(address)) {
				space.write_dword(address, val);
			} else {
				space.write_dword_unaligned(address, val);
			}
			break;
		case 64:
			if (QWORD_ALIGNED(address)) {
				space.write_qword(address, val);
			} else {
				space.write_qword_unaligned(address, val);
			}
			break;
		default:
			break;
	}
}

//-------------------------------------------------
//  mem_direct_read - templated direct memory readers for <sign>,<size>
//  -> manager:machine().devices[":maincpu"].spaces["program"]:read_direct_i8(0xC000)
//-------------------------------------------------

template <typename T>
T lua_engine::addr_space::direct_mem_read(offs_t address)
{
	T mem_content = 0;
	offs_t lowmask = space.data_width() / 8 - 1;
	for(int i = 0; i < sizeof(T); i++)
	{
		int addr = space.endianness() == ENDIANNESS_LITTLE ? address + sizeof(T) - 1 - i : address + i;
		uint8_t *base = (uint8_t *)space.get_read_ptr(space.address_to_byte(addr & ~lowmask));
		if(!base)
			continue;
		mem_content <<= 8;
		if(space.endianness() == ENDIANNESS_BIG)
			mem_content |= base[BYTE8_XOR_BE(addr) & lowmask];
		else
			mem_content |= base[BYTE8_XOR_LE(addr) & lowmask];
	}

	return mem_content;
}

//-------------------------------------------------
//  mem_direct_write - templated memory writer for <sign>,<size>
//  -> manager:machine().devices[":maincpu"].spaces["program"]:write_direct_u16(0xC000, 0xF00D)
//-------------------------------------------------

template <typename T>
void lua_engine::addr_space::direct_mem_write(offs_t address, T val)
{
	offs_t lowmask = space.data_width() / 8 - 1;
	for(int i = 0; i < sizeof(T); i++)
	{
		int addr = space.endianness() == ENDIANNESS_BIG ? address + sizeof(T) - 1 - i : address + i;
		uint8_t *base = (uint8_t *)space.get_read_ptr(space.address_to_byte(addr & ~lowmask));
		if(!base)
			continue;
		if(space.endianness() == ENDIANNESS_BIG)
			base[BYTE8_XOR_BE(addr) & lowmask] = val & 0xff;
		else
			base[BYTE8_XOR_LE(addr) & lowmask] = val & 0xff;
		val >>= 8;
	}
}

//-------------------------------------------------
//  region_read - templated region readers for <sign>,<size>
//  -> manager:machine():memory().regions[":maincpu"]:read_i8(0xC000)
//-------------------------------------------------

template <typename T>
T lua_engine::region_read(memory_region &region, offs_t address)
{
	T mem_content = 0;
	offs_t lowmask = region.bytewidth() - 1;
	for(int i = 0; i < sizeof(T); i++)
	{
		int addr = region.endianness() == ENDIANNESS_LITTLE ? address + sizeof(T) - 1 - i : address + i;
		if(addr >= region.bytes())
			continue;
		mem_content <<= 8;
		if(region.endianness() == ENDIANNESS_BIG)
			mem_content |= region.as_u8((BYTE8_XOR_BE(addr) & lowmask) | (addr & ~lowmask));
		else
			mem_content |= region.as_u8((BYTE8_XOR_LE(addr) & lowmask) | (addr & ~lowmask));
	}

	return mem_content;
}

//-------------------------------------------------
//  region_write - templated region writer for <sign>,<size>
//  -> manager:machine():memory().regions[":maincpu"]:write_u16(0xC000, 0xF00D)
//-------------------------------------------------

template <typename T>
void lua_engine::region_write(memory_region &region, offs_t address, T val)
{
	offs_t lowmask = region.bytewidth() - 1;
	for(int i = 0; i < sizeof(T); i++)
	{
		int addr = region.endianness() == ENDIANNESS_BIG ? address + sizeof(T) - 1 - i : address + i;
		if(addr >= region.bytes())
			continue;
		if(region.endianness() == ENDIANNESS_BIG)
			region.base()[(BYTE8_XOR_BE(addr) & lowmask) | (addr & ~lowmask)] = val & 0xff;
		else
			region.base()[(BYTE8_XOR_LE(addr) & lowmask) | (addr & ~lowmask)] = val & 0xff;
		val >>= 8;
	}
}

//-------------------------------------------------
//  share_read - templated share readers for <sign>,<size>
//  -> manager:machine():memory().shares[":maincpu"]:read_i8(0xC000)
//-------------------------------------------------

template <typename T>
T lua_engine::share_read(memory_share &share, offs_t address)
{
	T mem_content = 0;
	offs_t lowmask = share.bytewidth() - 1;
	uint8_t* ptr = (uint8_t*)share.ptr();
	for(int i = 0; i < sizeof(T); i++)
	{
		int addr = share.endianness() == ENDIANNESS_LITTLE ? address + sizeof(T) - 1 - i : address + i;
		if(addr >= share.bytes())
			continue;
		mem_content <<= 8;
		if(share.endianness() == ENDIANNESS_BIG)
			mem_content |= ptr[(BYTE8_XOR_BE(addr) & lowmask) | (addr & ~lowmask)];
		else
			mem_content |= ptr[(BYTE8_XOR_LE(addr) & lowmask) | (addr & ~lowmask)];
	}

	return mem_content;
}

//-------------------------------------------------
//  share_write - templated share writer for <sign>,<size>
//  -> manager:machine():memory().shares[":maincpu"]:write_u16(0xC000, 0xF00D)
//-------------------------------------------------

template <typename T>
void lua_engine::share_write(memory_share &share, offs_t address, T val)
{
	offs_t lowmask = share.bytewidth() - 1;
	uint8_t* ptr = (uint8_t*)share.ptr();
	for(int i = 0; i < sizeof(T); i++)
	{
		int addr = share.endianness() == ENDIANNESS_BIG ? address + sizeof(T) - 1 - i : address + i;
		if(addr >= share.bytes())
			continue;
		if(share.endianness() == ENDIANNESS_BIG)
			ptr[(BYTE8_XOR_BE(addr) & lowmask) | (addr & ~lowmask)] = val & 0xff;
		else
			ptr[(BYTE8_XOR_LE(addr) & lowmask) | (addr & ~lowmask)] = val & 0xff;
		val >>= 8;
	}
}

//-------------------------------------------------
//  lua_engine - constructor
//-------------------------------------------------

lua_engine::lua_engine()
{
	m_machine = nullptr;
	m_lua_state = luaL_newstate();  /* create state */
	m_sol_state = std::make_unique<sol::state_view>(m_lua_state); // create sol view

	luaL_checkversion(m_lua_state);
	lua_gc(m_lua_state, LUA_GCSTOP, 0);  /* stop collector during initialization */
	sol().open_libraries();

	// Get package.preload so we can store builtins in it.
	sol()["package"]["preload"]["zlib"] = &luaopen_zlib;
	sol()["package"]["preload"]["lfs"] = &luaopen_lfs;
	sol()["package"]["preload"]["linenoise"] = &luaopen_linenoise;
	sol()["package"]["preload"]["lsqlite3"] = &luaopen_lsqlite3;

	lua_gc(m_lua_state, LUA_GCRESTART, 0);
}

//-------------------------------------------------
//  ~lua_engine - destructor
//-------------------------------------------------

lua_engine::~lua_engine()
{
	close();
}

sol::object lua_engine::call_plugin(const std::string &name, sol::object in)
{
	std::string field = "cb_" + name;
	sol::object obj = sol().registry()[field];
	if(obj.is<sol::protected_function>())
	{
		auto res = (obj.as<sol::protected_function>())(in);
		if(!res.valid())
		{
			sol::error err = res;
			osd_printf_error("[LUA ERROR] in call_plugin: %s\n", err.what());
		}
		else
			return res.get<sol::object>();
	}
	return sol::make_object(sol(), sol::nil);
}

void lua_engine::menu_populate(const std::string &menu, std::vector<std::tuple<std::string, std::string, std::string>> &menu_list)
{
	std::string field = "menu_pop_" + menu;
	sol::object obj = sol().registry()[field];
	if(obj.is<sol::protected_function>())
	{
		auto res = (obj.as<sol::protected_function>())();
		if(!res.valid())
		{
			sol::error err = res;
			osd_printf_error("[LUA ERROR] in menu_populate: %s\n", err.what());
		}
		else
		{
			sol::table table = res;
			for(auto &entry : table)
			{
				if(entry.second.is<sol::table>())
				{
					sol::table enttable = entry.second.as<sol::table>();
					menu_list.emplace_back(enttable.get<std::string, std::string, std::string>(1, 2, 3));
				}
			}
		}
	}
}

bool lua_engine::menu_callback(const std::string &menu, int index, const std::string &event)
{
	std::string field = "menu_cb_" + menu;
	bool ret = false;
	sol::object obj = sol().registry()[field];
	if(obj.is<sol::protected_function>())
	{
		auto res = (obj.as<sol::protected_function>())(index, event);
		if(!res.valid())
		{
			sol::error err = res;
			osd_printf_error("[LUA ERROR] in menu_callback: %s\n", err.what());
		}
		else
			ret = res;
	}
	return ret;
}

bool lua_engine::execute_function(const char *id)
{
	sol::object functable = sol().registry()[id];
	if(functable.is<sol::table>())
	{
		for(auto &func : functable.as<sol::table>())
		{
			if(func.second.is<sol::protected_function>())
			{
				auto ret = (func.second.as<sol::protected_function>())();
				if(!ret.valid())
				{
					sol::error err = ret;
					osd_printf_error("[LUA ERROR] in execute_function: %s\n", err.what());
				}
			}
		}
		return true;
	}
	return false;
}

void lua_engine::register_function(sol::function func, const char *id)
{
	sol::object functable = sol().registry()[id];
	if(functable.is<sol::table>())
		functable.as<sol::table>().add(func);
	else
		sol().registry().create_named(id, 1, func);
}

void lua_engine::on_machine_prestart()
{
	execute_function("LUA_ON_PRESTART");
}

void lua_engine::on_machine_start()
{
	execute_function("LUA_ON_START");
}

void lua_engine::on_machine_stop()
{
	execute_function("LUA_ON_STOP");
}

void lua_engine::on_machine_pause()
{
	execute_function("LUA_ON_PAUSE");
}

void lua_engine::on_machine_resume()
{
	execute_function("LUA_ON_RESUME");
}

void lua_engine::on_machine_frame()
{
	execute_function("LUA_ON_FRAME");
}

void lua_engine::on_frame_done()
{
	execute_function("LUA_ON_FRAME_DONE");
}

void lua_engine::on_periodic()
{
	execute_function("LUA_ON_PERIODIC");
}

void lua_engine::attach_notifiers()
{
	machine().add_notifier(MACHINE_NOTIFY_RESET, machine_notify_delegate(&lua_engine::on_machine_prestart, this), true);
	machine().add_notifier(MACHINE_NOTIFY_RESET, machine_notify_delegate(&lua_engine::on_machine_start, this));
	machine().add_notifier(MACHINE_NOTIFY_EXIT, machine_notify_delegate(&lua_engine::on_machine_stop, this));
	machine().add_notifier(MACHINE_NOTIFY_PAUSE, machine_notify_delegate(&lua_engine::on_machine_pause, this));
	machine().add_notifier(MACHINE_NOTIFY_RESUME, machine_notify_delegate(&lua_engine::on_machine_resume, this));
	machine().add_notifier(MACHINE_NOTIFY_FRAME, machine_notify_delegate(&lua_engine::on_machine_frame, this));
}

//-------------------------------------------------
//  initialize - initialize lua hookup to emu engine
//-------------------------------------------------

void lua_engine::initialize()
{

/*
 * emu.app_name() - return application name
 * emu.app_version() - return application version
 * emu.gamename() - returns game full name
 * emu.softname() - returns softlist name
 * emu.keypost(keys) - post keys to natural keyboard
 * emu.time() - return emulation time
 * emu.start(driver) - start given driver
 * emu.pause() - pause emulation
 * emu.unpause() - unpause emulation
 * emu.register_prestart(callback) - callback before reset
 * emu.register_start(callback) - callback after reset
 * emu.register_stop(callback) - callback after stopping
 * emu.register_pause(callback) - callback at pause
 * emu.register_resume(callback) - callback at resume
 * emu.register_frame(callback) - callback at end of frame
 * emu.register_frame_done(callback) - callback after frame is drawn to screen (for overlays)
 * emu.register_periodic(callback) - periodic callback while program is running
 * emu.register_menu(event_callback, populate_callback, name) - callbacks for plugin menu
 * emu.print_verbose(str) -- output to stderr at verbose level
 * emu.print_error(str) -- output to stderr at error level
 * emu.print_info(str) -- output to stderr at info level
 * emu.print_debug(str) -- output to stderr at debug level
 * emu.driver_find(driver) -- find and return game_driver for driver
 */
	sol::table emu = sol().create_named_table("emu");
	emu["app_name"] = &emulator_info::get_appname_lower;
	emu["app_version"] = &emulator_info::get_bare_build_version;
	emu["gamename"] = [this](){ return machine().system().type.fullname(); };
	emu["romname"] = [this](){ return machine().basename(); };
	emu["softname"] = [this]() { return machine().options().software_name(); };
	emu["keypost"] = [this](const char *keys){ machine().ioport().natkeyboard().post_utf8(keys); };
	emu["time"] = [this](){ return machine().time().as_double(); };
	emu["start"] = [this](const char *driver) {
			int i = driver_list::find(driver);
			if (i != -1)
			{
				mame_machine_manager::instance()->schedule_new_driver(driver_list::driver(i));
				machine().schedule_hard_reset();
			}
			return 1;
		};
	emu["pause"] = [this](){ return machine().pause(); };
	emu["unpause"] = [this](){ return machine().resume(); };
	emu["register_prestart"] = [this](sol::function func){ register_function(func, "LUA_ON_PRESTART"); };
	emu["register_start"] = [this](sol::function func){ register_function(func, "LUA_ON_START"); };
	emu["register_stop"] = [this](sol::function func){ register_function(func, "LUA_ON_STOP"); };
	emu["register_pause"] = [this](sol::function func){ register_function(func, "LUA_ON_PAUSE"); };
	emu["register_resume"] = [this](sol::function func){ register_function(func, "LUA_ON_RESUME"); };
	emu["register_frame"] = [this](sol::function func){ register_function(func, "LUA_ON_FRAME"); };
	emu["register_frame_done"] = [this](sol::function func){ register_function(func, "LUA_ON_FRAME_DONE"); };
	emu["register_periodic"] = [this](sol::function func){ register_function(func, "LUA_ON_PERIODIC"); };
	emu["register_menu"] = [this](sol::function cb, sol::function pop, const std::string &name) {
			std::string cbfield = "menu_cb_" + name;
			std::string popfield = "menu_pop_" + name;
			sol().registry()[cbfield] = cb;
			sol().registry()[popfield] = pop;
			m_menu.push_back(name);
		};
	emu["show_menu"] = [this](const char *name) {
			mame_ui_manager &mui = mame_machine_manager::instance()->ui();
			render_container &container = machine().render().ui_container();
			ui::menu_plugin::show_menu(mui, container, (char *)name);
		};
	emu["register_callback"] = [this](sol::function cb, const std::string &name) {
			std::string field = "cb_" + name;
			sol().registry()[field] = cb;
		};
	emu["print_verbose"] = [](const char *str) { osd_printf_verbose("%s\n", str); };
	emu["print_error"] = [](const char *str) { osd_printf_error("%s\n", str); };
	emu["print_info"] = [](const char *str) { osd_printf_info("%s\n", str); };
	emu["print_debug"] = [](const char *str) { osd_printf_debug("%s\n", str); };
	emu["driver_find"] = [this](const char *driver) -> sol::object {
			int i = driver_list::find(driver);
			if(i == -1)
				return sol::make_object(sol(), sol::nil);
			return sol::make_object(sol(), driver_list::driver(i));
		};
	emu["wait"] = lua_CFunction([](lua_State *L) {
			lua_engine *engine = mame_machine_manager::instance()->lua();
			luaL_argcheck(L, lua_isnumber(L, 1), 1, "waiting duration expected");
			engine->machine().scheduler().timer_set(attotime::from_double(lua_tonumber(L, 1)), timer_expired_delegate(FUNC(lua_engine::resume), engine), 0, L);
			return lua_yield(L, 0);
		});

/*
 * emu.file([opt] searchpath, flags) - flags can be as in osdcore "OPEN_FLAG_*" or lua style with 'rwc' with addtional c for create *and truncate* (be careful)
 *                                     support zipped files on the searchpath
 * file:open(name) - open first file matching name in searchpath, supports read and write sockets as "socket.127.0.0.1:1234"
 * file:open_next() - open next file matching name in searchpath
 * file:read(len) - only reads len bytes, doen't do lua style formats
 * file:write(data) - write data to file
 * file:seek(offset, whence) - whence is as C "SEEK_*" int
 * file:seek([opt] whence, [opt] offset) - lua style "set"|"cur"|"end", returns cur offset
 * file:size() -
 * file:filename() - name of current file, container name if file is in zip
 * file:fullpath() -
*/

	emu.new_usertype<emu_file>("file", sol::call_constructor, sol::initializers([](emu_file &file, u32 flags) { new (&file) emu_file(flags); },
				[](emu_file &file, const char *path, u32 flags) { new (&file) emu_file(path, flags); },
				[](emu_file &file, const char *mode) {
					int flags = 0;
					for(int i = 0; i < 2; i++) // limit to three chars
					{
						switch(mode[i])
						{
							case 'r':
								flags |= OPEN_FLAG_READ;
								break;
							case 'w':
								flags |= OPEN_FLAG_WRITE;
								break;
							case 'c':
								flags |= OPEN_FLAG_CREATE;
								break;
						}
					}
					new (&file) emu_file(flags);
				},
				[](emu_file &file, const char *path, const char* mode) {
					int flags = 0;
					for(int i = 0; i < 2; i++) // limit to three chars
					{
						switch(mode[i])
						{
							case 'r':
								flags |= OPEN_FLAG_READ;
								break;
							case 'w':
								flags |= OPEN_FLAG_WRITE;
								break;
							case 'c':
								flags |= OPEN_FLAG_CREATE;
								break;
						}
					}
					new (&file) emu_file(path, flags);
				}),
			"read", [](emu_file &file, sol::buffer *buff) { buff->set_len(file.read(buff->get_ptr(), buff->get_len())); return buff; },
			"write", [](emu_file &file, const std::string &data) { return file.write(data.data(), data.size()); },
			"open", static_cast<osd_file::error (emu_file::*)(const std::string &)>(&emu_file::open),
			"open_next", &emu_file::open_next,
			"seek", sol::overload([](emu_file &file) { return file.tell(); },
				[this](emu_file &file, s64 offset, int whence) -> sol::object {
					if(file.seek(offset, whence))
						return sol::make_object(sol(), sol::nil);
					else
						return sol::make_object(sol(), file.tell());
				},
				[this](emu_file &file, const char* whence) -> sol::object {
					int wval = -1;
					const char *seekdirs[] = {"set", "cur", "end"};
					for(int i = 0; i < 3; i++)
					{
						if(!strncmp(whence, seekdirs[i], 3))
						{
							wval = i;
							break;
						}
					}
					if(wval < 0 || wval >= 3)
						return sol::make_object(sol(), sol::nil);
					if(file.seek(0, wval))
						return sol::make_object(sol(), sol::nil);
					return sol::make_object(sol(), file.tell());
				},
				[this](emu_file &file, const char* whence, s64 offset) -> sol::object {
					int wval = -1;
					const char *seekdirs[] = {"set", "cur", "end"};
					for(int i = 0; i < 3; i++)
					{
						if(!strncmp(whence, seekdirs[i], 3))
						{
							wval = i;
							break;
						}
					}
					if(wval < 0 || wval >= 3)
						return sol::make_object(sol(), sol::nil);
					if(file.seek(offset, wval))
						return sol::make_object(sol(), sol::nil);
					return sol::make_object(sol(), file.tell());
				}),
			"size", &emu_file::size,
			"filename", &emu_file::filename,
			"fullpath", &emu_file::fullpath);

/*
 * emu.thread()
 * thread.start(scr) - run scr (string not function) in a seperate thread in a new empty (other then modules) lua context
 * thread.continue(val) - resume thread and pass val to it
 * thread.result() - get thread result as string
 * thread.busy - check if thread is running
 * thread.yield - check if thread is yielded
*/

	emu.new_usertype<context>("thread", sol::call_constructor, sol::constructors<sol::types<>>(),
			"start", [this](context &ctx, const char *scr) {
					std::string script(scr);
					if(ctx.busy)
						return false;
					std::thread th([&ctx, script]() {
							sol::state thstate;
							thstate.open_libraries();
							thstate["package"]["preload"]["zlib"] = &luaopen_zlib;
							thstate["package"]["preload"]["lfs"] = &luaopen_lfs;
							thstate["package"]["preload"]["linenoise"] = &luaopen_linenoise;
							sol::load_result res = thstate.load(script);
							if(res.valid())
							{
								sol::protected_function func = res.get<sol::protected_function>();
								thstate["yield"] = [&ctx, &thstate]() {
										std::mutex m;
										std::unique_lock<std::mutex> lock(m);
										ctx.result = thstate["status"];
										ctx.yield = true;
										ctx.sync.wait(lock);
										ctx.yield = false;
										thstate["status"] = ctx.result;
									};
								auto ret = func();
								if (ret.valid()) {
									const char *tmp = ret.get<const char *>();
									if (tmp != nullptr)
										ctx.result = tmp;
									else
										exit(0);
								}
							}
							ctx.busy = false;
						});
					ctx.busy = true;
					ctx.yield = false;
					th.detach();
					return true;
				},
			"continue", [this](context &ctx, const char *val) {
					if(!ctx.yield)
						return;
					ctx.result = val;
					ctx.sync.notify_all();
				},
			"result", sol::property([this](context &ctx) -> std::string {
					if(ctx.busy && !ctx.yield)
						return "";
					return ctx.result;
				}),
			"busy", sol::readonly(&context::busy),
			"yield", sol::readonly(&context::yield));

	emu.new_usertype<save_item>("item", sol::call_constructor, sol::initializers([this](save_item &item, int index) {
					if(!machine().save().indexed_item(index, item.base, item.size, item.count))
					{
						item.base = nullptr;
						item.size = 0;
						item.count= 0;
					}
				}),
			"size", sol::readonly(&save_item::size),
			"count", sol::readonly(&save_item::count),
			"read", [this](save_item &item, int offset) -> sol::object {
					uint64_t ret = 0;
					if(!item.base || (offset > item.count))
						return sol::make_object(sol(), sol::nil);
					switch(item.size)
					{
						case 1:
						default:
							ret = ((uint8_t *)item.base)[offset];
							break;
						case 2:
							ret = ((uint16_t *)item.base)[offset];
							break;
						case 4:
							ret = ((uint32_t *)item.base)[offset];
							break;
						case 8:
							ret = ((uint64_t *)item.base)[offset];
							break;
					}
					return sol::make_object(sol(), ret);
				},
			"read_block", [this](save_item &item, int offset, sol::buffer *buff) {
					if(!item.base || ((offset + buff->get_len()) > (item.size * item.count)))
						buff->set_len(0);
					else
						memcpy(buff->get_ptr(), item.base, buff->get_len());
					return buff;
				},
			"write", [](save_item &item, int offset, uint64_t value) {
					if(!item.base || (offset > item.count))
						return;
					switch(item.size)
					{
						case 1:
						default:
							((uint8_t *)item.base)[offset] = (uint8_t)value;
							break;
						case 2:
							((uint16_t *)item.base)[offset] = (uint16_t)value;
							break;
						case 4:
							((uint32_t *)item.base)[offset] = (uint32_t)value;
							break;
						case 8:
							((uint64_t *)item.base)[offset] = (uint64_t)value;
							break;
					}
				});

/* manager:options()
 * machine:options()
 * machine:ui():options()
 * options:help() - get help for options
 * options:command(command) - return output for command
 * options.entries[] - get table of option entries
 */

	sol().registry().new_usertype<core_options>("core_options", "new", sol::no_constructor,
			"help", &core_options::output_help,
			"command", &core_options::command,
			"entries", sol::property([this](core_options &options) {
				sol::table table = sol().create_table();
				int unadorned_index = 0;
				for (auto &curentry : options.entries())
				{
					const char *name = curentry->names().size() > 0
						? curentry->name().c_str()
						: nullptr;
					bool is_unadorned = false;
					// check if it's unadorned
					if (name && strlen(name) && !strcmp(name, options.unadorned(unadorned_index)))
					{
						unadorned_index++;
						is_unadorned = true;
					}
					if (curentry->type() != core_options::option_type::HEADER && curentry->type() != core_options::option_type::COMMAND && !is_unadorned)
						table[name] = &*curentry;
				}
				return table;
			}));

/* options.entries[entry_name]
 * entry.value() - get value of entry
 * entry.value(val) - set entry to val
 * entry.description() - get info about entry
 * entry.default_value() - get default for entry
 * entry.minimum() - get min value for entry
 * entry.maximum() - get max value for entry
 * entry.has_range() - are min and max valid for entry
 */

	sol().registry().new_usertype<core_options::entry>("core_options_entry", "new", sol::no_constructor,
			"value", sol::overload([this](core_options::entry &e, bool val) {
					if(e.type() != OPTION_BOOLEAN)
						luaL_error(m_lua_state, "Cannot set option to wrong type");
					else
						e.set_value(val ? "1" : "0", OPTION_PRIORITY_CMDLINE);
				},
				[this](core_options::entry &e, float val) {
					if(e.type() != OPTION_FLOAT)
						luaL_error(m_lua_state, "Cannot set option to wrong type");
					else
						e.set_value(string_format("%f", val).c_str(), OPTION_PRIORITY_CMDLINE);
				},
				[this](core_options::entry &e, int val) {
					if(e.type() != OPTION_INTEGER)
						luaL_error(m_lua_state, "Cannot set option to wrong type");
					else
						e.set_value(string_format("%d", val).c_str(), OPTION_PRIORITY_CMDLINE);
				},
				[this](core_options::entry &e, const char *val) {
					if(e.type() != OPTION_STRING)
						luaL_error(m_lua_state, "Cannot set option to wrong type");
					else
						e.set_value(val, OPTION_PRIORITY_CMDLINE);
				},
				[this](core_options::entry &e) -> sol::object {
					if (e.type() == core_options::option_type::INVALID)
						return sol::make_object(sol(), sol::nil);
					switch(e.type())
					{
						case core_options::option_type::BOOLEAN:
							return sol::make_object(sol(), atoi(e.value()) != 0);
						case core_options::option_type::INTEGER:
							return sol::make_object(sol(), atoi(e.value()));
						case core_options::option_type::FLOAT:
							return sol::make_object(sol(), atof(e.value()));
						default:
							return sol::make_object(sol(), e.value());
					}
				}),
			"description", &core_options::entry::description,
			"default_value", &core_options::entry::default_value,
			"minimum", &core_options::entry::minimum,
			"maximum", &core_options::entry::maximum,
			"has_range", &core_options::entry::has_range);

/* manager:machine()
 * machine:exit() - close program
 * machine:hard_reset() - hard reset emulation
 * machine:soft_reset() - soft reset emulation
 * machine:save(filename) - save state to filename
 * machine:load(filename) - load state from filename
 * machine:system() - get game_driver for running driver
 * machine:video() - get video_manager
 * machine:render() - get render_manager
 * machine:ioport() - get ioport_manager
 * machine:parameters() - get parameter_manager
 * machine:options() - get machine core_options
 * machine:output() - get output_manager
 * machine:input() - get input_manager
 * machine:uiinput() - get ui_input_manager
 * machine.paused - get paused state
 * machine.devices - get device table
 * machine.screens - get screens table
 * machine.images - get available image devices
 * machine:popmessage(str) - print str as popup
 * machine:logerror(str) - print str to log
 */

	sol().registry().new_usertype<running_machine> ("machine", "new", sol::no_constructor,
			"exit", &running_machine::schedule_exit,
			"hard_reset", &running_machine::schedule_hard_reset,
			"soft_reset", &running_machine::schedule_soft_reset,
			"save", &running_machine::schedule_save,
			"load", &running_machine::schedule_load,
			"system", &running_machine::system,
			"video", &running_machine::video,
			"render", &running_machine::render,
			"ioport", &running_machine::ioport,
			"parameters", &running_machine::parameters,
			"memory", &running_machine::memory,
			"options", [](running_machine &m) { return static_cast<core_options *>(&m.options()); },
			"outputs", &running_machine::output,
			"input", &running_machine::input,
			"uiinput", &running_machine::ui_input,
			"paused", sol::property(&running_machine::paused),
			"devices", sol::property([this](running_machine &m) {
					std::function<void(device_t &, sol::table)> tree;
					sol::table table = sol().create_table();
					tree = [&tree](device_t &root, sol::table table) {
						for(device_t &dev : root.subdevices())
						{
							if(dev.configured() && dev.started())
							{
								table[dev.tag()] = &dev;
								tree(dev, table);
							}
						}
					};
					tree(m.root_device(), table);
					return table;
				}),
			"screens", sol::property([this](running_machine &r) {
					sol::table table = sol().create_table();
					for(device_t *dev = r.first_screen(); dev != nullptr; dev = dev->next())
					{
						screen_device *sc = dynamic_cast<screen_device *>(dev);
						if (sc && sc->configured() && sc->started() && sc->type())
							table[sc->tag()] = sc;
					}
					return table;
				}),
			"images", sol::property([this](running_machine &r) {
					sol::table image_table = sol().create_table();
					for(device_image_interface &image : image_interface_iterator(r.root_device()))
					{
						image_table[image.brief_instance_name()] = &image;
						image_table[image.instance_name()] = &image;
					}
					return image_table;
				}),
			"popmessage", sol::overload([](running_machine &m, const char *str) { m.popmessage("%s", str); },
					[](running_machine &m) { m.popmessage(); }),
			"logerror", [](running_machine &m, const char *str) { m.logerror("[luaengine] %s\n", str); } );

/* game_driver - this should be self explanatory
 */

	sol().registry().new_usertype<game_driver>("game_driver", "new", sol::no_constructor,
			"source_file", sol::property([] (game_driver const &driver) { return &driver.type.source()[0]; }),
			"parent", sol::readonly(&game_driver::parent),
			"name", sol::property([] (game_driver const &driver) { return &driver.name[0]; }),
			"description", sol::property([] (game_driver const &driver) { return &driver.type.fullname()[0]; }),
			"year", sol::readonly(&game_driver::year),
			"manufacturer", sol::readonly(&game_driver::manufacturer),
			"compatible_with", sol::readonly(&game_driver::compatible_with),
			"default_layout", sol::readonly(&game_driver::default_layout));

/* machine.devices[device_tag]
 * device:name() - device long name
 * device:shortname() - device short name
 * device:tag() - device tree tag
 * device:owner() - device parent tag
 * device.spaces[] - device address spaces table
 * device.state[] - device state entries table
 * device.items[] - device save state items table
 */

	sol().registry().new_usertype<device_t>("device", "new", sol::no_constructor,
			"name", &device_t::name,
			"shortname", &device_t::shortname,
			"tag", &device_t::tag,
			"owner", &device_t::owner,
			"spaces", sol::property([this](device_t &dev) {
					device_memory_interface *memdev = dynamic_cast<device_memory_interface *>(&dev);
					sol::table sp_table = sol().create_table();
					if(!memdev)
						return sp_table;
					for(int sp = 0; sp < memdev->max_space_count(); ++sp)
					{
						if(memdev->has_space(sp))
							sp_table[memdev->space(sp).name()] = addr_space(memdev->space(sp), *memdev);
					}
					return sp_table;
				}),
			"state", sol::property([this](device_t &dev) {
					sol::table st_table = sol().create_table();
					if(!dynamic_cast<device_state_interface *>(&dev))
						return st_table;
					// XXX: refrain from exporting non-visible entries?
					for(auto &s : dev.state().state_entries())
						st_table[s->symbol()] = s.get();
					return st_table;
				}),
			"items", sol::property([this](device_t &dev) {
					sol::table table = sol().create_table();
					std::string tag = dev.tag();
					// 10000 is enough?
					for(int i = 0; i < 10000; i++)
					{
						std::string name;
						const char *item;
						unsigned int size, count;
						void *base;
						item = dev.machine().save().indexed_item(i, base, size, count);
						if(!item)
							break;
						name = &(strchr(item, '/')[1]);
						if(name.substr(0, name.find("/")) == tag)
						{
							name = name.substr(name.find("/") + 1, std::string::npos);
							table[name] = i;
						}
					}
					return table;
				}));

	sol().registry().new_usertype<addr_space>("addr_space", sol::call_constructor, sol::constructors<sol::types<address_space &, device_memory_interface &>>(),
			"read_i8", &addr_space::mem_read<int8_t>,
			"read_u8", &addr_space::mem_read<uint8_t>,
			"read_i16", &addr_space::mem_read<int16_t>,
			"read_u16", &addr_space::mem_read<uint16_t>,
			"read_i32", &addr_space::mem_read<int32_t>,
			"read_u32", &addr_space::mem_read<uint32_t>,
			"read_i64", &addr_space::mem_read<int64_t>,
			"read_u64", &addr_space::mem_read<uint64_t>,
			"write_i8", &addr_space::mem_write<int8_t>,
			"write_u8", &addr_space::mem_write<uint8_t>,
			"write_i16", &addr_space::mem_write<int16_t>,
			"write_u16", &addr_space::mem_write<uint16_t>,
			"write_i32", &addr_space::mem_write<int32_t>,
			"write_u32", &addr_space::mem_write<uint32_t>,
			"write_i64", &addr_space::mem_write<int64_t>,
			"write_u64", &addr_space::mem_write<uint64_t>,
			"read_log_i8", &addr_space::log_mem_read<int8_t>,
			"read_log_u8", &addr_space::log_mem_read<uint8_t>,
			"read_log_i16", &addr_space::log_mem_read<int16_t>,
			"read_log_u16", &addr_space::log_mem_read<uint16_t>,
			"read_log_i32", &addr_space::log_mem_read<int32_t>,
			"read_log_u32", &addr_space::log_mem_read<uint32_t>,
			"read_log_i64", &addr_space::log_mem_read<int64_t>,
			"read_log_u64", &addr_space::log_mem_read<uint64_t>,
			"write_log_i8", &addr_space::log_mem_write<int8_t>,
			"write_log_u8", &addr_space::log_mem_write<uint8_t>,
			"write_log_i16", &addr_space::log_mem_write<int16_t>,
			"write_log_u16", &addr_space::log_mem_write<uint16_t>,
			"write_log_i32", &addr_space::log_mem_write<int32_t>,
			"write_log_u32", &addr_space::log_mem_write<uint32_t>,
			"write_log_i64", &addr_space::log_mem_write<int64_t>,
			"write_log_u64", &addr_space::log_mem_write<uint64_t>,
			"read_direct_i8", &addr_space::direct_mem_read<int8_t>,
			"read_direct_u8", &addr_space::direct_mem_read<uint8_t>,
			"read_direct_i16", &addr_space::direct_mem_read<int16_t>,
			"read_direct_u16", &addr_space::direct_mem_read<uint16_t>,
			"read_direct_i32", &addr_space::direct_mem_read<int32_t>,
			"read_direct_u32", &addr_space::direct_mem_read<uint32_t>,
			"read_direct_i64", &addr_space::direct_mem_read<int64_t>,
			"read_direct_u64", &addr_space::direct_mem_read<uint64_t>,
			"write_direct_i8", &addr_space::direct_mem_write<int8_t>,
			"write_direct_u8", &addr_space::direct_mem_write<uint8_t>,
			"write_direct_i16", &addr_space::direct_mem_write<int16_t>,
			"write_direct_u16", &addr_space::direct_mem_write<uint16_t>,
			"write_direct_i32", &addr_space::direct_mem_write<int32_t>,
			"write_direct_u32", &addr_space::direct_mem_write<uint32_t>,
			"write_direct_i64", &addr_space::direct_mem_write<int64_t>,
			"write_direct_u64", &addr_space::direct_mem_write<uint64_t>,
			"name", sol::property(&addr_space::name),
			"map", sol::property([this](addr_space &sp) {
					address_space &space = sp.space;
					sol::table map = sol().create_table();
					for (address_map_entry &entry : space.map()->m_entrylist)
					{
						sol::table mapentry = sol().create_table();
						mapentry["offset"] = space.address_to_byte(entry.m_addrstart) & space.bytemask();
						mapentry["endoff"] = space.address_to_byte(entry.m_addrend) & space.bytemask();
						mapentry["readtype"] = entry.m_read.m_type;
						mapentry["writetype"] = entry.m_write.m_type;
						map.add(mapentry);
					}
					return map;
				}));

/* machine:ioport()
 * ioport:count_players() - get count of player controllers
 * ioport.ports[] - ioports table
 */

	sol().registry().new_usertype<ioport_manager>("ioport", "new", sol::no_constructor,
			"count_players", &ioport_manager::count_players,
			"ports", sol::property([this](ioport_manager &im) {
					sol::table port_table = sol().create_table();
					for (auto &port : im.ports())
						port_table[port.second->tag()] = port.second.get();
					return port_table;
				}));

/* ioport.ports[port_tag]
 * port:tag() - get port tag
 * port:active() - get port status
 * port:live() - get port ioport_port_live (TODO: not usable from lua as of now)
 * port:read() - get port value
 * port:write(val, mask) - set port to value & mask (output fields only, for other fields use field:set_value(val))
 * port:field(mask) - get ioport_field for port and mask
 * port.field[] - get ioport_field table
 */

	sol().registry().new_usertype<ioport_port>("ioport_port", "new", sol::no_constructor,
			"tag", &ioport_port::tag,
			"active", &ioport_port::active,
			"live", &ioport_port::live,
			"read", &ioport_port::read,
			"write", &ioport_port::write,
			"field", &ioport_port::field,
			"fields", sol::property([this](ioport_port &p){
					sol::table f_table = sol().create_table();
					for(ioport_field &field : p.fields())
					{
						if (field.type_class() != INPUT_CLASS_INTERNAL)
							f_table[field.name()] = &field;
					}
					return f_table;
				}));

/* port.field[field_tag] - this should be self explanatory
 */

	sol().registry().new_usertype<ioport_field>("ioport_field", "new", sol::no_constructor,
			"set_value", &ioport_field::set_value,
			"device", sol::property(&ioport_field::device),
			"name", sol::property(&ioport_field::name),
			"player", sol::property(&ioport_field::player, &ioport_field::set_player),
			"mask", sol::property(&ioport_field::mask),
			"defvalue", sol::property(&ioport_field::defvalue),
			"sensitivity", sol::property(&ioport_field::sensitivity),
			"way", sol::property(&ioport_field::way),
			"is_analog", sol::property(&ioport_field::is_analog),
			"is_digital_joystick", sol::property(&ioport_field::is_digital_joystick),
			"enabled", sol::property(&ioport_field::enabled),
			"optional", sol::property(&ioport_field::optional),
			"cocktail", sol::property(&ioport_field::cocktail),
			"toggle", sol::property(&ioport_field::toggle),
			"rotated", sol::property(&ioport_field::rotated),
			"analog_reverse", sol::property(&ioport_field::analog_reverse),
			"analog_reset", sol::property(&ioport_field::analog_reset),
			"analog_wraps", sol::property(&ioport_field::analog_wraps),
			"analog_invert", sol::property(&ioport_field::analog_invert),
			"impulse", sol::property(&ioport_field::impulse),
			"type", sol::property(&ioport_field::type),
			"crosshair_scale", sol::property(&ioport_field::crosshair_scale, &ioport_field::set_crosshair_scale),
			"crosshair_offset", sol::property(&ioport_field::crosshair_offset, &ioport_field::set_crosshair_offset));

/* machine:parameters()
 * parameter:add(tag, val) - add tag = val parameter
 * parameter:lookup(tag) - get val for tag
 */

	sol().registry().new_usertype<parameters_manager>("parameters", "new", sol::no_constructor,
			"add", &parameters_manager::add,
			"lookup", &parameters_manager::lookup);

/* machine:video()
 * video:begin_recording([opt] filename) - start recording to filename if given or default
 * video:end_recording() - stop recording
 * video:snapshot() - save shot of all screens
 * video:is_recording() - get recording status
 * video:skip_this_frame() - is current frame going to be skipped
 * video:speed_factor() - get speed factor
 * video:speed_percent() - get percent from realtime
 * video.frameskip - current frameskip
 * video.throttled - throttle state
 * video.throttle_rate - throttle rate
 */

	sol().registry().new_usertype<video_manager>("video", "new", sol::no_constructor,
			"begin_recording", sol::overload([this](video_manager &vm, const char *filename) {
					std::string fn = filename;
					strreplace(fn, "/", PATH_SEPARATOR);
					strreplace(fn, "%g", machine().basename());
					vm.begin_recording(fn.c_str(), video_manager::MF_AVI);
				},
				[](video_manager &vm) { vm.begin_recording(nullptr, video_manager::MF_AVI); }),
			"end_recording", [this](video_manager &vm) {
					if(!vm.is_recording())
					{
						machine().logerror("[luaengine] No active recording to stop\n");
						return;
					}
					vm.end_recording(video_manager::MF_AVI);
				},
			"snapshot", &video_manager::save_active_screen_snapshots,
			"is_recording", &video_manager::is_recording,
			"skip_this_frame", &video_manager::skip_this_frame,
			"speed_factor", &video_manager::speed_factor,
			"speed_percent", &video_manager::speed_percent,
			"frame_update", &video_manager::frame_update,
			"frameskip", sol::property(&video_manager::frameskip, &video_manager::set_frameskip),
			"throttled", sol::property(&video_manager::throttled, &video_manager::set_throttled),
			"throttle_rate", sol::property(&video_manager::throttle_rate, &video_manager::set_throttle_rate));

/* machine:input()
 * input:code_from_token(token) - get input_code for KEYCODE_* string token
 * input:code_pressed(code) - get pressed state for input_code
 * input:code_to_token(code) - get KEYCODE_* string token for code
 * input:code_name(code) - get code friendly name
 * input:seq_from_tokens(tokens) - get input_seq for multiple space separated KEYCODE_* string tokens
 * input:seq_pressed(seq) - get pressed state for input_seq
 * input:seq_to_token(seq) - get KEYCODE_* string tokens for seq
 * input:seq_to_name(seq) - get seq friendly name
 */

	sol().registry().new_usertype<input_manager>("input", "new", sol::no_constructor,
			"code_from_token", [](input_manager &input, const char *token) { return sol::make_user(input.code_from_token(token)); },
			"code_pressed", [](input_manager &input, sol::user<input_code> code) { return input.code_pressed(code); },
			"code_to_token", [](input_manager &input, sol::user<input_code> code) { return input.code_to_token(code); },
			"code_name", [](input_manager &input, sol::user<input_code> code) { return input.code_name(code); },
			"seq_from_tokens", [](input_manager &input, const char *tokens) { input_seq seq; input.seq_from_tokens(seq, tokens); return sol::make_user(seq); },
			"seq_pressed", [](input_manager &input, sol::user<input_seq> seq) { return input.seq_pressed(seq); },
			"seq_to_tokens", [](input_manager &input, sol::user<input_seq> seq) { return input.seq_to_tokens(seq); },
			"seq_name", [](input_manager &input, sol::user<input_seq> seq) { return input.seq_name(seq); },
			"seq_poll_start", [](input_manager &input, input_item_class cls, sol::object seq) {
					input_seq *start = nullptr;
					if(seq.is<sol::user<input_seq>>())
						start = &seq.as<sol::user<input_seq>>();
					input.seq_poll_start(cls, start);
				},
			"seq_poll", &input_manager::seq_poll,
			"seq_poll_final", [](input_manager &input) { return sol::make_user(input.seq_poll_final()); });

/* machine:uiinput()
 * uiinput:find_mouse() - returns x, y, button state, ui render target
 * uiinput:pressed(key) - get pressed state for ui key
 */

	sol().registry().new_usertype<ui_input_manager>("uiinput", "new", sol::no_constructor,
			"find_mouse", [](ui_input_manager &ui) {
					int32_t x, y;
					bool button;
					render_target *rt = ui.find_mouse(&x, &y, &button);
					return std::tuple<int32_t, int32_t, bool, render_target *>(x, y, button, rt);
				},
			"pressed", &ui_input_manager::pressed);

/* render.targets[target_index]
 * render:ui_target()
 * target:view_bounds() - get x0, x1, y0, y1 bounds for target
 * target:width() - get target width
 * target:height() - get target height
 * target:pixel_aspect() - get target aspect
 * target:hidden() - is target hidden
 * target:is_ui_target() - is ui render target
 * target:index() - target index
 * target:view_name(index) - current target layout view name
 * target.max_update_rate -
 * target.view - current target layout view
 * target.orientation - current target orientation
 * target.backdrops - enable backdrops
 * target.bezels - enable bezels
 * target.marquees - enable marquees
 * target.screen_overlay - enable overlays
 * target.zoom - enable zoom
 */

	sol().registry().new_usertype<render_target>("target", "new", sol::no_constructor,
			"view_bounds", [](render_target &rt) {
					const render_bounds b = rt.current_view()->bounds();
					return std::tuple<float, float, float, float>(b.x0, b.x1, b.y0, b.y1);
				},
			"width", &render_target::width,
			"height", &render_target::height,
			"pixel_aspect", &render_target::pixel_aspect,
			"hidden", &render_target::hidden,
			"is_ui_target", &render_target::is_ui_target,
			"index", &render_target::index,
			"view_name", &render_target::view_name,
			"max_update_rate", sol::property(&render_target::max_update_rate, &render_target::set_max_update_rate),
			"view", sol::property(&render_target::view, &render_target::set_view),
			"orientation", sol::property(&render_target::orientation, &render_target::set_orientation),
			"backdrops", sol::property(&render_target::backdrops_enabled, &render_target::set_backdrops_enabled),
			"overlays", sol::property(&render_target::overlays_enabled, &render_target::set_overlays_enabled),
			"bezels", sol::property(&render_target::bezels_enabled, &render_target::set_bezels_enabled),
			"marquees", sol::property(&render_target::marquees_enabled, &render_target::set_marquees_enabled),
			"screen_overlay", sol::property(&render_target::screen_overlay_enabled, &render_target::set_screen_overlay_enabled),
			"zoom", sol::property(&render_target::zoom_to_screen, &render_target::set_zoom_to_screen));

/* render:ui_container() - this should be self explanatory
 */

	sol().registry().new_usertype<render_container>("render_container", "new", sol::no_constructor,
			"orientation", &render_container::orientation,
			"xscale", &render_container::xscale,
			"yscale", &render_container::yscale,
			"xoffset", &render_container::xoffset,
			"yoffset", &render_container::yoffset,
			"is_empty", &render_container::is_empty);

/* machine:render()
 * render:max_update_rate() -
 * render:ui_target() - target for ui drawing
 * render:ui_container() - container for ui drawing
 * render.target[] - render_target table
 */

	sol().registry().new_usertype<render_manager>("render", "new", sol::no_constructor,
			"max_update_rate", &render_manager::max_update_rate,
			"ui_target", &render_manager::ui_target,
			"ui_container", &render_manager::ui_container,
			"targets", [this](render_manager &r) {
					sol::table target_table = sol().create_table();
					int tc = 0;
					for(render_target &curr_rt : r.targets())
						target_table[tc++] = &curr_rt;
					return target_table;
				});

/* machine.screens[screen_tag]
 * screen:draw_box(x1, y1, x2, y2, fillcol, linecol) - draw box from (x1, y1)-(x2, y2) colored linecol filled with fillcol
 * screen:draw_line(x1, y1, x2, y2, linecol) - draw line from (x1, y1)-(x2, y2) colored linecol
 * screen:draw_text(x || justify, y, message, [opt] color) - draw message at (x, y) or at line y with left, right, center justification
 * screen:height() - screen height
 * screen:width() - screen width
 * screen:orientation() - screen angle, flipx, flipy
 * screen:refresh() - screen refresh rate
 * screen:snapshot() - save snap shot
 * screen:type() - screen drawing type
 * screen:frame_number() - screen frame count
 * screen:name() - screen device full name
 * screen:shortname() - screen device short name
 * screen:tag() - screen device tag
 * screen:xscale() - screen x scale factor
 * screen:yscale() - screen y scale factor
*/

	sol().registry().new_usertype<screen_device>("screen_dev", "new", sol::no_constructor,
			"draw_box", [](screen_device &sdev, float x1, float y1, float x2, float y2, uint32_t bgcolor, uint32_t fgcolor) {
					int sc_width = sdev.visible_area().width();
					int sc_height = sdev.visible_area().height();
					x1 = std::min(std::max(0.0f, x1), float(sc_width-1)) / float(sc_width);
					y1 = std::min(std::max(0.0f, y1), float(sc_height-1)) / float(sc_height);
					x2 = std::min(std::max(0.0f, x2), float(sc_width-1)) / float(sc_width);
					y2 = std::min(std::max(0.0f, y2), float(sc_height-1)) / float(sc_height);
					mame_machine_manager::instance()->ui().draw_outlined_box(sdev.container(), x1, y1, x2, y2, fgcolor, bgcolor);
				},
			"draw_line", [](screen_device &sdev, float x1, float y1, float x2, float y2, uint32_t color) {
					int sc_width = sdev.visible_area().width();
					int sc_height = sdev.visible_area().height();
					x1 = std::min(std::max(0.0f, x1), float(sc_width-1)) / float(sc_width);
					y1 = std::min(std::max(0.0f, y1), float(sc_height-1)) / float(sc_height);
					x2 = std::min(std::max(0.0f, x2), float(sc_width-1)) / float(sc_width);
					y2 = std::min(std::max(0.0f, y2), float(sc_height-1)) / float(sc_height);
					sdev.container().add_line(x1, y1, x2, y2, UI_LINE_WIDTH, rgb_t(color), PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA));
				},
			"draw_text", [this](screen_device &sdev, sol::object xobj, float y, const char *msg, sol::object color) {
					int sc_width = sdev.visible_area().width();
					int sc_height = sdev.visible_area().height();
					auto justify = ui::text_layout::LEFT;
					float x = 0;
					if(xobj.is<float>())
					{
						x = std::min(std::max(0.0f, xobj.as<float>()), float(sc_width-1)) / float(sc_width);
						y = std::min(std::max(0.0f, y), float(sc_height-1)) / float(sc_height);
					}
					else if(xobj.is<const char *>())
					{
						std::string just_str = xobj.as<const char *>();
						if(just_str == "right")
							justify = ui::text_layout::RIGHT;
						else if(just_str == "center")
							justify = ui::text_layout::CENTER;
					}
					else
					{
						luaL_error(m_lua_state, "Error in param 1 to draw_text");
						return;
					}
					rgb_t textcolor = UI_TEXT_COLOR;
					rgb_t bgcolor = UI_TEXT_BG_COLOR;
					if(color.is<uint32_t>())
						textcolor = rgb_t(color.as<uint32_t>());
					mame_machine_manager::instance()->ui().draw_text_full(sdev.container(), msg, x, y, (1.0f - x),
										justify, ui::text_layout::WORD, mame_ui_manager::NORMAL, textcolor,
										bgcolor, nullptr, nullptr);
				},
			"height", [](screen_device &sdev) { return sdev.visible_area().height(); },
			"width", [](screen_device &sdev) { return sdev.visible_area().width(); },
			"orientation", [](screen_device &sdev) {
					uint32_t flags = sdev.machine().system().flags & ORIENTATION_MASK;
					int rotation_angle = 0;
					switch (flags)
					{
						case ORIENTATION_FLIP_X:
							rotation_angle = 0;
							break;
						case ORIENTATION_SWAP_XY:
						case ORIENTATION_SWAP_XY|ORIENTATION_FLIP_X:
							rotation_angle = 90;
							break;
						case ORIENTATION_FLIP_Y:
						case ORIENTATION_FLIP_X|ORIENTATION_FLIP_Y:
							rotation_angle = 180;
							break;
						case ORIENTATION_SWAP_XY|ORIENTATION_FLIP_Y:
						case ORIENTATION_SWAP_XY|ORIENTATION_FLIP_X|ORIENTATION_FLIP_Y:
							rotation_angle = 270;
							break;
					}
					return std::tuple<int, bool, bool>(rotation_angle, flags & ORIENTATION_FLIP_X, flags & ORIENTATION_FLIP_Y);
				},
			"refresh", [](screen_device &sdev) { return ATTOSECONDS_TO_HZ(sdev.refresh_attoseconds()); },
			"snapshot", [this](screen_device &sdev, sol::object filename) -> sol::object {
					emu_file file(machine().options().snapshot_directory(), OPEN_FLAG_WRITE | OPEN_FLAG_CREATE | OPEN_FLAG_CREATE_PATHS);
					osd_file::error filerr;
					if(filename.is<const char *>())
					{
						std::string snapstr(filename.as<const char *>());
						strreplace(snapstr, "/", PATH_SEPARATOR);
						strreplace(snapstr, "%g", machine().basename());
						filerr = file.open(snapstr.c_str());
					}
					else
						filerr = machine().video().open_next(file, "png");
					if(filerr != osd_file::error::NONE)
						return sol::make_object(sol(), filerr);
					machine().video().save_snapshot(&sdev, file);
					return sol::make_object(sol(), sol::nil);
				},
			"type", [](screen_device &sdev) {
					switch (sdev.screen_type())
					{
						case SCREEN_TYPE_RASTER:  return "raster"; break;
						case SCREEN_TYPE_VECTOR:  return "vector"; break;
						case SCREEN_TYPE_LCD:     return "lcd"; break;
						case SCREEN_TYPE_SVG:     return "svg"; break;
						default: break;
					}
					return "unknown";
				},
			"frame_number", &screen_device::frame_number,
			"name", &screen_device::name,
			"shortname", &screen_device::shortname,
			"tag", &screen_device::tag,
			"xscale", &screen_device::xscale,
			"yscale", &screen_device::yscale);

/* mame_manager:ui()
 * ui:is_menu_active() - ui menu state
 * ui:options() - ui core_options
 * ui.show_fps - fps display enabled
 * ui.show_profiler - profiler display enabled
 * ui:get_line_height() - current ui font height
 * ui:get_string_width(str, scale) - get str width with ui font at scale factor of current font size
 * ui:get_char_width(char) - get width of utf8 glyph char with ui font
 */

	sol().registry().new_usertype<mame_ui_manager>("ui", "new", sol::no_constructor,
			"is_menu_active", &mame_ui_manager::is_menu_active,
			"options", [](mame_ui_manager &m) { return static_cast<core_options *>(&m.options()); },
			"show_fps", sol::property(&mame_ui_manager::show_fps, &mame_ui_manager::set_show_fps),
			"show_profiler", sol::property(&mame_ui_manager::show_profiler, &mame_ui_manager::set_show_profiler),
			"single_step", sol::property(&mame_ui_manager::single_step, &mame_ui_manager::set_single_step),
			"get_line_height", &mame_ui_manager::get_line_height,
			"get_string_width", &mame_ui_manager::get_string_width,
			// sol converts char32_t to a string
			"get_char_width", [](mame_ui_manager &m, uint32_t utf8char) { return m.get_char_width(utf8char); });

/* device.state[]
 * state:name() - get device state name
 * state:value() - get device state value
 * state:is_visible() - is state visible in debugger
 * state:is_divider() - is state a divider
 */

	sol().registry().new_usertype<device_state_entry>("dev_space", "new", sol::no_constructor,
			"name", &device_state_entry::symbol,
			"value", sol::property([this](device_state_entry &entry) -> uint64_t {
					device_state_interface *state = entry.parent_state();
					if(state)
					{
						machine().save().dispatch_presave();
						return state->state_int(entry.index());
					}
					return 0;
				},
				[this](device_state_entry &entry, uint64_t val) {
					device_state_interface *state = entry.parent_state();
					if(state)
					{
						state->set_state_int(entry.index(), val);
						machine().save().dispatch_presave();
					}
				}),
			"is_visible", &device_state_entry::visible,
			"is_divider", &device_state_entry::divider);

/* machine:memory()
 * memory.banks[] - table of memory banks
 * memory.regions[] - table of memory regions
 * memory.shares[] - table of memory shares
 */

	sol().registry().new_usertype<memory_manager>("memory", "new", sol::no_constructor,
			"banks", sol::property([this](memory_manager &mm) {
					sol::table table = sol().create_table();
					for (auto &bank : mm.banks())
						table[bank.second->tag()] = bank.second.get();
					return table;
				}),
			"regions", sol::property([this](memory_manager &mm) {
					sol::table table = sol().create_table();
					for (auto &region : mm.regions())
						table[region.second->name()] = region.second.get();
					return table;
				}),
			"shares", sol::property([this](memory_manager &mm) {
					sol::table table = sol().create_table();
					for (auto &share : mm.shares())
						table[share.first] = share.second.get();
					return table;
				}));

	sol().registry().new_usertype<memory_region>("region", "new", sol::no_constructor,
			"read_i8", &region_read<int8_t>,
			"read_u8", &region_read<uint8_t>,
			"read_i16", &region_read<int16_t>,
			"read_u16", &region_read<uint16_t>,
			"read_i32", &region_read<int32_t>,
			"read_u32", &region_read<uint32_t>,
			"read_i64", &region_read<int64_t>,
			"read_u64", &region_read<uint64_t>,
			"write_i8", &region_write<int8_t>,
			"write_u8", &region_write<uint8_t>,
			"write_i16", &region_write<int16_t>,
			"write_u16", &region_write<uint16_t>,
			"write_i32", &region_write<int32_t>,
			"write_u32", &region_write<uint32_t>,
			"write_i64", &region_write<int64_t>,
			"write_u64", &region_write<uint64_t>,
			"size", sol::property(&memory_region::bytes));

	sol().registry().new_usertype<memory_share>("share", "new", sol::no_constructor,
			"read_i8", &share_read<int8_t>,
			"read_u8", &share_read<uint8_t>,
			"read_i16", &share_read<int16_t>,
			"read_u16", &share_read<uint16_t>,
			"read_i32", &share_read<int32_t>,
			"read_u32", &share_read<uint32_t>,
			"read_i64", &share_read<int64_t>,
			"read_u64", &share_read<uint64_t>,
			"write_i8", &share_write<int8_t>,
			"write_u8", &share_write<uint8_t>,
			"write_i16", &share_write<int16_t>,
			"write_u16", &share_write<uint16_t>,
			"write_i32", &share_write<int32_t>,
			"write_u32", &share_write<uint32_t>,
			"write_i64", &share_write<int64_t>,
			"write_u64", &share_write<uint64_t>,
			"size", sol::property(&memory_share::bytes));

/* machine:output()
 * output:set_value(name, val) - set output name to val
 * output:set_indexed_value(index, val) - set output index to val
 * output:get_value(name) - get output name value
 * output:get_indexed_value(index) - get output index value
 * output:name_to_id(name) - get index for name
 * output:id_to_name(index) - get name for index
 */

	sol().registry().new_usertype<output_manager>("output", "new", sol::no_constructor,
			"set_value", &output_manager::set_value,
			"set_indexed_value", &output_manager::set_indexed_value,
			"get_value", &output_manager::get_value,
			"get_indexed_value", &output_manager::get_indexed_value,
			"name_to_id", &output_manager::name_to_id,
			"id_to_name", &output_manager::id_to_name);

/* machine.images[] - images are floppy/cart/cdrom/tape/hdd etc, otherwise this should be self explanatory
 */

	sol().registry().new_usertype<device_image_interface>("image", "new", sol::no_constructor,
			"exists", &device_image_interface::exists,
			"filename", &device_image_interface::filename,
			"longname", &device_image_interface::longname,
			"manufacturer", &device_image_interface::manufacturer,
			"year", &device_image_interface::year,
			"software_list_name", &device_image_interface::software_list_name,
			"image_type_name", &device_image_interface::image_type_name,
			"load", &device_image_interface::load,
			"unload", &device_image_interface::unload,
			"crc", &device_image_interface::crc,
			"device", sol::property(static_cast<const device_t &(device_image_interface::*)() const>(&device_image_interface::device)),
			"is_readable", sol::property(&device_image_interface::is_readable),
			"is_writeable", sol::property(&device_image_interface::is_writeable),
			"is_creatable", sol::property(&device_image_interface::is_creatable),
			"is_reset_on_load", sol::property(&device_image_interface::is_reset_on_load));

	sol().registry().new_usertype<mame_machine_manager>("manager", "new", sol::no_constructor,
			"machine", &machine_manager::machine,
			"options", [](mame_machine_manager &m) { return static_cast<core_options *>(&m.options()); },
			"plugins", [](mame_machine_manager &m) { return static_cast<core_options *>(&m.plugins()); },
			"ui", &mame_machine_manager::ui);
	sol()["manager"] = std::ref(*mame_machine_manager::instance());
	sol()["mame_manager"] = std::ref(*mame_machine_manager::instance());
}

//-------------------------------------------------
//  frame_hook - called at each frame refresh, used to draw a HUD
//-------------------------------------------------
bool lua_engine::frame_hook()
{
	return execute_function("LUA_ON_FRAME_DONE");
}

//-------------------------------------------------
//  close - close and cleanup of lua engine
//-------------------------------------------------

void lua_engine::close()
{
	m_sol_state.reset();
	if (m_lua_state)
	{
		lua_settop(m_lua_state, 0);  /* clear stack */
		lua_close(m_lua_state);
		m_lua_state = nullptr;
	}
}

void lua_engine::resume(void *ptr, int nparam)
{
	lua_State *L = static_cast<lua_State *>(ptr);
	int stat = lua_resume(L, nullptr, 0);
	if((stat != LUA_OK) && (stat != LUA_YIELD))
	{
		osd_printf_error("[LUA ERROR] in resume: %s\n", lua_tostring(L, -1));
		lua_pop(L, 1);
	}
}

void lua_engine::run(sol::load_result res)
{
	if(res.valid())
	{
		auto ret = (res.get<sol::protected_function>())();
		if(!ret.valid())
		{
			sol::error err = ret;
			osd_printf_error("[LUA ERROR] in run: %s\n", err.what());
		}
	}
	else
		osd_printf_error("[LUA ERROR] %d loading Lua script\n", (int)res.status());
}

//-------------------------------------------------
//  execute - load and execute script
//-------------------------------------------------

void lua_engine::load_script(const char *filename)
{
	run(sol().load_file(filename));
}

//-------------------------------------------------
//  execute_string - execute script from string
//-------------------------------------------------

void lua_engine::load_string(const char *value)
{
	run(sol().load(value));
}
