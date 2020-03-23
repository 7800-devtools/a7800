// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic
/***************************************************************************

    luaengine.h

    Controls execution of the core MAME system.

***************************************************************************/

#pragma once

#ifndef __EMU_H__
#error Dont include this file directly; include emu.h instead.
#endif

#ifndef __LUA_ENGINE_H__
#define __LUA_ENGINE_H__

#if defined(__GNUC__) && (__GNUC__ > 6)
#pragma GCC diagnostic ignored "-Wnoexcept-type"
#endif

#include <map>
#include <condition_variable>
#define SOL_SAFE_USERTYPE
//#define SOL_CHECK_ARGUMENTS
#include "sol2/sol.hpp"

struct lua_State;

class lua_engine
{
public:
	// construction/destruction
	lua_engine();
	~lua_engine();

	void initialize();
	void load_script(const char *filename);
	void load_string(const char *value);

	bool frame_hook();

	void menu_populate(const std::string &menu, std::vector<std::tuple<std::string, std::string, std::string>> &menu_list);
	bool menu_callback(const std::string &menu, int index, const std::string &event);

	void set_machine(running_machine *machine) { m_machine = machine; }
	std::vector<std::string> &get_menu() { return m_menu; }
	void attach_notifiers();
	void on_frame_done();
	void on_periodic();

	template<typename T, typename U>
	bool call_plugin(const std::string &name, const T in, U &out)
	{
		bool ret = false;
		sol::object outobj = call_plugin(name, sol::make_object(sol(), in));
		if(outobj.is<U>())
		{
			out = outobj.as<U>();
			ret = true;
		}
		return ret;
	}

	template<typename T, typename U>
	bool call_plugin(const std::string &name, const T in, std::vector<U> &out)
	{
		bool ret = false;
		sol::object outobj = call_plugin(name, sol::make_object(sol(), in));
		if(outobj.is<sol::table>())
		{
			for(auto &entry : outobj.as<sol::table>())
			{
				if(entry.second.template is<U>())
				{
					out.push_back(entry.second.template as<U>());
					ret = true;
				}
			}
		}
		return ret;
	}

	// this can also check if a returned table contains type T
	template<typename T, typename U>
	bool call_plugin_check(const std::string &name, const U in, bool table = false)
	{
		bool ret = false;
		sol::object outobj = call_plugin(name, sol::make_object(sol(), in));
		if(outobj.is<T>() && !table)
			ret = true;
		else if(outobj.is<sol::table>() && table)
		{
			// check just one entry, checking the whole thing shouldn't be necessary as this only supports homogeneous tables
			if(outobj.as<sol::table>().begin().operator*().second.template is<T>())
				ret = true;
		}
		return ret;
	}

	template<typename T>
	void call_plugin_set(const std::string &name, const T in)
	{
		call_plugin(name, sol::make_object(sol(), in));
	}

	sol::state_view &sol() const { return *m_sol_state; }
private:
	// internal state
	lua_State *m_lua_state;
	std::unique_ptr<sol::state_view> m_sol_state;
	running_machine *m_machine;

	std::vector<std::string> m_menu;

	running_machine &machine() const { return *m_machine; }

	void on_machine_prestart();
	void on_machine_start();
	void on_machine_stop();
	void on_machine_pause();
	void on_machine_resume();
	void on_machine_frame();

	void resume(void *ptr, int nparam);
	void register_function(sol::function func, const char *id);
	bool execute_function(const char *id);
	sol::object call_plugin(const std::string &name, sol::object in);

	struct addr_space {
		addr_space(address_space &space, device_memory_interface &dev) :
			space(space), dev(dev) {}
		template<typename T> T mem_read(offs_t address, sol::object shift);
		template<typename T> void mem_write(offs_t address, T val, sol::object shift);
		template<typename T> T log_mem_read(offs_t address);
		template<typename T> void log_mem_write(offs_t address, T val);
		template<typename T> T direct_mem_read(offs_t address);
		template<typename T> void direct_mem_write(offs_t address, T val);
		const char *name() const { return space.name(); }

		address_space &space;
		device_memory_interface &dev;
	};

	template<typename T> static T share_read(memory_share &share, offs_t address);
	template<typename T> static void share_write(memory_share &share, offs_t address, T val);
	template<typename T> static T region_read(memory_region &region, offs_t address);
	template<typename T> static void region_write(memory_region &region, offs_t address, T val);

	struct save_item {
		void *base;
		unsigned int size;
		unsigned int count;
	};

	void close();

	void run(sol::load_result res);

	struct context
	{
		context() { busy = false; yield = false; }
		std::string result;
		std::condition_variable sync;
		bool busy;
		bool yield;
	};
};

#endif  /* __LUA_ENGINE_H__ */
