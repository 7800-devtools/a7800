// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
//============================================================
//
//  chainmanager.cpp - BGFX shader chain manager
//
//  Provides loading for BGFX shader effect chains, defined
//  by chain.h and read by chainreader.h
//
//============================================================

#include <bx/readerwriter.h>
#include <bx/crtimpl.h>

#include "emu.h"
#include "../frontend/mame/ui/slider.h"

#include "osdcore.h"
#include "modules/osdwindow.h"

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>

#include "bgfxutil.h"

#include "chainmanager.h"
#include "chainreader.h"
#include "chain.h"

#include "texture.h"
#include "target.h"
#include "slider.h"

#include "sliderdirtynotifier.h"

using namespace rapidjson;

const uint32_t chain_manager::CHAIN_NONE = 0;

chain_manager::chain_manager(running_machine& machine, osd_options& options, texture_manager& textures, target_manager& targets, effect_manager& effects, uint32_t window_index, slider_dirty_notifier& slider_notifier)
	: m_machine(machine)
	, m_options(options)
	, m_textures(textures)
	, m_targets(targets)
	, m_effects(effects)
	, m_window_index(window_index)
	, m_slider_notifier(slider_notifier)
	, m_screen_count(0)
{
	refresh_available_chains();
	parse_chain_selections(options.bgfx_screen_chains());
}

chain_manager::~chain_manager()
{
	destroy_chains();
}

void chain_manager::refresh_available_chains()
{
	m_available_chains.clear();
	m_available_chains.push_back(chain_desc("none", ""));

	std::string chains_path;
	osd_subst_env(chains_path, util::string_format("%s" PATH_SEPARATOR "chains", m_options.bgfx_path()));
	find_available_chains(chains_path, "");

	destroy_unloaded_chains();
}

void chain_manager::destroy_unloaded_chains()
{
	// O(shaders*available_chains), but we don't care because asset reloading happens rarely
	for (int i = 0; i < m_chain_names.size(); i++)
	{
		std::string name = m_chain_names[i];
		if (name.length() > 0)
		{
			for (chain_desc desc : m_available_chains)
			{
				if (desc.m_name == name)
				{
					delete m_screen_chains[i];
					m_screen_chains[i] = nullptr;
					m_chain_names[i] = "";
					m_current_chain[i] = CHAIN_NONE;
					break;
				}
			}
		}
	}
}

void chain_manager::find_available_chains(std::string root, std::string path)
{
	osd::directory::ptr directory = osd::directory::open(root + path);
	if (directory != nullptr)
	{
		for (const osd::directory::entry *entry = directory->read(); entry != nullptr; entry = directory->read())
		{
			if (entry->type == osd::directory::entry::entry_type::FILE)
			{
				std::string name(entry->name);
				std::string extension(".json");

				// Does the name has at least one character in addition to ".json"?
				if (name.length() > extension.length())
				{
					size_t start = name.length() - extension.length();
					std::string test_segment = name.substr(start, extension.length());

					// Does it end in .json?
					if (test_segment == extension)
					{
						m_available_chains.push_back(chain_desc(name.substr(0, start), path));
					}
				}
			}
			else if (entry->type == osd::directory::entry::entry_type::DIR)
			{
				std::string name = entry->name;
				if (!(name == "." || name == ".."))
				{
					std::string appended_path = path + "/" + name;
					if (path.length() == 0)
					{
						appended_path = name;
					}
					find_available_chains(root, path + "/" + name);
				}
			}
		}
	}
}

bgfx_chain* chain_manager::load_chain(std::string name, uint32_t screen_index)
{
	if (name.length() < 5 || (name.compare(name.length() - 5, 5, ".json") != 0))
	{
		name = name + ".json";
	}
	std::string path;
	osd_subst_env(path, util::string_format("%s" PATH_SEPARATOR "chains" PATH_SEPARATOR, m_options.bgfx_path()));
	path += name;

	bx::CrtFileReader reader;
	if (!bx::open(&reader, path.c_str()))
	{
		printf("Unable to open chain file %s, falling back to no post processing\n", path.c_str());
		return nullptr;
	}

	int32_t size(bx::getSize(&reader));

	char* data = new char[size + 1];
	bx::read(&reader, reinterpret_cast<void*>(data), size);
	bx::close(&reader);
	data[size] = 0;

	Document document;
	document.Parse<kParseCommentsFlag>(data);

	delete [] data;

	if (document.HasParseError())
	{
		std::string error(GetParseError_En(document.GetParseError()));
		printf("Unable to parse chain %s. Errors returned:\n", path.c_str());
		printf("%s\n", error.c_str());
		return nullptr;
	}

	bgfx_chain* chain = chain_reader::read_from_value(document, name + ": ", *this, screen_index);

	if (chain == nullptr)
	{
		printf("Unable to load chain %s, falling back to no post processing\n", path.c_str());
		return nullptr;
	}

	return chain;
}

void chain_manager::parse_chain_selections(std::string chain_str)
{
	std::vector<std::string> chain_names = split_option_string(chain_str);

	while (m_current_chain.size() != chain_names.size())
	{
		m_screen_chains.push_back(nullptr);
		m_chain_names.push_back("");
		m_current_chain.push_back(CHAIN_NONE);
	}

	for (size_t index = 0; index < chain_names.size(); index++)
	{
		size_t chain_index = 0;
		for (chain_index = 0; chain_index < m_available_chains.size(); chain_index++)
		{
			if (m_available_chains[chain_index].m_name == chain_names[index])
			{
				break;
			}
		}

		if (chain_index < m_available_chains.size())
		{
			m_current_chain[index] = chain_index;
			m_chain_names[index] = m_available_chains[chain_index].m_name;
		}
		else
		{
			m_current_chain[index] = CHAIN_NONE;
			m_chain_names[index] = "";
		}
	}
}

std::vector<std::string> chain_manager::split_option_string(std::string chain_str) const
{
	std::vector<std::string> chain_names;

	uint32_t length = chain_str.length();
	uint32_t win = 0;
	uint32_t last_start = 0;
	for (uint32_t i = 0; i < length + 1; i++)
	{
		if (i == length || chain_str[i] == ',' || chain_str[i] == ':')
		{
			if (win == m_window_index)
			{
				chain_names.push_back(chain_str.substr(last_start, i - last_start));
			}
			last_start = i + 1;
			if (chain_str[i] == ':')
			{
				win++;
			}
		}
	}

	return chain_names;
}

void chain_manager::load_chains()
{
	for (size_t chain = 0; chain < m_current_chain.size() && chain < m_screen_chains.size(); chain++)
	{
		if (m_current_chain[chain] != CHAIN_NONE)
		{
			chain_desc& desc = m_available_chains[m_current_chain[chain]];
			m_chain_names[chain] = desc.m_name;
			m_screen_chains[chain] = load_chain(desc.m_path + "/" + desc.m_name, uint32_t(chain));
		}
	}
}

void chain_manager::destroy_chains()
{
	for (size_t index = 0; index < m_screen_chains.size(); index++)
	{
		if (m_screen_chains[index] != nullptr)
		{
			delete m_screen_chains[index];
			m_screen_chains[index] = nullptr;
		}
	}
}

void chain_manager::reload_chains()
{
	destroy_chains();
	load_chains();
}

bgfx_chain* chain_manager::screen_chain(uint32_t screen)
{
	if (screen >= m_screen_chains.size())
	{
		return m_screen_chains[m_screen_chains.size() - 1];
	}
	else
	{
		return m_screen_chains[screen];
	}
}

void chain_manager::process_screen_quad(uint32_t view, uint32_t screen, render_primitive* prim, osd_window &window)
{
	uint16_t tex_width(prim->texture.width);
	uint16_t tex_height(prim->texture.height);

	const bgfx::Memory* mem = bgfx_util::mame_texture_data_to_bgfx_texture_data(prim->flags & PRIMFLAG_TEXFORMAT_MASK,
		tex_width, tex_height, prim->texture.rowpixels, prim->texture.palette, prim->texture.base);

	std::string full_name = "screen" + std::to_string(screen);
	bgfx_texture *texture = new bgfx_texture(full_name, bgfx::TextureFormat::RGBA8, tex_width, tex_height, mem, BGFX_TEXTURE_U_CLAMP | BGFX_TEXTURE_V_CLAMP | BGFX_TEXTURE_MIN_POINT | BGFX_TEXTURE_MAG_POINT | BGFX_TEXTURE_MIP_POINT);
	m_textures.add_provider(full_name, texture);

	const bool any_targets_rebuilt = m_targets.update_target_sizes(screen, tex_width, tex_height, TARGET_STYLE_GUEST);
	if (any_targets_rebuilt)
	{
		for (bgfx_chain* chain : m_screen_chains)
		{
			if (chain != nullptr)
			{
				chain->repopulate_targets();
			}
		}
	}

	bgfx_chain* chain = screen_chain(screen);
	chain->process(prim, view, screen, m_textures, window, bgfx_util::get_blend_state(PRIMFLAG_GET_BLENDMODE(prim->flags)));
	view += chain->applicable_passes();

	m_textures.add_provider(full_name, nullptr);
	delete texture;
}

std::vector<render_primitive*> chain_manager::count_screens(render_primitive* prim)
{
	std::vector<render_primitive*> screens;

	int screen_count = 0;
	while (prim != nullptr)
	{
		if (PRIMFLAG_GET_SCREENTEX(prim->flags))
		{
			screen_count++;
			screens.push_back(prim);
		}
		prim = prim->next();
	}

	if (screen_count > 0)
	{
		update_screen_count(screen_count);
		m_targets.update_screen_count(screen_count);
	}

	return screens;
}

void chain_manager::update_screen_count(uint32_t screen_count)
{
	if (screen_count != m_screen_count)
	{
		m_slider_notifier.set_sliders_dirty();
		m_screen_count = screen_count;

		// Ensure we have one screen chain entry per screen
		while (m_screen_chains.size() < m_screen_count)
		{
			m_screen_chains.push_back(nullptr);
			m_chain_names.push_back("");
			m_current_chain.push_back(CHAIN_NONE);
		}

		// Ensure we have a screen chain selection slider per screen
		while (m_selection_sliders.size() < m_screen_count)
		{
			create_selection_slider(m_selection_sliders.size());
		}

		load_chains();
	}
}

int32_t chain_manager::slider_changed(running_machine &machine, void *arg, int id, std::string *str, int32_t newval)
{
	if (newval != SLIDER_NOCHANGE)
	{
		m_current_chain[id] = newval;

		std::vector<std::vector<float>> settings = slider_settings();
		reload_chains();
		restore_slider_settings(id, settings);

		m_slider_notifier.set_sliders_dirty();
	}

	if (str != nullptr)
	{
		*str = string_format("%s", m_available_chains[m_current_chain[id]].m_name.c_str());
	}

	return m_current_chain[id];
}

void chain_manager::create_selection_slider(uint32_t screen_index)
{
	if (screen_index < m_selection_sliders.size())
	{
		return;
	}

	std::string description = "Window " + std::to_string(m_window_index) + ", Screen " + std::to_string(screen_index) + " Effect:";
	size_t size = sizeof(slider_state) + description.length();
	slider_state *state = reinterpret_cast<slider_state *>(auto_alloc_array_clear(m_machine, uint8_t, size));

	state->minval = 0;
	state->defval = m_current_chain[screen_index];
	state->maxval = m_available_chains.size() - 1;
	state->incval = 1;

	using namespace std::placeholders;
	state->update = std::bind(&chain_manager::slider_changed, this, _1, _2, _3, _4, _5);
	state->arg = this;
	state->id = screen_index;
	strcpy(state->description, description.c_str());

	ui::menu_item item;
	item.text = state->description;
	item.subtext = "";
	item.flags = 0;
	item.ref = state;
	item.type = ui::menu_item_type::SLIDER;
	m_selection_sliders.push_back(item);
}

uint32_t chain_manager::handle_screen_chains(uint32_t view, render_primitive *starting_prim, osd_window& window)
{
	std::vector<render_primitive*> screens = count_screens(starting_prim);

	if (screens.size() == 0)
	{
		return 0;
	}

	// Process each screen as necessary
	uint32_t used_views = 0;
	uint32_t screen_index = 0;
	for (render_primitive* prim : screens)
	{
		if (m_current_chain[screen_index] == CHAIN_NONE || screen_chain(screen_index) == nullptr)
		{
			screen_index++;
			continue;
		}

		uint16_t screen_width(floorf(prim->get_full_quad_width() + 0.5f));
		uint16_t screen_height(floorf(prim->get_full_quad_height() + 0.5f));
		if (window.swap_xy())
		{
			std::swap(screen_width, screen_height);
		}

		const bool any_targets_rebuilt = m_targets.update_target_sizes(screen_index, screen_width, screen_height, TARGET_STYLE_NATIVE);
		if (any_targets_rebuilt)
		{
			for (bgfx_chain* chain : m_screen_chains)
			{
				if (chain != nullptr)
				{
					chain->repopulate_targets();
				}
			}
		}

		process_screen_quad(view + used_views, screen_index, prim, window);
		used_views += screen_chain(screen_index)->applicable_passes();

		screen_index++;
	}

	bgfx::setViewFrameBuffer(view + used_views, BGFX_INVALID_HANDLE);

	return used_views;
}

bool chain_manager::has_applicable_chain(uint32_t screen)
{
	return screen < m_screen_count && m_current_chain[screen] != CHAIN_NONE && m_screen_chains[screen] != nullptr;
}

bool chain_manager::needs_sliders()
{
	return m_screen_count > 0 && m_available_chains.size() > 1;
}

void chain_manager::restore_slider_settings(int32_t id, std::vector<std::vector<float>>& settings)
{
	if (!needs_sliders())
	{
		return;
	}

	for (size_t index = 0; index < m_screen_chains.size() && index < m_screen_count; index++)
	{
		if (index == id)
		{
			continue;
		}

		bgfx_chain* chain = m_screen_chains[index];
		if (chain == nullptr)
		{
			continue;
		}

		std::vector<bgfx_slider*> chain_sliders = chain->sliders();
		for (size_t slider = 0; slider < chain_sliders.size(); slider++)
		{
			chain_sliders[slider]->import(settings[index][slider]);
		}
	}
}

std::vector<std::vector<float>> chain_manager::slider_settings()
{
	std::vector<std::vector<float>> curr;

	if (!needs_sliders())
	{
		return curr;
	}

	for (size_t index = 0; index < m_screen_chains.size() && index < m_screen_count; index++)
	{
		curr.push_back(std::vector<float>());

		bgfx_chain* chain = m_screen_chains[index];
		if (chain == nullptr)
		{
			continue;
		}

		std::vector<bgfx_slider*> chain_sliders = chain->sliders();
		for (bgfx_slider* slider : chain_sliders)
		{
			curr[index].push_back(slider->value());
		}
	}

	return curr;
}

std::vector<ui::menu_item> chain_manager::get_slider_list()
{
	std::vector<ui::menu_item> sliders;

	if (!needs_sliders())
	{
		return sliders;
	}

	for (size_t index = 0; index < m_screen_chains.size() && index < m_screen_count; index++)
	{
		bgfx_chain* chain = m_screen_chains[index];
		sliders.push_back(m_selection_sliders[index]);

		if (chain == nullptr)
		{
			continue;
		}

		std::vector<bgfx_chain_entry*> chain_entries = chain->entries();
		for (bgfx_chain_entry* entry : chain_entries)
		{
			std::vector<bgfx_input_pair*> entry_inputs = entry->inputs();
			for (bgfx_input_pair* input : entry_inputs)
			{
				std::vector<ui::menu_item> input_sliders = input->get_slider_list();
				for (ui::menu_item slider : input_sliders)
				{
					sliders.push_back(slider);
				}
			}
		}

		std::vector<bgfx_slider*> chain_sliders = chain->sliders();
		for (bgfx_slider* slider : chain_sliders)
		{
			slider_state* core_slider = slider->core_slider();

			ui::menu_item item;
			item.text = core_slider->description;
			item.subtext = "";
			item.flags = 0;
			item.ref = core_slider;
			item.type = ui::menu_item_type::SLIDER;
			m_selection_sliders.push_back(item);

			sliders.push_back(item);
		}

		if (chain_sliders.size() > 0)
		{
			ui::menu_item item;
			item.text = MENU_SEPARATOR_ITEM;
			item.subtext = "";
			item.flags = 0;
			item.ref = nullptr;
			item.type = ui::menu_item_type::SEPARATOR;

			sliders.push_back(item);
		}
	}

	return sliders;
}
