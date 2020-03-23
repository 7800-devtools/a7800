// license:BSD-3-Clause
// copyright-holders:Maurizio Petrarota
/***************************************************************************

    ui/selgame.h

    Main UI menu.

***************************************************************************/

#pragma once

#ifndef MAME_FRONTEND_UI_SELGAME_H
#define MAME_FRONTEND_UI_SELGAME_H

#include "ui/selmenu.h"

class media_auditor;

namespace ui {

class menu_select_game : public menu_select_launch
{
public:
	menu_select_game(mame_ui_manager &mui, render_container &container, const char *gamename);
	virtual ~menu_select_game();

	// force game select menu
	static void force_game_select(mame_ui_manager &mui, render_container &container);

protected:
	virtual bool menu_has_search_active() override { return !m_search.empty(); }

private:
	enum
	{
		CONF_OPTS = 1,
		CONF_MACHINE,
		CONF_PLUGINS,
	};

	enum { VISIBLE_GAMES_IN_SEARCH = 200 };
	std::string m_search;
	static bool first_start;
	static int m_isabios;
	int highlight;

	static std::vector<const game_driver *> m_sortedlist;
	std::vector<const game_driver *> m_availsortedlist;
	std::vector<const game_driver *> m_unavailsortedlist;
	std::vector<const game_driver *> m_displaylist;

	const game_driver *m_searchlist[VISIBLE_GAMES_IN_SEARCH + 1];

	virtual void populate(float &customtop, float &custombottom) override;
	virtual void handle() override;

	// draw left panel
	virtual float draw_left_panel(float x1, float y1, float x2, float y2) override;

	// get selected software and/or driver
	virtual void get_selection(ui_software_info const *&software, game_driver const *&driver) const override;

	// text for main top/bottom panels
	virtual void make_topbox_text(std::string &line0, std::string &line1, std::string &line2) const override;
	virtual std::string make_driver_description(game_driver const &driver) const override;
	virtual std::string make_software_description(ui_software_info const &software) const override;

	// internal methods
	void build_custom();
	void build_category();
	void build_available_list();
	void build_list(const char *filter_text = nullptr, int filter = 0, bool bioscheck = false, std::vector<const game_driver *> vec = {});

	bool isfavorite() const;
	void populate_search();
	void init_sorted_list();
	bool load_available_machines();
	void load_custom_filters();

	static std::string make_error_text(bool summary, media_auditor const &auditor);

	void *get_selection_ptr() const
	{
		void *const selected_ref(get_selection_ref());
		return (uintptr_t(selected_ref) > skip_main_items) ? selected_ref : m_prev_selected;
	}

	// General info
	virtual void general_info(const game_driver *driver, std::string &buffer) override;

	// handlers
	void inkey_select(const event *menu_event);
	void inkey_select_favorite(const event *menu_event);
	void inkey_special(const event *menu_event);
	void inkey_export();
};

} // namespace ui

#endif  // MAME_FRONTEND_UI_SELGAME_H
