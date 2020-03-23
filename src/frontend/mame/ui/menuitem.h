// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria, Aaron Giles, Nathan Woods

/***************************************************************************

    ui/menuitem.h

    Internal data representation for a UI menu item.

***************************************************************************/

#pragma once

#ifndef MAME_FRONTEND_UI_MENUITEM_H
#define MAME_FRONTEND_UI_MENUITEM_H


namespace ui {
// special menu item for separators
#define MENU_SEPARATOR_ITEM         "---"

// types of menu items (TODO: please expand)
enum class menu_item_type
{
	UNKNOWN,
	SLIDER,
	SEPARATOR
};

class menu_item
{
public:
	menu_item() = default;
	menu_item(menu_item const &) = default;
	menu_item(menu_item &&) = default;
	menu_item &operator=(menu_item const &) = default;
	menu_item &operator=(menu_item &&) = default;

	std::string     text;
	std::string     subtext;
	uint32_t          flags;
	void            *ref;
	menu_item_type  type;   // item type (eventually will go away when itemref is proper ui_menu_item class rather than void*)
};

} // namespace ui

#endif // MAME_FRONTEND_UI_MENUITEM_H
