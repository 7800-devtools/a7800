// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria, Aaron Giles, Nathan Woods
/***************************************************************************

    text.h

    Text functionality for MAME's crude user interface

***************************************************************************/

#pragma once

#ifndef MAME_FRONTEND_UI_TEXT_H
#define MAME_FRONTEND_UI_TEXT_H

#include "palette.h"
#include "unicode.h"

class render_font;
class render_container;

namespace ui {
/***************************************************************************
TYPE DEFINITIONS
***************************************************************************/

class text_layout
{
public:
	// justification options for text
	enum text_justify
	{
		LEFT = 0,
		CENTER,
		RIGHT
	};

	// word wrapping options
	enum word_wrapping
	{
		NEVER,
		TRUNCATE,
		WORD
	};

	// ctor/dtor
	text_layout(render_font &font, float xscale, float yscale, float width, text_justify justify, word_wrapping wrap);
	text_layout(text_layout &&that);
	~text_layout();

	// accessors
	render_font &font() const { return m_font; }
	float xscale() const { return m_xscale;  }
	float yscale() const { return m_yscale; }
	float width() const { return m_width; }
	text_justify justify() const { return m_justify; }
	word_wrapping wrap() const { return m_wrap; }

	// methods
	float actual_left() const;
	float actual_width() const;
	float actual_height() const;
	bool empty() const { return m_lines.size() == 0; }
	bool hit_test(float x, float y, size_t &start, size_t &span) const;
	void restyle(size_t start, size_t span, rgb_t *fgcolor, rgb_t *bgcolor);
	int get_wrap_info(std::vector<int> &xstart, std::vector<int> &xend) const;
	void emit(render_container &container, float x, float y);
	void add_text(const char *text, rgb_t fgcolor = rgb_t::white(), rgb_t bgcolor = rgb_t::transparent(), float size = 1.0)
	{
		// create the style
		char_style style = { 0, };
		style.fgcolor = fgcolor;
		style.bgcolor = bgcolor;
		style.size = size;

		// and add the text
		add_text(text, style);
	}

private:
	// text style information - in a struct to facilitate copying
	struct char_style
	{
		rgb_t fgcolor;
		rgb_t bgcolor;
		float size;
	};

	// information about the "source" of a character - also in a struct
	// to facilitate copying
	struct source_info
	{
		size_t start;
		size_t span;
	};

	// this should really be "positioned glyph" as glyphs != characters, but
	// we'll get there eventually
	struct positioned_char
	{
		char32_t character;
		char_style style;
		source_info source;
		float xoffset;
		float xwidth;
	};

	// class to represent a line
	class line
	{
	public:
		line(text_layout &layout, text_justify justify, float yoffset, float height);

		// methods
		void add_character(char32_t ch, const char_style &style, const source_info &source);
		void truncate(size_t position);

		// accessors
		float xoffset() const;
		float yoffset() const { return m_yoffset; }
		float width() const { return m_width; }
		float height() const { return m_height; }
		text_justify justify() const { return m_justify; }
		size_t character_count() const { return m_characters.size(); }
		const positioned_char &character(size_t index) const { return m_characters[index]; }
		positioned_char &character(size_t index) { return m_characters[index]; }

	private:
		std::vector<positioned_char> m_characters;
		text_layout &m_layout;
		text_justify m_justify;
		float m_yoffset;
		float m_width;
		float m_height;
	};

	// instance variables
	render_font &m_font;
	float m_xscale;
	float m_yscale;
	float m_width;
	mutable float m_calculated_actual_width;
	text_justify m_justify;
	word_wrapping m_wrap;
	std::vector<std::unique_ptr<line>> m_lines;
	line *m_current_line;
	size_t m_last_break;
	size_t m_text_position;
	bool m_truncating;

	// methods
	void add_text(const char *text, const char_style &style);
	void start_new_line(text_justify justify, float height);
	float get_char_width(char32_t ch, float size);
	void truncate_wrap();
	void word_wrap();
	void invalidate_calculated_actual_width();
};

} // namespace ui

#endif  // MAME_FRONTEND_UI_TEXT_H
