// license:BSD-3-Clause
// copyright-holders:Nathan Woods
/***************************************************************************

    charconv.cpp

    Imgtool character set conversion routines.

***************************************************************************/

#include "corestr.h"
#include "charconv.h"
#include "unicode.h"
#include "coretmpl.h"

imgtool::simple_charconverter imgtool::charconverter_iso_8859_1(nullptr, nullptr);


//-------------------------------------------------
//  from_utf8
//-------------------------------------------------

void imgtool::charconverter::from_utf8(std::ostream &dest, const std::string &src) const
{
	from_utf8(dest, src.c_str(), src.size());
}


//-------------------------------------------------
//  to_utf8
//-------------------------------------------------

void imgtool::charconverter::to_utf8(std::ostream &dest, const std::string &src) const
{
	to_utf8(dest, src.c_str(), src.size());
}


//-------------------------------------------------
//  simple_charconverter::simple_charconverter
//-------------------------------------------------

imgtool::simple_charconverter::simple_charconverter(const char32_t lowpage[0x80], const char32_t highpage[0x80], unicode_normalization_form norm)
	: m_norm(norm), m_lowpage(lowpage), m_highpage(highpage)
{
	// build the reverse lookup table
	for (int i = 0; i < 256; i++)
	{
		const char32_t *page = i >= 128 ? m_highpage : m_lowpage;
		char32_t unicode_char = page ? page[i % 128] : i;
		m_reverse_lookup.emplace_back(unicode_char, (char)i);
	}

	// and sort it
	std::sort(m_reverse_lookup.begin(), m_reverse_lookup.end(), [](const std::pair<char32_t, char> &a, const std::pair<char32_t, char> &b)
	{
		return b.first > a.first;
	});
}


//-------------------------------------------------
//  from_utf8
//-------------------------------------------------

void imgtool::simple_charconverter::from_utf8(std::ostream &dest, const char *src, size_t src_length) const
{
	// normalize the incoming unicode
	std::string normalized_src = normalize_unicode(src, src_length, m_norm);

	auto iter = normalized_src.begin();
	while (iter != normalized_src.end())
	{
		// get the next character
		char32_t ch;
		int rc = uchar_from_utf8(&ch, &*iter, normalized_src.end() - iter);
		if (rc < 0)
		{
			ch = 0xFFFD;
			rc = 1;
		}
		iter += rc;

		// do the reverse lookup
		auto lookup = std::lower_bound(m_reverse_lookup.begin(), m_reverse_lookup.end(), ch, [](const std::pair<char32_t, char> &a, const char32_t &b)
		{
			return a.first < b;
		});
		if (lookup == m_reverse_lookup.end())
			throw charconverter_exception();

		// and output the results
		dest << lookup->second;
	}
}


//-------------------------------------------------
//  to_utf8
//-------------------------------------------------

void imgtool::simple_charconverter::to_utf8(std::ostream &dest, const char *src, size_t src_length) const
{
	for (size_t i = 0; i < src_length; i++)
	{
		// which page is this in?
		const char32_t *page = ((src[i] & 0x80) == 0) ? m_lowpage : m_highpage;

		// is this page present?
		if ((src[i] & 0x80) == 0)
		{
			// no - pass it on
			dest << src[i];
		}
		else
		{
			// yes - we need to do a lookup
			size_t base = ((src[i] & 0x80) == 0) ? 0x00 : 0x80;
			char32_t ch = page[((unsigned char)(src[i])) - base];
			if (ch == 0)
				throw charconverter_exception();

			dest << utf8_from_uchar(ch);
		}
	}
}
