// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * nl_string.c
 *
 */

#include "pfmtlog.h"
#include "palloc.h"

#include <cstring>
#include <cstdlib>
#include <cstdarg>
#include <algorithm>
#include <locale>
#include <iostream>

namespace plib {

pfmt &pfmt::format_element(const char *l, const unsigned cfmt_spec,  ...)
{
	va_list ap;
	va_start(ap, cfmt_spec);
	pstring fmt("%");
	char buf[2048]; // FIXME
	std::size_t sl;

	m_arg++;

	pstring search("{");
	search += plib::to_string(m_arg);
	sl = search.length();

	auto p = m_str.find(search + ":");
	sl++; // ":"
	if (p == pstring::npos) // no further specifiers
	{
		p = m_str.find(search + "}");
		if (p == pstring::npos) // not found try default
		{
			sl = 2;
			p = m_str.find("{}");
		}
		if (p == pstring::npos)
		{
			sl=1;
			p = m_str.find("{");
			if (p != pstring:: npos)
			{
				auto p1 = m_str.find("}", p);
				if (p1 != pstring::npos)
				{
					sl = p1 - p + 1;
					fmt += m_str.substr(p+1, p1 - p - 1);
				}
			}
		}
	}
	else
	{
		auto p1 = m_str.find("}", p);
		if (p1 != pstring::npos)
		{
			sl = p1 - p + 1;
			fmt += ((m_arg>=10) ? m_str.substr(p+4, p1 - p - 4) : m_str.substr(p+3, p1 - p - 3));
		}
	}
	pstring::code_t pend = fmt.at(fmt.length() - 1);
	if (pstring("duxo").find(cfmt_spec) != pstring::npos)
	{
		if (pstring("duxo").find(pend) == pstring::npos)
			fmt += (pstring(l, pstring::UTF8) + cfmt_spec);
		else
			fmt = fmt.left(fmt.length() - 1) + pstring(l, pstring::UTF8) + fmt.right(1);
	}
	else if (pstring("fge").find(cfmt_spec) != pstring::npos)
	{
		if (pstring("fge").find(pend) == pstring::npos)
			fmt += cfmt_spec;
	}
	else
		fmt += cfmt_spec;
	vsprintf(buf, fmt.c_str(), ap);
	if (p != pstring::npos)
		m_str = m_str.substr(0, p) + pstring(buf, pstring::UTF8) + m_str.substr(p + sl);
	va_end(ap);
	return *this;
}

}
