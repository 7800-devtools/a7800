// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * palloc.c
 *
 */

#include "pexception.h"
#include "pfmtlog.h"

#include <cfenv>

#if (defined(__x86_64__) || defined(__i386__)) && defined(__linux__)
#define HAS_FEENABLE_EXCEPT     (1)
#else
#define HAS_FEENABLE_EXCEPT     (0)
#endif

namespace plib {
//============================================================
//  Exceptions
//============================================================

pexception::pexception(const pstring &text)
: m_text(text)
{
}

pexception::~pexception() noexcept
{
}

file_e::file_e(const pstring &fmt, const pstring &filename)
	: pexception(pfmt(fmt)(filename))
{
}

file_e::~file_e() noexcept
{
}

file_open_e::file_open_e(const pstring &filename)
	: file_e("File open failed: {}", filename)
{
}

file_open_e::~file_open_e() noexcept
{

}

file_read_e::file_read_e(const pstring &filename)
	: file_e("File read failed: {}", filename)
{
}

file_read_e::~file_read_e() noexcept
{

}

file_write_e::file_write_e(const pstring &filename)
	: file_e("File write failed: {}", filename)
{
}

file_write_e::~file_write_e() noexcept
{
}

null_argument_e::null_argument_e(const pstring &argument)
	: pexception(pfmt("Null argument passed: {}")(argument))
{
}

null_argument_e::~null_argument_e() noexcept
{
}

out_of_mem_e::out_of_mem_e(const pstring &location)
	: pexception(pfmt("Out of memory: {}")(location))
{
}

out_of_mem_e::~out_of_mem_e() noexcept
{
}

fpexception_e::fpexception_e(const pstring &text)
	: pexception(pfmt("Out of memory: {}")(text))
{
}

fpexception_e::~fpexception_e() noexcept
{
}

bool fpsignalenabler::m_enable = false;

fpsignalenabler::fpsignalenabler(unsigned fpexceptions)
{
#if HAS_FEENABLE_EXCEPT
	if (m_enable)
	{
		int b = 0;
		if (fpexceptions & plib::FP_INEXACT) b = b | FE_INEXACT;
		if (fpexceptions & plib::FP_DIVBYZERO) b = b | FE_DIVBYZERO;
		if (fpexceptions & plib::FP_UNDERFLOW) b = b | FE_UNDERFLOW;
		if (fpexceptions & plib::FP_OVERFLOW) b = b | FE_OVERFLOW;
		if (fpexceptions & plib::FP_INVALID) b = b | FE_INVALID;
		m_last_enabled = feenableexcept(b);
	}
#else
	m_last_enabled = 0;
#endif
}


fpsignalenabler::~fpsignalenabler()
{
#if HAS_FEENABLE_EXCEPT
	if (m_enable)
	{
		fedisableexcept(FE_ALL_EXCEPT);  // Enable all floating point exceptions but FE_INEXACT
		feenableexcept(m_last_enabled);  // Enable all floating point exceptions but FE_INEXACT
	}
#endif
}

bool fpsignalenabler::supported()
{
	return true;
}

bool fpsignalenabler::global_enable(bool enable)
{
	bool old = m_enable;
	m_enable = enable;
	return old;
}


}
