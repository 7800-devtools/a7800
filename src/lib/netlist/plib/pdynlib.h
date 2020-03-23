// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * pdynlib.h
 */

#ifndef PDYNLIB_H_
#define PDYNLIB_H_

#include "pstring.h"

namespace plib {
// ----------------------------------------------------------------------------------------
// pdynlib: dynamic loading of libraries  ...
// ----------------------------------------------------------------------------------------

class dynlib
{
public:
	explicit dynlib(const pstring libname);
	dynlib(const pstring path, const pstring libname);
	~dynlib();

	bool isLoaded() const;

	template <typename T>
	T getsym(const pstring name)
	{
		return reinterpret_cast<T>(getsym_p(name));
	}
private:
	void *getsym_p(const pstring name);

	bool m_isLoaded;
	void *m_lib;
};

template <typename R, typename... Args>
class dynproc
{
public:
	using calltype = R(*) (Args... args);

	dynproc() : m_sym(nullptr) { }

	dynproc(dynlib &dl, const pstring &name)
	{
		m_sym = dl.getsym<calltype>(name);
	}

	void load(dynlib &dl, const pstring &name)
	{
		m_sym = dl.getsym<calltype>(name);
	}

	R operator ()(Args&&... args) const
	{
		return m_sym(std::forward<Args>(args)...);
		//return m_sym(args...);
	}

	bool resolved() { return m_sym != nullptr; }
private:
	calltype m_sym;
};

}

#endif /* PSTRING_H_ */
