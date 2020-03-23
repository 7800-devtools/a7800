// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * poptions.h
 *
 */

#pragma once

#ifndef POPTIONS_H_
#define POPTIONS_H_

#include "pstring.h"
#include "plists.h"
#include "putil.h"

namespace plib {
/***************************************************************************
    Options
***************************************************************************/

class options;

class option_base
{
public:
	option_base(options &parent, pstring help);
	virtual ~option_base();

	pstring help() { return m_help; }
private:
	pstring m_help;
};

class option_group : public option_base
{
public:
	option_group(options &parent, pstring group, pstring help)
	: option_base(parent, help), m_group(group) { }
	~option_group();

	pstring group() { return m_group; }
private:
	pstring m_group;
};

class option_example : public option_base
{
public:
	option_example(options &parent, pstring group, pstring help)
	: option_base(parent, help), m_example(group) { }
	~option_example();

	pstring example() { return m_example; }
private:
	pstring m_example;
};


class option : public option_base
{
public:
	option(options &parent, pstring ashort, pstring along, pstring help, bool has_argument);
	~option();

	/* no_argument options will be called with "" argument */

	pstring short_opt() { return m_short; }
	pstring long_opt() { return m_long; }
	bool has_argument() { return m_has_argument ; }
	bool was_specified() { return m_specified; }

	int do_parse(const pstring &argument)
	{
		m_specified = true;
		return parse(argument);
	}

protected:
	virtual int parse(const pstring &argument) = 0;

private:
	pstring m_short;
	pstring m_long;
	bool m_has_argument;
	bool m_specified;
};

class option_str : public option
{
public:
	option_str(options &parent, pstring ashort, pstring along, pstring defval, pstring help)
	: option(parent, ashort, along, help, true), m_val(defval)
	{}

	pstring operator ()() { return m_val; }

protected:
	virtual int parse(const pstring &argument) override;

private:
	pstring m_val;
};

class option_str_limit : public option
{
public:
	option_str_limit(options &parent, pstring ashort, pstring along, pstring defval, pstring limit, pstring help)
	: option(parent, ashort, along, help, true), m_val(defval)
	, m_limit(plib::psplit(limit, ":"))
	{
	}

	pstring operator ()() { return m_val; }
	const std::vector<pstring> &limit() { return m_limit; }

protected:
	virtual int parse(const pstring &argument) override;

private:
	pstring m_val;
	std::vector<pstring> m_limit;
};

class option_bool : public option
{
public:
	option_bool(options &parent, pstring ashort, pstring along, pstring help)
	: option(parent, ashort, along, help, false), m_val(false)
	{}

	bool operator ()() { return m_val; }

protected:
	virtual int parse(const pstring &argument) override;

private:
	bool m_val;
};

class option_double : public option
{
public:
	option_double(options &parent, pstring ashort, pstring along, double defval, pstring help)
	: option(parent, ashort, along, help, true), m_val(defval)
	{}

	double operator ()() { return m_val; }

protected:
	virtual int parse(const pstring &argument) override;

private:
	double m_val;
};

class option_long : public option
{
public:
	option_long(options &parent, pstring ashort, pstring along, long defval, pstring help)
	: option(parent, ashort, along, help, true), m_val(defval)
	{}

	long operator ()() { return m_val; }

protected:
	virtual int parse(const pstring &argument) override;

private:
	long m_val;
};

class option_vec : public option
{
public:
	option_vec(options &parent, pstring ashort, pstring along, pstring help)
	: option(parent, ashort, along, help, true)
	{}

	std::vector<pstring> operator ()() { return m_val; }

protected:
	virtual int parse(const pstring &argument) override;

private:
	std::vector<pstring> m_val;
};

class options
{
public:

	options();
	explicit options(option *o[]);

	~options();

	void register_option(option_base *opt);
	int parse(int argc, char *argv[]);

	pstring help(pstring description, pstring usage,
			unsigned width = 72, unsigned indent = 20);

	pstring app() { return m_app; }

private:
	static pstring split_paragraphs(pstring text, unsigned width, unsigned indent,
			unsigned firstline_indent);

	option *getopt_short(pstring arg);
	option *getopt_long(pstring arg);

	std::vector<option_base *> m_opts;
	pstring m_app;
};

}

#endif /* POPTIONS_H_ */
