// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * poptions.cpp
 *
 */

#include "poptions.h"

namespace plib {
/***************************************************************************
    Options
***************************************************************************/

	option_base::option_base(options &parent, pstring help)
	: m_help(help)
	{
		parent.register_option(this);
	}

	option_base::~option_base()
	{
	}

	option_group::~option_group()
	{
	}

	option_example::~option_example()
	{
	}

	option::option(options &parent, pstring ashort, pstring along, pstring help, bool has_argument)
	: option_base(parent, help), m_short(ashort), m_long(along),
	  m_has_argument(has_argument), m_specified(false)
	{
	}

	option::~option()
	{
	}

	int option_str::parse(const pstring &argument)
	{
		m_val = argument;
		return 0;
	}

	int option_str_limit::parse(const pstring &argument)
	{
		if (plib::container::contains(m_limit, argument))
		{
			m_val = argument;
			return 0;
		}
		else
			return 1;
	}

	int option_bool::parse(const pstring &argument)
	{
		m_val = true;
		return 0;
	}

	int option_double::parse(const pstring &argument)
	{
		bool err = false;
		m_val = argument.as_double(&err);
		return (err ? 1 : 0);
	}

	int option_long::parse(const pstring &argument)
	{
		bool err = false;
		m_val = argument.as_long(&err);
		return (err ? 1 : 0);
	}

	int option_vec::parse(const pstring &argument)
	{
		bool err = false;
		m_val.push_back(argument);
		return (err ? 1 : 0);
	}

	options::options()
	{
	}

	options::options(option *o[])
	{
		int i=0;
		while (o[i] != nullptr)
		{
			m_opts.push_back(o[i]);
			i++;
		}
	}

	options::~options()
	{
		m_opts.clear();
	}

	void options::register_option(option_base *opt)
	{
		m_opts.push_back(opt);
	}

	int options::parse(int argc, char *argv[])
	{
		m_app = pstring(argv[0], pstring::UTF8);

		for (int i=1; i<argc; )
		{
			pstring arg(argv[i], pstring::UTF8);
			option *opt = nullptr;
			pstring opt_arg;
			bool has_equal_arg = false;

			if (arg.startsWith("--"))
			{
				auto v = psplit(arg.substr(2),"=");
				opt = getopt_long(v[0]);
				has_equal_arg = (v.size() > 1);
				if (has_equal_arg)
				{
					for (unsigned j = 1; j < v.size() - 1; j++)
						opt_arg = opt_arg + v[j] + "=";
					opt_arg += v[v.size()-1];
				}
			}
			else if (arg.startsWith("-"))
			{
				std::size_t p = 1;
				opt = getopt_short(arg.substr(p, 1));
				++p;
				if (p < arg.length())
				{
					has_equal_arg = true;
					opt_arg = arg.substr(p);
				}
			}
			else
			{
				return i;
			}
			if (opt == nullptr)
				return i;
			if (opt->has_argument())
			{
				if (has_equal_arg)
				{
					if (opt->do_parse(opt_arg) != 0)
						return i;
				}
				else
				{
					i++; // FIXME: are there more arguments?
					if (opt->do_parse(pstring(argv[i], pstring::UTF8)) != 0)
						return i - 1;
				}
			}
			else
			{
				if (has_equal_arg)
					return i;
				opt->do_parse("");
			}
			i++;
		}
		return argc;
	}

	pstring options::split_paragraphs(pstring text, unsigned width, unsigned indent,
			unsigned firstline_indent)
	{
		auto paragraphs = psplit(text,"\n");
		pstring ret("");

		for (auto &p : paragraphs)
		{
			pstring line = pstring("").rpad(" ", firstline_indent);
			for (auto &s : psplit(p, " "))
			{
				if (line.length() + s.length() > width)
				{
					ret += line + "\n";
					line = pstring("").rpad(" ", indent);
				}
				line += s + " ";
			}
			ret += line + "\n";
		}
		return ret;
	}

	pstring options::help(pstring description, pstring usage,
			unsigned width, unsigned indent)
	{
		pstring ret;

		ret = split_paragraphs(description, width, 0, 0) + "\n";
		ret += "Usage:\t" + usage + "\n\nOptions:\n\n";

		for (auto & optbase : m_opts )
		{
			if (auto opt = dynamic_cast<option *>(optbase))
			{
				pstring line = "";
				if (opt->short_opt() != "")
					line += "  -" + opt->short_opt();
				if (opt->long_opt() != "")
				{
					if (line != "")
						line += ", ";
					else
						line = "      ";
					line += "--" + opt->long_opt();
					if (opt->has_argument())
					{
						line += "=";
						option_str_limit *ol = dynamic_cast<option_str_limit *>(opt);
						if (ol)
						{
							for (auto &v : ol->limit())
							{
								line += v + "|";
							}
							line = line.left(line.length() - 1);
						}
						else
							line += "Value";
					}
				}
				line = line.rpad(" ", indent - 2) + "  ";
				if (line.length() > indent)
				{
					//ret += "TestGroup abc\n  def gef\nxyz\n\n" ;
					ret += line + "\n";
					ret += split_paragraphs(opt->help(), width, indent, indent);
				}
				else
					ret += split_paragraphs(line + opt->help(), width, indent, 0);
			}
			else if (auto grp = dynamic_cast<option_group *>(optbase))
			{
				ret += "\n" + grp->group() + ":\n";
				if (grp->help() != "") ret += split_paragraphs(grp->help(), width, 4, 4) + "\n";
			}
		}
		pstring ex("");
		for (auto & optbase : m_opts )
		{
			if (auto example = dynamic_cast<option_example *>(optbase))
			{
				ex += "> " + example->example()+"\n\n";
				ex += split_paragraphs(example->help(), width, 4, 4) + "\n";
			}
		}
		if (ex.length() > 0)
		{
			ret += "\n\nExamples:\n\n" + ex;
		}
		return ret;
	}

	option *options::getopt_short(pstring arg)
	{
		for (auto & optbase : m_opts)
		{
			auto opt = dynamic_cast<option *>(optbase);
			if (opt && opt->short_opt() == arg)
				return opt;
		}
		return nullptr;
	}
	option *options::getopt_long(pstring arg)
	{
		for (auto & optbase : m_opts)
		{
			auto opt = dynamic_cast<option *>(optbase);
			if (opt && opt->long_opt() == arg)
				return opt;
		}
		return nullptr;
	}

} // namespace plib

