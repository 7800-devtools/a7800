// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * nlsetup.c
 *
 */

#include "plib/palloc.h"
#include "plib/putil.h"
#include "nl_base.h"
#include "nl_setup.h"
#include "nl_parser.h"
#include "nl_factory.h"
#include "devices/nlid_system.h"
#include "devices/nlid_proxy.h"
#include "analog/nld_twoterm.h"
#include "solver/nld_solver.h"
#include "devices/nlid_truthtable.h"

// ----------------------------------------------------------------------------------------
// setup_t
// ----------------------------------------------------------------------------------------

namespace netlist
{
setup_t::setup_t(netlist_t &netlist)
	: m_netlist(netlist)
	, m_factory(*this)
	, m_proxy_cnt(0)
	, m_frontier_cnt(0)
{
	devices::initialize_factory(m_factory);
}

setup_t::~setup_t()
{
	m_links.clear();
	m_alias.clear();
	m_params.clear();
	m_terminals.clear();
	m_param_values.clear();

	m_sources.clear();
}

pstring setup_t::build_fqn(const pstring &obj_name) const
{
	if (m_namespace_stack.empty())
		//return netlist().name() + "." + obj_name;
		return obj_name;
	else
		return m_namespace_stack.top() + "." + obj_name;
}

void setup_t::namespace_push(const pstring &aname)
{
	if (m_namespace_stack.empty())
		//m_namespace_stack.push(netlist().name() + "." + aname);
		m_namespace_stack.push(aname);
	else
		m_namespace_stack.push(m_namespace_stack.top() + "." + aname);
}

void setup_t::namespace_pop()
{
	m_namespace_stack.pop();
}

void setup_t::register_lib_entry(const pstring &name, const pstring &sourcefile)
{
	factory().register_device(plib::make_unique_base<factory::element_t, factory::library_element_t>(*this, name, name, "", sourcefile));
}

void setup_t::register_dev(const pstring &classname, const pstring &name)
{
	auto f = factory().factory_by_name(classname);
	if (f == nullptr)
		log().fatal(MF_1_CLASS_1_NOT_FOUND, classname);
	/* make sure we parse macro library entries */
	f->macro_actions(netlist(), name);
	m_device_factory.push_back(std::pair<pstring, factory::element_t *>(build_fqn(name), f));
}

bool setup_t::device_exists(const pstring &name) const
{
	for (auto e : m_device_factory)
	{
		if (e.first == name)
			return true;
	}
	return false;
}


void setup_t::register_model(const pstring &model_in)
{
	auto pos = model_in.find(" ");
	if (pos == pstring::npos)
		log().fatal(MF_1_UNABLE_TO_PARSE_MODEL_1, model_in);
	pstring model = model_in.left(pos).trim().ucase();
	pstring def = model_in.substr(pos + 1).trim();
	if (!m_models.insert({model, def}).second)
		log().fatal(MF_1_MODEL_ALREADY_EXISTS_1, model_in);
}

void setup_t::register_alias_nofqn(const pstring &alias, const pstring &out)
{
	if (!m_alias.insert({alias, out}).second)
		log().fatal(MF_1_ADDING_ALI1_TO_ALIAS_LIST, alias);
}

void setup_t::register_alias(const pstring &alias, const pstring &out)
{
	pstring alias_fqn = build_fqn(alias);
	pstring out_fqn = build_fqn(out);
	register_alias_nofqn(alias_fqn, out_fqn);
}

void setup_t::register_dippins_arr(const pstring &terms)
{
	std::vector<pstring> list(plib::psplit(terms,", "));
	if (list.size() == 0 || (list.size() % 2) == 1)
		log().fatal(MF_1_DIP_PINS_MUST_BE_AN_EQUAL_NUMBER_OF_PINS_1,
				build_fqn(""));
	std::size_t n = list.size();
	for (std::size_t i = 0; i < n / 2; i++)
	{
		register_alias(plib::pfmt("{1}")(i+1), list[i * 2]);
		register_alias(plib::pfmt("{1}")(n-i), list[i * 2 + 1]);
	}
}

pstring setup_t::termtype_as_str(detail::core_terminal_t &in) const
{
	switch (in.type())
	{
		case detail::terminal_type::TERMINAL:
			return pstring("TERMINAL");
		case detail::terminal_type::INPUT:
			return pstring("INPUT");
		case detail::terminal_type::OUTPUT:
			return pstring("OUTPUT");
	}
	log().fatal(MF_1_UNKNOWN_OBJECT_TYPE_1, static_cast<unsigned>(in.type()));
	return pstring("Error");
}

pstring setup_t::get_initial_param_val(const pstring &name, const pstring &def)
{
	auto i = m_param_values.find(name);
	if (i != m_param_values.end())
		return i->second;
	else
		return def;
}

double setup_t::get_initial_param_val(const pstring &name, const double def)
{
	auto i = m_param_values.find(name);
	if (i != m_param_values.end())
	{
		double vald = 0;
		if (sscanf(i->second.c_str(), "%lf", &vald) != 1)
			log().fatal(MF_2_INVALID_NUMBER_CONVERSION_1_2, name, i->second);
		return vald;
	}
	else
		return def;
}

int setup_t::get_initial_param_val(const pstring &name, const int def)
{
	auto i = m_param_values.find(name);
	if (i != m_param_values.end())
	{
		double vald = 0;
		if (sscanf(i->second.c_str(), "%lf", &vald) != 1)
			log().fatal(MF_2_INVALID_NUMBER_CONVERSION_1_2, name, i->second);
		return static_cast<int>(vald);
	}
	else
		return def;
}

void setup_t::register_param(const pstring &name, param_t &param)
{
	if (!m_params.insert({param.name(), param_ref_t(param.name(), param.device(), param)}).second)
		log().fatal(MF_1_ADDING_PARAMETER_1_TO_PARAMETER_LIST, name);
}

void setup_t::register_term(detail::core_terminal_t &term)
{
	if (!m_terminals.insert({term.name(), &term}).second)
		log().fatal(MF_2_ADDING_1_2_TO_TERMINAL_LIST, termtype_as_str(term),
				term.name());
	log().debug("{1} {2}\n", termtype_as_str(term), term.name());
}

void setup_t::register_link_arr(const pstring &terms)
{
	std::vector<pstring> list(plib::psplit(terms,", "));
	if (list.size() < 2)
		log().fatal(MF_2_NET_C_NEEDS_AT_LEAST_2_TERMINAL);
	for (std::size_t i = 1; i < list.size(); i++)
	{
		register_link(list[0], list[i]);
	}
}


void setup_t::register_link_fqn(const pstring &sin, const pstring &sout)
{
	link_t temp = link_t(sin, sout);
	log().debug("link {1} <== {2}\n", sin, sout);
	m_links.push_back(temp);
}

void setup_t::register_link(const pstring &sin, const pstring &sout)
{
	register_link_fqn(build_fqn(sin), build_fqn(sout));
}

void setup_t::remove_connections(const pstring &pin)
{
	pstring pinfn = build_fqn(pin);
	bool found = false;

	for (auto link = m_links.begin(); link != m_links.end(); )
	{
		if ((link->first == pinfn) || (link->second == pinfn))
		{
			log().verbose("removing connection: {1} <==> {2}\n", link->first, link->second);
			link = m_links.erase(link);
			found = true;
		}
		else
			link++;
	}
	if (!found)
		log().fatal(MF_1_FOUND_NO_OCCURRENCE_OF_1, pin);
}


void setup_t::register_frontier(const pstring &attach, const double r_IN, const double r_OUT)
{
	pstring frontier_name = plib::pfmt("frontier_{1}")(m_frontier_cnt);
	m_frontier_cnt++;
	register_dev("FRONTIER_DEV", frontier_name);
	register_param(frontier_name + ".RIN", r_IN);
	register_param(frontier_name + ".ROUT", r_OUT);
	register_link(frontier_name + ".G", "GND");
	pstring attfn = build_fqn(attach);
	pstring front_fqn = build_fqn(frontier_name);
	bool found = false;
	for (auto & link  : m_links)
	{
		if (link.first == attfn)
		{
			link.first = front_fqn + ".I";
			found = true;
		}
		else if (link.second == attfn)
		{
			link.second = front_fqn + ".I";
			found = true;
		}
	}
	if (!found)
		log().fatal(MF_1_FOUND_NO_OCCURRENCE_OF_1, attach);
	register_link(attach, frontier_name + ".Q");
}


void setup_t::register_param(const pstring &param, const double value)
{
	register_param(param, plib::pfmt("{1:.9}").e(value));
}

void setup_t::register_param(const pstring &param, const pstring &value)
{
	pstring fqn = build_fqn(param);

	auto idx = m_param_values.find(fqn);
	if (idx == m_param_values.end())
	{
		if (!m_param_values.insert({fqn, value}).second)
			log().fatal(MF_1_ADDING_PARAMETER_1_TO_PARAMETER_LIST,
					param);
	}
	else
	{
		log().warning(MW_3_OVERWRITING_PARAM_1_OLD_2_NEW_3, fqn, idx->second,
				value);
		m_param_values[fqn] = value;
	}
}

const pstring setup_t::resolve_alias(const pstring &name) const
{
	pstring temp = name;
	pstring ret;

	/* FIXME: Detect endless loop */
	do {
		ret = temp;
		auto p = m_alias.find(ret);
		temp = (p != m_alias.end() ? p->second : "");
	} while (temp != "" && temp != ret);

	log().debug("{1}==>{2}\n", name, ret);
	return ret;
}

detail::core_terminal_t *setup_t::find_terminal(const pstring &terminal_in, bool required)
{
	const pstring &tname = resolve_alias(terminal_in);
	auto ret = m_terminals.find(tname);
	/* look for default */
	if (ret == m_terminals.end())
	{
		/* look for ".Q" std output */
		ret = m_terminals.find(tname + ".Q");
	}

	detail::core_terminal_t *term = (ret == m_terminals.end() ? nullptr : ret->second);

	if (term == nullptr && required)
		log().fatal(MF_2_TERMINAL_1_2_NOT_FOUND, terminal_in, tname);
	if (term != nullptr)
		log().debug("Found input {1}\n", tname);
	return term;
}

detail::core_terminal_t *setup_t::find_terminal(const pstring &terminal_in,
		detail::terminal_type atype, bool required)
{
	const pstring &tname = resolve_alias(terminal_in);
	auto ret = m_terminals.find(tname);
	/* look for default */
	if (ret == m_terminals.end() && atype == detail::terminal_type::OUTPUT)
	{
		/* look for ".Q" std output */
		ret = m_terminals.find(tname + ".Q");
	}
	if (ret == m_terminals.end() && required)
		log().fatal(MF_2_TERMINAL_1_2_NOT_FOUND, terminal_in, tname);

	detail::core_terminal_t *term = (ret == m_terminals.end() ? nullptr : ret->second);

	if (term != nullptr && term->type() != atype)
	{
		if (required)
			log().fatal(MF_2_OBJECT_1_2_WRONG_TYPE, terminal_in, tname);
		else
			term = nullptr;
	}
	if (term != nullptr)
		log().debug("Found input {1}\n", tname);

	return term;
}

param_t *setup_t::find_param(const pstring &param_in, bool required) const
{
	const pstring param_in_fqn = build_fqn(param_in);

	const pstring &outname = resolve_alias(param_in_fqn);
	auto ret = m_params.find(outname);
	if (ret == m_params.end() && required)
		log().fatal(MF_2_PARAMETER_1_2_NOT_FOUND, param_in_fqn, outname);
	if (ret != m_params.end())
		log().debug("Found parameter {1}\n", outname);
	return (ret == m_params.end() ? nullptr : &ret->second.m_param);
}

devices::nld_base_proxy *setup_t::get_d_a_proxy(detail::core_terminal_t &out)
{
	nl_assert(out.is_logic());

	logic_output_t &out_cast = static_cast<logic_output_t &>(out);
	devices::nld_base_proxy *proxy = out_cast.get_proxy();

	if (proxy == nullptr)
	{
		// create a new one ...
		pstring x = plib::pfmt("proxy_da_{1}_{2}")(out.name())(m_proxy_cnt);
		auto new_proxy =
				out_cast.logic_family()->create_d_a_proxy(netlist(), x, &out_cast);
		m_proxy_cnt++;

		//new_proxy->start_dev();

		/* connect all existing terminals to new net */

		for (auto & p : out.net().m_core_terms)
		{
			p->clear_net(); // de-link from all nets ...
			if (!connect(new_proxy->proxy_term(), *p))
				log().fatal(MF_2_CONNECTING_1_TO_2,
						new_proxy->proxy_term().name(), (*p).name());
		}
		out.net().m_core_terms.clear(); // clear the list

		out.net().add_terminal(new_proxy->in());
		out_cast.set_proxy(proxy);

		proxy = new_proxy.get();

		netlist().register_dev(std::move(new_proxy));
	}
	return proxy;
}

devices::nld_base_proxy *setup_t::get_a_d_proxy(detail::core_terminal_t &inp)
{
	nl_assert(inp.is_logic());

	logic_input_t &incast = dynamic_cast<logic_input_t &>(inp);
	devices::nld_base_proxy *proxy = incast.get_proxy();

	if (proxy != nullptr)
		return proxy;
	else
	{
		log().debug("connect_terminal_input: connecting proxy\n");
		pstring x = plib::pfmt("proxy_ad_{1}_{2}")(inp.name())(m_proxy_cnt);
		auto new_proxy = incast.logic_family()->create_a_d_proxy(netlist(), x, &incast);
		//auto new_proxy = plib::owned_ptr<devices::nld_a_to_d_proxy>::Create(netlist(), x, &incast);
		incast.set_proxy(new_proxy.get());
		m_proxy_cnt++;

		auto ret = new_proxy.get();

		/* connect all existing terminals to new net */

		if (inp.has_net())
		{
			for (auto & p : inp.net().m_core_terms)
			{
				p->clear_net(); // de-link from all nets ...
				if (!connect(ret->proxy_term(), *p))
					log().fatal(MF_2_CONNECTING_1_TO_2,
							ret->proxy_term().name(), (*p).name());
			}
			inp.net().m_core_terms.clear(); // clear the list
		}
		ret->out().net().add_terminal(inp);
		netlist().register_dev(std::move(new_proxy));
		return ret;
	}
}

void setup_t::merge_nets(detail::net_t &thisnet, detail::net_t &othernet)
{
	log().debug("merging nets ...\n");
	if (&othernet == &thisnet)
	{
		log().warning(MW_1_CONNECTING_1_TO_ITSELF, thisnet.name());
		return; // Nothing to do
	}

	if (thisnet.isRailNet() && othernet.isRailNet())
		log().fatal(MF_2_MERGE_RAIL_NETS_1_AND_2,
				thisnet.name(), othernet.name());

	if (othernet.isRailNet())
	{
		log().debug("othernet is railnet\n");
		merge_nets(othernet, thisnet);
	}
	else
	{
		othernet.move_connections(thisnet);
	}
}



void setup_t::connect_input_output(detail::core_terminal_t &in, detail::core_terminal_t &out)
{
	if (out.is_analog() && in.is_logic())
	{
		auto proxy = get_a_d_proxy(in);

		out.net().add_terminal(proxy->proxy_term());
	}
	else if (out.is_logic() && in.is_analog())
	{
		devices::nld_base_proxy *proxy = get_d_a_proxy(out);

		connect_terminals(proxy->proxy_term(), in);
		//proxy->out().net().register_con(in);
	}
	else
	{
		if (in.has_net())
			merge_nets(out.net(), in.net());
		else
			out.net().add_terminal(in);
	}
}


void setup_t::connect_terminal_input(terminal_t &term, detail::core_terminal_t &inp)
{
	if (inp.is_analog())
	{
		connect_terminals(inp, term);
	}
	else if (inp.is_logic())
	{
		log().verbose("connect terminal {1} (in, {2}) to {3}\n", inp.name(),
				inp.is_analog() ? pstring("analog") : inp.is_logic() ? pstring("logic") : pstring("?"), term.name());
		auto proxy = get_a_d_proxy(inp);

		//out.net().register_con(proxy->proxy_term());
		connect_terminals(term, proxy->proxy_term());

	}
	else
	{
		log().fatal(MF_1_OBJECT_INPUT_TYPE_1, inp.name());
	}
}

void setup_t::connect_terminal_output(terminal_t &in, detail::core_terminal_t &out)
{
	if (out.is_analog())
	{
		log().debug("connect_terminal_output: {1} {2}\n", in.name(), out.name());
		/* no proxy needed, just merge existing terminal net */
		if (in.has_net())
			merge_nets(out.net(), in.net());
		else
			out.net().add_terminal(in);
	}
	else if (out.is_logic())
	{
		log().debug("connect_terminal_output: connecting proxy\n");
		devices::nld_base_proxy *proxy = get_d_a_proxy(out);

		connect_terminals(proxy->proxy_term(), in);
	}
	else
	{
		log().fatal(MF_1_OBJECT_OUTPUT_TYPE_1, out.name());
	}
}

void setup_t::connect_terminals(detail::core_terminal_t &t1, detail::core_terminal_t &t2)
{
	if (t1.has_net() && t2.has_net())
	{
		log().debug("T2 and T1 have net\n");
		merge_nets(t1.net(), t2.net());
	}
	else if (t2.has_net())
	{
		log().debug("T2 has net\n");
		t2.net().add_terminal(t1);
	}
	else if (t1.has_net())
	{
		log().debug("T1 has net\n");
		t1.net().add_terminal(t2);
	}
	else
	{
		log().debug("adding analog net ...\n");
		// FIXME: Nets should have a unique name
		auto anet = plib::palloc<analog_net_t>(netlist(),"net." + t1.name());
		netlist().m_nets.push_back(plib::owned_ptr<analog_net_t>(anet, true));
		t1.set_net(anet);
		anet->add_terminal(t2);
		anet->add_terminal(t1);
	}
}

static detail::core_terminal_t &resolve_proxy(detail::core_terminal_t &term)
{
	if (term.is_logic())
	{
		logic_t &out = dynamic_cast<logic_t &>(term);
		if (out.has_proxy())
			return out.get_proxy()->proxy_term();
	}
	return term;
}

bool setup_t::connect_input_input(detail::core_terminal_t &t1, detail::core_terminal_t &t2)
{
	bool ret = false;
	if (t1.has_net())
	{
		if (t1.net().isRailNet())
			ret = connect(t2, t1.net().railterminal());
		if (!ret)
		{
			for (auto & t : t1.net().m_core_terms)
			{
				if (t->is_type(detail::terminal_type::TERMINAL))
					ret = connect(t2, *t);
				if (ret)
					break;
			}
		}
	}
	if (!ret && t2.has_net())
	{
		if (t2.net().isRailNet())
			ret = connect(t1, t2.net().railterminal());
		if (!ret)
		{
			for (auto & t : t2.net().m_core_terms)
			{
				if (t->is_type(detail::terminal_type::TERMINAL))
					ret = connect(t1, *t);
				if (ret)
					break;
			}
		}
	}
	return ret;
}



bool setup_t::connect(detail::core_terminal_t &t1_in, detail::core_terminal_t &t2_in)
{
	log().debug("Connecting {1} to {2}\n", t1_in.name(), t2_in.name());
	detail::core_terminal_t &t1 = resolve_proxy(t1_in);
	detail::core_terminal_t &t2 = resolve_proxy(t2_in);
	bool ret = true;

	if (t1.is_type(detail::terminal_type::OUTPUT) && t2.is_type(detail::terminal_type::INPUT))
	{
		if (t2.has_net() && t2.net().isRailNet())
			log().fatal(MF_1_INPUT_1_ALREADY_CONNECTED, t2.name());
		connect_input_output(t2, t1);
	}
	else if (t1.is_type(detail::terminal_type::INPUT) && t2.is_type(detail::terminal_type::OUTPUT))
	{
		if (t1.has_net()  && t1.net().isRailNet())
			log().fatal(MF_1_INPUT_1_ALREADY_CONNECTED, t1.name());
		connect_input_output(t1, t2);
	}
	else if (t1.is_type(detail::terminal_type::OUTPUT) && t2.is_type(detail::terminal_type::TERMINAL))
	{
		connect_terminal_output(dynamic_cast<terminal_t &>(t2), t1);
	}
	else if (t1.is_type(detail::terminal_type::TERMINAL) && t2.is_type(detail::terminal_type::OUTPUT))
	{
		connect_terminal_output(dynamic_cast<terminal_t &>(t1), t2);
	}
	else if (t1.is_type(detail::terminal_type::INPUT) && t2.is_type(detail::terminal_type::TERMINAL))
	{
		connect_terminal_input(dynamic_cast<terminal_t &>(t2), t1);
	}
	else if (t1.is_type(detail::terminal_type::TERMINAL) && t2.is_type(detail::terminal_type::INPUT))
	{
		connect_terminal_input(dynamic_cast<terminal_t &>(t1), t2);
	}
	else if (t1.is_type(detail::terminal_type::TERMINAL) && t2.is_type(detail::terminal_type::TERMINAL))
	{
		connect_terminals(dynamic_cast<terminal_t &>(t1), dynamic_cast<terminal_t &>(t2));
	}
	else if (t1.is_type(detail::terminal_type::INPUT) && t2.is_type(detail::terminal_type::INPUT))
	{
		ret = connect_input_input(t1, t2);
	}
	else
		ret = false;
		//netlist().error("Connecting {1} to {2} not supported!\n", t1.name(), t2.name());
	return ret;
}

void setup_t::resolve_inputs()
{
	log().verbose("Resolving inputs ...");

	/* Netlist can directly connect input to input.
	 * We therefore first park connecting inputs and retry
	 * after all other terminals were connected.
	 */
	int tries = NL_MAX_LINK_RESOLVE_LOOPS;
	while (m_links.size() > 0 && tries >  0)
	{

		for (auto li = m_links.begin(); li != m_links.end(); )
		{
			const pstring t1s = li->first;
			const pstring t2s = li->second;
			detail::core_terminal_t *t1 = find_terminal(t1s);
			detail::core_terminal_t *t2 = find_terminal(t2s);

			if (connect(*t1, *t2))
				li = m_links.erase(li);
			else
				li++;
		}
		tries--;
	}
	if (tries == 0)
	{
		for (auto & link : m_links)
			log().warning(MF_2_CONNECTING_1_TO_2, link.first, link.second);

		log().fatal(MF_0_LINK_TRIES_EXCEEDED);
	}

	log().verbose("deleting empty nets ...");

	// delete empty nets

	netlist().m_nets.erase(
			std::remove_if(netlist().m_nets.begin(), netlist().m_nets.end(),
					[](plib::owned_ptr<detail::net_t> &x)
					{
						if (x->num_cons() == 0)
						{
							x->netlist().log().verbose("Deleting net {1} ...", x->name());
							return true;
						}
						else
							return false;
					}), netlist().m_nets.end());

	pstring errstr("");

	log().verbose("looking for terminals not connected ...");
	for (auto & i : m_terminals)
	{
		detail::core_terminal_t *term = i.second;
		if (!term->has_net() && dynamic_cast< devices::NETLIB_NAME(dummy_input) *>(&term->device()) != nullptr)
			log().warning(MW_1_DUMMY_1_WITHOUT_CONNECTIONS, term->name());
		else if (!term->has_net())
			errstr += plib::pfmt("Found terminal {1} without a net\n")(term->name());
		else if (term->net().num_cons() == 0)
			log().warning(MW_1_TERMINAL_1_WITHOUT_CONNECTIONS, term->name());
	}
	//FIXME: error string handling
	if (errstr != "")
		log().fatal("{1}", errstr);

}

void setup_t::start_devices()
{
	pstring env = plib::util::environment("NL_LOGS", "");

	if (env != "")
	{
		log().debug("Creating dynamic logs ...");
		std::vector<pstring> loglist(plib::psplit(env, ":"));
		for (pstring ll : loglist)
		{
			pstring name = "log_" + ll;
			auto nc = factory().factory_by_name("LOG")->Create(netlist(), name);
			register_link(name + ".I", ll);
			log().debug("    dynamic link {1}: <{2}>\n",ll, name);
			netlist().register_dev(std::move(nc));
		}
	}
}

plib::plog_base<netlist_t, NL_DEBUG> &setup_t::log()
{
	return netlist().log();
}
const plib::plog_base<netlist_t, NL_DEBUG> &setup_t::log() const
{
	return netlist().log();
}


// ----------------------------------------------------------------------------------------
// Model
// ----------------------------------------------------------------------------------------

static pstring model_string(detail::model_map_t &map)
{
	pstring ret = map["COREMODEL"] + "(";
	for (auto & i : map)
		ret = ret + i.first + "=" + i.second + " ";

	return ret + ")";
}

void setup_t::model_parse(const pstring &model_in, detail::model_map_t &map)
{
	pstring model = model_in;
	std::size_t pos = 0;
	pstring key;

	while (true)
	{
		pos = model.find("(");
		if (pos != pstring::npos) break;

		key = model.ucase();
		auto i = m_models.find(key);
		if (i == m_models.end())
			log().fatal(MF_1_MODEL_NOT_FOUND, model);
		model = i->second;
	}
	pstring xmodel = model.left(pos);

	if (xmodel.equals("_"))
		map["COREMODEL"] = key;
	else
	{
		auto i = m_models.find(xmodel);
		if (i != m_models.end())
			model_parse(xmodel, map);
		else
			log().fatal(MF_1_MODEL_NOT_FOUND, model_in);
	}

	pstring remainder = model.substr(pos + 1).trim();
	if (!remainder.endsWith(")"))
		log().fatal(MF_1_MODEL_ERROR_1, model);
	// FIMXE: Not optimal
	remainder = remainder.left(remainder.length() - 1);

	std::vector<pstring> pairs(plib::psplit(remainder," ", true));
	for (pstring &pe : pairs)
	{
		auto pose = pe.find("=");
		if (pose == pstring::npos)
			log().fatal(MF_1_MODEL_ERROR_ON_PAIR_1, model);
		map[pe.left(pose).ucase()] = pe.substr(pose + 1);
	}
}

const pstring setup_t::model_value_str(detail::model_map_t &map, const pstring &entity)
{
	pstring ret;

	if (entity != entity.ucase())
		log().fatal(MF_2_MODEL_PARAMETERS_NOT_UPPERCASE_1_2, entity,
				model_string(map));
	if (map.find(entity) == map.end())
		log().fatal(MF_2_ENTITY_1_NOT_FOUND_IN_MODEL_2, entity, model_string(map));
	else
		ret = map[entity];

	return ret;
}

nl_double setup_t::model_value(detail::model_map_t &map, const pstring &entity)
{
	pstring tmp = model_value_str(map, entity);

	nl_double factor = NL_FCONST(1.0);
	auto p = std::next(tmp.begin(), static_cast<pstring::difference_type>(tmp.length() - 1));
	switch (*p)
	{
		case 'M': factor = 1e6; break;
		case 'k': factor = 1e3; break;
		case 'm': factor = 1e-3; break;
		case 'u': factor = 1e-6; break;
		case 'n': factor = 1e-9; break;
		case 'p': factor = 1e-12; break;
		case 'f': factor = 1e-15; break;
		case 'a': factor = 1e-18; break;
		default:
			if (*p < '0' || *p > '9')
			log().fatal(MF_1_UNKNOWN_NUMBER_FACTOR_IN_1, entity);
	}
	if (factor != NL_FCONST(1.0))
		tmp = tmp.left(tmp.length() - 1);
	return tmp.as_double() * factor;
}

class logic_family_std_proxy_t : public logic_family_desc_t
{
public:
	logic_family_std_proxy_t() { }
	virtual plib::owned_ptr<devices::nld_base_d_to_a_proxy> create_d_a_proxy(netlist_t &anetlist,
			const pstring &name, logic_output_t *proxied) const override;
	virtual plib::owned_ptr<devices::nld_base_a_to_d_proxy> create_a_d_proxy(netlist_t &anetlist, const pstring &name, logic_input_t *proxied) const override;
};

plib::owned_ptr<devices::nld_base_d_to_a_proxy> logic_family_std_proxy_t::create_d_a_proxy(netlist_t &anetlist,
		const pstring &name, logic_output_t *proxied) const
{
	return plib::owned_ptr<devices::nld_base_d_to_a_proxy>::Create<devices::nld_d_to_a_proxy>(anetlist, name, proxied);
}
plib::owned_ptr<devices::nld_base_a_to_d_proxy> logic_family_std_proxy_t::create_a_d_proxy(netlist_t &anetlist, const pstring &name, logic_input_t *proxied) const
{
	return plib::owned_ptr<devices::nld_base_a_to_d_proxy>::Create<devices::nld_a_to_d_proxy>(anetlist, name, proxied);
}


const logic_family_desc_t *setup_t::family_from_model(const pstring &model)
{
	detail::model_map_t map;
	model_parse(model, map);

	if (model_value_str(map, "TYPE") == "TTL")
		return family_TTL();
	if (model_value_str(map, "TYPE") == "CD4XXX")
		return family_CD4XXX();

	for (auto & e : netlist().m_family_cache)
		if (e.first == model)
			return e.second.get();

	auto ret = plib::make_unique_base<logic_family_desc_t, logic_family_std_proxy_t>();

	ret->m_fixed_V = model_value(map, "FV");
	ret->m_low_thresh_PCNT = model_value(map, "IVL");
	ret->m_high_thresh_PCNT = model_value(map, "IVH");
	ret->m_low_VO = model_value(map, "OVL");
	ret->m_high_VO = model_value(map, "OVH");
	ret->m_R_low = model_value(map, "ORL");
	ret->m_R_high = model_value(map, "ORH");

	auto retp = ret.get();

	netlist().m_family_cache.emplace_back(model, std::move(ret));

	return retp;
}

void setup_t::tt_factory_create(tt_desc &desc, const pstring &sourcefile)
{
	devices::tt_factory_create(*this, desc, sourcefile);
}


// ----------------------------------------------------------------------------------------
// Sources
// ----------------------------------------------------------------------------------------

void setup_t::include(const pstring &netlist_name)
{
	for (auto &source : m_sources)
	{
		if (source->parse(netlist_name))
			return;
	}
	log().fatal(MF_1_NOT_FOUND_IN_SOURCE_COLLECTION, netlist_name);
}

std::unique_ptr<plib::pistream> setup_t::get_data_stream(const pstring &name)
{
	for (auto &source : m_sources)
	{
		if (source->type() == source_t::DATA)
		{
			auto strm = source->stream(name);
			if (strm)
				return strm;
		}
	}
	log().warning(MW_1_DATA_1_NOT_FOUND, name);
	return std::unique_ptr<plib::pistream>(nullptr);
}


bool setup_t::parse_stream(plib::putf8_reader &istrm, const pstring &name)
{
	plib::pomemstream ostrm;
	plib::putf8_writer owrt(ostrm);

	plib::ppreprocessor(&m_defines).process(istrm, owrt);
	plib::pimemstream istrm2(ostrm);
	plib::putf8_reader reader2(istrm2);
	return parser_t(reader2, *this).parse(name);
}

void setup_t::register_define(pstring defstr)
{
	auto p = defstr.find("=");
	if (p != pstring::npos)
		register_define(defstr.left(p), defstr.substr(p+1));
	else
		register_define(defstr, "1");
}

// ----------------------------------------------------------------------------------------
// base sources
// ----------------------------------------------------------------------------------------

bool source_t::parse(const pstring &name)
{
	if (m_type != SOURCE)
		return false;
	else
	{
		auto rstream = stream(name);
		plib::putf8_reader reader(*rstream);
		return m_setup.parse_stream(reader, name);
	}
}

std::unique_ptr<plib::pistream> source_string_t::stream(const pstring &name)
{
	return plib::make_unique_base<plib::pistream, plib::pimemstream>(m_str.c_str(), m_str.mem_t_size());
}

std::unique_ptr<plib::pistream> source_mem_t::stream(const pstring &name)
{
	return plib::make_unique_base<plib::pistream, plib::pimemstream>(m_str.c_str(), m_str.mem_t_size());
}

std::unique_ptr<plib::pistream> source_file_t::stream(const pstring &name)
{
	return plib::make_unique_base<plib::pistream, plib::pifilestream>(m_filename);
}

bool source_proc_t::parse(const pstring &name)
{
	if (name == m_setup_func_name)
	{
		m_setup_func(setup());
		return true;
	}
	else
		return false;
}

std::unique_ptr<plib::pistream> source_proc_t::stream(const pstring &name)
{
	std::unique_ptr<plib::pistream> p(nullptr);
	return p;
}

}

