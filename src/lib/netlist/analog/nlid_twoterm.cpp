// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * nld_twoterm.c
 *
 */

#include "../solver/nld_solver.h"

#include "nlid_twoterm.h"
#include "../nl_factory.h"

#include <cmath>

namespace netlist
{
	namespace analog
	{
// ----------------------------------------------------------------------------------------
// generic_diode
// ----------------------------------------------------------------------------------------

generic_diode::generic_diode(device_t &dev, pstring name)
	: m_Vd(dev, name + ".m_Vd", 0.7)
	, m_Id(dev, name + ".m_Id", 0.0)
	, m_G(dev,  name + ".m_G", 1e-15)
	, m_Vt(0.0)
	, m_Vmin(0.0)
	, m_Is(0.0)
	, m_logIs(0.0)
	, m_n(0.0)
	, m_gmin(1e-15)
	, m_VtInv(0.0)
	, m_Vcrit(0.0)
{
	set_param(1e-15, 1, 1e-15);
}

void generic_diode::set_param(const nl_double Is, const nl_double n, nl_double gmin)
{
	static constexpr double csqrt2 = 1.414213562373095048801688724209; //std::sqrt(2.0);
	m_Is = Is;
	m_logIs = std::log(Is);
	m_n = n;
	m_gmin = gmin;

	m_Vt = 0.0258 * m_n;
	m_Vmin = -5.0 * m_Vt;

	m_Vcrit = m_Vt * std::log(m_Vt / m_Is / csqrt2);
	m_VtInv = 1.0 / m_Vt;
}

void generic_diode::update_diode(const nl_double nVd)
{
	if (nVd < m_Vmin)
	{
		m_Vd = nVd;
		m_G = m_gmin;
		m_Id = - m_Is;
	}
	else if (nVd < m_Vcrit)
	{
		m_Vd = nVd;
		//m_Vd = m_Vd + 10.0 * m_Vt * std::tanh((nVd - m_Vd) / 10.0 / m_Vt);
		//const double IseVDVt = m_Is * std::exp(m_Vd * m_VtInv);
		const double IseVDVt = std::exp(m_logIs + m_Vd * m_VtInv);
		m_Id = IseVDVt - m_Is;
		m_G = IseVDVt * m_VtInv + m_gmin;
	}
	else
	{
		const double a = std::max((nVd - m_Vd) * m_VtInv, NL_FCONST(-0.99));
		m_Vd = m_Vd + std::log1p(a) * m_Vt;
		//const double IseVDVt = m_Is * std::exp(m_Vd * m_VtInv);
		const double IseVDVt = std::exp(m_logIs + m_Vd * m_VtInv);
		m_Id = IseVDVt - m_Is;
		m_G = IseVDVt * m_VtInv + m_gmin;
	}
}

// ----------------------------------------------------------------------------------------
// nld_twoterm
// ----------------------------------------------------------------------------------------

NETLIB_UPDATE(twoterm)
{
	/* only called if connected to a rail net ==> notify the solver to recalculate */
	/* we only need to call the non-rail terminal */
	if (m_P.has_net() && !m_P.net().isRailNet())
		m_P.solve_now();
	else if (m_N.has_net() && !m_N.net().isRailNet())
		m_N.solve_now();
}

// ----------------------------------------------------------------------------------------
// nld_R_base
// ----------------------------------------------------------------------------------------

NETLIB_RESET(R_base)
{
	NETLIB_NAME(twoterm)::reset();
	set_R(1.0 / netlist().gmin());
}

NETLIB_UPDATE(R_base)
{
	NETLIB_NAME(twoterm)::update();
}

// ----------------------------------------------------------------------------------------
// nld_R
// ----------------------------------------------------------------------------------------

NETLIB_UPDATE_PARAM(R)
{
	update_dev();
	set_R(std::max(m_R(), netlist().gmin()));
}

NETLIB_RESET(R)
{
	NETLIB_NAME(twoterm)::reset();
	set_R(std::max(m_R(), netlist().gmin()));
}

// ----------------------------------------------------------------------------------------
// nld_POT
// ----------------------------------------------------------------------------------------

NETLIB_RESET(POT)
{
	nl_double v = m_Dial();
	if (m_DialIsLog())
		v = (std::exp(v) - 1.0) / (std::exp(1.0) - 1.0);

	m_R1.set_R(std::max(m_R() * v, netlist().gmin()));
	m_R2.set_R(std::max(m_R() * (NL_FCONST(1.0) - v), netlist().gmin()));
}

NETLIB_UPDATE_PARAM(POT)
{
	m_R1.update_dev();
	m_R2.update_dev();

	nl_double v = m_Dial();
	if (m_DialIsLog())
		v = (std::exp(v) - 1.0) / (std::exp(1.0) - 1.0);

	m_R1.set_R(std::max(m_R() * v, netlist().gmin()));
	m_R2.set_R(std::max(m_R() * (NL_FCONST(1.0) - v), netlist().gmin()));

}

// ----------------------------------------------------------------------------------------
// nld_POT2
// ----------------------------------------------------------------------------------------

NETLIB_RESET(POT2)
{
	nl_double v = m_Dial();

	if (m_DialIsLog())
		v = (std::exp(v) - 1.0) / (std::exp(1.0) - 1.0);
	if (m_Reverse())
		v = 1.0 - v;
	m_R1.set_R(std::max(m_R() * v, netlist().gmin()));
}


NETLIB_UPDATE_PARAM(POT2)
{
	m_R1.update_dev();

	nl_double v = m_Dial();

	if (m_DialIsLog())
		v = (std::exp(v) - 1.0) / (std::exp(1.0) - 1.0);
	if (m_Reverse())
		v = 1.0 - v;
	m_R1.set_R(std::max(m_R() * v, netlist().gmin()));
}

// ----------------------------------------------------------------------------------------
// nld_C
// ----------------------------------------------------------------------------------------

NETLIB_RESET(C)
{
	// FIXME: Startup conditions
	set(netlist().gmin(), 0.0, -5.0 / netlist().gmin());
	//set(netlist().gmin(), 0.0, 0.0);
}

NETLIB_UPDATE_PARAM(C)
{
	m_GParallel = netlist().gmin();
}

NETLIB_UPDATE(C)
{
	NETLIB_NAME(twoterm)::update();
}

NETLIB_TIMESTEP(C)
{
	/* Gpar should support convergence */
	const nl_double G = m_C() / step +  m_GParallel;
	const nl_double I = -G * deltaV();
	set_mat( G, -G, -I,
			-G,  G,  I);
	//set(G, 0.0, I);
}

// ----------------------------------------------------------------------------------------
// nld_L
// ----------------------------------------------------------------------------------------

NETLIB_RESET(L)
{
	m_GParallel = netlist().gmin();
	m_I = 0.0;
	m_G = m_GParallel;
	set_mat( m_G, -m_G, -m_I,
			-m_G,  m_G,  m_I);
	//set(1.0/NETLIST_GMIN, 0.0, -5.0 * NETLIST_GMIN);
}

NETLIB_UPDATE_PARAM(L)
{
}

NETLIB_UPDATE(L)
{
	NETLIB_NAME(twoterm)::update();
}

NETLIB_TIMESTEP(L)
{
	/* Gpar should support convergence */
	m_I += m_I + m_G * deltaV();
	m_G = step / m_L() + m_GParallel;
	set_mat( m_G, -m_G, -m_I,
			-m_G,  m_G,  m_I);
	//set(m_G, 0.0, m_I);
}

// ----------------------------------------------------------------------------------------
// nld_D
// ----------------------------------------------------------------------------------------

NETLIB_RESET(D)
{
	nl_double Is = m_model.m_IS;
	nl_double n = m_model.m_N;

	m_D.set_param(Is, n, netlist().gmin());
	set(m_D.G(), 0.0, m_D.Ieq());
}

NETLIB_UPDATE_PARAM(D)
{
	nl_double Is = m_model.m_IS;
	nl_double n = m_model.m_N;

	m_D.set_param(Is, n, netlist().gmin());
}

NETLIB_UPDATE(D)
{
	NETLIB_NAME(twoterm)::update();
}

NETLIB_UPDATE_TERMINALS(D)
{
	m_D.update_diode(deltaV());
	const nl_double G = m_D.G();
	const nl_double I = m_D.Ieq();
	set_mat( G, -G, -I,
			-G,  G,  I);
	//set(m_D.G(), 0.0, m_D.Ieq());
}

// ----------------------------------------------------------------------------------------
// nld_VS
// ----------------------------------------------------------------------------------------

NETLIB_RESET(VS)
{
	NETLIB_NAME(twoterm)::reset();
	this->set(1.0 / m_R(), m_V(), 0.0);
}

NETLIB_UPDATE(VS)
{
	NETLIB_NAME(twoterm)::update();
}

NETLIB_TIMESTEP(VS)
{
	this->set(1.0 / m_R(),
			m_compiled.evaluate(std::vector<double>({netlist().time().as_double()})),
			0.0);
}

// ----------------------------------------------------------------------------------------
// nld_CS
// ----------------------------------------------------------------------------------------

NETLIB_RESET(CS)
{
	NETLIB_NAME(twoterm)::reset();
	const nl_double I = m_I();

	set_mat(0.0, 0.0, -I,
			0.0, 0.0,  I);
	//this->set(0.0, 0.0, m_I());
}

NETLIB_UPDATE(CS)
{
	NETLIB_NAME(twoterm)::update();
}

NETLIB_TIMESTEP(CS)
{
	const double I = m_compiled.evaluate(std::vector<double>({netlist().time().as_double()}));
	set_mat(0.0, 0.0, -I,
			0.0, 0.0,  I);
}

	} //namespace analog

	namespace devices {
		NETLIB_DEVICE_IMPL_NS(analog, R)
		NETLIB_DEVICE_IMPL_NS(analog, POT)
		NETLIB_DEVICE_IMPL_NS(analog, POT2)
		NETLIB_DEVICE_IMPL_NS(analog, C)
		NETLIB_DEVICE_IMPL_NS(analog, L)
		NETLIB_DEVICE_IMPL_NS(analog, D)
		NETLIB_DEVICE_IMPL_NS(analog, VS)
		NETLIB_DEVICE_IMPL_NS(analog, CS)
	}

} // namespace netlist
