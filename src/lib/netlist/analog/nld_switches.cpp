// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * nld_legacy.c
 *
 */

#include "nlid_twoterm.h"
#include "../nl_base.h"
#include "../nl_factory.h"

#define R_OFF   (1.0 / netlist().gmin())
#define R_ON    0.01

namespace netlist
{
	namespace analog
	{
	// ----------------------------------------------------------------------------------------
	// SWITCH
	// ----------------------------------------------------------------------------------------

	NETLIB_OBJECT(switch1)
	{
		NETLIB_CONSTRUCTOR(switch1)
		, m_R(*this, "R")
		, m_POS(*this, "POS", 0)
		{
			register_subalias("1", m_R.m_P);
			register_subalias("2", m_R.m_N);
		}

		NETLIB_RESETI();
		NETLIB_UPDATEI();
		NETLIB_UPDATE_PARAMI();

		analog::NETLIB_SUB(R_base) m_R;
		param_logic_t              m_POS;
	};


	NETLIB_RESET(switch1)
	{
		m_R.set_R(R_OFF);
	}

	NETLIB_UPDATE(switch1)
	{
		if (!m_POS())
		{
			m_R.set_R(R_OFF);
		}
		else
		{
			m_R.set_R(R_ON);
		}

		m_R.update_dev();
	}

	NETLIB_UPDATE_PARAM(switch1)
	{
		update();
	}

// ----------------------------------------------------------------------------------------
// SWITCH2
// ----------------------------------------------------------------------------------------

	NETLIB_OBJECT(switch2)
	{
		NETLIB_CONSTRUCTOR(switch2)
		, m_R1(*this, "R1")
		, m_R2(*this, "R2")
		, m_POS(*this, "POS", 0)
		{
			connect(m_R1.m_N, m_R2.m_N);

			register_subalias("1", m_R1.m_P);
			register_subalias("2", m_R2.m_P);

			register_subalias("Q", m_R1.m_N);
		}

		NETLIB_RESETI();
		NETLIB_UPDATEI();
		NETLIB_UPDATE_PARAMI();

		analog::NETLIB_SUB(R_base) m_R1;
		analog::NETLIB_SUB(R_base) m_R2;
		param_logic_t             m_POS;
	};

	NETLIB_RESET(switch2)
	{
		m_R1.set_R(R_ON);
		m_R2.set_R(R_OFF);
	}

	NETLIB_UPDATE(switch2)
	{
		if (!m_POS())
		{
			m_R1.set_R(R_ON);
			m_R2.set_R(R_OFF);
		}
		else
		{
			m_R1.set_R(R_OFF);
			m_R2.set_R(R_ON);
		}

		m_R1.update_dev();
		m_R2.update_dev();
	}

	NETLIB_UPDATE_PARAM(switch2)
	{
		update();
	}

	} //namespace analog

	namespace devices {
		NETLIB_DEVICE_IMPL_NS(analog, switch1)
		NETLIB_DEVICE_IMPL_NS(analog, switch2)
	}
} // namespace netlist
