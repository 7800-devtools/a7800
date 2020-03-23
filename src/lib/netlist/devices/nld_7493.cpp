// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * nld_7493.cpp
 *
 */

#include "nld_7493.h"
#include "../nl_base.h"

namespace netlist
{
	namespace devices
	{

	static constexpr netlist_time out_delay = NLTIME_FROM_NS(18);
	static constexpr netlist_time out_delay2 = NLTIME_FROM_NS(36);
	static constexpr netlist_time out_delay3 = NLTIME_FROM_NS(54);

	NETLIB_OBJECT(7493)
	{
		NETLIB_CONSTRUCTOR(7493)
		, m_R1(*this, "R1")
		, m_R2(*this, "R2")
		, m_reset(*this, "_m_reset", 0)
		, m_a(*this, "_m_a", 0)
		, m_bcd(*this, "_m_b", 0)
		, m_CLKA(*this, "CLKA", NETLIB_DELEGATE(7493, updA))
		, m_CLKB(*this, "CLKB", NETLIB_DELEGATE(7493, updB))
		, m_QA(*this, "QA")
		, m_QB(*this, "QB")
		, m_QC(*this, "QC")
		, m_QD(*this, "QD")
		{
		}

	private:
		NETLIB_RESETI();
		NETLIB_UPDATEI();

		NETLIB_HANDLERI(updA)
		{
			if (m_reset)
			{
				m_a ^= 1;
				m_QA.push(m_a, out_delay);
			}
		}

		NETLIB_HANDLERI(updB)
		{
			if (m_reset)
			{
				++m_bcd &= static_cast<std::uint8_t>(0x07);
				m_QD.push((m_bcd >> 2) & 1, out_delay3);
				m_QC.push((m_bcd >> 1) & 1, out_delay2);
				m_QB.push(m_bcd & 1, out_delay);
			}
		}

		logic_input_t m_R1;
		logic_input_t m_R2;

		state_var_sig m_reset;
		state_var_sig m_a;
		state_var_sig m_bcd;

		logic_input_t m_CLKA;
		logic_input_t m_CLKB;

		logic_output_t m_QA;
		logic_output_t m_QB;
		logic_output_t m_QC;
		logic_output_t m_QD;
	};

	NETLIB_OBJECT_DERIVED(7493_dip, 7493)
	{
		NETLIB_CONSTRUCTOR_DERIVED(7493_dip, 7493)
		{
			register_subalias("1", "CLKB");
			register_subalias("2", "R1");
			register_subalias("3", "R2");

			// register_subalias("4", ); --> NC
			// register_subalias("5", ); --> VCC
			// register_subalias("6", ); --> NC
			// register_subalias("7", ); --> NC

			register_subalias("8", "QC");
			register_subalias("9", "QB");
			// register_subalias("10", ); -. GND
			register_subalias("11", "QD");
			register_subalias("12", "QA");
			// register_subalias("13", ); -. NC
			register_subalias("14", "CLKA");
		}
	};

	NETLIB_RESET(7493)
	{
		m_reset = 1;
		m_a = m_bcd = 0;
		m_CLKA.set_state(logic_t::STATE_INP_HL);
		m_CLKB.set_state(logic_t::STATE_INP_HL);
	}

	NETLIB_UPDATE(7493)
	{
		m_reset = (m_R1() & m_R2()) ^ 1;

		if (m_reset)
		{
			m_CLKA.activate_hl();
			m_CLKB.activate_hl();
		}
		else
		{
			m_CLKA.inactivate();
			m_CLKB.inactivate();
			m_QA.push_force(0, NLTIME_FROM_NS(40));
			m_QB.push_force(0, NLTIME_FROM_NS(40));
			m_QC.push_force(0, NLTIME_FROM_NS(40));
			m_QD.push_force(0, NLTIME_FROM_NS(40));
			m_a = m_bcd = 0;
		}
	}

	NETLIB_DEVICE_IMPL(7493)
	NETLIB_DEVICE_IMPL(7493_dip)

	} //namespace devices
} // namespace netlist
