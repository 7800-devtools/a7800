// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * nld_9316.c
 *
 */

#include "nld_9316.h"
#include "../nl_base.h"

namespace netlist
{
	namespace devices
	{

	static constexpr const unsigned MAXCNT = 15;

	NETLIB_OBJECT(9316)
	{
		NETLIB_CONSTRUCTOR(9316)
		, m_CLK(*this, "CLK", NETLIB_DELEGATE(9316, clk))
		, m_ENT(*this, "ENT")
		, m_LOADQ(*this, "LOADQ")
		, m_cnt(*this, "m_cnt", 0)
		, m_ENP(*this, "ENP")
		, m_CLRQ(*this, "CLRQ")
		, m_A(*this, "A", NETLIB_DELEGATE(9316, noop))
		, m_B(*this, "B", NETLIB_DELEGATE(9316, noop))
		, m_C(*this, "C", NETLIB_DELEGATE(9316, noop))
		, m_D(*this, "D", NETLIB_DELEGATE(9316, noop))
		, m_Q(*this, {{ "QA", "QB", "QC", "QD" }})
		, m_RC(*this, "RC")
		{
		}

	private:
		NETLIB_RESETI();
		NETLIB_UPDATEI();
		NETLIB_HANDLERI(clk);
		NETLIB_HANDLERI(noop) { }

		logic_input_t m_CLK;

		logic_input_t m_ENT;
		logic_input_t m_LOADQ;

		state_var<unsigned> m_cnt;

		logic_input_t m_ENP;
		logic_input_t m_CLRQ;

		logic_input_t m_A;
		logic_input_t m_B;
		logic_input_t m_C;
		logic_input_t m_D;

		object_array_t<logic_output_t, 4> m_Q;
		logic_output_t m_RC;


	private:
		//inline void update_outputs_all(const unsigned &cnt, const netlist_time &out_delay) noexcept
		inline void update_outputs_all(const unsigned &cnt, const netlist_time &out_delay) noexcept
		{
			m_Q[0].push((cnt >> 0) & 1, out_delay);
			m_Q[1].push((cnt >> 1) & 1, out_delay);
			m_Q[2].push((cnt >> 2) & 1, out_delay);
			m_Q[3].push((cnt >> 3) & 1, out_delay);
		}
	};

	NETLIB_OBJECT_DERIVED(9316_dip, 9316)
	{
		NETLIB_CONSTRUCTOR_DERIVED(9316_dip, 9316)
		{
			register_subalias("1", "CLRQ");
			register_subalias("2", "CLK");
			register_subalias("3", "A");
			register_subalias("4", "B");
			register_subalias("5", "C");
			register_subalias("6", "D");
			register_subalias("7", "ENP");
			// register_subalias("8", "); -. GND

			register_subalias("9", "LOADQ");
			register_subalias("10", "ENT");
			register_subalias("11", "QD");
			register_subalias("12", "QC");
			register_subalias("13", "QB");
			register_subalias("14", "QA");
			register_subalias("15", "RC");
			// register_subalias("16", ); -. VCC
		}
	};

	NETLIB_RESET(9316)
	{
		m_CLK.set_state(logic_t::STATE_INP_LH);
		m_cnt = 0;
	}

	NETLIB_HANDLER(9316, clk)
	{
		if (m_LOADQ())
		{
			++m_cnt &= MAXCNT;
			//m_RC.push(m_ENT() && (m_cnt == MAXCNT), NLTIME_FROM_NS(27));
			if (m_cnt == MAXCNT)
			{
				m_RC.push(m_ENT(), NLTIME_FROM_NS(27));
				update_outputs_all(MAXCNT, NLTIME_FROM_NS(20));
			}
			else if (m_cnt == 0)
			{
				m_RC.push(0, NLTIME_FROM_NS(27));
				update_outputs_all(0, NLTIME_FROM_NS(20));
			}
			else
				update_outputs_all(m_cnt, NLTIME_FROM_NS(20));
		}
		else
		{
			m_cnt = (m_D() << 3) | (m_C() << 2) | (m_B() << 1) | (m_A() << 0);
			m_RC.push(m_ENT() && (m_cnt == MAXCNT), NLTIME_FROM_NS(27));
			update_outputs_all(m_cnt, NLTIME_FROM_NS(22));
		}
	}

	NETLIB_UPDATE(9316)
	{
		const netlist_sig_t CLRQ(m_CLRQ());
		const netlist_sig_t ENT(m_ENT());

		if (((m_LOADQ() ^ 1) || (ENT && m_ENP())) && CLRQ)
		{
			m_CLK.activate_lh();
		}
		else
		{
			m_CLK.inactivate();
			if (!CLRQ && (m_cnt>0))
			{
				m_cnt = 0;
				update_outputs_all(m_cnt, NLTIME_FROM_NS(36));
			}
		}
		m_RC.push(ENT && (m_cnt == MAXCNT), NLTIME_FROM_NS(27));
	}


	NETLIB_DEVICE_IMPL(9316)
	NETLIB_DEVICE_IMPL(9316_dip)

	} //namespace devices
} // namespace netlist
