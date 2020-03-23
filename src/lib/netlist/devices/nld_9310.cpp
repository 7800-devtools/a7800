// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * nld_9310.c
 *
 */

#include "nld_9310.h"
#include "../nl_base.h"

#define MAXCNT 9

namespace netlist
{
	namespace devices
	{
	NETLIB_OBJECT(9310_subABCD)
	{
		NETLIB_CONSTRUCTOR(9310_subABCD)
		, m_A(*this, "A")
		, m_B(*this, "B")
		, m_C(*this, "C")
		, m_D(*this, "D")
		{
		}

		NETLIB_RESETI();
		//NETLIB_UPDATEI();

	public:
		logic_input_t m_A;
		logic_input_t m_B;
		logic_input_t m_C;
		logic_input_t m_D;

		unsigned read_ABCD() const
		{
			return (m_D() << 3) | (m_C() << 2) | (m_B() << 1) | (m_A() << 0);
		}
	};

	NETLIB_OBJECT(9310_sub)
	{
		NETLIB_CONSTRUCTOR(9310_sub)
		, m_CLK(*this, "CLK")
		, m_ABCD(nullptr)
		, m_QA(*this, "QA")
		, m_QB(*this, "QB")
		, m_QC(*this, "QC")
		, m_QD(*this, "QD")
		, m_RC(*this, "RC")
		, m_cnt(*this, "m_cnt", 0)
		, m_loadq(*this, "m_loadq", 0)
		, m_ent(*this, "m_ent", 0)
		{
		}
		NETLIB_RESETI();
		NETLIB_UPDATEI();
	public:
		inline void update_outputs_all(const unsigned cnt, const netlist_time out_delay);
		inline void update_outputs(const unsigned cnt);

		logic_input_t m_CLK;

		NETLIB_NAME(9310_subABCD) *m_ABCD;

		logic_output_t m_QA;
		logic_output_t m_QB;
		logic_output_t m_QC;
		logic_output_t m_QD;
		logic_output_t m_RC;

		state_var<unsigned> m_cnt;
		state_var<netlist_sig_t> m_loadq;
		state_var<netlist_sig_t> m_ent;
	};

	NETLIB_OBJECT(9310)
	{
		NETLIB_CONSTRUCTOR(9310)
		, subABCD(*this, "subABCD")
		, sub(*this, "sub")
		, m_ENP(*this, "ENP")
		, m_ENT(*this, "ENT")
		, m_CLRQ(*this, "CLRQ")
		, m_LOADQ(*this, "LOADQ")
		{
			sub.m_ABCD = &(subABCD);

			register_subalias("CLK", sub.m_CLK);


			register_subalias("A", subABCD.m_A);
			register_subalias("B", subABCD.m_B);
			register_subalias("C", subABCD.m_C);
			register_subalias("D", subABCD.m_D);

			register_subalias("QA", sub.m_QA);
			register_subalias("QB", sub.m_QB);
			register_subalias("QC", sub.m_QC);
			register_subalias("QD", sub.m_QD);
			register_subalias("RC", sub.m_RC);
		}
		NETLIB_RESETI();
		NETLIB_UPDATEI();

	public:
		NETLIB_SUB(9310_subABCD) subABCD;
		NETLIB_SUB(9310_sub) sub;
		logic_input_t m_ENP;
		logic_input_t m_ENT;
		logic_input_t m_CLRQ;
		logic_input_t m_LOADQ;
	};

	NETLIB_OBJECT_DERIVED(9310_dip, 9310)
	{
		NETLIB_CONSTRUCTOR_DERIVED(9310_dip, 9310)
		{
			register_subalias("1", m_CLRQ);
			register_subalias("2", sub.m_CLK);
			register_subalias("3", subABCD.m_A);
			register_subalias("4", subABCD.m_B);
			register_subalias("5", subABCD.m_C);
			register_subalias("6", subABCD.m_D);
			register_subalias("7", m_ENP);
			// register_subalias("8", ); -. GND

			register_subalias("9", m_LOADQ);
			register_subalias("10", m_ENT);
			register_subalias("11", sub.m_QD);
			register_subalias("12", sub.m_QC);
			register_subalias("13", sub.m_QB);
			register_subalias("14", sub.m_QA);
			register_subalias("15", sub.m_RC);
			// register_subalias("16", ); -. VCC
		}
	};

	NETLIB_RESET(9310)
	{
		sub.do_reset();
		subABCD.do_reset();
	}

	NETLIB_RESET(9310_subABCD)
	{
	}

	NETLIB_RESET(9310_sub)
	{
		m_CLK.set_state(logic_t::STATE_INP_LH);
		m_cnt = 0;
		m_loadq = 1;
		m_ent = 1;
	}

	NETLIB_UPDATE(9310_sub)
	{
		if (m_loadq)
		{
			if (m_cnt < MAXCNT - 1)
			{
				++m_cnt;
				update_outputs(m_cnt);
			}
			else if (m_cnt == MAXCNT - 1)
			{
				m_cnt = MAXCNT;
				m_RC.push(m_ent, NLTIME_FROM_NS(20));
				m_QA.push(1, NLTIME_FROM_NS(20));
			}
			else // MAXCNT
			{
				m_RC.push(0, NLTIME_FROM_NS(20));
				m_cnt = 0;
				update_outputs_all(m_cnt, NLTIME_FROM_NS(20));
			}
		}
		else
		{
			m_cnt = m_ABCD->read_ABCD();
			m_RC.push(m_ent & (m_cnt == MAXCNT), NLTIME_FROM_NS(27));
			update_outputs_all(m_cnt, NLTIME_FROM_NS(22));
		}
	}

	NETLIB_UPDATE(9310)
	{
		sub.m_loadq = m_LOADQ();
		sub.m_ent = m_ENT();
		const netlist_sig_t clrq = m_CLRQ();

		if ((!sub.m_loadq || (sub.m_ent & m_ENP())) && clrq)
		{
			sub.m_CLK.activate_lh();
			sub.m_RC.push(sub.m_ent & (sub.m_cnt == MAXCNT), NLTIME_FROM_NS(27));
		}
		else
		{
			sub.m_CLK.inactivate();
			if (!clrq && (sub.m_cnt>0))
			{
				sub.update_outputs_all(0, NLTIME_FROM_NS(36));
				sub.m_cnt = 0;
				//return;
			}
			sub.m_RC.push(sub.m_ent & (sub.m_cnt == MAXCNT), NLTIME_FROM_NS(27));
		}
	}

	inline NETLIB_FUNC_VOID(9310_sub, update_outputs_all, (const unsigned cnt, const netlist_time out_delay))
	{
		m_QA.push((cnt >> 0) & 1, out_delay);
		m_QB.push((cnt >> 1) & 1, out_delay);
		m_QC.push((cnt >> 2) & 1, out_delay);
		m_QD.push((cnt >> 3) & 1, out_delay);
	}

	inline NETLIB_FUNC_VOID(9310_sub, update_outputs, (const unsigned cnt))
	{
		/* static */ const netlist_time out_delay = NLTIME_FROM_NS(20);
	#if 0
	//    for (int i=0; i<4; i++)
	//        m_Q[i], (cnt >> i) & 1, delay[i]);
		m_QA, (cnt >> 0) & 1, out_delay);
		m_QB, (cnt >> 1) & 1, out_delay);
		m_QC, (cnt >> 2) & 1, out_delay);
		m_QD, (cnt >> 3) & 1, out_delay);
	#else
		if ((cnt & 1) == 1)
			m_QA.push(1, out_delay);
		else
		{
			m_QA.push(0, out_delay);
			switch (cnt)
			{
			case 0x00:
				m_QB.push(0, out_delay);
				m_QC.push(0, out_delay);
				m_QD.push(0, out_delay);
				break;
			case 0x02:
			case 0x06:
			case 0x0A:
			case 0x0E:
				m_QB.push(1, out_delay);
				break;
			case 0x04:
			case 0x0C:
				m_QB.push(0, out_delay);
				m_QC.push(1, out_delay);
				break;
			case 0x08:
				m_QB.push(0, out_delay);
				m_QC.push(0, out_delay);
				m_QD.push(1, out_delay);
				break;
			}

		}
	#endif
	}

	NETLIB_DEVICE_IMPL(9310)
	NETLIB_DEVICE_IMPL(9310_dip)

	} //namespace devices
} // namespace netlist
