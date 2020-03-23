// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * nld_ms_sor.h
 *
 * Generic successive over relaxation solver.
 *
 * Fow w==1 we will do the classic Gauss-Seidel approach
 *
 */

#ifndef NLD_MS_SOR_H_
#define NLD_MS_SOR_H_

#include <algorithm>

#include "nld_ms_direct.h"
#include "nld_solver.h"

namespace netlist
{
	namespace devices
	{
template <std::size_t m_N, std::size_t storage_N>
class matrix_solver_SOR_t: public matrix_solver_direct_t<m_N, storage_N>
{
public:

	matrix_solver_SOR_t(netlist_t &anetlist, const pstring &name, const solver_parameters_t *params, const std::size_t size)
		: matrix_solver_direct_t<m_N, storage_N>(anetlist, name, matrix_solver_t::ASCENDING, params, size)
		, m_lp_fact(*this, "m_lp_fact", 0)
		{
		}

	virtual ~matrix_solver_SOR_t() override {}

	virtual void vsetup(analog_net_t::list_t &nets) override;
	virtual unsigned vsolve_non_dynamic(const bool newton_raphson) override;

private:
	state_var<nl_double> m_lp_fact;
};

// ----------------------------------------------------------------------------------------
// matrix_solver - Gauss - Seidel
// ----------------------------------------------------------------------------------------


template <std::size_t m_N, std::size_t storage_N>
void matrix_solver_SOR_t<m_N, storage_N>::vsetup(analog_net_t::list_t &nets)
{
	matrix_solver_direct_t<m_N, storage_N>::vsetup(nets);
}

template <std::size_t m_N, std::size_t storage_N>
unsigned matrix_solver_SOR_t<m_N, storage_N>::vsolve_non_dynamic(const bool newton_raphson)
{
	const std::size_t iN = this->N();
	bool resched = false;
	unsigned resched_cnt = 0;

	/* ideally, we could get an estimate for the spectral radius of
	 * Inv(D - L) * U
	 *
	 * and estimate using
	 *
	 * omega = 2.0 / (1.0 + std::sqrt(1-rho))
	 */

	const nl_double ws = this->m_params.m_gs_sor;

	nl_double w[storage_N];
	nl_double one_m_w[storage_N];
	nl_double RHS[storage_N];
	nl_double new_V[storage_N];

	for (std::size_t k = 0; k < iN; k++)
	{
		nl_double gtot_t = 0.0;
		nl_double gabs_t = 0.0;
		nl_double RHS_t = 0.0;

		const std::size_t term_count = this->m_terms[k]->count();
		const nl_double * const RESTRICT gt = this->m_terms[k]->gt();
		const nl_double * const RESTRICT go = this->m_terms[k]->go();
		const nl_double * const RESTRICT Idr = this->m_terms[k]->Idr();
		const nl_double * const *other_cur_analog = this->m_terms[k]->connected_net_V();

		new_V[k] = this->m_nets[k]->Q_Analog();

		for (std::size_t i = 0; i < term_count; i++)
		{
			gtot_t = gtot_t + gt[i];
			RHS_t = RHS_t + Idr[i];
		}

		for (std::size_t i = this->m_terms[k]->m_railstart; i < term_count; i++)
			RHS_t = RHS_t  + go[i] * *other_cur_analog[i];

		RHS[k] = RHS_t;

		if (USE_GABS)
		{
			for (std::size_t i = 0; i < term_count; i++)
				gabs_t = gabs_t + std::abs(go[i]);

			gabs_t *= NL_FCONST(0.5); // derived by try and error
			if (gabs_t <= gtot_t)
			{
				w[k] = ws / gtot_t;
				one_m_w[k] = NL_FCONST(1.0) - ws;
			}
			else
			{
				w[k] = NL_FCONST(1.0) / (gtot_t + gabs_t);
				one_m_w[k] = NL_FCONST(1.0) - NL_FCONST(1.0) * gtot_t / (gtot_t + gabs_t);
			}
		}
		else
		{
			w[k] = ws / gtot_t;
			one_m_w[k] = NL_FCONST(1.0) - ws;
		}
	}

	const nl_double accuracy = this->m_params.m_accuracy;

	do {
		resched = false;
		nl_double err = 0;
		for (std::size_t k = 0; k < iN; k++)
		{
			const int * RESTRICT net_other = this->m_terms[k]->connected_net_idx();
			const std::size_t railstart = this->m_terms[k]->m_railstart;
			const nl_double * RESTRICT go = this->m_terms[k]->go();

			nl_double Idrive = 0.0;
			for (std::size_t i = 0; i < railstart; i++)
				Idrive = Idrive + go[i] * new_V[net_other[i]];

			const nl_double new_val = new_V[k] * one_m_w[k] + (Idrive + RHS[k]) * w[k];

			err = std::max(std::abs(new_val - new_V[k]), err);
			new_V[k] = new_val;
		}

		if (err > accuracy)
			resched = true;

		resched_cnt++;
	//} while (resched && (resched_cnt < this->m_params.m_gs_loops));
	} while (resched && ((resched_cnt < this->m_params.m_gs_loops)));

	this->m_iterative_total += resched_cnt;
	this->m_stat_calculations++;

	if (resched)
	{
		// Fallback to direct solver ...
		this->m_iterative_fail++;
		return matrix_solver_direct_t<m_N, storage_N>::vsolve_non_dynamic(newton_raphson);
	}

	for (std::size_t k = 0; k < iN; k++)
		this->m_nets[k]->set_Q_Analog(new_V[k]);

	return resched_cnt;
}

	} //namespace devices
} // namespace netlist

#endif /* NLD_MS_SOR_H_ */
