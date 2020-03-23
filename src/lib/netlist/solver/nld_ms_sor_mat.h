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

#ifndef NLD_MS_SOR_MAT_H_
#define NLD_MS_SOR_MAT_H_

#include <algorithm>

#include "nld_ms_direct.h"
#include "nld_matrix_solver.h"
#include "nld_solver.h"

namespace netlist
{
	namespace devices
	{
template <std::size_t m_N, std::size_t storage_N>
class matrix_solver_SOR_mat_t: public matrix_solver_direct_t<m_N, storage_N>
{
	friend class matrix_solver_t;

public:

	matrix_solver_SOR_mat_t(netlist_t &anetlist, const pstring &name, const solver_parameters_t *params, std::size_t size)
		: matrix_solver_direct_t<m_N, storage_N>(anetlist, name, matrix_solver_t::DESCENDING, params, size)
		, m_Vdelta(*this, "m_Vdelta", 0.0)
		, m_omega(*this, "m_omega", params->m_gs_sor)
		, m_lp_fact(*this, "m_lp_fact", 0)
		, m_gs_fail(*this, "m_gs_fail", 0)
		, m_gs_total(*this, "m_gs_total", 0)
		{
		}

	virtual ~matrix_solver_SOR_mat_t() override {}

	virtual void vsetup(analog_net_t::list_t &nets) override;

	virtual unsigned vsolve_non_dynamic(const bool newton_raphson) override;

private:
	state_var<nl_double[storage_N]> m_Vdelta;

	state_var<nl_double> m_omega;
	state_var<nl_double> m_lp_fact;
	state_var<int> m_gs_fail;
	state_var<int> m_gs_total;
};

// ----------------------------------------------------------------------------------------
// matrix_solver - Gauss - Seidel
// ----------------------------------------------------------------------------------------

template <std::size_t m_N, std::size_t storage_N>
void matrix_solver_SOR_mat_t<m_N, storage_N>::vsetup(analog_net_t::list_t &nets)
{
	matrix_solver_direct_t<m_N, storage_N>::vsetup(nets);
}

#if 0
//FIXME: move to solve_base
template <unsigned m_N, unsigned storage_N>
nl_double matrix_solver_SOR_mat_t<m_N, storage_N>::vsolve()
{
	/*
	 * enable linear prediction on first newton pass
	 */

	if (USE_LINEAR_PREDICTION)
		for (unsigned k = 0; k < this->N(); k++)
		{
			this->m_last_V[k] = this->m_nets[k]->m_cur_Analog;
			this->m_nets[k]->m_cur_Analog = this->m_nets[k]->m_cur_Analog + this->m_Vdelta[k] * this->current_timestep() * m_lp_fact;
		}
	else
		for (unsigned k = 0; k < this->N(); k++)
		{
			this->m_last_V[k] = this->m_nets[k]->m_cur_Analog;
		}

	this->solve_base(this);

	if (USE_LINEAR_PREDICTION)
	{
		nl_double sq = 0;
		nl_double sqo = 0;
		const nl_double rez_cts = 1.0 / this->current_timestep();
		for (unsigned k = 0; k < this->N(); k++)
		{
			const analog_net_t *n = this->m_nets[k];
			const nl_double nv = (n->Q_Analog() - this->m_last_V[k]) * rez_cts ;
			sq += nv * nv;
			sqo += this->m_Vdelta[k] * this->m_Vdelta[k];
			this->m_Vdelta[k] = nv;
		}

		// FIXME: used to be 1e90, but this would not be compatible with float
		if (sqo > NL_FCONST(1e-20))
			m_lp_fact = std::min(std::sqrt(sq/sqo), (nl_double) 2.0);
		else
			m_lp_fact = NL_FCONST(0.0);
	}


	return this->compute_next_timestep();
}
#endif

template <std::size_t m_N, std::size_t storage_N>
unsigned matrix_solver_SOR_mat_t<m_N, storage_N>::vsolve_non_dynamic(const bool newton_raphson)
{
	/* The matrix based code looks a lot nicer but actually is 30% slower than
	 * the optimized code which works directly on the data structures.
	 * Need something like that for gaussian elimination as well.
	 */


	nl_double new_v[storage_N] = { 0.0 };
	const std::size_t iN = this->N();

	matrix_solver_t::build_LE_A<matrix_solver_SOR_mat_t>();
	matrix_solver_t::build_LE_RHS<matrix_solver_SOR_mat_t>();

	bool resched = false;

	unsigned resched_cnt = 0;


#if 0
	static int ws_cnt = 0;
	ws_cnt++;
	if (1 && ws_cnt % 200 == 0)
	{
		// update omega
		nl_double lambdaN = 0;
		nl_double lambda1 = 1e9;
		for (int k = 0; k < iN; k++)
		{
	#if 0
			nl_double akk = std::abs(this->m_A[k][k]);
			if ( akk > lambdaN)
				lambdaN = akk;
			if (akk < lambda1)
				lambda1 = akk;
	#else
			nl_double akk = std::abs(this->m_A[k][k]);
			nl_double s = 0.0;
			for (int i=0; i<iN; i++)
				s = s + std::abs(this->m_A[k][i]);
			akk = s / akk - 1.0;
			if ( akk > lambdaN)
				lambdaN = akk;
			if (akk < lambda1)
				lambda1 = akk;
	#endif
		}
		//printf("lambda: %f %f\n", lambda, 2.0 / (1.0 + 2 * sqrt(lambda)) );

		//ws = 2.0 / (2.0 - lambdaN - lambda1);
		m_omega = 2.0 / (2.0 - lambda1);
		//printf("%f %f %f\n", m_omega, lambda1, lambdaN);
	}
#endif

	for (std::size_t k = 0; k < iN; k++)
		new_v[k] = this->m_nets[k]->Q_Analog();

	do {
		resched = false;
		nl_double cerr = 0.0;

		for (std::size_t k = 0; k < iN; k++)
		{
			nl_double Idrive = 0;

			const auto *p = this->m_terms[k]->m_nz.data();
			const std::size_t e = this->m_terms[k]->m_nz.size();

			for (std::size_t i = 0; i < e; i++)
				Idrive = Idrive + this->A(k,p[i]) * new_v[p[i]];

			const nl_double delta = m_omega * (this->RHS(k) - Idrive) / this->A(k,k);
			cerr = std::max(cerr, std::abs(delta));
			new_v[k] += delta;
		}

		if (cerr > this->m_params.m_accuracy)
		{
			resched = true;
		}
		resched_cnt++;
	} while (resched && (resched_cnt < this->m_params.m_gs_loops));

	this->m_stat_calculations++;
	this->m_iterative_total += resched_cnt;
	this->m_gs_total += resched_cnt;

	if (resched)
	{
		this->m_iterative_fail++;
		//this->netlist().warning("Falling back to direct solver .. Consider increasing RESCHED_LOOPS");
		this->m_gs_fail++;

		return matrix_solver_direct_t<m_N, storage_N>::solve_non_dynamic(newton_raphson);
	}
	else {
		this->store(new_v);
		return resched_cnt;
	}

}

	} //namespace devices
} // namespace netlist

#endif /* NLD_MS_GAUSS_SEIDEL_H_ */
