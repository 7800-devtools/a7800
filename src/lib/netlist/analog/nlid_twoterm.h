// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * nld_twoterm.h
 *
 * Devices with two terminals ...
 *
 *
 *       (k)
 *  +-----T-----+
 *  |     |     |
 *  |  +--+--+  |
 *  |  |     |  |
 *  |  R     |  |
 *  |  R     |  |
 *  |  R     I  |
 *  |  |     I  |  Device n
 *  |  V+    I  |
 *  |  V     |  |
 *  |  V-    |  |
 *  |  |     |  |
 *  |  +--+--+  |
 *  |     |     |
 *  +-----T-----+
 *       (l)
 *
 *  This is a resistance in series to a voltage source and paralleled by a
 *  current source. This is suitable to model voltage sources, current sources,
 *  resistors, capacitors, inductances and diodes.
 *
 */

#ifndef NLID_TWOTERM_H_
#define NLID_TWOTERM_H_

#include "../nl_base.h"
#include "../plib/pfunction.h"

// -----------------------------------------------------------------------------
// Implementation
// -----------------------------------------------------------------------------

namespace netlist
{
	namespace analog
	{
// -----------------------------------------------------------------------------
// nld_twoterm
// -----------------------------------------------------------------------------

NETLIB_OBJECT(twoterm)
{
	NETLIB_CONSTRUCTOR_EX(twoterm, bool terminals_owned = false)
	, m_P(bselect(terminals_owned, owner, *this), (terminals_owned ? name + "." : "") + "1")
	, m_N(bselect(terminals_owned, owner, *this), (terminals_owned ? name + "." : "") + "2")
	{
		m_P.m_otherterm = &m_N;
		m_N.m_otherterm = &m_P;
	}

	terminal_t m_P;
	terminal_t m_N;

	//NETLIB_UPDATE_TERMINALSI() { }
	//NETLIB_RESETI() { }
	NETLIB_UPDATEI();

public:
	/* inline */ void set(const nl_double G, const nl_double V, const nl_double I)
	{
		/*      GO, GT, I                */
		m_P.set( G,  G, (  V) * G - I);
		m_N.set( G,  G, ( -V) * G + I);
	}

	/* inline */ nl_double deltaV() const
	{
		return m_P.net().Q_Analog() - m_N.net().Q_Analog();
	}

	void set_mat(const nl_double a11, const nl_double a12, const nl_double r1,
				 const nl_double a21, const nl_double a22, const nl_double r2)
	{
		/*      GO, GT, I                */
		m_P.set(-a12, a11, r1);
		m_N.set(-a21, a22, r2);
	}

private:
	template <class C>
	static core_device_t &bselect(bool b, C &d1, core_device_t &d2)
	{
		core_device_t *h = dynamic_cast<core_device_t *>(&d1);
		return b ? *h : d2;
	}
};


// -----------------------------------------------------------------------------
// nld_R
// -----------------------------------------------------------------------------

NETLIB_OBJECT_DERIVED(R_base, twoterm)
{
	NETLIB_CONSTRUCTOR_DERIVED(R_base, twoterm)
	{
	}

public:
	inline void set_R(const nl_double R)
	{
		const nl_double G = NL_FCONST(1.0) / R;
		set_mat( G, -G, 0.0,
				-G,  G, 0.0);
	}

protected:
	NETLIB_RESETI();
	NETLIB_UPDATEI();

};

NETLIB_OBJECT_DERIVED(R, R_base)
{
	NETLIB_CONSTRUCTOR_DERIVED(R, R_base)
	, m_R(*this, "R", 1e9)
	{
	}

	param_double_t m_R;

protected:

	NETLIB_RESETI();
	//NETLIB_UPDATEI() { }
	NETLIB_UPDATE_PARAMI();

private:
	/* protect set_R ... it's a recipe to desaster when used to bypass the parameter */
	using NETLIB_NAME(R_base)::set_R;
};

// -----------------------------------------------------------------------------
// nld_POT
// -----------------------------------------------------------------------------

NETLIB_OBJECT(POT)
{
	NETLIB_CONSTRUCTOR(POT)
	, m_R1(*this, "_R1")
	, m_R2(*this, "_R2")
	, m_R(*this, "R", 10000)
	, m_Dial(*this, "DIAL", 0.5)
	, m_DialIsLog(*this, "DIALLOG", 0)
	{
		register_subalias("1", m_R1.m_P);
		register_subalias("2", m_R1.m_N);
		register_subalias("3", m_R2.m_N);

		connect(m_R2.m_P, m_R1.m_N);

	}

	//NETLIB_UPDATEI();
	NETLIB_RESETI();
	NETLIB_UPDATE_PARAMI();

private:
	NETLIB_SUB(R_base) m_R1;
	NETLIB_SUB(R_base) m_R2;

	param_double_t m_R;
	param_double_t m_Dial;
	param_logic_t m_DialIsLog;
};

NETLIB_OBJECT(POT2)
{
	NETLIB_CONSTRUCTOR(POT2)
	, m_R1(*this, "_R1")
	, m_R(*this, "R", 10000)
	, m_Dial(*this, "DIAL", 0.5)
	, m_DialIsLog(*this, "DIALLOG", 0)
	, m_Reverse(*this, "REVERSE", 0)
	{
		register_subalias("1", m_R1.m_P);
		register_subalias("2", m_R1.m_N);

	}

	//NETLIB_UPDATEI();
	NETLIB_RESETI();
	NETLIB_UPDATE_PARAMI();

private:
	NETLIB_SUB(R_base) m_R1;

	param_double_t m_R;
	param_double_t m_Dial;
	param_logic_t m_DialIsLog;
	param_logic_t m_Reverse;
};


// -----------------------------------------------------------------------------
// nld_C
// -----------------------------------------------------------------------------

NETLIB_OBJECT_DERIVED(C, twoterm)
{
public:
	NETLIB_CONSTRUCTOR_DERIVED(C, twoterm)
	, m_C(*this, "C", 1e-6)
	, m_GParallel(0.0)
	{
		//register_term("1", m_P);
		//register_term("2", m_N);
	}

	NETLIB_IS_TIMESTEP(true)
	NETLIB_TIMESTEPI();

	param_double_t m_C;

protected:
	NETLIB_RESETI();
	NETLIB_UPDATEI();
	NETLIB_UPDATE_PARAMI();

private:
	nl_double m_GParallel;

};

// -----------------------------------------------------------------------------
// nld_L
// -----------------------------------------------------------------------------

NETLIB_OBJECT_DERIVED(L, twoterm)
{
public:
	NETLIB_CONSTRUCTOR_DERIVED(L, twoterm)
	, m_L(*this, "L", 1e-6)
	, m_GParallel(0.0)
	, m_G(0.0)
	, m_I(0.0)
	{
		//register_term("1", m_P);
		//register_term("2", m_N);
	}

	NETLIB_IS_TIMESTEP(true)
	NETLIB_TIMESTEPI();

	param_double_t m_L;

protected:
	NETLIB_RESETI();
	NETLIB_UPDATEI();
	NETLIB_UPDATE_PARAMI();

private:
	nl_double m_GParallel;
	nl_double m_G;
	nl_double m_I;
};

// -----------------------------------------------------------------------------
// A generic diode model to be used in other devices (Diode, BJT ...)
// -----------------------------------------------------------------------------

class generic_diode
{
public:
	generic_diode(device_t &dev, pstring name);

	void update_diode(const double nVd);

	void set_param(const double Is, const double n, double gmin);

	double I() const { return m_Id; }
	double G() const { return m_G; }
	double Ieq() const { return (m_Id - m_Vd * m_G); }
	double Vd() const { return m_Vd; }

	/* owning object must save those ... */

private:
	state_var<double> m_Vd;
	state_var<double> m_Id;
	state_var<double> m_G;

	double m_Vt;
	double m_Vmin;
	double m_Is;
	double m_logIs;
	double m_n;
	double m_gmin;

	double m_VtInv;
	double m_Vcrit;
};

/*! Class representing the diode model paramers.
 *  This is the model representation of the diode model. Typically, SPICE uses
 *  the following parameters. A "Y" in the first column indicates that the
 *  parameter is actually used in netlist.
 *
 *   |NL? |name  |parameter                        |units|default| example|area  |
 *   |:--:|:-----|:--------------------------------|:----|------:|-------:|:----:|
 *   | Y  |IS    |saturation current               |A    |1.0e-14| 1.0e-14|   *  |
 *   |    |RS    |ohmic resistanc                  |Ohm  |      0|      10|   *  |
 *   | Y  |N     |emission coefficient             |-    |      1|       1|      |
 *   |    |TT    |transit-time                     |sec  |      0|   0.1ns|      |
 *   |    |CJO   |zero-bias junction capacitance   |F    |      0|     2pF|   *  |
 *   |    |VJ    |junction potential               |V    |      1|     0.6|      |
 *   |    |M     |grading coefficient              |-    |    0.5|     0.5|      |
 *   |    |EG    |band-gap energy                  |eV   |   1.11| 1.11 Si|      |
 *   |    |XTI   |saturation-current temp.exp      |-    |      3|3.0 pn. 2.0 Schottky| |
 *   |    |KF    |flicker noise coefficient        |-    |      0|        |      |
 *   |    |AF    |flicker noise exponent           |-    |      1|        |      |
 *   |    |FC    |coefficient for forward-bias depletion capacitance formula|-|0.5|| |
 *   |    |BV    |reverse breakdown voltage        |V    |infinite|     40|      |
 *   |    |IBV   |current at breakdown voltage     |V    |  0.001|        |      |
 *   |    |TNOM  |parameter measurement temperature|deg C|     27|      50|      |
 *
 */

class diode_model_t : public param_model_t
{
public:
	diode_model_t(device_t &device, const pstring &name, const pstring &val)
	: param_model_t(device, name, val)
	, m_IS(*this, "IS")
	, m_N(*this, "N")
	{}

	value_t m_IS;    //!< saturation current.
	value_t m_N;     //!< emission coefficient.
};


// -----------------------------------------------------------------------------
// nld_D
// -----------------------------------------------------------------------------

NETLIB_OBJECT_DERIVED(D, twoterm)
{
public:
	NETLIB_CONSTRUCTOR_DERIVED(D, twoterm)
	, m_model(*this, "MODEL", "D")
	, m_D(*this, "m_D")
	{
		register_subalias("A", m_P);
		register_subalias("K", m_N);
	}

	template <class CLASS>
	NETLIB_NAME(D)(CLASS &owner, const pstring &name, const pstring &model)
	: NETLIB_NAME(twoterm)(owner, name)
	, m_model(*this, "MODEL", model)
	, m_D(*this, "m_D")
	{
		register_subalias("A", m_P);
		register_subalias("K", m_N);
	}

	NETLIB_IS_DYNAMIC(true)
	NETLIB_UPDATE_TERMINALSI();

	diode_model_t m_model;

protected:
	NETLIB_RESETI();
	NETLIB_UPDATEI();
	NETLIB_UPDATE_PARAMI();

	generic_diode m_D;
};


// -----------------------------------------------------------------------------
// nld_VS - Voltage source
//
// netlist voltage source must have inner resistance
// -----------------------------------------------------------------------------

NETLIB_OBJECT_DERIVED(VS, twoterm)
{
public:
	NETLIB_CONSTRUCTOR_DERIVED(VS, twoterm)
	, m_R(*this, "R", 0.1)
	, m_V(*this, "V", 0.0)
	, m_func(*this,"FUNC", "")
	, m_compiled(this->name() + ".FUNCC", this, this->netlist().state())
	{
		register_subalias("P", m_P);
		register_subalias("N", m_N);
		if (m_func() != "")
			m_compiled.compile_postfix(std::vector<pstring>({{"T"}}), m_func());
	}

	NETLIB_IS_TIMESTEP(m_func() != "")
	NETLIB_TIMESTEPI();

protected:
	NETLIB_UPDATEI();
	NETLIB_RESETI();

	param_double_t m_R;
	param_double_t m_V;
	param_str_t m_func;
	plib::pfunction m_compiled;
};

// -----------------------------------------------------------------------------
// nld_CS - Current source
// -----------------------------------------------------------------------------

NETLIB_OBJECT_DERIVED(CS, twoterm)
{
public:
	NETLIB_CONSTRUCTOR_DERIVED(CS, twoterm)
	, m_I(*this, "I", 1.0)
	, m_func(*this,"FUNC", "")
	, m_compiled(this->name() + ".FUNCC", this, this->netlist().state())
	{
		register_subalias("P", m_P);
		register_subalias("N", m_N);
		if (m_func() != "")
			m_compiled.compile_postfix(std::vector<pstring>({{"T"}}), m_func());
	}

	NETLIB_IS_TIMESTEP(m_func() != "")
	NETLIB_TIMESTEPI();
protected:

	NETLIB_UPDATEI();
	NETLIB_RESETI();

	param_double_t m_I;
	param_str_t m_func;
	plib::pfunction m_compiled;
};


	} //namespace devices
} // namespace netlist

#endif /* NLD_TWOTERM_H_ */
