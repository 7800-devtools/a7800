// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * palloc.c
 *
 */

#include "pfunction.h"
#include "pfmtlog.h"
#include "putil.h"
#include "pexception.h"

#include <cmath>
#include <stack>

namespace plib {

void pfunction::compile(const std::vector<pstring> &inputs, const pstring &expr)
{
	if (expr.startsWith("rpn:"))
		compile_postfix(inputs, expr.substr(4));
	else
		compile_infix(inputs, expr);
}

void pfunction::compile_postfix(const std::vector<pstring> &inputs, const pstring &expr)
{
	std::vector<pstring> cmds(plib::psplit(expr, " "));
	compile_postfix(inputs, cmds, expr);
}

void pfunction::compile_postfix(const std::vector<pstring> &inputs,
		const std::vector<pstring> &cmds, const pstring &expr)
{
	m_precompiled.clear();
	int stk = 0;

	for (const pstring &cmd : cmds)
	{
		rpn_inst rc;
		if (cmd == "+")
			{ rc.m_cmd = ADD; stk -= 1; }
		else if (cmd == "-")
			{ rc.m_cmd = SUB; stk -= 1; }
		else if (cmd == "*")
			{ rc.m_cmd = MULT; stk -= 1; }
		else if (cmd == "/")
			{ rc.m_cmd = DIV; stk -= 1; }
		else if (cmd == "pow")
			{ rc.m_cmd = POW; stk -= 1; }
		else if (cmd == "sin")
			{ rc.m_cmd = SIN; stk -= 0; }
		else if (cmd == "cos")
			{ rc.m_cmd = COS; stk -= 0; }
		else if (cmd == "rand")
			{ rc.m_cmd = RAND; stk += 1; }
		else
		{
			for (unsigned i = 0; i < inputs.size(); i++)
			{
				if (inputs[i] == cmd)
				{
					rc.m_cmd = PUSH_INPUT;
					rc.m_param = i;
					stk += 1;
					break;
				}
			}
			if (rc.m_cmd != PUSH_INPUT)
			{
				bool err = false;
				rc.m_cmd = PUSH_CONST;
				rc.m_param = cmd.as_double(&err);
				if (err)
					throw plib::pexception(plib::pfmt("nld_function: unknown/misformatted token <{1}> in <{2}>")(cmd)(expr));
				stk += 1;
			}
		}
		if (stk < 1)
			throw plib::pexception(plib::pfmt("nld_function: stack underflow on token <{1}> in <{2}>")(cmd)(expr));
		m_precompiled.push_back(rc);
	}
	if (stk != 1)
		throw plib::pexception(plib::pfmt("nld_function: stack count different to one on <{2}>")(expr));
}

static int get_prio(pstring v)
{
	if (v == "(" || v == ")")
		return 1;
	else if (v.left(1) >= "a" && v.left(1) <= "z")
		return 0;
	else if (v == "*" || v == "/")
		return 20;
	else if (v == "+" || v == "-")
		return 10;
	else if (v == "^")
		return 30;
	else
		return -1;
}

static pstring pop_check(std::stack<pstring> &stk, const pstring &expr)
{
	if (stk.size() == 0)
		throw plib::pexception(plib::pfmt("nld_function: stack underflow during infix parsing of: <{1}>")(expr));
	pstring res = stk.top();
	stk.pop();
	return res;
}

void pfunction::compile_infix(const std::vector<pstring> &inputs, const pstring &expr)
{
	// Shunting-yard infix parsing
	std::vector<pstring> sep = {"(", ")", ",", "*", "/", "+", "-", "^"};
	std::vector<pstring> sexpr(plib::psplit(expr.replace_all(" ",""), sep));
	std::stack<pstring> opstk;
	std::vector<pstring> postfix;

	//printf("dbg: %s\n", expr.c_str());
	for (unsigned i = 0; i < sexpr.size(); i++)
	{
		pstring &s = sexpr[i];
		if (s=="(")
			opstk.push(s);
		else if (s==")")
		{
			pstring x = pop_check(opstk, expr);
			while (x != "(")
			{
				postfix.push_back(x);
				x = pop_check(opstk, expr);
			}
			if (opstk.size() > 0 && get_prio(opstk.top()) == 0)
				postfix.push_back(pop_check(opstk, expr));
		}
		else if (s==",")
		{
			pstring x = pop_check(opstk, expr);
			while (x != "(")
			{
				postfix.push_back(x);
				x = pop_check(opstk, expr);
			}
			opstk.push(x);
		}
		else {
			int p = get_prio(s);
			if (p>0)
			{
				if (opstk.size() == 0)
					opstk.push(s);
				else
				{
					if (get_prio(opstk.top()) >= get_prio(s))
						postfix.push_back(pop_check(opstk, expr));
					opstk.push(s);
				}
			}
			else if (p == 0) // Function or variable
			{
				if (sexpr[i+1] == "(")
					opstk.push(s);
				else
					postfix.push_back(s);
			}
			else
				postfix.push_back(s);
		}
	}
	while (opstk.size() > 0)
	{
		postfix.push_back(opstk.top());
		opstk.pop();
	}
	compile_postfix(inputs, postfix, expr);
}


#define ST1 stack[ptr]
#define ST2 stack[ptr-1]

#define OP(OP, ADJ, EXPR) \
case OP: \
	ptr-=ADJ; \
	stack[ptr-1] = EXPR; \
	break;

double pfunction::evaluate(const std::vector<double> &values)
{
	double stack[20];
	unsigned ptr = 0;
	for (auto &rc : m_precompiled)
	{
		switch (rc.m_cmd)
		{
			OP(ADD,  1, ST2 + ST1)
			OP(MULT, 1, ST2 * ST1)
			OP(SUB,  1, ST2 - ST1)
			OP(DIV,  1, ST2 / ST1)
			OP(POW,  1, std::pow(ST2, ST1))
			OP(SIN,  0, std::sin(ST2));
			OP(COS,  0, std::cos(ST2));
			case RAND:
				stack[ptr++] = lfsr_random();
				break;
			case PUSH_INPUT:
				stack[ptr++] = values[static_cast<unsigned>(rc.m_param)];
				break;
			case PUSH_CONST:
				stack[ptr++] = rc.m_param;
				break;
		}
	}
	return stack[ptr-1];
}

}
