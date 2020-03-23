// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * ptypes.h
 *
 */

#ifndef PTYPES_H_
#define PTYPES_H_

#include "pconfig.h"
#include "pstring.h"

#include <type_traits>
#include <limits>

namespace plib
{
	template<typename T> struct is_integral : public std::is_integral<T> { };
	template<typename T> struct numeric_limits : public std::numeric_limits<T> { };

	/* 128 bit support at least on GCC is not fully supported */
#if PHAS_INT128
	template<> struct is_integral<UINT128> { static constexpr bool value = true; };
	template<> struct is_integral<INT128> { static constexpr bool value = true; };
	template<> struct numeric_limits<UINT128>
	{
		static inline constexpr UINT128 max()
		{
			return ~((UINT128)0);
		}
	};
	template<> struct numeric_limits<INT128>
	{
		static inline constexpr INT128 max()
		{
			return (~((UINT128)0)) >> 1;
		}
	};
#endif

	//============================================================
	// prevent implicit copying
	//============================================================

	struct nocopyassignmove
	{
	protected:
		nocopyassignmove() = default;
		~nocopyassignmove() = default;
	private:
		nocopyassignmove(const nocopyassignmove &) = delete;
		nocopyassignmove(nocopyassignmove &&) = delete;
		nocopyassignmove &operator=(const nocopyassignmove &) = delete;
		nocopyassignmove &operator=(nocopyassignmove &&) = delete;
	};

	struct nocopyassign
	{
	protected:
		nocopyassign() = default;
		~nocopyassign() = default;
	private:
		nocopyassign(const nocopyassign &) = delete;
		nocopyassign &operator=(const nocopyassign &) = delete;
	};

	//============================================================
	//  penum - strongly typed enumeration
	//============================================================

	struct penum_base
	{
	protected:
		static int from_string_int(const char *str, const char *x);
		static pstring nthstr(int n, const char *str);
	};

}

#define P_ENUM(ename, ...) \
	struct ename : public plib::penum_base { \
		enum E { __VA_ARGS__ }; \
		ename (E v) : m_v(v) { } \
		bool set_from_string (const pstring &s) { \
			static const char *strings = # __VA_ARGS__; \
			int f = from_string_int(strings, s.c_str()); \
			if (f>=0) { m_v = static_cast<E>(f); return true; } else { return false; } \
		} \
		operator E() const {return m_v;} \
		bool operator==(const ename &rhs) const {return m_v == rhs.m_v;} \
		bool operator==(const E &rhs) const {return m_v == rhs;} \
		const pstring name() const { \
			static const char *strings = # __VA_ARGS__; \
			return nthstr(static_cast<int>(m_v), strings); \
		} \
		private: E m_v; };


#endif /* PTYPES_H_ */
