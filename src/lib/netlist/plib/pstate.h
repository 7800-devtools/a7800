// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * pstate.h
 *
 */

#ifndef PSTATE_H_
#define PSTATE_H_

#include "pstring.h"
#include "ptypes.h"

#include <vector>
#include <memory>

// ----------------------------------------------------------------------------------------
// state saving ...
// ----------------------------------------------------------------------------------------

namespace plib {
class state_manager_t
{
public:

	struct datatype_t
	{
		datatype_t(std::size_t bsize, bool bintegral, bool bfloat)
		: size(bsize), is_integral(bintegral), is_float(bfloat), is_custom(false)
		{}
		explicit datatype_t(bool bcustom)
		: size(0), is_integral(false), is_float(false), is_custom(bcustom)
		{}

		const std::size_t size;
		const bool is_integral;
		const bool is_float;
		const bool is_custom;
	};

	template<typename T> struct datatype_f
	{
		static inline const datatype_t f()
		{
			return datatype_t(sizeof(T),
					plib::is_integral<T>::value || std::is_enum<T>::value,
					std::is_floating_point<T>::value);
		}
	};

	class callback_t
	{
	public:
		using list_t = std::vector<callback_t *>;

		virtual ~callback_t();

		virtual void register_state(state_manager_t &manager, const pstring &module) = 0;
		virtual void on_pre_save() = 0;
		virtual void on_post_load() = 0;
	protected:
	};

	struct entry_t
	{
		using list_t = std::vector<std::unique_ptr<entry_t>>;

		entry_t(const pstring &stname, const datatype_t &dt, const void *owner,
				const std::size_t count, void *ptr)
		: m_name(stname), m_dt(dt), m_owner(owner), m_callback(nullptr), m_count(count), m_ptr(ptr) { }

		entry_t(const pstring &stname, const void *owner, callback_t *callback)
		: m_name(stname), m_dt(datatype_t(true)), m_owner(owner), m_callback(callback), m_count(0), m_ptr(nullptr) { }

		~entry_t() { }

		pstring             m_name;
		const datatype_t    m_dt;
		const void *        m_owner;
		callback_t *        m_callback;
		const std::size_t   m_count;
		void *              m_ptr;
	};

	state_manager_t();
	~state_manager_t();

	template<typename C> void save_item(const void *owner, C &state, const pstring &stname)
	{
		save_state_ptr( owner, stname, datatype_f<C>::f(), 1, &state);
	}

	template<typename C, std::size_t N> void save_item(const void *owner, C (&state)[N], const pstring &stname)
	{
		save_state_ptr(owner, stname, datatype_f<C>::f(), N, &(state[0]));
	}

	template<typename C> void save_item(const void *owner, C *state, const pstring &stname, const std::size_t count)
	{
		save_state_ptr(owner, stname, datatype_f<C>::f(), count, state);
	}

	template<typename C>
	void save_item(const void *owner, std::vector<C> &v, const pstring &stname)
	{
		save_state(v.data(), owner, stname, v.size());
	}

	void pre_save();
	void post_load();
	void remove_save_items(const void *owner);

	const entry_t::list_t &save_list() const { return m_save; }

	void save_state_ptr(const void *owner, const pstring &stname, const datatype_t &dt, const std::size_t count, void *ptr);

protected:

private:
	entry_t::list_t m_save;
	entry_t::list_t m_custom;

};

template<> void state_manager_t::save_item(const void *owner, callback_t &state, const pstring &stname);

}

#endif /* PSTATE_H_ */
