// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * palloc.h
 *
 */

#ifndef PALLOC_H_
#define PALLOC_H_

#include "pstring.h"

#include <vector>
#include <memory>

namespace plib {

//============================================================
//  Memory allocation
//============================================================

template<typename T, typename... Args>
T *palloc(Args&&... args)
{
	return new T(std::forward<Args>(args)...);
}

template<typename T>
void pfree(T *ptr)
{
	delete ptr;
}

template<typename T>
T* palloc_array(const std::size_t num)
{
	return new T[num]();
}

template<typename T>
void pfree_array(T *ptr)
{
	delete [] ptr;
}

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
	return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

template<typename BC, typename DC, typename... Args>
static std::unique_ptr<BC> make_unique_base(Args&&... args)
{
	std::unique_ptr<BC> ret(new DC(std::forward<Args>(args)...));
	return ret;
}

template <typename SC>
class owned_ptr
{
private:
	owned_ptr()
	: m_ptr(nullptr), m_is_owned(true) { }
public:
	owned_ptr(SC *p, bool owned) noexcept
	: m_ptr(p), m_is_owned(owned)
	{ }

	owned_ptr(const owned_ptr &r) = delete;
	owned_ptr & operator =(owned_ptr &r) = delete;

	template<typename DC>
	owned_ptr & operator =(owned_ptr<DC> &&r)
	{
		if (m_is_owned && (m_ptr != nullptr))
			delete m_ptr;
		m_is_owned = r.m_is_owned;
		m_ptr = r.m_ptr;
		r.m_is_owned = false;
		r.m_ptr = nullptr;
		return *this;
	}

	owned_ptr(owned_ptr &&r) noexcept
	{
		m_is_owned = r.m_is_owned;
		m_ptr = r.m_ptr;
		r.m_is_owned = false;
		r.m_ptr = nullptr;
	}

	template<typename DC>
	owned_ptr(owned_ptr<DC> &&r) noexcept
	{
		m_ptr = static_cast<SC *>(r.get());
		m_is_owned = r.is_owned();
		r.release();
	}

	~owned_ptr()
	{
		if (m_is_owned && (m_ptr != nullptr))
			delete m_ptr;
		m_is_owned = false;
		m_ptr = nullptr;
	}
	template<typename DC, typename... Args>
	static owned_ptr Create(Args&&... args)
	{
		owned_ptr a;
		DC *x = new DC(std::forward<Args>(args)...);
		a.m_ptr = static_cast<SC *>(x);
		return std::move(a);
	}

	template<typename... Args>
	static owned_ptr Create(Args&&... args)
	{
		owned_ptr a;
		a.m_ptr = new SC(std::forward<Args>(args)...);
		return std::move(a);
	}
	SC * release()
	{
		SC *tmp = m_ptr;
		m_is_owned = false;
		m_ptr = nullptr;
		return tmp;
	}

	bool is_owned() const { return m_is_owned; }

	SC * operator ->() const { return m_ptr; }
	SC & operator *() const { return *m_ptr; }
	SC * get() const { return m_ptr; }
private:
	SC *m_ptr;
	bool m_is_owned;
};

class mempool
{
private:
	struct block
	{
		block() : m_num_alloc(0), m_free(0), cur_ptr(nullptr), data(nullptr) { }
		std::size_t m_num_alloc;
		std::size_t m_free;
		char *cur_ptr;
		char *data;
	};

	size_t new_block();
	size_t mininfosize();

	struct info
	{
		info() : m_block(0) { }
		size_t m_block;
	};

	size_t m_min_alloc;
	size_t m_min_align;

	std::vector<block> m_blocks;

public:
	mempool(size_t min_alloc, size_t min_align);
	~mempool();

	void *alloc(size_t size);
	void free(void *ptr);

};

}

#endif /* PALLOC_H_ */
