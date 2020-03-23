// license:BSD-3-Clause
// copyright-holders:Aaron Giles, Vas Crabb
/***************************************************************************

    coretmpl.h

    Core templates for basic non-string types.

***************************************************************************/
#ifndef MAME_UTIL_CORETMPL_H
#define MAME_UTIL_CORETMPL_H

#pragma once

#include "osdcomm.h"
#include "osdcore.h"
#include "corealloc.h"

#include <array>
#include <cassert>
#include <cstddef>
#include <functional>
#include <initializer_list>
#include <iterator>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>


// ======================> simple_list

// a simple_list is a singly-linked list whose 'next' pointer is owned
// by the object
template<class _ElementType>
class simple_list final
{
public:
	class auto_iterator
	{
	public:
		typedef int difference_type;
		typedef _ElementType value_type;
		typedef _ElementType *pointer;
		typedef _ElementType &reference;
		typedef std::forward_iterator_tag iterator_category;

		// construction/destruction
		auto_iterator() noexcept : m_current(nullptr) { }
		auto_iterator(_ElementType *ptr) noexcept : m_current(ptr) { }

		// required operator overloads
		bool operator==(const auto_iterator &iter) const noexcept { return m_current == iter.m_current; }
		bool operator!=(const auto_iterator &iter) const noexcept { return m_current != iter.m_current; }
		_ElementType &operator*() const noexcept { return *m_current; }
		_ElementType *operator->() const noexcept { return m_current; }
		// note that _ElementType::next() must not return a const ptr
		auto_iterator &operator++() noexcept { m_current = m_current->next(); return *this; }
		auto_iterator operator++(int) noexcept { auto_iterator result(*this); m_current = m_current->next(); return result; }

	private:
		// private state
		_ElementType *m_current;
	};

	// construction/destruction
	simple_list() noexcept
		: m_head(nullptr)
		, m_tail(nullptr)
		, m_count(0)
	{
	}
	~simple_list() noexcept { reset(); }

	// we don't support deep copying
	simple_list(const simple_list &) = delete;
	simple_list &operator=(const simple_list &) = delete;

	// but we do support cheap swap/move
	simple_list(simple_list &&list) : simple_list() { operator=(std::move(list)); }
	simple_list &operator=(simple_list &&list)
	{
		using std::swap;
		swap(m_head, list.m_head);
		swap(m_tail, list.m_tail);
		swap(m_count, list.m_count);
	}

	// simple getters
	_ElementType *first() const noexcept { return m_head; }
	_ElementType *last() const noexcept { return m_tail; }
	int count() const noexcept { return m_count; }
	bool empty() const noexcept { return m_count == 0; }

	// range iterators
	auto_iterator begin() const noexcept { return auto_iterator(m_head); }
	auto_iterator end() const noexcept { return auto_iterator(nullptr); }

	// remove (free) all objects in the list, leaving an empty list
	void reset() noexcept
	{
		while (m_head != nullptr)
			remove(*m_head);
	}

	// add the given object to the head of the list
	_ElementType &prepend(_ElementType &object) noexcept
	{
		object.m_next = m_head;
		m_head = &object;
		if (m_tail == nullptr)
			m_tail = m_head;
		m_count++;
		return object;
	}

	// add the given list to the head of the list
	void prepend_list(simple_list<_ElementType> &list) noexcept
	{
		int count = list.count();
		if (count == 0)
			return;
		_ElementType *tail = list.last();
		_ElementType *head = list.detach_all();
		tail->m_next = m_head;
		m_head = head;
		if (m_tail == nullptr)
			m_tail = tail;
		m_count += count;
	}

	// add the given object to the tail of the list
	_ElementType &append(_ElementType &object) noexcept
	{
		object.m_next = nullptr;
		if (m_tail != nullptr)
			m_tail = m_tail->m_next = &object;
		else
			m_tail = m_head = &object;
		m_count++;
		return object;
	}

	// add the given list to the tail of the list
	void append_list(simple_list<_ElementType> &list) noexcept
	{
		int count = list.count();
		if (count == 0)
			return;
		_ElementType *tail = list.last();
		_ElementType *head = list.detach_all();
		if (m_tail != nullptr)
			m_tail->m_next = head;
		else
			m_head = head;
		m_tail = tail;
		m_count += count;
	}

	// insert the given object after a particular object (nullptr means prepend)
	_ElementType &insert_after(_ElementType &object, _ElementType *insert_after) noexcept
	{
		if (insert_after == nullptr)
			return prepend(object);
		object.m_next = insert_after->m_next;
		insert_after->m_next = &object;
		if (m_tail == insert_after)
			m_tail = &object;
		m_count++;
		return object;
	}

	// insert the given object before a particular object (nullptr means append)
	_ElementType &insert_before(_ElementType &object, _ElementType *insert_before) noexcept
	{
		if (insert_before == nullptr)
			return append(object);
		for (_ElementType **curptr = &m_head; *curptr != nullptr; curptr = &(*curptr)->m_next)
			if (*curptr == insert_before)
			{
				object.m_next = insert_before;
				*curptr = &object;
				if (m_head == insert_before)
					m_head = &object;
				m_count++;
				return object;
			}
		return object;
	}

	// replace an item in the list at the same location, and remove it
	_ElementType &replace_and_remove(_ElementType &object, _ElementType &toreplace) noexcept
	{
		_ElementType *prev = nullptr;
		for (_ElementType *cur = m_head; cur != nullptr; prev = cur, cur = cur->m_next)
			if (cur == &toreplace)
			{
				if (prev != nullptr)
					prev->m_next = &object;
				else
					m_head = &object;
				if (m_tail == &toreplace)
					m_tail = &object;
				object.m_next = toreplace.m_next;
				global_free(&toreplace);
				return object;
			}
		return append(object);
	}

	// detach the head item from the list, but don't free its memory
	_ElementType *detach_head() noexcept
	{
		_ElementType *result = m_head;
		if (result != nullptr)
		{
			m_head = result->m_next;
			m_count--;
			if (m_head == nullptr)
				m_tail = nullptr;
		}
		return result;
	}

	// detach the given item from the list, but don't free its memory
	_ElementType &detach(_ElementType &object) noexcept
	{
		_ElementType *prev = nullptr;
		for (_ElementType *cur = m_head; cur != nullptr; prev = cur, cur = cur->m_next)
			if (cur == &object)
			{
				if (prev != nullptr)
					prev->m_next = object.m_next;
				else
					m_head = object.m_next;
				if (m_tail == &object)
					m_tail = prev;
				m_count--;
				return object;
			}
		return object;
	}

	// detach the entire list, returning the head, but don't free memory
	_ElementType *detach_all() noexcept
	{
		_ElementType *result = m_head;
		m_head = m_tail = nullptr;
		m_count = 0;
		return result;
	}

	// remove the given object and free its memory
	void remove(_ElementType &object) noexcept
	{
		global_free(&detach(object));
	}

	// find an object by index in the list
	_ElementType *find(int index) const noexcept
	{
		for (_ElementType *cur = m_head; cur != nullptr; cur = cur->m_next)
			if (index-- == 0)
				return cur;
		return nullptr;
	}

	// return the index of the given object in the list
	int indexof(const _ElementType &object) const noexcept
	{
		int index = 0;
		for (_ElementType *cur = m_head; cur != nullptr; cur = cur->m_next)
		{
			if (cur == &object)
				return index;
			index++;
		}
		return -1;
	}

private:
	// internal state
	_ElementType *  m_head;         // head of the singly-linked list
	_ElementType *  m_tail;         // tail of the singly-linked list
	int             m_count;        // number of objects in the list
};


// ======================> simple_list_wrapper

// a simple_list_wrapper wraps an existing object with a next pointer so it
// can live in a simple_list without requiring the object to have a next
// pointer
template<class _ObjectType>
class simple_list_wrapper
{
public:
	template<class U> friend class simple_list;

	// construction/destruction
	simple_list_wrapper(_ObjectType *object)
		: m_next(nullptr),
			m_object(object) { }

	// operators
	operator _ObjectType *() { return m_object; }
	operator _ObjectType *() const { return m_object; }
	_ObjectType *operator *() { return m_object; }
	_ObjectType *operator *() const { return m_object; }

	// getters
	simple_list_wrapper *next() const { return m_next; }
	_ObjectType *object() const { return m_object; }

private:
	// internal state
	simple_list_wrapper *   m_next;
	_ObjectType *           m_object;
};


// ======================> fixed_allocator

// a fixed_allocator is a simple class that maintains a free pool of objects
template<class _ItemType>
class fixed_allocator
{
	// we don't support deep copying
	fixed_allocator(const fixed_allocator &);
	fixed_allocator &operator=(const fixed_allocator &);

public:
	// construction/destruction
	fixed_allocator() { }

	// allocate a new item, either by recycling an old one, or by allocating a new one
	_ItemType *alloc()
	{
		_ItemType *result = m_freelist.detach_head();
		if (result == nullptr)
			result = global_alloc(_ItemType);
		return result;
	}

	// reclaim an item by adding it to the free list
	void reclaim(_ItemType *item) { if (item != nullptr) m_freelist.append(*item); }
	void reclaim(_ItemType &item) { m_freelist.append(item); }

	// reclaim all items from a list
	void reclaim_all(simple_list<_ItemType> &_list) { m_freelist.append_list(_list); }

private:
	// internal state
	simple_list<_ItemType>  m_freelist;     // list of free objects
};


// ======================> contiguous_sequence_wrapper

namespace util {

using osd::u8;
using osd::u16;
using osd::u32;
using osd::u64;

using osd::s8;
using osd::s16;
using osd::s32;
using osd::s64;


// wraps an existing sequence of values
template<typename T>
class contiguous_sequence_wrapper
{
public:
	typedef std::ptrdiff_t difference_type;
	typedef std::size_t size_type;
	typedef T value_type;
	typedef T &reference;
	typedef const T &const_reference;
	typedef T *pointer;
	typedef T *iterator;
	typedef const T *const_iterator;
	typedef std::reverse_iterator<iterator> reverse_iterator;
	typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

	contiguous_sequence_wrapper(T *ptr, std::size_t size)
		: m_begin(ptr)
		, m_end(ptr + size)
	{
	}

	contiguous_sequence_wrapper(const contiguous_sequence_wrapper &that) = default;

	// iteration
	iterator begin() { return m_begin; }
	const_iterator begin() const { return m_begin; }
	const_iterator cbegin() const { return m_begin; }
	iterator end() { return m_end; }
	const_iterator end() const { return m_end; }
	const_iterator cend() const { return m_end; }

	// reverse iteration
	reverse_iterator rbegin() { return std::reverse_iterator<iterator>(end()); }
	const_reverse_iterator rbegin() const { return std::reverse_iterator<const_iterator>(end()); }
	const_reverse_iterator crbegin() const { return std::reverse_iterator<const_iterator>(cend()); }
	reverse_iterator rend() { return std::reverse_iterator<iterator>(begin()); }
	const_reverse_iterator rend() const { return std::reverse_iterator<iterator>(begin()); }
	const_reverse_iterator crend() const { return std::reverse_iterator<iterator>(begin()); }

	// capacity
	size_type size() const { return m_end - m_begin; }
	size_type max_size() const { return size(); }
	bool empty() const { return size() == 0; }

	// element access
	reference front() { return operator[](0); }
	const_reference front() const { return operator[](0); }
	reference back() { return operator[](size() - 1); }
	const_reference back() const { return operator[](size() - 1); }
	reference operator[] (size_type n) { return m_begin[n]; }
	const_reference operator[] (size_type n) const { return m_begin[n]; }
	reference at(size_type n) { check_in_bounds(n); return operator[](n); }
	const_reference at(size_type n) const { check_in_bounds(n); return operator[](n); }

private:
	iterator m_begin;
	iterator m_end;

	void check_in_bounds(size_type n)
	{
		if (n < 0 || n >= size())
			throw std::out_of_range("invalid contiguous_sequence_wrapper<T> subscript");
	}
};


// LRU cache that behaves like std::map with differences:
// * drops least-recently used items if necessary on insert to prevent size from exceeding max_size
// * operator[], at, insert, emplace and find freshen existing entries
// * iterates from least- to most-recently used rather than in order by key
// * iterators to dropped items are invalidated
// * not all map interfaces implemented
// * copyable and swappable but not movable
// * swap may invalidate past-the-end iterator, other iterators refer to new container
template <typename Key, typename T, typename Compare = std::less<Key>, class Allocator = std::allocator<std::pair<Key const, T> > >
class lru_cache_map
{
private:
	class iterator_compare;
	typedef std::list<std::pair<Key const, T>, Allocator> value_list;
	typedef typename std::allocator_traits<Allocator>::template rebind_alloc<typename value_list::iterator> iterator_allocator_type;
	typedef std::set<typename value_list::iterator, iterator_compare, iterator_allocator_type> iterator_set;

	class iterator_compare
	{
	public:
		typedef std::true_type is_transparent;
		iterator_compare(Compare const &comp) : m_comp(comp) { }
		iterator_compare(iterator_compare const &that) = default;
		iterator_compare(iterator_compare &&that) = default;
		Compare key_comp() const { return m_comp; }
		iterator_compare &operator=(iterator_compare const &that) = default;
		iterator_compare &operator=(iterator_compare &&that) = default;
		bool operator()(typename value_list::iterator const &lhs, typename value_list::iterator const &rhs) const { return m_comp(lhs->first, rhs->first); }
		template <typename K> bool operator()(typename value_list::iterator const &lhs, K const &rhs) const { return m_comp(lhs->first, rhs); }
		template <typename K> bool operator()(K const &lhs, typename value_list::iterator const &rhs) const { return m_comp(lhs, rhs->first); }
	private:
		Compare m_comp;
	};

public:
	typedef Key key_type;
	typedef T mapped_type;
	typedef std::pair<Key const, T> value_type;
	typedef typename value_list::size_type size_type;
	typedef typename value_list::difference_type difference_type;
	typedef Compare key_compare;
	typedef Allocator allocator_type;
	typedef value_type &reference;
	typedef value_type const &const_reference;
	typedef typename std::allocator_traits<Allocator>::pointer pointer;
	typedef typename std::allocator_traits<Allocator>::const_pointer const_pointer;
	typedef typename value_list::iterator iterator;
	typedef typename value_list::const_iterator const_iterator;
	typedef typename value_list::reverse_iterator reverse_iterator;
	typedef typename value_list::const_reverse_iterator const_reverse_iterator;

	explicit lru_cache_map(size_type max_size)
		: lru_cache_map(max_size, key_compare())
	{
	}
	lru_cache_map(size_type max_size, key_compare const &comp, allocator_type const &alloc = allocator_type())
		: m_max_size(max_size)
		, m_size(0U)
		, m_elements(alloc)
		, m_mapping(iterator_compare(comp), iterator_allocator_type(alloc))
	{
		assert(0U < m_max_size);
	}
	lru_cache_map(lru_cache_map const &that)
		: m_max_size(that.m_max_size)
		, m_size(that.m_size)
		, m_elements(that.m_elements)
		, m_mapping(that.m_mapping.key_comp(), that.m_mapping.get_allocator())
	{
		for (iterator it = m_elements.begin(); it != m_elements.end(); ++it)
			m_mapping.insert(it);
		assert(m_elements.size() == m_size);
		assert(m_mapping.size() == m_size);
	}

	allocator_type get_allocator() const { return m_elements.get_allocator(); }

	iterator begin() { return m_elements.begin(); }
	const_iterator begin() const { return m_elements.cbegin(); }
	const_iterator cbegin() const { return m_elements.cbegin(); }
	iterator end() { return m_elements.end(); }
	const_iterator end() const { return m_elements.cend(); }
	const_iterator cend() const { return m_elements.cend(); }
	reverse_iterator rbegin() { return m_elements.rbegin(); }
	const_reverse_iterator rbegin() const { return m_elements.crbegin(); }
	const_reverse_iterator crbegin() const { return m_elements.crbegin(); }
	reverse_iterator rend() { return m_elements.end(); }
	const_reverse_iterator rend() const { return m_elements.crend(); }
	const_reverse_iterator crend() const { return m_elements.crend(); }

	bool empty() const { return !m_size; }
	size_type size() const { return m_size; }
	size_type max_size() const { return m_max_size; }

	mapped_type &operator[](key_type const &key)
	{
		typename iterator_set::iterator existing(m_mapping.lower_bound(key));
		if ((m_mapping.end() != existing) && !m_mapping.key_comp()(key, *existing))
		{
			m_elements.splice(m_elements.cend(), m_elements, *existing);
			return (*existing)->second;
		}
		make_space(existing);
		iterator const inserted(m_elements.emplace(m_elements.end(), std::piecewise_construct, std::forward_as_tuple(key), std::tuple<>()));
		m_mapping.insert(existing, inserted);
		++m_size;
		assert(m_elements.size() == m_size);
		assert(m_mapping.size() == m_size);
		return inserted->second;
	}
	mapped_type &operator[](key_type &&key)
	{
		typename iterator_set::iterator existing(m_mapping.lower_bound(key));
		if ((m_mapping.end() != existing) && !m_mapping.key_comp()(key, *existing))
		{
			m_elements.splice(m_elements.cend(), m_elements, *existing);
			return (*existing)->second;
		}
		make_space(existing);
		iterator const inserted(m_elements.emplace(m_elements.end(), std::piecewise_construct, std::forward_as_tuple(std::move(key)), std::tuple<>()));
		m_mapping.insert(existing, inserted);
		++m_size;
		assert(m_elements.size() == m_size);
		assert(m_mapping.size() == m_size);
		return inserted->second;
	}
	mapped_type &at(key_type const &key)
	{
		typename iterator_set::iterator existing(m_mapping.find(key));
		if (m_mapping.end() != existing)
		{
			m_elements.splice(m_elements.cend(), m_elements, *existing);
			return (*existing)->second;
		}
		else
		{
			throw std::out_of_range("lru_cache_map::at");
		}
	}
	mapped_type const &at(key_type const &key) const
	{
		typename iterator_set::iterator existing(m_mapping.find(key));
		if (m_mapping.end() != existing)
		{
			m_elements.splice(m_elements.cend(), m_elements, *existing);
			return (*existing)->second;
		}
		else
		{
			throw std::out_of_range("lru_cache_map::at");
		}
	}

	void clear()
	{
		m_size = 0U;
		m_elements.clear();
		m_mapping.clear();
	}
	std::pair<iterator, bool> insert(value_type const &value)
	{
		typename iterator_set::iterator existing(m_mapping.lower_bound(value.first));
		if ((m_mapping.end() != existing) && !m_mapping.key_comp()(value.first, *existing))
		{
			m_elements.splice(m_elements.cend(), m_elements, *existing);
			return std::pair<iterator, bool>(*existing, false);
		}
		make_space(existing);
		iterator const inserted(m_elements.emplace(m_elements.end(), value));
		m_mapping.insert(existing, inserted);
		++m_size;
		assert(m_elements.size() == m_size);
		assert(m_mapping.size() == m_size);
		return std::pair<iterator, bool>(inserted, true);
	}
	std::pair<iterator, bool> insert(value_type &&value)
	{
		typename iterator_set::iterator existing(m_mapping.lower_bound(value.first));
		if ((m_mapping.end() != existing) && !m_mapping.key_comp()(value.first, *existing))
		{
			m_elements.splice(m_elements.cend(), m_elements, *existing);
			return std::pair<iterator, bool>(*existing, false);
		}
		make_space(existing);
		iterator const inserted(m_elements.emplace(m_elements.end(), std::move(value)));
		m_mapping.insert(existing, inserted);
		++m_size;
		assert(m_elements.size() == m_size);
		assert(m_mapping.size() == m_size);
		return std::pair<iterator, bool>(inserted, true);
	}
	template <typename P>
	std::enable_if_t<std::is_constructible<value_type, P>::value, std::pair<iterator, bool> > insert(P &&value)
	{
		return emplace(std::forward<P>(value));
	}
	template <typename InputIt>
	void insert(InputIt first, InputIt last)
	{
		while (first != last)
		{
			insert(*first);
			++first;
		}
	}
	void insert(std::initializer_list<value_type> ilist)
	{
		for (value_type const &value : ilist)
			insert(value);
	}
	template <typename... Params>
	std::pair<iterator, bool> emplace(Params &&... args)
	{
		// TODO: is there a more efficient way than depending on value_type being efficiently movable?
		return insert(value_type(std::forward<Params>(args)...));
	}
	iterator erase(const_iterator pos)
	{
		m_mapping.erase(m_elements.erase(pos, pos));
		iterator const result(m_elements.erase(pos));
		--m_size;
		assert(m_elements.size() == m_size);
		assert(m_mapping.size() == m_size);
		return result;
	}
	iterator erase(const_iterator first, const_iterator last)
	{
		iterator pos(m_elements.erase(first, first));
		while (pos != last)
		{
			m_mapping.erase(pos);
			pos = m_elements.erase(pos);
			--m_size;
		}
		assert(m_elements.size() == m_size);
		assert(m_mapping.size() == m_size);
		return pos;
	}
	size_type erase(key_type const &key)
	{
		typename iterator_set::iterator const found(m_mapping.find(key));
		if (m_mapping.end() == found)
		{
			return 0U;
		}
		else
		{
			m_elements.erase(*found);
			m_mapping.erase(found);
			--m_size;
			assert(m_elements.size() == m_size);
			assert(m_mapping.size() == m_size);
			return 1U;
		}
	}
	void swap(lru_cache_map &that)
	{
		using std::swap;
		swap(m_max_size, that.m_max_size);
		swap(m_size, that.m_size);
		swap(m_elements, that.m_elements);
		swap(m_mapping, that.m_mapping);
	}

	size_type count(key_type const &key) const
	{
		// TODO: perhaps this should freshen an element
		return m_mapping.count(key);
	}
	template <typename K>
	size_type count(K const &x) const
	{
		// FIXME: should only enable this overload if Compare::is_transparent
		// TODO: perhaps this should freshen an element
		return m_mapping.count(x);
	}
	iterator find(key_type const &key)
	{
		typename iterator_set::const_iterator const found(m_mapping.find(key));
		if (m_mapping.end() == found)
		{
			return m_elements.end();
		}
		else
		{
			m_elements.splice(m_elements.cend(), m_elements, *found);
			return *found;
		}
	}
	iterator find(key_type const &key) const
	{
		typename iterator_set::const_iterator const found(m_mapping.find(key));
		if (m_mapping.end() == found)
		{
			return m_elements.end();
		}
		else
		{
			m_elements.splice(m_elements.cend(), m_elements, *found);
			return *found;
		}
	}
	template <typename K>
	iterator find(K const &x)
	{
		// FIXME: should only enable this overload if Compare::is_transparent
		typename iterator_set::const_iterator const found(m_mapping.find(x));
		if (m_mapping.end() == found)
		{
			return m_elements.end();
		}
		else
		{
			m_elements.splice(m_elements.cend(), m_elements, *found);
			return *found;
		}
	}
	template <typename K>
	iterator find(K const &x) const
	{
		// FIXME: should only enable this overload if Compare::is_transparent
		typename iterator_set::const_iterator const found(m_mapping.find(x));
		if (m_mapping.end() == found)
		{
			return m_elements.end();
		}
		else
		{
			m_elements.splice(m_elements.cend(), m_elements, *found);
			return *found;
		}
	}

	key_compare key_comp() const
	{
		return m_mapping.key_comp().key_comp();
	}

	lru_cache_map &operator=(lru_cache_map const &that)
	{
		m_max_size = that.m_max_size;
		m_size = that.m_size;
		m_elements = that.m_elements;
		m_mapping.clear();
		for (iterator it = m_elements.begin(); it != m_elements.end(); ++it)
			m_mapping.insert(it);
		assert(m_elements.size() == m_size);
		assert(m_mapping.size() == m_size);
	}

private:
	void make_space(typename iterator_set::iterator &existing)
	{
		while (m_max_size <= m_size)
		{
			if ((m_mapping.end() != existing) && (m_elements.begin() == *existing))
				existing = m_mapping.erase(existing);
			else
				m_mapping.erase(m_elements.begin());
			m_elements.erase(m_elements.begin());
			--m_size;
		}
	}

	size_type           m_max_size;
	size_type           m_size;
	mutable value_list  m_elements;
	iterator_set        m_mapping;
};

template <typename Key, typename T, typename Compare, class Allocator>
void swap(lru_cache_map<Key, T, Compare, Allocator> &lhs, lru_cache_map<Key, T, Compare, Allocator> &rhs)
{
	lhs.swap(rhs);
}


template <typename T, std::size_t N, bool WriteWrap = false, bool ReadWrap = WriteWrap>
class fifo : protected std::array<T, N>
{
public:
	fifo()
		: std::array<T, N>()
		, m_head(this->begin())
		, m_tail(this->begin())
		, m_empty(true)
	{
		static_assert(0U < N, "FIFO must have at least one element");
	}
	fifo(fifo<T, N, WriteWrap, ReadWrap> const &) = delete;
	fifo(fifo<T, N, WriteWrap, ReadWrap> &&) = delete;
	fifo<T, N, WriteWrap, ReadWrap> &operator=(fifo<T, N, WriteWrap, ReadWrap> const &) = delete;
	fifo<T, N, WriteWrap, ReadWrap> &operator=(fifo<T, N, WriteWrap, ReadWrap> &&) = delete;

	template <bool W, bool R>
	fifo(fifo<T, N, W, R> const &that)
		: std::array<T, N>(that)
		, m_head(std::advance(this->begin(), std::distance(that.begin(), that.m_head)))
		, m_tail(std::advance(this->begin(), std::distance(that.begin(), that.m_tail)))
		, m_empty(that.m_empty)
	{
	}

	template <bool W, bool R>
	fifo(fifo<T, N, W, R> &&that)
		: std::array<T, N>(std::move(that))
		, m_head(std::advance(this->begin(), std::distance(that.begin(), that.m_head)))
		, m_tail(std::advance(this->begin(), std::distance(that.begin(), that.m_tail)))
		, m_empty(that.m_empty)
	{
	}

	template <bool W, bool R>
	fifo<T, N, WriteWrap, ReadWrap> &operator=(fifo<T, N, W, R> const &that)
	{
		std::array<T, N>::operator=(that);
		m_head = std::advance(this->begin(), std::distance(that.begin(), that.m_head));
		m_tail = std::advance(this->begin(), std::distance(that.begin(), that.m_tail));
		m_empty = that.m_empty;
		return *this;
	}

	template <bool W, bool R>
	fifo<T, N, WriteWrap, ReadWrap> &operator=(fifo<T, N, WriteWrap, ReadWrap> &&that)
	{
		std::array<T, N>::operator=(std::move(that));
		m_head = std::advance(this->begin(), std::distance(that.begin(), that.m_head));
		m_tail = std::advance(this->begin(), std::distance(that.begin(), that.m_tail));
		m_empty = that.m_empty;
		return *this;
	}

	bool full() const { return !m_empty && (m_head == m_tail); }
	bool empty() const { return m_empty; }

	void enqueue(T const &v)
	{
		if (WriteWrap || m_empty || (m_head != m_tail))
		{
			*m_tail = v;
			if (this->end() == ++m_tail)
				m_tail = this->begin();
			m_empty = false;
		}
	}

	void enqueue(T &&v)
	{
		if (WriteWrap || m_empty || (m_head != m_tail))
		{
			*m_tail = std::move(v);
			if (this->end() == ++m_tail)
				m_tail = this->begin();
			m_empty = false;
		}
	}

	T const &dequeue()
	{
		T const &result(*m_head);
		if (ReadWrap || !m_empty)
		{
			if (this->end() == ++m_head)
				m_head = this->begin();
			m_empty = (m_head == m_tail);
		}
		return result;
	}

	void poke(T &v)
	{
		*m_tail = v;
	}

	void poke(T &&v)
	{
		*m_tail = std::move(v);
	}

	T const &peek() const
	{
		return *m_head;
	}

	void clear()
	{
		m_head = m_tail = this->begin();
		m_empty = true;
	}

private:
	typename fifo::iterator m_head, m_tail;
	bool                    m_empty;
};

}; // namespace util

#endif // MAME_UTIL_CORETMPL_H
