// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * nllists.h
 *
 */

#pragma once

#ifndef NLLISTS_H_
#define NLLISTS_H_

#include "nl_config.h"
#include "netlist_types.h"
#include "plib/plists.h"
#include "plib/pchrono.h"
#include "plib/ptypes.h"

#include <atomic>
#include <thread>
#include <mutex>
#include <algorithm>
#include <utility>

// ----------------------------------------------------------------------------------------
// timed queue
// ----------------------------------------------------------------------------------------

/*
 * Use -DUSE_HEAP=1 to use stdc++ heap functions instead of linear processing.
 *
 * This slows down processing by about 25% on a Kaby Lake.
 */

#ifndef USE_HEAP
#define USE_HEAP    (0)
#endif

namespace netlist
{
	//FIXME: move to an appropriate place
	template<bool enabled_ = true>
	class pspin_mutex
	{
	public:
		pspin_mutex() noexcept { }
		void lock() noexcept{ while (m_lock.test_and_set(std::memory_order_acquire)) { } }
		void unlock() noexcept { m_lock.clear(std::memory_order_release); }
	private:
		std::atomic_flag m_lock = ATOMIC_FLAG_INIT;
	};

	template<>
	class pspin_mutex<false>
	{
	public:
		void lock() const noexcept { }
		void unlock() const noexcept { }
	};

	template <class Element, class Time>
	struct pqentry_t final
	{
		constexpr pqentry_t() noexcept : m_exec_time(), m_object(nullptr) { }
		constexpr pqentry_t(const Time &t, const Element &o) noexcept : m_exec_time(t), m_object(o) { }
		~pqentry_t() = default;
		constexpr pqentry_t(const pqentry_t &e) noexcept = default;
		constexpr pqentry_t(pqentry_t &&e) = default;
		pqentry_t& operator=(pqentry_t && other) noexcept = default;
		pqentry_t& operator=(const pqentry_t &other) noexcept = default;

		void swap(pqentry_t &other) noexcept
		{
			std::swap(m_exec_time, other.m_exec_time);
			std::swap(m_object, other.m_object);
		}

		struct QueueOp
		{
			static constexpr bool less(const pqentry_t &lhs, const pqentry_t &rhs) noexcept
			{
				return (lhs.m_exec_time < rhs.m_exec_time);
			}

			static constexpr bool equal(const pqentry_t &lhs, const pqentry_t &rhs) noexcept
			{
				return lhs.m_object == rhs.m_object;
			}

			static constexpr bool equal(const pqentry_t &lhs, const Element &rhs) noexcept
			{
				return lhs.m_object == rhs;
			}

			static constexpr pqentry_t never() noexcept { return pqentry_t(Time::never(), nullptr); }
		};

		Time m_exec_time;
		Element m_object;
	};

#if !USE_HEAP
	/* Use TS = true for a threadsafe queue */
	template <class T, bool TS, class QueueOp = typename T::QueueOp>
	class timed_queue : plib::nocopyassignmove
	{
	public:

		explicit timed_queue(const std::size_t list_size)
		: m_list(list_size)
		{
			clear();
		}

		constexpr std::size_t capacity() const noexcept { return m_list.capacity() - 1; }
		constexpr bool empty() const noexcept { return (m_end == &m_list[1]); }

		void push(T &&e) noexcept
		{
			/* Lock */
			tqlock lck(m_lock);
			T * i(m_end);
			for (; QueueOp::less(*(i - 1), e); --i)
			{
				*(i) = std::move(*(i-1));
				m_prof_sortmove.inc();
			}
			*i = std::move(e);
			++m_end;
			m_prof_call.inc();
		}

		void pop() noexcept              { --m_end; }
		const T &top() const noexcept { return *(m_end-1); }

		template <class R>
		void remove(const R &elem) noexcept
		{
			/* Lock */
			tqlock lck(m_lock);
			for (T * i = m_end - 1; i > &m_list[0]; --i)
			{
				if (QueueOp::equal(*i, elem))
				{
					--m_end;
					for (;i < m_end; ++i)
						*i = std::move(*(i+1));
					return;
				}
			}
		}

		void retime(const T &elem) noexcept
		{
			/* Lock */
			tqlock lck(m_lock);
			for (T * i = m_end - 1; i > &m_list[0]; --i)
			{
				if (QueueOp::equal(*i, elem)) // partial equal!
				{
					*i = elem;
					while (QueueOp::less(*(i-1), *i))
					{
						std::swap(*(i-1), *i);
						--i;
					}
					while (i < m_end && QueueOp::less(*i, *(i+1)))
					{
						std::swap(*(i+1), *i);
						++i;
					}
					return;
				}
			}
		}

		void clear()
		{
			tqlock lck(m_lock);
			m_end = &m_list[0];
			/* put an empty element with maximum time into the queue.
			 * the insert algo above will run into this element and doesn't
			 * need a comparison with queue start.
			 */
			m_list[0] = QueueOp::never();
			m_end++;
		}

		// save state support & mame disasm

		constexpr const T *listptr() const noexcept { return &m_list[1]; }
		constexpr std::size_t size() const noexcept { return static_cast<std::size_t>(m_end - &m_list[1]); }
		constexpr const T & operator[](const std::size_t index) const noexcept { return m_list[ 1 + index]; }
	private:
		using tqmutex = pspin_mutex<TS>;
		using tqlock = std::lock_guard<tqmutex>;

		tqmutex m_lock;
		T * m_end;
		std::vector<T> m_list;

	public:
		// profiling
		nperfcount_t m_prof_sortmove;
		nperfcount_t m_prof_call;
	};
#else
	template <class T, bool TS, class QueueOp = typename T::QueueOp>
	class timed_queue : plib::nocopyassignmove
	{
	public:

		struct compare
		{
			constexpr bool operator()(const T &a, const T &b) const { return QueueOp::less(b,a); }
		};

		explicit timed_queue(const std::size_t list_size)
		: m_list(list_size)
		{
			clear();
		}

		constexpr std::size_t capacity() const noexcept { return m_list.capacity(); }
		constexpr bool empty() const noexcept { return &m_list[0] == m_end; }

		void push(T &&e) noexcept
		{
			/* Lock */
			tqlock lck(m_lock);
			*m_end++ = e;
			std::push_heap(&m_list[0], m_end, compare());
			m_prof_call.inc();
		}

		void pop() noexcept
		{
			std::pop_heap(&m_list[0], m_end, compare());
			m_end--;
		}

		const T &top() const noexcept { return m_list[0]; }

		template <class R>
		void remove(const R &elem) noexcept
		{
			/* Lock */
			tqlock lck(m_lock);
			for (T * i = m_end - 1; i >= &m_list[0]; i--)
			{
				if (QueueOp::equal(*i, elem))
				{
					m_end--;
					for (;i < m_end; i++)
						*i = std::move(*(i+1));
					std::make_heap(&m_list[0], m_end, compare());
					return;
				}
			}
		}

		void retime(const T &elem) noexcept
		{
			/* Lock */
			tqlock lck(m_lock);
			for (T * i = m_end - 1; i >= &m_list[0]; i--)
			{
				if (QueueOp::equal(*i, elem)) // partial equal!
				{
					*i = elem;
					std::make_heap(&m_list[0], m_end, compare());
					return;
				}
			}
		}

		void clear()
		{
			tqlock lck(m_lock);
			m_list.clear();
			m_end = &m_list[0];
		}

		// save state support & mame disasm

		constexpr const T *listptr() const { return &m_list[0]; }
		constexpr std::size_t size() const noexcept { return m_list.size(); }
		constexpr const T & operator[](const std::size_t index) const { return m_list[ 0 + index]; }
	private:
		using tqmutex = pspin_mutex<TS>;
		using tqlock = std::lock_guard<tqmutex>;

		tqmutex m_lock;
		std::vector<T> m_list;
		T *m_end;

	public:
		// profiling
		nperfcount_t m_prof_sortmove;
		nperfcount_t m_prof_call;
	};
#endif
}

#endif /* NLLISTS_H_ */
