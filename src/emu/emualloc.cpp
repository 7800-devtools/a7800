// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    emualloc.c

    Memory allocation helpers for the core emulator.

***************************************************************************/

#include "emucore.h"


//**************************************************************************
//  DEBUGGING
//**************************************************************************

#define LOG_ALLOCS      (0)



//**************************************************************************
//  GLOBALS
//**************************************************************************

osd::u64 resource_pool::s_id = 0;



//**************************************************************************
//  RESOURCE POOL
//**************************************************************************

//-------------------------------------------------
//  resource_pool - constructor for a new resource
//  pool
//-------------------------------------------------

resource_pool::resource_pool(int hash_size)
	: m_hash_size(hash_size),
		m_hash(hash_size),
		m_ordered_head(nullptr),
		m_ordered_tail(nullptr)
{
	memset(&m_hash[0], 0, hash_size*sizeof(m_hash[0]));
}


//-------------------------------------------------
//  ~resource_pool - destructor for a resource
//  pool; make sure all tracked objects are
//  deleted
//-------------------------------------------------

resource_pool::~resource_pool()
{
	clear();
}


//-------------------------------------------------
//  add - add a new item to the resource pool
//-------------------------------------------------

void resource_pool::add(resource_pool_item &item, size_t size, const char *type)
{
	std::lock_guard<std::mutex> lock(m_listlock);

	// insert into hash table
	int hashval = reinterpret_cast<uintptr_t>(item.m_ptr) % m_hash_size;
	item.m_next = m_hash[hashval];
	m_hash[hashval] = &item;

	// fetch the ID of this item's pointer; some implementations put hidden data
	// before, so if we don't find it, check 4 bytes ahead
	item.m_id = ++s_id;
	if (LOG_ALLOCS)
		fprintf(stderr, "#%06d, add %s, %d bytes\n", u32(item.m_id), type, u32(size));

	// find the entry to insert after
	resource_pool_item *insert_after;
	for (insert_after = m_ordered_tail; insert_after != nullptr; insert_after = insert_after->m_ordered_prev)
		if (insert_after->m_id < item.m_id)
			break;

	// insert into the appropriate spot
	if (insert_after != nullptr)
	{
		item.m_ordered_next = insert_after->m_ordered_next;
		if (item.m_ordered_next != nullptr)
			item.m_ordered_next->m_ordered_prev = &item;
		else
			m_ordered_tail = &item;
		item.m_ordered_prev = insert_after;
		insert_after->m_ordered_next = &item;
	}
	else
	{
		item.m_ordered_next = m_ordered_head;
		if (item.m_ordered_next != nullptr)
			item.m_ordered_next->m_ordered_prev = &item;
		else
			m_ordered_tail = &item;
		item.m_ordered_prev = nullptr;
		m_ordered_head = &item;
	}
}


//-------------------------------------------------
//  remove - remove a specific item from the
//  resource pool
//-------------------------------------------------

void resource_pool::remove(void *ptr)
{
	// ignore NULLs
	if (ptr == nullptr)
		return;

	// search for the item
	std::lock_guard<std::mutex> lock(m_listlock);

	int hashval = reinterpret_cast<uintptr_t>(ptr) % m_hash_size;
	for (resource_pool_item **scanptr = &m_hash[hashval]; *scanptr != nullptr; scanptr = &(*scanptr)->m_next)

		// must match the pointer
		if ((*scanptr)->m_ptr == ptr)
		{
			// remove from hash table
			resource_pool_item *deleteme = *scanptr;
			*scanptr = deleteme->m_next;

			// remove from ordered list
			if (deleteme->m_ordered_prev != nullptr)
				deleteme->m_ordered_prev->m_ordered_next = deleteme->m_ordered_next;
			else
				m_ordered_head = deleteme->m_ordered_next;
			if (deleteme->m_ordered_next != nullptr)
				deleteme->m_ordered_next->m_ordered_prev = deleteme->m_ordered_prev;
			else
				m_ordered_tail = deleteme->m_ordered_prev;

			// delete the object and break
			if (LOG_ALLOCS)
				fprintf(stderr, "#%06d, delete %d bytes\n", u32(deleteme->m_id), u32(deleteme->m_size));
			global_free(deleteme);
			break;
		}
}


//-------------------------------------------------
//  find - find a specific item in the resource
//  pool
//-------------------------------------------------

resource_pool_item *resource_pool::find(void *ptr)
{
	// search for the item
	std::lock_guard<std::mutex> lock(m_listlock);

	int hashval = reinterpret_cast<uintptr_t>(ptr) % m_hash_size;
	resource_pool_item *item;
	for (item = m_hash[hashval]; item != nullptr; item = item->m_next)
		if (item->m_ptr == ptr)
			break;

	return item;
}


//-------------------------------------------------
//  contains - return true if given ptr is
//  contained by one of the objects in the pool
//-------------------------------------------------

bool resource_pool::contains(void *_ptrstart, void *_ptrend)
{
	u8 *ptrstart = reinterpret_cast<u8 *>(_ptrstart);
	u8 *ptrend = reinterpret_cast<u8 *>(_ptrend);

	// search for the item
	std::lock_guard<std::mutex> lock(m_listlock);

	resource_pool_item *item = nullptr;
	for (item = m_ordered_head; item != nullptr; item = item->m_ordered_next)
	{
		u8 *objstart = reinterpret_cast<u8 *>(item->m_ptr);
		u8 *objend = objstart + item->m_size;
		if (ptrstart >= objstart && ptrend <= objend)
			return true;
	}
	return false;
}


//-------------------------------------------------
//  clear - remove all items from a resource pool
//-------------------------------------------------

void resource_pool::clear()
{
	// important: delete from earliest to latest; this allows objects to clean up after
	// themselves if they wish
	while (m_ordered_head != nullptr)
		remove(m_ordered_head->m_ptr);
}
