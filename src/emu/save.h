// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    save.h

    Save state management functions.

***************************************************************************/

#pragma once

#ifndef __EMU_H__
#error Dont include this file directly; include emu.h instead.
#endif

#ifndef MAME_EMU_SAVE_H
#define MAME_EMU_SAVE_H



//**************************************************************************
//  CONSTANTS
//**************************************************************************

enum save_error
{
	STATERR_NONE,
	STATERR_ILLEGAL_REGISTRATIONS,
	STATERR_INVALID_HEADER,
	STATERR_READ_ERROR,
	STATERR_WRITE_ERROR
};



//**************************************************************************
//  MACROS
//**************************************************************************

// callback delegate for presave/postload
typedef named_delegate<void ()> save_prepost_delegate;


// use this to declare a given type is a simple, non-pointer type that can be
// saved; in general, this is intended only to be used for specific enum types
// defined by your device
#define ALLOW_SAVE_TYPE(TYPE) \
	template<> struct save_manager::type_checker<TYPE> { static const bool is_atom = true; static const bool is_pointer = false; }

// use this as above, but also to declare that std::vector<TYPE> is safe as well
#define ALLOW_SAVE_TYPE_AND_ARRAY(TYPE) \
	ALLOW_SAVE_TYPE(TYPE); \
	template<> inline void save_manager::save_item(device_t *device, const char *module, const char *tag, int index, std::vector<TYPE> &value, const char *name) { save_memory(device, module, tag, index, name, &value[0], sizeof(TYPE), value.size()); }


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class state_entry
{
public:
	// construction/destruction
	state_entry(void *data, const char *name, device_t *device, const char *module, const char *tag, int index, u8 size, u32 count);

	// helpers
	void flip_data();

	// state
	void *          m_data;                 // pointer to the memory to save/restore
	std::string     m_name;                 // full name
	device_t *      m_device;               // associated device, nullptr if none
	std::string     m_module;               // module name
	std::string     m_tag;                  // tag name
	int             m_index;                // index
	u8              m_typesize;             // size of the raw data type
	u32             m_typecount;            // number of items
	u32             m_offset;               // offset within the final structure
};

class save_manager
{
	// type_checker is a set of templates to identify valid save types
	template<typename _ItemType> struct type_checker { static const bool is_atom = false; static const bool is_pointer = false; };
	template<typename _ItemType> struct type_checker<_ItemType*> { static const bool is_atom = false; static const bool is_pointer = true; };

public:
	// construction/destruction
	save_manager(running_machine &machine);

	// getters
	running_machine &machine() const { return m_machine; }
	int registration_count() const { return m_entry_list.size(); }
	bool registration_allowed() const { return m_reg_allowed; }

	// registration control
	void allow_registration(bool allowed = true);
	const char *indexed_item(int index, void *&base, u32 &valsize, u32 &valcount) const;

	// function registration
	void register_presave(save_prepost_delegate func);
	void register_postload(save_prepost_delegate func);

	// callback dispatching
	void dispatch_presave();
	void dispatch_postload();

	// generic memory registration
	void save_memory(device_t *device, const char *module, const char *tag, u32 index, const char *name, void *val, u32 valsize, u32 valcount = 1);

	// templatized wrapper for general objects
	template<typename _ItemType>
	void save_item(device_t *device, const char *module, const char *tag, int index, _ItemType &value, const char *valname)
	{
		if (type_checker<_ItemType>::is_pointer) throw emu_fatalerror("Called save_item on a pointer with no count!");
		if (!type_checker<_ItemType>::is_atom) throw emu_fatalerror("Called save_item on a non-fundamental type!");
		save_memory(device, module, tag, index, valname, &value, sizeof(value));
	}

	// templatized wrapper for 1-dimensional arrays
	template<typename _ItemType, std::size_t N>
	void save_item(device_t *device, const char *module, const char *tag, int index, _ItemType (&value)[N], const char *valname)
	{
		if (!type_checker<_ItemType>::is_atom) throw emu_fatalerror("Called save_item on a non-fundamental type!");
		save_memory(device, module, tag, index, valname, &value[0], sizeof(value[0]), N);
	}

	// templatized wrapper for 2-dimensional arrays
	template<typename _ItemType, std::size_t M, std::size_t N>
	void save_item(device_t *device, const char *module, const char *tag, int index, _ItemType (&value)[M][N], const char *valname)
	{
		if (!type_checker<_ItemType>::is_atom) throw emu_fatalerror("Called save_item on a non-fundamental type!");
		save_memory(device, module, tag, index, valname, &value[0][0], sizeof(value[0][0]), M * N);
	}

	// templatized wrapper for pointers
	template<typename _ItemType>
	void save_pointer(device_t *device, const char *module, const char *tag, int index, _ItemType *value, const char *valname, u32 count)
	{
		if (!type_checker<_ItemType>::is_atom) throw emu_fatalerror("Called save_item on a non-fundamental type!");
		save_memory(device, module, tag, index, valname, value, sizeof(*value), count);
	}

	// global memory registration
	template<typename _ItemType>
	void save_item(_ItemType &value, const char *valname, int index = 0) { save_item(nullptr, "global", nullptr, index, value, valname); }
	template<typename _ItemType>
	void save_pointer(_ItemType *value, const char *valname, u32 count, int index = 0) { save_pointer(nullptr, "global", nullptr, index, value, valname, count); }

	// file processing
	static save_error check_file(running_machine &machine, emu_file &file, const char *gamename, void (CLIB_DECL *errormsg)(const char *fmt, ...));
	save_error write_file(emu_file &file);
	save_error read_file(emu_file &file);

private:
	// internal helpers
	u32 signature() const;
	void dump_registry() const;
	static save_error validate_header(const u8 *header, const char *gamename, u32 signature, void (CLIB_DECL *errormsg)(const char *fmt, ...), const char *error_prefix);

	// state callback item
	class state_callback
	{
	public:
		// construction/destruction
		state_callback(save_prepost_delegate callback);

		save_prepost_delegate m_func;               // delegate
	};

	// internal state
	running_machine &       m_machine;              // reference to our machine
	bool                    m_reg_allowed;          // are registrations allowed?
	int                     m_illegal_regs;         // number of illegal registrations

	std::vector<std::unique_ptr<state_entry>> m_entry_list;          // list of registered entries
	std::vector<std::unique_ptr<state_callback>> m_presave_list;     // list of pre-save functions
	std::vector<std::unique_ptr<state_callback>> m_postload_list;    // list of post-load functions
};


// template specializations to enumerate the fundamental atomic types you are allowed to save
ALLOW_SAVE_TYPE_AND_ARRAY(char)
ALLOW_SAVE_TYPE          (bool); // std::vector<bool> may be packed internally
ALLOW_SAVE_TYPE_AND_ARRAY(s8)
ALLOW_SAVE_TYPE_AND_ARRAY(u8)
ALLOW_SAVE_TYPE_AND_ARRAY(s16)
ALLOW_SAVE_TYPE_AND_ARRAY(u16)
ALLOW_SAVE_TYPE_AND_ARRAY(s32)
ALLOW_SAVE_TYPE_AND_ARRAY(u32)
ALLOW_SAVE_TYPE_AND_ARRAY(s64)
ALLOW_SAVE_TYPE_AND_ARRAY(u64)
ALLOW_SAVE_TYPE_AND_ARRAY(PAIR)
ALLOW_SAVE_TYPE_AND_ARRAY(PAIR64)
ALLOW_SAVE_TYPE_AND_ARRAY(float)
ALLOW_SAVE_TYPE_AND_ARRAY(double)
ALLOW_SAVE_TYPE_AND_ARRAY(endianness_t)
ALLOW_SAVE_TYPE_AND_ARRAY(rgb_t)



//**************************************************************************
//  INLINE FUNCTIONS
//**************************************************************************

//-------------------------------------------------
//  save_item - specialized save_item for bitmaps
//-------------------------------------------------

template<>
inline void save_manager::save_item(device_t *device, const char *module, const char *tag, int index, bitmap_ind8 &value, const char *name)
{
	save_memory(device, module, tag, index, name, &value.pix(0), value.bpp() / 8, value.rowpixels() * value.height());
}

template<>
inline void save_manager::save_item(device_t *device, const char *module, const char *tag, int index, bitmap_ind16 &value, const char *name)
{
	save_memory(device, module, tag, index, name, &value.pix(0), value.bpp() / 8, value.rowpixels() * value.height());
}

template<>
inline void save_manager::save_item(device_t *device, const char *module, const char *tag, int index, bitmap_ind32 &value, const char *name)
{
	save_memory(device, module, tag, index, name, &value.pix(0), value.bpp() / 8, value.rowpixels() * value.height());
}

template<>
inline void save_manager::save_item(device_t *device, const char *module, const char *tag, int index, bitmap_rgb32 &value, const char *name)
{
	save_memory(device, module, tag, index, name, &value.pix(0), value.bpp() / 8, value.rowpixels() * value.height());
}


//-------------------------------------------------
//  save_item - specialized save_item for attotimes
//-------------------------------------------------

template<>
inline void save_manager::save_item(device_t *device, const char *module, const char *tag, int index, attotime &value, const char *name)
{
	std::string tempstr = std::string(name).append(".attoseconds");
	save_memory(device, module, tag, index, tempstr.c_str(), &value.m_attoseconds, sizeof(value.m_attoseconds));
	tempstr.assign(name).append(".seconds");
	save_memory(device, module, tag, index, tempstr.c_str(), &value.m_seconds, sizeof(value.m_seconds));
}


#endif  /* MAME_EMU_SAVE_H */
