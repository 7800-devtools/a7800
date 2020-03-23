// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/*********************************************************************

    debugcpu.cpp

    Debugger CPU/memory interface engine.

***************************************************************************/

#include "emu.h"
#include "debugcpu.h"

#include "express.h"
#include "debugcon.h"
#include "debugvw.h"

#include "debugger.h"
#include "emuopts.h"
#include "screen.h"
#include "uiinput.h"

#include "coreutil.h"
#include "osdepend.h"
#include "xmlfile.h"

#include <ctype.h>
#include <fstream>


enum
{
	EXECUTION_STATE_STOPPED,
	EXECUTION_STATE_RUNNING
};

const size_t debugger_cpu::NUM_TEMP_VARIABLES = 10;

/*-------------------------------------------------
    constructor - initialize the CPU
    information for debugging
-------------------------------------------------*/

debugger_cpu::debugger_cpu(running_machine &machine)
	: m_machine(machine)
	, m_livecpu(nullptr)
	, m_visiblecpu(nullptr)
	, m_breakcpu(nullptr)
	, m_symtable(nullptr)
	, m_execution_state(EXECUTION_STATE_STOPPED)
	, m_bpindex(1)
	, m_wpindex(1)
	, m_rpindex(1)
	, m_wpdata(0)
	, m_wpaddr(0)
	, m_last_periodic_update_time(0)
	, m_comments_loaded(false)
{
	screen_device *first_screen = m_machine.first_screen();

	m_tempvar = make_unique_clear<u64[]>(NUM_TEMP_VARIABLES);

	/* create a global symbol table */
	m_symtable = std::make_unique<symbol_table>(&m_machine);

	// configure our base memory accessors
	configure_memory(*m_symtable);

	/* add "wpaddr", "wpdata", "cycles", "cpunum", "logunmap" to the global symbol table */
	m_symtable->add("wpaddr", symbol_table::READ_ONLY, &m_wpaddr);
	m_symtable->add("wpdata", symbol_table::READ_ONLY, &m_wpdata);

	using namespace std::placeholders;
	m_symtable->add("cpunum", nullptr, std::bind(&debugger_cpu::get_cpunum, this, _1, _2));
	m_symtable->add("beamx", (void *)first_screen, std::bind(&debugger_cpu::get_beamx, this, _1, _2));
	m_symtable->add("beamy", (void *)first_screen, std::bind(&debugger_cpu::get_beamy, this, _1, _2));
	m_symtable->add("frame", (void *)first_screen, std::bind(&debugger_cpu::get_frame, this, _1, _2));

	/* add the temporary variables to the global symbol table */
	for (int regnum = 0; regnum < NUM_TEMP_VARIABLES; regnum++)
	{
		char symname[10];
		sprintf(symname, "temp%d", regnum);
		m_symtable->add(symname, symbol_table::READ_WRITE, &m_tempvar[regnum]);
	}

	/* first CPU is visible by default */
	m_visiblecpu = m_machine.firstcpu;

	/* add callback for breaking on VBLANK */
	if (m_machine.first_screen() != nullptr)
		m_machine.first_screen()->register_vblank_callback(vblank_state_delegate(&debugger_cpu::on_vblank, this));
}

void debugger_cpu::configure_memory(symbol_table &table)
{
	using namespace std::placeholders;
	table.configure_memory(
		&m_machine,
		std::bind(&debugger_cpu::expression_validate, this, _1, _2, _3),
		std::bind(&debugger_cpu::expression_read_memory, this, _1, _2, _3, _4, _5, _6),
		std::bind(&debugger_cpu::expression_write_memory, this, _1, _2, _3, _4, _5, _6, _7));
}

/*-------------------------------------------------
    flush_traces - flushes all traces; this is
    useful if a trace is going on when we
    fatalerror
-------------------------------------------------*/

void debugger_cpu::flush_traces()
{
	/* this can be called on exit even when no debugging is enabled, so
	 make sure the devdebug is valid before proceeding */
	for (device_t &device : device_iterator(m_machine.root_device()))
		if (device.debug() != nullptr)
			device.debug()->trace_flush();
}



/***************************************************************************
    DEBUGGING STATUS AND INFORMATION
***************************************************************************/

/*-------------------------------------------------
    get_visible_cpu - return the visible CPU
    device (the one that commands should apply to)
-------------------------------------------------*/

device_t* debugger_cpu::get_visible_cpu()
{
	return m_visiblecpu;
}


/*-------------------------------------------------
    within_instruction_hook - true if the debugger
    is currently live
-------------------------------------------------*/

bool debugger_cpu::within_instruction_hook()
{
	return m_within_instruction_hook;
}


/*-------------------------------------------------
    is_stopped - return true if the
    current execution state is stopped
-------------------------------------------------*/

bool debugger_cpu::is_stopped()
{
	return m_execution_state == EXECUTION_STATE_STOPPED;
}


/***************************************************************************
    SYMBOL TABLE INTERFACES
***************************************************************************/

/*-------------------------------------------------
    get_global_symtable - return the global
    symbol table
-------------------------------------------------*/

symbol_table* debugger_cpu::get_global_symtable()
{
	return m_symtable.get();
}


/*-------------------------------------------------
    get_visible_symtable - return the
    locally-visible symbol table
-------------------------------------------------*/

symbol_table* debugger_cpu::get_visible_symtable()
{
	return &m_visiblecpu->debug()->symtable();
}


/*-------------------------------------------------
    source_script - specifies a debug command
    script to execute
-------------------------------------------------*/

void debugger_cpu::source_script(const char *file)
{
	// close any existing source file
	m_source_file.reset();

	// open a new one if requested
	if (file != nullptr)
	{
		auto source_file = std::make_unique<std::ifstream>(file, std::ifstream::in);
		if (source_file->fail())
		{
			if (m_machine.phase() == machine_phase::RUNNING)
				m_machine.debugger().console().printf("Cannot open command file '%s'\n", file);
			else
				fatalerror("Cannot open command file '%s'\n", file);
		}
		else
		{
			m_source_file = std::move(source_file);
		}
	}
}



//**************************************************************************
//  MEMORY AND DISASSEMBLY HELPERS
//**************************************************************************

//-------------------------------------------------
//  omment_save - save all comments for the given
//  machine
//-------------------------------------------------

bool debugger_cpu::comment_save()
{
	bool comments_saved = false;

	// if we don't have a root, bail
	util::xml::file::ptr const root = util::xml::file::create();
	if (!root)
		return false;

	// wrap in a try/catch to handle errors
	try
	{
		// create a comment node
		util::xml::data_node *const commentnode = root->add_child("mamecommentfile", nullptr);
		if (commentnode == nullptr)
			throw emu_exception();
		commentnode->set_attribute_int("version", COMMENT_VERSION);

		// create a system node
		util::xml::data_node *const systemnode = commentnode->add_child("system", nullptr);
		if (systemnode == nullptr)
			throw emu_exception();
		systemnode->set_attribute("name", m_machine.system().name);

		// for each device
		bool found_comments = false;
		for (device_t &device : device_iterator(m_machine.root_device()))
			if (device.debug() && device.debug()->comment_count() > 0)
			{
				// create a node for this device
				util::xml::data_node *const curnode = systemnode->add_child("cpu", nullptr);
				if (curnode == nullptr)
					throw emu_exception();
				curnode->set_attribute("tag", device.tag());

				// export the comments
				if (!device.debug()->comment_export(*curnode))
					throw emu_exception();
				found_comments = true;
			}

		// flush the file
		if (found_comments)
		{
			emu_file file(m_machine.options().comment_directory(), OPEN_FLAG_WRITE | OPEN_FLAG_CREATE | OPEN_FLAG_CREATE_PATHS);
			osd_file::error filerr = file.open(m_machine.basename(), ".cmt");
			if (filerr == osd_file::error::NONE)
			{
				root->write(file);
				comments_saved = true;
			}
		}
	}
	catch (emu_exception &)
	{
		return false;
	}

	// free and get out of here
	return comments_saved;
}

//-------------------------------------------------
//  comment_load - load all comments for the given
//  machine
//-------------------------------------------------

bool debugger_cpu::comment_load(bool is_inline)
{
	// open the file
	emu_file file(m_machine.options().comment_directory(), OPEN_FLAG_READ);
	osd_file::error filerr = file.open(m_machine.basename(), ".cmt");

	// if an error, just return false
	if (filerr != osd_file::error::NONE)
		return false;

	// wrap in a try/catch to handle errors
	util::xml::file::ptr const root = util::xml::file::read(file, nullptr);
	try
	{
		// read the file
		if (!root)
			throw emu_exception();

		// find the config node
		util::xml::data_node const *const commentnode = root->get_child("mamecommentfile");
		if (commentnode == nullptr)
			throw emu_exception();

		// validate the config data version
		int version = commentnode->get_attribute_int("version", 0);
		if (version != COMMENT_VERSION)
			throw emu_exception();

		// check to make sure the file is applicable
		util::xml::data_node const *const systemnode = commentnode->get_child("system");
		const char *const name = systemnode->get_attribute_string("name", "");
		if (strcmp(name, m_machine.system().name) != 0)
			throw emu_exception();

		// iterate over devices
		for (util::xml::data_node const *cpunode = systemnode->get_child("cpu"); cpunode; cpunode = cpunode->get_next_sibling("cpu"))
		{
			const char *cputag_name = cpunode->get_attribute_string("tag", "");
			device_t *device = m_machine.device(cputag_name);
			if (device != nullptr)
			{
				if(is_inline == false)
					m_machine.debugger().console().printf("@%s\n", cputag_name);

				if (!device->debug()->comment_import(*cpunode,is_inline))
					throw emu_exception();
			}
		}
	}
	catch (emu_exception &)
	{
		// clean up in case of error
		return false;
	}

	// success!
	return true;
}



/***************************************************************************
    DEBUGGER MEMORY ACCESSORS
***************************************************************************/

/*-------------------------------------------------
    read_byte - return a byte from the specified
    memory space
-------------------------------------------------*/

u8 debugger_cpu::read_byte(address_space &space, offs_t address, bool apply_translation)
{
	device_memory_interface &memory = space.device().memory();

	/* mask against the logical byte mask */
	address &= space.logbytemask();

	/* translate if necessary; if not mapped, return 0xff */
	u8 result;
	if (apply_translation && !memory.translate(space.spacenum(), TRANSLATE_READ_DEBUG, address))
	{
		result = 0xff;
	}
	else
	{   /* otherwise, call the byte reading function for the translated address */
		result = space.read_byte(address);
	}

	return result;
}


/*-------------------------------------------------
    read_word - return a word from the specified
    memory space
-------------------------------------------------*/

u16 debugger_cpu::read_word(address_space &space, offs_t address, bool apply_translation)
{
	/* mask against the logical byte mask */
	address &= space.logbytemask();

	u16 result;
	if (!WORD_ALIGNED(address))
	{   /* if this is misaligned read, or if there are no word readers, just read two bytes */
		u8 byte0 = read_byte(space, address + 0, apply_translation);
		u8 byte1 = read_byte(space, address + 1, apply_translation);

		/* based on the endianness, the result is assembled differently */
		if (space.endianness() == ENDIANNESS_LITTLE)
			result = byte0 | (byte1 << 8);
		else
			result = byte1 | (byte0 << 8);
	}
	else
	{   /* otherwise, this proceeds like the byte case */
		device_memory_interface &memory = space.device().memory();

		/* translate if necessary; if not mapped, return 0xffff */
		if (apply_translation && !memory.translate(space.spacenum(), TRANSLATE_READ_DEBUG, address))
		{
			result = 0xffff;
		}
		else
		{   /* otherwise, call the byte reading function for the translated address */
			result = space.read_word(address);
		}
	}

	return result;
}


/*-------------------------------------------------
    read_dword - return a dword from the specified
    memory space
-------------------------------------------------*/

u32 debugger_cpu::read_dword(address_space &space, offs_t address, bool apply_translation)
{
	/* mask against the logical byte mask */
	address &= space.logbytemask();

	u32 result;
	if (!DWORD_ALIGNED(address))
	{   /* if this is a misaligned read, or if there are no dword readers, just read two words */
		u16 word0 = read_word(space, address + 0, apply_translation);
		u16 word1 = read_word(space, address + 2, apply_translation);

		/* based on the endianness, the result is assembled differently */
		if (space.endianness() == ENDIANNESS_LITTLE)
			result = word0 | (word1 << 16);
		else
			result = word1 | (word0 << 16);
	}
	else
	{   /* otherwise, this proceeds like the byte case */
		device_memory_interface &memory = space.device().memory();

		if (apply_translation && !memory.translate(space.spacenum(), TRANSLATE_READ_DEBUG, address))
		{   /* translate if necessary; if not mapped, return 0xffffffff */
			result = 0xffffffff;
		}
		else
		{   /* otherwise, call the byte reading function for the translated address */
			result = space.read_dword(address);
		}
	}

	return result;
}


/*-------------------------------------------------
    read_qword - return a qword from the specified
    memory space
-------------------------------------------------*/

u64 debugger_cpu::read_qword(address_space &space, offs_t address, bool apply_translation)
{
	/* mask against the logical byte mask */
	address &= space.logbytemask();

	u64 result;
	if (!QWORD_ALIGNED(address))
	{   /* if this is a misaligned read, or if there are no qword readers, just read two dwords */
		u32 dword0 = read_dword(space, address + 0, apply_translation);
		u32 dword1 = read_dword(space, address + 4, apply_translation);

		/* based on the endianness, the result is assembled differently */
		if (space.endianness() == ENDIANNESS_LITTLE)
			result = dword0 | (u64(dword1) << 32);
		else
			result = dword1 | (u64(dword0) << 32);
	}
	else
	{   /* otherwise, this proceeds like the byte case */
		device_memory_interface &memory = space.device().memory();

		/* translate if necessary; if not mapped, return 0xffffffffffffffff */
		if (apply_translation && !memory.translate(space.spacenum(), TRANSLATE_READ_DEBUG, address))
		{
			result = ~u64(0);
		}
		else
		{   /* otherwise, call the byte reading function for the translated address */
			result = space.read_qword(address);
		}
	}

	return result;
}


/*-------------------------------------------------
    read_memory - return 1,2,4 or 8 bytes
    from the specified memory space
-------------------------------------------------*/

u64 debugger_cpu::read_memory(address_space &space, offs_t address, int size, bool apply_translation)
{
	u64 result = ~u64(0) >> (64 - 8*size);
	switch (size)
	{
		case 1:     result = read_byte(space, address, apply_translation);    break;
		case 2:     result = read_word(space, address, apply_translation);    break;
		case 4:     result = read_dword(space, address, apply_translation);   break;
		case 8:     result = read_qword(space, address, apply_translation);   break;
	}
	return result;
}


/*-------------------------------------------------
    write_byte - write a byte to the specified
    memory space
-------------------------------------------------*/

void debugger_cpu::write_byte(address_space &space, offs_t address, u8 data, bool apply_translation)
{
	device_memory_interface &memory = space.device().memory();

	/* mask against the logical byte mask */
	address &= space.logbytemask();

	/* translate if necessary; if not mapped, we're done */
	if (apply_translation && !memory.translate(space.spacenum(), TRANSLATE_WRITE_DEBUG, address))
		;

	/* otherwise, call the byte reading function for the translated address */
	else
		space.write_byte(address, data);

	m_memory_modified = true;
}


/*-------------------------------------------------
    write_word - write a word to the specified
    memory space
-------------------------------------------------*/

void debugger_cpu::write_word(address_space &space, offs_t address, u16 data, bool apply_translation)
{
	/* mask against the logical byte mask */
	address &= space.logbytemask();

	/* if this is a misaligned write, or if there are no word writers, just read two bytes */
	if (!WORD_ALIGNED(address))
	{
		if (space.endianness() == ENDIANNESS_LITTLE)
		{
			write_byte(space, address + 0, data >> 0, apply_translation);
			write_byte(space, address + 1, data >> 8, apply_translation);
		}
		else
		{
			write_byte(space, address + 0, data >> 8, apply_translation);
			write_byte(space, address + 1, data >> 0, apply_translation);
		}
	}

	/* otherwise, this proceeds like the byte case */
	else
	{
		device_memory_interface &memory = space.device().memory();

		/* translate if necessary; if not mapped, we're done */
		if (apply_translation && !memory.translate(space.spacenum(), TRANSLATE_WRITE_DEBUG, address))
			;

		/* otherwise, call the byte reading function for the translated address */
		else
			space.write_word(address, data);

		m_memory_modified = true;
	}
}


/*-------------------------------------------------
    write_dword - write a dword to the specified
    memory space
-------------------------------------------------*/

void debugger_cpu::write_dword(address_space &space, offs_t address, u32 data, bool apply_translation)
{
	/* mask against the logical byte mask */
	address &= space.logbytemask();

	/* if this is a misaligned write, or if there are no dword writers, just read two words */
	if (!DWORD_ALIGNED(address))
	{
		if (space.endianness() == ENDIANNESS_LITTLE)
		{
			write_word(space, address + 0, data >> 0, apply_translation);
			write_word(space, address + 2, data >> 16, apply_translation);
		}
		else
		{
			write_word(space, address + 0, data >> 16, apply_translation);
			write_word(space, address + 2, data >> 0, apply_translation);
		}
	}

	/* otherwise, this proceeds like the byte case */
	else
	{
		device_memory_interface &memory = space.device().memory();

		/* translate if necessary; if not mapped, we're done */
		if (apply_translation && !memory.translate(space.spacenum(), TRANSLATE_WRITE_DEBUG, address))
			;

		/* otherwise, call the byte reading function for the translated address */
		else
			space.write_dword(address, data);

		m_memory_modified = true;
	}
}


/*-------------------------------------------------
    write_qword - write a qword to the specified
    memory space
-------------------------------------------------*/

void debugger_cpu::write_qword(address_space &space, offs_t address, u64 data, bool apply_translation)
{
	/* mask against the logical byte mask */
	address &= space.logbytemask();

	/* if this is a misaligned write, or if there are no qword writers, just read two dwords */
	if (!QWORD_ALIGNED(address))
	{
		if (space.endianness() == ENDIANNESS_LITTLE)
		{
			write_dword(space, address + 0, data >> 0, apply_translation);
			write_dword(space, address + 4, data >> 32, apply_translation);
		}
		else
		{
			write_dword(space, address + 0, data >> 32, apply_translation);
			write_dword(space, address + 4, data >> 0, apply_translation);
		}
	}

	/* otherwise, this proceeds like the byte case */
	else
	{
		device_memory_interface &memory = space.device().memory();

		/* translate if necessary; if not mapped, we're done */
		if (apply_translation && !memory.translate(space.spacenum(), TRANSLATE_WRITE_DEBUG, address))
			;

		/* otherwise, call the byte reading function for the translated address */
		else
			space.write_qword(address, data);

		m_memory_modified = true;
	}
}


/*-------------------------------------------------
    write_memory - write 1,2,4 or 8 bytes to the
    specified memory space
-------------------------------------------------*/

void debugger_cpu::write_memory(address_space &space, offs_t address, u64 data, int size, bool apply_translation)
{
	switch (size)
	{
		case 1:     write_byte(space, address, data, apply_translation);  break;
		case 2:     write_word(space, address, data, apply_translation);  break;
		case 4:     write_dword(space, address, data, apply_translation); break;
		case 8:     write_qword(space, address, data, apply_translation); break;
	}
}


/*-------------------------------------------------
    read_opcode - read 1,2,4 or 8 bytes at the
    given offset from opcode space
-------------------------------------------------*/

u64 debugger_cpu::read_opcode(address_space &space, offs_t address, int size)
{
	device_memory_interface &memory = space.device().memory();

	u64 result = ~u64(0) & (~u64(0) >> (64 - 8*size));

	/* keep in logical range */
	address &= space.logbytemask();

	/* if we're bigger than the address bus, break into smaller pieces */
	if (size > space.data_width() / 8)
	{
		int halfsize = size / 2;
		u64 r0 = read_opcode(space, address + 0, halfsize);
		u64 r1 = read_opcode(space, address + halfsize, halfsize);

		if (space.endianness() == ENDIANNESS_LITTLE)
			return r0 | (r1 << (8 * halfsize));
		else
			return r1 | (r0 << (8 * halfsize));
	}

	/* translate to physical first */
	if (!memory.translate(space.spacenum(), TRANSLATE_FETCH_DEBUG, address))
		return result;

	/* keep in physical range */
	address &= space.bytemask();

	/* switch off the size and handle unaligned accesses */
	switch (size)
	{
		case 1:
			result = space.read_byte(address);
			break;

		case 2:
			result = space.read_word_unaligned(address);
			break;

		case 4:
			result = space.read_dword_unaligned(address);
			break;

		case 6:
		case 8:
			result = space.read_qword_unaligned(address);
			break;
	}

	return result;
}



/***************************************************************************
    INTERNAL HELPERS
***************************************************************************/

/*-------------------------------------------------
    on_vblank - called when a VBLANK hits
-------------------------------------------------*/

void debugger_cpu::on_vblank(screen_device &device, bool vblank_state)
{
	/* just set a global flag to be consumed later */
	if (vblank_state)
		m_vblank_occurred = true;
}


/*-------------------------------------------------
    reset_transient_flags - reset the transient
    flags on all CPUs
-------------------------------------------------*/

void debugger_cpu::reset_transient_flags()
{
	/* loop over CPUs and reset the transient flags */
	for (device_t &device : device_iterator(m_machine.root_device()))
		device.debug()->reset_transient_flag();
	m_stop_when_not_device = nullptr;
}


/*-------------------------------------------------
    process_source_file - executes commands from
    a source file
-------------------------------------------------*/

void debugger_cpu::process_source_file()
{
	std::string buf;

	// loop until the file is exhausted or until we are executing again
	while (m_execution_state == EXECUTION_STATE_STOPPED
			&& m_source_file
			&& std::getline(*m_source_file, buf))
	{
		// strip out comments (text after '//')
		size_t pos = buf.find("//");
		if (pos != std::string::npos)
			buf.resize(pos);

		// strip whitespace
		strtrimrightspace(buf);

		// execute the command
		if (!buf.empty())
			m_machine.debugger().console().execute_command(buf, true);
	}

	if (m_source_file && !m_source_file->good())
	{
		if (!m_source_file->eof())
			m_machine.debugger().console().printf("I/O error, script processing terminated\n");
		m_source_file.reset();
	}
}



/***************************************************************************
    EXPRESSION HANDLERS
***************************************************************************/

/*-------------------------------------------------
    expression_get_device - return a device
    based on a case insensitive tag search
-------------------------------------------------*/

device_t* debugger_cpu::expression_get_device(const char *tag)
{
	// convert to lowercase then lookup the name (tags are enforced to be all lower case)
	std::string fullname(tag);
	strmakelower(fullname);
	return m_machine.device(fullname.c_str());
}


/*-------------------------------------------------
    expression_read_memory - read 1,2,4 or 8 bytes
    at the given offset in the given address
    space
-------------------------------------------------*/

u64 debugger_cpu::expression_read_memory(void *param, const char *name, expression_space spacenum, u32 address, int size, bool disable_se)
{
	switch (spacenum)
	{
		case EXPSPACE_PROGRAM_LOGICAL:
		case EXPSPACE_DATA_LOGICAL:
		case EXPSPACE_IO_LOGICAL:
		case EXPSPACE_SPACE3_LOGICAL:
		{
			device_t *device = nullptr;
			device_memory_interface *memory;

			if (name != nullptr)
				device = expression_get_device(name);
			if (device == nullptr || !device->interface(memory))
			{
				device = get_visible_cpu();
				memory = &device->memory();
			}
			if (memory->has_space(AS_PROGRAM + (spacenum - EXPSPACE_PROGRAM_LOGICAL)))
			{
				address_space &space = memory->space(AS_PROGRAM + (spacenum - EXPSPACE_PROGRAM_LOGICAL));
				auto dis = m_machine.disable_side_effect(disable_se);
				return read_memory(space, space.address_to_byte(address), size, true);
			}
			break;
		}

		case EXPSPACE_PROGRAM_PHYSICAL:
		case EXPSPACE_DATA_PHYSICAL:
		case EXPSPACE_IO_PHYSICAL:
		case EXPSPACE_SPACE3_PHYSICAL:
		{
			device_t *device = nullptr;
			device_memory_interface *memory;

			if (name != nullptr)
				device = expression_get_device(name);
			if (device == nullptr || !device->interface(memory))
			{
				device = get_visible_cpu();
				memory = &device->memory();
			}
			if (memory->has_space(AS_PROGRAM + (spacenum - EXPSPACE_PROGRAM_PHYSICAL)))
			{
				address_space &space = memory->space(AS_PROGRAM + (spacenum - EXPSPACE_PROGRAM_PHYSICAL));
				auto dis = m_machine.disable_side_effect(disable_se);
				return read_memory(space, space.address_to_byte(address), size, false);
			}
			break;
		}

		case EXPSPACE_RAMWRITE:
		{
			device_t *device = nullptr;
			device_memory_interface *memory;

			if (name != nullptr)
				device = expression_get_device(name);
			if (device == nullptr || !device->interface(memory))
			{
				device = get_visible_cpu();
				memory = &device->memory();
			}
			auto dis = m_machine.disable_side_effect(disable_se);
			return expression_read_program_direct(memory->space(AS_PROGRAM), (spacenum == EXPSPACE_OPCODE), address, size);
			break;
		}

		case EXPSPACE_OPCODE:
		{
			device_t *device = nullptr;
			device_memory_interface *memory;

			if (name != nullptr)
				device = expression_get_device(name);
			if (device == nullptr || !device->interface(memory))
			{
				device = get_visible_cpu();
				memory = &device->memory();
			}
			auto dis = m_machine.disable_side_effect(disable_se);
			return expression_read_program_direct(memory->space(AS_OPCODES), (spacenum == EXPSPACE_OPCODE), address, size);
			break;
		}

		case EXPSPACE_REGION:
			if (name == nullptr)
				break;
			return expression_read_memory_region(name, address, size);
			break;

		default:
			break;
	}

	return 0;
}


/*-------------------------------------------------
    expression_read_program_direct - read memory
    directly from an opcode or RAM pointer
-------------------------------------------------*/

u64 debugger_cpu::expression_read_program_direct(address_space &space, int opcode, offs_t address, int size)
{
	u8 *base;

	/* adjust the address into a byte address, but not if being called recursively */
	if ((opcode & 2) == 0)
		address = space.address_to_byte(address);

	/* call ourself recursively until we are byte-sized */
	if (size > 1)
	{
		int halfsize = size / 2;

		/* read each half, from lower address to upper address */
		u64 r0 = expression_read_program_direct(space, opcode | 2, address + 0, halfsize);
		u64 r1 = expression_read_program_direct(space, opcode | 2, address + halfsize, halfsize);

		/* assemble based on the target endianness */
		if (space.endianness() == ENDIANNESS_LITTLE)
			return r0 | (r1 << (8 * halfsize));
		else
			return r1 | (r0 << (8 * halfsize));
	}

	/* handle the byte-sized final requests */
	else
	{
		/* lowmask specified which address bits are within the databus width */
		offs_t lowmask = space.data_width() / 8 - 1;

		/* get the base of memory, aligned to the address minus the lowbits */
		base = (u8 *)space.get_read_ptr(address & ~lowmask);

		/* if we have a valid base, return the appropriate byte */
		if (base != nullptr)
		{
			if (space.endianness() == ENDIANNESS_LITTLE)
				return base[BYTE8_XOR_LE(address) & lowmask];
			else
				return base[BYTE8_XOR_BE(address) & lowmask];
		}
	}

	return 0;
}


/*-------------------------------------------------
    expression_read_memory_region - read memory
    from a memory region
-------------------------------------------------*/

u64 debugger_cpu::expression_read_memory_region(const char *rgntag, offs_t address, int size)
{
	memory_region *region = m_machine.root_device().memregion(rgntag);
	u64 result = ~u64(0) >> (64 - 8*size);

	/* make sure we get a valid base before proceeding */
	if (region != nullptr)
	{
		/* call ourself recursively until we are byte-sized */
		if (size > 1)
		{
			int halfsize = size / 2;
			u64 r0, r1;

			/* read each half, from lower address to upper address */
			r0 = expression_read_memory_region(rgntag, address + 0, halfsize);
			r1 = expression_read_memory_region(rgntag, address + halfsize, halfsize);

			/* assemble based on the target endianness */
			if (region->endianness() == ENDIANNESS_LITTLE)
				result = r0 | (r1 << (8 * halfsize));
			else
				result = r1 | (r0 << (8 * halfsize));
		}

		/* only process if we're within range */
		else if (address < region->bytes())
		{
			/* lowmask specified which address bits are within the databus width */
			u32 lowmask = region->bytewidth() - 1;
			u8 *base = region->base() + (address & ~lowmask);

			/* if we have a valid base, return the appropriate byte */
			if (region->endianness() == ENDIANNESS_LITTLE)
				result = base[BYTE8_XOR_LE(address) & lowmask];
			else
				result = base[BYTE8_XOR_BE(address) & lowmask];
		}
	}
	return result;
}


/*-------------------------------------------------
    expression_write_memory - write 1,2,4 or 8
    bytes at the given offset in the given address
    space
-------------------------------------------------*/

void debugger_cpu::expression_write_memory(void *param, const char *name, expression_space spacenum, u32 address, int size, u64 data, bool disable_se)
{
	device_t *device = nullptr;
	device_memory_interface *memory;

	switch (spacenum)
	{
		case EXPSPACE_PROGRAM_LOGICAL:
		case EXPSPACE_DATA_LOGICAL:
		case EXPSPACE_IO_LOGICAL:
		case EXPSPACE_SPACE3_LOGICAL:
			if (name != nullptr)
				device = expression_get_device(name);
			if (device == nullptr || !device->interface(memory))
			{
				device = get_visible_cpu();
				memory = &device->memory();
			}
			if (memory->has_space(AS_PROGRAM + (spacenum - EXPSPACE_PROGRAM_LOGICAL)))
			{
				address_space &space = memory->space(AS_PROGRAM + (spacenum - EXPSPACE_PROGRAM_LOGICAL));
				auto dis = m_machine.disable_side_effect(disable_se);
				write_memory(space, space.address_to_byte(address), data, size, true);
			}
			break;

		case EXPSPACE_PROGRAM_PHYSICAL:
		case EXPSPACE_DATA_PHYSICAL:
		case EXPSPACE_IO_PHYSICAL:
		case EXPSPACE_SPACE3_PHYSICAL:
			if (name != nullptr)
				device = expression_get_device(name);
			if (device == nullptr || !device->interface(memory))
			{
				device = get_visible_cpu();
				memory = &device->memory();
			}
			if (memory->has_space(AS_PROGRAM + (spacenum - EXPSPACE_PROGRAM_PHYSICAL)))
			{
				address_space &space = memory->space(AS_PROGRAM + (spacenum - EXPSPACE_PROGRAM_PHYSICAL));
				auto dis = m_machine.disable_side_effect(disable_se);
				write_memory(space, space.address_to_byte(address), data, size, false);
			}
			break;

		case EXPSPACE_RAMWRITE: {
			if (name != nullptr)
				device = expression_get_device(name);
			if (device == nullptr || !device->interface(memory))
			{
				device = get_visible_cpu();
				memory = &device->memory();
			}
			auto dis = m_machine.disable_side_effect(disable_se);
			expression_write_program_direct(memory->space(AS_PROGRAM), (spacenum == EXPSPACE_OPCODE), address, size, data);
			break;
		}

		case EXPSPACE_OPCODE: {
			if (name != nullptr)
				device = expression_get_device(name);
			if (device == nullptr || !device->interface(memory))
			{
				device = get_visible_cpu();
				memory = &device->memory();
			}
			auto dis = m_machine.disable_side_effect(disable_se);
			expression_write_program_direct(memory->space(AS_OPCODES), (spacenum == EXPSPACE_OPCODE), address, size, data);
			break;
		}

		case EXPSPACE_REGION:
			if (name == nullptr)
				break;
			expression_write_memory_region(name, address, size, data);
			break;

		default:
			break;
	}
}


/*-------------------------------------------------
    expression_write_program_direct - write memory
    directly to an opcode or RAM pointer
-------------------------------------------------*/

void debugger_cpu::expression_write_program_direct(address_space &space, int opcode, offs_t address, int size, u64 data)
{
	/* adjust the address into a byte address, but not if being called recursively */
	if ((opcode & 2) == 0)
		address = space.address_to_byte(address);

	/* call ourself recursively until we are byte-sized */
	if (size > 1)
	{
		int halfsize = size / 2;

		/* break apart based on the target endianness */
		u64 halfmask = ~u64(0) >> (64 - 8 * halfsize);
		u64 r0, r1;
		if (space.endianness() == ENDIANNESS_LITTLE)
		{
			r0 = data & halfmask;
			r1 = (data >> (8 * halfsize)) & halfmask;
		}
		else
		{
			r0 = (data >> (8 * halfsize)) & halfmask;
			r1 = data & halfmask;
		}

		/* write each half, from lower address to upper address */
		expression_write_program_direct(space, opcode | 2, address + 0, halfsize, r0);
		expression_write_program_direct(space, opcode | 2, address + halfsize, halfsize, r1);
	}

	/* handle the byte-sized final case */
	else
	{
		/* lowmask specified which address bits are within the databus width */
		offs_t lowmask = space.data_width() / 8 - 1;

		/* get the base of memory, aligned to the address minus the lowbits */
		u8 *base = (u8 *)space.get_read_ptr(address & ~lowmask);

		/* if we have a valid base, write the appropriate byte */
		if (base != nullptr)
		{
			if (space.endianness() == ENDIANNESS_LITTLE)
				base[BYTE8_XOR_LE(address) & lowmask] = data;
			else
				base[BYTE8_XOR_BE(address) & lowmask] = data;
			m_memory_modified = true;
		}
	}
}


/*-------------------------------------------------
    expression_write_memory_region - write memory
    from a memory region
-------------------------------------------------*/

void debugger_cpu::expression_write_memory_region(const char *rgntag, offs_t address, int size, u64 data)
{
	memory_region *region = m_machine.root_device().memregion(rgntag);

	/* make sure we get a valid base before proceeding */
	if (region != nullptr)
	{
		/* call ourself recursively until we are byte-sized */
		if (size > 1)
		{
			int halfsize = size / 2;

			/* break apart based on the target endianness */
			u64 halfmask = ~u64(0) >> (64 - 8 * halfsize);
			u64 r0, r1;
			if (region->endianness() == ENDIANNESS_LITTLE)
			{
				r0 = data & halfmask;
				r1 = (data >> (8 * halfsize)) & halfmask;
			}
			else
			{
				r0 = (data >> (8 * halfsize)) & halfmask;
				r1 = data & halfmask;
			}

			/* write each half, from lower address to upper address */
			expression_write_memory_region(rgntag, address + 0, halfsize, r0);
			expression_write_memory_region(rgntag, address + halfsize, halfsize, r1);
		}

		/* only process if we're within range */
		else if (address < region->bytes())
		{
			/* lowmask specified which address bits are within the databus width */
			u32 lowmask = region->bytewidth() - 1;
			u8 *base = region->base() + (address & ~lowmask);

			/* if we have a valid base, set the appropriate byte */
			if (region->endianness() == ENDIANNESS_LITTLE)
			{
				base[BYTE8_XOR_LE(address) & lowmask] = data;
			}
			else
			{
				base[BYTE8_XOR_BE(address) & lowmask] = data;
			}
			m_memory_modified = true;
		}
	}
}


/*-------------------------------------------------
    expression_validate - validate that the
    provided expression references an
    appropriate name
-------------------------------------------------*/

expression_error::error_code debugger_cpu::expression_validate(void *param, const char *name, expression_space space)
{
	device_t *device = nullptr;
	device_memory_interface *memory;

	switch (space)
	{
	case EXPSPACE_PROGRAM_LOGICAL:
	case EXPSPACE_DATA_LOGICAL:
	case EXPSPACE_IO_LOGICAL:
	case EXPSPACE_SPACE3_LOGICAL:
		if (name)
		{
			device = expression_get_device(name);
			if (device == nullptr)
				return expression_error::INVALID_MEMORY_NAME;
		}
		if (!device)
			device = get_visible_cpu();
		if (!device->interface(memory) || !memory->has_space(AS_PROGRAM + (space - EXPSPACE_PROGRAM_LOGICAL)))
			return expression_error::NO_SUCH_MEMORY_SPACE;
		break;

	case EXPSPACE_PROGRAM_PHYSICAL:
	case EXPSPACE_DATA_PHYSICAL:
	case EXPSPACE_IO_PHYSICAL:
	case EXPSPACE_SPACE3_PHYSICAL:
		if (name)
		{
			device = expression_get_device(name);
			if (device == nullptr)
				return expression_error::INVALID_MEMORY_NAME;
		}
		if (!device)
			device = get_visible_cpu();
		if (!device->interface(memory) || !memory->has_space(AS_PROGRAM + (space - EXPSPACE_PROGRAM_PHYSICAL)))
			return expression_error::NO_SUCH_MEMORY_SPACE;
		break;

	case EXPSPACE_RAMWRITE:
		if (name)
		{
			device = expression_get_device(name);
			if (device == nullptr || !device->interface(memory))
				return expression_error::INVALID_MEMORY_NAME;
		}
		if (!device)
			device = get_visible_cpu();
		if (!device->interface(memory) || !memory->has_space(AS_PROGRAM))
			return expression_error::NO_SUCH_MEMORY_SPACE;
		break;

	case EXPSPACE_OPCODE:
		if (name)
		{
			device = expression_get_device(name);
			if (device == nullptr || !device->interface(memory))
				return expression_error::INVALID_MEMORY_NAME;
		}
		if (!device)
			device = get_visible_cpu();
		if (!device->interface(memory) || !memory->has_space(AS_OPCODES))
			return expression_error::NO_SUCH_MEMORY_SPACE;
		break;

	case EXPSPACE_REGION:
		if (!name)
			return expression_error::MISSING_MEMORY_NAME;
		if (!m_machine.root_device().memregion(name) || !m_machine.root_device().memregion(name)->base())
			return expression_error::INVALID_MEMORY_NAME;
		break;

	default:
		return expression_error::NO_SUCH_MEMORY_SPACE;
	}
	return expression_error::NONE;
}



/***************************************************************************
    VARIABLE GETTERS/SETTERS
***************************************************************************/

/*-------------------------------------------------
    get_beamx - get beam horizontal position
-------------------------------------------------*/

u64 debugger_cpu::get_beamx(symbol_table &table, void *ref)
{
	screen_device *screen = reinterpret_cast<screen_device *>(ref);
	return (screen != nullptr) ? screen->hpos() : 0;
}


/*-------------------------------------------------
    get_beamy - get beam vertical position
-------------------------------------------------*/

u64 debugger_cpu::get_beamy(symbol_table &table, void *ref)
{
	screen_device *screen = reinterpret_cast<screen_device *>(ref);
	return (screen != nullptr) ? screen->vpos() : 0;
}


/*-------------------------------------------------
    get_frame - get current frame number
-------------------------------------------------*/

u64 debugger_cpu::get_frame(symbol_table &table, void *ref)
{
	screen_device *screen = reinterpret_cast<screen_device *>(ref);
	return (screen != nullptr) ? screen->frame_number() : 0;
}


/*-------------------------------------------------
    get_cpunum - getter callback for the
    'cpunum' symbol
-------------------------------------------------*/

u64 debugger_cpu::get_cpunum(symbol_table &table, void *ref)
{
	execute_interface_iterator iter(m_machine.root_device());
	return iter.indexof(m_visiblecpu->execute());
}


void debugger_cpu::start_hook(device_t *device, bool stop_on_vblank)
{
	// stash a pointer to the current live CPU
	assert(m_livecpu == nullptr);
	m_livecpu = device;

	// if we're a new device, stop now
	if (m_stop_when_not_device != nullptr && m_stop_when_not_device != device)
	{
		m_stop_when_not_device = nullptr;
		m_execution_state = EXECUTION_STATE_STOPPED;
		reset_transient_flags();
	}

	// if we're running, do some periodic updating
	if (m_execution_state != EXECUTION_STATE_STOPPED)
	{
		if (device == m_visiblecpu && osd_ticks() > m_last_periodic_update_time + osd_ticks_per_second() / 4)
		{   // check for periodic updates
			m_machine.debug_view().update_all();
			m_machine.debug_view().flush_osd_updates();
			m_last_periodic_update_time = osd_ticks();
		}
		else if (device == m_breakcpu)
		{   // check for pending breaks
			m_execution_state = EXECUTION_STATE_STOPPED;
			m_breakcpu = nullptr;
		}

		// if a VBLANK occurred, check on things
		if (m_vblank_occurred)
		{
			m_vblank_occurred = false;

			// if we were waiting for a VBLANK, signal it now
			if (stop_on_vblank)
			{
				m_execution_state = EXECUTION_STATE_STOPPED;
				m_machine.debugger().console().printf("Stopped at VBLANK\n");
			}
		}
		// check for debug keypresses
		if (m_machine.ui_input().pressed(IPT_UI_DEBUG_BREAK))
			m_visiblecpu->debug()->halt_on_next_instruction("User-initiated break\n");
	}
}

void debugger_cpu::stop_hook(device_t *device)
{
	assert(m_livecpu == device);

	// clear the live CPU
	m_livecpu = nullptr;
}

void debugger_cpu::ensure_comments_loaded()
{
	if (!m_comments_loaded)
	{
		comment_load(true);
		m_comments_loaded = true;
	}
}


//-------------------------------------------------
//  go_next_device - execute until we hit the next
//  device
//-------------------------------------------------

void debugger_cpu::go_next_device(device_t *device)
{
	m_stop_when_not_device = device;
	m_execution_state = EXECUTION_STATE_RUNNING;
}

void debugger_cpu::go_vblank()
{
	m_vblank_occurred = false;
	m_execution_state = EXECUTION_STATE_RUNNING;
}

void debugger_cpu::halt_on_next_instruction(device_t *device, util::format_argument_pack<std::ostream> &&args)
{
	// if something is pending on this CPU already, ignore this request
	if (device == m_breakcpu)
		return;

	// output the message to the console
	m_machine.debugger().console().vprintf(std::move(args));

	// if we are live, stop now, otherwise note that we want to break there
	if (device == m_livecpu)
	{
		m_execution_state = EXECUTION_STATE_STOPPED;
		if (m_livecpu != nullptr)
			m_livecpu->debug()->compute_debug_flags();
	}
	else
	{
		m_breakcpu = device;
	}
}

//**************************************************************************
//  DEVICE DEBUG
//**************************************************************************

//-------------------------------------------------
//  device_debug - constructor
//-------------------------------------------------

device_debug::device_debug(device_t &device)
	: m_device(device)
	, m_exec(nullptr)
	, m_memory(nullptr)
	, m_state(nullptr)
	, m_disasm(nullptr)
	, m_flags(0)
	, m_symtable(&device, device.machine().debugger().cpu().get_global_symtable())
	, m_instrhook(nullptr)
	, m_stepaddr(0)
	, m_stepsleft(0)
	, m_stopaddr(0)
	, m_stoptime(attotime::zero)
	, m_stopirq(0)
	, m_stopexception(0)
	, m_endexectime(attotime::zero)
	, m_total_cycles(0)
	, m_last_total_cycles(0)
	, m_pc_history_index(0)
	, m_bplist(nullptr)
	, m_rplist(nullptr)
	, m_trace(nullptr)
	, m_hotspot_threshhold(0)
	, m_track_pc_set()
	, m_track_pc(false)
	, m_comment_set()
	, m_comment_change(0)
	, m_track_mem_set()
	, m_track_mem(false)
{
	memset(m_pc_history, 0, sizeof(m_pc_history));

	// find out which interfaces we have to work with
	device.interface(m_exec);
	device.interface(m_memory);
	device.interface(m_state);
	device.interface(m_disasm);

	// set up state-related stuff
	if (m_state != nullptr)
	{
		// add global symbol for cycles and totalcycles
		if (m_exec != nullptr)
		{
			m_symtable.add("cycles", nullptr, get_cycles);
			m_symtable.add("totalcycles", nullptr, get_totalcycles);
			m_symtable.add("lastinstructioncycles", nullptr, get_lastinstructioncycles);
		}

		// add entries to enable/disable unmap reporting for each space
		if (m_memory != nullptr)
		{
			if (m_memory->has_space(AS_PROGRAM))
				m_symtable.add("logunmap", (void *)&m_memory->space(AS_PROGRAM), get_logunmap, set_logunmap);
			if (m_memory->has_space(AS_DATA))
				m_symtable.add("logunmapd", (void *)&m_memory->space(AS_DATA), get_logunmap, set_logunmap);
			if (m_memory->has_space(AS_IO))
				m_symtable.add("logunmapi", (void *)&m_memory->space(AS_IO), get_logunmap, set_logunmap);
			if (m_memory->has_space(AS_OPCODES))
				m_symtable.add("logunmapo", (void *)&m_memory->space(AS_OPCODES), get_logunmap, set_logunmap);
		}

		// add all registers into it
		std::string tempstr;
		for (const auto &entry : m_state->state_entries())
		{
			strmakelower(tempstr.assign(entry->symbol()));
			m_symtable.add(tempstr.c_str(), (void *)(uintptr_t)entry->index(), get_state, set_state, entry->format_string());
		}
	}

	// set up execution-related stuff
	if (m_exec != nullptr)
	{
		m_flags = DEBUG_FLAG_OBSERVING | DEBUG_FLAG_HISTORY;

		// if no curpc, add one
		if (m_state != nullptr && m_symtable.find("curpc") == nullptr)
			m_symtable.add("curpc", nullptr, get_current_pc);
	}

	// set up trace
	using namespace std::placeholders;
	m_device.machine().add_logerror_callback(std::bind(&device_debug::errorlog_write_line, this, _1));
}


//-------------------------------------------------
//  ~device_debug - constructor
//-------------------------------------------------

device_debug::~device_debug()
{
	// free breakpoints and watchpoints
	breakpoint_clear_all();
	watchpoint_clear_all();
	registerpoint_clear_all();
}

//-------------------------------------------------
//  start_hook - the scheduler calls this hook
//  before beginning execution for the given device
//-------------------------------------------------

void device_debug::start_hook(const attotime &endtime)
{
	assert((m_device.machine().debug_flags & DEBUG_FLAG_ENABLED) != 0);

	m_device.machine().debugger().cpu().start_hook(&m_device, (m_flags & DEBUG_FLAG_STOP_VBLANK) != 0);

	// update the target execution end time
	m_endexectime = endtime;

	// recompute the debugging mode
	compute_debug_flags();
}


//-------------------------------------------------
//  stop_hook - the scheduler calls this hook when
//  ending execution for the given device
//-------------------------------------------------

void device_debug::stop_hook()
{
	m_device.machine().debugger().cpu().stop_hook(&m_device);
}


//-------------------------------------------------
//  interrupt_hook - called when an interrupt is
//  acknowledged
//-------------------------------------------------

void device_debug::interrupt_hook(int irqline)
{
	// see if this matches a pending interrupt request
	if ((m_flags & DEBUG_FLAG_STOP_INTERRUPT) != 0 && (m_stopirq == -1 || m_stopirq == irqline))
	{
		m_device.machine().debugger().cpu().set_execution_state(EXECUTION_STATE_STOPPED);
		m_device.machine().debugger().console().printf("Stopped on interrupt (CPU '%s', IRQ %d)\n", m_device.tag(), irqline);
		compute_debug_flags();
	}
}


//-------------------------------------------------
//  exception_hook - called when an exception is
//  generated
//-------------------------------------------------

void device_debug::exception_hook(int exception)
{
	// see if this matches a pending interrupt request
	if ((m_flags & DEBUG_FLAG_STOP_EXCEPTION) != 0 && (m_stopexception == -1 || m_stopexception == exception))
	{
		m_device.machine().debugger().cpu().set_execution_state(EXECUTION_STATE_STOPPED);
		m_device.machine().debugger().console().printf("Stopped on exception (CPU '%s', exception %d)\n", m_device.tag(), exception);
		compute_debug_flags();
	}
}


//-------------------------------------------------
//  instruction_hook - called by the CPU cores
//  before executing each instruction
//-------------------------------------------------

void device_debug::instruction_hook(offs_t curpc)
{
	running_machine &machine = m_device.machine();
	debugger_cpu& debugcpu = machine.debugger().cpu();

	// note that we are in the debugger code
	debugcpu.set_within_instruction(true);

	// update the history
	m_pc_history[m_pc_history_index++ % HISTORY_SIZE] = curpc;

	// update total cycles
	m_last_total_cycles = m_total_cycles;
	m_total_cycles = m_exec->total_cycles();

	// are we tracking our recent pc visits?
	if (m_track_pc)
	{
		const u32 crc = compute_opcode_crc32(curpc);
		m_track_pc_set.insert(dasm_pc_tag(curpc, crc));
	}

	// are we tracing?
	if (m_trace != nullptr)
		m_trace->update(curpc);

	// per-instruction hook?
	if (debugcpu.execution_state() != EXECUTION_STATE_STOPPED && (m_flags & DEBUG_FLAG_HOOKED) != 0 && (*m_instrhook)(m_device, curpc))
		debugcpu.set_execution_state(EXECUTION_STATE_STOPPED);

	// handle single stepping
	if (debugcpu.execution_state() != EXECUTION_STATE_STOPPED && (m_flags & DEBUG_FLAG_STEPPING_ANY) != 0)
	{
		// is this an actual step?
		if (m_stepaddr == ~0 || curpc == m_stepaddr)
		{
			// decrement the count and reset the breakpoint
			m_stepsleft--;
			m_stepaddr = ~0;

			// if we hit 0, stop
			if (m_stepsleft == 0)
				debugcpu.set_execution_state(EXECUTION_STATE_STOPPED);

			// update every 100 steps until we are within 200 of the end
			else if ((m_flags & DEBUG_FLAG_STEPPING_OUT) == 0 && (m_stepsleft < 200 || m_stepsleft % 100 == 0))
			{
				machine.debug_view().update_all();
				machine.debug_view().flush_osd_updates();
				machine.debugger().refresh_display();
			}
		}
	}

	// handle breakpoints
	if (debugcpu.execution_state() != EXECUTION_STATE_STOPPED && (m_flags & (DEBUG_FLAG_STOP_TIME | DEBUG_FLAG_STOP_PC | DEBUG_FLAG_LIVE_BP)) != 0)
	{
		// see if we hit a target time
		if ((m_flags & DEBUG_FLAG_STOP_TIME) != 0 && machine.time() >= m_stoptime)
		{
			machine.debugger().console().printf("Stopped at time interval %.1g\n", machine.time().as_double());
			debugcpu.set_execution_state(EXECUTION_STATE_STOPPED);
		}

		// check the temp running breakpoint and break if we hit it
		else if ((m_flags & DEBUG_FLAG_STOP_PC) != 0 && m_stopaddr == curpc)
		{
			machine.debugger().console().printf("Stopped at temporary breakpoint %X on CPU '%s'\n", m_stopaddr, m_device.tag());
			debugcpu.set_execution_state(EXECUTION_STATE_STOPPED);
		}

		// check for execution breakpoints
		else if ((m_flags & DEBUG_FLAG_LIVE_BP) != 0)
			breakpoint_check(curpc);
	}

	// if we are supposed to halt, do it now
	if (debugcpu.execution_state() == EXECUTION_STATE_STOPPED)
	{
		bool firststop = true;

		// load comments if we haven't yet
		debugcpu.ensure_comments_loaded();

		// reset any transient state
		debugcpu.reset_transient_flags();
		debugcpu.set_break_cpu(nullptr);

		// remember the last visible CPU in the debugger
		debugcpu.set_visible_cpu(&m_device);

		// update all views
		machine.debug_view().update_all();
		machine.debugger().refresh_display();

		// wait for the debugger; during this time, disable sound output
		m_device.machine().sound().debugger_mute(true);
		while (debugcpu.execution_state() == EXECUTION_STATE_STOPPED)
		{
			// flush any pending updates before waiting again
			machine.debug_view().flush_osd_updates();

			emulator_info::periodic_check();

			// clear the memory modified flag and wait
			debugcpu.set_memory_modified(false);
			if (machine.debug_flags & DEBUG_FLAG_OSD_ENABLED)
				machine.osd().wait_for_debugger(m_device, firststop);
			firststop = false;

			// if something modified memory, update the screen
			if (debugcpu.memory_modified())
			{
				machine.debug_view().update_all(DVT_DISASSEMBLY);
				machine.debugger().refresh_display();
			}

			// check for commands in the source file
			machine.debugger().cpu().process_source_file();

			// if an event got scheduled, resume
			if (machine.scheduled_event_pending())
				debugcpu.set_execution_state(EXECUTION_STATE_RUNNING);
		}
		m_device.machine().sound().debugger_mute(false);

		// remember the last visible CPU in the debugger
		debugcpu.set_visible_cpu(&m_device);
	}

	// handle step out/over on the instruction we are about to execute
	if ((m_flags & (DEBUG_FLAG_STEPPING_OVER | DEBUG_FLAG_STEPPING_OUT)) != 0 && m_stepaddr == ~0)
		prepare_for_step_overout(m_device.safe_pcbase());

	// no longer in debugger code
	debugcpu.set_within_instruction(false);
}


//-------------------------------------------------
//  memory_read_hook - the memory system calls
//  this hook when watchpoints are enabled and a
//  memory read happens
//-------------------------------------------------

void device_debug::memory_read_hook(address_space &space, offs_t address, u64 mem_mask)
{
	// check watchpoints
	watchpoint_check(space, WATCHPOINT_READ, address, 0, mem_mask);

	// check hotspots
	if (!m_hotspots.empty())
		hotspot_check(space, address);
}


//-------------------------------------------------
//  memory_write_hook - the memory system calls
//  this hook when watchpoints are enabled and a
//  memory write happens
//-------------------------------------------------

void device_debug::memory_write_hook(address_space &space, offs_t address, u64 data, u64 mem_mask)
{
	if (m_track_mem)
	{
		dasm_memory_access const newAccess(space.spacenum(), address, data, history_pc(0));
		std::pair<std::set<dasm_memory_access>::iterator, bool> trackedAccess = m_track_mem_set.insert(newAccess);
		if (!trackedAccess.second)
			trackedAccess.first->m_pc = newAccess.m_pc;
	}
	watchpoint_check(space, WATCHPOINT_WRITE, address, data, mem_mask);
}


//-------------------------------------------------
//  set_instruction_hook - set a hook to be
//  called on each instruction for a given device
//-------------------------------------------------

void device_debug::set_instruction_hook(debug_instruction_hook_func hook)
{
	// set the hook and also the CPU's flag for fast knowledge of the hook
	m_instrhook = hook;
	if (hook != nullptr)
		m_flags |= DEBUG_FLAG_HOOKED;
	else
		m_flags &= ~DEBUG_FLAG_HOOKED;
}


//-------------------------------------------------
//  ignore - ignore/observe a given device
//-------------------------------------------------

void device_debug::ignore(bool ignore)
{
	assert(m_exec != nullptr);

	if (ignore)
		m_flags &= ~DEBUG_FLAG_OBSERVING;
	else
		m_flags |= DEBUG_FLAG_OBSERVING;

	if (&m_device == m_device.machine().debugger().cpu().live_cpu() && ignore)
	{
		assert(m_exec != nullptr);
		go_next_device();
	}
}


//-------------------------------------------------
//  single_step - single step the device past the
//  requested number of instructions
//-------------------------------------------------

void device_debug::single_step(int numsteps)
{
	assert(m_exec != nullptr);

	m_stepsleft = numsteps;
	m_stepaddr = ~0;
	m_flags |= DEBUG_FLAG_STEPPING;
	m_device.machine().debugger().cpu().set_execution_state(EXECUTION_STATE_RUNNING);
}


//-------------------------------------------------
//  single_step_over - single step the device over
//  the requested number of instructions
//-------------------------------------------------

void device_debug::single_step_over(int numsteps)
{
	assert(m_exec != nullptr);

	m_stepsleft = numsteps;
	m_stepaddr = ~0;
	m_flags |= DEBUG_FLAG_STEPPING_OVER;
	m_device.machine().debugger().cpu().set_execution_state(EXECUTION_STATE_RUNNING);
}


//-------------------------------------------------
//  single_step_out - single step the device
//  out of the current function
//-------------------------------------------------

void device_debug::single_step_out()
{
	assert(m_exec != nullptr);

	m_stepsleft = 100;
	m_stepaddr = ~0;
	m_flags |= DEBUG_FLAG_STEPPING_OUT;
	m_device.machine().debugger().cpu().set_execution_state(EXECUTION_STATE_RUNNING);
}


//-------------------------------------------------
//  go - execute the device until it hits the given
//  address
//-------------------------------------------------

void device_debug::go(offs_t targetpc)
{
	assert(m_exec != nullptr);

	m_stopaddr = targetpc;
	m_flags |= DEBUG_FLAG_STOP_PC;
	m_device.machine().debugger().cpu().set_execution_state(EXECUTION_STATE_RUNNING);
}


//-------------------------------------------------
//  go_vblank - execute until the next VBLANK
//-------------------------------------------------

void device_debug::go_vblank()
{
	assert(m_exec != nullptr);

	m_flags |= DEBUG_FLAG_STOP_VBLANK;
	m_device.machine().debugger().cpu().go_vblank();
}


//-------------------------------------------------
//  go_interrupt - execute until the specified
//  interrupt fires on the device
//-------------------------------------------------

void device_debug::go_interrupt(int irqline)
{
	assert(m_exec != nullptr);

	m_stopirq = irqline;
	m_flags |= DEBUG_FLAG_STOP_INTERRUPT;
	m_device.machine().debugger().cpu().set_execution_state(EXECUTION_STATE_RUNNING);
}

void device_debug::go_next_device()
{
	m_device.machine().debugger().cpu().go_next_device(&m_device);
}

//-------------------------------------------------
//  go_exception - execute until the specified
//  exception fires on the visible CPU
//-------------------------------------------------

void device_debug::go_exception(int exception)
{
	assert(m_exec != nullptr);

	m_stopexception = exception;
	m_flags |= DEBUG_FLAG_STOP_EXCEPTION;
	m_device.machine().debugger().cpu().set_execution_state(EXECUTION_STATE_RUNNING);
}


//-------------------------------------------------
//  go_milliseconds - execute until the specified
//  delay elapses
//-------------------------------------------------

void device_debug::go_milliseconds(u64 milliseconds)
{
	assert(m_exec != nullptr);

	m_stoptime = m_device.machine().time() + attotime::from_msec(milliseconds);
	m_flags |= DEBUG_FLAG_STOP_TIME;
	m_device.machine().debugger().cpu().set_execution_state(EXECUTION_STATE_RUNNING);
}


//-------------------------------------------------
//  halt_on_next_instruction_impl - halt in the
//  debugger on the next instruction, internal
//  implementation which is necessary solely due
//  to templates in C++ being janky as all get out
//-------------------------------------------------

void device_debug::halt_on_next_instruction_impl(util::format_argument_pack<std::ostream> &&args)
{
	assert(m_exec != nullptr);
	m_device.machine().debugger().cpu().halt_on_next_instruction(&m_device, std::move(args));
}

//-------------------------------------------------
//  breakpoint_set - set a new breakpoint,
//  returning its index
//-------------------------------------------------

int device_debug::breakpoint_set(offs_t address, const char *condition, const char *action)
{
	// allocate a new one
	u32 id = m_device.machine().debugger().cpu().get_breakpoint_index();
	breakpoint *bp = auto_alloc(m_device.machine(), breakpoint(this, m_symtable, id, address, condition, action));

	// hook it into our list
	bp->m_next = m_bplist;
	m_bplist = bp;

	// update the flags and return the index
	breakpoint_update_flags();
	return bp->m_index;
}


//-------------------------------------------------
//  breakpoint_clear - clear a breakpoint by index,
//  returning true if we found it
//-------------------------------------------------

bool device_debug::breakpoint_clear(int index)
{
	// scan the list to see if we own this breakpoint
	for (breakpoint **bp = &m_bplist; *bp != nullptr; bp = &(*bp)->m_next)
		if ((*bp)->m_index == index)
		{
			breakpoint *deleteme = *bp;
			*bp = deleteme->m_next;
			auto_free(m_device.machine(), deleteme);
			breakpoint_update_flags();
			return true;
		}

	// we don't own it, return false
	return false;
}


//-------------------------------------------------
//  breakpoint_clear_all - clear all breakpoints
//-------------------------------------------------

void device_debug::breakpoint_clear_all()
{
	// clear the head until we run out
	while (m_bplist != nullptr)
		breakpoint_clear(m_bplist->index());
}


//-------------------------------------------------
//  breakpoint_enable - enable/disable a breakpoint
//  by index, returning true if we found it
//-------------------------------------------------

bool device_debug::breakpoint_enable(int index, bool enable)
{
	// scan the list to see if we own this breakpoint
	for (breakpoint *bp = m_bplist; bp != nullptr; bp = bp->next())
		if (bp->m_index == index)
		{
			bp->m_enabled = enable;
			breakpoint_update_flags();
			return true;
		}

	// we don't own it, return false
	return false;
}


//-------------------------------------------------
//  breakpoint_enable_all - enable/disable all
//  breakpoints
//-------------------------------------------------

void device_debug::breakpoint_enable_all(bool enable)
{
	// apply the enable to all breakpoints we own
	for (breakpoint *bp = m_bplist; bp != nullptr; bp = bp->next())
		breakpoint_enable(bp->index(), enable);
}


//-------------------------------------------------
//  watchpoint_set - set a new watchpoint,
//  returning its index
//-------------------------------------------------

int device_debug::watchpoint_set(address_space &space, int type, offs_t address, offs_t length, const char *condition, const char *action)
{
	if (space.spacenum() >= int(m_wplist.size()))
		m_wplist.resize(space.spacenum()+1, nullptr);

	// allocate a new one
	u32 id = m_device.machine().debugger().cpu().get_watchpoint_index();
	watchpoint *wp = auto_alloc(m_device.machine(), watchpoint(this, m_symtable, id, space, type, address, length, condition, action));

	// hook it into our list
	wp->m_next = m_wplist[space.spacenum()];
	m_wplist[space.spacenum()] = wp;

	// update the flags and return the index
	watchpoint_update_flags(wp->m_space);
	return wp->m_index;
}


//-------------------------------------------------
//  watchpoint_clear - clear a watchpoint by index,
//  returning true if we found it
//-------------------------------------------------

bool device_debug::watchpoint_clear(int index)
{
	// scan the list to see if we own this breakpoint
	for (int spacenum = 0; spacenum < int(m_wplist.size()); ++spacenum)
		for (watchpoint **wp = &m_wplist[spacenum]; *wp != nullptr; wp = &(*wp)->m_next)
			if ((*wp)->m_index == index)
			{
				watchpoint *deleteme = *wp;
				address_space &space = deleteme->m_space;
				*wp = deleteme->m_next;
				auto_free(m_device.machine(), deleteme);
				watchpoint_update_flags(space);
				return true;
			}

	// we don't own it, return false
	return false;
}


//-------------------------------------------------
//  watchpoint_clear_all - clear all watchpoints
//-------------------------------------------------

void device_debug::watchpoint_clear_all()
{
	// clear the head until we run out
	for (int spacenum = 0; spacenum < int(m_wplist.size()); ++spacenum)
		while (m_wplist[spacenum] != nullptr)
			watchpoint_clear(m_wplist[spacenum]->index());
}


//-------------------------------------------------
//  watchpoint_enable - enable/disable a watchpoint
//  by index, returning true if we found it
//-------------------------------------------------

bool device_debug::watchpoint_enable(int index, bool enable)
{
	// scan the list to see if we own this watchpoint
	for (int spacenum = 0; spacenum < int(m_wplist.size()); ++spacenum)
		for (watchpoint *wp = m_wplist[spacenum]; wp != nullptr; wp = wp->next())
			if (wp->m_index == index)
			{
				wp->m_enabled = enable;
				watchpoint_update_flags(wp->m_space);
				return true;
			}

	// we don't own it, return false
	return false;
}


//-------------------------------------------------
//  watchpoint_enable_all - enable/disable all
//  watchpoints
//-------------------------------------------------

void device_debug::watchpoint_enable_all(bool enable)
{
	// apply the enable to all watchpoints we own
	for (int spacenum = 0; spacenum < int(m_wplist.size()); ++spacenum)
		for (watchpoint *wp = m_wplist[spacenum]; wp != nullptr; wp = wp->next())
			watchpoint_enable(wp->index(), enable);
}


//-------------------------------------------------
//  registerpoint_set - set a new registerpoint,
//  returning its index
//-------------------------------------------------

int device_debug::registerpoint_set(const char *condition, const char *action)
{
	// allocate a new one
	u32 id = m_device.machine().debugger().cpu().get_registerpoint_index();
	registerpoint *rp = auto_alloc(m_device.machine(), registerpoint(m_symtable, id, condition, action));

	// hook it into our list
	rp->m_next = m_rplist;
	m_rplist = rp;

	// update the flags and return the index
	breakpoint_update_flags();
	return rp->m_index;
}


//-------------------------------------------------
//  registerpoint_clear - clear a registerpoint by index,
//  returning true if we found it
//-------------------------------------------------

bool device_debug::registerpoint_clear(int index)
{
	// scan the list to see if we own this registerpoint
	for (registerpoint **rp = &m_rplist; *rp != nullptr; rp = &(*rp)->m_next)
		if ((*rp)->m_index == index)
		{
			registerpoint *deleteme = *rp;
			*rp = deleteme->m_next;
			auto_free(m_device.machine(), deleteme);
			breakpoint_update_flags();
			return true;
		}

	// we don't own it, return false
	return false;
}


//-------------------------------------------------
//  registerpoint_clear_all - clear all registerpoints
//-------------------------------------------------

void device_debug::registerpoint_clear_all()
{
	// clear the head until we run out
	while (m_rplist != nullptr)
		registerpoint_clear(m_rplist->index());
}


//-------------------------------------------------
//  registerpoint_enable - enable/disable a registerpoint
//  by index, returning true if we found it
//-------------------------------------------------

bool device_debug::registerpoint_enable(int index, bool enable)
{
	// scan the list to see if we own this conditionpoint
	for (registerpoint *rp = m_rplist; rp != nullptr; rp = rp->next())
		if (rp->m_index == index)
		{
			rp->m_enabled = enable;
			breakpoint_update_flags();
			return true;
		}

	// we don't own it, return false
	return false;
}


//-------------------------------------------------
//  registerpoint_enable_all - enable/disable all
//  registerpoints
//-------------------------------------------------

void device_debug::registerpoint_enable_all(bool enable)
{
	// apply the enable to all registerpoints we own
	for (registerpoint *rp = m_rplist; rp != nullptr; rp = rp->next())
		registerpoint_enable(rp->index(), enable);
}


//-------------------------------------------------
//  hotspot_track - enable/disable tracking of
//  hotspots
//-------------------------------------------------

void device_debug::hotspot_track(int numspots, int threshhold)
{
	// if we already have tracking enabled, kill it
	m_hotspots.clear();

	// only start tracking if we have a non-zero count
	if (numspots > 0)
	{
		// allocate memory for hotspots
		m_hotspots.resize(numspots);
		memset(&m_hotspots[0], 0xff, numspots*sizeof(m_hotspots[0]));

		// fill in the info
		m_hotspot_threshhold = threshhold;
	}

	// update the watchpoint flags to include us
	if (m_memory != nullptr && m_memory->has_space(AS_PROGRAM))
		watchpoint_update_flags(m_memory->space(AS_PROGRAM));
}


//-------------------------------------------------
//  history_pc - return an entry from the PC
//  history
//-------------------------------------------------

offs_t device_debug::history_pc(int index) const
{
	if (index > 0)
		index = 0;
	if (index <= -HISTORY_SIZE)
		index = -HISTORY_SIZE + 1;
	return m_pc_history[(m_pc_history_index + ARRAY_LENGTH(m_pc_history) - 1 + index) % ARRAY_LENGTH(m_pc_history)];
}


//-------------------------------------------------
//  track_pc_visited - returns a boolean stating
//  if this PC has been visited or not.  CRC32 is
//  done in this function on currently active CPU.
//  TODO: Take a CPU context as input
//-------------------------------------------------

bool device_debug::track_pc_visited(const offs_t& pc) const
{
	if (m_track_pc_set.empty())
		return false;
	const u32 crc = compute_opcode_crc32(pc);
	return m_track_pc_set.find(dasm_pc_tag(pc, crc)) != m_track_pc_set.end();
}


//-------------------------------------------------
//  set_track_pc_visited - set this pc as visited.
//  TODO: Take a CPU context as input
//-------------------------------------------------

void device_debug::set_track_pc_visited(const offs_t& pc)
{
	const u32 crc = compute_opcode_crc32(pc);
	m_track_pc_set.insert(dasm_pc_tag(pc, crc));
}


//-------------------------------------------------
//  track_mem_pc_from_address_data - returns the pc that
//  wrote the data to this address or (offs_t)(-1) for
//  'not available'.
//-------------------------------------------------

offs_t device_debug::track_mem_pc_from_space_address_data(const int& space,
															const offs_t& address,
															const u64& data) const
{
	const offs_t missing = (offs_t)(-1);
	if (m_track_mem_set.empty())
		return missing;
	std::set<dasm_memory_access>::iterator const mem_access = m_track_mem_set.find(dasm_memory_access(space, address, data, 0));
	if (mem_access == m_track_mem_set.end()) return missing;
	return mem_access->m_pc;
}


//-------------------------------------------------
//  comment_add - adds a comment to the list at
//  the given address
//-------------------------------------------------

void device_debug::comment_add(offs_t addr, const char *comment, rgb_t color)
{
	// create a new item for the list
	u32 const crc = compute_opcode_crc32(addr);
	dasm_comment const newComment = dasm_comment(addr, crc, comment, color);
	std::pair<std::set<dasm_comment>::iterator, bool> const inserted = m_comment_set.insert(newComment);
	if (!inserted.second)
	{
		// Insert returns false if comment exists
		m_comment_set.erase(inserted.first);
		m_comment_set.insert(newComment);
	}

	// force an update
	m_comment_change++;
}


//-------------------------------------------------
//  comment_remove - removes a comment at the
//  given address with a matching CRC
//-------------------------------------------------

bool device_debug::comment_remove(offs_t addr)
{
	const u32 crc = compute_opcode_crc32(addr);
	size_t const removed = m_comment_set.erase(dasm_comment(addr, crc, "", 0xffffffff));
	if (removed != 0U) m_comment_change++;
	return removed != 0U;
}


//-------------------------------------------------
//  comment_text - return the text of a comment
//-------------------------------------------------

const char *device_debug::comment_text(offs_t addr) const
{
	const u32 crc = compute_opcode_crc32(addr);
	auto comment = m_comment_set.find(dasm_comment(addr, crc, "", 0));
	if (comment == m_comment_set.end()) return nullptr;
	return comment->m_text.c_str();
}


//-------------------------------------------------
//  comment_export - export the comments to the
//  given XML data node
//-------------------------------------------------

bool device_debug::comment_export(util::xml::data_node &curnode)
{
	// iterate through the comments
	for (const auto & elem : m_comment_set)
	{
		util::xml::data_node *datanode = curnode.add_child("comment", util::xml::normalize_string(elem.m_text.c_str()));
		if (datanode == nullptr)
			return false;
		datanode->set_attribute_int("address", elem.m_address);
		datanode->set_attribute_int("color", elem.m_color);
		datanode->set_attribute("crc", string_format("%08X", elem.m_crc).c_str());
	}
	return true;
}


//-------------------------------------------------
//  comment_import - import the comments from the
//  given XML data node
//-------------------------------------------------

bool device_debug::comment_import(util::xml::data_node const &cpunode, bool is_inline)
{
	// iterate through nodes
	for (util::xml::data_node const *datanode = cpunode.get_child("comment"); datanode; datanode = datanode->get_next_sibling("comment"))
	{
		// extract attributes
		offs_t address = datanode->get_attribute_int("address", 0);
		rgb_t color = datanode->get_attribute_int("color", 0);

		u32 crc;
		sscanf(datanode->get_attribute_string("crc", nullptr), "%08X", &crc);

		// add the new comment
		if(is_inline == true)
			m_comment_set.insert(dasm_comment(address, crc, datanode->get_value(), color));
		else
			m_device.machine().debugger().console().printf(" %08X - %s\n", address, datanode->get_value());
	}
	return true;
}


//-------------------------------------------------
//  compute_opcode_crc32 - determine the CRC of
//  the opcode bytes at the given address
//-------------------------------------------------

u32 device_debug::compute_opcode_crc32(offs_t pc) const
{
	// Basically the same thing as dasm_wrapped, but with some tiny savings
	assert(m_memory != nullptr);

	// determine the adjusted PC
	address_space &decrypted_space = m_memory->has_space(AS_OPCODES) ? m_memory->space(AS_OPCODES) : m_memory->space(AS_PROGRAM);
	address_space &space = m_memory->space(AS_PROGRAM);
	offs_t pcbyte = space.address_to_byte(pc) & space.bytemask();

	// fetch the bytes up to the maximum
	u8 opbuf[64], argbuf[64];
	int maxbytes = (m_disasm != nullptr) ? m_disasm->max_opcode_bytes() : 1;
	for (int numbytes = 0; numbytes < maxbytes; numbytes++)
	{
		opbuf[numbytes] = m_device.machine().debugger().cpu().read_opcode(decrypted_space, pcbyte + numbytes, 1);
		argbuf[numbytes] = m_device.machine().debugger().cpu().read_opcode(space, pcbyte + numbytes, 1);
	}

	u32 numbytes = maxbytes;
	if (m_disasm != nullptr)
	{
		// disassemble to our buffer
		std::ostringstream diasmbuf;
		numbytes = m_disasm->disassemble(diasmbuf, pc, opbuf, argbuf) & DASMFLAG_LENGTHMASK;
	}

	// return a CRC of the exact count of opcode bytes
	return core_crc32(0, opbuf, numbytes);
}


//-------------------------------------------------
//  trace - trace execution of a given device
//-------------------------------------------------

void device_debug::trace(FILE *file, bool trace_over, bool detect_loops, bool logerror, const char *action)
{
	// delete any existing tracers
	m_trace = nullptr;

	// if we have a new file, make a new tracer
	if (file != nullptr)
		m_trace = std::make_unique<tracer>(*this, *file, trace_over, detect_loops, logerror, action);
}


//-------------------------------------------------
//  trace_printf - output data into the given
//  device's tracefile, if tracing
//-------------------------------------------------

void device_debug::trace_printf(const char *fmt, ...)
{
	if (m_trace != nullptr)
	{
		va_list va;
		va_start(va, fmt);
		m_trace->vprintf(fmt, va);
		va_end(va);
	}
}


//-------------------------------------------------
//  compute_debug_flags - compute the global
//  debug flags for optimal efficiency
//-------------------------------------------------

void device_debug::compute_debug_flags()
{
	running_machine &machine = m_device.machine();
	debugger_cpu& debugcpu = machine.debugger().cpu();

	// clear out global flags by default, keep DEBUG_FLAG_OSD_ENABLED
	machine.debug_flags &= DEBUG_FLAG_OSD_ENABLED;
	machine.debug_flags |= DEBUG_FLAG_ENABLED;

	// if we are ignoring this CPU, or if events are pending, we're done
	if ((m_flags & DEBUG_FLAG_OBSERVING) == 0 || machine.scheduled_event_pending() || machine.save_or_load_pending())
		return;

	// if we're stopped, keep calling the hook
	if (debugcpu.execution_state() == EXECUTION_STATE_STOPPED)
		machine.debug_flags |= DEBUG_FLAG_CALL_HOOK;

	// if we're tracking history, or we're hooked, or stepping, or stopping at a breakpoint
	// make sure we call the hook
	if ((m_flags & (DEBUG_FLAG_HISTORY | DEBUG_FLAG_HOOKED | DEBUG_FLAG_STEPPING_ANY | DEBUG_FLAG_STOP_PC | DEBUG_FLAG_LIVE_BP)) != 0)
		machine.debug_flags |= DEBUG_FLAG_CALL_HOOK;

	// also call if we are tracing
	if (m_trace != nullptr)
		machine.debug_flags |= DEBUG_FLAG_CALL_HOOK;

	// if we are stopping at a particular time and that time is within the current timeslice, we need to be called
	if ((m_flags & DEBUG_FLAG_STOP_TIME) && m_endexectime <= m_stoptime)
		machine.debug_flags |= DEBUG_FLAG_CALL_HOOK;
}


//-------------------------------------------------
//  prepare_for_step_overout - prepare things for
//  stepping over an instruction
//-------------------------------------------------

void device_debug::prepare_for_step_overout(offs_t pc)
{
	// disassemble the current instruction and get the flags
	std::string dasmbuffer;
	offs_t dasmresult = dasm_wrapped(dasmbuffer, pc);

	// if flags are supported and it's a call-style opcode, set a temp breakpoint after that instruction
	if ((dasmresult & DASMFLAG_SUPPORTED) != 0 && (dasmresult & DASMFLAG_STEP_OVER) != 0)
	{
		int extraskip = (dasmresult & DASMFLAG_OVERINSTMASK) >> DASMFLAG_OVERINSTSHIFT;
		pc += dasmresult & DASMFLAG_LENGTHMASK;

		// if we need to skip additional instructions, advance as requested
		while (extraskip-- > 0)
			pc += dasm_wrapped(dasmbuffer, pc) & DASMFLAG_LENGTHMASK;
		m_stepaddr = pc;
	}

	// if we're stepping out and this isn't a step out instruction, reset the steps until stop to a high number
	if ((m_flags & DEBUG_FLAG_STEPPING_OUT) != 0)
	{
		if ((dasmresult & DASMFLAG_SUPPORTED) != 0 && (dasmresult & DASMFLAG_STEP_OUT) == 0)
			m_stepsleft = 100;
		else
			m_stepsleft = 1;
	}
}


//-------------------------------------------------
//  breakpoint_update_flags - update the device's
//  breakpoint flags
//-------------------------------------------------

void device_debug::breakpoint_update_flags()
{
	// see if there are any enabled breakpoints
	m_flags &= ~DEBUG_FLAG_LIVE_BP;
	for (breakpoint *bp = m_bplist; bp != nullptr; bp = bp->m_next)
		if (bp->m_enabled)
		{
			m_flags |= DEBUG_FLAG_LIVE_BP;
			break;
		}

	if ( ! ( m_flags & DEBUG_FLAG_LIVE_BP ) )
	{
		// see if there are any enabled registerpoints
		for (registerpoint *rp = m_rplist; rp != nullptr; rp = rp->m_next)
		{
			if (rp->m_enabled)
			{
				m_flags |= DEBUG_FLAG_LIVE_BP;
			}
		}
	}

	// push the flags out globally
	if (m_device.machine().debugger().cpu().live_cpu() != nullptr)
		m_device.machine().debugger().cpu().live_cpu()->debug()->compute_debug_flags();
}


//-------------------------------------------------
//  breakpoint_check - check the breakpoints for
//  a given device
//-------------------------------------------------

void device_debug::breakpoint_check(offs_t pc)
{
	debugger_cpu& debugcpu = m_device.machine().debugger().cpu();

	// see if we match
	for (breakpoint *bp = m_bplist; bp != nullptr; bp = bp->m_next)
		if (bp->hit(pc))
		{
			// halt in the debugger by default
			debugcpu.set_execution_state(EXECUTION_STATE_STOPPED);

			// if we hit, evaluate the action
			if (!bp->m_action.empty())
				m_device.machine().debugger().console().execute_command(bp->m_action, false);

			// print a notification, unless the action made us go again
			if (debugcpu.execution_state() == EXECUTION_STATE_STOPPED)
				m_device.machine().debugger().console().printf("Stopped at breakpoint %X\n", bp->m_index);
			break;
		}

	// see if we have any matching registerpoints
	for (registerpoint *rp = m_rplist; rp != nullptr; rp = rp->m_next)
	{
		if (rp->hit())
		{
			// halt in the debugger by default
			debugcpu.set_execution_state(EXECUTION_STATE_STOPPED);

			// if we hit, evaluate the action
			if (!rp->m_action.empty())
			{
				m_device.machine().debugger().console().execute_command(rp->m_action, false);
			}

			// print a notification, unless the action made us go again
			if (debugcpu.execution_state() == EXECUTION_STATE_STOPPED)
			{
				m_device.machine().debugger().console().printf("Stopped at registerpoint %X\n", rp->m_index);
			}
			break;
		}
	}
}


//-------------------------------------------------
//  watchpoint_update_flags - update the device's
//  watchpoint flags
//-------------------------------------------------

void device_debug::watchpoint_update_flags(address_space &space)
{
	// if hotspots are enabled, turn on all reads
	bool enableread = false;
	if (!m_hotspots.empty())
		enableread = true;

	// see if there are any enabled breakpoints
	bool enablewrite = false;
	if (space.spacenum() < int(m_wplist.size()))
		for (watchpoint *wp = m_wplist[space.spacenum()]; wp != nullptr; wp = wp->m_next)
			if (wp->m_enabled)
			{
				if (wp->m_type & WATCHPOINT_READ)
					enableread = true;
				if (wp->m_type & WATCHPOINT_WRITE)
					enablewrite = true;
			}

	// push the flags out globally
	space.enable_read_watchpoints(enableread);
	space.enable_write_watchpoints(enablewrite);
}


//-------------------------------------------------
//  watchpoint_check - check the watchpoints
//  for a given CPU and address space
//-------------------------------------------------

void device_debug::watchpoint_check(address_space& space, int type, offs_t address, u64 value_to_write, u64 mem_mask)
{
	space.machine().debugger().cpu().watchpoint_check(space, type, address, value_to_write, mem_mask, m_wplist);
}

void debugger_cpu::watchpoint_check(address_space& space, int type, offs_t address, u64 value_to_write, u64 mem_mask, std::vector<device_debug::watchpoint *> &wplist)
{
	// if we're within debugger code, don't stop
	if (m_within_instruction_hook || m_machine.side_effect_disabled())
		return;

	m_within_instruction_hook = true;

	// adjust address, size & value_to_write based on mem_mask.
	offs_t size = 0;
	if (mem_mask != 0)
	{
		int bus_size = space.data_width() / 8;
		int address_offset = 0;

		while (address_offset < bus_size && (mem_mask & 0xff) == 0)
		{
			address_offset++;
			value_to_write >>= 8;
			mem_mask >>= 8;
		}

		while (mem_mask != 0)
		{
			size++;
			mem_mask >>= 8;
		}

		// (1<<(size*8))-1 won't work when size is 8; let's just use a lut
		static const u64 masks[] = {
				0x0U,
				0xffU,
				0xffffU,
				0xffffffU,
				0xffffffffU,
				0xffffffffffU,
				0xffffffffffffU,
				0xffffffffffffffU,
				0xffffffffffffffffU};
		value_to_write &= masks[size];

		if (space.endianness() == ENDIANNESS_LITTLE)
			address += address_offset;
		else
			address += bus_size - size - address_offset;
	}

	// if we are a write watchpoint, stash the value that will be written
	m_wpaddr = address;
	if (type & WATCHPOINT_WRITE)
		m_wpdata = value_to_write;

	// see if we match
	if (space.spacenum() < int(wplist.size()))
		for (device_debug::watchpoint *wp = wplist[space.spacenum()]; wp != nullptr; wp = wp->next())
			if (wp->hit(type, address, size))
			{
				// halt in the debugger by default
				m_execution_state = EXECUTION_STATE_STOPPED;

				// if we hit, evaluate the action
				if (!wp->action().empty())
					m_machine.debugger().console().execute_command(wp->action(), false);

				// print a notification, unless the action made us go again
				if (m_execution_state == EXECUTION_STATE_STOPPED)
				{
					static const char *const sizes[] =
					{
						"0bytes", "byte", "word", "3bytes", "dword", "5bytes", "6bytes", "7bytes", "qword"
					};
					offs_t pc = space.device().safe_pcbase();
					std::string buffer;

					if (type & WATCHPOINT_WRITE)
					{
						buffer = string_format("Stopped at watchpoint %X writing %s to %08X (PC=%X)", wp->index(), sizes[size], space.byte_to_address(address), pc);
						if (value_to_write >> 32)
							buffer.append(string_format(" (data=%X%08X)", u32(value_to_write >> 32), u32(value_to_write)));
						else
							buffer.append(string_format(" (data=%X)", u32(value_to_write)));
					}
					else
						buffer = string_format("Stopped at watchpoint %X reading %s from %08X (PC=%X)", wp->index(), sizes[size], space.byte_to_address(address), pc);
					m_machine.debugger().console().printf("%s\n", buffer.c_str());
					space.device().debug()->compute_debug_flags();
				}
				break;
			}

	m_within_instruction_hook = false;
}


//-------------------------------------------------
//  hotspot_check - check for hotspots on a
//  memory read access
//-------------------------------------------------

void device_debug::hotspot_check(address_space &space, offs_t address)
{
	offs_t curpc = m_device.safe_pcbase();

	// see if we have a match in our list
	unsigned int hotindex;
	for (hotindex = 0; hotindex < m_hotspots.size(); hotindex++)
		if (m_hotspots[hotindex].m_access == address && m_hotspots[hotindex].m_pc == curpc && m_hotspots[hotindex].m_space == &space)
			break;

	// if we didn't find any, make a new entry
	if (hotindex == m_hotspots.size())
	{
		// if the bottom of the list is over the threshold, print it
		hotspot_entry &spot = m_hotspots[m_hotspots.size() - 1];
		if (spot.m_count > m_hotspot_threshhold)
			space.machine().debugger().console().printf("Hotspot @ %s %08X (PC=%08X) hit %d times (fell off bottom)\n", space.name(), spot.m_access, spot.m_pc, spot.m_count);

		// move everything else down and insert this one at the top
		memmove(&m_hotspots[1], &m_hotspots[0], sizeof(m_hotspots[0]) * (m_hotspots.size() - 1));
		m_hotspots[0].m_access = address;
		m_hotspots[0].m_pc = curpc;
		m_hotspots[0].m_space = &space;
		m_hotspots[0].m_count = 1;
	}

	// if we did find one, increase the count and move it to the top
	else
	{
		m_hotspots[hotindex].m_count++;
		if (hotindex != 0)
		{
			hotspot_entry temp = m_hotspots[hotindex];
			memmove(&m_hotspots[1], &m_hotspots[0], sizeof(m_hotspots[0]) * hotindex);
			m_hotspots[0] = temp;
		}
	}
}


//-------------------------------------------------
//  dasm_wrapped - wraps calls to the disassembler
//  by fetching the opcode bytes to a temporary
//  buffer and then disassembling them
//-------------------------------------------------

u32 device_debug::dasm_wrapped(std::string &buffer, offs_t pc)
{
	assert(m_memory != nullptr && m_disasm != nullptr);

	// determine the adjusted PC
	address_space &decrypted_space = m_memory->has_space(AS_OPCODES) ? m_memory->space(AS_OPCODES) : m_memory->space(AS_PROGRAM);
	address_space &space = m_memory->space(AS_PROGRAM);
	offs_t pcbyte = space.address_to_byte(pc) & space.bytemask();

	// fetch the bytes up to the maximum
	u8 opbuf[64], argbuf[64];
	int maxbytes = m_disasm->max_opcode_bytes();
	for (int numbytes = 0; numbytes < maxbytes; numbytes++)
	{
		opbuf[numbytes] = m_device.machine().debugger().cpu().read_opcode(decrypted_space, pcbyte + numbytes, 1);
		argbuf[numbytes] = m_device.machine().debugger().cpu().read_opcode(space, pcbyte + numbytes, 1);
	}

	// disassemble to our buffer
	std::ostringstream stream;
	uint32_t result = m_disasm->disassemble(stream, pc, opbuf, argbuf);
	buffer = stream.str();
	return result;
}


//-------------------------------------------------
//  get_current_pc - getter callback for a device's
//  current instruction pointer
//-------------------------------------------------

u64 device_debug::get_current_pc(symbol_table &table, void *ref)
{
	device_t *device = reinterpret_cast<device_t *>(table.globalref());
	return device->safe_pcbase();
}


//-------------------------------------------------
//  get_cycles - getter callback for the
//  'cycles' symbol
//-------------------------------------------------

u64 device_debug::get_cycles(symbol_table &table, void *ref)
{
	device_t *device = reinterpret_cast<device_t *>(table.globalref());
	return device->debug()->m_exec->cycles_remaining();
}


//-------------------------------------------------
//  get_totalcycles - getter callback for the
//  'totalcycles' symbol
//-------------------------------------------------

u64 device_debug::get_totalcycles(symbol_table &table, void *ref)
{
	device_t *device = reinterpret_cast<device_t *>(table.globalref());
	return device->debug()->m_total_cycles;
}


//-------------------------------------------------
//  get_lastinstructioncycles - getter callback for the
//  'lastinstructioncycles' symbol
//-------------------------------------------------

u64 device_debug::get_lastinstructioncycles(symbol_table &table, void *ref)
{
	device_t *device = reinterpret_cast<device_t *>(table.globalref());
	device_debug *debug = device->debug();
	return debug->m_total_cycles - debug->m_last_total_cycles;
}


//-------------------------------------------------
//  get_logunmap - getter callback for the logumap
//  symbols
//-------------------------------------------------

u64 device_debug::get_logunmap(symbol_table &table, void *ref)
{
	address_space &space = *reinterpret_cast<address_space *>(table.globalref());
	return space.log_unmap();
}


//-------------------------------------------------
//  set_logunmap - setter callback for the logumap
//  symbols
//-------------------------------------------------

void device_debug::set_logunmap(symbol_table &table, void *ref, u64 value)
{
	address_space &space = *reinterpret_cast<address_space *>(table.globalref());
	space.set_log_unmap(value ? true : false);
}


//-------------------------------------------------
//  get_state - getter callback for a device's
//  state symbols
//-------------------------------------------------

u64 device_debug::get_state(symbol_table &table, void *ref)
{
	device_t *device = reinterpret_cast<device_t *>(table.globalref());
	return device->debug()->m_state->state_int(reinterpret_cast<uintptr_t>(ref));
}


//-------------------------------------------------
//  set_state - setter callback for a device's
//  state symbols
//-------------------------------------------------

void device_debug::set_state(symbol_table &table, void *ref, u64 value)
{
	device_t *device = reinterpret_cast<device_t *>(table.globalref());
	device->debug()->m_state->set_state_int(reinterpret_cast<uintptr_t>(ref), value);
}



//**************************************************************************
//  DEBUG BREAKPOINT
//**************************************************************************

//-------------------------------------------------
//  breakpoint - constructor
//-------------------------------------------------

device_debug::breakpoint::breakpoint(device_debug* debugInterface,
										symbol_table &symbols,
										int index,
										offs_t address,
										const char *condition,
										const char *action)
	: m_debugInterface(debugInterface),
		m_next(nullptr),
		m_index(index),
		m_enabled(true),
		m_address(address),
		m_condition(&symbols, (condition != nullptr) ? condition : "1"),
		m_action((action != nullptr) ? action : "")
{
}


//-------------------------------------------------
//  hit - detect a hit
//-------------------------------------------------

bool device_debug::breakpoint::hit(offs_t pc)
{
	// don't hit if disabled
	if (!m_enabled)
		return false;

	// must match our address
	if (m_address != pc)
		return false;

	// must satisfy the condition
	if (!m_condition.is_empty())
	{
		try
		{
			return (m_condition.execute() != 0);
		}
		catch (expression_error &)
		{
			return false;
		}
	}

	return true;
}



//**************************************************************************
//  DEBUG WATCHPOINT
//**************************************************************************

//-------------------------------------------------
//  watchpoint - constructor
//-------------------------------------------------

device_debug::watchpoint::watchpoint(device_debug* debugInterface,
										symbol_table &symbols,
										int index,
										address_space &space,
										int type,
										offs_t address,
										offs_t length,
										const char *condition,
										const char *action)
	: m_debugInterface(debugInterface),
		m_next(nullptr),
		m_space(space),
		m_index(index),
		m_enabled(true),
		m_type(type),
		m_address(space.address_to_byte(address) & space.bytemask()),
		m_length(space.address_to_byte(length)),
		m_condition(&symbols, (condition != nullptr) ? condition : "1"),
		m_action((action != nullptr) ? action : "")
{
}


//-------------------------------------------------
//  hit - detect a hit
//-------------------------------------------------

bool device_debug::watchpoint::hit(int type, offs_t address, int size)
{
	// don't hit if disabled
	if (!m_enabled)
		return false;

	// must match the type
	if ((m_type & type) == 0)
		return false;

	// must match our address
	if (address + size <= m_address || address >= m_address + m_length)
		return false;

	// must satisfy the condition
	if (!m_condition.is_empty())
	{
		try
		{
			return (m_condition.execute() != 0);
		}
		catch (expression_error &)
		{
			return false;
		}
	}
	return true;
}



//**************************************************************************
//  DEBUG REGISTERPOINT
//**************************************************************************

//-------------------------------------------------
//  registerpoint - constructor
//-------------------------------------------------

device_debug::registerpoint::registerpoint(symbol_table &symbols, int index, const char *condition, const char *action)
	: m_next(nullptr),
		m_index(index),
		m_enabled(true),
		m_condition(&symbols, (condition != nullptr) ? condition : "1"),
		m_action((action != nullptr) ? action : "")
{
}


//-------------------------------------------------
//  hit - detect a hit
//-------------------------------------------------

bool device_debug::registerpoint::hit()
{
	// don't hit if disabled
	if (!m_enabled)
		return false;

	// must satisfy the condition
	if (!m_condition.is_empty())
	{
		try
		{
			return (m_condition.execute() != 0);
		}
		catch (expression_error &)
		{
			return false;
		}
	}

	return true;
}



//**************************************************************************
//  TRACER
//**************************************************************************

//-------------------------------------------------
//  tracer - constructor
//-------------------------------------------------

device_debug::tracer::tracer(device_debug &debug, FILE &file, bool trace_over, bool detect_loops, bool logerror, const char *action)
	: m_debug(debug)
	, m_file(file)
	, m_action((action != nullptr) ? action : "")
	, m_detect_loops(detect_loops)
	, m_logerror(logerror)
	, m_loops(0)
	, m_nextdex(0)
	, m_trace_over(trace_over)
	, m_trace_over_target(~0)
{
	memset(m_history, 0, sizeof(m_history));
}


//-------------------------------------------------
//  ~tracer - destructor
//-------------------------------------------------

device_debug::tracer::~tracer()
{
	// make sure we close the file if we can
	fclose(&m_file);
}


//-------------------------------------------------
//  update - log to the tracefile the data for a
//  given instruction
//-------------------------------------------------

void device_debug::tracer::update(offs_t pc)
{
	// are we in trace over mode and in a subroutine?
	if (m_trace_over && m_trace_over_target != ~0)
	{
		if (m_trace_over_target != pc)
			return;
		m_trace_over_target = ~0;
	}

	if (m_detect_loops)
	{
		// check for a loop condition
		int count = 0;
		for (auto & elem : m_history)
			if (elem == pc)
				count++;

		// if more than 1 hit, just up the loop count and get out
		if (count > 1)
		{
			m_loops++;
			return;
		}

		// if we just finished looping, indicate as much
		if (m_loops != 0)
			fprintf(&m_file, "\n   (loops for %d instructions)\n\n", m_loops);
		m_loops = 0;
	}

	// execute any trace actions first
	if (!m_action.empty())
		m_debug.m_device.machine().debugger().console().execute_command(m_action, false);

	// print the address
	std::string buffer;
	int logaddrchars = m_debug.logaddrchars();
	if (m_debug.is_octal())
	{
		buffer = string_format("%0*o: ", logaddrchars*3/2, pc);
	}
	else
	{
		buffer = string_format("%0*X: ", logaddrchars, pc);
	}

	// print the disassembly
	std::string dasm;
	offs_t dasmresult = m_debug.dasm_wrapped(dasm, pc);
	buffer.append(dasm);

	// output the result
	fprintf(&m_file, "%s\n", buffer.c_str());

	// do we need to step the trace over this instruction?
	if (m_trace_over && (dasmresult & DASMFLAG_SUPPORTED) != 0 && (dasmresult & DASMFLAG_STEP_OVER) != 0)
	{
		int extraskip = (dasmresult & DASMFLAG_OVERINSTMASK) >> DASMFLAG_OVERINSTSHIFT;
		offs_t trace_over_target = pc + (dasmresult & DASMFLAG_LENGTHMASK);

		// if we need to skip additional instructions, advance as requested
		while (extraskip-- > 0)
			trace_over_target += m_debug.dasm_wrapped(dasm, trace_over_target) & DASMFLAG_LENGTHMASK;

		m_trace_over_target = trace_over_target;
	}

	// log this PC
	m_nextdex = (m_nextdex + 1) % TRACE_LOOPS;
	m_history[m_nextdex] = pc;
	fflush(&m_file);
}


//-------------------------------------------------
//  vprintf - generic print to the trace file
//-------------------------------------------------

void device_debug::tracer::vprintf(const char *format, va_list va)
{
	// pass through to the file
	vfprintf(&m_file, format, va);
	fflush(&m_file);
}


//-------------------------------------------------
//  flush - flush any pending changes to the trace
//  file
//-------------------------------------------------

void device_debug::tracer::flush()
{
	fflush(&m_file);
}


//-------------------------------------------------
//  dasm_pc_tag - constructor
//-------------------------------------------------

device_debug::dasm_pc_tag::dasm_pc_tag(const offs_t& address, const u32& crc)
	: m_address(address),
		m_crc(crc)
{
}

//-------------------------------------------------
//  dasm_memory_access - constructor
//-------------------------------------------------

device_debug::dasm_memory_access::dasm_memory_access(const int& address_space,
														const offs_t& address,
														const u64& data,
														const offs_t& pc)
	: m_address_space(address_space),
		m_address(address),
		m_data(data),
		m_pc(pc)
{
}

//-------------------------------------------------
//  dasm_comment - constructor
//-------------------------------------------------

device_debug::dasm_comment::dasm_comment(offs_t address, u32 crc, const char *text, rgb_t color)
	: dasm_pc_tag(address, crc),
		m_text(text),
		m_color(std::move(color))
{
}


//-------------------------------------------------
//  dasm_comment - constructor
//-------------------------------------------------

void device_debug::errorlog_write_line(const char *line)
{
	if (m_trace && m_trace->logerror())
		trace_printf("%s", line);
}
