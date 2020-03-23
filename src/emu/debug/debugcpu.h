// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/*********************************************************************

    debugcpu.h

    Debugger CPU/memory interface engine.

***************************************************************************/

#ifndef MAME_EMU_DEBUG_DEBUGCPU_H
#define MAME_EMU_DEBUG_DEBUGCPU_H

#pragma once

#include "express.h"

#include <set>


//**************************************************************************
//  CONSTANTS
//**************************************************************************

constexpr u8 WATCHPOINT_READ        = 1;
constexpr u8 WATCHPOINT_WRITE       = 2;
constexpr u8 WATCHPOINT_READWRITE   = WATCHPOINT_READ | WATCHPOINT_WRITE;

constexpr int COMMENT_VERSION       = 1;



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

typedef int (*debug_instruction_hook_func)(device_t &device, offs_t curpc);


// ======================> device_debug

// [TODO] This whole thing is terrible.

class device_debug
{
public:
	// breakpoint class
	class breakpoint
	{
		friend class device_debug;

	public:
		// construction/destruction
		breakpoint(device_debug* debugInterface,
					symbol_table &symbols,
					int index,
					offs_t address,
					const char *condition = nullptr,
					const char *action = nullptr);

		// getters
		const device_debug *debugInterface() const { return m_debugInterface; }
		breakpoint *next() const { return m_next; }
		int index() const { return m_index; }
		bool enabled() const { return m_enabled; }
		offs_t address() const { return m_address; }
		const char *condition() const { return m_condition.original_string(); }
		const char *action() const { return m_action.c_str(); }

		// setters
		void setEnabled(bool value) { m_enabled = value; }

	private:
		// internals
		bool hit(offs_t pc);

		const device_debug * m_debugInterface;           // the interface we were created from
		breakpoint *         m_next;                     // next in the list
		int                  m_index;                    // user reported index
		bool                 m_enabled;                  // enabled?
		offs_t               m_address;                  // execution address
		parsed_expression    m_condition;                // condition
		std::string          m_action;                   // action
	};

	// watchpoint class
	class watchpoint
	{
		friend class device_debug;

	public:
		// construction/destruction
		watchpoint(device_debug* debugInterface,
					symbol_table &symbols,
					int index,
					address_space &space,
					int type,
					offs_t address,
					offs_t length,
					const char *condition = nullptr,
					const char *action = nullptr);

		// getters
		const device_debug *debugInterface() const { return m_debugInterface; }
		watchpoint *next() const { return m_next; }
		address_space &space() const { return m_space; }
		int index() const { return m_index; }
		int type() const { return m_type; }
		bool enabled() const { return m_enabled; }
		offs_t address() const { return m_address; }
		offs_t length() const { return m_length; }
		const char *condition() const { return m_condition.original_string(); }
		const std::string &action() const { return m_action; }

		// setters
		void setEnabled(bool value) { m_enabled = value; }

		// internals
		bool hit(int type, offs_t address, int size);

	private:
		const device_debug * m_debugInterface;           // the interface we were created from
		watchpoint *         m_next;                     // next in the list
		address_space &      m_space;                    // address space
		int                  m_index;                    // user reported index
		bool                 m_enabled;                  // enabled?
		u8                   m_type;                     // type (read/write)
		offs_t               m_address;                  // start address
		offs_t               m_length;                   // length of watch area
		parsed_expression    m_condition;                // condition
		std::string          m_action;                   // action
	};

	// registerpoint class
	class registerpoint
	{
		friend class device_debug;

	public:
		// construction/destruction
		registerpoint(symbol_table &symbols, int index, const char *condition, const char *action = nullptr);

		// getters
		registerpoint *next() const { return m_next; }
		int index() const { return m_index; }
		bool enabled() const { return m_enabled; }
		const char *condition() const { return m_condition.original_string(); }
		const char *action() const { return m_action.c_str(); }

	private:
		// internals
		bool hit();

		registerpoint *     m_next;                     // next in the list
		int                 m_index;                    // user reported index
		bool                m_enabled;                  // enabled?
		parsed_expression   m_condition;                // condition
		std::string         m_action;                   // action
	};

public:
	// construction/destruction
	device_debug(device_t &device);
	~device_debug();

	// getters
	symbol_table &symtable() { return m_symtable; }

	// commonly-used pass-throughs
	int logaddrchars() const { return (m_memory != nullptr && m_memory->has_space(AS_PROGRAM)) ? m_memory->space(AS_PROGRAM).logaddrchars() : 8; }
	bool is_octal() const { return (m_memory != nullptr && m_memory->has_space(AS_PROGRAM)) ? m_memory->space(AS_PROGRAM).is_octal() : false; }
	device_t& device() const { return m_device; }

	// hooks used by the rest of the system
	void start_hook(const attotime &endtime);
	void stop_hook();
	void interrupt_hook(int irqline);
	void exception_hook(int exception);
	void instruction_hook(offs_t curpc);
	void memory_read_hook(address_space &space, offs_t address, u64 mem_mask);
	void memory_write_hook(address_space &space, offs_t address, u64 data, u64 mem_mask);

	// hooks into our operations
	void set_instruction_hook(debug_instruction_hook_func hook);

	// debugger focus
	void ignore(bool ignore = true);
	bool observing() const { return ((m_flags & DEBUG_FLAG_OBSERVING) != 0); }

	// single stepping
	void single_step(int numsteps = 1);
	void single_step_over(int numsteps = 1);
	void single_step_out();

	// execution
	void go(offs_t targetpc = ~0);
	void go_vblank();
	void go_interrupt(int irqline = -1);
	void go_exception(int exception);
	void go_milliseconds(u64 milliseconds);
	void go_next_device();

	template <typename Format, typename... Params>
	void halt_on_next_instruction(Format &&fmt, Params &&... args)
	{
		halt_on_next_instruction_impl(util::make_format_argument_pack(std::forward<Format>(fmt), std::forward<Params>(args)...));
	}

	// breakpoints
	breakpoint *breakpoint_first() const { return m_bplist; }
	int breakpoint_set(offs_t address, const char *condition = nullptr, const char *action = nullptr);
	bool breakpoint_clear(int index);
	void breakpoint_clear_all();
	bool breakpoint_enable(int index, bool enable = true);
	void breakpoint_enable_all(bool enable = true);

	// watchpoints
	int watchpoint_space_count() const { return m_wplist.size(); }
	watchpoint *watchpoint_first(int spacenum) const { return m_wplist[spacenum]; }
	int watchpoint_set(address_space &space, int type, offs_t address, offs_t length, const char *condition, const char *action);
	bool watchpoint_clear(int wpnum);
	void watchpoint_clear_all();
	bool watchpoint_enable(int index, bool enable = true);
	void watchpoint_enable_all(bool enable = true);

	// registerpoints
	registerpoint *registerpoint_first() const { return m_rplist; }
	int registerpoint_set(const char *condition, const char *action = nullptr);
	bool registerpoint_clear(int index);
	void registerpoint_clear_all();
	bool registerpoint_enable(int index, bool enable = true);
	void registerpoint_enable_all(bool enable = true );

	// hotspots
	bool hotspot_tracking_enabled() const { return !m_hotspots.empty(); }
	void hotspot_track(int numspots, int threshhold);

	// comments
	void comment_add(offs_t address, const char *comment, rgb_t color);
	bool comment_remove(offs_t addr);
	const char *comment_text(offs_t addr) const;
	u32 comment_count() const { return m_comment_set.size(); }
	u32 comment_change_count() const { return m_comment_change; }
	bool comment_export(util::xml::data_node &node);
	bool comment_import(util::xml::data_node const &node, bool is_inline);
	u32 compute_opcode_crc32(offs_t pc) const;

	// history
	offs_t history_pc(int index) const;

	// pc tracking
	void set_track_pc(bool value) { m_track_pc = value; }
	bool track_pc_visited(const offs_t& pc) const;
	void set_track_pc_visited(const offs_t& pc);
	void track_pc_data_clear() { m_track_pc_set.clear(); }

	// memory tracking
	void set_track_mem(bool value) { m_track_mem = value; }
	offs_t track_mem_pc_from_space_address_data(const int& space,
												const offs_t& address,
												const u64& data) const;
	void track_mem_data_clear() { m_track_mem_set.clear(); }

	// tracing
	void trace(FILE *file, bool trace_over, bool detect_loops, bool logerror, const char *action);
	void trace_printf(const char *fmt, ...) ATTR_PRINTF(2,3);
	void trace_flush() { if (m_trace != nullptr) m_trace->flush(); }

	void reset_transient_flag() { m_flags &= ~DEBUG_FLAG_TRANSIENT; }

	static const int HISTORY_SIZE = 256;

	// debugger_cpu helpers
	void compute_debug_flags();

private:
	void halt_on_next_instruction_impl(util::format_argument_pack<std::ostream> &&args);

	// internal helpers
	void prepare_for_step_overout(offs_t pc);
	u32 dasm_wrapped(std::string &buffer, offs_t pc);
	void errorlog_write_line(const char *line);

	// breakpoint and watchpoint helpers
	void breakpoint_update_flags();
	void breakpoint_check(offs_t pc);
	void watchpoint_update_flags(address_space &space);
	void watchpoint_check(address_space &space, int type, offs_t address, u64 value_to_write, u64 mem_mask);
	void hotspot_check(address_space &space, offs_t address);

	// symbol get/set callbacks
	static u64 get_current_pc(symbol_table &table, void *ref);
	static u64 get_cycles(symbol_table &table, void *ref);
	static u64 get_totalcycles(symbol_table &table, void *ref);
	static u64 get_lastinstructioncycles(symbol_table &table, void *ref);
	static u64 get_logunmap(symbol_table &table, void *ref);
	static void set_logunmap(symbol_table &table, void *ref, u64 value);
	static u64 get_state(symbol_table &table, void *ref);
	static void set_state(symbol_table &table, void *ref, u64 value);

	// basic device information
	device_t &                 m_device;                // device we are attached to
	device_execute_interface * m_exec;                  // execute interface, if present
	device_memory_interface *  m_memory;                // memory interface, if present
	device_state_interface *   m_state;                 // state interface, if present
	device_disasm_interface *  m_disasm;                // disasm interface, if present

	// global state
	u32                         m_flags;                // debugging flags for this CPU
	symbol_table                m_symtable;             // symbol table for expression evaluation
	debug_instruction_hook_func m_instrhook;            // per-instruction callback hook

	// stepping information
	offs_t                  m_stepaddr;                 // step target address for DEBUG_FLAG_STEPPING_OVER
	int                     m_stepsleft;                // number of steps left until done

	// execution information
	offs_t                  m_stopaddr;                 // stop address for DEBUG_FLAG_STOP_PC
	attotime                m_stoptime;                 // stop time for DEBUG_FLAG_STOP_TIME
	int                     m_stopirq;                  // stop IRQ number for DEBUG_FLAG_STOP_INTERRUPT
	int                     m_stopexception;            // stop exception number for DEBUG_FLAG_STOP_EXCEPTION
	attotime                m_endexectime;              // ending time of the current execution
	u64                     m_total_cycles;             // current total cycles
	u64                     m_last_total_cycles;        // last total cycles

	// history
	offs_t                  m_pc_history[HISTORY_SIZE]; // history of recent PCs
	u32                     m_pc_history_index;         // current history index

	// breakpoints and watchpoints
	breakpoint *            m_bplist;                   // list of breakpoints
	std::vector<watchpoint *> m_wplist;                 // watchpoint lists for each address space
	registerpoint *         m_rplist;                   // list of registerpoints

	// tracing
	class tracer
	{
	public:
		tracer(device_debug &debug, FILE &file, bool trace_over, bool detect_loops, bool logerror, const char *action);
		~tracer();

		void update(offs_t pc);
		void vprintf(const char *format, va_list va);
		void flush();
		bool logerror() const { return m_logerror; }

	private:
		static const int TRACE_LOOPS = 64;

		device_debug &      m_debug;                    // reference to our owner
		FILE &              m_file;                     // tracing file for this CPU
		std::string         m_action;                   // action to perform during a trace
		offs_t              m_history[TRACE_LOOPS];     // history of recent PCs
		bool                m_detect_loops;             // whether or not we should detect loops
		bool                m_logerror;                 // whether or not we should collect logerror output
		int                 m_loops;                    // number of instructions in a loop
		int                 m_nextdex;                  // next index
		bool                m_trace_over;               // true if we're tracing over
		offs_t              m_trace_over_target;        // target for tracing over
														//    (0 = not tracing over,
														//    ~0 = not currently tracing over)
	};
	std::unique_ptr<tracer>                m_trace;                    // tracer state

	// hotspots
	struct hotspot_entry
	{
		offs_t              m_access;                   // access address
		offs_t              m_pc;                       // PC of the access
		address_space *     m_space;                    // space where the access occurred
		u32                 m_count;                    // number of hits
	};
	std::vector<hotspot_entry> m_hotspots;            // hotspot list
	int                     m_hotspot_threshhold;       // threshhold for the number of hits to print

	// pc tracking
	class dasm_pc_tag
	{
	public:
		dasm_pc_tag(const offs_t& address, const u32& crc);

		// required to be included in a set
		bool operator < (const dasm_pc_tag& rhs) const
		{
			if (m_address == rhs.m_address)
				return m_crc < rhs.m_crc;
			return (m_address < rhs.m_address);
		}

		offs_t m_address;       // Stores [nothing] for a given address & crc32
		u32    m_crc;
	};
	std::set<dasm_pc_tag> m_track_pc_set;
	bool m_track_pc;

	// comments
	class dasm_comment : public dasm_pc_tag
	{
	public:
		dasm_comment(offs_t address, u32 crc, const char *text, rgb_t color);

		std::string  m_text;        // Stores comment text & color for a given address & crc32
		rgb_t        m_color;
	};
	std::set<dasm_comment> m_comment_set;               // collection of comments
	u32                 m_comment_change;            // change counter for comments

	// memory tracking
	class dasm_memory_access
	{
	public:
		dasm_memory_access(const int& address_space,
							const offs_t& address,
							const u64& data,
							const offs_t& pc);

		// required to be included in a set
		bool operator < (const dasm_memory_access& rhs) const
		{
			if ((m_address == rhs.m_address) && (m_address_space == rhs.m_address_space))
				return m_data < rhs.m_data;
			else if (m_address_space == rhs.m_address_space)
				return m_address < rhs.m_address;
			else
				return m_address_space < rhs.m_address_space;
		}

		// Stores the PC for a given address, memory region, and data value
		int m_address_space;
		offs_t           m_address;
		u64              m_data;
		mutable offs_t   m_pc;
	};
	std::set<dasm_memory_access> m_track_mem_set;
	bool m_track_mem;

	// internal flag values
	static constexpr u32 DEBUG_FLAG_OBSERVING       = 0x00000001;       // observing this CPU
	static constexpr u32 DEBUG_FLAG_HISTORY         = 0x00000002;       // tracking this CPU's history
	static constexpr u32 DEBUG_FLAG_TRACING         = 0x00000004;       // tracing this CPU
	static constexpr u32 DEBUG_FLAG_TRACING_OVER    = 0x00000008;       // tracing this CPU with step over behavior
	static constexpr u32 DEBUG_FLAG_HOOKED          = 0x00000010;       // per-instruction callback hook
	static constexpr u32 DEBUG_FLAG_STEPPING        = 0x00000020;       // CPU is single stepping
	static constexpr u32 DEBUG_FLAG_STEPPING_OVER   = 0x00000040;       // CPU is stepping over a function
	static constexpr u32 DEBUG_FLAG_STEPPING_OUT    = 0x00000080;       // CPU is stepping out of a function
	static constexpr u32 DEBUG_FLAG_STOP_PC         = 0x00000100;       // there is a pending stop at cpu->breakpc
	static constexpr u32 DEBUG_FLAG_STOP_INTERRUPT  = 0x00000400;       // there is a pending stop on the next interrupt
	static constexpr u32 DEBUG_FLAG_STOP_EXCEPTION  = 0x00000800;       // there is a pending stop on the next exception
	static constexpr u32 DEBUG_FLAG_STOP_VBLANK     = 0x00001000;       // there is a pending stop on the next VBLANK
	static constexpr u32 DEBUG_FLAG_STOP_TIME       = 0x00002000;       // there is a pending stop at cpu->stoptime
	static constexpr u32 DEBUG_FLAG_LIVE_BP         = 0x00010000;       // there are live breakpoints for this CPU

	static constexpr u32 DEBUG_FLAG_STEPPING_ANY    = DEBUG_FLAG_STEPPING | DEBUG_FLAG_STEPPING_OVER | DEBUG_FLAG_STEPPING_OUT;
	static constexpr u32 DEBUG_FLAG_TRACING_ANY     = DEBUG_FLAG_TRACING | DEBUG_FLAG_TRACING_OVER;
	static constexpr u32 DEBUG_FLAG_TRANSIENT       = DEBUG_FLAG_STEPPING_ANY | DEBUG_FLAG_STOP_PC |
			DEBUG_FLAG_STOP_INTERRUPT | DEBUG_FLAG_STOP_EXCEPTION | DEBUG_FLAG_STOP_VBLANK | DEBUG_FLAG_STOP_TIME;
};

//**************************************************************************
//  CPU DEBUGGING
//**************************************************************************

class debugger_cpu
{
public:
	debugger_cpu(running_machine &machine);

	/* ----- initialization and cleanup ----- */

	/* flushes all traces; this is useful if a trace is going on when we fatalerror */
	void flush_traces();

	void configure_memory(symbol_table &table);


	/* ----- debugging status & information ----- */

	/* return the visible CPU device (the one that commands should apply to) */
	device_t *get_visible_cpu();

	/* true if the debugger is currently stopped within an instruction hook callback */
	bool within_instruction_hook();

	/* return true if the current execution state is stopped */
	bool is_stopped();


	/* ----- symbol table interfaces ----- */

	/* return the global symbol table */
	symbol_table *get_global_symtable();

	/* return the locally-visible symbol table */
	symbol_table *get_visible_symtable();


	/* ----- misc debugger functions ----- */

	/* specifies a debug command script to execute */
	void source_script(const char *file);


	/* ----- debugger comment helpers ----- */

	// save all comments for a given machine
	bool comment_save();

	// load all comments for a given machine
	bool comment_load(bool is_inline);


	/* ----- debugger memory accessors ----- */

	/* return a byte from the specified memory space */
	u8 read_byte(address_space &space, offs_t address, bool apply_translation);

	/* return a word from the specified memory space */
	u16 read_word(address_space &space, offs_t address, bool apply_translation);

	/* return a dword from the specified memory space */
	u32 read_dword(address_space &space, offs_t address, bool apply_translation);

	/* return a qword from the specified memory space */
	u64 read_qword(address_space &space, offs_t address, bool apply_translation);

	/* return 1,2,4 or 8 bytes from the specified memory space */
	u64 read_memory(address_space &space, offs_t address, int size, bool apply_translation);

	/* write a byte to the specified memory space */
	void write_byte(address_space &space, offs_t address, u8 data, bool apply_translation);

	/* write a word to the specified memory space */
	void write_word(address_space &space, offs_t address, u16 data, bool apply_translation);

	/* write a dword to the specified memory space */
	void write_dword(address_space &space, offs_t address, u32 data, bool apply_translation);

	/* write a qword to the specified memory space */
	void write_qword(address_space &space, offs_t address, u64 data, bool apply_translation);

	/* write 1,2,4 or 8 bytes to the specified memory space */
	void write_memory(address_space &space, offs_t address, u64 data, int size, bool apply_translation);

	/* read 1,2,4 or 8 bytes at the given offset from opcode space */
	u64 read_opcode(address_space &space, offs_t offset, int size);

	// getters
	bool within_instruction_hook() const { return m_within_instruction_hook; }
	bool memory_modified() const { return m_memory_modified; }
	int execution_state() const { return m_execution_state; }
	device_t *live_cpu() { return m_livecpu; }
	u32 get_breakpoint_index() { return m_bpindex++; }
	u32 get_watchpoint_index() { return m_wpindex++; }
	u32 get_registerpoint_index() { return m_rpindex++; }

	// setters
	void set_visible_cpu(device_t * visiblecpu) { m_visiblecpu = visiblecpu; }
	void set_break_cpu(device_t * breakcpu) { m_breakcpu = breakcpu; }
	void set_within_instruction(bool within_instruction) { m_within_instruction_hook = within_instruction; }
	void set_memory_modified(bool memory_modified) { m_memory_modified = memory_modified; }
	void set_execution_state(int execution_state) { m_execution_state = execution_state; }

	// device_debug helpers
	// [TODO] [RH]: Look into this more later, can possibly merge these two classes
	void start_hook(device_t *device, bool stop_on_vblank);
	void stop_hook(device_t *device);
	void go_next_device(device_t *device);
	void go_vblank();
	void halt_on_next_instruction(device_t *device, util::format_argument_pack<std::ostream> &&args);
	void ensure_comments_loaded();
	void reset_transient_flags();
	void process_source_file();
	void watchpoint_check(address_space& space, int type, offs_t address, u64 value_to_write, u64 mem_mask, std::vector<device_debug::watchpoint *> &wplist);

private:
	static const size_t NUM_TEMP_VARIABLES;

	/* expression handlers */
	u64 expression_read_memory(void *param, const char *name, expression_space space, u32 address, int size, bool disable_se);
	u64 expression_read_program_direct(address_space &space, int opcode, offs_t address, int size);
	u64 expression_read_memory_region(const char *rgntag, offs_t address, int size);
	void expression_write_memory(void *param, const char *name, expression_space space, u32 address, int size, u64 data, bool disable_se);
	void expression_write_program_direct(address_space &space, int opcode, offs_t address, int size, u64 data);
	void expression_write_memory_region(const char *rgntag, offs_t address, int size, u64 data);
	expression_error::error_code expression_validate(void *param, const char *name, expression_space space);
	device_t* expression_get_device(const char *tag);

	/* variable getters/setters */
	u64 get_cpunum(symbol_table &table, void *ref);
	u64 get_beamx(symbol_table &table, void *ref);
	u64 get_beamy(symbol_table &table, void *ref);
	u64 get_frame(symbol_table &table, void *ref);

	/* internal helpers */
	void on_vblank(screen_device &device, bool vblank_state);

	running_machine&    m_machine;

	device_t *  m_livecpu;
	device_t *  m_visiblecpu;
	device_t *  m_breakcpu;

	std::unique_ptr<std::istream> m_source_file;        // script source file

	std::unique_ptr<symbol_table> m_symtable;           // global symbol table

	bool        m_within_instruction_hook;
	bool        m_vblank_occurred;
	bool        m_memory_modified;

	int         m_execution_state;
	device_t *  m_stop_when_not_device; // stop execution when the device ceases to be this

	u32         m_bpindex;
	u32         m_wpindex;
	u32         m_rpindex;

	u64         m_wpdata;
	u64         m_wpaddr;
	std::unique_ptr<u64[]> m_tempvar;

	osd_ticks_t m_last_periodic_update_time;

	bool        m_comments_loaded;
};

#endif // MAME_EMU_DEBUG_DEBUGCPU_H
