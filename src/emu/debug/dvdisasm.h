// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/*********************************************************************

    dvdisasm.h

    Disassembly debugger view.

***************************************************************************/

#ifndef MAME_EMU_DEBUG_DVDISASM_H
#define MAME_EMU_DEBUG_DVDISASM_H

#pragma once

#include "debugvw.h"

#include "vecstream.h"


//**************************************************************************
//  CONSTANTS
//**************************************************************************

// selections for what goes into the right-hand column
enum disasm_right_column
{
	DASM_RIGHTCOL_NONE,
	DASM_RIGHTCOL_RAW,
	DASM_RIGHTCOL_ENCRYPTED,
	DASM_RIGHTCOL_COMMENTS
};



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// a disassembly view_source
class debug_view_disasm_source : public debug_view_source
{
	friend class debug_view_disasm;

	// construction/destruction
	debug_view_disasm_source(const char *name, device_t &device);

public:
	// getters
	address_space &space() const { return m_space; }

private:
	// internal state
	device_disasm_interface *m_disasmintf;      // disassembly interface
	address_space &     m_space;                // address space to display
	address_space &     m_decrypted_space;      // address space to display for decrypted opcodes
};


// debug view for disassembly
class debug_view_disasm : public debug_view
{
	friend class debug_view_manager;

	// construction/destruction
	debug_view_disasm(running_machine &machine, debug_view_osd_update_func osdupdate, void *osdprivate);
	virtual ~debug_view_disasm();

public:
	// getters
	const char *expression() const { return m_expression.string(); }
	disasm_right_column right_column() const { return m_right_column; }
	u32 backward_steps() const { return m_backwards_steps; }
	u32 disasm_width() const { return m_dasm_width; }
	offs_t selected_address();

	// setters
	void set_expression(const std::string &expression);
	void set_right_column(disasm_right_column contents);
	void set_backward_steps(u32 steps);
	void set_disasm_width(u32 width);
	void set_selected_address(offs_t address);

protected:
	// view overrides
	virtual void view_update() override;
	virtual void view_notify(debug_view_notification type) override;
	virtual void view_char(int chval) override;
	virtual void view_click(const int button, const debug_view_xy& pos) override;

private:
	// The information of one disassembly line. May become the actual
	// external interface at one point
	struct dasm_line {
		offs_t m_byteaddress;                   // address of the first byte of the instruction
		std::string m_adr;                      // instruction address as a string
		std::string m_dasm;                     // disassembly
		std::string m_rawdata;                  // textual representation of the instruction values
		std::string m_encdata;                  // textual representation of encrypted instruction values
		std::string m_comment;                  // comment, when present

		bool operator == (const dasm_line &right) const {
			return
				m_byteaddress == right.m_byteaddress &&
				m_adr == right.m_adr &&
				m_dasm == right.m_dasm &&
				m_rawdata == right.m_rawdata &&
				m_encdata == right.m_encdata &&
				m_comment == right.m_comment;
		}

		bool operator != (const dasm_line &right) const {
			return !(*this == right);
		}
	};

	// internal helpers
	void enumerate_sources();
	offs_t find_pc_backwards(offs_t targetpc, int numinstrs);
	std::string generate_bytes(offs_t pcbyte, int numbytes, int granularity, bool encrypted);
	bool recompute(offs_t pc, int startline, int lines);
	void print(int row, std::string text, int start, int end, u8 attrib);

	// internal state
	disasm_right_column m_right_column;         // right column contents
	u32                 m_backwards_steps;      // number of backwards steps
	u32                 m_dasm_width;           // width of the disassembly area
	u8 *                m_last_direct_raw;      // last direct raw value
	u8 *                m_last_direct_decrypted;// last direct decrypted value
	u32                 m_last_change_count;    // last comment change count
	offs_t              m_last_pcbyte;          // last PC byte value
	int                 m_divider1, m_divider2; // left and right divider columns
	int                 m_divider3;             // comment divider column
	debug_view_expression m_expression;         // expression-related information
	std::vector<dasm_line> m_dasm;              // disassembled instructions

	// constants
	static constexpr int DEFAULT_DASM_LINES = 1000;
	static constexpr int DEFAULT_DASM_WIDTH = 50;
	static constexpr int DASM_MAX_BYTES = 16;
};

#endif // MAME_EMU_DEBUG_DVDISASM_H
