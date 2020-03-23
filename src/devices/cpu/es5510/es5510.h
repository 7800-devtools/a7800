// license:BSD-3-Clause
// copyright-holders:Christian Brunschen
/**********************************************************************************************
 *
 *   es5510.h - Ensoniq ES5510 (ESP) driver
 *   by Christian Brunschen
 *
 **********************************************************************************************/

#ifndef MAME_CPU_ES5510_ES5510_H
#define MAME_CPU_ES5510_ES5510_H

#pragma once


class es5510_device : public cpu_device {
public:
	es5510_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER(host_r);
	DECLARE_WRITE8_MEMBER(host_w);

	int16_t ser_r(int offset);
	void ser_w(int offset, int16_t data);

	enum line_t {
		ES5510_HALT = 0
	};

	enum state_t {
		STATE_RUNNING = 0,
		STATE_HALTED = 1
	};

	struct alu_op_t {
		int operands;
		const char * const opcode;
	};

	enum op_src_dst_t {
		SRC_DST_REG =   1 << 0,
		SRC_DST_DELAY = 1 << 1,
		SRC_DST_BOTH =  (1 << 0) | (1 << 1)
	};

	struct op_select_t {
		const op_src_dst_t alu_src;
		const op_src_dst_t alu_dst;
		const op_src_dst_t mac_src;
		const op_src_dst_t mac_dst;
	};

	enum ram_control_access_t {
		RAM_CONTROL_DELAY = 0,
		RAM_CONTROL_TABLE_A,
		RAM_CONTROL_TABLE_B,
		RAM_CONTROL_IO
	};

	enum ram_cycle_t {
		RAM_CYCLE_READ = 0,
		RAM_CYCLE_WRITE = 1,
		RAM_CYCLE_DUMP_FIFO = 2
	};

	struct ram_control_t {
		ram_cycle_t cycle;
		ram_control_access_t access;
		const char * const description;
	};

	static const alu_op_t ALU_OPS[16];
	static const op_select_t OPERAND_SELECT[16];
	static const ram_control_t RAM_CONTROL[8];

	struct alu_t {
		uint8_t aReg;
		uint8_t bReg;
		op_src_dst_t src;
		op_src_dst_t dst;
		uint8_t op;
		int32_t aValue;
		int32_t bValue;
		int32_t result;
		bool update_ccr;
		bool write_result;
	};

	struct mulacc_t {
		uint8_t cReg;
		uint8_t dReg;
		op_src_dst_t src;
		op_src_dst_t dst;
		bool accumulate;
		int32_t cValue;
		int32_t dValue;
		int64_t product;
		int64_t result;
		bool write_result;
	};

	struct ram_t {
		int32_t address;     // up to 20 bits, left-justified within the right 24 bits of the 32-bit word
		bool io;           // I/O space, rather than delay line memory
		ram_cycle_t cycle; // cycle type
	};

	// direct access to the 'HALT' pin - not just through the
	void set_HALT(bool halt) { halt_asserted = halt; }
	bool get_HALT() { return halt_asserted; }

	void run_once();
	void list_program(void(p)(const char *, ...));

	// for testing purposes
	uint64_t &_instr(int pc) { return instr[pc % 160]; }
	int16_t &_dram(int addr) { return dram[addr & 0xfffff]; }

	// publicly visible for testing purposes
	int32_t read_reg(uint8_t reg);
	void write_reg(uint8_t reg, int32_t value);
	void write_to_dol(int32_t value);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual space_config_vector memory_space_config() const override;
	virtual uint64_t execute_clocks_to_cycles(uint64_t clocks) const override;
	virtual uint64_t execute_cycles_to_clocks(uint64_t cycles) const override;
	virtual uint32_t execute_min_cycles() const override;
	virtual uint32_t execute_max_cycles() const override;
	virtual uint32_t execute_input_lines() const override;
	virtual void execute_run() override;
	virtual uint32_t disasm_min_opcode_bytes() const override;
	virtual uint32_t disasm_max_opcode_bytes() const override;
	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;
	virtual void execute_set_input(int linenum, int state) override;

	int32_t alu_operation(uint8_t op, int32_t aValue, int32_t bValue, uint8_t &flags);
	void alu_operation_end();

private:
	int icount;
	bool halt_asserted;
	uint8_t pc;
	state_t state;
	int32_t gpr[0xc0];     // 24 bits, right justified
	int16_t ser0r;
	int16_t ser0l;
	int16_t ser1r;
	int16_t ser1l;
	int16_t ser2r;
	int16_t ser2l;
	int16_t ser3r;
	int16_t ser3l;
	int64_t machl;        // 48 bits, right justified and sign extended
	bool mac_overflow;  // whether reading the MAC register should return a saturated replacement value
	int32_t dil;
	int32_t memsiz;
	int32_t memmask;
	int32_t memincrement;
	int8_t memshift;
	int32_t dlength;
	int32_t abase;
	int32_t bbase;
	int32_t dbase;
	int32_t sigreg;
	int mulshift;
	int8_t ccr;           // really, 5 bits, left justified
	int8_t cmr;           // really, 6 bits, left justified
	int32_t dol[2];
	int dol_count;

	uint64_t instr[160];    // 48 bits, right justified
	int16_t dram[1<<20];   // there are up to 20 address bits (at least 16 expected), left justified within the 24 bits of a gpr or dadr; we preallocate all of it.

	// latch registers for host interaction
	int32_t  dol_latch;     // 24 bits
	int32_t  dil_latch;     // 24 bits
	uint32_t dadr_latch;    // 24 bits
	int32_t  gpr_latch;     // 24 bits, holding up to 20 address bits, left justified
	uint64_t instr_latch;   // 48 bits, right justified
	uint8_t  ram_sel;       // effectively a boolean
	uint8_t  host_control;  //

	// currently executing instruction(s)
	alu_t alu;
	mulacc_t mulacc;
	ram_t ram, ram_p, ram_pp; // ram operations for cycles N, N-1 and N-2
};

DECLARE_DEVICE_TYPE(ES5510, es5510_device)

#endif // MAME_CPU_ES5510_ES5510_H
