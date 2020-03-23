// license:BSD-3-Clause
// copyright-holders:Andrew Gardner
/***************************************************************************

    dsp56k.h
    Interface file for the portable Motorola/Freescale DSP56k emulator.
    Written by Andrew Gardner

***************************************************************************/


#ifndef MAME_CPU_DSP56K_DSP56K_H
#define MAME_CPU_DSP56K_DSP56K_H

#pragma once



// IRQ Lines
// MODA and MODB are also known as IRQA and IRQB
#define DSP56K_IRQ_MODA  0
#define DSP56K_IRQ_MODB  1
#define DSP56K_IRQ_MODC  2
#define DSP56K_IRQ_RESET 3  /* Is this needed? */


namespace DSP56K {

/***************************************************************************
    STRUCTURES & TYPEDEFS
***************************************************************************/
// 5-4 Host Interface
struct dsp56k_host_interface
{
	// **** Dsp56k side **** //
	// Host Control Register
	uint16_t* hcr;

	// Host Status Register
	uint16_t* hsr;

	// Host Transmit/Receive Data
	uint16_t* htrx;

	// **** Host CPU side **** //
	// Interrupt Control Register
	uint8_t icr;

	// Command Vector Register
	uint8_t cvr;

	// Interrupt Status Register
	uint8_t isr;

	// Interrupt Vector Register
	uint8_t ivr;

	// Transmit / Receive Registers
	uint8_t trxh;
	uint8_t trxl;

	// HACK - Host interface bootstrap write offset
	uint16_t bootstrap_offset;

};

// 1-9 ALU
struct dsp56k_data_alu
{
	// Four 16-bit input registers (can be accessed as 2 32-bit registers)
	PAIR x;
	PAIR y;

	// Two 32-bit accumulator registers + 8-bit accumulator extension registers
	PAIR64 a;
	PAIR64 b;

	// An accumulation shifter
	// One data bus shifter/limiter
	// A parallel, single cycle, non-pipelined Multiply-Accumulator (MAC) unit
	// Basics
};

// 1-10 Address Generation Unit (AGU)
struct dsp56k_agu
{
	// Four address registers
	uint16_t r0;
	uint16_t r1;
	uint16_t r2;
	uint16_t r3;

	// Four offset registers
	uint16_t n0;
	uint16_t n1;
	uint16_t n2;
	uint16_t n3;

	// Four modifier registers
	uint16_t m0;
	uint16_t m1;
	uint16_t m2;
	uint16_t m3;

	// Used in loop processing
	uint16_t temp;

	// FM.4-5 - hmmm?
	// uint8_t status;

	// Basics
};

// 1-11 Program Control Unit (PCU)
struct dsp56k_pcu
{
	// Program Counter
	uint16_t pc;

	// Loop Address
	uint16_t la;

	// Loop Counter
	uint16_t lc;

	// Status Register
	uint16_t sr;

	// Operating Mode Register
	uint16_t omr;

	// Stack Pointer
	uint16_t sp;

	// Stack (TODO: 15-level?)
	PAIR ss[16];

	// Controls IRQ processing
	void (*service_interrupts)(void);

	// A list of pending interrupts (indices into dsp56k_interrupt_sources array)
	int8_t pending_interrupts[32];

	// Basics

	// Other PCU internals
	uint16_t reset_vector;

};

// 1-8 The dsp56156 CORE
struct dsp56k_core
{
	// PROGRAM CONTROLLER
	dsp56k_pcu PCU;

	// ADR ALU (AGU)
	dsp56k_agu AGU;

	// CLOCK GEN
	//static emu_timer *dsp56k_timer;   // 1-5, 1-8 - Clock gen

	// DATA ALU
	dsp56k_data_alu ALU;

	// OnCE

	// IBS and BITFIELD UNIT

	// Host Interface
	dsp56k_host_interface HI;

	// IRQ line states
	bool modA_state;
	bool modB_state;
	bool modC_state;
	bool reset_state;

	// HACK - Bootstrap mode state variable.
	uint8_t bootstrap_mode;

	uint8_t   repFlag;    // Knowing if we're in a 'repeat' state (dunno how the processor does this)
	uint32_t  repAddr;    // The address of the instruction to repeat...


	/* MAME internal stuff */
	int icount;

	uint32_t          ppc;
	uint32_t          op;
	int             interrupt_cycles;
	void            (*output_pins_changed)(uint32_t pins);
	cpu_device *device;
	address_space *program;
	direct_read_data *direct;
	address_space *data;

	uint16_t peripheral_ram[0x40];
	uint16_t *program_ram;
};


class dsp56k_device : public cpu_device
{
public:
	dsp56k_device(const machine_config &mconfig, const char *_tag, device_t *_owner, uint32_t _clock);

	DECLARE_READ16_MEMBER( peripheral_register_r );
	DECLARE_WRITE16_MEMBER( peripheral_register_w );

	void  host_interface_write(uint8_t offset, uint8_t data);
	uint8_t host_interface_read(uint8_t offset);

	uint16_t get_peripheral_memory(uint16_t addr);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_execute_interface overrides
	virtual uint64_t execute_clocks_to_cycles(uint64_t clocks) const override { return (clocks + 2 - 1) / 2; }
	virtual uint64_t execute_cycles_to_clocks(uint64_t cycles) const override { return (cycles * 2); }
	virtual uint32_t execute_min_cycles() const override { return 1; }
	virtual uint32_t execute_max_cycles() const override { return 8; }
	virtual uint32_t execute_input_lines() const override { return 4; }
	virtual uint32_t execute_default_irq_vector() const override { return 0; }
	virtual void execute_run() override;
	virtual void execute_set_input(int inputnum, int state) override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// device_state_interface overrides
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	// device_disasm_interface overrides
	virtual uint32_t disasm_min_opcode_bytes() const override { return 2; }
	virtual uint32_t disasm_max_opcode_bytes() const override { return 4; }
	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;

private:
	address_space_config m_program_config;
	address_space_config m_data_config;
	required_shared_ptr<uint16_t> m_program_ram;

	dsp56k_core m_dsp56k_core;

	void agu_init();
	void alu_init();
};

} // namespace DSP56K


DECLARE_DEVICE_TYPE_NS(DSP56156, DSP56K, dsp56k_device)
using DSP56K::dsp56k_device;

extern CPU_DISASSEMBLE( dsp56k );

#endif // MAME_CPU_DSP56K_DSP56K_H
