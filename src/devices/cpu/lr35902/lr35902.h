// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
#ifndef MAME_CPU_LR35902_LR35902_H
#define MAME_CPU_LR35902_LR35902_H

#pragma once


#define MCFG_LR35902_TIMER_CB(_devcb) \
	devcb = &lr35902_cpu_device::set_timer_cb(*device, DEVCB_##_devcb);

// The first release of this CPU has a bug where the programcounter
// is not incremented properly after an interrupt after the halt opcode.
// This was fixed in a newer revision.
#define MCFG_LR35902_HALT_BUG \
	lr35902_cpu_device::set_halt_bug(*device);

// The GameBoy has a bug where OAM data gets corrupted if you inc/dec
// a 16-bit register in the $fe** region.
// note: oldval is in hiword, newval is in loword
#define MCFG_LR35902_INCDEC16_CB(_devcb) \
	devcb = &lr35902_cpu_device::set_incdec16_cb(*device, DEVCB_##_devcb);


enum
{
	LR35902_PC=1, LR35902_SP, LR35902_A, LR35902_F, LR35902_B, LR35902_C, LR35902_D, LR35902_E, LR35902_H, LR35902_L,
	LR35902_IRQ_STATE,
	/* Pseudo registers to keep track of the interrupt statuses */
	LR35902_IE, LR35902_IF,
	/* Pseudo register to change and check the cpu operating speed */
	LR35902_SPEED
};


class lr35902_cpu_device : public cpu_device
{
public:
	// construction/destruction
	lr35902_cpu_device(const machine_config &mconfig, const char *_tag, device_t *_owner, uint32_t _clock);

	// static configuration helpers
	template<class _Object> static devcb_base &set_timer_cb(device_t &device, _Object object) { return downcast<lr35902_cpu_device &>(device).m_timer_func.set_callback(object); }
	template<class _Object> static devcb_base &set_incdec16_cb(device_t &device, _Object object) { return downcast<lr35902_cpu_device &>(device).m_incdec16_func.set_callback(object); }
	static void set_halt_bug(device_t &device) { downcast<lr35902_cpu_device &>(device).m_has_halt_bug = true; }

	uint8_t get_speed();
	void set_speed(uint8_t speed_request);

	inline uint8_t get_ie() { return m_IE; }
	inline void set_ie(uint8_t data) { m_IE = data; }

	inline uint8_t get_if() { return m_IF; }
	inline void set_if(uint8_t data) { m_IF = data; }

	inline void dma_cycles_to_burn(uint16_t cycles_to_burn) { m_dma_cycles_to_burn += cycles_to_burn; }

	// Needed for some gameboy operation which needs to read the results
	// of setting an input during the currently running timeslice.
	// Can become protected again once this core becomes cycle accurate.
	virtual void execute_set_input(int inputnum, int state) override;

	enum
	{
		/* Interrupts */
		VBL_INT = 0,    /* V-Blank    */
		LCD_INT = 1,    /* LCD Status */
		TIM_INT = 2,    /* Timer      */
		SIO_INT = 3,    /* Serial I/O */
		EXT_INT = 4     /* Joypad     */
	};

protected:

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_execute_interface overrides
	virtual uint32_t execute_min_cycles() const override { return 1; }
	virtual uint32_t execute_max_cycles() const override { return 16; }
	virtual uint32_t execute_input_lines() const override { return 5; }
	virtual void execute_run() override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// device_state_interface overrides
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	// device_disasm_interface overrides
	virtual uint32_t disasm_min_opcode_bytes() const override { return 1; }
	virtual uint32_t disasm_max_opcode_bytes() const override { return 4; }
	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;

	inline void cycles_passed(uint8_t cycles);
	inline uint8_t mem_read_byte(uint16_t addr);
	inline void mem_write_byte(uint16_t addr, uint8_t data);
	inline uint16_t mem_read_word(uint16_t addr);
	inline void mem_write_word(uint16_t addr, uint16_t data);
	inline void check_interrupts();

	address_space_config m_program_config;

	uint8_t m_A;
	uint8_t m_F;
	uint8_t m_B;
	uint8_t m_C;
	uint8_t m_D;
	uint8_t m_E;
	uint8_t m_H;
	uint8_t m_L;

	uint16_t m_SP;
	uint16_t m_PC;

	/* Interrupt related */
	uint8_t m_IE;
	uint8_t m_IF;
	int m_irq_state;
	bool m_handle_ei_delay;
	lr35902_cpu_device *m_device;
	address_space *m_program;
	int m_icount;

	/* Fetch & execute related */
	int m_execution_state;
	uint8_t m_op;

	/* Others */
	int m_gb_speed;
	int m_gb_speed_change_pending;
	int m_enable;
	bool m_has_halt_bug;
	uint32_t m_dma_cycles_to_burn;
	bool m_entering_halt;

	/* Callbacks */
	devcb_write8 m_timer_func;
	devcb_write32 m_incdec16_func;
};

DECLARE_DEVICE_TYPE(LR35902, lr35902_cpu_device)

#endif // MAME_CPU_LR35902_LR35902_H
