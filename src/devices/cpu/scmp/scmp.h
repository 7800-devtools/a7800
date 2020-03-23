// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic
#ifndef MAME_CPU_SCMP_SCMP_H
#define MAME_CPU_SCMP_SCMP_H

#pragma once


#define MCFG_SCMP_CONFIG(_flag_out_devcb, _sout_devcb, _sin_devcb, _sensea_devcb, _senseb_devcb, _halt_devcb) \
	scmp_device::set_flag_out_cb(*device, DEVCB_##_flag_out_devcb); \
	scmp_device::set_sout_cb(*device, DEVCB_##_sout_devcb); \
	scmp_device::set_sin_cb(*device, DEVCB_##_sin_devcb); \
	scmp_device::set_sensea_cb(*device, DEVCB_##_sensea_devcb); \
	scmp_device::set_senseb_cb(*device, DEVCB_##_senseb_devcb); \
	scmp_device::set_halt_cb(*device, DEVCB_##_halt_devcb);


class scmp_device : public cpu_device
{
public:
	// construction/destruction
	scmp_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration helpers
	template <class Object> static devcb_base &set_flag_out_cb(device_t &device, Object &&cb) { return downcast<scmp_device &>(device).m_flag_out_func.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_sout_cb(device_t &device, Object &&cb) { return downcast<scmp_device &>(device).m_sout_func.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_sin_cb(device_t &device, Object &&cb) { return downcast<scmp_device &>(device).m_sin_func.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_sensea_cb(device_t &device, Object &&cb) { return downcast<scmp_device &>(device).m_sensea_func.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_senseb_cb(device_t &device, Object &&cb) { return downcast<scmp_device &>(device).m_senseb_func.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_halt_cb(device_t &device, Object &&cb) { return downcast<scmp_device &>(device).m_halt_func.set_callback(std::forward<Object>(cb)); }

protected:
	enum
	{
		SCMP_PC, SCMP_P1, SCMP_P2, SCMP_P3, SCMP_AC, SCMP_ER, SCMP_SR
	};

	scmp_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_execute_interface overrides
	virtual uint32_t execute_min_cycles() const override { return 5; }
	virtual uint32_t execute_max_cycles() const override { return 131593; }
	virtual uint32_t execute_input_lines() const override { return 0; }
	virtual void execute_run() override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// device_state_interface overrides
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	// device_disasm_interface overrides
	virtual uint32_t disasm_min_opcode_bytes() const override { return 1; }
	virtual uint32_t disasm_max_opcode_bytes() const override { return 2; }
	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;

private:
	address_space_config m_program_config;

	PAIR    m_PC;
	PAIR    m_P1;
	PAIR    m_P2;
	PAIR    m_P3;
	uint8_t   m_AC;
	uint8_t   m_ER;
	uint8_t   m_SR;

	address_space *m_program;
	direct_read_data *m_direct;
	int                 m_icount;

	devcb_write8       m_flag_out_func;
	devcb_write_line   m_sout_func;
	devcb_read_line    m_sin_func;
	devcb_read_line    m_sensea_func;
	devcb_read_line    m_senseb_func;
	devcb_write_line   m_halt_func;

	inline uint16_t ADD12(uint16_t addr, int8_t val);
	inline uint8_t ROP();
	inline uint8_t ARG();
	inline uint8_t RM(uint32_t a);
	inline void WM(uint32_t a, uint8_t v);
	inline void illegal(uint8_t opcode);
	inline PAIR *GET_PTR_REG(int num);
	inline void BIN_ADD(uint8_t val);
	inline void DEC_ADD(uint8_t val);
	inline uint16_t GET_ADDR(uint8_t code);
	void execute_one(int opcode);
	void take_interrupt();
};


class ins8060_device : public scmp_device
{
public:
	// construction/destruction
	ins8060_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual uint64_t execute_clocks_to_cycles(uint64_t clocks) const override { return (clocks + 2 - 1) / 2; }
	virtual uint64_t execute_cycles_to_clocks(uint64_t cycles) const override { return (cycles * 2); }
};


DECLARE_DEVICE_TYPE(SCMP, scmp_device)
DECLARE_DEVICE_TYPE(INS8060, ins8060_device)

#endif // MAME_CPU_SCMP_SCMP_H
