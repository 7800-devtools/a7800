// license:GPL-2.0+
// copyright-holders:Couriersud
/***************************************************************************

    netlist.h

    Discrete netlist implementation.

****************************************************************************/

#ifndef MAME_MACHINE_NETLIST_H
#define MAME_MACHINE_NETLIST_H

#include "netlist/nl_time.h"

class nld_sound_out;
class nld_sound_in;

namespace netlist {
	class setup_t;
	class param_double_t;
	class param_int_t;
	class param_logic_t;
	class param_ptr_t;
}


// MAME specific configuration


#define MCFG_NETLIST_SETUP(_setup)                                                  \
	netlist_mame_device::static_set_constructor(*device, NETLIST_NAME(_setup));

#define MCFG_NETLIST_ANALOG_INPUT(_basetag, _tag, _name)                            \
	MCFG_DEVICE_ADD(_basetag ":" _tag, NETLIST_ANALOG_INPUT, 0)                     \
	netlist_mame_analog_input_device::static_set_name(*device, _name);

#define MCFG_NETLIST_ANALOG_MULT_OFFSET(_mult, _offset)                             \
	netlist_mame_sub_interface::static_set_mult_offset(*device, _mult, _offset);

#define MCFG_NETLIST_ANALOG_OUTPUT(_basetag, _tag, _IN, _class, _member, _class_tag) \
	MCFG_DEVICE_ADD(_basetag ":" _tag, NETLIST_ANALOG_OUTPUT, 0)                    \
	netlist_mame_analog_output_device::static_set_params(*device, _IN,              \
				netlist_mame_analog_output_device::output_delegate(& _class :: _member, \
						# _class "::" # _member, _class_tag, (_class *)nullptr)   );

#define MCFG_NETLIST_LOGIC_OUTPUT(_basetag, _tag, _IN, _class, _member, _class_tag) \
	MCFG_DEVICE_ADD(_basetag ":" _tag, NETLIST_LOGIC_OUTPUT, 0)                    \
	netlist_mame_logic_output_device::static_set_params(*device, _IN,              \
				netlist_mame_logic_output_device::output_delegate(& _class :: _member, \
						# _class "::" # _member, _class_tag, (_class *)nullptr)   );

#define MCFG_NETLIST_LOGIC_INPUT(_basetag, _tag, _name, _shift)             \
	MCFG_DEVICE_ADD(_basetag ":" _tag, NETLIST_LOGIC_INPUT, 0)              \
	netlist_mame_logic_input_device::static_set_params(*device, _name, _shift);

#define MCFG_NETLIST_INT_INPUT(_basetag, _tag, _name, _shift, _mask)        \
	MCFG_DEVICE_ADD(_basetag ":" _tag, NETLIST_INT_INPUT, 0)                \
	netlist_mame_int_input_device::static_set_params(*device, _name, _mask, _shift);

#define MCFG_NETLIST_RAM_POINTER(_basetag, _tag, _name) \
	MCFG_DEVICE_ADD(_basetag ":" _tag, NETLIST_RAM_POINTER, 0) \
	netlist_mame_ram_pointer_device::static_set_params(*device, _name ".m_RAM");

#define MCFG_NETLIST_STREAM_INPUT(_basetag, _chan, _name)                           \
	MCFG_DEVICE_ADD(_basetag ":cin" # _chan, NETLIST_STREAM_INPUT, 0)               \
	netlist_mame_stream_input_device::static_set_params(*device, _chan, _name);

#define MCFG_NETLIST_STREAM_OUTPUT(_basetag, _chan, _name)                          \
	MCFG_DEVICE_ADD(_basetag ":cout" # _chan, NETLIST_STREAM_OUTPUT, 0)             \
	netlist_mame_stream_output_device::static_set_params(*device, _chan, _name);

#define NETLIST_LOGIC_PORT_CHANGED(_base, _tag)                                     \
	PORT_CHANGED_MEMBER(_base ":" _tag, netlist_mame_logic_input_device, input_changed, 0)

#define NETLIST_INT_PORT_CHANGED(_base, _tag)                                     \
	PORT_CHANGED_MEMBER(_base ":" _tag, netlist_mame_logic_input_device, input_changed, 0)

#define NETLIST_ANALOG_PORT_CHANGED(_base, _tag)                                    \
	PORT_CHANGED_MEMBER(_base ":" _tag, netlist_mame_analog_input_device, input_changed, 0)


#define MEMREGION_SOURCE(_name) \
		netlist_mame_device::register_memregion_source(setup, _name);

#define NETDEV_ANALOG_CALLBACK_MEMBER(_name) \
	void _name(const double data, const attotime &time)

#define NETDEV_LOGIC_CALLBACK_MEMBER(_name) \
	void _name(const int data, const attotime &time)



// ----------------------------------------------------------------------------------------
// netlist_mame_device
// ----------------------------------------------------------------------------------------

class netlist_mame_device : public device_t
{
public:
	class netlist_mame_t;

	// construction/destruction
	netlist_mame_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~netlist_mame_device();

	static void static_set_constructor(device_t &device, void (*setup_func)(netlist::setup_t &));

	ATTR_HOT inline netlist::setup_t &setup();
	ATTR_HOT inline netlist_mame_t &netlist() { return *m_netlist; }

	ATTR_HOT inline const netlist::netlist_time last_time_update() { return m_old; }
	ATTR_HOT void update_time_x();
	ATTR_HOT void check_mame_abort_slice();

	static void register_memregion_source(netlist::setup_t &setup, const char *name);

	int m_icount;

protected:
	netlist_mame_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// Custom to netlist ...
	virtual void nl_register_devices() { }

	// device_t overrides
	virtual void device_config_complete() override;
	virtual void device_validity_check(validity_checker &valid) const override;
	virtual void device_start() override;
	virtual void device_stop() override;
	virtual void device_reset() override;
	virtual void device_post_load() override;
	virtual void device_pre_save() override;
	virtual void device_clock_changed() override;

	netlist::netlist_time m_div;

private:
	void save_state();

	/* timing support here - so sound can hijack it ... */
	netlist::netlist_time        m_rem;
	netlist::netlist_time        m_old;

	netlist_mame_t *    m_netlist;

	void (*m_setup_func)(netlist::setup_t &);
};

// ----------------------------------------------------------------------------------------
// netlist_mame_cpu_device
// ----------------------------------------------------------------------------------------

class netlist_mame_cpu_device : public netlist_mame_device,
								public device_execute_interface,
								public device_state_interface,
								public device_disasm_interface,
								public device_memory_interface
{
public:
	// construction/destruction
	netlist_mame_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// netlist_mame_device
	virtual void nl_register_devices() override;

	// device_t overrides
	virtual void device_start() override;

	// device_execute_interface overrides
	virtual uint64_t execute_clocks_to_cycles(uint64_t clocks) const override;
	virtual uint64_t execute_cycles_to_clocks(uint64_t cycles) const override;

	ATTR_HOT virtual void execute_run() override;

	// device_disasm_interface overrides
	ATTR_COLD virtual uint32_t disasm_min_opcode_bytes() const override { return 1; }
	ATTR_COLD virtual uint32_t disasm_max_opcode_bytes() const override { return 1; }
	ATTR_COLD virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	//  device_state_interface overrides
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	address_space_config m_program_config;

private:
	int m_genPC;
};

// ----------------------------------------------------------------------------------------
// netlist_mame_sound_device
// ----------------------------------------------------------------------------------------

class netlist_mame_sound_device : public netlist_mame_device,
								  public device_sound_interface
{
public:
	// construction/destruction
	netlist_mame_sound_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	inline sound_stream *get_stream() { return m_stream; }


	// device_sound_interface overrides
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

protected:
	// netlist_mame_device
	virtual void nl_register_devices() override;

	// device_t overrides
	virtual void device_start() override;

private:
	static constexpr int MAX_OUT = 10;
	nld_sound_out *m_out[MAX_OUT];
	nld_sound_in *m_in;
	sound_stream *m_stream;
	int m_num_inputs;
	int m_num_outputs;

};

// ----------------------------------------------------------------------------------------
// netlist_mame_sub_interface
// ----------------------------------------------------------------------------------------

class netlist_mame_sub_interface
{
public:
	// construction/destruction
	netlist_mame_sub_interface(device_t &aowner)
		: m_offset(0.0), m_mult(1.0)
		, m_owner(dynamic_cast<netlist_mame_device *>(&aowner))
		, m_sound(dynamic_cast<netlist_mame_sound_device *>(&aowner))
	{
	}

	virtual void custom_netlist_additions(netlist::setup_t &setup) { }
	virtual void pre_parse_action(netlist::setup_t &setup) { }

	inline netlist_mame_device &nl_owner() const { return *m_owner; }

	inline bool is_sound_device() const { return bool(m_sound); }

	inline void update_to_current_time() { m_sound->get_stream()->update(); }

	static void static_set_mult_offset(device_t &device, const double mult, const double offset);

protected:
	double m_offset;
	double m_mult;

private:
	netlist_mame_device *const m_owner;
	netlist_mame_sound_device *const m_sound;
};

// ----------------------------------------------------------------------------------------
// netlist_mame_analog_input_device
// ----------------------------------------------------------------------------------------

class netlist_mame_analog_input_device : public device_t, public netlist_mame_sub_interface
{
public:

	// construction/destruction
	netlist_mame_analog_input_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void static_set_name(device_t &device, const char *param_name);

	void write(const double val);

	inline DECLARE_INPUT_CHANGED_MEMBER(input_changed)
	{
		if (m_auto_port)
			write((double(newval) - double(field.minval())) / double(field.maxval() - field.minval()));
		else
			write(newval);
	}
	inline DECLARE_WRITE_LINE_MEMBER(write_line)       { write(state);  }
	inline DECLARE_WRITE8_MEMBER(write8)               { write(data);   }
	inline DECLARE_WRITE16_MEMBER(write16)             { write(data);   }
	inline DECLARE_WRITE32_MEMBER(write32)             { write(data);   }
	inline DECLARE_WRITE64_MEMBER(write64)             { write(data);   }

protected:
	// device-level overrides
	virtual void device_start() override;

private:
	netlist::param_double_t *m_param;
	bool   m_auto_port;
	const char *m_param_name;
};

// ----------------------------------------------------------------------------------------
// netlist_mame_analog_output_device
// ----------------------------------------------------------------------------------------

class netlist_mame_analog_output_device : public device_t, public netlist_mame_sub_interface
{
public:
	typedef device_delegate<void (const double, const attotime &)> output_delegate;

	// construction/destruction
	netlist_mame_analog_output_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void static_set_params(device_t &device, const char *in_name, output_delegate &&adelegate);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void custom_netlist_additions(netlist::setup_t &setup) override;

private:
	const char *m_in;
	output_delegate m_delegate;
};

// ----------------------------------------------------------------------------------------
// netlist_mame_logic_output_device
// ----------------------------------------------------------------------------------------

class netlist_mame_logic_output_device : public device_t, public netlist_mame_sub_interface
{
public:
	typedef device_delegate<void(const int, const attotime &)> output_delegate;

	// construction/destruction
	netlist_mame_logic_output_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void static_set_params(device_t &device, const char *in_name, output_delegate &&adelegate);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void custom_netlist_additions(netlist::setup_t &setup) override;

private:
	const char *m_in;
	output_delegate m_delegate;
};

// ----------------------------------------------------------------------------------------
// netlist_mame_int_input_device
// ----------------------------------------------------------------------------------------

class netlist_mame_int_input_device : public device_t, public netlist_mame_sub_interface
{
public:
	// construction/destruction
	netlist_mame_int_input_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void static_set_params(device_t &device, const char *param_name, const uint32_t mask, const uint32_t shift);

	void write(const uint32_t val);

	inline DECLARE_INPUT_CHANGED_MEMBER(input_changed) { write(newval); }
	DECLARE_WRITE_LINE_MEMBER(write_line)       { write(state);  }
	DECLARE_WRITE8_MEMBER(write8)               { write(data);   }
	DECLARE_WRITE16_MEMBER(write16)             { write(data);   }
	DECLARE_WRITE32_MEMBER(write32)             { write(data);   }
	DECLARE_WRITE64_MEMBER(write64)             { write(data);   }

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	netlist::param_int_t *m_param;
	uint32_t m_mask;
	uint32_t m_shift;
	const char *m_param_name;
};


// ----------------------------------------------------------------------------------------
// netlist_mame_logic_input_device
// ----------------------------------------------------------------------------------------

class netlist_mame_logic_input_device : public device_t, public netlist_mame_sub_interface
{
public:
	// construction/destruction
	netlist_mame_logic_input_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void static_set_params(device_t &device, const char *param_name, const uint32_t shift);

	void write(const uint32_t val);

	inline DECLARE_INPUT_CHANGED_MEMBER(input_changed) { write(newval); }
	DECLARE_WRITE_LINE_MEMBER(write_line)       { write(state);  }
	DECLARE_WRITE8_MEMBER(write8)               { write(data);   }
	DECLARE_WRITE16_MEMBER(write16)             { write(data);   }
	DECLARE_WRITE32_MEMBER(write32)             { write(data);   }
	DECLARE_WRITE64_MEMBER(write64)             { write(data);   }

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	netlist::param_logic_t *m_param;
	uint32_t m_shift;
	const char *m_param_name;
};

// ----------------------------------------------------------------------------------------
// netlist_mame_ram_pointer_device
// ----------------------------------------------------------------------------------------

class netlist_mame_ram_pointer_device : public device_t, public netlist_mame_sub_interface
{
public:
	// construction/destruction
	netlist_mame_ram_pointer_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	uint8_t* ptr() const { return m_data; }

	static void static_set_params(device_t &device, const char *param_name);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	netlist::param_ptr_t *m_param;
	const char *m_param_name;
	uint8_t* m_data;
};

// ----------------------------------------------------------------------------------------
// netlist_mame_stream_input_device
// ----------------------------------------------------------------------------------------

class netlist_mame_stream_input_device : public device_t, public netlist_mame_sub_interface
{
public:
	// construction/destruction
	netlist_mame_stream_input_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void static_set_params(device_t &device, int channel, const char *param_name);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void custom_netlist_additions(netlist::setup_t &setup) override;
private:
	uint32_t m_channel;
	const char *m_param_name;
};

// ----------------------------------------------------------------------------------------
// netlist_mame_stream_output_device
// ----------------------------------------------------------------------------------------

class netlist_mame_stream_output_device : public device_t, public netlist_mame_sub_interface
{
public:
	// construction/destruction
	netlist_mame_stream_output_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void static_set_params(device_t &device, int channel, const char *out_name);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void custom_netlist_additions(netlist::setup_t &setup) override;

private:
	uint32_t m_channel;
	const char *m_out_name;
};


// device type definition
DECLARE_DEVICE_TYPE(NETLIST_CORE,          netlist_mame_device)
DECLARE_DEVICE_TYPE(NETLIST_CPU,           netlist_mame_cpu_device)
DECLARE_DEVICE_TYPE(NETLIST_SOUND,         netlist_mame_sound_device)
DECLARE_DEVICE_TYPE(NETLIST_ANALOG_INPUT,  netlist_mame_analog_input_device)
DECLARE_DEVICE_TYPE(NETLIST_LOGIC_INPUT,   netlist_mame_logic_input_device)
DECLARE_DEVICE_TYPE(NETLIST_INT_INPUT,     netlist_mame_int_input_device)
DECLARE_DEVICE_TYPE(NETLIST_RAM_POINTER,   netlist_mame_ram_pointer_device)
DECLARE_DEVICE_TYPE(NETLIST_LOGIC_OUTPUT,  netlist_mame_logic_output_device)
DECLARE_DEVICE_TYPE(NETLIST_ANALOG_OUTPUT, netlist_mame_analog_output_device)
DECLARE_DEVICE_TYPE(NETLIST_STREAM_INPUT,  netlist_mame_stream_input_device)
DECLARE_DEVICE_TYPE(NETLIST_STREAM_OUTPUT, netlist_mame_stream_output_device)

#endif // MAME_MACHINE_NETLIST_H
