// license:BSD-3-Clause
// copyright-holders:Dirk Best
/***************************************************************************

    Input Merger

    Used to connect multiple lines to a single device input while
    keeping it pulled high or low

***************************************************************************/

#ifndef MAME_MACHINE_INPUT_MERGER_H
#define MAME_MACHINE_INPUT_MERGER_H

#pragma once



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_INPUT_MERGER_ACTIVE_HIGH(_tag) \
	MCFG_DEVICE_ADD(_tag, INPUT_MERGER_ACTIVE_HIGH, 0)

#define MCFG_INPUT_MERGER_ACTIVE_LOW(_tag) \
	MCFG_DEVICE_ADD(_tag, INPUT_MERGER_ACTIVE_LOW, 0)

#define MCFG_INPUT_MERGER_OUTPUT_HANDLER(_devcb) \
	devcb = &input_merger_device::set_output_handler(*device, DEVCB_##_devcb);


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> input_merger_device

class input_merger_device : public device_t
{
public:
	// callback
	template <class Object> static devcb_base &set_output_handler(device_t &device, Object &&cb)
	{ return downcast<input_merger_device &>(device).m_output_handler.set_callback(std::forward<Object>(cb)); }

	// input lines
	DECLARE_WRITE_LINE_MEMBER( in0_w ) { if (bool(state) != m_state[0]) { m_state[0] = state; update_state(); } }
	DECLARE_WRITE_LINE_MEMBER( in1_w ) { if (bool(state) != m_state[1]) { m_state[1] = state; update_state(); } }
	DECLARE_WRITE_LINE_MEMBER( in2_w ) { if (bool(state) != m_state[2]) { m_state[2] = state; update_state(); } }
	DECLARE_WRITE_LINE_MEMBER( in3_w ) { if (bool(state) != m_state[3]) { m_state[3] = state; update_state(); } }
	DECLARE_WRITE_LINE_MEMBER( in4_w ) { if (bool(state) != m_state[4]) { m_state[4] = state; update_state(); } }
	DECLARE_WRITE_LINE_MEMBER( in5_w ) { if (bool(state) != m_state[5]) { m_state[5] = state; update_state(); } }
	DECLARE_WRITE_LINE_MEMBER( in6_w ) { if (bool(state) != m_state[6]) { m_state[6] = state; update_state(); } }
	DECLARE_WRITE_LINE_MEMBER( in7_w ) { if (bool(state) != m_state[7]) { m_state[7] = state; update_state(); } }

protected:
	// constructor/destructor
	input_merger_device(machine_config const &mconfig, device_type type, char const *tag, device_t *owner, uint32_t clock);
	virtual ~input_merger_device() override;

	// device-level overrides
	virtual void device_start() override;

	virtual void update_state() = 0;

	devcb_write_line m_output_handler;
	bool m_state[8];
};

// ======================> input_merger_active_high_device

class input_merger_active_high_device : public input_merger_device
{
public:
	input_merger_active_high_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_reset() override;

	// input_merger device overrides
	virtual void update_state() override;
};

// ======================> input_merger_active_low_device

class input_merger_active_low_device : public input_merger_device
{
public:
	input_merger_active_low_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_reset() override;

	// input_merger device overrides
	virtual void update_state() override;
};


// device type definition
DECLARE_DEVICE_TYPE(INPUT_MERGER_ACTIVE_HIGH, input_merger_active_high_device)
DECLARE_DEVICE_TYPE(INPUT_MERGER_ACTIVE_LOW,  input_merger_active_low_device)

#endif // MAME_MACHINE_INPUT_MERGER_H
