// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/**********************************************************************

    5/74174/5 Hex/Quad D Flip-Flops with Clear

***********************************************************************

    Connection Diagram:
              ___ ___                         ___ ___
    CLEAR  1 |*  u   | 16  Vcc      CLEAR  1 |*  u   | 16  Vcc
       Q1  2 |       | 15  Q6          Q1  2 |       | 15  Q4
       D1  3 |       | 14  D6         /Q1  3 |       | 14  /Q4
       D2  4 |       | 13  D5          D1  4 |       | 13  D4
       Q2  5 |       | 12  Q5          D2  5 |       | 12  D3
       D3  6 |       | 11  D4         /Q2  6 |       | 11  /Q3
       Q3  7 |       | 10  Q4          Q2  7 |       | 10  Q3
      GND  8 |_______| 9   CLOCK      GND  8 |_______| 9   CLOCK

              5/74174                         5/74175

***********************************************************************

    Function Table:
     _________________________________
    |       Inputs        |  Outputs* |
    |---------------------|-----------|
    | Clear | Clock |  D  |  Q  | /Q  |
    |-------|-------|-----|-----|-----|
    |   L   |   X   |  X  |  L  |  H  |
    |   H   |   ^   |  H  |  H  |  L  |
    |   H   |   ^   |  L  |  L  |  H  |
    |   H   |   L   |  X  |  Q0 |  Q0 |
    |_______|_______|_____|_____|_____|

    H = High Level (steady state)
    L = Low Level (steady state)
    X = Don't Care
    ^ = Transition from low to high level
    Q0 = The level of Q before the indicated steady-state input conditions were established.
    * = 175 only

**********************************************************************/

#ifndef MAME_MACHINE_74175_H
#define MAME_MACHINE_74175_H

#pragma once


#define MCFG_74174_Q1_CB(_devcb) \
	devcb = &ttl741745_device::set_q1_cb(*device, DEVCB_##_devcb);

#define MCFG_74174_Q2_CB(_devcb) \
	devcb = &ttl741745_device::set_q2_cb(*device, DEVCB_##_devcb);

#define MCFG_74174_Q3_CB(_devcb) \
	devcb = &ttl741745_device::set_q3_cb(*device, DEVCB_##_devcb);

#define MCFG_74174_Q4_CB(_devcb) \
	devcb = &ttl741745_device::set_q4_cb(*device, DEVCB_##_devcb);

#define MCFG_74175_Q1_CB(_devcb) \
	devcb = &ttl741745_device::set_q1_cb(*device, DEVCB_##_devcb);

#define MCFG_74175_Q2_CB(_devcb) \
	devcb = &ttl741745_device::set_q2_cb(*device, DEVCB_##_devcb);

#define MCFG_74175_Q3_CB(_devcb) \
	devcb = &ttl741745_device::set_q3_cb(*device, DEVCB_##_devcb);

#define MCFG_74175_Q4_CB(_devcb) \
	devcb = &ttl741745_device::set_q4_cb(*device, DEVCB_##_devcb);

#define MCFG_74174_Q5_CB(_devcb) \
	devcb = &ttl74174_device::set_q5_cb(*device, DEVCB_##_devcb);

#define MCFG_74174_Q6_CB(_devcb) \
	devcb = &ttl74174_device::set_q6_cb(*device, DEVCB_##_devcb);

#define MCFG_74175_NOT_Q1_CB(_devcb) \
	devcb = &ttl74175_device::set_not_q1_cb(*device, DEVCB_##_devcb);

#define MCFG_74175_NOT_Q2_CB(_devcb) \
	devcb = &ttl74175_device::set_not_q2_cb(*device, DEVCB_##_devcb);

#define MCFG_74175_NOT_Q3_CB(_devcb) \
	devcb = &ttl74175_device::set_not_q3_cb(*device, DEVCB_##_devcb);

#define MCFG_74175_NOT_Q4_CB(_devcb) \
	devcb = &ttl74175_device::set_not_q1_cb(*device, DEVCB_##_devcb);

#define MCFG_74174_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, TTL74174, 0)

#define MCFG_74175_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, TTL74175, 0)

class ttl741745_device : public device_t
{
public:
	template <class Object> static devcb_base &set_q1_cb(device_t &device, Object &&cb) { return downcast<ttl741745_device &>(device).m_q1_func.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_q2_cb(device_t &device, Object &&cb) { return downcast<ttl741745_device &>(device).m_q2_func.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_q3_cb(device_t &device, Object &&cb) { return downcast<ttl741745_device &>(device).m_q3_func.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_q4_cb(device_t &device, Object &&cb) { return downcast<ttl741745_device &>(device).m_q4_func.set_callback(std::forward<Object>(cb)); }

	DECLARE_WRITE_LINE_MEMBER( clear_w );
	DECLARE_WRITE_LINE_MEMBER( d1_w );
	DECLARE_WRITE_LINE_MEMBER( d2_w );
	DECLARE_WRITE_LINE_MEMBER( d3_w );
	DECLARE_WRITE_LINE_MEMBER( d4_w );
	DECLARE_WRITE_LINE_MEMBER( clock_w );

	uint8_t q_w();

protected:
	ttl741745_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override;
	virtual void device_reset() override;

	virtual void init();
	virtual void tick();

	devcb_write_line m_q1_func;
	devcb_write_line m_q2_func;
	devcb_write_line m_q3_func;
	devcb_write_line m_q4_func;

	uint8_t m_clock;
	uint8_t m_clear;

	uint8_t m_d1;
	uint8_t m_d2;
	uint8_t m_d3;
	uint8_t m_d4;

	uint8_t m_q1;
	uint8_t m_q2;
	uint8_t m_q3;
	uint8_t m_q4;
};

class ttl74174_device : public ttl741745_device
{
public:
	ttl74174_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_q5_cb(device_t &device, Object &&cb) { return downcast<ttl74174_device &>(device).m_q5_func.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_q6_cb(device_t &device, Object &&cb) { return downcast<ttl74174_device &>(device).m_q6_func.set_callback(std::forward<Object>(cb)); }

	DECLARE_WRITE_LINE_MEMBER( d5_w );
	DECLARE_WRITE_LINE_MEMBER( d6_w );

protected:
	virtual void device_start() override;

	virtual void init() override;
	virtual void tick() override;

private:
	devcb_write_line m_q5_func;
	devcb_write_line m_q6_func;

	uint8_t m_d5;
	uint8_t m_d6;

	uint8_t m_q5;
	uint8_t m_q6;
};

class ttl74175_device : public ttl741745_device
{
public:
	ttl74175_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_not_q1_cb(device_t &device, Object &&cb) { return downcast<ttl74175_device &>(device).m_not_q1_func.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_not_q2_cb(device_t &device, Object &&cb) { return downcast<ttl74175_device &>(device).m_not_q2_func.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_not_q3_cb(device_t &device, Object &&cb) { return downcast<ttl74175_device &>(device).m_not_q3_func.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_not_q4_cb(device_t &device, Object &&cb) { return downcast<ttl74175_device &>(device).m_not_q4_func.set_callback(std::forward<Object>(cb)); }

protected:
	virtual void device_start() override;

	virtual void tick() override;

private:
	devcb_write_line m_not_q1_func;
	devcb_write_line m_not_q2_func;
	devcb_write_line m_not_q3_func;
	devcb_write_line m_not_q4_func;

	uint8_t m_not_q1;
	uint8_t m_not_q2;
	uint8_t m_not_q3;
	uint8_t m_not_q4;
};

// device type definition
DECLARE_DEVICE_TYPE(TTL74174, ttl74174_device)
DECLARE_DEVICE_TYPE(TTL74175, ttl74175_device)

#endif // MAME_MACHINE_74175_H
