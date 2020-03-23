// license:BSD-3-Clause
// copyright-holders:Joakim Larsson Edstrom
/**********************************************************************
*
*   Motorola MC68153 BIM - Bus Interrupter Module
*
*                           _____   _____
*                  Vcc   1 |*    \_/     | 40  A3
*                  RW*   2 |             | 39  A2
*                  CS*   3 |             | 38  A1
*                 DTACK* 4 |             | 37  D7
*                 IACK*  5 |             | 36  D6
*                IACKIN* 6 |             | 35  D5
*               IACKOUT* 7 |             | 34  D4
*                 IRQ1*  8 |             | 33  D3
*                  GND   9 |             | 32  D2
*                  GND  10 |             | 31  GND
*                  Vcc  11 |             | 30  VCC
*                 IRQ2* 12 |   MC68153   | 29  D1
*                 IRQ3* 13 |  EI68C153   | 28  D0
*                 IRQ4* 14 |             | 27  INTAE*
*                 IRQ5* 15 |             | 26  INTAL1
*                 IRQ6* 16 |             | 25  INTAL0
*                 IRQ7* 17 |             | 24  INT3*
*                  CLK* 18 |             | 23  INT2*
*                 INT0* 19 |             | 22  INT1*
*                  GND  20 |_____________| 21  Vcc
*
**********************************************************************/

#ifndef MAME_MACHINE_68153BIM_H
#define MAME_MACHINE_68153BIM_H

#pragma once


//**************************************************************************
//  DEVICE CONFIGURATION MACROS
//**************************************************************************

/* Variant ADD macros - use the right one to enable the right feature set! */
#define MCFG_MC68153_ADD(_tag, _clock) \
	MCFG_DEVICE_ADD(_tag, MC68153, _clock)

#define MCFG_EI68C153_ADD(_tag, _clock) \
	MCFG_DEVICE_ADD(_tag, EI68C153, _clock)

#define MCFG_BIM68153_OUT_INT_CB(_devcb) \
	devcb = &bim68153_device::set_out_int_callback(*device, DEVCB_##_devcb);

// These callback sets INTAL0 and INTAL1 but is probably not needed as the
// shorthand OUT_IACK0..OUT_IACK3 below embedd the channel information
#define MCFG_BIM68153_OUT_INTAL0_CB(_devcb) \
	devcb = &bim68153_device::set_out_intal0_callback(*device, DEVCB_##_devcb);

#define MCFG_BIM68153_OUT_INTAL1_CB(_devcb) \
	devcb = &bim68153_device::set_out_intal1_callback(*device, DEVCB_##_devcb);

// LOCAL IACK callbacks emulating the INTAL0 and INTAL1 outputs for INTAE requesting a vector from a sub device
#define MCFG_BIM68153_OUT_IACK0_CB(_devcb)                              \
	devcb = &bim68153_device::set_out_iack0_callback(*device, DEVCB_##_devcb);
#define MCFG_BIM68153_OUT_IACK1_CB(_devcb)                              \
	devcb = &bim68153_device::set_out_iack1_callback(*device, DEVCB_##_devcb);
#define MCFG_BIM68153_OUT_IACK2_CB(_devcb)                              \
	devcb = &bim68153_device::set_out_iack2_callback(*device, DEVCB_##_devcb);
#define MCFG_BIM68153_OUT_IACK3_CB(_devcb)                              \
	devcb = &bim68153_device::set_out_iack3_callback(*device, DEVCB_##_devcb);

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class bim68153_device;

class bim68153_channel : public device_t
{
	friend class bim68153_device;

public:
	bim68153_channel(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// read register handlers
	uint8_t do_bimreg_control_r();
	uint8_t do_bimreg_vector_r();

	// write register handlers
	void do_bimreg_control_w(uint8_t data);
	void do_bimreg_vector_w(uint8_t vector);

	DECLARE_WRITE_LINE_MEMBER( int_w );
	devcb_read8 m_out_iack_cb;
	uint8_t m_int_state;

	// Register state
	uint8_t m_control;
	uint8_t m_vector;

protected:
	enum
	{
		NONE     = 0,
		PENDING  = 1,
	};

	enum
	{
		RCV_IDLE     = 0,
		RCV_SEEKING  = 1,
		RCV_SAMPLING = 2
	};

	enum
	{
		REG_CNTRL_INT_LVL_MSK   = 0x07,
		REG_CNTRL_INT_AUT_DIS   = 0x08,
		REG_CNTRL_INT_ENABLE    = 0x10,
		REG_CNTRL_INT_EXT       = 0x20,
		REG_CNTRL_INT_AUT_CLR   = 0x40,
		REG_CNTRL_INT_FLAG      = 0x80,
	};
	int m_int;      // interrupt request from connected device
	int m_index;    // Which channel am I?
	bim68153_device *m_bim; // my device
};


class bim68153_device : public device_t
{
	friend class bim68153_channel;

public:
	// construction/destruction
	bim68153_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	IRQ_CALLBACK_MEMBER(iack);
	int acknowledge();
	int get_irq_level();

	template <class Object> static devcb_base &set_out_int_callback(device_t &device, Object &&cb) { return downcast<bim68153_device &>(device).m_out_int_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_intal0_callback(device_t &device, Object &&cb) { return downcast<bim68153_device &>(device).m_out_intal0_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_intal1_callback(device_t &device, Object &&cb) { return downcast<bim68153_device &>(device).m_out_intal1_cb.set_callback(std::forward<Object>(cb)); }

	template <class Object> static devcb_base &set_out_iack0_callback(device_t &device, Object &&cb) { return downcast<bim68153_device &>(device).m_chn[CHN_0]->m_out_iack_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_iack1_callback(device_t &device, Object &&cb) { return downcast<bim68153_device &>(device).m_chn[CHN_1]->m_out_iack_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_iack2_callback(device_t &device, Object &&cb) { return downcast<bim68153_device &>(device).m_chn[CHN_2]->m_out_iack_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_iack4_callback(device_t &device, Object &&cb) { return downcast<bim68153_device &>(device).m_chn[CHN_3]->m_out_iack_cb.set_callback(std::forward<Object>(cb)); }

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );


	DECLARE_WRITE_LINE_MEMBER( iackin_w ) { m_iackin = state; }
	DECLARE_WRITE_LINE_MEMBER( int0_w ) { m_chn[CHN_0]->int_w(state); }
	DECLARE_WRITE_LINE_MEMBER( int1_w ) { m_chn[CHN_1]->int_w(state); }
	DECLARE_WRITE_LINE_MEMBER( int2_w ) { m_chn[CHN_2]->int_w(state); }
	DECLARE_WRITE_LINE_MEMBER( int3_w ) { m_chn[CHN_3]->int_w(state); }

protected:
	bim68153_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, uint32_t variant);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;

	void trigger_interrupt(int ch);
	int get_channel_index(bim68153_channel *ch);

	// Register enums
	enum
	{
		REG_CONTROL  = 0x00,
		REG_VECTOR   = 0x04,
		CHN_MSK      = 0x03,
		CHN_0        = 0x00,
		CHN_1        = 0x01,
		CHN_2        = 0x02,
		CHN_3        = 0x03,
	};

	enum
	{
		INT_CHN0 = 0,
		INT_CHN1 = 1,
		INT_CHN2 = 2,
		INT_CHN3 = 3,
	};

	// Variants of BIM
	enum
	{
		TYPE_MC68153   = 0x001,
		TYPE_EI68C153  = 0x002,
	};

	required_device<bim68153_channel> m_chn[4];

	devcb_write_line    m_out_int_cb;

	// iack signalling towards subdevices, see also out_iack_callbacks in each bim68153_channel which overlaps
	devcb_write_line    m_out_intal0_cb;
	devcb_write_line    m_out_intal1_cb;

	// Daisy chain signals
	devcb_read8         m_out_iackout_cb;
	int m_iackin;
	int m_irq_level;
};

class ei68c153_device : public bim68153_device
{
public :
	ei68c153_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

// device type definition
DECLARE_DEVICE_TYPE(MC68153,         bim68153_device)
DECLARE_DEVICE_TYPE(EI68C153,        ei68c153_device)
DECLARE_DEVICE_TYPE(MC68153_CHANNEL, bim68153_channel)

#endif // MAME_MACHINE_68153BIM_H
