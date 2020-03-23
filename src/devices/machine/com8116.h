// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    COM8116 Dual Baud Rate Generator (Programmable Divider) emulation

    COM5016 is a mostly-compatible clone of this chip, with +12V on
    pin 9 rather than NC.

**********************************************************************
                            _____   _____
             XTAL/EXT1   1 |*    \_/     | 18  XTAL/EXT2
                   +5V   2 |             | 17  fT
                    fR   3 |             | 16  Ta
                    Ra   4 |   COM8116   | 15  Tb
                    Rb   5 |   COM8116T  | 14  Tc
                    Rc   6 |   COM8136   | 13  Td
                    Rd   7 |   COM8136T  | 12  STT
                   STR   8 |             | 11  GND
                    NC   9 |_____________| 10  fX/4

**********************************************************************/

#ifndef MAME_MACHINE_COM8116_H
#define MAME_MACHINE_COM8116_H

#pragma once




///*************************************************************************
//  INTERFACE CONFIGURATION MACROS
///*************************************************************************

#define MCFG_COM8116_FX4_HANDLER(_devcb) \
	devcb = &com8116_device::set_fx4_handler(*device, DEVCB_##_devcb);

#define MCFG_COM8116_FR_HANDLER(_devcb) \
	devcb = &com8116_device::set_fr_handler(*device, DEVCB_##_devcb);

#define MCFG_COM8116_FT_HANDLER(_devcb) \
	devcb = &com8116_device::set_ft_handler(*device, DEVCB_##_devcb);


///*************************************************************************
//  TYPE DEFINITIONS
///*************************************************************************

// ======================> com8116_device

class com8116_device :  public device_t
{
public:
	// construction/destruction
	com8116_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_fx4_handler(device_t &device, Object &&cb) { return downcast<com8116_device &>(device).m_fx4_handler.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_fr_handler(device_t &device, Object &&cb) { return downcast<com8116_device &>(device).m_fr_handler.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_ft_handler(device_t &device, Object &&cb) { return downcast<com8116_device &>(device).m_ft_handler.set_callback(std::forward<Object>(cb)); }

	void str_w(uint8_t data);
	DECLARE_WRITE8_MEMBER( str_w );
	void stt_w(uint8_t data);
	DECLARE_WRITE8_MEMBER( stt_w );

protected:
	static const int divisors_16X_5_0688MHz[];
	static const int divisors_16X_6_01835MHz[];
	static const int divisors_16X_4_9152MHz[];
	static const int divisors_32X_5_0688MHz[];
	static const int divisors_16X_2_7648MHz[];
	static const int divisors_16X_5_0688MHz_030[];
	static const int divisors_16X_4_6080MHz[];
	static const int divisors_16X_4_9152MHz_SY2661_1[];
	static const int divisors_16X_4_9152MHz_SY2661_2[];

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int m_param, void *ptr) override;

private:
	enum
	{
		TIMER_FX4,
		TIMER_FR,
		TIMER_FT
	};

	devcb_write_line   m_fx4_handler;
	devcb_write_line   m_fr_handler;
	devcb_write_line   m_ft_handler;

	int m_fx4;
	int m_fr;
	int m_ft;

	const int *m_fr_divisors;
	const int *m_ft_divisors;

	// timers
	emu_timer *m_fx4_timer;
	emu_timer *m_fr_timer;
	emu_timer *m_ft_timer;
};


// device type definition
DECLARE_DEVICE_TYPE(COM8116, com8116_device)

#endif // MAME_MACHINE_COM8116_H
