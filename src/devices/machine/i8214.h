// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Intel 8214 Priority Interrupt Controller emulation

**********************************************************************
                            _____   _____
                   _B0   1 |*    \_/     | 24  Vcc
                   _B1   2 |             | 23  _ECS
                   _B2   3 |             | 22  _R7
                  _SGS   4 |             | 21  _R6
                  _INT   5 |             | 20  _R5
                  _CLK   6 |    8214     | 19  _R4
                  INTE   7 |             | 18  _R3
                   _A0   8 |             | 17  _R2
                   _A1   9 |             | 16  _R1
                   _A2  10 |             | 15  _R0
                  _ELR  11 |             | 14  ENLG
                   GND  12 |_____________| 13  ETLG

**********************************************************************/

#ifndef MAME_MACHINE_I8214_H
#define MAME_MACHINE_I8214_H

#pragma once



///*************************************************************************
//  INTERFACE CONFIGURATION MACROS
///*************************************************************************

#define MCFG_I8214_IRQ_CALLBACK(_write) \
	devcb = &i8214_device::set_irq_wr_callback(*device, DEVCB_##_write);

#define MCFG_I8214_ENLG_CALLBACK(_write) \
	devcb = &i8214_device::set_enlg_wr_callback(*device, DEVCB_##_write);



///*************************************************************************
//  TYPE DEFINITIONS
///*************************************************************************

// ======================> i8214_device

class i8214_device : public device_t
{
public:
	// construction/destruction
	i8214_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_irq_wr_callback(device_t &device, Object &&cb) { return downcast<i8214_device &>(device).m_write_irq.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_enlg_wr_callback(device_t &device, Object &&cb) { return downcast<i8214_device &>(device).m_write_enlg.set_callback(std::forward<Object>(cb)); }

	DECLARE_WRITE_LINE_MEMBER( sgs_w );
	DECLARE_WRITE_LINE_MEMBER( etlg_w );
	DECLARE_WRITE_LINE_MEMBER( inte_w );

	uint8_t a_r();
	void b_w(uint8_t data);
	void r_w(int line, int state);

protected:
	// device-level overrides
	virtual void device_start() override;

private:
	void trigger_interrupt(int level);
	void check_interrupt();
	void update_interrupt_line();

	devcb_write_line   m_write_irq;
	devcb_write_line   m_write_enlg;

	int m_inte;                 // interrupt enable
	int m_int_dis;              // interrupt (latch) disable flip-flop
	int m_a;                    // request level
	int m_current_status;
	uint8_t m_r;                  // interrupt request latch
	int m_sgs;                  // status group select
	int m_etlg;                 // enable this level group
};


// device type definition
DECLARE_DEVICE_TYPE(I8214, i8214_device)

#endif // MAME_MACHINE_I8214_H
