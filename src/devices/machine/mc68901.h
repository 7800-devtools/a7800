// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Motorola MC68901 Multi Function Peripheral emulation

**********************************************************************
                            _____   _____
                  R/_W   1 |*    \_/     | 48  _CS
                   RS1   2 |             | 47  _DS
                   RS2   3 |             | 46  _DTACK
                   RS3   4 |             | 45  _IACK
                   RS4   5 |             | 44  D7
                   RS5   6 |             | 43  D6
                    TC   7 |             | 42  D5
                    SO   8 |             | 41  D4
                    SI   9 |             | 40  D3
                    RC  10 |             | 39  D2
                   Vcc  11 |             | 38  D1
                    NC  12 |   MC68901   | 37  D0
                   TAO  13 |   MK68901   | 36  GND
                   TBO  14 |             | 35  CLK
                   TCO  15 |             | 34  _IEI
                   TDO  16 |             | 33  _IEO
                 XTAL1  17 |             | 32  _IRQ
                 XTAL2  18 |             | 31  _RR
                   TAI  19 |             | 30  _TR
                   TBI  20 |             | 29  I7
                _RESET  21 |             | 28  I6
                    I0  22 |             | 27  I5
                    I1  23 |             | 26  I4
                    I2  24 |_____________| 25  I3

**********************************************************************/

#ifndef MAME_MACHINE_MC68901_H
#define MAME_MACHINE_MC68901_H

#pragma once



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_MC68901_TIMER_CLOCK(_clk) \
	mc68901_device::set_timer_clock(*device, _clk);

#define MCFG_MC68901_RX_CLOCK(_clk) \
	mc68901_device::set_rx_clock(*device, _clk);

#define MCFG_MC68901_TX_CLOCK(_clk) \
	mc68901_device::set_tx_clock(*device, _clk);

#define MCFG_MC68901_OUT_IRQ_CB(_devcb) \
	devcb = &mc68901_device::set_out_irq_callback(*device, DEVCB_##_devcb);

#define MCFG_MC68901_OUT_GPIO_CB(_devcb) \
	devcb = &mc68901_device::set_out_gpio_callback(*device, DEVCB_##_devcb);

#define MCFG_MC68901_OUT_TAO_CB(_devcb) \
	devcb = &mc68901_device::set_out_tao_callback(*device, DEVCB_##_devcb);

#define MCFG_MC68901_OUT_TBO_CB(_devcb) \
	devcb = &mc68901_device::set_out_tbo_callback(*device, DEVCB_##_devcb);

#define MCFG_MC68901_OUT_TCO_CB(_devcb) \
	devcb = &mc68901_device::set_out_tco_callback(*device, DEVCB_##_devcb);

#define MCFG_MC68901_OUT_TDO_CB(_devcb) \
	devcb = &mc68901_device::set_out_tdo_callback(*device, DEVCB_##_devcb);

#define MCFG_MC68901_OUT_SO_CB(_devcb) \
	devcb = &mc68901_device::set_out_so_callback(*device, DEVCB_##_devcb);

/*#define MCFG_MC68901_OUT_RR_CB(_devcb) \
    devcb = &mc68901_device::set_out_rr_callback(*device, DEVCB_##_devcb);

#define MCFG_MC68901_OUT_TR_CB(_devcb) \
    devcb = &mc68901_device::set_out_tr_callback(*device, DEVCB_##_devcb);*/


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************


// ======================> mc68901_device

class mc68901_device :  public device_t,
						public device_serial_interface
{
public:
	// construction/destruction
	mc68901_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void set_timer_clock(device_t &device, int timer_clock) { downcast<mc68901_device &>(device).m_timer_clock = timer_clock; }
	static void set_rx_clock(device_t &device, int rx_clock) { downcast<mc68901_device &>(device).m_rx_clock = rx_clock; }
	static void set_tx_clock(device_t &device, int tx_clock) { downcast<mc68901_device &>(device).m_tx_clock = tx_clock; }
	template <class Object> static devcb_base &set_out_irq_callback(device_t &device, Object &&cb) { return downcast<mc68901_device &>(device).m_out_irq_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_gpio_callback(device_t &device, Object &&cb) { return downcast<mc68901_device &>(device).m_out_gpio_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_tao_callback(device_t &device, Object &&cb) { return downcast<mc68901_device &>(device).m_out_tao_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_tbo_callback(device_t &device, Object &&cb) { return downcast<mc68901_device &>(device).m_out_tbo_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_tco_callback(device_t &device, Object &&cb) { return downcast<mc68901_device &>(device).m_out_tco_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_tdo_callback(device_t &device, Object &&cb) { return downcast<mc68901_device &>(device).m_out_tdo_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_so_callback(device_t &device, Object &&cb) { return downcast<mc68901_device &>(device).m_out_so_cb.set_callback(std::forward<Object>(cb)); }
	//template <class Object> static devcb_base &set_rr_callback(device_t &device, Object &&cb) { return downcast<mc68901_device &>(device).m_out_rr_cb.set_callback(std::forward<Object>(cb)); }
	//template <class Object> static devcb_base &set_tr_callback(device_t &device, Object &&cb) { return downcast<mc68901_device &>(device).m_out_tr_cb.set_callback(std::forward<Object>(cb)); }

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );

	int get_vector();

	DECLARE_WRITE_LINE_MEMBER( i0_w );
	DECLARE_WRITE_LINE_MEMBER( i1_w );
	DECLARE_WRITE_LINE_MEMBER( i2_w );
	DECLARE_WRITE_LINE_MEMBER( i3_w );
	DECLARE_WRITE_LINE_MEMBER( i4_w );
	DECLARE_WRITE_LINE_MEMBER( i5_w );
	DECLARE_WRITE_LINE_MEMBER( i6_w );
	DECLARE_WRITE_LINE_MEMBER( i7_w );

	DECLARE_WRITE_LINE_MEMBER( tai_w );
	DECLARE_WRITE_LINE_MEMBER( tbi_w );

	DECLARE_WRITE_LINE_MEMBER( write_rx );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// device_serial_interface overrides
	virtual void tra_callback() override;
	virtual void tra_complete() override;
	virtual void rcv_complete() override;

	void check_interrupts();
	void take_interrupt(uint16_t mask);
	void rx_buffer_full();
	void rx_error();
	void timer_count(int index);
	void timer_input(int index, int value);
	void gpio_input(int bit, int state);
	void gpio_output();
	void register_w(offs_t offset, uint8_t data);

private:
	enum
	{
		TIMER_A = 0,
		TIMER_B,
		TIMER_C,
		TIMER_D
	};

	enum
	{
		REGISTER_GPIP = 0,
		REGISTER_AER,
		REGISTER_DDR,
		REGISTER_IERA,
		REGISTER_IERB,
		REGISTER_IPRA,
		REGISTER_IPRB,
		REGISTER_ISRA,
		REGISTER_ISRB,
		REGISTER_IMRA,
		REGISTER_IMRB,
		REGISTER_VR,
		REGISTER_TACR,
		REGISTER_TBCR,
		REGISTER_TCDCR,
		REGISTER_TADR,
		REGISTER_TBDR,
		REGISTER_TCDR,
		REGISTER_TDDR,
		REGISTER_SCR,
		REGISTER_UCR,
		REGISTER_RSR,
		REGISTER_TSR,
		REGISTER_UDR
	};

	enum
	{
		INT_GPI0 = 0,
		INT_GPI1,
		INT_GPI2,
		INT_GPI3,
		INT_TIMER_D,
		INT_TIMER_C,
		INT_GPI4,
		INT_GPI5,
		INT_TIMER_B,
		INT_XMIT_ERROR,
		INT_XMIT_BUFFER_EMPTY,
		INT_RCV_ERROR,
		INT_RCV_BUFFER_FULL,
		INT_TIMER_A,
		INT_GPI6,
		INT_GPI7
	};

	enum
	{
		GPIP_0 = 0,
		GPIP_1,
		GPIP_2,
		GPIP_3,
		GPIP_4,
		GPIP_5,
		GPIP_6,
		GPIP_7
	};

	enum
	{
		SERIAL_START = 0,
		SERIAL_DATA,
		SERIAL_PARITY,
		SERIAL_STOP
	};

	enum
	{
		XMIT_OFF = 0,
		XMIT_STARTING,
		XMIT_ON,
		XMIT_BREAK,
		XMIT_STOPPING
	};

	static const int INT_MASK_GPIO[];
	static const int INT_MASK_TIMER[];
	static const int GPIO_TIMER[];
	static const int PRESCALER[];

	int m_timer_clock;      /* timer clock */
	int m_rx_clock;         /* serial receive clock */
	int m_tx_clock;         /* serial transmit clock */

	devcb_write_line        m_out_irq_cb;

	devcb_write8            m_out_gpio_cb;

	devcb_write_line        m_out_tao_cb;
	devcb_write_line        m_out_tbo_cb;
	devcb_write_line        m_out_tco_cb;
	devcb_write_line        m_out_tdo_cb;

	devcb_write_line        m_out_so_cb;
	//devcb_write_line        m_out_rr_cb;
	//devcb_write_line        m_out_tr_cb;

	//int m_device_type;                      /* device type */

	/* registers */
	uint8_t m_gpip;                           /* general purpose I/O register */
	uint8_t m_aer;                            /* active edge register */
	uint8_t m_ddr;                            /* data direction register */

	uint16_t m_ier;                           /* interrupt enable register */
	uint16_t m_ipr;                           /* interrupt pending register */
	uint16_t m_isr;                           /* interrupt in-service register */
	uint16_t m_imr;                           /* interrupt mask register */
	uint8_t m_vr;                             /* vector register */

	uint8_t m_tacr;                           /* timer A control register */
	uint8_t m_tbcr;                           /* timer B control register */
	uint8_t m_tcdcr;                          /* timers C and D control register */
	uint8_t m_tdr[4];     /* timer data registers */

	uint8_t m_scr;                            /* synchronous character register */
	uint8_t m_ucr;                            /* USART control register */
	uint8_t m_tsr;                            /* transmitter status register */
	uint8_t m_rsr;                            /* receiver status register */
	uint8_t m_transmit_buffer;                /* USART data register */
	int m_transmit_pending;
	uint8_t m_receive_buffer;
	int m_receive_pending;
	uint8_t m_gpio_input;
	uint8_t m_gpio_output;

	/* counter timer state */
	uint8_t m_tmc[4];     /* timer main counters */
	int m_ti[4];            /* timer in latch */
	int m_to[4];            /* timer out latch */

	/* interrupt state */
	//int m_irqlevel;                         /* interrupt level latch */

	/* serial state */
	uint8_t m_next_rsr;                       /* receiver status register latch */
	int m_rsr_read;                         /* receiver status register read flag */

	// timers
	emu_timer *m_timer[4]; /* counter timers */
};


// device type definition
DECLARE_DEVICE_TYPE(MC68901, mc68901_device)

#endif // MAME_MACHINE_MC68901_H
