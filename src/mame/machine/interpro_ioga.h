// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

#ifndef MAME_MACHINE_INTERPRO_IOGA_H
#define MAME_MACHINE_INTERPRO_IOGA_H

#pragma once

#define MCFG_INTERPRO_IOGA_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, INTERPRO_IOGA, XTAL_12_5MHz)

#define MCFG_INTERPRO_IOGA_NMI_CB(_out_nmi) \
	devcb = &interpro_ioga_device::static_set_out_nmi_callback(*device, DEVCB_##_out_nmi);

#define MCFG_INTERPRO_IOGA_IRQ_CB(_out_irq) \
	devcb = &interpro_ioga_device::static_set_out_irq_callback(*device, DEVCB_##_out_irq);

#define MCFG_INTERPRO_IOGA_DMA_CB(_channel, _dma_r, _dma_w) \
	devcb = &interpro_ioga_device::static_set_dma_r_callback(*device, _channel, DEVCB_##_dma_r); \
	devcb = &interpro_ioga_device::static_set_dma_w_callback(*device, _channel, DEVCB_##_dma_w);

#define MCFG_INTERPRO_IOGA_DMA_SERIAL_CB(_channel, _dma_r, _dma_w) \
	devcb = &interpro_ioga_device::static_set_dma_serial_r_callback(*device, _channel, DEVCB_##_dma_r); \
	devcb = &interpro_ioga_device::static_set_dma_serial_w_callback(*device, _channel, DEVCB_##_dma_w);

#define MCFG_INTERPRO_IOGA_FDCTC_CB(_tc) \
	devcb = &interpro_ioga_device::static_set_fdc_tc_callback(*device, DEVCB_##_tc);

#define MCFG_INTERPRO_IOGA_ETH_CA_CB(_ca) \
	devcb = &interpro_ioga_device::static_set_eth_ca_callback(*device, DEVCB_##_ca);

#define MCFG_INTERPRO_IOGA_DMA_BUS(_mmu, _space)

// timer 0 seem to be a 60Hz cycle
#define IOGA_TIMER0_IRQ     14

// best guess for timer 1 is 10MHz based on typical prescaler value of 1000 and timer value of 100 for a delay of 100ms
#define IOGA_TIMER1_IRQ     15
#define IOGA_TIMER1_VMASK   0xffff
#define IOGA_TIMER1_START   0x10000
#define IOGA_TIMER1_EXPIRED 0x20000

// best guess for timer 3 is 12.5MHz based on typical value of 12500 for a delay of 1ms
#define IOGA_TIMER3_CLOCK   XTAL_12_5MHz
#define IOGA_TIMER3_IRQ     1
#define IOGA_TIMER3_VMASK   0x3fffffff
#define IOGA_TIMER3_START   0x40000000
#define IOGA_TIMER3_EXPIRED 0x80000000

#define IOGA_INTERRUPT_COUNT           19
#define IOGA_INTERRUPT_PENDING         0x0100
#define IOGA_INTERRUPT_ENABLE_EXTERNAL 0x0200
#define IOGA_INTERRUPT_EDGE            0x0400
#define IOGA_INTERRUPT_NEGPOL          0x0800
#define IOGA_INTERRUPT_ENABLE_INTERNAL 0x1000

#define IOGA_INTERRUPT_NONE     0
#define IOGA_INTERRUPT_NMI      1
#define IOGA_INTERRUPT_INTERNAL 2
#define IOGA_INTERRUPT_EXTERNAL 3
#define IOGA_INTERRUPT_SOFT_LO  4
#define IOGA_INTERRUPT_SOFT_HI  5

#define IOGA_DMA_CHANNELS 3
#define IOGA_DMA_PLOTTER  0
#define IOGA_DMA_SCSI     1
#define IOGA_DMA_FLOPPY   2

#define IOGA_DMA_SERIAL_CHANNELS 3
#define IOGA_DMA_SERIAL0  0
#define IOGA_DMA_SERIAL1  1
#define IOGA_DMA_SERIAL2  2

class interpro_ioga_device : public device_t
{
public:
	interpro_ioga_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template<class _Object> static devcb_base &static_set_out_nmi_callback(device_t &device, _Object object) { return downcast<interpro_ioga_device &>(device).m_out_nmi_func.set_callback(object); }
	template<class _Object> static devcb_base &static_set_out_irq_callback(device_t &device, _Object object) { return downcast<interpro_ioga_device &>(device).m_out_irq_func.set_callback(object); }

	template<class _Object> static devcb_base &static_set_dma_r_callback(device_t &device, int channel, _Object object) { return downcast<interpro_ioga_device &>(device).m_dma_channel[channel].device_r.set_callback(object); }
	template<class _Object> static devcb_base &static_set_dma_w_callback(device_t &device, int channel, _Object object) { return downcast<interpro_ioga_device &>(device).m_dma_channel[channel].device_w.set_callback(object); }

	template<class _Object> static devcb_base &static_set_dma_serial_r_callback(device_t &device, int channel, _Object object) { return downcast<interpro_ioga_device &>(device).m_dma_serial_channel[channel].device_r.set_callback(object); }
	template<class _Object> static devcb_base &static_set_dma_serial_w_callback(device_t &device, int channel, _Object object) { return downcast<interpro_ioga_device &>(device).m_dma_serial_channel[channel].device_w.set_callback(object); }

	template<class _Object> static devcb_base &static_set_fdc_tc_callback(device_t &device, _Object object) { return downcast<interpro_ioga_device &>(device).m_fdc_tc_func.set_callback(object); }
	template<class _Object> static devcb_base &static_set_eth_ca_callback(device_t &device, _Object object) { return downcast<interpro_ioga_device &>(device).m_eth_ca_func.set_callback(object); }

	virtual DECLARE_ADDRESS_MAP(map, 32);

	// external interrupt lines
	DECLARE_WRITE_LINE_MEMBER(ir0_w) { set_irq_line(2, state); }
	DECLARE_WRITE_LINE_MEMBER(ir1_w) { set_irq_line(3, state); }
	DECLARE_WRITE_LINE_MEMBER(ir2_w) { set_irq_line(4, state); }
	DECLARE_WRITE_LINE_MEMBER(ir3_w) { set_irq_line(5, state); }
	DECLARE_WRITE_LINE_MEMBER(ir4_w) { set_irq_line(6, state); }
	DECLARE_WRITE_LINE_MEMBER(ir5_w) { set_irq_line(7, state); }
	DECLARE_WRITE_LINE_MEMBER(ir6_w) { set_irq_line(8, state); }
	DECLARE_WRITE_LINE_MEMBER(ir7_w) { set_irq_line(9, state); }
	DECLARE_WRITE_LINE_MEMBER(ir8_w) { set_irq_line(10, state); }
	// FIXME: this is a workaround for the mc146818 code which inverts the normal irq state convention
	DECLARE_WRITE_LINE_MEMBER(ir9_w) { set_irq_line(11, !state); }
	DECLARE_WRITE_LINE_MEMBER(ir10_w) { set_irq_line(12, state); }
	DECLARE_WRITE_LINE_MEMBER(ir11_w) { set_irq_line(17, state); }
	DECLARE_WRITE_LINE_MEMBER(ir12_w) { set_irq_line(18, state); }

	IRQ_CALLBACK_MEMBER(inta_cb);

	DECLARE_WRITE_LINE_MEMBER(drq_plotter) { drq(state, IOGA_DMA_PLOTTER); }
	DECLARE_WRITE_LINE_MEMBER(drq_scsi) { drq(state, IOGA_DMA_SCSI); }
	DECLARE_WRITE_LINE_MEMBER(drq_floppy) { drq(state, IOGA_DMA_FLOPPY); }

	DECLARE_WRITE_LINE_MEMBER(drq_serial0) { drq_serial(state, IOGA_DMA_SERIAL0); }
	DECLARE_WRITE_LINE_MEMBER(drq_serial1) { drq_serial(state, IOGA_DMA_SERIAL1); }
	DECLARE_WRITE_LINE_MEMBER(drq_serial2) { drq_serial(state, IOGA_DMA_SERIAL2); }

	enum eth_remap_mask
	{
		ETH_REMAP_CHA_FLUSH = 0x00000008,
		ETH_REMAP_CHA_BUF   = 0x00000010,
		ETH_REMAP_CHA_QUAD  = 0x00000020,
		ETH_REMAP_CHA_WTAG  = 0x000001c0,
		ETH_REMAP_CHA_RTAG  = 0x00000e00,
		ETH_REMAP_ADDR      = 0xfffff000
	};
	DECLARE_READ32_MEMBER(eth_remap_r) { return m_eth_remap; }
	DECLARE_WRITE32_MEMBER(eth_remap_w);

	enum eth_remap_page_mask
	{
		ETH_REMAP_CHB_FLUSH = 0x00000008,
		ETH_REMAP_CHB_BUF   = 0x00000010,
		ETH_REMAP_CHB_QUAD  = 0x00000020,
		ETH_REMAP_CHB_WTAG  = 0x000001c0,
		ETH_REMAP_CHB_RTAG  = 0x00000e00,
		ETH_MAP_PAGE        = 0xfffff000
	};
	DECLARE_READ32_MEMBER(eth_map_page_r) { return m_eth_map_page; }
	DECLARE_WRITE32_MEMBER(eth_map_page_w);

	enum eth_control_mask
	{
		ETH_CA             = 0x00000001, // channel attention
		ETH_MAPEN          = 0x00000002,
		ETH_REMAP_CHC_BUF  = 0x00000010,
		ETH_REMAP_CHC_QUAD = 0x00000020,
		ETH_REMAP_CHC_WTAG = 0x000001c0,
		ETH_REMAP_CHC_RTAG = 0x00000e00,
		ETH_BERR           = 0x00001000,
		ETH_MMBE           = 0x00002000,
		ETH_RESET          = 0x00004000
	};
	DECLARE_READ32_MEMBER(eth_control_r) { return m_eth_control; }
	DECLARE_WRITE32_MEMBER(eth_control_w);

	DECLARE_READ32_MEMBER(timer_prescaler_r) { return m_prescaler; }
	DECLARE_READ32_MEMBER(timer0_r) { return m_timer_reg[0]; }
	DECLARE_READ32_MEMBER(timer1_r);

	enum arbctl_mask
	{
		ARBCTL_BGR_ETHC = 0x0001,
		ARBCTL_BGR_SCSI = 0x0002,
		ARBCTL_BGR_PLOT = 0x0004,
		ARBCTL_BGR_FDC  = 0x0008,
		ARBCTL_BGR_SER0 = 0x0010,
		ARBCTL_BGR_SER1 = 0x0020,
		ARBCTL_BGR_SER2 = 0x0040,
		ARBCTL_BGR_ETHB = 0x0080,
		ARBCTL_BGR_ETHA = 0x0100
	};
	DECLARE_READ16_MEMBER(arbctl_r) { return m_arbctl; }
	DECLARE_WRITE16_MEMBER(arbctl_w);
	DECLARE_READ32_MEMBER(timer2_r) { return m_timer_reg[2]; }
	DECLARE_READ32_MEMBER(timer3_r);

	DECLARE_WRITE32_MEMBER(timer_prescaler_w) {
		// this logic satisfies prescaler tests, but fails timer prescaler tests
		if ((data & 0x7fff) < 0x100 && (data & 0x7fff) != 0)
			m_prescaler = (data ^ 0xffff0000);
		else
			m_prescaler = (data ^ 0xffff0000) - 0x10000;

		//logerror("prescaler: input 0x%08x output 0x%08x\n", data, m_prescaler);
	}
	DECLARE_WRITE32_MEMBER(timer0_w) { write_timer(0, data, IOGA_TIMER_0); }
	DECLARE_WRITE32_MEMBER(timer1_w) { write_timer(1, data, IOGA_TIMER_1); }
	DECLARE_WRITE32_MEMBER(timer2_w) { write_timer(2, data, IOGA_TIMER_2); }
	DECLARE_WRITE32_MEMBER(timer3_w) { write_timer(3, data, IOGA_TIMER_3); }

	enum hwicr_mask
	{
		IRQ_PENDING         = 0x0100,
		IRQ_ENABLE_EXTERNAL = 0x0200,
		IRQ_EDGE            = 0x0400,
		IRQ_NEGPOL          = 0x0800,
		IRQ_ENABLE_INTERNAL = 0x1000
	};
	DECLARE_READ16_MEMBER(icr_r) { return m_hwicr[offset]; }
	DECLARE_WRITE16_MEMBER(icr_w);
	DECLARE_READ16_MEMBER(icr18_r) { return icr_r(space, 18, mem_mask); }
	DECLARE_WRITE16_MEMBER(icr18_w) { icr_w(space, 18, data, mem_mask); }

	DECLARE_READ8_MEMBER(softint_r) { return m_softint; }
	DECLARE_WRITE8_MEMBER(softint_w);

	enum nmictrl_mask
	{
		NMI_ALL     = 0x01,
		NMI_ENABLE1 = 0x02,
		NMI_EDGE    = 0x04,
		NMI_NEGPOL  = 0x08,
		NMI_ENABLE2 = 0x10,

		NMI_IE      = NMI_ENABLE1 | NMI_ENABLE2
	};
	DECLARE_READ8_MEMBER(nmictrl_r) { return m_nmictrl; }
	DECLARE_WRITE8_MEMBER(nmictrl_w);

	DECLARE_READ16_MEMBER(softint_vector_r) { return m_swicr[offset]; }
	DECLARE_WRITE16_MEMBER(softint_vector_w);

	enum dma_ctrl_mask
	{
		DMA_CTRL_TCZERO  = 0x00000001, // transfer count zero
		DMA_CTRL_TAG     = 0x00000e00, // bus tag
		DMA_CTRL_BERR    = 0x00400000, // bus error
		DMA_CTRL_ERR     = 0x00800000, // checked for in scsi isr

		DMA_CTRL_BGR     = 0x01000000, // cleared when command complete (maybe bus grant required?)
		DMA_CTRL_WAIT    = 0x02000000, // waiting for bus grant
		DMA_CTRL_X       = 0x04000000, // set during fdc dma?

		DMA_CTRL_VIRTUAL = 0x20000000, // use virtual addressing
		DMA_CTRL_WRITE   = 0x40000000, // memory to device transfer

		DMA_CTRL_WMASK   = 0xfd000e00  // writable fields
	};
	DECLARE_READ32_MEMBER(dma_plotter_r) { return dma_r(space, offset, mem_mask, IOGA_DMA_PLOTTER); }
	DECLARE_WRITE32_MEMBER(dma_plotter_w) { dma_w(space, offset, data, mem_mask, IOGA_DMA_PLOTTER); }
	DECLARE_READ32_MEMBER(dma_scsi_r) { return dma_r(space, offset, mem_mask, IOGA_DMA_SCSI); }
	DECLARE_WRITE32_MEMBER(dma_scsi_w) { dma_w(space, offset, data, mem_mask, IOGA_DMA_SCSI); }
	DECLARE_READ32_MEMBER(dma_floppy_r) { return dma_r(space, offset, mem_mask, IOGA_DMA_FLOPPY); }
	DECLARE_WRITE32_MEMBER(dma_floppy_w) { dma_w(space, offset, data, mem_mask, IOGA_DMA_FLOPPY); }

	DECLARE_READ32_MEMBER(dma_serial0_addr_r) { return dma_serial_addr_r(space, offset, mem_mask, IOGA_DMA_SERIAL0); }
	DECLARE_WRITE32_MEMBER(dma_serial0_addr_w) { dma_serial_addr_w(space, offset, data, mem_mask, IOGA_DMA_SERIAL0); }
	DECLARE_READ16_MEMBER(dma_serial0_count_r) { return dma_serial_count_r(space, offset, mem_mask, IOGA_DMA_SERIAL0); }
	DECLARE_WRITE16_MEMBER(dma_serial0_count_w) { dma_serial_count_w(space, offset, data, mem_mask, IOGA_DMA_SERIAL0); }
	DECLARE_READ16_MEMBER(dma_serial0_ctrl_r) { return dma_serial_ctrl_r(space, offset, mem_mask, IOGA_DMA_SERIAL0); }
	DECLARE_WRITE16_MEMBER(dma_serial0_ctrl_w) { dma_serial_ctrl_w(space, offset, data, mem_mask, IOGA_DMA_SERIAL0); }

	DECLARE_READ32_MEMBER(dma_serial1_addr_r) { return dma_serial_addr_r(space, offset, mem_mask, IOGA_DMA_SERIAL1); }
	DECLARE_WRITE32_MEMBER(dma_serial1_addr_w) { dma_serial_addr_w(space, offset, data, mem_mask, IOGA_DMA_SERIAL1); }
	DECLARE_READ16_MEMBER(dma_serial1_count_r) { return dma_serial_count_r(space, offset, mem_mask, IOGA_DMA_SERIAL1); }
	DECLARE_WRITE16_MEMBER(dma_serial1_count_w) { dma_serial_count_w(space, offset, data, mem_mask, IOGA_DMA_SERIAL1); }
	DECLARE_READ16_MEMBER(dma_serial1_ctrl_r) { return dma_serial_ctrl_r(space, offset, mem_mask, IOGA_DMA_SERIAL1); }
	DECLARE_WRITE16_MEMBER(dma_serial1_ctrl_w) { dma_serial_ctrl_w(space, offset, data, mem_mask, IOGA_DMA_SERIAL1); }

	DECLARE_READ32_MEMBER(dma_serial2_addr_r) { return dma_serial_addr_r(space, offset, mem_mask, IOGA_DMA_SERIAL2); }
	DECLARE_WRITE32_MEMBER(dma_serial2_addr_w) { dma_serial_addr_w(space, offset, data, mem_mask, IOGA_DMA_SERIAL2); }
	DECLARE_READ16_MEMBER(dma_serial2_count_r) { return dma_serial_count_r(space, offset, mem_mask, IOGA_DMA_SERIAL2); }
	DECLARE_WRITE16_MEMBER(dma_serial2_count_w) { dma_serial_count_w(space, offset, data, mem_mask, IOGA_DMA_SERIAL2); }
	DECLARE_READ16_MEMBER(dma_serial2_ctrl_r) { return dma_serial_ctrl_r(space, offset, mem_mask, IOGA_DMA_SERIAL2); }
	DECLARE_WRITE16_MEMBER(dma_serial2_ctrl_w) { dma_serial_ctrl_w(space, offset, data, mem_mask, IOGA_DMA_SERIAL2); }

	DECLARE_READ32_MEMBER(dma_plotter_eosl_r) { return m_dma_plotter_eosl; }
	DECLARE_WRITE32_MEMBER(dma_plotter_eosl_w) { m_dma_plotter_eosl = data; }

	DECLARE_READ32_MEMBER(error_address_r) { return m_error_address; }

	enum error_businfo_mask
	{
		BINFO_CT     = 0x003f,
		BINFO_TAG    = 0x01c0,
		BINFO_BG     = 0x0e00,
		BINFO_BERR   = 0x1000,
		BINFO_MMBE   = 0x2000,
		BINFO_SNAPOK = 0x4000,
		BINFO_MSBE   = 0x8000
	};
	enum error_businfo_bg
	{
		BINFO_BG_IOD      = 0x0200,
		BINFO_BG_ICAMMU   = 0x0400,
		BINFO_BG_DCAMMU   = 0x0800,
		BINFO_BG_SRMASTER = 0x0c00
	};
	DECLARE_READ32_MEMBER(error_businfo_r);
	DECLARE_READ32_MEMBER(bus_timeout_r) { return m_bus_timeout; }
	DECLARE_WRITE32_MEMBER(bus_timeout_w) { m_bus_timeout = data; }

	DECLARE_WRITE32_MEMBER(bus_error) { m_error_address = data; m_error_businfo = offset; }

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	static const device_timer_id IOGA_TIMER_0 = 0;
	static const device_timer_id IOGA_TIMER_1 = 1;
	static const device_timer_id IOGA_TIMER_2 = 2;
	static const device_timer_id IOGA_TIMER_3 = 3;

	static const device_timer_id IOGA_CLOCK = 4;

	void set_nmi_line(int state);
	void set_irq_line(int irq, int state);
	void set_irq_soft(int irq, int state);
	void write_timer(int timer, u32 value, device_timer_id id);

	void interrupt_clock();
	void dma_clock();
	void dma_serial_clock();

	void drq(int state, int channel);
	void drq_serial(int state, int channel);
	devcb_write_line m_out_nmi_func;
	devcb_write_line m_out_irq_func;
	address_space *m_memory_space;

	enum dma_states
	{
		IDLE,
		WAIT,
		COMMAND,
		TRANSFER,
		COMPLETE,
		FINAL
	};
	// dma channels
	struct dma
	{
		u32 real_address;
		u32 virtual_address;
		u32 transfer_count;
		u32 control;

		int drq_state;
		dma_states state;
		devcb_read8 device_r;
		devcb_write8 device_w;

		const u16 arb_mask;
		const int channel;
		const char *name;
	} m_dma_channel[IOGA_DMA_CHANNELS];
	u32 m_dma_plotter_eosl;

	// dma serial channels
	struct dma_serial
	{
		u32 address;
		u16 count;
		u16 control;

		int drq_state;
		dma_states state;
		devcb_read8 device_r;
		devcb_write8 device_w;

		const u16 arb_mask;
		const int channel;
		const char *name;
	} m_dma_serial_channel[IOGA_DMA_SERIAL_CHANNELS];

	devcb_write_line m_fdc_tc_func;
	devcb_write_line m_eth_ca_func;

	u32 m_active_interrupt_type;
	u32 m_active_interrupt_number;

	u32 m_hwint_forced;
	bool m_nmi_pending;

	u16 m_hwicr[IOGA_INTERRUPT_COUNT];
	u8 m_softint;
	u8 m_nmictrl;
	u16 m_swicr[8];

	u32 m_prescaler;
	u32 m_timer_reg[3];
	u16 m_timer1_count;
	u32 m_timer3_count;
	emu_timer *m_timer[4];

	u32 dma_r(address_space &space, offs_t offset, u32 mem_mask, int channel);
	void dma_w(address_space &space, offs_t offset, u32 data, u32 mem_mask, int channel);

	u32 dma_serial_addr_r(address_space &space, offs_t offset, u32 mem_mask, int channel) { return m_dma_serial_channel[channel].address; }
	void dma_serial_addr_w(address_space &space, offs_t offset, u32 data, u32 mem_mask, int channel);
	u16 dma_serial_count_r(address_space &space, offs_t offset, u16 mem_mask, int channel) { return m_dma_serial_channel[channel].count; }
	void dma_serial_count_w(address_space &space, offs_t offset, u16 data, u16 mem_mask, int channel);
	u16 dma_serial_ctrl_r(address_space &space, offs_t offset, u16 mem_mask, int channel) { return m_dma_serial_channel[channel].control; }
	void dma_serial_ctrl_w(address_space &space, offs_t offset, u16 data, u16 mem_mask, int channel);

	u16 m_arbctl;

	u32 m_error_address;
	u32 m_error_businfo;
	u32 m_bus_timeout;

	emu_timer *m_ioga_clock;
	int m_nmi_state;
	int m_irq_state;

	u32 m_int_line;

	bool nmi(int state);
	bool irq(int state);

	u32 m_eth_remap;
	u32 m_eth_map_page;
	u32 m_eth_control;
};

// device type definition
DECLARE_DEVICE_TYPE(INTERPRO_IOGA, interpro_ioga_device)

#endif // MAME_MACHINE_INTERPRO_IOGA_H
