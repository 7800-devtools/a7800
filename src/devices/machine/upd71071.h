// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
#ifndef MAME_MACHINE_UPD71071_H
#define MAME_MACHINE_UPD71071_H

#pragma once


class upd71071_device : public device_t
{
public:
	upd71071_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void static_set_cpu_tag(device_t &device, const char *tag) { downcast<upd71071_device &>(device).m_cpu.set_tag(tag); }
	static void set_clock(device_t &device, int clock) { downcast<upd71071_device &>(device).m_upd_clock = clock; }

	template <class Object> static devcb_base &set_out_hreq_callback(device_t &device, Object &&cb) { return downcast<upd71071_device &>(device).m_out_hreq_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_eop_callback(device_t &device, Object &&cb) { return downcast<upd71071_device &>(device).m_out_eop_cb.set_callback(std::forward<Object>(cb)); }

	template <unsigned N, class Object> static devcb_base &set_dma_read_callback(device_t &device, Object &&cb) { return downcast<upd71071_device &>(device).m_dma_read_cb[N].set_callback(std::forward<Object>(cb)); }
	template <unsigned N, class Object> static devcb_base &set_dma_write_callback(device_t &device, Object &&cb) { return downcast<upd71071_device &>(device).m_dma_write_cb[N].set_callback(std::forward<Object>(cb)); }
	template <unsigned N, class Object> static devcb_base &set_out_dack_callback(device_t &device, Object &&cb) { return downcast<upd71071_device &>(device).m_out_dack_cb[N].set_callback(std::forward<Object>(cb)); }

	DECLARE_READ8_MEMBER(read);
	DECLARE_WRITE8_MEMBER(write);
	DECLARE_WRITE_LINE_MEMBER(set_hreq);
	DECLARE_WRITE_LINE_MEMBER(set_eop);

	int dmarq(int state, int channel);

protected:
	// device-level overrides
	virtual void device_start() override;

private:
	struct upd71071_reg
	{
		uint8_t initialise;
		uint8_t channel;
		uint16_t count_current[4];
		uint16_t count_base[4];
		uint32_t address_current[4];
		uint32_t address_base[4];
		uint16_t device_control;
		uint8_t mode_control[4];
		uint8_t status;
		uint8_t temp_l;
		uint8_t temp_h;
		uint8_t request;
		uint8_t mask;
	};

	void soft_reset();
	TIMER_CALLBACK_MEMBER(dma_transfer_timer);

	// internal state
	upd71071_reg m_reg;
	int m_selected_channel;
	int m_buswidth;
	int m_dmarq[4];
	emu_timer* m_timer[4];
	//int m_in_progress[4];
	//int m_transfer_size[4];
	int m_base;
	int m_upd_clock;
	devcb_write_line    m_out_hreq_cb;
	devcb_write_line    m_out_eop_cb;
	devcb_read16        m_dma_read_cb[4];
	devcb_write16       m_dma_write_cb[4];
	devcb_write_line    m_out_dack_cb[4];
	int m_hreq;
	int m_eop;
	optional_device<cpu_device> m_cpu;
};

DECLARE_DEVICE_TYPE(UPD71071, upd71071_device)

#define MCFG_UPD71071_CPU(tag) \
		upd71071_device::static_set_cpu_tag(*device, ("^" tag));

#define MCFG_UPD71071_CLOCK(clk) \
	upd71071_device::set_clock(*device, (clk));

#define MCFG_UPD71071_OUT_HREQ_CB(cb) \
		devcb = &upd71071_device::set_out_hreq_callback(*device, (DEVCB_##cb));

#define MCFG_UPD71071_OUT_EOP_CB(cb) \
		devcb = &upd71071_device::set_out_eop_callback(*device, (DEVCB_##cb));

#define MCFG_UPD71071_DMA_READ_0_CB(cb) \
		devcb = &upd71071_device::set_dma_read_callback<0>(*device, (DEVCB_##cb));

#define MCFG_UPD71071_DMA_READ_1_CB(cb) \
		devcb = &upd71071_device::set_dma_read_callback<1>(*device, (DEVCB_##cb));

#define MCFG_UPD71071_DMA_READ_2_CB(cb) \
		devcb = &upd71071_device::set_dma_read_callback<2>(*device, (DEVCB_##cb));

#define MCFG_UPD71071_DMA_READ_3_CB(cb) \
		devcb = &upd71071_device::set_dma_read_callback<3>(*device, (DEVCB_##cb));

#define MCFG_UPD71071_DMA_WRITE_0_CB(cb) \
		devcb = &upd71071_device::set_dma_write_callback<0>(*device, (DEVCB_##cb));

#define MCFG_UPD71071_DMA_WRITE_1_CB(cb) \
		devcb = &upd71071_device::set_dma_write_callback<1>(*device, (DEVCB_##cb));

#define MCFG_UPD71071_DMA_WRITE_2_CB(cb) \
		devcb = &upd71071_device::set_dma_write_callback<2>(*device, (DEVCB_##cb));

#define MCFG_UPD71071_DMA_WRITE_3_CB(cb) \
		devcb = &upd71071_device::set_dma_write_callback<3>(*device, (DEVCB_##cb));

#define MCFG_UPD71071_OUT_DACK_0_CB(cb) \
		devcb = &upd71071_device::set_out_dack_callback<0>(*device, (DEVCB_##cb));

#define MCFG_UPD71071_OUT_DACK_1_CB(cb) \
		devcb = &upd71071_device::set_out_dack_callback<1>(*device, (DEVCB_##cb));

#define MCFG_UPD71071_OUT_DACK_2_CB(cb) \
		devcb = &upd71071_device::set_out_dack_callback<2>(*device, (DEVCB_##cb));

#define MCFG_UPD71071_OUT_DACK_3_CB(cb) \
		devcb = &upd71071_device::set_out_dack_callback<3>(*device, (DEVCB_##cb));

#endif // MAME_MACHINE_UPD71071_H
