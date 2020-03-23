// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
#ifndef MAME_MACHINE_NEXTKBD_H
#define MAME_MACHINE_NEXTKBD_H

#pragma once


#define MCFG_NEXTKBD_INT_CHANGE_CALLBACK(_write) \
	devcb = &nextkbd_device::set_int_change_wr_callback(*device, DEVCB_##_write);

#define MCFG_NEXTKBD_INT_POWER_CALLBACK(_write) \
	devcb = &nextkbd_device::set_int_power_wr_callback(*device, DEVCB_##_write);

#define MCFG_NEXTKBD_INT_NMI_CALLBACK(_write) \
	devcb = &nextkbd_device::set_int_nmi_wr_callback(*device, DEVCB_##_write);

class nextkbd_device : public device_t {
public:
	nextkbd_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_int_change_wr_callback(device_t &device, Object &&cb) { return downcast<nextkbd_device &>(device).int_change_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_int_power_wr_callback(device_t &device, Object &&cb) { return downcast<nextkbd_device &>(device).int_power_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_int_nmi_wr_callback(device_t &device, Object &&cb) { return downcast<nextkbd_device &>(device).int_nmi_cb.set_callback(std::forward<Object>(cb)); }

	DECLARE_ADDRESS_MAP(amap, 32);

	DECLARE_READ8_MEMBER(status_snd_r);
	DECLARE_READ8_MEMBER(status_kms_r);
	DECLARE_READ8_MEMBER(status_dma_r);
	DECLARE_READ8_MEMBER(status_cmd_r);
	DECLARE_READ32_MEMBER(cdata_r);
	DECLARE_READ32_MEMBER(kmdata_r);

	DECLARE_WRITE8_MEMBER(ctrl_snd_w);
	DECLARE_WRITE8_MEMBER(ctrl_kms_w);
	DECLARE_WRITE8_MEMBER(ctrl_dma_w);
	DECLARE_WRITE8_MEMBER(ctrl_cmd_w);
	DECLARE_WRITE32_MEMBER(cdata_w);
	DECLARE_WRITE32_MEMBER(kmdata_w);

	DECLARE_INPUT_CHANGED_MEMBER(update);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	virtual ioport_constructor device_input_ports() const override;

private:
	// Big thanks to the previous emulator for that information
	enum {
		C_SOUND_OUT_ENABLE    = 0x80, // rw
		C_SOUND_OUT_REQUEST   = 0x40, // r
		C_SOUND_OUT_UNDERRUN  = 0x20, // rw
		C_SOUND_IN_ENABLE     = 0x08, // rw
		C_SOUND_IN_REQUEST    = 0x04, // r
		C_SOUND_IN_OVERRUN    = 0x02, // rw
		C_SOUND_WMASK         = 0xaa,

		C_KBD_INTERRUPT       = 0x80, // r
		C_KBD_DATA            = 0x40, // r
		C_KBD_OVERRUN         = 0x20, // rw
		C_KBD_NMI             = 0x10, // rw?
		C_KMS_INTERRUPT       = 0x08, // r
		C_KMS_DATA            = 0x04, // r
		C_KMS_OVERRUN         = 0x02, // rw
		C_KMS_WMASK           = 0x22,

		C_SDMA_OUT_PENDING    = 0x80, // r
		C_SDMA_OUT_RUNNING    = 0x40, // r
		C_CDMA_PENDING        = 0x20, // r
		C_CDMA_RUNNING        = 0x10, // r
		C_KMS_ENABLE          = 0x02, // rw
		C_LOOPBACK            = 0x01, // rw
		C_WMASK               = 0x03,

		D_ERROR               = 0x40000000,
		D_USER                = 0x20000000,
		D_MASTER              = 0x10000000,
		D_SECONDARY           = 0x01000000,
		D_VERSION_OLD         = 0x00000000,
		D_VERSION_NEW         = 0x00010000,
		D_VERSION_DIGITAL     = 0x00020000,

		D_KBD_VALID           = 0x00008000,
		D_KBD_KEYDOWN         = 0x00000080
	};

	enum { FIFO_SIZE = 32 };

	devcb_write_line int_change_cb, int_power_cb, int_nmi_cb;
	required_ioport mousex;
	required_ioport mousey;
	required_ioport mousebtn;

	emu_timer *poll_timer;
	bool nmi_active;

	uint32_t cdata, kmdata, fifo_ir, fifo_iw, fifo_size;
	uint32_t fifo[FIFO_SIZE];
	uint32_t km_address;
	uint32_t prev_mousex, prev_mousey, prev_mousebtn;
	uint16_t modifiers_state;
	uint8_t ctrl_snd, ctrl_kms, ctrl_dma, ctrl_cmd;

	void fifo_push(uint32_t val);
	uint32_t fifo_pop();
	bool fifo_empty() const;

	void update_mouse(bool force_update);
	void send();
	void handle_fifo_command();
	void handle_kbd_command();
	void handle_command();
};

DECLARE_DEVICE_TYPE(NEXTKBD, nextkbd_device)

#endif // MAME_MACHINE_NEXTKBD_H
