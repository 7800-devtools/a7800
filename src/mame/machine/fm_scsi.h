// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
/*
 * fm_scsi.h
 *
 * SCSI controller used in Fujitsu FMR-50, FMR-60, and FM-Towns
 *
 */

#ifndef MAME_MACHINE_FM_SCSI_H
#define MAME_MACHINE_FM_SCSI_H

#include "machine/legscsi.h"

#define MCFG_FMSCSI_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, FMSCSI, 0)

#define MCFG_FMSCSI_IRQ_HANDLER(_devcb) \
	devcb = &fmscsi_device::set_irq_handler(*device, DEVCB_##_devcb);
#define MCFG_FMSCSI_DRQ_HANDLER(_devcb) \
	devcb = &fmscsi_device::set_drq_handler(*device, DEVCB_##_devcb);

class fmscsi_device : public legacy_scsi_host_adapter
{
public:
	// construction/destruction
	fmscsi_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration helpers
	template <class Object> static devcb_base &set_irq_handler(device_t &device, Object &&cb) { return downcast<fmscsi_device &>(device).m_irq_handler.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_drq_handler(device_t &device, Object &&cb) { return downcast<fmscsi_device &>(device).m_drq_handler.set_callback(std::forward<Object>(cb)); }

	// any publically accessible interfaces needed for runtime
	uint8_t fmscsi_data_r(void);
	void fmscsi_data_w(uint8_t data);
	uint8_t fmscsi_status_r(void);
	void fmscsi_control_w(uint8_t data);
	DECLARE_READ8_MEMBER( fmscsi_r );
	DECLARE_WRITE8_MEMBER( fmscsi_w );

	void set_phase(int phase);
	int get_phase(void);
	void set_input_line(uint8_t line, uint8_t state);
	uint8_t get_input_line(uint8_t line);
	void set_output_line(uint8_t line, uint8_t state);
	uint8_t get_output_line(uint8_t line);

protected:
	// device-level overrides (none are required, but these are common)
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	// internal device state goes here
	static const device_timer_id TIMER_TRANSFER = 0;
	static const device_timer_id TIMER_PHASE = 1;

	int get_scsi_cmd_len(uint8_t cbyte);
	void stop_transfer();

	devcb_write_line m_irq_handler;
	devcb_write_line m_drq_handler;

	uint8_t m_command[32];
	//uint8_t m_result[32];
	uint8_t m_command_index;
	int m_result_length;
	uint32_t m_result_index;
	uint8_t m_input_lines;
	uint8_t m_output_lines;
	uint8_t m_data;
	uint8_t m_last_id;
	uint8_t m_phase;
	uint8_t m_target;
	uint8_t m_buffer[512];
	emu_timer* m_transfer_timer;
	emu_timer* m_phase_timer;
};

DECLARE_DEVICE_TYPE(FMSCSI, fmscsi_device)

#endif // MAME_MACHINE_FM_SCSI_H
