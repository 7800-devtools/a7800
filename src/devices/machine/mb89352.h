// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
/*
 * mb89352.h
 *
 *  Created on: 16/01/2011
 */

#ifndef MAME_MACHINE_MB89352_H
#define MAME_MACHINE_MB89352_H

#include "legscsi.h"

#define MCFG_MB89352A_IRQ_CB(_devcb) \
	devcb = &mb89352_device::set_irq_callback(*device, DEVCB_##_devcb);

#define MCFG_MB89352A_DRQ_CB(_devcb) \
	devcb = &mb89352_device::set_drq_callback(*device, DEVCB_##_devcb);

class mb89352_device : public legacy_scsi_host_adapter
{
public:
	// construction/destruction
	mb89352_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_irq_callback(device_t &device, Object &&cb) { return downcast<mb89352_device &>(device).m_irq_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_drq_callback(device_t &device, Object &&cb) { return downcast<mb89352_device &>(device).m_drq_cb.set_callback(std::forward<Object>(cb)); }

	// any publically accessible interfaces needed for runtime
	DECLARE_READ8_MEMBER( mb89352_r );
	DECLARE_WRITE8_MEMBER( mb89352_w );

	void set_phase(int phase);

protected:
	// device-level overrides (none are required, but these are common)
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_stop() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	// internal device state goes here
	static const device_timer_id TIMER_TRANSFER = 0;

	int get_scsi_cmd_len(uint8_t cbyte);
	//void set_ints(uint8_t flag);

	devcb_write_line m_irq_cb;  /* irq callback */
	devcb_write_line m_drq_cb;  /* drq callback */

	uint8_t m_phase;  // current SCSI phase
	uint8_t m_target; // current SCSI target
	uint8_t m_bdid;  // Bus device ID (SCSI ID of the bus?)
	uint8_t m_ints;  // Interrupt Sense
	uint8_t m_temp;  // Temporary register (To/From SCSI bus)
	uint8_t m_data;  // Data register
	uint8_t m_scmd;  // SPC Command register
	uint32_t m_transfer_count;  // byte transfer counter, also used as a timeout counter for selection.
	uint8_t m_int_enable;
	uint8_t m_sel_enable;
	uint8_t m_resel_enable;
	uint8_t m_parity_enable;
	uint8_t m_arbit_enable;
	uint8_t m_busfree_int_enable;
	uint8_t m_line_status;
	uint8_t m_spc_status;
	uint8_t m_error_status;
	uint8_t m_command_index;
	uint8_t m_command[16];
	uint32_t m_transfer_index;
	uint8_t m_buffer[512];

	emu_timer* m_transfer_timer;
};

DECLARE_DEVICE_TYPE(MB89352A, mb89352_device)

#endif // MAME_MACHINE_MB89352_H
