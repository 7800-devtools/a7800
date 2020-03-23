// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
#ifndef MAME_MACHINE_PCI_SMBUS_H
#define MAME_MACHINE_PCI_SMBUS_H

#include "pci.h"

#define MCFG_SMBUS_ADD(_tag, _main_id, _revision, _subdevice_id) \
	MCFG_PCI_DEVICE_ADD(_tag, SMBUS, _main_id, _revision, 0x0c0500, _subdevice_id)

class smbus_device : public pci_device {
public:
	smbus_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER  (hst_sts_r);
	DECLARE_WRITE8_MEMBER (hst_sts_w);
	DECLARE_READ8_MEMBER  (hst_cnt_r);
	DECLARE_WRITE8_MEMBER (hst_cnt_w);
	DECLARE_READ8_MEMBER  (hst_cmd_r);
	DECLARE_WRITE8_MEMBER (hst_cmd_w);
	DECLARE_READ8_MEMBER  (xmit_slva_r);
	DECLARE_WRITE8_MEMBER (xmit_slva_w);
	DECLARE_READ8_MEMBER  (hst_d0_r);
	DECLARE_WRITE8_MEMBER (hst_d0_w);
	DECLARE_READ8_MEMBER  (hst_d1_r);
	DECLARE_WRITE8_MEMBER (hst_d1_w);
	DECLARE_READ8_MEMBER  (host_block_db_r);
	DECLARE_WRITE8_MEMBER (host_block_db_w);
	DECLARE_READ8_MEMBER  (pec_r);
	DECLARE_WRITE8_MEMBER (pec_w);
	DECLARE_READ8_MEMBER  (rcv_slva_r);
	DECLARE_WRITE8_MEMBER (rcv_slva_w);
	DECLARE_READ16_MEMBER (slv_data_r);
	DECLARE_WRITE16_MEMBER(slv_data_w);
	DECLARE_READ8_MEMBER  (aux_sts_r);
	DECLARE_WRITE8_MEMBER (aux_sts_w);
	DECLARE_READ8_MEMBER  (aux_ctl_r);
	DECLARE_WRITE8_MEMBER (aux_ctl_w);
	DECLARE_READ8_MEMBER  (smlink_pin_ctl_r);
	DECLARE_WRITE8_MEMBER (smlink_pin_ctl_w);
	DECLARE_READ8_MEMBER  (smbus_pin_ctl_r);
	DECLARE_WRITE8_MEMBER (smbus_pin_ctl_w);
	DECLARE_READ8_MEMBER  (slv_sts_r);
	DECLARE_WRITE8_MEMBER (slv_sts_w);
	DECLARE_READ8_MEMBER  (slv_cmd_r);
	DECLARE_WRITE8_MEMBER (slv_cmd_w);
	DECLARE_READ8_MEMBER  (notify_daddr_r);
	DECLARE_READ8_MEMBER  (notify_dlow_r);
	DECLARE_READ8_MEMBER  (notify_dhigh_r);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	DECLARE_ADDRESS_MAP(map, 32);

	uint16_t slv_data;

	uint8_t hst_sts, hst_cnt, hst_cmd, xmit_slva, hst_d0, hst_d1;
	uint8_t host_block_db, pec, rcv_slva, aux_sts, aux_ctl;
	uint8_t smlink_pin_ctl, smbus_pin_ctl, slv_sts, slv_cmd, notify_daddr, notify_dlow, notify_dhigh;
};

DECLARE_DEVICE_TYPE(SMBUS, smbus_device)

#endif // MAME_MACHINE_PCI_SMBUS_H
