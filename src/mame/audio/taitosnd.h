// license:BSD-3-Clause
// copyright-holders:Philip Bennett
#ifndef MAME_AUDIO_TAITOSND_H
#define MAME_AUDIO_TAITOSND_H


//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_TC0140SYT_MASTER_CPU(_tag) \
	tc0140syt_device::set_master_tag(*device, "^" _tag);

#define MCFG_TC0140SYT_SLAVE_CPU(_tag) \
	tc0140syt_device::set_slave_tag(*device, "^" _tag);


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> tc0140syt_device

class tc0140syt_device : public device_t
{
public:
	tc0140syt_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void set_master_tag(device_t &device, const char *tag) { downcast<tc0140syt_device &>(device).m_mastercpu.set_tag(tag); }
	static void set_slave_tag(device_t &device, const char *tag)  { downcast<tc0140syt_device &>(device).m_slavecpu.set_tag(tag); }

	// MASTER (4-bit bus) control functions
	DECLARE_WRITE8_MEMBER( master_port_w );
	DECLARE_WRITE8_MEMBER( master_comm_w );
	DECLARE_READ8_MEMBER( master_comm_r );

	// SLAVE (4-bit bus) control functions ONLY
	DECLARE_WRITE8_MEMBER( slave_port_w );
	DECLARE_READ8_MEMBER( slave_comm_r );
	DECLARE_WRITE8_MEMBER( slave_comm_w );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	void update_nmi();

	uint8_t     m_slavedata[4];  /* Data on master->slave port (4 nibbles) */
	uint8_t     m_masterdata[4]; /* Data on slave->master port (4 nibbles) */
	uint8_t     m_mainmode;      /* Access mode on master cpu side */
	uint8_t     m_submode;       /* Access mode on slave cpu side */
	uint8_t     m_status;        /* Status data */
	uint8_t     m_nmi_enabled;   /* 1 if slave cpu has nmi's enabled */

	required_device<cpu_device> m_mastercpu;     /* this is the maincpu */
	required_device<cpu_device> m_slavecpu;      /* this is the audiocpu */
};

DECLARE_DEVICE_TYPE(TC0140SYT, tc0140syt_device)


#endif // MAME_AUDIO_TAITOSND_H
