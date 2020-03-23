// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria
#ifndef MAME_MACHINE_NAMCOIO_H
#define MAME_MACHINE_NAMCOIO_H


/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

class namcoio_device : public device_t
{
public:
	template <unsigned N, class Object> static devcb_base &set_in_callback(device_t &device, Object &&cb) { return downcast<namcoio_device &>(device).m_in_cb[N].set_callback(std::forward<Object>(cb)); }
	template <unsigned N, class Object> static devcb_base &set_out_callback(device_t &device, Object &&cb) { return downcast<namcoio_device &>(device).m_out_cb[N].set_callback(std::forward<Object>(cb)); }


	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );

	WRITE_LINE_MEMBER( set_reset_line );
	READ_LINE_MEMBER( read_reset_line );

	virtual void customio_run() = 0;

protected:
	namcoio_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, int device_type);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	enum {
			TYPE_NAMCO56XX,
			TYPE_NAMCO58XX,
			TYPE_NAMCO59XX
	};

	// internal state
	uint8_t          m_ram[16];

	devcb_read8 m_in_cb[4];
	devcb_write8 m_out_cb[2];

	int            m_reset;
	int32_t          m_lastcoins, m_lastbuttons;
	int32_t          m_credits;
	int32_t          m_coins[2];
	int32_t          m_coins_per_cred[2];
	int32_t          m_creds_per_coin[2];
	int32_t          m_in_count;

	void handle_coins( int swap );

private:
	//int m_device_type;
};

class namco56xx_device : public namcoio_device
{
public:
	namco56xx_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void customio_run() override;
};

class namco58xx_device : public namcoio_device
{
public:
	namco58xx_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void customio_run() override;
};

class namco59xx_device : public namcoio_device
{
public:
	namco59xx_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void customio_run() override;
};

DECLARE_DEVICE_TYPE(NAMCO_56XX, namco56xx_device)
DECLARE_DEVICE_TYPE(NAMCO_58XX, namco58xx_device)
DECLARE_DEVICE_TYPE(NAMCO_59XX, namco59xx_device)

/***************************************************************************
    DEVICE CONFIGURATION MACROS
***************************************************************************/

#define MCFG_NAMCO56XX_IN_0_CB(_devcb) \
	devcb = &namco56xx_device::set_in_callback<0>(*device, DEVCB_##_devcb);

#define MCFG_NAMCO56XX_IN_1_CB(_devcb) \
	devcb = &namco56xx_device::set_in_callback<1>(*device, DEVCB_##_devcb);

#define MCFG_NAMCO56XX_IN_2_CB(_devcb) \
	devcb = &namco56xx_device::set_in_callback<2>(*device, DEVCB_##_devcb);

#define MCFG_NAMCO56XX_IN_3_CB(_devcb) \
	devcb = &namco56xx_device::set_in_callback<3>(*device, DEVCB_##_devcb);

#define MCFG_NAMCO56XX_OUT_0_CB(_devcb) \
	devcb = &namco56xx_device::set_out_callback<0>(*device, DEVCB_##_devcb);

#define MCFG_NAMCO56XX_OUT_1_CB(_devcb) \
	devcb = &namco56xx_device::set_out_callback<1>(*device, DEVCB_##_devcb);


#define MCFG_NAMCO58XX_IN_0_CB(_devcb) \
	devcb = &namco58xx_device::set_in_callback<0>(*device, DEVCB_##_devcb);

#define MCFG_NAMCO58XX_IN_1_CB(_devcb) \
	devcb = &namco58xx_device::set_in_callback<1>(*device, DEVCB_##_devcb);

#define MCFG_NAMCO58XX_IN_2_CB(_devcb) \
	devcb = &namco58xx_device::set_in_callback<2>(*device, DEVCB_##_devcb);

#define MCFG_NAMCO58XX_IN_3_CB(_devcb) \
	devcb = &namco58xx_device::set_in_callback<3>(*device, DEVCB_##_devcb);

#define MCFG_NAMCO58XX_OUT_0_CB(_devcb) \
	devcb = &namco58xx_device::set_out_callback<0>(*device, DEVCB_##_devcb);

#define MCFG_NAMCO58XX_OUT_1_CB(_devcb) \
	devcb = &namco58xx_device::set_out_callback<1>(*device, DEVCB_##_devcb);


#define MCFG_NAMCO59XX_IN_0_CB(_devcb) \
	devcb = &namco59xx_device::set_in_callback<0>(*device, DEVCB_##_devcb);

#define MCFG_NAMCO59XX_IN_1_CB(_devcb) \
	devcb = &namco59xx_device::set_in_callback<1>(*device, DEVCB_##_devcb);

#define MCFG_NAMCO59XX_IN_2_CB(_devcb) \
	devcb = &namco59xx_device::set_in_callback<2>(*device, DEVCB_##_devcb);

#define MCFG_NAMCO59XX_IN_3_CB(_devcb) \
	devcb = &namco59xx_device::set_in_callback<3>(*device, DEVCB_##_devcb);

#define MCFG_NAMCO59XX_OUT_0_CB(_devcb) \
	devcb = &namco59xx_device::set_out_callback<0>(*device, DEVCB_##_devcb);

#define MCFG_NAMCO59XX_OUT_1_CB(_devcb) \
	devcb = &namco59xx_device::set_out_callback<1>(*device, DEVCB_##_devcb);

#endif // MAME_MACHINE_NAMCOIO_H
