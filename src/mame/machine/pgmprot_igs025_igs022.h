// license:BSD-3-Clause
// copyright-holders:David Haywood, ElSemi

class pgm_022_025_state : public pgm_state
{
public:
	pgm_022_025_state(const machine_config &mconfig, device_type type, const char *tag)
		: pgm_state(mconfig, type, tag),
			m_sharedprotram(*this, "sharedprotram"),
			m_igs025(*this,"igs025"),
			m_igs022(*this,"igs022")

	{ }

	void pgm_dw3_decrypt();
	void pgm_killbld_decrypt();

	required_shared_ptr<uint16_t> m_sharedprotram;

	DECLARE_DRIVER_INIT(killbld);
	DECLARE_DRIVER_INIT(drgw3);
	DECLARE_MACHINE_RESET(killbld);
	DECLARE_MACHINE_RESET(dw3);

	void igs025_to_igs022_callback( void );

	required_device<igs025_device> m_igs025;
	required_device<igs022_device> m_igs022;
};

MACHINE_CONFIG_EXTERN(pgm_022_025_dw3);
MACHINE_CONFIG_EXTERN(pgm_022_025_killbld);

INPUT_PORTS_EXTERN( killbld );
INPUT_PORTS_EXTERN( dw3 );
INPUT_PORTS_EXTERN( dw3j );
