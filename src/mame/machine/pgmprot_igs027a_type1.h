// license:BSD-3-Clause
// copyright-holders:David Haywood, ElSemi, Xing Xing
#include "includes/pgm.h"

class pgm_arm_type1_state : public pgm_state
{
public:
	pgm_arm_type1_state(const machine_config &mconfig, device_type type, const char *tag)
		: pgm_state(mconfig, type, tag)
		, m_arm7_shareram(*this, "arm7_shareram")
		, m_prot(*this, "prot")
	{
		m_curslots = 0;
		m_puzzli_54_trigger = 0;
	}

	/////////////// simulations
	uint16_t m_value0;
	uint16_t m_value1;
	uint16_t m_valuekey;
	uint16_t m_ddp3lastcommand;
	uint32_t m_valueresponse;
	int m_curslots;
	uint32_t m_slots[0x100];

	// pstars / oldsplus / kov
	uint16_t m_pstar_e7_value;
	uint16_t m_pstar_b1_value;
	uint16_t m_pstar_ce_value;
	uint16_t m_kov_c0_value;
	uint16_t m_kov_cb_value;
	uint16_t m_kov_fe_value;
	uint16_t m_extra_ram[0x100];
	// puzzli2
	int32_t m_puzzli_54_trigger;

	typedef void (pgm_arm_type1_state::*pgm_arm_sim_command_handler)(int pc);

	pgm_arm_sim_command_handler arm_sim_handler;

	/////////////// emulation
	uint16_t        m_pgm_arm_type1_highlatch_arm_w;
	uint16_t        m_pgm_arm_type1_lowlatch_arm_w;
	uint16_t        m_pgm_arm_type1_highlatch_68k_w;
	uint16_t        m_pgm_arm_type1_lowlatch_68k_w;
	uint32_t        m_pgm_arm_type1_counter;
	optional_shared_ptr<uint32_t> m_arm7_shareram;

	optional_device<cpu_device> m_prot;

	DECLARE_DRIVER_INIT(photoy2k);
	DECLARE_DRIVER_INIT(kovsh);
	DECLARE_DRIVER_INIT(kovshp);
	DECLARE_DRIVER_INIT(kovshxas);
	DECLARE_DRIVER_INIT(kovlsqh2);
	DECLARE_DRIVER_INIT(kovqhsgs);
	DECLARE_DRIVER_INIT(ddp3);
	DECLARE_DRIVER_INIT(ket);
	DECLARE_DRIVER_INIT(espgal);
	DECLARE_DRIVER_INIT(puzzli2);
	DECLARE_DRIVER_INIT(py2k2);
	DECLARE_DRIVER_INIT(pgm3in1);
	DECLARE_DRIVER_INIT(pstar);
	DECLARE_DRIVER_INIT(kov);
	DECLARE_DRIVER_INIT(kovboot);
	DECLARE_DRIVER_INIT(oldsplus);
	DECLARE_MACHINE_START(pgm_arm_type1);

	DECLARE_READ32_MEMBER( pgm_arm7_type1_protlatch_r );
	DECLARE_WRITE32_MEMBER( pgm_arm7_type1_protlatch_w );
	DECLARE_READ16_MEMBER( pgm_arm7_type1_68k_protlatch_r );
	DECLARE_WRITE16_MEMBER( pgm_arm7_type1_68k_protlatch_w );
	DECLARE_READ16_MEMBER( pgm_arm7_type1_ram_r );
	DECLARE_WRITE16_MEMBER( pgm_arm7_type1_ram_w );
	DECLARE_READ32_MEMBER( pgm_arm7_type1_unk_r );
	DECLARE_READ32_MEMBER( pgm_arm7_type1_exrom_r );
	DECLARE_READ32_MEMBER( pgm_arm7_type1_shareram_r );
	DECLARE_WRITE32_MEMBER( pgm_arm7_type1_shareram_w );
	void pgm_arm7_type1_latch_init();
	DECLARE_READ16_MEMBER( kovsh_fake_region_r );
	DECLARE_WRITE16_MEMBER( kovshp_asic27a_write_word );
	void pgm_decode_kovlsqh2_tiles();
	void pgm_decode_kovlsqh2_sprites(uint8_t *src );
	void pgm_decode_kovlsqh2_samples();
	void pgm_decode_kovqhsgs_program();
	void pgm_decode_kovqhsgs2_program();
	DECLARE_READ16_MEMBER( pgm_arm7_type1_sim_r );
	void command_handler_ddp3(int pc);
	void command_handler_puzzli2(int pc);
	void command_handler_py2k2(int pc);
	void command_handler_pstars(int pc);
	void command_handler_kov(int pc);
	void command_handler_oldsplus(int pc);
	DECLARE_WRITE16_MEMBER( pgm_arm7_type1_sim_w );
	DECLARE_READ16_MEMBER( pgm_arm7_type1_sim_protram_r );
	DECLARE_READ16_MEMBER( pstars_arm7_type1_sim_protram_r );
	int m_simregion;

	/* puzzli2 protection internal state stuff */
	int stage;
	int tableoffs;
	int tableoffs2;
	int entries_left;
	int currentcolumn;
	int currentrow;
	int num_entries;
	int full_entry;
	int prev_tablloc;
	int numbercolumns;
	int depth;
	uint16_t m_row_bitmask;
	int hackcount;
	int hackcount2;
	int hack_47_value;
	int hack_31_table_offset;
	int hack_31_table_offset2;
	int p2_31_retcounter;

	uint8_t coverage[256]; // coverage is how much of the table we've managed to verify using known facts about the table structure

	int command_31_write_type;


	// the maximum level size returned or read by the device appears to be this size
	uint16_t level_structure[8][10];


	int puzzli2_take_leveldata_value(uint8_t datvalue);
};

MACHINE_CONFIG_EXTERN( pgm_arm_type1 );
MACHINE_CONFIG_EXTERN( pgm_arm_type1_sim );
MACHINE_CONFIG_EXTERN( pgm_arm_type1_cave );

INPUT_PORTS_EXTERN( sango );
INPUT_PORTS_EXTERN( sango_ch );
INPUT_PORTS_EXTERN( photoy2k );
INPUT_PORTS_EXTERN( oldsplus );
INPUT_PORTS_EXTERN( pstar );
INPUT_PORTS_EXTERN( py2k2 );
INPUT_PORTS_EXTERN( puzzli2 );
INPUT_PORTS_EXTERN( kovsh );
