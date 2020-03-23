// license:LGPL-2.1+
// copyright-holders:Olivier Galibert, Angelo Salese, David Haywood, Tomasz Slanina
#include "audio/seibu.h"
#include "machine/seibucop/seibucop.h"
#include "video/seibu_crtc.h"

ADDRESS_MAP_EXTERN(raiden2_sound_map, 8);
ADDRESS_MAP_EXTERN(zeroteam_sound_map, 8);

class raiden2_state : public driver_device
{
public:
	raiden2_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		/*
		  back_data(*this, "back_data"),
		  fore_data(*this, "fore_data"),
		  mid_data(*this, "mid_data"),
		  text_data(*this, "text_data"),
		  */
			sprites(*this, "sprites") ,
			m_maincpu(*this, "maincpu"),
			m_seibu_sound(*this, "seibu_sound"),
			m_gfxdecode(*this, "gfxdecode"),
			m_palette(*this, "palette"),

			bg_bank(0),
			fg_bank(0),
			mid_bank(0),
			tx_bank(0),
			raiden2_tilemap_enable(0),
			prg_bank(0),
			cop_bank(0),

			sprite_prot_x(0),
			sprite_prot_y(0),
			dst1(0),
			cop_spr_maxx(0),
			cop_spr_off(0),

			tile_buffer(320, 256),
			sprite_buffer(320, 256),
			m_raiden2cop(*this, "raiden2cop")
	{
		memset(scrollvals, 0, sizeof(uint16_t)*6);
		memset(sprite_prot_src_addr, 0, sizeof(uint16_t)*2);

	}

	std::unique_ptr<uint16_t[]> back_data;
	std::unique_ptr<uint16_t[]> fore_data;
	std::unique_ptr<uint16_t[]> mid_data;
	std::unique_ptr<uint16_t[]> text_data; // private buffers, allocated in init
	required_shared_ptr<uint16_t> sprites;
	required_device<cpu_device> m_maincpu;
	optional_device<seibu_sound_device> m_seibu_sound;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;




	DECLARE_WRITE16_MEMBER ( raiden2_bank_w );
	DECLARE_READ16_MEMBER ( cop_tile_bank_2_r );
	DECLARE_WRITE16_MEMBER ( cop_tile_bank_2_w );
	DECLARE_WRITE16_MEMBER ( raidendx_cop_bank_2_w );
	DECLARE_WRITE16_MEMBER ( tilemap_enable_w );
	DECLARE_WRITE16_MEMBER ( tile_scroll_w );
	DECLARE_WRITE16_MEMBER ( tile_bank_01_w );
	DECLARE_WRITE16_MEMBER ( raiden2_background_w );
	DECLARE_WRITE16_MEMBER ( raiden2_foreground_w );
	DECLARE_WRITE16_MEMBER ( raiden2_midground_w );
	DECLARE_WRITE16_MEMBER ( raiden2_text_w );
	DECLARE_WRITE16_MEMBER(m_videoram_private_w);

	DECLARE_WRITE16_MEMBER( sprcpt_val_1_w );
	DECLARE_WRITE16_MEMBER( sprcpt_val_2_w );
	DECLARE_WRITE16_MEMBER( sprcpt_data_1_w );
	DECLARE_WRITE16_MEMBER( sprcpt_data_2_w );
	DECLARE_WRITE16_MEMBER( sprcpt_data_3_w );
	DECLARE_WRITE16_MEMBER( sprcpt_data_4_w );
	DECLARE_WRITE16_MEMBER( sprcpt_adr_w );
	DECLARE_WRITE16_MEMBER( sprcpt_flags_1_w );
	DECLARE_WRITE16_MEMBER( sprcpt_flags_2_w );

	DECLARE_READ16_MEMBER( raiden2_sound_comms_r );
	DECLARE_WRITE16_MEMBER( raiden2_sound_comms_w );

	void common_reset();

	static uint16_t const raiden_blended_colors[];
	static uint16_t const xsedae_blended_colors[];
	static uint16_t const zeroteam_blended_colors[];

	bool blend_active[0x800]; // cfg

	tilemap_t *background_layer,*midground_layer,*foreground_layer,*text_layer;


	int bg_bank, fg_bank, mid_bank, tx_bank;
	uint16_t raiden2_tilemap_enable;
	uint8_t prg_bank;
	uint16_t cop_bank;

	uint16_t scrollvals[6];




	DECLARE_WRITE16_MEMBER( sprite_prot_x_w );
	DECLARE_WRITE16_MEMBER( sprite_prot_y_w );
	DECLARE_WRITE16_MEMBER( sprite_prot_src_seg_w );
	DECLARE_WRITE16_MEMBER( sprite_prot_src_w );
	DECLARE_READ16_MEMBER( sprite_prot_src_seg_r );
	DECLARE_READ16_MEMBER( sprite_prot_dst1_r );
	DECLARE_READ16_MEMBER( sprite_prot_maxx_r );
	DECLARE_READ16_MEMBER( sprite_prot_off_r );
	DECLARE_WRITE16_MEMBER( sprite_prot_dst1_w );
	DECLARE_WRITE16_MEMBER( sprite_prot_maxx_w );
	DECLARE_WRITE16_MEMBER( sprite_prot_off_w );

	uint16_t sprite_prot_x,sprite_prot_y,dst1,cop_spr_maxx,cop_spr_off;
	uint16_t sprite_prot_src_addr[2];



	void draw_sprites(const rectangle &cliprect);



	const int *cur_spri; // cfg

	DECLARE_DRIVER_INIT(raidendx);
	DECLARE_DRIVER_INIT(xsedae);
	DECLARE_DRIVER_INIT(zeroteam);
	DECLARE_DRIVER_INIT(raiden2);
	TILE_GET_INFO_MEMBER(get_back_tile_info);
	TILE_GET_INFO_MEMBER(get_mid_tile_info);
	TILE_GET_INFO_MEMBER(get_fore_tile_info);
	TILE_GET_INFO_MEMBER(get_text_tile_info);
	DECLARE_MACHINE_RESET(raiden2);
	DECLARE_VIDEO_START(raiden2);
	DECLARE_MACHINE_RESET(zeroteam);
	DECLARE_MACHINE_RESET(xsedae);
	DECLARE_MACHINE_RESET(raidendx);
	uint32_t screen_update_raiden2(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	INTERRUPT_GEN_MEMBER(raiden2_interrupt);
	void combine32(uint32_t *val, int offset, uint16_t data, uint16_t mem_mask);
	void sprcpt_init(void);

	void blend_layer(bitmap_rgb32 &bitmap, const rectangle &cliprect, bitmap_ind16 &source, int layer);
	void tilemap_draw_and_blend(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect, tilemap_t *tilemap);

	void init_blending(const uint16_t *table);

	bitmap_ind16 tile_buffer, sprite_buffer;
	optional_device<raiden2cop_device> m_raiden2cop;

protected:
	virtual void machine_start() override;
};
