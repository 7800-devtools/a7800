// license:BSD-3-Clause
// copyright-holders:Phil Stroffolino, hap, R. Belmont
/***************************************************************************

    Namco System 22 / Super System 22 hardware

***************************************************************************/

#include "machine/eeprompar.h"
#include "video/rgbutil.h"
#include "video/poly.h"
#include "screen.h"

enum
{
	NAMCOS22_AIR_COMBAT22,
	NAMCOS22_ALPINE_RACER,
	NAMCOS22_CYBER_COMMANDO,
	NAMCOS22_CYBER_CYCLES,
	NAMCOS22_PROP_CYCLE,
	NAMCOS22_RAVE_RACER,
	NAMCOS22_RIDGE_RACER,
	NAMCOS22_RIDGE_RACER2,
	NAMCOS22_TIME_CRISIS,
	NAMCOS22_VICTORY_LAP,
	NAMCOS22_ACE_DRIVER,
	NAMCOS22_ALPINE_RACER_2,
	NAMCOS22_ALPINE_SURFER,
	NAMCOS22_TOKYO_WARS,
	NAMCOS22_AQUA_JET,
	NAMCOS22_DIRT_DASH,
	NAMCOS22_ARMADILLO_RACING
};


struct namcos22_polyvertex
{
	float x, y, z;
	int u, v; /* 0..0xfff */
	int bri;  /* 0..0xff */
};

enum namcos22_scenenode_type
{
	NAMCOS22_SCENENODE_NONLEAF,
	NAMCOS22_SCENENODE_QUAD,
	NAMCOS22_SCENENODE_SPRITE
};

#define NAMCOS22_RADIX_BITS 4
#define NAMCOS22_RADIX_BUCKETS (1 << NAMCOS22_RADIX_BITS)
#define NAMCOS22_RADIX_MASK (NAMCOS22_RADIX_BUCKETS - 1)

struct namcos22_scenenode
{
	namcos22_scenenode_type type;
	struct namcos22_scenenode *next;
	union
	{
		struct
		{
			struct namcos22_scenenode *next[NAMCOS22_RADIX_BUCKETS];
		} nonleaf;

		struct
		{
			float vx, vy, vw, vh;
			int texturebank;
			int color;
			int cmode;
			int flags;
			int cz_adjust;
			int direct;
			namcos22_polyvertex v[4];
		} quad;

		struct
		{
			int tile, color, pri;
			int flipx, flipy;
			int linktype;
			int cols, rows;
			int xpos, ypos;
			int cx_min, cx_max;
			int cy_min, cy_max;
			int sizex, sizey;
			int translucency;
			int cz;
		} sprite;
	} data;
};


struct namcos22_object_data
{
	/* poly / sprites */
	rgbaint_t fogcolor;
	rgbaint_t fadecolor;
	rgbaint_t polycolor;
	const pen_t *pens;
	bitmap_rgb32 *destbase;
	bitmap_ind8 *primap;
	int bn;
	int flags;
	int prioverchar;
	int cmode;
	int fadefactor;
	int pfade_enabled;
	int fogfactor;
	int zfog_enabled;
	int cz_adjust;
	int cz_sdelta;
	const uint8_t *czram;

	/* sprites */
	const uint8_t *source;
	int alpha;
	int line_modulo;
	int flipx;
	int flipy;
};


class namcos22_state;

class namcos22_renderer : public poly_manager<float, namcos22_object_data, 4, 8000>
{
public:
	namcos22_renderer(namcos22_state &state);

	void render_scene(screen_device &screen, bitmap_rgb32 &bitmap);
	struct namcos22_scenenode *new_scenenode(running_machine &machine, uint32_t zsort, namcos22_scenenode_type type);

	void reset();

private:
	namcos22_state &m_state;

	struct namcos22_scenenode m_scenenode_root;
	struct namcos22_scenenode *m_scenenode_cur;

	int m_clipx;
	int m_clipy;
	rectangle m_cliprect;

	inline uint8_t nthbyte(const uint32_t *src, int n) { return (src[n / 4] << ((n & 3) * 8)) >> 24; }
	inline uint16_t nthword(const uint32_t *src, int n) { return (src[n / 2] << ((n & 1) * 16)) >> 16; }

	void render_scene_nodes(screen_device &screen, bitmap_rgb32 &bitmap, struct namcos22_scenenode *node);
	void render_sprite(screen_device &screen, bitmap_rgb32 &bitmap, struct namcos22_scenenode *node);
	void poly3d_drawquad(screen_device &screen, bitmap_rgb32 &bitmap, struct namcos22_scenenode *node);
	void poly3d_drawsprite(screen_device &screen, bitmap_rgb32 &dest_bmp, uint32_t code, uint32_t color, int flipx, int flipy, int sx, int sy, int scalex, int scaley, int cz_factor, int prioverchar, int alpha);

	void free_scenenode(struct namcos22_scenenode *node);
	struct namcos22_scenenode *alloc_scenenode(running_machine &machine, struct namcos22_scenenode *node);

	void renderscanline_uvi_full(int32_t scanline, const extent_t &extent, const namcos22_object_data &extra, int threadid);
	void renderscanline_sprite(int32_t scanline, const extent_t &extent, const namcos22_object_data &extra, int threadid);
};


enum namcos22_dsp_upload_state
{
	NAMCOS22_DSP_UPLOAD_READY,
	NAMCOS22_DSP_UPLOAD_DEST,
	NAMCOS22_DSP_UPLOAD_DATA
};

#define NAMCOS22_MAX_LIT_SURFACES 0x80
#define NAMCOS22_MAX_RENDER_CMD_SEQ 0x1c

class namcos22_state : public driver_device
{
public:
	namcos22_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_master(*this, "master"),
		m_slave(*this, "slave"),
		m_mcu(*this, "mcu"),
		m_iomcu(*this, "iomcu"),
		m_spriteram(*this, "spriteram"),
		m_shareram(*this, "shareram"),
		m_eeprom(*this, "eeprom"),
		m_pSlaveExternalRAM(*this, "slaveextram"),
		m_pMasterExternalRAM(*this, "masterextram"),
		m_paletteram(*this, "paletteram"),
		m_cgram(*this, "cgram"),
		m_textram(*this, "textram"),
		m_polygonram(*this, "polygonram"),
		m_mixer(*this, "video_mixer"),
		m_gamma_proms(*this, "gamma_proms"),
		m_vics_data(*this, "vics_data"),
		m_vics_control(*this, "vics_control"),
		m_czattr(*this, "czattr"),
		m_tilemapattr(*this, "tilemapattr"),
		m_czram(*this, "czram"),
		m_motor_timer(*this, "motor_timer"),
		m_pc_pedal_interrupt(*this, "pc_p_int"),
		m_gfxdecode(*this, "gfxdecode"),
		m_screen(*this, "screen"),
		m_palette(*this, "palette"),
		m_adc_ports(*this, "ADC.%u", 0),
		m_p1(*this, "P1"),
		m_p2(*this, "P2"),
		m_mcup5a(*this, "MCUP5A"),
		m_mcup5b(*this, "MCUP5B")
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_master;
	required_device<cpu_device> m_slave;
	required_device<cpu_device> m_mcu;
	optional_device<cpu_device> m_iomcu;
	optional_shared_ptr<uint32_t> m_spriteram;
	required_shared_ptr<uint32_t> m_shareram;
	required_device<eeprom_parallel_28xx_device> m_eeprom;
	required_shared_ptr<uint16_t> m_pSlaveExternalRAM;
	required_shared_ptr<uint16_t> m_pMasterExternalRAM;
	required_shared_ptr<uint32_t> m_paletteram;
	required_shared_ptr<uint32_t> m_cgram;
	required_shared_ptr<uint32_t> m_textram;
	required_shared_ptr<uint32_t> m_polygonram;
	required_shared_ptr<uint32_t> m_mixer;
	optional_region_ptr<uint8_t> m_gamma_proms;
	optional_shared_ptr<uint32_t> m_vics_data;
	optional_shared_ptr<uint32_t> m_vics_control;
	optional_shared_ptr<uint32_t> m_czattr;
	required_shared_ptr<uint32_t> m_tilemapattr;
	optional_shared_ptr<uint32_t> m_czram;
	optional_device<timer_device> m_motor_timer;
	optional_device<timer_device> m_pc_pedal_interrupt;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
	optional_ioport_array<8> m_adc_ports;
	optional_ioport m_p1;
	optional_ioport m_p2;
	optional_ioport m_mcup5a;
	optional_ioport m_mcup5b;

	uint8_t m_syscontrol[0x20];
	bool m_dsp_irq_enabled;
	emu_timer *m_ar_tb_interrupt[2];
	uint16_t m_dsp_master_bioz;
	std::unique_ptr<uint32_t[]> m_pointram;
	uint32_t m_old_coin_state;
	uint32_t m_credits1;
	uint32_t m_credits2;
	uint32_t m_point_address;
	uint32_t m_point_data;
	uint16_t m_SerialDataSlaveToMasterNext;
	uint16_t m_SerialDataSlaveToMasterCurrent;
	int m_RenderBufSize;
	uint16_t m_RenderBufData[NAMCOS22_MAX_RENDER_CMD_SEQ];
	uint16_t m_portbits[2];
	int m_irq_state;
	namcos22_dsp_upload_state m_dsp_upload_state;
	int m_UploadDestIdx;
	uint32_t m_alpinesa_protection;
	int m_motor_status;
	int m_p4;
	uint16_t m_su_82;
	uint16_t m_keycus_id;
	uint16_t m_keycus_rng;
	int m_gametype;
	int m_is_ss22;
	int m_chipselect;
	int m_spot_enable;
	int m_spot_read_address;
	int m_spot_write_address;
	std::unique_ptr<uint16_t[]> m_spotram;
	std::unique_ptr<uint16_t[]> m_banked_czram[4];
	std::unique_ptr<uint8_t[]> m_recalc_czram[4];
	uint32_t m_cz_was_written[4];
	int m_cz_adjust;
	namcos22_renderer *m_poly;
	uint16_t *m_texture_tilemap;
	std::unique_ptr<uint8_t[]> m_texture_tileattr;
	uint8_t *m_texture_tiledata;
	std::unique_ptr<uint8_t[]> m_texture_ayx_to_pixel;
	uint16_t m_dspram_bank;
	uint16_t m_dspram16_latch;
	bool m_slave_simulation_active;
	int32_t m_absolute_priority;
	int32_t m_objectshift;
	uint16_t m_PrimitiveID;
	float m_viewmatrix[4][4];
	uint8_t m_LitSurfaceInfo[NAMCOS22_MAX_LIT_SURFACES];
	int32_t m_SurfaceNormalFormat;
	unsigned m_LitSurfaceCount;
	unsigned m_LitSurfaceIndex;
	int m_pointrom_size;
	int32_t *m_pointrom;
	std::unique_ptr<uint8_t[]> m_dirtypal;
	std::unique_ptr<bitmap_ind16> m_mix_bitmap;
	tilemap_t *m_bgtilemap;

	int m_mixer_flags;
	int m_fog_r;
	int m_fog_g;
	int m_fog_b;
	int m_fog_r_per_cztype[4];
	int m_fog_g_per_cztype[4];
	int m_fog_b_per_cztype[4];
	uint32_t m_fog_colormask;
	int m_poly_fade_r;
	int m_poly_fade_g;
	int m_poly_fade_b;
	int m_poly_fade_enabled;
	int m_screen_fade_r;
	int m_screen_fade_g;
	int m_screen_fade_b;
	int m_screen_fade_factor;
	int m_spot_limit;
	int m_poly_translucency;
	int m_text_palbase;

	float m_camera_zoom;
	float m_camera_vx;
	float m_camera_vy;
	float m_camera_vw;
	float m_camera_vh;
	float m_camera_lx; // unit vector for light direction
	float m_camera_ly; // "
	float m_camera_lz; // "
	int m_camera_ambient; // 0.0..1.0
	int m_camera_power;   // 0.0..1.0

	DECLARE_WRITE32_MEMBER(namcos22s_czram_w);
	DECLARE_READ32_MEMBER(namcos22s_czram_r);
	DECLARE_READ32_MEMBER(namcos22s_vics_control_r);
	DECLARE_WRITE32_MEMBER(namcos22s_vics_control_w);
	DECLARE_WRITE32_MEMBER(namcos22_textram_w);
	DECLARE_READ32_MEMBER(namcos22_tilemapattr_r);
	DECLARE_WRITE32_MEMBER(namcos22_tilemapattr_w);
	DECLARE_READ32_MEMBER(namcos22s_spotram_r);
	DECLARE_WRITE32_MEMBER(namcos22s_spotram_w);
	DECLARE_READ32_MEMBER(namcos22_dspram_r);
	DECLARE_WRITE32_MEMBER(namcos22_dspram_w);
	DECLARE_WRITE32_MEMBER(namcos22_cgram_w);
	DECLARE_WRITE32_MEMBER(namcos22_paletteram_w);
	DECLARE_WRITE16_MEMBER(namcos22_dspram16_bank_w);
	DECLARE_READ16_MEMBER(namcos22_dspram16_r);
	DECLARE_WRITE16_MEMBER(namcos22_dspram16_w);
	DECLARE_CUSTOM_INPUT_MEMBER(alpine_motor_read);
	DECLARE_READ16_MEMBER(pdp_status_r);
	DECLARE_READ16_MEMBER(pdp_begin_r);
	DECLARE_READ16_MEMBER(slave_external_ram_r);
	DECLARE_WRITE16_MEMBER(slave_external_ram_w);
	DECLARE_READ16_MEMBER(dsp_hold_signal_r);
	DECLARE_WRITE16_MEMBER(dsp_hold_ack_w);
	DECLARE_WRITE16_MEMBER(dsp_xf_output_w);
	DECLARE_WRITE16_MEMBER(point_address_w);
	DECLARE_WRITE16_MEMBER(point_loword_iw);
	DECLARE_WRITE16_MEMBER(point_hiword_w);
	DECLARE_READ16_MEMBER(point_loword_r);
	DECLARE_READ16_MEMBER(point_hiword_ir);
	DECLARE_WRITE16_MEMBER(dsp_unk2_w);
	DECLARE_READ16_MEMBER(dsp_unk_port3_r);
	DECLARE_WRITE16_MEMBER(upload_code_to_slave_dsp_w);
	DECLARE_READ16_MEMBER(dsp_unk8_r);
	DECLARE_READ16_MEMBER(custom_ic_status_r);
	DECLARE_READ16_MEMBER(dsp_upload_status_r);
	DECLARE_READ16_MEMBER(master_external_ram_r);
	DECLARE_WRITE16_MEMBER(master_external_ram_w);
	DECLARE_WRITE16_MEMBER(slave_serial_io_w);
	DECLARE_READ16_MEMBER(master_serial_io_r);
	DECLARE_WRITE16_MEMBER(dsp_unk_porta_w);
	DECLARE_WRITE16_MEMBER(dsp_led_w);
	DECLARE_WRITE16_MEMBER(dsp_unk8_w);
	DECLARE_WRITE16_MEMBER(master_render_device_w);
	DECLARE_READ16_MEMBER(dsp_bioz_r);
	DECLARE_READ16_MEMBER(dsp_slave_port3_r);
	DECLARE_READ16_MEMBER(dsp_slave_port4_r);
	DECLARE_READ16_MEMBER(dsp_slave_port5_r);
	DECLARE_READ16_MEMBER(dsp_slave_port6_r);
	DECLARE_WRITE16_MEMBER(dsp_slave_portc_w);
	DECLARE_READ16_MEMBER(dsp_slave_port8_r);
	DECLARE_READ16_MEMBER(dsp_slave_portb_r);
	DECLARE_WRITE16_MEMBER(dsp_slave_portb_w);
	DECLARE_READ32_MEMBER(namcos22_sci_r);
	DECLARE_READ8_MEMBER(namcos22_system_controller_r);
	DECLARE_WRITE8_MEMBER(namcos22s_system_controller_w);
	DECLARE_WRITE8_MEMBER(namcos22_system_controller_w);
	DECLARE_READ16_MEMBER(namcos22_keycus_r);
	DECLARE_WRITE16_MEMBER(namcos22_keycus_w);
	DECLARE_READ16_MEMBER(namcos22_portbit_r);
	DECLARE_WRITE16_MEMBER(namcos22_portbit_w);
	DECLARE_READ16_MEMBER(namcos22_dipswitch_r);
	DECLARE_READ32_MEMBER(namcos22_gun_r);
	DECLARE_WRITE16_MEMBER(namcos22_cpuleds_w);
	DECLARE_READ32_MEMBER(alpinesa_prot_r);
	DECLARE_WRITE32_MEMBER(alpinesa_prot_w);
	DECLARE_WRITE32_MEMBER(namcos22s_chipselect_w);
	DECLARE_READ16_MEMBER(s22mcu_shared_r);
	DECLARE_WRITE16_MEMBER(s22mcu_shared_w);
	DECLARE_WRITE8_MEMBER(mcu_port4_w);
	DECLARE_READ8_MEMBER(mcu_port4_r);
	DECLARE_WRITE8_MEMBER(mcu_port5_w);
	DECLARE_READ8_MEMBER(mcu_port5_r);
	DECLARE_WRITE8_MEMBER(mcu_port6_w);
	DECLARE_READ8_MEMBER(mcu_port6_r);
	DECLARE_WRITE8_MEMBER(mcu_port7_w);
	DECLARE_READ8_MEMBER(mcu_port7_r);
	DECLARE_READ8_MEMBER(namcos22s_mcu_adc_r);
	DECLARE_WRITE8_MEMBER(propcycle_mcu_port5_w);
	DECLARE_WRITE8_MEMBER(alpine_mcu_port5_w);
	DECLARE_READ8_MEMBER(mcu_port4_s22_r);
	DECLARE_READ8_MEMBER(iomcu_port4_s22_r);
	DECLARE_READ16_MEMBER(mcu141_speedup_r);
	DECLARE_WRITE16_MEMBER(mcu_speedup_w);
	DECLARE_READ16_MEMBER(mcu130_speedup_r);
	DECLARE_READ16_MEMBER(mcuc74_speedup_r);

	inline uint8_t nthbyte(const uint32_t *src, int n) { return (src[n / 4] << ((n & 3) * 8)) >> 24; }
	inline uint16_t nthword(const uint32_t *src, int n) { return (src[n / 2] << ((n & 1) * 16)) >> 16; }

	inline int32_t signed18(int32_t val) { return (val & 0x00020000) ? (int32_t)(val | 0xfffc0000) : val & 0x0001ffff; }
	inline int32_t signed24(int32_t val) { return (val & 0x00800000) ? (int32_t)(val | 0xff000000) : val & 0x007fffff; }

	inline float dspfixed_to_nativefloat(int16_t val) { return val / (float)0x7fff; }
	float dspfloat_to_nativefloat(uint32_t val);

	void handle_driving_io();
	void handle_coinage(int slots, int address_is_odd);
	void handle_cybrcomm_io();
	uint32_t pdp_polygonram_read(offs_t offs);
	void pdp_polygonram_write(offs_t offs, uint32_t data);
	void point_write(offs_t offs, uint32_t data);
	void slave_halt();
	void slave_enable();
	void enable_slave_simulation(bool enable);

	void matrix3d_multiply(float a[4][4], float b[4][4]);
	void matrix3d_identity(float m[4][4]);
	void transform_point(float *vx, float *vy, float *vz, float m[4][4]);
	void transform_normal(float *nx, float *ny, float *nz, float m[4][4]);
	void register_normals(int32_t addr, float m[4][4]);

	void blit_single_quad(bitmap_rgb32 &bitmap, uint32_t color, uint32_t addr, float m[4][4], int32_t polyshift, int flags, int packetformat);
	void blit_quads(bitmap_rgb32 &bitmap, int32_t addr, float m[4][4], int32_t base);
	void blit_polyobject(bitmap_rgb32 &bitmap, int code, float m[4][4]);

	void slavesim_handle_bb0003(const int32_t *src);
	void slavesim_handle_200002(bitmap_rgb32 &bitmap, const int32_t *src);
	void slavesim_handle_300000(const int32_t *src);
	void slavesim_handle_233002(const int32_t *src);
	void simulate_slavedsp(bitmap_rgb32 &bitmap);

	int32_t point_read(int32_t addr);
	void init_tables();
	void update_mixer();
	void update_palette();
	void recalc_czram();
	void draw_direct_poly(const uint16_t *src);
	void draw_polygons(bitmap_rgb32 &bitmap);
	void draw_sprites(bitmap_rgb32 &bitmap, const rectangle &cliprect);
	void draw_sprite_group(bitmap_rgb32 &bitmap, const rectangle &cliprect, const uint32_t *src, const uint32_t *attr, int num_sprites, int deltax, int deltay, int y_lowres);
	void draw_text_layer(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	void namcos22s_mix_text_layer(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect, int prival);
	void namcos22_mix_text_layer(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	void install_c74_speedup();
	void install_130_speedup();
	void install_141_speedup();
	void namcos22_init(int game_type);

	DECLARE_DRIVER_INIT(acedrvr);
	DECLARE_DRIVER_INIT(aquajet);
	DECLARE_DRIVER_INIT(adillor);
	DECLARE_DRIVER_INIT(cybrcyc);
	DECLARE_DRIVER_INIT(raveracw);
	DECLARE_DRIVER_INIT(ridger2j);
	DECLARE_DRIVER_INIT(victlap);
	DECLARE_DRIVER_INIT(cybrcomm);
	DECLARE_DRIVER_INIT(timecris);
	DECLARE_DRIVER_INIT(tokyowar);
	DECLARE_DRIVER_INIT(propcycl);
	DECLARE_DRIVER_INIT(alpiner2);
	DECLARE_DRIVER_INIT(dirtdash);
	DECLARE_DRIVER_INIT(airco22);
	DECLARE_DRIVER_INIT(alpiner);
	DECLARE_DRIVER_INIT(ridgeraj);
	DECLARE_DRIVER_INIT(alpinesa);

	TILE_GET_INFO_MEMBER(get_text_tile_info);
	virtual void machine_reset() override;
	virtual void machine_start() override;
	virtual void video_start() override;
	DECLARE_MACHINE_START(adillor);
	uint32_t screen_update_namcos22s(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_namcos22(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	INTERRUPT_GEN_MEMBER(namcos22s_interrupt);
	INTERRUPT_GEN_MEMBER(namcos22_interrupt);
	TIMER_DEVICE_CALLBACK_MEMBER(adillor_trackball_update);
	TIMER_CALLBACK_MEMBER(adillor_trackball_interrupt);
	TIMER_DEVICE_CALLBACK_MEMBER(propcycl_pedal_update);
	TIMER_DEVICE_CALLBACK_MEMBER(propcycl_pedal_interrupt);
	TIMER_DEVICE_CALLBACK_MEMBER(alpine_steplock_callback);
	TIMER_DEVICE_CALLBACK_MEMBER(dsp_master_serial_irq);
	TIMER_DEVICE_CALLBACK_MEMBER(dsp_slave_serial_irq);
	TIMER_DEVICE_CALLBACK_MEMBER(mcu_irq);
};
