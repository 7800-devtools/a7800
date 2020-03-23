// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    Hudson/NEC HuC6270 interface

**********************************************************************/

#ifndef MAME_VIDEO_HUC6270_H
#define MAME_VIDEO_HUC6270_H

#pragma once


#define MCFG_HUC6270_VRAM_SIZE(_size) \
	huc6270_device::set_vram_size(*device, _size);

#define MCFG_HUC6270_IRQ_CHANGED_CB(_devcb) \
	devcb = &huc6270_device::set_irq_changed_callback(*device, DEVCB_##_devcb);

class huc6270_device : public device_t
{
public:
	// construction/destruction
	huc6270_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void set_vram_size(device_t &device, uint32_t vram_size) { downcast<huc6270_device &>(device).m_vram_size = vram_size; }
	template <class Object> static devcb_base &set_irq_changed_callback(device_t &device, Object &&cb) { return downcast<huc6270_device &>(device).m_irq_changed_cb.set_callback(std::forward<Object>(cb)); }

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );
	DECLARE_READ16_MEMBER( next_pixel );
	inline DECLARE_READ16_MEMBER( time_until_next_event )
	{
		return m_horz_to_go * 8 + m_horz_steps;
	}

	DECLARE_WRITE_LINE_MEMBER( vsync_changed );
	DECLARE_WRITE_LINE_MEMBER( hsync_changed );

	static const uint16_t HUC6270_SPRITE     = 0x0100;    // sprite colour information
	static const uint16_t HUC6270_BACKGROUND = 0x0000;    // background colour information

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	inline void fetch_bat_tile_row();
	void add_sprite( int index, int x, int pattern, int line, int flip_x, int palette, int priority, int sat_lsb );
	void select_sprites();
	inline void handle_vblank();
	inline void next_vert_state();
	inline void next_horz_state();

private:
	enum class v_state : u8 {
		VSW,
		VDS,
		VDW,
		VCR
	};

	enum class h_state : u8 {
		HDS,
		HDW,
		HDE,
		HSW
	};


	/* Size of Video ram (mandatory) */
	uint32_t m_vram_size;

	/* Callback for when the irq line may have changed (mandatory) */
	devcb_write_line    m_irq_changed_cb;

	uint8_t   m_register_index;

	/* HuC6270 registers */
	uint16_t  m_mawr;
	uint16_t  m_marr;
	uint16_t  m_vrr;
	uint16_t  m_vwr;
	uint16_t  m_cr;
	uint16_t  m_rcr;
	uint16_t  m_bxr;
	uint16_t  m_byr;
	uint16_t  m_mwr;
	uint16_t  m_hsr;
	uint16_t  m_hdr;
	uint16_t  m_vpr;
	uint16_t  m_vdw;
	uint16_t  m_vcr;
	uint16_t  m_dcr;
	uint16_t  m_sour;
	uint16_t  m_desr;
	uint16_t  m_lenr;
	uint16_t  m_dvssr;
	uint8_t   m_status;

	/* To keep track of external hsync and vsync signals */
	int m_hsync;
	int m_vsync;

	/* internal variables */
	v_state m_vert_state;
	h_state m_horz_state;
	int m_vd_triggered;
	int m_vert_to_go;
	int m_horz_to_go;
	int m_horz_steps;
	int m_raster_count;
	int m_dvssr_written;
	int m_satb_countdown;
	int m_dma_enabled;
	uint16_t m_byr_latched;
	uint16_t m_bxr_latched;
	uint16_t m_bat_address;
	uint16_t m_bat_address_mask;
	uint16_t m_bat_row;
	uint16_t m_bat_column;
	uint8_t m_bat_tile_row[8];
	/* Internal sprite attribute table. SATB DMA is used to transfer data
	   from VRAM to this internal table.
	*/
	uint16_t m_sat[4*64];
	int m_sprites_this_line;
	int m_sprite_row_index;
	uint16_t  m_sprite_row[1024];
	std::unique_ptr<uint16_t[]>  m_vram;
	uint16_t  m_vram_mask;

	static constexpr uint8_t vram_increments[4] = { 1, 32, 64, 128 };
};


DECLARE_DEVICE_TYPE(HUC6270, huc6270_device)

#endif // MAME_VIDEO_HUC6270_H
