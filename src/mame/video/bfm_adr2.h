// license:BSD-3-Clause
// copyright-holders:David Haywood
#ifndef MAME_INCLUDES_BFM_ADR2_H
#define MAME_INCLUDES_BFM_ADR2_H

#pragma once


class bfm_adder2_device : public device_t, public device_gfx_interface
{
public:
	// construction/destruction
	bfm_adder2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	TILE_GET_INFO_MEMBER( get_tile0_info );
	TILE_GET_INFO_MEMBER( get_tile1_info );

	DECLARE_READ8_MEMBER( screen_ram_r );
	DECLARE_WRITE8_MEMBER( screen_ram_w );
	DECLARE_READ8_MEMBER( normal_ram_r );
	DECLARE_WRITE8_MEMBER( normal_ram_w );
	DECLARE_WRITE8_MEMBER( adder2_rom_page_w );
	DECLARE_WRITE8_MEMBER( adder2_c001_w );
	DECLARE_WRITE8_MEMBER( adder2_screen_page_w );
	DECLARE_READ8_MEMBER( adder2_vbl_ctrl_r );
	DECLARE_WRITE8_MEMBER( adder2_vbl_ctrl_w );
	DECLARE_READ8_MEMBER( adder2_uart_ctrl_r );
	DECLARE_WRITE8_MEMBER( adder2_uart_ctrl_w );
	DECLARE_READ8_MEMBER( adder2_uart_rx_r );
	DECLARE_WRITE8_MEMBER( adder2_uart_tx_w );
	DECLARE_READ8_MEMBER( adder2_irq_r );

	DECLARE_WRITE8_MEMBER(vid_uart_tx_w);
	DECLARE_WRITE8_MEMBER(vid_uart_ctrl_w);
	DECLARE_READ8_MEMBER(vid_uart_rx_r);
	DECLARE_READ8_MEMBER(vid_uart_ctrl_r);

	void adder2_decode_char_roms();

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	int m_adder2_screen_page_reg;        // access/display select
	int m_adder2_c101;
	int m_adder2_rx;
	int m_adder_vbl_triggered;           // flag <>0, VBL IRQ triggered
	int m_adder2_acia_triggered;         // flag <>0, ACIA receive IRQ

	uint8_t m_adder_ram[0xE80];              // normal RAM
	uint8_t m_adder_screen_ram[2][0x1180];   // paged  display RAM

	tilemap_t *m_tilemap0;  // tilemap screen0
	tilemap_t *m_tilemap1;  // timemap screen1

	uint8_t m_adder2_data_from_sc2;
	uint8_t m_adder2_data_to_sc2;

	uint8_t m_adder2_data;
	uint8_t m_adder2_sc2data;

	optional_device<cpu_device> m_cpu;

	uint32_t update_screen(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	INTERRUPT_GEN_MEMBER( adder2_vbl );
};

// device type definition
DECLARE_DEVICE_TYPE(BFM_ADDER2, bfm_adder2_device)


//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_BFM_ADDER2_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, BFM_ADDER2, 0)

#endif // MAME_INCLUDES_BFM_ADR2_H
