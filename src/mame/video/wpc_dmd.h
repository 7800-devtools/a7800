// license:BSD-3-Clause
// copyright-holders:Olivier Galibert

// Williams Pinball Controller Dot Matrix Display

// A 128x32 plasma display with 16 pages and refreshed at 240Hz (for PWM luminosity control)

#ifndef MAME_VIDEO_WPC_DMD_H
#define MAME_VIDEO_WPC_DMD_H

#define MCFG_WPC_DMD_ADD( _tag, _scanline_cb ) \
	MCFG_DEVICE_ADD( _tag, WPC_DMD, 0 ) \
	devcb = &wpc_dmd_device::set_scanline_cb(*device, DEVCB_##_scanline_cb);

class wpc_dmd_device : public device_t
{
public:
	wpc_dmd_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~wpc_dmd_device();

	DECLARE_ADDRESS_MAP(registers, 8);

	DECLARE_WRITE8_MEMBER(bank0_w);
	DECLARE_WRITE8_MEMBER(bank2_w);
	DECLARE_WRITE8_MEMBER(bank4_w);
	DECLARE_WRITE8_MEMBER(bank6_w);
	DECLARE_WRITE8_MEMBER(bank8_w);
	DECLARE_WRITE8_MEMBER(banka_w);
	DECLARE_WRITE8_MEMBER(visible_page_w);
	DECLARE_WRITE8_MEMBER(firq_scanline_w);

	template <class Object> static devcb_base &set_scanline_cb(device_t &device, Object &&cb) { return downcast<wpc_dmd_device &>(device).scanline_cb.set_callback(std::forward<Object>(cb)); }

protected:
	devcb_write_line scanline_cb;
	required_memory_bank dmd0, dmd2, dmd4, dmd6, dmd8, dmda;

	uint8_t cur_scanline, visible_page, firq_scanline;
	std::vector<uint8_t> ram, screen_buffer, bitcounts;

	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	TIMER_DEVICE_CALLBACK_MEMBER(scanline_timer);
	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
};

DECLARE_DEVICE_TYPE(WPC_DMD, wpc_dmd_device)

#endif // MAME_VIDEO_WPC_DMD_H
