// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Sinclair ZX8301 emulation

**********************************************************************
                            _____   _____
                DTACKL   1 |*    \_/     | 40  WEL
                   A17   2 |             | 39  PCENL
                   A16   3 |             | 38  VDA
                  RDWL   4 |             | 37  ROWL
                 DSMCL   5 |             | 36  TX0EL
                   VCC   6 |             | 35  XTAL2
                CLKCPU   7 |             | 34  XTAL1
                  RASL   8 |             | 33  ROM0EH
                 CAS0L   9 |             | 32  BLUE
                 CAS1L  10 |    ZX8301   | 31  GREEN
                VSYNCH  11 |     ULA     | 30  RED
                CSYNCL  12 |             | 29  DB7
                   DA0  13 |             | 28  DA7
                   DB0  14 |             | 27  DA6
                   VDD  15 |             | 26  DB6
                   DB1  16 |             | 25  DB5
                   DA1  17 |             | 24  DA5
                   DA2  18 |             | 23  DB4
                   DB2  19 |             | 22  DA4
                   DA3  20 |_____________| 21  DB3

**********************************************************************/

#ifndef MAME_VIDEO_ZX8301_H
#define MAME_VIDEO_ZX8301_H

#pragma once




///*************************************************************************
//  INTERFACE CONFIGURATION MACROS
///*************************************************************************

#define MCFG_ZX8301_CPU(_tag) \
	zx8301_device::static_set_cpu_tag(*device, "^" _tag);

#define MCFG_ZX8301_VSYNC_CALLBACK(_write) \
	devcb = &zx8301_device::set_vsync_wr_callback(*device, DEVCB_##_write);



///*************************************************************************
//  TYPE DEFINITIONS
///*************************************************************************

// ======================> zx8301_device

class zx8301_device :   public device_t,
						public device_memory_interface,
						public device_video_interface
{
public:
	// construction/destruction
	zx8301_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_vsync_wr_callback(device_t &device, Object &&cb) { return downcast<zx8301_device &>(device).m_write_vsync.set_callback(std::forward<Object>(cb)); }
	static void static_set_cpu_tag(device_t &device, const char *tag) { downcast<zx8301_device &>(device).m_cpu.set_tag(tag); }

	DECLARE_WRITE8_MEMBER( control_w );
	DECLARE_READ8_MEMBER( data_r );
	DECLARE_WRITE8_MEMBER( data_w );

	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// device_config_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// address space configurations
	const address_space_config      m_space_config;

	inline uint8_t readbyte(offs_t address);
	inline void writebyte(offs_t address, uint8_t data);

	void draw_line_mode4(bitmap_rgb32 &bitmap, int y, uint16_t da);
	void draw_line_mode8(bitmap_rgb32 &bitmap, int y, uint16_t da);

private:
	enum
	{
		TIMER_VSYNC,
		TIMER_FLASH
	};

	required_device<cpu_device> m_cpu;

	devcb_write_line   m_write_vsync;

	//address_space *m_data;

	int m_dispoff;                  // display off
	int m_mode8;                    // mode8 active
	int m_base;                     // video ram base address
	int m_flash;                    // flash
	int m_vsync;                    // vertical sync
	int m_vda;                      // valid data address

	emu_timer *m_vsync_timer;       // vertical sync timer
	emu_timer *m_flash_timer;       // flash timer
};


// device type definition
DECLARE_DEVICE_TYPE(ZX8301, zx8301_device)



#endif // MAME_VIDEO_ZX8301_H
