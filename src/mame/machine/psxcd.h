// license:BSD-3-Clause
// copyright-holders:smf,R. Belmont,pSXAuthor,Carl
#ifndef MAME_MACHINE_PSXCD_H
#define MAME_MACHINE_PSXCD_H

#pragma once

#include "imagedev/chd_cd.h"
#include "sound/spu.h"

//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_PSXCD_ADD(_tag, _devname) \
	MCFG_DEVICE_ADD(_tag, PSXCD, 0)

#define MCFG_PSXCD_IRQ_HANDLER(_devcb) \
	devcb = &psxcd_device::set_irq_handler(*device, DEVCB_##_devcb);

class psxcd_device : public cdrom_image_device
{
public:
	psxcd_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration helpers
	template <class Object> static devcb_base &set_irq_handler(device_t &device, Object &&cb) { return downcast<psxcd_device &>(device).m_irq_handler.set_callback(std::forward<Object>(cb)); }
	virtual image_init_result call_load() override;
	virtual void call_unload() override;

	DECLARE_WRITE8_MEMBER( write );
	DECLARE_READ8_MEMBER( read );
	void start_dma(uint8_t *mainram, uint32_t size);

protected:
	virtual void device_start() override;
	virtual void device_stop() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

private:
	static constexpr unsigned MAX_PSXCD_TIMERS = 4;

	void write_command(uint8_t byte);

	typedef void (psxcd_device::*cdcmd)();
	struct command_result
	{
		uint8_t data[32], sz, res;
		command_result *next;
	};
	union CDPOS {
		uint8_t b[4];
		uint32_t w;
	};

	void cdcmd_sync();
	void cdcmd_nop();
	void cdcmd_setloc();
	void cdcmd_play();
	void cdcmd_forward();
	void cdcmd_backward();
	void cdcmd_readn();
	void cdcmd_standby();
	void cdcmd_stop();
	void cdcmd_pause();
	void cdcmd_init();
	void cdcmd_mute();
	void cdcmd_demute();
	void cdcmd_setfilter();
	void cdcmd_setmode();
	void cdcmd_getparam();
	void cdcmd_getlocl();
	void cdcmd_getlocp();
	void cdcmd_gettn();
	void cdcmd_gettd();
	void cdcmd_seekl();
	void cdcmd_seekp();
	void cdcmd_test();
	void cdcmd_id();
	void cdcmd_reads();
	void cdcmd_reset();
	void cdcmd_readtoc();
	void cdcmd_unknown12();
	void cdcmd_illegal17();
	void cdcmd_illegal18();
	void cdcmd_illegal1d();

	static const cdcmd cmd_table[31];
	void illegalcmd(uint8_t cmd);

	void cmd_complete(command_result *res);
	void send_result(uint8_t res, uint8_t *data=nullptr, int sz=0, int delay=default_irq_delay, uint8_t errcode = 0);
	command_result *prepare_result(uint8_t res, uint8_t *data=nullptr, int sz=0, uint8_t errcode = 0);

	void start_read();
	void start_play();
	void stop_read();
	void read_sector();
	void play_sector();
	uint32_t sub_loc(CDPOS src1, CDPOS src2);
	int add_system_event(int type, uint64_t t, command_result *ptr);

	uint8_t bcd_to_decimal(const uint8_t bcd) { return ((bcd>>4)*10)+(bcd&0xf); }
	uint8_t decimal_to_bcd(const uint8_t dec) { return ((dec/10)<<4)|(dec%10); }
	uint32_t msf_to_lba_ps(uint32_t msf) { msf = msf_to_lba(msf); return (msf>150)?(msf-150):msf; }
	uint32_t lba_to_msf_ps(uint32_t lba) { return lba_to_msf_alt(lba+150); }

	static const unsigned int sector_buffer_size=16, default_irq_delay=16000;   //480;  //8000; //2000<<2;
	static const unsigned int raw_sector_size=2352;

	uint8_t cmdbuf[64], mode, secbuf[sector_buffer_size][raw_sector_size];
	uint8_t filter_file, filter_channel, lastsechdr[8], status;
	int rdp;
	uint8_t m_cursec, sectail;
	uint16_t m_transcurr;
	uint8_t m_transbuf[raw_sector_size];
	command_result *res_queue, *m_int1;

	struct {
		uint8_t sr, ir, imr;
		struct {
			uint8_t ll, lr, rl, rr;
		} vol;
	} m_regs;

	CDPOS loc, curpos;

#ifdef LSB_FIRST
	enum {
		M = 2,
		S = 1,
		F = 0
	};
#else
	enum {
		M = 1,
		S = 2,
		F = 3
	};
#endif

	bool open, m_mute, m_dmaload;
	device_timer_id next_read_event;
	int64_t next_sector_t;
	unsigned int autopause_sector, start_read_delay, read_sector_cycles, preread_delay;

	uint32_t m_param_count;
	uint32_t m_sysclock;
	emu_timer *m_timers[MAX_PSXCD_TIMERS];
	bool m_timerinuse[MAX_PSXCD_TIMERS];


	devcb_write_line m_irq_handler;
	cpu_device *m_maincpu;
	spu_device *m_spu;
};

// device type definition
DECLARE_DEVICE_TYPE(PSXCD, psxcd_device)

#endif // MAME_MACHINE_PSXCD_H
