// license:BSD-3-Clause
// copyright-holders:Sergey Svishchev
/**********************************************************************

    Intel 7220 Bubble Memory Controller emulation

**********************************************************************
                            _____   _____
             /PWR.FAIL   1 |*    \_/     | 40  Vcc
            /RESET.OUT   2 |             | 39  /X+
                   CLK   3 |             | 38  /X-
                /RESET   4 |             | 37  /Y+
                   /RD   5 |             | 36  /Y-
                   /WR   6 |             | 35  /TM.A
                 /DACK   7 |             | 34  /TM.B
                   DRQ   8 |             | 33  /REP.EN
                   INT   9 |             | 32  /BOOT.EN
                    A0  10 |     7220    | 31  /SWAP.EN
                    D0  11 |             | 30  /BOOT.S.EN
                    D1  12 |             | 29  C/D
                    D2  13 |             | 28  /DET.ON
                    D3  14 |             | 27  /ERR.FLG
                    D4  15 |             | 26  /WAIT
                    D5  16 |             | 25  /BUS.RD
                    D6  17 |             | 24  /SHIFT.CLK
                    D7  18 |             | 23  /SYNC
                    D8  19 |             | 22  DIO
                   GND  20 |_____________| 21  /CS

**********************************************************************/

#ifndef MAME_MACHINE_I7220_H
#define MAME_MACHINE_I7220_H

#pragma once

#define I7110_MBM_SIZE (128 * 1024) // 1 megabit
#define I7115_MBM_SIZE (512 * 1024) // 4 megabit


///*************************************************************************
//  INTERFACE CONFIGURATION MACROS
///*************************************************************************

#define MCFG_I7220_IRQ_CALLBACK(_write) \
	devcb = &i7220_device::set_intrq_wr_callback(*device, DEVCB_##_write);

#define MCFG_I7220_DRQ_CALLBACK(_write) \
	devcb = &i7220_device::set_drq_wr_callback(*device, DEVCB_##_write);

#define MCFG_I7220_DATA_SIZE(data_size) \
	i7220_device::set_data_size(*device, data_size);



///*************************************************************************
//  TYPE DEFINITIONS
///*************************************************************************

// ======================> i7220_device

class i7220_device : public device_t,
					 public device_image_interface
{
public:
	// construction/destruction
	i7220_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_intrq_wr_callback(device_t &device, Object &&cb) { return downcast<i7220_device &>(device).intrq_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_drq_wr_callback(device_t &device, Object &&cb) { return downcast<i7220_device &>(device).drq_cb.set_callback(std::forward<Object>(cb)); }

	static void set_data_size(device_t &device, int data_size) { downcast<i7220_device &>(device).m_data_size = data_size; }

	// image-level overrides
	virtual image_init_result call_load() override;

	virtual iodevice_t image_type() const override { return IO_MEMCARD; }

	virtual bool is_readable()  const override { return 1; }
	virtual bool is_writeable() const override { return 1; }
	virtual bool is_creatable() const override { return 0; }
	virtual bool must_be_loaded() const override { return 1; }
	virtual bool is_reset_on_load() const override { return 0; }
	virtual const char *file_extensions() const override { return "bubble"; }

	DECLARE_READ8_MEMBER(read);
	DECLARE_WRITE8_MEMBER(write);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	int m_data_size;

private:
	enum {
		PHASE_IDLE, PHASE_CMD, PHASE_EXEC, PHASE_RESULT
	};

	enum {
		// General "doing nothing" state
		IDLE,

		// Main states
		INIT,
		READ_FSA,
		READ_DATA,
		WRITE_DATA,
		FAIL,

		// Sub-states
		INITIALIZE,

		WAIT_FSA_REPLY,
		WAIT_FSA_REPLY_DONE,

		WAIT_FIFO,
		WAIT_FIFO_DONE,

		COMMAND_DONE,

		SECTOR_READ,
		SECTOR_WRITTEN
	};

	enum {
		C_WRITE_BOOTLOOP_REG_MASKED,
		C_INIT,
		C_READ,
		C_WRITE,
		C_READ_SEEK,
		C_READ_BOOTLOOP_REG,
		C_WRITE_BOOTLOOP_REG,
		C_WRITE_BOOTLOOP,
		C_READ_FSA_STATUS,
		C_ABORT,
		C_WRITE_SEEK,
		C_READ_BOOTLOOP,
		C_READ_CORRECTED,
		C_RESET_FIFO,
		C_MBM_PURGE,
		C_RESET
	};

	enum {
		R_UR     = 0x0a,
		R_BLR_L  = 0x0b,
		R_BLR_M  = 0x0c,
		R_ER     = 0x0d,
		R_AR_L   = 0x0e,
		R_AR_M   = 0x0f,
	};

	enum {
		ER_IEN   = 0x01,
		ER_IEE   = 0x02,
		ER_DMAEN = 0x04,
		ER_MFBTR = 0x08,
		ER_WBE   = 0x10,
		ER_ERCD  = 0x20,
		ER_EICD  = 0x40,
		ER_EPI   = 0x80
	};

	enum {
		SR_FIFO  = 0x01,
		SR_PE    = 0x02,
		SR_UE    = 0x04,
		SR_CE    = 0x08,
		SR_TE    = 0x10,
		SR_FAIL  = 0x20,
		SR_DONE  = 0x40,
		SR_BUSY  = 0x80,

		SR_CLEAR = 0x7E
	};

	struct bubble_info {
		emu_timer *tm;
		int main_state, sub_state;
		int limit, counter;
	};

	void delay_cycles(emu_timer *tm, int cycles);
	void set_drq(bool state);
	void set_irq(bool state);

	void update_regs();
	void update_drq();

	void start_command(int cmd);
	void general_continue(bubble_info &bi);
	void command_end(bubble_info &bi, bool success);

	void command_fail_start(bubble_info &bi);
	void command_fail_continue(bubble_info &bi);

	void init_start(bubble_info &bi);
	void init_continue(bubble_info &bi);

	void read_fsa_start(bubble_info &bi);
	void read_fsa_continue(bubble_info &bi);

	void read_data_start(bubble_info &bi);
	void read_data_continue(bubble_info &bi);

	void write_data_start(bubble_info &bi);
	void write_data_continue(bubble_info &bi);

	void fifo_clear();
	void fifo_push(uint8_t val);
	uint8_t fifo_pop();

	int main_phase;
	bool drq, irq;
	bubble_info bi;

	uint8_t buf[32];
	int blr_count, blr_nfc, ar_addr, ar_mbm;

	devcb_write_line intrq_cb;
	devcb_write_line drq_cb;

	uint8_t m_regs[16];
	uint8_t m_rac;
	uint8_t m_cmdr;
	uint8_t m_str;
	uint16_t m_blr;
	uint16_t m_ar;
	int m_fifo_size;
	util::fifo<uint8_t, 40> m_fifo;
};


// device type definition
DECLARE_DEVICE_TYPE(I7220, i7220_device)

#endif // MAME_MACHINE_I7220_H
