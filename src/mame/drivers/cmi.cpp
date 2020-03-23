// license:BSD-3-Clause
// copyright-holders:Phil Bennett
/***************************************************************************

    Fairlight CMI Series

    driver by Phil Bennett

    Systems supported:

        * CMI IIx

    To do:

    * MASTER 'TIM' test fails
    * LGTST  'TIM' test is out of tolerance without 6840 hack.

    Information from:

             CMI SYSTEM SERVICE MANUAL
        FAIRLIGHT INSTRUMENTS, FEBRUARY 1985
                  Revision 2.1

    This document is available on archive.org at the following URL:

    https://archive.org/details/fairlight_CMI-IIx_SERVICE_MANUAL

    Summary:

        The Fairlight CMI system conists typically of:
            - One velocity-sensitive unweighted keyboard, with a numeric
              keypad and several control surfaces
            - (Optionally) one additional keyboard, not velocity-sensitive
            - One alphanumeric keyboard for manual control
            - A 15-inch green-screen monitor and light pen for more direct
              control
            - A box consisting of:
              * An audio board including balanced line drivers for eight
                channels and mixed output
              * A 500-watt power supply
              * A 21-slot backplane
              * Two 8-inch double-density floppy disk drives. The format
                used is soft-sectored, 128 bytes per sector (single density),
                or 256 bytes per sector (double density), using FM
                recording.
              * And the following cards:
                Slot  1: Master Card CMI-02
                Slot  2: General Interface Card CMI-08/28 (Optional)
                Slots 3-11: 8 Channel Controller Cards & 1 Voice Master Module Card, order unknown
                Slot 12: 64K System RAM Q-096
                Slot 13: 256K System RAM Q-256
                Slot 14: 256K System RAM Q-256
                Slot 15: 4-Port ACIA Module Q-014 (Optional)
                Slot 16: Processor Control Module Q-133
                Slot 17: Central Processor Module Q-209
                Slot 18: Lightpen/Graphics Interface Q-219
                Slot 19: Floppy Disk Controller QFC-9
                Slot 20: Hard Disk Controller Q-077 (Optional)

        The Master Keyboard
        -------------------

        The master keyboard has the following features:
            - A serial connector for communicating with the CMI mainframe
            - A connector for a slave keyboard
            - A connector for the alphanumeric keyboard
            - Connectors for pedal controls
            - Three slider-type analog controls
            - Two switch controls (one momentary, one toggle on/off)
            - Two lamp indicators for the switches with software-defined
              control
            - A 12-character LED alphanumeric display
            - A 16-switch keypad

        All communications with all peripherals and controls on the master
        keyboard is handled via the master keyboard's controller, and as
        such there is one single serial link to the "CMI mainframe" box
        itself.

        Q209 Dual 6809 Central Processor Card
        -------------------------------------

        The CPU card has two 6809 processors, with robust inter-CPU
        communications capabilities including:
            - Uninterruptible instructions
            - CPU-specific ID register and memory map registers
            - Interprocessor interrupts
            - Automatic memory map-switching register

        The CPUs are multiplexed onto the address and data buses
        in an interleaved manner such that there is no contention
        on simultaneous memory accesses.

        All system timing is derived from a 40MHz clock crystal, which
        is divided into two opposite-phase 20MHz squre waves.

        Other data entry from service manual to be completed later - RH 12 Aug 2016

****************************************************************************/

#include "emu.h"
#include "emu.h"

#include "cpu/m6800/m6800.h"
#include "cpu/m6809/m6809.h"
#include "cpu/m68000/m68000.h"

#include "machine/6821pia.h"
#include "machine/6840ptm.h"
#include "machine/6850acia.h"
#include "machine/7474.h"
#include "machine/clock.h"
#include "machine/i8214.h"
#include "machine/input_merger.h"
#include "machine/mos6551.h"
#include "machine/msm5832.h"
#include "machine/wd_fdc.h"

#include "video/dl1416.h"

#include "screen.h"
#include "speaker.h"


#define Q209_CPU_CLOCK      4000000 // ?

#define M6809_CLOCK             8000000 // wrong
#define MASTER_OSCILLATOR       34291712

#define CPU_1                   0
#define CPU_2                   1

#define MAPPING_A               1
#define MAPPING_B               0

#define NUM_Q256_CARDS          1   // Max of 2
#define NUM_CHANNEL_CARDS       8

#define PAGE_SIZE               2048
#define PAGE_COUNT              (65536 / PAGE_SIZE)
#define PAGE_MASK               (PAGE_SIZE - 1)
#define PAGE_SHIFT              5

#define PIXEL_CLOCK             10380000        // Add to xtal.h
#define HTOTAL                  672
#define HBLANK_END              0
#define HBLANK_START            512
#define VTOTAL                  304
#define VBLANK_END              0
#define VBLANK_START            256

#define HBLANK_FREQ     ((double)PIXEL_CLOCK / (double)HTOTAL)
#define VBLANK_FREQ     ((double)HBLANK_FREQ / (double)VTOTAL)

#define MAPSEL_P2_B             0x00
#define MAPSEL_P2_A             0x03
#define MAPSEL_P2_A_DMA1        0x04
#define MAPSEL_P2_A_DMA2        0x05
#define MAPSEL_P2_A_DMA3        0x06
#define MAPSEL_P2_A_DMA4        0x07
#define MAPSEL_P1_B             0x08
#define MAPSEL_P1_A             0x0b
#define MAPSEL_P1_A_DMA1        0x0c
#define MAPSEL_P1_A_DMA2        0x0d
#define MAPSEL_P1_A_DMA3        0x0e
#define MAPSEL_P1_A_DMA4        0x0f

#define IRQ_ACINT_LEVEL         (0 ^ 7)
#define IRQ_MIDINT_LEVEL        (0 ^ 7)
#define IRQ_TIMINT_LEVEL        (1 ^ 7)
#define IRQ_INTP1_LEVEL         (2 ^ 7)
#define IRQ_IPI1_LEVEL          (3 ^ 7)
#define IRQ_SMIDINT_LEVEL       (3 ^ 7)
#define IRQ_AIC_LEVEL           (4 ^ 7)

#define IRQ_CHINT8_LEVEL        (15 ^ 7)
#define IRQ_CHINT7_LEVEL        (14 ^ 7)
#define IRQ_CHINT6_LEVEL        (13 ^ 7)
#define IRQ_CHINT5_LEVEL        (12 ^ 7)
#define IRQ_CHINT4_LEVEL        (11 ^ 7)
#define IRQ_CHINT3_LEVEL        (10 ^ 7)
#define IRQ_CHINT2_LEVEL        (9 ^ 7)
#define IRQ_CHINT1_LEVEL        (8 ^ 7)

static const int ch_int_levels[8] =
{
	12, 8, 13, 9, 14, 10, 15, 11 //IRQ_CHINT8_LEVEL, IRQ_CHINT7_LEVEL, IRQ_CHINT6_LEVEL, IRQ_CHINT5_LEVEL, IRQ_CHINT4_LEVEL, IRQ_CHINT3_LEVEL, IRQ_CHINT2_LEVEL, IRQ_CHINT1_LEVEL
};

#define IRQ_PERRINT_LEVEL       (0 ^ 7)
#define IRQ_RTCINT_LEVEL        (0 ^ 7)
#define IRQ_RINT_LEVEL          (1 ^ 7)
#define IRQ_INTP2_LEVEL         (2 ^ 7)
#define IRQ_IPI2_LEVEL          (3 ^ 7)
#define IRQ_TOUCHINT_LEVEL      (4 ^ 7)
#define IRQ_PENINT_LEVEL        (5 ^ 7)
#define IRQ_ADINT_LEVEL         (6 ^ 7)
#define IRQ_DISKINT_LEVEL       (7 ^ 7)

#define FDC_CONTROL_INTEN       (1 << 2)

#define FDC_STATUS_READY        (1 << 3)
#define FDC_STATUS_TWO_SIDED    (1 << 4)
#define FDC_STATUS_DISK_CHANGE  (1 << 5)
#define FDC_STATUS_INTERRUPT    (1 << 6)
#define FDC_STATUS_DRIVER_LOAD  (1 << 7)


#define ENV_DIR_DOWN            0
#define ENV_DIR_UP              1

#define MCFG_CMI01A_ADD(_tag, _channel)  \
	MCFG_DEVICE_ADD(_tag, CMI01A_CHANNEL_CARD, 0) \
	cmi01a_device::set_channel_number(*device, _channel);

#define MCFG_CMI01A_CHANNEL_NUMBER(_channel) \
	cmi01a_device::set_channel_number(*device, _channel);

class cmi01a_device : public device_t, public device_sound_interface {
public:
	cmi01a_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void set_channel_number(device_t &device, int channel) { dynamic_cast<cmi01a_device&>(device).m_channel = channel; }

	DECLARE_WRITE8_MEMBER( write );
	DECLARE_READ8_MEMBER( read );

	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	static const device_timer_id TIMER_ZX = 0;

	required_device<pia6821_device> m_pia_0;
	required_device<pia6821_device> m_pia_1;
	required_device<ptm6840_device> m_ptm;

	required_device<pia6821_device> m_cmi02_pia_0;
	required_device<pia6821_device> m_cmi02_pia_1;

	sound_stream* m_stream;

private:
	void zx_timer_cb();
	void run_voice();
	void update_wave_addr(int inc);
	void update_interrupts();

	emu_timer * m_zx_timer;
	uint8_t       m_zx_flag;
	uint8_t       m_zx_ff;

	int     m_channel;
	std::unique_ptr<uint8_t[]>    m_wave_ram;
	uint16_t  m_segment_cnt;
	uint8_t   m_new_addr;     // Flag
	uint8_t   m_env_dir_ctrl;
	uint8_t   m_vol_latch;
	uint8_t   m_flt_latch;
	uint8_t m_rp;
	uint8_t m_ws;
	int     m_dir;

	double  m_freq;
	bool    m_active;

	int     m_ptm_out0;

	int     m_pia_0_irqa;
	int     m_pia_0_irqb;
	int     m_pia_1_irqa;
	int     m_pia_1_irqb;
	int     m_ptm_irq;
	int     m_irq_state;

	DECLARE_WRITE8_MEMBER( rp_w );
	DECLARE_WRITE8_MEMBER( ws_dir_w );
	DECLARE_READ_LINE_MEMBER( tri_r );
	DECLARE_WRITE_LINE_MEMBER( pia_0_ca2_w );
	DECLARE_WRITE_LINE_MEMBER( pia_0_cb2_w );
	DECLARE_WRITE_LINE_MEMBER( pia_0_irqa );
	DECLARE_WRITE_LINE_MEMBER( pia_0_irqb );

	DECLARE_READ_LINE_MEMBER( eosi_r );
	DECLARE_READ_LINE_MEMBER( zx_r );
	DECLARE_WRITE8_MEMBER( pia_1_a_w );
	DECLARE_WRITE8_MEMBER( pia_1_b_w );
	DECLARE_WRITE_LINE_MEMBER( pia_1_irqa );
	DECLARE_WRITE_LINE_MEMBER( pia_1_irqb );

	DECLARE_WRITE_LINE_MEMBER( ptm_irq );
	DECLARE_WRITE_LINE_MEMBER( ptm_out0 );
};

DEFINE_DEVICE_TYPE(CMI01A_CHANNEL_CARD, cmi01a_device, "cmi_01a", "Fairlight CMI-01A Channel Card")

cmi01a_device::cmi01a_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, CMI01A_CHANNEL_CARD, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_pia_0(*this, "cmi01a_pia_0")
	, m_pia_1(*this, "cmi01a_pia_1")
	, m_ptm(*this, "cmi01a_ptm")
	, m_cmi02_pia_0(*this, "^cmi02_pia_1")
	, m_cmi02_pia_1(*this, "^cmi02_pia_2")
	, m_stream(nullptr)
{
}

MACHINE_CONFIG_MEMBER( cmi01a_device::device_add_mconfig )
	MCFG_DEVICE_ADD("cmi01a_pia_0", PIA6821, 0) // pia_cmi01a_1_config
	MCFG_PIA_READCB1_HANDLER(READLINE(cmi01a_device, tri_r))
	MCFG_PIA_WRITEPA_HANDLER(WRITE8(cmi01a_device, ws_dir_w))
	MCFG_PIA_WRITEPB_HANDLER(WRITE8(cmi01a_device, rp_w))
	MCFG_PIA_CA2_HANDLER(WRITELINE(cmi01a_device, pia_0_ca2_w))
	MCFG_PIA_CB2_HANDLER(WRITELINE(cmi01a_device, pia_0_cb2_w))
	MCFG_PIA_IRQA_HANDLER(WRITELINE(cmi01a_device, pia_0_irqa))
	MCFG_PIA_IRQB_HANDLER(WRITELINE(cmi01a_device, pia_0_irqb))

	MCFG_DEVICE_ADD("cmi01a_pia_1", PIA6821, 0) // pia_cmi01a_2_config
	MCFG_PIA_READCA1_HANDLER(READLINE(cmi01a_device, zx_r))
	MCFG_PIA_READCA2_HANDLER(READLINE(cmi01a_device, eosi_r))
	MCFG_PIA_WRITEPA_HANDLER(WRITE8(cmi01a_device, pia_1_a_w))
	MCFG_PIA_WRITEPB_HANDLER(WRITE8(cmi01a_device, pia_1_b_w))
	MCFG_PIA_IRQA_HANDLER(WRITELINE(cmi01a_device, pia_1_irqa))
	MCFG_PIA_IRQB_HANDLER(WRITELINE(cmi01a_device, pia_1_irqb))

	MCFG_DEVICE_ADD("cmi01a_ptm", PTM6840, 2000000) // ptm_cmi01a_config
	MCFG_PTM6840_EXTERNAL_CLOCKS(250000, 500000, 500000)
	MCFG_PTM6840_OUT0_CB(WRITELINE(cmi01a_device, ptm_out0))
	MCFG_PTM6840_IRQ_CB(WRITELINE(cmi01a_device, ptm_irq))
MACHINE_CONFIG_END


void cmi01a_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	if (m_active && m_vol_latch)
	{
		int length = samples;
		int seg_addr = m_segment_cnt & 0x7f;
		uint8_t *wave_ptr = &m_wave_ram[m_segment_cnt & 0x3fff];
		stream_sample_t *buf = outputs[0];

		while (length--)
		{
			*buf++ = wave_ptr[seg_addr];
			seg_addr = (seg_addr + 1) & 0x7f;
		}

		m_segment_cnt = (m_segment_cnt & ~0x7f) | seg_addr;
	}
	else
	{
		memset(outputs[0], 0, samples);
	}
}

void cmi01a_device::device_start()
{
	m_wave_ram = std::make_unique<uint8_t[]>(0x4000);

	m_zx_timer = timer_alloc(TIMER_ZX);
	m_zx_timer->adjust(attotime::never);

	m_stream = stream_alloc(0, 1, 44100);
}

void cmi01a_device::device_reset()
{
	m_ptm->set_g1(1);
	m_ptm->set_g2(1);
	m_ptm->set_g3(1);

	m_pia_0_irqa = 0;
	m_pia_0_irqb = 0;
	m_pia_1_irqa = 0;
	m_pia_1_irqb = 0;
	m_ptm_irq = 0;
	m_irq_state = 0;

	m_segment_cnt = 0;
	m_new_addr = 0;
	m_env_dir_ctrl = 0;
	m_vol_latch = 0;
	m_flt_latch = 0;
	m_rp = 0;
	m_ws = 0;
	m_dir = 0;

	m_freq = 0.0;
	m_active = false;

	m_ptm_out0 = 0;
}

class cmi_state : public driver_device
{
public:
	cmi_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu1(*this, "maincpu1")
		, m_maincpu2(*this, "maincpu2")
		, m_muskeyscpu(*this, "muskeys")
		, m_alphakeyscpu(*this, "alphakeys")
		, m_midicpu(*this, "smptemidi")
		, m_cmi07cpu(*this, "cmi07cpu")
		, m_msm5832(*this, "msm5832")
		, m_i8214_0(*this, "i8214_1")
		, m_i8214_1(*this, "i8214_2")
		, m_i8214_2(*this, "i8214_3")
		, m_q133_pia_0(*this, "q133_pia_1")
		, m_q133_pia_1(*this, "q133_pia_2")
		, m_q133_ptm(*this, "q133_ptm")
		, m_q133_acia_0(*this, "q133_acia_0")
		, m_q133_acia_1(*this, "q133_acia_1")
		, m_q133_acia_2(*this, "q133_acia_2")
		, m_q133_acia_3(*this, "q133_acia_3")
		, m_q133_region(*this, "q133")
		, m_q219_pia(*this, "q219_pia")
		, m_q219_ptm(*this, "q219_ptm")
		, m_cmi02_pia_0(*this, "cmi02_pia_1")
		, m_cmi02_pia_1(*this, "cmi02_pia_2")
		, m_cmi02_ptm(*this, "cmi02_ptm")
		, m_ank_pia(*this, "ank_pia")
		, m_acia_mkbd_kbd(*this, "acia_mkbd_kbd")
		, m_acia_mkbd_cmi(*this, "acia_mkbd_cmi")
		, m_cmi07_ptm(*this, "cmi07_ptm")
		, m_qfc9_region(*this, "qfc9")
		, m_floppy0(*this, "wd1791:0")
		, m_floppy1(*this, "wd1791:1")
		, m_wd1791(*this, "wd1791")
		, m_channels(*this, "cmi01a_%u", 0)
		, m_cmi10_pia_u20(*this, "cmi10_pia_u20")
		, m_cmi10_pia_u21(*this, "cmi10_pia_u21")
		, m_dp1(*this, "dp1")
		, m_dp2(*this, "dp2")
		, m_dp3(*this, "dp3")
		, m_screen(*this, "screen")
		, m_palette(*this, "palette")
		, m_ankrow_ports(*this, "ROW%u", 0)
		, m_lp_x_port(*this, "LP_X")
		, m_lp_y_port(*this, "LP_Y")
		, m_lp_touch_port(*this, "LP_TOUCH")
		, m_keypad_a_port(*this, "KEYPAD_A")
		, m_keypad_b_port(*this, "KEYPAD_B")
		, m_key_mux_ports{ { *this, "KEY_%u_0", 0 }, { *this, "KEY_%u_1", 0 }, { *this, "KEY_%u_2", 0 }, { *this, "KEY_%u_3", 0 } }
		, m_cmi07_ram(*this, "cmi07_ram")
	{
	}

	void set_interrupt(int cpunum, int level, int state);

	virtual void machine_reset() override;
	virtual void machine_start() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	static const device_timer_id TIMER_MAP_SWITCH = 0;
	static const device_timer_id TIMER_HBLANK = 1;
	static const device_timer_id TIMER_JAM_TIMEOUT = 2;
	static const device_timer_id TIMER_CMI10_SCND = 3;

	DECLARE_DRIVER_INIT( cmi2x );

	// CPU card
	DECLARE_WRITE_LINE_MEMBER( q133_acia_irq0 );
	DECLARE_WRITE_LINE_MEMBER( q133_acia_irq1 );
	DECLARE_WRITE_LINE_MEMBER( q133_acia_irq2 );
	DECLARE_WRITE_LINE_MEMBER( q133_acia_irq3 );
	DECLARE_WRITE8_MEMBER( i8214_cpu1_w );
	DECLARE_WRITE8_MEMBER( i8214_cpu2_w );
	DECLARE_WRITE_LINE_MEMBER( i8214_1_int_w );
	DECLARE_WRITE_LINE_MEMBER( i8214_2_int_w );
	DECLARE_WRITE_LINE_MEMBER( i8214_3_int_w );
	DECLARE_WRITE_LINE_MEMBER( i8214_3_enlg );
	DECLARE_READ8_MEMBER( shared_ram_r );
	DECLARE_WRITE8_MEMBER( shared_ram_w );

	DECLARE_READ8_MEMBER( q133_1_porta_r );
	DECLARE_WRITE8_MEMBER( q133_1_porta_w );
	DECLARE_WRITE8_MEMBER( q133_1_portb_w );

	INTERRUPT_GEN_MEMBER( cmi_iix_vblank );
	IRQ_CALLBACK_MEMBER( cpu1_interrupt_callback );
	IRQ_CALLBACK_MEMBER( cpu2_interrupt_callback );

	// Video-related
	DECLARE_READ8_MEMBER( video_r );
	DECLARE_READ8_MEMBER( lightpen_r );
	DECLARE_READ8_MEMBER( pia_q219_b_r );
	DECLARE_WRITE8_MEMBER( video_w );
	DECLARE_WRITE8_MEMBER( vscroll_w );
	DECLARE_WRITE8_MEMBER( video_attr_w );
	DECLARE_READ8_MEMBER( vram_r );
	DECLARE_WRITE8_MEMBER( vram_w );
	DECLARE_WRITE_LINE_MEMBER( pia_q219_irqa );
	DECLARE_WRITE_LINE_MEMBER( pia_q219_irqb );
	DECLARE_WRITE_LINE_MEMBER( ptm_q219_irq );
	uint32_t screen_update_cmi2x(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	// Memory mapping
	DECLARE_READ8_MEMBER( rom_r );
	DECLARE_WRITE8_MEMBER( map_ram_w );
	DECLARE_READ8_MEMBER( vector_r );
	DECLARE_READ8_MEMBER( map_r );
	DECLARE_WRITE8_MEMBER( map_w );
	DECLARE_READ8_MEMBER( atomic_r );
	DECLARE_WRITE8_MEMBER( cpufunc_w );
	DECLARE_READ8_MEMBER( parity_r );
	DECLARE_WRITE8_MEMBER( mapsel_w );
	DECLARE_READ8_MEMBER( irq_ram_r );
	DECLARE_WRITE8_MEMBER( irq_ram_w );

	// MIDI/SMPTE
	DECLARE_WRITE16_MEMBER( midi_dma_w );
	DECLARE_READ16_MEMBER( midi_dma_r );

	// Floppy
	DECLARE_WRITE8_MEMBER( fdc_w );
	DECLARE_READ8_MEMBER( fdc_r );
	DECLARE_WRITE_LINE_MEMBER( wd1791_irq );
	DECLARE_WRITE_LINE_MEMBER( wd1791_drq );

	// Master card
	DECLARE_READ8_MEMBER( cmi02_r );
	DECLARE_WRITE8_MEMBER( cmi02_w );
	DECLARE_WRITE8_MEMBER( master_tune_w );
	DECLARE_WRITE_LINE_MEMBER( cmi02_ptm_irq );
	DECLARE_WRITE_LINE_MEMBER( cmi02_ptm_o1 );

	// Alphanumeric keyboard
	DECLARE_READ8_MEMBER( ank_col_r );
	DECLARE_READ_LINE_MEMBER( ank_rts_r );
	DECLARE_WRITE_LINE_MEMBER( ank_irqa_w );
	DECLARE_WRITE_LINE_MEMBER( ank_irqb_w );

	// ???
	DECLARE_READ8_MEMBER( cmi07_r );
	DECLARE_WRITE8_MEMBER( cmi07_w );

	// Music keyboard/alphanumeric display/keypad
	DECLARE_WRITE8_MEMBER( cmi10_u20_a_w );
	DECLARE_WRITE8_MEMBER( cmi10_u20_b_w );
	DECLARE_READ_LINE_MEMBER( cmi10_u20_cb1_r );
	DECLARE_WRITE_LINE_MEMBER( cmi10_u20_cb2_w );
	DECLARE_WRITE_LINE_MEMBER( cmi10_u21_cb2_w );
	DECLARE_READ8_MEMBER( cmi10_u21_a_r );
	DECLARE_WRITE16_MEMBER( cmi_iix_update_dp1 );
	DECLARE_WRITE16_MEMBER( cmi_iix_update_dp2 );
	DECLARE_WRITE16_MEMBER( cmi_iix_update_dp3 );

	DECLARE_WRITE_LINE_MEMBER( msm5832_irq );
	DECLARE_WRITE_LINE_MEMBER( mkbd_kbd_acia_int );
	DECLARE_WRITE_LINE_MEMBER( mkbd_cmi_acia_int );
	DECLARE_WRITE_LINE_MEMBER( cmi07_irq );
	DECLARE_WRITE_LINE_MEMBER( mkbd_acia_clock );

protected:

	required_device<m6809e_device> m_maincpu1;
	required_device<m6809e_device> m_maincpu2;
	required_device<m6802_cpu_device> m_muskeyscpu;
	required_device<m6802_cpu_device> m_alphakeyscpu;
	required_device<m68000_device> m_midicpu;
	required_device<m6809e_device> m_cmi07cpu;

	required_device<msm5832_device> m_msm5832;
	required_device<i8214_device> m_i8214_0;
	required_device<i8214_device> m_i8214_1;
	required_device<i8214_device> m_i8214_2;
	required_device<pia6821_device> m_q133_pia_0;
	required_device<pia6821_device> m_q133_pia_1;
	required_device<ptm6840_device> m_q133_ptm;
	required_device<mos6551_device> m_q133_acia_0;
	required_device<mos6551_device> m_q133_acia_1;
	required_device<mos6551_device> m_q133_acia_2;
	required_device<mos6551_device> m_q133_acia_3;
	required_memory_region m_q133_region;

	required_device<pia6821_device> m_q219_pia;
	required_device<ptm6840_device> m_q219_ptm;

	required_device<pia6821_device> m_cmi02_pia_0;
	required_device<pia6821_device> m_cmi02_pia_1;
	required_device<ptm6840_device> m_cmi02_ptm;

	required_device<pia6821_device> m_ank_pia;
	required_device<acia6850_device> m_acia_mkbd_kbd;
	required_device<acia6850_device> m_acia_mkbd_cmi;

	required_device<ptm6840_device> m_cmi07_ptm;

	required_memory_region m_qfc9_region;
	required_device<floppy_connector> m_floppy0;
	required_device<floppy_connector> m_floppy1;
	required_device<fd1791_device> m_wd1791;

	required_device_array<cmi01a_device, 8> m_channels;

	required_device<pia6821_device> m_cmi10_pia_u20;
	required_device<pia6821_device> m_cmi10_pia_u21;
	required_device<dl1416_device> m_dp1;
	required_device<dl1416_device> m_dp2;
	required_device<dl1416_device> m_dp3;

	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;

	required_ioport_array<8> m_ankrow_ports;

	required_ioport m_lp_x_port;
	required_ioport m_lp_y_port;
	required_ioport m_lp_touch_port;

	required_ioport m_keypad_a_port;
	required_ioport m_keypad_b_port;

	required_ioport_array<3> m_key_mux_ports[4];

	required_shared_ptr<uint8_t> m_cmi07_ram;

	address_space *m_cpu1space;
	address_space *m_cpu2space;

private:

	emu_timer *m_map_switch_timer;
	emu_timer *m_hblank_timer;
	emu_timer *m_cmi10_scnd_timer;
	emu_timer *m_jam_timeout_timer;

	uint8_t m_video_data;

	// Memory
	bool map_is_active(int cpunum, int map, uint8_t *map_info);
	void update_address_space(int cpunum, uint8_t mapinfo);
	void install_video_ram(int cpunum);
	void install_peripherals(int cpunum);

	// Video
	void hblank();
	void update_video_pos(int y, int x, int byte_size);
	void video_write(int offset);

	// Floppy
	void dma_fdc_rom();
	void write_fdc_ctrl(uint8_t data);
	void fdc_dma_transfer();

	// Q133 CPU Card
	uint8_t *m_q133_rom;
	uint8_t m_q133_acia_irq;

	uint8_t   m_hp_int;
	std::unique_ptr<uint8_t[]>    m_shared_ram;
	std::unique_ptr<uint8_t[]>    m_scratch_ram[2];

	/* Memory management */
	uint8_t   m_map_sel[16];
	std::unique_ptr<uint8_t[]>    m_map_ram[2];
	std::unique_ptr<uint8_t[]>    m_q256_ram[2];
	uint8_t   m_map_ram_latch;
	int     m_cpu_active_space[2]; // TODO: Make one register
	int     m_cpu_map_switch[2];
	uint8_t   m_irq_address[2][2];
	int     m_m6809_bs_hack_cnt;
	int     m_m6809_bs_hack_cpu;

	/* Q219 lightpen/graphics card */
	std::unique_ptr<uint8_t[]>    m_video_ram;
	uint16_t  m_x_pos;
	uint8_t   m_y_pos;
	uint16_t  m_lp_x;
	uint8_t   m_lp_y;
	uint8_t   m_q219_b_touch;

	/* QFC9 floppy disk controller card */
	uint8_t * m_qfc9_region_ptr;
	int     m_fdc_drq;
	uint8_t   m_fdc_addr;
	uint8_t   m_fdc_ctrl;
	uint8_t   m_fdc_status;
	PAIR    m_fdc_dma_addr;
	PAIR    m_fdc_dma_cnt;

	/* CMI-07 */
	uint8_t   m_cmi07_ctrl;

	/* CMI-10 */
	uint8_t   m_scnd;

	/* Musical keyboard */
	uint8_t   m_msm5832_addr;
	int     m_mkbd_kbd_acia_irq;
	int     m_mkbd_cmi_acia_irq;

	// Alphanumeric keyboard
	int     m_ank_irqa;
	int     m_ank_irqb;

	// Master card (CMI-02)
	int     m_cmi02_ptm_irq;
};

/**************************************
 *
 *  Video hardware
 *
 *************************************/

uint32_t cmi_state::screen_update_cmi2x(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	const pen_t *pen = m_palette->pens();
	uint8_t y_scroll = m_q219_pia->a_output();
	uint8_t invert = (!BIT(m_q219_pia->b_output(), 3)) & 1;

	for (int y = cliprect.min_y; y <= cliprect.max_y; ++y)
	{
		uint8_t *src = &m_video_ram[(512/8) * ((y + y_scroll) & 0xff)];
		uint32_t *dest = &bitmap.pix32(y, cliprect.min_x);

		for (int x = cliprect.min_x; x <= cliprect.max_x; x += 8)
		{
			uint8_t data = *src++;

			/* Store 8 pixels */
			for (int i = 0; i < 8; ++i)
				*dest++ = pen[BIT(data, 7 - i) ^ invert];
		}
	}

	/* Get lightpen position */
	//if (LPCEN && NOT_TOUCHING)
	if (m_lp_touch_port->read() && BIT(m_q219_pia->b_output(), 1))
	{
		/* Invert target pixel */
		bitmap.pix32(m_lp_y_port->read(), m_lp_x_port->read()) ^= 0x00ffffff;
	}



	return 0;
}

void cmi_state::hblank()
{
	int v = m_screen->vpos();

	if (!m_screen->vblank())
	{
		int _touch = m_lp_touch_port->read();
		int _tfh = !BIT(m_q219_pia->b_output(), 2);

		if (v == m_lp_y_port->read())
		{
			if (_touch)
				m_q219_b_touch = 0;
			else
				m_q219_b_touch = 1 << 5;

			m_q219_pia->ca1_w(!_touch ? 1 : 0);

			if (!_touch || !_tfh)
			{
				/* Latch the counters */
				m_lp_x = m_lp_x_port->read();
				m_lp_y = m_lp_y_port->read();

				/* LPSTB */
				m_q219_pia->cb1_w(1);
			}
		}
	}
	/* Adjust for next scanline */
	if (++v >= VTOTAL)
		v = 0;

	m_hblank_timer->adjust(m_screen->time_until_pos(v, HBLANK_START));

}

void cmi_state::update_video_pos(int y, int x, int byte_size)
{
	uint8_t *video_addr = &m_video_ram[m_y_pos * (512 / 8) + (m_x_pos / 8)];

	if (byte_size)
	{
		*video_addr = m_video_data;
	}
	else
	{
		int bit_mask = 1 << ((7 ^ m_x_pos) & 7);

		*video_addr &= ~bit_mask;
		*video_addr |= m_video_data & bit_mask;
	}

	if (y > 0)
		m_y_pos = (m_y_pos + 1) & 0xff;
	else if (y < 0)
		m_y_pos = (m_y_pos - 1) & 0xff;

	if (x > 0)
		m_x_pos = (m_x_pos + 1) & 0x1ff;
	else if (x < 0)
		m_x_pos = (m_x_pos - 1) & 0x1ff;
}

void cmi_state::video_write(int offset)
{
	switch (offset)
	{
		case 0x0: update_video_pos( 0,  0, 0); break;
		case 0x1: update_video_pos( 0,  1, 0); break;
		case 0x2: update_video_pos( 0, -1, 0); break;
		case 0x3: update_video_pos( 0,  0, 1); break;
		case 0x4: update_video_pos( 1,  0, 0); break;
		case 0x5: update_video_pos( 1,  1, 0); break;
		case 0x6: update_video_pos( 1, -1, 0); break;
		case 0x7: update_video_pos( 1,  0, 1); break;
		case 0x8: update_video_pos(-1,  0, 0); break;
		case 0x9: update_video_pos(-1,  1, 0); break;
		case 0xa: update_video_pos(-1, -1, 0); break;
		case 0xb: update_video_pos(-1,  0, 1); break;
//      default: printf("Video Write %x %x\n", offset, m_video_data);
	}
}

READ8_MEMBER( cmi_state::video_r )
{
	if (machine().side_effect_disabled())
		return m_video_data;

	m_video_data = m_video_ram[m_y_pos * (512 / 8) + (m_x_pos / 8)];

	video_write(offset);
	return m_video_data;
}

READ8_MEMBER( cmi_state::lightpen_r )
{
	if (offset & 2)
		return m_lp_y;
	else
		return m_lp_x >> 1;
}

READ8_MEMBER( cmi_state::pia_q219_b_r )
{
	return ((m_lp_x << 7) & 0x80) | m_q219_b_touch;
}


WRITE8_MEMBER( cmi_state::video_w )
{
	m_video_data = data;
	video_write(offset);
}

WRITE8_MEMBER( cmi_state::vscroll_w )
{
	// TODO: Partial updates. Also, this should be done through a PIA
}

WRITE8_MEMBER( cmi_state::video_attr_w )
{
	// TODO
}

WRITE8_MEMBER( cmi_state::vram_w )
{
	m_video_ram[offset] = data;
}

READ8_MEMBER( cmi_state::vram_r )
{
	if (machine().side_effect_disabled())
		return m_video_ram[offset];

	/* Latch the current video position */
	m_y_pos = (offset >> 6) & 0xff;
	m_x_pos = (offset & 0x3f) << 3;

	return m_video_ram[offset];
}



/* Memory handling */

READ8_MEMBER( cmi_state::rom_r )
{
	uint16_t base = (&space == m_cpu2space ? 0x1000 : 0x2000);
	return *(((uint8_t *)m_q133_region->base()) + base + offset);
}

WRITE8_MEMBER( cmi_state::map_ram_w )
{
	if ((offset & 1) == 0)
	{
		m_map_ram_latch = data;
	}
	else
	{
		for (int i = 0; i < NUM_Q256_CARDS; ++i)
		{
			uint8_t map_info;
			int map = (offset >> 6);
			int page_enable = ((m_map_ram_latch & 0x80) && (i == (m_map_ram_latch & 7))) ? 0x80 : 0;

			m_map_ram[i][offset >> 1] = page_enable | (data & 0x7f);

			/* Determine if this map is in use by either CPU */
			if (map_is_active(CPU_1, map, &map_info))
				update_address_space(0, map_info);

			if (map_is_active(CPU_2, map, &map_info))
				update_address_space(1, map_info);
		}
	}
}

READ8_MEMBER( cmi_state::vector_r )
{
	return m_q133_rom[((&space.device() == m_maincpu2) ? 0xbfe : 0xffe) + offset];
}

READ8_MEMBER( cmi_state::map_r )
{
	int cpunum = (&space.device() == m_maincpu1) ? 0 : 1;
	uint8_t data = (m_cpu_active_space[1] << 2) | (m_cpu_active_space[0] << 1) | cpunum;
	return data;
}

WRITE8_MEMBER( cmi_state::map_w )
{
	int cpunum = (&space.device() == m_maincpu1) ? 0 : 1;

	m_map_switch_timer->adjust(attotime::from_ticks(data & 0xf, M6809_CLOCK), cpunum);
}

READ8_MEMBER( cmi_state::irq_ram_r )
{
	int cpunum = (&space.device() == m_maincpu1) ? 0 : 1;

	if (machine().side_effect_disabled())
		return m_scratch_ram[cpunum][0xf8 + offset];

	if (m_m6809_bs_hack_cnt > 0 && m_m6809_bs_hack_cpu == cpunum)
	{
		m_m6809_bs_hack_cnt--;
		return m_irq_address[cpunum][offset];
	}
	return m_scratch_ram[cpunum][0xf8 + offset];
}

WRITE8_MEMBER( cmi_state::irq_ram_w )
{
	int cpunum = (&space.device() == m_maincpu1) ? 0 : 1;
	m_scratch_ram[cpunum][0xf8 + offset] = data;
}

void cmi_state::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	switch (id)
	{
		case TIMER_MAP_SWITCH:
		{
			m_cpu_active_space[param] = m_cpu_map_switch[param];
			uint8_t map_info = (m_cpu_map_switch[param] == MAPPING_A) ?
							 m_map_sel[param ? MAPSEL_P2_A : MAPSEL_P1_A] :
							 m_map_sel[param ? MAPSEL_P2_B : MAPSEL_P1_B];
			update_address_space(param, map_info);
			m_map_switch_timer->adjust(attotime::never);
			break;
		}

		case TIMER_HBLANK:
			hblank();
			break;

		case TIMER_JAM_TIMEOUT:
			m_maincpu2->set_input_line(INPUT_LINE_HALT, CLEAR_LINE);
			m_jam_timeout_timer->adjust(attotime::never);
			break;

		case TIMER_CMI10_SCND:
			m_cmi10_pia_u20->ca1_w(m_scnd);
			m_scnd ^= 1;
			m_cmi10_pia_u21->ca1_w(m_scnd);
			break;
	}
}

READ8_MEMBER( cmi_state::atomic_r )
{
	// TODO
	//printf("atomic access\n");
	return 0;
}

WRITE8_MEMBER( cmi_state::cpufunc_w )
{
	int cpunum = data & 1;
	int idx = data & 6;
	int bit = (data & 8) >> 3;

	switch (idx)
	{
		case 0: set_interrupt(cpunum, IRQ_IPI2_LEVEL, bit ? ASSERT_LINE : CLEAR_LINE);
				break;
		case 2: // TODO: Hardware trace
				printf("TODO: Hardware trace %02x\n", data);
				break;
		case 4: m_cpu_map_switch[cpunum] = bit;
				break;
		case 6: if (cpunum == CPU_1)
					m_maincpu1->set_input_line(M6809_FIRQ_LINE, bit ? ASSERT_LINE : CLEAR_LINE);
				else
					m_maincpu2->set_input_line(M6809_FIRQ_LINE, bit ? ASSERT_LINE : CLEAR_LINE);
				break;
	}
}

READ8_MEMBER( cmi_state::parity_r )
{
	//printf("parity_r %04x\n", offset);
	// TODO
	return 0xff;
}

WRITE8_MEMBER( cmi_state::mapsel_w )
{
	data ^= 0x1f;
	m_map_sel[offset] = data;

	if ((offset == MAPSEL_P1_A) && (m_cpu_active_space[0] == MAPPING_A))
		update_address_space(0, data);
	else if ((offset == MAPSEL_P1_B) && (m_cpu_active_space[0] == MAPPING_B))
		update_address_space(0, data);

	if ((offset == MAPSEL_P2_A) && (m_cpu_active_space[1] == MAPPING_A))
		update_address_space(1, data);
	else if ((offset == MAPSEL_P2_B) && (m_cpu_active_space[1] == MAPPING_B))
		update_address_space(1, data);
}



WRITE16_MEMBER( cmi_state::midi_dma_w )
{
	address_space *cmi_space = ((offset & 0x8000) ? m_cpu2space : m_cpu1space);
	offset &= 0x7fff;

	if (ACCESSING_BITS_0_7)
		cmi_space->write_byte(offset * 2 + 1, data);
	if (ACCESSING_BITS_8_15)
		cmi_space->write_byte(offset * 2, data >> 8);
}

READ16_MEMBER( cmi_state::midi_dma_r )
{
	address_space *cmi_space = ((offset & 0x8000) ? m_cpu2space : m_cpu1space);
	offset &= 0x7fff;
	return cmi_space->read_word(offset * 2);
}

/* The maps are dynamically populated */
static ADDRESS_MAP_START( maincpu1_map, AS_PROGRAM, 8, cmi_state )
	AM_RANGE(0xfffe, 0xffff) AM_READ(vector_r)
ADDRESS_MAP_END

static ADDRESS_MAP_START( maincpu2_map, AS_PROGRAM, 8, cmi_state )
	AM_RANGE(0xfffe, 0xffff) AM_READ(vector_r)
ADDRESS_MAP_END

static ADDRESS_MAP_START( muskeys_map, AS_PROGRAM, 8, cmi_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x007f) AM_RAM
	AM_RANGE(0x0080, 0x0083) AM_DEVREADWRITE("cmi10_pia_u21", pia6821_device, read, write)
	AM_RANGE(0x0090, 0x0093) AM_DEVREADWRITE("cmi10_pia_u20", pia6821_device, read, write)
	AM_RANGE(0x00a0, 0x00a0) AM_DEVREADWRITE("acia_mkbd_kbd", acia6850_device, status_r, control_w)
	AM_RANGE(0x00a1, 0x00a1) AM_DEVREADWRITE("acia_mkbd_kbd", acia6850_device, data_r, data_w)
	AM_RANGE(0x00b0, 0x00b0) AM_DEVREADWRITE("acia_mkbd_cmi", acia6850_device, status_r, control_w)
	AM_RANGE(0x00b1, 0x00b1) AM_DEVREADWRITE("acia_mkbd_cmi", acia6850_device, data_r, data_w)
	AM_RANGE(0x4000, 0x47ff) AM_RAM
	AM_RANGE(0xb000, 0xb400) AM_ROM
	AM_RANGE(0xf000, 0xffff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START( alphakeys_map, AS_PROGRAM, 8, cmi_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x007f) AM_RAM
	AM_RANGE(0x4000, 0x7fff) AM_READ_PORT("ANK_OPTIONS")
	AM_RANGE(0x8000, 0xbfff) AM_DEVREADWRITE("ank_pia", pia6821_device, read, write)
	AM_RANGE(0xc000, 0xc3ff) AM_ROM AM_MIRROR(0x3c00)
ADDRESS_MAP_END

static ADDRESS_MAP_START( midicpu_map, AS_PROGRAM, 16, cmi_state )
	AM_RANGE(0x000000, 0x003fff) AM_ROM
	AM_RANGE(0x040000, 0x05ffff) AM_READWRITE(midi_dma_r, midi_dma_w)
//  AM_RANGE(0x060000, 0x06001f) TIMERS
//  AM_RANGE(0x060050, 0x06005f) ACIA
//  AM_RANGE(0x060070, 0x06007f) SMPTE
	AM_RANGE(0x080000, 0x083fff) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START( cmi07cpu_map, AS_PROGRAM, 8, cmi_state )
	AM_RANGE(0x0000, 0x3fff) AM_NOP // TODO
	AM_RANGE(0x4000, 0x4fff) AM_NOP // TODO
	AM_RANGE(0x8000, 0x8fff) AM_DEVREADWRITE("cmi07_ptm", ptm6840_device, read, write)
	AM_RANGE(0xc000, 0xffff) AM_RAM AM_SHARE("cmi07_ram")
ADDRESS_MAP_END

/* Input ports */
static INPUT_PORTS_START( cmi2x )
	PORT_START("LP_X")
	PORT_BIT( 0xffff, HBLANK_START/2, IPT_LIGHTGUN_X) PORT_NAME ("Lightpen X") PORT_MINMAX(0, HBLANK_START - 1) PORT_SENSITIVITY(50) PORT_CROSSHAIR(X, 1.0, 0.0, 0)

	PORT_START("LP_Y")
	PORT_BIT( 0xffff, VBLANK_START/2, IPT_LIGHTGUN_Y) PORT_NAME ("Lightpen Y") PORT_MINMAX(0, VBLANK_START - 1) PORT_SENSITIVITY(50) PORT_CROSSHAIR(Y, 1.0, 0.0, 0)

	PORT_START("LP_TOUCH")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_NAME ( "Lightpen Touch" ) PORT_CODE( MOUSECODE_BUTTON1 )

	/* Alphanumeric keyboard */
	PORT_START("ANK_OPTIONS")
	PORT_DIPNAME( 0x07, 0x00, "Speed (baud)" )
	PORT_DIPSETTING(    0x00, "9600" )
	PORT_DIPSETTING(    0x01, "4800" )
	PORT_DIPSETTING(    0x02, "2400" )
	PORT_DIPSETTING(    0x03, "1200" )
	PORT_DIPSETTING(    0x04, "600"  )
	PORT_DIPSETTING(    0x05, "300"  )
	PORT_DIPSETTING(    0x06, "150"  )
	PORT_DIPSETTING(    0x07, "110"  )

	PORT_DIPNAME( 0x30, 0x20, "Parity" )
	PORT_DIPSETTING(    0x00, "Even" )
	PORT_DIPSETTING(    0x10, "None, bit 7 is 0" )
	PORT_DIPSETTING(    0x20, "Odd" )
	PORT_DIPSETTING(    0x30, "None, bit 7 is 1" )

	PORT_START("ROW0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_2)                PORT_CHAR('2')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_4)                PORT_CHAR('4')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_6)                PORT_CHAR('6')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_8)                PORT_CHAR('8')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS)            PORT_CHAR('-')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_PLUS_PAD)         PORT_CHAR('+')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_COLON)            PORT_CHAR(':')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_9)                PORT_CHAR('9')

	PORT_START("ROW1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_1)                PORT_CHAR('1')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_3)                PORT_CHAR('3')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_T)                PORT_CHAR('T')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_U)                PORT_CHAR('U')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_RIGHT)            PORT_NAME("Right")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_UP)               PORT_NAME("Up")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS_PAD)        PORT_CHAR('-')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_0)                PORT_CHAR('0')

	PORT_START("ROW2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_ESC)              PORT_CHAR(UCHAR_MAMEKEY(ESC))
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_E)                PORT_CHAR('E')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_5)                PORT_CHAR('5')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_J)                PORT_CHAR('J')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD)                                     PORT_NAME("Set")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD)                                     PORT_NAME("Add")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH)            PORT_CHAR('/')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA)            PORT_CHAR(',')

	PORT_START("ROW3")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_LSHIFT)           PORT_NAME("LShift")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_X)                PORT_CHAR('X')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_V)                PORT_CHAR('V')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_N)                PORT_CHAR('N')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_DOWN)             PORT_NAME("Down")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD)                                     PORT_NAME("Clear")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD)                                     PORT_NAME("WTF")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_L)                PORT_CHAR('L')

	PORT_START("ROW4")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_Q)                PORT_CHAR('Q')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_W)                PORT_CHAR('W')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_G)                PORT_CHAR('G')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_M)                PORT_CHAR('M')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_EQUALS)           PORT_CHAR('=')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD)                                     PORT_NAME("Home")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSPACE)        PORT_CHAR(8)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_K)                PORT_CHAR('K')

	PORT_START("ROW5")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_Z)                PORT_CHAR('Z')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_C)                PORT_CHAR('C')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_F)                PORT_CHAR('F')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_7)                PORT_CHAR('7')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_ENTER)            PORT_NAME("Return (a)")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_ENTER)            PORT_NAME("Return (b)")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_P)                PORT_CHAR('P')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_I)                PORT_CHAR('I')

	PORT_START("ROW6")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_A)                PORT_CHAR('A')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_D)                PORT_CHAR('D')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_B)                PORT_CHAR('S')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_H)                PORT_CHAR('H')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD)                                     PORT_NAME("Sub")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_SPACE)            PORT_CHAR(' ')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_RSHIFT)           PORT_NAME("RShift")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_STOP)             PORT_CHAR('.')

	PORT_START("ROW7")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_LCONTROL)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_S)                PORT_CHAR('S')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_R)                PORT_CHAR('R')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_Y)                PORT_CHAR('Y')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_LEFT)             PORT_NAME("Left")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_O)                PORT_CHAR('O')

	/* Keypad */
	PORT_START("KEYPAD_A")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_1_PAD)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_2_PAD)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_3_PAD)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_4_PAD)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_5_PAD)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_6_PAD)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_7_PAD)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_8_PAD)

	PORT_START("KEYPAD_B")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_9_PAD)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_0_PAD)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_1_PAD)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_2_PAD)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_3_PAD)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_4_PAD)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_5_PAD)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_6_PAD)

	/* Master musical keyboard */
	PORT_START("KEY_0_0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F0")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F0 #")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G0")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G0 #")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A1")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A1 #")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("B1")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C1")

	PORT_START("KEY_0_1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C1 #")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D1")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D1 #")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("E1")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F1")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F1 #")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G1")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G1 #")

	PORT_START("KEY_0_2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A2")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A2 #")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("B2")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C2")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C2 #")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D2")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D2 #")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("E2")

	PORT_START("KEY_0_3")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("KEY_1_0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F2")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F2 #")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G2")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G2 #")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A3")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A3 #")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("B3")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C3")

	PORT_START("KEY_1_1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C3 #")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D3")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D3 #")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("E3")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F3")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G3")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G3 #")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A4")

	PORT_START("KEY_1_2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A4 #")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("B4")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("B4 #")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C4")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C4 #")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D4")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D4 #")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("E4")

	PORT_START("KEY_1_3")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("KEY_2_0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F4")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F4 #")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G4")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G4 #")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A5")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A5 #")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("B5")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C5")

	PORT_START("KEY_2_1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C5 #")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D5")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D5 #")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("E5")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F5")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F5 #")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G5")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G5 #")

	PORT_START("KEY_2_2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A6")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A6 #")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("B6")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C6")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C6 #")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D6")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D6 #")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("E6")

	PORT_START("KEY_2_3")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F6")
INPUT_PORTS_END

bool cmi_state::map_is_active(int cpunum, int map, uint8_t *map_info)
{
	if (m_cpu_active_space[cpunum] == MAPPING_A)
	{
		*map_info = m_map_sel[cpunum ? MAPSEL_P2_A : MAPSEL_P1_A];

		if ((*map_info & 0x1f) == map)
			return 1;
	}
	else
	{
		*map_info = m_map_sel[cpunum ? MAPSEL_P2_B : MAPSEL_P1_B];

		if ((*map_info & 0x1f) == map)
			return 1;
	}

	return 0;
}

void cmi_state::update_address_space(int cpunum, uint8_t mapinfo)
{
	int map = mapinfo & 0x1f;
	bool vram_en = !BIT(mapinfo, 5);
	bool periph_en = !BIT(mapinfo, 7);
	int i;

	address_space *space = (cpunum == 0 ? m_cpu1space : m_cpu2space);

	space->unmap_readwrite(0x0000, 0xffff);

	/* Step through the map RAM assignments */
	for (int page = 0; page < PAGE_COUNT; ++page)
	{
		int address = page * PAGE_SIZE;
		uint8_t page_info = 0;

		/* Scan through the cards */
		for (i = 0; i < NUM_Q256_CARDS; ++i)
		{
			page_info = m_map_ram[i][(map << PAGE_SHIFT) + page];

			/* Page is enabled in this bank */
			if (page_info & 0x80)
				break;
		}

		if (BIT(m_cmi07_ctrl, 6))
		{
			if ((cpunum == 0) || !BIT(m_cmi07_ctrl, 7))
			{
				if (m_cmi07_ctrl & 0x30)
				if ((address & 0xc000) == ((m_cmi07_ctrl & 0x30) << 10))
				{
					space->install_ram(address, address + PAGE_SIZE - 1, &m_cmi07_ram[(page * PAGE_SIZE) & 0x3fff]);
					continue;
				}
			}
		}

		/* No banks had this page enabled - skip */
		if ((page_info & 0x80) == 0)
			continue;

		/* If Video RAM is enabled, don't install RAM here */
		if (vram_en && address >= 0x8000 && address <= 0xbfff)
			continue;

		/* If peripherals are enabled, don't install RAM here */
		if (periph_en && address >= 0xf000 && address <= 0xffff) // TODO
			continue;

		/* Now map the RAM page */
		space->install_ram(address, address + PAGE_SIZE - 1, &m_q256_ram[i][(page_info & 0x7f) * PAGE_SIZE]);
	}

	if (vram_en)
		install_video_ram(cpunum);

	if (periph_en)
		install_peripherals(cpunum);
}

WRITE8_MEMBER( cmi_state::cmi07_w )
{
	m_cmi07_ctrl = data;

	m_cmi07cpu->set_input_line(INPUT_LINE_RESET, BIT(data, 0) ? CLEAR_LINE : ASSERT_LINE);
	m_cmi07cpu->set_input_line(M6809_FIRQ_LINE,  BIT(data, 1) ? CLEAR_LINE : ASSERT_LINE);
	m_cmi07cpu->set_input_line(INPUT_LINE_NMI,   BIT(data, 2) ? CLEAR_LINE : ASSERT_LINE);
	m_cmi07cpu->set_input_line(INPUT_LINE_HALT,  BIT(data, 3) ? CLEAR_LINE : ASSERT_LINE);

	/* We need to update the address spaces */
	uint8_t map_info = (m_cpu_active_space[0] == MAPPING_A) ? m_map_sel[MAPSEL_P1_A] : m_map_sel[MAPSEL_P1_B];
	update_address_space(0, map_info);

	map_info = (m_cpu_active_space[1] == MAPPING_A) ? m_map_sel[MAPSEL_P2_A] : m_map_sel[MAPSEL_P2_B];
	update_address_space(1, map_info);
}

READ8_MEMBER( cmi_state::cmi07_r )
{
	//printf("CMI 07 R: %x\n", offset);
	return 0xff;
}

WRITE_LINE_MEMBER( cmi_state::q133_acia_irq0 )
{
	m_q133_acia_irq = (m_q133_acia_irq & ~1) | state;
	set_interrupt(CPU_1, IRQ_ACINT_LEVEL, m_q133_acia_irq ? ASSERT_LINE : CLEAR_LINE);
}

WRITE_LINE_MEMBER( cmi_state::q133_acia_irq1 )
{
	m_q133_acia_irq = (m_q133_acia_irq & ~2) | (state << 1);
	set_interrupt(CPU_1, IRQ_ACINT_LEVEL, m_q133_acia_irq ? ASSERT_LINE : CLEAR_LINE);
}

WRITE_LINE_MEMBER( cmi_state::q133_acia_irq2 )
{
	m_q133_acia_irq = (m_q133_acia_irq & ~4) | (state << 2);
	set_interrupt(CPU_1, IRQ_ACINT_LEVEL, m_q133_acia_irq ? ASSERT_LINE : CLEAR_LINE);
}

WRITE_LINE_MEMBER( cmi_state::q133_acia_irq3 )
{
	m_q133_acia_irq = (m_q133_acia_irq & ~8) | (state << 3);
	set_interrupt(CPU_1, IRQ_ACINT_LEVEL, m_q133_acia_irq ? ASSERT_LINE : CLEAR_LINE);
}

READ8_MEMBER( cmi_state::ank_col_r )
{
	int row = m_ank_pia->b_output() ^ 0xff;

	switch (row)
	{
		case 0x01: return m_ankrow_ports[0]->read();
		case 0x02: return m_ankrow_ports[1]->read();
		case 0x04: return m_ankrow_ports[2]->read();
		case 0x08: return m_ankrow_ports[3]->read();
		case 0x10: return m_ankrow_ports[4]->read();
		case 0x20: return m_ankrow_ports[5]->read();
		case 0x40: return m_ankrow_ports[6]->read();
		case 0x80: return m_ankrow_ports[7]->read();
		default:   return 0xff;
	}
}


/**************************************
 *
 *  Floppy disk interface
 *
 *************************************/

void cmi_state::dma_fdc_rom()
{
	/* DMA channel 1 is used*/
	uint8_t map_info = m_map_sel[MAPSEL_P2_A_DMA1];
	int map = map_info & 0x1f;
	int addr = m_fdc_dma_addr.w.l & ~PAGE_MASK;
	int page = addr / PAGE_SIZE;
	uint8_t p_info = 0;

	/* Active low */
	m_fdc_status &= ~FDC_STATUS_DRIVER_LOAD;

	int i;
	for (i = 0; i < NUM_Q256_CARDS; ++i)
	{
		p_info = m_map_ram[i][(map << PAGE_SHIFT) | page];

		if (p_info & 0x80)
			break;
	}

	if ((p_info & 0x80) == 0)
	{
		printf("Trying to DMA FDC driver to a non-enabled page!\n");
		return;
	}

	/* TODO: This should be stuck in a deferred write */
	int cnt = std::min(m_fdc_dma_cnt.w.l ^ 0xffff, 2048);
	memcpy(&m_q256_ram[i][(p_info & 0x7f) * PAGE_SIZE], m_qfc9_region_ptr, cnt);
	m_fdc_status |= FDC_STATUS_DRIVER_LOAD;

	/* TODO: Is this correct? */
	m_fdc_dma_addr.w.l += 0x800;
	m_fdc_dma_cnt.w.l = 0;
}

void cmi_state::write_fdc_ctrl(uint8_t data)
{
	int drive = data & 1;
	int side = BIT(data, 5) ? 1 : 0;

	switch (drive)
	{
	case 0: m_wd1791->set_floppy(m_floppy0->get_device()); break;
	case 1: m_wd1791->set_floppy(m_floppy1->get_device()); break;
	}

	if (m_floppy0->get_device()) m_floppy0->get_device()->ss_w(side);
	if (m_floppy1->get_device()) m_floppy1->get_device()->ss_w(side);

	m_wd1791->dden_w(BIT(data, 7) ? true : false);

	m_fdc_ctrl = data;
}

WRITE8_MEMBER( cmi_state::fdc_w )
{
	if (offset == 0)
	{
		switch (m_fdc_addr)
		{
			case 0x0: write_fdc_ctrl(data);         break;
			case 0x2: m_fdc_dma_addr.b.l = data;    break;
			case 0x4: m_fdc_dma_addr.b.h  = data;   break;
			case 0x6: m_fdc_dma_cnt.b.l = data;     break;
			case 0x8: m_fdc_dma_cnt.b.h = data;     break;
			case 0xa: dma_fdc_rom();                break;
			case 0xc: m_wd1791->cmd_w(data ^ 0xff);        break;
			case 0xd: m_wd1791->track_w(data ^ 0xff);      break;
			case 0xe: m_wd1791->sector_w(data ^ 0xff);     break;
			case 0xf: m_wd1791->data_w(data ^ 0xff);       break;
			default: printf("fdc_w: Invalid access (%x with %x)", m_fdc_addr, data);
		}
	}
	else
		m_fdc_addr = data;
}

READ8_MEMBER( cmi_state::fdc_r )
{
	if (machine().side_effect_disabled())
		return 0;

	if (offset == 0)
	{
		switch (m_fdc_addr)
		{
			case 0xc: { return m_wd1791->status_r() ^ 0xff; }
			case 0xd: { return m_wd1791->track_r() ^ 0xff; }
			case 0xe: { return m_wd1791->sector_r() ^ 0xff; }
			case 0xf: { return m_wd1791->data_r() ^ 0xff; }
			default:  return 0;
		}
	}
	else
		return m_fdc_status;
}

void cmi_state::fdc_dma_transfer()
{
	/* DMA channel 1 is used*/
	uint8_t map_info = m_map_sel[MAPSEL_P2_A_DMA1];
	int map = map_info & 0x1f;

	int cpu_page = (m_fdc_dma_addr.w.l & ~PAGE_MASK) / PAGE_SIZE;
	int phys_page = 0;

	int i;
	for (i = 0; i < NUM_Q256_CARDS; ++i)
	{
		phys_page = m_map_ram[i][(map << PAGE_SHIFT) | cpu_page];

		if (phys_page & 0x80)
			break;
	}

	//phys_page &= 0x7f;

	/* Transfer from disk to RAM */
	if (!BIT(m_fdc_ctrl, 4))
	{
		/* Read a byte at a time */
		uint8_t data = m_wd1791->data_r() ^ 0xff;

		if (m_fdc_dma_cnt.w.l == 0xffff)
			return;

		if (m_cmi07_ctrl & 0x30)
			if (BIT(m_cmi07_ctrl, 6) && !BIT(m_cmi07_ctrl, 7))
			{
				if ((m_fdc_dma_addr.w.l & 0xc000) == ((m_cmi07_ctrl & 0x30) << 10))
					m_cmi07_ram[m_fdc_dma_addr.w.l & 0x3fff] = data;
			}

		if (phys_page & 0x80)
			m_q256_ram[i][((phys_page & 0x7f) * PAGE_SIZE) + (m_fdc_dma_addr.w.l & PAGE_MASK)] = data;

		if (!BIT(m_fdc_ctrl, 3))
			m_fdc_dma_addr.w.l++;
	}

	// Transfer from RAM to disk
	else
	{
		if (m_fdc_dma_cnt.w.l == 0xffff)
			return;

		/* Write a byte at a time */
		uint8_t data = 0;

		/* TODO: This should be stuck in a deferred write */
		if (phys_page & 0x80)
			data = m_q256_ram[i][((phys_page & 0x7f) * PAGE_SIZE) + (m_fdc_dma_addr.w.l & PAGE_MASK)];

		m_wd1791->data_w(data ^ 0xff);

		if (!BIT(m_fdc_ctrl, 3))
			m_fdc_dma_addr.w.l++;
	}

	m_fdc_dma_cnt.w.l++;
}

WRITE_LINE_MEMBER( cmi_state::wd1791_irq )
{
	if (state)
	{
		m_fdc_status |= FDC_STATUS_INTERRUPT;

		if (m_fdc_ctrl & FDC_CONTROL_INTEN)
			set_interrupt(CPU_2, IRQ_DISKINT_LEVEL, ASSERT_LINE);

	}
	else
	{
		m_fdc_status &= ~FDC_STATUS_INTERRUPT;
		set_interrupt(CPU_2, IRQ_DISKINT_LEVEL, CLEAR_LINE);
	}
}

WRITE_LINE_MEMBER( cmi_state::wd1791_drq )
{
	m_fdc_drq = state;
	if (state)
		fdc_dma_transfer();
}



/**************************************
 *
 *  Master card and channel cards
 *
 *************************************/

/*
0 - 1f
20 = PIA
21 = PIA
22 = PIA
23 = PIA

24 = ADC
25 = ADC
26 = HALT CPU 2
27 = UNHALT CPU 2
28 - 2B = PIA
*/

WRITE8_MEMBER( cmi_state::master_tune_w )
{
//  double mfreq = (double)data * ((double)MASTER_OSCILLATOR / 2.0) / 256.0;
}

WRITE_LINE_MEMBER( cmi01a_device::pia_0_ca2_w )
{
	// upate_stream()
	if (!state)
	{
		m_segment_cnt = 0x4000 | ((m_pia_0->a_output() & 0x7f) << 7);
		m_new_addr = 1;
		m_pia_1->cb1_w(1);
	}
}

WRITE_LINE_MEMBER( cmi01a_device::pia_0_irqa )
{
	m_pia_0_irqa = state;
	//printf("CH%d pia0 irqa int: %x\n", m_channel, state);
	update_interrupts();
}

WRITE_LINE_MEMBER( cmi01a_device::pia_0_irqb )
{
	m_pia_0_irqb = state;
	//printf("CH%d pia0 irqb int: %x\n", m_channel, state);
	update_interrupts();
}

WRITE8_MEMBER( cmi01a_device::pia_1_a_w )
{
// top two
}

WRITE8_MEMBER( cmi01a_device::pia_1_b_w )
{
}

WRITE8_MEMBER( cmi01a_device::rp_w )
{
	m_rp = data;
}

WRITE8_MEMBER( cmi01a_device::ws_dir_w )
{
	m_ws = data & 0x7f;
	m_dir = (data >> 7) & 1;
}

READ_LINE_MEMBER( cmi01a_device::tri_r )
{
	bool top_terminal_count = (m_dir == ENV_DIR_UP && m_rp == 0);
	bool bottom_terminal_count = (m_dir == ENV_DIR_DOWN && m_rp == 0xff);
	return (top_terminal_count || bottom_terminal_count) ? 1 : 0;
}

WRITE_LINE_MEMBER( cmi01a_device::pia_1_irqa )
{
	m_pia_1_irqa = state;
	//printf("CH%d pia1 irqa int: %x\n", m_channel, state);
	update_interrupts();
}

WRITE_LINE_MEMBER( cmi01a_device::pia_1_irqb )
{
	m_pia_1_irqb = state;
	//printf("CH%d pia1 irqb int: %x\n", m_channel, state);
	update_interrupts();
}

void cmi01a_device::update_interrupts()
{
	int old_state = m_irq_state;
	m_irq_state = m_pia_0_irqa || m_pia_0_irqb || m_pia_1_irqa || m_pia_1_irqb || m_ptm_irq;

	if (m_irq_state != old_state)
		dynamic_cast<cmi_state*>(owner())->set_interrupt(CPU_1, ch_int_levels[m_channel], m_irq_state ? ASSERT_LINE : CLEAR_LINE);
}

WRITE_LINE_MEMBER( cmi01a_device::ptm_irq )
{
	m_ptm_irq = state;
	//printf("CH%d ptm irq int: %x\n", m_channel, state);
	update_interrupts();
}

void cmi01a_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	switch(id)
	{
		case TIMER_ZX:
			zx_timer_cb();
			break;
	}
}

void cmi01a_device::zx_timer_cb()
{
	/* Set ZX */
	if (m_zx_flag == 0)
		m_pia_1->ca1_w(1);
	else
		m_pia_1->ca1_w(0);

	m_zx_flag ^= 1;

	if (m_zx_flag == 0)
	{
		/* Low to high transition - clock flip flop */
		int op = m_ptm_out0;

		/* Set /ZCINT */
		if (op != m_zx_ff)
			m_pia_0->ca1_w(0);

		m_zx_ff = op;
		m_pia_0->ca1_w(1);
	}
}

void cmi01a_device::run_voice()
{
	int val_a = m_pia_1->a_output();
	int pitch = ((val_a & 3) << 8) | m_pia_1->b_output();
	int o_val = (val_a >> 2) & 0xf;

	int m_tune = m_cmi02_pia_0->b_output();
	double mfreq = (double)(0xf00 | m_tune) * ((double)MASTER_OSCILLATOR / 2.0) / 4096.0;

	double cfreq = ((double)(0x800 | (pitch << 1))* mfreq) / 4096.0;

//  if (cfreq > 0.0)
	{
		/* Octave register enabled? */
		if (!(o_val & 0x8))
			cfreq /= 2 << ((7 ^ o_val) & 7);

		cfreq /= 16.0f;

		m_freq = cfreq;

		m_stream->set_sample_rate(cfreq);

		// Set timers and things?
		attotime zx_period = attotime::from_ticks(64, cfreq);
		m_zx_timer->adjust(zx_period, 0, zx_period);

		m_active = true;
	}
}

WRITE_LINE_MEMBER( cmi01a_device::pia_0_cb2_w )
{
	//streams_update();

	/* RUN */
	if (state)
	{
		m_segment_cnt = 0x4000 | ((m_pia_0->a_output() & 0x7f) << 7);
		m_new_addr = 1;

		/* Clear /EOSI */
//      pia6821_cb1_w(card->pia[1], 0, 1);

		/* Clear ZX */
		m_pia_1->ca1_w(0);

		/* Clear /ZCINT */
		m_pia_0->ca1_w(1);

		m_ptm->set_g1(0);
		m_ptm->set_g2(0);
		m_ptm->set_g3(0);

		run_voice();
	}
	else
	{
		/* Clear /EOSI */
		m_pia_1->cb1_w(1);

		m_ptm->set_g1(1);
		m_ptm->set_g2(1);
		m_ptm->set_g3(1);

		//printf("Stop %d\n", m_channel);

		m_zx_timer->adjust(attotime::never);
		m_active = false;
		m_zx_flag = 0;  // TEST
		m_zx_ff = 0;
	}

}

void cmi01a_device::update_wave_addr(int inc)
{
	int old_cnt = m_segment_cnt;

	if (inc)
		++m_segment_cnt;

	/* Update end of sound interrupt flag */
	m_pia_1->cb1_w((m_segment_cnt & 0x4000) >> 14);

	/* TODO Update zero crossing flag */
	m_pia_1->ca1_w((m_segment_cnt & 0x40) >> 6);

	/* Clock a latch on a transition */
	if ((old_cnt & 0x40) && !(m_segment_cnt & 0x40))
	{
		// TODO: ECLK
		m_pia_1->ca2_w(1);
		m_pia_1->ca2_w(0);
	}

	/* Zero crossing interrupt is a pulse */
}

WRITE_LINE_MEMBER( cmi01a_device::ptm_out0 )
{
	m_ptm_out0 = state;
}

READ_LINE_MEMBER( cmi01a_device::eosi_r )
{
	return (m_segment_cnt & 0x4000) >> 14;
}

READ_LINE_MEMBER( cmi01a_device::zx_r )
{
	return m_segment_cnt & 0x40;
}

WRITE8_MEMBER( cmi01a_device::write )
{
	//printf("C%d W: %02x = %02x\n", m_channel, offset, data);

	switch (offset)
	{
		case 0x0:
			if (m_new_addr)
				m_new_addr = 0;

			m_wave_ram[m_segment_cnt & 0x3fff] = data;
			update_wave_addr(1);
			break;

		case 0x3:
			m_env_dir_ctrl = ENV_DIR_DOWN;
			break;

		case 0x4:
			m_env_dir_ctrl = ENV_DIR_UP;
			break;

		case 0x5:
			m_vol_latch = data;
			break;

		case 0x6:
			m_flt_latch = data;
			break;

		case 0x8: case 0x9: case 0xa: case 0xb:
			m_pia_0->write(space, offset & 3, data);
			break;

		case 0xc: case 0xd: case 0xe: case 0xf:
			m_pia_1->write(space, (BIT(offset, 0) << 1) | BIT(offset, 1), data);
			break;

		case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17:
		{
			/* PTM addressing is a little funky */
			int a0 = offset & 1;
			int a1 = (m_ptm_out0 && BIT(offset, 3)) || (!BIT(offset, 3) && BIT(offset, 2));
			int a2 = BIT(offset, 1);

			//printf("CH%d PTM W: [%x] = %02x\n", m_channel, (a2 << 2) | (a1 << 1) | a0, data);
			m_ptm->write(space, (a2 << 2) | (a1 << 1) | a0, data);
			break;
		}

		default:
			printf("Unknown channel card %d write to E0%02X = %02X\n", m_channel, offset, data);
			break;
	}
}

READ8_MEMBER( cmi01a_device::read )
{
	if (machine().side_effect_disabled())
		return 0;

	uint8_t data = 0;

	switch (offset)
	{
		case 0x0:
			if (m_new_addr)
			{
				m_new_addr = 0;
				break;
			}
			data = m_wave_ram[m_segment_cnt & 0x3fff];
			update_wave_addr(1);
			break;

		case 0x3:
			m_env_dir_ctrl = ENV_DIR_DOWN;
			break;

		case 0x4:
			m_env_dir_ctrl = ENV_DIR_UP;
			break;

		case 0x5:
			data = 0xff;
			break;

		case 0x8: case 0x9: case 0xa: case 0xb:
			data = m_pia_0->read(space, offset & 3);
			break;

		case 0xc: case 0xd: case 0xe: case 0xf:
			data = m_pia_1->read(space, (BIT(offset, 0) << 1) | BIT(offset, 1));
			break;

		case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17:
		{
			int a0 = offset & 1;
			int a1 = (m_ptm_out0 && BIT(offset, 3)) || (!BIT(offset, 3) && BIT(offset, 2));
			int a2 = BIT(offset, 1);

			data = m_ptm->read(space, (a2 << 2) | (a1 << 1) | a0);

			//printf("CH%d PTM R: [%x] %02x\n", m_channel, (a2 << 2) | (a1 << 1) | a0, data);
			break;
		}

		default:
			printf("Unknown channel card %d read from E0%02X\n", m_channel, offset);
			break;
	}

	//printf("C%d R: %02x = %02x\n", m_channel, offset, data);

	return data;
}

WRITE_LINE_MEMBER( cmi_state::cmi02_ptm_irq )
{
	//printf("cmi02_ptm_irq: %d\n", state);
	m_cmi02_ptm_irq = state;
	set_interrupt(CPU_1, IRQ_TIMINT_LEVEL, m_cmi02_ptm_irq ? ASSERT_LINE : CLEAR_LINE);
}

WRITE_LINE_MEMBER( cmi_state::cmi02_ptm_o1 )
{
	m_cmi02_ptm->set_c1(state);
	m_cmi02_ptm->set_c3(state);
}

READ8_MEMBER( cmi_state::cmi02_r )
{
	if (machine().side_effect_disabled())
		return 0;

	if (offset <= 0x1f)
	{
		int ch_mask = m_cmi02_pia_0->a_output();

		for (int i = 0; i < 8; ++i)
		{
			if (ch_mask & (1 << i))
			{
				return m_channels[i]->read(space, offset & 0x1f, 0xff);
			}
		}

		return 0xff;
	}
	else
	{
		switch (offset)
		{
			case 0x20: case 0x21: case 0x22: case 0x23:
				return m_cmi02_pia_0->read(space, offset & 3);

			case 0x26:
				m_maincpu2->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
				/* LS123 one-shot with 10n and 150k */
				m_jam_timeout_timer->adjust(attotime::from_usec(675));
				return 0xff;

			case 0x27:
				m_maincpu2->set_input_line(INPUT_LINE_HALT, CLEAR_LINE);
				return 0xff;

			case 0x28: case 0x29: case 0x2a: case 0x2b:
				return m_cmi02_pia_1->read(space, offset & 3);

			case 0x38: case 0x39: case 0x3a: case 0x3b: case 0x3c: case 0x3d: case 0x3e: case 0x3f:
				return m_cmi02_ptm->read(space, offset & 7);

			default:
				logerror("CMI02 R: %x\n", offset);
				return 0;
		}
	}
}

WRITE8_MEMBER( cmi_state::cmi02_w )
{
	if (offset <= 0x1f)
	{
		int ch_mask = m_cmi02_pia_0->a_output();

		for (int i = 0; i < 8; ++i)
		{
			if (ch_mask & (1 << i))
				m_channels[i]->write(space, offset & 0x1f, data, 0xff);
		}
	}
	else
	{
		switch (offset)
		{
			case 0x20: case 0x21: case 0x22: case 0x23:
				m_cmi02_pia_0->write(space, offset & 3, data);
				break;

			case 0x28: case 0x29: case 0x2a: case 0x2b:
				m_cmi02_pia_1->write(space, offset & 3, data);
				break;

			case 0x30:
				m_maincpu1->set_input_line(M6809_IRQ_LINE, CLEAR_LINE);
				data ^= 0xff;
				m_i8214_2->sgs_w((data >> 3) & 1);
				m_i8214_2->b_w(data & 0x7);
				break;

			case 0x31: case 0x32:
				set_interrupt(0, IRQ_INTP1_LEVEL, (offset & 2) ? CLEAR_LINE : ASSERT_LINE);
				break;

			case 0x33: case 0x34:
				set_interrupt(1, IRQ_INTP2_LEVEL, (offset & 4) ? CLEAR_LINE : ASSERT_LINE);
				break;

			case 0x38: case 0x39: case 0x3a: case 0x3b: case 0x3c: case 0x3d: case 0x3e: case 0x3f:
				m_cmi02_ptm->write(space, offset & 7, data);
				break;

			default:
				logerror("CMI02 W: %x %x\n", offset, data);
		}
	}
}

void cmi_state::install_video_ram(int cpunum)
{
	address_space *space = (cpunum == CPU_1 ? m_cpu1space : m_cpu2space);

	space->install_readwrite_handler(0x8000, 0xbfff, read8_delegate(FUNC(cmi_state::vram_r),this), write8_delegate(FUNC(cmi_state::vram_w),this));
}

WRITE8_MEMBER( cmi_state::i8214_cpu1_w )
{
	//printf("i8214_cpu1_w: %02x\n", data);
	m_maincpu1->set_input_line(M6809_IRQ_LINE, CLEAR_LINE);
	data ^= 0xff;
	m_i8214_0->sgs_w((data >> 3) & 1);
	m_i8214_0->b_w(data & 0x7);
}


WRITE8_MEMBER( cmi_state::i8214_cpu2_w )
{
	//printf("i8214_cpu2_w: %02x\n", data);
	m_maincpu2->set_input_line(M6809_IRQ_LINE, CLEAR_LINE);
	data ^= 0xff;
	m_i8214_1->sgs_w((data >> 3) & 1);
	m_i8214_1->b_w(data & 0x7);
}

// TODO: replace with AM_SHARE
READ8_MEMBER( cmi_state::shared_ram_r )
{
	return m_shared_ram[offset];
}

WRITE8_MEMBER( cmi_state::shared_ram_w )
{
	m_shared_ram[offset] = data;
}

WRITE_LINE_MEMBER( cmi_state::ank_irqa_w )
{
	m_ank_irqa = state;

	if (m_ank_irqa)
		m_alphakeyscpu->set_input_line(M6802_IRQ_LINE, ASSERT_LINE);
	else if(!m_ank_irqb)
		m_alphakeyscpu->set_input_line(M6802_IRQ_LINE, CLEAR_LINE);
}

WRITE_LINE_MEMBER( cmi_state::ank_irqb_w )
{
	m_ank_irqb = state;

	if (m_ank_irqb)
		m_alphakeyscpu->set_input_line(M6802_IRQ_LINE, ASSERT_LINE);
	else if(!m_ank_irqa)
		m_alphakeyscpu->set_input_line(M6802_IRQ_LINE, CLEAR_LINE);
}

READ_LINE_MEMBER( cmi_state::ank_rts_r )
{
//  printf("ANK RTS?\n");
	return 0;
}

void cmi_state::install_peripherals(int cpunum)
{
	address_space *space = (cpunum == CPU_1 ? m_cpu1space : m_cpu2space);

	space->install_readwrite_handler(0xe000, 0xe03f, read8_delegate(FUNC(cmi_state::cmi02_r),this), write8_delegate(FUNC(cmi_state::cmi02_w),this));

	space->install_readwrite_handler(0xf000, 0xf7ff, read8_delegate(FUNC(cmi_state::rom_r),this), write8_delegate(FUNC(cmi_state::map_ram_w),this));

	space->install_rom(0xf800, 0xfbff, m_q133_rom + (cpunum == CPU_2 ? 0x1800 : 0x2800));

	space->install_readwrite_handler(0xfc40, 0xfc4f, read8_delegate(FUNC(cmi_state::parity_r),this), write8_delegate(FUNC(cmi_state::mapsel_w),this));
	space->nop_readwrite(0xfc5a, 0xfc5b); // Q077 HDD controller - not installed
	space->install_readwrite_handler(0xfc5e, 0xfc5e, read8_delegate(FUNC(cmi_state::atomic_r),this), write8_delegate(FUNC(cmi_state::cpufunc_w),this));
	space->install_readwrite_handler(0xfc5f, 0xfc5f, read8_delegate(FUNC(cmi_state::map_r),this), write8_delegate(FUNC(cmi_state::map_w),this));

	space->install_readwrite_handler(0xfc80, 0xfc83, read8_delegate(FUNC(mos6551_device::read),m_q133_acia_0.target()), write8_delegate(FUNC(mos6551_device::write),m_q133_acia_0.target()));
	space->install_readwrite_handler(0xfc84, 0xfc87, read8_delegate(FUNC(mos6551_device::read),m_q133_acia_1.target()), write8_delegate(FUNC(mos6551_device::write),m_q133_acia_1.target()));
	space->install_readwrite_handler(0xfc88, 0xfc8b, read8_delegate(FUNC(mos6551_device::read),m_q133_acia_2.target()), write8_delegate(FUNC(mos6551_device::write),m_q133_acia_2.target()));
	space->install_readwrite_handler(0xfc8c, 0xfc8f, read8_delegate(FUNC(mos6551_device::read),m_q133_acia_3.target()), write8_delegate(FUNC(mos6551_device::write),m_q133_acia_3.target()));
	space->install_readwrite_handler(0xfc90, 0xfc97, read8_delegate(FUNC(ptm6840_device::read),m_q133_ptm.target()), write8_delegate(FUNC(ptm6840_device::write),m_q133_ptm.target()));

	space->install_readwrite_handler(0xfcbc, 0xfcbc, read8_delegate(FUNC(cmi_state::cmi07_r),this), write8_delegate(FUNC(cmi_state::cmi07_w),this));

	space->install_read_handler(0xfcc0, 0xfcc3, read8_delegate(FUNC(cmi_state::lightpen_r),this));
	space->install_readwrite_handler(0xfcc4, 0xfcc7, read8_delegate(FUNC(pia6821_device::read),m_q219_pia.target()), write8_delegate(FUNC(pia6821_device::write),m_q219_pia.target()));
	space->install_readwrite_handler(0xfcc8, 0xfccf, read8_delegate(FUNC(ptm6840_device::read),m_q219_ptm.target()), write8_delegate(FUNC(ptm6840_device::write),m_q219_ptm.target()));
	space->install_readwrite_handler(0xfcd0, 0xfcdc, read8_delegate(FUNC(cmi_state::video_r),this), write8_delegate(FUNC(cmi_state::video_w),this));
	space->install_readwrite_handler(0xfce0, 0xfce1, read8_delegate(FUNC(cmi_state::fdc_r),this), write8_delegate(FUNC(cmi_state::fdc_w),this));
	space->nop_readwrite(0xfce2, 0xfcef); // Monitor ROM will attempt to detect floppy disk controller cards in this entire range
	space->install_readwrite_handler(0xfcf0, 0xfcf7, read8_delegate(FUNC(pia6821_device::read),m_q133_pia_0.target()), write8_delegate(FUNC(pia6821_device::write),m_q133_pia_0.target()));
	space->install_readwrite_handler(0xfcf8, 0xfcff, read8_delegate(FUNC(pia6821_device::read),m_q133_pia_1.target()), write8_delegate(FUNC(pia6821_device::write),m_q133_pia_1.target()));

	space->install_write_handler(0xfcfc, 0xfcfc, write8_delegate(FUNC(cmi_state::i8214_cpu1_w),this));
	space->install_write_handler(0xfcfd, 0xfcfd, write8_delegate(FUNC(cmi_state::i8214_cpu2_w),this));

	space->install_readwrite_handler(0xfd00, 0xfeff, read8_delegate(FUNC(cmi_state::shared_ram_r),this), write8_delegate(FUNC(cmi_state::shared_ram_w),this));

	space->install_ram(0xff00, 0xfff7, &m_scratch_ram[cpunum][0]);
	space->install_ram(0xfffa, 0xfffd, &m_scratch_ram[cpunum][0xfa]);

	space->install_readwrite_handler(0xfff8, 0xfff9, read8_delegate(FUNC(cmi_state::irq_ram_r),this), write8_delegate(FUNC(cmi_state::irq_ram_w),this));
	space->install_read_handler(0xfffe, 0xffff, read8_delegate(FUNC(cmi_state::vector_r),this));
}


/*************************************
 *
 *  Interrupt Handling
 *
 *************************************/

WRITE_LINE_MEMBER( cmi_state::ptm_q219_irq )
{
	set_interrupt(CPU_2, IRQ_RINT_LEVEL, state);
}

IRQ_CALLBACK_MEMBER( cmi_state::cpu1_interrupt_callback )
{
	/* Switch to mapping A */
	m_cpu_active_space[CPU_1] = MAPPING_A;
	update_address_space(CPU_1, m_map_sel[MAPSEL_P1_A]);

	if (irqline == INPUT_LINE_IRQ0)
	{
		int vector = (m_hp_int ? 0xffe0 : 0xffd0);
		int level = (m_hp_int ? m_i8214_2->a_r() : m_i8214_0->a_r()) ^ 7;
		m_irq_address[CPU_1][0] = m_cpu1space->read_byte(vector + level*2);
		m_irq_address[CPU_1][1] = m_cpu1space->read_byte(vector + level*2 + 1);

		m_m6809_bs_hack_cnt = 2;
		m_m6809_bs_hack_cpu = CPU_1;

		//printf("cpu1 interrupt, will be pushing address %02x%02x\n", m_irq_address[CPU_1][0], m_irq_address[CPU_1][1]);
	}

	return 0;
}

IRQ_CALLBACK_MEMBER( cmi_state::cpu2_interrupt_callback )
{
	/* Switch to mapping A */
	m_cpu_active_space[CPU_2] = MAPPING_A;
	update_address_space(CPU_2, m_map_sel[MAPSEL_P2_A]);

	if (irqline == INPUT_LINE_IRQ0)
	{
		int level = m_i8214_1->a_r() ^ 0x7;
		m_irq_address[CPU_2][0] = m_cpu2space->read_byte(0xffe0 + level*2);
		m_irq_address[CPU_2][1] = m_cpu2space->read_byte(0xffe0 + level*2 + 1);

		m_m6809_bs_hack_cnt = 2;
		m_m6809_bs_hack_cpu = CPU_2;

		//printf("cpu1 interrupt, will be pushing address %02x%02x\n", m_irq_address[CPU_2][0], m_irq_address[CPU_2][1]);
	}
	return 0;
}

void cmi_state::set_interrupt(int cpunum, int level, int state)
{
	//printf("CPU%d Int: %x State: %x\n", cpunum + 1, level, state);

	i8214_device *i8214 = ((cpunum == CPU_2) ? m_i8214_1 : (level < 8 ? m_i8214_2 : m_i8214_0));
	i8214->r_w(level & 7, state ? 0 : 1);
}

WRITE_LINE_MEMBER( cmi_state::i8214_1_int_w )
{
	//printf("i8214_1_int_w: %d\n", state);
	if (state)
		m_maincpu1->set_input_line(M6809_IRQ_LINE, ASSERT_LINE);
}

WRITE_LINE_MEMBER( cmi_state::i8214_2_int_w )
{
	//printf("i8214_2_int_w: %d\n", state);
	if (state)
		m_maincpu2->set_input_line(M6809_IRQ_LINE, ASSERT_LINE);
}

WRITE_LINE_MEMBER( cmi_state::i8214_3_int_w )
{
	//printf("i8214_3_int_w: %d\n", state);
	m_hp_int = state;
	if (state)
		m_maincpu1->set_input_line(M6809_IRQ_LINE, ASSERT_LINE);
}


WRITE_LINE_MEMBER( cmi_state::i8214_3_enlg )
{
	// Not needed?
//  m_hp_int = state;
}

WRITE_LINE_MEMBER( cmi_state::pia_q219_irqa )
{
	set_interrupt(CPU_2, IRQ_TOUCHINT_LEVEL, state);
}

WRITE_LINE_MEMBER( cmi_state::pia_q219_irqb )
{
	set_interrupt(CPU_2, IRQ_PENINT_LEVEL, state);
}


/* E9 - E11 - MSM5832RS */
/*
    A0-A3  = MSM5832 A0-A3
    A4+CA1 = MSM5832 D0
    A5+CB1 = MSM5832 D1
    A6     = MSM5832 D2
    A7     = MSM5832 D3
    B0     = HOLD
    B1     = READ
    B2     = WRITE
    CB2    = OPTO
    B3     = ----
    B4-B7  = PC0-3

    IRQA/B = /RTINT?
*/

READ8_MEMBER( cmi_state::q133_1_porta_r )
{
	if (BIT(m_q133_pia_0->b_output(), 1))
	{
		return m_msm5832->data_r(space, m_msm5832_addr) << 4;
	}
	return 0xff;
}

WRITE8_MEMBER( cmi_state::q133_1_porta_w )
{
	m_msm5832_addr = data & 0xf;
	m_msm5832->address_w(data & 0x0f);
}

WRITE8_MEMBER( cmi_state::q133_1_portb_w )
{
	m_msm5832->hold_w(BIT(data, 0));
	m_msm5832->read_w(BIT(data, 1));
	m_msm5832->write_w(BIT(data, 2));
}

/*
    PA0-7 = BKA0-7 (display)

    PB0 = DA1
    PB1 = DA0
    PB2 = CS2
    PB3 = CU2
    PB4 = CS1
    PB5 = CU1
    PB6 = CS0
    PB7 = CU0

    CB1 = /KPAD
    CB2 = /DWS
*/

WRITE8_MEMBER( cmi_state::cmi10_u20_a_w )
{
	// low 7 bits connected to alphanumeric display data lines
	m_dp1->data_w(data & 0x7f);
	m_dp2->data_w(data & 0x7f);
	m_dp3->data_w(data & 0x7f);

	/*
	int bk = data;
	int bit = 0;

	if (BIT(bk, 3))
	    bit = BIT(input_port_read(device->machine, "KEYPAD_A"), bk & 7);
	else if (!BIT(bk, 4))
	    bit = BIT(input_port_read(device->machine, "KEYPAD_B"), bk & 7);

	pia6821_cb1_w(m_cmi10_pia_u20, 0, !bit);
	*/
}

WRITE8_MEMBER( cmi_state::cmi10_u20_b_w )
{
	// connected to alphanumeric display control lines
	uint8_t const addr = bitswap<2>(data, 0, 1);

	m_dp1->ce_w(BIT(data, 6));
	m_dp1->cu_w(BIT(data, 7));
	m_dp1->addr_w(addr);

	m_dp2->ce_w(BIT(data, 4));
	m_dp2->cu_w(BIT(data, 5));
	m_dp2->addr_w(addr);

	m_dp3->ce_w(BIT(data, 2));
	m_dp3->cu_w(BIT(data, 3));
	m_dp3->addr_w(addr);
}

READ_LINE_MEMBER( cmi_state::cmi10_u20_cb1_r )
{
	int bk = m_cmi10_pia_u20->a_output();
	int bit = 0;

	if (BIT(bk, 3))
		bit = BIT(m_keypad_a_port->read(), bk & 7);
	else if (!BIT(bk, 4))
		bit = BIT(m_keypad_b_port->read(), bk & 7);

	return !bit;
}

WRITE_LINE_MEMBER( cmi_state::cmi10_u20_cb2_w )
{
	// connected to alphanumeric display write strobe
	m_dp1->wr_w(state);
	m_dp2->wr_w(state);
	m_dp3->wr_w(state);
}

WRITE16_MEMBER( cmi_state::cmi_iix_update_dp1 )
{
	output().set_digit_value(0 + (offset ^ 3), data);
}

WRITE16_MEMBER( cmi_state::cmi_iix_update_dp2 )
{
	output().set_digit_value(4 + (offset ^ 3), data);
}

WRITE16_MEMBER( cmi_state::cmi_iix_update_dp3 )
{
	output().set_digit_value(8 + (offset ^ 3), data);
}

/* Begin Conversion */
WRITE_LINE_MEMBER( cmi_state::cmi10_u21_cb2_w )
{
	// if 0
//  state = state;
}


READ8_MEMBER( cmi_state::cmi10_u21_a_r )
{
#if 0
//  int thld = m_cmi10_pia_u21->ca2_output();
	int sel = m_cmi10_pia_u20->a_output();
	int key = sel & 7;
	int mux = (sel >> 3) & 3;
	uint8_t data = 0x38; // slave keyboard not used


	for (int module = 0; module < 3; ++module)
	{
//      char keyname[16];
		uint8_t keyval;
		int state = 1;

		if (mux == 0 && key == 3)
		{
			//keyval = input_port_read(device->machine, "ANALOG");

			/* Unpressed */
			if (keyval <= 0)
				state = 1;
			/* In flight */

	#if 0
			else if (keyval <= 80)
			{
				if (thld == 1)
					state = 0;
				else
					state = 1;
			}
			/* Fully depressed */
	#endif
			else
				state = 0;

		}

		data |= state << module;
	}

	return data;
#else
	int sel = m_cmi10_pia_u20->a_output();
	int key = sel & 7;
	int mux = (sel >> 3) & 3;
	uint8_t data = 0xf8; // slave keyboard not used

	for (int module = 0; module < 3; ++module)
	{
		uint8_t keyval = m_key_mux_ports[mux][module]->read();
		data |= BIT(keyval, key) << module;
	}

	return data;
#endif
}

 /*************************************
 *
 *  6850 ACIAs
 *
 *************************************/

//static int kbd_to_cmi;
//static int cmi_to_kbd;

WRITE_LINE_MEMBER( cmi_state::mkbd_acia_clock )
{
	m_acia_mkbd_kbd->write_rxc(state);
	m_acia_mkbd_kbd->write_txc(state);
	m_acia_mkbd_cmi->write_rxc(state);
	m_acia_mkbd_cmi->write_txc(state);
	m_q133_acia_0->write_rxc(state);
	m_q133_acia_1->write_rxc(state);
	m_q133_acia_2->write_rxc(state);
	m_q133_acia_3->write_rxc(state);
}

WRITE_LINE_MEMBER( cmi_state::msm5832_irq )
{
#if 0
	set_interrupt(CPU_2, IRQ_RTCINT_LEVEL, state ? ASSERT_LINE : CLEAR_LINE);
#endif
}

WRITE_LINE_MEMBER( cmi_state::mkbd_kbd_acia_int )
{
	m_mkbd_kbd_acia_irq = state;

	if (m_mkbd_kbd_acia_irq)
	{
		m_muskeyscpu->set_input_line(INPUT_LINE_IRQ0, ASSERT_LINE);
	}
	else if (!m_mkbd_cmi_acia_irq)
	{
		m_muskeyscpu->set_input_line(INPUT_LINE_IRQ0, CLEAR_LINE);
	}
}

WRITE_LINE_MEMBER( cmi_state::mkbd_cmi_acia_int )
{
	m_mkbd_cmi_acia_irq = state;

	if (m_mkbd_cmi_acia_irq)
		m_muskeyscpu->set_input_line(INPUT_LINE_IRQ0, ASSERT_LINE);
	else if (!m_mkbd_kbd_acia_irq)
		m_muskeyscpu->set_input_line(INPUT_LINE_IRQ0, CLEAR_LINE);
}

WRITE_LINE_MEMBER( cmi_state::cmi07_irq )
{
	m_cmi07cpu->set_input_line(INPUT_LINE_IRQ0, state ? ASSERT_LINE : CLEAR_LINE);
}

void cmi_state::machine_reset()
{
	m_cpu1space = &m_maincpu1->space(AS_PROGRAM);
	m_cpu2space = &m_maincpu2->space(AS_PROGRAM);

	m_qfc9_region_ptr = (uint8_t *)m_qfc9_region->base();

	/* Set 8214 interrupt lines */
	m_i8214_0->etlg_w(1);
	m_i8214_0->inte_w(1);
	m_i8214_1->etlg_w(1);
	m_i8214_1->inte_w(1);
	m_i8214_2->etlg_w(1);
	m_i8214_2->inte_w(1);

	m_hblank_timer->adjust(m_screen->time_until_pos(0, HBLANK_START));

	m_scnd = 0;

	for (int cpunum = 0; cpunum < 2; ++cpunum)
	{
		address_space *space = (cpunum == CPU_1 ? m_cpu1space : m_cpu2space);

		space->unmap_readwrite(0x0000, 0xffff);

		/* Select A (system) spaces */
		m_cpu_active_space[cpunum] = MAPPING_A;

		install_peripherals(cpunum);

		m_irq_address[cpunum][0] = space->read_byte(0xfff8);
		m_irq_address[cpunum][1] = space->read_byte(0xfff9);
	}

	// TODO - we need to detect empty disk drive!!
	m_fdc_status |= FDC_STATUS_READY;

	/* CMI-07 */
	m_cmi07_ctrl = 0;
	m_cmi07cpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);

	m_cmi10_scnd_timer->adjust(attotime::from_hz(4000000 / 4 / 2048 / 2), 0, attotime::from_hz(4000000 / 4 / 2048 / 2));
	m_scnd = 0;

	m_ank_irqa = 0;
	m_ank_irqb = 0;
	m_q133_acia_irq = 0;
	m_cmi02_ptm_irq = 0;
	m_m6809_bs_hack_cnt = 0;
	m_m6809_bs_hack_cpu = 0;
}

void cmi_state::machine_start()
{
	m_q133_rom = (uint8_t *)m_q133_region->base();

	// allocate timers for the built-in two channel timer
	m_map_switch_timer = timer_alloc(TIMER_MAP_SWITCH);
	m_hblank_timer = timer_alloc(TIMER_HBLANK);
	m_cmi10_scnd_timer = timer_alloc(TIMER_CMI10_SCND);
	m_jam_timeout_timer = timer_alloc(TIMER_JAM_TIMEOUT);

	m_map_switch_timer->adjust(attotime::never);
	m_hblank_timer->adjust(attotime::never);
	m_cmi10_scnd_timer->adjust(attotime::never);
	m_jam_timeout_timer->adjust(attotime::never);

	/* Allocate 1kB memory mapping RAM */
	m_map_ram[0] = std::make_unique<uint8_t[]>(0x400);
	m_map_ram[1] = std::make_unique<uint8_t[]>(0x400);

	/* Allocate 256kB for each Q256 RAM card */
	m_q256_ram[0] = std::make_unique<uint8_t[]>(0x40000);
	m_q256_ram[1] = std::make_unique<uint8_t[]>(0x40000);

	/* Allocate 16kB video RAM */
	m_video_ram = std::make_unique<uint8_t[]>(0x4000);

	/* Allocate 512B shared RAM */
	m_shared_ram = std::make_unique<uint8_t[]>(0x200);

	/* Allocate 256B scratch RAM per CPU */
	m_scratch_ram[0] = std::make_unique<uint8_t[]>(0x100);
	m_scratch_ram[1] = std::make_unique<uint8_t[]>(0x100);

	m_msm5832->cs_w(1);
}

INTERRUPT_GEN_MEMBER( cmi_state::cmi_iix_vblank )
{
	/* VSYNC */
	m_q219_pia->cb2_w(1);
	m_q219_pia->cb2_w(0);

	/* LPSTB */
	m_q219_pia->cb1_w(0);
}

static SLOT_INTERFACE_START( cmi2x_floppies )
	SLOT_INTERFACE( "8dsdd", FLOPPY_8_DSDD )
	SLOT_INTERFACE( "8dssd", FLOPPY_8_DSSD )
SLOT_INTERFACE_END

static MACHINE_CONFIG_START( cmi2x )
	MCFG_CPU_ADD("maincpu1", M6809E, Q209_CPU_CLOCK)
	MCFG_CPU_PROGRAM_MAP(maincpu1_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", cmi_state, cmi_iix_vblank)
	MCFG_CPU_IRQ_ACKNOWLEDGE_DRIVER(cmi_state, cpu1_interrupt_callback)
	MCFG_QUANTUM_PERFECT_CPU("maincpu1")

	MCFG_CPU_ADD("maincpu2", M6809E, Q209_CPU_CLOCK)
	MCFG_CPU_PROGRAM_MAP(maincpu2_map)
	MCFG_CPU_IRQ_ACKNOWLEDGE_DRIVER(cmi_state, cpu2_interrupt_callback)
	MCFG_QUANTUM_PERFECT_CPU("maincpu2")

	MCFG_CPU_ADD("muskeys", M6802, 3840000)
	MCFG_CPU_PROGRAM_MAP(muskeys_map)

	MCFG_CPU_ADD("alphakeys", M6802, 3840000)
	MCFG_CPU_PROGRAM_MAP(alphakeys_map)
	MCFG_CPU_PERIODIC_INT_DRIVER(cmi_state, irq0_line_hold, 9600) // TODO: PIA controls this

	MCFG_CPU_ADD("smptemidi", M68000, 10000000)
	MCFG_CPU_PROGRAM_MAP(midicpu_map)

	MCFG_CPU_ADD("cmi07cpu", M6809E, 4000000) // ?
	MCFG_CPU_PROGRAM_MAP(cmi07cpu_map)

	/* alpha-numeric display */
	MCFG_DEVICE_ADD("dp1", DL1416T, 0)
	MCFG_DL1416_UPDATE_HANDLER(WRITE16(cmi_state, cmi_iix_update_dp1))
	MCFG_DEVICE_ADD("dp2", DL1416T, 0)
	MCFG_DL1416_UPDATE_HANDLER(WRITE16(cmi_state, cmi_iix_update_dp2))
	MCFG_DEVICE_ADD("dp3", DL1416T, 0)
	MCFG_DL1416_UPDATE_HANDLER(WRITE16(cmi_state, cmi_iix_update_dp3))

	/* video hardware */
	MCFG_SCREEN_ADD_MONOCHROME("screen", RASTER, rgb_t::green())
	MCFG_SCREEN_RAW_PARAMS(PIXEL_CLOCK, HTOTAL, HBLANK_END, HBLANK_START, VTOTAL, VBLANK_END, VBLANK_START)
	MCFG_SCREEN_UPDATE_DRIVER(cmi_state, screen_update_cmi2x)
	MCFG_PALETTE_ADD_MONOCHROME("palette")

	MCFG_MSM5832_ADD("msm5832", XTAL_32_768kHz)

	MCFG_DEVICE_ADD("i8214_1", I8214, 1000000) // cmi_8214_intf_1
	MCFG_I8214_IRQ_CALLBACK(WRITELINE(cmi_state, i8214_1_int_w))
	MCFG_DEVICE_ADD("i8214_2", I8214, 1000000) // cmi_8214_intf_2
	MCFG_I8214_IRQ_CALLBACK(WRITELINE(cmi_state, i8214_2_int_w))
	MCFG_DEVICE_ADD("i8214_3", I8214, 1000000) // cmi_8214_intf_3
	MCFG_I8214_IRQ_CALLBACK(WRITELINE(cmi_state, i8214_3_int_w))
	MCFG_I8214_ENLG_CALLBACK(WRITELINE(cmi_state, i8214_3_enlg))

	MCFG_DEVICE_ADD("q133_pia_1", PIA6821, 0) // pia_q133_1_config
	MCFG_PIA_READPA_HANDLER(READ8(cmi_state, q133_1_porta_r));
	MCFG_PIA_WRITEPA_HANDLER(WRITE8(cmi_state, q133_1_porta_w));
	MCFG_PIA_WRITEPB_HANDLER(WRITE8(cmi_state, q133_1_portb_w));

	MCFG_DEVICE_ADD("q133_pia_2", PIA6821, 0) // pia_q133_2_config
	MCFG_DEVICE_ADD("q133_ptm", PTM6840, 2000000) // ptm_q133_config
	MCFG_PTM6840_EXTERNAL_CLOCKS(1024, 1, 111) // Third is todo

	MCFG_DEVICE_ADD("q219_pia", PIA6821, 0) // pia_q219_config
	MCFG_PIA_READPB_HANDLER(READ8(cmi_state, pia_q219_b_r));
	MCFG_PIA_WRITEPA_HANDLER(WRITE8(cmi_state, vscroll_w))
	MCFG_PIA_WRITEPB_HANDLER(WRITE8(cmi_state, video_attr_w))
	MCFG_PIA_IRQA_HANDLER(WRITELINE(cmi_state, pia_q219_irqa))
	MCFG_PIA_IRQB_HANDLER(WRITELINE(cmi_state, pia_q219_irqb))

	MCFG_DEVICE_ADD("q219_ptm", PTM6840, 2000000) // ptm_q219_config
	MCFG_PTM6840_EXTERNAL_CLOCKS(HBLANK_FREQ, VBLANK_FREQ, 1000000)
	MCFG_PTM6840_IRQ_CB(WRITELINE(cmi_state, ptm_q219_irq))

	MCFG_DEVICE_ADD("cmi02_pia_1", PIA6821, 0) // pia_cmi02_1_config
	MCFG_PIA_WRITEPB_HANDLER(WRITE8(cmi_state, master_tune_w))

	MCFG_DEVICE_ADD("cmi02_pia_2", PIA6821, 0) // pia_cmi02_2_config

	MCFG_DEVICE_ADD("cmi02_ptm", PTM6840, 2000000) // ptm_cmi02_config TODO
	MCFG_PTM6840_OUT1_CB(WRITELINE(cmi_state, cmi02_ptm_o1))
	MCFG_PTM6840_IRQ_CB(WRITELINE(cmi_state, cmi02_ptm_irq))

	MCFG_DEVICE_ADD("mkbd_acia_clock", CLOCK, 9600*16)
	MCFG_CLOCK_SIGNAL_HANDLER(WRITELINE(cmi_state, mkbd_acia_clock))

	MCFG_DEVICE_ADD("q133_acia_0", MOS6551, XTAL_1_8432MHz)
	MCFG_MOS6551_XTAL(XTAL_1_8432MHz)
	MCFG_MOS6551_IRQ_HANDLER(WRITELINE(cmi_state, q133_acia_irq0))

	MCFG_DEVICE_ADD("q133_acia_1", MOS6551, XTAL_1_8432MHz)
	MCFG_MOS6551_XTAL(XTAL_1_8432MHz)
	MCFG_MOS6551_IRQ_HANDLER(WRITELINE(cmi_state, q133_acia_irq1))

	MCFG_DEVICE_ADD("q133_acia_2", MOS6551, XTAL_1_8432MHz)
	MCFG_MOS6551_XTAL(XTAL_1_8432MHz)
	MCFG_MOS6551_IRQ_HANDLER(WRITELINE(cmi_state, q133_acia_irq2))

	MCFG_DEVICE_ADD("q133_acia_3", MOS6551, XTAL_1_8432MHz)
	MCFG_MOS6551_XTAL(XTAL_1_8432MHz)
	MCFG_MOS6551_IRQ_HANDLER(WRITELINE(cmi_state, q133_acia_irq3))

	MCFG_DEVICE_ADD("acia_mkbd_kbd", ACIA6850, XTAL_1_8432MHz / 12) // acia_mkbd_kbd
	MCFG_DEVICE_ADD("acia_mkbd_cmi", ACIA6850, XTAL_1_8432MHz / 12) // acia_mkbd_cmi
	MCFG_DEVICE_ADD("ank_pia", PIA6821, 0) // pia_ank_config

	MCFG_DEVICE_MODIFY("q133_acia_0")
	MCFG_MOS6551_TXD_HANDLER(DEVWRITELINE("acia_mkbd_cmi", acia6850_device, write_rxd))
	MCFG_MOS6551_RTS_HANDLER(DEVWRITELINE("acia_mkbd_cmi", acia6850_device, write_cts))

	MCFG_DEVICE_MODIFY("acia_mkbd_cmi")
	MCFG_ACIA6850_TXD_HANDLER(DEVWRITELINE("q133_acia_0", mos6551_device, write_rxd))
	MCFG_ACIA6850_RTS_HANDLER(DEVWRITELINE("q133_acia_0", mos6551_device, write_cts))
	MCFG_ACIA6850_IRQ_HANDLER(WRITELINE(cmi_state, mkbd_cmi_acia_int))

	MCFG_DEVICE_MODIFY("acia_mkbd_kbd")
	MCFG_ACIA6850_TXD_HANDLER(DEVWRITELINE("ank_pia", pia6821_device, cb2_w))
	MCFG_ACIA6850_RTS_HANDLER(DEVWRITELINE("ank_pia", pia6821_device, ca2_w))
	MCFG_ACIA6850_IRQ_HANDLER(WRITELINE(cmi_state, mkbd_kbd_acia_int))

	MCFG_INPUT_MERGER_ACTIVE_HIGH("irqs")
	MCFG_INPUT_MERGER_OUTPUT_HANDLER(INPUTLINE("alphakeys", M6802_IRQ_LINE))

	MCFG_DEVICE_MODIFY("ank_pia")
	MCFG_PIA_READPA_HANDLER(READ8(cmi_state, ank_col_r))
	MCFG_PIA_READCB1_HANDLER(READLINE(cmi_state, ank_rts_r))
	MCFG_PIA_CA2_HANDLER(DEVWRITELINE("acia_mkbd_kbd", acia6850_device, write_cts))
	MCFG_PIA_CB2_HANDLER(DEVWRITELINE("acia_mkbd_kbd", acia6850_device, write_rxd))
	MCFG_PIA_IRQA_HANDLER(WRITELINE(cmi_state, ank_irqa_w))
	MCFG_PIA_IRQB_HANDLER(WRITELINE(cmi_state, ank_irqb_w))

	MCFG_DEVICE_ADD("ank_pia_clock", CLOCK, 9600)
	MCFG_CLOCK_SIGNAL_HANDLER(DEVWRITELINE("ank_pia", pia6821_device, ca1_w))

	MCFG_DEVICE_ADD("cmi07_ptm", PTM6840, 2000000) // ptm_cmi07_config TODO
	MCFG_PTM6840_IRQ_CB(WRITELINE(cmi_state, cmi07_irq))

	MCFG_FD1791_ADD("wd1791", XTAL_16MHz / 8) // wd1791_interface
	MCFG_WD_FDC_INTRQ_CALLBACK(WRITELINE(cmi_state, wd1791_irq))
	MCFG_WD_FDC_DRQ_CALLBACK(WRITELINE(cmi_state, wd1791_drq))
	MCFG_FLOPPY_DRIVE_ADD("wd1791:0", cmi2x_floppies, "8dsdd", floppy_image_device::default_floppy_formats)
	MCFG_FLOPPY_DRIVE_ADD("wd1791:1", cmi2x_floppies, "8dsdd", floppy_image_device::default_floppy_formats)

	/* Musical keyboard */
	MCFG_DEVICE_ADD("cmi10_pia_u20", PIA6821, 0)
	MCFG_PIA_READCB1_HANDLER(READLINE(cmi_state, cmi10_u20_cb1_r))
	MCFG_PIA_WRITEPA_HANDLER(WRITE8(cmi_state, cmi10_u20_a_w))
	MCFG_PIA_WRITEPB_HANDLER(WRITE8(cmi_state, cmi10_u20_b_w))
	MCFG_PIA_CB2_HANDLER(WRITELINE(cmi_state, cmi10_u20_cb2_w))

	MCFG_DEVICE_ADD("cmi10_pia_u21", PIA6821, 0)
	MCFG_PIA_READPA_HANDLER(READ8(cmi_state, cmi10_u21_a_r))
	MCFG_PIA_CB2_HANDLER(WRITELINE(cmi_state, cmi10_u21_cb2_w))

	MCFG_SPEAKER_STANDARD_MONO("mono")

	// Channel cards
	MCFG_CMI01A_ADD("cmi01a_0", 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
	MCFG_CMI01A_ADD("cmi01a_1", 1)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
	MCFG_CMI01A_ADD("cmi01a_2", 2)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
	MCFG_CMI01A_ADD("cmi01a_3", 3)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
	MCFG_CMI01A_ADD("cmi01a_4", 4)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
	MCFG_CMI01A_ADD("cmi01a_5", 5)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
	MCFG_CMI01A_ADD("cmi01a_6", 6)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
	MCFG_CMI01A_ADD("cmi01a_7", 7)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
MACHINE_CONFIG_END

ROM_START( cmi2x )
	/* Q133 Processor control card */
	ROM_REGION( 0x3000, "q133", 0 )
	ROM_LOAD( "q9f0mrk1.bin", 0x000, 0x800, CRC(16f195cc) SHA1(fcc4be370ba60ae5a4145c36cdbdc97a7be91f8f) )
	ROM_LOAD( "f8lmrk5.bin",  0x800, 0x800, CRC(cfc7967f) SHA1(0695cc757cf6fab35414dc068dd2a3e50084685c) )

	/* For CPU1 */
	ROM_COPY( "q133", 0x000, 0x1000, 0x800 )
	ROM_COPY( "q133", 0x800, 0x1800, 0x400 )

	/* For CPU2 */
	ROM_COPY( "q133", 0x000, 0x2000, 0x800 )
	ROM_COPY( "q133", 0xc00, 0x2800, 0x400 )

	/* General Interface (SMPTE/MIDI) CPU */
	ROM_REGION( 0x4000, "smptemidi", 0 )
	ROM_LOAD16_BYTE( "mon1110e.bin", 0x0000, 0x2000, CRC(476f7d5f) SHA1(9af21e0072eaa58cae42947c20dca05d35dfadd0) )
	ROM_LOAD16_BYTE( "mon1110o.bin", 0x0001, 0x2000, CRC(150c8ebe) SHA1(bbd371bebac29628f60537832d0587e83323ad01) )

	/* QFC9 Floppy disk controller driver */
	ROM_REGION( 0x800, "qfc9", 0 )
	ROM_LOAD( "dqfc911.bin", 0x00, 0x800, CRC(5bc38db2) SHA1(bd840e19e51a336e669c40b9e18cdaf6b3c62a8a) )

	/* Musical keyboard CPU */
	// Both of these dumps have been trimmed to size from within a roughly 2x-bigger file.
	// The actual size is known based on the format apparently used by the dumping device, shared with the prom
	// dumps and cmikeys4.bin dump.
	ROM_REGION( 0x10000, "muskeys", 0 )
	ROM_LOAD( "velkeysd.bin", 0xb000, 0x0400, CRC(9b636781) SHA1(be29a72a1d6d313dafe0b63951b5e3e18ddb9a21) )
	ROM_LOAD( "kbdioa.bin",   0xfc00, 0x0400, CRC(a5cbe218) SHA1(bc6784aaa5697c28eab126e20500139b8d0c1f50) )

	/* Alphanumeric keyboard CPU */
	// This dump has been trimmed to size from within a roughly 2x-bigger file. The actual size is known based
	// on the format apparently used by the dumping device, shared with the prom dumps and music keys dump.
	ROM_REGION( 0x10000, "alphakeys", 0 )
	ROM_LOAD( "cmikeys4.bin", 0xc000, 0x400, CRC(b214fbe9) SHA1(8c404f58ba3e5a50aa42f761e966c74374e96cc9) )

	// All of these PROM dumps have been trimmed to size from within a roughly 2x-bigger file.
	// The actual sizes are known from the schematics and the starting address of the actual PROM data was obvious
	// based on repeated data in some of the 256x4 PROMs, but it would be nice to get redumps, in the extremely
	// unlikely event that someone finds a CMI IIx for sale.
	ROM_REGION( 0x420, "proms", 0 )
	ROM_LOAD( "brom.bin",   0x000, 0x100, CRC(3f730d15) SHA1(095df6eee95b9ad6418b910fb5d2ae46913750f9) ) // Unknown use, lightgun/graphics card
	ROM_LOAD( "srom.bin",   0x100, 0x100, CRC(a1b4b71b) SHA1(6ea96480af2f1e43967f209218a74fc17972ce0e) ) // Used to generate signal timing for lightpen
	ROM_LOAD( "mrom.bin",   0x200, 0x100, CRC(dc26642c) SHA1(49b207ff80d1b055c3b855dc954129846c49bfe3) ) // Unknown use, master card
	ROM_LOAD( "timrom.bin", 0x300, 0x100, CRC(a426e4a2) SHA1(6b7ea128c730f5afd1042820ccd55bbda683afd8) ) // Unknown use, master card
	ROM_LOAD( "wrom.bin",   0x400, 0x020, CRC(68a9e17f) SHA1(c3364a37a8d19a1882d7910add1c1df9b63ee32c) ) // Unknown use, lightgun/graphics card
ROM_END

/* TODO: Machine start? */
DRIVER_INIT_MEMBER( cmi_state, cmi2x )
{
}

CONS( 1983, cmi2x, 0, 0, cmi2x, cmi2x, cmi_state, cmi2x, "Fairlight", "CMI IIx", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
