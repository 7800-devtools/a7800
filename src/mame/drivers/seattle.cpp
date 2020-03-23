// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/*************************************************************************

    Driver for Atari/Midway Phoenix/Seattle/Flagstaff hardware games

    driver by Aaron Giles

    Games supported:
        * Wayne Gretzky's 3d Hockey    [Phoenix, Atari, ~100MHz, 4MB RAM, 1xTMU]

        * Bio Freaks                   [Seattle, Midway, ???MHz, 8MB RAM, 1xTMU]
        * CarnEvil                     [Seattle, Midway, 150MHz, 8MB RAM, 1xTMU]
        * NFL Blitz                    [Seattle, Midway, 150MHz, 8MB RAM, 1xTMU]
        * NFL Blitz 99                 [Seattle, Midway, 150MHz, 8MB RAM, 1xTMU]
        * NFL Blitz 2000               [Seattle, Midway, 150MHz, 8MB RAM, 1xTMU]
        * Mace: The Dark Age           [Seattle, Atari,  200MHz, 8MB RAM, 1xTMU]

        * California Speed             [Seattle + Widget, Atari, 150MHz, 8MB RAM, 1xTMU]
        * Vapor TRX                    [Seattle + Widget, Atari, 192MHz, 8MB RAM, 1xTMU]
        * Hyperdrive                   [Seattle + Widget, Midway, 200MHz, 8MB RAM, 1xTMU]

        * San Francisco Rush           [Flagstaff, Atari, 192MHz, 2xTMU]
        * San Francisco Rush: The Rock [Flagstaff, Atari, 192MHz, 8MB RAM, 2xTMU]

    Known bugs:
        * Carnevil: lets you set the flash brightness; need to emulate that

***************************************************************************

    Phoenix hardware main board:

        * 100MHz R4700 main CPU (50MHz system clock)
        * Galileo GT64010 system controller
        * National Semiconductor PC87415 IDE controller
        * 3dfx FBI with 2MB frame buffer
        * 3dfx TMU with 4MB txture memory
        * Midway I/O ASIC
        * 4MB DRAM for main CPU
        * 512KB boot ROM
        * 16MHz ADSP 2115 audio CPU
        * 4MB DRAM for audio CPU
        * 32KB boot ROM

    Seattle hardware main board:

        * 144MHz/150MHz/192MHz/200MHz R5000 main CPU (system clock 48MHz/50MHz)
        * Galileo GT64010 system controller
        * National Semiconductor PC87415 IDE controller
        * 3dfx FBI with 2MB frame buffer
        * 3dfx TMU with 4MB txture memory
        * Midway I/O ASIC
        * 8MB DRAM for main CPU
        * 512KB boot ROM
        * 16MHz ADSP 2115 audio CPU
        * 4MB DRAM for audio CPU
        * 32KB boot ROM

    Flagstaff hardware main board:

        * 200MHz R5000 main CPU (system clock 50MHz)
        * Galileo GT64010 system controller
        * National Semiconductor PC87415 IDE controller
        * SMC91C94 ethernet controller
        * ADC0848 8 x A-to-D converters
        * 3dfx FBI with 2MB frame buffer
        * 2 x 3dfx TMU with 4MB txture memory
        * Midway I/O ASIC
        * 8MB DRAM for main CPU
        * 512KB boot ROM
        * 33MHz TMS32C031 audio CPU
        * 8MB ROM space for audio CPU
        * 512KB boot ROM

    Widget board:

        * SMC91C94 ethernet controller
        * ADC0848 8 x A-to-D converters

***************************************************************************

    Blitz '99 board:

    Seattle 5770-15206-08
        1x R5000 CPU (heatsink covering part numbers)
        1x Midway 5410-14589-00 IO chip
        1x Midway 5410-14590-00 ??
        1x Midway 5410-15349-00 Orbit 61142A ??
        1x ADSP-2115
        1x Midway security PIC Blitz 99 25" 481xxxxxx (U96)
        1x mid sized QFP (Galileo?) has heatsink (u86)
        1x mid sized QFP (PixelFX?) heakstink (u17)
        1x mid sized QFP, smaller (TexelFX?) heatsink (u87)
        12x v53c16258hk40 256Kx16 RAM (near Voodoo section)
        1x PC87415VCG IDE controller
        1x lh52256cn-70ll RAM  (near Galileo)
        1x 7can2 4k (unknown Texas Instruments SOIC)
        2x IDT 7201 LA 35J  (512x9 Async FIFO)
        1x DS232AS serial port chip
        1x Altera PLCC Seattle A-21859 (u50)
        1x Altera PLCC PAD_C1 (u60)
        3x IS61C256AH-15 (32Kx8 SRAM) near ADSP
        1x TMS418160ADZ (1Meg x 16 RAM) near ADSP
        4x TMS418160ADZ (1Meg x 16 RAM) near CPU
        1x TVP3409-17OCFN (3dfx DAC?)
        1xAD1866 audio DAC
        1x Maxim max693acwe (watchdog) near 2325 battery and Midway IO chip
        4MHz crystal attached to security PIC
        14.31818MHz crystal near 3dfx DAC
        16MHz crystal attached to ADSP
        16.6667MHz crystal near Midway IO chip
        33.3333MHz crystal near IDE chip and Galileo(PCI bus I assume)
        50MHz crystal near CPU

    Boot ROM-1.3
    Sound ROM-1.02

    Connectors:
        P2 and P6 look like PCI slots, but with no connectors soldered in, near
            3dfx/Galileo/IDE section.
        P19 is for the Daisy Dukes widget board(used by Cal Speed), and maybe
            the Carnevil gun board.
        P28 is a large 120 pin connector that is not populated, right next to
            the CPU.
        P20 is a 10 pin connector labeled "factory test".
        P1 is a 5 pin unpopulated connector marked "snd in" no idea what it
            would be for.
        P11 is a 5 pin connector marked "snd out" for line level stereo output.
        P25 is a standard IDE connector marked "Disk Drive" P15 is a laptop
            sized IDE connection right next to it.
        P9 and P10 are 14 pin connectors marked "Player 3 I/O" and player 4
            respectively.
        P16 is a 6 pin marked "Aux in"
        P3 is a 6 pin marked "Bill in"
        P8 is a 14 pin marked "Aux Latched Outputs"
        P22 is a 9 pin marked "serial port"

***************************************************************************

    Interrupt summary:

                        __________
    UART clear-to-send |          |                     __________
    -------(0x2000)--->|          |   Ethernet/Widget  |          |
                       |          |   ----(IRQ3/4/5)-->|          |
    UART data ready    |          |                    |          |
    -------(0x1000)--->|          |                    |          |
                       |          |   VSYNC            |          |
    Main-to-sound empty|  IOASIC  |   ----(IRQ3/4/5)-->|          |
    -------(0x0080)--->|          |                    |          |
                       |          |                    |          |
    Sound-to-main full |          |   IDE Controller   |   CPU    |
    -------(0x0040)--->|          |   -------(IRQ2)--->|          |
                       |          |                    |          |
    Sound FIFO empty   |          |                    |          |
    -------(0x0008)--->|          |   IOASIC Summary   |          |
                       |__________|----------(IRQ1)--->|          |
                                                       |          |
                        __________                     |          |
    Timer 3            |          |   Galileo Summary  |          |
    -------(0x0800)--->|          |----------(IRQ0)--->|          |
                       |          |                    |__________|
    Timer 2            |          |
    -------(0x0400)--->|          |
                       |          |
    Timer 1            |          |
    -------(0x0200)--->|          |
                       |          |
    Timer 0            |          |
    -------(0x0100)--->|          |
                       | Galileo  |
    DMA channel 3      |          |
    -------(0x0080)--->|          |
                       |          |
    DMA channel 2      |          |
    -------(0x0040)--->|          |
                       |          |
    DMA channel 1      |          |
    -------(0x0020)--->|          |
                       |          |
    DMA channel 0      |          |
    -------(0x0010)--->|          |
                       |__________|

**************************************************************************/

#include "emu.h"
#include "audio/cage.h"
#include "audio/dcs.h"

#include "cpu/adsp2100/adsp2100.h"
#include "cpu/mips/mips3.h"
#include "machine/midwayic.h"
#include "machine/nvram.h"
#include "machine/smc91c9x.h"

#include "machine/pci.h"
#include "machine/gt64xxx.h"
#include "machine/pci-ide.h"
#include "video/voodoo_pci.h"
#include "screen.h"

#include "calspeed.lh"
#include "vaportrx.lh"
#include "hyprdriv.lh"
#include "sfrush.lh"

/*************************************
 *
 *  Debugging constants
 *
 *************************************/

#define LOG_WIDGET          (0)



/*************************************
 *
 *  Core constants
 *
 *************************************/

#define SYSTEM_CLOCK            50000000

/* various board configurations */
#define PHOENIX_CONFIG          (0)
#define SEATTLE_CONFIG          (1)
#define SEATTLE_WIDGET_CONFIG   (2)
#define FLAGSTAFF_CONFIG        (3)

/* static interrupts */
#define GALILEO_IRQ_NUM         MIPS3_IRQ0
#define IOASIC_IRQ_NUM          MIPS3_IRQ1
#define IDE_IRQ_NUM             MIPS3_IRQ2

/* configurable interrupts */
#define ETHERNET_IRQ_SHIFT      (1)
#define WIDGET_IRQ_SHIFT        (1)
#define VBLANK_IRQ_SHIFT        (7)

/*************************************
*
*  Widget board constants
*
*************************************/

/* Widget registers */
#define WREG_ETHER_ADDR     (0x00/4)
#define WREG_INTERRUPT      (0x04/4)
#define WREG_OUTPUT         (0x0C/4)
#define WREG_ANALOG         (0x10/4)
#define WREG_ETHER_DATA     (0x14/4)

/* Widget interrupts */
#define WINT_ETHERNET_SHIFT (2)

/*************************************
*  Structures
*************************************/

struct widget_data
{
	/* ethernet register address */
	uint8_t           ethernet_addr;

	/* IRQ information */
	uint8_t           irq_num;
	uint8_t           irq_mask;
};

class seattle_state : public driver_device
{
public:
	seattle_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_nvram(*this, "nvram"),
		m_maincpu(*this, "maincpu"),
		m_cage(*this, "cage"),
		m_dcs(*this, "dcs"),
		m_ethernet(*this, "ethernet"),
		m_ioasic(*this, "ioasic")
		{}

	required_device<nvram_device> m_nvram;
	required_device<mips3_device> m_maincpu;
	optional_device<atari_cage_seattle_device> m_cage;
	optional_device<dcs_audio_device> m_dcs;
	optional_device<smc91c94_device> m_ethernet;
	required_device<midway_ioasic_device> m_ioasic;

	widget_data m_widget;
	uint32_t m_interrupt_enable;
	uint32_t m_interrupt_config;
	uint32_t m_asic_reset;
	std::vector<uint32_t> m_nvram_data;
	uint8_t m_board_config;
	uint8_t m_ethernet_irq_num;
	uint8_t m_ethernet_irq_state;
	uint8_t m_vblank_irq_num;
	uint8_t m_vblank_latch;
	uint8_t m_vblank_state;
	uint8_t m_pending_analog_read;
	uint8_t m_status_leds;
	uint32_t m_cmos_write_enabled;
	uint32_t m_output;
	uint8_t m_output_mode;
	DECLARE_READ32_MEMBER(interrupt_state_r);
	DECLARE_READ32_MEMBER(interrupt_state2_r);
	DECLARE_READ32_MEMBER(interrupt_config_r);
	DECLARE_WRITE32_MEMBER(interrupt_config_w);
	DECLARE_READ32_MEMBER(seattle_interrupt_enable_r);
	DECLARE_WRITE32_MEMBER(seattle_interrupt_enable_w);
	DECLARE_WRITE32_MEMBER(vblank_clear_w);
	DECLARE_READ32_MEMBER(analog_port_r);
	DECLARE_WRITE32_MEMBER(analog_port_w);
	DECLARE_READ32_MEMBER(carnevil_gun_r);
	DECLARE_WRITE32_MEMBER(carnevil_gun_w);
	DECLARE_WRITE32_MEMBER(cmos_w);
	DECLARE_READ32_MEMBER(cmos_r);
	DECLARE_WRITE32_MEMBER(cmos_protect_w);
	DECLARE_READ32_MEMBER(cmos_protect_r);
	DECLARE_WRITE32_MEMBER(seattle_watchdog_w);
	DECLARE_READ32_MEMBER(asic_reset_r);
	DECLARE_WRITE32_MEMBER(asic_reset_w);
	DECLARE_WRITE32_MEMBER(asic_fifo_w);
	DECLARE_READ32_MEMBER(status_leds_r);
	DECLARE_WRITE32_MEMBER(status_leds_w);
	DECLARE_READ32_MEMBER(ethernet_r);
	DECLARE_WRITE32_MEMBER(ethernet_w);
	DECLARE_READ32_MEMBER(output_r);
	DECLARE_WRITE32_MEMBER(output_w);
	DECLARE_READ32_MEMBER(widget_r);
	DECLARE_WRITE32_MEMBER(widget_w);
	DECLARE_WRITE32_MEMBER(wheel_board_w);


	DECLARE_WRITE_LINE_MEMBER(ide_interrupt);
	DECLARE_WRITE_LINE_MEMBER(vblank_assert);

	DECLARE_DRIVER_INIT(sfrush);
	DECLARE_DRIVER_INIT(blitz2k);
	DECLARE_DRIVER_INIT(carnevil);
	DECLARE_DRIVER_INIT(biofreak);
	DECLARE_DRIVER_INIT(calspeed);
	DECLARE_DRIVER_INIT(sfrushrk);
	DECLARE_DRIVER_INIT(vaportrx);
	DECLARE_DRIVER_INIT(hyprdriv);
	DECLARE_DRIVER_INIT(blitz);
	DECLARE_DRIVER_INIT(wg3dh);
	DECLARE_DRIVER_INIT(mace);
	DECLARE_DRIVER_INIT(blitz99);
	virtual void machine_start() override;
	virtual void machine_reset() override;

	DECLARE_WRITE_LINE_MEMBER(ethernet_interrupt);
	DECLARE_WRITE_LINE_MEMBER(ioasic_irq);
	void update_vblank_irq();
	void update_galileo_irqs();
	void widget_reset();
	void update_widget_irq();
	void init_common(int config);

};


/*************************************
 *
 *  Machine init
 *
 *************************************/

void seattle_state::machine_start()
{
	/* set the fastest DRC options, but strict verification */
	m_maincpu->mips3drc_set_options(MIPS3DRC_FASTEST_OPTIONS + MIPS3DRC_STRICT_VERIFY);

	/* configure fast RAM regions */
	//m_maincpu->add_fastram(0x00000000, 0x007fffff, FALSE, m_rambase);
	//m_maincpu->add_fastram(0x1fc00000, 0x1fc7ffff, TRUE,  m_rombase);

	save_item(NAME(m_widget.ethernet_addr));
	save_item(NAME(m_widget.irq_num));
	save_item(NAME(m_widget.irq_mask));
	save_item(NAME(m_interrupt_enable));
	save_item(NAME(m_interrupt_config));
	save_item(NAME(m_asic_reset));
	save_item(NAME(m_board_config));
	save_item(NAME(m_ethernet_irq_num));
	save_item(NAME(m_ethernet_irq_state));
	save_item(NAME(m_vblank_irq_num));
	save_item(NAME(m_vblank_latch));
	save_item(NAME(m_vblank_state));
	save_item(NAME(m_pending_analog_read));
	save_item(NAME(m_status_leds));
	save_item(NAME(m_cmos_write_enabled));
	save_item(NAME(m_output));
	save_item(NAME(m_output_mode));
}


void seattle_state::machine_reset()
{
	m_vblank_state = 0;
	m_vblank_irq_num = 0;
	m_interrupt_config = 0;
	m_interrupt_enable = 0;

	/* reset either the DCS2 board or the CAGE board */
	if (machine().device("dcs") != nullptr)
	{
		m_dcs->reset_w(1);
		m_dcs->reset_w(0);
	}
	else if (machine().device("cage") != nullptr)
	{
		m_cage->control_w(0);
		m_cage->control_w(3);
	}

	if (m_board_config == SEATTLE_WIDGET_CONFIG)
		widget_reset();
}

/*************************************
*
*  Ethernet interrupts
*
*************************************/

WRITE_LINE_MEMBER(seattle_state::ethernet_interrupt)
{
	m_ethernet_irq_state = state;
	if (m_board_config == FLAGSTAFF_CONFIG)
	{
		uint8_t assert = m_ethernet_irq_state && (m_interrupt_enable & (1 << ETHERNET_IRQ_SHIFT));
		if (m_ethernet_irq_num != 0)
			m_maincpu->set_input_line(m_ethernet_irq_num, assert ? ASSERT_LINE : CLEAR_LINE);
	}
	else if (m_board_config == SEATTLE_WIDGET_CONFIG)
		update_widget_irq();
}

/*************************************
*
*  I/O ASIC interrupts
*
*************************************/

WRITE_LINE_MEMBER(seattle_state::ioasic_irq)
{
	m_maincpu->set_input_line(IOASIC_IRQ_NUM, state);
}

WRITE32_MEMBER(seattle_state::wheel_board_w)
{
	//logerror("wheel_board_w: data = %08x\n", data);
	/* two writes in pairs. flag off first, on second. arg remains the same. */
	bool flag = (data & (1 << 11));
	uint8_t op = (data >> 8) & 0x7;
	uint8_t arg = data & 0xff;

	if (flag)
	{
		switch (op)
		{
		case 0x0:
			machine().output().set_value("wheel", arg); // target wheel angle. signed byte.
			break;

		case 0x4:
			for (uint8_t bit = 0; bit < 8; bit++)
				machine().output().set_lamp_value(bit, (arg >> bit) & 0x1);
			break;

		case 0x5:
			for (uint8_t bit = 0; bit < 8; bit++)
				machine().output().set_lamp_value(8 + bit, (arg >> bit) & 0x1);
			break;
		}
	}
}

/*************************************
*
*  Configurable interrupts
*
*************************************/

READ32_MEMBER(seattle_state::interrupt_state_r)
{
	uint32_t result = 0;
	result |= m_ethernet_irq_state << ETHERNET_IRQ_SHIFT;
	result |= m_vblank_latch << VBLANK_IRQ_SHIFT;
	return result;
}


READ32_MEMBER(seattle_state::interrupt_state2_r)
{
	uint32_t result = interrupt_state_r(space, offset, mem_mask);
	result |= m_vblank_state << 8;
	return result;
}


READ32_MEMBER(seattle_state::interrupt_config_r)
{
	return m_interrupt_config;
}

WRITE32_MEMBER(seattle_state::interrupt_config_w)
{
	int irq;
	COMBINE_DATA(&m_interrupt_config);

	/* VBLANK: clear anything pending on the old IRQ */
	if (m_vblank_irq_num != 0)
		m_maincpu->set_input_line(m_vblank_irq_num, CLEAR_LINE);

	/* VBLANK: compute the new IRQ vector */
	irq = (m_interrupt_config >> (2 * VBLANK_IRQ_SHIFT)) & 3;
	m_vblank_irq_num = (irq != 0) ? (2 + irq) : 0;

	/* Widget board case */
	if (m_board_config == SEATTLE_WIDGET_CONFIG)
	{
		/* Widget: clear anything pending on the old IRQ */
		if (m_widget.irq_num != 0)
			m_maincpu->set_input_line(m_widget.irq_num, CLEAR_LINE);

		/* Widget: compute the new IRQ vector */
		irq = (m_interrupt_config >> (2 * WIDGET_IRQ_SHIFT)) & 3;
		m_widget.irq_num = (irq != 0) ? (2 + irq) : 0;
	}

	/* Flagstaff board case */
	if (m_board_config == FLAGSTAFF_CONFIG)
	{
		/* Ethernet: clear anything pending on the old IRQ */
		if (m_ethernet_irq_num != 0)
			m_maincpu->set_input_line(m_ethernet_irq_num, CLEAR_LINE);

		/* Ethernet: compute the new IRQ vector */
		irq = (m_interrupt_config >> (2 * ETHERNET_IRQ_SHIFT)) & 3;
		m_ethernet_irq_num = (irq != 0) ? (2 + irq) : 0;
	}

	/* update the states */
	update_vblank_irq();
	ethernet_interrupt(m_ethernet_irq_state);
}


READ32_MEMBER(seattle_state::seattle_interrupt_enable_r)
{
	return m_interrupt_enable;
}

WRITE32_MEMBER(seattle_state::seattle_interrupt_enable_w)
{
	uint32_t old = m_interrupt_enable;
	COMBINE_DATA(&m_interrupt_enable);
	if (old != m_interrupt_enable)
	{
		if (m_vblank_latch)
			update_vblank_irq();
		if (m_ethernet_irq_state)
			ethernet_interrupt(m_ethernet_irq_state);
		//logerror("seattle_state::seattle_interrupt_enable_w = %08X m_interrupt_config=%08X\n", m_interrupt_enable, m_interrupt_config);
	}
}

/*************************************
*
*  VBLANK interrupts
*
*************************************/

void seattle_state::update_vblank_irq()
{
	int state = CLEAR_LINE;

	/* skip if no interrupt configured */
	if (m_vblank_irq_num == 0)
		return;

	/* if the VBLANK has been latched, and the interrupt is enabled, assert */
	if (m_vblank_latch && (m_interrupt_enable & (1 << VBLANK_IRQ_SHIFT))) {
		state = ASSERT_LINE;
	}
	m_maincpu->set_input_line(m_vblank_irq_num, state);
	//logerror("seattle_state::update_vblank_irq(%i)\n", state);
}


WRITE32_MEMBER(seattle_state::vblank_clear_w)
{
	/* clear the latch and update the IRQ */
	m_vblank_latch = 0;
	update_vblank_irq();
}


WRITE_LINE_MEMBER(seattle_state::vblank_assert)
{
	/* cache the raw state */
	m_vblank_state = state;

	/* latch on the correct polarity transition */
	if ((state && !(m_interrupt_enable & 0x100)) || (!state && (m_interrupt_enable & 0x100)))
	{
		m_vblank_latch = 1;
		update_vblank_irq();
	}
}
/*************************************
*
*  Analog input handling (ADC0848)
*
*************************************/

READ32_MEMBER(seattle_state::analog_port_r)
{
	return m_pending_analog_read;
}


WRITE32_MEMBER(seattle_state::analog_port_w)
{
	static const char *const portnames[] = { "AN0", "AN1", "AN2", "AN3", "AN4", "AN5", "AN6", "AN7" };

	if (data < 8 || data > 15)
		logerror("%08X:Unexpected analog port select = %08X\n", space.device().safe_pc(), data);
	m_pending_analog_read = ioport(portnames[data & 7])->read();
}

/*************************************
*
*  CarnEvil gun handling
*
*************************************/

READ32_MEMBER(seattle_state::carnevil_gun_r)
{
	uint32_t result = 0;

	switch (offset)
	{
		case 0:     /* low 8 bits of X */
			result = (ioport("LIGHT0_X")->read() << 4) & 0xff;
			break;

		case 1:     /* upper 4 bits of X */
			result = (ioport("LIGHT0_X")->read() >> 4) & 0x0f;
			result |= (ioport("FAKE")->read() & 0x03) << 4;
			result |= 0x40;
			break;

		case 2:     /* low 8 bits of Y */
			result = (ioport("LIGHT0_Y")->read() << 2) & 0xff;
			break;

		case 3:     /* upper 4 bits of Y */
			result = (ioport("LIGHT0_Y")->read() >> 6) & 0x03;
			break;

		case 4:     /* low 8 bits of X */
			result = (ioport("LIGHT1_X")->read() << 4) & 0xff;
			break;

		case 5:     /* upper 4 bits of X */
			result = (ioport("LIGHT1_X")->read() >> 4) & 0x0f;
			result |= (ioport("FAKE")->read() & 0x30);
			result |= 0x40;
			break;

		case 6:     /* low 8 bits of Y */
			result = (ioport("LIGHT1_Y")->read() << 2) & 0xff;
			break;

		case 7:     /* upper 4 bits of Y */
			result = (ioport("LIGHT1_Y")->read() >> 6) & 0x03;
			break;
	}
	return result;
}


WRITE32_MEMBER(seattle_state::carnevil_gun_w)
{
	logerror("carnevil_gun_w(%d) = %02X\n", offset, data);
}

/*************************************
*
*  Ethernet access
*
*************************************/

READ32_MEMBER(seattle_state::ethernet_r)
{
	if (!(offset & 8))
		return m_ethernet->read(space, offset & 7, mem_mask & 0xffff);
	else
		return m_ethernet->read(space, offset & 7, mem_mask & 0x00ff);
}


WRITE32_MEMBER(seattle_state::ethernet_w)
{
	if (!(offset & 8))
		m_ethernet->write(space, offset & 7, data & 0xffff, mem_mask | 0xffff);
	else
		m_ethernet->write(space, offset & 7, data & 0x00ff, mem_mask | 0x00ff);
}

/*************************************
*
*  Widget board access
*
*************************************/

void seattle_state::widget_reset()
{
	uint8_t saved_irq = m_widget.irq_num;
	memset(&m_widget, 0, sizeof(m_widget));
	m_widget.irq_num = saved_irq;
}


void seattle_state::update_widget_irq()
{
	uint8_t state = m_ethernet_irq_state << WINT_ETHERNET_SHIFT;
	uint8_t mask = m_widget.irq_mask;
	uint8_t assert = ((mask & state) != 0) && (m_interrupt_enable & (1 << WIDGET_IRQ_SHIFT));

	/* update the IRQ state */
	if (m_widget.irq_num != 0)
		m_maincpu->set_input_line(m_widget.irq_num, assert ? ASSERT_LINE : CLEAR_LINE);
}


READ32_MEMBER(seattle_state::output_r)
{
	return m_output;
}


WRITE32_MEMBER(seattle_state::output_w)
{
	uint8_t op = (data >> 8) & 0xF;
	uint8_t arg = data & 0xFF;

	switch (op)
	{
		default:
			logerror("Unknown output (%02X) = %02X\n", op, arg);
			break;

		case 0xF: break; // sync/security wrapper commands. arg matches the wrapped command.

		case 0x7:
			m_output_mode = arg;
			break;

		case 0xB:
			switch (m_output_mode)
			{
				default:
					logerror("Unknown output with mode (%02X) = %02X\n", m_output_mode, arg);
					break;

				case 0x04:
					output().set_value("wheel", arg); // wheel motor delta. signed byte.
					break;

				case 0x05:
					for (uint8_t bit = 0; bit < 8; bit++)
						output().set_lamp_value(bit, (arg >> bit) & 0x1);
					break;

				case 0x06: // Hyperdrive LEDs 0-7
					for (uint8_t bit = 0; bit < 8; bit++)
						output().set_led_value(bit, (arg >> bit) & 0x1);
					break;

				case 0x07: // Hyperdrive LEDs 8-15
					for (uint8_t bit = 0; bit < 8; bit++)
						output().set_led_value(8 + bit, (arg >> bit) & 0x1);
					break;

				case 0x08: // Hyperdrive LEDs 16-23 (Only uses up to 19)
					for (uint8_t bit = 0; bit < 8; bit++)
						output().set_led_value(16 + bit, (arg >> bit) & 0x1);
					break;
			}
			break;
	}
}


READ32_MEMBER(seattle_state::widget_r)
{
	uint32_t result = ~0;

	switch (offset)
	{
		case WREG_ETHER_ADDR:
			result = m_widget.ethernet_addr;
			break;

		case WREG_INTERRUPT:
			result = m_ethernet_irq_state << WINT_ETHERNET_SHIFT;
			result = ~result;
			break;

		case WREG_OUTPUT:
			result = output_r(m_maincpu->space(AS_PROGRAM), 0, mem_mask);
			break;

		case WREG_ANALOG:
			result = analog_port_r(m_maincpu->space(AS_PROGRAM), 0, mem_mask);
			break;

		case WREG_ETHER_DATA:
			result = m_ethernet->read(space, m_widget.ethernet_addr & 7, mem_mask & 0xffff);
			break;
	}

	if (LOG_WIDGET)
		logerror("Widget read (%02X) = %08X & %08X\n", offset*4, result, mem_mask);
	return result;
}


WRITE32_MEMBER(seattle_state::widget_w)
{
	if (LOG_WIDGET)
		logerror("Widget write (%02X) = %08X & %08X\n", offset*4, data, mem_mask);

	switch (offset)
	{
		case WREG_ETHER_ADDR:
			m_widget.ethernet_addr = data;
			break;

		case WREG_INTERRUPT:
			m_widget.irq_mask = data;
			update_widget_irq();
			break;

		case WREG_OUTPUT:
			output_w(m_maincpu->space(AS_PROGRAM), 0, data, mem_mask);
			break;

		case WREG_ANALOG:
			analog_port_w(m_maincpu->space(AS_PROGRAM), 0, data, mem_mask);
			break;

		case WREG_ETHER_DATA:
			m_ethernet->write(space, m_widget.ethernet_addr & 7, data & 0xffff, mem_mask & 0xffff);
			break;
	}
}


/*************************************
*
*  CMOS access
*
*************************************/

WRITE32_MEMBER(seattle_state::cmos_w)
{
	if (m_cmos_write_enabled)
		COMBINE_DATA(&m_nvram_data[offset]);
	m_cmos_write_enabled = false;
}


READ32_MEMBER(seattle_state::cmos_r)
{
	return m_nvram_data[offset];
}


WRITE32_MEMBER(seattle_state::cmos_protect_w)
{
	m_cmos_write_enabled = true;
}


READ32_MEMBER(seattle_state::cmos_protect_r)
{
	return m_cmos_write_enabled;
}



/*************************************
*
*  Misc accesses
*
*************************************/

WRITE32_MEMBER(seattle_state::seattle_watchdog_w)
{
	space.device().execute().eat_cycles(100);
}

READ32_MEMBER(seattle_state::asic_reset_r)
{
	return m_asic_reset;
}

WRITE32_MEMBER(seattle_state::asic_reset_w)
{
	COMBINE_DATA(&m_asic_reset);
	if (!(m_asic_reset & 0x0002))
		m_ioasic->ioasic_reset();
}


WRITE32_MEMBER(seattle_state::asic_fifo_w)
{
	m_ioasic->fifo_w(data);
}


READ32_MEMBER(seattle_state::status_leds_r)
{
	return m_status_leds | 0xffffff00;
}


WRITE32_MEMBER(seattle_state::status_leds_w)
{
	if (ACCESSING_BITS_0_7)
		m_status_leds = data;
}

/*************************************
*
*  Memory maps
*
*************************************/

/*

    WG3DH config:

RAS[1:0] = 00000000-007FFFFF
  RAS[0] = 00000000-001FFFFF
  RAS[1] = 00200000-003FFFFF

RAS[3:2] = 04000000-07FFFFFF
  RAS[2] = 04000000-05FFFFFF
  RAS[3] = 06000000-07FFFFFF

PCI I/O  = 0A000000-0BFFFFFF
PCI Mem  = 08000000-09FFFFFF

 CS[2:0] = 10000000-15FFFFFF
   CS[0] = 10000000-11FFFFFF
   CS[1] = 12000000-13FFFFFF
   CS[2] = 14000000-15FFFFFF

 CS[3]/B = 16000000-1FFFFFFF
   CS[3] = 16000000-17FFFFFF



    Carnevil config:

RAS[1:0] = 00000000-03FFFFFF
  RAS[0] = 00000000-00BFFFFF
  RAS[1] = 00100000-03FFFFFF

RAS[3:2] = 04000000-07FFFFFF
  RAS[2] = 04000000-05FFFFFF
  RAS[3] = 06000000-07FFFFFF

PCI I/O  = 0A000000-0BFFFFFF
PCI Mem  = 08000000-09FFFFFF

 CS[2:0] = 10000000-15FFFFFF
   CS[0] = 10000000-11FFFFFF
   CS[1] = 12000000-13FFFFFF
   CS[2] = 14000000-15FFFFFF

 CS[3]/B = 16000000-1FFFFFFF
   CS[3] = 16000000-17FFFFFF



    SFRush config:

RAS[1:0] = 00000000-007FFFFF
  RAS[0] = 00000000-007FFFFF
  RAS[1] = 00080000-00AFFFFF

RAS[3:2] = 04000000-07FFFFFF
  RAS[2] = 04000000-05FFFFFF
  RAS[3] = 06000000-07FFFFFF

PCI I/O  = 0A000000-0BFFFFFF
PCI Mem  = 08000000-09FFFFFF

 CS[2:0] = 10000000-15FFFFFF
   CS[0] = 10000000-11FFFFFF
   CS[1] = 12000000-13FFFFFF
   CS[2] = 14000000-15FFFFFF

 CS[3]/B = 16000000-1FFFFFFF
   CS[3] = 16000000-17FFFFFF

*/

static ADDRESS_MAP_START(seattle_cs0_map, AS_PROGRAM, 32, seattle_state)
ADDRESS_MAP_END

static ADDRESS_MAP_START(seattle_cs1_map, AS_PROGRAM, 32, seattle_state)
	AM_RANGE(0x01000000, 0x01000003) AM_WRITE(asic_fifo_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START(seattle_cs2_map, AS_PROGRAM, 32, seattle_state)
	AM_RANGE(0x00000000, 0x00000003) AM_READWRITE(analog_port_r, analog_port_w)  // Flagstaff only
ADDRESS_MAP_END

// This map shares the PHOENIX, SEATTLE, and SEATTLE_WIDGET calls
static ADDRESS_MAP_START(seattle_cs3_map, AS_PROGRAM, 32, seattle_state)
	AM_RANGE(0x00000000, 0x0000003f) AM_DEVREADWRITE("ioasic", midway_ioasic_device, read, write)
	AM_RANGE(0x00100000, 0x0011ffff) AM_READWRITE(cmos_r, cmos_w)
	AM_RANGE(0x00800000, 0x0080001f) AM_READWRITE(carnevil_gun_r, carnevil_gun_w) // Carnevil driver only
	AM_RANGE(0x00c00000, 0x00c0001f) AM_READWRITE(widget_r, widget_w); // Seattle widget only
	AM_RANGE(0x01000000, 0x01000003) AM_READWRITE(cmos_protect_r, cmos_protect_w)
	AM_RANGE(0x01100000, 0x01100003) AM_WRITE(seattle_watchdog_w)
	AM_RANGE(0x01300000, 0x01300003) AM_READWRITE(seattle_interrupt_enable_r, seattle_interrupt_enable_w)
	AM_RANGE(0x01400000, 0x01400003) AM_READWRITE(interrupt_config_r, interrupt_config_w)
	AM_RANGE(0x01500000, 0x01500003) AM_READ(interrupt_state_r)
	AM_RANGE(0x01600000, 0x01600003) AM_READ(interrupt_state2_r)
	AM_RANGE(0x01700000, 0x01700003) AM_WRITE(vblank_clear_w)
	AM_RANGE(0x01800000, 0x01800003) AM_NOP
	AM_RANGE(0x01900000, 0x01900003) AM_READWRITE(status_leds_r, status_leds_w)
	AM_RANGE(0x01f00000, 0x01f00003) AM_READWRITE(asic_reset_r, asic_reset_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START(seattle_flagstaff_cs3_map, AS_PROGRAM, 32, seattle_state)
	AM_RANGE(0x00000000, 0x0000003f) AM_DEVREADWRITE("ioasic", midway_ioasic_device, read, write)
	AM_RANGE(0x00100000, 0x0011ffff) AM_READWRITE(cmos_r, cmos_w)
	AM_RANGE(0x00c00000, 0x00c0003f) AM_READWRITE(ethernet_r, ethernet_w);
	AM_RANGE(0x01000000, 0x01000003) AM_READWRITE(cmos_protect_r, cmos_protect_w)
	AM_RANGE(0x01100000, 0x01100003) AM_WRITE(seattle_watchdog_w)
	AM_RANGE(0x01300000, 0x01300003) AM_READWRITE(seattle_interrupt_enable_r, seattle_interrupt_enable_w)
	AM_RANGE(0x01400000, 0x01400003) AM_READWRITE(interrupt_config_r, interrupt_config_w)
	AM_RANGE(0x01500000, 0x01500003) AM_READ(interrupt_state_r)
	AM_RANGE(0x01600000, 0x01600003) AM_READ(interrupt_state2_r)
	AM_RANGE(0x01700000, 0x01700003) AM_WRITE(vblank_clear_w)
	AM_RANGE(0x01800000, 0x01800003) AM_NOP
	AM_RANGE(0x01900000, 0x01900003) AM_READWRITE(status_leds_r, status_leds_w)
	AM_RANGE(0x01f00000, 0x01f00003) AM_READWRITE(asic_reset_r, asic_reset_w)
ADDRESS_MAP_END

/*************************************
 *
 *  Common input ports
 *
 *************************************/

static INPUT_PORTS_START( seattle_common )
	PORT_START("DIPS")
	PORT_DIPNAME( 0x0001, 0x0001, "Unknown0001" )
	PORT_DIPSETTING(      0x0001, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0002, 0x0002, "Unknown0002" )
	PORT_DIPSETTING(      0x0002, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0004, 0x0004, "Unknown0004" )
	PORT_DIPSETTING(      0x0004, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0008, 0x0008, "Unknown0008" )
	PORT_DIPSETTING(      0x0008, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0010, 0x0010, "Unknown0010" )
	PORT_DIPSETTING(      0x0010, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0020, 0x0020, "Unknown0020" )
	PORT_DIPSETTING(      0x0020, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0040, 0x0040, "Unknown0040" )
	PORT_DIPSETTING(      0x0040, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0080, 0x0080, "Unknown0080" )
	PORT_DIPSETTING(      0x0080, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0100, 0x0100, "Unknown0100" )
	PORT_DIPSETTING(      0x0100, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0200, 0x0200, "Unknown0200" )
	PORT_DIPSETTING(      0x0200, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0400, 0x0400, "Unknown0400" )
	PORT_DIPSETTING(      0x0400, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0800, 0x0800, "Unknown0800" )
	PORT_DIPSETTING(      0x0800, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x1000, 0x1000, "Unknown1000" )
	PORT_DIPSETTING(      0x1000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x2000, 0x2000, "Unknown2000" )
	PORT_DIPSETTING(      0x2000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x4000, 0x4000, "Unknown4000" )
	PORT_DIPSETTING(      0x4000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x8000, 0x8000, "Unknown8000" )
	PORT_DIPSETTING(      0x8000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )

	PORT_START("SYSTEM")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_TILT )
	PORT_SERVICE_NO_TOGGLE( 0x0010, IP_ACTIVE_LOW )
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_COIN3 )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_COIN4 )
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_START3 )
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_START4 )
	PORT_BIT( 0x0800, IP_ACTIVE_LOW, IPT_VOLUME_DOWN )
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_VOLUME_UP )
	PORT_BIT( 0x6000, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_BILL1 )

	PORT_START("IN1")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY PORT_PLAYER(1)
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_PLAYER(1)
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_PLAYER(1)
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_PLAYER(1)
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(1)
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(1)
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(1)
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_PLAYER(1)   /* 3d cam */
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x0800, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)
	PORT_BIT( 0x2000, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(2)
	PORT_BIT( 0x4000, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_PLAYER(2)

	PORT_START("IN2")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY PORT_PLAYER(3)
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_PLAYER(3)
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_PLAYER(3)
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_PLAYER(3)
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(3)
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(3)
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(3)
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY PORT_PLAYER(4)
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_PLAYER(4)
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_PLAYER(4)
	PORT_BIT( 0x0800, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_PLAYER(4)
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(4)
	PORT_BIT( 0x2000, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(4)
	PORT_BIT( 0x4000, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(4)
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_UNUSED )
INPUT_PORTS_END



/*************************************
 *
 *  Input ports
 *
 *************************************/

static INPUT_PORTS_START( wg3dh )
	PORT_INCLUDE(seattle_common)

	PORT_MODIFY("DIPS")
	PORT_SERVICE_NO_TOGGLE( 0x0001, IP_ACTIVE_LOW )
	PORT_DIPNAME( 0x0002, 0x0002, "Boot ROM Test" )
	PORT_DIPSETTING(      0x0002, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
INPUT_PORTS_END


static INPUT_PORTS_START( mace )
	PORT_INCLUDE(seattle_common)

	PORT_MODIFY("DIPS")
	PORT_DIPNAME( 0x0040, 0x0040, "Boot ROM Test" )
	PORT_DIPSETTING(      0x0040, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_SERVICE_NO_TOGGLE( 0x0080, IP_ACTIVE_LOW )
	PORT_DIPNAME( 0x8000, 0x0000, "Resolution" )
	PORT_DIPSETTING(      0x8000, DEF_STR( Low ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Medium ) )
INPUT_PORTS_END


static INPUT_PORTS_START( sfrush )
	PORT_INCLUDE(seattle_common)

	PORT_MODIFY("DIPS")
	PORT_SERVICE_NO_TOGGLE( 0x0001, IP_ACTIVE_LOW )
	PORT_DIPNAME( 0x0002, 0x0002, "Boot ROM Test" )
	PORT_DIPSETTING(      0x0002, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )

	PORT_MODIFY("SYSTEM")
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_START1 ) PORT_NAME("Abort") PORT_PLAYER(1) /* Abort */
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_BUTTON11 ) PORT_NAME(DEF_STR( Reverse )) PORT_PLAYER(1)    /* reverse */
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_START3 )
	PORT_BIT( 0x0800, IP_ACTIVE_LOW, IPT_START4 )  /* There are no start buttons on a Rush! */
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0xe000, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_MODIFY("IN1")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_BUTTON8 ) PORT_NAME("View 1") PORT_PLAYER(1)   /* view 1 */
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_BUTTON9 ) PORT_NAME("View 2") PORT_PLAYER(1)   /* view 2 */
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_BUTTON10 ) PORT_NAME("View 3") PORT_PLAYER(1)  /* view 3 */
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_BUTTON12 ) PORT_NAME("Music") PORT_PLAYER(1)   /* music */
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_BUTTON13 ) PORT_NAME("Track 1") PORT_PLAYER(1) /* track 1 */
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_BUTTON14 ) PORT_NAME("Track 2") PORT_PLAYER(1) /* track 2 */
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_BUTTON15 ) PORT_NAME("Track 3") PORT_PLAYER(1) /* track 3 */
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_BUTTON16 ) PORT_NAME("Track 4") PORT_PLAYER(1) /* track 4 */
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_NAME("1st Gear") PORT_PLAYER(1) /* 1st gear */
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_BUTTON5 ) PORT_NAME("2nd Gear") PORT_PLAYER(1) /* 2nd gear */
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_BUTTON6 ) PORT_NAME("3rd Gear") PORT_PLAYER(1) /* 3rd gear */
	PORT_BIT( 0x0800, IP_ACTIVE_LOW, IPT_BUTTON7 ) PORT_NAME("4th Gear") PORT_PLAYER(1) /* 4th gear */
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_VOLUME_DOWN )
	PORT_BIT( 0x2000, IP_ACTIVE_LOW, IPT_VOLUME_UP )
	PORT_BIT( 0x4000, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_MODIFY("IN2")
	PORT_BIT( 0xffff, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("AN0")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )

	PORT_START("AN1")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )

	PORT_START("AN2")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )

	PORT_START("AN3")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )

	PORT_START("AN4")   /* Accel */
	PORT_BIT( 0xff, 0x00, IPT_PEDAL ) PORT_SENSITIVITY(25) PORT_KEYDELTA(20) PORT_PLAYER(1)

	PORT_START("AN5")   /* Brake */
	PORT_BIT( 0xff, 0x00, IPT_PEDAL2 ) PORT_SENSITIVITY(25) PORT_KEYDELTA(100) PORT_PLAYER(1)

	PORT_START("AN6")   /* Clutch */
	PORT_BIT( 0xff, 0x00, IPT_PEDAL3 ) PORT_SENSITIVITY(25) PORT_KEYDELTA(100) PORT_PLAYER(1)

	PORT_START("AN7")   /* Steer */
	PORT_BIT( 0xff, 0x80, IPT_PADDLE ) PORT_MINMAX(0x10,0xf0) PORT_SENSITIVITY(25) PORT_KEYDELTA(5)
INPUT_PORTS_END


static INPUT_PORTS_START( sfrushrk )
	PORT_INCLUDE(sfrush)

	PORT_MODIFY("DIPS")
	PORT_DIPNAME( 0x0001, 0x0001, "Calibrate at startup" )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0001, DEF_STR( On ) )
	PORT_DIPNAME( 0x0002, 0x0002, "Unknown0002" )
	PORT_DIPSETTING(      0x0002, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0040, 0x0040, "Boot ROM Test" )
	PORT_DIPSETTING(      0x0040, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_SERVICE_NO_TOGGLE( 0x0080, IP_ACTIVE_LOW )
INPUT_PORTS_END


static INPUT_PORTS_START( calspeed )
	PORT_INCLUDE(seattle_common)

	PORT_MODIFY("DIPS")
	PORT_DIPNAME( 0x0040, 0x0040, "Boot ROM Test" )
	PORT_DIPSETTING(      0x0040, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_SERVICE_NO_TOGGLE( 0x0080, IP_ACTIVE_LOW )

	PORT_MODIFY("SYSTEM")
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_SERVICE2 ) /* test */
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_START3 )
	PORT_BIT( 0xe000, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_MODIFY("IN1")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_BUTTON10 ) PORT_NAME("Radio") PORT_PLAYER(1)   /* radio */
	PORT_BIT( 0x000c, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_BUTTON7 ) PORT_NAME("View 1") PORT_PLAYER(1)   /* road cam */
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_BUTTON8 ) PORT_NAME("View 2") PORT_PLAYER(1)   /* tailgate cam */
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_BUTTON9 ) PORT_NAME("View 3") PORT_PLAYER(1)   /* sky cam */
	PORT_BIT( 0x0f80, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_NAME("1st Gear") PORT_PLAYER(1) /* 1st gear */
	PORT_BIT( 0x2000, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_NAME("2nd Gear") PORT_PLAYER(1) /* 2nd gear */
	PORT_BIT( 0x4000, IP_ACTIVE_LOW, IPT_BUTTON5 ) PORT_NAME("3rd Gear") PORT_PLAYER(1) /* 3rd gear */
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_BUTTON6 ) PORT_NAME("4th Gear") PORT_PLAYER(1) /* 4th gear */

	PORT_MODIFY("IN2")
	PORT_BIT( 0xffff, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("AN0")   /* Steer */
	PORT_BIT( 0xff, 0x80, IPT_PADDLE ) PORT_MINMAX(0x10,0xf0) PORT_SENSITIVITY(25) PORT_KEYDELTA(5)

	PORT_START("AN1")   /* Accel */
	PORT_BIT( 0xff, 0x00, IPT_PEDAL ) PORT_SENSITIVITY(25) PORT_KEYDELTA(20) PORT_PLAYER(1)

	PORT_START("AN2")   /* Brake */
	PORT_BIT( 0xff, 0x00, IPT_PEDAL2 ) PORT_SENSITIVITY(25) PORT_KEYDELTA(100) PORT_PLAYER(1)

	PORT_START("AN3")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )

	PORT_START("AN4")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )

	PORT_START("AN5")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )

	PORT_START("AN6")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )

	PORT_START("AN7")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )
INPUT_PORTS_END


static INPUT_PORTS_START( vaportrx )
	PORT_INCLUDE(seattle_common)

	PORT_MODIFY("DIPS")
	PORT_DIPNAME( 0x0040, 0x0040, "Boot ROM Test" )
	PORT_DIPSETTING(      0x0040, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_SERVICE_NO_TOGGLE( 0x0080, IP_ACTIVE_LOW )

	PORT_MODIFY("SYSTEM")
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(1)   /* left trigger */
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_SERVICE2 )                 /* test */
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_START3 )
	PORT_BIT( 0x0800, IP_ACTIVE_LOW, IPT_VOLUME_DOWN )
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_VOLUME_UP )
	PORT_BIT( 0xe000, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_MODIFY("IN1")
	PORT_BIT( 0x000f, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_CODE(KEYCODE_A) PORT_PLAYER(1)  /* right trigger */
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(1)                       /* left thumb */
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_BUTTON5 ) PORT_CODE(KEYCODE_S) PORT_PLAYER(1)  /* right thumb */
	PORT_BIT( 0x0180, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(1)                       /* left view */
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0800, IP_ACTIVE_LOW, IPT_BUTTON6 ) PORT_CODE(KEYCODE_Q) PORT_PLAYER(1)  /* right view */
	PORT_BIT( 0xf000, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_MODIFY("IN2")
	PORT_BIT( 0xffff, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("AN0")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )

	PORT_START("AN1")
	PORT_BIT( 0xff, 0x80, IPT_AD_STICK_Y ) PORT_SENSITIVITY(100) PORT_KEYDELTA(10) PORT_PLAYER(1)

	PORT_START("AN2")
	PORT_BIT( 0xff, 0x80, IPT_AD_STICK_X ) PORT_SENSITIVITY(100) PORT_KEYDELTA(10) PORT_PLAYER(1)

	PORT_START("AN3")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )

	PORT_START("AN4")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )

	PORT_START("AN5")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )

	PORT_START("AN6")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )

	PORT_START("AN7")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )
INPUT_PORTS_END


static INPUT_PORTS_START( biofreak )
	PORT_INCLUDE(seattle_common)

	PORT_MODIFY("DIPS")
	PORT_DIPNAME( 0x0001, 0x0001, "Hilink download??" )
	PORT_DIPSETTING(      0x0001, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0002, 0x0002, "Boot ROM Test" )
	PORT_DIPSETTING(      0x0002, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )

	PORT_MODIFY("IN1")
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(1)   /* LP = P1 left punch */
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(1)   /* F  = P1 ??? */
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(1)   /* RP = P1 right punch */
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)   /* LP = P1 left punch */
	PORT_BIT( 0x2000, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(2)   /* F  = P1 ??? */
	PORT_BIT( 0x4000, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)   /* RP = P1 right punch */
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_MODIFY("IN2")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_PLAYER(1)   /* LK = P1 left kick */
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_BUTTON5 ) PORT_PLAYER(1)   /* RK = P1 right kick */
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_BUTTON6 ) PORT_PLAYER(1)   /* T  = P1 ??? */
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_PLAYER(2)   /* LK = P2 left kick */
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_BUTTON5 ) PORT_PLAYER(2)   /* RK = P2 right kick */
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_BUTTON6 ) PORT_PLAYER(2)   /* T  = P2 ??? */
	PORT_BIT( 0xff80, IP_ACTIVE_LOW, IPT_UNUSED )
INPUT_PORTS_END


static INPUT_PORTS_START( blitz )
	PORT_INCLUDE(seattle_common)

	PORT_MODIFY("DIPS")
	PORT_DIPNAME( 0x0001, 0x0000, "Coinage Source" )
	PORT_DIPSETTING(      0x0001, "Dipswitch" )
	PORT_DIPSETTING(      0x0000, "CMOS" )
	PORT_DIPNAME( 0x000e, 0x000e, DEF_STR( Coinage ))
	PORT_DIPSETTING(      0x000e, "Mode 1" )
	PORT_DIPSETTING(      0x0008, "Mode 2" )
	PORT_DIPSETTING(      0x0009, "Mode 3" )
	PORT_DIPSETTING(      0x0002, "Mode 4" )
	PORT_DIPSETTING(      0x000c, "Mode ECA" )
//  PORT_DIPSETTING(      0x0004, "Not Used 1" )        /* Marked as Unused in the manual */
//  PORT_DIPSETTING(      0x0008, "Not Used 2" )        /* Marked as Unused in the manual */
	PORT_DIPSETTING(      0x0000, DEF_STR( Free_Play ))
	PORT_DIPNAME( 0x0030, 0x0030, "Curency Type" )
	PORT_DIPSETTING(      0x0030, DEF_STR( USA ) )
	PORT_DIPSETTING(      0x0020, DEF_STR( French ) )
	PORT_DIPSETTING(      0x0010, DEF_STR( German ) )
//  PORT_DIPSETTING(      0x0000, "Not Used" )      /* Marked as Unused in the manual */
	PORT_DIPSETTING(      0x0000, DEF_STR( Free_Play ))
	PORT_DIPNAME( 0x0040, 0x0000, DEF_STR( Unknown ))   /* Marked as Unused in the manual */
	PORT_DIPSETTING(      0x0040, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0080, 0x0080, "Power Up Test Loop" )
	PORT_DIPSETTING(      0x0080, "One Time" )
	PORT_DIPSETTING(      0x0000, "Continuous" )
	PORT_DIPNAME( 0x0100, 0x0100, "Joysticks" )
	PORT_DIPSETTING(      0x0100, "8-Way" )
	PORT_DIPSETTING(      0x0000, "49-Way" )
	PORT_DIPNAME( 0x0600, 0x0200, "Graphics Mode" )
	PORT_DIPSETTING(      0x0200, "512x385 @ 25KHz" )
	PORT_DIPSETTING(      0x0400, "512x256 @ 15KHz" )
//  PORT_DIPSETTING(      0x0600, "0" )         /* Marked as Unused in the manual */
//  PORT_DIPSETTING(      0x0000, "3" )         /* Marked as Unused in the manual */
	PORT_DIPNAME( 0x1800, 0x1800, "Graphics Speed" )
	PORT_DIPSETTING(      0x0000, "45 MHz" )
	PORT_DIPSETTING(      0x0800, "47 MHz" )
	PORT_DIPSETTING(      0x1000, "49 MHz" )
	PORT_DIPSETTING(      0x1800, "51 MHz" )
	PORT_DIPNAME( 0x2000, 0x0000, "Bill Validator" )
	PORT_DIPSETTING(      0x2000, DEF_STR( None ) )
	PORT_DIPSETTING(      0x0000, "One" )
	PORT_DIPNAME( 0x4000, 0x0000, "Power On Self Test" )
	PORT_DIPSETTING(      0x0000, DEF_STR( No ))
	PORT_DIPSETTING(      0x4000, DEF_STR( Yes ))
	PORT_DIPNAME( 0x8000, 0x8000, "Test Switch" )
	PORT_DIPSETTING(      0x8000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )

	PORT_MODIFY("IN1")
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_MODIFY("IN2")
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_UNUSED )
INPUT_PORTS_END


static INPUT_PORTS_START( blitz99 )
	PORT_INCLUDE(blitz)

	PORT_MODIFY("DIPS")
	PORT_DIPNAME( 0x003e, 0x003e, DEF_STR( Coinage ))
	PORT_DIPSETTING(      0x003e, "USA 1" )
	PORT_DIPSETTING(      0x003c, "USA 2" )
	PORT_DIPSETTING(      0x003a, "USA 3" )
	PORT_DIPSETTING(      0x0038, "USA 4" )
	PORT_DIPSETTING(      0x0036, "USA 5" )
	PORT_DIPSETTING(      0x0034, "USA 6" )
	PORT_DIPSETTING(      0x0032, "USA 7" )
	PORT_DIPSETTING(      0x0030, "USA ECA" )
	PORT_DIPSETTING(      0x002e, "France 1" )
	PORT_DIPSETTING(      0x002c, "France 2" )
	PORT_DIPSETTING(      0x002a, "France 3" )
	PORT_DIPSETTING(      0x0028, "France 4" )
	PORT_DIPSETTING(      0x0026, "France 5" )
	PORT_DIPSETTING(      0x0024, "France 6" )
	PORT_DIPSETTING(      0x0022, "France 7" )
	PORT_DIPSETTING(      0x0020, "France ECA" )
	PORT_DIPSETTING(      0x001e, "German 1" )
	PORT_DIPSETTING(      0x001c, "German 2" )
	PORT_DIPSETTING(      0x001a, "German 3" )
	PORT_DIPSETTING(      0x0018, "German 4" )
	PORT_DIPSETTING(      0x0016, "German 5" )
//  PORT_DIPSETTING(      0x0014, "German 5" )
//  PORT_DIPSETTING(      0x0012, "German 5" )
	PORT_DIPSETTING(      0x0010, "German ECA" )
	PORT_DIPSETTING(      0x000e, "U.K. 1 ECA" )
	PORT_DIPSETTING(      0x000c, "U.K. 2 ECA" )
	PORT_DIPSETTING(      0x000a, "U.K. 3 ECA" )
	PORT_DIPSETTING(      0x0008, "U.K. 4" )
	PORT_DIPSETTING(      0x0006, "U.K. 5" )
	PORT_DIPSETTING(      0x0004, "U.K. 6 ECA" )
	PORT_DIPSETTING(      0x0002, "U.K. 7 ECA" )
	PORT_DIPSETTING(      0x0000, DEF_STR( Free_Play ))
	PORT_DIPNAME( 0x0040, 0x0000, DEF_STR( Unknown ))
	PORT_DIPSETTING(      0x0040, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x2000, 0x0000, DEF_STR( Players ) )
	PORT_DIPSETTING(      0x2000, "2" )
	PORT_DIPSETTING(      0x0000, "4" )
INPUT_PORTS_END


static INPUT_PORTS_START( carnevil )
	PORT_INCLUDE( seattle_common )

	PORT_MODIFY("DIPS")
	PORT_DIPNAME( 0x0001, 0x0000, "Coinage Source" )
	PORT_DIPSETTING(      0x0001, "Dipswitch" )
	PORT_DIPSETTING(      0x0000, "CMOS" )
	PORT_DIPNAME( 0x003e, 0x003e, DEF_STR( Coinage ))
	PORT_DIPSETTING(      0x003e, "USA 1" )
	PORT_DIPSETTING(      0x003c, "USA 2" )
	PORT_DIPSETTING(      0x003a, "USA 3" )
	PORT_DIPSETTING(      0x0038, "USA 4" )
	PORT_DIPSETTING(      0x0036, "USA 5" )
	PORT_DIPSETTING(      0x0034, "USA 6" )
	PORT_DIPSETTING(      0x0032, "USA 7" )
	PORT_DIPSETTING(      0x0030, "USA ECA" )
	PORT_DIPSETTING(      0x002e, "France 1" )
	PORT_DIPSETTING(      0x002c, "France 2" )
	PORT_DIPSETTING(      0x002a, "France 3" )
	PORT_DIPSETTING(      0x0028, "France 4" )
	PORT_DIPSETTING(      0x0026, "France 5" )
	PORT_DIPSETTING(      0x0024, "France 6" )
	PORT_DIPSETTING(      0x0022, "France 7" )
	PORT_DIPSETTING(      0x0020, "France ECA" )
	PORT_DIPSETTING(      0x001e, "German 1" )
	PORT_DIPSETTING(      0x001c, "German 2" )
	PORT_DIPSETTING(      0x001a, "German 3" )
	PORT_DIPSETTING(      0x0018, "German 4" )
	PORT_DIPSETTING(      0x0016, "German 5" )
//  PORT_DIPSETTING(      0x0014, "German 5" )
//  PORT_DIPSETTING(      0x0012, "German 5" )
	PORT_DIPSETTING(      0x0010, "German ECA" )
	PORT_DIPSETTING(      0x000e, "U.K. 1" )
	PORT_DIPSETTING(      0x000c, "U.K. 2" )
	PORT_DIPSETTING(      0x000a, "U.K. 3" )
	PORT_DIPSETTING(      0x0008, "U.K. 4" )
	PORT_DIPSETTING(      0x0006, "U.K. 5" )
	PORT_DIPSETTING(      0x0004, "U.K. 6" )
	PORT_DIPSETTING(      0x0002, "U.K. 7 ECA" )
	PORT_DIPSETTING(      0x0000, DEF_STR( Free_Play ))
	PORT_DIPNAME( 0x0040, 0x0000, DEF_STR( Unknown ))
	PORT_DIPSETTING(      0x0040, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0080, 0x0080, "Power Up Test Loop" )
	PORT_DIPSETTING(      0x0080, DEF_STR( No ))
	PORT_DIPSETTING(      0x0000, DEF_STR( Yes ))
	PORT_DIPNAME( 0x0100, 0x0000, DEF_STR( Unknown ))
	PORT_DIPSETTING(      0x0100, "0" )
	PORT_DIPSETTING(      0x0000, "1" )
	PORT_DIPNAME( 0x0600, 0x0400, "Resolution" )
//  PORT_DIPSETTING(      0x0600, "0" )
//  PORT_DIPSETTING(      0x0200, DEF_STR( Medium ) )
	PORT_DIPSETTING(      0x0400, DEF_STR( Low ) )
//  PORT_DIPSETTING(      0x0000, "3" )
	PORT_DIPNAME( 0x1800, 0x1800, "Graphics Speed" )
	PORT_DIPSETTING(      0x0000, "45 MHz" )
	PORT_DIPSETTING(      0x0800, "47 MHz" )
	PORT_DIPSETTING(      0x1000, "49 MHz" )
	PORT_DIPSETTING(      0x1800, "51 MHz" )
	PORT_DIPNAME( 0x2000, 0x0000, DEF_STR( Unknown ))
	PORT_DIPSETTING(      0x2000, "0" )
	PORT_DIPSETTING(      0x0000, "1" )
	PORT_DIPNAME( 0x4000, 0x0000, "Power On Self Test" )
	PORT_DIPSETTING(      0x0000, DEF_STR( No ))
	PORT_DIPSETTING(      0x4000, DEF_STR( Yes ))
	PORT_DIPNAME( 0x8000, 0x8000, "Test Switch" )
	PORT_DIPSETTING(      0x8000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )

	PORT_MODIFY("SYSTEM")
	PORT_BIT( 0x0780, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_MODIFY("IN1")
	PORT_BIT( 0xffff, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_MODIFY("IN2")
	PORT_BIT( 0xffff, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("LIGHT0_X")              /* fake analog X */
	PORT_BIT( 0xff, 0x80, IPT_LIGHTGUN_X ) PORT_CROSSHAIR(X, 1.0, 0.0, 0) PORT_SENSITIVITY(50) PORT_KEYDELTA(10)

	PORT_START("LIGHT0_Y")              /* fake analog Y */
	PORT_BIT( 0xff, 0x80, IPT_LIGHTGUN_Y ) PORT_CROSSHAIR(Y, 1.0, 0.0, 0) PORT_SENSITIVITY(70) PORT_KEYDELTA(10)

	PORT_START("LIGHT1_X")              /* fake analog X */
	PORT_BIT( 0xff, 0x80, IPT_LIGHTGUN_X ) PORT_CROSSHAIR(X, 1.0, 0.0, 0) PORT_SENSITIVITY(50) PORT_KEYDELTA(10) PORT_PLAYER(2)

	PORT_START("LIGHT1_Y")              /* fake analog Y */
	PORT_BIT( 0xff, 0x80, IPT_LIGHTGUN_Y ) PORT_CROSSHAIR(Y, 1.0, 0.0, 0) PORT_SENSITIVITY(70) PORT_KEYDELTA(10) PORT_PLAYER(2)

	PORT_START("FAKE")                  /* fake switches */
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_BUTTON1 ) PORT_PLAYER(1)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_BUTTON2 ) PORT_PLAYER(1)
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_BUTTON2 ) PORT_PLAYER(2)
INPUT_PORTS_END


static INPUT_PORTS_START( hyprdriv )
	PORT_INCLUDE( seattle_common )

	PORT_MODIFY("DIPS")
	PORT_DIPNAME( 0x0001, 0x0000, "Coinage Source" )
	PORT_DIPSETTING(      0x0001, "Dipswitch" )
	PORT_DIPSETTING(      0x0000, "CMOS" )
	PORT_DIPNAME( 0x003e, 0x0034, DEF_STR( Coinage ))
	PORT_DIPSETTING(      0x003e, "USA 10" )
	PORT_DIPSETTING(      0x003c, "USA 11" )
	PORT_DIPSETTING(      0x003a, "USA 12" )
	PORT_DIPSETTING(      0x0038, "USA 13" )
	PORT_DIPSETTING(      0x0036, "USA 9" )
	PORT_DIPSETTING(      0x0034, "USA 1" )
	PORT_DIPSETTING(      0x0032, "USA 2" )
	PORT_DIPSETTING(      0x0030, "USA ECA" )
	PORT_DIPSETTING(      0x002e, "France 1" )
	PORT_DIPSETTING(      0x002c, "France 2" )
	PORT_DIPSETTING(      0x002a, "France 3" )
	PORT_DIPSETTING(      0x0028, "France 4" )
	PORT_DIPSETTING(      0x0026, "France 5" )
	PORT_DIPSETTING(      0x0024, "France 6" )
	PORT_DIPSETTING(      0x0022, "France 7" )
	PORT_DIPSETTING(      0x0020, "France ECA" )
	PORT_DIPSETTING(      0x001e, "German 1" )
	PORT_DIPSETTING(      0x001c, "German 2" )
	PORT_DIPSETTING(      0x001a, "German 3" )
	PORT_DIPSETTING(      0x0018, "German 4" )
	PORT_DIPSETTING(      0x0016, "German 5" )
	PORT_DIPSETTING(      0x0014, "German 5" )
	PORT_DIPSETTING(      0x0012, "German 5" )
	PORT_DIPSETTING(      0x0010, "German ECA" )
	PORT_DIPSETTING(      0x000e, "U.K. 1 ECA" )
	PORT_DIPSETTING(      0x000c, "U.K. 2 ECA" )
	PORT_DIPSETTING(      0x000a, "U.K. 3 ECA" )
	PORT_DIPSETTING(      0x0008, "U.K. 4" )
	PORT_DIPSETTING(      0x0006, "U.K. 5" )
	PORT_DIPSETTING(      0x0004, "U.K. 6 ECA" )
	PORT_DIPSETTING(      0x0002, "U.K. 7 ECA" )
	PORT_DIPSETTING(      0x0000, DEF_STR( Free_Play ))
	PORT_DIPNAME( 0x0040, 0x0000, DEF_STR( Unknown ))
	PORT_DIPSETTING(      0x0040, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0080, 0x0080, "Power Up Test Loop" )
	PORT_DIPSETTING(      0x0080, DEF_STR( No ))
	PORT_DIPSETTING(      0x0000, DEF_STR( Yes ))
	PORT_DIPNAME( 0x0100, 0x0000, DEF_STR( Unknown ))
	PORT_DIPSETTING(      0x0100, "0" )
	PORT_DIPSETTING(      0x0000, "1" )
	PORT_DIPNAME( 0x0600, 0x0200, "Resolution" )
	PORT_DIPSETTING(      0x0600, "0" )
	PORT_DIPSETTING(      0x0200, DEF_STR( Medium ) )
	PORT_DIPSETTING(      0x0400, DEF_STR( Low ) )
	PORT_DIPSETTING(      0x0000, "3" )
	PORT_DIPNAME( 0x1800, 0x0000, "Graphics Speed" )
	PORT_DIPSETTING(      0x0000, "45 MHz" )
	PORT_DIPSETTING(      0x0800, "47 MHz" )
	PORT_DIPSETTING(      0x1000, "49 MHz" )
	PORT_DIPSETTING(      0x1800, "51 MHz" )
	PORT_DIPNAME( 0x2000, 0x2000, "Brake" )
	PORT_DIPSETTING(      0x2000, "Enabled" )
	PORT_DIPSETTING(      0x0000, "Disabled" )
	PORT_DIPNAME( 0x4000, 0x0000, "Power On Self Test" )
	PORT_DIPSETTING(      0x0000, DEF_STR( No ))
	PORT_DIPSETTING(      0x4000, DEF_STR( Yes ))
	PORT_DIPNAME( 0x8000, 0x8000, "Test Switch" )
	PORT_DIPSETTING(      0x8000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )

	PORT_MODIFY("IN1")
	PORT_BIT( 0x0003, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_JOYSTICKLEFT_LEFT ) PORT_2WAY PORT_PLAYER(1)
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_JOYSTICKLEFT_RIGHT ) PORT_2WAY PORT_PLAYER(1)
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(1)
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_PLAYER(1)
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_BUTTON5 ) PORT_PLAYER(1)
	PORT_BIT( 0xff80, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_MODIFY("IN2")
	PORT_BIT( 0xffff, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("AN0")
	PORT_BIT( 0x00ff, 0x80, IPT_AD_STICK_X ) PORT_MINMAX(0x10,0xf0) PORT_SENSITIVITY(25) PORT_KEYDELTA(25)
	PORT_BIT( 0xff00, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("AN1")   /* Accel */
	PORT_BIT( 0xff, 0x00, IPT_PEDAL ) PORT_SENSITIVITY(25) PORT_KEYDELTA(20) PORT_PLAYER(1)

	PORT_START("AN2")   /* Brake */
	PORT_BIT( 0xff, 0x00, IPT_PEDAL2 ) PORT_SENSITIVITY(25) PORT_KEYDELTA(100) PORT_PLAYER(1)

	PORT_START("AN3")
	PORT_BIT( 0xff, 0x80, IPT_AD_STICK_Y ) PORT_MINMAX(0x10,0xf0) PORT_SENSITIVITY(25) PORT_KEYDELTA(25)

	PORT_START("AN4")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )

	PORT_START("AN5")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )

	PORT_START("AN6")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )

	PORT_START("AN7")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )

	/* 2008-06 FP: is this ever read?? */
	PORT_START("AN8")
	PORT_BIT( 0xff, 0x80, IPT_SPECIAL )
INPUT_PORTS_END



/*************************************
 *
 *  Machine drivers
 *
 *************************************/
#define PCI_ID_GALILEO  ":pci:00.0"
#define PCI_ID_VIDEO    ":pci:08.0"
#define PCI_ID_IDE      ":pci:09.0"

static MACHINE_CONFIG_START( seattle_common )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", R5000LE, SYSTEM_CLOCK*3)
	MCFG_MIPS3_ICACHE_SIZE(16384)
	MCFG_MIPS3_DCACHE_SIZE(16384)
	MCFG_MIPS3_SYSTEM_CLOCK(SYSTEM_CLOCK)

	// PCI Bus Devices
	MCFG_PCI_ROOT_ADD(":pci")

	MCFG_GT64010_ADD(PCI_ID_GALILEO, ":maincpu", SYSTEM_CLOCK, GALILEO_IRQ_NUM)
	MCFG_GT64XXX_SET_CS(0, seattle_cs0_map)
	MCFG_GT64XXX_SET_CS(1, seattle_cs1_map)
	MCFG_GT64XXX_SET_CS(2, seattle_cs2_map)
	MCFG_GT64XXX_SET_CS(3, seattle_cs3_map)
	MCFG_GT64XX_SET_SIMM0(0x00800000)

	MCFG_IDE_PCI_ADD(PCI_ID_IDE, 0x100b0002, 0x01, 0x0)
	MCFG_IDE_PCI_IRQ_ADD(":maincpu", IDE_IRQ_NUM)
	MCFG_IDE_PCI_SET_LEGACY_TOP(0x0a0)

	MCFG_VOODOO_PCI_ADD(PCI_ID_VIDEO, TYPE_VOODOO_1, ":maincpu")
	MCFG_VOODOO_PCI_FBMEM(2)
	MCFG_VOODOO_PCI_TMUMEM(4, 0)
	MCFG_DEVICE_MODIFY(PCI_ID_VIDEO":voodoo")
	MCFG_VOODOO_VBLANK_CB(DEVWRITELINE(":", seattle_state, vblank_assert))
	MCFG_VOODOO_STALL_CB(DEVWRITELINE(PCI_ID_GALILEO, gt64xxx_device, pci_stall))


	MCFG_NVRAM_ADD_1FILL("nvram")

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(57)
	MCFG_SCREEN_SIZE(640, 480)
	MCFG_SCREEN_VISIBLE_AREA(0, 639, 0, 479)
	MCFG_SCREEN_UPDATE_DEVICE(PCI_ID_VIDEO, voodoo_pci_device, screen_update)
	/* sound hardware */
MACHINE_CONFIG_END


static MACHINE_CONFIG_DERIVED( phoenixsa, seattle_common )
	MCFG_CPU_REPLACE("maincpu", R4700LE, SYSTEM_CLOCK*2)
	MCFG_MIPS3_ICACHE_SIZE(16384)
	MCFG_MIPS3_DCACHE_SIZE(16384)
	MCFG_MIPS3_SYSTEM_CLOCK(SYSTEM_CLOCK)

	MCFG_DEVICE_MODIFY(PCI_ID_GALILEO)
	MCFG_GT64XX_SET_SIMM0(0x00200000)
	MCFG_GT64XX_SET_SIMM1(0x00200000)
MACHINE_CONFIG_END


static MACHINE_CONFIG_DERIVED( seattle150, seattle_common )
	MCFG_CPU_REPLACE("maincpu", R5000LE, SYSTEM_CLOCK*3)
	MCFG_MIPS3_ICACHE_SIZE(16384)
	MCFG_MIPS3_DCACHE_SIZE(16384)
	MCFG_MIPS3_SYSTEM_CLOCK(SYSTEM_CLOCK)
MACHINE_CONFIG_END


static MACHINE_CONFIG_DERIVED( seattle150_widget, seattle150 )
	MCFG_SMC91C94_ADD("ethernet")
	MCFG_SMC91C94_IRQ_CALLBACK(WRITELINE(seattle_state, ethernet_interrupt))
MACHINE_CONFIG_END


static MACHINE_CONFIG_DERIVED( seattle200, seattle_common )
	MCFG_CPU_REPLACE("maincpu", R5000LE, SYSTEM_CLOCK*4)
	MCFG_MIPS3_ICACHE_SIZE(16384)
	MCFG_MIPS3_DCACHE_SIZE(16384)
	MCFG_MIPS3_SYSTEM_CLOCK(SYSTEM_CLOCK)
MACHINE_CONFIG_END


static MACHINE_CONFIG_DERIVED( seattle200_widget, seattle200 )
	MCFG_SMC91C94_ADD("ethernet")
	MCFG_SMC91C94_IRQ_CALLBACK(WRITELINE(seattle_state, ethernet_interrupt))
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( flagstaff, seattle_common )
	MCFG_CPU_REPLACE("maincpu", R5000LE, SYSTEM_CLOCK*4)
	MCFG_MIPS3_ICACHE_SIZE(16384)
	MCFG_MIPS3_DCACHE_SIZE(16384)
	MCFG_MIPS3_SYSTEM_CLOCK(SYSTEM_CLOCK)

	MCFG_DEVICE_MODIFY(PCI_ID_GALILEO)
	MCFG_GT64XXX_SET_CS(3, seattle_flagstaff_cs3_map)

	MCFG_SMC91C94_ADD("ethernet")
	MCFG_SMC91C94_IRQ_CALLBACK(WRITELINE(seattle_state, ethernet_interrupt))

	MCFG_DEVICE_MODIFY(PCI_ID_VIDEO)
	MCFG_VOODOO_PCI_FBMEM(2)
	MCFG_VOODOO_PCI_TMUMEM(4, 4)
MACHINE_CONFIG_END

// Per game configurations

static MACHINE_CONFIG_DERIVED( wg3dh, phoenixsa )
	MCFG_DEVICE_ADD("dcs", DCS2_AUDIO_2115, 0)
	MCFG_DCS2_AUDIO_DRAM_IN_MB(2)
	MCFG_DCS2_AUDIO_POLLING_OFFSET(0x3839)

	MCFG_DEVICE_ADD("ioasic", MIDWAY_IOASIC, 0)
	MCFG_MIDWAY_IOASIC_SHUFFLE(MIDWAY_IOASIC_STANDARD)
	MCFG_MIDWAY_IOASIC_UPPER(310/* others? */)
	MCFG_MIDWAY_IOASIC_YEAR_OFFS(80)
	MCFG_MIDWAY_IOASIC_IRQ_CALLBACK(WRITELINE(seattle_state, ioasic_irq))
MACHINE_CONFIG_END


static MACHINE_CONFIG_DERIVED( mace, seattle150 )
	MCFG_DEVICE_ADD("dcs", DCS2_AUDIO_2115, 0)
	MCFG_DCS2_AUDIO_DRAM_IN_MB(2)
	MCFG_DCS2_AUDIO_POLLING_OFFSET(0x3839)

	MCFG_DEVICE_ADD("ioasic", MIDWAY_IOASIC, 0)
	MCFG_MIDWAY_IOASIC_SHUFFLE(MIDWAY_IOASIC_MACE)
	MCFG_MIDWAY_IOASIC_UPPER(319/* others? */)
	MCFG_MIDWAY_IOASIC_YEAR_OFFS(80)
	MCFG_MIDWAY_IOASIC_IRQ_CALLBACK(WRITELINE(seattle_state, ioasic_irq))
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( sfrush, flagstaff )
	MCFG_DEVICE_ADD("cage", ATARI_CAGE_SEATTLE, 0)
	MCFG_ATARI_CAGE_SPEEDUP(0x5236)
	MCFG_ATARI_CAGE_IRQ_CALLBACK(DEVWRITE8("ioasic",midway_ioasic_device,cage_irq_handler))

	MCFG_DEVICE_ADD("ioasic", MIDWAY_IOASIC, 0)
	MCFG_MIDWAY_IOASIC_SHUFFLE(MIDWAY_IOASIC_STANDARD)
	MCFG_MIDWAY_IOASIC_UPPER(315/* no alternates */)
	MCFG_MIDWAY_IOASIC_YEAR_OFFS(100)
	MCFG_MIDWAY_IOASIC_IRQ_CALLBACK(WRITELINE(seattle_state, ioasic_irq))
	MCFG_MIDWAY_IOASIC_AUX_OUT_CB(WRITE32(seattle_state, wheel_board_w))
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( sfrushrk, flagstaff )
	MCFG_DEVICE_ADD("cage", ATARI_CAGE_SEATTLE, 0)
	MCFG_ATARI_CAGE_SPEEDUP(0x5329)
	MCFG_ATARI_CAGE_IRQ_CALLBACK(DEVWRITE8("ioasic",midway_ioasic_device,cage_irq_handler))

	MCFG_DEVICE_ADD("ioasic", MIDWAY_IOASIC, 0)
	MCFG_MIDWAY_IOASIC_SHUFFLE(MIDWAY_IOASIC_SFRUSHRK)
	MCFG_MIDWAY_IOASIC_UPPER(331/* unknown */)
	MCFG_MIDWAY_IOASIC_YEAR_OFFS(100)
	MCFG_MIDWAY_IOASIC_IRQ_CALLBACK(WRITELINE(seattle_state, ioasic_irq))
	MCFG_MIDWAY_IOASIC_AUX_OUT_CB(WRITE32(seattle_state, wheel_board_w))
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( sfrushrkw, sfrushrk )
	MCFG_DEVICE_MODIFY("ioasic")
	MCFG_MIDWAY_IOASIC_SHUFFLE(MIDWAY_IOASIC_STANDARD)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( calspeed, seattle150_widget )
	MCFG_DEVICE_ADD("dcs", DCS2_AUDIO_2115, 0)
	MCFG_DCS2_AUDIO_DRAM_IN_MB(2)
	MCFG_DCS2_AUDIO_POLLING_OFFSET(0x39c0)

	MCFG_DEVICE_ADD("ioasic", MIDWAY_IOASIC, 0)
	MCFG_MIDWAY_IOASIC_SHUFFLE(MIDWAY_IOASIC_CALSPEED)
	MCFG_MIDWAY_IOASIC_UPPER(328/* others? */)
	MCFG_MIDWAY_IOASIC_YEAR_OFFS(100)
	MCFG_MIDWAY_IOASIC_IRQ_CALLBACK(WRITELINE(seattle_state, ioasic_irq))
	MCFG_MIDWAY_IOASIC_AUTO_ACK(1)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( vaportrx, seattle200_widget )
	MCFG_DEVICE_ADD("dcs", DCS2_AUDIO_2115, 0)
	MCFG_DCS2_AUDIO_DRAM_IN_MB(2)
	MCFG_DCS2_AUDIO_POLLING_OFFSET(0x39c2)

	MCFG_DEVICE_ADD("ioasic", MIDWAY_IOASIC, 0)
	MCFG_MIDWAY_IOASIC_SHUFFLE(MIDWAY_IOASIC_VAPORTRX)
	MCFG_MIDWAY_IOASIC_UPPER(324/* 334? unknown */)
	MCFG_MIDWAY_IOASIC_YEAR_OFFS(100)
	MCFG_MIDWAY_IOASIC_IRQ_CALLBACK(WRITELINE(seattle_state, ioasic_irq))
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( biofreak, seattle150 )
	MCFG_DEVICE_ADD("dcs", DCS2_AUDIO_2115, 0)
	MCFG_DCS2_AUDIO_DRAM_IN_MB(2)
	MCFG_DCS2_AUDIO_POLLING_OFFSET(0x3835)

	MCFG_DEVICE_ADD("ioasic", MIDWAY_IOASIC, 0)
	MCFG_MIDWAY_IOASIC_SHUFFLE(MIDWAY_IOASIC_STANDARD)
	MCFG_MIDWAY_IOASIC_UPPER(231/* no alternates */)
	MCFG_MIDWAY_IOASIC_YEAR_OFFS(80)
	MCFG_MIDWAY_IOASIC_IRQ_CALLBACK(WRITELINE(seattle_state, ioasic_irq))
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( blitz, seattle150 )
	MCFG_DEVICE_ADD("dcs", DCS2_AUDIO_2115, 0)
	MCFG_DCS2_AUDIO_DRAM_IN_MB(2)
	MCFG_DCS2_AUDIO_POLLING_OFFSET(0x39c2)

	MCFG_DEVICE_ADD("ioasic", MIDWAY_IOASIC, 0)
	MCFG_MIDWAY_IOASIC_SHUFFLE(MIDWAY_IOASIC_BLITZ99)
	MCFG_MIDWAY_IOASIC_UPPER(444/* or 528 */)
	MCFG_MIDWAY_IOASIC_YEAR_OFFS(80)
	MCFG_MIDWAY_IOASIC_IRQ_CALLBACK(WRITELINE(seattle_state, ioasic_irq))
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( blitz99, seattle150 )
	MCFG_DEVICE_ADD("dcs", DCS2_AUDIO_2115, 0)
	MCFG_DCS2_AUDIO_DRAM_IN_MB(2)
	MCFG_DCS2_AUDIO_POLLING_OFFSET(0x0afb)

	MCFG_DEVICE_ADD("ioasic", MIDWAY_IOASIC, 0)
	MCFG_MIDWAY_IOASIC_SHUFFLE(MIDWAY_IOASIC_BLITZ99)
	MCFG_MIDWAY_IOASIC_UPPER(481/* or 484 or 520 */)
	MCFG_MIDWAY_IOASIC_YEAR_OFFS(80)
	MCFG_MIDWAY_IOASIC_IRQ_CALLBACK(WRITELINE(seattle_state, ioasic_irq))
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( blitz2k, seattle150 )
	MCFG_DEVICE_ADD("dcs", DCS2_AUDIO_2115, 0)
	MCFG_DCS2_AUDIO_DRAM_IN_MB(2)
	MCFG_DCS2_AUDIO_POLLING_OFFSET(0x0b5d)

	MCFG_DEVICE_ADD("ioasic", MIDWAY_IOASIC, 0)
	MCFG_MIDWAY_IOASIC_SHUFFLE(MIDWAY_IOASIC_BLITZ99)
	MCFG_MIDWAY_IOASIC_UPPER(494/* or 498 */)
	MCFG_MIDWAY_IOASIC_YEAR_OFFS(80)
	MCFG_MIDWAY_IOASIC_IRQ_CALLBACK(WRITELINE(seattle_state, ioasic_irq))
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( carnevil, seattle150 )
	MCFG_DEVICE_ADD("dcs", DCS2_AUDIO_2115, 0)
	MCFG_DCS2_AUDIO_DRAM_IN_MB(2)
	MCFG_DCS2_AUDIO_POLLING_OFFSET(0x0af7)

	MCFG_DEVICE_ADD("ioasic", MIDWAY_IOASIC, 0)
	MCFG_MIDWAY_IOASIC_SHUFFLE(MIDWAY_IOASIC_CARNEVIL)
	MCFG_MIDWAY_IOASIC_UPPER(469/* 469 or 486 or 528 */)
	MCFG_MIDWAY_IOASIC_YEAR_OFFS(80)
	MCFG_MIDWAY_IOASIC_IRQ_CALLBACK(WRITELINE(seattle_state, ioasic_irq))
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( hyprdriv, seattle200_widget )
	MCFG_DEVICE_ADD("dcs", DCS2_AUDIO_2115, 0)
	MCFG_DCS2_AUDIO_DRAM_IN_MB(2)
	MCFG_DCS2_AUDIO_POLLING_OFFSET(0x0af7)

	MCFG_DEVICE_ADD("ioasic", MIDWAY_IOASIC, 0)
	MCFG_MIDWAY_IOASIC_SHUFFLE(MIDWAY_IOASIC_HYPRDRIV)
	MCFG_MIDWAY_IOASIC_UPPER(469/* unknown */)
	MCFG_MIDWAY_IOASIC_YEAR_OFFS(80)
	MCFG_MIDWAY_IOASIC_IRQ_CALLBACK(WRITELINE(seattle_state, ioasic_irq))
MACHINE_CONFIG_END

/*************************************
 *
 *  ROM definitions
 *
 *************************************/

ROM_START( wg3dh )
	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )  /* Boot Code Version L1.2 (10/8/96) */
	ROM_LOAD( "wg3dh_12.u32", 0x000000, 0x80000, CRC(15e4cea2) SHA1(72c0db7dc53ce645ba27a5311b5ce803ad39f131) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Hard Drive Version 1.3 (Guts 10/15/96, Main 10/15/96) */
	DISK_IMAGE( "wg3dh", 0, SHA1(4fc6f25d7f043d9bcf8743aa8df1d9be3cbc375b) )

	ROM_REGION16_LE( 0x10000, "dcs", 0 )    /* ADSP-2115 data Version L1.1 */
	ROM_LOAD16_BYTE( "soundl11.u95", 0x000000, 0x8000, CRC(c589458c) SHA1(0cf970a35910a74cdcf3bd8119bfc0c693e19b00) )
ROM_END


ROM_START( mace )
	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )  /* Boot Code Version 1.0ce 7/2/97 */
	ROM_LOAD( "mace10ce.u32", 0x000000, 0x80000, CRC(7a50b37e) SHA1(33788835f84a9443566c80bee9f20a1691490c6d) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Hard Drive Version 1.0B 6/10/97 (Guts 7/2/97, Main 7/2/97) */
	DISK_IMAGE( "mace", 0, SHA1(96ec8d3ff5dd894e21aa81403bcdbeba44bb97ea) )

	ROM_REGION16_LE( 0x10000, "dcs", 0 )    /* ADSP-2115 data Version L1.1, Labeled as Version 1.0 */
	ROM_LOAD16_BYTE( "soundl11.u95", 0x000000, 0x8000, CRC(c589458c) SHA1(0cf970a35910a74cdcf3bd8119bfc0c693e19b00) )
ROM_END


ROM_START( macea )
	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )  /* Boot Code Version ??? 5/7/97 */
	ROM_LOAD( "maceboot.u32", 0x000000, 0x80000, CRC(effe3ebc) SHA1(7af3ca3580d6276ffa7ab8b4c57274e15ee6bcbb) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Hard Drive Version 1.0a (Guts 6/9/97, Main 5/12/97) */
	DISK_IMAGE( "macea", 0, BAD_DUMP SHA1(9bd4a60627915d71932cab24f89c48ea21f4c1cb) )

	ROM_REGION16_LE( 0x10000, "dcs", 0 )    /* ADSP-2115 data Version L1.1 */
	ROM_LOAD16_BYTE( "soundl11.u95", 0x000000, 0x8000, CRC(c589458c) SHA1(0cf970a35910a74cdcf3bd8119bfc0c693e19b00) )
ROM_END


ROM_START( sfrush )
	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )  /* Boot Code Version L1.0 */
	ROM_LOAD( "hdboot.u32", 0x000000, 0x80000, CRC(39a35f1b) SHA1(c46d83448399205d38e6e41dd56abbc362254254) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	ROM_REGION32_LE( 0x200000, "cageboot", 0 )  /* TMS320C31 boot ROM  Version L1.0 */
	ROM_LOAD32_BYTE( "sndboot.u69", 0x000000, 0x080000, CRC(7e52cdc7) SHA1(f735063e19d2ca672cef6d761a2a47df272e8c59) )

	ROM_REGION32_LE( 0x1000000, "cage", 0 ) /* TMS320C31 sound ROMs */
	ROM_LOAD32_WORD( "sfrush.u62",  0x400000, 0x200000, CRC(5d66490e) SHA1(bd39ea3b45d44cae6ca5890f365653326bbecd2d) )
	ROM_LOAD32_WORD( "sfrush.u61",  0x400002, 0x200000, CRC(f3a00ee8) SHA1(c1ac780efc32b2e30522d7cc3e6d92e7daaadddd) )
	ROM_LOAD32_WORD( "sfrush.u53",  0x800000, 0x200000, CRC(71f8ddb0) SHA1(c24bef801f43bae68fda043c4356e8cf1298ca97) )
	ROM_LOAD32_WORD( "sfrush.u49",  0x800002, 0x200000, CRC(dfb0a54c) SHA1(ed34f9485f7a7e5bb73bf5c6428b27548e12db12) )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Hard Drive Version L1.06 */
	DISK_IMAGE( "sfrush", 0, SHA1(e2db0270a707fb2115207f988d5751081d6b4994) )
ROM_END

ROM_START( sfrusha )
	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )  /* Boot Code Version L1.06A */
	ROM_LOAD( "HDBOOTV1_06A.bin", 0x000000, 0x80000, CRC(f247ba60) SHA1(850f97002eb1e362c3df870d7b6a1b5524ab983d) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	ROM_REGION32_LE( 0x200000, "cageboot", 0 )  /* TMS320C31 boot ROM  Version L1.0 */
	ROM_LOAD32_BYTE( "sndboot.u69", 0x000000, 0x080000, CRC(7e52cdc7) SHA1(f735063e19d2ca672cef6d761a2a47df272e8c59) )

	ROM_REGION32_LE( 0x1000000, "cage", 0 ) /* TMS320C31 sound ROMs */
	ROM_LOAD32_WORD( "sfrush.u62",  0x400000, 0x200000, CRC(5d66490e) SHA1(bd39ea3b45d44cae6ca5890f365653326bbecd2d) )
	ROM_LOAD32_WORD( "sfrush.u61",  0x400002, 0x200000, CRC(f3a00ee8) SHA1(c1ac780efc32b2e30522d7cc3e6d92e7daaadddd) )
	ROM_LOAD32_WORD( "sfrush.u53",  0x800000, 0x200000, CRC(71f8ddb0) SHA1(c24bef801f43bae68fda043c4356e8cf1298ca97) )
	ROM_LOAD32_WORD( "sfrush.u49",  0x800002, 0x200000, CRC(dfb0a54c) SHA1(ed34f9485f7a7e5bb73bf5c6428b27548e12db12) )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Hard Drive Version L1.06 */
	DISK_IMAGE( "sfrush", 0, SHA1(e2db0270a707fb2115207f988d5751081d6b4994) )
ROM_END



ROM_START( sfrushrk )
	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )  /* Boot Code */
	ROM_LOAD( "boot.bin",   0x000000, 0x080000, CRC(0555b3cf) SHA1(a48abd6d06a26f4f9b6c52d8c0af6095b6be57fd) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	ROM_REGION32_LE( 0x200000, "cageboot", 0 )  /* TMS320C31 boot ROM */
	ROM_LOAD32_BYTE( "audboot.bin",    0x000000, 0x080000, CRC(c70c060d) SHA1(dd014bd13efdf5adc5450836bd4650351abefc46) )

	ROM_REGION32_LE( 0x1000000, "cage", 0 ) /* TMS320C31 sound ROMs */
	ROM_LOAD32_WORD( "audio.u62",  0x400000, 0x200000, CRC(cacf09e3) SHA1(349af1767cb0ee2a0eb9d7c2ab078fcae5fec8e7) )
	ROM_LOAD32_WORD( "audio.u61",  0x400002, 0x200000, CRC(ea895d29) SHA1(1edde0497f2abd1636c5d7bcfbc03bcff321261c) )
	ROM_LOAD32_WORD( "audio.u53",  0x800000, 0x200000, CRC(51c89a14) SHA1(6bc62bcda224040a4596d795132874828011a038) )
	ROM_LOAD32_WORD( "audio.u49",  0x800002, 0x200000, CRC(e6b684d3) SHA1(1f5bab7fae974cecc8756dd23e3c7aa2cf6e7dc7) )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Hard Drive Version 1.2 */
	DISK_IMAGE( "sfrushrk", 0, SHA1(e763f26aca67ebc17fe8b8df4fba91d492cf7837) )
ROM_END


ROM_START( sfrushrkw )
	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )  /* Boot Code */
	ROM_LOAD( "1ff6.bin",   0x000000, 0x080000, CRC(0b30f080) SHA1(2dfa14b927d1c185e6876b1bf464c117682331c9) ) // Hand written "Rush The Rock w/wavenet 1ff6 8/10/97"

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	ROM_REGION32_LE( 0x200000, "cageboot", 0 )  /* TMS320C31 boot ROM */
	ROM_LOAD32_BYTE( "audboot.bin",    0x000000, 0x080000, CRC(c70c060d) SHA1(dd014bd13efdf5adc5450836bd4650351abefc46) )

	ROM_REGION32_LE( 0x1000000, "cage", 0 ) /* TMS320C31 sound ROMs */
	ROM_LOAD32_WORD( "audio.u62",  0x400000, 0x200000, CRC(cacf09e3) SHA1(349af1767cb0ee2a0eb9d7c2ab078fcae5fec8e7) )
	ROM_LOAD32_WORD( "audio.u61",  0x400002, 0x200000, CRC(ea895d29) SHA1(1edde0497f2abd1636c5d7bcfbc03bcff321261c) )
	ROM_LOAD32_WORD( "audio.u53",  0x800000, 0x200000, CRC(51c89a14) SHA1(6bc62bcda224040a4596d795132874828011a038) )
	ROM_LOAD32_WORD( "audio.u49",  0x800002, 0x200000, CRC(e6b684d3) SHA1(1f5bab7fae974cecc8756dd23e3c7aa2cf6e7dc7) )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Hard Drive Version 1.2 */
	DISK_IMAGE( "sfrushrk", 0, SHA1(e763f26aca67ebc17fe8b8df4fba91d492cf7837) )
ROM_END

ROM_START( sfrushrkwo )
	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )  /* Boot Code */
	ROM_LOAD( "boottest.bin",   0x000000, 0x080000, CRC(3f83f8e0) SHA1(c1862fc35c119586f79f23c52ecea6c35c37828a) ) // Labeled "Rush The Rock Boot Eeprom Test Only"

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	ROM_REGION32_LE( 0x200000, "cageboot", 0 )  /* TMS320C31 boot ROM */
	ROM_LOAD32_BYTE( "audboot.bin",    0x000000, 0x080000, CRC(c70c060d) SHA1(dd014bd13efdf5adc5450836bd4650351abefc46) )

	ROM_REGION32_LE( 0x1000000, "cage", 0 ) /* TMS320C31 sound ROMs */
	ROM_LOAD32_WORD( "audio.u62",  0x400000, 0x200000, CRC(cacf09e3) SHA1(349af1767cb0ee2a0eb9d7c2ab078fcae5fec8e7) )
	ROM_LOAD32_WORD( "audio.u61",  0x400002, 0x200000, CRC(ea895d29) SHA1(1edde0497f2abd1636c5d7bcfbc03bcff321261c) )
	ROM_LOAD32_WORD( "audio.u53",  0x800000, 0x200000, CRC(51c89a14) SHA1(6bc62bcda224040a4596d795132874828011a038) )
	ROM_LOAD32_WORD( "audio.u49",  0x800002, 0x200000, CRC(e6b684d3) SHA1(1f5bab7fae974cecc8756dd23e3c7aa2cf6e7dc7) )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Hard Drive Version 1.2 */
	DISK_IMAGE( "sfrushrk", 0, SHA1(e763f26aca67ebc17fe8b8df4fba91d492cf7837) )
ROM_END

ROM_START( calspeed )
	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )  /* Boot Code Version 1.2 (2/18/98) */
	ROM_LOAD( "caspd1_2.u32", 0x000000, 0x80000, CRC(0a235e4e) SHA1(b352f10fad786260b58bd344b5002b6ea7aaf76d) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Release version 2.1a (4/17/98) (Guts 1.25 4/17/98, Main 4/17/98) */
	DISK_IMAGE( "calspeed", 0, SHA1(08d411c591d4b8bbdd6437ea80d01c4cec8516f8) )

	ROM_REGION16_LE( 0x10000, "dcs", 0 )    /* ADSP-2115 data Version 1.02 */
	ROM_LOAD16_BYTE( "sound102.u95", 0x000000, 0x8000, CRC(bec7d3ae) SHA1(db80aa4a645804a4574b07b9f34dec6b6b64190d) )
ROM_END

ROM_START( calspeeda )
	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )  /* Boot Code Version 1.2 (2/18/98) */
	ROM_LOAD( "caspd1_2.u32", 0x000000, 0x80000, CRC(0a235e4e) SHA1(b352f10fad786260b58bd344b5002b6ea7aaf76d) )

	// it actually asks you to replace this with the original rom after the upgrade is complete, which is weird because this is a perfectly valid newer revision of the boot code, but probably explains why the parent set was still on 1.2
	ROM_LOAD( "Cal speed update  U32 boot ver, 1,4 5EF6", 0x000000, 0x80000, CRC(fd627637) SHA1(b0c2847cbecfc00344e402386d13240d55a0814e) ) // boot code 1.4 Apr 17 1998 21:18:31

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )
	ROM_SYSTEM_BIOS( 0, "noupdate",       "No Update Rom" )

	ROM_SYSTEM_BIOS( 1, "up16_1",       "Disk Update 1.0x to 2.1a (1.25) Step 1 of 3" )
	ROMX_LOAD("eprom #1 2.1A 90A7", 0x000000, 0x100000, CRC(bc0f373e) SHA1(bf53f1953ccab8da9ce784e4d20dd2ec0d0eff6a), ROM_BIOS(2))
	ROM_SYSTEM_BIOS( 2, "up16_2",       "Disk Update 1.0x to 2.1a (1.25) Step 2 of 3" )
	ROMX_LOAD("eprom #2 2.1A 9F84", 0x000000, 0x100000, CRC(5782da30) SHA1(eaeea3655bc9c1cedefdfb0088d4716584788669), ROM_BIOS(3))
	ROM_SYSTEM_BIOS( 3, "up16_3",       "Disk Update 1.0x to 2.1a (1.25) Step 3 of 3" )
	ROMX_LOAD("eprom #3 2.1A 3286", 0x000000, 0x100000, CRC(e7d8c88f) SHA1(06c11241ac439527b361826784aef4c58689892e), ROM_BIOS(4))


	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Release version 1.0r8a (4/10/98) (Guts 4/10/98, Main 4/10/98) */
	DISK_IMAGE( "cs_10r8a", 0, SHA1(ba4e7589740e0647938c81c5082bb71d8826bad4) )

	ROM_REGION16_LE( 0x10000, "dcs", 0 )    /* ADSP-2115 data Version 1.02 */
	ROM_LOAD16_BYTE( "sound102.u95", 0x000000, 0x8000, CRC(bec7d3ae) SHA1(db80aa4a645804a4574b07b9f34dec6b6b64190d) )
ROM_END

ROM_START( calspeedb )
	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )  /* Boot Code Version 1.2 (2/18/98) */
	ROM_LOAD( "caspd1_2.u32", 0x000000, 0x80000, CRC(0a235e4e) SHA1(b352f10fad786260b58bd344b5002b6ea7aaf76d) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Release version 1.0r7a (3/4/98) (Guts 3/3/98, Main 1/19/98) */
	DISK_IMAGE( "calspeda", 0, SHA1(6b1c3a7530195ef7309b06a651b01c8b3ece92c6) )

	ROM_REGION16_LE( 0x10000, "dcs", 0 )    /* ADSP-2115 data Version 1.02 */
	ROM_LOAD16_BYTE( "sound102.u95", 0x000000, 0x8000, CRC(bec7d3ae) SHA1(db80aa4a645804a4574b07b9f34dec6b6b64190d) )
ROM_END

ROM_START( vaportrx )
	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )
	ROM_LOAD( "vtrxboot.bin", 0x000000, 0x80000, CRC(ee487a6c) SHA1(fb9efda85047cf615f24f7276a9af9fd542f3354) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )
	DISK_IMAGE( "vaportrx", 0, SHA1(fe53ca7643d2ed2745086abb7f2243c69678cab1) )

	ROM_REGION16_LE( 0x10000, "dcs", 0 )    /* ADSP-2115 data Version 1.02 */
	ROM_LOAD16_BYTE( "vaportrx.snd", 0x000000, 0x8000, CRC(bec7d3ae) SHA1(db80aa4a645804a4574b07b9f34dec6b6b64190d) )
ROM_END


ROM_START( vaportrxp )
	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )
	ROM_LOAD( "vtrxboot.bin", 0x000000, 0x80000, CRC(ee487a6c) SHA1(fb9efda85047cf615f24f7276a9af9fd542f3354) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" ) /* Guts: Apr 10 1998 11:03:14  Main: Apr 10 1998 11:27:44 */
	DISK_IMAGE( "vaportrp", 0, SHA1(6c86637c442ebd6994eee8c0ae0dce343c35dbe9) )

	ROM_REGION16_LE( 0x10000, "dcs", 0 )    /* ADSP-2115 data Version 1.02 */
	ROM_LOAD16_BYTE( "vaportrx.snd", 0x000000, 0x8000, CRC(bec7d3ae) SHA1(db80aa4a645804a4574b07b9f34dec6b6b64190d) )
ROM_END


ROM_START( biofreak )
	ROM_REGION16_LE( 0x10000, "dcs", 0 )    /* ADSP-2115 data Version 1.02 */
	ROM_LOAD16_BYTE( "sound102.u95", 0x000000, 0x8000, CRC(bec7d3ae) SHA1(db80aa4a645804a4574b07b9f34dec6b6b64190d) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 ) /* Seattle System Boot ROM Version 0.1i Apr 14 1997  14:52:53 */
	ROM_LOAD( "biofreak.u32", 0x000000, 0x80000, CRC(cefa00bb) SHA1(7e171610ede1e8a448fb8d175f9cb9e7d549de28) )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" ) /* Build Date 12/11/97 */
	DISK_IMAGE( "biofreak", 0, SHA1(711241642f92ded8eaf20c418ea748989183fe10) )
ROM_END


ROM_START( blitz )
	ROM_REGION16_LE( 0x10000, "dcs", 0 )    /* ADSP-2115 data Version 1.02 */
	ROM_LOAD16_BYTE( "sound102.u95", 0x000000, 0x8000, CRC(bec7d3ae) SHA1(db80aa4a645804a4574b07b9f34dec6b6b64190d) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )  /* Boot Code Version 1.2 */
	ROM_LOAD( "blitz1_2.u32", 0x000000, 0x80000, CRC(38dbecf5) SHA1(7dd5a5b3baf83a7f8f877ff4cd3f5e8b5201b36f) )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Hard Drive Version 1.21 */
	DISK_IMAGE( "blitz", 0, SHA1(9131c7888e89b3c172780156ed3fe1fe46f78b0a) )
ROM_END


ROM_START( blitz11 )
	ROM_REGION16_LE( 0x10000, "dcs", 0 )    /* ADSP-2115 data Version 1.02 */
	ROM_LOAD16_BYTE( "sound102.u95", 0x000000, 0x8000, CRC(bec7d3ae) SHA1(db80aa4a645804a4574b07b9f34dec6b6b64190d) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )  /* Boot Code Version 1.1 */
	ROM_LOAD( "blitz1_1.u32", 0x000000, 0x80000, CRC(8163ce02) SHA1(89b432d8879052f6c5534ee49599f667f50a010f) )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Hard Drive Version 1.21 */
	DISK_IMAGE( "blitz", 0, SHA1(9131c7888e89b3c172780156ed3fe1fe46f78b0a) )
ROM_END


ROM_START( blitz99 )
	ROM_REGION16_LE( 0x10000, "dcs", 0 )    /* ADSP-2115 data Version 1.02 */
	ROM_LOAD16_BYTE( "sound102.u95", 0x000000, 0x8000, CRC(bec7d3ae) SHA1(db80aa4a645804a4574b07b9f34dec6b6b64190d) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )  /* Boot Code Version 1.0 */
	ROM_LOAD( "bltz9910.u32", 0x000000, 0x80000, CRC(777119b2) SHA1(40d255181c2f3a787919c339e83593fd506779a5) )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Hard Drive Version 1.30 */
	DISK_IMAGE( "blitz99", 0, SHA1(19877e26ffce81dd525031e9e2b4f83ff982e2d9) )
ROM_END

ROM_START( blitz99a )
	ROM_REGION16_LE( 0x10000, "dcs", 0 )    /* ADSP-2115 data Version 1.02 */
	ROM_LOAD16_BYTE( "sound102.u95", 0x000000, 0x8000, CRC(bec7d3ae) SHA1(db80aa4a645804a4574b07b9f34dec6b6b64190d) )

	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )  /* Boot Code Version 1.0 */
	ROM_LOAD( "bltz9910.u32", 0x000000, 0x80000, CRC(777119b2) SHA1(40d255181c2f3a787919c339e83593fd506779a5) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF ) // to use this rom run with -bios up130 and go into TEST mode to update.
	ROM_SYSTEM_BIOS( 0, "noupdate",       "No Update Rom" )
	ROM_SYSTEM_BIOS( 1, "up130",       "Update to 1.30" )
	ROMX_LOAD( "rev.-1.3.u33", 0x000000, 0x100000, CRC(0a0fde5a) SHA1(1edb671c66819f634a9f1daa35331a99b2bda01a), ROM_BIOS(2) )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Hard Drive Version 1.30 */
	DISK_IMAGE( "blitz99a", 0, SHA1(43f834727ce01d7a63b482fc28cbf292477fc6f2) )
ROM_END


ROM_START( blitz2k )
	ROM_REGION16_LE( 0x10000, "dcs", 0 )    /* ADSP-2115 data Version 1.02 */
	ROM_LOAD16_BYTE( "sound102.u95", 0x000000, 0x8000, CRC(bec7d3ae) SHA1(db80aa4a645804a4574b07b9f34dec6b6b64190d) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )  /* Boot Code Version 1.4 */
	ROM_LOAD( "bltz2k14.u32", 0x000000, 0x80000, CRC(ac4f0051) SHA1(b8125c17370db7bfd9b783230b4ef3d5b22a2025) )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Hard Drive Version 1.5 */
	DISK_IMAGE( "blitz2k", 0, SHA1(e89b7fbd4b4a9854d47ae97493e0afffbd1f69e7) )
ROM_END


ROM_START( carnevil )
	ROM_REGION16_LE( 0x10000, "dcs", 0 )    /* ADSP-2115 data Version 1.02 */
	ROM_LOAD16_BYTE( "sound102.u95", 0x000000, 0x8000, CRC(bec7d3ae) SHA1(db80aa4a645804a4574b07b9f34dec6b6b64190d) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 ) /* Boot Rom Version 1.9 */
	ROM_LOAD( "carnevil1_9.u32", 0x000000, 0x80000, CRC(82c07f2e) SHA1(fa51c58022ce251c53bad12fc6ffadb35adb8162) )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Hard Drive v1.0.3  Diagnostics v3.4 / Feb 1 1999 16:00:07 */
	DISK_IMAGE( "carnevil", 0, SHA1(5cffb0de63ad36eb01c5951bab04d3f8a9e23e16) )
ROM_END


ROM_START( carnevil1 )
	ROM_REGION16_LE( 0x10000, "dcs", 0 )    /* ADSP-2115 data Version 1.02 */
	ROM_LOAD16_BYTE( "sound102.u95", 0x000000, 0x8000, CRC(bec7d3ae) SHA1(db80aa4a645804a4574b07b9f34dec6b6b64190d) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )

	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 ) /* Boot Rom Version 1.9 */
	ROM_LOAD( "carnevil1_9.u32", 0x000000, 0x80000, CRC(82c07f2e) SHA1(fa51c58022ce251c53bad12fc6ffadb35adb8162) )

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Hard Drive v1.0.1  Diagnostics v3.3 / Oct 20 1998 11:44:41 */
	DISK_IMAGE( "carnevi1", 0, BAD_DUMP SHA1(94532727512280930a100fe473bf3a938fe2d44f) )
ROM_END


ROM_START( hyprdriv )
	ROM_REGION16_LE( 0x10000, "dcs", 0 )    /* ADSP-2115 data Version 1.02 */
	ROM_LOAD16_BYTE( "seattle.snd", 0x000000, 0x8000, BAD_DUMP CRC(bec7d3ae) SHA1(db80aa4a645804a4574b07b9f34dec6b6b64190d) )

	ROM_REGION32_LE( 0x100000, PCI_ID_GALILEO":update", ROMREGION_ERASEFF )
	ROM_SYSTEM_BIOS( 0, "noupdate",       "No Update Rom" )
	ROM_SYSTEM_BIOS( 1, "update",       "Unknown Update" )
	ROMX_LOAD( "hyperdrive1.2.u33", 0x000000, 0x100000, CRC(fcc922fb) SHA1(7bfa4f0614f561ba77ad2dc7d776af2c3e84b7e7), ROM_BIOS(2) )
	/*  it's either an update to 1.40, or an older version, either way we can't use it with the drive we have, it reports the following

	    'Valid Update Rom Detected'
	    'Processing Rom'
	    'Rom is Wrong Revision Level'
	    'Operation Failure'

	*/

	ROM_REGION32_LE( 0x80000, PCI_ID_GALILEO":rom", 0 )
	ROM_LOAD( "hyperdrive1.1.u32", 0x000000, 0x80000, CRC(3120991e) SHA1(8e47888a5a23c9d3c0d0c64497e1cfb4e46c2cd6) )  /* Boot Rom Version 2. */ // doesn't work, maybe for older drive?
	ROM_LOAD( "hyprdrve.u32", 0x000000, 0x80000, CRC(3e18cb80) SHA1(b18cc4253090ee1d65d72a7ec0c426ed08c4f238) )  /* Boot Rom Version 9. */

	DISK_REGION( PCI_ID_IDE":ide:0:hdd:image" )    /* Version 1.40  Oct 23 1998  15:16:00 */
	DISK_IMAGE( "hyprdriv", 0, SHA1(8cfa343797575b32f46cc24150024be48963a03e) )
ROM_END



/*************************************
 *
 *  Driver init
 *
 *************************************/

void seattle_state::init_common(int config)
{
	// Setup nvram
	m_nvram_data.resize(0x20000/4);
	m_nvram->set_base(m_nvram_data.data(), 0x20000);

	/* switch off the configuration */
	m_board_config = config;
	switch (config)
	{
		case PHOENIX_CONFIG:
			/* original Phoenix board only has 4MB of RAM */
			//m_maincpu->space(AS_PROGRAM).unmap_readwrite(0x00400000, 0x007fffff);
			break;

		case SEATTLE_WIDGET_CONFIG:
			/* set up the widget board */
			//m_maincpu->space(AS_PROGRAM).install_readwrite_handler(0x16c00000, 0x16c0001f, read32_delegate(FUNC(seattle_state::widget_r),this), write32_delegate(FUNC(seattle_state::widget_w),this));
			break;

		case FLAGSTAFF_CONFIG:
			/* set up the analog inputs */
			//m_maincpu->space(AS_PROGRAM).install_readwrite_handler(0x14000000, 0x14000003, read32_delegate(FUNC(seattle_state::analog_port_r),this), write32_delegate(FUNC(seattle_state::analog_port_w),this));

			/* set up the ethernet controller */
			//m_maincpu->space(AS_PROGRAM).install_readwrite_handler(0x16c00000, 0x16c0003f, read32_delegate(FUNC(seattle_state::ethernet_r),this), write32_delegate(FUNC(seattle_state::ethernet_w),this));
			break;
	}
}


DRIVER_INIT_MEMBER(seattle_state,wg3dh)
{
	init_common(PHOENIX_CONFIG);

	/* speedups */
	m_maincpu->mips3drc_add_hotspot(0x8004413C, 0x0C0054B4, 250);     /* confirmed */
	m_maincpu->mips3drc_add_hotspot(0x80094930, 0x00A2102B, 250);     /* confirmed */
	m_maincpu->mips3drc_add_hotspot(0x80092984, 0x3C028011, 250);     /* confirmed */
}


DRIVER_INIT_MEMBER(seattle_state,mace)
{
	init_common(SEATTLE_CONFIG);

	/* speedups */
	m_maincpu->mips3drc_add_hotspot(0x800108F8, 0x8C420000, 250);     /* confirmed */
}


DRIVER_INIT_MEMBER(seattle_state,sfrush)
{
	init_common(FLAGSTAFF_CONFIG);

	/* speedups */
	m_maincpu->mips3drc_add_hotspot(0x80059F34, 0x3C028012, 250);     /* confirmed */
	m_maincpu->mips3drc_add_hotspot(0x800A5AF4, 0x8E300010, 250);     /* confirmed */
	m_maincpu->mips3drc_add_hotspot(0x8004C260, 0x3C028012, 250);     /* confirmed */
}


DRIVER_INIT_MEMBER(seattle_state,sfrushrk)
{
	init_common(FLAGSTAFF_CONFIG);

	/* speedups */
	m_maincpu->mips3drc_add_hotspot(0x800343E8, 0x3C028012, 250);     /* confirmed */
	m_maincpu->mips3drc_add_hotspot(0x8008F4F0, 0x3C028012, 250);     /* confirmed */
	m_maincpu->mips3drc_add_hotspot(0x800A365C, 0x8E300014, 250);     /* confirmed */
	m_maincpu->mips3drc_add_hotspot(0x80051DAC, 0x3C028012, 250);     /* confirmed */
}


DRIVER_INIT_MEMBER(seattle_state,calspeed)
{
	init_common(SEATTLE_WIDGET_CONFIG);

	/* speedups */
	m_maincpu->mips3drc_add_hotspot(0x80032534, 0x02221024, 250);     /* confirmed */
	m_maincpu->mips3drc_add_hotspot(0x800B1BE4, 0x8E110014, 250);     /* confirmed */
}


DRIVER_INIT_MEMBER(seattle_state,vaportrx)
{
	init_common(SEATTLE_WIDGET_CONFIG);

	/* speedups */
	m_maincpu->mips3drc_add_hotspot(0x80049F14, 0x3C028020, 250);     /* confirmed */
	m_maincpu->mips3drc_add_hotspot(0x8004859C, 0x3C028020, 250);     /* confirmed */
	m_maincpu->mips3drc_add_hotspot(0x8005922C, 0x8E020014, 250);     /* confirmed */
}


DRIVER_INIT_MEMBER(seattle_state,biofreak)
{
	init_common(SEATTLE_CONFIG);
}


DRIVER_INIT_MEMBER(seattle_state,blitz)
{
	init_common(SEATTLE_CONFIG);

	/* for some reason, the code in the ROM appears buggy; this is a small patch to fix it */
	//m_rombase[0x934/4] += 4;

	/* main CPU speedups */
	m_maincpu->mips3drc_add_hotspot(0x80135510, 0x3C028024, 250);     /* confirmed */
	m_maincpu->mips3drc_add_hotspot(0x800087DC, 0x8E820010, 250);     /* confirmed */
}


DRIVER_INIT_MEMBER(seattle_state,blitz99)
{
	init_common(SEATTLE_CONFIG);

	/* speedups */
	m_maincpu->mips3drc_add_hotspot(0x8014E41C, 0x3C038025, 250);     /* confirmed */
	m_maincpu->mips3drc_add_hotspot(0x80011F10, 0x8E020018, 250);     /* confirmed */
}


DRIVER_INIT_MEMBER(seattle_state,blitz2k)
{
	init_common(SEATTLE_CONFIG);

	/* speedups */
	m_maincpu->mips3drc_add_hotspot(0x8015773C, 0x3C038025, 250);     /* confirmed */
	m_maincpu->mips3drc_add_hotspot(0x80012CA8, 0x8E020018, 250);     /* confirmed */
}


DRIVER_INIT_MEMBER(seattle_state,carnevil)
{
	init_common(SEATTLE_CONFIG);

	/* set up the gun */
	//m_maincpu->space(AS_PROGRAM).install_readwrite_handler(0x16800000, 0x1680001f, read32_delegate(FUNC(seattle_state::carnevil_gun_r),this), write32_delegate(FUNC(seattle_state::carnevil_gun_w),this));

	/* speedups */
	m_maincpu->mips3drc_add_hotspot(0x8015176C, 0x3C03801A, 250);     /* confirmed */
	m_maincpu->mips3drc_add_hotspot(0x80011FBC, 0x8E020018, 250);     /* confirmed */
}


DRIVER_INIT_MEMBER(seattle_state,hyprdriv)
{
	init_common(SEATTLE_WIDGET_CONFIG);

	/* speedups */
	m_maincpu->mips3drc_add_hotspot(0x801643BC, 0x3C03801B, 250);     /* confirmed */
	m_maincpu->mips3drc_add_hotspot(0x80011FB8, 0x8E020018, 250);     /* confirmed */
	//m_maincpu->mips3drc_add_hotspot(0x80136A80, 0x3C02801D, 250);      /* potential */
}



/*************************************
 *
 *  Game drivers
 *
 *************************************/

/* Atari */
GAME(  1996, wg3dh,      0,        wg3dh,     wg3dh,    seattle_state, wg3dh,    ROT0, "Atari Games",  "Wayne Gretzky's 3D Hockey", MACHINE_SUPPORTS_SAVE )
GAME(  1996, mace,       0,        mace,      mace,     seattle_state, mace,     ROT0, "Atari Games",  "Mace: The Dark Age (boot ROM 1.0ce, HDD 1.0b)", MACHINE_SUPPORTS_SAVE )
GAME(  1997, macea,      mace,     mace,      mace,     seattle_state, mace,     ROT0, "Atari Games",  "Mace: The Dark Age (HDD 1.0a)", MACHINE_SUPPORTS_SAVE )
GAMEL( 1996, sfrush,     0,        sfrush,    sfrush,   seattle_state, sfrush,   ROT0, "Atari Games",  "San Francisco Rush (boot rom L 1.0)", MACHINE_SUPPORTS_SAVE, layout_sfrush )
GAMEL( 1996, sfrusha,    sfrush,   sfrush,    sfrush,   seattle_state, sfrush,   ROT0, "Atari Games",  "San Francisco Rush (boot rom L 1.06A)", MACHINE_SUPPORTS_SAVE, layout_sfrush )
GAMEL( 1996, sfrushrk,   0,        sfrushrk,  sfrushrk, seattle_state, sfrushrk, ROT0, "Atari Games",  "San Francisco Rush: The Rock (boot rom L 1.0, GUTS Oct 6 1997 / MAIN Oct 16 1997)", MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE, layout_sfrush )
GAMEL( 1996, sfrushrkw,  sfrushrk, sfrushrkw, sfrush,   seattle_state, sfrushrk, ROT0, "Atari Games",  "San Francisco Rush: The Rock (Wavenet, boot rom L 1.38, GUTS Aug 19 1997 / MAIN Aug 19 1997)", MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE, layout_sfrush )
GAMEL( 1996, sfrushrkwo, sfrushrk, sfrushrkw, sfrush,   seattle_state, sfrushrk, ROT0, "Atari Games",  "San Francisco Rush: The Rock (Wavenet, boot rom L 1.38, GUTS Aug 6 1997 / MAIN Aug 5 1997)", MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE, layout_sfrush )
GAMEL( 1998, calspeed,   0,        calspeed,  calspeed, seattle_state, calspeed, ROT0, "Atari Games",  "California Speed (Version 2.1a Apr 17 1998, GUTS 1.25 Apr 17 1998 / MAIN Apr 17 1998)", MACHINE_SUPPORTS_SAVE, layout_calspeed )
GAMEL( 1998, calspeeda,  calspeed, calspeed,  calspeed, seattle_state, calspeed, ROT0, "Atari Games",  "California Speed (Version 1.0r8 Mar 10 1998, GUTS Mar 10 1998 / MAIN Mar 10 1998)", MACHINE_SUPPORTS_SAVE, layout_calspeed )
GAMEL( 1998, calspeedb,  calspeed, calspeed,  calspeed, seattle_state, calspeed, ROT0, "Atari Games",  "California Speed (Version 1.0r7a Mar 4 1998, GUTS Mar 3 1998 / MAIN Jan 19 1998)", MACHINE_SUPPORTS_SAVE, layout_calspeed )



GAMEL( 1998, vaportrx,   0,        vaportrx,  vaportrx, seattle_state, vaportrx, ROT0, "Atari Games",  "Vapor TRX", MACHINE_SUPPORTS_SAVE, layout_vaportrx )
GAMEL( 1998, vaportrxp,  vaportrx, vaportrx,  vaportrx, seattle_state, vaportrx, ROT0, "Atari Games",  "Vapor TRX (prototype)", MACHINE_SUPPORTS_SAVE, layout_vaportrx )

/* Midway */
GAME(  1997, biofreak,   0,        biofreak,  biofreak, seattle_state, biofreak, ROT0, "Midway Games", "BioFreaks (prototype)", MACHINE_SUPPORTS_SAVE )
GAME(  1997, blitz,      0,        blitz,     blitz,    seattle_state, blitz,    ROT0, "Midway Games", "NFL Blitz (boot ROM 1.2)", MACHINE_SUPPORTS_SAVE )
GAME(  1997, blitz11,    blitz,    blitz,     blitz,    seattle_state, blitz,    ROT0, "Midway Games", "NFL Blitz (boot ROM 1.1)", MACHINE_SUPPORTS_SAVE )
GAME(  1998, blitz99,    0,        blitz99,   blitz99,  seattle_state, blitz99,  ROT0, "Midway Games", "NFL Blitz '99 (ver 1.30, Sep 22 1998)", MACHINE_SUPPORTS_SAVE )
GAME(  1998, blitz99a,   blitz99,  blitz99,   blitz99,  seattle_state, blitz99,  ROT0, "Midway Games", "NFL Blitz '99 (ver 1.2, Aug 28 1998)", MACHINE_SUPPORTS_SAVE )
GAME(  1999, blitz2k,    0,        blitz2k,   blitz99,  seattle_state, blitz2k,  ROT0, "Midway Games", "NFL Blitz 2000 Gold Edition (ver 1.2, Sep 22 1999)", MACHINE_SUPPORTS_SAVE )
GAME(  1998, carnevil,   0,        carnevil,  carnevil, seattle_state, carnevil, ROT0, "Midway Games", "CarnEvil (v1.0.3)", MACHINE_SUPPORTS_SAVE )
GAME(  1998, carnevil1,  carnevil, carnevil,  carnevil, seattle_state, carnevil, ROT0, "Midway Games", "CarnEvil (v1.0.1)", MACHINE_SUPPORTS_SAVE )
GAMEL( 1998, hyprdriv,   0,        hyprdriv,  hyprdriv, seattle_state, hyprdriv, ROT0, "Midway Games", "Hyperdrive", MACHINE_SUPPORTS_SAVE, layout_hyprdriv )
