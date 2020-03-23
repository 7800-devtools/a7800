// license:BSD-3-Clause
// copyright-holders:Vas Crabb
/*
 Intel INTELLEC® 4

 The MOD 4 system has a 4004/4008/4009 chipset.  It has front panel
 switches for holding or pulsing the CPU's TEST line, and a CPU LED
 driven by clock phase 2.

 The MOD 40 system has a 4040/4289 chipset.  It replaces the TEST
 switches with STOP and SINGLE STEP switches, and replaces the CPU LED
 with a RUN LED that's lit when the CPU isn't acknowledging a STOP or
 HALT condition.

 The MOD 4 and MOD 40 systems use the same monitor PROM.  The monitor
 program doesn't use any 4040-specific features, and the console RAM
 readback feature avoids the need for the RPM instruction by latching
 the data and reading it through the ROM port space.

 Strictly speaking, the INTELLEC® 4 and INTELLEC® 4/MOD 4 are different
 machines.  The former uses an earlier control board without support
 for MOD 40 features; the latter uses the same control board as the
 MOD 40 jumpered to provide TEST line control rather than 4040 STOP
 and SINGLE STEP features.

 The MOD 40 repurposes the pins originally used for ROM 1 input port
 lines to expose stop/interrupt request/acknowledge.  It's pretty
 obvious that the MOD 40 was designed as a minimal modification.  A
 set of X1 display LEDs would have been fantastic, as it would show the
 contents of the accumulator with the 4040 CPU, while the X3 LEDs really
 aren't very useful.  However, since the 4004 just echoes M2 during X1,
 the control board has no provision for latching data during this cycle
 and would have required significant layout changes to cater for this.

 Set the terminal for 110 1/8/N/2 to talk to the monitor.
 It only accepts uppercase letters, digits, comma, and carriage return.
 Paper tape reader run/stop is sent to RTS on the serial port.

 TODO:
 * Universal slot cards
 * Expose general-purpose I/O?
 */
#include "emu.h"

#include "machine/imm6_76.h"

#include "bus/intellec4/intellec4.h"
#include "bus/rs232/rs232.h"
#include "cpu/mcs40/mcs40.h"
#include "machine/bankdev.h"

#include "intlc44.lh"
#include "intlc440.lh"


namespace {

/***********************************************************************
    INTELLEC® 4 base driver
***********************************************************************/

class intellec4_state : public driver_device
{
public:
	void bus_cycle(mcs40_cpu_device_base::phase step, u8 sync, u8 data);

	DECLARE_READ8_MEMBER(pm_read);
	DECLARE_WRITE8_MEMBER(pm_write);

	DECLARE_READ8_MEMBER(rom0_in);
	DECLARE_READ8_MEMBER(rom2_in);
	DECLARE_READ8_MEMBER(rom3_in);
	DECLARE_READ8_MEMBER(rome_in);
	DECLARE_READ8_MEMBER(romf_in);

	DECLARE_WRITE8_MEMBER(rom0_out);
	DECLARE_WRITE8_MEMBER(rom1_out);
	DECLARE_WRITE8_MEMBER(rom2_out);
	DECLARE_WRITE8_MEMBER(rom3_out);
	DECLARE_WRITE8_MEMBER(rome_out);
	DECLARE_WRITE8_MEMBER(romf_out);

	DECLARE_WRITE8_MEMBER(ram0_out);
	DECLARE_WRITE8_MEMBER(ram1_out);

	// universal slot outputs
	DECLARE_WRITE_LINE_MEMBER(bus_reset_4002);
	DECLARE_WRITE_LINE_MEMBER(bus_user_reset);

	// front panel switches
	DECLARE_INPUT_CHANGED_MEMBER(sw_reset);
	DECLARE_INPUT_CHANGED_MEMBER(sw_reset_mode);
	template <unsigned N> DECLARE_INPUT_CHANGED_MEMBER(sw_prg_mode);
	DECLARE_INPUT_CHANGED_MEMBER(sw_run);
	DECLARE_INPUT_CHANGED_MEMBER(sw_next_inst);
	DECLARE_INPUT_CHANGED_MEMBER(sw_decr);
	DECLARE_INPUT_CHANGED_MEMBER(sw_incr);
	DECLARE_INPUT_CHANGED_MEMBER(sw_load);
	DECLARE_INPUT_CHANGED_MEMBER(sw_cma_enable);
	DECLARE_INPUT_CHANGED_MEMBER(sw_cma_write);
	DECLARE_INPUT_CHANGED_MEMBER(sw_prgm_pwr);
	DECLARE_INPUT_CHANGED_MEMBER(sw_do_enable);

protected:
	intellec4_state(machine_config const &mconfig, device_type type, char const *tag)
		: driver_device(mconfig, type, tag)
		, m_cpu(*this, "maincpu")
		, m_bus(*this, "bus")
		, m_prg_ram(*this, "ram")
		, m_sw_mode(*this, "MODE")
		, m_program_banks(*this, "prgbank")
		, m_rom_port_banks(*this, "rpbank")
		, m_prom_programmer(*this, "promprg")
		, m_tty(*this, "tty")
		, m_memory(*this, "memory"), m_status(*this, "status")
		, m_sw_control(*this, "CONTROL")
		, m_sw_addr_data(*this, "ADDRDAT")
		, m_sw_passes(*this, "PASSES")
		, m_sw_prom_prgm(*this, "PROM")
	{
	}

	virtual void driver_start() override;
	virtual void driver_reset() override;

	required_device<mcs40_cpu_device_base>              m_cpu;
	required_device<bus::intellec4::univ_bus_device>    m_bus;
	required_shared_ptr<u8>                             m_prg_ram;
	required_ioport                                     m_sw_mode;

private:
	enum
	{
		BANK_PRG_MON = 0,
		BANK_PRG_PROM,
		BANK_PRG_RAM,
		BANK_PRG_NONE,

		BANK_IO_MON = 0,
		BANK_IO_PROM,
		BANK_IO_NEITHER,

		BIT_SW_RESET = 2,
		BIT_SW_RESET_MODE,
		BIT_SW_MON,
		BIT_SW_RAM,
		BIT_SW_PROM,

		BIT_SW_RUN = 0,
		BIT_SW_NEXT_INST,
		BIT_SW_DECR,
		BIT_SW_INCR,
		BIT_SW_LOAD,
		BIT_SW_CMA_ENABLE,
		BIT_SW_CMA_WRITE,

		BIT_SW_PRGM_PWR = 0,
		BIT_SW_DATA_OUT_ENABLE
	};

	TIMER_CALLBACK_MEMBER(reset_expired);

	// LED update helpers
	void display_address(u16 value, u16 mask);
	void display_instruction(u8 value, u8 mask);
	void display_active_bank(u8 value);
	void display_execution(u8 value, u8 mask);
	void display_pointer(u8 value, u8 mask);

	void trigger_reset();
	void check_4002_reset();
	void reset_panel();

	required_device<address_map_bank_device>    m_program_banks, m_rom_port_banks;
	required_device<intel_imm6_76_device>       m_prom_programmer;
	required_device<rs232_port_device>          m_tty;

	required_shared_ptr<u8>     m_memory, m_status;
	required_ioport             m_sw_control, m_sw_addr_data, m_sw_passes;
	required_ioport             m_sw_prom_prgm;

	emu_timer   *m_reset_timer = nullptr;

	// program memory access
	u8      m_ram_page = 0U, m_ram_data = 0U;
	bool    m_ram_write = false;

	// PROM programmer
	u8      m_prom_addr = 0U, m_prom_data = 0U;

	// control board state
	bool    m_cpu_reset = false;
	bool    m_ff_prg_mode[3] = { false, false, false };

	// current state of signals from bus
	bool    m_bus_reset_4002 = false, m_bus_user_reset = false;

	// front panel state
	u16     m_latched_addr = 0U, m_display_addr = 0U;
	u8      m_display_instr = 0U, m_display_bank = 0U, m_display_exec = 0U, m_display_ptr = 0U;
	u8      m_pass_counter = 0U;
	bool    m_panel_reset = false;
	bool    m_next_inst = false, m_adr_cmp_latch = false, m_search_complete = false;
	bool    m_src = false, m_pointer_valid = false;
	bool    m_cma_enable = false, m_cma_write = false;

	// current state of front panel switches
	bool    m_sw_reset = false, m_sw_reset_mode = false;
	bool    m_sw_prg_mode[3] = { false, false, false };
	bool    m_sw_run = false;
};


/*----------------------------------
  System address spaces
----------------------------------*/

ADDRESS_MAP_START(intellec4_program_banks, mcs40_cpu_device_base::AS_ROM, 8, intellec4_state)
	ADDRESS_MAP_UNMAP_LOW

	// 0x0000...0x0fff MON
	AM_RANGE(0x0000, 0x03ff) AM_ROM AM_REGION("monitor", 0x0000)

	// 0x1000...0x1fff PROM

	// 0x1000...0x1fff RAM
	AM_RANGE(0x2000, 0x2fff) AM_READONLY AM_SHARE("ram")

	// 0x3000...0x3fff unmapped in case someone presses two mode switches at once
ADDRESS_MAP_END

ADDRESS_MAP_START(intellec4_rom_port_banks, mcs40_cpu_device_base::AS_ROM_PORTS, 8, intellec4_state)
	ADDRESS_MAP_UNMAP_HIGH

	// 0x0000...0x07ff MON
	AM_RANGE(0x0000, 0x000f) AM_MIRROR(0x1f00) AM_READWRITE(rom0_in, rom0_out)
	AM_RANGE(0x0010, 0x001f) AM_MIRROR(0x1f00) AM_WRITE(rom1_out)
	AM_RANGE(0x0020, 0x002f) AM_MIRROR(0x1f00) AM_READWRITE(rom2_in, rom2_out)
	AM_RANGE(0x0030, 0x003f) AM_MIRROR(0x1f00) AM_READWRITE(rom3_in, rom3_out)
	AM_RANGE(0x00e0, 0x00ef) AM_MIRROR(0x1f00) AM_READWRITE(rome_in, rome_out)
	AM_RANGE(0x00f0, 0x00ff) AM_MIRROR(0x1f00) AM_READWRITE(romf_in, romf_out)

	// 0x0800...0x0fff PROM

	// 0x1000...0x17ff neither

	// 0x1800...0x1fff unused
ADDRESS_MAP_END


/*---------------------------------
  CPU views of address spaces
---------------------------------*/

ADDRESS_MAP_START(intellec4_rom, mcs40_cpu_device_base::AS_ROM, 8, intellec4_state)
	ADDRESS_MAP_UNMAP_LOW
	AM_RANGE(0x0000, 0x0fff) AM_DEVICE("prgbank", address_map_bank_device, amap8)
ADDRESS_MAP_END

ADDRESS_MAP_START(intellec4_ram_memory, mcs40_cpu_device_base::AS_RAM_MEMORY, 8, intellec4_state)
	ADDRESS_MAP_UNMAP_LOW
	AM_RANGE(0x0000, 0x00ff) AM_RAM AM_SHARE("memory") // 4 * 4002
ADDRESS_MAP_END

ADDRESS_MAP_START(intellec4_rom_ports, mcs40_cpu_device_base::AS_ROM_PORTS, 8, intellec4_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x07ff) AM_DEVICE("rpbank", address_map_bank_device, amap8)
ADDRESS_MAP_END

ADDRESS_MAP_START(intellec4_ram_status, mcs40_cpu_device_base::AS_RAM_STATUS, 8, intellec4_state)
	ADDRESS_MAP_UNMAP_LOW
	AM_RANGE(0x0000, 0x003f) AM_RAM AM_SHARE("status") // 4 * 4002
ADDRESS_MAP_END

ADDRESS_MAP_START(intellec4_ram_ports, mcs40_cpu_device_base::AS_RAM_PORTS, 8, intellec4_state)
	AM_RANGE(0x00, 0x00) AM_WRITE(ram0_out)
	AM_RANGE(0x01, 0x01) AM_WRITE(ram1_out)
ADDRESS_MAP_END

ADDRESS_MAP_START(intellec4_program_memory, mcs40_cpu_device_base::AS_PROGRAM_MEMORY, 8, intellec4_state)
	ADDRESS_MAP_UNMAP_LOW
	AM_RANGE(0x0000, 0x01ff) AM_READWRITE(pm_read, pm_write)
ADDRESS_MAP_END


/*----------------------------------
  Common machine configuration
----------------------------------*/

DEVICE_INPUT_DEFAULTS_START(tty)
	DEVICE_INPUT_DEFAULTS("RS232_TXBAUD",    0x00ff, RS232_BAUD_110)
	DEVICE_INPUT_DEFAULTS("RS232_RXBAUD",    0x00ff, RS232_BAUD_110)
	DEVICE_INPUT_DEFAULTS("RS232_STARTBITS", 0x00ff, RS232_STARTBITS_1)
	DEVICE_INPUT_DEFAULTS("RS232_DATABITS",  0x00ff, RS232_DATABITS_8)
	DEVICE_INPUT_DEFAULTS("RS232_PARITY",    0x00ff, RS232_PARITY_NONE)
	DEVICE_INPUT_DEFAULTS("RS232_STOPBITS",  0x00ff, RS232_STOPBITS_2)
	DEVICE_INPUT_DEFAULTS("TERM_CONF",       0x01c0, 0x0000)
	DEVICE_INPUT_DEFAULTS("FLOW_CONTROL",    0x0001, 0x0000)
DEVICE_INPUT_DEFAULTS_END

MACHINE_CONFIG_START(intellec4)
	MCFG_DEVICE_ADD("prgbank", ADDRESS_MAP_BANK, 0)
	MCFG_DEVICE_PROGRAM_MAP(intellec4_program_banks)
	MCFG_ADDRESS_MAP_BANK_ENDIANNESS(ENDIANNESS_LITTLE)
	MCFG_ADDRESS_MAP_BANK_DATABUS_WIDTH(8)
	MCFG_ADDRESS_MAP_BANK_ADDRBUS_WIDTH(14)
	MCFG_ADDRESS_MAP_BANK_STRIDE(0x1000)

	MCFG_DEVICE_ADD("rpbank", ADDRESS_MAP_BANK, 0)
	MCFG_DEVICE_PROGRAM_MAP(intellec4_rom_port_banks)
	MCFG_ADDRESS_MAP_BANK_ENDIANNESS(ENDIANNESS_LITTLE)
	MCFG_ADDRESS_MAP_BANK_DATABUS_WIDTH(8)
	MCFG_ADDRESS_MAP_BANK_ADDRBUS_WIDTH(14)
	MCFG_ADDRESS_MAP_BANK_STRIDE(0x1000)

	MCFG_DEVICE_ADD("promprg", INTEL_IMM6_76, 0)

	MCFG_RS232_PORT_ADD("tty", default_rs232_devices, "terminal")
	MCFG_DEVICE_CARD_DEVICE_INPUT_DEFAULTS("terminal",   tty)
	MCFG_DEVICE_CARD_DEVICE_INPUT_DEFAULTS("null_modem", tty)

	MCFG_DEVICE_ADD("bus", INTELLEC4_UNIV_BUS, 518000. / 7)
	MCFG_INTELLEC4_UNIV_BUS_ROM_SPACE("prgbank", AS_PROGRAM)
	MCFG_INTELLEC4_UNIV_BUS_ROM_PORTS_SPACE("rpbank", AS_PROGRAM)
	MCFG_INTELLEC4_UNIV_BUS_MEMORY_SPACE("maincpu", mcs40_cpu_device_base::AS_RAM_MEMORY)
	MCFG_INTELLEC4_UNIV_BUS_STATUS_SPACE("maincpu", mcs40_cpu_device_base::AS_RAM_STATUS)
	MCFG_INTELLEC4_UNIV_BUS_RAM_PORTS_SPACE("maincpu", mcs40_cpu_device_base::AS_RAM_PORTS)
	MCFG_INTELLEC4_UNIV_BUS_RESET_4002_CB(WRITELINE(intellec4_state, bus_reset_4002))
	MCFG_INTELLEC4_UNIV_BUS_USER_RESET_CB(WRITELINE(intellec4_state, bus_user_reset))
	MCFG_INTELLEC4_UNIV_SLOT_ADD("bus", "j7",  518000. / 7, intellec4_univ_cards, "imm4_90")
	MCFG_INTELLEC4_UNIV_SLOT_ADD("bus", "j8",  518000. / 7, intellec4_univ_cards, "imm6_26")
	MCFG_INTELLEC4_UNIV_SLOT_ADD("bus", "j9",  518000. / 7, intellec4_univ_cards, nullptr)
	MCFG_INTELLEC4_UNIV_SLOT_ADD("bus", "j10", 518000. / 7, intellec4_univ_cards, nullptr)
	MCFG_INTELLEC4_UNIV_SLOT_ADD("bus", "j11", 518000. / 7, intellec4_univ_cards, nullptr)
	MCFG_INTELLEC4_UNIV_SLOT_ADD("bus", "j12", 518000. / 7, intellec4_univ_cards, nullptr)
	MCFG_INTELLEC4_UNIV_SLOT_ADD("bus", "j13", 518000. / 7, intellec4_univ_cards, nullptr)
	MCFG_INTELLEC4_UNIV_SLOT_ADD("bus", "j14", 518000. / 7, intellec4_univ_cards, nullptr)
	MCFG_INTELLEC4_UNIV_SLOT_ADD("bus", "j15", 518000. / 7, intellec4_univ_cards, nullptr)
	MCFG_INTELLEC4_UNIV_SLOT_ADD("bus", "j16", 518000. / 7, intellec4_univ_cards, nullptr)
	MCFG_INTELLEC4_UNIV_SLOT_ADD("bus", "j17", 518000. / 7, intellec4_univ_cards, nullptr)
	MCFG_INTELLEC4_UNIV_SLOT_ADD("bus", "j18", 518000. / 7, intellec4_univ_cards, nullptr)
	MCFG_INTELLEC4_UNIV_SLOT_ADD("bus", "j19", 518000. / 7, intellec4_univ_cards, nullptr)
MACHINE_CONFIG_END


/*----------------------------------
  Common front panel switches
----------------------------------*/

INPUT_PORTS_START(intellec4)
	PORT_START("MODE")
	PORT_BIT( 0x0004, IP_ACTIVE_LOW,  IPT_KEYPAD )             PORT_NAME("RESET")                                         PORT_CHANGED_MEMBER(DEVICE_SELF, intellec4_state, sw_reset,       nullptr)
	PORT_BIT( 0x0008, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("RESET MODE")                                    PORT_CHANGED_MEMBER(DEVICE_SELF, intellec4_state, sw_reset_mode,  nullptr)
	PORT_BIT( 0x0010, IP_ACTIVE_LOW,  IPT_KEYPAD )             PORT_NAME("MON")              PORT_CODE(KEYCODE_1_PAD)     PORT_CHANGED_MEMBER(DEVICE_SELF, intellec4_state, sw_prg_mode<0>, nullptr)
	PORT_BIT( 0x0020, IP_ACTIVE_LOW,  IPT_KEYPAD )             PORT_NAME("RAM")              PORT_CODE(KEYCODE_2_PAD)     PORT_CHANGED_MEMBER(DEVICE_SELF, intellec4_state, sw_prg_mode<1>, nullptr)
	PORT_BIT( 0x0040, IP_ACTIVE_LOW,  IPT_KEYPAD )             PORT_NAME("PROM")             PORT_CODE(KEYCODE_3_PAD)     PORT_CHANGED_MEMBER(DEVICE_SELF, intellec4_state, sw_prg_mode<2>, nullptr)

	PORT_START("CONTROL")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW,  IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("RUN")                                           PORT_CHANGED_MEMBER(DEVICE_SELF, intellec4_state, sw_run,         nullptr)
	PORT_BIT( 0x0002, IP_ACTIVE_LOW,  IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("NEXT INST")                                     PORT_CHANGED_MEMBER(DEVICE_SELF, intellec4_state, sw_next_inst,   nullptr)
	PORT_BIT( 0x0004, IP_ACTIVE_HIGH, IPT_KEYPAD )             PORT_NAME("DECR")             PORT_CODE(KEYCODE_MINUS_PAD) PORT_CHANGED_MEMBER(DEVICE_SELF, intellec4_state, sw_decr,        nullptr)
	PORT_BIT( 0x0008, IP_ACTIVE_HIGH, IPT_KEYPAD )             PORT_NAME("INCR")             PORT_CODE(KEYCODE_PLUS_PAD)  PORT_CHANGED_MEMBER(DEVICE_SELF, intellec4_state, sw_incr,        nullptr)
	PORT_BIT( 0x0010, IP_ACTIVE_HIGH, IPT_KEYPAD )             PORT_NAME("LOAD")             PORT_CODE(KEYCODE_ENTER_PAD) PORT_CHANGED_MEMBER(DEVICE_SELF, intellec4_state, sw_load,        nullptr)
	PORT_BIT( 0x0020, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("CMA ENABLE")                                    PORT_CHANGED_MEMBER(DEVICE_SELF, intellec4_state, sw_cma_enable,  nullptr)
	PORT_BIT( 0x0040, IP_ACTIVE_HIGH, IPT_KEYPAD )             PORT_NAME("CMA WRITE")        PORT_CODE(KEYCODE_SLASH_PAD) PORT_CHANGED_MEMBER(DEVICE_SELF, intellec4_state, sw_cma_write,   nullptr)

	PORT_START("ADDRDAT")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW,  IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("ADDRESS/DATA 0")
	PORT_BIT( 0x0002, IP_ACTIVE_LOW,  IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("ADDRESS/DATA 1")
	PORT_BIT( 0x0004, IP_ACTIVE_LOW,  IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("ADDRESS/DATA 2")
	PORT_BIT( 0x0008, IP_ACTIVE_LOW,  IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("ADDRESS/DATA 3")
	PORT_BIT( 0x0010, IP_ACTIVE_LOW,  IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("ADDRESS/DATA 4")
	PORT_BIT( 0x0020, IP_ACTIVE_LOW,  IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("ADDRESS/DATA 5")
	PORT_BIT( 0x0040, IP_ACTIVE_LOW,  IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("ADDRESS/DATA 6")
	PORT_BIT( 0x0080, IP_ACTIVE_LOW,  IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("ADDRESS/DATA 7")
	PORT_BIT( 0x0100, IP_ACTIVE_LOW,  IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("ADDRESS/DATA 8")
	PORT_BIT( 0x0200, IP_ACTIVE_LOW,  IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("ADDRESS/DATA 9")
	PORT_BIT( 0x0400, IP_ACTIVE_LOW,  IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("ADDRESS/DATA 10")
	PORT_BIT( 0x0800, IP_ACTIVE_LOW,  IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("ADDRESS/DATA 11")

	PORT_START("PASSES")
	PORT_BIT( 0x0001, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("PASSES 0")
	PORT_BIT( 0x0002, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("PASSES 1")
	PORT_BIT( 0x0004, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("PASSES 2")
	PORT_BIT( 0x0008, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("PASSES 3")

	PORT_START("PROM")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW,  IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("PRGM PROM PWR")                                 PORT_CHANGED_MEMBER(DEVICE_SELF, intellec4_state, sw_prgm_pwr,    nullptr)
	PORT_CONFNAME( 0x0002, 0x0002, "PROM PROGRAMMER DATA OUT ENABLE" )                                                    PORT_CHANGED_MEMBER(DEVICE_SELF, intellec4_state, sw_do_enable,   nullptr)
	PORT_CONFSETTING(      0x0000, DEF_STR(Off) )
	PORT_CONFSETTING(      0x0002, DEF_STR(On)  )
INPUT_PORTS_END


/*----------------------------------
  System bus cycle handler
----------------------------------*/

void intellec4_state::bus_cycle(mcs40_cpu_device_base::phase step, u8 sync, u8 data)
{
	switch (step)
	{
	case mcs40_cpu_device_base::phase::A1:
		if (m_cma_enable)
		{
			display_address(m_latched_addr, 0x0fffU);
			display_instruction(m_prg_ram[m_latched_addr], 0xffU);
		}
		else if (!m_search_complete)
		{
			display_address(u16(data), 0x000fU);
			m_src = false;
		}
		if (m_cma_write)
		{
			m_prg_ram[m_latched_addr] = u8(~m_sw_addr_data->read() & 0x00ffU);
			m_cma_write = false;
		}
		break;
	case mcs40_cpu_device_base::phase::A2:
		if (!m_search_complete && !m_cma_enable)
			display_address(u16(data) << 4, 0x00f0U);
		break;
	case mcs40_cpu_device_base::phase::A3:
		if (!m_search_complete && !m_cma_enable)
		{
			display_address(u16(data) << 8, 0x0f00U);
			display_active_bank(~m_cpu->get_cm_ram());
		}
		break;
	case mcs40_cpu_device_base::phase::M1:
		if (!m_search_complete && !m_cma_enable)
			display_instruction(data << 4, 0xf0U);
		break;
	case mcs40_cpu_device_base::phase::M2:
		if (!m_search_complete && !m_cma_enable)
			display_instruction(data, 0x0fU);
		break;
	case mcs40_cpu_device_base::phase::X1:
		// not connected to anything
		break;
	case mcs40_cpu_device_base::phase::X2:
		if (!m_search_complete && !m_cma_enable)
		{
			display_execution(data << 4, 0xf0U);
			m_src = BIT(~m_cpu->get_cm_rom(), 0);
			if (m_src)
				display_pointer(data << 4, 0xf0U);
		}
		break;
	case mcs40_cpu_device_base::phase::X3:
		if (m_search_complete != m_adr_cmp_latch)
			machine().output().set_value("led_status_search", m_search_complete = m_adr_cmp_latch);
		if (!m_search_complete && !m_cma_enable)
		{
			display_execution(data, 0x0fU);
			if (m_src)
			{
				display_pointer(data, 0x0fU);
				if (!m_panel_reset && !m_pointer_valid)
					machine().output().set_value("led_status_ptr_valid", m_pointer_valid = true);
			}
		}
		if (!m_panel_reset && !m_adr_cmp_latch)
			m_adr_cmp_latch = (m_latched_addr == m_display_addr) && !((m_pass_counter ^ m_sw_passes->read()) & 0x0fU);
		if (!m_search_complete && !m_panel_reset && !m_cma_enable && !m_sw_run && (m_latched_addr == m_display_addr))
			m_pass_counter = (m_pass_counter + 1U) & 0x0fU;
		if (m_adr_cmp_latch && !m_next_inst && !m_search_complete)
			machine().output().set_value("led_status_search", m_search_complete = true);
		if (!m_cpu_reset && !m_cma_enable && !m_sw_run)
			m_panel_reset = false;
		break;
	}
}


/*----------------------------------
  Program memory access handlers
----------------------------------*/

READ8_MEMBER(intellec4_state::pm_read)
{
	if (!machine().side_effect_disabled())
	{
		// always causes data to be latched
		u16 const addr((u16(m_ram_page) << 8) | ((offset >> 1) & 0x00ffU));
		m_ram_data = m_prg_ram[addr];
	}

	// the C outputs of the 4289 are always high for RPM/WPM so it's equivalent to romf_in
	return m_ram_data & 0x0fU;
}

WRITE8_MEMBER(intellec4_state::pm_write)
{
	// always causes data to be latched
	u16 const addr((u16(m_ram_page) << 8) | ((offset >> 1) & 0x00ffU));
	m_ram_data = m_prg_ram[addr];
	if (m_ram_write)
	{
		bool const first(BIT(offset, 0));
		m_prg_ram[addr] = (m_ram_data & (first ? 0x0fU : 0xf0U)) | ((data & 0x0fU) << (first ? 4 : 0));
	}
}


/*----------------------------------
  I/O port handlers
----------------------------------*/

READ8_MEMBER(intellec4_state::rom0_in)
{
	// bit 0 of this port is ANDed with the TTY input
	return m_tty->rxd_r() ? 0x0eU : 0x0fU;
}

READ8_MEMBER(intellec4_state::rom2_in)
{
	// lower nybble of PROM programmer data
	return m_prom_programmer->do_r() & 0x0fU;
}

READ8_MEMBER(intellec4_state::rom3_in)
{
	// upper nybble of PROM programmer data
	return (m_prom_programmer->do_r() >> 4) & 0x0fU;
}

READ8_MEMBER(intellec4_state::rome_in)
{
	// upper nybble of RAM data latch
	return (m_ram_data >> 4) & 0x0fU;
}

READ8_MEMBER(intellec4_state::romf_in)
{
	// lower nybble of RAM data latch
	return m_ram_data & 0x0fU;
}

WRITE8_MEMBER(intellec4_state::rom0_out)
{
	// lower nybble of PROM programmer address
	m_prom_addr = (m_prom_addr & 0xf0U) | (data & 0x0fU);
	m_prom_programmer->a_w(m_prom_addr);
}

WRITE8_MEMBER(intellec4_state::rom1_out)
{
	// upper nybble of PROM programmer address
	m_prom_addr = (m_prom_addr & 0x0fU) | ((data << 4) & 0xf0U);
	m_prom_programmer->a_w(m_prom_addr);
}

WRITE8_MEMBER(intellec4_state::rom2_out)
{
	// lower nybble of PROM programmer data
	m_prom_data = (m_prom_data & 0xf0U) | (data & 0x0fU);
	m_prom_programmer->di_w(m_prom_data);
}

WRITE8_MEMBER(intellec4_state::rom3_out)
{
	// upper nybble of PROM programmer data
	m_prom_data = (m_prom_data & 0x0fU) | ((data << 4) & 0xf0U);
	m_prom_programmer->di_w(m_prom_data);
}

WRITE8_MEMBER(intellec4_state::rome_out)
{
	// bit 0 of this port enables program memory write
	m_ram_write = BIT(data, 0);
}

WRITE8_MEMBER(intellec4_state::romf_out)
{
	// sets the program memory page for read/write operations
	m_ram_page = data & 0x0fU;
}

WRITE8_MEMBER(intellec4_state::ram0_out)
{
	// bit 0 of this port controls the TTY current loop
	m_tty->write_txd(BIT(data, 0));
}

WRITE8_MEMBER(intellec4_state::ram1_out)
{
	// bit 0 of this port controls the paper tape motor (0 = stop, 1 = run)
	m_tty->write_rts(BIT(~data, 0));

	// bits 1 and 2 of this port enable PROM write
	m_prom_programmer->r_w_a(BIT(~data, 1));
	m_prom_programmer->r_w(BIT(~data, 2));
}


/*----------------------------------
  Bus signal handlers
----------------------------------*/

WRITE_LINE_MEMBER(intellec4_state::bus_reset_4002)
{
	m_bus_reset_4002 = 0 == state;
	check_4002_reset();
}

WRITE_LINE_MEMBER(intellec4_state::bus_user_reset)
{
	if (!state)
		trigger_reset();
	m_bus_user_reset = 0 == state;
}


/*----------------------------------
  Front panel switch handlers
----------------------------------*/

INPUT_CHANGED_MEMBER(intellec4_state::sw_reset)
{
	if (!newval && oldval)
		trigger_reset();
	m_sw_reset = !bool(newval);
}

INPUT_CHANGED_MEMBER(intellec4_state::sw_reset_mode)
{
	m_sw_reset_mode = bool(newval);
	if (m_cpu_reset)
	{
		m_bus->reset_4002_in(m_sw_reset_mode ? 0 : 1);
		check_4002_reset();
	}
}

template <unsigned N> INPUT_CHANGED_MEMBER(intellec4_state::sw_prg_mode)
{
	static constexpr char const *const mode_leds[3] = { "led_mode_mon", "led_mode_ram", "led_mode_prom" };
	static constexpr int prg_banks[3] = { BANK_PRG_MON, BANK_PRG_RAM, BANK_PRG_PROM };
	static constexpr int io_banks[3] = { BANK_IO_MON, BANK_IO_NEITHER, BANK_IO_PROM };

	if (oldval && !newval)
	{
		if (((0U == N) || !m_sw_prg_mode[0]) && ((1U == N) || !m_sw_prg_mode[1]) && ((2U == N) || !m_sw_prg_mode[2]))
		{
			if (!m_ff_prg_mode[N])
			{
				m_program_banks->set_bank(prg_banks[N]);
				m_rom_port_banks->set_bank(io_banks[N]);
				machine().output().set_value(mode_leds[N], m_ff_prg_mode[N] = true);
			}
			trigger_reset();
		}
		else
		{
			m_program_banks->set_bank(BANK_PRG_NONE);
			m_rom_port_banks->set_bank(BANK_IO_NEITHER);
		}
		if ((0U != N) && m_ff_prg_mode[0])
			machine().output().set_value(mode_leds[0], m_ff_prg_mode[0] = false);
		if ((1U != N) && m_ff_prg_mode[1])
			machine().output().set_value(mode_leds[1], m_ff_prg_mode[1] = false);
		if ((2U != N) && m_ff_prg_mode[2])
			machine().output().set_value(mode_leds[2], m_ff_prg_mode[2] = false);
	}
	m_sw_prg_mode[N] = !bool(newval);
}

INPUT_CHANGED_MEMBER(intellec4_state::sw_run)
{
	m_sw_run = !bool(newval);
	if (m_sw_run)
		reset_panel();
}

INPUT_CHANGED_MEMBER(intellec4_state::sw_next_inst)
{
	m_next_inst = !bool(newval);
}

INPUT_CHANGED_MEMBER(intellec4_state::sw_decr)
{
	// connected to a pulse generator circuit
	if (newval && !oldval)
	{
		m_latched_addr = (m_latched_addr - 1U) & 0x0fffU;
		reset_panel();
	}
}

INPUT_CHANGED_MEMBER(intellec4_state::sw_incr)
{
	// connected to a pulse generator circuit
	if (newval && !oldval)
	{
		m_latched_addr = (m_latched_addr + 1U) & 0x0fffU;
		reset_panel();
	}
}

INPUT_CHANGED_MEMBER(intellec4_state::sw_load)
{
	// connected to a pulse generator circuit
	if (newval && !oldval)
	{
		m_latched_addr = ~m_sw_addr_data->read() & 0x0fffU;
		reset_panel();
	}
}

INPUT_CHANGED_MEMBER(intellec4_state::sw_cma_enable)
{
	m_cma_enable = bool(newval);
	if (m_cma_enable)
		reset_panel();
}

INPUT_CHANGED_MEMBER(intellec4_state::sw_cma_write)
{
	if (newval && !oldval)
		m_cma_write = m_cma_enable;
}

INPUT_CHANGED_MEMBER(intellec4_state::sw_prgm_pwr)
{
	m_prom_programmer->prgm_prom_pwr(newval);
}

INPUT_CHANGED_MEMBER(intellec4_state::sw_do_enable)
{
	m_prom_programmer->data_out_enable(newval);
}


/*----------------------------------
  driver_device implementation
----------------------------------*/

void intellec4_state::driver_start()
{
	m_reset_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(intellec4_state::reset_expired), this));

	save_item(NAME(m_ram_page));
	save_item(NAME(m_ram_data));
	save_item(NAME(m_ram_write));

	save_item(NAME(m_prom_addr));
	save_item(NAME(m_prom_data));

	save_item(NAME(m_cpu_reset));
	save_item(NAME(m_ff_prg_mode));

	save_item(NAME(m_bus_reset_4002));
	save_item(NAME(m_bus_user_reset));

	save_item(NAME(m_latched_addr));
	save_item(NAME(m_display_addr));
	save_item(NAME(m_display_instr));
	save_item(NAME(m_display_bank));
	save_item(NAME(m_display_exec));
	save_item(NAME(m_display_ptr));
	save_item(NAME(m_pass_counter));
	save_item(NAME(m_panel_reset));
	save_item(NAME(m_next_inst));
	save_item(NAME(m_adr_cmp_latch));
	save_item(NAME(m_search_complete));
	save_item(NAME(m_src));
	save_item(NAME(m_pointer_valid));
	save_item(NAME(m_cma_enable));
	save_item(NAME(m_cma_write));

	save_item(NAME(m_sw_reset));
	save_item(NAME(m_sw_reset_mode));
	save_item(NAME(m_sw_prg_mode));
	save_item(NAME(m_sw_run));

	m_ff_prg_mode[0] = true;
	m_ff_prg_mode[1] = m_ff_prg_mode[2] = false;
};

void intellec4_state::driver_reset()
{
	// set stuff according to initial state of front panel
	ioport_value const sw_mode(m_sw_mode->read()), sw_control(m_sw_control->read());
	m_sw_reset       = BIT(~sw_mode,    BIT_SW_RESET);
	m_sw_reset_mode  = BIT( sw_mode,    BIT_SW_RESET_MODE);
	m_sw_prg_mode[0] = BIT(~sw_mode,    BIT_SW_MON);
	m_sw_prg_mode[1] = BIT(~sw_mode,    BIT_SW_RAM);
	m_sw_prg_mode[2] = BIT(~sw_mode,    BIT_SW_PROM);
	m_sw_run         = BIT(~sw_control, BIT_SW_RUN);
	m_next_inst      = BIT(~sw_control, BIT_SW_NEXT_INST);
	m_cma_enable     = BIT( sw_control, BIT_SW_CMA_ENABLE);
	m_ff_prg_mode[0] = m_ff_prg_mode[0] && !m_sw_prg_mode[1] && !m_sw_prg_mode[2];
	m_ff_prg_mode[1] = m_ff_prg_mode[1] && !m_sw_prg_mode[0] && !m_sw_prg_mode[2];
	m_ff_prg_mode[2] = m_ff_prg_mode[2] && !m_sw_prg_mode[0] && !m_sw_prg_mode[1];
	m_panel_reset = m_panel_reset || m_cma_enable || m_sw_run;
	if (m_panel_reset)
	{
		m_pass_counter = 0U;
		m_adr_cmp_latch = false;
		m_search_complete = m_search_complete && m_next_inst;
		m_pointer_valid = false;
	}

	// ensure we're consistent with the state of the bus
	m_bus_reset_4002 = 0 == m_bus->reset_4002_out();
	m_bus_user_reset = 0 == m_bus->user_reset_out();

	// ensure device inputs are all in the correct state
	m_cpu->set_input_line(INPUT_LINE_RESET, m_cpu_reset ? ASSERT_LINE : CLEAR_LINE);
	m_bus->cpu_reset_in(m_cpu_reset ? 0 : 1);
	m_bus->reset_4002_in((m_cpu_reset && m_sw_reset_mode) ? 0 : 1);

	// set up the PROM programmer
	ioport_value const sw_prom_prgm(m_sw_prom_prgm->read());
	m_prom_programmer->data_out_enable(BIT(sw_prom_prgm, BIT_SW_DATA_OUT_ENABLE));
	m_prom_programmer->data_in_positive(1); // not jumpered in, onboard pullup
	m_prom_programmer->data_out_positive(0); // jumpered to ground
	m_prom_programmer->prgm_prom_pwr(BIT(sw_prom_prgm, BIT_SW_PRGM_PWR));

	// set front panel LEDs
	machine().output().set_value("led_status_ptr_valid", m_pointer_valid);
	machine().output().set_value("led_status_search",    m_search_complete);
	machine().output().set_value("led_mode_mon",         m_ff_prg_mode[0]);
	machine().output().set_value("led_mode_ram",         m_ff_prg_mode[1]);
	machine().output().set_value("led_mode_prom",        m_ff_prg_mode[2]);
}


/*----------------------------------
  Timer handlers
----------------------------------*/

TIMER_CALLBACK_MEMBER(intellec4_state::reset_expired)
{
	if (m_cpu_reset)
	{
		m_cpu_reset = false;
		m_cpu->set_input_line(INPUT_LINE_RESET, CLEAR_LINE);
		m_bus->cpu_reset_in(1);
		if (m_sw_reset_mode)
		{
			m_bus->reset_4002_in(1);
			check_4002_reset();
		}
	}
}


/*----------------------------------
  LED update helpers
----------------------------------*/

void intellec4_state::display_address(u16 value, u16 mask)
{
	u16 const diff((value ^ m_display_addr) & mask);
	m_display_addr ^= diff;
	if (BIT(diff, 0))
		machine().output().set_value("led_address_a1_0", BIT(value, 0));
	if (BIT(diff, 1))
		machine().output().set_value("led_address_a1_1", BIT(value, 1));
	if (BIT(diff, 2))
		machine().output().set_value("led_address_a1_2", BIT(value, 2));
	if (BIT(diff, 3))
		machine().output().set_value("led_address_a1_3", BIT(value, 3));
	if (BIT(diff, 4))
		machine().output().set_value("led_address_a2_0", BIT(value, 4));
	if (BIT(diff, 5))
		machine().output().set_value("led_address_a2_1", BIT(value, 5));
	if (BIT(diff, 6))
		machine().output().set_value("led_address_a2_2", BIT(value, 6));
	if (BIT(diff, 7))
		machine().output().set_value("led_address_a2_3", BIT(value, 7));
	if (BIT(diff, 8))
		machine().output().set_value("led_address_a3_0", BIT(value, 8));
	if (BIT(diff, 9))
		machine().output().set_value("led_address_a3_1", BIT(value, 9));
	if (BIT(diff, 10))
		machine().output().set_value("led_address_a3_2", BIT(value, 10));
	if (BIT(diff, 11))
		machine().output().set_value("led_address_a3_3", BIT(value, 11));
}

void intellec4_state::display_instruction(u8 value, u8 mask)
{
	u16 const diff((value ^ m_display_instr) & mask);
	m_display_instr ^= diff;
	if (BIT(diff, 0))
		machine().output().set_value("led_instruction_m2_0", BIT(value, 0));
	if (BIT(diff, 1))
		machine().output().set_value("led_instruction_m2_1", BIT(value, 1));
	if (BIT(diff, 2))
		machine().output().set_value("led_instruction_m2_2", BIT(value, 2));
	if (BIT(diff, 3))
		machine().output().set_value("led_instruction_m2_3", BIT(value, 3));
	if (BIT(diff, 4))
		machine().output().set_value("led_instruction_m1_0", BIT(value, 4));
	if (BIT(diff, 5))
		machine().output().set_value("led_instruction_m1_1", BIT(value, 5));
	if (BIT(diff, 6))
		machine().output().set_value("led_instruction_m1_2", BIT(value, 6));
	if (BIT(diff, 7))
		machine().output().set_value("led_instruction_m1_3", BIT(value, 7));
}

void intellec4_state::display_active_bank(u8 value)
{
	u8 const diff((value ^ m_display_bank) & 0x0fU);
	m_display_bank ^= diff;
	if (BIT(diff, 0))
		machine().output().set_value("led_active_bank_0", BIT(value, 0));
	if (BIT(diff, 1))
		machine().output().set_value("led_active_bank_1", BIT(value, 1));
	if (BIT(diff, 2))
		machine().output().set_value("led_active_bank_2", BIT(value, 2));
	if (BIT(diff, 3))
		machine().output().set_value("led_active_bank_3", BIT(value, 3));
}

void intellec4_state::display_execution(u8 value, u8 mask)
{
	u16 const diff((value ^ m_display_exec) & mask);
	m_display_exec ^= diff;
	if (BIT(diff, 0))
		machine().output().set_value("led_execution_x3_0", BIT(value, 0));
	if (BIT(diff, 1))
		machine().output().set_value("led_execution_x3_1", BIT(value, 1));
	if (BIT(diff, 2))
		machine().output().set_value("led_execution_x3_2", BIT(value, 2));
	if (BIT(diff, 3))
		machine().output().set_value("led_execution_x3_3", BIT(value, 3));
	if (BIT(diff, 4))
		machine().output().set_value("led_execution_x2_0", BIT(value, 4));
	if (BIT(diff, 5))
		machine().output().set_value("led_execution_x2_1", BIT(value, 5));
	if (BIT(diff, 6))
		machine().output().set_value("led_execution_x2_2", BIT(value, 6));
	if (BIT(diff, 7))
		machine().output().set_value("led_execution_x2_3", BIT(value, 7));
}

void intellec4_state::display_pointer(u8 value, u8 mask)
{
	u16 const diff((value ^ m_display_ptr) & mask);
	m_display_ptr ^= diff;
	if (BIT(diff, 0))
		machine().output().set_value("led_last_ptr_x3_0", BIT(value, 0));
	if (BIT(diff, 1))
		machine().output().set_value("led_last_ptr_x3_1", BIT(value, 1));
	if (BIT(diff, 2))
		machine().output().set_value("led_last_ptr_x3_2", BIT(value, 2));
	if (BIT(diff, 3))
		machine().output().set_value("led_last_ptr_x3_3", BIT(value, 3));
	if (BIT(diff, 4))
		machine().output().set_value("led_last_ptr_x2_0", BIT(value, 4));
	if (BIT(diff, 5))
		machine().output().set_value("led_last_ptr_x2_1", BIT(value, 5));
	if (BIT(diff, 6))
		machine().output().set_value("led_last_ptr_x2_2", BIT(value, 6));
	if (BIT(diff, 7))
		machine().output().set_value("led_last_ptr_x2_3", BIT(value, 7));
}


/*----------------------------------
  Internal helpers
----------------------------------*/

void intellec4_state::trigger_reset()
{
	// (re-)trigger the reset monostable
	if (!m_sw_reset && !m_sw_prg_mode[0] && !m_sw_prg_mode[1] && !m_sw_prg_mode[2] && !m_bus_user_reset)
	{
		// 9602 with Rx = 27kΩ, Cx = 0.1µF
		// K * Rx(kΩ) * Cx(pF) * (1 + (1 / Rx(kΩ))) = 0.34 * 27 * 100000 * (1 + (1 / 27)) = 952000ns
		m_reset_timer->adjust(attotime::from_usec(952));
		if (!m_cpu_reset)
		{
			m_cpu_reset = true;
			m_cpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
			m_bus->cpu_reset_in(0);
			reset_panel();
			if (m_sw_reset_mode)
			{
				m_bus->reset_4002_in(0);
				check_4002_reset();
			}
		}
	}
}

void intellec4_state::check_4002_reset()
{
	// FIXME: this really takes multiple cycles to erase, and prevents writes while held
	if ((m_cpu_reset && m_sw_reset_mode) || m_bus_reset_4002)
	{
		std::fill_n(&m_memory[0], m_memory.bytes(), 0U);
		std::fill_n(&m_status[0], m_memory.bytes(), 0U);
	}
}

void intellec4_state::reset_panel()
{
	if (!m_panel_reset)
	{
		m_pass_counter = 0U;
		m_panel_reset = true;
		m_adr_cmp_latch = false;
		if (m_search_complete && !m_next_inst)
			machine().output().set_value("led_status_search", m_search_complete = false);
		if (m_pointer_valid)
			machine().output().set_value("led_status_ptr_valid", m_pointer_valid = false);
	}
}



/***********************************************************************
    MOD 4 driver
***********************************************************************/

class mod4_state : public intellec4_state
{
public:
	mod4_state(machine_config const &mconfig, device_type type, char const *tag)
		: intellec4_state(mconfig, type, tag)
	{
	}

	// universal slot outputs
	DECLARE_WRITE_LINE_MEMBER(bus_test);

	// front panel switches
	DECLARE_INPUT_CHANGED_MEMBER(sw_hold);
	DECLARE_INPUT_CHANGED_MEMBER(sw_one_shot);

protected:
	virtual void driver_start() override;
	virtual void driver_reset() override;

private:
	enum
	{
		BIT_SW_HOLD = 0,
		BIT_SW_ONE_SHOT
	};

	TIMER_CALLBACK_MEMBER(one_shot_expired);

	emu_timer   *m_one_shot_timer = nullptr;

	// control board state
	bool    m_one_shot = false;

	// current state of signals from bus
	bool    m_bus_test = false;

	// current state of front panel switches
	bool    m_sw_hold = false;
};


/*----------------------------------
  MOD 4-specific configuration
----------------------------------*/

MACHINE_CONFIG_DERIVED(mod4, intellec4)
	MCFG_CPU_ADD("maincpu", I4004, 5185000. / 7)
	MCFG_I4004_ROM_MAP(intellec4_rom)
	MCFG_I4004_RAM_MEMORY_MAP(intellec4_ram_memory)
	MCFG_I4004_ROM_PORTS_MAP(intellec4_rom_ports)
	MCFG_I4004_RAM_STATUS_MAP(intellec4_ram_status)
	MCFG_I4004_RAM_PORTS_MAP(intellec4_ram_ports)
	MCFG_I4004_PROGRAM_MEMORY_MAP(intellec4_program_memory)
	MCFG_I4004_BUS_CYCLE_CB(BUSCYCLE(mod4_state, bus_cycle));
	MCFG_I4004_SYNC_CB(DEVWRITELINE("bus", bus::intellec4::univ_bus_device, sync_in))

	MCFG_DEVICE_MODIFY("bus")
	MCFG_INTELLEC4_UNIV_BUS_TEST_CB(WRITELINE(mod4_state, bus_test))

	MCFG_DEFAULT_LAYOUT(layout_intlc44)
MACHINE_CONFIG_END


/*----------------------------------
  MOD 4-specific switches
----------------------------------*/

INPUT_PORTS_START(mod4)
	PORT_INCLUDE(intellec4)

	PORT_MODIFY("MODE")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW,  IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("HOLD")     PORT_CODE(KEYCODE_LEFT)  PORT_CHANGED_MEMBER(DEVICE_SELF, mod4_state, sw_hold,     nullptr)
	PORT_BIT( 0x0002, IP_ACTIVE_HIGH, IPT_KEYPAD )             PORT_NAME("ONE SHOT") PORT_CODE(KEYCODE_RIGHT) PORT_CHANGED_MEMBER(DEVICE_SELF, mod4_state, sw_one_shot, nullptr)
INPUT_PORTS_END


/*----------------------------------
  Bus signal handlers
----------------------------------*/

WRITE_LINE_MEMBER(mod4_state::bus_test)
{
	if (!m_one_shot && !m_sw_hold)
		m_cpu->set_input_line(I4004_TEST_LINE, state ? CLEAR_LINE : ASSERT_LINE);
	m_bus_test = 0 == state;
}


/*----------------------------------
  Front panel switch handlers
----------------------------------*/

INPUT_CHANGED_MEMBER(mod4_state::sw_hold)
{
	// overridden by the one-shot monostable and the bus test signal
	if (!m_one_shot)
	{
		if (!m_bus_test)
			m_cpu->set_input_line(I4004_TEST_LINE, newval ? CLEAR_LINE : ASSERT_LINE);
		m_bus->test_in(newval ? 1 : 0);
	}
	m_sw_hold = !bool(newval);
}

INPUT_CHANGED_MEMBER(mod4_state::sw_one_shot)
{
	if (newval && !oldval)
	{
		// 9602 with Rx = 20kΩ, Cx = 0.005µF
		// K * Rx(kΩ) * Cx(pF) * (1 + (1 / Rx(kΩ))) = 0.34 * 20 * 5000 * (1 + (1 / 20)) = 35700ns
		m_one_shot_timer->adjust(attotime::from_nsec(35700));
		if (!m_one_shot)
		{
			m_one_shot = true;
			if (!m_sw_hold)
			{
				m_bus->test_in(0);
				if (!m_bus_test)
					m_cpu->set_input_line(I4004_TEST_LINE, ASSERT_LINE);
			}
		}
	}
}


/*----------------------------------
  driver_device implementation
----------------------------------*/

void mod4_state::driver_start()
{
	intellec4_state::driver_start();

	m_one_shot_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(mod4_state::one_shot_expired), this));

	save_item(NAME(m_one_shot));

	save_item(NAME(m_bus_test));

	save_item(NAME(m_sw_hold));

	m_one_shot = false;
}

void mod4_state::driver_reset()
{
	intellec4_state::driver_reset();

	// set stuff according to initial state of front panel
	m_sw_hold = BIT(~m_sw_mode->read(), BIT_SW_HOLD);

	// ensure we're consistent with the state of the bus
	m_bus_test = 0 == m_bus->test_out();

	// ensure device inputs are all in the correct state
	m_cpu->set_input_line(I4004_TEST_LINE, (m_one_shot || m_bus_test || m_sw_hold) ? ASSERT_LINE : CLEAR_LINE);
	m_bus->test_in((m_one_shot || m_sw_hold) ? 0 : 1);

	// set front panel LEDs
	machine().output().set_value("led_status_cpu", true); // actually driven by clock phase 2 - let's assume it's always running
}


/*----------------------------------
  Timer handlers
----------------------------------*/

TIMER_CALLBACK_MEMBER(mod4_state::one_shot_expired)
{
	m_one_shot = false;
	if (!m_sw_hold)
	{
		m_bus->test_in(1);
		if (!m_bus_test)
			m_cpu->set_input_line(I4004_TEST_LINE, CLEAR_LINE);
	}
}



/***********************************************************************
    MOD 40 driver
***********************************************************************/

class mod40_state : public intellec4_state
{
public:
	mod40_state(machine_config const &mconfig, device_type type, char const *tag)
		: intellec4_state(mconfig, type, tag)
	{
	}

	DECLARE_WRITE_LINE_MEMBER(stp_ack);

	// universal slot outputs
	DECLARE_WRITE_LINE_MEMBER(bus_stop);
	DECLARE_WRITE_LINE_MEMBER(bus_test);

	// front panel switches
	DECLARE_INPUT_CHANGED_MEMBER(sw_stop);
	DECLARE_INPUT_CHANGED_MEMBER(sw_single_step);

protected:
	virtual void driver_start() override;
	virtual void driver_reset() override;

private:
	enum
	{
		BIT_SW_STOP = 0,
		BIT_SW_SINGLE_STEP
	};

	TIMER_CALLBACK_MEMBER(single_step_expired);

	emu_timer   *m_single_step_timer = nullptr;

	// control board state
	bool    m_stp_ack = false, m_single_step = false;

	// current state of signals from bus
	bool    m_bus_stop = false;

	// current state of front panel switches
	bool    m_sw_stop = false;
};


/*----------------------------------
  MOD 40-specific configuration
----------------------------------*/

MACHINE_CONFIG_DERIVED(mod40, intellec4)
	MCFG_CPU_ADD("maincpu", I4040, 5185000. / 7)
	MCFG_I4040_ROM_MAP(intellec4_rom)
	MCFG_I4040_RAM_MEMORY_MAP(intellec4_ram_memory)
	MCFG_I4040_ROM_PORTS_MAP(intellec4_rom_ports)
	MCFG_I4040_RAM_STATUS_MAP(intellec4_ram_status)
	MCFG_I4040_RAM_PORTS_MAP(intellec4_ram_ports)
	MCFG_I4040_PROGRAM_MEMORY_MAP(intellec4_program_memory)
	MCFG_I4040_BUS_CYCLE_CB(BUSCYCLE(mod40_state, bus_cycle));
	MCFG_I4040_SYNC_CB(DEVWRITELINE("bus", bus::intellec4::univ_bus_device, sync_in))
	MCFG_I4040_STP_ACK_CB(WRITELINE(mod40_state, stp_ack))

	MCFG_DEVICE_MODIFY("bus")
	MCFG_INTELLEC4_UNIV_BUS_STOP_CB(WRITELINE(mod40_state, bus_stop))
	MCFG_INTELLEC4_UNIV_BUS_TEST_CB(WRITELINE(mod40_state, bus_test))

	MCFG_DEFAULT_LAYOUT(layout_intlc440)
MACHINE_CONFIG_END


/*----------------------------------
  MOD 40-specific switches
----------------------------------*/

INPUT_PORTS_START(mod40)
	PORT_INCLUDE(intellec4)

	PORT_MODIFY("MODE")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW,  IPT_KEYPAD ) PORT_TOGGLE PORT_NAME("STOP")        PORT_CODE(KEYCODE_LEFT)  PORT_CHANGED_MEMBER(DEVICE_SELF, mod40_state, sw_stop,        0)
	PORT_BIT( 0x0002, IP_ACTIVE_HIGH, IPT_KEYPAD )             PORT_NAME("SINGLE STEP") PORT_CODE(KEYCODE_RIGHT) PORT_CHANGED_MEMBER(DEVICE_SELF, mod40_state, sw_single_step, 0)
INPUT_PORTS_END


/*----------------------------------
  CPU output handlers
----------------------------------*/

WRITE_LINE_MEMBER(mod40_state::stp_ack)
{
	// resets the single-step monostable
	if (m_stp_ack && state)
	{
		m_single_step_timer->reset();
		m_single_step = false;
		if (m_sw_stop)
		{
			m_bus->stop_in(0);
			if (!m_bus_stop)
				m_cpu->set_input_line(I4040_STP_LINE, ASSERT_LINE);
		}
	}
	m_stp_ack = 0 == state;
	machine().output().set_value("led_status_run",  !m_stp_ack);
	m_bus->stop_acknowledge_in(state);
}


WRITE_LINE_MEMBER(mod40_state::bus_stop)
{
	// will not allow the CPU to step/run
	if (m_single_step || !m_sw_stop)
		m_cpu->set_input_line(I4040_STP_LINE, state ? CLEAR_LINE : ASSERT_LINE);
	m_bus_stop = 0 == state;
}


/*----------------------------------
  Bus signal handlers
----------------------------------*/

WRITE_LINE_MEMBER(mod40_state::bus_test)
{
	m_cpu->set_input_line(I4040_TEST_LINE, state ? CLEAR_LINE : ASSERT_LINE);
}


/*----------------------------------
  Front panel switch handlers
----------------------------------*/

INPUT_CHANGED_MEMBER(mod40_state::sw_stop)
{
	// overridden by the single-step monostable and the bus stop signal
	if (!m_single_step)
	{
	   if (!m_bus_stop)
			m_cpu->set_input_line(I4040_STP_LINE, newval ? CLEAR_LINE : ASSERT_LINE);
		m_bus->stop_in(newval ? 1 : 0);
	}
	m_sw_stop = !bool(newval);
}

INPUT_CHANGED_MEMBER(mod40_state::sw_single_step)
{
	// (re-)triggers the single-step monostable
	if (m_stp_ack && newval && !oldval)
	{
		// 9602 with Rx = 20kΩ, Cx = 0.005µF
		// K * Rx(kΩ) * Cx(pF) * (1 + (1 / Rx(kΩ))) = 0.34 * 20 * 5000 * (1 + (1 / 20)) = 35700ns
		m_single_step_timer->adjust(attotime::from_nsec(35700));
		if (!m_single_step)
		{
			m_single_step = true;
			if (m_sw_stop)
			{
				m_bus->stop_in(1);
				if (!m_bus_stop)
					m_cpu->set_input_line(I4040_STP_LINE, CLEAR_LINE);
			}
		}
	}
}


/*----------------------------------
  driver_device implementation
----------------------------------*/

void mod40_state::driver_start()
{
	intellec4_state::driver_start();

	m_single_step_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(mod40_state::single_step_expired), this));

	save_item(NAME(m_stp_ack));
	save_item(NAME(m_single_step));

	save_item(NAME(m_bus_stop));

	save_item(NAME(m_sw_stop));

	m_stp_ack = m_single_step = false;
}

void mod40_state::driver_reset()
{
	intellec4_state::driver_reset();

	// set stuff according to initial state of front panel
	m_sw_stop = BIT(~m_sw_mode->read(), BIT_SW_STOP);

	// ensure we're consistent with the state of the bus
	m_bus_stop = 0 == m_bus->stop_out();

	// ensure device inputs are all in the correct state
	m_cpu->set_input_line(I4040_TEST_LINE, m_bus->test_out() ? CLEAR_LINE : ASSERT_LINE);
	m_cpu->set_input_line(I4040_STP_LINE, ((m_sw_stop && !m_single_step) || m_bus_stop) ? ASSERT_LINE : CLEAR_LINE);
	m_bus->test_in(1);
	m_bus->stop_in((m_sw_stop && !m_single_step) ? 0 : 1);

	// set front panel LEDs
	machine().output().set_value("led_status_run", !m_stp_ack);
}


/*----------------------------------
  Timer handlers
----------------------------------*/

TIMER_CALLBACK_MEMBER(mod40_state::single_step_expired)
{
	m_single_step = false;
	if (m_sw_stop)
	{
		m_bus->stop_in(0);
		if (!m_bus_stop)
			m_cpu->set_input_line(I4040_STP_LINE, ASSERT_LINE);
	}
}


/***********************************************************************
    ROM definitions
***********************************************************************/

ROM_START(intlc44)
	ROM_REGION(0x0400, "monitor", 0) // 4 * 1702A
	ROM_DEFAULT_BIOS("v2.1")
	ROM_SYSTEM_BIOS(0, "v2.1", "MON 4 V2.1")
	ROMX_LOAD("mon_4-000-v_2.1.a1",      0x0000, 0x0100, CRC(8d1f56ff) SHA1(96bc19be9be4e92195fad82d7a3cadb763ab6e3f), ROM_BIOS(1))
	ROMX_LOAD("mon_4-100-v_2.1.a2",      0x0100, 0x0100, CRC(66562a4f) SHA1(040749c45e95dfc39b3397d0c31c8b4c11f0a5fc), ROM_BIOS(1))
	ROMX_LOAD("mon_4-200-v_2.1.a3",      0x0200, 0x0100, CRC(fe039c68) SHA1(1801cfcc7514412865c0fdc7d1800fcf583a2d2a), ROM_BIOS(1))
	ROMX_LOAD("mon_4-300-v_2.1.a4",      0x0300, 0x0100, CRC(3724d5af) SHA1(b764b3bb3541fbda875f7a7655f46aa54b332631), ROM_BIOS(1))
ROM_END

ROM_START(intlc440)
	ROM_REGION(0x0400, "monitor", 0) // 4 * 1702A
	ROM_DEFAULT_BIOS("v2.1")
	ROM_SYSTEM_BIOS(0, "v2.1", "MON 4 V2.1")
	ROMX_LOAD("mon_4-000-v_2.1.a1",      0x0000, 0x0100, CRC(8d1f56ff) SHA1(96bc19be9be4e92195fad82d7a3cadb763ab6e3f), ROM_BIOS(1))
	ROMX_LOAD("mon_4-100-v_2.1.a2",      0x0100, 0x0100, CRC(66562a4f) SHA1(040749c45e95dfc39b3397d0c31c8b4c11f0a5fc), ROM_BIOS(1))
	ROMX_LOAD("mon_4-200-v_2.1.a3",      0x0200, 0x0100, CRC(fe039c68) SHA1(1801cfcc7514412865c0fdc7d1800fcf583a2d2a), ROM_BIOS(1))
	ROMX_LOAD("mon_4-300-v_2.1.a4",      0x0300, 0x0100, CRC(3724d5af) SHA1(b764b3bb3541fbda875f7a7655f46aa54b332631), ROM_BIOS(1))
	ROM_SYSTEM_BIOS(1, "v2.1_1200", "MON 4 V2.1 1200 Baud hack")
	ROMX_LOAD("mon_4-000-v_2.1.a1",      0x0000, 0x0100, CRC(8d1f56ff) SHA1(96bc19be9be4e92195fad82d7a3cadb763ab6e3f), ROM_BIOS(2))
	ROMX_LOAD("i40_mon-1.a2",            0x0100, 0x0100, CRC(cd9fecd6) SHA1(9c4fb85118c881687fd4b324e5089df05d1e63d1), ROM_BIOS(2))
	ROMX_LOAD("i40_mon-2.a3",            0x0200, 0x0100, CRC(037de128) SHA1(3694636e1f4e23688b36ea9ee755a0c5888f4328), ROM_BIOS(2))
	ROMX_LOAD("1200_baud-i40_mon-f3.a4", 0x0300, 0x0100, CRC(f3198d79) SHA1(b7903073b69f487b6f78842c08694f12225d85f0), ROM_BIOS(2))
ROM_END

} // anonymous namespace


/***********************************************************************
    Machine definitions
***********************************************************************/

//    YEAR   NAME      PARENT  COMPAT  MACHINE   INPUT  STATE        INIT  COMPANY  FULLNAME             FLAGS
COMP( 1973?, intlc44,  0,      0,      mod4,     mod4,  mod4_state,  0,    "Intel", "INTELLEC 4/MOD 4",  MACHINE_NO_SOUND_HW | MACHINE_CLICKABLE_ARTWORK | MACHINE_SUPPORTS_SAVE )
COMP( 1974?, intlc440, 0,      0,      mod40,    mod40, mod40_state, 0,    "Intel", "INTELLEC 4/MOD 40", MACHINE_NO_SOUND_HW | MACHINE_CLICKABLE_ARTWORK | MACHINE_SUPPORTS_SAVE )
