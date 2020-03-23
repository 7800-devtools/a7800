// license:BSD-3-Clause
// copyright-holders:Vas Crabb
/*
 * Simple 68705 programmer
 * The 68705-based parts have an internal bootstrap ROM that can be used
 * to copy data to the internal EPROM.  The circuit just consists of a
 * 12-bit counter, a ROM, and some discrete glue attached to port B.
 * Data is read on port A.
 *
 * Put the external ROM in cart1 and the initial MCU image in cart2
 * (with nothing in cart2 it assumes a blank MCU).
 *
 * To do a dry run and verify, just toggle off reset (button 1).  The
 * bootstrap will run through the programming procedure but EPROM write
 * will be disabled.  It will then proceed to compare the EPROM contents
 * to the external ROM contents.
 *
 * To program the EPROM, toggle Vpp enable (button 2) and then toggle
 * off reset.  The bootstrap will write the contents of the external ROM
 * to the internal EPROM and then verify the result.
 *
 * When programming is complete, the "Programmed" LED will be lit.  When
 * verification is complete, the "Verified" LED will be lit.  If
 * verification fails, the program stops and the "Address" digits show
 * the address one past the location that failed for P3/P5, or the
 * location that failed for R3/U3.
 */
#include "emu.h"

#include "bus/generic/carts.h"
#include "bus/generic/slot.h"
#include "cpu/m6805/m68705.h"

#include "m68705prg.lh"


namespace {

class m68705prg_state_base : public driver_device
{
public:
	m68705prg_state_base(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_sw(*this, "SW")
		, m_mcu_region(*this, "mcu")
		, m_eprom_image(*this, "eprom_image")
		, m_mcu_image(*this, "mcu_image")
		, m_input_poll_timer(nullptr)
		, m_addr(0x0000)
		, m_pb_val(0xff)
	{
	}

	DECLARE_DEVICE_IMAGE_LOAD_MEMBER(eprom_load)
	{
		auto const desired(m_mcu_region.bytes());
		auto const actual(m_eprom_image->common_get_size("rom"));
		if (desired > actual)
		{
			image.seterror(IMAGE_ERROR_UNSPECIFIED, "Unsupported EPROM size");
			return image_init_result::FAIL;
		}
		else
		{
			m_eprom_image->rom_alloc(desired, GENERIC_ROM8_WIDTH, ENDIANNESS_BIG);
			m_eprom_image->common_load_rom(m_eprom_image->get_rom_base(), desired, "rom");
			return image_init_result::PASS;
		}
	}

	DECLARE_DEVICE_IMAGE_LOAD_MEMBER(mcu_load)
	{
		auto const desired(m_mcu_region.bytes());
		auto const actual(m_mcu_image->common_get_size("rom"));
		if (desired != actual)
		{
			image.seterror(IMAGE_ERROR_UNSPECIFIED, "Incorrect internal MCU EPROM size");
			return image_init_result::FAIL;
		}
		else
		{
			m_mcu_image->common_load_rom(&m_mcu_region[0], actual, "rom");
			return image_init_result::PASS;
		}
	}

	DECLARE_DRIVER_INIT(m68705prg)
	{
		m_input_poll_timer = timer_alloc(TIMER_INPUT_POLL);

		save_item(NAME(m_addr));
		save_item(NAME(m_pb_val));

		m_addr = 0x0000;
		m_pb_val = 0xff;
	}

	DECLARE_MACHINE_START(m68705prg)
	{
		output().set_digit_value(0, s_7seg[(m_addr >> 0) & 0x0f]);
		output().set_digit_value(1, s_7seg[(m_addr >> 4) & 0x0f]);
		output().set_digit_value(2, s_7seg[(m_addr >> 8) & 0x0f]);

		m_input_poll_timer->adjust(attotime::from_hz(120), 0, attotime::from_hz(120));
	}

protected:
	enum
	{
		TIMER_INPUT_POLL
	};

	required_ioport                         m_sw;
	required_region_ptr<u8>                 m_mcu_region;
	required_device<generic_slot_device>    m_eprom_image;
	required_device<generic_slot_device>    m_mcu_image;

	emu_timer *     m_input_poll_timer;

	u16             m_addr;
	u8              m_pb_val;

	static u8 const s_7seg[16];
};

u8 const m68705prg_state_base::s_7seg[16] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71 };


template <typename Device>
class m68705prg_state : public m68705prg_state_base
{
public:
	m68705prg_state(const machine_config &mconfig, device_type type, const char *tag)
		: m68705prg_state_base(mconfig, type, tag)
		, m_mcu(*this, "mcu")
	{
	}

	DECLARE_WRITE8_MEMBER(pb_w)
	{
		// PB4: address counter reset (active high)
		// PB3: address counter clock (falling edge)
		// PB2: Verified LED (active low)
		// PB1: Programmed LED (active low)
		// PB0: apply Vpp (active low)

		if (BIT(data, 4))
			m_addr = 0x0000;
		else if (!BIT(data, 3) && BIT(m_pb_val, 3))
			m_addr = (m_addr + 1) & 0x0fff;
		output().set_led_value(0, !BIT(data, 2));
		output().set_led_value(1, !BIT(data, 1));
		output().set_led_value(3, !BIT(data, 0) && BIT(m_sw->read(), 1));
		m_mcu->set_input_line(M68705_VPP_LINE, (!BIT(data, 0) && BIT(m_sw->read(), 1)) ? ASSERT_LINE : CLEAR_LINE);

		m_pb_val = data;

		u8 const *const ptr(m_eprom_image->get_rom_base());
		m_mcu->pa_w(space, 0, ptr ? ptr[m_addr & m_mcu_region.mask()] : 0xff);

		output().set_digit_value(0, s_7seg[(m_addr >> 0) & 0x0f]);
		output().set_digit_value(1, s_7seg[(m_addr >> 4) & 0x0f]);
		output().set_digit_value(2, s_7seg[(m_addr >> 8) & 0x0f]);
	}

	DECLARE_MACHINE_RESET(m68705prg)
	{
		m_sw->field(0x01)->live().value = 0;
		m_sw->field(0x02)->live().value = 0;

		output().set_led_value(2, 1);
		output().set_led_value(3, 0);

		m_mcu->set_input_line(M68705_IRQ_LINE, ASSERT_LINE);
		m_mcu->set_input_line(M68705_VPP_LINE, CLEAR_LINE);
		m_mcu->set_input_line(M68705_VIHTP_LINE, ASSERT_LINE);
		m_mcu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
	}

protected:
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override
	{
		switch (id)
		{
		case TIMER_INPUT_POLL:
			input_poll_callback(ptr, param);
			break;
		default:
			driver_device::device_timer(timer, id, param, ptr);
		}
	}

	TIMER_CALLBACK_MEMBER(input_poll_callback)
	{
		ioport_value const switches(m_sw->read());
		bool const reset(!BIT(switches, 0));
		bool const vpp(BIT(switches, 1) && !BIT(m_pb_val, 0));

		output().set_led_value(2, reset);
		output().set_led_value(3, vpp);

		m_mcu->set_input_line(M68705_VPP_LINE, vpp ? ASSERT_LINE : CLEAR_LINE);
		m_mcu->set_input_line(INPUT_LINE_RESET, reset ? ASSERT_LINE : CLEAR_LINE);
	}

	required_device<Device> m_mcu;
};

typedef m68705prg_state<m68705p3_device> p3prg_state;
typedef m68705prg_state<m68705p5_device> p5prg_state;
typedef m68705prg_state<m68705r3_device> r3prg_state;
typedef m68705prg_state<m68705u3_device> u3prg_state;


INPUT_PORTS_START(m68705prg)
	PORT_START("SW")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_BUTTON1 ) PORT_TOGGLE PORT_NAME("Reset")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_BUTTON2 ) PORT_TOGGLE PORT_NAME("Vpp")
INPUT_PORTS_END


MACHINE_CONFIG_START(m68705prg)
	MCFG_QUANTUM_PERFECT_CPU("mcu")

	MCFG_GENERIC_SOCKET_ADD("eprom_image", generic_plain_slot, "eprom")
	MCFG_GENERIC_EXTENSIONS("bin,rom")
	MCFG_GENERIC_LOAD(m68705prg_state_base, eprom_load)

	MCFG_GENERIC_SOCKET_ADD("mcu_image", generic_plain_slot, "mcu")
	MCFG_GENERIC_EXTENSIONS("bin,rom")
	MCFG_GENERIC_LOAD(m68705prg_state_base, mcu_load)

	MCFG_MACHINE_START_OVERRIDE(m68705prg_state_base, m68705prg)

	MCFG_DEFAULT_LAYOUT(layout_m68705prg)
MACHINE_CONFIG_END

MACHINE_CONFIG_DERIVED(m68705p3prg, m68705prg)
	MCFG_CPU_ADD("mcu", M68705P3, XTAL_1MHz)
	MCFG_M68705_PORTB_W_CB(WRITE8(p3prg_state, pb_w))

	MCFG_MACHINE_RESET_OVERRIDE(p3prg_state, m68705prg)
MACHINE_CONFIG_END

MACHINE_CONFIG_DERIVED(m68705p5prg, m68705prg)
	MCFG_CPU_ADD("mcu", M68705P5, XTAL_1MHz)
	MCFG_M68705_PORTB_W_CB(WRITE8(p5prg_state, pb_w))

	MCFG_MACHINE_RESET_OVERRIDE(p5prg_state, m68705prg)
MACHINE_CONFIG_END

MACHINE_CONFIG_DERIVED(m68705r3prg, m68705prg)
	MCFG_CPU_ADD("mcu", M68705R3, XTAL_1MHz)
	MCFG_M68705_PORTB_W_CB(WRITE8(r3prg_state, pb_w))

	MCFG_MACHINE_RESET_OVERRIDE(r3prg_state, m68705prg)
MACHINE_CONFIG_END

MACHINE_CONFIG_DERIVED(m68705u3prg, m68705prg)
	MCFG_CPU_ADD("mcu", M68705U3, XTAL_1MHz)
	MCFG_M68705_PORTB_W_CB(WRITE8(u3prg_state, pb_w))

	MCFG_MACHINE_RESET_OVERRIDE(u3prg_state, m68705prg)
MACHINE_CONFIG_END


ROM_START( 705p3prg )
	ROM_REGION( 0x0800, "mcu", 0 )
	ROM_FILL(0x0000, 0x0800, 0x00)
ROM_END

ROM_START( 705p5prg )
	ROM_REGION( 0x0800, "mcu", 0 )
	ROM_FILL(0x0000, 0x0800, 0x00)
ROM_END

ROM_START( 705r3prg )
	ROM_REGION( 0x1000, "mcu", 0 )
	ROM_FILL(0x0000, 0x1000, 0x00)
ROM_END

ROM_START( 705u3prg )
	ROM_REGION( 0x1000, "mcu", 0 )
	ROM_FILL(0x0000, 0x1000, 0x00)
ROM_END

} // anonymous namespace


//    YEAR  NAME      PARENT    COMPAT  MACHINE      INPUT      STATE        INIT       COMPANY     FULLNAME                FLAGS
COMP( 1984, 705p5prg, 0,        0,      m68705p5prg, m68705prg, p5prg_state, m68705prg, "Motorola", "MC68705P5 Programmer", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )
COMP( 1984, 705p3prg, 705p5prg, 0,      m68705p3prg, m68705prg, p3prg_state, m68705prg, "Motorola", "MC68705P3 Programmer", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )
COMP( 1984, 705r3prg, 705p5prg, 0,      m68705r3prg, m68705prg, r3prg_state, m68705prg, "Motorola", "MC68705R3 Programmer", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )
COMP( 1984, 705u3prg, 705p5prg, 0,      m68705u3prg, m68705prg, u3prg_state, m68705prg, "Motorola", "MC68705U3 Programmer", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )
