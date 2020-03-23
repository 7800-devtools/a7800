// license:BSD-3-Clause
// copyright-holders:Philip Bennett
/***************************************************************************

    Microprose Games machine hardware

****************************************************************************/

#include "emu.h"
#include "includes/micro3d.h"
#include "audio/micro3d.h"

#include "cpu/m68000/m68000.h"
#include "cpu/tms34010/tms34010.h"
#include "cpu/am29000/am29000.h"
#include "cpu/mcs51/mcs51.h"


/*************************************
 *
 *  Defines
 *
 *************************************/

#define MAC_CLK             XTAL_10MHz
#define VTXROM_FMT(x)       (((x) << 14) | ((x) & (1 << 15) ? 0xc0000000 : 0))


/*************************************
 *
 *  68681 DUART
 *
 *************************************/

WRITE_LINE_MEMBER(micro3d_state::duart_irq_handler)
{
	m_maincpu->set_input_line_and_vector(3, state, m_duart68681->get_irq_vector());
}

WRITE_LINE_MEMBER(micro3d_state::duart_txb)
{
	m_m68681_tx0 = state;
	m_audiocpu->set_input_line(MCS51_RX_LINE, ASSERT_LINE);
	// TODO: next line should be behind a timer callback which lasts one audiocpu clock cycle
	m_audiocpu->set_input_line(MCS51_RX_LINE, CLEAR_LINE);
}

READ8_MEMBER(micro3d_state::data_to_i8031)
{
	return m_m68681_tx0;
}

WRITE8_MEMBER(micro3d_state::data_from_i8031)
{
	m_duart68681->rx_b_w(data);
}

/*
 * 0: Monitor port P4
 * 1: 5V
 * 2: /AM29000 present
 * 3: /TMS34010 present
 * 4: -
 * 5: -
 */
READ8_MEMBER(micro3d_state::duart_input_r)
{
	return 0x2;
}

/*
 * 5: /I8051 reset
 * 7: Status LED
*/
WRITE8_MEMBER(micro3d_state::duart_output_w)
{
	m_audiocpu->set_input_line(INPUT_LINE_RESET, data & 0x20 ? CLEAR_LINE : ASSERT_LINE);
}


/*************************************
 *
 *  SCN2651 (TMS34010)
 *
 *************************************/

enum
{
	RX, TX, STATUS, SYN1, SYN2, DLE, MODE1, MODE2, COMMAND
};


WRITE16_MEMBER(micro3d_state::micro3d_ti_uart_w)
{
	switch (offset)
	{
		case 0x0:
		{
			m_ti_uart[TX] = data;
#if VGB_MONITOR_DISPLAY
			mame_debug_printf("%c",data);
#endif
			m_ti_uart[STATUS] |= 1;
			break;
		}
		case 0x1:
		{
			if (m_ti_uart_mode_cycle == 0)
			{
				m_ti_uart[MODE1] = data;
				m_ti_uart_mode_cycle = 1;
			}
			else
			{
				m_ti_uart[MODE2] = data;
				m_ti_uart_mode_cycle = 0;
			}
			break;
		}
		case 0x2:
		{
			if (m_ti_uart_sync_cycle == 0)
			{
				m_ti_uart[SYN1] = data;
				m_ti_uart_mode_cycle = 1;
			}
			else if (m_ti_uart_sync_cycle == 1)
			{
				m_ti_uart[SYN2] = data;
				m_ti_uart_mode_cycle = 2;
			}
			else
			{
				m_ti_uart[DLE] = data;
				m_ti_uart_mode_cycle = 0;
			}
			break;
		}
		case 0x3:
		{
			m_ti_uart[COMMAND] = data;
			m_ti_uart_mode_cycle = 0;
			m_ti_uart_sync_cycle = 0;
			break;
		}
	}
}

READ16_MEMBER(micro3d_state::micro3d_ti_uart_r)
{
	switch (offset)
	{
		case 0x0:
		{
			m_ti_uart[STATUS] ^= 2;
			return m_ti_uart[RX];
		}
		case 0x1:
		{
			if (m_ti_uart_mode_cycle == 0)
			{
				m_ti_uart_mode_cycle = 1;
				return m_ti_uart[MODE1];
			}
			else
			{
				m_ti_uart_mode_cycle = 0;
				return m_ti_uart[MODE2];
			}
		}
		case 0x2:
		{
			return m_ti_uart[STATUS];
		}
		case 0x3:
		{
			m_ti_uart_mode_cycle = m_ti_uart_sync_cycle = 0;
			return m_ti_uart[COMMAND];
		}
		default:
		{
			logerror("Unknown TI UART access.\n");
			return 0;
		}
	}
}


/*************************************
 *
 *  Z8530 SCC (Am29000)
 *
 *************************************/

WRITE32_MEMBER(micro3d_state::micro3d_scc_w)
{
#if DRMATH_MONITOR_DISPLAY
	if (offset == 1)
		osd_printf_debug("%c", data);
#endif
}

READ32_MEMBER(micro3d_state::micro3d_scc_r)
{
	if (offset == 1)
		return 0xd;
	else
		return 5;
}



/*************************************
 *
 *  Math unit
 *
 *************************************/

static inline int64_t dot_product(micro3d_vtx *v1, micro3d_vtx *v2)
{
	int64_t result = ((int64_t)v1->x * (int64_t)v2->x) +
					((int64_t)v1->y * (int64_t)v2->y) +
					((int64_t)v1->z * (int64_t)v2->z);
	return result;
}

static inline int64_t normalised_multiply(int32_t a, int32_t b)
{
	int64_t result;

	result = (int64_t)a * (int64_t)b;
	return result >> 14;
}

void micro3d_state::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	switch (id)
	{
	case TIMER_MAC_DONE:
		mac_done_callback(ptr, param);
		break;
	case TIMER_ADC_DONE:
		adc_done_callback(ptr, param);
		break;
	default:
		assert_always(false, "Unknown id in micro3d_state::device_timer");
	}
}

TIMER_CALLBACK_MEMBER(micro3d_state::mac_done_callback)
{
	m_drmath->set_input_line(AM29000_INTR0, ASSERT_LINE);
	m_mac_stat = 0;
}

WRITE32_MEMBER(micro3d_state::micro3d_mac1_w)
{
	m_vtx_addr = (data & 0x3ffff);
	m_sram_w_addr = (data >> 18) & 0xfff;
}

READ32_MEMBER(micro3d_state::micro3d_mac2_r)
{
	return (m_mac_inst << 1) | m_mac_stat;
}

WRITE32_MEMBER(micro3d_state::micro3d_mac2_w)
{
	uint32_t cnt = data & 0xff;
	uint32_t inst = (data >> 8) & 0x1f;
	uint32_t mac_cycles = 1;

	uint32_t mrab11;
	uint32_t vtx_addr;
	uint32_t sram_r_addr;
	uint32_t sram_w_addr;
	uint32_t *mac_sram;

	m_mac_stat = BIT(data, 13);
	m_mac_inst = inst & 0x7;
	m_mrab11 = (data >> 18) & (1 << 11);
	m_sram_r_addr = (data >> 18) & 0xfff;

	mrab11 = m_mrab11;
	vtx_addr = m_vtx_addr;
	sram_r_addr = m_sram_r_addr;
	sram_w_addr = m_sram_w_addr;
	mac_sram = m_mac_sram;

	if (data & (1 << 14))
		m_drmath->set_input_line(AM29000_INTR0, CLEAR_LINE);

	switch (inst)
	{
		case 0x00: break;
		case 0x04: break;

		case 0x0b: cnt += 0x100;
		case 0x0a: cnt += 0x100;
		case 0x09: cnt += 0x100;
		case 0x08:
		{
			int i;
			const uint16_t *rom = (uint16_t*)m_vertex->base();

			for (i = 0; i <= cnt; ++i)
			{
				int64_t acc;
				micro3d_vtx v1;

				v1.x = VTXROM_FMT(rom[vtx_addr]);   vtx_addr++;
				v1.y = VTXROM_FMT(rom[vtx_addr]);   vtx_addr++;
				v1.z = VTXROM_FMT(rom[vtx_addr]);   vtx_addr++;

				acc  = normalised_multiply(mac_sram[mrab11 + 0x7f0], v1.x);
				acc += normalised_multiply(mac_sram[mrab11 + 0x7f1], v1.y);
				acc += normalised_multiply(mac_sram[mrab11 + 0x7f2], v1.z);
				acc += mac_sram[mrab11 + 0x7f3];
				mac_sram[sram_w_addr++] = acc;

				acc  = normalised_multiply(mac_sram[mrab11 + 0x7f4], v1.x);
				acc += normalised_multiply(mac_sram[mrab11 + 0x7f5], v1.y);
				acc += normalised_multiply(mac_sram[mrab11 + 0x7f6], v1.z);
				acc += mac_sram[mrab11 + 0x7f7];
				mac_sram[sram_w_addr++] = acc;

				acc  = normalised_multiply(mac_sram[mrab11 + 0x7f8], v1.x);
				acc += normalised_multiply(mac_sram[mrab11 + 0x7f9], v1.y);
				acc += normalised_multiply(mac_sram[mrab11 + 0x7fa], v1.z);
				acc += mac_sram[mrab11 + 0x7fb];
				mac_sram[sram_w_addr++] = acc;

				mac_cycles = 16 * cnt;
			}

			break;
		}
		case 0x0e: cnt += 0x100;
		case 0x0d: cnt += 0x100;
		case 0x0c:
		{
			int i;
			const uint16_t *rom = (uint16_t*)m_vertex->base();

			for (i = 0; i <= cnt; ++i)
			{
				int64_t acc;
				micro3d_vtx v1;

				v1.x = VTXROM_FMT(rom[vtx_addr]);   vtx_addr++;
				v1.y = VTXROM_FMT(rom[vtx_addr]);   vtx_addr++;
				v1.z = VTXROM_FMT(rom[vtx_addr]);   vtx_addr++;

				acc  = normalised_multiply(mac_sram[mrab11 + 0x7f0], v1.x);
				acc += normalised_multiply(mac_sram[mrab11 + 0x7f1], v1.y);
				acc += normalised_multiply(mac_sram[mrab11 + 0x7f2], v1.z);
				mac_sram[sram_w_addr++] = acc;

				acc  = normalised_multiply(mac_sram[mrab11 + 0x7f4], v1.x);
				acc += normalised_multiply(mac_sram[mrab11 + 0x7f5], v1.y);
				acc += normalised_multiply(mac_sram[mrab11 + 0x7f6], v1.z);
				mac_sram[sram_w_addr++] = acc;

				acc  = normalised_multiply(mac_sram[mrab11 + 0x7f8], v1.x);
				acc += normalised_multiply(mac_sram[mrab11 + 0x7f9], v1.y);
				acc += normalised_multiply(mac_sram[mrab11 + 0x7fa], v1.z);
				mac_sram[sram_w_addr++] = acc;

				mac_cycles = 12 * cnt;
			}
			break;
		}
		case 0x0f:
		{
			int i;
			const uint16_t *rom = (uint16_t*)m_vertex->base();

			for (i = 0; i <= cnt; ++i, vtx_addr += 4)
			{
				mac_sram[sram_w_addr++] = VTXROM_FMT(rom[vtx_addr + 0]);
				mac_sram[sram_w_addr++] = VTXROM_FMT(rom[vtx_addr + 1]);
				mac_sram[sram_w_addr++] = VTXROM_FMT(rom[vtx_addr + 2]);
				mac_sram[sram_w_addr++] = VTXROM_FMT(rom[vtx_addr + 3]);
			}

			mac_cycles = 8 * cnt;
			break;
		}
		/* Dot product of SRAM vectors with single SRAM vector */
		case 0x11: cnt += 0x100;
		case 0x10:
		{
			int i;
			micro3d_vtx v2;

			v2.x = mac_sram[mrab11 + 0x7fc];
			v2.y = mac_sram[mrab11 + 0x7fd];
			v2.z = mac_sram[mrab11 + 0x7fe];

			for (i = 0; i <= cnt; ++i)
			{
				micro3d_vtx v1;
				int64_t dp;

				v1.x = mac_sram[sram_r_addr++];
				v1.y = mac_sram[sram_r_addr++];
				v1.z = mac_sram[sram_r_addr++];

				dp = dot_product(&v1, &v2);
				mac_sram[sram_w_addr++] = dp >> 32;
				mac_sram[sram_w_addr++] = dp & 0xffffffff;
				mac_sram[sram_w_addr++] = 0;
			}

			mac_cycles = 10 * cnt;
			break;
		}
		/* Dot product of SRAM vectors with SRAM vectors */
		case 0x16: cnt += 0x100;
		case 0x15: cnt += 0x100;
		case 0x14:
		{
			int i;

			for (i = 0; i <= cnt; ++i)
			{
				micro3d_vtx v1;
				micro3d_vtx v2;
				int64_t dp;

				v1.x = mac_sram[sram_r_addr++];
				v1.y = mac_sram[sram_r_addr++];
				v1.z = mac_sram[sram_r_addr++];

				v2.x = mac_sram[vtx_addr++];
				v2.y = mac_sram[vtx_addr++];
				v2.z = mac_sram[vtx_addr++];

				dp = dot_product(&v1, &v2);
				mac_sram[sram_w_addr++] = dp >> 32;
				mac_sram[sram_w_addr++] = dp & 0xffffffff;
				mac_sram[sram_w_addr++] = 0;
			}

			mac_cycles = 10 * cnt;
			break;
		}
		default:
			logerror("Unknown MAC instruction : %x\n", inst);
			break;
	}

	/* TODO: Calculate a better estimate for timing */
	if (m_mac_stat)
		timer_set(attotime::from_hz(MAC_CLK) * mac_cycles, TIMER_MAC_DONE);

	m_mrab11 = mrab11;
	m_vtx_addr = vtx_addr;
	m_sram_r_addr = sram_r_addr;
	m_sram_w_addr = sram_w_addr;
}


/*************************************
 *
 *  Analog controls
 *
 *************************************/

READ16_MEMBER(micro3d_state::micro3d_encoder_h_r)
{
	uint16_t x_encoder = m_joystick_x.read_safe(0);
	uint16_t y_encoder = m_joystick_y.read_safe(0);

	return (y_encoder & 0xf00) | ((x_encoder & 0xf00) >> 8);
}

READ16_MEMBER(micro3d_state::micro3d_encoder_l_r)
{
	uint16_t x_encoder = m_joystick_x.read_safe(0);
	uint16_t y_encoder = m_joystick_y.read_safe(0);

	return ((y_encoder & 0xff) << 8) | (x_encoder & 0xff);
}

TIMER_CALLBACK_MEMBER(micro3d_state::adc_done_callback)
{
	switch (param)
	{
		case 0: m_adc_val = m_throttle.read_safe(0);
				break;
		case 1: m_adc_val = (uint8_t)((255.0/100.0) * m_volume->read() + 0.5);
				break;
		case 2: break;
		case 3: break;
	}

//  mc68901_int_gen(machine(), GPIP3);
}

READ16_MEMBER(micro3d_state::micro3d_adc_r)
{
	return m_adc_val;
}

WRITE16_MEMBER(micro3d_state::micro3d_adc_w)
{
	/* Only handle single-ended mode */
	if (data < 4 || data > 7)
	{
		logerror("ADC0844 unhandled MUX mode: %x\n", data);
		return;
	}

	timer_set(attotime::from_usec(40), TIMER_ADC_DONE, data & ~4);
}

CUSTOM_INPUT_MEMBER(micro3d_state::botss_hwchk_r)
{
	return m_botss_latch;
}

READ16_MEMBER(micro3d_state::botss_140000_r)
{
	m_botss_latch = 0;
	return 0xffff;
}

READ16_MEMBER(micro3d_state::botss_180000_r)
{
	m_botss_latch = 1;
	return 0xffff;
}

/*************************************
 *
 *  CPU control
 *
 *************************************/

WRITE16_MEMBER(micro3d_state::micro3d_reset_w)
{
	data >>= 8;
	m_drmath->set_input_line(INPUT_LINE_RESET, data & 1 ? CLEAR_LINE : ASSERT_LINE);
	m_vgb->set_input_line(INPUT_LINE_RESET, data & 2 ? CLEAR_LINE : ASSERT_LINE);
	/* TODO: Joystick reset? */
}

WRITE16_MEMBER(micro3d_state::host_drmath_int_w)
{
	m_drmath->set_input_line(AM29000_INTR2, ASSERT_LINE);
	machine().scheduler().boost_interleave(attotime::zero, attotime::from_usec(10));
}


/*************************************
 *
 *
 *
 *************************************/

WRITE32_MEMBER(micro3d_state::micro3d_shared_w)
{
	m_shared_ram[offset * 2 + 1] = data & 0xffff;
	m_shared_ram[offset * 2 + 0] = data >> 16;
}

READ32_MEMBER(micro3d_state::micro3d_shared_r)
{
	return (m_shared_ram[offset * 2] << 16) | m_shared_ram[offset * 2 + 1];
}

WRITE32_MEMBER(micro3d_state::drmath_int_w)
{
	m_maincpu->set_input_line(5, HOLD_LINE);
}

WRITE32_MEMBER(micro3d_state::drmath_intr2_ack)
{
	m_drmath->set_input_line(AM29000_INTR2, CLEAR_LINE);
}


/***************************************************************************

    8031 port mappings:

    Port 1                          Port 2
    =======                         ======
    0: S/H sel A     (O)            0:
    1: S/H sel B     (O)            1:
    2: S/H sel C     (O)            2: uPD bank select (O)
    3: S/H en        (O)            3: /uPD busy       (I)
    4: DS1267 data   (O)            4: /uPD reset      (O)
    5: DS1267 clock  (O)            5: Watchdog reset  (O)
    6: /DS1267 reset (O)            6:
    7: Test SW       (I)            7:

***************************************************************************/


WRITE8_MEMBER(micro3d_state::micro3d_snd_dac_a)
{
	m_noise_1->dac_w(data);
	m_noise_2->dac_w(data);
}

WRITE8_MEMBER(micro3d_state::micro3d_snd_dac_b)
{
	/* TODO: This controls upd7759 volume */
}

WRITE8_MEMBER(micro3d_state::micro3d_sound_io_w)
{
	m_sound_port_latch[offset] = data;

	switch (offset)
	{
		case 0x01:
		{
			micro3d_sound_device *noise = (data & 4) ? m_noise_2 : m_noise_1;
			noise->noise_sh_w(data);
			break;
		}
		case 0x03:
		{
			m_upd7759->set_bank_base((data & 0x4) ? 0x20000 : 0);
			m_upd7759->reset_w((data & 0x10) ? 0 : 1);
			break;
		}
	}
}

READ8_MEMBER(micro3d_state::micro3d_sound_io_r)
{
	switch (offset)
	{
		case 0x01:  return (m_sound_port_latch[offset] & 0x7f) | m_sound_sw->read();
		case 0x03:  return (m_sound_port_latch[offset] & 0xf7) | (m_upd7759->busy_r() ? 0x08 : 0);
		default:    return 0;
	}
}

WRITE8_MEMBER(micro3d_state::micro3d_upd7759_w)
{
	m_upd7759->port_w(space, 0, data);
	m_upd7759->start_w(0);
	m_upd7759->start_w(1);
}


/*************************************
 *
 *  Driver initialisation
 *
 *************************************/

DRIVER_INIT_MEMBER(micro3d_state,micro3d)
{
	address_space &space = m_drmath->space(AS_DATA);

	/* The Am29000 program seems to rely on RAM from 0x00470000 onwards being
	non-zero on a reset, otherwise the 3D object data doesn't get uploaded! */
	space.write_dword(0x00470000, 0xa5a5a5a5);

	/* TODO? BOTSS crashes when starting the final stage because the 68000
	overwrites memory in use by the Am29000. Slowing down the 68000 slightly
	avoids this */
	m_maincpu->set_clock_scale(0.945f);
}

DRIVER_INIT_MEMBER(micro3d_state,botss)
{
	address_space &space = m_maincpu->space(AS_PROGRAM);

	/* Required to pass the hardware version check */
	space.install_read_handler(0x140000, 0x140001, read16_delegate(FUNC(micro3d_state::botss_140000_r),this));
	space.install_read_handler(0x180000, 0x180001, read16_delegate(FUNC(micro3d_state::botss_180000_r),this));

	DRIVER_INIT_CALL(micro3d);
}

void micro3d_state::machine_reset()
{
	m_ti_uart[STATUS] = 1;

	m_vgb->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
	m_drmath->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
	m_audiocpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
}
