// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/***************************************************************************

    i8x9x.h

    MCS96, 8x9x branch, the original version

***************************************************************************/

#include "emu.h"
#include "i8x9x.h"

i8x9x_device::i8x9x_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	mcs96_device(mconfig, type, tag, owner, clock, 8),
	io_config("io", ENDIANNESS_LITTLE, 16, 16, -1), io(nullptr), base_timer2(0), ad_done(0), hso_command(0), ad_command(0), hso_time(0), ad_result(0),
	ios0(0), ios1(0), ioc0(0), ioc1(0), sbuf(0), sp_stat(0), serial_send_buf(0), serial_send_timer(0)
{
}

offs_t i8x9x_device::disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options)
{
	return disasm_generic(stream, pc, oprom, opram, options, disasm_entries);
}

device_memory_interface::space_config_vector i8x9x_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM, &program_config),
		std::make_pair(AS_IO,      &io_config)
	};
}

void i8x9x_device::device_start()
{
	mcs96_device::device_start();
	io = &space(AS_IO);
	cycles_scaling = 3;
}

void i8x9x_device::device_reset()
{
	mcs96_device::device_reset();
	memset(hso_info, 0, sizeof(hso_info));
	memset(&hso_cam_hold, 0, sizeof(hso_cam_hold));
	hso_command = 0;
	hso_time = 0;
	base_timer2 = 0;
	ios0 = ios1 = ioc0 = ioc1 = 0x00;
	ad_result = 0;
	ad_done = 0;
	sp_stat = 0;
	serial_send_timer = 0;
}

void i8x9x_device::commit_hso_cam()
{
	for(int i=0; i<8; i++)
		if(!hso_info[i].active) {
			if(hso_command != 0x18 && hso_command != 0x19)
				logerror("%s: hso cam %02x %04x in slot %d (%04x)\n", tag(), hso_command, hso_time, i, PPC);
			hso_info[i].active = true;
			hso_info[i].command = hso_command;
			hso_info[i].time = hso_time;
			internal_update(total_cycles());
			return;
		}
	hso_cam_hold.active = true;
	hso_cam_hold.command = hso_command;
	hso_cam_hold.time = hso_time;
}

void i8x9x_device::ad_start(uint64_t current_time)
{
	ad_result = (io->read_word(2*((ad_command & 7) + A0)) << 6) | 8 | (ad_command & 7);
	ad_done = current_time + 88;
	internal_update(current_time);
}

void i8x9x_device::serial_send(uint8_t data)
{
	serial_send_buf = data;
	serial_send_timer = total_cycles() + 9600;
}

void i8x9x_device::serial_send_done()
{
	serial_send_timer = 0;
	io->write_word(SERIAL*2, serial_send_buf);
	pending_irq |= IRQ_SERIAL;
	sp_stat |= 0x20;
	check_irq();
}

void i8x9x_device::io_w8(uint8_t adr, uint8_t data)
{
	switch(adr) {
	case 0x02:
		ad_command = data;
		if(ad_command & 8)
			ad_start(total_cycles());
		break;
	case 0x03:
		logerror("%s: hsi_mode %02x (%04x)\n", tag(), data, PPC);
		break;
	case 0x04:
		hso_time = (hso_time & 0xff00) | data;
		break;
	case 0x05:
		hso_time = (hso_time & 0x00ff) | (data << 8);
		commit_hso_cam();
		break;
	case 0x06:
		hso_command = data;
		break;
	case 0x07:
		logerror("%s: sbuf %02x (%04x)\n", tag(), data, PPC);
		serial_send(data);
		break;
	case 0x08:
		PSW = (PSW & 0xff00) | data;
		check_irq();
		break;
	case 0x09:
		pending_irq = data;
		logerror("%s: int_pending %02x (%04x)\n", tag(), data, PPC);
		break;
	case 0x0a:
		logerror("%s: watchdog %02x (%04x)\n", tag(), data, PPC);
		break;
	case 0x0e:
		logerror("%s: baud rate %02x (%04x)\n", tag(), data, PPC);
		break;
	case 0x0f:
		logerror("%s: io port 1 %02x (%04x)\n", tag(), data, PPC);
		io->write_word(P1*2, data);
		break;
	case 0x10:
		logerror("%s: io port 2 %02x (%04x)\n", tag(), data, PPC);
		io->write_word(P2*2, data);
		break;
	case 0x11:
		logerror("%s: sp con %02x (%04x)\n", tag(), data, PPC);
		break;
	case 0x15:
		logerror("%s: ioc0 %02x (%04x)\n", tag(), data, PPC);
		ioc0 = data;
		break;
	case 0x16:
		logerror("%s: ioc1 %02x (%04x)\n", tag(), data, PPC);
		ioc1 = data;
		break;
	case 0x17:
		logerror("%s: pwm control %02x (%04x)\n", tag(), data, PPC);
		break;
	}
	return;
}

void i8x9x_device::io_w16(uint8_t adr, uint16_t data)
{
	switch(adr) {
	case 0:
		break;
	case 4:
		hso_time = data;
		commit_hso_cam();
		break;
	default:
		io_w8(adr, data);
		io_w8(adr+1, data>>8);
		break;
	}
	return;
}

uint8_t i8x9x_device::io_r8(uint8_t adr)
{
	switch(adr) {
	case 0x00:
		return 0x00;
	case 0x01:
		return 0x00;
	case 0x02:
		return ad_result;
	case 0x03:
		return ad_result >> 8;
	case 0x04:
		logerror("%s: read hsi time l (%04x)\n", tag(), PPC);
		return 0x00;
	case 0x05:
		logerror("%s: read hsi time h (%04x)\n", tag(), PPC);
		return 0x00;
	case 0x06:
		logerror("%s: read hsi status (%04x)\n", tag(), PPC);
		return 0x00;
	case 0x07:
		logerror("%s: read sbuf %02x (%04x)\n", tag(), sbuf, PPC);
		return sbuf;
	case 0x08:
		return PSW;
	case 0x09:
		logerror("%s: read int pending (%04x)\n", tag(), PPC);
		return pending_irq;
	case 0x0a:
		logerror("%s: read timer1 l (%04x)\n", tag(), PPC);
		return timer_value(1, total_cycles());
	case 0x0b:
		logerror("%s: read timer1 h (%04x)\n", tag(), PPC);
		return timer_value(1, total_cycles()) >> 8;
	case 0x0c:
		logerror("%s: read timer2 l (%04x)\n", tag(), PPC);
		return timer_value(2, total_cycles());
	case 0x0d:
		logerror("%s: read timer2 h (%04x)\n", tag(), PPC);
		return timer_value(2, total_cycles()) >> 8;
	case 0x0e: {
		static int last = -1;
		if(io->read_word(P0*2) != last) {
			last = io->read_word(P0*2);
			logerror("%s: read p0 %02x\n", tag(), io->read_word(P0*2));
		}
		return io->read_word(P0*2);
	}
	case 0x0f:
		return io->read_word(P1*2);
	case 0x10:
		return io->read_word(P2*2);
	case 0x11: {
		uint8_t res = sp_stat;
		sp_stat &= 0x80;
		logerror("%s: read sp stat %02x (%04x)\n", tag(), res, PPC);
		return res;
	}
	case 0x15:
		logerror("%s: read ios 0 %02x (%04x)\n", tag(), ios0, PPC);
		return ios0;
	case 0x16: {
		uint8_t res = ios1;
		ios1 = ios1 & 0xc0;
		return res;
	}
	default:
		logerror("%s: io_r8 %02x (%04x)\n", tag(), adr, PPC);
		return 0x00;
	}
}

uint16_t i8x9x_device::io_r16(uint8_t adr)
{
	switch(adr) {
	case 0x00:
		return 0x0000;
	case 0x02:
		return ad_result;
	case 0x04:
		logerror("%s: read hsi time (%04x)\n", tag(), PPC);
		return 0x0000;
	case 0x0a:
		return timer_value(1, total_cycles());
	case 0x0c:
		logerror("%s: read timer2 (%04x)\n", tag(), PPC);
		return timer_value(2, total_cycles());
	default:
		return io_r8(adr) | (io_r8(adr+1) << 8);
	}
}

void i8x9x_device::do_exec_partial()
{
}

void i8x9x_device::serial_w(uint8_t val)
{
	sbuf = val;
	sp_stat |= 0x40;
	pending_irq |= IRQ_SERIAL;
	check_irq();
}

uint16_t i8x9x_device::timer_value(int timer, uint64_t current_time) const
{
	if(timer == 2)
		current_time -= base_timer2;
	return current_time >> 3;
}

uint64_t i8x9x_device::timer_time_until(int timer, uint64_t current_time, uint16_t timer_value) const
{
	uint64_t timer_base = timer == 2 ? base_timer2 : 0;
	uint64_t delta = (current_time - timer_base) >> 3;
	uint32_t tdelta = uint16_t(timer_value - delta);
	if(!tdelta)
		tdelta = 0x10000;
	return timer_base + ((delta + tdelta) << 3);
}

void i8x9x_device::trigger_cam(int id, uint64_t current_time)
{
	hso_cam_entry &cam = hso_info[id];
	cam.active = false;
	switch(cam.command & 0x0f) {
	case 0x8: case 0x9: case 0xa: case 0xb:
		ios1 |= 1 << (cam.command & 3);
		pending_irq |= IRQ_SOFT;
		check_irq();
		break;

	default:
		logerror("%s: Action %x unimplemented\n", tag(), cam.command & 0x0f);
		break;
	}
}

void i8x9x_device::internal_update(uint64_t current_time)
{
	uint16_t current_timer1 = timer_value(1, current_time);
	uint16_t current_timer2 = timer_value(2, current_time);

	for(int i=0; i<8; i++)
		if(hso_info[i].active) {
			uint8_t cmd = hso_info[i].command;
			uint16_t t = hso_info[i].time;
			if(((cmd & 0x40) && t == current_timer2) ||
				(!(cmd & 0x40) && t == current_timer1)) {
				if(cmd != 0x18 && cmd != 0x19)
					logerror("%s: hso cam %02x %04x in slot %d triggered\n",
								tag(), cmd, t, i);
				trigger_cam(i, current_time);
			}
		}

	if(current_time == ad_done) {
		ad_done = 0;
		ad_result &= ~8;
	}

	if(current_time == serial_send_timer)
		serial_send_done();

	uint64_t event_time = 0;
	for(int i=0; i<8; i++) {
		if(!hso_info[i].active && hso_cam_hold.active) {
			hso_info[i] = hso_cam_hold;
			hso_cam_hold.active = false;
			logerror("%s: hso cam %02x %04x in slot %d from hold\n", tag(), hso_cam_hold.command, hso_cam_hold.time, i);
		}
		if(hso_info[i].active) {
			uint64_t new_time = timer_time_until(hso_info[i].command & 0x40 ? 2 : 1, current_time, hso_info[i].time);
			if(!event_time || new_time < event_time)
				event_time = new_time;
		}
	}

	if(ad_done && ad_done < event_time)
		event_time = ad_done;

	if(serial_send_timer && serial_send_timer < event_time)
		event_time = serial_send_timer;

	recompute_bcount(event_time);
}

c8095_device::c8095_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	i8x9x_device(mconfig, C8095, tag, owner, clock)
{
}

p8098_device::p8098_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	i8x9x_device(mconfig, P8098, tag, owner, clock)
{
}

DEFINE_DEVICE_TYPE(C8095, c8095_device, "c8095", "C8095")
DEFINE_DEVICE_TYPE(P8098, p8098_device, "p8098", "P8098")

#include "cpu/mcs96/i8x9x.hxx"
