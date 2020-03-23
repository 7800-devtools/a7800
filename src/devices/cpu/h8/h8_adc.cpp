// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
#include "emu.h"
#include "h8_adc.h"

// Verbosity level
// 0 = no messages
// 1 = everything
const int V = 0;

DEFINE_DEVICE_TYPE(H8_ADC_3337, h8_adc_3337_device, "h8_adc_3337", "H8/3337 ADC")
DEFINE_DEVICE_TYPE(H8_ADC_3006, h8_adc_3006_device, "h8_adc_3006", "H8/3006 ADC")
DEFINE_DEVICE_TYPE(H8_ADC_2245, h8_adc_2245_device, "h8_adc_2245", "H8/2245 ADC")
DEFINE_DEVICE_TYPE(H8_ADC_2320, h8_adc_2320_device, "h8_adc_2320", "H8/2320 ADC")
DEFINE_DEVICE_TYPE(H8_ADC_2357, h8_adc_2357_device, "h8_adc_2357", "H8/2357 ADC")
DEFINE_DEVICE_TYPE(H8_ADC_2655, h8_adc_2655_device, "h8_adc_2655", "H8/2655 ADC")

h8_adc_device::h8_adc_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	cpu(*this, DEVICE_SELF_OWNER),
	intc(nullptr), io(nullptr), intc_tag(nullptr), intc_vector(0), adcsr(0), adcr(0), register_mask(0), trigger(0), start_mode(0), start_channel(0),
	end_channel(0), start_count(0), mode(0), channel(0), count(0), analog_powered(false), adtrg(false), next_event(0)
{
	suspend_on_interrupt = false;
	analog_power_control = false;
}

void h8_adc_device::set_info(const char *_intc_tag, int _intc_vector)
{
	intc_tag = _intc_tag;
	intc_vector = _intc_vector;
}

READ8_MEMBER(h8_adc_device::addr8_r)
{
	if(V>=1) logerror("addr8_r %d %03x\n", offset, addr[offset >> 1]);
	return offset & 1 ? addr[offset >> 1] << 6 : addr[offset >> 1] >> 2;
}

READ16_MEMBER(h8_adc_device::addr16_r)
{
	if(V>=1) logerror("addr16_r %d %03x\n", offset, addr[offset]);
	return addr[offset];
}

READ8_MEMBER(h8_adc_device::adcsr_r)
{
	if(V>=1) logerror("adcsr_r %02x\n", adcsr);
	return adcsr;
}

READ8_MEMBER(h8_adc_device::adcr_r)
{
	if(V>=1) logerror("adcr_r %02x\n", adcr);
	return adcr;
}

WRITE8_MEMBER(h8_adc_device::adcsr_w)
{
	if(V>=1) logerror("adcsr_w %02x\n", data);
	uint8_t prev = adcsr;
	adcsr = (data & 0x7f) | (adcsr & data & F_ADF);
	mode_update();
	if((prev & F_ADF) && !(adcsr & F_ADF)) {
		if(mode & HALTED) {
			mode &= ~HALTED;
			if(!(adcsr & F_ADST)) {
				sampling();
				conversion_wait(false, false);
			} else
				done();
		}
	}

	if(!(prev & F_ADST) && (adcsr & F_ADST))
		start_conversion();
}

WRITE8_MEMBER(h8_adc_device::adcr_w)
{
	if(V>=1) logerror("adcr_w %02x\n", data);
	adcr = data;
	mode_update();
}

WRITE_LINE_MEMBER(h8_adc_device::adtrg_w)
{
	if(state != adtrg) {
		adtrg = state;
		if(!adtrg && (trigger & T_EXT) && !(adcsr & F_ADST)) {
			adcsr |= F_ADST;
			start_conversion();
		}
	}
}

void h8_adc_device::set_suspend(bool suspend)
{
}

void h8_adc_device::device_start()
{
	io = &cpu->space(AS_IO);
	intc = siblingdevice<h8_intc_device>(intc_tag);
	save_item(NAME(addr));
	save_item(NAME(buf));
	save_item(NAME(adcsr));
	save_item(NAME(adcr));
	save_item(NAME(trigger));
	save_item(NAME(start_mode));
	save_item(NAME(start_channel));
	save_item(NAME(end_channel));
	save_item(NAME(start_count));
	save_item(NAME(suspend_on_interrupt));
	save_item(NAME(analog_power_control));
	save_item(NAME(mode));
	save_item(NAME(channel));
	save_item(NAME(count));
	save_item(NAME(analog_powered));
	save_item(NAME(next_event));
	save_item(NAME(adtrg));
}

void h8_adc_device::device_reset()
{
	memset(addr, 0, sizeof(addr));
	memset(buf, 0, sizeof(buf));
	adcsr = adcr = 0;
	trigger = T_SOFT;
	start_mode = IDLE;
	start_channel = end_channel = 0;
	start_count = 1;
	mode = IDLE;
	channel = 0;
	count = 0;
	next_event = 0;
	mode_update();
	analog_powered = !analog_power_control;
	adtrg = true;
}

void h8_adc_device::done()
{
	mode = IDLE;
	adcsr &= ~F_ADST;
	if(analog_power_control)
		analog_powered = false;
}

uint64_t h8_adc_device::internal_update(uint64_t current_time)
{
	if(next_event && next_event <= current_time) {
		next_event = 0;
		timeout(current_time);
	}
	return next_event;
}

void h8_adc_device::conversion_wait(bool first, bool poweron, uint64_t current_time)
{
	if(current_time)
		next_event = current_time + conversion_time(first, poweron);
	else {
		next_event = cpu->total_cycles() + conversion_time(first, poweron);
		cpu->internal_update();
	}
}

void h8_adc_device::buffer_value(int port, int buffer)
{
	buf[buffer] = io->read_word(2*(h8_device::ADC_0 + port));
	if(V>=1) logerror("adc buffer %d -> %d:%03x\n", port, buffer, buf[buffer]);
}

void h8_adc_device::commit_value(int reg, int buffer)
{
	reg &= register_mask;
	if(V>=1) logerror("adc commit %d -> %d:%03x\n", buffer, reg, buf[buffer]);
	addr[reg] = buf[buffer];
}

void h8_adc_device::sampling()
{
	if(mode & COUNTED)
		channel = get_channel_index(start_count - count);
	if(mode & DUAL) {
		buffer_value(channel, 0);
		buffer_value(channel+1, 1);
	} else
		buffer_value(channel);
}

void h8_adc_device::start_conversion()
{
	mode = start_mode;
	channel = start_channel;
	count = start_count;
	sampling();
	conversion_wait(true, !analog_powered);
	analog_powered = true;
}

void h8_adc_device::timeout(uint64_t current_time)
{
	if(mode & BUFFER) {
		do_buffering((mode & DUAL) && (channel & 1));
		if((mode & DUAL) && !(channel & 1)) {
			channel++;
			conversion_wait(false, false, current_time);
			return;
		}
	} else {
		if(mode & DUAL) {
			if(channel & 1)
				commit_value(channel, 1);
			else {
				commit_value(channel, 0);
				channel++;
				conversion_wait(false, false, current_time);
				return;
			}
		} else
			commit_value(channel);
	}

	if(mode & ROTATE) {
		if(channel != end_channel) {
			channel++;
			sampling();
			conversion_wait(false, false, current_time);
			return;
		}
		channel = start_channel;
	}

	if(mode & COUNTED) {
		count--;
		if(count) {
			sampling();
			conversion_wait(false, false, current_time);
			return;
		}
	}

	adcsr |= F_ADF;
	if(adcsr & F_ADIE)
		intc->internal_interrupt(intc_vector);

	if(mode & REPEAT) {
		if(suspend_on_interrupt && (adcsr & F_ADIE)) {
			mode |= HALTED;
			return;
		}
		channel = start_channel;
		count = start_count;
		sampling();
		conversion_wait(false, false, current_time);
		return;
	}

	done();
}

void h8_adc_device::do_buffering(int buffer)
{
	throw emu_fatalerror("%s: Buffering requested but unimplemented for this device\n", tag());
}

int h8_adc_device::get_channel_index(int count)
{
	throw emu_fatalerror("%s: Indexing requested but unimplemented for this device\n", tag());
}


h8_adc_3337_device::h8_adc_3337_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h8_adc_device(mconfig, H8_ADC_3337, tag, owner, clock)
{
	register_mask = 3;
}

int h8_adc_3337_device::conversion_time(bool first, bool poweron)
{
	int tm;
	if(first)
		tm = adcsr & 0x08 ? 134 : 266;
	else
		tm = adcsr & 0x08 ? 128 : 256;
	return tm;
}

void h8_adc_3337_device::mode_update()
{
	trigger = adcr & 0x80 ? T_EXT : T_SOFT;

	if(adcsr & 0x10) {
		start_mode = ACTIVE | ROTATE;
		start_channel = adcsr & 4;
		end_channel = adcsr & 7;
	} else {
		start_mode = ACTIVE;
		start_channel = end_channel = adcsr & 7;
	}
}


h8_adc_3006_device::h8_adc_3006_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h8_adc_device(mconfig, H8_ADC_3006, tag, owner, clock)
{
	register_mask = 3;
}

int h8_adc_3006_device::conversion_time(bool first, bool poweron)
{
	int tm;
	if(first)
		tm = adcsr & 0x08 ? 70 : 134;
	else
		tm = adcsr & 0x08 ? 66 : 128;
	return tm;
}

void h8_adc_3006_device::mode_update()
{
	trigger = adcr & 0x80 ? T_EXT|T_TIMER : T_SOFT;

	if(adcsr & 0x10) {
		start_mode = ACTIVE | ROTATE;
		start_channel = adcsr & 4;
		end_channel = adcsr & 7;
	} else {
		start_mode = ACTIVE;
		start_channel = end_channel = adcsr & 7;
	}
}


h8_adc_2245_device::h8_adc_2245_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h8_adc_device(mconfig, H8_ADC_2245, tag, owner, clock)
{
	register_mask = 3;
}

int h8_adc_2245_device::conversion_time(bool first, bool poweron)
{
	int tm;
	if(first)
		tm = adcsr & 0x08 ? 134 : 266;
	else
		tm = adcsr & 0x08 ? 128 : 256;
	return tm;
}

void h8_adc_2245_device::mode_update()
{
	trigger = 1 << ((adcr >> 6) & 3);

	if(adcsr & 0x10) {
		start_mode = ACTIVE | ROTATE;
		start_channel = 0;
		end_channel = adcsr & 3;
	} else {
		start_mode = ACTIVE;
		start_channel = end_channel = adcsr & 3;
	}
}


h8_adc_2320_device::h8_adc_2320_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h8_adc_device(mconfig, H8_ADC_2320, tag, owner, clock)
{
	register_mask = 3;
}

int h8_adc_2320_device::conversion_time(bool first, bool poweron)
{
	int tm;
	if(first)
		if(adcr & 0x04)
			tm = adcsr & 0x08 ? 134 : 266;
		else
			tm = adcsr & 0x08 ? 68 : 580;
	else
		if(adcr & 0x04)
			tm = adcsr & 0x08 ? 128 : 256;
		else
			tm = adcsr & 0x08 ? 64 : 512;
	return tm;
}

void h8_adc_2320_device::mode_update()
{
	trigger = 1 << ((adcr >> 6) & 3);

	if(adcsr & 0x10) {
		start_mode = ACTIVE | ROTATE;
		start_channel = adcsr & 4;
		end_channel = adcsr & 7;
	} else {
		start_mode = ACTIVE;
		start_channel = end_channel = adcsr & 7;
	}
}


h8_adc_2357_device::h8_adc_2357_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h8_adc_device(mconfig, H8_ADC_2357, tag, owner, clock)
{
	register_mask = 3;
}

int h8_adc_2357_device::conversion_time(bool first, bool poweron)
{
	int tm;
	if(first)
		tm = adcsr & 0x08 ? 134 : 266;
	else
		tm = adcsr & 0x08 ? 128 : 256;
	return tm;
}

void h8_adc_2357_device::mode_update()
{
	trigger = 1 << ((adcr >> 6) & 3);

	if(adcsr & 0x10) {
		start_mode = ACTIVE | ROTATE;
		start_channel = adcsr & 4;
		end_channel = adcsr & 7;
	} else {
		start_mode = ACTIVE;
		start_channel = end_channel = adcsr & 7;
	}
}


h8_adc_2655_device::h8_adc_2655_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h8_adc_device(mconfig, H8_ADC_2655, tag, owner, clock)
{
	suspend_on_interrupt = true;
	register_mask = 7;
}

int h8_adc_2655_device::conversion_time(bool first, bool poweron)
{
	int tm = adcsr & 0x10 ? 44 : 24;
	if(first)
		tm += adcsr & 0x10 ? 20 : 10;
	if(poweron)
		tm += 200;
	return tm;
}

void h8_adc_2655_device::mode_update()
{
	trigger = 1 << ((adcr >> 4) & 3);
	analog_power_control = !(adcr & 0x40);

	mode = ACTIVE | (adcr & 0x08 ? REPEAT : 0);

	if(adcsr & 0x03) {
		mode |= BUFFER;

	}

	if(adcsr & 0x08) {
		mode |= ROTATE;
		start_channel = 0;
		if(adcr & 0x04) {
			mode |= DUAL;
			end_channel = (adcsr & 6)+1;
		} else
			end_channel = adcsr & 7;
	} else
		start_channel = end_channel = adcsr & 7;

}

void h8_adc_2655_device::do_buffering(int buffer)
{
	if((mode & COUNTED) && channel >= 2) {
		commit_value(channel, buffer);
		return;
	}
	switch(adcsr & 3) {
	case 0:
		commit_value(channel, buffer);
		break;
	case 1:
		addr[1] = addr[0];
		commit_value(0, buffer);
		break;
	case 2:
		addr[2+buffer] = addr[buffer];
		commit_value(buffer, buffer);
		break;
	case 3:
		addr[3] = addr[2];
		addr[2] = addr[1];
		addr[1] = addr[0];
		commit_value(0, buffer);
		break;
	}
}

int h8_adc_2655_device::get_channel_index(int count)
{
	abort();
}
