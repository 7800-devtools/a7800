// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
#include "emu.h"
#include "h8_timer16.h"

// Verbosity level
// 0 = no messages
// 1 = everything
const int V = 0;

DEFINE_DEVICE_TYPE(H8_TIMER16,          h8_timer16_device,          "h8_timer16",          "H8 16-bit timer")
DEFINE_DEVICE_TYPE(H8_TIMER16_CHANNEL,  h8_timer16_channel_device,  "h8_timer16_channel",  "H8 16-bit timer channel")
DEFINE_DEVICE_TYPE(H8H_TIMER16_CHANNEL, h8h_timer16_channel_device, "h8h_timer16_channel", "H8H 16-bit timer channel")
DEFINE_DEVICE_TYPE(H8S_TIMER16_CHANNEL, h8s_timer16_channel_device, "h8s_timer16_channel", "H8S 16-bit timer channel")

h8_timer16_channel_device::h8_timer16_channel_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h8_timer16_channel_device(mconfig, H8_TIMER16_CHANNEL, tag, owner, clock)
{
	chain_tag = nullptr;
}

h8_timer16_channel_device::h8_timer16_channel_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	cpu(*this, "^^"), chained_timer(nullptr), intc(nullptr), intc_tag(nullptr), tier_mask(0), tgr_count(0), tbr_count(0), tgr_clearing(0), tcr(0), tier(0), ier(0), isr(0), clock_type(0),
	clock_divider(0), tcnt(0), last_clock_update(0), event_time(0), phase(0), counter_cycle(0), counter_incrementing(false), channel_active(false)
{
	chain_tag = nullptr;
}

void h8_timer16_channel_device::set_info(int _tgr_count, int _tbr_count, const char *intc, int irq_base)
{
	tgr_count = _tgr_count;
	tbr_count = _tbr_count;
	intc_tag = intc;

	interrupt[0] = irq_base++;
	interrupt[1] = irq_base++;
	interrupt[2] = -1;
	interrupt[3] = -1;
	interrupt[4] = irq_base;
	interrupt[5] = irq_base;
}

READ8_MEMBER(h8_timer16_channel_device::tcr_r)
{
	return tcr;
}

WRITE8_MEMBER(h8_timer16_channel_device::tcr_w)
{
	update_counter();
	tcr = data;
	if(V>=1) logerror("tcr_w %02x\n", data);
	tcr_update();
	recalc_event();
}

READ8_MEMBER(h8_timer16_channel_device::tmdr_r)
{
	return 0x00;
}

WRITE8_MEMBER(h8_timer16_channel_device::tmdr_w)
{
	if(V>=1) logerror("tmdr_w %02x\n", data);
}

READ8_MEMBER(h8_timer16_channel_device::tior_r)
{
	return 0x00;
}

WRITE8_MEMBER(h8_timer16_channel_device::tior_w)
{
	if(V>=1) logerror("tior_w %d, %02x\n", offset, data);
}

void h8_timer16_channel_device::set_ier(uint8_t value)
{
	update_counter();
	ier = value;
	recalc_event();
}

void h8_timer16_channel_device::set_enable(bool enable)
{
	update_counter();
	channel_active = enable;
	recalc_event();
}

READ8_MEMBER(h8_timer16_channel_device::tier_r)
{
	return tier;
}

WRITE8_MEMBER(h8_timer16_channel_device::tier_w)
{
	update_counter();
	if(V>=1) logerror("tier_w %02x\n", data);
	tier = data;
	tier_update();
	if(V>=1) logerror("irq %c%c%c%c%c%c trigger=%d\n",
						ier & IRQ_A ? 'a' : '.',
						ier & IRQ_B ? 'b' : '.',
						ier & IRQ_C ? 'c' : '.',
						ier & IRQ_D ? 'd' : '.',
						ier & IRQ_V ? 'v' : '.',
						ier & IRQ_U ? 'u' : '.',
						ier & IRQ_TRIG ? 1 : 0);
	recalc_event();
}

READ8_MEMBER(h8_timer16_channel_device::tsr_r)
{
	return isr_to_sr();
}

WRITE8_MEMBER(h8_timer16_channel_device::tsr_w)
{
	if(V>=1) logerror("tsr_w %02x\n", data);
	isr_update(data);
}

READ16_MEMBER(h8_timer16_channel_device::tcnt_r)
{
	update_counter();
	return tcnt;
}

WRITE16_MEMBER(h8_timer16_channel_device::tcnt_w)
{
	update_counter();
	COMBINE_DATA(&tcnt);
	if(V>=1) logerror("tcnt_w %04x\n", tcnt);
	recalc_event();
}

READ16_MEMBER(h8_timer16_channel_device::tgr_r)
{
	return tgr[offset];
}

WRITE16_MEMBER(h8_timer16_channel_device::tgr_w)
{
	update_counter();
	COMBINE_DATA(tgr + offset);
	if(V>=1) logerror("tgr%c_w %04x\n", 'a'+offset, tgr[offset]);
	recalc_event();
}

READ16_MEMBER(h8_timer16_channel_device::tbr_r)
{
	return tgr[offset+tgr_count];
}

WRITE16_MEMBER(h8_timer16_channel_device::tbr_w)
{
	COMBINE_DATA(tgr + offset + tgr_count);
	if(V>=1) logerror("tbr%c_w %04x\n", 'a'+offset, tgr[offset]);
}

void h8_timer16_channel_device::device_start()
{
	intc = owner()->siblingdevice<h8_intc_device>(intc_tag);
	channel_active = false;
	device_reset();

	save_item(NAME(tgr_clearing));
	save_item(NAME(tcr));
	save_item(NAME(tier));
	save_item(NAME(ier));
	save_item(NAME(isr));
	save_item(NAME(clock_type));
	save_item(NAME(clock_divider));
	save_item(NAME(tcnt));
	save_item(NAME(tgr));
	save_item(NAME(last_clock_update));
	save_item(NAME(event_time));
	save_item(NAME(phase));
	save_item(NAME(counter_cycle));
	save_item(NAME(counter_incrementing));
	save_item(NAME(channel_active));
}

void h8_timer16_channel_device::device_reset()
{
	// Don't touch channel_active here, top level device handles it
	tcr = 0;
	tcnt = 0;
	memset(tgr, 0xff, sizeof(tgr));
	tgr_clearing = TGR_CLEAR_NONE;
	clock_type = DIV_1;
	clock_divider = 0;
	counter_cycle = 0x10000;
	phase = 0;
	tier = 0x40 & tier_mask;
	ier = 0;
	isr = 0;
	last_clock_update = 0;
	event_time = 0;
	counter_incrementing = true;
}

uint64_t h8_timer16_channel_device::internal_update(uint64_t current_time)
{
	if(event_time && current_time >= event_time) {
		update_counter(current_time);
		recalc_event(current_time);
	}

	return event_time;
}

void h8_timer16_channel_device::update_counter(uint64_t cur_time)
{
	if(clock_type != DIV_1)
		return;

	if(!cur_time)
		cur_time = cpu->total_cycles();

	if(!channel_active) {
		last_clock_update = cur_time;
		return;
	}

	uint64_t base_time = last_clock_update;
	uint64_t new_time = cur_time;
	if(clock_divider) {
		base_time = (base_time + phase) >> clock_divider;
		new_time = (new_time + phase) >> clock_divider;
	}
	if(counter_incrementing) {
		int tt = tcnt + new_time - base_time;
		tcnt = tt % counter_cycle;

		for(int i=0; i<tgr_count; i++)
			if((ier & (1 << i)) && (tt == tgr[i] || tcnt == tgr[i]) && interrupt[i] != -1) {
				isr |= 1 << i;
				intc->internal_interrupt(interrupt[i]);
			}
		if(tt >= 0x10000 && (ier & IRQ_V) && interrupt[4] != -1) {
			isr |= IRQ_V;
			intc->internal_interrupt(interrupt[4]);
		}
	} else
		tcnt = (((tcnt ^ 0xffff) + new_time - base_time) % counter_cycle) ^ 0xffff;
	last_clock_update = cur_time;
}

void h8_timer16_channel_device::recalc_event(uint64_t cur_time)
{
	if(!channel_active) {
		event_time = 0;
		return;
	}

	bool update_cpu = cur_time == 0;
	uint64_t old_event_time = event_time;

	if(clock_type != DIV_1) {
		event_time = 0;
		if(old_event_time && update_cpu)
			cpu->internal_update();

		return;
	}

	if(!cur_time)
		cur_time = cpu->total_cycles();

	if(counter_incrementing) {
		uint32_t event_delay = 0xffffffff;
		if(tgr_clearing >= 0 && tgr[tgr_clearing])
			counter_cycle = tgr[tgr_clearing];
		else {
			counter_cycle = 0x10000;
			if(ier & IRQ_V) {
				event_delay = counter_cycle - tcnt;
				if(!event_delay)
					event_delay = counter_cycle;
			}
		}
		for(int i=0; i<tgr_count; i++)
			if(ier & (1 << i)) {
				uint32_t new_delay = 0xffffffff;
				if(tgr[i] > tcnt) {
					if(tcnt >= counter_cycle || tgr[i] <= counter_cycle)
						new_delay = tgr[i] - tcnt;
				} else if(tgr[i] <= counter_cycle) {
					if(tcnt < counter_cycle)
						new_delay = (counter_cycle - tcnt) + tgr[i];
					else
						new_delay = (0x10000 - tcnt) + tgr[i];
				}

				if(event_delay > new_delay)
					event_delay = new_delay;
			}

		if(event_delay != 0xffffffff)
			event_time = ((((cur_time + (1ULL << clock_divider) - phase) >> clock_divider) + event_delay - 1) << clock_divider) + phase;
		else
			event_time = 0;

	} else {
		logerror("decrementing counter\n");
		exit(1);
	}

	if(old_event_time != event_time && update_cpu)
		cpu->internal_update();
}

h8_timer16_device::h8_timer16_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, H8_TIMER16, tag, owner, clock),
	cpu(*this, DEVICE_SELF_OWNER)
{
}

void h8_timer16_device::set_info(int count, uint8_t tstr)
{
	timer_count = count;
	default_tstr = tstr;
}

void h8_timer16_device::device_start()
{
	memset(timer_channel, 0, sizeof(timer_channel));
	for(int i=0; i<timer_count; i++) {
		char tm[2];
		sprintf(tm, "%d", i);
		timer_channel[i] = subdevice<h8_timer16_channel_device>(tm);
	}

	save_item(NAME(tstr));
}

void h8_timer16_device::device_reset()
{
	tstr = default_tstr;
	for(int i=0; i<timer_count; i++)
		timer_channel[i]->set_enable((tstr >> i) & 1);
}


READ8_MEMBER(h8_timer16_device::tstr_r)
{
	return tstr;
}

WRITE8_MEMBER(h8_timer16_device::tstr_w)
{
	if(V>=1) logerror("tstr_w %02x\n", data);
	tstr = data;
	for(int i=0; i<timer_count; i++)
		timer_channel[i]->set_enable((tstr >> i) & 1);
}

READ8_MEMBER(h8_timer16_device::tsyr_r)
{
	return 0x00;
}

WRITE8_MEMBER(h8_timer16_device::tsyr_w)
{
	if(V>=1) logerror("tsyr_w %02x\n", data);
}

READ8_MEMBER(h8_timer16_device::tmdr_r)
{
	return 0x00;
}

WRITE8_MEMBER(h8_timer16_device::tmdr_w)
{
	if(V>=1) logerror("tmdr_w %02x\n", data);
}

READ8_MEMBER(h8_timer16_device::tfcr_r)
{
	return 0x00;
}

WRITE8_MEMBER(h8_timer16_device::tfcr_w)
{
	if(V>=1) logerror("tfcr_w %02x\n", data);
}

READ8_MEMBER(h8_timer16_device::toer_r)
{
	return 0x00;
}

WRITE8_MEMBER(h8_timer16_device::toer_w)
{
	if(V>=1) logerror("toer_w %02x\n", data);
}

READ8_MEMBER(h8_timer16_device::tocr_r)
{
	return 0x00;
}

WRITE8_MEMBER(h8_timer16_device::tocr_w)
{
	if(V>=1) logerror("tocr_w %02x\n", data);
}

READ8_MEMBER(h8_timer16_device::tisr_r)
{
	uint8_t r = 0;
	for(int i=0; i<timer_count; i++)
		r |= timer_channel[i]->tisr_r(offset) << i;
	for(int i=timer_count; i<4; i++)
		r |= 0x11 <<i;

	if(V>=1) logerror("tisr%c_r %02x\n", 'a'+offset, r);

	return r;
}

WRITE8_MEMBER(h8_timer16_device::tisr_w)
{
	if(V>=1) logerror("tisr%c_w %02x\n", 'a'+offset, data);
	for(int i=0; i<timer_count; i++)
		timer_channel[i]->tisr_w(offset, data >> i);
}

READ8_MEMBER(h8_timer16_device::tisrc_r)
{
	return tisr_r(space, 2, mem_mask);
}

WRITE8_MEMBER(h8_timer16_device::tisrc_w)
{
	tisr_w(space, 2, data, mem_mask);
}

WRITE8_MEMBER(h8_timer16_device::tolr_w)
{
	if(V>=1) logerror("tocr_w %02x\n", data);
}



void h8_timer16_channel_device::tier_update()
{
}

void h8_timer16_channel_device::isr_update(uint8_t val)
{
}

uint8_t h8_timer16_channel_device::isr_to_sr() const
{
	return 0x00;
}

void h8_timer16_channel_device::tcr_update()
{
}

void h8_timer16_channel_device::tisr_w(int offset, uint8_t value)
{
	update_counter();
	if(!(value & 0x01)) {
		switch(offset) {
		case 0:
			isr &= ~IRQ_A;
			break;
		case 1:
			isr &= ~IRQ_B;
			break;
		case 2:
			isr &= ~IRQ_V;
			break;
		}
	}
	if(value & 0x10) {
		switch(offset) {
		case 0:
			ier |= IRQ_A;
			break;
		case 1:
			ier |= IRQ_B;
			break;
		case 2:
			ier |= IRQ_V;
			break;
		}
	} else {
		switch(offset) {
		case 0:
			ier &= ~IRQ_A;
			break;
		case 1:
			ier &= ~IRQ_B;
			break;
		case 2:
			ier &= ~IRQ_V;
			break;
		}
	}
	recalc_event();
}

uint8_t h8_timer16_channel_device::tisr_r(int offset) const
{
	switch(offset) {
	case 0:
		return ((ier & IRQ_A) ? 0x10 : 0x00) | ((isr & IRQ_A) ? 0x01 : 0x00);
	case 1:
		return ((ier & IRQ_B) ? 0x10 : 0x00) | ((isr & IRQ_B) ? 0x01 : 0x00);
	case 2:
		return ((ier & IRQ_V) ? 0x10 : 0x00) | ((isr & IRQ_V) ? 0x01 : 0x00);
	}
	return 0x00;
}

h8h_timer16_channel_device::h8h_timer16_channel_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h8_timer16_channel_device(mconfig, H8H_TIMER16_CHANNEL, tag, owner, clock)
{
}

h8h_timer16_channel_device::~h8h_timer16_channel_device()
{
}

void h8h_timer16_channel_device::set_info(int _tgr_count, int _tbr_count, const char *intc, int irq_base)
{
	tgr_count = _tgr_count;
	tbr_count = _tbr_count;
	intc_tag = intc;

	interrupt[0] = irq_base++;
	interrupt[1] = irq_base++;
	interrupt[2] = -1;
	interrupt[3] = -1;
	interrupt[4] = irq_base;
	interrupt[5] = irq_base;
}

void h8h_timer16_channel_device::tier_update()
{
	tier = tier | 0xf8;
	ier =
		(tier & 0x01 ? IRQ_A : 0) |
		(tier & 0x02 ? IRQ_B : 0) |
		(tier & 0x04 ? IRQ_V : 0);
}

void h8h_timer16_channel_device::isr_update(uint8_t val)
{
	if(!(val & 1))
		isr &= ~IRQ_A;
	if(!(val & 2))
		isr &= ~IRQ_B;
	if(!(val & 4))
		isr &= ~IRQ_V;
}

uint8_t h8h_timer16_channel_device::isr_to_sr() const
{
	return 0xf8 | (isr & IRQ_V ? 4 : 0) | (isr & IRQ_B ? 2 : 0) | (isr & IRQ_A ? 1 : 0);
}


void h8h_timer16_channel_device::tcr_update()
{
	switch(tcr & 0x60) {
	case 0x00:
		tgr_clearing = TGR_CLEAR_NONE;
		if(V>=1) logerror("No automatic tcnt clearing\n");
		break;
	case 0x20: case 0x40: {
		tgr_clearing = tcr & 0x20 ? 0 : 1;
		if(V>=1) logerror("Auto-clear on tgr%c (%04x)\n", 'a'+tgr_clearing, tgr[tgr_clearing]);
		break;
	}
	case 0x60:
		tgr_clearing = TGR_CLEAR_EXT;
		if(V>=1) logerror("External sync clear\n");
		break;
	}

	int count_type = tcr & 7;
	if(count_type < 4) {
		clock_type = DIV_1;
		clock_divider = count_type;
		if(V>=1) logerror("clock divider %d (%d)\n", clock_divider, 1 << clock_divider);
		if(count_type <= DIV_2)
			phase = 0;
		else {
			switch(tcr & 0x18) {
			case 0x00:
				phase = 0;
				if(V>=1) logerror("Phase 0\n");
				break;
			case 0x08:
				phase = 1 << (clock_divider-1);
				if(V>=1) logerror("Phase 180\n");
				break;
			case 0x10: case 0x18:
				phase = 0;
				clock_divider--;
				if(V>=1) logerror("Phase 0+180\n");
				break;
			}
		}
	} else {
		clock_type = INPUT_A + (count_type-4);
		clock_divider = 0;
		phase = 0;
		if(V>=1) logerror("counting input %c\n", 'a'+count_type-INPUT_A);
	}
}

h8s_timer16_channel_device::h8s_timer16_channel_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h8_timer16_channel_device(mconfig, H8S_TIMER16_CHANNEL, tag, owner, clock)
{
}

h8s_timer16_channel_device::~h8s_timer16_channel_device()
{
}

void h8s_timer16_channel_device::set_chain(const char *_chain_tag)
{
	chain_tag = _chain_tag;
}

void h8s_timer16_channel_device::set_info(int _tgr_count, uint8_t _tier_mask, const char *intc, int irq_base,
											int t0, int t1, int t2, int t3, int t4, int t5, int t6, int t7)
{
	tgr_count = _tgr_count;
	tbr_count = 0;
	tier_mask = _tier_mask;
	intc_tag = intc;

	interrupt[0] = irq_base++;
	interrupt[1] = irq_base++;
	interrupt[2] = tier_mask & 0x04 ? -1 : irq_base++;
	interrupt[3] = tier_mask & 0x08 ? -1 : irq_base++;
	interrupt[4] = irq_base;
	interrupt[5] = tier_mask & 0x20 ? -1 : irq_base++;

	count_types[0] = t0;
	count_types[1] = t1;
	count_types[2] = t2;
	count_types[3] = t3;
	count_types[4] = t4;
	count_types[5] = t5;
	count_types[6] = t6;
	count_types[7] = t7;
}

void h8s_timer16_channel_device::tier_update()
{
	tier = (tier & ~tier_mask) | 0x40;
	ier =
		(tier & 0x01 ? IRQ_A : 0) |
		(tier & 0x02 ? IRQ_B : 0) |
		(tier & 0x04 ? IRQ_C : 0) |
		(tier & 0x08 ? IRQ_D : 0) |
		(tier & 0x10 ? IRQ_V : 0) |
		(tier & 0x20 ? IRQ_U : 0) |
		(tier & 0x80 ? IRQ_TRIG : 0);
}

void h8s_timer16_channel_device::isr_update(uint8_t val)
{
	isr &= (val | tier_mask | 0xc0);
}

uint8_t h8s_timer16_channel_device::isr_to_sr() const
{
	return 0xc0 | isr;
}

void h8s_timer16_channel_device::tcr_update()
{
	switch(tcr & 0x60) {
	case 0x00:
		tgr_clearing = TGR_CLEAR_NONE;
		if(V>=1) logerror("No automatic tcnt clearing\n");
		break;
	case 0x20: case 0x40: {
		tgr_clearing = tcr & 0x20 ? 0 : 1;
		if(tgr_count > 2 && (tcr & 0x80))
			tgr_clearing += 2;
		if(V>=1) logerror("Auto-clear on tgr%c\n", 'a'+tgr_clearing);
		break;
	}
	case 0x60:
		tgr_clearing = TGR_CLEAR_EXT;
		if(V>=1) logerror("External sync clear\n");
		break;
	}

	int count_type = count_types[tcr & 7];
	if(count_type >= DIV_1 && clock_type <= DIV_4) {
		clock_type = DIV_1;
		clock_divider = count_type - DIV_1;
		if(V>=1) logerror("clock divider %d (%d)\n", clock_divider, 1 << clock_divider);
		if(!clock_divider)
			phase = 0;
		else {
			switch(tcr & 0x18) {
			case 0x00:
				phase = 0;
				if(V>=1) logerror("Phase 0\n");
				break;
			case 0x08:
				phase = 1 << (clock_divider-1);
				if(V>=1) logerror("Phase 180\n");
				break;
			case 0x10: case 0x18:
				phase = 0;
				clock_divider--;
				if(V>=1) logerror("Phase 0+180\n");
				break;
			}
		}

	} else if(count_type == CHAIN) {
		clock_type = CHAIN;
		clock_divider = 0;
		phase = 0;
		if(V>=1) logerror("chained timer\n");

	} else if(count_type >= INPUT_A && count_type <= INPUT_D) {
		clock_type = count_type;
		clock_divider = 0;
		phase = 0;
		if(V>=1) logerror("counting input %c\n", 'a'+count_type-INPUT_A);
	}
}
