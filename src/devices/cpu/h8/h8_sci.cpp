// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
#include "emu.h"
#include "h8_sci.h"

// Verbosity level
// 0 = no messages
// 1 = transmitted/recieved bytes, reception errors and clock setup
// 2 = everything but status register reads
// 3 = everything
const int V = 1;


DEFINE_DEVICE_TYPE(H8_SCI, h8_sci_device, "h8_sci", "H8 Serial Communications Interface")

const char *const h8_sci_device::state_names[] = { "idle", "start", "bit", "parity", "stop", "last-tick" };

h8_sci_device::h8_sci_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, H8_SCI, tag, owner, clock),
	cpu(*this, DEVICE_SELF_OWNER),
	tx_cb(*this),
	clk_cb(*this), intc(nullptr), intc_tag(nullptr), external_to_internal_ratio(0), internal_to_external_ratio(0), sync_timer(nullptr), eri_int(0), rxi_int(0), txi_int(0), tei_int(0),
	tx_state(0), rx_state(0), tx_bit(0), rx_bit(0), clock_state(0), clock_mode(0), tx_parity(0), rx_parity(0), ext_clock_counter(0), clock_value(false), ext_clock_value(false), rx_value(false),
	rdr(0), tdr(0), smr(0), scr(0), ssr(0), brr(0), rsr(0), tsr(0), clock_base(0), divider(0)
{
	external_clock_period = attotime::never;
}

void h8_sci_device::set_info(const char *_intc_tag, int eri, int rxi, int txi, int tei)
{
	intc_tag = _intc_tag;
	eri_int = eri;
	rxi_int = rxi;
	txi_int = txi;
	tei_int = tei;
}

void h8_sci_device::set_external_clock_period(const attotime &period)
{
	external_clock_period = period;
}

WRITE8_MEMBER(h8_sci_device::smr_w)
{
	smr = data;
	if(V>=2) logerror("smr_w %02x %s %c%c%c%s /%d (%06x)\n", data,
						data & SMR_CA ? "sync" : "async",
						data & SMR_CHR ? '7' : '8',
						data & SMR_PE ? data & SMR_OE ? 'o' : 'e' : 'n',
						data & SMR_STOP ? '2' : '1',
						data & SMR_MP ? " mp" : "",
						1 << 2*(data & SMR_CKS),
						cpu->pc());
	clock_update();
}

READ8_MEMBER(h8_sci_device::smr_r)
{
	if(V>=2) logerror("smr_r %02x (%06x)\n", smr, cpu->pc());
	return smr;
}

WRITE8_MEMBER(h8_sci_device::brr_w)
{
	brr = data;
	if(V>=2) logerror("brr_w %02x (%06x)\n", data, cpu->pc());
	clock_update();
}

READ8_MEMBER(h8_sci_device::brr_r)
{
	if(V>=2) logerror("brr_r %02x (%06x)\n", brr, cpu->pc());
	return brr;
}

bool h8_sci_device::is_sync_start() const
{
	return (smr & SMR_CA) && ((scr & (SCR_TE|SCR_RE)) == (SCR_TE|SCR_RE));
}

bool h8_sci_device::has_recv_error() const
{
	return ssr & (SSR_ORER|SSR_PER|SSR_FER);
}

WRITE8_MEMBER(h8_sci_device::scr_w)
{
	if(V>=2) logerror("scr_w %02x%s%s%s%s%s%s clk=%d (%06x)\n", data,
						data & SCR_TIE  ? " txi" : "",
						data & SCR_RIE  ? " rxi" : "",
						data & SCR_TE   ? " tx" : "",
						data & SCR_RE   ? " rx" : "",
						data & SCR_MPIE ? " mpi" : "",
						data & SCR_TEIE ? " tei" : "",
						data & SCR_CKE,
						cpu->pc());

	uint8_t delta = scr ^ data;
	scr = data;
	clock_update();

	if((delta & SCR_RE) && !(scr & SCR_RE)) {
		rx_state = ST_IDLE;
		clock_stop(CLK_RX);
	}

	if((delta & SCR_RE) && (scr & SCR_RE) && rx_state == ST_IDLE && !has_recv_error() && !is_sync_start())
		rx_start();
	if((delta & SCR_TIE) && (scr & SCR_TIE) && (ssr & SSR_TDRE))
		intc->internal_interrupt(txi_int);
	if((delta & SCR_TEIE) && (scr & SCR_TEIE) && (ssr & SSR_TEND))
		intc->internal_interrupt(tei_int);
	if((delta & SCR_RIE) && (scr & SCR_RIE) && (ssr & SSR_RDRF))
		intc->internal_interrupt(rxi_int);
	if((delta & SCR_RIE) && (scr & SCR_RIE) && has_recv_error())
		intc->internal_interrupt(eri_int);
}

READ8_MEMBER(h8_sci_device::scr_r)
{
	if(V>=2) logerror("scr_r %02x (%06x)\n", scr, cpu->pc());
	return scr;
}

WRITE8_MEMBER(h8_sci_device::tdr_w)
{
	if(V>=2) logerror("tdr_w %02x (%06x)\n", data, cpu->pc());
	tdr = data;
	if(cpu->access_is_dma()) {
		ssr &= ~SSR_TDRE;
		if(tx_state == ST_IDLE)
			tx_start();
	}
}

READ8_MEMBER(h8_sci_device::tdr_r)
{
	if(V>=2) logerror("tdr_r %02x (%06x)\n", tdr, cpu->pc());
	return tdr;
}

WRITE8_MEMBER(h8_sci_device::ssr_w)
{
	cpu->synchronize();

	if(!(scr & SCR_TE)) {
		data |= SSR_TDRE;
		ssr |= SSR_TDRE;
	}
	if((ssr & SSR_TDRE) && !(data & SSR_TDRE))
	{
		ssr &= ~SSR_TEND;
		scr &= ~SCR_TIE;
	}
	ssr = ((ssr & ~SSR_MPBT) | (data & SSR_MPBT)) & (data | (SSR_TEND|SSR_MPB|SSR_MPBT));
	if(V>=2) logerror("ssr_w %02x -> %02x (%06x)\n", data, ssr, cpu->pc());

	if(tx_state == ST_IDLE && !(ssr & SSR_TDRE))
		tx_start();

	if((scr & SCR_RE) && rx_state == ST_IDLE && !has_recv_error() && !is_sync_start())
		rx_start();
}

READ8_MEMBER(h8_sci_device::ssr_r)
{
	if(V>=3) logerror("ssr_r %02x (%06x)\n", ssr, cpu->pc());
	return ssr;
}

READ8_MEMBER(h8_sci_device::rdr_r)
{
	if(V>=2) logerror("rdr_r %02x (%06x)\n", rdr, cpu->pc());
	if(cpu->access_is_dma())
		ssr &= ~SSR_RDRF;
	return rdr;
}

WRITE8_MEMBER(h8_sci_device::scmr_w)
{
	if(V>=2) logerror("scmr_w %02x (%06x)\n", data, cpu->pc());
}

READ8_MEMBER(h8_sci_device::scmr_r)
{
	if(V>=2) logerror("scmr_r (%06x)\n", cpu->pc());
	return 0x00;
}

void h8_sci_device::clock_update()
{
	// Sync: Divider must be the time of a half-period (both edges are used, datarate*2)
	// Async: Divider must be the time of one period (only raising edge used, datarate*16)

	divider = 2 << (2*(smr & SMR_CKS));
	divider *= brr+1;

	if(smr & SMR_CA) {
		if(scr & SCR_CKE1)
			clock_mode = CLKM_EXTERNAL_SYNC;
		else
			clock_mode = CLKM_INTERNAL_SYNC_OUT;
	} else {
		if(scr & SCR_CKE1)
			clock_mode = CLKM_EXTERNAL_ASYNC;
		else if(scr & SCR_CKE0)
			clock_mode = CLKM_INTERNAL_ASYNC_OUT;
		else
			clock_mode = CLKM_INTERNAL_ASYNC;
	}

	if(clock_mode == CLKM_EXTERNAL_ASYNC && !external_clock_period.is_never())
		clock_mode = CLKM_EXTERNAL_RATE_ASYNC;
	if(clock_mode == CLKM_EXTERNAL_SYNC && !external_clock_period.is_never())
		clock_mode = CLKM_EXTERNAL_RATE_SYNC;

	if(V>=1) {
		char buf[4096];
		switch(clock_mode) {
		case CLKM_INTERNAL_ASYNC:
			sprintf(buf, "clock internal at %d Hz, async, bitrate %d bps\n", int(cpu->clock() / divider), int(cpu->clock() / (divider*16)));
			break;
		case CLKM_INTERNAL_ASYNC_OUT:
			sprintf(buf, "clock internal at %d Hz, async, bitrate %d bps, output\n", int(cpu->clock() / divider), int(cpu->clock() / (divider*16)));
			break;

		case CLKM_EXTERNAL_ASYNC:
			sprintf(buf, "clock external, async\n");
			break;
		case CLKM_EXTERNAL_RATE_ASYNC:
			sprintf(buf, "clock external at %d Hz, async, bitrate %d bps\n", int(cpu->clock()*internal_to_external_ratio), int(cpu->clock()*internal_to_external_ratio/16));
			break;

		case CLKM_INTERNAL_SYNC_OUT:
			sprintf(buf, "clock internal at %d Hz, sync, output\n", int(cpu->clock() / (divider*2)));
			break;

		case CLKM_EXTERNAL_SYNC:
			sprintf(buf, "clock external, sync\n");
			break;
		case CLKM_EXTERNAL_RATE_SYNC:
			sprintf(buf, "clock external at %d Hz, sync\n", int(cpu->clock()*internal_to_external_ratio));
			break;
		}
		if(buf != last_clock_message) {
			last_clock_message = buf;
			logerror("%s", buf);
		}
	}
}

void h8_sci_device::device_start()
{
	tx_cb.resolve_safe();
	clk_cb.resolve_safe();

	sync_timer = timer_alloc(0);

	if(external_clock_period.is_never()) {
		internal_to_external_ratio = 0;
		external_to_internal_ratio = 0;
	} else {
		external_to_internal_ratio = (external_clock_period*cpu->clock()).as_double();
		internal_to_external_ratio = 1/external_to_internal_ratio;
	}

	intc = siblingdevice<h8_intc_device>(intc_tag);
	save_item(NAME(rdr));
	save_item(NAME(tdr));
	save_item(NAME(smr));
	save_item(NAME(scr));
	save_item(NAME(ssr));
	save_item(NAME(brr));
	save_item(NAME(rsr));
	save_item(NAME(tsr));
	save_item(NAME(rx_bit));
	save_item(NAME(tx_bit));
	save_item(NAME(rx_state));
	save_item(NAME(tx_state));
	save_item(NAME(tx_parity));
	save_item(NAME(clock_state));
	save_item(NAME(clock_value));
	save_item(NAME(clock_base));
	save_item(NAME(divider));
	save_item(NAME(ext_clock_value));
	save_item(NAME(ext_clock_counter));
	save_item(NAME(cur_sync_time));
}

void h8_sci_device::device_reset()
{
	rdr = 0x00;
	tdr = 0xff;
	smr = 0x00;
	scr = 0x00;
	ssr = 0x84;
	brr = 0xff;
	rsr = 0x00;
	tsr = 0xff;
	rx_bit = 0;
	tx_bit = 0;
	tx_state = ST_IDLE;
	rx_state = ST_IDLE;
	clock_state = 0;
	clock_mode = CLKM_INTERNAL_ASYNC;
	clock_base = 0;
	clock_update();
	clock_value = true;
	ext_clock_value = true;
	ext_clock_counter = 0;
	rx_value = true;
	clk_cb(clock_value);
	tx_cb(1);
	cur_sync_time = attotime::never;
}

void h8_sci_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	// Used only to force system-wide syncs
}

WRITE_LINE_MEMBER(h8_sci_device::rx_w)
{
	rx_value = state;
	if(V>=2) logerror("rx=%d\n", state);
	if(!rx_value && !(clock_state & CLK_RX) && rx_state != ST_IDLE)
		clock_start(CLK_RX);
}

WRITE_LINE_MEMBER(h8_sci_device::clk_w)
{
	if(ext_clock_value != state) {
		ext_clock_value = state;
		if(clock_state) {
			switch(clock_mode) {
			case CLKM_EXTERNAL_ASYNC:
				if(ext_clock_value) {
					ext_clock_counter = (ext_clock_counter+1) & 15;

					if((clock_state & CLK_TX) && ext_clock_counter == 0)
						tx_dropped_edge();
					if((clock_state & CLK_RX) && ext_clock_counter == 8)
						rx_raised_edge();
				}
				break;

			case CLKM_EXTERNAL_SYNC:
				if((!ext_clock_value) && (clock_state & CLK_TX))
					tx_dropped_edge();

				else if(ext_clock_value && (clock_state & CLK_RX))
					rx_raised_edge();
				break;
			}
		}
	}
}

uint64_t h8_sci_device::internal_update(uint64_t current_time)
{
	uint64_t event = 0;
	switch(clock_mode) {
	case CLKM_INTERNAL_SYNC_OUT:
		if(clock_state || !clock_value) {
			uint64_t fp = divider*2;
			if(current_time >= clock_base) {
				uint64_t delta = current_time - clock_base;
				if(delta >= fp) {
					delta -= fp;
					clock_base += fp;
				}
				assert(delta < fp);

				bool new_clock = delta >= divider;
				if(new_clock != clock_value) {
					cpu->synchronize();
					if((!new_clock) && (clock_state & CLK_TX))
						tx_dropped_edge();

					else if(new_clock && (clock_state & CLK_RX))
						rx_raised_edge();

					clock_value = new_clock;
					if(clock_state || clock_value)
						clk_cb(clock_value);
				}
			}
			event = clock_base + (clock_value ? fp : divider);
		}
		break;

	case CLKM_INTERNAL_ASYNC:
	case CLKM_INTERNAL_ASYNC_OUT:
		if(clock_state || !clock_value) {
			uint64_t fp = divider*16;
			if(current_time >= clock_base) {
				uint64_t delta = current_time - clock_base;
				if(delta >= fp) {
					delta -= fp;
					clock_base += fp;
				}
				assert(delta < fp);
				bool new_clock = delta >= divider*8;
				if(new_clock != clock_value) {
					cpu->synchronize();
					if((!new_clock) && (clock_state & CLK_TX))
						tx_dropped_edge();

					else if(new_clock && (clock_state & CLK_RX))
						rx_raised_edge();

					clock_value = new_clock;
					if(clock_mode == CLKM_INTERNAL_ASYNC_OUT && (clock_state || !clock_value))
						clk_cb(clock_value);
				}
			}

			event = clock_base + (clock_value ? fp : divider*8);
		}
		break;

	case CLKM_EXTERNAL_RATE_SYNC:
		if(clock_state || !clock_value) {
			uint64_t ctime = uint64_t(current_time*internal_to_external_ratio*2);
			if(ctime >= clock_base) {
				uint64_t delta = ctime - clock_base;
				clock_base += delta & ~1;
				delta &= 1;
				bool new_clock = delta >= 1;
				if(new_clock != clock_value) {
					cpu->synchronize();
					if((!new_clock) && (clock_state & CLK_TX))
						tx_dropped_edge();

					else if(new_clock && (clock_state & CLK_RX))
						rx_raised_edge();

					clock_value = new_clock;
				}
			}

			event = uint64_t((clock_base + (clock_value ? 2 : 1))*external_to_internal_ratio)+1;
		}
		break;

	case CLKM_EXTERNAL_RATE_ASYNC:
		if(clock_state || !clock_value) {
			uint64_t ctime = uint64_t(current_time*internal_to_external_ratio);
			if(ctime >= clock_base) {
				uint64_t delta = ctime - clock_base;
				clock_base += delta & ~15;
				delta &= 15;
				bool new_clock = delta >= 8;
				if(new_clock != clock_value) {
					cpu->synchronize();
					if((!new_clock) && (clock_state & CLK_TX))
						tx_dropped_edge();

					else if(new_clock && (clock_state & CLK_RX))
						rx_raised_edge();

					clock_value = new_clock;
				}
			}

			event = uint64_t((clock_base + (clock_value ? 16 : 8))*external_to_internal_ratio)+1;
		}
		break;

	case CLKM_EXTERNAL_ASYNC:
	case CLKM_EXTERNAL_SYNC:
		break;;
	}
	if(event) {
		attotime ctime = machine().time();
		attotime sync_time = attotime::from_ticks(event-10, cpu->clock());
		if(cur_sync_time != sync_time && sync_time > ctime) {
			sync_timer->adjust(sync_time - ctime);
			cur_sync_time = sync_time;
		}
	}

	return event;
}

void h8_sci_device::clock_start(int mode)
{
	// Happens when back-to-back
	if(clock_state & mode)
		return;

	if(!clock_state) {
		cpu->synchronize();
		clock_state = mode;
		switch(clock_mode) {
		case CLKM_INTERNAL_ASYNC:
		case CLKM_INTERNAL_ASYNC_OUT:
		case CLKM_INTERNAL_SYNC_OUT:
			if(V>=2) logerror("Starting internal clock\n");
			clock_base = cpu->total_cycles();
			cpu->internal_update();
			break;

		case CLKM_EXTERNAL_RATE_ASYNC:
			if(V>=2) logerror("Simulating external clock async\n");
			clock_base = uint64_t(cpu->total_cycles()*internal_to_external_ratio);
			cpu->internal_update();
			break;

		case CLKM_EXTERNAL_RATE_SYNC:
			if(V>=2) logerror("Simulating external clock sync\n");
			clock_base = uint64_t(cpu->total_cycles()*2*internal_to_external_ratio);
			cpu->internal_update();
			break;

		case CLKM_EXTERNAL_ASYNC:
			if(V>=2) logerror("Waiting for external clock async\n");
			ext_clock_counter = 15;
			break;

		case CLKM_EXTERNAL_SYNC:
			if(V>=2) logerror("Waiting for external clock sync\n");
			break;
		}
	} else
		clock_state |= mode;
}

void h8_sci_device::clock_stop(int mode)
{
	clock_state &= ~mode;
	cpu->internal_update();
}

void h8_sci_device::tx_start()
{
	ssr |= SSR_TDRE;
	tsr = tdr;
	tx_parity = smr & SMR_OE ? 0 : 1;
	if(V>=1) logerror("start transmit %02x '%c'\n", tsr, tsr >= 32 && tsr < 127 ? tsr : '.');
	if(scr & SCR_TIE)
		intc->internal_interrupt(txi_int);
	if(smr & SMR_CA) {
		tx_state = ST_BIT;
		tx_bit = 8;
	} else {
		tx_state = ST_START;
		tx_bit = 1;
	}
	clock_start(CLK_TX);
	if(rx_state == ST_IDLE && !has_recv_error() && is_sync_start())
		rx_start();
}

void h8_sci_device::tx_dropped_edge()
{
	if(V>=2) logerror("tx_dropped_edge state=%s bit=%d\n", state_names[tx_state], tx_bit);
	switch(tx_state) {
	case ST_START:
		tx_cb(false);
		assert(tx_bit == 1);
		tx_state = ST_BIT;
		tx_bit = smr & SMR_CHR ? 7 : 8;
		break;

	case ST_BIT:
		tx_parity ^= (tsr & 1);
		tx_cb(tsr & 1);
		tsr >>= 1;
		tx_bit--;
		if(!tx_bit) {
			if(smr & SMR_CA) {
				if(!(ssr & SSR_TDRE))
					tx_start();
				else {
					tx_state = ST_LAST_TICK;
					tx_bit = 0;
				}
			} else if(smr & SMR_PE) {
				tx_state = ST_PARITY;
				tx_bit = 1;
			} else {
				tx_state = ST_STOP;
				tx_bit = smr & SMR_STOP ? 2 : 1;
			}
		}
		break;

	case ST_PARITY:
		tx_cb(tx_parity);
		assert(tx_bit == 1);
		tx_state = ST_STOP;
		tx_bit = smr & SMR_STOP ? 2 : 1;
		break;

	case ST_STOP:
		tx_cb(true);
		tx_bit--;
		if(!tx_bit) {
			if(!(ssr & SSR_TDRE))
					tx_start();
			else {
				tx_state = ST_LAST_TICK;
				tx_bit = 0;
			}
		}
		break;

	case ST_LAST_TICK:
		tx_state = ST_IDLE;
		tx_bit = 0;
		clock_stop(CLK_TX);
		tx_cb(1);
		ssr |= SSR_TEND|SSR_TDRE;
		if(scr & SCR_TEIE)
			intc->internal_interrupt(tei_int);
		break;

	default:
		abort();
	}
	if(V>=2) logerror("            -> state=%s bit=%d\n", state_names[tx_state], tx_bit);
}

void h8_sci_device::rx_start()
{
	ssr |= SSR_TDRE;
	rx_parity = smr & SMR_OE ? 0 : 1;
	rsr = 0x00;
	if(V>=2) logerror("start receive\n");
	if(smr & SMR_CA) {
		rx_state = ST_BIT;
		rx_bit = 8;
		clock_start(CLK_RX);
	} else {
		rx_state = ST_START;
		rx_bit = 1;
		if(!rx_value)
			clock_start(CLK_RX);
	}
}

void h8_sci_device::rx_done()
{
	if(!(ssr & SSR_FER)) {
		if((smr & SMR_PE) && rx_parity) {
			ssr |= SSR_PER;
			if(V>=1) logerror("Receive parity error\n");
		} else if(ssr & SSR_RDRF) {
			ssr |= SSR_ORER;
			if(V>=1) logerror("Receive overrun\n");
		} else {
			ssr |= SSR_RDRF;
			if(V>=1) logerror("Received %02x '%c'\n", rsr, rsr >= 32 && rsr < 127 ? rsr : '.');
			rdr = rsr;
		}
	}
	if(scr & SCR_RIE) {
		if(has_recv_error())
			intc->internal_interrupt(eri_int);
		else
			intc->internal_interrupt(rxi_int);
	}
	if((scr & SCR_RE) && !has_recv_error() && !is_sync_start())
		rx_start();
	else {
		clock_stop(CLK_RX);
		rx_state = ST_IDLE;
	}
}

void h8_sci_device::rx_raised_edge()
{
	if(V>=2) logerror("rx_raised_edge state=%s bit=%d\n", state_names[rx_state], rx_bit);
	switch(rx_state) {
	case ST_START:
		if(rx_value) {
			clock_stop(CLK_RX);
			break;
		}
		rx_state = ST_BIT;
		rx_bit = smr & SMR_CHR ? 7 : 8;
		break;

	case ST_BIT:
		rx_parity ^= rx_value;
		rsr >>= 1;
		if(rx_value) {
			rx_parity = !rx_parity;
			rsr |= (smr & (SMR_CA|SMR_CHR)) == SMR_CHR ? 0x40 : 0x80;
		}
		rx_bit--;
		if(!rx_bit) {
			if(smr & SMR_CA)
				rx_done();
			else if(smr & SMR_PE) {
				rx_state = ST_PARITY;
				rx_bit = 1;
			} else {
				rx_state = ST_STOP;
				rx_bit = 1; // Always 1 on rx
			}
		}
		break;

	case ST_PARITY:
		rx_parity ^= rx_value;
		assert(rx_bit == 1);
		rx_state = ST_STOP;
		rx_bit = 1;
		break;

	case ST_STOP:
		assert(rx_bit == 1);
		if(!rx_value)
			ssr |= SSR_FER;
		else if((smr & SMR_PE) && rx_parity)
			ssr |= SSR_PER;
		rx_done();
		break;

	default:
		abort();
	}
	if(V>=2) logerror("            -> state=%s, bit=%d\n", state_names[rx_state], rx_bit);
}
